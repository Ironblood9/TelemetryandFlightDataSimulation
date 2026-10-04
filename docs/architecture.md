# Architecture

Design notes and rationale for the Telemetry & Flight Data Simulation project.
The byte-level contract between the two components lives in
[`protocol-spec.md`](protocol-spec.md); this document covers everything else.

---

## 1. System context

The system is a closed loop that never touches real hardware:

```
                    ┌──────────────────────────────────────────┐
                    │            flight dynamics model         │
                    │  state machine + 3-DOF force integration │
                    └───────────────────┬──────────────────────┘
                                        │ ground truth state
                    ┌───────────────────▼──────────────────────┐
                    │              sensor suite                │
                    │  air data computer · engine monitor      │
                    │  (models noise, bias, latency, validity)  │
                    └───────────────────┬──────────────────────┘
                                        │ sensor samples
                    ┌───────────────────▼──────────────────────┐
                    │  frame encoder + CRC32 + sequence number │
                    └───────────────────┬──────────────────────┘
                                        │ UDP datagrams
                    ┌───────────────────▼──────────────────────┐
                    │         ground station (C# / WPF)        │
                    │  decode → plot → gauge → alarm           │
                    └──────────────────────────────────────────┘
```

Modelling the *sensors* separately from the *physics* is what makes this worth
building: it is where bias, quantisation, latency and validity flags enter the
system, and therefore where most real telemetry defects come from.

---

## 2. Component layering

The engine is layered strictly top-down. Nothing below a layer may depend on
anything above it.

| Layer | Responsibility | Planned location |
|---|---|---|
| **CLI / host** | Argument parsing, lifecycle, headless operation | `src/main.cpp` |
| **Real-time** | Task scheduling, priority, watchdog, jitter accounting | `src/rtos/`, `include/fse/rtos/` |
| **Transport** | Framing, UDP send/receive, TCP control link, statistics | `src/transport/` |
| **Sensors** | Sample production, noise, bias, validity, health | `src/sensor/` |
| **Physics** | Flight phase state machine, force integration | `src/flight_model/` |
| **Platform** | Clocks, monotonic timestamps, byte order | `src/platform/` |

Two abstractions exist purely to keep the boundaries honest:

* `IFlightModel` — the physics layer produces a state vector and nothing else.
* `ITaskScheduler` — the real-time layer schedules work and knows nothing about
  telemetry. This is also the seam where a FreeRTOS backend can be added later
  without touching the layers above.

---

## 3. Threading and scheduling model

*(Designed here, implemented in M4.)*

The intended topology — one thread per priority class, joined by lock-free
queues:

```
  ┌──────────────┐   deadline 1 ms    ┌───────────────────────┐
  │ Telemetry    │ ─────────────────▶ │ SpscRingBuffer<Frame> │
  │ producer     │   Critical prio    │  (lock-free, 4096)    │
  └──────────────┘                    └───────────┬───────────┘
                                                   │
  ┌──────────────┐   deadline 100 ms               ▼
  │ Logger       │ ─────────────────▶   ┌───────────────────────┐
  │ (background) │   Low prio          │ Transport task         │
  └──────────────┘                      │ priority inheritance   │
                                        │ mutex around the socket│
                                        └───────────┬───────────┘
                                                    ▼
                                        ┌───────────────────────┐
                                        │ Watchdog / monitor    │
                                        └───────────────────────┘
```

Rules that the real-time layer will enforce (they are collected as their own
document in M4, `realtime-rules.md`):

1. **No dynamic allocation** after initialisation. All buffers are sized once.
2. **No blocking calls** in a real-time task. Socket writes are non-blocking.
3. **Single-writer principle** everywhere it is possible; it is what removes
   the need for locks in the first place.
4. **Never hold a lock across a deadline boundary.** The priority-inheriting
   mutex exists solely for the socket handle, and the critical section is a
   single `sendto`.

The rationale for (3) is mechanical, not stylistic: in a single-producer /
single-consumer ring buffer only the producer writes the head index and only the
consumer writes the tail index, so no compare-and-swap is required at all. A
naive "just use a mutex" queue would introduce two context switches per frame
into the one data path that must not have them.

---

## 4. Build system

### Why CMake and not MSBuild

The `.vcxproj` was removed in M0. Rationale:

* A `.vcxproj` hard-codes a toolset, a Windows SDK and an IDE. It cannot be
  consumed by a Linux CI runner or a container image.
* With CMake, *presets* select the generator, so the committed definition is
  portable while each environment still gets its natural toolchain.
* The cost is real: the C++ side loses Solution Explorer, and every contributor
  needs CMake 3.28+ instead of a Visual Studio install.

The WPF project stays in `.slnx` because the XAML designer and hot reload are
genuinely useful and are not available in the CMake folder view. The two build
worlds do not overlap, which is why this split is clean rather than awkward.

### Warning policy

`cmake/ProjectOptions.cmake` defines one interface target, `fse::compile_options`,
that every C++ target links against. It carries:

* `/W4 /permissive- /Zc:__cplusplus /Zc:preprocessor /utf-8` on MSVC,
  `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
  -Wdouble-promotion -Wold-style-cast` elsewhere;
* per-configuration optimisation through generator expressions, so a single
  build tree can hold Debug and Release simultaneously;
* optional `/WX` / `-Werror` (`FSE_WARNINGS_AS_ERRORS`);
* optional ASan + UBSan (`FSE_ENABLE_SANITIZERS`).

`-Wconversion` is enabled deliberately. Narrowing conversions are the single
most common source of silent telemetry corruption — millimetres versus metres,
degrees versus radians, `int16` versus `float32` — and the noise is a one-off
cost paid while the codebase is small.

### Why Ninja is not the local default

The bundled Ninja generator requires a Developer Command Prompt: `cl.exe` is not
resolvable from a normal shell, and CMake fails with *"No CMAKE_CXX_COMPILER
could be found"*. The `Visual Studio 18 2026` generator locates the toolchain
itself, so `cmake --preset x64-debug` works from any terminal. Ninja is used in
CI, where the MSVC environment is sourced explicitly.

---

## 5. Environment constraints

Recorded here because they cost real debugging time and will bite again.

### Non-ASCII user names break runtime file output in native binaries

The development machine's user name contains `ı` (U+0131, Latin small letter
dotless i). It is **not** representable in code page 1252.

The MSVC C runtime narrows `wargv` to `argv` when `main(int, char**)` is used.
The dotless `ı` is replaced by `?`, and any wide-argv-derived path therefore
becomes an invalid path for the narrow Win32 file API:

```
[FATAL] ... Unable to open file "C:\Users\Ms?\...\probe2.json"
```

Verified behaviour: a path under `C:\Windows\Temp` (pure ASCII) works; the same
filename under the user profile does not.

**Mitigations already in place**

* `gtest_add_tests()` is used instead of `gtest_discover_tests()`. The latter
  runs the test binary at build time with `--gtest_output=json:<absolute path>`
  and aborted for exactly this reason. `gtest_add_tests()` scans the sources at
  configure time and performs no runtime file I/O.
* Native tools in later milestones must write to **relative** paths. A relative
  path contains no user-profile component, so it survives the narrowing step.
* .NET is unaffected — it is Unicode end to end.

This is an environment property, not a defect in the project, and CI is not
affected (GitHub Actions paths are ASCII). It is documented rather than worked
around so the next reader does not rediscover it.

### Docker daemon

The Docker CLI and Compose v5 are installed, but the daemon is not running by
default. Milestone M7 needs it started.

---

## 6. Testing strategy

| Layer | Tool | What it protects |
|---|---|---|
| Physics | GoogleTest | Forces, state transitions, energy sanity |
| Determinism | GoogleTest | Same seed ⇒ bit-identical output, cross-platform |
| Encoding | GoogleTest + golden vectors | Byte-exact frame layout |
| Scheduling | GoogleTest | Lock-free queue invariants under concurrency |
| Decoding | xUnit + the same golden vectors | C++/C# agreement on the wire |
| Transport | GoogleTest soak | Frame count, CRC errors, latency percentiles |
| Build contract | GoogleTest | The invariants later code silently relies on |

The M0 `test_build_contract.cpp` suite is unusual in deliberately checking things
that are usually assumed: that C++20 is genuinely enabled, that engine headers
are reachable under the `fse` prefix, and that version constants still match this
repository's documentation.

---

## 7. Security posture

The transport is **not** authenticated or encrypted in this project. That is a
deliberate scope decision, not an oversight, and it is worth stating precisely
because an unencrypted ground link is the first question any reviewer asks:

* The simulation runs on loopback or an isolated lab network.
* The threat model that *would* apply — a crewed or payload-bearing downlink —
  requires DTLS for the datagram channel and mutual TLS for the control
  channel, plus a key provisioning story.
* Nothing in the architecture prevents adding them: the transport is already
  behind `ITelemetryTransport` and the control channel is a single `sendto` /
  `recv` pair. Encrypting there is an additive change.

`docs/` gains a full threat model in M8.