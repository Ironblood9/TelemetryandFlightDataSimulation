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

| Layer | Responsibility | Location | M |
|---|---|---|---|
| **CLI / host** | Argument parsing, lifecycle, headless operation | `src/main.cpp` | **M1** |
| **Real-time** | Task scheduling, priority, watchdog, jitter accounting | `src/rtos/` | M4 |
| **Transport** | Framing, UDP send/receive, TCP control link, statistics | `src/transport/` | M2–M3 |
| **Sensors** | Sample production, noise, bias, validity, health | `src/sensor/` | **M1** |
| **Guidance** | Phase setpoints, altitude/speed/heading loops | `src/autopilot.cpp` | **M1** |
| **Physics** | Flight phase state machine, force integration | `src/flight_model/` | **M1** |
| **Platform** | Clocks, monotonic timestamps, byte order | `src/platform/` | M2 |

Two abstractions exist purely to keep the boundaries honest:

* `IFlightModel` — the guidance, phase machine and sensors all talk to the
  dynamics through this, never to a concrete model. It is the seam where a
  higher-fidelity aerodynamic solution could be substituted.
* `ITaskScheduler` — *(M4)* schedules work and knows nothing about telemetry.
  This is where a FreeRTOS backend can be added later without touching anything
  above it.

---

## 3. Simulation step order

The order of operations per step is the contract, and getting it wrong is the
classic cause of a phase transition that appears to happen *after* the manoeuvre
it describes:

```
1. advance the phase machine on the *previous* state
2. engage the guidance for the new phase, if it changed
3. compute control commands from the new phase
4. step the flight model
5. run the sensors over the new truth and emit a sample
```

The phase therefore always leads the physics by exactly one step, and every
emitted frame describes the state that was just computed rather than the one the
commands were derived from.

### Integration: one force model, ground as a constraint

The runway and the air share **one** integrator. Lift, drag, thrust and gravity
are always integrated, and the landing gear is applied afterwards as
non-penetration plus rolling friction.

The alternative — branching between a ground handler and a flight integrator —
was implemented first and did not work. At rotation speed the wing's lift is
within a few percent of the weight, so the branch flipped every few steps and the
aircraft hopped down the runway. Making lift-off an emergent consequence of the
net vertical force turning positive removes the decision entirely.

Two further details the constraint has to get right:

* **Thrust is integrated as a force only.** Adding a longitudinal throttle term
  in the ground handler as well double counts it.
* **The accelerometer reads 1 g on the ground.** The wheels carry whatever the
  wing does not. Leaving the aerodynamic load factor there would report 0 g while
  parked, which no real instrument does and which every consumer would have to
  special case.

### Guidance: altitude hold as inverse dynamics

The altitude loop is a cascade — altitude error → vertical speed → flight path
angle → pitch — with a **feed-forward** angle of attack:

```
α_ff = (CL_required − CL0 − CL_flap) / CL_α ,  CL_required = W / (q · S)
pitch_command = path_angle + α_ff
```

`α_ff` is the angle of attack at which the wing carries exactly the current
weight at the current speed and density. A fixed trim angle instead — 3° was the
first attempt — commands more than twice the lift a cruise attitude requires, so
the climb never settles and the descent never begins.

## 4. Threading and scheduling model

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

## 5. Build system

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

## 6. Environment constraints

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

### Wind is off unless a scenario asks for it

`AerodynamicModel` carries no wind by default. A wind model that is on unless it
is switched off leaks into every test that constructs a bare model — and "the
aircraft is parked but the pitot reads 6 m/s" is exactly the sort of surprise
that costs an afternoon. `SimulationConfig::wind_enabled` installs one
explicitly.

The related trap is the taxi guard. A parked aircraft in a 6 m/s tailwind has a
non-zero *indicated* airspeed while standing still, so a guard written against
`ias` clears it for departure before it has moved. The guard uses ground speed,
and `AircraftState` carries both.

---

## 7. Testing strategy

| Layer | Tool | What it protects |
|---|---|---|
| Physics | GoogleTest | Forces, state transitions, energy sanity, ISA atmosphere |
| Determinism | GoogleTest | Same seed ⇒ bit-identical output, exact integer timestamps |
| PRNG | GoogleTest | Upstream PCG32 reference vectors, normal moments |
| Phase machine | GoogleTest | Table integrity, every guard in isolation, no illegal transitions |
| Sensors | GoogleTest | Lag, noise budget, validity flags, alignment |
| Encoding | GoogleTest + golden vectors | Byte-exact frame layout *(M2)* |
| Scheduling | GoogleTest | Lock-free queue invariants under concurrency *(M4)* |
| Decoding | xUnit + the same golden vectors | C++/C# agreement on the wire *(M5)* |
| Transport | GoogleTest soak | Frame count, CRC errors, latency percentiles *(M3)* |
| Build contract | GoogleTest | The invariants later code silently relies on |

173 tests at the end of M1, one file per module rather than one file per
milestone: `test_types`, `test_noise`, `test_phase_enum`, `test_atmosphere`,
`test_dynamics`, `test_phase_table`, `test_phase_machine`, `test_autopilot`,
`test_sensors`, `test_simulation`, `test_determinism`, plus the M0
`test_build_contract`. A failure names the module it came from, and a reader
looking for "how is the guidance tested" finds one file instead of grepping.

The suites are deliberately picky about the things a demo would hide:

* the atmosphere is pinned against the **ISA reference table** at 5 000 m, which
  is the only reason the density-law exponent is right — the missing `-1` in
  `g/(R·L) − 1` produced a 13% error that no eyeball test would catch;
* the PRNG is checked against the **upstream reference vectors**, not against
  whatever this implementation happens to produce, so the test proves it is PCG32
  rather than merely unchanging;
* a level-flight test **searches for the trim point** over both pitch and
  throttle, because solving only one leaves the other wherever it landed and the
  test would be asserting on an accident;
* the parameter set is checked for **internal consistency**: stall speeds,
  approach speeds, rotation speed and thrust-to-weight have to agree with each
  other, or the flight profile looks plausible in a plot and cannot be flown.

The M0 `test_build_contract.cpp` suite is unusual in deliberately checking things
that are usually assumed: that C++20 is genuinely enabled, that engine headers
are reachable under the `fse` prefix, and that version constants still match this
repository's documentation.

The composition tests are where the defects were. Every module was individually
correct and three real faults only appeared when they were flown together: a
post-landing rollout that re-took off because the taxi guard fired while the
aircraft was still rolling, a climb that lost all its power for the second or two
it spent on the runway after rotation, and a heading loop that was unreachable
code, leaving the cruise leg open. No unit test can see any of those, which is
the argument for having this layer at all.

### What determinism does and does not mean here

IEEE-754 pins `+ − × ÷` and `sqrt` exactly, so those are bit-identical on any
conforming platform. `sin`, `cos`, `log`, `exp` and `pow` are *implementation
defined* and may differ by one unit in the last place between libm versions.

The tests therefore assert three separate things:

1. repeated runs of the same binary are bit-identical (guaranteed);
2. the integer PRNG stream is reproducible on any platform, so cross-platform
   regression baselines are possible;
3. nothing that must be reproducible is ever NaN, denormal or out of bounds.

Cross-compiler bit-exactness of the *float* trajectory is deliberately **not**
asserted, because the claim would be false. Saying so precisely is worth more than
a test that cannot be written.

---

## 8. Security posture

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