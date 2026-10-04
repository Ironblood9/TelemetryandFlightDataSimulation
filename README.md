# Telemetry & Flight Data Simulation

**A real-time flight dynamics engine and ground station, built to demonstrate the
software side of avionics: deterministic simulation, a versioned binary wire
protocol, lock-free real-time tasking, and a desktop telemetry display.**

> **Status: milestone M0 of 9.** The build system, toolchain contract and CI
> skeleton are in place. The simulation model, transport, real-time layer and
> ground station are designed and scheduled but not yet implemented. See
> [Roadmap](#roadmap) for exactly what exists today.

---

## Why this project

Flight data is a real-time stream with hard constraints: it is produced at a
fixed rate, it must reach the ground within a bounded latency, and a *stale*
sample is worse than a missing one. That single property shapes every design
decision in this repository:

| Concern | Consequence |
|---|---|
| A fixed production rate | Deterministic, fixed-step simulation — never wall-clock driven |
| Bounded latency | Lock-free single-producer/single-consumer queues; a priority-inheriting mutex where mutual exclusion is genuinely unavoidable |
| Stale data is worthless | Dropping the oldest frame under load is the *correct* backpressure, not blocking the producer |
| Two implementations, one wire | A versioned frame layout validated in both C++ and C# against byte-exact golden vectors |

The ground station is written in C# / WPF and the engine in modern C++ — the
split is deliberate, because the interesting engineering problem in that
combination is *agreeing on bytes*.

---

## Architecture

```
   ┌────────────────────────────────────────┐         UDP :5700 @ 100 Hz
   │  flight-sim  (C++20, CMake)            │  ─── TelemetryFrame ─────────────▶  ┌──────────────────────────────────┐
   │                                        │      (seq + CRC32, 84 bytes)      │  ground-station (C# / WPF)        │
   │  FlightDynamicsModel                   │                                     │                                  │
   │   └ SensorSuite                        │         TCP :5701 (control)        │  TelemetryReceiver                │
   │       ├ AirDataComputer                │  ◀── SET_PHASE / INJECT_FAULT ────  │  FrameDispatcher (Channel<T>)    │
   │       └ EngineMonitor (thermal lag)    │                                     │  ScottPlot live plots             │
   │   └ FrameEncoder                       │                                     │  AirSpeedGauge, ArtificialHorizon │
   │   └ TaskScheduler (ITaskScheduler)     │                                     │  Link quality + alarm panels      │
   │       └ SpscRingBuffer<T>              │                                     │                                  │
   └────────────────────────────────────────┘                                     └──────────────────────────────────┘
            ▲ M1–M3                                                          ▲ M4
            └─────────────────── Phase 1–3: engine ──────────────────────────┴──────── Phase 4: UI
```

Detailed design notes: [`docs/architecture.md`](docs/architecture.md).
Byte-level wire contract: [`docs/protocol-spec.md`](docs/protocol-spec.md).

---

## Repository layout

```
.
├── CMakeLists.txt                  # Top-level build, one source of truth
├── CMakePresets.json               # Configure / build / test presets
├── cmake/
│   └── ProjectOptions.cmake        # Warning, hardening and codegen flags
├── FlightSimulationEngine/         # C++20 · the simulation engine
│   ├── include/fse/                #   Public headers, namespace `fse`
│   ├── src/                        #   Implementation
│   └── tests/                      #   GoogleTest suites
├── GroundStationUI/                # .NET 10 · the WPF ground station
├── docs/                           # Architecture, protocol spec, design notes
├── protocol/                       # Wire format + byte-exact golden vectors (M2)
├── tools/                          # Golden-vector generator, inspectors (M2+)
└── .github/workflows/              # CI
```

---

## Building

CMake **is** the build system for the C++ engine — there is no `.vcxproj`, so the
same commands work on Windows, Linux and macOS and in CI.

### Prerequisites

* A C++20 compiler — MSVC 19.3x+ (Visual Studio 2022/2026), GCC 11+, or Clang 14+
* CMake 3.28 or newer
* .NET 10 SDK (only for the ground station)

### Commands

```powershell
# Configure + build the engine and run the tests
cmake --preset x64-debug
cmake --build --preset x64-debug
ctest  --preset x64-debug
```

In Visual Studio, use **File → Open → Folder** on the repository root; CMake
Presets are picked up automatically and IntelliSense works as usual.

The WPF ground station lives in `TelemetryandFlightDataSimulation.slnx` and is
built normally with `dotnet build` — it is deliberately kept in a solution so
that XAML hot reload and the designer keep working.

### Build presets

| Preset | Purpose |
|---|---|
| `x64-debug` | Windows local development, `/W4 /WX`, multi-config build tree |
| `x64-release` | Windows local development, optimised |
| `x64-tidy` | Debug with `clang-tidy` wired into the build |
| `ci-gcc-debug` | Linux · GCC · **ASan + UBSan** |
| `ci-gcc-release` | Linux · GCC · optimised |
| `ci-clang-release` | Linux · Clang · optimised |
| `ci-msvc-release` | Windows · Ninja + `cl` (used by GitHub Actions) |

---

## Testing

```powershell
ctest --preset x64-debug --output-on-failure
```

C++: GoogleTest, pulled in through `FetchContent` — no submodule, no vendored
copy, no lockfile to maintain. Warnings are errors (`/WX`, `-Werror`) and the
non-MSVC compilers additionally build with `-Wconversion -Wsign-conversion`,
because silent unit-mismatch is the classic telemetry defect.

Ground station: xUnit, including decoder tests that run against the same golden
vectors as the C++ encoder, so the two implementations cannot drift apart.

---

## Design decisions & trade-offs

The section a reviewer actually reads.

**CMake instead of `.vcxproj`.** The generator is chosen by CMake *Presets*, so
the committed build definition is portable. A `.vcxproj` pins a toolset and an
IDE; that is fine for a single Windows developer and unusable for a Linux CI
runner or a container. Cost: we give up the Solution Explorer view of the C++
side (and the WPF side keeps it, because XAML tooling needs it).

**Warnings as errors, plus conversion warnings.** `-Wconversion` is
unpopular and produces noise, and that noise is precisely the class of bug this
project exists to avoid: millimetres versus metres, degrees versus radians,
`int16` versus `float32`. The cost is paid once, at the start.

**Fixed-step integration with an accumulator.** Variable-`dt` integration is
simpler and looks fine, but it makes the simulation non-reproducible and makes
jitter impossible to measure. Determinism is a property we intend to *test*, not
intend to hope for.

**Own PRNG rather than `<random>`.** `std::uniform_real_distribution` is not
specified to produce the same values across implementations or versions. A
simulation whose "same seed, same flight" claim cannot be trusted across
compilers is not a simulation you can regression-test.

**Serialise field by field; never `memcpy` a struct to the wire.** Struct
padding, alignment and host endianness make the naive version work on the
author's machine and corrupt data everywhere else. Explicit little-endian
encoding plus `static_assert(offsetof(...))` turns a silent data corruption bug
into a compile error.

**UDP for telemetry, TCP for commands.** UDP cannot be lossless, and a
retransmission mechanism for sensor data is the wrong trade: a late sample is
worthless. Instead every frame carries a sequence number and a CRC32, so the
receiver *detects* and *measures* loss rather than pretending it does not
happen, and commands — which must arrive — travel on a separate reliable
channel.

**C++ and C# share a golden-vector test, not a code generator.** Generating the
C# decoder from the C++ header would hide drift until runtime. A committed
byte-exact fixture, verified by tests on both sides, fails the build the moment
either implementation changes the contract.

---

## Roadmap

| Milestone | Focus | Deliverable | Status |
|---|---|---|---|
| **M0** | Toolchain & build system | CMake presets, warning policy, test + CI skeleton | **Done** |
| M1 | Low-level C++ & data generation | Flight dynamics model, sensor suite, deterministic 100 Hz loop | Planned |
| M2 | Wire protocol | Versioned frame layout, CRC32, golden vectors | Planned |
| M3 | Communications | UDP telemetry transport, TCP control link, gap statistics | Planned |
| M4 | Real-time system | Priority task scheduler, SPSC ring buffer, watchdog, jitter report | Planned |
| M5 | Ground station | WPF UI: live plots, gauges, link quality, alarm panel | Planned |
| M6 | Recording & replay | Binary recorder + playback, so the UI runs without the engine | Planned |
| M7 | Containerisation & CI/CD | Multi-stage images, compose, sanitizer matrix, image releases | Planned |
| M8 | Portfolio polish | Benchmarks, threat model, design notes | Planned |

---

## Documentation

| Document | Contents |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | Layering, threading model, build system, environment constraints |
| [`docs/protocol-spec.md`](docs/protocol-spec.md) | Byte-level frame layout, CRC, sequence semantics, versioning rules |
| [`docs/realtime-rules.md`](docs/realtime-rules.md) | *M4* — rules the real-time layer must obey |
| [`docs/interview-notes.md`](docs/interview-notes.md) | *M8* — anticipated review questions and the reasoning behind each answer |

---

## License

See [`LICENSE`](LICENSE).