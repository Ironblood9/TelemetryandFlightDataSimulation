# Telemetry Wire Protocol — version 1

**Status:** specification frozen for milestone M2. The C++ encoder and the C#
decoder are both validated against `protocol/golden_vectors.bin`.

This document is the contract. Where it and any code disagree, one of them is a
bug — and a test will say which.

---

## 1. Design rules

These rules are not negotiable and are the reason the format looks the way it
does.

1. **Little-endian, explicitly.** Every multi-byte field is encoded and decoded
   byte by byte. A structure is *never* `memcpy`-ed to or from the wire:
   compilers insert padding, alignment requirements differ between x86-64 and
   ARM64, and the byte order of the host is not guaranteed to match the network
   order. Every field offset is pinned with `static_assert(offsetof(...))` on the
   C++ side and with explicit `BitConverter`/`BinaryPrimitives` reads on the
   C# side.
2. **Fixed-width types only.** `u8`, `u16`, `u32`, `u64`, `f32`. Never `long`,
   `size_t`, `int`, `bool` or `enum` on the wire — their sizes are
   implementation-defined.
3. **No bit fields, no enums, no unions.** They have implementation-defined
   layout and would defeat rule 1.
4. **Floats are IEEE-754 binary32.** Anything requiring more range or precision
   (timestamps, accumulators) is an integer.
5. **Version every frame.** A receiver that does not understand `version` must
   discard the frame rather than guess.

---

## 2. Transport

| Channel | Transport | Port | Guarantee |
|---|---|---|---|
| Telemetry | UDP unicast | 5700 | Best effort. Loss is expected and *measured*. |
| Control | TCP | 5701 | Reliable, ordered. Commands and events. |

**Why telemetry is not lossless.** It cannot be, over UDP, and attempting to
make it so would be the wrong trade. A retransmitted sensor sample arrives after
the decision it was supposed to inform; a stale airspeed reading is worse than a
missing one because the receiver cannot tell the difference. So:

* the telemetry channel is deliberately best-effort and low-latency;
* every frame carries a **sequence number**, so the receiver detects loss
  exactly instead of silently drawing a straight line across the gap;
* every frame carries a **CRC32**, so corruption is distinguished from loss;
* anything that must arrive — commands, configuration, log events — travels on
  the reliable TCP channel.

This is the classic real-time split, and it is the answer to "UDP is lossy, so
how is your link reliable": *the reliable part is on the reliable channel, and
the unreliable part is measured rather than hidden.*

### Timing budget

At the default 100 Hz production rate, one frame is 10 ms of system time. The
design target is end-to-end (encode → UDP → decode) **p99 < 5 ms on loopback**,
which leaves five frames of slack before a consumer could notice. These numbers
are measured, not guessed: see the benchmark table in the README (populated in
M3/M4).

---

## 3. Telemetry frame

One `TelemetryFrame` per UDP datagram. Total size **84 bytes**, which is well
under the 1472-byte payload budget of a 1500-byte Ethernet MTU, so no
fragmentation occurs. The sender sets the DF bit to guarantee that.

| Offset | Size | Type | Field | Unit / notes |
|---:|---:|---|---|---|
| 0 | 2 | `u16` | `magic` | `0xA55A`. Framing sanity check. |
| 2 | 1 | `u8` | `version` | `1`. See §5. |
| 3 | 1 | `u8` | `aircraft_id` | Slot number of the simulated aircraft. |
| 4 | 4 | `u32` | `sequence` | Monotonic, wraps at 2³². Gap detection (§4). |
| 8 | 8 | `u64` | `timestamp_ns` | Monotonic clock, nanoseconds. Never wall clock. |
| 16 | 1 | `u8` | `phase` | `FlightPhase`, §3.1. |
| 17 | 2 | `u16` | `status_flags` | Bitfield, §3.2. |
| 20 | 4 | `f32` | `airspeed_mps` | Indicated airspeed relative to the airmass. |
| 24 | 4 | `f32` | `altitude_m` | Above mean sea level. |
| 28 | 4 | `f32` | `vertical_speed_mps` | Rate of climb; positive is up. |
| 32 | 4 | `f32` | `heading_deg` | `[0, 360)`, clockwise from true north. |
| 36 | 4 | `f32` | `pitch_deg` | `[-90, 90]`, positive nose-up. |
| 40 | 4 | `f32` | `roll_deg` | `[-180, 180)`, positive right-wing-down. |
| 44 | 4 | `f32` | `engine_temp[0]` | Left outer. |
| 48 | 4 | `f32` | `engine_temp[1]` | Left inner. |
| 52 | 4 | `f32` | `engine_temp[2]` | Right inner. |
| 56 | 4 | `f32` | `engine_temp[3]` | Right outer. |
| 60 | 4 | `f32` | `fuel_press_kpa` | Gauge pressure. |
| 64 | 4 | `f32` | `oil_press_kpa` | Gauge pressure. |
| 68 | 4 | `f32` | `accel_x_mps2` | Body axes, specific force. |
| 72 | 4 | `f32` | `accel_y_mps2` | |
| 76 | 4 | `f32` | `accel_z_mps2` | Includes gravity: 1 g at rest on the deck. |
| 80 | 4 | `u32` | `crc32` | Over bytes `[0, 80)`. §3.3. |

### 3.1 `FlightPhase` enumeration

| Value | Name | Meaning |
|---:|---|---|
| 0 | `Preflight` | On the ground, systems coming up. |
| 1 | `Taxi` | Ground movement under low thrust. |
| 2 | `TakeoffRoll` | Main gear down, accelerating. |
| 3 | `Climb` | Positive rate of climb. |
| 4 | `Cruise` | Altitude hold within a tolerance band. |
| 5 | `Descent` | Negative rate of climb. |
| 6 | `Approach` | Decelerating, configured for landing. |
| 7 | `Landing` | Final approach and touchdown. |
| 8 | `GoAround` | Aborted landing, climbing again. |
| 9 | `Failure` | Fault latched; `status_flags` carries the reason. |

Unknown values are not an error — the receiver keeps the last known phase and
raises a protocol-version warning. Forward compatibility beats strictness here.

### 3.2 `status_flags` bitfield

| Mask | Name | Meaning |
|---:|---|---|
| `0x0001` | `EngineRunning` | At least one engine producing thrust. |
| `0x0002` | `EngineDegraded` | Running, but outside the nominal envelope. |
| `0x0004` | `SensorInvalid` | One or more samples failed their validity check. |
| `0x0008` | `LinkDegraded` | Receiver observed gaps or CRC errors recently. |
| `0x0010` | `SimulatedFault` | A fault was injected by the operator. |
| `0x0020` | `ControlLinkDown` | No control-channel traffic. |
| `0x0040` | `RedundancySwitched` | The active sensor source is the backup. |
| `0x8000` | `Reserved` | Must be zero in v1. |

### 3.3 CRC-32

CRC-32/ISO-HDLC (the "IEEE" / zlib CRC-32):

| Parameter | Value |
|---|---|
| Width | 32 |
| Polynomial | `0x04C11DB7` |
| Initial value | `0xFFFFFFFF` |
| Reflect in / out | true / true |
| Final XOR | `0xFFFFFFFF` |
| Check value (`"123456789"`) | `0xCBF43926` |

Computed over bytes `[0, 80)` — the header and payload, excluding the CRC field
itself.

---

## 4. Sequence and gap semantics

`sequence` increments once per produced frame and wraps modulo 2³². A receiver
maintains `expected_sequence` and classifies every datagram:

| Condition | Classification | Receiver action |
|---|---|---|
| `seq == expected` | In order | Accept, advance `expected`. |
| `seq == expected - 1` | Duplicate | Discard silently. |
| `seq > expected` | Gap | Accept, count `(seq - expected)` lost frames, insert a gap marker into the plot, advance `expected`. |
| `seq < expected - 1` | Out of order (reordered network) | Accept, count as reordered. |
| CRC mismatch | Corrupt | Discard, increment `crc_errors`. Not counted as loss. |

Counters exposed to the UI as the *link quality* panel:

```
sent · received · gaps · crc_errors · duplicates · out_of_order
max_gap_size · latency_p50 · latency_p99 · link_uptime_pct
```

The distinction between *lost* and *corrupt* is deliberate: they have different
causes (congestion versus noise or hardware) and therefore different fixes.

---

## 5. Versioning rules

`version` is the compatibility contract. The rules:

* **Minor, additive changes** — a new field appended, so long as `crc32` still
  covers a known prefix — do **not** bump `version`. Receivers must ignore
  trailing bytes they do not recognise.
* **Any change to an existing field's offset, size, type or unit** bumps
  `version` and requires updating both implementations in the same commit.
* A receiver seeing an unknown `version` **discards** the frame and increments a
  `protocol_mismatch` counter. It never attempts a partial parse.
* `magic` is checked before `version`, so a stream that is not ours is rejected
  in one comparison.

Any version bump must regenerate `protocol/golden_vectors.bin` via
`tools/gen_golden_vectors.py` and land the regenerated file in the same commit as
the header change.

---

## 6. Control channel (TCP 5701)

Newline-delimited JSON, one message per line, in both directions. JSON is a
deliberate choice here: the control channel is low-rate and operator-facing, so
readability with `netcat` during debugging is worth more than the bytes saved.

### 6.1 Ground station → simulator

```jsonc
{"type":"set_phase","phase":"Descent","seq":1}          // force a phase transition
{"type":"set_seed","seed":12345,"seq":2}                // reseed the noise generators
{"type":"inject_fault","fault":"engine_fire","seq":3}   // latch a failure
{"type":"pause","seq":4}                                // stop producing, keep the link up
{"type":"resume","seq":5}
{"type":"set_rate","hz":200,"seq":6}                    // production rate, 1..1000
{"type":"snapshot","seq":7}                             // request a state dump
```

### 6.2 Simulator → ground station

```jsonc
{"type":"ack","seq":3,"result":"ok"}
{"type":"nack","seq":4,"result":"rejected","reason":"invalid_phase_transition"}
{"type":"event","level":"warning","code":"engine_degraded","text":"Engine 2 EGT rising"}
{"type":"stats","sent":84213,"gaps":0,"crc_errors":0,"jitter_ns":141000}
{"type":"snapshot","state":{ ... }}
```

Every command carries a client-assigned `seq` that the simulator echoes in its
`ack`/`nack`, so the UI can correlate a request with its outcome.

---

## 7. Conformance testing

`protocol/golden_vectors.bin` is a committed, byte-exact fixture containing ten
representative frames — nominal cruise, a phase transition, an engine fault, a
CRC error, and so on. `tools/gen_golden_vectors.py` regenerates it; nobody edits
it by hand.

Two tests depend on it, and they are the mechanism that keeps the C++ and C#
implementations honest:

* **C++** (`protocol/test_frame_codec.cpp`) — encoding a known state produces
  bytes identical to the fixture, byte for byte.
* **C#** (`GroundStationUI.Tests/TelemetryDecoderTests.cs`) — decoding the
  fixture yields the known field values.

If either implementation drifts, one of the two tests fails in CI. That is the
entire point: a shared document is not a guarantee, a shared *fixture verified
on both sides* is.