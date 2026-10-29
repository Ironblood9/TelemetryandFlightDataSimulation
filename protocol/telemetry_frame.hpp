// ---------------------------------------------------------------------------
// Telemetry frame — the wire format.
//
// One `TelemetryFrame` per UDP datagram, 84 bytes. The layout is frozen in
// `docs/protocol-spec.md` section 3 and must not change without a version bump.
//
// The struct is packed (no implicit padding) and every offset is pinned with
// `static_assert(offsetof(...))`. A change to any field's offset, size or type
// is a compile error rather than a silent wire-format break.
//
// The struct is NEVER memcpy'd to or from the wire. encode() and decode()
// write each field individually in little-endian byte order, so the host byte
// order is irrelevant. The struct exists so that the field names, types and
// offsets live in one place instead of being scattered across the encoder.
// ---------------------------------------------------------------------------

#pragma once

#include <cstddef>
#include <cstdint>

#include "protocol/crc32.hpp"

namespace fse
{
namespace protocol
{

/// Magic number: 0xA55A. Framing sanity check on the receiving end.
constexpr std::uint16_t kMagic = 0xA55AU;

/// Protocol version. Bumped only on a breaking change to the frame layout.
constexpr std::uint8_t kProtocolVersion = 1U;

/// Size of one frame on the wire, in bytes.
constexpr std::size_t kFrameSize = 84U;

/// Number of bytes covered by the CRC (everything except the CRC field itself).
constexpr std::size_t kCrcCoverage = 80U;

#pragma pack(push, 1)
struct TelemetryFrame
{
    // --- Header (bytes 0..19) ---
    std::uint16_t magic;          // 0:  0xA55A
    std::uint8_t  version;        // 2:  protocol version
    std::uint8_t  aircraft_id;    // 3:  slot number
    std::uint32_t sequence;       // 4:  monotonic, wraps at 2^32
    std::uint64_t timestamp_ns;   // 8:  monotonic clock, nanoseconds
    std::uint8_t  phase;          // 16: FlightPhase
    std::uint16_t status_flags;    // 17: bitfield
    std::uint8_t  reserved;       // 19: padding to 4-byte align airspeed

    // --- Payload (bytes 20..79) ---
    float airspeed_mps;           // 20: indicated airspeed
    float altitude_m;             // 24: above mean sea level
    float vertical_speed_mps;     // 28: rate of climb, positive up
    float heading_deg;            // 32: [0, 360), clockwise from true north
    float pitch_deg;              // 36: [-90, 90], positive nose-up
    float roll_deg;               // 40: [-180, 180), positive right-wing-down

    float engine_temp_c[4];       // 44: left outer, left inner, right inner, right outer

    float fuel_press_kpa;         // 60: gauge pressure
    float oil_press_kpa;          // 64: gauge pressure

    float accel_x_mps2;           // 68: body axes, specific force
    float accel_y_mps2;           // 72
    float accel_z_mps2;           // 76: includes gravity, 1 g at rest

    // --- Trailer (bytes 80..83) ---
    std::uint32_t crc32;          // 80: over bytes [0, 80)
};
#pragma pack(pop)

// --- Offset pins -------------------------------------------------------------
// A change to any of these is a compile error, which is the point: the wire
// format is frozen and a silent shift is worse than a build failure.

static_assert(offsetof(TelemetryFrame, magic) == 0);
static_assert(offsetof(TelemetryFrame, version) == 2);
static_assert(offsetof(TelemetryFrame, aircraft_id) == 3);
static_assert(offsetof(TelemetryFrame, sequence) == 4);
static_assert(offsetof(TelemetryFrame, timestamp_ns) == 8);
static_assert(offsetof(TelemetryFrame, phase) == 16);
static_assert(offsetof(TelemetryFrame, status_flags) == 17);
static_assert(offsetof(TelemetryFrame, reserved) == 19);
static_assert(offsetof(TelemetryFrame, airspeed_mps) == 20);
static_assert(offsetof(TelemetryFrame, altitude_m) == 24);
static_assert(offsetof(TelemetryFrame, vertical_speed_mps) == 28);
static_assert(offsetof(TelemetryFrame, heading_deg) == 32);
static_assert(offsetof(TelemetryFrame, pitch_deg) == 36);
static_assert(offsetof(TelemetryFrame, roll_deg) == 40);
static_assert(offsetof(TelemetryFrame, engine_temp_c) == 44);
static_assert(offsetof(TelemetryFrame, fuel_press_kpa) == 60);
static_assert(offsetof(TelemetryFrame, oil_press_kpa) == 64);
static_assert(offsetof(TelemetryFrame, accel_x_mps2) == 68);
static_assert(offsetof(TelemetryFrame, accel_y_mps2) == 72);
static_assert(offsetof(TelemetryFrame, accel_z_mps2) == 76);
static_assert(offsetof(TelemetryFrame, crc32) == 80);

static_assert(sizeof(TelemetryFrame) == kFrameSize);

// --- Encode / decode --------------------------------------------------------

/// Encode a frame into a 84-byte buffer in little-endian byte order.
/// The CRC is computed over bytes [0, 80) and written into the trailer.
void encode(const TelemetryFrame& frame, std::uint8_t* buffer) noexcept;

/// Decode a 84-byte buffer into a frame. Returns true if the CRC matches.
[[nodiscard]] bool decode(const std::uint8_t* buffer, TelemetryFrame& frame) noexcept;

/// Compute the CRC over a raw buffer without decoding. Used by the receiver
/// to check integrity before committing to a decode.
[[nodiscard]] std::uint32_t compute_crc(const std::uint8_t* buffer) noexcept;

}  // namespace protocol
}  // namespace fse