// ---------------------------------------------------------------------------
// Encode / decode for the telemetry frame.
//
// Every field is written byte by byte in little-endian order. The struct is
// never memcpy'd to or from the wire: a packed struct happens to have the
// right layout on x86, but relying on that is how a frame ends up with host
// byte order on a big-endian machine, or with padding bytes where the spec
// says there should be data.
//
// The CRC is computed over bytes [0, 80) -- everything except the CRC field
// itself. The receiver recomputes it and compares; a mismatch means the frame
// is corrupt and must be discarded, not repaired.
// ---------------------------------------------------------------------------

#include "protocol/telemetry_frame.hpp"

#include <cstring>

namespace fse
{
namespace protocol
{

namespace
{

void write_u16(std::uint8_t* buf, std::uint16_t value) noexcept
{
    buf[0] = static_cast<std::uint8_t>(value & 0xFFU);
    buf[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
}

void write_u32(std::uint8_t* buf, std::uint32_t value) noexcept
{
    buf[0] = static_cast<std::uint8_t>(value & 0xFFU);
    buf[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
    buf[2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
    buf[3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
}

void write_u64(std::uint8_t* buf, std::uint64_t value) noexcept
{
    for (int i = 0; i < 8; ++i)
    {
        buf[i] = static_cast<std::uint8_t>((value >> (i * 8)) & 0xFFU);
    }
}

void write_f32(std::uint8_t* buf, float value) noexcept
{
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32(buf, bits);
}

std::uint16_t read_u16(const std::uint8_t* buf) noexcept
{
    return static_cast<std::uint16_t>(buf[0]) |
           (static_cast<std::uint16_t>(buf[1]) << 8);
}

std::uint32_t read_u32(const std::uint8_t* buf) noexcept
{
    return static_cast<std::uint32_t>(buf[0]) |
           (static_cast<std::uint32_t>(buf[1]) << 8) |
           (static_cast<std::uint32_t>(buf[2]) << 16) |
           (static_cast<std::uint32_t>(buf[3]) << 24);
}

std::uint64_t read_u64(const std::uint8_t* buf) noexcept
{
    std::uint64_t value = 0;
    for (int i = 0; i < 8; ++i)
    {
        value |= (static_cast<std::uint64_t>(buf[i]) << (i * 8));
    }
    return value;
}

float read_f32(const std::uint8_t* buf) noexcept
{
    const std::uint32_t bits = read_u32(buf);
    float value = 0.0F;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

void encode(const TelemetryFrame& frame, std::uint8_t* buffer) noexcept
{
    write_u16(buffer + 0, frame.magic);
    buffer[2] = frame.version;
    buffer[3] = frame.aircraft_id;
    write_u32(buffer + 4, frame.sequence);
    write_u64(buffer + 8, frame.timestamp_ns);
    buffer[16] = frame.phase;
    write_u16(buffer + 17, frame.status_flags);
    buffer[19] = frame.reserved;

    write_f32(buffer + 20, frame.airspeed_mps);
    write_f32(buffer + 24, frame.altitude_m);
    write_f32(buffer + 28, frame.vertical_speed_mps);
    write_f32(buffer + 32, frame.heading_deg);
    write_f32(buffer + 36, frame.pitch_deg);
    write_f32(buffer + 40, frame.roll_deg);

    for (int i = 0; i < 4; ++i)
    {
        write_f32(buffer + 44 + (i * 4), frame.engine_temp_c[i]);
    }

    write_f32(buffer + 60, frame.fuel_press_kpa);
    write_f32(buffer + 64, frame.oil_press_kpa);

    write_f32(buffer + 68, frame.accel_x_mps2);
    write_f32(buffer + 72, frame.accel_y_mps2);
    write_f32(buffer + 76, frame.accel_z_mps2);

    // CRC over bytes [0, 80).
    const std::uint32_t crc = crc32_iso_hdlc(buffer, kCrcCoverage);
    write_u32(buffer + 80, crc);
}

bool decode(const std::uint8_t* buffer, TelemetryFrame& frame) noexcept
{
    frame.magic          = read_u16(buffer + 0);
    frame.version        = buffer[2];
    frame.aircraft_id    = buffer[3];
    frame.sequence       = read_u32(buffer + 4);
    frame.timestamp_ns   = read_u64(buffer + 8);
    frame.phase          = buffer[16];
    frame.status_flags   = read_u16(buffer + 17);
    frame.reserved       = buffer[19];

    frame.airspeed_mps       = read_f32(buffer + 20);
    frame.altitude_m         = read_f32(buffer + 24);
    frame.vertical_speed_mps = read_f32(buffer + 28);
    frame.heading_deg        = read_f32(buffer + 32);
    frame.pitch_deg          = read_f32(buffer + 36);
    frame.roll_deg           = read_f32(buffer + 40);

    for (int i = 0; i < 4; ++i)
    {
        frame.engine_temp_c[i] = read_f32(buffer + 44 + (i * 4));
    }

    frame.fuel_press_kpa = read_f32(buffer + 60);
    frame.oil_press_kpa  = read_f32(buffer + 64);

    frame.accel_x_mps2 = read_f32(buffer + 68);
    frame.accel_y_mps2 = read_f32(buffer + 72);
    frame.accel_z_mps2 = read_f32(buffer + 76);

    frame.crc32 = read_u32(buffer + 80);

    // Verify integrity.
    const std::uint32_t computed = crc32_iso_hdlc(buffer, kCrcCoverage);
    return computed == frame.crc32;
}

std::uint32_t compute_crc(const std::uint8_t* buffer) noexcept
{
    return crc32_iso_hdlc(buffer, kCrcCoverage);
}

}  // namespace protocol
}  // namespace fse