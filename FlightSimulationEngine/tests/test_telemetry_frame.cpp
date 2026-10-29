// ---------------------------------------------------------------------------
// Telemetry frame tests.
//
// These pin the wire format. A change to any field's offset, size or type is
// a compile error (static_assert), and a change to the encoded bytes is a test
// failure. The two together are what make the format a contract rather than a
// convention.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "protocol/telemetry_frame.hpp"

namespace fse
{
namespace protocol
{
namespace
{

/// Builds a frame with every field set to a recognisable value, so that a
/// byte-level mismatch in the test output points at the field that moved.
TelemetryFrame make_test_frame()
{
    TelemetryFrame frame{};
    frame.magic              = kMagic;
    frame.version            = kProtocolVersion;
    frame.aircraft_id        = 7U;
    frame.sequence           = 0x12345678U;
    frame.timestamp_ns       = 0x0123456789ABCDEFULL;
    frame.phase              = 4U; // kCruise
    frame.status_flags       = 0x0003U;
    frame.reserved           = 0U;
    frame.airspeed_mps       = 123.5F;
    frame.altitude_m         = 900.0F;
    frame.vertical_speed_mps = 5.5F;
    frame.heading_deg        = 270.0F;
    frame.pitch_deg          = 3.5F;
    frame.roll_deg           = -15.0F;
    frame.engine_temp_c[0]   = 600.0F;
    frame.engine_temp_c[1]   = 610.0F;
    frame.engine_temp_c[2]   = 620.0F;
    frame.engine_temp_c[3]   = 630.0F;
    frame.fuel_press_kpa     = 42.0F;
    frame.oil_press_kpa      = 38.0F;
    frame.accel_x_mps2       = 0.1F;
    frame.accel_y_mps2       = 9.81F;
    frame.accel_z_mps2       = -0.2F;
    return frame;
}

TEST(TelemetryFrame, SizeIs84Bytes)
{
    EXPECT_EQ(sizeof(TelemetryFrame), 84U);
    EXPECT_EQ(kFrameSize, 84U);
}

TEST(TelemetryFrame, MagicIsCorrect)
{
    EXPECT_EQ(kMagic, 0xA55AU);
}

TEST(TelemetryFrame, RoundTripPreservesAllFields)
{
    const TelemetryFrame original = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(original, buffer.data());

    TelemetryFrame decoded{};
    const bool     crc_ok = decode(buffer.data(), decoded);

    EXPECT_TRUE(crc_ok) << "CRC mismatch on a clean frame";
    EXPECT_EQ(decoded.magic, original.magic);
    EXPECT_EQ(decoded.version, original.version);
    EXPECT_EQ(decoded.aircraft_id, original.aircraft_id);
    EXPECT_EQ(decoded.sequence, original.sequence);
    EXPECT_EQ(decoded.timestamp_ns, original.timestamp_ns);
    EXPECT_EQ(decoded.phase, original.phase);
    std::uint16_t decoded_status = 0, original_status = 0;
    std::memcpy(&decoded_status, &decoded.status_flags, sizeof(decoded_status));
    std::memcpy(&original_status, &original.status_flags, sizeof(original_status));
    EXPECT_EQ(decoded_status, original_status);
    EXPECT_FLOAT_EQ(decoded.airspeed_mps, original.airspeed_mps);
    EXPECT_FLOAT_EQ(decoded.altitude_m, original.altitude_m);
    EXPECT_FLOAT_EQ(decoded.vertical_speed_mps, original.vertical_speed_mps);
    EXPECT_FLOAT_EQ(decoded.heading_deg, original.heading_deg);
    EXPECT_FLOAT_EQ(decoded.pitch_deg, original.pitch_deg);
    EXPECT_FLOAT_EQ(decoded.roll_deg, original.roll_deg);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(decoded.engine_temp_c[i], original.engine_temp_c[i]) << "engine " << i;
    }
    EXPECT_FLOAT_EQ(decoded.fuel_press_kpa, original.fuel_press_kpa);
    EXPECT_FLOAT_EQ(decoded.oil_press_kpa, original.oil_press_kpa);
    EXPECT_FLOAT_EQ(decoded.accel_x_mps2, original.accel_x_mps2);
    EXPECT_FLOAT_EQ(decoded.accel_y_mps2, original.accel_y_mps2);
    EXPECT_FLOAT_EQ(decoded.accel_z_mps2, original.accel_z_mps2);
    EXPECT_EQ(decoded.crc32, compute_crc(buffer.data()));
}

TEST(TelemetryFrame, EncodeWritesLittleEndian)
{
    TelemetryFrame frame{};
    frame.sequence = 0x01020304U;

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    // Little-endian: least significant byte first.
    EXPECT_EQ(buffer[4], 0x04U);
    EXPECT_EQ(buffer[5], 0x03U);
    EXPECT_EQ(buffer[6], 0x02U);
    EXPECT_EQ(buffer[7], 0x01U);
}

TEST(TelemetryFrame, EncodeWritesMagicAtOffset0)
{
    TelemetryFrame frame{};
    frame.magic = kMagic;

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    EXPECT_EQ(buffer[0], 0x5AU); // 0xA55A little-endian
    EXPECT_EQ(buffer[1], 0xA5U);
}

TEST(TelemetryFrame, DecodeRejectsCorruptedFrame)
{
    TelemetryFrame frame = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    // Flip a bit in the payload (airspeed field).
    buffer[20] ^= 0x01U;

    TelemetryFrame decoded{};
    const bool     crc_ok = decode(buffer.data(), decoded);

    EXPECT_FALSE(crc_ok) << "a corrupted frame must not pass the CRC check";
}

TEST(TelemetryFrame, DecodeRejectsCorruptedCrc)
{
    TelemetryFrame frame = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    // Flip a bit in the CRC field itself.
    buffer[80] ^= 0x01U;

    TelemetryFrame decoded{};
    const bool     crc_ok = decode(buffer.data(), decoded);

    EXPECT_FALSE(crc_ok);
}

TEST(TelemetryFrame, CrcCoversBytes0To80)
{
    // The CRC is over [0, 80). A change to byte 79 must be detected; a change
    // to byte 80 (the CRC itself) is a different kind of corruption.
    TelemetryFrame frame = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    const std::uint32_t original_crc = compute_crc(buffer.data());

    // Change byte 79 (last payload byte, accel_z).
    buffer[79] ^= 0xFFU;
    const std::uint32_t corrupted_crc = compute_crc(buffer.data());

    EXPECT_NE(original_crc, corrupted_crc) << "the CRC did not cover byte 79";
}

TEST(TelemetryFrame, CrcDoesNotCoverTheCrcField)
{
    // Bytes 80..83 are the CRC itself and must not be included in the
    // computation, otherwise the CRC would be self-referential.
    TelemetryFrame frame = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    // Change the CRC field.
    buffer[80] ^= 0xFFU;
    const std::uint32_t recomputed = compute_crc(buffer.data());

    // The recomputed CRC should match the original (bytes 0..79 unchanged).
    TelemetryFrame                       original = make_test_frame();
    std::array<std::uint8_t, kFrameSize> original_buffer{};
    encode(original, original_buffer.data());

    EXPECT_EQ(recomputed, compute_crc(original_buffer.data()))
        << "the CRC covered the CRC field itself";
}

TEST(TelemetryFrame, EngineTempArrayIsContiguous)
{
    // engine_temp_c must occupy bytes 44..59 with no gaps, so that a C#
    // decoder can read it as a fixed-size array.
    TelemetryFrame frame = make_test_frame();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    // Read engine_temp[0] directly from bytes 44..47.
    float temp0 = 0.0F;
    std::memcpy(&temp0, buffer.data() + 44, sizeof(temp0));
    // It should be little-endian 600.0.
    EXPECT_FLOAT_EQ(temp0, 600.0F) << "engine_temp[0] not at offset 44";
}

TEST(TelemetryFrame, VersionFieldIsOne)
{
    TelemetryFrame frame{};
    frame.version = kProtocolVersion;

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    EXPECT_EQ(buffer[2], 1U);
}

} // namespace
} // namespace protocol
} // namespace fse