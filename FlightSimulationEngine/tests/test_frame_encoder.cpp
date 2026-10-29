// ---------------------------------------------------------------------------
// Frame encoder tests.
//
// The encoder is the single place where the in-memory representation and the
// wire representation meet. Every field mapping is tested here, so that a
// rename or a reordering in either struct is caught immediately rather than
// showing up as a shifted column in a CSV file two milestones later.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "protocol/frame_encoder.hpp"
#include "protocol/telemetry_frame.hpp"

namespace fse
{
namespace protocol
{
namespace
{

SensorSample make_sample()
{
    SensorSample sample{};
    sample.timestamp_ns       = 0x0123456789ABCDEFULL;
    sample.sequence           = 42U;
    sample.phase              = FlightPhase::kCruise;
    sample.status_bits        = 0x0005U;
    sample.airspeed_mps       = 123.5F;
    sample.altitude_m         = 900.0F;
    sample.vertical_speed_mps = 5.5F;
    sample.heading_deg        = 270.0F;
    sample.pitch_deg          = 3.5F;
    sample.roll_deg           = -15.0F;
    sample.engine_temp_c[0]   = 600.0F;
    sample.engine_temp_c[1]   = 610.0F;
    sample.engine_temp_c[2]   = 620.0F;
    sample.engine_temp_c[3]   = 630.0F;
    sample.fuel_press_kpa     = 42.0F;
    sample.oil_press_kpa      = 38.0F;
    sample.accel_mps2         = Vec3{0.1F, 9.81F, -0.2F};
    return sample;
}

TEST(FrameEncoder, MapsEveryField)
{
    const SensorSample   sample = make_sample();
    const TelemetryFrame frame  = make_frame(sample, 7U);

    EXPECT_EQ(frame.magic, kMagic);
    EXPECT_EQ(frame.version, kProtocolVersion);
    EXPECT_EQ(frame.aircraft_id, 7U);
    EXPECT_EQ(frame.sequence, sample.sequence);
    EXPECT_EQ(frame.timestamp_ns, sample.timestamp_ns);
    EXPECT_EQ(frame.phase, static_cast<std::uint8_t>(sample.phase));
    std::uint16_t status = 0;
    std::memcpy(&status, &frame.status_flags, sizeof(status));
    EXPECT_EQ(status, sample.status_bits);
    EXPECT_FLOAT_EQ(frame.airspeed_mps, sample.airspeed_mps);
    EXPECT_FLOAT_EQ(frame.altitude_m, sample.altitude_m);
    EXPECT_FLOAT_EQ(frame.vertical_speed_mps, sample.vertical_speed_mps);
    EXPECT_FLOAT_EQ(frame.heading_deg, sample.heading_deg);
    EXPECT_FLOAT_EQ(frame.pitch_deg, sample.pitch_deg);
    EXPECT_FLOAT_EQ(frame.roll_deg, sample.roll_deg);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_FLOAT_EQ(frame.engine_temp_c[i], sample.engine_temp_c[static_cast<std::size_t>(i)])
            << "engine " << i;
    }
    EXPECT_FLOAT_EQ(frame.fuel_press_kpa, sample.fuel_press_kpa);
    EXPECT_FLOAT_EQ(frame.oil_press_kpa, sample.oil_press_kpa);
    EXPECT_FLOAT_EQ(frame.accel_x_mps2, sample.accel_mps2.x);
    EXPECT_FLOAT_EQ(frame.accel_y_mps2, sample.accel_mps2.y);
    EXPECT_FLOAT_EQ(frame.accel_z_mps2, sample.accel_mps2.z);
}

TEST(FrameEncoder, DefaultAircraftIdIsOne)
{
    const SensorSample   sample = make_sample();
    const TelemetryFrame frame  = make_frame(sample);

    EXPECT_EQ(frame.aircraft_id, 1U);
}

TEST(FrameEncoder, EncodedFrameDecodesBackToTheSameSample)
{
    const SensorSample   sample = make_sample();
    const TelemetryFrame frame  = make_frame(sample, 3U);

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(frame, buffer.data());

    TelemetryFrame decoded{};
    EXPECT_TRUE(decode(buffer.data(), decoded));

    EXPECT_EQ(decoded.sequence, sample.sequence);
    EXPECT_EQ(decoded.timestamp_ns, sample.timestamp_ns);
    EXPECT_FLOAT_EQ(decoded.airspeed_mps, sample.airspeed_mps);
    EXPECT_FLOAT_EQ(decoded.altitude_m, sample.altitude_m);
    EXPECT_FLOAT_EQ(decoded.accel_x_mps2, sample.accel_mps2.x);
    EXPECT_FLOAT_EQ(decoded.accel_y_mps2, sample.accel_mps2.y);
    EXPECT_FLOAT_EQ(decoded.accel_z_mps2, sample.accel_mps2.z);
}

TEST(FrameEncoder, DoesNotSetTheCrc)
{
    // The CRC is computed by encode(), not by make_frame(). If make_frame()
    // set it, the value would be stale the moment the frame was modified, and
    // every caller would have to remember to recompute it. The encoder's job
    // is field mapping only.
    const SensorSample   sample = make_sample();
    const TelemetryFrame frame  = make_frame(sample);

    EXPECT_EQ(frame.crc32, 0U);
}

TEST(FrameEncoder, RoundTripsThroughTheFullPipeline)
{
    // The end-to-end check: SensorSample -> TelemetryFrame -> bytes -> TelemetryFrame.
    // This is the path the ground station will exercise in reverse.
    const SensorSample sample = make_sample();

    std::array<std::uint8_t, kFrameSize> buffer{};
    encode(make_frame(sample, 1U), buffer.data());

    TelemetryFrame decoded{};
    ASSERT_TRUE(decode(buffer.data(), decoded));

    EXPECT_EQ(decoded.sequence, sample.sequence);
    EXPECT_EQ(decoded.phase, static_cast<std::uint8_t>(sample.phase));
    std::uint16_t decoded_status = 0;
    std::memcpy(&decoded_status, &decoded.status_flags, sizeof(decoded_status));
    EXPECT_EQ(decoded_status, sample.status_bits);
}

} // namespace
} // namespace protocol
} // namespace fse