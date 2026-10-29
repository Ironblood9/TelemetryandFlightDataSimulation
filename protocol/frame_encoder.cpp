#include "protocol/frame_encoder.hpp"

namespace fse
{
namespace protocol
{

TelemetryFrame make_frame(const SensorSample& sample, std::uint8_t aircraft_id) noexcept
{
    TelemetryFrame frame{};
    frame.magic              = kMagic;
    frame.version            = kProtocolVersion;
    frame.aircraft_id        = aircraft_id;
    frame.sequence           = sample.sequence;
    frame.timestamp_ns       = sample.timestamp_ns;
    frame.phase              = static_cast<std::uint8_t>(sample.phase);
    frame.status_flags       = sample.status_bits;
    frame.reserved           = 0U;
    frame.airspeed_mps       = sample.airspeed_mps;
    frame.altitude_m         = sample.altitude_m;
    frame.vertical_speed_mps = sample.vertical_speed_mps;
    frame.heading_deg        = sample.heading_deg;
    frame.pitch_deg          = sample.pitch_deg;
    frame.roll_deg           = sample.roll_deg;
    for (int i = 0; i < 4; ++i)
    {
        frame.engine_temp_c[i] = sample.engine_temp_c[static_cast<std::size_t>(i)];
    }
    frame.fuel_press_kpa = sample.fuel_press_kpa;
    frame.oil_press_kpa  = sample.oil_press_kpa;
    frame.accel_x_mps2   = sample.accel_mps2.x;
    frame.accel_y_mps2   = sample.accel_mps2.y;
    frame.accel_z_mps2   = sample.accel_mps2.z;
    // CRC is computed by encode().
    return frame;
}

} // namespace protocol
} // namespace fse