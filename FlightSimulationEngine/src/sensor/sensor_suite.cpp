#include "fse/sensor.hpp"

#include <algorithm>
#include <cmath>

namespace fse
{

// ---------------------------------------------------------------------------
// AirDataComputer
// ---------------------------------------------------------------------------

void AirDataComputer::reset(std::uint64_t seed, Scalar initial_altitude_m) noexcept
{
    seed_ = seed;
    noise_rng_.reseed(seed, kNoiseStream);
    altitude_bias_.reseed(seed, kBiasStream);

    ias_mps_ = 0.0F;

    // The barometric chain starts at the seeded altitude, not at zero.
    altitude_m_                = initial_altitude_m;
    previous_altitude_m_       = initial_altitude_m;
    altitude_trend_m_          = initial_altitude_m;
    previous_altitude_trend_m_ = initial_altitude_m;
    vertical_speed_mps_        = 0.0F;
    heading_deg_               = 0.0F;
    pitch_deg_                 = 0.0F;
    roll_deg_                  = 0.0F;
    specific_force_mps2_       = Vec3{0.0F, 0.0F, 0.0F};

    alignment_s_        = 0.0F;
    alignment_bias_deg_ = kInitialAlignmentErrorDeg;

    // Nothing is trusted until the alignment has run and the filters have
    // settled. Reporting kGood on the first frame would be a lie that the
    // consumer has no way to detect.
    ias_quality_      = SensorQuality::kInvalid;
    altitude_quality_ = SensorQuality::kInvalid;
    attitude_quality_ = SensorQuality::kInvalid;
}

void AirDataComputer::update(const AircraftState& state, Scalar dt) noexcept
{
    alignment_s_ += dt;
    alignment_bias_deg_ = approach(alignment_bias_deg_, 0.0F, kAlignmentTimeS / 4.5F, dt);

    // --- Pitot --------------------------------------------------------------
    // The pitot measures stagnation pressure; recovering the indicated value
    // from it is the flight model's job. What this stage contributes is the
    // static line lag and the transducer noise.
    ias_mps_ = approach(ias_mps_, state.ias_mps, kPitotTauS, dt) +
               noise_rng_.next_normal(0.0F, kIasNoiseMps);

    // --- Barometric altitude ------------------------------------------------
    const Scalar altitude_truth = state.position_m.y + altitude_bias_.next(dt);
    previous_altitude_m_        = altitude_m_;
    altitude_m_                 = approach(altitude_m_, altitude_truth, kBarometricTauS, dt) +
                  noise_rng_.next_normal(0.0F, kAltitudeNoiseM);

    // --- Vertical speed -----------------------------------------------------
    // Differentiating the *filtered* altitude is the honest model: it inherits
    // the barometric lag, so the channel both trails the true rate and rounds
    // off the corners of a manoeuvre. Differentiating the truth would describe
    // an instrument nobody flies.
    //
    // The derivative is taken from a much harder filtered altitude than the one
    // that is reported. At 100 Hz the difference between two consecutive noisy
    // barometric samples is 2.55 m (sqrt(2) * 1.8 m of sample noise), which
    // divided by dt is 255 m/s of entirely fictitious climb rate. Real vertical
    // speed instruments average over seconds for exactly this reason.
    altitude_trend_m_ = approach(altitude_trend_m_, altitude_m_, kAltitudeTrendTauS, dt);

    const Scalar baro_rate =
        (dt > 0.0F) ? ((altitude_trend_m_ - previous_altitude_trend_m_) / dt) : 0.0F;
    previous_altitude_trend_m_ = altitude_trend_m_;

    vertical_speed_mps_ = approach(vertical_speed_mps_, baro_rate, kVerticalSpeedTauS, dt) +
                          noise_rng_.next_normal(0.0F, kVerticalSpeedNoiseMps);

    // --- Attitude -----------------------------------------------------------
    heading_deg_ = wrap_degrees(
        approach(heading_deg_, state.heading_deg + alignment_bias_deg_, kAttitudeTauS, dt) +
        noise_rng_.next_normal(0.0F, kAttitudeNoiseDeg));
    pitch_deg_ = approach(pitch_deg_, state.pitch_deg, kAttitudeTauS, dt) +
                 noise_rng_.next_normal(0.0F, kAttitudeNoiseDeg);
    roll_deg_ = wrap_degrees_signed(approach(roll_deg_, state.roll_deg, kAttitudeTauS, dt) +
                                    noise_rng_.next_normal(0.0F, kAttitudeNoiseDeg));

    // --- Specific force -----------------------------------------------------
    // Body vertical specific force: 1 g in level flight, more in a pull up,
    // collapsing towards zero as the wing stalls.
    specific_force_mps2_ = Vec3{0.0F, state.load_factor * kGravity, 0.0F};

    // --- Validity -----------------------------------------------------------
    if (!is_finite(ias_mps_) || ias_mps_ > kMaxTrustedIasMps || ias_mps_ < -5.0F)
    {
        ias_quality_ = SensorQuality::kInvalid;
    }
    else if (ias_mps_ > (kMaxTrustedIasMps * 0.92F))
    {
        ias_quality_ = SensorQuality::kDegraded;
    }
    else
    {
        ias_quality_ = SensorQuality::kGood;
    }

    if (!is_finite(altitude_m_) || altitude_m_ < kMinTrustedAltitudeM)
    {
        altitude_quality_ = SensorQuality::kInvalid;
    }
    else if (!aligned())
    {
        altitude_quality_ = SensorQuality::kDegraded;
    }
    else
    {
        altitude_quality_ = SensorQuality::kGood;
    }

    attitude_quality_ = aligned() ? SensorQuality::kGood : SensorQuality::kInvalid;
}

// ---------------------------------------------------------------------------
// EngineMonitor
// ---------------------------------------------------------------------------

void EngineMonitor::reset(std::uint64_t seed) noexcept
{
    seed_ = seed;
    noise_rng_.reseed(seed, kNoiseStream);

    temperature_c_.fill(kAmbientTemperatureC);
    target_temperature_c_.fill(kAmbientTemperatureC);
    fuel_press_kpa_ = 0.0F;
    oil_press_kpa_  = 0.0F;
    engine_quality_ = SensorQuality::kInvalid;
}

void EngineMonitor::update(const AircraftState& state, Scalar dt) noexcept
{
    const Scalar throttle = clamp(state.throttle, 0.0F, 1.0F);

    // Temperature at idle is deliberately non zero: an idling turbine still sits
    // well above ambient.
    constexpr Scalar idle_fraction = 0.25F;
    Scalar           target        = kAmbientTemperatureC +
                    (kMaxTemperatureC * (idle_fraction + ((1.0F - idle_fraction) * throttle)));

    if (has_fault(state.faults, FaultCode::kEngineOut))
    {
        target = kAmbientTemperatureC;
    }
    else if (has_fault(state.faults, FaultCode::kEngineFire))
    {
        // A fire is a temperature excursion, not a throttle change.
        target = kMaxTemperatureC * 1.12F;
    }
    else if (state.fuel_kg <= 0.0F)
    {
        // Fuel exhaustion stops the engines even without an explicit fault.
        target = kAmbientTemperatureC;
    }

    target_temperature_c_.fill(target);

    // Each channel gets independent noise. A single shared deviate would be
    // common mode, which the ground station would be unable to distinguish from
    // a real engine to engine spread.
    for (std::size_t i = 0; i < kEngineCount; ++i)
    {
        // First order thermal lag. This is the reason the channel exists: the
        // temperature trails the throttle by several seconds, and that lag is
        // what the health logic has to survive.
        temperature_c_[i] =
            approach(temperature_c_[i], target_temperature_c_[i], kThermalTauS, dt) +
            noise_rng_.next_normal(0.0F, kTemperatureNoiseC);
    }

    const auto [coldest_it, hottest_it] =
        std::minmax_element(temperature_c_.begin(), temperature_c_.end());
    const Scalar hottest = *hottest_it;
    const Scalar spread  = hottest - *coldest_it;

    // Fuel and oil pressure follow the throttle with a much shorter lag: they
    // are hydraulic, not thermal, quantities.
    fuel_press_kpa_ = approach(fuel_press_kpa_, kMaxFuelPressureKpa * (0.35F + (0.65F * throttle)),
                               kPressureTauS, dt) +
                      noise_rng_.next_normal(0.0F, 0.9F);
    oil_press_kpa_ = approach(oil_press_kpa_, kMaxOilPressureKpa * (0.20F + (0.80F * throttle)),
                              kPressureTauS, dt) +
                     noise_rng_.next_normal(0.0F, 1.4F);

    fuel_press_kpa_ = clamp(fuel_press_kpa_, 0.0F, kMaxFuelPressureKpa);
    oil_press_kpa_  = clamp(oil_press_kpa_, 0.0F, kMaxOilPressureKpa);

    if (has_fault(state.faults, FaultCode::kEngineFire) || hottest > kRedlineTemperatureC)
    {
        engine_quality_ = SensorQuality::kInvalid;
    }
    else if (hottest > (kRedlineTemperatureC * 0.92F) || spread > kEngineSpreadDegradedC)
    {
        engine_quality_ = SensorQuality::kDegraded;
    }
    else if (throttle > kIdleQualityThreshold)
    {
        engine_quality_ = SensorQuality::kGood;
    }
    else
    {
        // Idling is not a fault, but it is not good data either.
        engine_quality_ = SensorQuality::kDegraded;
    }
}

// ---------------------------------------------------------------------------
// SensorSuite
// ---------------------------------------------------------------------------

void SensorSuite::reset(std::uint64_t seed, Scalar initial_altitude_m) noexcept
{
    air_data_.reset(seed, initial_altitude_m);
    engines_.reset(seed);
}

void SensorSuite::update(const AircraftState& state, SensorSample& out, Scalar dt) noexcept
{
    air_data_.update(state, dt);
    engines_.update(state, dt);

    out.phase  = state.phase;
    out.faults = state.faults;

    out.airspeed_mps       = air_data_.indicated_airspeed_mps();
    out.altitude_m         = air_data_.barometric_altitude_m();
    out.vertical_speed_mps = air_data_.vertical_speed_mps();
    out.heading_deg        = air_data_.heading_deg();
    out.pitch_deg          = air_data_.pitch_deg();
    out.roll_deg           = air_data_.roll_deg();
    out.engine_temp_c      = engines_.temperatures_c();
    out.fuel_press_kpa     = engines_.fuel_pressure_kpa();
    out.oil_press_kpa      = engines_.oil_pressure_kpa();
    out.accel_mps2         = air_data_.specific_force_mps2();

    SensorStatus status = SensorStatus::kNone;
    if (air_data_.airspeed_quality() != SensorQuality::kGood)
    {
        status |= SensorStatus::kAirDataDegraded;
    }
    if (air_data_.altitude_quality() != SensorQuality::kGood)
    {
        status |= SensorStatus::kBarometricAltitudeUnreliable;
    }
    if (air_data_.attitude_quality() != SensorQuality::kGood)
    {
        status |= SensorStatus::kAttitudeNotAligned;
    }
    if (engines_.engine_quality() != SensorQuality::kGood)
    {
        status |= SensorStatus::kEngineDegraded;
    }
    out.status_bits = to_bits(status);
}

} // namespace fse