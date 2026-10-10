#pragma once

/// @file sensor.hpp
/// @brief Sensor suite: the layer between the true state and the telemetry.
///
/// Modelling the sensors separately from the physics is the point of this file.
/// A telemetry stream never sees the true state; it sees what the air data
/// computer and the engine monitor *believe*. That belief is shaped by lag,
/// bias, noise and validity rules, and every one of those is a source of
/// behaviour a consumer of the telemetry has to cope with.

#include <array>
#include <cstdint>

#include "fse/aircraft.hpp"
#include "fse/noise.hpp"
#include "fse/types.hpp"

namespace fse
{

/// Per channel health, reported alongside the value.
enum class SensorQuality : std::uint8_t
{
    kGood = 0,
    /// Usable but outside the calibrated envelope.
    kDegraded = 1,
    /// Not usable. A consumer must substitute, not trust.
    kInvalid = 2,
};

/// Suite level status, mapped one to one onto the `status_flags` field of the
/// telemetry frame. See docs/protocol-spec.md section 3.2.
enum class SensorStatus : std::uint16_t
{
    kNone                         = 0U,
    kAirDataDegraded              = 1U << 0U,
    kEngineDegraded               = 1U << 1U,
    kAttitudeNotAligned           = 1U << 2U,
    kBarometricAltitudeUnreliable = 1U << 3U,
};

[[nodiscard]] constexpr std::uint16_t to_bits(SensorStatus status) noexcept
{
    return static_cast<std::uint16_t>(status);
}

[[nodiscard]] constexpr bool has_status(SensorStatus status, SensorStatus bit) noexcept
{
    return (static_cast<std::uint16_t>(status) & static_cast<std::uint16_t>(bit)) != 0U;
}

constexpr SensorStatus operator|(SensorStatus lhs, SensorStatus rhs) noexcept
{
    return static_cast<SensorStatus>(static_cast<std::uint16_t>(lhs) |
                                     static_cast<std::uint16_t>(rhs));
}

constexpr SensorStatus operator&(SensorStatus lhs, SensorStatus rhs) noexcept
{
    return static_cast<SensorStatus>(static_cast<std::uint16_t>(lhs) &
                                     static_cast<std::uint16_t>(rhs));
}

constexpr SensorStatus& operator|=(SensorStatus& lhs, SensorStatus rhs) noexcept
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr SensorStatus& operator&=(SensorStatus& lhs, SensorStatus rhs) noexcept
{
    lhs = lhs & rhs;
    return lhs;
}

/// One produced telemetry cycle. Field for field, this is what the frame
/// encoder consumes in milestone M2.
struct SensorSample
{
    std::uint64_t timestamp_ns{0};
    std::uint32_t sequence{0};

    FlightPhase phase{FlightPhase::kPreflight};
    FaultCode   faults{FaultCode::kNone};

    Scalar airspeed_mps{0.0F};
    Scalar altitude_m{0.0F};
    Scalar vertical_speed_mps{0.0F};
    Scalar heading_deg{0.0F};
    Scalar pitch_deg{0.0F};
    Scalar roll_deg{0.0F};

    std::array<Scalar, kEngineCount> engine_temp_c{};

    Scalar fuel_press_kpa{0.0F};
    Scalar oil_press_kpa{0.0F};

    /// Specific force in the body frame, m/s^2. Contains gravity, so it reads
    /// close to 1 g in level flight.
    Vec3 accel_mps2{};

    std::uint16_t status_bits{0};
};

/// Air data computer: pitot static system plus an attitude reference.
///
/// The filters are first order lags rather than instant copies, because that is
/// what produces the two artefacts every telemetry consumer has to handle: the
/// vertical speed channel *lags* the true one, and the indicated airspeed
/// settles slowly after a manoeuvre.
class AirDataComputer
{
public:
    /// @param initial_altitude_m  Seeds the barometric filters. A real altimeter
    ///        is set to the field elevation before take-off for exactly this
    ///        reason; without it the altitude channel spends the first seconds of
    ///        a run ramping from zero, and a derivative of that ramp is a
    ///        fictitious climb rate of hundreds of metres per second.
    void reset(std::uint64_t seed, Scalar initial_altitude_m = 0.0F) noexcept;
    void update(const AircraftState& state, Scalar dt) noexcept;

    [[nodiscard]] Scalar indicated_airspeed_mps() const noexcept
    {
        return ias_mps_;
    }
    [[nodiscard]] Scalar barometric_altitude_m() const noexcept
    {
        return altitude_m_;
    }
    [[nodiscard]] Scalar vertical_speed_mps() const noexcept
    {
        return vertical_speed_mps_;
    }
    [[nodiscard]] Scalar heading_deg() const noexcept
    {
        return heading_deg_;
    }
    [[nodiscard]] Scalar pitch_deg() const noexcept
    {
        return pitch_deg_;
    }
    [[nodiscard]] Scalar roll_deg() const noexcept
    {
        return roll_deg_;
    }
    [[nodiscard]] Vec3 specific_force_mps2() const noexcept
    {
        return specific_force_mps2_;
    }

    [[nodiscard]] SensorQuality airspeed_quality() const noexcept
    {
        return ias_quality_;
    }
    [[nodiscard]] SensorQuality altitude_quality() const noexcept
    {
        return altitude_quality_;
    }
    [[nodiscard]] SensorQuality attitude_quality() const noexcept
    {
        return attitude_quality_;
    }

    /// True once the attitude reference has completed its alignment.
    [[nodiscard]] bool aligned() const noexcept
    {
        return alignment_s_ >= kAlignmentTimeS;
    }

private:
    /// Time the attitude reference needs before it is trusted.
    static constexpr Scalar kAlignmentTimeS = 8.0F;

    /// Filter time constants, seconds.
    static constexpr Scalar kPitotTauS         = 0.18F;
    static constexpr Scalar kBarometricTauS    = 0.40F;
    static constexpr Scalar kVerticalSpeedTauS = 0.80F;
    static constexpr Scalar kAttitudeTauS      = 0.06F;
    /// The altitude used *only* for the rate channel is filtered far harder than
    /// the altitude channel itself.
    static constexpr Scalar kAltitudeTrendTauS = 4.0F;

    /// 1 sigma noise levels.
    static constexpr Scalar kIasNoiseMps           = 0.22F;
    static constexpr Scalar kAltitudeNoiseM        = 1.8F;
    static constexpr Scalar kVerticalSpeedNoiseMps = 0.25F;
    static constexpr Scalar kAttitudeNoiseDeg      = 0.05F;

    /// Values outside these bounds are reported degraded or invalid rather than
    /// quietly passed on.
    static constexpr Scalar kMaxTrustedIasMps    = 260.0F;
    static constexpr Scalar kMinTrustedAltitudeM = -50.0F;

    /// Alignment error that decays to zero over the alignment period.
    static constexpr Scalar kInitialAlignmentErrorDeg = 2.5F;

    /// Amplitude and correlation time of the barometric calibration drift.
    static constexpr Scalar kAltitudeBiasSigmaM = 2.5F;
    static constexpr Scalar kAltitudeBiasTauS   = 120.0F;

    /// Independent generator streams, so retuning one channel's noise cannot
    /// shift another channel's sequence.
    static constexpr std::uint64_t kNoiseStream = 0x243f6a8885a308d3ULL;
    static constexpr std::uint64_t kBiasStream  = 0x13198a2e03707344ULL;

    Scalar ias_mps_{0.0F};
    Scalar altitude_m_{0.0F};
    /// Previous filtered altitude, kept so the vertical speed channel can
    /// differentiate the barometric signal instead of the truth.
    Scalar previous_altitude_m_{0.0F};
    /// Long-window filtered altitude, and its previous value. The vertical speed
    /// channel differentiates *this*, never the fast barometric signal.
    Scalar altitude_trend_m_{0.0F};
    Scalar previous_altitude_trend_m_{0.0F};
    Scalar vertical_speed_mps_{0.0F};
    Scalar heading_deg_{0.0F};
    Scalar pitch_deg_{0.0F};
    Scalar roll_deg_{0.0F};
    Vec3   specific_force_mps2_{};

    /// Slowly varying calibration bias, in metres. A constant offset would be
    /// too easy to remove downstream.
    random::ColouredNoise altitude_bias_{0U, kAltitudeBiasSigmaM, kAltitudeBiasTauS, kBiasStream};

    Scalar alignment_s_{0.0F};
    Scalar alignment_bias_deg_{0.0F};

    SensorQuality ias_quality_{SensorQuality::kInvalid};
    SensorQuality altitude_quality_{SensorQuality::kInvalid};
    SensorQuality attitude_quality_{SensorQuality::kInvalid};

    random::Pcg32 noise_rng_{0U, kNoiseStream};
    std::uint64_t seed_{0};
};

/// Engine monitor: exhaust gas temperature and the hydraulic pressures.
class EngineMonitor
{
public:
    void reset(std::uint64_t seed) noexcept;
    void update(const AircraftState& state, Scalar dt) noexcept;

    [[nodiscard]] const std::array<Scalar, kEngineCount>& temperatures_c() const noexcept
    {
        return temperature_c_;
    }
    [[nodiscard]] Scalar fuel_pressure_kpa() const noexcept
    {
        return fuel_press_kpa_;
    }
    [[nodiscard]] Scalar oil_pressure_kpa() const noexcept
    {
        return oil_press_kpa_;
    }
    [[nodiscard]] SensorQuality engine_quality() const noexcept
    {
        return engine_quality_;
    }

    /// Temperature above which an engine is reported degraded.
    static constexpr Scalar kRedlineTemperatureC = 820.0F;
    /// Engine to engine spread above which the channel is degraded. Two engines
    /// running differently is a real fault indication, not noise.
    static constexpr Scalar kEngineSpreadDegradedC = 40.0F;
    /// Ambient temperature used as the cold soak reference, degrees Celsius.
    static constexpr Scalar kAmbientTemperatureC = 15.0F;
    /// Throttle below which the engines are idling rather than producing.
    static constexpr Scalar kIdleQualityThreshold = 0.05F;

private:
    /// Thermal lag of the exhaust gas temperature, seconds. Large on purpose:
    /// this is the observable the model exists to produce.
    static constexpr Scalar kThermalTauS       = 9.0F;
    static constexpr Scalar kMaxTemperatureC   = 780.0F;
    static constexpr Scalar kPressureTauS      = 0.55F;
    static constexpr Scalar kTemperatureNoiseC = 1.2F;

    static constexpr Scalar kMaxFuelPressureKpa = kSeaLevelPressureKpa;
    static constexpr Scalar kMaxOilPressureKpa  = 420.0F;

    /// Independent noise stream, see `AirDataComputer::kNoiseStream`.
    static constexpr std::uint64_t kNoiseStream = 0xa4093822299f31d0ULL;

    std::array<Scalar, kEngineCount> temperature_c_{};
    std::array<Scalar, kEngineCount> target_temperature_c_{};
    Scalar                           fuel_press_kpa_{0.0F};
    Scalar                           oil_press_kpa_{0.0F};
    SensorQuality                    engine_quality_{SensorQuality::kInvalid};

    random::Pcg32 noise_rng_{0U, kNoiseStream};
    std::uint64_t seed_{0};
};

/// Owns both sensor chains and produces one `SensorSample` per simulation step.
class SensorSuite
{
public:
    void reset(std::uint64_t seed, Scalar initial_altitude_m = 0.0F) noexcept;
    void update(const AircraftState& state, SensorSample& out, Scalar dt) noexcept;

    [[nodiscard]] const AirDataComputer& air_data() const noexcept
    {
        return air_data_;
    }
    [[nodiscard]] const EngineMonitor& engines() const noexcept
    {
        return engines_;
    }

private:
    AirDataComputer air_data_;
    EngineMonitor   engines_;
};

} // namespace fse