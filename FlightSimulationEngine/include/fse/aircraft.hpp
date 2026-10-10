#pragma once

/// @file aircraft.hpp
/// @brief Vehicle parameters and the complete state vector of the simulation.

#include <array>
#include <cmath>
#include <cstdint>

#include "fse/flight_phase.hpp"
#include "fse/types.hpp"

namespace fse
{

/// Number of engines modelled. Four is the minimum that makes an asymmetric
/// engine-out case interesting, and it matches the four temperature channels in
/// the telemetry frame.
inline constexpr std::size_t kEngineCount = 4U;

/// Phases in which the aircraft is not on the runway. Used when a scenario has to
/// start mid-flight: placing a "cruise" scenario on the ground produces a
/// simulation that never moves, because no phase after take-off commands a
/// take-off throttle.
[[nodiscard]] constexpr bool is_airborne_phase(FlightPhase phase) noexcept
{
    switch (phase)
    {
        case FlightPhase::kPreflight:
        case FlightPhase::kTaxi:
        case FlightPhase::kTakeoffRoll:
            return false;
        case FlightPhase::kClimb:
        case FlightPhase::kCruise:
        case FlightPhase::kDescent:
        case FlightPhase::kApproach:
        case FlightPhase::kLanding:
        case FlightPhase::kGoAround:
        case FlightPhase::kFailure:
            return true;
    }
    return false;
}

/// Indices into `AircraftState::engine_temp_c`.
inline constexpr std::size_t kEngineLeftOuter  = 0U;
inline constexpr std::size_t kEngineLeftInner  = 1U;
inline constexpr std::size_t kEngineRightInner = 2U;
inline constexpr std::size_t kEngineRightOuter = 3U;

/// Physical constants of the airframe. Not state: this is fixed for the
/// vehicle type and belongs in configuration rather than in the state vector.
///
/// The numbers describe a light twin engine trainer with a 2900 kg maximum
/// take-off mass. They are chosen to be mutually consistent rather than to copy
/// any particular type, and the consistency is what the unit tests check: the
/// stall speed must sit below the approach speed, and the thrust to weight
/// ratio must be high enough to accelerate to rotation on the available runway
/// but low enough that the aircraft does not accelerate uncontrollably in
/// level flight.
struct AircraftParameters
{
    Scalar mass_kg{2900.0F};
    Scalar wing_area_m2{16.2F};
    Scalar wing_span_m{11.8F};

    // Lift curve: CL = cl0 + cl_per_flap_deg * flap + cl_alpha_per_deg * alpha,
    // clipped to a flap dependent limit.
    Scalar cl0{0.25F};
    Scalar cl_alpha_per_deg{0.10F};
    Scalar cl_max{1.45F};
    Scalar cl_min{-1.10F};
    /// Flaps add lift as well as drag. Without this the aircraft cannot fly the
    /// approach and landing speeds, because the unassisted stall speed sits
    /// above them.
    Scalar cl_per_flap_deg{0.035F};
    /// Angle at which the wing stalls, in degrees of angle of attack.
    Scalar stall_alpha_deg{15.0F};

    // Drag polar: CD = cd0 + induced_drag_factor * CL^2 + cd_per_flap_deg * flap.
    Scalar cd0{0.045F};
    Scalar induced_drag_factor{0.055F};
    Scalar cd_per_flap_deg{0.012F};
    /// Extra CD per degree of angle of attack beyond the stall.
    Scalar post_stall_cd_per_deg{0.06F};

    Scalar max_thrust_per_engine_n{4500.0F};
    /// First order spool lag of the engines, seconds.
    Scalar engine_spool_tau_s{1.8F};

    // Fuel burn drives the mission profile.
    Scalar fuel_flow_kg_per_s{0.16F};

    // Ground handling.
    Scalar rolling_resistance{0.025F};
    /// Rotation speed, after which pitch authority becomes useful. Kept above
    /// the clean stall speed so the aircraft is not leaving the runway already
    /// stalled; `AircraftParameters` tests assert the relationship.
    Scalar rotate_ias_mps{58.0F};
    Scalar max_ground_speed_mps{90.0F};

    // Actuator limits.
    Scalar max_pitch_deg{25.0F};
    Scalar max_roll_deg{55.0F};
    Scalar max_pitch_rate_dps{12.0F};
    Scalar max_roll_rate_dps{30.0F};
    Scalar max_airspeed_mps{300.0F};

    [[nodiscard]] static constexpr AircraftParameters default_light_aircraft() noexcept
    {
        return {};
    }

    /// Weight, N. Convenience for the flight and test code.
    [[nodiscard]] constexpr Scalar weight_n() const noexcept
    {
        return mass_kg * kGravity;
    }

    /// Stall speed as indicated airspeed at sea level, m/s.
    ///
    /// Used by the tests as a consistency check on the whole parameter set: the
    /// approach and touchdown speeds in `MissionProfile` have to be above it, or
    /// the aircraft cannot fly the approach it is being asked to fly.
    [[nodiscard]] Scalar stall_ias_sea_level_mps() const noexcept
    {
        // 1/2 rho V^2 S CL_max = W, solved for V. At sea level indicated and
        // true airspeed coincide, so no density correction is needed.
        const Scalar dynamic_pressure_needed =
            (2.0F * weight_n()) / (kSeaLevelDensity * wing_area_m2 * cl_max);
        return std::sqrt((2.0F * dynamic_pressure_needed) / kSeaLevelDensity);
    }
};

/// Named phases of the simulated mission, in seconds. The profile makes the
/// flight deterministic and demonstrable: the same seed always flies the same
/// route with the same faults.
struct MissionProfile
{
    Scalar taxi_hold_s{3.0F};
    /// Ground speed the aircraft must reach before it is cleared for departure.
    Scalar taxi_accel_mps{5.0F};
    /// Throttle held while taxiing.
    Scalar taxi_throttle{0.22F};

    Scalar climb_ias_mps{62.0F};
    Scalar climb_altitude_m{900.0F};
    Scalar climb_target_ias_mps{72.0F};
    Scalar climb_flap_deg{0.0F};

    Scalar cruise_altitude_m{900.0F};
    Scalar cruise_ias_mps{105.0F};
    Scalar cruise_heading_deg{90.0F};
    Scalar cruise_duration_s{40.0F};
    /// Bank angle for a 30 degree turn inserted partway through cruise, so the
    /// telemetry shows a coordinated turn with the expected load factor.
    Scalar cruise_turn_bank_deg{30.0F};
    Scalar cruise_turn_duration_s{10.0F};

    Scalar descent_ias_mps{95.0F};

    /// The approach gate: the altitude the descent continues down to, and the
    /// altitude the approach then holds while the aircraft decelerates and
    /// configures. There is deliberately only one of these. A descent that
    /// levelled off above the gate would leave the aircraft parked between two
    /// phases for ever, which is exactly the sort of gap a phase table must not
    /// contain.
    Scalar approach_altitude_m{450.0F};
    Scalar approach_ias_mps{62.0F};
    Scalar approach_flap_deg{15.0F};

    Scalar landing_flap_deg{30.0F};
    Scalar flare_altitude_m{12.0F};
    /// Touchdown is flown at full flap, so this has to sit above the *flapped*
    /// stall speed rather than the clean one. `AircraftParameters` tests assert
    /// the relationship, which is why the value cannot simply be "looks fast".
    Scalar touchdown_ias_mps{47.0F};
    /// Touchdown faster than this is a missed landing and triggers a go-around.
    Scalar max_touchdown_ias_mps{58.0F};

    Scalar go_around_altitude_m{700.0F};
};

/// Complete state of the vehicle. Plain data, no invariants beyond "all finite".
struct AircraftState
{
    // --- Kinematics, world frame (X east, Y up, Z north) --------------------
    Vec3 position_m{};
    Vec3 velocity_mps{};

    /// Orientation, degrees. Heading is `[0, 360)`, pitch `[-90, 90]`, roll
    /// `(-180, 180]`.
    Scalar heading_deg{0.0F};
    Scalar pitch_deg{0.0F};
    Scalar roll_deg{0.0F};

    /// Body rates, degrees per second.
    Scalar pitch_rate_dps{0.0F};
    Scalar roll_rate_dps{0.0F};
    Scalar yaw_rate_dps{0.0F};

    // --- Propulsion and configuration ---------------------------------------
    Scalar throttle{0.0F};
    Scalar flap_deg{0.0F};
    Scalar fuel_kg{1200.0F};
    /// Commanded, before the actuator lag.
    Scalar throttle_command{0.0F};

    // --- Derived, refreshed every step --------------------------------------
    /// Indicated airspeed: what the pitot tube would read.
    Scalar ias_mps{0.0F};
    /// True airspeed over the ground.
    Scalar tas_mps{0.0F};
    /// Speed over the ground, m/s. Equals `tas_mps` in still air and, crucially,
    /// stays zero for a parked aircraft in wind — which is the quantity a ground
    /// crew cares about and the one the taxi guard uses.
    Scalar ground_speed_mps{0.0F};
    /// Angle of attack, degrees.
    Scalar alpha_deg{0.0F};
    /// Total energy height, metres.
    Scalar energy_height_m{0.0F};
    /// Normal load factor.
    Scalar load_factor{1.0F};

    // --- Program -----------------------------------------------------------
    FlightPhase phase{FlightPhase::kPreflight};
    FaultCode   faults{FaultCode::kNone};

    /// Step index since reset. Drives the timestamp so that time never
    /// accumulates floating point drift.
    std::uint64_t step_index{0};
};

} // namespace fse
