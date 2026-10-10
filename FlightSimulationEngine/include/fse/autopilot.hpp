#pragma once

/// @file autopilot.hpp
/// @brief Phase dependent guidance that turns a flight phase into control
///        surface commands.
///
/// A cascade structure is used deliberately, because that is how the real thing
/// is organised and because it is the structure that stays stable:
///
///   altitude error -> vertical speed -> flight path angle -> pitch command
///   heading error   -> bank command
///   speed error     -> throttle command
///
/// The outer loop is slow and forgiving, the inner loop is fast and tight. A
/// single gain trying to do all three at once is the usual source of an
/// oscillation that only appears in the telemetry.

#include <cstdint>

#include "fse/aircraft.hpp"
#include "fse/flight_model.hpp"
#include "fse/types.hpp"

namespace fse
{

/// Gains and limits of the guidance loops.
struct AutopilotGains
{
    // Altitude hold: metres of error per m/s of demanded vertical speed.
    Scalar altitude_to_vertical_speed{0.10F};
    Scalar max_vertical_speed_mps{12.0F};
    /// Vertical speed error to flight path angle, degrees per m/s.
    Scalar vertical_speed_to_path_deg{1.30F};
    Scalar max_path_angle_deg{15.0F};

    // Speed hold.
    Scalar speed_to_throttle{0.020F};
    Scalar throttle_integral_gain{0.010F};
    Scalar max_throttle_integral{0.60F};

    // Heading hold.
    Scalar heading_to_bank_deg{0.45F};
    Scalar max_bank_deg{30.0F};

    // Takeoff roll.
    Scalar rotate_pitch_deg{9.0F};
};

/// Computes control commands for the current phase.
class Autopilot
{
public:
    Autopilot();

    void reset() noexcept;

    /// Recomputes the setpoints for @p phase. Call once per phase change, not
    /// once per step.
    void engage(FlightPhase phase, const MissionProfile& profile) noexcept;

    /// Produces the control commands for the current step.
    ///
    /// @p parameters is required because the altitude loop is written as inverse
    /// dynamics: the commanded angle of attack is computed from the lift the
    /// aircraft needs right now, which is a function of its wing and its weight.
    [[nodiscard]] ControlInputs update(const AircraftState& state, const MissionProfile& profile,
                                       const AircraftParameters& parameters, Scalar dt) noexcept;

    [[nodiscard]] const ControlInputs& commands() const noexcept
    {
        return commands_;
    }
    [[nodiscard]] FlightPhase engaged_phase() const noexcept
    {
        return phase_;
    }

    /// Diagnostic: what the autopilot currently demands vertically.
    [[nodiscard]] Scalar commanded_vertical_speed_mps() const noexcept
    {
        return commanded_vs_mps_;
    }
    [[nodiscard]] Scalar commanded_altitude_m() const noexcept
    {
        return commanded_altitude_m_;
    }

    [[nodiscard]] AutopilotGains& gains() noexcept
    {
        return gains_;
    }
    [[nodiscard]] const AutopilotGains& gains() const noexcept
    {
        return gains_;
    }

    /// Angle of attack at which the wing carries exactly the current weight,
    /// degrees. This is the feed-forward term of the altitude loop.
    ///
    /// Public because it depends on nothing but the state and the airframe, so
    /// it is not really a member of the guidance at all: it is the aerodynamics
    /// question "what angle does this wing need right now". Tests use it to
    /// check that inverse dynamics directly, which is the only way to tell a
    /// correct trim term from one that happens to give a flyable trace.
    [[nodiscard]] static Scalar trim_alpha_deg(const AircraftState&      state,
                                               const AircraftParameters& parameters,
                                               const MissionProfile&     profile) noexcept;

private:
    /// The two guidance laws. Deliberately private: they are only reachable
    /// through update(), which chooses between them on the aircraft's height.
    /// Making them public would let a caller bypass that choice, and bypassing
    /// it is precisely the bug the routing exists to prevent -- an air loop
    /// commanding pitch while the aircraft is still on the runway winds the
    /// throttle integral up against a command that is never executed.
    [[nodiscard]] ControlInputs update_ground(const AircraftState&      state,
                                              const MissionProfile&     profile,
                                              const AircraftParameters& parameters,
                                              Scalar                    dt) noexcept;
    [[nodiscard]] ControlInputs update_air(const AircraftState&      state,
                                           const MissionProfile&     profile,
                                           const AircraftParameters& parameters,
                                           Scalar                    dt) noexcept;

private:
    /// Cruise inserts a banked turn partway through, so the telemetry shows a
    /// coordinated turn and the resulting load factor.
    [[nodiscard]] Scalar cruise_target_heading_deg(const MissionProfile& profile) const noexcept;

    AutopilotGains gains_{};
    ControlInputs  commands_{};
    FlightPhase    phase_{FlightPhase::kPreflight};

    Scalar commanded_altitude_m_{0.0F};
    Scalar commanded_vs_mps_{0.0F};
    Scalar throttle_integral_{0.0F};
    /// Distance flown along the current cruise leg, used to time the turn.
    Scalar cruise_leg_time_s_{0.0F};
};

} // namespace fse