#pragma once

/// @file phase_machine.hpp
/// @brief Data driven flight phase transitions.

#include <array>
#include <cstdint>

#include "fse/aircraft.hpp"
#include "fse/flight_phase.hpp"
#include "fse/types.hpp"

namespace fse
{

/// Everything a guard is allowed to look at.
///
/// Guards are plain function pointers over this context rather than members or
/// `std::function`: they are pure, they are listed in one table, and they can be
/// called directly from a test without constructing a simulation.
struct PhaseContext
{
    const AircraftState&      state;
    const AircraftParameters& parameters;
    const MissionProfile&     profile;
    /// Seconds spent in the current phase.
    Scalar phase_elapsed_s{0.0F};
    /// Altitude the climb phase is currently targeting. Kept separate from the
    /// profile because a mission may climb in stages.
    Scalar climb_target_altitude_m{0.0F};
};

using PhaseGuard = bool (*)(const PhaseContext& context);

/// Reasons reported for transitions that are not table entries. They live
/// beside the table so the phase trace never has to guess why the phase changed.
inline constexpr const char* kFaultReason    = "fault latched";
inline constexpr const char* kOperatorReason = "operator command";

/// One legal transition.
struct PhaseTransition
{
    FlightPhase from;
    FlightPhase to;
    PhaseGuard  guard;
    /// Human readable reason, printed in the phase trace. Having the string in
    /// the table means the trace cannot drift away from the logic.
    const char* reason;
};

// --- Guards ----------------------------------------------------------------

/// Preflight is complete once the ground crew checklist has run for long enough.
[[nodiscard]] bool guard_checklist_complete(const PhaseContext& context) noexcept;

/// Taxi speed reached and clearance given.
[[nodiscard]] bool guard_taxi_complete(const PhaseContext& context) noexcept;

/// Rotation speed reached, so pitch authority is useful.
[[nodiscard]] bool guard_rotation_speed(const PhaseContext& context) noexcept;

/// Climb has captured the target altitude and the vertical speed is settling.
[[nodiscard]] bool guard_climb_complete(const PhaseContext& context) noexcept;

/// The climb lost airspeed below a safe margin, so the go-around is flown.
[[nodiscard]] bool guard_climb_airspeed_lost(const PhaseContext& context) noexcept;

/// The commanded cruise time has elapsed.
[[nodiscard]] bool guard_cruise_complete(const PhaseContext& context) noexcept;

/// Descended to the approach gate altitude.
[[nodiscard]] bool guard_descent_complete(const PhaseContext& context) noexcept;

/// Short enough to begin the flare.
[[nodiscard]] bool guard_flare_entry(const PhaseContext& context) noexcept;

/// Touched down within the speed limits and stopped on the runway.
[[nodiscard]] bool guard_touchdown_ok(const PhaseContext& context) noexcept;

/// Touched down too fast, or hit before the flare.
[[nodiscard]] bool guard_touchdown_missed(const PhaseContext& context) noexcept;

/// Back at a safe altitude after a go-around.
[[nodiscard]] bool guard_go_around_complete(const PhaseContext& context) noexcept;

/// The transition table, in evaluation order.
///
/// Order matters: the first entry whose `from` matches the current phase and
/// whose guard passes wins. `kFailure` is deliberately absent — it is entered
/// from any phase when a fault is latched, which is a condition rather than a
/// scheduled transition.
inline constexpr std::array<PhaseTransition, 11> kPhaseTransitions{{
    {FlightPhase::kPreflight, FlightPhase::kTaxi, guard_checklist_complete, "checklist complete"},
    {FlightPhase::kTaxi, FlightPhase::kTakeoffRoll, guard_taxi_complete,
     "taxi speed reached, cleared for departure"},
    {FlightPhase::kTakeoffRoll, FlightPhase::kClimb, guard_rotation_speed,
     "rotation speed reached"},
    {FlightPhase::kClimb, FlightPhase::kCruise, guard_climb_complete, "climb altitude captured"},
    {FlightPhase::kClimb, FlightPhase::kGoAround, guard_climb_airspeed_lost,
     "airspeed margin lost during climb"},
    {FlightPhase::kCruise, FlightPhase::kDescent, guard_cruise_complete, "cruise time elapsed"},
    {FlightPhase::kDescent, FlightPhase::kApproach, guard_descent_complete,
     "approach gate reached"},
    {FlightPhase::kApproach, FlightPhase::kLanding, guard_flare_entry, "flare entry"},
    {FlightPhase::kLanding, FlightPhase::kTaxi, guard_touchdown_ok, "touchdown within limits"},
    {FlightPhase::kLanding, FlightPhase::kGoAround, guard_touchdown_missed,
     "touchdown outside limits"},
    {FlightPhase::kGoAround, FlightPhase::kClimb, guard_go_around_complete,
     "go-around altitude reached"},
}};

/// Owns the current flight phase and advances it according to the table.
class PhaseMachine
{
public:
    PhaseMachine() = default;

    void reset(FlightPhase initial_phase = FlightPhase::kPreflight) noexcept;

    /// Advances the elapsed timer and applies at most one transition.
    void update(Scalar dt, const AircraftState& state, const AircraftParameters& parameters,
                const MissionProfile& profile) noexcept;

    /// Operator override, used by the control channel. Enters @p phase directly
    /// and resets the elapsed timer, because the guards that follow are only
    /// meaningful relative to the phase that started them.
    void force(FlightPhase phase) noexcept;

    [[nodiscard]] FlightPhase phase() const noexcept
    {
        return phase_;
    }
    [[nodiscard]] Scalar elapsed_s() const noexcept
    {
        return elapsed_s_;
    }
    [[nodiscard]] std::uint32_t transition_count() const noexcept
    {
        return transition_count_;
    }

    /// The transition that produced the current phase, or `nullptr` if the
    /// machine has not moved since `reset()`. Tests use it to assert *why* a
    /// transition happened, not just that it did.
    [[nodiscard]] const PhaseTransition* last_transition() const noexcept
    {
        return last_transition_;
    }

    /// Human readable reason for the current phase. Covers the paths that are
    /// not table entries, such as a latched fault or an operator override, so
    /// the phase trace never has to guess.
    [[nodiscard]] const char* transition_reason() const noexcept
    {
        return last_reason_;
    }

    /// Altitude the climb phase is currently targeting.
    void set_climb_target_altitude_m(Scalar altitude_m) noexcept
    {
        climb_target_altitude_m_ = altitude_m;
    }
    [[nodiscard]] Scalar climb_target_altitude_m() const noexcept
    {
        return climb_target_altitude_m_;
    }

    /// True while the machine is held in the failure phase.
    [[nodiscard]] bool in_failure_phase() const noexcept
    {
        return phase_ == FlightPhase::kFailure;
    }

private:
    [[nodiscard]] const PhaseTransition* find_transition(const AircraftState&      state,
                                                         const AircraftParameters& parameters,
                                                         const MissionProfile&     profile,
                                                         Scalar phase_elapsed_s) const noexcept;

    FlightPhase            phase_{FlightPhase::kPreflight};
    Scalar                 elapsed_s_{0.0F};
    Scalar                 climb_target_altitude_m_{0.0F};
    std::uint32_t          transition_count_{0};
    const PhaseTransition* last_transition_{nullptr};
    const char*            last_reason_{nullptr};
};

} // namespace fse
