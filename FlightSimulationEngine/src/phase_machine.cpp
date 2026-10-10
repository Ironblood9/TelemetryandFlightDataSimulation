#include "fse/phase_machine.hpp"

#include <cmath>

namespace fse
{

namespace
{

/// Vertical speed the climb phase must settle inside before it reports capture.
constexpr Scalar kClimbVerticalToleranceMps = 2.5F;

/// Altitude error the climb phase must be inside before it reports capture.
constexpr Scalar kClimbAltitudeToleranceM = 25.0F;

/// Capture band around the approach gate.
///
/// A bare `altitude <= gate` test can never be satisfied when the descent is
/// holding exactly the gate altitude: the altitude loop converges onto its
/// setpoint from one side and the guard then waits for the other side of a
/// threshold the aircraft is already sitting on. The handover therefore happens
/// on entry into the capture band, which is what a real approach transition is.
constexpr Scalar kApproachCaptureToleranceM = 15.0F;

/// The flare begins slightly above the nominal flare height, for the same reason
/// as the approach gate: the approach tracks the flare height as its altitude
/// setpoint and would otherwise never cross a bare threshold sitting on it.
constexpr Scalar kFlareCaptureToleranceM = 8.0F;

/// Below this indicated airspeed a climb is considered to have lost the margin
/// and must be abandoned.
constexpr Scalar kClimbMinimumIasMps = 45.0F;

/// Time the aircraft must stay stopped before rollout counts as complete.
constexpr Scalar kRolloutStopS = 2.0F;

constexpr Scalar kTouchdownHeightM = 1.0F;

[[nodiscard]] constexpr Scalar altitude_agl(const AircraftState& state) noexcept
{
    return state.position_m.y;
}

} // namespace

// ---------------------------------------------------------------------------
// Guards
// ---------------------------------------------------------------------------

bool guard_checklist_complete(const PhaseContext& context) noexcept
{
    return context.phase_elapsed_s >= context.profile.taxi_hold_s;
}

bool guard_taxi_complete(const PhaseContext& context) noexcept
{
    // Ground speed, not airspeed. A parked aircraft in a tailwind has a non zero
    // indicated airspeed while standing still, and an airspeed based guard would
    // clear it for departure before it had moved.
    return context.state.ground_speed_mps >= context.profile.taxi_accel_mps;
}

bool guard_rotation_speed(const PhaseContext& context) noexcept
{
    return context.state.ias_mps >= context.parameters.rotate_ias_mps;
}

bool guard_climb_complete(const PhaseContext& context) noexcept
{
    const Scalar altitude_error = context.state.position_m.y - context.climb_target_altitude_m;
    return std::abs(altitude_error) <= kClimbAltitudeToleranceM &&
           std::abs(context.state.velocity_mps.y) <= kClimbVerticalToleranceMps;
}

bool guard_climb_airspeed_lost(const PhaseContext& context) noexcept
{
    return context.state.ias_mps < kClimbMinimumIasMps;
}

bool guard_cruise_complete(const PhaseContext& context) noexcept
{
    return context.phase_elapsed_s >= context.profile.cruise_duration_s;
}

bool guard_descent_complete(const PhaseContext& context) noexcept
{
    const Scalar gate = context.profile.approach_altitude_m + kApproachCaptureToleranceM;
    return altitude_agl(context.state) <= gate;
}

bool guard_flare_entry(const PhaseContext& context) noexcept
{
    const Scalar flare = context.profile.flare_altitude_m + kFlareCaptureToleranceM;
    return altitude_agl(context.state) <= flare;
}

bool guard_touchdown_ok(const PhaseContext& context) noexcept
{
    const bool on_ground   = altitude_agl(context.state) <= kTouchdownHeightM;
    const bool slow_enough = context.state.ias_mps <= context.profile.max_touchdown_ias_mps;
    return on_ground && slow_enough && context.phase_elapsed_s >= kRolloutStopS;
}

bool guard_touchdown_missed(const PhaseContext& context) noexcept
{
    return altitude_agl(context.state) <= kTouchdownHeightM &&
           context.state.ias_mps > context.profile.max_touchdown_ias_mps;
}

bool guard_go_around_complete(const PhaseContext& context) noexcept
{
    return altitude_agl(context.state) >= context.profile.go_around_altitude_m;
}

// ---------------------------------------------------------------------------
// PhaseMachine
// ---------------------------------------------------------------------------

void PhaseMachine::reset(FlightPhase initial_phase) noexcept
{
    phase_            = initial_phase;
    elapsed_s_        = 0.0F;
    transition_count_ = 0U;
    last_transition_  = nullptr;
    last_reason_      = nullptr;
}

void PhaseMachine::force(FlightPhase phase) noexcept
{
    phase_           = phase;
    elapsed_s_       = 0.0F;
    last_transition_ = nullptr;
    last_reason_     = kOperatorReason;
}

const PhaseTransition* PhaseMachine::find_transition(const AircraftState&      state,
                                                     const AircraftParameters& parameters,
                                                     const MissionProfile&     profile,
                                                     Scalar phase_elapsed_s) const noexcept
{
    const PhaseContext context{state, parameters, profile, phase_elapsed_s,
                               climb_target_altitude_m_};

    for (const auto& candidate : kPhaseTransitions)
    {
        if (candidate.from != phase_)
        {
            continue;
        }
        if (candidate.guard(context))
        {
            return &candidate;
        }
    }
    return nullptr;
}

void PhaseMachine::update(Scalar dt, const AircraftState& state,
                          const AircraftParameters& parameters,
                          const MissionProfile&     profile) noexcept
{
    if (dt > 0.0F)
    {
        elapsed_s_ += dt;
    }

    // A latched fault overrides every schedule. This is checked before the
    // table so that a failure can never be delayed by a phase whose guard is
    // already true.
    if (state.faults != FaultCode::kNone && phase_ != FlightPhase::kFailure)
    {
        phase_           = FlightPhase::kFailure;
        elapsed_s_       = 0.0F;
        last_transition_ = nullptr;
        last_reason_     = kFaultReason;
        transition_count_ += 1U;
        return;
    }

    if (phase_ == FlightPhase::kFailure)
    {
        return;
    }

    const PhaseTransition* transition = find_transition(state, parameters, profile, elapsed_s_);
    if (transition == nullptr)
    {
        return;
    }

    phase_           = transition->to;
    elapsed_s_       = 0.0F;
    last_transition_ = transition;
    last_reason_     = transition->reason;
    transition_count_ += 1U;
}

} // namespace fse
