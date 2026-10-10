#include "fse/autopilot.hpp"

#include <cmath>

namespace fse
{

namespace
{

/// Extra nose up during the flare, added on top of the landing pitch.
constexpr Scalar kFlarePitchDeg{5.0F};

/// Throttle scheduled idle during the descent and the approach.
constexpr Scalar kIdleThrottle{0.05F};

[[nodiscard]] Scalar approach_vertical_speed_for(Scalar altitude_error, Scalar max_vs,
                                                 const AutopilotGains& gains) noexcept
{
    // Asymmetric capture: descending is limited harder than climbing, because a
    // vertical speed at low altitude leaves almost no margin.
    const Scalar limit = (altitude_error < 0.0F) ? (max_vs * 0.8F) : max_vs;
    return clamp(altitude_error * gains.altitude_to_vertical_speed, -limit, limit);
}

} // namespace

Autopilot::Autopilot() = default;

void Autopilot::reset() noexcept
{
    commands_             = ControlInputs{};
    phase_                = FlightPhase::kPreflight;
    commanded_altitude_m_ = 0.0F;
    commanded_vs_mps_     = 0.0F;
    throttle_integral_    = 0.0F;
    cruise_leg_time_s_    = 0.0F;
}

void Autopilot::engage(FlightPhase phase, const MissionProfile& profile) noexcept
{
    phase_ = phase;
    // The integral only carries the throttle bias, so it has to be released on
    // every phase change: a climb bias would otherwise fight the descent.
    throttle_integral_ = 0.0F;
    cruise_leg_time_s_ = 0.0F;

    switch (phase)
    {
        case FlightPhase::kPreflight:
            commanded_altitude_m_ = 0.0F;
            commanded_vs_mps_     = 0.0F;
            break;
        case FlightPhase::kTaxi:
        case FlightPhase::kTakeoffRoll:
            commanded_altitude_m_ = 0.0F;
            commanded_vs_mps_     = 0.0F;
            break;
        case FlightPhase::kClimb:
        case FlightPhase::kGoAround:
            commanded_altitude_m_ = profile.climb_altitude_m;
            break;
        case FlightPhase::kCruise:
            commanded_altitude_m_ = profile.cruise_altitude_m;
            break;
        case FlightPhase::kDescent:
            // The descent continues to the approach gate, see the note on
            // `MissionProfile::approach_altitude_m`.
            commanded_altitude_m_ = profile.approach_altitude_m;
            break;
        case FlightPhase::kApproach:
            commanded_altitude_m_ = profile.approach_altitude_m;
            break;
        case FlightPhase::kLanding:
            commanded_altitude_m_ = 0.0F;
            break;
        case FlightPhase::kFailure:
            commanded_altitude_m_ = 0.0F;
            break;
    }
}

Scalar Autopilot::cruise_target_heading_deg(const MissionProfile& profile) const noexcept
{
    // A 30 degree bank held for a fixed time is the turn; the heading it returns
    // to afterwards is offset by twice the heading change, so the leg stays a
    // closed pattern rather than drifting.
    if (cruise_leg_time_s_ < (profile.cruise_duration_s * 0.25F))
    {
        return profile.cruise_heading_deg;
    }
    if (cruise_leg_time_s_ < (profile.cruise_duration_s * 0.25F + profile.cruise_turn_duration_s))
    {
        return wrap_degrees(profile.cruise_heading_deg + 90.0F);
    }
    return wrap_degrees(profile.cruise_heading_deg + 180.0F);
}

Scalar Autopilot::trim_alpha_deg(const AircraftState& state, const AircraftParameters& parameters,
                                 const MissionProfile& profile) noexcept
{
    (void)profile;

    // Inverse dynamics: what angle of attack does this wing need, at this speed
    // and this altitude, to carry exactly the weight?
    //
    // Using a fixed trim angle instead — 3 degrees was the first attempt — makes
    // the inner loop fight the lift it has just created: at cruise speed the wing
    // needs well under a degree, so a 3 degree trim commands more than twice the
    // lift the aircraft requires, which shows up as a climb that never settles
    // and a descent that never starts.
    const Scalar tas              = max(state.tas_mps, 1.0F);
    const Scalar rho              = Atmosphere::density(max(state.position_m.y, 0.0F));
    const Scalar dynamic_pressure = 0.5F * rho * tas * tas;

    if (dynamic_pressure <= 1.0F)
    {
        // Parked or nearly so: any answer is meaningless, return level.
        return 0.0F;
    }

    // The *achieved* flap position, not the commanded one: the actuator lags by
    // seconds and the trim has to agree with the wing that is actually fitted.
    const Scalar flap_lift = parameters.cl_per_flap_deg * clamp(state.flap_deg, 0.0F, 30.0F);

    const Scalar required_cl = parameters.weight_n() / (dynamic_pressure * parameters.wing_area_m2);

    return (required_cl - parameters.cl0 - flap_lift) / parameters.cl_alpha_per_deg;
}

ControlInputs Autopilot::update(const AircraftState& state, const MissionProfile& profile,
                                const AircraftParameters& parameters, Scalar dt) noexcept
{
    const bool on_ground = state.position_m.y <= 0.0F;

    return on_ground ? update_ground(state, profile, parameters, dt)
                     : update_air(state, profile, parameters, dt);
}

ControlInputs Autopilot::update_ground(const AircraftState& state, const MissionProfile& profile,
                                       const AircraftParameters& parameters, Scalar dt) noexcept
{
    (void)parameters;
    (void)dt;
    commands_             = ControlInputs{};
    commands_.heading_deg = profile.cruise_heading_deg;
    commands_.roll_deg    = 0.0F;
    commands_.pitch_deg   = 0.0F;
    commands_.flap_deg    = 0.0F;

    switch (phase_)
    {
        case FlightPhase::kPreflight:
            commands_.throttle = 0.0F;
            break;
        case FlightPhase::kTaxi:
            // Hold the taxi threshold rather than a fixed speed, so the guard in the
            // phase table and the throttle come from the same number.
            commands_.throttle =
                (state.ground_speed_mps < profile.taxi_accel_mps) ? profile.taxi_throttle : 0.0F;
            break;
        case FlightPhase::kTakeoffRoll:
            commands_.throttle = 1.0F;
            // Rotate at the same speed the phase table waits for, and hold the nose
            // up afterwards.
            //
            // These were two different numbers once -- the guidance used
            // `climb_ias_mps * 0.85` while the table used
            // `AircraftParameters::rotate_ias_mps`. The aircraft therefore rotated
            // about 5 m/s below the speed at which the machine moved on to climb,
            // which is below the speed at which the wing is meant to carry the
            // weight. The net vertical force was still negative, so nothing
            // happened for another few seconds and the nose-up command just sat
            // there being ignored. One number, from the parameter that is named
            // after the thing it means.
            commands_.pitch_deg =
                (state.ias_mps >= parameters.rotate_ias_mps) ? gains_.rotate_pitch_deg : 0.0F;
            break;
        case FlightPhase::kLanding:
            // Rollout: keep the nose on the centreline and take the power off.
            commands_.throttle = 0.0F;
            commands_.flap_deg = profile.landing_flap_deg;
            break;
        case FlightPhase::kClimb:
        case FlightPhase::kGoAround:
            // Reached while still on the runway. The phase table hands over at
            // rotation speed, which is the instant the nose comes up -- and the
            // aircraft needs another second or two of runway to actually leave
            // the ground. Falling through to the default here cut the throttle
            // the moment the machine reached climb, so the aircraft sat on the
            // runway at zero power, decelerated, and never flew the sortie at
            // all.
            //
            // Keep flying the take-off until the height based routing hands over
            // to the airborne law. Same commands as the ground roll, so the
            // handover is invisible rather than a step change.
            commands_.throttle = 1.0F;
            commands_.pitch_deg =
                (state.ias_mps >= parameters.rotate_ias_mps) ? gains_.rotate_pitch_deg : 0.0F;
            break;
        case FlightPhase::kFailure:
            commands_.throttle = 0.0F;
            break;
        default:
            commands_.throttle = 0.0F;
            break;
    }

    return commands_;
}

ControlInputs Autopilot::update_air(const AircraftState& state, const MissionProfile& profile,
                                    const AircraftParameters& parameters, Scalar dt) noexcept
{
    commands_             = ControlInputs{};
    commands_.heading_deg = state.heading_deg;

    // --- Target altitude and speed for the phase ---------------------------
    switch (phase_)
    {
        case FlightPhase::kClimb:
            commanded_altitude_m_ = profile.climb_altitude_m;
            break;
        case FlightPhase::kGoAround:
            commanded_altitude_m_ = profile.go_around_altitude_m;
            break;
        case FlightPhase::kCruise:
            cruise_leg_time_s_ += dt;
            commanded_altitude_m_ = profile.cruise_altitude_m;
            commands_.heading_deg = cruise_target_heading_deg(profile);
            break;
        case FlightPhase::kDescent:
            // Keep descending to the approach gate. Targeting any higher altitude
            // would leave the aircraft holding it and never satisfying the guard
            // that hands over to the approach.
            commanded_altitude_m_ = profile.approach_altitude_m;
            break;
        case FlightPhase::kApproach:
            // The approach flies a glide path down to the flare. Holding the gate
            // altitude here would satisfy the handover guard that *started* this
            // phase and never satisfy the one that ends it.
            commanded_altitude_m_ = profile.flare_altitude_m;
            commands_.flap_deg    = profile.approach_flap_deg;
            break;
        case FlightPhase::kLanding:
            commands_.flap_deg = profile.landing_flap_deg;
            break;
        default:
            break;
    }

    // --- Outer loop: altitude -> vertical speed -----------------------------
    //
    // `turn_bank_bias` is the inserted cruise turn, below. It is declared here so
    // the roll command at the end has one place to add the two contributions up.
    Scalar turn_bank_bias = 0.0F;

    const Scalar altitude_error = commanded_altitude_m_ - state.position_m.y;
    const Scalar target_vs =
        approach_vertical_speed_for(altitude_error, gains_.max_vertical_speed_mps, gains_);
    commanded_vs_mps_ = target_vs;

    // --- Inner loop: vertical speed -> flight path angle -> pitch -----------
    const Scalar vs_error   = target_vs - state.velocity_mps.y;
    const Scalar path_angle = clamp(vs_error * gains_.vertical_speed_to_path_deg,
                                    -gains_.max_path_angle_deg, gains_.max_path_angle_deg);

    // Feed-forward: the commanded pitch is the commanded path angle plus the
    // angle of attack the wing needs to hold the aircraft up. Without this the
    // loop relies on the integral term to discover the trim angle, which it does
    // slowly and with overshoot.
    Scalar pitch_command = path_angle + trim_alpha_deg(state, parameters, profile);

    Scalar target_ias    = profile.cruise_ias_mps;
    Scalar throttle_bias = 0.35F;

    switch (phase_)
    {
        case FlightPhase::kClimb:
            target_ias    = profile.climb_target_ias_mps;
            throttle_bias = 0.65F;
            break;
        case FlightPhase::kGoAround:
            target_ias    = profile.climb_target_ias_mps;
            throttle_bias = 1.0F;
            break;
        case FlightPhase::kCruise:
            target_ias    = profile.cruise_ias_mps;
            throttle_bias = 0.30F;
            // Bank through the inserted turn. This is a bias the heading loop
            // below corrects, not a replacement for it.
            if (cruise_leg_time_s_ >= (profile.cruise_duration_s * 0.25F) &&
                cruise_leg_time_s_ <
                    (profile.cruise_duration_s * 0.25F + profile.cruise_turn_duration_s))
            {
                turn_bank_bias = profile.cruise_turn_bank_deg;
            }
            break;
        case FlightPhase::kDescent:
            target_ias    = profile.descent_ias_mps;
            throttle_bias = kIdleThrottle;
            break;
        case FlightPhase::kApproach:
            target_ias    = profile.approach_ias_mps;
            throttle_bias = 0.28F;
            break;
        case FlightPhase::kLanding:
            target_ias    = profile.touchdown_ias_mps;
            throttle_bias = 0.18F;
            // The flare: pitch up as the ground comes up, and close the throttle.
            if (state.position_m.y <= profile.flare_altitude_m)
            {
                pitch_command += kFlarePitchDeg;
                throttle_bias = 0.08F;
            }
            break;
        case FlightPhase::kFailure:
            // No power, wings level, best effort to stay upright.
            target_ias    = profile.cruise_ias_mps;
            throttle_bias = 0.0F;
            break;
        default:
            break;
    }

    // --- Speed loop: airspeed error -> throttle ----------------------------
    const Scalar speed_error = target_ias - state.ias_mps;
    throttle_integral_ += speed_error * gains_.throttle_integral_gain * dt;
    throttle_integral_ =
        clamp(throttle_integral_, -gains_.max_throttle_integral, gains_.max_throttle_integral);

    const Scalar throttle = clamp(
        throttle_bias + (speed_error * gains_.speed_to_throttle) + throttle_integral_, 0.0F, 1.0F);

    // --- Heading loop: heading error -> bank --------------------------------
    //
    // The commanded heading is initialised to the current heading, so outside
    // cruise this loop has nothing to correct and the aircraft simply holds
    // whatever course it arrived on -- which is the intent: a climb should not
    // start turning itself towards somewhere.
    //
    // In cruise the command is the *leg* heading rather than the current one,
    // and that is what makes this loop the only thing that brings the aircraft
    // back on course after the inserted turn. It used to be skipped in cruise
    // altogether, which left `heading_to_bank_deg` and `max_bank_deg`
    // unreachable: the aircraft banked for the turn, changed heading by twice
    // the bank angle, and then drifted off the leg for the rest of the cruise
    // with nothing acting on the error at all.
    //
    // The turn is a bias that the loop corrects rather than a value the loop
    // overwrites, so the coordinated turn the telemetry is meant to show still
    // happens and the heading error decays through it instead of fighting it.
    const Scalar heading_error = angle_difference_deg(commands_.heading_deg, state.heading_deg);
    const Scalar hold_bank = clamp(heading_error * gains_.heading_to_bank_deg, -gains_.max_bank_deg,
                                   gains_.max_bank_deg);

    commands_.roll_deg =
        clamp(turn_bank_bias + hold_bank, -gains_.max_bank_deg, gains_.max_bank_deg);

    commands_.pitch_deg = clamp(pitch_command, -25.0F, 25.0F);
    commands_.throttle  = throttle;

    return commands_;
}

} // namespace fse