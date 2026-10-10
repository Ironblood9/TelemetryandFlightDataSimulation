// ---------------------------------------------------------------------------
// Autopilot tests.
//
// The guidance is the part of the engine most likely to be *tuned*, and tuning
// without a regression net is how a flight that used to reach 900 m starts
// reaching 780 m. So the assertions here are about the shape of the response --
// does it stay inside its limits, does it unwind, does it settle -- rather than
// about particular output values.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>

#include "fse/autopilot.hpp"

namespace fse
{
namespace
{

const AircraftParameters kParameters{};
const MissionProfile     kProfile{};

/// An aircraft in cruise flight at the profile's cruise altitude.
[[nodiscard]] AircraftState cruise_state()
{
    AircraftState state{};
    state.position_m.y     = kProfile.cruise_altitude_m;
    state.ias_mps          = kProfile.cruise_ias_mps;
    state.tas_mps          = kProfile.cruise_ias_mps;
    state.ground_speed_mps = kProfile.cruise_ias_mps;
    state.heading_deg      = kProfile.cruise_heading_deg;
    state.phase            = FlightPhase::kCruise;
    return state;
}

/// An aircraft parked on the runway.
[[nodiscard]] AircraftState ground_state(FlightPhase phase)
{
    AircraftState state{};
    state.phase = phase;
    return state;
}

void run(Autopilot& autopilot, const AircraftState& state, Scalar seconds, Scalar dt = 0.01F)
{
    const auto steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i)
    {
        static_cast<void>(autopilot.update(state, kProfile, kParameters, dt));
    }
}

// --- Trim --------------------------------------------------------------------

TEST(TrimAlpha, SolvesForTheLiftCoefficientLevelFlightNeeds)
{
    // Inverse dynamics, not a lookup: the trim angle of attack is whatever the
    // wing needs to carry the weight at the commanded speed, solved from the
    // lift curve rather than stored as a constant.
    const AircraftState state    = cruise_state();
    const Scalar        required = Autopilot::trim_alpha_deg(state, kParameters, kProfile);

    const Scalar flap_lift = kParameters.cl_per_flap_deg * state.flap_deg;
    const Scalar cl = kParameters.cl0 + (kParameters.cl_alpha_per_deg * required) + flap_lift;

    const Scalar dynamic_pressure =
        0.5F * Atmosphere::density(state.position_m.y) * state.tas_mps * state.tas_mps;
    const Scalar cl_for_level_flight =
        kParameters.weight_n() / (dynamic_pressure * kParameters.wing_area_m2);

    // This is the property that matters: the trim angle produces exactly the
    // lift coefficient that balances the weight at this speed and altitude.
    EXPECT_NEAR(cl, cl_for_level_flight, 1.0e-3F);
}

TEST(TrimAlpha, SlowFlightNeedsMoreAlphaThanFastFlight)
{
    AircraftState slow = cruise_state();
    slow.tas_mps       = kProfile.cruise_ias_mps * 0.7F;

    AircraftState fast = cruise_state();
    fast.tas_mps       = kProfile.cruise_ias_mps * 1.3F;

    EXPECT_GT(Autopilot::trim_alpha_deg(slow, kParameters, kProfile),
              Autopilot::trim_alpha_deg(fast, kParameters, kProfile))
        << "the required angle of attack is a function of speed";
}

TEST(TrimAlpha, ParkedAircraftGetsLevelFlightRatherThanNaN)
{
    // The dynamic pressure is zero, so the naive solution divides by zero. A NaN
    // here would propagate into the commanded pitch and take the whole
    // simulation with it, silently.
    const Scalar alpha =
        Autopilot::trim_alpha_deg(ground_state(FlightPhase::kPreflight), kParameters, kProfile);

    EXPECT_TRUE(std::isfinite(alpha));
    EXPECT_FLOAT_EQ(alpha, 0.0F);
}

TEST(TrimAlpha, IsAlwaysFiniteAcrossTheUsableSpeedRange)
{
    for (const Scalar speed_fraction : {0.1F, 0.4F, 0.6F, 0.8F, 1.0F, 1.2F, 1.5F, 2.0F})
    {
        AircraftState state = cruise_state();
        state.tas_mps       = kProfile.cruise_ias_mps * speed_fraction;

        const Scalar alpha = Autopilot::trim_alpha_deg(state, kParameters, kProfile);
        EXPECT_TRUE(std::isfinite(alpha))
            << "trim alpha at " << static_cast<double>(speed_fraction) << " of cruise speed";
    }
}

TEST(TrimAlpha, AccountsForTheFlapsActuallyFitted)
{
    // The actuator lags by seconds, so the trim has to agree with the wing that
    // is fitted rather than the one that has been commanded. With flaps out the
    // wing makes more lift at the same angle, so the required angle is lower.
    AircraftState clean = cruise_state();
    clean.flap_deg      = 0.0F;

    AircraftState flapped = cruise_state();
    flapped.flap_deg      = kProfile.approach_flap_deg;

    EXPECT_LT(Autopilot::trim_alpha_deg(flapped, kParameters, kProfile),
              Autopilot::trim_alpha_deg(clean, kParameters, kProfile));
}

// --- Ground phases ----------------------------------------------------------
//
// update_ground and update_air are private on purpose, so these go through
// update(). That is the more valuable test anyway: it exercises the routing
// decision as well as the ground law, which is what "the autopilot commanded a
// pitch while the aircraft was still on the runway" actually means.

TEST(AutopilotGround, RoutesToTheGroundLawWhileOnTheRunway)
{
    // The discriminator is height, not the phase field. A state that claims to
    // be in cruise but is sitting at zero altitude must still get the ground
    // law, or a scenario that starts on the runway with a cruise phase
    // commanded would pitch into the ground.
    Autopilot     autopilot;
    AircraftState on_the_ground    = ground_state(FlightPhase::kCruise);
    on_the_ground.ground_speed_mps = 0.0F;
    autopilot.engage(FlightPhase::kCruise, kProfile);

    const auto commands = autopilot.update(on_the_ground, kProfile, kParameters, 0.01F);

    // Cruise has no ground law of its own, so it falls through to the default:
    // throttle closed and no control input.
    EXPECT_FLOAT_EQ(commands.throttle, 0.0F);
    EXPECT_FLOAT_EQ(commands.pitch_deg, 0.0F);
    EXPECT_FLOAT_EQ(commands.roll_deg, 0.0F);
}

TEST(AutopilotGround, TaxiUsesTheProfileThrottle)
{
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kTaxi, kProfile);

    AircraftState slow    = ground_state(FlightPhase::kTaxi);
    slow.ground_speed_mps = kProfile.taxi_accel_mps * 0.5F;

    const auto commands = autopilot.update(slow, kProfile, kParameters, 0.01F);
    EXPECT_NEAR(commands.throttle, kProfile.taxi_throttle, 1.0e-3F);
}

TEST(AutopilotGround, TakeoffRollCommandsFullThrottleThenRotation)
{
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kTakeoffRoll, kProfile);

    // Below the rotation speed the nose stays down: rotating early scrubs lift
    // and lengthens the ground roll for nothing.
    AircraftState slow = ground_state(FlightPhase::kTakeoffRoll);
    slow.ias_mps       = kProfile.climb_ias_mps * 0.5F;
    auto commands      = autopilot.update(slow, kProfile, kParameters, 0.01F);

    EXPECT_FLOAT_EQ(commands.throttle, 1.0F);
    EXPECT_FLOAT_EQ(commands.pitch_deg, 0.0F);

    // Above it, the nose comes up and stays up.
    AircraftState fast = ground_state(FlightPhase::kTakeoffRoll);
    fast.ias_mps       = kProfile.climb_ias_mps;
    commands           = autopilot.update(fast, kProfile, kParameters, 0.01F);

    EXPECT_FLOAT_EQ(commands.throttle, 1.0F);
    EXPECT_FLOAT_EQ(commands.pitch_deg, autopilot.gains().rotate_pitch_deg);
}

TEST(AutopilotGround, RotationAgreesWithThePhaseTable)
{
    // The phase table and the throttle come from the same number. If the
    // autopilot gained its own private copy of the rotation speed, changing the
    // mission profile would quietly stop rotating the aircraft.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kTakeoffRoll, kProfile);

    AircraftState just_below = ground_state(FlightPhase::kTakeoffRoll);
    just_below.ias_mps       = kParameters.rotate_ias_mps - 1.0F;
    EXPECT_FLOAT_EQ(autopilot.update(just_below, kProfile, kParameters, 0.01F).pitch_deg, 0.0F)
        << "the aircraft rotated before the phase table's rotation guard would fire";

    AircraftState just_above = ground_state(FlightPhase::kTakeoffRoll);
    just_above.ias_mps       = kParameters.rotate_ias_mps + 1.0F;
    EXPECT_GT(autopilot.update(just_above, kProfile, kParameters, 0.01F).pitch_deg, 0.0F)
        << "the aircraft is at rotation speed but the autopilot has not rotated it";
}

TEST(AutopilotGround, LandingTakesThePowerOffAndSetsLandingFlaps)
{
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kLanding, kProfile);

    const auto commands =
        autopilot.update(ground_state(FlightPhase::kLanding), kProfile, kParameters, 0.01F);

    EXPECT_FLOAT_EQ(commands.throttle, 0.0F);
    EXPECT_FLOAT_EQ(commands.flap_deg, kProfile.landing_flap_deg);
}

TEST(AutopilotGround, FailureLeavesTheAircraftWithoutThrust)
{
    // An engine fire must not be answered with the throttle open.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kFailure, kProfile);

    const auto commands =
        autopilot.update(ground_state(FlightPhase::kFailure), kProfile, kParameters, 0.01F);
    EXPECT_FLOAT_EQ(commands.throttle, 0.0F);
}

// --- Airborne loops ----------------------------------------------------------

TEST(AutopilotAir, CommandsTheAltitudeEachPhaseTargets)
{
    // These are the values the flight schedule actually depends on. Two of them
    // are not the obvious choice and both have a reason recorded in
    // autopilot.cpp: the approach flies a glide path down to the flare height,
    // and holding the gate altitude there would satisfy the handover guard that
    // *started* the phase and never the one that ends it.
    struct Expectation
    {
        FlightPhase phase;
        Scalar      altitude_m;
    };

    const Expectation expectations[]{
        {FlightPhase::kClimb, kProfile.climb_altitude_m},
        {FlightPhase::kGoAround, kProfile.go_around_altitude_m},
        {FlightPhase::kCruise, kProfile.cruise_altitude_m},
        {FlightPhase::kDescent, kProfile.approach_altitude_m},
        {FlightPhase::kApproach, kProfile.flare_altitude_m},
        {FlightPhase::kLanding, 0.0F},
    };

    for (const auto& expectation : expectations)
    {
        Autopilot autopilot;
        autopilot.engage(expectation.phase, kProfile);

        AircraftState state  = cruise_state();
        state.phase          = expectation.phase;
        state.position_m.y   = 300.0F;
        state.velocity_mps.y = 0.0F;

        run(autopilot, state, 2.0F);

        EXPECT_FLOAT_EQ(autopilot.commanded_altitude_m(), expectation.altitude_m)
            << "phase " << to_string(expectation.phase);
    }
}

TEST(AutopilotAir, DescentAndApproachTargetDifferentAltitudes)
{
    // Worth its own test because it is the one that decides whether the
    // simulation ever reaches the approach at all: if the approach targeted the
    // same altitude as the descent, the aircraft would hold the gate for ever
    // and the flare guard would never be evaluated.
    Autopilot descent;
    descent.engage(FlightPhase::kDescent, kProfile);
    run(descent, cruise_state(), 1.0F);

    Autopilot approach;
    approach.engage(FlightPhase::kApproach, kProfile);
    run(approach, cruise_state(), 1.0F);

    EXPECT_GT(descent.commanded_altitude_m(), approach.commanded_altitude_m());
}

TEST(AutopilotAir, VerticalSpeedCommandStaysInsideTheGainLimit)
{
    // The altitude loop's output is a vertical speed, and that speed is the
    // input to the flight path loop. If the first one runs away, the second has
    // nothing left to work with.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kClimb, kProfile);

    AircraftState far_below = cruise_state();
    far_below.position_m.y  = 0.0F;
    far_below.phase         = FlightPhase::kClimb;

    run(autopilot, far_below, 30.0F);

    EXPECT_LE(std::abs(autopilot.commanded_vertical_speed_mps()),
              autopilot.gains().max_vertical_speed_mps)
        << "commanded vertical speed exceeded the gain limit";

    // And the descent side is limited too, to 80% of the climb limit so the
    // aircraft does not dive at the ground to make up time.
    Autopilot descending;
    descending.engage(FlightPhase::kDescent, kProfile);

    AircraftState far_above = cruise_state();
    far_above.position_m.y  = 8000.0F;
    far_above.phase         = FlightPhase::kDescent;

    run(descending, far_above, 30.0F);
    EXPECT_GE(descending.commanded_vertical_speed_mps(), -autopilot.gains().max_vertical_speed_mps);
    EXPECT_LT(descending.commanded_vertical_speed_mps(), 0.0F);
}

TEST(AutopilotAir, CommandsStayInsideTheAirframeLimits)
{
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kClimb, kProfile);

    AircraftState unreachable = cruise_state();
    unreachable.position_m.y  = 0.0F;
    unreachable.heading_deg   = kProfile.cruise_heading_deg + 90.0F;

    run(autopilot, unreachable, 30.0F);

    const auto commands = autopilot.commands();
    EXPECT_LE(std::abs(commands.pitch_deg), kParameters.max_pitch_deg);
    EXPECT_LE(std::abs(commands.roll_deg), autopilot.gains().max_bank_deg);
    EXPECT_GE(commands.flap_deg, 0.0F);
    EXPECT_LE(commands.flap_deg, 30.0F);
}

TEST(AutopilotAir, ThrottleStaysBetweenZeroAndFullAndDoesNotWindUp)
{
    // Commanding the integrator past its limit is the classic windup bug: the
    // throttle sits at 1.0 for the rest of the flight because the integral term
    // has no way to unwind.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kClimb, kProfile);

    AircraftState impossible = cruise_state();
    impossible.position_m.y  = 0.0F;

    for (int i = 0; i < 5000; ++i)
    {
        const auto commands = autopilot.update(impossible, kProfile, kParameters, 0.01F);
        ASSERT_GE(commands.throttle, 0.0F) << "at step " << i;
        ASSERT_LE(commands.throttle, 1.0F) << "throttle integrator wound up at step " << i;
    }
}

TEST(AutopilotAir, SpeedErrorProducesTheRightThrottleDirection)
{
    Autopilot fast_aircraft;
    fast_aircraft.engage(FlightPhase::kCruise, kProfile);

    Autopilot slow_aircraft;
    slow_aircraft.engage(FlightPhase::kCruise, kProfile);

    // The speed loop reads indicated airspeed, not true airspeed, so the test
    // has to move ias_mps. Setting only tas_mps leaves the loop seeing a
    // perfectly on-speed aircraft.
    AircraftState fast = cruise_state();
    fast.ias_mps       = kProfile.cruise_ias_mps * 1.4F;
    AircraftState slow = cruise_state();
    slow.ias_mps       = kProfile.cruise_ias_mps * 0.6F;

    run(fast_aircraft, fast, 5.0F);
    run(slow_aircraft, slow, 5.0F);

    EXPECT_LT(fast_aircraft.commands().throttle, slow_aircraft.commands().throttle)
        << "an aircraft above target speed must be commanded slower, not faster";
}

TEST(AutopilotAir, AHeadingErrorTurnsTheShortWayRound)
{
    // Twenty degrees east and twenty degrees west are the same magnitude of
    // correction. Getting the angle arithmetic wrong produces a full turn
    // instead of a correction, and it presents as a grossly over-gained heading
    // loop rather than as a bug in the subtraction.
    Autopilot turning_east;
    turning_east.engage(FlightPhase::kCruise, kProfile);

    Autopilot turning_west;
    turning_west.engage(FlightPhase::kCruise, kProfile);

    // The cruise leg target is `cruise_heading_deg` rotated by twice the turn
    // bank, so the heading error the loop sees is measured against *that*. It is
    // only known after an update, because that is when the leg timer starts and
    // the leg heading is chosen.
    AircraftState on_leg = cruise_state();
    static_cast<void>(turning_east.update(on_leg, kProfile, kParameters, 0.01F));
    const Scalar leg_target = turning_east.commands().heading_deg;

    AircraftState twenty_east = cruise_state();
    twenty_east.heading_deg   = wrap_degrees(leg_target + 20.0F);
    AircraftState twenty_west = cruise_state();
    twenty_west.heading_deg   = wrap_degrees(leg_target - 20.0F);

    run(turning_east, twenty_east, 0.5F);
    run(turning_west, twenty_west, 0.5F);

    const Scalar east_bank = turning_east.commands().roll_deg;
    const Scalar west_bank = turning_west.commands().roll_deg;

    EXPECT_NEAR(std::abs(east_bank), std::abs(west_bank), 1.0e-3F)
        << "a symmetric heading error must command a symmetric bank";
    EXPECT_NE(std::signbit(east_bank), std::signbit(west_bank))
        << "turning east and turning west produced the same bank";
}

TEST(AutopilotAir, BankStaysInsideTheLimitForALargeHeadingError)
{
    Autopilot    autopilot;
    const Scalar max_bank = autopilot.gains().max_bank_deg;
    autopilot.engage(FlightPhase::kCruise, kProfile);

    AircraftState backwards = cruise_state();
    backwards.heading_deg   = wrap_degrees(kProfile.cruise_heading_deg + 170.0F);

    run(autopilot, backwards, 5.0F);

    EXPECT_LE(std::abs(autopilot.commands().roll_deg), max_bank + 1.0e-3F);
}

// --- Lifecycle ---------------------------------------------------------------

TEST(Autopilot, EngageReleasesTheThrottleIntegral)
{
    // The integral only carries the throttle bias, so it has to be released on
    // every phase change: a climb bias would otherwise fight the descent, and
    // the aircraft would arrive at the approach still at full power.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kClimb, kProfile);

    // Low but still airborne: `update()` routes on height, so a state at exactly
    // zero would reach the ground law and read the throttle from the phase table
    // rather than from the speed loop.
    AircraftState climbing = cruise_state();
    climbing.position_m.y  = 1.0F;
    // At the climb target speed, not the cruise speed. The speed loop closes the
    // throttle on an aircraft that is *above* its target, and cruise_ias_mps is
    // well above climb_target_ias_mps, so a cruise-speed climb would sit at zero
    // throttle and this test would be measuring the wrong thing entirely.
    climbing.ias_mps = kProfile.climb_target_ias_mps;
    run(autopilot, climbing, 30.0F);
    ASSERT_GT(autopilot.commands().throttle, 0.6F);

    autopilot.engage(FlightPhase::kDescent, kProfile);

    AircraftState descending = cruise_state();
    descending.position_m.y  = 3000.0F;
    run(autopilot, descending, 1.0F);

    EXPECT_LT(autopilot.commands().throttle, 0.6F)
        << "the climb throttle bias survived the phase change";
}

TEST(Autopilot, ResetClearsTheIntegrator)
{
    // Without this, a reset run inherits the previous run's wind-up and the
    // throttle is wrong for the first few seconds -- which is exactly what makes
    // a determinism test fail for the wrong reason.
    Autopilot autopilot;
    autopilot.engage(FlightPhase::kClimb, kProfile);

    // Low but still airborne: `update()` routes on height, so a state at exactly
    // zero would reach the ground law and read the throttle from the phase table
    // rather than from the speed loop.
    AircraftState climbing = cruise_state();
    climbing.position_m.y  = 1.0F;
    climbing.ias_mps       = kProfile.climb_target_ias_mps;
    run(autopilot, climbing, 30.0F);
    ASSERT_GT(autopilot.commands().throttle, 0.6F);

    autopilot.reset();

    EXPECT_FLOAT_EQ(autopilot.commands().throttle, 0.0F);
    EXPECT_FLOAT_EQ(autopilot.commanded_vertical_speed_mps(), 0.0F);
    EXPECT_FLOAT_EQ(autopilot.commanded_altitude_m(), 0.0F);
    EXPECT_EQ(autopilot.engaged_phase(), FlightPhase::kPreflight);
}

TEST(Autopilot, UpdateRoutesToTheGroundOrAirPathByPhase)
{
    // The routing is a precondition, not a detail: calling the air loop while
    // the aircraft is still on the runway commands a pitch the ground loop is
    // about to override, and the throttle integral winds up against a command
    // that is never executed.
    struct Case
    {
        FlightPhase phase;
        bool        airborne;
    };

    const Case cases[]{
        {FlightPhase::kPreflight, false},   {FlightPhase::kTaxi, false},
        {FlightPhase::kTakeoffRoll, false}, {FlightPhase::kLanding, false},
        {FlightPhase::kClimb, true},        {FlightPhase::kCruise, true},
        {FlightPhase::kDescent, true},      {FlightPhase::kApproach, true},
        {FlightPhase::kGoAround, true},     {FlightPhase::kFailure, true},
    };

    for (const auto& c : cases)
    {
        Autopilot autopilot;
        autopilot.engage(c.phase, kProfile);

        AircraftState state = c.airborne ? cruise_state() : ground_state(c.phase);
        state.phase         = c.phase;
        state.heading_deg   = kProfile.cruise_heading_deg + 25.0F;

        run(autopilot, state, 3.0F);

        const auto& commands = autopilot.commands();
        EXPECT_TRUE(std::isfinite(commands.pitch_deg)) << to_string(c.phase);
        EXPECT_TRUE(std::isfinite(commands.roll_deg)) << to_string(c.phase);
        EXPECT_TRUE(std::isfinite(commands.throttle)) << to_string(c.phase);
        EXPECT_GE(commands.throttle, 0.0F) << to_string(c.phase);
        EXPECT_LE(commands.throttle, 1.0F) << to_string(c.phase);
        EXPECT_GE(commands.heading_deg, 0.0F) << to_string(c.phase);
        EXPECT_LE(commands.heading_deg, 360.0F) << to_string(c.phase);
        EXPECT_TRUE(c.airborne || commands.roll_deg == 0.0F)
            << "the ground path commanded a bank in " << to_string(c.phase);
    }
}

TEST(Autopilot, IsDeterministicForTheSameInput)
{
    Autopilot first;
    Autopilot second;

    const auto run_both = [](Autopilot& autopilot)
    {
        autopilot.engage(FlightPhase::kClimb, kProfile);
        AircraftState climbing = cruise_state();
        climbing.position_m.y  = 0.0F;
        run(autopilot, climbing, 10.0F);
    };

    run_both(first);
    run_both(second);

    // Bit exact, not approximately equal: the throttle integral is an
    // accumulator, and any difference in the order of operations shows up in
    // full at the last bit several seconds later.
    EXPECT_EQ(first.commands().throttle, second.commands().throttle);
    EXPECT_EQ(first.commands().pitch_deg, second.commands().pitch_deg);
    EXPECT_EQ(first.commanded_vertical_speed_mps(), second.commanded_vertical_speed_mps());
}

} // namespace
} // namespace fse