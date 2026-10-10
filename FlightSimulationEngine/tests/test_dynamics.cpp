// ---------------------------------------------------------------------------
// Flight model behaviour.
//
// The parameter consistency and atmosphere tests already pin the numbers down,
// so this file is about what the model *does*: that it accelerates, that it
// lifts off when the wing can carry the weight, that it does not sink through
// the runway, and that the initial condition is restored by reset().
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>

#include "fse/flight_model.hpp"

namespace fse
{
namespace
{

/// Steps the model with a fixed command set, bypassing the guidance.
void run_fixed(AerodynamicModel& model, const ControlInputs& commands, Scalar seconds,
               Scalar dt = 0.01F)
{
    const auto steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i)
    {
        model.step(dt, commands);
    }
}

TEST(AerodynamicModel, StartsParkedAndLevel)
{
    AerodynamicModel model{};

    const AircraftState& state = model.state();
    EXPECT_FLOAT_EQ(state.position_m.y, 0.0F);
    EXPECT_FLOAT_EQ(state.ias_mps, 0.0F);
    EXPECT_FLOAT_EQ(state.pitch_deg, 0.0F);
    EXPECT_FLOAT_EQ(state.roll_deg, 0.0F);
    EXPECT_TRUE(model.on_ground());
    EXPECT_FALSE(model.airborne());
    EXPECT_EQ(state.phase, FlightPhase::kPreflight);
}

TEST(AerodynamicModel, CarriesNoWindUnlessAsked)
{
    // A wind model that is on unless it is switched off leaks into every test
    // that constructs a bare model, and "the aircraft is parked but the pitot
    // reads 6 m/s" is the sort of surprise that costs an afternoon. This is the
    // test that keeps that from coming back.
    AerodynamicModel model{};

    EXPECT_FLOAT_EQ(model.wind().steady_mps.x, 0.0F);
    EXPECT_FLOAT_EQ(model.wind().steady_mps.z, 0.0F);
    EXPECT_FLOAT_EQ(model.wind().gust_sigma_mps, 0.0F);
    EXPECT_FLOAT_EQ(model.state().tas_mps, 0.0F);

    // In still air a parked aircraft reads nothing.
    run_fixed(model, ControlInputs{}, 5.0F);
    EXPECT_FLOAT_EQ(model.state().ias_mps, 0.0F);

    // A tailwind gives it true airspeed but still no ground speed. The taxi
    // guard depends on that difference.
    model.wind() = WindModel::light_tailwind();
    model.reset(1U);
    run_fixed(model, ControlInputs{}, 5.0F);

    EXPECT_GT(model.state().tas_mps, 1.0F);
    EXPECT_FLOAT_EQ(model.state().ground_speed_mps, 0.0F);
    EXPECT_NEAR(model.state().ias_mps, model.state().tas_mps, 0.1F);
}

TEST(AerodynamicModel, StaysPutWithoutThrottle)
{
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 0.0F;

    run_fixed(model, commands, 30.0F);

    EXPECT_NEAR(model.state().ias_mps, 0.0F, 0.01F);
    EXPECT_NEAR(model.state().position_m.x, 0.0F, 0.01F);
    EXPECT_NEAR(model.state().position_m.z, 0.0F, 0.01F);
}

TEST(AerodynamicModel, AcceleratesUnderThrustAndStaysOnTheGround)
{
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 1.0F;

    run_fixed(model, commands, 10.0F);

    EXPECT_GT(model.state().ias_mps, 20.0F);
    EXPECT_FLOAT_EQ(model.state().position_m.y, 0.0F);

    // The wings cannot carry the weight at low speed with the nose on the
    // ground, so nothing leaves the runway yet.
    EXPECT_LT(model.state().load_factor, 1.2F);
}

TEST(AerodynamicModel, RollMotionIsIntegrated)
{
    // The ground handler has to integrate position as well as velocity. An
    // earlier version accelerated to rotation speed while standing on the
    // origin, because only the velocity was being integrated.
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle    = 1.0F;
    commands.heading_deg = 0.0F; // due north

    run_fixed(model, commands, 8.0F);

    const AircraftState& state = model.state();
    EXPECT_GT(state.position_m.z, 100.0F) << "the aircraft did not move along the runway";
    EXPECT_NEAR(state.position_m.x, 0.0F, 1.0F) << "it drifted sideways";
}

TEST(AerodynamicModel, LiftsOffWhenTheWingCarriesTheWeight)
{
    // The take-off decision is emergent: lift is always integrated and the
    // ground is a non-penetration constraint, so the aircraft leaves the runway
    // exactly when the net vertical force turns positive. An earlier design
    // branched between a ground handler and a flight integrator instead, and at
    // rotation speed -- where lift is within a few percent of the weight -- the
    // branch flipped every few steps and the aircraft hopped down the runway.
    //
    // The rotation itself is the guidance's job, so this test supplies it: nose
    // on the centreline and full power, held until there is enough airspeed for
    // the wing to be worth rotating, then the rotation pitch. That is what
    // Autopilot::update_ground does for FlightPhase::kTakeoffRoll.
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 1.0F;

    // Nose down, full power: this part must not lift the aircraft off, however
    // long it is held. If it does, something is adding vertical force that is
    // not in the force sum.
    run_fixed(model, commands, 15.0F);
    ASSERT_TRUE(model.on_ground()) << "lifted off with the nose still on the runway";

    commands.pitch_deg = 9.0F;
    run_fixed(model, commands, 10.0F);

    ASSERT_FALSE(model.on_ground()) << "never left the runway";
    EXPECT_GT(model.state().position_m.y, 0.0F);
    EXPECT_GT(model.state().tas_mps, model.parameters().rotate_ias_mps * 0.8F);
    EXPECT_GT(model.state().load_factor, 0.5F);
}

TEST(AerodynamicModel, NeverSinksThroughTheSurface)
{
    AerodynamicModel model{};

    // Full thrust and full nose down from the first step: the worst case for the
    // ground contact constraint.
    ControlInputs commands{};
    commands.throttle  = 1.0F;
    commands.pitch_deg = -25.0F;

    run_fixed(model, commands, 30.0F);

    EXPECT_GE(model.state().position_m.y, 0.0F);
    EXPECT_TRUE(std::isfinite(model.state().position_m.y));
}

TEST(AerodynamicModel, ReadsOneGOnTheGround)
{
    // The accelerometer reads 1 g with the aircraft level on its gear: the
    // wheels carry whatever the wing does not. Leaving the aerodynamic load
    // factor there would report 0 g while parked, which no real instrument does
    // and which every consumer would have to special case.
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 1.0F;

    run_fixed(model, commands, 2.0F);

    EXPECT_NEAR(model.state().load_factor, 1.0F, 1.0e-3F);
}

TEST(AerodynamicModel, ThrustSpoolsUpOverTime)
{
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 1.0F;

    model.step(0.01F, commands);
    const Scalar first = model.state().throttle;
    run_fixed(model, commands, 0.5F);
    const Scalar after_half_second = model.state().throttle;
    run_fixed(model, commands, 10.0F);
    const Scalar settled = model.state().throttle;

    // A spool lag is the point: the throttle must not reach the command at once.
    EXPECT_GT(first, 0.0F);
    EXPECT_LT(first, 0.5F);
    EXPECT_GT(after_half_second, first);
    EXPECT_GT(settled, after_half_second);
    EXPECT_NEAR(settled, 1.0F, 0.02F);
}

TEST(AerodynamicModel, EngineOutRemovesThrust)
{
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle = 1.0F;

    run_fixed(model, commands, 10.0F);
    ASSERT_GT(model.state().throttle, 0.5F);

    model.latch_fault(FaultCode::kEngineOut);

    // Long enough for the 1.8 s spool lag to run out; the throttle decays
    // exponentially and never reaches exactly zero.
    run_fixed(model, commands, 12.0F);

    EXPECT_NEAR(model.state().throttle, 0.0F, 0.01F);
}

TEST(AerodynamicModel, ResetRestoresTheInitialCondition)
{
    AerodynamicModel model{};
    ControlInputs    commands{};
    commands.throttle  = 1.0F;
    commands.pitch_deg = 9.0F;

    run_fixed(model, commands, 30.0F);
    ASSERT_GT(model.state().ias_mps, 30.0F);

    model.reset(99U);

    EXPECT_FLOAT_EQ(model.state().ias_mps, 0.0F);
    EXPECT_FLOAT_EQ(model.state().position_m.y, 0.0F);
    EXPECT_FLOAT_EQ(model.state().position_m.x, 0.0F);
    EXPECT_FLOAT_EQ(model.state().throttle, 0.0F);
    EXPECT_EQ(model.state().step_index, 0U);
    EXPECT_EQ(model.state().phase, FlightPhase::kPreflight);
}

TEST(AerodynamicModel, EnergyHeightFollowsTheEnergyEquation)
{
    AerodynamicModel model{};
    model.reset(21U);

    ControlInputs commands{};
    commands.throttle  = 0.8F;
    commands.pitch_deg = 8.0F;
    run_fixed(model, commands, 25.0F);

    const AircraftState& state = model.state();
    const Scalar         expected =
        state.position_m.y + (state.tas_mps * state.tas_mps / (2.0F * kGravity));

    EXPECT_NEAR(state.energy_height_m, expected, 0.01F);
}

TEST(AerodynamicModel, PlaceAirbornePutsTheAircraftInTrimmedFlight)
{
    // Scenario setup, not physics: a run that starts in cruise has to begin in
    // the air, because no airborne phase commands a take-off throttle and a
    // "cruise" scenario placed on the runway would sit there for ever.
    AerodynamicModel model{};
    model.reset(5U);
    model.place_airborne(900.0F, 100.0F, 90.0F);

    const AircraftState& state = model.state();
    EXPECT_FLOAT_EQ(state.position_m.y, 900.0F);
    EXPECT_NEAR(state.heading_deg, 90.0F, 1.0e-3F);
    EXPECT_TRUE(model.airborne());
    EXPECT_GT(state.tas_mps, 50.0F);
    EXPECT_FLOAT_EQ(state.pitch_deg, 0.0F);
}

TEST(AerodynamicModel, ZeroTimestepIsANoOp)
{
    AerodynamicModel model{};
    const Vec3       before = model.state().position_m;

    model.step(0.0F, ControlInputs{});

    EXPECT_FLOAT_EQ(model.state().position_m.x, before.x);
    EXPECT_EQ(model.state().step_index, 0U);
}

} // namespace
} // namespace fse