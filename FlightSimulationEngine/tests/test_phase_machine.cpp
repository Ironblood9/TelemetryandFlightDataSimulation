// ---------------------------------------------------------------------------
// PhaseMachine behaviour.
//
// The table and the guards are covered in test_phase_table.cpp and the
// enumerations in test_phase_enum.cpp. This file is about the machine itself:
// the ordering inside one update, what resets when, and the paths that are not
// table entries at all -- a latched fault and an operator override.
//
// Those two paths are the ones worth the most scrutiny, because they are the
// ones where the table says nothing and a reviewer has to read the code to know
// what happens.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include "fse/phase_machine.hpp"

namespace fse
{
namespace
{

[[nodiscard]] AircraftState make_state(FlightPhase phase, Scalar altitude_m, Scalar ias_mps,
                                       Scalar vertical_speed_mps = 0.0F)
{
    AircraftState state{};
    state.phase          = phase;
    state.position_m.y   = altitude_m;
    state.ias_mps        = ias_mps;
    state.velocity_mps.y = vertical_speed_mps;
    return state;
}

const AircraftParameters kParameters{};
const MissionProfile     kProfile{};

TEST(PhaseMachine, StartsWhereItIsTold)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kCruise);

    EXPECT_EQ(machine.phase(), FlightPhase::kCruise);
    EXPECT_FLOAT_EQ(machine.elapsed_s(), 0.0F);
    EXPECT_EQ(machine.transition_count(), 0U);
    EXPECT_EQ(machine.last_transition(), nullptr);
}

TEST(PhaseMachine, RecordsWhyItTransitioned)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kPreflight);

    const AircraftState state = make_state(FlightPhase::kPreflight, 0.0F, 0.0F);
    machine.update(0.01F, state, kParameters, kProfile);
    EXPECT_EQ(machine.phase(), FlightPhase::kPreflight) << "the checklist has not run yet";

    machine.update(5.0F, state, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kTaxi);
    ASSERT_NE(machine.last_transition(), nullptr);
    EXPECT_EQ(machine.last_transition()->from, FlightPhase::kPreflight);
    EXPECT_EQ(machine.last_transition()->to, FlightPhase::kTaxi);
    EXPECT_EQ(machine.transition_count(), 1U);

    // The reason is what the phase trace prints, so it has to come from the same
    // table entry that caused the move rather than being recomputed anywhere.
    EXPECT_STREQ(machine.transition_reason(), machine.last_transition()->reason);
}

TEST(PhaseMachine, ElapsedTimeResetsOnTransition)
{
    // The guards that gate on elapsed time -- checklist, cruise duration,
    // rollout -- are only meaningful relative to the phase that started them.
    // Without this reset, a 40 second preflight would satisfy the cruise guard
    // the instant the machine reached cruise.
    PhaseMachine machine;
    machine.reset(FlightPhase::kPreflight);

    const AircraftState state = make_state(FlightPhase::kPreflight, 0.0F, 0.0F);
    machine.update(10.0F, state, kParameters, kProfile);

    EXPECT_FLOAT_EQ(machine.elapsed_s(), 0.0F);
    EXPECT_EQ(machine.transition_count(), 1U);
}

TEST(PhaseMachine, AppliesAtMostOneTransitionPerUpdate)
{
    // A single update is a single tick of physics. Chaining two transitions in
    // one tick would let an aircraft placed just above the approach gate and
    // just below the flare height skip the entire landing phase.
    PhaseMachine machine;
    machine.reset(FlightPhase::kDescent);

    // Below the approach gate and below the flare height at the same time.
    const AircraftState low = make_state(FlightPhase::kDescent, 5.0F, 60.0F);
    machine.update(0.01F, low, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kApproach) << "it must not have skipped to landing";
    EXPECT_EQ(machine.transition_count(), 1U);

    machine.update(0.01F, low, kParameters, kProfile);
    EXPECT_EQ(machine.phase(), FlightPhase::kLanding);
    EXPECT_EQ(machine.transition_count(), 2U);
}

TEST(PhaseMachine, ANoOpUpdateDoesNotCount)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kPreflight);

    const AircraftState state = make_state(FlightPhase::kPreflight, 0.0F, 0.0F);
    for (int i = 0; i < 100; ++i)
    {
        machine.update(0.01F, state, kParameters, kProfile);
    }

    EXPECT_EQ(machine.transition_count(), 0U);
    EXPECT_EQ(machine.last_transition(), nullptr);
}

TEST(PhaseMachine, ClimbCaptureWinsWhenBothClimbGuardsPass)
{
    // The behavioural counterpart to the table order test. With the climb target
    // set to the current altitude *and* the airspeed below the go-around margin,
    // both climb guards are true. The table is ordered capture-first on purpose:
    // the go-around would throw away a climb that has already arrived, and the
    // cruise speed loop has the authority to recover from arriving slowly.
    PhaseMachine machine;
    machine.reset(FlightPhase::kClimb);
    machine.set_climb_target_altitude_m(900.0F);

    const AircraftState arrived_slow = make_state(FlightPhase::kClimb, 900.0F, 40.0F, 0.0F);
    ASSERT_TRUE(
        guard_climb_complete(PhaseContext{arrived_slow, kParameters, kProfile, 5.0F, 900.0F}));
    ASSERT_TRUE(
        guard_climb_airspeed_lost(PhaseContext{arrived_slow, kParameters, kProfile, 5.0F, 900.0F}))
        << "this test is only meaningful while both guards can pass";

    machine.update(0.01F, arrived_slow, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kCruise);
    EXPECT_STREQ(machine.transition_reason(), "climb altitude captured");
}

TEST(PhaseMachine, ClimbGoAroundStillFiresWhenTheAltitudeIsNotCaptured)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kClimb);
    machine.set_climb_target_altitude_m(900.0F);

    const AircraftState mushing = make_state(FlightPhase::kClimb, 300.0F, 40.0F, 1.0F);
    machine.update(0.01F, mushing, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kGoAround);
    EXPECT_STREQ(machine.transition_reason(), "airspeed margin lost during climb");
}

TEST(PhaseMachine, AFaultOverridesEveryScheduledTransition)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kCruise);

    AircraftState faulty = make_state(FlightPhase::kCruise, 900.0F, 105.0F);
    faulty.faults        = FaultCode::kEngineFire;

    // The cruise guard is already true after 100 s, so without the fault check
    // the machine would go to descent first. A failure must not be delayed by a
    // phase whose schedule happens to be due.
    machine.update(100.0F, faulty, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kFailure);
    EXPECT_TRUE(machine.in_failure_phase());
    EXPECT_STREQ(machine.transition_reason(), kFaultReason);
    EXPECT_EQ(machine.transition_count(), 1U);
}

TEST(PhaseMachine, FailureOverridesFromEveryPhase)
{
    // Not just from cruise. An engine fire during the flare must not be able to
    // slip through the handover to landing first.
    for (std::uint8_t i = 0; i < kFlightPhaseCount; ++i)
    {
        const auto phase = static_cast<FlightPhase>(i);
        if (phase == FlightPhase::kFailure)
        {
            continue;
        }

        PhaseMachine machine;
        machine.reset(phase);

        AircraftState faulty = make_state(phase, 100.0F, 90.0F);
        faulty.faults        = FaultCode::kEngineOut;

        machine.update(0.01F, faulty, kParameters, kProfile);
        EXPECT_EQ(machine.phase(), FlightPhase::kFailure)
            << "fault not honoured from " << to_string(phase);
    }
}

TEST(PhaseMachine, FailureIsAbsorbing)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kCruise);

    AircraftState faulty = make_state(FlightPhase::kFailure, 900.0F, 105.0F);
    faulty.faults        = FaultCode::kEngineFire;
    machine.update(0.01F, faulty, kParameters, kProfile);
    ASSERT_EQ(machine.phase(), FlightPhase::kFailure);

    // Even with the fault cleared, the machine must not wander off on its own.
    // There is no table entry out of kFailure and no operator command to clear
    // it, so leaving on its own would be a state nothing can describe.
    const AircraftState recovered = make_state(FlightPhase::kFailure, 900.0F, 105.0F);
    for (int i = 0; i < 1000; ++i)
    {
        machine.update(0.5F, recovered, kParameters, kProfile);
    }
    EXPECT_EQ(machine.phase(), FlightPhase::kFailure);
    EXPECT_EQ(machine.transition_count(), 1U) << "only the initial entry into failure counts";
}

TEST(PhaseMachine, ForceOverridesTheSchedule)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kPreflight);
    machine.force(FlightPhase::kApproach);

    EXPECT_EQ(machine.phase(), FlightPhase::kApproach);
    EXPECT_FLOAT_EQ(machine.elapsed_s(), 0.0F);
    EXPECT_EQ(machine.last_transition(), nullptr) << "an operator command has no table entry";
    EXPECT_STREQ(machine.transition_reason(), kOperatorReason);

    // And the machine then behaves completely normally from there.
    const AircraftState low = make_state(FlightPhase::kApproach, 5.0F, 60.0F);
    machine.update(0.01F, low, kParameters, kProfile);
    EXPECT_EQ(machine.phase(), FlightPhase::kLanding);
    EXPECT_STREQ(machine.transition_reason(), "flare entry");
}

TEST(PhaseMachine, ForceStillRespectsALatchedFault)
{
    // An operator command must not be able to talk the machine out of failure,
    // or the control channel becomes a way to hide a fire from the ground.
    PhaseMachine machine;
    machine.reset(FlightPhase::kFailure);

    AircraftState faulty = make_state(FlightPhase::kFailure, 900.0F, 105.0F);
    faulty.faults        = FaultCode::kEngineFire;

    machine.force(FlightPhase::kCruise);
    machine.update(0.01F, faulty, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kFailure);
}

TEST(PhaseMachine, ZeroTimestepDoesNotAdvanceAnything)
{
    PhaseMachine machine;
    machine.reset(FlightPhase::kPreflight);

    const AircraftState state = make_state(FlightPhase::kPreflight, 0.0F, 0.0F);
    machine.update(0.0F, state, kParameters, kProfile);

    EXPECT_EQ(machine.phase(), FlightPhase::kPreflight);
    EXPECT_FLOAT_EQ(machine.elapsed_s(), 0.0F);
}

TEST(PhaseMachine, NeverLeavesThePhaseTable)
{
    // Walk the machine through every phase with every guard satisfied and check
    // that it only ever lands somewhere the table describes. This is the
    // "unrecoverable state" check: a phase with no outgoing entry would show up
    // here as the simulation parking in a phase that no guard can ever leave.
    std::array<bool, kFlightPhaseCount> declared_destination{};
    for (const auto& transition : kPhaseTransitions)
    {
        declared_destination.at(static_cast<std::size_t>(transition.to)) = true;
    }
    declared_destination.at(static_cast<std::size_t>(FlightPhase::kFailure))   = true;
    declared_destination.at(static_cast<std::size_t>(FlightPhase::kPreflight)) = true;

    for (std::uint8_t i = 0; i < kFlightPhaseCount; ++i)
    {
        const auto start = static_cast<FlightPhase>(i);
        if (start == FlightPhase::kFailure)
        {
            continue;
        }

        PhaseMachine machine;
        machine.reset(start);
        machine.set_climb_target_altitude_m(0.0F);

        // A state that satisfies every guard at once: on the ground, at every
        // altitude, moving, fast.
        AircraftState everything;
        everything.position_m.y = 0.0F;
        everything.ias_mps      = 1000.0F;

        for (int step = 0; step < 50; ++step)
        {
            machine.update(1.0F, everything, kParameters, kProfile);
            ASSERT_TRUE(is_valid(machine.phase())) << "invalid phase from " << to_string(start);
            EXPECT_TRUE(declared_destination.at(static_cast<std::size_t>(machine.phase())))
                << "reached " << to_string(machine.phase()) << ", which no table entry describes";
        }
    }
}

} // namespace
} // namespace fse