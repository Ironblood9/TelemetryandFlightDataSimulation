// ---------------------------------------------------------------------------
// The phase transition table and its guards.
//
// Guards are plain function pointers over a PhaseContext precisely so that this
// file can call them directly, with no simulation in the picture. The alternative
// -- exercising them only through a full flight -- means a guard with a typo in
// it shows up as "the sortie took 40 seconds longer", which is a much harder
// thing to diagnose than a failing assertion.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <string>

#include "fse/phase_machine.hpp"

namespace fse
{
namespace
{

/// Sentinel returned by index_of() when the table has no such entry.
inline constexpr std::size_t kNoTransition = static_cast<std::size_t>(-1);

/// Position of a transition in the table, or kNoTransition.
///
/// The table is evaluated strictly in order and the first matching guard wins,
/// so the *position* of an entry is part of its meaning, not an implementation
/// detail. Returning an index rather than an iterator keeps that comparison
/// meaningful and printable.
[[nodiscard]] std::size_t index_of(FlightPhase from, FlightPhase to) noexcept
{
    for (std::size_t i = 0; i < kPhaseTransitions.size(); ++i)
    {
        const auto& transition = kPhaseTransitions[i];
        if (transition.from == from && transition.to == to)
        {
            return i;
        }
    }
    return kNoTransition;
}

/// A context with everything zeroed, so each test states only what it is about.
struct Fixture : ::testing::Test
{
    AircraftParameters parameters{};
    MissionProfile     profile{};
    AircraftState      state{};

    [[nodiscard]] PhaseContext context(Scalar elapsed_s = 0.0F, Scalar climb_target = 0.0F)
    {
        return PhaseContext{state, parameters, profile, elapsed_s, climb_target};
    }
};

TEST_F(Fixture, ChecklistGuardIsATimeGate)
{
    EXPECT_FALSE(guard_checklist_complete(context(2.9F)));
    EXPECT_TRUE(guard_checklist_complete(context(3.0F)));
    EXPECT_TRUE(guard_checklist_complete(context(3.1F)));
}

TEST_F(Fixture, TaxiGuardUsesGroundSpeedNotAirspeed)
{
    // Ground speed, not airspeed. A parked aircraft in a tailwind has a non zero
    // indicated airspeed while standing still, and an airspeed based guard would
    // clear it for departure before it had moved at all. This is the whole
    // reason the guard reads one field rather than the other.
    state.ias_mps          = 40.0F;
    state.ground_speed_mps = 0.0F;
    EXPECT_FALSE(guard_taxi_complete(context()));

    state.ground_speed_mps = profile.taxi_accel_mps;
    EXPECT_TRUE(guard_taxi_complete(context()));

    state.ias_mps          = 0.0F;
    state.ground_speed_mps = profile.taxi_accel_mps + 0.01F;
    EXPECT_TRUE(guard_taxi_complete(context())) << "still clear with the pitot reading zero";
}

TEST_F(Fixture, RotationGuardComparesAgainstTheAircraftParameter)
{
    state.ias_mps = parameters.rotate_ias_mps - 0.01F;
    EXPECT_FALSE(guard_rotation_speed(context()));

    state.ias_mps = parameters.rotate_ias_mps;
    EXPECT_TRUE(guard_rotation_speed(context()));
}

TEST_F(Fixture, ClimbCaptureNeedsBothAltitudeAndVerticalSpeedSettled)
{
    // Reaching the altitude is not enough: the guard also requires the vertical
    // speed to be inside its band, otherwise the machine reports "climb altitude
    // captured" while the aircraft is still passing through it at 15 m/s and
    // immediately transitions straight back out.
    constexpr Scalar kTarget = 900.0F;

    state.position_m.y   = kTarget;
    state.velocity_mps.y = 0.0F;
    EXPECT_TRUE(guard_climb_complete(context(0.0F, kTarget)));

    state.velocity_mps.y = 3.0F;
    EXPECT_FALSE(guard_climb_complete(context(0.0F, kTarget))) << "vertical speed not settled";

    state.velocity_mps.y = 0.0F;
    state.position_m.y   = kTarget - 30.0F;
    EXPECT_FALSE(guard_climb_complete(context(0.0F, kTarget))) << "altitude not captured";
}

TEST_F(Fixture, ClimbAirspeedLossTriggersTheGoAround)
{
    state.ias_mps = 46.0F;
    EXPECT_FALSE(guard_climb_airspeed_lost(context()));

    state.ias_mps = 44.0F;
    EXPECT_TRUE(guard_climb_airspeed_lost(context()));
}

TEST_F(Fixture, DescentGuardUsesACaptureBandNotABareThreshold)
{
    // A bare `altitude <= gate` test can never be satisfied when the descent is
    // holding exactly the gate altitude: the altitude loop converges onto its
    // setpoint from one side and the guard then waits for the other side of a
    // threshold the aircraft is already sitting on. The handover therefore
    // happens on *entry into* a band.
    const Scalar gate = profile.approach_altitude_m;

    state.position_m.y = gate + 16.0F;
    EXPECT_FALSE(guard_descent_complete(context()));

    state.position_m.y = gate + 14.0F;
    EXPECT_TRUE(guard_descent_complete(context())) << "should capture just inside the band";

    state.position_m.y = gate;
    EXPECT_TRUE(guard_descent_complete(context()));
}

TEST_F(Fixture, FlareGuardUsesTheSameBandArgument)
{
    const Scalar flare = profile.flare_altitude_m;

    state.position_m.y = flare + 9.0F;
    EXPECT_FALSE(guard_flare_entry(context()));

    state.position_m.y = flare + 7.0F;
    EXPECT_TRUE(guard_flare_entry(context()));
}

TEST_F(Fixture, TouchdownGuardsDisagreeExactlyOnSpeed)
{
    // Landing and go-around must partition the touchdown condition with no gap
    // and no overlap. `ok` additionally waits for the rollout to finish, so at
    // the moment of contact only the missed guard can fire.
    state.position_m.y = 0.5F;
    state.ias_mps      = profile.touchdown_ias_mps;

    EXPECT_FALSE(guard_touchdown_missed(context())) << "touchdown inside the limit";
    EXPECT_FALSE(guard_touchdown_ok(context(0.0F))) << "still rolling, not stopped yet";
    EXPECT_TRUE(guard_touchdown_ok(context(2.0F)));

    state.ias_mps = profile.max_touchdown_ias_mps + 1.0F;
    EXPECT_TRUE(guard_touchdown_missed(context()));
    EXPECT_FALSE(guard_touchdown_ok(context(10.0F)))
        << "a go-around must win even after a long rollout";

    // Exactly at the limit is still a good touchdown, not a missed one.
    state.ias_mps = profile.max_touchdown_ias_mps;
    EXPECT_FALSE(guard_touchdown_missed(context()));
}

TEST_F(Fixture, GoAroundCompletesAtTheProfileAltitude)
{
    state.position_m.y = profile.go_around_altitude_m - 1.0F;
    EXPECT_FALSE(guard_go_around_complete(context()));

    state.position_m.y = profile.go_around_altitude_m;
    EXPECT_TRUE(guard_go_around_complete(context()));
}

TEST(PhaseTable, EverySourcePhaseHasAtLeastOneExit)
{
    // A phase with no outgoing entry is a dead end the simulation can enter and
    // never leave. Every phase except kFailure, which is entered from anywhere
    // and has no scheduled exit, must appear as a `from`.
    std::array<bool, kFlightPhaseCount> has_exit{};

    for (const auto& transition : kPhaseTransitions)
    {
        has_exit.at(static_cast<std::size_t>(transition.from)) = true;
    }

    for (std::size_t i = 0; i < kFlightPhaseCount; ++i)
    {
        const auto phase = static_cast<FlightPhase>(i);
        if (phase == FlightPhase::kFailure)
        {
            continue; // entered from anywhere on a fault, no scheduled exit
        }
        EXPECT_TRUE(has_exit[i]) << "phase " << to_string(phase) << " is a dead end";
    }
}

TEST(PhaseTable, TaxiIsReachableBothWays)
{
    // Landing returns to taxi, which is why the table is not a simple chain. A
    // graph with an unnoticed one-way edge here shows up as an aircraft that
    // lands and then sits at the runway hold forever.
    EXPECT_NE(index_of(FlightPhase::kPreflight, FlightPhase::kTaxi), kNoTransition);
    EXPECT_NE(index_of(FlightPhase::kTaxi, FlightPhase::kTakeoffRoll), kNoTransition);
    EXPECT_NE(index_of(FlightPhase::kLanding, FlightPhase::kTaxi), kNoTransition);
}

TEST(PhaseTable, FailureIsNeverAScheduledDestination)
{
    // kFailure is entered when a fault is latched, which is a condition rather
    // than a schedule. If a table entry ever routed into it, the machine would
    // be able to enter failure with no fault latched and then never leave.
    for (const auto& transition : kPhaseTransitions)
    {
        EXPECT_NE(transition.to, FlightPhase::kFailure)
            << "table entry " << to_string(transition.from) << " -> failure";
    }
}

TEST(PhaseTable, ClimbCaptureIsCheckedBeforeAirspeedLoss)
{
    // Both climb guards can in principle be true at the same time: an aircraft
    // sitting at the climb target altitude with the airspeed decayed below the
    // go-around margin. The table checks capture first, and that is deliberate.
    //
    // The go-around is the expensive, alarming branch -- it throws away a
    // completed climb and puts a 700 m turn into the profile. When the aircraft
    // has already reached its target altitude, the cruise speed loop has the
    // authority to recover, so handing over is the better answer. Checking
    // airspeed loss first would convert "arrived slightly slow" into a
    // 40 second detour.
    //
    // This assertion pins an order that looks arbitrary to a reader skimming
    // the table, which is exactly why it is worth pinning.
    EXPECT_LT(index_of(FlightPhase::kClimb, FlightPhase::kCruise),
              index_of(FlightPhase::kClimb, FlightPhase::kGoAround))
        << "climb capture is checked before the go-around trigger";
}

TEST(PhaseTable, LandingGuardsAreMutuallyExclusiveSoTheirOrderDoesNotMatter)
{
    // Unlike the climb pair, the two landing guards partition the touchdown
    // condition exactly on the speed limit, so neither can preempt the other and
    // the table order is not load bearing. Asserting the partition here rather
    // than an index is what makes that claim checkable: if someone rewrites
    // `guard_touchdown_ok` to include the rollout timer in its *positive*
    // condition, this is the test that says so.
    const AircraftParameters parameters{};
    const MissionProfile     profile{};
    AircraftState            state{};

    for (const Scalar ias : {profile.touchdown_ias_mps, profile.max_touchdown_ias_mps - 1.0F,
                             profile.max_touchdown_ias_mps, profile.max_touchdown_ias_mps + 5.0F})
    {
        state.position_m.y = 0.5F;
        state.ias_mps      = ias;

        const PhaseContext early{state, parameters, profile, 0.0F, 0.0F};
        const PhaseContext stopped{state, parameters, profile, 5.0F, 0.0F};

        const bool ok_early    = guard_touchdown_ok(early);
        const bool missed      = guard_touchdown_missed(early);
        const bool ok_stopped  = guard_touchdown_ok(stopped);
        const bool missed_late = guard_touchdown_missed(stopped);

        EXPECT_FALSE(ok_early && missed)
            << "both landing guards true at ias=" << static_cast<double>(ias);
        EXPECT_FALSE(ok_stopped && missed_late);
        EXPECT_TRUE(ok_stopped || missed_late)
            << "neither landing guard true at ias=" << static_cast<double>(ias)
            << ": the aircraft would sit in landing for ever";
    }
}

TEST(PhaseTable, EveryTransitionCarriesANonEmptyReason)
{
    // The reason strings are printed in the phase trace. An empty one produces a
    // trace with blank cells that nobody can act on.
    for (const auto& transition : kPhaseTransitions)
    {
        ASSERT_NE(transition.reason, nullptr);
        EXPECT_FALSE(std::string(transition.reason).empty())
            << to_string(transition.from) << " -> " << to_string(transition.to);
        EXPECT_NE(transition.guard, nullptr);
    }
}

TEST(PhaseTable, TheAmbientReasonConstantsAreDistinct)
{
    // These are the two reasons that are *not* table entries. If they collided
    // with a table reason, the trace could not tell a latched fault from an
    // operator override.
    EXPECT_STRNE(kFaultReason, kOperatorReason);

    for (const auto& transition : kPhaseTransitions)
    {
        EXPECT_STRNE(transition.reason, kFaultReason);
        EXPECT_STRNE(transition.reason, kOperatorReason);
    }
}

} // namespace
} // namespace fse