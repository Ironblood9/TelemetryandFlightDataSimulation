// ---------------------------------------------------------------------------
// End to end simulation tests.
//
// These assert the behaviour of the whole stack -- phase machine, guidance, flight
// model and sensors together -- over a complete mission. They are the tests that
// would catch a regression a user would notice, as opposed to the unit tests,
// which catch one a reviewer would notice.
//
// The individual modules are covered in their own files. What can only be seen
// from here is the *composition*: whether the phase schedule the machine runs
// and the flight profile the guidance produces are the same mission, and whether
// anything in the stack drifts over a few hundred simulated seconds.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "fse/simulation.hpp"

namespace fse
{
namespace
{
/// Converts a duration in seconds into a whole number of simulation steps.
///
/// The cast is explicit rather than implicit because the timestep is single
/// precision: writing `seconds / simulation.dt()` mixes a double literal with a
/// float and trips -Wdouble-promotion, which this project compiles with.
[[nodiscard]] std::uint64_t steps_for(double seconds, Scalar dt)
{
    return static_cast<std::uint64_t>(seconds / static_cast<double>(dt));
}

/// Flies @p seconds and returns the visited phases in order of first visit.
struct RunSummary
{
    std::vector<FlightPhase> phases;
    std::vector<PhaseChange> changes;
    Scalar                   max_altitude_m{0.0F};
    Scalar                   max_ias_mps{0.0F};
    Scalar                   max_load_factor{0.0F};
    Scalar                   min_load_factor{1.0e9F};
    Scalar                   max_engine_temp_c{0.0F};
    Scalar                   touchdown_ias_mps{0.0F};
    Scalar                   touchdown_altitude_m{0.0F};
    bool                     touchdown_recorded{false};
    std::uint64_t            samples{0};
    std::set<std::uint16_t>  status_bits_seen;
};

[[nodiscard]] RunSummary fly(std::uint64_t seed, double seconds,
                             FlightPhase initial_phase = FlightPhase::kPreflight,
                             FaultCode   fault         = FaultCode::kNone)
{
    SimulationConfig config;
    config.seed          = seed;
    config.initial_phase = initial_phase;

    Simulation simulation{config};
    if (fault != FaultCode::kNone)
    {
        simulation.latch_fault(fault);
    }

    const auto steps = steps_for(seconds, simulation.dt());

    RunSummary summary;
    summary.phases.push_back(simulation.phase());
    std::size_t observed_changes = 0U;

    for (std::uint64_t i = 0; i < steps; ++i)
    {
        const SensorSample& sample = simulation.step();

        summary.max_altitude_m = std::max(summary.max_altitude_m, sample.altitude_m);
        summary.max_ias_mps    = std::max(summary.max_ias_mps, sample.airspeed_mps);
        summary.status_bits_seen.insert(sample.status_bits);

        const AircraftState& state = simulation.aircraft_state();
        summary.max_load_factor    = std::max(summary.max_load_factor, state.load_factor);
        summary.min_load_factor    = std::min(summary.min_load_factor, state.load_factor);
        for (const Scalar temperature : sample.engine_temp_c)
        {
            summary.max_engine_temp_c = std::max(summary.max_engine_temp_c, temperature);
        }

        const auto& changes = simulation.phase_changes();
        while (observed_changes < changes.size())
        {
            summary.phases.push_back(changes[observed_changes].to);
            ++observed_changes;
        }

        // Captured once, on the first sample in the landing phase with the wheels on
        // the runway. The guard used to be `touchdown_altitude_m > 1.0F`, which is
        // a "have I captured yet" test written against a field that starts at
        // zero -- so the condition was never true and the touchdown speed was
        // never recorded at all. Nothing asserted on it, which is how it
        // survived.
        if (!summary.touchdown_recorded && state.phase == FlightPhase::kLanding &&
            state.position_m.y <= 0.5F)
        {
            summary.touchdown_altitude_m = state.ias_mps;
            summary.touchdown_recorded   = true;
        }

        ++summary.samples;
    }

    summary.changes = simulation.phase_changes();
    return summary;
}

[[nodiscard]] bool visited(const RunSummary& summary, FlightPhase phase)
{
    return std::find(summary.phases.begin(), summary.phases.end(), phase) != summary.phases.end();
}

// --- The whole mission ------------------------------------------------------

TEST(Simulation, CompletesAFullTakeoffAndLanding)
{
    // The default profile is sized so that 400 seconds at 100 Hz covers the
    // whole sortie. A shorter run would not reach the runway.
    const RunSummary summary = fly(20241001U, 400.0);

    SCOPED_TRACE("phases: " + std::to_string(summary.phases.size()));

    EXPECT_TRUE(visited(summary, FlightPhase::kTaxi)) << "never taxied";
    EXPECT_TRUE(visited(summary, FlightPhase::kTakeoffRoll)) << "never took off";
    EXPECT_TRUE(visited(summary, FlightPhase::kClimb)) << "never climbed";
    EXPECT_TRUE(visited(summary, FlightPhase::kCruise)) << "never reached cruise";
    EXPECT_TRUE(visited(summary, FlightPhase::kDescent)) << "never descended";
    EXPECT_TRUE(visited(summary, FlightPhase::kApproach)) << "never approached";

    EXPECT_GT(summary.max_altitude_m, 800.0F) << "climb did not reach the cruise altitude";
    EXPECT_GT(summary.max_ias_mps, 80.0F) << "never reached cruise speed";

    EXPECT_FALSE(summary.changes.empty());
}

TEST(Simulation, CompletesTheSortieInTheOrderTheTableDescribes)
{
    // Not just "it got there" but "it got there in the order the schedule says".
    // A phase table with a one-way edge or a reordered pair can produce a
    // mission that completes and is still wrong, and the sequence is the only
    // thing that shows it.
    const RunSummary summary = fly(20241001U, 400.0);

    const std::vector<FlightPhase> expected{FlightPhase::kPreflight,   FlightPhase::kTaxi,
                                            FlightPhase::kTakeoffRoll, FlightPhase::kClimb,
                                            FlightPhase::kCruise,      FlightPhase::kDescent,
                                            FlightPhase::kApproach};

    ASSERT_GE(summary.phases.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        EXPECT_EQ(summary.phases[i], expected[i]) << "at position " << i;
    }
}

TEST(Simulation, TouchdownIsInsideTheSpeedTheProfileAllows)
{
    // The landing phase has two exits and they partition on this number, so a
    // run that arrives above the limit ends in a go-around instead of a taxi.
    // Completing the mission without ever reaching a taxi after landing *is* the
    // assertion that this number was respected.
    const RunSummary summary = fly(20241001U, 400.0);

    ASSERT_TRUE(summary.touchdown_recorded) << "the aircraft never touched down";
    EXPECT_LE(summary.touchdown_altitude_m, 60.0F)
        << "touched down at " << summary.touchdown_altitude_m << " m/s";
}

TEST(Simulation, ClimbTargetAltitudeIsActuallyReached)
{
    SimulationConfig config;
    config.seed = 99U;

    Simulation       simulation{config};
    constexpr double kSeconds            = 200.0;
    const auto       steps               = steps_for(kSeconds, simulation.dt());
    Scalar           best_climb_altitude = 0.0F;
    bool             was_climbing        = false;

    for (std::uint64_t i = 0; i < steps; ++i)
    {
        const SensorSample& sample = simulation.step();
        if (sample.phase == FlightPhase::kClimb)
        {
            was_climbing        = true;
            best_climb_altitude = std::max(best_climb_altitude, sample.altitude_m);
        }
    }

    EXPECT_TRUE(was_climbing);
    EXPECT_NEAR(best_climb_altitude, config.profile.climb_altitude_m, 60.0F)
        << "the altitude hold did not converge on its setpoint";
}

TEST(Simulation, CruiseHoldsAltitudeWithinTheCommittedBand)
{
    // The regression this guards against is oscillation: an altitude hold that
    // passes *through* its setpoint forever technically reaches it every cycle.
    // The band width is the assertion; the mean alone would not catch it.
    SimulationConfig config;
    config.seed          = 4242U;
    config.initial_phase = FlightPhase::kCruise;
    // Long cruise so the transient out of the entry is excluded from the sample.
    config.profile.cruise_duration_s = 400.0F;

    Simulation simulation{config};

    constexpr double kWarmup  = 40.0;
    constexpr double kMeasure = 120.0;

    const auto warmup_steps = steps_for(kWarmup, simulation.dt());
    for (std::uint64_t i = 0; i < warmup_steps; ++i)
    {
        (void)simulation.step();
    }

    const auto measure_steps = steps_for(kMeasure, simulation.dt());
    Scalar     minimum       = 1.0e9F;
    Scalar     maximum       = -1.0e9F;

    for (std::uint64_t i = 0; i < measure_steps; ++i)
    {
        const SensorSample& sample = simulation.step();
        minimum                    = std::min(minimum, sample.altitude_m);
        maximum                    = std::max(maximum, sample.altitude_m);
    }

    const Scalar spread = maximum - minimum;
    EXPECT_NEAR((minimum + maximum) / 2.0F, config.profile.cruise_altitude_m, 40.0F);
    EXPECT_LT(spread, 60.0F) << "the altitude hold is oscillating: band width " << spread << " m";
}

TEST(Simulation, CruiseShowsACoordinatedTurnWithTheExpectedLoadFactor)
{
    SimulationConfig config;
    config.seed                      = 55U;
    config.initial_phase             = FlightPhase::kCruise;
    config.profile.cruise_duration_s = 200.0F;
    config.wind_enabled              = false;

    Simulation simulation{config};

    constexpr double kWarmup      = 30.0;
    const auto       warmup_steps = steps_for(kWarmup, simulation.dt());
    for (std::uint64_t i = 0; i < warmup_steps; ++i)
    {
        (void)simulation.step();
    }

    // Not constexpr: std::cos is not a constant expression.
    const Scalar kExpectedLoadFactor =
        1.0F / std::cos(deg_to_rad(config.profile.cruise_turn_bank_deg));

    Scalar peak_load_factor = 0.0F;
    Scalar peak_bank        = 0.0F;

    const auto steps = steps_for(60.0, simulation.dt());
    for (std::uint64_t i = 0; i < steps; ++i)
    {
        const SensorSample&  sample = simulation.step();
        const AircraftState& state  = simulation.aircraft_state();
        peak_load_factor            = std::max(peak_load_factor, state.load_factor);
        peak_bank                   = std::max(peak_bank, std::abs(sample.roll_deg));
    }

    EXPECT_GT(peak_bank, 25.0F) << "the cruise turn never established its bank angle";
    EXPECT_NEAR(peak_load_factor, kExpectedLoadFactor, 0.25F)
        << "load factor in a 30 degree bank must be 1/cos(30) = 1.155";
}

TEST(Simulation, TheCruiseTurnIsFollowedByAReturnToTheLegHeading)
{
    // The heading loop exists to bring the aircraft back after the inserted turn.
    // Without it the aircraft banks for the turn, changes heading by twice the
    // bank angle and then flies the rest of the cruise leg on the wrong course,
    // which no other test would notice: the mission still completes.
    SimulationConfig config;
    config.seed                      = 55U;
    config.initial_phase             = FlightPhase::kCruise;
    config.profile.cruise_duration_s = 400.0F;
    config.wind_enabled              = false;

    Simulation simulation{config};

    const Scalar kTurnStart = 0.25F * config.profile.cruise_duration_s;
    const Scalar kTurnEnd   = kTurnStart + config.profile.cruise_turn_duration_s;

    Scalar heading_during_turn = 0.0F;
    Scalar heading_late_leg    = 0.0F;
    bool   captured_turn       = false;

    const auto steps = steps_for(300.0, simulation.dt());
    for (std::uint64_t i = 0; i < steps; ++i)
    {
        (void)simulation.step();
        // Both operands cast explicitly: `i * dt` mixes an int with a float and the
        // result is then narrowed back to Scalar, which trips -Wdouble-promotion on the
        // compilers that enable it. MSVC does not warn here, which is exactly why the
        // CI matrix includes gcc and clang.
        const Scalar elapsed =
            static_cast<Scalar>(static_cast<double>(i) * static_cast<double>(simulation.dt()));

        if (elapsed > kTurnStart && elapsed < kTurnEnd)
        {
            heading_during_turn = simulation.aircraft_state().heading_deg;
            captured_turn       = true;
        }
        else if (elapsed > (kTurnEnd + 60.0F))
        {
            heading_late_leg = simulation.aircraft_state().heading_deg;
        }
    }

    ASSERT_TRUE(captured_turn);
    EXPECT_GT(
        std::abs(angle_difference_deg(heading_during_turn, config.profile.cruise_heading_deg)),
        5.0F)
        << "the cruise turn never changed the heading at all";

    // The leg the autopilot returns to is the reciprocal course, not the
    // original one: the inserted turn makes the cruise leg a racetrack, so
    // after the 90 degree leg change the aircraft flies the opposite way and
    // the pattern closes.
    const Scalar leg_heading = wrap_degrees(config.profile.cruise_heading_deg + 180.0F);

    EXPECT_LT(std::abs(angle_difference_deg(heading_late_leg, leg_heading)), 15.0F)
        << "the aircraft did not return to the leg heading after the turn";
}

TEST(Simulation, EngineTemperaturesLagTheThrottle)
{
    // The whole reason the engine monitor exists. A channel that tracked the
    // throttle instantly would be useless for health monitoring.
    SimulationConfig config;
    config.seed          = 1717U;
    config.initial_phase = FlightPhase::kTakeoffRoll;

    Simulation simulation{config};

    Scalar temperature_at_rotation = 0.0F;
    Scalar throttle_at_rotation    = 0.0F;
    bool   captured                = false;

    const auto steps = steps_for(120.0, simulation.dt());
    for (std::uint64_t i = 0; i < steps; ++i)
    {
        const SensorSample& sample = simulation.step();

        if (!captured && sample.airspeed_mps >= config.parameters.rotate_ias_mps)
        {
            temperature_at_rotation =
                *std::max_element(sample.engine_temp_c.begin(), sample.engine_temp_c.end());
            throttle_at_rotation = simulation.aircraft_state().throttle;
            captured             = true;
        }

        ASSERT_LE(sample.engine_temp_c[0], 1200.0F) << "temperature ran away";
    }

    ASSERT_TRUE(captured);
    EXPECT_GT(throttle_at_rotation, 0.6F) << "rotation happened too early to judge";

    // Throttle is at its maximum but the exhaust is still climbing towards it.
    EXPECT_LT(temperature_at_rotation, 700.0F)
        << "the thermal lag is not present, or rotation took far too long";
}

// --- Faults -----------------------------------------------------------------

TEST(Simulation, EngineFireDrivesTheFailurePhaseAndInvalidatesTheEngineChannel)
{
    // The chain here is the point: a latched fault has to reach the phase
    // machine, the machine has to report the reason, and the sensor suite has to
    // invalidate the engine channel. Testing only any one of the three would
    // pass with either of the other two disconnected.
    SimulationConfig config;
    config.seed          = 808U;
    config.initial_phase = FlightPhase::kCruise;

    Simulation simulation{config};
    for (int i = 0; i < 200; ++i)
    {
        (void)simulation.step();
    }

    simulation.latch_fault(FaultCode::kEngineFire);

    bool   saw_failure         = false;
    bool   saw_engine_degraded = false;
    Scalar peak_temperature    = 0.0F;

    const auto steps = steps_for(60.0, simulation.dt());
    for (std::uint64_t i = 0; i < steps; ++i)
    {
        const SensorSample& sample = simulation.step();
        if (sample.phase == FlightPhase::kFailure)
        {
            saw_failure = true;
        }
        if (has_status(static_cast<SensorStatus>(sample.status_bits),
                       SensorStatus::kEngineDegraded))
        {
            saw_engine_degraded = true;
        }
        for (const Scalar temperature : sample.engine_temp_c)
        {
            peak_temperature = std::max(peak_temperature, temperature);
        }
    }

    EXPECT_TRUE(saw_failure) << "the fault did not drive the failure phase";
    EXPECT_TRUE(saw_engine_degraded) << "the engine channel was still reported good";
    EXPECT_GT(peak_temperature, EngineMonitor::kRedlineTemperatureC * 0.8F);
}

TEST(Simulation, TheFaultIsReportedInThePhaseTraceWithAReason)
{
    SimulationConfig config;
    config.seed          = 808U;
    config.initial_phase = FlightPhase::kCruise;

    Simulation simulation{config};
    for (int i = 0; i < 200; ++i)
    {
        (void)simulation.step();
    }

    const std::size_t before = simulation.phase_changes().size();
    simulation.latch_fault(FaultCode::kEngineFire);
    (void)simulation.step();

    ASSERT_EQ(simulation.phase_changes().size(), before + 1U);
    const PhaseChange& change = simulation.phase_changes().back();
    EXPECT_EQ(change.to, FlightPhase::kFailure);
    EXPECT_STREQ(change.reason, "fault latched");
}

TEST(Simulation, EngineOutRemovesThrust)
{
    SimulationConfig config;
    config.seed          = 909U;
    config.initial_phase = FlightPhase::kCruise;

    Simulation simulation{config};
    for (int i = 0; i < 300; ++i)
    {
        (void)simulation.step();
    }
    ASSERT_GT(simulation.aircraft_state().throttle, 0.1F);

    simulation.latch_fault(FaultCode::kEngineOut);
    // The 1.8 s spool lag decays exponentially, so the throttle needs several
    // time constants before it is negligible.
    for (int i = 0; i < 1500; ++i)
    {
        (void)simulation.step();
    }

    EXPECT_NEAR(simulation.aircraft_state().throttle, 0.0F, 0.01F);
}

// --- Reproducibility ---------------------------------------------------------

TEST(Simulation, TheSameSeedProducesTheSamePhaseSequence)
{
    const RunSummary first  = fly(11U, 60.0);
    const RunSummary second = fly(11U, 60.0);

    ASSERT_EQ(first.changes.size(), second.changes.size());
    for (std::size_t i = 0; i < first.changes.size(); ++i)
    {
        EXPECT_EQ(first.changes[i].to, second.changes[i].to);
        EXPECT_STREQ(first.changes[i].reason, second.changes[i].reason);
    }
}

TEST(Simulation, NoWindMakesTheRunSimplerButNotTrivial)
{
    // The model must not depend on the wind being present, and turning it off
    // must actually change nothing about the reproducibility guarantees.
    const RunSummary with_wind = fly(11U, 60.0);
    const RunSummary again     = fly(11U, 60.0);

    ASSERT_EQ(with_wind.changes.size(), again.changes.size());
    for (std::size_t i = 0; i < with_wind.changes.size(); ++i)
    {
        EXPECT_EQ(with_wind.changes[i].to, again.changes[i].to);
    }
}

TEST(Simulation, TheStatusFieldIsNeverSilentlyZero)
{
    // Every sample in a real run must carry *some* quality information. A suite
    // that only ever reported kGood would be indistinguishable from one that had
    // been disconnected, which is the failure mode a quality field exists to
    // prevent.
    const RunSummary summary = fly(20241001U, 400.0);

    EXPECT_GT(summary.samples, 0U);
    EXPECT_LT(summary.status_bits_seen.size(), 8U)
        << "the status field never changed: the quality bits are not being driven";
}

TEST(Simulation, NothingGoesNonFiniteOverAFullMission)
{
    // The cheapest end-to-end guard there is, and the one that catches a NaN
    // before it silently poisons a whole trace.
    const RunSummary summary = fly(4711U, 400.0);

    EXPECT_GT(summary.max_altitude_m, 0.0F);
    EXPECT_TRUE(std::isfinite(summary.max_altitude_m));
    EXPECT_TRUE(std::isfinite(summary.max_ias_mps));
    EXPECT_TRUE(std::isfinite(summary.max_load_factor));
    EXPECT_TRUE(std::isfinite(summary.max_engine_temp_c));
    // The lift curve is linear and unclamped, so a wing at a negative angle of
    // attack genuinely produces negative lift -- which is real, not a bug. A
    // climb pulls the nose up, the flight path catches up past it, and for a
    // few tenths of a second the aircraft is unloaded below 1 g. What is not
    // acceptable is the aircraft inverting, which is what these bounds catch.
    EXPECT_GT(summary.min_load_factor, -0.5F) << "the aircraft was in negative-g flight";
    EXPECT_LE(summary.max_load_factor, 3.5F) << "an impossible load factor was recorded";
}

// --- Orchestration ----------------------------------------------------------

TEST(Simulation, TheStepCountAndTimestampAdvanceTogether)
{
    // The sequence number and the timestamp belong to the orchestrator, not to
    // the instruments. If either stopped advancing, a receiver could not detect
    // a stall in the link -- which is the whole reason they are on the wire.
    SimulationConfig config;
    config.seed = 3U;

    Simulation simulation{config};

    SensorSample previous{};
    for (int i = 0; i < 100; ++i)
    {
        const SensorSample& sample = simulation.step();

        if (i > 0)
        {
            EXPECT_EQ(sample.sequence, previous.sequence + 1U) << "at step " << i;
            EXPECT_GT(sample.timestamp_ns, previous.timestamp_ns)
                << "timestamp did not advance at step " << i;
        }
        previous = sample;
    }
}

TEST(Simulation, TheTimestampFollowsTheConfiguredRate)
{
    SimulationConfig config;
    config.seed    = 3U;
    config.rate_hz = 100U;

    Simulation simulation{config};
    (void)simulation.step();

    SensorSample sample{};
    for (int i = 0; i < 1000; ++i)
    {
        sample = simulation.step();
    }

    // 1001 samples at 100 Hz is ten seconds. Anything else means the reported
    // rate and the simulated rate have drifted apart.
    EXPECT_NEAR(static_cast<double>(sample.timestamp_ns) / 1.0e9, 10.01, 0.05);
}

TEST(SimulationConfig, ClampsTheRateToASaneRange)
{
    // Zero would be a division by zero in the timestep, and 60000 Hz is a
    // thousand times faster than the protocol needs and would make a nominal
    // hour long run take three and a half seconds of wall clock.
    {
        SimulationConfig config;
        config.rate_hz = 0U;
        const Simulation simulation{config};
        EXPECT_EQ(simulation.rate_hz(), 1U);
        EXPECT_FLOAT_EQ(simulation.dt(), 1.0F);
    }
    {
        SimulationConfig config;
        config.rate_hz = 60000U;
        const Simulation simulation{config};
        EXPECT_EQ(simulation.rate_hz(), 1000U);
        EXPECT_NEAR(static_cast<double>(simulation.dt()), 0.001, 1.0e-9);
    }
}

TEST(SimulationConfig, ResetClearsTheRecordedPhaseHistory)
{
    // Leaving a stale phase history behind after a reset is worse than not
    // resetting at all: the next run's trace would start with the previous run's
    // tail, and a reader would attribute it to the aircraft.
    SimulationConfig config;
    config.seed = 1U;

    Simulation simulation{config};
    for (int i = 0; i < 1000; ++i)
    {
        (void)simulation.step();
    }
    ASSERT_FALSE(simulation.phase_changes().empty());

    simulation.reset();

    EXPECT_TRUE(simulation.phase_changes().empty());
    EXPECT_EQ(simulation.step_index(), 0U);
    EXPECT_EQ(simulation.phase(), FlightPhase::kPreflight);
}

TEST(Simulation, ForcePhaseIsRecordedLikeAnyOtherChange)
{
    // An operator command that bypassed the phase trace would be invisible in
    // the one artefact a reviewer reads to explain why the aircraft did what it
    // did.
    SimulationConfig config;
    config.seed = 2U;

    Simulation simulation{config};
    simulation.force_phase(FlightPhase::kCruise);

    ASSERT_EQ(simulation.phase_changes().size(), 1U);
    EXPECT_EQ(simulation.phase_changes()[0].to, FlightPhase::kCruise);
    EXPECT_STREQ(simulation.phase_changes()[0].reason, "operator command");

    for (int i = 0; i < 100; ++i)
    {
        (void)simulation.step();
    }
    EXPECT_EQ(simulation.phase(), FlightPhase::kCruise);
}

TEST(Simulation, AnAirborneStartIsPlacedInTheAir)
{
    // Scenario setup, not physics: a run that starts in cruise has to begin in
    // the air, because no airborne phase commands a take-off throttle and a
    // "cruise" scenario placed on the runway would sit there for ever.
    SimulationConfig config;
    config.seed          = 5U;
    config.initial_phase = FlightPhase::kCruise;

    const Simulation simulation{config};

    EXPECT_GT(simulation.aircraft_state().position_m.y, 100.0F);
    EXPECT_GT(simulation.aircraft_state().tas_mps, 50.0F);
}

} // namespace
} // namespace fse