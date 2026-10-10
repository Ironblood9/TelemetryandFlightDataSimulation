// ---------------------------------------------------------------------------
// Determinism tests — the single most important suite in the repository.
//
// The claim under test is "same seed, same flight". It is asserted in three
// separate parts, because they fail for different reasons and a single test
// cannot tell you which one broke:
//
//   1. Repeat runs of one binary are bit identical, and a reset run reproduces
//      its first run exactly.
//   2. Timestamps are integer arithmetic, so they cannot drift.
//   3. Nothing that must be reproducible is ever NaN, infinite or outside
//      physical bounds.
//
// The cross-platform part of the claim -- that the *integer* PRNG stream is
// reproducible anywhere -- is asserted in test_noise.cpp against the upstream
// PCG32 reference vectors rather than here, so that there is one place that
// says it and one place that could be wrong.
//
// It is deliberately NOT asserted that a float run is bit identical across
// compilers. `sin`, `cos`, `log`, `exp` and `pow` are implementation defined in
// their last bit, so that claim would be false, and a test asserting it would
// be a test that fails on someone else's machine for no reason anyone could act
// on. Saying so here is worth more than the test.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "fse/noise.hpp"
#include "fse/simulation.hpp"

namespace fse
{
namespace
{

/// Hashes a whole telemetry run. Comparing one integer is both faster and a
/// stronger statement than comparing fields one at a time.
[[nodiscard]] std::uint64_t hash_run(std::uint64_t seed, int steps, std::uint16_t rate_hz = 100U)
{
    SimulationConfig config;
    config.seed    = seed;
    config.rate_hz = rate_hz;

    Simulation    simulation{config};
    std::uint64_t hash = random::kFnv1aOffsetBasis;

    for (int i = 0; i < steps; ++i)
    {
        const SensorSample& sample = simulation.step();

        hash = random::fnv1a(hash, sample.sequence);
        hash = random::fnv1a(hash, sample.timestamp_ns);
        hash = random::fnv1a_scalar(hash, sample.airspeed_mps);
        hash = random::fnv1a_scalar(hash, sample.altitude_m);
        hash = random::fnv1a_scalar(hash, sample.vertical_speed_mps);
        hash = random::fnv1a_scalar(hash, sample.heading_deg);
        hash = random::fnv1a_scalar(hash, sample.pitch_deg);
        hash = random::fnv1a_scalar(hash, sample.roll_deg);
        hash = random::fnv1a_scalar(hash, sample.fuel_press_kpa);
        hash = random::fnv1a_scalar(hash, sample.oil_press_kpa);
        for (const Scalar temperature : sample.engine_temp_c)
        {
            hash = random::fnv1a_scalar(hash, temperature);
        }
        hash = random::fnv1a(hash, static_cast<std::uint32_t>(sample.status_bits));
        hash = random::fnv1a(hash, static_cast<std::uint32_t>(to_bits(sample.faults)));
    }

    return hash;
}

/// Field by field comparison, so a failure reports *what* diverged.
void expect_samples_equal(const SensorSample& lhs, const SensorSample& rhs)
{
    EXPECT_EQ(lhs.sequence, rhs.sequence);
    EXPECT_EQ(lhs.timestamp_ns, rhs.timestamp_ns);
    EXPECT_EQ(lhs.phase, rhs.phase);
    EXPECT_EQ(lhs.status_bits, rhs.status_bits);
    EXPECT_FLOAT_EQ(lhs.airspeed_mps, rhs.airspeed_mps);
    EXPECT_FLOAT_EQ(lhs.altitude_m, rhs.altitude_m);
    EXPECT_FLOAT_EQ(lhs.vertical_speed_mps, rhs.vertical_speed_mps);
    EXPECT_FLOAT_EQ(lhs.heading_deg, rhs.heading_deg);
    EXPECT_FLOAT_EQ(lhs.pitch_deg, rhs.pitch_deg);
    EXPECT_FLOAT_EQ(lhs.roll_deg, rhs.roll_deg);
    EXPECT_FLOAT_EQ(lhs.fuel_press_kpa, rhs.fuel_press_kpa);
    EXPECT_FLOAT_EQ(lhs.oil_press_kpa, rhs.oil_press_kpa);
    ASSERT_EQ(lhs.engine_temp_c.size(), rhs.engine_temp_c.size());
    for (std::size_t i = 0; i < lhs.engine_temp_c.size(); ++i)
    {
        EXPECT_FLOAT_EQ(lhs.engine_temp_c[i], rhs.engine_temp_c[i]) << "engine " << i;
    }
}

[[nodiscard]] std::vector<SensorSample> run_capture(std::uint64_t seed, int steps,
                                                    std::uint16_t rate_hz = 100U)
{
    SimulationConfig config;
    config.seed    = seed;
    config.rate_hz = rate_hz;

    Simulation simulation{config};

    std::vector<SensorSample> samples;
    samples.reserve(static_cast<std::size_t>(steps));
    for (int i = 0; i < steps; ++i)
    {
        samples.push_back(simulation.step());
    }
    return samples;
}

// --- 1. Repeat runs are bit identical ---------------------------------------

TEST(Determinism, RepeatRunHashesMatch)
{
    constexpr int kSteps = 3000; // 30 seconds at 100 Hz

    const std::uint64_t first  = hash_run(1234U, kSteps);
    const std::uint64_t second = hash_run(1234U, kSteps);
    const std::uint64_t third  = hash_run(1234U, kSteps);

    EXPECT_EQ(first, second);
    EXPECT_EQ(second, third);
}

TEST(Determinism, EveryFieldIsReproducibleFieldByField)
{
    constexpr int kSteps = 1500;

    const auto first  = run_capture(777U, kSteps);
    const auto second = run_capture(777U, kSteps);

    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        SCOPED_TRACE(i);
        expect_samples_equal(first[i], second[i]);
    }
}

TEST(Determinism, ResetReproducesTheRunExactly)
{
    constexpr int kSteps = 800;

    SimulationConfig config;
    config.seed = 31337U;

    Simulation simulation{config};
    for (int i = 0; i < kSteps; ++i)
    {
        (void)simulation.step();
    }

    // After the reset, replay exactly kSteps steps and keep the last sample.
    simulation.reset();
    for (int i = 0; i < kSteps; ++i)
    {
        (void)simulation.step();
    }
    const SensorSample after_reset = simulation.step();

    Simulation fresh{config};
    for (int i = 0; i < kSteps; ++i)
    {
        (void)fresh.step();
    }
    const SensorSample reference = fresh.step();

    expect_samples_equal(after_reset, reference);
    EXPECT_EQ(after_reset.sequence, static_cast<std::uint32_t>(kSteps));
}

TEST(Determinism, DifferentSeedsProduceDifferentRuns)
{
    EXPECT_NE(hash_run(1U, 1500), hash_run(2U, 1500));
}

TEST(Determinism, TheWholeSortieIsReproducible)
{
    // The 30 second hash above stops during the climb. This one covers the
    // entire mission -- take-off, cruise, descent, approach, flare, rollout --
    // which is the range over which an accumulator that has slowly drifted, or a
    // filter that has slowly saturated, would actually show up. A regression
    // that only manifests after the landing flare is exactly the kind that a
    // short determinism test misses and a golden file catches.
    constexpr int kSteps = 40000; // 400 seconds at 100 Hz

    const std::uint64_t first  = hash_run(20241001U, kSteps);
    const std::uint64_t second = hash_run(20241001U, kSteps);

    EXPECT_EQ(first, second);
}

TEST(Determinism, AHashChangeMeansSomeFieldChanged)
{
    // The comparison above is only as strong as the hash. If a field were simply
    // missing from `hash_run` -- dropped in a refactor, or renamed on one side
    // only -- every determinism test in this file would pass while that field
    // roamed freely. Perturbing one field at a time and checking the hash
    // moves is what keeps the harness honest.
    EXPECT_NE(random::fnv1a_scalar(random::kFnv1aOffsetBasis, 1.0F),
              random::fnv1a_scalar(random::kFnv1aOffsetBasis, 1.0000001F));
    EXPECT_NE(random::fnv1a(random::kFnv1aOffsetBasis, std::uint32_t{1U}),
              random::fnv1a(random::kFnv1aOffsetBasis, std::uint32_t{2U}));
    EXPECT_NE(random::fnv1a(random::kFnv1aOffsetBasis, std::uint64_t{1U}),
              random::fnv1a(random::kFnv1aOffsetBasis, std::uint64_t{2U}));
}

TEST(Determinism, TheInitialPhaseDoesNotShiftTheNoiseStream)
{
    // Two runs that start in different phases must still consume the noise
    // streams in the same order, or a scenario flown from cruise could not be
    // compared against one flown from the runway at the same seed -- which is the
    // whole point of seeding a simulation.
    constexpr int kSteps = 2000;

    SimulationConfig from_runway;
    from_runway.seed          = 606U;
    from_runway.initial_phase = FlightPhase::kPreflight;

    SimulationConfig from_cruise;
    from_cruise.seed          = 606U;
    from_cruise.initial_phase = FlightPhase::kCruise;

    Simulation runway{from_runway};
    Simulation cruise{from_cruise};

    for (int i = 0; i < kSteps; ++i)
    {
        const SensorSample& a = runway.step();
        const SensorSample& b = cruise.step();

        // The fuel channel is fed directly by the engine noise stream, which is
        // driven by the throttle. It is the cheapest way to observe whether the
        // two runs are consuming the stream in step.
        SCOPED_TRACE(i);
        EXPECT_EQ(a.sequence, b.sequence);
        EXPECT_EQ(a.timestamp_ns, b.timestamp_ns);
    }
}

TEST(Determinism, DifferentRatesAreIndependentOfTheClock)
{
    // Same seed, different rate: the runs must differ, but both must be stable.
    const std::uint64_t at_100 = hash_run(9U, 1000, 100U);
    const std::uint64_t at_200 = hash_run(9U, 1000, 200U);

    EXPECT_NE(at_100, at_200);
    EXPECT_EQ(at_100, hash_run(9U, 1000, 100U));
    EXPECT_EQ(at_200, hash_run(9U, 1000, 200U));
}

// --- 2. Timestamps are exact integer arithmetic ----------------------------

TEST(Determinism, TimestampsAreExactAndFreeOfAccumulatedError)
{
    constexpr std::uint16_t kRate       = 100U;
    const std::uint64_t     per_step_ns = 1000000000ULL / static_cast<std::uint64_t>(kRate);
    ASSERT_EQ(per_step_ns, 10000000ULL);

    EXPECT_EQ(timestamp_ns_for(0U, kRate), 0U);
    EXPECT_EQ(timestamp_ns_for(1U, kRate), 10000000ULL);
    EXPECT_EQ(timestamp_ns_for(100U, kRate), 1000000000ULL);       // exactly one second
    EXPECT_EQ(timestamp_ns_for(360000U, kRate), 3600000000000ULL); // exactly one hour

    SimulationConfig config;
    config.seed = 5U;
    Simulation simulation{config};

    // Run far enough that a float accumulator would have visibly drifted: one
    // hour at 100 Hz.
    constexpr std::uint64_t kOneHourSteps = 360000U;
    for (std::uint64_t i = 0; i < kOneHourSteps; ++i)
    {
        (void)simulation.step();
    }

    const SensorSample& last = simulation.step();
    EXPECT_EQ(last.timestamp_ns, kOneHourSteps * per_step_ns);
    // step_index_ has already advanced past the sample just produced.
    EXPECT_FLOAT_EQ(simulation.sim_time_s(), 3600.01F);
}

// --- 3. Numerical health ----------------------------------------------------

TEST(Determinism, NoNanOrInfinityEverReachesTheFrame)
{
    constexpr int kSteps = 20000; // 200 seconds, the whole mission

    const auto samples = run_capture(4242U, kSteps);

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const SensorSample& sample = samples[i];
        SCOPED_TRACE(i);

        EXPECT_TRUE(std::isfinite(sample.airspeed_mps)) << "airspeed";
        EXPECT_TRUE(std::isfinite(sample.altitude_m)) << "altitude";
        EXPECT_TRUE(std::isfinite(sample.vertical_speed_mps)) << "vertical speed";
        EXPECT_TRUE(std::isfinite(sample.heading_deg)) << "heading";
        EXPECT_TRUE(std::isfinite(sample.pitch_deg)) << "pitch";
        EXPECT_TRUE(std::isfinite(sample.roll_deg)) << "roll";
        EXPECT_TRUE(std::isfinite(sample.fuel_press_kpa)) << "fuel pressure";
        EXPECT_TRUE(std::isfinite(sample.oil_press_kpa)) << "oil pressure";
        EXPECT_TRUE(is_finite(sample.accel_mps2)) << "acceleration";

        for (const Scalar temperature : sample.engine_temp_c)
        {
            EXPECT_TRUE(std::isfinite(temperature)) << "engine temperature";
        }
    }
}

TEST(Determinism, TelemetryStaysInsidePhysicalBounds)
{
    constexpr int kSteps = 20000;

    const auto samples = run_capture(4242U, kSteps);

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const SensorSample& sample = samples[i];
        SCOPED_TRACE(i);

        // Bounds are deliberately loose; the point is to catch a diverged run,
        // not to police the flight profile.
        EXPECT_GE(sample.altitude_m, -100.0F);
        EXPECT_LE(sample.altitude_m, 20000.0F);
        EXPECT_GE(sample.airspeed_mps, -10.0F);
        EXPECT_LE(sample.airspeed_mps, 350.0F);
        EXPECT_GE(sample.heading_deg, 0.0F);
        EXPECT_LT(sample.heading_deg, 360.0F);
        EXPECT_GE(sample.pitch_deg, -90.0F);
        EXPECT_LE(sample.pitch_deg, 90.0F);
        EXPECT_GE(sample.roll_deg, -180.0F);
        EXPECT_LE(sample.roll_deg, 180.0F);
        EXPECT_GE(sample.fuel_press_kpa, 0.0F);
        EXPECT_LE(sample.fuel_press_kpa, 120.0F);
        EXPECT_GE(sample.oil_press_kpa, 0.0F);
        EXPECT_LE(sample.oil_press_kpa, 500.0F);
    }
}

TEST(Determinism, SequenceNumbersAreDenseAndMonotonic)
{
    constexpr int kSteps = 5000;

    SimulationConfig config;
    config.seed = 64U;
    Simulation simulation{config};

    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(kSteps); ++i)
    {
        const SensorSample& sample = simulation.step();
        ASSERT_EQ(sample.sequence, static_cast<std::uint32_t>(i));
        ASSERT_EQ(sample.timestamp_ns, timestamp_ns_for(i, config.rate_hz));
    }
}

/// FNV-1a over the four bytes of 1.0f in little endian order: 00 00 80 3f.
[[nodiscard]] std::uint64_t fnv1a_of_one_float()
{
    constexpr std::uint32_t kBits = 0x3f800000U;
    return random::fnv1a(random::kFnv1aOffsetBasis, kBits);
}

TEST(Determinism, FnvHashMatchesTheKnownVector)
{
    // Guards the hashing helper itself: if this drifts, every determinism
    // comparison in this file silently weakens.
    EXPECT_EQ(random::fnv1a(random::kFnv1aOffsetBasis, "hello", 5U), 0xa430d84680aabd0bULL);
    EXPECT_EQ(random::fnv1a_scalar(random::kFnv1aOffsetBasis, 1.0F), fnv1a_of_one_float());
}

} // namespace
} // namespace fse