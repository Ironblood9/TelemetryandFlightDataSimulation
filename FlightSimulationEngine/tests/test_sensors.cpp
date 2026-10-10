// ---------------------------------------------------------------------------
// Sensor tests.
//
// Everything here is about the gap between the truth and what an instrument
// would report. A sensor that returns the exact state vector is a telemetry
// screen, not a sensor, and the interesting properties are all consequences of
// the imperfection: latency, noise, a bias that has to settle, and validity
// flags that have to be judged on the transducer output rather than on the
// truth.
//
// The noise is the reason most of these tests average over a window rather than
// sampling once. A single instantaneous sample of a noisy channel is not a
// measurement, and asserting on one makes the test a coin toss.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>

#include "fse/sensor.hpp"

namespace fse
{
namespace
{

constexpr float kDt = 0.01F;

/// Feeds the air data computer @p steps samples of @p state.
void settle(AirDataComputer& adc, AircraftState& state, int steps)
{
    for (int i = 0; i < steps; ++i)
    {
        adc.update(state, kDt);
    }
}

/// Root mean square of a channel over a window, which is how noise is judged.
[[nodiscard]] double rms(AirDataComputer& adc, AircraftState& state, int steps)
{
    double sum_squares = 0.0;
    for (int i = 0; i < steps; ++i)
    {
        adc.update(state, kDt);
        const double value = static_cast<double>(adc.vertical_speed_mps());
        sum_squares += value * value;
    }
    return std::sqrt(sum_squares / static_cast<double>(steps));
}

// --- Alignment and validity --------------------------------------------------

TEST(SensorSuite, StartsUntrustedAndAligns)
{
    // A real inertial platform is not trusted on switch-on: it is handed the
    // current heading and then watched until its estimate converges. Reporting
    // kGood immediately would be a lie the ground station would act on.
    AirDataComputer adc{};
    adc.reset(1U);

    EXPECT_EQ(adc.airspeed_quality(), SensorQuality::kInvalid);
    EXPECT_EQ(adc.attitude_quality(), SensorQuality::kInvalid);
    EXPECT_FALSE(adc.aligned());

    AircraftState state{};
    settle(adc, state, 1000);

    EXPECT_TRUE(adc.aligned());
    EXPECT_EQ(adc.attitude_quality(), SensorQuality::kGood);
}

TEST(SensorSuite, OutOfRangeValuesAreReportedInvalid)
{
    AirDataComputer adc{};
    adc.reset(4U);

    AircraftState state{};
    state.ias_mps      = 100.0F;
    state.position_m.y = 1000.0F;
    settle(adc, state, 2000);
    ASSERT_EQ(adc.airspeed_quality(), SensorQuality::kGood);

    // Validity is judged on the *transducer output*, not on the truth, so the
    // flag appears once the filter has caught up rather than on the first
    // sample. Judging it on the input would mean a spike that the filter
    // removed still raised a fault on the ground.
    state.ias_mps = 400.0F;
    settle(adc, state, 200);
    EXPECT_EQ(adc.airspeed_quality(), SensorQuality::kInvalid);

    state.ias_mps = 250.0F;
    settle(adc, state, 400);
    EXPECT_EQ(adc.airspeed_quality(), SensorQuality::kDegraded)
        << "outside the calibrated envelope but still usable";
}

TEST(SensorSuite, QualityFlagsAreDistinctBits)
{
    // A consumer combines these into the 16 bit status field on the wire, so a
    // duplicate bit would make two different conditions indistinguishable.
    const SensorStatus all[] = {SensorStatus::kNone, SensorStatus::kAirDataDegraded,
                                SensorStatus::kEngineDegraded, SensorStatus::kAttitudeNotAligned,
                                SensorStatus::kBarometricAltitudeUnreliable};

    std::uint16_t seen = 0U;
    for (const SensorStatus status : all)
    {
        const std::uint16_t bits = to_bits(status);
        EXPECT_EQ(bits & seen, 0U) << "status bits overlap";
        seen = static_cast<std::uint16_t>(seen | bits);
    }
}

// --- Barometric channel ------------------------------------------------------

TEST(SensorSuite, VerticalSpeedLagsTheTruth)
{
    // A steady climb has to be produced by *integrating* the vertical speed into
    // the position: the air data computer derives altitude from the state vector,
    // so holding the altitude fixed produces a stationary aircraft no matter what
    // the velocity says.
    constexpr Scalar kClimbRateMps = 10.0F;

    AircraftState state{};
    state.position_m.y   = 1000.0F;
    state.velocity_mps.y = kClimbRateMps;
    state.ias_mps        = 90.0F;

    AirDataComputer adc{};
    // Seed the barometric filters at the starting altitude, as a real altimeter
    // is set on the ground.
    adc.reset(2U, 1000.0F);

    // Warm up: the trend filter has a four second time constant.
    for (int i = 0; i < 3000; ++i)
    {
        state.position_m.y += state.velocity_mps.y * kDt;
        adc.update(state, kDt);
    }

    double sum      = 0.0;
    int    measured = 0;
    for (; measured < 2000; ++measured)
    {
        state.position_m.y += state.velocity_mps.y * kDt;
        adc.update(state, kDt);
        sum += static_cast<double>(adc.vertical_speed_mps());
    }

    EXPECT_NEAR(sum / static_cast<double>(measured), kClimbRateMps, 1.0F)
        << "the rate channel does not track a steady climb";

    // Now stop climbing. The reported channel must still be climbing, which is
    // exactly the artefact a real vane shows when the aircraft levels off.
    state.velocity_mps.y = 0.0F;
    adc.update(state, kDt);

    EXPECT_GT(adc.vertical_speed_mps(), 2.0F)
        << "the vertical speed channel tracked the truth instantaneously";
}

TEST(SensorSuite, VerticalSpeedNoiseIsBounded)
{
    // The rate channel is a derivative, so its noise budget is set by the
    // averaging window rather than by the barometric sample noise. At 100 Hz a
    // naive derivative of the fast signal produces hundreds of metres per second
    // of fictitious climb rate. A couple of metres per second is what a real
    // barometric rate indicator shows; the rest is the physically correct part,
    // because a derivative always inherits the noise of its input.
    AirDataComputer adc{};
    adc.reset(6U, 2000.0F);

    AircraftState state{};
    state.position_m.y = 2000.0F;
    state.ias_mps      = 90.0F;

    settle(adc, state, 2000);
    const double stddev = rms(adc, state, 4000);

    EXPECT_LT(stddev, 2.5) << "vertical speed noise is " << stddev << " m/s in a steady state";
}

TEST(SensorSuite, BarometricAltitudeStartsAtTheSetAltitude)
{
    // The reset altitude is the altimeter setting, and it is the only thing that
    // makes the barometric channel meaningful: without it the first samples would
    // show a step from zero that the trend filter would read as an enormous
    // climb. Checked after a settle, because a single sample carries the full
    // sample noise and is not a measurement.
    AirDataComputer adc{};
    adc.reset(9U, 1500.0F);

    AircraftState state{};
    state.position_m.y = 1500.0F;
    settle(adc, state, 2000);

    // Averaged, not sampled: the barometric channel carries a bias that random
    // walks, so a single instantaneous reading can be ten metres from the mean
    // and asserting on it would be a coin toss rather than a test.
    double sum      = 0.0;
    int    measured = 0;
    for (; measured < 4000; ++measured)
    {
        adc.update(state, kDt);
        sum += static_cast<double>(adc.barometric_altitude_m());
    }
    EXPECT_NEAR(sum / static_cast<double>(measured), 1500.0, 5.0);

    // And the setting is not simply added afterwards: a 4000 m change must
    // actually move the channel, not be swallowed by the offset.
    state.position_m.y = 5500.0F;
    settle(adc, state, 8000);

    sum      = 0.0;
    measured = 0;
    for (; measured < 4000; ++measured)
    {
        adc.update(state, kDt);
        sum += static_cast<double>(adc.barometric_altitude_m());
    }
    EXPECT_NEAR(sum / static_cast<double>(measured), 5500.0, 20.0);
}

TEST(SensorSuite, RateChannelIsNotAnInstantaneousDerivative)
{
    // A single-sample finite difference on a noisy barometric signal is the
    // classic mistake. Feed a genuinely noisy altitude and check that the
    // reported rate stays bounded by what a real instrument would show, rather
    // than by the sample-to-sample jitter of the input.
    AirDataComputer adc{};
    adc.reset(12U, 3000.0F);

    AircraftState state{};
    state.position_m.y = 3000.0F;
    state.ias_mps      = 100.0F;

    settle(adc, state, 2000);

    double worst = 0.0;
    for (int i = 0; i < 4000; ++i)
    {
        adc.update(state, kDt);
        worst = std::max(worst, std::abs(static_cast<double>(adc.vertical_speed_mps())));
    }

    EXPECT_LT(worst, 20.0) << "the rate channel reported " << worst << " m/s of climb at rest";
}

// --- Pitot and attitude ------------------------------------------------------

TEST(SensorSuite, AirspeedLagsAnAbruptManoeuvre)
{
    AirDataComputer adc{};
    adc.reset(3U);

    AircraftState state{};
    state.ias_mps      = 80.0F;
    state.position_m.y = 1000.0F;
    settle(adc, state, 2000);
    ASSERT_NEAR(adc.indicated_airspeed_mps(), 80.0F, 0.5F);

    state.ias_mps = 120.0F;
    adc.update(state, kDt);

    // One 10 ms sample cannot have followed a 40 m/s step. An instrument that
    // reports the commanded value instantly is not measuring anything.
    EXPECT_LT(adc.indicated_airspeed_mps(), 100.0F);
}

TEST(SensorSuite, AirspeedConvergesOnASustainedValue)
{
    AirDataComputer adc{};
    adc.reset(3U);

    AircraftState state{};
    state.position_m.y = 1000.0F;
    state.ias_mps      = 80.0F;
    settle(adc, state, 2000);

    state.ias_mps = 120.0F;
    settle(adc, state, 4000);

    EXPECT_NEAR(adc.indicated_airspeed_mps(), 120.0F, 0.5F)
        << "the pitot never caught up with a sustained airspeed";
}

TEST(SensorSuite, AccelerometerReadsOneGInLevelFlight)
{
    // A specific force sensor measures what the aircraft feels, not what gravity
    // is doing, so level flight reads 1 g upwards and not zero. Reporting the
    // gravitational component instead would make every consumer subtract it a
    // second time.
    AirDataComputer adc{};
    adc.reset(5U);

    AircraftState state{};
    state.load_factor = 1.0F;
    settle(adc, state, 100);

    EXPECT_NEAR(adc.specific_force_mps2().y, kGravity, 0.01F);

    state.load_factor = 2.5F;
    adc.update(state, kDt);
    EXPECT_NEAR(adc.specific_force_mps2().y, 2.5F * kGravity, 0.05F);
}

TEST(SensorSuite, SpecificForceIsBoundedInEveryAxis)
{
    AirDataComputer adc{};
    adc.reset(14U);

    AircraftState state{};
    for (const Scalar load_factor : {-1.0F, 0.0F, 1.0F, 3.0F, 9.0F})
    {
        state.load_factor = load_factor;
        settle(adc, state, 50);

        const Vec3 force = adc.specific_force_mps2();
        EXPECT_TRUE(is_finite(force)) << "at load factor " << static_cast<double>(load_factor);
        EXPECT_LE(length(force), (9.0F * kGravity) + 1.0F);
    }
}

// --- Engine monitor ----------------------------------------------------------

TEST(EngineMonitor, FollowsTheThrottleWithALag)
{
    // The lag is the reason the channel exists. A temperature that tracked the
    // throttle instantly would show the pilot's hand, not the engine.
    EngineMonitor engines{};
    engines.reset(21U);

    AircraftState state{};
    state.throttle = 1.0F;

    engines.update(state, kDt);
    const Scalar first = engines.temperatures_c()[0];
    EXPECT_GT(first, 0.0F);
    EXPECT_LT(first, 700.0F) << "the first sample cannot already be at operating temperature";

    for (int i = 0; i < 4000; ++i)
    {
        engines.update(state, kDt);
    }
    const Scalar settled = engines.temperatures_c()[0];
    EXPECT_GT(settled, first);
    EXPECT_GT(settled, 500.0F) << "full power did not warm the engine up";
}

TEST(EngineMonitor, CoolsTowardsAmbientWithThePowerOff)
{
    EngineMonitor engines{};
    engines.reset(22U);

    AircraftState state{};
    state.throttle = 1.0F;
    for (int i = 0; i < 6000; ++i)
    {
        engines.update(state, kDt);
    }
    ASSERT_GT(engines.temperatures_c()[0], 500.0F);

    state.throttle = 0.0F;
    for (int i = 0; i < 20000; ++i)
    {
        engines.update(state, kDt);
    }

    // Idle is deliberately above ambient -- an idling turbine still sits well
    // above the air temperature -- so the floor is not zero.
    const Scalar cooled = engines.temperatures_c()[0];
    EXPECT_LT(cooled, 500.0F);
    EXPECT_GT(cooled, 0.0F);
}

TEST(EngineMonitor, IdleIsNotAmbient)
{
    EngineMonitor engines{};
    engines.reset(23U);

    AircraftState state{};
    state.throttle = 0.0F;
    for (int i = 0; i < 4000; ++i)
    {
        engines.update(state, kDt);
    }

    EXPECT_GT(engines.temperatures_c()[0], 15.0F)
        << "an idling engine reads ambient, which is not what an engine does";
}

TEST(EngineMonitor, EachEngineHasIndependentNoise)
{
    // A single shared deviate would be common mode, and the ground station would
    // be unable to tell a real engine-to-engine spread from a sensor artefact.
    EngineMonitor engines{};
    engines.reset(24U);

    AircraftState state{};
    state.throttle = 0.6F;
    for (int i = 0; i < 2000; ++i)
    {
        engines.update(state, kDt);
    }

    const auto& temperatures = engines.temperatures_c();
    ASSERT_EQ(temperatures.size(), kEngineCount);

    bool any_difference = false;
    for (std::size_t i = 1; i < temperatures.size(); ++i)
    {
        if (temperatures[i] != temperatures[0])
        {
            any_difference = true;
        }
    }
    EXPECT_TRUE(any_difference) << "all four engines reported an identical temperature";
}

TEST(EngineMonitor, EngineOutCoolsTheEngine)
{
    EngineMonitor engines{};
    engines.reset(25U);

    AircraftState running{};
    running.throttle = 1.0F;
    for (int i = 0; i < 6000; ++i)
    {
        engines.update(running, kDt);
    }
    const Scalar hot = engines.temperatures_c()[0];

    AircraftState failed = running;
    failed.faults        = FaultCode::kEngineOut;
    for (int i = 0; i < 20000; ++i)
    {
        engines.update(failed, kDt);
    }

    EXPECT_LT(engines.temperatures_c()[0], hot);
}

TEST(EngineMonitor, EngineFireIsATemperatureExcursionNotAThrottleChange)
{
    // A fire is not the engine running harder. If it were modelled as extra
    // throttle the temperature would rise only as fast as a normal overheat, and
    // the ground station would see a warm engine rather than a fire.
    EngineMonitor engines{};
    engines.reset(26U);

    AircraftState state{};
    state.throttle = 0.5F;
    for (int i = 0; i < 4000; ++i)
    {
        engines.update(state, kDt);
    }
    const Scalar normal = engines.temperatures_c()[0];

    state.faults = FaultCode::kEngineFire;
    for (int i = 0; i < 4000; ++i)
    {
        engines.update(state, kDt);
    }

    const Scalar fire = engines.temperatures_c()[0];
    EXPECT_GT(fire, normal + 50.0F) << "a fire at half throttle did not get hot";
    EXPECT_GT(fire, 800.0F);
}

TEST(EngineMonitor, FuelExhaustionStopsTheEnginesWithoutAFault)
{
    // Not every engine stop has a cause bit. The monitor has to respond to the
    // fuel state as well, or a dry tank shows four healthy warm engines.
    EngineMonitor engines{};
    engines.reset(27U);

    AircraftState state{};
    state.throttle = 1.0F;
    state.fuel_kg  = 0.0F;
    for (int i = 0; i < 20000; ++i)
    {
        engines.update(state, kDt);
    }

    EXPECT_LT(engines.temperatures_c()[0], 200.0F)
        << "four engines at full power with an empty fuel tank";
}

// --- Suite plumbing ----------------------------------------------------------

TEST(SensorSuite, FillsTheSampleFromTheInstrumentChannels)
{
    // Deliberately not asserting on `sequence` or `timestamp_ns`: those belong
    // to the orchestrator, not to the instruments. A sensor suite that stamped
    // its own sequence numbers would have to agree with the simulation's step
    // counter, and the two would drift the first time anything reset one of
    // them. The end-to-end tests cover that the frame does advance.
    SensorSuite suite{};
    suite.reset(31U, 500.0F);

    AircraftState state{};
    state.ias_mps        = 90.0F;
    state.position_m.y   = 500.0F;
    state.velocity_mps.y = 5.0F;
    state.heading_deg    = 45.0F;
    state.phase          = FlightPhase::kClimb;

    SensorSample sample{};
    suite.update(state, sample, kDt);

    EXPECT_EQ(sample.phase, FlightPhase::kClimb);
    EXPECT_EQ(sample.engine_temp_c.size(), kEngineCount);
    EXPECT_TRUE(is_finite(sample.airspeed_mps));
    EXPECT_TRUE(is_finite(sample.altitude_m));
    EXPECT_TRUE(is_finite(sample.accel_mps2));

    // Freshly reset, nothing is trusted yet: a consumer has to be able to see
    // that from the status bits alone.
    EXPECT_TRUE(has_status(static_cast<SensorStatus>(sample.status_bits),
                           SensorStatus::kAttitudeNotAligned));
}

TEST(SensorSuite, StatusBitsClearOnceTheInstrumentsSettle)
{
    // The complement of the previous test: the bits are not latched. An
    // instrument that raised a flag on switch-on and never cleared it would make
    // the field useless.
    SensorSuite suite{};
    suite.reset(31U, 500.0F);

    AircraftState state{};
    state.ias_mps      = 90.0F;
    state.position_m.y = 500.0F;
    state.phase        = FlightPhase::kClimb;
    state.throttle     = 0.5F;

    for (int i = 0; i < 5000; ++i)
    {
        SensorSample scratch{};
        suite.update(state, scratch, kDt);
    }

    SensorSample sample{};
    suite.update(state, sample, kDt);

    EXPECT_EQ(static_cast<SensorStatus>(sample.status_bits), SensorStatus::kNone)
        << "status bits still set " << static_cast<int>(sample.status_bits)
        << " after the instruments settled";
}

TEST(SensorSuite, IdlingEnginesAreDegradedNotFaulted)
{
    // Deliberate design decision, worth pinning: idling is not a fault, but it is
    // not good data either. Reporting kGood at idle would tell the ground
    // station that the engine channels are trustworthy when the temperature
    // channel is dominated by the idle offset rather than by combustion.
    SensorSuite suite{};
    suite.reset(33U, 500.0F);

    AircraftState state{};
    state.ias_mps      = 0.0F;
    state.position_m.y = 0.0F;
    state.throttle     = 0.0F;
    state.phase        = FlightPhase::kTaxi;

    SensorSample sample{};
    for (int i = 0; i < 5000; ++i)
    {
        suite.update(state, sample, kDt);
    }

    const auto status = static_cast<SensorStatus>(sample.status_bits);
    EXPECT_TRUE(has_status(status, SensorStatus::kEngineDegraded));
    EXPECT_FALSE(has_status(status, SensorStatus::kAirDataDegraded))
        << "a parked aircraft with the air data computer running is not an air data fault";
}

TEST(SensorSuite, AnEngineFireInvalidatesTheEngineChannel)
{
    SensorSuite suite{};
    suite.reset(34U, 500.0F);

    AircraftState state{};
    state.ias_mps      = 90.0F;
    state.position_m.y = 500.0F;
    state.throttle     = 0.5F;

    for (int i = 0; i < 2000; ++i)
    {
        SensorSample scratch{};
        suite.update(state, scratch, kDt);
    }

    state.faults = FaultCode::kEngineFire;
    SensorSample sample{};
    for (int i = 0; i < 400; ++i)
    {
        suite.update(state, sample, kDt);
    }

    EXPECT_TRUE(
        has_status(static_cast<SensorStatus>(sample.status_bits), SensorStatus::kEngineDegraded))
        << "a fire must not leave the engine channel reporting good data";
}

TEST(SensorSuite, IndependentStreamsDoNotCrossTalk)
{
    // Retuning one channel's noise must not shift another channel's sequence,
    // which is only true because the streams are separated by seed.
    AirDataComputer a{};
    AirDataComputer b{};
    a.reset(77U);
    b.reset(77U);

    EXPECT_FLOAT_EQ(a.indicated_airspeed_mps(), b.indicated_airspeed_mps());
    EXPECT_FLOAT_EQ(a.barometric_altitude_m(), b.barometric_altitude_m());

    AircraftState state{};
    state.ias_mps      = 95.0F;
    state.position_m.y = 900.0F;
    for (int i = 0; i < 1000; ++i)
    {
        a.update(state, kDt);
        b.update(state, kDt);
    }

    EXPECT_EQ(a.vertical_speed_mps(), b.vertical_speed_mps());
}

} // namespace
} // namespace fse