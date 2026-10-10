// ---------------------------------------------------------------------------
// Enumeration tests: the flight phase list and the fault bit flags.
//
// The numeric values of FlightPhase are transmitted verbatim in the telemetry
// frame, so they are a wire contract rather than an implementation detail. The
// test below is the one place that says so.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <string>

#include "fse/flight_phase.hpp"

namespace fse
{
namespace
{

TEST(FlightPhaseNames, EnumerationMatchesTheWireContract)
{
    // docs/protocol-spec.md, section 3.1. Reordering these silently breaks
    // every ground station built against protocol version 1, and nothing in the
    // build would notice.
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kPreflight), 0U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kTaxi), 1U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kTakeoffRoll), 2U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kClimb), 3U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kCruise), 4U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kDescent), 5U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kApproach), 6U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kLanding), 7U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kGoAround), 8U);
    EXPECT_EQ(static_cast<std::uint8_t>(FlightPhase::kFailure), 9U);
}

TEST(FlightPhaseNames, EveryPhaseRoundTripsThroughItsName)
{
    for (std::uint8_t i = 0; i < kFlightPhaseCount; ++i)
    {
        const auto phase = static_cast<FlightPhase>(i);
        ASSERT_TRUE(is_valid(phase)) << "index " << static_cast<int>(i);

        const auto parsed = flight_phase_from_string(to_string(phase));
        ASSERT_TRUE(parsed.has_value()) << to_string(phase);
        EXPECT_EQ(*parsed, phase);
    }
}

TEST(FlightPhaseNames, ParsingIsCaseInsensitive)
{
    // The control channel accepts names from an operator, so the parse has to
    // tolerate the casing they actually type.
    EXPECT_EQ(flight_phase_from_string("CRUISE"), FlightPhase::kCruise);
    EXPECT_EQ(flight_phase_from_string("Takeoff_Roll"), FlightPhase::kTakeoffRoll);
    EXPECT_EQ(flight_phase_from_string("go_around"), FlightPhase::kGoAround);
}

TEST(FlightPhaseNames, ParsingRejectsUnknownNames)
{
    // Silently defaulting to preflight would turn a typo in a control command
    // into a parked aircraft instead of a rejected command.
    EXPECT_FALSE(flight_phase_from_string("").has_value());
    EXPECT_FALSE(flight_phase_from_string("orbit").has_value());
    EXPECT_FALSE(flight_phase_from_string("cruise altitude").has_value());
}

TEST(FlightPhaseNames, OutOfRangeValuesAreDetectedAndNamed)
{
    // A receiver must be able to survive a frame from a newer sender.
    EXPECT_FALSE(is_valid(static_cast<FlightPhase>(10)));
    EXPECT_FALSE(is_valid(static_cast<FlightPhase>(255)));
    EXPECT_EQ(to_string(static_cast<FlightPhase>(200)), "unknown");
}

TEST(FaultCodes, CombineAndDetect)
{
    EXPECT_FALSE(has_fault(FaultCode::kNone, FaultCode::kEngineFire));
    EXPECT_TRUE(has_fault(FaultCode::kEngineFire, FaultCode::kEngineFire));

    const auto combined =
        static_cast<FaultCode>(to_bits(FaultCode::kEngineFire) | to_bits(FaultCode::kSensorFault));
    EXPECT_TRUE(has_fault(combined, FaultCode::kEngineFire));
    EXPECT_TRUE(has_fault(combined, FaultCode::kSensorFault));
    EXPECT_FALSE(has_fault(combined, FaultCode::kHydraulicLow));
}

TEST(FaultCodes, NamesAreReported)
{
    EXPECT_EQ(to_string(FaultCode::kNone), "none");
    EXPECT_EQ(to_string(FaultCode::kEngineFire), "engine_fire");
    EXPECT_EQ(to_string(FaultCode::kEngineOut), "engine_out");

    // Several simultaneous faults are summarised rather than listing them all:
    // the ground station renders one line, and the individual bits are still
    // available on the wire.
    const auto multiple =
        static_cast<FaultCode>(to_bits(FaultCode::kEngineFire) | to_bits(FaultCode::kHydraulicLow));
    EXPECT_EQ(to_string(multiple), "multiple");
}

TEST(FaultCodes, BitsAreDistinct)
{
    // A duplicate bit would make two faults indistinguishable on the wire.
    const FaultCode all[] = {FaultCode::kEngineFire, FaultCode::kEngineOut, FaultCode::kSensorFault,
                             FaultCode::kHydraulicLow, FaultCode::kDegradedAirframe};

    std::uint16_t seen = 0U;
    for (const FaultCode fault : all)
    {
        const std::uint16_t bits = to_bits(fault);
        EXPECT_EQ(bits & seen, 0U) << "fault bits overlap";
        seen = static_cast<std::uint16_t>(seen | bits);
    }
    EXPECT_EQ(seen, 0x001FU);
}

} // namespace
} // namespace fse