#pragma once

/// @file flight_phase.hpp
/// @brief Flight phase enumeration and the data driven transition table.
///
/// The phase machine is deliberately *not* a `switch` inside the update loop.
/// Every legal transition lives in one table together with the guard that
/// permits it, which means the set of legal behaviour can be read, reviewed and
/// unit tested as data — and an illegal transition is rejected by construction
/// rather than by convention.

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace fse
{

/// Lifecycle of a flight. The numeric values are part of the wire contract:
/// they are transmitted verbatim in the telemetry frame, so they must not be
/// reordered. See docs/protocol-spec.md section 3.1.
enum class FlightPhase : std::uint8_t
{
    kPreflight   = 0,
    kTaxi        = 1,
    kTakeoffRoll = 2,
    kClimb       = 3,
    kCruise      = 4,
    kDescent     = 5,
    kApproach    = 6,
    kLanding     = 7,
    kGoAround    = 8,
    kFailure     = 9,
};

inline constexpr std::size_t kFlightPhaseCount = 10U;

[[nodiscard]] std::string_view to_string(FlightPhase phase) noexcept;

[[nodiscard]] constexpr bool is_valid(FlightPhase phase) noexcept
{
    return static_cast<std::uint8_t>(phase) < kFlightPhaseCount;
}

/// Parses a phase name, case insensitively. Returns `std::nullopt` if unknown.
[[nodiscard]] std::optional<FlightPhase> flight_phase_from_string(std::string_view name) noexcept;

/// Fault causes that latch the `kFailure` phase. Values are bit flags so the
/// telemetry frame can report several simultaneously.
enum class FaultCode : std::uint16_t
{
    kNone             = 0U,
    kEngineFire       = 1U << 0U,
    kEngineOut        = 1U << 1U,
    kSensorFault      = 1U << 2U,
    kHydraulicLow     = 1U << 3U,
    kDegradedAirframe = 1U << 4U,
};

[[nodiscard]] constexpr bool has_fault(FaultCode flags, FaultCode bit) noexcept
{
    return (static_cast<std::uint16_t>(flags) & static_cast<std::uint16_t>(bit)) != 0U;
}

[[nodiscard]] constexpr std::uint16_t to_bits(FaultCode flags) noexcept
{
    return static_cast<std::uint16_t>(flags);
}

[[nodiscard]] std::string_view to_string(FaultCode flags) noexcept;

} // namespace fse