#include "fse/flight_phase.hpp"

#include <array>
#include <string>

namespace fse
{

namespace
{

struct PhaseName
{
    FlightPhase      phase;
    std::string_view name;
};

// Order matches the FlightPhase enumerator, which is itself part of the wire
// contract.
constexpr std::array<PhaseName, kFlightPhaseCount> kPhaseNames{{
    {FlightPhase::kPreflight, "preflight"},
    {FlightPhase::kTaxi, "taxi"},
    {FlightPhase::kTakeoffRoll, "takeoff_roll"},
    {FlightPhase::kClimb, "climb"},
    {FlightPhase::kCruise, "cruise"},
    {FlightPhase::kDescent, "descent"},
    {FlightPhase::kApproach, "approach"},
    {FlightPhase::kLanding, "landing"},
    {FlightPhase::kGoAround, "go_around"},
    {FlightPhase::kFailure, "failure"},
}};

constexpr char to_lower(char c) noexcept
{
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

} // namespace

std::string_view to_string(FlightPhase phase) noexcept
{
    for (const auto& entry : kPhaseNames)
    {
        if (entry.phase == phase)
        {
            return entry.name;
        }
    }
    return "unknown";
}

std::optional<FlightPhase> flight_phase_from_string(std::string_view name) noexcept
{
    std::string lowered;
    lowered.reserve(name.size());
    for (const char c : name)
    {
        lowered.push_back(to_lower(c));
    }

    for (const auto& entry : kPhaseNames)
    {
        if (entry.name == lowered)
        {
            return entry.phase;
        }
    }
    return std::nullopt;
}

std::string_view to_string(FaultCode flags) noexcept
{
    if (flags == FaultCode::kNone)
    {
        return "none";
    }

    // A bit set is a single string; several bits are reported as the first
    // match plus a count, because the ground station only renders one line.
    static constexpr std::array<FaultCode, 5> kAll{
        FaultCode::kEngineFire, FaultCode::kEngineOut, FaultCode::kSensorFault,
        FaultCode::kHydraulicLow, FaultCode::kDegradedAirframe};

    std::string_view first = "unknown";
    std::size_t      count = 0U;
    for (const FaultCode bit : kAll)
    {
        if (has_fault(flags, bit))
        {
            ++count;
            if (count == 1U)
            {
                switch (bit)
                {
                    case FaultCode::kEngineFire:
                        first = "engine_fire";
                        break;
                    case FaultCode::kEngineOut:
                        first = "engine_out";
                        break;
                    case FaultCode::kSensorFault:
                        first = "sensor_fault";
                        break;
                    case FaultCode::kHydraulicLow:
                        first = "hydraulic_low";
                        break;
                    case FaultCode::kDegradedAirframe:
                        first = "degraded_airframe";
                        break;
                    case FaultCode::kNone:
                        break;
                }
            }
        }
    }

    return (count == 1U) ? first : "multiple";
}

} // namespace fse