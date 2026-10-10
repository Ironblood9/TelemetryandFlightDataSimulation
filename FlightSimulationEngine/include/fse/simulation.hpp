#pragma once

/// @file simulation.hpp
/// @brief The 100 Hz simulation loop that ties the layers together.
///
/// Order of operations per step is the contract:
///
///   1. advance the phase machine on the *current* state,
///   2. let the guidance engage the new phase if it changed,
///   3. compute control commands from the new phase,
///   4. step the flight model,
///   5. run the sensors over the new truth and emit a `SensorSample`.
///
/// The phase therefore always leads the physics by one step, and the telemetry
/// always describes the state that was just computed rather than the one the
/// commands were derived from. Getting this order wrong is the classic source of
/// a phase transition that appears to happen after the manoeuvre it describes.

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "fse/aircraft.hpp"
#include "fse/autopilot.hpp"
#include "fse/flight_model.hpp"
#include "fse/flight_phase.hpp"
#include "fse/phase_machine.hpp"
#include "fse/sensor.hpp"
#include "fse/types.hpp"

namespace fse
{

struct SimulationConfig
{
    /// Telemetry production rate, Hz. The default matches the protocol's
    /// nominal 100 Hz and the 10 ms real-time deadline.
    std::uint16_t      rate_hz{100};
    std::uint64_t      seed{0x243f6a8885a308d3ULL};
    AircraftParameters parameters{};
    MissionProfile     profile{};
    FlightPhase        initial_phase{FlightPhase::kPreflight};
    bool               wind_enabled{true};
};

/// A phase change, recorded so a test or a CLI run can assert the reason.
struct PhaseChange
{
    FlightPhase   from;
    FlightPhase   to;
    std::uint64_t step_index;
    std::uint32_t sequence;
    const char*   reason;
};

/// Owns every layer and produces one telemetry sample per step.
class Simulation
{
public:
    explicit Simulation(SimulationConfig config = {});

    /// Restores the initial condition. Same seed, same run, always.
    void reset();

    /// Advances exactly one sample period and returns the sample produced.
    ///
    /// The timestep is derived from `rate_hz` by integer arithmetic, so the
    /// simulation time never accumulates floating point drift: step 3 600 000 is
    /// at exactly one hour regardless of how the run was chopped up.
    [[nodiscard]] const SensorSample& step();

    [[nodiscard]] Scalar        dt() const noexcept;
    [[nodiscard]] Scalar        sim_time_s() const noexcept;
    [[nodiscard]] std::uint64_t step_index() const noexcept
    {
        return step_index_;
    }
    [[nodiscard]] std::uint16_t rate_hz() const noexcept
    {
        return config_.rate_hz;
    }

    [[nodiscard]] const AircraftState& aircraft_state() const noexcept
    {
        return model_->state();
    }
    [[nodiscard]] const PhaseMachine& phase_machine() const noexcept
    {
        return phases_;
    }
    [[nodiscard]] FlightPhase phase() const noexcept
    {
        return phases_.phase();
    }
    [[nodiscard]] const Autopilot& autopilot() const noexcept
    {
        return autopilot_;
    }

    [[nodiscard]] const std::vector<PhaseChange>& phase_changes() const noexcept
    {
        return phase_changes_;
    }

    /// Operator override from the control channel. Records the transition so it
    /// shows up in the phase trace like any other change.
    void force_phase(FlightPhase phase);

    void latch_fault(FaultCode fault);

    [[nodiscard]] const SimulationConfig& config() const noexcept
    {
        return config_;
    }
    [[nodiscard]] SensorSuite& sensors() noexcept
    {
        return sensors_;
    }

private:
    void engage_phase(FlightPhase phase);

    SimulationConfig config_;

    // Owned through a pointer to the interface: the layers above depend on the
    // abstraction, and a higher fidelity model can be substituted without
    // touching them.
    std::unique_ptr<IFlightModel> model_;
    Autopilot                     autopilot_;
    PhaseMachine                  phases_;
    SensorSuite                   sensors_;

    Scalar                   dt_{0.01F};
    std::uint64_t            step_index_{0};
    std::uint32_t            sequence_{0};
    FlightPhase              engaged_phase_{FlightPhase::kPreflight};
    SensorSample             sample_{};
    std::vector<PhaseChange> phase_changes_;
};

/// Nanosecond timestamp for a step index at a given rate.
///
/// Exact integer arithmetic: no accumulated error, and identical results on
/// every platform. A rate that does not divide a second truncates, which is
/// documented rather than rounded because rounding would accumulate.
[[nodiscard]] constexpr std::uint64_t timestamp_ns_for(std::uint64_t step_index,
                                                       std::uint16_t rate_hz) noexcept
{
    return (step_index * 1000000000ULL) / static_cast<std::uint64_t>(rate_hz);
}

} // namespace fse