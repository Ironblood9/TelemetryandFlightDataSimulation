#include "fse/simulation.hpp"

#include <memory>

namespace fse
{

namespace
{

/// The rate is clamped to this range. 1 Hz is the slowest anything useful and
/// 1000 Hz is well beyond any real telemetry link, but a nonsensical rate must
/// be rejected rather than producing a zero timestep.
constexpr std::uint16_t kMinRateHz = 1U;
constexpr std::uint16_t kMaxRateHz = 1000U;

} // namespace

Simulation::Simulation(SimulationConfig config) : config_(config)
{
    if (config_.rate_hz < kMinRateHz)
    {
        config_.rate_hz = kMinRateHz;
    }
    if (config_.rate_hz > kMaxRateHz)
    {
        config_.rate_hz = kMaxRateHz;
    }

    dt_ = 1.0F / static_cast<Scalar>(config_.rate_hz);

    auto model = std::make_unique<AerodynamicModel>(config_.parameters);
    // The airframe carries no wind unless the configuration asks for one, so
    // that a bare model used in a test behaves like still air.
    if (config_.wind_enabled)
    {
        model->wind() = WindModel::light_tailwind();
    }
    model_ = std::move(model);

    reset();
}

void Simulation::reset()
{
    model_->reset(config_.seed);

    Scalar initial_altitude_m = 0.0F;

    // A scenario that starts airborne has to be placed in the air. Otherwise the
    // aircraft sits on the runway with the throttle at zero, because no phase
    // after take-off commands enough power to get it moving.
    if (is_airborne_phase(config_.initial_phase))
    {
        const MissionProfile& profile = config_.profile;

        Scalar ias_mps = profile.cruise_ias_mps;

        switch (config_.initial_phase)
        {
            case FlightPhase::kClimb:
                initial_altitude_m = profile.climb_altitude_m * 0.5F;
                ias_mps            = profile.climb_target_ias_mps;
                break;
            case FlightPhase::kCruise:
            case FlightPhase::kDescent:
            case FlightPhase::kFailure:
                initial_altitude_m = profile.cruise_altitude_m;
                ias_mps            = profile.cruise_ias_mps;
                break;
            case FlightPhase::kApproach:
                initial_altitude_m = profile.approach_altitude_m;
                ias_mps            = profile.approach_ias_mps;
                break;
            case FlightPhase::kLanding:
                initial_altitude_m = profile.flare_altitude_m + 30.0F;
                ias_mps            = profile.touchdown_ias_mps;
                break;
            case FlightPhase::kGoAround:
                initial_altitude_m = profile.go_around_altitude_m * 0.5F;
                ias_mps            = profile.climb_target_ias_mps;
                break;
            default:
                break;
        }

        model_->place_airborne(initial_altitude_m, ias_mps, profile.cruise_heading_deg);
    }

    // The barometric filters are seeded with the initial altitude so the first
    // emitted sample is already meaningful.
    sensors_.reset(config_.seed, initial_altitude_m);
    autopilot_.reset();
    phases_.reset(config_.initial_phase);
    phases_.set_climb_target_altitude_m(config_.profile.climb_altitude_m);

    step_index_ = 0U;
    sequence_   = 0U;
    phase_changes_.clear();
    sample_ = SensorSample{};

    engaged_phase_ = phases_.phase();
    autopilot_.engage(engaged_phase_, config_.profile);
    model_->set_phase(engaged_phase_);

    sensors_.update(model_->state(), sample_, dt_);
}

Scalar Simulation::dt() const noexcept
{
    return dt_;
}

Scalar Simulation::sim_time_s() const noexcept
{
    return static_cast<Scalar>(step_index_) * dt_;
}

void Simulation::engage_phase(FlightPhase phase)
{
    if (phase == engaged_phase_)
    {
        return;
    }

    const char* reason = phases_.transition_reason();
    phase_changes_.push_back(PhaseChange{engaged_phase_, phase, step_index_, sequence_,
                                         reason != nullptr ? reason : "unspecified"});

    engaged_phase_ = phase;
    autopilot_.engage(phase, config_.profile);
}

const SensorSample& Simulation::step()
{
    // 1. Phase first, evaluated against the state the previous step produced.
    phases_.update(dt_, model_->state(), config_.parameters, config_.profile);
    engage_phase(phases_.phase());

    // Publish the phase into the state vector so the sensor suite frames it.
    model_->set_phase(phases_.phase());

    // 2. Guidance, then physics.
    const ControlInputs commands =
        autopilot_.update(model_->state(), config_.profile, config_.parameters, dt_);
    model_->step(dt_, commands);

    // 3. Sensors over the freshly computed truth.
    sensors_.update(model_->state(), sample_, dt_);

    sample_.timestamp_ns = timestamp_ns_for(step_index_, config_.rate_hz);
    sample_.sequence     = sequence_;

    step_index_ += 1U;
    sequence_ += 1U;

    return sample_;
}

void Simulation::force_phase(FlightPhase phase)
{
    phases_.force(phase);
    engage_phase(phase);
    model_->set_phase(phase);
}

void Simulation::latch_fault(FaultCode fault)
{
    model_->latch_fault(fault);
}

} // namespace fse
