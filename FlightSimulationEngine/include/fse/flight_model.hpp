#pragma once

/// @file flight_model.hpp
/// @brief The flight dynamics abstraction and its atmospheric helpers.

#include <cstdint>

#include "fse/aircraft.hpp"
#include "fse/noise.hpp"
#include "fse/types.hpp"

namespace fse
{

/// Standard atmosphere properties at a geometric altitude.
///
/// Uses the ISA troposphere lapse law. Valid below the tropopause
/// (11 000 m); the aircraft model never leaves the troposphere.
struct Atmosphere
{
    /// Air density, kg/m^3.
    [[nodiscard]] static Scalar density(Scalar altitude_m) noexcept;
    /// Static temperature, K.
    [[nodiscard]] static Scalar temperature(Scalar altitude_m) noexcept;
    /// Speed of sound, m/s.
    [[nodiscard]] static Scalar speed_of_sound(Scalar altitude_m) noexcept;
    /// Ratio of the local density to the sea level density, used for the
    /// thrust lapse.
    [[nodiscard]] static Scalar density_ratio(Scalar altitude_m) noexcept;
};

/// Steady wind plus gust, expressed in the world frame.
///
/// Deliberately zero by default. A wind model that is on unless it is switched
/// off leaks into every test that constructs a bare model, and "the aircraft is
/// parked but reads 6 m/s" is exactly the sort of surprise that costs an
/// afternoon. `Simulation` installs a wind explicitly when the configuration
/// asks for one.
struct WindModel
{
    /// Mean wind velocity, m/s. X east, Y up, Z north.
    Vec3 steady_mps{};
    /// Standard deviation of the gust component, m/s.
    Scalar gust_sigma_mps{0.0F};
    /// Gust correlation time, seconds.
    Scalar gust_tau_s{4.0F};
    /// Direction the gust acts along, unit vector. Gusts are one dimensional:
    /// modelling a full vector field would add cost without changing any
    /// conclusion in the telemetry.
    Vec3 gust_axis{0.0F, 0.0F, 1.0F};

    /// A representative light wind: 6 m/s from the north plus 2.5 m/s gusts.
    /// Used by `SimulationConfig::wind_enabled`.
    [[nodiscard]] static constexpr WindModel light_tailwind() noexcept
    {
        WindModel model;
        model.steady_mps     = Vec3{0.0F, 0.0F, -6.0F};
        model.gust_sigma_mps = 2.5F;
        return model;
    }
};

/// Commanded control surface and propulsion positions.
///
/// These are *commands*, not achieved positions: the model applies actuator lag
/// and rate limits internally, which is what makes the response realistic and
/// what makes the phase transitions take time.
struct ControlInputs
{
    /// Commanded pitch attitude, degrees.
    Scalar pitch_deg{0.0F};
    /// Commanded bank angle, degrees. Positive is right wing down.
    Scalar roll_deg{0.0F};
    /// Commanded heading, degrees `[0, 360)`. Used by the ground turn and by
    /// the autopilot's turn coordinator.
    Scalar heading_deg{0.0F};
    /// Commanded throttle in `[0, 1]`.
    Scalar throttle{0.0F};
    /// Commanded flap deflection, degrees.
    Scalar flap_deg{0.0F};
};

/// Interface every flight dynamics implementation satisfies.
///
/// The interface exists so the phase machine, the autopilot and the sensor
/// suite never depend on a concrete aerodynamics implementation. That is what
/// makes it possible to swap in a higher fidelity model — or a recorded
/// trajectory — without touching the layers above.
class IFlightModel
{
public:
    IFlightModel()                               = default;
    IFlightModel(const IFlightModel&)            = default;
    IFlightModel(IFlightModel&&)                 = default;
    IFlightModel& operator=(const IFlightModel&) = default;
    IFlightModel& operator=(IFlightModel&&)      = default;
    virtual ~IFlightModel()                      = default;

    /// Restores the initial condition and makes the run reproducible for
    /// @p seed.
    virtual void reset(std::uint64_t seed) = 0;

    /// Advances the dynamics by exactly @p dt seconds.
    virtual void step(Scalar dt, const ControlInputs& commands) = 0;

    [[nodiscard]] virtual const AircraftState& state() const noexcept = 0;

    [[nodiscard]] virtual AircraftParameters&       parameters() noexcept       = 0;
    [[nodiscard]] virtual const AircraftParameters& parameters() const noexcept = 0;

    /// Latches a fault. The phase machine notices it on the next step.
    virtual void latch_fault(FaultCode fault) noexcept = 0;

    /// Publishes the current flight phase into the state vector.
    ///
    /// The phase machine owns the phase, but the phase is also part of the
    /// vehicle's situation and the sensor suite copies it into every frame. This
    /// setter is how the simulation keeps the two consistent without either side
    /// having to query the other.
    virtual void set_phase(FlightPhase phase) noexcept = 0;

    /// Places the aircraft in trimmed level flight at the given condition.
    ///
    /// Scenario setup, not physics: a run that starts in cruise has to begin in
    /// the air, because no airborne phase commands a take-off throttle and a
    /// "cruise" scenario placed on the runway would sit there for ever.
    virtual void place_airborne(Scalar altitude_m, Scalar ias_mps, Scalar heading_deg) noexcept = 0;
};

/// Point-mass flight model with a linear lift curve, a parabolic drag polar and
/// a coordinated turn.
///
/// Scope, stated plainly: this is a three degree of freedom translational model
/// with kinematic rotation. It reproduces the forces, the energy budget and the
/// response times that matter for telemetry, and it is deliberately *not* a
/// six degree of freedom aerodynamic solution — there are no moments of
/// inertia, no damping derivatives and no asymmetric aerodynamics. Modelling
/// attitude as a commanded quantity with actuator lag is the right trade here:
/// a telemetry pipeline cannot tell the difference, and the code stays
/// auditable.
class AerodynamicModel final : public IFlightModel
{
public:
    AerodynamicModel();
    explicit AerodynamicModel(AircraftParameters parameters);

    void reset(std::uint64_t seed) override;
    void step(Scalar dt, const ControlInputs& commands) override;

    [[nodiscard]] const AircraftState& state() const noexcept override
    {
        return state_;
    }
    [[nodiscard]] AircraftParameters& parameters() noexcept override
    {
        return parameters_;
    }
    [[nodiscard]] const AircraftParameters& parameters() const noexcept override
    {
        return parameters_;
    }
    void latch_fault(FaultCode fault) noexcept override;
    void set_phase(FlightPhase phase) noexcept override;
    void place_airborne(Scalar altitude_m, Scalar ias_mps, Scalar heading_deg) noexcept override;

    [[nodiscard]] WindModel& wind() noexcept
    {
        return wind_;
    }
    [[nodiscard]] const WindModel& wind() const noexcept
    {
        return wind_;
    }

    /// Ground clearance threshold. The contact model is a hard constraint at
    /// zero altitude rather than a spring, because a spring would need a
    /// stiffness and damping pair chosen arbitrarily.
    static constexpr Scalar kGroundContactAltitudeM = 0.0F;

    /// The landing gear is in contact with the runway.
    [[nodiscard]] bool on_ground() const noexcept;

    /// Strictly above the surface. The difference from `on_ground()` matters
    /// during rotation, when the aircraft is still at zero altitude but is
    /// already flying.
    [[nodiscard]] bool airborne() const noexcept;

private:
    void update_attitude(Scalar dt, const ControlInputs& commands);
    void update_propulsion(Scalar dt, const ControlInputs& commands);

    /// Single force integrator, shared by the runway and the air. See the
    /// implementation for why there is deliberately no separate ground model.
    void integrate(Scalar dt);

    /// Non-penetration and rolling friction, applied after the integration.
    void apply_ground_constraint(Scalar dt);

    /// Refreshes the derived read-outs (airspeeds, energy height, gust).
    void update_derived(Scalar dt);

    [[nodiscard]] Vec3   air_velocity() const noexcept;
    [[nodiscard]] Scalar true_airspeed() const noexcept;
    [[nodiscard]] Vec3   forward_direction() const noexcept;

    AircraftParameters parameters_;
    AircraftState      state_;
    WindModel          wind_;

    /// Seeded gust process. Generated with unit standard deviation and scaled
    /// by `WindModel::gust_sigma_mps` at the point of use.
    random::ColouredNoise gust_noise_;

    /// Realised throttle, after the spool lag. Never negative.
    Scalar throttle_actual_{0.0F};
    Scalar flap_actual_deg_{0.0F};

    /// Cached gust value, advanced once per step.
    Scalar        gust_mps_{0.0F};
    std::uint64_t seed_{0};
    /// Heading command, remembered for the ground steering constraint.
    Scalar heading_command_deg_{0.0F};
};

/// Unit vector pointing along the ground track for a heading and pitch.
///
/// Heading 0 is north (+Z) and increases towards east (+X), matching the
/// telemetry frame definition in docs/protocol-spec.md.
[[nodiscard]] Vec3 forward_direction(Scalar heading_deg, Scalar pitch_deg) noexcept;

} // namespace fse