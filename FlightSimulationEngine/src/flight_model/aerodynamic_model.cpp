#include "fse/flight_model.hpp"

#include <cmath>

namespace fse
{

namespace
{

/// Attitude response time constants. These are the difference between a model
/// that looks like an aircraft and one that looks like a spreadsheet.
constexpr Scalar kPitchAttitudeTauS = 0.55F;
constexpr Scalar kRollAttitudeTauS  = 0.40F;

/// Flap actuator lag, seconds. The engines have their own lag in
/// `AircraftParameters::engine_spool_tau_s`.
constexpr Scalar kFlapActuatorTauS = 2.5F;

/// Ground steering authority, degrees per second. Deliberately slow: a nose
/// wheel is not a rudder.
constexpr Scalar kGroundTurnRateDps = 3.0F;

/// Below this airspeed the coordinated turn formula is not meaningful; it
/// divides by the true airspeed, and a light aircraft does not turn on the spot.
constexpr Scalar kMinTurningAirspeedMps = 15.0F;

/// Ground speed below which the rolling model is skipped entirely, because
/// there is no direction to scale. Thrust still acts, so the aircraft starts
/// moving as soon as it exceeds rolling resistance.
constexpr Scalar kRollingSpeedFloorMps = 1.0e-6F;

/// The gust process is generated with unit standard deviation and scaled to the
/// configured amplitude where it is used, so the altitude dependence stays
/// visible in one place.
constexpr Scalar kUnitGustSigma = 1.0F;

/// Stream selector for the gust process. Distinct from the sensor streams so a
/// reseed gives the same airframe motion even if sensor noise is retuned.
constexpr std::uint64_t kGustStream = 0x9e3779b97f4a7c15ULL;

/// Gust amplitude grows with altitude because the boundary layer thins out.
[[nodiscard]] Scalar gust_amplitude_for(Scalar altitude_m) noexcept
{
    return clamp(altitude_m / 3000.0F, 0.25F, 1.0F);
}

} // namespace

// ---------------------------------------------------------------------------
// Atmosphere and forward_direction
// ---------------------------------------------------------------------------
// Both live in src/atmosphere.cpp. They are a stateless lookup with no
// dependency on the airframe, and the guidance needs the same numbers to
// compute the angle of attack its wing currently requires. Keeping them out of
// this file also lets them be tested against the ISA reference table without
// standing up a simulation.

// ---------------------------------------------------------------------------
// AerodynamicModel
// ---------------------------------------------------------------------------

AerodynamicModel::AerodynamicModel()
    : AerodynamicModel(AircraftParameters::default_light_aircraft())
{
}

AerodynamicModel::AerodynamicModel(AircraftParameters parameters)
    : parameters_(parameters), gust_noise_(0U, kUnitGustSigma, wind_.gust_tau_s, kGustStream)
{
    reset(0U);
}

void AerodynamicModel::reset(std::uint64_t seed)
{
    seed_          = seed;
    state_         = AircraftState{};
    state_.fuel_kg = parameters_.mass_kg * 0.33F;

    throttle_actual_     = 0.0F;
    flap_actual_deg_     = 0.0F;
    gust_mps_            = 0.0F;
    heading_command_deg_ = 0.0F;

    gust_noise_.reseed(seed, kGustStream);

    update_derived(0.0F);
}

void AerodynamicModel::latch_fault(FaultCode fault) noexcept
{
    const std::uint16_t merged =
        static_cast<std::uint16_t>(to_bits(state_.faults) | to_bits(fault));
    state_.faults = static_cast<FaultCode>(merged);
}

void AerodynamicModel::set_phase(FlightPhase phase) noexcept
{
    state_.phase = phase;
}

void AerodynamicModel::place_airborne(Scalar altitude_m, Scalar ias_mps,
                                      Scalar heading_deg) noexcept
{
    state_.position_m     = Vec3{state_.position_m.x, altitude_m, state_.position_m.z};
    state_.heading_deg    = wrap_degrees(heading_deg);
    state_.pitch_deg      = 0.0F;
    state_.roll_deg       = 0.0F;
    state_.pitch_rate_dps = 0.0F;
    state_.roll_rate_dps  = 0.0F;
    state_.yaw_rate_dps   = 0.0F;

    // Level flight: the velocity is along the ground track at the requested
    // ground speed. The airspeed follows from the density in update_derived().
    state_.velocity_mps = fse::forward_direction(state_.heading_deg, 0.0F) * max(ias_mps, 1.0F);

    update_derived(0.0F);
}

bool AerodynamicModel::on_ground() const noexcept
{
    return state_.position_m.y <= kGroundContactAltitudeM;
}

bool AerodynamicModel::airborne() const noexcept
{
    return state_.position_m.y > kGroundContactAltitudeM;
}

Vec3 AerodynamicModel::air_velocity() const noexcept
{
    Vec3 wind = wind_.steady_mps;
    if (wind_.gust_sigma_mps > 0.0F)
    {
        wind += wind_.gust_axis * gust_mps_;
    }
    return state_.velocity_mps - wind;
}

Scalar AerodynamicModel::true_airspeed() const noexcept
{
    return length(air_velocity());
}

Vec3 AerodynamicModel::forward_direction() const noexcept
{
    return fse::forward_direction(state_.heading_deg, state_.pitch_deg);
}

void AerodynamicModel::step(Scalar dt, const ControlInputs& commands)
{
    if (dt <= 0.0F)
    {
        return;
    }

    update_propulsion(dt, commands);
    update_attitude(dt, commands);

    heading_command_deg_ = commands.heading_deg;
    integrate(dt);

    state_.step_index += 1U;
    update_derived(dt);
}

void AerodynamicModel::update_propulsion(Scalar dt, const ControlInputs& commands)
{
    // An engine fire or outage removes thrust authority rather than corrupting
    // it: the number stays in range, which is what a real failure indication
    // looks like on the ground station.
    Scalar throttle_target = clamp(commands.throttle, 0.0F, 1.0F);
    if (has_fault(state_.faults, FaultCode::kEngineFire))
    {
        throttle_target *= 0.35F;
    }
    if (has_fault(state_.faults, FaultCode::kEngineOut))
    {
        throttle_target = 0.0F;
    }

    state_.throttle_command = throttle_target;
    throttle_actual_ =
        approach(throttle_actual_, throttle_target, parameters_.engine_spool_tau_s, dt);

    flap_actual_deg_ =
        approach(flap_actual_deg_, clamp(commands.flap_deg, 0.0F, 30.0F), kFlapActuatorTauS, dt);

    state_.throttle = throttle_actual_;
    state_.flap_deg = flap_actual_deg_;
}

void AerodynamicModel::update_attitude(Scalar dt, const ControlInputs& commands)
{
    const Scalar pitch_target =
        clamp(commands.pitch_deg, -parameters_.max_pitch_deg, parameters_.max_pitch_deg);
    const Scalar roll_target =
        clamp(commands.roll_deg, -parameters_.max_roll_deg, parameters_.max_roll_deg);

    // First order lag towards the commanded attitude, then a rate limit. The lag
    // models inertia and control effectiveness; the rate limit models what the
    // airframe and the pilot can actually do.
    const Scalar pitch_demand = (pitch_target - state_.pitch_deg) / kPitchAttitudeTauS;
    state_.pitch_rate_dps =
        move_towards(state_.pitch_rate_dps, pitch_demand, parameters_.max_pitch_rate_dps, dt);
    state_.pitch_deg = clamp(state_.pitch_deg + (state_.pitch_rate_dps * dt), -90.0F, 90.0F);

    const Scalar roll_demand = (roll_target - state_.roll_deg) / kRollAttitudeTauS;
    state_.roll_rate_dps =
        move_towards(state_.roll_rate_dps, roll_demand, parameters_.max_roll_rate_dps, dt);
    state_.roll_deg = wrap_degrees_signed(state_.roll_deg + (state_.roll_rate_dps * dt));

    // Coordinated turn: the yaw rate a steady bank angle produces at this
    // airspeed. This is the standard g*tan(phi)/V relation, and it is why a
    // level turn increases the load factor.
    const Scalar tas = true_airspeed();
    if (tas >= kMinTurningAirspeedMps)
    {
        const Scalar bank_rad = deg_to_rad(state_.roll_deg);
        state_.yaw_rate_dps   = rad_to_deg(kGravity * std::tan(bank_rad) / tas);
    }
    else
    {
        state_.yaw_rate_dps = 0.0F;
    }

    if (on_ground())
    {
        // The gear keeps the wings level. It must NOT keep the nose on the
        // ground: rotation is the elevator rotating the aircraft about the main
        // gear, so pitch has to follow the command even before lift-off.
        state_.roll_deg      = approach(state_.roll_deg, 0.0F, 0.25F, dt);
        state_.roll_rate_dps = 0.0F;
    }

    state_.heading_deg = wrap_degrees(state_.heading_deg + (state_.yaw_rate_dps * dt));
}

void AerodynamicModel::integrate(Scalar dt)
{
    const Vec3   air      = air_velocity();
    const Scalar tas      = length(air);
    const Scalar altitude = max(state_.position_m.y, 0.0F);
    const Scalar rho      = Atmosphere::density(altitude);

    // --- Aerodynamic coefficients -----------------------------------------
    const Vec3 air_dir = (tas > 1.0e-3F) ? (air * (1.0F / tas)) : Vec3{0.0F, 1.0F, 0.0F};

    // Flight path angle and angle of attack. Zero sideslip is assumed, which is
    // consistent with the coordinated turn assumption in update_attitude().
    const Scalar flight_path_deg = rad_to_deg(std::asin(clamp(air_dir.y, -1.0F, 1.0F)));
    const Scalar alpha_deg       = state_.pitch_deg - flight_path_deg;
    state_.alpha_deg             = alpha_deg;

    // Flaps add lift and raise the stall limit, not only drag.
    const Scalar flap_lift = parameters_.cl_per_flap_deg * flap_actual_deg_;
    const Scalar cl_limit  = parameters_.cl_max + flap_lift;

    Scalar cl = parameters_.cl0 + flap_lift + (parameters_.cl_alpha_per_deg * alpha_deg);
    cl        = clamp(cl, parameters_.cl_min, cl_limit);

    Scalar cd = parameters_.cd0 + (parameters_.induced_drag_factor * cl * cl) +
                (parameters_.cd_per_flap_deg * flap_actual_deg_);

    // Post stall: the lift curve collapses and drag rises steeply. Without this
    // the model would let the aircraft pull through the stall, which no
    // telemetry stream would ever show.
    const Scalar stall_excess = max(alpha_deg - parameters_.stall_alpha_deg, 0.0F);
    if (stall_excess > 0.0F)
    {
        cd += parameters_.post_stall_cd_per_deg * stall_excess;
        cl /= 1.0F + (0.08F * stall_excess);
    }

    const Scalar dynamic_pressure = 0.5F * rho * tas * tas;
    const Scalar lift             = dynamic_pressure * parameters_.wing_area_m2 * cl;
    const Scalar drag             = dynamic_pressure * parameters_.wing_area_m2 * cd;

    // --- Propulsion ---------------------------------------------------------
    Scalar throttle = throttle_actual_;

    // Fuel burn. The simulation has a finite tank on purpose: a mission that
    // quietly never ends is not a test case.
    if (state_.fuel_kg > 0.0F)
    {
        state_.fuel_kg = max(0.0F, state_.fuel_kg - (parameters_.fuel_flow_kg_per_s * dt));
    }
    if (state_.fuel_kg <= 0.0F)
    {
        throttle         = 0.0F;
        throttle_actual_ = 0.0F;
        state_.throttle  = 0.0F;
    }

    const Scalar thrust = throttle * parameters_.max_thrust_per_engine_n *
                          static_cast<Scalar>(kEngineCount) * Atmosphere::density_ratio(altitude);

    // --- Force directions ---------------------------------------------------
    // Lift is perpendicular to the relative wind, in the aircraft's plane of
    // symmetry: project world up onto the plane normal to the air velocity.
    const Vec3 lift_dir = normalized(Vec3{0.0F, 1.0F, 0.0F} - (air_dir * air_dir.y));
    const Vec3 drag_dir = -air_dir;

    const Vec3 force = (lift_dir * lift) + (drag_dir * drag) + (forward_direction() * thrust) +
                       Vec3{0.0F, -parameters_.mass_kg * kGravity, 0.0F};

    // Semi implicit Euler: update the velocity first and use the new velocity
    // for the position update. First order accurate and, unlike explicit Euler,
    // it does not pump energy into the orbit.
    state_.velocity_mps += (force * (1.0F / parameters_.mass_kg)) * dt;
    state_.position_m += state_.velocity_mps * dt;

    state_.load_factor = lift / (parameters_.mass_kg * kGravity);

    apply_ground_constraint(dt);
}

void AerodynamicModel::apply_ground_constraint(Scalar dt)
{
    if (state_.position_m.y > kGroundContactAltitudeM)
    {
        return;
    }

    state_.position_m.y = kGroundContactAltitudeM;
    if (state_.velocity_mps.y < 0.0F)
    {
        state_.velocity_mps.y = 0.0F;
    }

    // The wheels are on the ground: roll, and steer.
    //
    // This is a *constraint*, not a separate model. Applying it after the force
    // integration means lift is always evaluated, so the transition to flight is
    // simply the moment the net vertical force turns positive. An earlier design
    // chose between a ground handler and a flight integrator instead, and the
    // take-off chattered between the two at the rotation speed, where lift sits
    // within a few percent of the weight: exactly the regime a discrete branch
    // cannot represent.
    //
    // Only friction is applied here. Thrust was already integrated as a force, so
    // adding a longitudinal throttle term as well would double count it.
    Vec3         horizontal{state_.velocity_mps.x, 0.0F, state_.velocity_mps.z};
    const Scalar ground_speed = length(horizontal);

    if (ground_speed > kRollingSpeedFloorMps)
    {
        // Rolling resistance is modelled purely as a deceleration and is allowed
        // to bring the aircraft to a stop, but never to reverse it. An earlier
        // version added a "static friction" branch that zeroed any speed below a
        // threshold, which silently pinned the aircraft to the apron: the speed
        // gained from thrust in one step is smaller than the threshold, so it was
        // discarded before it could ever accumulate.
        const Scalar deceleration = parameters_.rolling_resistance * kGravity;
        const Scalar rolled_speed = max(0.0F, ground_speed - (deceleration * dt));
        horizontal                = horizontal * (rolled_speed / ground_speed);
    }

    state_.velocity_mps = horizontal;

    // The nose wheel steers towards the commanded heading, slowly, but only
    // while there is a real airspeed to steer with.
    if (ground_speed > 1.0F)
    {
        const Scalar turn   = clamp(angle_difference_deg(heading_command_deg_, state_.heading_deg),
                                    -kGroundTurnRateDps, kGroundTurnRateDps);
        state_.heading_deg  = wrap_degrees(state_.heading_deg + (turn * dt));
        state_.yaw_rate_dps = turn;
    }
    else
    {
        state_.yaw_rate_dps = 0.0F;
    }

    // The accelerometer reads 1 g with the aircraft sitting level on its gear:
    // the wheels carry whatever the wing does not. Leaving the aerodynamic
    // load factor here would report 0 g on the ground, which no real instrument
    // does and which a consumer of the telemetry would have to special case.
    state_.load_factor = 1.0F;
}

void AerodynamicModel::update_derived(Scalar dt)
{
    const Scalar tas = true_airspeed();
    state_.tas_mps   = tas;

    // Ground speed is what the wheels experience. In still air it equals TAS;
    // for a parked aircraft in a 6 m/s wind it is zero while TAS is 6, which is
    // exactly the distinction a taxi decision needs.
    state_.ground_speed_mps = length(state_.velocity_mps);

    // Indicated airspeed compresses with density the way a pitot tube does.
    const Scalar ratio = Atmosphere::density_ratio(state_.position_m.y);
    state_.ias_mps     = tas * std::sqrt(max(ratio, 0.05F));

    // Specific total energy height: the height the aircraft could reach if it
    // converted all of its kinetic and potential energy into altitude.
    state_.energy_height_m = state_.position_m.y + (tas * tas / (2.0F * kGravity));

    // Advance the gust process. It is the only stochastic input to the
    // airframe, and it is seeded, so the whole run stays reproducible.
    if (dt > 0.0F)
    {
        gust_mps_ =
            gust_noise_.next(dt) * (wind_.gust_sigma_mps * gust_amplitude_for(state_.position_m.y));
    }
}

} // namespace fse
