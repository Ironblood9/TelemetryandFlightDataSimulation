// ---------------------------------------------------------------------------
// Standard atmosphere.
//
// Split out from the aerodynamic model because it is a stateless lookup table
// with no dependency on the airframe: the guidance needs it to compute the
// angle of attack its wing currently needs, and so does the model. Keeping it
// separate also means it can be tested against the ISA reference table without
// standing up a simulation.
// ---------------------------------------------------------------------------

#include "fse/flight_model.hpp"

#include <cmath>

namespace fse
{

Scalar Atmosphere::temperature(Scalar altitude_m) noexcept
{
    // The lapse law is only valid inside the troposphere. Above it the formula
    // runs the temperature negative, and sqrt() of a negative number in
    // speed_of_sound() would then hand a NaN to every derived field of the state
    // vector. Clamping the temperature keeps the model *meaningless* above the
    // tropopause instead of making it actively poisonous, which is the property
    // that actually matters: nothing here is ever used at 40 km, but a NaN would
    // propagate silently the moment an altitude was wrong by a lot.
    constexpr Scalar kAbsoluteZeroK = 1.0F;
    return max(kSeaLevelTemperature - (kTemperatureLapseRate * max(altitude_m, 0.0F)),
               kAbsoluteZeroK);
}

Scalar Atmosphere::density(Scalar altitude_m) noexcept
{
    const Scalar altitude = max(altitude_m, 0.0F);
    const Scalar ratio    = 1.0F - ((kTemperatureLapseRate * altitude) / kSeaLevelTemperature);
    return kSeaLevelDensity * std::pow(max(ratio, 0.05F), kAirDensityExponent);
}

Scalar Atmosphere::density_ratio(Scalar altitude_m) noexcept
{
    return density(altitude_m) / kSeaLevelDensity;
}

Scalar Atmosphere::speed_of_sound(Scalar altitude_m) noexcept
{
    return std::sqrt(1.4F * kSpecificGasConstantAir * temperature(altitude_m));
}

Vec3 forward_direction(Scalar heading_deg, Scalar pitch_deg) noexcept
{
    const Scalar heading = deg_to_rad(wrap_degrees(heading_deg));
    const Scalar pitch   = deg_to_rad(pitch_deg);

    // Ground track direction: heading measured from north towards east.
    const Vec3 horizontal{std::sin(heading), 0.0F, std::cos(heading)};
    const Vec3 up{0.0F, 1.0F, 0.0F};

    return (horizontal * std::cos(pitch)) + (up * std::sin(pitch));
}

} // namespace fse