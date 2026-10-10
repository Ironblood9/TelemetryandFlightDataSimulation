// ---------------------------------------------------------------------------
// Standard atmosphere tests.
//
// The reference values come from the ISA tables, not from this implementation.
// That distinction is the entire value of the file: an atmospheric model that
// is "close enough" produces a flight profile that looks plausible in a plot
// and cannot be flown, and the only way to notice is to compare against
// something external.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>

#include "fse/flight_model.hpp"

namespace fse
{
namespace
{

TEST(Atmosphere, MatchesIsaAtSeaLevel)
{
    EXPECT_NEAR(Atmosphere::density(0.0F), 1.225F, 1.0e-4F);
    EXPECT_NEAR(Atmosphere::temperature(0.0F), 288.15F, 1.0e-2F);
    EXPECT_NEAR(Atmosphere::density_ratio(0.0F), 1.0F, 1.0e-6F);

    // Speed of sound is the one place the model is sensitive to the value of
    // the ratio of specific heats.
    EXPECT_NEAR(Atmosphere::speed_of_sound(0.0F), 340.294F, 0.05F);
}

TEST(Atmosphere, MatchesIsaReferenceTableAtFiveKilometres)
{
    // This single assertion is what pinned the density law exponent. The law is
    // rho = rho0 (1 - L h / T0)^k with k = g / (R L) - 1, and the -1 is easy to
    // forget: writing g / (R L) instead produced 0.6527 kg/m^3 here against a
    // true value of 0.73612, a thirteen percent error that no amount of
    // looking at a flight trace would have revealed.
    EXPECT_NEAR(Atmosphere::temperature(5000.0F), 255.65F, 1.0e-2F);
    EXPECT_NEAR(Atmosphere::density(5000.0F), 0.73612F, 5.0e-5F);
}

TEST(Atmosphere, LapseRateIsLinearInAltitude)
{
    // The troposphere model is a constant lapse rate, so the temperature drop is
    // exactly proportional to height.
    const Scalar drop_1000 = Atmosphere::temperature(0.0F) - Atmosphere::temperature(1000.0F);
    const Scalar drop_3000 = Atmosphere::temperature(0.0F) - Atmosphere::temperature(3000.0F);

    EXPECT_NEAR(drop_1000, 6.5F, 1.0e-2F);
    EXPECT_NEAR(drop_3000, 3.0F * drop_1000, 1.0e-2F);
}

TEST(Atmosphere, DensityFallsMonotonically)
{
    Scalar previous = Atmosphere::density(0.0F);
    for (Scalar altitude = 250.0F; altitude <= 15000.0F; altitude += 250.0F)
    {
        const Scalar density = Atmosphere::density(altitude);
        ASSERT_TRUE(std::isfinite(density)) << "at " << static_cast<double>(altitude) << " m";
        ASSERT_LE(density, previous)
            << "density increased at " << static_cast<double>(altitude) << " m";
        previous = density;
    }
}

TEST(Atmosphere, BelowSeaLevelIsClampedRatherThanExtrapolated)
{
    // The model clamps at sea level rather than extrapolating the lapse law
    // downwards. That is a deliberate choice: the alternative is a density that
    // keeps rising for an aircraft that is on the ground, and a negative altitude
    // that would then propagate into the altitude channel of the telemetry.
    EXPECT_NEAR(Atmosphere::density(-500.0F), Atmosphere::density(0.0F), 1.0e-6F);
    EXPECT_NEAR(Atmosphere::temperature(-500.0F), Atmosphere::temperature(0.0F), 1.0e-4F);
    EXPECT_TRUE(std::isfinite(Atmosphere::density(-500.0F)));
    EXPECT_GT(Atmosphere::density(-500.0F), 0.0F);
}

TEST(Atmosphere, StaysFiniteAtAbsurdAltitudes)
{
    // Beyond the tropopause the formula has no physical meaning, but it must not
    // become a NaN: the flight model clamps and clamps badly otherwise, and a NaN
    // propagates silently through every derived field of the state vector. The
    // temperature floor in atmosphere.cpp is what makes this hold -- without it,
    // speed_of_sound() was sqrt() of a negative number above about 44 km.
    for (const Scalar altitude : {11000.0F, 20000.0F, 50000.0F, 100000.0F})
    {
        EXPECT_TRUE(std::isfinite(Atmosphere::density(altitude)))
            << "density at " << static_cast<double>(altitude) << " m";
        EXPECT_TRUE(std::isfinite(Atmosphere::temperature(altitude)))
            << "temperature at " << static_cast<double>(altitude) << " m";
        EXPECT_TRUE(std::isfinite(Atmosphere::speed_of_sound(altitude)))
            << "speed of sound at " << static_cast<double>(altitude) << " m";
    }
}

TEST(Atmosphere, SpeedOfSoundFallsWithTemperature)
{
    EXPECT_LT(Atmosphere::speed_of_sound(5000.0F), Atmosphere::speed_of_sound(0.0F));
    EXPECT_NEAR(Atmosphere::speed_of_sound(5000.0F), 320.51F, 0.05F);
}

TEST(ForwardDirection, MatchesTheDocumentedAxisConvention)
{
    // The telemetry frame defines heading 0 as north (+Z) increasing towards
    // east (+X). If this ever changes, the heading channel silently becomes a
    // mirror image in the ground station.
    const Vec3 north = forward_direction(0.0F, 0.0F);
    EXPECT_NEAR(north.x, 0.0F, 1.0e-6F);
    EXPECT_NEAR(north.y, 0.0F, 1.0e-6F);
    EXPECT_NEAR(north.z, 1.0F, 1.0e-6F);

    const Vec3 east = forward_direction(90.0F, 0.0F);
    EXPECT_NEAR(east.x, 1.0F, 1.0e-6F);
    EXPECT_NEAR(east.z, 0.0F, 1.0e-6F);

    // Wrapping: 360 degrees is the same direction as zero.
    const Vec3 wrapped = forward_direction(360.0F, 0.0F);
    EXPECT_NEAR(wrapped.z, 1.0F, 1.0e-6F);
}

TEST(ForwardDirection, PositivePitchPointsUp)
{
    const Vec3 up = forward_direction(0.0F, 30.0F);
    EXPECT_NEAR(up.y, std::sin(deg_to_rad(30.0F)), 1.0e-5F);

    // And it stays a unit vector, which the force projections assume.
    EXPECT_NEAR(length(up), 1.0F, 1.0e-5F);
}

} // namespace
} // namespace fse