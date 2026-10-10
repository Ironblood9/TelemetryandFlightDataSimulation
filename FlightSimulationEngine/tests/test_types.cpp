// ---------------------------------------------------------------------------
// Core scalar and vector tests.
//
// These cover the helpers every other module is built on, which means a silent
// error here would surface as an inexplicable physics bug four layers up.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <type_traits>

#include "fse/types.hpp"

namespace fse
{
namespace
{

TEST(Vec3, DefaultsToZero)
{
    constexpr Vec3 v{};
    EXPECT_FLOAT_EQ(v.x, 0.0F);
    EXPECT_FLOAT_EQ(v.y, 0.0F);
    EXPECT_FLOAT_EQ(v.z, 0.0F);
}

TEST(Vec3, ArithmeticIsComponentWise)
{
    constexpr Vec3 a{1.0F, 2.0F, 3.0F};
    constexpr Vec3 b{4.0F, 5.0F, 6.0F};

    constexpr Vec3 sum = a + b;
    EXPECT_FLOAT_EQ(sum.x, 5.0F);
    EXPECT_FLOAT_EQ(sum.y, 7.0F);
    EXPECT_FLOAT_EQ(sum.z, 9.0F);

    constexpr Vec3 difference = b - a;
    EXPECT_FLOAT_EQ(difference.x, 3.0F);
    EXPECT_FLOAT_EQ(difference.y, 3.0F);
    EXPECT_FLOAT_EQ(difference.z, 3.0F);

    constexpr Vec3 scaled = a * 2.0F;
    EXPECT_FLOAT_EQ(scaled.x, 2.0F);
    EXPECT_FLOAT_EQ(scaled.y, 4.0F);
    EXPECT_FLOAT_EQ(scaled.z, 6.0F);

    constexpr Vec3 negated = -a;
    EXPECT_FLOAT_EQ(negated.x, -1.0F);
}

TEST(Vec3, DotAndCrossFollowTheRightHandedConvention)
{
    // World frame is X east, Y up, Z north. Right handed means X cross Y = Z.
    constexpr Vec3 x{1.0F, 0.0F, 0.0F};
    constexpr Vec3 y{0.0F, 1.0F, 0.0F};
    constexpr Vec3 z{0.0F, 0.0F, 1.0F};

    EXPECT_FLOAT_EQ(dot(x, y), 0.0F);
    EXPECT_FLOAT_EQ(dot(x, x), 1.0F);

    const Vec3 forward = cross(x, y);
    EXPECT_FLOAT_EQ(forward.x, z.x);
    EXPECT_FLOAT_EQ(forward.y, z.y);
    EXPECT_FLOAT_EQ(forward.z, z.z);

    // Cross product is anticommutative: the sign flip is what catches a
    // transposed implementation.
    const Vec3 reversed = cross(y, x);
    EXPECT_FLOAT_EQ(dot(forward, reversed), -1.0F);
}

TEST(Vec3, LengthAndNormalize)
{
    const Vec3 v{3.0F, 4.0F, 12.0F};
    EXPECT_FLOAT_EQ(length_squared(v), 169.0F);
    EXPECT_FLOAT_EQ(length(v), 13.0F);

    const Vec3 unit = normalized(v);
    EXPECT_NEAR(length(unit), 1.0F, 1.0e-5F);

    // Normalising a zero vector must not produce NaN: the flight model divides
    // by the airspeed and a NaN here would be invisible until much later.
    const Vec3 from_zero = normalized(Vec3{});
    EXPECT_FLOAT_EQ(from_zero.x, 0.0F);
    EXPECT_FLOAT_EQ(from_zero.y, 0.0F);
    EXPECT_FLOAT_EQ(from_zero.z, 0.0F);
    EXPECT_TRUE(is_finite(from_zero));
}

TEST(Vec3, IsFiniteDetectsCorruption)
{
    EXPECT_TRUE(is_finite(Vec3{1.0F, 2.0F, 3.0F}));
    EXPECT_FALSE(is_finite(Vec3{std::nanf(""), 0.0F, 0.0F}));
    EXPECT_FALSE(is_finite(Vec3{0.0F, std::numeric_limits<Scalar>::infinity(), 0.0F}));

    EXPECT_TRUE(is_finite(1.0F));
    EXPECT_FALSE(is_finite(std::nanf("")));
}

TEST(ScalarHelpers, ClampAndLerp)
{
    EXPECT_FLOAT_EQ(clamp(5.0F, 0.0F, 1.0F), 1.0F);
    EXPECT_FLOAT_EQ(clamp(-5.0F, 0.0F, 1.0F), 0.0F);
    EXPECT_FLOAT_EQ(clamp(0.5F, 0.0F, 1.0F), 0.5F);

    EXPECT_FLOAT_EQ(lerp(0.0F, 10.0F, 0.25F), 2.5F);
}

TEST(ScalarHelpers, MinAndMaxPromoteMixedIntegerTypes)
{
    // The reason these exist instead of std::min / std::max: the standard
    // versions demand identical argument types, and this codebase compares 32 bit
    // sequence numbers against 64 bit counters often enough that the casts would
    // be noise. Integers of differing width promote exactly, with no loss.
    static_assert(std::is_same_v<decltype(max(3, 7L)), long>);
    EXPECT_EQ(max(3, 7L), 7L);
    EXPECT_EQ(min(3, 7L), 3L);

    static_assert(std::is_same_v<decltype(max(std::uint32_t{1}, std::uint64_t{2})), std::uint64_t>);
    EXPECT_EQ(max(std::uint32_t{1}, std::uint64_t{2}), 2U);

    EXPECT_EQ(max(3, 7), 7);
    EXPECT_FLOAT_EQ(max(1.0F, 2.0F), 2.0F);
}

TEST(ScalarHelpers, MixedFloatingPointPrecisionIsNotEncouraged)
{
    // Documented rather than asserted: max(1.0F, 2.0) would widen to double and
    // max(1.0F, 2) would narrow an int, and both are refused by the warning
    // policy the project compiles with. The helpers exist for the integer case.
    // This test exists so that the reasoning is recorded next to the code, and so
    // that anyone "improving" it to handle float mixes meets the explanation.
    EXPECT_FLOAT_EQ(max(1.0F, 2.0F), 2.0F);
}

TEST(ScalarHelpers, DegreeRadianConversionRoundTrips)
{
    EXPECT_NEAR(deg_to_rad(180.0F), 3.14159265F, 1.0e-6F);
    EXPECT_NEAR(rad_to_deg(deg_to_rad(90.0F)), 90.0F, 1.0e-4F);
}

TEST(ScalarHelpers, WrapDegreesNormalisesToASingleTurn)
{
    EXPECT_FLOAT_EQ(wrap_degrees(0.0F), 0.0F);
    EXPECT_FLOAT_EQ(wrap_degrees(360.0F), 0.0F);
    EXPECT_FLOAT_EQ(wrap_degrees(370.0F), 10.0F);
    EXPECT_FLOAT_EQ(wrap_degrees(-10.0F), 350.0F);
    EXPECT_FLOAT_EQ(wrap_degrees(-370.0F), 350.0F);
}

TEST(ScalarHelpers, WrapDegreesSignedCentresOnZero)
{
    EXPECT_FLOAT_EQ(wrap_degrees_signed(0.0F), 0.0F);
    EXPECT_FLOAT_EQ(wrap_degrees_signed(180.0F), 180.0F);
    EXPECT_FLOAT_EQ(wrap_degrees_signed(190.0F), -170.0F);
    EXPECT_FLOAT_EQ(wrap_degrees_signed(-190.0F), 170.0F);
}

TEST(ScalarHelpers, AngleDifferenceTakesTheShortWayRound)
{
    // This is the quantity the heading hold and the ground steering both use, so
    // a sign error here shows up as a turn in the wrong direction.
    EXPECT_FLOAT_EQ(angle_difference_deg(10.0F, 350.0F), 20.0F);
    EXPECT_FLOAT_EQ(angle_difference_deg(350.0F, 10.0F), -20.0F);
    EXPECT_FLOAT_EQ(angle_difference_deg(90.0F, 90.0F), 0.0F);
    EXPECT_FLOAT_EQ(angle_difference_deg(0.0F, 180.0F), 180.0F);
}

TEST(ScalarHelpers, ApproachIsFrameRateIndependent)
{
    // After exactly one time constant the *error* must be 1/e, whatever the step
    // size. That property is what lets the timestep change without every filter
    // in the model having to be retuned.
    constexpr Scalar tau               = 1.0F;
    constexpr Scalar kErrorAfterOneTau = 0.36787944F; // exp(-1)

    Scalar fine = 0.0F;
    for (int i = 0; i < 1000; ++i)
    {
        fine = approach(fine, 1.0F, tau, 0.001F);
    }
    EXPECT_NEAR(1.0F - fine, kErrorAfterOneTau, 1.0e-3F);

    // One big step covering the same span of time must land in the same place.
    // If it does not, the filter is step-size dependent and every latency figure
    // derived from it is a function of the sample rate.
    const Scalar coarse = approach(0.0F, 1.0F, tau, 1.0F);
    EXPECT_NEAR(1.0F - coarse, kErrorAfterOneTau, 1.0e-4F);

    // Half a time constant is half the decay.
    const Scalar half = approach(0.0F, 1.0F, tau, 0.5F);
    EXPECT_NEAR(1.0F - half, 0.60653066F, 1.0e-4F);

    // A zero time constant means "go there now".
    EXPECT_FLOAT_EQ(approach(0.0F, 1.0F, 0.0F, 0.01F), 1.0F);
}

TEST(ScalarHelpers, MoveTowardsRespectsTheRateLimit)
{
    Scalar value = 0.0F;

    // Rate limited: 10 units per second for 0.5 s is 5 units, not 100.
    value = move_towards(value, 100.0F, 10.0F, 0.5F);
    EXPECT_FLOAT_EQ(value, 5.0F);

    // Close enough to arrive exactly, and never to overshoot.
    value = move_towards(value, 6.0F, 10.0F, 1.0F);
    EXPECT_FLOAT_EQ(value, 6.0F);
}

} // namespace
} // namespace fse