// ---------------------------------------------------------------------------
// PRNG tests.
//
// The integer stream is asserted against the *upstream reference vectors*, not
// against whatever this implementation happens to produce. That is the whole
// point of writing our own generator: a self-referential golden value proves only
// that the code does not change, whereas a reference vector proves it is PCG32.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fse/noise.hpp"

namespace fse::random
{
namespace
{

/// The canonical `pcg32-demo.c` output: pcg32_srandom_r(&rng, 42, 54).
///
/// Reference: https://www.pcg-random.org/download.html
constexpr std::uint32_t kReferenceOutputs[] = {0xa15c02b7U, 0x7b47f409U, 0xba1d3330U,
                                               0x83d2f293U, 0xbfa4784bU, 0xcbed606eU};

TEST(Pcg32, MatchesUpstreamReferenceVectors)
{
    Pcg32 rng{42U, 54U};

    for (const std::uint32_t expected : kReferenceOutputs)
    {
        EXPECT_EQ(rng.next_u32(), expected) << "stream diverged from the PCG32 reference";
    }
}

TEST(Pcg32, SameSeedProducesSameStream)
{
    Pcg32 a{12345U, 1U};
    Pcg32 b{12345U, 1U};

    for (int i = 0; i < 1000; ++i)
    {
        EXPECT_EQ(a.next_u32(), b.next_u32()) << "diverged at draw " << i;
    }
}

TEST(Pcg32, DifferentStreamsAreDecorrelated)
{
    Pcg32 a{42U, 1U};
    Pcg32 b{42U, 2U};

    int collisions = 0;
    for (int i = 0; i < 256; ++i)
    {
        if (a.next_u32() == b.next_u32())
        {
            ++collisions;
        }
    }

    // Two independent 32 bit streams collide with probability 2^-32 per draw, so
    // even one collision in 256 draws would be alarming.
    EXPECT_EQ(collisions, 0);
}

TEST(Pcg32, ReseedReturnsToTheSamePosition)
{
    Pcg32 rng{7U, 3U};
    for (int i = 0; i < 100; ++i)
    {
        (void)rng.next_u32();
    }

    rng.reseed(7U, 3U);
    const std::uint32_t first = rng.next_u32();

    Pcg32 fresh{7U, 3U};
    EXPECT_EQ(fresh.next_u32(), first);
}

TEST(Pcg32, ReseedDiscardsTheCachedNormal)
{
    Pcg32 rng{99U};

    // Draw an odd number of normals so a deviate is left in the Box-Muller
    // cache. If reseed did not clear it, the next draw would return that stale
    // value instead of a fresh one and the stream position would depend on
    // history.
    (void)rng.next_normal();
    rng.reseed(99U);

    Pcg32 fresh{99U};
    EXPECT_FLOAT_EQ(rng.next_normal(), fresh.next_normal());
}

TEST(Pcg32, NextUnitStaysInRange)
{
    Pcg32 rng{2024U};

    for (int i = 0; i < 20000; ++i)
    {
        const Scalar value = rng.next_unit();
        ASSERT_GE(value, 0.0F);
        ASSERT_LT(value, 1.0F);
    }
}

TEST(Pcg32, NextUnitIsRoughlyUniform)
{
    Pcg32 rng{555U};

    constexpr int kBins       = 20;
    int           bins[kBins] = {};
    constexpr int kDraws      = 200000;

    for (int i = 0; i < kDraws; ++i)
    {
        const Scalar value = rng.next_unit();
        const int    index = static_cast<int>(value * static_cast<Scalar>(kBins));
        ASSERT_GE(index, 0);
        ASSERT_LT(index, kBins);
        ++bins[index];
    }

    // Every bin within 5% of the expected count: a chi-square test at this
    // sample size would trip on far smaller deviations than any that matter.
    const double expected = static_cast<double>(kDraws) / static_cast<double>(kBins);
    for (const int count : bins)
    {
        EXPECT_NEAR(static_cast<double>(count), expected, expected * 0.05);
    }
}

TEST(Pcg32, NextNormalHasTheRequestedMoments)
{
    Pcg32 rng{31337U};

    constexpr int kDraws      = 200000;
    double        sum         = 0.0;
    double        sum_squares = 0.0;

    for (int i = 0; i < kDraws; ++i)
    {
        const double value = static_cast<double>(rng.next_normal());
        sum += value;
        sum_squares += value * value;
    }

    const double mean = sum / static_cast<double>(kDraws);
    // Unbiased sample variance: the Box-Muller cache means consecutive draws are
    // not strictly independent, so n-1 is the honest denominator.
    const double variance =
        (sum_squares - (sum * sum / static_cast<double>(kDraws))) / static_cast<double>(kDraws - 1);

    EXPECT_NEAR(mean, 0.0, 0.02);
    EXPECT_NEAR(std::sqrt(variance), 1.0, 0.02);
}

TEST(Pcg32, NextNormalScalesAndOffsets)
{
    Pcg32 a{4242U};
    Pcg32 b{4242U};

    for (int i = 0; i < 500; ++i)
    {
        const Scalar value     = a.next_normal(100.0F, 5.0F);
        const Scalar reference = b.next_normal();
        EXPECT_FLOAT_EQ(value, 100.0F + (5.0F * reference));
    }
}

TEST(Pcg32, NextNormalIsNeverInfinite)
{
    // The Box-Muller transform evaluates log(u1). If u1 could be zero the result
    // would be an infinity, and a NaN would propagate into every telemetry
    // field. The generator shifts u1 off zero precisely to prevent this.
    Pcg32 rng{1U};

    for (int i = 0; i < 100000; ++i)
    {
        const Scalar value = rng.next_normal();
        ASSERT_TRUE(std::isfinite(value)) << "non finite deviate at draw " << i;
    }
}

TEST(ColouredNoise, HasTheRequestedVarianceAndStaysBounded)
{
    ColouredNoise noise{8080U, 2.0F, 4.0F};

    constexpr Scalar dt     = 0.01F;
    constexpr int    kDraws = 400000; // 4000 seconds, 1000 correlation times

    double sum         = 0.0;
    double sum_squares = 0.0;
    double max_abs     = 0.0;

    for (int i = 0; i < kDraws; ++i)
    {
        const Scalar value = noise.next(dt);
        sum += static_cast<double>(value);
        sum_squares += static_cast<double>(value) * static_cast<double>(value);
        const double magnitude = std::abs(static_cast<double>(value));
        if (magnitude > max_abs)
        {
            max_abs = magnitude;
        }
    }

    const double n        = static_cast<double>(kDraws);
    const double mean     = sum / n;
    const double variance = (sum_squares - (sum * sum / n)) / (n - 1.0);

    // The variance is the mathematical claim and is asserted tightly.
    EXPECT_NEAR(std::sqrt(variance), 2.0F, 0.05F);

    // The mean of a *coloured* process is estimated from only T/tau independent
    // samples — 1000 here — so its standard error is about 2/sqrt(1000) = 0.06
    // even before accounting for the correlation. A tight bound on the mean would
    // be asserting that a seeded realisation happens to be convenient.
    EXPECT_NEAR(mean, 0.0, 0.25);

    // A stationary process with sigma = 2 has essentially no excursion beyond
    // eight sigma. This also catches an integration that runs away, which a
    // mis-scaled gain does at large tau/dt.
    EXPECT_LT(max_abs, 16.0);
}

TEST(ColouredNoise, ZeroTauIsConstant)
{
    ColouredNoise noise{5U, 1.0F, 0.0F};

    Scalar value = noise.next(0.01F);
    for (int i = 0; i < 100; ++i)
    {
        EXPECT_FLOAT_EQ(noise.next(0.01F), value);
    }
    value = noise.next(0.0F);
    EXPECT_FLOAT_EQ(value, noise.value());
}

TEST(Fnv1a, IsStableAndOrderSensitive)
{
    // Not constexpr: hashing goes through a byte loop, which cannot be evaluated
    // at compile time.
    const std::uint64_t empty_hash = fnv1a(kFnv1aOffsetBasis, nullptr, 0U);
    EXPECT_EQ(empty_hash, kFnv1aOffsetBasis);

    // Known value for the FNV-1a 64 bit hash of "hello".
    EXPECT_EQ(fnv1a(kFnv1aOffsetBasis, "hello", 5U), 0xa430d84680aabd0bULL);

    const std::uint64_t ab = fnv1a(kFnv1aOffsetBasis, static_cast<std::uint32_t>(0x0102U));
    const std::uint64_t ba = fnv1a(kFnv1aOffsetBasis, static_cast<std::uint32_t>(0x0201U));
    EXPECT_NE(ab, ba);
}

TEST(Fnv1a, DistinguishesPositiveAndNegativeZero)
{
    // -0.0 and +0.0 compare equal, so a hash over the numeric value would hide
    // a sign bug. Hashing the bit pattern must not.
    EXPECT_NE(fnv1a_scalar(kFnv1aOffsetBasis, 0.0F), fnv1a_scalar(kFnv1aOffsetBasis, -0.0F));
}

} // namespace
} // namespace fse::random