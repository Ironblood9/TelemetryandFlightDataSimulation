#include "fse/noise.hpp"

#include <cmath>
#include <cstring>

namespace fse::random
{

namespace
{

/// Two to the power of -32, the scale factor for a full 32 bit output.
constexpr Scalar kInvTwoPow32 = 2.3283064365386963e-10F;

/// Two to the power of -24: the scale factor for the 24 bit uniform draw used by
/// `next_unit()`. Exactly representable in binary32, so the product cannot round
/// up to 1.0F.
constexpr Scalar kInvTwoPow24 = 5.9604644775390625e-08F;

constexpr Scalar kTwoPi = 6.28318530717958647692F;

} // namespace

// ---------------------------------------------------------------------------
// Pcg32
// ---------------------------------------------------------------------------

void Pcg32::reseed(std::uint64_t seed, std::uint64_t stream) noexcept
{
    // The increment must be odd; the reference shifts the stream left by one
    // and forces the low bit.
    increment_ = (stream << 1U) | 1U;
    state_     = 0U;

    // The reference discards one output while mixing the seed in. The value is
    // thrown away deliberately, hence the explicit (void).
    (void)next_u32();
    state_ += seed;
    (void)next_u32();

    // A reseed must not expose a stale cached deviate, otherwise the stream
    // position would depend on how many draws happened before it.
    has_cached_normal_ = false;
    cached_normal_     = 0.0F;
}

std::uint32_t Pcg32::next_u32() noexcept
{
    const std::uint64_t previous = state_;
    state_                       = (previous * kMultiplier) + increment_;

    // XSH RR: xorshift the high bits down, then rotate by the top five bits.
    const std::uint32_t xorshifted =
        static_cast<std::uint32_t>(((previous >> 18U) ^ previous) >> 27U);
    const std::uint32_t rotation = static_cast<std::uint32_t>(previous >> 59U);

    return (xorshifted >> rotation) | (xorshifted << ((0U - rotation) & 31U));
}

Scalar Pcg32::next_unit() noexcept
{
    // Only the top 24 bits are used. Multiplying the full 32 bit value by 2^-32
    // in single precision would round 0xFFFFFFFF up to exactly 1.0f — the
    // float nearest to 2^32 is 2^32 — so the documented [0, 1) range would be
    // violated for the whole top 512 values of the output. 24 bits of mantissa
    // is four orders of magnitude more resolution than any noise application
    // here needs, and it is provably in range.
    return static_cast<Scalar>(next_u32() >> 8U) * kInvTwoPow24;
}

Scalar Pcg32::next_range(Scalar low, Scalar high) noexcept
{
    return low + ((high - low) * next_unit());
}

Scalar Pcg32::next_normal() noexcept
{
    if (has_cached_normal_)
    {
        has_cached_normal_ = false;
        return cached_normal_;
    }

    // Box-Muller. u1 is shifted off zero because log(0) diverges; the smallest
    // positive 32 bit value is 1 / 2^32, which keeps the log finite.
    const Scalar u1 = (static_cast<Scalar>(next_u32()) + kInvTwoPow32) * kInvTwoPow32;
    const Scalar u2 = next_unit();

    const Scalar radius = std::sqrt(-2.0F * std::log(u1));
    const Scalar angle  = kTwoPi * u2;

    // Cached rather than discarded: two deviates cost one pair of transcendentals.
    cached_normal_     = radius * std::sin(angle);
    has_cached_normal_ = true;

    return radius * std::cos(angle);
}

// ---------------------------------------------------------------------------
// FNV-1a
// ---------------------------------------------------------------------------

std::uint64_t fnv1a(std::uint64_t hash, const void* data, std::size_t size) noexcept
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i)
    {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= kFnv1aPrime;
    }
    return hash;
}

std::uint64_t fnv1a(std::uint64_t hash, std::uint32_t value) noexcept
{
    // Mixed byte by byte so that the result does not depend on the host's
    // endianness.
    for (int shift = 0; shift < 32; shift += 8)
    {
        hash ^= static_cast<std::uint64_t>((value >> shift) & 0xFFU);
        hash *= kFnv1aPrime;
    }
    return hash;
}

std::uint64_t fnv1a(std::uint64_t hash, std::uint64_t value) noexcept
{
    for (int shift = 0; shift < 64; shift += 8)
    {
        hash ^= static_cast<std::uint64_t>((value >> shift) & 0xFFU);
        hash *= kFnv1aPrime;
    }
    return hash;
}

std::uint64_t fnv1a_scalar(std::uint64_t hash, Scalar value) noexcept
{
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value), "Scalar must be a 32 bit float");
    std::memcpy(&bits, &value, sizeof(bits));
    return fnv1a(hash, bits);
}

// ---------------------------------------------------------------------------
// ColouredNoise
// ---------------------------------------------------------------------------

void ColouredNoise::reseed(std::uint64_t seed, std::uint64_t stream) noexcept
{
    generator_.reseed(seed, stream);
    value_ = 0.0F;
}

Scalar ColouredNoise::next(Scalar dt) noexcept
{
    if (dt <= 0.0F || tau_ <= 0.0F)
    {
        return value_;
    }

    // Discrete Ornstein-Uhlenbeck step:
    //     y[n+1] = (1 - a) * y[n] + k * w[n],   a = dt / tau
    // with w standard normal.
    //
    // The stationary variance follows from V = (1-a)^2 V + k^2, that is
    //     V = k^2 / (a * (2 - a))
    // so the gain which produces V = sigma^2 exactly is
    //     k = sigma * sqrt(a * (2 - a)).
    // Dropping the `a` from that denominator leaves the result a factor 1/a too
    // large - 400x at tau/dt = 400 - and the symptom is a slow wander instead
    // of the small bounded noise the caller asked for.
    const Scalar alpha   = clamp(dt / tau_, 1.0e-6F, 1.0F);
    const Scalar deviate = generator_.next_normal();

    const Scalar gain = sigma_ * std::sqrt(alpha * (2.0F - alpha));
    value_            = (value_ * (1.0F - alpha)) + (gain * deviate);
    return value_;
}

} // namespace fse::random
