#pragma once

/// @file noise.hpp
/// @brief Deterministic pseudo random number generation and sensor noise.
///
/// ## Why not `<random>`?
/// `std::uniform_real_distribution` and `std::normal_distribution` are
/// *implementation defined*. Two compilers, or two versions of the same
/// compiler, legitimately produce different values for the same engine and the
/// same seed. A simulation whose central claim is "same seed, same flight"
/// cannot be regression tested on top of that, so this project implements its
/// own generator and its own normal transform.
///
/// The integer stream is bit-identical on every platform: it only uses
/// multiplication, XOR and shifts. The Gaussian transform uses `log`/`cos`/
/// `sin`, which are *not* bit-reproducible across libm implementations, so
/// exact reproducibility is guaranteed per build, not across compilers. That
/// distinction is asserted explicitly in `test_determinism.cpp` rather than
/// glossed over.

#include <cstddef>
#include <cstdint>

#include "fse/types.hpp"

namespace fse::random
{

/// FNV-1a 64 bit offset basis and prime.
inline constexpr std::uint64_t kFnv1aOffsetBasis = 14695981039346656037ULL;
inline constexpr std::uint64_t kFnv1aPrime       = 1099511628211ULL;

/// PCG32 — permuted congruential generator, 64 bit state, 32 bit output.
///
/// Reference implementation by M. E. O'Neill, matching `pcg32_random_r` and
/// `pcg32_srandom_r` bit for bit. Verified against the upstream reference
/// vectors in `test_noise.cpp`.
class Pcg32
{
public:
    /// @param seed        Initial state seed.
    /// @param stream      Stream selector. Different streams decorrelate two
    ///                    generators that share a seed, which is how the sensor
    ///                    suite gives each channel its own reproducible noise.
    explicit Pcg32(std::uint64_t seed, std::uint64_t stream = kDefaultStream) noexcept
    {
        reseed(seed, stream);
    }

    /// Restores the generator to the state defined by @p seed and @p stream.
    void reseed(std::uint64_t seed, std::uint64_t stream = kDefaultStream) noexcept;

    /// Next raw 32 bit output.
    [[nodiscard]] std::uint32_t next_u32() noexcept;

    /// Uniform in `[0, 1)` with 32 bits of resolution.
    [[nodiscard]] Scalar next_unit() noexcept;

    /// Uniform in `[low, high)`.
    [[nodiscard]] Scalar next_range(Scalar low, Scalar high) noexcept;

    /// Standard normal deviate via the Box-Muller transform.
    ///
    /// Two deviates are produced per pair of transcendentals and the second is
    /// cached, so half the `log`/`cos` calls are avoided. The cache is dropped
    /// by `reseed()`, keeping the stream position a pure function of the number
    /// of draws.
    [[nodiscard]] Scalar next_normal() noexcept;

    /// Normal deviate with zero mean and unit variance, scaled and offset.
    [[nodiscard]] Scalar next_normal(Scalar mean, Scalar stddev) noexcept
    {
        return mean + (stddev * next_normal());
    }

private:
    static constexpr std::uint64_t kDefaultStream = 0xda3e39cb94b95bdbULL;
    static constexpr std::uint64_t kMultiplier    = 6364136223846793005ULL;

    std::uint64_t state_{0};
    std::uint64_t increment_{0};

    bool   has_cached_normal_{false};
    Scalar cached_normal_{0.0F};
};

/// Adds @p size bytes at @p data to the running FNV-1a hash.
///
/// Used by the determinism tests to compare whole telemetry streams with a
/// single integer instead of millions of element-wise comparisons.
[[nodiscard]] std::uint64_t fnv1a(std::uint64_t hash, const void* data, std::size_t size) noexcept;

[[nodiscard]] std::uint64_t fnv1a(std::uint64_t hash, std::uint32_t value) noexcept;
[[nodiscard]] std::uint64_t fnv1a(std::uint64_t hash, std::uint64_t value) noexcept;

/// Hashes the raw IEEE-754 bit pattern, not the numeric value.
///
/// Two NaNs with different payloads must hash differently, and `+0.0` must not
/// collide with `-0.0`, so the bytes are what gets hashed.
[[nodiscard]] std::uint64_t fnv1a_scalar(std::uint64_t hash, Scalar value) noexcept;

/// First order low-pass coloured noise, used for gusts and slow sensor drift.
///
/// A white deviate is fed through the discrete Ornstein-Uhlenbeck recursion
///     y[n+1] = (1 - a) y[n] + sigma sqrt(a (2 - a)) w[n],   a = dt / tau
/// whose stationary standard deviation is exactly `sigma` for any timestep.
/// The gain is what makes that true; a plain `y += sigma (w - y)` is neither
/// step-size independent nor correctly scaled.
///
/// Compared with summing raw deviates, this cannot run away, so it stays well
/// behaved for long missions at any timestep.
class ColouredNoise
{
public:
    /// @param sigma  Standard deviation of the *output* in the limit.
    /// @param tau    Correlation time constant in seconds.
    ColouredNoise(std::uint64_t seed, Scalar sigma, Scalar tau, std::uint64_t stream = 1) noexcept
        : generator_{seed, stream}, sigma_{sigma}, tau_{tau}, value_{0.0F}
    {
    }

    void reseed(std::uint64_t seed, std::uint64_t stream = 1) noexcept;

    /// Advances the process by @p dt seconds and returns the new value.
    [[nodiscard]] Scalar next(Scalar dt) noexcept;

    [[nodiscard]] Scalar value() const noexcept
    {
        return value_;
    }
    void set_value(Scalar v) noexcept
    {
        value_ = v;
    }

private:
    Pcg32  generator_;
    Scalar sigma_;
    Scalar tau_;
    Scalar value_;
};

} // namespace fse::random