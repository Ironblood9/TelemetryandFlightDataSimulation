#pragma once

/// @file types.hpp
/// @brief Core scalar and vector types plus the small math helpers the flight
///        model needs.
///
/// Everything in the simulation is single precision on purpose. The values the
/// model computes are the values that go into the telemetry frame, so keeping
/// one precision end to end removes a whole class of "the graph and the packet
/// disagree" defects.
///
/// ## Reproducibility caveat
/// IEEE-754 pins `+ - * /` and `sqrt` exactly, so those are bit-identical on
/// every conforming platform. Transcendental functions (`sin`, `cos`, `atan2`,
/// `exp`, `pow`, `log`) are *implementation defined* and may differ by an ULP
/// between libm implementations. The determinism tests therefore assert exact
/// equality for repeated runs of the same binary, and cross-platform equality
/// only for the integer PRNG stream. See docs/architecture.md.

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace fse
{

/// The single floating point type used across the simulation and the wire.
using Scalar = float;

/// Vector in the world frame: X east, Y up, Z north. Units are metres.
struct Vec3
{
    Scalar x{0.0F};
    Scalar y{0.0F};
    Scalar z{0.0F};

    constexpr Vec3() noexcept = default;

    constexpr Vec3(Scalar x_in, Scalar y_in, Scalar z_in) noexcept : x(x_in), y(y_in), z(z_in) {}

    constexpr Vec3& operator+=(const Vec3& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(Scalar s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    constexpr Vec3 operator-() const noexcept
    {
        return {-x, -y, -z};
    }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 lhs, const Vec3& rhs) noexcept
{
    lhs += rhs;
    return lhs;
}

[[nodiscard]] constexpr Vec3 operator-(Vec3 lhs, const Vec3& rhs) noexcept
{
    lhs -= rhs;
    return lhs;
}

[[nodiscard]] constexpr Vec3 operator*(Vec3 v, Scalar s) noexcept
{
    v *= s;
    return v;
}

[[nodiscard]] constexpr Vec3 operator*(Scalar s, Vec3 v) noexcept
{
    v *= s;
    return v;
}

[[nodiscard]] constexpr Scalar dot(const Vec3& a, const Vec3& b) noexcept
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/// Right-handed cross product: Y up, Z north, X east.
[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept
{
    return {(a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x)};
}

[[nodiscard]] constexpr Scalar length_squared(const Vec3& v) noexcept
{
    return dot(v, v);
}

[[nodiscard]] inline Scalar length(const Vec3& v) noexcept
{
    return std::sqrt(length_squared(v));
}

/// Returns the unit vector, or the zero vector if @p v is (near) zero.
[[nodiscard]] inline Vec3 normalized(const Vec3& v) noexcept
{
    const Scalar len_sq = length_squared(v);
    if (len_sq <= 1.0e-12F)
    {
        return {};
    }
    return v * (1.0F / std::sqrt(len_sq));
}

/// True when every component is finite; guards against NaN leaking into a frame.
///
/// `inline` rather than `constexpr`: `std::isfinite` is not a constant
/// expression in C++20, so a `constexpr` wrapper could never satisfy the
/// standard's requirement that *some* invocation be constant evaluable.
[[nodiscard]] inline bool is_finite(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

[[nodiscard]] inline bool is_finite(Scalar v) noexcept
{
    return std::isfinite(v);
}

// ---------------------------------------------------------------------------
// Scalar helpers
// ---------------------------------------------------------------------------

template <typename T>
[[nodiscard]] constexpr T clamp(T value, T low, T high) noexcept
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

/// Own `min`/`max` rather than using `std::min`/`std::max`.
///
/// The reason is mixed *integer* types: std::min and std::max demand identical
/// argument types, and this codebase compares 32 bit sequence numbers against
/// 64 bit counters and size_t indices against literals often enough that the
/// casts would be noise. These promote to the common type instead.
///
/// Mixing floating point precisions is deliberately not the use case. The
/// promotion is the one a conditional operator would perform, so max(1.0F, 2.0)
//  would widen to double -- and the project compiles with -Wdouble-promotion and
/// -Wimplicit-int-float-conversion precisely so that a silent change of
/// precision is reported at the call site. A helper that quietly encouraged it
/// would undo that policy at every call site it saved a cast.
template <typename A, typename B>
[[nodiscard]] constexpr std::common_type_t<A, B> max(A a, B b) noexcept
{
    return (a < b) ? b : a;
}

template <typename A, typename B>
[[nodiscard]] constexpr std::common_type_t<A, B> min(A a, B b) noexcept
{
    return (b < a) ? b : a;
}

[[nodiscard]] constexpr Scalar lerp(Scalar a, Scalar b, Scalar t) noexcept
{
    return a + ((b - a) * t);
}

[[nodiscard]] constexpr Scalar deg_to_rad(Scalar deg) noexcept
{
    return deg * 0.017453292519943295F;
}

[[nodiscard]] constexpr Scalar rad_to_deg(Scalar rad) noexcept
{
    return rad * 57.29577951308232F;
}

/// Wraps an angle into `[0, 360)`.
[[nodiscard]] constexpr Scalar wrap_degrees(Scalar deg) noexcept
{
    Scalar wrapped = deg;
    while (wrapped < 0.0F)
    {
        wrapped += 360.0F;
    }
    while (wrapped >= 360.0F)
    {
        wrapped -= 360.0F;
    }
    return wrapped;
}

/// Wraps an angle into `(-180, 180]`, which is what the roll channel carries.
[[nodiscard]] constexpr Scalar wrap_degrees_signed(Scalar deg) noexcept
{
    const Scalar wrapped = wrap_degrees(deg);
    return (wrapped > 180.0F) ? (wrapped - 360.0F) : wrapped;
}

/// Smallest signed rotation taking @p from_deg to @p to_deg, in `(-180, 180]`.
[[nodiscard]] constexpr Scalar angle_difference_deg(Scalar to_deg, Scalar from_deg) noexcept
{
    return wrap_degrees_signed(to_deg - from_deg);
}

/// Frame-rate independent exponential approach of @p current towards @p target.
///
/// `tau` is the time constant: after `tau` seconds the remaining error is
/// `1/e`. Using an exponential rather than a linear rate limit keeps the
/// response continuous when the timestep changes.
[[nodiscard]] inline Scalar approach(Scalar current, Scalar target, Scalar tau, Scalar dt) noexcept
{
    if (tau <= 0.0F)
    {
        return target;
    }
    const Scalar alpha = 1.0F - std::exp(-dt / tau);
    return current + ((target - current) * alpha);
}

/// Moves @p current towards @p target by at most @p max_rate * @p dt.
[[nodiscard]] inline Scalar move_towards(Scalar current, Scalar target, Scalar max_rate,
                                         Scalar dt) noexcept
{
    const Scalar error = target - current;
    const Scalar limit = max_rate * dt;
    if (error > limit)
    {
        return current + limit;
    }
    if (error < -limit)
    {
        return current - limit;
    }
    return target;
}

// --- Physical constants -----------------------------------------------------
// ISA standard atmosphere at sea level unless stated otherwise.

inline constexpr Scalar kGravity                = 9.80665F;   // m/s^2
inline constexpr Scalar kSeaLevelDensity        = 1.225F;     // kg/m^3
inline constexpr Scalar kSeaLevelPressure       = 101325.0F;  // Pa
inline constexpr Scalar kSeaLevelTemperature    = 288.15F;    // K
inline constexpr Scalar kTemperatureLapseRate   = 0.0065F;    // K/m
inline constexpr Scalar kSpecificGasConstantAir = 287.05287F; // J/(kg*K)

/// Exponent of the ISA troposphere density law
/// rho(h) = rho0 * (1 - L*h/T0) ^ kAirDensityExponent.
///
/// The exponent is g/(R*L) - 1 = 4.25588, not g/(R*L): the `- 1` comes from
/// dividing the ideal gas law by the hydrostatic equation and it is easy to
/// forget. The reference test pins the resulting density against the ISA table
/// at 5 000 m, which is the only reason this constant is right.
inline constexpr Scalar kAirDensityExponent = 4.2558797F;

/// Standard sea level pressure expressed in kilopascals, matching the units of
/// the telemetry frame.
inline constexpr Scalar kSeaLevelPressureKpa = 101.325F;

} // namespace fse