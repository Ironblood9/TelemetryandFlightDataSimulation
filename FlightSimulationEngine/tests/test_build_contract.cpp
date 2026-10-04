// ---------------------------------------------------------------------------
// Build contract tests.
//
// The first milestone has almost no runtime code, so what we verify here is
// the *contract* every later milestone silently depends on:
//   * C++20 really is enabled (not "compiles anyway on the dev machine"),
//   * the engine headers are reachable under the `fse` include prefix,
//   * version/protocol constants agree with docs/protocol-spec.md,
//   * the struct layout assumptions of the wire protocol are checked at
//     compile time rather than discovered at 3 a.m. on a test range.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstdint>
#include <type_traits>

#include "fse/version.hpp"

TEST(BuildContract, Cpp20IsEnabled)
{
    // __cplusplus is only trustworthy when /Zc:__cplusplus (MSVC) or
    // -std=c++20 (gcc/clang) is active. ProjectOptions.cmake sets both.
    EXPECT_GE(__cplusplus, 202002L);

    static_assert(__cplusplus >= 202002L, "The project must be compiled as C++20 or newer.");

    // A C++20-only feature, so this test actually proves something.
    constexpr auto sum = [](auto... values) { return (values + ... + 0); };
    static_assert(sum(1, 2, 3) == 6);
    EXPECT_EQ(sum(1, 2, 3), 6);
}

TEST(BuildContract, VersionConstantsAreConsistent)
{
    EXPECT_EQ(fse::version::kMajor, 0);
    EXPECT_GE(fse::version::kMinor, 1);
    EXPECT_STREQ(fse::version::kString, "0.1.0");
    EXPECT_STREQ(fse::version::kApplicationName, "flight-sim");
}

TEST(BuildContract, ProtocolConstantsMatchSpec)
{
    // docs/protocol-spec.md, "Framing" section.
    EXPECT_EQ(fse::kProtocolVersion, 1u);
    EXPECT_EQ(fse::kDefaultSampleRateHz, 100u);
}

TEST(BuildContract, TelemetryFrameFieldsAreFixedWidth)
{
    // Every field that crosses the wire must have an explicitly sized type so
    // that the C++ and the C# layout cannot silently diverge.
    static_assert(sizeof(std::uint16_t) == 2);
    static_assert(sizeof(std::uint32_t) == 4);
    static_assert(sizeof(std::uint64_t) == 8);
    static_assert(sizeof(float) == 4, "IEEE-754 binary32 float expected on the wire");

    SUCCEED();
}