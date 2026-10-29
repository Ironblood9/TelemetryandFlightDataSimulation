// ---------------------------------------------------------------------------
// CRC-32/ISO-HDLC tests.
//
// The check value is the single most important assertion in this file, and
// arguably in the whole protocol layer. If it fails, every integrity check
// above it is built on a broken foundation, and the failure is in this file
// rather than in the caller.
//
// The check value 0xCBF43926 for the ASCII string "123456789" is the
// standard test vector from the CRC Zoo (Ross Williams). It is the same value
// zlib's crc32() produces, which is what makes it a cross-implementation
// reference rather than a self-referential golden value.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "protocol/crc32.hpp"

namespace fse
{
namespace
{

using fse::protocol::crc32_iso_hdlc;

TEST(Crc32, CheckValueMatchesTheStandard)
{
    // This is the test that matters. If this fails, nothing above it can be
    // trusted.
    constexpr std::string_view kCheckInput = "123456789";
    EXPECT_EQ(crc32_iso_hdlc(kCheckInput), 0xCBF43926U);
}

TEST(Crc32, EmptyInputIsZero)
{
    // CRC of zero bytes is defined as the initial value XORed with itself,
    // which is zero. A non-zero result here would mean the final XOR is being
    // skipped or applied twice.
    EXPECT_EQ(crc32_iso_hdlc(nullptr, 0), 0U);
}

TEST(Crc32, EmptyBufferOverloadMatches)
{
    const std::vector<std::uint8_t> empty{};
    EXPECT_EQ(crc32_iso_hdlc(empty), 0U);
}

TEST(Crc32, SingleByteValuesAreDistinct)
{
    // Every possible byte value must produce a distinct CRC, otherwise the
    // table has a collision and two different single-byte payloads would be
    // indistinguishable. This is the property that makes a 32-bit CRC useful.
    std::array<std::uint32_t, 256> seen{};
    for (std::uint32_t i = 0; i < 256; ++i)
    {
        const std::uint8_t  byte = static_cast<std::uint8_t>(i);
        const std::uint32_t crc  = crc32_iso_hdlc(&byte, 1);
        seen[i]                  = crc;
    }

    std::set<std::uint32_t> distinct(seen.begin(), seen.end());
    EXPECT_EQ(distinct.size(), 256U) << "CRC-32 has a collision on single-byte inputs";
}

TEST(Crc32, KnownVectorTheQuickBrownFox)
{
    // A longer known vector, cross-checked against zlib and online CRC
    // calculators. Catches a table generation bug that happens to preserve
    // the short check value.
    constexpr std::string_view kFox = "The quick brown fox jumps over the lazy dog";
    EXPECT_EQ(crc32_iso_hdlc(kFox), 0x414FA339U);
}

TEST(Crc32, KnownVectorHelloWorld)
{
    constexpr std::string_view kHello = "Hello, World!";
    EXPECT_EQ(crc32_iso_hdlc(kHello), 0xEC4AC3D0U);
}

TEST(Crc32, AppendIsNotCommutative)
{
    // CRC is not a simple XOR of per-byte contributions. If it were, appending
    // would commute and the integrity check would be trivially forgeable.
    constexpr std::string_view a = "ab";
    constexpr std::string_view b = "ba";

    EXPECT_NE(crc32_iso_hdlc(a), crc32_iso_hdlc(b));
}

TEST(Crc32, CrcIsNotXorOfParts)
{
    // CRC is not a simple XOR of per-byte or per-chunk contributions. If it
    // were, the integrity check would be trivially forgeable and a single bit
    // flip in the payload would be invisible when two chunks happened to
    // cancel. This is why the protocol computes the CRC over the *entire* frame
    // in one call rather than per-field.
    constexpr std::string_view kData = "123456789";

    const std::uint32_t whole = crc32_iso_hdlc(kData);

    const std::size_t   half   = kData.size() / 2;
    const std::uint32_t part_a = crc32_iso_hdlc(kData.substr(0, half));
    const std::uint32_t part_b = crc32_iso_hdlc(kData.substr(half));

    EXPECT_NE(part_a ^ part_b, whole) << "CRC is behaving as a simple XOR of chunks";
}

TEST(Crc32, KnownVectorZerosAndOnes)
{
    // All-zeros and all-ones are the edge cases that expose off-by-one errors
    // in the table indexing.
    std::array<std::uint8_t, 4> zeros{};
    std::array<std::uint8_t, 4> ones{};
    ones.fill(0xFFU);

    // These are known values from zlib's crc32().
    EXPECT_EQ(crc32_iso_hdlc(zeros), 0x2144DF1CU);
    EXPECT_EQ(crc32_iso_hdlc(ones), 0xFFFFFFFFU);
}

} // namespace
} // namespace fse