// ---------------------------------------------------------------------------
// CRC-32/ISO-HDLC
//
// The same algorithm as zlib's crc32() and PKZIP's CRC-32. Reflected
// polynomial 0xEDB88320, initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF.
// Check value for the ASCII string "123456789" is 0xCBF43926.
//
// Every other integrity check in this project is built on top of this one, so
// the check value is asserted in the very first test. If that test ever fails,
// nothing above it can be trusted and the failure is in this file, not in the
// caller.
// ---------------------------------------------------------------------------

#pragma once

#include <cstddef>
#include <cstdint>

namespace fse
{
namespace protocol
{

/// CRC-32/ISO-HDLC over a contiguous byte range.
///
/// The result is the same as zlib's `crc32()` and PKZIP's CRC-32. The
/// implementation is table-driven with a compile-time generated table, so
/// there is no first-call initialisation and no hidden state.
[[nodiscard]] std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t length) noexcept;

/// Overload for a contiguous range (std::span, std::string_view, std::vector).
template <typename Range>
[[nodiscard]] std::uint32_t crc32_iso_hdlc(const Range& data) noexcept
{
    if constexpr (requires { data.data(); })
    {
        return crc32_iso_hdlc(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
    }
    else
    {
        return crc32_iso_hdlc(data.data(), data.size());
    }
}

} // namespace protocol
} // namespace fse