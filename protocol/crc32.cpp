#include "protocol/crc32.hpp"

#include <array>

namespace fse
{
namespace protocol
{

namespace
{

/// Reflected polynomial for CRC-32/ISO-HDLC.
constexpr std::uint32_t kPolynomial = 0xEDB88320U;

/// Build the 256-entry lookup table at compile time. The table is the only
/// state the algorithm carries, and because it is constexpr there is no
/// first-call initialisation and no thread-safety concern.
constexpr std::array<std::uint32_t, 256> make_table()
{
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t i = 0; i < 256; ++i)
    {
        std::uint32_t crc = i;
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ kPolynomial;
            }
            else
            {
                crc >>= 1;
            }
        }
        table[i] = crc;
    }
    return table;
}

constexpr std::array<std::uint32_t, 256> kTable = make_table();

} // namespace

std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t length) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length; ++i)
    {
        crc = kTable[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU;
}

} // namespace protocol
} // namespace fse