#pragma once

/// @file version.hpp
/// @brief Build and protocol version constants for the simulation engine.
///
/// Version numbers are part of the wire contract: see
/// `docs/protocol-spec.md`.

#include <cstdint>

namespace fse
{

/// Semantic version of the flight simulator itself.
namespace version
{

inline constexpr int kMajor = 0;
inline constexpr int kMinor = 1;
inline constexpr int kPatch = 0;

/// Human readable version, e.g. "0.1.0".
inline constexpr const char* kString = "0.1.0";

/// Name of the executable.
inline constexpr const char* kApplicationName = "flight-sim";

} // namespace version

/// Version of the telemetry wire protocol. Bumped independently of the
/// application version whenever the frame layout changes in a way that is not
/// backward compatible.
inline constexpr std::uint8_t kProtocolVersion = 1;

/// Default telemetry production rate. 100 Hz is fast enough to make latency
/// measurable while staying well inside a typical 100 Mbit/s link budget.
inline constexpr std::uint16_t kDefaultSampleRateHz = 100;

} // namespace fse