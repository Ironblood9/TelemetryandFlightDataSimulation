// ---------------------------------------------------------------------------
// Frame encoder.
//
// Converts a SensorSample into a TelemetryFrame. This is the single place
// where the in-memory representation and the wire representation meet, so
// every field mapping is explicit and tested here rather than scattered
// across the sender and the receiver.
//
// The encoder does not set the CRC: encode() computes it from the payload.
// It does not touch the timestamp or the sequence either; those belong to the
// orchestrator, which is the only thing that knows the sample's position in
// the stream.
// ---------------------------------------------------------------------------

#pragma once

#include "fse/sensor.hpp"
#include "protocol/telemetry_frame.hpp"

namespace fse
{
namespace protocol
{

/// Build a TelemetryFrame from a SensorSample.
///
/// @param sample       the sensor output for one step
/// @param aircraft_id  slot number of the simulated aircraft (1..255)
TelemetryFrame make_frame(const SensorSample& sample, std::uint8_t aircraft_id = 1U) noexcept;

} // namespace protocol
} // namespace fse