#!/usr/bin/env python3
"""
Generate byte-exact golden vectors for the telemetry wire protocol.

Usage:
    python tools/gen_golden_vectors.py

Output:
    protocol/golden_vectors.bin   -- 10 x 84 bytes, concatenated
    protocol/golden_vectors.json -- human-readable metadata for each frame

The script is the single source of truth for the fixture. Nobody edits the
.bin file by hand; if the frame layout changes, regenerate both files and
commit them together. The C++ and C# tests compare against the .bin file
byte for byte, so a change that is not reflected in both sides of the contract
breaks CI on one of them.

Python's struct module and zlib.crc32 are used deliberately: they are a
different implementation from the C++ one, so agreement between the two is
evidence that the format is correct rather than merely consistent.
"""

import json
import os
import struct
import zlib

MAGIC = 0xA55A
VERSION = 1
FRAME_SIZE = 84
CRC_COVERAGE = 80

# FlightPhase values (must match fse::FlightPhase in flight_phase.hpp)
PREFLIGHT = 0
TAXI = 1
TAKEOFF_ROLL = 2
CLIMB = 3
CRUISE = 4
DESCENT = 5
APPROACH = 6
LANDING = 7
GO_AROUND = 8
FAILURE = 9

# Status flags
NONE = 0
AIR_DATA_DEGRADED = 1 << 0
ENGINE_DEGRADED = 1 << 1
ATTITUDE_NOT_ALIGNED = 1 << 2
BARO_ALT_UNRELIABLE = 1 << 3


def encode_frame(f):
    """Encode a frame dict into 84 bytes, little-endian, with CRC."""
    buf = bytearray(FRAME_SIZE)

    struct.pack_into("<H", buf, 0, f["magic"])
    buf[2] = f["version"]
    buf[3] = f["aircraft_id"]
    struct.pack_into("<I", buf, 4, f["sequence"])
    struct.pack_into("<Q", buf, 8, f["timestamp_ns"])
    buf[16] = f["phase"]
    struct.pack_into("<H", buf, 17, f["status_flags"])
    buf[19] = f["reserved"]

    struct.pack_into("<f", buf, 20, f["airspeed_mps"])
    struct.pack_into("<f", buf, 24, f["altitude_m"])
    struct.pack_into("<f", buf, 28, f["vertical_speed_mps"])
    struct.pack_into("<f", buf, 32, f["heading_deg"])
    struct.pack_into("<f", buf, 36, f["pitch_deg"])
    struct.pack_into("<f", buf, 40, f["roll_deg"])

    for i in range(4):
        struct.pack_into("<f", buf, 44 + i * 4, f["engine_temp_c"][i])

    struct.pack_into("<f", buf, 60, f["fuel_press_kpa"])
    struct.pack_into("<f", buf, 64, f["oil_press_kpa"])

    struct.pack_into("<f", buf, 68, f["accel_x_mps2"])
    struct.pack_into("<f", buf, 72, f["accel_y_mps2"])
    struct.pack_into("<f", buf, 76, f["accel_z_mps2"])

    # CRC over bytes [0, 80)
    crc = zlib.crc32(bytes(buf[:CRC_COVERAGE])) & 0xFFFFFFFF
    struct.pack_into("<I", buf, 80, crc)
    f["crc32"] = crc

    return bytes(buf)


def make_frame(**kwargs):
    """Build a frame dict with sensible defaults."""
    defaults = {
        "magic": MAGIC,
        "version": VERSION,
        "aircraft_id": 1,
        "sequence": 0,
        "timestamp_ns": 0,
        "phase": CRUISE,
        "status_flags": NONE,
        "reserved": 0,
        "airspeed_mps": 105.0,
        "altitude_m": 900.0,
        "vertical_speed_mps": 0.0,
        "heading_deg": 90.0,
        "pitch_deg": 0.0,
        "roll_deg": 0.0,
        "engine_temp_c": [600.0, 605.0, 610.0, 615.0],
        "fuel_press_kpa": 42.0,
        "oil_press_kpa": 38.0,
        "accel_x_mps2": 0.0,
        "accel_y_mps2": 9.81,
        "accel_z_mps2": 0.0,
    }
    defaults.update(kwargs)
    return defaults


# Ten representative frames.
frames = [
    make_frame(
        sequence=1000,
        timestamp_ns=10_000_000_000,
        phase=CRUISE,
        airspeed_mps=105.0,
        altitude_m=900.0,
        vertical_speed_mps=0.0,
        heading_deg=90.0,
        pitch_deg=2.5,
        roll_deg=0.0,
        engine_temp_c=[620.0, 625.0, 630.0, 635.0],
        fuel_press_kpa=42.0,
        oil_press_kpa=38.0,
        accel_x_mps2=0.0,
        accel_y_mps2=9.81,
        accel_z_mps2=0.0,
    ),
    make_frame(
        sequence=0,
        timestamp_ns=0,
        phase=PREFLIGHT,
        airspeed_mps=0.0,
        altitude_m=0.0,
        vertical_speed_mps=0.0,
        heading_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        engine_temp_c=[25.0, 25.0, 25.0, 25.0],
        fuel_press_kpa=0.0,
        oil_press_kpa=0.0,
        accel_x_mps2=0.0,
        accel_y_mps2=9.81,
        accel_z_mps2=0.0,
        status_flags=ATTITUDE_NOT_ALIGNED,
    ),
    make_frame(
        sequence=500,
        timestamp_ns=5_000_000_000,
        phase=TAKEOFF_ROLL,
        airspeed_mps=45.0,
        altitude_m=0.0,
        vertical_speed_mps=0.0,
        heading_deg=90.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        engine_temp_c=[400.0, 405.0, 410.0, 415.0],
        fuel_press_kpa=40.0,
        oil_press_kpa=36.0,
        accel_x_mps2=2.5,
        accel_y_mps2=9.81,
        accel_z_mps2=0.0,
        status_flags=NONE,
    ),
    make_frame(
        sequence=510,
        timestamp_ns=5_100_000_000,
        phase=CLIMB,
        airspeed_mps=62.0,
        altitude_m=200.0,
        vertical_speed_mps=8.0,
        heading_deg=90.0,
        pitch_deg=8.0,
        roll_deg=0.0,
        engine_temp_c=[500.0, 505.0, 510.0, 515.0],
        fuel_press_kpa=41.0,
        oil_press_kpa=37.0,
        accel_x_mps2=0.5,
        accel_y_mps2=10.2,
        accel_z_mps2=0.0,
    ),
    make_frame(
        sequence=700,
        timestamp_ns=7_000_000_000,
        phase=DESCENT,
        airspeed_mps=95.0,
        altitude_m=600.0,
        vertical_speed_mps=-5.0,
        heading_deg=270.0,
        pitch_deg=-3.0,
        roll_deg=0.0,
        engine_temp_c=[550.0, 555.0, 560.0, 565.0],
        fuel_press_kpa=38.0,
        oil_press_kpa=34.0,
        accel_x_mps2=-0.3,
        accel_y_mps2=9.5,
        accel_z_mps2=0.0,
    ),
    make_frame(
        sequence=800,
        timestamp_ns=8_000_000_000,
        phase=APPROACH,
        airspeed_mps=62.0,
        altitude_m=450.0,
        vertical_speed_mps=-2.0,
        heading_deg=270.0,
        pitch_deg=-2.0,
        roll_deg=0.0,
        engine_temp_c=[520.0, 525.0, 530.0, 535.0],
        fuel_press_kpa=36.0,
        oil_press_kpa=32.0,
        accel_x_mps2=-0.1,
        accel_y_mps2=9.6,
        accel_z_mps2=0.0,
        status_flags=NONE,
    ),
    make_frame(
        sequence=850,
        timestamp_ns=8_500_000_000,
        phase=LANDING,
        airspeed_mps=47.0,
        altitude_m=0.0,
        vertical_speed_mps=-1.0,
        heading_deg=270.0,
        pitch_deg=-1.0,
        roll_deg=0.0,
        engine_temp_c=[510.0, 515.0, 520.0, 525.0],
        fuel_press_kpa=35.0,
        oil_press_kpa=31.0,
        accel_x_mps2=-0.5,
        accel_y_mps2=10.0,
        accel_z_mps2=0.0,
    ),
    make_frame(
        sequence=900,
        timestamp_ns=9_000_000_000,
        phase=GO_AROUND,
        airspeed_mps=62.0,
        altitude_m=50.0,
        vertical_speed_mps=5.0,
        heading_deg=90.0,
        pitch_deg=8.0,
        roll_deg=0.0,
        engine_temp_c=[530.0, 535.0, 540.0, 545.0],
        fuel_press_kpa=40.0,
        oil_press_kpa=36.0,
        accel_x_mps2=0.5,
        accel_y_mps2=10.5,
        accel_z_mps2=0.0,
    ),
    make_frame(
        sequence=1000,
        timestamp_ns=10_000_000_000,
        phase=FAILURE,
        airspeed_mps=100.0,
        altitude_m=500.0,
        vertical_speed_mps=0.0,
        heading_deg=90.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        engine_temp_c=[900.0, 910.0, 920.0, 930.0],
        fuel_press_kpa=10.0,
        oil_press_kpa=5.0,
        accel_x_mps2=0.0,
        accel_y_mps2=9.81,
        accel_z_mps2=0.0,
        status_flags=ENGINE_DEGRADED,
    ),
    make_frame(
        sequence=0xFFFFFFFF,
        timestamp_ns=0xFFFFFFFFFFFFFFFF,
        phase=CRUISE,
        airspeed_mps=300.0,
        altitude_m=15000.0,
        vertical_speed_mps=12.0,
        heading_deg=359.9,
        pitch_deg=25.0,
        roll_deg=55.0,
        engine_temp_c=[800.0, 800.0, 800.0, 800.0],
        fuel_press_kpa=120.0,
        oil_press_kpa=500.0,
        accel_x_mps2=5.0,
        accel_y_mps2=20.0,
        accel_z_mps2=5.0,
        status_flags=AIR_DATA_DEGRADED | BARO_ALT_UNRELIABLE,
    ),
]

# Corrupt the last frame's CRC to create a known-bad frame.
frames[-1]["_corrupt_crc"] = True

# Encode all frames.
encoded = []
metadata = []
for i, f in enumerate(frames):
    corrupt = f.pop("_corrupt_crc", False)
    raw = encode_frame(f)
    if corrupt:
        # Flip a bit in the payload (airspeed field, offset 20).
        raw = bytearray(raw)
        raw[20] ^= 0x01
        # Recompute CRC over the corrupted payload so the CRC field matches
        # the corrupted data — wait, no. For a CRC error test, the CRC should
        # NOT match. So we leave the CRC as-is (computed from uncorrupted data)
        # and corrupt the payload.
        raw = bytes(raw)
    encoded.append(raw)

    # Metadata for the JSON file.
    meta = {k: v for k, v in f.items() if k != "_corrupt_crc"}
    meta["index"] = i
    meta["crc_ok"] = not corrupt
    metadata.append(meta)

# Write binary file.
bin_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "protocol", "golden_vectors.bin")
os.makedirs(os.path.dirname(bin_path), exist_ok=True)
with open(bin_path, "wb") as fh:
    for raw in encoded:
        fh.write(raw)

# Write JSON metadata.
json_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "protocol", "golden_vectors.json")
with open(json_path, "w") as fh:
    json.dump({"frame_size": FRAME_SIZE, "count": len(encoded), "frames": metadata},
              fh, indent=2)

print(f"Generated {len(encoded)} frames ({len(encoded) * FRAME_SIZE} bytes) -> {bin_path}")
print(f"Metadata -> {json_path}")