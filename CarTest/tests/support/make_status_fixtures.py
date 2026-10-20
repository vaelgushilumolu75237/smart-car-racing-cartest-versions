"""Create target-matched synthetic IDLE status uploads without building firmware."""

import argparse
import json
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import runtime_log_export as exporter
import run_status_reader as reader


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    plan = reader.build_plan(args.elf)
    layout = exporter.inspect_elf(args.elf)
    scalar = bytearray(plan["scalar_read"]["size"])
    values = {
        "car_runtime_log_enabled": 1,
        "car_button_start_event_count": 2,
        "car_main_loop_heartbeat_ms": 250,
    }
    for name, value in values.items():
        symbol = plan["scalar_symbols"][name]
        struct.pack_into("<I", scalar,
                         symbol["address_u32"] - plan["scalar_read"]["address_u32"], value)
    header = bytearray(64)
    header[:4] = b"RLOG"
    struct.pack_into("<5H", header, 4, plan["log_version"], 64, 64,
                     exporter.CAPACITY, exporter.PERIOD_MS)
    struct.pack_into("<2B", header, 14, exporter.STATE_IDLE, 0)
    struct.pack_into("<7I", header, 16, 0, 0, 0, 0, 0, 0, 0)
    header[44:60] = layout.build_id
    struct.pack_into("<I", header, 40, zlib.crc32(header) & 0xFFFFFFFF)
    (out / "scalar-region.bin").write_bytes(scalar)
    (out / "runtime-log-header.bin").write_bytes(header)
    bad_crc_header = bytearray(header)
    bad_crc_header[40] ^= 1
    (out / "runtime-log-header-idle-bad-crc.bin").write_bytes(bad_crc_header)
    (out / "flash-rom-descriptor.bin").write_bytes(layout.descriptor_bytes)
    frozen_header = bytearray(64)
    frozen_header[:4] = b"RLOG"
    struct.pack_into("<5H", frozen_header, 4, plan["log_version"], 64, 64,
                     exporter.CAPACITY, exporter.PERIOD_MS)
    struct.pack_into("<2B", frozen_header, 14, exporter.STATE_FROZEN, 0)
    struct.pack_into("<7I", frozen_header, 16, 12, 55, 1000, 1011, 0, 0, 0)
    frozen_header[44:60] = layout.build_id
    struct.pack_into("<I", frozen_header, 40, zlib.crc32(frozen_header) & 0xFFFFFFFF)
    (out / "runtime-log-header-frozen.bin").write_bytes(frozen_header)

    diagnostic_path = None
    frozen_diagnostic_path = None
    diagnostic_plan = plan["runtime_diagnostic_snapshot"]
    if diagnostic_plan["available"]:
        def make_diagnostic(state, count, session_id, start_events, start_token):
            raw = bytearray(reader.DIAGNOSTIC_SIZE)
            struct.pack_into("<IHHII", raw, 0, reader.DIAGNOSTIC_MAGIC,
                             reader.DIAGNOSTIC_VERSION, reader.DIAGNOSTIC_SIZE,
                             2, 0)
            # Deliberately allow logger calls to be fewer than records: the
            # call counter includes IDLE calls and can observe multiple events.
            struct.pack_into("<8IhhHHII", raw, 16, start_events, 0, 1,
                             start_token, state, count, 123456, 0,
                             0, 0, 1530, 1, session_id, 0)
            struct.pack_into("<I", raw, 12, zlib.crc32(raw) & 0xFFFFFFFF)
            return bytes(raw)

        diagnostic_path = out / "runtime-diagnostic.bin"
        diagnostic_path.write_bytes(make_diagnostic(
            exporter.STATE_IDLE, 0, 0, 0, 0))
        frozen_diagnostic_path = out / "runtime-diagnostic-frozen.bin"
        frozen_diagnostic_path.write_bytes(make_diagnostic(
            exporter.STATE_FROZEN, 12, 55, 2, 7))

    speed_pi_idle_path = None
    speed_pi_active_path = None
    speed_pi_frozen_path = None
    speed_pi_plan = plan["speed_pi_diagnostic_snapshot"]
    if speed_pi_plan["available"]:
        def make_speed_pi_diagnostic(path, token, flags, pi_type):
            raw = bytearray(reader.SPEED_PI_DIAGNOSTIC_SIZE)
            struct.pack_into("<IHHII", raw, 0,
                             reader.SPEED_PI_DIAGNOSTIC_MAGIC,
                             reader.SPEED_PI_DIAGNOSTIC_VERSION,
                             reader.SPEED_PI_DIAGNOSTIC_SIZE, 2, 0)
            struct.pack_into("<4I", raw, 16, token, 1000, 1200, 1190)
            struct.pack_into("<4i", raw, 32, 2400, 2300, 2200, 2100)
            struct.pack_into("<6H", raw, 48, 500, 490, 700, 680, 680, 660)
            struct.pack_into("<6I", raw, 60, 3, 0, 100, 50, flags, pi_type)
            struct.pack_into("<11I", raw, 84, *([0] * 11))
            crc_data = bytearray(raw)
            crc_data[12:16] = bytes(4)
            struct.pack_into("<I", raw, 12, zlib.crc32(crc_data) & 0xFFFFFFFF)
            path.write_bytes(raw)

        speed_pi_idle_path = out / "speed-pi-diagnostic-idle.bin"
        make_speed_pi_diagnostic(speed_pi_idle_path, 0, 0, 0)
        active_flags = (
            reader.SPEED_PI_DIAGNOSTIC_FLAG_ENABLED |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_ARMED |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_ACTIVE |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID)
        speed_pi_active_path = out / "speed-pi-diagnostic-active.bin"
        make_speed_pi_diagnostic(speed_pi_active_path, 7, active_flags, 2)
        frozen_flags = (
            reader.SPEED_PI_DIAGNOSTIC_FLAG_ENABLED |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_ARMED |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID |
            reader.SPEED_PI_DIAGNOSTIC_FLAG_FROZEN)
        speed_pi_frozen_path = out / "speed-pi-diagnostic-frozen.bin"
        make_speed_pi_diagnostic(speed_pi_frozen_path, 7, frozen_flags, 2)
    print(json.dumps({
        "build_id": layout.build_id.hex(),
        "log_version": plan["log_version"],
        "descriptor_address": plan["descriptor_read"]["address"],
        "scalar_address": plan["scalar_read"]["address"],
        "scalar_size": plan["scalar_read"]["size"],
        "header_address": plan["header_read"]["address"],
        "diagnostic_available": bool(diagnostic_plan["available"]),
        "diagnostic_address": diagnostic_plan["address"],
        "speed_pi_diagnostic_available": bool(speed_pi_plan["available"]),
        "speed_pi_diagnostic_address": speed_pi_plan["address"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
