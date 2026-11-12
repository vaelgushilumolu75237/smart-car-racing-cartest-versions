"""Verify additive slope diagnostic bits in the unchanged 64-byte log ABI."""

from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
EXPORTER_PATH = PROJECT_ROOT / "tools" / "runtime_log_export.py"
SPEC = importlib.util.spec_from_file_location("runtime_log_export", EXPORTER_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("could not load runtime_log_export.py")
EXPORTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EXPORTER
SPEC.loader.exec_module(EXPORTER)


def make_record(vision_flags: int) -> bytes:
    record = bytearray(64)
    struct.pack_into("<II", record, 0, 1234, 7)
    struct.pack_into("<6B", record, 58, 2, 0, 77, vision_flags, 0, 0)
    return bytes(record)


def main() -> int:
    required_columns = {
        "vision_latest_slope_hint",
        "vision_slope_assist_commanded",
    }
    missing = required_columns.difference(EXPORTER.CSV_FIELDS)
    if missing:
        raise AssertionError("missing slope CSV columns: {}".format(sorted(missing)))

    legacy = EXPORTER._parse_record(make_record(0x0F), 7, 1000, 0)
    if legacy["vision_flags"] != 0x0F:
        raise AssertionError("legacy flags were not preserved")
    if legacy["vision_latest_slope_hint"] != 0:
        raise AssertionError("legacy record unexpectedly reports a slope hint")
    if legacy["vision_slope_assist_commanded"] != 0:
        raise AssertionError("legacy record unexpectedly reports slope assist")

    slope = EXPORTER._parse_record(make_record(0xCF), 7, 1000, 1)
    if slope["vision_flags"] != 0xCF:
        raise AssertionError("new diagnostic bits were not preserved")
    if slope["vision_latest_slope_hint"] != 1:
        raise AssertionError("latest slope hint bit was not decoded")
    if slope["vision_slope_assist_commanded"] != 1:
        raise AssertionError("commanded assist bit was not decoded")

    print("RUNTIME_LOG_EXPORT_FLAGS_PASS record_bytes=64 legacy=0x0f slope=0xcf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
