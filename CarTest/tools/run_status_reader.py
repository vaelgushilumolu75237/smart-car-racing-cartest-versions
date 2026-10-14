#!/usr/bin/env python3
"""Plan and decode a bounded, read-only CarTest live-status snapshot.

This is diagnostic telemetry, not a runtime-log export. It reads eight
32-bit firmware symbols and the 64-byte runtime-log header only; it never
reads the runtime-log record area.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
import zlib
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import runtime_log_export as exporter


SCALAR_SYMBOLS = (
    "car_runtime_log_enabled",
    "car_button_start_event_count",
    "car_manual_run_request",
    "car_mode",
    "car_fault",
    "car_main_loop_heartbeat_ms",
    "car_iwdg_reset_seen",
    "car_init_rcc_rsr_reset_snapshot",
)
SCALAR_SIZE = 4
MAX_SCALAR_READ_SIZE = 4096
D1_SRAM_START = 0x24000000
D1_SRAM_END = 0x24080000
RCC_RSR_IWDG1RSTF_MASK = 0x04000000
STATE_NAMES = {
    exporter.STATE_IDLE: "idle",
    exporter.STATE_ACTIVE: "active",
    exporter.STATE_FROZEN: "frozen",
}


class StatusReadError(ValueError):
    """The selected ELF or the bounded status snapshot is invalid."""


def _fail(message: str) -> None:
    raise StatusReadError(message)


def _hex(address: int) -> str:
    return "0x{:08X}".format(address)


def build_plan(elf_path: str | Path) -> Dict[str, Any]:
    """Derive all addresses from the selected ELF and validate every scalar."""
    layout = exporter.inspect_elf(elf_path)
    _blob, sections, symbols = exporter._parse_elf(layout.path)

    planned_symbols: Dict[str, Dict[str, Any]] = {}
    selected_section = None
    for name in SCALAR_SYMBOLS:
        symbol = exporter._require_symbol(symbols, name)
        section = exporter._section_for_symbol(sections, symbol, ".bss")
        if symbol.size != SCALAR_SIZE:
            _fail("ELF symbol '{}' must be exactly 4 bytes (found {})".format(name, symbol.size))
        if section.kind != 8:  # SHT_NOBITS, RAM .bss
            _fail("ELF symbol '{}' must be backed by the .bss RAM section".format(name))
        if not (section.flags & 0x2 and section.flags & 0x1):  # ALLOC | WRITE
            _fail("ELF symbol '{}' is not in allocated writable RAM".format(name))
        if not (D1_SRAM_START <= symbol.value and
                symbol.value + symbol.size <= D1_SRAM_END):
            _fail("ELF symbol '{}' is outside STM32H743 D1 SRAM".format(name))
        if selected_section is None:
            selected_section = section
        elif section.index != selected_section.index:
            _fail("Status symbols must share one .bss section for a bounded read")
        planned_symbols[name] = {
            "address": _hex(symbol.value),
            "address_u32": symbol.value,
            "size": symbol.size,
            "section": section.name,
            "interpretation": "uint32_little_endian",
        }

    symbol_start = min(item["address_u32"] for item in planned_symbols.values())
    symbol_end = max(item["address_u32"] + item["size"]
                     for item in planned_symbols.values())
    span = symbol_end - symbol_start
    if span <= 0 or span > MAX_SCALAR_READ_SIZE:
        _fail("Status symbols span {} bytes; the maximum permitted read is {} bytes".format(
            span, MAX_SCALAR_READ_SIZE))
    if (selected_section is None or symbol_start < selected_section.address or
            symbol_end > selected_section.address + selected_section.size):
        _fail("Combined status read is not fully contained in the selected .bss section")
    if not (D1_SRAM_START <= symbol_start and symbol_end <= D1_SRAM_END):
        _fail("Combined status read is outside STM32H743 D1 SRAM")

    header_end = layout.storage_address + exporter.HEADER_SIZE
    if not (D1_SRAM_START <= layout.storage_address and header_end <= D1_SRAM_END):
        _fail("Runtime-log header range is outside STM32H743 D1 SRAM")

    return {
        "elf_path": str(layout.path),
        "elf_sha256": layout.sha256,
        "machine": "ARM",
        "endianness": "little",
        "build_id": layout.build_id.hex(),
        "scalar_symbols": planned_symbols,
        "scalar_read": {
            "address": _hex(symbol_start),
            "address_u32": symbol_start,
            "size": span,
            "section": selected_section.name,
            "maximum_size": MAX_SCALAR_READ_SIZE,
        },
        "header_read": {
            "address": _hex(layout.storage_address),
            "address_u32": layout.storage_address,
            "size": exporter.HEADER_SIZE,
        },
        "record_area_read": False,
    }


def _decode_header(raw: bytes, expected_build_id: str) -> Dict[str, Any]:
    if len(raw) != exporter.HEADER_SIZE:
        _fail("Runtime-log header read must be exactly 64 bytes (found {})".format(len(raw)))
    if raw[:4] != exporter.HEADER_MAGIC:
        _fail("Runtime-log header magic is invalid")

    version, header_size, record_size, capacity, period = struct.unpack_from("<5H", raw, 4)
    state, flags = struct.unpack_from("<2B", raw, 14)
    count, session_id, start_ms, stop_ms, dropped_count, records_crc, header_crc = struct.unpack_from(
        "<7I", raw, 16)
    observed_build_id = raw[44:60].hex()
    zeroed = bytearray(raw)
    zeroed[40:44] = b"\0\0\0\0"
    computed_crc = zlib.crc32(zeroed) & 0xFFFFFFFF
    crc_valid = computed_crc == header_crc
    build_id_matches = observed_build_id.lower() == expected_build_id.lower()

    expected_schema = (exporter.LOG_VERSION, exporter.HEADER_SIZE, exporter.RECORD_SIZE,
                       exporter.CAPACITY, exporter.PERIOD_MS)
    schema_valid = (version, header_size, record_size, capacity, period) == expected_schema
    if not schema_valid:
        _fail("Runtime-log header schema does not match this ELF/exporter")

    state_name = STATE_NAMES.get(state, "unknown")
    warnings = []
    if not build_id_matches:
        warnings.append("Observed header build ID does not match the selected ELF")
    if not crc_valid:
        if state == exporter.STATE_ACTIVE:
            crc_status = "uncertain_live_mutation"
            warnings.append("Header CRC differs; an ACTIVE header may have changed during this live read")
        else:
            crc_status = "invalid"
            warnings.append("Runtime-log header CRC32 is invalid")
    else:
        crc_status = "valid"
    if state not in STATE_NAMES:
        warnings.append("Observed runtime-log state value is outside the defined enum")
    if count > capacity:
        warnings.append("Observed runtime-log count exceeds the header capacity")

    return {
        "magic": "RLOG",
        "version": version,
        "header_size": header_size,
        "record_size": record_size,
        "capacity": capacity,
        "period_ms": period,
        "state": state,
        "state_name": state_name,
        "flags": flags,
        "count": count,
        "session_id": session_id,
        "start_ms": start_ms,
        "stop_ms": stop_ms,
        "dropped_count": dropped_count,
        "records_crc32": "0x{:08X}".format(records_crc),
        "records_crc_checked": False,
        "build_id": observed_build_id,
        "build_id_matches_elf": build_id_matches,
        "header_crc32_read": "0x{:08X}".format(header_crc),
        "header_crc32_computed": "0x{:08X}".format(computed_crc),
        "header_crc_status": crc_status,
        "schema_valid": schema_valid,
        "record_payload_read": False,
        "warnings": warnings,
    }


def decode_snapshot(elf_path: str | Path, scalar_path: str | Path,
                    header_path: str | Path) -> Dict[str, Any]:
    plan = build_plan(elf_path)
    scalar_raw = Path(scalar_path).read_bytes()
    header_raw = Path(header_path).read_bytes()
    scalar_read = plan["scalar_read"]
    if len(scalar_raw) != scalar_read["size"]:
        _fail("Scalar read length mismatch: expected {}, got {}".format(
            scalar_read["size"], len(scalar_raw)))

    scalars: Dict[str, Any] = {}
    read_start = scalar_read["address_u32"]
    for name, symbol in plan["scalar_symbols"].items():
        offset = symbol["address_u32"] - read_start
        value = struct.unpack_from("<I", scalar_raw, offset)[0]
        scalars[name] = value
    scalars["car_mode_name"] = exporter.MODE_NAMES.get(scalars["car_mode"], "unknown")
    scalars["car_fault_name"] = exporter.FAULT_NAMES.get(scalars["car_fault"], "unknown")
    scalars["iwdg_reset_flag_in_rcc_rsr"] = bool(
        scalars["car_init_rcc_rsr_reset_snapshot"] & RCC_RSR_IWDG1RSTF_MASK)
    scalars["rcc_rsr_iwdg1rstf_mask"] = _hex(RCC_RSR_IWDG1RSTF_MASK)

    header = _decode_header(header_raw, plan["build_id"])
    status = "success" if not header["warnings"] else "read_with_warnings"
    return {
        "status": status,
        "snapshot_kind": "live_diagnostic_only",
        "diagnostic_is_run_record": False,
        "reads_are_atomic_together": False,
        "consistency_note": (
            "The scalar region and 64-byte header were read in two separate SWD uploads. "
            "Live values may change between reads; this snapshot is not a runtime-log record. "
            "Current CarTest firmware source does not enable D-cache. If a later firmware "
            "build enables D-cache, direct SWD scalar reads may be stale unless those values "
            "are published coherently."
        ),
        "elf": {
            "path": plan["elf_path"],
            "sha256": plan["elf_sha256"],
            "build_id": plan["build_id"],
            "endianness": plan["endianness"],
        },
        "reads": {
            "scalar_region": plan["scalar_read"],
            "runtime_log_header": plan["header_read"],
            "runtime_log_record_area_read": False,
        },
        "scalars": scalars,
        "runtime_log_header": header,
        "raw_files": {
            "scalar_region": {
                "path": str(Path(scalar_path).resolve()),
                "bytes": len(scalar_raw),
                "sha256": hashlib.sha256(scalar_raw).hexdigest(),
            },
            "runtime_log_header": {
                "path": str(Path(header_path).resolve()),
                "bytes": len(header_raw),
                "sha256": hashlib.sha256(header_raw).hexdigest(),
            },
        },
    }


def _emit_json(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    plan_parser = subparsers.add_parser("plan", help="validate ELF symbols and print bounded read plan")
    plan_parser.add_argument("--elf", required=True)
    decode_parser = subparsers.add_parser("decode", help="decode raw scalar/header bytes as diagnostic JSON")
    decode_parser.add_argument("--elf", required=True)
    decode_parser.add_argument("--scalars", required=True)
    decode_parser.add_argument("--header", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "plan":
            _emit_json(build_plan(args.elf))
        else:
            _emit_json(decode_snapshot(args.elf, args.scalars, args.header))
        return 0
    except (StatusReadError, exporter.ExportError, OSError, struct.error) as exc:
        print("run-status-reader: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
