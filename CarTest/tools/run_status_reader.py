#!/usr/bin/env python3
"""Plan and decode bounded, read-only CarTest runtime diagnostics.

New ELFs expose one CRC-protected 64-byte runtime diagnostic snapshot. The
reader brackets it with two 64-byte runtime-log header reads so ACTIVE count
progress can be distinguished from stale or inconsistent state. Older ELFs
without the diagnostic symbol retain the legacy scalar/header path. No log
record payload is read here.
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


REQUIRED_SCALAR_SYMBOLS = (
    "car_runtime_log_enabled",
    "car_button_start_event_count",
    "car_manual_run_request",
    "car_mode",
    "car_fault",
    "car_main_loop_heartbeat_ms",
    "car_iwdg_reset_seen",
    "car_init_rcc_rsr_reset_snapshot",
)
HOLD_SNAPSHOT_SYMBOLS = (
    "car_vision_hold_snapshot_entry",
    "car_vision_hold_snapshot_error_norm",
    "car_vision_hold_snapshot_sequence",
    "car_vision_hold_snapshot_source_rx_ms",
    "car_vision_hold_snapshot_entered_ms",
    "car_vision_hold_snapshot_target_us",
    "car_vision_hold_snapshot_applied_us",
    "car_vision_hold_snapshot_left_permille",
    "car_vision_hold_snapshot_right_permille",
)
SCALAR_SYMBOLS = REQUIRED_SCALAR_SYMBOLS + HOLD_SNAPSHOT_SYMBOLS
SIGNED_SCALAR_SYMBOLS = {"car_vision_hold_snapshot_error_norm"}
SCALAR_SIZE = 4
MAX_SCALAR_READ_SIZE = 8192
D1_SRAM_START = 0x24000000
D1_SRAM_END = 0x24080000
RCC_RSR_IWDG1RSTF_MASK = 0x04000000
STATE_NAMES = {
    exporter.STATE_IDLE: "idle",
    exporter.STATE_ACTIVE: "active",
    exporter.STATE_FROZEN: "frozen",
}
DIAGNOSTIC_SYMBOL = "g_runtime_diag_snapshot"
DIAGNOSTIC_MAGIC = 0x47414944
DIAGNOSTIC_VERSION = 1
DIAGNOSTIC_SIZE = 64
DIAGNOSTIC_CRC_OFFSET = 12
DIAGNOSTIC_CRC_SIZE = 4
DIAGNOSTIC_FLAGS_LOG_ENABLED = 0x0001
DIAGNOSTIC_KNOWN_FLAGS = DIAGNOSTIC_FLAGS_LOG_ENABLED
SPEED_PI_DIAGNOSTIC_SYMBOL = "g_speed_pi_diag_snapshot"
SPEED_PI_DIAGNOSTIC_MAGIC = 0x47414950
SPEED_PI_DIAGNOSTIC_VERSION = 1
SPEED_PI_DIAGNOSTIC_SIZE = 128
SPEED_PI_DIAGNOSTIC_CRC_OFFSET = 12
SPEED_PI_DIAGNOSTIC_CRC_SIZE = 4
SPEED_PI_DIAGNOSTIC_FLAG_ENABLED = 1 << 0
SPEED_PI_DIAGNOSTIC_FLAG_ARMED = 1 << 1
SPEED_PI_DIAGNOSTIC_FLAG_ACTIVE = 1 << 2
SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID = 1 << 3
SPEED_PI_DIAGNOSTIC_FLAG_FROZEN = 1 << 4
SPEED_PI_DIAGNOSTIC_FLAG_HOLD_ACTIVE = 1 << 5
SPEED_PI_DIAGNOSTIC_KNOWN_FLAGS = (
    SPEED_PI_DIAGNOSTIC_FLAG_ENABLED |
    SPEED_PI_DIAGNOSTIC_FLAG_ARMED |
    SPEED_PI_DIAGNOSTIC_FLAG_ACTIVE |
    SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID |
    SPEED_PI_DIAGNOSTIC_FLAG_FROZEN |
    SPEED_PI_DIAGNOSTIC_FLAG_HOLD_ACTIVE
)
SPEED_PI_TYPE_NAMES = {0: "none", 1: "position", 2: "incremental"}
SPEED_PI_STATUS_NAMES = {
    0: "stopped", 1: "disabled", 2: "not_ready", 3: "running",
    4: "hold", 5: "invalid",
}
SPEED_PI_OUTPUT_CAP_PERMILLE = 700


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
        symbol = symbols.get(name)
        if symbol is None:
            if name in REQUIRED_SCALAR_SYMBOLS:
                _fail("ELF is missing required symbol '{}'".format(name))
            planned_symbols[name] = {
                "available": False,
                "address": None,
                "address_u32": None,
                "size": None,
                "section": None,
                "interpretation": "signed_int32_twos_complement" if name in SIGNED_SCALAR_SYMBOLS else "uint32_little_endian",
            }
            continue
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
            "available": True,
            "address": _hex(symbol.value),
            "address_u32": symbol.value,
            "size": symbol.size,
            "section": section.name,
            "interpretation": "signed_int32_twos_complement" if name in SIGNED_SCALAR_SYMBOLS else "uint32_little_endian",
        }

    present_symbols = [item for item in planned_symbols.values() if item["available"]]
    symbol_start = min(item["address_u32"] for item in present_symbols)
    symbol_end = max(item["address_u32"] + item["size"]
                     for item in present_symbols)
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

    diagnostic_symbol = symbols.get(DIAGNOSTIC_SYMBOL)
    if diagnostic_symbol is None:
        diagnostic_plan = {
            "available": False,
            "symbol": DIAGNOSTIC_SYMBOL,
            "address": None,
            "address_u32": None,
            "size": None,
            "section": None,
        }
    else:
        diagnostic_section = exporter._section_for_symbol(
            sections, diagnostic_symbol, ".runtime_diag")
        if diagnostic_symbol.size != DIAGNOSTIC_SIZE:
            _fail("ELF symbol '{}' must be exactly {} bytes (found {})".format(
                DIAGNOSTIC_SYMBOL, DIAGNOSTIC_SIZE, diagnostic_symbol.size))
        if diagnostic_section.kind != 8:  # SHT_NOBITS, NOLOAD/NOINIT RAM.
            _fail("ELF symbol '{}' must be backed by a NOLOAD .runtime_diag section".format(
                DIAGNOSTIC_SYMBOL))
        if not (diagnostic_section.flags & 0x2 and diagnostic_section.flags & 0x1):
            _fail("ELF symbol '{}' is not in allocated writable RAM".format(
                DIAGNOSTIC_SYMBOL))
        if diagnostic_section.alignment < 32 or diagnostic_symbol.value % 32:
            _fail("ELF symbol '{}' must be 32-byte aligned".format(DIAGNOSTIC_SYMBOL))
        if not (D1_SRAM_START <= diagnostic_symbol.value and
                diagnostic_symbol.value + DIAGNOSTIC_SIZE <= D1_SRAM_END):
            _fail("ELF symbol '{}' is outside STM32H743 D1 SRAM".format(
                DIAGNOSTIC_SYMBOL))
        diagnostic_plan = {
            "available": True,
            "symbol": DIAGNOSTIC_SYMBOL,
            "address": _hex(diagnostic_symbol.value),
            "address_u32": diagnostic_symbol.value,
            "size": DIAGNOSTIC_SIZE,
            "section": diagnostic_section.name,
            "section_alignment": diagnostic_section.alignment,
        }

    speed_pi_diagnostic_symbol = symbols.get(SPEED_PI_DIAGNOSTIC_SYMBOL)
    if speed_pi_diagnostic_symbol is None:
        speed_pi_diagnostic_plan = {
            "available": False,
            "symbol": SPEED_PI_DIAGNOSTIC_SYMBOL,
            "address": None,
            "address_u32": None,
            "size": None,
            "section": None,
        }
    else:
        speed_pi_section = exporter._section_for_symbol(
            sections, speed_pi_diagnostic_symbol, ".speed_pi_diag")
        if speed_pi_diagnostic_symbol.size != SPEED_PI_DIAGNOSTIC_SIZE:
            _fail("ELF symbol '{}' must be exactly {} bytes (found {})".format(
                SPEED_PI_DIAGNOSTIC_SYMBOL, SPEED_PI_DIAGNOSTIC_SIZE,
                speed_pi_diagnostic_symbol.size))
        if speed_pi_section.kind != 8:
            _fail("ELF symbol '{}' must be backed by a NOLOAD .speed_pi_diag section".format(
                SPEED_PI_DIAGNOSTIC_SYMBOL))
        if not (speed_pi_section.flags & 0x2 and speed_pi_section.flags & 0x1):
            _fail("ELF symbol '{}' is not in allocated writable RAM".format(
                SPEED_PI_DIAGNOSTIC_SYMBOL))
        if (speed_pi_section.alignment < 32 or
                speed_pi_diagnostic_symbol.value % 32):
            _fail("ELF symbol '{}' must be 32-byte aligned".format(
                SPEED_PI_DIAGNOSTIC_SYMBOL))
        if not (D1_SRAM_START <= speed_pi_diagnostic_symbol.value and
                speed_pi_diagnostic_symbol.value + SPEED_PI_DIAGNOSTIC_SIZE <= D1_SRAM_END):
            _fail("ELF symbol '{}' is outside STM32H743 D1 SRAM".format(
                SPEED_PI_DIAGNOSTIC_SYMBOL))
        speed_pi_diagnostic_plan = {
            "available": True,
            "symbol": SPEED_PI_DIAGNOSTIC_SYMBOL,
            "address": _hex(speed_pi_diagnostic_symbol.value),
            "address_u32": speed_pi_diagnostic_symbol.value,
            "size": SPEED_PI_DIAGNOSTIC_SIZE,
            "section": speed_pi_section.name,
            "section_alignment": speed_pi_section.alignment,
        }

    return {
        "elf_path": str(layout.path),
        "elf_sha256": layout.sha256,
        "machine": "ARM",
        "endianness": "little",
        "build_id": layout.build_id.hex(),
        "log_version": int(layout.descriptor["log_version"]),
        "hold_quantum_ms": exporter._hold_quantum_ms(int(layout.descriptor["log_version"])),
        "descriptor_read": {
            "address": "0x{:08X}".format(layout.descriptor_address),
            "address_u32": layout.descriptor_address,
            "size": exporter.HEADER_SIZE,
            "expected_sha256": hashlib.sha256(layout.descriptor_bytes).hexdigest(),
        },
        "required_scalar_symbols": list(REQUIRED_SCALAR_SYMBOLS),
        "optional_hold_snapshot_symbols": list(HOLD_SNAPSHOT_SYMBOLS),
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
        "runtime_diagnostic_snapshot": diagnostic_plan,
        "speed_pi_diagnostic_snapshot": speed_pi_diagnostic_plan,
        "record_area_read": False,
    }


def _decode_header(raw: bytes, expected_build_id: str,
                   expected_log_version: int) -> Dict[str, Any]:
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

    expected_schema = (expected_log_version, exporter.HEADER_SIZE, exporter.RECORD_SIZE,
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
        "hold_quantum_ms": exporter._hold_quantum_ms(version),
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
        "header_crc_valid": crc_valid,
        "schema_valid": schema_valid,
        "record_payload_read": False,
        "warnings": warnings,
    }


def decode_runtime_diagnostic(raw: bytes) -> Dict[str, Any]:
    """Validate and decode one 64-byte firmware-published diagnostic record."""
    if len(raw) != DIAGNOSTIC_SIZE:
        _fail("Runtime diagnostic snapshot must be exactly {} bytes (found {})".format(
            DIAGNOSTIC_SIZE, len(raw)))
    magic, version, size, generation, observed_crc = struct.unpack_from("<IHHII", raw, 0)
    if magic != DIAGNOSTIC_MAGIC:
        _fail("Runtime diagnostic snapshot magic is invalid")
    if version != DIAGNOSTIC_VERSION or size != DIAGNOSTIC_SIZE:
        _fail("Runtime diagnostic snapshot version/size is unsupported")
    if generation == 0 or generation & 1:
        _fail("Runtime diagnostic snapshot generation is zero or odd (writer active or uninitialized)")
    crc_data = bytearray(raw)
    crc_data[DIAGNOSTIC_CRC_OFFSET:DIAGNOSTIC_CRC_OFFSET + DIAGNOSTIC_CRC_SIZE] = bytes(4)
    computed_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
    if computed_crc != observed_crc:
        _fail("Runtime diagnostic snapshot CRC32 is invalid")

    (start_events, stop_events, log_update_calls, start_token,
     cpu_state, cpu_count, uptime_ms, reset_flags,
     left_output_permille, right_output_permille, servo_pulse_us,
     flags, header_session_id, reserved) = struct.unpack_from("<8IhhHHII", raw, 16)
    if cpu_state not in STATE_NAMES:
        _fail("Runtime diagnostic snapshot contains an unknown CPU log state")
    if flags & ~DIAGNOSTIC_KNOWN_FLAGS:
        _fail("Runtime diagnostic snapshot contains unsupported flag bits")
    if reserved != 0:
        _fail("Runtime diagnostic snapshot reserved word is not zero")
    if cpu_count > exporter.CAPACITY:
        _fail("Runtime diagnostic snapshot CPU log count exceeds capacity")
    return {
        "magic": "DIAG",
        "version": version,
        "size": size,
        "generation": generation,
        "crc32_read": "0x{:08X}".format(observed_crc),
        "crc32_computed": "0x{:08X}".format(computed_crc),
        "crc_valid": True,
        "start_events": start_events,
        "stop_events": stop_events,
        "log_update_calls": log_update_calls,
        "start_token": start_token,
        "cpu_header_state": cpu_state,
        "cpu_header_state_name": STATE_NAMES[cpu_state],
        "cpu_header_count": cpu_count,
        "uptime_ms": uptime_ms,
        "reset_flags_rcc_rsr": "0x{:08X}".format(reset_flags),
        "left_output_permille": left_output_permille,
        "right_output_permille": right_output_permille,
        "servo_pulse_us": servo_pulse_us,
        "log_enabled": bool(flags & DIAGNOSTIC_FLAGS_LOG_ENABLED),
        "flags": flags,
        "header_session_id": header_session_id,
        "reserved": reserved,
    }


def decode_speed_pi_diagnostic(raw: bytes) -> Dict[str, Any]:
    """Validate and decode one fixed 128-byte speed-PI session snapshot."""
    if len(raw) != SPEED_PI_DIAGNOSTIC_SIZE:
        _fail("Speed PI diagnostic snapshot must be exactly {} bytes (found {})".format(
            SPEED_PI_DIAGNOSTIC_SIZE, len(raw)))
    magic, version, size, generation, observed_crc = struct.unpack_from(
        "<IHHII", raw, 0)
    if magic != SPEED_PI_DIAGNOSTIC_MAGIC:
        _fail("Speed PI diagnostic snapshot magic is invalid")
    if version != SPEED_PI_DIAGNOSTIC_VERSION or size != SPEED_PI_DIAGNOSTIC_SIZE:
        _fail("Speed PI diagnostic snapshot version/size is unsupported")
    if generation == 0 or generation & 1:
        _fail("Speed PI diagnostic snapshot generation is zero or odd (writer active or uninitialized)")
    crc_data = bytearray(raw)
    crc_data[SPEED_PI_DIAGNOSTIC_CRC_OFFSET:
             SPEED_PI_DIAGNOSTIC_CRC_OFFSET + SPEED_PI_DIAGNOSTIC_CRC_SIZE] = bytes(4)
    computed_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
    if computed_crc != observed_crc:
        _fail("Speed PI diagnostic snapshot CRC32 is invalid")

    start_token, start_ms, snapshot_ms, sample_ms = struct.unpack_from(
        "<4I", raw, 16)
    (target_left, target_right, measured_left, measured_right) = struct.unpack_from(
        "<4i", raw, 32)
    (nominal_left, nominal_right, requested_left, requested_right,
     applied_left, applied_right) = struct.unpack_from("<6H", raw, 48)
    (status, termination_fault, saturation_left_ms, saturation_right_ms,
     flags, pi_type) = struct.unpack_from("<6I", raw, 60)
    reserved = struct.unpack_from("<11I", raw, 84)

    if flags & ~SPEED_PI_DIAGNOSTIC_KNOWN_FLAGS:
        _fail("Speed PI diagnostic snapshot contains unsupported flag bits")
    if (flags & SPEED_PI_DIAGNOSTIC_FLAG_ACTIVE and
            flags & SPEED_PI_DIAGNOSTIC_FLAG_FROZEN):
        _fail("Speed PI diagnostic snapshot cannot be both ACTIVE and FROZEN")
    if any(reserved):
        _fail("Speed PI diagnostic snapshot reserved words are not zero")
    if pi_type not in SPEED_PI_TYPE_NAMES:
        _fail("Speed PI diagnostic snapshot contains an unknown PI type")
    if any(value < 0 or value > 100_000_000 for value in
           (target_left, target_right, measured_left, measured_right)):
        _fail("Speed PI diagnostic snapshot contains a speed outside the supported range")
    if any(value > SPEED_PI_OUTPUT_CAP_PERMILLE for value in
           (nominal_left, nominal_right, requested_left, requested_right,
            applied_left, applied_right)):
        _fail("Speed PI diagnostic snapshot contains an output above the 700-permille cap")

    return {
        "magic": "PIAG",
        "version": version,
        "size": size,
        "generation": generation,
        "crc32_read": "0x{:08X}".format(observed_crc),
        "crc32_computed": "0x{:08X}".format(computed_crc),
        "crc_valid": True,
        "start_token": start_token,
        "association": "manual PE7 start token",
        "start_ms": start_ms,
        "snapshot_ms": snapshot_ms,
        "sample_ms": sample_ms,
        "target_left_millirps": target_left,
        "target_right_millirps": target_right,
        "measured_left_millirps": measured_left,
        "measured_right_millirps": measured_right,
        "nominal_ff_left_permille": nominal_left,
        "nominal_ff_right_permille": nominal_right,
        "requested_left_permille": requested_left,
        "requested_right_permille": requested_right,
        "applied_left_permille": applied_left,
        "applied_right_permille": applied_right,
        "status": status,
        "status_name": SPEED_PI_STATUS_NAMES.get(status, "unknown"),
        "termination_fault": termination_fault,
        "saturation_left_ms": saturation_left_ms,
        "saturation_right_ms": saturation_right_ms,
        "saturation_semantics": (
            "Cumulative sampled duration at the 700-permille upper cap; "
            "counted once per distinct valid sample_ms while armed, saturated, "
            "and outside vision hold. Zero-output intervals do not count."
        ),
        "flags": flags,
        "enabled": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_ENABLED),
        "armed": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_ARMED),
        "active": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_ACTIVE),
        "last_sample_valid": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID),
        "frozen": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_FROZEN),
        "hold_active": bool(flags & SPEED_PI_DIAGNOSTIC_FLAG_HOLD_ACTIVE),
        "pi_type": pi_type,
        "pi_type_name": SPEED_PI_TYPE_NAMES[pi_type],
        "reserved": list(reserved),
        "start_token_matches_runtime_diagnostic": None,
    }


def _classify_runtime_diagnostic(diag: Dict[str, Any],
                                 header_before: Dict[str, Any],
                                 header_after: Dict[str, Any]) -> Tuple[str, list[str], bool]:
    """Compare CPU state/count with surrounding headers, allowing live progress."""
    reasons: list[str] = []
    active_progress = False
    before_state = int(header_before["state"])
    after_state = int(header_after["state"])
    before_count = int(header_before["count"])
    after_count = int(header_after["count"])
    cpu_state = int(diag["cpu_header_state"])
    cpu_count = int(diag["cpu_header_count"])
    session_before = int(header_before["session_id"])
    session_after = int(header_after["session_id"])
    cpu_session = int(diag["header_session_id"])

    if not header_before["build_id_matches_elf"] or not header_after["build_id_matches_elf"]:
        reasons.append("Bracketed runtime-log header build ID does not match the selected ELF")
    if before_state not in STATE_NAMES or after_state not in STATE_NAMES:
        reasons.append("Bracketed runtime-log header contains an unknown state")
    if after_count < before_count:
        reasons.append("Runtime-log count moved backwards between header reads")
    if session_before != session_after and before_state != exporter.STATE_IDLE:
        reasons.append("Runtime-log session changed during the diagnostic bracket")

    if before_state == exporter.STATE_IDLE and after_state == exporter.STATE_IDLE:
        never_started = (
            cpu_state == exporter.STATE_IDLE and cpu_count == 0 and
            cpu_session == 0 and int(diag["start_token"]) == 0 and
            before_count == after_count == 0 and
            session_before == session_after == 0 and
            header_before["header_crc_valid"] and header_after["header_crc_valid"]
        )
        if never_started and not reasons:
            if int(diag["start_events"]) != 0:
                reasons.append("PE7 start events were seen, but the logger has not consumed a start token")
            if not diag["log_enabled"]:
                reasons.append("Runtime logger is disabled")
            return "neverstarted", reasons, active_progress
        if not header_before["header_crc_valid"] or not header_after["header_crc_valid"]:
            reasons.append("Stable IDLE runtime-log header CRC is invalid")
        reasons.append("IDLE headers disagree with the CPU diagnostic state/session")
        return "inconsistent", reasons, active_progress

    if before_state == exporter.STATE_ACTIVE and after_state == exporter.STATE_ACTIVE:
        active_progress = before_count != after_count or cpu_count != after_count
        active_headers_crc_valid = (header_before["header_crc_valid"] and
                                    header_after["header_crc_valid"])
        if not active_headers_crc_valid:
            reasons.append("ACTIVE runtime-log header CRC is invalid; retry the status snapshot")
        session_ok = (session_before != 0 and session_before == session_after == cpu_session)
        counts_sensible = (0 <= before_count <= exporter.CAPACITY and
                           0 <= cpu_count <= exporter.CAPACITY and
                           0 <= after_count <= exporter.CAPACITY)
        # Reads are ordered header -> diagnostic -> header, but can be slow
        # enough for any number of records to commit between them.
        counts_monotonic = before_count <= cpu_count <= after_count
        if (cpu_state == exporter.STATE_ACTIVE and session_ok and counts_sensible and
                counts_monotonic and not reasons):
            return "active", reasons, active_progress
        reasons.append("ACTIVE CPU snapshot does not match the bracketed session or monotonic counts")
        return "inconsistent", reasons, active_progress

    if before_state == exporter.STATE_IDLE and after_state in (
            exporter.STATE_ACTIVE, exporter.STATE_FROZEN):
        active_progress = True
        if after_state == exporter.STATE_ACTIVE and not (
                header_before["header_crc_valid"] and header_after["header_crc_valid"]):
            reasons.append("ACTIVE runtime-log header CRC is invalid; retry the status snapshot")
        session_ok = (session_after != 0 and session_after == cpu_session)
        if after_state == exporter.STATE_ACTIVE and cpu_state == exporter.STATE_ACTIVE:
            counts_monotonic = before_count == 0 and before_session == 0 and 0 <= cpu_count <= after_count
            if (session_ok and counts_monotonic and
                    after_count <= exporter.CAPACITY and not reasons):
                return "active", reasons, active_progress
        if after_state == exporter.STATE_FROZEN:
            reasons.append("IDLE-to-FROZEN transition crossed the diagnostic bracket; retry for a stable FROZEN snapshot")
            return "inconsistent", reasons, active_progress
        reasons.append("New session appeared during the diagnostic bracket without a matching CPU snapshot")
        return "inconsistent", reasons, active_progress

    if after_state == exporter.STATE_FROZEN and before_state in (
            exporter.STATE_ACTIVE, exporter.STATE_FROZEN):
        active_progress = (before_state == exporter.STATE_ACTIVE or before_count != after_count or
                           cpu_state == exporter.STATE_ACTIVE or cpu_count != after_count)
        session_ok = (session_before != 0 and session_before == session_after == cpu_session)
        transitioning = before_state == exporter.STATE_ACTIVE or cpu_state == exporter.STATE_ACTIVE
        if transitioning:
            reasons.append("ACTIVE-to-FROZEN transition crossed the diagnostic bracket; retry for a stable FROZEN snapshot")
            return "inconsistent", reasons, active_progress
        stable_frozen = (cpu_state == exporter.STATE_FROZEN and
                         cpu_count == after_count == before_count and
                         header_before["header_crc_valid"] and
                         header_after["header_crc_valid"])
        if stable_frozen and session_ok and not reasons:
            return "frozen", reasons, active_progress
        reasons.append("FROZEN header does not match CPU snapshot state/count/session")
        return "inconsistent", reasons, active_progress

    reasons.append("Unsupported CPU/runtime-log state transition in the diagnostic bracket")
    return "inconsistent", reasons, active_progress


def _classify_legacy_headers(header_before: Dict[str, Any],
                             header_after: Dict[str, Any]) -> Tuple[str, list[str]]:
    """Keep old ELF diagnostics usable while marking their weaker evidence."""
    before_state = int(header_before["state"])
    after_state = int(header_after["state"])
    before_count = int(header_before["count"])
    after_count = int(header_after["count"])
    before_session = int(header_before["session_id"])
    after_session = int(header_after["session_id"])
    warnings = ["Legacy ELF has no coherent runtime diagnostic snapshot; header-only classification used."]
    if not header_before["build_id_matches_elf"] or not header_after["build_id_matches_elf"]:
        return "legacy_inconsistent", warnings + ["Bracketed runtime-log header build ID does not match the selected ELF"]
    if after_count < before_count:
        return "legacy_inconsistent", warnings + ["Runtime-log count moved backwards between header reads"]
    if before_state == exporter.STATE_IDLE and after_state == exporter.STATE_IDLE:
        if (before_count == after_count == 0 and before_session == after_session == 0 and
                header_before["header_crc_valid"] and header_after["header_crc_valid"]):
            return "legacy_idle", warnings
        return "legacy_inconsistent", warnings + ["Legacy IDLE header contains run data or an invalid CRC"]
    if after_state == exporter.STATE_ACTIVE and before_state in (
            exporter.STATE_IDLE, exporter.STATE_ACTIVE):
        if before_state == exporter.STATE_IDLE or before_session == after_session:
            return "legacy_active", warnings
        return "legacy_inconsistent", warnings + ["Legacy session changed during ACTIVE read"]
    if (before_state == after_state == exporter.STATE_FROZEN and
            before_count == after_count and before_session == after_session and
            before_session != 0 and header_before["header_crc_valid"] and
            header_after["header_crc_valid"]):
        return "legacy_frozen", warnings
    if before_state == exporter.STATE_ACTIVE and after_state == exporter.STATE_FROZEN:
        return "legacy_inconsistent", warnings + ["ACTIVE-to-FROZEN transition crossed the header reads; retry"]
    return "legacy_inconsistent", warnings + ["Legacy runtime-log headers are inconsistent"]


def decode_snapshot(elf_path: str | Path, scalar_path: str | Path,
                    header_path: str | Path,
                    descriptor_path: str | Path | None = None,
                    header_after_path: str | Path | None = None,
                    diagnostic_path: str | Path | None = None,
                    speed_pi_diagnostic_path: str | Path | None = None) -> Dict[str, Any]:
    plan = build_plan(elf_path)
    flash_descriptor = None
    if descriptor_path is not None:
        flash_descriptor = exporter.validate_flash_descriptor(elf_path, descriptor_path)
    scalar_raw = Path(scalar_path).read_bytes()
    header_raw = Path(header_path).read_bytes()
    header_after_raw = (Path(header_after_path).read_bytes()
                        if header_after_path is not None else header_raw)
    scalar_read = plan["scalar_read"]
    if len(scalar_raw) != scalar_read["size"]:
        _fail("Scalar read length mismatch: expected {}, got {}".format(
            scalar_read["size"], len(scalar_raw)))

    scalars: Dict[str, Any] = {}
    read_start = scalar_read["address_u32"]
    for name, symbol in plan["scalar_symbols"].items():
        if not symbol["available"]:
            scalars[name] = None
            continue
        offset = symbol["address_u32"] - read_start
        value = struct.unpack_from("<I", scalar_raw, offset)[0]
        if name in SIGNED_SCALAR_SYMBOLS and value & 0x80000000:
            value -= 0x100000000
        scalars[name] = value
    scalars["car_mode_name"] = exporter.MODE_NAMES.get(scalars["car_mode"], "unknown")
    scalars["car_fault_name"] = exporter.FAULT_NAMES.get(scalars["car_fault"], "unknown")
    scalars["iwdg_reset_flag_in_rcc_rsr"] = bool(
        scalars["car_init_rcc_rsr_reset_snapshot"] & RCC_RSR_IWDG1RSTF_MASK)
    scalars["rcc_rsr_iwdg1rstf_mask"] = _hex(RCC_RSR_IWDG1RSTF_MASK)

    snapshot_present = [name for name in HOLD_SNAPSHOT_SYMBOLS
                        if plan["scalar_symbols"][name]["available"]]
    scalars["car_vision_hold_snapshot_status"] = (
        "unavailable" if not snapshot_present else
        "available" if len(snapshot_present) == len(HOLD_SNAPSHOT_SYMBOLS) else "partial")
    entry_value = scalars.get("car_vision_hold_snapshot_entry")
    scalars["car_vision_hold_snapshot_published"] = (
        None if entry_value is None else bool(entry_value))
    header = _decode_header(header_raw, plan["build_id"], plan["log_version"])
    header_after = _decode_header(header_after_raw, plan["build_id"], plan["log_version"])
    diagnostic = None
    diagnostic_class = None
    active_progress = False
    diagnostic_warnings: list[str] = []
    if plan["runtime_diagnostic_snapshot"]["available"]:
        if diagnostic_path is None:
            _fail("Selected ELF contains g_runtime_diag_snapshot but no snapshot upload was supplied")
        diagnostic_raw = Path(diagnostic_path).read_bytes()
        diagnostic = decode_runtime_diagnostic(diagnostic_raw)
        diagnostic_class, diagnostic_warnings, active_progress = _classify_runtime_diagnostic(
            diagnostic, header, header_after)
    elif diagnostic_path is not None:
        _fail("A runtime diagnostic upload was supplied for an ELF without g_runtime_diag_snapshot")
    else:
        diagnostic_class, diagnostic_warnings = _classify_legacy_headers(header, header_after)

    speed_pi_diagnostic = None
    speed_pi_warnings: list[str] = []
    speed_pi_plan = plan["speed_pi_diagnostic_snapshot"]
    if speed_pi_plan["available"]:
        if speed_pi_diagnostic_path is None:
            _fail("Selected ELF contains g_speed_pi_diag_snapshot but no snapshot upload was supplied")
        speed_pi_diagnostic = decode_speed_pi_diagnostic(
            Path(speed_pi_diagnostic_path).read_bytes())
        state_after = int(header_after["state"])
        pi_type = int(speed_pi_diagnostic["pi_type"])
        pi_active = bool(speed_pi_diagnostic["active"])
        pi_frozen = bool(speed_pi_diagnostic["frozen"])
        if pi_active or pi_frozen:
            if diagnostic is None:
                speed_pi_warnings.append(
                    "Speed PI snapshot is associated by manual PE7 start token, "
                    "but this ELF has no runtime diagnostic token for comparison.")
                diagnostic_class = "inconsistent"
            else:
                token_matches = (
                    int(speed_pi_diagnostic["start_token"]) ==
                    int(diagnostic["start_token"]))
                speed_pi_diagnostic["start_token_matches_runtime_diagnostic"] = token_matches
                if not token_matches:
                    speed_pi_warnings.append(
                        "Speed PI manual PE7 start token does not match the runtime diagnostic token.")
                    diagnostic_class = "inconsistent"
        if pi_type != 0 and state_after in (exporter.STATE_ACTIVE, exporter.STATE_FROZEN):
            expected_pi_state = "active" if state_after == exporter.STATE_ACTIVE else "frozen"
            if not speed_pi_diagnostic[expected_pi_state]:
                speed_pi_warnings.append(
                    "Speed PI snapshot is not {} while the runtime log is {}.".format(
                        expected_pi_state.upper(), STATE_NAMES[state_after].upper()))
                diagnostic_class = "inconsistent"
        if diagnostic_class in ("inconsistent", "legacy_inconsistent"):
            diagnostic_class = "inconsistent"
    elif speed_pi_diagnostic_path is not None:
        _fail("A speed PI diagnostic upload was supplied for an ELF without g_speed_pi_diag_snapshot")

    consistency_warnings = [
        "Scalar region, runtime diagnostic, and runtime-log headers use separate SWD uploads; "
        "the two headers bracket the single-record CRC-protected diagnostic snapshot."
    ]
    warnings = list(header["warnings"]) + list(header_after["warnings"]) + consistency_warnings
    warnings.extend(diagnostic_warnings)
    warnings.extend(speed_pi_warnings)
    if descriptor_path is None:
        warnings.append("Target Flash descriptor was not uploaded; the running firmware ELF was not verified.")
    status = diagnostic_class or ("read_with_warnings" if warnings else
                                  "idle" if header["state"] == exporter.STATE_IDLE else "success")
    return {
        "status": status,
        "status_class": diagnostic_class,
        "snapshot_kind": "live_diagnostic_only",
        "diagnostic_is_run_record": False,
        "csv_created": False,
        "reads_are_atomic_together": False,
        "warnings": warnings,
        "consistency_warnings": consistency_warnings,
        "flash_descriptor": flash_descriptor,
        "observing_active_progress": active_progress,
        "consistency_note": (
            "The 64-byte runtime and 128-byte speed PI diagnostic snapshots are each "
            "generation/CRC-protected records. Runtime-log headers are uploaded before and "
            "after them. The speed PI snapshot is linked to the current run by the manual "
            "PE7 start token, checked against the runtime diagnostic when present. ACTIVE "
            "records may advance during separate SWD calls; stable IDLE/FROZEN state, count, "
            "session, and enabled PI lifecycle must agree."
        ),
        "elf": {
            "path": plan["elf_path"],
            "sha256": plan["elf_sha256"],
            "build_id": plan["build_id"],
            "endianness": plan["endianness"],
            "log_version": plan["log_version"],
            "hold_quantum_ms": plan["hold_quantum_ms"],
        },
        "reads": {
            "flash_rom_descriptor": plan["descriptor_read"],
            "scalar_region": plan["scalar_read"],
            "runtime_log_header": plan["header_read"],
            "runtime_log_header_after": plan["header_read"],
            "runtime_diagnostic_snapshot": plan["runtime_diagnostic_snapshot"],
            "speed_pi_diagnostic_snapshot": plan["speed_pi_diagnostic_snapshot"],
            "runtime_log_record_area_read": False,
        },
        "scalars": scalars,
        "runtime_log_header": header,
        "runtime_log_header_after": header_after,
        "runtime_diagnostic_snapshot": diagnostic,
        "speed_pi_diagnostic_snapshot": speed_pi_diagnostic,
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
            "runtime_log_header_after": {
                "path": str(Path(header_after_path).resolve()) if header_after_path is not None else str(Path(header_path).resolve()),
                "bytes": len(header_after_raw),
                "sha256": hashlib.sha256(header_after_raw).hexdigest(),
            },
            "runtime_diagnostic_snapshot": (None if diagnostic_path is None else {
                "path": str(Path(diagnostic_path).resolve()),
                "bytes": len(Path(diagnostic_path).read_bytes()),
                "sha256": hashlib.sha256(Path(diagnostic_path).read_bytes()).hexdigest(),
            }),
            "speed_pi_diagnostic_snapshot": (None if speed_pi_diagnostic_path is None else {
                "path": str(Path(speed_pi_diagnostic_path).resolve()),
                "bytes": len(Path(speed_pi_diagnostic_path).read_bytes()),
                "sha256": hashlib.sha256(Path(speed_pi_diagnostic_path).read_bytes()).hexdigest(),
            }),
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
    decode_parser.add_argument("--header-after",
                               help="optional 64-byte runtime-log header uploaded after the diagnostics")
    decode_parser.add_argument("--runtime-diagnostic",
                               help="optional 64-byte g_runtime_diag_snapshot upload")
    decode_parser.add_argument("--speed-pi-diagnostic",
                               help="optional 128-byte g_speed_pi_diag_snapshot upload")
    decode_parser.add_argument("--descriptor",
                               help="optional uploaded 64-byte Flash descriptor to verify the target ELF")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "plan":
            _emit_json(build_plan(args.elf))
        else:
            _emit_json(decode_snapshot(args.elf, args.scalars, args.header,
                                       descriptor_path=args.descriptor,
                                       header_after_path=args.header_after,
                                       diagnostic_path=args.runtime_diagnostic,
                                       speed_pi_diagnostic_path=args.speed_pi_diagnostic))
        return 0
    except (StatusReadError, exporter.ExportError, OSError, struct.error) as exc:
        print("run-status-reader: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
