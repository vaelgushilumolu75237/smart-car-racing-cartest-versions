#!/usr/bin/env python3
"""Inspect CarTest.elf and decode the frozen RAM runtime log without dependencies."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import struct
import sys
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple, Union


DESCRIPTOR_SYMBOL = "car_runtime_log_rom_descriptor"
STORAGE_SYMBOL = "car_runtime_log_storage"
START_SYMBOL = "__runtime_log_start"
END_SYMBOL = "__runtime_log_end"

DESCRIPTOR_MAGIC = b"RLDS"
HEADER_MAGIC = b"RLOG"
ENDIAN_TAG = 0x12345678
DESCRIPTOR_VERSION = 1
LOG_VERSION = 1
HEADER_SIZE = 64
RECORD_SIZE = 64
CAPACITY = 6144
PERIOD_MS = 50
ALIGNMENT = 32
BUFFER_SIZE = HEADER_SIZE + CAPACITY * RECORD_SIZE

STATE_IDLE = 0
STATE_ACTIVE = 1
STATE_FROZEN = 2
HEADER_FLAG_FULL = 0x01
HEADER_FLAG_TRUNCATED = 0x02
EVENT_NAMES = {
    0: "periodic",
    1: "start",
    2: "stop",
    3: "fault",
    4: "mode",
    5: "recovery",
    6: "auto_recovery",
    7: "hold_enter",
    8: "hold_exit",
    9: "hold_deadline",
    10: "button_start",
    11: "button_stop",
}
VISION_FLAG_NAMES = {
    0: "latest_valid",
    1: "usable_fresh",
    2: "timeout",
    3: "latest_frame_present",
    4: "hold_active",
    5: "hold_expired",
    6: "latest_slope_hint",
    7: "slope_assist_commanded",
}
MODE_NAMES = {
    0: "boot_safe",
    1: "wait_switch_release",
    2: "wait_local_ack",
    3: "wait_new_frame",
    4: "wait_start_release",
    5: "wait_start_edge",
    6: "bench_running",
    7: "bench_complete",
    8: "fault_latched",
    9: "recovery_wait",
    10: "follow_running",
    11: "debug_steering_running",
    12: "follow_vision_hold",
}
FAULT_NAMES = {
    0: "none",
    1: "watchdog_reset",
    2: "loop_gap",
    3: "uart",
    4: "rx_overflow",
    5: "rx_stale",
    6: "protocol",
    7: "sequence",
    8: "vision_invalid",
    9: "vision_timeout",
    10: "switch_release",
    11: "encoder_init",
    12: "clock",
    13: "timer",
    14: "config",
    15: "watchdog_init",
}
STOP_REASON_NAMES = {
    1: "manual_stop",
    2: "hard_fault",
    3: "bench_complete",
    4: "session_ended",
}


class ExportError(ValueError):
    """Input ELF or RAM log failed a safety or format check."""


@dataclass(frozen=True)
class Section:
    index: int
    name: str
    kind: int
    flags: int
    address: int
    offset: int
    size: int
    link: int
    info: int
    alignment: int
    entry_size: int


@dataclass(frozen=True)
class Symbol:
    name: str
    value: int
    size: int
    binding: int
    kind: int
    section_index: int


@dataclass(frozen=True)
class ElfLayout:
    path: Path
    sha256: str
    storage_address: int
    storage_size: int
    descriptor_address: int
    build_id: bytes
    descriptor: Dict[str, int]


def _fail(message: str) -> None:
    raise ExportError(message)


def _read_c_string(blob: bytes, offset: int) -> str:
    if offset < 0 or offset >= len(blob):
        _fail("ELF string-table offset is outside the table")
    end = blob.find(b"\0", offset)
    if end < 0:
        _fail("ELF string-table entry is not NUL terminated")
    return blob[offset:end].decode("ascii", errors="strict")


def _parse_elf(path: Path) -> Tuple[bytes, List[Section], Dict[str, Symbol]]:
    try:
        blob = path.read_bytes()
    except OSError as exc:
        _fail("Cannot read ELF file: {}".format(exc))
    if len(blob) < 52 or blob[:4] != b"\x7fELF":
        _fail("Selected file is not an ELF32 file")
    if blob[4] != 1:
        _fail("ELF must be 32-bit (ELFCLASS32)")
    if blob[5] != 1:
        _fail("ELF must use little-endian byte order")
    if blob[6] != 1:
        _fail("ELF identification version is unsupported")

    header = struct.unpack_from("<HHIIIIIHHHHHH", blob, 16)
    e_type, machine = header[0], header[1]
    e_shoff, e_shentsize, e_shnum, e_shstrndx = header[5], header[10], header[11], header[12]
    if machine != 40:  # EM_ARM
        _fail("ELF machine is not ARM (EM_ARM)")
    if e_type not in (2, 3):  # ET_EXEC / ET_DYN
        _fail("ELF is not a linked executable image")
    if e_shentsize < 40 or e_shnum == 0 or e_shstrndx >= e_shnum:
        _fail("ELF section table is missing or malformed")
    if e_shoff + e_shentsize * e_shnum > len(blob):
        _fail("ELF section table extends past end of file")

    raw_sections = []
    for index in range(e_shnum):
        offset = e_shoff + index * e_shentsize
        values = struct.unpack_from("<IIIIIIIIII", blob, offset)
        raw_sections.append(values)
    names_section = raw_sections[e_shstrndx]
    names_offset, names_size = names_section[4], names_section[5]
    if names_offset + names_size > len(blob):
        _fail("ELF section-name table extends past end of file")
    section_names = blob[names_offset:names_offset + names_size]

    sections: List[Section] = []
    for index, values in enumerate(raw_sections):
        name_index, kind, flags, address, offset, size, link, info, alignment, entry_size = values
        name = _read_c_string(section_names, name_index) if name_index else ""
        if kind != 8 and offset + size > len(blob):  # SHT_NOBITS has no file payload
            _fail("ELF section '{}' extends past end of file".format(name))
        sections.append(Section(index, name, kind, flags, address, offset, size,
                                link, info, alignment, entry_size))

    symbols: Dict[str, Symbol] = {}
    for section in sections:
        if section.kind not in (2, 11):  # SHT_SYMTAB / SHT_DYNSYM
            continue
        if section.link >= len(sections):
            _fail("ELF symbol table has an invalid string-table link")
        string_section = sections[section.link]
        string_table = blob[string_section.offset:string_section.offset + string_section.size]
        entry_size = section.entry_size or 16
        if entry_size < 16 or section.size % entry_size:
            _fail("ELF symbol table has an invalid entry size")
        for offset in range(section.offset, section.offset + section.size, entry_size):
            name_idx, value, size, info, _other, section_index = struct.unpack_from("<IIIBBH", blob, offset)
            if not name_idx:
                continue
            name = _read_c_string(string_table, name_idx)
            symbols[name] = Symbol(name, value, size, info >> 4, info & 0x0F, section_index)
    return blob, sections, symbols


def _section_for_symbol(sections: Sequence[Section], symbol: Symbol, expected_name: str) -> Section:
    if symbol.section_index == 0 or symbol.section_index >= len(sections):
        _fail("ELF symbol '{}' is undefined or has an invalid section".format(symbol.name))
    section = sections[symbol.section_index]
    if section.name != expected_name:
        _fail("ELF symbol '{}' must be in {} (found {})".format(
            symbol.name, expected_name, section.name or "unnamed section"))
    if symbol.kind != 1:  # STT_OBJECT
        _fail("ELF symbol '{}' is not an object symbol".format(symbol.name))
    if symbol.value < section.address or symbol.value + symbol.size > section.address + section.size:
        _fail("ELF symbol '{}' lies outside its section".format(symbol.name))
    return section


def _require_symbol(symbols: Dict[str, Symbol], name: str) -> Symbol:
    symbol = symbols.get(name)
    if symbol is None:
        _fail("ELF is missing required symbol '{}'".format(name))
    return symbol


def inspect_elf(elf_path: Union[os.PathLike, str]) -> ElfLayout:
    path = Path(elf_path).resolve()
    if path.name.lower() != "cartest.elf":
        _fail("Selected ELF must be named CarTest.elf")
    blob, sections, symbols = _parse_elf(path)

    storage = _require_symbol(symbols, STORAGE_SYMBOL)
    storage_section = _section_for_symbol(sections, storage, ".runtime_log")
    if storage_section.kind != 8:  # SHT_NOBITS, emitted by a NOLOAD RAM section
        _fail("Runtime log RAM section must be SHT_NOBITS/NOLOAD")
    if not (storage_section.flags & 0x2 and storage_section.flags & 0x1):  # ALLOC | WRITE
        _fail("Runtime log RAM section must be allocated and writable")
    if storage.size != BUFFER_SIZE or storage_section.size != BUFFER_SIZE:
        _fail("Runtime log ELF storage must be exactly {} bytes".format(BUFFER_SIZE))
    if storage.value % ALIGNMENT:
        _fail("Runtime log storage is not {}-byte aligned".format(ALIGNMENT))
    if not (0x24000000 <= storage.value and storage.value + storage.size <= 0x24080000):
        _fail("Runtime log storage is outside STM32H743 D1 SRAM")

    start = _require_symbol(symbols, START_SYMBOL)
    end = _require_symbol(symbols, END_SYMBOL)
    if start.value != storage.value or end.value != storage.value + storage.size:
        _fail("Runtime log linker bounds do not exactly bracket the storage object")
    if start.section_index != storage.section_index or end.section_index != storage.section_index:
        _fail("Runtime log linker bounds do not belong to .runtime_log")

    descriptor_symbol = _require_symbol(symbols, DESCRIPTOR_SYMBOL)
    descriptor_section = _section_for_symbol(sections, descriptor_symbol, ".runtime_log_descriptor")
    if descriptor_symbol.size != HEADER_SIZE or descriptor_section.size != HEADER_SIZE:
        _fail("ROM descriptor symbol/section must be exactly 64 bytes")
    if descriptor_section.kind != 1:  # SHT_PROGBITS
        _fail("ROM descriptor must be present in the ELF file image")
    if not (descriptor_section.flags & 0x2) or descriptor_section.flags & 0x1:
        _fail("ROM descriptor section must be allocated and read-only")
    if not (0x08000000 <= descriptor_symbol.value < 0x08200000):
        _fail("ROM descriptor is outside STM32H743 internal Flash")

    descriptor_offset = descriptor_section.offset + descriptor_symbol.value - descriptor_section.address
    descriptor = blob[descriptor_offset:descriptor_offset + HEADER_SIZE]
    if len(descriptor) != HEADER_SIZE or descriptor[:4] != DESCRIPTOR_MAGIC:
        _fail("ROM descriptor magic or length is invalid")
    descriptor_version, descriptor_size, log_version, header_size, record_size, capacity, period, alignment = struct.unpack_from(
        "<8H", descriptor, 4)
    buffer_size, endian_tag = struct.unpack_from("<II", descriptor, 20)
    build_id = descriptor[28:44]
    reserved = descriptor[44:64]
    if (descriptor_version, descriptor_size, log_version, header_size, record_size,
            capacity, period, alignment, buffer_size, endian_tag) != (
            DESCRIPTOR_VERSION, 64, LOG_VERSION, HEADER_SIZE, RECORD_SIZE,
            CAPACITY, PERIOD_MS, ALIGNMENT, BUFFER_SIZE, ENDIAN_TAG):
        _fail("ROM descriptor schema values do not match this exporter")
    if not any(build_id):
        _fail("ROM descriptor build ID is empty")
    if any(reserved):
        _fail("ROM descriptor reserved bytes are not zero")

    descriptor_values = {
        "descriptor_version": descriptor_version,
        "descriptor_size": descriptor_size,
        "log_version": log_version,
        "header_size": header_size,
        "record_size": record_size,
        "capacity": capacity,
        "period_ms": period,
        "alignment": alignment,
        "buffer_size": buffer_size,
        "endian_tag": endian_tag,
    }
    return ElfLayout(
        path=path,
        sha256=hashlib.sha256(blob).hexdigest(),
        storage_address=storage.value,
        storage_size=storage.size,
        descriptor_address=descriptor_symbol.value,
        build_id=build_id,
        descriptor=descriptor_values,
    )


def _crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def _parse_header(header_bytes: bytes, layout: ElfLayout) -> Dict[str, Union[int, str, bool]]:
    if len(header_bytes) != HEADER_SIZE:
        _fail("Runtime log header must be exactly {} bytes".format(HEADER_SIZE))
    if header_bytes[:4] != HEADER_MAGIC:
        _fail("Runtime log header magic is invalid")
    version, header_size, record_size, capacity, period = struct.unpack_from("<5H", header_bytes, 4)
    state, flags = struct.unpack_from("<2B", header_bytes, 14)
    count, session_id, start_ms, stop_ms, dropped_count, records_crc, header_crc = struct.unpack_from(
        "<7I", header_bytes, 16)
    build_id = header_bytes[44:60]
    stop_reason = header_bytes[60]
    reserved = header_bytes[61:64]
    expected_values = (LOG_VERSION, HEADER_SIZE, RECORD_SIZE, CAPACITY, PERIOD_MS)
    if (version, header_size, record_size, capacity, period) != expected_values:
        _fail("Runtime log header schema does not match the ROM descriptor")
    zeroed_header = bytearray(header_bytes)
    zeroed_header[40:44] = b"\0\0\0\0"
    if _crc32(bytes(zeroed_header)) != header_crc:
        _fail("Runtime log header CRC32 is invalid")
    if any(reserved):
        _fail("Runtime log header reserved bytes are not zero")
    if build_id != layout.build_id:
        _fail(
            "Runtime log header build ID mismatch at RAM address "
            "0x{:08X}: selected CarTest.elf expects {}, header contains {}"
            .format(layout.storage_address, layout.build_id.hex(), build_id.hex())
        )
    if state == STATE_IDLE:
        _fail("Runtime log is idle; no run has been recorded")
    if state == STATE_ACTIVE:
        _fail("Runtime log is active; stop and freeze the run before exporting")
    if state != STATE_FROZEN:
        _fail("Runtime log state is invalid")
    if flags & ~(HEADER_FLAG_FULL | HEADER_FLAG_TRUNCATED):
        _fail("Runtime log header contains unsupported flags")
    if count < 2 or count > CAPACITY:
        _fail("Frozen runtime log count must be between 2 and capacity")
    if session_id == 0:
        _fail("Frozen runtime log session_id must be nonzero")
    if stop_reason not in (1, 2, 3, 4):
        _fail("Frozen runtime log stop_reason is outside the defined range 1..4")
    is_full = bool(flags & HEADER_FLAG_FULL)
    is_truncated = bool(flags & HEADER_FLAG_TRUNCATED)
    if is_full != (count >= CAPACITY - 1):
        _fail("Runtime log FULL flag disagrees with committed record count")
    if is_truncated != bool(dropped_count):
        _fail("Runtime log TRUNCATED flag disagrees with dropped_count")
    if dropped_count and (not is_full or count != CAPACITY):
        _fail("Runtime log reports dropped records before reaching capacity")
    return {
        "version": version,
        "header_size": header_size,
        "record_size": record_size,
        "capacity": capacity,
        "period_ms": period,
        "state": state,
        "state_name": "frozen",
        "flags": flags,
        "buffer_full": is_full,
        "records_truncated": is_truncated,
        "count": count,
        "session_id": session_id,
        "start_ms": start_ms,
        "stop_ms": stop_ms,
        "dropped_count": dropped_count,
        "records_crc32": records_crc,
        "header_crc32": header_crc,
        "build_id": build_id.hex(),
        "stop_reason": stop_reason,
        "stop_reason_name": STOP_REASON_NAMES[stop_reason],
    }


def inspect_header(elf_path: Union[os.PathLike, str], header_path: Union[os.PathLike, str]) -> Dict[str, Union[int, str, bool]]:
    layout = inspect_elf(elf_path)
    try:
        header = Path(header_path).read_bytes()
    except OSError as exc:
        _fail("Cannot read runtime log header: {}".format(exc))
    parsed = _parse_header(header, layout)
    parsed["payload_size"] = HEADER_SIZE + int(parsed["count"]) * RECORD_SIZE
    return parsed


def _parse_record(record: bytes, session_id: int, start_ms: int, index: int) -> Dict[str, Union[int, str, bool]]:
    if len(record) != RECORD_SIZE:
        _fail("Runtime log record {} is truncated".format(index))
    t_ms, record_session = struct.unpack_from("<II", record, 0)
    encoder_left, encoder_right = struct.unpack_from("<qq", record, 8)
    fault_generation = struct.unpack_from("<I", record, 24)[0]
    servo_req, servo_limited, servo_applied, motor_req, motor_left, motor_right = struct.unpack_from(
        "<6H", record, 28)
    ccr1, ccr2, ccr3, ccr4 = struct.unpack_from("<4H", record, 40)
    vision_error = struct.unpack_from("<h", record, 48)[0]
    sequence = struct.unpack_from("<H", record, 50)[0]
    auto_recovery_count = struct.unpack_from("<I", record, 52)[0]
    event_flags = struct.unpack_from("<H", record, 56)[0]
    mode, fault, confidence, vision_flags, recovery_frame_count, recovery_active = struct.unpack_from(
        "<6B", record, 58)
    if record_session != session_id:
        _fail("Runtime log record {} belongs to a different session".format(index))
    if event_flags & ~0x7FFF:
        _fail("Runtime log record {} contains unsupported event flags".format(index))
    hold_exit_cause = (event_flags >> 12) & 0x07
    if hold_exit_cause > 5 or (hold_exit_cause != 0 and
                               not (event_flags & (1 << 8))):
        _fail("Runtime log record {} has an invalid hold-exit cause".format(index))
    if (event_flags & (1 << 8)) and hold_exit_cause == 0:
        _fail("Runtime log record {} is missing its hold-exit cause".format(index))
    if vision_flags & ~0xFF:
        _fail("Runtime log record {} contains unsupported vision flags".format(index))
    if recovery_active not in (0, 1):
        _fail("Runtime log record {} has an invalid recovery_active value".format(index))
    vision_hold_active = int(bool(vision_flags & (1 << 4)))
    vision_hold_expired = int(bool(vision_flags & (1 << 5)))
    if vision_hold_active and vision_hold_expired:
        _fail("Runtime log record {} marks vision hold active and expired".format(index))
    if mode == 12:
        hold_ticks = recovery_frame_count >> 2
        if hold_ticks > 50:
            _fail("Runtime log record {} has a hold duration above 1000 ms".format(index))
        if not (vision_hold_active or vision_hold_expired):
            _fail("Runtime log mode 12 is missing active/expired hold flags")
        hold_elapsed_ms = hold_ticks * 20
        hold_valid_count = recovery_frame_count & 0x03
        decoded_recovery_count = hold_valid_count
    else:
        hold_elapsed_ms = 0
        hold_valid_count = 0
        decoded_recovery_count = recovery_frame_count
        if vision_hold_active or vision_hold_expired:
            _fail("Runtime log hold flags are only valid in mode 12")

    # HAL tick time is a wrapping u32. Difference from the session start remains
    # correct for a session shorter than one full 49.7-day tick period.
    elapsed_ms = (t_ms - start_ms) & 0xFFFFFFFF
    row: Dict[str, Union[int, str, bool]] = {
        "t_ms": t_ms,
        "t_elapsed_ms": elapsed_ms,
        "session_id": record_session,
        "encoder_left_count": encoder_left,
        "encoder_right_count": encoder_right,
        "fault_generation": fault_generation,
        "servo_requested_us": servo_req,
        "servo_limited_us": servo_limited,
        "servo_applied_us": servo_applied,
        "motor_requested_permille": motor_req,
        "motor_left_applied_permille": motor_left,
        "motor_right_applied_permille": motor_right,
        "tim3_ccr1_counts": ccr1,
        "tim3_ccr2_counts": ccr2,
        "tim3_ccr3_counts": ccr3,
        "tim3_ccr4_counts": ccr4,
        "vision_latest_error_norm_milli": vision_error,
        "vision_sequence": sequence,
        "auto_recovery_count": auto_recovery_count,
        "event_flags": event_flags,
        "vision_hold_exit_cause": hold_exit_cause,
        "mode": mode,
        "mode_name": MODE_NAMES.get(mode, "unknown_{}".format(mode)),
        "fault": fault,
        "fault_name": FAULT_NAMES.get(fault, "unknown_{}".format(fault)),
        "vision_confidence_percent": confidence,
        "vision_flags": vision_flags,
        "vision_latest_valid": int(bool(vision_flags & (1 << 0))),
        "vision_usable_fresh": int(bool(vision_flags & (1 << 1))),
        "vision_timeout": int(bool(vision_flags & (1 << 2))),
        "vision_latest_frame_present": int(bool(vision_flags & (1 << 3))),
        "vision_latest_slope_hint": int(bool(vision_flags & (1 << 6))),
        "vision_slope_assist_commanded": int(bool(vision_flags & (1 << 7))),
        "recovery_frame_count": decoded_recovery_count,
        "recovery_active": recovery_active,
        "vision_hold_active": vision_hold_active,
        "vision_hold_expired": vision_hold_expired,
        "vision_hold_elapsed_ms": hold_elapsed_ms,
        "vision_hold_valid_count": hold_valid_count,
    }
    for bit, name in EVENT_NAMES.items():
        row["event_" + name] = int(bool(event_flags & (1 << bit)))
    return row


CSV_FIELDS = [
    "t_ms", "t_elapsed_ms", "session_id",
    "encoder_left_count", "encoder_right_count", "fault_generation",
    "servo_requested_us", "servo_limited_us", "servo_applied_us",
    "motor_requested_permille", "motor_left_applied_permille", "motor_right_applied_permille",
    "tim3_ccr1_counts", "tim3_ccr2_counts", "tim3_ccr3_counts", "tim3_ccr4_counts",
    "vision_latest_error_norm_milli", "vision_sequence", "vision_confidence_percent", "vision_flags",
    "vision_latest_valid", "vision_usable_fresh", "vision_timeout", "vision_latest_frame_present",
    "vision_latest_slope_hint", "vision_slope_assist_commanded",
    "vision_hold_active", "vision_hold_expired", "vision_hold_elapsed_ms",
    "vision_hold_valid_count", "auto_recovery_count", "recovery_frame_count", "recovery_active",
    "mode", "mode_name", "fault", "fault_name", "event_flags",
    "event_periodic", "event_start", "event_stop", "event_fault", "event_mode",
    "event_recovery", "event_auto_recovery",
    "vision_hold_exit_cause", "event_hold_enter", "event_hold_exit",
    "event_hold_deadline", "event_button_start", "event_button_stop",
]


def decode_buffer(elf_path: Union[os.PathLike, str], raw_path: Union[os.PathLike, str]) -> Tuple[ElfLayout, Dict[str, Union[int, str, bool]], List[Dict[str, Union[int, str, bool]]]]:
    layout = inspect_elf(elf_path)
    try:
        raw = Path(raw_path).read_bytes()
    except OSError as exc:
        _fail("Cannot read uploaded runtime log: {}".format(exc))
    if len(raw) < HEADER_SIZE:
        _fail("Uploaded runtime log is shorter than its header")
    header = _parse_header(raw[:HEADER_SIZE], layout)
    expected_size = HEADER_SIZE + int(header["count"]) * RECORD_SIZE
    if len(raw) != expected_size:
        _fail("Uploaded runtime log length is {} bytes; header requires exactly {}".format(len(raw), expected_size))
    payload = raw[HEADER_SIZE:]
    if _crc32(payload) != int(header["records_crc32"]):
        _fail("Runtime log records CRC32 is invalid")
    rows = []
    for index in range(int(header["count"])):
        offset = index * RECORD_SIZE
        rows.append(_parse_record(payload[offset:offset + RECORD_SIZE],
                                  int(header["session_id"]), int(header["start_ms"]), index))
    if rows[0]["t_ms"] != header["start_ms"]:
        _fail("First runtime log record time does not match header start_ms")
    if not rows[0]["event_start"]:
        _fail("First runtime log record is missing the START event")
    if rows[-1]["t_ms"] != header["stop_ms"]:
        _fail("Final runtime log record time does not match header stop_ms")
    if not rows[-1]["event_stop"]:
        _fail("Final runtime log record is missing the STOP event")
    if (rows[-1]["servo_applied_us"] != 0 or
            rows[-1]["motor_left_applied_permille"] != 0 or
            rows[-1]["motor_right_applied_permille"] != 0 or
            any(rows[-1]["tim3_ccr{}_counts".format(channel)] != 0 for channel in (1, 2, 3, 4))):
        _fail("Final STOP record must show zero software-applied servo and motor outputs")
    elapsed_stop_ms = (int(header["stop_ms"]) - int(header["start_ms"])) & 0xFFFFFFFF
    previous_elapsed_ms = 0
    for index, row in enumerate(rows):
        elapsed_ms = int(row["t_elapsed_ms"])
        if elapsed_ms < previous_elapsed_ms:
            _fail("Runtime log record timestamps are not monotonic modulo-u32")
        if elapsed_ms > elapsed_stop_ms:
            _fail("Runtime log record time is after header stop_ms")
        previous_elapsed_ms = elapsed_ms
    if previous_elapsed_ms != elapsed_stop_ms:
        _fail("Final record elapsed time does not match header start_ms/stop_ms")
    return layout, header, rows


def _atomic_write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n", dir=str(path.parent),
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temp_path = Path(stream.name)
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(str(temp_path), str(path))
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def _atomic_write_csv(path: Path, rows: Iterable[Dict[str, Union[int, str, bool]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8-sig", newline="", dir=str(path.parent),
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temp_path = Path(stream.name)
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="raise")
            writer.writeheader()
            writer.writerows(rows)
        os.replace(str(temp_path), str(path))
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def _layout_json(layout: ElfLayout) -> Dict[str, object]:
    return {
        "elf_path": str(layout.path),
        "elf_sha256": layout.sha256,
        "machine": "ARM",
        "storage_symbol": STORAGE_SYMBOL,
        "storage_address": "0x{:08X}".format(layout.storage_address),
        "storage_address_u32": layout.storage_address,
        "storage_size": layout.storage_size,
        "descriptor_symbol": DESCRIPTOR_SYMBOL,
        "descriptor_address": "0x{:08X}".format(layout.descriptor_address),
        "build_id": layout.build_id.hex(),
        "descriptor": layout.descriptor,
    }


def command_inspect(args: argparse.Namespace) -> int:
    layout = inspect_elf(args.elf)
    print(json.dumps(_layout_json(layout), sort_keys=True))
    return 0


def command_check_header(args: argparse.Namespace) -> int:
    layout = inspect_elf(args.elf)
    try:
        header = Path(args.header).read_bytes()
    except OSError as exc:
        _fail("Cannot read runtime log header: {}".format(exc))
    parsed = _parse_header(header, layout)
    parsed["payload_size"] = HEADER_SIZE + int(parsed["count"]) * RECORD_SIZE
    print(json.dumps(parsed, sort_keys=True))
    return 0


def command_decode(args: argparse.Namespace) -> int:
    layout, header, rows = decode_buffer(args.elf, args.raw)
    csv_path = Path(args.out_csv).resolve()
    metadata_path = Path(args.metadata).resolve() if args.metadata else csv_path.with_suffix(".metadata.json")
    # Validate everything before creating either success artifact.
    metadata = {
        "elf": _layout_json(layout),
        "log": header,
        "record_count": len(rows),
        "dropped_count": int(header["dropped_count"]),
        "buffer_full": bool(header["buffer_full"]),
        "records_truncated": bool(header["records_truncated"]),
        "raw_file": str(Path(args.raw).resolve()),
        "raw_size": Path(args.raw).stat().st_size,
        "csv_file": str(csv_path),
        "csv_rows": len(rows),
    }
    try:
        _atomic_write_csv(csv_path, rows)
        _atomic_write_json(metadata_path, metadata)
    except Exception:
        if csv_path.exists():
            csv_path.unlink()
        if metadata_path.exists():
            metadata_path.unlink()
        raise
    print(json.dumps({"csv_path": str(csv_path), "metadata_path": str(metadata_path),
                      "record_count": len(rows), "dropped_count": int(header["dropped_count"]),
                      "buffer_full": bool(header["buffer_full"]),
                      "records_truncated": bool(header["records_truncated"])}, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="validate the selected ELF and print RAM layout as JSON")
    inspect_parser.add_argument("--elf", required=True)
    inspect_parser.set_defaults(func=command_inspect)

    header_parser = subparsers.add_parser("check-header", help="validate a 64-byte frozen RAM header")
    header_parser.add_argument("--elf", required=True)
    header_parser.add_argument("--header", required=True)
    header_parser.set_defaults(func=command_check_header)

    decode_parser = subparsers.add_parser("decode", help="validate uploaded bytes and atomically write CSV")
    decode_parser.add_argument("--elf", required=True)
    decode_parser.add_argument("--raw", required=True)
    decode_parser.add_argument("--out-csv", required=True)
    decode_parser.add_argument("--metadata")
    decode_parser.set_defaults(func=command_decode)
    return parser


def main(argv: Union[Sequence[str], None] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (ExportError, OSError, struct.error, UnicodeError) as exc:
        print("runtime-log-export: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
