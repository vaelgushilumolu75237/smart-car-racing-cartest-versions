#!/usr/bin/env python3
"""Offline comparison of OpenMV steering diagnostics and a frozen H743 run CSV."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple


CAMERA_FIELDS = (
    "sequence", "capture_ms", "send_ms", "valid", "full_write", "write_error",
    "reverse_fresh", "position_norm", "preview_candidate_norm", "preview_applied_norm",
    "sent_error_norm", "raw_control_norm", "filtered_error_norm", "reference_error_norm",
    "row_count", "source_code", "reverse_generation",
)
H743_FIELDS = (
    "t_ms", "t_elapsed_ms", "session_id", "vision_sequence", "vision_latest_valid",
    "vision_latest_error_norm_milli", "mode", "servo_requested_us", "servo_limited_us",
    "servo_applied_us",
)
HOLD_QUANTUM_MS = {1: 20, 2: 50}
U32_MODULUS = 1 << 32
U16_MODULUS = 1 << 16
CONTROL_EXTENSION_V2_COLUMN = "control_ext_v2_hex"
CURVE_TRACE_V2_COLUMN = "curve_trace_v2_hex"
CURVE_TRACE_V2_FIELDS = (
    "version", "payload_length", "sequence", "flags", "frame_valid",
    "slope_hint", "confidence", "lateral_error", "curve_phase",
    "slowdown_request_permille", "reserved", "crc16",
)
CONTROL_EXTENSION_V2_FIELDS = (
    "version", "flags", "pd_active", "preview_profile",
    "preview_confirmed", "geometry_valid", "position_same_sign",
    "duplicate", "profile_code", "position_p", "derivative_d",
    "pd_output", "preview_candidate", "preview_gain_milli",
    "preview_applied", "capture_dt_ms", "preview_state", "reset_code",
    "sent_control", "crc16",
)
CONTROL_EXTENSION_V2_STAGE_FIELDS = (
    "position_p", "derivative_d", "pd_output", "preview_candidate",
    "preview_gain_milli", "preview_applied", "sent_control",
)


class AnalysisError(ValueError):
    """Input evidence is malformed or cannot be safely paired."""


def _fail(message: str) -> None:
    raise AnalysisError(message)


def _parse_int(value: Any, name: str, minimum: int | None = None,
               maximum: int | None = None) -> int:
    try:
        result = int(str(value).strip(), 0)
    except (TypeError, ValueError):
        try:
            result = int(str(value).strip(), 10)
        except (TypeError, ValueError):
            _fail("{} must be an integer (got {!r})".format(name, value))
    if minimum is not None and result < minimum:
        _fail("{} is below {}".format(name, minimum))
    if maximum is not None and result > maximum:
        _fail("{} is above {}".format(name, maximum))
    return result


def _parse_bool(value: Any, name: str) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y"):
        return True
    if text in ("0", "false", "no", "n"):
        return False
    _fail("{} must be a boolean/0/1 (got {!r})".format(name, value))


def _crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _decode_control_extension_v2(value: Any,
                                 base: Mapping[str, Any]) -> Tuple[Dict[str, Any] | None, List[str], bool]:
    """Decode and bind a camera V2 control record to its base trace row.

    The extension is optional for historical traces. A present but invalid
    extension remains visible with its decoded values and an explicit issue
    list; callers must only aggregate values whose valid flag is true.
    """
    if value is None or not str(value).strip():
        return None, [], False
    text = str(value).strip()
    if len(text) != 64 or any(character not in "0123456789abcdefABCDEF"
                               for character in text):
        return None, ["extension must be exactly 32 bytes of hexadecimal"], True
    raw = bytes.fromhex(text)

    def u16(offset: int) -> int:
        return raw[offset] | (raw[offset + 1] << 8)

    def i16(offset: int) -> int:
        number = u16(offset)
        return number - 0x10000 if number & 0x8000 else number

    def u32(offset: int) -> int:
        return (raw[offset] | (raw[offset + 1] << 8) |
                (raw[offset + 2] << 16) | (raw[offset + 3] << 24))

    flags = raw[1]
    profile_code = (flags >> 6) & 0x03
    record: Dict[str, Any] = {
        "version": raw[0],
        "flags": flags,
        "pd_active": bool(flags & 0x01),
        "preview_profile": bool(flags & 0x02),
        "preview_confirmed": bool(flags & 0x04),
        "geometry_valid": bool(flags & 0x08),
        "position_same_sign": bool(flags & 0x10),
        "duplicate": bool(flags & 0x20),
        "profile_code": profile_code,
        "sequence": u16(2),
        "capture_ms": u32(4),
        "send_ms": u32(8),
        "position_p": i16(12),
        "derivative_d": i16(14),
        "pd_output": i16(16),
        "preview_candidate": i16(18),
        "preview_gain_milli": u16(20),
        "preview_applied": i16(22),
        "capture_dt_ms": u16(24),
        "preview_state": raw[26],
        "reset_code": raw[27],
        "sent_control": i16(28),
        "crc16": u16(30),
    }
    issues: List[str] = []
    if record["version"] != 2:
        issues.append("extension version is not 2")
    expected_crc = _crc16_ccitt_false(raw[:30])
    if record["crc16"] != expected_crc:
        issues.append("extension CRC16/CCITT-FALSE mismatch")
    for name in ("sequence", "capture_ms", "send_ms"):
        if record[name] != base[name]:
            issues.append("extension {} does not match base row".format(name))
    if record["sent_control"] != base["sent_error_norm"]:
        issues.append("extension sent_control does not match base packet error")
    if profile_code == 3:
        issues.append("extension profile code 3 is reserved")
    if record["preview_state"] > 3:
        issues.append("extension preview state is outside 0..3")
    if record["reset_code"] > 11:
        issues.append("extension reset code is outside 0..11")
    return record, issues, True


def _decode_curve_trace_v2(value: Any,
                           base: Mapping[str, Any]) -> Tuple[Dict[str, Any] | None, List[str], bool]:
    """Decode a raw V2 UART frame and bind it to its camera trace row.

    The wire frame contains no timestamp. Capture/send timing therefore comes
    only from the associated base row and is never decoded or cross-clock
    compared from these bytes.
    """
    if value is None or not str(value).strip():
        return None, [], False
    text = str(value).strip()
    if len(text) != 32 or any(character not in "0123456789abcdefABCDEF"
                               for character in text):
        return None, ["trace must be exactly 16 bytes of hexadecimal"], True
    raw = bytes.fromhex(text)

    def u16(offset: int) -> int:
        return raw[offset] | (raw[offset + 1] << 8)

    def i16(offset: int) -> int:
        number = u16(offset)
        return number - 0x10000 if number & 0x8000 else number

    flags = raw[6]
    record: Dict[str, Any] = {
        "version": raw[2],
        "payload_length": raw[3],
        "sequence": u16(4),
        "flags": flags,
        "frame_valid": bool(flags & 0x01),
        "slope_hint": bool(flags & 0x02),
        "confidence": raw[7],
        "lateral_error": i16(8),
        "curve_phase": raw[10],
        "reserved": raw[11],
        "slowdown_request_permille": u16(12),
        "crc16": u16(14),
    }
    issues: List[str] = []
    if raw[0:2] != bytes((0xA5, 0x5A)):
        issues.append("sync header is not A5 5A")
    if record["version"] != 2:
        issues.append("frame version is not 2")
    if record["payload_length"] != 8:
        issues.append("payload length is not 8")
    if flags & ~0x03:
        issues.append("frame contains unknown flag bits")
    if record["reserved"] != 0:
        issues.append("reserved byte is nonzero")
    if record["crc16"] != _crc16_ccitt_false(raw[2:14]):
        issues.append("frame CRC16/CCITT-FALSE mismatch")
    if record["confidence"] > 100:
        issues.append("confidence is outside 0..100")
    if not -1000 <= record["lateral_error"] <= 1000:
        issues.append("lateral error is outside -1000..1000")
    if record["curve_phase"] > 3:
        issues.append("curve phase is reserved outside 0..3")
    if record["slowdown_request_permille"] > 1000:
        issues.append("slowdown request is outside 0..1000 permille")
    if not record["frame_valid"] and (
            flags != 0 or record["confidence"] != 0 or
            record["lateral_error"] != 0 or record["curve_phase"] != 0 or
            record["slowdown_request_permille"] != 0):
        issues.append("invalid observation has nonzero observation/control fields")
    if record["sequence"] != base["sequence"]:
        issues.append("frame sequence does not match base row")
    if record["frame_valid"] != base["valid"]:
        issues.append("frame valid flag does not match base row")
    if record["lateral_error"] != base["sent_error_norm"]:
        issues.append("frame lateral error does not match base packet error")
    return record, issues, True


def _metadata_tokens(line: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    cleaned = line.strip().lstrip("# ")
    for token in cleaned.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key and value:
            result[key.strip()] = value.strip().strip(",;")
    return result


def _parse_camera_block(lines: Sequence[str], block_number: int) -> Tuple[Dict[str, str], List[Dict[str, Any]]]:
    header_index = None
    fieldnames: List[str] = []
    for index, line in enumerate(lines):
        if not line.strip() or "=" in line and not line.lower().startswith("sequence,"):
            continue
        try:
            candidate = next(csv.reader([line]))
        except csv.Error:
            continue
        names = [name.strip() for name in candidate]
        if all(field in names for field in CAMERA_FIELDS):
            header_index = index
            fieldnames = names
            break
    if header_index is None:
        _fail("Camera trace block {} has no complete CSV header".format(block_number))

    metadata: Dict[str, str] = {}
    for line in lines[:header_index]:
        metadata.update(_metadata_tokens(line))

    csv_text = "\n".join([",".join(fieldnames)] + list(lines[header_index + 1:]))
    try:
        parsed_rows = list(csv.DictReader(csv_text.splitlines()))
    except csv.Error as exc:
        _fail("Camera trace CSV is malformed: {}".format(exc))

    records: List[Dict[str, Any]] = []
    for input_index, source in enumerate(parsed_rows):
        if not source or not any(value not in (None, "") for value in source.values()):
            continue
        if None in source:
            _fail("Camera trace row {} has more cells than the header".format(input_index + 1))
        record: Dict[str, Any] = dict(source)
        record["sequence"] = _parse_int(source.get("sequence"), "camera sequence", 0, 0xFFFF)
        record["capture_ms"] = _parse_int(source.get("capture_ms"), "camera capture_ms", 0, 0xFFFFFFFF)
        record["send_ms"] = _parse_int(source.get("send_ms"), "camera send_ms", 0, 0xFFFFFFFF)
        for name in ("valid", "full_write", "reverse_fresh"):
            record[name] = _parse_bool(source.get(name), "camera " + name)
        record["write_error"] = _parse_int(source.get("write_error"), "camera write_error")
        for name in CAMERA_FIELDS[7:]:
            record[name] = _parse_int(source.get(name), "camera " + name)
        extension, extension_issues, extension_present = _decode_control_extension_v2(
            source.get(CONTROL_EXTENSION_V2_COLUMN), record)
        record["control_ext_v2_present"] = extension_present
        record["control_ext_v2_valid"] = (
            bool(extension_present and not extension_issues)
            if extension_present else None)
        record["control_ext_v2_issues"] = extension_issues
        for name in CONTROL_EXTENSION_V2_FIELDS:
            record["control_ext_v2_" + name] = (
                extension.get(name) if extension is not None else None)
        curve_trace, curve_issues, curve_present = _decode_curve_trace_v2(
            source.get(CURVE_TRACE_V2_COLUMN), record)
        record["curve_trace_v2_present"] = curve_present
        record["curve_trace_v2_valid"] = (
            bool(curve_present and not curve_issues) if curve_present else None)
        record["curve_trace_v2_issues"] = curve_issues
        for name in CURVE_TRACE_V2_FIELDS:
            record["curve_trace_v2_" + name] = (
                curve_trace.get(name) if curve_trace is not None else None)
        record["_row_index"] = len(records)
        record["_trace_block"] = block_number
        records.append(record)
    return metadata, records


def load_camera_trace(path: str | Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    source_path = Path(path)
    try:
        text = source_path.read_text(encoding="utf-8-sig")
    except OSError as exc:
        _fail("Cannot read camera trace: {}".format(exc))
    lines = text.splitlines()
    begin_indices = [i for i, line in enumerate(lines) if "STEERING_DIAG_BEGIN" in line]
    end_indices = [i for i, line in enumerate(lines) if "STEERING_DIAG_END" in line]
    blocks: List[Sequence[str]] = []
    if begin_indices or end_indices:
        if len(begin_indices) != len(end_indices) or not begin_indices:
            _fail("Camera trace must have paired STEERING_DIAG_BEGIN/END markers")
        cursor = 0
        for begin, end in zip(begin_indices, end_indices):
            if begin >= end or begin < cursor:
                _fail("Camera trace diagnostic markers are out of order")
            blocks.append(lines[begin:end])
            cursor = end + 1
    else:
        blocks = [lines]

    combined: List[Dict[str, Any]] = []
    block_metadata: List[Dict[str, Any]] = []
    for block_number, block in enumerate(blocks):
        metadata, records = _parse_camera_block(block, block_number)
        raw_period = metadata.get("ticks_period_ms", "0")
        try:
            period = _parse_int(raw_period, "ticks_period_ms", 0, 0xFFFFFFFF)
        except AnalysisError:
            period = 0
            metadata["ticks_period_parse_error"] = str(raw_period)
        metadata["ticks_period_ms"] = period
        for record in records:
            if period > 0 and (record["capture_ms"] >= period or record["send_ms"] >= period):
                _fail("Camera timestamp is outside the declared ticks_period_ms in block {}".format(
                    block_number))
        if "record_errors" in metadata:
            metadata["record_errors"] = _parse_int(
                metadata["record_errors"], "camera record_errors", 0)
        else:
            metadata["record_errors"] = 0
        metadata["block_number"] = block_number
        if "count" in metadata:
            try:
                metadata["count_matches_rows"] = _parse_int(metadata["count"], "trace count", 0) == len(records)
            except AnalysisError:
                metadata["count_matches_rows"] = False
        block_metadata.append(metadata)
        combined.extend(records)
    if not combined:
        _fail("Camera trace contains no diagnostic rows")
    metadata_out: Dict[str, Any] = {
        "source": str(source_path.resolve()),
        "blocks": block_metadata,
        "row_count": len(combined),
        "ticks_period_ms": block_metadata[0]["ticks_period_ms"] if len(block_metadata) == 1 else None,
    }
    return metadata_out, combined


def _unwrap_camera(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    session_number = -1
    previous_block = None
    previous_sequence = None
    previous_capture = None
    previous_send = None
    sequence_unwrapped = None
    capture_unwrapped = None
    send_unwrapped = None
    session_warnings: Dict[int, List[str]] = defaultdict(list)

    for record in records:
        period = int(record["_trace_period_ms"])
        new_session = record["_trace_block"] != previous_block
        sequence_delta = None
        capture_delta = None
        send_delta = None
        if not new_session and previous_sequence is not None:
            sequence_delta = (record["sequence"] - previous_sequence) % U16_MODULUS
            if sequence_delta > U16_MODULUS // 2:
                new_session = True
                session_warnings[session_number + 1].append("sequence moved backward outside the 16-bit wrap window")
            if period > 0 and previous_capture is not None and previous_send is not None:
                capture_delta = (record["capture_ms"] - previous_capture) % period
                send_delta = (record["send_ms"] - previous_send) % period
                if capture_delta >= period // 2 or send_delta >= period // 2:
                    new_session = True
                    session_warnings[session_number + 1].append("camera clock moved backward outside the declared tick wrap window")
            elif period == 0 and (record["capture_ms"] < previous_capture or record["send_ms"] < previous_send):
                new_session = True
                session_warnings[session_number + 1].append("camera clock decreased while ticks_period_ms is unknown")

        if new_session:
            session_number += 1
            previous_sequence = None
            previous_capture = None
            previous_send = None
            sequence_unwrapped = None
            capture_unwrapped = None
            send_unwrapped = None

        if previous_sequence is None:
            sequence_unwrapped = record["sequence"]
        else:
            sequence_delta = (record["sequence"] - previous_sequence) % U16_MODULUS
            sequence_unwrapped += sequence_delta

        if period > 0:
            if previous_capture is None:
                capture_unwrapped = record["capture_ms"]
                send_unwrapped = record["send_ms"]
            else:
                capture_delta = (record["capture_ms"] - previous_capture) % period
                send_delta = (record["send_ms"] - previous_send) % period
                capture_unwrapped += capture_delta
                send_unwrapped += send_delta
            record["capture_unwrapped_ms"] = capture_unwrapped
            record["send_unwrapped_ms"] = send_unwrapped
        else:
            record["capture_unwrapped_ms"] = None
            record["send_unwrapped_ms"] = None

        record["camera_session"] = session_number
        record["sequence_unwrapped"] = sequence_unwrapped
        record["session_warnings"] = list(session_warnings.get(session_number, []))
        previous_block = record["_trace_block"]
        previous_sequence = record["sequence"]
        previous_capture = record["capture_ms"]
        previous_send = record["send_ms"]
    return records


def _load_h743(path: str | Path, status_path: str | Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    try:
        status = json.loads(Path(status_path).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as exc:
        _fail("Cannot read status JSON: {}".format(exc))
    header = status.get("runtime_log_header")
    elf = status.get("elf", {})
    if not isinstance(header, Mapping) or not isinstance(elf, Mapping):
        _fail("Status JSON must contain elf and runtime_log_header objects")
    if header.get("state_name") != "frozen":
        _fail("Status JSON does not describe a FROZEN runtime log")
    if header.get("build_id_matches_elf") is not True:
        _fail("Status JSON header build ID does not match its selected ELF")
    if header.get("header_crc_status") != "valid":
        _fail("Status JSON runtime-log header CRC is not valid")
    version = _parse_int(header.get("version"), "runtime-log version", 1, 2)
    if version not in HOLD_QUANTUM_MS:
        _fail("Unsupported runtime-log version {}".format(version))
    elf_version = _parse_int(elf.get("log_version"), "ELF log version", 1, 2)
    if elf_version != version:
        _fail("Status JSON ELF and runtime-log versions disagree")
    if int(header.get("hold_quantum_ms", HOLD_QUANTUM_MS[version])) != HOLD_QUANTUM_MS[version]:
        _fail("Status JSON hold quantum does not match the runtime-log version")

    rows: List[Dict[str, Any]] = []
    try:
        with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            missing = [name for name in H743_FIELDS if name not in (reader.fieldnames or [])]
            if missing:
                _fail("H743 CSV is missing required columns: " + ", ".join(missing))
            for index, source in enumerate(reader):
                row: Dict[str, Any] = dict(source)
                for name in ("t_ms", "t_elapsed_ms", "session_id", "vision_sequence",
                             "vision_latest_error_norm_milli", "mode", "servo_requested_us",
                             "servo_limited_us", "servo_applied_us"):
                    row[name] = _parse_int(source.get(name), "H743 " + name)
                row["vision_sequence"] = _parse_int(row["vision_sequence"], "H743 vision_sequence", 0, 0xFFFF)
                row["vision_latest_valid"] = _parse_bool(source.get("vision_latest_valid"),
                                                        "H743 vision_latest_valid")
                for name in ("vision_latest_frame_present", "vision_usable_fresh", "event_hold_enter"):
                    if name in source and source[name] not in (None, ""):
                        row[name] = _parse_bool(source[name], "H743 " + name)
                row["_row_index"] = index
                rows.append(row)
    except OSError as exc:
        _fail("Cannot read H743 CSV: {}".format(exc))
    if not rows:
        _fail("H743 CSV has no records")

    status_session = _parse_int(header.get("session_id"), "status session_id")
    start_ms = _parse_int(header.get("start_ms"), "status start_ms", 0, 0xFFFFFFFF)
    stop_ms = _parse_int(header.get("stop_ms"), "status stop_ms", 0, 0xFFFFFFFF)
    run_elapsed = (stop_ms - start_ms) & 0xFFFFFFFF
    sessions = {row["session_id"] for row in rows}
    if sessions != {status_session}:
        _fail("H743 CSV session IDs do not exactly match the status header session")
    previous_elapsed = -1
    for row in rows:
        if row["t_elapsed_ms"] < previous_elapsed:
            _fail("H743 CSV t_elapsed_ms is not monotonic within the status session")
        if row["t_elapsed_ms"] > run_elapsed:
            _fail("H743 CSV record is after the status header stop_ms")
        if row["t_ms"] != ((start_ms + row["t_elapsed_ms"]) & 0xFFFFFFFF):
            _fail("H743 CSV t_ms does not agree with status start_ms and t_elapsed_ms")
        previous_elapsed = row["t_elapsed_ms"]
    return status, rows


def _status_snapshot(status: Mapping[str, Any]) -> Dict[str, Any]:
    scalars = status.get("scalars", {})
    snapshot_names = (
        "car_vision_hold_snapshot_entry", "car_vision_hold_snapshot_error_norm",
        "car_vision_hold_snapshot_sequence", "car_vision_hold_snapshot_source_rx_ms",
        "car_vision_hold_snapshot_entered_ms", "car_vision_hold_snapshot_target_us",
        "car_vision_hold_snapshot_applied_us", "car_vision_hold_snapshot_left_permille",
        "car_vision_hold_snapshot_right_permille",
    )
    if not isinstance(scalars, Mapping) or not any(name in scalars for name in snapshot_names):
        return {"available": False, "published": None, "reason": "status JSON has no optional hold snapshot fields"}
    values = {name: scalars.get(name) for name in snapshot_names}
    entry = values["car_vision_hold_snapshot_entry"]
    if entry is None:
        return {"available": False, "published": None, "usable": False,
                "reason": "selected ELF does not expose the optional hold snapshot"}
    entry = _parse_int(entry, "hold snapshot entry", 0, 0xFFFFFFFF)
    result: Dict[str, Any] = {"available": True, "published": entry > 0,
                              "usable": False, "entry": entry}
    for name, value in values.items():
        short = name.removeprefix("car_vision_hold_snapshot_")
        if short == "entry":
            continue
        if value is None:
            result[short] = None
        else:
            result[short] = _parse_int(value, "hold snapshot " + short)
    if not result["published"]:
        result["reason"] = "snapshot entry marker is zero; no published hold-entry snapshot is available"
        return result

    issues = []
    required_values = ("error_norm", "sequence", "source_rx_ms", "entered_ms", "target_us",
                       "applied_us", "left_permille", "right_permille")
    for name in required_values:
        if result.get(name) is None:
            issues.append("{} is missing".format(name))
    for name in ("source_rx_ms", "entered_ms"):
        value = result.get(name)
        if value is not None and not 0 <= value <= 0xFFFFFFFF:
            issues.append("{} is outside the unsigned 32-bit range".format(name))
    value = result.get("sequence")
    if value is not None and not 0 <= value <= 0xFFFF:
        issues.append("sequence is outside the unsigned 16-bit range")
    value = result.get("error_norm")
    if value is not None and not -1000 <= value <= 1000:
        issues.append("error_norm is outside [-1000, 1000]")
    for name in ("target_us", "applied_us"):
        value = result.get(name)
        if value is not None and not 1100 <= value <= 2500:
            issues.append("{} is outside [1100, 2500]".format(name))
    for name in ("left_permille", "right_permille"):
        value = result.get(name)
        if value is not None and not 0 <= value <= 700:
            issues.append("{} is outside [0, 700]".format(name))
    result["usable"] = not issues
    if issues:
        result["warnings"] = issues
        result["reason"] = "published hold snapshot is incomplete or outside allowed value ranges; it was not used for attribution"
    return result


def _is_effective_send(record: Mapping[str, Any]) -> bool:
    return bool(record["valid"] and record["full_write"] and record["write_error"] == 0)


def _median_summary(values: Sequence[int | float]) -> Dict[str, Any] | None:
    if not values:
        return None
    ordered = sorted(values)
    median = statistics.median(ordered)
    deviations = [abs(value - median) for value in ordered]
    return {
        "count": len(ordered),
        "min": ordered[0],
        "median": median,
        "max": ordered[-1],
        "median_absolute_deviation": statistics.median(deviations),
    }


def _range_summary(values: Iterable[int]) -> Dict[str, Any] | None:
    ordered = sorted(values)
    if not ordered:
        return None
    return {"count": len(ordered), "min": ordered[0], "max": ordered[-1]}


def _camera_sequence_gaps(camera_rows: Sequence[Mapping[str, Any]],
                          selected_session: int | None) -> List[Dict[str, int]]:
    if selected_session is None:
        return []
    rows = sorted((row for row in camera_rows
                   if row["camera_session"] == selected_session),
                  key=lambda row: int(row["sequence_unwrapped"]))
    gaps = []
    for previous, current in zip(rows, rows[1:]):
        missing = int(current["sequence_unwrapped"]) - int(previous["sequence_unwrapped"]) - 1
        if missing > 0:
            gaps.append({
                "after_sequence": int(previous["sequence"]),
                "before_sequence": int(current["sequence"]),
                "missing_count": missing,
            })
    return gaps


def _validate_export_metadata(csv_path: str | Path, status: Mapping[str, Any],
                              h743_rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Cross-check adjacent exporter metadata when present; session/time checks are mandatory."""
    path = Path(csv_path)
    candidates = (path.parent / "decode-metadata.json", path.parent / "run-metadata.json")
    existing = [candidate for candidate in candidates if candidate.is_file()]
    if not existing:
        return {
            "checked": False,
            "status": "not_present",
            "warning": "No adjacent exporter metadata JSON was supplied; CSV/status session and time checks were still applied.",
        }

    expected_version = int(status["runtime_log_header"]["version"])
    expected_quantum = HOLD_QUANTUM_MS[expected_version]
    expected_build = str(status["elf"].get("build_id", "")).lower()
    expected_session = int(status["runtime_log_header"]["session_id"])
    found: List[str] = []
    warnings: List[str] = []
    for metadata_path in existing:
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as exc:
            _fail("Cannot read adjacent exporter metadata {}: {}".format(metadata_path, exc))
        found.append(metadata_path.name)
        if metadata_path.name == "decode-metadata.json":
            log = metadata.get("log", {})
            elf = metadata.get("elf", {})
            version = metadata.get("log_version", log.get("version"))
            quantum = metadata.get("holdQuantum_ms", log.get("hold_quantum_ms"))
            build_id = elf.get("build_id")
            session_id = log.get("session_id")
            record_count = metadata.get("record_count")
        else:
            export_status = metadata.get("status")
            if export_status not in ("success", "success_with_warnings"):
                _fail("Adjacent run-metadata.json does not report a successful CSV export")
            if export_status == "success_with_warnings":
                warnings.append(
                    "CSV export decoded successfully, but a later status snapshot failed or warned: {}".format(
                        metadata.get("diagnostic_failure") or metadata.get("error") or
                        "see adjacent run-metadata.json"))
            initial = metadata.get("initial_header", {})
            version = metadata.get("log_version", initial.get("version"))
            quantum = metadata.get("holdQuantum_ms")
            build_id = metadata.get("build_id")
            session_id = initial.get("session_id")
            record_count = metadata.get("record_count")
        if version is None or _parse_int(version, "metadata log version", 1, 2) != expected_version:
            _fail("Adjacent exporter metadata log version does not match the selected status")
        if quantum is None or _parse_int(quantum, "metadata hold quantum", 0) != expected_quantum:
            _fail("Adjacent exporter metadata hold quantum does not match the selected status")
        if build_id is None or str(build_id).lower() != expected_build:
            _fail("Adjacent exporter metadata build ID does not match the selected status")
        if session_id is None or _parse_int(session_id, "metadata session ID") != expected_session:
            _fail("Adjacent exporter metadata session ID does not match the selected status")
        if record_count is not None and _parse_int(record_count, "metadata record count", 0) != len(h743_rows):
            _fail("Adjacent exporter metadata record count does not match the supplied CSV")
    return {"checked": True, "status": "matched", "files": found, "warnings": warnings}


def analyze(camera_meta: Mapping[str, Any], camera_rows: List[Dict[str, Any]],
            status: Mapping[str, Any], h743_rows: List[Dict[str, Any]]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    for row in camera_rows:
        block = camera_meta["blocks"][row["_trace_block"]]
        row["_trace_period_ms"] = int(block["ticks_period_ms"])
    _unwrap_camera(camera_rows)

    snapshot = _status_snapshot(status)
    candidates: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for row in camera_rows:
        candidates[row["sequence"]].append(row)
    ambiguous_sequences = {sequence for sequence, occurrences in candidates.items()
                           if len(occurrences) > 1}
    camera_sessions = sorted({row["camera_session"] for row in camera_rows})
    status_header = status["runtime_log_header"]
    if status_header.get("header_crc_status") != "valid":
        _fail("Status JSON runtime-log header CRC is not valid")
    run_start_ms = _parse_int(status_header.get("start_ms"), "status start_ms", 0, 0xFFFFFFFF)
    run_stop_ms = _parse_int(status_header.get("stop_ms"), "status stop_ms", 0, 0xFFFFFFFF)
    run_elapsed_ms = (run_stop_ms - run_start_ms) & 0xFFFFFFFF

    snapshot_anchor_session = None
    snapshot_anchor_record = None
    if snapshot.get("available") and snapshot.get("published") and snapshot.get("usable"):
        snapshot_sequence = snapshot.get("sequence")
        if snapshot_sequence is not None:
            snapshot_sequence = _parse_int(snapshot_sequence, "hold snapshot sequence", 0, 0xFFFF)
            matches = candidates.get(snapshot_sequence, [])
            if len(matches) == 1 and _is_effective_send(matches[0]):
                snapshot["camera_source_match"] = "unique_valid_full_send"
                pending_camera_source = matches[0]
                snapshot["camera_error_matches_snapshot"] = int(
                    pending_camera_source["sent_error_norm"] == snapshot.get("error_norm"))
                snapshot["camera_session"] = pending_camera_source["camera_session"]
            elif len(matches) > 1:
                snapshot["camera_source_match"] = "ambiguous_duplicate_sequence"
                pending_camera_source = None
            elif matches:
                snapshot["camera_source_match"] = "sequence_was_not_a_valid_full_send"
                pending_camera_source = None
            else:
                snapshot["camera_source_match"] = "sequence_not_present_in_camera_trace"
                pending_camera_source = None

            entered_ms = snapshot.get("entered_ms")
            source_rx_ms = snapshot.get("source_rx_ms")
            entry_within = None
            source_within = None
            if entered_ms is not None:
                entered_ms = _parse_int(entered_ms, "hold snapshot entered_ms", 0, 0xFFFFFFFF)
                entered_elapsed = (int(entered_ms) - run_start_ms) & 0xFFFFFFFF
                entry_within = entered_elapsed <= run_elapsed_ms
                snapshot["entered_elapsed_ms"] = entered_elapsed
            if source_rx_ms is not None:
                source_rx_ms = _parse_int(source_rx_ms, "hold snapshot source_rx_ms", 0, 0xFFFFFFFF)
                source_elapsed = (int(source_rx_ms) - run_start_ms) & 0xFFFFFFFF
                source_within = source_elapsed <= run_elapsed_ms
                snapshot["source_rx_elapsed_ms"] = source_elapsed
            snapshot["entry_within_status_run"] = entry_within
            snapshot["source_rx_within_status_run"] = source_within

            entry_rows = [row for row in h743_rows
                          if row.get("event_hold_enter") is True]
            snapshot["h743_hold_entry_rows"] = len(entry_rows)
            entry_matches = []
            if entered_ms is not None:
                entry_elapsed = (int(entered_ms) - run_start_ms) & 0xFFFFFFFF
                entry_matches = [row for row in entry_rows
                                 if row["vision_sequence"] == snapshot_sequence and
                                 abs(row["t_elapsed_ms"] - entry_elapsed) <= 50]
            latest_entry_elapsed = max((row["t_elapsed_ms"] for row in entry_rows), default=None)
            entry_is_latest = (len(entry_matches) == 1 and
                               latest_entry_elapsed == entry_matches[0]["t_elapsed_ms"])
            snapshot["entry_matches_h743_log"] = bool(entry_is_latest)
            snapshot["entry_match_count"] = len(entry_matches)
            snapshot["snapshot_matches_latest_h743_hold"] = bool(entry_is_latest)
            if entry_is_latest:
                h_entry = entry_matches[0]
                snapshot["h743_entry_row_index"] = h_entry["_row_index"]
                snapshot["h743_entry_error_matches"] = int(
                    h_entry["vision_latest_error_norm_milli"] == snapshot.get("error_norm"))
            else:
                snapshot["h743_entry_error_matches"] = None

            source_before_entry = False
            if (snapshot.get("source_rx_elapsed_ms") is not None and
                    snapshot.get("entered_elapsed_ms") is not None and entry_within and source_within):
                source_before_entry = snapshot["source_rx_elapsed_ms"] <= snapshot["entered_elapsed_ms"]
            source_error_matches = bool(
                pending_camera_source is not None and
                snapshot.get("camera_error_matches_snapshot") == 1)
            if (pending_camera_source is not None and entry_within is True and
                    source_within is True and source_before_entry and entry_matches and
                    snapshot.get("h743_entry_error_matches") == 1 and source_error_matches):
                snapshot_anchor_record = pending_camera_source
                snapshot_anchor_session = pending_camera_source["camera_session"]
            elif pending_camera_source is not None:
                snapshot["reason"] = (
                    "hold snapshot entry time, source RX time, H743 entry record, sequence, or signed error did not all match this run")
            elif snapshot.get("reason") is None:
                snapshot["reason"] = "snapshot source sequence has no unique valid full camera send"

    selected_camera_session = None
    session_selection_reason = None
    if len(camera_sessions) == 1:
        selected_camera_session = camera_sessions[0]
        session_selection_reason = "single_monotonic_camera_session"
    elif snapshot_anchor_session is not None:
        selected_camera_session = snapshot_anchor_session
        session_selection_reason = "anchored_by_unique_hold_snapshot_sequence"
    else:
        session_selection_reason = "multiple_camera_sessions_without_a_unique_hold_snapshot_anchor"

    h_elapsed_by_seq: Dict[int, List[int]] = defaultdict(list)
    all_seen_h_sequences: List[int] = []
    for row in h743_rows:
        sequence = row["vision_sequence"]
        all_seen_h_sequences.append(sequence)
        if row["vision_latest_valid"]:
            h_elapsed_by_seq[sequence].append(row["t_elapsed_ms"])

    matched_rows: List[Dict[str, Any]] = []
    unmatched_camera: set[int] = set()
    unmatched_h743: set[int] = set()
    ambiguous_h743: set[int] = set()
    if selected_camera_session is not None:
        for hrow in h743_rows:
            if not hrow["vision_latest_valid"]:
                continue
            sequence = hrow["vision_sequence"]
            occurrences = candidates.get(sequence, [])
            if len(occurrences) > 1:
                ambiguous_h743.add(sequence)
                continue
            if not occurrences or occurrences[0]["camera_session"] != selected_camera_session:
                unmatched_h743.add(sequence)
                continue
            camera = occurrences[0]
            if not _is_effective_send(camera):
                unmatched_h743.add(sequence)
                continue
            matched_record = {
                "camera_session": camera["camera_session"],
                "camera_row_index": camera["_row_index"],
                "sequence": sequence,
                "sequence_unwrapped": camera["sequence_unwrapped"],
                "camera_capture_ms": camera["capture_ms"],
                "camera_send_ms": camera["send_ms"],
                "camera_send_unwrapped_ms": camera["send_unwrapped_ms"],
                "camera_valid": int(camera["valid"]),
                "camera_full_write": int(camera["full_write"]),
                "camera_write_error": camera["write_error"],
                "camera_reverse_fresh": int(camera["reverse_fresh"]),
                "position_norm": camera["position_norm"],
                "preview_candidate_norm": camera["preview_candidate_norm"],
                "preview_applied_norm": camera["preview_applied_norm"],
                "sent_error_norm": camera["sent_error_norm"],
                "raw_control_norm": camera["raw_control_norm"],
                "filtered_error_norm": camera["filtered_error_norm"],
                "reference_error_norm": camera["reference_error_norm"],
                "h743_row_index": hrow["_row_index"],
                "h743_t_ms": hrow["t_ms"],
                "h743_t_elapsed_ms": hrow["t_elapsed_ms"],
                "h743_mode": hrow["mode"],
                "h743_latest_error_norm": hrow["vision_latest_error_norm_milli"],
                "error_matches": int(hrow["vision_latest_error_norm_milli"] == camera["sent_error_norm"]),
                "servo_requested_us": hrow["servo_requested_us"],
                "servo_limited_us": hrow["servo_limited_us"],
                "servo_applied_us": hrow["servo_applied_us"],
                "control_ext_v2_present": camera.get("control_ext_v2_present", False),
                "control_ext_v2_valid": camera.get("control_ext_v2_valid"),
                "control_ext_v2_issues": "; ".join(
                    camera.get("control_ext_v2_issues", [])),
                "curve_trace_v2_present": camera.get("curve_trace_v2_present", False),
                "curve_trace_v2_valid": camera.get("curve_trace_v2_valid"),
                "curve_trace_v2_issues": "; ".join(
                    camera.get("curve_trace_v2_issues", [])),
            }
            for extension_field in CONTROL_EXTENSION_V2_FIELDS:
                matched_record["control_ext_v2_" + extension_field] = camera.get(
                    "control_ext_v2_" + extension_field)
            for trace_field in CURVE_TRACE_V2_FIELDS:
                matched_record["curve_trace_v2_" + trace_field] = camera.get(
                    "curve_trace_v2_" + trace_field)
            matched_rows.append(matched_record)

    matched_seq_to_camera = {row["sequence"]: row for row in matched_rows}
    matched_send_sequences = sorted(matched_seq_to_camera)
    for row in camera_rows:
        if (selected_camera_session is not None and row["camera_session"] == selected_camera_session
                and _is_effective_send(row) and row["sequence"] not in matched_seq_to_camera):
            unmatched_camera.add(row["sequence"])

    if snapshot.get("available") and snapshot.get("published"):
        if snapshot_anchor_record is not None:
            snapshot["camera_error_matches_snapshot"] = int(
                snapshot_anchor_record["sent_error_norm"] == snapshot.get("error_norm"))
            snapshot["camera_source_valid_full_send"] = True
        elif snapshot.get("camera_source_match"):
            snapshot["camera_source_valid_full_send"] = False
            snapshot["camera_error_matches_snapshot"] = None
            if snapshot.get("reason") is None:
                snapshot["reason"] = "snapshot source sequence has no unique valid full camera send"

    camera_effective = [row for row in camera_rows if _is_effective_send(row)]
    valid_candidates = [row for row in camera_effective
                        if selected_camera_session is None or row["camera_session"] == selected_camera_session]
    unique_valid = [row for row in valid_candidates if len(candidates[row["sequence"]]) == 1]
    control_extension_rows = [row for row in camera_rows
                              if row.get("control_ext_v2_present")]
    control_extension_invalid = [row for row in control_extension_rows
                                 if not row.get("control_ext_v2_valid")]
    control_extension_missing = len(camera_rows) - len(control_extension_rows)
    control_extension_valid = [row for row in control_extension_rows
                               if row.get("control_ext_v2_valid")]
    if not control_extension_rows:
        control_extension_status = "legacy_absent"
    elif control_extension_invalid:
        control_extension_status = "invalid"
    elif control_extension_missing:
        control_extension_status = "partial"
    else:
        control_extension_status = "valid"
    unique_valid_control_extension = [
        row for row in unique_valid if row.get("control_ext_v2_valid")]
    control_extension_report = {
        "status": control_extension_status,
        "record_count": len(camera_rows),
        "present_records": len(control_extension_rows),
        "valid_records": len(control_extension_valid),
        "invalid_records": len(control_extension_invalid),
        "missing_records": control_extension_missing,
        "invalid_rows": [{
            "row_index": row["_row_index"],
            "issues": row.get("control_ext_v2_issues", []),
        } for row in control_extension_invalid],
        "crc": "CRC16/CCITT-FALSE over bytes 0..29, stored little-endian at bytes 30..31",
        "ranges": {
            field: _range_summary(
                int(row["control_ext_v2_" + field])
                for row in unique_valid_control_extension
                if row.get("control_ext_v2_" + field) is not None)
            for field in CONTROL_EXTENSION_V2_STAGE_FIELDS
        },
        "range_basis": "Unique effective camera sends with a valid, base-row-matched V2 extension.",
    }
    curve_trace_rows = [row for row in camera_rows
                        if row.get("curve_trace_v2_present")]
    curve_trace_invalid = [row for row in curve_trace_rows
                           if not row.get("curve_trace_v2_valid")]
    curve_trace_missing = len(camera_rows) - len(curve_trace_rows)
    curve_trace_valid = [row for row in curve_trace_rows
                         if row.get("curve_trace_v2_valid")]
    if not curve_trace_rows:
        curve_trace_status = "legacy_absent"
    elif curve_trace_invalid:
        curve_trace_status = "invalid"
    elif curve_trace_missing:
        curve_trace_status = "partial"
    else:
        curve_trace_status = "valid"
    curve_trace_report = {
        "status": curve_trace_status,
        "record_count": len(camera_rows),
        "present_records": len(curve_trace_rows),
        "valid_records": len(curve_trace_valid),
        "invalid_records": len(curve_trace_invalid),
        "missing_records": curve_trace_missing,
        "invalid_rows": [{
            "row_index": row["_row_index"],
            "issues": row.get("curve_trace_v2_issues", []),
        } for row in curve_trace_invalid],
        "layout": "actual 16-byte UART V2 frame: A5 5A, version, payload length, sequence LE, flags, confidence, signed lateral error LE, curve phase, reserved zero, slowdown request permille LE, CRC-16/CCITT-FALSE over bytes 2..13 stored LE",
        "sequence_binding": "The frame sequence, valid flag, and lateral error are checked against the same CSV row. Capture/send timestamps come from the CSV row; the wire frame contains no timestamp.",
        "curve_phase_counts": dict(Counter(
            int(row["curve_trace_v2_curve_phase"])
            for row in curve_trace_valid)),
        "slowdown_request_permille": _range_summary(
            int(row["curve_trace_v2_slowdown_request_permille"])
            for row in curve_trace_valid),
    }
    stage_fields = (
        "position_norm", "preview_candidate_norm", "preview_applied_norm", "sent_error_norm",
        "raw_control_norm", "filtered_error_norm", "reference_error_norm",
    )
    stage_summary = {name: _range_summary(row[name] for row in unique_valid) for name in stage_fields}
    composition = {
        "candidate_minus_position": _range_summary(
            row["preview_candidate_norm"] - row["position_norm"] for row in unique_valid),
        "applied_minus_candidate": _range_summary(
            row["preview_applied_norm"] - row["preview_candidate_norm"] for row in unique_valid),
        "sent_minus_applied_preview": _range_summary(
            row["sent_error_norm"] - row["preview_applied_norm"] for row in unique_valid),
        "raw_minus_sent": _range_summary(
            row["raw_control_norm"] - row["sent_error_norm"] for row in unique_valid),
        "filtered_minus_raw": _range_summary(
            row["filtered_error_norm"] - row["raw_control_norm"] for row in unique_valid),
        "reference_minus_filtered": _range_summary(
            row["reference_error_norm"] - row["filtered_error_norm"] for row in unique_valid),
        "units_note": "Differences retain recorded integer units; they are descriptive and are not gain estimates.",
    }

    request_limited = [row for row in matched_rows
                       if row["servo_requested_us"] != row["servo_limited_us"]]
    limited_applied = [row for row in matched_rows
                       if row["servo_limited_us"] != row["servo_applied_us"]]
    pwm_summary = {
        name: _range_summary(row[name] for row in matched_rows)
        for name in ("servo_requested_us", "servo_limited_us", "servo_applied_us")
    }
    response_constraints = {
        "matched_h743_rows": len(matched_rows),
        "request_limited_difference_rows": len(request_limited),
        "limited_applied_difference_rows": len(limited_applied),
        "applied_zero_rows": sum(row["servo_applied_us"] == 0 for row in matched_rows),
        "pwm_ranges_us": pwm_summary,
        "interpretation_limit": "These are software pulse-width values; they do not measure mechanical steering angle.",
    }

    trace_record_errors = sum(
        _parse_int(block.get("record_errors", 0), "camera record_errors", 0)
        for block in camera_meta["blocks"])
    metadata_mismatches = [block.get("block_number") for block in camera_meta["blocks"]
                          if block.get("count_matches_rows") is False]
    if trace_record_errors:
        trace_warnings = ["camera diagnostic record parsing reported errors; evidence is incomplete"]
    else:
        trace_warnings = []
    if metadata_mismatches:
        trace_warnings.append("camera diagnostic metadata count does not match parsed CSV rows")
    if control_extension_invalid:
        trace_warnings.append("camera control extension V2 has CRC, schema, or base-row binding errors")
    if control_extension_rows and control_extension_missing:
        trace_warnings.append("camera control extension V2 is missing from some rows")
    if curve_trace_invalid:
        trace_warnings.append("camera curve trace V2 has CRC, schema, range, or base-row binding errors")
    if curve_trace_rows and curve_trace_missing:
        trace_warnings.append("camera curve trace V2 is missing from some rows")
    if snapshot.get("published") and not snapshot.get("usable"):
        trace_warnings.append("published hold snapshot values are incomplete or outside allowed ranges; hold attribution was refused")
    analysis_status = (
        "incomplete_camera_trace" if trace_record_errors or metadata_mismatches or
        control_extension_invalid or (control_extension_rows and control_extension_missing) or
        curve_trace_invalid or (curve_trace_rows and curve_trace_missing) else
        "no_matched_sequences" if not matched_rows else
        "complete" if selected_camera_session is not None else
        "insufficient_session_evidence")
    report: Dict[str, Any] = {
        "analysis": "offline_openmv_h743_steering_chain",
        "status": analysis_status,
        "inputs": {
            "camera_trace": camera_meta["source"],
            "h743_csv": None,
            "h743_status": status.get("elf", {}).get("path"),
        },
        "runtime_log": {
            "version": int(status_header["version"]),
            "hold_quantum_ms": HOLD_QUANTUM_MS[int(status_header["version"])],
            "period_ms": int(status_header["period_ms"]),
            "session_id": int(status_header["session_id"]),
            "record_count": len(h743_rows),
        },
        "camera_trace": {
            "row_count": len(camera_rows),
            "session_count": len(camera_sessions),
            "selected_session": selected_camera_session,
            "session_selection_reason": session_selection_reason,
            "sessions": [{
                "session": session,
                "record_count": sum(row["camera_session"] == session for row in camera_rows),
                "ticks_period_ms": next(row["_trace_period_ms"] for row in camera_rows
                                         if row["camera_session"] == session),
            } for session in camera_sessions],
            "metadata": camera_meta["blocks"],
            "record_errors": trace_record_errors,
            "metadata_count_mismatches": metadata_mismatches,
            "evidence_warnings": trace_warnings,
            "duplicate_or_cross_session_sequences": sorted(ambiguous_sequences),
            "control_extension_v2": control_extension_report,
            "curve_trace_v2": curve_trace_report,
        },
        "sequence_comparison": {
            "camera_effective_full_sends": len(camera_effective),
            "camera_unique_effective_sends_in_selected_session": len(unique_valid),
            "matched_h743_rows": len(matched_rows),
            "matched_unique_sequences": len(matched_send_sequences),
            "matched_sequences": matched_send_sequences,
            "unmatched_camera_sequences": sorted(unmatched_camera),
            "unmatched_h743_sequences": sorted(unmatched_h743),
            "ambiguous_h743_sequences_refused": sorted(ambiguous_h743),
            "h743_invalid_latest_rows_ignored": sum(not row["vision_latest_valid"] for row in h743_rows),
            "camera_missing_sequence_gaps": _camera_sequence_gaps(camera_rows, selected_camera_session),
            "h743_sequence_wrap_observed": any(
                ((right - left) % U16_MODULUS) <= U16_MODULUS // 2 and right < left
                for left, right in zip(all_seen_h_sequences, all_seen_h_sequences[1:])),
            "matching_rule": "Only a unique camera sequence from a valid, full UART write with write_error=0 can pair to an H743 latest-valid sequence. Invalid rows and ambiguous sequences are excluded.",
        },
        "clock_alignment": {
            "camera_ticks_period_ms": camera_meta.get("ticks_period_ms"),
            "h743_tick_period_ms": U32_MODULUS,
            "estimate": None,
            "uncertainty": "Camera ticks and H743 HAL u32 milliseconds have independent epochs. Sequence matches do not establish a clock offset, so no cross-clock subtraction is performed.",
            "camera_unwrap_used": bool(selected_camera_session is not None and
                                        any(row["_trace_period_ms"] > 0 for row in camera_rows)),
        },
        "steering_command_stages": {
            "ranges": stage_summary,
            "observed_differences": composition,
            "basis": "Camera diagnostics contain normalized integer fields; no gain or steering-angle conversion is inferred.",
        },
        "h743_response_and_constraints": response_constraints,
        "hold_snapshot": snapshot,
        "limits": [
            "The hold snapshot describes only the most recently published hold entry and is matched to one unique, time-near, latest H743 hold-entry record.",
            "The trace and H743 clocks are independent; matching sequence numbers do not provide a clock offset.",
            "PWM pulse widths describe software commands and applied outputs, not measured mechanical wheel angle.",
            "This offline analysis does not establish physical steering response, best gains, or vehicle behavior.",
        ],
    }
    return report, matched_rows


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n",
                                         dir=str(path.parent), prefix=path.name + ".",
                                         suffix=".tmp", delete=False) as stream:
            temp_path = Path(stream.name)
            stream.write(text)
        temp_path.replace(path)
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def _write_matched_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = list(rows[0]) if rows else [
        "camera_session", "camera_row_index", "sequence", "sequence_unwrapped",
        "camera_capture_ms", "camera_send_ms", "camera_send_unwrapped_ms",
        "camera_valid", "camera_full_write", "camera_write_error", "camera_reverse_fresh",
        "position_norm", "preview_candidate_norm", "preview_applied_norm", "sent_error_norm",
        "raw_control_norm", "filtered_error_norm", "reference_error_norm",
        "h743_row_index", "h743_t_ms", "h743_t_elapsed_ms", "h743_mode",
        "h743_latest_error_norm", "error_matches", "servo_requested_us",
        "servo_limited_us", "servo_applied_us",
        "control_ext_v2_present", "control_ext_v2_valid",
        "control_ext_v2_issues",
        *("control_ext_v2_" + name for name in CONTROL_EXTENSION_V2_FIELDS),
        "curve_trace_v2_present", "curve_trace_v2_valid",
        "curve_trace_v2_issues",
        *("curve_trace_v2_" + name for name in CURVE_TRACE_V2_FIELDS),
    ]
    from io import StringIO
    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
    writer.writeheader()
    writer.writerows(rows)
    _atomic_write(path, stream.getvalue())


def _render_markdown(report: Mapping[str, Any]) -> str:
    comparison = report["sequence_comparison"]
    timing = report["clock_alignment"]
    constraints = report["h743_response_and_constraints"]
    snapshot = report["hold_snapshot"]
    lines = [
        "# Offline steering-chain comparison",
        "",
        "Status: **{}**".format(report["status"]),
        "",
        "The match uses a unique OpenMV sequence with `valid=1`, `full_write=1`, and `write_error=0`, paired with an H743 latest-valid sequence. Ambiguous sequence numbers and invalid latest-frame rows are excluded.",
        "",
        "## Sequence evidence",
        "",
        "- Matched H743 rows: {}".format(comparison["matched_h743_rows"]),
        "- Unique matched sequences: {}".format(comparison["matched_unique_sequences"]),
        "- Camera effective full sends: {}".format(comparison["camera_effective_full_sends"]),
        "- H743 invalid latest rows ignored: {}".format(comparison["h743_invalid_latest_rows_ignored"]),
        "- Ambiguous sequences refused: {}".format(report["camera_trace"]["duplicate_or_cross_session_sequences"]),
        "- Camera record errors reported: {}".format(report["camera_trace"]["record_errors"]),
        "",
        "## Clock alignment",
        "",
        "Camera `ticks_period_ms`: {}. Clock offset estimate: unavailable. {}".format(
            timing["camera_ticks_period_ms"], timing["uncertainty"]),
        "",
        "## Command and output stages",
        "",
        "Observed normalized camera field ranges: `{}`.".format(
            json.dumps(report["steering_command_stages"]["ranges"], sort_keys=True)),
        "",
        "Observed signed differences between recorded stages: `{}`. These retain recorded integer units and do not estimate a gain.".format(
            json.dumps(report["steering_command_stages"]["observed_differences"], sort_keys=True)),
        "",
        "Camera control extension V2: **{}** ({} valid, {} invalid, {} absent of {}). Per-frame P, D, P+D, preview, and final sent values are retained in the matched CSV when the extension passes its CRC and base-row binding checks.".format(
            report["camera_trace"]["control_extension_v2"]["status"],
            report["camera_trace"]["control_extension_v2"]["valid_records"],
            report["camera_trace"]["control_extension_v2"]["invalid_records"],
            report["camera_trace"]["control_extension_v2"]["missing_records"],
            report["camera_trace"]["control_extension_v2"]["record_count"]),
        "",
        "Camera curve trace V2: **{}** ({} valid, {} invalid, {} absent of {}). The matched CSV retains decoded phase/request fields after CRC and same-row sequence/valid/error checks. Wire bytes carry no timestamp; row capture/send values remain the only timing evidence.".format(
            report["camera_trace"]["curve_trace_v2"]["status"],
            report["camera_trace"]["curve_trace_v2"]["valid_records"],
            report["camera_trace"]["curve_trace_v2"]["invalid_records"],
            report["camera_trace"]["curve_trace_v2"]["missing_records"],
            report["camera_trace"]["curve_trace_v2"]["record_count"]),
        "",
        "Servo request/limit/applied differences: {} request-to-limited, {} limited-to-applied across {} matched H743 rows. Pulse-width values are software data, not mechanical angle measurements.".format(
            constraints["request_limited_difference_rows"], constraints["limited_applied_difference_rows"],
            constraints["matched_h743_rows"]),
        "",
        "## Latest hold-entry snapshot",
        "",
        "`{}`".format(json.dumps(snapshot, sort_keys=True)),
        "",
        "The snapshot covers only the most recently published entry. Invalid camera records whose error field is zero are not treated as a centered steering target.",
        "",
        "## Limits",
        "",
    ]
    lines.extend("- " + item for item in report["limits"])
    lines.extend(["", "Artifacts: `steering_chain_report.json` and `steering_chain_matched.csv`.", ""])
    return "\n".join(lines)


def run(args: argparse.Namespace) -> Dict[str, Any]:
    camera_meta, camera_rows = load_camera_trace(args.camera_trace)
    status, h743_rows = _load_h743(args.h743_csv, args.status)
    metadata_report = _validate_export_metadata(args.h743_csv, status, h743_rows)
    report, matched_rows = analyze(camera_meta, camera_rows, status, h743_rows)
    report["h743_export_metadata"] = metadata_report
    report["inputs"]["h743_csv"] = str(Path(args.h743_csv).resolve())
    report["inputs"]["h743_status_json"] = str(Path(args.status).resolve())
    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    _atomic_write(out_dir / "steering_chain_report.json",
                  json.dumps(report, indent=2, sort_keys=True) + "\n")
    _atomic_write(out_dir / "steering_chain_report.md", _render_markdown(report))
    _write_matched_csv(out_dir / "steering_chain_matched.csv", matched_rows)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-trace", required=True)
    parser.add_argument("--h743-csv", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--out", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = run(args)
        print(json.dumps({
            "status": report["status"],
            "matched_h743_rows": report["sequence_comparison"]["matched_h743_rows"],
            "matched_unique_sequences": report["sequence_comparison"]["matched_unique_sequences"],
            "report_json": str(Path(args.out).resolve() / "steering_chain_report.json"),
            "report_markdown": str(Path(args.out).resolve() / "steering_chain_report.md"),
        }, sort_keys=True))
        return 0 if report["status"] == "complete" else 2
    except (AnalysisError, OSError, csv.Error, json.JSONDecodeError) as exc:
        print("analyze-steering-chain: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
