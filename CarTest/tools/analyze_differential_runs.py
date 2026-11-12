#!/usr/bin/env python3
"""Compare three fixed Follow differential profiles using exported RAM logs.

This tool reads a manual trial manifest, the successful export CSV/metadata for
each listed run, and an ARTIFACTS.json profile whitelist. It never connects to a
board, starts a motor, writes firmware, or modifies the run CSVs.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

try:
    from runtime_log_export import ExportError, decode_buffer, inspect_elf
except ImportError:  # pragma: no cover - supports import-by-file test runners
    from tools.runtime_log_export import ExportError, decode_buffer, inspect_elf  # type: ignore


PROFILE_DIFF_PERMILLE = {"none": 0, "medium": 50, "strong": 100}
HARD_FAULT_CODES = {1, 2, 11, 12, 13, 14, 15}
COMM_FAULT_CODES = {3, 4, 5, 6, 7}
FAULT_NAMES = {
    0: "NONE",
    1: "WATCHDOG_RESET",
    2: "LOOP_GAP",
    3: "UART",
    4: "RX_OVERFLOW",
    5: "RX_STALE",
    6: "PROTOCOL",
    7: "SEQUENCE",
    8: "VISION_INVALID",
    9: "VISION_TIMEOUT",
    10: "SWITCH_RELEASE",
    11: "ENCODER_INIT",
    12: "CLOCK",
    13: "TIMER",
    14: "CONFIG",
    15: "WATCHDOG_INIT",
}
MODE_FAULT_LATCHED = 8
MODE_FOLLOW_RUNNING = 10
MODE_FOLLOW_VISION_HOLD = 12
STATE_FROZEN = 2
U32_MASK = 0xFFFFFFFF
REQUIRED_CSV_FIELDS = {
    "t_ms",
    "t_elapsed_ms",
    "session_id",
    "servo_requested_us",
    "servo_limited_us",
    "servo_applied_us",
    "motor_requested_permille",
    "motor_left_applied_permille",
    "motor_right_applied_permille",
    "vision_latest_error_norm_milli",
    "vision_latest_valid",
    "vision_usable_fresh",
    "vision_timeout",
    "vision_hold_active",
    "vision_hold_expired",
    "mode",
    "fault",
    "event_start",
    "event_stop",
    "event_fault",
    "event_hold_deadline",
    "event_button_start",
    "event_button_stop",
    "tim3_ccr1_counts",
    "tim3_ccr2_counts",
    "tim3_ccr3_counts",
    "tim3_ccr4_counts",
}
INTEGER_CSV_FIELDS = REQUIRED_CSV_FIELDS | {
    "encoder_left_count",
    "encoder_right_count",
    "vision_confidence_percent",
    "event_hold_enter",
    "event_hold_exit",
    "event_auto_recovery",
}
EVENT_FIELDS = {
    "event_start",
    "event_stop",
    "event_fault",
    "event_hold_deadline",
    "event_button_start",
    "event_button_stop",
}
OUTPUT_ZERO_FIELDS = (
    "servo_applied_us",
    "motor_left_applied_permille",
    "motor_right_applied_permille",
    "tim3_ccr1_counts",
    "tim3_ccr2_counts",
    "tim3_ccr3_counts",
    "tim3_ccr4_counts",
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$", re.IGNORECASE)
BUILD_ID_RE = re.compile(r"^[0-9a-f]{32}$", re.IGNORECASE)
FLOAT_TIE_TOLERANCE = 1e-6
EXPECTED_REQUEST_PERMILLE = 500
MOTOR_HARD_CAP_PERMILLE = 700
MIN_SESSIONS_PER_PROFILE = 3


class AnalysisError(Exception):
    """Invalid analysis inputs or an unsupported schema."""


@dataclass(frozen=True)
class Artifact:
    variant: str
    diff_permille: int
    elf_sha256: str
    build_id: str
    elf_path: Optional[Path]
    servo_center_us: Optional[int]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AnalysisError(f"Cannot read JSON {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise AnalysisError(f"JSON root must be an object: {path}")
    return data


def _require_nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise AnalysisError(f"{name} must be a non-empty string")
    return value.strip()


def _require_bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise AnalysisError(f"{name} must be an explicit JSON boolean")
    return value


def _require_finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AnalysisError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise AnalysisError(f"{name} must be finite")
    return result


def _resolve_path(value: Any, base: Path, name: str) -> Path:
    raw = _require_nonempty_string(value, name)
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def load_artifacts(path: Path, dataset_type: str) -> dict[str, Artifact]:
    raw = _read_json(path)
    if raw.get("schema_version") != 1:
        raise AnalysisError("ARTIFACTS.json schema_version must be 1")
    profiles = raw.get("profiles")
    if isinstance(profiles, dict):
        turn_profile_present = any(
            "turn" in str(name).lower() or
            (isinstance(profile, dict) and profile.get("diff_permille") == 200)
            for name, profile in profiles.items()
        )
        if turn_profile_present:
            raise AnalysisError(
                "The best-of-three differential analyzer does not support "
                "DebugFollowTurn (d=200, 70/30 targets, 60us servo and "
                "20-permille motor steps). Use a turn-response-specific "
                "analysis; do not score it with the legacy 0/50/100 thresholds."
            )
    if not isinstance(profiles, dict) or set(profiles) != set(PROFILE_DIFF_PERMILLE):
        raise AnalysisError("ARTIFACTS.json must contain exactly none, medium, and strong profiles")

    artifacts: dict[str, Artifact] = {}
    seen_hashes: set[str] = set()
    seen_build_ids: set[str] = set()
    for variant, expected_diff in PROFILE_DIFF_PERMILLE.items():
        profile = profiles.get(variant)
        if not isinstance(profile, dict):
            raise AnalysisError(f"ARTIFACTS profile {variant} must be an object")
        if profile.get("variant") != variant:
            raise AnalysisError(f"ARTIFACTS profile {variant} has a mismatched variant field")
        if profile.get("diff_permille") != expected_diff:
            raise AnalysisError(
                f"ARTIFACTS profile {variant} diff_permille must be {expected_diff}"
            )
        sha = _require_nonempty_string(profile.get("elf_sha256"), f"{variant}.elf_sha256").lower()
        build_id = _require_nonempty_string(profile.get("build_id"), f"{variant}.build_id").lower()
        if not SHA256_RE.fullmatch(sha):
            raise AnalysisError(f"ARTIFACTS {variant}.elf_sha256 must be 64 hexadecimal characters")
        if not BUILD_ID_RE.fullmatch(build_id):
            raise AnalysisError(f"ARTIFACTS {variant}.build_id must be 32 hexadecimal characters")
        if sha in seen_hashes or build_id in seen_build_ids:
            raise AnalysisError("ARTIFACTS profiles must have unique ELF SHA-256 and build IDs")
        seen_hashes.add(sha)
        seen_build_ids.add(build_id)

        elf_path: Optional[Path] = None
        if profile.get("elf_path"):
            elf_path = _resolve_path(profile["elf_path"], path.parent, f"{variant}.elf_path")
        elif dataset_type == "real":
            raise AnalysisError(f"Real-data ARTIFACTS profile {variant} must include elf_path")

        center = profile.get("servo_center_us")
        if center is not None:
            if isinstance(center, bool) or not isinstance(center, int) or not (1 <= center <= 65535):
                raise AnalysisError(f"ARTIFACTS {variant}.servo_center_us must be an integer in 1..65535")

        artifact = Artifact(variant, expected_diff, sha, build_id, elf_path, center)
        if elf_path is not None:
            if not elf_path.is_file():
                raise AnalysisError(f"ARTIFACTS {variant}.elf_path does not exist: {elf_path}")
            try:
                layout = inspect_elf(elf_path)
            except (ExportError, OSError) as exc:
                raise AnalysisError(f"Cannot inspect ARTIFACTS ELF for {variant}: {exc}") from exc
            if layout.sha256.lower() != sha or layout.build_id.hex().lower() != build_id:
                raise AnalysisError(
                    f"ARTIFACTS ELF bytes do not match the listed SHA/build ID for {variant}"
                )
        artifacts[variant] = artifact
    return artifacts


def _validate_conditions(entry: dict[str, Any]) -> dict[str, Any]:
    layout_id = _require_nonempty_string(entry.get("layout_id"), "layout_id")
    camera_build_id = _require_nonempty_string(entry.get("camera_build_id"), "camera_build_id")
    battery_voltage_v = _require_finite_number(entry.get("battery_voltage_v"), "battery_voltage_v")
    if battery_voltage_v <= 0:
        raise AnalysisError("battery_voltage_v must be greater than zero")
    battery_state = _require_nonempty_string(entry.get("battery_state"), "battery_state")
    lighting_id = _require_nonempty_string(entry.get("lighting_id"), "lighting_id")
    lighting_notes = _require_nonempty_string(entry.get("lighting_notes"), "lighting_notes")
    return {
        "layout_id": layout_id,
        "camera_build_id": camera_build_id,
        "battery_voltage_v": battery_voltage_v,
        "battery_state": battery_state,
        "lighting_id": lighting_id,
        "lighting_notes": lighting_notes,
    }


def _resolve_run_paths(path_value: Any, manifest_dir: Path) -> tuple[Path, Path]:
    run_path = _resolve_path(path_value, manifest_dir, "path")
    if run_path.is_dir():
        return run_path / "run.csv", run_path / "run-metadata.json"
    if run_path.name.lower() == "run.csv":
        return run_path, run_path.with_name("run-metadata.json")
    raise AnalysisError("path must be an export directory or its run.csv file")


def _parse_int(value: Any, field: str, row_number: int) -> int:
    if value is None or (isinstance(value, str) and not value.strip()):
        raise AnalysisError(f"CSV row {row_number} has an empty required value for {field}")
    try:
        number = int(str(value).strip(), 10)
    except (TypeError, ValueError) as exc:
        raise AnalysisError(f"CSV row {row_number} has a non-integer {field}") from exc
    return number


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, int]]]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames
            if not fields or any(not isinstance(field, str) or not field.strip() for field in fields):
                raise AnalysisError(f"CSV has no valid header: {path}")
            if len(set(fields)) != len(fields):
                raise AnalysisError(f"CSV has duplicate header names: {path}")
            missing = sorted(REQUIRED_CSV_FIELDS - set(fields))
            if missing:
                raise AnalysisError("CSV is missing critical columns: " + ", ".join(missing))
            rows: list[dict[str, int]] = []
            for row_number, raw in enumerate(reader, start=2):
                if None in raw:
                    raise AnalysisError(f"CSV row {row_number} has more cells than header fields")
                row: dict[str, int] = {}
                for field in fields:
                    value = raw.get(field)
                    if field in INTEGER_CSV_FIELDS or field in REQUIRED_CSV_FIELDS:
                        row[field] = _parse_int(value, field, row_number)
                rows.append(row)
    except AnalysisError:
        raise
    except (OSError, UnicodeError, csv.Error) as exc:
        raise AnalysisError(f"Cannot read valid CSV {path}: {exc}") from exc
    if len(rows) < 2:
        raise AnalysisError("CSV must contain at least two records")
    return list(fields), rows


def _metadata_error(metadata: dict[str, Any]) -> Optional[str]:
    if metadata.get("status") != "success":
        state = metadata.get("status", "missing status")
        stage = metadata.get("stage", "unknown stage")
        detail = metadata.get("error")
        if isinstance(detail, str) and "DEV_CONNECT_ERR" in detail:
            return (
                f"failed export at {stage}: DEV_CONNECT_ERR; no usable RAM header/payload was read; "
                "the run is not eligible"
            )
        return f"export status is {state} at {stage}; only successful exports are eligible"
    if metadata.get("stage") != "complete":
        return "successful export metadata stage is not complete"
    return None


def _validate_export(
    csv_path: Path,
    metadata_path: Path,
    artifact: Artifact,
    dataset_type: str,
    synthetic_decoder: Optional[Callable[..., Any]] = None,
) -> tuple[dict[str, Any], list[str], list[dict[str, int]], dict[str, Any]]:
    if not metadata_path.is_file():
        raise AnalysisError(f"missing current export metadata file: {metadata_path.name}")
    metadata = _read_json(metadata_path)
    failure = _metadata_error(metadata)
    if failure:
        raise AnalysisError(failure)
    if not csv_path.is_file():
        raise AnalysisError(f"successful export is missing run.csv: {csv_path}")
    sha = str(metadata.get("elf_sha256", "")).lower()
    build_id = str(metadata.get("build_id", "")).lower()
    if sha != artifact.elf_sha256 or build_id != artifact.build_id:
        raise AnalysisError(
            f"ELF profile/hash mismatch: expected {artifact.variant} sha={artifact.elf_sha256} "
            f"build={artifact.build_id}; metadata has sha={sha or 'missing'} build={build_id or 'missing'}"
        )
    if not SHA256_RE.fullmatch(sha) or not BUILD_ID_RE.fullmatch(build_id):
        raise AnalysisError("metadata ELF SHA/build ID format is invalid")

    header = metadata.get("initial_header")
    if not isinstance(header, dict):
        raise AnalysisError("metadata initial_header is null or missing")
    if header.get("state") != STATE_FROZEN or str(header.get("state_name", "")).lower() != "frozen":
        raise AnalysisError("metadata header is not FROZEN state=2")
    if str(header.get("build_id", "")).lower() != build_id:
        raise AnalysisError("metadata header build ID does not match top-level ELF build ID")
    if metadata.get("record_count") != header.get("count"):
        raise AnalysisError("metadata record_count disagrees with frozen header count")
    if metadata.get("dropped_count") != header.get("dropped_count"):
        raise AnalysisError("metadata dropped_count disagrees with frozen header")
    if metadata.get("buffer_full") != header.get("buffer_full"):
        raise AnalysisError("metadata buffer_full disagrees with frozen header")
    if metadata.get("records_truncated") != header.get("records_truncated"):
        raise AnalysisError("metadata records_truncated disagrees with frozen header")
    raw_sha = metadata.get("raw_sha256")
    if not isinstance(raw_sha, str) or not SHA256_RE.fullmatch(raw_sha):
        raise AnalysisError("metadata raw_sha256 is null or malformed")
    raw_path = csv_path.parent / "runtime-log.bin"
    if not raw_path.is_file():
        raise AnalysisError("successful export is missing runtime-log.bin")
    actual_raw_sha = hashlib.sha256(raw_path.read_bytes()).hexdigest()
    if actual_raw_sha.lower() != raw_sha.lower():
        raise AnalysisError("runtime-log.bin SHA-256 disagrees with export metadata")
    initial_session_id = header.get("session_id")
    if isinstance(initial_session_id, bool) or not isinstance(initial_session_id, int) or initial_session_id <= 0:
        raise AnalysisError("metadata frozen header session_id is invalid")
    period_ms = header.get("period_ms")
    if isinstance(period_ms, bool) or not isinstance(period_ms, int) or period_ms <= 0:
        raise AnalysisError("metadata frozen header period_ms is invalid")
    for field in ("start_ms", "stop_ms"):
        value = header.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or not (0 <= value <= U32_MASK):
            raise AnalysisError(f"metadata frozen header {field} is outside u32 range")

    fields, rows = _read_csv(csv_path)
    if len(rows) != metadata.get("record_count"):
        raise AnalysisError("run.csv row count disagrees with current export metadata")
    for row_number, row in enumerate(rows, start=2):
        session_id = row["session_id"]
        if session_id != initial_session_id:
            raise AnalysisError(f"CSV row {row_number} session_id disagrees with frozen header")
        if row["fault"] not in FAULT_NAMES:
            raise AnalysisError(f"CSV row {row_number} contains unknown fault enum {row['fault']}")
        if row["mode"] < 0 or row["mode"] > MODE_FOLLOW_VISION_HOLD:
            raise AnalysisError(f"CSV row {row_number} contains unknown mode enum {row['mode']}")
        for flag in EVENT_FIELDS | {
            "vision_latest_valid",
            "vision_usable_fresh",
            "vision_timeout",
            "vision_hold_active",
            "vision_hold_expired",
        }:
            if row[flag] not in (0, 1):
                raise AnalysisError(f"CSV row {row_number} {flag} must be 0 or 1")
    if rows[0]["t_ms"] != header.get("start_ms") or rows[0]["t_elapsed_ms"] != 0:
        raise AnalysisError("first CSV row time disagrees with frozen header start_ms")
    expected_elapsed_stop = (int(header["stop_ms"]) - int(header["start_ms"])) & U32_MASK
    if rows[-1]["t_ms"] != header.get("stop_ms") or rows[-1]["t_elapsed_ms"] != expected_elapsed_stop:
        raise AnalysisError("last CSV row time disagrees with frozen header stop_ms")

    decoder: Optional[Callable[..., Any]] = None
    decode_validation = "synthetic-test-only; raw ABI decode not requested"
    if dataset_type == "real":
        if artifact.elf_path is None:
            raise AnalysisError("real-data artifact is missing its ELF path for raw-log decoding")
        decoder = decode_buffer
        decode_validation = "runtime-log-export.decode_buffer"
    elif artifact.elf_path is not None:
        decoder = decode_buffer
        decode_validation = "runtime-log-export.decode_buffer"
    elif synthetic_decoder is not None:
        decoder = synthetic_decoder
        decode_validation = "synthetic host mock decoder"

    if decoder is not None:
        try:
            _layout, decoded_header, decoded_rows = decoder(artifact.elf_path, raw_path)
        except (ExportError, OSError, TypeError, ValueError) as exc:
            raise AnalysisError(f"runtime-log.bin failed ABI decoding: {exc}") from exc
        header_fields = {
            "state": "state",
            "state_name": "state_name",
            "count": "count",
            "session_id": "session_id",
            "start_ms": "start_ms",
            "stop_ms": "stop_ms",
            "dropped_count": "dropped_count",
            "buffer_full": "buffer_full",
            "records_truncated": "records_truncated",
            "period_ms": "period_ms",
            "build_id": "build_id",
            "stop_reason": "stop_reason",
        }
        for metadata_field, decoded_field in header_fields.items():
            expected = header.get(metadata_field)
            actual = decoded_header.get(decoded_field)
            if metadata_field == "build_id":
                expected = str(expected or "").lower()
                actual = str(actual or "").lower()
            if expected != actual:
                raise AnalysisError(
                    f"run-metadata.json initial_header {metadata_field} disagrees with runtime-log.bin"
                )
        if len(decoded_rows) != len(rows):
            raise AnalysisError("decoded runtime-log.bin row count disagrees with run.csv")
        for row_index, (csv_row, decoded_row) in enumerate(zip(rows, decoded_rows), start=2):
            for field in REQUIRED_CSV_FIELDS:
                if csv_row.get(field) != decoded_row.get(field):
                    raise AnalysisError(
                        f"run.csv row {row_index} {field} disagrees with decoded runtime-log.bin"
                    )

    reasons: list[str] = []
    buffer_full = metadata.get("buffer_full") is True
    truncated = metadata.get("records_truncated") is True
    dropped_count = metadata.get("dropped_count")
    if buffer_full:
        reasons.append("buffer_full: run may be truncated at the end")
    if truncated or dropped_count != 0:
        reasons.append("records_truncated or dropped_count is non-zero")
    return metadata, reasons, rows, {
        "period_ms": period_ms,
        "raw_sha256": raw_sha.lower(),
        "fields": fields,
        "raw_decode_validation": decode_validation,
    }


def _validate_time(rows: list[dict[str, int]], period_ms: int, max_gap_ms: Optional[int]) -> tuple[bool, list[str], int]:
    limit = max_gap_ms if max_gap_ms is not None else period_ms * 3
    if limit <= 0:
        return False, ["max_gap_ms must be positive"], limit
    reasons: list[str] = []
    maximum_gap = 0
    for row_number, row in enumerate(rows, start=1):
        if not (0 <= row["t_ms"] <= U32_MASK):
            return False, [f"t_ms is outside u32 range at row {row_number}"], limit
        if not (0 <= row["t_elapsed_ms"] <= U32_MASK):
            return False, [f"t_elapsed_ms is outside u32 range at row {row_number}"], limit
    for index in range(len(rows) - 1):
        current = rows[index]
        following = rows[index + 1]
        elapsed_delta = following["t_elapsed_ms"] - current["t_elapsed_ms"]
        if elapsed_delta < 0:
            reasons.append(f"t_elapsed_ms moves backwards at rows {index + 1}/{index + 2}")
            break
        if elapsed_delta > U32_MASK:
            reasons.append(f"t_elapsed_ms delta exceeds u32 at rows {index + 1}/{index + 2}")
            break
        raw_delta = (following["t_ms"] - current["t_ms"]) & U32_MASK
        if raw_delta != elapsed_delta:
            reasons.append(f"HAL u32 time wrap is inconsistent at rows {index + 1}/{index + 2}")
            break
        maximum_gap = max(maximum_gap, elapsed_delta)
        if elapsed_delta > limit:
            reasons.append(
                f"time gap {elapsed_delta} ms exceeds the {limit} ms analysis limit "
                f"at rows {index + 1}/{index + 2}"
            )
            break
    return not reasons, reasons, limit


def _weighted_mean(total: float, duration_ms: int) -> Optional[float]:
    if duration_ms <= 0:
        return None
    return total / duration_ms


def _sign(value: int) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0


def _is_reliable_route(row: dict[str, int]) -> bool:
    return (
        row["mode"] == MODE_FOLLOW_RUNNING
        and row["vision_latest_valid"] == 1
        and row["vision_usable_fresh"] == 1
        and row["vision_timeout"] == 0
        and row["vision_hold_active"] == 0
        and row["vision_hold_expired"] == 0
    )


def _is_road_ready(row: dict[str, int]) -> bool:
    left = row["motor_left_applied_permille"]
    right = row["motor_right_applied_permille"]
    return (
        _is_reliable_route(row)
        and row["motor_requested_permille"] == EXPECTED_REQUEST_PERMILLE
        and 1100 <= row["servo_applied_us"] <= 2500
        and 400 <= left <= 600
        and 400 <= right <= 600
        and left + right == 1000
    )


def analyze_metrics(
    rows: list[dict[str, int]],
    period_ms: int,
    startup_exclude_ms: Optional[int],
    max_gap_ms: Optional[int],
    servo_center_us: Optional[int],
) -> tuple[Optional[dict[str, Any]], list[str]]:
    time_valid, time_reasons, gap_limit = _validate_time(rows, period_ms, max_gap_ms)
    if not time_valid:
        return None, time_reasons
    additional_startup = startup_exclude_ms if startup_exclude_ms is not None else 0
    if additional_startup < 0:
        return None, ["startup_exclude_ms must be non-negative"]

    start_marks = [i for i, row in enumerate(rows) if row["event_start"] == 1]
    stop_marks = [i for i, row in enumerate(rows) if row["event_stop"] == 1]
    if len(start_marks) != 1 or len(stop_marks) != 1:
        return None, ["requires exactly one event_start and one event_stop in the log"]
    start_index, stop_index = start_marks[0], stop_marks[0]
    if start_index >= stop_index:
        return None, ["event_start must occur before event_stop"]
    if rows[start_index]["event_button_start"] != 1:
        return None, ["event_start row does not include the PE7 button-start event"]
    if rows[stop_index]["event_button_stop"] != 1:
        return None, ["event_stop row does not include the PE7 button-stop event"]
    ready_indices = [i for i in range(start_index, stop_index) if _is_road_ready(rows[i])]
    if not ready_indices:
        return None, ["no road-ready sample before event_stop"]
    ready_index = ready_indices[0]
    ready_ms = rows[ready_index]["t_elapsed_ms"]
    start_ms = ready_ms + additional_startup
    stop_ms = rows[stop_index]["t_elapsed_ms"]
    if start_ms >= stop_ms:
        return None, ["additional startup exclusion after road-ready leaves no scoring window"]

    window_ms = stop_ms - start_ms
    session_start_ms = rows[start_index]["t_elapsed_ms"]
    session_duration_ms = stop_ms - session_start_ms
    hold_ms = 0
    expired_ms = 0
    reliable_ms = 0
    err_abs_integral = 0.0
    req_limited_integral = 0.0
    req_applied_integral = 0.0
    limited_applied_integral = 0.0
    motor_requested_integral = 0.0
    motor_left_integral = 0.0
    motor_right_integral = 0.0
    motor_pair_mean_integral = 0.0
    motor_req_applied_abs_integral = 0.0
    vision_timeout_ms = 0
    comm_fault_ms = 0
    comm_fault_event_count = 0
    hold_expired_event_count = 0
    recovery_event_count = 0
    recovery_resume_count = 0
    recovery_wait_ms = 0
    recovery_ramp_ms = 0
    auto_recovery_event_count = 0
    hold_active_event_count = 0
    hold_expired_flag_previous = rows[start_index]["vision_hold_expired"]
    previous_servo_sign = 0
    steering_reversals = 0
    maximum_req_limited = 0
    maximum_req_applied = 0
    maximum_limited_applied = 0
    maximum_motor_applied = 0
    max_requested_seen: Optional[int] = None
    all_requests_equal_target = True
    moved_both_motors = False
    left_counts_delta_integral = 0.0
    right_counts_delta_integral = 0.0
    count_proxy_intervals = 0
    motor_req_applied_abs_max = 0
    scored_duration_ms = 0
    recovery_pending = False
    recovery_zero_seen = False

    for index in range(start_index, stop_index + 1):
        row = rows[index]
        if row.get("event_hold_enter", 0) == 1:
            hold_active_event_count += 1
        if row.get("event_hold_deadline", 0) == 1 or (
            row["vision_hold_expired"] and not hold_expired_flag_previous
        ):
            hold_expired_event_count += 1
        if row.get("event_auto_recovery", 0) == 1:
            auto_recovery_event_count += 1
        if row["fault"] in COMM_FAULT_CODES and row["event_fault"] == 1:
            comm_fault_event_count += 1
        hold_expired_flag_previous = row["vision_hold_expired"]

        maximum_motor_applied = max(
            maximum_motor_applied,
            abs(row["motor_left_applied_permille"]),
            abs(row["motor_right_applied_permille"]),
        )

        if index > ready_index:
            expired_signal = row["vision_hold_expired"] == 1 or row.get("event_hold_deadline", 0) == 1
            if expired_signal and not recovery_pending:
                recovery_pending = True
                recovery_zero_seen = False
                recovery_event_count += 1
            if recovery_pending and _is_road_ready(row):
                recovery_pending = False
                recovery_resume_count += 1
                recovery_zero_seen = False

        if index == stop_index:
            break

        next_row = rows[index + 1]
        interval_start = row["t_elapsed_ms"]
        interval_end = next_row["t_elapsed_ms"]
        session_duration = interval_end - interval_start
        if session_duration > 0:
            if row["vision_hold_active"]:
                hold_ms += session_duration
            if row["vision_hold_expired"]:
                expired_ms += session_duration
            if row["vision_timeout"]:
                vision_timeout_ms += session_duration
            if row["fault"] in COMM_FAULT_CODES:
                comm_fault_ms += session_duration
            if recovery_pending:
                currently_zero = (
                    row["motor_left_applied_permille"] == 0
                    and row["motor_right_applied_permille"] == 0
                )
                if recovery_zero_seen and not currently_zero:
                    recovery_ramp_ms += session_duration
                else:
                    recovery_wait_ms += session_duration
                if currently_zero:
                    recovery_zero_seen = True

        overlap_start = max(interval_start, start_ms)
        overlap_end = min(interval_end, stop_ms)
        duration = overlap_end - overlap_start
        if duration <= 0 or recovery_pending:
            continue

        scored_duration_ms += duration

        requested = row["motor_requested_permille"]
        left = row["motor_left_applied_permille"]
        right = row["motor_right_applied_permille"]
        servo_requested = row["servo_requested_us"]
        servo_limited = row["servo_limited_us"]
        servo_applied = row["servo_applied_us"]
        abs_req_limited = abs(servo_requested - servo_limited)
        abs_req_applied = abs(servo_requested - servo_applied)
        abs_limited_applied = abs(servo_limited - servo_applied)

        maximum_req_limited = max(maximum_req_limited, abs_req_limited)
        maximum_req_applied = max(maximum_req_applied, abs_req_applied)
        maximum_limited_applied = max(maximum_limited_applied, abs_limited_applied)
        max_requested_seen = requested if max_requested_seen is None else max(max_requested_seen, requested)
        all_requests_equal_target = all_requests_equal_target and requested == EXPECTED_REQUEST_PERMILLE
        moved_both_motors = moved_both_motors or (left > 0 and right > 0)

        motor_requested_integral += requested * duration
        motor_left_integral += left * duration
        motor_right_integral += right * duration
        motor_pair_mean_integral += ((left + right) / 2.0) * duration
        motor_req_applied_abs = abs(requested - ((left + right) / 2.0))
        motor_req_applied_abs_integral += motor_req_applied_abs * duration
        motor_req_applied_abs_max = max(motor_req_applied_abs_max, motor_req_applied_abs)
        req_limited_integral += abs_req_limited * duration
        req_applied_integral += abs_req_applied * duration
        limited_applied_integral += abs_limited_applied * duration

        if _is_reliable_route(row):
            reliable_ms += duration
            err_abs_integral += abs(row["vision_latest_error_norm_milli"]) * duration

        if servo_center_us is not None and servo_applied > 0:
            current_sign = _sign(servo_applied - servo_center_us)
            if current_sign != 0:
                if previous_servo_sign != 0 and current_sign != previous_servo_sign:
                    steering_reversals += 1
                previous_servo_sign = current_sign

        if "encoder_left_count" in row and "encoder_right_count" in row and interval_end > interval_start:
            fraction = duration / (interval_end - interval_start)
            left_delta = next_row.get("encoder_left_count", row["encoder_left_count"]) - row["encoder_left_count"]
            right_delta = next_row.get("encoder_right_count", row["encoder_right_count"]) - row["encoder_right_count"]
            left_counts_delta_integral += left_delta * fraction
            right_counts_delta_integral += right_delta * fraction
            count_proxy_intervals += 1

    requested_mean = _weighted_mean(motor_requested_integral, scored_duration_ms)
    applied_mean = _weighted_mean(motor_pair_mean_integral, scored_duration_ms)
    metrics = {
        "road_ready_elapsed_ms": ready_ms,
        "initial_startup_delay_ms": ready_ms - session_start_ms,
        "window_start_elapsed_ms": start_ms,
        "window_stop_elapsed_ms": stop_ms,
        "window_duration_ms": window_ms,
        "startup_exclude_ms": additional_startup,
        "session_duration_ms": session_duration_ms,
        "scored_duration_ms": scored_duration_ms,
        "recovery_wait_ms": recovery_wait_ms,
        "recovery_ramp_ms": recovery_ramp_ms,
        "recovery_event_count": recovery_event_count,
        "recovery_resume_count": recovery_resume_count,
        "max_gap_limit_ms": gap_limit,
        "max_observed_gap_ms": max(
            (rows[i + 1]["t_elapsed_ms"] - rows[i]["t_elapsed_ms"] for i in range(len(rows) - 1)),
            default=0,
        ),
        "hold_active_ms": hold_ms,
        "hold_active_percent": hold_ms * 100.0 / session_duration_ms if session_duration_ms else 0.0,
        "hold_expired_ms": expired_ms,
        "hold_expired_percent": expired_ms * 100.0 / session_duration_ms if session_duration_ms else 0.0,
        "hold_expired_event_count": hold_expired_event_count,
        "hold_enter_event_count": hold_active_event_count,
        "reliable_route_ms": reliable_ms,
        "reliable_route_percent": reliable_ms * 100.0 / scored_duration_ms if scored_duration_ms else 0.0,
        "reliable_abs_error_norm_milli_mean": _weighted_mean(err_abs_integral, reliable_ms),
        "vision_timeout_ms": vision_timeout_ms,
        "comm_fault_ms": comm_fault_ms,
        "comm_fault_event_count": comm_fault_event_count,
        "auto_recovery_event_count": auto_recovery_event_count,
        "motor_requested_permille_mean": requested_mean,
        "motor_left_applied_permille_mean": _weighted_mean(motor_left_integral, scored_duration_ms),
        "motor_right_applied_permille_mean": _weighted_mean(motor_right_integral, scored_duration_ms),
        "motor_applied_pair_mean_permille": applied_mean,
        "motor_requested_vs_applied_abs_permille_mean": _weighted_mean(
            motor_req_applied_abs_integral, scored_duration_ms
        ),
        "motor_requested_vs_applied_abs_permille_max": motor_req_applied_abs_max,
        "motor_requested_matches_500": bool(
            requested_mean is not None
            and abs(requested_mean - EXPECTED_REQUEST_PERMILLE) <= FLOAT_TIE_TOLERANCE
            and all_requests_equal_target
        ),
        "motor_applied_max_permille": maximum_motor_applied,
        "motor_applied_above_expected_600_permille": maximum_motor_applied > 600,
        "motor_left_and_right_nonzero_seen": moved_both_motors,
        "servo_requested_vs_limited_abs_us_mean": _weighted_mean(req_limited_integral, scored_duration_ms),
        "servo_requested_vs_applied_abs_us_mean": _weighted_mean(req_applied_integral, scored_duration_ms),
        "servo_limited_vs_applied_abs_us_mean": _weighted_mean(limited_applied_integral, scored_duration_ms),
        "servo_requested_vs_limited_abs_us_max": maximum_req_limited,
        "servo_requested_vs_applied_abs_us_max": maximum_req_applied,
        "servo_limited_vs_applied_abs_us_max": maximum_limited_applied,
        "servo_command_direction_reversals": steering_reversals if servo_center_us is not None else None,
        "servo_center_us_from_artifacts": servo_center_us,
        "signed_encoder_left_counts_per_s_proxy": (
            left_counts_delta_integral * 1000.0 / scored_duration_ms if count_proxy_intervals and scored_duration_ms else None
        ),
        "signed_encoder_right_counts_per_s_proxy": (
            right_counts_delta_integral * 1000.0 / scored_duration_ms if count_proxy_intervals and scored_duration_ms else None
        ),
        "encoder_proxy_units": "signed encoder counts/s; no wheel-size or m/s conversion",
    }
    if max_requested_seen is not None:
        metrics["motor_requested_max_permille"] = max_requested_seen
    return metrics, []


def _find_hard_fault(rows: list[dict[str, int]], metadata: dict[str, Any]) -> tuple[bool, list[str]]:
    codes = sorted({row["fault"] for row in rows if row["fault"] in HARD_FAULT_CODES})
    latched = any(row["mode"] == MODE_FAULT_LATCHED for row in rows)
    reasons = ["hard-fault fault enum(s): " + ", ".join(f"{code} {FAULT_NAMES[code]}" for code in codes)] if codes else []
    if latched:
        reasons.append("mode=8 FAULT_LATCHED observed")
    header = metadata.get("initial_header", {})
    if header.get("stop_reason") == 2:
        reasons.append("frozen header stop_reason=2 HARD_FAULT")
    return bool(reasons), reasons


def _compare_conditions(runs: list[dict[str, Any]]) -> tuple[bool, list[str], dict[str, Any]]:
    condition_runs = [run for run in runs if run.get("export_valid") and run.get("conditions")]
    if not condition_runs:
        return False, ["no successful artifact-matched exports have comparable condition metadata"], {}
    conditions = [run["conditions"] for run in condition_runs]
    layout_ids = sorted({item["layout_id"] for item in conditions})
    camera_ids = sorted({item["camera_build_id"] for item in conditions})
    lighting_ids = sorted({item["lighting_id"] for item in conditions})
    voltages = [item["battery_voltage_v"] for item in conditions]
    voltage_min = min(voltages)
    voltage_max = max(voltages)
    span_decimal = Decimal(str(voltage_max)) - Decimal(str(voltage_min))
    span = float(span_decimal)
    battery_states = sorted({item["battery_state"] for item in conditions})
    reasons: list[str] = []
    if len(layout_ids) != 1:
        reasons.append("layout_id differs across candidate trials")
    if len(camera_ids) != 1:
        reasons.append("camera_build_id differs across candidate trials")
    if len(lighting_ids) != 1:
        reasons.append("lighting_id differs across candidate trials")
    if span_decimal > Decimal("0.2"):
        reasons.append(f"battery voltage span {span:.3f} V exceeds 0.200 V")
    conditions_summary = {
        "compared_exported_runs": len(condition_runs),
        "layout_ids": layout_ids,
        "camera_build_ids": camera_ids,
        "lighting_ids": lighting_ids,
        "battery_voltage_min_v": voltage_min,
        "battery_voltage_max_v": voltage_max,
        "battery_voltage_span_v": span,
        "battery_states": battery_states,
        "battery_state_variation_warning": len(battery_states) > 1,
    }
    return not reasons, reasons, conditions_summary


def _metric_median(runs: list[dict[str, Any]], name: str) -> Optional[float]:
    values = [run["metrics"].get(name) for run in runs if isinstance(run.get("metrics"), dict)]
    values = [float(value) for value in values if isinstance(value, (int, float)) and not isinstance(value, bool)]
    return statistics.median(values) if values else None


def _order_profiles(profiles: list[dict[str, Any]]) -> list[dict[str, Any]]:
    metrics = (
        "median_hold_active_ms",
        "median_reliable_abs_error_norm_milli_mean",
        "median_servo_requested_vs_limited_abs_us_mean",
        "median_servo_requested_vs_limited_abs_us_max",
        "median_servo_requested_vs_applied_abs_us_mean",
        "median_servo_requested_vs_applied_abs_us_max",
    )
    groups: list[list[dict[str, Any]]] = [profiles]
    for metric in metrics:
        next_groups: list[list[dict[str, Any]]] = []
        for group in groups:
            ordered = sorted(group, key=lambda profile: float(profile[metric]))
            bucket: list[dict[str, Any]] = []
            anchor: Optional[float] = None
            for profile in ordered:
                value = float(profile[metric])
                if anchor is None or abs(value - anchor) <= FLOAT_TIE_TOLERANCE:
                    if anchor is None:
                        anchor = value
                    bucket.append(profile)
                else:
                    next_groups.append(bucket)
                    bucket = [profile]
                    anchor = value
            if bucket:
                next_groups.append(bucket)
        groups = next_groups
    flattened: list[dict[str, Any]] = []
    for group in groups:
        flattened.extend(sorted(group, key=lambda profile: profile["diff_permille"]))
    return flattened


def _analyze_run(
    entry: dict[str, Any],
    manifest_dir: Path,
    artifacts: dict[str, Artifact],
    dataset_type: str,
    expected_trial_set_id: Optional[str],
    startup_exclude_ms: Optional[int],
    max_gap_ms: Optional[int],
    synthetic_decoder: Optional[Callable[..., Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "run_id": entry.get("run_id"),
        "variant": entry.get("variant"),
        "trial_set_id": entry.get("trial_set_id"),
        "export_valid": False,
        "group_data_valid": False,
        "group_data_reasons": [],
        "eligible_for_profile": False,
        "outcome": "unverified",
        "rejection_reasons": [],
        "anomalies": [],
        "conditions": None,
        "metrics": None,
        "fault_codes_seen": [],
        "hard_fault": False,
    }
    group_data_reasons: list[str] = []
    try:
        run_id = _require_nonempty_string(entry.get("run_id"), "run_id")
        result["run_id"] = run_id
        run_trial_set_id = _require_nonempty_string(entry.get("trial_set_id"), "trial_set_id")
        if expected_trial_set_id is None or run_trial_set_id != expected_trial_set_id:
            raise AnalysisError("run trial_set_id does not match the manifest trial_set_id")
        variant = _require_nonempty_string(entry.get("variant"), "variant")
        if variant not in PROFILE_DIFF_PERMILLE:
            raise AnalysisError("variant must be one of none, medium, strong")
        result["variant"] = variant
        observed_completed = _require_bool(entry.get("completed"), "completed")
        observed_out_of_track = _require_bool(entry.get("out_of_track"), "out_of_track")
        result["observed"] = {
            "completed": observed_completed,
            "out_of_track": observed_out_of_track,
        }
        result["conditions"] = _validate_conditions(entry)
        csv_path, metadata_path = _resolve_run_paths(entry.get("path"), manifest_dir)
        result["path"] = str(csv_path.parent)
        artifact = artifacts[variant]
        metadata, export_reasons, rows, details = _validate_export(
            csv_path, metadata_path, artifact, dataset_type, synthetic_decoder
        )
        result["metadata_path"] = str(metadata_path)
        result["csv_path"] = str(csv_path)
        result["session_id"] = rows[0]["session_id"]
        result["profile_diff_permille"] = artifact.diff_permille
        result["metadata_status"] = metadata.get("status")
        result["elf_sha256"] = metadata.get("elf_sha256")
        result["build_id"] = metadata.get("build_id")
        result["raw_sha256"] = details["raw_sha256"]
        result["period_ms"] = details["period_ms"]
        result["record_count"] = len(rows)
        result["export_valid"] = True
        result["raw_decode_validation"] = details["raw_decode_validation"]
        result["rejection_reasons"].extend(export_reasons)
        group_data_reasons.extend(export_reasons)

        session_identity = (artifact.build_id, rows[0]["session_id"], details["raw_sha256"])
        result["session_identity"] = ":".join(str(part) for part in session_identity)
        time_valid, time_reasons, _ = _validate_time(rows, details["period_ms"], max_gap_ms)
        if not time_valid:
            group_data_reasons.extend(time_reasons)
        metrics, time_or_window_reasons = analyze_metrics(
            rows,
            details["period_ms"],
            startup_exclude_ms,
            max_gap_ms,
            artifact.servo_center_us,
        )
        if metrics is not None:
            result["metrics"] = metrics
        result["rejection_reasons"].extend(time_or_window_reasons)
        data_integrity_prefixes = (
            "requires exactly one event_start",
            "event_start must occur before event_stop",
            "event_start row does not include the PE7 button-start event",
            "event_stop row does not include the PE7 button-stop event",
            "additional startup exclusion after road-ready leaves no scoring window",
        )
        group_data_reasons.extend(
            reason for reason in time_or_window_reasons if reason.startswith(data_integrity_prefixes)
        )

        hard_fault, hard_reasons = _find_hard_fault(rows, metadata)
        result["hard_fault"] = hard_fault
        result["rejection_reasons"].extend(hard_reasons)
        result["fault_codes_seen"] = sorted({row["fault"] for row in rows if row["fault"] != 0})
        if not observed_completed:
            result["rejection_reasons"].append("user-observed completed=false")
        if observed_out_of_track:
            result["rejection_reasons"].append("user-observed out_of_track=true")
        if metrics is None:
            if observed_completed and not observed_out_of_track and not hard_fault:
                group_data_reasons.append(
                    "completed=true but the export has no usable road-ready scoring window"
                )
        else:
            if metrics["reliable_route_ms"] <= 0:
                result["rejection_reasons"].append("no reliable-route time in the steady window")
            if not metrics["motor_left_and_right_nonzero_seen"]:
                result["rejection_reasons"].append("no interval records nonzero applied output on both motors")
            if not metrics["motor_requested_matches_500"]:
                result["rejection_reasons"].append("time-weighted requested motor command is not exactly 500‰")
            if metrics["motor_applied_max_permille"] > MOTOR_HARD_CAP_PERMILLE:
                result["rejection_reasons"].append("recorded applied motor command exceeds hard cap 700‰")
            if metrics["motor_applied_above_expected_600_permille"]:
                message = "recorded motor applied value exceeds the current algorithm's expected 600‰ maximum"
                result["anomalies"].append(message)
                result["rejection_reasons"].append(message)
            if observed_completed and not observed_out_of_track and not hard_fault and (
                metrics["reliable_route_ms"] <= 0 or not metrics["motor_left_and_right_nonzero_seen"]
            ):
                group_data_reasons.append(
                    "completed=true but no reliable route or simultaneous non-zero motor interval is recorded"
                )
            final_row = rows[-1]
            nonzero_final = [field for field in OUTPUT_ZERO_FIELDS if final_row.get(field, 0) != 0]
            if nonzero_final:
                result["rejection_reasons"].append(
                    "final frozen record has non-zero output fields: " + ", ".join(nonzero_final)
                )
        result["eligible_for_profile"] = not result["rejection_reasons"]
    except (AnalysisError, KeyError, TypeError, ValueError) as exc:
        result["rejection_reasons"].append(str(exc))
        result["eligible_for_profile"] = False
        group_data_reasons.append(str(exc))
    result["group_data_reasons"] = list(dict.fromkeys(group_data_reasons))
    result["group_data_valid"] = not result["group_data_reasons"] and result["export_valid"]
    if result["group_data_valid"] and result["eligible_for_profile"]:
        result["outcome"] = "passed"
    elif result["group_data_valid"]:
        result["outcome"] = "failed"
    return result


def _deduplicate_runs(runs: list[dict[str, Any]]) -> list[str]:
    seen_run_ids: dict[str, dict[str, Any]] = {}
    seen_sessions: dict[str, dict[str, Any]] = {}
    group_reasons: list[str] = []

    def invalidate_duplicate(run: dict[str, Any], reason: str) -> None:
        run["rejection_reasons"].append(reason)
        run["group_data_reasons"].append(reason)
        run["group_data_valid"] = False
        run["eligible_for_profile"] = False
        run["outcome"] = "unverified"

    for run in runs:
        run_id = run.get("run_id")
        if isinstance(run_id, str):
            if run_id in seen_run_ids:
                prior = seen_run_ids[run_id]
                reason = "duplicate manifest run_id"
                invalidate_duplicate(prior, reason)
                invalidate_duplicate(run, reason)
                group_reasons.append(reason)
            else:
                seen_run_ids[run_id] = run
        identity = run.get("session_identity")
        if run.get("export_valid") and isinstance(identity, str):
            if identity in seen_sessions:
                prior = seen_sessions[identity]
                reason = f"duplicate frozen session copied from run_id={prior.get('run_id')}"
                invalidate_duplicate(prior, reason)
                invalidate_duplicate(run, reason)
                group_reasons.append(reason)
            else:
                seen_sessions[identity] = run
    return list(dict.fromkeys(group_reasons))


def scan_export_diagnostics(root: Optional[Path]) -> list[dict[str, Any]]:
    if root is None or not root.exists():
        return []
    diagnostics: list[dict[str, Any]] = []
    for metadata_path in sorted(root.rglob("run-metadata.json")):
        try:
            metadata = _read_json(metadata_path)
        except AnalysisError as exc:
            diagnostics.append({"path": str(metadata_path.parent), "status": "invalid_metadata", "error": str(exc)})
            continue
        if metadata.get("status") != "failed":
            continue
        error = str(metadata.get("error") or "")
        diagnostics.append(
            {
                "path": str(metadata_path.parent),
                "status": "failed_export",
                "stage": metadata.get("stage"),
                "error_code": "DEV_CONNECT_ERR" if "DEV_CONNECT_ERR" in error else None,
                "error": error,
                "csv_present": (metadata_path.parent / "run.csv").is_file(),
                "header_available": metadata.get("initial_header") is not None,
                "raw_file_count": len(metadata.get("raw_files") or []),
                "elf_sha256": metadata.get("elf_sha256"),
                "build_id": metadata.get("build_id"),
            }
        )
    return diagnostics


def analyze_manifest(
    manifest_path: Path,
    artifacts_path: Path,
    diagnostics_root: Optional[Path],
    startup_exclude_ms: Optional[int],
    max_gap_ms: Optional[int],
    synthetic_decoder: Optional[Callable[..., Any]] = None,
) -> dict[str, Any]:
    manifest = _read_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AnalysisError("trial manifest schema_version must be 1")
    dataset_type = manifest.get("dataset_type")
    if dataset_type not in {"real", "synthetic-test-only"}:
        raise AnalysisError("trial manifest dataset_type must be real or synthetic-test-only")
    if not isinstance(manifest.get("runs"), list):
        raise AnalysisError("trial manifest runs must be an array")
    if synthetic_decoder is not None and dataset_type != "synthetic-test-only":
        raise AnalysisError("synthetic_decoder is permitted only for synthetic-test-only manifests")
    raw_trial_set_id = manifest.get("trial_set_id")
    trial_set_id = raw_trial_set_id.strip() if isinstance(raw_trial_set_id, str) and raw_trial_set_id.strip() else None
    manifest_reasons = [] if trial_set_id is not None else ["manifest trial_set_id is missing or empty"]
    artifacts = load_artifacts(artifacts_path, dataset_type)
    runs: list[dict[str, Any]] = []
    for index, entry in enumerate(manifest["runs"]):
        if not isinstance(entry, dict):
            runs.append(
                {
                    "run_id": f"entry-{index + 1}",
                    "variant": None,
                    "export_valid": False,
                    "group_data_valid": False,
                    "group_data_reasons": ["manifest run entry must be an object"],
                    "eligible_for_profile": False,
                    "outcome": "unverified",
                    "rejection_reasons": ["manifest run entry must be an object"],
                    "metrics": None,
                }
            )
            continue
        runs.append(
            _analyze_run(
                entry,
                manifest_path.parent,
                artifacts,
                dataset_type,
                trial_set_id,
                startup_exclude_ms,
                max_gap_ms,
                synthetic_decoder,
            )
        )
    duplicate_reasons = _deduplicate_runs(runs)
    conditions_comparable, condition_reasons, conditions = _compare_conditions(runs)

    group_reasons = list(manifest_reasons)
    group_reasons.extend(duplicate_reasons)
    for variant in PROFILE_DIFF_PERMILLE:
        count = sum(run.get("variant") == variant for run in runs)
        if count != MIN_SESSIONS_PER_PROFILE:
            group_reasons.append(
                f"{variant} has {count} attempts; exactly {MIN_SESSIONS_PER_PROFILE} are required"
            )
    for run in runs:
        if run.get("variant") not in PROFILE_DIFF_PERMILLE:
            group_reasons.append(f"unassigned manifest attempt {run.get('run_id', '—')} has an unknown variant")
        if not run.get("group_data_valid"):
            reasons = "; ".join(run.get("group_data_reasons") or run.get("rejection_reasons") or [])
            group_reasons.append(
                f"attempt {run.get('run_id', '—')} has incomplete or unverified data"
                + (f": {reasons}" if reasons else "")
            )
    if not conditions_comparable:
        group_reasons.extend(condition_reasons)
    group_reasons = list(dict.fromkeys(group_reasons))
    group_complete = not group_reasons

    summaries: list[dict[str, Any]] = []
    for variant, diff in PROFILE_DIFF_PERMILLE.items():
        matching_runs = [run for run in runs if run.get("variant") == variant]
        eligible_runs = [run for run in matching_runs if run.get("eligible_for_profile")]
        data_valid_runs = [run for run in matching_runs if run.get("group_data_valid")]
        profile_data_complete = (
            len(matching_runs) == MIN_SESSIONS_PER_PROFILE
            and len(data_valid_runs) == MIN_SESSIONS_PER_PROFILE
        )
        profile_passed = profile_data_complete and len(eligible_runs) == MIN_SESSIONS_PER_PROFILE
        profile_status = (
            "insufficient" if not profile_data_complete else "passed" if profile_passed else "failed"
        )
        profile_metrics = eligible_runs if profile_passed else []
        summary = {
            "variant": variant,
            "diff_permille": diff,
            "artifact_sha256": artifacts[variant].elf_sha256,
            "artifact_build_id": artifacts[variant].build_id,
            "attempted_runs": len(matching_runs),
            "required_attempt_runs": MIN_SESSIONS_PER_PROFILE,
            "export_valid_runs": sum(bool(run.get("export_valid")) for run in matching_runs),
            "group_data_valid_runs": len(data_valid_runs),
            "profile_eligible_runs": len(eligible_runs),
            "failed_runs": sum(run.get("group_data_valid") and not run.get("eligible_for_profile") for run in matching_runs),
            "required_eligible_runs": MIN_SESSIONS_PER_PROFILE,
            "profile_status": profile_status,
            "eligible": profile_passed,
            "median_hold_active_ms": _metric_median(profile_metrics, "hold_active_ms"),
            "median_hold_active_percent": _metric_median(profile_metrics, "hold_active_percent"),
            "median_reliable_route_percent": _metric_median(profile_metrics, "reliable_route_percent"),
            "median_reliable_abs_error_norm_milli_mean": _metric_median(
                profile_metrics, "reliable_abs_error_norm_milli_mean"
            ),
            "median_servo_requested_vs_limited_abs_us_mean": _metric_median(
                profile_metrics, "servo_requested_vs_limited_abs_us_mean"
            ),
            "median_servo_requested_vs_applied_abs_us_mean": _metric_median(
                profile_metrics, "servo_requested_vs_applied_abs_us_mean"
            ),
            "median_servo_limited_vs_applied_abs_us_mean": _metric_median(
                profile_metrics, "servo_limited_vs_applied_abs_us_mean"
            ),
            "median_servo_requested_vs_limited_abs_us_max": _metric_median(
                profile_metrics, "servo_requested_vs_limited_abs_us_max"
            ),
            "median_servo_requested_vs_applied_abs_us_max": _metric_median(
                profile_metrics, "servo_requested_vs_applied_abs_us_max"
            ),
            "median_motor_requested_permille_mean": _metric_median(
                profile_metrics, "motor_requested_permille_mean"
            ),
            "median_motor_applied_pair_mean_permille": _metric_median(
                profile_metrics, "motor_applied_pair_mean_permille"
            ),
            "median_comm_fault_ms": _metric_median(profile_metrics, "comm_fault_ms"),
            "median_comm_fault_event_count": _metric_median(profile_metrics, "comm_fault_event_count"),
            "median_hold_expired_event_count": _metric_median(profile_metrics, "hold_expired_event_count"),
            "median_recovery_wait_ms": _metric_median(profile_metrics, "recovery_wait_ms"),
            "median_steering_reversals": _metric_median(profile_metrics, "servo_command_direction_reversals"),
            "ranking_status": "not-ranked",
            "rank": None,
            "synthetic_order": None,
        }
        summaries.append(summary)

    can_rank = group_complete
    sortable = [summary for summary in summaries if summary["eligible"]]
    has_all_primary_metrics = all(
        summary["median_hold_active_ms"] is not None
        and summary["median_reliable_abs_error_norm_milli_mean"] is not None
        and summary["median_servo_requested_vs_limited_abs_us_mean"] is not None
        and summary["median_servo_requested_vs_limited_abs_us_max"] is not None
        and summary["median_servo_requested_vs_applied_abs_us_mean"] is not None
        and summary["median_servo_requested_vs_applied_abs_us_max"] is not None
        for summary in sortable
    )
    if can_rank and sortable and has_all_primary_metrics:
        ordered = _order_profiles(sortable)
        for index, summary in enumerate(ordered, start=1):
            if dataset_type == "real":
                summary["rank"] = index
                summary["ranking_status"] = "ranked"
            else:
                summary["synthetic_order"] = index
                summary["ranking_status"] = "synthetic-test-order-only"
    else:
        if not group_complete:
            reason = "trial set is insufficient"
        elif not sortable:
            reason = "no eligible profile: all three attempts failed for every profile"
        else:
            reason = "one or more primary comparison metrics are unavailable"
        for summary in summaries:
            if summary["profile_status"] == "failed":
                summary["ranking_status"] = "profile failed: one or more of exactly three attempts failed"
            else:
                summary["ranking_status"] = reason

    diagnostics = scan_export_diagnostics(diagnostics_root)
    recommendation = None
    result_status = "insufficient"
    trial_set_status = "insufficient"
    if group_complete:
        trial_set_status = "rankable" if sortable and has_all_primary_metrics else "no_eligible_profile"
    if dataset_type == "synthetic-test-only":
        result_status = "synthetic-test-only"
    elif can_rank and sortable and has_all_primary_metrics:
        result_status = "ranked"
        recommendation = next(summary["variant"] for summary in summaries if summary["rank"] == 1)
    elif can_rank and not sortable:
        result_status = "no_eligible_profile"
    return {
        "schema_version": 1,
        "dataset_type": dataset_type,
        "trial_set_id": trial_set_id,
        "trial_set_status": trial_set_status,
        "trial_set_complete": group_complete,
        "trial_set_reasons": group_reasons,
        "manifest_path": str(manifest_path.resolve()),
        "artifacts_path": str(artifacts_path.resolve()),
        "analysis_status": result_status,
        "recommendation": recommendation,
        "synthetic_order_is_not_a_vehicle_recommendation": dataset_type == "synthetic-test-only",
        "comparison": {
            "layout_camera_lighting_battery_comparable": conditions_comparable,
            "condition_reasons": condition_reasons,
            "conditions": conditions,
            "battery_voltage_tolerance_v": 0.2,
            "same_battery_state_required": False,
            "condition_policy": "same layout/camera/lighting IDs and battery voltage span <= 0.2 V",
        },
        "analysis_policy": {
            "attempts_required_per_profile": MIN_SESSIONS_PER_PROFILE,
            "all_attempts_must_be_retained": True,
            "profile_pass_requires_all_attempts": True,
            "trial_set_id_required_and_matched_on_every_attempt": True,
            "expected_motor_requested_permille": EXPECTED_REQUEST_PERMILLE,
            "motor_applied_hard_cap_permille": MOTOR_HARD_CAP_PERMILLE,
            "current_algorithm_expected_motor_applied_max_permille": 600,
            "hard_fault_codes": sorted(HARD_FAULT_CODES),
            "hard_fault_names": [FAULT_NAMES[code] for code in sorted(HARD_FAULT_CODES)],
            "communication_fault_codes_reported_separately": sorted(COMM_FAULT_CODES),
            "mode_8_is_fault_latched": True,
            "startup_exclude_ms": startup_exclude_ms,
            "default_startup_exclude": "0 ms after the first road-ready sample",
            "road_ready_rule": (
                "mode=10 reliable route, requested=500, servo_applied_us in 1100..2500, "
                "each applied motor in 400..600 and their sum equals 1000"
            ),
            "recovery_scoring_rule": "exclude expired-hold wait and zero-to-road-ready recovery ramp from follow/error scoring",
            "session_hold_and_fault_accounting": "event_start through event_stop, including initial startup and recovery",
            "default_max_gap": "3 * logged period_ms",
            "max_gap_override_ms": max_gap_ms,
            "time_weighting": "record state is integrated over its interval until the next record",
            "steady_window": "[first road-ready sample + optional startup exclusion, event_stop), excluding post-expired recovery wait",
            "reliable_route_rule": "mode=10, latest valid and usable fresh, no timeout, no active/expired hold",
            "servo_differences_are_software_values": True,
            "servo_differences_measure_mechanical_delay": False,
        },
        "profiles": summaries,
        "runs": runs,
        "export_diagnostics": diagnostics,
    }


def _csv_value(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return format(value, ".9g")
    return str(value)


SUMMARY_FIELDS = [
    "variant",
    "diff_permille",
    "artifact_sha256",
    "artifact_build_id",
    "attempted_runs",
    "required_attempt_runs",
    "export_valid_runs",
    "group_data_valid_runs",
    "profile_eligible_runs",
    "failed_runs",
    "required_eligible_runs",
    "profile_status",
    "eligible",
    "rank",
    "synthetic_order",
    "ranking_status",
    "median_hold_active_ms",
    "median_hold_active_percent",
    "median_reliable_route_percent",
    "median_reliable_abs_error_norm_milli_mean",
    "median_servo_requested_vs_limited_abs_us_mean",
    "median_servo_requested_vs_limited_abs_us_max",
    "median_servo_requested_vs_applied_abs_us_mean",
    "median_servo_limited_vs_applied_abs_us_mean",
    "median_servo_requested_vs_applied_abs_us_max",
    "median_motor_requested_permille_mean",
    "median_motor_applied_pair_mean_permille",
    "median_comm_fault_ms",
    "median_comm_fault_event_count",
    "median_hold_expired_event_count",
    "median_steering_reversals",
]


def _render_report(analysis: dict[str, Any]) -> str:
    lines = ["# 差速试车日志分析", ""]
    dataset_type = analysis["dataset_type"]
    if dataset_type == "synthetic-test-only":
        lines.extend(
            [
                "> **合成测试数据，仅验证分析逻辑；不代表实车观察，也不能据此推荐档位。**",
                "",
            ]
        )
    lines.extend(
        [
            f"- 数据类型：`{dataset_type}`",
            f"- trial_set_id：`{analysis.get('trial_set_id') or 'missing'}`",
            f"- 分析状态：`{analysis['analysis_status']}`",
            f"- 试验组完整性：`{analysis.get('trial_set_status', 'insufficient')}`",
        ]
    )
    if analysis.get("recommendation"):
        lines.append(f"- 当前数据下排名第一：`{analysis['recommendation']}`")
    elif dataset_type == "real":
        if analysis.get("analysis_status") == "no_eligible_profile":
            lines.append("- 排名结论：**no_eligible_profile**；完整试验组中的所有档位均未通过。")
        else:
            lines.append("- 排名结论：**insufficient**；不能据此选择差速档位。")
    if analysis.get("trial_set_reasons"):
        lines.append("- 试验组问题：" + "; ".join(analysis["trial_set_reasons"]))
    lines.append("")

    comparison = analysis["comparison"]
    lines.append("## 试车条件")
    lines.append("")
    lines.append(f"- 全局可比：`{comparison['layout_camera_lighting_battery_comparable']}`")
    if comparison.get("conditions"):
        conditions = comparison["conditions"]
        lines.append(f"- layout ID：{', '.join(conditions['layout_ids'])}")
        lines.append(f"- camera build ID：{', '.join(conditions['camera_build_ids'])}")
        lines.append(f"- lighting ID：{', '.join(conditions['lighting_ids'])}")
        lines.append(
            f"- 电池电压：{conditions['battery_voltage_min_v']:.3f}–"
            f"{conditions['battery_voltage_max_v']:.3f} V，跨度 {conditions['battery_voltage_span_v']:.3f} V"
        )
        if conditions["battery_state_variation_warning"]:
            lines.append("- 警告：人工填写的 battery_state 有差异；本轮规则只按实测电压跨度判定。")
    for reason in comparison["condition_reasons"]:
        lines.append(f"- 不可比原因：{reason}")
    if not comparison.get("conditions"):
        lines.append("- 当前清单没有成功且匹配固件的导出记录，无法核验比较条件。")
    lines.append("")

    lines.append("## 档位汇总")
    lines.append("")
    lines.append(
        "| 档位 | d (‰) | 尝试/要求 | 数据完整 | 全部通过 | 状态/排名 | hold 总时长中位数 (ms) | "
        "hold-expired events | 恢复等待中位数 (ms) | 可靠路线占比 (%) | 可靠路线误差 (‰) | "
        "req→limited 均值/最大值 (µs) | req→applied 均值/最大值 (µs) | comm fault ms/events | 舵机命令反向次数 |"
    )
    lines.append("|---|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for profile in analysis["profiles"]:
        order = profile["rank"] if profile["rank"] is not None else profile["synthetic_order"]
        hold = profile["median_hold_active_ms"]
        limited_mean = profile["median_servo_requested_vs_limited_abs_us_mean"]
        limited_max = profile["median_servo_requested_vs_limited_abs_us_max"]
        applied_mean = profile["median_servo_requested_vs_applied_abs_us_mean"]
        applied_max = profile["median_servo_requested_vs_applied_abs_us_max"]
        lines.append(
            "| {variant} | {diff} | {attempted}/{required} | {data_valid} | {eligible_count} | "
            "{status}/{order} | {hold} | {hold_expired} | {recovery_wait} | {reliable_percent} | {error} | "
            "{limited_mean}/{limited_max} | {applied_mean}/{applied_max} | {comm_ms}/{comm_events} | {reversals} |".format(
                variant=profile["variant"],
                diff=profile["diff_permille"],
                attempted=profile["attempted_runs"],
                required=profile["required_attempt_runs"],
                data_valid=profile["group_data_valid_runs"],
                eligible_count=profile["profile_eligible_runs"],
                status=profile["profile_status"],
                order=order if order is not None else "—",
                hold=_fmt_number(hold),
                hold_expired=_fmt_number(profile["median_hold_expired_event_count"]),
                recovery_wait=_fmt_number(profile["median_recovery_wait_ms"]),
                reliable_percent=_fmt_number(profile["median_reliable_route_percent"]),
                error=_fmt_number(profile["median_reliable_abs_error_norm_milli_mean"]),
                limited_mean=_fmt_number(limited_mean),
                limited_max=_fmt_number(limited_max),
                applied_mean=_fmt_number(applied_mean),
                applied_max=_fmt_number(applied_max),
                comm_ms=_fmt_number(profile["median_comm_fault_ms"]),
                comm_events=_fmt_number(profile["median_comm_fault_event_count"]),
                reversals=_fmt_number(profile["median_steering_reversals"]),
            )
        )
    lines.append("")
    lines.append(
        "每档必须在同一 trial_set_id 中恰好列出 3 次物理尝试。三次都必须有完整、唯一、可解码且身份匹配的日志；"
        "completed=false、out_of_track=true 或 hard fault 会使该档失败，不能用额外成功尝试替代。通信 soft fault 单独统计，不默认按 hard fault 处理。"
    )
    lines.append("")

    lines.append("## 运行记录")
    lines.append("")
    lines.append("| run_id | trial_set_id | 档位 | session | 导出 | 数据完整 | 人工结果 | 结果 | eligible | 原因 |")
    lines.append("|---|---|---|---:|---|---|---|---|---|---|")
    if not analysis["runs"]:
        lines.append("| — | — | — | — | — | no | 未填写 | unverified | no | manifest runs 为空 |")
    for run in analysis["runs"]:
        observed = run.get("observed") or {}
        outcome = "completed={}; out_of_track={}".format(
            observed.get("completed", "missing"), observed.get("out_of_track", "missing")
        )
        reasons = "; ".join(run.get("rejection_reasons", [])) or "—"
        lines.append(
            f"| {run.get('run_id', '—')} | {run.get('trial_set_id', '—')} | {run.get('variant', '—')} | "
            f"{run.get('session_id', '—')} | {'success' if run.get('export_valid') else 'failed'} | "
            f"{'yes' if run.get('group_data_valid') else 'no'} | {outcome} | {run.get('outcome', 'unverified')} | "
            f"{'yes' if run.get('eligible_for_profile') else 'no'} | {reasons} |"
        )
    lines.append("")

    diagnostics = analysis["export_diagnostics"]
    lines.append("## 导出诊断")
    lines.append("")
    if not diagnostics:
        lines.append("未扫描到失败导出记录。")
    else:
        lines.append(f"扫描到 {len(diagnostics)} 条失败导出记录；这些记录不计入试车样本。")
        lines.append("")
        lines.append("| 目录 | stage | code | CSV | RAM header | 说明 |")
        lines.append("|---|---|---|---|---|---|")
        for item in diagnostics:
            summary = "no data acquired" if item.get("error_code") == "DEV_CONNECT_ERR" else (item.get("error") or "")
            summary = summary.replace("|", "/").replace("\n", " ")
            if len(summary) > 180:
                summary = summary[:177] + "..."
            lines.append(
                f"| {item['path']} | {item.get('stage', '—')} | {item.get('error_code') or '—'} | "
                f"{item.get('csv_present')} | {item.get('header_available')} | {summary} |"
            )
    lines.append("")

    lines.append("## 计算口径与限制")
    lines.append("")
    lines.append(
        "- 初始评分从 event_start 后首个 ready 样本开始：mode=10、可靠路线、requested=500‰、"
        "servo_applied=1100..2500 µs、两侧 applied 各 400..600‰ 且和为 1000‰。默认额外启动剔除为 0 ms；"
        "`--startup-exclude-ms` 只能在该 ready 点后追加，不能替代 ready 条件。"
    )
    lines.append(
        "- 评分从首个 ready 点到 event_stop；后续 hold-expired 后的等待和零输出恢复 ramp，直到再次满足同一 ready 条件，"
        "会从视觉误差、舵机跟随和电机请求/应用跟随评分中排除。恢复等待与 ramp 时长单独保留。"
    )
    lines.append(
        "- hold active/expired、vision timeout、communication fault 的持续时间及事件数覆盖完整物理 session（event_start 至 event_stop），"
        "包括初次 ready 前和恢复等待期间，不会随评分窗口过滤。"
    )
    lines.append(
        "- 每个真实 run 都使用白名单 ELF 与 runtime-log.bin 做 CRC/version/header 解码，并逐行核对 CSV 必需字段；"
        "只做 SHA 校验不足以通过。合成 host fixture 不代表真实 ABI 或车辆观察。"
    )
    lines.append("- 持续时间按记录间隔积分；同一 HAL ms 的连续事件记录允许存在，零时长区间不积分但事件字段仍计数。默认拒绝超过 3 个 period 的间隔。")
    lines.append("- HAL u32 时间回绕通过 `t_ms` 模差与 `t_elapsed_ms` 差值交叉验证；倒退、错配或大间隔会使试验组 insufficient。")
    lines.append("- requested/limited/applied 差值来自日志软件数值；它不测量舵机机械响应或真实机械延迟。")
    lines.append("- 硬上限仍为 700‰；本轮算法输出超过 600‰ 会标为异常并使该次尝试失败。")
    lines.append("- 编码器结果只按带符号 counts/s 输出，不换算轮径、距离或 m/s；方向需结合现场编码器方向核验。")
    lines.append("- 只有结构完整、条件可比且恰好 3 次尝试全部通过的档位进入排序；通过档之间依次比较 hold 总时长、可靠路线误差、requested-applied 平均差、最大差，最后以较小 d 优先。")
    lines.append("")
    return "\n".join(lines)


def _fmt_number(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def _write_outputs(output_dir: Path, analysis: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.csv"
    with summary_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for profile in analysis["profiles"]:
            writer.writerow({field: _csv_value(profile.get(field)) for field in SUMMARY_FIELDS})
    (output_dir / "analysis.json").write_text(
        json.dumps(analysis, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "report.md").write_text(_render_report(analysis), encoding="utf-8")
    template = {
        "schema_version": 1,
        "dataset_type": "real",
        "trial_set_id": "<unique ID for this parameter set and experiment group>",
        "description": "Manual template only. No observations are included; fill each of exactly three attempts per profile from the operator's observation.",
        "runs": [],
        "run_entry_template": {
            "trial_set_id": "<same value as manifest trial_set_id>",
            "run_id": "<unique physical session ID>",
            "path": "<export directory containing run.csv and run-metadata.json>",
            "variant": "none | medium | strong",
            "completed": None,
            "out_of_track": None,
            "layout_id": "<same marked layout identifier for all comparison runs>",
            "camera_build_id": "<camera firmware/build identifier>",
            "battery_voltage_v": None,
            "battery_state": "<observed battery state>",
            "lighting_id": "<same lighting condition identifier>",
            "lighting_notes": "<free-form lighting notes>",
        },
        "requirements": {
            "completed": "Replace null with explicit true/false observed boolean.",
            "out_of_track": "Replace null with explicit true/false observed boolean.",
            "attempts_per_profile": 3,
            "all_attempts_must_be_retained": True,
            "failed_attempts_cannot_be_replaced": True,
            "battery_voltage_span_v_max": 0.2,
        },
    }
    (output_dir / "manual-metadata-template.json").write_text(
        json.dumps(template, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, help="manual trial manifest JSON")
    parser.add_argument("--artifacts", required=True, help="control ARTIFACTS.json SHA/build-ID whitelist")
    parser.add_argument("--output-dir", required=True, help="directory for analysis.json, summary.csv, report.md")
    parser.add_argument("--diagnostics-root", help="optional export directory tree to list failed exports only")
    parser.add_argument("--startup-exclude-ms", type=int, help="override default 3 * frozen log period")
    parser.add_argument("--max-gap-ms", type=int, help="override default 3 * frozen log period")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        manifest_path = Path(args.manifest).expanduser().resolve()
        artifacts_path = Path(args.artifacts).expanduser().resolve()
        diagnostics_root = Path(args.diagnostics_root).expanduser().resolve() if args.diagnostics_root else None
        if args.startup_exclude_ms is not None and args.startup_exclude_ms < 0:
            raise AnalysisError("--startup-exclude-ms must be non-negative")
        if args.max_gap_ms is not None and args.max_gap_ms <= 0:
            raise AnalysisError("--max-gap-ms must be positive")
        analysis = analyze_manifest(
            manifest_path,
            artifacts_path,
            diagnostics_root,
            args.startup_exclude_ms,
            args.max_gap_ms,
        )
        _write_outputs(Path(args.output_dir).expanduser().resolve(), analysis)
        print(
            json.dumps(
                {
                    "analysis_status": analysis["analysis_status"],
                    "recommendation": analysis["recommendation"],
                    "profile_count": len(analysis["profiles"]),
                    "run_count": len(analysis["runs"]),
                    "diagnostic_count": len(analysis["export_diagnostics"]),
                    "output_dir": str(Path(args.output_dir).expanduser().resolve()),
                },
                sort_keys=True,
            )
        )
        return 0
    except AnalysisError as exc:
        print(f"analyze-differential-runs: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"analyze-differential-runs: filesystem error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
