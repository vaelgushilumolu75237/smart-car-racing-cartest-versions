#!/usr/bin/env python3
"""Read-only acceptance check for OpenMV board and IDE preview FPS logs.

Board rates are derived only from V7_PERF_WINDOW counters and their board-side
start_ms/end_ms interval. Host process timing and video playback FPS are never
used as substitutes for those counters. IDE preview evidence is a separate CSV
of distinct newly displayed images, aligned to each board 10-second window.

The --board-evidence option is an operator attestation, not cryptographic
authentication. This script reads evidence; it does not create or alter board
logs, connect to a board, or measure live camera behavior.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import re
import sys
from pathlib import Path
from typing import Any


SCHEMA = "openmv-fps-acceptance-v1"
WINDOW_MS = 10_000
MIN_REQUIRED_WINDOWS = 6
STRICT_FPS = 60
BOARD_PREFIX = "V7_PERF_WINDOW "
BOARD_REQUIRED_FIELDS = (
    "run", "index", "start_ms", "end_ms", "distinct_captures",
    "write_attempts", "full_uart_writes", "valid_frames", "uart_write_errors",
)
PREVIEW_REQUIRED_COLUMNS = (
    "start_ms", "end_ms", "distinct_preview_frames",
)


def _parse_nonnegative_int(value: str) -> int:
    if not re.fullmatch(r"\d+", value):
        raise ValueError("expected a non-negative integer")
    return int(value)


def _parse_board_line(line: str, line_number: int) -> dict[str, Any]:
    payload = line[len(BOARD_PREFIX):]
    tokens: dict[str, str] = {}
    duplicate_keys: list[str] = []
    for token in payload.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key in tokens:
            duplicate_keys.append(key)
        tokens[key] = value

    missing = [key for key in BOARD_REQUIRED_FIELDS if key not in tokens]
    fields: dict[str, int] = {}
    invalid: list[str] = []
    for key in BOARD_REQUIRED_FIELDS:
        if key not in tokens:
            continue
        try:
            fields[key] = _parse_nonnegative_int(tokens[key])
        except ValueError:
            invalid.append(key)

    reasons: list[str] = []
    if missing:
        reasons.append("missing_required_fields:" + ",".join(missing))
    if invalid:
        reasons.append("invalid_integer_fields:" + ",".join(invalid))
    if duplicate_keys:
        reasons.append("duplicate_fields:" + ",".join(sorted(set(duplicate_keys))))

    window: dict[str, Any] = {
        "line_number": line_number,
        "fields": fields,
        "status": "INCOMPLETE" if reasons else "INCOMPLETE",
        "metric_status": "INCOMPLETE" if reasons else "INCOMPLETE",
        "reasons": reasons,
    }
    if not reasons:
        start_ms = fields["start_ms"]
        end_ms = fields["end_ms"]
        duration_ms = end_ms - start_ms
        window["duration_ms"] = duration_ms
        if duration_ms != WINDOW_MS:
            window["reasons"].append("window_duration_not_10000_ms")
        if fields["valid_frames"] > fields["distinct_captures"]:
            window["reasons"].append("valid_frames_exceed_distinct_captures")
        if fields["full_uart_writes"] > fields["write_attempts"]:
            window["reasons"].append("full_uart_writes_exceed_write_attempts")

        if window["reasons"]:
            window["metric_status"] = "FAIL"
        else:
            capture_rate = fields["distinct_captures"] * 1000.0 / duration_ms
            uart_rate = fields["full_uart_writes"] * 1000.0 / duration_ms
            valid_rate = fields["valid_frames"] * 1000.0 / duration_ms
            valid_ratio = (
                fields["valid_frames"] / fields["distinct_captures"]
                if fields["distinct_captures"] else None
            )
            window["rates"] = {
                "distinct_capture_fps": round(capture_rate, 6),
                "full_uart_write_fps": round(uart_rate, 6),
                "valid_frame_fps": round(valid_rate, 6),
                "valid_frame_fraction_of_captures": (
                    round(valid_ratio, 6) if valid_ratio is not None else None
                ),
                "basis": "V7_PERF_WINDOW counters divided by board-reported interval",
            }
            failures = []
            if fields["distinct_captures"] * 1000 <= STRICT_FPS * duration_ms:
                failures.append("distinct_capture_rate_not_strictly_above_60_fps")
            if fields["full_uart_writes"] * 1000 <= STRICT_FPS * duration_ms:
                failures.append("full_uart_write_rate_not_strictly_above_60_fps")
            if fields["uart_write_errors"] != 0:
                failures.append("uart_write_errors_not_zero")
            window["reasons"].extend(failures)
            window["metric_status"] = "FAIL" if failures else "PASS"

    return window


def _parse_board_log(text: str) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    windows = []
    runtime_errors = []
    traceback_open = False
    for line_number, line in enumerate(text.splitlines(), start=1):
        if line.startswith(BOARD_PREFIX):
            windows.append(_parse_board_line(line, line_number))
        if line.startswith("Traceback (most recent call last):"):
            if not traceback_open:
                runtime_errors.append({
                    "line_number": line_number,
                    "kind": "board_runtime_traceback",
                })
            traceback_open = True
        elif traceback_open and line.strip():
            if re.match(r"^[A-Za-z_][A-Za-z0-9_.]*(?:Error|Exception|Interrupt|Exit)(?::|\s*$)", line):
                traceback_open = False
        elif re.match(r"^(?:ERROR|FATAL)(?::|\s|$)", line):
            runtime_errors.append({
                "line_number": line_number,
                "kind": "board_runtime_error_marker",
            })
    return windows, runtime_errors


def _assess_order_and_coverage(
    windows: list[dict[str, Any]],
) -> tuple[list[list[int]], list[dict[str, Any]], bool]:
    """Return continuous segments (window positions), issues, and hard failure."""
    issues: list[dict[str, Any]] = []
    hard_failure = False
    seen: set[tuple[int, int]] = set()
    last_by_run: dict[int, tuple[int, int, int, int]] = {}
    segments_by_run: dict[int, list[list[int]]] = {}

    for position, window in enumerate(windows):
        fields = window["fields"]
        if not all(key in fields for key in ("run", "index", "start_ms", "end_ms")):
            continue
        run_id = fields["run"]
        index = fields["index"]
        start_ms = fields["start_ms"]
        end_ms = fields["end_ms"]
        identity = (run_id, index)
        if identity in seen:
            issues.append({
                "line_number": window["line_number"],
                "kind": "duplicate_board_window",
                "run": run_id,
                "index": index,
            })
            window["reasons"].append("duplicate_run_index")
            window["metric_status"] = "FAIL"
            hard_failure = True
            continue
        seen.add(identity)

        previous = last_by_run.get(run_id)
        if previous is not None:
            prev_index, prev_start, prev_end, prev_position = previous
            if index <= prev_index or start_ms < prev_start:
                issues.append({
                    "line_number": window["line_number"],
                    "kind": "reversed_board_window_order",
                    "run": run_id,
                    "index": index,
                })
                window["reasons"].append("board_windows_not_in_increasing_order")
                window["metric_status"] = "FAIL"
                hard_failure = True
                segments_by_run.setdefault(run_id, []).append([])
            elif index == prev_index + 1 and start_ms == prev_end:
                segments = segments_by_run.setdefault(run_id, [])
                if not segments:
                    segments.append([prev_position])
                elif not segments[-1] or segments[-1][-1] != prev_position:
                    segments.append([prev_position])
                segments[-1].append(position)
            else:
                issues.append({
                    "line_number": window["line_number"],
                    "kind": "board_window_gap",
                    "run": run_id,
                    "after_index": prev_index,
                    "index": index,
                })
                segments_by_run.setdefault(run_id, []).append([position])
        else:
            segments_by_run.setdefault(run_id, []).append([position])
        last_by_run[run_id] = (index, start_ms, end_ms, position)

    segments = [segment for group in segments_by_run.values() for segment in group if segment]
    return segments, issues, hard_failure


def _parse_preview_csv(text: str) -> tuple[list[dict[str, int]], list[dict[str, Any]], bool]:
    issues: list[dict[str, Any]] = []
    rows: list[dict[str, int]] = []
    hard_failure = False
    try:
        reader = csv.DictReader(io.StringIO(text, newline=""))
    except csv.Error:
        return rows, [{"kind": "preview_csv_parse_error"}], True

    headers = reader.fieldnames or []
    if len(headers) != len(set(headers)):
        return rows, [{"kind": "duplicate_preview_csv_header"}], True
    missing_headers = [name for name in PREVIEW_REQUIRED_COLUMNS if name not in headers]
    if missing_headers:
        return rows, [{
            "kind": "missing_preview_csv_columns",
            "columns": missing_headers,
        }], True

    seen_intervals: set[tuple[int, int]] = set()
    previous_interval: tuple[int, int] | None = None
    for row_number, row in enumerate(reader, start=2):
        if not row or all(
            value is None or (isinstance(value, str) and not value.strip())
            for value in row.values()
        ):
            continue
        if None in row or any(
            value is not None and not isinstance(value, str)
            for value in row.values()
        ):
            issues.append({"row_number": row_number, "kind": "malformed_preview_csv_row"})
            hard_failure = True
            continue
        try:
            parsed = {
                key: _parse_nonnegative_int((row.get(key) or "").strip())
                for key in PREVIEW_REQUIRED_COLUMNS
            }
        except ValueError:
            issues.append({"row_number": row_number, "kind": "invalid_preview_csv_integer"})
            hard_failure = True
            continue
        start_ms = parsed["start_ms"]
        end_ms = parsed["end_ms"]
        interval = (start_ms, end_ms)
        if end_ms <= start_ms:
            issues.append({"row_number": row_number, "kind": "invalid_preview_window_interval"})
            hard_failure = True
        if interval in seen_intervals:
            issues.append({"row_number": row_number, "kind": "duplicate_preview_window"})
            hard_failure = True
        if previous_interval is not None and start_ms < previous_interval[0]:
            issues.append({"row_number": row_number, "kind": "reversed_preview_window_order"})
            hard_failure = True
        if previous_interval is not None and start_ms < previous_interval[1]:
            issues.append({"row_number": row_number, "kind": "overlapping_preview_windows"})
            hard_failure = True
        seen_intervals.add(interval)
        previous_interval = interval
        parsed["row_number"] = row_number
        rows.append(parsed)
    return rows, issues, hard_failure


def _build_preview_result(
    scenario: str,
    preview_text: str | None,
    board_windows: list[dict[str, Any]],
) -> tuple[dict[str, Any], bool]:
    if scenario == "no-preview":
        if preview_text is not None:
            return {
                "status": "FAIL",
                "windows": [],
                "issues": [{"kind": "preview_csv_conflicts_with_no_preview_scenario"}],
                "evidence_source": None,
            }, True
        return {
            "status": "NOT_REQUIRED",
            "windows": [],
            "issues": [],
            "evidence_source": None,
        }, False
    if preview_text is None:
        return {
            "status": "INCOMPLETE",
            "windows": [],
            "issues": [{"kind": "independent_preview_evidence_missing"}],
            "evidence_source": None,
        }, False

    rows, issues, hard_failure = _parse_preview_csv(preview_text)
    row_by_interval: dict[tuple[int, int], dict[str, int]] = {}
    for row in rows:
        row_by_interval[(row["start_ms"], row["end_ms"])] = row

    results = []
    covered = 0
    for board_position, board_window in enumerate(board_windows):
        fields = board_window["fields"]
        if not all(key in fields for key in ("start_ms", "end_ms")):
            results.append({
                "board_window_position": board_position,
                "status": "INCOMPLETE",
                "reason": "board_window_interval_unavailable",
            })
            continue
        interval = (fields["start_ms"], fields["end_ms"])
        row = row_by_interval.get(interval)
        if row is None:
            results.append({
                "board_run": fields.get("run"),
                "board_index": fields.get("index"),
                "start_ms": interval[0],
                "end_ms": interval[1],
                "status": "INCOMPLETE",
                "reason": "no_exactly_aligned_preview_window",
            })
            continue
        covered += 1
        duration_ms = interval[1] - interval[0]
        frame_count = row["distinct_preview_frames"]
        preview_rate = frame_count * 1000.0 / duration_ms if duration_ms > 0 else None
        passed = duration_ms > 0 and frame_count * 1000 > STRICT_FPS * duration_ms
        results.append({
            "board_run": fields.get("run"),
            "board_index": fields.get("index"),
            "start_ms": interval[0],
            "end_ms": interval[1],
            "distinct_preview_frames": frame_count,
            "distinct_preview_fps": round(preview_rate, 6) if preview_rate is not None else None,
            "status": "PASS" if passed else "FAIL",
            "reason": None if passed else "distinct_preview_rate_not_strictly_above_60_fps",
            "source_row_number": row["row_number"],
        })

    if hard_failure:
        status = "FAIL"
    elif any(window["status"] == "FAIL" for window in results):
        status = "FAIL"
    elif not rows or covered != len(board_windows):
        status = "INCOMPLETE"
    else:
        status = "PASS"
    return {
        "status": status,
        "windows": results,
        "issues": issues,
        "evidence_source": "independent_distinct_preview_frames_csv",
        "host_video_fps_used": False,
    }, hard_failure


def evaluate(
    board_log_text: str,
    *,
    scenario: str,
    preview_csv_text: str | None = None,
    board_evidence: bool = False,
    min_windows: int = MIN_REQUIRED_WINDOWS,
    board_log_sha256: str | None = None,
    board_log_path: str | None = None,
    preview_csv_sha256: str | None = None,
    preview_csv_path: str | None = None,
) -> dict[str, Any]:
    if scenario not in ("no-preview", "ide-preview"):
        raise ValueError("scenario must be no-preview or ide-preview")
    if min_windows < MIN_REQUIRED_WINDOWS:
        raise ValueError("min_windows cannot be less than 6")

    windows, runtime_errors = _parse_board_log(board_log_text)
    segments, order_issues, hard_order_failure = _assess_order_and_coverage(windows)
    longest_streak = max((len(segment) for segment in segments), default=0)
    valid_duration = longest_streak >= min_windows

    preview, preview_hard_failure = _build_preview_result(
        scenario, preview_csv_text, windows)

    for window in windows:
        metric_status = window["metric_status"]
        if not board_evidence and metric_status == "PASS":
            window["status"] = "INCOMPLETE"
            window["reasons"].append("board_source_not_attested_by_operator")
        else:
            window["status"] = metric_status

    board_issues = list(order_issues)
    if not windows:
        board_issues.append({"kind": "no_v7_perf_windows_found"})
    if not valid_duration:
        board_issues.append({
            "kind": "insufficient_contiguous_10s_windows",
            "required": min_windows,
            "observed_longest_streak": longest_streak,
        })
    if not board_evidence:
        board_issues.append({"kind": "board_source_attestation_missing"})
    if runtime_errors:
        board_issues.extend(runtime_errors)

    has_metric_failure = any(window["metric_status"] == "FAIL" for window in windows)
    if not board_evidence:
        overall = "INCOMPLETE"
    elif (has_metric_failure or hard_order_failure or runtime_errors or
          preview_hard_failure or preview["status"] == "FAIL"):
        overall = "FAIL"
    elif not valid_duration or any(window["metric_status"] == "INCOMPLETE" for window in windows):
        overall = "INCOMPLETE"
    elif preview["status"] == "INCOMPLETE":
        overall = "INCOMPLETE"
    else:
        overall = "PASS"

    if overall == "PASS":
        reasons = ["all_acceptance_checks_passed"]
    elif overall == "FAIL":
        reasons = []
        if has_metric_failure:
            reasons.append("one_or_more_board_windows_failed_metrics")
        if hard_order_failure:
            reasons.append("duplicate_or_reversed_board_windows")
        if runtime_errors:
            reasons.append("board_runtime_error_found_in_log")
        if preview_hard_failure or preview["status"] == "FAIL":
            reasons.append("preview_evidence_failed_or_malformed")
    else:
        reasons = []
        if not board_evidence:
            reasons.append("board_source_not_attested_by_operator")
        if not valid_duration:
            reasons.append("fewer_than_required_consecutive_complete_windows")
        if any(window["metric_status"] == "INCOMPLETE" for window in windows):
            reasons.append("one_or_more_board_windows_lack_required_current_fields")
        if preview["status"] == "INCOMPLETE":
            reasons.append("independent_preview_evidence_or_aligned_coverage_missing")
    return {
        "schema": SCHEMA,
        "status": overall,
        "scenario": scenario,
        "criteria": {
            "board_window_ms": WINDOW_MS,
            "minimum_consecutive_board_windows": min_windows,
            "strict_rate_threshold_fps": STRICT_FPS,
            "distinct_captures_must_be_strictly_greater_than_per_10s": 600,
            "full_uart_writes_must_be_strictly_greater_than_per_10s": 600,
            "uart_write_errors_must_equal": 0,
            "preview_windows_must_exactly_match_board_intervals": scenario == "ide-preview",
            "preview_distinct_frames_must_be_strictly_greater_than_per_10s": (
                600 if scenario == "ide-preview" else None
            ),
            "valid_frames_are_reported_separately_and_are_not_a_throughput_gate": True,
        },
        "inputs": {
            "board_log_path": board_log_path,
            "board_log_sha256": board_log_sha256,
            "v7_perf_window_records": len(windows),
            "board_evidence_attested_by_operator": bool(board_evidence),
            "board_evidence_authenticity": "not_cryptographically_verified",
            "preview_csv_path": preview_csv_path,
            "preview_csv_sha256": preview_csv_sha256,
            "host_timing_used_for_board_rates": False,
            "resolution_320x120_independently_verified": False,
        },
        "board": {
            "status": (
                "FAIL" if has_metric_failure or hard_order_failure or runtime_errors
                else "INCOMPLETE" if (
                    not board_evidence or not valid_duration or not windows or
                    any(window["metric_status"] == "INCOMPLETE" for window in windows)
                )
                else "PASS"
            ),
            "longest_consecutive_10s_streak": longest_streak,
            "continuous_segments": [
                {
                    "run": windows[segment[0]]["fields"].get("run"),
                    "indices": [windows[position]["fields"].get("index") for position in segment],
                    "duration_ms": len(segment) * WINDOW_MS,
                }
                for segment in segments
            ],
            "windows": windows,
            "issues": board_issues,
            "runtime_errors": runtime_errors,
            "validity_is_reported_separately": True,
        },
        "preview": preview,
        "reasons": reasons,
        "limitations": [
            "The board-source flag is an operator attestation; input authenticity is not cryptographically verified.",
            "V7_PERF_WINDOW does not independently encode 320x120 resolution; this report does not verify resolution.",
            "IDE preview rates use only distinct_preview_frames CSV counts and supplied intervals, never host video FPS.",
        ],
    }


def _min_windows_argument(value: str) -> int:
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if result < MIN_REQUIRED_WINDOWS:
        raise argparse.ArgumentTypeError("must be at least 6 complete 10-second windows")
    return result


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _read_optional(path: Path | None) -> tuple[str | None, str | None, str | None]:
    if path is None:
        return None, None, None
    try:
        data = path.read_bytes()
    except OSError as exc:
        return None, None, f"{type(exc).__name__}: {exc}"
    return data.decode("utf-8-sig", errors="replace"), _sha256(data), None


def _result_exit_code(status: str) -> int:
    return {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}.get(status, 2)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path,
                        help="read-only source log containing V7_PERF_WINDOW lines")
    parser.add_argument("--scenario", required=True,
                        choices=("no-preview", "ide-preview"),
                        help="explicitly identify whether IDE real-time preview was enabled")
    parser.add_argument("--preview-csv", type=Path,
                        help="independent evidence CSV: start_ms,end_ms,distinct_preview_frames")
    parser.add_argument("--min-windows", type=_min_windows_argument, default=MIN_REQUIRED_WINDOWS,
                        help="required consecutive complete 10-second windows (minimum 6; default 6)")
    parser.add_argument("--board-evidence", action="store_true",
                        help="operator attests the input log is actual board output; not cryptographic proof")
    parser.add_argument("--output", type=Path,
                        help="write JSON report here (must not be the log or preview CSV)")
    args = parser.parse_args(argv)

    log_path = args.log
    log_text, log_hash, log_error = _read_optional(log_path)
    if log_error:
        report = evaluate(
            "", scenario=args.scenario, preview_csv_text=None,
            board_evidence=args.board_evidence, min_windows=args.min_windows,
            board_log_path=str(log_path),
        )
        report["status"] = "INCOMPLETE"
        report["board"]["status"] = "INCOMPLETE"
        report["board"]["issues"].append({"kind": "board_log_unreadable", "detail": log_error})
        report["reasons"] = ["board_log_unreadable"]
    else:
        preview_text, preview_hash, preview_error = _read_optional(args.preview_csv)
        report = evaluate(
            log_text or "",
            scenario=args.scenario,
            preview_csv_text=preview_text,
            board_evidence=args.board_evidence,
            min_windows=args.min_windows,
            board_log_sha256=log_hash,
            board_log_path=str(log_path),
            preview_csv_sha256=preview_hash,
            preview_csv_path=str(args.preview_csv) if args.preview_csv else None,
        )
        if preview_error:
            report["preview"] = {
                "status": "FAIL",
                "windows": [],
                "issues": [{"kind": "preview_csv_unreadable", "detail": preview_error}],
                "evidence_source": "independent_distinct_preview_frames_csv",
                "host_video_fps_used": False,
            }
            if args.board_evidence:
                report["status"] = "FAIL"
                report["reasons"] = ["preview_evidence_failed_or_malformed"]

    if args.output:
        resolved_output = args.output.resolve()
        protected_inputs = {log_path.resolve()}
        if args.preview_csv:
            protected_inputs.add(args.preview_csv.resolve())
        if resolved_output in protected_inputs:
            parser.error("--output must not overwrite --log or --preview-csv")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        print(json.dumps(report, indent=2, sort_keys=True))
    return _result_exit_code(report["status"])


if __name__ == "__main__":
    raise SystemExit(main())
