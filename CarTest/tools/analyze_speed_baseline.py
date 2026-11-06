#!/usr/bin/env python3
"""Estimate straight-running wheel RPS from an exported CarTest CSV.

The result is an observed software baseline only; this tool does not tune PI
gains or establish physical straightness, wheel speed, or hill-load behavior.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


LEFT_CPR_DEFAULT = 59995
RIGHT_CPR_DEFAULT = 60239
LEFT_SIGN_DEFAULT = 1
RIGHT_SIGN_DEFAULT = -1
CENTER_US_DEFAULT = 1530
MAX_STEERING_OFFSET_US_DEFAULT = 40
MAX_ABS_VISION_ERROR_DEFAULT = 100
MAX_SAMPLE_GAP_MS_DEFAULT = 100
BASELINE_DUTY_PERMILLE_DEFAULT = 500
BASELINE_DUTY_TOLERANCE_PERMILLE_DEFAULT = 5
MIN_PAIRS_DEFAULT = 20
MAX_DISPERSION_PERCENT_DEFAULT = 15
REQUIRED_FIELDS = {
    "session_id", "t_ms", "encoder_left_count", "encoder_right_count",
    "mode_name", "fault", "vision_usable_fresh", "vision_hold_active",
    "vision_hold_expired", "vision_latest_error_norm_milli",
    "servo_applied_us", "motor_left_applied_permille",
    "motor_right_applied_permille",
}


def _integer(row: Dict[str, str], key: str, row_number: int) -> int:
    try:
        return int(row[key], 10)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("row {} has invalid {}".format(row_number, key)) from exc


def _qualifies(row: Dict[str, str], args: argparse.Namespace,
               row_number: int) -> bool:
    return (
        row.get("mode_name") == "follow_running" and
        _integer(row, "fault", row_number) == 0 and
        _integer(row, "vision_usable_fresh", row_number) == 1 and
        _integer(row, "vision_hold_active", row_number) == 0 and
        _integer(row, "vision_hold_expired", row_number) == 0 and
        abs(_integer(row, "vision_latest_error_norm_milli", row_number)) <=
            args.max_abs_vision_error and
        abs(_integer(row, "servo_applied_us", row_number) -
            args.center_us) <= args.max_steering_offset_us and
        abs(_integer(row, "motor_left_applied_permille", row_number) -
            args.baseline_duty_permille) <= args.baseline_duty_tolerance and
        abs(_integer(row, "motor_right_applied_permille", row_number) -
            args.baseline_duty_permille) <= args.baseline_duty_tolerance
    )


def _split_segments(rows: Sequence[Dict[str, str]], args: argparse.Namespace
                    ) -> Tuple[List[List[Dict[str, int]]], int]:
    segments: List[List[Dict[str, int]]] = []
    current: List[Dict[str, int]] = []
    previous: Optional[Tuple[Dict[str, str], int]] = None
    rejected = 0

    def finish_segment() -> None:
        nonlocal current
        if current:
            segments.append(current)
            current = []

    for index, row in enumerate(rows, start=2):
        if not _qualifies(row, args, index):
            finish_segment()
            previous = None
            continue

        if previous is None:
            previous = (row, index)
            continue

        previous_row, previous_line = previous
        session = _integer(row, "session_id", index)
        previous_session = _integer(previous_row, "session_id", previous_line)
        current_ms = _integer(row, "t_ms", index) & 0xFFFFFFFF
        previous_ms = _integer(previous_row, "t_ms", previous_line) & 0xFFFFFFFF
        elapsed_ms = (current_ms - previous_ms) & 0xFFFFFFFF
        if (session != previous_session or elapsed_ms == 0 or
                elapsed_ms > args.max_sample_gap_ms):
            rejected += 1
            finish_segment()
            previous = (row, index)
            continue

        left_delta = (
            _integer(row, "encoder_left_count", index) -
            _integer(previous_row, "encoder_left_count", previous_line)
        ) * args.left_sign
        right_delta = (
            _integer(row, "encoder_right_count", index) -
            _integer(previous_row, "encoder_right_count", previous_line)
        ) * args.right_sign
        if left_delta <= 0 or right_delta <= 0:
            rejected += 1
            finish_segment()
            previous = (row, index)
            continue

        left_millirps = (left_delta * 1_000_000) // (
            args.left_cpr * elapsed_ms)
        right_millirps = (right_delta * 1_000_000) // (
            args.right_cpr * elapsed_ms)
        current.append({
            "elapsed_ms": elapsed_ms,
            "left_millirps": left_millirps,
            "right_millirps": right_millirps,
        })
        previous = (row, index)

    finish_segment()
    return segments, rejected


def _percentile(values: Sequence[int], percent: int) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("percentile requires samples")
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _wheel_summary(values: Sequence[int]) -> Dict[str, object]:
    median = float(statistics.median(values))
    p10 = _percentile(values, 10)
    p90 = _percentile(values, 90)
    dispersion = ((p90 - p10) / median * 100.0) if median > 0 else None
    return {
        "median_millirps": int(round(median)),
        "median_rps": median / 1000.0,
        "minimum_millirps": min(values),
        "maximum_millirps": max(values),
        "p10_millirps": round(p10, 2),
        "p90_millirps": round(p90, 2),
        "p10_p90_dispersion_percent": round(dispersion, 2)
            if dispersion is not None else None,
    }


def analyze(csv_path: Path, args: argparse.Namespace) -> Dict[str, object]:
    with csv_path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        fields = set(reader.fieldnames or ())
        missing = sorted(REQUIRED_FIELDS - fields)
        if missing:
            raise ValueError("CSV is missing required fields: {}".format(
                ", ".join(missing)))
        rows = list(reader)

    segments, rejected = _split_segments(rows, args)
    longest = max(segments, key=lambda value: (len(value),
                  sum(item["elapsed_ms"] for item in value)), default=[])
    enough = len(longest) >= args.min_pairs
    if longest:
        left = [item["left_millirps"] for item in longest]
        right = [item["right_millirps"] for item in longest]
        left_summary = _wheel_summary(left)
        right_summary = _wheel_summary(right)
        duration_ms = sum(item["elapsed_ms"] for item in longest)
        dispersion_ok = (
            left_summary["p10_p90_dispersion_percent"] is not None and
            right_summary["p10_p90_dispersion_percent"] is not None and
            left_summary["p10_p90_dispersion_percent"] <=
                args.max_dispersion_percent and
            right_summary["p10_p90_dispersion_percent"] <=
                args.max_dispersion_percent
        )
    else:
        left_summary = None
        right_summary = None
        duration_ms = 0
        dispersion_ok = False

    if not enough:
        status = "insufficient_continuous_pairs"
    elif not dispersion_ok:
        status = "unstable_speed_segment"
    else:
        status = "recommendation_ready"
    recommendation = None
    if status == "recommendation_ready":
        recommendation = {
            "left_baseline_rps": left_summary["median_rps"],
            "right_baseline_rps": right_summary["median_rps"],
            "basis": "longest continuous qualifying segment median",
        }

    return {
        "status": status,
        "csv": str(csv_path.resolve()),
        "qualified_segments": len(segments),
        "qualified_pairs_all_segments": sum(len(segment) for segment in segments),
        "selected_segment_pairs": len(longest),
        "selected_segment_duration_ms": duration_ms,
        "minimum_pairs_required": args.min_pairs,
        "rejected_or_discontinuous_pairs": rejected,
        "filters": {
            "mode_name": "follow_running",
            "fault": 0,
            "vision_usable_fresh": 1,
            "hold_active_or_expired": 0,
            "max_abs_vision_error_norm_milli": args.max_abs_vision_error,
            "servo_center_us": args.center_us,
            "max_servo_offset_us": args.max_steering_offset_us,
            "software_duty_permille": args.baseline_duty_permille,
            "duty_tolerance_permille": args.baseline_duty_tolerance,
            "max_pair_gap_ms": args.max_sample_gap_ms,
            "max_p10_p90_dispersion_percent": args.max_dispersion_percent,
            "left_counts_per_rev": args.left_cpr,
            "left_sign": args.left_sign,
            "right_counts_per_rev": args.right_cpr,
            "right_sign": args.right_sign,
        },
        "selected_segment_left": left_summary,
        "selected_segment_right": right_summary,
        "baseline_recommendation": recommendation,
        "pi_gains": None,
        "note": "This is a software-filtered observed wheel-speed baseline. It does not prove physical straightness, recommend PI gains, or establish hill-load performance.",
    }


def _positive(value: str) -> int:
    parsed = int(value, 10)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def _signed(value: str) -> int:
    parsed = int(value, 10)
    if parsed not in (-1, 1):
        raise argparse.ArgumentTypeError("must be -1 or 1")
    return parsed


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="successful runtime_log_export.py CSV")
    parser.add_argument("--left-cpr", type=_positive, default=LEFT_CPR_DEFAULT)
    parser.add_argument("--right-cpr", type=_positive, default=RIGHT_CPR_DEFAULT)
    parser.add_argument("--left-sign", type=_signed, default=LEFT_SIGN_DEFAULT)
    parser.add_argument("--right-sign", type=_signed, default=RIGHT_SIGN_DEFAULT)
    parser.add_argument("--center-us", type=int, default=CENTER_US_DEFAULT)
    parser.add_argument("--max-steering-offset-us", type=_positive,
                        default=MAX_STEERING_OFFSET_US_DEFAULT)
    parser.add_argument("--max-abs-vision-error", type=_positive,
                        default=MAX_ABS_VISION_ERROR_DEFAULT)
    parser.add_argument("--max-sample-gap-ms", type=_positive,
                        default=MAX_SAMPLE_GAP_MS_DEFAULT)
    parser.add_argument("--baseline-duty-permille", type=int,
                        default=BASELINE_DUTY_PERMILLE_DEFAULT)
    parser.add_argument("--baseline-duty-tolerance", type=int,
                        default=BASELINE_DUTY_TOLERANCE_PERMILLE_DEFAULT)
    parser.add_argument("--min-pairs", type=_positive, default=MIN_PAIRS_DEFAULT)
    parser.add_argument("--max-dispersion-percent", type=_positive,
                        default=MAX_DISPERSION_PERCENT_DEFAULT)
    parser.add_argument("--json-out", type=Path,
                        help="optional path to write the same JSON report")
    args = parser.parse_args(argv)
    if args.max_sample_gap_ms >= 0x80000000:
        parser.error("--max-sample-gap-ms must be below the u32 half-range")
    if not (0 <= args.baseline_duty_permille <= 700 and
            0 <= args.baseline_duty_tolerance <= 700 and
            args.baseline_duty_permille + args.baseline_duty_tolerance <= 700):
        parser.error("baseline duty and tolerance must remain within 0..700")
    if args.max_dispersion_percent > 100:
        parser.error("--max-dispersion-percent must be at most 100")
    if not args.csv.is_file():
        print(json.dumps({"status": "failed", "error": "CSV file does not exist",
                          "csv": str(args.csv)}, ensure_ascii=False),
              file=sys.stderr)
        return 2
    try:
        result = analyze(args.csv, args)
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        print(json.dumps({"status": "failed", "error": str(exc),
                          "csv": str(args.csv)}, ensure_ascii=False),
              file=sys.stderr)
        return 2
    serialized = json.dumps(result, ensure_ascii=False, indent=2)
    print(serialized)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(serialized + "\n", encoding="utf-8")
    return 0 if result["status"] == "recommendation_ready" else 3


if __name__ == "__main__":
    raise SystemExit(main())
