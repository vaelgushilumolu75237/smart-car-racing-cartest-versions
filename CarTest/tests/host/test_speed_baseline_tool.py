#!/usr/bin/env python3
"""Offline synthetic tests for tools/analyze_speed_baseline.py."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOL = PROJECT_ROOT / "tools" / "analyze_speed_baseline.py"
FIELDS = [
    "session_id", "t_ms", "encoder_left_count", "encoder_right_count",
    "mode_name", "fault", "vision_usable_fresh", "vision_hold_active",
    "vision_hold_expired", "vision_latest_error_norm_milli",
    "servo_applied_us", "motor_left_applied_permille",
    "motor_right_applied_permille",
]


def write_csv(path: Path, row_count: int, *, curve_at: int = -1,
              changed_left_duty_at: int = -1, zero_counts_at: int = -1,
              reverse_counts_at: int = -1, bad_gap_at: int = -1,
              startup_duty_rows: int = 0, jitter: bool = False) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        left_total = 0
        right_total = 0
        timestamp = 0xFFFFFF00
        for index in range(row_count):
            if index > 0:
                timestamp = (timestamp + (200 if index == bad_gap_at else 50)) & 0xFFFFFFFF
                if index == zero_counts_at:
                    normalized_step = 0
                elif index == reverse_counts_at:
                    normalized_step = -10
                elif jitter:
                    normalized_step = 180 if index % 2 else 420
                else:
                    normalized_step = 300
                left_total += normalized_step
                right_total -= normalized_step
            curved = index == curve_at
            changed_duty = (index == changed_left_duty_at or
                            index < startup_duty_rows)
            writer.writerow({
                "session_id": 7,
                "t_ms": timestamp,
                "encoder_left_count": left_total,
                "encoder_right_count": right_total,
                "mode_name": "follow_running",
                "fault": 0,
                "vision_usable_fresh": 1,
                "vision_hold_active": 0,
                "vision_hold_expired": 0,
                "vision_latest_error_norm_milli": 0,
                "servo_applied_us": 1700 if curved else 1530,
                "motor_left_applied_permille": 300 if changed_duty else 500,
                "motor_right_applied_permille": 500,
            })


class SpeedBaselineToolTests(unittest.TestCase):
    def run_tool(self, csv_path: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(TOOL), str(csv_path)],
            capture_output=True, text=True, check=False,
        )

    def test_wrap_safe_flat_forward_pairs_produce_baseline_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "run.csv"
            write_csv(csv_path, 26)
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "recommendation_ready")
            self.assertEqual(report["qualified_pairs_all_segments"], 25)
            self.assertEqual(report["selected_segment_pairs"], 25)
            self.assertEqual(report["selected_segment_duration_ms"], 1250)
            self.assertEqual(report["selected_segment_left"]["median_millirps"], 100)
            self.assertEqual(report["selected_segment_right"]["median_millirps"], 99)
            self.assertEqual(report["selected_segment_left"][
                "p10_p90_dispersion_percent"], 0.0)
            self.assertIsNone(report["pi_gains"])
            self.assertIsNotNone(report["baseline_recommendation"])

    def test_startup_duty_changes_split_segments(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "ramp.csv"
            write_csv(csv_path, 30, startup_duty_rows=11)
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 3)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "insufficient_continuous_pairs")
            self.assertLess(report["selected_segment_pairs"],
                            report["minimum_pairs_required"])
            self.assertIsNone(report["baseline_recommendation"])

    def test_changed_left_duty_splits_continuous_segment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "left-duty-change.csv"
            write_csv(csv_path, 30, changed_left_duty_at=15)
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 3)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "insufficient_continuous_pairs")
            self.assertLess(report["selected_segment_pairs"],
                            report["minimum_pairs_required"])

    def test_zero_reverse_and_bad_gap_never_join_a_steady_segment(self) -> None:
        for scenario in ("zero", "reverse", "gap"):
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as directory:
                csv_path = Path(directory) / "broken.csv"
                kwargs = {
                    "zero_counts_at": 12 if scenario == "zero" else -1,
                    "reverse_counts_at": 12 if scenario == "reverse" else -1,
                    "bad_gap_at": 12 if scenario == "gap" else -1,
                }
                write_csv(csv_path, 30, **kwargs)
                result = self.run_tool(csv_path)
                self.assertEqual(result.returncode, 3)
                report = json.loads(result.stdout)
                self.assertNotEqual(report["status"], "recommendation_ready")
                self.assertIsNone(report["baseline_recommendation"])

    def test_high_dispersion_is_not_a_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "jitter.csv"
            write_csv(csv_path, 30, jitter=True)
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 3)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "unstable_speed_segment")
            self.assertIsNone(report["baseline_recommendation"])

    def test_insufficient_rows_do_not_emit_recommendation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "short.csv"
            write_csv(csv_path, 4)
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 3)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "insufficient_continuous_pairs")
            self.assertLess(report["qualified_pairs_all_segments"],
                            report["minimum_pairs_required"])
            self.assertIsNone(report["pi_gains"])

    def test_missing_fields_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "bad.csv"
            csv_path.write_text("t_ms,mode_name\n0,follow_running\n",
                                encoding="utf-8")
            result = self.run_tool(csv_path)
            self.assertEqual(result.returncode, 2)
            self.assertIn("missing required fields", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
