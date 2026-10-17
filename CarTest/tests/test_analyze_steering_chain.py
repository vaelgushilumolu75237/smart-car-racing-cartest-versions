"""Evidence-bound clock/session and latest-hold matching tests."""

import sys
import json
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import analyze_steering_chain as analyzer  # noqa: E402


def camera_row(sequence, capture_ms, send_ms, error, block=0, index=0):
    return {
        "sequence": sequence,
        "capture_ms": capture_ms,
        "send_ms": send_ms,
        "valid": True,
        "full_write": True,
        "write_error": 0,
        "reverse_fresh": True,
        "position_norm": error,
        "preview_candidate_norm": error,
        "preview_applied_norm": error,
        "sent_error_norm": error,
        "raw_control_norm": error,
        "filtered_error_norm": error,
        "reference_error_norm": error,
        "row_count": 1,
        "source_code": 1,
        "reverse_generation": 1,
        "_row_index": index,
        "_trace_block": block,
        "_trace_period_ms": 256,
    }


def h743_row(sequence, elapsed, error, *, hold_enter=False, index=0):
    start_ms = 1000
    return {
        "t_ms": (start_ms + elapsed) & 0xFFFFFFFF,
        "t_elapsed_ms": elapsed,
        "session_id": 7,
        "vision_sequence": sequence,
        "vision_latest_valid": True,
        "vision_latest_error_norm_milli": error,
        "mode": 12 if hold_enter else 10,
        "servo_requested_us": 1600,
        "servo_limited_us": 1590,
        "servo_applied_us": 1580,
        "event_hold_enter": hold_enter,
        "_row_index": index,
    }


def make_status(snapshot_sequence=0, snapshot_entered=1100, snapshot_source=1090):
    return {
        "elf": {"build_id": "aabb", "log_version": 1, "path": "CarTest.elf"},
        "runtime_log_header": {
            "header_crc_status": "valid",
            "start_ms": 1000,
            "stop_ms": 1300,
            "session_id": 7,
            "version": 1,
            "period_ms": 50,
        },
        "scalars": {
            "car_vision_hold_snapshot_entry": 1,
            "car_vision_hold_snapshot_error_norm": -50,
            "car_vision_hold_snapshot_sequence": snapshot_sequence,
            "car_vision_hold_snapshot_source_rx_ms": snapshot_source,
            "car_vision_hold_snapshot_entered_ms": snapshot_entered,
            "car_vision_hold_snapshot_target_us": 1900,
            "car_vision_hold_snapshot_applied_us": 1600,
            "car_vision_hold_snapshot_left_permille": 120,
            "car_vision_hold_snapshot_right_permille": 180,
        },
    }


def make_camera_meta(block_count=1, record_errors=0):
    return {
        "source": "synthetic-camera.csv",
        "ticks_period_ms": 256 if block_count == 1 else None,
        "blocks": [
            {"ticks_period_ms": 256, "record_errors": record_errors,
             "count_matches_rows": True, "block_number": index}
            for index in range(block_count)
        ],
    }


class AnalyzeSteeringChainTests(unittest.TestCase):
    def _valid_inputs(self):
        cameras = [camera_row(65535, 248, 250, 20, index=0),
                   camera_row(0, 2, 5, -50, index=1)]
        h743 = [h743_row(65535, 0, 20, index=0),
                h743_row(0, 100, -50, hold_enter=True, index=1),
                h743_row(0, 150, -50, index=2)]
        return make_camera_meta(), cameras, make_status(), h743

    def test_camera_u16_sequence_and_declared_tick_period_wrap_independently(self):
        meta, cameras, status, h743 = self._valid_inputs()
        report, matches = analyzer.analyze(meta, cameras, status, h743)
        self.assertEqual(cameras[1]["camera_session"], cameras[0]["camera_session"])
        self.assertEqual(cameras[1]["sequence_unwrapped"], 65536)
        self.assertEqual(cameras[1]["send_unwrapped_ms"], 261)
        self.assertEqual(report["sequence_comparison"]["matched_unique_sequences"], 2)
        self.assertEqual(len(matches), 3)
        self.assertIsNone(report["clock_alignment"]["estimate"])
        self.assertIn("no cross-clock subtraction", report["clock_alignment"]["uncertainty"])

    def test_hold_snapshot_matches_only_unique_latest_entry_near_its_time_and_sequence(self):
        meta, cameras, status, h743 = self._valid_inputs()
        report, _matches = analyzer.analyze(meta, cameras, status, h743)
        snapshot = report["hold_snapshot"]
        self.assertTrue(snapshot["entry_matches_h743_log"])
        self.assertTrue(snapshot["snapshot_matches_latest_h743_hold"])
        self.assertEqual(snapshot["entry_match_count"], 1)
        self.assertEqual(snapshot["h743_entry_row_index"], 1)
        self.assertEqual(snapshot["camera_source_match"], "unique_valid_full_send")
        self.assertEqual(report["camera_trace"]["selected_session"], 0)

    def test_snapshot_is_refused_when_a_later_h743_hold_exists(self):
        meta, cameras, status, h743 = self._valid_inputs()
        later = h743_row(1, 200, 10, hold_enter=True, index=3)
        h743.append(later)
        report, _matches = analyzer.analyze(meta, cameras, status, h743)
        snapshot = report["hold_snapshot"]
        self.assertFalse(snapshot["entry_matches_h743_log"])
        self.assertFalse(snapshot["snapshot_matches_latest_h743_hold"])
        self.assertIsNone(snapshot["h743_entry_error_matches"])
        self.assertNotIn("anchored_by_unique_hold_snapshot_sequence",
                         report["camera_trace"]["session_selection_reason"])

    def test_out_of_range_published_snapshot_is_unusable_and_cannot_attribute_hold(self):
        meta, cameras, status, h743 = self._valid_inputs()
        status["scalars"]["car_vision_hold_snapshot_target_us"] = 2700
        report, _matches = analyzer.analyze(meta, cameras, status, h743)
        snapshot = report["hold_snapshot"]
        self.assertTrue(snapshot["published"])
        self.assertFalse(snapshot["usable"])
        self.assertFalse(snapshot.get("snapshot_matches_latest_h743_hold", False))
        self.assertNotIn("camera_source_match", snapshot)
        self.assertTrue(any("hold snapshot values" in warning
                            for warning in report["camera_trace"]["evidence_warnings"]))
        self.assertIn("target_us is outside [1100, 2500]", snapshot["warnings"])

    def test_duplicate_sequence_across_camera_sessions_is_ambiguous(self):
        cameras = [camera_row(0, 2, 5, -50, block=0, index=0),
                   camera_row(0, 2, 5, -50, block=1, index=1)]
        meta = make_camera_meta(block_count=2)
        report, _matches = analyzer.analyze(
            meta, cameras, make_status(), [h743_row(0, 100, -50, hold_enter=True, index=0)])
        self.assertEqual(report["hold_snapshot"]["camera_source_match"],
                         "ambiguous_duplicate_sequence")
        self.assertIsNone(report["camera_trace"]["selected_session"])
        self.assertIsNone(report["clock_alignment"]["estimate"])

    def test_camera_record_errors_are_reported_as_incomplete_evidence(self):
        meta, cameras, status, h743 = self._valid_inputs()
        meta["blocks"][0]["record_errors"] = 2
        report, _matches = analyzer.analyze(meta, cameras, status, h743)
        self.assertEqual(report["status"], "incomplete_camera_trace")
        self.assertEqual(report["camera_trace"]["record_errors"], 2)
        self.assertTrue(any("record parsing reported errors" in warning
                            for warning in report["camera_trace"]["evidence_warnings"]))

    def test_successful_export_with_later_status_warning_remains_analyzable(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "run.csv"
            csv_path.write_text("synthetic", encoding="utf-8")
            metadata = {
                "status": "success_with_warnings",
                "diagnostic_failure": "status wrapper could not write a follow-up snapshot",
                "log_version": 1,
                "holdQuantum_ms": 20,
                "build_id": "aabb",
                "initial_header": {"session_id": 7, "version": 1},
                "record_count": 3,
            }
            (Path(directory) / "run-metadata.json").write_text(
                json.dumps(metadata), encoding="utf-8")
            status = {
                "elf": {"build_id": "aabb"},
                "runtime_log_header": {"version": 1, "session_id": 7},
            }
            result = analyzer._validate_export_metadata(csv_path, status, [{}, {}, {}])
            self.assertEqual(result["status"], "matched")
            self.assertTrue(result["checked"])
            self.assertTrue(any("status snapshot failed or warned" in warning
                                for warning in result["warnings"]))

            status["runtime_log_header"]["session_id"] = 8
            with self.assertRaisesRegex(analyzer.AnalysisError, "session ID"):
                analyzer._validate_export_metadata(csv_path, status, [{}, {}, {}])


if __name__ == "__main__":
    unittest.main(verbosity=2)
