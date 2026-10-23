"""Evidence-bound clock/session and latest-hold matching tests."""

import sys
import csv
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


def encode_control_extension(sequence, capture_ms, send_ms, sent_control,
                             *, flags=0x41, position_p=25,
                             derivative_d=-5, pd_output=20,
                             preview_candidate=31, preview_gain_milli=750,
                             preview_applied=29, capture_dt_ms=16,
                             preview_state=2, reset_code=0):
    record = bytearray(32)

    def put_u16(offset, value):
        record[offset:offset + 2] = int(value).to_bytes(2, "little", signed=False)

    def put_i16(offset, value):
        record[offset:offset + 2] = int(value).to_bytes(2, "little", signed=True)

    def put_u32(offset, value):
        record[offset:offset + 4] = int(value).to_bytes(4, "little", signed=False)

    record[0] = 2
    record[1] = flags
    put_u16(2, sequence)
    put_u32(4, capture_ms)
    put_u32(8, send_ms)
    put_i16(12, position_p)
    put_i16(14, derivative_d)
    put_i16(16, pd_output)
    put_i16(18, preview_candidate)
    put_u16(20, preview_gain_milli)
    put_i16(22, preview_applied)
    put_u16(24, capture_dt_ms)
    record[26] = preview_state
    record[27] = reset_code
    put_i16(28, sent_control)
    put_u16(30, analyzer._crc16_ccitt_false(record[:30]))
    return record.hex()


def encode_curve_trace(sequence, valid, error, phase=0, slowdown=0,
                       *, version=2, reserved=0):
    raw = bytearray(16)
    raw[:4] = bytes((0xA5, 0x5A, version, 8))
    raw[4:6] = int(sequence).to_bytes(2, "little")
    raw[6] = 1 if valid else 0
    raw[7] = 80 if valid else 0
    raw[8:10] = int(error).to_bytes(2, "little", signed=True)
    raw[10] = phase
    raw[11] = reserved
    raw[12:14] = int(slowdown).to_bytes(2, "little")
    raw[14:16] = analyzer._crc16_ccitt_false(raw[2:14]).to_bytes(2, "little")
    return raw.hex()


def write_camera_trace(path, cameras, extensions=None, curve_traces=None):
    fields = list(analyzer.CAMERA_FIELDS)
    if extensions is not None:
        fields.append(analyzer.CONTROL_EXTENSION_V2_COLUMN)
    if curve_traces is not None:
        fields.append(analyzer.CURVE_TRACE_V2_COLUMN)
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write("# ticks_period_ms=256 count={} record_errors=0\n".format(
            len(cameras)))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, camera in enumerate(cameras):
            row = {name: camera[name] for name in analyzer.CAMERA_FIELDS}
            if extensions is not None:
                row[analyzer.CONTROL_EXTENSION_V2_COLUMN] = extensions[index]
            if curve_traces is not None:
                row[analyzer.CURVE_TRACE_V2_COLUMN] = curve_traces[index]
            writer.writerow(row)


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

    def test_control_extension_v2_crc_and_base_row_binding(self):
        self.assertEqual(analyzer._crc16_ccitt_false(b"123456789"), 0x29B1)
        base = {"sequence": 7, "capture_ms": 100, "send_ms": 108,
                "sent_error_norm": -35}
        valid = encode_control_extension(7, 100, 108, -35)
        decoded, issues, present = analyzer._decode_control_extension_v2(valid, base)
        self.assertTrue(present)
        self.assertEqual(issues, [])
        self.assertEqual(decoded["position_p"], 25)
        self.assertEqual(decoded["derivative_d"], -5)
        self.assertEqual(decoded["pd_output"], 20)
        self.assertEqual(decoded["preview_candidate"], 31)
        self.assertEqual(decoded["preview_applied"], 29)
        self.assertEqual(decoded["sent_control"], -35)

        mismatches = (
            ("sequence", encode_control_extension(8, 100, 108, -35)),
            ("capture_ms", encode_control_extension(7, 101, 108, -35)),
            ("send_ms", encode_control_extension(7, 100, 109, -35)),
            ("sent_control", encode_control_extension(7, 100, 108, -34)),
        )
        for field, extension in mismatches:
            with self.subTest(field=field):
                _decoded, row_issues, _present = analyzer._decode_control_extension_v2(
                    extension, base)
                expected = ("sent_control" if field == "sent_control" else field)
                self.assertTrue(any(expected in issue for issue in row_issues),
                                row_issues)

        corrupt = bytearray.fromhex(valid)
        corrupt[30] ^= 0x01
        _decoded, crc_issues, _present = analyzer._decode_control_extension_v2(
            corrupt.hex(), base)
        self.assertIn("extension CRC16/CCITT-FALSE mismatch", crc_issues)

    def test_curve_trace_v2_decodes_raw_uart_frame_and_binds_row(self):
        base = {"sequence": 7, "valid": True, "sent_error_norm": -35}
        wire = encode_curve_trace(7, True, -35, phase=2, slowdown=725)
        decoded, issues, present = analyzer._decode_curve_trace_v2(wire, base)
        self.assertTrue(present)
        self.assertEqual(issues, [])
        self.assertEqual(decoded["curve_phase"], 2)
        self.assertEqual(decoded["slowdown_request_permille"], 725)
        self.assertEqual(decoded["lateral_error"], -35)
        self.assertEqual(decoded["sequence"], 7)

        damaged = bytearray.fromhex(wire)
        damaged[14] ^= 1
        _decoded, issues, present = analyzer._decode_curve_trace_v2(
            damaged.hex(), base)
        self.assertTrue(present)
        self.assertIn("frame CRC16/CCITT-FALSE mismatch", " ".join(issues))

    def test_invalid_curve_trace_keeps_nonzero_transport_sequence(self):
        base = {"sequence": 65535, "valid": False, "sent_error_norm": 0}
        wire = encode_curve_trace(65535, False, 0)
        decoded, issues, present = analyzer._decode_curve_trace_v2(wire, base)
        self.assertTrue(present)
        self.assertEqual(issues, [])
        self.assertFalse(decoded["frame_valid"])
        self.assertEqual(decoded["sequence"], 65535)

    def test_curve_trace_integration_preserves_fields_and_classifies_partial(self):
        _meta, cameras, status, h743 = self._valid_inputs()
        traces = [
            encode_curve_trace(65535, True, 20, phase=1, slowdown=250),
            encode_curve_trace(0, True, -50, phase=2, slowdown=700),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "camera.csv"
            write_camera_trace(trace, cameras, curve_traces=traces)
            camera_meta, loaded = analyzer.load_camera_trace(trace)
            report, matches = analyzer.analyze(camera_meta, loaded, status, h743)
        trace_report = report["camera_trace"]["curve_trace_v2"]
        self.assertEqual(trace_report["status"], "valid")
        self.assertEqual(trace_report["valid_records"], 2)
        self.assertEqual(matches[0]["curve_trace_v2_curve_phase"], 1)
        self.assertEqual(matches[1]["curve_trace_v2_slowdown_request_permille"], 700)

        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "camera.csv"
            write_camera_trace(trace, cameras, curve_traces=[traces[0], ""])
            camera_meta, loaded = analyzer.load_camera_trace(trace)
            report, _ = analyzer.analyze(camera_meta, loaded, status, h743)
        self.assertEqual(report["camera_trace"]["curve_trace_v2"]["status"],
                         "partial")
        self.assertEqual(report["status"], "incomplete_camera_trace")

    def test_curve_trace_base_binding_error_is_classified_incomplete(self):
        _meta, cameras, status, h743 = self._valid_inputs()
        traces = [encode_curve_trace(1, True, 20),
                  encode_curve_trace(0, True, -50)]
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "camera.csv"
            write_camera_trace(trace, cameras, curve_traces=traces)
            camera_meta, loaded = analyzer.load_camera_trace(trace)
            report, matches = analyzer.analyze(camera_meta, loaded, status, h743)
        self.assertEqual(report["status"], "incomplete_camera_trace")
        self.assertFalse(matches[0]["curve_trace_v2_valid"])
        self.assertIn("sequence does not match base row",
                      matches[0]["curve_trace_v2_issues"])

    def test_control_extension_v2_fields_flow_to_matched_rows(self):
        meta, cameras, status, h743 = self._valid_inputs()
        extensions = [
            encode_control_extension(65535, 248, 250, 20,
                                     position_p=40, derivative_d=-10,
                                     pd_output=30, preview_candidate=45,
                                     preview_applied=43),
            encode_control_extension(0, 2, 5, -50,
                                     position_p=-30, derivative_d=-20,
                                     pd_output=-50, preview_candidate=-42,
                                     preview_applied=-40, flags=0x82,
                                     preview_state=1, reset_code=3),
        ]
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "camera.csv"
            matched_path = Path(directory) / "matched.csv"
            write_camera_trace(trace_path, cameras, extensions)
            camera_meta, loaded_cameras = analyzer.load_camera_trace(trace_path)
            report, matched = analyzer.analyze(camera_meta, loaded_cameras,
                                               status, h743)
            analyzer._write_matched_csv(matched_path, matched)
            with matched_path.open("r", encoding="utf-8", newline="") as stream:
                exported_matches = list(csv.DictReader(stream))

        self.assertEqual(report["status"], "complete")
        extension_report = report["camera_trace"]["control_extension_v2"]
        self.assertEqual(extension_report["status"], "valid")
        self.assertEqual(extension_report["valid_records"], 2)
        self.assertEqual(extension_report["invalid_records"], 0)
        self.assertEqual(extension_report["ranges"]["position_p"]["min"], -30)
        by_sequence = {row["sequence"]: row for row in matched}
        self.assertEqual(by_sequence[65535]["control_ext_v2_position_p"], 40)
        self.assertEqual(by_sequence[65535]["control_ext_v2_derivative_d"], -10)
        self.assertEqual(by_sequence[65535]["control_ext_v2_pd_output"], 30)
        self.assertEqual(by_sequence[65535]["control_ext_v2_preview_candidate"], 45)
        self.assertEqual(by_sequence[65535]["control_ext_v2_preview_applied"], 43)
        self.assertEqual(by_sequence[65535]["control_ext_v2_sent_control"], 20)
        self.assertTrue(by_sequence[0]["control_ext_v2_valid"])
        exported = next(row for row in exported_matches if row["sequence"] == "65535")
        self.assertEqual(exported["control_ext_v2_pd_output"], "30")
        self.assertEqual(exported["control_ext_v2_preview_candidate"], "45")
        self.assertEqual(exported["control_ext_v2_sent_control"], "20")

    def test_legacy_camera_trace_without_control_extension_remains_complete(self):
        _meta, cameras, status, h743 = self._valid_inputs()
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "legacy-camera.csv"
            write_camera_trace(trace_path, cameras)
            camera_meta, loaded_cameras = analyzer.load_camera_trace(trace_path)
            report, matched = analyzer.analyze(camera_meta, loaded_cameras,
                                               status, h743)

        self.assertEqual(report["status"], "complete")
        self.assertEqual(report["camera_trace"]["control_extension_v2"]["status"],
                         "legacy_absent")
        self.assertEqual(report["camera_trace"]["control_extension_v2"]["present_records"], 0)
        self.assertTrue(matched)
        self.assertIsNone(matched[0]["control_ext_v2_position_p"])

    def test_invalid_control_extension_marks_trace_incomplete(self):
        _meta, cameras, status, h743 = self._valid_inputs()
        extension = bytearray.fromhex(
            encode_control_extension(65535, 248, 250, 20))
        extension[30] ^= 0x01
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "bad-extension-camera.csv"
            write_camera_trace(trace_path, cameras,
                               [extension.hex(), encode_control_extension(0, 2, 5, -50)])
            camera_meta, loaded_cameras = analyzer.load_camera_trace(trace_path)
            report, _matched = analyzer.analyze(camera_meta, loaded_cameras,
                                                status, h743)

        self.assertEqual(report["status"], "incomplete_camera_trace")
        self.assertEqual(report["camera_trace"]["control_extension_v2"]["invalid_records"], 1)
        self.assertTrue(any("control extension V2" in warning
                            for warning in report["camera_trace"]["evidence_warnings"]))

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
