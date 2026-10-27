"""Host tests for the read-only OpenMV ring-diagnostic exporter."""

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOOLS_ROOT = PROJECT_ROOT / "tools"
V2_RUNTIME_ROOT = (
    PROJECT_ROOT / "logs" / "ring45-usb60-20261026" /
    "pipeline_perf_v8_ringdiag_autosave_v1")
sys.path.insert(0, str(TOOLS_ROOT))

import export_openmv_ring_diag as exporter


def _load_v2_runtime():
    module_path = V2_RUNTIME_ROOT / "ring_diagnostics.py"
    spec = importlib.util.spec_from_file_location(
        "_ring_diag_export_test_runtime", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RING = _load_v2_runtime()


def _make_v2(candidate_episode=False):
    diagnostic = RING.RingDiagnostics()
    diagnostic.begin_session(100)
    common_selection = {"source": "ring_right", "reason": "OK"}
    diagnostic.record_frame(
        100, 0xFFFE, 3, True, True, common_selection, {})
    tracker = {}
    if candidate_episode:
        tracker = {
            "candidate": True,
            "ring_confirmed": True,
            "state_id": 1,
            "reason": "OBSERVING",
            "reject_reason": "INPUT_INVALID",
            "reject_counts": list(range(13)),
            "ring_diagnostic_record": list(range(14)),
            "ring_diagnostic_stride": 14,
        }
    diagnostic.record_frame(
        160, 0xFFFF, 3, True, True, common_selection, tracker)
    diagnostic.freeze("PROFILE_INACTIVE")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "fixture.bin"
        diagnostic.save(path, "RING_DIAG_EXPORT_TEST")
        return path.read_bytes()


def _make_v1(data):
    converted = bytearray(data)
    header_size = 32
    payload = bytearray(converted[header_size:])
    # V1 stored the same signed field directly as elapsed milliseconds.
    metadata_offset = RING.BUILD_ID_BYTES + 3
    record_offset = metadata_offset + RING.EPISODE_META_SIZE
    payload[metadata_offset + 22:metadata_offset + 24] = b"\x00\x00"
    struct.pack_into("<h", payload, record_offset, 123)
    struct.pack_into(
        "<h", payload, record_offset + RING.FRAME_RECORD_SIZE, 999)
    struct.pack_into("<H", converted, 4, 1)
    converted[header_size:] = payload
    struct.pack_into("<I", converted, 28, zlib.crc32(payload) & 0xFFFFFFFF)
    return bytes(converted)


def _empty_v1():
    payload = bytearray(RING.BUILD_ID_BYTES + 3)
    payload[0] = 0
    header = struct.pack(
        "<4s10H2I", b"RNGD", 1, RING.HEADER_SIZE,
        RING.FRAME_RECORD_SIZE, RING.REJECT_COUNT, RING.EPISODE_CAPACITY,
        0, RING.PRE_FRAMES, RING.POST_FRAMES, 0, 1, len(payload),
        zlib.crc32(payload) & 0xFFFFFFFF)
    return header + payload


class ExportOpenMVRingDiagTests(unittest.TestCase):
    def _export(self, source, output, mode="input"):
        return exporter.export(
            input_path=str(source) if mode == "input" else None,
            directory=str(source) if mode == "directory" else None,
            output_dir=str(output), runtime_root=str(V2_RUNTIME_ROOT))

    @staticmethod
    def _read_csv(path):
        with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
            return list(csv.DictReader(stream))

    def test_v2_stop_snapshot_exports_nonempty_rows_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source_dir = root / "camera_mount"
            output_dir = root / "private_logs"
            source_dir.mkdir()
            source = source_dir / "ring_diagnostics_000001.bin"
            source.write_bytes(_make_v2())
            before = hashlib.sha256(source.read_bytes()).digest()

            status, summary = self._export(source, output_dir)

            self.assertEqual(status, 0)
            self.assertTrue(summary["source_read_only"])
            self.assertEqual(summary["file_summary"]["format_version"], 2)
            self.assertEqual(summary["file_summary"]["timestamp_unit_ms"], 20)
            self.assertEqual(summary["file_summary"]["stop_snapshot_count"], 1)
            self.assertTrue((output_dir / "frames.csv").is_file())
            rows = self._read_csv(output_dir / "frames.csv")
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["episode_kind"], "STOP_SNAPSHOT")
            self.assertEqual(rows[1]["episode_kind"], "STOP_SNAPSHOT")
            self.assertEqual([row["elapsed_ms"] for row in rows], ["0", "60"])
            self.assertEqual(rows[0]["frame_sequence"], "65534")
            self.assertEqual(rows[1]["frame_sequence"], "65535")
            self.assertEqual(rows[0]["profile"], "C")
            self.assertEqual(rows[0]["source"], "ring_right")
            self.assertEqual(rows[0]["observation_valid"], "1")
            self.assertEqual(hashlib.sha256(source.read_bytes()).digest(), before)
            summary_file = json.loads((output_dir / "summary.json").read_text(
                encoding="utf-8"))
            self.assertEqual(summary_file["status"], "ok")

    def test_v1_elapsed_milliseconds_and_legacy_snapshot_flags(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source_dir = root / "camera"
            output_dir = root / "export"
            source_dir.mkdir()
            source = source_dir / "legacy.bin"
            source.write_bytes(_make_v1(_make_v2()))

            status, summary = self._export(source, output_dir)

            self.assertEqual(status, 0)
            self.assertEqual(summary["file_summary"]["format_version"], 1)
            self.assertEqual(summary["file_summary"]["timestamp_unit_ms"], 1)
            self.assertEqual(summary["file_summary"]["stop_snapshot_count"], 0)
            rows = self._read_csv(output_dir / "frames.csv")
            self.assertEqual([row["elapsed_ms"] for row in rows], ["123", "999"])
            self.assertEqual(rows[0]["episode_kind"], "EPISODE")

    def test_default_production_runtime_root_decodes_v1(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source_dir = root / "camera"
            output_dir = root / "export"
            source_dir.mkdir()
            source = source_dir / "legacy.bin"
            source.write_bytes(_make_v1(_make_v2()))

            status, summary = exporter.export(
                input_path=str(source), output_dir=str(output_dir))

            self.assertEqual(status, 0)
            self.assertEqual(summary["runtime_root"],
                             str(PROJECT_ROOT / "openmv"))
            self.assertEqual(summary["file_summary"]["format_version"], 1)
            self.assertEqual(len(self._read_csv(output_dir / "frames.csv")), 2)

    def test_directory_uses_latest_valid_and_reports_corrupt_newer_file(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            camera_dir = root / "camera"
            output_dir = root / "logs"
            camera_dir.mkdir()
            (camera_dir / "ring_diagnostics_000001.bin").write_bytes(
                _make_v1(_make_v2()))
            (camera_dir / "ring_diagnostics_000003.bin").write_bytes(
                _make_v2())
            bad = bytearray(_make_v2())
            bad[-1] ^= 0x20
            (camera_dir / "ring_diagnostics_000004.bin").write_bytes(bad)

            status, summary = self._export(camera_dir, output_dir,
                                           mode="directory")

            self.assertEqual(status, 0)
            self.assertEqual(summary["latest_number"], 4)
            self.assertEqual(summary["selected_number"], 3)
            self.assertTrue(summary["fallback_attempted"])
            self.assertTrue(summary["fallback_used"])
            self.assertEqual(summary["fallback_from_number"], 4)
            self.assertEqual(summary["fallback_reason"], "CRC_MISMATCH")
            self.assertEqual(summary["rejected_candidates"][0]["number"], 4)
            self.assertEqual(summary["rejected_candidates"][0]["reason"],
                             "CRC_MISMATCH")
            self.assertEqual(summary["file_summary"]["format_version"], 2)

    def test_corrupt_only_latest_is_reported_without_claiming_fallback(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            camera_dir = root / "camera"
            output_dir = root / "logs"
            camera_dir.mkdir()
            bad = bytearray(_make_v2())
            bad[-1] ^= 0x80
            (camera_dir / "ring_diagnostics_000008.bin").write_bytes(bad)

            status, summary = self._export(camera_dir, output_dir,
                                           mode="directory")

            self.assertNotEqual(status, 0)
            self.assertEqual(summary["latest_number"], 8)
            self.assertFalse(summary["fallback_attempted"])
            self.assertFalse(summary["fallback_used"])
            self.assertEqual(summary["latest_rejection_reason"],
                             "CRC_MISMATCH")
            self.assertFalse((output_dir / "frames.csv").exists())

    def test_ring_candidate_tracker_reason_and_each_reject_counter(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source_dir = root / "input"
            output_dir = root / "output"
            source_dir.mkdir()
            source = source_dir / "candidate.bin"
            source.write_bytes(_make_v2(candidate_episode=True))

            status, summary = self._export(source, output_dir)

            self.assertEqual(status, 0)
            rows = self._read_csv(output_dir / "frames.csv")
            self.assertEqual(summary["file_summary"]["stop_snapshot_count"], 0)
            candidate_rows = [row for row in rows if row["candidate"] == "1"]
            self.assertTrue(candidate_rows)
            row = candidate_rows[0]
            self.assertEqual(row["episode_kind"], "RING_CANDIDATE")
            self.assertEqual(row["tracker_state"], "ENTRY_CONFIRM")
            self.assertEqual(row["tracker_reason"], "OBSERVING")
            self.assertEqual(row["reject_reason"], "INPUT_INVALID")
            self.assertEqual(row["reject_count_none"], "0")
            self.assertEqual(row["reject_count_input_invalid"], "1")
            self.assertEqual(row["reject_count_state_timeout"], "12")
            self.assertEqual(row["ring_diag_exit_fit_rows"], "13")

    def test_bad_crc_and_valid_empty_file_never_create_frames_csv(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source_dir = root / "input"
            source_dir.mkdir()
            bad_source = source_dir / "bad.bin"
            bad = bytearray(_make_v2())
            bad[-1] ^= 0x01
            bad_source.write_bytes(bad)
            bad_output = root / "bad_export"

            status, summary = self._export(bad_source, bad_output)

            self.assertNotEqual(status, 0)
            self.assertEqual(summary["reason"], "CRC_MISMATCH")
            self.assertFalse((bad_output / "frames.csv").exists())
            self.assertEqual(json.loads(
                (bad_output / "summary.json").read_text(encoding="utf-8")
            )["status"], "error")

            invalid_header_source = source_dir / "invalid_header.bin"
            invalid_header = bytearray(_make_v2())
            struct.pack_into("<H", invalid_header, 6, 31)
            invalid_header_source.write_bytes(invalid_header)
            invalid_header_output = root / "invalid_header_export"
            status, summary = self._export(
                invalid_header_source, invalid_header_output)
            self.assertNotEqual(status, 0)
            self.assertEqual(summary["reason"], "INVALID_HEADER")
            self.assertFalse((invalid_header_output / "frames.csv").exists())

            truncated_source = source_dir / "truncated.bin"
            truncated_source.write_bytes(_make_v2()[:-1])
            truncated_output = root / "truncated_export"
            status, summary = self._export(truncated_source, truncated_output)
            self.assertNotEqual(status, 0)
            self.assertEqual(summary["reason"],
                             "INVALID_SIZE_OR_TRUNCATED")
            self.assertFalse((truncated_output / "frames.csv").exists())

            empty_source = source_dir / "empty_v1.bin"
            empty_source.write_bytes(_empty_v1())
            empty_output = root / "empty_export"
            status, summary = self._export(empty_source, empty_output)
            self.assertNotEqual(status, 0)
            self.assertEqual(summary["reason"], "NO_VALID_FRAMES")
            self.assertFalse((empty_output / "frames.csv").exists())

    def test_output_directory_must_be_separate_from_camera_directory(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            camera_dir = root / "camera"
            camera_dir.mkdir()
            source = camera_dir / "ring_diagnostics_000001.bin"
            source.write_bytes(_make_v2())
            with self.assertRaisesRegex(ValueError, "separate"):
                self._export(source, camera_dir / "exports")


if __name__ == "__main__":
    unittest.main(verbosity=2)
