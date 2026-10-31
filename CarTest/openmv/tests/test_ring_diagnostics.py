"""Host checks for the bounded stopped-run ring-history file."""

import builtins
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ring_diagnostics


class RingDiagnosticsTests(unittest.TestCase):
    def _record(self, diagnostic, sequence, tracker=None, selection=None):
        return diagnostic.record_frame(
            sequence * 20, sequence & 0xFFFF, 2, True,
            sequence % 5 != 0,
            selection or {"source": "road", "reason": "OK"},
            tracker or {})

    def test_episode_has_unique_prehistory_and_fixed_ring_diagnostics(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        for sequence in range(64):
            self.assertTrue(self._record(diagnostic, sequence))
        features = [index - 4 for index in range(14)]
        tracker = {
            "candidate": True, "ring_confirmed": False, "state_id": 1,
            "ring_diagnostic_record": features,
            "ring_diagnostic_stride": 14,
            "reject_counts": [index for index in range(13)],
        }
        self.assertTrue(self._record(
            diagnostic, 64, tracker,
            {"source": "NONE", "reason": "NO_CONVEX_ARC"}))
        for sequence in range(65, 69):
            self.assertTrue(self._record(diagnostic, sequence))
        self.assertEqual(diagnostic.episode_count, 1)
        self.assertEqual(diagnostic._episode_meta[0], 68)
        self.assertEqual(diagnostic._episode_meta[1], 64)
        self.assertEqual(diagnostic._episode_meta[3], 1)

        # Header payload prefix is 63 build bytes, one length byte and overflow.
        metadata_offset = 63 + 1 + 2
        metadata = struct.unpack(
            ring_diagnostics.EPISODE_META_FORMAT,
            diagnostic._payload("host")[metadata_offset:
                                       metadata_offset +
                                       ring_diagnostics.EPISODE_META_SIZE])
        self.assertEqual(metadata[0:5], (68, 64, 4, 1, 0))
        self.assertEqual(metadata[5], 64)
        self.assertEqual(metadata[6], 64)
        self.assertEqual(metadata[7], 67)

        frame_offset = diagnostic._episode_offset(0, 64)
        record = diagnostic._frames[frame_offset:
                                    frame_offset +
                                    ring_diagnostics.FRAME_RECORD_SIZE]
        self.assertEqual(struct.unpack_from("<h", record, 34 * 2)[0], -4)
        self.assertEqual(struct.unpack_from("<h", record, 47 * 2)[0], 9)
        reject_offset = ring_diagnostics.BASE_FIELD_COUNT * 2
        self.assertEqual(
            struct.unpack_from("<13H", record, reject_offset),
            tuple(range(13)))

    def test_freeze_save_crc_decode_and_corruption_rejection(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        for sequence in range(70):
            tracker = ({"candidate": True, "state_id": 1}
                       if sequence == 65 else {})
            self._record(diagnostic, sequence, tracker)
        self.assertFalse(diagnostic.frozen)
        with self.assertRaises(RuntimeError):
            diagnostic.save("not-written.bin", "host")
        self.assertTrue(diagnostic.freeze("REVERSE_RESET"))
        self.assertFalse(diagnostic.freeze("REVERSE_RESET"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ring-diagnostics.bin"
            self.assertTrue(diagnostic.save(path, "RING45_HOST"))
            decoded = ring_diagnostics.RingDiagnostics.decode_file(
                path.read_bytes())
            self.assertEqual(decoded["build_id"], "RING45_HOST")
            self.assertEqual(decoded["episode_count"], 2)
            self.assertEqual(decoded["stop_snapshot_count"], 1)
            self.assertTrue(diagnostic.persisted)

            damaged = bytearray(path.read_bytes())
            damaged[-1] ^= 0x80
            with self.assertRaisesRegex(ValueError, "CRC"):
                ring_diagnostics.RingDiagnostics.decode_file(damaged)

    def test_episode_capacity_overflow_remains_bounded(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        sequence = 0
        for _episode in range(ring_diagnostics.EPISODE_CAPACITY + 1):
            self._record(diagnostic, sequence,
                         {"candidate": True, "state_id": 1})
            sequence += 1
            for _idle in range(3):
                self._record(diagnostic, sequence)
                sequence += 1
        self.assertEqual(diagnostic.episode_count,
                         ring_diagnostics.EPISODE_CAPACITY)
        self.assertEqual(diagnostic.episode_overflow, 1)
        self.assertLessEqual(
            ring_diagnostics.HEADER_SIZE + len(diagnostic._payload("host")),
            ring_diagnostics.MAX_FILE_BYTES)
        self.assertLessEqual(len(diagnostic.summary_lines("host")), 9)

    def test_io_failure_never_reports_persistence_success(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        self._record(diagnostic, 0, {"candidate": True, "state_id": 1})
        diagnostic.freeze()
        with patch.object(builtins, "open", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                diagnostic.save("ring-diagnostics.bin", "host")
        self.assertFalse(diagnostic.persisted)

    def test_save_creates_dedicated_subdirectory(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        self._record(diagnostic, 0, {"candidate": True, "state_id": 1})
        diagnostic.freeze("REVERSE_RESET")
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "ring_diag" / "ring-diagnostics.bin"
            self.assertTrue(diagnostic.save(target, "RING_DIAG_HOST"))
            self.assertTrue(target.is_file())
            decoded = ring_diagnostics.RingDiagnostics.decode_file(
                target.read_bytes())
            self.assertEqual(decoded["build_id"], "RING_DIAG_HOST")


if __name__ == "__main__":
    unittest.main(verbosity=2)
