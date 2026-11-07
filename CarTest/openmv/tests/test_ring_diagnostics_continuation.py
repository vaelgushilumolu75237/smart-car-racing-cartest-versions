"""Regression checks for long ring episodes and stopped tail snapshots."""

import struct
import tempfile
import unittest
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ring_diagnostics


def _record(diagnostic, frame_index, sequence, active):
    tracker = {
        "candidate": active,
        "ring_confirmed": active,
        "state_id": 3 if active else 0,
    }
    return diagnostic.record_frame(
        frame_index * ring_diagnostics.TIMESTAMP_TICK_MS,
        sequence & 0xFFFF,
        2,
        True,
        True,
        {"source": "road", "reason": "OK"},
        tracker)


def _save_and_decode(diagnostic, directory):
    path = Path(directory) / "ring-diagnostics.bin"
    diagnostic.save(path, "RING_DIAG_HOST")
    data = path.read_bytes()
    summary = ring_diagnostics.RingDiagnostics.decode_file(data)
    offset = (ring_diagnostics.HEADER_SIZE +
              ring_diagnostics.BUILD_ID_BYTES + 3)
    episodes = []
    for _ in range(summary["episode_count"]):
        meta = struct.unpack_from(
            ring_diagnostics.EPISODE_META_FORMAT, data, offset)
        offset += ring_diagnostics.EPISODE_META_SIZE
        frames = []
        for frame_index in range(meta[0]):
            frame_offset = (offset +
                            frame_index * ring_diagnostics.FRAME_RECORD_SIZE)
            values = struct.unpack_from(
                "<%dh" % ring_diagnostics.BASE_FIELD_COUNT,
                data, frame_offset)
            frames.append({
                "tick": values[0],
                "sequence": values[1] & 0xFFFF,
                "state_id": values[7],
                "candidate": values[9],
                "confirmed": values[10],
            })
        offset += meta[0] * ring_diagnostics.FRAME_RECORD_SIZE
        episodes.append({
            "frame_count": meta[0],
            "pre_count": meta[1],
            "post_count": meta[2],
            "candidate_count": meta[3],
            "confirmed_count": meta[4],
            "start_frame": meta[5],
            "start_sequence": meta[6],
            "end_sequence": meta[7],
            "last_state_id": meta[8],
            "flags": meta[9],
            "dropped_count": meta[10],
            "frames": frames,
        })
    return path, data, summary, episodes


class RingDiagnosticsContinuationTests(unittest.TestCase):
    def test_long_candidate_and_latest_stop_snapshot_survive_sequence_wrap(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        first_sequence = 0xFF80
        total_frames = 64 + 300 + 8
        for frame_index in range(total_frames):
            active = 64 <= frame_index < 364
            self.assertTrue(_record(
                diagnostic, frame_index,
                first_sequence + frame_index, active))

        self.assertEqual(diagnostic.episode_count, 2)
        self.assertTrue(diagnostic.freeze("REVERSE_RESET"))
        self.assertFalse(diagnostic.freeze("REVERSE_RESET"))

        with tempfile.TemporaryDirectory() as directory:
            path, data, summary, episodes = _save_and_decode(
                diagnostic, directory)
            self.assertEqual(summary["format_version"], 3)
            self.assertEqual(summary["record_size"],
                             ring_diagnostics.FRAME_RECORD_SIZE)
            self.assertEqual(data[4] | (data[5] << 8), 3)
            self.assertEqual(data[8] | (data[9] << 8),
                             ring_diagnostics.FRAME_RECORD_SIZE)
            self.assertEqual(summary["episode_count"], 3)
            self.assertEqual(summary["episode_overflow"], 0)
            self.assertEqual(summary["stop_snapshot_count"], 1)
            self.assertLessEqual(path.stat().st_size,
                                 ring_diagnostics.MAX_FILE_BYTES)

            first, continuation, snapshot = episodes
            self.assertEqual(
                (first["frame_count"], first["pre_count"],
                 first["post_count"], first["candidate_count"],
                 first["dropped_count"]),
                (256, 64, 192, 192, 0))
            self.assertEqual(first["start_frame"], 64)
            self.assertEqual(first["start_sequence"],
                             (first_sequence + 64) & 0xFFFF)
            self.assertEqual(first["end_sequence"],
                             (first_sequence + 255) & 0xFFFF)
            self.assertEqual(
                (continuation["frame_count"], continuation["pre_count"],
                 continuation["post_count"],
                 continuation["candidate_count"],
                 continuation["dropped_count"]),
                (175, 64, 111, 108, 0))
            self.assertEqual(continuation["start_frame"], 256)
            self.assertEqual(continuation["start_sequence"],
                             (first_sequence + 256) & 0xFFFF)
            self.assertEqual(continuation["end_sequence"],
                             (first_sequence + 366) & 0xFFFF)
            continuation_ticks = {
                frame["tick"] for frame in continuation["frames"]
            }
            self.assertTrue(set(range(364, 367)).issubset(
                continuation_ticks))

            candidate_ticks = {
                frame["tick"]
                for episode in episodes[:2]
                for frame in episode["frames"]
                if frame["candidate"]
            }
            self.assertTrue(set(range(64, 364)).issubset(candidate_ticks))
            self.assertEqual(snapshot["flags"],
                             ring_diagnostics.STOP_SNAPSHOT_FLAG)
            self.assertEqual(
                (snapshot["frame_count"], snapshot["pre_count"],
                 snapshot["post_count"], snapshot["candidate_count"],
                 snapshot["confirmed_count"]),
                (64, 64, 0, 0, 0))
            self.assertEqual(snapshot["start_frame"], total_frames - 64)
            self.assertEqual(snapshot["start_sequence"],
                             (first_sequence + total_frames - 64) & 0xFFFF)
            self.assertEqual(snapshot["end_sequence"],
                             (first_sequence + total_frames - 1) & 0xFFFF)
            snapshot_ticks = {frame["tick"] for frame in snapshot["frames"]}
            self.assertTrue(set(range(364, total_frames)).issubset(
                snapshot_ticks))

            damaged = bytearray(data)
            damaged[-1] ^= 0x80
            with self.assertRaisesRegex(ValueError, "CRC"):
                ring_diagnostics.RingDiagnostics.decode_file(damaged)

    def test_exact_capacity_boundary_keeps_exit_post_tail(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        first_sequence = 0xFFF0
        total_frames = 64 + 192 + 5
        for frame_index in range(total_frames):
            active = 64 <= frame_index < 256
            self.assertTrue(_record(
                diagnostic, frame_index,
                first_sequence + frame_index, active))

        self.assertEqual(diagnostic.episode_count, 2)
        self.assertTrue(diagnostic.freeze("REVERSE_RESET"))
        with tempfile.TemporaryDirectory() as directory:
            _, _, summary, episodes = _save_and_decode(
                diagnostic, directory)
        self.assertEqual(summary["episode_count"], 3)
        self.assertEqual(episodes[0]["frame_count"], 256)
        self.assertEqual(episodes[0]["dropped_count"], 0)
        self.assertEqual(
            (episodes[1]["pre_count"], episodes[1]["post_count"],
             episodes[1]["frame_count"], episodes[1]["dropped_count"]),
            (64, 3, 67, 0))
        stored_ticks = {
            frame["tick"]
            for episode in episodes
            for frame in episode["frames"]
        }
        self.assertTrue(set(range(256, total_frames)).issubset(stored_ticks))

    def test_capacity_overflow_preserves_newest_candidates_and_snapshot(self):
        diagnostic = ring_diagnostics.RingDiagnostics()
        first_sequence = 0xFFF0
        frame_index = 0
        for _ in range(ring_diagnostics.PRE_FRAMES):
            self.assertTrue(_record(
                diagnostic, frame_index, first_sequence + frame_index, False))
            frame_index += 1
        for _episode in range(ring_diagnostics.EPISODE_CAPACITY + 1):
            self.assertTrue(_record(
                diagnostic, frame_index, first_sequence + frame_index, True))
            frame_index += 1
            for _idle in range(3):
                self.assertTrue(_record(
                    diagnostic, frame_index,
                    first_sequence + frame_index, False))
                frame_index += 1

        self.assertEqual(diagnostic.episode_count,
                         ring_diagnostics.EPISODE_CAPACITY)
        self.assertEqual(diagnostic.episode_overflow, 1)
        self.assertTrue(diagnostic.freeze("REVERSE_RESET"))
        with tempfile.TemporaryDirectory() as directory:
            path, _, summary, episodes = _save_and_decode(
                diagnostic, directory)
            file_size = path.stat().st_size

        self.assertEqual(summary["format_version"], 3)
        self.assertEqual(summary["episode_count"],
                         ring_diagnostics.EPISODE_CAPACITY)
        self.assertEqual(summary["episode_overflow"], 2)
        self.assertEqual(summary["stop_snapshot_count"], 1)
        self.assertLessEqual(file_size, ring_diagnostics.MAX_FILE_BYTES)
        retained_candidates = [
            episode for episode in episodes
            if episode["candidate_count"] > 0 and
            not episode["flags"] & ring_diagnostics.STOP_SNAPSHOT_FLAG
        ]
        self.assertEqual(len(retained_candidates), 7)
        self.assertTrue(all(episode["dropped_count"] == 0
                            for episode in episodes))


if __name__ == "__main__":
    unittest.main(verbosity=2)
