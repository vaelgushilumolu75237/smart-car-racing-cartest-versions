"""Host coverage for the private 45 cm straight width-map collector."""

import json
from array import array
from pathlib import Path
import sys
import tempfile
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

from ring_width_calibration import RingWidthCalibration
import config
import fast_path


def straight_rows(width=30, gap=None, gaps=(), curve=False):
    rows = []
    for y in range(8, 60):
        if ((gap is not None and gap[0] <= y <= gap[1]) or
                any(start <= y <= end for start, end in gaps)):
            continue
        center2 = 79
        if curve and 25 <= y <= 35:
            center2 += (y - 25) * (35 - y) * 2
        actual_width = width
        left = (center2 - (actual_width - 1)) // 2
        right = left + actual_width - 1
        rows.extend((y, left, right, 7))
    return rows


def collect(calibration, gap=None):
    accepted = 0
    for _ in range(66):
        rows = straight_rows(gap=gap)
        if calibration.accept_rows(rows, len(rows) // 4, 4):
            accepted += 1
    return accepted


def ranged_rows(first_y, last_y, gaps=()):
    rows = []
    for y in range(first_y, last_y + 1):
        if any(start <= y <= end for start, end in gaps):
            continue
        rows.extend((y, 25, 54, 7))
    return rows


def straight_path_masks():
    width, height = 80, 60
    road = bytearray(width * height)
    edge = bytearray(width * height)
    known = bytearray([1]) * (width * height)
    luma = bytearray([30]) * (width * height)
    row_counts = bytearray(height)
    road_counts = bytearray(height)
    for y in range(8, 60):
        if y in (9, 10):
            continue
        left = 13 + int((59 - y) * 16 / 51 + 0.5)
        right = 73 - int((59 - y) * 14 / 51 + 0.5)
        for x in range(left, right + 1):
            index = y * width + x
            road[index] = 1
            luma[index] = 220
        edge[y * width + left - 1] = 1
        edge[y * width + right + 1] = 1
        row_counts[y] = 2
        road_counts[y] = right - left + 1
    return {
        "valid": True,
        "width": width,
        "height": height,
        "floor": 8,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": row_counts,
        "road_row_counts": road_counts,
        "reason": "OK",
        "diagnostics": {},
    }


class RingWidthCalibrationTests(unittest.TestCase):
    def test_fast_path_descending_rows_are_accepted_by_collector(self):
        masks = straight_path_masks()
        detector = fast_path.FastPathDetector(config)
        result = detector.detect(masks)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], 50)
        self.assertEqual(result["selected_row_stride"], 4)
        rows = result["selected_row_ints"]
        self.assertEqual(rows[0], 59)
        self.assertEqual(rows[(result["selected_row_count"] - 1) * 4], 8)
        self.assertNotIn(9, rows[:result["selected_row_count"] * 4:4])
        self.assertNotIn(10, rows[:result["selected_row_count"] * 4:4])

        collector = RingWidthCalibration()
        for index in range(67):
            result = detector.detect(masks)
            accepted = collector.accept_rows(
                result["selected_row_ints"],
                result["selected_row_count"],
                result["selected_row_stride"])
            self.assertEqual(accepted, 2 <= index < 66)
        self.assertEqual(collector.accepted_frames, 64)
        payload = collector.finalize()
        self.assertTrue(payload["ready"], payload["reason"])
        self.assertEqual(payload["width_valid"][9:11], [1, 1])
        self.assertEqual(payload["width_interpolated"][9:11], [1, 1])

    def test_collector_accepts_descending_rows_but_rejects_duplicate_rows(self):
        descending = array("i")
        for y in range(59, 7, -1):
            descending.extend((y, 25, 54, 7))
        collector = RingWidthCalibration()
        for _ in range(3):
            collector.accept_rows(descending, 52, 4)
        self.assertEqual(collector.accepted_frames, 1)

        duplicate = array("i", descending)
        duplicate[4] = duplicate[0]
        rejected = RingWidthCalibration()
        self.assertFalse(rejected.accept_rows(duplicate, 52, 4))
        self.assertEqual(rejected.reason, "INVALID_ROW_GEOMETRY")

    def test_waits_for_three_stable_frames_then_takes_64_medians(self):
        calibration = RingWidthCalibration()
        rows30 = straight_rows(width=30)
        rows31 = straight_rows(width=31)
        self.assertFalse(calibration.accept_rows(rows30, len(rows30) // 4, 4))
        self.assertFalse(calibration.accept_rows(rows31, len(rows31) // 4, 4))
        self.assertTrue(calibration.accept_rows(rows30, len(rows30) // 4, 4))
        for index in range(63):
            rows = rows30 if index & 1 else rows31
            calibration.accept_rows(rows, len(rows) // 4, 4)
        result = calibration.finalize()
        self.assertTrue(result["ready"], result["reason"])
        self.assertEqual(calibration.accepted_frames, 64)
        self.assertEqual(result["width_cells"][40], 31)
        self.assertEqual(result["width_valid"][40], 1)

    def test_interpolates_only_internal_gaps_of_at_most_three_rows(self):
        short_gap = RingWidthCalibration()
        self.assertEqual(collect(short_gap, gap=(20, 22)), 64)
        ready = short_gap.finalize()
        self.assertTrue(ready["ready"], ready["reason"])
        self.assertEqual(ready["width_valid"][20:23], [1, 1, 1])
        self.assertEqual(ready["width_cells"][20:23], [30, 30, 30])

        long_gap = RingWidthCalibration()
        rows = ranged_rows(8, 59, gaps=((30, 35),))
        accepted = 0
        for _ in range(66):
            accepted += bool(long_gap.accept_rows(rows, len(rows) // 4, 4))
        self.assertEqual(accepted, 64)
        ready = long_gap.finalize()
        self.assertTrue(ready["ready"], ready["reason"])
        self.assertEqual(ready["width_valid"][30:36], [0] * 6)
        self.assertEqual(ready["width_cells"][30:36], [0] * 6)

    def test_unknown_rows_are_allowed_only_when_reference_and_near_chain_exist(self):
        upper_unknown = RingWidthCalibration()
        rows = ranged_rows(20, 59)
        accepted = 0
        for _ in range(66):
            accepted += bool(upper_unknown.accept_rows(rows, len(rows) // 4, 4))
        self.assertEqual(accepted, 64)
        result = upper_unknown.finalize()
        self.assertTrue(result["ready"], result["reason"])
        self.assertEqual(result["width_valid"][8:20], [0] * 12)
        self.assertEqual(result["width_cells"][8:20], [0] * 12)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "partial_ring_width.json"
            upper_unknown.save(str(path))
            loaded = RingWidthCalibration.load(str(path))
            self.assertTrue(loaded.ready, loaded.reason)
            self.assertEqual(list(loaded.width_valid)[8:20], [0] * 12)

        reference_missing = RingWidthCalibration()
        rows = ranged_rows(8, 59, gaps=((36, 47),))
        for _ in range(66):
            reference_missing.accept_rows(rows, len(rows) // 4, 4)
        result = reference_missing.finalize()
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "REFERENCE_WIDTH_UNAVAILABLE")

        near_disconnected = RingWidthCalibration()
        # Keep five real near rows, separated so a short interpolation cannot
        # fabricate the required four-row contiguous anchor.
        rows = ranged_rows(8, 49) + ranged_rows(55, 57)
        for _ in range(66):
            near_disconnected.accept_rows(rows, len(rows) // 4, 4)
        result = near_disconnected.finalize()
        self.assertFalse(result["ready"])
        self.assertEqual(result["reason"], "NEAR_WIDTH_SUPPORT_INSUFFICIENT")

    def test_rejects_turn_fit_invalid_width_and_nonbilateral_rows(self):
        calibration = RingWidthCalibration()
        curved = straight_rows(curve=True)
        self.assertFalse(calibration.accept_rows(curved, len(curved) // 4, 4))
        self.assertEqual(calibration.reason, "BAD_STRAIGHT_FIT")

        narrow = straight_rows(width=7)
        self.assertFalse(calibration.accept_rows(narrow, len(narrow) // 4, 4))
        self.assertEqual(calibration.reason, "WIDTH_OUT_OF_RANGE")

        sparse = straight_rows()
        sparse = sparse[:35 * 4]
        self.assertFalse(calibration.accept_rows(sparse, len(sparse) // 4, 4))
        self.assertEqual(calibration.reason, "INSUFFICIENT_BILATERAL_ROWS")

        bad_flags = straight_rows()
        bad_flags[3] = 0
        self.assertFalse(calibration.accept_rows(
            bad_flags, len(bad_flags) // 4, 4))
        self.assertEqual(calibration.reason, "INVALID_ROW_GEOMETRY")

    def test_rejected_frames_are_bounded(self):
        calibration = RingWidthCalibration()
        rows = [1, 2, 3, 7]
        for _ in range(600):
            calibration.accept_rows(rows, 1, 4)
        self.assertEqual(calibration.attempted_frames, 600)
        self.assertFalse(calibration.accept_rows(rows, 1, 4))
        self.assertEqual(calibration.reason, "COLLECTION_TIMEOUT")

    def test_save_load_crc_and_fixed_geometry_validation(self):
        calibration = RingWidthCalibration()
        collect(calibration)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "ring_width.json"
            calibration.save(str(path))
            loaded = RingWidthCalibration.load(str(path))
            self.assertTrue(loaded.ready, loaded.reason)
            self.assertEqual(list(loaded.width_cells),
                             list(calibration.width_cells))
            self.assertTrue(all(loaded.width_valid[y] == 1
                                for y in range(8, 60)))

            data = json.loads(path.read_text(encoding="utf-8"))
            data["width_cells"][40] += 1
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(RingWidthCalibration.load(str(path)).reason,
                             "CRC_MISMATCH")

            data = json.loads(path.read_text(encoding="utf-8"))
            data["sensor_id"] += 1
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(RingWidthCalibration.load(str(path)).reason,
                             "GEOMETRY_MISMATCH")

    def test_save_surfaces_io_errors_without_publishing_partial_file(self):
        calibration = RingWidthCalibration()
        collect(calibration)
        with tempfile.TemporaryDirectory() as temp_dir:
            target = Path(temp_dir) / "missing" / "ring_width.json"
            with self.assertRaises(OSError):
                calibration.save(str(target))
            self.assertFalse(target.exists())
            self.assertFalse(Path(str(target) + ".tmp").exists())

    def test_missing_file_is_not_ready_and_never_guesses_width(self):
        calibration = RingWidthCalibration.load("missing-ring-width.json")
        self.assertFalse(calibration.ready)
        self.assertEqual(calibration.reason, "FILE_NOT_FOUND")
        self.assertEqual(sum(calibration.width_valid), 0)
        self.assertEqual(sum(calibration.width_cells), 0)


if __name__ == "__main__":
    unittest.main()
