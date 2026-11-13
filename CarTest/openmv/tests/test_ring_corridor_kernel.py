"""Typed Viper corridor result versus the unchanged object-safe route check."""
from array import array
from pathlib import Path
import random
import sys
import unittest

OPENMV = Path(__file__).resolve().parents[1]
if str(OPENMV) not in sys.path:
    sys.path.insert(0, str(OPENMV))
import track_elements as te


class RingCorridorKernelTests(unittest.TestCase):
    def tracker(self):
        t = te.TrackElementsTracker({})
        t.require_calibration = True
        t._calibration_ready = t._cal_check_ready = True
        t._current_road = t._current_valid = t._current_topology_clear = True
        t._near_left_rows = t._near_right_rows = 12
        t._near_left_run = t._near_right_run = 12
        t._near_left_ambiguous = t._near_right_ambiguous = 0
        t._calibration_width_valid = bytearray([1] * 60)
        t._calibration_width_cells = array('i', [30] * 60)
        t._current_status[:] = bytearray([1] * 120)
        t._current_edge2[:] = array('i', [25] * 60 + [85] * 60)
        return t

    def compare_reference(self, t, frozen):
        typed = t._calibration_width_cells
        actual = t._ordinary_corridor_frame(frozen)
        t._calibration_width_cells = list(typed)
        expected = t._ordinary_corridor_frame(frozen)
        t._calibration_width_cells = typed
        self.assertEqual(actual, expected)

    def test_random_geometry_and_frozen_fallback_match_reference(self):
        rng = random.Random(20261030)
        t = self.tracker()
        for case in range(2000):
            for y in range(60):
                left = rng.randrange(0, 70)
                right = rng.randrange(left + 1, 159)
                t._current_edge2[y] = left
                t._current_edge2[60+y] = right
                t._current_status[y] = 1 if case % 4 == 0 else rng.randrange(3)
                t._current_status[60+y] = 1 if case % 4 == 0 else rng.randrange(3)
                measured = (right-left)//2
                t._calibration_width_valid[y] = rng.randrange(3)
                t._calibration_width_cells[y] = rng.choice(
                    [measured, max(8, measured-1), 8, 9, 30, 70, 97, 100,
                     2147483647, -2147483648])
                t._ring_width_valid[y] = rng.randrange(2)
                t._ring_width[y] = rng.choice([measured, 8, 30, 70, 2147483647])
            for frozen in (False, True):
                self.compare_reference(t, frozen)

    def test_near_topology_and_calibration_gates_are_preserved(self):
        t = self.tracker()
        self.assertTrue(t._ordinary_corridor_frame(False))
        for field, value in (('_current_topology_clear', False),
                             ('_calibration_ready', False),
                             ('_cal_check_ready', False),
                             ('_near_left_rows', 0),
                             ('_near_right_run', 0),
                             ('_near_left_ambiguous', 1)):
            original = getattr(t, field)
            setattr(t, field, value)
            self.assertFalse(t._ordinary_corridor_frame(False), field)
            setattr(t, field, original)

    def test_byte_array_type_and_owner_changes_use_safe_fallback(self):
        t = self.tracker()
        self.assertTrue(t._ordinary_corridor_frame(False))
        self.assertTrue(t._corridor_width_fast)
        for widths in ([30] * 60, array('H', [30] * 60),
                       array('f', [30.] * 60)):
            t._calibration_width_cells = widths
            self.assertTrue(t._ordinary_corridor_frame(False))
            self.assertFalse(t._corridor_width_fast)
        t._calibration_width_cells = array('i', [30] * 60)
        self.assertTrue(t._ordinary_corridor_frame(False))
        self.assertTrue(t._corridor_width_fast)

    def test_row_count_span_and_signed_extremes_fail_closed(self):
        t = self.tracker()
        for width in (-2147483648, -1, 0, 2147483647):
            t._calibration_width_cells = array('i', [width] * 60)
            self.assertFalse(t._ordinary_corridor_frame(False))
        t._calibration_width_cells = array('i', [30] * 60)
        t._current_status[:] = bytearray(120)
        for y in range(8, 16):
            t._current_status[y] = t._current_status[60+y] = 1
        self.assertFalse(t._ordinary_corridor_frame(False))  # span only 7
        t._current_status[20] = t._current_status[80] = 1
        self.assertTrue(t._ordinary_corridor_frame(False))  # 9 rows, span12


if __name__ == '__main__':
    unittest.main()
