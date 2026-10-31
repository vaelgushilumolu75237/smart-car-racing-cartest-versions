from array import array
from pathlib import Path
import sys
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import track_elements as te

try:
    from .test_track_elements import TrackElementsHarness
except (ImportError, ValueError):
    from test_track_elements import TrackElementsHarness


class RingEntryTransitionTests(unittest.TestCase):
    def _harness(self, width=31, parked=True):
        h = TrackElementsHarness(require_calibration=True)
        valid = bytearray(60)
        widths = array("i", [0] * 60)
        for y in range(h.FLOOR, h.HEIGHT):
            valid[y] = 1
            widths[y] = width
        self.assertTrue(h.tracker.set_ring_calibration({
            "ready": True,
            "width_valid": valid,
            "width_cells": widths,
        }, require_calibration=True))
        if parked:
            rows = array("i")
            left = 25 if width == 31 else 5
            right = left + width - 1
            for y in range(h.FLOOR, h.HEIGHT):
                rows.extend((y, left, right, 7))
            for _ in range(3):
                h.tracker.observe_ring_calibration(rows, 52, 4)
            self.assertTrue(h.tracker.ring_calibration_check_ready)
        return h

    def _full_road(self, mask):
        mask["road_mask"] = bytearray([1] * (80 * 60))
        mask["known_region"] = bytearray([1] * (80 * 60))
        return mask

    def _near_pattern(self, h, side, status, edge2, positions):
        outer = te.SIDE_RIGHT if side == te.SIDE_LEFT else te.SIDE_LEFT
        base = outer * h.HEIGHT
        baseline = (h.tracker._baseline_right_by_y2 if outer == te.SIDE_RIGHT
                    else h.tracker._baseline_left_by_y2)
        for y in range(h.HEIGHT - h.tracker.near_rows, h.HEIGHT):
            status[base + y] = 0
            edge2[base + y] = 0
        for y in positions:
            status[base + y] = 1
            edge2[base + y] = int(baseline[y])

    def _prepare_inside(self, side, *, width=31, near_inner_rows=0,
                        near_outer_rows=8, outer_positions=None,
                        arc_start=20, arc_end=36):
        h = self._harness(width=width)
        h.prime()
        h.lose(side)
        result = None
        status = edge2 = mask = None
        for outward2 in (6, 8, 10):
            status, edge2, mask = h.entry_arc_inputs(
                side, outward2, near_inner_rows=near_inner_rows,
                near_outer_rows=near_outer_rows, arc_start=arc_start,
                arc_end=arc_end)
            if outer_positions is not None:
                self._near_pattern(h, side, status, edge2, outer_positions)
            mask = self._full_road(mask)
            result = h.frame(road_accepted=False, raw_status=status,
                             raw_edge2=edge2, mask_info=mask)
        return h, result, status, edge2, mask

    def _rows_by_y(self, result):
        rows = result["target_rows"]
        return {
            rows[index * te.TARGET_ROW_STRIDE]: (
                rows[index * te.TARGET_ROW_STRIDE + 1],
                rows[index * te.TARGET_ROW_STRIDE + 2],
                rows[index * te.TARGET_ROW_STRIDE + 3])
            for index in range(result["target_count"])
        }

    def test_mirrored_transition_uses_real_far_inner_and_near_outer_edges(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            with self.subTest(side=side):
                h, result, _, _, _ = self._prepare_inside(side)
                self.assertEqual(result["state_id"], te.STATE_INSIDE)
                self.assertTrue(result["ring_confirmed"])
                self.assertTrue(result["target_valid"])
                self.assertTrue(result["entry_transition"])
                self.assertTrue(h.tracker.entry_transition_valid_for(result))
                far_y = result["entry_transition_far_y"]
                near_y = result["entry_transition_near_y"]
                self.assertLessEqual(
                    far_y, h.tracker.reference_y -
                    (h.tracker.max_cache_gap + 3))
                self.assertLess(far_y, h.tracker.reference_y)
                self.assertLess(h.tracker.reference_y, near_y)
                self.assertEqual(result["entry_transition_near_rows"], 8)
                self.assertGreaterEqual(
                    result["entry_transition_inner_rows"],
                    h.tracker.ring_arc_min_rows)

                rows = self._rows_by_y(result)
                self.assertEqual(len(rows), result["target_count"])
                self.assertEqual(list(rows), sorted(rows))
                self.assertTrue(all(
                    flags & (te.TARGET_ENTRY_TRANSITION |
                             te.TARGET_WIDTH_FROM_CALIBRATION) ==
                    (te.TARGET_ENTRY_TRANSITION |
                     te.TARGET_WIDTH_FROM_CALIBRATION)
                    for _, _, flags in rows.values()))

                far_flags = rows[far_y][2]
                near_flags = rows[near_y][2]
                ref_flags = rows[h.tracker.reference_y][2]
                if side == te.SIDE_LEFT:
                    self.assertTrue(far_flags & te.TARGET_OBSERVED_LEFT)
                    self.assertTrue(near_flags & te.TARGET_OBSERVED_RIGHT)
                    self.assertLess(result["target_center2"], 80)
                else:
                    self.assertTrue(far_flags & te.TARGET_OBSERVED_RIGHT)
                    self.assertTrue(near_flags & te.TARGET_OBSERVED_LEFT)
                    self.assertGreater(result["target_center2"], 80)
                self.assertEqual(
                    ref_flags & (te.TARGET_INFERRED_LEFT |
                                 te.TARGET_INFERRED_RIGHT),
                    te.TARGET_INFERRED_LEFT | te.TARGET_INFERRED_RIGHT)
                self.assertEqual(
                    ref_flags & (te.TARGET_OBSERVED_LEFT |
                                 te.TARGET_OBSERVED_RIGHT), 0)
                for y in range(far_y + 1, near_y):
                    flags = rows[y][2]
                    self.assertEqual(
                        flags & (te.TARGET_INFERRED_LEFT |
                                 te.TARGET_INFERRED_RIGHT),
                        te.TARGET_INFERRED_LEFT | te.TARGET_INFERRED_RIGHT)
                    self.assertEqual(
                        flags & (te.TARGET_OBSERVED_LEFT |
                                 te.TARGET_OBSERVED_RIGHT), 0)

    def test_transition_certificate_is_bound_to_result_buffer_and_frame(self):
        h, result, _, _, _ = self._prepare_inside(te.SIDE_RIGHT)
        self.assertTrue(h.tracker.entry_transition_valid_for(result))
        self.assertFalse(h.tracker.entry_transition_valid_for(dict(result)))
        saved_rows = result["target_rows"]
        result["target_rows"] = array("i", saved_rows)
        self.assertFalse(h.tracker.entry_transition_valid_for(result))
        result["target_rows"] = saved_rows
        saved_frame = result["entry_transition_frame"]
        result["entry_transition_frame"] = saved_frame - 1
        self.assertFalse(h.tracker.entry_transition_valid_for(result))
        result["entry_transition_frame"] = saved_frame
        self.assertTrue(h.tracker.entry_transition_valid_for(result))

    def test_transition_requires_calibration_parked_check_and_frozen_width(self):
        for field in ("_calibration_ready", "_cal_check_ready",
                      "_ring_width_calibrated", "_ring_cache_frozen"):
            with self.subTest(field=field):
                h, result, _, _, mask = self._prepare_inside(te.SIDE_LEFT)
                setattr(h.tracker, field, False)
                self.assertFalse(h.tracker._build_target_rows(
                    mask, {"reason": "OK", "complete": True},
                    te.SIDE_LEFT))
                self.assertFalse(result["entry_transition"])
                self.assertFalse(h.tracker.entry_transition_valid_for(result))

        h = TrackElementsHarness(require_calibration=False)
        h.prime()
        h.lose(te.SIDE_LEFT)
        result = None
        for outward2 in (6, 8, 10):
            status, edge2, mask = h.entry_arc_inputs(
                te.SIDE_LEFT, outward2, near_inner_rows=0,
                near_outer_rows=8)
            result = h.frame(road_accepted=False, raw_status=status,
                             raw_edge2=edge2,
                             mask_info=self._full_road(mask))
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker.entry_transition_valid_for(result))

    def test_near_external_observation_keeps_original_support_and_run_gates(self):
        h, result, _, _, _ = self._prepare_inside(
            te.SIDE_RIGHT, near_outer_rows=4)
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertEqual(result["ring_arc_rejection"], "NONE")
        self.assertFalse(result["entry_transition"])
        self.assertEqual(h.tracker._near_left_rows, 4)

        h, result, _, _, _ = self._prepare_inside(
            te.SIDE_LEFT, outer_positions=(48, 49, 50, 52, 54))
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertEqual(result["ring_arc_rejection"], "NONE")
        self.assertEqual(h.tracker._near_right_rows, 5)
        self.assertEqual(h.tracker._near_right_run, 3)
        self.assertFalse(result["entry_transition"])

    def test_transition_requires_current_arc_and_far_reference_anchor(self):
        h, result, status, _, mask = self._prepare_inside(te.SIDE_RIGHT)
        inner_base = te.SIDE_RIGHT * h.HEIGHT
        for y in range(20, 36):
            h.tracker._current_status[inner_base + y] = 0
        self.assertFalse(h.tracker._build_target_rows(
            mask, {"reason": "OK", "complete": True}, te.SIDE_RIGHT))
        self.assertFalse(result["entry_transition"])

        h, result, _, _, _ = self._prepare_inside(
            te.SIDE_LEFT, arc_start=38, arc_end=47)
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker.entry_transition_valid_for(result))

    def test_input_topology_black_or_unknown_road_never_bypass_gates(self):
        h, result, _, _, mask = self._prepare_inside(te.SIDE_LEFT)
        self.assertFalse(h.tracker._build_target_rows(
            mask, {"reason": "BRANCH", "complete": False}, te.SIDE_LEFT))
        self.assertEqual(result["ring_target_rejection"], "TOPOLOGY")
        self.assertFalse(result["entry_transition"])

        h, result, _, _, mask = self._prepare_inside(te.SIDE_LEFT)
        invalid = dict(mask)
        invalid["valid"] = False
        self.assertFalse(h.tracker._build_target_rows(
            invalid, {"reason": "OK", "complete": True}, te.SIDE_LEFT))
        self.assertEqual(result["ring_target_rejection"], "INPUT")
        self.assertFalse(result["entry_transition"])

        for bad_field in ("road_mask", "known_region"):
            with self.subTest(bad_field=bad_field):
                h, result, _, _, mask = self._prepare_inside(te.SIDE_RIGHT)
                rows = self._rows_by_y(result)
                y = h.tracker.reference_y
                left, right, _ = rows[y]
                if bad_field == "road_mask":
                    mask[bad_field][y * h.WIDTH + left] = 0
                else:
                    mask[bad_field][y * h.WIDTH + right] = 0
                self.assertFalse(h.tracker._build_target_rows(
                    mask, {"reason": "OK", "complete": True},
                    te.SIDE_RIGHT))
                self.assertFalse(result["entry_transition"])
                self.assertFalse(h.tracker.entry_transition_valid_for(result))

    def test_width_gap_and_out_of_bounds_reject_transition(self):
        h, result, _, _, mask = self._prepare_inside(te.SIDE_RIGHT)
        ref_y = h.tracker.reference_y
        h.tracker._ring_nearest_y[ref_y] = ref_y + h.tracker.max_cache_gap + 1
        self.assertFalse(h.tracker._build_target_rows(
            mask, {"reason": "OK", "complete": True}, te.SIDE_RIGHT))
        self.assertFalse(result["entry_transition"])

        h, result, _, _, _ = self._prepare_inside(
            te.SIDE_LEFT, width=70)
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker.entry_transition_valid_for(result))

    def test_normal_near_inner_target_ends_transition_lease(self):
        h, result, _, _, _ = self._prepare_inside(
            te.SIDE_LEFT, near_inner_rows=8)
        self.assertEqual(result["state_id"], te.STATE_INSIDE)
        self.assertTrue(result["target_valid"])
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker._entry_transition_active)

        status, edge2, mask = h.entry_arc_inputs(
            te.SIDE_LEFT, 10, near_inner_rows=0, near_outer_rows=8)
        result = h.frame(road_accepted=False, raw_status=status,
                         raw_edge2=edge2, mask_info=self._full_road(mask))
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker._entry_transition_active)

    def test_only_real_confirmation_starts_nonrenewing_session_lease(self):
        h = self._harness()
        h.prime()
        h.lose(te.SIDE_RIGHT)
        for outward2 in (6, 8):
            status, edge2, mask = h.entry_arc_inputs(
                te.SIDE_RIGHT, outward2, near_inner_rows=0,
                near_outer_rows=8)
            result = h.frame(road_accepted=False, raw_status=status,
                             raw_edge2=edge2, mask_info=self._full_road(mask))
        self.assertEqual(result["state_id"], te.STATE_ENTER)
        self.assertFalse(h.tracker._entry_transition_active)
        self.assertFalse(h.tracker._build_target_rows(
            mask, {"reason": "OK", "complete": True}, te.SIDE_RIGHT))

        h, result, status, edge2, mask = self._prepare_inside(te.SIDE_RIGHT)
        lease_start = h.tracker._entry_transition_start_ms
        self.assertTrue(h.tracker._entry_transition_active)
        h.frame(road_accepted=False, raw_status=status, raw_edge2=edge2,
                mask_info=mask)
        self.assertEqual(h.tracker._entry_transition_start_ms, lease_start)

        expired_ms = (lease_start + h.tracker.max_entry_ms + 1) & \
            (te._TICKS_PERIOD - 1)
        result = h.frame(road_accepted=False, raw_status=status,
                         raw_edge2=edge2, mask_info=mask, now_ms=expired_ms)
        self.assertFalse(result["entry_transition"])
        self.assertFalse(h.tracker._entry_transition_active)

        h, result, status, edge2, mask = self._prepare_inside(te.SIDE_LEFT)
        h.frame(road_accepted=False, raw_status=status, raw_edge2=edge2,
                mask_info=mask, token="new-session")
        self.assertFalse(h.tracker._entry_transition_active)
        self.assertFalse(h.tracker.result["entry_transition"])

        h, result, _, _, _ = self._prepare_inside(te.SIDE_LEFT)
        h.tracker.reset("TEST_RESET")
        self.assertFalse(h.tracker._entry_transition_active)
        self.assertFalse(h.tracker.result["entry_transition"])


if __name__ == "__main__":
    unittest.main()
