"""Host tests for bounded ring and symmetric-cross observations."""

from array import array
from pathlib import Path
import sys
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import track_elements as te


class TrackElementsHarness:
    FLOOR = 8
    HEIGHT = 60
    WIDTH = 80
    BASE_LEFT = 25
    BASE_RIGHT = 55

    def __init__(self, settings=None, start_ms=0,
                 require_calibration=False):
        base = {
            "SURFACE_GRID_WIDTH": 80,
            "SURFACE_GRID_HEIGHT": 60,
            "SURFACE_ROI_FLOOR_Y": 8,
            "ROAD_REFERENCE_Y": 40,
            "TRACK_ELEMENTS_MIN_STABLE_FRAMES": 3,
            "TRACK_ELEMENTS_MIN_DROP_FRAMES": 2,
            "TRACK_ELEMENTS_EXIT_DROP_FRAMES": 2,
            "TRACK_ELEMENTS_EXIT_CONFIRM_FRAMES": 3,
            "TRACK_ELEMENTS_EXIT_REPAIR_FRAMES": 2,
            "TRACK_ELEMENTS_REACQUIRE_FRAMES": 3,
            "TRACK_ELEMENTS_MIN_SIDE_SUPPORT": 5,
            "TRACK_ELEMENTS_MIN_SIDE_CONTIGUOUS": 4,
            "TRACK_ELEMENTS_MAX_ACTIVE_FRAMES": 40,
            "TRACK_ELEMENTS_MAX_ENTRY_FRAMES": 12,
            "TRACK_ELEMENTS_CROSS_CONFIRM_FRAMES": 2,
        }
        if settings:
            base.update(settings)
        self.tracker = te.TrackElementsTracker(
            base, require_calibration=require_calibration)
        self.cache_valid = bytearray(60)
        self.cache_width = array("i", [0] * 60)
        self.cache_left2 = array("i", [0] * 60)
        self.cache_right2 = array("i", [0] * 60)
        self.profile_seq = 0
        self.odom = 100
        self.now = int(start_ms) & (te._TICKS_PERIOD - 1)
        for y in range(8, 60):
            self.cache_valid[y] = 1
            self.cache_width[y] = self.BASE_RIGHT - self.BASE_LEFT + 1
            self.cache_left2[y] = self.BASE_LEFT * 2 - 1
            self.cache_right2[y] = self.BASE_RIGHT * 2 + 1

    def mask(self, left_by_y=None, right_by_y=None, open_band=None,
             one_sided_open=False, extra_run=False, dark_hole=None):
        road = bytearray(80 * 60)
        edge = bytearray(80 * 60)
        known = bytearray([1] * (80 * 60))
        luma = bytearray([100] * (80 * 60))
        for y in range(8, 60):
            left = self.BASE_LEFT if left_by_y is None else left_by_y(y)
            right = self.BASE_RIGHT if right_by_y is None else right_by_y(y)
            if open_band is not None and open_band[0] <= y <= open_band[1]:
                left = open_band[2]
                right = (self.BASE_RIGHT if one_sided_open else open_band[3])
            row = y * 80
            for x in range(left, right + 1):
                road[row + x] = 1
            if left > 0:
                edge[row + left - 1] = 1
            if right < 79:
                edge[row + right + 1] = 1
            if extra_run and open_band is not None and open_band[0] <= y <= open_band[1]:
                for x in range(65, 72):
                    road[row + x] = 1
                edge[row + 64] = 1
                edge[row + 72] = 1
            if dark_hole is not None:
                hole_y, hole_left, hole_right = dark_hole
                if y == hole_y:
                    for x in range(hole_left, hole_right + 1):
                        road[row + x] = 0
        return {
            "valid": True,
            "width": 80,
            "height": 60,
            "floor": 8,
            "luma": luma,
            "road_mask": road,
            "edge_mask": edge,
            "known_region": known,
        }

    def entry_arc_inputs(self, side, outward2=6, *, outer_far=True,
                         near_outer_rows=4, near_inner_rows=5,
                         ambiguous_outer_y=None, arc_start=20,
                         arc_end=36, common_shift2=0):
        status = bytearray(120)
        edge2 = array("i", [0] * 120)
        left_cells = [self.BASE_LEFT] * self.HEIGHT
        right_cells = [self.BASE_RIGHT] * self.HEIGHT
        inner_base = side * self.HEIGHT
        outer_side = te.SIDE_RIGHT if side == te.SIDE_LEFT else te.SIDE_LEFT
        outer_base = outer_side * self.HEIGHT

        def shift_at(y):
            return (common_shift2(y) if callable(common_shift2) else
                    common_shift2)

        for y in range(arc_start, arc_end):
            common2 = shift_at(y)
            baseline_inner = int(
                self.tracker._baseline_left_by_y2[y]
                if side == te.SIDE_LEFT else
                self.tracker._baseline_right_by_y2[y])
            inner_edge2 = (baseline_inner + common2 - outward2
                           if side == te.SIDE_LEFT else
                           baseline_inner + common2 + outward2)
            status[inner_base + y] = 1
            edge2[inner_base + y] = inner_edge2
            baseline_outer = int(
                self.tracker._baseline_right_by_y2[y]
                if outer_side == te.SIDE_RIGHT else
                self.tracker._baseline_left_by_y2[y])
            outer_edge2 = baseline_outer + common2
            if side == te.SIDE_LEFT:
                left_cells[y] = (inner_edge2 + 1) // 2
                right_cells[y] = (outer_edge2 - 1) // 2
            else:
                right_cells[y] = (inner_edge2 - 1) // 2
                left_cells[y] = (outer_edge2 + 1) // 2
            if outer_far:
                if ambiguous_outer_y == y:
                    status[outer_base + y] = 2
                else:
                    status[outer_base + y] = 1
                    edge2[outer_base + y] = outer_edge2

        near_start = self.HEIGHT - near_inner_rows
        for y in range(near_start, self.HEIGHT):
            status[inner_base + y] = 1
            common2 = shift_at(y)
            edge2[inner_base + y] = int(
                self.tracker._baseline_left_by_y2[y]
                if side == te.SIDE_LEFT else
                self.tracker._baseline_right_by_y2[y]) + common2
        if near_outer_rows:
            near_outer_start = self.HEIGHT - near_outer_rows
            for y in range(near_outer_start, self.HEIGHT):
                status[outer_base + y] = 1
                edge2[outer_base + y] = int(
                    self.tracker._baseline_right_by_y2[y]
                    if outer_side == te.SIDE_RIGHT else
                    self.tracker._baseline_left_by_y2[y]) + shift_at(y)

        mask_info = self.mask(
            left_by_y=lambda y: left_cells[y],
            right_by_y=lambda y: right_cells[y])
        return status, edge2, mask_info

    def frame(self, left=25, right=55, *, road_accepted=True,
              raw_side=None, raw_status=None, raw_edge2=None,
              topology=None, mask_info=None,
              left_by_y=None, right_by_y=None, open_band=None,
              one_sided_open=False, extra_run=False, dark_hole=None,
              profile_id=2, active=True, token="session-1", profile_seq=None,
              odom=None, single_fresh=None, tracks=2, advance_ms=30,
              now_ms=None):
        if now_ms is None:
            self.now = (self.now + advance_ms) & (te._TICKS_PERIOD - 1)
        else:
            self.now = int(now_ms) & (te._TICKS_PERIOD - 1)
        self.profile_seq += 1
        if profile_seq is None:
            profile_seq = self.profile_seq & 0xffff
        if odom is None:
            self.odom += 10
            odom = self.odom
        rows = array("i")
        status = bytearray(120)
        edge2 = array("i", [0] * 120)
        contrast = array("i", [0] * 120)
        if road_accepted:
            for y in range(8, 60):
                row_left = left if left_by_y is None else left_by_y(y)
                row_right = right if right_by_y is None else right_by_y(y)
                rows.extend((y, row_left, row_right, 7))
        elif raw_side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            visible = left if raw_side == te.SIDE_LEFT else right
            for y in list(range(36, 45)) + list(range(48, 60)):
                index = raw_side * 60 + y
                status[index] = 1
                edge2[index] = (visible * 2 - 1 if raw_side == te.SIDE_LEFT
                                else visible * 2 + 1)
        elif raw_status is not None and raw_edge2 is not None:
            status = bytearray(raw_status)
            edge2 = array("i", raw_edge2)
        if single_fresh is None:
            single_fresh = raw_side is not None or raw_status is not None
        if topology is None:
            topology = {"reason": "OK", "complete": True}
        if mask_info is None:
            mask_info = self.mask(
                left_by_y, right_by_y, open_band, one_sided_open,
                extra_run, dark_hole)
        return self.tracker.update(
            self.now, profile_id, active, token, True, road_accepted,
            rows, len(rows) // 4, 4, tracks if road_accepted else 0,
            topology, single_fresh, status, edge2, contrast,
            self.cache_valid, self.cache_width, self.cache_left2,
            self.cache_right2, mask_info, profile_seq, odom)

    def prime(self):
        for _ in range(3):
            result = self.frame()
        assert self.tracker._baseline_ready
        return result

    def lose(self, side, count=2, left=25, right=55):
        missing = te.SIDE_RIGHT if side == te.SIDE_LEFT else te.SIDE_LEFT
        result = None
        for _ in range(count):
            result = self.frame(left, right, road_accepted=False,
                                raw_side=missing)
        return result

    def complete_left_ring(self, right_visible=55):
        self.prime()
        self.lose(te.SIDE_LEFT)
        result = None
        for outward2 in (6, 8, 10):
            status, edge2, mask = self.entry_arc_inputs(
                te.SIDE_LEFT, outward2)
            result = self.frame(
                road_accepted=False, raw_status=status,
                raw_edge2=edge2, mask_info=mask)
        return result

    def complete_right_ring(self, left_visible=25):
        self.prime()
        self.lose(te.SIDE_RIGHT)
        result = None
        for outward2 in (6, 8, 10):
            status, edge2, mask = self.entry_arc_inputs(
                te.SIDE_RIGHT, outward2)
            result = self.frame(
                road_accepted=False, raw_status=status,
                raw_edge2=edge2, mask_info=mask)
        return result


class TrackElementsTests(unittest.TestCase):
    def test_inactive_publish_refreshes_derived_outputs_after_fast_clear(self):
        h = TrackElementsHarness()
        result = h.complete_left_ring()
        self.assertTrue(result["ring_confirmed"])

        # These values are derived by _publish/_sync_ring_diagnostic. Poison
        # them to prove an inactive early return still refreshes the public
        # view after _clear_frame_output stops redundantly assigning them.
        result["candidate"] = True
        result["ring_confirmed"] = True
        result["control_eligible"] = True
        result["target_side_name"] = "RIGHT"
        result["ring_diagnostic_reason"] = "STALE"
        result["ring_arc_support_rows"] = 99
        result["ring_exit_residual_x2"] = 99
        result["state_elapsed_ms"] = 999

        result = h.frame(active=False)
        self.assertEqual(result["reason"], "SESSION_INACTIVE")
        self.assertFalse(result["candidate"])
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["control_eligible"])
        self.assertEqual(result["target_side_name"], "NONE")
        self.assertEqual(result["ring_diagnostic_reason"], "NONE")
        self.assertEqual(result["ring_arc_support_rows"], 0)
        self.assertEqual(result["ring_exit_residual_x2"], -1)
        self.assertEqual(result["state_elapsed_ms"], 270)

    def test_left_and_right_ring_sequences_are_mirrored(self):
        left = TrackElementsHarness()
        left_result = left.complete_left_ring()
        self.assertEqual(left_result["state"], "INSIDE")
        self.assertTrue(left_result["ring_confirmed"])
        self.assertTrue(left_result["control_eligible"])
        self.assertEqual(left_result["side"], te.SIDE_LEFT)

        right = TrackElementsHarness()
        right_result = right.complete_right_ring()
        self.assertEqual(right_result["state"], "INSIDE")
        self.assertTrue(right_result["ring_confirmed"])
        self.assertTrue(right_result["control_eligible"])
        self.assertEqual(right_result["side"], te.SIDE_RIGHT)

    def test_growing_inner_arc_confirms_both_sides_with_valid_routes(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            with self.subTest(side=side):
                h = TrackElementsHarness()
                h.prime()
                before_width = list(h.cache_width)
                h.lose(side)
                self.assertEqual(h.tracker.result["state"], "ENTER")

                status, edge2, mask = h.entry_arc_inputs(side, 6)
                result = h.frame(
                    road_accepted=False, raw_status=status,
                    raw_edge2=edge2, mask_info=mask)
                self.assertEqual(result["state"], "ENTER")
                self.assertEqual(result["ring_arc_growth_frames"], 1)

                status, edge2, mask = h.entry_arc_inputs(side, 8)
                result = h.frame(
                    road_accepted=False, raw_status=status,
                    raw_edge2=edge2, mask_info=mask)
                self.assertEqual(result["state"], "ENTER")
                self.assertEqual(result["ring_arc_growth_frames"], 2)

                status, edge2, mask = h.entry_arc_inputs(side, 10)
                result = h.frame(
                    road_accepted=False, raw_status=status,
                    raw_edge2=edge2, mask_info=mask)
                self.assertEqual(result["state"], "INSIDE")
                self.assertEqual(result["reason"], "GROWING_ARC_CONFIRMED")
                self.assertEqual(result["entry_confirmation_path"],
                                 "GROWING_ARC")
                self.assertEqual(result["ring_arc_growth_frames"], 3)
                self.assertEqual(result["ring_arc_growth_x2"], 4)
                self.assertTrue(result["ring_confirmed"])
                self.assertTrue(result["target_valid"])
                self.assertTrue(result["control_eligible"])
                self.assertFalse(result["growth_ready_but_target_rejected"])
                self.assertEqual(list(h.cache_width), before_width)
                result = h.frame(left=25, right=55)
                self.assertEqual(result["state"], "INSIDE")
                self.assertEqual(result["entry_confirmation_path"],
                                 "GROWING_ARC")

    def test_inner_arc_anchor_normalization_handles_both_shift_signs(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            for common_shift2 in (-6, 6):
                with self.subTest(side=side, shift=common_shift2):
                    h = TrackElementsHarness()
                    h.prime()
                    h.lose(side)
                    for outward2 in (6, 8, 10):
                        status, edge2, mask = h.entry_arc_inputs(
                            side, outward2, common_shift2=common_shift2)
                        result = h.frame(
                            road_accepted=False, raw_status=status,
                            raw_edge2=edge2, mask_info=mask)
                    self.assertEqual(result["state"], "INSIDE")
                    self.assertTrue(result["ring_confirmed"])

    def test_ambiguous_inner_or_outer_edge_never_counts_as_dropout(self):
        inner_ambiguous = TrackElementsHarness()
        inner_ambiguous.prime()
        for y in range(20, 32):
            inner_ambiguous.tracker._current_status[y] = 2
            inner_ambiguous.tracker._current_edge2[y] = 1000
        self.assertEqual(inner_ambiguous.tracker._find_entry_dropout_side(),
                         te.SIDE_NONE)

        outer_ambiguous = TrackElementsHarness()
        outer_ambiguous.prime()
        for y in range(20, 32):
            outer_ambiguous.tracker._current_status[y] = 0
            outer_ambiguous.tracker._current_status[
                outer_ambiguous.HEIGHT + y] = 2
            outer_ambiguous.tracker._current_edge2[
                outer_ambiguous.HEIGHT + y] = 1000
        self.assertEqual(outer_ambiguous.tracker._find_entry_dropout_side(),
                         te.SIDE_NONE)

    def test_confirmed_false_ring_releases_to_same_frame_ordinary_route(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        self.assertEqual(h.tracker.result["state"], "INSIDE")
        shared_result = h.tracker.result

        moved = h.frame(left=28, right=58)
        self.assertFalse(h.tracker.observe_route_outcome(
            h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 1)
        self.assertFalse(h.tracker.observe_route_outcome(
            h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 1)

        # A shallow ordinary bend moves both boundaries together by row.
        h.frame(left_by_y=lambda y: 27 if y < 40 else 26,
                right_by_y=lambda y: 57 if y < 40 else 56)
        self.assertFalse(h.tracker.observe_route_outcome(
            h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 2)

        result = h.frame(left=25, right=55)
        self.assertTrue(h.tracker.observe_route_outcome(
            h.now, False, True))
        self.assertIs(result, shared_result)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertEqual(result["reason"], "ORDINARY_ROUTE_REACQUIRED")
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["target_valid"])
        self.assertEqual(result["ring_width_source"], "NONE")

    def test_route_outcome_resets_on_interruption_route_rejection_and_ring_selection(self):
        interrupted = TrackElementsHarness()
        interrupted.complete_left_ring()
        interrupted.frame(left=28, right=58)
        self.assertFalse(interrupted.tracker.observe_route_outcome(
            interrupted.now, False, True))
        interrupted.frame(road_accepted=False, single_fresh=False)
        self.assertFalse(interrupted.tracker.observe_route_outcome(
            interrupted.now, False, True))
        self.assertEqual(interrupted.tracker._route_outcome_count, 0)
        for _ in range(2):
            interrupted.frame(left=28, right=58)
            self.assertFalse(interrupted.tracker.observe_route_outcome(
                interrupted.now, False, True))
        interrupted.frame(left=28, right=58)
        self.assertTrue(interrupted.tracker.observe_route_outcome(
            interrupted.now, False, True))

        guarded = TrackElementsHarness()
        guarded.complete_left_ring()
        guarded.frame(left=28, right=58)
        self.assertFalse(guarded.tracker.observe_route_outcome(
            guarded.now, False, True))
        guarded.frame(left=28, right=58)
        self.assertFalse(guarded.tracker.observe_route_outcome(
            guarded.now, False, False))
        self.assertEqual(guarded.tracker._route_outcome_count, 0)
        guarded.frame(left=28, right=58)
        self.assertFalse(guarded.tracker.observe_route_outcome(
            guarded.now, True, True))
        self.assertEqual(guarded.tracker._route_outcome_count, 0)
        for _ in range(2):
            guarded.frame(left=28, right=58)
            self.assertFalse(guarded.tracker.observe_route_outcome(
                guarded.now, False, True))
        guarded.frame(left=28, right=58)
        self.assertTrue(guarded.tracker.observe_route_outcome(
            guarded.now, False, True))

    def test_route_outcome_resets_when_session_identity_changes(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        h.frame(left=28, right=58)
        self.assertFalse(h.tracker.observe_route_outcome(h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 1)

        result = h.frame(left=28, right=58, token="new-session")
        self.assertFalse(h.tracker.observe_route_outcome(h.now, False, True))
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(h.tracker._route_outcome_count, 0)

    def test_route_outcome_rejects_bad_width_even_if_caller_flag_is_true(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        h.frame(left=10, right=70)
        self.assertFalse(h.tracker.observe_route_outcome(h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 0)
        self.assertEqual(h.tracker.result["state"], "INSIDE")

    def test_route_outcome_preserves_ring_geometry_and_requires_calibration(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        for _ in range(3):
            h.frame(left_by_y=lambda y: 22 if 20 <= y < 36 else 25,
                    right_by_y=lambda y: 55)
            self.assertFalse(h.tracker.observe_route_outcome(
                h.now, False, True))
        self.assertEqual(h.tracker._route_outcome_count, 0)
        self.assertEqual(h.tracker.result["state"], "INSIDE")

        no_cal = TrackElementsHarness()
        no_cal.complete_left_ring()
        no_cal.tracker.require_calibration = True
        no_cal.frame(left=28, right=58)
        self.assertFalse(no_cal.tracker.observe_route_outcome(
            no_cal.now, False, True))
        self.assertEqual(no_cal.tracker._route_outcome_count, 0)

    def test_cropped_far_outer_edge_is_allowed_with_stable_near_anchor(self):
        h = TrackElementsHarness()
        h.prime()
        h.lose(te.SIDE_LEFT)

        status, edge2, mask = h.entry_arc_inputs(
            te.SIDE_LEFT, 6, outer_far=False, near_outer_rows=4)
        result = h.frame(road_accepted=False, raw_status=status,
                         raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "ENTER")

        status, edge2, mask = h.entry_arc_inputs(
            te.SIDE_LEFT, 8, outer_far=False, near_outer_rows=4)
        result = h.frame(road_accepted=False, raw_status=status,
                         raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "ENTER")

        status, edge2, mask = h.entry_arc_inputs(
            te.SIDE_LEFT, 10, outer_far=False, near_outer_rows=4)
        result = h.frame(road_accepted=False, raw_status=status,
                         raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["reason"], "GROWING_ARC_CONFIRMED")
        self.assertEqual(result["entry_confirmation_path"], "GROWING_ARC")
        self.assertEqual(result["ring_near_anchor_rows"], 4)
        self.assertEqual(result["ring_near_anchor_run"], 4)
        self.assertTrue(result["target_valid"])
        self.assertTrue(result["control_eligible"])

    def test_growth_confirmation_does_not_bypass_target_corridor_guards(self):
        h = TrackElementsHarness()
        h.prime()
        h.lose(te.SIDE_LEFT)
        for outward2 in (6, 8, 10):
            status, edge2, mask = h.entry_arc_inputs(
                te.SIDE_LEFT, outward2, near_inner_rows=0)
            result = h.frame(road_accepted=False, raw_status=status,
                             raw_edge2=edge2, mask_info=mask)

        self.assertEqual(result["state"], "INSIDE")
        self.assertTrue(result["ring_confirmed"])
        self.assertTrue(result["growth_ready_but_target_rejected"])
        self.assertEqual(result["growth_target_rejection"], "NEAR_SUPPORT")
        self.assertEqual(result["ring_target_rejection"], "NEAR_SUPPORT")
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])

        no_reference = TrackElementsHarness()
        no_reference.prime()
        no_reference.lose(te.SIDE_LEFT)
        for outward2 in (6, 8, 10):
            status, edge2, mask = no_reference.entry_arc_inputs(
                te.SIDE_LEFT, outward2, arc_end=28)
            result = no_reference.frame(
                road_accepted=False, raw_status=status, raw_edge2=edge2,
                mask_info=mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertTrue(result["growth_ready_but_target_rejected"])
        self.assertEqual(result["growth_target_rejection"], "REFERENCE_GAP")
        self.assertEqual(result["ring_target_rejection"], "REFERENCE_GAP")
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])
        status, edge2, mask = no_reference.entry_arc_inputs(
            te.SIDE_LEFT, 8, arc_end=28)
        result = no_reference.frame(road_accepted=False, raw_status=status,
                                   raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["entry_confirmation_path"], "GROWING_ARC")
        self.assertTrue(result["growth_ready_but_target_rejected"])
        self.assertFalse(result["target_valid"])

    def test_entry_arc_requires_unambiguous_far_and_near_support(self):
        baseline = TrackElementsHarness().tracker
        self.assertEqual(baseline.min_near_support, 5)
        self.assertEqual(baseline.min_near_contiguous, 4)
        self.assertEqual(baseline.ring_entry_min_near_support, 4)
        self.assertEqual(baseline.ring_entry_min_near_contiguous, 3)

        missing_near = TrackElementsHarness()
        missing_near.prime()
        missing_near.lose(te.SIDE_LEFT)
        status, edge2, mask = missing_near.entry_arc_inputs(
            te.SIDE_LEFT, 8, near_outer_rows=0)
        result = missing_near.frame(
            road_accepted=False, raw_status=status, raw_edge2=edge2,
            mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["ring_arc_rejection"],
                         "OPPOSITE_NEAR_SUPPORT")
        self.assertFalse(result["ring_confirmed"])

        discontinuous = TrackElementsHarness()
        discontinuous.prime()
        discontinuous.lose(te.SIDE_LEFT)
        status, edge2, mask = discontinuous.entry_arc_inputs(
            te.SIDE_LEFT, 8, outer_far=False, near_outer_rows=8)
        for y in range(52, 60, 2):
            status[te.SIDE_RIGHT * 60 + y] = 0
        result = discontinuous.frame(
            road_accepted=False, raw_status=status, raw_edge2=edge2,
            mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["ring_arc_rejection"],
                         "OPPOSITE_NEAR_SUPPORT")
        self.assertFalse(result["ring_confirmed"])

        ambiguous = TrackElementsHarness()
        ambiguous.prime()
        ambiguous.lose(te.SIDE_LEFT)
        status, edge2, mask = ambiguous.entry_arc_inputs(
            te.SIDE_LEFT, 8, ambiguous_outer_y=24)
        result = ambiguous.frame(
            road_accepted=False, raw_status=status, raw_edge2=edge2,
            mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["ring_arc_rejection"], "OUTER_AMBIGUOUS")
        self.assertFalse(result["ring_confirmed"])

        only_border = TrackElementsHarness()
        only_border.prime()
        only_border.lose(te.SIDE_LEFT)
        status, edge2, mask = only_border.entry_arc_inputs(
            te.SIDE_LEFT, 0)
        result = only_border.frame(
            road_accepted=False, raw_status=status, raw_edge2=edge2,
            mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["ring_arc_rejection"], "INNER_ARC_SUPPORT")
        self.assertFalse(result["ring_confirmed"])

    def test_entry_arc_growth_resets_after_invalid_frame_and_rejects_jitter(self):
        interrupted = TrackElementsHarness()
        interrupted.prime()
        interrupted.lose(te.SIDE_LEFT)
        status, edge2, mask = interrupted.entry_arc_inputs(
            te.SIDE_LEFT, 6)
        result = interrupted.frame(road_accepted=False, raw_status=status,
                                  raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["ring_arc_growth_frames"], 1)
        result = interrupted.frame(road_accepted=False, single_fresh=False)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["ring_arc_growth_frames"], 0)
        for outward2 in (6, 8, 10):
            status, edge2, mask = interrupted.entry_arc_inputs(
                te.SIDE_LEFT, outward2)
            result = interrupted.frame(road_accepted=False,
                                      raw_status=status, raw_edge2=edge2,
                                      mask_info=mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["reason"], "GROWING_ARC_CONFIRMED")

        jitter = TrackElementsHarness()
        jitter.prime()
        jitter.lose(te.SIDE_LEFT)
        for outward2 in (6, 7, 6, 7, 6, 7):
            status, edge2, mask = jitter.entry_arc_inputs(
                te.SIDE_LEFT, outward2)
            result = jitter.frame(road_accepted=False, raw_status=status,
                                  raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertFalse(result["ring_confirmed"])
        self.assertEqual(result["ring_arc_growth_frames"], 2)
        self.assertEqual(result["ring_arc_growth_x2"], 1)

        stationary = TrackElementsHarness()
        stationary.prime()
        stationary.lose(te.SIDE_LEFT)
        for _ in range(4):
            status, edge2, mask = stationary.entry_arc_inputs(
                te.SIDE_LEFT, 6)
            result = stationary.frame(road_accepted=False,
                                      raw_status=status, raw_edge2=edge2,
                                      mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertFalse(result["ring_confirmed"])
        self.assertEqual(result["ring_arc_growth_x2"], 0)

    def test_one_reversal_frame_keeps_the_existing_peak_fallback_pending(self):
        h = TrackElementsHarness({
            "TRACK_ELEMENTS_RING_ARC_GROWING_SUPPORT_FRAMES": 4,
        })
        h.prime()
        h.lose(te.SIDE_LEFT)
        for outward2 in (6, 12, 6):
            status, edge2, mask = h.entry_arc_inputs(
                te.SIDE_LEFT, outward2, outer_far=False)
            result = h.frame(road_accepted=False, raw_status=status,
                             raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "ENTER")
        self.assertFalse(result["ring_confirmed"])
        self.assertEqual(h.tracker._reverse_count, 1)

        fallback = TrackElementsHarness({
            "TRACK_ELEMENTS_RING_ARC_GROWING_SUPPORT_FRAMES": 10,
        })
        fallback.prime()
        fallback.lose(te.SIDE_LEFT)
        for outward2 in (6, 12, 6, 6):
            status, edge2, mask = fallback.entry_arc_inputs(
                te.SIDE_LEFT, outward2, outer_far=False)
            result = fallback.frame(road_accepted=False, raw_status=status,
                                   raw_edge2=edge2, mask_info=mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["reason"], "RING_ARC_CONFIRMED")
        self.assertEqual(result["entry_confirmation_path"], "PEAK_REVERSAL")

    def test_ring_entry_still_requires_calibration_and_clear_topology(self):
        no_calibration = TrackElementsHarness(require_calibration=True)
        no_calibration.prime()
        result = no_calibration.lose(te.SIDE_LEFT)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(result["reason"], "RING_CALIBRATION_NOT_READY")
        self.assertFalse(result["candidate"])

        branch = TrackElementsHarness()
        branch.prime()
        branch.lose(te.SIDE_LEFT)
        status, edge2, mask = branch.entry_arc_inputs(te.SIDE_LEFT, 6)
        result = branch.frame(
            road_accepted=False, raw_status=status, raw_edge2=edge2,
            mask_info=mask,
            topology={"reason": "OK", "complete": True,
                      "branch_evidence": True})
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertFalse(result["ring_confirmed"])

    def test_entry_dropout_single_scan_preserves_mirrored_guards(self):
        straight = TrackElementsHarness()
        straight.prime()
        self.assertEqual(straight.tracker._find_entry_dropout_side(),
                         te.SIDE_NONE)
        self.assertEqual(straight.tracker._ring_diag_arc_rows, 0)
        self.assertGreaterEqual(straight.tracker._ring_diag_anchor_rows,
                                straight.tracker.min_near_support)

        left = TrackElementsHarness()
        left.prime()
        for y in range(left.FLOOR, 48):
            left.tracker._current_status[y] = 0
        self.assertEqual(left.tracker._find_entry_dropout_side(),
                         te.SIDE_LEFT)
        self.assertEqual(left.tracker._ring_diag_arc_rows, 40)
        self.assertGreaterEqual(left.tracker._ring_diag_anchor_rows,
                                left.tracker.min_near_support)

        right = TrackElementsHarness()
        right.prime()
        for y in range(right.FLOOR, 48):
            right.tracker._current_status[right.HEIGHT + y] = 0
        self.assertEqual(right.tracker._find_entry_dropout_side(),
                         te.SIDE_RIGHT)
        self.assertEqual(right.tracker._ring_diag_arc_rows, 40)

        # A long missing count split into runs shorter than four rows is not
        # enough to qualify; the near opposite-edge anchor remains present.
        split = TrackElementsHarness()
        split.prime()
        for y in range(split.FLOOR, 48):
            split.tracker._current_status[y] = 0
            split.tracker._baseline_row_valid[y] = 1 if y % 2 == 0 else 0
        self.assertEqual(split.tracker._find_entry_dropout_side(),
                         te.SIDE_NONE)
        self.assertEqual(split.tracker._ring_diag_arc_rows, 0)

        # Simultaneous mirrored drops remain ambiguous even with both anchors.
        both = TrackElementsHarness()
        both.prime()
        for y in range(both.FLOOR, 48):
            both.tracker._current_status[y] = 0
            both.tracker._current_status[both.HEIGHT + y] = 0
        self.assertEqual(both.tracker._find_entry_dropout_side(),
                         te.SIDE_NONE)
        self.assertEqual(both.tracker._ring_diag_arc_rows, 0)
        self.assertEqual(both.tracker._ring_diag_anchor_rows, 0)

    def test_entry_dropout_scan_detects_inner_only_outward_motion(self):
        left = TrackElementsHarness()
        left.prime()
        for y in range(20, 32):
            left.tracker._current_edge2[y] -= left.tracker.reappear_shift2
        self.assertEqual(left.tracker._find_entry_dropout_side(),
                         te.SIDE_LEFT)
        self.assertEqual(left.tracker._ring_diag_arc_rows, 12)

        right = TrackElementsHarness()
        right.prime()
        for y in range(20, 32):
            index = right.HEIGHT + y
            right.tracker._current_edge2[index] += \
                right.tracker.reappear_shift2
        self.assertEqual(right.tracker._find_entry_dropout_side(),
                         te.SIDE_RIGHT)
        self.assertEqual(right.tracker._ring_diag_arc_rows, 12)

    def test_entry_dropout_rejects_equal_common_translation(self):
        h = TrackElementsHarness()
        h.prime()
        for y in range(h.FLOOR, h.HEIGHT):
            h.tracker._current_edge2[y] += 8
            h.tracker._current_edge2[h.HEIGHT + y] += 8
        self.assertEqual(h.tracker._find_entry_dropout_side(), te.SIDE_NONE)
        self.assertEqual(h.tracker._ring_diag_arc_rows, 0)

    def test_common_translation_during_enter_is_not_inner_arc(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            with self.subTest(side=side):
                h = TrackElementsHarness()
                h.prime()
                h.lose(side)
                for _ in range(3):
                    status, edge2, mask = h.entry_arc_inputs(
                        side, 0, common_shift2=6)
                    result = h.frame(
                        road_accepted=False, raw_status=status,
                        raw_edge2=edge2, mask_info=mask)
                self.assertEqual(result["state"], "ENTER")
                self.assertFalse(result["ring_confirmed"])
                self.assertEqual(result["ring_arc_growth_frames"], 0)

    def test_locked_ring_side_uses_frozen_width_even_when_both_edges_show(self):
        h = TrackElementsHarness()
        result = h.complete_left_ring()
        self.assertEqual(result["state"], "INSIDE")
        frozen = list(h.tracker._ring_width)
        self.assertTrue(h.tracker._ring_cache_frozen)
        for y in range(8, 60):
            h.cache_width[y] = 35
            h.cache_left2[y] = 27 * 2 - 1
            h.cache_right2[y] = 59 * 2 + 1
        cache_snapshot = (list(h.cache_valid), list(h.cache_width),
                          list(h.cache_left2), list(h.cache_right2))
        result = h.frame(left=27, right=59)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(list(h.tracker._ring_width), frozen)
        self.assertTrue(result["target_valid"])
        self.assertGreater(result["target_count"], 0)
        route = result["target_rows"]
        self.assertEqual(route[1], 27)
        self.assertEqual(route[2], 57)  # 31-cell frozen width; live edge is 59.
        self.assertEqual(route[3], (te.TARGET_OBSERVED_LEFT |
                                    te.TARGET_INFERRED_RIGHT |
                                    te.TARGET_WIDTH_FROM_CACHE))
        self.assertEqual(list(h.cache_valid), cache_snapshot[0])
        self.assertEqual(list(h.cache_width), cache_snapshot[1])
        self.assertEqual(list(h.cache_left2), cache_snapshot[2])
        self.assertEqual(list(h.cache_right2), cache_snapshot[3])

    def test_one_sided_displacement_requires_arc_before_ring_confirmation(self):
        h = TrackElementsHarness()
        h.prime()
        result = h.frame(left=25, right=60, road_accepted=False,
                         raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "ENTRY_CONFIRM")
        self.assertTrue(result["candidate"])
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["control_eligible"])
        result = h.frame(left=25, right=60, road_accepted=False,
                         raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "ENTER")
        for _ in range(3):
            result = h.frame(left=25, right=60, road_accepted=False,
                             raw_side=te.SIDE_RIGHT)
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["target_valid"])

        mirror = TrackElementsHarness()
        mirror.prime()
        result = mirror.frame(left=20, right=55, road_accepted=False,
                              raw_side=te.SIDE_LEFT)
        self.assertEqual(result["state"], "ENTRY_CONFIRM")
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["control_eligible"])

    def test_ordinary_bilateral_corridor_vetoes_entry_and_s_curve_stays_rejected(self):
        bilateral = TrackElementsHarness()
        bilateral.prime()
        result = bilateral.frame(left=30, right=55)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(result["reason"], "ORDINARY_CORRIDOR_VETO")
        self.assertFalse(result["candidate"])
        self.assertTrue(bilateral.tracker._current_road)

        s_curve = TrackElementsHarness()
        s_curve.prime()
        s_curve.lose(te.SIDE_LEFT)
        s_curve.lose(te.SIDE_LEFT)
        for left, right in ((30, 60), (33, 63), (30, 60), (27, 57)):
            result = s_curve.frame(left=left, right=right)
        self.assertNotEqual(result["state"], "INSIDE")
        self.assertFalse(result["ring_confirmed"])

    def test_ordinary_corridor_clears_entry_confirm_and_enter(self):
        confirming = TrackElementsHarness()
        confirming.prime()
        result = confirming.frame(
            road_accepted=False, raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "ENTRY_CONFIRM")
        result = confirming.frame(left=25, right=55)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(result["reason"], "ORDINARY_CORRIDOR_VETO")
        self.assertFalse(result["ring_confirmed"])

        entering = TrackElementsHarness()
        entering.prime()
        entering.lose(te.SIDE_LEFT)
        self.assertEqual(entering.tracker.result["state"], "ENTER")
        result = entering.frame(left=25, right=55)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(result["reason"], "ORDINARY_CORRIDOR_VETO")
        self.assertFalse(result["ring_confirmed"])

    def test_branch_during_entry_aborts_candidate(self):
        h = TrackElementsHarness()
        h.prime()
        h.lose(te.SIDE_LEFT)
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         topology={"reason": "OK", "complete": True,
                                   "branch_evidence": True})
        self.assertNotEqual(result["state"], "INSIDE")
        self.assertFalse(result["ring_confirmed"])

    def test_production_ring_requires_calibration_and_three_frame_self_check(self):
        class Calibration:
            ready = True
            reason = "CALIBRATION_READY"
            width_valid = bytearray([0] * 8 + [1] * 52)
            width_cells = array("i", [0] * 8 + [31] * 52)

        tracker = te.TrackElementsTracker({}, Calibration(), True)
        rows = array("i")
        for y in range(59, 7, -1):
            rows.extend((y, 25, 55, 7))
        self.assertFalse(tracker.observe_ring_calibration(rows, 52, 4))
        self.assertEqual(tracker.result["ring_calibration_check_frames"], 1)
        self.assertFalse(tracker.observe_ring_calibration(rows, 52, 4))
        self.assertTrue(tracker.observe_ring_calibration(rows, 52, 4))
        self.assertTrue(tracker.result["ring_calibration_check_ready"])
        self.assertEqual(
            tracker.result["ring_calibration_median_error_permille"], 0)
        self.assertEqual(tracker.result["ring_diagnostic_stride"], 14)

        self.assertFalse(tracker.reset_ring_calibration_check("PARKED_GATE"))
        self.assertTrue(tracker.ring_calibration_check_ready)
        invalid = array("i", rows)
        invalid[3] = 0
        self.assertTrue(tracker.observe_ring_calibration(invalid, 52, 4))
        self.assertTrue(tracker.ring_calibration_check_ready)

    def test_startup_calibration_accepts_descending_unique_rows_and_rejects_duplicates(self):
        class Calibration:
            ready = True
            reason = "CALIBRATION_READY"
            width_valid = bytearray([0] * 8 + [1] * 52)
            width_cells = array("i", [0] * 8 + [31] * 52)

        tracker = te.TrackElementsTracker({}, Calibration(), True)
        descending = array("i")
        for y in range(59, 7, -1):
            descending.extend((y, 25, 55, 7))
        for _ in range(3):
            tracker.observe_ring_calibration(descending, 52, 4)
        self.assertTrue(tracker.ring_calibration_check_ready)

        duplicate_tracker = te.TrackElementsTracker({}, Calibration(), True)
        duplicate = array("i", descending)
        duplicate[4] = duplicate[0]
        self.assertFalse(duplicate_tracker.observe_ring_calibration(
            duplicate, 52, 4))
        self.assertEqual(duplicate_tracker.result[
            "ring_calibration_check_reason"], "INVALID_ROW_GEOMETRY")

    def test_partial_calibration_map_needs_reference_and_near_chain(self):
        class PartialCalibration:
            ready = True
            reason = "CALIBRATION_READY"
            width_valid = bytearray([0] * 20 + [1] * 40)
            width_cells = array("i", [0] * 20 + [31] * 40)

        tracker = te.TrackElementsTracker({}, PartialCalibration(), True)
        self.assertTrue(tracker.ring_calibration_ready)
        rows = array("i")
        for y in range(8, 60):
            rows.extend((y, 25, 55, 7))
        for _ in range(3):
            tracker.observe_ring_calibration(rows, 52, 4)
        self.assertTrue(tracker.ring_calibration_check_ready)

        class MissingReference:
            ready = True
            reason = "CALIBRATION_READY"
            width_valid = bytearray([0] * 60)
            width_cells = array("i", [0] * 60)

        for y in list(range(8, 36)) + list(range(48, 60)):
            MissingReference.width_valid[y] = 1
            MissingReference.width_cells[y] = 31
        missing = te.TrackElementsTracker({}, MissingReference(), True)
        self.assertFalse(missing.ring_calibration_ready)
        self.assertEqual(missing.ring_calibration_reason,
                         "REFERENCE_WIDTH_UNAVAILABLE")

    def test_calibrated_target_uses_map_and_rejects_out_of_fov_synthetic_edge(self):
        class Calibration:
            ready = True
            reason = "CALIBRATION_READY"
            width_valid = bytearray([0] * 8 + [1] * 52)
            width_cells = array("i", [0] * 8 + [32] * 52)

        h = TrackElementsHarness()
        h.prime()
        h.tracker.set_ring_calibration(Calibration(), True)
        rows = array("i")
        for y in range(8, 60):
            rows.extend((y, 25, 55, 7))
        for _ in range(3):
            h.tracker.observe_ring_calibration(rows, 52, 4)
        h.lose(te.SIDE_LEFT)
        h.lose(te.SIDE_LEFT)
        for outward2 in (6, 8, 10):
            status, edge2, mask = h.entry_arc_inputs(
                te.SIDE_LEFT, outward2)
            h.frame(road_accepted=False, raw_status=status,
                    raw_edge2=edge2, mask_info=mask)
        wide_mask = h.mask(
            left_by_y=lambda y: 27, right_by_y=lambda y: 58)
        result = h.frame(left=27, right=55, mask_info=wide_mask)
        self.assertEqual(result["state"], "INSIDE")
        self.assertTrue(result["control_eligible"])
        self.assertEqual(result["ring_width_source"],
                         "CALIBRATED_45CM_ROW_MAP")
        self.assertTrue(result["target_rows"][3] &
                        te.TARGET_WIDTH_FROM_CALIBRATION)
        self.assertEqual(result["target_rows"][2], 58)
        self.assertTrue(all(h.cache_width[y] == 31 for y in range(8, 60)))

        clipped_mask = h.mask(
            left_by_y=lambda y: 60, right_by_y=lambda y: 79)
        result = h.frame(road_accepted=False, raw_side=te.SIDE_LEFT,
                         left=60, right=79, mask_info=clipped_mask)
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])
        self.assertEqual(result["target_count"], 0)

    def test_one_frame_dropout_recovery_never_confirms_ring(self):
        h = TrackElementsHarness()
        h.prime()
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "ENTRY_CONFIRM")
        self.assertFalse(result["control_eligible"])
        self.assertFalse(result["target_valid"])
        result = h.frame(left=25, right=55)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertFalse(result["ring_confirmed"])

    def test_single_edge_loss_without_arc_times_out_to_reacquire(self):
        h = TrackElementsHarness({"TRACK_ELEMENTS_MAX_ENTRY_MS": 120})
        h.prime()
        h.lose(te.SIDE_LEFT)
        for _ in range(5):
            result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["control_eligible"])

    def test_odom_alone_does_not_advance_and_limit_only_aborts(self):
        h = TrackElementsHarness()
        self.assertEqual(h.tracker.max_active_millirev, 60000)
        result = h.frame(road_accepted=False, odom=50000, single_fresh=False)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertFalse(result["candidate"])

        h = TrackElementsHarness()
        h.prime()
        entry_odom = h.odom + 10
        h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                odom=entry_odom)
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         odom=entry_odom + 60001)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertFalse(result["ring_confirmed"])
        self.assertEqual(result["reason"], "ODOM_PROGRESS_LIMIT")
        self.assertTrue(result["millirev_guard_enabled"])

    def test_wrap_safe_entry_and_active_time_limits(self):
        period = te._TICKS_PERIOD
        self.assertEqual(te._ticks_diff(25, period - 25), 50)
        self.assertEqual(te._ticks_diff(period - 25, 25), -50)

        h = TrackElementsHarness(
            {"TRACK_ELEMENTS_MAX_ENTRY_FRAMES": 2},
            start_ms=period - 100)
        h.prime()
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT)
        self.assertEqual(result["state"], "ENTRY_CONFIRM")
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         advance_ms=2000)
        self.assertEqual(result["state"], "ENTER")
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         advance_ms=2000)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["state_elapsed_ms"], 2000)
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         advance_ms=2000)
        self.assertEqual(result["state"], "ENTER")
        self.assertEqual(result["state_elapsed_ms"], 4000)
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         advance_ms=1)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertEqual(result["reason"], "STATE_TIMEOUT")

        inside = TrackElementsHarness(
            {"TRACK_ELEMENTS_MAX_ACTIVE_FRAMES": 2},
            start_ms=period - 100)
        result = inside.complete_left_ring()
        self.assertEqual(result["state"], "INSIDE")
        for _ in range(75):
            result = inside.frame(left=27, right=55, advance_ms=30)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["state_elapsed_ms"], 2250)
        self.assertEqual(result["state_frames"], 75)

        result = inside.frame(left=27, right=55, advance_ms=13750)
        self.assertEqual(result["state"], "INSIDE")
        self.assertEqual(result["state_elapsed_ms"], 16000)
        result = inside.frame(left=27, right=55, advance_ms=1)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertEqual(result["reason"], "STATE_TIMEOUT")

    def test_profile_a_keeps_ring_off_and_does_not_block_cross(self):
        h = TrackElementsHarness()
        for _ in range(3):
            h.frame(profile_id=1)
        for _ in range(2):
            result = h.frame(profile_id=1, road_accepted=False,
                             raw_side=te.SIDE_RIGHT)
            self.assertEqual(result["state"], "OBSERVE")
            self.assertFalse(result["candidate"])
            self.assertFalse(result["ring_confirmed"])
            self.assertFalse(result["control_eligible"])
        for left in (30, 33, 30, 27):
            result = h.frame(profile_id=1, left=left, right=55)
            self.assertEqual(result["state"], "OBSERVE")
            self.assertFalse(result["ring_confirmed"])
        ring_events = [h.tracker._events[i + 1]
                       for i in range(0, len(h.tracker._events),
                                      te.EVENT_STRIDE)]
        self.assertNotIn(te.EVENT_ENTRY_CANDIDATE, ring_events)
        self.assertNotIn(te.EVENT_RING_CONFIRMED, ring_events)

        topology = {"reason": "OPEN_BAND", "complete": True,
                    "branch_evidence": True}
        common = dict(profile_id=1, road_accepted=False, single_fresh=False,
                      topology=topology, open_band=(24, 34, 20, 60))
        h.frame(**common)
        result = h.frame(**common)
        self.assertTrue(result["cross_route_usable"])
        self.assertTrue(result["cross_suppress_slowdown"])
        self.assertEqual(result["state"], "OBSERVE")

    def test_bilateral_visibility_inside_ring_does_not_infer_exit(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        for _ in range(8):
            result = h.frame(left=27, right=57)
        self.assertEqual(result["state"], "INSIDE")
        self.assertFalse(result["exit_reobserved"])
        self.assertFalse(result["exit_experimental"])

    def test_exit_requires_side_dropout_then_baseline_reappearance(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        for y in range(8, 60):
            h.cache_width[y] = 35
        live_width_before_exit = list(h.cache_width)
        result = h.lose(te.SIDE_RIGHT)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertTrue(result["exit_experimental"])
        self.assertEqual(result["side"], te.SIDE_LEFT)
        self.assertEqual(result["target_side"], te.SIDE_LEFT)
        self.assertTrue(result["target_valid"])
        self.assertTrue(result["control_eligible"])
        self.assertFalse(result["exit_reobserved"])

        self.assertEqual(list(h.cache_width), live_width_before_exit)
        for _ in range(3):
            result = h.frame(left=25, right=55)
        self.assertEqual(result["ring_exit_fit_rows"], 52)
        self.assertGreaterEqual(result["ring_exit_span_rows"], 12)
        self.assertLessEqual(result["ring_exit_residual_x2"], 4)
        self.assertEqual(result["state"], "EXIT_REPAIR")
        self.assertTrue(result["exit_experimental"])
        self.assertEqual(result["side"], te.SIDE_LEFT)
        self.assertEqual(result["target_side"], te.SIDE_LEFT)
        self.assertTrue(result["target_valid"])
        self.assertTrue(result["control_eligible"])
        for _ in range(h.tracker.exit_repair_frames):
            result = h.frame(left=25, right=55)
        self.assertEqual(result["state"], "REACQUIRE")
        self.assertTrue(result["exit_reobserved"])
        for _ in range(2):
            result = h.frame(left=25, right=55)
        self.assertEqual(result["state"], "OBSERVE")
        self.assertFalse(h.tracker._ring_cache_frozen)

        mirror = TrackElementsHarness()
        mirror.complete_right_ring()
        result = mirror.lose(te.SIDE_LEFT)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertEqual(result["side"], te.SIDE_RIGHT)
        self.assertEqual(result["target_side"], te.SIDE_RIGHT)
        self.assertTrue(result["target_valid"])
        self.assertFalse(result["exit_reobserved"])
        for _ in range(3):
            result = mirror.frame(left=25, right=55)
        self.assertEqual(result["state"], "EXIT_REPAIR")
        self.assertTrue(result["target_valid"])

    def test_exit_route_fails_closed_without_opposite_support_or_corridor(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        result = h.lose(te.SIDE_RIGHT)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertTrue(result["target_valid"])
        self.assertEqual(result["target_side"], te.SIDE_LEFT)

        # The exit is still provisional. The fresh inner edge can continue
        # ring following, but does not authorize opposite-side exit guidance.
        result = h.frame(road_accepted=False, raw_side=te.SIDE_LEFT)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertEqual(result["side"], te.SIDE_LEFT)
        self.assertEqual(result["target_side"], te.SIDE_LEFT)
        self.assertTrue(result["target_valid"])
        self.assertTrue(result["control_eligible"])
        self.assertFalse(result["exit_reobserved"])

        # Restore the opposite edge, but break its centerline corridor.
        result = h.frame(road_accepted=False, raw_side=te.SIDE_RIGHT,
                         topology={"reason": "ISLAND", "complete": True,
                                   "island_evidence": True},
                         dark_hole=(40, 45, 47))
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertEqual(result["target_side"], te.SIDE_NONE)
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])

    def test_exit_fit_residual_over_two_cells_does_not_confirm(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        result = h.lose(te.SIDE_RIGHT)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        for _ in range(3):
            result = h.frame(left=25, right=58)
        self.assertEqual(result["state"], "EXIT_CONFIRM")
        self.assertEqual(result["ring_exit_fit_rows"], 0)
        self.assertGreater(result["ring_exit_residual_x2"], 4)
        self.assertFalse(result["exit_reobserved"])

    def test_confirmed_ring_allows_island_only_with_clear_inner_corridor(self):
        h = TrackElementsHarness()
        h.complete_left_ring()
        ambiguous = {"reason": "OK", "complete": True,
                     "branch_evidence": True}
        result = h.frame(left=27, right=57, topology=ambiguous)
        self.assertEqual(result["state"], "INSIDE")
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])

        topology = {"reason": "ISLAND", "complete": True,
                    "island_evidence": True}
        result = h.frame(left=27, right=57, topology=topology)
        self.assertEqual(result["state"], "INSIDE")
        self.assertTrue(result["target_valid"])
        self.assertTrue(result["control_eligible"])

        result = h.frame(left=27, right=57, topology=topology,
                         dark_hole=(40, 35, 37))
        self.assertEqual(result["state"], "INSIDE")
        self.assertFalse(result["target_valid"])
        self.assertFalse(result["control_eligible"])

    def test_profile_token_reset_clears_state_but_seq_is_diagnostic_only(self):
        h = TrackElementsHarness()
        h.prime()
        for seq in (40, 41, 42):
            result = h.frame(profile_seq=seq)
            self.assertEqual(result["state"], "OBSERVE")
        h.lose(te.SIDE_LEFT)
        self.assertTrue(h.tracker._ring_cache_frozen)
        result = h.frame(token="session-2")
        self.assertEqual(result["state"], "OBSERVE")
        self.assertEqual(result["reason"], "OBSERVING")
        self.assertFalse(h.tracker._ring_cache_frozen)

    def test_explicit_reset_clears_published_route_and_event_state(self):
        h = TrackElementsHarness()
        result = h.complete_left_ring()
        self.assertTrue(result["control_eligible"])
        self.assertGreater(result["target_count"], 0)
        self.assertGreater(result["event_count"], 0)
        h.tracker.reset()
        result = h.tracker.result
        self.assertEqual(result["state"], "OBSERVE")
        self.assertFalse(result["candidate"])
        self.assertFalse(result["ring_confirmed"])
        self.assertFalse(result["control_eligible"])
        self.assertFalse(result["target_valid"])
        self.assertEqual(result["target_count"], 0)
        self.assertEqual(result["entry_confirmation_path"], "NONE")
        self.assertEqual(result["event_count"], 0)
        self.assertFalse(h.tracker._ring_cache_frozen)

    def test_symmetric_open_band_cross_uses_measured_and_interpolated_rows(self):
        h = TrackElementsHarness()
        h.prime()
        topology = {"reason": "OPEN_BAND", "complete": True,
                    "branch_evidence": True}
        common = dict(
            road_accepted=False, single_fresh=False, topology=topology,
            open_band=(24, 34, 20, 60), tracks=3)
        first = h.frame(**common)
        self.assertTrue(first["cross_candidate"])
        self.assertFalse(first["cross_route_usable"])
        result = h.frame(**common)
        self.assertEqual(result["cross_reason"], "CROSS_STRAIGHT_CONFIRMED")
        self.assertTrue(result["cross_route_usable"])
        self.assertTrue(result["cross_suppress_slowdown"])
        self.assertGreaterEqual(result["cross_near_rows"], 5)
        self.assertGreaterEqual(result["cross_far_rows"], 5)
        self.assertGreaterEqual(result["cross_span"], 6)
        self.assertEqual(result["cross_reference_y"], result["cross_near_y"])
        self.assertEqual(result["cross_lateral_error"], 13)
        records = result["cross_route_rows"]
        flags_by_y = {records[i]: records[i + 3]
                      for i in range(0, result["cross_route_count"] * 4, 4)}
        self.assertEqual(flags_by_y[result["cross_near_y"]],
                         te.CROSS_ROUTE_MEASURED)
        opening_y = 29
        self.assertEqual(flags_by_y[opening_y], te.CROSS_ROUTE_INTERPOLATED)

    def test_curve_asymmetric_neighbor_and_ambiguous_cross_are_rejected(self):
        h = TrackElementsHarness()
        h.prime()
        curve = lambda y: 24 + (y // 20)
        right_curve = lambda y: curve(y) + 30
        result = h.frame(left_by_y=curve, right_by_y=right_curve)
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])

        topology = {"reason": "OPEN_BAND", "complete": True,
                    "branch_evidence": True}
        result = h.frame(road_accepted=False, single_fresh=False,
                         topology=topology, open_band=(24, 34, 20, 52),
                         one_sided_open=True)
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])

        result = h.frame(road_accepted=False, single_fresh=False,
                         topology=topology, open_band=(24, 34, 20, 60),
                         extra_run=True)
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])

        dark_island_mask = h.mask(open_band=(24, 34, 20, 60),
                                  dark_hole=(29, 40, 42))
        result = h.frame(road_accepted=False, single_fresh=False,
                         topology=topology, mask_info=dark_island_mask)
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])

        missing_far_mask = h.mask(open_band=(24, 34, 20, 60))
        for y in range(8, 20):
            start = y * 80
            for x in range(80):
                missing_far_mask["road_mask"][start + x] = 0
                missing_far_mask["edge_mask"][start + x] = 0
        result = h.frame(road_accepted=False, single_fresh=False,
                         topology=topology, mask_info=missing_far_mask)
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])

        ambiguous = {"reason": "OPEN_BAND", "complete": True,
                     "branch_evidence": True,
                     "independent_branch_ambiguous": True}
        result = h.frame(road_accepted=False, single_fresh=False,
                         topology=ambiguous, open_band=(24, 34, 20, 60))
        self.assertFalse(result["cross_candidate"])
        self.assertFalse(result["cross_route_usable"])


if __name__ == "__main__":
    unittest.main()
