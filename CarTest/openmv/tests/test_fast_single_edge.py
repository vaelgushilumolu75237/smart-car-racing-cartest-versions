"""Synthetic host tests for bounded single-edge inference and width cache."""

from array import array
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import config  # noqa: E402
import fast_single_edge  # noqa: E402
from fast_single_edge import FastSingleEdgeTracker  # noqa: E402
from fast_topology import FastTopologyGuard  # noqa: E402


WIDTH = 80
HEIGHT = 60
FLOOR = config.SURFACE_ROI_FLOOR_Y
LEFT = 27
RIGHT = 52


def _mask(mode="both", left=LEFT, right=RIGHT, dark=25, bright=220,
          ring=False):
    road = bytearray(WIDTH * HEIGHT)
    edge = bytearray([1]) * (WIDTH * HEIGHT)
    known = bytearray([1]) * (WIDTH * HEIGHT)
    luma = bytearray([dark]) * (WIDTH * HEIGHT)
    row_counts = bytearray([WIDTH]) * HEIGHT
    for y in range(FLOOR, HEIGHT):
        row = y * WIDTH
        if mode in ("both", "left"):
            start = left
            stop = right if mode == "both" else WIDTH - 1
            for x in range(start, stop + 1):
                road[row + x] = 1
                luma[row + x] = bright
                edge[row + x] = 0
        if mode in ("both", "right"):
            start = left if mode == "both" else 0
            stop = right
            for x in range(start, stop + 1):
                road[row + x] = 1
                luma[row + x] = bright
                edge[row + x] = 0
        if ring and mode == "left":
            for x in range(left + 8, left + 11):
                road[row + x] = 0
                luma[row + x] = dark
                edge[row + x] = 1
        row_counts[y] = sum(edge[row:row + WIDTH])
    return {
        "valid": True,
        "width": WIDTH,
        "height": HEIGHT,
        "floor": FLOOR,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": row_counts,
    }


def _blank_bright_mask():
    road = bytearray([1]) * (WIDTH * HEIGHT)
    edge = bytearray(WIDTH * HEIGHT)
    known = bytearray([1]) * (WIDTH * HEIGHT)
    luma = bytearray([220]) * (WIDTH * HEIGHT)
    row_counts = bytearray(HEIGHT)
    return {
        "valid": True,
        "width": WIDTH,
        "height": HEIGHT,
        "floor": FLOOR,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": row_counts,
    }


def _add_closed_dark_ring(mask, left, top, width, height, dark=25):
    right = left + width - 1
    bottom = top + height - 1
    for y in range(top, bottom + 1):
        row = y * WIDTH
        for x in range(left, right + 1):
            if x not in (left, right) and y not in (top, bottom):
                continue
            index = row + x
            if mask["edge_mask"][index] == 0:
                mask["row_counts"][y] += 1
            mask["edge_mask"][index] = 1
            mask["road_mask"][index] = 0
            mask["luma"][index] = dark
    return mask


def _topology(mask):
    return FastTopologyGuard(config).check(mask)


def _bilateral_candidate(left=LEFT, right=RIGHT):
    rows = array("i")
    for y in range(FLOOR, HEIGHT):
        rows.extend((y, left, right, 7))
    return {
        "valid": True,
        "selected_row_ints": rows,
        "selected_row_count": HEIGHT - FLOOR,
        "selected_row_stride": 4,
    }


def _warm_tracker(tracker, dark=25, bright=220):
    mask = _mask("both", dark=dark, bright=bright)
    accepted = tracker.observe_bilateral(
        _bilateral_candidate(), mask["luma"], mask["edge_mask"],
        mask["road_mask"], mask["known_region"], True, True)
    return mask, accepted


def _remove_edge_rows(mask, first_y, last_y):
    for y in range(first_y, last_y + 1):
        start = y * WIDTH
        mask["edge_mask"][start:start + WIDTH] = bytes(WIDTH)
    return mask


def _set_left_boundary(mask, y, edge_x, dark=25, bright=220):
    row = y * WIDTH
    for x in range(WIDTH):
        index = row + x
        if x < edge_x:
            mask["road_mask"][index] = 0
            mask["luma"][index] = dark
            mask["edge_mask"][index] = 1
        else:
            mask["road_mask"][index] = 1
            mask["luma"][index] = bright
            mask["edge_mask"][index] = 0


def _support_scanner(left_support=(), left_ambiguous=(),
                     right_support=(), right_ambiguous=()):
    def scan(tracker):
        tracker._row_status[:] = bytes(len(tracker._row_status))
        tracker._row_edge2[:] = array("i", [0] * len(tracker._row_edge2))
        tracker._row_center2[:] = array("i", [0] * len(tracker._row_center2))
        height = tracker.height
        for y in left_support:
            tracker._row_status[y] = 1
            edge2 = int(tracker._anchor_left2[y])
            tracker._row_edge2[y] = edge2
            tracker._row_center2[y] = edge2 + int(tracker._cache_width[y])
        for y in left_ambiguous:
            tracker._row_status[y] = 2
        for y in right_support:
            slot = height + y
            tracker._row_status[slot] = 1
            edge2 = int(tracker._anchor_right2[y])
            tracker._row_edge2[slot] = edge2
            tracker._row_center2[slot] = edge2 - int(tracker._cache_width[y])
        for y in right_ambiguous:
            tracker._row_status[height + y] = 2
    return scan


def _pair_support_scanner(left_support, right_support,
                          left_center2=80, right_center2=80,
                          left_edge2=50, right_edge2=100):
    """Supply selected side chains for bounded same-lane agreement tests."""
    def scan(tracker):
        tracker._row_status[:] = bytes(len(tracker._row_status))
        tracker._row_edge2[:] = array("i", [0] * len(tracker._row_edge2))
        tracker._row_center2[:] = array("i", [0] * len(tracker._row_center2))
        height = tracker.height
        for y in left_support:
            tracker._row_status[y] = 1
            tracker._row_edge2[y] = left_edge2
            tracker._row_center2[y] = left_center2
        for y in right_support:
            slot = height + y
            tracker._row_status[slot] = 1
            tracker._row_edge2[slot] = right_edge2
            tracker._row_center2[slot] = right_center2
    return scan


def _switch_tracker():
    tracker = FastSingleEdgeTracker(config)
    _warm_tracker(tracker)
    left = tracker.detect(_mask("left"), True)
    if left.get("valid"):
        tracker.commit_single(left)
    return tracker


class FastSingleEdgeTests(unittest.TestCase):
    def setUp(self):
        self.tracker = FastSingleEdgeTracker(config)

    def test_first_single_without_dual_width_cache_is_invalid(self):
        result = self.tracker.detect(_mask("left"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "NO_WIDTH_CACHE")

    def test_bilateral_rows_are_observed_and_inferred_rows_are_distinct(self):
        mask, accepted = _warm_tracker(self.tracker)
        self.assertGreaterEqual(accepted, self.tracker.min_rows)
        self.assertGreaterEqual(self.tracker.cache_rows,
                                self.tracker.min_rows)
        result = self.tracker.detect(_mask("left"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["boundary_source"], "LEFT_ONLY")
        self.assertTrue(result["inferred"])
        self.assertEqual(result["observed_both_rows"], 0)
        self.assertEqual(result["observed_left_rows"], result["inferred_rows"])
        self.assertEqual(result["selected_row_stride"], 5)
        self.assertEqual(result["selected_row_ints"][4], 1 | 8 | 16 | 32)
        self.assertEqual(mask["width"], WIDTH)

    def test_cropped_left_and_right_routes_recover_from_dual_width(self):
        _warm_tracker(self.tracker)
        left = self.tracker.detect(_mask("left"), True)
        self.assertTrue(left["valid"], left["reason"])
        self.tracker.clear("TEST_RESET")
        _warm_tracker(self.tracker)
        right = self.tracker.detect(_mask("right"), True)
        self.assertTrue(right["valid"], right["reason"])
        self.assertEqual(right["boundary_source"], "RIGHT_ONLY")
        self.assertTrue(right["inference_flags"] & 1)

    def test_black_and_broad_blue_contrast_use_same_grayscale_path(self):
        for dark in (10, 65, 90):
            with self.subTest(dark_luma=dark):
                tracker = FastSingleEdgeTracker(config)
                _warm_tracker(tracker, dark=dark, bright=215)
                result = tracker.detect(
                    _mask("left", dark=dark, bright=215), True)
                self.assertTrue(result["valid"], result["reason"])
                self.assertEqual(result["boundary_source"], "LEFT_ONLY")

    def test_ring_interior_breaks_the_observed_to_center_corridor(self):
        _warm_tracker(self.tracker)
        result = self.tracker.detect(
            _mask("left", ring=True), True,
            {"reason": "ISLAND", "ambiguous": True,
             "island_evidence": True})
        self.assertFalse(result["valid"])
        self.assertNotEqual(result["reason"], "OK_SINGLE_EDGE_LEFT")
        self.assertGreater(
            self.tracker._diagnostic_counts[
                fast_single_edge._DIAG_IDENTITY_GAP_REJECTS], 0)

    def test_island_outside_the_anchored_corridor_does_not_blanket_block(self):
        warm = _mask("both")
        self.tracker.observe_bilateral(
            _bilateral_candidate(), warm["luma"], warm["edge_mask"],
            warm["road_mask"], warm["known_region"], True, True)
        mask = _mask("left")
        _add_closed_dark_ring(mask, left=65, top=33, width=3, height=14)
        topology = _topology(mask)
        self.assertTrue(topology["island_evidence"], topology)
        result = self.tracker.detect(mask, True, topology)
        self.assertTrue(result["valid"], result["reason"])
        self.assertTrue(result["topology_island_observed"])
        self.assertTrue(result["topology_identity_resolved"])

    def test_actual_internal_ring_component_cannot_impersonate_outer_edge(self):
        tracker = FastSingleEdgeTracker(config)
        warm = _mask("both", left=20, right=45)
        candidate = _bilateral_candidate(left=20, right=45)
        self.assertGreaterEqual(tracker.observe_bilateral(
            candidate, warm["luma"], warm["edge_mask"], warm["road_mask"],
            warm["known_region"], True, True), tracker.min_rows)

        # The only dark-to-bright edge near the cached left boundary belongs
        # to a closed component inside an otherwise bright image. It has the
        # old edge coordinate and a clear inferred-center corridor, but it is
        # not connected to the visible outer/cropped boundary.
        current = _add_closed_dark_ring(
            _blank_bright_mask(), left=10, top=34, width=11, height=14)
        topology = _topology(current)
        self.assertTrue(topology["island_evidence"], topology)
        result = tracker.detect(current, True, topology)
        self.assertFalse(result["valid"], result)
        self.assertFalse(result["topology_identity_resolved"])

    def test_actual_external_edge_survives_unrelated_closed_ring(self):
        tracker = FastSingleEdgeTracker(config)
        warm = _mask("both", left=20, right=45)
        candidate = _bilateral_candidate(left=20, right=45)
        self.assertGreaterEqual(tracker.observe_bilateral(
            candidate, warm["luma"], warm["edge_mask"], warm["road_mask"],
            warm["known_region"], True, True), tracker.min_rows)

        current = _mask("left", left=20, right=45)
        _add_closed_dark_ring(current, left=61, top=33, width=9, height=14)
        topology = _topology(current)
        self.assertTrue(topology["island_evidence"], topology)
        result = tracker.detect(current, True, topology)
        self.assertTrue(result["valid"], result)
        self.assertTrue(result["topology_identity_resolved"])

    def test_fork_or_branch_topology_is_never_followed_as_a_single_edge(self):
        warm = _mask("both")
        self.tracker.observe_bilateral(
            _bilateral_candidate(), warm["luma"], warm["edge_mask"],
            warm["road_mask"], warm["known_region"], True, True)
        branch = _blank_bright_mask()
        for y in range(28, 32):
            for x in range(WIDTH):
                index = y * WIDTH + x
                branch["edge_mask"][index] = 1
                branch["road_mask"][index] = 0
                branch["luma"][index] = 25
                branch["row_counts"][y] += 1
        for y in range(35, HEIGHT):
            for x in (15, 64):
                index = y * WIDTH + x
                branch["edge_mask"][index] = 1
                branch["road_mask"][index] = 0
                branch["luma"][index] = 25
                branch["row_counts"][y] += 1
        topology = _topology(branch)
        self.assertTrue(topology["branch_evidence"], topology)
        result = self.tracker.detect(branch, True, topology)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "TOPOLOGY_BRANCH_UNRESOLVED")

    def test_both_boundaries_cropped_and_no_current_edge_are_invalid(self):
        _warm_tracker(self.tracker)
        mask = _mask("left")
        mask["edge_mask"][:] = bytes(len(mask["edge_mask"]))
        result = self.tracker.detect(mask, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "NO_ANCHORED_SINGLE_EDGE")

    def test_width_cache_clear_requires_a_new_bilateral_observation(self):
        _warm_tracker(self.tracker)
        self.assertGreater(self.tracker.cache_rows, 0)
        self.tracker.clear("CONTROL_RESET")
        self.assertEqual(self.tracker.cache_rows, 0)
        result = self.tracker.detect(_mask("left"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "NO_WIDTH_CACHE")

    def test_identity_jump_is_rejected_and_continuous_curve_tracks_past_anchor(self):
        _warm_tracker(self.tracker)
        jump = self.tracker.detect(_mask("left", left=LEFT + 21,
                                         right=RIGHT + 21), True)
        self.assertFalse(jump["valid"])

        # Restart from the accepted bilateral anchor, then move one cell per
        # frame. The observed anchor advances with bounded smoothing while the
        # inferred boundary remains tied to the same current side.
        self.tracker.clear("TEST_RESET")
        _warm_tracker(self.tracker)
        for offset in range(1, 15):
            result = self.tracker.detect(
                _mask("left", left=LEFT + offset, right=RIGHT + offset),
                True)
            self.assertTrue(result["valid"], (offset, result["reason"]))
            self.tracker.commit_single(result)
        self.assertEqual(self.tracker.max_anchor2 // 2, 16)

    def test_legal_inward_and_outward_anchor_motion_uses_full_radius(self):
        cases = (
            ("left", LEFT + 12, RIGHT + 12),
            ("left", LEFT + 16, RIGHT + 16),
            ("left", LEFT - 12, RIGHT - 12),
            ("right", LEFT - 12, RIGHT - 12),
            ("right", LEFT - 16, RIGHT - 16),
            ("right", LEFT + 12, RIGHT + 12),
        )
        for side, left, right in cases:
            with self.subTest(side=side, left=left, right=right):
                tracker = FastSingleEdgeTracker(config)
                _warm_tracker(tracker)
                result = tracker.detect(_mask(side, left=left, right=right),
                                        True)
                self.assertTrue(result["valid"], result["reason"])

    def test_bottommost_segment_wins_after_four_missing_rows(self):
        _warm_tracker(self.tracker)
        current = _remove_edge_rows(_mask("left"), 29, 32)
        # The upper chain has 21 rows; the newer lower chain has only 8.
        _remove_edge_rows(current, 8, 28)
        _remove_edge_rows(current, 41, HEIGHT - 1)
        result = self.tracker.detect(current, True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], 8)
        selected_y = [result["selected_row_ints"][i]
                      for i in range(0, result["selected_row_count"] * 5, 5)]
        self.assertEqual(selected_y, list(range(33, 41)))
        self.assertEqual(result["reference_y"], 40.0)

    def test_three_missing_rows_remain_in_one_chain(self):
        _warm_tracker(self.tracker)
        current = _remove_edge_rows(_mask("left"), 28, 30)
        result = self.tracker.detect(current, True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], HEIGHT - FLOOR - 3)
        self.assertEqual(result["selected_row_ints"][0], FLOOR)
        self.assertEqual(result["selected_row_ints"][(result["selected_row_count"] - 1) * 5],
                         HEIGHT - 1)

    def test_jump_restarts_a_new_bottom_chain_and_publishes_only_it(self):
        _warm_tracker(self.tracker)
        current = _mask("left")
        for y in range(39, HEIGHT):
            _set_left_boundary(current, y, LEFT + 13)
        result = self.tracker.detect(current, True)
        self.assertTrue(result["valid"], result["reason"])
        selected_y = [result["selected_row_ints"][i]
                      for i in range(0, result["selected_row_count"] * 5, 5)]
        self.assertEqual(selected_y, list(range(39, HEIGHT)))
        self.assertEqual(result["reference_y"], 40.0)

    def test_ambiguous_row_breaks_local_chain_and_other_side_can_still_win(self):
        _warm_tracker(self.tracker)
        support = tuple(list(range(FLOOR, 39)) + list(range(40, 47)))
        scanner = _support_scanner(
            left_support=support, left_ambiguous=(39,))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = self.tracker.detect(_mask("left"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], 7)
        self.assertEqual(result["selected_row_ints"][0], 40)
        self.assertEqual(result["reference_y"], 40.0)

        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        scanner = _support_scanner(
            left_ambiguous=range(FLOOR, HEIGHT),
            right_support=range(20, HEIGHT))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("right"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["boundary_source"], "RIGHT_ONLY")

    def test_unresolved_equivalent_edge_branch_remains_rejected(self):
        _warm_tracker(self.tracker)
        scanner = _support_scanner(
            left_ambiguous=range(FLOOR, HEIGHT))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = self.tracker.detect(_mask("left"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "AMBIGUOUS_SINGLE_EDGE_CHAIN")

    def test_short_chain_can_reference_nearest_measured_row_without_y40(self):
        _warm_tracker(self.tracker)
        current = _remove_edge_rows(_mask("left"), FLOOR, 59)
        support = (48, 49, 51, 53, 54)
        for y in support:
            start = y * WIDTH
            base = _mask("left")
            current["edge_mask"][start:start + WIDTH] = \
                base["edge_mask"][start:start + WIDTH]
        result = self.tracker.detect(current, True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], 5)
        self.assertEqual(result["reference_y"], 48.0)
        self.assertTrue(result["weak_support"])

    def test_short_support_rows_and_span_have_distinct_reasons(self):
        _warm_tracker(self.tracker)
        scanner = _support_scanner(left_support=(48, 50, 52, 54))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            too_few = self.tracker.detect(_mask("left"), True)
        self.assertFalse(too_few["valid"])
        self.assertEqual(too_few["reason"], "SINGLE_EDGE_SUPPORT_ROWS")

        scanner = _support_scanner(left_support=(48, 49, 50, 51, 52))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            too_short = self.tracker.detect(_mask("left"), True)
        self.assertFalse(too_short["valid"])
        self.assertEqual(too_short["reason"], "SINGLE_EDGE_SUPPORT_SPAN")

    def test_same_lane_pair_accepts_exact_common_row_span_and_delta_limits(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        common = (35, 36, 37, 40, 41)
        left_support = (30, 31, 32) + common
        right_support = common + (43, 44, 45)
        scanner = _pair_support_scanner(
            left_support, right_support, left_center2=80,
            right_center2=86)
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("both"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["reason"], "OK_SINGLE_EDGE_RIGHT")
        diagnostic = tracker.pair_diagnostic_record
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_REASON],
                         fast_single_edge._PAIR_REASON_SAME_LANE)
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_COMMON_ROWS],
                         5)
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_SPAN], 6)
        self.assertEqual(
            diagnostic[fast_single_edge._PAIR_DIAG_MAX_DELTA2], 6)

    def test_same_lane_pair_rejects_different_lane_center_delta(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        common = (35, 36, 37, 40, 41)
        scanner = _pair_support_scanner(
            (30, 31, 32) + common, common + (43, 44, 45),
            left_center2=80, right_center2=87)
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("both"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "AMBIGUOUS_BOTH_SIDES")
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_REASON],
            fast_single_edge._PAIR_REASON_CENTER_DELTA)
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_MAX_DELTA2], 7)

    def test_pair_needs_five_rows_and_ignores_matching_rows_outside_chains(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        selected_common = (35, 36, 39, 40)
        unrelated_common = tuple(range(10, 16))
        left_support = (30, 31, 32, 33) + selected_common + unrelated_common
        right_support = selected_common + (41, 42, 43, 44) + unrelated_common
        scanner = _pair_support_scanner(
            left_support, right_support, left_center2=80,
            right_center2=87)
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("both"), True)
        self.assertFalse(result["valid"])
        diagnostic = tracker.pair_diagnostic_record
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_COMMON_ROWS],
                         4)
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_SPAN], 5)
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_MAX_DELTA2], 7)
        self.assertEqual(diagnostic[fast_single_edge._PAIR_DIAG_REASON],
                         fast_single_edge._PAIR_REASON_TOO_FEW_ROWS)

    def test_pair_does_not_use_agreement_outside_selected_chain_intersection(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        unrelated_common = tuple(range(10, 16))
        scanner = _pair_support_scanner(
            tuple(range(30, 38)) + unrelated_common,
            tuple(range(42, 50)) + unrelated_common)
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("both"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_COMMON_ROWS], 0)
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_REASON],
            fast_single_edge._PAIR_REASON_NO_COMMON_ROWS)

    def test_same_lane_pair_rejects_wrong_boundary_order(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        common = (35, 36, 37, 40, 41)
        scanner = _pair_support_scanner(
            (30, 31, 32) + common, common + (43, 44, 45),
            left_center2=80, right_center2=80,
            left_edge2=100, right_edge2=50)
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            result = tracker.detect(_mask("both"), True)
        self.assertFalse(result["valid"])
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_REASON],
            fast_single_edge._PAIR_REASON_BOUNDARY_ORDER)

    def test_same_lane_pair_prefers_previous_accepted_side_identity(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        left = tracker.detect(_mask("left"), True)
        self.assertTrue(left["valid"], left["reason"])
        tracker.commit_single(left)
        common = (35, 36, 37, 40, 41)
        scanner = _pair_support_scanner(
            (30, 31, 32) + common, common + (43, 44, 45))
        with patch.object(fast_single_edge, "_scan_single_edges_viper",
                          side_effect=scanner):
            paired = tracker.detect(_mask("both"), True)
        self.assertTrue(paired["valid"], paired["reason"])
        self.assertEqual(paired["boundary_source"], "LEFT_ONLY")
        self.assertEqual(tracker.pair_diagnostic_record[
            fast_single_edge._PAIR_DIAG_SELECTION],
            fast_single_edge._PAIR_SELECT_PREVIOUS_LEFT)

    def test_side_switch_requires_two_adjacent_frames_and_overlap(self):
        tracker = _switch_tracker()
        first = tracker.detect(_mask("right"), True)
        self.assertFalse(first["valid"])
        self.assertEqual(first["reason"], "SINGLE_EDGE_SIDE_SWITCH_PENDING")
        second = tracker.detect(_mask("right"), True)
        self.assertTrue(second["valid"], second["reason"])
        self.assertTrue(second["side_switch"])
        self.assertGreaterEqual(second["switch_overlap_rows"], 1)
        self.assertLessEqual(second["switch_max_center_delta_cells"], 12.0)
        tracker.commit_single(second)
        self.assertEqual(tracker._last_single_side, 1)

    def test_side_switch_without_overlap_is_rejected(self):
        tracker = FastSingleEdgeTracker(config)
        _warm_tracker(tracker)
        left = _remove_edge_rows(_mask("left"), 31, HEIGHT - 1)
        accepted = tracker.detect(left, True)
        self.assertTrue(accepted["valid"], accepted["reason"])
        tracker.commit_single(accepted)

        right = _remove_edge_rows(_mask("right"), FLOOR, 39)
        pending = tracker.detect(right, True)
        self.assertEqual(pending["reason"], "SINGLE_EDGE_SIDE_SWITCH_PENDING")
        rejected = tracker.detect(right, True)
        self.assertFalse(rejected["valid"])
        self.assertEqual(rejected["reason"],
                         "SINGLE_EDGE_SIDE_SWITCH_NO_OVERLAP")

    def test_side_switch_center_mismatch_over_twelve_cells_is_rejected(self):
        tracker = _switch_tracker()
        right = _mask("right", right=37)
        first = tracker.detect(right, True)
        self.assertEqual(first["reason"], "SINGLE_EDGE_SIDE_SWITCH_PENDING")
        rejected = tracker.detect(right, True)
        self.assertFalse(rejected["valid"])
        self.assertEqual(rejected["reason"],
                         "SINGLE_EDGE_SIDE_SWITCH_CENTER_MISMATCH")
        self.assertGreater(
            tracker._switch_max_center_delta2 * 0.5, 12.0)

    def test_skipped_or_rejected_frames_break_switch_pending_streak(self):
        barriers = (
            ("no_fresh", lambda tracker: tracker.detect(
                _mask("right"), False)),
            ("invalid_mask", lambda tracker: tracker.detect({}, True)),
            ("topology_branch", lambda tracker: tracker.detect(
                _mask("right"), True, {"branch": True})),
            ("both_sides", lambda tracker: tracker.detect(
                _mask("both"), True)),
            ("no_candidate", lambda tracker: tracker.detect(
                _remove_edge_rows(_mask("left"), FLOOR, HEIGHT - 1), True)),
            ("no_detect", lambda tracker: (
                tracker.begin_frame(), tracker.skip_frame("PATH_BLOCKED"))),
        )
        for name, barrier in barriers:
            with self.subTest(barrier=name):
                tracker = _switch_tracker()
                first = tracker.detect(_mask("right"), True)
                self.assertEqual(first["reason"],
                                 "SINGLE_EDGE_SIDE_SWITCH_PENDING")
                barrier(tracker)
                after = tracker.detect(_mask("right"), True)
                self.assertFalse(after["valid"])
                self.assertEqual(after["reason"],
                                 "SINGLE_EDGE_SIDE_SWITCH_PENDING")
                self.assertEqual(tracker._pending_switch_count, 1)

    def test_extended_diagnostics_keep_last_and_first_freeze_snapshot(self):
        _warm_tracker(self.tracker)
        result = self.tracker.detect(_mask("left"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.tracker.freeze_diagnostic_snapshot()
        frozen_reason = self.tracker._frozen_frame_reason
        frozen_rows = self.tracker._frozen_frame_rows
        self.assertEqual(frozen_reason, "OK_SINGLE_EDGE_LEFT")
        self.assertEqual(frozen_rows, result["selected_row_count"])

        failed = self.tracker.detect(_blank_bright_mask(), True)
        self.assertFalse(failed["valid"])
        self.assertNotEqual(self.tracker._last_frame_reason, frozen_reason)
        self.assertEqual(self.tracker._frozen_frame_reason, frozen_reason)
        self.assertEqual(self.tracker._frozen_frame_rows, frozen_rows)
        output = io.StringIO()
        with patch("sys.stdout", output):
            self.tracker.dump_extended_diagnostics(config.BUILD_ID)
        self.assertIn("FROZEN_FRAME", output.getvalue())
        self.assertIn("reference_y=40 weak_support=0", output.getvalue())
        self.assertIn("selected_chains,1", output.getvalue())

    def test_switching_observed_sides_requires_bilateral_reacquisition(self):
        _warm_tracker(self.tracker)
        left = self.tracker.detect(_mask("left"), True)
        self.assertTrue(left["valid"], left["reason"])
        self.tracker.commit_single(left)
        switched = self.tracker.detect(_mask("right"), True)
        self.assertFalse(switched["valid"])
        self.assertEqual(switched["reason"],
                         "SINGLE_EDGE_SIDE_SWITCH_PENDING")
        dual = _mask("both")
        candidate = _bilateral_candidate()
        self.assertGreater(self.tracker.observe_bilateral(
            candidate, dual["luma"], dual["edge_mask"], dual["road_mask"],
            dual["known_region"], True, True), 0)
        switched = self.tracker.detect(_mask("right"), True)
        self.assertTrue(switched["valid"], switched["reason"])


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(FastSingleEdgeTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
