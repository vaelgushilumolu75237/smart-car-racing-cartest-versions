"""Synthetic host tests for bounded single-edge inference and width cache."""

from array import array
from pathlib import Path
import sys
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import config  # noqa: E402
from fast_single_edge import FastSingleEdgeTracker  # noqa: E402
from fast_topology import FastTopologyGuard  # noqa: E402


WIDTH = 80
HEIGHT = 60
FLOOR = 20
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
        self.assertGreater(14, self.tracker.max_anchor2 // 2)

    def test_switching_observed_sides_requires_bilateral_reacquisition(self):
        _warm_tracker(self.tracker)
        left = self.tracker.detect(_mask("left"), True)
        self.assertTrue(left["valid"], left["reason"])
        self.tracker.commit_single(left)
        switched = self.tracker.detect(_mask("right"), True)
        self.assertFalse(switched["valid"])
        self.assertEqual(switched["reason"],
                         "SINGLE_EDGE_SIDE_SWITCH_REQUIRES_BILATERAL")
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
