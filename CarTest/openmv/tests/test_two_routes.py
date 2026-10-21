"""Synthetic parity and safety tests for the two single-edge routes."""

import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(PACKAGE_DIR / "tests"))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config  # noqa: E402
import main  # noqa: E402
from fast_single_edge import (  # noqa: E402
    FLAG_CLIPPED_FRAME_RIGHT,
    FLAG_IDENTITY_ANCHORED,
    FLAG_OBSERVED_LEFT,
    FLAG_OBSERVED_RIGHT,
    FLAG_WIDTH_CACHE,
    FastSingleEdgeTracker,
    ROUTE_CACHED_WIDTH,
    ROUTE_CLIPPED_MIDPOINT,
)
from test_fast_single_edge import (  # noqa: E402
    FLOOR,
    HEIGHT,
    LEFT,
    RIGHT,
    WIDTH,
    _add_closed_dark_ring,
    _bilateral_candidate,
    _blank_bright_mask,
    _mask,
    _topology,
    _warm_tracker,
)


def _tracker(route):
    settings = dict(vars(config))
    settings["SINGLE_EDGE_ROUTE_MODE"] = route
    return FastSingleEdgeTracker(settings)


def _branch_fixture():
    mask = _blank_bright_mask()
    for y in range(28, 32):
        row = y * WIDTH
        for x in range(WIDTH):
            index = row + x
            mask["edge_mask"][index] = 1
            mask["road_mask"][index] = 0
            mask["luma"][index] = 25
            mask["row_counts"][y] += 1
    for y in range(35, HEIGHT):
        for x in (15, 64):
            index = y * WIDTH + x
            mask["edge_mask"][index] = 1
            mask["road_mask"][index] = 0
            mask["luma"][index] = 25
            mask["row_counts"][y] += 1
    return mask


def _record_extra_dark_step(mask, x, dark=25):
    for y in range(FLOOR, HEIGHT):
        index = y * WIDTH + x
        mask["edge_mask"][index] = 1
        mask["luma"][index] = dark
        # Keep this cell in the known bright corridor to model two
        # incompatible edge candidates in the same road mask.
    return mask


class _OverlayImage:
    def __init__(self):
        self.crosses = []
        self.lines = []
        self.strings = []

    def draw_cross(self, point, color):
        self.crosses.append((point, color))

    def draw_line(self, line, color):
        self.lines.append((line, color))

    def draw_string(self, point, value, **kwargs):
        self.strings.append((point, value))


class TwoRouteTests(unittest.TestCase):
    def test_same_left_frame_matches_both_routes_with_distinct_inference(self):
        mask = _mask("left")
        cached = _tracker(ROUTE_CACHED_WIDTH)
        _warm_tracker(cached)
        cached_result = cached.detect(mask, True)

        clipped = _tracker(ROUTE_CLIPPED_MIDPOINT)
        clipped_result = clipped.detect(mask, True)

        self.assertTrue(cached_result["valid"], cached_result["reason"])
        self.assertTrue(clipped_result["valid"], clipped_result["reason"])
        self.assertEqual(cached_result["reason"], "OK_SINGLE_EDGE_LEFT")
        self.assertEqual(cached_result["actual_boundary"], "LEFT")
        self.assertEqual(cached_result["actual_boundary_x"], 26.5)
        self.assertEqual(cached_result["clipped_endpoint"], "NONE")
        self.assertEqual(clipped_result["reason"],
                         "OK_CLIPPED_MIDPOINT_LEFT")
        self.assertEqual(clipped.cache_rows, 0)
        self.assertEqual(clipped_result["cache_rows"], 0)
        self.assertEqual(clipped_result["observed_both_rows"], 0)
        self.assertEqual(clipped_result["boundary_source"], "LEFT_ONLY")
        self.assertEqual(clipped_result["actual_boundary"], "LEFT")
        self.assertEqual(clipped_result["actual_boundary_x"], 26.5)
        self.assertEqual(clipped_result["clipped_endpoint"], "RIGHT_FRAME")
        self.assertEqual(clipped_result["clipped_endpoint_x"], WIDTH - 1)
        expected_midpoint = (26.5 + (WIDTH - 1)) * 0.5
        self.assertLessEqual(
            abs(clipped_result["reference_center_x"] - expected_midpoint),
            0.25)
        self.assertAlmostEqual(clipped_result["reference_center_x"], 53.0)
        self.assertGreater(abs(cached_result["reference_center_x"] -
                               clipped_result["reference_center_x"]), 10.0)
        for offset in range(clipped_result["selected_row_count"]):
            row = offset * clipped_result["selected_row_stride"]
            flags = clipped_result["selected_row_ints"][row + 4]
            self.assertTrue(flags & FLAG_OBSERVED_LEFT)
            self.assertTrue(flags & FLAG_CLIPPED_FRAME_RIGHT)
            self.assertFalse(flags & FLAG_WIDTH_CACHE)
            self.assertFalse(flags & FLAG_IDENTITY_ANCHORED)

    def test_right_edge_is_mirrored_to_left_frame_endpoint(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        result = tracker.detect(_mask("right"), True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["boundary_source"], "RIGHT_ONLY")
        self.assertEqual(result["actual_boundary"], "RIGHT")
        self.assertEqual(result["actual_boundary_x"], 52.5)
        self.assertEqual(result["clipped_endpoint"], "LEFT_FRAME")
        self.assertEqual(result["clipped_endpoint_x"], 0)
        expected_midpoint = 52.5 * 0.5
        self.assertLessEqual(
            abs(result["reference_center_x"] - expected_midpoint), 0.25)
        self.assertEqual(result["observed_both_rows"], 0)
        self.assertEqual(tracker.cache_rows, 0)
        for offset in range(result["selected_row_count"]):
            flags = result["selected_row_ints"][
                offset * result["selected_row_stride"] + 4]
            self.assertTrue(flags & FLAG_OBSERVED_RIGHT)
            self.assertFalse(flags & FLAG_WIDTH_CACHE)

    def test_clipped_route_needs_fresh_keep_but_no_width_cache(self):
        clipped = _tracker(ROUTE_CLIPPED_MIDPOINT)
        no_keep = clipped.detect(_mask("left"), False)
        self.assertFalse(no_keep["valid"])
        self.assertEqual(no_keep["reason"], "NO_FRESH_KEEP")
        self.assertTrue(clipped.detect(_mask("left"), True)["valid"])
        self.assertEqual(clipped.cache_rows, 0)

        cached = _tracker(ROUTE_CACHED_WIDTH)
        no_width = cached.detect(_mask("left"), True)
        self.assertFalse(no_width["valid"])
        self.assertEqual(no_width["reason"], "NO_WIDTH_CACHE")

    def test_frame_border_without_current_edge_and_unknown_corridor_reject(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        border_only = _blank_bright_mask()
        for y in range(FLOOR, HEIGHT):
            for x in (0, WIDTH - 1):
                index = y * WIDTH + x
                border_only["edge_mask"][index] = 1
                border_only["road_mask"][index] = 0
                border_only["luma"][index] = 25
        result = tracker.detect(border_only, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "NO_CURRENT_BOUNDARY_OR_WHITE_CLIP")

        unknown = _mask("left")
        for y in range(FLOOR, HEIGHT):
            unknown["known_region"][y * WIDTH + 60] = 0
        result = tracker.detect(unknown, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "NO_CURRENT_BOUNDARY_OR_WHITE_CLIP")

    def test_current_internal_ring_branch_and_ambiguous_topology_all_reject(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        ring = _mask("left")
        _add_closed_dark_ring(ring, left=39, top=33, width=8, height=14)
        ring_topology = _topology(ring)
        self.assertTrue(ring_topology["island_evidence"], ring_topology)
        result = tracker.detect(ring, True, ring_topology)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "TOPOLOGY_ISLAND_UNRESOLVED")

        branch = _branch_fixture()
        branch_topology = _topology(branch)
        self.assertTrue(branch_topology["branch_evidence"], branch_topology)
        result = tracker.detect(branch, True, branch_topology)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "TOPOLOGY_BRANCH_UNRESOLVED")

        result = tracker.detect(_mask("left"), True, {
            "reason": "AMBIGUOUS_TOPOLOGY", "ambiguous": True,
        })
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "TOPOLOGY_AMBIGUITY_UNRESOLVED")

    def test_multiple_current_edges_in_one_corridor_are_ambiguous(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        mask = _record_extra_dark_step(_mask("left"), x=45)
        result = tracker.detect(mask, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "AMBIGUOUS_SINGLE_EDGE_CHAIN")

    def test_both_reliable_sides_are_rejected_as_ambiguous(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        mask = _mask("left")

        def force_reliable_both_sides(side):
            base = side * HEIGHT
            tracker._side_count[side] = tracker.min_rows
            tracker._side_first[side] = FLOOR
            tracker._side_last[side] = FLOOR + tracker.min_span
            tracker._side_ref_y[side] = int(tracker.reference_y)
            tracker._side_ref_center2[side] = 80
            tracker._side_ambiguous[side] = 0
            return True

        tracker._evaluate_side = force_reliable_both_sides
        result = tracker.detect(mask, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "AMBIGUOUS_BOTH_SIDES")

    def test_clipped_route_reuses_support_span_and_continuity_checks(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        short = _mask("left")
        keep_rows = tracker.min_rows - 1
        for y in range(FLOOR + keep_rows, HEIGHT):
            row = y * WIDTH
            short["edge_mask"][row:row + WIDTH] = bytes(WIDTH)
        result = tracker.detect(short, True)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "SINGLE_EDGE_SUPPORT_ROWS")

        discontinuous = _mask("left")
        for y in range(FLOOR + 20, HEIGHT):
            row = y * WIDTH
            for x in range(WIDTH):
                discontinuous["road_mask"][row + x] = 0
                discontinuous["edge_mask"][row + x] = 1
                discontinuous["luma"][row + x] = 25
            for x in range(47, WIDTH):
                discontinuous["road_mask"][row + x] = 1
                discontinuous["edge_mask"][row + x] = 0
                discontinuous["luma"][row + x] = 220
        result = tracker.detect(discontinuous, True)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["selected_row_count"], HEIGHT - FLOOR - 20)
        self.assertEqual(result["selected_row_ints"][0], FLOOR + 20)
        self.assertEqual(result["selected_row_ints"][(
            result["selected_row_count"] - 1) * 5], HEIGHT - 1)

    def test_side_switch_requires_a_published_bilateral_reacquisition(self):
        tracker = _tracker(ROUTE_CLIPPED_MIDPOINT)
        left = tracker.detect(_mask("left"), True)
        self.assertTrue(left["valid"], left["reason"])
        tracker.commit_single(left)
        switched = tracker.detect(_mask("right"), True)
        self.assertFalse(switched["valid"])
        self.assertEqual(switched["reason"],
                         "SINGLE_EDGE_SIDE_SWITCH_PENDING")

        dual = _mask("both")
        accepted = tracker.observe_bilateral(
            _bilateral_candidate(), dual["luma"], dual["edge_mask"],
            dual["road_mask"], dual["known_region"], True, True)
        self.assertEqual(accepted, 0)
        self.assertEqual(tracker.cache_rows, 0)
        switched = tracker.detect(_mask("right"), True)
        self.assertTrue(switched["valid"], switched["reason"])
        self.assertEqual(switched["boundary_source"], "RIGHT_ONLY")
        self.assertEqual(tracker.cache_rows, 0)

    def test_each_cached_width_profile_build_validates(self):
        for profile, build in config.STEERING_CONTROL_PROFILE_BUILD_IDS:
            with patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"), \
                    patch.object(config, "STEERING_CONTROL_PROFILE", profile), \
                    patch.object(config, "BUILD_ID", build):
                self.assertTrue(main._validate_fast_config())
        with patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "clipped_midpoint"), \
                patch.object(config, "STEERING_CONTROL_PROFILE", "pd"), \
                patch.object(config, "BUILD_ID", dict(
                    config.STEERING_CONTROL_PROFILE_BUILD_IDS)["pd"]):
            with self.assertRaisesRegex(RuntimeError, "route/build mismatch"):
                main._validate_fast_config()
        with patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"), \
                patch.object(config, "STEERING_CONTROL_PROFILE", "pd"), \
                patch.object(config, "BUILD_ID",
                             "ARBITRARY_BUILD"):
            with self.assertRaisesRegex(RuntimeError, "profile/build mismatch"):
                main._validate_fast_config()

    def test_overlay_draws_midpoint_chain_boundary_and_clip_separately(self):
        image = _OverlayImage()
        rows = array("i", [
            20, 61, 41, 0, FLAG_OBSERVED_LEFT | FLAG_CLIPPED_FRAME_RIGHT,
            21, 59, 39, 0, FLAG_OBSERVED_LEFT | FLAG_CLIPPED_FRAME_RIGHT,
        ])
        result = {
            "route_mode": ROUTE_CLIPPED_MIDPOINT,
            "selected_row_ints": rows,
            "selected_row_count": 2,
            "selected_row_stride": 5,
            "actual_boundary": "LEFT",
            "actual_boundary_x": 20.5,
            "clipped_endpoint": "RIGHT_FRAME",
            "clipped_endpoint_x": WIDTH - 1,
            "reference_center_x": 30.5,
            "reference_y": 20.0,
            "reason": "OK_CLIPPED_MIDPOINT_LEFT",
        }
        selection = {
            "valid": True, "source": "single_left", "quality": 80,
            "result": result, "reason": result["reason"],
            "boundary_source": "LEFT_ONLY", "inferred": True,
        }
        reference = {"x": 30.5, "y": 20.0, "kind": "CLIPPED_MIDPOINT"}
        with patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                             ROUTE_CLIPPED_MIDPOINT):
            main._draw_gray_result(
                image, selection, {}, {}, reference, True, 0, 0, 5, 30.0)

        p0 = main._surface_pixel((30.5, 20))
        p1 = main._surface_pixel((29.5, 21))
        self.assertIn(((p0[0], p0[1], p1[0], p1[1]), 255), image.lines)
        boundary_points = {point for point, color in image.crosses
                           if color == 192}
        self.assertEqual(boundary_points, {
            main._surface_pixel((20.5, 20)),
            main._surface_pixel((19.5, 21)),
        })
        endpoint_points = {point for point, color in image.crosses
                           if color == 96}
        self.assertEqual(endpoint_points, {
            main._surface_pixel((WIDTH - 1, 20)),
            main._surface_pixel((WIDTH - 1, 21)),
        })
        self.assertTrue(all(point[1] < main._surface_pixel((0, HEIGHT - 1))[1]
                            for point in boundary_points))
        self.assertIn("BP15 SINGLE_LEFT", image.strings[1][1])
        self.assertLessEqual(len(image.strings), 5)
        self.assertEqual(image.strings[0][1], "NOW V CLIP/SINGLE_LEFT")

        invalid_image = _OverlayImage()
        invalid_selection = {
            "valid": False, "source": "NONE", "reason": "NO_VALID_ROAD",
            "single_edge_reason": "NO_FRESH_KEEP",
        }
        with patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                             ROUTE_CLIPPED_MIDPOINT):
            main._draw_gray_result(
                invalid_image, invalid_selection, {}, {}, None, False,
                0, 0, 5, 30.0)
        invalid_text = " ".join(
            value for point, value in invalid_image.strings)
        self.assertIn("NO_FRESH_KEEP", invalid_text)
        self.assertLessEqual(len(invalid_image.strings), 5)
        self.assertEqual(invalid_image.strings[0][1], "NOW INVALID CLIP")

    def test_fallback_diagnostic_names_route_and_keeps_full_reason(self):
        text = main._format_single_edge_diagnostic(
            "clipped_midpoint", "TOPOLOGY_ISLAND_UNRESOLVED", None)
        self.assertIn("route=clipped_midpoint", text)
        self.assertIn("fallback=TOPOLOGY_ISLAND_UNRESOLVED", text)
        self.assertNotIn("actual=NONE", text)
        self.assertNotIn("clip=NONE", text)


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(TwoRouteTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
