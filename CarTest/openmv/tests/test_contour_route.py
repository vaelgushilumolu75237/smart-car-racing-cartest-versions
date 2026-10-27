"""Host checks for the guarded contour-to-row-mid fallback adapter."""

from array import array
from pathlib import Path
import sys
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import config  # noqa: E402
import contour_route  # noqa: E402
from contour_route import ContourRouteRecovery  # noqa: E402
import contour_trace  # noqa: E402


WIDTH = 80
HEIGHT = 60
FLOOR = config.SURFACE_ROI_FLOOR_Y


def _mask(left=25, right=54, floor=FLOOR, top=0):
    road = bytearray(WIDTH * HEIGHT)
    known = bytearray([1]) * (WIDTH * HEIGHT)
    edge = bytearray([1]) * (WIDTH * HEIGHT)
    row_counts = bytearray([WIDTH]) * HEIGHT
    for y in range(top, HEIGHT - 1):
        for x in range(left, right + 1):
            road[y * WIDTH + x] = 255
    return {
        "width": WIDTH,
        "height": HEIGHT,
        "floor": floor,
        "road_mask": road,
        "known_region": known,
        "edge_mask": edge,
        "row_counts": row_counts,
    }


def _prior(reason="INSUFFICIENT_FRESH_ROWS", topology=None):
    return {
        "valid": False,
        "reason": reason,
        "scene": "ROAD_CANDIDATE",
        "topology_evidence": topology or {
            "ambiguous": False,
            "branch_evidence": False,
            "island_evidence": False,
            "reason": "PARTIAL_KNOWN_ROI",
        },
    }


class ContourRouteTests(unittest.TestCase):
    def test_host_backend_labels_keep_python_bounded_fallback(self):
        self.assertFalse(contour_route._IS_MICROPYTHON)
        self.assertFalse(contour_trace._IS_MICROPYTHON)
        self.assertEqual("python-bounded", contour_route.COMPILER_BACKEND)
        self.assertEqual("python-bounded", contour_trace.COMPILER_BACKEND)

    def test_continuous_bilateral_path_recovers_row_mid_candidate(self):
        topology = {"ambiguous": False, "branch_evidence": False,
                    "island_evidence": False, "reason": "PARTIAL_KNOWN_ROI"}
        prior = _prior(topology=topology)
        recovery = ContourRouteRecovery(config)
        result = recovery.detect(_mask(), prior)
        self.assertIs(result["topology_evidence"], topology)
        self.assertTrue(result["valid"])
        self.assertEqual("OK_CONTOUR_BILATERAL", result["reason"])
        self.assertEqual("current_contour_bilateral",
                         result["reference_provenance"])
        self.assertEqual((25 + 54) * 0.5, result["reference_center_x"])
        self.assertEqual(4, result["selected_row_stride"])
        self.assertIsNone(result["selected_rows"])
        self.assertIsNone(result["boundary_evidence"])
        self.assertGreaterEqual(result["selected_row_count"],
                                recovery.min_rows)
        self.assertLessEqual(
            abs(result["reference_y"] - recovery.reference_y),
            recovery.reference_gap)
        self.assertTrue(recovery.used)
        self.assertTrue(recovery.trace_info["used"])
        self.assertEqual(recovery.workspace.left_count, recovery.Lcount)
        self.assertEqual(recovery.workspace.right_count, recovery.Rcount)

    def test_valid_prior_is_returned_as_same_untouched_object(self):
        prior = {"valid": True, "reason": "OK", "opaque": object()}
        recovery = ContourRouteRecovery(config)
        self.assertIs(prior, recovery.detect(_mask(), prior))
        self.assertEqual("OK", prior["reason"])
        self.assertEqual("PRIOR_ALREADY_VALID", recovery.reason)
        self.assertFalse(recovery.used)

    def test_early_reject_and_failed_trace_do_not_publish_prior_frame_rows(self):
        recovery = ContourRouteRecovery(config)
        mask = _mask()
        good_prior = _prior()
        first = recovery.detect(mask, good_prior)
        self.assertTrue(first["valid"])
        first_count = first["selected_row_count"]
        first_rows = tuple(recovery.selected_row_ints[:first_count * 4])
        first_trace = (
            recovery.trace_info["Lcount"],
            recovery.trace_info["Rcount"],
            recovery.trace_info["left_termination"],
            recovery.trace_info["right_termination"],
        )

        ambiguous_topology = {
            "ambiguous": True,
            "branch_evidence": False,
            "island_evidence": False,
            "reason": "AMBIGUOUS",
        }
        early_prior = _prior(topology=ambiguous_topology)
        early_prior["scene"] = "AMBIGUOUS_TOPOLOGY"
        self.assertIs(early_prior, recovery.detect(mask, early_prior))
        self.assertEqual("TOPOLOGY_UNRESOLVED", recovery.reason)
        self.assertFalse(recovery.used)
        self.assertEqual(0, recovery.selected_row_count)
        self.assertEqual(0, recovery.trace_info["selected_row_count"])
        self.assertEqual(0, recovery.trace_info["Lcount"])
        self.assertEqual(0, recovery.trace_info["Rcount"])
        self.assertFalse(recovery._result["valid"])
        self.assertEqual(0, recovery._result["selected_row_count"])
        self.assertIsNone(recovery._result["selected_rows"])
        self.assertIs(recovery.selected_row_ints,
                      recovery._result["selected_row_ints"])
        self.assertIs(recovery.workspace.left_points,
                      recovery.trace_info["left_points"])

        # A rejected trace follows a successful frame. The returned result
        # must not publish the prior candidate, while diagnostics describe
        # only the current trace.
        failed_mask = _mask(top=58)
        failed_prior = _prior()
        self.assertIs(failed_prior,
                      recovery.detect(failed_mask, failed_prior))
        self.assertEqual("INSUFFICIENT_FRESH_ROWS", recovery.reason)
        self.assertFalse(recovery._result["valid"])
        self.assertEqual(0, recovery._result["selected_row_count"])
        self.assertIsNone(recovery._result["selected_rows"])
        self.assertIs(recovery.selected_row_ints,
                      recovery._result["selected_row_ints"])
        self.assertEqual(1, recovery.selected_row_count)
        self.assertEqual(1, recovery.trace_info["selected_row_count"])
        self.assertGreater(recovery.trace_info["Lcount"], 0)
        self.assertEqual(recovery.workspace.left_count,
                         recovery.trace_info["Lcount"])
        self.assertEqual(recovery.workspace.right_count,
                         recovery.trace_info["Rcount"])
        self.assertFalse(recovery.trace_info["used"])

        next_prior = _prior()
        next_result = recovery.detect(mask, next_prior)
        fresh_recovery = ContourRouteRecovery(config)
        fresh_result = fresh_recovery.detect(mask, _prior())
        self.assertTrue(next_result["valid"])
        self.assertEqual(first_count, next_result["selected_row_count"])
        self.assertEqual(first_rows, tuple(
            recovery.selected_row_ints[:first_count * 4]))
        self.assertEqual(first_trace, (
            recovery.trace_info["Lcount"],
            recovery.trace_info["Rcount"],
            recovery.trace_info["left_termination"],
            recovery.trace_info["right_termination"],
        ))
        self.assertEqual(
            tuple(fresh_recovery.selected_row_ints[:first_count * 4]),
            tuple(recovery.selected_row_ints[:first_count * 4]))
        for key in ("valid", "reason", "lateral_error", "reference_y",
                    "reference_center_x", "vertical_span",
                    "selected_row_count", "selected_row_stride"):
            self.assertEqual(fresh_result.get(key), next_result.get(key), key)
        self.assertEqual(0, recovery.trace_info["ambiguous_row_count"])

    def test_topology_ambiguity_branch_and_island_are_rejected(self):
        mask = _mask()
        recovery = ContourRouteRecovery(config)
        for topology in (
                {"ambiguous": False, "branch_evidence": True,
                 "island_evidence": False, "reason": "OPEN_BAND"},
                {"ambiguous": False, "branch_evidence": False,
                 "island_evidence": True, "reason": "ISLAND"},
                {"ambiguous": True, "branch_evidence": False,
                 "island_evidence": False, "reason": "AMBIGUOUS"}):
            prior = _prior(topology=topology)
            result = recovery.detect(mask, prior)
            self.assertIs(result, prior)
            self.assertEqual("TOPOLOGY_UNRESOLVED", recovery.reason)
        ineligible = _prior("AMBIGUOUS_ROAD")
        self.assertIs(ineligible, recovery.detect(mask, ineligible))
        self.assertEqual("PRIOR_REASON_NOT_ELIGIBLE", recovery.reason)

    def test_rejected_results_keep_original_identity_and_scene(self):
        recovery = ContourRouteRecovery(config)
        ambiguous = _prior()
        ambiguous["scene"] = "AMBIGUOUS_CANDIDATE"
        original = dict(ambiguous)
        self.assertIs(ambiguous, recovery.detect(_mask(), ambiguous))
        self.assertEqual(original, ambiguous)
        self.assertEqual("TOPOLOGY_UNRESOLVED", recovery.reason)

        failed = _prior()
        failed["scene"] = "UNKNOWN"
        mask = _mask()
        for y in range(51, HEIGHT):
            mask["known_region"][y * WIDTH + 24] = 0
        original = dict(failed)
        self.assertIs(failed, recovery.detect(mask, failed))
        self.assertEqual(original, failed)
        self.assertNotEqual("OK_CONTOUR_BILATERAL", recovery.reason)

    def test_missing_topology_and_invalid_mask_metadata_return_prior(self):
        recovery = ContourRouteRecovery(config)
        missing_topology = {"valid": False,
                            "reason": "INSUFFICIENT_FRESH_ROWS",
                            "scene": "ROAD_CANDIDATE"}
        self.assertIs(missing_topology,
                      recovery.detect(_mask(), missing_topology))
        self.assertEqual("TOPOLOGY_EVIDENCE_MISSING", recovery.reason)

        for mask, expected_reason in (
                ({**_mask(), "floor": FLOOR + 1}, "MASK_INFO_INVALID"),
                ({**_mask(), "valid": 0}, "MASK_INVALID")):
            prior = _prior()
            before = dict(prior)
            self.assertIs(prior, recovery.detect(mask, prior))
            self.assertEqual(before, prior)
            self.assertEqual(expected_reason, recovery.reason)

    def test_route_limits_are_finite_and_minimum_rows_are_12(self):
        recovery = ContourRouteRecovery({
            "SURFACE_GRID_WIDTH": 80,
            "SURFACE_GRID_HEIGHT": 60,
            "ROAD_MIN_OBSERVED_ROWS": 4,
            "ROAD_MIN_VERTICAL_SPAN": 8,
        })
        self.assertEqual(12, recovery.min_rows)
        self.assertEqual(12, recovery.min_span)
        for field, invalid in (
                ("ROAD_MAX_CENTER_SLOPE", float("nan")),
                ("ROAD_LINK_TOLERANCE", float("inf")),
                ("ROAD_REFERENCE_Y", True),
                ("FAST_PATH_REFERENCE_MAX_GAP", -1.0),
                ("ROAD_MIN_OBSERVED_ROWS", True)):
            with self.subTest(field=field, invalid=invalid):
                with self.assertRaises(ValueError):
                    ContourRouteRecovery({field: invalid})

    def test_missing_clipped_unknown_and_border_only_edges_are_rejected(self):
        recovery = ContourRouteRecovery(config)
        for mask in (_mask(left=0, right=28), _mask(left=0, right=79)):
            prior = _prior()
            self.assertIs(prior, recovery.detect(mask, prior))

        unknown = _mask()
        for y in range(51, HEIGHT):
            unknown["known_region"][y * WIDTH + 24] = 0
        prior = _prior()
        self.assertIs(prior, recovery.detect(unknown, prior))
        self.assertNotEqual("OK_CONTOUR_BILATERAL", recovery.reason)

    def test_bottom_center_failure_does_not_jump_to_adjacent_track(self):
        mask = _mask(left=9, right=28)
        for y in range(0, HEIGHT - 1):
            for x in range(44, 66):
                mask["road_mask"][y * WIDTH + x] = 255
        recovery = ContourRouteRecovery(config)
        prior = _prior()
        self.assertIs(prior, recovery.detect(mask, prior))
        self.assertIn(recovery.reason,
                      ("SEED_ROAD_TOO_FAR_FROM_CENTER",
                       "SEED_AMBIGUOUS_CANDIDATES"))

    def test_same_row_multiple_real_boundaries_are_dropped(self):
        recovery = ContourRouteRecovery(config)
        mask = _mask()
        road = mask["road_mask"]
        y = 20
        for x in range(WIDTH):
            road[y * WIDTH + x] = 0
        for x in range(10, 16):
            road[y * WIDTH + x] = 255
        for x in range(40, 46):
            road[y * WIDTH + x] = 255
        recovery.workspace.left_points[0:4] = array("H", [y, 10, y, 40])
        recovery.workspace.left_dark_support[0:2] = bytes((1, 1))
        recovery.workspace.left_count = 2
        recovery._collect_trace_edges(road, mask["known_region"], FLOOR)
        self.assertEqual(1, recovery.trace_info["ambiguous_row_count"])
        self.assertEqual(0, recovery._row_valid[y])


if __name__ == "__main__":
    unittest.main()
