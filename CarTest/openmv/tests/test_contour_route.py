"""Host checks for the guarded contour-to-row-mid fallback adapter."""

from array import array
from pathlib import Path
import sys
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import config  # noqa: E402
from contour_route import ContourRouteRecovery  # noqa: E402


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
