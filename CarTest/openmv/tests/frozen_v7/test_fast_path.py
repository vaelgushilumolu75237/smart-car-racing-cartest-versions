"""Host regressions for the bounded V7 same-frame path core."""

import sys
import unittest
from array import array
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OPENMV = ROOT / "openmv"
if str(OPENMV) not in sys.path:
    sys.path.insert(0, str(OPENMV))

import boundary_evidence
import config
import fast_path
from fast_path import FastPathDetector


WIDTH = 80
HEIGHT = 60


def mask_for_rows(center_fn=None, half_width=18, y_range=(20, 60),
                  extra_corridors=(), dark_luma=30, road_luma=220,
                  missing_right=False, known_hole=None):
    """Construct explicit host masks; no camera or thresholding is implied."""
    if center_fn is None:
        center_fn = lambda _y: 39.5
    road = bytearray(WIDTH * HEIGHT)
    edge = bytearray(WIDTH * HEIGHT)
    known = bytearray([1]) * (WIDTH * HEIGHT)
    luma = bytearray([dark_luma]) * (WIDTH * HEIGHT)
    for y in range(y_range[0], y_range[1]):
        center = center_fn(y)
        corridors = [(center - half_width, center + half_width)]
        corridors.extend(extra_corridors)
        for left_value, right_value in corridors:
            left = max(0, int(left_value + 0.5))
            right = min(WIDTH - 1, int(right_value + 0.5))
            for x in range(left, right + 1):
                index = y * WIDTH + x
                road[index] = 1
                luma[index] = road_luma
            if left > 0:
                edge[y * WIDTH + left - 1] = 1
            if right < WIDTH - 1:
                edge[y * WIDTH + right + 1] = 1
        if missing_right:
            # Keep the bright run but make its only right transition unknown.
            right = int(center + half_width + 0.5)
            if right + 1 < WIDTH:
                known[y * WIDTH + right + 1] = 0
        if known_hole is not None and known_hole[1] == y:
            known[y * WIDTH + known_hole[0]] = 0
    return {
        "valid": True,
        "width": WIDTH,
        "height": HEIGHT,
        "floor": 20,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": bytearray(
            sum(edge[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "road_row_counts": bytearray(
            sum(road[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "reason": "OK",
        "diagnostics": {},
    }


def merge_after_reference_masks():
    """Two observed routes split at y=40 and merge into one upper run."""
    road = bytearray(WIDTH * HEIGHT)
    edge = bytearray(WIDTH * HEIGHT)
    known = bytearray([1]) * (WIDTH * HEIGHT)
    luma = bytearray([30]) * (WIDTH * HEIGHT)

    def add_run(y, left, right):
        left = int(left)
        right = int(right)
        for x in range(left, right + 1):
            index = y * WIDTH + x
            road[index] = 1
            luma[index] = 220
        if left > 0:
            edge[y * WIDTH + left - 1] = 1
        if right + 1 < WIDTH:
            edge[y * WIDTH + right + 1] = 1

    for y in range(20, 60):
        if y >= 49:
            add_run(y, 36, 43)
        elif y >= 40:
            t = (48 - y) / 8.0
            left_center = 35.0 - 15.0 * t
            right_center = 44.0 + 15.0 * t
            add_run(y, int(left_center - 3.5 + 0.5),
                    int(left_center - 3.5 + 0.5) + 7)
            add_run(y, int(right_center - 3.5 + 0.5),
                    int(right_center - 3.5 + 0.5) + 7)
        elif y >= 24:
            t = (39 - y) / 15.0
            left_center = 22.0 + 13.0 * t
            right_center = 57.0 - 13.0 * t
            add_run(y, int(left_center - 3.5 + 0.5),
                    int(left_center - 3.5 + 0.5) + 7)
            add_run(y, int(right_center - 3.5 + 0.5),
                    int(right_center - 3.5 + 0.5) + 7)
        else:
            add_run(y, 31, 48)

    return {
        "valid": True, "width": WIDTH, "height": HEIGHT, "floor": 20,
        "luma": luma, "road_mask": road, "edge_mask": edge,
        "known_region": known,
        "row_counts": bytearray(
            sum(edge[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "road_row_counts": bytearray(
            sum(road[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "reason": "OK", "diagnostics": {},
    }


class FastPathDetectorTests(unittest.TestCase):
    def test_straight_path_emits_observed_inclusive_bright_runs_and_reference(self):
        result = FastPathDetector(config).detect(mask_for_rows())
        self.assertTrue(result["valid"], result["reason"])
        self.assertGreaterEqual(result["observed_rows"], 4)
        self.assertGreaterEqual(result["vertical_span"], 8)
        self.assertTrue(result["selected_rows"])
        first = result["selected_rows"][0]
        self.assertEqual(first["right"] - first["left"] + 1, first["width"])
        self.assertEqual(first["center"], (first["left"] + first["right"]) / 2.0)
        self.assertTrue(first["observed"])
        self.assertEqual(first["provenance"], "OBSERVED")
        self.assertEqual(first["left_status"], "OBSERVED")
        self.assertEqual(first["right_status"], "OBSERVED")
        self.assertEqual(result["controller_reference"]["y"], result["reference_y"])
        self.assertEqual(result["reference_provenance"], "OBSERVED")
        self.assertIsInstance(result["selected_row_ints"], array)
        self.assertEqual(result["selected_row_count"], result["active_row_count"])
        self.assertEqual(result["selected_row_stride"], 4)
        selected = result["selected_row_ints"]
        for index, row in enumerate(result["selected_rows"]):
            offset = index * result["selected_row_stride"]
            self.assertEqual(selected[offset], row["y"])
            self.assertEqual(selected[offset + 1], row["left"])
            self.assertEqual(selected[offset + 2], row["right"])
            self.assertEqual(selected[offset + 3], 7)

    def test_bilateral_same_frame_evidence_gate_accepts_detector_output(self):
        masks = mask_for_rows()
        candidate = FastPathDetector(config).detect(masks)
        self.assertTrue(candidate["valid"], candidate["reason"])
        boundary_evidence.gate_road_candidate(
            candidate, masks["luma"], masks["edge_mask"],
            masks["road_mask"], masks["known_region"], config)
        self.assertTrue(candidate["valid"], candidate.get("boundary_evidence"))
        self.assertEqual(candidate["boundary_evidence"]["status"], "PASS")
        selection = {"valid": True, "source": "road"}
        reference_check = boundary_evidence.validate_selected_reference(
            selection, candidate["controller_reference"], None, candidate,
            config)
        self.assertTrue(reference_check["valid"], reference_check)

    def test_curve_direction_and_normalized_lateral_sign(self):
        left_result = FastPathDetector(config).detect(
            mask_for_rows(center_fn=lambda y: 30.0 + 0.20 * (y - 20)))
        right_result = FastPathDetector(config).detect(
            mask_for_rows(center_fn=lambda y: 49.0 - 0.20 * (y - 20)))
        self.assertTrue(left_result["valid"], left_result["reason"])
        self.assertTrue(right_result["valid"], right_result["reason"])
        self.assertLess(left_result["lateral_error"], 0)
        self.assertGreater(right_result["lateral_error"], 0)

    def test_normalized_lateral_uses_half_of_last_valid_pixel(self):
        normalize = fast_path._normalize_center
        self.assertEqual(normalize(0.0, WIDTH), -1000)
        self.assertEqual(normalize(19.75, WIDTH), -500)
        self.assertEqual(normalize(39.5, WIDTH), 0)
        self.assertEqual(normalize(59.25, WIDTH), 500)
        self.assertEqual(normalize(79.0, WIDTH), 1000)

    def test_invalid_fixed_geometry_and_unbounded_search_config_are_rejected(self):
        invalid_configs = (
            {"SURFACE_GRID_WIDTH": 80.5},
            {"SURFACE_ROI_FLOOR_Y": 60},
            {"FAST_PATH_MAX_CANDIDATES_PER_ROW": 9},
            {"FAST_PATH_MIN_OBSERVED_ROWS": 3},
            {"FAST_PATH_MIN_VERTICAL_SPAN": 7},
            {"ROAD_MAX_CENTER_SLOPE": float("nan")},
            {"ROAD_LINK_TOLERANCE": float("inf")},
            {"ROAD_AMBIGUITY_MARGIN": -1.0},
            {"FAST_PATH_REFERENCE_MAX_GAP": -1.0},
            {"FAST_PATH_REFERENCE_MAX_GAP": float("inf")},
        )
        for update in invalid_configs:
            settings = dict(config.ROAD_PARAMS)
            settings.update(update)
            settings.setdefault("SURFACE_GRID_WIDTH", WIDTH)
            settings.setdefault("SURFACE_GRID_HEIGHT", HEIGHT)
            settings.setdefault("SURFACE_ROI_FLOOR_Y", 20)
            with self.subTest(update=update):
                with self.assertRaises(ValueError):
                    FastPathDetector(settings)

    def test_all_white_and_unpaired_or_unknown_edge_are_invalid(self):
        all_white = {
            "valid": True, "width": WIDTH, "height": HEIGHT,
            "floor": 20,
            "luma": bytearray([220]) * (WIDTH * HEIGHT),
            "road_mask": bytearray([1]) * (WIDTH * HEIGHT),
            "edge_mask": bytearray(WIDTH * HEIGHT),
            "known_region": bytearray([1]) * (WIDTH * HEIGHT),
            "row_counts": bytearray(HEIGHT),
            "road_row_counts": bytearray([WIDTH]) * HEIGHT,
        }
        all_white_result = FastPathDetector(config).detect(all_white)
        self.assertFalse(all_white_result["valid"])
        self.assertEqual(all_white_result["reason"], "NO_PAIRED_EDGES")

        missing = FastPathDetector(config).detect(
            mask_for_rows(missing_right=True))
        self.assertFalse(missing["valid"])
        self.assertEqual(missing["reason"], "UNKNOWN_REGION")

    def test_insufficient_current_frame_support_is_invalid(self):
        result = FastPathDetector(config).detect(
            mask_for_rows(y_range=(37, 41)))
        self.assertFalse(result["valid"])
        self.assertIn(result["reason"], ("INSUFFICIENT_FRESH_ROWS",
                                          "INSUFFICIENT_REFERENCE_SUPPORT"))

    def test_nearest_current_observation_can_shift_reference_without_extrapolation(self):
        masks = mask_for_rows(y_range=(20, 30))
        candidate = FastPathDetector(config).detect(masks)
        self.assertTrue(candidate["valid"], candidate["reason"])
        self.assertEqual(candidate["reference_target_y"], 40.0)
        self.assertEqual(candidate["reference_y"], 29.0)
        self.assertEqual(candidate["reference_shift_cells"], -11.0)
        self.assertEqual(candidate["reference_provenance"], "NEAREST_OBSERVED")
        self.assertEqual(candidate["controller_reference"]["kind"],
                         "nearest_observed_y")
        self.assertEqual(candidate["controller_reference"]["target_y"], 40.0)
        self.assertEqual(candidate["active_row_count"], 10)

        boundary_evidence.gate_road_candidate(
            candidate, masks["luma"], masks["edge_mask"],
            masks["road_mask"], masks["known_region"], config)
        self.assertEqual(candidate["boundary_evidence"]["status"], "PASS")
        reference_check = boundary_evidence.validate_selected_reference(
            {"valid": True, "source": "road"},
            candidate["controller_reference"], None, candidate, config)
        self.assertTrue(reference_check["valid"], reference_check)

    def test_reused_detector_clears_unknown_frame_and_recovers_only_new_support(self):
        detector = FastPathDetector(config)
        masks = mask_for_rows()
        original_buffers = tuple(masks[key] for key in (
            "luma", "road_mask", "edge_mask", "known_region",
            "row_counts", "road_row_counts"))
        result = detector.detect(masks)
        self.assertTrue(result["valid"], result["reason"])
        reused_result_id = id(result)
        reused_rows_id = id(result["selected_rows"])
        self.assertEqual(result["active_row_count"], 40)
        self.assertEqual(result["controller_reference"]["x"], 40.0)
        self.assertEqual(result["topology_evidence"]["reason"], "OK")
        selected_data_id = id(result["selected_row_ints"])
        first_row = result["selected_rows"][0]
        self.assertEqual(first_row["left_status"], "OBSERVED")
        self.assertEqual(first_row["right_status"], "OBSERVED")
        self.assertEqual(first_row["provenance"], "OBSERVED")

        bad_size = dict(masks)
        bad_size["width"] = WIDTH - 1
        invalid_size = detector.detect(bad_size)
        self.assertIs(invalid_size, result)
        self.assertFalse(invalid_size["valid"])
        self.assertEqual(invalid_size["quality"], 0)
        self.assertEqual(invalid_size["lateral_error"], 0)
        self.assertIsNone(invalid_size["center_x"])
        self.assertIsNone(invalid_size["reference_center_x"])
        self.assertIsNone(invalid_size["controller_reference"])
        self.assertEqual(invalid_size["selected_row_count"], 0)
        self.assertEqual(len(invalid_size["selected_rows"]), 0)
        self.assertIsNone(invalid_size["topology_evidence"])

        # Keep the same frame dictionary and buffers but make the current
        # observation unknown. The prior row set/reference/topology cannot
        # survive this call.
        masks["known_region"][:] = bytearray(WIDTH * HEIGHT)
        invalid = detector.detect(masks)
        self.assertIs(invalid, result)
        self.assertFalse(invalid["valid"])
        self.assertEqual(invalid["quality"], 0)
        self.assertEqual(invalid["lateral_error"], 0)
        self.assertIsNone(invalid["center_x"])
        self.assertIsNone(invalid["reference_center_x"])
        self.assertIsNone(invalid["controller_reference"])
        self.assertEqual(invalid["active_row_count"], 0)
        self.assertEqual(invalid["selected_row_count"], 0)
        self.assertEqual(len(invalid["selected_rows"]), 0)
        self.assertEqual(invalid["topology_evidence"]["reason"],
                         "UNKNOWN_REGION")

        # Refill the same buffers with a shifted current-frame corridor and
        # prove the output returns only those coordinates and the new nearest
        # observed reference.
        fresh = mask_for_rows(center_fn=lambda _y: 52.5, y_range=(20, 30))
        for key in ("luma", "road_mask", "edge_mask", "known_region",
                    "row_counts", "road_row_counts"):
            masks[key][:] = fresh[key]
        masks["valid"] = True
        recovered = detector.detect(masks)
        self.assertIs(recovered, result)
        self.assertEqual(id(recovered), reused_result_id)
        self.assertEqual(id(recovered["selected_rows"]), reused_rows_id)
        self.assertEqual(id(recovered["selected_row_ints"]), selected_data_id)
        self.assertTrue(recovered["valid"], recovered["reason"])
        self.assertEqual(recovered["active_row_count"], 10)
        self.assertEqual(len(recovered["selected_rows"]), 10)
        self.assertEqual(recovered["selected_row_count"], 10)
        self.assertEqual(recovered["reference_y"], 29.0)
        self.assertEqual(recovered["controller_reference"]["x"], 53.0)
        self.assertEqual(recovered["controller_reference"]["y"], 29.0)
        self.assertEqual(recovered["topology_evidence"]["reason"], "OK")
        self.assertIs(recovered["selected_rows"][0], first_row)
        self.assertEqual(first_row["y"], 29)
        self.assertEqual(first_row["center"], 53.0)
        self.assertEqual(first_row["left_status"], "OBSERVED")
        self.assertEqual(first_row["right_status"], "OBSERVED")
        self.assertEqual(first_row["provenance"], "OBSERVED")
        for key, original in zip((
                "luma", "road_mask", "edge_mask", "known_region",
                "row_counts", "road_row_counts"), original_buffers):
            self.assertIs(masks[key], original)

        boundary_evidence.gate_road_candidate(
            recovered, masks["luma"], masks["edge_mask"],
            masks["road_mask"], masks["known_region"], config)
        reference_check = boundary_evidence.validate_selected_reference(
            {"valid": True, "source": "road"},
            recovered["controller_reference"], None, recovered, config)
        self.assertTrue(reference_check["valid"], reference_check)

    def test_candidate_overflow_is_reported_instead_of_truncated(self):
        narrow_config = dict(config.ROAD_PARAMS)
        narrow_config.update({
            "SURFACE_GRID_WIDTH": WIDTH,
            "SURFACE_GRID_HEIGHT": HEIGHT,
            "SURFACE_ROI_FLOOR_Y": 20,
            "FAST_PATH_MAX_CANDIDATES_PER_ROW": 2,
        })
        masks = mask_for_rows(y_range=(20, 60), extra_corridors=((4, 14),
                                                                  (64, 74)))
        result = FastPathDetector(narrow_config).detect(masks)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "ROW_CANDIDATE_OVERFLOW")
        self.assertEqual(result["quality"], 0)
        self.assertEqual(result["lateral_error"], 0)

    def test_topology_kernel_bounds_fails_closed_and_clears_reused_result(self):
        detector = FastPathDetector(config)
        masks = mask_for_rows()
        result = detector.detect(masks)
        self.assertTrue(result["valid"], result["reason"])
        self.assertGreater(result["selected_row_count"], 0)

        class BoundsFailureGuard:
            def check(self, _mask_info):
                return {
                    "island_evidence": False,
                    "branch_evidence": False,
                    "ambiguous": False,
                    "reason": "KERNEL_BOUNDS",
                }

        detector.topology_guard = BoundsFailureGuard()
        invalid = detector.detect(masks)
        self.assertIs(invalid, result)
        self.assertFalse(invalid["valid"])
        self.assertEqual(invalid["reason"], "KERNEL_BOUNDS")
        self.assertEqual(invalid["quality"], 0)
        self.assertEqual(invalid["lateral_error"], 0)
        self.assertEqual(invalid["reference_lateral_error"], 0)
        self.assertIsNone(invalid["center_x"])
        self.assertIsNone(invalid["reference_center_x"])
        self.assertIsNone(invalid["controller_reference"])
        self.assertEqual(invalid["active_row_count"], 0)
        self.assertEqual(invalid["selected_row_count"], 0)
        self.assertEqual(len(invalid["selected_rows"]), 0)
        self.assertEqual(invalid["topology_evidence"]["reason"],
                         "KERNEL_BOUNDS")

    def test_unknown_topology_fails_closed_and_clears_reused_result(self):
        detector = FastPathDetector(config)
        masks = mask_for_rows()
        result = detector.detect(masks)
        self.assertTrue(result["valid"], result["reason"])
        self.assertGreater(result["selected_row_count"], 0)

        class UnknownRegionGuard:
            def check(self, _mask_info):
                return {
                    "island_evidence": False,
                    "branch_evidence": False,
                    "ambiguous": True,
                    "reason": "UNKNOWN_REGION",
                }

        detector.topology_guard = UnknownRegionGuard()
        invalid = detector.detect(masks)
        self.assertIs(invalid, result)
        self.assertFalse(invalid["valid"])
        self.assertEqual(invalid["reason"], "UNKNOWN_REGION")
        self.assertEqual(invalid["quality"], 0)
        self.assertEqual(invalid["lateral_error"], 0)
        self.assertEqual(invalid["reference_lateral_error"], 0)
        self.assertIsNone(invalid["center_x"])
        self.assertIsNone(invalid["reference_center_x"])
        self.assertIsNone(invalid["controller_reference"])
        self.assertEqual(invalid["active_row_count"], 0)
        self.assertEqual(invalid["selected_row_count"], 0)
        self.assertEqual(len(invalid["selected_rows"]), 0)
        self.assertEqual(invalid["topology_evidence"]["reason"],
                         "UNKNOWN_REGION")

    def test_competing_well_supported_corridors_are_invalid(self):
        masks = mask_for_rows(extra_corridors=((4, 15), (64, 75)),
                              half_width=12)
        result = FastPathDetector(config).detect(masks)
        self.assertFalse(result["valid"])
        self.assertIn(result["reason"], ("AMBIGUOUS_ROAD", "ROW_CANDIDATE_OVERFLOW"))

    def test_divergent_routes_that_merge_before_endpoint_are_still_ambiguous(self):
        detector = FastPathDetector(config)
        result = detector.detect(merge_after_reference_masks())
        self.assertFalse(result["valid"], result["reason"])
        self.assertEqual(result["reason"], "AMBIGUOUS_ROAD")

        # Both near-score predecessor chains end on the same single observed
        # run at y=20, while their shared observation at controller y=40 is
        # separated. This specifically guards against endpoint-only checks.
        endpoint_layer = HEIGHT - 1 - 20
        self.assertEqual(detector._counts[endpoint_layer], 1)
        endpoint = detector._candidate_node(endpoint_layer, 0)
        first_state = detector._state_id(endpoint, 0)
        second_state = first_state + 1
        self.assertGreaterEqual(detector._state_quality[first_state], 0)
        self.assertGreaterEqual(detector._state_quality[second_state], 0)
        winner = (first_state if detector._scores[first_state] >=
                  detector._scores[second_state] else second_state)
        competitor = (second_state if winner == first_state else first_state)
        self.assertLessEqual(
            detector._scores[winner] - detector._scores[competitor],
            detector.ambiguity_margin)
        self.assertTrue(detector._competing_path_separates(competitor))

        # Field 9 now contains the layer-propagated divergence summary. Check
        # every viable state against the former parent-chain definition so a
        # merged route cannot lose separation during the optimization.
        for layer in range(detector.row_count):
            for slot in range(detector._counts[layer]):
                node = layer * detector.max_per_row + slot
                for state_slot in range(2):
                    state = node * 2 + state_slot
                    if detector._state_quality[state] >= 0:
                        self.assertEqual(
                            int(detector._state_data[
                                state * detector.STATE_STRIDE + 9]),
                            int(detector._competing_path_separates(state)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
