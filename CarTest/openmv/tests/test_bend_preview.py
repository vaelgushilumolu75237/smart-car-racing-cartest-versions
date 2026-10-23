"""Current-route heading and nearby-reference regressions; host evidence only."""
from array import array
import unittest
from test_turn_response_weights import (
    config, main, fast_gray, fast_path, _path_candidate, _BufferImage)
import fast_boundary
import fast_single_edge
from contour_route import ContourRouteRecovery


def aggregate(workspace, points, support=None, valid=True,
              preview_enabled=True):
    candidate = _path_candidate([(y, x - 10, x + 10) for y, x in points])
    candidate["boundary_support_rows"] = bytearray(
        support if support is not None else [1] * len(points))
    candidate["boundary_support_count"] = len(points)
    selection = {"valid": valid, "source": "road", "result": candidate}
    return workspace.aggregate(selection, candidate, config)


def _preview_info():
    return {
        "position_p": 0, "derivative_d": 0, "pd_output": 0,
        "preview_candidate": 0, "preview_gain_milli": 0,
        "preview_applied": 0, "capture_dt_ms": 0xFFFF,
        "reset_code": 0, "profile_code": 2, "pd_enabled": True,
        "preview_enabled": True, "preview_confirmed": False,
        "preview_geometry_valid": False, "preview_same_sign": False,
        "preview_state": 0, "duplicate": False,
    }


def _preview_step(preview, info, now, dt, candidate, position,
                  geometry=True):
    return preview.update(
        now, dt, True, True, "road", "OBSERVED_BOTH",
        "ROAD_REFERENCE", 40, geometry, candidate, position,
        lambda newer, older: newer - older, info)


def _paired_contour_frame():
    width = config.SURFACE_GRID_WIDTH
    height = config.SURFACE_GRID_HEIGHT
    floor = config.SURFACE_ROI_FLOOR_Y
    road = bytearray(width * height)
    edge = bytearray(width * height)
    luma = bytearray([25]) * (width * height)
    known = bytearray([1]) * (width * height)
    for y in range(height - 1):
        row = y * width
        for x in range(25, 55):
            road[row + x] = 255
            luma[row + x] = 220
        for x in range(width):
            if road[row + x] == 0:
                edge[row + x] = 1
    return {
        "width": width, "height": height, "floor": floor,
        "road_mask": road, "edge_mask": edge,
        "luma": luma, "known_region": known,
        "valid": True,
    }


def _clear_prior(reason="INSUFFICIENT_FRESH_ROWS"):
    return {
        "valid": False, "reason": reason, "scene": "ROAD_CANDIDATE",
        "topology_evidence": {
            "reason": "PARTIAL_KNOWN_ROI", "ambiguous": False,
            "branch_evidence": False, "island_evidence": False,
        },
    }


class BendPreviewTests(unittest.TestCase):
    def setUp(self):
        self.w = main._FastRowWeightWorkspace()

    def test_position_pd_uses_capture_time_and_reports_only_applied_d(self):
        pd = main._FastPositionPD(config)
        pd.update(0, None, True, 0, "road", "OBSERVED_BOTH",
                  "ROAD_REFERENCE", 40, "pd", lambda a, b: a - b)
        second = pd.update(
            20, 20, True, 1000, "road", "OBSERVED_BOTH",
            "ROAD_REFERENCE", 40, "pd", lambda a, b: a - b)
        self.assertEqual(second["derivative_d"], 100)
        self.assertEqual(second["pd_output"], 1000)
        duplicate = pd.update(
            20, 0, True, 1000, "road", "OBSERVED_BOTH",
            "ROAD_REFERENCE", 40, "pd", lambda a, b: a - b)
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(duplicate["derivative_d"], 100)
        self.assertEqual(duplicate["pd_output"], 1000)

        no_pd = main._FastPositionPD(config)
        no_pd.update(0, None, True, 0, "road", "OBSERVED_BOTH",
                     "ROAD_REFERENCE", 40, "no_pd", lambda a, b: a - b)
        applied = no_pd.update(
            20, 20, True, 1000, "road", "OBSERVED_BOTH",
            "ROAD_REFERENCE", 40, "no_pd", lambda a, b: a - b)
        self.assertEqual(applied["derivative_d"], 0)
        self.assertEqual(applied["pd_output"], 1000)

    def test_pd_resets_on_exact_reference_and_boundary_identity_change(self):
        pd = main._FastPositionPD(config)
        pd.update(0, None, True, 0, "road", "OBSERVED_BOTH",
                  "ROAD_REFERENCE", 40, "pd", lambda a, b: a - b)
        reference = pd.update(
            20, 20, True, 100, "road", "OBSERVED_BOTH",
            "ROAD_REFERENCE", 41, "pd", lambda a, b: a - b)
        self.assertEqual((reference["derivative_d"], reference["reset_code"]),
                         (0, 5))
        boundary = pd.update(
            40, 20, True, 200, "road", "LEFT_ONLY",
            "ROAD_REFERENCE", 41, "pd", lambda a, b: a - b)
        self.assertEqual((boundary["derivative_d"], boundary["reset_code"]),
                         (0, 4))

    def test_preview_confirmation_needs_three_distinct_frames_and_sixty_ms(self):
        preview = main._FastAdaptivePreview(config)
        info = _preview_info()
        self.assertEqual(_preview_step(preview, info, 0, None, 200, 100), 0)
        self.assertEqual(preview.stable_count, 1)
        self.assertEqual(_preview_step(preview, info, 0, 0, 200, 100), 0)
        self.assertEqual(preview.stable_count, 1)
        self.assertFalse(preview.confirmed)
        _preview_step(preview, info, 30, 30, 200, 100)
        self.assertFalse(preview.confirmed)
        applied = _preview_step(preview, info, 60, 30, 200, 100)
        self.assertTrue(preview.confirmed)
        self.assertEqual(preview.stable_count, 3)
        self.assertEqual(info["preview_gain_milli"], 113)
        self.assertGreater(applied, 0)
        self.assertEqual(info["preview_state"], 2)

    def test_preview_position_support_gain_cap_slew_and_exit_decay(self):
        preview = main._FastAdaptivePreview(config)
        info = _preview_info()
        _preview_step(preview, info, 0, None, 1000, 40)
        _preview_step(preview, info, 30, 30, 1000, 40)
        _preview_step(preview, info, 60, 30, 1000, 40)
        self.assertFalse(preview.confirmed)
        _preview_step(preview, info, 90, 30, 1000, 250)
        self.assertTrue(preview.confirmed)
        self.assertEqual(info["preview_gain_milli"], 450)
        self.assertLessEqual(preview.applied, 120)
        for tick in range(120, 420, 30):
            _preview_step(preview, info, tick, 30, 1000, 250)
        self.assertEqual(preview.applied, 120)
        before = preview.applied
        after = _preview_step(
            preview, info, 450, 30, 1000, 250, geometry=False)
        self.assertEqual(after, before - 15)
        self.assertEqual(info["preview_state"], 3)
        for tick in range(480, 720, 30):
            _preview_step(preview, info, tick, 30, -200, 250)
        self.assertFalse(preview.confirmed)
        self.assertLessEqual(preview.applied, 0)

    def test_position_only_profile_keeps_preview_as_diagnostic(self):
        points = [(y, 46 if y < 33 else 38 if y < 46 else 36)
                  for y in range(20, 59)]
        result = dict(aggregate(self.w, points, preview_enabled=False))
        self.assertTrue(result["valid"], result)
        self.assertEqual(result["preview_candidate"], 400)
        self.assertEqual(result["heading_contribution"], 0)
        self.assertEqual(result["steering_error"], result["position_error"])
        self.assertEqual(result["preview_rejection_reason"],
                         "AWAITING_CONTROL_PROFILE")
        text = main._build_gray_overlay_text_cache(
            {"valid": True, "source": "road", "ring_state": "IDLE"},
            {}, {}, None, True, result["steering_error"],
            result["steering_error"], 0, 20.0, weight_info=result)
        self.assertTrue(any("CAN+400" in line for line in text))
        self.assertTrue(any("PD" in line for line in text))

    def test_supported_reference_error_is_diagnostic_only_and_near_y40(self):
        points = [(y, 43 if y == 40 else 42) for y in range(20, 59)]
        result = aggregate(self.w, points)
        self.assertEqual(result["reference_row_y"], 40)
        self.assertEqual(
            result["reference_error"],
            main._normalize_fast_reference(43, config.SURFACE_GRID_WIDTH))

        support = [1] * len(points)
        support[40 - 20] = 0
        cached_support = aggregate(self.w, points, support)
        self.assertEqual(cached_support["reference_row_y"], 39)
        self.assertEqual(
            cached_support["reference_error"],
            main._normalize_fast_reference(42, config.SURFACE_GRID_WIDTH))

        no_near_reference = aggregate(
            self.w, [(y, 42) for y in range(20, 35)])
        self.assertIsNone(no_near_reference["reference_row_y"])
        self.assertIsNone(no_near_reference["reference_error"])

        common_rows = [(y, 40) for y in range(20, 26)]
        low_edge = dict(aggregate(
            self.w, common_rows + [(36, 43), (45, 42)]))
        high_edge = dict(aggregate(
            self.w, common_rows + [(35, 42), (44, 44)]))
        self.assertEqual(low_edge["reference_row_y"], 36)
        self.assertEqual(high_edge["reference_row_y"], 44)
        self.assertEqual(low_edge["steering_error"],
                         low_edge["position_error"])
        self.assertEqual(high_edge["steering_error"],
                         high_edge["position_error"])

    def test_y40_reference_error_matches_left_safe_reference_diagnostic(self):
        points = [(y, 35 if y == 40 else 36) for y in range(20, 59)]
        result = dict(aggregate(self.w, points))
        self.assertEqual(result["reference_row_y"], 40)
        self.assertEqual(result["reference_error"], -114)
        self.assertEqual(result["steering_error"], result["position_error"])

    def test_single_edge_cached_rows_can_supply_reference_diagnostic(self):
        rows = array("i", [
            20, 80, 40, 0, 3,
            22, 80, 40, 0, 3,
            24, 80, 40, 0, 3,
            28, 80, 40, 0, 3,
            30, 80, 40, 0, 3,
            40, 84, 40, 0, 3,
            44, 86, 40, 0, 3,
        ])
        selection = {
            "valid": True,
            "source": "single_left",
            "result": {"selected_row_ints": rows,
                       "selected_row_count": 7,
                       "selected_row_stride": 5},
        }
        result = self.w.aggregate(selection, {}, config)
        self.assertTrue(result["valid"], result)
        self.assertEqual(result["reference_row_y"], 40)
        self.assertEqual(
            result["reference_error"],
            main._normalize_fast_reference(42, config.SURFACE_GRID_WIDTH))

    def test_integrated_mirrored_bend_candidates_remain_diagnostic(self):
        right_bend = [(y, x - 4) for y, x in
                      [(y, 46 if y < 33 else 38 if y < 46 else 36)
                       for y in range(20, 59)]]
        left_bend = [(y, 79 - x) for y, x in right_bend]
        left = dict(aggregate(self.w, right_bend))
        right = dict(aggregate(self.w, left_bend))
        self.assertLess(left["position_error"], 0)
        self.assertGreater(left["preview_candidate"], 0)
        self.assertEqual(left["heading_contribution"], 0)
        self.assertEqual(left["steering_error"], left["position_error"])
        self.assertGreater(right["position_error"], 0)
        self.assertLess(right["preview_candidate"], 0)
        self.assertEqual(right["heading_contribution"], 0)
        self.assertEqual(right["steering_error"], right["position_error"])

    def test_overlay_shows_position_applied_candidate_and_control(self):
        text = main._build_gray_overlay_text_cache(
            {"valid": True, "source": "road", "ring_state": "IDLE"},
            {}, {}, None, True, 305, 305, 0, 20.0,
            weight_info={
                "valid": True, "position_error": -25,
                "heading_error": 330, "heading_contribution": 330,
                "preview_candidate": 330,
                "preview_curvature_cells": 2.0, "preview_valid": True,
                "preview_rejection_reason": None,
                "preview_near_x": 42.0, "preview_near_y": 40,
                "preview_far_x": 45.0, "preview_far_y": 22,
                "reference_error": 24,
            })
        self.assertTrue(any("P-25 D+0 CAN+330" in line for line in text))
        self.assertTrue(any("CMD+305" in line for line in text))

        mirrored_text = main._build_gray_overlay_text_cache(
            {"valid": True, "source": "road", "ring_state": "IDLE"},
            {}, {}, None, True, 311, 311, 0, 20.0,
            weight_info={
                "valid": True, "position_error": -43,
                "heading_error": 354, "heading_contribution": 354,
                "preview_candidate": 354,
                "preview_curvature_cells": 1.5, "preview_valid": True,
                "preview_rejection_reason": None,
                "preview_near_x": 42.0, "preview_near_y": 40,
                "preview_far_x": 45.0, "preview_far_y": 22,
                "reference_error": -12,
            })
        self.assertTrue(any("P-43 D+0 CAN+354" in line
                            for line in mirrored_text))
        self.assertTrue(any("CMD+311" in line for line in mirrored_text))

    def test_mirrored_position_and_heading_remain_symmetric(self):
        right = [(y, 46 if y < 33 else 38 if y < 46 else 36)
                 for y in range(20, 59)]
        left = [(y, 79 - x) for y, x in right]
        a = dict(aggregate(self.w, right))
        b = dict(aggregate(self.w, left))
        self.assertEqual(a["raw_error"], -b["raw_error"])
        self.assertEqual(a["steering_error"], a["position_error"])
        self.assertEqual(b["steering_error"], b["position_error"])
        self.assertGreater(a["preview_candidate"], 0)
        self.assertEqual(a["preview_candidate"],
                         -b["preview_candidate"])
        self.assertEqual(a["heading_contribution"], 0)
        self.assertEqual(b["heading_contribution"], 0)

    def test_parallel_centered_and_offset_straights_have_no_preview(self):
        for x in (35, 40, 45):
            result = aggregate(self.w, [(y, x) for y in range(20, 59)])
            self.assertTrue(result["preview_valid"])
            self.assertEqual(result["heading_contribution"], 0)
            self.assertEqual(result["steering_error"], result["raw_error"])

    def test_mirror_and_current_frame_direction_reversal(self):
        points = [(y, 40 + (58 - y) ** 2 // 100) for y in range(20, 59)]
        a = dict(aggregate(self.w, points))
        b = dict(aggregate(self.w, [(y, 79 - x) for y, x in points]))
        self.assertEqual(a["steering_error"], -b["steering_error"])
        self.assertEqual(a["heading_contribution"], 0)
        self.assertEqual(b["heading_contribution"], 0)

    def test_diagonal_straights_are_not_mistaken_for_curved_entries(self):
        for points in ([(y, 20 + y // 2) for y in range(20, 59)],
                       [(y, 60 - y // 2) for y in range(20, 59)]):
            result = aggregate(self.w, points)
            self.assertTrue(result["preview_valid"])
            self.assertNotEqual(result["heading_error"], 0)
            self.assertEqual(result["heading_contribution"], 0)
            self.assertEqual(result["steering_error"], result["raw_error"])

    def test_position_shares_remain_exact(self):
        points = [(8, 20), (9, 20), (10, 20)] + [
            (y, 50) for y in (18, 24, 40, 58)]
        result = aggregate(self.w, points)
        self.assertEqual(result["reason"], "DEGRADED_CORE_SUPPORT")
        self.assertEqual(result["weight_mode"], "DEGRADED_CORE_SUPPORT")
        self.assertEqual(result["reference_row_y"], 24)
        self.assertEqual(result["center_x"], 50.0)
        self.assertEqual(result["heading_contribution"], 0)

    def test_span_or_endpoint_support_missing_does_not_create_heading(self):
        for points in ([(y, 40 + y - 40) for y in range(36, 46)],
                       [(y, 40 + y - 40) for y in (30, 32, 34, 36, 38, 40, 42, 44)]):
            result = aggregate(self.w, points)
            self.assertTrue(result["valid"])
            self.assertFalse(result["preview_valid"])
            self.assertEqual(result["steering_error"], result["raw_error"])

    def test_disconnected_far_geometry_does_not_change_direction(self):
        points = [(y, 60) for y in range(18, 30)]
        points += [(y, 40) for y in range(40, 59)]
        result = aggregate(self.w, points)
        self.assertTrue(result["preview_valid"])
        self.assertEqual(result["heading_contribution"], 0)
        self.assertGreaterEqual(result["preview_far_y"], 40)

    def test_contour_recovery_is_gated_before_selection_and_bilateral_cache(self):
        frame = _paired_contour_frame()
        prior = _clear_prior()
        recovery = ContourRouteRecovery(config)
        candidate = recovery.detect(frame, prior)
        self.assertIsNot(candidate, prior)
        self.assertTrue(candidate["valid"])
        self.assertIsNone(candidate["selected_rows"])

        selection = {
            "valid": True, "quality": candidate["quality"],
            "lateral_error": candidate["lateral_error"],
            "source": "road", "result": candidate,
            "comparison": {},
        }
        validator = fast_boundary.FastBoundaryEvidence(config)
        self.assertIsNone(main._approved_bilateral_cache_candidate(
            selection, candidate, {"valid": False}, frame))
        validator.gate_fast_path_candidate(
            candidate, frame["luma"], frame["edge_mask"],
            frame["road_mask"], frame["known_region"], config)
        self.assertTrue(candidate["valid"], candidate["reason"])
        selection, reference, boundary_reference = \
            main._validate_selected_fast_boundary(
                selection, candidate, validator)
        self.assertTrue(boundary_reference["valid"], boundary_reference)
        self.assertTrue(selection["valid"])
        self.assertEqual(reference["kind"], "reference_y_current_contour_bilateral")

        cache = fast_single_edge.FastSingleEdgeTracker(config)
        approved = main._approved_bilateral_cache_candidate(
            selection, candidate, boundary_reference, frame)
        self.assertIs(approved, candidate)
        self.assertGreater(cache.observe_bilateral(
            approved, frame["luma"], frame["edge_mask"],
            frame["road_mask"], frame["known_region"], True, True), 0)
        rows_before = cache.cache_rows
        self.assertGreater(rows_before, 0)

        boundary_fail_frame = _paired_contour_frame()
        boundary_fail_frame["edge_mask"] = bytearray(
            config.SURFACE_GRID_WIDTH * config.SURFACE_GRID_HEIGHT)
        boundary_fail_prior = _clear_prior("NO_PAIRED_EDGES")
        boundary_fail_original = dict(boundary_fail_prior)
        unverified = recovery.detect(
            boundary_fail_frame, boundary_fail_prior)
        self.assertIsNot(unverified, boundary_fail_prior)
        unverified, gate_reason, gate_error = main._gate_contour_candidate(
            unverified, boundary_fail_prior, validator,
            boundary_fail_frame, config)
        self.assertIs(unverified, boundary_fail_prior)
        self.assertTrue(gate_reason.startswith("BOUNDARY_"), gate_reason)
        self.assertIsNone(gate_error)
        self.assertEqual(boundary_fail_original, boundary_fail_prior)
        self.assertIsNone(main._approved_bilateral_cache_candidate(
            {"valid": False, "source": "NONE", "result": None},
            boundary_fail_prior, {"valid": False}, boundary_fail_frame))
        self.assertEqual(rows_before, cache.cache_rows)

        ambiguous = _clear_prior("AMBIGUOUS_ROAD")
        ambiguous["scene"] = "AMBIGUOUS_CANDIDATE"
        original = dict(ambiguous)
        self.assertIs(ambiguous, recovery.detect(frame, ambiguous))
        self.assertEqual(original, ambiguous)
        self.assertIsNone(main._approved_bilateral_cache_candidate(
            {"valid": False, "source": "NONE", "result": None},
            ambiguous, {"valid": False}, frame))
        self.assertEqual(rows_before, cache.cache_rows)

    def test_invalid_frame_does_not_draw_a_persisted_contour_trace(self):
        class _Image:
            def __init__(self):
                self.lines = []
                self.crosses = []

            def draw_line(self, line, color):
                self.lines.append((line, color))

            def draw_cross(self, point, color):
                self.crosses.append((point, color))

            def draw_string(self, point, value, **kwargs):
                pass

        recovery = ContourRouteRecovery(config)
        frame = _paired_contour_frame()
        result = recovery.detect(frame, _clear_prior())
        self.assertTrue(result["valid"])
        image = _Image()
        main._draw_gray_result(
            image, {"valid": False, "source": "NONE"}, {}, result,
            None, False, 0, 0, 0, 0.0, text_cache=(),
            contour_trace_info=recovery.trace_info, contour_used=False)
        self.assertEqual([], image.lines)
        self.assertEqual([], image.crosses)

    def test_two_disconnected_reference_chains_disable_preview(self):
        points = [(y, 40) for y in range(20, 38)]
        points += [(y, 50) for y in range(41, 59)]
        result = aggregate(self.w, points)
        self.assertFalse(result["preview_valid"])

    def test_conflicting_duplicate_rows_are_not_averaged_into_a_route(self):
        points = [(y, 40) for y in range(20, 59)] + [(40, 50)]
        result = aggregate(self.w, points)
        self.assertFalse(result["preview_valid"])
        result = aggregate(self.w, [(y, 40) for y in range(20, 59)] + [(40, 40)])
        self.assertTrue(result["preview_valid"])
        self.assertEqual(result["heading_contribution"], 0)

    def test_unsupported_rows_do_not_steer(self):
        points = [(y, 40) for y in range(20, 59)] + [(18, 60), (19, 60)]
        result = aggregate(self.w, points, [1] * 39 + [0, 0])
        self.assertEqual(result["heading_contribution"], 0)
        result = aggregate(self.w, points, valid=False)
        self.assertFalse(result["valid"])
        self.assertEqual(result["steering_error"], 0)
        self.assertIsNone(result["preview_far_x"])
        self.assertIsNone(result["preview_rejection_reason"])
        self.assertIsNone(result["reference_error"])

    def test_upper_only_path_cannot_replace_near_route_or_seed_cache(self):
        raw = bytearray(320 * 120)
        for y in range(120):
            grid_y = y // 2
            for x in range(320):
                grid_x = x // 4
                raw[y * 320 + x] = (220 if 16 <= grid_y < 36 and
                                    48 <= grid_x < 60 else 25)
        masks = fast_gray.FastGrayMasks(config).build(_BufferImage(raw))
        path = fast_path.FastPathDetector(config).detect(masks)
        self.assertFalse(path["valid"], path)
        self.assertTrue(main._single_edge_fallback_allowed(path))
        tracker = fast_single_edge.FastSingleEdgeTracker(config)
        self.assertEqual(tracker.detect(masks, False)["reason"], "NO_FRESH_KEEP")
        self.assertEqual(tracker.detect(masks, True)["reason"], "NO_WIDTH_CACHE")
        self.assertFalse(main._single_edge_fallback_allowed({
            "reason": "AMBIGUOUS_ROAD", "topology_evidence": {}}))
        self.assertFalse(main._single_edge_fallback_allowed({
            "reason": "INSUFFICIENT_FRESH_ROWS", "topology_evidence": {"branch": True}}))


if __name__ == "__main__":
    unittest.main(verbosity=2)
