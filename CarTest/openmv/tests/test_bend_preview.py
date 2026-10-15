"""Current-route heading and nearby-reference regressions; host evidence only."""
import unittest
from test_turn_response_weights import (
    config, main, fast_gray, fast_path, _path_candidate, _BufferImage)
import fast_single_edge


def aggregate(workspace, points, support=None, valid=True):
    candidate = _path_candidate([(y, x - 10, x + 10) for y, x in points])
    candidate["boundary_support_rows"] = bytearray(
        support if support is not None else [1] * len(points))
    candidate["boundary_support_count"] = len(points)
    selection = {"valid": valid, "source": "road", "result": candidate}
    return workspace.aggregate(selection, candidate, config)


class BendPreviewTests(unittest.TestCase):
    def setUp(self):
        self.w = main._FastRowWeightWorkspace()

    def test_equal_position_but_opposite_heading_changes_control(self):
        right = [(y, 46 if y < 33 else 38 if y < 46 else 36)
                 for y in range(20, 59)]
        left = [(y, 80 - x) for y, x in right]
        a = dict(aggregate(self.w, right))
        b = dict(aggregate(self.w, left))
        self.assertEqual(a["raw_error"], b["raw_error"])
        self.assertGreater(a["steering_error"], 0)
        self.assertLess(b["steering_error"], 0)
        self.assertEqual(a["heading_contribution"], 400)
        self.assertEqual(b["heading_contribution"], -400)

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
        self.assertEqual(a["heading_contribution"], -b["heading_contribution"])

    def test_diagonal_straights_are_not_mistaken_for_curved_entries(self):
        for points in ([(y, 20 + y // 2) for y in range(20, 59)],
                       [(y, 60 - y // 2) for y in range(20, 59)]):
            result = aggregate(self.w, points)
            self.assertTrue(result["preview_valid"])
            self.assertNotEqual(result["heading_error"], 0)
            self.assertEqual(result["heading_contribution"], 0)
            self.assertEqual(result["steering_error"], result["raw_error"])

    def test_position_shares_remain_exact(self):
        points = [(10, 20)] + [(y, 50) for y in (18, 24, 40, 58)]
        result = aggregate(self.w, points)
        self.assertEqual((result["far_weight"], result["near_weight"]), (.2, .8))
        self.assertEqual(result["center_x"], 44.0)
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

    def test_upper_only_path_cannot_replace_near_route_or_seed_cache(self):
        raw = bytearray(160 * 120)
        for y in range(120):
            for x in range(160):
                raw[y * 160 + x] = 220 if 16 <= y < 36 and 48 <= x < 96 else 25
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
