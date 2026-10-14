"""Regression tests for V7 overlay timing and bounded preview drawing."""

import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config  # noqa: E402
import main  # noqa: E402


class _Image:
    def __init__(self):
        self.lines = []
        self.crosses = []
        self.strings = []

    def draw_line(self, line, color):
        self.lines.append((line, color))

    def draw_cross(self, point, color):
        self.crosses.append((point, color))

    def draw_string(self, point, value, **kwargs):
        self.strings.append((point, value))


class _Clock:
    def __init__(self):
        self.us = 0

    @staticmethod
    def ticks_diff(new, old):
        return new - old

    def ticks_us(self):
        return self.us


def _expected_samples(row_count):
    sample_count = min(row_count, 13)
    if sample_count <= 1:
        return [0] if row_count else []
    return [index * (row_count - 1) // (sample_count - 1)
            for index in range(sample_count)]


class V7OverlayAccountingTests(unittest.TestCase):
    def test_cache_refresh_cost_is_overlay_time_and_flush_is_separate(self):
        clock = _Clock()
        state = main._V7OverlayState(0)
        image = _Image()

        def build_text(*args):
            clock.us += 550
            return ("D100 route", "D100 edge", "D100 status")

        def flush_preview():
            clock.us += 930

        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(main, "_build_gray_overlay_text_cache",
                             side_effect=build_text):
            main._render_v7_preview_frame(
                image, {"source": "NONE", "reason": "INVALID"}, {}, {},
                None, False, 0, 0, 0, 0.0, state, 0, flush_preview)

        self.assertEqual(state.draw_us, 550)
        self.assertEqual(state.flush_us, 930)
        self.assertEqual(clock.us, 1480)

    def test_bilateral_centerline_is_limited_to_13_points_with_endpoints(self):
        rows = []
        for index in range(40):
            rows.append({
                "y": 20.0 + index,
                "left": 20.0 + index * 0.2,
                "right": 60.0 + index * 0.2,
                "center": 40.0 + index * 0.25,
                "boundary": {
                    "left": (19.0 + index * 0.2, 220),
                    "right": (61.0 + index * 0.2, 220),
                },
            })
        image = _Image()

        with patch.object(config, "OVERLAY_ENABLED", True):
            main._draw_gray_result(
                image, {"source": "road", "quality": 90}, {},
                {"selected_rows": rows},
                {"x": 40.0, "y": 20.0, "kind": "ROAD"},
                True, 0, 0, 0, 0.0, text_cache=())

        sample_indices = _expected_samples(len(rows))
        center_lines = [line for line, color in image.lines
                        if color == 255]
        self.assertEqual(len(center_lines), 12)
        first_point = main._surface_pixel((
            rows[sample_indices[0]]["center"], rows[sample_indices[0]]["y"]))
        second_point = main._surface_pixel((
            rows[sample_indices[1]]["center"], rows[sample_indices[1]]["y"]))
        penultimate_point = main._surface_pixel((
            rows[sample_indices[-2]]["center"],
            rows[sample_indices[-2]]["y"]))
        last_point = main._surface_pixel((
            rows[sample_indices[-1]]["center"],
            rows[sample_indices[-1]]["y"]))
        self.assertEqual(center_lines[0], (
            first_point[0], first_point[1], second_point[0], second_point[1]))
        self.assertEqual(center_lines[-1], (
            penultimate_point[0], penultimate_point[1],
            last_point[0], last_point[1]))

        boundary_lines = [line for line, color in image.lines
                          if color == 96]
        self.assertLessEqual(len(boundary_lines), 24)
        self.assertEqual(boundary_lines[0][:2],
                         main._road_boundary_point(rows[0], "left"))
        self.assertEqual(boundary_lines[-2][2:],
                         main._road_boundary_point(rows[-1], "left"))

    def test_clipped_edge_and_endpoint_markers_share_the_13_row_budget(self):
        packed_rows = array("i")
        for index in range(40):
            packed_rows.extend((20 + index, 80 + index, 20 + index, 0, 0))
        single_result = {
            "route_mode": "clipped_midpoint",
            "selected_row_ints": packed_rows,
            "selected_row_count": 40,
            "selected_row_stride": 5,
            "actual_boundary": "LEFT",
            "actual_boundary_x": 10.0,
            "clipped_endpoint_x": 79.0,
        }
        selection = {"source": "single_left", "quality": 90,
                     "result": single_result}
        reference = {"x": 40.0, "y": 20.0,
                     "kind": "CLIPPED_MIDPOINT"}
        image = _Image()

        with patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                             "clipped_midpoint"):
            main._draw_gray_result(
                image, selection, {}, {}, reference, True, 0, 0, 0, 0.0,
                text_cache=())

        sample_indices = _expected_samples(40)
        edge_crosses = [point for point, color in image.crosses
                        if color == 192]
        endpoint_crosses = [point for point, color in image.crosses
                            if color == 96]
        self.assertEqual(len(edge_crosses), 13)
        self.assertEqual(len(endpoint_crosses), 13)
        for cross_points, x_for_row in (
                (edge_crosses, lambda index: (20 + index) * 0.5),
                (endpoint_crosses, lambda index: 79.0)):
            expected = [main._surface_pixel((x_for_row(index), 20 + index))
                        for index in sample_indices]
            self.assertEqual(cross_points, expected)

        midpoint_lines = [line for line, color in image.lines
                          if color == 255]
        self.assertEqual(len(midpoint_lines), 12)
        self.assertEqual(midpoint_lines[0][:2],
                         main._surface_pixel((40.0, 20)))
        self.assertEqual(midpoint_lines[-1][2:],
                         main._surface_pixel((59.5, 59)))


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            V7OverlayAccountingTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
