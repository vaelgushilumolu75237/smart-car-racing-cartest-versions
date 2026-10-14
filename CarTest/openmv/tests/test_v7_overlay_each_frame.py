"""Host coverage for current-frame V7 overlay and preview flushing."""

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
        self.us += 100
        return self.us


class V7OverlayEachFrameTests(unittest.TestCase):
    def test_each_frame_draws_and_flushes_while_details_refresh_every_100ms(self):
        rows = [
            {"y": 20.0, "left": 20.0, "right": 60.0, "center": 40.0,
             "boundary": {"left": (19.0, 220), "right": (61.0, 220)}},
            {"y": 21.0, "left": 22.0, "right": 62.0, "center": 42.0,
             "boundary": {"left": (21.0, 220), "right": (63.0, 220)}},
        ]
        road_result = {"selected_rows": rows}
        road_selection = {"source": "road", "quality": 80, "reason": "OK"}
        road_reference = {"x": 40.0, "y": 20.0, "kind": "ROAD"}
        clipped_rows = array("i", [
            20, 61, 41, 0, 0,
            21, 59, 39, 0, 0,
        ])
        clipped_result = {
            "route_mode": "clipped_midpoint",
            "selected_row_ints": clipped_rows,
            "selected_row_count": 2,
            "selected_row_stride": 5,
            "actual_boundary": "LEFT",
            "actual_boundary_x": 20.5,
            "clipped_endpoint_x": 79,
            "reason": "OK_CLIPPED_MIDPOINT_LEFT",
        }
        clipped_selection = {
            "source": "single_left", "quality": 80,
            "result": clipped_result,
        }
        clipped_reference = {"x": 30.0, "y": 20.0,
                             "kind": "CLIPPED_MIDPOINT"}
        invalid_selection = {
            "source": "NONE", "reason": "NO_VALID_ROAD",
            "single_edge_reason": "NO_FRESH_KEEP",
        }

        state = main._V7OverlayState(0)
        clock = _Clock()
        flushed = []
        images = [_Image(), _Image(), _Image(), _Image()]
        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"), \
                patch.object(main, "_build_gray_overlay_text_cache",
                             wraps=main._build_gray_overlay_text_cache) as text_builder:
            main._render_v7_preview_frame(
                images[0], road_selection, {}, road_result, road_reference,
                True, 0, 0, 3, 30.0, state, 0,
                lambda: flushed.append("0"))
            self.assertGreater(len(images[0].lines), 0)
            actual_left_start = main._surface_pixel((19.0, 20.0))
            self.assertEqual(images[0].lines[0][0][:2], actual_left_start)
            bilateral_midline = (
                main._surface_pixel((40.0, 20.0))[0],
                main._surface_pixel((40.0, 20.0))[1],
                main._surface_pixel((42.0, 21.0))[0],
                main._surface_pixel((42.0, 21.0))[1],
            )
            self.assertIn((bilateral_midline, 255), images[0].lines)

            with patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                              "clipped_midpoint"):
                main._render_v7_preview_frame(
                    images[1], clipped_selection, {}, {}, clipped_reference,
                    True, 1, 1, 4, 30.0, state, 50,
                    lambda: flushed.append("50"))
            self.assertGreater(len(images[1].lines), 0)
            clipped_midline = (
                main._surface_pixel((30.5, 20))[0],
                main._surface_pixel((30.5, 20))[1],
                main._surface_pixel((29.5, 21))[0],
                main._surface_pixel((29.5, 21))[1],
            )
            self.assertIn((clipped_midline, 255), images[1].lines)
            self.assertIn("NOW V CLIP/SINGLE_LEFT", images[1].strings[0][1])

            with patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                              "clipped_midpoint"):
                main._render_v7_preview_frame(
                    images[2], invalid_selection, {}, {}, None,
                    False, 0, 0, 5, 30.0, state, 99,
                    lambda: flushed.append("99"))
                main._render_v7_preview_frame(
                    images[3], invalid_selection, {}, {}, None,
                    False, 0, 0, 6, 30.0, state, 100,
                    lambda: flushed.append("100"))

        self.assertEqual(text_builder.call_count, 3)
        self.assertEqual(flushed, ["0", "50", "99", "100"])
        self.assertEqual(state.draw_us, 100)
        self.assertEqual(state.flush_us, 100)
        self.assertIn("NOW INVALID CLIP", images[2].strings[0][1])
        self.assertEqual(images[2].lines, [])
        self.assertEqual(images[2].crosses, [])
        self.assertIn("NO_FRESH_KEEP", " ".join(
            value for point, value in images[3].strings))

    def test_current_bilateral_diagnostic_reports_measured_edges_and_reference(self):
        selection = {"source": "road"}
        rows = [{
            "y": 23.0,
            "boundary": {"left": (18.5, 201), "right": (61.5, 220)},
        }]
        left, right, reference = main._format_selected_actual_diagnostic(
            selection, {"selected_rows": rows}, {"x": 40.0, "y": 23.0},
            True)
        self.assertEqual((left, right, reference),
                         ("18.5", "61.5", "40.0,23.0"))


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            V7OverlayEachFrameTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
