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
        self.images = []

    def draw_line(self, line, color):
        self.lines.append((line, color))

    def draw_cross(self, point, color):
        self.crosses.append((point, color))

    def draw_string(self, point, value, **kwargs):
        self.strings.append((point, value))

    def draw_image(self, source, x, y, **kwargs):
        self.images.append((source, x, y, kwargs))

    def clear(self):
        self.lines.clear()
        self.crosses.clear()
        self.strings.clear()


class _CountingImage(_Image):
    def __init__(self):
        super().__init__()
        self.draw_string_calls = 0

    def draw_string(self, point, value, **kwargs):
        self.draw_string_calls += 1
        super().draw_string(point, value, **kwargs)


class _NonMaterializingRows:
    def __init__(self, packed, count):
        self.packed = packed
        self.active_count = count

    def __len__(self):
        raise AssertionError("packed formatter must not size the row view")

    def __getitem__(self, index):
        raise AssertionError("packed formatter must not expose row records")

    def __iter__(self):
        raise AssertionError("packed formatter must not iterate the row view")


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
    def test_status_bitmap_redraws_only_on_change_and_blits_each_frame(self):
        status_image = _CountingImage()
        state = main._V7OverlayState(0, status_image=status_image)
        clock = _Clock()
        frames = [_Image() for _ in range(4)]
        road_selection = {"source": "road", "quality": 80,
                          "reason": "CURRENT_ROAD",
                          "track_profile_label": "A",
                          "track_ring_state": "OBSERVE",
                          "track_ring_reason": "RING_DISABLED_PROFILE_A"}
        single_selection = {
            "source": "single_right", "quality": 80,
            "reason": "CURRENT_SINGLE_RIGHT", "result": {},
            "track_profile_label": "B", "track_ring_state": "INSIDE",
            "track_ring_reason": "RING_INSIDE_EXIT_UNOBSERVED",
        }
        invalid_selection = {
            "source": "NONE", "reason": "CURRENT_INVALID",
            "track_profile_label": "C", "track_ring_state": "EXIT_CONFIRM",
            "track_ring_reason": "EXIT_EXPERIMENTAL_CONFIRM",
        }

        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"):
            main._render_v7_preview_frame(
                frames[0], road_selection, {}, {"selected_rows": ()}, None,
                True, 0, 0, 1, 30.0, state, 0, lambda: None)
            self.assertEqual(state.status_text, "TRK A OBSERVE:A_OFF")
            self.assertEqual(status_image.draw_string_calls, 1)

            main._render_v7_preview_frame(
                frames[1], road_selection, {}, {"selected_rows": ()}, None,
                True, 0, 0, 2, 30.0, state, 50, lambda: None)
            self.assertEqual(state.status_text, "TRK A OBSERVE:A_OFF")
            self.assertEqual(status_image.draw_string_calls, 1)

            with patch.object(config, "SINGLE_EDGE_ROUTE_MODE",
                              "clipped_midpoint"):
                main._render_v7_preview_frame(
                    frames[2], single_selection, {}, {}, None, True,
                    0, 0, 3, 30.0, state, 99, lambda: None)
                self.assertEqual(
                    state.status_text, "TRK B INSIDE:INSIDE")
                self.assertEqual(status_image.draw_string_calls, 2)

                main._render_v7_preview_frame(
                    frames[3], invalid_selection, {}, {}, None, False,
                    0, 0, 4, 30.0, state, 100, lambda: None)
                self.assertEqual(state.status_text, "TRK C EXIT_CO:EXIT_CO")
                self.assertEqual(status_image.draw_string_calls, 3)

        for frame in frames:
            self.assertEqual(len(frame.images), 1)
            self.assertIs(frame.images[0][0], status_image)
            self.assertEqual(frame.images[0][1:3], (0, 0))
            self.assertEqual(frame.images[0][3], {})
            self.assertFalse(any(point == (0, 0)
                                 for point, value in frame.strings))

    def test_status_bitmap_width_covers_profile_state_and_reason(self):
        current_statuses = (
            "TRK UNK IDLE:NOT_RUN",
            "TRK B INSIDE:INSIDE",
            "TRK C EXIT_REP:EXIT_REO",
        )
        longest = max(current_statuses, key=len)
        self.assertGreaterEqual(
            main._OVERLAY_STATUS_IMAGE_WIDTH, len(longest) * 8)

        for status in current_statuses:
            self.assertLessEqual(
                len(status) * 8, main._OVERLAY_STATUS_IMAGE_WIDTH)

    def test_status_bitmap_uses_optional_mask_and_draws_unmasked_text(self):
        status_image = _CountingImage()
        status_mask = _CountingImage()
        masked_state = main._V7OverlayState(
            0, status_image=status_image, status_mask=status_mask)
        masked_frame = _Image()
        clock = _Clock()
        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"):
            main._render_v7_preview_frame(
                masked_frame, {"source": "road"}, {},
                {"selected_rows": ()}, None, True, 0, 0, 1, 30.0,
                masked_state, 0, lambda: None)
        self.assertEqual(masked_frame.images[0][3], {"mask": status_mask})

        fallback_state = main._V7OverlayState(
            0, status_image=_CountingImage())
        fallback_frame = _Image()
        long_route_mode = "unrecognized_route_mode_which_exceeds_bitmap"
        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE", long_route_mode):
            main._render_v7_preview_frame(
                fallback_frame, {"source": "road"}, {},
                {"selected_rows": ()}, None, True, 0, 0, 1, 30.0,
                fallback_state, 0, lambda: None)
        self.assertTrue(fallback_state.status_bitmap_ready)
        self.assertEqual(fallback_state.status_text, "TRK UNK IDLE:NOT_RUN")
        self.assertIs(fallback_frame.images[0][0],
                      fallback_state.status_image)
        self.assertEqual(fallback_frame.images[0][3], {})

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
            self.assertEqual(images[1].strings[0][1],
                             "TRK UNK IDLE:NOT_RUN")

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

        self.assertEqual(text_builder.call_count, 2)
        self.assertEqual(flushed, ["0", "50", "99", "100"])
        self.assertEqual(state.draw_us, 100)
        self.assertEqual(state.flush_us, 100)
        self.assertEqual(images[2].strings[0][1],
                         "TRK UNK IDLE:NOT_RUN")
        self.assertEqual(images[2].lines, [])
        self.assertEqual(images[2].crosses, [])
        self.assertIn("NO_FRESH_KEEP", " ".join(
            value for point, value in images[3].strings))

    def test_cached_detail_bitmap_keeps_only_text_and_live_status_updates(self):
        detail_image = _Image()
        detail_mask = _Image()
        state = main._V7OverlayState(0, detail_image, detail_mask)
        clock = _Clock()
        frame0 = _Image()
        frame1 = _Image()
        frame2 = _Image()
        road_selection = {"source": "road", "quality": 80,
                          "reason": "CURRENT_ROAD"}
        invalid_selection = {"source": "NONE", "reason": "CURRENT_INVALID"}
        with patch.object(main, "time", clock), \
                patch.object(config, "OVERLAY_ENABLED", True), \
                patch.object(config, "OVERLAY_PERIOD_MS", 100), \
                patch.object(config, "SINGLE_EDGE_ROUTE_MODE", "cached_width"), \
                patch.object(main, "_build_gray_overlay_text_cache",
                             wraps=main._build_gray_overlay_text_cache) as build:
            main._render_v7_preview_frame(
                frame0, road_selection, {}, {"selected_rows": ()},
                {"x": 40.0, "y": 40.0}, True, 0, 0, 1, 30.0, state, 0,
                lambda: None)
            cached = state.text_cache
            main._render_v7_preview_frame(
                frame1, invalid_selection, {}, {}, None, False, 0, 0, 2,
                30.0, state, 50, lambda: None)
            self.assertEqual(state.text_cache, cached)
            main._render_v7_preview_frame(
                frame2, invalid_selection, {}, {}, None, False, 0, 0, 3,
                30.0, state, 100, lambda: None)

        self.assertEqual(build.call_count, 2)
        self.assertEqual(len(frame0.images), 1)
        self.assertIs(frame0.images[0][0], detail_image)
        self.assertEqual(frame0.images[0][1:3], (0, 10))
        self.assertIs(frame0.images[0][3]["mask"], detail_mask)
        self.assertEqual(len(frame1.strings), 1)
        self.assertEqual(frame1.strings[0][1], "TRK UNK IDLE:NOT_RUN")
        self.assertEqual(len(frame1.images), 1)
        self.assertEqual(detail_image.lines, [])
        self.assertEqual(detail_image.crosses, [])
        self.assertEqual(detail_mask.lines, [])
        self.assertEqual(detail_mask.crosses, [])

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

    def test_packed_current_row_view_matches_legacy_format_without_materializing(self):
        selection = {"source": "road"}
        packed = array("i", [
            20, 20, 60, 7,
            23, 19, 61, 7,
            24, 17, 62, 7,
        ])
        legacy_rows = [
            {"y": 20, "boundary": {"left": (19, 20),
                                    "right": (61, 20)}},
            {"y": 23, "boundary": {"left": (18, 23),
                                    "right": (62, 23)}},
            {"y": 24, "boundary": {"left": (16, 24),
                                    "right": (63, 24)}},
        ]
        reference = {"x": 40.04, "y": 23.009}
        expected = main._format_selected_actual_diagnostic(
            selection, {"selected_rows": legacy_rows}, reference, True)

        lazy_rows = _NonMaterializingRows(packed, len(legacy_rows))
        actual = main._format_selected_actual_diagnostic(
            selection,
            {"selected_rows": lazy_rows, "selected_row_ints": packed,
             "selected_row_count": len(legacy_rows),
             "selected_row_stride": 4},
            reference, True)
        self.assertEqual(actual, expected)
        self.assertEqual(actual, ("18.0", "62.0", "40.0,23.0"))

        outside_tolerance = {"x": 40.04, "y": 23.011}
        expected_miss = main._format_selected_actual_diagnostic(
            selection, {"selected_rows": legacy_rows}, outside_tolerance, True)
        actual_miss = main._format_selected_actual_diagnostic(
            selection,
            {"selected_rows": lazy_rows, "selected_row_ints": packed,
             "selected_row_count": len(legacy_rows),
             "selected_row_stride": 4},
            outside_tolerance, True)
        self.assertEqual(actual_miss, expected_miss)
        self.assertEqual(actual_miss, ("NONE", "NONE", "40.0,23.0"))

    def test_packed_only_legacy_edge_rounding_is_unchanged(self):
        selection = {"source": "road"}
        packed = array("i", [23, 19, 61, 7])
        actual = main._format_selected_actual_diagnostic(
            selection,
            {"selected_rows": None, "selected_row_ints": packed,
             "selected_row_count": 1, "selected_row_stride": 4},
            {"x": 40.0, "y": 23.0}, True)
        self.assertEqual(actual, ("18.5", "61.5", "40.0,23.0"))


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            V7OverlayEachFrameTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
