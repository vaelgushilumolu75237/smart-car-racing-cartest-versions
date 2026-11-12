"""Host checks for the standalone four-direction contour port."""

import array
from pathlib import Path
import sys
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import contour_trace as trace_module  # noqa: E402
from contour_trace import (
    ContourPostWorkspace,
    ContourTraceWorkspace,
    SEED_AMBIGUOUS,
    TERM_BAD_SEED,
    TERM_CAPACITY,
    TERM_COLLISION,
    TERM_DIRECTION_LIMIT,
    TERM_EDGE,
    TERM_ITERATION_BUDGET,
)


WIDTH = 80
HEIGHT = 60


def empty_mask(width=WIDTH, height=HEIGHT):
    return {
        "width": width,
        "height": height,
        "road_mask": bytearray(width * height),
        "known_region": bytearray([1]) * (width * height),
    }


def paint_corridor(mask, center_by_row=None, half_width=15):
    width = mask["width"]
    height = mask["height"]
    road = mask["road_mask"]
    for y in range(height - 1):
        center = 39.5 if center_by_row is None else center_by_row(y)
        left = int(center - half_width + 0.5)
        right = int(center + half_width - 0.5)
        for x in range(max(0, left), min(width, right + 1)):
            road[y * width + x] = 255
    return mask


def points(path, count):
    return [(path[index * 2], path[index * 2 + 1])
            for index in range(count)]


def _run_trace_branch(use_viper, callback):
    saved_flag = trace_module._IS_MICROPYTHON
    pointer_names = ("ptr8", "ptr16", "ptr32")
    saved_pointers = {name: getattr(trace_module, name, None)
                      for name in pointer_names}
    existed = {name: hasattr(trace_module, name) for name in pointer_names}
    if use_viper:
        for name in pointer_names:
            setattr(trace_module, name, lambda value: value)
    trace_module._IS_MICROPYTHON = use_viper
    try:
        return callback()
    finally:
        trace_module._IS_MICROPYTHON = saved_flag
        for name in pointer_names:
            if existed[name]:
                setattr(trace_module, name, saved_pointers[name])
            elif hasattr(trace_module, name):
                delattr(trace_module, name)


def _trace_snapshot(workspace):
    event_count = workspace.event_count
    return {
        "left_points": tuple(workspace.left_points[:workspace.left_count * 2]),
        "right_points": tuple(workspace.right_points[:workspace.right_count * 2]),
        "left_support": tuple(workspace.left_dark_support[:workspace.left_count]),
        "right_support": tuple(workspace.right_dark_support[:workspace.right_count]),
        "left_row_support": tuple(workspace.left_row_support),
        "right_row_support": tuple(workspace.right_row_support),
        "left_count": workspace.left_count,
        "right_count": workspace.right_count,
        "left_lost": workspace.left_lost,
        "right_lost": workspace.right_lost,
        "left_termination": workspace.left_termination,
        "right_termination": workspace.right_termination,
        "seed_status": workspace.seed_status,
        "iterations": workspace.iterations,
        "event_count": event_count,
        "events": (tuple(workspace.event_side[:event_count]),
                   tuple(workspace.event_points[:event_count * 2]),
                   tuple(workspace.event_directions[:event_count]),
                   tuple(workspace.event_errors[:event_count]),
                   tuple(workspace.event_lost[:event_count]),
                   tuple(workspace.event_actions[:event_count])),
    }


class ContourTraceTests(unittest.TestCase):
    def test_auto_seed_and_straight_trace_have_real_support(self):
        mask = paint_corridor(empty_mask())
        workspace = ContourTraceWorkspace()
        self.assertTrue(workspace.find_seeds(mask))
        self.assertEqual((58, 25),
                         (workspace.left_seed_y, workspace.left_seed_x))
        self.assertEqual((58, 54),
                         (workspace.right_seed_y, workspace.right_seed_x))

        result = workspace.trace(mask)
        self.assertEqual(59, result.left_count)
        self.assertEqual(59, result.right_count)
        self.assertEqual(TERM_EDGE, result.left_termination)
        self.assertEqual(TERM_EDGE, result.right_termination)
        self.assertEqual((58, 25), points(result.left_points, 1)[0])
        self.assertEqual((0, 25), points(result.left_points,
                                         result.left_count)[-1])
        self.assertEqual((0, 54), points(result.right_points,
                                         result.right_count)[-1])
        self.assertTrue(all(result.left_dark_support[:result.left_count]))
        self.assertTrue(all(result.right_dark_support[:result.right_count]))
        self.assertEqual(1, result.left_row_support[20])
        self.assertEqual(1, result.right_row_support[20])

        bottom = paint_corridor(empty_mask())
        for x in range(25, 55):
            bottom["road_mask"][59 * WIDTH + x] = 255
        bottom_trace = ContourTraceWorkspace().trace(bottom)
        self.assertEqual((59, 25), points(bottom_trace.left_points, 1)[0])
        self.assertGreater(bottom_trace.left_count, 2)
        self.assertEqual(TERM_EDGE, bottom_trace.left_termination)

    def test_bends_and_s_curve_follow_geometry(self):
        def left_curve(y):
            if y < 24:
                return 32.0
            if y < 40:
                return 32.0 + (y - 24) * 0.47
            return 39.5

        def s_curve(y):
            if y < 18:
                return 36.0
            if y < 31:
                return 36.0 + (31 - y) * 0.45
            if y < 44:
                return 41.85 - (y - 31) * 0.45
            return 36.0

        for shape in (left_curve, s_curve):
            result = ContourTraceWorkspace().trace(
                paint_corridor(empty_mask(), shape))
            self.assertGreater(result.left_count, 20)
            self.assertGreater(result.right_count, 20)
            self.assertNotEqual(TERM_BAD_SEED, result.left_termination)
            self.assertNotEqual(TERM_BAD_SEED, result.right_termination)
            self.assertGreater(len(set(x for _y, x in
                                       points(result.left_points,
                                              result.left_count))), 2)

    def test_horizontal_mirror_swaps_borders(self):
        mask = paint_corridor(empty_mask(), lambda y: 38.0 +
                              (4.0 if 18 <= y <= 38 else 0.0))
        mirrored = empty_mask()
        for y in range(HEIGHT):
            for x in range(WIDTH):
                mirrored["road_mask"][y * WIDTH + (WIDTH - 1 - x)] = \
                    mask["road_mask"][y * WIDTH + x]
                mirrored["known_region"][y * WIDTH + (WIDTH - 1 - x)] = \
                    mask["known_region"][y * WIDTH + x]
        original = ContourTraceWorkspace().trace(mask)
        flipped = ContourTraceWorkspace().trace(mirrored)
        expected_left = [(y, WIDTH - 1 - x) for y, x in
                         points(original.right_points, original.right_count)]
        expected_right = [(y, WIDTH - 1 - x) for y, x in
                          points(original.left_points, original.left_count)]
        self.assertEqual(expected_left,
                         points(flipped.left_points, flipped.left_count))
        self.assertEqual(expected_right,
                         points(flipped.right_points, flipped.right_count))
        self.assertEqual(original.left_termination, flipped.right_termination)
        self.assertEqual(original.right_termination, flipped.left_termination)

    def test_closed_ring_terminates_when_borders_meet(self):
        mask = empty_mask()
        road = mask["road_mask"]
        for y in range(16, 60):
            for x in range(19, 61):
                if not (26 <= y <= 49 and 29 <= x <= 50):
                    road[y * WIDTH + x] = 255
        result = ContourTraceWorkspace(iteration_budget=1800).trace(mask)
        self.assertGreater(result.left_count, 10)
        self.assertGreater(result.right_count, 0)
        self.assertIn(TERM_COLLISION,
                      (result.left_termination, result.right_termination))

    def test_unknown_or_clipped_side_is_not_dark_support(self):
        mask = paint_corridor(empty_mask())
        for y in range(18, 26):
            index = y * WIDTH + 24
            mask["known_region"][index] = 0
            mask["road_mask"][index] = 125
        result = ContourTraceWorkspace().trace(mask)
        trace_points = points(result.left_points, result.left_count)
        support_by_y = {trace_points[i][0]: result.left_dark_support[i]
                        for i in range(result.left_count)}
        self.assertGreater(result.left_lost, 0)
        self.assertTrue(all(support_by_y[y] == 0 for y in range(18, 26)))
        self.assertTrue(all(result.right_dark_support[:result.right_count]))

        clipped = empty_mask()
        clipped["road_mask"][10 * WIDTH] = 255
        bad = ContourTraceWorkspace().trace(clipped, (10, 0), (10, 5))
        self.assertEqual(TERM_BAD_SEED, bad.left_termination)
        self.assertEqual(0, bad.left_dark_support[0])

    def test_lost_count_uses_pre_move_side_sample_only_after_move(self):
        for unknown_y, expected_increment_y, expected_zero_y in (
                (30, 29, 30), (29, 28, 29)):
            with self.subTest(unknown_y=unknown_y):
                mask = paint_corridor(empty_mask())
                side_index = unknown_y * WIDTH + 24
                mask["known_region"][side_index] = 0
                mask["road_mask"][side_index] = 125
                result = ContourTraceWorkspace().trace(mask)
                left_points = points(result.left_points, result.left_count)
                lost_by_y = {
                    left_points[index][0]: result.left_lost_history[index]
                    for index in range(result.left_count)
                }
                self.assertEqual(0, lost_by_y[expected_zero_y])
                self.assertEqual(1, lost_by_y[expected_increment_y])

    def test_seed_rejects_bad_dimensions_and_ambiguous_white_runs(self):
        workspace = ContourTraceWorkspace()
        with self.assertRaises(ValueError):
            workspace.trace({"width": 79, "height": 60,
                             "road_mask": bytearray(79 * 60),
                             "known_region": bytearray([1]) * (79 * 60)})

        mask = empty_mask()
        road = mask["road_mask"]
        for x in range(10, 20):
            road[58 * WIDTH + x] = 255
        for x in range(36, 52):
            road[58 * WIDTH + x] = 255
        self.assertFalse(workspace.find_seeds(mask))
        self.assertEqual(SEED_AMBIGUOUS, workspace.seed_status)

        unsupported = paint_corridor(empty_mask())
        for y in range(51, 60):
            unsupported["known_region"][y * WIDTH + 24] = 0
        self.assertFalse(workspace.find_seeds(unsupported))
        failed = workspace.trace(unsupported)
        self.assertEqual(TERM_BAD_SEED, failed.left_termination)

    def test_capacity_iteration_and_direction_limits_are_bounded(self):
        mask = paint_corridor(empty_mask())
        capacity = ContourTraceWorkspace(max_points=8).trace(mask)
        self.assertEqual(TERM_CAPACITY, capacity.left_termination)
        self.assertEqual(8, capacity.left_count)

        budget = ContourTraceWorkspace(iteration_budget=1).trace(mask)
        self.assertEqual(TERM_ITERATION_BUDGET, budget.left_termination)
        self.assertEqual(TERM_ITERATION_BUDGET, budget.right_termination)

        isolated = empty_mask()
        isolated["road_mask"][10 * WIDTH + 5] = 255
        isolated["road_mask"][10 * WIDTH + 14] = 255
        bounded = ContourTraceWorkspace().trace(
            isolated, (10, 5), (10, 14))
        self.assertEqual(TERM_DIRECTION_LIMIT, bounded.left_termination)
        self.assertEqual(TERM_DIRECTION_LIMIT, bounded.right_termination)
        self.assertEqual(10, bounded.left_final_error)
        self.assertEqual(10, bounded.right_final_error)

    def test_original_320x240_dimensions_straight_and_bend(self):
        width = 320
        height = 240

        def corridor(center_by_row):
            mask = empty_mask(width, height)
            road = mask["road_mask"]
            for y in range(height):
                center = center_by_row(y)
                left = int(center - 30 + 0.5)
                right = int(center + 30 - 0.5)
                for x in range(left, right + 1):
                    road[y * width + x] = 255
            return mask

        straight = ContourTraceWorkspace(
            width, height, max_points=300, iteration_budget=700).trace(
                corridor(lambda _y: 159.5), (239, 130), (239, 189))
        self.assertEqual(240, straight.left_count)
        self.assertEqual(240, straight.right_count)
        self.assertEqual((239, 130), points(straight.left_points, 1)[0])
        self.assertEqual((239, 189), points(straight.right_points, 1)[0])
        self.assertEqual((0, 130), points(straight.left_points,
                                           straight.left_count)[-1])

        bent = ContourTraceWorkspace(
            width, height, max_points=300, iteration_budget=700).trace(
                corridor(lambda y: 145.0 if y < 100 else
                         145.0 + min(30.0, (y - 100) * 0.4)),
                (239, 145), (239, 204))
        self.assertGreater(bent.left_count, 100)
        self.assertGreater(bent.right_count, 100)
        self.assertGreater(len(set(x for _y, x in points(
            bent.left_points, bent.left_count))), 3)
        self.assertNotEqual(TERM_BAD_SEED, bent.left_termination)
        self.assertNotEqual(TERM_BAD_SEED, bent.right_termination)


class ContourViperParityTests(unittest.TestCase):
    def _run_shape(self, use_viper, mask, **workspace_options):
        def run():
            workspace = ContourTraceWorkspace(
                record_events=True, **workspace_options)
            workspace.trace(mask)
            return _trace_snapshot(workspace)
        return _run_trace_branch(use_viper, run)

    def test_viper_helpers_match_reference_trace_and_reset_reuse(self):
        straight = paint_corridor(empty_mask())
        ring = empty_mask()
        for y in range(16, HEIGHT):
            for x in range(19, 61):
                if not (26 <= y <= 49 and 29 <= x <= 50):
                    ring["road_mask"][y * WIDTH + x] = 255
        no_road = empty_mask()
        ambiguous = empty_mask()
        for x in range(10, 20):
            ambiguous["road_mask"][58 * WIDTH + x] = 255
        for x in range(36, 52):
            ambiguous["road_mask"][58 * WIDTH + x] = 255

        cases = (
            ("straight", straight, {}),
            ("collision", ring, {"iteration_budget": 1800}),
            ("seed_failure", no_road, {}),
            ("ambiguous_seed", ambiguous, {}),
            ("capacity", straight, {"max_points": 8}),
        )
        for name, mask, options in cases:
            with self.subTest(case=name):
                expected = self._run_shape(False, mask, **options)
                actual = self._run_shape(True, mask, **options)
                self.assertEqual(expected, actual)
        self.assertIn(TERM_COLLISION,
                      (self._run_shape(True, ring, iteration_budget=1800)[
                          "left_termination"],
                       self._run_shape(True, ring, iteration_budget=1800)[
                          "right_termination"]))

        def reuse(use_viper):
            def run():
                workspace = ContourTraceWorkspace(record_events=True)
                workspace.trace(straight)
                workspace.trace(no_road)
                workspace.trace(straight)
                return _trace_snapshot(workspace)
            return _run_trace_branch(use_viper, run)
        self.assertEqual(reuse(False), reuse(True))

    def test_viper_state_collision_table_full_collision_and_clear_helpers(self):
        def exercise(use_viper):
            def run():
                workspace = ContourTraceWorkspace(
                    max_points=8, iteration_budget=8, state_table_size=16)
                mask = workspace._state_mask
                y, x, direction, error, side = 10, 12, 0, 0, 0
                flat = y * WIDTH + x
                code = ((error + 10) * 4 + direction) * 2 + side
                first_slot = (flat * 131 + code * 17) & mask
                workspace._state_position[first_slot] = 1
                workspace._state_code[first_slot] = 1
                self.assertEqual(0, workspace._state_seen(
                    y, x, direction, error, side))
                self.assertEqual(1, workspace._state_seen(
                    y, x, direction, error, side))

                for index in range(len(workspace._state_position)):
                    workspace._state_position[index] = index + 1
                    workspace._state_code[index] = 255
                self.assertEqual(-1, workspace._state_seen(
                    y, x, direction, error, side))

                workspace.right_points[0] = 20
                workspace.right_points[1] = 30
                workspace.right_count = 1
                self.assertTrue(workspace._collides(0, 20, 30))
                self.assertFalse(workspace._collides(0, 20, 31))

                workspace.left_row_support[:] = array.array(
                    "H", [1] * HEIGHT)
                workspace.right_row_support[:] = array.array(
                    "H", [2] * HEIGHT)
                workspace._clear()
                self.assertEqual((0,) * len(workspace._state_position),
                                 tuple(workspace._state_position))
                self.assertEqual(bytes(len(workspace._state_code)),
                                 bytes(workspace._state_code))
                self.assertEqual((0,) * HEIGHT,
                                 tuple(workspace.left_row_support))
                self.assertEqual((0,) * HEIGHT,
                                 tuple(workspace.right_row_support))
            return _run_trace_branch(use_viper, run)

        exercise(False)
        exercise(True)


class ContourGeometryTests(unittest.TestCase):
    def test_filter_preserves_endpoints_and_does_not_cross_unknown(self):
        known = bytearray([1]) * (12 * 12)
        path = array.array("H", [5, 1, 4, 2, 3, 3, 9, 9, 3, 7, 4, 8, 5, 9])
        known[9 * 12 + 9] = 0
        work = ContourPostWorkspace(12, 12, 16)
        filtered, count = work.filter_path(path, 7, known, radius=2)
        self.assertEqual(7, count)
        self.assertEqual((5, 1), (filtered[0], filtered[1]))
        self.assertEqual((5, 9), (filtered[12], filtered[13]))
        self.assertEqual((9, 9), (filtered[6], filtered[7]))
        self.assertEqual((3, 7), (filtered[8], filtered[9]))

    def test_arc_resampling_splits_unknown_segments(self):
        known = bytearray([1]) * (12 * 12)
        path = array.array("H", [2, 1, 2, 3, 2, 5, 8, 8, 8, 10])
        known[8 * 12 + 8] = 0
        work = ContourPostWorkspace(12, 12, 16)
        output, count, starts = work.resample_path(path, 5, known, 2.0)
        self.assertEqual(1, starts[0])
        self.assertEqual(1, starts[count - 2])
        self.assertEqual((2.0, 1.0), (output[0], output[1]))
        self.assertEqual((2.0, 5.0), (output[4], output[5]))
        self.assertEqual((8.0, 10.0), (output[(count - 1) * 2],
                                        output[(count - 1) * 2 + 1]))
        self.assertEqual(0, starts[1])

    def test_supported_normal_offset_and_three_point_corner(self):
        known = bytearray([1]) * (16 * 16)
        path = array.array("H", [12, 5, 11, 5, 10, 5, 9, 5, 8, 5])
        support = bytearray([1]) * 5
        work = ContourPostWorkspace(16, 16, 8)
        offset, valid, count = work.offset_inward(
            path, 5, support, known, "left", width=2.0)
        self.assertEqual(5, count)
        self.assertTrue(all(valid[:count]))
        self.assertAlmostEqual(7.0, offset[1])
        known[10 * 16 + 7] = 0
        work.offset_inward(path, 5, support, known, "left", width=2.0)
        self.assertEqual(0, work.offset_valid[2])

        turn = array.array("H", [5, 1, 4, 1, 3, 1, 3, 2, 3, 3])
        corners, corner_count = work.extract_corners(
            turn, 5, bytearray([1]) * (16 * 16), min_angle_deg=45)
        self.assertEqual(1, corner_count)
        self.assertEqual(2, corners[0])


if __name__ == "__main__":
    unittest.main()
