"""320x120 crop, fixed-point row weights and speed-stage host regressions.

These tests validate production components with synthetic buffers. They do
not claim camera, OpenMV-board, H743, or vehicle acceptance.
"""

import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import fast_gray
import fast_path
import main


WIDTH = 320
HEIGHT = 120
GRID_WIDTH = 80
GRID_HEIGHT = 60


class _BufferImage:
    def __init__(self, raw, width=WIDTH, height=HEIGHT):
        self.raw = raw
        self._width = width
        self._height = height

    def width(self):
        return self._width

    def height(self):
        return self._height

    def bytearray(self):
        return self.raw


class _SensorStub:
    GRAYSCALE = 1
    QVGA = 2

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name == "get_id":
            return lambda: 0x1324
        if name in ("reset", "set_pixformat", "set_framesize",
                    "set_windowing", "set_hmirror", "set_vflip",
                    "set_auto_gain", "set_auto_exposure", "skip_frames"):
            return lambda *args, **kwargs: self.calls.append(
                (name, args, kwargs))
        raise AttributeError(name)


def _path_candidate(rows):
    packed = array("i")
    for row in rows:
        if len(row) == 2:
            y, center_x = row
            left_x, right_x = center_x - 10, center_x + 10
        elif len(row) == 3:
            y, left_x, right_x = row
        else:
            raise ValueError("path rows must contain (y, center) or (y, left, right)")
        packed.extend((y, left_x, right_x, 7))
    return {
        "valid": True,
        "quality": 90,
        "reason": "OK",
        "selected_row_ints": packed,
        "selected_row_count": len(rows),
        "selected_row_stride": 4,
        "boundary_support_rows": bytearray([1] * len(rows)),
        "boundary_support_count": len(rows),
    }


def _aggregate_road(rows, support=None):
    workspace = main._FastRowWeightWorkspace()
    candidate = _path_candidate(rows)
    if support is not None:
        candidate["boundary_support_rows"] = bytearray(support)
    info = workspace.aggregate(
        {"valid": True, "source": "road", "result": candidate},
        candidate, config)
    return workspace, info


class TurnResponseWeightTests(unittest.TestCase):
    def test_sensor_uses_qvga_center_crop_without_resize(self):
        sensor = _SensorStub()
        self.assertEqual(main._configure_gray_sensor(sensor, fast=True),
                         0x1324)
        calls = {name: (args, kwargs)
                 for name, args, kwargs in sensor.calls}
        self.assertEqual(calls["set_framesize"][0], (sensor.QVGA,))
        self.assertEqual(calls["set_windowing"][0], ((0, 60, 320, 120),))
        self.assertEqual(config.IMAGE_WIDTH, 320)
        self.assertEqual(config.IMAGE_HEIGHT, 120)
        self.assertEqual((config.SURFACE_GRID_WIDTH,
                          config.SURFACE_GRID_HEIGHT), (80, 60))
        self.assertEqual((config.SURFACE_PIXEL_SCALE_X,
                          config.SURFACE_PIXEL_SCALE_Y), (4, 2))
        self.assertEqual(main._surface_pixel((10, 10)), (42, 21))
        self.assertTrue(main._validate_fast_config())

    def test_fixed_point_weights_are_symmetric_peaked_and_bounded(self):
        weights = config.STEERING_ROW_WEIGHT_Q
        self.assertEqual(len(weights), GRID_HEIGHT)
        self.assertEqual(config.STEERING_ROW_WEIGHT_Q_SCALE, 1000)
        self.assertEqual(tuple(weights), tuple(reversed(weights)))
        self.assertTrue(all(150 <= weight <= 1000 for weight in weights))
        self.assertEqual(max(weights), 1000)
        self.assertEqual((weights[29], weights[30]), (1000, 1000))
        self.assertLess(weights[0], weights[10])
        self.assertLess(weights[10], weights[20])
        self.assertLess(weights[20], weights[29])

    def test_minpool_checks_each_of_the_eight_source_pixels(self):
        builder = fast_gray.FastGrayMasks(config)
        raw = bytearray([255]) * (WIDTH * HEIGHT)
        grid_x, grid_y = 17, 23
        for dy in range(2):
            for dx in range(4):
                raw[:] = bytes([255]) * len(raw)
                source_x = grid_x * 4 + dx
                source_y = grid_y * 2 + dy
                raw[source_y * WIDTH + source_x] = 13
                self.assertEqual(
                    builder._read_frame(_BufferImage(raw)), "buffer")
                self.assertEqual(builder.luma[grid_y * GRID_WIDTH + grid_x],
                                 13, (dx, dy))

    def test_all_accepted_rows_use_the_configured_parabolic_weights(self):
        rows = [(y, 28 + (y // 7)) for y in range(8, 59)]
        _workspace, info = _aggregate_road(rows)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["reason"], "OK")
        self.assertEqual(info["weight_mode"], "PARABOLIC_ROWS")
        q = config.STEERING_ROW_WEIGHT_Q
        expected_x = sum(center * q[y] for y, center in rows) / float(
            sum(q[y] for y, _center in rows))
        self.assertAlmostEqual(info["center_x"], expected_x, places=6)
        self.assertEqual(info["supported_rows"], len(rows))
        self.assertGreater(info["near_weight"], 0.0)
        self.assertGreater(info["far_weight"], 0.0)
        self.assertAlmostEqual(info["near_weight"] + info["far_weight"],
                               1.0)

    def test_typed_weight_kernel_matches_python_sum_after_row_dedup(self):
        # The repeated y=20 row conflicts and must not add steering weight.
        # The equally near y=39/y=41 reference rows also retain their existing
        # conflict behavior while the numeric kernel computes the main center.
        rows = [(20, 35), (20, 60), (30, 38), (36, 39),
                (39, 40), (41, 45)]
        workspace, info = _aggregate_road(rows)
        self.assertTrue(info["valid"], info)
        self.assertTrue(workspace._weight_q_kernel_ready)
        self.assertEqual(info["supported_rows"], 5)
        self.assertIsNone(info["reference_row_y"])
        self.assertIsNone(info["reference_error"])

        q = config.STEERING_ROW_WEIGHT_Q
        accepted = [(y, center) for index, (y, center) in enumerate(rows)
                    if y not in [prior_y for prior_y, _ in rows[:index]]]
        expected_weighted_x2 = sum(2 * center * q[y]
                                   for y, center in accepted)
        expected_total_weight = sum(q[y] for y, _center in accepted)
        expected_core_weight = sum(q[y] for y, _center in accepted
                                   if config.STEERING_CORE_RAW_Y_MIN //
                                   config.SURFACE_PIXEL_SCALE_Y <= y <=
                                   config.STEERING_CORE_RAW_Y_MAX //
                                   config.SURFACE_PIXEL_SCALE_Y)
        self.assertEqual(tuple(workspace._weight_sums),
                         (expected_weighted_x2, expected_total_weight,
                          expected_core_weight))
        expected_center_x = expected_weighted_x2 / float(
            2 * expected_total_weight)
        self.assertAlmostEqual(info["center_x"], expected_center_x, places=6)
        self.assertEqual(info["raw_error"],
                         main._normalize_fast_reference(
                             expected_center_x, GRID_WIDTH))

    def test_typed_row_kernel_matches_python_fallback(self):
        duplicate_rows = [
            (20, 35), (20, 60), (30, 38), (36, 39), (39, 40), (41, 45),
        ]
        duplicate_candidate = _path_candidate(duplicate_rows)

        bounded_rows = [
            (-1, 30), (8, -4), (20, 30), (60, 32), (33, 90),
        ]
        bounded_candidate = _path_candidate(bounded_rows)
        bounded_candidate["boundary_support_rows"] = bytearray(
            [1, 1, 1, 1, 1])
        bounded_candidate["selected_row_ints"][4 * 4 + 3] = 3

        single_rows = array("i")
        for y in range(8, 59, 3):
            single_rows.extend((y, 2 * (30 + y // 10), 30, 7, 1))
        single_result = {
            "selected_row_ints": single_rows,
            "selected_row_count": len(single_rows) // 5,
            "selected_row_stride": 5,
        }

        cases = [
            ({"valid": True, "source": "road",
              "result": duplicate_candidate}, duplicate_candidate),
            ({"valid": True, "source": "road",
              "result": bounded_candidate}, bounded_candidate),
            ({"valid": True, "source": "single_left",
              "result": single_result}, {}),
        ]

        def aggregate(selection, road_result, native_enabled):
            workspace = main._FastRowWeightWorkspace()
            with patch.object(main, "_NATIVE_ACCELERATION_ENABLED",
                              native_enabled):
                info = workspace.aggregate(selection, road_result, config)
            state = (
                tuple(workspace._center_x2), tuple(workspace._row_y),
                tuple(workspace._preview_valid_rows),
                tuple(workspace._preview_center_x2),
                tuple(workspace._preview_touched_rows),
                workspace._preview_touched_count,
                tuple(workspace._weight_sums),
            )
            return info, state

        for selection, road_result in cases:
            with self.subTest(source=selection["source"]):
                self.assertEqual(
                    aggregate(selection, road_result, True),
                    aggregate(selection, road_result, False))

    def test_bilateral_minimum_matches_existing_four_rows_eight_span_gate(self):
        # The production FastPathDetector accepts four observed rows spanning
        # eight grid rows. Aggregation must not add a stricter global floor.
        rows = [(20, 40), (22, 40), (26, 40), (28, 40)]
        _workspace, accepted = _aggregate_road(rows)
        self.assertTrue(accepted["valid"], accepted)
        self.assertEqual(accepted["supported_rows"], 4)
        self.assertEqual(max(y for y, _ in rows) -
                         min(y for y, _ in rows), 8)

        _workspace, rejected = _aggregate_road(rows, [0, 0, 0, 0])
        self.assertFalse(rejected["valid"])
        self.assertEqual(rejected["reason"], "NO_SUPPORTED_ROWS")

    def test_incomplete_core_uses_nearest_measured_row_and_marks_degraded(self):
        # A remote singleton at y=58 must not replace the selected chain's
        # closest core reference at y=15 when the core support gate is absent.
        rows = [(y, 31) for y in range(10, 16)] + [(58, 70)]
        _workspace, info = _aggregate_road(rows)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["reason"], "DEGRADED_CORE_SUPPORT")
        self.assertEqual(info["weight_mode"], "DEGRADED_CORE_SUPPORT")
        self.assertEqual(info["reference_row_y"], 15)
        self.assertEqual(info["center_x"], 31.0)
        self.assertEqual(info["raw_error"],
                         main._normalize_fast_reference(31.0, GRID_WIDTH))

    def test_core_support_requires_three_rows_and_two_grid_span(self):
        rows = [(y, 40) for y in (20, 21, 22, 28, 35, 40, 45)]
        support = [1] * len(rows)
        support[2] = 0
        _workspace, info = _aggregate_road(rows, support)
        # Four accepted core rows still cover two grid rows and can weight.
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["near_count"], 4)
        self.assertEqual(info["weight_mode"], "PARABOLIC_ROWS")

        close_core = [(8, 40), (10, 40), (20, 40), (21, 40),
                      (21, 40), (50, 40), (58, 40)]
        _workspace, degraded = _aggregate_road(close_core)
        self.assertTrue(degraded["valid"], degraded)
        self.assertEqual(degraded["reason"], "DEGRADED_CORE_SUPPORT")

    def test_single_edge_routes_use_same_accepted_row_weights(self):
        expected = []
        rows = [(y, 2 * (30 + y // 10), 30, 7, 1)
                for y in range(8, 59)]
        packed = array("i")
        for row in rows:
            packed.extend(row)
        for source in ("single_left", "single_right"):
            tracker_result = {
                "selected_row_ints": packed,
                "selected_row_count": len(rows),
                "selected_row_stride": 5,
            }
            workspace = main._FastRowWeightWorkspace()
            info = workspace.aggregate(
                {"valid": True, "source": source,
                 "result": tracker_result}, {}, config)
            self.assertTrue(info["valid"], info)
            expected.append(info["center_x"])
        self.assertEqual(expected[0], expected[1])

    def test_profile_constants_keep_geometry_only_preview_at_old_gain(self):
        profile_builds = dict(config.STEERING_CONTROL_PROFILE_BUILD_IDS)
        self.assertEqual(config.BUILD_ID,
                         "OPENMV_ACKERMANN320_COORDINATED_20261022")
        self.assertEqual(config.PREVIEW_MAX_GAIN, 0.45)
        self.assertEqual(config.PREVIEW_APPLIED_LIMIT, 120)
        for profile, build_id in profile_builds.items():
            with self.subTest(profile=profile), \
                    patch.object(config, "STEERING_CONTROL_PROFILE", profile), \
                    patch.object(config, "BUILD_ID", build_id):
                self.assertTrue(main._validate_fast_config())

    def test_ackermann_package_profiles_keep_pd_preview_semantics(self):
        for profile in ("geometry_perf_only", "coordinated"):
            with self.subTest(profile=profile):
                controller = main._FastPositionPD(config)
                info = controller.update(
                    100, 33, True, 120, "road", "OBSERVED_BOTH", "road",
                    28, profile, lambda newer, older: newer - older)
                self.assertEqual(info["profile_code"], 2)
                self.assertTrue(info["pd_enabled"])
                self.assertTrue(info["preview_enabled"])


class SpeedStageTests(unittest.TestCase):
    def setUp(self):
        self.stage = main._FastSpeedStage(config)
        self.ticks_diff = lambda newer, older: newer - older

    def _update(self, tick, valid=True, position=0, candidate=0,
                candidate_valid=True, preview_state=0, frame_token=None):
        return self.stage.update(
            tick, valid, position, candidate, candidate_valid,
            preview_state, self.ticks_diff, frame_token)

    def test_candidate_latch_ignores_position_sign_and_requires_frames_and_time(self):
        self.assertEqual(self._update(0, position=-20, candidate=90), (1, 0))
        self.assertEqual(self._update(30, position=-20, candidate=90), (1, 0))
        self.assertEqual(self._update(59, position=-20, candidate=90), (1, 0))
        self.assertEqual(self._update(60, position=-20, candidate=90),
                         (1, 1000))
        self.assertTrue(self.stage.active)

    def test_turn_exit_release_invalid_and_stop_reset_semantics(self):
        for tick in (0, 30, 60):
            state = self._update(tick, position=10, candidate=-100,
                                 preview_state=2)
        self.assertEqual(state, (2, 1000))
        self.assertTrue(self.stage.turn_seen)
        self.assertEqual(self._update(70, valid=False), (0, 0))
        self.assertTrue(self.stage.active)

        self.assertEqual(self._update(100, position=20, candidate=10,
                                      preview_state=0), (2, 1000))
        self.assertEqual(self._update(200, position=20, candidate=10),
                         (2, 1000))
        self.assertEqual(self._update(300, position=20, candidate=10),
                         (2, 1000))
        self.assertEqual(self._update(400, position=20, candidate=10),
                         (3, 0))
        self.assertFalse(self.stage.active)
        self.assertEqual(self._update(500, position=0, candidate=0), (3, 0))
        self.assertEqual(self._update(1200, position=0, candidate=0), (0, 0))

        for tick in (1300, 1330, 1360):
            self._update(tick, position=0, candidate=100)
        self.assertTrue(self.stage.active)
        self.stage.lifecycle_event("KEEP")
        self.assertTrue(self.stage.active)
        self.stage.lifecycle_event("STOP")
        self.assertFalse(self.stage.active)
        self.assertEqual(self.stage.update(
            1400, True, 0, 0, True, 0, self.ticks_diff), (0, 0))

    def test_duplicate_frames_and_large_gaps_do_not_confirm_candidates(self):
        self.assertEqual(self._update(0, candidate=90, frame_token=1), (1, 0))
        self.assertEqual(self._update(20, candidate=90, frame_token=1), (1, 0))
        self.assertEqual(self.stage.pending_count, 1)
        self.assertEqual(self._update(30, candidate=90, frame_token=2), (1, 0))
        self.assertEqual(self._update(150, candidate=90, frame_token=3), (1, 0))
        self.assertEqual(self.stage.pending_count, 1)
        self.assertEqual(self._update(180, candidate=90, frame_token=4), (1, 0))
        self.assertEqual(self._update(210, candidate=90, frame_token=5),
                         (1, 1000))

    def test_duplicate_timestamp_does_not_hide_gap_before_next_fresh_frame(self):
        self.assertEqual(self._update(0, candidate=90, frame_token=10),
                         (1, 0))
        self.assertEqual(self._update(130, candidate=90, frame_token=10),
                         (1, 0))
        self.assertEqual(self.stage.pending_count, 1)
        self.assertEqual(self._update(131, candidate=90, frame_token=11),
                         (1, 0))
        self.assertEqual(self.stage.pending_count, 1)
        self.assertEqual(self._update(161, candidate=90, frame_token=12),
                         (1, 0))
        self.assertEqual(self._update(191, candidate=90, frame_token=13),
                         (1, 1000))

    def test_duplicate_frames_do_not_satisfy_release_duration(self):
        for tick, token in ((0, 1), (30, 2), (60, 3)):
            state = self._update(tick, candidate=100, preview_state=2,
                                 frame_token=token)
        self.assertEqual(state, (2, 1000))
        self.assertEqual(self._update(100, position=0, candidate=0,
                                      frame_token=4), (2, 1000))
        self.assertEqual(self._update(250, position=0, candidate=0,
                                      frame_token=4), (2, 1000))
        self.assertEqual(self._update(360, position=0, candidate=0,
                                      frame_token=5), (2, 1000))
        self.assertEqual(self._update(500, position=0, candidate=0,
                                      frame_token=5), (2, 1000))
        self.assertEqual(self._update(520, position=0, candidate=0,
                                      frame_token=6), (2, 1000))
        for tick, token in ((600, 7), (680, 8), (760, 9)):
            self.assertEqual(self._update(
                tick, position=0, candidate=0, frame_token=token), (2, 1000))
        self.assertEqual(self._update(820, position=0, candidate=0,
                                      frame_token=10), (3, 0))

    def test_tick_wrap_keeps_candidate_confirmation_interval_correct(self):
        period = 1 << 30
        diff = lambda newer, older: ((newer - older + period // 2) % period -
                                     period // 2)
        stage = main._FastSpeedStage(config)
        for tick in (period - 40, period - 10, 20):
            state = stage.update(tick, True, -20, 90, True, 0, diff)
        self.assertEqual(state, (1, 1000))

    def test_straight_route_keeps_zero_preview_candidate_valid_for_release(self):
        rows = [(y, 40) for y in range(8, 59)]
        workspace = main._FastRowWeightWorkspace()
        candidate = _path_candidate(rows)
        info = workspace.aggregate(
            {"valid": True, "source": "road", "result": candidate},
            candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertTrue(info["preview_valid"])
        self.assertEqual(info["preview_candidate"], 0)

        stage = main._FastSpeedStage(config)
        for tick in (0, 30, 60):
            stage.update(tick, True, 120, 100, True, 2,
                         self.ticks_diff, tick // 30)
        self.assertTrue(stage.active)
        self.assertEqual(stage.update(
            100, True, 0, info["preview_candidate"], info["valid"], 0,
            self.ticks_diff, 3), (2, 1000))
        self.assertEqual(stage.update(
            200, True, 0, info["preview_candidate"], info["valid"], 0,
            self.ticks_diff, 4), (2, 1000))
        self.assertEqual(stage.update(
            300, True, 0, info["preview_candidate"], info["valid"], 0,
            self.ticks_diff, 5), (2, 1000))
        self.assertEqual(stage.update(
            400, True, 0, info["preview_candidate"], info["valid"], 0,
            self.ticks_diff, 6), (3, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
