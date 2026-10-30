"""Finite _main_fast runs use the wider ticks_ms deadline window."""

import contextlib
import io
import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


OPENMV_DIR = Path(__file__).resolve().parents[1]
TEST_DIR = OPENMV_DIR / "tests"
for _path in (OPENMV_DIR, TEST_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import main
import test_fast_main_ring_slope as _ring_test_helpers
from test_fast_main_ring_slope import (
    _BoundaryValidator, _CrossStraightElements,
    _FixedElementState, _GrayBuilder, _OpenBandPathDetector, _PathDetector,
    _Sensor, _SingleEdgeTracker,
)


class _StopAfterSecondFrame(BaseException):
    pass


class _WrappingTime:
    """Model OpenMV's 30-bit tick counters and signed half-period diff."""

    PERIOD = 1 << 30
    MASK = PERIOD - 1
    HALF_PERIOD = PERIOD >> 1

    def __init__(self, start_us=None, start_ms=None, step_us=100):
        self.us = 0
        self.step_us = step_us
        self.start_us = (self.PERIOD - 100_000_000
                         if start_us is None else start_us)
        self.start_ms = (self.PERIOD - 100_000
                         if start_ms is None else start_ms)
        self.us_reads = []
        self.ms_reads = []

    def ticks_us(self):
        self.us += self.step_us
        value = (self.start_us + self.us) & self.MASK
        self.us_reads.append(value)
        return value

    def ticks_ms(self):
        value = (self.start_ms + self.us // 1000) & self.MASK
        self.ms_reads.append(value)
        return value

    @classmethod
    def ticks_diff(cls, new, old):
        difference = (new - old) & cls.MASK
        if difference >= cls.HALF_PERIOD:
            difference -= cls.PERIOD
        return difference

    @classmethod
    def ticks_add(cls, value, delta):
        return (value + delta) & cls.MASK

    def advance_us(self, amount):
        self.us += amount

    def current_us(self):
        return (self.start_us + self.us) & self.MASK

    def current_ms(self):
        return (self.start_ms + self.us // 1000) & self.MASK

    def sleep_ms(self, milliseconds):
        self.advance_us(milliseconds * 1000)

    @staticmethod
    def clock():
        return _Clock()


class _Clock:
    @staticmethod
    def tick():
        return None

    @staticmethod
    def fps():
        return 30.0


class _DeadlineUART:
    def __init__(self, fake_time, advance_after_first=True):
        self.fake_time = fake_time
        self.advance_after_first = advance_after_first
        self.frames = []

    @staticmethod
    def any():
        return 0

    @staticmethod
    def readinto(_buffer, _count=None):
        return 0

    def write(self, frame):
        self.frames.append(bytes(frame))
        if len(self.frames) == 1 and self.advance_after_first:
            # Move beyond ticks_us' signed half-period while crossing ticks_ms' wrap.
            self.fake_time.advance_us(600_000_000)
        elif len(self.frames) == 2:
            raise _StopAfterSecondFrame()
        return len(frame)


class MainFastDurationDeadlineTests(unittest.TestCase):
    def test_contour_loader_accepts_only_the_two_compiler_backends(self):
        for backend in ("python-bounded", "micropython-native-viper"):
            with self.subTest(backend=backend):
                route = types.SimpleNamespace(
                    RUNTIME_API_VERSION=main.CONTOUR_RUNTIME_API_VERSION,
                    COMPILER_BACKEND=backend)
                trace = types.SimpleNamespace(
                    RUNTIME_API_VERSION=main.CONTOUR_RUNTIME_API_VERSION,
                    COMPILER_BACKEND=backend)
                with patch.object(main, "contour_route", route), \
                        patch.object(main, "contour_trace", trace):
                    modules = main._load_contour_runtime()
                self.assertEqual(modules, (
                    ("contour_route.py", route),
                    ("contour_trace.py", trace)))

    def test_contour_loader_rejects_unknown_backend_and_wrong_api(self):
        valid_api = main.CONTOUR_RUNTIME_API_VERSION
        route = types.SimpleNamespace(
            RUNTIME_API_VERSION=valid_api, COMPILER_BACKEND="unknown")
        trace = types.SimpleNamespace(
            RUNTIME_API_VERSION=valid_api,
            COMPILER_BACKEND="python-bounded")
        with patch.object(main, "contour_route", route), \
                patch.object(main, "contour_trace", trace):
            with self.assertRaisesRegex(RuntimeError, "backend is unknown"):
                main._load_contour_runtime()

        route.RUNTIME_API_VERSION = valid_api + 1
        route.COMPILER_BACKEND = "python-bounded"
        with patch.object(main, "contour_route", route), \
                patch.object(main, "contour_trace", trace):
            with self.assertRaisesRegex(RuntimeError, "API is"):
                main._load_contour_runtime()

    def test_ring_diagnostic_extension_is_conditional_and_reports_growth(self):
        runner = _ring_test_helpers.FastMainRingSlopeWireTests(
            "test_unconfirmed_ring_observation_keeps_the_normal_road_route")
        _frame, straight_log, _cache_commits = runner._run_profile_element_case(
            _CrossStraightElements, config.VEHICLE_PROFILE_A,
            _OpenBandPathDetector, clock_step_us=1)
        self.assertNotIn("V7RING ", straight_log)

        class _BlockedGrowingArc(_FixedElementState):
            def __init__(self, settings, _calibration=None,
                         require_calibration=True):
                super().__init__(settings, _calibration, require_calibration)
                self.result.update({
                    "side_name": "RIGHT",
                    "ring_arc_support_rows": 5,
                    "ring_arc_span_rows": 4,
                    "ring_near_anchor_rows": 3,
                    "ring_near_anchor_run": 2,
                    "ring_arc_growth_frames": 2,
                    "ring_arc_growth_x2": 6,
                    "entry_confirmation_path": "GROWING_ARC",
                    "ring_arc_rejection": "OPPOSITE_NEAR_SUPPORT",
                    "target_valid": False,
                    "growth_ready_but_target_rejected": True,
                    "growth_target_rejection": "TARGET_NOT_BUILT",
                })

        _frame, ring_log, _cache_commits = runner._run_profile_element_case(
            _BlockedGrowingArc, config.VEHICLE_PROFILE_B)
        line = next((value for value in ring_log.splitlines()
                     if value.startswith("V7RING ")), None)
        self.assertIsNotNone(line, ring_log)
        for field in ("st=ENTRY_CONFIRM", "sd=RIGHT", "arc=5/4",
                      "near=3/2", "gf=2 dx2=+6", "path=GROWING_ARC",
                      "tgt=0",
                      "rej=OPPOSITE_NEAR_SUPPORT/TARGET_NOT_BUILT"):
            self.assertIn(field, line)

    def test_ambiguous_pair_diagnostic_reads_only_the_public_16_int_record(self):
        class _AmbiguousRoadPath(_PathDetector):
            def __init__(self, settings):
                super().__init__(settings)
                self.result.update({
                    "valid": False, "quality": 0, "reason": "NO_PATH",
                    "scene": "ROAD",
                    "topology_evidence": {
                        "reason": "NO_PATH", "complete": True,
                        "ambiguous": False, "branch_evidence": False,
                        "island": False, "island_evidence": False,
                    },
                })

        pair_record = array(
            "i", [7, 26, 15, 9, -1, 0, 8, 12, 0, 0, -1, 0, 0, 0, 0, 0])
        runner = _ring_test_helpers.FastMainRingSlopeWireTests(
            "test_unconfirmed_ring_observation_keeps_the_normal_road_route")
        _frame, log, _cache_commits = runner._run_profile_element_case(
            _FixedElementState, config.VEHICLE_PROFILE_B,
            _AmbiguousRoadPath, single_edge_result={
                "valid": False, "reason": "AMBIGUOUS_BOTH_SIDES"},
            pair_diagnostic_record=pair_record)
        lines = [line for line in log.splitlines()
                 if line.startswith("V7PAIR ")]
        self.assertEqual(len(lines), 1, log)
        self.assertIn("rows=26", lines[0])
        self.assertIn("span=15", lines[0])
        self.assertIn("maxdx2=9", lines[0])
        self.assertIn("left=8", lines[0])
        self.assertIn("right=12", lines[0])
        self.assertIn("reason=7:CENTER_DELTA", lines[0])

    def _run(self, duration_s, fake_time):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        fake_sensor = _Sensor()
        fake_uart = _DeadlineUART(fake_time)
        output = io.StringIO()

        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", fake_sensor), \
                patch.object(main, "time", fake_time), \
                patch.object(main, "_service_reverse_control",
                             return_value=True), \
                patch.object(main.fast_gray, "FastGrayMasks", _GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             _PathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             _BoundaryValidator), \
                patch.object(main.fast_single_edge, "FastSingleEdgeTracker",
                             _SingleEdgeTracker), \
                patch.object(main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(
                    main, "_validate_selected_fast_boundary",
                    side_effect=lambda selection, _road, _validator: (
                        selection,
                        {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                        {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(
                    config, "TRACK_PACKAGE_BUILD_ID",
                    "CARTEST_H750_RING_ENTRY_V26R6_20261030"), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=duration_s)

        return fake_time, fake_uart, output.getvalue()

    def test_600_second_run_uses_ms_across_both_tick_wrap_conditions(self):
        fake_time = _WrappingTime()
        fake_time, fake_uart, output = self._run(600, fake_time)

        self.assertEqual(len(fake_uart.frames), 1)
        # Startup reads debug time first and captures the finite-run origin next.
        start_us = fake_time.us_reads[0]
        start_ms = fake_time.ms_reads[1]
        self.assertLess(
            fake_time.ticks_diff(fake_time.current_us(), start_us), 0,
            "600 seconds exceeds ticks_us' signed half-period")
        self.assertGreaterEqual(
            fake_time.ticks_diff(fake_time.current_ms(), start_ms), 600_000,
            "the run crossed the ticks_ms counter wrap and reached its deadline")
        self.assertIn("V7_FAST_RUN_DONE run=0 duration_s=600 frames=1", output)

    def test_zero_duration_does_not_capture_a_frame(self):
        fake_time, fake_uart, output = self._run(0, _WrappingTime())
        self.assertEqual(fake_uart.frames, [])
        self.assertIn("V7_FAST_RUN_DONE run=0 duration_s=0 frames=0", output)

    def test_none_keeps_running_without_a_finite_deadline(self):
        fake_time = _WrappingTime()
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        fake_sensor = _Sensor()
        fake_uart = _DeadlineUART(fake_time)
        output = io.StringIO()

        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", fake_sensor), \
                patch.object(main, "time", fake_time), \
                patch.object(main, "_service_reverse_control",
                             return_value=True), \
                patch.object(main.fast_gray, "FastGrayMasks", _GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             _PathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             _BoundaryValidator), \
                patch.object(main.fast_single_edge, "FastSingleEdgeTracker",
                             _SingleEdgeTracker), \
                patch.object(main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(
                    main, "_validate_selected_fast_boundary",
                    side_effect=lambda selection, _road, _validator: (
                        selection,
                        {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                        {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(
                    config, "TRACK_PACKAGE_BUILD_ID",
                    "CARTEST_H750_RING_ENTRY_V26R6_20261030"), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            with self.assertRaises(_StopAfterSecondFrame):
                main._main_fast(duration_s=None)

        self.assertEqual(len(fake_uart.frames), 2)
if __name__ == "__main__":
    unittest.main(verbosity=2)
