"""Host-stub checks for V7 main-loop freshness, reference and UART safety.

These tests exercise the current source with fake camera/time/UART modules.
They are not OpenMV board or camera acceptance evidence.
"""

import importlib.util
import re
import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


TEST_ROOT = Path(__file__).resolve().parents[1]
OPENMV_DIR = TEST_ROOT / "openmv"
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import config


def import_main():
    """Load OpenMV main.py while satisfying its board-only imports."""
    with patch.dict(sys.modules, {
            "pyb": types.ModuleType("pyb"),
            "sensor": types.ModuleType("sensor"),
    }):
        spec = importlib.util.spec_from_file_location(
            "sixth_openmv_fast_main_for_tests", OPENMV_DIR / "main.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def decode_packet(protocol, packet):
    packet = bytes(packet)
    if len(packet) != 12:
        raise AssertionError("not a protocol-v1 frame")
    if packet[:2] != b"\xA5\x5A":
        raise AssertionError("bad sync bytes")
    expected = protocol.crc16_ccitt_false(packet[2:10])
    actual = packet[10] | (packet[11] << 8)
    if actual != expected:
        raise AssertionError("bad packet CRC")
    raw = packet[8] | (packet[9] << 8)
    if raw & 0x8000:
        raw -= 0x10000
    return {
        "sequence": packet[4] | (packet[5] << 8),
        "valid": packet[6] & 1,
        "flags": packet[6],
        "confidence": packet[7],
        "error": raw,
        "crc": actual,
    }


class FakeTime:
    """Deterministic wrap-safe clock; tests may delay a packet build."""

    TICKS_PERIOD = 1 << 30
    TICKS_HALF_PERIOD = TICKS_PERIOD >> 1

    def __init__(self):
        self.us = 0
        self.fake_clock = self
        self.sleep_calls = []

    def ticks_us(self):
        value = self.us % self.TICKS_PERIOD
        self.us += 100
        return value

    def ticks_ms(self):
        return (self.us // 1000) % self.TICKS_PERIOD

    @classmethod
    def ticks_diff(cls, now, before):
        return ((now - before + cls.TICKS_HALF_PERIOD) %
                cls.TICKS_PERIOD) - cls.TICKS_HALF_PERIOD

    @classmethod
    def ticks_add(cls, tick, delta):
        return (tick + delta) % cls.TICKS_PERIOD

    def clock(self):
        return self

    def tick(self):
        self.us += 1000

    def sleep_ms(self, duration):
        self.sleep_calls.append(duration)
        self.us += int(duration) * 1000

    @staticmethod
    def fps():
        return 60.0


class FakeImage:
    def width(self):
        return config.IMAGE_WIDTH

    def height(self):
        return config.IMAGE_HEIGHT


class FakeSensor:
    GRAYSCALE = 1
    QQVGA = 2

    def __init__(self, frame_limit):
        self.frame_limit = frame_limit
        self.frames = 0
        self.settings = []
        self.flush_calls = 0
        self.events = []
        self.fake_time = None

    def reset(self):
        self.settings.append(("reset", (), {}))

    @staticmethod
    def get_id():
        return 0x1234

    def set_pixformat(self, *args, **kwargs):
        self.settings.append(("set_pixformat", args, kwargs))

    def set_framesize(self, *args, **kwargs):
        self.settings.append(("set_framesize", args, kwargs))

    def set_hmirror(self, *args, **kwargs):
        self.settings.append(("set_hmirror", args, kwargs))

    def set_vflip(self, *args, **kwargs):
        self.settings.append(("set_vflip", args, kwargs))

    def set_auto_gain(self, *args, **kwargs):
        self.settings.append(("set_auto_gain", args, kwargs))

    def set_auto_exposure(self, *args, **kwargs):
        self.settings.append(("set_auto_exposure", args, kwargs))

    def skip_frames(self, *args, **kwargs):
        self.settings.append(("skip_frames", args, kwargs))

    @staticmethod
    def get_exposure_us():
        return 10000

    @staticmethod
    def get_gain_db():
        return 12.0

    def snapshot(self):
        if self.frames >= self.frame_limit:
            raise StopAfterFrames()
        self.frames += 1
        return FakeImage()

    def flush(self):
        self.flush_calls += 1
        self.events.append("flush")
        if self.fake_time is not None:
            # Make the board preview publication cost observable in the
            # overlay stage instead of treating flush as a zero-cost stub.
            self.fake_time.us += 500


class DrawRecorder:
    def __init__(self):
        self.lines = []
        self.crosses = []
        self.strings = []

    def draw_line(self, line, **kwargs):
        self.lines.append((line, kwargs))

    def draw_cross(self, point, **kwargs):
        self.crosses.append((point, kwargs))

    def draw_string(self, point, text, **kwargs):
        self.strings.append((point, text, kwargs))


class StopAfterFrames(Exception):
    """Sentinel to stop an otherwise unbounded real-time loop after N frames."""


class FakeUart:
    short_write = False

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame) - 1 if FakeUart.short_write else len(frame)


class FakePyb:
    def __init__(self):
        self.uart = None

    def UART(self, *args, **kwargs):
        self.uart = FakeUart(*args, **kwargs)
        return self.uart


class FakeGC:
    def __init__(self):
        self.collect_calls = 0

    def collect(self):
        self.collect_calls += 1


class FakeGrayBuilder:
    instances = []
    fail_on_build = False
    failure_message = "injected mask failure"

    def __init__(self, settings):
        self.calls = 0
        self.instances.append(self)
        self.info = {
            "width": 80,
            "height": 60,
            "floor": 20,
            "valid": True,
            "reason": "OK",
            "trusted_block_count": 3,
            "threshold_min": 80,
            "threshold_max": 120,
            "luma": bytes([180]) * (80 * 60),
            "road_mask": bytes([1]) * (80 * 60),
            "edge_mask": bytes([0]) * (80 * 60),
            "known_region": bytes([1]) * (80 * 60),
        }

    def build(self, image):
        self.calls += 1
        if FakeGrayBuilder.fail_on_build:
            raise RuntimeError(FakeGrayBuilder.failure_message)
        return self.info


class FakePathDetector:
    instances = []
    fail_on_detect = False
    failure_message = "injected path failure"

    def __init__(self, settings):
        self.calls = 0
        self.instances.append(self)
        self.selected_row_ints = array("i", [0] * (60 * 4))

    def detect(self, mask_info):
        self.calls += 1
        if FakePathDetector.fail_on_detect:
            raise RuntimeError(FakePathDetector.failure_message)
        self.selected_row_ints[0:8] = array(
            "i", [40, 20, 59, 7, 35, 21, 58, 7])
        # Deliberately make the weighted/general path center differ from the
        # exact controller-row center used by _selected_reference().
        return {
            "valid": True,
            "quality": 92,
            "lateral_error": 333,
            "reason": "OK",
            "center_x": 55.0,
            "mean_center_x": 55.0,
            "reference_center_x": 39.5,
            "reference_y": 40.0,
            "reference_provenance": "OBSERVED",
            "selected_rows": ({"x": 55.0, "y": 35.0},
                              {"x": 54.0, "y": 45.0}),
            "selected_row_ints": self.selected_row_ints,
            "selected_row_count": 2,
            "selected_row_stride": 4,
            "active_row_count": 2,
        }


class FakeBoundaryValidator:
    instances = []
    fail_on_gate = False
    failure_message = "injected boundary failure"

    def __init__(self, settings):
        self.settings = settings
        self.gate_calls = 0
        self.reference_calls = 0
        self.instances.append(self)

    def gate_fast_path_candidate(self, candidate, luma, edge_mask,
                                 road_mask, known_region, settings):
        self.gate_calls += 1
        if FakeBoundaryValidator.fail_on_gate:
            raise RuntimeError(FakeBoundaryValidator.failure_message)
        if (candidate.get("selected_row_stride") != 4 or
                candidate.get("selected_row_count") != 2 or
                not isinstance(candidate.get("selected_row_ints"), array)):
            raise AssertionError("V7 boundary did not receive packed path rows")
        return candidate

    def validate_selected_reference(self, selection, reference,
                                    surface_result, road_result, settings):
        self.reference_calls += 1
        valid = (selection.get("source") == "road" and
                 road_result is selection.get("result") and
                 reference == {"x": 39.5, "y": 40.0,
                               "kind": "reference_y_observed"})
        return {"valid": valid,
                "reason": "OK" if valid else "BAD_REFERENCE",
                "support_count": 2 if valid else 0}


def make_runtime_stubs(main_module):
    gray = types.SimpleNamespace(
        RUNTIME_API_VERSION=7,
        COMPILER_BACKEND="cpython-test-shim",
        FastGrayMasks=FakeGrayBuilder,
    )
    path = types.SimpleNamespace(
        RUNTIME_API_VERSION=7,
        FastPathDetector=FakePathDetector,
    )
    topology = types.SimpleNamespace(
        RUNTIME_API_VERSION=7,
        COMPILER_BACKEND="cpython-test-shim",
    )
    boundary = types.SimpleNamespace(
        RUNTIME_API_VERSION=7,
        COMPILER_BACKEND="cpython-test-shim",
        FastBoundaryEvidence=FakeBoundaryValidator,
    )

    def accept_exact_reference(selection, *args):
        reference = main_module._selected_reference(selection)
        return selection, reference, {"valid": True, "reason": "OK"}

    return gray, path, topology, boundary, accept_exact_reference


class FastMainHostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = import_main()

    def test_fresh_filter_matches_legacy_ema_invalid_and_gap_resets(self):
        legacy = __import__("adaptive_detector").ObservationFilter(
            0.45, 200, 60)
        fast = self.main._FreshObservationFilter(0.45, 200, 60)
        observations = (
            (0, True, 60, 10),
            (40, True, 90, 30),
            (80, True, 90, -9),
            (100, False, 0, 1000),
            (130, True, 100, -11),
            (130, True, 59, 500),
            (160, True, 80, 40),
            (361, True, 80, -5),  # >200ms gap clears the old EMA.
            (360, True, 80, 10),  # backwards ticks also clears it.
        )
        for now, valid, confidence, raw in observations:
            with self.subTest(now=now, valid=valid, confidence=confidence):
                self.assertEqual(
                    fast.update(now, valid, confidence, raw,
                                lambda newer, older: newer - older),
                    legacy.update(now, valid, confidence, raw,
                                  lambda newer, older: newer - older))

    def test_selected_reference_is_exact_point_not_path_mean(self):
        result = FakePathDetector(None).detect(None)
        selection = {"valid": True, "source": "road", "result": result,
                     "comparison": {}}
        reference = self.main._selected_reference(selection)
        self.assertEqual(reference,
                         {"x": 39.5, "y": 40.0,
                          "kind": "reference_y_observed"})
        self.assertNotEqual(reference["x"], result["mean_center_x"])
        self.assertEqual(self.main._normalize_fast_reference(
            reference["x"], 80), 0)

    def test_perf_window_clear_valid_full_counter_and_overflow_bound(self):
        window = self.main._V7PerfWindow()
        self.assertEqual(window.frames, 0)
        self.assertEqual(window.valid_full_writes, 0)
        window.frames = 2
        window.valid_full_writes = 2
        window.record("total_cycle", 300000)
        window.record("total_cycle", 600000)
        summary = window.summary_line(4, 0, 2)
        self.assertIn("valid_full_writes=2", summary)
        stage = re.search(r"total_cycle:n=2,[^|]+", summary).group(0)
        self.assertIn("p95_upper=600000", stage)
        self.assertIn("max=600000", stage)
        window.clear()
        self.assertEqual(window.frames, 0)
        self.assertEqual(window.valid_full_writes, 0)
        self.assertEqual(window.counts["total_cycle"], 0)
        self.assertEqual(sum(window.bins["total_cycle"]), 0)
        window.record("total_cycle", 300000)
        self.assertIn("p95_upper=300000",
                      window._stage_summary("total_cycle"))
        lines = window.summary_lines(4, 1, 2)
        self.assertTrue(lines[0].startswith("V7_PERF_WINDOW"))
        self.assertIn("valid_full_writes=0", lines[0])
        self.assertEqual(len(lines), len(self.main._V7_PERF_STAGES) + 1)
        total_cycle = [line for line in lines
                       if " total_cycle:" in line][0]
        self.assertIn("p95_upper=300000", total_cycle)

    def test_perf_window_advances_across_ticks_us_wrap(self):
        fake_time = FakeTime()
        period = fake_time.TICKS_PERIOD
        start = period - 5_000_000
        send_end = fake_time.ticks_add(start, 25_250_000)
        with patch.object(self.main, "time", fake_time):
            next_start, advanced = self.main._advance_v7_perf_window(
                send_end, start, 10_000_000)
        self.assertEqual(advanced, 2)
        self.assertEqual(next_start,
                         fake_time.ticks_add(start, 20_000_000))
        self.assertEqual(fake_time.ticks_diff(send_end, next_start),
                         5_250_000)

        exact_boundary = fake_time.ticks_add(next_start, 10_000_000)
        with patch.object(self.main, "time", fake_time):
            next_start, advanced = self.main._advance_v7_perf_window(
                exact_boundary, next_start, 10_000_000)
        self.assertEqual(advanced, 1)
        self.assertEqual(next_start, exact_boundary)

    def test_budget_gate_rejects_late_valid_and_allows_exact_limit(self):
        gate = self.main._v7_valid_within_budget
        self.assertTrue(gate(True, 178000, 200000, 22000))
        self.assertFalse(gate(True, 178001, 200000, 22000))
        self.assertFalse(gate(False, 1, 200000, 22000))

    def test_api_and_mixed_build_mismatches_fail_closed(self):
        self.main._load_fast_runtime()
        self.assertEqual(config.VISION_BACKEND, "fast")
        self.assertEqual(config.BUILD_ID,
                         "SIXTH_V7_FAST_GRAY_RT45_20261005")
        with patch.object(self.main.fast_boundary,
                          "RUNTIME_API_VERSION", 6):
            with self.assertRaisesRegex(RuntimeError, "fast_boundary.py API"):
                self.main._validate_v7_runtime_apis()
        with patch.object(self.main.fast_gray, "RUNTIME_API_VERSION", 6):
            with self.assertRaisesRegex(RuntimeError, "fast_gray.py API"):
                self.main._validate_v7_runtime_apis()
        with patch.object(config, "BUILD_ID", "mixed-v7-build"):
            with self.assertRaisesRegex(RuntimeError, "config/build mismatch"):
                self.main._validate_fast_config()

    def test_main_dispatches_fast_backend_to_fast_loop(self):
        with patch.object(config, "VISION_BACKEND", "fast"), \
                patch.object(self.main, "_main_fast") as run_fast:
            self.main.main()
        run_fast.assert_called_once_with()

    def _run_fast_stub(self, frames, inject_late_pack=False,
                       duration_s=None, detector_failure_stage=None,
                       debug=False, detector_error_text=None,
                       uart_short_write=False, overlay=False):
        fake_time = FakeTime()
        fake_sensor = FakeSensor(frames)
        fake_sensor.fake_time = fake_time
        fake_pyb = FakePyb()
        fake_gc = FakeGC()
        FakeGrayBuilder.instances = []
        FakePathDetector.instances = []
        FakeBoundaryValidator.instances = []
        FakeGrayBuilder.fail_on_build = detector_failure_stage == "MASK"
        FakePathDetector.fail_on_detect = detector_failure_stage == "PATH"
        FakeBoundaryValidator.fail_on_gate = detector_failure_stage == "BOUNDARY"
        FakeGrayBuilder.failure_message = detector_error_text or \
            "injected mask failure"
        FakePathDetector.failure_message = detector_error_text or \
            "injected path failure"
        FakeBoundaryValidator.failure_message = detector_error_text or \
            "injected boundary failure"
        FakeUart.short_write = uart_short_write
        gray, path, topology, boundary, _accept = make_runtime_stubs(self.main)
        pack_calls = []
        console = []
        draw_calls = []
        perf_samples = []
        perf_instances = []
        perf_class = self.main._V7PerfWindow

        class TrackingPerfWindow(perf_class):
            def __init__(self):
                super().__init__()
                perf_instances.append(self)

            def record(self, stage, elapsed_us):
                perf_samples.append((stage, int(elapsed_us)))
                return super().record(stage, elapsed_us)

        real_pack = self.main.vision_protocol.pack_frame

        def pack_frame(sequence, valid, confidence, lateral_error):
            pack_calls.append((sequence, valid, confidence, lateral_error))
            packet = real_pack(sequence, valid, confidence, lateral_error)
            if inject_late_pack and valid and len(pack_calls) == 1:
                fake_time.us += 180000
            return packet

        def draw_result(*args):
            fake_sensor.events.append("draw")
            draw_calls.append(args)

        with patch.object(config, "VISION_BACKEND", "fast"), \
                patch.object(config, "DEBUG_ENABLED", debug), \
                patch.object(config, "DEBUG_PERIOD_MS", 1), \
                patch.object(config, "OVERLAY_ENABLED", overlay), \
                patch.object(self.main, "sensor", fake_sensor), \
                patch.object(self.main, "pyb", fake_pyb), \
                patch.object(self.main, "time", fake_time), \
                patch.object(self.main, "gc", fake_gc), \
                patch.object(self.main, "_V7PerfWindow",
                             TrackingPerfWindow), \
                patch.object(self.main, "fast_gray", gray), \
                patch.object(self.main, "fast_path", path), \
                patch.object(self.main, "fast_topology", topology), \
                patch.object(self.main, "fast_boundary", boundary), \
                patch.object(self.main, "_draw_gray_result",
                             side_effect=draw_result), \
                patch.object(self.main.vision_protocol, "pack_frame",
                             side_effect=pack_frame), \
                patch("builtins.print", side_effect=lambda *args:
                      console.append(" ".join(str(arg) for arg in args))):
            if duration_s is None:
                with self.assertRaises(StopAfterFrames):
                    self.main._main_fast()
            else:
                self.main._main_fast(duration_s=duration_s, run_id=9)
        return {
            "sensor": fake_sensor,
            "uart": fake_pyb.uart,
            "gc": fake_gc,
            "gray": gray,
            "path": path,
            "boundary": boundary,
            "pack_calls": pack_calls,
            "console": console,
            "draw_calls": draw_calls,
            "perf_samples": perf_samples,
            "time": fake_time,
            "perf": perf_instances[0] if perf_instances else None,
        }

    def test_35_frame_fast_loop_uses_reference_and_unique_crc_packets(self):
        run = self._run_fast_stub(35)
        frames = run["uart"].frames
        self.assertEqual(run["sensor"].frames, 35)
        self.assertEqual(len(FakeGrayBuilder.instances), 1)
        self.assertEqual(FakeGrayBuilder.instances[0].calls, 35)
        self.assertEqual(len(FakePathDetector.instances), 1)
        self.assertEqual(FakePathDetector.instances[0].calls, 35)
        self.assertEqual(len(FakeBoundaryValidator.instances), 1)
        self.assertEqual(FakeBoundaryValidator.instances[0].gate_calls, 35)
        self.assertEqual(FakeBoundaryValidator.instances[0].reference_calls, 35)
        self.assertEqual(len(frames), 35)
        self.assertEqual(run["gc"].collect_calls, 1)
        self.assertEqual(len(run["pack_calls"]), 35)
        packets = [decode_packet(self.main.vision_protocol, frame)
                   for frame in frames]
        self.assertEqual([packet["sequence"] for packet in packets],
                         list(range(35)))
        self.assertTrue(all(packet["valid"] == 1 for packet in packets))
        self.assertTrue(all(packet["confidence"] == 92 for packet in packets))
        self.assertTrue(all(packet["error"] == 0 for packet in packets))
        self.assertTrue(all(call[1:] == (1, 92, 0)
                            for call in run["pack_calls"]))

    def test_packet_ready_late_valid_is_replaced_by_same_sequence_invalid(self):
        run = self._run_fast_stub(1, inject_late_pack=True)
        self.assertEqual(len(run["pack_calls"]), 2)
        self.assertEqual(run["pack_calls"], [
            (0, 1, 92, 0),
            (0, 0, 0, 0),
        ])
        self.assertEqual(len(run["uart"].frames), 1)
        packet = decode_packet(self.main.vision_protocol,
                               run["uart"].frames[0])
        self.assertEqual(packet["sequence"], 0)
        self.assertEqual(packet["flags"], 0)
        self.assertEqual(packet["valid"], 0)
        self.assertEqual(packet["confidence"], 0)
        self.assertEqual(packet["error"], 0)

    def test_detector_exceptions_count_and_debug_while_sending_invalid_packets(self):
        for stage in ("MASK", "PATH", "BOUNDARY"):
            with self.subTest(stage=stage):
                run = self._run_fast_stub(
                    1, detector_failure_stage=stage, debug=True)
                self.assertEqual(len(run["uart"].frames), 1)
                packet = decode_packet(
                    self.main.vision_protocol, run["uart"].frames[0])
                self.assertEqual(packet["valid"], 0)
                self.assertEqual(packet["flags"], 0)
                self.assertEqual(packet["confidence"], 0)
                self.assertEqual(packet["error"], 0)

                debug_lines = [line for line in run["console"]
                               if line.startswith("V7DBG ")]
                self.assertEqual(len(debug_lines), 1)
                self.assertIn("txerr=0 deterr=1", debug_lines[0])
                self.assertIn("derr=%s:injected %s failure" % (
                    stage, stage.lower()), debug_lines[0])

    def test_detector_error_and_uart_short_write_counters_are_independent(self):
        run = self._run_fast_stub(
            1, detector_failure_stage="MASK", debug=True,
            uart_short_write=True)
        self.assertEqual(run["pack_calls"], [(0, 0, 0, 0)])
        self.assertEqual(len(run["uart"].frames), 1)
        packet = decode_packet(self.main.vision_protocol,
                               run["uart"].frames[0])
        self.assertEqual((packet["valid"], packet["flags"],
                          packet["confidence"], packet["error"]),
                         (0, 0, 0, 0))
        debug = [line for line in run["console"]
                 if line.startswith("V7DBG ")]
        self.assertEqual(len(debug), 1)
        self.assertIn("txerr=1 deterr=1", debug[0])
        self.assertEqual(run["perf"].write_attempts, 1)
        self.assertEqual(run["perf"].write_errors, 1)
        self.assertEqual(run["perf"].valid_frames, 0)
        self.assertEqual(run["perf"].full_writes, 0)
        self.assertEqual(run["perf"].valid_full_writes, 0)

        # A short UART write is counted once and is never retried under the
        # same sequence. Only full writes enter valid_full_writes.
        short_valid = self._run_fast_stub(
            1, debug=True, uart_short_write=True)
        self.assertEqual(short_valid["pack_calls"], [(0, 1, 92, 0)])
        self.assertEqual(len(short_valid["uart"].frames), 1)
        self.assertEqual(short_valid["perf"].write_attempts, 1)
        self.assertEqual(short_valid["perf"].write_errors, 1)
        self.assertEqual(short_valid["perf"].valid_full_writes, 0)
        self.assertIn("txerr=1 deterr=0", [
            line for line in short_valid["console"]
            if line.startswith("V7DBG ")][0])

    def test_detector_error_text_is_bounded_and_not_printed_each_frame(self):
        run = self._run_fast_stub(
            1, detector_failure_stage="MASK", debug=True,
            detector_error_text="x" * 200)
        debug = [line for line in run["console"]
                 if line.startswith("V7DBG ")]
        self.assertEqual(len(debug), 1)
        error_text = re.search(r"derr=(\S+) fps=", debug[0]).group(1)
        self.assertEqual(len(error_text), 96)
        self.assertTrue(error_text.startswith("MASK:"))

        quiet = self._run_fast_stub(
            1, detector_failure_stage="MASK", debug=False)
        self.assertEqual(len(quiet["console"]), 1)
        self.assertTrue(quiet["console"][0].startswith("V7_BUILD="))

    def test_overlay_age_and_debug_thresholds_use_capture_to_send_metrics(self):
        run = self._run_fast_stub(
            1, inject_late_pack=True, debug=True, overlay=True)
        capture_to_send = [value for stage, value in run["perf_samples"]
                           if stage == "capture_to_send"]
        self.assertEqual(len(capture_to_send), 1)
        expected_ms = capture_to_send[0] // 1000
        self.assertGreaterEqual(expected_ms, 180)
        self.assertEqual(len(run["draw_calls"]), 1)
        self.assertEqual(run["sensor"].flush_calls, 1)
        self.assertEqual(run["sensor"].events, ["draw", "flush"])
        overlay = [value for stage, value in run["perf_samples"]
                   if stage == "overlay"]
        self.assertEqual(len(overlay), 1)
        self.assertGreaterEqual(overlay[0], 500)
        # _draw_gray_result's ninth argument is the p=...ms overlay value.
        self.assertEqual(run["draw_calls"][0][8], expected_ms)

        debug_lines = [line for line in run["console"]
                       if line.startswith("V7DBG ")]
        self.assertEqual(len(debug_lines), 1)
        line = debug_lines[0]
        self.assertIn("age_send_ms=%d" % expected_ms, line)
        self.assertIn("trusted_block_count=3", line)
        self.assertIn("threshold_min=80", line)
        self.assertIn("threshold_max=120", line)
        self.assertLess(len(line), 500)

        no_overlay = self._run_fast_stub(1, overlay=False)
        self.assertEqual(no_overlay["sensor"].flush_calls, 0)

    def test_fast_overlay_draws_sparse_contours_and_keeps_control_markers(self):
        image = DrawRecorder()
        rows = tuple({"y": y, "left": 20 + (y - 20) // 10,
                      "right": 59 + (y - 20) // 10}
                     for y in range(20, 60))
        road = {"selected_rows": rows}
        selection = {"valid": True, "source": "road", "quality": 90,
                     "reason": "OK"}
        reference = {"x": 39.5, "y": 40.0, "kind": "reference_y"}
        with patch.object(config, "OVERLAY_ENABLED", True):
            self.main._draw_gray_result(
                image, selection, {"sections": ()}, road, reference,
                True, 0, 0, 20, 45.0)

        # Forty detector rows are rendered as five connected segments per
        # side, while preserving both endpoints and the raw/filtered markers.
        self.assertEqual(len(image.lines), 10)
        self.assertEqual(len(image.crosses), 2)
        self.assertEqual(len(image.strings), 4)
        self.assertEqual(image.strings[0][1], "VALID r=+0 f=+0")
        self.assertEqual(image.lines[0][0][1],
                         self.main._surface_pixel((rows[0]["left"],
                                                   rows[0]["y"]))[1])
        self.assertEqual(image.lines[-2][0][3],
                         self.main._surface_pixel((rows[-1]["left"],
                                                   rows[-1]["y"]))[1])

        surface_image = DrawRecorder()
        sections = tuple({"left": (15 + index, index),
                          "right": (65 - index, index)}
                         for index in range(32))
        self.main._draw_sparse_surface_contour(surface_image, sections)
        self.assertEqual(len(surface_image.lines), 10)
        self.assertEqual(surface_image.lines[-2][0][3],
                         self.main._surface_pixel(sections[-1]["left"])[1])

        invalid_image = DrawRecorder()
        with patch.object(config, "OVERLAY_ENABLED", True):
            self.main._draw_gray_result(
                invalid_image, {"valid": False, "reason": "NO_PATH"},
                {"sections": ()}, {"selected_rows": ()}, None,
                False, 999, -999, 20, 45.0)
        self.assertEqual(invalid_image.strings[0][1],
                         "INVALID r=+0 f=+0")

    def test_finite_fast_run_emits_short_footer_and_drains_sdk(self):
        run = self._run_fast_stub(0, duration_s=0)
        self.assertEqual(run["sensor"].frames, 0)
        self.assertEqual(run["uart"].frames, [])
        self.assertEqual(run["pack_calls"], [])
        header = [line for line in run["console"]
                  if line.startswith("V7_PERF_WINDOW")]
        stages = [line for line in run["console"]
                  if line.startswith("V7_PERF_STAGE")]
        done = [line for line in run["console"]
                if line.startswith("V7_FAST_RUN_DONE")]
        self.assertEqual(len(header), 1)
        self.assertEqual(len(stages), len(self.main._V7_PERF_STAGES))
        self.assertEqual(len(done), 1)
        self.assertEqual(run["console"][-1], done[0])
        self.assertIn("frames=0", done[0])
        self.assertEqual(run["time"].sleep_calls,
                         [5] * (len(stages) + 1) + [500])


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(
        stream=sys.stdout, verbosity=2))
