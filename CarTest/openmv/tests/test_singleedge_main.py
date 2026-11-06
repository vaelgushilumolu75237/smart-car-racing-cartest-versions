"""Host tests for main-loop reverse-session handling and fail-closed wiring."""

from array import array
import sys
import types
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from unittest.mock import patch


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config  # noqa: E402
import main  # noqa: E402
import fast_single_edge  # noqa: E402
import reverse_control  # noqa: E402
import vision_protocol  # noqa: E402


class _FakeUart:
    def __init__(self, data=b"", no_read=False):
        self.data = bytearray(data)
        self.no_read = no_read

    def any(self):
        return len(self.data)

    def readinto(self, buffer, count):
        if self.no_read:
            return None
        count = min(count, len(self.data))
        buffer[:count] = self.data[:count]
        del self.data[:count]
        return count


class _StopFrames(Exception):
    pass


class _FakeTime:
    def __init__(self):
        self.us = 0

    def ticks_us(self):
        self.us += 100
        return self.us

    def ticks_ms(self):
        return self.us // 1000

    @staticmethod
    def ticks_diff(newer, older):
        return newer - older

    @staticmethod
    def ticks_add(value, delta):
        return value + delta

    def clock(self):
        return self

    def tick(self):
        self.us += 100

    @staticmethod
    def fps():
        return 100.0

    def delay_ms(self, value):
        self.us += int(value) * 1000


class _IntegrationUart(_FakeUart):
    def __init__(self, short_write_at=None):
        super().__init__()
        self.frames = []
        self.short_write_at = short_write_at

    def write(self, packet):
        self.frames.append(bytes(packet))
        if self.short_write_at == len(self.frames) - 1:
            return len(packet) - 1
        return len(packet)


class _FakePyb:
    def __init__(self, uart):
        self.uart = uart

    def UART(self, *args, **kwargs):
        return self.uart


class _FakeSensor:
    def __init__(self, controls):
        self.controls = controls
        self.uart = None
        self.index = 0
        self.flush_calls = 0

    def snapshot(self):
        if self.index >= len(self.controls):
            raise _StopFrames()
        if self.uart is not None:
            self.uart.data.extend(self.controls[self.index])
        self.index += 1
        return _FakeImage()

    @staticmethod
    def get_exposure_us():
        return 10000

    @staticmethod
    def get_gain_db():
        return 12.0

    def flush(self):
        self.flush_calls += 1


class _FakeImage:
    @staticmethod
    def width():
        return 320

    @staticmethod
    def height():
        return 120

    @staticmethod
    def draw_line(*args, **kwargs):
        pass

    @staticmethod
    def draw_cross(*args, **kwargs):
        pass

    @staticmethod
    def draw_string(*args, **kwargs):
        pass


def _grid_mask(mode, left=24, right=48, sparse=False, curve=False):
    width = config.SURFACE_GRID_WIDTH
    height = config.SURFACE_GRID_HEIGHT
    floor = config.SURFACE_ROI_FLOOR_Y
    road = bytearray(width * height)
    edge = bytearray([1]) * (width * height)
    known = bytearray([1]) * (width * height)
    luma = bytearray([25]) * (width * height)
    rows = range(floor, height)
    if sparse:
        rows = (30, 32, 34, 36, 38, 40, 42, 44)
        road = bytearray([1]) * (width * height)
        edge = bytearray(width * height)
        luma = bytearray([220]) * (width * height)
    for y in rows:
        row = y * width
        if curve:
            curve_left = left + int((y - floor) * 8 /
                                    (height - 1 - floor))
            x0, x1 = curve_left, curve_left + (right - left)
        elif mode == "both":
            x0, x1 = left, right
        elif mode == "left":
            x0, x1 = left, width - 1
        else:
            x0, x1 = 0, right
        if sparse and mode == "left":
            for x in range(left):
                road[row + x] = 0
                edge[row + x] = 1
                luma[row + x] = 25
        for x in range(x0, x1 + 1):
            road[row + x] = 1
            edge[row + x] = 0
            luma[row + x] = 220
    return {
        "valid": True,
        "width": width,
        "height": height,
        "floor": floor,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
    }


def _dual_path_result(left=24, right=48, quality=1):
    rows = array("i")
    for y in range(config.SURFACE_ROI_FLOOR_Y,
                   config.SURFACE_GRID_HEIGHT):
        rows.extend((y, left, right, 7))
    return {
        "valid": True,
        "quality": quality,
        "lateral_error": 0,
        "reason": "OK",
        "scene": "TRACK",
        "reference_center_x": (left + right) * 0.5,
        "reference_y": 40.0,
        "reference_provenance": "OBSERVED_BOTH",
        "selected_row_ints": rows,
        "selected_row_count": config.SURFACE_GRID_HEIGHT -
                              config.SURFACE_ROI_FLOOR_Y,
        "selected_row_stride": 4,
        "active_row_count": config.SURFACE_GRID_HEIGHT -
                            config.SURFACE_ROI_FLOOR_Y,
        "topology_evidence": {
            "reason": "OK", "island_evidence": False,
            "branch_evidence": False, "ambiguous": False,
        },
    }


def _decode_observation(frame):
    frame = bytes(frame)
    if (len(frame) != 16 or frame[:2] != b"\xA5\x5A" or
            frame[2] not in (2, 3) or frame[3] != 8):
        raise AssertionError("invalid observation frame")
    expected = main.vision_protocol.crc16_ccitt_false(frame[2:14])
    actual = frame[14] | frame[15] << 8
    if actual != expected:
        raise AssertionError("observation CRC mismatch")
    raw = frame[8] | frame[9] << 8
    if raw & 0x8000:
        raw -= 0x10000
    return {
        "valid": frame[6] & 1,
        "quality": frame[7],
        "error": raw,
        "profile_ack": frame[11] if frame[2] == 3 else 0,
        "version": frame[2],
    }


class SingleEdgeMainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        main._load_singleedge_runtime()

    def _service(self, uart, parser, drain, now):
        return main._service_reverse_control(
            uart, parser, bytearray(reverse_control.MAX_FEED_BYTES), drain,
            now, lambda newer, older: newer - older)

    def test_expired_lease_discards_queued_old_keep_before_fresh_reset(self):
        parser = reverse_control.ReverseControlParser()
        drain = array("i", [0])
        initial = bytes(reverse_control.pack_frame(0, 100)) + bytes(
            reverse_control.pack_frame(1, 101))
        self.assertTrue(self._service(_FakeUart(initial), parser, drain, 0))

        stale = _FakeUart(bytes(reverse_control.pack_frame(1, 102)))
        self.assertFalse(self._service(stale, parser, drain, 250))
        self.assertEqual(parser.last_event, "CONTROL_TIMEOUT")
        self.assertEqual(len(stale.data), 0)
        self.assertEqual(drain[0], 1)
        self.assertFalse(self._service(stale, parser, drain, 251))
        self.assertEqual(drain[0], 0)

        reset_and_keep = bytes(reverse_control.pack_frame(0, 103)) + bytes(
            reverse_control.pack_frame(1, 104))
        self.assertTrue(self._service(
            _FakeUart(reset_and_keep), parser, drain, 300))

    def test_receive_overflow_drains_in_bounded_chunks_then_requires_reset(self):
        parser = reverse_control.ReverseControlParser()
        drain = array("i", [0])
        excessive = _FakeUart(
            bytes(reverse_control.pack_frame(0, 1)) * 9)
        self.assertFalse(self._service(excessive, parser, drain, 0))
        self.assertEqual(parser.last_event, "RX_OVERFLOW")
        self.assertEqual(len(excessive.data), 8)
        self.assertFalse(self._service(excessive, parser, drain, 1))
        self.assertEqual(len(excessive.data), 0)
        self.assertFalse(self._service(excessive, parser, drain, 2))
        self.assertEqual(drain[0], 0)

        valid = bytes(reverse_control.pack_frame(0, 9)) + bytes(
            reverse_control.pack_frame(1, 10))
        self.assertTrue(self._service(_FakeUart(valid), parser, drain, 3))

    def test_readinto_none_does_not_parse_reused_buffer_contents(self):
        parser = reverse_control.ReverseControlParser()
        stale = bytes(reverse_control.pack_frame(0, 1)) + bytes(
            reverse_control.pack_frame(1, 2))
        buffer = bytearray(stale)
        uart = _FakeUart(stale, no_read=True)
        self.assertFalse(main._service_reverse_control(
            uart, parser, buffer, array("i", [0]), 0,
            lambda newer, older: newer - older))
        self.assertEqual(parser.buffered_bytes, 0)
        self.assertFalse(parser.single_enabled)

    def test_timeout_discards_partial_frame_and_fresh_reset_recovers(self):
        parser = reverse_control.ReverseControlParser()
        reset = reverse_control.pack_frame(0, 20)
        parser.feed(reset, 4, 0)
        self.assertEqual(parser.buffered_bytes, 4)
        self.assertTrue(parser.poll(250, lambda newer, older: newer - older))
        self.assertEqual(parser.buffered_bytes, 0)
        parser.feed(reset[4:], 4, 251)
        self.assertFalse(parser.single_enabled)
        parser.feed(reset, len(reset), 252)
        parser.feed(reverse_control.pack_frame(1, 21), 8, 253)
        self.assertTrue(parser.single_enabled)

    def test_fallback_rejects_branch_ambiguity_but_allows_checked_island(self):
        self.assertFalse(main._single_edge_fallback_allowed({
            "reason": "NO_PAIRED_EDGES", "scene": "UNKNOWN",
            "topology_evidence": {"branch_evidence": True}}))
        self.assertFalse(main._single_edge_fallback_allowed({
            "reason": "AMBIGUOUS_ROAD", "scene": "AMBIGUOUS_CANDIDATE"}))
        self.assertTrue(main._single_edge_fallback_allowed({
            "reason": "NO_PAIRED_EDGES", "scene": "UNKNOWN",
            "topology_evidence": {"island_evidence": True}}))

    def test_new_bundle_apis_are_required_and_quality_is_diagnostic_only(self):
        self.assertTrue(main._validate_fast_config())
        self.assertEqual(config.SINGLEEDGE_RUNTIME_API_VERSION, 2)
        with patch.object(main.reverse_control, "RUNTIME_API_VERSION", 1):
            with self.assertRaisesRegex(RuntimeError, "reverse_control.py API"):
                main._validate_fast_config()
        with patch.object(main.fast_path,
                          "CONFIDENCE_DIAGNOSTIC_ONLY", False):
            with self.assertRaisesRegex(RuntimeError,
                                        "confidence gate mode"):
                main._validate_fast_config()
        with patch.object(config, "SURFACE_MIN_CONFIDENCE", 0):
            self.assertTrue(main._validate_fast_config())

    def test_main_pipeline_dual_single_dual_preserves_filter_and_clears_on_error(self):
        """Exercise the real main loop, UART control parser and tracker together."""
        main._load_fast_runtime()
        timebase = _FakeTime()
        uart = _IntegrationUart(short_write_at=9)
        controls = (
            bytes(reverse_control.pack_frame(0, 1)) +
            bytes(reverse_control.pack_frame(1, 2)) +
            bytes(reverse_control.pack_profile_state(
                0, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 3)) +
            bytes(reverse_control.pack_profile_state(
                1, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 4)) +
            bytes(reverse_control.pack_profile_state(
                2, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 5)) +
            bytes(reverse_control.pack_profile_state(
                3, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 6)) +
            bytes(reverse_control.pack_profile_state(
                4, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 7)) +
            bytes(reverse_control.pack_profile_state(
                5, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 8)) +
            bytes(reverse_control.pack_profile_state(
                6, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(0, 9)) +
            bytes(reverse_control.pack_frame(1, 10)) +
            bytes(reverse_control.pack_profile_state(
                7, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 11)) +
            bytes(reverse_control.pack_profile_state(
                8, reverse_control.PROFILE_A)),
            bytes(reverse_control.pack_frame(1, 12)) +
            bytes(reverse_control.pack_profile_state(
                9, reverse_control.PROFILE_A)),
        )
        camera = _FakeSensor(controls)
        camera.uart = uart

        masks = (
            _grid_mask("both", left=19, right=43, curve=True),
            _grid_mask("left", left=26, right=50, sparse=True),
            _grid_mask("both", left=19, right=43, curve=True),
            _grid_mask("left", left=26, right=50, sparse=True),
            _grid_mask("both", left=19, right=43, curve=True),
            _grid_mask("left", left=26, right=50, sparse=True),
            _grid_mask("left", left=26, right=50, sparse=True),
            _grid_mask("both", left=19, right=43, curve=True),
            _grid_mask("left", left=26, right=50, sparse=True),
            _grid_mask("left", left=26, right=50, sparse=True),
        )
        paths = (
            _dual_path_result(24, 48, quality=1),
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
            _dual_path_result(24, 48, quality=1),
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
            _dual_path_result(24, 48, quality=1),
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
            _dual_path_result(24, 48, quality=1),
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
            {"valid": False, "quality": 0, "reason": "NO_PAIRED_EDGES",
             "scene": "UNKNOWN", "topology_evidence": {}},
        )

        trackers = []
        base_tracker = fast_single_edge.FastSingleEdgeTracker

        class TrackingTracker(base_tracker):
            def __init__(self, settings):
                super().__init__(settings)
                trackers.append(self)

        filters = []
        console = StringIO()
        base_filter = main._FreshObservationFilter

        class TrackingFilter(base_filter):
            def __init__(self, *args):
                super().__init__(*args)
                self.reset_count = 0
                filters.append(self)

            def reset(self):
                self.reset_count += 1
                super().reset()

        class FakeMaskBuilder:
            def __init__(self, settings):
                self.index = 0

            def build(self, image):
                current = self.index
                self.index += 1
                if current in (0, 5):
                    # Cross the 178 ms pre-write deadline once before seeding
                    # and once after warmup; neither late frame may cache.
                    timebase.delay_ms(180)
                return masks[current]

        class FakePathDetector:
            def __init__(self, settings):
                self.index = 0

            def detect(self, mask_info):
                current = self.index
                self.index += 1
                candidate = dict(paths[current])
                if candidate.get("valid"):
                    packed = candidate["selected_row_ints"]
                    for row_index in range(candidate["selected_row_count"]):
                        offset = row_index * candidate["selected_row_stride"]
                        y = packed[offset]
                        # Keep the cached bilateral path continuous so the
                        # later single-edge frames retain a real identity
                        # anchor, while making the weighted aggregate differ
                        # from the fixed geometric safety reference.
                        left = 19 + int((y - config.SURFACE_ROI_FLOOR_Y) *
                                        8 / (config.SURFACE_GRID_HEIGHT - 1 -
                                             config.SURFACE_ROI_FLOOR_Y))
                        packed[offset + 1] = left
                        packed[offset + 2] = left + 24
                return candidate

        class FakeBoundaryValidator:
            def __init__(self, settings):
                pass

            @staticmethod
            def gate_fast_path_candidate(candidate, *args):
                count = candidate["selected_row_count"]
                candidate["boundary_support_rows"] = bytearray(
                    [1] * count)
                candidate["boundary_support_count"] = count
                return candidate

            @staticmethod
            def validate_selected_reference(*args):
                return {"valid": True, "reason": "HOST_TEST"}

        class FakeGC:
            @staticmethod
            def collect():
                pass

        with patch.object(main, "_configure_gray_sensor", return_value=0), \
                patch.object(main, "sensor", camera), \
                patch.object(main, "pyb", _FakePyb(uart)), \
                patch.object(main, "time", timebase), \
                patch.object(main, "gc", FakeGC()), \
                patch.object(main.fast_gray, "FastGrayMasks",
                             FakeMaskBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             FakePathDetector), \
                patch.object(main.fast_boundary,
                             "FastBoundaryEvidence", FakeBoundaryValidator), \
                patch.object(main.fast_single_edge,
                             "FastSingleEdgeTracker", TrackingTracker), \
                patch.object(main, "_FreshObservationFilter", TrackingFilter), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "DEBUG_PERIOD_MS", 0), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                patch.object(config, "PERF_GC_PERIOD_FRAMES", 1000), \
                redirect_stdout(console):
            with self.assertRaises(_StopFrames):
                main._main_fast()

        self.assertEqual(len(uart.frames), 10)
        self.assertEqual(camera.flush_calls, 10)
        decoded = [_decode_observation(frame) for frame in uart.frames]
        self.assertTrue(all(frame[2] == vision_protocol.VERSION_V3
                            for frame in uart.frames))
        self.assertEqual([item["profile_ack"] for item in decoded],
                         [reverse_control.PROFILE_A] * len(decoded))
        debug_lines = [line for line in console.getvalue().splitlines()
                       if line.startswith("V7DBG ")]
        control_by_sequence = {}
        for line in console.getvalue().splitlines():
            if line.startswith("V7CTRL "):
                control = dict(token.split("=", 1)
                               for token in line.split()[1:]
                               if "=" in token)
                control_by_sequence[int(control["seq"])] = control
        self.assertTrue(debug_lines)
        self.assertTrue(any("safe_ref=" in line and "wctrl=" in line and
                            "rows_nf=" in line and "eff_nf=" in line and
                            "pos_weighted=" in line and "raw_pos=" in line and
                            "tx_cmd=" in line and "road_why=" in line
                            for line in debug_lines))
        self.assertLess(max(len(line) for line in debug_lines), 1024)
        self.assertEqual(decoded[0]["valid"], 0)  # frame-budget rejection
        self.assertEqual(decoded[1]["valid"], 0)  # first single has no cache
        self.assertEqual((decoded[2]["valid"], decoded[2]["quality"]),
                         (1, 1))  # numeric score is diagnostic only
        self.assertEqual((decoded[3]["valid"], decoded[3]["quality"]),
                         (1, 56))  # below legacy min confidence 60
        raw_single = main._normalize_fast_reference(38.0, 80)
        self.assertNotEqual(decoded[3]["error"], raw_single)
        raw_dual = main._normalize_fast_reference(36.0, 80)
        self.assertEqual((decoded[4]["valid"], decoded[4]["quality"]),
                         (1, 1))  # single-to-dual transition stays filtered
        self.assertNotEqual(decoded[4]["error"], raw_dual)
        for index, frame in enumerate(uart.frames):
            sequence = frame[4] | frame[5] << 8
            control = control_by_sequence[sequence]
            preview_value = int(control["preview"].split(":", 1)[1])
            expected_command = max(
                -1000, min(1000, int(control["pd"]) + preview_value))
            self.assertEqual(decoded[index]["error"],
                             int(control["tx_cmd"]),
                             "%s != %s for sequence %d" % (
                                 decoded[index]["error"], control["tx_cmd"],
                                 sequence))
            if int(control["reset"]) != 9:  # UART error resets state post-write.
                self.assertEqual(int(control["tx_cmd"]), expected_command,
                                 "control=%r" % control)
        self.assertEqual(decoded[2]["error"], int(control_by_sequence[2]["pd"]))
        self.assertNotEqual(decoded[2]["error"], raw_dual)
        self.assertEqual(decoded[5]["valid"], 0)  # late frame clears cache
        self.assertEqual(decoded[6]["valid"], 0)  # cache must reacquire
        self.assertEqual((decoded[7]["valid"], decoded[7]["quality"]),
                         (1, 1))
        self.assertEqual(int(control_by_sequence[7]["d_component"]), 0)
        self.assertEqual(decoded[7]["error"], int(control_by_sequence[7]["p"]))
        self.assertEqual((decoded[8]["valid"], decoded[8]["quality"]),
                         (1, 56))
        self.assertNotEqual(decoded[8]["error"], raw_single)
        self.assertEqual((decoded[9]["valid"], decoded[9]["quality"]),
                         (1, 56))  # failed short write was attempted valid
        self.assertEqual(trackers[0].cache_rows, 0)
        self.assertIsNone(filters[0].value)
        self.assertGreaterEqual(filters[0].reset_count, 3)


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(SingleEdgeMainTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
