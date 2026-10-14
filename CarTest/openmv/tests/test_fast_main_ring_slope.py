"""Exercise the real _main_fast selection, aggregate, pack, and UART path."""

import contextlib
import io
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
import main
import vision_protocol


WIDTH = 80
HEIGHT = 60


class _Time:
    def __init__(self):
        self.us = 0

    def ticks_us(self):
        self.us += 100
        return self.us

    def ticks_ms(self):
        return self.us // 1000

    @staticmethod
    def ticks_diff(new, old):
        return new - old

    @staticmethod
    def ticks_add(value, delta):
        return value + delta

    def sleep_ms(self, _milliseconds):
        return None

    def clock(self):
        return _Clock()


class _Clock:
    @staticmethod
    def tick():
        return None

    @staticmethod
    def fps():
        return 30.0


class _Image:
    def __init__(self):
        self.raw = bytearray(160 * 120)

    @staticmethod
    def width():
        return 160

    @staticmethod
    def height():
        return 120

    def bytearray(self):
        return self.raw


class _Sensor:
    def __init__(self):
        self.image = _Image()

    @staticmethod
    def get_exposure_us():
        return 5000

    @staticmethod
    def get_gain_db():
        return 0.0

    def snapshot(self):
        return self.image

    @staticmethod
    def flush():
        return None


class _UART:
    def __init__(self, fake_time):
        self.fake_time = fake_time
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        # End the finite one-frame main-loop run after the packet is written.
        self.fake_time.us = 1100000
        return len(frame)


class _GrayBuilder:
    def __init__(self, _settings):
        self.road = bytearray(WIDTH * HEIGHT)
        self.edge = bytearray(WIDTH * HEIGHT)
        self.known = bytearray(WIDTH * HEIGHT)
        for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
            for x in range(WIDTH):
                self.known[y * WIDTH + x] = 1
                if x in (29, 31, 49, 51):
                    self.edge[y * WIDTH + x] = 1
                if x in (30, 50):
                    self.road[y * WIDTH + x] = 1

    def build(self, _image):
        return {"road_mask": self.road, "edge_mask": self.edge,
                "known_region": self.known,
                "luma": bytearray(WIDTH * HEIGHT),
                "trusted_block_count": 4, "threshold_min": 10,
                "threshold_max": 20, "reason": "TEST_MASK"}


class _PathDetector:
    def __init__(self, _settings):
        rows = ((10, 20, 40), (20, 40, 60),
                (40, 40, 60), (55, 40, 60))
        packed = array("i")
        selected = []
        for y, left, right in rows:
            packed.extend((y, left, right, 7))
            selected.append({"y": float(y), "left": float(left),
                             "right": float(right),
                             "center": (left + right) * 0.5,
                             "boundary": {
                                 "left": (left - 1, 220),
                                 "right": (right + 1, 220)}})
        self.result = {
            "valid": True, "quality": 84, "reason": "OK",
            "scene": "ROAD", "lateral_error": 0,
            "reference_y": 40, "reference_center_x": 50.0,
            "selected_row_ints": packed, "selected_row_count": 4,
            "selected_row_stride": 4, "selected_rows": selected,
            "topology_evidence": {"reason": "TEST_CLEAR"},
        }

    def detect(self, _mask_info):
        return self.result


class _BoundaryValidator:
    def __init__(self, _settings):
        return None

    @staticmethod
    def gate_fast_path_candidate(candidate, *_args):
        candidate["boundary_support_rows"] = bytearray([1] * 4)
        candidate["boundary_support_count"] = 4
        candidate["boundary_evidence"] = {
            "reason": "TEST_ACCEPTED", "bilateral_supported": 4,
            "pairs_tested": 4}
        return candidate


class _SingleEdgeTracker:
    cache_rows = 0

    def __init__(self, _settings):
        return None

    @staticmethod
    def clear(_reason):
        return None

    @staticmethod
    def observe_bilateral(*_args):
        return None

    @staticmethod
    def detect(*_args):
        return {"valid": False, "reason": "NOT_USED"}


class _RingSlopeTracker:
    def __init__(self, _settings, ring_route, slope_hint):
        self.ring_route = ring_route
        self.slope_hint = slope_hint
        self.ring = {"candidate": None, "state": "IDLE", "side_id": 0,
                     "active": False, "reason": "TEST_IDLE",
                     "pending_count": 0}
        self.slope = {"hint": slope_hint, "stage": "TEST", "reason": "TEST"}
        self.slope_update_calls = 0
        self.candidate = None
        if ring_route:
            path = array("i", [50] * HEIGHT)
            for y in range(HEIGHT):
                if y < 18:
                    path[y] = 30
            valid = bytearray(HEIGHT)
            for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
                valid[y] = 1
            self.candidate = {
                "valid": True, "side_id": 2, "side": "RIGHT",
                "reason": "TEST_CURRENT_ARC_PORT", "arc_y_max": 55,
                "near_seed_x": 49, "near_seed_y": 53,
                "near_center_x": 49.0, "near_center_y": 53,
                "heading_error": 600,
                "port_x": 77, "port_y": 10, "clip_endpoint_x": 79,
                "boundary_source": "RING_INNER_RIGHT",
                "path_x_rows": path, "path_valid_rows": valid,
            }
            self.ring.update({"candidate": self.candidate,
                              "state": "INSIDE_RIGHT", "side_id": 2,
                              "side": "RIGHT", "active": True,
                              "reason": "TEST_ACTIVE"})

    def clear(self, reason="TEST_CLEAR"):
        self.ring.update({"state": "IDLE", "side_id": 0,
                          "active": False, "reason": reason})
        self.slope["hint"] = 0

    def detect_ring_candidate(self, *_args):
        if self.candidate is not None:
            return self.candidate
        return {"valid": False, "ambiguous": False, "side_id": 0,
                "reason": "TEST_NO_RING"}

    def update_ring_state(self, *_args):
        return self.ring

    def update_slope(self, *_args):
        self.slope_update_calls += 1
        return self.slope


class FastMainRingSlopeWireTests(unittest.TestCase):
    def _run_main_fast(self, ring_route, slope_hint):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        fake_time = _Time()
        fake_sensor = _Sensor()
        fake_uart = _UART(fake_time)

        tracker_instances = []

        class TrackerFactory:
            def __new__(cls, settings):
                tracker = _RingSlopeTracker(settings, ring_route, slope_hint)
                tracker_instances.append(tracker)
                return tracker

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
                patch.object(main.fast_ring_slope, "FastRingSlope",
                             TrackerFactory), \
                patch.object(main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, road, _validator: (
                                 selection,
                                 {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=1)
        self.assertEqual(len(fake_uart.frames), 1)
        return fake_uart.frames[0], output.getvalue(), tracker_instances[0]

    def test_main_fast_ring_path_heading_and_weighted_error_reach_uart(self):
        frame, log, tracker = self._run_main_fast(True, 0)
        self.assertEqual(len(frame), 12)
        self.assertEqual(frame[6], 0x01)
        weighted_position = main._normalize_fast_reference(46.0, WIDTH)
        self.assertEqual(int.from_bytes(frame[8:10], "little", signed=True),
                         min(1000, weighted_position + 300))
        self.assertIn("source=ring_right", log)
        self.assertIn("head=+300", log)
        self.assertIn("ring=INSIDE_RIGHT", log)
        self.assertIn("ref=49.0,53.0", log)
        self.assertIn("seed=49,53", log)
        self.assertIn("slope=DISABLED:0", log)
        self.assertEqual(tracker.slope_update_calls, 0)
        self.assertLess(max(len(line.encode("utf-8")) for line in
                            log.splitlines()), 1024)
        self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:10]),
                         int.from_bytes(frame[10:12], "little"))

    def test_main_fast_never_emits_slope_hint_even_if_tracker_offers_one(self):
        frame, log, tracker = self._run_main_fast(False, 1)
        self.assertEqual(frame[6], vision_protocol.FLAG_VALID)
        self.assertTrue(any("slope=DISABLED:0" in line
                            for line in log.splitlines()
                            if line.startswith("V7DBG")))
        self.assertIn("slope_classifier=disabled slope_hint=always0", log)
        self.assertEqual(tracker.slope_update_calls, 0)
        self.assertFalse(main.SLOPE_CLASSIFIER_RUNTIME_ENABLED)
        self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:10]),
                         int.from_bytes(frame[10:12], "little"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
