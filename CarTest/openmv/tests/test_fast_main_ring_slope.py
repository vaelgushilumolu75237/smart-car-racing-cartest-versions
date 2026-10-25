"""Exercise the real _main_fast selection, aggregate, pack, and UART path."""

import contextlib
import io
import importlib.util
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
import vision_protocol
from test_ring_slope_host import (
    _genuine_ring_fixture, _ordinary_band_fixture, _ring_fixture)


WIDTH = 80
HEIGHT = 60


class _Time:
    def __init__(self, step_us=100):
        self.us = 0
        self.step_us = step_us

    def ticks_us(self):
        self.us += self.step_us
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
        self.raw = bytearray(320 * 120)

    @staticmethod
    def width():
        return 320

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
    def __init__(self, fake_time, stop_after=1, rx_data=b""):
        self.fake_time = fake_time
        self.frames = []
        self.stop_after = stop_after
        self.rx_data = bytearray(rx_data)

    def any(self):
        return len(self.rx_data)

    def readinto(self, buffer, count=None):
        if count is None:
            count = len(buffer)
        received = min(count, len(self.rx_data))
        for index in range(received):
            buffer[index] = self.rx_data[index]
        del self.rx_data[:received]
        return received

    def write(self, frame):
        self.frames.append(bytes(frame))
        # End the finite main-loop run after the selected packet count.
        if len(self.frames) >= self.stop_after:
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
                (30, 40, 60), (40, 40, 60), (55, 40, 60))
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
            "selected_row_ints": packed, "selected_row_count": 5,
            "selected_row_stride": 4, "selected_rows": selected,
            "topology_evidence": {"reason": "TEST_CLEAR"},
        }

    def detect(self, _mask_info):
        return self.result


class _OpenBandPathDetector(_PathDetector):
    def __init__(self, _settings):
        super().__init__(_settings)
        self.result.update({
            "valid": False, "quality": 0, "reason": "OPEN_BAND",
            "selected_row_ints": array("i"), "selected_row_count": 0,
            "selected_rows": (),
            "topology_evidence": {
                "reason": "OPEN_BAND", "complete": False,
                "branch_evidence": True, "island": False,
            },
        })


class _BoundaryValidator:
    def __init__(self, _settings):
        return None

    @staticmethod
    def gate_fast_path_candidate(candidate, *_args):
        row_count = candidate.get("selected_row_count", 0)
        candidate["boundary_support_rows"] = bytearray([1] * row_count)
        candidate["boundary_support_count"] = row_count
        candidate["boundary_evidence"] = {
            "reason": "TEST_ACCEPTED", "bilateral_supported": row_count,
            "pairs_tested": row_count}
        return candidate


class _SingleEdgeTracker:
    cache_rows = 0
    freeze_calls = 0
    skip_calls = 0

    def __init__(self, _settings):
        type(self).freeze_calls = 0
        type(self).skip_calls = 0
        self._row_status = bytearray(2 * HEIGHT)
        self._row_edge2 = array("i", [0] * (2 * HEIGHT))
        self._row_contrast = array("i", [0] * (2 * HEIGHT))
        self._frame_scan_done = False
        self._cache_valid = bytearray(HEIGHT)
        self._cache_width = array("i", [0] * HEIGHT)
        self._cache_left2 = array("i", [0] * HEIGHT)
        self._cache_right2 = array("i", [0] * HEIGHT)

    @staticmethod
    def clear(_reason):
        return None

    def begin_frame(self):
        return None

    def skip_frame(self, _reason="DETECT_NOT_CALLED"):
        type(self).skip_calls += 1
        return None

    def freeze_diagnostic_snapshot(self):
        type(self).freeze_calls += 1
        return True

    @staticmethod
    def observe_bilateral(*_args):
        return None

    @staticmethod
    def detect(*_args):
        return {"valid": False, "reason": "NOT_USED"}


class _ActiveRingElements:
    def __init__(self, _settings):
        self.min_target_rows = 5
        self.max_cache_gap = 3
        self.rows = array("i")
        flags = (main.track_elements.TARGET_INFERRED_LEFT |
                 main.track_elements.TARGET_OBSERVED_RIGHT |
                 main.track_elements.TARGET_WIDTH_FROM_CACHE)
        for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
            self.rows.extend((y, 25, 49, flags))
        self.result = {
            "state": "INSIDE", "reason": "TEST_RING_ACTIVE",
            "side": main.track_elements.SIDE_RIGHT,
            "control_eligible": True, "target_valid": True,
            "target_rows": self.rows, "target_count": len(self.rows) // 4,
            "target_stride": 4, "target_center2": 74,
            "target_lateral_error": -76, "observed_rows": 52,
            "inferred_rows": 52, "cross_route_usable": False,
            "cross_suppress_slowdown": False, "profile_id": 2,
            "profile_seq": 11, "session_token": 1,
        }

    def update(self, *_args, **_kwargs):
        return self.result

    def reset(self, _reason="TEST_RESET"):
        return self.result


class _FixedElementState:
    STATE_ID_NAME = "STATE_ENTRY_CONFIRM"
    STATE = "ENTRY_CONFIRM"
    RING_CONFIRMED = False
    REASON = "CONFIRMING_ONE_SIDE_LOSS"

    def __init__(self, _settings):
        self.result = {
            "state": self.STATE,
            "state_id": getattr(main.track_elements, self.STATE_ID_NAME),
            "reason": self.REASON, "candidate": True,
            "ring_confirmed": self.RING_CONFIRMED,
            "control_eligible": False, "target_valid": False,
            "target_rows": array("i"), "target_count": 0,
            "target_stride": main.track_elements.TARGET_ROW_STRIDE,
            "cross_route_usable": False, "cross_route_count": 0,
            "cross_route_stride": main.track_elements.ROAD_ROW_STRIDE,
            "cross_reason": "NO_CROSS_EVIDENCE",
            "side": main.track_elements.SIDE_RIGHT,
        }

    def update(self, *_args, **_kwargs):
        self.result["state"] = self.STATE
        self.result["state_id"] = getattr(
            main.track_elements, self.STATE_ID_NAME)
        self.result["reason"] = self.REASON
        self.result["ring_confirmed"] = self.RING_CONFIRMED
        return self.result

    def reset(self, _reason="TEST_RESET"):
        return self.result


class _ConfirmedRingNoTarget(_FixedElementState):
    STATE_ID_NAME = "STATE_EXIT_CONFIRM"
    STATE = "EXIT_CONFIRM"
    RING_CONFIRMED = True
    REASON = "EXIT_EXPERIMENTAL_CONFIRM"

    def reset(self, _reason="TEST_RESET"):
        return None


class _CrossStraightElements(_FixedElementState):
    STATE_ID_NAME = "STATE_OBSERVE"
    STATE = "OBSERVE"
    RING_CONFIRMED = False
    REASON = "RING_DISABLED_PROFILE_A"

    def __init__(self, settings):
        super().__init__(settings)
        rows = array("i")
        for y in range(settings.SURFACE_ROI_FLOOR_Y, HEIGHT):
            rows.extend((y, 20, 54,
                         main.track_elements.CROSS_ROUTE_MEASURED))
        self.result.update({
            "cross_route_usable": True, "cross_route_rows": rows,
            "cross_route_count": len(rows) //
            main.track_elements.ROAD_ROW_STRIDE,
            "cross_route_stride": main.track_elements.ROAD_ROW_STRIDE,
            "cross_reference_center2": 74, "cross_reference_y": 40,
            "cross_lateral_error": -63, "cross_near_rows": 5,
            "cross_far_rows": 5, "cross_suppress_slowdown": True,
            "cross_reason": "CROSS_STRAIGHT_CONFIRMED",
            "profile_id": config.VEHICLE_PROFILE_A, "profile_seq": 11,
            "session_token": 1,
        })


class _RingSlopeTracker:
    def __init__(self, _settings, ring_route, slope_hint):
        self.ring_route = ring_route
        self.slope_hint = slope_hint
        self.ring = {"candidate": None, "state": "IDLE", "side_id": 0,
                     "active": False, "reason": "TEST_IDLE",
                     "pending_count": 0,
                     "route_authorized": False,
                     "current_geometry_ok": False}
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
                "entry_geometry_ok": True,
                "current_geometry_ok": True,
                "route_authorized": True,
                "path_row_count": sum(valid),
                "path_y_span": 51,
                "path_near_anchor": True,
                "port_segments": 1,
                "port_x": 77, "port_y": 10, "clip_endpoint_x": 79,
                "boundary_source": "RING_INNER_RIGHT",
                "path_x_rows": path, "path_valid_rows": valid,
            }
            self.ring.update({"candidate": self.candidate,
                              "state": "INSIDE_RIGHT", "side_id": 2,
                              "side": "RIGHT", "active": True,
                              "reason": "TEST_ACTIVE",
                              "route_authorized": True,
                              "current_geometry_ok": True})

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
    def _run_profile_element_case(self, element_type, profile_id,
                                  path_type=_PathDetector,
                                  clock_step_us=100):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        fake_time = _Time(clock_step_us)
        profile_bytes = (
            bytes(main.reverse_control.pack_frame(0, 100)) +
            bytes(main.reverse_control.pack_frame(1, 101)) +
            bytes(main.reverse_control.pack_profile_state(
                11, profile_id, main.reverse_control.PROFILE_SESSION_ACTIVE,
                session_token=1)))
        fake_uart = _UART(fake_time, rx_data=profile_bytes)
        output = io.StringIO()
        cache_commits = []

        class _RecordingSingleEdgeTracker(_SingleEdgeTracker):
            def observe_bilateral(self, *args):
                cache_commits.append(args)

        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", _Sensor()), \
                patch.object(main, "time", fake_time), \
                patch.object(main.fast_gray, "FastGrayMasks", _GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector", path_type), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             _BoundaryValidator), \
                patch.object(main.fast_single_edge, "FastSingleEdgeTracker",
                             _RecordingSingleEdgeTracker), \
                patch.object(main.track_elements, "TrackElementsTracker",
                             element_type), \
                patch.object(main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, _road, _validator: (
                                 selection,
                                 {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=1)
        self.assertEqual(len(fake_uart.frames), 1)
        return fake_uart.frames[0], output.getvalue(), cache_commits

    def _run_main_fast(self, ring_route, slope_hint):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        fake_time = _Time()
        fake_sensor = _Sensor()
        fake_uart = _UART(fake_time)

        tracker_instances = []

        tracker_instances.append(
            _RingSlopeTracker(config, ring_route, slope_hint))

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

    def _run_fixture_sequence(self, runtime_main, settings, fixture,
                              stop_after):
        runtime_main._load_fast_runtime()
        runtime_main._load_singleedge_runtime()
        masks, road_result, _support = fixture
        fake_time = _Time()
        fake_sensor = _Sensor()
        fake_uart = _UART(fake_time, stop_after)

        class FixtureGrayBuilder:
            def __init__(self, _settings):
                return None

            def build(self, _image):
                result = dict(masks)
                result.update({
                    "luma": bytearray(WIDTH * HEIGHT),
                    "trusted_block_count": 4,
                    "threshold_min": 10,
                    "threshold_max": 20,
                    "reason": "HOST_FIXTURE",
                })
                return result

        class FixturePathDetector:
            def __init__(self, _settings):
                return None

            def detect(self, _mask_info):
                return road_result

        class FixtureBoundaryValidator:
            def __init__(self, _settings):
                return None

            @staticmethod
            def gate_fast_path_candidate(candidate, *_args):
                count = candidate.get("selected_row_count", 0)
                candidate["boundary_support_rows"] = bytearray([1] * count)
                candidate["boundary_support_count"] = count
                candidate["boundary_evidence"] = {
                    "reason": "HOST_FIXTURE", "bilateral_supported": count,
                    "pairs_tested": count,
                }
                return candidate

        output = io.StringIO()
        with patch.object(runtime_main, "_configure_gray_sensor",
                          return_value=1), \
                patch.object(runtime_main, "sensor", fake_sensor), \
                patch.object(runtime_main, "time", fake_time), \
                patch.object(runtime_main, "_service_reverse_control",
                             return_value=True), \
                patch.object(runtime_main.fast_gray, "FastGrayMasks",
                             FixtureGrayBuilder), \
                patch.object(runtime_main.fast_path, "FastPathDetector",
                             FixturePathDetector), \
                patch.object(runtime_main.fast_boundary,
                             "FastBoundaryEvidence",
                             FixtureBoundaryValidator), \
                patch.object(runtime_main.fast_single_edge,
                             "FastSingleEdgeTracker", _SingleEdgeTracker), \
                patch.object(runtime_main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(runtime_main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, _road, _validator: (
                                 selection,
                                 {"x": 40.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "HOST_VALID"})), \
                patch.object(settings, "DEBUG_ENABLED", True), \
                patch.object(settings, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            runtime_main._main_fast(duration_s=1)
        self.assertEqual(len(fake_uart.frames), stop_after)
        return fake_uart.frames, output.getvalue()

    def _load_baseline_main(self):
        # Keep the exact pre-ring 17-file runtime beside the host tests so
        # public checkpoints can reproduce the ordinary-route UART oracle
        # without relying on a private test-data archive.
        baseline_dir = TEST_DIR / "frozen_pre_ring"
        names = ("config", "vision_protocol", "fast_gray", "fast_path",
                 "fast_topology", "fast_boundary", "fast_single_edge",
                 "reverse_control", "fast_ring_slope", "main")
        saved_modules = {name: sys.modules.get(name) for name in names}
        for name in names:
            sys.modules.pop(name, None)
        sys.path.insert(0, str(baseline_dir))
        try:
            spec = importlib.util.spec_from_file_location(
                "main", baseline_dir / "main.py")
            baseline_main = importlib.util.module_from_spec(spec)
            sys.modules["main"] = baseline_main
            spec.loader.exec_module(baseline_main)
            return baseline_dir, names, saved_modules, baseline_main
        except Exception:
            sys.path.remove(str(baseline_dir))
            for name in names:
                sys.modules.pop(name, None)
            for name, module in saved_modules.items():
                if module is not None:
                    sys.modules[name] = module
            raise

    def _restore_current_modules(self, baseline_dir, names, saved_modules):
        sys.path.remove(str(baseline_dir))
        for name in names:
            sys.modules.pop(name, None)
        for name, module in saved_modules.items():
            if module is not None:
                sys.modules[name] = module

    @unittest.skip("independent ring route was removed from production")
    def test_main_fast_ring_path_uses_current_rows_without_heading_kick(self):
        frame, log, tracker = self._run_main_fast(True, 0)
        self.assertEqual(len(frame), 16)
        self.assertEqual(frame[6], 0x01)
        packet_command = int.from_bytes(frame[8:10], "little", signed=True)
        control_line = next(line for line in log.splitlines()
                            if line.startswith("V7CTRL "))
        control = dict(token.split("=", 1) for token in control_line.split()[1:]
                       if "=" in token)
        self.assertEqual(int(control["preview"].split(":", 1)[1]), 0)
        self.assertEqual(packet_command, int(control["tx_cmd"]))
        self.assertEqual(packet_command, int(control["pd"]))
        self.assertEqual(
            packet_command,
            max(-1000, min(1000,
                           int(control["p"]) +
                           int(control["d_component"]))))
        self.assertIn("source=ring_right", log)
        self.assertIn("preview=+0", log)
        self.assertIn("preview_candidate=+0", log)
        self.assertIn("preview_block=-", log)
        self.assertIn("reference_error=-", log)
        self.assertIn("raw_pos=", log)
        self.assertIn("pos_ema=", log)
        self.assertIn("tx_cmd=%d" % packet_command, log)
        self.assertIn("auth=1 geo=1", log)
        self.assertIn("ring=INSIDE_RIGHT", log)
        self.assertIn("ref=49.0,53.0", log)
        self.assertIn("seed=49,53", log)
        self.assertIn("slope=DISABLED:0", log)
        self.assertEqual(tracker.slope_update_calls, 0)
        self.assertLess(max(len(line.encode("utf-8")) for line in
                            log.splitlines()), 1024)
        self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:14]),
                         int.from_bytes(frame[14:16], "little"))

    def test_main_fast_never_emits_slope_hint_even_if_tracker_offers_one(self):
        frame, log, tracker = self._run_main_fast(False, 1)
        self.assertEqual(frame[6], vision_protocol.FLAG_VALID)
        self.assertTrue(any("slope=DISABLED:0" in line
                            for line in log.splitlines()
                            if line.startswith("V7DBG")))
        self.assertIn("slope_classifier=disabled slope_hint=always0", log)
        self.assertEqual(tracker.slope_update_calls, 0)
        self.assertFalse(main.SLOPE_CLASSIFIER_RUNTIME_ENABLED)
        self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:14]),
                         int.from_bytes(frame[14:16], "little"))

    @unittest.skip("ring-entry routing is intentionally absent")
    def test_real_detector_left_and_right_entry_reaches_uart_after_three_frames(self):
        for side, source in ((1, "ring_left"), (2, "ring_right")):
            with self.subTest(side=side):
                fixture = _genuine_ring_fixture(side)
                frames, log = self._run_fixture_sequence(
                    main, config, fixture, stop_after=3)
                self.assertEqual(len(frames), 3)
                self.assertTrue(all(len(frame) == 12 for frame in frames))
                self.assertEqual(frames[-1][6] & vision_protocol.FLAG_VALID,
                                 vision_protocol.FLAG_VALID)
                self.assertIn("source=" + source, log)
                self.assertIn("auth=1 geo=1", log)
                self.assertIn("gate=ARC_AND_CONNECTED_PORT", log)
                self.assertIn("preview=+0", log)
                for frame in frames:
                    self.assertEqual(
                        vision_protocol.crc16_ccitt_false(frame[2:10]),
                        int.from_bytes(frame[10:12], "little"))

    @unittest.skip("independent ring-curvature routing is intentionally absent")
    def test_independent_curvature_entry_reaches_uart_after_three_frames(self):
        for side, source in ((1, "ring_left"), (2, "ring_right")):
            with self.subTest(side=side):
                fixture = _ordinary_band_fixture(
                    side, "u_curve", inner_scale=1.2, outer_scale=0.3,
                    outer_sign=-1.0)
                frames, log = self._run_fixture_sequence(
                    main, config, fixture, stop_after=3)
                self.assertEqual(len(frames), 3)
                self.assertTrue(all(len(frame) == 12 for frame in frames))
                self.assertEqual(frames[-1][6] & vision_protocol.FLAG_VALID,
                                 vision_protocol.FLAG_VALID)
                self.assertIn("source=" + source, log)
                self.assertIn("fit=INDEPENDENT_CURVATURE", log)
                self.assertIn("auth=1 geo=1", log)
                self.assertIn("gate=ARC_AND_CONNECTED_PORT", log)
                for frame in frames:
                    self.assertEqual(
                        vision_protocol.crc16_ccitt_false(frame[2:10]),
                        int.from_bytes(frame[10:12], "little"))

    def test_ordinary_profiles_keep_road_authority_with_preview(self):
        fixture = _ordinary_band_fixture(1, "straight")
        frames, log = self._run_fixture_sequence(
            main, config, fixture, stop_after=3)
        self.assertEqual(len(frames), 3)
        self.assertIn("source=road", log)
        self.assertIn("ring_route=profile_gated_experimental", log)
        for frame in frames:
            self.assertEqual(len(frame), 16)
            self.assertEqual(frame[2:4], b"\x03\x08")
            self.assertEqual(frame[11], 0)
            self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:14]),
                             int.from_bytes(frame[14:16], "little"))

    def test_fresh_profile_b_ring_target_reaches_the_existing_uart_chain(self):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        fake_time = _Time()
        profile_bytes = (
            bytes(main.reverse_control.pack_frame(0, 100)) +
            bytes(main.reverse_control.pack_frame(1, 101)) +
            bytes(main.reverse_control.pack_profile_state(
                11, main.reverse_control.PROFILE_B,
                main.reverse_control.PROFILE_SESSION_ACTIVE,
                session_token=1)))
        fake_uart = _UART(fake_time, rx_data=profile_bytes)
        output = io.StringIO()
        cache_commits = []
        weighted_routes = []

        class _RecordingSingleEdgeTracker(_SingleEdgeTracker):
            def observe_bilateral(self, *args):
                cache_commits.append(args)

        class _RecordingWeightWorkspace(main._FastRowWeightWorkspace):
            def aggregate(self, selection, road_result, settings):
                info = super().aggregate(selection, road_result, settings)
                if selection.get("source") in ("ring_left", "ring_right"):
                    rows = selection["result"]["selected_row_ints"]
                    count = selection["result"]["selected_row_count"]
                    weighted_routes.append((
                        info.copy(), selection.get("source"),
                        selection.get("boundary_source"),
                        array("i", rows[:count *
                                       main.fast_single_edge.ROW_STRIDE])))
                return info

        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", _Sensor()), \
                patch.object(main, "time", fake_time), \
                patch.object(main.fast_gray, "FastGrayMasks", _GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector", _PathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             _BoundaryValidator), \
                patch.object(main.fast_single_edge, "FastSingleEdgeTracker",
                             _RecordingSingleEdgeTracker), \
                patch.object(main.track_elements, "TrackElementsTracker",
                             _ActiveRingElements), \
                patch.object(main, "_FastRowWeightWorkspace",
                             _RecordingWeightWorkspace), \
                patch.object(main.pyb, "UART", return_value=fake_uart,
                             create=True), \
                patch.object(main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, _road, _validator: (
                                 selection,
                                 {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(config, "DEBUG_ENABLED", True), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=1)
        self.assertEqual(len(fake_uart.frames), 1)
        frame = fake_uart.frames[0]
        self.assertEqual(frame[2:4], b"\x03\x08")
        self.assertEqual(frame[11], main.reverse_control.PROFILE_B)
        self.assertEqual(frame[6] & vision_protocol.FLAG_VALID,
                         vision_protocol.FLAG_VALID)
        self.assertEqual(frame[10], vision_protocol.CURVE_PHASE_TURN)
        self.assertEqual(int.from_bytes(frame[12:14], "little"), 1000)
        self.assertIn("source=ring_right", output.getvalue())
        self.assertIn(
            "TRACK_BUILD=CARTEST_H750_TRACK_PROFILES_V1_20261025 "
            "forward=V3/16 reverse=V1/8+V2/20 profile=awaiting_state",
            output.getvalue())
        self.assertEqual(cache_commits, [])
        self.assertEqual(len(weighted_routes), 1)
        weight_info, source, boundary_source, ring_rows = weighted_routes[0]
        self.assertTrue(weight_info["valid"], weight_info)
        self.assertEqual(source, "ring_right")
        self.assertEqual(boundary_source, "RING_WIDTH_CACHE_RIGHT")
        self.assertLess(weight_info["position_error"], 0)
        self.assertLess(int.from_bytes(frame[8:10], "little", signed=True), 0)
        self.assertGreater(
            weight_info["position_error"] *
            int.from_bytes(frame[8:10], "little", signed=True), 0)
        self.assertEqual(
            ring_rows[3], main.track_elements.SIDE_RIGHT)
        self.assertTrue(
            ring_rows[4] & main.fast_single_edge.FLAG_OBSERVED_RIGHT)
        self.assertTrue(
            ring_rows[4] & main.fast_single_edge.FLAG_INFERRED_LEFT)
        self.assertFalse(
            ring_rows[4] & main.fast_single_edge.FLAG_OBSERVED_LEFT)
        self.assertEqual(vision_protocol.crc16_ccitt_false(frame[2:14]),
                         int.from_bytes(frame[14:16], "little"))

    def test_confirmed_ring_without_a_target_is_sent_invalid(self):
        frame, log, cache_commits = self._run_profile_element_case(
            _ConfirmedRingNoTarget, config.VEHICLE_PROFILE_B)
        self.assertEqual(frame[11], main.reverse_control.PROFILE_B)
        self.assertEqual(frame[6] & vision_protocol.FLAG_VALID, 0)
        self.assertEqual(frame[8:10], b"\x00\x00")
        self.assertEqual(frame[10], vision_protocol.CURVE_PHASE_STRAIGHT)
        self.assertEqual(int.from_bytes(frame[12:14], "little"), 0)
        self.assertIn("RING_TARGET_UNSUPPORTED", log)
        self.assertEqual(cache_commits, [])

    def test_unconfirmed_ring_observation_keeps_the_normal_road_route(self):
        frame, log, _cache_commits = self._run_profile_element_case(
            _FixedElementState, config.VEHICLE_PROFILE_B)
        self.assertEqual(frame[11], main.reverse_control.PROFILE_B)
        self.assertEqual(frame[6] & vision_protocol.FLAG_VALID,
                         vision_protocol.FLAG_VALID)
        self.assertIn("source=road", log)
        self.assertIn("ring=ENTRY_CONFIRM:CONFIRMING_ONE_SIDE_LOSS", log)
        self.assertNotIn("RING_TARGET_UNSUPPORTED", log)

    def test_profile_a_cross_straight_uses_the_validated_row_weight_path(self):
        frame, log, cache_commits = self._run_profile_element_case(
            _CrossStraightElements, config.VEHICLE_PROFILE_A,
            _OpenBandPathDetector, clock_step_us=1)
        self.assertEqual(frame[11], main.reverse_control.PROFILE_A)
        self.assertEqual(frame[6] & vision_protocol.FLAG_VALID,
                         vision_protocol.FLAG_VALID)
        self.assertEqual(frame[10], vision_protocol.CURVE_PHASE_STRAIGHT)
        self.assertEqual(int.from_bytes(frame[12:14], "little"), 0)
        self.assertIn("source=road b=CROSS_STRAIGHT", log)
        self.assertIn("b=CROSS_STRAIGHT", log)
        self.assertIn("wctrl=37.0", log)
        self.assertIn("pos_weighted=-63", log)
        self.assertEqual(cache_commits, [])

    def test_ring_target_uses_the_existing_weight_path_without_legacy_loader(self):
        rows = array("i")
        for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
            rows.extend((y, 60 + y // 8, 30, 7, 1))
        result = {
            "selected_row_ints": rows,
            "selected_row_count": len(rows) // 5,
            "selected_row_stride": 5,
        }
        info = main._FastRowWeightWorkspace().aggregate(
            {"valid": True, "source": "ring_left", "result": result},
            {}, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["source"], "ring_left")
        self.assertFalse(hasattr(main, "fast_ring_slope"))
        runtime = Path(main.__file__).read_text(encoding="utf-8")
        self.assertNotIn("FastRingSlope", runtime)
        self.assertNotIn("detect_ring_candidate", runtime)
        self.assertNotIn("_aggregate_legacy", runtime)
        self.assertEqual(main._steering_diag_source_code("ring_right"), 5)

    def test_overlay_status_includes_current_profile_ring_state_and_reason(self):
        overlay_state = main._V7OverlayState(0)
        status = main._v7_overlay_status(
            overlay_state,
            {"source": "ring_right", "track_profile_label": "B",
             "track_ring_state": "INSIDE",
             "track_ring_reason": "RING_INSIDE_EXIT_UNOBSERVED"},
            True)
        self.assertEqual(status, "TRK B INSIDE:INSIDE")

    def test_ring_support_side_change_resets_pd_identity_without_derivative(self):
        pd = main._FastPositionPD(config)
        ticks_diff = lambda new, old: new - old
        pd.update(10, 10, True, -60, "ring_right",
                  "RING_WIDTH_CACHE_RIGHT",
                  "reference_y_ring_width_cache_right", 40,
                  config.STEERING_CONTROL_PROFILE, ticks_diff)
        pd.update(20, 10, True, -55, "ring_right",
                  "RING_WIDTH_CACHE_RIGHT",
                  "reference_y_ring_width_cache_right", 40,
                  config.STEERING_CONTROL_PROFILE, ticks_diff)
        changed = pd.update(
            30, 10, True, -50, "ring_left", "RING_WIDTH_CACHE_LEFT",
            "reference_y_ring_width_cache_left", 40,
            config.STEERING_CONTROL_PROFILE, ticks_diff)
        self.assertEqual(
            changed["reset_code"], main._CONTROL_RESET_CODES["IDENTITY"])
        self.assertEqual(changed["derivative_d"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
