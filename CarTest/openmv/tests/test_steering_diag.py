"""Host checks for the optional fixed-size OpenMV steering trace."""

import ast
import contextlib
import io
import subprocess
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


OPENMV_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = OPENMV_DIR.parent
BASELINE_DIR = (PROJECT_DIR / "test-data" /
                "steering-hold3s-20261016" / "baseline" / "openmv")
TEST_DIR = OPENMV_DIR / "tests"
for _path in (OPENMV_DIR, TEST_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import main
import vision_protocol
import test_fast_main_ring_slope as main_harness


def _frame(sequence, valid, error=0):
    return vision_protocol.pack_frame(sequence, valid, 60 if valid else 0,
                                      error if valid else 0, 0)


class SteeringDiagBufferTests(unittest.TestCase):
    def setUp(self):
        self.diag = main._SteeringDiagBuffer()

    def test_default_is_enabled_with_exact_preallocated_ring(self):
        self.assertIs(config.STEERING_DIAG_ENABLED, True)
        self.assertIsInstance(main._STEERING_DIAG, main._SteeringDiagBuffer)
        self.assertEqual(len(main._STEERING_DIAG.data), 128 * 32)

    def test_disabled_factory_does_not_allocate_a_buffer(self):
        with patch.object(main, "_SteeringDiagBuffer") as constructor:
            self.assertIsNone(main._make_steering_diag(False))
        constructor.assert_not_called()

    def test_disabled_config_import_keeps_module_buffer_unallocated(self):
        openmv_path = str(OPENMV_DIR)
        script = (
            "import sys, types; sys.modules['pyb'] = types.ModuleType('pyb'); "
            "sys.modules['sensor'] = types.ModuleType('sensor'); "
            "sys.path.insert(0, %r); import config; "
            "config.STEERING_DIAG_ENABLED = False; import main; "
            "assert main._STEERING_DIAG is None; "
            "assert main._STEERING_DIAG_TICKS_PERIOD_MS == 0; "
            "print('disabled buffer=none')" % openmv_path)
        completed = subprocess.run(
            [sys.executable, "-B", "-c", script],
            cwd=str(PROJECT_DIR), capture_output=True, text=True,
            check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("disabled buffer=none", completed.stdout)

    def test_record_uses_actual_packet_bytes_and_fixed_little_endian_layout(self):
        period = 1 << 30
        frame = _frame(0xFFFF, 1, -1000)
        recorded = self.diag.record(
            frame, True, False, True, period - 10, 5,
            -1000, -100, 75, -925, -900, 250, 255, 5, 0xFFFFFFFF)
        self.assertTrue(recorded)
        data = self.diag.data
        self.assertEqual(len(data), 128 * 32)
        self.assertEqual(data[0:4], bytes((0xFF, 0xFF, 0x0B, 0)))
        self.assertEqual(data[4:8], bytes((0xF6, 0xFF, 0xFF, 0x3F)))
        self.assertEqual(data[8:12], bytes((5, 0, 0, 0)))
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 12), -1000)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 14), -100)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 16), 75)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 18), -1000)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 20), -925)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 22), -900)
        self.assertEqual(main._SteeringDiagBuffer._i16(data, 24), 250)
        self.assertEqual(data[26:28], bytes((255, 5)))
        self.assertEqual(main._SteeringDiagBuffer._u32(data, 28), 0xFFFFFFFF)
        self.assertEqual(self.diag.record_errors, 0)

    def test_first_invalid_does_not_freeze_and_write_errors_do_not_advance(self):
        self.diag.record(_frame(0, 0), True, False, False,
                         1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.assertFalse(self.diag.frozen)
        self.diag.record(_frame(1, 1, 100), False, True, True,
                         3, 4, 20, 35, 35, 55, 50, 10, 9, 1, 7)
        self.assertFalse(self.diag.frozen)
        self.assertFalse(self.diag.last_full_valid)
        self.diag.record(_frame(2, 1, -25), True, False, True,
                         5, 6, 30, 40, 20, 50, 48, 11, 8, 2, 8)
        self.assertTrue(self.diag.last_full_valid)
        self.diag.record(_frame(3, 0), False, True, False,
                         7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 9)
        self.assertFalse(self.diag.frozen)
        self.assertTrue(self.diag.last_full_valid)
        self.diag.record(_frame(4, 0), True, False, False,
                         9, 10, 0, 0, 0, 0, 0, 0, 0, 0, 10)
        self.assertTrue(self.diag.frozen)
        self.assertFalse(self.diag.last_full_valid)
        self.assertEqual(self.diag.count, 5)
        self.assertEqual(self.diag.data[4 * 32 + 2] & 0x07, 0x02)
        self.assertFalse(self.diag.record(
            _frame(5, 1), True, False, True, 11, 12,
            0, 0, 0, 0, 0, 0, 0, 0, 11))
        self.assertEqual(self.diag.count, 5)

    def test_ring_wrap_keeps_newest_128_in_oldest_to_newest_order(self):
        for sequence in range(130):
            self.diag.record(_frame(sequence, 1, sequence % 1000),
                             True, False, False,
                             sequence, sequence + 1, sequence,
                             0, 0, sequence, sequence, 0,
                             sequence % 256, 1, sequence)
        self.assertEqual(self.diag.count, 128)
        self.assertEqual(self.diag.overwritten, 2)
        self.assertEqual(self.diag.write_index, 2)
        oldest = self.diag.write_index
        self.assertEqual(main._SteeringDiagBuffer._u16(
            self.diag.data, oldest * 32), 2)
        output = io.StringIO()
        with patch.object(main, "_STEERING_DIAG_TICKS_PERIOD_MS", 1 << 30), \
                contextlib.redirect_stdout(output):
            self.diag.dump(config.BUILD_ID)
        lines = output.getvalue().splitlines()
        self.assertIn("count=128 frozen=0 overwritten=2 record_errors=0",
                      lines[0])
        self.assertIn("ticks_period_ms=1073741824", lines[0])
        self.assertEqual(lines[1], self.diag._CSV_HEADER)
        self.assertTrue(lines[2].startswith("2,"))
        self.assertTrue(lines[-2].startswith("129,"))
        self.assertEqual(lines[-1], "STEERING_DIAG_END")

    def test_rearm_clears_metadata_without_replacing_the_buffer(self):
        self.diag.record(_frame(1, 1), True, False, False,
                         1, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0)
        self.diag.record(_frame(2, 0), True, False, False,
                         2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.assertTrue(self.diag.frozen)
        old_buffer = self.diag.data
        self.diag.record_errors = 4
        self.diag.rearm()
        self.assertIs(self.diag.data, old_buffer)
        self.assertEqual((self.diag.write_index, self.diag.count,
                          self.diag.overwritten,
                          self.diag.record_errors), (0, 0, 0, 0))
        self.assertFalse(self.diag.frozen)
        self.assertFalse(self.diag.last_full_valid)

    def test_ticks_period_reads_microPython_period_or_returns_unknown_zero(self):
        period = 1 << 30

        class FakeTime:
            @staticmethod
            def ticks_add(value, delta):
                return (value + delta) % period

        self.assertEqual(main._ticks_period_ms(FakeTime), period)
        self.assertEqual(main._ticks_period_ms(object()), 0)

    def test_disabled_dump_is_explicit(self):
        output = io.StringIO()
        with patch.object(main, "_STEERING_DIAG", None), \
                contextlib.redirect_stdout(output):
            self.assertFalse(main.steering_diag_dump())
        self.assertIn("STEERING_DIAG_DISABLED", output.getvalue())

    def test_real_main_continues_detection_and_sending_after_diag_freeze(self):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        fake_time = main_harness._Time()
        fake_sensor = main_harness._Sensor()
        diag = main._SteeringDiagBuffer()
        uart = _ThreeFrameUART(fake_time)
        detector_instances = []
        row_weight_outputs = []
        original_aggregate = main._FastRowWeightWorkspace.aggregate

        class AlternatingPathDetector:
            def __init__(self, settings):
                self.inner = main_harness._PathDetector(settings)
                self.calls = 0
                detector_instances.append(self)

            def detect(self, mask_info):
                self.calls += 1
                if self.calls == 2:
                    return {"valid": False, "quality": 0,
                            "reason": "TEST_INVALID"}
                return self.inner.detect(mask_info)

        def capture_weight_output(workspace, *args, **kwargs):
            info = original_aggregate(workspace, *args, **kwargs)
            row_weight_outputs.append((
                info["position_error"], info["preview_candidate"],
                info["heading_contribution"], info["steering_error"]))
            return info

        output = io.StringIO()
        def force_preview_candidate(workspace, *_args):
            workspace.info["preview_candidate"] = 400
            workspace.info["preview_valid"] = True
        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", fake_sensor), \
                patch.object(main, "time", fake_time), \
                patch.object(main, "_STEERING_DIAG", diag), \
                patch.object(main, "_service_reverse_control",
                             return_value=True), \
                patch.object(main.fast_gray, "FastGrayMasks",
                             main_harness._GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             AlternatingPathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             main_harness._BoundaryValidator), \
                patch.object(main.fast_single_edge,
                             "FastSingleEdgeTracker",
                             main_harness._SingleEdgeTracker), \
                patch.object(main.fast_ring_slope, "FastRingSlope",
                             lambda settings: main_harness._RingSlopeTracker(
                                 settings, ring_route=False,
                                 slope_hint=0)), \
                patch.object(main._FastRowWeightWorkspace, "aggregate",
                             new=capture_weight_output), \
                patch.object(main._FastRowWeightWorkspace,
                             "_compute_road_preview",
                             new=force_preview_candidate), \
                patch.object(main.pyb, "UART", return_value=uart,
                             create=True), \
                patch.object(main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, _road, _validator: (
                                 selection,
                                 {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(config, "DEBUG_ENABLED", False), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=1)

        self.assertEqual(detector_instances[0].calls, 3)
        self.assertEqual(len(uart.frames), 3)
        self.assertEqual([len(frame) for frame in uart.frames], [12, 12, 12])
        self.assertEqual([frame[4] for frame in uart.frames], [0, 1, 2])
        self.assertEqual([frame[6] & vision_protocol.FLAG_VALID
                          for frame in uart.frames], [1, 0, 1])
        self.assertTrue(diag.frozen)
        self.assertEqual(diag.count, 2)
        self.assertEqual(diag.record_errors, 0)
        self.assertEqual(main_harness._SingleEdgeTracker.freeze_calls, 1)
        self.assertGreaterEqual(len(row_weight_outputs), 3)
        self.assertTrue(all(applied == 0 and steering == position
                            for position, _candidate, applied, steering
                            in row_weight_outputs))
        first = diag.data
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 12),
                         row_weight_outputs[0][0])
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 14),
                         row_weight_outputs[0][1])
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 16),
                         row_weight_outputs[0][2])
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 20),
                         row_weight_outputs[0][3])
        packet_filtered_error = uart.frames[0][8] | (uart.frames[0][9] << 8)
        if packet_filtered_error & 0x8000:
            packet_filtered_error -= 0x10000
        # The first accepted observation initializes the existing filter from
        # the position-only raw control. Check the actual UART payload as well
        # as the RAM trace fields populated from that same packet.
        self.assertEqual(packet_filtered_error, row_weight_outputs[0][3])
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 18),
                         packet_filtered_error)
        self.assertEqual(main._SteeringDiagBuffer._i16(first, 22),
                         packet_filtered_error)

    def test_real_main_counts_diag_failure_and_keeps_sending(self):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        fake_time = main_harness._Time()
        fake_sensor = main_harness._Sensor()
        diag = main._SteeringDiagBuffer()
        uart = _ThreeFrameUART(fake_time, stop_after=1)
        output = io.StringIO()
        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", fake_sensor), \
                patch.object(main, "time", fake_time), \
                patch.object(main, "_STEERING_DIAG", diag), \
                patch.object(diag, "record",
                             side_effect=RuntimeError("mock trace failure")), \
                patch.object(main, "_service_reverse_control",
                             return_value=True), \
                patch.object(main.fast_gray, "FastGrayMasks",
                             main_harness._GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             main_harness._PathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             main_harness._BoundaryValidator), \
                patch.object(main.fast_single_edge,
                             "FastSingleEdgeTracker",
                             main_harness._SingleEdgeTracker), \
                patch.object(main.fast_ring_slope, "FastRingSlope",
                             lambda settings: main_harness._RingSlopeTracker(
                                 settings, ring_route=False,
                                 slope_hint=0)), \
                patch.object(main.pyb, "UART", return_value=uart,
                             create=True), \
                patch.object(main, "_validate_selected_fast_boundary",
                             side_effect=lambda selection, _road, _validator: (
                                 selection,
                                 {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                                 {"valid": True, "reason": "TEST_VALID"})), \
                patch.object(config, "DEBUG_ENABLED", False), \
                patch.object(config, "OVERLAY_ENABLED", False), \
                contextlib.redirect_stdout(output):
            main._main_fast(duration_s=1)
            self.assertTrue(main.steering_diag_dump())

        self.assertEqual(len(uart.frames), 1)
        self.assertEqual(len(uart.frames[0]), 12)
        self.assertTrue(uart.frames[0][6] & vision_protocol.FLAG_VALID)
        self.assertEqual(diag.record_errors, 1)
        self.assertEqual(diag.count, 0)
        self.assertIn("record_errors=1", output.getvalue())


class SteeringDiagAlgorithmFreezeTests(unittest.TestCase):
    @staticmethod
    def _normalized_source(source, baseline=False):
        if baseline:
            return source
        source = source.replace(
            "AGGRESSIVE_TURN_POS_ONLY_20261017",
            "CONTOUR_ARRAYFIX_20261016")
        extension_start = source.index("# SINGLE_EDGE_EXT_DIAG_BEGIN")
        extension_end = source.index("# SINGLE_EDGE_EXT_DIAG_END",
                                     extension_start)
        extension_end = source.index("\n", extension_end) + 1
        source = source[:extension_start] + source[extension_end:]
        source = source.replace(
            "        single_edge_tracker.begin_frame()\n", "")
        source = source.replace(
            "        if single_candidate is None:\n"
            "            single_edge_tracker.skip_frame(single_diagnostic_reason)\n",
            "")
        helper_start = source.index("class _SteeringDiagBuffer:")
        helper_end = source.index("_V6_RUNTIME_MODULES = ()", helper_start)
        source = source[:helper_start] + source[helper_end:]
        block_start = source.index("        # STEERING_DIAG_TRACE_BEGIN")
        block_end = source.index("        # STEERING_DIAG_TRACE_END", block_start)
        block_end = source.index("\n", block_end) + 1
        source = source[:block_start] + source[block_end:]
        source = source.replace(
            '    "pack_uart", "capture_to_send", "diag_trace", "overlay",\n',
            '    "pack_uart", "capture_to_send", "overlay",\n')
        source = source.replace(
            '        perf.record("diag_trace", diag_trace_us)\n', '')
        source = source.replace(
            '    if not isinstance(getattr(config, "STEERING_DIAG_ENABLED", None), bool):\n'
            '        raise ValueError("V7 STEERING_DIAG_ENABLED must be bool")\n',
            '')
        return source

    @staticmethod
    def _normalize_position_only_changes(current, baseline):
        """Remove the intentional profile/diagnostic delta before AST compare."""
        current = current.replace(
            '                _compose_ordinary_steering_error(\n'
            '                    info["position_error"], info["preview_candidate"],\n'
            '                    getattr(settings, "ORDINARY_PREVIEW_CONTROL_ENABLED", False))',
            '                _compose_ordinary_steering_error(\n'
            '                    info["position_error"], info["preview_candidate"])')
        current = current.replace(
            '    if not isinstance(getattr(config, "ORDINARY_PREVIEW_CONTROL_ENABLED", None),\n'
            '                      bool):\n'
            '        raise ValueError(\n'
            '            "V7 ORDINARY_PREVIEW_CONTROL_ENABLED must be bool")\n',
            '')

        def replace_function(source, reference, name):
            marker = "def " + name + "("
            start = source.index(marker)
            end = source.find("\ndef ", start + len(marker))
            if end < 0:
                end = len(source)
            ref_start = reference.index(marker)
            ref_end = reference.find("\ndef ", ref_start + len(marker))
            if ref_end < 0:
                ref_end = len(reference)
            return source[:start] + reference[ref_start:ref_end] + source[end:]

        # The helper and overlay are the two intentionally changed functions;
        # their behavior has focused tests above, while this check protects
        # the rest of the vision algorithm from unrelated edits.
        current = replace_function(
            current, baseline, "_compose_ordinary_steering_error")
        current = replace_function(
            current, baseline, "_build_gray_overlay_text_cache")
        current = current.replace(
            'detail_text=%dms preview=per_frame ordinary_preview=%s "\n'
            '          "uart=3@115200/8N1 exposure_us=%s ',
            'detail_text=%dms preview=per_frame uart=3@115200/8N1 exposure_us=%s ')
        current = current.replace(
            '              ("on" if getattr(config,\n'
            '                               "ORDINARY_PREVIEW_CONTROL_ENABLED", False)\n'
            '               else "off"),\n',
            '')
        current = current.replace(
            'preview_mode=%s preview=%+d preview_candidate=%+d bend=%.2f ',
            'preview=%+d preview_candidate=%+d bend=%.2f ')
        current = current.replace(
            '                      "DISABLED", slope_hint,\n'
            '                      ("ON" if getattr(\n'
            '                          config, "ORDINARY_PREVIEW_CONTROL_ENABLED", False)\n'
            '                       else "OFF"),\n',
            '                      "DISABLED", slope_hint,\n')
        return current

    def test_vision_algorithm_ast_matches_baseline_after_diag_and_build_changes(self):
        baseline_main = (BASELINE_DIR / "main.py").read_text(encoding="utf-8")
        current_main = (OPENMV_DIR / "main.py").read_text(encoding="utf-8")
        baseline_config = (BASELINE_DIR / "config.py").read_text(
            encoding="utf-8")
        current_config = (OPENMV_DIR / "config.py").read_text(
            encoding="utf-8")
        current_config = current_config.replace(
            'BUILD_ID = "AGGRESSIVE_TURN_POS_ONLY_20261017"',
            'BUILD_ID = "CONTOUR_ARRAYFIX_20261016"')
        current_config = current_config.replace(
            "SINGLE_EDGE_ANCHOR_RADIUS_CELLS = 16.0",
            "SINGLE_EDGE_ANCHOR_RADIUS_CELLS = 10.0")
        current_config = current_config.replace(
            "SINGLE_EDGE_MIN_ROWS = 5", "SINGLE_EDGE_MIN_ROWS = 8")
        current_config = current_config.replace(
            "SINGLE_EDGE_MIN_SPAN = 6", "SINGLE_EDGE_MIN_SPAN = 12")
        current_config = current_config.replace(
            "SINGLE_EDGE_MAX_MISSING_ROWS = 3\n", "")
        current_config = current_config.replace(
            "SINGLE_EDGE_WEAK_SUPPORT_ROWS = 8\n", "")
        current_config = current_config.replace(
            "STEERING_DIAG_ENABLED = True\n", "")
        current_config = current_config.replace(
            "ORDINARY_PREVIEW_CONTROL_ENABLED = False\n", "")
        current_main = self._normalized_source(current_main)
        current_main = self._normalize_position_only_changes(
            current_main, baseline_main)
        self.assertEqual(
            ast.dump(ast.parse(current_main), include_attributes=False),
            ast.dump(ast.parse(baseline_main), include_attributes=False))
        self.assertEqual(
            ast.dump(ast.parse(current_config), include_attributes=False),
            ast.dump(ast.parse(baseline_config), include_attributes=False))


class _ThreeFrameUART:
    def __init__(self, fake_time, stop_after=3):
        self.fake_time = fake_time
        self.frames = []
        self.stop_after = stop_after

    def write(self, frame):
        self.frames.append(bytes(frame))
        if len(self.frames) >= self.stop_after:
            self.fake_time.us = 1100000
        return len(frame)


if __name__ == "__main__":
    unittest.main()
