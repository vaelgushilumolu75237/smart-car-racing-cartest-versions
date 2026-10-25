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
BASELINE_DIR = (OPENMV_DIR / "tests" / "fixtures" /
                "steering_hold3s_20261016")
TEST_DIR = OPENMV_DIR / "tests"
for _path in (OPENMV_DIR, TEST_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import main
import fast_single_edge
import reverse_control
import vision_protocol
import test_fast_main_ring_slope as main_harness


def _frame(sequence, valid, error=0):
    return vision_protocol.pack_frame(sequence, valid, 60 if valid else 0,
                                      error if valid else 0, 0)


def _record_frame(diag, sequence, valid):
    return diag.record(
        _frame(sequence, valid), True, False, False,
        sequence, sequence, 0, 0, 0, 0, 0, 0, 0, 0, 0)


class _FreezeSnapshotTracker:
    def __init__(self):
        self.freeze_calls = 0

    def freeze_diagnostic_snapshot(self):
        self.freeze_calls += 1
        return True


class SteeringDiagBufferTests(unittest.TestCase):
    def setUp(self):
        self.diag = main._SteeringDiagBuffer()

    def test_default_is_enabled_with_exact_preallocated_ring(self):
        self.assertIs(config.STEERING_DIAG_ENABLED, True)
        self.assertIsInstance(main._STEERING_DIAG, main._SteeringDiagBuffer)
        self.assertEqual(len(main._STEERING_DIAG.data), 256 * 32)
        self.assertEqual(len(main._STEERING_DIAG.extension_data), 256 * 16)
        self.assertEqual(len(main._STEERING_DIAG.control_extension_data),
                         256 * 32)
        self.assertEqual(main._SteeringDiagBuffer.RECORD_SIZE, 32)
        self.assertEqual(main._SteeringDiagBuffer.PRE_TRIGGER_CAPACITY, 64)
        self.assertEqual(main._SteeringDiagBuffer.POST_TRIGGER_CAPACITY, 192)

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
        control = {
            "profile_code": 1, "pd_enabled": True,
            "preview_enabled": True, "preview_confirmed": True,
            "preview_geometry_valid": True, "preview_same_sign": True,
            "duplicate": True, "position_p": -1000,
            "derivative_d": -100, "pd_output": -925,
            "preview_candidate": -100, "preview_gain_milli": 200,
            "preview_applied": 75, "capture_dt_ms": 25,
            "preview_state": 2, "reset_code": 5,
        }
        recorded = self.diag.record(
            frame, True, False, True, period - 10, 5,
            -1000, -100, 75, -925, -900, 250, 255, 5, 0xFFFFFFFF,
            tuple(range(16)), control)
        self.assertTrue(recorded)
        data = self.diag.data
        self.assertEqual(len(data), 256 * 32)
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
        self.assertEqual(self.diag.extension_data[:16], bytes(range(16)))
        extension = self.diag.control_extension_data[:32]
        self.assertEqual(extension[0], 2)
        self.assertEqual(extension[1], 0x7F)
        self.assertEqual(main._SteeringDiagBuffer._u16(extension, 2), 0xFFFF)
        self.assertEqual(main._SteeringDiagBuffer._u32(extension, 4),
                         period - 10)
        self.assertEqual(main._SteeringDiagBuffer._u32(extension, 8), 5)
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 12), -1000)
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 14), -100)
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 16), -925)
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 18), -100)
        self.assertEqual(main._SteeringDiagBuffer._u16(extension, 20), 200)
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 22), 75)
        self.assertEqual(main._SteeringDiagBuffer._u16(extension, 24), 25)
        self.assertEqual(extension[26:28], bytes((2, 5)))
        self.assertEqual(main._SteeringDiagBuffer._i16(extension, 28), -1000)
        expected_crc = vision_protocol.crc16_ccitt_false(extension[:30])
        self.assertEqual(main._SteeringDiagBuffer._u16(extension, 30),
                         expected_crc)
        extension[8] ^= 1
        self.assertNotEqual(main._SteeringDiagBuffer._u16(extension, 30),
                            vision_protocol.crc16_ccitt_false(extension[:30]))
        self.assertEqual(self.diag.record_errors, 0)

    def test_viper_crc_matches_reference_and_full_record_bytes(self):
        table = main._SteeringDiagBuffer._CRC16_NIBBLE_TABLE_ARRAY
        for seed in range(64):
            offset = seed % 7
            payload = bytearray((index * 37 + seed * 19) & 0xFF
                                for index in range(offset + 32))
            expected = vision_protocol.crc16_ccitt_false(
                payload[offset:offset + 30])
            actual = main._crc16_control_extension_viper(
                payload, offset, table)
            self.assertEqual(actual, expected, seed)
            self.assertEqual(payload[offset + 30], expected & 0xFF, seed)
            self.assertEqual(payload[offset + 31], expected >> 8, seed)

        reference = main._SteeringDiagBuffer()
        accelerated = main._SteeringDiagBuffer()
        control = {
            "profile_code": 3, "pd_enabled": True,
            "preview_enabled": True, "preview_confirmed": True,
            "preview_geometry_valid": False, "preview_same_sign": True,
            "duplicate": False, "position_p": -321,
            "derivative_d": 123, "pd_output": -198,
            "preview_candidate": 77, "preview_gain_milli": 600,
            "preview_applied": 46, "capture_dt_ms": 19,
            "preview_state": 2, "reset_code": 4,
        }
        frame = _frame(0xBEEF, 1, -777)
        args = (frame, True, False, True, 0x12345678, 0x23456789,
                -321, 77, 46, -198, -208, 15, 22, 5, 0xDEADBEEF,
                tuple(range(16)), control)
        with patch.object(main, "_NATIVE_ACCELERATION_ENABLED", False):
            self.assertTrue(reference.record(*args))
        with patch.object(main, "_NATIVE_ACCELERATION_ENABLED", True):
            self.assertTrue(accelerated.record(*args))
        self.assertEqual(accelerated.data, reference.data)
        self.assertEqual(accelerated.extension_data, reference.extension_data)
        self.assertEqual(accelerated.control_extension_data,
                         reference.control_extension_data)
        self.assertEqual(accelerated.curve_trace_data,
                         reference.curve_trace_data)

        no_diag_reference = main._SteeringDiagBuffer()
        no_diag_accelerated = main._SteeringDiagBuffer()
        no_diag_args = (
            _frame(17, 1, 400), True, False, True,
            0x7FFFFFFF, -0x80000000, 0x7FFFFFFF, -0x80000000,
            40000, -40000, 32768, -32769, 1000, 8, -1, None, None)
        with patch.object(main, "_NATIVE_ACCELERATION_ENABLED", False):
            self.assertTrue(no_diag_reference.record(*no_diag_args))
        with patch.object(main, "_NATIVE_ACCELERATION_ENABLED", True):
            self.assertTrue(no_diag_accelerated.record(*no_diag_args))
        self.assertEqual(no_diag_accelerated.control_extension_data,
                         no_diag_reference.control_extension_data)

    def test_none_pending_side_roundtrips_and_trigger_trace_is_chronological(self):
        tracker = fast_single_edge.FastSingleEdgeTracker(config)
        tracker.begin_frame()
        tracker.skip_frame("NO_FRESH_KEEP")
        candidate_diagnostic = tracker.pair_diagnostic_record
        self.assertEqual(candidate_diagnostic[10], -1)

        for sequence in range(70):
            _record_frame(self.diag, sequence, True)
        self.diag.record(
            _frame(70, 0), True, False, False,
            70, 70, 0, 0, 0, 0, 0, 0, 0, 0, 70,
            candidate_diagnostic)
        for sequence in range(71, 262):
            _record_frame(self.diag, sequence, False)

        self.assertTrue(self.diag.frozen)
        self.assertEqual(self.diag.count, 256)
        self.assertEqual(self.diag.extension_data[64 * 16 + 10], 255)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.diag.dump(config.BUILD_ID)
        lines = output.getvalue().splitlines()
        rows = [line.split(",") for line in lines[4:-1]]
        self.assertEqual(len(rows), 256)
        self.assertEqual([int(row[0]) for row in rows], list(range(6, 262)))
        self.assertEqual(rows[64][17 + 10], "-1")

    def test_trigger_is_first_successful_full_valid_to_invalid_write(self):
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
        self.assertFalse(self.diag.frozen)
        self.assertTrue(self.diag.triggered)
        self.assertFalse(self.diag.last_full_valid)
        self.assertEqual(self.diag.count, 5)
        self.assertEqual(self.diag.pre_count, 4)
        self.assertEqual(self.diag.post_count, 1)
        self.assertEqual(self.diag.data[64 * 32 + 2] & 0x07, 0x02)
        self.assertTrue(self.diag.record(
            _frame(5, 1), True, False, True, 11, 12,
            0, 0, 0, 0, 0, 0, 0, 0, 11))
        self.assertEqual(self.diag.count, 6)

    def test_pretrigger_ring_wrap_keeps_newest_64_in_chronological_order(self):
        for sequence in range(70):
            self.diag.record(_frame(sequence, 1, sequence % 1000),
                             True, False, False,
                             sequence, sequence + 1, sequence,
                             0, 0, sequence, sequence, 0,
                             sequence % 256, 1, sequence)
        self.assertEqual(self.diag.count, 64)
        self.assertEqual(self.diag.pre_count, 64)
        self.assertEqual(self.diag.overwritten, 6)
        self.assertEqual(self.diag.write_index, 6)
        oldest = self.diag.write_index
        self.assertEqual(main._SteeringDiagBuffer._u16(
            self.diag.data, oldest * 32), 6)
        output = io.StringIO()
        with patch.object(main, "_STEERING_DIAG_TICKS_PERIOD_MS", 1 << 30), \
                contextlib.redirect_stdout(output):
            self.diag.dump(config.BUILD_ID)
        lines = output.getvalue().splitlines()
        self.assertIn("capacity=256 record_size=32 extension_record_size=16",
                      lines[0])
        self.assertIn("pre_capacity=64 post_capacity=192", lines[0])
        self.assertIn("count=64 frozen=0 triggered=0 pre_count=64",
                      lines[0])
        self.assertIn("overwritten=6 record_errors=0", lines[0])
        self.assertIn("ticks_period_ms=1073741824", lines[0])
        self.assertIn("extension=single_edge_candidate_v2", lines[1])
        self.assertIn("control_extension_v2_record_size=32", lines[0])
        self.assertIn("timestamps=camera_ticks_ms_snapshot_complete,uart_write_return",
                      lines[2])
        self.assertIn("14-15:derivative_d_i16_clamped_component", lines[2])
        self.assertEqual(lines[3], self.diag._CSV_HEADER)
        self.assertTrue(lines[4].startswith("6,"))
        self.assertTrue(lines[-2].startswith("69,"))
        self.assertEqual(lines[-1], "STEERING_DIAG_END")

    def test_trigger_starts_fixed_post_region_without_rotating_pre_ring(self):
        for sequence in range(70):
            self.diag.record(_frame(sequence, 1), True, False, False,
                             sequence, sequence, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, tuple([sequence] * 16))
        first_invalid = _frame(70, 0)
        self.diag.record(first_invalid, True, False, False,
                         70, 70, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         tuple([70] * 16))
        self.assertTrue(self.diag.triggered)
        self.assertEqual(self.diag.pre_count, 64)
        self.assertEqual(self.diag.pre_oldest, 6)
        self.assertEqual(self.diag.post_count, 1)
        self.assertEqual(self.diag.count, 65)
        self.assertEqual(main._SteeringDiagBuffer._u16(
            self.diag.data, 64 * 32), 70)
        self.assertEqual(self.diag.extension_data[64 * 16], 70)
        for sequence in range(71, 262):
            self.diag.record(_frame(sequence, 0), True, False, False,
                             sequence, sequence, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, tuple([sequence & 255] * 16))
        self.assertTrue(self.diag.triggered)
        self.diag.freeze("TEST_FREEZE")
        self.assertTrue(self.diag.frozen)
        self.assertEqual(self.diag.freeze_reason, "POST_COMPLETE")
        self.assertEqual(self.diag.post_count, 192)
        self.assertEqual(self.diag.count, 256)
        self.assertFalse(self.diag.dumped)

    def test_rearm_clears_metadata_without_replacing_the_buffer(self):
        self.diag.record(_frame(1, 1), True, False, False,
                         1, 2, 0, 0, 0, 0, 0, 0, 0, 1, 0)
        self.diag.record(_frame(2, 0), True, False, False,
                         2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        for sequence in range(3, 194):
            _record_frame(self.diag, sequence, False)
        self.assertTrue(self.diag.frozen)
        old_buffer = self.diag.data
        self.diag.record_errors = 4
        self.diag.rearm()
        self.assertIs(self.diag.data, old_buffer)
        self.assertEqual((self.diag.write_index, self.diag.count,
                          self.diag.overwritten,
                          self.diag.record_errors), (0, 0, 0, 0))
        self.assertEqual((self.diag.pre_count, self.diag.pre_oldest,
                          self.diag.post_count), (0, 0, 0))
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

    def test_initial_keep_discards_precontrol_trace_and_reset_dumps_early(self):
        diag = main._SteeringDiagBuffer()
        session = main._SteeringDiagSession(diag)
        parser = reverse_control.ReverseControlParser()

        reset = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 10)
        parser.feed(reset, len(reset), 1)
        session.observe(parser, object(), config.BUILD_ID)
        self.assertFalse(session.session_active)
        _record_frame(diag, 0, True)

        keep = reverse_control.pack_frame(
            reverse_control.COMMAND_KEEP, 11)
        parser.feed(keep, len(keep), 2)
        session.observe(parser, object(), config.BUILD_ID)
        self.assertTrue(session.session_active)
        self.assertEqual(diag.count, 0)
        self.assertFalse(diag.frozen)

        _record_frame(diag, 2, True)
        output = io.StringIO()
        tracker = _FreezeSnapshotTracker()
        stop = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 12)
        with contextlib.redirect_stdout(output):
            parser.feed(stop, len(stop), 3)
            session.observe(parser, tracker, config.BUILD_ID)
            parser.feed(stop, len(stop), 4)
            session.observe(parser, tracker, config.BUILD_ID)
        self.assertTrue(diag.frozen)
        self.assertTrue(diag.dumped)
        self.assertEqual(diag.freeze_reason, "REVERSE_RESET")
        self.assertEqual(diag.count, 1)
        self.assertEqual(tracker.freeze_calls, 1)
        self.assertEqual(output.getvalue().count("STEERING_DIAG_BEGIN"), 1)
        self.assertIn("\n2,", output.getvalue())

    def test_reset_and_keep_consumed_together_finalize_then_rearm(self):
        diag = main._SteeringDiagBuffer()
        session = main._SteeringDiagSession(diag)
        parser = reverse_control.ReverseControlParser()
        tracker = _FreezeSnapshotTracker()

        reset = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 20)
        keep = reverse_control.pack_frame(
            reverse_control.COMMAND_KEEP, 21)
        parser.feed(reset, len(reset), 1)
        session.observe(parser, tracker, config.BUILD_ID)
        parser.feed(keep, len(keep), 2)
        session.observe(parser, tracker, config.BUILD_ID)
        _record_frame(diag, 5, True)

        combined = (reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 22) + reverse_control.pack_frame(
                reverse_control.COMMAND_KEEP, 23))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            parser.feed(combined, len(combined), 3)
            self.assertEqual(parser.last_event, "KEEP")
            session.observe(parser, tracker, config.BUILD_ID)
        self.assertEqual(output.getvalue().count("STEERING_DIAG_BEGIN"), 1)
        self.assertEqual(tracker.freeze_calls, 1)
        self.assertTrue(session.session_active)
        self.assertTrue(parser.single_enabled)
        self.assertEqual(diag.count, 0)
        self.assertFalse(diag.frozen)
        self.assertFalse(diag.dumped)

    def test_postcapture_freezes_without_dump_until_accepted_reset(self):
        diag = main._SteeringDiagBuffer()
        session = main._SteeringDiagSession(diag)
        parser = reverse_control.ReverseControlParser()
        tracker = _FreezeSnapshotTracker()
        reset = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 30)
        keep = reverse_control.pack_frame(
            reverse_control.COMMAND_KEEP, 31)
        parser.feed(reset, len(reset), 1)
        session.observe(parser, tracker, config.BUILD_ID)
        parser.feed(keep, len(keep), 2)
        session.observe(parser, tracker, config.BUILD_ID)

        for sequence in range(64):
            _record_frame(diag, sequence, True)
        _record_frame(diag, 64, False)
        for sequence in range(65, 256):
            _record_frame(diag, sequence, False)
        self.assertTrue(diag.frozen)
        self.assertEqual(diag.freeze_reason, "POST_COMPLETE")
        self.assertEqual(diag.count, 256)
        self.assertFalse(diag.dumped)
        self.assertEqual(tracker.freeze_calls, 0)

        output = io.StringIO()
        stop = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 32)
        with contextlib.redirect_stdout(output):
            parser.feed(stop, len(stop), 3)
            session.observe(parser, tracker, config.BUILD_ID)
        self.assertTrue(diag.dumped)
        self.assertEqual(output.getvalue().count("STEERING_DIAG_BEGIN"), 1)
        self.assertEqual(tracker.freeze_calls, 0)

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
        control_frames = [
            (reverse_control.pack_frame(
                reverse_control.COMMAND_RESET, 0) + reverse_control.pack_frame(
                    reverse_control.COMMAND_KEEP, 1)),
            b"",
            reverse_control.pack_frame(
                reverse_control.COMMAND_RESET, 2),
        ]
        control_index = [0]

        def service_control(_uart, parser, _buffer, _drain, now_ms,
                            _ticks_diff):
            index = control_index[0]
            control_index[0] += 1
            payload = (control_frames[index]
                       if index < len(control_frames) else b"")
            if payload:
                parser.feed(payload, len(payload), now_ms)
            return parser.single_enabled

        with patch.object(main, "_configure_gray_sensor", return_value=1), \
                patch.object(main, "sensor", fake_sensor), \
                patch.object(main, "time", fake_time), \
                patch.object(main, "_STEERING_DIAG", diag), \
                patch.object(main, "_service_reverse_control",
                             side_effect=service_control), \
                patch.object(main.fast_gray, "FastGrayMasks",
                             main_harness._GrayBuilder), \
                patch.object(main.fast_path, "FastPathDetector",
                             AlternatingPathDetector), \
                patch.object(main.fast_boundary, "FastBoundaryEvidence",
                             main_harness._BoundaryValidator), \
                patch.object(main.fast_single_edge,
                             "FastSingleEdgeTracker",
                             main_harness._SingleEdgeTracker), \
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
        self.assertEqual([len(frame) for frame in uart.frames], [16, 16, 16])
        self.assertEqual([frame[4] for frame in uart.frames], [0, 1, 2])
        self.assertEqual([frame[6] & vision_protocol.FLAG_VALID
                          for frame in uart.frames], [1, 0, 1])
        self.assertTrue(diag.frozen)
        self.assertTrue(diag.triggered)
        self.assertTrue(diag.dumped)
        self.assertEqual(diag.freeze_reason, "REVERSE_RESET")
        self.assertEqual(diag.count, 2)
        self.assertEqual(diag.post_count, 1)
        self.assertEqual(diag.record_errors, 0)
        self.assertEqual(main_harness._SingleEdgeTracker.freeze_calls, 1)
        self.assertEqual(output.getvalue().count("STEERING_DIAG_BEGIN"), 1)
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
        self.assertEqual(len(uart.frames[0]), 16)
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
            "SECOND_TURN_CONTINUITY_20261018",
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
        source = source.replace(
            "    steering_diag_session = _SteeringDiagSession(_STEERING_DIAG)\n",
            "")
        source = source.replace(
            "        steering_diag_session.observe(\n"
            "            reverse_parser, single_edge_tracker, config.BUILD_ID)\n",
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

    def test_unrelated_vision_ast_matches_baseline_after_control_changes(self):
        baseline_main = (BASELINE_DIR / "main.py").read_text(encoding="utf-8")
        current_main = (OPENMV_DIR / "main.py").read_text(encoding="utf-8")
        baseline_config = (BASELINE_DIR / "config.py").read_text(
            encoding="utf-8")
        current_config = (OPENMV_DIR / "config.py").read_text(
            encoding="utf-8")
        current_config = current_config.replace(
            'BUILD_ID = "SECOND_TURN_CONTINUITY_20261018"',
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
        current_tree = ast.parse(current_main)
        baseline_tree = ast.parse(baseline_main)

        def named_definitions(tree):
            return {node.name: ast.dump(node, include_attributes=False)
                    for node in tree.body
                    if isinstance(node, (ast.FunctionDef, ast.ClassDef))}

        current_defs = named_definitions(current_tree)
        baseline_defs = named_definitions(baseline_tree)
        self.assertEqual(
            current_defs.keys() - baseline_defs.keys(),
            {"_FastAdaptivePreview", "_FastPositionPD", "_FastSpeedStage",
             "_V7SummaryQueue", "_drain_v7_summary_queue",
             "_noop_preview_flush", "_v7_overlay_status",
             "_weighted_row_sums_viper",
             "_aggregate_selected_rows_viper",
             "_crc16_control_extension_viper",
             "_write_control_extension_viper",
             "_write_camera_diag_record_viper",
             # Additive native USB preview support introduced after this
             # frozen algorithm baseline; exercised by preview/package tests.
             "_FreshCaptureFPS", "_road_overlay_rows_unexposed",
             "_road_preview_int32_array_valid",
             "_road_overlay_packed_array_valid",
             "_build_usb_preview_compact_text", "_usb_vcp_is_connected",
             "_fixture_usb_vcp",
             "_NativeUSBPreviewSource", "_is_native_usb_preview_source",
             "_publish_current_usb_preview_jpeg",
             "_draw_road_overlay_packed",
             "_native_preview_control_api_available",
             "_open_usb_vcp_safely",
             "_road_preview_integer_geometry_viper",
             # Authorized profile/ring route integration is isolated behind
             # these two bounded runtime helpers and covered by main-path
             # protocol/steering tests.
             "_load_track_elements_runtime", "_fill_ring_control_route",
             # UART FIFO compatibility wrapper is transport-only; the adapter
             # behavior is covered by test_uart_fifo_wrapper.py.
             "_VisionUARTWriter"})
        self.assertEqual(baseline_defs.keys() - current_defs.keys(), set())
        # Geometry/V2 work intentionally adjusts camera setup, pixel mapping,
        # runtime API checks, and per-window summaries in addition to the
        # steering-control and RAM-fixture paths. Keep this list explicit so
        # unrelated definitions remain frozen against the prior runtime.
        changed_defs = {
            "_FastRowWeightWorkspace", "_FreshObservationFilter",
            "_V7OverlayState", "_V7PerfWindow", "_draw_gray_result",
            "_draw_sparse_surface_contour", "_draw_sparse_road_contour",
            "_service_reverse_control", "_selected_reference",
            "_validate_selected_fast_boundary", "_rounded_local",
            "_normalize_fast_reference",
            "_selected_actual_boundaries",
            "_main_fast", "_render_v7_preview_frame",
            "_validate_fast_config", "_configure_sensor",
            "_configure_gray_sensor", "_surface_pixel",
            "_validate_v6_gray_config", "_validate_v7_runtime_apis",
            "_load_fast_runtime", "_main_compare", "_print_v7_summary",
        }
        self.assertEqual(
            {name for name in current_defs.keys() & baseline_defs.keys()
             if current_defs[name] != baseline_defs[name]},
            changed_defs)
        for name in current_defs.keys() & baseline_defs.keys() - changed_defs:
            self.assertEqual(current_defs[name], baseline_defs[name], name)

        def assignments(source):
            tree = ast.parse(source)
            result = {}
            other_statements = []
            for node in tree.body:
                if isinstance(node, ast.Assign):
                    for target in node.targets:
                        if isinstance(target, ast.Name):
                            result[target.id] = ast.dump(
                                node, include_attributes=False)
                else:
                    other_statements.append(ast.dump(
                        node, include_attributes=False))
            return result, other_statements

        current_assignments, current_other = assignments(current_config)
        baseline_assignments, baseline_other = assignments(baseline_config)
        self.assertEqual(current_other, baseline_other)
        new_assignments = {
            "GRAY_MINPOOL_SCALE_X", "GRAY_MINPOOL_SCALE_Y",
            "POSITION_PD_DERIVATIVE_TAU_MS", "POSITION_PD_D_LIMIT",
            "POSITION_PD_MAX_DT_MS", "POSITION_PD_TIME_MS",
            "PREVIEW_APPLIED_LIMIT", "PREVIEW_CANDIDATE_MIN_ABS",
            "PREVIEW_CONFIRM_FRAMES", "PREVIEW_CONFIRM_MS",
            "PREVIEW_MAX_GAIN", "PREVIEW_POSITION_FULL_GAIN_ABS",
            "PREVIEW_POSITION_MIN_ABS", "PREVIEW_SLEW_PER_SECOND",
            "SENSOR_WINDOW_X", "SENSOR_WINDOW_Y",
            "SPEED_CANDIDATE_CONFIRM_FRAMES", "SPEED_CANDIDATE_CONFIRM_MS",
            "SPEED_CANDIDATE_MIN_ABS", "SPEED_EXIT_PHASE_MS",
            "SPEED_RELEASE_CANDIDATE_ABS", "SPEED_RELEASE_CONFIRM_MS",
            "SPEED_RELEASE_POSITION_ABS", "SPEED_SLOWDOWN_REQUEST_PERMILLE",
            "SPEED_STAGE_ENABLED",
            "STEERING_CORE_MIN_SUPPORT_ROWS",
            "STEERING_CORE_MIN_SUPPORT_SPAN_GRID",
            "STEERING_CORE_RAW_Y_MAX", "STEERING_CORE_RAW_Y_MIN",
            "STEERING_CONTROL_PROFILE", "STEERING_CONTROL_PROFILE_BUILD_IDS",
            "TRACK_PACKAGE_BUILD_ID",
            "VEHICLE_PROFILE_UNKNOWN", "VEHICLE_PROFILE_A",
            "VEHICLE_PROFILE_B", "VEHICLE_PROFILE_C",
            "RING_ENABLED_PROFILE_IDS", "PROFILE_STATE_REVISION",
            "PROFILE_STATE_TIMEOUT_MS",
            "TRACK_ELEMENTS_MAX_ENTRY_MS",
            "TRACK_ELEMENTS_MAX_ACTIVE_MS",
            "TRACK_ELEMENTS_MAX_ACTIVE_MILLIREV",
            "TRACK_ELEMENTS_NEAR_ROWS",
            "TRACK_ELEMENTS_MIN_NEAR_SUPPORT",
            "TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS",
            "TRACK_ELEMENTS_MIN_STABLE_FRAMES",
            "TRACK_ELEMENTS_MIN_SIDE_SUPPORT",
            "TRACK_ELEMENTS_MIN_SIDE_CONTIGUOUS",
            "TRACK_ELEMENTS_MAX_MISSING_SIDE_ROWS",
            "TRACK_ELEMENTS_MIN_DROP_FRAMES",
            "TRACK_ELEMENTS_OPPOSITE_POSITION_TOLERANCE_X2",
            "TRACK_ELEMENTS_REAPPEAR_SHIFT_X2",
            "TRACK_ELEMENTS_PEAK_REVERSAL_X2",
            "TRACK_ELEMENTS_PEAK_CONFIRM_FRAMES",
            "TRACK_ELEMENTS_MIN_TARGET_ROWS",
            "TRACK_ELEMENTS_MAX_CACHE_GAP",
            "TRACK_ELEMENTS_EXIT_DROP_FRAMES",
            "TRACK_ELEMENTS_EXIT_CONFIRM_FRAMES",
            "TRACK_ELEMENTS_EXIT_REPAIR_FRAMES",
            "TRACK_ELEMENTS_EXIT_WIDTH_TOLERANCE_CELLS",
            "TRACK_ELEMENTS_EXIT_CENTER_TOLERANCE_X2",
            "TRACK_ELEMENTS_REACQUIRE_FRAMES",
            "TRACK_ELEMENTS_CROSS_CONFIRM_FRAMES",
            "TRACK_ELEMENTS_CROSS_MIN_OPEN_ROWS",
            "TRACK_ELEMENTS_CROSS_OPEN_MIN_CELLS",
            "TRACK_ELEMENTS_CROSS_SYMMETRY_TOL_CELLS",
            "TRACK_ELEMENTS_CROSS_CENTER_TOLERANCE_X2",
            "TRACK_ELEMENTS_CROSS_WIDTH_TOLERANCE_CELLS",
            "STEERING_OUTPUT_LIMIT", "STEERING_ROW_WEIGHT_Q",
            "STEERING_ROW_WEIGHT_Q_SCALE", "SURFACE_PIXEL_SCALE_X",
            # Current V2 USB preview configuration, additive to the baseline.
            "SURFACE_PIXEL_SCALE_Y", "USB_PREVIEW_ENABLED",
            "USB_PREVIEW_OVERLAY_MODE", "USB_PREVIEW_QUALITY",
            "USB_PREVIEW_BUILD_ID",
        }
        changed_assignments = {
            "BUILD_ID", "FRAME_SIZE_NAME", "IMAGE_WIDTH",
            "OVERLAY_PERIOD_MS",
        }
        removed_assignments = {
            "GRAY_MINPOOL_SCALE", "STEERING_FAR_WEIGHT",
            "STEERING_NEAR_WEIGHT", "STEERING_SPLIT_GRID_Y",
            "STEERING_SPLIT_IMAGE_Y", "SURFACE_PIXEL_SCALE",
            "TURN_RESPONSE_VERSION",
        }
        self.assertEqual(current_assignments.keys() - baseline_assignments.keys(),
                         new_assignments)
        self.assertEqual(baseline_assignments.keys() - current_assignments.keys(),
                         removed_assignments)
        self.assertEqual(
            {name for name in current_assignments.keys() &
             baseline_assignments.keys()
             if current_assignments[name] != baseline_assignments[name]},
            changed_assignments)
        for name in current_assignments.keys() & baseline_assignments.keys() - changed_assignments:
            self.assertEqual(current_assignments[name],
                             baseline_assignments[name], name)


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
