"""Host coverage for the bounded camera-to-H743 D1 sideband sender."""

import sys
import types
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import main  # noqa: E402
import reverse_control  # noqa: E402
import vision_protocol  # noqa: E402


_TICKS_PERIOD = 1 << 30


def _ticks_diff(new, old):
    half = _TICKS_PERIOD // 2
    return ((new - old + half) % _TICKS_PERIOD) - half


class _UART:
    def __init__(self, mode="SYNCHRONOUS_FALLBACK", fifo_done=True,
                 writes_result=None):
        self.mode = mode
        self.done = fifo_done
        self.writes_result = writes_result
        self.writes = []
        self.buffer_ids = []

    def write(self, data):
        self.buffer_ids.append(id(data))
        self.writes.append(bytes(data))
        if self.writes_result is not None:
            return self.writes_result
        return len(data)

    def fifo_txdone(self):
        return self.done


def _send(sender, uart, now_ms, **overrides):
    values = {
        "interval_ms": 1000,
        "enabled": True,
        "primary_full_write": True,
        "primary_late": False,
        "remaining_us": 100000,
        "status_flags": 0x2B,
        "calibration_code": 1,
        "observation_code": 1,
        "reverse_code": 2,
        "raw_profile_ack": 2,
        "outgoing_profile_ack": 1,
        "vision_sequence": 0x1234,
        "vision_tx_errors": 0,
        "detector_errors": 0,
        "reverse_rejects": 0,
    }
    values.update(overrides)
    return sender.send_after_primary(
        uart, now_ms, _ticks_diff, values["interval_ms"],
        values["enabled"], values["primary_full_write"],
        values["primary_late"], values["remaining_us"],
        values["status_flags"], values["calibration_code"],
        values["observation_code"], values["reverse_code"],
        values["raw_profile_ack"], values["outgoing_profile_ack"],
        values["vision_sequence"], values["vision_tx_errors"],
        values["detector_errors"], values["reverse_rejects"])


class CameraUARTDiagnosticTests(unittest.TestCase):
    def test_d1_frame_uses_fixed_v3_layout_and_crc_range(self):
        packet = bytearray(16)
        returned = vision_protocol.pack_diagnostic_into(
            packet, 0x1234, 1, 0x02, 1, 2, 3, 0x21, 0x78, 0x56)
        self.assertIs(returned, packet)
        self.assertEqual(bytes(packet[:14]), bytes((
            0xA5, 0x5A, 0xD1, 0x08, 0x34, 0x12,
            1, 2, 1, 2, 3, 0x21, 0x78, 0x56)))
        self.assertEqual(
            int.from_bytes(packet[14:16], "little"),
            vision_protocol.crc16_ccitt_false(packet[2:14]))

    def test_profile_capability_requires_recognized_fresh_revision_one(self):
        parser = reverse_control.ReverseControlParser()
        no_capability = reverse_control.pack_profile_state(
            1, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE, session_token=5)
        parser.feed(no_capability, len(no_capability), 0)
        self.assertEqual(parser.profile_ack(0, _ticks_diff),
                         reverse_control.PROFILE_B)
        self.assertTrue(parser.profile_session_active)
        self.assertFalse(parser.profile_diagnostics_enabled(0, _ticks_diff))

        with_capability = reverse_control.pack_profile_state(
            2, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE |
            reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED,
            session_token=5)
        parser.feed(with_capability, len(with_capability), 100)
        self.assertTrue(parser.profile_session_active)
        self.assertTrue(parser.profile_diagnostics_enabled(349, _ticks_diff))
        self.assertFalse(parser.profile_diagnostics_enabled(350, _ticks_diff))
        self.assertEqual(parser.profile_ack(349, _ticks_diff),
                         reverse_control.PROFILE_B)

        unknown = reverse_control.pack_profile_state(
            3, 99, reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED)
        parser.feed(unknown, len(unknown), 400)
        self.assertFalse(parser.profile_diagnostics_enabled(400, _ticks_diff))

    def test_old_h743_capability_absence_prevents_all_d1_writes(self):
        parser = reverse_control.ReverseControlParser()
        frame = reverse_control.pack_profile_state(
            1, reverse_control.PROFILE_A, revision=1)
        parser.feed(frame, len(frame), 0)
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        for now in (0, 1000, 2000):
            _send(sender, uart, now,
                  enabled=parser.profile_diagnostics_enabled(
                      now, _ticks_diff))
        self.assertEqual(uart.writes, [])

        fresh_parser = reverse_control.ReverseControlParser()
        capable = reverse_control.pack_profile_state(
            1, reverse_control.PROFILE_A,
            reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED)
        fresh_parser.feed(capable, len(capable), 0)
        capable_sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        capable_uart = _UART()
        _send(capable_sender, capable_uart, 0,
              enabled=fresh_parser.profile_diagnostics_enabled(0, _ticks_diff))
        _send(capable_sender, capable_uart, 1000,
              enabled=fresh_parser.profile_diagnostics_enabled(1000,
                                                               _ticks_diff))
        self.assertEqual(len(capable_uart.writes), 1)

    def test_interval_is_clamped_to_one_record_per_second(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        _send(sender, uart, 0, interval_ms=1)
        self.assertFalse(_send(sender, uart, 999, interval_ms=1))
        self.assertEqual(len(uart.writes), 1)
        self.assertTrue(_send(sender, uart, 1000, interval_ms=1))
        self.assertEqual([frame[6] for frame in uart.writes], [3, 1])

    def test_identity_then_status_counters_and_changed_status_schedule(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        start = _TICKS_PERIOD - 500
        original_packet_id = id(sender.packet)

        self.assertTrue(_send(sender, uart, start))
        self.assertEqual(uart.writes[-1][6], 3)
        self.assertEqual(uart.writes[-1][7:14], bytes((
            1, 0x29, 0x10, 0x26, 0x20, 1, 0)))

        self.assertFalse(_send(sender, uart, (start + 999) % _TICKS_PERIOD))
        self.assertTrue(_send(sender, uart, (start + 1000) % _TICKS_PERIOD))
        status = uart.writes[-1]
        self.assertEqual(status[6], 1)
        self.assertEqual(status[7:14], bytes((
            0x2B, 1, 1, 2, 0x21, 0x34, 0x12)))

        changed_at = (start + 2000) % _TICKS_PERIOD
        self.assertTrue(_send(
            sender, uart, changed_at, status_flags=0x2A))
        self.assertEqual(uart.writes[-1][6], 1)
        self.assertEqual(uart.writes[-1][7], 0x2A)

        self.assertTrue(_send(
            sender, uart, (start + 3000) % _TICKS_PERIOD,
            status_flags=0x2A))
        self.assertEqual(uart.writes[-1][6], 2)
        self.assertEqual(uart.buffer_ids, [original_packet_id] * 4)
        self.assertIs(sender.packet, sender.packet)

    def test_status_ack_and_validity_follow_the_primary_v3_bytes(self):
        diagnostics = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        self.assertEqual(diagnostics.calibration_code(
            "CALIBRATION_READY", True), 1)
        self.assertEqual(diagnostics.calibration_code(
            "FILE_NOT_FOUND", True), 2)
        self.assertEqual(diagnostics.calibration_code(
            "NEAR_WIDTH_SUPPORT_INSUFFICIENT", True), 8)
        self.assertEqual(diagnostics.calibration_code(
            "CALIBRATION_READY", False), 6)
        primary = vision_protocol.pack_frame_v3(
            0x2345, 1, 80, 25, profile_ack=0)
        status_reason = diagnostics.observation_code(
            "RING_CALIBRATION_REQUIRED", None,
            bool(primary[6] & vision_protocol.FLAG_VALID), "road")
        self.assertEqual(status_reason, 1)

        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        _send(sender, uart, 0)
        _send(sender, uart, 1000, raw_profile_ack=3,
              outgoing_profile_ack=primary[11], vision_sequence=0x2345,
              status_flags=(0x01 | 0x02 | 0x08 | 0x20))
        self.assertEqual(uart.writes[-1][6], 1)
        self.assertEqual(uart.writes[-1][11], 0x30)
        self.assertEqual(uart.writes[-1][12:14], b"\x45\x23")

        invalid_primary = vision_protocol.pack_frame_v3(
            0x2346, 0, 0, 0, profile_ack=2)
        self.assertEqual(diagnostics.observation_code(
            "RING_CALIBRATION_REQUIRED", None,
            bool(invalid_primary[6] & vision_protocol.FLAG_VALID), "NONE"), 2)

    def test_counter_fields_saturate_and_set_overflow_flag(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        _send(sender, uart, 0)
        _send(sender, uart, 1000)
        _send(sender, uart, 2000, vision_tx_errors=65536,
              detector_errors=321, reverse_rejects=70000)
        counters = uart.writes[-1]
        self.assertEqual(counters[6:14], bytes((
            2, 1, 0xFF, 0xFF, 0x41, 0x01, 0xFF, 0xFF)))

    def test_counters_cannot_be_starved_by_repeated_status_changes(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        _send(sender, uart, 0)
        _send(sender, uart, 1000)
        for index, now in enumerate((2000, 3000, 4000, 5000, 6000),
                                    start=1):
            _send(sender, uart, now, status_flags=0x2B + index)
        self.assertEqual([frame[6] for frame in uart.writes],
                         [3, 1, 1, 2, 1, 1, 2])

    def test_identity_is_resent_after_capability_goes_stale_and_returns(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        _send(sender, uart, 0)
        _send(sender, uart, 1000)
        _send(sender, uart, 1500, enabled=False)
        _send(sender, uart, 2000)
        self.assertEqual([frame[6] for frame in uart.writes], [3, 1, 3])

    def test_fifo_busy_budget_and_failed_primary_skip_without_retry(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        busy = _UART(mode="FIFO_QUEUED", fifo_done=False)
        self.assertFalse(_send(sender, busy, 0))
        self.assertEqual(busy.writes, [])
        busy.done = True
        self.assertFalse(_send(sender, busy, 1, remaining_us=999))
        self.assertTrue(_send(sender, busy, 2, remaining_us=1000))
        self.assertEqual(busy.writes[-1][6], 3)

        failing = _UART(writes_result=7)
        failed_sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        self.assertFalse(_send(failed_sender, failing, 0))
        self.assertEqual(failed_sender.errors, 1)
        self.assertFalse(_send(failed_sender, failing, 999))
        self.assertEqual(len(failing.writes), 1)
        self.assertFalse(_send(failed_sender, failing, 1000,
                               primary_full_write=False))
        self.assertFalse(_send(failed_sender, failing, 1000,
                               primary_late=True))
        self.assertEqual(len(failing.writes), 1)

    def test_sync_mode_requires_budget_and_diag_errors_are_isolated(self):
        sender = main._VisionUARTWriter.CameraDiagnostics(0x20261029)
        uart = _UART()
        self.assertFalse(_send(sender, uart, 0, remaining_us=21999))
        self.assertTrue(_send(sender, uart, 0, remaining_us=22000))

        parser = reverse_control.ReverseControlParser()
        reset = reverse_control.pack_frame(0, 1)
        keep = reverse_control.pack_frame(1, 2)
        parser.feed(reset, len(reset), 0)
        parser.feed(keep, len(keep), 10)
        generation = parser.generation
        failed_uart = _UART(writes_result=1)
        self.assertFalse(_send(
            main._VisionUARTWriter.CameraDiagnostics(0x20261029),
            failed_uart, 0))
        self.assertTrue(parser.control_is_fresh(20, _ticks_diff))
        self.assertEqual(parser.generation, generation)
        self.assertEqual(parser.last_event, "KEEP")


if __name__ == "__main__":
    unittest.main(verbosity=2)
