"""CanMV any() is a 0/1 readiness flag, including when many bytes wait."""
import unittest
from unittest.mock import patch

import test_hal as fixtures
hal = fixtures.hal
import main
import reverse_control as reverse
import wireless_link as wireless


class ReadinessTests(unittest.TestCase):
    setUp = fixtures.HALTests.setUp
    tearDown = fixtures.HALTests.tearDown

    def exercise(self, old_semantics=False):
        uart = hal.UARTAdapter(1, 115200)
        stream = bytearray()
        uart.device.any = lambda: int(bool(stream))

        def read(count):
            data = bytes(stream[:count])
            del stream[:count]
            return data or None

        def readinto(buffer):
            data = read(len(buffer)) or b""
            buffer[:len(data)] = data
            return len(data)

        uart.device.read = read
        uart.device.readinto = readinto
        if old_semantics:
            uart.any = lambda: uart.device.any()
        parser = reverse.ReverseControlParser()
        link = wireless.WirelessCameraLink(enabled=True, parked_cb=lambda: True)
        drain = [0]
        with patch.object(main, "reverse_control", reverse):
            for step in range(50):
                now = 1000 + step * 100
                stream.extend(reverse.pack_profile_state(step, 3, 6))
                if step % 2 == 0:
                    stream.extend(wireless.encode_frame(
                        wireless.TYPE_HELLO, step, 17, now,
                        (17).to_bytes(4, "little") + b"\x01\0\0\0"))
                main._service_reverse_control(
                    uart, parser, bytearray(64), drain, now,
                    lambda a, b: a - b, link, receive_budget=256)
        return parser, link, len(stream)

    def test_old_count_interpretation_reproduces_partial_timeout(self):
        parser, link, remaining = self.exercise(True)
        self.assertEqual(parser.accepted_profile_states, 0)
        self.assertFalse(parser.profile_is_fresh(5900, lambda a, b: a - b))
        self.assertGreater(remaining, 1000)

    def test_readiness_prefetch_keeps_profiles_and_hello_current(self):
        parser, link, remaining = self.exercise()
        self.assertEqual(parser.accepted_profile_states, 50)
        self.assertEqual(parser.profile_ack(5900, lambda a, b: a - b), 3)
        self.assertEqual(link.rx_frames, 25)
        self.assertTrue(link._hello_clock_is_fresh(5900))
        self.assertEqual(remaining, 0)

    def test_no_read_when_native_reports_not_ready(self):
        uart = hal.UARTAdapter(1, 115200)
        uart.device.any = lambda: 0
        uart.device.read = lambda count: self.fail("read on empty native UART")
        self.assertEqual(uart.any(), 0)


if __name__ == "__main__":
    unittest.main()
