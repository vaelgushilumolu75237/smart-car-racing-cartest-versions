"""Model regular MCU traffic larger than one legacy 64-byte feed."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "openmv"))
import main
import reverse_control as reverse
import wireless_link as wireless


class BurstUART:
    def __init__(self):
        self.pending = bytearray()
        self.read_sizes = []

    def any(self):
        return len(self.pending)

    def readinto(self, output, count):
        n = min(count, len(output), len(self.pending))
        output[:n] = self.pending[:n]
        del self.pending[:n]
        self.read_sizes.append(n)
        return n


class MultiplexedBurstTests(unittest.TestCase):
    def setUp(self):
        self.reverse_patch = patch.object(main, "reverse_control", reverse)
        self.reverse_patch.start()
        self.addCleanup(self.reverse_patch.stop)

    def exercise(self, budget):
        uart = BurstUART()
        parser = reverse.ReverseControlParser()
        link = wireless.WirelessCameraLink(enabled=True, parked_cb=lambda: True)
        drain = [0]
        for step in range(12):
            now = 1000 + step * 100
            # Four profile records, RESET, and HELLO: 124 valid bytes per poll.
            for seq in range(4 * step, 4 * step + 4):
                uart.pending.extend(reverse.pack_profile_state(
                    seq, reverse.PROFILE_C, reverse.PROFILE_GYRO350_SUPPORTED))
            uart.pending.extend(reverse.pack_frame(reverse.COMMAND_RESET, step))
            uart.pending.extend(wireless.encode_frame(
                wireless.TYPE_HELLO, step, 17, now,
                (17).to_bytes(4, "little") + b"\x01\x00\x00\x00"))
            start = len(uart.read_sizes)
            main._service_reverse_control(uart, parser, bytearray(64), drain,
                                          now, lambda a, b: a - b, link,
                                          receive_budget=budget)
            self.assertLessEqual(sum(uart.read_sizes[start:]), budget)
            self.assertLessEqual(max(uart.read_sizes[start:]), 64)
        return uart, parser, link

    def test_original_budget_reproduces_profile_starvation(self):
        _, parser, _ = self.exercise(64)
        self.assertLess(parser.accepted_profile_states, 5)
        self.assertFalse(parser.profile_is_fresh(2100, lambda a, b: a - b))

    def test_bounded_multiplex_budget_accepts_profiles_and_hello(self):
        uart, parser, link = self.exercise(256)
        self.assertEqual(parser.accepted_profile_states, 48)
        self.assertEqual(parser.profile_ack(2100, lambda a, b: a - b), 3)
        self.assertEqual(link.session, 17)
        self.assertEqual(len(uart.pending), 0)

    def test_expired_snapshot_is_not_relabelled_fresh(self):
        uart = BurstUART()
        parser = reverse.ReverseControlParser()
        profile = reverse.pack_profile_state(1, 3, reverse.PROFILE_GYRO350_SUPPORTED)
        parser.feed(profile, len(profile), 0)
        uart.pending.extend(reverse.pack_profile_state(2, 3, 4))
        main._service_reverse_control(uart, parser, bytearray(64), [0],
                                      300, lambda a, b: a - b, receive_budget=256)
        self.assertFalse(parser.profile_is_fresh(300, lambda a, b: a - b))
        uart.pending.extend(reverse.pack_profile_state(3, 3, 4))
        main._service_reverse_control(uart, parser, bytearray(64), [0],
                                      310, lambda a, b: a - b, receive_budget=256)
        self.assertTrue(parser.profile_is_fresh(310, lambda a, b: a - b))


if __name__ == "__main__":
    unittest.main()
