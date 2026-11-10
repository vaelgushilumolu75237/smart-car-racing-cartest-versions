"""Whole-chunk legacy forwarding preserves the bytewise parser contract."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'openmv'))
import reverse_control as rc
import wireless_link as wl


class MultiplexBurstTests(unittest.TestCase):
    def test_bounded_chunks_match_single_byte_delivery(self):
        payload = (rc.pack_frame(rc.COMMAND_RESET, 1) +
                   rc.pack_frame(rc.COMMAND_KEEP, 2) +
                   rc.pack_profile_state(1, 3, 7, session_token=0xD391A65A) +
                   wl.encode_frame(wl.TYPE_HELLO, 1, 123, 1000,
                                   bytes((123, 0, 0, 0, 1, 0, 0, 0))) +
                   rc.pack_frame(rc.COMMAND_KEEP, 3) +
                   rc.pack_profile_state(2, 3, 7, session_token=55))
        for seed in range(80):
            rng = random.Random(seed)
            a, b = wl.WirelessCameraLink(), wl.WirelessCameraLink()
            pa, pb = rc.ReverseControlParser(), rc.ReverseControlParser()
            offset = 0
            while offset < len(payload):
                size = rng.randrange(1, 65)
                chunk = bytearray(payload[offset:offset + size])
                # Stale bytes beyond the declared read length cannot disable
                # the safe legacy fast path or be exposed to either parser.
                length = len(chunk)
                chunk += b'\xd3\x91\xa6\x5a'
                a.feed_multiplexed(chunk, length, 1000, pa)
                for value in chunk[:length]:
                    b.feed_multiplexed(bytes((value,)), 1, 1000, pb)
                self.assertEqual(pa.__dict__, pb.__dict__, (seed, offset))
                self.assertEqual(a.session, b.session)
                self.assertEqual(a._legacy_length, b._legacy_length)
                self.assertEqual(a.tx_queue, b.tx_queue)
                offset += length


if __name__ == '__main__':
    unittest.main()
