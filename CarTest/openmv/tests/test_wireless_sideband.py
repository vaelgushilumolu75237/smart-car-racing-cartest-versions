"""Host tests of the actual camera bridge, permissions and saved profiles."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
def load(name):
    spec = importlib.util.spec_from_file_location("test_" + name, ROOT / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
w = load("wireless_sideband")
r = load("reverse_control")
v = load("vision_protocol")


class WirelessTests(unittest.TestCase):
    def setUp(self):
        self.reverse = r.ReverseControlParser()
        self.reverse.feed(r.pack_profile_state(1, r.PROFILE_C, 0, 1, 77, 0), 20, 100)
        self.link = w.WirelessSideband(self.reverse)
        self.hello()
        self.link.queue.clear()

    def hello(self, key=2, parked=1, now=100, sequence=1):
        p = bytearray(8)
        w._put_u32(p, 0, 123)
        p[4], p[5] = parked, key
        self.feed(w.encode_frame(w.TYPE_HELLO, sequence, 123, now, p), now)

    def feed(self, packet, now=100):
        for start in range(0, len(packet), 17):
            part = packet[start:start+17]
            self.link.feed_multiplexed(part, len(part), now)

    def command(self, op, sequence=2, extra=b"", now=110, timestamp=None):
        p = bytes((op, 1, 0, 0)) + extra
        self.feed(w.encode_frame(w.TYPE_COMMAND, sequence, 123,
                                 now if timestamp is None else timestamp, p), now)
        return self.link.cached_ack[0] if self.link.cached_ack else None

    def test_fifo_busy_after_primary_eventually_sends_reply(self):
        class UART:
            def __init__(self):
                self.polls = 0
                self.sent = []
            def fifo_txdone(self):
                self.polls += 1
                return self.polls >= 4
            def write(self, packet):
                self.sent.append(bytes(packet))
                return len(packet)
        uart = UART()
        clock = iter(range(0, 10000, 100))
        original = getattr(w.time, "ticks_us", None)
        w.time.ticks_us = lambda: next(clock)
        try:
            self.hello(sequence=2, now=110)
            self.link.service_tx(uart, 110)
            self.assertEqual(w.decode_frame(uart.sent[0])["type"], w.TYPE_HELLO_REPLY)
            self.assertEqual(self.command(14, sequence=3, now=120), w.ACK_OK)
            self.link.service_tx(uart, 120)
            self.assertEqual(w.decode_frame(uart.sent[-1])["type"], w.TYPE_ACK)
        finally:
            if original is None:
                del w.time.ticks_us
            else:
                w.time.ticks_us = original

    def test_stuck_fifo_wait_is_bounded_and_preserves_reply(self):
        class UART:
            def fifo_txdone(self):
                return False
            def write(self, packet):
                raise AssertionError("busy FIFO must not write")
        clock = iter(range(0, 10000, 100))
        original = getattr(w.time, "ticks_us", None)
        w.time.ticks_us = lambda: next(clock)
        try:
            self.hello(sequence=2, now=110)
            queued = list(self.link.queue)
            self.link.service_tx(UART(), 110)
            self.assertEqual(self.link.queue, queued)
        finally:
            if original is None:
                del w.time.ticks_us
            else:
                w.time.ticks_us = original

    def test_wire_crc_and_roundtrip(self):
        packet = w.encode_frame(3, 99, 123, 100, b"example")
        self.assertEqual(w._u32(packet, len(packet)-4), zlib.crc32(packet[:-4]))
        self.assertEqual(w.decode_frame(packet)["payload"], b"example")
        packet[-1] ^= 1
        self.assertIsNone(w.decode_frame(packet))

    def test_live_pd_is_atomic_and_used_in_control(self):
        p = bytearray((1, 2, 3, 0, 0, 0, 0, 17, 0, 0, 0, 0))
        w._put_u32(p, 3, 1500); w._put_u32(p, 8, 500)
        self.assertEqual(self.command(4, extra=p), w.ACK_OK)
        info = dict(position_p=-400, derivative_d=200, pd_output=-200)
        self.link.apply_control(info, 1000)
        self.assertEqual(info, dict(position_p=-600, derivative_d=100, pd_output=-500))
        p[-1] = 127
        self.assertEqual(self.command(4, sequence=3, extra=p), w.ACK_RANGE)
        self.assertEqual((self.link.vision_p, self.link.vision_d), (1500, 500))

    def test_duplicate_never_reapplies_operation(self):
        p = bytearray((1, 1, 3, 0, 0, 0, 0))
        w._put_u32(p, 3, 1300)
        self.command(4, extra=p)
        generation = self.link.generation
        self.link.vision_p = 900
        self.command(4, extra=p)
        self.assertEqual(self.link.vision_p, 900)
        self.assertEqual(self.link.generation, generation)

    def test_stale_or_wrong_session_cannot_apply(self):
        self.link.vision_p = 1300
        self.command(6, now=200, timestamp=0xffff0000)
        self.assertEqual(self.link.vision_p, 1300)
        self.feed(w.encode_frame(3, 3, 456, 110, bytes((6, 1, 0, 0))), 110)
        self.assertEqual(self.link.vision_p, 1300)

    def test_save_default_and_calibration_require_parked(self):
        self.hello(parked=0, sequence=2, now=120)
        for sequence, op in enumerate((5, 6, 10), 3):
            self.assertEqual(self.command(op, sequence=sequence, now=130), w.ACK_NOT_PARKED)
        self.assertFalse(self.link.calibration_requested)

    def test_width_calibration_duplicate_does_not_restart(self):
        self.assertEqual(self.command(10), w.ACK_PENDING)
        self.link.calibration_requested = False
        self.command(10)
        self.assertFalse(self.link.calibration_requested)
        self.link.calibration_progress(120, 64, 80, True, True)
        self.assertIsNone(self.link.calibration_sequence)
        self.assertEqual(self.link.cached_ack[0], w.ACK_OK)



    def test_outer_payload_cannot_forge_reverse_keep(self):
        reset = r.pack_frame(r.COMMAND_RESET, 10)
        self.reverse.feed(reset, len(reset), 100)
        self.feed(w.encode_frame(20, 2, 123, 110, r.pack_frame(r.COMMAND_KEEP, 11)), 110)
        self.assertFalse(self.reverse.single_enabled)
        keep = r.pack_frame(r.COMMAND_KEEP, 11)
        self.feed(keep, 110)
        self.assertTrue(self.reverse.single_enabled)

    def test_bad_outer_crc_does_not_forge_reverse_keep(self):
        reset = r.pack_frame(r.COMMAND_RESET, 10)
        self.reverse.feed(reset, len(reset), 100)
        frame = w.encode_frame(20, 2, 123, 110, r.pack_frame(r.COMMAND_KEEP, 11))
        frame[-1] ^= 1
        self.feed(frame, 110)
        self.assertFalse(self.reverse.single_enabled)

    def test_queue_is_bounded(self):
        for sequence in range(2, 40):
            self.command(14, sequence)
        self.assertLessEqual(len(self.link.queue), 4)
        self.assertGreater(self.link.dropped, 0)

    def test_six_saved_profiles_and_crc_fallback(self):
        with tempfile.TemporaryDirectory() as temp:
            path = str(Path(temp) / "settings.json")
            store = w.ProfileStore(path)
            for key in range(6):
                store.save(key, 700 + key * 100, 1500 - key * 100)
            loaded = w.ProfileStore(path)
            self.assertEqual(loaded.rows, [[700+k*100, 1500-k*100] for k in range(6)])
            Path(path).write_text('{"invalid":1}')
            fallback = w.ProfileStore(path)
            self.assertEqual(fallback.rows[:5], loaded.rows[:5])
            self.assertEqual(fallback.rows[5], [1000, 1000])

    def test_v4_wire_layout(self):
        packet = bytearray(16)
        v.pack_into_v4(packet, 11, 1, 90, -100, 0, 2, 800, 3, 2, -1, 0)
        self.assertEqual(packet[2], 4)
        self.assertEqual(packet[10], 2 | 8 | 16)
        self.assertEqual(packet[11], 3)
        self.assertEqual(packet[14] | packet[15] << 8, v.crc16_ccitt_false(packet[2:14]))
        with self.assertRaises(ValueError):
            v.pack_into_v4(packet, 11, 0, 0, 0, ordinary_route=1)

if __name__ == "__main__":
    unittest.main()
