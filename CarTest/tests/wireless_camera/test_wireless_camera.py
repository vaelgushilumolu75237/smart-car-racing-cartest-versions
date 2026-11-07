import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OPENMV = os.path.join(ROOT, "openmv")
if OPENMV not in sys.path:
    sys.path.insert(0, OPENMV)

import wireless_link as wl


class FakeClock:
    def __init__(self, value=1000):
        self.value = value

    def ticks_ms(self):
        return self.value


class FakeReverse:
    def __init__(self):
        self.buffered_bytes = 0
        self.bytes = bytearray()

    def feed(self, data, length, _now):
        self.bytes.extend(data[:length])
        self.buffered_bytes = len(self.bytes)


class Sink:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame)


def hello(epoch=0x12345678, parked=True, sequence=1, timestamp=1000):
    payload = bytearray(8)
    payload[0:4] = int(epoch).to_bytes(4, "little")
    payload[4] = 1 if parked else 0
    return wl.encode_frame(wl.TYPE_HELLO, sequence, epoch, timestamp, payload)


def command(opcode, sequence, session, timestamp, body=b"", target=wl.TARGET_CAMERA):
    payload = bytearray(4 + len(body))
    payload[0] = opcode
    payload[1] = target
    payload[4:] = body
    return wl.encode_frame(wl.TYPE_COMMAND, sequence, session, timestamp, payload)


def ack_payload(body):
    return bytes((body[0], body[1], body[2], body[3])) + body[4:16]


class WirelessCameraTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.sink = Sink()
        self.capture_count = 0

        def capture():
            self.capture_count += 1
            return bytes((i & 0xFF for i in range(320 * 240))), 320, 240

        self.link = wl.WirelessCameraLink(
            enabled=True, parked_cb=lambda: True, capture_gray=capture,
            clock=self.clock, state_period_ms=50, diag_period_ms=500,
            binary_period_ms=500)

    def feed(self, data, now=1000):
        self.link.feed_bytes(data, len(data), now)

    def test_codec_rejects_bad_crc_and_accepts_split_frame(self):
        frame = wl.encode_frame(wl.TYPE_DIAG, 4, 7, 100, b"abc")
        parser = wl.OuterParser()
        result = None
        for value in frame[:8]:
            self.assertIsNone(parser.feed(value))
        for value in frame[8:]:
            result = parser.feed(value)
        self.assertEqual(result["type"], wl.TYPE_DIAG)
        damaged = bytearray(frame)
        damaged[-1] ^= 0x80
        for value in damaged:
            self.assertIsNone(parser.feed(value))
        self.assertEqual(parser.crc_errors, 1)

    def test_hello_clock_and_parked_state(self):
        self.feed(hello())
        self.assertEqual(self.link.session, 0x12345678)
        self.assertTrue(self.link.hello_parked)
        self.assertEqual(len(self.link.tx_queue), 1)
        # Repeated H743 HELLO does not reset the strict command sequence.
        self.feed(hello(sequence=2))
        self.assertTrue(self.link.command_seq_valid is False)
        self.assertEqual(self.link.session, 0x12345678)

    def test_replay_is_cached_and_stale_is_rejected(self):
        self.feed(hello())
        body = bytes((1, 1, 3, 0)) + bytes((wl.PARAM_VISION_P, 0xE8, 0x03, 0, 0))
        request = command(wl.OP_SET_PARAMS, 10, self.link.session, 1000, body)
        self.feed(request)
        self.assertEqual(self.capture_count, 0)
        self.assertEqual(self.link.vision_p_permille, 1000)
        # Same request sequence is a duplicate and cannot execute again.
        self.feed(request)
        self.assertEqual(self.link.parser.frames_ok, 3)
        stale = command(wl.OP_CAPTURE, 11, self.link.session, 0)
        self.feed(stale, now=1001)
        self.assertIsNone(self.link.image_data)
        self.assertEqual(self.link.command_errors, 1)

    def test_multiplex_does_not_expose_image_a6_5a_to_reverse(self):
        reverse = FakeReverse()
        image_like = bytes((0xA6, 0x5A, 0x00, 0xFF))
        outer = wl.encode_frame(wl.TYPE_IMAGE, 2, 0x77, 1100, image_like)
        self.link.feed_multiplexed(outer, len(outer), 1100, reverse)
        self.assertEqual(bytes(reverse.bytes), b"")
        # A genuine legacy byte stream is forwarded when no outer candidate
        # is active; this also proves the split does not swallow reverse data.
        reverse.bytes[:] = b""
        reverse.buffered_bytes = 0
        self.link.feed_multiplexed(b"\xA6\x5A", 2, 1101, reverse)
        self.assertEqual(bytes(reverse.bytes), b"\xA6\x5A")

    def test_capture_snapshot_is_single_immutable_stream(self):
        self.feed(hello())
        request = command(wl.OP_CAPTURE, 20, self.link.session, 1000)
        self.feed(request)
        self.assertEqual(self.capture_count, 1)
        self.assertEqual(len(self.link.image_data), 320 * 240)
        offsets = []
        chunks = []
        while self.link.image_data is not None or self.link.tx_queue:
            self.link.service_tx(self.sink, 1000)
            while self.sink.frames:
                outgoing = self.sink.frames.pop(0)
                frame = wl.decode_frame(outgoing)
                if frame and frame["type"] == wl.TYPE_IMAGE:
                    payload = frame["payload"]
                    frame_id = int.from_bytes(payload[0:4], "little")
                    offset = int.from_bytes(payload[4:8], "little")
                    total = int.from_bytes(payload[8:12], "little")
                    width = int.from_bytes(payload[12:14], "little")
                    height = int.from_bytes(payload[14:16], "little")
                    self.assertEqual((width, height, payload[16]), (320, 240, wl.IMAGE_GRAY))
                    self.assertLessEqual(len(payload) - wl.IMAGE_META_SIZE, 128)
                    offsets.append((frame_id, offset, total))
                    chunks.append(bytes(payload[wl.IMAGE_META_SIZE:]))
            if len(offsets) > 700:
                self.fail("capture did not finish within bounded chunks")
        self.assertEqual(len(offsets), (320 * 240 + 127) // 128)
        self.assertEqual([item[1] for item in offsets], list(range(0, 320 * 240, 128)))
        self.assertEqual(b"".join(chunks), bytes((i & 0xFF for i in range(320 * 240))))

    def test_rates_and_queue_are_bounded(self):
        self.feed(hello())
        mask = {"road_mask": bytearray([1] * (80 * 60))}
        for index in range(100):
            self.clock.value = 1000 + index * 10
            if index in (41, 82):
                self.feed(hello(sequence=index + 2,
                                timestamp=self.clock.value),
                          now=self.clock.value)
            self.link.publish_frame(self.clock.value, index, True, 90, "road",
                                    "IDLE", "NONE", 2, 3, 4, 20.0, 0, 0,
                                    1, False, 0, "OK", mask_info=mask)
            self.assertLessEqual(len(self.link.tx_queue), 8)
            self.link.service_tx(self.sink, self.clock.value)
        output = [wl.decode_frame(item) for item in self.sink.frames]
        self.assertTrue(all(item is not None for item in output))
        self.assertGreaterEqual(sum(item["type"] == wl.TYPE_STATE for item in output), 15)
        self.assertGreaterEqual(sum(item["type"] == wl.TYPE_DIAG for item in output), 1)


if __name__ == "__main__":
    unittest.main()
