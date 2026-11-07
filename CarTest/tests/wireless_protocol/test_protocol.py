import struct
import unittest

from tools.wireless.protocol import (
    Frame, FrameParser, FrameType, ProtocolError, SeekFree, decode_frame,
    encode_frame, pack_command, sequence_forward,
)


class WirelessProtocolTests(unittest.TestCase):
    def test_golden_frame_and_crc(self):
        frame = Frame(FrameType.COMMAND, 0x01020304, 0xAABBCCDD, 123456,
                      b"\x01\x02\x03")
        raw = encode_frame(frame)
        self.assertEqual(raw.hex(),
                         "d39101030300000004030201ddccbbaa40e20100010203"
                         "927f74bb")
        self.assertEqual(decode_frame(raw), frame)

    def test_parser_noise_truncation_crc_and_reserved(self):
        raw = encode_frame(Frame(16, 2, 3, 4, b"abc"))
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + raw[:5]), [])
        self.assertEqual(parser.feed(raw[5:]), [Frame(16, 2, 3, 4, b"abc")])
        corrupt = bytearray(raw)
        corrupt[-1] ^= 1
        parser = FrameParser()
        self.assertEqual(parser.feed(corrupt), [])
        self.assertEqual(parser.crc_errors, 1)
        reserved = bytearray(raw)
        reserved[6] = 1
        parser = FrameParser()
        self.assertEqual(parser.feed(reserved), [])
        self.assertEqual(parser.reserved_errors, 1)

    def test_bounds_and_command_payload(self):
        with self.assertRaises(ProtocolError):
            encode_frame(Frame(1, 1, 1, 1, bytes(161)))
        self.assertEqual(pack_command(3, 1, b"x"), b"\x03\x01\x00\x00x")
        self.assertTrue(sequence_forward(0, 0xFFFFFFFF))
        self.assertFalse(sequence_forward(0xFFFFFFFF, 0))

    def test_seekfree_golden_frames(self):
        self.assertEqual(SeekFree.oscilloscope_config(2).hex(), "aabc100200000000")
        osc = SeekFree.oscilloscope([1.0, -2.0])
        self.assertEqual(len(osc), 16)
        self.assertEqual(osc.hex(), "aa3b1002000000000000803f000000c0")
        self.assertEqual(SeekFree.debug_param(1, 1.5).hex(),
                         "557420010000c03f")
        self.assertEqual(SeekFree.camera(SeekFree.CAMERA_BINARY, 8, 1,
                                         b"\xA5").hex(), "aab702010800010001000000a5")

    def test_seekfree_image_and_param_bounds(self):
        with self.assertRaises(ProtocolError):
            SeekFree.oscilloscope([0.0] * 17)
        with self.assertRaises(ProtocolError):
            SeekFree.debug_param(8, 0.0)
        with self.assertRaises(ProtocolError):
            SeekFree.camera(SeekFree.CAMERA_GRAY, 8, 1, b"\xA5")


if __name__ == "__main__":
    unittest.main()
