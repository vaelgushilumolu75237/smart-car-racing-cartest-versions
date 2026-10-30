"""Host-only checks for the CarTest OpenMV UART V2/V3 transmitter."""

import csv
import re
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VECTOR_FILE = Path(__file__).with_name("vision-protocol-v2.csv")
V3_VECTOR_FILE = (
    PACKAGE_DIR.parent / "tests" / "data" / "vision-protocol-v3.csv")
V3_INVALID_ACK_VECTOR_FILE = (
    PACKAGE_DIR.parent / "tests" / "data" /
    "vision-protocol-v3-invalid-ack.csv")
sys.path.insert(0, str(PACKAGE_DIR))

import vision_protocol  # noqa: E402
import config  # noqa: E402
import fast_boundary  # noqa: E402
import fast_gray  # noqa: E402
import fast_path  # noqa: E402
import fast_topology  # noqa: E402

class VisionProtocolTests(unittest.TestCase):
    def test_package_build_and_runtime_api_identity(self):
        self.assertIsInstance(config.BUILD_ID, str)
        self.assertTrue(config.BUILD_ID)
        self.assertEqual(config.V7_RUNTIME_API_VERSION, 7)
        self.assertEqual(config.SINGLE_EDGE_ROUTE_MODE, "cached_width")
        main_source = (PACKAGE_DIR / "main.py").read_text(encoding="utf-8")
        self.assertRegex(main_source, r'(?m)^V7_BUILD_ID = "[A-Z0-9_]+"$')
        for module in (fast_gray, fast_path, fast_topology, fast_boundary):
            with self.subTest(module=module.__name__):
                self.assertEqual(module.RUNTIME_API_VERSION, 7)

    def test_crc_known_check_value(self):
        self.assertEqual(vision_protocol.crc16_ccitt_false(b"123456789"),
                         0x29B1)

    def test_all_shared_vectors_match_exact_bytes(self):
        with VECTOR_FILE.open("r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 13)
        for row in rows:
            with self.subTest(name=row["name"]):
                frame = vision_protocol.pack_frame(
                    int(row["seq"]), int(row["valid"]),
                    int(row["confidence"]), int(row["lateral_error"]),
                    int(row["slope_hint"]), int(row["curve_phase"]),
                    int(row["slowdown_request_permille"]))
                self.assertEqual(bytes(frame).hex(), row["hex"])

    def test_layout_and_little_endian_fields(self):
        frame = vision_protocol.pack_frame(
            0x1234, 1, 88, -456, 1, 2, 0x0123)
        self.assertEqual(len(frame), 16)
        self.assertEqual(bytes(frame[:4]), b"\xA5\x5A\x02\x08")
        self.assertEqual(bytes(frame[4:6]), b"\x34\x12")
        self.assertEqual(frame[6:8], bytearray((3, 88)))
        self.assertEqual(bytes(frame[8:10]), b"\x38\xFE")
        self.assertEqual(frame[10:12], bytearray((2, 0)))
        self.assertEqual(bytes(frame[12:14]), b"\x23\x01")
        expected_crc = vision_protocol.crc16_ccitt_false(frame[2:14])
        self.assertEqual(frame[14] | frame[15] << 8, expected_crc)

    def test_invalid_observation_has_zero_confidence_and_error(self):
        frame = vision_protocol.pack_frame(65535, 0, 0, 0)
        self.assertEqual(frame[4:6], bytearray((0xFF, 0xFF)))
        self.assertEqual(frame[6:14], bytearray(8))
        self.assertEqual(
            frame[14] | frame[15] << 8,
            vision_protocol.crc16_ccitt_false(frame[2:14]))

    def test_slope_hint_flag_is_opt_in_and_preserves_legacy_calls(self):
        with VECTOR_FILE.open("r", newline="", encoding="ascii") as stream:
            legacy_vector = next(
                row for row in csv.DictReader(stream)
                if row["name"] == "normal")
        legacy = vision_protocol.pack_frame(1, 1, 75, 123)
        self.assertEqual(legacy[6], vision_protocol.FLAG_VALID)
        self.assertEqual(bytes(legacy).hex(), legacy_vector["hex"])
        self.assertEqual(bytes(vision_protocol.pack(1, 1, 75, 123)),
                         bytes(legacy))

        hinted = vision_protocol.pack_frame(6, 1, 77, 222, 1)
        self.assertEqual(hinted[6],
                         vision_protocol.FLAG_VALID |
                         vision_protocol.FLAG_SLOPE_HINT)
        self.assertEqual(hinted[6] & ~vision_protocol.KNOWN_FLAGS_MASK, 0)
        self.assertEqual(len(hinted), vision_protocol.FRAME_SIZE)
        self.assertEqual(
            hinted[14] | hinted[15] << 8,
            vision_protocol.crc16_ccitt_false(hinted[2:14]))

    def test_slope_hint_is_boolean_and_invalid_frames_must_clear_all_flags(self):
        for hint in (-1, 2, 1.0):
            with self.subTest(hint=hint):
                with self.assertRaises((TypeError, ValueError)):
                    vision_protocol.pack_frame(1, 1, 1, 0, hint)
        with self.assertRaises(ValueError):
            vision_protocol.pack_frame(0, 0, 0, 0, 1)
        invalid = vision_protocol.pack_frame(0, 0, 0, 0)
        self.assertEqual(invalid[4:14], bytearray(10))

    def test_pack_into_reuses_caller_buffer_and_preserves_tail(self):
        out = bytearray([0xCC] * 20)
        returned = vision_protocol.pack_into(
            out, 10, 1, 80, -250, 1, 2, 1000)
        self.assertIs(returned, out)
        self.assertEqual(len(out), 20)
        self.assertEqual(out[16:], bytearray([0xCC] * 4))
        self.assertEqual(out[2], vision_protocol.VERSION)
        self.assertEqual(out[3], vision_protocol.PAYLOAD_LENGTH)

        with self.assertRaises(ValueError):
            vision_protocol.pack_into(bytearray(15), 1, 1, 1, 0)
        with self.assertRaises(TypeError):
            vision_protocol.pack_into(bytes(16), 1, 1, 1, 0)

    def test_v3_profile_ack_uses_reserved_byte_and_preserves_v2_packer(self):
        legacy = vision_protocol.pack_frame(7, 1, 91, -333, 1, 2, 1000)
        self.assertEqual(legacy[2], 2)
        self.assertEqual(legacy[11], 0)

        for profile_ack in range(4):
            with self.subTest(profile_ack=profile_ack):
                frame = vision_protocol.pack_frame_v3(
                    7, 1, 91, -333, 1, 2, 1000, profile_ack)
                self.assertEqual(len(frame), 16)
                self.assertEqual(frame[2], vision_protocol.VERSION_V3)
                self.assertEqual(frame[:2], legacy[:2])
                self.assertEqual(frame[3:11], legacy[3:11])
                self.assertEqual(frame[11], profile_ack)
                self.assertEqual(frame[12:14], legacy[12:14])
                self.assertEqual(
                    frame[14] | frame[15] << 8,
                    vision_protocol.crc16_ccitt_false(frame[2:14]))

        invalid = vision_protocol.pack_frame_v3(
            8, 0, 0, 0, 0, 0, 0, 3)
        self.assertEqual(invalid[6:11], bytearray(5))
        self.assertEqual(invalid[11], 3)
        self.assertEqual(
            invalid[14] | invalid[15] << 8,
            vision_protocol.crc16_ccitt_false(invalid[2:14]))

        for profile_ack in (-1, 4, True, 1.0):
            with self.subTest(profile_ack=profile_ack):
                with self.assertRaises((TypeError, ValueError)):
                    vision_protocol.pack_frame_v3(
                        1, 1, 1, 0, profile_ack=profile_ack)

    def test_shared_v3_vectors_match_exact_bytes(self):
        with V3_VECTOR_FILE.open(
                "r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 5)
        self.assertEqual(
            list(rows[0]), ["case", "sequence", "valid", "confidence",
                            "error", "slope_hint", "curve_phase",
                            "slowdown_request_permille", "profile_ack",
                            "hex"])
        for row in rows:
            with self.subTest(case=row["case"]):
                frame = vision_protocol.pack_frame_v3(
                    int(row["sequence"]), int(row["valid"]),
                    int(row["confidence"]), int(row["error"]),
                    int(row["slope_hint"]), int(row["curve_phase"]),
                    int(row["slowdown_request_permille"]),
                    int(row["profile_ack"]))
                self.assertEqual(bytes(frame).hex(), row["hex"].lower())

    def test_shared_v3_invalid_ack_vectors_are_crc_valid_and_rejected(self):
        with V3_INVALID_ACK_VECTOR_FILE.open(
                "r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertGreaterEqual(len(rows), 1)
        self.assertEqual(list(rows[0]), ["case", "hex", "expected_status"])
        for row in rows:
            with self.subTest(case=row["case"]):
                frame = bytes.fromhex(row["hex"])
                self.assertEqual(len(frame), vision_protocol.FRAME_SIZE)
                self.assertEqual(frame[2], vision_protocol.VERSION_V3)
                self.assertEqual(frame[11], 4)
                self.assertEqual(
                    frame[14] | frame[15] << 8,
                    vision_protocol.crc16_ccitt_false(frame[2:14]))
                self.assertEqual(row["expected_status"], "BAD_HEADER")
                with self.assertRaises(ValueError):
                    vision_protocol.pack_frame_v3(
                        frame[4] | frame[5] << 8,
                        1 if frame[6] & vision_protocol.FLAG_VALID else 0,
                        frame[7],
                        ((frame[8] | frame[9] << 8) - 0x10000
                         if frame[9] & 0x80 else
                         frame[8] | frame[9] << 8),
                        1 if frame[6] & vision_protocol.FLAG_SLOPE_HINT else 0,
                        frame[10], frame[12] | frame[13] << 8, frame[11])

    def test_curve_phase_and_slowdown_bounds_and_invalid_payload_zero(self):
        for phase in (-1, 4, 255):
            with self.subTest(phase=phase):
                with self.assertRaises(ValueError):
                    vision_protocol.pack_frame(1, 1, 1, 0, 0, phase, 0)
        for request in (-1, 1001, 65535):
            with self.subTest(request=request):
                with self.assertRaises(ValueError):
                    vision_protocol.pack_frame(1, 1, 1, 0, 0, 0, request)
        for arguments in ((0, 0, 1, 0, 0, 0, 0),
                          (0, 0, 0, 1, 0, 0, 0),
                          (0, 0, 0, 0, 0, 1, 0),
                          (0, 0, 0, 0, 0, 0, 1)):
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    vision_protocol.pack_frame(*arguments)

    def test_out_of_range_and_malformed_inputs_are_rejected(self):
        invalid_arguments = (
            (-1, 1, 1, 0),
            (0x10000, 1, 1, 0),
            (1, 2, 1, 0),
            (1, 1, 101, 0),
            (1, 1, 1, 1001),
            (1, 1, 1, -1001),
            (1, 0, 1, 0),
            (1, 0, 0, 1),
            (1, 1.0, 1, 0),
        )
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments):
                with self.assertRaises((TypeError, ValueError)):
                    vision_protocol.pack_frame(*arguments)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(
        VisionProtocolTests)
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
