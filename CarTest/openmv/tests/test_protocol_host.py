"""Host-only checks for the CarTest OpenMV UART v1 transmitter."""

import csv
import importlib.util
import re
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VECTOR_FILE = Path(__file__).with_name("vision-protocol-v1.csv")
sys.path.insert(0, str(PACKAGE_DIR))

import vision_protocol  # noqa: E402
import config  # noqa: E402
import fast_boundary  # noqa: E402
import fast_gray  # noqa: E402
import fast_path  # noqa: E402
import fast_topology  # noqa: E402

CLIPPED_PROTOCOL_PATH = (
    PACKAGE_DIR.parent / "openmv_clipped_midpoint" / "vision_protocol.py")
CLIPPED_SPEC = importlib.util.spec_from_file_location(
    "vision_protocol_clipped", CLIPPED_PROTOCOL_PATH)
vision_protocol_clipped = importlib.util.module_from_spec(CLIPPED_SPEC)
CLIPPED_SPEC.loader.exec_module(vision_protocol_clipped)
PACKERS = (
    ("openmv", vision_protocol),
    ("openmv_clipped_midpoint", vision_protocol_clipped),
)


class VisionProtocolTests(unittest.TestCase):
    def test_package_build_and_runtime_api_identity(self):
        expected_build = "CACHED_WIDTH_CORRECTION_FIRST_20261015"
        self.assertEqual(config.BUILD_ID, expected_build)
        self.assertEqual(config.V7_RUNTIME_API_VERSION, 7)
        self.assertEqual(config.SINGLE_EDGE_ROUTE_MODE, "cached_width")
        main_source = (PACKAGE_DIR / "main.py").read_text(encoding="utf-8")
        self.assertRegex(
            main_source,
            r'(?m)^V7_BUILD_ID = "CARTEST_OPENMV_VISION_V1_20261006"$')
        for module in (fast_gray, fast_path, fast_topology, fast_boundary):
            with self.subTest(module=module.__name__):
                self.assertEqual(module.RUNTIME_API_VERSION, 7)

    def test_crc_known_check_value(self):
        for name, packer in PACKERS:
            with self.subTest(package=name):
                self.assertEqual(
                    packer.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_all_shared_vectors_match_exact_bytes(self):
        with VECTOR_FILE.open("r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 10)
        for package_name, packer in PACKERS:
            for row in rows:
                with self.subTest(package=package_name, name=row["name"]):
                    frame = packer.pack_frame(
                        int(row["seq"]), int(row["valid"]),
                        int(row["confidence"]), int(row["lateral_error"]),
                        int(row.get("slope_hint") or 0))
                    self.assertEqual(bytes(frame).hex(), row["hex"])

    def test_layout_and_little_endian_fields(self):
        for package_name, packer in PACKERS:
            with self.subTest(package=package_name):
                frame = packer.pack_frame(0x1234, 1, 88, -456)
                self.assertEqual(len(frame), 12)
                self.assertEqual(bytes(frame[:4]), b"\xA5\x5A\x01\x04")
                self.assertEqual(bytes(frame[4:6]), b"\x34\x12")
                self.assertEqual(frame[6:8], bytearray((1, 88)))
                self.assertEqual(bytes(frame[8:10]), b"\x38\xFE")
                expected_crc = packer.crc16_ccitt_false(frame[2:10])
                self.assertEqual(frame[10] | frame[11] << 8, expected_crc)

    def test_invalid_observation_has_zero_confidence_and_error(self):
        for package_name, packer in PACKERS:
            with self.subTest(package=package_name):
                frame = packer.pack_frame(7, 0, 0, 0)
                self.assertEqual(frame[6:10], bytearray(4))

    def test_slope_hint_flag_is_opt_in_and_preserves_legacy_calls(self):
        with VECTOR_FILE.open("r", newline="", encoding="ascii") as stream:
            legacy_vector = next(
                row for row in csv.DictReader(stream)
                if row["name"] == "normal")
        for package_name, packer in PACKERS:
            with self.subTest(package=package_name):
                legacy = packer.pack_frame(1, 1, 75, 123)
                self.assertEqual(legacy[6], packer.FLAG_VALID)
                self.assertEqual(bytes(legacy).hex(), legacy_vector["hex"])
                self.assertEqual(
                    bytes(packer.pack(1, 1, 75, 123)), bytes(legacy))

                hinted = packer.pack_frame(6, 1, 77, 222, 1)
                self.assertEqual(
                    hinted[6], packer.FLAG_VALID | packer.FLAG_SLOPE_HINT)
                self.assertEqual(hinted[6] & ~packer.KNOWN_FLAGS_MASK, 0)
                self.assertEqual(len(hinted), packer.FRAME_SIZE)
                self.assertEqual(
                    hinted[10] | hinted[11] << 8,
                    packer.crc16_ccitt_false(hinted[2:10]))

    def test_slope_hint_is_boolean_and_invalid_frames_must_clear_all_flags(self):
        for package_name, packer in PACKERS:
            for hint in (-1, 2, 1.0):
                with self.subTest(package=package_name, hint=hint):
                    with self.assertRaises((TypeError, ValueError)):
                        packer.pack_frame(1, 1, 1, 0, hint)
            with self.subTest(package=package_name, invalid_hint=1):
                with self.assertRaises(ValueError):
                    packer.pack_frame(1, 0, 0, 0, 1)
            invalid = packer.pack_frame(1, 0, 0, 0)
            with self.subTest(package=package_name, invalid_frame=True):
                self.assertEqual(invalid[6], 0)

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
