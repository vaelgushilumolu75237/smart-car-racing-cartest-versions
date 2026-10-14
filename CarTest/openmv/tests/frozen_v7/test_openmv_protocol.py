import csv
import sys
import unittest
from pathlib import Path


sys.path.insert(0, "openmv")
import vision_protocol


class SharedProtocolVectorTests(unittest.TestCase):
    def test_micro_python_encoder_matches_shared_csv(self):
        csv_path = Path("test-data/vision-protocol-v1.csv")
        with csv_path.open("r", newline="") as source:
            rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 9)
        for vector in rows:
            with self.subTest(name=vector["name"]):
                packed = vision_protocol.pack_frame(
                    int(vector["seq"]), int(vector["valid"]),
                    int(vector["confidence"]),
                    int(vector["lateral_error"]))
                self.assertEqual(bytes(packed).hex(), vector["hex"])

    def test_crc_check_string(self):
        self.assertEqual(vision_protocol.crc16_ccitt_false(
            b"123456789"), 0x29B1)

    def test_invalid_frame_payload_must_be_zero(self):
        for confidence, error in ((1, 0), (0, 1)):
            with self.subTest(confidence=confidence, error=error):
                with self.assertRaises(ValueError):
                    vision_protocol.pack_frame(1, 0, confidence, error)


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(
        stream=sys.stdout, verbosity=2))
