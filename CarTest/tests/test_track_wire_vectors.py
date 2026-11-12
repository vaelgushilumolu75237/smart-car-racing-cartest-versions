"""The camera must produce the same bytes as the MCU's shared vectors."""
import csv
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location("track_wire_" + name,
                                                 ROOT / "openmv" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SharedWireVectorTests(unittest.TestCase):
    def test_reverse_profile_vectors(self):
        module = load("reverse_control")
        path = ROOT / "tests/data/reverse-profile-state-v2.csv"
        with path.open(encoding="utf-8-sig", newline="") as source:
            rows = list(csv.DictReader(source))
        self.assertGreaterEqual(len(rows), 4)
        for row in rows:
            with self.subTest(case=row["case"]):
                actual = module.pack_profile_state(
                    int(row["sequence"]), int(row["profile_id"]), int(row["active"]),
                    int(row["revision"]), int(row["session_token"]),
                    int(row["mean_forward_wheel_millirev"]))
                self.assertEqual(actual.hex().upper(), row["hex"])
                parser = module.ReverseControlParser()
                parser.feed(actual, len(actual), 100)
                expected_ack = int(row["profile_id"])
                if expected_ack not in (1, 2, 3):
                    expected_ack = 0
                self.assertEqual(parser.profile_ack(100, lambda a, b: a - b), expected_ack)

    def test_forward_profile_vectors(self):
        module = load("vision_protocol")
        path = ROOT / "tests/data/vision-protocol-v3.csv"
        with path.open(encoding="utf-8-sig", newline="") as source:
            rows = list(csv.DictReader(source))
        self.assertGreaterEqual(len(rows), 5)
        for row in rows:
            with self.subTest(case=row["case"]):
                packet = module.pack_frame_v3(
                    int(row["sequence"]), int(row["valid"]), int(row["confidence"]),
                    int(row["error"]), int(row["slope_hint"]), int(row["curve_phase"]),
                    int(row["slowdown_request_permille"]), int(row["profile_ack"]))
                self.assertEqual(packet.hex().upper(), row["hex"])


if __name__ == "__main__":
    unittest.main()
