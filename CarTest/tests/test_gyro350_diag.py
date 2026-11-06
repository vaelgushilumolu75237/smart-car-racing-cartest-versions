"""Verify new IMU/ring faults survive the existing status snapshot ABI."""
from pathlib import Path
import struct
import sys
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import run_status_reader as reader
import runtime_log_export as exporter


def snapshot(fault):
    raw = bytearray(64)
    raw[:4] = reader.ACKERMANN_SNAPSHOT_MAGIC
    raw[4] = 2
    raw[5] = 64
    raw[6] = (1 << 0) | (1 << 3) | (1 << 5)
    struct.pack_into("<hHhH", raw, 24, 0, 32768, 0, 1530)
    struct.pack_into("<5i", raw, 32, 7392, 7392, 7392, 0, 0)
    raw[56] = 8  # hard-fault-latched mode
    raw[57] = fault
    struct.pack_into("<I", raw, 60, zlib.crc32(raw[:60]) & 0xffffffff)
    return bytes(raw)


class GyroFaultDiagnosticTests(unittest.TestCase):
    def test_new_faults_decode_through_existing_snapshot(self):
        for code, name in ((16, "imu"), (17, "gyro_ring")):
            with self.subTest(code=code):
                result = reader.decode_ackermann_snapshot(snapshot(code))
                self.assertTrue(result["valid"], result.get("issues"))
                self.assertEqual(result["fault_reason_name"], name)
                self.assertTrue(result["fault_locked"])

    def test_unknown_fault_still_rejected(self):
        result = reader.decode_ackermann_snapshot(snapshot(18))
        self.assertFalse(result["valid"])
        self.assertTrue(any("fault_reason" in issue for issue in result["issues"]))


if __name__ == "__main__":
    unittest.main()
