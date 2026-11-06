"""Host tests for optional, bounded camera records obtained through H743."""
import struct
import unittest
import zlib

from tools import camera_uart_diag_reader as reader


def snapshot(entries=(), rejected=0):
    raw = bytearray(reader.SIZE)
    struct.pack_into("<IHHII", raw, 0, reader.MAGIC, 1, reader.SIZE, 2, 0)
    struct.pack_into("<II", raw, 16, len(entries), rejected)
    count = min(len(entries), reader.CAPACITY)
    struct.pack_into("<HHHHI", raw, 52,
                     entries[-1][1] if entries else 0, count,
                     len(entries) % reader.CAPACITY, 0,
                     max(0, len(entries) - reader.CAPACITY))
    seen = 0
    for index, (time_ms, sequence, payload) in enumerate(entries):
        struct.pack_into("<I", raw, 24, time_ms)
        kind = payload[0]
        if kind == 1:
            raw[36:44] = payload
            struct.pack_into("<I", raw, 28, time_ms)
            seen |= 1
        elif kind == 2:
            raw[44:52] = payload
            struct.pack_into("<I", raw, 32, time_ms)
            seen |= 2
        elif kind == 3:
            raw[64:72] = payload
            struct.pack_into("<I", raw, 72, time_ms)
            seen |= 4
        offset = reader.HEADER_SIZE + (index % reader.CAPACITY) * reader.RECORD_SIZE
        struct.pack_into("<IHH", raw, offset, time_ms, sequence, 0)
        raw[offset + 8:offset + 16] = payload
    struct.pack_into("<H", raw, 58, seen)
    return seal(raw)


def seal(raw):
    raw = bytearray(raw)
    raw[12:16] = bytes(4)
    struct.pack_into("<I", raw, 12, zlib.crc32(raw) & 0xFFFFFFFF)
    return bytes(raw)


STATUS = bytes((1, 0x1D, 2, 2, 2, 0x30, 17, 0))
COUNTERS = bytes((2, 0, 1, 0, 3, 0, 4, 0))
IDENTITY = bytes((3, 1, 0x29, 0x10, 0x26, 0x20, 1, 0))


class CameraUARTReaderTests(unittest.TestCase):
    def test_actual_ack_and_missing_calibration_are_distinct_from_raw_ack(self):
        result = reader.decode_camera_uart_snapshot(
            snapshot(((100, 0, IDENTITY), (200, 1, STATUS), (300, 2, COUNTERS))), 400)
        self.assertTrue(result["fresh"])
        state = result["latest_status"]
        self.assertEqual(state["calibration_reason"], "FILE_NOT_FOUND")
        self.assertEqual(state["raw_profile_ack"], 3)
        self.assertEqual(state["actual_transmitted_ack"], 0)
        self.assertTrue(state["calibration_gate_blocked"])
        self.assertFalse(state["actual_observation_valid"])
        self.assertEqual(result["latest_counters"]["detector_errors"], 3)
        self.assertEqual(result["latest_identity"]["build_code"], "0x20261029")
        self.assertFalse(result["is_full_text_camera_log"])

    def test_staleness_uses_status_age_not_more_recent_counter(self):
        raw = snapshot(((10, 1, STATUS), (5009, 2, COUNTERS)))
        self.assertTrue(reader.decode_camera_uart_snapshot(raw, 5009)["fresh"])
        result = reader.decode_camera_uart_snapshot(raw, 5010)
        self.assertFalse(result["fresh"])
        self.assertEqual(result["status"], "stale")
        self.assertTrue(result["latest_counters"]["fresh"])

    def test_wrapped_clock_and_future_nonatomic_read(self):
        raw = snapshot(((0xFFFFFFF0, 1, STATUS),))
        self.assertEqual(reader.decode_camera_uart_snapshot(raw, 20)["latest_status"]["age_ms"], 36)
        result = reader.decode_camera_uart_snapshot(snapshot(((100, 1, STATUS),)), 99)
        self.assertFalse(result["fresh"])
        self.assertTrue(result["latest_status"]["clock_order_uncertain"])

    def test_bounded_ring_is_returned_chronologically(self):
        result = reader.decode_camera_uart_snapshot(
            snapshot(tuple((i * 100, i, STATUS) for i in range(20))), 2000)
        self.assertEqual(result["overwritten_count"], 4)
        self.assertEqual([r["diagnostic_sequence"] for r in result["records"]], list(range(4, 20)))

    def test_uninitialized_empty_and_counters_only_do_not_claim_status(self):
        self.assertEqual(reader.decode_camera_uart_snapshot(bytes(reader.SIZE), 0)["status"], "uninitialized")
        for raw in (snapshot(), snapshot(((10, 1, COUNTERS),))):
            result = reader.decode_camera_uart_snapshot(raw, 11)
            self.assertFalse(result["fresh"])
            self.assertEqual(result["status"], "status_not_received")

    def test_torn_corrupt_odd_and_wrong_size_are_not_current(self):
        raw = bytearray(snapshot(((10, 1, STATUS),)))
        raw[100] ^= 1
        self.assertEqual(reader.decode_camera_uart_snapshot(raw, 20)["status"], "torn_or_corrupt")
        struct.pack_into("<I", raw, 8, 3)
        self.assertEqual(reader.decode_camera_uart_snapshot(seal(raw), 20)["status"], "updating")
        self.assertFalse(reader.decode_camera_uart_snapshot(raw[:-1], 20)["fresh"])

    def test_bounds_flags_reserved_and_unknown_payloads_are_rejected(self):
        for offset, value in ((54, 17), (56, 16), (58, 8), (76, 1), (reader.HEADER_SIZE + 6, 1),
                              (36, 9), (37, 128), (38, 250), (41, 0x44)):
            with self.subTest(offset=offset):
                raw = bytearray(snapshot(((10, 1, STATUS),)))
                raw[offset] = value
                result = reader.decode_camera_uart_snapshot(seal(raw), 20)
                self.assertEqual(result["status"], "invalid")
                self.assertFalse(result["fresh"])

    def test_counter_saturation_is_not_hidden(self):
        raw = snapshot(((10, 1, bytes((2, 1, 255, 255, 2, 0, 3, 0))),))
        counters = reader.decode_camera_uart_snapshot(raw, 11)["latest_counters"]
        self.assertEqual(counters["vision_tx_errors"], 65535)
        self.assertTrue(counters["saturated"])


if __name__ == "__main__":
    unittest.main()
