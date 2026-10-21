"""Offline integrity tests for the frozen PTRC trace exporter."""

import struct
import unittest
import zlib

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import run_status_reader as reader  # noqa: E402
import speed_pi_trace_export as trace_export  # noqa: E402


BUILD_ID = bytes.fromhex("00112233445566778899aabbccddeeff")


def make_sample(sample_index, record_ms, sequence=1):
    raw = bytearray(struct.pack(
        trace_export.TRACE_RECORD_FORMAT,
        sample_index, record_ms & 0xFFFFFFFF, (record_ms - 2) & 0xFFFFFFFF,
        sequence & 0xFFFF, (record_ms - 1) & 0xFFFFFFFF,
        2, reader.SPEED_PI_TRACE_FLAG_SPEED_VALID |
        reader.SPEED_PI_TRACE_FLAG_ARMED |
        reader.SPEED_PI_TRACE_FLAG_PROPOSAL_VALID,
        4200, 4100, 3900, 3800, 800, 790, 798, 788, 0))
    struct.pack_into("<I", raw, trace_export.TRACE_CRC_OFFSET, 0)
    struct.pack_into("<I", raw, trace_export.TRACE_CRC_OFFSET,
                     zlib.crc32(raw) & 0xFFFFFFFF)
    return bytes(raw)


def make_trace(total_samples=3, state=2, timestamp_start=100):
    capacity = reader.SPEED_PI_TRACE_CAPACITY
    count = min(total_samples, capacity)
    next_index = total_samples % capacity if total_samples >= capacity else total_samples
    first_slot = next_index if count == capacity else 0
    first_sample_index = total_samples - count
    records = [make_sample(first_sample_index + i, timestamp_start + i, i + 9)
               for i in range(count)]
    raw = bytearray(reader.SPEED_PI_TRACE_SIZE)
    for i, record in enumerate(records):
        slot = (first_slot + i) % capacity
        offset = reader.SPEED_PI_TRACE_HEADER_SIZE + slot * reader.SPEED_PI_TRACE_SAMPLE_SIZE
        raw[offset:offset + len(record)] = record
    records_crc = zlib.crc32(b"".join(records)) & 0xFFFFFFFF
    header = bytearray(reader.SPEED_PI_TRACE_HEADER_SIZE)
    struct.pack_into("<IHHIII", header, 0, reader.SPEED_PI_TRACE_MAGIC,
                     reader.SPEED_PI_TRACE_VERSION,
                     reader.SPEED_PI_TRACE_HEADER_SIZE, 2, 0,
                     records_crc if state == 2 else 0)
    struct.pack_into("<III", header, 20, 77 if state else 0,
                     90 if state else 0,
                     (timestamp_start + count) & 0xFFFFFFFF if state == 2 else 0)
    struct.pack_into("<3H", header, 32, capacity,
                     count if state else 0,
                     next_index if state else 0)
    struct.pack_into("<2B", header, 38, 2 if state else 0, state)
    struct.pack_into("<2I", header, 40, total_samples if state else 0,
                     max(0, total_samples - count) if state else 0)
    header[48:64] = BUILD_ID
    struct.pack_into("<I", header, 12, zlib.crc32(header) & 0xFFFFFFFF)
    raw[:64] = header
    return bytes(raw)


class SpeedPiTraceExportTests(unittest.TestCase):
    def test_frozen_trace_decodes_and_detects_ring_wrap_past_capacity(self):
        raw = make_trace(total_samples=reader.SPEED_PI_TRACE_CAPACITY + 5,
                         timestamp_start=0xFFFFFFF0)
        decoded = trace_export.decode_trace(raw, BUILD_ID.hex())
        self.assertEqual(decoded["sample_count"], reader.SPEED_PI_TRACE_CAPACITY)
        self.assertTrue(decoded["ring_wrapped"])
        self.assertEqual(decoded["lost_samples"], 5)
        self.assertEqual(decoded["records"][0]["sample_index"], 5)
        self.assertEqual(decoded["records"][-1]["sample_index"],
                         reader.SPEED_PI_TRACE_CAPACITY + 4)

    def test_record_crc_tamper_is_rejected(self):
        raw = bytearray(make_trace())
        raw[64 + 20] ^= 1
        with self.assertRaisesRegex(trace_export.TraceExportError,
                                    "record 0 CRC32 is invalid"):
            trace_export.decode_trace(bytes(raw), BUILD_ID.hex())

    def test_ring_crc_tamper_after_repaired_sample_crc_is_rejected(self):
        raw = bytearray(make_trace())
        record_offset = 64
        raw[record_offset + 20] ^= 1
        raw[record_offset + 44:record_offset + 48] = bytes(4)
        struct.pack_into("<I", raw, record_offset + 44,
                         zlib.crc32(raw[record_offset:record_offset + 48]) & 0xFFFFFFFF)
        with self.assertRaisesRegex(trace_export.TraceExportError,
                                    "chronological ring CRC32 is invalid"):
            trace_export.decode_trace(bytes(raw), BUILD_ID.hex())

    def test_active_and_idle_traces_do_not_create_exportable_csv_data(self):
        for state, name in ((0, "IDLE"), (1, "ACTIVE")):
            with self.subTest(state=name):
                raw = make_trace(state=state)
                with self.assertRaisesRegex(trace_export.TraceExportError,
                                            "only FROZEN traces can be exported"):
                    trace_export.decode_trace(raw, BUILD_ID.hex())

    def test_selected_elf_build_id_must_match_trace(self):
        with self.assertRaisesRegex(trace_export.TraceExportError,
                                    "does not match the selected ELF"):
            trace_export.decode_trace(make_trace(), "ffeeddccbbaa99887766554433221100")

    def test_pi_diagnostic_session_must_match_trace(self):
        pi_diagnostic = {
            "frozen": True,
            "start_token": 78,
            "start_ms": 90,
            "pi_type": 2,
        }
        with self.assertRaisesRegex(trace_export.TraceExportError,
                                    "does not match the frozen PI diagnostic session"):
            trace_export.decode_trace(make_trace(), BUILD_ID.hex(), pi_diagnostic=pi_diagnostic)

    def test_runtime_diagnostic_session_token_must_match_trace(self):
        runtime_diagnostic = {"start_token": 78}
        with self.assertRaisesRegex(trace_export.TraceExportError,
                                    "does not match the runtime diagnostic"):
            trace_export.decode_trace(make_trace(), BUILD_ID.hex(),
                                      runtime_diagnostic=runtime_diagnostic)


if __name__ == "__main__":
    unittest.main(verbosity=2)
