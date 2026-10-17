"""Offline validation tests using the logger's host-generated raw fixture."""

import csv
import os
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import runtime_log_export as exporter  # noqa: E402


ELF_PATH = os.environ.get("RUNTIME_LOG_TEST_ELF")
BUFFER_PATH = os.environ.get("RUNTIME_LOG_TEST_BUFFER")
OTHER_PROFILE_ELF = os.environ.get("RUNTIME_LOG_WRONG_PROFILE_ELF")


def refresh_crcs(buffer_bytes):
    data = bytearray(buffer_bytes)
    records = bytes(data[exporter.HEADER_SIZE:])
    struct.pack_into("<I", data, 36, zlib.crc32(records) & 0xFFFFFFFF)
    data[40:44] = b"\0\0\0\0"
    struct.pack_into("<I", data, 40, zlib.crc32(data[:64]) & 0xFFFFFFFF)
    return bytes(data)


def make_capacity_buffer(base, count, flags, dropped_count):
    """Expand a real C logger fixture into the exact ABI capacity variants."""
    source = bytearray(base)
    source_header = source[:64]
    source_count = struct.unpack_from("<I", source_header, 16)[0]
    source_records = [bytes(source[64 + i * 64:64 + (i + 1) * 64])
                      for i in range(source_count)]
    first = bytearray(source_records[0])
    last = bytearray(source_records[-1])
    middle = bytearray(first)
    struct.pack_into("<H", middle, 56, 0)
    start_ms = struct.unpack_from("<I", source_header, 24)[0]
    for record in (first, middle, last):
        struct.pack_into("<I", record, 0, start_ms)
    records = [bytes(first)] + [bytes(middle)] * (count - 2) + [bytes(last)]
    data = bytearray(64 + count * 64)
    data[:64] = source_header
    data[64:] = b"".join(records)
    struct.pack_into("<B", data, 15, flags)
    struct.pack_into("<I", data, 16, count)
    struct.pack_into("<I", data, 28, start_ms)
    struct.pack_into("<I", data, 32, dropped_count)
    return refresh_crcs(data)


class RuntimeLogExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not ELF_PATH or not BUFFER_PATH:
            raise unittest.SkipTest(
                "Set RUNTIME_LOG_TEST_ELF and RUNTIME_LOG_TEST_BUFFER to the sibling-built CarTest.elf and C logger fixture."
            )
        cls.elf = Path(ELF_PATH).resolve()
        cls.buffer_path = Path(BUFFER_PATH).resolve()
        cls.base = cls.buffer_path.read_bytes()
        cls.layout, cls.header, cls.rows = exporter.decode_buffer(cls.elf, cls.buffer_path)
        cls.header_bytes = cls.base[:64]

    def assert_rejected(self, data, message=None):
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = Path(temp_dir) / "broken.bin"
            raw_path.write_bytes(data)
            with self.assertRaises(exporter.ExportError, msg=message):
                exporter.decode_buffer(self.elf, raw_path)

    def replace_header(self, mutate):
        data = bytearray(self.base)
        mutate(data)
        return refresh_crcs(data)

    def make_header_with_identity(self, state, build_id=None):
        header = bytearray(self.header_bytes)
        struct.pack_into("<B", header, 14, state)
        if state == exporter.STATE_IDLE:
            struct.pack_into("<B", header, 15, 0)
            struct.pack_into("<6I", header, 16, 0, 0, 0, 0, 0, 0)
            struct.pack_into("<B", header, 60, 0)
        if build_id is not None:
            header[44:60] = build_id
        header[40:44] = bytes(4)
        struct.pack_into("<I", header, 40, zlib.crc32(header) & 0xFFFFFFFF)
        return bytes(header)

    def test_real_c_fixture_matches_elf_and_schema(self):
        expected_count = struct.unpack_from("<I", self.header_bytes, 16)[0]
        self.assertGreaterEqual(expected_count, 2)
        self.assertEqual(len(self.base), 64 + expected_count * 64)
        self.assertEqual(self.layout.storage_size, exporter.BUFFER_SIZE)
        self.assertEqual(self.header["build_id"], self.layout.build_id.hex())
        self.assertTrue(self.rows[0]["event_start"])
        self.assertTrue(self.rows[-1]["event_stop"])
        self.assertEqual(len(self.rows), expected_count)
        self.assertEqual(set(self.rows[0]), set(exporter.CSV_FIELDS))
        for field in ("vision_latest_valid", "vision_usable_fresh", "vision_timeout",
                      "vision_latest_frame_present", "recovery_active", "event_start"):
            self.assertIn(self.rows[0][field], (0, 1))

    def test_mode12_decodes_quantized_hold_elapsed_and_route_count(self):
        hold_rows = [row for row in self.rows if row["mode"] == 12]
        self.assertEqual(len(hold_rows), 1)
        hold = hold_rows[0]
        self.assertEqual(hold["vision_hold_active"], 1)
        self.assertEqual(hold["vision_hold_expired"], 0)
        hold_offset = next(64 + index * 64 for index in range(len(self.rows))
                           if self.base[64 + index * 64 + 58] == 12)
        expected_hold_ticks = self.base[hold_offset + 62] >> 2
        self.assertEqual(hold["vision_hold_elapsed_ms"],
                         self.header["hold_quantum_ms"] * expected_hold_ticks)
        self.assertEqual(hold["vision_hold_valid_count"], 2)
        self.assertEqual(hold["recovery_frame_count"], 2)
        for row in self.rows:
            if row["mode"] != 12:
                self.assertEqual(row["vision_hold_elapsed_ms"], 0)
                self.assertEqual(row["vision_hold_valid_count"], 0)

        corruptions = []
        too_long = bytearray(self.base)
        maximum_ticks = 50 if self.header["hold_quantum_ms"] == 20 else 60
        too_long[64 + 64 + 62] = ((maximum_ticks + 1) << 2) | 1
        corruptions.append(refresh_crcs(too_long))

        both_states = bytearray(self.base)
        both_states[64 + 64 + 61] |= (1 << 5)
        corruptions.append(refresh_crcs(both_states))

        missing_state = bytearray(self.base)
        missing_state[64 + 64 + 61] &= ~(1 << 4)
        corruptions.append(refresh_crcs(missing_state))

        non_hold_flags = bytearray(self.base)
        non_hold_flags[64 + 64 + 58] = 1
        corruptions.append(refresh_crcs(non_hold_flags))
        for data in corruptions:
            self.assert_rejected(data)

    def test_hold_button_events_and_exit_reason_decode(self):
        hold_rows = [row for row in self.rows if row["mode"] == 12]
        self.assertEqual(hold_rows[0]["event_hold_enter"], 1)
        self.assertEqual(self.rows[-1]["event_hold_exit"], 1)
        self.assertEqual(self.rows[-1]["vision_hold_exit_cause"], 1)
        self.assertEqual(self.rows[-1]["event_button_stop"], 1)

    def test_reserved_event_bits_and_invalid_exit_reason_are_rejected(self):
        corruptions = []
        reserved_bit = bytearray(self.base)
        struct.pack_into("<H", reserved_bit, 64 + 2 * 64 + 56, 1 << 15)
        corruptions.append(refresh_crcs(reserved_bit))

        cause_without_exit = bytearray(self.base)
        struct.pack_into("<H", cause_without_exit, 64 + 2 * 64 + 56,
                         (1 << 12) | (1 << 2))
        corruptions.append(refresh_crcs(cause_without_exit))

        invalid_cause = bytearray(self.base)
        struct.pack_into("<H", invalid_cause, 64 + 2 * 64 + 56,
                         (6 << 12) | (1 << 8))
        corruptions.append(refresh_crcs(invalid_cause))

        missing_cause = bytearray(self.base)
        struct.pack_into("<H", missing_cause, 64 + 2 * 64 + 56,
                         1 << 8)
        corruptions.append(refresh_crcs(missing_cause))

        for data in corruptions:
            self.assert_rejected(data)

    def test_offline_cli_writes_csv_and_metadata_only_after_full_validation(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            raw = temp / "run.bin"
            csv_path = temp / "run.csv"
            meta_path = temp / "run.metadata.json"
            raw.write_bytes(self.base)
            result = exporter.main(["decode", "--elf", str(self.elf), "--raw", str(raw),
                                    "--out-csv", str(csv_path), "--metadata", str(meta_path)])
            self.assertEqual(result, 0)
            self.assertTrue(csv_path.is_file())
            self.assertTrue(meta_path.is_file())
            with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), len(self.rows))
            self.assertEqual(rows[0]["encoder_left_count"], str(self.rows[0]["encoder_left_count"]))
            self.assertEqual(rows[0]["mode_name"], exporter.MODE_NAMES[self.rows[0]["mode"]])
            self.assertEqual(rows[0]["fault_name"], exporter.FAULT_NAMES[self.rows[0]["fault"]])
            self.assertEqual(exporter.STOP_REASON_NAMES[self.header["stop_reason"]],
                             self.header["stop_reason_name"])
            self.assertEqual(rows[-1]["servo_applied_us"], "0")

    def test_short_or_long_upload_is_rejected(self):
        self.assert_rejected(self.base[:-1])
        self.assert_rejected(self.base + b"\0")

    def test_header_magic_version_crc_build_id_and_reserved_bytes(self):
        self.assert_rejected(self.replace_header(lambda data: data.__setitem__(slice(0, 4), b"BAD!")))
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<H", data, 4, 3)))
        corrupt_crc = bytearray(self.base)
        corrupt_crc[40] ^= 0x01
        self.assert_rejected(bytes(corrupt_crc))
        self.assert_rejected(self.replace_header(lambda data: data.__setitem__(44, data[44] ^ 0x01)))
        self.assert_rejected(self.replace_header(lambda data: data.__setitem__(61, 1)))

    def test_matching_idle_and_active_headers_report_state_after_validation(self):
        cases = (
            (exporter.STATE_IDLE, "Runtime log is idle; no run has been recorded"),
            (exporter.STATE_ACTIVE, "Runtime log is active; stop and freeze the run before exporting"),
        )
        for state, expected in cases:
            with self.subTest(state=state):
                header = self.make_header_with_identity(state, self.layout.build_id)
                with self.assertRaises(exporter.ExportError) as caught:
                    exporter._parse_header(header, self.layout)
                self.assertEqual(str(caught.exception), expected)

    def test_mismatched_idle_active_and_frozen_headers_report_identity(self):
        wrong_build_id = bytearray(self.layout.build_id)
        wrong_build_id[0] ^= 0x80
        expected = (
            "Runtime log header build ID mismatch at RAM address "
            "0x{:08X}: selected CarTest.elf expects {}, header contains {}"
        )
        for state in (exporter.STATE_IDLE, exporter.STATE_ACTIVE,
                      exporter.STATE_FROZEN):
            with self.subTest(state=state):
                header = self.make_header_with_identity(state, bytes(wrong_build_id))
                with self.assertRaises(exporter.ExportError) as caught:
                    exporter._parse_header(header, self.layout)
                self.assertEqual(str(caught.exception), expected.format(
                    self.layout.storage_address,
                    self.layout.build_id.hex(), bytes(wrong_build_id).hex()))

    def test_corrupt_header_crc_precedes_build_id_and_state_errors(self):
        wrong_build_id = bytearray(self.layout.build_id)
        wrong_build_id[0] ^= 0x80
        header = bytearray(self.make_header_with_identity(
            exporter.STATE_IDLE, bytes(wrong_build_id)))
        header[40] ^= 0x01
        with self.assertRaises(exporter.ExportError) as caught:
            exporter._parse_header(bytes(header), self.layout)
        self.assertEqual(str(caught.exception), "Runtime log header CRC32 is invalid")

        header = bytearray(self.make_header_with_identity(
            exporter.STATE_IDLE, bytes(wrong_build_id)))
        header[61] = 1
        header[40:44] = bytes(4)
        struct.pack_into("<I", header, 40, zlib.crc32(header) & 0xFFFFFFFF)
        with self.assertRaises(exporter.ExportError) as caught:
            exporter._parse_header(bytes(header), self.layout)
        self.assertEqual(str(caught.exception),
                         "Runtime log header reserved bytes are not zero")

    def test_header_state_count_session_stop_reason_and_flag_semantics(self):
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<B", data, 14, 1)))
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<I", data, 16, 1)))
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<I", data, 16, exporter.CAPACITY + 1)))
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<I", data, 20, 0)))
        self.assert_rejected(self.replace_header(lambda data: struct.pack_into("<B", data, 60, 0)))

        def add_truncated_without_drop(data):
            data[15] |= exporter.HEADER_FLAG_TRUNCATED

        self.assert_rejected(self.replace_header(add_truncated_without_drop))

        def invert_full_bit(data):
            data[15] ^= exporter.HEADER_FLAG_FULL

        self.assert_rejected(self.replace_header(invert_full_bit))

    def test_clean_full_and_truncated_full_buffers_use_distinct_flags(self):
        clean = make_capacity_buffer(self.base, exporter.CAPACITY, exporter.HEADER_FLAG_FULL, 0)
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = Path(temp_dir) / "full.bin"
            raw_path.write_bytes(clean)
            _layout, header, rows = exporter.decode_buffer(self.elf, raw_path)
        self.assertEqual(len(rows), exporter.CAPACITY)
        self.assertTrue(header["buffer_full"])
        self.assertFalse(header["records_truncated"])
        self.assertEqual(header["dropped_count"], 0)

        truncated = make_capacity_buffer(self.base, exporter.CAPACITY,
                                         exporter.HEADER_FLAG_FULL | exporter.HEADER_FLAG_TRUNCATED, 3)
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = Path(temp_dir) / "truncated.bin"
            raw_path.write_bytes(truncated)
            _layout, header, _rows = exporter.decode_buffer(self.elf, raw_path)
        self.assertTrue(header["buffer_full"])
        self.assertTrue(header["records_truncated"])
        self.assertEqual(header["dropped_count"], 3)

    def test_records_crc_session_id_terminal_stop_and_time_relationship(self):
        bad_records_crc = bytearray(self.base)
        bad_records_crc[64 + 8] ^= 1
        self.assert_rejected(bytes(bad_records_crc))

        def mismatch_record_session(data):
            data[64 + 4] ^= 1

        self.assert_rejected(self.replace_header(mismatch_record_session))

        def clear_terminal_stop(data):
            count = struct.unpack_from("<I", data, 16)[0]
            offset = 64 + (count - 1) * 64 + 56
            struct.pack_into("<H", data, offset, struct.unpack_from("<H", data, offset)[0] & ~4)

        self.assert_rejected(self.replace_header(clear_terminal_stop))

        def set_terminal_servo_nonzero(data):
            count = struct.unpack_from("<I", data, 16)[0]
            struct.pack_into("<H", data, 64 + (count - 1) * 64 + 32, 1)

        self.assert_rejected(self.replace_header(set_terminal_servo_nonzero))

        def change_start_time(data):
            struct.pack_into("<I", data, 24, struct.unpack_from("<I", data, 24)[0] + 1)

        self.assert_rejected(self.replace_header(change_start_time))

    def test_elapsed_time_wrap_and_signed_64_bit_encoders(self):
        data = bytearray(self.base)
        count = struct.unpack_from("<I", data, 16)[0]
        start_ms = 0xFFFFFFFE
        stop_ms = (start_ms + count - 1) & 0xFFFFFFFF
        struct.pack_into("<I", data, 24, start_ms)
        struct.pack_into("<I", data, 28, stop_ms)
        for index in range(count):
            offset = 64 + index * 64
            struct.pack_into("<I", data, offset, (start_ms + index) & 0xFFFFFFFF)
        struct.pack_into("<qq", data, 64 + 8, -(1 << 62) + 17, (1 << 62) + 19)
        wrapped = refresh_crcs(data)
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = Path(temp_dir) / "wrapped.bin"
            raw_path.write_bytes(wrapped)
            _layout, header, rows = exporter.decode_buffer(self.elf, raw_path)
        self.assertEqual(rows[0]["t_ms"], start_ms)
        self.assertEqual(rows[-1]["t_ms"], stop_ms)
        self.assertEqual(rows[-1]["t_elapsed_ms"], count - 1)
        self.assertEqual(rows[0]["encoder_left_count"], -(1 << 62) + 17)
        self.assertEqual(rows[0]["encoder_right_count"], (1 << 62) + 19)

    def test_invalid_elf_target_and_other_selected_profile(self):
        original = self.elf.read_bytes()
        bad_machine = bytearray(original)
        bad_machine[18:20] = b"\0\0"
        with tempfile.TemporaryDirectory() as temp_dir:
            wrong_elf = Path(temp_dir) / "CarTest.elf"
            wrong_elf.write_bytes(bad_machine)
            with self.assertRaises(exporter.ExportError):
                exporter.inspect_elf(wrong_elf)

        if OTHER_PROFILE_ELF:
            other = exporter.inspect_elf(OTHER_PROFILE_ELF)
            self.assertNotEqual(other.build_id, self.layout.build_id)
            with tempfile.TemporaryDirectory() as temp_dir:
                raw_path = Path(temp_dir) / "wrong-profile.bin"
                raw_path.write_bytes(self.base)
                with self.assertRaises(exporter.ExportError):
                    exporter.decode_buffer(OTHER_PROFILE_ELF, raw_path)


class RuntimeLogVersionContractTests(unittest.TestCase):
    """Use the preserved 249-record V1 run to exercise real ELF/descriptor decoding."""

    elf = PROJECT_ROOT / "test-data" / "failure3-20261015" / "baseline" / "build" / "DebugFollowTurn" / "CarTest.elf"
    raw = PROJECT_ROOT / "logs" / "runs" / "20261015-234044-791-13908" / "runtime-log.bin"

    @classmethod
    def setUpClass(cls):
        if not cls.elf.is_file() or not cls.raw.is_file():
            raise unittest.SkipTest("Preserved V1 ELF and runtime-log fixture are unavailable")
        cls.base = cls.raw.read_bytes()

    @staticmethod
    def _replace_descriptor_version(source_elf, target_elf, version):
        blob, sections, symbols = exporter._parse_elf(source_elf)
        symbol = symbols[exporter.DESCRIPTOR_SYMBOL]
        section = sections[symbol.section_index]
        offset = section.offset + symbol.value - section.address
        changed = bytearray(blob)
        struct.pack_into("<H", changed, offset + 8, version)
        target_elf.write_bytes(changed)

    @staticmethod
    def _payload_with_version_and_hold(source, version, ticks):
        data = bytearray(source)
        struct.pack_into("<H", data, 4, version)
        record_count = struct.unpack_from("<I", data, 16)[0]
        hold_offsets = [64 + index * 64 for index in range(record_count)
                        if data[64 + index * 64 + 58] == 12]
        if not hold_offsets:
            raise AssertionError("V1 run fixture has no mode-12 record")
        struct.pack_into("<B", data, hold_offsets[0] + 62, ticks << 2)
        return refresh_crcs(data)

    def test_real_v1_elf_descriptor_and_249_record_run_decode(self):
        layout, header, rows = exporter.decode_buffer(self.elf, self.raw)
        self.assertEqual(layout.descriptor["log_version"], 1)
        self.assertEqual(header["hold_quantum_ms"], 20)
        self.assertEqual(len(rows), 249)
        with tempfile.TemporaryDirectory() as temp_dir:
            descriptor_path = Path(temp_dir) / "flash-rom-descriptor-test.bin"
            descriptor_path.write_bytes(layout.descriptor_bytes)
            self.assertTrue(exporter.validate_flash_descriptor(self.elf, descriptor_path)["matches_elf"])

    def test_v1_20ms_quantum_and_50_tick_maximum(self):
        good = self._payload_with_version_and_hold(self.base, 1, 50)
        with tempfile.TemporaryDirectory() as temp_dir:
            raw = Path(temp_dir) / "v1.bin"
            raw.write_bytes(good)
            _layout, header, rows = exporter.decode_buffer(self.elf, raw)
        hold = next(row for row in rows if row["mode"] == 12)
        self.assertEqual(header["hold_quantum_ms"], 20)
        self.assertEqual(hold["vision_hold_elapsed_ms"], 1000)
        with tempfile.TemporaryDirectory() as temp_dir:
            raw = Path(temp_dir) / "v1-too-long.bin"
            raw.write_bytes(self._payload_with_version_and_hold(self.base, 1, 51))
            with self.assertRaisesRegex(exporter.ExportError, "above 1000 ms"):
                exporter.decode_buffer(self.elf, raw)

    def test_v2_50ms_quantum_and_60_tick_maximum_follow_the_rom_descriptor(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            v2_elf = root / "CarTest.elf"
            self._replace_descriptor_version(self.elf, v2_elf, 2)
            self.assertEqual(exporter.inspect_elf(v2_elf).descriptor["log_version"], 2)
            good_raw = root / "v2.bin"
            good_raw.write_bytes(self._payload_with_version_and_hold(self.base, 2, 60))
            layout, header, rows = exporter.decode_buffer(v2_elf, good_raw)
            hold = next(row for row in rows if row["mode"] == 12)
            self.assertEqual(layout.descriptor["log_version"], 2)
            self.assertEqual(header["hold_quantum_ms"], 50)
            self.assertEqual(hold["vision_hold_elapsed_ms"], 3000)

            bad_raw = root / "v2-too-long.bin"
            bad_raw.write_bytes(self._payload_with_version_and_hold(self.base, 2, 61))
            with self.assertRaisesRegex(exporter.ExportError, "above 3000 ms"):
                exporter.decode_buffer(v2_elf, bad_raw)

            descriptor = root / "descriptor.bin"
            descriptor.write_bytes(layout.descriptor_bytes)
            self.assertTrue(exporter.validate_flash_descriptor(v2_elf, descriptor)["matches_elf"])
            descriptor_bytes = bytearray(descriptor.read_bytes())
            descriptor_bytes[8] = 1
            descriptor.write_bytes(descriptor_bytes)
            with self.assertRaisesRegex(exporter.ExportError, "Flash ROM descriptor mismatch"):
                exporter.validate_flash_descriptor(v2_elf, descriptor)

            descriptor.write_bytes(bytes([0xFF]) * 64)
            with self.assertRaises(exporter.ExportError) as caught:
                exporter.validate_flash_descriptor(v2_elf, descriptor)
            self.assertIn("selected address has no valid RLDS descriptor", str(caught.exception))
            self.assertIn("actual target build ID is unknown", str(caught.exception))
            self.assertNotIn("target contains", str(caught.exception))


if __name__ == "__main__":
    unittest.main(verbosity=2)
