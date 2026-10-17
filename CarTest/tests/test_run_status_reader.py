"""Offline validation for ELF-derived, read-only live-status snapshots."""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import run_status_reader as reader  # noqa: E402
import runtime_log_export as exporter  # noqa: E402


ELF = PROJECT_ROOT / "deliverables" / "CarTest-20261014-0422-after" / "build" / "DebugFollowTurn" / "CarTest.elf"
OLD_ELF = PROJECT_ROOT / "test-data" / "failure3-20261015" / "baseline" / "build" / "DebugFollowTurn" / "CarTest.elf"


def make_idle_header(plan):
    header = bytearray(64)
    header[:4] = b"RLOG"
    struct.pack_into("<5H", header, 4, plan["log_version"], 64, 64, 6144, 50)
    struct.pack_into("<2B", header, 14, exporter.STATE_IDLE, 0)
    struct.pack_into("<7I", header, 16, 0, 0, 0, 0, 0, 0, 0)
    header[44:60] = bytes.fromhex(plan["build_id"])
    struct.pack_into("<I", header, 40, zlib.crc32(header) & 0xFFFFFFFF)
    return bytes(header)


def make_scalar_region(plan, values=None):
    values = values or {}
    scalar_read = plan["scalar_read"]
    data = bytearray(scalar_read["size"])
    defaults = {
        "car_runtime_log_enabled": 1,
        "car_button_start_event_count": 3,
        "car_manual_run_request": 0,
        "car_mode": 0,
        "car_fault": 0,
        "car_main_loop_heartbeat_ms": 250,
        "car_iwdg_reset_seen": 0,
        "car_init_rcc_rsr_reset_snapshot": 0,
    }
    defaults.update(values)
    for name, value in defaults.items():
        symbol = plan["scalar_symbols"].get(name)
        if not symbol or not symbol["available"]:
            continue
        offset = symbol["address_u32"] - scalar_read["address_u32"]
        struct.pack_into("<i" if name in reader.SIGNED_SCALAR_SYMBOLS else "<I",
                         data, offset, int(value))
    return bytes(data)


class RunStatusReaderTests(unittest.TestCase):
    def test_plan_covers_required_optional_and_flash_identity_reads(self):
        before = hashlib.sha256(ELF.read_bytes()).hexdigest()
        plan = reader.build_plan(ELF)
        self.assertEqual(plan["machine"], "ARM")
        self.assertEqual(plan["endianness"], "little")
        self.assertEqual(set(reader.REQUIRED_SCALAR_SYMBOLS), set(plan["required_scalar_symbols"]))
        self.assertEqual(set(reader.HOLD_SNAPSHOT_SYMBOLS), set(plan["optional_hold_snapshot_symbols"]))
        self.assertEqual(len(plan["scalar_symbols"]), 17)
        self.assertEqual(plan["descriptor_read"]["size"], 64)
        self.assertTrue(plan["descriptor_read"]["address"].startswith("0x080"))
        self.assertLessEqual(plan["scalar_read"]["size"], 4096)
        self.assertEqual(plan["header_read"]["size"], 64)
        self.assertFalse(plan["record_area_read"])
        self.assertEqual(hashlib.sha256(ELF.read_bytes()).hexdigest(), before)

    def test_old_matching_elf_without_optional_snapshot_symbols_remains_usable(self):
        plan = reader.build_plan(OLD_ELF)
        self.assertTrue(all(not plan["scalar_symbols"][name]["available"]
                            for name in reader.HOLD_SNAPSHOT_SYMBOLS))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scalars = root / "scalars.bin"
            header = root / "header.bin"
            descriptor = root / "descriptor.bin"
            scalars.write_bytes(make_scalar_region(plan))
            header.write_bytes(make_idle_header(plan))
            descriptor.write_bytes(bytes.fromhex(
                exporter.inspect_elf(OLD_ELF).descriptor_bytes.hex()))
            result = reader.decode_snapshot(OLD_ELF, scalars, header, descriptor)
        self.assertEqual(result["scalars"]["car_vision_hold_snapshot_status"], "unavailable")
        self.assertIsNone(result["scalars"]["car_vision_hold_snapshot_published"])
        self.assertEqual(result["flash_descriptor"]["status"], "matched")

    def test_idle_status_warns_about_separate_non_atomic_reads_and_creates_no_csv(self):
        plan = reader.build_plan(ELF)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scalars = root / "scalars.bin"
            header = root / "header.bin"
            descriptor = root / "descriptor.bin"
            scalars.write_bytes(make_scalar_region(plan))
            header.write_bytes(make_idle_header(plan))
            descriptor.write_bytes(exporter.inspect_elf(ELF).descriptor_bytes)
            result = reader.decode_snapshot(ELF, scalars, header, descriptor)
        self.assertEqual(result["runtime_log_header"]["state_name"], "idle")
        self.assertEqual(result["runtime_log_header"]["count"], 0)
        self.assertTrue(result["runtime_log_header"]["build_id_matches_elf"])
        self.assertEqual(result["runtime_log_header"]["header_crc_status"], "valid")
        self.assertEqual(result["flash_descriptor"]["status"], "matched")
        self.assertFalse(result["csv_created"])
        self.assertFalse(result["reads_are_atomic_together"])
        self.assertTrue(any("separate SWD uploads" in warning for warning in result["warnings"]))

    def test_nine_optional_globals_decode_and_signed_error_stays_signed(self):
        base = reader.build_plan(ELF)
        plan = copy.deepcopy(base)
        start = plan["scalar_read"]["address_u32"]
        for index, name in enumerate(reader.HOLD_SNAPSHOT_SYMBOLS):
            address = start + base["scalar_read"]["size"] + index * 4
            plan["scalar_symbols"][name].update(
                available=True, address="0x{:08X}".format(address),
                address_u32=address, size=4, section=".bss")
        plan["scalar_read"]["size"] += len(reader.HOLD_SNAPSHOT_SYMBOLS) * 4
        values = {
            "car_vision_hold_snapshot_entry": 2,
            "car_vision_hold_snapshot_error_norm": -321,
            "car_vision_hold_snapshot_sequence": 65535,
            "car_vision_hold_snapshot_source_rx_ms": 0xFFFFFFF0,
            "car_vision_hold_snapshot_entered_ms": 0xFFFFFFF5,
            "car_vision_hold_snapshot_target_us": 1900,
            "car_vision_hold_snapshot_applied_us": 1600,
            "car_vision_hold_snapshot_left_permille": 120,
            "car_vision_hold_snapshot_right_permille": 180,
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scalar = root / "scalars.bin"
            header = root / "header.bin"
            descriptor = root / "descriptor.bin"
            scalar.write_bytes(make_scalar_region(plan, values))
            header.write_bytes(make_idle_header(plan))
            descriptor.write_bytes(exporter.inspect_elf(ELF).descriptor_bytes)
            with mock.patch.object(reader, "build_plan", return_value=plan):
                result = reader.decode_snapshot(ELF, scalar, header, descriptor)
        self.assertEqual(result["scalars"]["car_vision_hold_snapshot_status"], "available")
        self.assertTrue(result["scalars"]["car_vision_hold_snapshot_published"])
        self.assertEqual(result["scalars"]["car_vision_hold_snapshot_error_norm"], -321)
        self.assertEqual(result["scalars"]["car_vision_hold_snapshot_sequence"], 65535)

    def test_flash_mismatch_is_rejected_before_attempting_scalar_or_header_reads(self):
        with tempfile.TemporaryDirectory() as directory:
            descriptor = Path(directory) / "wrong-flash.bin"
            data = bytearray(exporter.inspect_elf(ELF).descriptor_bytes)
            data[28] ^= 0x01
            descriptor.write_bytes(data)
            with self.assertRaisesRegex(exporter.ExportError, "Flash ROM descriptor mismatch"):
                reader.decode_snapshot(ELF, Path(directory) / "missing-scalars",
                                       Path(directory) / "missing-header", descriptor)

    def test_required_symbol_absence_and_bad_symbol_size_are_rejected(self):
        original = reader.exporter._parse_elf
        def missing(elf):
            blob, sections, symbols = original(elf)
            symbols = dict(symbols)
            symbols.pop("car_button_start_event_count", None)
            return blob, sections, symbols
        with mock.patch.object(reader.exporter, "_parse_elf", side_effect=missing):
            with self.assertRaisesRegex(reader.StatusReadError, "car_button_start_event_count"):
                reader.build_plan(ELF)

        def wrong_size(elf):
            blob, sections, symbols = original(elf)
            symbols = dict(symbols)
            symbols["car_main_loop_heartbeat_ms"] = dataclasses.replace(
                symbols["car_main_loop_heartbeat_ms"], size=3)
            return blob, sections, symbols
        with mock.patch.object(reader.exporter, "_parse_elf", side_effect=wrong_size):
            with self.assertRaisesRegex(reader.StatusReadError, "must be exactly 4 bytes"):
                reader.build_plan(ELF)

    def test_truncated_scalar_region_is_rejected(self):
        plan = reader.build_plan(ELF)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            truncated = root / "truncated.bin"
            header = root / "header.bin"
            truncated.write_bytes(make_scalar_region(plan)[:-1])
            header.write_bytes(make_idle_header(plan))
            with self.assertRaisesRegex(reader.StatusReadError, "Scalar read length mismatch"):
                reader.decode_snapshot(ELF, truncated, header)


if __name__ == "__main__":
    unittest.main(verbosity=2)
