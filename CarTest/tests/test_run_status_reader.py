"""Offline validation for the bounded live-status reader."""

from __future__ import annotations

import dataclasses
import hashlib
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import run_status_reader as reader  # noqa: E402


ELF = PROJECT_ROOT / "build" / "DebugFollowTurn" / "CarTest.elf"
FIXTURES = PROJECT_ROOT / "test-data" / "run-status-20261014" / "fixtures"
HEADER = FIXTURES / "current-run-header-idle.bin"
SCALARS = FIXTURES / "synthetic-scalar-region.bin"


class RunStatusReaderTests(unittest.TestCase):
    def test_actual_elf_plans_only_eight_bounded_scalars_and_header(self):
        before = hashlib.sha256(ELF.read_bytes()).hexdigest()
        plan = reader.build_plan(ELF)
        self.assertEqual(plan["machine"], "ARM")
        self.assertEqual(plan["endianness"], "little")
        self.assertEqual(len(plan["scalar_symbols"]), 8)
        self.assertEqual(plan["scalar_read"]["address"], "0x2400004C")
        self.assertEqual(plan["scalar_read"]["size"], 600)
        self.assertLessEqual(plan["scalar_read"]["size"], 4096)
        self.assertEqual(plan["header_read"]["address"], "0x24001040")
        self.assertEqual(plan["header_read"]["size"], 64)
        self.assertFalse(plan["record_area_read"])
        self.assertTrue(all(symbol["size"] == 4 for symbol in plan["scalar_symbols"].values()))
        self.assertEqual(hashlib.sha256(ELF.read_bytes()).hexdigest(), before)

    def test_current_matching_header_decodes_idle_with_valid_crc(self):
        result = reader.decode_snapshot(ELF, SCALARS, HEADER)
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["snapshot_kind"], "live_diagnostic_only")
        self.assertFalse(result["diagnostic_is_run_record"])
        self.assertFalse(result["reads_are_atomic_together"])
        header = result["runtime_log_header"]
        self.assertEqual(header["state"], 0)
        self.assertEqual(header["state_name"], "idle")
        self.assertEqual(header["count"], 0)
        self.assertTrue(header["build_id_matches_elf"])
        self.assertEqual(header["header_crc_status"], "valid")
        self.assertFalse(header["record_payload_read"])
        scalars = result["scalars"]
        self.assertEqual(scalars["car_runtime_log_enabled"], 1)
        self.assertEqual(scalars["car_mode_name"], "boot_safe")
        self.assertFalse(scalars["iwdg_reset_flag_in_rcc_rsr"])
        self.assertIn("D-cache", result["consistency_note"])

    def test_missing_scalar_symbol_is_a_clear_plan_error(self):
        original = reader.exporter._require_symbol

        def missing(symbols, name):
            if name == "car_button_start_event_count":
                reader._fail("ELF is missing required symbol '{}'".format(name))
            return original(symbols, name)

        with mock.patch.object(reader.exporter, "_require_symbol", side_effect=missing):
            with self.assertRaisesRegex(reader.StatusReadError, "car_button_start_event_count"):
                reader.build_plan(ELF)

    def test_non_four_byte_scalar_symbol_is_rejected(self):
        original = reader.exporter._require_symbol

        def wrong_size(symbols, name):
            symbol = original(symbols, name)
            if name == "car_main_loop_heartbeat_ms":
                return dataclasses.replace(symbol, size=3)
            return symbol

        with mock.patch.object(reader.exporter, "_require_symbol", side_effect=wrong_size):
            with self.assertRaisesRegex(reader.StatusReadError, "must be exactly 4 bytes"):
                reader.build_plan(ELF)

    def test_active_header_crc_mismatch_is_marked_uncertain_not_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            active_path = Path(directory) / "active-header.bin"
            active = bytearray(HEADER.read_bytes())
            active[14] = 1  # ACTIVE transition without rewriting the rest/CRC.
            active_path.write_bytes(active)
            result = reader.decode_snapshot(ELF, SCALARS, active_path)
        header = result["runtime_log_header"]
        self.assertEqual(result["status"], "read_with_warnings")
        self.assertEqual(header["state_name"], "active")
        self.assertEqual(header["header_crc_status"], "uncertain_live_mutation")

    def test_truncated_scalar_region_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            truncated = Path(directory) / "truncated.bin"
            truncated.write_bytes(SCALARS.read_bytes()[:-1])
            with self.assertRaisesRegex(reader.StatusReadError, "Scalar read length mismatch"):
                reader.decode_snapshot(ELF, truncated, HEADER)


if __name__ == "__main__":
    unittest.main(verbosity=2)
