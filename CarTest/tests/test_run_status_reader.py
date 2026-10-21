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


def make_header(plan, state, count=0, session_id=0):
    header = bytearray(64)
    header[:4] = b"RLOG"
    struct.pack_into("<5H", header, 4, plan["log_version"], 64, 64,
                     exporter.CAPACITY, exporter.PERIOD_MS)
    struct.pack_into("<2B", header, 14, state, 0)
    start_ms = 1000 if count else 0
    stop_ms = start_ms + count - 1 if state == exporter.STATE_FROZEN and count else 0
    struct.pack_into("<7I", header, 16, count, session_id, start_ms,
                     stop_ms, 0, 0, 0)
    header[44:60] = bytes.fromhex(plan["build_id"])
    struct.pack_into("<I", header, 40, zlib.crc32(header) & 0xFFFFFFFF)
    return bytes(header)


def make_runtime_diagnostic(state, count=0, session_id=0, start_events=0,
                            start_token=0, log_enabled=True, generation=2,
                            log_update_calls=6):
    raw = bytearray(reader.DIAGNOSTIC_SIZE)
    struct.pack_into("<IHHII", raw, 0, reader.DIAGNOSTIC_MAGIC,
                     reader.DIAGNOSTIC_VERSION, reader.DIAGNOSTIC_SIZE,
                     generation, 0)
    struct.pack_into("<8IhhHHII", raw, 16, start_events, 0, log_update_calls, start_token,
                     state, count, 123456, 0x1234, -120, 100, 1530,
                     int(log_enabled), session_id, 0)
    crc_data = bytearray(raw)
    crc_data[12:16] = bytes(4)
    struct.pack_into("<I", raw, 12, zlib.crc32(crc_data) & 0xFFFFFFFF)
    return bytes(raw)


def make_receive_diagnostic(window_start=100, duration=1000,
                            accepted_total=20, accepted_window=10,
                            invalid_total=2, invalid_window=1,
                            sequence=65535, flags=None):
    if flags is None:
        flags = (reader.RECEIVE_DIAGNOSTIC_FLAG_HAVE_SEQUENCE |
                 reader.RECEIVE_DIAGNOSTIC_FLAG_WINDOW_VALID)
    raw = bytearray(reader.RECEIVE_DIAGNOSTIC_SIZE)
    snapshot_ms = (window_start + duration) & 0xFFFFFFFF
    struct.pack_into("<IHHII", raw, 0, reader.RECEIVE_DIAGNOSTIC_MAGIC,
                     reader.RECEIVE_DIAGNOSTIC_VERSION,
                     reader.RECEIVE_DIAGNOSTIC_SIZE, 2, 0)
    struct.pack_into("<12I", raw, 16, snapshot_ms, window_start, duration,
                     accepted_total, accepted_window, invalid_total,
                     invalid_window, 3, 1, 2, 0, 4)
    struct.pack_into("<2H", raw, 64, sequence, flags)
    struct.pack_into("<I", raw, 12, zlib.crc32(raw) & 0xFFFFFFFF)
    return bytes(raw)


def synthetic_diagnostic_plan(elf):
    plan = copy.deepcopy(reader.build_plan(elf))
    plan["runtime_diagnostic_snapshot"] = {
        "available": True,
        "symbol": reader.DIAGNOSTIC_SYMBOL,
        "address": "0x2407F000",
        "address_u32": 0x2407F000,
        "size": reader.DIAGNOSTIC_SIZE,
        "section": ".runtime_diag",
        "section_alignment": 32,
    }
    return plan


def synthetic_speed_pi_diagnostic_plan(elf):
    plan = synthetic_diagnostic_plan(elf)
    plan["speed_pi_diagnostic_snapshot"] = {
        "available": True,
        "symbol": reader.SPEED_PI_DIAGNOSTIC_SYMBOL,
        "address": "0x2407E000",
        "address_u32": 0x2407E000,
        "size": reader.SPEED_PI_DIAGNOSTIC_SIZE,
        "section": ".speed_pi_diag",
        "section_alignment": 32,
    }
    return plan


def make_speed_pi_diagnostic(start_token=7, flags=None, generation=2,
                             pi_type=2, saturation_left_ms=50,
                             saturation_right_ms=20,
                             version=reader.SPEED_PI_DIAGNOSTIC_VERSION_V1):
    if flags is None:
        flags = (reader.SPEED_PI_DIAGNOSTIC_FLAG_ENABLED |
                 reader.SPEED_PI_DIAGNOSTIC_FLAG_ARMED |
                 reader.SPEED_PI_DIAGNOSTIC_FLAG_LAST_SAMPLE_VALID |
                 reader.SPEED_PI_DIAGNOSTIC_FLAG_FROZEN)
    raw = bytearray(reader.SPEED_PI_DIAGNOSTIC_SIZE)
    struct.pack_into("<IHHII", raw, 0, reader.SPEED_PI_DIAGNOSTIC_MAGIC,
                     version,
                     reader.SPEED_PI_DIAGNOSTIC_SIZE, generation, 0)
    struct.pack_into("<4I", raw, 16, start_token, 1000, 1100, 1090)
    struct.pack_into("<4i", raw, 32, 2400, 2300, 2200, 2100)
    struct.pack_into("<6H", raw, 48, 500, 490, 700, 690, 680, 670)
    struct.pack_into("<6I", raw, 60, 3, 0, saturation_left_ms,
                     saturation_right_ms, flags, pi_type)
    reserved = [0] * 11
    if version == reader.SPEED_PI_DIAGNOSTIC_VERSION_V2:
        reserved[0] = reader.SPEED_PI_DIAGNOSTIC_V2_OUTPUT_CAP_PERMILLE
        struct.pack_into("<6H", raw, 48, 500, 490, 1000, 990, 980, 970)
    struct.pack_into("<11I", raw, 84, *reserved)
    crc_data = bytearray(raw)
    crc_data[12:16] = bytes(4)
    struct.pack_into("<I", raw, 12, zlib.crc32(crc_data) & 0xFFFFFFFF)
    return bytes(raw)


class RunStatusReaderTests(unittest.TestCase):
    def test_plan_covers_required_optional_and_flash_identity_reads(self):
        before = hashlib.sha256(ELF.read_bytes()).hexdigest()
        plan = reader.build_plan(ELF)
        self.assertEqual(plan["machine"], "ARM")
        self.assertEqual(plan["endianness"], "little")
        self.assertEqual(set(reader.REQUIRED_SCALAR_SYMBOLS), set(plan["required_scalar_symbols"]))
        self.assertEqual(set(reader.HOLD_SNAPSHOT_SYMBOLS), set(plan["optional_hold_snapshot_symbols"]))
        self.assertEqual(len(plan["scalar_symbols"]), 20)
        self.assertEqual(plan["optional_receive_rate_symbols"],
                         list(reader.RECEIVE_RATE_SYMBOLS))
        self.assertEqual(plan["descriptor_read"]["size"], 64)
        self.assertTrue(plan["descriptor_read"]["address"].startswith("0x080"))
        self.assertLessEqual(plan["scalar_read"]["size"],
                             reader.MAX_SCALAR_READ_SIZE)
        self.assertEqual(plan["scalar_read"]["maximum_size"],
                         reader.MAX_SCALAR_READ_SIZE)
        self.assertEqual(plan["header_read"]["size"], 64)
        self.assertFalse(plan["record_area_read"])
        self.assertFalse(plan["runtime_diagnostic_snapshot"]["available"])
        self.assertFalse(plan["speed_pi_diagnostic_snapshot"]["available"])
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
        self.assertEqual(result["status_class"], "legacy_idle")

    def _decode_pi_case(self, header, runtime_diag, pi_diag):
        plan = synthetic_speed_pi_diagnostic_plan(ELF)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scalar = root / "scalars.bin"
            before = root / "header-before.bin"
            after = root / "header-after.bin"
            runtime_path = root / "runtime-diagnostic.bin"
            pi_path = root / "speed-pi-diagnostic.bin"
            descriptor = root / "descriptor.bin"
            scalar.write_bytes(make_scalar_region(plan))
            before.write_bytes(header)
            after.write_bytes(header)
            runtime_path.write_bytes(runtime_diag)
            pi_path.write_bytes(pi_diag)
            descriptor.write_bytes(exporter.inspect_elf(ELF).descriptor_bytes)
            with mock.patch.object(reader, "build_plan", return_value=plan):
                return reader.decode_snapshot(
                    ELF, scalar, before, descriptor,
                    header_after_path=after,
                    diagnostic_path=runtime_path,
                    speed_pi_diagnostic_path=pi_path)

    def test_speed_pi_diagnostic_validates_and_decodes_frozen_snapshot(self):
        raw = make_speed_pi_diagnostic()
        decoded = reader.decode_speed_pi_diagnostic(raw)
        self.assertEqual(decoded["pi_type_name"], "incremental")
        self.assertTrue(decoded["frozen"])
        self.assertEqual(decoded["start_token"], 7)
        self.assertEqual(decoded["saturation_left_ms"], 50)
        self.assertIn("distinct valid sample_ms", decoded["saturation_semantics"])

        odd = bytearray(raw)
        struct.pack_into("<I", odd, 8, 3)
        with self.assertRaises(reader.StatusReadError):
            reader.decode_speed_pi_diagnostic(bytes(odd))

        torn = bytearray(raw)
        torn[40] ^= 1
        with self.assertRaises(reader.StatusReadError):
            reader.decode_speed_pi_diagnostic(bytes(torn))

        reserved = bytearray(raw)
        struct.pack_into("<I", reserved, 84, 1)
        reserved[12:16] = bytes(4)
        struct.pack_into("<I", reserved, 12, zlib.crc32(reserved) & 0xFFFFFFFF)
        with self.assertRaises(reader.StatusReadError):
            reader.decode_speed_pi_diagnostic(bytes(reserved))

    def test_speed_pi_v2_declares_1000_output_cap_while_v1_keeps_700(self):
        v1 = reader.decode_speed_pi_diagnostic(make_speed_pi_diagnostic())
        v2 = reader.decode_speed_pi_diagnostic(make_speed_pi_diagnostic(
            version=reader.SPEED_PI_DIAGNOSTIC_VERSION_V2))
        self.assertEqual(v1["output_cap_permille"], 700)
        self.assertEqual(v2["output_cap_permille"], 1000)
        self.assertEqual(v2["requested_left_permille"], 1000)

        invalid = bytearray(make_speed_pi_diagnostic(
            version=reader.SPEED_PI_DIAGNOSTIC_VERSION_V2))
        struct.pack_into("<I", invalid, 84, 700)
        invalid[12:16] = bytes(4)
        struct.pack_into("<I", invalid, 12, zlib.crc32(invalid) & 0xFFFFFFFF)
        with self.assertRaisesRegex(reader.StatusReadError, "cap/reserved words"):
            reader.decode_speed_pi_diagnostic(bytes(invalid))

    def test_receive_snapshot_reports_wrap_safe_actual_rate_and_invalid_ratio(self):
        decoded = reader.decode_receive_diagnostic(make_receive_diagnostic(
            window_start=0xFFFFFFF0, duration=2010,
            accepted_total=500, accepted_window=201,
            invalid_total=50, invalid_window=10, sequence=7))
        self.assertEqual(decoded["window_duration_ms"], 2010)
        self.assertEqual(decoded["accepted_millifps"], 100000)
        self.assertEqual(decoded["invalid_observation_permille"], 49)
        self.assertEqual(decoded["latest_sequence"], 7)

        torn = bytearray(make_receive_diagnostic())
        torn[72] ^= 0x01
        with self.assertRaisesRegex(reader.StatusReadError, "CRC32 is invalid"):
            reader.decode_receive_diagnostic(bytes(torn))

    def test_direct_receive_rate_scalars_are_optional_and_decoded_as_millifps(self):
        base = reader.build_plan(ELF)
        plan = copy.deepcopy(base)
        start = plan["scalar_read"]["address_u32"]
        for index, name in enumerate(reader.RECEIVE_RATE_SYMBOLS):
            address = start + base["scalar_read"]["size"] + index * 4
            plan["scalar_symbols"][name].update(
                available=True, address="0x{:08X}".format(address),
                address_u32=address, size=4, section=".bss")
        plan["scalar_read"]["size"] += len(reader.RECEIVE_RATE_SYMBOLS) * 4
        values = {
            "car_rx_frame_rate_millifps": 27500,
            "car_rx_invalid_observation_permille": 20,
            "car_rx_frame_rate_valid": 1,
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
        self.assertEqual(result["scalars"]["car_rx_frame_rate_millifps"], 27500)
        self.assertEqual(result["scalars"]["car_rx_frame_rate_fps"], 27.5)
        self.assertTrue(result["scalars"]["car_rx_frame_rate_valid"])
        self.assertIn("not camera sensor", result["scalars"]["car_rx_rate_semantics"])

    def test_speed_pi_manual_token_must_match_runtime_diagnostic(self):
        plan = reader.build_plan(ELF)
        header = make_header(plan, exporter.STATE_FROZEN, count=18, session_id=55)
        runtime = make_runtime_diagnostic(
            exporter.STATE_FROZEN, count=18, session_id=55,
            start_events=3, start_token=7)
        matching = self._decode_pi_case(header, runtime, make_speed_pi_diagnostic())
        self.assertEqual(matching["status_class"], "frozen")
        self.assertTrue(matching["speed_pi_diagnostic_snapshot"][
            "start_token_matches_runtime_diagnostic"])

        mismatched = self._decode_pi_case(
            header, runtime, make_speed_pi_diagnostic(start_token=8))
        self.assertEqual(mismatched["status_class"], "inconsistent")
        self.assertTrue(any("start token does not match" in warning
                            for warning in mismatched["warnings"]))

    def _decode_diagnostic_case(self, header_before, diagnostic, header_after=None):
        plan = synthetic_diagnostic_plan(ELF)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scalar = root / "scalars.bin"
            before = root / "header-before.bin"
            after = root / "header-after.bin"
            diag = root / "runtime-diagnostic.bin"
            descriptor = root / "descriptor.bin"
            scalar.write_bytes(make_scalar_region(plan))
            before.write_bytes(header_before)
            after.write_bytes(header_after if header_after is not None else header_before)
            diag.write_bytes(diagnostic)
            descriptor.write_bytes(exporter.inspect_elf(ELF).descriptor_bytes)
            with mock.patch.object(reader, "build_plan", return_value=plan):
                return reader.decode_snapshot(
                    ELF, scalar, before, descriptor,
                    header_after_path=after, diagnostic_path=diag)

    def test_runtime_diagnostic_decodes_neverstarted_and_reports_unconsumed_start(self):
        plan = reader.build_plan(ELF)
        header = make_header(plan, exporter.STATE_IDLE)
        result = self._decode_diagnostic_case(
            header, make_runtime_diagnostic(exporter.STATE_IDLE,
                                            start_events=2, start_token=0))
        self.assertEqual(result["status_class"], "neverstarted")
        self.assertFalse(result["csv_created"])
        self.assertEqual(result["runtime_diagnostic_snapshot"]["start_events"], 2)
        self.assertEqual(result["runtime_diagnostic_snapshot"]["start_token"], 0)
        self.assertTrue(any("not consumed" in warning for warning in result["warnings"]))

    def test_active_count_progress_across_header_bracket_is_not_corruption(self):
        plan = reader.build_plan(ELF)
        before = make_header(plan, exporter.STATE_ACTIVE, count=20, session_id=55)
        after = make_header(plan, exporter.STATE_ACTIVE, count=3000, session_id=55)
        result = self._decode_diagnostic_case(
            before, make_runtime_diagnostic(exporter.STATE_ACTIVE, count=1500,
                                            session_id=55, start_events=3,
                                            start_token=7), after)
        self.assertEqual(result["status_class"], "active")
        self.assertTrue(result["observing_active_progress"])
        self.assertFalse(result["csv_created"])

        outside_bracket = self._decode_diagnostic_case(
            before, make_runtime_diagnostic(exporter.STATE_ACTIVE, count=3001,
                                            session_id=55, start_events=3,
                                            start_token=7), after)
        self.assertEqual(outside_bracket["status_class"], "inconsistent")

        before_future = self._decode_diagnostic_case(
            before, make_runtime_diagnostic(exporter.STATE_ACTIVE, count=19,
                                            session_id=55, start_events=3,
                                            start_token=7), after)
        self.assertEqual(before_future["status_class"], "inconsistent")

    def test_active_header_with_invalid_crc_is_retryable_inconsistent(self):
        plan = reader.build_plan(ELF)
        before = make_header(plan, exporter.STATE_ACTIVE, count=10, session_id=55)
        after = bytearray(make_header(plan, exporter.STATE_ACTIVE, count=12, session_id=55))
        after[40] ^= 1
        result = self._decode_diagnostic_case(
            before, make_runtime_diagnostic(exporter.STATE_ACTIVE, count=11,
                                            session_id=55, start_events=3,
                                            start_token=7), bytes(after))
        self.assertEqual(result["status_class"], "inconsistent")
        self.assertTrue(any("ACTIVE runtime-log header CRC is invalid" in warning
                            for warning in result["warnings"]))

    def test_stable_frozen_snapshot_requires_exact_header_count_and_session(self):
        plan = reader.build_plan(ELF)
        header = make_header(plan, exporter.STATE_FROZEN, count=18, session_id=55)
        result = self._decode_diagnostic_case(
            header, make_runtime_diagnostic(exporter.STATE_FROZEN, count=18,
                                            session_id=55, start_events=3,
                                            start_token=7, log_update_calls=1))
        self.assertEqual(result["status_class"], "frozen")
        self.assertFalse(result["csv_created"])

        count_mismatch = self._decode_diagnostic_case(
            header, make_runtime_diagnostic(exporter.STATE_FROZEN, count=17,
                                            session_id=55, start_events=3,
                                            start_token=7))
        self.assertEqual(count_mismatch["status_class"], "inconsistent")

        session_mismatch = self._decode_diagnostic_case(
            header, make_runtime_diagnostic(exporter.STATE_FROZEN, count=18,
                                            session_id=56, start_events=3,
                                            start_token=7))
        self.assertEqual(session_mismatch["status_class"], "inconsistent")

    def test_idle_to_frozen_transition_is_retried_as_inconsistent(self):
        plan = reader.build_plan(ELF)
        idle = make_header(plan, exporter.STATE_IDLE)
        frozen = make_header(plan, exporter.STATE_FROZEN, count=18, session_id=55)
        result = self._decode_diagnostic_case(
            idle, make_runtime_diagnostic(exporter.STATE_FROZEN, count=18,
                                          session_id=55, start_events=2,
                                          start_token=7), frozen)
        self.assertEqual(result["status_class"], "inconsistent")
        self.assertTrue(any("transition crossed" in warning for warning in result["warnings"]))

    def test_corrupt_or_torn_runtime_diagnostic_is_rejected(self):
        plan = reader.build_plan(ELF)
        header = make_header(plan, exporter.STATE_IDLE)
        corrupt = bytearray(make_runtime_diagnostic(exporter.STATE_IDLE))
        corrupt[40] ^= 0x80
        with self.assertRaisesRegex(reader.StatusReadError, "snapshot CRC32 is invalid"):
            self._decode_diagnostic_case(header, bytes(corrupt))

        odd_generation = make_runtime_diagnostic(exporter.STATE_IDLE, generation=3)
        with self.assertRaisesRegex(reader.StatusReadError, "generation is zero or odd"):
            self._decode_diagnostic_case(header, odd_generation)

    def test_idle_cpu_snapshot_and_active_header_are_inconsistent(self):
        plan = reader.build_plan(ELF)
        before = make_header(plan, exporter.STATE_ACTIVE, count=10, session_id=55)
        after = make_header(plan, exporter.STATE_ACTIVE, count=12, session_id=55)
        result = self._decode_diagnostic_case(
            before, make_runtime_diagnostic(exporter.STATE_IDLE), after)
        self.assertEqual(result["status_class"], "inconsistent")
        self.assertFalse(result["csv_created"])

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

    def test_scalar_span_above_bounded_read_limit_is_rejected(self):
        original = reader.exporter._parse_elf

        def too_wide(elf):
            blob, sections, symbols = original(elf)
            symbols = dict(symbols)
            far_symbol = symbols["car_main_loop_heartbeat_ms"]
            base = min(symbols[name].value for name in reader.SCALAR_SYMBOLS
                       if name in symbols)
            target = base + reader.MAX_SCALAR_READ_SIZE
            symbols[far_symbol.name] = dataclasses.replace(
                far_symbol, value=target)

            section_index = far_symbol.section_index
            section = sections[section_index]
            section_end = target + far_symbol.size
            sections = list(sections)
            sections[section_index] = dataclasses.replace(
                section,
                size=max(section.address + section.size, section_end) - section.address)
            return blob, sections, symbols

        with mock.patch.object(reader.exporter, "_parse_elf", side_effect=too_wide):
            with self.assertRaisesRegex(reader.StatusReadError,
                                        "maximum permitted read is 8192 bytes"):
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
