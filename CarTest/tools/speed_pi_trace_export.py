#!/usr/bin/env python3
"""Validate and export one frozen Speed PI trace section from STM32H743 RAM."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import struct
import sys
import zlib
from pathlib import Path
from typing import Any, Sequence

import run_status_reader as status_reader
import runtime_log_export as runtime_export


TRACE_RECORD_FORMAT = "<IIIHI BBiiiiHHHHI"
TRACE_CRC_OFFSET = 44
TRACE_COLUMNS = (
    "sample_index", "record_ms", "sample_ms", "vision_sequence",
    "vision_arrival_ms", "pi_type", "flags", "speed_valid", "armed",
    "hold_active", "proposal_valid", "target_left_millirps",
    "target_right_millirps", "measured_left_millirps",
    "measured_right_millirps", "requested_left_permille",
    "requested_right_permille", "applied_left_permille",
    "applied_right_permille",
)


class TraceExportError(ValueError):
    """A trace snapshot is incomplete, inconsistent, or corrupt."""


def _fail(message: str) -> None:
    raise TraceExportError(message)


def _crc32(raw: bytes) -> int:
    return zlib.crc32(raw) & 0xFFFFFFFF


def decode_trace(raw: bytes, expected_build_id: str,
                 pi_diagnostic: dict[str, Any] | None = None,
                 runtime_diagnostic: dict[str, Any] | None = None,
                 receive_diagnostic: dict[str, Any] | None = None) -> dict[str, Any]:
    """Validate header, chronological ring, sample CRCs and optional session links."""
    expected_size = status_reader.SPEED_PI_TRACE_SIZE
    if len(raw) != expected_size:
        _fail("Speed PI trace upload must be exactly {} bytes (found {})".format(
            expected_size, len(raw)))
    try:
        header = status_reader.decode_speed_pi_trace_header(
            raw[:status_reader.SPEED_PI_TRACE_HEADER_SIZE], expected_build_id)
    except status_reader.StatusReadError as exc:
        raise TraceExportError(str(exc)) from exc
    if header["state"] != 2:
        _fail("Speed PI trace is {}; only FROZEN traces can be exported".format(
            header["state_name"].upper()))
    if header["count"] == 0:
        _fail("FROZEN speed PI trace has no records; no CSV export was created")
    if pi_diagnostic is not None:
        if (not pi_diagnostic["frozen"] or
                pi_diagnostic["start_token"] != header["start_token"] or
                pi_diagnostic["start_ms"] != header["start_ms"] or
                pi_diagnostic["pi_type"] != header["pi_type"]):
            _fail("Speed PI trace does not match the frozen PI diagnostic session")
    if runtime_diagnostic is not None and (
            runtime_diagnostic["start_token"] != header["start_token"]):
        _fail("Speed PI trace start token does not match the runtime diagnostic")

    capacity = int(header["capacity"])
    count = int(header["count"])
    next_index = int(header["next_index"])
    first_slot = next_index if count == capacity else 0
    first_sample_index = int(header["total_samples"]) - count
    ordered: list[dict[str, int]] = []
    ordered_raw: list[bytes] = []
    for ordinal in range(count):
        slot = (first_slot + ordinal) % capacity
        offset = status_reader.SPEED_PI_TRACE_HEADER_SIZE + slot * status_reader.SPEED_PI_TRACE_SAMPLE_SIZE
        record = raw[offset:offset + status_reader.SPEED_PI_TRACE_SAMPLE_SIZE]
        if len(record) != status_reader.SPEED_PI_TRACE_SAMPLE_SIZE:
            _fail("Speed PI trace contains a truncated ring record")
        values = struct.unpack(TRACE_RECORD_FORMAT, record)
        (sample_index, record_ms, sample_ms, vision_sequence, vision_arrival_ms,
         pi_type, flags, target_left, target_right, measured_left, measured_right,
         requested_left, requested_right, applied_left, applied_right,
         observed_crc) = values
        crc_data = bytearray(record)
        crc_data[TRACE_CRC_OFFSET:TRACE_CRC_OFFSET + 4] = bytes(4)
        computed_crc = _crc32(bytes(crc_data))
        if computed_crc != observed_crc:
            _fail("Speed PI trace record {} CRC32 is invalid".format(ordinal))
        if sample_index != first_sample_index + ordinal:
            _fail("Speed PI trace sample indices are not chronological and contiguous")
        if pi_type != header["pi_type"]:
            _fail("Speed PI trace record PI type differs from the session header")
        if flags & ~status_reader.SPEED_PI_TRACE_KNOWN_FLAGS:
            _fail("Speed PI trace record contains unsupported flag bits")
        if any(value < 0 or value > 100_000_000 for value in
               (target_left, target_right, measured_left, measured_right)):
            _fail("Speed PI trace record contains a speed outside the supported range")
        if any(value > 1000 for value in
               (requested_left, requested_right, applied_left, applied_right)):
            _fail("Speed PI trace record contains an output above the 1000-permille cap")
        if ordinal:
            previous_ms = ordered[-1]["record_ms"]
            delta = (record_ms - previous_ms) & 0xFFFFFFFF
            if delta >= 0x80000000:
                _fail("Speed PI trace record timestamps move backwards")
        row = {
            "sample_index": sample_index, "record_ms": record_ms,
            "sample_ms": sample_ms, "vision_sequence": vision_sequence,
            "vision_arrival_ms": vision_arrival_ms, "pi_type": pi_type,
            "flags": flags,
            "speed_valid": int(bool(flags & status_reader.SPEED_PI_TRACE_FLAG_SPEED_VALID)),
            "armed": int(bool(flags & status_reader.SPEED_PI_TRACE_FLAG_ARMED)),
            "hold_active": int(bool(flags & status_reader.SPEED_PI_TRACE_FLAG_HOLD)),
            "proposal_valid": int(bool(flags & status_reader.SPEED_PI_TRACE_FLAG_PROPOSAL_VALID)),
            "target_left_millirps": target_left,
            "target_right_millirps": target_right,
            "measured_left_millirps": measured_left,
            "measured_right_millirps": measured_right,
            "requested_left_permille": requested_left,
            "requested_right_permille": requested_right,
            "applied_left_permille": applied_left,
            "applied_right_permille": applied_right,
        }
        ordered.append(row)
        ordered_raw.append(record)

    computed_records_crc = _crc32(b"".join(ordered_raw))
    observed_records_crc = int(header["records_crc32"], 16)
    if computed_records_crc != observed_records_crc:
        _fail("Speed PI trace chronological ring CRC32 is invalid")

    receive_link = None
    if receive_diagnostic is not None:
        have_sequence = bool(receive_diagnostic["have_sequence"])
        latest_sequence = receive_diagnostic["latest_sequence"]
        found = any(row["vision_sequence"] == latest_sequence for row in ordered)
        receive_link = {
            "status": ("sequence_matched" if found else
                       "latest_sequence_not_in_trace" if have_sequence else
                       "no_latest_sequence"),
            "latest_sequence": latest_sequence if have_sequence else None,
            "latest_sequence_found_in_trace": found if have_sequence else None,
            "note": "The receive snapshot has no PI session token; sequence match is observational, not a session identity proof.",
        }
    return {
        "schema": "PTRC-v1",
        "build_id": header["build_id"],
        "state": header["state_name"],
        "session": {
            "start_token": header["start_token"], "start_ms": header["start_ms"],
            "stop_ms": header["stop_ms"], "pi_type": header["pi_type_name"],
        },
        "sample_count": len(ordered),
        "total_samples": header["total_samples"],
        "lost_samples": header["lost_samples"],
        "ring_wrapped": header["total_samples"] > capacity,
        "header_crc32": header["header_crc32_read"],
        "records_crc32": "0x{:08X}".format(computed_records_crc),
        "records_crc_valid": True,
        "pi_diagnostic_matched": pi_diagnostic is not None,
        "runtime_diagnostic_matched": runtime_diagnostic is not None,
        "receive_diagnostic_sequence_link": receive_link,
        "records": ordered,
    }


def export_trace(elf_path: str | Path, raw_path: str | Path,
                 csv_path: str | Path, metadata_path: str | Path,
                 pi_path: str | Path | None = None,
                 runtime_path: str | Path | None = None,
                 receive_path: str | Path | None = None) -> dict[str, Any]:
    layout = runtime_export.inspect_elf(elf_path)
    pi = (status_reader.decode_speed_pi_diagnostic(Path(pi_path).read_bytes())
          if pi_path is not None else None)
    runtime = (status_reader.decode_runtime_diagnostic(Path(runtime_path).read_bytes())
               if runtime_path is not None else None)
    receive = (status_reader.decode_receive_diagnostic(Path(receive_path).read_bytes())
               if receive_path is not None else None)
    raw = Path(raw_path).read_bytes()
    result = decode_trace(raw, layout.build_id.hex(), pi, runtime, receive)
    result["elf"] = {
        "path": str(Path(elf_path).resolve()), "sha256": layout.sha256,
        "build_id": layout.build_id.hex(),
    }
    result["raw"] = {
        "path": str(Path(raw_path).resolve()), "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }
    csv_destination = Path(csv_path)
    metadata_destination = Path(metadata_path)
    csv_destination.parent.mkdir(parents=True, exist_ok=True)
    metadata_destination.parent.mkdir(parents=True, exist_ok=True)
    with csv_destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRACE_COLUMNS)
        writer.writeheader()
        writer.writerows(result["records"])
    result["csv"] = {
        "path": str(csv_destination.resolve()),
        "sha256": hashlib.sha256(csv_destination.read_bytes()).hexdigest(),
    }
    result["records"] = None
    metadata_destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                                    encoding="utf-8")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--raw", required=True)
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--pi-diagnostic")
    parser.add_argument("--runtime-diagnostic")
    parser.add_argument("--receive-diagnostic")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = export_trace(args.elf, args.raw, args.out_csv, args.metadata,
                              args.pi_diagnostic, args.runtime_diagnostic,
                              args.receive_diagnostic)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (TraceExportError, status_reader.StatusReadError,
            runtime_export.ExportError, OSError, struct.error) as exc:
        for path in (args.out_csv, args.metadata):
            try:
                Path(path).unlink(missing_ok=True)
            except OSError:
                pass
        print("speed-pi-trace-export: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
