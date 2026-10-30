"""Validate a raw OpenMV custom-DFU partition upload without touching hardware.

OpenMV's bootloader returns a four-zero short packet after the requested
partition bytes. This script checks that trailer and excludes it from hashes
and image comparisons.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


EOF_SENTINEL = b"\x00\x00\x00\x00"


def parse_size(value: str) -> int:
    try:
        size = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected a byte count (decimal or 0x-prefixed)") from exc
    if size <= 0:
        raise argparse.ArgumentTypeError("partition size must be positive")
    return size


def payload_from_upload(path: Path, partition_size: int) -> bytes:
    raw = path.read_bytes()
    expected_size = partition_size + len(EOF_SENTINEL)
    if len(raw) != expected_size:
        raise ValueError(
            f"{path}: expected {expected_size} bytes (partition {partition_size} + "
            f"4-byte DFU EOF sentinel), got {len(raw)}"
        )
    if raw[-len(EOF_SENTINEL):] != EOF_SENTINEL:
        raise ValueError(f"{path}: final four bytes are not the DFU EOF sentinel 00000000")
    return raw[:partition_size]


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upload", type=Path, required=True,
                        help="raw file written by dfu-util -U")
    parser.add_argument("--partition-size", type=parse_size, required=True,
                        help="full partition size in bytes (decimal or 0x-prefixed)")
    compare = parser.add_mutually_exclusive_group()
    compare.add_argument("--expected-prefix", type=Path,
                          help="firmware/image expected at the start of the partition")
    compare.add_argument("--compare-upload", type=Path,
                         help="earlier raw upload; compare the entire partition payload")
    parser.add_argument("--write-payload", type=Path,
                        help="optionally write the verified payload without its four-byte EOF sentinel")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        payload = payload_from_upload(args.upload, args.partition_size)
        output_path = None
        if args.write_payload is not None:
            output_path = args.write_payload.expanduser().resolve()
            if output_path == args.upload.expanduser().resolve():
                raise ValueError("--write-payload must not overwrite the raw upload")
            if output_path.exists():
                raise ValueError(f"refusing to overwrite existing payload file: {output_path}")
        result = {
            "upload": str(args.upload.resolve()),
            "raw_upload_bytes": args.upload.stat().st_size,
            "partition_payload_bytes": len(payload),
            "dfu_eof_sentinel": "00000000",
            "partition_payload_sha256": sha256(payload),
        }
        if args.write_payload is not None:
            result["payload_written_without_sentinel"] = str(output_path)
        if args.expected_prefix is not None:
            expected = args.expected_prefix.read_bytes()
            if len(expected) > len(payload):
                raise ValueError("expected prefix is larger than the partition payload")
            if payload[:len(expected)] != expected:
                raise ValueError("partition prefix does not match --expected-prefix")
            result["expected_prefix"] = str(args.expected_prefix.resolve())
            result["expected_prefix_bytes"] = len(expected)
            result["expected_prefix_sha256"] = sha256(expected)
            result["expected_prefix_matches"] = True
        elif args.compare_upload is not None:
            baseline = payload_from_upload(args.compare_upload, args.partition_size)
            if payload != baseline:
                raise ValueError("partition payload differs from --compare-upload")
            result["compare_upload"] = str(args.compare_upload.resolve())
            result["full_partition_matches"] = True
        if output_path is not None:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("xb") as payload_file:
                payload_file.write(payload)
            result["payload_written_without_sentinel"] = str(output_path)
        print(json.dumps(result, indent=2, sort_keys=True))
    except (OSError, ValueError) as exc:
        print(f"DFU_READBACK_ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
