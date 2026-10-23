#!/usr/bin/env python3
"""Replace only the embedded main.py in the 30 s coordinated RAM pipeline.

The source remains an experimental RAM-only candidate. Every other byte of
the existing coordinated pipeline script is preserved.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOLS_ROOT = PROJECT_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from build_openmv_ackermann320_packages import (  # noqa: E402
    OPENMV_DIR,
    PACKAGES,
    package_source_text,
)
from build_openmv_native_perfwindow_probe import (  # noqa: E402
    decorate_perf_methods,
)


BASELINE_RAM_SOURCE = (PROJECT_ROOT / "logs" / "camera-bench-20261023" /
                       "package-coordinated-ram-source.py")
OUTPUT_ROOT = (PROJECT_ROOT / "logs" / "usb-preview-20261024" /
               "native-perfwindow")
DEFAULT_OUTPUT = OUTPUT_ROOT / \
    "native-perfwindow-full-pipeline-30s-ram-source.py"
DEFAULT_MANIFEST = OUTPUT_ROOT / \
    "native-perfwindow-full-pipeline-30s-manifest.json"
MAIN_ASSIGNMENT_PREFIX = b"_ram_sources['main.py']="


def sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _coordinated_main_source() -> bytes:
    matches = [package for package in PACKAGES
               if package["name"] == "coordinated"]
    if len(matches) != 1:
        raise RuntimeError("coordinated profile is unavailable")
    source = package_source_text(OPENMV_DIR / "main.py", matches[0])
    return source.encode("utf-8")


def _main_assignment_line(lines: list[bytes]) -> int:
    found = [index for index, line in enumerate(lines)
             if MAIN_ASSIGNMENT_PREFIX in line]
    if len(found) != 1:
        raise RuntimeError("expected exactly one embedded main.py assignment")
    return found[0]


def _decode_main_assignment(line: bytes) -> bytes:
    content = line.rstrip(b"\r\n")
    _, separator, literal = content.partition(b"=")
    if not separator or not literal.startswith(b"b"):
        raise RuntimeError("embedded main.py is not a bytes literal")
    compressed = ast.literal_eval(literal.decode("ascii"))
    if not isinstance(compressed, bytes):
        raise RuntimeError("embedded main.py literal did not decode to bytes")
    return zlib.decompress(compressed)


def build(baseline_path: Path = BASELINE_RAM_SOURCE,
          output_path: Path = DEFAULT_OUTPUT,
          manifest_path: Path = DEFAULT_MANIFEST) -> dict:
    baseline_path = Path(baseline_path)
    if not baseline_path.is_file():
        raise FileNotFoundError("coordinated RAM baseline not found: %s" %
                                baseline_path)
    baseline_payload = baseline_path.read_bytes()
    baseline_lines = baseline_payload.splitlines(keepends=True)
    assignment_index = _main_assignment_line(baseline_lines)
    baseline_main = _decode_main_assignment(baseline_lines[assignment_index])

    expected_base_main = _coordinated_main_source()
    if baseline_main != expected_base_main:
        raise RuntimeError(
            "embedded baseline main.py differs from current coordinated source")

    candidate_main = decorate_perf_methods(baseline_main.decode("utf-8"))
    candidate_main_bytes = candidate_main.encode("utf-8")
    old_line = baseline_lines[assignment_index]
    old_content = old_line.rstrip(b"\r\n")
    ending = old_line[len(old_content):]
    left, separator, _ = old_content.partition(b"=")
    if not separator:
        raise RuntimeError("embedded main.py assignment lacks '='")
    new_line = left + b"=" + repr(zlib.compress(candidate_main_bytes)).encode(
        "ascii") + ending
    candidate_lines = list(baseline_lines)
    candidate_lines[assignment_index] = new_line
    candidate_payload = b"".join(candidate_lines)

    if _decode_main_assignment(candidate_lines[assignment_index]) != \
            candidate_main_bytes:
        raise RuntimeError("replacement main.py failed decompression check")
    changed_lines = [index for index, (old, new) in enumerate(
        zip(baseline_lines, candidate_lines)) if old != new]
    if changed_lines != [assignment_index]:
        raise RuntimeError("candidate changed lines outside embedded main.py")
    compile(candidate_payload.decode("utf-8"),
            "native_perfwindow_full_pipeline_30s.py", "exec")

    output_path = Path(output_path)
    manifest_path = Path(manifest_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(candidate_payload)
    manifest = {
        "experiment": "native_v7_perfwindow_full_pipeline_30s",
        "production_modified": False,
        "baseline_source": str(baseline_path),
        "baseline_source_sha256": sha256(baseline_payload),
        "candidate_source": str(output_path),
        "candidate_source_bytes": len(candidate_payload),
        "candidate_source_sha256": sha256(candidate_payload),
        "embedded_main_sha256": sha256(candidate_main_bytes),
        "embedded_main_bytes": len(candidate_main_bytes),
        "only_changed_line": assignment_index + 1,
        "changed_line_count": len(changed_lines),
        "main_methods_decorated": ["clear", "record", "_stage_summary"],
        "other_pipeline_bytes_preserved": True,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) +
                             "\n", encoding="utf-8", newline="")
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=BASELINE_RAM_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args(argv)
    manifest = build(args.baseline, args.output, args.manifest)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
