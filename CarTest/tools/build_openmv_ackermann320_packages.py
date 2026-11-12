"""Build the two allowlisted Ackermann 320x120 OpenMV upload packages.

This builder is intentionally separate from the historical Bottom80 profile
builder: it refuses to overwrite either delivery root and copies only the
19 runtime files declared by tools/checkpoint_version.py.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


PROJECT_DIR = Path(__file__).resolve().parents[1]
OPENMV_DIR = PROJECT_DIR / "openmv"
DELIVERY_ROOT = (PROJECT_DIR / "deliverables" /
                 "OpenMV_Ackermann320_20261022")
VALIDATION_ROOT = (PROJECT_DIR / "test-data" /
                   "OpenMV_Ackermann320_20261023-validation")
COMPILER = Path(
    r"D:\OpenMV IDE\share\qtcreator\mpy-cross\windows\mpy_cross"
    r"\mpy-cross.exe")
TARGET = "armv7emsp"
HOST_SUITE_COMMAND = "python -m unittest discover -s openmv/tests -p 'test_*.py'"
HOST_SUITE_COVERAGE = [
    "16-byte UART V2 fields, invalid-frame zeroing, CRC, and curve trace",
    "320x120 crop and 80x60 4x2 min-pool/mask equivalence",
    "road path, topology, bilateral/single-edge support, and boundary checks",
    "Q1000 row weighting, core-support degradation, and reference filtering",
    "PD/preview and speed-stage state transitions, including stale-frame guards",
    "per-frame overlay behavior, cached detail text, bounded summary FIFO",
    "paired package profiles, runtime allowlist, source hashes, and mpy-cross hashes",
]
RUNTIME_NAMES = (
    "adaptive_detector.py", "boundary_evidence.py", "config.py",
    "contour_route.py", "contour_trace.py", "fast_boundary.py",
    "fast_gray.py", "fast_path.py", "fast_single_edge.py",
    "fast_topology.py", "grayscale_adapter.py", "line_detector.py",
    "main.py", "reverse_control.py", "road_detector.py",
    "source_selector.py", "surface_detector.py", "thresholding.py",
    "vision_protocol.py",
)
PACKAGES = (
    {
        "name": "geometry_perf_only",
        "build_id": "OPENMV_ACKERMANN320_GEOMETRY_PERF_ONLY_20261022",
        "preview_gain": 0.35,
        "preview_limit": 100,
        "speed_stage": False,
    },
    {
        "name": "coordinated",
        "build_id": "OPENMV_ACKERMANN320_COORDINATED_20261022",
        "preview_gain": 0.45,
        "preview_limit": 120,
        "speed_stage": True,
    },
)
PACKAGE_README = """# OpenMV Ackermann 320x120 package: {name}

Build ID: `{build_id}`  
Runtime selector: `{name}`  
Preview bound: gain `{preview_gain}`, contribution limit `+/-{preview_limit}`  
Speed stage: `{speed_label}`

Upload the 19 top-level `.py` files from this folder together and restart the
OpenMV interpreter. Do not mix files between profiles, or upload the host-only
`.mpy` validation outputs. The package manifest lists every production module
and its source and cross-compile hashes.

Both packages use the same perception and route code. The sensor is configured
for QVGA and captures the explicit center crop `(x=0, y=60, width=320,
height=120)`. This changes the vertical field of view: the 320x240 sensor image
is cropped to its middle 120 rows; it is not resized or stretched. The 80x60
grid uses 4x2 min-pooling and examines every source pixel in the crop. Overlay
coordinates scale independently by x4 and y2.

Row weights are the fixed-point Q1000 table for
`w(y)=0.15 + 0.85*(1-((y-59.5)/59.5)^2)` over original crop rows y=0..119.
Weights peak in the center and taper toward both edges; normalization includes
only supported rows from the selected same-lane chain. Full weighting requires
at least three accepted core rows spanning two grid rows in original y=40..79.
If that core support is absent, the selected chain's nearest measured core row
is used and diagnostics mark `DEGRADED_CORE_SUPPORT`; a disconnected point
cannot take control. Existing route validity and topology checks remain in
force.

Both packages retain the existing PD controller and confirmed preview state
machine. `geometry_perf_only` keeps the prior preview limits (`0.35`, `100`)
and disables the independent speed request, so its V2 phase/request fields are
always zero. `coordinated` uses preview limits (`0.45`, `120`) and enables the
independent speed stage: after three fresh same-direction candidate frames
over at least 60 ms it requests slowdown `1000`; release evidence requires a
valid absolute filtered position below 50 and valid candidate magnitude below
80 for 300 ms. Invalid frames publish phase/request zero but do not clear the
internal speed latch. Only STOP or RESET clears that latch.

Both packages send the same 16-byte UART V2 wire format. Byte 10 is curve
phase (0 straight, 1 candidate, 2 turn, 3 exit), byte 11 is reserved zero,
bytes 12..13 are the little-endian slowdown request in permille, and bytes
14..15 contain CRC-16/CCITT-FALSE over bytes 2..13. Invalid observations keep
their incrementing sequence but clear all observation and control fields.
The existing 32-byte camera diagnostic and control-extension record layouts
remain intact; `curve_trace_v2_hex` stores the actual 16-byte V2 frame.

Validation: all 19 production `.py` modules were cross-compiled for
`armv7emsp` with the compiler recorded in `PACKAGE_MANIFEST.json`. The IDE
archive's mpy-cross is older than the reported board MicroPython runtime, so
this is source compatibility evidence, not proof that the camera accepts the
`.mpy` output or runs the source. Host regression tests also do not prove
hardware timing. Run the current host suite with
`{host_suite_command}`. It ran {host_tests} tests with
{host_skips} expected skips. It covers V2 framing/CRC and trace, the 4x2 image
geometry and mask equivalence, path/topology/boundary validation, weighted
reference and degraded core support, steering and speed-stage transitions,
overlay/summary behavior, and package manifests/hashes. The historical
`openmv/tests/run_v7_host_regression.py` uses frozen 160x120/2x2 fixtures and
does not validate this package's 320x120/4x2 geometry. The 60-new-frame/s
full-pipeline target is **not accepted** by this package: final board and
OpenMV IDE preview measurements remain separate required evidence. No camera
upload, H743 flash/reset, or motor start was performed by this builder.
"""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8", newline="")


def replace_exact_line(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError("expected exactly one config line: " + old)
    return text.replace(old, new, 1)


def package_source_text(source: Path, package: dict) -> str:
    text = source.read_text(encoding="utf-8")
    if source.name != "config.py":
        return text
    profile = package["name"]
    text = replace_exact_line(
        text, 'STEERING_CONTROL_PROFILE = "pd_preview"',
        'STEERING_CONTROL_PROFILE = "%s"' % profile)
    text = replace_exact_line(
        text, "PREVIEW_MAX_GAIN = 0.45",
        "PREVIEW_MAX_GAIN = %s" % package["preview_gain"])
    text = replace_exact_line(
        text, "PREVIEW_APPLIED_LIMIT = 120",
        "PREVIEW_APPLIED_LIMIT = %d" % package["preview_limit"])
    text = replace_exact_line(
        text, "SPEED_STAGE_ENABLED = True",
        "SPEED_STAGE_ENABLED = %s" %
        ("True" if package["speed_stage"] else "False"))
    return text


def verify_runtime_set() -> list[Path]:
    actual = {path.name for path in OPENMV_DIR.glob("*.py")}
    expected = set(RUNTIME_NAMES)
    if actual != expected | {"fast_ring_slope.py"}:
        raise RuntimeError(
            "production module set changed; checkpoint allowlist must be "
            "reviewed before packaging (actual=%r)" % sorted(actual))
    sources = [OPENMV_DIR / name for name in RUNTIME_NAMES]
    missing = [path.name for path in sources if not path.is_file()]
    if missing:
        raise RuntimeError("runtime source is missing: " + ", ".join(missing))
    return sources


def compiler_metadata() -> tuple[str, str]:
    result = subprocess.run([str(COMPILER), "--version"], capture_output=True,
                            text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError("cannot query mpy-cross version")
    version = result.stdout.strip() or result.stderr.strip()
    return version, sha256_file(COMPILER)


def run_host_suite(validation_root: Path) -> dict:
    command = [sys.executable, "-m", "unittest", "discover", "-s",
               "openmv/tests", "-p", "test_*.py"]
    result = subprocess.run(command, cwd=PROJECT_DIR, capture_output=True,
                            text=True, check=False)
    output = result.stdout + result.stderr
    (validation_root / "host-unittest.log").write_text(
        output, encoding="utf-8", newline="")
    if result.returncode != 0:
        raise RuntimeError("current OpenMV host suite failed (%d):\n%s" %
                           (result.returncode, output))
    match = re.search(r"Ran (\d+) tests?", output)
    if match is None:
        raise RuntimeError("host suite did not report its test count:\n" +
                           output)
    skip_match = re.search(r"skipped=(\d+)", output)
    skipped = int(skip_match.group(1)) if skip_match else 0
    return {
        "command": HOST_SUITE_COMMAND,
        "tests_run": int(match.group(1)),
        "skipped": skipped,
        "passed": int(match.group(1)) - skipped,
        "exit_code": result.returncode,
        "log_path": (VALIDATION_ROOT / "host-unittest.log").relative_to(
            PROJECT_DIR).as_posix(),
        "coverage": HOST_SUITE_COVERAGE,
        "historical_runner": {
            "path": "openmv/tests/run_v7_host_regression.py",
            "fixture_geometry": "160x120 with uniform 2x2 source-to-grid scale",
            "current_320x120_acceptance": False,
        },
    }


def build_package(package: dict, sources: list[Path], staging_root: Path,
                  compile_root: Path, compiler_version: str,
                  compiler_hash: str, host_suite: dict) -> dict:
    name = package["name"]
    package_dir = staging_root / name
    package_dir.mkdir(parents=True)
    compile_dir = compile_root / name
    compile_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    compiled_modules = []
    for source in sources:
        output_source = package_dir / source.name
        source_bytes = package_source_text(source, package).encode("utf-8")
        output_source.write_bytes(source_bytes)
        output_mpy = compile_dir / (source.stem + ".mpy")
        command = [str(COMPILER), "-march=" + TARGET, "-s", source.name,
                   "-o", str(output_mpy), str(output_source)]
        result = subprocess.run(command, capture_output=True, text=True,
                                check=False)
        if result.returncode != 0:
            raise RuntimeError(
                "mpy-cross failed for %s/%s (%d):\n%s\n%s" %
                (name, source.name, result.returncode,
                 result.stdout, result.stderr))
        source_hash = sha256_bytes(source_bytes)
        compiled_hash = sha256_file(output_mpy)
        relative_compiled = output_mpy.relative_to(PROJECT_DIR).as_posix()
        entries.append({
            "name": source.name,
            "size_bytes": len(source_bytes),
            "sha256": source_hash,
            "mpy_cross_exit_code": result.returncode,
            "compiled_size_bytes": output_mpy.stat().st_size,
            "compiled_sha256": compiled_hash,
            "compiled_validation_path": relative_compiled,
        })
        compiled_modules.append({
            "name": source.name,
            "exit_code": result.returncode,
            "source_sha256": source_hash,
            "compiled_size_bytes": output_mpy.stat().st_size,
            "compiled_sha256": compiled_hash,
            "compiled_validation_path": relative_compiled,
        })

    build_id = package["build_id"]
    readme = PACKAGE_README.format(
        name=name, build_id=build_id,
        preview_gain=package["preview_gain"],
        preview_limit=package["preview_limit"],
        speed_label="enabled" if package["speed_stage"] else "disabled",
        host_suite_command=host_suite["command"],
        host_tests=host_suite["tests_run"],
        host_skips=host_suite["skipped"])
    (package_dir / "README.md").write_text(readme, encoding="utf-8",
                                          newline="")
    manifest = {
        "schema_version": 1,
        "package_id": build_id,
        "build_id": build_id,
        "control_profile": name,
        "config_profile_selector": name,
        "release_status": "HOST_TESTED_CROSS_COMPILED_HARDWARE_VALIDATION_PENDING",
        "runtime_file_count": len(entries),
        "runtime_files": entries,
        "image_geometry": {
            "sensor_mode": "QVGA",
            "crop": {"x": 0, "y": 60, "width": 320, "height": 120},
            "resized": False,
            "grid": {"width": 80, "height": 60},
            "minpool": {"source_x": 4, "source_y": 2,
                        "all_source_pixels_examined": True},
            "overlay_scale": {"x": 4, "y": 2},
            "vertical_field_of_view": "center 120 rows of QVGA; reduced from full 240-row sensor image",
        },
        "row_weighting": {
            "formula": "w(y)=0.15+0.85*(1-((y-59.5)/59.5)^2)",
            "coordinate": "original crop y=0..119",
            "fixed_point_scale": 1000,
            "core_raw_y": [40, 79],
            "core_min_support_rows": 3,
            "core_min_support_span_grid_rows": 2,
            "fallback": "nearest measured core row from selected chain, marked DEGRADED_CORE_SUPPORT",
            "aggregation": "normalized across accepted support from selected same-lane chain",
        },
        "protocol": {
            "version": 2,
            "frame_size_bytes": 16,
            "curve_phase_offset": 10,
            "reserved_zero_offset": 11,
            "slowdown_request_permille_offset": 12,
            "crc_offset": 14,
            "crc": "CRC-16/CCITT-FALSE over bytes 2..13, little-endian",
            "invalid_sequence": "increments normally; observation/control fields zero",
        },
        "curve_trace": {
            "field": "curve_trace_v2_hex",
            "record_size_bytes": 16,
            "source": "the exact V2 UART payload sent for the same camera row",
            "camera_diag_record_size_bytes": 32,
            "control_ext_v2_record_size_bytes": 32,
        },
        "control": {
            "pd_enabled": True,
            "preview_enabled": True,
            "preview_max_gain": package["preview_gain"],
            "preview_applied_limit": package["preview_limit"],
            "preview_candidate_min_abs": 80,
            "preview_confirm_frames": 3,
            "preview_confirm_ms": 60,
            "preview_slew_per_second": 500,
            "speed_stage_enabled": package["speed_stage"],
            "speed_candidate_abs": 80,
            "speed_candidate_frames": 3,
            "speed_candidate_confirm_ms": 60,
            "slowdown_request_permille": 1000,
            "release_position_abs_below": 50,
            "release_candidate_abs_below": 80,
            "release_confirm_ms": 300,
            "exit_phase_ms": 800,
            "invalid_frame": "publish phase=0/request=0 but retain internal latch",
            "latch_clear_events": ["STOP", "RESET"],
        },
        "cross_compile": {
            "target": TARGET,
            "compiler_path": str(COMPILER),
            "compiler_version": compiler_version,
            "compiler_sha256": compiler_hash,
            "command_template": "mpy-cross -march=armv7emsp -s <module.py> -o <module.mpy> <module.py>",
            "module_count": len(compiled_modules),
            "all_exit_codes_zero": all(
                row["exit_code"] == 0 for row in compiled_modules),
            "scope": "MicroPython source compatibility for ARMv7EM-SP; not board runtime or Viper timing proof",
            "modules": compiled_modules,
        },
        "target_runtime_reported_version": "MicroPython 1.28.0-49",
        "matching_mpy_cross_compiler_available": False,
        "validation": {
            "host_suite_command": host_suite["command"],
            "host_suite_tests_run": host_suite["tests_run"],
            "host_suite_skipped": host_suite["skipped"],
            "host_suite_passed": host_suite["passed"],
            "host_suite_coverage": host_suite["coverage"],
            "historical_160x120_runner_is_current_acceptance": False,
            "hardware_full_pipeline_target_fps": 60,
            "hardware_full_pipeline_target_verified_for_this_package": False,
            "openmv_ide_preview_target_verified_for_this_package": False,
        },
        "hardware_validation": {
            "openmv_uploaded": False,
            "camera_board_execution_verified_for_this_package": False,
            "h743_flashed_or_reset": False,
            "motor_started": False,
            "vehicle_tested": False,
            "full_pipeline_60fps_verified": False,
        },
        "limits": [
            "Cross-compilation and host tests do not establish camera board runtime or vehicle behavior.",
            "The reported board runtime is newer than the available mpy-cross archive compiler.",
            "No final 60-new-frame/s full-pipeline acceptance is claimed.",
        ],
    }
    write_json(package_dir / "PACKAGE_MANIFEST.json", manifest)
    return {
        "control_profile": name,
        "build_id": build_id,
        "runtime_file_count": len(entries),
        "manifest_sha256": sha256_file(package_dir / "PACKAGE_MANIFEST.json"),
        "cross_compile_module_count": len(compiled_modules),
        "cross_compile_all_exit_codes_zero": all(
            row["exit_code"] == 0 for row in compiled_modules),
    }


def main() -> int:
    global COMPILER
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", type=Path, default=COMPILER)
    parser.add_argument(
        "--replace-existing", action="store_true",
        help="replace only this builder's previous package/validation outputs")
    args = parser.parse_args()
    COMPILER = args.compiler
    if not COMPILER.is_file():
        raise SystemExit("OpenMV mpy-cross compiler not found: %s" % COMPILER)
    if DELIVERY_ROOT.exists() and not args.replace_existing:
        raise SystemExit("refusing to overwrite existing package root: %s" %
                         DELIVERY_ROOT)
    if VALIDATION_ROOT.exists() and not args.replace_existing:
        raise SystemExit("refusing to overwrite existing validation root: %s" %
                         VALIDATION_ROOT)

    sources = verify_runtime_set()
    compiler_version, compiler_hash = compiler_metadata()
    temporary_parent = PROJECT_DIR / "tmp"
    temporary_parent.mkdir(parents=True, exist_ok=True)
    VALIDATION_ROOT.mkdir(parents=True, exist_ok=True)
    host_suite = run_host_suite(VALIDATION_ROOT)
    with tempfile.TemporaryDirectory(prefix="ackermann320-package-",
                                     dir=str(temporary_parent)) as temp_name:
        stage = Path(temp_name) / "deliverables"
        stage.mkdir()
        validation_compile_root = VALIDATION_ROOT / "mpy-cross"
        validation_compile_root.mkdir(parents=True, exist_ok=True)
        package_reports = []
        try:
            for package in PACKAGES:
                package_reports.append(build_package(
                    package, sources, stage, validation_compile_root,
                    compiler_version, compiler_hash, host_suite))
            DELIVERY_ROOT.parent.mkdir(parents=True, exist_ok=True)
            previous_root = Path(temp_name) / "previous-delivery"
            if DELIVERY_ROOT.exists():
                os.replace(DELIVERY_ROOT, previous_root)
            try:
                os.replace(stage, DELIVERY_ROOT)
            except Exception:
                if previous_root.exists():
                    os.replace(previous_root, DELIVERY_ROOT)
                raise
        except Exception:
            raise

    report = {
        "schema_version": 1,
        "created_by": "tools/build_openmv_ackermann320_packages.py",
        "delivery_root": DELIVERY_ROOT.relative_to(PROJECT_DIR).as_posix(),
        "compiler_path": str(COMPILER),
        "compiler_version": compiler_version,
        "compiler_sha256": compiler_hash,
        "target": TARGET,
        "packages": package_reports,
        "host_suite": host_suite,
        "all_compiles_passed": all(
            row["cross_compile_all_exit_codes_zero"]
            for row in package_reports),
        "hardware_upload_performed": False,
    }
    write_json(VALIDATION_ROOT / "BUILD_REPORT.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
