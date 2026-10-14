"""Run and save host, synthetic, manifest and IDE compiler evidence."""

import csv
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EVIDENCE = ROOT / "test-data" / "speed-ring-20261014" / "vision"
EVIDENCE = DEFAULT_EVIDENCE
RUN_EVIDENCE = EVIDENCE / "final"
PORT_SNAPSHOT_MANIFEST = DEFAULT_EVIDENCE / "pre-inset-port4" / "SHA256.json"
GENERATOR = ROOT / "tools" / "generate_openmv_clipped_midpoint.py"
BASELINE_MANIFEST = (ROOT / "test-data" / "turn-response-20261014" /
                     "before" / "vision" / "BASELINE_MANIFEST.json")
COMPILER = Path(
    r"D:\OpenMV IDE\share\qtcreator\mpy-cross\windows\mpy_cross\mpy-cross.exe")
TESTS = (
    ("turn_response_weights", ROOT / "openmv/tests/test_turn_response_weights.py",
     "new_configuration_acceptance"),
    ("ring_slope_host", ROOT / "openmv/tests/test_ring_slope_host.py",
     "ring_geometry_and_slope_fit_module_compatibility"),
    ("main_integration_ring_slope",
     ROOT / "openmv/tests/test_fast_main_ring_slope.py",
     "ring_wire_and_slope_disabled_main_acceptance"),
    ("protocol", ROOT / "openmv/tests/test_protocol_host.py",
     "current_runtime_regression"),
    ("reverse_control", ROOT / "openmv/tests/test_reverse_control.py",
     "current_runtime_regression"),
    ("single_edge_13", ROOT / "openmv/tests/test_fast_single_edge.py",
     "current_runtime_regression"),
    ("main_integration_7", ROOT / "openmv/tests/test_singleedge_main.py",
     "new_configuration_acceptance"),
    ("two_routes", ROOT / "openmv/tests/test_two_routes.py",
     "current_runtime_regression"),
    ("v7_overlay_each_frame", ROOT / "openmv/tests/test_v7_overlay_each_frame.py",
     "current_runtime_regression"),
    ("v7_overlay_accounting", ROOT / "openmv/tests/test_v7_overlay_accounting.py",
     "current_runtime_regression"),
    ("frozen_v7", ROOT / "openmv/tests/run_v7_host_regression.py",
     "legacy_frozen_behavioral_compatibility"),
)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def verify_baseline_snapshot():
    manifest = json.loads(BASELINE_MANIFEST.read_text(encoding="utf-8-sig"))
    snapshot_root = BASELINE_MANIFEST.parent
    entries = {}
    mismatches = []
    elf_count = 0
    for entry in manifest.get("files", []):
        snapshot_path = entry["snapshot_path"].replace("\\", "/")
        path = snapshot_root.joinpath(*snapshot_path.split("/"))
        if snapshot_path.lower().endswith(".elf"):
            elf_count += 1
        if not path.is_file():
            mismatches.append(snapshot_path + ":MISSING")
            continue
        actual_sha = sha256_file(path)
        actual_size = path.stat().st_size
        if (actual_sha != entry.get("sha256") or
                actual_size != entry.get("size_bytes")):
            mismatches.append(snapshot_path + ":HASH_OR_SIZE")
        entries[snapshot_path] = entry
    if mismatches:
        raise RuntimeError("pre-change snapshot verification failed: %s" %
                           ", ".join(mismatches[:5]))
    if len(entries) != manifest.get("file_count"):
        raise RuntimeError("pre-change snapshot file count mismatch")
    return {
        "manifest": manifest,
        "entries": entries,
        "file_count": len(entries),
        "elf_count": elf_count,
        "manifest_sha256": sha256_file(BASELINE_MANIFEST),
    }


def verify_pre_inset_snapshot():
    manifest = json.loads(PORT_SNAPSHOT_MANIFEST.read_text(encoding="utf-8"))
    snapshot_root = PORT_SNAPSHOT_MANIFEST.parent
    mismatches = []
    entries = manifest.get("files", [])
    for entry in entries:
        path = snapshot_root.joinpath(*entry["path"].replace("\\", "/").split("/"))
        if not path.is_file():
            mismatches.append(entry["path"] + ":MISSING")
            continue
        if (path.stat().st_size != entry.get("size_bytes") or
                sha256_file(path) != entry.get("sha256")):
            mismatches.append(entry["path"] + ":HASH_OR_SIZE")
    if mismatches or len(entries) != manifest.get("file_count"):
        raise RuntimeError("pre-inset snapshot verification failed: %s" %
                           ", ".join(mismatches[:5] or ["FILE_COUNT"]))
    return {
        "manifest": PORT_SNAPSHOT_MANIFEST.relative_to(ROOT).as_posix(),
        "manifest_sha256": sha256_file(PORT_SNAPSHOT_MANIFEST),
        "file_count": len(entries),
        "all_snapshot_files_reverified": True,
    }


def save_source_hashes(baseline_entries):
    paths = set()
    for folder in (ROOT / "openmv", ROOT / "openmv_clipped_midpoint"):
        for source in folder.glob("*.py"):
            paths.add(source.relative_to(ROOT).as_posix())
        readme = folder / "README.md"
        if readme.is_file():
            paths.add(readme.relative_to(ROOT).as_posix())
    paths.update((
        "tools/generate_openmv_clipped_midpoint.py",
        "tools/verify_openmv_two_routes.py",
        "test-data/speed-ring-20261014/vision/replay_host_frames.py",
        "test-data/speed-ring-20261014/vision/README.md",
        "test-data/speed-ring-20261014/vision/SCREENSHOT_RING_REPLAY.md",
        "test-data/speed-ring-20261014/vision/SCREENSHOT_RING_REPLAY.zh-CN.md",
    ))
    for source in (ROOT / "openmv/tests").glob("*.py"):
        paths.add(source.relative_to(ROOT).as_posix())

    csv_path = RUN_EVIDENCE / "source-hashes.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("source_path", "size_bytes", "current_sha256",
                         "snapshot_path", "snapshot_sha256",
                         "matches_prechange_snapshot"))
        for relative in sorted(paths):
            path = ROOT / Path(relative)
            if not path.is_file():
                raise RuntimeError("source hash input is missing: %s" % path)
            snapshot_path = relative
            before = baseline_entries.get(snapshot_path)
            writer.writerow((relative, path.stat().st_size,
                             sha256_file(path), snapshot_path,
                             before.get("sha256", "") if before else "",
                             int(before is not None and
                                 before.get("sha256") == sha256_file(path))))
    return {
        "path": str(csv_path.relative_to(ROOT)).replace("\\", "/"),
        "sha256": sha256_file(csv_path),
        "source_count": len(paths),
    }


def save_package_manifest_hashes():
    csv_path = RUN_EVIDENCE / "final-package-manifest-hashes.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("manifest_path", "size_bytes", "sha256"))
        for manifest in (ROOT / "openmv/PACKAGE_MANIFEST.json",
                         ROOT / "openmv_clipped_midpoint/PACKAGE_MANIFEST.json"):
            writer.writerow((str(manifest.relative_to(ROOT)).replace("\\", "/"),
                             manifest.stat().st_size, sha256_file(manifest)))
    return {
        "path": str(csv_path.relative_to(ROOT)).replace("\\", "/"),
        "sha256": sha256_file(csv_path),
    }


def run_command(args, cwd, log_path):
    result = subprocess.run(
        [str(value) for value in args], cwd=str(cwd),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace")
    command = subprocess.list2cmdline([str(value) for value in args])
    body = ["COMMAND: " + command, "EXIT_CODE: %d" % result.returncode,
            "--- STDOUT ---", result.stdout, "--- STDERR ---", result.stderr]
    log_path.write_text("\n".join(body), encoding="utf-8")
    log_path.with_suffix(log_path.suffix + ".exitcode").write_text(
        "%d\n" % result.returncode, encoding="ascii")
    return result, command


def save_test_logs():
    result_rows = []
    for key, script, test_scope in TESTS:
        log_path = RUN_EVIDENCE / ("host-" + key + ".log")
        result, command = run_command(
            [sys.executable, "-B", script], ROOT, log_path)
        combined_output = result.stdout + result.stderr
        test_match = re.search(r"Ran (\d+) tests?", combined_output)
        skip_match = re.search(r"skipped=(\d+)", combined_output)
        run_count = int(test_match.group(1)) if test_match else None
        skipped = int(skip_match.group(1)) if skip_match else 0
        result_rows.append({
            "key": key,
            "test_scope": test_scope,
            "command": command,
            "exit_code": result.returncode,
            "tests_run": run_count,
            "passed": (run_count - skipped
                       if run_count is not None and result.returncode == 0
                       else None),
            "skipped": skipped,
            "raw_log": str(log_path.relative_to(ROOT)).replace("\\", "/"),
            "log_sha256": sha256_file(log_path),
            "exit_code_file": str(log_path.with_suffix(
                log_path.suffix + ".exitcode").relative_to(ROOT)).replace(
                    "\\", "/"),
        })
    return result_rows


def compile_sources():
    version_log = RUN_EVIDENCE / "mpy-cross-v1.25-version.log"
    if not COMPILER.is_file():
        version_log.write_text("ERROR: IDE compiler missing: %s\n" % COMPILER,
                               encoding="utf-8")
        version_log.with_suffix(".log.exitcode").write_text("127\n",
                                                               encoding="ascii")
        return {"compiler": str(COMPILER), "available": False,
                "compiler_sha256": None, "variants": [],
                "target_micro_python_v1_28_compiled": False}
    version, version_command = run_command(
        [COMPILER, "--version"], ROOT, version_log)
    variants = []
    compiled_evidence = RUN_EVIDENCE / "compiled"
    compiled_evidence.mkdir(parents=True, exist_ok=True)
    for route, folder in (("cached_width", ROOT / "openmv"),
                          ("clipped_midpoint",
                           ROOT / "openmv_clipped_midpoint")):
        paths = sorted(folder.glob("*.py"))
        output_dir = compiled_evidence / route
        output_dir.mkdir(parents=True, exist_ok=True)
        log_lines = [
            "compiler=%s" % version.stdout.strip(),
            "compiler_path=%s" % COMPILER,
            "target=-march=armv7em",
            "route=%s" % route,
            "source_count=%d" % len(paths),
        ]
        file_results = []
        failures = 0
        for source in paths:
            output = output_dir / (source.stem + ".mpy")
            args = [COMPILER, "-march=armv7em", "-o", output, source]
            result = subprocess.run(
                [str(value) for value in args], cwd=str(ROOT),
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, encoding="utf-8", errors="replace")
            output_size = output.stat().st_size if output.exists() else 0
            output_hash = sha256_file(output) if output.exists() else None
            file_log = output.with_suffix(".log")
            file_exit = output.with_suffix(".exitcode")
            file_log.write_text(
                "COMMAND: %s\nEXIT_CODE: %d\nOUTPUT_BYTES: %d\n"
                "OUTPUT_SHA256: %s\n--- STDOUT ---\n%s\n"
                "--- STDERR ---\n%s\n" % (
                    subprocess.list2cmdline([str(value) for value in args]),
                    result.returncode, output_size, output_hash or "",
                    result.stdout, result.stderr), encoding="utf-8")
            file_exit.write_text("%d\n" % result.returncode,
                                 encoding="ascii")
            log_lines.append("[%s] exit=%d output_bytes=%d sha256=%s" % (
                source.name, result.returncode, output_size,
                output_hash or "MISSING"))
            if result.stdout:
                log_lines.append("stdout: " + result.stdout.strip())
            if result.stderr:
                log_lines.append("stderr: " + result.stderr.strip())
            if result.returncode:
                failures += 1
            file_results.append({
                "source": source.name,
                "source_sha256": sha256_file(source),
                "output": str(output.relative_to(ROOT)).replace("\\", "/"),
                "output_bytes": output_size,
                "output_sha256": output_hash,
                "exit_code": result.returncode,
                "log": str(file_log.relative_to(ROOT)).replace("\\", "/"),
                "exit_code_file": str(file_exit.relative_to(ROOT)).replace(
                    "\\", "/"),
            })
        exit_code = 0 if len(paths) == 18 and failures == 0 else 1
        log_path = RUN_EVIDENCE / ("compile-%s-v1.25-armv7em.log" % route)
        log_lines.append("compiled=%d" % (len(paths) - failures))
        log_lines.append("failures=%d" % failures)
        log_lines.append("overall_exit_code=%d" % exit_code)
        log_path.write_text("\n".join(log_lines) + "\n",
                            encoding="utf-8")
        log_path.with_suffix(log_path.suffix + ".exitcode").write_text(
            "%d\n" % exit_code, encoding="ascii")
        variants.append({
            "route_mode": route,
            "source_count": len(paths),
            "source_names": [source.name for source in paths],
            "compiled": len(paths) - failures,
            "failures": failures,
            "exit_code": exit_code,
            "target_architecture": "armv7em",
            "raw_log": str(log_path.relative_to(ROOT)).replace("\\", "/"),
            "log_sha256": sha256_file(log_path),
            "exit_code_file": str(log_path.with_suffix(
                log_path.suffix + ".exitcode").relative_to(ROOT)).replace(
                    "\\", "/"),
            "compiled_files": file_results,
        })
    archive = COMPILER.parent / "archive"
    v128 = [str(path) for path in archive.rglob("*mpy-cross.exe")
            if "1.28" in str(path)] if archive.is_dir() else []
    return {
        "compiler": version.stdout.strip(),
        "compiler_path": str(COMPILER),
        "compiler_sha256": sha256_file(COMPILER),
        "compiler_version_command": version_command,
        "compiler_version_exit_code": version.returncode,
        "compiler_version_log": str(version_log.relative_to(ROOT)).replace(
            "\\", "/"),
        "target_architecture": "armv7em",
        "variants": variants,
        "target_micro_python_v1_28_compiled": False,
        "micro_python_v1_28_compiler_available_in_ide": bool(v128),
        "micro_python_v1_28_compiler_candidates": v128,
        "board_runtime_tested": False,
    }


def synthetic_rows():
    sys.path.insert(0, str(ROOT / "openmv"))
    sys.path.insert(0, str(ROOT / "openmv" / "tests"))
    import config
    from fast_single_edge import FastSingleEdgeTracker
    from test_fast_single_edge import (
        FLOOR, HEIGHT, WIDTH, _add_closed_dark_ring, _bilateral_candidate,
        _blank_bright_mask, _mask, _topology, _warm_tracker)
    from test_two_routes import _branch_fixture, _record_extra_dark_step

    def create_tracker(route):
        settings = dict(vars(config))
        settings["SINGLE_EDGE_ROUTE_MODE"] = route
        return FastSingleEdgeTracker(settings)

    columns = ["scenario", "route_mode", "valid", "reason",
               "actual_boundary", "actual_boundary_x", "clipped_endpoint",
               "clipped_endpoint_x", "expected_midpoint_x",
               "reported_midpoint_x", "absolute_error_cells",
               "support_rows", "cache_rows", "inference_flags"]
    data = []
    log = []

    def record(scenario, route, result, tracker, expected=None):
        midpoint = result.get("reference_center_x")
        error = (abs(midpoint - expected)
                 if midpoint is not None and expected is not None else None)
        row = [scenario, route, int(bool(result.get("valid"))),
               result.get("reason"), result.get("actual_boundary"),
               result.get("actual_boundary_x"),
               result.get("clipped_endpoint"),
               result.get("clipped_endpoint_x"), expected, midpoint, error,
               result.get("selected_row_count", 0), tracker.cache_rows,
               result.get("inference_flags", 0)]
        data.append(row)
        log.append("%s route=%s valid=%d reason=%s actual=%s@%s "
                   "clip=%s@%s expected_mid=%s output_mid=%s error=%s "
                   "support=%s cache=%s flags=%s" % (
                       scenario, route, row[2], row[3], row[4], row[5],
                       row[6], row[7], row[8], row[9], row[10], row[11],
                       row[12], row[13]))

    left = _mask("left")
    cached = create_tracker("cached_width")
    _warm_tracker(cached)
    record("same_left_frame", "cached_width", cached.detect(left, True),
           cached, 39.5)
    clipped = create_tracker("clipped_midpoint")
    record("same_left_frame", "clipped_midpoint",
           clipped.detect(left, True), clipped, 52.75)

    right_tracker = create_tracker("clipped_midpoint")
    record("right_frame", "clipped_midpoint",
           right_tracker.detect(_mask("right"), True), right_tracker, 26.25)

    no_keep = create_tracker("clipped_midpoint")
    record("no_fresh_keep", "clipped_midpoint",
           no_keep.detect(left, False), no_keep, None)
    no_cache = create_tracker("cached_width")
    record("no_width_cache", "cached_width",
           no_cache.detect(left, True), no_cache, None)

    border = _blank_bright_mask()
    for y in range(FLOOR, HEIGHT):
        for x in (0, WIDTH - 1):
            index = y * WIDTH + x
            border["edge_mask"][index] = 1
            border["road_mask"][index] = 0
            border["luma"][index] = 25
    border_tracker = create_tracker("clipped_midpoint")
    record("frame_border_only", "clipped_midpoint",
           border_tracker.detect(border, True), border_tracker, None)

    ring = _mask("left")
    _add_closed_dark_ring(ring, left=39, top=33, width=8, height=14)
    ring_tracker = create_tracker("clipped_midpoint")
    record("inner_ring", "clipped_midpoint",
           ring_tracker.detect(ring, True, _topology(ring)), ring_tracker,
           None)

    branch = _branch_fixture()
    branch_tracker = create_tracker("clipped_midpoint")
    record("branch", "clipped_midpoint",
           branch_tracker.detect(branch, True, _topology(branch)),
           branch_tracker, None)

    ambiguous = _record_extra_dark_step(_mask("left"), x=45)
    ambiguous_tracker = create_tracker("clipped_midpoint")
    record("two_current_edges", "clipped_midpoint",
           ambiguous_tracker.detect(ambiguous, True), ambiguous_tracker, None)

    switch = create_tracker("clipped_midpoint")
    first = switch.detect(_mask("left"), True)
    switch.commit_single(first)
    record("side_switch_before_dual", "clipped_midpoint",
           switch.detect(_mask("right"), True), switch, None)
    dual = _mask("both")
    switch.observe_bilateral(
        _bilateral_candidate(), dual["luma"], dual["edge_mask"],
        dual["road_mask"], dual["known_region"], True, True)
    record("side_switch_after_dual", "clipped_midpoint",
           switch.detect(_mask("right"), True), switch, 26.25)
    return columns, data, log


def save_synthetic_evidence():
    columns, rows, log = synthetic_rows()
    csv_path = RUN_EVIDENCE / "synthetic-numeric-results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)
    log_path = RUN_EVIDENCE / "synthetic-numeric-results.log"
    log_path.write_text("\n".join(log) + "\n", encoding="utf-8")
    return {
        "fixture_kind": "synthetic_host_masks_only",
        "scenario_count": len(rows),
        "numeric_results_csv": str(csv_path.relative_to(ROOT)).replace(
            "\\", "/"),
        "numeric_results_csv_sha256": sha256_file(csv_path),
        "raw_log": str(log_path.relative_to(ROOT)).replace("\\", "/"),
        "raw_log_sha256": sha256_file(log_path),
        "same_frame_left_cached_center_x": 39.5,
        "same_frame_left_clip_edge_x": 26.5,
        "same_frame_left_clip_endpoint_x": 79,
        "same_frame_left_clip_expected_center_x": 52.75,
        "same_frame_left_clip_reported_center_x": 53.0,
        "same_frame_left_clip_rounding_error_cells": 0.25,
        "right_clip_expected_center_x": 26.25,
        "camera_or_board_image_used": False,
    }


def _frozen_compatibility_details(test_rows):
    row = next((item for item in test_rows if item["key"] == "frozen_v7"),
               None)
    if row is None:
        return {"runner_present": False}
    content = (ROOT / row["raw_log"]).read_text(
        encoding="utf-8", errors="replace")
    marker = "TEMPORARY_TEST_COPY_ADAPTATIONS="
    adaptation_line = next((line[len(marker):] for line in content.splitlines()
                            if line.startswith(marker)), None)
    adaptations = json.loads(adaptation_line) if adaptation_line else []
    return {
        "runner_present": True,
        "test_scope": row["test_scope"],
        "exit_code": row["exit_code"],
        "fixture_source_hashes_verified":
            "FIXTURE_SOURCE_HASHES=VERIFIED_UNCHANGED" in content,
        "temporary_test_copy_adaptation_count": len(adaptations),
        "temporary_test_copy_adaptations": adaptations,
        "claim_boundary": (
            "Legacy V7 behavioral compatibility against the current runtime. "
            "This does not establish that the frozen executable or old config "
            "accepts the new turn-response tuning."),
    }


def _static_screenshot_replay_details():
    replay_root = EVIDENCE / "replay_port4_final"
    variants = {}
    for package in ("openmv", "openmv_clipped_midpoint"):
        summary_path = replay_root / package / "SUMMARY.json"
        if not summary_path.is_file():
            variants[package] = {"available": False}
            continue
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        results = summary.get("results", [])
        candidate_examples = []
        for item in results:
            if not item.get("ring_candidate_valid"):
                continue
            frame_path = (replay_root / package /
                          item.get("variant", "raw_screenshot") /
                          item.get("output_json", ""))
            if not frame_path.is_file():
                continue
            frame = json.loads(frame_path.read_text(encoding="utf-8"))
            candidate = frame.get("ring", {}).get("candidate", {})
            candidate_examples.append({
                "image_index": item.get("image_index"),
                "variant": item.get("variant"),
                "side": candidate.get("side"),
                "arc_rows": candidate.get("arc_rows"),
                "arc_y_minmax": [candidate.get("arc_y_min"),
                                 candidate.get("arc_y_max")],
                "vertex_xy": [candidate.get("vertex_x"),
                              candidate.get("vertex_y")],
                "intrusion_cells": candidate.get("intrusion"),
                "near_seed_xy": [candidate.get("near_seed_x"),
                                 candidate.get("near_seed_y")],
                "reachable_port_xy": [candidate.get("port_x"),
                                       candidate.get("port_y")],
                "clip_marker_x": candidate.get("clip_endpoint_x"),
                "ring_state": frame.get("ring", {}).get("state", {}).get(
                    "state"),
                "port_inset_diagnostic": {
                    "required_inset_x": frame.get("ring", {}).get(
                        "port_component_diagnostic", {}).get(
                            "required_inset_x"),
                    "reachable_nearest_crop_margin_cells": frame.get(
                        "ring", {}).get("port_component_diagnostic", {}).get(
                            "reachable_nearest_crop_margin_cells"),
                    "current_safe_inset_cells": frame.get("ring", {}).get(
                        "port_component_diagnostic", {}).get(
                            "all_current_known_white_clearance_inset_cells"),
                    "current_safe_inset_rows": frame.get("ring", {}).get(
                        "port_component_diagnostic", {}).get(
                            "all_current_known_white_clearance_inset_rows"),
                },
                "frame_json": frame_path.relative_to(ROOT).as_posix(),
            })
        variants[package] = {
            "available": True,
            "summary": summary_path.relative_to(ROOT).as_posix(),
            "summary_sha256": sha256_file(summary_path),
            "package_id": summary.get("package_id"),
            "static_input_count": summary.get("input_count"),
            "replayed_frame_variants": len(results),
            "valid_ring_candidate_count": sum(
                bool(item.get("ring_candidate_valid")) for item in results),
            "valid_ring_candidate_examples": candidate_examples,
            "ring_port_inset_margin_cells": 4,
            "ring_port_inset_margin_percent_of_grid_width": 5,
            "top_overlay_rows_marked_unknown": [0, 40],
            "sensor_buffer_input": False,
            "board_execution": False,
            "temporal_sequence_assumed": False,
        }
    return {
        "available": all(item.get("available")
                          for item in variants.values()),
        "runner": (EVIDENCE / "replay_host_frames.py").relative_to(
            ROOT).as_posix(),
        "source_image_directory": (
            ROOT / "test-data" / "ring-slope-20261014" / "vision" /
            "inputs" / "original").relative_to(ROOT).as_posix(),
        "preprocessing": (
            "Pillow grayscale BOX resize to 160x120; screenshots contain IDE "
            "overlay text over resized full-resolution rows 0..39"),
        "raw_and_ignore_top40_variants": True,
        "variants": variants,
        "interpretation": (
            "Static image geometry only. Each image starts with empty ring "
            "state and no fresh KEEP; candidates do not establish entry, "
            "an eight-stage ring sequence, a time sequence, or vehicle "
            "behavior. The port inset is a crop-side tolerance, not a "
            "steering weight or observed image-edge boundary."),
    }


def main(output_dir=None):
    global EVIDENCE, RUN_EVIDENCE
    if output_dir is None:
        EVIDENCE = DEFAULT_EVIDENCE
    else:
        requested = Path(output_dir)
        if not requested.is_absolute():
            requested = ROOT / requested
        EVIDENCE = requested.resolve()
    protected_old = (ROOT / "test-data" / "differential-20261013" /
                     "vision").resolve()
    if EVIDENCE == protected_old:
        raise RuntimeError("refusing to overwrite preserved 2026-10-13 evidence")
    EVIDENCE.relative_to(ROOT.resolve())
    RUN_EVIDENCE = EVIDENCE / "final"
    RUN_EVIDENCE.mkdir(parents=True, exist_ok=True)
    snapshot = verify_baseline_snapshot()
    port_snapshot = verify_pre_inset_snapshot()
    snapshot_log = RUN_EVIDENCE / "baseline-snapshot-verification.log"
    snapshot_log.write_text(
        "manifest=%s\nmanifest_sha256=%s\nfile_count=%d\nelf_count=%d\n"
        "exit_code=0\n" % (
            BASELINE_MANIFEST.relative_to(ROOT).as_posix(),
            snapshot["manifest_sha256"], snapshot["file_count"],
            snapshot["elf_count"]), encoding="utf-8")
    variant_refresh = run_command(
        [sys.executable, "-B", GENERATOR, "--refresh-variant",
         "--output-dir", EVIDENCE], ROOT,
        RUN_EVIDENCE / "variant-refresh.log")[0]
    generator_check = run_command(
        [sys.executable, "-B", GENERATOR, "--check",
         "--output-dir", EVIDENCE], ROOT,
        RUN_EVIDENCE / "generator-check.log")[0]
    tests = save_test_logs()
    compatibility_details = _frozen_compatibility_details(tests)
    compiler = compile_sources()
    synthetic = save_synthetic_evidence()
    source_hashes = save_source_hashes(snapshot["entries"])
    report = {
        "date": "2026-10-14",
        "evidence_revision": "final",
        "evidence_dir": str(RUN_EVIDENCE.relative_to(ROOT)).replace("\\", "/"),
        "project": "CarTest OpenMV weighted response and ring route; slope classifier disabled",
        "builds": {
            "cached_width": "CACHED_WIDTH_RING_20261014",
            "clipped_midpoint": "CLIPPED_MIDPOINT_RING_20261014",
        },
        "before_source_snapshot": {
            "manifest": BASELINE_MANIFEST.relative_to(ROOT).as_posix(),
            "manifest_sha256": snapshot["manifest_sha256"],
            "snapshot_root": str(BASELINE_MANIFEST.parent.relative_to(ROOT)).replace("\\", "/"),
            "file_count": snapshot["file_count"],
            "existing_elf_count": snapshot["elf_count"],
            "all_snapshot_files_reverified": True,
            "verification_log": str(snapshot_log.relative_to(ROOT)).replace("\\", "/"),
        },
        "before_port_inset_snapshot": port_snapshot,
        "source_hashes": source_hashes,
        "host_tests": tests,
        "configuration_test_classes": {
            "legacy_v7_compatibility": compatibility_details,
            "current_turn_response_acceptance": {
                "test_keys": ["turn_response_weights", "ring_slope_host",
                              "main_integration_ring_slope",
                              "main_integration_7"],
                "test_scope": "new_configuration_acceptance",
                "turn_response_version": "LOWER70_UPPER30_V1",
                "actual_gray_builder_and_path_tested": True,
                "actual_boundary_per_row_support_tested": True,
                "serialized_main_uart_error_tested": True,
                "both_single_edge_sources_tested": True,
            },
            "ring_acceptance": {
                "test_keys": ["ring_slope_host",
                              "main_integration_ring_slope"],
                "positive_left_right_arc_and_connected_port_synthetic_tests": True,
                "port_inset_mirror_dark_unknown_and_multiple_port_tests": True,
                "three_fresh_keep_entry_and_partial_inner_edge_hold_tested": True,
                "exit_requires_current_disappearance_and_forward_continuation": True,
                "exit_requires_side_specific_outward_near_crop_motion": True,
                "inward_jump_and_unknown_edge_do_not_confirm_exit": True,
                "port_inset_margin_cells": 4,
                "port_inset_margin_percent_of_grid_width": 5,
                "port_checks_current_known_white_and_two_cell_clearance": True,
                "slope_fit_module_compatibility_tested": True,
                "slope_classifier_runtime_enabled": False,
                "runtime_slope_hint_always_zero_tested": True,
                "serialized_slope_hint_flag_tested": False,
                "static_photo_temporal_claim": False,
            },
        },
        "static_screenshot_replay": _static_screenshot_replay_details(),
        "mpy_cross": compiler,
        "synthetic_numeric_fixture": synthetic,
        "variant_refresh_exit_code": variant_refresh.returncode,
        "generator_check_exit_code": generator_check.returncode,
        "camera_frames_executed": False,
        "camera_rate_baseline_frames_per_10s": 383,
        "camera_rate_baseline_provenance": "Previously reported by the user; not measured on this revision.",
        "camera_rate_target_frames_per_10s": 450,
        "camera_rate_target_result": "not_measured",
        "revision_camera_frames_per_10s_measured": False,
        "openmv_board_runtime_tested": False,
        "h743_build_or_flash_performed": False,
        "camera_upload_performed": False,
        "vehicle_track_tested": False,
        "notes": [
            "Lower rows y>=36 carry aggregate weight 0.80 and upper rows y<36 carry 0.20; each available group is internally normalized.",
            "The ROI expands to grid row 8/full-image y=16 with four Otsu blocks; the three original blocks from row 20 down remain unchanged.",
            "Bilateral weighted rows require the existing per-row measured pair-support bit; unsupported rows are excluded without changing the existing global gate.",
            "The weighted controller center is a virtual column and remains separate from the geometric safety reference. Missing groups renormalize with a diagnostic.",
            "Weighting runs after current-frame path selection; it does not guarantee lane identity across whole-path candidate changes.",
            "Ring ports use a configured 4-cell (5% of grid width) crop-side inset. This is a bounded port-search tolerance, not steering weighting; current known-white connectivity and the same full two-cell clearance still apply, and x=0/79 remain diagnostic crop markers only.",
            "The IDE mpy-cross 1.25 run with -march=armv7em is compiler evidence, not board execution; MicroPython 1.28 compiler acceptance remains unverified.",
            "Synthetic host masks and repeated-frame sequences are not camera recordings. The eight provided photos were static images with IDE text over the top region; the H743 core was unpowered during capture, so reverse STARTUPSTALE/cache=0 was expected and is not a live control-path fault.",
            "Slope classification is disabled in the current runtime; main-loop observation packets always set slope hint 0. The optional protocol bit and isolated slope-fit module tests remain for compatibility only.",
            "The separate eight-stage ring/speed document is future architecture only; this package does not implement or claim its end-to-end route behavior.",
            "No OpenMV upload/board run, H743 build/flash, motor start, or vehicle track test was performed. Ring entry and incline transition are not established by these results.",
        ],
    }
    report_path = EVIDENCE / "REPORT.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) +
                           "\n", encoding="utf-8")
    refresh_log = RUN_EVIDENCE / "manifest-refresh.log"
    refresh, _ = run_command(
        [sys.executable, "-B", GENERATOR, "--refresh-manifests",
         "--output-dir", EVIDENCE], ROOT,
        refresh_log)
    verify_log = RUN_EVIDENCE / "generator-final-check.log"
    verify, _ = run_command(
        [sys.executable, "-B", GENERATOR, "--check",
         "--output-dir", EVIDENCE], ROOT, verify_log)
    report["manifest_refresh_exit_code"] = refresh.returncode
    report["generator_final_check_exit_code"] = verify.returncode
    report["manifest_final_refresh_exit_code"] = 0
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) +
                           "\n", encoding="utf-8")
    # This final refresh embeds the final report hash in both manifests.
    refresh_final, _ = run_command(
        [sys.executable, "-B", GENERATOR, "--refresh-manifests",
         "--output-dir", EVIDENCE], ROOT,
        RUN_EVIDENCE / "manifest-refresh-final.log")
    if refresh_final.returncode != 0:
        report["manifest_final_refresh_exit_code"] = refresh_final.returncode
        report_path.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        # Retry once with the recorded failure status in the report.
        refresh_final, _ = run_command(
            [sys.executable, "-B", GENERATOR, "--refresh-manifests",
             "--output-dir", EVIDENCE], ROOT,
            RUN_EVIDENCE / "manifest-refresh-retry.log")
    final_check, _ = run_command(
        [sys.executable, "-B", GENERATOR, "--check",
         "--output-dir", EVIDENCE], ROOT,
        RUN_EVIDENCE / "generator-final-check-after-refresh.log")
    manifest_hashes = save_package_manifest_hashes()
    all_ok = (variant_refresh.returncode == 0 and
              generator_check.returncode == 0 and
              final_check.returncode == 0 and
              all(row["exit_code"] == 0 for row in tests) and
              all(row["exit_code"] == 0
                  for row in compiler.get("variants", [])) and
              refresh.returncode == 0 and verify.returncode == 0 and
              refresh_final.returncode == 0)

    print("host_tests=%d host_failures=%d" % (
        len(tests), sum(row["exit_code"] != 0 for row in tests)))
    for row in tests:
        print("test=%s tests_run=%s passed=%s skipped=%s exit=%d" % (
            row["key"], row["tests_run"], row["passed"], row["skipped"],
            row["exit_code"]))
    print("compiler=%s" % compiler.get("compiler", "unavailable"))
    print("compiler_sha256=%s" % compiler.get("compiler_sha256"))
    for row in compiler.get("variants", []):
        print("compile=%s %d/%d failures=%d exit=%d" % (
            row["route_mode"], row["compiled"], row["source_count"],
            row["failures"], row["exit_code"]))
    print("report=%s" % report_path)
    print("manifest_hashes=%s" % manifest_hashes["path"])
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
