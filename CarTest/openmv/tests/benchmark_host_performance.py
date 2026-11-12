"""Host-only timing for row-workspace clearing and controller-profile overhead."""

import hashlib
import json
import platform
import statistics
import sys
import time
import types
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
TEST_DIR = Path(__file__).resolve().parent
PROJECT_DIR = OPENMV_DIR.parent
for path in (OPENMV_DIR, TEST_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))
import config
import main
from test_turn_response_weights import (
    _FullClearReferenceWorkspace, _path_candidate)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _make_case(rows, support=None):
    candidate = _path_candidate(rows)
    if support is None:
        support = [1] * len(rows)
    candidate["boundary_support_rows"] = bytearray(support)
    candidate["boundary_support_count"] = len(rows)
    return {"valid": True, "source": "road", "result": candidate}, candidate


def _workspace_cases():
    dense_rows = []
    for y in range(config.SURFACE_ROI_FLOOR_Y,
                   config.SURFACE_GRID_HEIGHT):
        center = 25 + (y * 7 % 20)
        dense_rows.append((y, center - 8, center + 8))
    sparse_rows = [
        (8, 20, 44), (10, 21, 45), (20, 25, 49),
        (24, 27, 51), (40, 29, 53), (58, 31, 55),
    ]
    duplicate_rows = [
        (20, 22, 50), (20, 25, 53), (39, 24, 52),
        (40, 24, 52), (41, 24, 52), (55, 26, 54),
    ]
    cases = [
        _make_case(dense_rows),
        _make_case(sparse_rows),
        _make_case(duplicate_rows),
        _make_case([(10, 20, 50), (40, 24, 54)], [0, 0]),
    ]
    return cases


def _time_workspace(workspace_type, cases, rounds=7, iterations=1400):
    samples = []
    checksum = None
    for round_index in range(rounds):
        workspace = workspace_type()
        for index in range(80):
            selection, candidate = cases[index % len(cases)]
            workspace.aggregate(selection, candidate, config)
        value = 0
        start = time.perf_counter_ns()
        for index in range(iterations):
            selection, candidate = cases[index % len(cases)]
            info = workspace.aggregate(selection, candidate, config)
            value += info["raw_error"] + info["supported_rows"]
        elapsed = time.perf_counter_ns() - start
        samples.append(elapsed / float(iterations))
        if checksum is None:
            checksum = value
        elif value != checksum:
            raise RuntimeError("workspace variants produced different output")
        # Alternate order at the caller level; this method resets state per
        # round so each repetition has the same synthetic sequence.
    return {
        "median_ns_per_frame": statistics.median(samples),
        "rounds_ns_per_frame": samples,
        "checksum": checksum,
    }


def _ticks_diff(newer, older):
    return newer - older


def _controller_samples(count):
    result = []
    for index in range(count):
        sign = -1 if ((index // 100) & 1) else 1
        position = sign * (70 + (index * 37 % 250))
        candidate = sign * (180 + (index * 53 % 420))
        capture_ms = index * 20
        result.append((capture_ms, 20 if index else None,
                       position, candidate))
    return result


def _time_control_profile(profile, samples, rounds=7):
    timings = []
    pd_final_abs = 0
    preview_final_abs = 0
    for _round_index in range(rounds):
        pd_controller = main._FastPositionPD(config)
        preview_controller = main._FastAdaptivePreview(config)
        checksum = 0
        start = time.perf_counter_ns()
        for capture_ms, dt_ms, position, candidate in samples:
            info = pd_controller.update(
                capture_ms, dt_ms, True, position,
                "road", "OBSERVED_BOTH", "ROAD_REFERENCE", 40,
                profile, _ticks_diff)
            applied = preview_controller.update(
                capture_ms, dt_ms, True, profile == "pd_preview",
                "road", "OBSERVED_BOTH", "ROAD_REFERENCE", 40,
                True, candidate, position, _ticks_diff, info)
            checksum += info["pd_output"] + applied
        elapsed = time.perf_counter_ns() - start
        timings.append(elapsed / float(len(samples)))
        pd_final_abs = abs(pd_controller.last_derivative)
        preview_final_abs = abs(preview_controller.applied)
    return {
        "median_ns_per_frame": statistics.median(timings),
        "rounds_ns_per_frame": timings,
        "final_derivative_abs": pd_final_abs,
        "final_preview_applied_abs": preview_final_abs,
    }


def main_entry():
    cases = _workspace_cases()
    full = _time_workspace(_FullClearReferenceWorkspace, cases)
    touched = _time_workspace(main._FastRowWeightWorkspace, cases)
    baseline_ns = full["median_ns_per_frame"]
    current_ns = touched["median_ns_per_frame"]
    workspace_report = {
        "synthetic_case_count": len(cases),
        "frame_mix": [
            "dense valid rows from ROI floor to bottom",
            "sparse valid road rows",
            "duplicate/conflicting row centers",
            "no supported rows",
        ],
        "baseline_reference": {
            "behavior": "60-entry clear of _row_weight and _preview_valid_rows on every frame (120 zero stores)",
            "implementation": "test-only reference workspace; current row aggregation, full-clear reset semantics",
            "median_ns_per_frame": full["median_ns_per_frame"],
            "rounds_ns_per_frame": full["rounds_ns_per_frame"],
            "checksum": full["checksum"],
        },
        "touched_row_implementation": {
            "behavior": "clear only unique preview rows recorded as touched; no _row_weight reset because all consumed entries are overwritten before read",
            "median_ns_per_frame": touched["median_ns_per_frame"],
            "rounds_ns_per_frame": touched["rounds_ns_per_frame"],
            "checksum": touched["checksum"],
        },
        "median_change_percent_vs_full_clear": (
            (current_ns - baseline_ns) * 100.0 / baseline_ns
            if baseline_ns else 0.0),
        "estimated_zero_stores_per_frame": {
            "full_clear_reference": 120,
            "touched_row_path": "one per unique touched preview row; max 42 in configured y=18..59 band",
        },
        "semantic_equivalence_test": "test_turn_response_weights.test_touched_row_clear_matches_full_clear_over_mixed_frames",
    }

    controller_input = _controller_samples(1600)
    control_results = {}
    for profile in ("no_pd", "pd", "pd_preview"):
        control_results[profile] = _time_control_profile(
            profile, controller_input)
    no_pd_ns = control_results["no_pd"]["median_ns_per_frame"]
    for profile in ("pd", "pd_preview"):
        control_results[profile]["median_delta_ns_vs_no_pd"] = (
            control_results[profile]["median_ns_per_frame"] - no_pd_ns)
        control_results[profile]["median_delta_percent_vs_no_pd"] = (
            (control_results[profile]["median_ns_per_frame"] - no_pd_ns) *
            100.0 / no_pd_ns if no_pd_ns else 0.0)

    report = {
        "schema_version": 1,
        "scope": "HOST_ONLY_SYNTHETIC_CPYTHON_MICROBENCHMARK",
        "timestamp_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "environment": {
            "python_version": sys.version,
            "platform": platform.platform(),
            "processor": platform.processor(),
            "machine": platform.machine(),
        },
        "source_hashes": {
            "main.py": _sha256(OPENMV_DIR / "main.py"),
            "config.py": _sha256(OPENMV_DIR / "config.py"),
            "benchmark_host_performance.py": _sha256(Path(__file__)),
        },
        "hotspot_optimization": workspace_report,
        "controller_profile_timing": {
            "input_count_per_round": len(controller_input),
            "input": "precomputed filtered position, signed geometry candidate, 20ms capture interval; same sequence for every profile",
            "included": "position PD update plus preview-state update",
            "excluded": [
                "image acquisition and downsampling",
                "thresholding, path and boundary scans",
                "preview candidate geometry extraction",
                "sensor/Viper frame loop, overlays, UART and garbage collection",
            ],
            "profiles": control_results,
        },
        "scope_findings": {
            "actual_existing_hotspot_reduced": "per-frame clearing of the fixed 60-row workspace arrays; measured against the preserved pre-SpeedPD ZIP implementation and a full-clear reference",
            "image_processing_hotspots_optimized_in_this_change": [],
            "controller_overhead_measured_separately": True,
            "image_resolution_or_safety_gate_reduced": False,
            "safety_semantics_changed_for_speed": False,
            "production_board_45fps_verified": False,
            "board_or_vehicle_runtime_tested": False,
        },
        "interpretation": "CPython timings compare identical synthetic calls on this host. They do not predict OpenMV execution time or establish 45 fps.",
    }
    destination = (PROJECT_DIR / "test-data" /
                   "speedpd-20261020-validation" / "camera" /
                   "host-performance-comparison.json")
    destination.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8", newline="")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main_entry()
