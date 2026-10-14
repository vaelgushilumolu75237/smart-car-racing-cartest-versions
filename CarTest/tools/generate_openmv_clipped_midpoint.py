"""Generate/verify the isolated clipped-midpoint OpenMV source variant."""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "openmv"
TARGET = ROOT / "openmv_clipped_midpoint"
DEFAULT_EVIDENCE = ROOT / "test-data" / "speed-ring-20261014" / "vision"
EVIDENCE = DEFAULT_EVIDENCE
CACHED_BUILD = "CACHED_WIDTH_RING_20261014"
CLIPPED_BUILD = "CLIPPED_MIDPOINT_RING_20261014"
SOURCE_MODE = 'SINGLE_EDGE_ROUTE_MODE = "cached_width"'
TARGET_MODE = 'SINGLE_EDGE_ROUTE_MODE = "clipped_midpoint"'
SOURCE_BUILD = 'BUILD_ID = "%s"' % CACHED_BUILD
TARGET_BUILD = 'BUILD_ID = "%s"' % CLIPPED_BUILD


def _sha256(data):
    return hashlib.sha256(data).hexdigest().upper()


def _runtime_names():
    names = sorted(path.name for path in SOURCE.glob("*.py"))
    if len(names) != 18:
        raise RuntimeError("expected 18 top-level runtime .py files; found %d" %
                           len(names))
    return names


def _clipped_config(data):
    source = data.decode("utf-8")
    if source.count(SOURCE_BUILD) != 1 or source.count(SOURCE_MODE) != 1:
        raise RuntimeError("formal config build/mode markers are unexpected")
    target = source.replace(SOURCE_BUILD, TARGET_BUILD, 1)
    target = target.replace(SOURCE_MODE, TARGET_MODE, 1)
    return target.encode("utf-8")


def _variant_readme():
    return ("# Clipped midpoint OpenMV V7 package\n\n"
            "This folder is an independently uploadable runtime package. "
            "Its build ID is `%s` and its selected single-edge route is "
            "`clipped_midpoint`. The 18 runtime Python files are generated "
            "from the formal `openmv/` package; only `config.py` differs.\n\n"
            "Both routes use the same frame-local turn-response aggregate: "
            "accepted lower 70%% rows carry 80%% of the steering weight, and "
            "accepted upper 30%% rows carry 20%%. The full-frame split is "
            "y=36 (grid y=18). Four threshold blocks start at grid row 8 "
            "(full-image y=16), adding upper observations while retaining the "
            "three original lower blocks. Per-row bilateral support is "
            "required; ambiguous or unsupported rows contribute nothing. If "
            "one group is absent, the available group is renormalized and the "
            "diagnostic names the missing group.\n\n"
            "The aggregate is a virtual controller column; it remains separate "
            "from the validated geometric safety reference. EMA alpha is 0.70. "
            "The weighting changes steering response but does not guarantee "
            "that whole-path selection will never move to another lane.\n\n"
            "Ring handling uses only current supported boundary arcs and a "
            "known connected white-road port/path. Three fresh KEEP frames "
            "confirm entry. The route seed is the nearest supported bilateral "
            "row no more than four grid rows from the last supported row with "
            "known-white and full two-cell "
            "clearance; the selected seed drives the current ring reference and "
            "heading. Partial current inner-edge evidence may retain "
            "INSIDE identity, but without a full current arc/port candidate "
            "the main loop uses its ordinary accepted road/single-edge route "
            "and does not keep ring heading from history. Unknown geometry "
            "cannot synthesize a new route. Exit needs side-specific outward "
            "near-crop inner-edge movement plus current bilateral road and "
            "forward continuation for three consecutive frames; inward jumps "
            "or unknown edges do not confirm exit. A reachable port may approach to the "
            "configured 5%% viewport inset (4 cells on this 80-cell grid); "
            "the port tolerance does not relax mask, clearance, arc, or "
            "uniqueness checks and is unrelated to steering weights. "
            "Slope classification is disabled in this "
            "build and every emitted slope hint remains zero; the optional "
            "protocol bit is retained for compatibility.\n\n"
            "The single-edge route requires a fresh reverse RESET/KEEP lease, "
            "a current dark-to-white edge, and a known white-road corridor to "
            "the opposite frame endpoint. It does not read or update "
            "width-cache data. The frame endpoint is shown as clipped "
            "geometry and is never reported as a measured boundary. A stale "
            "lease disables lease-dependent single-edge recovery and ring "
            "takeover; it does not invalidate an independently accepted "
            "current bilateral road observation.\n\n"
            "V7 draws current-frame valid geometry after detection and UART "
            "transmission, then flushes that frame to the IDE preview. The "
            "display samples at most 13 row points, including both endpoints, "
            "for no more than 12 connecting segments; clipped edge markers "
            "use the same sampled rows. Current validity is drawn every frame; "
            "compact detail strings refresh every 100 ms. The `overlay` timing "
            "includes detail-cache formatting and drawing; `preview_flush` "
            "remains separate from capture-to-send age. An invalid frame has "
            "no current route geometry.\n\n"
            "Upload all 18 top-level `.py` files from this folder together. "
            "Do not mix them with `openmv/` files. See `../openmv/README.md` "
            "for shared protocol, test, and validation boundaries.\n" %
            CLIPPED_BUILD).encode("utf-8")


def _route_metadata(route):
    return {
        "mode": route,
        "build_id": CACHED_BUILD if route == "cached_width" else CLIPPED_BUILD,
        "singleedge_runtime_api": 2,
        "requirements": (
            [
                "current-frame dark-to-white step on the observed side",
                "known white road corridor continues to the opposite frame endpoint",
                "existing support, span, continuity and ambiguity checks pass",
                "fresh reverse RESET/KEEP lease remains valid",
                "island, branch or ambiguous topology is rejected",
                "clipped endpoint is never reported as an observed boundary",
                "no width-cache data is read or written",
            ] if route == "clipped_midpoint" else [
                "current-frame dark-to-white observed side is matched to a validated bilateral width cache",
                "width cache is populated only by a valid published bilateral observation",
                "fresh reverse RESET/KEEP lease remains valid",
            ]),
        "center_equation": (
            "left: (current left edge + image right endpoint) / 2; "
            "right: (current right edge + image left endpoint) / 2"
            if route == "clipped_midpoint" else
            "current edge plus/minus the smoothed bilateral width"),
        "turn_response": {
            "version": "LOWER70_UPPER30_V1",
            "image_split_y": 36,
            "grid_split_y": 18,
            "lower_70_percent_weight": 0.80,
            "upper_30_percent_weight": 0.20,
            "row_density_normalized_within_each_group": True,
            "per_row_bilateral_pair_support_required": True,
            "missing_group_renormalization": True,
            "observation_filter_alpha": 0.70,
            "safety_reference_is_separate_from_virtual_weighted_center": True,
        },
        "ring_and_slope": {
            "runtime_module": "fast_ring_slope.py",
            "runtime_api_version": 1,
            "ring_requires_current_supported_arc_and_connected_white_port": True,
            "ring_seed_uses_nearest_safe_supported_row_within_four_grid_rows": True,
            "ring_port_inset_margin_cells": 4,
            "ring_port_inset_margin_percent_of_viewport": 5,
            "ring_port_margin_is_steering_weight": False,
            "ring_port_retains_known_white_two_cell_clearance": True,
            "ring_entry_fresh_keep_frames": 3,
            "ring_exit_requires_current_inner_edge_disappearance_and_forward_bilateral_frames": 3,
            "slope_classifier_runtime_enabled": False,
            "slope_hint_always_zero": True,
            "slope_hint_flag_bit": 1,
            "optional_slope_protocol_bit_retained": True,
        },
    }


def _build_manifest(base, folder, route):
    manifest = copy.deepcopy(base)
    build_id = CACHED_BUILD if route == "cached_width" else CLIPPED_BUILD
    manifest["schema_version"] = 3
    manifest["package_id"] = build_id
    manifest["build_id"] = build_id
    manifest["route_mode"] = route
    manifest["release_status"] = "HOST_TESTED_OPENMV_BOARD_VALIDATION_PENDING"
    manifest.setdefault("runtime_api", {})["singleedge"] = 2
    old_entries = {item["name"]: item
                   for item in manifest.get("runtime_files", [])}
    entries = []
    for name in _runtime_names():
        data = (folder / name).read_bytes()
        entry = copy.deepcopy(old_entries.get(name, {}))
        entry.update({"name": name,
                      "size_bytes": len(data),
                      "sha256": _sha256(data)})
        entries.append(entry)
    manifest["runtime_file_count"] = len(entries)
    manifest["runtime_files"] = entries
    manifest["changed_runtime_files"] = [
        name for name in ("config.py", "main.py", "fast_gray.py",
                          "fast_path.py", "fast_boundary.py",
                          "fast_single_edge.py", "fast_ring_slope.py",
                          "vision_protocol.py")
        if name in _runtime_names()]
    manifest["single_edge_routes"] = {
        "default_formal_route": "cached_width",
        "supported_build_pairs": [
            {"route_mode": "cached_width", "build_id": CACHED_BUILD},
            {"route_mode": "clipped_midpoint", "build_id": CLIPPED_BUILD},
        ],
        "selected_package": _route_metadata(route),
        "same_source_except_config": route == "clipped_midpoint",
        "protocol_unchanged": {
            "observation_frame_bytes": 12,
            "observation_protocol_version": 1,
            "reverse_control_timeout_ms": 250,
            "reverse_control_api_version": 1,
        },
    }
    if route == "clipped_midpoint":
        cache = copy.deepcopy(manifest.get("singleedge_cache_model", {}))
        cache["no_cache_behavior"] = (
            "Width cache is not consulted. A single edge is valid only with "
            "the current-frame white corridor and fresh RESET/KEEP lease.")
        manifest["singleedge_cache_model"] = cache
    report_path = EVIDENCE / "REPORT.json"
    if report_path.is_file():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        manifest["two_route_evidence"] = report
        manifest["host_test_results"] = report.get(
            "host_tests", manifest.get("host_test_results", []))
        compiler = report.get("mpy_cross", {})
        variants = compiler.get("variants", [])
        manifest["host_mpy_cross_validation"] = {
            "compiler": compiler.get("compiler"),
            "compiler_path": compiler.get("compiler_path"),
            "target_architecture": compiler.get("target_architecture"),
            "runtime_sources_per_variant": 18,
            "variants": variants,
            "all_variants_exit_zero": bool(variants) and all(
                item.get("exit_code") == 0 for item in variants),
            "target_micro_python_v1_28_compiled": compiler.get(
                "target_micro_python_v1_28_compiled", False),
            "micro_python_v1_28_compiler_available_in_ide": compiler.get(
                "micro_python_v1_28_compiler_available_in_ide", False),
            "target_board_runtime_tested": False,
            "evidence_report": "../" +
                report_path.relative_to(ROOT).as_posix(),
            "evidence_report_sha256": _sha256(report_path.read_bytes()),
        }
    readme = folder / "README.md"
    if readme.is_file():
        manifest["readme"] = "README.md"
        manifest["readme_sha256"] = _sha256(readme.read_bytes())
    return manifest


def generate():
    if TARGET.exists():
        raise RuntimeError("target already exists; preserving it: %s" % TARGET)
    base = json.loads((SOURCE / "PACKAGE_MANIFEST.json").read_text(
        encoding="utf-8"))
    TARGET.mkdir(parents=False)
    try:
        for name in _runtime_names():
            data = (SOURCE / name).read_bytes()
            if name == "config.py":
                data = _clipped_config(data)
            (TARGET / name).write_bytes(data)
        (TARGET / "README.md").write_bytes(_variant_readme())
        manifest = _build_manifest(base, TARGET, "clipped_midpoint")
        (TARGET / "PACKAGE_MANIFEST.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
    except Exception:
        # Leave partial output for inspection; never delete or overwrite it.
        raise
    return refresh_manifests()


def _verify_existing_target_before_refresh():
    if not TARGET.is_dir():
        raise RuntimeError("target directory is missing: %s" % TARGET)
    manifest_path = TARGET / "PACKAGE_MANIFEST.json"
    if not manifest_path.is_file():
        raise RuntimeError("target manifest is missing; preserving target")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = {item.get("name"): item
               for item in manifest.get("runtime_files", [])}
    existing_names = sorted(path.name for path in TARGET.glob("*.py"))
    if existing_names != sorted(entries) or not set(existing_names).issubset(
            set(_runtime_names())):
        raise RuntimeError(
            "target sources differ from its recorded inventory; preserving target")
    stale_manifest_entries = 0
    for name in existing_names:
        path = TARGET / name
        entry = entries[name]
        data = path.read_bytes()
        recorded_match = (entry.get("sha256") == _sha256(data) and
                          entry.get("size_bytes") == len(data))
        expected = (SOURCE / name).read_bytes()
        if name == "config.py":
            expected = _clipped_config(expected)
        if not recorded_match and data != expected:
            raise RuntimeError(
                "target runtime differs from its manifest and formal source; "
                "preserving %s" %
                name)
        if not recorded_match:
            stale_manifest_entries += 1
    return stale_manifest_entries


def refresh_variant():
    """Refresh generated clipped sources only after old manifest verification."""
    if not TARGET.exists():
        return generate()
    stale_manifest_entries = _verify_existing_target_before_refresh()
    clipped_base = json.loads((TARGET / "PACKAGE_MANIFEST.json").read_text(
        encoding="utf-8"))
    for name in _runtime_names():
        data = (SOURCE / name).read_bytes()
        if name == "config.py":
            data = _clipped_config(data)
        (TARGET / name).write_bytes(data)
    (TARGET / "README.md").write_bytes(_variant_readme())
    clipped = _build_manifest(clipped_base, TARGET, "clipped_midpoint")
    (TARGET / "PACKAGE_MANIFEST.json").write_text(
        json.dumps(clipped, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    refreshed = refresh_manifests()
    print("REFRESHED clipped variant from paired source; stale prior manifest "
          "runtime entries=%d; preserved prior snapshot remains authoritative" %
          stale_manifest_entries)
    return refreshed


def refresh_manifests():
    base = json.loads((SOURCE / "PACKAGE_MANIFEST.json").read_text(
        encoding="utf-8"))
    formal = _build_manifest(base, SOURCE, "cached_width")
    (SOURCE / "PACKAGE_MANIFEST.json").write_text(
        json.dumps(formal, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    if TARGET.exists():
        clipped_base = json.loads((TARGET / "PACKAGE_MANIFEST.json").read_text(
            encoding="utf-8"))
        clipped = _build_manifest(clipped_base, TARGET, "clipped_midpoint")
        (TARGET / "PACKAGE_MANIFEST.json").write_text(
            json.dumps(clipped, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
    return verify()


def verify():
    if not TARGET.is_dir():
        raise RuntimeError("target directory is missing: %s" % TARGET)
    names = _runtime_names()
    source_config = (SOURCE / "config.py").read_text(encoding="utf-8")
    if (source_config.count(SOURCE_BUILD) != 1 or
            source_config.count(SOURCE_MODE) != 1):
        raise RuntimeError("formal config build/mode markers are unexpected")
    target_py = sorted(path.name for path in TARGET.glob("*.py"))
    if target_py != names:
        raise RuntimeError("target .py inventory differs from the 18-file source")
    expected_config = _clipped_config((SOURCE / "config.py").read_bytes())
    if (TARGET / "config.py").read_bytes() != expected_config:
        raise RuntimeError("target config is not the exact clipped route variant")
    different = []
    for name in names:
        if name == "config.py":
            continue
        if (SOURCE / name).read_bytes() != (TARGET / name).read_bytes():
            different.append(name)
    if different:
        raise RuntimeError("non-config runtime source differs: %s" %
                           ", ".join(different))
    manifest = json.loads((TARGET / "PACKAGE_MANIFEST.json").read_text(
        encoding="utf-8"))
    if (manifest.get("build_id") != CLIPPED_BUILD or
            manifest.get("route_mode") != "clipped_midpoint"):
        raise RuntimeError("target package manifest route/build mismatch")
    entries = {item["name"]: item for item in manifest.get("runtime_files", [])}
    for name in names:
        data = (TARGET / name).read_bytes()
        entry = entries.get(name)
        if (entry is None or entry.get("sha256") != _sha256(data) or
                entry.get("size_bytes") != len(data)):
            raise RuntimeError("target manifest hash/size mismatch for %s" % name)
    for folder, route, build in ((SOURCE, "cached_width", CACHED_BUILD),
                                 (TARGET, "clipped_midpoint", CLIPPED_BUILD)):
        package = json.loads((folder / "PACKAGE_MANIFEST.json").read_text(
            encoding="utf-8"))
        if (package.get("route_mode") != route or
                package.get("build_id") != build):
            raise RuntimeError("package route/build mismatch in %s" % folder)
        route_info = package.get("single_edge_routes") or {}
        expected_pairs = [
            {"route_mode": "cached_width", "build_id": CACHED_BUILD},
            {"route_mode": "clipped_midpoint", "build_id": CLIPPED_BUILD},
        ]
        if route_info.get("supported_build_pairs") != expected_pairs:
            raise RuntimeError("route/build pair allowlist mismatch in %s" %
                               folder)
        entries = {item["name"]: item
                   for item in package.get("runtime_files", [])}
        if package.get("runtime_file_count") != len(names):
            raise RuntimeError("runtime file count mismatch in %s" % folder)
        for name in names:
            data = (folder / name).read_bytes()
            entry = entries.get(name)
            if (entry is None or entry.get("sha256") != _sha256(data) or
                    entry.get("size_bytes") != len(data)):
                raise RuntimeError(
                    "%s manifest hash/size mismatch for %s" %
                    (route, name))
        readme = folder / "README.md"
        if not readme.is_file():
            raise RuntimeError("package README is missing: %s" % folder)
        if package.get("readme_sha256") != _sha256(readme.read_bytes()):
            raise RuntimeError("package README hash mismatch in %s" % folder)
    print("VERIFIED runtime_py=18 config_different=1 other_runtime_py_different=0")
    for name in names:
        data = (TARGET / name).read_bytes()
        print("%s %s %d" % (name, _sha256(data), len(data)))
    return True


def main_cli():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_EVIDENCE,
        help="project-local report directory (default: speed-ring-20261014/vision)")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true",
                       help="verify existing files without modifying them")
    group.add_argument("--refresh-manifests", action="store_true",
                       help="refresh hashes and route evidence in both manifests")
    group.add_argument("--refresh-variant", action="store_true",
                       help="manifest-guard and refresh the clipped runtime sources")
    args = parser.parse_args()
    try:
        global EVIDENCE
        output = args.output_dir
        if not output.is_absolute():
            output = ROOT / output
        EVIDENCE = output.resolve()
        EVIDENCE.relative_to(ROOT.resolve())
        if args.check:
            verify()
        elif args.refresh_manifests:
            refresh_manifests()
        elif args.refresh_variant:
            refresh_variant()
        else:
            generate()
    except (OSError, ValueError, RuntimeError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main_cli())
