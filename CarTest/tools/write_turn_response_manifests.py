"""Record the eight current CarTest ELFs and preserve the seven prior ELFs."""

from __future__ import annotations

import hashlib
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import runtime_log_export as exporter  # noqa: E402


PRESETS = [
    "Debug", "DebugSteering", "DebugBench", "DebugFollow",
    "DebugFollowNoLog", "DebugFollowDiff5", "DebugFollowDiff10",
    "DebugFollowTurn",
]
FOLLOW_PARAMETERS = {
    "DebugFollow": (0, 0, 600, 10, 40),
    "DebugFollowNoLog": (0, 0, 600, 10, 40),
    "DebugFollowDiff5": (50, 50, 600, 10, 40),
    "DebugFollowDiff10": (100, 100, 600, 10, 40),
    "DebugFollowTurn": (200, 200, 700, 20, 60),
}
OLD_ANALYZER_VARIANTS = {
    "none": "DebugFollow",
    "medium": "DebugFollowDiff5",
    "strong": "DebugFollowDiff10",
}
OLD_PRESETS = ["Debug", "DebugSteering", "DebugBench", *OLD_ANALYZER_VARIANTS.values(),
               "DebugFollowNoLog"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_cache(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        raw_key, rest = line.split("=", 1)
        key = raw_key.split(":", 1)[0]
        values[key] = rest.split(":", 1)[0]
    return values


def role_for(preset: str) -> str:
    return {
        "Debug": "actuators_locked",
        "DebugSteering": "motor_locked_steering_diagnostic",
        "DebugBench": "motor_locked_bench_diagnostic",
        "DebugFollow": "legacy_follow_straight_50",
        "DebugFollowNoLog": "legacy_follow_without_ram_logging",
        "DebugFollowDiff5": "legacy_follow_differential_50",
        "DebugFollowDiff10": "legacy_follow_differential_100",
        "DebugFollowTurn": "new_follow_turn_response_200",
    }[preset]


def profile_parameters(preset: str) -> dict[str, object] | None:
    if preset not in FOLLOW_PARAMETERS:
        return None
    outer, inner, normal_cap, motor_step, servo_step = FOLLOW_PARAMETERS[preset]
    return {
        "straight_target_permille": 500,
        "outer_differential_permille": outer,
        "inner_differential_permille": inner,
        "full_turn_left_right_target_permille": [500 + outer, 500 - inner],
        "normal_cap_permille": normal_cap,
        "hard_cap_permille": 700,
        "startup_motor_step_permille_per_10ms": 10,
        "normal_motor_step_permille_per_10ms": motor_step,
        "startup_servo_step_us_per_20ms": 10,
        "normal_servo_step_us_per_20ms": servo_step,
        "steering_gain_permille": 700,
    }


def save_original_path_forms(
    elf_manifest: dict[str, object], artifacts: dict[str, object], snapshot: Path
) -> list[str]:
    """Archive the pre-repoint relative-path form as provenance, not as a live manifest."""
    archive = snapshot / "historical-path-form-manifests"
    archive.mkdir(parents=True, exist_ok=True)

    elf_copy = json.loads(json.dumps(elf_manifest))
    for profile in elf_copy.get("profiles", []):
        preset = profile.get("preset")
        profile["elf_path"] = f"../../../build/{preset}/CarTest.elf"
    elf_copy_path = archive / "ELF_MANIFEST.original-paths.json"
    elf_copy_path.write_text(json.dumps(elf_copy, indent=2) + "\n", encoding="utf-8")

    artifact_copy = json.loads(json.dumps(artifacts))
    for variant, preset in OLD_ANALYZER_VARIANTS.items():
        artifact_copy["profiles"][variant]["elf_path"] = (
            f"../../../build/{preset}/CarTest.elf"
        )
    artifact_copy_path = archive / "ARTIFACTS.original-paths.json"
    artifact_copy_path.write_text(
        json.dumps(artifact_copy, indent=2) + "\n", encoding="utf-8"
    )
    return [
        str(elf_copy_path.relative_to(ROOT)).replace(os.sep, "/"),
        str(artifact_copy_path.relative_to(ROOT)).replace(os.sep, "/"),
    ]


def current_profiles(output: Path) -> list[dict[str, object]]:
    profiles: list[dict[str, object]] = []
    for preset in PRESETS:
        elf = ROOT / "build" / preset / "CarTest.elf"
        cache = read_cache(ROOT / "build" / preset / "CMakeCache.txt")
        exit_path = output / "buildlogs" / f"{preset}.log.exitcode"
        if not elf.is_file() or not exit_path.is_file():
            raise RuntimeError(f"Missing current build output or exit record for {preset}")
        exit_code = int(exit_path.read_text(encoding="ascii").strip())
        if exit_code != 0:
            raise RuntimeError(f"Build {preset} returned exit code {exit_code}")
        layout = exporter.inspect_elf(elf)
        flags = {
            key: cache[key]
            for key in (
                "CARTEST_ENABLE_BENCH", "CARTEST_ENABLE_FOLLOW",
                "CARTEST_ENABLE_FOLLOW_TURN", "CARTEST_ENABLE_STEERING",
                "CARTEST_HARDWARE_ACCEPTED", "CARTEST_RUNTIME_LOG_ENABLED",
                "CARTEST_FOLLOW_DIFF_PERMILLE",
            )
            if key in cache
        }
        item: dict[str, object] = {
            "preset": preset,
            "role": role_for(preset),
            "exitcode": exit_code,
            "build_log": f"buildlogs/{preset}.log",
            "build_exitcode_path": f"buildlogs/{preset}.log.exitcode",
            "elf_path": f"../../../build/{preset}/CarTest.elf",
            "elf_sha256": layout.sha256,
            "build_id": layout.build_id.hex(),
            "runtime_log_storage_address": f"0x{layout.storage_address:08x}",
            "runtime_log_storage_size": layout.storage_size,
            "cache_flags": flags,
        }
        parameters = profile_parameters(preset)
        if parameters is not None:
            item["profile_parameters"] = parameters
        profiles.append(item)
    return profiles


def preserve_old_manifests(snapshot: Path) -> list[str]:
    historical = ROOT / "test-data" / "differential-20261013" / "control"
    baseline_elfs = snapshot / "elfs"
    if not historical.is_dir() or not baseline_elfs.is_dir():
        raise RuntimeError("Historical control manifests or baseline ELF snapshots are missing")

    elf_manifest_path = historical / "ELF_MANIFEST.json"
    elf_manifest = json.loads(elf_manifest_path.read_text(encoding="utf-8"))
    artifacts_path = historical / "ARTIFACTS.json"
    artifacts = json.loads(artifacts_path.read_text(encoding="utf-8"))
    archived_path_forms = save_original_path_forms(elf_manifest, artifacts, snapshot)
    for profile in elf_manifest.get("profiles", []):
        preset = profile.get("preset")
        if preset not in OLD_PRESETS:
            raise RuntimeError(f"Unexpected historical preset in ELF manifest: {preset}")
        baseline = baseline_elfs / preset / "CarTest.elf"
        layout = exporter.inspect_elf(baseline)
        if (layout.sha256.lower() != profile.get("elf_sha256", "").lower() or
                layout.build_id.hex().lower() != profile.get("build_id", "").lower()):
            raise RuntimeError(f"Baseline snapshot does not match historical manifest for {preset}")
        profile["elf_path"] = Path(os.path.relpath(baseline, historical)).as_posix()
    elf_manifest_path.write_text(json.dumps(elf_manifest, indent=2) + "\n", encoding="utf-8")

    for variant, preset in OLD_ANALYZER_VARIANTS.items():
        profile = artifacts.get("profiles", {}).get(variant)
        if not isinstance(profile, dict):
            raise RuntimeError(f"Historical ARTIFACTS.json is missing {variant}")
        baseline = baseline_elfs / preset / "CarTest.elf"
        layout = exporter.inspect_elf(baseline)
        if (layout.sha256.lower() != profile.get("elf_sha256", "").lower() or
                layout.build_id.hex().lower() != profile.get("build_id", "").lower()):
            raise RuntimeError(f"Baseline snapshot does not match historical analyzer artifact {variant}")
        profile["elf_path"] = Path(os.path.relpath(baseline, historical)).as_posix()
    artifacts_path.write_text(json.dumps(artifacts, indent=2) + "\n", encoding="utf-8")
    return archived_path_forms


def main() -> int:
    output = ROOT / "test-data" / "turn-response-20261014" / "control"
    output.mkdir(parents=True, exist_ok=True)
    snapshot = ROOT / "test-data" / "turn-response-20261014" / "before" / "control"
    archived_path_forms = preserve_old_manifests(snapshot)
    profiles = current_profiles(output)
    build_ids = [str(profile["build_id"]) for profile in profiles]
    elf_hashes = [str(profile["elf_sha256"]) for profile in profiles]
    if len(set(build_ids)) != len(build_ids) or len(set(elf_hashes)) != len(elf_hashes):
        raise RuntimeError("Current profile ELF hashes and ROM build IDs must be unique")

    toolchain = {
        "arm_gcc": "GNU Tools for STM32 14.3.1 20250623",
        "cmake": "4.3.1",
        "ninja": "1.13.2",
    }
    manifest = {
        "schema_version": 1,
        "artifact_set": "turn-response-20261014",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "toolchain": toolchain,
        "profiles": profiles,
    }
    (output / "ELF_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    compatibility = {
        "legacy_best_of_three_analyzer": "unsupported for this eight-profile manifest; DebugFollowTurn d=200 must use a dedicated turn-response analyzer",
        "generic_64_record_export": "supported; RAM record layout is unchanged",
    }
    artifact_profiles = {
        str(profile["preset"]): {
            "variant": str(profile["preset"]),
            "diff_permille": int(profile.get("profile_parameters", {}).get(
                "outer_differential_permille", 0)),
            "elf_path": profile["elf_path"],
            "elf_sha256": profile["elf_sha256"],
            "build_id": profile["build_id"],
            "servo_center_us": 1530,
            "role": profile["role"],
        }
        for profile in profiles
    }
    artifacts = {
        "schema_version": 1,
        "artifact_set": "turn-response-20261014",
        "analysis_compatibility": compatibility,
        "profiles": artifact_profiles,
    }
    (output / "ARTIFACTS.json").write_text(
        json.dumps(artifacts, indent=2) + "\n", encoding="utf-8")

    preserved = {
        "original_evidence": "test-data/differential-20261013/control",
        "baseline_snapshot": "test-data/turn-response-20261014/before/control/elfs",
        "reconstructed_original_path_form_copies": archived_path_forms,
        "manifests_repointed_to_verified_byte_identical_baseline_copies": [
            "test-data/differential-20261013/control/ELF_MANIFEST.json",
            "test-data/differential-20261013/control/ARTIFACTS.json",
        ],
        "policy": "The archived path-form copies reconstruct the prior relative ../../../build paths while preserving recorded identity metadata; they are provenance copies and do not resolve from their archive location. In the active historical manifests, only elf_path values were changed so recorded hashes and ROM build IDs resolve to byte-identical baseline ELFs. JSON formatting was normalized. Historical trial results were not regenerated.",
    }
    (snapshot / "PRESERVED_HISTORICAL_ARTIFACTS.json").write_text(
        json.dumps(preserved, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
