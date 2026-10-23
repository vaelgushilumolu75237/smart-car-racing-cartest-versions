"""Save a source/ELF checkpoint and optionally publish it to the owner's GitHub.

Credentials are obtained from Git Credential Manager in memory and never logged.
The checkpoint worktree is separate from the live CarTest source directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import urllib.error
import urllib.request

PROJECT = Path(__file__).resolve().parents[1]
STORE = PROJECT / "deliverables" / "github-checkpoints" / "worktree"
REPO_NAME = "smart-car-racing-cartest-versions"
DIRS = ("App", "Inc", "Src", "Drivers", "cmake", "tools", "tests", "docs", ".vscode", "openmv")
ROOT_FILES = ("CMakeLists.txt", "CMakePresets.json", "CarTest.code-workspace",
              "project-description.json", "stm32h743xi_flash.ld", "README.md", "AGENTS.md",
              "README-恢复说明.md", "origin-manifest.json", "SHA256SUMS.txt")
SKIP_DIRS = {"__pycache__", ".git", "logs", "tmp", "node_modules"}
SKIP_SUFFIXES = {".pyc", ".obj", ".o", ".exe", ".dll", ".png", ".jpg",
                 ".jpeg", ".mp4", ".avi", ".bin", ".log"}
# Only these synthetic/source fixtures are public; never copy the vehicle-data
# tree wholesale. They are required by the shipped host regression commands.
PUBLIC_TEST_FIXTURES = (
    "test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.manifest.json",
    "test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.zip",
    "test-data/runtime-log-ab-20261012/baseline/Src__main.c",
)
UPHILL_A_BASELINE_SHA256 = "667c95a7aef79b2630486faf7421a87e067ee582439dbcfad166d2203222718d"
UPHILL_FROZEN_PROFILE_SHA256 = {
    "B": "f50336905ea5a072f687a2a0edaa74412f115ed5e21f40ced85c0dd378e0866c",
    "C": "3c7da3c26054764bec0949753212272d37857c015d5f351d13b4a0b085755d4f",
}
# The public checkpoint carries only these explicit H743 profile artifacts,
# their summaries, and named pair/evidence documents. Do not copy the broader
# deliverables tree or build logs.
UPHILL_PROFILE_DELIVERIES = {
    "B": {
        "profile": "DebugFollowTurn65",
        "elf": "deliverables/H743_Uphill_20261019/B_65_35/CarTest.elf",
        "manifest": "deliverables/H743_Uphill_20261019/B_65_35/manifest.json",
    },
    "C": {
        "profile": "DebugFollowIncrementalPI",
        "elf": "deliverables/H743_Uphill_20261019/C_IncrementalPI/CarTest.elf",
        "manifest": "deliverables/H743_Uphill_20261019/C_IncrementalPI/manifest.json",
    },
}
UPHILL_ARTIFACT_MANIFEST = "deliverables/H743_Uphill_20261019/artifact-manifest.json"
UPHILL_BUILD_SUMMARY = "deliverables/H743_Uphill_20261019/build-summary.json"
SPEEDPD_PROFILE_DELIVERIES = {
    "B": {
        "profile": "DebugFollowTurn65",
        "elf": "deliverables/H743_SpeedPD_20261020/B_OpenLoop/CarTest.elf",
        "manifest": "deliverables/H743_SpeedPD_20261020/B_OpenLoop/manifest.json",
    },
    "C": {
        "profile": "DebugFollowIncrementalPI",
        "elf": "deliverables/H743_SpeedPD_20261020/C_IncrementalPI/CarTest.elf",
        "manifest": "deliverables/H743_SpeedPD_20261020/C_IncrementalPI/manifest.json",
    },
}
SPEEDPD_ARTIFACT_MANIFEST = "deliverables/H743_SpeedPD_20261020/artifact-manifest.json"
SPEEDPD_BUILD_SUMMARY = "deliverables/H743_SpeedPD_20261020/build-summary.json"
OPENMV_SPEEDPD_ROOT = "deliverables/OpenMV_Bottom80_Steering_20261021"
OPENMV_SPEEDPD_PACKAGES = ("no_pd", "pd", "pd_preview")
ACKERMANN_DELIVERY_ROOT = "deliverables/H743_Ackermann60_20261022"
ACKERMANN_PROFILE_DELIVERIES = {
    "coordinated": {
        "profile": "DebugFollowIncrementalPI",
        "directory": "C_Coordinated",
        "elf": f"{ACKERMANN_DELIVERY_ROOT}/C_Coordinated/CarTest.elf",
        "manifest": f"{ACKERMANN_DELIVERY_ROOT}/C_Coordinated/manifest.json",
        "coordination_enabled": True,
        "servo_slew_step_us": 160,
    },
    "baseline": {
        "profile": "DebugFollowIncrementalPIBaseline",
        "directory": "C_Baseline",
        "elf": f"{ACKERMANN_DELIVERY_ROOT}/C_Baseline/CarTest.elf",
        "manifest": f"{ACKERMANN_DELIVERY_ROOT}/C_Baseline/manifest.json",
        "coordination_enabled": False,
        "servo_slew_step_us": 120,
    },
}
ACKERMANN_ARTIFACT_MANIFEST = f"{ACKERMANN_DELIVERY_ROOT}/artifact-manifest.json"
ACKERMANN_BUILD_SUMMARY = f"{ACKERMANN_DELIVERY_ROOT}/build-summary.json"
ACKERMANN_PAIRED_DELIVERY_DOC = f"{ACKERMANN_DELIVERY_ROOT}/PAIRED_DELIVERY.md"
ACKERMANN_PAIRED_RUN_STEPS = f"{ACKERMANN_DELIVERY_ROOT}/validation/PAIRED_RUN_STEPS.md"
ACKERMANN_TOOLING_EVIDENCE = (
    f"{ACKERMANN_DELIVERY_ROOT}/validation/protocol-tooling/TEST_EVIDENCE.md")
OPENMV_ACKERMANN_ROOT = "deliverables/OpenMV_Ackermann320_20261022"
OPENMV_ACKERMANN_PACKAGES = ("geometry_perf_only", "coordinated")
OPENMV_ACKERMANN_RUNTIME_FILES = frozenset({
    "adaptive_detector.py", "boundary_evidence.py", "config.py",
    "contour_route.py", "contour_trace.py", "fast_boundary.py", "fast_gray.py",
    "fast_path.py", "fast_single_edge.py", "fast_topology.py",
    "grayscale_adapter.py", "line_detector.py", "main.py", "reverse_control.py",
    "road_detector.py", "source_selector.py", "surface_detector.py",
    "thresholding.py", "vision_protocol.py",
})
OPENMV_NATIVE_PREVIEW_ROOT = "deliverables/OpenMV_NativePreview_20261024"
OPENMV_NATIVE_PREVIEW_REQUIRED_FILES = tuple(
    f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}" for name in (
        "firmware.bin", "firmware.elf", "build-summary.txt",
        "build-summary.txt.sha256", "artifact-manifest.json",
    )
)
# These two named review documents may be added by the release owner. No other
# candidate-folder files (including the CI zip and logs) enter the checkpoint.
OPENMV_NATIVE_PREVIEW_OPTIONAL_DOCUMENTS = tuple(
    f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}" for name in ("README.md", "EVIDENCE.md")
)
OPENMV_NATIVE_PREVIEW_PATCH = (
    "tools/vendor-patches/openmv-v5.0.0/0001-csi-snapshot-preview-kwarg.patch")
OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256 = (
    "2bbb92b2b25705002ae16b9e9f18f57fc171ca8f0549680cc15260cce0b468d5")
OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256 = {
    "build-summary.txt": "5f448297965d25ec76807ca3e3404ca4753c3acf73880c9dd5cf2bae87996dd7",
    "build-summary.txt.sha256": "4b7467ab38a26d14ffc08c7b94cc6f90465ba8133ff524d2487d63e2f7343876",
    "firmware.bin": "9689d686881d2d358694f580256d0698f01226cd018c6f6951d1d7abf5c663b7",
    "firmware.elf": "dafaec93152163769e81e0f86fcd9b280b289f4e6a49a87d1c29ac1fd559caaa",
}
OPENMV_NATIVE_PREVIEW_EXPECTED_BUILD = {
    "openmv_tag": "v5.0.0",
    "openmv_commit": "7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79",
    "openmv_sdk_version": "1.6.0",
    "patch_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256,
    "firmware.bin_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["firmware.bin"],
    "firmware.elf_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["firmware.elf"],
}
OPENMV_NATIVE_PREVIEW_RUN_ID = 40003611704
OPENMV_NATIVE_PREVIEW_BIN_BYTES = 1_736_532
OPENMV_NATIVE_PREVIEW_ELF_BYTES = 11_322_036
OPENMV_NATIVE_PREVIEW_VECTOR_SP = 0x00010000
OPENMV_NATIVE_PREVIEW_VECTOR_RESET = 0x08040299
PUBLIC_DELIVERY_FILES = (
    "deliverables/H743_Uphill_20261019/A_Baseline/CarTest.elf",
    "deliverables/H743_Uphill_20261019/A_Baseline/manifest.json",
    UPHILL_PROFILE_DELIVERIES["B"]["elf"],
    UPHILL_PROFILE_DELIVERIES["B"]["manifest"],
    UPHILL_PROFILE_DELIVERIES["C"]["elf"],
    UPHILL_PROFILE_DELIVERIES["C"]["manifest"],
    UPHILL_ARTIFACT_MANIFEST,
    UPHILL_BUILD_SUMMARY,
    SPEEDPD_PROFILE_DELIVERIES["B"]["elf"],
    SPEEDPD_PROFILE_DELIVERIES["B"]["manifest"],
    SPEEDPD_PROFILE_DELIVERIES["C"]["elf"],
    SPEEDPD_PROFILE_DELIVERIES["C"]["manifest"],
    SPEEDPD_ARTIFACT_MANIFEST,
    SPEEDPD_BUILD_SUMMARY,
    ACKERMANN_PROFILE_DELIVERIES["coordinated"]["elf"],
    ACKERMANN_PROFILE_DELIVERIES["coordinated"]["manifest"],
    ACKERMANN_PROFILE_DELIVERIES["baseline"]["elf"],
    ACKERMANN_PROFILE_DELIVERIES["baseline"]["manifest"],
    ACKERMANN_ARTIFACT_MANIFEST,
    ACKERMANN_BUILD_SUMMARY,
    ACKERMANN_PAIRED_DELIVERY_DOC,
    ACKERMANN_PAIRED_RUN_STEPS,
    ACKERMANN_TOOLING_EVIDENCE,
)


def run_git(*args: str) -> str:
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "-C", str(STORE), *args], capture_output=True,
                            text=True, encoding="utf-8", env=env, timeout=180)
    if result.returncode:
        raise RuntimeError(f"git {args[0]} failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout.strip()


def api(path: str, token: str, body=None):
    headers = {"Authorization": "Bearer " + token,
               "User-Agent": "CarTest-version-checkpoint",
               "Accept": "application/vnd.github+json"}
    request = urllib.request.Request("https://api.github.com" + path, headers=headers,
                                    data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        if error.code == 404 and body is None:
            return None
        raise RuntimeError(f"GitHub API {path}: HTTP {error.code}") from None


def authenticated_user():
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "credential", "fill"],
                            input="protocol=https\nhost=github.com\n\n", capture_output=True,
                            text=True, env=env, timeout=30)
    if result.returncode:
        raise RuntimeError("Git Credential Manager has no usable noninteractive GitHub credential.")
    credential = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    token = credential.get("password", "")
    user = api("/user", token)
    if not user or not user.get("login"):
        raise RuntimeError("GitHub account verification failed.")
    return user, token


def safe_remove(path: Path):
    resolved = path.resolve()
    if resolved.parent != STORE.resolve() or resolved.name != "CarTest":
        raise RuntimeError("Refusing to clear a directory outside the checkpoint worktree.")
    if resolved.exists():
        shutil.rmtree(resolved)


def copy_tree(source: Path, destination: Path):
    for item in sorted(source.rglob("*")):
        relative = item.relative_to(source)
        if any(part in SKIP_DIRS for part in relative.parts):
            continue
        if item.is_symlink():
            raise RuntimeError(f"Symlink is not allowed in a public source checkpoint: {relative}")
        if item.is_file() and item.suffix.lower() not in SKIP_SUFFIXES:
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(item, target)


def copy_allowlisted_file(source: Path, destination: Path, relative_name: str):
    item = source / relative_name
    if not item.is_file() or item.is_symlink():
        raise RuntimeError(f"Required public delivery file is missing or linked: {relative_name}")
    target = destination / relative_name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(item, target)


def normalize_relative_path(value) -> str | None:
    if not isinstance(value, str):
        return None
    return value.replace("\\", "/")


def index_build_rows(summary: dict) -> dict:
    builds = summary.get("builds", [])
    if isinstance(builds, dict):
        return {str(name): row for name, row in builds.items() if isinstance(row, dict)}
    if isinstance(builds, list):
        return {row.get("preset"): row for row in builds if isinstance(row, dict)}
    raise RuntimeError("Build summary must contain build rows keyed by preset or a list of rows.")


def verify_uphill_deliveries(source: Path):
    a_elf = source / PUBLIC_DELIVERY_FILES[0]
    a_manifest = source / PUBLIC_DELIVERY_FILES[1]
    if not a_elf.is_file() or a_elf.is_symlink():
        raise RuntimeError("The frozen uphill A ELF is missing or linked.")
    if hashlib.sha256(a_elf.read_bytes()).hexdigest() != UPHILL_A_BASELINE_SHA256:
        raise RuntimeError("Frozen uphill A ELF hash mismatch.")
    try:
        parsed_a = json.loads(a_manifest.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Frozen uphill A manifest is missing or invalid.") from error
    if parsed_a.get("sha256") != UPHILL_A_BASELINE_SHA256:
        raise RuntimeError("Frozen uphill A manifest does not match the allowlisted ELF hash.")

    try:
        artifact_manifest = json.loads(
            (source / UPHILL_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / UPHILL_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Uphill artifact manifest or build summary is missing or invalid.") from error

    artifacts = artifact_manifest.get("artifacts", {})
    build_rows = index_build_rows(build_summary)
    for variant, spec in UPHILL_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        delivery_manifest_path = source / spec["manifest"]
        for path, label in ((delivery_elf, "delivery ELF"),
                            (delivery_manifest_path, "delivery manifest")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"Uphill {variant} {label} is missing or linked: {path}")

        delivery_bytes = delivery_elf.read_bytes()
        elf_sha256 = hashlib.sha256(delivery_bytes).hexdigest()
        if elf_sha256 != UPHILL_FROZEN_PROFILE_SHA256[variant]:
            raise RuntimeError(f"Frozen uphill {variant} delivery ELF hash mismatch.")
        try:
            variant_manifest = json.loads(
                delivery_manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"Uphill {variant} manifest is invalid.") from error
        if (variant_manifest.get("profile") != spec["profile"] or
                variant_manifest.get("sha256") != elf_sha256):
            raise RuntimeError(f"Uphill {variant} manifest does not match its ELF/profile.")

        artifact = artifacts.get(variant, {})
        if (normalize_relative_path(artifact.get("delivery_path")) != spec["elf"] or
                artifact.get("sha256") != elf_sha256 or
                normalize_relative_path(artifact.get("export_elf")) != spec["elf"]):
            raise RuntimeError(f"Top-level artifact manifest does not match uphill {variant}.")
        build_row = build_rows.get(spec["profile"], {})
        if (build_row.get("exit_code") != 0 or
                build_row.get("sha256") != elf_sha256 or
                normalize_relative_path(build_row.get("elf")) != f"build/{spec['profile']}/CarTest.elf"):
            raise RuntimeError(f"Frozen build summary does not match uphill {variant} ELF.")


def verify_speedpd_deliveries(source: Path):
    """Check the frozen SpeedPD pair against its own manifests and summary.

    The profile names are reused by later builds, so the current build output
    at ``build/<profile>`` is not authoritative for this historical delivery.
    """
    try:
        artifact_manifest = json.loads(
            (source / SPEEDPD_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / SPEEDPD_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("SpeedPD artifact manifest or build summary is missing or invalid.") from error

    if (artifact_manifest.get("schema_version") != 1 or
            artifact_manifest.get("package") != "H743_SpeedPD_20261020" or
            normalize_relative_path(artifact_manifest.get("build_summary")) !=
            SPEEDPD_BUILD_SUMMARY):
        raise RuntimeError("SpeedPD artifact manifest metadata is inconsistent.")
    artifacts = artifact_manifest.get("artifacts", {})
    summary_artifacts = build_summary.get("artifacts", {})
    variant_manifests = artifact_manifest.get("per_variant_manifests", {})
    if build_summary.get("source_inputs_stable") is not True:
        raise RuntimeError("SpeedPD build summary does not certify stable source inputs.")
    if (build_summary.get("schema_version") != 1 or
            build_summary.get("package") != "H743_SpeedPD_20261020" or
            not isinstance(build_summary.get("source_sha256"), dict)):
        raise RuntimeError("SpeedPD build summary metadata is inconsistent.")
    build_rows = index_build_rows(build_summary)
    for variant, spec in SPEEDPD_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        delivery_manifest_path = source / spec["manifest"]
        for path, label in ((delivery_elf, "delivery ELF"),
                            (delivery_manifest_path, "delivery manifest")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"SpeedPD {variant} {label} is missing or linked: {path}")

        delivery_bytes = delivery_elf.read_bytes()
        elf_sha256 = hashlib.sha256(delivery_bytes).hexdigest()
        elf_size = len(delivery_bytes)
        try:
            variant_manifest = json.loads(
                delivery_manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"SpeedPD {variant} manifest is invalid.") from error
        if (variant_manifest.get("profile") != spec["profile"] or
                variant_manifest.get("sha256") != elf_sha256 or
                variant_manifest.get("size_bytes") != elf_size or
                variant_manifest.get("build_id") is None or
                normalize_relative_path(variant_manifest.get("delivery_path")) !=
                spec["elf"] or
                normalize_relative_path(variant_manifest.get("export_elf")) !=
                spec["elf"] or
                variant_manifest.get("source_sha256") !=
                build_summary["source_sha256"]):
            raise RuntimeError(f"SpeedPD {variant} manifest does not match its ELF/profile.")

        artifact = artifacts.get(variant, {})
        if (normalize_relative_path(artifact.get("delivery_path")) != spec["elf"] or
                artifact.get("sha256") != elf_sha256 or
                normalize_relative_path(artifact.get("export_elf")) != spec["elf"] or
                artifact.get("profile") != spec["profile"] or
                artifact.get("build_id") != variant_manifest.get("build_id")):
            raise RuntimeError(f"SpeedPD artifact manifest does not match {variant}.")
        summary_artifact = summary_artifacts.get(variant, {})
        if (normalize_relative_path(summary_artifact.get("delivery_path")) !=
                spec["elf"] or
                normalize_relative_path(summary_artifact.get("export_elf")) !=
                spec["elf"] or
                summary_artifact.get("sha256") != elf_sha256 or
                summary_artifact.get("profile") != spec["profile"] or
                summary_artifact.get("build_id") != variant_manifest.get("build_id")):
            raise RuntimeError(f"SpeedPD build summary artifact does not match {variant}.")
        if normalize_relative_path(variant_manifests.get(variant)) != spec["manifest"]:
            raise RuntimeError(f"SpeedPD artifact manifest does not name {variant} manifest.")

        build_row = build_rows.get(spec["profile"], {})
        if (build_row.get("exit_code") != 0 or
                build_row.get("sha256") != elf_sha256 or
                build_row.get("size_bytes") != elf_size or
                build_row.get("profile") != spec["profile"] or
                build_row.get("build_id") != variant_manifest.get("build_id") or
                normalize_relative_path(build_row.get("elf")) !=
                f"build/{spec['profile']}/CarTest.elf"):
            raise RuntimeError(f"Frozen SpeedPD build summary does not match {variant} ELF.")


def verify_ackermann_deliveries(source: Path, expected_presets: set[str]):
    """Check the exact coordinated/baseline ELF pair and bounded build evidence."""
    try:
        artifact_manifest = json.loads(
            (source / ACKERMANN_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / ACKERMANN_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Ackermann artifact manifest or build summary is missing or invalid.") from error

    if (artifact_manifest.get("schema_version") != 1 or
            not artifact_manifest.get("generated_utc") or
            not artifact_manifest.get("package_id")):
        raise RuntimeError("Ackermann artifact manifest metadata is incomplete.")
    if (build_summary.get("schema_version") not in (None, 1) or
            build_summary.get("package_id", artifact_manifest["package_id"]) !=
            artifact_manifest["package_id"]):
        raise RuntimeError("Ackermann build summary package identity does not match its artifact manifest.")
    if build_summary.get("source_inputs_stable") is False:
        raise RuntimeError("Ackermann build summary reports that source inputs changed during build.")

    build_rows = index_build_rows(build_summary)
    if set(build_rows) != expected_presets:
        raise RuntimeError("Ackermann build summary must contain exactly the 12 declared build presets.")
    for preset, row in build_rows.items():
        row_path = normalize_relative_path(row.get("elf_path", row.get("elf")))
        row_bytes = row.get("elf_bytes", row.get("bytes"))
        row_sha = row.get("elf_sha256", row.get("sha256"))
        if (row.get("exit_code") != 0 or not row.get("log") or
                not row_path or row_bytes is None or not row_sha):
            raise RuntimeError(f"Ackermann build evidence is incomplete or failed for {preset}.")
        expected_path = f"build/{preset}/CarTest.elf"
        if row_path != expected_path:
            raise RuntimeError(f"Ackermann build summary names an unexpected ELF path for {preset}.")
        build_elf = source / expected_path
        log_value = normalize_relative_path(row.get("log"))
        if not build_elf.is_file() or build_elf.is_symlink():
            raise RuntimeError(f"Ackermann preset ELF is missing or linked: {expected_path}")
        build_bytes = build_elf.read_bytes()
        if row_bytes != len(build_bytes) or row_sha != hashlib.sha256(build_bytes).hexdigest():
            raise RuntimeError(f"Ackermann build summary hash/size does not match {preset} ELF.")
        if (not log_value or Path(log_value).is_absolute() or ".." in Path(log_value).parts):
            raise RuntimeError(f"Ackermann build log path is unsafe for {preset}.")
        log_path = source / ACKERMANN_DELIVERY_ROOT / log_value
        if not log_path.is_file() or log_path.is_symlink():
            raise RuntimeError(f"Ackermann build log is missing or linked for {preset}.")
        exitcode_value = normalize_relative_path(row.get("exitcode_file"))
        if (not exitcode_value or Path(exitcode_value).is_absolute() or
                ".." in Path(exitcode_value).parts):
            raise RuntimeError(f"Ackermann build exit-code path is unsafe for {preset}.")
        exitcode_path = source / ACKERMANN_DELIVERY_ROOT / exitcode_value
        if (not exitcode_path.is_file() or exitcode_path.is_symlink() or
                exitcode_path.read_text(encoding="utf-8-sig").strip() != "0"):
            raise RuntimeError(f"Ackermann build exit-code evidence is missing or failed for {preset}.")

    artifact_rows = artifact_manifest.get("artifacts")
    if isinstance(artifact_rows, dict):
        artifact_rows = list(artifact_rows.values())
    if not isinstance(artifact_rows, list) or len(artifact_rows) != 2:
        raise RuntimeError("Ackermann artifact manifest must contain exactly two ELF artifact rows.")
    artifacts_by_path = {}
    for row in artifact_rows:
        if not isinstance(row, dict):
            raise RuntimeError("Ackermann artifact manifest contains a malformed artifact row.")
        relative = normalize_relative_path(row.get("delivery_path", row.get("relative_path", row.get("path"))))
        if relative in artifacts_by_path:
            raise RuntimeError("Ackermann artifact manifest contains duplicate delivery paths.")
        artifacts_by_path[relative] = row

    for variant, spec in ACKERMANN_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        manifest_path = source / spec["manifest"]
        build_elf = source / "build" / spec["profile"] / "CarTest.elf"
        for path, label in ((delivery_elf, "delivery ELF"),
                            (manifest_path, "profile manifest"),
                            (build_elf, "preset ELF")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"Ackermann {variant} {label} is missing or linked: {path}")
        elf_bytes = delivery_elf.read_bytes()
        if elf_bytes != build_elf.read_bytes():
            raise RuntimeError(f"Ackermann {variant} delivery ELF differs from its named preset build.")
        elf_sha = hashlib.sha256(elf_bytes).hexdigest()
        try:
            profile_manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"Ackermann {variant} profile manifest is invalid.") from error
        if (profile_manifest.get("schema_version") != 1 or
                profile_manifest.get("profile") not in (spec["profile"], spec["directory"]) or
                profile_manifest.get("preset", spec["profile"]) != spec["profile"] or
                profile_manifest.get("coordination_enabled") is not spec["coordination_enabled"] or
                profile_manifest.get("servo_slew_step_us") != spec["servo_slew_step_us"] or
                profile_manifest.get("elf") != "CarTest.elf" or
                profile_manifest.get("bytes") != len(elf_bytes) or
                profile_manifest.get("sha256") != elf_sha or
                normalize_relative_path(profile_manifest.get("build_log")) !=
                f"../build-logs/{spec['profile']}.log"):
            raise RuntimeError(f"Ackermann {variant} profile manifest does not match its ELF/contract.")
        profile_build_log = manifest_path.parent / profile_manifest["build_log"]
        ackermann_root = (source / ACKERMANN_DELIVERY_ROOT).resolve()
        resolved_build_log = profile_build_log.resolve()
        if (ackermann_root not in resolved_build_log.parents or
                not resolved_build_log.is_file() or profile_build_log.is_symlink()):
            raise RuntimeError(f"Ackermann {variant} profile build log is missing, linked, or escapes its package.")

    for required_delivery_doc in (
            ACKERMANN_PAIRED_DELIVERY_DOC, ACKERMANN_PAIRED_RUN_STEPS,
            ACKERMANN_TOOLING_EVIDENCE):
        path = source / required_delivery_doc
        if not path.is_file() or path.is_symlink():
            raise RuntimeError(f"Required Ackermann package documentation/evidence is missing or linked: {required_delivery_doc}")

        artifact_path = f"{spec['directory']}/CarTest.elf"
        artifact = artifacts_by_path.get(artifact_path)
        if (not artifact or artifact.get("preset") != spec["profile"] or
                artifact.get("coordination_enabled") is not spec["coordination_enabled"] or
                artifact.get("bytes") != len(elf_bytes) or artifact.get("sha256") != elf_sha):
            raise RuntimeError(f"Ackermann artifact manifest does not match {variant} ELF/coordination.")
        row = build_rows[spec["profile"]]
        row_path = normalize_relative_path(row.get("elf_path", row.get("elf")))
        row_bytes = row.get("elf_bytes", row.get("bytes"))
        row_sha = row.get("elf_sha256", row.get("sha256"))
        expected_build_path = f"build/{spec['profile']}/CarTest.elf"
        if row_path != expected_build_path or row_bytes != len(elf_bytes) or row_sha != elf_sha:
            raise RuntimeError(f"Ackermann build summary does not match {variant} ELF.")


def _require_regular_checkpoint_file(source: Path, relative_name: str, label: str) -> Path:
    path = source
    for part in Path(relative_name).parts:
        path = path / part
        if path.is_symlink():
            raise RuntimeError(f"OpenMV native-preview {label} is linked: {relative_name}")
    if not path.is_file():
        raise RuntimeError(f"OpenMV native-preview {label} is missing: {relative_name}")
    return path


def verify_openmv_native_preview_delivery(source: Path) -> list[str]:
    """Validate and return only the pinned, not-yet-flashed OPENMV4P firmware candidate."""
    candidate_root = source / OPENMV_NATIVE_PREVIEW_ROOT
    if candidate_root.is_symlink():
        raise RuntimeError("OpenMV native-preview delivery directory is linked.")

    required_paths = {
        relative_name: _require_regular_checkpoint_file(
            source, relative_name, "allowlisted artifact")
        for relative_name in OPENMV_NATIVE_PREVIEW_REQUIRED_FILES
    }
    manifest_path = required_paths[
        f"{OPENMV_NATIVE_PREVIEW_ROOT}/artifact-manifest.json"]
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV native-preview artifact manifest is invalid.") from error
    if (not isinstance(manifest, dict) or
            manifest.get("run_id") != OPENMV_NATIVE_PREVIEW_RUN_ID or
            manifest.get("conclusion") != "success" or
            manifest.get("hardware_flashed") is not False or
            manifest.get("hardware_tested") is not False):
        raise RuntimeError("OpenMV native-preview run evidence is not the pinned successful, unflashed build.")

    manifest_hashes = manifest.get("files_sha256")
    if manifest_hashes != OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256:
        raise RuntimeError("OpenMV native-preview artifact manifest file/hash allowlist does not match the pinned build.")
    for name, expected_sha256 in OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256.items():
        relative_name = f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}"
        content = required_paths[relative_name].read_bytes()
        if hashlib.sha256(content).hexdigest() != expected_sha256:
            raise RuntimeError(f"OpenMV native-preview artifact hash mismatch: {name}")

    summary_path = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/build-summary.txt"]
    try:
        summary_lines = summary_path.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV native-preview build summary is unreadable.") from error
    summary = {}
    for line in summary_lines:
        if not line or "=" not in line:
            raise RuntimeError("OpenMV native-preview build summary has an invalid row.")
        key, value = line.split("=", 1)
        if not key or key in summary:
            raise RuntimeError("OpenMV native-preview build summary has duplicate or empty keys.")
        summary[key] = value
    if summary != OPENMV_NATIVE_PREVIEW_EXPECTED_BUILD:
        raise RuntimeError("OpenMV native-preview tag, commit, SDK, patch, or image hashes do not match the pinned build.")

    summary_sha = hashlib.sha256(summary_path.read_bytes()).hexdigest()
    summary_sha_path = required_paths[
        f"{OPENMV_NATIVE_PREVIEW_ROOT}/build-summary.txt.sha256"]
    try:
        summary_sha_fields = summary_sha_path.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV native-preview build-summary checksum is unreadable.") from error
    if summary_sha != OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["build-summary.txt"]:
        raise RuntimeError("OpenMV native-preview build-summary hash is not the pinned build.")
    if summary_sha_fields != [summary_sha, "build-summary.txt"]:
        raise RuntimeError("OpenMV native-preview build-summary checksum does not match the summary file.")

    patch_path = _require_regular_checkpoint_file(
        source, OPENMV_NATIVE_PREVIEW_PATCH, "pinned source patch")
    patch_sha = hashlib.sha256(patch_path.read_bytes()).hexdigest()
    if patch_sha != OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256:
        raise RuntimeError("OpenMV native-preview live pinned patch hash does not match the expected patch.")
    if summary.get("patch_sha256") != patch_sha:
        raise RuntimeError("OpenMV native-preview build summary patch hash does not match the live patch.")

    firmware_bin = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/firmware.bin"].read_bytes()
    firmware_elf = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/firmware.elf"].read_bytes()
    if len(firmware_bin) != OPENMV_NATIVE_PREVIEW_BIN_BYTES:
        raise RuntimeError("OpenMV native-preview firmware.bin size does not match the pinned image.")
    if len(firmware_elf) != OPENMV_NATIVE_PREVIEW_ELF_BYTES:
        raise RuntimeError("OpenMV native-preview firmware.elf size does not match the pinned image.")
    if (len(firmware_elf) < 52 or firmware_elf[:4] != b"\x7fELF" or
            firmware_elf[4] != 1 or firmware_elf[5] != 1 or firmware_elf[6] != 1):
        raise RuntimeError("OpenMV native-preview firmware.elf is not a little-endian ELF32 image.")
    elf_type, elf_machine, elf_version, elf_entry = struct.unpack_from("<HHII", firmware_elf, 16)
    if (elf_type != 2 or elf_machine != 40 or elf_version != 1 or
            elf_entry != OPENMV_NATIVE_PREVIEW_VECTOR_RESET or
            b"OPENMV4P" not in firmware_elf or b"STM32H743" not in firmware_elf):
        raise RuntimeError("OpenMV native-preview ELF target or entry point does not match OPENMV4P.")
    if len(firmware_bin) < 8:
        raise RuntimeError("OpenMV native-preview firmware.bin is too short for a vector table.")
    vector_sp, vector_reset = struct.unpack_from("<II", firmware_bin, 0)
    if (vector_sp != OPENMV_NATIVE_PREVIEW_VECTOR_SP or
            vector_reset != elf_entry or vector_reset != OPENMV_NATIVE_PREVIEW_VECTOR_RESET):
        raise RuntimeError("OpenMV native-preview firmware.bin vector table does not match the ELF.")

    allowlisted = list(OPENMV_NATIVE_PREVIEW_REQUIRED_FILES)
    for relative_name in OPENMV_NATIVE_PREVIEW_OPTIONAL_DOCUMENTS:
        path = source / relative_name
        if path.exists() or path.is_symlink():
            _require_regular_checkpoint_file(source, relative_name, "optional review document")
            allowlisted.append(relative_name)
    return allowlisted


def verify_openmv_ackermann_packages(source: Path) -> list[str]:
    """Return only the declared production runtime files for the paired camera packages."""
    allowlisted: list[str] = []
    shared_runtime_names: set[str] | None = None
    for profile in OPENMV_ACKERMANN_PACKAGES:
        relative_root = f"{OPENMV_ACKERMANN_ROOT}/{profile}"
        package_root = source / relative_root
        manifest_path = package_root / "PACKAGE_MANIFEST.json"
        readme_path = package_root / "README.md"
        for path, label in ((manifest_path, "manifest"), (readme_path, "README")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"OpenMV Ackermann {profile} {label} is missing or linked.")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"OpenMV Ackermann {profile} manifest is invalid.") from error
        runtime_files = manifest.get("runtime_files")
        protocol = manifest.get("protocol")
        curve_trace = manifest.get("curve_trace")
        if (manifest.get("schema_version") != 1 or
                manifest.get("control_profile") != profile or
                manifest.get("config_profile_selector") != profile or
                not manifest.get("package_id") or not manifest.get("build_id") or
                manifest.get("runtime_file_count") != len(OPENMV_ACKERMANN_RUNTIME_FILES) or
                not isinstance(runtime_files, list) or
                len(runtime_files) != len(OPENMV_ACKERMANN_RUNTIME_FILES) or
                not isinstance(protocol, dict) or
                protocol.get("version") != 2 or
                protocol.get("frame_size_bytes") != 16 or
                protocol.get("curve_phase_offset") != 10 or
                protocol.get("reserved_zero_offset") != 11 or
                protocol.get("slowdown_request_permille_offset") != 12 or
                protocol.get("crc_offset") != 14 or
                not isinstance(curve_trace, dict) or
                curve_trace.get("field") != "curve_trace_v2_hex" or
                curve_trace.get("record_size_bytes") != 16 or
                curve_trace.get("camera_diag_record_size_bytes") != 32 or
                curve_trace.get("control_ext_v2_record_size_bytes") != 32):
            raise RuntimeError(f"OpenMV Ackermann {profile} manifest metadata is incomplete.")
        runtime_names: set[str] = set()
        for entry in runtime_files:
            name = entry.get("name") if isinstance(entry, dict) else None
            if (not isinstance(name, str) or Path(name).name != name or
                    not name.endswith(".py") or name in runtime_names or
                    name not in OPENMV_ACKERMANN_RUNTIME_FILES):
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime list is unsafe, duplicated, or not allowlisted.")
            runtime_names.add(name)
            runtime_path = package_root / name
            if not runtime_path.is_file() or runtime_path.is_symlink():
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime file is missing or linked: {name}")
            content = runtime_path.read_bytes()
            if (entry.get("size_bytes") != len(content) or
                    entry.get("sha256") != hashlib.sha256(content).hexdigest() or
                    entry.get("mpy_cross_exit_code") != 0):
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime/compiler evidence mismatch: {name}")
            allowlisted.append(f"{relative_root}/{name}")
        if runtime_names != OPENMV_ACKERMANN_RUNTIME_FILES:
            raise RuntimeError(f"OpenMV Ackermann {profile} runtime set does not match the production allowlist.")
        if shared_runtime_names is None:
            shared_runtime_names = runtime_names
        elif runtime_names != shared_runtime_names:
            raise RuntimeError("OpenMV Ackermann profiles do not share the same runtime module set.")
        allowlisted.extend((f"{relative_root}/PACKAGE_MANIFEST.json", f"{relative_root}/README.md"))
    return allowlisted


def verify_openmv_speedpd_packages(source: Path) -> list[str]:
    """Validate and return only the explicit runtime files for the three camera variants."""
    allowlisted: list[str] = []
    shared_runtime_names: set[str] | None = None
    for profile in OPENMV_SPEEDPD_PACKAGES:
        relative_root = f"{OPENMV_SPEEDPD_ROOT}/{profile}"
        package_root = source / relative_root
        manifest_path = package_root / "PACKAGE_MANIFEST.json"
        readme_path = package_root / "README.md"
        for path, label in ((manifest_path, "manifest"), (readme_path, "README")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"OpenMV SpeedPD {profile} {label} is missing or linked.")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"OpenMV SpeedPD {profile} manifest is invalid.") from error
        runtime_files = manifest.get("runtime_files")
        if (manifest.get("schema_version") != 1 or
                manifest.get("control_profile") != profile or
                not manifest.get("package_id") or not manifest.get("build_id") or
                manifest.get("runtime_file_count") != 20 or
                not isinstance(runtime_files, list) or len(runtime_files) != 20):
            raise RuntimeError(f"OpenMV SpeedPD {profile} manifest metadata is incomplete.")

        runtime_names: set[str] = set()
        for entry in runtime_files:
            name = entry.get("name") if isinstance(entry, dict) else None
            if (not isinstance(name, str) or Path(name).name != name or
                    not name.endswith(".py") or name in runtime_names):
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime file list is unsafe or duplicated.")
            runtime_names.add(name)
            runtime_path = package_root / name
            if not runtime_path.is_file() or runtime_path.is_symlink():
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime file is missing or linked: {name}")
            runtime_bytes = runtime_path.read_bytes()
            runtime_sha = hashlib.sha256(runtime_bytes).hexdigest()
            if (entry.get("size_bytes") != len(runtime_bytes) or
                    entry.get("sha256") != runtime_sha or
                    entry.get("mpy_cross_exit_code") != 0):
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime/compiler evidence mismatch: {name}")
            allowlisted.append(f"{relative_root}/{name}")

        if shared_runtime_names is None:
            shared_runtime_names = runtime_names
        elif runtime_names != shared_runtime_names:
            raise RuntimeError("OpenMV SpeedPD profiles do not share the same runtime module set.")
        allowlisted.extend((
            f"{relative_root}/PACKAGE_MANIFEST.json",
            f"{relative_root}/README.md",
        ))
    return allowlisted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=PROJECT)
    parser.add_argument("--version", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--publish", action="store_true")
    args = parser.parse_args()
    for value in (args.version, args.branch):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]*", value) or ".." in value or value.endswith("/"):
            raise ValueError("Use a plain version/branch name without traversal or Git special syntax.")
    source = args.source.resolve()
    if source == STORE.resolve() or STORE.resolve() in source.parents:
        raise ValueError("The source must be outside the checkpoint worktree.")
    if not (source / "Src" / "main.c").is_file() or not (source / "openmv" / "main.py").is_file():
        raise ValueError("A complete CarTest H743/OpenMV source tree is required.")
    presets = json.loads((source / "CMakePresets.json").read_text(encoding="utf-8-sig"))
    profile_names = sorted(p["name"] for p in presets.get("buildPresets", [])
                           if not p.get("hidden", False))
    for profile in profile_names:
        if not (source / "build" / profile / "CarTest.elf").is_file():
            raise RuntimeError(f"Build the declared profile before checkpointing: {profile}")
    verify_uphill_deliveries(source)
    verify_speedpd_deliveries(source)
    verify_ackermann_deliveries(source, set(profile_names))
    openmv_speedpd_files = verify_openmv_speedpd_packages(source)
    openmv_ackermann_files = verify_openmv_ackermann_packages(source)
    openmv_native_preview_files = verify_openmv_native_preview_delivery(source)
    STORE.mkdir(parents=True, exist_ok=True)
    if not (STORE / ".git").is_dir():
        run_git("init", "-b", "main")
    run_git("config", "core.autocrlf", "false")
    if run_git("status", "--porcelain"):
        raise RuntimeError("Checkpoint worktree has uncommitted changes; preserve them before retrying.")
    existing = run_git("for-each-ref", "--format=%(refname:short)", "refs/heads").splitlines()
    if args.branch in existing:
        run_git("switch", args.branch)
    elif existing:
        run_git("switch", "-c", args.branch)
    elif args.branch != "main":
        run_git("symbolic-ref", "HEAD", "refs/heads/" + args.branch)
    if run_git("tag", "--list", args.version):
        raise RuntimeError("This immutable version tag already exists; choose a new version.")
    target = STORE / "CarTest"
    safe_remove(target)
    target.mkdir()
    for directory in DIRS:
        if (source / directory).is_dir():
            copy_tree(source / directory, target / directory)
    for name in ROOT_FILES:
        if (source / name).is_file():
            shutil.copy2(source / name, target / name)
    for name in PUBLIC_TEST_FIXTURES:
        fixture = source / name
        if not fixture.is_file() or fixture.is_symlink():
            raise RuntimeError(f"Required public regression fixture is missing or linked: {name}")
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(fixture, destination)
    for name in PUBLIC_DELIVERY_FILES:
        copy_allowlisted_file(source, target, name)
    for name in openmv_speedpd_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_ackermann_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_native_preview_files:
        copy_allowlisted_file(source, target, name)
    for profile in profile_names:
        elf = source / "build" / profile / "CarTest.elf"
        output = target / "build" / elf.parent.name / elf.name
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(elf, output)
    manifest = {str(p.relative_to(target)).replace("\\", "/"):
                hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(target.rglob("*")) if p.is_file()}
    (STORE / "SHA256SUMS.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (STORE / "VERSION.json").write_text(json.dumps({"version": args.version,
        "status": args.status, "branch": args.branch, "files": len(manifest),
        "hardware_actions": "none"}, indent=2) + "\n", encoding="utf-8")
    (STORE / "README.md").write_text("# CarTest version checkpoints\n\n"
        "H743 source, OpenMV upload files, and paired ELF checkpoints.\n\n"
        "Select a tag to restore a complete version. Experimental versions are not vehicle acceptance.\n"
        "The live source directory is not modified by this checkpoint tool.\n", encoding="utf-8")
    (STORE / ".gitattributes").write_text("* -text\n", encoding="utf-8")
    user, token = authenticated_user() if args.publish else ({"login": "CarTest-checkpoint", "id": 0}, "")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "add", "--all")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "commit", "-m", f"Checkpoint {args.version}: {args.status}")
    run_git("tag", args.version)
    commit = run_git("rev-parse", "HEAD")
    if args.publish:
        repo = api(f"/repos/{user['login']}/{REPO_NAME}", token)
        if repo is None:
            repo = api("/user/repos", token, {"name": REPO_NAME, "private": False,
                "description": "STM32H743 CarTest and OpenMV version checkpoints", "auto_init": False})
        if repo.get("private") or repo.get("owner", {}).get("login") != user["login"]:
            raise RuntimeError("Repository visibility/ownership does not match the authorized public backup.")
        remote = repo["clone_url"]
        if "origin" in run_git("remote").splitlines():
            if run_git("remote", "get-url", "origin") != remote:
                raise RuntimeError("The existing origin differs from the authorized backup repository.")
        else:
            run_git("remote", "add", "origin", remote)
        run_git("push", "--atomic", "origin", args.branch, "refs/tags/" + args.version)
        remote_commit = run_git("ls-remote", "origin", "refs/tags/" + args.version).split()[0]
        if remote_commit != commit:
            raise RuntimeError("Remote version verification failed.")
        print(json.dumps({"repository": repo["html_url"], "version": args.version,
                          "commit": commit, "remote_verified": True, "files": len(manifest)}))
    else:
        print(json.dumps({"version": args.version, "commit": commit, "published": False}))


if __name__ == "__main__":
    main()
