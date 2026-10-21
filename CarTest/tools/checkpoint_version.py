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
# The public checkpoint carries only these explicit A/B/C delivery artifacts
# and their summaries. Do not copy the broader deliverables tree or build logs.
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
    """Require new B/C snapshots to match the current builds, leaving uphill frozen."""
    try:
        artifact_manifest = json.loads(
            (source / SPEEDPD_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / SPEEDPD_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("SpeedPD artifact manifest or build summary is missing or invalid.") from error

    artifacts = artifact_manifest.get("artifacts", {})
    if build_summary.get("source_inputs_stable") is not True:
        raise RuntimeError("SpeedPD build summary does not certify stable source inputs.")
    build_rows = index_build_rows(build_summary)
    for variant, spec in SPEEDPD_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        delivery_manifest_path = source / spec["manifest"]
        build_elf = source / "build" / spec["profile"] / "CarTest.elf"
        for path, label in ((delivery_elf, "delivery ELF"),
                            (delivery_manifest_path, "delivery manifest"),
                            (build_elf, "profile build ELF")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"SpeedPD {variant} {label} is missing or linked: {path}")

        delivery_bytes = delivery_elf.read_bytes()
        if delivery_bytes != build_elf.read_bytes():
            raise RuntimeError(f"SpeedPD {variant} delivery ELF differs from {spec['profile']}.")
        elf_sha256 = hashlib.sha256(delivery_bytes).hexdigest()
        try:
            variant_manifest = json.loads(
                delivery_manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"SpeedPD {variant} manifest is invalid.") from error
        if (variant_manifest.get("profile") != spec["profile"] or
                variant_manifest.get("sha256") != elf_sha256):
            raise RuntimeError(f"SpeedPD {variant} manifest does not match its ELF/profile.")

        artifact = artifacts.get(variant, {})
        if (normalize_relative_path(artifact.get("delivery_path")) != spec["elf"] or
                artifact.get("sha256") != elf_sha256 or
                normalize_relative_path(artifact.get("export_elf")) != spec["elf"]):
            raise RuntimeError(f"SpeedPD artifact manifest does not match {variant}.")
        build_row = build_rows.get(spec["profile"], {})
        if (build_row.get("exit_code") != 0 or
                build_row.get("sha256") != elf_sha256 or
                normalize_relative_path(build_row.get("elf")) != f"build/{spec['profile']}/CarTest.elf"):
            raise RuntimeError(f"SpeedPD build summary does not match {variant} ELF.")


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
    openmv_speedpd_files = verify_openmv_speedpd_packages(source)
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
