"""Explicit K230 checkpoint path using the existing checkpoint Git publisher.

The old OpenMV delivery allowlists deliberately do not apply to a RISC-V port.
Public copying is restricted to named source roots, current runtime, paired
ELFs and reviewed validation summaries. No calibration or captured data.
"""
import hashlib
import json
from pathlib import Path
import shutil
import checkpoint_version as base

OWNER = "vaelgushilumolu75237"


def checkpoint(args):
    source = args.source.resolve()
    store = base.STORE.resolve()
    if source == store or store in source.parents:
        raise ValueError("Source must be outside checkpoint worktree")
    for name in ("Src/main.c", "openmv/main.py", "openmv/k230_hal.py",
                 "openmv/k230_settings.py", "openmv/PACKAGE_MANIFEST.json",
                 "docs/K230_PORT_20261106.md", "docs/K230_VALIDATION.json"):
        if not (source / name).is_file():
            raise ValueError("Incomplete K230 source: " + name)
    manifest = json.loads((source / "openmv/PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    for name, digest in manifest["source_sha256"].items():
        if Path(name).name != name or hashlib.sha256((source / "openmv" / name).read_bytes()).hexdigest() != digest:
            raise ValueError("K230 runtime hash mismatch: " + name)
    validation = json.loads((source / "docs/K230_VALIDATION.json").read_text(encoding="utf-8"))
    if validation["camera_tests"]["failures"] or validation["camera_tests"]["errors"]:
        raise ValueError("Algorithm tests have failures")
    if not validation["hal_tests_passed"] or not validation["all_bytecode_compiled"]:
        raise ValueError("K230 HAL or bytecode checks incomplete")
    if manifest.get("gyro350_required"):
        for gate in ("gyro_camera_tests_passed", "gyro_driver_state_tests_passed",
                     "gyro_integration_tests_passed", "legacy_host_tests_passed", "diagnostic_tests_passed"):
            if validation.get(gate) is not True:
                raise ValueError("Gyro verification incomplete: " + gate)
        if not validation.get("gyro_main_tests", {}).get("passed"):
            raise ValueError("Production gyro output tests incomplete")
        for name, expected in validation["mcu_source_sha256"].items():
            if hashlib.sha256((source / name).read_bytes()).hexdigest() != expected:
                raise ValueError("MCU source changed since validation: " + name)
    if manifest.get("wireless_required") and not validation.get("wireless_tests_passed"):
        raise ValueError("Wireless verification incomplete")
    if manifest.get("release_version") in ("v20261108-k230-imu-mode0-v8",
                                          "v20261108-k230-imu-timing-v9"):
        report = validation.get("wireless_reports", {}).get("IMU_GPIO_SPI", {})
        if validation.get("imu_gpio_spi_tests_passed") is not True or report.get("passed") is not True:
            raise ValueError("GPIO SPI driver verification incomplete")
        tested = report.get("tested_source_sha256", {})
        if tested.get("App/icm42688_stm32.c") != hashlib.sha256(
                (source / "App/icm42688_stm32.c").read_bytes()).hexdigest():
            raise ValueError("GPIO SPI driver changed since low-level tests")
    presets = json.loads((source / "CMakePresets.json").read_text(encoding="utf-8-sig"))
    profiles = sorted(p["name"] for p in presets["buildPresets"] if not p.get("hidden"))
    if set(profiles) != set(validation["built_profiles"]):
        raise ValueError("All declared MCU builds must have current evidence")
    for profile in profiles:
        p = source / "build" / profile / "CarTest.elf"
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != validation["elf_sha256"][profile]:
            raise ValueError("ELF build evidence mismatch: " + profile)
    store.mkdir(parents=True, exist_ok=True)
    if not (store / ".git").is_dir():
        base.run_git("init", "-b", args.branch)
    if base.run_git("status", "--porcelain"):
        raise RuntimeError("Checkpoint worktree is dirty; preserve it first")
    if base.run_git("tag", "--list", args.version):
        raise RuntimeError("Immutable tag already exists")
    if base.run_git("branch", "--show-current") != args.branch:
        raise RuntimeError("Use the dedicated K230 checkpoint branch")
    target = store / "CarTest"
    base.safe_remove(target)
    target.mkdir()
    for directory in ("App", "Inc", "Src", "Drivers", "cmake", "tools", "tests", ".vscode"):
        if (source / directory).is_dir():
            base.copy_tree(source / directory, target / directory,
                           excluded_subtrees=base.PUBLIC_COPY_EXCLUDED_SUBTREES.get(directory, ()))
    for filename in base.ROOT_FILES:
        if (source / filename).is_file():
            shutil.copy2(source / filename, target / filename)
    for filename in manifest["runtime_files"] + ["PACKAGE_MANIFEST.json", "README.md"]:
        if "calibration.json" in filename:
            raise ValueError("Private calibration is forbidden in public input")
        destination = target / "openmv" / filename
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(source / "openmv" / filename, destination)
    # Source oracles and synthetic vectors are needed to reproduce camera tests.
    # Real historical calibration maps are explicitly excluded, even in fixtures.
    base.copy_tree(source / "openmv/tests", target / "openmv/tests", excluded_subtrees=(
        "frozen_pre_ring/ring_width_calibration.json",
        "fixtures/steering_hold3s_20261016/ring_width_calibration.json"))
    for fixture in ("test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.manifest.json",
                    "test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.zip",
                    "test-data/runtime-log-ab-20261012/baseline/Src__main.c",
                    "deliverables/CarTest-20261014-0422-after/build/DebugFollowTurn/CarTest.elf",
                    "test-data/failure3-20261015/baseline/build/DebugFollowTurn/CarTest.elf"):
        if fixture not in base.PUBLIC_TEST_FIXTURES:
            raise ValueError("Fixture must be explicitly approved as source/synthetic input")
        expected = base.PUBLIC_TEST_FIXTURE_SHA256.get(fixture)
        if expected and hashlib.sha256((source / fixture).read_bytes()).hexdigest() != expected:
            raise ValueError("Historical ELF fixture identity changed: " + fixture)
        base.copy_allowlisted_file(source, target, fixture)
    doc_files = ["K230_PORT_20261106.md", "K230_VALIDATION.json", "K230_PORT_TRANSFORM.json",
                 "GYRO350_20261106.md", "GYRO350_VENDOR_FILES.json"]
    if (source / 'docs/FRESH_ROUTE_V24_CN.md').is_file():
        doc_files.append('FRESH_ROUTE_V24_CN.md')
    if (source / 'docs/UART_RX_V25_CN.md').is_file():
        doc_files += ['UART_RX_V25_CN.md', 'UART_RX_V25_VALIDATION.json']
    if (source / 'docs/SINGLE_EDGE_AUTHORITY_V26_CN.md').is_file():
        doc_files += ['SINGLE_EDGE_AUTHORITY_V26_CN.md',
                      'SINGLE_EDGE_AUTHORITY_V26_VALIDATION.json']
    if (source / 'docs/TURN_SPEED_V27_CN.md').is_file():
        doc_files += ['TURN_SPEED_V27_CN.md', 'TURN_SPEED_V27_VALIDATION.json']
    if (source / 'docs/SPEED_V28_CN.md').is_file():
        doc_files += ['SPEED_V28_CN.md', 'SPEED_V28_VALIDATION.json']
    if (source / 'docs/RING_BASELINE_V29_CN.md').is_file():
        doc_files += ['RING_BASELINE_V29_CN.md', 'RING_BASELINE_V29_VALIDATION.json']
    if (source / 'docs/NEW_A_SPEED_V30_CN.md').is_file():
        doc_files += ['NEW_A_SPEED_V30_CN.md', 'NEW_A_SPEED_V30_VALIDATION.json']
    if (source / 'docs/FPS65_V31_CN.md').is_file():
        doc_files += ['FPS65_V31_CN.md', 'FPS65_V31_VALIDATION.json']
    if manifest.get("wireless_required"):
        doc_files += ["WIRELESS_20261107.md", "WIRELESS_PC_PROTOCOL.md", "WIRELESS_CONTRACT.md",
                      "WIRELESS_VENDOR_FILES.json", "WIRELESS_VALIDATION.json"]
    for filename in doc_files:
        destination = target / "docs" / filename
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(source / "docs" / filename, destination)
    for profile in profiles:
        destination = target / "build" / profile / "CarTest.elf"
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / "build" / profile / "CarTest.elf", destination)
    hashes = {p.relative_to(target).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(target.rglob("*")) if p.is_file()}
    (store / "SHA256SUMS.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    (store / "VERSION.json").write_text(json.dumps({"version": args.version,
        "status": args.status, "branch": args.branch, "files": len(hashes),
        "hardware_actions": validation.get("hardware_actions", "none"),
        "deployment_actions": "none", "platform": "Yahboom K230 deluxe",
        "hardware_diagnostics": validation.get("hardware_diagnostics", {}),
        "private_calibration_included": False}, indent=2), encoding="utf-8")
    (store / ".gitattributes").write_text("* -text\n", encoding="ascii")
    base.run_git("config", "core.autocrlf", "false")
    base.run_git("add", "--all")
    base.run_git("-c", "user.name=CarTest-checkpoint", "-c",
                 "user.email=cartest-checkpoint@users.noreply.github.com",
                 "commit", "-m", "Checkpoint " + args.version + ": " + args.status)
    base.run_git("tag", args.version)
    commit = base.run_git("rev-parse", "HEAD")
    result = {"version": args.version, "commit": commit, "remote_verified": False}
    if args.publish:
        user, token = base.authenticated_user()
        if user["login"].lower() != OWNER.lower():
            raise RuntimeError("Authenticated account differs from authorized repository owner")
        repo = base.api("/repos/" + OWNER + "/" + base.REPO_NAME, token)
        if not repo or repo.get("private") or repo["owner"]["login"].lower() != OWNER.lower():
            raise RuntimeError("Public authorized repository verification failed")
        remote = repo["clone_url"]
        if "origin" in base.run_git("remote").splitlines():
            if base.run_git("remote", "get-url", "origin") != remote:
                raise RuntimeError("Unexpected remote")
        else:
            base.run_git("remote", "add", "origin", remote)
        base.run_git("push", "--atomic", "origin", args.branch, "refs/tags/" + args.version)
        tag = base.run_git("ls-remote", "origin", "refs/tags/" + args.version).split()
        branch = base.run_git("ls-remote", "origin", "refs/heads/" + args.branch).split()
        if not tag or not branch or tag[0] != commit or branch[0] != commit:
            raise RuntimeError("Remote commit/tag verification failed")
        result.update(repository=repo["html_url"], remote_verified=True)
    (source.parent / "PUBLISH_RESULT.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))
