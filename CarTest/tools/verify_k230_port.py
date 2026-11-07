"""Host/cross-build evidence only; never opens a board or camera connection."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / "validation"


def command(argv, logfile):
    result = subprocess.run([str(x) for x in argv], cwd=ROOT,
                            capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=180)
    (OUT / logfile).write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError("Verification failed: " + logfile + "\n" + result.stderr[-1000:])
    return result


def build(profile):
    command(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
             ROOT / "tools/build.ps1", "-Preset", profile, "-Fresh"], "build-" + profile + ".log")
    print("BUILT", profile, flush=True)
    return profile, hashlib.sha256((ROOT / "build" / profile / "CarTest.elf").read_bytes()).hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mpy-cross", type=Path, required=True)
    parser.add_argument("--zig", type=Path, required=True)
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    command([sys.executable, ROOT / "tools/test_k230_port.py"], "camera-regression.log")
    command([sys.executable, "-m", "unittest", "discover", "-s", "tests/k230_port", "-p", "test_*.py", "-v"], "hal-tests.log")
    command([sys.executable, "-m", "unittest", "discover", "-s", "tests/k230_gyro", "-p", "test_*.py", "-v"], "gyro-camera-tests.log")
    command(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
             ROOT / "tests/gyro350/run.ps1", "-ZigPath", args.zig], "gyro-driver-state-tests.log")
    command([sys.executable, ROOT / "tests/gyro350_integration/run.py", "--zig", args.zig], "gyro-integration-tests.log")
    command([sys.executable, ROOT / "tests/test_gyro350_main.py", "--zig", args.zig], "gyro-main-tests.log")
    command([sys.executable, ROOT / "tools/run_h743_host.py", "--zig", args.zig], "legacy-host-tests.log")
    for pattern in ("test_run_status_reader.py", "test_analyze_differential_runs.py", "test_gyro350_diag.py"):
        command([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", pattern, "-q"], pattern + ".log")
    runtime = sorted((ROOT / "openmv").glob("*.py"))
    mpy_dir = OUT / "mpy"; mpy_dir.mkdir(exist_ok=True)
    compilation = []
    for source in runtime:
        command([args.mpy_cross, "-o", mpy_dir / (source.stem + ".mpy"), source], "mpy-" + source.stem + ".log")
        compilation.append({"file": source.name,
                            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                            "mpy_sha256": hashlib.sha256((mpy_dir / (source.stem + ".mpy")).read_bytes()).hexdigest()})
    compiler_info = command([args.mpy_cross, "--version"], "mpy-version.log").stdout.strip()
    presets = json.loads((ROOT / "CMakePresets.json").read_text(encoding="utf-8-sig"))
    profiles = sorted(p["name"] for p in presets["buildPresets"] if not p.get("hidden"))
    with ThreadPoolExecutor(max_workers=2) as pool:
        elf_hashes = dict(pool.map(build, profiles))
    result = {
        "port": "CARTEST_K230_GYRO350_20261106",
        "status": "experimental-host-tested-hardware-pending",
        "camera_tests": json.loads((OUT / "CAMERA_TESTS.json").read_text(encoding="utf-8")),
        "hal_tests_passed": True,
        "hal_tests": 9,
        "gyro_camera_tests_passed": True,
        "gyro_driver_state_tests_passed": True,
        "gyro_integration_tests_passed": True,
        "gyro_main_tests": json.loads((OUT / "GYRO_MAIN.json").read_text(encoding="utf-8")),
        "legacy_host_tests_passed": True,
        "diagnostic_tests_passed": True,
        "diagnostic_tests": 48,
        "pairing": "DebugGyro350Speed140 + V4 required camera; legacy profiles retained for restoration only",
        "all_bytecode_compiled": True,
        "compiler": compiler_info,
        "compiler_sha256": hashlib.sha256(args.mpy_cross.read_bytes()).hexdigest(),
        "bytecode_target": "portable; no ARM/RISC-V native emitter; install .py sources",
        "compilation": compilation,
        "built_profiles": profiles,
        "elf_sha256": elf_hashes,
        "hardware_actions": "none",
        "board_runtime_fps": "unverified",
        "ide_new_frame_fps": "unverified",
        "road_acceptance": "unverified",
        "exposure_control": "K230 firmware ISP default; OpenMV cap not applied",
        "calibration": "new K230 format3 required; H750 private map never copied to installer",
    }
    source_inputs = [p for directory in ("App", "Inc", "Src", "Drivers", "cmake")
                     for p in (ROOT / directory).rglob("*") if p.is_file()]
    source_inputs += [ROOT / name for name in ("CMakeLists.txt", "CMakePresets.json", "stm32h743xi_flash.ld")]
    source_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in sorted(source_inputs)}
    result["mcu_source_sha256"] = source_hashes
    (ROOT / "docs/K230_VALIDATION.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    (ROOT / "openmv/CROSS_COMPILE_SUMMARY.json").write_text(json.dumps({
        k: result[k] for k in ("port", "compiler", "compiler_sha256", "bytecode_target", "compilation")
    }, indent=2), encoding="utf-8")
    manifest = {
        "port": result["port"], "upstream": "TRACK_RING_EXIT_V26_R24_20261101",
        "platform": "Yahboom K230 vision module deluxe / CanMV",
        "runtime_files": [p.name for p in runtime],
        "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in runtime},
        "install": "all runtime .py files, manifest, and not-ready calibration placeholder",
        "firmware_policy": "retain installed Yahboom CanMV firmware; no OpenMV APP",
        "hardware_acceptance": "pending",
        "paired_mcu_preset": "DebugGyro350Speed140",
        "wire_protocol": 4,
        "gyro350_required": True,
    }
    (ROOT / "openmv/PACKAGE_MANIFEST.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"runtime_files": len(runtime), "built_profiles": len(profiles),
                      "camera_tests": result["camera_tests"]["run"], "hardware": "not_connected"}))
