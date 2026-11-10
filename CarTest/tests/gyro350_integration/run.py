"""Build and run the production V4 camera-to-H743 gyro-ring host harness."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent
DEFAULT_ZIG = Path(
    r"D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe"
)


def run_checked(args: list[str], cwd: Path, **kwargs) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(args, cwd=cwd, text=True, encoding="utf-8",
                            errors="replace", capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(
            "command failed: " + " ".join(args) + "\n" + result.stdout + result.stderr
        )
    return result


def find_vs_install() -> Path:
    vswhere = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe")
    result = run_checked(
        [str(vswhere), "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        ROOT,
    )
    path = Path(result.stdout.strip())
    if not path:
        raise RuntimeError("Visual Studio C compiler installation not found")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zig", type=Path, default=DEFAULT_ZIG)
    parser.add_argument("--wireless", action="store_true")
    parser.add_argument("--json", type=Path,
                        default=ROOT.parent / "validation" / "GYRO_INTEGRATION.json")
    args = parser.parse_args()
    if args.wireless:
        args.json = ROOT.parent / "validation" / "WIRELESS_VISION_INTEGRATION.json"
    vs = find_vs_install()
    vcvars = vs / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    if not vcvars.is_file():
        raise RuntimeError(f"missing vcvars64.bat: {vcvars}")
    if not args.zig.is_file():
        raise RuntimeError(f"missing requested Zig path: {args.zig}")

    sources_c = [
        "App/commissioning.c", "App/servo_safety.c", "App/reverse_control.c",
        "App/car_vision.c", "App/encoder_diag.c", "App/runtime_log.c",
        "App/uart_rx_service.c", "App/run_block_reason.c", "App/manual_run_button.c",
        "App/follow_output.c", "App/speed_observer.c", "App/speed_pi.c",
        "App/follow_speed_pi.c", "App/follow_incremental_pi.c", "App/runtime_diag.c",
        "App/receive_diag.c", "App/gyro_ring.c", "App/gyro350_authority.c",
    ]
    source_cpp = ["App/vision_protocol.cpp", str(TEST_DIR.relative_to(ROOT) /
                                                  "test_gyro350_integration.cpp")]
    if args.wireless:
        sources_c += ["App/wireless_runtime.c", "App/wireless_protocol.c",
                      "App/wireless_control.c", "App/wireless_settings.c", "App/wireless_flash_stm32.c"]

    with tempfile.TemporaryDirectory(prefix="CarTestGyro350-") as temp_name:
        temp = Path(temp_name)
        objects: list[Path] = []
        batch: list[str] = ["@echo off", f'call "{vcvars}" >nul 2>&1',
                            "if errorlevel 1 exit /b 1", f'cd /d "{ROOT}"']
        common = (
            f'/I"{ROOT / "Inc"}" /DCARTEST_GYRO350_ENABLED=1 '
            '/DCARTEST_TRACK_PROFILE=3 /DCARTEST_SPEED_PERCENT=140 '
            '/DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600 '
            '/DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400'
        )
        if args.wireless:
            common += ' /DCARTEST_WIRELESS_ENABLED=1'

        for index, source in enumerate(sources_c):
            obj = temp / f"c{index}.obj"
            objects.append(obj)
            flags = common + " /std:c11 /W4 /TC"
            if source == "App/wireless_flash_stm32.c":
                flags = flags.replace('/DCARTEST_WIRELESS_ENABLED=1', '')
            if source == "App/runtime_log.c":
                flags += " /DCAR_RUNTIME_LOG_HOST_TEST"
            if source == "App/runtime_diag.c":
                flags += " /DCAR_RUNTIME_DIAG_HOST_TEST"
            if source == "App/receive_diag.c":
                flags += " /DCAR_RECEIVE_DIAG_HOST_TEST"
            batch.append(f'cl /nologo {flags} /c /Fo"{obj}" "{ROOT / source}"')
            batch.append("if errorlevel 1 exit /b 1")

        for index, source in enumerate(source_cpp):
            obj = temp / f"cpp{index}.obj"
            objects.append(obj)
            batch.append(
                f'cl /nologo {common} /std:c++17 /EHsc /W4 /TP /c /Fo"{obj}" "{ROOT / source}"'
            )
            batch.append("if errorlevel 1 exit /b 1")

        exe = temp / "gyro350-integration.exe"
        batch.append(f'cl /nologo /Fe:"{exe}" ' + " ".join(f'"{obj}"' for obj in objects))
        batch.append("if errorlevel 1 exit /b 1")
        batch_path = temp / "build.bat"
        batch_path.write_text("\n".join(batch) + "\n", encoding="ascii")
        build = subprocess.run(["cmd.exe", "/d", "/s", "/c", str(batch_path)],
                               cwd=ROOT, text=True, encoding="utf-8",
                               errors="replace", capture_output=True)
        if build.returncode:
            raise RuntimeError("MSVC integration build failed\n" + build.stdout + build.stderr)
        result = subprocess.run([str(exe)], cwd=ROOT, text=True,
                                encoding="utf-8", errors="replace",
                                capture_output=True)
        print(result.stdout, end="")
        if result.returncode:
            raise RuntimeError("gyro350 integration assertions failed\n" + result.stderr)

    args.json.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "passed": True,
        "status": "passed",
        "harness": "tests/gyro350_integration/test_gyro350_integration.cpp",
        "build": "MSVC x64 host, CARTEST_GYRO350_ENABLED=1, profile C, speed 140%",
        "zig_path": str(args.zig),
        "zig_sha256": hashlib.sha256(args.zig.read_bytes()).hexdigest(),
        "hardware_actions": "none",
        "cases": [
            "startup release and IMU/profile gate",
            "production car_vision RX V4 parser path",
            "left entry, inside zero, signed 350 degree orbit",
            "center 1530 us and 300 ms straight stage",
            "invalid route bounded search and three-frame recovery",
            "right mirrored signed progress",
            "IMU fault and PE7 stop reset",
            "wireless recoverable protocol fault preserves run request and three-route automatic recovery" if args.wireless else "wireless disabled",
            "wireless ordinary line loss keeps session after 3s stop; three fresh routes restore PWM-gated output; local stop cancels" if args.wireless else "wireless disabled",
        ],
    }
    args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
