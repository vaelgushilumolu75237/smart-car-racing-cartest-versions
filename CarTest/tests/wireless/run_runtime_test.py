"""Build and run the host wireless runtime integration test with Zig."""

from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ZIG = Path(r"D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe")


def main() -> None:
    zig = Path(os.environ.get("CARTES_TEST_ZIG", DEFAULT_ZIG))
    if not zig.exists():
        raise SystemExit(f"zig compiler not found: {zig}")
    with tempfile.TemporaryDirectory(prefix="cartest-wireless-runtime-") as temp:
        exe = Path(temp) / "wireless_runtime_test.exe"
        command = [
            str(zig), "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-DCARTEST_WIRELESS_ENABLED=1", "-DCARTEST_GYRO350_ENABLED=1",
            "-DCARTEST_TRACK_PROFILE=0", "-DCARTEST_SPEED_PERCENT=100",
            "-I", str(ROOT / "Inc"),
            str(ROOT / "tests/wireless/test_runtime.c"),
            str(ROOT / "App/wireless_runtime.c"),
            str(ROOT / "App/wireless_protocol.c"),
            str(ROOT / "App/wireless_control.c"),
            str(ROOT / "App/wireless_settings.c"),
            str(ROOT / "tests/wireless/wireless_flash_stub.c"),
            "-o", str(exe),
        ]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(exe)], cwd=ROOT, check=True)
    print("wireless runtime profile/assistant/camera diagnostics tests passed")


if __name__ == "__main__":
    main()
