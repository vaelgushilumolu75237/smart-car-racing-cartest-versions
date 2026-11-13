"""Real wireless-enabled boot settings and ring control regression."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ZIG = Path(r"D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe")


def main():
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        stub = (ROOT / "tests/wireless/wireless_flash_stub.c").read_text()
        stub = stub.replace("#include <string.h>", "#include <string.h>\nextern unsigned test_saved_settings;")
        stub = stub.replace("    (void)settings;\n    (void)generation;\n    (void)selected_slot;\n    return 0u;", """    if (test_saved_settings) {
        wireless_settings_defaults(settings);
        settings->gyro_rate_mdps=85714;
        settings->wheel_scale_permille=750;
        *generation=7;*selected_slot=0;return 1;
    }
    return 0;""", 1)
        (temp / "flash.c").write_text(stub)
        for tune in (0, 3, 4):
            exe = temp / f"rate-{tune}.exe"
            command = [ZIG, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", "Inc",
                       "-DCARTEST_WIRELESS_ENABLED=1", "-DCARTEST_GYRO350_ENABLED=1",
                       "-DCARTEST_TRACK_PROFILE=3", "-DCARTEST_SPEED_PERCENT=100",
                       "-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600",
                       "-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400",
                       f"-DCARTEST_RING_TUNE={tune}", "App/wireless_runtime.c",
                       "App/wireless_protocol.c", "App/wireless_control.c", "App/wireless_settings.c",
                       "App/gyro_ring.c", "App/icm42688.c", "tests/ring_profiles/test_wireless_rate.c",
                       temp / "flash.c", "-o", exe]
            subprocess.run([str(x) for x in command], cwd=ROOT, check=True)
            subprocess.run([str(exe)], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
