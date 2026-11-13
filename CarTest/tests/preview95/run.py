"""Exercise the real C speed ramps and paired reverse encoder for C and new C."""
import argparse
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zig", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="cartest-preview95-") as tmp:
        for variant in (0, 1):
            exe = Path(tmp) / f"profile-{variant}.exe"
            subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", "Inc",
                "-DCARTEST_TRACK_PROFILE=3", "-DCARTEST_SPEED_PERCENT=100", "-DCARTEST_GYRO350_ENABLED=1",
                "-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600",
                "-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400",
                f"-DCARTEST_PREVIEW95_BEND110={variant}", "App/ackermann_control.c",
                "App/reverse_control.c", "tests/preview95/test_profile.c", "-o", str(exe)], cwd=ROOT, check=True)
            subprocess.run([str(exe)], cwd=ROOT, check=True)
if __name__ == "__main__": main()
