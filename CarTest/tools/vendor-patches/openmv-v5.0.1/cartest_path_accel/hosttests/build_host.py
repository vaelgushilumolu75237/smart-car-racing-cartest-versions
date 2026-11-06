"""Build the pure-C core as a CPython ctypes DLL with local host GCC."""

import os
import subprocess
import sys
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
ROOT = TASK.parents[3]
GCC = Path(os.environ.get(
    "CARTEST_HOST_GCC",
    r"D:\OpenMV_IDE\share\qtcreator\stedgeai\Utilities\windows\mingw64\bin\gcc.exe"))
OUTPUT_DIR = ROOT / "logs" / "ring45-usb60-20261026" / "path-c-hostbuild"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT = OUTPUT_DIR / "cartest_path_accel.dll"
SOURCE = TASK / "core" / "cartest_path_accel_core.c"
HEADER_DIR = TASK / "core"
COMMAND = [
    str(GCC), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-Wpedantic", "-shared", "-I", str(HEADER_DIR),
    "-o", str(OUTPUT), str(SOURCE),
]
ENV = os.environ.copy()
ENV["PATH"] = str(GCC.parent) + os.pathsep + ENV.get("PATH", "")

version = subprocess.run([str(GCC), "--version"], check=False,
                         capture_output=True, text=True, env=ENV)
result = subprocess.run(COMMAND, cwd=TASK, check=False,
                        capture_output=True, text=True, env=ENV)
lines = [
    "Compiler: " + str(GCC),
    "Version output:", version.stdout.strip(),
    "Command: " + subprocess.list2cmdline(COMMAND),
    "Exit code: %d" % result.returncode,
    "stdout:", result.stdout.rstrip(),
    "stderr:", result.stderr.rstrip(),
]
log = OUTPUT_DIR / "build.log"
log.write_text("\n".join(lines) + "\n", encoding="utf-8")
sys.stdout.write(log.read_text(encoding="utf-8"))
raise SystemExit(result.returncode)
