import argparse
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument("--zig",required=True);a=p.parse_args()
with tempfile.TemporaryDirectory() as d:
    exe=Path(d)/"motor_max.exe"
    subprocess.run([a.zig,"cc","-std=c11","-Wall","-Wextra","-Werror","-I","Inc",
                    "App/motor_max_test.c","App/manual_run_button.c","App/speed_observer.c",
                    "tests/motor_max/test_motor_max.c","-o",str(exe)],cwd=ROOT,check=True)
    subprocess.run([str(exe)],check=True)
