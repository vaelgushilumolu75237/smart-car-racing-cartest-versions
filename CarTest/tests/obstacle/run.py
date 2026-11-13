"""Host-only red detector, six-profile bypass and control-authority tests."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

def main():
    p=argparse.ArgumentParser();p.add_argument("--zig",required=True);args=p.parse_args()
    def run(cmd):
        r=subprocess.run([str(x) for x in cmd],cwd=ROOT,capture_output=True,text=True)
        print(r.stdout+r.stderr,end="",flush=True)
        if r.returncode: raise SystemExit(r.returncode)
    run(["python","-m","unittest","discover","-s","tests/obstacle","-p","test_*.py"])
    with tempfile.TemporaryDirectory(prefix="cartest-obstacle-") as tmp:
        for profile,percent in [(1,100),(2,100),(3,100),(3,120),(3,130),(3,140)]:
            fast,slow=(4800,4000) if profile==2 else (5600,4400)
            flags=[f"-DCARTEST_TRACK_PROFILE={profile}",f"-DCARTEST_SPEED_PERCENT={percent}",
                   f"-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS={fast}",
                   f"-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS={slow}"]
            for name,sources,extra in [
                ("core",["App/obstacle_bypass.c","tests/obstacle/test_obstacle.c"],[]),
                ("authority",["App/commissioning.c","App/gyro350_authority.c","App/gyro_ring.c",
                              "App/obstacle_authority.c","App/obstacle_bypass.c","tests/obstacle/test_authority.c"],
                 ["-DCARTEST_GYRO350_ENABLED=1","-DCARTEST_RED_OBSTACLE_ENABLED=1",
                  "-ffunction-sections","-fdata-sections","-Wl,--gc-sections"])]:
                exe=Path(tmp)/f"{name}-{profile}-{percent}.exe"
                run([args.zig,"cc","-std=c11","-Wall","-Wextra","-Werror","-I","Inc",*flags,*extra,*sources,"-o",exe])
                run([exe])
if __name__=="__main__":main()
