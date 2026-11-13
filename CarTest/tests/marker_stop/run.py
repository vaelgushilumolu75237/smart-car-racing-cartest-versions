"""Compile and exercise the actual protocol/control path in all six profiles."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zig', required=True)
    args = parser.parse_args()
    def run(cmd):
        p = subprocess.run([str(x) for x in cmd], cwd=ROOT, capture_output=True, text=True)
        print(p.stdout + p.stderr, end='', flush=True)
        if p.returncode:
            raise SystemExit(p.returncode)
    run(['python', '-m', 'unittest', 'discover', '-s', 'openmv/tests', '-p', 'test_black_marker.py'])
    with tempfile.TemporaryDirectory(prefix='cartest-marker-') as temp:
        protocol = Path(temp) / 'protocol.o'
        run([args.zig, 'c++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I', 'Inc',
             '-c', 'App/vision_protocol.cpp', '-o', protocol])
        for profile, percent in ((1, 100), (2, 100), (3, 100), (3, 120), (3, 130), (3, 140)):
            fast, slow = (4800, 4000) if profile == 2 else (5600, 4400)
            exe = Path(temp) / f'marker-{profile}-{percent}.exe'
            run([args.zig, 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                 '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-I', 'Inc',
                 '-DCARTEST_GYRO350_ENABLED=1', '-DCARTEST_RED_OBSTACLE_ENABLED=1',
                 f'-DCARTEST_TRACK_PROFILE={profile}', f'-DCARTEST_SPEED_PERCENT={percent}',
                 f'-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS={fast}',
                 f'-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS={slow}',
                 'App/commissioning.c', 'App/gyro350_authority.c', 'App/gyro_ring.c',
                 'App/obstacle_authority.c', 'App/obstacle_bypass.c',
                 'tests/marker_stop/test_marker.c', protocol, '-o', exe])
            run([exe])

if __name__ == '__main__':
    main()
