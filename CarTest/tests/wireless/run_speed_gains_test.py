"""Verify wireless P/I values reach the production wheel-speed controller."""
import os
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
ZIG=Path(os.environ.get('CARTES_TEST_ZIG',r'D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe'))
with tempfile.TemporaryDirectory(prefix='cartest-speed-gains-') as temp:
    exe=Path(temp)/'speed-gains.exe'
    subprocess.run([str(ZIG),'cc','-std=c11','-Wall','-Wextra','-Werror',
        '-DCARTEST_WIRELESS_ENABLED=1','-DCARTEST_FOLLOW_INCREMENTAL_PI_ENABLED=1',
        '-I',str(ROOT/'Inc'),str(ROOT/'tests/wireless/test_speed_gains.c'),
        str(ROOT/'App/follow_incremental_pi.c'),'-o',str(exe)],check=True,cwd=ROOT)
    subprocess.run([str(exe)],check=True,cwd=ROOT)
