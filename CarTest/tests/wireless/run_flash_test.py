"""Test the production flash driver on Windows RAM, without device access."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ZIG = Path(os.environ.get('CARTES_TEST_ZIG', r'D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe'))
with tempfile.TemporaryDirectory(prefix='cartest-flash-host-') as temp:
    exe=Path(temp)/'flash-test.exe'
    subprocess.run([str(ZIG),'cc','-std=c11','-Wall','-Wextra','-Werror',
        '-DCARTEST_WIRELESS_ENABLED=1','-I',str(ROOT/'tests/wireless/flash_hal'),
        '-I',str(ROOT/'Inc'),str(ROOT/'tests/wireless/test_flash.c'),
        str(ROOT/'App/wireless_flash_stm32.c'),str(ROOT/'App/wireless_settings.c'),
        '-lkernel32','-o',str(exe)],check=True,cwd=ROOT)
    subprocess.run([str(exe)],check=True,cwd=ROOT)
