"""Compile and execute the production wireless control/settings host harness."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zig', required=True)
    args = parser.parse_args()
    sources = ['tests/wireless_control/test_wireless_control.c',
               'App/wireless_control.c', 'App/wireless_settings.c',
               'App/wireless_protocol.c']
    with tempfile.TemporaryDirectory(prefix='wireless-control-') as name:
        executable = Path(name) / 'control.exe'
        commands = [[args.zig, 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                     '-I', str(ROOT / 'Inc'), *sources, '-o', str(executable)],
                    [str(executable)]]
        for command in commands:
            result = subprocess.run(command, cwd=ROOT, capture_output=True,
                                    text=True, encoding='utf-8', errors='replace')
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        print(result.stdout.strip())
    tested = sources + ['Inc/wireless_control.h', 'Inc/wireless_settings.h',
                        'Inc/wireless_protocol.h', 'tests/wireless_control/run.py']
    report = {'passed': True, 'hardware_actions': 'none',
              'summary': result.stdout.strip(),
              'tested_source_sha256': {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                                       for p in tested}}
    (ROOT.parent / 'validation/WIRELESS_CONTROL.json').write_text(
        json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
