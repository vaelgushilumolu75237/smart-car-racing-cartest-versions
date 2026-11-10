"""Refresh camera regression and portable-bytecode evidence without device I/O."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / 'validation'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mpy-cross', type=Path, required=True)
    args = parser.parse_args()
    runtime = sorted((ROOT / 'openmv').glob('*.py'))
    paths = runtime + [Path(__file__).resolve(), ROOT / 'tools/test_k230_port.py']
    for directory in ('openmv/tests', 'tests/k230_port', 'tests/k230_gyro', 'tests/wireless_camera'):
        paths += sorted((ROOT / directory).rglob('*.py'))
    hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}
    OUT.mkdir(exist_ok=True)

    def run(command, name):
        result = subprocess.run([str(p) for p in command], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', errors='replace')
        output = result.stdout + result.stderr
        (OUT / (name + '.log')).write_text(output, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(name + ' failed:\n' + output[-2000:])
        return output

    run([sys.executable, 'tools/test_k230_port.py'], 'camera-regression')
    algorithm = json.loads((OUT / 'CAMERA_TESTS.json').read_text(encoding='utf-8'))
    algorithm.update(passed=not algorithm['failures'] and not algorithm['errors'])
    reports = {}
    for name, directory in (('hal', 'k230_port'), ('gyro_camera', 'k230_gyro'),
                            ('wireless_camera_tests', 'wireless_camera')):
        command = [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests/' + directory,
                   '-p', 'test_*.py', '-q']
        output = run(command, name)
        match = re.search(r'Ran (\d+) tests?', output)
        if not match:
            raise RuntimeError('Missing unittest count: ' + name)
        reports[name] = dict(passed=True, tests=int(match.group(1)), command=command[1:])
    bytecode = OUT / 'mpy'
    bytecode.mkdir(exist_ok=True)

    def compile_source(source):
        destination = bytecode / (source.stem + '.mpy')
        run([args.mpy_cross, '-o', destination, source], 'mpy-' + source.stem)
        return dict(file=source.name, source_sha256=digest(source), mpy_sha256=digest(destination))

    with ThreadPoolExecutor(max_workers=2) as pool:
        compilation = list(pool.map(compile_source, runtime))
    if hashes != {p.relative_to(ROOT).as_posix(): digest(p) for p in paths}:
        raise RuntimeError('Camera sources/tests changed during verification')
    report = dict(passed=True, scope='Production camera host tests and portable bytecode only',
                  hardware_actions='none', wireless_tests_passed=True,
                  k230_port_full_regression=algorithm, compilation=compilation,
                  source_sha256=hashes,
                  mpy_cross=dict(all_bytecode_compiled=True, compiled_modules=len(runtime),
                                 compiler=str(args.mpy_cross), compiler_sha256=digest(args.mpy_cross)),
                  **reports)
    (OUT / 'WIRELESS_CAMERA.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=True, algorithm_tests=algorithm['run'],
                          suites={key: value['tests'] for key, value in reports.items()},
                          compiled_modules=len(runtime))))


if __name__ == '__main__':
    main()
