"""Run the existing production C/C++ host suites using Zig; no board access."""
from pathlib import Path
import argparse
import concurrent.futures
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / "validation/h743-host"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zig", type=Path, required=True)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    def run(argv, name):
        result = subprocess.run([str(a) for a in argv], cwd=ROOT,
                                capture_output=True, text=True, errors="replace", timeout=180)
        (OUT / name).write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            raise RuntimeError(name + "\n" + result.stdout[-2000:] + result.stderr[-2000:])
        return result.stdout

    # Retain the flags and unit list of the established MSVC runner.
    units = {}
    for line in (ROOT / "tests/host/run_tests.ps1").read_text(encoding="utf-8").splitlines():
        if "cl /nologo" not in line or " /c " not in line:
            continue
        index = re.search(r"\$objectFiles\[(\d+)\]", line)
        source = re.search(r"Join-Path \$(projectRoot|PSScriptRoot) '([^']+)'", line)
        if not index or not source:
            continue
        i = int(index[1])
        if i not in list(range(19)) + [21, 22, 23, 24]:
            continue
        path = (ROOT if source[1] == "projectRoot" else ROOT / "tests/host") / source[2].replace("\\", "/")
        units[i] = (path, ["-D" + d for d in re.findall(r"/D([^\s]+)", line)])
    assert set(units) == set(range(19)) | {21, 22, 23, 24}

    def compile_unit(item):
        i, (source, defines) = item
        obj = OUT / (str(i) + ".o")
        run([args.zig, "c++" if source.suffix == ".cpp" else "cc",
             "-std=c++17" if source.suffix == ".cpp" else "-std=c11",
             "-I", ROOT / "Inc", "-c", source, "-o", obj, *defines], str(i) + ".log")
        return i, obj

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        objects = dict(pool.map(compile_unit, units.items()))
    summaries = {}
    for name, indices in {
        "CarTestHostTests": [i for i in sorted(objects) if i not in (21, 22, 23)],
        "VisionHoldProfile": [23, 21, 1, 2, 3, 4, 5, 6, 8, 9, 10, 22, 12, 13, 14, 15, 16, 17, 24],
    }.items():
        exe = OUT / (name + ".exe")
        run([args.zig, "c++", *[objects[i] for i in indices], "-o", exe], name + "-link.log")
        output = run([exe], name + "-run.log")
        summaries[name] = output.strip()
        print(output[-4000:])
    (OUT / "RESULT.json").write_text(json.dumps({"passed": True, "summaries": summaries,
        "gyro_enabled": False, "hardware": "not_connected"}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
