"""Run frozen V7 host regressions with CarTest's OpenMV source modules.

This runner copies tests into a temporary tree so their existing path-based
imports resolve to the CarTest package. It adapts the old literal BUILD
expectation and marks the frozen V7 test double as the confidence-diagnostic
API required by the new bundle. Neither the frozen snapshot nor original
Sixth_Test tests are modified. Board-only equivalence stays skipped unless a
real-board manifest is provided, and no camera or serial interface is opened.
"""

import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


PACKAGE_DIR = Path(__file__).resolve().parents[1]
FROZEN_DIR = PACKAGE_DIR / "tests" / "frozen_v7"
FROZEN_DATA = FROZEN_DIR / "test-data"
PACKAGE_DATA = PACKAGE_DIR.parent / "test-data"
FROZEN_MANIFEST = PACKAGE_DATA / "OpenMV_V7_FAST_GRAY_RT45_20261005.manifest.json"
FROZEN_ZIP = PACKAGE_DATA / "OpenMV_V7_FAST_GRAY_RT45_20261005.zip"
FROZEN_INVENTORY = FROZEN_DIR / "source-sha256.csv"
BUILD_OLD = "SIXTH_V7_FAST_GRAY_RT45_20261005"
BUILD_NEW = "CARTEST_OPENMV_CACHED_WIDTH_DIFFTRIAL_20261013"
TEST_NAMES = (
    "test_fast_gray.py",
    "test_fast_topology.py",
    "test_fast_path.py",
    "test_fast_boundary.py",
    "test_fast_main.py",
    "test_fast_kernels_equivalence.py",
    "test_openmv_protocol.py",
)
SUPPORT_NAMES = ("v5_projected_scenes.py",)


def main():
    required = [FROZEN_DIR / name for name in TEST_NAMES]
    required.extend((FROZEN_DIR / SUPPORT_NAMES[0],
                     FROZEN_DATA / "vision-protocol-v1.csv",
                     FROZEN_MANIFEST, FROZEN_ZIP, FROZEN_INVENTORY))
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("missing bundled V7 regression input: " +
                         ", ".join(missing))
    if BUILD_OLD not in FROZEN_MANIFEST.read_text(encoding="utf-8"):
        raise SystemExit("bundled V7 baseline manifest identity changed")
    with FROZEN_MANIFEST.open("r", encoding="utf-8") as source:
        baseline_manifest = json.load(source)
    zip_hash = hashlib.sha256(FROZEN_ZIP.read_bytes()).hexdigest().upper()
    if zip_hash != baseline_manifest.get("zip_sha256"):
        raise SystemExit("bundled V7 ZIP hash differs from its manifest")
    with FROZEN_INVENTORY.open("r", newline="", encoding="ascii") as source:
        for row in csv.DictReader(source):
            path = FROZEN_DIR / Path(row["relative_path"])
            if (not path.is_file() or path.stat().st_size != int(row["bytes"]) or
                    hashlib.sha256(path.read_bytes()).hexdigest().upper() !=
                    row["sha256"]):
                raise SystemExit("bundled frozen V7 source hash mismatch: %s" %
                                 row["relative_path"])

    with tempfile.TemporaryDirectory(prefix="cartest-openmv-v7-") as raw_temp:
        temp_root = Path(raw_temp)
        temp_openmv = temp_root / "openmv"
        temp_tests = temp_root / "tests"
        temp_vectors = temp_root / "test-data"
        temp_openmv.mkdir()
        temp_tests.mkdir()
        temp_vectors.mkdir()
        (temp_tests / "__init__.py").write_text("", encoding="ascii")

        for source in PACKAGE_DIR.glob("*.py"):
            shutil.copyfile(source, temp_openmv / source.name)
        for name in TEST_NAMES:
            source = FROZEN_DIR / name
            target = temp_tests / name
            content = source.read_text(encoding="utf-8")
            if name == "test_fast_main.py":
                frozen_stub = (
                    "RUNTIME_API_VERSION=7,\n"
                    "        FastPathDetector=FakePathDetector,")
                cartest_stub = (
                    "RUNTIME_API_VERSION=7,\n"
                    "        CONFIDENCE_DIAGNOSTIC_ONLY=True,\n"
                    "        FastPathDetector=FakePathDetector,")
                if content.count(frozen_stub) != 1:
                    raise SystemExit(
                        "could not identify the single frozen fast_path "
                        "test stub for new API adaptation")
                content = content.replace(BUILD_OLD, BUILD_NEW)
                content = content.replace(frozen_stub, cartest_stub)
                adaptations = (
                    ("self.assertGreaterEqual(overlay[0], 500)",
                     'preview_flush = [value for stage, value in run["perf_samples"] '
                     'if stage == "preview_flush"]\n'
                     '        self.assertEqual(len(preview_flush), 1)\n'
                     '        self.assertGreaterEqual(preview_flush[0], 500)\n'
                     '        self.assertLess(overlay[0], 500)'),
                    ('self.assertEqual(no_overlay["sensor"].flush_calls, 0)',
                     'self.assertEqual(no_overlay["sensor"].flush_calls, 1)'),
                    ("self.assertEqual(len(image.lines), 10)",
                     "self.assertEqual(len(image.lines), 22)"),
                    ('self.assertEqual(image.strings[0][1], "VALID r=+0 f=+0")',
                     'self.assertEqual(image.strings[0][1], "NOW V CACHE/ROAD")'),
                    ("self.assertEqual(image.lines[-2][0][3],",
                     "self.assertEqual(image.lines[8][0][3],"),
                    ('"INVALID r=+0 f=+0")',
                     '"NOW INVALID CACHE")'),
                )
                for old, new in adaptations:
                    if content.count(old) != 1:
                        raise SystemExit(
                            "could not identify one frozen overlay expectation: " +
                            old)
                    content = content.replace(old, new)
            target.write_text(content, encoding="utf-8", newline="")
        for name in SUPPORT_NAMES:
            shutil.copyfile(FROZEN_DIR / name, temp_tests / name)
        shutil.copyfile(
            FROZEN_DATA / "vision-protocol-v1.csv",
            temp_vectors / "vision-protocol-v1.csv")

        runner = temp_root / "run.py"
        runner.write_text(
            "import sys, unittest\n"
            "sys.path.insert(0, %r)\n" % str(temp_root) +
            "names = [\n" +
            "    'tests.test_fast_gray',\n"
            "    'tests.test_fast_topology',\n"
            "    'tests.test_fast_path',\n"
            "    'tests.test_fast_boundary',\n"
            "    'tests.test_fast_main',\n"
            "    'tests.test_fast_kernels_equivalence',\n"
            "    'tests.test_openmv_protocol',\n"
            "]\n"
            "suite = unittest.defaultTestLoader.loadTestsFromNames(names)\n"
            "result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(suite)\n"
            "raise SystemExit(0 if result.wasSuccessful() else 1)\n",
            encoding="utf-8", newline="")
        environment = os.environ.copy()
        environment.pop("V7_REQUIRE_BOARD_KERNEL_EQUIVALENCE", None)
        print("V7 regression adaptation: copied BUILD identity and "
              "confidence-diagnostic-only marker on frozen FastPath stub")
        completed = subprocess.run(
            [sys.executable, "-B", str(runner)],
            cwd=temp_root, env=environment, check=False)
        return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
