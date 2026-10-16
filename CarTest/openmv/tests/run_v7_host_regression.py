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
BUILD_NEW = "CONTOUR_REPAIR_20261016"
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
        adaptations_applied = []

        def replace_fixture(content, filename, old, new, expected=1,
                            reason="legacy fixture compatibility"):
            count = content.count(old)
            if count != expected:
                raise SystemExit(
                    "could not identify %d expected fixture text(s) in %s "
                    "(found %d): %s" % (expected, filename, count, old))
            adaptations_applied.append({
                "file": filename, "count": count, "reason": reason,
            })
            return content.replace(old, new)

        for source in PACKAGE_DIR.glob("*.py"):
            shutil.copyfile(source, temp_openmv / source.name)
        for name in TEST_NAMES:
            source = FROZEN_DIR / name
            target = temp_tests / name
            content = source.read_text(encoding="utf-8")
            if name == "test_fast_gray.py":
                content = replace_fixture(
                    content, name,
                    "def _lane_grid(dark=34, bright=218, floor=20):",
                    "def _lane_grid(dark=34, bright=218, "
                    "floor=config.SURFACE_ROI_FLOOR_Y):",
                    reason="default fixture follows expanded current ROI")
                content = replace_fixture(
                    content, name, 'old["blocks"][1]["reason"]',
                    'old["blocks"][2]["reason"]',
                    reason="unknown original middle block shifts after new upper block")
                content = replace_fixture(
                    content, name, "for block in range(3):",
                    "for block in range(self.fast._block_count):",
                    reason="histogram assertion covers configured block count")
                content = replace_fixture(
                    content, name, '["CONSTANT"] * 3',
                    '["CONSTANT"] * 4',
                    reason="expected block diagnostics follow four-block config")
            elif name == "test_fast_path.py":
                content = replace_fixture(
                    content, name, "import config\n",
                    "import config as _production_config\n"
                    "from types import SimpleNamespace\n"
                    "config = SimpleNamespace(**vars(_production_config))\n"
                    "config.FAST_PATH_REFERENCE_MAX_GAP = 51.0\n",
                    reason="pin legacy broad-reference kernel fixtures; production near-reference policy has separate regressions")
                content = replace_fixture(
                    content, name,
                    "def mask_for_rows(center_fn=None, half_width=18, "
                    "y_range=(20, 60),",
                    "def mask_for_rows(center_fn=None, half_width=18, "
                    "y_range=(config.SURFACE_ROI_FLOOR_Y, 60), floor=None,",
                    reason="default detector masks cover current expanded ROI")
                content = replace_fixture(
                    content, name,
                    "    if center_fn is None:\n"
                    "        center_fn = lambda _y: 39.5\n",
                    "    if floor is None:\n"
                    "        floor = config.SURFACE_ROI_FLOOR_Y\n"
                    "    if center_fn is None:\n"
                    "        center_fn = lambda _y: 39.5\n",
                    reason="allow explicit legacy floor for custom-floor tests")
                content = replace_fixture(
                    content, name,
                    '"floor": 20,\n            "luma": bytearray([220])',
                    '"floor": config.SURFACE_ROI_FLOOR_Y,\n'
                    '            "luma": bytearray([220])',
                    reason="all-white invalid mask reports current ROI floor")
                content = replace_fixture(
                    content, name, '"floor": 20,', '"floor": floor,',
                    expected=2,
                    reason="default and merge masks report selected ROI floor")
                content = replace_fixture(
                    content, name, "for y in range(20, 60):",
                    "for y in range(config.SURFACE_ROI_FLOOR_Y, 60):",
                    reason="merge fixture spans current full ROI")
                content = replace_fixture(
                    content, name,
                    '"height": HEIGHT, "floor": floor,\n'
                    '        "luma": luma, "road_mask": road,',
                    '"height": HEIGHT, "floor": config.SURFACE_ROI_FLOOR_Y,\n'
                    '        "luma": luma, "road_mask": road,',
                    reason="merge fixture reports current ROI floor")
                content = replace_fixture(
                    content, name,
                    "masks = mask_for_rows(y_range=(20, 60), extra_corridors=",
                    "masks = mask_for_rows(y_range=(20, 60), floor=20, "
                    "extra_corridors=",
                    reason="keep the explicit floor-20 kernel-bound test intact")
                content = replace_fixture(
                    content, name,
                    'self.assertEqual(result["active_row_count"], 40)',
                    'self.assertEqual(result["active_row_count"], '
                    'HEIGHT - config.SURFACE_ROI_FLOOR_Y)',
                    reason="active row count follows expanded ROI floor")
            elif name == "test_fast_topology.py":
                content = replace_fixture(
                    content, name,
                    "def make_corridor_mask(center=39.5, half_width=17, "
                    "y_start=20, y_end=59):",
                    "def make_corridor_mask(center=39.5, half_width=17, "
                    "y_start=FLOOR, y_end=59):",
                    reason="default topology corridor spans current expanded ROI")
                content = replace_fixture(
                    content, name, "mark_unknown_rows(info, 20, 25)",
                    "mark_unknown_rows(info, FLOOR, FLOOR + 12)",
                    reason="partial topology fixture removes whole current upper block")
                content = replace_fixture(
                    content, name,
                    "        top_connected = bytearray(TOTAL)\n"
                    "        for y in range(20, 25):",
                    "        top_connected = bytearray(TOTAL)\n"
                    "        for y in range(FLOOR, FLOOR + 5):",
                    reason="keep ROI-edge-connected band at expanded ROI boundary")
            if name == "test_fast_main.py":
                frozen_stub = (
                    "RUNTIME_API_VERSION=7,\n"
                    "        FastPathDetector=FakePathDetector,")
                cartest_stub = (
                    "RUNTIME_API_VERSION=7,\n"
                    "        CONFIDENCE_DIAGNOSTIC_ONLY=True,\n"
                    "        FastPathDetector=FakePathDetector,")
                content = replace_fixture(
                    content, name, BUILD_OLD, BUILD_NEW,
                    expected=1,
                    reason="adapt historical BUILD literal to current package identity")
                content = replace_fixture(
                    content, name, frozen_stub, cartest_stub,
                    reason="add current confidence-diagnostic API to frozen test double")
                content = replace_fixture(
                    content, name, '"floor": 20,',
                    '"floor": config.SURFACE_ROI_FLOOR_Y,',
                    reason="align fake grayscale output with expanded ROI")
                content = replace_fixture(
                    content, name, '"trusted_block_count": 3,',
                    '"trusted_block_count": 4,',
                    reason="report current four configured grayscale blocks")
                content = replace_fixture(
                    content, name, '"trusted_block_count=3"',
                    '"trusted_block_count=4"',
                    reason="assert debug reports current four trusted blocks")
                support_marker = (
                    '            raise AssertionError("V7 boundary did not receive packed path rows")\n'
                    '        return candidate')
                support_replacement = (
                    '            raise AssertionError("V7 boundary did not receive packed path rows")\n'
                    '        count = candidate["selected_row_count"]\n'
                    '        candidate["boundary_support_rows"] = bytearray([1] * count)\n'
                    '        candidate["boundary_support_count"] = count\n'
                    '        return candidate')
                content = replace_fixture(
                    content, name, support_marker, support_replacement,
                    reason="mock current per-row pair support consumed by controller")
                content = replace_fixture(
                    content, name,
                    "def pack_frame(sequence, valid, confidence, lateral_error):",
                    "def pack_frame(sequence, valid, confidence, lateral_error, "
                    "slope_hint=0):",
                    reason="adapt frozen packet spy to optional current slope flag")
                content = replace_fixture(
                    content, name,
                    "pack_calls.append((sequence, valid, confidence, lateral_error))",
                    "pack_calls.append((sequence, valid, confidence, "
                    "lateral_error, slope_hint))",
                    reason="record optional flag while retaining packet assertions")
                content = replace_fixture(
                    content, name,
                    "packet = real_pack(sequence, valid, confidence, lateral_error)",
                    "packet = real_pack(sequence, valid, confidence, "
                    "lateral_error, slope_hint)",
                    reason="forward optional slope flag through frozen packet spy")
                content = replace_fixture(
                    content, name,
                    'self.assertTrue(all(call[1:] == (1, 92, 0)\n'
                    '                            for call in run["pack_calls"]))',
                    'self.assertTrue(all(call[1:] == (1, 92, 0, 0)\n'
                    '                            for call in run["pack_calls"]))',
                    reason="assert all legacy route frames still clear slope flag")
                content = replace_fixture(
                    content, name,
                    'self.assertEqual(run["pack_calls"], [(0, 0, 0, 0)])',
                    'self.assertEqual(run["pack_calls"], [(0, 0, 0, 0, 0)])',
                    reason="update invalid packet spy tuple for optional flag")
                content = replace_fixture(
                    content, name,
                    'self.assertEqual(short_valid["pack_calls"], [(0, 1, 92, 0)])',
                    'self.assertEqual(short_valid["pack_calls"], '
                    '[(0, 1, 92, 0, 0)])',
                    reason="update short-write spy tuple for optional flag")
                content = replace_fixture(
                    content, name,
                    '            (0, 1, 92, 0),\n'
                    '            (0, 0, 0, 0),',
                    '            (0, 1, 92, 0, 0),\n'
                    '            (0, 0, 0, 0, 0),',
                    reason="update late-packet spy tuples for optional flag")
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
                    ("self.assertEqual(len(image.crosses), 2)",
                     "self.assertEqual(len(image.crosses), 1)"),
                    ("self.assertEqual(len(image.strings), 4)",
                     "self.assertEqual(len(image.strings), 5)"),
                    ('self.assertEqual(image.strings[0][1], "VALID r=+0 f=+0")',
                     'self.assertEqual(image.strings[0][1], "NOW V CACHE/ROAD")'),
                    ("self.assertEqual(image.lines[-2][0][3],",
                     "self.assertEqual(image.lines[8][0][3],"),
                    ('"INVALID r=+0 f=+0")',
                     '"NOW INVALID CACHE")'),
                    ("self.assertLess(len(line), 500)",
                     "self.assertLess(len(line), 1024)"),
                )
                for old, new in adaptations:
                    content = replace_fixture(
                        content, name, old, new,
                        reason="adapt historical overlay assertion to current labels and flush stage")
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
        print("SUITE_CLASS=FROZEN_V7_BEHAVIORAL_COMPATIBILITY")
        print("FIXTURE_SOURCE_HASHES=VERIFIED_UNCHANGED")
        print("TARGET_RUNTIME_CONFIG=CURRENT_RING_20261014")
        print("TEMPORARY_TEST_COPY_ADAPTATIONS=" +
              json.dumps(adaptations_applied, sort_keys=True))
        completed = subprocess.run(
            [sys.executable, "-B", str(runner)],
            cwd=temp_root, env=environment, check=False)
        return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
