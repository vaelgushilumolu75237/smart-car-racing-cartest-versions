"""Integrity and paired-profile checks for the Ackermann 320x120 packages."""
import hashlib
import json
from pathlib import Path
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = (PROJECT_DIR / "deliverables" /
                "OpenMV_Ackermann320_20261022")
RUNTIME_NAMES = {
    "adaptive_detector.py", "boundary_evidence.py", "config.py",
    "contour_route.py", "contour_trace.py", "fast_boundary.py",
    "fast_gray.py", "fast_path.py", "fast_single_edge.py",
    "fast_topology.py", "grayscale_adapter.py", "line_detector.py",
    "main.py", "reverse_control.py", "road_detector.py",
    "source_selector.py", "surface_detector.py", "thresholding.py",
    "vision_protocol.py",
}
EXPECTED = {
    "geometry_perf_only": {
        "build_id": "OPENMV_ACKERMANN320_GEOMETRY_PERF_ONLY_20261022",
        "gain": 0.35,
        "limit": 100,
        "speed_stage": False,
    },
    "coordinated": {
        "build_id": "OPENMV_ACKERMANN320_COORDINATED_20261022",
        "gain": 0.45,
        "limit": 120,
        "speed_stage": True,
    },
}


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _verify_compiled_outputs(entries):
    validation_root = PROJECT_DIR / "test-data" / \
        "OpenMV_Ackermann320_20261023-validation"
    compiled_paths = []
    for entry in entries:
        compiled_relative = Path(entry["compiled_validation_path"])
        relative_text = compiled_relative.as_posix()
        prefix = "test-data/OpenMV_Ackermann320_20261023-validation/"
        if not relative_text.startswith(prefix):
            raise AssertionError(
                "compiled validation path is outside the named evidence tree: "
                + relative_text)
        if (compiled_relative.suffix != ".mpy" or
                len(entry["compiled_sha256"]) != 64 or
                entry["compiled_size_bytes"] <= 0):
            raise AssertionError("invalid compiled-output metadata: " +
                                 entry["name"])
        compiled_paths.append(PROJECT_DIR / compiled_relative)

    present = [path.is_file() for path in compiled_paths]
    if not any(present):
        if validation_root.exists():
            raise AssertionError(
                "compiled validation tree exists but its .mpy outputs are missing")
        return
    if not all(present):
        missing = [str(path) for path, exists in zip(compiled_paths, present)
                   if not exists]
        raise AssertionError("incomplete compiled validation outputs: " +
                             ", ".join(missing))
    for entry, compiled in zip(entries, compiled_paths):
        if compiled.stat().st_size != entry["compiled_size_bytes"]:
            raise AssertionError("compiled output size mismatch: " +
                                 entry["name"])
        if _sha256(compiled) != entry["compiled_sha256"]:
            raise AssertionError("compiled output hash mismatch: " +
                                 entry["name"])


class AckermannPackageTests(unittest.TestCase):
    def test_manifests_cover_allowlisted_runtime_and_cross_compile_evidence(self):
        if not all((PACKAGE_ROOT / profile /
                    "PACKAGE_MANIFEST.json").is_file()
                   for profile in EXPECTED):
            self.skipTest("Ackermann 320x120 package build has not run")
        package_runtime_hashes = {}
        for profile, expected in EXPECTED.items():
            with self.subTest(profile=profile):
                package_dir = PACKAGE_ROOT / profile
                manifest_path = package_dir / "PACKAGE_MANIFEST.json"
                readme_path = package_dir / "README.md"
                self.assertTrue(manifest_path.is_file())
                self.assertTrue(readme_path.is_file())
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                self.assertEqual(manifest["schema_version"], 1)
                self.assertEqual(manifest["control_profile"], profile)
                self.assertEqual(manifest["config_profile_selector"], profile)
                self.assertEqual(manifest["build_id"], expected["build_id"])
                self.assertEqual(manifest["package_id"], expected["build_id"])
                self.assertEqual(manifest["runtime_file_count"], 19)
                entries = manifest["runtime_files"]
                self.assertEqual(len(entries), 19)
                self.assertEqual({entry["name"] for entry in entries},
                                 RUNTIME_NAMES)
                self.assertEqual(manifest["cross_compile"]["target"],
                                 "armv7emsp")
                self.assertEqual(manifest["cross_compile"]["module_count"], 19)
                self.assertTrue(
                    manifest["cross_compile"]["all_exit_codes_zero"])
                self.assertEqual(manifest["protocol"]["version"], 2)
                self.assertEqual(manifest["protocol"]["frame_size_bytes"], 16)
                self.assertEqual(manifest["curve_trace"]["record_size_bytes"],
                                 16)
                self.assertEqual(
                    manifest["curve_trace"]["camera_diag_record_size_bytes"],
                    32)
                self.assertEqual(
                    manifest["curve_trace"]["control_ext_v2_record_size_bytes"],
                    32)
                self.assertEqual(
                    manifest["control"]["preview_max_gain"], expected["gain"])
                self.assertEqual(
                    manifest["control"]["preview_applied_limit"],
                    expected["limit"])
                self.assertEqual(
                    manifest["control"]["speed_stage_enabled"],
                    expected["speed_stage"])
                self.assertFalse(
                    manifest["validation"][
                        "hardware_full_pipeline_target_verified_for_this_package"])
                validation = manifest["validation"]
                self.assertEqual(
                    validation["host_suite_command"],
                    "python -m unittest discover -s openmv/tests -p 'test_*.py'")
                self.assertGreaterEqual(validation["host_suite_tests_run"], 200)
                self.assertEqual(
                    validation["host_suite_passed"] +
                    validation["host_suite_skipped"],
                    validation["host_suite_tests_run"])
                self.assertFalse(validation[
                    "historical_160x120_runner_is_current_acceptance"])
                readme = readme_path.read_text(encoding="utf-8")
                self.assertIn(validation["host_suite_command"], readme)
                self.assertIn(
                    "%d tests" % validation["host_suite_tests_run"], readme)
                self.assertIn("does not validate this package's 320x120/4x2",
                              readme)
                self.assertIn("fixed-point Q1000 table", readme)
                self.assertEqual(
                    {path.name for path in package_dir.glob("*.py")},
                    RUNTIME_NAMES)
                self.assertFalse(list(package_dir.glob("*.mpy")))
                config_text = (package_dir / "config.py").read_text(
                    encoding="utf-8")
                self.assertIn(
                    'STEERING_CONTROL_PROFILE = "%s"' % profile,
                    config_text)
                self.assertIn("PREVIEW_MAX_GAIN = %s" % expected["gain"],
                              config_text)
                self.assertIn("PREVIEW_APPLIED_LIMIT = %d" % expected["limit"],
                              config_text)
                self.assertIn("SPEED_STAGE_ENABLED = %s" %
                              ("True" if expected["speed_stage"] else "False"),
                              config_text)

                file_hashes = {}
                for entry in entries:
                    source = package_dir / entry["name"]
                    content = source.read_bytes()
                    self.assertEqual(len(content), entry["size_bytes"])
                    self.assertEqual(_sha256(source), entry["sha256"])
                    self.assertEqual(entry["mpy_cross_exit_code"], 0)
                    file_hashes[entry["name"]] = entry["sha256"]
                _verify_compiled_outputs(entries)
                package_runtime_hashes[profile] = file_hashes

        for name in RUNTIME_NAMES - {"config.py"}:
            self.assertEqual(package_runtime_hashes["geometry_perf_only"][name],
                             package_runtime_hashes["coordinated"][name], name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
