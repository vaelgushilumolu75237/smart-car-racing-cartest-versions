"""Integrity checks for the three complete OpenMV upload profiles."""

import hashlib
import json
from pathlib import Path
import unittest

from tools.build_openmv_ackermann320_packages import RUNTIME_NAMES as ACKERMANN_RUNTIME_NAMES


PROJECT_DIR = Path(__file__).resolve().parents[2]
OPENMV_DIR = PROJECT_DIR / "openmv"
PACKAGE_ROOT = (PROJECT_DIR / "deliverables" /
                "OpenMV_Bottom80_Steering_20261021")
EXPECTED = {
    "no_pd": "BOTTOM80_NO_PD_20261020",
    "pd": "BOTTOM80_POSPD_20261020",
    "pd_preview": "BOTTOM80_PD_PREVIEW_20261020",
}


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _verify_compiled_outputs(entries):
    validation_root = PROJECT_DIR / "test-data" / \
        "speedpd-20261020-validation" / "camera" / "mpy-cross-armv7emsp"
    compiled_paths = []
    for item in entries:
        compiled_relative = Path(item["compiled_validation_path"])
        relative_text = compiled_relative.as_posix()
        self_prefix = (
            "test-data/speedpd-20261020-validation/camera/"
            "mpy-cross-armv7emsp/")
        if not relative_text.startswith(self_prefix):
            raise AssertionError(
                "compiled validation path is outside the named evidence tree: "
                + relative_text)
        if (compiled_relative.suffix != ".mpy" or
                len(item["compiled_sha256"]) != 64 or
                item["compiled_size_bytes"] <= 0):
            raise AssertionError("invalid compiled-output metadata: " +
                                 item["name"])
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
    for item, compiled in zip(entries, compiled_paths):
        if compiled.stat().st_size != item["compiled_size_bytes"]:
            raise AssertionError("compiled output size mismatch: " +
                                 item["name"])
        if _sha256(compiled) != item["compiled_sha256"]:
            raise AssertionError("compiled output hash mismatch: " +
                                 item["name"])


class ProfilePackageTests(unittest.TestCase):
    def test_all_three_manifests_cover_complete_profile_matched_sources(self):
        # This verifies an immutable 20261021 package. New live modules must
        # neither change its historical allowlist nor force it to be rebuilt.
        source_names = sorted(tuple(ACKERMANN_RUNTIME_NAMES) + ("fast_ring_slope.py",))
        self.assertEqual(len(source_names), 20)
        manifests = {}
        for profile, build_id in EXPECTED.items():
            package_dir = PACKAGE_ROOT / profile
            manifest_path = package_dir / "PACKAGE_MANIFEST.json"
            self.assertTrue(manifest_path.is_file(), profile)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifests[profile] = manifest
            self.assertEqual(manifest["control_profile"], profile)
            self.assertEqual(manifest["package_id"], build_id)
            self.assertEqual(manifest["build_id"], build_id)
            self.assertEqual(manifest["runtime_file_count"], 20)
            self.assertEqual(
                sorted(item["name"] for item in manifest["runtime_files"]),
                source_names)
            self.assertEqual(
                manifest["cross_compile"]["target"], "armv7emsp")
            self.assertEqual(manifest["cross_compile"]["module_count"], 20)
            self.assertTrue(
                manifest["cross_compile"]["all_exit_codes_zero"])
            self.assertFalse(
                manifest["hardware_validation"]["openmv_uploaded"])
            self.assertFalse(
                manifest["hardware_validation"]["production_45fps_verified"])

            config_text = (package_dir / "config.py").read_text(
                encoding="utf-8")
            self.assertEqual(
                config_text.count('STEERING_CONTROL_PROFILE = "%s"' % profile),
                1)
            self.assertEqual(len(list(package_dir.glob("*.py"))), 20)
            self.assertFalse(list(package_dir.glob("*.mpy")))
            for item in manifest["runtime_files"]:
                source = package_dir / item["name"]
                self.assertEqual(source.stat().st_size, item["size_bytes"])
                self.assertEqual(_sha256(source), item["sha256"])
                self.assertEqual(item["mpy_cross_exit_code"], 0)
            _verify_compiled_outputs(manifest["runtime_files"])

    def test_profile_identity_is_unique_and_other_runtime_sources_are_shared(self):
        manifests = {}
        for profile in EXPECTED:
            path = PACKAGE_ROOT / profile / "PACKAGE_MANIFEST.json"
            manifests[profile] = json.loads(path.read_text(encoding="utf-8"))
        ids = [manifest["package_id"] for manifest in manifests.values()]
        self.assertEqual(len(ids), len(set(ids)))
        file_hashes = {
            profile: {item["name"]: item["sha256"]
                      for item in manifest["runtime_files"]}
            for profile, manifest in manifests.items()
        }
        for name in file_hashes["pd"]:
            if name == "config.py":
                self.assertEqual(len({values[name]
                                      for values in file_hashes.values()}), 3)
            else:
                self.assertEqual(len({values[name]
                                      for values in file_hashes.values()}), 1,
                                 name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
