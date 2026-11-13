"""Regression checks for historical SpeedPD checkpoint verification."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PROJECT_ROOT / "tools" / "checkpoint_version.py"
_SPEC = importlib.util.spec_from_file_location("checkpoint_version", TOOL_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("checkpoint_version.py is unavailable")
checkpoint_version = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(checkpoint_version)


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _write_speedpd_tree(root: Path) -> tuple[dict, dict]:
    package_root = root / "deliverables" / "H743_SpeedPD_20261020"
    source_hashes = {"Src/main.c": "a" * 64}
    artifact_manifest = {
        "schema_version": 1,
        "package": "H743_SpeedPD_20261020",
        "build_summary": checkpoint_version.SPEEDPD_BUILD_SUMMARY,
        "artifacts": {},
        "per_variant_manifests": {},
    }
    summary = {
        "schema_version": 1,
        "package": "H743_SpeedPD_20261020",
        "source_inputs_stable": True,
        "source_sha256": source_hashes,
        "artifacts": {},
        "builds": {},
    }

    for variant, spec in checkpoint_version.SPEEDPD_PROFILE_DELIVERIES.items():
        profile = spec["profile"]
        payload = ("frozen SpeedPD " + variant).encode("ascii")
        digest = hashlib.sha256(payload).hexdigest()
        build_id = "frozen-build-" + variant
        delivery_path = root / spec["elf"]
        delivery_path.parent.mkdir(parents=True, exist_ok=True)
        delivery_path.write_bytes(payload)
        variant_manifest_path = root / spec["manifest"]
        _write_json(variant_manifest_path, {
            "profile": profile,
            "size_bytes": len(payload),
            "sha256": digest,
            "build_id": build_id,
            "delivery_path": spec["elf"],
            "export_elf": spec["elf"],
            "source_sha256": source_hashes,
        })
        artifact = {
            "profile": profile,
            "delivery_path": spec["elf"],
            "export_elf": spec["elf"],
            "sha256": digest,
            "build_id": build_id,
        }
        artifact_manifest["artifacts"][variant] = artifact
        summary["artifacts"][variant] = artifact
        artifact_manifest["per_variant_manifests"][variant] = spec["manifest"]
        summary["builds"][profile] = {
            "profile": profile,
            "elf": f"build/{profile}/CarTest.elf",
            "exit_code": 0,
            "sha256": digest,
            "size_bytes": len(payload),
            "build_id": build_id,
        }

        # Current same-named outputs may differ from this frozen package.
        current_build = root / "build" / profile / "CarTest.elf"
        current_build.parent.mkdir(parents=True, exist_ok=True)
        current_build.write_bytes(b"later build with the same profile name")

    _write_json(root / checkpoint_version.SPEEDPD_ARTIFACT_MANIFEST,
                artifact_manifest)
    _write_json(root / checkpoint_version.SPEEDPD_BUILD_SUMMARY, summary)
    return artifact_manifest, summary


class FrozenSpeedPDCheckpointTests(unittest.TestCase):
    def test_historical_delivery_ignores_different_current_profile_builds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_speedpd_tree(root)
            checkpoint_version.verify_speedpd_deliveries(root)

    def test_frozen_build_summary_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, summary = _write_speedpd_tree(root)
            first_profile = next(iter(summary["builds"]))
            summary["builds"][first_profile]["sha256"] = "0" * 64
            _write_json(root / checkpoint_version.SPEEDPD_BUILD_SUMMARY,
                        summary)
            with self.assertRaisesRegex(RuntimeError,
                                        "Frozen SpeedPD build summary"):
                checkpoint_version.verify_speedpd_deliveries(root)


if __name__ == "__main__":
    unittest.main(verbosity=2)
