#!/usr/bin/env python3
"""Static contract checks for the frozen OpenMV v5.0.0 vendor patch."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCH = ROOT / "tools" / "vendor-patches" / "openmv-v5.0.0" / (
    "0001-csi-snapshot-preview-kwarg.patch")
MANIFEST = PATCH.with_name("manifest.json")
NO_UPDATE = 1 << 0  # OMV_CSI_FLAG_NO_UPDATE in the pinned v5.0.0 header.


def _patch_text() -> str:
    return PATCH.read_text(encoding="utf-8")


class PreviewPatchContractTests(unittest.TestCase):
    def test_patch_hash_matches_manifest(self) -> None:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        actual = hashlib.sha256(PATCH.read_bytes()).hexdigest().upper()
        self.assertEqual(actual, manifest["patch_sha256"])

    def test_both_snapshot_apis_are_covered(self) -> None:
        text = _patch_text()
        self.assertIn("a/modules/py_csi.c", text)
        self.assertIn("b/modules/py_csi.c", text)
        self.assertIn("a/modules/py_csi_ng.c", text)
        self.assertIn("b/modules/py_csi_ng.c", text)

    def test_both_modules_export_capability_marker(self) -> None:
        text = _patch_text()
        self.assertEqual(text.count("MP_QSTR_PREVIEW_CONTROL_API"), 2)
        self.assertEqual(text.count("MP_ROM_INT(1)"), 2)

    def test_legacy_flag_is_default_on_and_false_suppresses(self) -> None:
        text = _patch_text()
        self.assertIn("MP_QSTR_PREVIEW_CONTROL_API", text)
        self.assertIn("MP_QSTR_preview", text)
        self.assertIn("MP_OBJ_NEW_QSTR(MP_QSTR_preview)", text)
        self.assertNotIn("MP_ROM_QSTR(MP_QSTR_preview)", text)
        self.assertIn("!mp_obj_is_true(preview_kw->value)", text)
        self.assertIn("flags |= OMV_CSI_FLAG_NO_UPDATE", text)
        self.assertIn("py_image_cobj(image), flags", text)

    def test_csi_flag_is_default_on_and_false_suppresses(self) -> None:
        text = _patch_text()
        self.assertIn("ARG_preview", text)
        self.assertIn("MP_QSTR_preview, MP_ARG_BOOL | MP_ARG_KW_ONLY", text)
        self.assertIn(".u_bool = true", text)
        self.assertIn("if (!args[ARG_preview].u_bool)", text)
        self.assertIn("OMV_CSI_FLAG_NO_UPDATE", text)

    def test_probe_requires_both_capability_markers(self) -> None:
        probe = (ROOT / "openmv" / "tests" /
                 "ram_usb_preview_probe.py").read_text(encoding="utf-8")
        self.assertIn('getattr(sensor, "PREVIEW_CONTROL_API", None) != 1', probe)
        self.assertIn('getattr(csi, "PREVIEW_CONTROL_API", None) != 1', probe)
        self.assertIn("PREVIEW_EVERY_N = 4", probe)
        self.assertIn("SCALE = 0.25", probe)

    def test_flag_contract_preserves_default_and_suppresses_false(self) -> None:
        def flags(preview: bool = True) -> int:
            return 0 if preview else NO_UPDATE

        self.assertEqual(flags(), 0)
        self.assertEqual(flags(True), 0)
        self.assertEqual(flags(False), NO_UPDATE)


def _check_apply(source_root: pathlib.Path) -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    patch_hash = hashlib.sha256(PATCH.read_bytes()).hexdigest().upper()
    if patch_hash != manifest["patch_sha256"]:
        raise SystemExit(f"patch hash mismatch: {patch_hash}")
    expected = manifest["base_sha256"]
    paths = tuple(expected)
    for relative in paths:
        actual = hashlib.sha256((source_root / relative).read_bytes()).hexdigest().upper()
        if actual != expected[relative]:
            raise SystemExit(f"base hash mismatch for {relative}: {actual}")

    header = subprocess.run(
        ["git", "show", f"{manifest['commit']}:common/omv_csi.h"],
        cwd=source_root,
        capture_output=True,
        text=True,
        check=False,
    )
    if (header.returncode or not re.search(
            r"OMV_CSI_FLAG_NO_UPDATE\s*=\s*\(1\s*<<\s*0\)",
            header.stdout)):
        raise SystemExit("pinned NO_UPDATE flag value is not 1 << 0")

    micro_python_tree = subprocess.run(
        ["git", "ls-tree", manifest["commit"], "lib/micropython"],
        cwd=source_root,
        capture_output=True,
        text=True,
        check=False,
    )
    if (micro_python_tree.returncode or
            manifest["micropython_commit"] not in micro_python_tree.stdout):
        raise SystemExit("OpenMV base does not pin the expected MicroPython commit")

    with tempfile.TemporaryDirectory(prefix="openmv-preview-patch-") as temp:
        work = pathlib.Path(temp)
        for relative in paths:
            destination = work / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_root / relative, destination)
        for check_only in (True, False):
            command = ["git", "apply"]
            if check_only:
                command.append("--check")
            command.append(str(PATCH))
            result = subprocess.run(
                command,
                cwd=work,
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode:
                raise SystemExit(result.stderr or result.stdout or "git apply failed")

        legacy = (work / "modules/py_csi.c").read_text(encoding="utf-8")
        csi = (work / "modules/py_csi_ng.c").read_text(encoding="utf-8")
        if ("flags |= OMV_CSI_FLAG_NO_UPDATE" not in legacy or
                "py_image_cobj(image), flags" not in legacy):
            raise SystemExit("legacy sensor.snapshot preview flag was not applied")
        if ("MP_QSTR_preview, MP_ARG_BOOL | MP_ARG_KW_ONLY" not in csi or
                ".u_bool = true" not in csi or
                "if (!args[ARG_preview].u_bool)" not in csi):
            raise SystemExit("CSI.snapshot preview argument was not applied")
        if "omv_csi_snapshot(self->csi, &image, flags)" not in csi:
            raise SystemExit("CSI.snapshot does not pass flags to capture")
        for module, source in (("sensor", legacy), ("csi", csi)):
            if ("MP_QSTR_PREVIEW_CONTROL_API" not in source or
                    "MP_ROM_INT(1)" not in source):
                raise SystemExit(
                    f"{module} module is missing PREVIEW_CONTROL_API=1")

    print("pinned base hashes, NO_UPDATE header, patch apply, and source semantics: PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=pathlib.Path)
    args = parser.parse_args()
    if args.source_root is not None:
        _check_apply(args.source_root.resolve())
    unittest.main(argv=[__file__])
