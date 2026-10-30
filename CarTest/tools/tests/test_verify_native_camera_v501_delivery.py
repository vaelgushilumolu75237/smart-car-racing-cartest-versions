from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools import verify_native_camera_v501_delivery as validator


PROJECT = Path(__file__).resolve().parents[2]


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def fake_image_pair(salt: int = 0) -> tuple[bytes, bytes]:
    firmware_bin = bytearray((0xFF ^ salt) for _ in range(64))
    struct.pack_into("<II", firmware_bin, 0, validator.VECTOR_MSP, 0x08040299)

    firmware_elf = bytearray(0x140)
    firmware_elf[:16] = b"\x7fELF\x01\x01\x01" + bytes(9)
    struct.pack_into("<HHII", firmware_elf, 16, 2, 40, 1, 0x08040299)
    struct.pack_into("<I", firmware_elf, 28, 52)  # e_phoff
    struct.pack_into("<H", firmware_elf, 40, 52)  # e_ehsize
    struct.pack_into("<HH", firmware_elf, 42, 32, 1)  # e_phentsize, e_phnum
    struct.pack_into(
        "<IIIIIIII", firmware_elf, 52,
        1, 0x100, validator.APP_START, validator.APP_START,
        len(firmware_bin), len(firmware_bin), 5, 0x1000,
    )
    firmware_elf[0x100:0x100 + len(firmware_bin)] = firmware_bin
    return bytes(firmware_bin), bytes(firmware_elf)


class DeliveryFixture:
    def __init__(self):
        self.temp = tempfile.TemporaryDirectory(prefix="openmv-v501-validator-")
        self.source = Path(self.temp.name)
        self.relative = Path("deliverables") / "OpenMVNativeFixture"
        self.package = self.source / self.relative
        self.rollback_bin, self.rollback_elf = fake_image_pair(7)
        self.candidate_bin, self.candidate_elf = fake_image_pair(11)
        self._copy_current_source_pins()
        self._copy_current_runtime()
        self._write_package()

    def close(self):
        self.temp.cleanup()

    def _copy_current_source_pins(self):
        vendor = PROJECT / validator.MODULE_SOURCE_ROOT
        target_vendor = self.source / validator.MODULE_SOURCE_ROOT
        for name in (*validator.PATCH_FILES, *validator.PATCH_PIN_FILES,
                     *validator.PATCH_AUX_SOURCEHASH_FILES):
            destination = target_vendor / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(vendor / name, destination)
        for logical in (*validator.MODULE_SOURCES, *validator.JPEG_GUARD_OVERLAY_SOURCES):
            destination = target_vendor / logical
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(vendor / logical, destination)

    def _copy_current_runtime(self):
        manifest_source = PROJECT / "openmv" / "PACKAGE_MANIFEST.json"
        manifest = json.loads(manifest_source.read_text(encoding="utf-8"))
        target_dir = self.source / "openmv"
        target_dir.mkdir(parents=True, exist_ok=True)
        runtime_names = [
            row if isinstance(row, str) else row.get("file")
            for row in manifest["runtime_files"]
        ]
        manifest["runtime_files"] = runtime_names
        write_json(target_dir / "PACKAGE_MANIFEST.json", manifest)
        for name in runtime_names:
            shutil.copyfile(PROJECT / "openmv" / name, target_dir / name)

    def copy_frozen_runtime_reference(self):
        relative = Path("deliverables") / "TrackProfilesFixture_v16" / "OpenMV"
        target_dir = self.source / relative
        target_dir.mkdir(parents=True)
        source_dir = self.source / "openmv"
        manifest = json.loads(
            (source_dir / "PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
        write_json(target_dir / "PACKAGE_MANIFEST.json", manifest)
        for name in manifest["runtime_files"]:
            shutil.copyfile(source_dir / name, target_dir / name)
        return relative

    def _write_package(self):
        self.package.mkdir(parents=True)
        (self.package / "rollback").mkdir()
        (self.package / "README.md").write_text(
            "Bench-only package. Road acceptance is pending; motor_start=false.\n",
            encoding="utf-8")
        (self.package / "firmware.bin").write_bytes(self.candidate_bin)
        (self.package / "firmware.elf").write_bytes(self.candidate_elf)
        (self.package / "rollback" / "firmware.bin").write_bytes(self.rollback_bin)
        (self.package / "rollback" / "firmware.elf").write_bytes(self.rollback_elf)

        patch_rows, module_sources, overlay_sources = validator._pin_patch_sources(self.source)
        source_manifest = {
            "schema_version": 1,
            "base": {
                "tag": validator.OPENMV_TAG,
                "openmv_commit": validator.OPENMV_COMMIT,
                "micropython_commit": validator.MICROPYTHON_COMMIT,
            },
            "target": validator.TARGET,
            "patch_series": patch_rows,
            "module_sources": module_sources,
            "jpeg_guard_overlay_sources": overlay_sources,
            "vehicle_logs": [],
            "calibration_files": [],
        }
        write_json(self.package / "source-manifest.json", source_manifest)

        runtime_manifest = json.loads(
            (self.source / "openmv" / "PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
        runtime_rows = []
        for name in runtime_manifest["runtime_files"]:
            content = (self.source / "openmv" / name).read_bytes()
            runtime_rows.append({
                "file": name,
                "sha256": sha256(content),
                "mpy_sha256": sha256(("fixture-mpy:" + name).encode()),
                "compile_exit_code": 0,
            })

        build_verification = {
            "artifact_class": "PRIVATE_OPENMV4P_FULL_CLEAN_BUILD_NOT_HARDWARE_VALIDATED",
            "deployment_eligible": False,
            "hardware_touched": False,
            "flash_performed": False,
            "source_commit": validator.OPENMV_COMMIT,
            "micropython_commit": validator.MICROPYTHON_COMMIT,
            "target": "OPENMV4P / STM32H743xx",
            "build": {
                "initial_file_count": 0,
                "runtime_object_count": 787,
                "dependency_file_count": 531,
                "exit_code": 0,
            },
            "patch_series": patch_rows,
            "module_sources": {
                Path(name).name: {"sha256": digest, "q_tree_sha256": digest}
                for name, digest in module_sources.items()
            },
            "jpeg_guard_overlay_sources": {
                ("ports/stm32/" + Path(name).name if "stm32" in name else "lib/imlib/" + Path(name).name):
                    {"sha256": digest, "q_tree_sha256": digest}
                for name, digest in overlay_sources.items()
            },
            "runtime_python_sources": runtime_rows,
            "generated_qstrs": sorted(validator.REQUIRED_QSTRS),
            "vectors": {
                "app_vector_base": "0x08040000",
                "bin_vector_msp": "0x00010000",
                "bin_vector_reset": "0x08040299",
                "elf_entry": "0x08040299",
                "app_load_vaddr": "0x08040000",
            },
            "artifacts": {
                "firmware.bin": {"bytes": len(self.candidate_bin), "sha256": sha256(self.candidate_bin)},
                "firmware.elf": {"bytes": len(self.candidate_elf), "sha256": sha256(self.candidate_elf)},
            },
            "verification": {
                "fresh_empty_build_directory": True,
                "full_qstr_generation": True,
                "hardware_validation": False,
                "make_exit_code": 0,
            },
        }
        write_json(self.package / "build-verification.json", build_verification)

        rollback_manifest = {
            "schema_version": 1,
            "target": validator.TARGET,
            "provenance": "confirmed-current-j-image",
            "files_sha256": {
                "firmware.bin": sha256(self.rollback_bin),
                "firmware.elf": sha256(self.rollback_elf),
            },
        }
        write_json(self.package / "rollback" / "manifest.json", rollback_manifest)

        file_hashes = {
            "firmware.bin": sha256(self.candidate_bin),
            "firmware.elf": sha256(self.candidate_elf),
            "source-manifest.json": sha256((self.package / "source-manifest.json").read_bytes()),
            "build-verification.json": sha256((self.package / "build-verification.json").read_bytes()),
            "rollback/firmware.bin": sha256(self.rollback_bin),
            "rollback/firmware.elf": sha256(self.rollback_elf),
            "rollback/manifest.json": sha256((self.package / "rollback" / "manifest.json").read_bytes()),
        }
        artifact_manifest = {
            "schema_version": 1,
            "package_id": self.package.name,
            "target": validator.TARGET,
            "artifact_class": "bench-only-road-pending",
            "application_base": "0x08040000",
            "application_end": "0x08200000",
            "motor_start": False,
            "road_acceptance": False,
            "deployment_eligible": False,
            "files_sha256": file_hashes,
            "evidence_refs": {
                "flash": {"sha256": "A" * 64, "result": "fixture only"},
                "readback": {"sha256": "B" * 64, "result": "fixture only"},
                "bench": {"sha256": "C" * 64, "result": "fixture only"},
            },
        }
        write_json(self.package / "artifact-manifest.json", artifact_manifest)
        self.refresh_sums()

    def refresh_sums(self):
        hashes = {}
        for path in sorted(self.package.rglob("*")):
            if path.is_file() and path.name != "SHA256SUMS.json":
                hashes[path.relative_to(self.package).as_posix()] = sha256(path.read_bytes())
        write_json(self.package / "SHA256SUMS.json", hashes)

    def refresh_artifact_hash(self, name: str):
        path = self.package / "artifact-manifest.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        if name in manifest["files_sha256"]:
            manifest["files_sha256"][name] = sha256((self.package / name).read_bytes())
        write_json(path, manifest)
        self.refresh_sums()


class VerifyNativeCameraV501DeliveryTests(unittest.TestCase):
    def setUp(self):
        self.fixture = DeliveryFixture()
        self.rollback_hash_patch = patch.multiple(
            validator,
            ROLLBACK_BIN_SHA256=sha256(self.fixture.rollback_bin),
            ROLLBACK_ELF_SHA256=sha256(self.fixture.rollback_elf),
        )
        self.rollback_hash_patch.start()
        self.addCleanup(self.rollback_hash_patch.stop)
        self.addCleanup(self.fixture.close)

    def test_valid_synthetic_elf_package_returns_exact_allowlist(self):
        result = validator.verify_native_camera_v501_delivery(
            self.fixture.source, self.fixture.relative)
        self.assertEqual(result, [
            f"{self.fixture.relative.as_posix()}/{name}"
            for name in validator.PUBLIC_NATIVE_V501_FILES
        ])

    def _use_software_only_manifest(self):
        path = self.fixture.package / "artifact-manifest.json"
        manifest = json.loads(path.read_text(encoding="utf8"))
        manifest.update(artifact_class="software-build-only-hardware-pending",
                        hardware_validation=False, flash_performed=False,
                        board_fps_verified=False, ide_preview_fps_verified=False,
                        rollback_scope="historical-j-reference-not-replacement-camera-backup",
                        evidence_refs={})
        write_json(path, manifest)
        self.fixture.refresh_sums()
        return path, manifest

    def test_software_only_build_requires_no_borrowed_device_evidence(self):
        self._use_software_only_manifest()
        result = validator.verify_native_camera_v501_delivery(
            self.fixture.source, self.fixture.relative)
        self.assertEqual(len(result), len(validator.PUBLIC_NATIVE_V501_FILES))

    def test_software_only_cannot_claim_device_acceptance(self):
        path, original = self._use_software_only_manifest()
        for field in ("hardware_validation", "flash_performed", "board_fps_verified",
                      "ide_preview_fps_verified"):
            for value in (True, None):
                with self.subTest(field=field, value=value):
                    manifest = dict(original, **{field: value})
                    write_json(path, manifest)
                    self.fixture.refresh_sums()
                    with self.assertRaisesRegex(RuntimeError, field):
                        validator.verify_native_camera_v501_delivery(
                            self.fixture.source, self.fixture.relative)

    def test_software_only_rejects_historical_evidence_as_current(self):
        path, manifest = self._use_software_only_manifest()
        manifest["evidence_refs"] = {"bench": {"sha256": "A" * 64, "result": "old bench"}}
        write_json(path, manifest)
        self.fixture.refresh_sums()
        with self.assertRaisesRegex(RuntimeError, "must not claim device evidence"):
            validator.verify_native_camera_v501_delivery(self.fixture.source, self.fixture.relative)

    def test_software_only_requires_explicit_historical_rollback_scope(self):
        path, manifest = self._use_software_only_manifest()
        del manifest["rollback_scope"]
        write_json(path, manifest)
        self.fixture.refresh_sums()
        with self.assertRaisesRegex(RuntimeError, "rollback scope"):
            validator.verify_native_camera_v501_delivery(self.fixture.source, self.fixture.relative)

    def _use_v16_object_row_runtime_manifest(self):
        manifest_path = self.fixture.source / "openmv" / "PACKAGE_MANIFEST.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        build_path = self.fixture.package / "build-verification.json"
        build = json.loads(build_path.read_text(encoding="utf-8"))
        runtime_rows = build["runtime_python_sources"]
        manifest["experiment"] = "pipeline_perf_v16_codec_guard"
        manifest["candidate_only"] = True
        manifest["hardware_validation"] = False
        manifest["deployment_eligible"] = False
        manifest["runtime_file_count"] = 23
        manifest["runtime_files"] = [
            {
                "file": row["file"],
                "candidate_sha256": row["sha256"],
                "mpy_sha256": row["mpy_sha256"],
                "compile_exit_code": row["compile_exit_code"],
            }
            for row in runtime_rows
        ]
        write_json(manifest_path, manifest)

    def test_v16_object_row_package_manifest_matches_build_rows(self):
        self._use_v16_object_row_runtime_manifest()
        result = validator.verify_native_camera_v501_delivery(
            self.fixture.source, self.fixture.relative)
        self.assertEqual(len(result), len(validator.PUBLIC_NATIVE_V501_FILES))

    def test_v16_object_row_package_manifest_hash_mismatch_is_rejected(self):
        self._use_v16_object_row_runtime_manifest()
        manifest_path = self.fixture.source / "openmv" / "PACKAGE_MANIFEST.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["runtime_files"][0]["candidate_sha256"] = "A" * 64
        write_json(manifest_path, manifest)
        with self.assertRaisesRegex(RuntimeError, "package-manifest source/MPY record"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_frozen_runtime_reference_keeps_default_strict_and_returns_34_files(self):
        runtime_relative = self.fixture.copy_frozen_runtime_reference()
        live_main = self.fixture.source / "openmv" / "main.py"
        live_main.write_bytes(live_main.read_bytes() + b"# changed live runtime\n")

        with self.assertRaisesRegex(RuntimeError, "differs from live OpenMV"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

        result = validator.verify_native_camera_v501_delivery(
            self.fixture.source, self.fixture.relative,
            runtime_root=runtime_relative)
        expected = [
            f"{self.fixture.relative.as_posix()}/{name}"
            for name in validator.PUBLIC_NATIVE_V501_FILES
        ]
        expected.append(
            f"{runtime_relative.as_posix()}/PACKAGE_MANIFEST.json")
        expected.extend(
            f"{runtime_relative.as_posix()}/{name}"
            for name in json.loads(
                ((self.fixture.source / runtime_relative) /
                 "PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))["runtime_files"]
        )
        self.assertEqual(result, expected)
        self.assertEqual(len(result), 34)

    def test_tampered_frozen_runtime_source_is_rejected(self):
        runtime_relative = self.fixture.copy_frozen_runtime_reference()
        frozen_main = self.fixture.source / runtime_relative / "main.py"
        frozen_main.write_bytes(frozen_main.read_bytes() + b"# tampered\n")
        with self.assertRaisesRegex(
                RuntimeError, "differs from frozen OpenMV runtime reference"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative,
                runtime_root=runtime_relative)

    def test_tampered_frozen_runtime_manifest_is_rejected(self):
        runtime_relative = self.fixture.copy_frozen_runtime_reference()
        manifest_path = self.fixture.source / runtime_relative / "PACKAGE_MANIFEST.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["runtime_files"].pop()
        write_json(manifest_path, manifest)
        with self.assertRaisesRegex(
                RuntimeError,
                "frozen OpenMV runtime reference manifest must enumerate exactly 23"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative,
                runtime_root=runtime_relative)

    def test_unsafe_frozen_runtime_reference_paths_are_rejected(self):
        for runtime_root in ("deliverables/../openmv", "openmv"):
            with self.subTest(runtime_root=runtime_root), self.assertRaisesRegex(
                    RuntimeError, "runtime root"):
                validator.verify_native_camera_v501_delivery(
                    self.fixture.source, self.fixture.relative,
                    runtime_root=runtime_root)

    def test_symlinked_frozen_runtime_reference_is_rejected(self):
        runtime_relative = self.fixture.copy_frozen_runtime_reference()
        linked_root = self.fixture.source / "deliverables" / "LinkedRuntime"
        real_is_symlink = Path.is_symlink

        def simulated_symlink(path):
            return path == linked_root or real_is_symlink(path)

        with patch.object(Path, "is_symlink", simulated_symlink):
            with self.assertRaisesRegex(RuntimeError, "runtime root contains a symlink"):
                validator.verify_native_camera_v501_delivery(
                    self.fixture.source, self.fixture.relative,
                    runtime_root=Path("deliverables") / "LinkedRuntime")

    def test_extra_file_is_rejected(self):
        (self.fixture.package / "unexpected.log").write_text("private", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "allowlist mismatch"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_sha256_sums_mismatch_is_rejected(self):
        (self.fixture.package / "README.md").write_text("changed after manifest\n", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "SHA256SUMS.json digest mismatch: README.md"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_path_escape_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "safe source-relative"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, "deliverables/../outside")

    def test_candidate_load_segment_outside_app_is_rejected(self):
        elf_path = self.fixture.package / "firmware.elf"
        elf = bytearray(elf_path.read_bytes())
        struct.pack_into("<I", elf, 52 + 12, validator.APP_END)
        elf_path.write_bytes(elf)
        self.fixture.refresh_sums()
        with self.assertRaisesRegex(RuntimeError, "outside app flash"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_candidate_load_bytes_must_match_bin(self):
        elf_path = self.fixture.package / "firmware.elf"
        elf = bytearray(elf_path.read_bytes())
        elf[0x100 + 20] ^= 1
        elf_path.write_bytes(elf)
        self.fixture.refresh_sums()
        with self.assertRaisesRegex(RuntimeError, "bytes differ from ELF PT_LOAD"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_missing_codec_qstr_is_rejected(self):
        path = self.fixture.package / "build-verification.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["generated_qstrs"].remove("jpeg_encode_timeout_ms")
        write_json(path, data)
        self.fixture.refresh_artifact_hash("build-verification.json")
        with self.assertRaisesRegex(RuntimeError, "QSTR list is missing"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_nonzero_build_exit_is_rejected(self):
        path = self.fixture.package / "build-verification.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["build"]["exit_code"] = 1
        write_json(path, data)
        self.fixture.refresh_artifact_hash("build-verification.json")
        with self.assertRaisesRegex(RuntimeError, "build.exit_code=0"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_motor_start_claim_is_rejected(self):
        path = self.fixture.package / "artifact-manifest.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["motor_start"] = True
        write_json(path, data)
        self.fixture.refresh_sums()
        with self.assertRaisesRegex(RuntimeError, "motor_start=false"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_source_manifest_must_match_current_patch(self):
        path = self.fixture.package / "source-manifest.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        data["patch_series"][4]["sha256"] = "0" * 64
        write_json(path, data)
        self.fixture.refresh_artifact_hash("source-manifest.json")
        with self.assertRaisesRegex(RuntimeError, "source-manifest patch_series"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_rollback_pair_hash_is_pinned(self):
        path = self.fixture.package / "rollback" / "firmware.bin"
        path.write_bytes(path.read_bytes()[:-1] + b"\x00")
        self.fixture.refresh_artifact_hash("rollback/firmware.bin")
        with self.assertRaisesRegex(RuntimeError, "confirmed current v5.0.1 image"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)

    def test_symlink_is_rejected_when_supported(self):
        link = self.fixture.package / "README.md"
        link.unlink()
        target = self.fixture.package / "firmware.bin"
        try:
            os.symlink(target, link)
        except (OSError, NotImplementedError) as exc:
            self.skipTest(f"symlink creation unavailable: {type(exc).__name__}")
        with self.assertRaisesRegex(RuntimeError, "symlink"):
            validator.verify_native_camera_v501_delivery(
                self.fixture.source, self.fixture.relative)


if __name__ == "__main__":
    unittest.main()
