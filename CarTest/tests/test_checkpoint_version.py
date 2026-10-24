import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


PROJECT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "checkpoint_version", PROJECT / "tools" / "checkpoint_version.py")
checkpoint = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(checkpoint)


def write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


class CheckpointAllowlistTests(unittest.TestCase):
    def test_public_copy_excludes_only_tools_experiments_subtree(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            destination = root / "destination"
            (source / "tools" / "experiments").mkdir(parents=True)
            (source / "tools" / "nested" / "experiments").mkdir(parents=True)
            (source / "openmv" / "experiments").mkdir(parents=True)
            (source / "tools" / "experiments" / "private.py").write_text(
                "private\n", encoding="utf-8")
            (source / "tools" / "nested" / "experiments" / "keep.py").write_text(
                "keep nested\n", encoding="utf-8")
            (source / "openmv" / "experiments" / "keep.py").write_text(
                "keep openmv\n", encoding="utf-8")

            for directory in ("tools", "openmv"):
                checkpoint.copy_tree(
                    source / directory,
                    destination / directory,
                    excluded_subtrees=checkpoint.PUBLIC_COPY_EXCLUDED_SUBTREES.get(
                        directory, ()),
                )

            self.assertFalse((destination / "tools" / "experiments").exists())
            self.assertEqual(
                (destination / "tools" / "nested" / "experiments" / "keep.py").read_text(
                    encoding="utf-8"),
                "keep nested\n",
            )
            self.assertEqual(
                (destination / "openmv" / "experiments" / "keep.py").read_text(
                    encoding="utf-8"),
                "keep openmv\n",
            )

    def make_native_preview_fixture(self, source: Path):
        for relative_name in checkpoint.OPENMV_NATIVE_PREVIEW_REQUIRED_FILES:
            source_path = PROJECT / relative_name
            destination = source / relative_name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(source_path.read_bytes())
        patch_path = source / checkpoint.OPENMV_NATIVE_PREVIEW_PATCH
        patch_path.parent.mkdir(parents=True, exist_ok=True)
        patch_path.write_bytes((PROJECT / checkpoint.OPENMV_NATIVE_PREVIEW_PATCH).read_bytes())

    def make_usb60_fixture(self, source: Path):
        for relative_name in (
                f"{checkpoint.OPENMV_ACKERMANN_ROOT}/coordinated/PACKAGE_MANIFEST.json",
                *checkpoint.OPENMV_USB60_BASELINE_H743_ELFS):
            source_path = PROJECT / relative_name
            destination = source / relative_name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(source_path.read_bytes())

        package = source / checkpoint.OPENMV_USB60_CANDIDATE_ROOT
        package.mkdir(parents=True)
        runtime_entries = []
        compile_modules = []
        for name in sorted(checkpoint.OPENMV_USB60_CANDIDATE_RUNTIME_FILES):
            content = ("synthetic candidate module: " + name + "\n").encode()
            (package / name).write_bytes(content)
            source_sha = hashlib.sha256(content).hexdigest()
            compiled = ("synthetic compiled module: " + name).encode()
            compiled_sha = hashlib.sha256(compiled).hexdigest()
            runtime_entries.append({
                "name": name, "size_bytes": len(content), "sha256": source_sha,
            })
            compile_modules.append({
                "name": name, "source_sha256": source_sha,
                "compiled_sha256": compiled_sha,
                "compiled_size_bytes": len(compiled), "exit_code": 0,
            })
        readme = b"Synthetic hardware-pending package.\n"
        cross_compile = {
            "target": "armv7emsp", "module_count": 19,
            "all_exit_codes_zero": True, "modules": compile_modules,
            "compiler_path": "mpy-cross fixture",
            "compiler_version": "MicroPython fixture",
        }
        summary = json.dumps({
            "schema_version": 1, "cross_compile": cross_compile,
        }, sort_keys=True).encode() + b"\n"
        (package / "README.md").write_bytes(readme)
        (package / "CROSS_COMPILE_SUMMARY.json").write_bytes(summary)
        documents = {
            "README.md": {
                "size_bytes": len(readme), "sha256": hashlib.sha256(readme).hexdigest(),
            },
            "CROSS_COMPILE_SUMMARY.json": {
                "size_bytes": len(summary), "sha256": hashlib.sha256(summary).hexdigest(),
            },
        }
        baseline_manifest = json.loads((
            source / checkpoint.OPENMV_ACKERMANN_ROOT / "coordinated" /
            "PACKAGE_MANIFEST.json").read_text(encoding="utf-8-sig"))
        manifest = {
            "schema_version": 1,
            "package_id": checkpoint.OPENMV_USB60_CANDIDATE_PACKAGE_ID,
            "release_status": checkpoint.OPENMV_USB60_CHECKPOINT_STATUS,
            "runtime_file_count": len(runtime_entries),
            "runtime_files": runtime_entries,
            "documents": documents,
            "baseline_coordinated_manifest_sha256":
                checkpoint.OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256,
            "baseline": {
                "package": f"{checkpoint.OPENMV_ACKERMANN_ROOT}/coordinated",
                "package_manifest_sha256":
                    checkpoint.OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256,
                "runtime_files": [
                    {key: entry[key] for key in ("name", "size_bytes", "sha256")}
                    for entry in baseline_manifest["runtime_files"]
                ],
                "h743_elfs": [
                    {"path": path, "sha256": digest}
                    for path, digest in checkpoint.OPENMV_USB60_BASELINE_H743_ELFS.items()
                ],
            },
            "baseline_h743_elfs": [
                {"path": path, "sha256": digest}
                for path, digest in checkpoint.OPENMV_USB60_BASELINE_H743_ELFS.items()
            ],
            "limits": ["No 60 FPS acceptance is claimed."],
            "production_openmv_unchanged": True,
            "full_pipeline_60fps_verified": False,
            "hardware_validation": {
                name: False
                for name in checkpoint.OPENMV_USB60_HW_VALIDATION_FALSE_FIELDS
            },
            "cross_compile": {
                **cross_compile,
            },
        }
        write_json(package / checkpoint.OPENMV_USB60_CANDIDATE_MANIFEST, manifest)
        return package

    def test_usb60_candidate_is_exactly_allowlisted_and_hash_verified(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            self.make_usb60_fixture(source)
            package = source / checkpoint.OPENMV_USB60_CANDIDATE_ROOT
            manifest_path = package / checkpoint.OPENMV_USB60_CANDIDATE_MANIFEST
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["hardware_validation"]["physical_uart_throughput_verified"] = False
            write_json(manifest_path, manifest)

            with self.assertRaisesRegex(ValueError, "only be checkpointed"):
                checkpoint.select_openmv_usb60_candidate(source, "experimental-software-tested")
            allowlist = checkpoint.select_openmv_usb60_candidate(
                source, checkpoint.OPENMV_USB60_CHECKPOINT_STATUS)

            self.assertEqual(len(allowlist), 19 + 3)
            self.assertEqual(
                {Path(name).name for name in allowlist},
                checkpoint.OPENMV_USB60_CANDIDATE_RUNTIME_FILES | {
                    "README.md", "CROSS_COMPILE_SUMMARY.json", "PACKAGE_MANIFEST.json",
                },
            )
            self.assertTrue(all(name.startswith(
                checkpoint.OPENMV_USB60_CANDIDATE_ROOT + "/") for name in allowlist))

    def test_usb60_candidate_rejects_changed_h743_baseline_reference(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            manifest_path = package / checkpoint.OPENMV_USB60_CANDIDATE_MANIFEST
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["baseline"]["h743_elfs"][0]["sha256"] = "0" * 64
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "H743 baseline reference"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_usb60_candidate_rejects_hardware_validation_claim(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            manifest_path = package / checkpoint.OPENMV_USB60_CANDIDATE_MANIFEST
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["hardware_validation"]["full_pipeline_60fps_verified"] = True
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "hardware-validation state"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_usb60_candidate_rejects_extra_file(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            (package / "serial-console.log").write_text("private\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "file allowlist mismatch"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_usb60_candidate_rejects_traversal_runtime_path(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            manifest_path = package / checkpoint.OPENMV_USB60_CANDIDATE_MANIFEST
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["runtime_files"][0]["name"] = "../outside.py"
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "unsafe or duplicate path"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_usb60_candidate_rejects_symlink(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            link = package / "linked.txt"
            original_rglob = Path.rglob
            original_is_symlink = Path.is_symlink

            def rglob_with_link(path, pattern):
                entries = list(original_rglob(path, pattern))
                if path == package:
                    entries.append(link)
                return entries

            def mark_link(path):
                if path == link:
                    return True
                return original_is_symlink(path)

            with patch.object(Path, "rglob", rglob_with_link), \
                    patch.object(Path, "is_symlink", mark_link):
                with self.assertRaisesRegex(RuntimeError, "contains a symlink"):
                    checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_usb60_candidate_rejects_tampered_runtime_and_document(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            module = package / sorted(checkpoint.OPENMV_USB60_CANDIDATE_RUNTIME_FILES)[0]
            module.write_bytes(module.read_bytes() + b"tampered\n")
            with self.assertRaisesRegex(RuntimeError, "runtime hash/size mismatch"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            package = self.make_usb60_fixture(source)
            summary = package / "CROSS_COMPILE_SUMMARY.json"
            summary.write_bytes(summary.read_bytes() + b" ")
            with self.assertRaisesRegex(RuntimeError, "document hash/size mismatch"):
                checkpoint.verify_openmv_usb60_candidate_delivery(source)

    def test_openmv_native_preview_candidate_is_validated_and_explicitly_allowlisted(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            self.make_native_preview_fixture(source)
            delivery = source / checkpoint.OPENMV_NATIVE_PREVIEW_ROOT
            (delivery / "README.md").write_text("Review notes\n", encoding="utf-8")
            (delivery / "EVIDENCE.md").write_text("Build evidence\n", encoding="utf-8")
            (delivery / "ci-artifact.zip").write_bytes(b"must stay local")

            allowlist = checkpoint.verify_openmv_native_preview_delivery(source)

            self.assertEqual(allowlist, [
                *checkpoint.OPENMV_NATIVE_PREVIEW_REQUIRED_FILES,
                *checkpoint.OPENMV_NATIVE_PREVIEW_OPTIONAL_DOCUMENTS,
            ])
            self.assertNotIn(
                f"{checkpoint.OPENMV_NATIVE_PREVIEW_ROOT}/ci-artifact.zip", allowlist)
            self.assertEqual(set(checkpoint.OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256), {
                Path(name).name for name in checkpoint.OPENMV_NATIVE_PREVIEW_REQUIRED_FILES
                if Path(name).name in checkpoint.OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256
            })

    def test_openmv_native_preview_rejects_missing_artifact(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            self.make_native_preview_fixture(source)
            (source / checkpoint.OPENMV_NATIVE_PREVIEW_ROOT / "firmware.elf").unlink()

            with self.assertRaisesRegex(RuntimeError, "allowlisted artifact is missing"):
                checkpoint.verify_openmv_native_preview_delivery(source)

    def test_openmv_native_preview_rejects_artifact_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            self.make_native_preview_fixture(source)
            firmware_bin = source / checkpoint.OPENMV_NATIVE_PREVIEW_ROOT / "firmware.bin"
            damaged = bytearray(firmware_bin.read_bytes())
            damaged[-1] ^= 1
            firmware_bin.write_bytes(damaged)

            with self.assertRaisesRegex(RuntimeError, "artifact hash mismatch: firmware.bin"):
                checkpoint.verify_openmv_native_preview_delivery(source)

    def test_openmv_native_preview_rejects_live_patch_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            self.make_native_preview_fixture(source)
            patch_path = source / checkpoint.OPENMV_NATIVE_PREVIEW_PATCH
            patch_path.write_bytes(patch_path.read_bytes() + b"\n")

            with self.assertRaisesRegex(RuntimeError, "live pinned patch hash"):
                checkpoint.verify_openmv_native_preview_delivery(source)

    def test_ackermann_delivery_pair_and_twelve_build_rows_are_hash_checked(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            presets = {
                "Debug", "DebugSteering", "DebugBench", "DebugFollow",
                "DebugFollowNoLog", "DebugFollowDiff5", "DebugFollowDiff10",
                "DebugFollowTurn", "DebugFollowSpeedPI", "DebugFollowTurn65",
                "DebugFollowIncrementalPI", "DebugFollowIncrementalPIBaseline",
            }
            build_rows = []
            build_logs_root = source / checkpoint.ACKERMANN_DELIVERY_ROOT / "build-logs"
            for preset in sorted(presets):
                elf = source / "build" / preset / "CarTest.elf"
                elf.parent.mkdir(parents=True, exist_ok=True)
                data = ("ELF fixture " + preset).encode()
                elf.write_bytes(data)
                log_name = preset + ".log"
                (build_logs_root / log_name).parent.mkdir(parents=True, exist_ok=True)
                (build_logs_root / log_name).write_text("synthetic build evidence\n", encoding="utf-8")
                exitcode_name = preset + ".exitcode"
                (build_logs_root / exitcode_name).write_text("0\n", encoding="utf-8")
                build_rows.append({
                    "preset": preset, "exit_code": 0,
                    "log": "build-logs/" + log_name,
                    "exitcode_file": "build-logs/" + exitcode_name,
                    "elf": f"build/{preset}/CarTest.elf",
                    "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(),
                })

            artifacts = []
            for variant, spec in checkpoint.ACKERMANN_PROFILE_DELIVERIES.items():
                package = source / checkpoint.ACKERMANN_DELIVERY_ROOT / spec["directory"]
                package.mkdir(parents=True, exist_ok=True)
                data = (source / "build" / spec["profile"] / "CarTest.elf").read_bytes()
                delivery = package / "CarTest.elf"
                delivery.write_bytes(data)
                sha = hashlib.sha256(data).hexdigest()
                write_json(package / "manifest.json", {
                    "schema_version": 1,
                    "profile": spec["directory"],
                    "preset": spec["profile"],
                    "coordination_enabled": spec["coordination_enabled"],
                    "servo_slew_step_us": spec["servo_slew_step_us"],
                    "elf": "CarTest.elf", "bytes": len(data), "sha256": sha,
                    "build_log": "../build-logs/" + spec["profile"] + ".log",
                })
                artifacts.append({
                    "delivery_path": f"{spec['directory']}/CarTest.elf",
                    "preset": spec["profile"],
                    "coordination_enabled": spec["coordination_enabled"],
                    "bytes": len(data), "sha256": sha,
                })

            write_json(source / checkpoint.ACKERMANN_BUILD_SUMMARY, {
                "schema_version": 1, "builds": build_rows,
            })
            write_json(source / checkpoint.ACKERMANN_ARTIFACT_MANIFEST, {
                "schema_version": 1, "generated_utc": "2026-10-23T00:00:00Z",
                "package_id": "H743_Ackermann60_20261022", "artifacts": artifacts,
            })
            paired_doc = source / checkpoint.ACKERMANN_PAIRED_DELIVERY_DOC
            paired_doc.parent.mkdir(parents=True, exist_ok=True)
            paired_doc.write_text("synthetic paired delivery documentation\n", encoding="utf-8")
            run_steps = source / checkpoint.ACKERMANN_PAIRED_RUN_STEPS
            run_steps.parent.mkdir(parents=True, exist_ok=True)
            run_steps.write_text("synthetic paired run steps\n", encoding="utf-8")
            evidence = source / checkpoint.ACKERMANN_TOOLING_EVIDENCE
            evidence.parent.mkdir(parents=True, exist_ok=True)
            evidence.write_text("synthetic tooling evidence\n", encoding="utf-8")
            checkpoint.verify_ackermann_deliveries(source, presets)

            manifest_path = source / checkpoint.ACKERMANN_PROFILE_DELIVERIES["coordinated"]["manifest"]
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["build_log"] = "../../outside.log"
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "profile manifest does not match"):
                checkpoint.verify_ackermann_deliveries(source, presets)
            manifest["build_log"] = "../build-logs/DebugFollowIncrementalPI.log"
            write_json(manifest_path, manifest)

            exitcode_path = build_logs_root / "DebugFollowIncrementalPI.exitcode"
            exitcode_path.write_text("1\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "exit-code evidence is missing or failed"):
                checkpoint.verify_ackermann_deliveries(source, presets)
            exitcode_path.write_text("0\n", encoding="utf-8")

            artifacts[1]["coordination_enabled"] = True
            write_json(source / checkpoint.ACKERMANN_ARTIFACT_MANIFEST, {
                "schema_version": 1, "generated_utc": "2026-10-23T00:00:00Z",
                "package_id": "H743_Ackermann60_20261022", "artifacts": artifacts,
            })
            with self.assertRaisesRegex(RuntimeError, "does not match baseline ELF/coordination"):
                checkpoint.verify_ackermann_deliveries(source, presets)

    def test_openmv_ackermann_allowlist_copies_only_nineteen_runtime_modules(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)
            for profile in checkpoint.OPENMV_ACKERMANN_PACKAGES:
                package = source / checkpoint.OPENMV_ACKERMANN_ROOT / profile
                package.mkdir(parents=True, exist_ok=True)
                entries = []
                for name in sorted(checkpoint.OPENMV_ACKERMANN_RUNTIME_FILES):
                    content = (profile + ":" + name).encode()
                    (package / name).write_bytes(content)
                    entries.append({
                        "name": name,
                        "size_bytes": len(content),
                        "sha256": hashlib.sha256(content).hexdigest(),
                        "mpy_cross_exit_code": 0,
                    })
                (package / "README.md").write_text("Synthetic package\n", encoding="utf-8")
                write_json(package / "PACKAGE_MANIFEST.json", {
                    "schema_version": 1, "package_id": "OpenMV-test",
                    "build_id": "source-and-mpy-hash", "control_profile": profile,
                    "config_profile_selector": profile,
                    "runtime_file_count": len(entries), "runtime_files": entries,
                    "protocol": {
                        "version": 2, "frame_size_bytes": 16,
                        "curve_phase_offset": 10, "reserved_zero_offset": 11,
                        "slowdown_request_permille_offset": 12, "crc_offset": 14,
                    },
                    "curve_trace": {
                        "field": "curve_trace_v2_hex", "record_size_bytes": 16,
                        "camera_diag_record_size_bytes": 32,
                        "control_ext_v2_record_size_bytes": 32,
                    },
                })

            allowlist = checkpoint.verify_openmv_ackermann_packages(source)
            self.assertEqual(len(allowlist), 2 * (19 + 2))
            self.assertNotIn("fast_ring_slope.py", "\n".join(allowlist))
            self.assertTrue(all("tests" not in item and "logs" not in item for item in allowlist))

            package = source / checkpoint.OPENMV_ACKERMANN_ROOT / "coordinated"
            manifest_path = package / "PACKAGE_MANIFEST.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["protocol"]["version"] = 1
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "manifest metadata is incomplete"):
                checkpoint.verify_openmv_ackermann_packages(source)
            manifest["protocol"]["version"] = 2
            content = b"ring module deliberately rejected"
            (package / "fast_ring_slope.py").write_bytes(content)
            manifest["runtime_files"].append({
                "name": "fast_ring_slope.py", "size_bytes": len(content),
                "sha256": hashlib.sha256(content).hexdigest(), "mpy_cross_exit_code": 0,
            })
            manifest["runtime_file_count"] += 1
            write_json(manifest_path, manifest)
            with self.assertRaisesRegex(RuntimeError, "manifest metadata is incomplete"):
                checkpoint.verify_openmv_ackermann_packages(source)


if __name__ == "__main__":
    unittest.main()
