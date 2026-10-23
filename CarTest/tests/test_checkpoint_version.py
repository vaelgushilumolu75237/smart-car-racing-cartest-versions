import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


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
