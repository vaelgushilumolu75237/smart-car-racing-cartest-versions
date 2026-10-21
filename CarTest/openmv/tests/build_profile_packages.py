"""Build three complete OpenMV source bundles and mpy-cross validation logs."""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess


OPENMV_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = OPENMV_DIR.parent
DELIVERY_ROOT = (PROJECT_DIR / "deliverables" /
                 "OpenMV_Bottom80_Steering_20261021")
VALIDATION_ROOT = (PROJECT_DIR / "test-data" /
                   "speedpd-20261020-validation" / "camera")
COMPILER = Path(
    r"D:\OpenMV IDE\share\qtcreator\mpy-cross\windows\mpy_cross"
    r"\mpy-cross.exe")
TARGET = "armv7emsp"
PROFILES = (
    ("no_pd", "BOTTOM80_NO_PD_20261020"),
    ("pd", "BOTTOM80_POSPD_20261020"),
    ("pd_preview", "BOTTOM80_PD_PREVIEW_20261020"),
)
CONTROL_SETTINGS = {
    "row_weighting": "full-image y=24 split; near 0.80, far 0.20",
    "far_support": "at least 3 rows spanning at least 2 grid rows; otherwise near-only",
    "invalid_position": "no near rows",
    "position_ema": "existing EMA, unchanged",
    "pd": {
        "input": "filtered position only",
        "derivative_low_pass_tau_ms": 60,
        "derivative_time_ms": 30,
        "derivative_component_limit": 100,
        "maximum_capture_dt_ms": 100,
        "combined_output_limit": 1000,
        "duplicate_capture": "no history update; hold output",
    },
    "preview": {
        "default_enabled": False,
        "candidate_min_abs": 80,
        "same_sign_distinct_frames": 3,
        "confirmation_min_ms": 60,
        "position_min_abs": 50,
        "linear_gain_full_at_position_abs": 250,
        "maximum_gain": 0.35,
        "applied_limit": 100,
        "slew_per_second": 500,
    },
    "reset_conditions": [
        "invalid capture", "capture dt over 100 ms", "identity change",
        "actual selected weighted reference-row change",
    ],
    "observation_uart": "12-byte V1, unchanged",
    "control_diagnostic_extension": {
        "version": 2,
        "record_bytes": 32,
        "sequence": "independent u16",
        "capture_send_timestamps": "camera ticks_ms; camera-clock-relative",
        "crc": "CRC-16/CCITT-FALSE over bytes 0-29, stored little-endian at 30-31",
        "derivative_field": "D component after its +/-100 clamp and before total P+D clamp",
        "sent_control": "signed control from actual base observation frame",
    },
}
SOURCE_README = """# CarTest OpenMV camera

The production source defaults to profile `pd`, build ID `BOTTOM80_POSPD_20261020`. Three complete upload packages are in `../deliverables/OpenMV_Bottom80_Steering_20261021/`. The first camera trial should use `no_pd` to establish the same perception and weighted-position baseline before enabling derivative control. Upload one package at a time: copy its 20 top-level `.py` files together and restart the OpenMV interpreter. Do not mix profiles or upload host `.mpy` files or tests.

| Package | Control behavior | Trial order |
| --- | --- | --- |
| `no_pd` | Existing position EMA output only | First |
| `pd` | Position-only P+D after the EMA | Second |
| `pd_preview` | PD plus bounded confirmed geometry preview | Optional last |

All profiles preserve the same image and route processing. Full-image y=24 divides the 80% near and 20% far groups. The far group must contain at least three supported rows spanning at least two grid rows; otherwise the near rows normalize to the full position. No near rows means an invalid position. The existing EMA is retained. PD uses only that filtered position, a 60 ms low-pass derivative, 30 ms derivative time, D limit 100, maximum capture interval 100 ms, and combined output limit 1000. Invalid frames, long capture gaps, source/identity changes and changes to the actual selected weighted reference row reset the controller. Repeated captures do not advance its history.

Preview remains off in `no_pd` and `pd`. In `pd_preview`, a geometry-qualified candidate of at least 80 magnitude must keep the same sign for three distinct captures over at least 60 ms, with same-sign filtered position of at least 50. Gain rises linearly to 0.35 at position magnitude 250, with applied contribution capped at 100 and slew limited to 500 per second. Existing geometry conflict, invalidity, identity and reference resets remain in force.

The 12-byte observation UART V1 frame is unchanged. The camera diagnostic dump has a separate 32-byte control-extension v2 record with its own sequence, capture/send camera-clock timestamps, actual sent command and CRC-16/CCITT-FALSE. Camera times are relative to the OpenMV clock and must not be subtracted from STM32 arrival times as if they shared an epoch.

`PACKAGE_MANIFEST.json` identifies the default `pd` package. Each upload folder has its own manifest, build ID and file hashes. All 60 profile modules were cross-compiled with bundled mpy-cross 1.25.0 for the ARMv7EM-SP target. The reported board runtime is MicroPython 1.28.0-49; no matching mpy-cross compiler was available in the IDE archive, so 1.28 compiler acceptance remains unverified. The production OpenMV host suite passes 183 tests; the frozen V7 suite passes 79 with one expected board-only case skipped. These are host and compiler checks only. `test-data/speedpd-20261020-validation/camera/host-performance-comparison.json` reports the full-clear versus touched-row aggregation comparison and controller-only profile timing on synthetic CPython inputs. It does not benchmark the sensor/Viper frame loop or predict camera frame rate. No image-processing resolution reduction or safety-gate removal was made for speed. Production OpenMV 45 fps and vehicle behavior remain unverified; no camera upload or motor run was performed.
"""


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8", newline="")


def runtime_files():
    files = sorted(OPENMV_DIR.glob("*.py"), key=lambda path: path.name)
    if len(files) != 20:
        raise SystemExit("expected 20 production .py files, found %d" %
                         len(files))
    return files


def copy_profile_sources(destination, files, profile):
    destination.mkdir(parents=True, exist_ok=False)
    for source in files:
        content = source.read_text(encoding="utf-8")
        if source.name == "config.py":
            original = 'STEERING_CONTROL_PROFILE = "pd"'
            replacement = 'STEERING_CONTROL_PROFILE = "%s"' % profile
            if content.count(original) != 1:
                raise SystemExit("cannot identify single PD profile selector")
            content = content.replace(original, replacement, 1)
            if profile != "pd" and original in content:
                raise SystemExit("profile selector replacement failed")
        (destination / source.name).write_text(
            content, encoding="utf-8", newline="")


def compile_profile(profile, package_dir, compiler_version, compiler_hash):
    output_dir = VALIDATION_ROOT / "mpy-cross-armv7emsp" / profile
    output_dir.mkdir(parents=True, exist_ok=False)
    compiled = []
    for source in sorted(package_dir.glob("*.py"), key=lambda path: path.name):
        output = output_dir / (source.stem + ".mpy")
        command = [str(COMPILER), "-march=" + TARGET, "-s", source.name,
                   "-o", str(output), str(source)]
        result = subprocess.run(command, capture_output=True, text=True,
                                check=False)
        if result.returncode != 0:
            raise SystemExit(
                "mpy-cross failed (%s, %s):\n%s\n%s" %
                (profile, source.name, result.stdout, result.stderr))
        compiled.append({
            "name": source.name,
            "source_sha256": sha256_file(source),
            "compiled_path": str(output.relative_to(PROJECT_DIR)).replace(
                "\\", "/"),
            "compiled_size_bytes": output.stat().st_size,
            "compiled_sha256": sha256_file(output),
            "exit_code": result.returncode,
        })
    if len(compiled) != 20:
        raise SystemExit("did not compile exactly 20 runtime modules")
    return {
        "target": TARGET,
        "compiler_path": str(COMPILER),
        "compiler_version": compiler_version,
        "compiler_sha256": compiler_hash,
        "command_template": 'mpy-cross -march=armv7emsp -s <module.py> -o <module.mpy> <module.py>',
        "module_count": len(compiled),
        "all_exit_codes_zero": all(item["exit_code"] == 0
                                   for item in compiled),
        "scope": "MicroPython .mpy source compatibility for the OpenMV ARMv7EM-SP target; not board execution or Viper machine-code runtime proof",
        "modules": compiled,
    }


def profile_readme(profile, build_id):
    first_trial = ("This is the recommended first upload for the camera trial."
                   if profile == "no_pd" else
                   "This is an optional comparison profile; begin with no_pd.")
    preview_line = ("Confirmed adaptive preview is enabled in this profile."
                    if profile == "pd_preview" else
                    "Adaptive preview control is disabled in this profile.")
    return (
        "# OpenMV Bottom80 steering package\n\n"
        "Profile: `%s`  \nBuild ID: `%s`\n\n"
        "%s %s\n\n"
        "Upload all 20 top-level `.py` files from this folder together, then "
        "restart the OpenMV interpreter. Do not mix profiles, upload tests, "
        "or place host-generated `.mpy` validation files on the camera. The "
        "12-byte observation UART protocol is unchanged.\n\n"
        "All profiles use full-frame y=24 weighting (near 80%% / far 20%%). "
        "Far support requires at least three rows spanning two grid rows; "
        "otherwise the near group is normalized alone. No near rows means "
        "invalid position. The existing position EMA remains the PD input.\n\n"
        "PD uses a 60 ms derivative low-pass time constant, 30 ms derivative "
        "time, +/-100 D component, 100 ms maximum capture gap and +/-1000 "
        "combined output. Preview requires three distinct same-sign frames "
        "over at least 60 ms, candidate magnitude >=80, same-sign position "
        "magnitude >=50, gain up to 0.35 at |position|=250, +/-100 applied "
        "limit and 500/s slew. %s\n\n"
        "`PACKAGE_MANIFEST.json` records all source and compiled hashes. "
        "The bundled compiler is mpy-cross 1.25.0 for ARMv7EM-SP; the reported "
        "board runtime is MicroPython 1.28.0-49 and no matching compiler was "
        "available in the IDE archive. Cross-compilation does not prove 1.28 "
        "acceptance, OpenMV board runtime, vehicle behavior, or 45 fps. No "
        "hardware upload was performed.\n"
    ) % (profile, build_id, first_trial, preview_line, preview_line)


def main():
    if not COMPILER.is_file():
        raise SystemExit("OpenMV mpy-cross compiler not found: %s" % COMPILER)
    files = runtime_files()
    if DELIVERY_ROOT.exists():
        raise SystemExit("refusing to overwrite profile packages: %s" %
                         DELIVERY_ROOT)
    compiler_version_result = subprocess.run(
        [str(COMPILER), "--version"], capture_output=True, text=True,
        check=False)
    if compiler_version_result.returncode != 0:
        raise SystemExit("cannot query mpy-cross version")
    compiler_version = (compiler_version_result.stdout.strip() or
                        compiler_version_result.stderr.strip())
    compiler_hash = sha256_file(COMPILER)
    DELIVERY_ROOT.mkdir(parents=True, exist_ok=False)
    package_reports = []

    for profile, build_id in PROFILES:
        package_dir = DELIVERY_ROOT / profile
        copy_profile_sources(package_dir, files, profile)
        compile_report = compile_profile(
            profile, package_dir, compiler_version, compiler_hash)
        entries = []
        for source in sorted(package_dir.glob("*.py"), key=lambda path: path.name):
            compiled = next(item for item in compile_report["modules"]
                            if item["name"] == source.name)
            entries.append({
                "name": source.name,
                "size_bytes": source.stat().st_size,
                "sha256": sha256_file(source),
                "mpy_cross_exit_code": compiled["exit_code"],
                "compiled_size_bytes": compiled["compiled_size_bytes"],
                "compiled_sha256": compiled["compiled_sha256"],
                "compiled_validation_path": compiled["compiled_path"],
            })
        (package_dir / "README.md").write_text(
            profile_readme(profile, build_id), encoding="utf-8", newline="")
        manifest = {
            "schema_version": 1,
            "package_id": build_id,
            "build_id": build_id,
            "control_profile": profile,
            "release_status": "HOST_TESTED_CROSS_COMPILED_HARDWARE_VALIDATION_PENDING",
            "route_mode": "cached_width",
            "runtime_file_count": len(entries),
            "runtime_files": entries,
            "control": CONTROL_SETTINGS,
            "cross_compile": compile_report,
            "target_runtime_reported_version": "MicroPython 1.28.0-49",
            "matching_mpy_cross_compiler_available": False,
            "target_runtime_reported_version": "MicroPython 1.28.0-49",
            "matching_mpy_cross_compiler_available": False,
            "validation": {
                "production_host_suite": "see test-data/speedpd-20261020-validation/camera/openmv-unittest-discover.log",
                "frozen_v7_suite": "see test-data/speedpd-20261020-validation/camera/frozen-v7-regression.log",
                "host_performance": "see test-data/speedpd-20261020-validation/camera/host-performance-comparison.json",
            },
            "hardware_validation": {
                "openmv_uploaded": False,
                "camera_board_runtime_tested": False,
                "vehicle_tested": False,
                "production_45fps_verified": False,
            },
            "limits": [
                "Host tests and mpy-cross do not establish board runtime or vehicle steering.",
                "No camera upload, H743 flash/reset, or motor start was performed.",
                "OpenMV production 45 fps remains unverified.",
            ],
        }
        write_json(package_dir / "PACKAGE_MANIFEST.json", manifest)
        package_reports.append({
            "profile": profile,
            "package_id": build_id,
            "directory": str(package_dir.relative_to(PROJECT_DIR)).replace(
                "\\", "/"),
            "runtime_file_count": len(entries),
            "manifest_sha256": sha256_file(package_dir / "PACKAGE_MANIFEST.json"),
            "cross_compile_exit_code": 0,
            "cross_compile_module_count": len(compile_report["modules"]),
            "cross_compile_target": TARGET,
        })

    shutil.copyfile(DELIVERY_ROOT / "pd" / "PACKAGE_MANIFEST.json",
                    OPENMV_DIR / "PACKAGE_MANIFEST.json")
    (OPENMV_DIR / "README.md").write_text(
        SOURCE_README, encoding="utf-8", newline="")
    report = {
        "schema_version": 1,
        "created_by": "openmv/tests/build_profile_packages.py",
        "profiles": package_reports,
        "all_profile_compiles_passed": True,
        "total_compiled_module_count": sum(
            item["cross_compile_module_count"] for item in package_reports),
        "compiler_path": str(COMPILER),
        "compiler_version": compiler_version,
        "compiler_sha256": compiler_hash,
        "target": TARGET,
        "hardware_upload_performed": False,
    }
    write_json(VALIDATION_ROOT / "profile-package-build-report.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
