"""Freeze and verify six paired OpenMV/gyro/SeekFree builds; no device access."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess

try:
    from .build_track_profiles import RUNTIME_NAMES as LEGACY_RUNTIME
    from .build_track_profiles import sha256, write_json, verify_manifest, resolve_compiler
    from .runtime_log_export import inspect_elf
except ImportError:
    from build_track_profiles import RUNTIME_NAMES as LEGACY_RUNTIME
    from build_track_profiles import sha256, write_json, verify_manifest, resolve_compiler
    from runtime_log_export import inspect_elf

ROOT = Path(__file__).resolve().parents[1]
KIND = "openmv_gyro_wireless"
EXTENSION_VERSION = "v20261112-openmv-timed150-startdiag"
# The shared camera changes profile only after a valid reverse-state handshake.
PROFILES = (
    ("A_FastNoRing", "DebugFastNoRing", 1, 100, 5600, 4400, 0, 0),
    ("B_RingBase", "DebugRingBase", 2, 100, 4800, 4000, 4000, 64935),
    ("C_RingFast", "DebugRingFast", 3, 100, 5600, 4400, 5280, 85714),
    ("C_Speed120", "DebugRingFastSpeed120", 3, 120, 6720, 5280, 6336, 102857),
    ("C_Speed130", "DebugRingFastSpeed130", 3, 130, 7280, 5720, 6864, 111429),
    ("C_Speed140", "DebugRingFastSpeed140", 3, 140, 7840, 6160, 7392, 120000),
)


def runtime_names():
    return tuple(LEGACY_RUNTIME) + (
        "gyro_link.py", "entry_calibration_common.py", "visual_entry_gate.py",
        "wireless_sideband.py")


def read_cache(source, preset):
    result = {}
    for line in (Path(source) / "build" / preset / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if not line.startswith(("#", "//")) and "=" in line:
            key, value = line.split("=", 1)
            result[key.split(":", 1)[0]] = value
    return result


def expected_profile(row):
    label, preset, profile, percent, straight, bend, orbit, yaw = row
    ring_target = ((orbit * 75) + 50) // 100 if orbit else 0
    return dict(label=label, preset=preset, profile_id=profile, speed_percent=percent,
                straight_target_millirps=straight, curve_outer_target_millirps=bend,
                ring_outer_base_target_millirps=orbit,
                ring_outer_target_millirps=ring_target,
                exit_outer_target_millirps=orbit,
                entry_orbit_speed_percent=75,
                entry_orbit_servo_gain_permille=1300,
                entry_variant="original_confirmed_ring_delay150",
                ring_rate_mdps=yaw, gyro_authority=profile != 1)


def verify_cache(source, row):
    _, preset, profile, percent, *_ = row
    cache = read_cache(source, preset)
    required = dict(CARTEST_TRACK_PROFILE=str(profile), CARTEST_SPEED_PERCENT=str(percent),
                    CARTEST_ENABLE_GYRO350="ON", CARTEST_ENABLE_WIRELESS="ON",
                    CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI="ON",
                    CARTEST_ENABLE_ACKERMANN_COORDINATION="ON",
                    CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS="4800" if profile == 2 else "5600",
                    CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS="4000" if profile == 2 else "4400")
    if any(cache.get(k) != v for k, v in required.items()):
        raise ValueError("Wrong feature/profile build cache: " + preset)
    return required


def public_delivery_files(source, relative_directory):
    source = Path(source).resolve()
    relative = Path(relative_directory)
    directory = (source / relative).resolve()
    if relative.is_absolute() or ".." in relative.parts or not directory.is_relative_to(source / "deliverables"):
        raise ValueError("Delivery must be inside source/deliverables")
    manifest = verify_manifest(directory)
    metadata = json.loads((directory / "profiles.json").read_text(encoding="utf-8"))
    if metadata.get("delivery_kind") != KIND or metadata.get("profiles") is None:
        raise ValueError("Wrong delivery kind")
    profiles = metadata["profiles"]
    if len(profiles) != len(PROFILES):
        raise ValueError("Expected six paired profiles")
    camera = json.loads((directory / "OpenMV/PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    names = runtime_names()
    if camera.get("runtime_files") != list(names):
        raise ValueError("Unreviewed camera module membership")
    rows = camera.get("compilation", [])
    if len(rows) != len(names) or {r.get("file") for r in rows} != set(names):
        raise ValueError("Incomplete camera compilation")
    by_name = {r["file"]: r for r in rows}
    files = {"README.md", "WIRELESS_PROTOCOL.md", "profiles.json", "OpenMV/PACKAGE_MANIFEST.json", "validation/software-validation.json"}
    for name in names:
        src = source / "openmv" / name
        frozen = directory / "OpenMV" / name
        mpy = directory / "validation/mpy" / (Path(name).stem + ".mpy")
        row = by_name[name]
        if (sha256(src) != sha256(frozen) or row.get("source_sha256") != sha256(frozen)
                or row.get("exit_code") != 0 or row.get("mpy_sha256") != sha256(mpy)):
            raise ValueError("Camera source/compiler evidence mismatch: " + name)
        files.update(("OpenMV/" + name, "validation/mpy/" + Path(name).stem + ".mpy",
                      "validation/mpy/" + name + ".log", "validation/mpy/" + name + ".exitcode"))
    for row, actual in zip(PROFILES, profiles):
        expected = expected_profile(row)
        if any(actual.get(k) != v for k, v in expected.items()):
            raise ValueError("Wrong profile labels/targets")
        verify_cache(source, row)
        label, preset = row[:2]
        elf = directory / "H743" / label / "CarTest.elf"
        if sha256(elf) != sha256(source / "build" / preset / "CarTest.elf") or actual.get("elf_sha256") != sha256(elf):
            raise ValueError("Mismatched paired ELF: " + preset)
        if inspect_elf(elf).build_id.hex() != actual.get("build_id"):
            raise ValueError("Mismatched ELF build identity: " + preset)
        frozen_meta = json.loads((elf.parent / "manifest.json").read_text(encoding="utf-8"))
        if frozen_meta != actual:
            raise ValueError("Profile manifest differs from delivery metadata")
        files.update(("H743/" + label + "/CarTest.elf", "H743/" + label + "/manifest.json"))
    if set(manifest) != files:
        raise ValueError("Unreviewed delivery payload membership")
    validation = json.loads((directory / "validation/software-validation.json").read_text(encoding="utf-8"))
    if validation.get("hardware_tested") is not False or validation.get("all_required_checks_passed") is not True:
        raise ValueError("Software validation evidence is incomplete")
    return [(relative / name).as_posix() for name in sorted(files | {"SHA256SUMS.json"})]


def build(source, output, compiler, validation, readme):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError("Refusing to overwrite immutable delivery")
    compiler = resolve_compiler(compiler)
    for row in PROFILES:
        verify_cache(source, row)
        inspect_elf(source / "build" / row[1] / "CarTest.elf")
    output.mkdir(parents=True)
    camera, compiled = output / "OpenMV", output / "validation/mpy"
    camera.mkdir(); compiled.mkdir(parents=True)
    compilation = []
    for name in runtime_names():
        src = source / "openmv" / name
        shutil.copy2(src, camera / name)
        mpy = compiled / (Path(name).stem + ".mpy")
        result = subprocess.run([str(compiler), "-march=armv7emsp", "-o", str(mpy), str(src)], capture_output=True, text=True)
        (compiled / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        (compiled / (name + ".exitcode")).write_text(str(result.returncode) + "\n", encoding="utf-8")
        if result.returncode:
            raise RuntimeError("OpenMV cross-compile failed: " + name)
        compilation.append(dict(file=name, source_sha256=sha256(src), exit_code=0, mpy_sha256=sha256(mpy)))
    profiles = []
    for row in PROFILES:
        meta = expected_profile(row)
        elf = source / "build" / row[1] / "CarTest.elf"
        layout = inspect_elf(elf)
        meta.update(elf_sha256=sha256(elf), build_id=layout.build_id.hex(), hardware_acceptance="pending_manual_tests")
        target = output / "H743" / row[0]
        target.mkdir(parents=True)
        shutil.copy2(elf, target / "CarTest.elf")
        write_json(target / "manifest.json", meta)
        profiles.append(meta)
    write_json(camera / "PACKAGE_MANIFEST.json", dict(
               extension_version=EXTENSION_VERSION,
               entry_variant="original_confirmed_ring_delay150",
               entry_calibration_schema="NONE_TIMED150",
               entry_calibration_required=False,
               ring_entry_orbit_speed_percent=75,
               ring_entry_orbit_servo_gain_permille=1300,
               runtime_files=list(runtime_names()), compilation=compilation,
               compiler_sha256=sha256(compiler), compiler_target="armv7emsp", sensor="MT9V034",
               image_width=320, image_height=120, forward_wire="V4/16-byte", shared_profile_selection=True,
               camera_firmware_policy="retain_current_vendor_firmware", image_transfer=False))
    write_json(output / "profiles.json", dict(
               delivery_kind=KIND, variant=EXTENSION_VERSION,
               entry_variant="original_confirmed_ring_delay150",
               calibration_status="entry_calibration_not_required",
               profiles=profiles, hardware_tested=False))
    shutil.copy2(validation, output / "validation/software-validation.json")
    shutil.copy2(readme, output / "README.md")
    shutil.copy2(source / "tests/wireless/PROTOCOL.md", output / "WIRELESS_PROTOCOL.md")
    write_json(output / "SHA256SUMS.json", {p.relative_to(output).as_posix(): sha256(p) for p in output.rglob("*") if p.is_file()})
    public_delivery_files(source, output.relative_to(source))
    return dict(directory=str(output), profiles=len(profiles), camera_modules=len(runtime_names()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compiler", type=Path)
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--readme", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output, args.compiler, args.validation, args.readme)))


if __name__ == "__main__":
    main()
