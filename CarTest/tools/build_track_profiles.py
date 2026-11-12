"""Freeze the explicit H750-camera/three-H743-profile delivery after validation.

Never uploads, flashes, starts a vehicle, or overwrites an existing delivery.
The camera source is shared: profile selection is received from the paired MCU.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

try:
    from .build_openmv_ackermann320_packages import RUNTIME_NAMES as BASE_RUNTIME
    from .runtime_log_export import inspect_elf
except ImportError:
    from build_openmv_ackermann320_packages import RUNTIME_NAMES as BASE_RUNTIME
    from runtime_log_export import inspect_elf

ROOT = Path(__file__).resolve().parents[1]
RUNTIME_NAMES = tuple(BASE_RUNTIME) + (
    "track_elements.py", "ring_width_calibration.py", "ring_diagnostics.py",
    "fast_ring_slope.py")
PROFILES = (
    ("A_FastNoRing", "DebugFastNoRing", 1, 5600, 4400, False),
    ("B_RingBase", "DebugRingBase", 2, 4800, 4000, True),
    ("C_RingFast", "DebugRingFast", 3, 5600, 4400, True),
)
DEFAULT_COMPILER = ROOT / "tmp/mpy-cross-1.28.0/mpy-cross.exe"


def resolve_compiler(requested=None):
    if requested is not None:
        path = Path(requested)
        if not path.is_file():
            raise FileNotFoundError("mpy-cross compiler not found: " + str(path))
        return path.resolve()
    if DEFAULT_COMPILER.is_file():
        return DEFAULT_COMPILER
    try:
        import mpy_cross
        return Path(mpy_cross.mpy_cross).resolve()
    except ImportError as error:
        raise FileNotFoundError(
            "Supply --compiler, or run with uv run --with mpy-cross==1.28.0.post2") from error


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n",
                          encoding="utf-8")


def verify_build_cache(source, preset, profile_id, straight, curve):
    cache_path = Path(source) / "build" / preset / "CMakeCache.txt"
    cache = {}
    for line in cache_path.read_text(encoding="utf-8").splitlines():
        if "=" in line and not line.startswith(("//", "#")):
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    required = {"CARTEST_TRACK_PROFILE": str(profile_id),
                "CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS": str(straight),
                "CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS": str(curve),
                "CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI": "ON",
                "CARTEST_ENABLE_ACKERMANN_COORDINATION": "ON"}
    if any(cache.get(key) != value for key, value in required.items()):
        raise ValueError("Build cache does not match the declared profile: " + preset)
    return required


def verify_manifest(directory):
    """Verify exact membership as well as every byte, rejecting extra payloads."""
    directory = Path(directory)
    manifest = json.loads((directory / "SHA256SUMS.json").read_text(encoding="utf-8"))
    actual = {p.relative_to(directory).as_posix(): sha256(p)
              for p in directory.rglob("*")
              if p.is_file() and p != directory / "SHA256SUMS.json"}
    if actual != manifest:
        raise ValueError("Delivery membership or content hash mismatch")
    if any(p.is_symlink() for p in directory.rglob("*")):
        raise ValueError("Linked payloads are not supported")
    return manifest


def public_delivery_files(source, relative_directory):
    """Return an explicit publication allowlist after validating the full delivery."""
    source = Path(source).resolve()
    relative = Path(relative_directory)
    directory = (source / relative).resolve()
    if (relative.is_absolute() or ".." in relative.parts or
            not directory.is_relative_to(source / "deliverables")):
        raise ValueError("Track delivery must be a local deliverables subdirectory")
    verify_manifest(directory)
    camera_manifest = json.loads((directory / "OpenMV/PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    if camera_manifest.get("runtime_files") != list(RUNTIME_NAMES):
        raise ValueError("Track runtime allowlist differs from the reviewed set")
    rows = camera_manifest.get("compilation", [])
    if (len(rows) != len(RUNTIME_NAMES) or
            {row.get("file") for row in rows} != set(RUNTIME_NAMES) or
            any(row.get("exit_code") != 0 for row in rows)):
        raise ValueError("Track runtime compilation is incomplete")
    by_name = {row["file"]: row for row in rows}
    for name in RUNTIME_NAMES:
        if sha256(directory / "OpenMV" / name) != sha256(source / "openmv" / name):
            raise ValueError("Track delivery is stale relative to the live camera source")
        compiled = directory / "validation/mpy" / (Path(name).stem + ".mpy")
        if (by_name[name].get("source_sha256") != sha256(directory / "OpenMV" / name) or
                by_name[name].get("mpy_sha256") != sha256(compiled)):
            raise ValueError("Track compiler evidence does not match the source or output")
    profiles = json.loads((directory / "profiles.json").read_text(encoding="utf-8"))["profiles"]
    if len(profiles) != len(PROFILES):
        raise ValueError("Track profile list is incomplete")
    files = ["README.md", "profiles.json", "SHA256SUMS.json", "OpenMV/PACKAGE_MANIFEST.json"]
    files.extend("OpenMV/" + name for name in RUNTIME_NAMES)
    for name in RUNTIME_NAMES:
        files.extend(("validation/mpy/" + Path(name).stem + ".mpy",
                      "validation/mpy/" + name + ".log",
                      "validation/mpy/" + name + ".exitcode"))
    for actual, (label, preset, profile_id, straight, curve, ring) in zip(profiles, PROFILES):
        expected = {"label": label, "preset": preset, "profile_id": profile_id,
                    "straight_target_millirps": straight, "curve_outer_target_millirps": curve,
                    "ring_enabled": ring}
        if any(actual.get(key) != value for key, value in expected.items()):
            raise ValueError("Track profile parameters differ from approved values")
        frozen = directory / "H743" / label / "CarTest.elf"
        live = source / "build" / preset / "CarTest.elf"
        if sha256(frozen) != sha256(live) or actual.get("elf_sha256") != sha256(frozen):
            raise ValueError("Track ELF is stale or mislabeled: " + preset)
        if actual.get("build_id") != inspect_elf(frozen).build_id.hex():
            raise ValueError("Track ELF build ID differs from manifest")
        files.extend(("H743/" + label + "/CarTest.elf", "H743/" + label + "/manifest.json"))
    listed = set(json.loads((directory / "SHA256SUMS.json").read_text(encoding="utf-8")))
    if listed != set(files) - {"SHA256SUMS.json"}:
        raise ValueError("Track delivery contains unreviewed payload names")
    return [(relative / name).as_posix() for name in files]


def build(source, output, compiler):
    source, output = Path(source), Path(output)
    compiler = resolve_compiler(compiler)
    if output.exists():
        raise FileExistsError("Refusing to overwrite an immutable delivery: " + str(output))
    for name in RUNTIME_NAMES:
        if not (source / "openmv" / name).is_file():
            raise FileNotFoundError(name)
    layouts = {}
    for label, preset, profile_id, straight, curve, _ in PROFILES:
        verify_build_cache(source, preset, profile_id, straight, curve)
        layouts[label] = inspect_elf(source / "build" / preset / "CarTest.elf")
    output.mkdir(parents=True)
    camera = output / "OpenMV"
    camera.mkdir()
    compiled = output / "validation" / "mpy"
    compiled.mkdir(parents=True)
    compilation = []
    for name in RUNTIME_NAMES:
        src = source / "openmv" / name
        shutil.copy2(src, camera / name)
        mpy = compiled / (Path(name).stem + ".mpy")
        command = [str(compiler), "-march=armv7emsp", "-o", str(mpy), str(src)]
        result = subprocess.run(command, capture_output=True, text=True, errors="replace")
        (compiled / (name + ".log")).write_text(result.stdout + result.stderr,
                                                encoding="utf-8")
        (compiled / (name + ".exitcode")).write_text(str(result.returncode) + "\n")
        compilation.append({"file": name, "source_sha256": sha256(src),
                            "exit_code": result.returncode,
                            "mpy_sha256": sha256(mpy) if mpy.exists() else None})
        if result.returncode:
            write_json(output / "compile-summary.json", compilation)
            raise RuntimeError("Cross-compilation failed: " + name)
    profiles = []
    for label, preset, profile_id, straight, curve, ring in PROFILES:
        target = output / "H743" / label
        target.mkdir(parents=True)
        layout = layouts[label]
        shutil.copy2(layout.path, target / "CarTest.elf")
        metadata = {"label": label, "preset": preset, "profile_id": profile_id,
                    "straight_target_millirps": straight,
                    "curve_outer_target_millirps": curve,
                    "ring_enabled": ring,
                    "ring_outer_target_millirps": curve if ring else None,
                    "build_id": layout.build_id.hex(), "elf_sha256": layout.sha256,
                    "runtime_log_record_size": 64, "runtime_log_header_size": 64,
                    "vehicle_acceptance": "pending_manual_road_tests"}
        write_json(target / "manifest.json", metadata)
        profiles.append(metadata)
    compiler_version = subprocess.run([str(compiler), "--version"], capture_output=True,
                                      text=True, check=True).stdout.strip()
    write_json(camera / "PACKAGE_MANIFEST.json", {
        "runtime_files": list(RUNTIME_NAMES), "shared_profile_selection": True,
        "sensor": "MT9V034", "image_width": 320, "image_height": 120,
        "camera_firmware_policy": "retain_current_vendor_firmware",
        "compiler_sha256": sha256(compiler), "compiler_target": "armv7emsp",
        "compiler_version": compiler_version,
        "compilation": compilation,
        "mpy_output_policy": "validation_only_upload_python_sources"})
    write_json(output / "profiles.json", {"version": 1, "profiles": profiles,
        "forward_wire": "V3/16-byte/profile-ack", "reverse_state_wire": "V2/20-byte/revision1",
        "reverse_heartbeat_wire": "V1/8-byte/RESET-KEEP",
        "hardware_acceptance": "pending_manual_road_tests"})
    (output / "README.md").write_text(
        "# H750 camera and three H743 profiles\n\n"
        "Upload all 23 Python files in OpenMV together; do not upload validation .mpy files.\n"
        "Flash exactly one H743 ELF. It selects A/B/C through versioned communication.\n"
        "An unknown, stale, or mismatched profile must not start a new Follow session.\n"
        "B/C require the CRC-verified 45 cm straight calibration and parked width check; A is unchanged.\n"
        "Run run_ring_width_calibration('/flash/ring45_ground_raw.bin') only at the REPL after stopping the loop; it sends invalid V3/ACK0 frames.\n"
        "Persist frozen ring history with ring_diag_persist() before removing camera power.\n"
        "A=5.6/4.4 rps, ring off; B=4.8/4.0 rps, ring on; C=5.6/4.4 rps, ring on.\n"
        "Values are wheel-speed targets, not measured body speeds.\n\n"
        "Keep the current H750 vendor firmware. No old H743 camera firmware is included.\n"
        "Software tests and cross-compilation do not establish ring traversal or FPS.\n"
        "Perform manual raised-wheel checks, straight, individual bends, crossroads, "
        "left/right rings, then complete laps with matching logs/video.\n",
        encoding="utf-8")
    manifest = {p.relative_to(output).as_posix(): sha256(p)
                for p in sorted(output.rglob("*")) if p.is_file()}
    write_json(output / "SHA256SUMS.json", manifest)
    verify_manifest(output)
    return {"directory": str(output), "files": len(manifest), "runtime_files": len(RUNTIME_NAMES)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compiler", type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if args.verify:
        print(json.dumps({"files": len(verify_manifest(args.output)), "verified": True}))
    else:
        print(json.dumps(build(args.source.resolve(), args.output.resolve(), args.compiler)))


if __name__ == "__main__":
    main()
