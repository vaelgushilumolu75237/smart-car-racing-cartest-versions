"""Create a complete local delivery without captured/user calibration data."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_hex(elf, hex_path):
    cache = (elf.parent / "CMakeCache.txt").read_text(
        encoding="utf-8", errors="replace")
    marker = "CMAKE_C_COMPILER_AR:FILEPATH="
    compiler_ar = next((line[len(marker):].strip() for line in cache.splitlines()
                        if line.startswith(marker)), None)
    if not compiler_ar:
        raise RuntimeError("Verified toolchain path is missing from CMakeCache")
    objcopy = Path(compiler_ar).with_name("arm-none-eabi-objcopy.exe")
    if not objcopy.is_file():
        raise RuntimeError("ARM objcopy is missing beside the verified compiler")
    result = subprocess.run([str(objcopy), "-O", "ihex", str(elf), str(hex_path)],
                            capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError("ELF to Intel HEX conversion failed: " + result.stderr)
    payload = hex_path.read_text(encoding="ascii")
    if not payload.startswith(":") or not payload.rstrip().endswith(":00000001FF"):
        raise RuntimeError("Generated Intel HEX has an invalid envelope")
    if hex_path.stat().st_mtime_ns < elf.stat().st_mtime_ns:
        raise RuntimeError("Generated HEX predates the validated ELF")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-name', help='New local install/archive stem; never reuses a historical installer')
    args = parser.parse_args()
    if args.output_name and (Path(args.output_name).name != args.output_name or
                            any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.output_name)):
        raise ValueError('Output name must be a plain ASCII filename stem')
    manifest = json.loads((ROOT / "openmv/PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    wireless = manifest.get("wireless_required", False)
    aux_pins = bool(manifest.get("wireless_aux_gpio"))
    seekfree_start = manifest.get("release_version") == "v20261108-k230-seekfree-start-v10"
    timing_fix = manifest.get("release_version") == "v20261108-k230-imu-timing-v9"
    imu_fix = manifest.get("release_version") == "v20261108-k230-imu-mode0-v8"
    uart_fix = manifest.get("release_version") == "v20261108-k230-uart-readinto-v7"
    display_fix = manifest.get("release_version") == "v20261108-k230-transport-diag-v6"
    if (manifest.get('release_version') and
            not any((seekfree_start, timing_fix, imu_fix, uart_fix, display_fix)) and
            not args.output_name):
        raise ValueError('A new release requires --output-name to preserve historical installers')
    install_root = OUT / ("Install_SeekFree_Start_v10" if seekfree_start else "Install_IMU_Timing_v9" if timing_fix else "Install_IMU_Mode0_v8" if imu_fix else "Install_UART_Readinto_v7" if uart_fix else "Install_Transport_Diag_v6" if display_fix else "Install_Wireless_v2_RTS_CMD" if aux_pins else "Install")
    if args.output_name:
        install_root = OUT / ('Install_' + args.output_name)
    install = install_root / "K230"
    install.mkdir(parents=True, exist_ok=True)
    for name in manifest["runtime_files"] + ["PACKAGE_MANIFEST.json", "README.md", "ring_width_calibration.json"]:
        source = ROOT / "openmv" / name
        if name in manifest["source_sha256"] and digest(source) != manifest["source_sha256"][name]:
            raise RuntimeError("Runtime changed since validation: " + name)
        shutil.copy2(source, install / name)
    calibration = json.loads((install / "ring_width_calibration.json").read_text(encoding="utf-8"))
    if calibration.get("ready") is not False or calibration.get("reason") != "K230_RECALIBRATION_REQUIRED":
        raise RuntimeError("Installer must never contain H750 or fabricated K230 calibration")
    # A gyro-required camera cannot be installed with the historical V3 ELFs.
    mapping = ({"Wireless_Gyro350_C_Plus40": "DebugWirelessGyro350Speed140"} if wireless
               else {"Gyro350_C_Plus40": "DebugGyro350Speed140"})
    existing_profiles = install_root / "H743"
    if existing_profiles.exists() and set(p.name for p in existing_profiles.iterdir()) - set(mapping):
        raise RuntimeError("Unexpected stale MCU installer directories; preserve them before packaging")
    for name, profile in mapping.items():
        directory = install_root / "H743" / name
        directory.mkdir(parents=True, exist_ok=True)
        elf = ROOT / "build" / profile / "CarTest.elf"
        hex_path = ROOT / "build" / profile / "CarTest.hex"
        make_hex(elf, hex_path)
        for suffix in ("elf", "hex"):
            shutil.copy2(ROOT / "build" / profile / ("CarTest." + suffix), directory)
        (directory / "PAIR.json").write_text(json.dumps({"profile": profile,
            "camera_port": manifest["port"], "elf_sha256": digest(directory / "CarTest.elf"),
            "release_version": manifest.get("release_version"), "flashed": False}, indent=2), encoding="utf-8")
    for name in ("GYRO350_20261106.md", "K230_PORT_20261106.md", "K230_VALIDATION.json"):
        shutil.copy2(ROOT / "docs" / name, install_root / name)
    if (ROOT / 'docs/FRESH_ROUTE_V24_CN.md').is_file():
        shutil.copy2(ROOT / 'docs/FRESH_ROUTE_V24_CN.md', install_root / 'FRESH_ROUTE_V24_CN.md')
    if (ROOT / 'docs/UART_RX_V25_CN.md').is_file():
        for name in ('UART_RX_V25_CN.md','UART_RX_V25_VALIDATION.json'):
            shutil.copy2(ROOT/'docs'/name, install_root/name)
    if (ROOT / 'docs/SINGLE_EDGE_AUTHORITY_V26_CN.md').is_file():
        for name in ('SINGLE_EDGE_AUTHORITY_V26_CN.md',
                     'SINGLE_EDGE_AUTHORITY_V26_VALIDATION.json'):
            shutil.copy2(ROOT/'docs'/name, install_root/name)
    if (ROOT / 'docs/TURN_SPEED_V27_CN.md').is_file():
        for name in ('TURN_SPEED_V27_CN.md', 'TURN_SPEED_V27_VALIDATION.json'):
            shutil.copy2(ROOT/'docs'/name, install_root/name)
    if (ROOT / 'docs/SPEED_V28_CN.md').is_file():
        for name in ('SPEED_V28_CN.md', 'SPEED_V28_VALIDATION.json'):
            shutil.copy2(ROOT/'docs'/name, install_root/name)
    if (ROOT / 'docs/RING_BASELINE_V29_CN.md').is_file():
        for name in ('RING_BASELINE_V29_CN.md', 'RING_BASELINE_V29_VALIDATION.json'):
            shutil.copy2(ROOT/'docs'/name, install_root/name)
    if wireless:
        for name in ("WIRELESS_20261107.md", "WIRELESS_PC_PROTOCOL.md", "WIRELESS_VALIDATION.json"):
            shutil.copy2(ROOT / "docs" / name, install_root / name)
    if seekfree_start:
        shutil.copy2(ROOT / "docs/SEEKFREE_START_V10_CN.md", install_root / "SEEKFREE_START_V10_CN.md")
    baseline = json.loads((OUT / "private-baseline/SOURCE_BEFORE.json").read_text(encoding="utf-8"))
    source = Path(baseline["source"])
    changed = [name for name, expected in baseline["files"].items() if digest(source / name) != expected]
    if changed:
        raise RuntimeError("Original r24 tree changed: " + repr(changed))
    (OUT / "validation/BASELINE_UNCHANGED.json").write_text(json.dumps({
        "verified_files": len(baseline["files"]), "changed": changed,
        "original_archive_sha256": baseline["archive_sha256"]}, indent=2), encoding="utf-8")
    candidates = []
    for p in ROOT.rglob("*"):
        if not p.is_file():
            continue
        rel = p.relative_to(ROOT)
        if any(part in (".git", "__pycache__", "logs", "tmp", "deliverables", "validation") for part in rel.parts):
            continue
        if p.name.startswith("k230_wireless_settings"):
            continue
        if p.name == "ring_width_calibration.json" and rel.as_posix() != "openmv/ring_width_calibration.json":
            continue  # Historical real calibration belongs only in private restoration.
        if rel.parts[0] == "build" and p.name not in ("CarTest.elf", "CarTest.hex"):
            continue
        if rel.parts[0] == "build" and p.name == "CarTest.hex" and rel.parts[1] not in mapping.values():
            continue  # Historical HEX files can be stale relative to their ELF.
        if p.suffix in (".obj", ".o", ".pyc", ".pdb", ".exe", ".dll"):
            continue
        candidates.append(p)
    candidates.append(ROOT / "deliverables/CarTest-20261014-0422-after/build/DebugFollowTurn/CarTest.elf")
    if (OUT / "PUBLISH_RESULT.json").is_file():
        published = json.loads((OUT / 'PUBLISH_RESULT.json').read_text(encoding='utf-8'))
        if published.get('version') == manifest.get('release_version'):
            candidates.append(OUT / "PUBLISH_RESULT.json")
    candidates.extend(p for p in install_root.rglob("*") if p.is_file())
    candidates.extend(p for p in (OUT / "validation").glob("*.json") if p.is_file())
    hashes = {p.relative_to(OUT).as_posix(): digest(p) for p in sorted(candidates)}
    inventory = OUT / "COMPLETE_PACKAGE_SHA256.json"
    inventory.write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    archive = OUT / ("CarTest_K230_SeekFree_Start_v10_20261108.zip" if seekfree_start else "CarTest_K230_IMU_Timing_v9_20261108.zip" if timing_fix else "CarTest_K230_IMU_Mode0_v8_20261108.zip" if imu_fix else "CarTest_K230_UART_Readinto_v7_20261108.zip" if uart_fix else "CarTest_K230_Transport_Diag_v6_20261108.zip" if display_fix else "CarTest_K230_Wireless_v2_RTS_CMD_20261107.zip" if aux_pins else
                     "CarTest_K230_Wireless_20261107.zip" if wireless else "CarTest_K230_Gyro350_20261106.zip")
    if args.output_name:
        archive = OUT / ('CarTest_' + args.output_name + '_COMPLETE.zip')
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for p in candidates + [inventory]:
            z.write(p, p.relative_to(OUT).as_posix())
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:
            raise RuntimeError("ZIP CRC validation failed")
        for name, expected in hashes.items():
            if hashlib.sha256(z.read(name)).hexdigest() != expected:
                raise RuntimeError("ZIP member mismatch: " + name)
    result = {"archive": archive.name, "sha256": digest(archive),
              "files": len(hashes) + 1, "bytes": archive.stat().st_size,
              "private_h750_calibration_included": False,
              "k230_calibration": "not-ready placeholder", "hardware_actions": "none"}
    (OUT / "ARCHIVE_VERIFIED.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))
