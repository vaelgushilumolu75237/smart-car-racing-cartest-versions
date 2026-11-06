"""Create a complete local delivery without captured/user calibration data."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    manifest = json.loads((ROOT / "openmv/PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    install = OUT / "Install/K230"
    install.mkdir(parents=True, exist_ok=True)
    for name in manifest["runtime_files"] + ["PACKAGE_MANIFEST.json", "README.md", "ring_width_calibration.json"]:
        source = ROOT / "openmv" / name
        if name in manifest["source_sha256"] and digest(source) != manifest["source_sha256"][name]:
            raise RuntimeError("Runtime changed since validation: " + name)
        shutil.copy2(source, install / name)
    calibration = json.loads((install / "ring_width_calibration.json").read_text(encoding="utf-8"))
    if calibration.get("ready") is not False or calibration.get("reason") != "K230_RECALIBRATION_REQUIRED":
        raise RuntimeError("Installer must never contain H750 or fabricated K230 calibration")
    mapping = {"C_Base": "DebugRingFast", "C_Plus20": "DebugRingFastSpeed120",
               "C_Plus30": "DebugRingFastSpeed130", "C_Plus40": "DebugRingFastSpeed140"}
    for name, profile in mapping.items():
        directory = OUT / "Install/H743" / name
        directory.mkdir(parents=True, exist_ok=True)
        for suffix in ("elf", "hex"):
            shutil.copy2(ROOT / "build" / profile / ("CarTest." + suffix), directory)
        (directory / "PAIR.json").write_text(json.dumps({"profile": profile,
            "camera_port": manifest["port"], "elf_sha256": digest(directory / "CarTest.elf"),
            "flashed": False}, indent=2), encoding="utf-8")
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
        if any(part in (".git", "__pycache__", "logs", "tmp", "deliverables") for part in rel.parts):
            continue
        if rel.parts[0] == "build" and p.name not in ("CarTest.elf", "CarTest.hex"):
            continue
        if p.suffix in (".obj", ".o", ".pyc", ".pdb"):
            continue
        candidates.append(p)
    candidates.extend(p for p in (OUT / "Install").rglob("*") if p.is_file())
    candidates.extend(p for p in (OUT / "validation").glob("*.json") if p.is_file())
    hashes = {p.relative_to(OUT).as_posix(): digest(p) for p in sorted(candidates)}
    inventory = OUT / "COMPLETE_PACKAGE_SHA256.json"
    inventory.write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    archive = OUT / "CarTest_K230_R24_Port_20261106.zip"
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
