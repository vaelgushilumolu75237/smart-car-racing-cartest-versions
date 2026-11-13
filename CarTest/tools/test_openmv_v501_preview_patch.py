"""Verify the pinned OpenMV v5.0.1 OPENMV4P preview-control patch."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PATCH_DIR = PROJECT_ROOT / "tools" / "vendor-patches" / "openmv-v5.0.1"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def git_text(source: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source), *args], text=True
    ).strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    args = parser.parse_args()
    source = args.source_root.resolve()
    patch_file = PATCH_DIR / "0001-snapshot-preview-control.patch"
    manifest = json.loads((PATCH_DIR / "manifest.json").read_text("utf-8"))
    patch = patch_file.read_bytes()

    if sha256(patch) != manifest["patch_sha256"]:
        raise SystemExit("patch SHA-256 does not match manifest")
    if git_text(source, "rev-parse", "HEAD") != manifest["commit"]:
        raise SystemExit("source checkout does not match pinned commit")

    for relative, expected in manifest["base_sha256"].items():
        actual = sha256(
            subprocess.check_output(
                ["git", "-C", str(source), "show", "HEAD:" + relative]
            )
        )
        if actual != expected:
            raise SystemExit("pinned base hash mismatch: " + relative)

    apply_check = subprocess.run(
        ["git", "-C", str(source), "apply", "--check", str(patch_file)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    reverse_check = subprocess.run(
        ["git", "-C", str(source), "apply", "--reverse", "--check", str(patch_file)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if apply_check.returncode and reverse_check.returncode:
        raise SystemExit("source is neither clean upstream nor exactly patched")

    board_mk = (source / "boards/OPENMV4P/board_config.mk").read_text("utf-8")
    board_h = (source / "boards/OPENMV4P/board_config.h").read_text("utf-8")
    boot_h = (source / "boards/OPENMV4P/boot_config.h").read_text("utf-8")
    if "MCU=STM32H743xx\n" not in board_mk:
        raise SystemExit("OPENMV4P MCU no longer matches pinned H743 target")
    if "OMV_JLINK_ARGS=--device STM32H743VI\n" not in board_mk:
        raise SystemExit("OPENMV4P debug device no longer matches H743")
    if "#define OMV_FLASH_LENGTH                      2048K" not in board_h:
        raise SystemExit("OPENMV4P internal flash map changed")
    if "#define OMV_DRAM_ORIGIN                       0xC0000000" not in board_h:
        raise SystemExit("OPENMV4P SDRAM origin changed")
    if "#define OMV_BOOT_QSPI_FLASH_SIZE    (0x2000000)" not in boot_h:
        raise SystemExit("OPENMV4P QSPI size changed")

    print(
        "PASS: pinned OpenMV v5.0.1 patch and OPENMV4P/H743 board map "
        + ("apply cleanly" if apply_check.returncode == 0 else "are exactly applied")
    )
    print("NOTE: no H750 target is present in this upstream tag")
    return 0


if __name__ == "__main__":
    sys.exit(main())
