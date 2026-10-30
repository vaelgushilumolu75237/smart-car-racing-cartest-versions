#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/../../.." && pwd)"
source_root="${1:-}"
if [[ -z "$source_root" || ! -d "$source_root/.git" ]]; then
    echo "Usage: $0 /path/to/clean/openmv-v5.0.1-checkout" >&2
    exit 2
fi
source_root="$(cd -- "$source_root" && pwd)"
patch_file="$script_dir/0001-snapshot-preview-control.patch"
artifact_dir="${2:-$project_root/tmp/openmv-v5.0.1-openmv4p-artifacts}"
expected_commit="631681e5ac5ab332c920b6bf9cd3dba5cf299eac"
expected_patch="E3A93287D75298664E68068547F9A1F68D15DB0422F0BC08DCC1E7D4D6ADF193"

actual_commit="$(git -C "$source_root" rev-parse HEAD)"
if [[ "$actual_commit" != "$expected_commit" ]]; then
    echo "Unexpected OpenMV source commit: $actual_commit" >&2
    exit 3
fi
actual_patch="$(sha256sum "$patch_file" | awk '{print toupper($1)}')"
if [[ "$actual_patch" != "$expected_patch" ]]; then
    echo "Patch SHA-256 mismatch: $actual_patch" >&2
    exit 4
fi

python3 - "$source_root" "$script_dir/manifest.json" <<'PY'
import hashlib
import json
from pathlib import Path
import subprocess
import sys

source = Path(sys.argv[1])
manifest = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
for relative, expected in manifest["base_sha256"].items():
    data = subprocess.check_output(
        ["git", "-C", str(source), "show", "HEAD:" + relative]
    )
    actual = hashlib.sha256(data).hexdigest().upper()
    if actual != expected:
        raise SystemExit("Pinned source hash mismatch: " + relative)
actual_micropython = subprocess.check_output(
    ["git", "-C", str(source), "rev-parse", "HEAD:lib/micropython"],
    text=True,
).strip()
if actual_micropython != manifest["micropython_commit"]:
    raise SystemExit("Pinned MicroPython submodule commit mismatch")
PY

board_mk="$source_root/boards/OPENMV4P/board_config.mk"
board_h="$source_root/boards/OPENMV4P/board_config.h"
boot_h="$source_root/boards/OPENMV4P/boot_config.h"
grep -qx 'MCU=STM32H743xx' "$board_mk"
grep -qx 'OMV_JLINK_ARGS=--device STM32H743VI' "$board_mk"
grep -qx '#define OMV_FLASH_LENGTH                      2048K' "$board_h"
grep -qx '#define OMV_DRAM_ORIGIN                       0xC0000000' "$board_h"
grep -qx '#define OMV_DRAM_LENGTH                       32M' "$board_h"
grep -qx '#define OMV_BOOT_QSPI_FLASH_SIZE    (0x2000000)  // Must always be a power of 2.' "$boot_h"

if git -C "$source_root" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
    changed_files="$(git -C "$source_root" diff --name-only)"
    unexpected_files="$(printf '%s\n' "$changed_files" | grep -Ev '^(modules/py_csi_ng\.c|scripts/libraries/sensor\.py)$' || true)"
    if [[ -n "$unexpected_files" ]]; then
        echo "Unexpected modified source files:" >&2
        printf '%s\n' "$unexpected_files" >&2
        exit 5
    fi
    echo "Preview patch is already applied."
elif git -C "$source_root" apply --check "$patch_file"; then
    if [[ -n "$(git -C "$source_root" diff --name-only)" ]]; then
        echo "Refusing to patch a source tree with unrelated changes." >&2
        exit 5
    fi
    git -C "$source_root" apply "$patch_file"
else
    echo "Source tree is neither clean upstream nor patched with this exact diff." >&2
    exit 6
fi

if ! grep -q 'PREVIEW_CONTROL_API' "$source_root/modules/py_csi_ng.c" || \
   ! grep -q 'OMV_CSI_FLAG_NO_UPDATE' "$source_root/modules/py_csi_ng.c" || \
   ! grep -q 'PREVIEW_CONTROL_API = csi.PREVIEW_CONTROL_API' "$source_root/scripts/libraries/sensor.py"; then
    echo "Patched preview API markers are missing." >&2
    exit 7
fi

if [[ "$(cat "$source_root/SDK_VERSION")" != "1.6.0" ]]; then
    echo "Unexpected OpenMV SDK version." >&2
    exit 8
fi

(
    cd "$source_root"
    make sdk
    make -j"$(nproc)" TARGET=OPENMV4P
)

firmware_bin="$source_root/build/OPENMV4P/bin/firmware.bin"
firmware_elf="$source_root/build/OPENMV4P/bin/firmware.elf"
test -s "$firmware_bin"
test -s "$firmware_elf"
mkdir -p "$artifact_dir"
cp -- "$firmware_bin" "$firmware_elf" "$artifact_dir/"
python3 - "$artifact_dir" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
manifest = {
    "openmv_commit": "631681e5ac5ab332c920b6bf9cd3dba5cf299eac",
    "target": "OPENMV4P",
    "sdk_version": "1.6.0",
    "patch_sha256": "E3A93287D75298664E68068547F9A1F68D15DB0422F0BC08DCC1E7D4D6ADF193",
    "artifacts": {},
}
for name in ("firmware.bin", "firmware.elf"):
    data = (output / name).read_bytes()
    manifest["artifacts"][name] = {
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest().upper(),
    }
(output / "build-manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
)
print(json.dumps(manifest, indent=2))
PY
