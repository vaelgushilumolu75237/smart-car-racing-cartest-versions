#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/../../.." && pwd)"
source_root="${1:-}"
build_dir="${2:-}"
artifact_dir="${3:-}"
if [[ -z "$source_root" || -z "$build_dir" || -z "$artifact_dir" || \
      ! -d "$source_root/.git" ]]; then
    echo "Usage: $0 /q/openmv-v5.0.1-checkout /q/c /d/path/to/new-artifacts" >&2
    exit 2
fi
source_root="$(cd -- "$source_root" && pwd)"
if [[ "$(git -C "$source_root" rev-parse HEAD)" != \
      "631681e5ac5ab332c920b6bf9cd3dba5cf299eac" ]]; then
    echo "Unexpected OpenMV base commit." >&2
    exit 3
fi
if git -C "$source_root" apply --check \
        "$script_dir/0001-snapshot-preview-control.patch" >/dev/null 2>&1; then
    echo "The pinned preview patch is not applied." >&2
    exit 4
fi
if ! git -C "$source_root" apply --reverse --check \
        "$script_dir/0001-snapshot-preview-control.patch" >/dev/null 2>&1; then
    echo "The source is not at the pinned preview patch state." >&2
    exit 5
fi
if git -C "$source_root/lib/micropython" apply --check -p3 \
        "$script_dir/0002-stm32h7-uart-fifo-write.patch" \
        >/dev/null 2>&1; then
    echo "The pinned UART FIFO patch is not applied." >&2
    exit 6
fi
if ! git -C "$source_root/lib/micropython" apply --reverse --check -p3 \
        "$script_dir/0002-stm32h7-uart-fifo-write.patch" \
        >/dev/null 2>&1; then
    echo "The MicroPython submodule is not at the pinned FIFO patch state." >&2
    exit 7
fi

script_dir_native="$(cygpath -w "$script_dir")"
source_root_native="$(cygpath -w "$source_root")"
python3 - "$script_dir_native" "$source_root_native" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

patch_dir = Path(sys.argv[1])
source_root = Path(sys.argv[2])
manifest = json.loads((patch_dir / "0003-cartest-accel-module.manifest.json")
                      .read_text(encoding="utf-8"))
patch = patch_dir / manifest["patch_file"]
actual_patch = hashlib.sha256(patch.read_bytes()).hexdigest().upper()
if actual_patch != manifest["patch_sha256"]:
    raise SystemExit("0003 patch SHA-256 mismatch")
for relative, entry in manifest["files"].items():
    source = patch_dir / "cartest_accel" / Path(entry["source"]).relative_to(
        "cartest_accel")
    target = source_root / relative
    for label, path in (("source", source), ("build-tree", target)):
        actual = hashlib.sha256(path.read_bytes()).hexdigest().upper()
        if actual != entry["source_sha256"]:
            raise SystemExit("%s hash mismatch: %s" % (label, path))
modules_mk = (source_root / "modules/micropython.mk").read_text(
    encoding="utf-8")
if "$(wildcard $(OMV_MOD_DIR)/*.c)" not in modules_mk:
    raise SystemExit("OpenMV user-module C wildcard is missing")
PY

if [[ -e "$build_dir" ]] && [[ -n "$(find "$build_dir" -mindepth 1 -print -quit)" ]]; then
    echo "Refusing non-empty build directory: $build_dir" >&2
    exit 8
fi
if [[ -e "$artifact_dir" ]]; then
    echo "Refusing to overwrite artifact directory: $artifact_dir" >&2
    exit 9
fi

mkdir -p "$artifact_dir"
initial_files=0
if [[ -d "$build_dir" ]]; then
    initial_files="$(find "$build_dir" -mindepth 1 -type f | wc -l | tr -d ' ')"
fi
echo "OPENMV_HEAD=$(git -C "$source_root" rev-parse HEAD)"
echo "MICROPYTHON_HEAD=$(git -C "$source_root/lib/micropython" rev-parse HEAD)"
echo "TARGET=OPENMV4P STM32H743xx; firmware target only, no bootloader/ROMFS/flash"
echo "BUILD_DIR=$build_dir"
echo "BUILD_DIR_INITIAL_FILES=$initial_files"
echo "MODULES=cartest_accel (API_VERSION=1)"
echo "MODULE_SOURCE_GLOB=modules/micropython.mk SRC_USERMOD wildcard *.c"
echo "PATCH_SERIES=0001-preview,0002-uart-fifo,0003-cartest-accel"
echo "MICROPY_MPYCROSS=${MICROPY_MPYCROSS:-/r/mpy-cross-1.28.0/mpy-cross.exe}"
echo "PYTHON=${PYTHON:-/r/openmv-build-tools/bin/python3}"

cd -- "$source_root"
make -j"${JOBS:-8}" TARGET=OPENMV4P BUILD="$build_dir" \
    OMV_ENABLE_BL=0 SDK_DIR="${SDK_DIR:-/r/openmv-sdk-version-check-only}" \
    MICROPY_MPYCROSS="${MICROPY_MPYCROSS:-/r/mpy-cross-1.28.0/mpy-cross.exe}" \
    PYTHON="${PYTHON:-/r/openmv-build-tools/bin/python3}" \
    firmware

test -s "$build_dir/bin/firmware.bin"
test -s "$build_dir/bin/firmware.elf"
cp -- "$build_dir/bin/firmware.bin" "$artifact_dir/firmware.bin"
cp -- "$build_dir/bin/firmware.elf" "$artifact_dir/firmware.elf"
cp -- "$build_dir/lib/micropython/genhdr/qstrdefs.generated.h" \
    "$artifact_dir/qstrdefs.generated.h"
source_root_native="$(cygpath -w "$source_root")"
build_dir_native="$(cygpath -w "$build_dir")"
artifact_dir_native="$(cygpath -w "$artifact_dir")"
python3 - "$source_root_native" "$build_dir_native" \
    "$artifact_dir_native" "$initial_files" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

source = Path(sys.argv[1])
build = Path(sys.argv[2])
output = Path(sys.argv[3])
initial_files = int(sys.argv[4])
artifacts = {}
for name in ("firmware.bin", "firmware.elf", "qstrdefs.generated.h"):
    path = output / name
    data = path.read_bytes()
    artifacts[name] = {"bytes": len(data),
                       "sha256": hashlib.sha256(data).hexdigest().upper()}
qstr = (output / "qstrdefs.generated.h").read_text(encoding="utf-8")
required_qstrs = ("cartest_accel", "API_VERSION", "api_version",
                  "region_segments", "internal_component",
                  "WRITE_FIFO_API", "deinit", "flush", "init", "irq",
                  "readchar", "sendbreak", "txdone", "writechar",
                  "write_fifo", "fifo_txdone")
missing_qstrs = [name for name in required_qstrs
                 if "MP_QSTR_" + name not in qstr]
if missing_qstrs:
    raise SystemExit("Missing generated module QSTRs: " + ", ".join(missing_qstrs))
uart_qstrs = required_qstrs[5:]
uart_audit = []
for name in uart_qstrs:
    line = next((line.strip() for line in qstr.splitlines()
                 if "MP_QSTR_" + name + "," in line), None)
    if line is None:
        raise SystemExit("Missing UART API QSTR: " + name)
    uart_audit.append({"name": name, "definition": line})
(output / "uart-qstr-audit.json").write_text(
    json.dumps(uart_audit, indent=2) + "\n", encoding="utf-8")
uart_audit_bytes = (output / "uart-qstr-audit.json").read_bytes()
artifacts["uart-qstr-audit.json"] = {
    "bytes": len(uart_audit_bytes),
    "sha256": hashlib.sha256(uart_audit_bytes).hexdigest().upper(),
}
objects = list(build.rglob("*.o"))
dependencies = list(build.rglob("*.P"))
for expected in ("lib/micropython/modules/py_cartest_accel.o",
                 "lib/micropython/modules/cartest_accel_core.o"):
    if not (build / expected).is_file():
        raise SystemExit("Missing module object: " + expected)
manifest = {
    "artifact_class": "PRIVATE_BUILD_ONLY_NOT_HARDWARE_VALIDATED",
    "deployment_eligible": False,
    "flash_performed": False,
    "board_target": "OPENMV4P",
    "compile_mcu_define": "STM32H743xx",
    "source_commit": "631681e5ac5ab332c920b6bf9cd3dba5cf299eac",
    "micropython_commit": "dd984559aa42841908b78397b7dac32f57da803f",
    "patches": [
        {"file": "0001-snapshot-preview-control.patch",
         "sha256": "E3A93287D75298664E68068547F9A1F68D15DB0422F0BC08DCC1E7D4D6ADF193"},
        {"file": "0002-stm32h7-uart-fifo-write.patch",
         "sha256": "8D1649CC9FB74E00B6D1234F6C8A3A62D70B90EBD605053F069B202EC36890FB"},
        {"file": "0003-cartest-accel-module.patch",
         "sha256": "7153724D48A01BFACE490E6942C0ADEA67C2DB0A8D7444A57D46D381A685EC4D"},
    ],
    "build": {"directory": str(build), "initial_files": initial_files,
              "runtime_object_count": len(objects),
              "dependency_file_count": len(dependencies),
              "module_api_version": 1,
              "module_objects": ["lib/micropython/modules/py_cartest_accel.o",
                                 "lib/micropython/modules/cartest_accel_core.o"],
              "generated_qstrs": list(required_qstrs[:5]),
              "uart_qstrs": list(uart_qstrs)},
    "artifacts": artifacts,
}
(output / "build-manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
print(json.dumps(manifest, indent=2))
PY
