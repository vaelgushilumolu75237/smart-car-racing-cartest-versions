#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat >&2 <<'EOF'
Usage:
  build-openmv4p-ring45-native-clean.sh [--verify-only] \
      SOURCE_ROOT FRESH_BUILD_DIR ARTIFACT_DIR PROJECT_ROOT RUNTIME_ROOT

SOURCE_ROOT    Pinned, already-patched OpenMV v5.0.1 checkout (for example /q)
FRESH_BUILD_DIR Empty, new build directory (for example /q/ring45-native)
ARTIFACT_DIR   New, absent output directory
PROJECT_ROOT   CarTest repository root
RUNTIME_ROOT   Directory holding 23 runtime .py files and PACKAGE_MANIFEST.json

--verify-only checks the pinned five-patch state, source hashes, runtime
inventory, and output-directory preconditions. It exits before creating files
or invoking make.
EOF
}

verify_only=0
if [[ "${1:-}" == "--verify-only" ]]; then
    verify_only=1
    shift
fi
if [[ "$#" -ne 5 ]]; then
    usage
    exit 2
fi

source_input="$1"
build_input="$2"
artifact_input="$3"
project_input="$4"
runtime_input="$5"

canonical_dir() {
    local input="$1"
    [[ -d "$input" ]] || {
        echo "Required directory not found: $input" >&2
        return 1
    }
    (cd -- "$input" && pwd -P)
}

canonical_future_path() {
    local input="$1"
    local parent base parent_abs
    [[ "$input" == /* ]] || input="$PWD/$input"
    parent="$(dirname -- "$input")"
    base="$(basename -- "$input")"
    [[ -n "$base" && "$base" != "." && "$base" != "/" ]] || {
        echo "Expected a new directory path, got: $input" >&2
        return 1
    }
    [[ -d "$parent" ]] || {
        echo "Output parent directory not found: $parent" >&2
        return 1
    }
    parent_abs="$(cd -- "$parent" && pwd -P)"
    printf '%s/%s\n' "$parent_abs" "$base"
}

source_root="$(canonical_dir "$source_input")"
project_root="$(canonical_dir "$project_input")"
runtime_root="$(canonical_dir "$runtime_input")"
build_dir="$(canonical_future_path "$build_input")"
artifact_dir="$(canonical_future_path "$artifact_input")"
patch_dir="$project_root/tools/vendor-patches/openmv-v5.0.1"

if [[ ! -e "$source_root/.git" ]]; then
    echo "SOURCE_ROOT is not a Git checkout: $source_root" >&2
    exit 3
fi
if [[ ! -d "$source_root/lib/micropython" ]]; then
    echo "Missing MicroPython submodule: $source_root/lib/micropython" >&2
    exit 3
fi
if [[ ! -f "$runtime_root/PACKAGE_MANIFEST.json" ]]; then
    echo "Missing runtime package manifest: $runtime_root/PACKAGE_MANIFEST.json" >&2
    exit 3
fi

if [[ -L "$build_dir" || -f "$build_dir" ]]; then
    echo "Fresh build path must be an absent or ordinary directory: $build_dir" >&2
    exit 4
fi
if [[ -d "$build_dir" ]] &&
   [[ -n "$(find "$build_dir" -mindepth 1 -print -quit)" ]]; then
    echo "Refusing non-empty build directory (no old .o/.P reuse): $build_dir" >&2
    exit 4
fi
if [[ -e "$artifact_dir" || -L "$artifact_dir" ]]; then
    echo "Refusing to overwrite artifact directory: $artifact_dir" >&2
    exit 5
fi
if [[ "$build_dir" == "$artifact_dir" ||
      "$build_dir" == "$artifact_dir"/* ||
      "$artifact_dir" == "$build_dir"/* ]]; then
    echo "Build and artifact directories must not overlap." >&2
    exit 5
fi

jobs="${JOBS:-4}"
python_bin="${PYTHON:-/r/openmv-build-tools/bin/python3}"
mpy_cross="${MICROPY_MPYCROSS:-/r/mpy-cross-1.28.0/mpy-cross.exe}"
sdk_dir="${SDK_DIR:-/r/openmv-sdk-version-check-only}"
make_command="${MAKE:-make}"
if [[ "$jobs" != "4" ]]; then
    echo "This runner fixes four build jobs; got JOBS=$jobs" >&2
    exit 6
fi
command -v "$make_command" >/dev/null 2>&1 || {
    echo "MAKE command not found: $make_command" >&2
    exit 6
}
command -v "$python_bin" >/dev/null 2>&1 || {
    echo "PYTHON command not found: $python_bin" >&2
    exit 6
}
[[ -x "$mpy_cross" ]] || {
    echo "MICROPY_MPYCROSS is not executable: $mpy_cross" >&2
    exit 6
}
[[ -e "$sdk_dir" ]] || {
    echo "SDK_DIR marker/path not found: $sdk_dir" >&2
    exit 6
}

expected_openmv="631681e5ac5ab332c920b6bf9cd3dba5cf299eac"
expected_micropython="dd984559aa42841908b78397b7dac32f57da803f"
[[ "$(git -C "$source_root" rev-parse HEAD)" == "$expected_openmv" ]] || {
    echo "Unexpected OpenMV source commit." >&2
    exit 7
}
[[ "$(git -C "$source_root/lib/micropython" rev-parse HEAD)" == "$expected_micropython" ]] || {
    echo "Unexpected MicroPython source commit." >&2
    exit 7
}

# This runner checks an already-patched tree. It never applies or reverts patches.
git -C "$source_root" apply --reverse --check \
    "$patch_dir/0001-snapshot-preview-control.patch"
git -C "$source_root/lib/micropython" apply --reverse --check -p3 \
    "$patch_dir/0002-stm32h7-uart-fifo-write.patch"
git -C "$source_root" apply --reverse --check \
    "$patch_dir/0003-cartest-accel-module.patch"
git -C "$source_root" apply --reverse --check \
    "$patch_dir/0004-cartest-path-accel-module.patch"
git -C "$source_root" apply --reverse --check \
    "$patch_dir/0005-jpeg-preview-codec-guard.patch"

"$python_bin" - "$source_root" "$project_root" "$runtime_root" \
    "$patch_dir" "$build_dir" "$artifact_dir" "$0" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import sys

source_root, project_root, runtime_root, patch_root, build_dir, artifact_dir, runner_path = map(
    Path, sys.argv[1:])

def fail(message):
    raise SystemExit(message)

def load(path):
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        fail("Cannot read JSON %s: %s" % (path, exc))

def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()

def check_hash(path, expected, label):
    if not path.is_file():
        fail("Missing %s: %s" % (label, path))
    actual = sha(path)
    if actual != str(expected).upper():
        fail("SHA-256 mismatch for %s: %s" % (label, path))
    return actual

manifest1 = load(patch_root / "manifest.json")
manifest2 = load(patch_root / "0002-stm32h7-uart-fifo-write.manifest.json")
manifest3 = load(patch_root / "0003-cartest-accel-module.manifest.json")
manifest4 = load(patch_root / "0004-cartest-path-accel-module.manifest.json")
sourcehash4 = load(patch_root / "0004-cartest-path-accel-module.sourcehash.json")
manifest5 = load(patch_root / "0005-jpeg-preview-codec-guard.manifest.json")

patch_specs = [
    (1, "0001-snapshot-preview-control.patch", manifest1["patch_sha256"]),
    (2, "0002-stm32h7-uart-fifo-write.patch", manifest2["patch_sha256"]),
    (3, "0003-cartest-accel-module.patch", manifest3["patch_sha256"]),
    (4, "0004-cartest-path-accel-module.patch", manifest4["patch_sha256"]),
    (5, "0005-jpeg-preview-codec-guard.patch", manifest5["patch_sha256"]),
]
patch_hashes = {}
for position, filename, expected in patch_specs:
    actual = check_hash(patch_root / filename, expected, "patch %03d" % position)
    patch_hashes[filename] = actual

if manifest1.get("patch") != patch_specs[0][1]:
    fail("0001 manifest names a different patch")
if manifest2.get("applies_after", {}).get("patch_sha256", "").upper() != patch_hashes[patch_specs[0][1]]:
    fail("0002 does not follow the pinned 0001 patch hash")
if manifest3.get("applies_after", {}).get("patch_sha256", "").upper() != patch_hashes[patch_specs[1][1]]:
    fail("0003 does not follow the pinned 0002 patch hash")
if manifest4.get("applies_after", {}).get("patch_sha256", "").upper() != patch_hashes[patch_specs[2][1]]:
    fail("0004 does not follow the pinned 0003 patch hash")
if manifest5.get("applies_after", {}).get("patch_sha256", "").upper() != patch_hashes[patch_specs[3][1]]:
    fail("0005 does not follow the pinned 0004 patch hash")

for manifest in (manifest2, manifest3, manifest4, manifest5):
    base = manifest.get("base", {})
    if base.get("openmv_commit") != "631681e5ac5ab332c920b6bf9cd3dba5cf299eac":
        fail("Patch manifest is not pinned to OpenMV v5.0.1")
    if base.get("micropython_commit") != "dd984559aa42841908b78397b7dac32f57da803f":
        fail("Patch manifest is not pinned to the expected MicroPython commit")

module_sources = {}
for manifest in (manifest3, manifest4):
    for destination, record in manifest.get("files", {}).items():
        source = patch_root / record["source"]
        expected = record.get("source_sha256", record.get("sha256"))
        source_sha = check_hash(source, expected, "module source " + destination)
        tree_sha = check_hash(source_root / destination, expected,
                              "patched-tree module " + destination)
        module_sources[destination] = {
            "source": record["source"],
            "sha256": source_sha,
            "q_tree_sha256": tree_sha,
        }

if sourcehash4.get("patch_sha256", "").upper() != patch_hashes[patch_specs[3][1]]:
    fail("0004 sourcehash patch SHA does not match 0004 manifest")
for record in sourcehash4.get("sources", []):
    source = patch_root / record["source"]
    expected = record["sha256"]
    check_hash(source, expected, "0004 sourcehash source")
    check_hash(source_root / record["destination"], expected,
               "0004 sourcehash destination")

overlay_sources = {}
for record in manifest5.get("files", []):
    source = patch_root / record["source"]
    destination = source_root / record["destination"]
    source_sha = check_hash(source, record["sha256"], "JPEG guard overlay source")
    tree_sha = check_hash(destination, record["sha256"], "JPEG guard source tree")
    overlay_sources[record["destination"]] = {
        "source": record["source"],
        "sha256": source_sha,
        "q_tree_sha256": tree_sha,
    }

board_mk = (source_root / "boards/OPENMV4P/board_config.mk").read_text(
    encoding="utf-8")
board_h = (source_root / "boards/OPENMV4P/board_config.h").read_text(
    encoding="utf-8")
if not re.search(r"^\s*PORT\s*=\s*stm32\s*$", board_mk, re.MULTILINE):
    fail("OPENMV4P board_config.mk does not select PORT=stm32")
if not re.search(r"^\s*#define\s+OMV_JPEG_CODEC_ENABLE\s+\(?1\)?\s*$",
                 board_h, re.MULTILINE):
    fail("OPENMV4P board config does not enable the JPEG codec")

runtime_manifest = load(runtime_root / "PACKAGE_MANIFEST.json")
raw_runtime_files = runtime_manifest.get("runtime_files")
if not isinstance(raw_runtime_files, list) or len(raw_runtime_files) != 23:
    fail("Runtime package must list exactly 23 Python modules")
if raw_runtime_files and isinstance(raw_runtime_files[0], dict):
    runtime_names = [item.get("file") for item in raw_runtime_files]
    runtime_records = {item.get("file"): item for item in raw_runtime_files}
else:
    runtime_names = raw_runtime_files
    runtime_records = {item.get("file"): item for item in
                       runtime_manifest.get("compilation", [])}
if (len(set(runtime_names)) != 23 or
        any(not isinstance(name, str) or Path(name).name != name or
            not name.endswith(".py") for name in runtime_names)):
    fail("Runtime manifest must list 23 unique top-level .py files")
live_py = {path.name for path in runtime_root.glob("*.py")}
if live_py != set(runtime_names):
    fail("Runtime root .py files do not match PACKAGE_MANIFEST.json")

runtime_mismatches = []
runtime_sources = []
for filename in runtime_names:
    source = runtime_root / filename
    actual_source_sha = sha(source)
    record = runtime_records.get(filename, {})
    declared_sha = (record.get("candidate_sha256") or
                    record.get("source_sha256"))
    matches = (actual_source_sha.lower() == declared_sha.lower()
               if declared_sha else None)
    compile_rc = record.get("compile_exit_code", record.get("exit_code"))
    mpy_sha = record.get("mpy_sha256")
    if matches is not True or compile_rc != 0 or not mpy_sha:
        runtime_mismatches.append(filename)
    mpy_path = runtime_root / "validation" / "mpy" / (Path(filename).stem + ".mpy")
    if not mpy_path.is_file():
        mpy_path = runtime_root / (Path(filename).stem + ".mpy")
    mpy_artifact_sha = sha(mpy_path) if mpy_path.is_file() else None
    if mpy_artifact_sha and mpy_sha and mpy_artifact_sha.lower() != mpy_sha.lower():
        runtime_mismatches.append(filename + ":mpy")
    runtime_sources.append({
        "file": filename,
        "sha256": actual_source_sha,
        "manifest_source_sha256": declared_sha,
        "manifest_source_hash_matches": matches,
        "mpy_sha256": mpy_sha,
        "mpy_artifact_sha256": mpy_artifact_sha,
        "compile_exit_code": record.get("compile_exit_code",
                                        record.get("exit_code")),
    })
if runtime_mismatches:
    fail("Runtime source/compile manifest mismatch: " +
         ",".join(runtime_mismatches))

required_qstrs = [
    "cartest_accel", "API_VERSION", "api_version", "region_segments",
    "internal_component", "HEAP_STATS_API", "heap_stats",
    "PREVIEW_CODEC_GUARD_API", "jpeg_encode_timeout_ms",
    "cartest_path_accel", "extract_rows", "connect_rows",
    "WRITE_FIFO_API", "write_fifo", "fifo_txdone",
]
runner_text = runner_path.read_text(encoding="utf-8")
post_python_blocks = re.findall(
    r"<<'PY'\r?\n(.*?)\r?\nPY\s*(?=\r?\n|$)",
    runner_text, re.DOTALL)
if len(post_python_blocks) != 2:
    fail("Expected exactly two embedded Python blocks in the runner")
try:
    compile(post_python_blocks[1], str(runner_path) + ":post-build", "exec")
except SyntaxError as exc:
    fail("Post-build manifest Python has a syntax error: %s" % exc)
print("PRECONDITIONS_PASS=1")
print("OPENMV_HEAD=631681e5ac5ab332c920b6bf9cd3dba5cf299eac")
print("MICROPYTHON_HEAD=dd984559aa42841908b78397b7dac32f57da803f")
print("PATCH_COUNT=5")
print("RUNTIME_SOURCE_HASH_MISMATCH_COUNT=%d" % len(runtime_mismatches))
print("REQUIRED_QSTR_COUNT=%d" % len(required_qstrs))
print("POSTBUILD_PYTHON_SYNTAX_PASS=1")
print("BUILD_DIR=%s" % build_dir)
print("BUILD_DIR_INITIAL_ENTRY_COUNT=0")
print("ARTIFACT_DIR_ABSENT=1")
if runtime_mismatches:
    print("RUNTIME_MANIFEST_DRIFT=" + ",".join(runtime_mismatches))
PY

if [[ "$verify_only" -eq 1 ]]; then
    echo "VERIFY_ONLY=1; MAKE_NOT_RUN=1; NO_OUTPUT_DIRECTORIES_CREATED=1"
    exit 0
fi

mkdir -p -- "$build_dir" "$artifact_dir"
initial_entry="$(find "$build_dir" -mindepth 1 -print -quit)"
if [[ -n "$initial_entry" ]]; then
    echo "Build directory became non-empty before make: $initial_entry" >&2
    exit 8
fi
initial_entry_count=0

record_runner_exit() {
    local rc=$?
    if [[ -d "$artifact_dir" && ! -e "$artifact_dir/runner.exitcode" ]]; then
        printf '%s\n' "$rc" > "$artifact_dir/runner.exitcode"
    fi
}
trap record_runner_exit EXIT

jobs="${JOBS:-4}"
python_bin="${PYTHON:-/r/openmv-build-tools/bin/python3}"
mpy_cross="${MICROPY_MPYCROSS:-/r/mpy-cross-1.28.0/mpy-cross.exe}"
sdk_dir="${SDK_DIR:-/r/openmv-sdk-version-check-only}"
make_command="${MAKE:-make}"
echo "OPENMV_BUILD target=OPENMV4P mcu=STM32H743xx bootloader=disabled"
echo "BUILD_DIR=$build_dir initial_entry_count=0 jobs=$jobs OMV_ENABLE_BL=0"
echo "MAKE=$make_command PYTHON=$python_bin MICROPY_MPYCROSS=$mpy_cross SDK_DIR=$sdk_dir"
echo "PATCH_SERIES=0001,0002,0003,0004,0005; source tree remains unmodified by patch operations"

set +e
"$make_command" --no-print-directory -j4 -C "$source_root" \
    TARGET=OPENMV4P BUILD="$build_dir" OMV_ENABLE_BL=0 \
    SDK_DIR="$sdk_dir" MICROPY_MPYCROSS="$mpy_cross" \
    PYTHON="$python_bin" firmware 2>&1 | tee "$artifact_dir/build.log"
pipeline_status=("${PIPESTATUS[@]}")
make_exit="${pipeline_status[0]:-1}"
tee_exit="${pipeline_status[1]:-1}"
set -e
printf '%s\n' "$make_exit" > "$artifact_dir/build.exitcode"
if [[ "$tee_exit" -ne 0 ]]; then
    printf '%s\n' 1 > "$artifact_dir/runner.exitcode"
    echo "Could not capture the complete build log." >&2
    exit 1
fi
if [[ "$make_exit" -ne 0 ]]; then
    printf '%s\n' "$make_exit" > "$artifact_dir/runner.exitcode"
    echo "make failed with exit code $make_exit; see $artifact_dir/build.log" >&2
    exit "$make_exit"
fi

test -s "$build_dir/bin/firmware.bin"
test -s "$build_dir/bin/firmware.elf"
mkdir -p -- "$artifact_dir"
cp -- "$build_dir/bin/firmware.bin" "$artifact_dir/firmware.bin"
cp -- "$build_dir/bin/firmware.elf" "$artifact_dir/firmware.elf"
cp -- "$build_dir/lib/micropython/genhdr/qstrdefs.generated.h" \
    "$artifact_dir/qstrdefs.generated.h"

set +e
"$python_bin" - "$source_root" "$build_dir" "$artifact_dir" \
    "$project_root" "$runtime_root" "$initial_entry_count" "$jobs" \
    "$python_bin" "$mpy_cross" "$sdk_dir" "$make_command" <<'PY'
import hashlib
import json
from pathlib import Path
import struct
import sys

(source_root, build_dir, artifact_dir, project_root, runtime_root) = map(
    Path, sys.argv[1:6])
initial_entry_count = int(sys.argv[6])
jobs = int(sys.argv[7])
python_bin, mpy_cross, sdk_dir = map(Path, sys.argv[8:11])
make_command = sys.argv[11]
if initial_entry_count != 0:
    raise SystemExit("Build directory was not empty at make start")

def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()

def load(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))

patch_root = project_root / "tools/vendor-patches/openmv-v5.0.1"
manifest_paths = [
    ("0001-snapshot-preview-control.patch", patch_root / "manifest.json"),
    ("0002-stm32h7-uart-fifo-write.patch",
     patch_root / "0002-stm32h7-uart-fifo-write.manifest.json"),
    ("0003-cartest-accel-module.patch",
     patch_root / "0003-cartest-accel-module.manifest.json"),
    ("0004-cartest-path-accel-module.patch",
     patch_root / "0004-cartest-path-accel-module.manifest.json"),
    ("0005-jpeg-preview-codec-guard.patch",
     patch_root / "0005-jpeg-preview-codec-guard.manifest.json"),
]
patch_series = []
for index, (filename, manifest_path) in enumerate(manifest_paths, start=1):
    patch_path = patch_root / filename
    manifest = load(manifest_path)
    if filename == "0001-snapshot-preview-control.patch":
        expected = manifest["patch_sha256"]
    else:
        expected = manifest["patch_sha256"]
    patch_sha = sha(patch_path)
    if patch_sha != expected.upper():
        raise SystemExit("Patch changed during build: " + filename)
    patch_series.append({"position": index, "file": filename,
                         "sha256": patch_sha,
                         "manifest_file": manifest_path.name})

module_sources = {}
for manifest_file in ("0003-cartest-accel-module.manifest.json",
                      "0004-cartest-path-accel-module.manifest.json"):
    manifest = load(patch_root / manifest_file)
    for destination, record in manifest["files"].items():
        source = patch_root / record["source"]
        target = source_root / destination
        expected = record.get("source_sha256", record.get("sha256"))
        source_sha = sha(source)
        tree_sha = sha(target)
        if source_sha != expected.upper() or tree_sha != expected.upper():
            raise SystemExit("Module source changed during build: " + destination)
        module_sources[destination] = {"source": record["source"],
                                       "sha256": source_sha,
                                       "q_tree_sha256": tree_sha}

manifest5 = load(patch_root / "0005-jpeg-preview-codec-guard.manifest.json")
overlay_sources = {}
for record in manifest5["files"]:
    source = patch_root / record["source"]
    target = source_root / record["destination"]
    source_sha = sha(source)
    tree_sha = sha(target)
    if source_sha != record["sha256"].upper() or tree_sha != source_sha:
        raise SystemExit("JPEG guard source changed during build: " +
                         record["destination"])
    overlay_sources[record["destination"]] = {
        "source": record["source"], "sha256": source_sha,
        "q_tree_sha256": tree_sha}

runtime_manifest = load(runtime_root / "PACKAGE_MANIFEST.json")
raw_runtime_files = runtime_manifest["runtime_files"]
if raw_runtime_files and isinstance(raw_runtime_files[0], dict):
    runtime_names = [item["file"] for item in raw_runtime_files]
    runtime_records = {item["file"]: item for item in raw_runtime_files}
else:
    runtime_names = raw_runtime_files
    runtime_records = {item["file"]: item for item in
                       runtime_manifest.get("compilation", [])}
runtime_python_sources = []
runtime_manifest_mismatches = []
for filename in runtime_names:
    source = runtime_root / filename
    source_sha = sha(source)
    record = runtime_records.get(filename, {})
    declared_sha = (record.get("candidate_sha256") or
                    record.get("source_sha256"))
    hash_matches = (source_sha == declared_sha.upper()) if declared_sha else None
    if hash_matches is False:
        runtime_manifest_mismatches.append(filename)
    mpy_expected = record.get("mpy_sha256")
    compile_exit_code = record.get("compile_exit_code", record.get("exit_code"))
    if compile_exit_code != 0 or not mpy_expected:
        runtime_manifest_mismatches.append(filename)
    mpy_path = runtime_root / "validation" / "mpy" / (Path(filename).stem + ".mpy")
    if not mpy_path.is_file():
        mpy_path = runtime_root / (Path(filename).stem + ".mpy")
    mpy_actual = sha(mpy_path) if mpy_path.is_file() else None
    if mpy_actual and mpy_expected and mpy_actual.lower() != mpy_expected.lower():
        runtime_manifest_mismatches.append(filename + ":mpy")
    runtime_python_sources.append({
        "file": filename,
        "sha256": source_sha,
        "manifest_source_sha256": declared_sha,
        "manifest_source_hash_matches": hash_matches,
        "mpy_sha256": mpy_expected,
        "mpy_artifact_sha256": mpy_actual,
        "compile_exit_code": record.get("compile_exit_code",
                                        record.get("exit_code")),
    })
if runtime_manifest_mismatches:
    raise SystemExit("Runtime source/compile manifest changed during build: " +
                     ",".join(runtime_manifest_mismatches))

qstr_path = artifact_dir / "qstrdefs.generated.h"
qstr = qstr_path.read_text(encoding="utf-8")
required_qstrs = [
    "cartest_accel", "API_VERSION", "api_version", "region_segments",
    "internal_component", "HEAP_STATS_API", "heap_stats",
    "PREVIEW_CODEC_GUARD_API", "jpeg_encode_timeout_ms",
    "cartest_path_accel", "extract_rows", "connect_rows",
    "WRITE_FIFO_API", "write_fifo", "fifo_txdone",
]
missing = [name for name in required_qstrs if "MP_QSTR_" + name not in qstr]
if missing:
    raise SystemExit("Missing generated module QSTRs: " + ", ".join(missing))

firmware_bin = artifact_dir / "firmware.bin"
binary = firmware_bin.read_bytes()
if len(binary) < 8:
    raise SystemExit("Firmware binary is too short to contain an app vector")
vector_msp, vector_reset = struct.unpack_from("<II", binary, 0)
app_vector_base = 0x08040000
if vector_reset < app_vector_base:
    raise SystemExit("Firmware bin reset vector is outside the app region")

elf = (artifact_dir / "firmware.elf").read_bytes()
if elf[:4] != b"\x7fELF" or elf[4] != 1 or elf[5] != 1:
    raise SystemExit("Expected little-endian ELF32 firmware")
entry = struct.unpack_from("<I", elf, 24)[0]
phoff = struct.unpack_from("<I", elf, 28)[0]
phentsize = struct.unpack_from("<H", elf, 42)[0]
phnum = struct.unpack_from("<H", elf, 44)[0]
load_addresses = []
for index in range(phnum):
    offset = phoff + index * phentsize
    p_type, _off, p_vaddr, _paddr, p_filesz, _memsz, _flags, _align = \
        struct.unpack_from("<IIIIIIII", elf, offset)
    if p_type == 1 and p_filesz and 0x08000000 <= p_vaddr < 0x09000000:
        load_addresses.append(p_vaddr)
if not load_addresses:
    raise SystemExit("ELF has no flash PT_LOAD segment")
app_load = min(load_addresses)
if app_load != app_vector_base or entry != vector_reset:
    raise SystemExit("ELF/bin application vectors do not agree at 0x08040000")

source_head = "631681e5ac5ab332c920b6bf9cd3dba5cf299eac"
micropython_head = "dd984559aa42841908b78397b7dac32f57da803f"
build_dir_string = str(build_dir)
object_count = len(list(build_dir.rglob("*.o")))
dependency_count = len(list(build_dir.rglob("*.P")))
required_objects = [
    "lib/micropython/modules/py_cartest_accel.o",
    "lib/micropython/modules/cartest_accel_core.o",
    "lib/micropython/modules/py_cartest_path_accel.o",
    "lib/micropython/modules/cartest_path_accel_core.o",
]
missing_objects = [name for name in required_objects
                   if not (build_dir / name).is_file()]
if missing_objects:
    raise SystemExit("Missing C3/C4 module objects: " + ", ".join(missing_objects))

artifacts = {}
for name in ("firmware.bin", "firmware.elf", "qstrdefs.generated.h", "build.log"):
    path = artifact_dir / name
    artifacts[name] = {"bytes": path.stat().st_size, "sha256": sha(path)}

cleanup_limit = manifest5.get("cleanup_limit", {})
manifest = {
    "artifact_class": "OPENMV4P_RING45_NATIVE_APP_ONLY_CLEAN_BUILD",
    "deployment_eligible": False,
    "hardware_touched": False,
    "flash_performed": False,
    "source_root": str(source_root),
    "project_root": str(project_root),
    "runtime_root": str(runtime_root),
    "source_commit": source_head,
    "micropython_commit": micropython_head,
    "target": "OpenMV vendor OPENMV4P / STM32H743xx",
    "board_compatibility_note": (
        "This sample is identified by the vendor tree as OPENMV4P/H743 and "
        "was exercised on a user H750-marked board. It does not establish "
        "generic H750 compatibility."),
    "build": {
        "directory": build_dir_string,
        "initial_entry_count": 0,
        "jobs": jobs,
        "omv_enable_bl": 0,
        "make": str(make_command),
        "make_args": [
            "--no-print-directory", "-j4", "-C", str(source_root),
            "TARGET=OPENMV4P", "BUILD=" + str(build_dir),
            "OMV_ENABLE_BL=0", "SDK_DIR=" + str(sdk_dir),
            "MICROPY_MPYCROSS=" + str(mpy_cross),
            "PYTHON=" + str(python_bin), "firmware",
        ],
        "exit_code": 0,
        "python": str(python_bin),
        "micropython_mpycross": str(mpy_cross),
        "sdk_dir": str(sdk_dir),
        "runtime_object_count": object_count,
        "dependency_file_count": dependency_count,
        "module_objects": required_objects,
    },
    "patch_series": patch_series,
    "module_sources": module_sources,
    "jpeg_guard_overlay_sources": overlay_sources,
    "runtime_python_sources": runtime_python_sources,
    "runtime_manifest_source_mismatch_files": runtime_manifest_mismatches,
    "generated_qstrs": required_qstrs,
    "vectors": {
        "app_vector_base": "0x08040000",
        "bin_vector_msp": "0x%08X" % vector_msp,
        "bin_vector_reset": "0x%08X" % vector_reset,
        "elf_entry": "0x%08X" % entry,
        "app_load_vaddr": "0x%08X" % app_load,
    },
    "artifacts": artifacts,
    "verification": {
        "fresh_empty_build_directory": True,
        "patches_already_applied_and_reverse_checked": True,
        "module_and_overlay_source_hashes_match": True,
        "runtime_source_hashes_recorded_live": True,
        "runtime_package_manifest_matches_all_sources": not bool(runtime_manifest_mismatches),
        "full_qstr_generation": True,
        "app_only_vectors_at_08040000": True,
        "hardware_validation_by_this_runner": False,
        "hal_abort_cleanup_hard_bound_ms": cleanup_limit.get("hal_abort_cleanup_hard_bound_ms"),
        "nominal_hal_abort_cleanup_may_reach_ms": cleanup_limit.get("nominal_hal_abort_cleanup_may_reach_ms"),
        "caller_encode_timeout_ms": cleanup_limit.get("caller_encode_timeout_ms"),
    },
}
(artifact_dir / "build-verification.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
print(json.dumps({
    "build_exit_code": 0,
    "artifact_dir": str(artifact_dir),
    "objects": object_count,
    "dependencies": dependency_count,
    "patch_count": len(patch_series),
    "runtime_source_count": len(runtime_python_sources),
    "runtime_manifest_source_mismatch_count": len(runtime_manifest_mismatches),
    "required_qstr_count": len(required_qstrs),
    "vectors": manifest["vectors"],
    "artifacts": artifacts,
}, indent=2))
PY
post_exit=$?
set -e
if [[ "$post_exit" -ne 0 ]]; then
    printf '%s\n' "$post_exit" > "$artifact_dir/runner.exitcode"
    echo "Post-build verification failed with exit code $post_exit." >&2
    exit "$post_exit"
fi

printf '%s\n' 0 > "$artifact_dir/runner.exitcode"
echo "BUILD_SUCCESS=1 ARTIFACT_DIR=$artifact_dir"
