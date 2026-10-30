#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat >&2 <<'EOF'
Usage: build-openmv4p-app-only.sh [--verify-only]

Builds only the packaged OpenMV vendor OPENMV4P / STM32H743xx firmware app.
It does not flash, upload, reset, or contact hardware. The historical runner
and all dependencies are resolved relative to this exported package.
EOF
}

verify_only=0
if [[ "${1:-}" == "--verify-only" ]]; then
    verify_only=1
    shift
fi
if [[ "$#" -ne 0 ]]; then
    usage
    exit 2
fi

runner_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
package_root="$(cd -- "$runner_dir/.." && pwd -P)"
source_root="$package_root/source/openmv"
tools_root="$package_root/tools"
project_root="$package_root/project"
runtime_root="$project_root/deliverables/TrackProfiles_Ring45_USB60_v16_20261027/OpenMV"
if [[ -f "$project_root/deliverables/TrackProfiles_RingJoint_v25_20261029/OpenMV/PACKAGE_MANIFEST.json" ]]; then
    runtime_root="$project_root/deliverables/TrackProfiles_RingJoint_v25_20261029/OpenMV"
fi
historical_runner="$project_root/tools/vendor-patches/openmv-v5.0.1/build-openmv4p-ring45-native-clean.sh"
build_root_override="${OPENMV_BUILD_ROOT_POSIX:-}"
run_id="${OPENMV_RUN_ID:-$(date -u +'%y%m%d%H%M%S')-$$}"
if [[ -n "$build_root_override" ]]; then
    work_root="$build_root_override"
    build_dir="$work_root"
else
    work_root="$runner_dir/work"
    build_dir="$work_root/b-$run_id"
fi

git_root="$tools_root/git"
arm_bin="$tools_root/arm/bin"
build_tools_bin="$tools_root/build-tools/bin"
python_home="$tools_root/python"
python_exe="$python_home/python.exe"
mpy_cross="$tools_root/mpy-cross-1.28.0/mpy-cross.exe"
sdk_dir="$tools_root/sdk-marker"
make_exe="$build_tools_bin/make.exe"
python_shim="$runner_dir/python3"
cygpath_exe="$git_root/usr/bin/cygpath.exe"
if [[ -n "${OPENMV_RUNTIME_ROOT_NATIVE:-}" ]]; then
    runtime_root="$("$cygpath_exe" -u "$OPENMV_RUNTIME_ROOT_NATIVE")"
fi

required_paths=(
    "$source_root/.git"
    "$source_root/lib/micropython/.git"
    "$source_root/Makefile"
    "$project_root/tools/vendor-patches/openmv-v5.0.1/manifest.json"
    "$historical_runner"
    "$runtime_root/PACKAGE_MANIFEST.json"
    "$arm_bin/arm-none-eabi-gcc.exe"
    "$git_root/usr/bin/bash.exe"
    "$cygpath_exe"
    "$git_root/mingw64/bin/git.exe"
    "$build_tools_bin/make.exe"
    "$python_exe"
    "$mpy_cross"
    "$sdk_dir/sdk.version"
    "$python_shim"
)
for required_path in "${required_paths[@]}"; do
    if [[ ! -e "$required_path" ]]; then
        echo "Required packaged file or directory is missing: $required_path" >&2
        exit 3
    fi
done

if [[ "$(tr -d '\r\n' < "$sdk_dir/sdk.version")" != "$(tr -d '\r\n' < "$source_root/SDK_VERSION")" ]]; then
    echo "SDK marker does not match source SDK_VERSION." >&2
    exit 3
fi

mkdir -p -- "$runner_dir/work" "$work_root" "$runner_dir/artifacts"
artifact_dir="$runner_dir/artifacts/$run_id"
while [[ -e "$artifact_dir" || -L "$artifact_dir" ]]; do
    run_id="$run_id-$RANDOM"
    if [[ -n "$build_root_override" ]]; then
        echo "Artifact path collision for supplied fresh build directory: $artifact_dir" >&2
        exit 4
    fi
    build_dir="$work_root/b-$run_id"
    artifact_dir="$runner_dir/artifacts/$run_id"
done

chmod +x "$python_shim"

system_root="${SystemRoot:-${SYSTEMROOT:-C:\\Windows}}"
case "$system_root" in
    [A-Za-z]:*) system32_native="${system_root%\\}/System32" ;;
    *) system32_native="${system_root%/}/System32" ;;
esac
system32_posix="$("$cygpath_exe" -u "$system32_native")"
export PATH="$runner_dir:$arm_bin:$build_tools_bin:$git_root/usr/bin:$git_root/mingw64/bin:$git_root/cmd:$git_root/bin:$system32_posix"
export JOBS=4
export MAKE="$make_exe"
export PYTHON="$python_shim"
export MICROPY_MPYCROSS="$mpy_cross"
export SDK_DIR="$sdk_dir"

export OPENMV_PYTHON_HOME_WINDOWS="$("$cygpath_exe" -w "$python_home")"
export OPENMV_Q_ROOT_POSIX="$source_root"
export OPENMV_R_ROOT_POSIX="$tools_root"
export PYTHONHOME="$OPENMV_PYTHON_HOME_WINDOWS"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUTF8=1

# Keep all build-time lookups local to the export. Git is restricted to local
# file transport; package managers are set to offline mode and proxy settings
# are removed from this child process.
export GIT_CONFIG_NOSYSTEM=1
export GIT_CONFIG_GLOBAL=/dev/null
export GIT_ALLOW_PROTOCOL=file
export GIT_TERMINAL_PROMPT=0
export GIT_OPTIONAL_LOCKS=0
export PIP_NO_INDEX=1
export UV_OFFLINE=1
export CARGO_NET_OFFLINE=true
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY

if [[ "$verify_only" -eq 1 ]]; then
    echo "PREFLIGHT_ONLY=1 BUILD_DIR=$build_dir ARTIFACT_DIR=$artifact_dir"
    bash "$historical_runner" --verify-only \
        "$source_root" "$build_dir" "$artifact_dir" \
        "$project_root" "$runtime_root"
    exit $?
fi

echo "OPENMV_BUILD target=OPENMV4P mcu=STM32H743xx bootloader=disabled"
echo "OFFLINE_BUILD=1 SDK_DIR=$sdk_dir (version marker only)"
echo "BUILD_DIR=$build_dir ARTIFACT_DIR=$artifact_dir jobs=4"
echo "CHILD_PATH=$PATH"
echo 'HARDWARE_FLASH_UPLOAD=not-performed'
set +e
# Preserve errors before make starts as well as the make/post-build output.
# The historical runner requires its artifact directory to be absent initially.
mkdir -p -- "$runner_dir/diagnostics"
console_log="$runner_dir/diagnostics/console-$run_id.log"
if [[ -e "$console_log" ]]; then
    echo "Refusing to overwrite console log: $console_log" >&2
    exit 4
fi
bash "$historical_runner" \
    "$source_root" "$build_dir" "$artifact_dir" \
    "$project_root" "$runtime_root" 2>&1 | tee "$console_log"
pipeline_codes=("${PIPESTATUS[@]}")
runner_exit_code=${pipeline_codes[0]}
if [[ "${pipeline_codes[1]}" -ne 0 && "$runner_exit_code" -eq 0 ]]; then
    runner_exit_code=1
fi
set -e

mkdir -p -- "$artifact_dir"
cp -- "$console_log" "$artifact_dir/runner-console.log"
if [[ ! -e "$artifact_dir/runner.exitcode" ]]; then
    printf '%s\n' "$runner_exit_code" > "$artifact_dir/runner.exitcode"
fi

if [[ -d "$artifact_dir" ]]; then
    {
        printf 'target=OPENMV4P / STM32H743xx / app-only\n'
        printf 'bootloader=disabled\n'
        printf 'hardware_flash_upload=not-performed\n'
        printf 'source_root=%s\n' "$source_root"
        printf 'source_commit=%s\n' "$(git -C "$source_root" rev-parse HEAD)"
        printf 'micropython_commit=%s\n' "$(git -C "$source_root/lib/micropython" rev-parse HEAD)"
        printf 'runtime_root=%s\n' "$runtime_root"
        printf 'package_root_native=%s\n' "${OPENMV_PACKAGE_ROOT_NATIVE:-$package_root}"
        printf 'build_root_native=%s\n' "${OPENMV_BUILD_ROOT_NATIVE:-$work_root}"
        printf 'build_dir_native=%s\n' "${OPENMV_BUILD_DIR_NATIVE:-$build_dir}"
        printf 'temporary_package_drive=%s:\n' "${OPENMV_PACKAGE_DRIVE:-unset}"
        printf 'temporary_build_drive=%s:\n' "${OPENMV_BUILD_DRIVE:-unset}"
        printf 'artifact_dir=%s\n' "$artifact_dir"
        printf 'build_dir=%s\n' "$build_dir"
        printf 'PATH=%s\n' "$PATH"
        printf 'git_version=%s\n' "$(git --version 2>&1 | head -n 1)"
        printf 'make_version=%s\n' "$("$MAKE" --version 2>&1 | head -n 1)"
        printf 'arm_none_eabi_gcc_version=%s\n' "$(arm-none-eabi-gcc --version 2>&1 | head -n 1)"
        printf 'python_version=%s\n' "$("$PYTHON" --version 2>&1 | head -n 1)"
        printf 'mpy_cross_version=%s\n' "$("$MICROPY_MPYCROSS" --version 2>&1 | head -n 1)"
    } > "$artifact_dir/runner-environment.txt"
fi

exit "$runner_exit_code"
