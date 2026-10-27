"""Validate the explicitly selected OpenMV v5.0.1 native-camera package.

This validator is intentionally independent from the v5.0.0 native-preview
and UART-FIFO delivery rules. The bundle schema is documented in
``SOURCE_MANIFEST_SCHEMA`` and is kept small enough to audit in review.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import struct
from typing import Any


OPENMV_TAG = "v5.0.1"
OPENMV_COMMIT = "631681e5ac5ab332c920b6bf9cd3dba5cf299eac"
MICROPYTHON_COMMIT = "dd984559aa42841908b78397b7dac32f57da803f"
TARGET = "OPENMV4P"
APP_START = 0x08040000
APP_END = 0x08200000
VECTOR_MSP = 0x00010000
ROLLBACK_BIN_SHA256 = "1E471194F0C9B44239AADF70288103DABCCD840B6B884C65827B04A5B33087B7"
ROLLBACK_ELF_SHA256 = "E5371B12CF716AF54E264D50EAF67D931A89E363313A7E571906AEFB19BA86FD"

PATCH_FILES = (
    "0001-snapshot-preview-control.patch",
    "0002-stm32h7-uart-fifo-write.patch",
    "0003-cartest-accel-module.patch",
    "0004-cartest-path-accel-module.patch",
    "0005-jpeg-preview-codec-guard.patch",
)
PATCH_PIN_FILES = (
    "manifest.json",
    "0002-stm32h7-uart-fifo-write.manifest.json",
    "0003-cartest-accel-module.manifest.json",
    "0004-cartest-path-accel-module.manifest.json",
    "0005-jpeg-preview-codec-guard.manifest.json",
)
PATCH_AUX_SOURCEHASH_FILES = ("0004-cartest-path-accel-module.sourcehash.json",)

MODULE_SOURCE_ROOT = "tools/vendor-patches/openmv-v5.0.1"
MODULE_SOURCES = {
    "cartest_accel/core/cartest_accel_core.c":
        "572D9377437F9F67FE765B98A02B5C138D752651C6D96F604F85173CA277DDD2",
    "cartest_accel/core/cartest_accel_core.h":
        "A26463AFFA48F604D91C4AB2BC4214B63148CC4019C925C3A3E1FCDF0CBDD19D",
    "cartest_accel/binding/py_cartest_accel.c":
        "5D8C16AD3E3568DCAFA2D8D063E0839F5BF6B2D2995D7C88850F61394863C864",
    "cartest_path_accel/core/cartest_path_accel_core.c":
        "EA2562C2CCA191AF1524D1982F2037145708BDA8F7215AB725AC27BF35F5FBC1",
    "cartest_path_accel/core/cartest_path_accel_core.h":
        "970145189AB81A9432E5DE219FEE06F6D21860CDB9CF9280A70F06D947DA7237",
    "cartest_path_accel/binding/py_cartest_path_accel.c":
        "3AD6929E4E42039FD1DB02C778F4D91CBD94DAA5D4F0807EE8EBD29CC358AA78",
}
JPEG_GUARD_OVERLAY_SOURCES = {
    "jpeg_preview_guard/overlay/ports/stm32/stm_jpeg.c":
        "44684B59B86F4387E8F60EEFEF21BA7646E793F4FFFD0C38671FC6EAAF70C2DA",
    "jpeg_preview_guard/overlay/lib/imlib/imlib.h":
        "03D2F49CF4C1D0816B5EEF5107DAC47B1D33748DF43813245BCD293D7F742D9E",
    "jpeg_preview_guard/overlay/ports/stm32/jpeg_preview_guard_cleanup.h":
        "6031F7884C76EB3A9458E095646129F0702D8E2C8C65203FDE510DC417B8F94D",
}

REQUIRED_QSTRS = frozenset({
    "cartest_accel", "API_VERSION", "api_version", "region_segments",
    "internal_component", "HEAP_STATS_API", "heap_stats",
    "PREVIEW_CODEC_GUARD_API", "jpeg_encode_timeout_ms",
    "cartest_path_accel", "extract_rows", "connect_rows",
})

PUBLIC_NATIVE_V501_FILES = (
    "README.md",
    "build-verification.json",
    "artifact-manifest.json",
    "source-manifest.json",
    "SHA256SUMS.json",
    "firmware.bin",
    "firmware.elf",
    "rollback/firmware.bin",
    "rollback/firmware.elf",
    "rollback/manifest.json",
)
_EXPECTED_FILE_SET = frozenset(PUBLIC_NATIVE_V501_FILES)
_EXPECTED_DIR_SET = frozenset({"rollback"})

SOURCE_MANIFEST_SCHEMA = {
    "schema_version": 1,
    "base": {
        "tag": OPENMV_TAG,
        "openmv_commit": OPENMV_COMMIT,
        "micropython_commit": MICROPYTHON_COMMIT,
    },
    "target": TARGET,
    "patch_series": "ordered rows: {file, sha256}; exactly PATCH_FILES",
    "module_sources": "six source-relative paths mapped to exact SHA-256 strings",
    "jpeg_guard_overlay_sources": "three source-relative paths mapped to exact SHA-256 strings",
    "vehicle_logs": [],
    "calibration_files": [],
}


def _error(message: str) -> None:
    raise RuntimeError("OpenMV v5.0.1 native-camera delivery: " + message)


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _normal_hash(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", value):
        _error(f"{label} is not a SHA-256 digest")
    return value.upper()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        _error(f"{label} is unreadable or invalid JSON ({type(exc).__name__})")
    if not isinstance(value, dict):
        _error(f"{label} must be a JSON object")
    return value


def _parse_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        _error(f"{label} must be an integer address")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value, 0)
        except ValueError:
            pass
    _error(f"{label} must be an integer or 0x-prefixed address")


def _ensure_no_symlink_components(root: Path, relative: Path, label: str) -> Path:
    current = root
    for component in relative.parts:
        if component in ("", ".", ".."):
            _error(f"{label} has an unsafe path component")
        current = current / component
        if current.is_symlink():
            _error(f"{label} contains a symlink: {current}")
    return current


def _resolve_delivery(source: Path, relative_directory: Path | str) -> tuple[Path, str]:
    source = Path(source).resolve()
    relative = Path(relative_directory)
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        _error("delivery directory must be a safe source-relative path")
    if relative.parts[0].lower() != "deliverables":
        _error("delivery directory must be under deliverables/")
    deliverables = source / "deliverables"
    root = _ensure_no_symlink_components(source, relative, "delivery directory")
    try:
        resolved = root.resolve(strict=True)
        resolved.relative_to(deliverables.resolve(strict=True))
    except (OSError, ValueError):
        _error("delivery directory is missing or resolves outside deliverables/")
    if not resolved.is_dir():
        _error("delivery path is not a directory")
    return resolved, relative.as_posix()


def _verify_exact_tree(root: Path) -> dict[str, Path]:
    actual_files: dict[str, Path] = {}
    actual_dirs: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            _error(f"package contains a symlink: {relative}")
        if path.is_dir():
            actual_dirs.add(relative)
        elif path.is_file():
            actual_files[relative] = path
        else:
            _error(f"package contains a non-regular filesystem entry: {relative}")
    if frozenset(actual_files) != _EXPECTED_FILE_SET:
        missing = sorted(_EXPECTED_FILE_SET - set(actual_files))
        extra = sorted(set(actual_files) - _EXPECTED_FILE_SET)
        _error(f"package file allowlist mismatch; missing={missing}, extra={extra}")
    if frozenset(actual_dirs) != _EXPECTED_DIR_SET:
        _error(f"package directory allowlist mismatch: {sorted(actual_dirs)}")
    return actual_files


def _verify_sha256_sums(files: dict[str, Path]) -> dict[str, str]:
    sums_path = files["SHA256SUMS.json"]
    sums = _load_json(sums_path, "SHA256SUMS.json")
    expected_names = set(files) - {"SHA256SUMS.json"}
    if set(sums) != expected_names:
        _error("SHA256SUMS.json membership must list every other package file exactly once")
    normalized: dict[str, str] = {}
    for name in sorted(expected_names):
        digest = _normal_hash(sums[name], f"SHA256SUMS.json[{name}]")
        actual = _sha256_file(files[name])
        if digest != actual:
            _error(f"SHA256SUMS.json digest mismatch: {name}")
        normalized[name] = digest
    return normalized


def _pin_patch_sources(source: Path) -> tuple[list[dict[str, Any]], dict[str, str], dict[str, str]]:
    vendor_root = source / MODULE_SOURCE_ROOT
    patch_rows: list[dict[str, Any]] = []
    module_sources: dict[str, str] = {}
    overlay_sources: dict[str, str] = {}

    for position, (patch_name, pin_name) in enumerate(zip(PATCH_FILES, PATCH_PIN_FILES), 1):
        pin_path = vendor_root / pin_name
        patch_path = vendor_root / patch_name
        if pin_path.is_symlink() or patch_path.is_symlink() or not pin_path.is_file() or not patch_path.is_file():
            _error(f"pinned patch or source manifest is missing/linked: {patch_name}")
        pin = _load_json(pin_path, pin_name)
        if pin_name == "manifest.json":
            pin_patch_name = pin.get("patch")
            pinned_sha = pin.get("patch_sha256")
            base = pin
        else:
            pin_patch_name = pin.get("patch_file")
            pinned_sha = pin.get("patch_sha256")
            base = pin.get("base", {})
        if pin_patch_name != patch_name:
            _error(f"source pin names a different patch than {patch_name}")
        pinned_commit = base.get("openmv_commit", base.get("commit"))
        if (base.get("tag") != OPENMV_TAG or
                pinned_commit != OPENMV_COMMIT or
                base.get("micropython_commit") != MICROPYTHON_COMMIT):
            _error(f"source pin base does not match {OPENMV_TAG}: {pin_name}")
        actual_sha = _sha256_file(patch_path)
        if _normal_hash(pinned_sha, f"{pin_name}.patch_sha256") != actual_sha:
            _error(f"live patch hash differs from source pin: {patch_name}")
        if pin.get("series_position", position) != position:
            _error(f"patch series position is wrong: {patch_name}")
        patch_rows.append({"file": patch_name, "sha256": actual_sha})

        if patch_name == "0003-cartest-accel-module.patch":
            entries = pin.get("files", [])
            if isinstance(entries, dict):
                entries = list(entries.values())
            if not isinstance(entries, list):
                _error(f"source file pins in {pin_name} are invalid")
            for entry in entries:
                _add_source_hash(module_sources, entry, "source_sha256", pin_name)
        elif patch_name == "0004-cartest-path-accel-module.patch":
            sourcehash_path = vendor_root / PATCH_AUX_SOURCEHASH_FILES[0]
            if sourcehash_path.is_symlink() or not sourcehash_path.is_file():
                _error("C4 sourcehash file is missing or linked")
            sourcehash = _load_json(sourcehash_path, PATCH_AUX_SOURCEHASH_FILES[0])
            if (sourcehash.get("patch_file") != patch_name or
                    sourcehash.get("patch_sha256") != actual_sha or
                    sourcehash.get("series_position") != position or
                    sourcehash.get("base", {}).get("tag") != OPENMV_TAG or
                    sourcehash.get("base", {}).get("target") != TARGET):
                _error("C4 sourcehash metadata does not match its current patch and target")
            entries = sourcehash.get("sources", [])
            if not isinstance(entries, list):
                _error(f"source file pins in {pin_name} are invalid")
            for entry in entries:
                _add_source_hash(module_sources, entry, "sha256", pin_name)
        elif patch_name == "0005-jpeg-preview-codec-guard.patch":
            entries = pin.get("files", [])
            if not isinstance(entries, list):
                _error(f"source file pins in {pin_name} are invalid")
            for entry in entries:
                _add_source_hash(overlay_sources, entry, "sha256", pin_name)

    if module_sources != MODULE_SOURCES:
        _error("live C3/C4 module source hashes differ from pinned module sourcehashes")
    if overlay_sources != JPEG_GUARD_OVERLAY_SOURCES:
        _error("live C5 overlay hashes differ from the reviewed JPEG guard sources")
    return patch_rows, module_sources, overlay_sources


def _add_source_hash(output: dict[str, str], entry: Any, hash_key: str, source_label: str) -> None:
    if not isinstance(entry, dict):
        _error(f"source row in {source_label} is not an object")
    name = entry.get("source")
    digest = _normal_hash(entry.get(hash_key), f"{source_label}:{name}")
    if not isinstance(name, str) or name in output:
        _error(f"duplicate or invalid source path in {source_label}")
    output[name] = digest


def _logical_source_key(name: Any, expected: dict[str, str], label: str) -> str:
    if not isinstance(name, str) or not name:
        _error(f"{label} source path is invalid")
    normalized = name.replace("\\", "/")
    if normalized.startswith("/") or ".." in PurePosixPath(normalized).parts:
        _error(f"{label} source path contains traversal: {name}")
    if normalized.startswith("./"):
        normalized = normalized[2:]
    if normalized in expected:
        return normalized
    matches = [key for key in expected if normalized.endswith("/" + key) or
               PurePosixPath(key).name == PurePosixPath(normalized).name]
    if len(matches) == 1:
        return matches[0]
    _error(f"{label} contains an unknown or ambiguous source path: {name}")


def _normalize_source_map(value: Any, expected: dict[str, str], label: str) -> dict[str, str]:
    result: dict[str, str] = {}
    if isinstance(value, dict):
        rows = list(value.items())
    elif isinstance(value, list):
        rows = []
        for item in value:
            if not isinstance(item, dict):
                _error(f"{label} rows must be objects")
            rows.append((item.get("source", item.get("file", item.get("path"))), item))
    else:
        _error(f"{label} must be a source map or row list")

    for name, record in rows:
        key = _logical_source_key(name, expected, label)
        if key in result:
            _error(f"{label} repeats source {key}")
        if isinstance(record, dict):
            digest_value = record.get("sha256", record.get("source_sha256"))
            qtree = record.get("q_tree_sha256")
            if qtree is not None:
                _normal_hash(qtree, f"{label}:{key}.q_tree_sha256")
        else:
            digest_value = record
        result[key] = _normal_hash(digest_value, f"{label}:{key}.sha256")
    if set(result) != set(expected):
        _error(f"{label} membership does not match the pinned sources")
    for name, digest in expected.items():
        if result[name] != digest:
            _error(f"{label} hash mismatch: {name}")
    return result


def _normalize_patch_rows(value: Any, expected: list[dict[str, Any]], label: str) -> None:
    if not isinstance(value, list):
        _error(f"{label} must be an ordered patch list")
    normalized = []
    for row in value:
        if not isinstance(row, dict):
            _error(f"{label} entries must be objects")
        name = row.get("file", row.get("patch_file"))
        digest = row.get("sha256", row.get("patch_sha256"))
        normalized.append({"file": name, "sha256": _normal_hash(digest, f"{label}:{name}")})
    if normalized != expected:
        _error(f"{label} does not match the current five-patch v5.0.1 series")


def _verify_source_manifest(
    source: Path,
    manifest: dict[str, Any],
    expected_patches: list[dict[str, Any]],
    expected_modules: dict[str, str],
    expected_overlays: dict[str, str],
) -> None:
    base = manifest.get("base")
    if (type(manifest.get("schema_version")) is not int or manifest.get("schema_version") != 1 or
            not isinstance(base, dict) or
            base.get("tag") != OPENMV_TAG or
            base.get("openmv_commit") != OPENMV_COMMIT or
            base.get("micropython_commit") != MICROPYTHON_COMMIT or
            manifest.get("target") != TARGET):
        _error("source-manifest.json upstream or target does not match the pinned v5.0.1 source")
    _normalize_patch_rows(manifest.get("patch_series"), expected_patches, "source-manifest patch_series")
    _normalize_source_map(manifest.get("module_sources"), expected_modules, "source-manifest module_sources")
    _normalize_source_map(
        manifest.get("jpeg_guard_overlay_sources"), expected_overlays,
        "source-manifest jpeg_guard_overlay_sources")
    if manifest.get("vehicle_logs") != [] or manifest.get("calibration_files") != []:
        _error("source-manifest must not include vehicle logs or calibration payloads")
    if "runtime_python_sources" in manifest:
        _error("source-manifest must not pin runtime Python files before promotion")

    for logical, expected in {**expected_modules, **expected_overlays}.items():
        path = source / MODULE_SOURCE_ROOT / logical
        if path.is_symlink() or not path.is_file():
            _error(f"live source file is missing or linked: {logical}")
        if _sha256_file(path) != expected:
            _error(f"live source file hash does not match its source pin: {logical}")


def _normalize_runtime_rows(value: Any, expected_names: set[str]) -> dict[str, dict[str, Any]]:
    if isinstance(value, dict):
        rows = [{"file": name, **(record if isinstance(record, dict) else {"sha256": record})}
                for name, record in value.items()]
    elif isinstance(value, list):
        rows = value
    else:
        _error("build-verification runtime_python_sources must be a list or object")
    normalized: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict):
            _error("runtime_python_sources rows must be objects")
        name = row.get("file", row.get("source"))
        if not isinstance(name, str):
            _error("runtime_python_sources contains a row without a file")
        short_name = PurePosixPath(name.replace("\\", "/")).name
        if short_name not in expected_names or short_name in normalized:
            _error(f"runtime_python_sources has an unknown or duplicate file: {name}")
        digest = _normal_hash(row.get("sha256", row.get("source_sha256")), f"runtime:{short_name}")
        mpy_digest = _normal_hash(row.get("mpy_sha256"), f"runtime:{short_name}.mpy_sha256")
        compile_exit_code = row.get("compile_exit_code", row.get("exit_code"))
        if type(compile_exit_code) is not int or compile_exit_code != 0:
            _error(f"runtime Python compile did not succeed: {short_name}")
        normalized[short_name] = {"sha256": digest, "mpy_sha256": mpy_digest}
    if set(normalized) != expected_names:
        _error("runtime_python_sources does not enumerate the current 23-file runtime")
    return normalized


def _verify_runtime_sources(source: Path, build_verification: dict[str, Any]) -> None:
    package_manifest_path = source / "openmv" / "PACKAGE_MANIFEST.json"
    if package_manifest_path.is_symlink() or not package_manifest_path.is_file():
        _error("live OpenMV PACKAGE_MANIFEST.json is missing or linked")
    runtime_manifest = _load_json(package_manifest_path, "live OpenMV PACKAGE_MANIFEST.json")
    raw_runtime_rows = runtime_manifest.get("runtime_files")
    if not isinstance(raw_runtime_rows, list) or len(raw_runtime_rows) != 23:
        _error("live OpenMV manifest must enumerate exactly 23 runtime source files")

    package_rows: dict[str, dict[str, Any]] | None = None
    if all(isinstance(name, str) for name in raw_runtime_rows):
        runtime_names = raw_runtime_rows
    elif all(isinstance(row, dict) for row in raw_runtime_rows):
        package_rows = {}
        runtime_names = []
        for row in raw_runtime_rows:
            name = row.get("file")
            if (not isinstance(name, str) or
                    PurePosixPath(name.replace("\\", "/")).name != name or
                    name in package_rows):
                _error("live OpenMV manifest has an unsafe or duplicate runtime source row")
            package_rows[name] = row
            runtime_names.append(name)
        if (runtime_manifest.get("experiment") != "pipeline_perf_v16_codec_guard" or
                runtime_manifest.get("candidate_only") is not True or
                runtime_manifest.get("hardware_validation") is not False or
                runtime_manifest.get("deployment_eligible") is not False or
                runtime_manifest.get("runtime_file_count") != 23):
            _error("v16 object-row package manifest does not preserve its candidate-only metadata")
    else:
        _error("live OpenMV manifest runtime_files must use the pinned string or v16 row schema")

    if (len(set(runtime_names)) != 23 or
            any(PurePosixPath(name.replace("\\", "/")).name != name for name in runtime_names)):
        _error("live OpenMV manifest must enumerate exactly 23 safe, unique runtime source names")
    rows = _normalize_runtime_rows(
        build_verification.get("runtime_python_sources"), set(runtime_names))

    if package_rows is not None:
        for name, package_row in package_rows.items():
            source_sha = _normal_hash(
                package_row.get("candidate_sha256"),
                f"live OpenMV manifest:{name}.candidate_sha256")
            mpy_sha = _normal_hash(
                package_row.get("mpy_sha256"),
                f"live OpenMV manifest:{name}.mpy_sha256")
            compile_exit_code = package_row.get("compile_exit_code")
            if (source_sha != rows[name]["sha256"] or
                    mpy_sha != rows[name]["mpy_sha256"] or
                    type(compile_exit_code) is not int or compile_exit_code != 0):
                _error(f"v16 package-manifest source/MPY record differs from build-verification: {name}")

    for name in runtime_names:
        path = source / "openmv" / name
        if path.is_symlink() or not path.is_file():
            _error(f"live OpenMV runtime source is missing or linked: {name}")
        if _sha256_file(path) != rows[name]["sha256"]:
            _error(f"build-verification runtime source hash differs from the main package: {name}")


def _normalize_artifact_rows(value: Any, label: str) -> dict[str, dict[str, Any]]:
    if not isinstance(value, dict):
        _error(f"{label} must be an artifact map")
    normalized: dict[str, dict[str, Any]] = {}
    for name, record in value.items():
        if not isinstance(name, str) or not isinstance(record, dict):
            _error(f"{label} entries must be named objects")
        short_name = PurePosixPath(name.replace("\\", "/")).name
        if short_name in normalized:
            _error(f"{label} repeats {short_name}")
        normalized[short_name] = record
    return normalized


def _verify_build_artifacts(files: dict[str, Path], build_verification: dict[str, Any]) -> None:
    artifacts = build_verification.get("artifacts", build_verification.get("outputs"))
    rows = _normalize_artifact_rows(artifacts, "build-verification artifacts")
    for name in ("firmware.bin", "firmware.elf"):
        record = rows.get(name)
        if record is None:
            _error(f"build-verification has no {name} output record")
        content = files[name].read_bytes()
        if record.get("bytes") != len(content):
            _error(f"build-verification byte count mismatch: {name}")
        digest = record.get("sha256", record.get("SHA256"))
        if _normal_hash(digest, f"build-verification:{name}") != _sha256_bytes(content):
            _error(f"build-verification hash mismatch: {name}")


def _verify_build_verification(
    source: Path,
    files: dict[str, Path],
    build_verification: dict[str, Any],
    expected_patches: list[dict[str, Any]],
    expected_modules: dict[str, str],
    expected_overlays: dict[str, str],
    candidate_pair: dict[str, int],
) -> None:
    base = build_verification.get("base", {})
    openmv_commit = build_verification.get("source_commit", base.get("openmv_commit"))
    micropython_commit = build_verification.get("micropython_commit", base.get("micropython_commit"))
    target = build_verification.get("target")
    target_ok = (target == TARGET or
                 (isinstance(target, str) and target.startswith(TARGET + " / ")))
    schema_version = build_verification.get("schema_version", 1)
    if (type(schema_version) is not int or schema_version != 1 or
            not target_ok or
            openmv_commit != OPENMV_COMMIT or micropython_commit != MICROPYTHON_COMMIT):
        _error("build-verification target or upstream commits do not match the pinned build")

    if build_verification.get("deployment_eligible") is not False:
        _error("build-verification must mark the image as not deployment eligible")
    if build_verification.get("hardware_touched") is not False:
        _error("build-verification must record no hardware actions during compilation")
    if build_verification.get("flash_performed") is not False:
        _error("build-verification must record flash_performed=false")

    build = build_verification.get("build")
    if (not isinstance(build, dict) or type(build.get("exit_code")) is not int or
            build.get("exit_code") != 0):
        _error("build-verification must explicitly record build.exit_code=0")
    if type(build.get("initial_file_count")) is not int or build.get("initial_file_count") != 0:
        _error("build-verification must identify a fresh empty build directory")
    if type(build.get("runtime_object_count")) is not int or build["runtime_object_count"] <= 0:
        _error("build-verification runtime object count is missing or empty")
    if type(build.get("dependency_file_count")) is not int or build["dependency_file_count"] <= 0:
        _error("build-verification dependency file count is missing or empty")

    verification = build_verification.get("verification")
    if not isinstance(verification, dict):
        _error("build-verification verification section is missing")
    full_qstr = verification.get("full_qstr_generation", verification.get("qstr_full_generation"))
    if full_qstr is not True:
        _error("build-verification does not prove full QSTR generation")
    fresh_build = (verification.get("fresh_build_no_old_objects") is True or
                   verification.get("fresh_empty_build_directory") is True)
    if not fresh_build:
        _error("build-verification does not prove a fresh build without old objects")
    if verification.get("hardware_validation") is not False:
        _error("build-verification hardware_validation must remain false")
    if ("flash_performed" in verification and
            verification.get("flash_performed") is not False):
        _error("build-verification flash_performed must remain false")

    _normalize_patch_rows(build_verification.get("patch_series"), expected_patches,
                          "build-verification patch_series")
    _normalize_source_map(build_verification.get("module_sources"), expected_modules,
                          "build-verification module_sources")
    _normalize_source_map(build_verification.get("jpeg_guard_overlay_sources"), expected_overlays,
                          "build-verification jpeg_guard_overlay_sources")
    _verify_runtime_sources(source, build_verification)

    if ("make_exit_code" in verification and
            (type(verification.get("make_exit_code")) is not int or
             verification.get("make_exit_code") != 0)):
        _error("build-verification make_exit_code is not zero")

    qstr_names = build_verification.get("generated_qstrs")
    if not isinstance(qstr_names, list) or any(not isinstance(name, str) for name in qstr_names):
        _error("build-verification generated_qstrs must be a complete name list")
    if not REQUIRED_QSTRS.issubset(set(qstr_names)):
        _error("build-verification QSTR list is missing cartest JPEG codec API names")

    vectors = build_verification.get("vectors")
    if not isinstance(vectors, dict):
        _error("build-verification vectors record is missing")
    vector_values = {
        "app_vector_base": APP_START,
        "bin_vector_msp": candidate_pair["vector_msp"],
        "bin_vector_reset": candidate_pair["vector_reset"],
        "elf_entry": candidate_pair["entry"],
        "app_load_vaddr": APP_START,
    }
    for name, expected in vector_values.items():
        if _parse_int(vectors.get(name), f"vectors.{name}") != expected:
            _error(f"build-verification vector does not match the image: {name}")

    _verify_build_artifacts(files, build_verification)


def _verify_elf_bin_pair(firmware_bin: bytes, firmware_elf: bytes, label: str) -> dict[str, int]:
    if len(firmware_bin) < 8 or len(firmware_bin) > APP_END - APP_START:
        _error(f"{label} firmware.bin length is outside the application range")
    if (len(firmware_elf) < 52 or firmware_elf[:4] != b"\x7fELF" or
            firmware_elf[4] != 1 or firmware_elf[5] != 1 or firmware_elf[6] != 1):
        _error(f"{label} firmware.elf must be a little-endian ELF32 image")
    elf_type, machine, elf_version, entry = struct.unpack_from("<HHII", firmware_elf, 16)
    if elf_type != 2 or machine != 40 or elf_version != 1:
        _error(f"{label} firmware.elf must be ARM ELF32 ET_EXEC")
    vector_msp, vector_reset = struct.unpack_from("<II", firmware_bin, 0)
    if vector_msp != VECTOR_MSP:
        _error(f"{label} firmware.bin initial MSP does not match OPENMV4P")
    if (vector_reset != entry or not (APP_START <= (vector_reset & ~1) < APP_END) or
            not (APP_START <= (entry & ~1) < APP_END)):
        _error(f"{label} firmware.bin reset vector and ELF entry disagree or leave app flash")

    phoff = struct.unpack_from("<I", firmware_elf, 28)[0]
    phentsize, phnum = struct.unpack_from("<HH", firmware_elf, 42)
    if phnum == 0 or phentsize < 32 or phoff + phentsize * phnum > len(firmware_elf):
        _error(f"{label} firmware.elf program-header table is invalid")
    load_count = 0
    compared_bytes = 0
    for index in range(phnum):
        offset = phoff + index * phentsize
        p_type, p_offset, _vaddr, paddr, filesz, memsz, _flags, _align = struct.unpack_from(
            "<IIIIIIII", firmware_elf, offset)
        if p_type != 1 or filesz == 0:
            continue
        load_count += 1
        if memsz < filesz or p_offset + filesz > len(firmware_elf):
            _error(f"{label} ELF PT_LOAD file/memory bounds are invalid")
        if paddr < APP_START or paddr + filesz > APP_END:
            _error(f"{label} non-empty ELF PT_LOAD is outside app flash")
        bin_offset = paddr - APP_START
        if bin_offset + filesz > len(firmware_bin):
            _error(f"{label} firmware.bin does not cover an ELF PT_LOAD address")
        elf_bytes = firmware_elf[p_offset:p_offset + filesz]
        if firmware_bin[bin_offset:bin_offset + filesz] != elf_bytes:
            _error(f"{label} firmware.bin bytes differ from ELF PT_LOAD at 0x{paddr:08X}")
        compared_bytes += filesz
    if not load_count or not compared_bytes:
        _error(f"{label} firmware.elf has no non-empty app flash PT_LOAD segments")
    return {"entry": entry, "vector_msp": vector_msp,
            "vector_reset": vector_reset, "load_segments": load_count,
            "compared_load_bytes": compared_bytes}


def _verify_artifact_manifest(
    manifest: dict[str, Any],
    package_name: str,
    files: dict[str, Path],
) -> None:
    if (type(manifest.get("schema_version")) is not int or manifest.get("schema_version") != 1 or
            manifest.get("package_id") != package_name or
            manifest.get("target") != TARGET or
            manifest.get("artifact_class") != "bench-only-road-pending" or
            manifest.get("motor_start") is not False or
            manifest.get("road_acceptance") is not False or
            manifest.get("deployment_eligible") is not False):
        _error("artifact-manifest must identify a bench-only, road-pending, motor_start=false package")
    if (_parse_int(manifest.get("application_base"), "application_base") != APP_START or
            _parse_int(manifest.get("application_end"), "application_end") != APP_END):
        _error("artifact-manifest application flash range does not match OPENMV4P")

    expected_hashes = {
        "firmware.bin": _sha256_file(files["firmware.bin"]),
        "firmware.elf": _sha256_file(files["firmware.elf"]),
        "source-manifest.json": _sha256_file(files["source-manifest.json"]),
        "build-verification.json": _sha256_file(files["build-verification.json"]),
        "rollback/firmware.bin": _sha256_file(files["rollback/firmware.bin"]),
        "rollback/firmware.elf": _sha256_file(files["rollback/firmware.elf"]),
        "rollback/manifest.json": _sha256_file(files["rollback/manifest.json"]),
    }
    declared_hashes = manifest.get("files_sha256")
    if not isinstance(declared_hashes, dict) or set(declared_hashes) != set(expected_hashes):
        _error("artifact-manifest files_sha256 membership is incomplete or has extras")
    for name, actual in expected_hashes.items():
        if _normal_hash(declared_hashes[name], f"artifact-manifest:{name}") != actual:
            _error(f"artifact-manifest hash mismatch: {name}")

    evidence = manifest.get("evidence_refs")
    if not isinstance(evidence, dict) or set(evidence) != {"flash", "readback", "bench"}:
        _error("artifact-manifest must provide hash-only flash, readback, and bench references")
    for kind, record in evidence.items():
        if not isinstance(record, dict) or set(record) != {"sha256", "result"}:
            _error(f"artifact-manifest {kind} reference must contain only sha256 and result")
        _normal_hash(record.get("sha256"), f"artifact-manifest evidence_refs.{kind}")
        if not isinstance(record.get("result"), str) or not record["result"].strip():
            _error(f"artifact-manifest evidence_refs.{kind}.result must be a short result string")


def _verify_rollback(files: dict[str, Path]) -> None:
    rollback_bin = files["rollback/firmware.bin"].read_bytes()
    rollback_elf = files["rollback/firmware.elf"].read_bytes()
    if _sha256_bytes(rollback_bin) != ROLLBACK_BIN_SHA256:
        _error("rollback firmware.bin is not the confirmed current v5.0.1 image")
    if _sha256_bytes(rollback_elf) != ROLLBACK_ELF_SHA256:
        _error("rollback firmware.elf is not the confirmed current v5.0.1 image")
    rollback_manifest = _load_json(files["rollback/manifest.json"], "rollback/manifest.json")
    rollback_hashes = rollback_manifest.get("files_sha256")
    if (not isinstance(rollback_hashes, dict) or
            set(rollback_hashes) != {"firmware.bin", "firmware.elf"} or
            _normal_hash(rollback_hashes.get("firmware.bin"), "rollback firmware.bin pin") != ROLLBACK_BIN_SHA256 or
            _normal_hash(rollback_hashes.get("firmware.elf"), "rollback firmware.elf pin") != ROLLBACK_ELF_SHA256 or
            type(rollback_manifest.get("schema_version")) is not int or
            rollback_manifest.get("schema_version") != 1 or
            rollback_manifest.get("target") != TARGET or
            rollback_manifest.get("provenance") != "confirmed-current-j-image"):
        _error("rollback manifest does not pin the confirmed current j-image pair")
    _verify_elf_bin_pair(rollback_bin, rollback_elf, "rollback")


def verify_native_camera_v501_delivery(
    source: Path,
    relative_directory: Path | str,
) -> list[str]:
    """Validate and return only the explicit OpenMV v5.0.1 native app bundle."""
    source = Path(source).resolve()
    root, relative_name = _resolve_delivery(source, relative_directory)
    files = _verify_exact_tree(root)
    _verify_sha256_sums(files)

    expected_patches, expected_modules, expected_overlays = _pin_patch_sources(source)
    source_manifest = _load_json(files["source-manifest.json"], "source-manifest.json")
    _verify_source_manifest(source, source_manifest, expected_patches,
                            expected_modules, expected_overlays)

    build_verification = _load_json(files["build-verification.json"], "build-verification.json")
    candidate_pair = _verify_elf_bin_pair(files["firmware.bin"].read_bytes(),
                                          files["firmware.elf"].read_bytes(), "candidate")
    _verify_build_verification(source, files, build_verification,
                               expected_patches, expected_modules, expected_overlays,
                               candidate_pair)

    artifact_manifest = _load_json(files["artifact-manifest.json"], "artifact-manifest.json")
    _verify_artifact_manifest(artifact_manifest, root.name, files)
    _verify_rollback(files)
    return [f"{relative_name}/{name}" for name in PUBLIC_NATIVE_V501_FILES]

