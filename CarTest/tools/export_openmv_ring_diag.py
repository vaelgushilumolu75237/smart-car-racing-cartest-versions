#!/usr/bin/env python3
"""Read-only host exporter for OpenMV RNGD ring-diagnostic files.

The tool never opens a camera connection and never modifies its input. It
requires a separate output directory for the JSON summary and nonempty CSV.
"""

import argparse
import csv
import importlib.util
import json
import os
from pathlib import Path
import re
import struct
import sys


TOOL_PATH = Path(__file__).resolve()
PROJECT_ROOT = TOOL_PATH.parents[1]
DEFAULT_RUNTIME_ROOT = PROJECT_ROOT / "openmv"
NUMBERED_NAME = re.compile(r"^ring_diagnostics_([0-9]{6})\.bin$")

_BASE_COLUMNS = (
    "elapsed_ms", "frame_sequence", "profile_id", "active_follow",
    "observation_valid", "source_code", "selection_reason_code",
    "tracker_state_id", "tracker_reason_code", "candidate",
    "ring_confirmed", "side", "arc_rows", "path_y_span",
    "target_count", "observed_rows", "inferred_rows", "reject_reason_code",
    "reject_total", "ring_calibration_ready",
    "ring_calibration_check_frames", "ring_calibration_support_rows",
    "ring_calibration_span_rows", "ring_calibration_error_permille",
    "entry_geometry_ok", "current_geometry_ok", "route_authorized",
    "arc_y_min", "arc_y_max", "vertex_y", "vertex_x", "intrusion",
    "port_x", "port_y", "ring_diag_reason_code", "ring_diag_state_id",
    "ring_diag_candidate_side", "ring_diag_calibration_ready",
    "ring_diag_calibration_reason_code", "ring_diag_startup_check_ready",
    "ring_diag_startup_reason_code", "ring_diag_stable_frames",
    "ring_diag_support_rows", "ring_diag_span_rows",
    "ring_diag_median_error_permille", "ring_diag_midfar_arc_rows",
    "ring_diag_near_anchor_rows", "ring_diag_exit_fit_rows",
)

_RING_DIAG_COLUMNS = (
    "ring_diag_reason_code", "ring_diag_state_id", "ring_diag_candidate_side",
    "ring_diag_calibration_ready", "ring_diag_calibration_reason_code",
    "ring_diag_startup_check_ready", "ring_diag_startup_reason_code",
    "ring_diag_stable_frames", "ring_diag_support_rows",
    "ring_diag_span_rows", "ring_diag_median_error_permille",
    "ring_diag_midfar_arc_rows", "ring_diag_near_anchor_rows",
    "ring_diag_exit_fit_rows",
)


def _code_name(value, choices):
    if value == 0 and choices and choices[0] == "":
        return "NONE"
    if 0 <= value < len(choices):
        return choices[value] or "NONE"
    return "UNKNOWN_%d" % value


def _load_runtime(runtime_root=None):
    root = Path(runtime_root).expanduser().resolve() if runtime_root else \
        DEFAULT_RUNTIME_ROOT.resolve()
    module_path = root / "ring_diagnostics.py"
    if not module_path.is_file():
        raise FileNotFoundError("ring_diagnostics.py not found under %s" % root)
    spec = importlib.util.spec_from_file_location(
        "_openmv_ring_diagnostics_decoder", module_path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load ring_diagnostics.py from %s" % root)
    module = importlib.util.module_from_spec(spec)
    previous_bytecode_setting = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous_bytecode_setting
    if not callable(getattr(getattr(module, "RingDiagnostics", None),
                            "decode_file", None)):
        raise ImportError("ring_diagnostics.RingDiagnostics.decode_file is missing")
    return module, root


def _episode_kind(format_version, flags, candidate_count, confirmed_count,
                  stop_snapshot_flag):
    if format_version == 2 and flags & stop_snapshot_flag:
        return "STOP_SNAPSHOT"
    if candidate_count or confirmed_count:
        return "RING_CANDIDATE"
    return "EPISODE"


def _decode_candidate(data, decoder):
    """Call the runtime decoder, then validate and unpack every bounded record."""
    header_size = getattr(decoder, "HEADER_SIZE", 32)
    record_size = getattr(decoder, "FRAME_RECORD_SIZE", 122)
    base_field_count = getattr(decoder, "BASE_FIELD_COUNT", 48)
    reject_count = getattr(decoder, "REJECT_COUNT", 13)
    episode_capacity = getattr(decoder, "EPISODE_CAPACITY", 8)
    pre_capacity = getattr(decoder, "PRE_FRAMES", 64)
    post_capacity = getattr(decoder, "POST_FRAMES", 192)
    max_file_bytes = getattr(decoder, "MAX_FILE_BYTES", 262144)
    build_id_bytes = getattr(decoder, "BUILD_ID_BYTES", 63)
    episode_format = getattr(decoder, "EPISODE_META_FORMAT", "<HHHHHIHHHHH")
    episode_meta_size = getattr(decoder, "EPISODE_META_SIZE", 24)
    stop_snapshot_flag = getattr(decoder, "STOP_SNAPSHOT_FLAG", 1)
    expected_record_size = base_field_count * 2 + reject_count * 2

    if len(data) < header_size or len(data) > max_file_bytes:
        raise ValueError("FILE_SIZE_OUT_OF_BOUNDS")
    declared_header_size = struct.unpack_from("<H", data, 6)[0]
    declared_payload_size = struct.unpack_from("<I", data, 24)[0]
    if declared_header_size != header_size:
        raise ValueError("INVALID_HEADER_SIZE")
    declared_total_size = declared_header_size + declared_payload_size
    if declared_total_size > max_file_bytes:
        raise ValueError("FILE_SIZE_OUT_OF_BOUNDS")
    if len(data) < declared_total_size:
        raise ValueError("TRUNCATED_FILE")
    if len(data) > declared_total_size:
        raise ValueError("TRAILING_FILE_BYTES")
    if len(data) >= 6:
        raw_version = struct.unpack_from("<H", data, 4)[0]
        if raw_version == 2 and getattr(decoder, "FORMAT_VERSION", 1) == 1:
            raise ValueError("RUNTIME_DECODER_DOES_NOT_SUPPORT_V2")
    runtime_summary = decoder.RingDiagnostics.decode_file(data)
    if not isinstance(runtime_summary, dict):
        raise ValueError("runtime decoder returned no summary")
    magic, version, declared_header, declared_record, declared_rejects, \
        declared_capacity, episode_count, declared_pre, declared_post, \
        episode_overflow, freeze_reason, payload_size, expected_crc = \
        struct.unpack_from("<4s10H2I", data, 0)
    if magic != b"RNGD":
        raise ValueError("MAGIC_MISMATCH")
    if version not in (1, 2):
        raise ValueError("UNSUPPORTED_FORMAT_VERSION_%d" % version)
    if (declared_header != header_size or declared_record != record_size or
            declared_record != expected_record_size or
            declared_rejects != reject_count or
            declared_capacity != episode_capacity or
            declared_pre != pre_capacity or declared_post != post_capacity or
            episode_count > episode_capacity or
            payload_size > max_file_bytes - header_size or
            len(data) != header_size + payload_size):
        raise ValueError("INVALID_HEADER_OR_SIZE")
    if runtime_summary.get("format_version") != version:
        raise ValueError("RUNTIME_DECODER_VERSION_MISMATCH")
    if runtime_summary.get("record_size") != record_size:
        raise ValueError("RUNTIME_DECODER_RECORD_SIZE_MISMATCH")
    if runtime_summary.get("episode_count") != episode_count:
        raise ValueError("RUNTIME_DECODER_EPISODE_COUNT_MISMATCH")
    if runtime_summary.get("payload_size") != payload_size:
        raise ValueError("RUNTIME_DECODER_PAYLOAD_SIZE_MISMATCH")

    payload = data[header_size:]
    # decode_file performs CRC validation; retain an independent field check so
    # a decoder implementation cannot silently accept a mismatched header CRC.
    crc_fn = getattr(decoder, "_crc32", None)
    if not callable(crc_fn):
        raise ValueError("RUNTIME_DECODER_CRC_FUNCTION_MISSING")
    actual_crc = int(crc_fn(payload)) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("CRC_MISMATCH")

    payload_prefix_size = build_id_bytes + 3
    if len(payload) < payload_prefix_size:
        raise ValueError("TRUNCATED_PAYLOAD_PREFIX")
    build_length = payload[0]
    if build_length > build_id_bytes:
        raise ValueError("INVALID_BUILD_ID_LENGTH")
    payload_overflow = struct.unpack_from("<H", payload,
                                          build_id_bytes + 1)[0]
    if payload_overflow != episode_overflow:
        raise ValueError("EPISODE_OVERFLOW_HEADER_MISMATCH")
    build_id = bytes(payload[1:1 + build_length]).decode("ascii", "replace")
    episode_offset = payload_prefix_size
    rows = []
    episodes = []
    for episode_index in range(episode_count):
        if episode_offset + episode_meta_size > len(payload):
            raise ValueError("TRUNCATED_EPISODE_METADATA_%d" % episode_index)
        meta = struct.unpack_from(episode_format, payload, episode_offset)
        (frame_count, pre_count, post_count, candidate_count, confirmed_count,
         start_frame, start_sequence, end_sequence, last_state_id, flags,
         dropped_count) = meta
        if (frame_count <= 0 or frame_count > pre_capacity + post_capacity or
                pre_count > pre_capacity or post_count > post_capacity or
                frame_count != pre_count + post_count):
            raise ValueError("INVALID_EPISODE_BOUNDS_%d" % episode_index)
        if version == 2:
            if flags & ~stop_snapshot_flag:
                raise ValueError("INVALID_EPISODE_FLAGS_%d" % episode_index)
            if flags & stop_snapshot_flag:
                if (pre_count <= 0 or post_count != 0 or
                        candidate_count != 0 or confirmed_count != 0):
                    raise ValueError("INVALID_STOP_SNAPSHOT_%d" % episode_index)
        else:
            # Field 9 was reserved in V1 and must not be interpreted as V2 flags.
            flags = 0
        episode_offset += episode_meta_size
        episode_bytes = frame_count * record_size
        if episode_offset + episode_bytes > len(payload):
            raise ValueError("TRUNCATED_EPISODE_RECORDS_%d" % episode_index)
        episode_info = {
            "episode_index": episode_index,
            "kind": _episode_kind(version, flags, candidate_count,
                                   confirmed_count, stop_snapshot_flag),
            "flags": flags,
            "frame_count": frame_count,
            "pre_count": pre_count,
            "post_count": post_count,
            "candidate_count": candidate_count,
            "confirmed_count": confirmed_count,
            "start_frame": start_frame,
            "start_sequence": start_sequence,
            "end_sequence": end_sequence,
            "last_state_id": last_state_id,
            "dropped_count": dropped_count,
        }
        episodes.append(episode_info)
        for frame_index in range(frame_count):
            offset = episode_offset + frame_index * record_size
            values = struct.unpack_from("<%dh" % base_field_count, payload,
                                        offset)
            rejects = struct.unpack_from(
                "<%dH" % reject_count, payload,
                offset + base_field_count * 2)
            elapsed_raw = values[0]
            if elapsed_raw < 0:
                raise ValueError("NEGATIVE_ELAPSED_TIME_EPISODE_%d_FRAME_%d" %
                                 (episode_index, frame_index))
            unit_ms = 20 if version == 2 else 1
            row = {
                "episode_index": episode_index,
                "episode_kind": episode_info["kind"],
                "episode_flags": flags,
                "episode_frame_index": frame_index,
                "episode_frame_count": frame_count,
                "episode_pre_count": pre_count,
                "episode_post_count": post_count,
                "episode_candidate_count": candidate_count,
                "episode_confirmed_count": confirmed_count,
                "episode_start_frame": start_frame,
                "episode_start_sequence": start_sequence,
                "episode_end_sequence": end_sequence,
                "episode_last_state_id": last_state_id,
                "episode_dropped_count": dropped_count,
                "elapsed_ms": elapsed_raw * unit_ms,
                "frame_sequence": values[1] & 0xFFFF,
                "profile_id": values[2],
                "active_follow": values[3],
                "observation_valid": values[4],
                "source_code": values[5],
                "selection_reason_code": values[6],
                "tracker_state_id": values[7],
                "tracker_reason_code": values[8],
                "candidate": values[9],
                "ring_confirmed": values[10],
                "side": values[11],
                "arc_rows": values[12],
                "path_y_span": values[13],
                "target_count": values[14],
                "observed_rows": values[15],
                "inferred_rows": values[16],
                "reject_reason_code": values[17],
                "reject_total": values[18],
                "ring_calibration_ready": values[19],
                "ring_calibration_check_frames": values[20],
                "ring_calibration_support_rows": values[21],
                "ring_calibration_span_rows": values[22],
                "ring_calibration_error_permille": values[23],
                "entry_geometry_ok": values[24],
                "current_geometry_ok": values[25],
                "route_authorized": values[26],
                "arc_y_min": values[27],
                "arc_y_max": values[28],
                "vertex_y": values[29],
                "vertex_x": values[30],
                "intrusion": values[31],
                "port_x": values[32],
                "port_y": values[33],
            }
            for ring_index, column in enumerate(_RING_DIAG_COLUMNS):
                row[column] = values[34 + ring_index]
            row["source"] = _code_name(
                values[5], tuple(
                    name for name, _code in sorted(
                        getattr(decoder, "_SOURCE_CODES", {}).items(),
                        key=lambda item: item[1])))
            row["profile"] = {0: "UNKNOWN", 1: "A", 2: "B", 3: "C"}.get(
                values[2], "UNKNOWN_%d" % values[2])
            row["selection_reason"] = _code_name(
                values[6], getattr(decoder, "_SELECTION_REASONS", ()))
            row["tracker_state"] = _code_name(
                values[7], getattr(decoder, "_TRACK_STATES", ()))
            row["tracker_reason"] = _code_name(
                values[8], getattr(decoder, "_TRACK_REASONS", ()))
            row["reject_reason"] = _code_name(
                values[17], getattr(decoder, "_REJECT_REASONS", ()))
            for reject_index, reject_name in enumerate(
                    getattr(decoder, "_REJECT_REASONS", ())):
                if reject_index < len(rejects):
                    row["reject_count_" + (reject_name or "none").lower()] = \
                        rejects[reject_index]
            for reject_index in range(len(getattr(
                    decoder, "_REJECT_REASONS", ())), reject_count):
                row["reject_count_%02d" % reject_index] = rejects[reject_index]
            rows.append(row)
        episode_offset += episode_bytes
    if episode_offset != len(payload):
        raise ValueError("TRAILING_OR_UNCLAIMED_PAYLOAD_BYTES")
    if not rows:
        raise ValueError("NO_VALID_FRAMES")

    freeze_reasons = getattr(decoder, "_FREEZE_REASONS", ())
    summary = {
        "format_version": version,
        "timestamp_unit_ms": 20 if version == 2 else 1,
        "header_size": header_size,
        "frame_record_size": record_size,
        "episode_count": episode_count,
        "episode_overflow": episode_overflow,
        "freeze_reason_code": freeze_reason,
        "freeze_reason": _code_name(freeze_reason, freeze_reasons),
        "build_id": build_id,
        "payload_size": payload_size,
        "crc32": "%08X" % actual_crc,
        "stop_snapshot_count": sum(
            1 for episode in episodes if episode["kind"] == "STOP_SNAPSHOT"),
        "frame_count": len(rows),
        "episodes": episodes,
    }
    return summary, rows


def _reason_code(exc):
    message = str(exc)
    upper = message.upper()
    if "CRC" in upper:
        return "CRC_MISMATCH"
    if "NO_VALID_FRAMES" in upper:
        return "NO_VALID_FRAMES"
    if "HEADER" in upper:
        return "INVALID_HEADER"
    if "TRUNCATED" in upper or "SIZE" in upper or "BOUNDS" in upper or \
            "TRAILING" in upper:
        return "INVALID_SIZE_OR_TRUNCATED"
    if "MAGIC" in upper:
        return "MAGIC_MISMATCH"
    if ("VERSION" in upper or "FORMAT" in upper or
            "UNSUPPORTED" in upper or "DOES_NOT_SUPPORT" in upper):
        return "UNSUPPORTED_OR_INVALID_FORMAT"
    return type(exc).__name__.upper()


def _directory_candidates(directory):
    candidates = []
    for path in Path(directory).iterdir():
        match = NUMBERED_NAME.match(path.name)
        if match and path.is_file():
            number = int(match.group(1))
            if 1 <= number <= 999999:
                candidates.append((number, path))
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates


def _paths_overlap(left, right):
    left_path = Path(left).resolve()
    right_path = Path(right).resolve()
    try:
        common = Path(os.path.commonpath((str(left_path), str(right_path))))
    except ValueError:
        return False
    return common == left_path or common == right_path


def _safe_write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False,
                      allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    except Exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _write_csv(path, rows, fieldnames):
    if not rows:
        raise ValueError("refusing to write an empty frames.csv")
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("x", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames,
                                    extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    except Exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _fieldnames(decoder):
    fields = [
        "episode_index", "episode_kind", "episode_flags",
        "episode_frame_index", "episode_frame_count", "episode_pre_count",
        "episode_post_count", "episode_candidate_count",
        "episode_confirmed_count", "episode_start_frame",
        "episode_start_sequence", "episode_end_sequence",
        "episode_last_state_id", "episode_dropped_count",
    ] + list(_BASE_COLUMNS)
    reject_names = getattr(decoder, "_REJECT_REASONS", ())
    fields.extend("reject_count_" + (name or "none").lower()
                  for name in reject_names)
    for reject_index in range(len(reject_names),
                              getattr(decoder, "REJECT_COUNT", 13)):
        fields.append("reject_count_%02d" % reject_index)
    fields.extend(("source", "selection_reason", "tracker_state",
                   "tracker_reason", "reject_reason", "profile"))
    return fields


def _failure_entry(path, number, exc):
    return {
        "file": str(path),
        "number": number,
        "reason": _reason_code(exc),
        "detail": str(exc),
    }


def export(input_path=None, directory=None, output_dir=None, runtime_root=None):
    """Export one file or the newest valid numbered file to a separate folder."""
    if (input_path is None) == (directory is None):
        raise ValueError("provide exactly one of input_path or directory")
    if output_dir is None:
        raise ValueError("output_dir is required")

    output = Path(output_dir).expanduser().resolve()
    source = Path(input_path if input_path is not None else directory).expanduser()
    source = source.resolve()
    protected_source = source.parent if input_path is not None else source
    if _paths_overlap(protected_source, output):
        raise ValueError("output_dir must be separate from the source path")
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    csv_path = output / "frames.csv"
    if summary_path.exists() or csv_path.exists():
        raise FileExistsError(
            "output files already exist; choose a new private output directory")
    if (summary_path.with_name(summary_path.name + ".tmp").exists() or
            csv_path.with_name(csv_path.name + ".tmp").exists()):
        raise FileExistsError("stale output temporary file exists")

    decoder, loaded_runtime_root = _load_runtime(runtime_root)
    candidates = ([(None, source)] if input_path is not None else
                  _directory_candidates(source))
    rejected = []
    chosen = None
    frame_rows = None
    file_summary = None
    latest_number = candidates[0][0] if candidates else None
    if not candidates:
        rejected.append({
            "file": str(source), "number": None,
            "reason": "NO_NUMBERED_FILES",
            "detail": "no ring_diagnostics_NNNNNN.bin files found",
        })
    for number, path in candidates:
        try:
            with path.open("rb") as stream:
                data = stream.read(getattr(decoder, "MAX_FILE_BYTES", 262144) + 1)
            if len(data) > getattr(decoder, "MAX_FILE_BYTES", 262144):
                raise ValueError("FILE_SIZE_OUT_OF_BOUNDS")
            file_summary, frame_rows = _decode_candidate(data, decoder)
            chosen = (number, path)
            break
        except Exception as exc:
            rejected.append(_failure_entry(path, number, exc))

    fallback_used = bool(
        directory is not None and chosen is not None and
        chosen[0] != latest_number)
    fallback_attempted = bool(
        directory is not None and len(candidates) > 1 and rejected)
    base = {
        "schema_version": 1,
        "status": "error",
        "source_mode": "input" if input_path is not None else "directory",
        "source_path": str(source),
        "runtime_root": str(loaded_runtime_root),
        "source_read_only": True,
        "latest_number": latest_number,
        "selected_file": str(chosen[1]) if chosen else None,
        "selected_number": chosen[0] if chosen else None,
        "fallback_attempted": fallback_attempted,
        "fallback_used": fallback_used,
        "rejected_candidates": rejected,
        "reason": None,
    }
    if fallback_used:
        base["fallback_from_number"] = latest_number
        base["fallback_reason"] = (
            rejected[0]["reason"] if rejected else "UNKNOWN")
    if chosen is None or not frame_rows:
        if not candidates:
            base["reason"] = "NO_NUMBERED_FILES"
        elif input_path is not None:
            base["reason"] = (
                rejected[0]["reason"] if rejected else "NO_VALID_FRAMES")
        else:
            base["reason"] = "NO_VALID_CANDIDATES"
            if rejected:
                base["latest_rejection_reason"] = rejected[0]["reason"]
        _safe_write_json(summary_path, base)
        return 2, base

    base["status"] = "ok"
    base["reason"] = "OK"
    base["file_summary"] = file_summary
    base["frame_count"] = len(frame_rows)
    try:
        _write_csv(csv_path, frame_rows, _fieldnames(decoder))
        _safe_write_json(summary_path, base)
    except Exception:
        for path in (csv_path, csv_path.with_name(csv_path.name + ".tmp"),
                     summary_path, summary_path.with_name(summary_path.name + ".tmp")):
            try:
                path.unlink()
            except OSError:
                pass
        raise
    if fallback_used:
        latest_rejection = rejected[0] if rejected else None
        print("WARNING: selected older valid file %s after rejecting newest %s (%s)" % (
            chosen[1].name,
            latest_rejection["file"] if latest_rejection else "unknown",
            latest_rejection["reason"] if latest_rejection else "unknown"),
            file=sys.stderr)
    print("exported %d frames from %s" % (len(frame_rows), chosen[1]))
    return 0, base


def build_parser():
    parser = argparse.ArgumentParser(
        description="Export validated OpenMV ring diagnostic files without camera access.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", help="one ring_diagnostics binary file")
    source.add_argument("--directory",
                        help="mounted camera directory containing numbered files")
    parser.add_argument("--output-dir", required=True,
                        help="separate private directory for summary.json and frames.csv")
    parser.add_argument("--runtime-root", default=None,
                        help="directory with ring_diagnostics.py; defaults to project openmv/")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        status, summary = export(
            input_path=args.input, directory=args.directory,
            output_dir=args.output_dir, runtime_root=args.runtime_root)
    except Exception as exc:
        print("export failed: %s: %s" % (type(exc).__name__, str(exc)),
              file=sys.stderr)
        return 2
    if status != 0:
        print("export failed: %s" % summary.get("reason"), file=sys.stderr)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
