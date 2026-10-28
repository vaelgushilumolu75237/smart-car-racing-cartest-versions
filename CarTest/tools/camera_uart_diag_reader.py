"""Decode bounded camera UART diagnostics retained in H743 RAM.

These records describe camera-reported state, not a direct camera memory read,
physical vehicle acceptance, or a camera sensor FPS measurement.
"""
from __future__ import annotations

import struct
import zlib


SYMBOL = "g_camera_diag_snapshot"
SIZE = 352
MAGIC = 0x47444943
VERSION = 1
HEADER_SIZE = 96
RECORD_SIZE = 16
CAPACITY = 16
STALE_MS = 5000

CALIBRATION_NAMES = (
    "UNKNOWN", "CALIBRATION_READY", "FILE_NOT_FOUND", "CRC_MISMATCH",
    "GEOMETRY_MISMATCH", "INVALID_CALIBRATION_DATA", "SENSOR_ID_MISMATCH",
    "NO_CALIBRATION", "INSUFFICIENT_SUPPORT", "OTHER",
)
OBSERVATION_NAMES = (
    "UNKNOWN", "OK", "RING_CALIBRATION_REQUIRED", "DETECTOR_EXCEPTION",
    "FRAME_BUDGET", "NO_ROUTE", "SESSION_EXPIRED", "OTHER_REJECTED",
)
REVERSE_NAMES = (
    "STARTUP", "RESET", "KEEP", "CONTROL_TIMEOUT", "KEEP_WITHOUT_RESET",
    "UART_ERROR", "BAD_FRAME", "OTHER",
)


def _payload(raw: bytes) -> dict:
    kind, flags = raw[:2]
    if kind == 1:
        if (flags & 0x80 or raw[2] >= len(CALIBRATION_NAMES)
                or raw[3] >= len(OBSERVATION_NAMES)
                or raw[4] >= len(REVERSE_NAMES)
                or raw[5] >> 4 > 3 or raw[5] & 15 > 3):
            raise ValueError("invalid camera status payload")
        return {
            "kind": "status", "flags": flags,
            "calibration_map_ready": bool(flags & 1),
            "parked_check_passed": bool(flags & 2),
            "reverse_keep_fresh": bool(flags & 4),
            "profile_fresh": bool(flags & 8),
            "calibration_gate_blocked": bool(flags & 16),
            "actual_observation_valid": bool(flags & 32),
            "fifo_writer_available": bool(flags & 64),
            "calibration_reason": CALIBRATION_NAMES[raw[2]],
            "observation_reason": OBSERVATION_NAMES[raw[3]],
            "reverse_reason": REVERSE_NAMES[raw[4]],
            "raw_profile_ack": raw[5] >> 4,
            "actual_transmitted_ack": raw[5] & 15,
            "vision_sequence": struct.unpack_from("<H", raw, 6)[0],
        }
    if kind == 2:
        if flags & ~1:
            raise ValueError("invalid camera counter flags")
        tx, detector, reverse = struct.unpack_from("<3H", raw, 2)
        return {"kind": "counters", "saturated": bool(flags & 1),
                "vision_tx_errors": tx, "detector_errors": detector,
                "reverse_rejects": reverse}
    if kind == 3:
        features = struct.unpack_from("<H", raw, 6)[0]
        if flags != 1 or features & ~1:
            raise ValueError("unsupported camera diagnostic identity")
        return {"kind": "identity", "schema": flags,
                "build_code": "0x%08X" % struct.unpack_from("<I", raw, 2)[0],
                "features": features,
                "identity_is_full_source_proof": False}
    raise ValueError("unknown camera diagnostic kind")


def decode_camera_uart_snapshot(raw: bytes, now_ms: int) -> dict:
    """Return an explicit unavailable/stale/torn result for optional telemetry."""
    result = {"available": True, "status": "invalid", "fresh": False,
              "records": [], "is_full_text_camera_log": False,
              "source": "camera UART records cached by H743",
              "stale_after_ms": STALE_MS}
    if len(raw) != SIZE:
        result["error"] = "camera diagnostic snapshot length mismatch"
        return result
    if not any(raw):
        result["status"] = "uninitialized"
        return result
    magic, version, size, generation, crc = struct.unpack_from("<IHHII", raw)
    result["generation"] = generation
    if magic != MAGIC or version != VERSION or size != SIZE:
        result["error"] = "camera diagnostic header mismatch"
        return result
    if generation & 1:
        result.update(status="updating", error="camera snapshot writer active")
        return result
    checked = bytearray(raw)
    checked[12:16] = bytes(4)
    if zlib.crc32(checked) & 0xFFFFFFFF != crc:
        result.update(status="torn_or_corrupt", error="camera snapshot CRC mismatch")
        return result
    accepted, rejected, latest_ms, status_ms, counters_ms = struct.unpack_from("<5I", raw, 16)
    latest_seq, count, next_slot, seen = struct.unpack_from("<4H", raw, 52)
    overwritten = struct.unpack_from("<I", raw, 60)[0]
    if (count > CAPACITY or next_slot >= CAPACITY or seen & ~7
            or count != min(accepted, CAPACITY) or next_slot != accepted % CAPACITY
            or overwritten != max(accepted - CAPACITY, 0) or any(raw[76:96])):
        result["error"] = "camera snapshot ring bounds or reserved fields invalid"
        return result

    def timed(payload: bytes, arrival_ms: int) -> dict:
        entry = _payload(payload)
        age = (int(now_ms) - arrival_ms) & 0xFFFFFFFF
        entry.update(arrival_ms=arrival_ms, age_ms=age,
                     fresh=age < STALE_MS,
                     clock_order_uncertain=age >= 0x80000000)
        return entry

    try:
        latest_status = timed(raw[36:44], status_ms) if seen & 1 else None
        latest_counters = timed(raw[44:52], counters_ms) if seen & 2 else None
        identity_ms = struct.unpack_from("<I", raw, 72)[0]
        latest_identity = timed(raw[64:72], identity_ms) if seen & 4 else None
        for expected, item in (("status", latest_status), ("counters", latest_counters),
                               ("identity", latest_identity)):
            if item is not None and item["kind"] != expected:
                raise ValueError("camera latest-payload kind mismatch")
        records = []
        start = next_slot if count == CAPACITY else 0
        for index in range(count):
            offset = HEADER_SIZE + ((start + index) % CAPACITY) * RECORD_SIZE
            arrival, sequence, reserved = struct.unpack_from("<IHH", raw, offset)
            if reserved:
                raise ValueError("camera record reserved field is nonzero")
            record = timed(raw[offset + 8:offset + 16], arrival)
            record["diagnostic_sequence"] = sequence
            records.append(record)
    except ValueError as exc:
        result["error"] = str(exc)
        return result
    fresh = bool(latest_status and latest_status["fresh"])
    result.update(status="fresh" if fresh else "stale" if latest_status else "status_not_received",
                  fresh=fresh, crc_valid=True, accepted_count=accepted,
                  rejected_count=rejected, latest_arrival_ms=latest_ms,
                  latest_sequence=latest_seq, overwritten_count=overwritten,
                  latest_status=latest_status, latest_counters=latest_counters,
                  latest_identity=latest_identity, records=records,
                  ring_capacity=CAPACITY)
    return result
