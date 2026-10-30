#!/usr/bin/env python3
"""Generate a RAM-only, fail-closed H750 camera full-loop probe script.

The generated script imports the explicit 23-file production runtime already
staged on the camera. It captures real 320x120 sensor frames through the native
preview source, enables the production Gray/CRC8 frame marker for visible
cadence checks, injects a synthetic parked B/C profile preamble followed by
an active lease after ten parked profile samples, and writes only CRC-correct
invalid V3/ACK0 packets through the production UART writer by default. The
optional synchronous mode preserves the earlier UART.write comparison. It
verifies the landed ring-width map and its production startup check. It is not
a Follow acceptance test and does not flash, upload, write camera storage, or
start the vehicle.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

try:
    from .build_track_profiles import RUNTIME_NAMES
except ImportError:
    from build_track_profiles import RUNTIME_NAMES


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OPENMV_ROOT = PROJECT_ROOT / "openmv"
DEFAULT_OUTPUT_ROOT = (PROJECT_ROOT / "logs" /
                       "h750-track-probe-20261025" / "stagedbench")
PROFILE_IDS = {"B": 2, "C": 3}
EXPECTED_RUNTIME_MODULE_COUNT = 23
PARKED_PROFILE_STATUS_FRAMES = 10
RUN_SECONDS = 35
REPORT_LINE_MAX_UTF8_BYTES = 850
REPORT_STATS_PER_LINE = 8
REPORT_MAX_WINDOWS = 4
REPORT_CRC_TAIL_MAX_UTF8_BYTES = 96

STAT_NAMES = (
    "sensor_snapshots",
    "native_preview_source_verified",
    "preview_marker_enabled",
    "native_preview_active_frames",
    "native_preview_headless_frames",
    "native_preview_flush_calls",
    "native_preview_publish_successes",
    "native_preview_publish_failures",
    "computed_packets",
    "computed_v3_packets",
    "computed_crc_valid_packets",
    "computed_crc_bad_packets",
    "computed_valid_flag_packets",
    "computed_invalid_flag_packets",
    "computed_expected_ack_packets",
    "computed_other_ack_packets",
    "invalid_wire_attempts",
    "invalid_wire_frames",
    "invalid_wire_full_accepts",
    "invalid_wire_short_accepts",
    "invalid_wire_errors",
    "invalid_wire_crc_valid_frames",
    "invalid_wire_forced_invalid_frames",
    "invalid_wire_ack_zero_frames",
    "invalid_wire_fifo_queued_frames",
    "invalid_wire_sync_fallback_frames",
    "invalid_wire_sync_baseline_frames",
    "invalid_wire_fifo_txdone_checks",
    "invalid_wire_fifo_txdone_true",
    "invalid_wire_fifo_txdone_false",
    "invalid_wire_fifo_txdone_errors",
    "real_uart_rx_discarded_bytes",
    "real_uart_rx_discard_reads",
    "synthetic_reset_frames",
    "synthetic_keep_frames",
    "synthetic_profile_frames",
    "synthetic_inactive_profile_frames",
    "synthetic_active_profile_frames",
    "sensor_identity_verified",
    "landed_calibration_ready",
    "usb_connection_polls",
    "usb_disconnected_polls",
    "main_stdout_calls_suppressed",
    "summary_windows",
    "summary_window_overflow",
    "summary_window_duplicates",
)
STAT_INDEX = {name: index for index, name in enumerate(STAT_NAMES)}

CALIBRATION_PROOF_HELPER = r'''def _verify_calibration_probe_proof(stats, stat_index):
    if stats[stat_index["sensor_identity_verified"]] != 1:
        raise RuntimeError("probe did not verify the real camera sensor identity")
    if stats[stat_index["native_preview_source_verified"]] != 1:
        raise RuntimeError("probe did not verify the native preview source")
    if stats[stat_index["preview_marker_enabled"]] != 1:
        raise RuntimeError("probe did not enable the visible frame marker")
    if stats[stat_index["landed_calibration_ready"]] != 1:
        raise RuntimeError("landed ring-width calibration is not ready")
    if stats[stat_index["synthetic_inactive_profile_frames"]] < 10:
        raise RuntimeError("probe did not send ten inactive profile states")
    if stats[stat_index["synthetic_active_profile_frames"]] < 1:
        raise RuntimeError("probe did not transition to an active profile")
    if stats[stat_index["computed_expected_ack_packets"]] == 0:
        raise RuntimeError(
            "parked calibration startup check did not unlock the active profile")
    snapshots = stats[stat_index["sensor_snapshots"]]
    preview_states = (
        stats[stat_index["native_preview_active_frames"]] +
        stats[stat_index["native_preview_headless_frames"]])
    if snapshots == 0 or preview_states != snapshots:
        raise RuntimeError("native preview capture state accounting is incomplete")
    return True
'''

RUNTIME_IMPORT_HELPERS = r'''_CAMERA_ROOT = __CAMERA_ROOT__
_RUNTIME_MODULES = __RUNTIME_NAMES__
_MODULE_HASHES = __MODULE_HASHES__
_EXPECTED_RUNTIME_MODULE_COUNT = __EXPECTED_RUNTIME_MODULE_COUNT__


def _sha256_hex(digest):
    alphabet = "0123456789abcdef"
    chars = []
    for value in digest:
        chars.append(alphabet[(int(value) >> 4) & 15])
        chars.append(alphabet[int(value) & 15])
    return "".join(chars)


def _verify_camera_runtime_hashes():
    for filename in _RUNTIME_MODULES:
        source_path = _CAMERA_ROOT + "/" + filename
        handle = open(source_path, "rb")
        hasher = uhashlib.sha256()
        try:
            while True:
                block = handle.read(1024)
                if not block:
                    break
                hasher.update(block)
        finally:
            handle.close()
        if _sha256_hex(hasher.digest()) != _MODULE_HASHES[filename]:
            raise RuntimeError("runtime SHA256 mismatch: " + filename)


def _verify_loaded_module_paths():
    expected_prefix = _CAMERA_ROOT + "/"
    for filename in _RUNTIME_MODULES:
        module_name = filename[:-3]
        module = sys.modules.get(module_name)
        if module is None:
            continue
        module_path = getattr(module, "__file__", None)
        if module_path != expected_prefix + filename:
            raise RuntimeError(
                "runtime module path mismatch: " + module_name + "=" +
                str(module_path))
'''

RUNTIME_IMPORT_BOOTSTRAP = r'''sys.path.insert(0, _CAMERA_ROOT)
_verify_camera_runtime_hashes()
for _runtime_name in _RUNTIME_MODULES:
    _runtime_module_name = _runtime_name[:-3]
    if _runtime_module_name in sys.modules:
        del sys.modules[_runtime_module_name]
import config, main, reverse_control, vision_protocol
main._validate_fast_config()
_verify_loaded_module_paths()
'''

# This code is inserted verbatim into the MicroPython runner and is also
# executed by host tests. Keep it free of imports, annotations and f-strings.
# Reuse the production Viper CRC's 16-entry table instead of doing 112 Python
# bit iterations twice per frame (computed packet check + invalid repack).
WIRE_HELPERS = r'''_CRC16_NIBBLE_TABLE_ARRAY = (
    main._SteeringDiagBuffer._CRC16_NIBBLE_TABLE_ARRAY)


@micropython.viper
def _crc16_ccitt_false_viper(data_object, start: int, end: int,
                             table_object) -> int:
    data = ptr8(data_object)
    table = ptr16(table_object)
    crc = 0xFFFF
    index = start
    while index < end:
        crc = crc ^ (data[index] << 8)
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        index += 1
    return crc


@micropython.native
def _crc16_ccitt_false(data, start, end):
    return _crc16_ccitt_false_viper(
        data, start, end, _CRC16_NIBBLE_TABLE_ARRAY)


@micropython.native
def _crc32_iso_hdlc(data):
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= int(value)
        for _bit in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


@micropython.native
def _computed_packet_flags(packet):
    """Return (crc_valid, valid_flag, profile_ack) for one computed packet."""
    if (packet is None or len(packet) != 16 or
            int(packet[0]) != 0xA5 or int(packet[1]) != 0x5A or
            int(packet[2]) != 3 or int(packet[3]) != 8):
        return (0, 0, 0xFF)
    expected = _crc16_ccitt_false(packet, 2, 14)
    actual = int(packet[14]) | (int(packet[15]) << 8)
    if expected != actual:
        return (0, 0, int(packet[11]))
    return (1, int(packet[6]) & 1, int(packet[11]))


@micropython.native
def _pack_invalid_v3(out, sequence):
    """Build an explicitly invalid, CRC-correct V3 frame with ACK=0."""
    out[0] = 0xA5
    out[1] = 0x5A
    out[2] = 3
    out[3] = 8
    out[4] = sequence & 0xFF
    out[5] = (sequence >> 8) & 0xFF
    out[6] = 0
    out[7] = 0
    out[8] = 0
    out[9] = 0
    out[10] = 0
    out[11] = 0
    out[12] = 0
    out[13] = 0
    crc = _crc16_ccitt_false(out, 2, 14)
    out[14] = crc & 0xFF
    out[15] = (crc >> 8) & 0xFF
    return out
'''

SYNTHETIC_REVERSE_HELPER = r'''class _SyntheticReverseInput:
    """Independent V1/V2 lease source; live UART RX is drained and discarded."""
    PARKED_PROFILE_STATUS_FRAMES = 10

    def __init__(self, physical_uart, reverse_module, time_module,
                 stats, stat_index, profile_id, sequence_seed=0xFFFE):
        self.physical_uart = physical_uart
        self.reverse = reverse_module
        self.time = time_module
        self.stats = stats
        self.stat_index = stat_index
        self.profile_id = profile_id
        self.buffer = bytearray(64)
        self.discard_buffer = bytearray(64)
        self.read_offset = 0
        self.pending = 0
        self.reset_pending = True
        self.control_sequence = 0
        self.profile_sequence = sequence_seed & 0xFFFF
        self.profile_status_frames_sent = 0
        now = self.time.ticks_ms()
        self.next_keep_ms = self.time.ticks_add(now, 50)
        self.next_profile_ms = now

    def _increment(self, name, amount=1):
        index = self.stat_index[name]
        self.stats[index] = (self.stats[index] + amount) & 0xFFFFFFFF

    def _discard_live_rx(self):
        # Bound work per camera parser poll even if RX traffic is continuous.
        # Remaining physical bytes stay private to this wrapper and are
        # discarded on later polls; none are ever returned to the parser.
        for _chunk_index in range(4):
            available = self.physical_uart.any()
            if (isinstance(available, bool) or
                    not isinstance(available, int) or available < 0):
                raise OSError("invalid physical UART.any result")
            if available == 0:
                return
            requested = min(available, len(self.discard_buffer))
            received = self.physical_uart.readinto(
                self.discard_buffer, requested)
            if received is None:
                received = 0
            if (isinstance(received, bool) or
                    not isinstance(received, int) or received < 0 or
                    received > requested):
                raise OSError("invalid physical UART.readinto result")
            if received == 0:
                return
            self._increment("real_uart_rx_discarded_bytes", received)
            self._increment("real_uart_rx_discard_reads")

    def _append(self, frame):
        if self.pending + len(frame) > len(self.buffer):
            raise OSError("synthetic reverse queue capacity exceeded")
        for index in range(len(frame)):
            self.buffer[self.pending + index] = frame[index]
        self.pending += len(frame)

    def _schedule(self):
        now = self.time.ticks_ms()
        if self.reset_pending:
            self._append(self.reverse.pack_frame(
                self.reverse.COMMAND_RESET, self.control_sequence))
            self.reset_pending = False
            self.control_sequence = (self.control_sequence + 1) & 0xFFFF
            self._increment("synthetic_reset_frames")
        elif self.time.ticks_diff(now, self.next_keep_ms) >= 0:
            self._append(self.reverse.pack_frame(
                self.reverse.COMMAND_KEEP, self.control_sequence))
            self.control_sequence = (self.control_sequence + 1) & 0xFFFF
            self.next_keep_ms = self.time.ticks_add(now, 50)
            self._increment("synthetic_keep_frames")

        if self.time.ticks_diff(now, self.next_profile_ms) >= 0:
            active = (self.profile_status_frames_sent >=
                      self.PARKED_PROFILE_STATUS_FRAMES)
            self._append(self.reverse.pack_profile_state(
                self.profile_sequence, self.profile_id,
                flags=(self.reverse.PROFILE_SESSION_ACTIVE if active else 0),
                revision=self.reverse.PROFILE_REVISION,
                session_token=(1 if active else 0),
                mean_forward_wheel_millirev=0))
            self.profile_sequence = (self.profile_sequence + 1) & 0xFFFF
            self.next_profile_ms = self.time.ticks_add(now, 100)
            self.profile_status_frames_sent += 1
            self._increment("synthetic_profile_frames")
            self._increment("synthetic_active_profile_frames" if active else
                            "synthetic_inactive_profile_frames")

    def any(self):
        self._discard_live_rx()
        if self.pending == 0:
            self.read_offset = 0
            self._schedule()
        return self.pending

    def readinto(self, destination, requested=None):
        if requested is None:
            requested = len(destination)
        count = min(len(destination), requested, self.pending)
        for index in range(count):
            destination[index] = self.buffer[self.read_offset + index]
        self.read_offset += count
        self.pending -= count
        if self.pending == 0:
            self.read_offset = 0
        return count
'''

SAFE_UART_HELPER = r'''class _SafeInvalidUART:
    """Send only invalid ACK0 frames through the selected real UART path."""
    WIRE_SEQUENCE_SEED = 0xB000

    def __init__(self, physical_uart, synthetic_input, profile_id,
                 uart_mode):
        if uart_mode not in ("production", "synchronous"):
            raise ValueError("uart mode must be production or synchronous")
        self.physical_uart = physical_uart
        self.synthetic_input = synthetic_input
        self.profile_id = profile_id
        self.uart_mode = uart_mode
        self.writer = (main._VisionUARTWriter(physical_uart)
                       if uart_mode == "production" else None)
        self.wire_sequence = self.WIRE_SEQUENCE_SEED
        self.wire_buffer = bytearray(16)
        self._finished = False
        self._txdone_result = None

    def write(self, computed_packet):
        _stats[_STAT_INDEX["computed_packets"]] += 1
        crc_valid, valid_flag, profile_ack = _computed_packet_flags(
            computed_packet)
        if len(computed_packet) == 16 and int(computed_packet[2]) == 3:
            _stats[_STAT_INDEX["computed_v3_packets"]] += 1
        if crc_valid:
            _stats[_STAT_INDEX["computed_crc_valid_packets"]] += 1
            if valid_flag:
                _stats[_STAT_INDEX["computed_valid_flag_packets"]] += 1
            else:
                _stats[_STAT_INDEX["computed_invalid_flag_packets"]] += 1
            if profile_ack == self.profile_id:
                _stats[_STAT_INDEX["computed_expected_ack_packets"]] += 1
            else:
                _stats[_STAT_INDEX["computed_other_ack_packets"]] += 1
        else:
            _stats[_STAT_INDEX["computed_crc_bad_packets"]] += 1

        sequence = self.wire_sequence
        self.wire_sequence = (self.wire_sequence + 1) & 0xFFFF
        _pack_invalid_v3(self.wire_buffer, sequence)
        _stats[_STAT_INDEX["invalid_wire_attempts"]] += 1
        _stats[_STAT_INDEX["invalid_wire_frames"]] += 1
        if (self.wire_buffer[6] == 0 and self.wire_buffer[11] == 0 and
                _crc16_ccitt_false(self.wire_buffer, 2, 14) ==
                (self.wire_buffer[14] | (self.wire_buffer[15] << 8))):
            _stats[_STAT_INDEX["invalid_wire_crc_valid_frames"]] += 1
            _stats[_STAT_INDEX["invalid_wire_forced_invalid_frames"]] += 1
            _stats[_STAT_INDEX["invalid_wire_ack_zero_frames"]] += 1
        try:
            if self.uart_mode == "synchronous":
                written = self.physical_uart.write(self.wire_buffer)
                _stats[_STAT_INDEX[
                    "invalid_wire_sync_baseline_frames"]] += 1
            else:
                written = self.writer.write(self.wire_buffer)
                if self.writer.mode == self.writer.FIFO_QUEUED:
                    _stats[_STAT_INDEX[
                        "invalid_wire_fifo_queued_frames"]] += 1
                elif self.writer.mode == self.writer.SYNCHRONOUS_FALLBACK:
                    _stats[_STAT_INDEX[
                        "invalid_wire_sync_fallback_frames"]] += 1
                else:
                    raise RuntimeError("unknown production UART writer mode")
        except Exception:
            _stats[_STAT_INDEX["invalid_wire_errors"]] += 1
            raise
        if type(written) is int and written == len(self.wire_buffer):
            _stats[_STAT_INDEX["invalid_wire_full_accepts"]] += 1
        else:
            _stats[_STAT_INDEX["invalid_wire_short_accepts"]] += 1
        return written

    def finish(self):
        """Check queued transmit completion once, outside the frame loop."""
        if self._finished:
            return self._txdone_result
        self._finished = True
        if (self.writer is None or
                self.writer.mode != self.writer.FIFO_QUEUED):
            return None
        _stats[_STAT_INDEX["invalid_wire_fifo_txdone_checks"]] += 1
        try:
            txdone = self.writer.fifo_txdone()
        except Exception:
            _stats[_STAT_INDEX[
                "invalid_wire_fifo_txdone_errors"]] += 1
            self._txdone_result = "ERROR"
            return self._txdone_result
        if txdone is True:
            _stats[_STAT_INDEX["invalid_wire_fifo_txdone_true"]] += 1
            self._txdone_result = True
            return self._txdone_result
        if txdone is False:
            _stats[_STAT_INDEX["invalid_wire_fifo_txdone_false"]] += 1
            self._txdone_result = False
            return self._txdone_result
        _stats[_STAT_INDEX["invalid_wire_fifo_txdone_errors"]] += 1
        self._txdone_result = "INVALID_RESULT"
        return self._txdone_result

    def any(self):
        return self.synthetic_input.any()

    def readinto(self, destination, requested=None):
        return self.synthetic_input.readinto(destination, requested)
'''

WINDOW_HELPERS = r'''class _BoundedWindowTable:
    """Four-row preallocated copy of the production V7 perf windows."""
    BASE_NAMES = (
        "frames", "write_attempts", "full_writes", "valid_frames",
        "valid_full_writes", "late_invalid_frames", "write_errors",
        "last_sequence", "heap_samples", "heap_free_min",
        "heap_alloc_max", "heap_free_last", "heap_alloc_last",
        "attempted_by_loop")
    EXTRA_NAMES = (
        "sensor_snapshots",
        "invalid_wire_frames", "invalid_wire_full_accepts",
        "synthetic_keep_frames", "synthetic_profile_frames",
        "native_preview_active_frames", "native_preview_headless_frames",
        "native_preview_flush_calls",
        "native_preview_publish_successes",
        "native_preview_publish_failures")

    def __init__(self, stages, stats, stat_index):
        self.stages = tuple(stages)
        self.stats = stats
        self.stat_index = stat_index
        self.stride = len(self.BASE_NAMES) + len(self.EXTRA_NAMES) + \
                      len(self.stages) * 4
        self.rows = array("I", [0] * (4 * self.stride))
        self.present = bytearray(4)
        self.previous_extras = array("I", [0] * len(self.EXTRA_NAMES))
        self.count = 0
        self.overflow = 0
        self.duplicates = 0

    def record(self, perf, window_index, attempted):
        if (not isinstance(window_index, int) or
                window_index < 0 or window_index >= 4):
            self.overflow += 1
            return False
        if self.present[window_index]:
            self.duplicates += 1
            return False
        offset = window_index * self.stride
        for index, name in enumerate(self.BASE_NAMES):
            if name == "attempted_by_loop":
                value = attempted
            else:
                value = getattr(perf, name)
            if value is None:
                value = 0
            self.rows[offset + index] = int(value) & 0xFFFFFFFF

        cursor = offset + len(self.BASE_NAMES)
        for index, name in enumerate(self.EXTRA_NAMES):
            current = self.stats[self.stat_index[name]]
            delta = (current - self.previous_extras[index]) & 0xFFFFFFFF
            self.rows[cursor] = delta
            self.previous_extras[index] = current
            cursor += 1

        for stage in self.stages:
            count = perf.counts[stage]
            total = perf.sums[stage]
            maximum = perf.maxima[stage]
            p95 = 0
            if count:
                target = (count * 95 + 99) // 100
                accumulated = 0
                bins = perf.bins[stage]
                for bucket in range(len(bins)):
                    accumulated += bins[bucket]
                    if accumulated >= target:
                        p95 = (maximum if bucket == 201 else
                               (bucket + 1) * 1000)
                        break
            self.rows[cursor] = count & 0xFFFFFFFF
            self.rows[cursor + 1] = (total // count if count else 0) & 0xFFFFFFFF
            self.rows[cursor + 2] = p95 & 0xFFFFFFFF
            self.rows[cursor + 3] = maximum & 0xFFFFFFFF
            cursor += 4

        self.present[window_index] = 1
        self.count += 1
        return True


class _NoSummaryQueue:
    """Avoid the production 256-string stdout queue in this RAM probe."""
    def __init__(self):
        self.overflow_windows = 0

    @property
    def pending(self):
        return 0

    def enqueue(self, _lines):
        return True

    def pop(self):
        return None


_REPORT_LINE_MAX_UTF8_BYTES = __REPORT_LINE_MAX_UTF8_BYTES__
_REPORT_CRC_TAIL_MAX_UTF8_BYTES = __REPORT_CRC_TAIL_MAX_UTF8_BYTES__
_REPORT_STATS_PER_LINE = __REPORT_STATS_PER_LINE__
_REPORT_MAX_WINDOWS = __REPORT_MAX_WINDOWS__


def _validate_report_lines(lines, stat_count, stage_count):
    stat_lines = ((stat_count + _REPORT_STATS_PER_LINE - 1) //
                  _REPORT_STATS_PER_LINE)
    line_limit = (1 + stat_lines + 1 +
                  _REPORT_MAX_WINDOWS * (1 + stage_count))
    if len(lines) > line_limit:
        raise RuntimeError("probe report line count exceeds fixed bound")
    for line in lines:
        line_bytes = len(line.encode("utf-8"))
        if line_bytes > _REPORT_LINE_MAX_UTF8_BYTES:
            raise RuntimeError(
                "probe report line exceeds 850 UTF-8 bytes: %d (%s)" % (
                    line_bytes, line[:64]))
    return lines


def _build_report_lines(profile_label, profile_id, stats, stat_names,
                        windows, window_ms, module_count, error_text,
                        usb_seen_disconnected, uart_mode="production"):
    stat_index = {name: index for index, name in enumerate(stat_names)}
    if error_text:
        while len(error_text.encode("utf-8")) > 96:
            error_text = error_text[:-1]
    if stats[stat_index["invalid_wire_fifo_txdone_true"]]:
        txdone_result = "true"
    elif stats[stat_index["invalid_wire_fifo_txdone_false"]]:
        txdone_result = "false"
    elif stats[stat_index["invalid_wire_fifo_txdone_errors"]]:
        txdone_result = "error"
    else:
        txdone_result = "not_checked"
    lines = [("H750_TRACK_PROBE scope=real_sensor+synthetic_reverse+"
              "invalid_real_uart production_valid_control=0 "
              "full_follow_acceptance=0 profile=%s/%d duration_s=35 "
              "sensor=320x120 modules=%d profile_sequence=10parked_then_active "
              "preview_source=native marker=gray24crc8_opt_in "
              "token=1 odometry_millirev=0 wire=V3-invalid-ACK0 "
              "uart_mode=%s fifo_queued=%d sync_fallback=%d "
              "sync_baseline=%d fifo_txdone=%s txdone_checks=%d "
              "probe_wrapper=viper_crc_inspect+invalid_repack "
              "cdc_detached_observed=%d error=%s" % (
                  profile_label, profile_id, module_count,
                  uart_mode,
                  stats[stat_index["invalid_wire_fifo_queued_frames"]],
                  stats[stat_index["invalid_wire_sync_fallback_frames"]],
                  stats[stat_index["invalid_wire_sync_baseline_frames"]],
                  txdone_result,
                  stats[stat_index["invalid_wire_fifo_txdone_checks"]],
                  1 if usb_seen_disconnected else 0,
                  error_text if error_text else "none"))]
    stats_per_line = _REPORT_STATS_PER_LINE
    stats_line_count = ((len(stat_names) + stats_per_line - 1) //
                        stats_per_line)
    for start in range(0, len(stat_names), stats_per_line):
        chunk_index = start // stats_per_line + 1
        fields = []
        for index in range(start, min(start + stats_per_line,
                                      len(stat_names))):
            fields.append("%s=%d" % (stat_names[index], stats[index]))
        lines.append("H750_TRACK_PROBE_TOTAL part=%d/%d %s" % (
            chunk_index, stats_line_count, " ".join(fields)))
    if windows is None:
        lines.append("H750_TRACK_PROBE_WINDOWS count=0")
        return _validate_report_lines(lines, len(stat_names), 0)
    present_count = 0
    for window_index in range(_REPORT_MAX_WINDOWS):
        if windows.present[window_index]:
            present_count += 1
    lines.append("H750_TRACK_PROBE_WINDOWS count=%d" % present_count)
    for window_index in range(_REPORT_MAX_WINDOWS):
        if not windows.present[window_index]:
            continue
        offset = window_index * windows.stride
        cursor = offset
        fields = []
        for name in windows.BASE_NAMES:
            fields.append("%s=%d" % (name, windows.rows[cursor]))
            cursor += 1
        for name in windows.EXTRA_NAMES:
            fields.append("%s_delta=%d" % (name, windows.rows[cursor]))
            cursor += 1
        stage_lines = []
        for name in windows.stages:
            count = windows.rows[cursor]
            average = windows.rows[cursor + 1]
            p95 = windows.rows[cursor + 2]
            maximum = windows.rows[cursor + 3]
            stage_lines.append(
                "H750_TRACK_PROBE_STAGE window=%d name=%s n=%d avg_us=%d "
                "p95_upper_us=%d max_us=%d" % (
                    window_index, name, count, average, p95, maximum))
            cursor += 4
        lines.append("H750_TRACK_PROBE_WINDOW index=%d start_ms=%d end_ms=%d %s" % (
            window_index, window_index * window_ms,
            min((window_index + 1) * window_ms, 35000), " ".join(fields)))
        lines.extend(stage_lines)
    return _validate_report_lines(lines, len(stat_names),
                                  len(windows.stages))
'''


def render_window_helpers() -> str:
    return (WINDOW_HELPERS
            .replace("__REPORT_LINE_MAX_UTF8_BYTES__",
                     str(REPORT_LINE_MAX_UTF8_BYTES))
            .replace("__REPORT_CRC_TAIL_MAX_UTF8_BYTES__",
                     str(REPORT_CRC_TAIL_MAX_UTF8_BYTES))
            .replace("__REPORT_STATS_PER_LINE__",
                     str(REPORT_STATS_PER_LINE))
            .replace("__REPORT_MAX_WINDOWS__",
                     str(REPORT_MAX_WINDOWS)))

PROBE_BODY = r'''_PROFILE_ID = __PROFILE_ID__
_PROFILE_LABEL = "__PROFILE_LABEL__"
_UART_MODE = "__UART_MODE__"
_RUN_ID = 20261025
_RUN_SECONDS = 35
_MAX_RECONNECT_WAIT_MS = 120000
_STAT_NAMES = __STAT_NAMES__
_STAT_INDEX = __STAT_INDEX__
_WINDOW_CAPACITY = 4
_stats = array("I", [0] * len(_STAT_NAMES))
_error_text = "none"
_windows = None
_probe_source = None
_probe_sink = None
_probe_uart = None
_probe_usb = None
_usb_seen_disconnected = False
_report_dumped = False


class _RealSensorSource(main._NativeUSBPreviewSource):
    name = "H750_REAL_SENSOR_320X120"
    USB_PREVIEW_SOURCE_API = 1

    def __init__(self, sensor_module, usb_vcp, enabled, quality):
        main._NativeUSBPreviewSource.__init__(
            self, sensor_module, usb_vcp, enabled, quality)

    def snapshot(self):
        global _usb_seen_disconnected
        image = main._NativeUSBPreviewSource.snapshot(self)
        _stats[_STAT_INDEX["sensor_snapshots"]] += 1
        _stats[_STAT_INDEX["usb_connection_polls"]] += 1
        if self.frame_preview_active:
            _stats[_STAT_INDEX["native_preview_active_frames"]] += 1
        else:
            _usb_seen_disconnected = True
            _stats[_STAT_INDEX["native_preview_headless_frames"]] += 1
            _stats[_STAT_INDEX["usb_disconnected_polls"]] += 1
        return image

    def flush(self):
        main._NativeUSBPreviewSource.flush(self)
        _stats[_STAT_INDEX["native_preview_flush_calls"]] += 1
        if self.last_state == "NATIVE_ACTIVE":
            _stats[_STAT_INDEX["native_preview_publish_successes"]] += 1
        elif self.last_state == "NATIVE_ERROR":
            _stats[_STAT_INDEX["native_preview_publish_failures"]] += 1


def _record_window(perf, run_id, window_index, attempted, queue=None):
    if _windows is None:
        return False
    stored = _windows.record(perf, window_index, attempted)
    if stored:
        _stats[_STAT_INDEX["summary_windows"]] += 1
    _stats[_STAT_INDEX["summary_window_overflow"]] = _windows.overflow
    _stats[_STAT_INDEX["summary_window_duplicates"]] = _windows.duplicates
    return stored


def _suppress_main_stdout(*args, **kwargs):
    _stats[_STAT_INDEX["main_stdout_calls_suppressed"]] += 1


def _dump_h750_track_probe_report():
    global _report_dumped
    if _report_dumped:
        return False
    if (_probe_usb is None or
            not callable(getattr(_probe_usb, "isconnected", None)) or
            not _probe_usb.isconnected()):
        return False
    time.sleep_ms(1000)
    if not _probe_usb.isconnected():
        return False
    lines = _build_report_lines(
        _PROFILE_LABEL, _PROFILE_ID, _stats, _STAT_NAMES, _windows,
        config.PERF_WINDOW_MS, len(_RUNTIME_MODULES), _error_text,
        _usb_seen_disconnected, _UART_MODE)
    payload = ("\n".join(lines) + "\n").encode("utf-8")
    checksum = _crc32_iso_hdlc(payload)
    for line in lines:
        if not _probe_usb.isconnected():
            return False
        print(line)
        time.sleep_ms(50)
    crc_line = ("H750_TRACK_PROBE_CRC32=%08X payload_bytes=%d "
                "coverage=all_report_lines_before_crc" % (
                    checksum, len(payload)))
    if len(crc_line.encode("utf-8")) > _REPORT_CRC_TAIL_MAX_UTF8_BYTES:
        raise RuntimeError("probe CRC tail exceeds fixed byte limit")
    if not _probe_usb.isconnected():
        return False
    print(crc_line)
    _report_dumped = True
    return True


def h750_track_probe_dump():
    """Manually dump the retained result after USB CDC reconnects."""
    return _dump_h750_track_probe_report()


try:
    if _PROFILE_ID not in (2, 3):
        raise ValueError("probe profile must be B or C")
    if config.IMAGE_WIDTH != 320 or config.IMAGE_HEIGHT != 120:
        raise RuntimeError("probe requires the production 320x120 crop")
    if len(_RUNTIME_MODULES) != _EXPECTED_RUNTIME_MODULE_COUNT:
        raise RuntimeError("probe requires the explicit 23-module runtime")
    if config.RING_WIDTH_CALIBRATION_REQUIRED is not True:
        raise RuntimeError("probe requires production ring calibration gating")
    if not callable(getattr(main, "_NativeUSBPreviewSource", None)):
        raise RuntimeError("production native USB preview source is missing")
    if not callable(getattr(main, "_native_preview_control_api_available", None)):
        raise RuntimeError("native preview control API check is missing")
    if not main._native_preview_control_api_available():
        raise RuntimeError("native preview firmware API markers are missing")
    if config.USB_PREVIEW_ENABLED is not True:
        raise RuntimeError("native preview probe requires USB preview enabled")
    if type(config.PREVIEW_FPS_MARKER_ENABLED) is not bool:
        raise RuntimeError("visible frame marker setting must be bool")
    _probe_usb = pyb.USB_VCP()
    if not callable(getattr(_probe_usb, "isconnected", None)):
        raise RuntimeError("USB_VCP.isconnected() is required for bounded output")

    # Configure and identify the actual sensor before fixture mode starts.
    # _main_fast receives this measured ID through its explicit fixture API so
    # fixture mode cannot bypass the production calibration sensor check.
    _probe_sensor_id = main._configure_gray_sensor(sensor, fast=True)
    if (isinstance(_probe_sensor_id, bool) or
            not isinstance(_probe_sensor_id, int) or
            _probe_sensor_id != main.ring_width_calibration.SENSOR_ID):
        raise RuntimeError("probe camera sensor identity mismatch")
    _stats[_STAT_INDEX["sensor_identity_verified"]] = 1
    _landed_calibration = (
        main.ring_width_calibration.RingWidthCalibration.load(
            config.RING_WIDTH_CALIBRATION_PATH))
    if not _landed_calibration.ready:
        raise RuntimeError("landed ring calibration not ready: " +
                           str(_landed_calibration.reason))
    _stats[_STAT_INDEX["landed_calibration_ready"]] = 1
    _probe_uart = pyb.UART(
        config.UART_ID, config.UART_BAUDRATE, bits=config.UART_BITS,
        parity=None, stop=config.UART_STOP_BITS,
        timeout=config.UART_TIMEOUT_MS,
        timeout_char=config.UART_CHAR_TIMEOUT_MS)
    _probe_source = _RealSensorSource(
        sensor, _probe_usb, config.USB_PREVIEW_ENABLED,
        config.USB_PREVIEW_QUALITY)
    if not main._is_native_usb_preview_source(_probe_source):
        raise RuntimeError("probe source did not satisfy native preview API")
    _stats[_STAT_INDEX["native_preview_source_verified"]] = 1
    _synthetic_input = _SyntheticReverseInput(
        _probe_uart, reverse_control, time, _stats, _STAT_INDEX,
        _PROFILE_ID)
    _probe_sink = _SafeInvalidUART(
        _probe_uart, _synthetic_input, _PROFILE_ID, _UART_MODE)
    _windows = _BoundedWindowTable(
        main._V7_PERF_STAGES, _stats, _STAT_INDEX)

    _original_print = getattr(main, "print", None)
    _had_print = hasattr(main, "print")
    _original_summary = main._print_v7_summary
    _original_queue = main._V7SummaryQueue
    _original_marker_setting = config.PREVIEW_FPS_MARKER_ENABLED
    main.print = _suppress_main_stdout
    main._print_v7_summary = _record_window
    main._V7SummaryQueue = _NoSummaryQueue
    # The normal application leaves this opt-in diagnostic disabled. Enable
    # the actual production marker renderer for this visible-cadence probe.
    config.PREVIEW_FPS_MARKER_ENABLED = True
    _stats[_STAT_INDEX["preview_marker_enabled"]] = 1
    try:
        main._main_fast(
            duration_s=_RUN_SECONDS,
            run_id=_RUN_ID,
            frame_source=_probe_source,
            uart_sink=_probe_sink,
            fixture_sensor_id=_probe_sensor_id)
    finally:
        _probe_sink.finish()
        config.PREVIEW_FPS_MARKER_ENABLED = _original_marker_setting
        main._print_v7_summary = _original_summary
        main._V7SummaryQueue = _original_queue
        if _had_print:
            main.print = _original_print
        elif hasattr(main, "print"):
            del main.print
    _verify_calibration_probe_proof(_stats, _STAT_INDEX)
except Exception as _exc:
    _error_text = type(_exc).__name__ + ":" + str(_exc)
    if len(_error_text) > 96:
        _error_text = _error_text[:96]
finally:
    H750_TRACK_PROBE_RESULT = {
        "profile": _PROFILE_LABEL,
        "profile_id": _PROFILE_ID,
        "duration_s": _RUN_SECONDS,
        "scope": "real_sensor+native_usb_preview+synthetic_reverse+invalid_real_uart",
        "preview_source_api": 1,
        "preview_marker": "gray24crc8_opt_in",
        "production_valid_control": False,
        "stats": _stats,
        "stat_names": _STAT_NAMES,
        "windows": _windows,
        "runtime_modules": _RUNTIME_MODULES,
        "module_hashes_at_generation": _MODULE_HASHES,
        "error": _error_text,
        "report_dumped": False,
    }
    _wait_start = time.ticks_ms()
    while (_probe_usb is not None and
           callable(getattr(_probe_usb, "isconnected", None)) and
           not _probe_usb.isconnected()):
        if time.ticks_diff(time.ticks_ms(), _wait_start) >= _MAX_RECONNECT_WAIT_MS:
            break
        time.sleep_ms(100)
    if (_probe_usb is not None and
            callable(getattr(_probe_usb, "isconnected", None)) and
            _probe_usb.isconnected()):
        _dump_h750_track_probe_report()
        H750_TRACK_PROBE_RESULT["report_dumped"] = _report_dumped
    else:
        # No stdout is attempted while CDC is closed. The in-RAM result and
        # h750_track_probe_dump() remain available after a later reconnect.
        H750_TRACK_PROBE_RESULT["report_dumped"] = False
'''


def verify_runtime_root(runtime_root: Path) -> dict[str, str]:
    """Check the exact explicit 23-module production runtime allowlist."""
    if len(RUNTIME_NAMES) != EXPECTED_RUNTIME_MODULE_COUNT:
        raise ValueError(
            "production runtime allowlist has %d modules; expected %d" % (
                len(RUNTIME_NAMES), EXPECTED_RUNTIME_MODULE_COUNT))
    runtime_root = Path(runtime_root)
    if not runtime_root.is_dir():
        raise FileNotFoundError("runtime directory not found: %s" % runtime_root)
    expected = set(RUNTIME_NAMES)
    actual_py = {path.name for path in runtime_root.glob("*.py")}
    missing = [name for name in RUNTIME_NAMES
               if not (runtime_root / name).is_file()]
    if missing:
        raise FileNotFoundError("runtime files missing: " + ", ".join(missing))
    if actual_py != expected:
        raise ValueError(
            "runtime directory must contain the exact %d-module allowlist; "
            "missing=%r extra=%r" % (
                EXPECTED_RUNTIME_MODULE_COUNT,
                sorted(expected - actual_py), sorted(actual_py - expected)))
    return {name: hashlib.sha256(
        (runtime_root / name).read_bytes()).hexdigest()
        for name in RUNTIME_NAMES}


def validate_camera_root(camera_root: str) -> str:
    """Require a simple absolute POSIX directory under camera /flash."""
    if not isinstance(camera_root, str) or not camera_root.isascii():
        raise ValueError("camera root must be an ASCII POSIX /flash path")
    if not camera_root.startswith("/flash/"):
        raise ValueError("camera root must be below /flash")
    parts = camera_root.split("/")
    if (len(parts) < 3 or parts[0] != "" or parts[1] != "flash" or
            any(not part or not all(
                char.isalnum() or char in "_-" for char in part)
                for part in parts[2:])):
        raise ValueError(
            "camera root components may contain only ASCII letters, digits, "
            "underscore and hyphen")
    return camera_root


def _camera_root_argument(value: str) -> str:
    try:
        return validate_camera_root(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def build_probe_source(runtime_root: Path = OPENMV_ROOT,
                       profile: str = "B",
                       camera_root: str | None = None,
                       uart_mode: str = "production") -> str:
    """Build a small runner; production module source is never embedded."""
    profile = str(profile).upper()
    if profile not in PROFILE_IDS:
        raise ValueError("profile must be B or C")
    if uart_mode not in ("production", "synchronous"):
        raise ValueError("uart_mode must be production or synchronous")
    if camera_root is None:
        raise ValueError("camera_root is required for fail-closed imports")
    camera_root = validate_camera_root(camera_root)
    module_hashes = verify_runtime_root(runtime_root)
    import_helpers = (RUNTIME_IMPORT_HELPERS
                      .replace("__CAMERA_ROOT__", repr(camera_root))
                      .replace("__RUNTIME_NAMES__", repr(tuple(RUNTIME_NAMES)))
                      .replace("__EXPECTED_RUNTIME_MODULE_COUNT__",
                               str(EXPECTED_RUNTIME_MODULE_COUNT))
                      .replace("__MODULE_HASHES__", repr(module_hashes)))
    body = (PROBE_BODY
            .replace("__PROFILE_ID__", str(PROFILE_IDS[profile]))
            .replace("__PROFILE_LABEL__", profile)
            .replace("__UART_MODE__", uart_mode)
            .replace("__STAT_NAMES__", repr(STAT_NAMES))
            .replace("__STAT_INDEX__", repr(STAT_INDEX)))
    source = "\n".join((
        "# Generated by tools/build_h750_track_probe.py; RAM execution only.",
        "# Scope: real sensor + synthetic reverse + real UART invalid V3 ACK0.",
        "import sys, time, gc",
        "from array import array",
        "import pyb, sensor, uhashlib, micropython",
        import_helpers.rstrip(),
        RUNTIME_IMPORT_BOOTSTRAP.rstrip(),
        CALIBRATION_PROOF_HELPER.rstrip(),
        WIRE_HELPERS.rstrip(),
        SYNTHETIC_REVERSE_HELPER.rstrip(),
        SAFE_UART_HELPER.rstrip(),
        render_window_helpers().rstrip(),
        body.rstrip(),
        ""))
    source = source.replace("\r\n", "\n").replace("\r", "\n")
    compile(source, "h750_track_probe_%s.py" % profile, "exec")
    return source


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("B", "C"), default="B",
                        help="synthetic camera profile lease (default B)")
    parser.add_argument("--uart-mode", choices=("production", "synchronous"),
                        default="production",
                        help=("production uses main._VisionUARTWriter; "
                              "synchronous selects the direct UART.write baseline"))
    parser.add_argument("--runtime-root", type=Path, default=OPENMV_ROOT,
                        help="exact staged 23-module OpenMV runtime directory")
    parser.add_argument("--camera-root", required=True,
                        type=_camera_root_argument,
                        help="explicit POSIX import directory on camera, below /flash")
    parser.add_argument("--output", type=Path, default=None,
                        help="generated RAM-only runner path")
    args = parser.parse_args(argv)
    hashes = verify_runtime_root(args.runtime_root)
    source = build_probe_source(args.runtime_root, args.profile,
                                args.camera_root, args.uart_mode)
    output_name = "h750-track-probe-%s.py" % args.profile
    if args.uart_mode == "synchronous":
        output_name = "h750-track-probe-%s-synchronous.py" % args.profile
    output = args.output or (DEFAULT_OUTPUT_ROOT / output_name)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    source_bytes = source.encode("utf-8")
    output.write_bytes(source_bytes)
    print("output=%s" % output)
    print("bytes=%d" % len(source_bytes))
    print("sha256=%s" % hashlib.sha256(source_bytes).hexdigest())
    print("profile=%s id=%d runtime_modules=%d" % (
        args.profile, PROFILE_IDS[args.profile], len(RUNTIME_NAMES)))
    print("runtime_source_hashes=")
    for name in RUNTIME_NAMES:
        print("  %s %s" % (name, hashes[name]))
    print("scope=real_sensor+native_usb_preview+synthetic_reverse+"
          "invalid_real_uart; uart_mode=%s; marker=gray24crc8_opt_in; "
          "production_valid_control=0; no upload/flash/vehicle start" %
          args.uart_mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
