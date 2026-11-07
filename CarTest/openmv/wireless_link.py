"""Bounded K230 side of the D3/91 wireless diagnostic link.

The camera still emits the established 16-byte V4 control frame first.  This
module is a side-band service for the H743 wireless bridge: it consumes only
complete D3/91 frames, never sends an image on the control path, and keeps all
buffers bounded.  It intentionally has no dependency on the camera driver so
that the parser and command rules can be exercised on a host.

The H743 owns the wireless session clock.  A HELLO payload is
``epoch:u32, parked:u8, reserved[3]``; its outer session is the same epoch.
Commands are checked against the clock offset learned from that HELLO.  Image
payloads use an 18-byte metadata prefix followed by at most 128 data bytes:
``format:u8, flags:u8, frame_id:u32, offset:u32, total:u32,
width:u16, height:u16``.
"""

import time

try:
    import binascii
except ImportError:
    binascii = None


MAGIC0 = 0xD3
MAGIC1 = 0x91
VERSION = 1
HEADER_SIZE = 20
CRC_SIZE = 4
MAX_PAYLOAD = 160
MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE

TYPE_HELLO = 1
TYPE_HELLO_REPLY = 2
TYPE_COMMAND = 3
TYPE_ACK = 4
TYPE_STATE = 16
TYPE_DIAG = 17
TYPE_IMAGE = 18
TYPE_CONFIG = 19
TYPE_EVENT = 20

TARGET_MCU = 0
TARGET_CAMERA = 1

OP_INFO = 0x01
OP_START = 0x02
OP_STOP = 0x03
OP_SET_PARAMS = 0x04
OP_SAVE = 0x05
OP_DEFAULT = 0x06
OP_TEST_MOTOR = 0x07
OP_TEST_SERVO = 0x08
OP_IMU_CAL = 0x09
OP_CAMERA_CAL = 0x0A
OP_CAPTURE = 0x0B
OP_GET_PARAMS = 0x0E

ACK_OK = 0
ACK_PENDING = 1
ACK_BAD_FRAME = 2
ACK_BAD_SESSION = 3
ACK_STALE = 4
ACK_DUPLICATE = 5
ACK_OUT_OF_ORDER = 6
ACK_NOT_PARKED = 7
ACK_BUSY = 8
ACK_DENIED = 9
ACK_RANGE = 10
ACK_UNSUPPORTED = 11
ACK_BAD_PARAM = 12
ACK_FAULT = 13
ACK_FORWARD = 14
ACK_NOT_READY = 15

PARAM_VISION_P = 3
PARAM_VISION_D = 17

IMAGE_BINARY = 1
IMAGE_GRAY = 2
IMAGE_FLAGS_FIRST = 1
IMAGE_FLAGS_LAST = 2
IMAGE_META_SIZE = 18
IMAGE_DATA_LIMIT = 128
MAX_IMAGE_BYTES = 76800

HELLO_PAYLOAD_SIZE = 8
CAP_CAMERA_DIAGNOSTIC = 1 << 0
CAP_BINARY_IMAGE = 1 << 1
CAP_GRAY_CAPTURE = 1 << 2
CAP_CAMERA_CALIBRATION = 1 << 3
CAP_PD_TUNING = 1 << 4


def _u16(data, offset):
    return (int(data[offset]) | (int(data[offset + 1]) << 8))


def _u32(data, offset):
    return (int(data[offset]) | (int(data[offset + 1]) << 8) |
            (int(data[offset + 2]) << 16) |
            (int(data[offset + 3]) << 24))


def _i32(data, offset):
    value = _u32(data, offset)
    return value - 0x100000000 if value & 0x80000000 else value


def _put_u16(data, offset, value):
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def _put_i16(data, offset, value):
    value = int(value) & 0xFFFF
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def _put_u32(data, offset, value):
    value &= 0xFFFFFFFF
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF
    data[offset + 2] = (value >> 16) & 0xFF
    data[offset + 3] = (value >> 24) & 0xFF


def _put_i32(data, offset, value):
    _put_u32(data, offset, int(value) & 0xFFFFFFFF)


def _code(value, table):
    if isinstance(value, int) and not isinstance(value, bool):
        return value & 0xFF
    if value is None:
        return 0
    return table.get(str(value), 0)


def crc32(data, length=None):
    """IEEE CRC32, matching ``car_wireless_crc32`` in the H743 build."""
    if length is None:
        length = len(data)
    if binascii is not None:
        try:
            return int(binascii.crc32(data[:length])) & 0xFFFFFFFF
        except (AttributeError, TypeError):
            pass
    crc = 0xFFFFFFFF
    for index in range(length):
        crc ^= int(data[index])
        for _bit in range(8):
            crc = ((crc >> 1) ^ 0xEDB88320
                   if crc & 1 else crc >> 1)
    return crc ^ 0xFFFFFFFF


def encode_frame(frame_type, sequence, session, timestamp_ms, payload=b""):
    """Encode one bounded outer frame as a bytearray."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("wireless payload exceeds 160 bytes")
    result = bytearray(HEADER_SIZE + len(payload) + CRC_SIZE)
    result[0] = MAGIC0
    result[1] = MAGIC1
    result[2] = VERSION
    result[3] = int(frame_type) & 0xFF
    _put_u16(result, 4, len(payload))
    _put_u16(result, 6, 0)
    _put_u32(result, 8, sequence)
    _put_u32(result, 12, session)
    _put_u32(result, 16, timestamp_ms)
    if payload:
        result[HEADER_SIZE:HEADER_SIZE + len(payload)] = payload
    _put_u32(result, HEADER_SIZE + len(payload), crc32(
        result, HEADER_SIZE + len(payload)))
    return result


def decode_frame(data):
    """Return a small dict for a complete valid frame, otherwise ``None``."""
    if data is None or len(data) < HEADER_SIZE + CRC_SIZE:
        return None
    if data[0] != MAGIC0 or data[1] != MAGIC1 or data[2] != VERSION:
        return None
    if _u16(data, 6) != 0:
        return None
    payload_len = _u16(data, 4)
    total = HEADER_SIZE + payload_len + CRC_SIZE
    if payload_len > MAX_PAYLOAD or len(data) != total:
        return None
    if _u32(data, HEADER_SIZE + payload_len) != crc32(
            data, HEADER_SIZE + payload_len):
        return None
    return {
        "type": int(data[3]),
        "payload_len": payload_len,
        "sequence": _u32(data, 8),
        "session": _u32(data, 12),
        "timestamp_ms": _u32(data, 16),
        "payload": data[HEADER_SIZE:HEADER_SIZE + payload_len],
    }


def _sequence_forward(sequence, previous):
    delta = (int(sequence) - int(previous)) & 0xFFFFFFFF
    return delta != 0 and delta < 0x80000000


def _ticks_diff(now, then):
    diff_fn = getattr(time, "ticks_diff", None)
    if callable(diff_fn):
        return diff_fn(now, then)
    return int(now) - int(then)


class OuterParser:
    """Allocation-free stream parser for D3/91 frames."""

    def __init__(self):
        self.buffer = bytearray(MAX_FRAME)
        self.length = 0
        self.expected = 0
        self.frames_ok = 0
        self.sync_errors = 0
        self.length_errors = 0
        self.crc_errors = 0
        self.reserved_errors = 0

    def reset(self):
        self.length = 0
        self.expected = 0

    def feed(self, value):
        value = int(value) & 0xFF
        if self.length == 0:
            if value == MAGIC0:
                self.buffer[0] = value
                self.length = 1
            return None
        if self.length == 1 and value != MAGIC1:
            self.sync_errors += 1
            self.reset()
            if value == MAGIC0:
                self.buffer[0] = value
                self.length = 1
            return None
        if self.length >= MAX_FRAME:
            self.length_errors += 1
            self.reset()
            return None
        self.buffer[self.length] = value
        self.length += 1
        if self.length == HEADER_SIZE:
            if self.buffer[2] != VERSION:
                self.sync_errors += 1
                self.reset()
                return None
            if _u16(self.buffer, 6) != 0:
                self.reserved_errors += 1
                self.reset()
                return None
            payload_len = _u16(self.buffer, 4)
            if payload_len > MAX_PAYLOAD:
                self.length_errors += 1
                self.reset()
                return None
            self.expected = HEADER_SIZE + payload_len + CRC_SIZE
        if self.expected and self.length == self.expected:
            if _u32(self.buffer, HEADER_SIZE + _u16(self.buffer, 4)) != crc32(
                    self.buffer, HEADER_SIZE + _u16(self.buffer, 4)):
                self.crc_errors += 1
                self.reset()
                return None
            frame = decode_frame(self.buffer[:self.length])
            self.reset()
            if frame is None:
                self.crc_errors += 1
                return None
            self.frames_ok += 1
            return frame
        return None


def _pack_ack(status, opcode, target, request_seq, detail=0,
              settings_generation=0, flags=0):
    payload = bytearray(16)
    payload[0] = int(status) & 0xFF
    payload[1] = int(opcode) & 0xFF
    payload[2] = int(target) & 0xFF
    payload[3] = int(flags) & 0xFF
    _put_u32(payload, 4, request_seq)
    _put_u32(payload, 8, detail)
    _put_u32(payload, 12, settings_generation)
    return payload


def _pack_hello_reply(epoch, capabilities):
    payload = bytearray(8)
    _put_u32(payload, 0, epoch)
    _put_u32(payload, 4, capabilities)
    return payload


def _pack_camera_config(version, generation, vision_p, vision_d):
    """Pack the 16-byte camera sidecar record used by the PC tool."""
    payload = bytearray(16)
    _put_u32(payload, 0, version)
    _put_u32(payload, 4, generation)
    _put_u16(payload, 8, vision_p)
    _put_u16(payload, 10, vision_d)
    _put_u32(payload, 12, crc32(payload, 12))
    return payload


def _pack_state(frame_id, valid, quality, source, ring_state, side,
                lateral_error, control, processing_ms, fps_x10,
                curve_phase, slowdown, profile, profile_active,
                session_token, detector_reason):
    """Stable camera STATE payload (36 bytes, all integer fields).

    Layout: frame_id u32, valid/quality/source/ring_state/side u8 x5,
    lateral_error/control i16 x2, processing_ms/fps_x10 u16 x2,
    curve_phase u8, slowdown u16, profile/profile_active u8 x2,
    session_token u32, detector_reason u16, reserved[8].
    """
    payload = bytearray(36)
    _put_u32(payload, 0, frame_id)
    payload[4] = 1 if valid else 0
    payload[5] = max(0, min(100, int(quality)))
    # Keep the original coarse source codes stable. The production route
    # selector uses both the lower-case route names below and the upper-case
    # aliases in diagnostic fixtures, so accept both without turning an
    # otherwise valid ring frame into source 0.
    payload[6] = _code(source, {
        "NONE": 0, "none": 0,
        "road": 1, "ROAD": 1, "surface": 1, "SURFACE": 1,
        "single_left": 2, "SINGLE_LEFT": 2,
        "single_right": 3, "SINGLE_RIGHT": 3,
        "ring_left": 4, "RING_LEFT": 4,
        "ring_right": 5, "RING_RIGHT": 5})
    # The tracker has seven internal states, while the wire contract keeps
    # four coarse phases for compatibility with existing PC decoders.
    payload[7] = _code(ring_state, {
        "IDLE": 0, "idle": 0, "OBSERVE": 0,
        "ENTRY": 1, "entry": 1, "ENTRY_CONFIRM": 1, "ENTER": 1,
        "RING_ENTRY": 1, "RING_ENTRY_CONFIRM": 1,
        "INSIDE": 2, "inside": 2, "RING_INSIDE": 2,
        "EXIT": 3, "exit": 3, "EXIT_CONFIRM": 3,
        "EXIT_REPAIR": 3, "REACQUIRE": 3,
        "RING_EXIT": 3, "RING_EXIT_CONFIRM": 3,
        "RING_EXIT_REPAIR": 3, "RING_REACQUIRE": 3})
    payload[8] = _code(side, {"NONE": 0, "LEFT": 1, "RIGHT": 2})
    _put_i16(payload, 9, int(lateral_error))
    _put_i16(payload, 11, int(control))
    _put_u16(payload, 13, max(0, min(0xFFFF, int(processing_ms))))
    _put_u16(payload, 15, max(0, min(0xFFFF, int(fps_x10))))
    payload[17] = int(curve_phase) & 0xFF
    _put_u16(payload, 18, max(0, min(1000, int(slowdown))))
    payload[20] = int(profile) & 0xFF
    payload[21] = 1 if profile_active else 0
    _put_u32(payload, 22, session_token)
    if isinstance(detector_reason, int) and not isinstance(detector_reason, bool):
        reason_code = detector_reason & 0xFFFF
    else:
        reason_text = "" if detector_reason is None else str(detector_reason)
        reason_code = 0
        for value in reason_text:
            reason_code = ((reason_code * 33) + ord(value)) & 0xFFFF
    _put_u16(payload, 26, reason_code)
    # bytes 28..35 are reserved for future state fields.
    return payload


def _pack_diag(frame_id, processing_us, capture_us, tx_errors,
               parser_errors, command_errors, memory_free, flags):
    payload = bytearray(32)
    _put_u32(payload, 0, frame_id)
    _put_u32(payload, 4, processing_us)
    _put_u32(payload, 8, capture_us)
    _put_u32(payload, 12, tx_errors)
    _put_u32(payload, 16, parser_errors)
    _put_u32(payload, 20, command_errors)
    _put_u32(payload, 24, memory_free)
    _put_u32(payload, 28, flags)
    return payload


def _pack_image(format_id, flags, frame_id, offset, total, width, height,
                source, data):
    if len(data) > IMAGE_DATA_LIMIT:
        raise ValueError("image chunk exceeds 128 bytes")
    payload = bytearray(IMAGE_META_SIZE + len(data))
    # Fixed cross-language header: <IIIHHBB> = id, offset, total, width,
    # height, format, flags.  It is shared with App/wireless_runtime.c.
    _put_u32(payload, 0, frame_id)
    _put_u32(payload, 4, offset)
    _put_u32(payload, 8, total)
    _put_u16(payload, 12, width)
    _put_u16(payload, 14, height)
    payload[16] = int(format_id) & 0xFF
    payload[17] = int(flags) & 0xFF
    payload[IMAGE_META_SIZE:] = source[offset:offset + len(data)]
    return payload


class WirelessCameraLink:
    """K230 command/telemetry service.

    ``capture_gray`` and ``camera_cal`` are callbacks supplied by the main
    loop.  They are intentionally called only after ``parked_cb`` says the
    most recent H743 HELLO is fresh and parked.  A callback must return an
    immutable full frame (or bytes-like object); the service copies it once.
    """

    def __init__(self, enabled=True, parked_cb=None, capture_gray=None,
                 camera_cal=None, settings_store=None, clock=None,
                 state_period_ms=50, diag_period_ms=500,
                 binary_period_ms=500):
        self.enabled = bool(enabled)
        self.parser = OuterParser()
        self.parked_cb = parked_cb
        self.capture_gray = capture_gray
        self.camera_cal = camera_cal
        self.settings_store = settings_store
        self.clock = clock or time
        self.session = 0
        self.clock_offset_ms = None
        self.hello_fresh_ms = None
        self.hello_parked = False
        self.last_command_seq = 0
        self.command_seq_valid = False
        self.last_ack = None
        self.last_ack_opcode = 0
        self.last_ack_target = 0
        self.pending_camera_cal_seq = None
        self.out_sequence = 0
        self.tx_queue = []
        self.tx_dropped = 0
        self.command_errors = 0
        self.rx_frames = 0
        self.rx_rejected = 0
        self.capture_busy = False
        self.capture_frame_id = 0
        self.image_format = 0
        self.image_data = None
        self.image_width = 0
        self.image_height = 0
        self.image_offset = 0
        self.image_total = 0
        self.last_binary_frame_id = -1
        self.last_state_ms = None
        self.last_diag_ms = None
        self.last_binary_ms = None
        self.settings_generation = 0
        self.vision_p_permille = 1000
        self.vision_d_permille = 1000
        self.settings_crc = 0
        self.state_period_ms = max(10, int(state_period_ms))
        self.diag_period_ms = max(100, int(diag_period_ms))
        self.binary_period_ms = max(100, int(binary_period_ms))
        self._now_ms = 0
        self._legacy_candidate = bytearray(MAX_FRAME)
        self._legacy_length = 0
        self._legacy_expected = 0
        self._one_byte = bytearray(1)

    @property
    def capabilities(self):
        return (CAP_CAMERA_DIAGNOSTIC | CAP_BINARY_IMAGE |
                CAP_GRAY_CAPTURE | CAP_CAMERA_CALIBRATION | CAP_PD_TUNING)

    def _now(self):
        tick = getattr(self.clock, "ticks_ms", None)
        return int(tick() if callable(tick) else 0) & 0xFFFFFFFF

    def _parked(self):
        if not callable(self.parked_cb):
            return False
        try:
            return bool(self.parked_cb())
        except Exception:
            return False

    def _enqueue(self, frame, priority=False):
        if not self.enabled:
            return False
        if len(self.tx_queue) >= 8:
            # Only ACK/HELLO_REPLY may evict the oldest telemetry packet.
            if not priority:
                self.tx_dropped += 1
                return False
            self.tx_queue.pop(0)
            self.tx_dropped += 1
        self.tx_queue.append(frame)
        return True

    def _send_frame(self, frame_type, payload=b"", timestamp_ms=None,
                    priority=False):
        if timestamp_ms is None:
            timestamp_ms = self._now()
        frame = encode_frame(frame_type, self.out_sequence, self.session,
                             timestamp_ms, payload)
        self.out_sequence = (self.out_sequence + 1) & 0xFFFFFFFF
        return self._enqueue(frame, priority=priority)

    def _ack(self, status, opcode, target, request_seq, detail=0,
             flags=0, cache=True):
        payload = _pack_ack(status, opcode, target, request_seq, detail,
                            self.settings_generation, flags)
        self._send_frame(TYPE_ACK, payload, priority=True)
        if cache:
            self.last_ack = bytes(payload)
            self.last_ack_opcode = int(opcode) & 0xFF
            self.last_ack_target = int(target) & 0xFF
        return status

    def _reject(self, status, opcode=0, target=TARGET_CAMERA,
                request_seq=0, detail=0):
        self.rx_rejected += 1
        self.command_errors += 1
        # A rejected/stale command must not replace the last successful ACK;
        # otherwise a retransmitted valid command would receive an unrelated
        # error payload from the duplicate path.
        return self._ack(status, opcode, target, request_seq, detail,
                         cache=False)

    def _frame_age(self, timestamp_ms, now_ms):
        if self.clock_offset_ms is None:
            return None
        local_timestamp = (int(timestamp_ms) + int(self.clock_offset_ms)) & 0xFFFFFFFF
        return _ticks_diff(now_ms, local_timestamp)

    def _handle_hello(self, frame, now_ms):
        payload = frame["payload"]
        if len(payload) != HELLO_PAYLOAD_SIZE:
            return self._reject(ACK_BAD_FRAME, TYPE_HELLO, TARGET_CAMERA,
                                frame["sequence"])
        epoch = _u32(payload, 0)
        parked = payload[4] == 1 and payload[5] == 0 and payload[6] == 0 and payload[7] == 0
        if epoch == 0 or frame["session"] != epoch:
            return self._reject(ACK_BAD_SESSION, TYPE_HELLO, TARGET_CAMERA,
                                frame["sequence"])
        if self.capture_busy and self.session != epoch:
            return self._reject(ACK_BUSY, TYPE_HELLO, TARGET_CAMERA,
                                frame["sequence"])
        session_changed = self.session != epoch
        self.session = epoch
        self.clock_offset_ms = _ticks_diff(now_ms, frame["timestamp_ms"])
        self.hello_fresh_ms = now_ms
        self.hello_parked = parked
        if session_changed:
            self.command_seq_valid = False
            self.last_command_seq = 0
            self.last_ack = None
            self.pending_camera_cal_seq = None
        self._send_frame(TYPE_HELLO_REPLY,
                         _pack_hello_reply(epoch, self.capabilities),
                         timestamp_ms=now_ms, priority=True)
        return ACK_OK

    def _hello_is_fresh(self, now_ms):
        return (self.hello_fresh_ms is not None and
                0 <= _ticks_diff(now_ms, self.hello_fresh_ms) <= 400 and
                self.hello_parked)

    def _hello_clock_is_fresh(self, now_ms):
        return (self.session != 0 and self.hello_fresh_ms is not None and
                0 <= _ticks_diff(now_ms, self.hello_fresh_ms) <= 400)

    def _validate_command_header(self, frame, now_ms):
        if frame["session"] == 0 or frame["session"] != self.session:
            return ACK_BAD_SESSION
        age = self._frame_age(frame["timestamp_ms"], now_ms)
        if age is None or age < -50 or age > 1000:
            return ACK_STALE
        sequence = frame["sequence"]
        if self.command_seq_valid:
            if sequence == self.last_command_seq:
                return ACK_DUPLICATE
            if not _sequence_forward(sequence, self.last_command_seq):
                return ACK_OUT_OF_ORDER
        return ACK_OK

    def _apply_params(self, payload, moving):
        if len(payload) < 6:
            return ACK_BAD_PARAM
        group = int(payload[4])
        count = int(payload[5])
        if count < 1 or count > 16 or len(payload) != 6 + count * 5:
            return ACK_BAD_PARAM
        candidate_p = self.vision_p_permille
        candidate_d = self.vision_d_permille
        for index in range(count):
            offset = 6 + index * 5
            param_id = int(payload[offset])
            value = _i32(payload, offset + 1)
            if group not in (1, 2) or param_id not in (PARAM_VISION_P,
                                                          PARAM_VISION_D):
                return ACK_UNSUPPORTED
            if value < 500 or value > 1500:
                return ACK_RANGE
            if param_id == PARAM_VISION_P:
                candidate_p = value
            else:
                candidate_d = value
        self.vision_p_permille = candidate_p
        self.vision_d_permille = candidate_d
        return ACK_OK

    def _load_settings(self):
        if self.settings_store is None:
            return True
        try:
            record = self.settings_store.load()
            if isinstance(record, dict):
                self.vision_p_permille = int(record.get("vision_p", 1000))
                self.vision_d_permille = int(record.get("vision_d", 1000))
                self.settings_generation = int(record.get("generation", 0))
                self.settings_crc = int(record.get("crc", 0))
            return True
        except Exception:
            return False

    def _save_settings(self):
        if self.settings_store is None:
            return False
        try:
            result = self.settings_store.save({
                "version": 1,
                "vision_p": self.vision_p_permille,
                "vision_d": self.vision_d_permille,
                "generation": (self.settings_generation + 1) & 0xFFFFFFFF,
            })
            if isinstance(result, dict):
                self.settings_generation = int(result.get(
                    "generation", self.settings_generation + 1))
                self.settings_crc = int(result.get("crc", self.settings_crc))
            else:
                self.settings_generation = (self.settings_generation + 1) & 0xFFFFFFFF
            return True
        except Exception:
            return False

    def _default_settings(self):
        self.vision_p_permille = 1000
        self.vision_d_permille = 1000
        if self.settings_store is None:
            return True
        try:
            # DEFAULT changes the live RAM values only.  Flash/sidecar writes
            # require an explicit SAVE command.
            self.settings_generation = (self.settings_generation + 1) & 0xFFFFFFFF
            self.settings_crc = 0
            return True
        except Exception:
            return False

    def _queue_camera_config(self):
        return self._send_frame(
            TYPE_CONFIG,
            _pack_camera_config(1, self.settings_generation,
                                self.vision_p_permille,
                                self.vision_d_permille),
            priority=True)

    def complete_camera_calibration(self, status, detail=0):
        """Complete the asynchronous CAMERA_CAL command, if one is pending."""
        request_seq = self.pending_camera_cal_seq
        if request_seq is None:
            return False
        self.pending_camera_cal_seq = None
        self._ack(status, OP_CAMERA_CAL, TARGET_CAMERA, request_seq, detail)
        return True

    def publish_camera_calibration_progress(self, accepted, attempted,
                                            reason, ready=False):
        """Publish bounded calibration progress for the complete tool."""
        payload = bytearray(16)
        payload[0] = 1  # CAMERA_CAL event
        payload[1] = 1 if ready else 0
        _put_u16(payload, 2, max(0, min(0xFFFF, int(accepted))))
        _put_u16(payload, 4, max(0, min(0xFFFF, int(attempted))))
        code = 0
        for value in ("" if reason is None else str(reason)):
            code = ((code * 33) + ord(value)) & 0xFFFF
        _put_u16(payload, 6, code)
        return self._send_frame(TYPE_EVENT, payload)

    def _capture_snapshot(self, now_ms):
        if not self._hello_is_fresh(now_ms) or not self._parked():
            return ACK_NOT_PARKED
        if self.capture_busy or self.image_data is not None:
            return ACK_BUSY
        if not callable(self.capture_gray):
            return ACK_NOT_READY
        self.capture_busy = True
        try:
            result = self.capture_gray()
            if isinstance(result, tuple) and len(result) == 3:
                data, width, height = result
            else:
                data, width, height = result, 320, 240
            if not isinstance(data, (bytes, bytearray)):
                return ACK_NOT_READY
            if int(width) != 320 or int(height) != 240 or len(data) != MAX_IMAGE_BYTES:
                return ACK_BAD_FRAME
            # Copy once; the callback is then free to return to the live camera.
            self.image_data = bytearray(data)
            self.image_format = IMAGE_GRAY
            self.image_width = 320
            self.image_height = 240
            self.image_total = len(self.image_data)
            self.capture_frame_id = (self.capture_frame_id + 1) & 0xFFFFFFFF
            self.image_offset = 0
            return ACK_OK
        except Exception:
            self.image_data = None
            return ACK_NOT_READY
        finally:
            self.capture_busy = False

    def _handle_command(self, frame, now_ms):
        payload = frame["payload"]
        sequence = frame["sequence"]
        if len(payload) < 4:
            return self._reject(ACK_BAD_FRAME, 0, TARGET_CAMERA, sequence)
        opcode = int(payload[0])
        target = int(payload[1])
        if payload[2] != 0 or payload[3] != 0:
            return self._reject(ACK_BAD_FRAME, opcode, target, sequence)
        if target != TARGET_CAMERA:
            return self._ack(ACK_FORWARD, opcode, target, sequence)
        header_status = self._validate_command_header(frame, now_ms)
        if header_status == ACK_DUPLICATE:
            if self.last_ack is not None:
                self._send_frame(TYPE_ACK, self.last_ack, priority=True)
            if (self.last_ack_opcode == OP_GET_PARAMS and
                    self.last_ack_target == TARGET_CAMERA):
                self._queue_camera_config()
            return header_status
        if header_status != ACK_OK:
            return self._reject(header_status, opcode, target, sequence)
        self.command_seq_valid = True
        self.last_command_seq = sequence
        status = ACK_UNSUPPORTED
        detail = 0
        parked = self._hello_is_fresh(now_ms) and self._parked()
        if opcode == OP_INFO:
            status = ACK_OK
            detail = self.capabilities
        elif opcode == OP_GET_PARAMS:
            status = ACK_OK
            detail = 16
        elif opcode == OP_SET_PARAMS:
            # The camera exposes only the vision P/D pair.  H743-side driving
            # parameters stay on target MCU and are never silently accepted.
            status = self._apply_params(payload, not parked)
        elif opcode == OP_SAVE:
            status = ACK_OK if parked and self._save_settings() else (
                ACK_NOT_PARKED if not parked else ACK_FAULT)
        elif opcode == OP_DEFAULT:
            status = ACK_OK if parked and self._default_settings() else (
                ACK_NOT_PARKED if not parked else ACK_FAULT)
        elif opcode == OP_CAMERA_CAL:
            if not parked:
                status = ACK_NOT_PARKED
            elif not callable(self.camera_cal):
                status = ACK_NOT_READY
            else:
                try:
                    result = self.camera_cal()
                    if isinstance(result, int) and not isinstance(result, bool):
                        status = int(result)
                    else:
                        status = ACK_OK if result else ACK_FAULT
                    if status == ACK_PENDING:
                        self.pending_camera_cal_seq = sequence
                except Exception:
                    status = ACK_FAULT
        elif opcode == OP_CAPTURE:
            status = self._capture_snapshot(now_ms)
        elif opcode == OP_STOP:
            # MCU executes the stop.  Camera stops any pending image stream.
            self.image_data = None
            self.image_offset = 0
            status = ACK_FORWARD
        elif opcode in (OP_START, OP_TEST_MOTOR, OP_TEST_SERVO, OP_IMU_CAL):
            status = ACK_FORWARD
        self._ack(status, opcode, target, sequence, detail)
        if opcode == OP_GET_PARAMS and status == ACK_OK:
            self._queue_camera_config()
        return status

    def _handle_frame(self, frame, now_ms):
        self.rx_frames += 1
        if frame["type"] == TYPE_HELLO:
            return self._handle_hello(frame, now_ms)
        if frame["type"] != TYPE_COMMAND:
            return None
        return self._handle_command(frame, now_ms)

    def feed_bytes(self, data, length=None, now_ms=None):
        """Consume D3/91 frames from a shared UART stream.

        This method only parses wireless frames.  ``feed_multiplexed`` below
        additionally forwards legacy bytes to ``ReverseControlParser`` and is
        used by the production main loop to prevent image bytes from looking
        like an A6/5A reverse-control frame.
        """
        if not self.enabled:
            return 0
        if length is None:
            length = len(data)
        if now_ms is None:
            now_ms = self._now()
        self._now_ms = now_ms
        handled = 0
        for index in range(max(0, min(int(length), len(data)))):
            frame = self.parser.feed(data[index])
            if frame is not None:
                self._handle_frame(frame, now_ms)
                handled += 1
        return handled

    def _forward_legacy_byte(self, value, parser, now_ms):
        if parser is None:
            return
        self._one_byte[0] = value
        parser.feed(self._one_byte, 1, now_ms)

    def feed_multiplexed(self, data, length, now_ms, reverse_parser,
                         forward_legacy=True):
        """Split complete D3/91 frames from the legacy A6/5A stream.

        A D3 byte inside an in-progress legacy frame is forwarded immediately;
        otherwise it starts an outer candidate.  A complete valid outer frame
        is consumed wholesale, so arbitrary A6/5A bytes in image data can no
        longer renew or create a reverse-control lease.
        """
        if not self.enabled:
            if forward_legacy and reverse_parser is not None:
                reverse_parser.feed(data, length, now_ms)
            return 0
        handled = 0
        for index in range(max(0, min(int(length), len(data)))):
            value = int(data[index]) & 0xFF
            if self._legacy_length == 0:
                buffered = getattr(reverse_parser, "buffered_bytes", 0)
                if value == MAGIC0 and buffered == 0:
                    self._legacy_candidate[0] = value
                    self._legacy_length = 1
                    continue
                if forward_legacy:
                    self._forward_legacy_byte(value, reverse_parser, now_ms)
                continue
            if self._legacy_length == 1 and value != MAGIC1:
                # A lone D3 was not a wireless frame.  It cannot be part of a
                # valid legacy sync, but forwarding it preserves byte order.
                if forward_legacy:
                    self._forward_legacy_byte(MAGIC0, reverse_parser, now_ms)
                self._legacy_length = 0
                self._legacy_expected = 0
                if value == MAGIC0 and getattr(reverse_parser, "buffered_bytes", 0) == 0:
                    self._legacy_candidate[0] = value
                    self._legacy_length = 1
                elif forward_legacy:
                    self._forward_legacy_byte(value, reverse_parser, now_ms)
                continue
            if self._legacy_length >= MAX_FRAME:
                self.parser.length_errors += 1
                self._legacy_length = 0
                self._legacy_expected = 0
                if forward_legacy:
                    self._forward_legacy_byte(value, reverse_parser, now_ms)
                continue
            self._legacy_candidate[self._legacy_length] = value
            self._legacy_length += 1
            if self._legacy_length == HEADER_SIZE:
                if (self._legacy_candidate[2] != VERSION or
                        _u16(self._legacy_candidate, 6) != 0 or
                        _u16(self._legacy_candidate, 4) > MAX_PAYLOAD):
                    self._legacy_length = 0
                    self._legacy_expected = 0
                    # Drop a malformed outer candidate as a bounded wireless
                    # error; do not expose its payload to legacy parsing.
                    self.parser.length_errors += 1
                    continue
                self._legacy_expected = (HEADER_SIZE +
                                         _u16(self._legacy_candidate, 4) +
                                         CRC_SIZE)
            if self._legacy_expected and self._legacy_length == self._legacy_expected:
                frame = decode_frame(self._legacy_candidate[:self._legacy_length])
                self._legacy_length = 0
                self._legacy_expected = 0
                if frame is not None:
                    self._handle_frame(frame, now_ms)
                    handled += 1
                else:
                    self.parser.crc_errors += 1
        return handled

    def publish_frame(self, now_ms, frame_id, valid, quality, source,
                      ring_state, side, lateral_error, control,
                      processing_ms, fps, curve_phase, slowdown, profile,
                      profile_active, session_token, detector_reason,
                      processing_us=0, capture_us=0, tx_errors=0,
                      memory_free=0, mask_info=None):
        """Queue telemetry at bounded rates; call after the 16-byte V4 write."""
        if not self.enabled or not self._hello_clock_is_fresh(now_ms):
            return
        if (self.last_state_ms is None or
                _ticks_diff(now_ms, self.last_state_ms) >= self.state_period_ms):
            self.last_state_ms = now_ms
            self._send_frame(TYPE_STATE, _pack_state(
                frame_id, valid, quality, source, ring_state, side,
                lateral_error, control, processing_ms, int(float(fps) * 10),
                curve_phase, slowdown, profile, profile_active,
                session_token, detector_reason))
        if (self.last_diag_ms is None or
                _ticks_diff(now_ms, self.last_diag_ms) >= self.diag_period_ms):
            self.last_diag_ms = now_ms
            parser_errors = (self.parser.crc_errors + self.parser.length_errors +
                             self.parser.sync_errors + self.parser.reserved_errors)
            flags = (1 if self._hello_is_fresh(now_ms) else 0) | (
                2 if self.capture_busy else 0)
            self._send_frame(TYPE_DIAG, _pack_diag(
                frame_id, processing_us, capture_us, tx_errors,
                parser_errors, self.command_errors, memory_free, flags))
        if (mask_info is not None and
                (self.last_binary_ms is None or
                 _ticks_diff(now_ms, self.last_binary_ms) >=
                 self.binary_period_ms)):
            binary = pack_binary_80x60(mask_info)
            self.last_binary_ms = now_ms
            self._start_image(binary, IMAGE_BINARY, 80, 60)

    def _start_image(self, data, format_id, width, height):
        if self.image_data is not None:
            return False
        if len(data) > MAX_IMAGE_BYTES:
            return False
        self.image_data = bytearray(data)
        self.image_format = format_id
        self.image_width = width
        self.image_height = height
        self.image_total = len(self.image_data)
        self.image_offset = 0
        self.capture_frame_id = (self.capture_frame_id + 1) & 0xFFFFFFFF
        return True

    def _queue_image_chunk(self, now_ms):
        if self.image_data is None:
            return False
        offset = self.image_offset
        remaining = self.image_total - offset
        count = min(IMAGE_DATA_LIMIT, remaining)
        flags = 0
        if offset == 0:
            flags |= IMAGE_FLAGS_FIRST
        if count == remaining:
            flags |= IMAGE_FLAGS_LAST
        payload = _pack_image(
            self.image_format, flags, self.capture_frame_id,
            offset, self.image_total, self.image_width,
            self.image_height, self.image_data,
            self.image_data[offset:offset + count])
        if not self._send_frame(TYPE_IMAGE, payload, timestamp_ms=now_ms):
            return False
        self.image_offset += count
        if self.image_offset >= self.image_total:
            self.image_data = None
            self.image_offset = 0
        return True

    def _write_one(self, uart):
        if not self.tx_queue:
            return False
        frame = self.tx_queue.pop(0)
        try:
            written = uart.write(frame)
            if written is None or int(written) != len(frame):
                self.tx_dropped += 1
                return False
            return True
        except Exception:
            self.tx_dropped += 1
            return False

    def service_tx(self, uart, now_ms=None):
        """Write bounded side-band frames after the current V4 frame.

        One queued ACK/telemetry frame is sent first. If another queued frame
        remains it gets the second slot; otherwise one image chunk uses that
        slot. This keeps control and numeric telemetry ahead of images without
        starving a parked capture stream.
        """
        if not self.enabled:
            return False
        if now_ms is None:
            now_ms = self._now()
        if self.tx_queue:
            if not self._write_one(uart):
                return False
            if self.tx_queue:
                return self._write_one(uart)
            if self._queue_image_chunk(now_ms):
                return self._write_one(uart)
            return True
        if self._queue_image_chunk(now_ms):
            return self._write_one(uart)
        return False


def pack_binary_80x60(mask_info):
    """Pack the current 80x60 road mask, one bit per cell, row-major."""
    source = None
    if isinstance(mask_info, dict):
        source = mask_info.get("road_mask")
        if source is None:
            source = mask_info.get("luma")
    if source is None:
        source = bytearray(80 * 60)
    result = bytearray((80 * 60 + 7) // 8)
    limit = min(80 * 60, len(source))
    for index in range(limit):
        value = int(source[index])
        if value:
            result[index >> 3] |= 1 << (index & 7)
    return result


__all__ = [
    "WirelessCameraLink", "OuterParser", "encode_frame", "decode_frame",
    "crc32", "pack_binary_80x60", "TYPE_HELLO", "TYPE_HELLO_REPLY",
    "TYPE_COMMAND", "TYPE_ACK", "TYPE_STATE", "TYPE_DIAG", "TYPE_IMAGE",
    "TYPE_CONFIG", "TYPE_EVENT",
    "TARGET_MCU", "TARGET_CAMERA", "OP_INFO", "OP_START", "OP_STOP",
    "OP_SET_PARAMS", "OP_SAVE", "OP_DEFAULT", "OP_TEST_MOTOR",
    "OP_TEST_SERVO", "OP_IMU_CAL", "OP_CAMERA_CAL", "OP_CAPTURE",
    "OP_GET_PARAMS",
    "ACK_OK", "ACK_FORWARD", "ACK_DUPLICATE", "ACK_STALE",
    "ACK_NOT_PARKED", "ACK_BAD_SESSION", "ACK_OUT_OF_ORDER", "ACK_BUSY",
    "ACK_NOT_READY", "PARAM_VISION_P", "PARAM_VISION_D",
]
