"""OpenMV numeric/command sideband; no image transport or PC application.

D3/91 framing is shared with T2. Reverse controls stay on A6/5A and the
16-byte vision packet is always sent before a bounded sideband packet.
"""
import time
import os
try:
    import ujson as json
except ImportError:
    import json
try:
    import binascii
except ImportError:
    binascii = None
MAGIC0=0xD3
MAGIC1=0x91
VERSION=1
HEADER_SIZE=20
CRC_SIZE=4
MAX_PAYLOAD=160
MAX_FRAME=184
TYPE_HELLO=1
TYPE_HELLO_REPLY=2
TYPE_COMMAND=3
TYPE_ACK=4
TYPE_STATE=16
TYPE_CONFIG=19
TYPE_EVENT=20
TARGET_CAMERA=1
OP_SET_PARAMS=4
OP_SAVE=5
OP_DEFAULT=6
OP_CAMERA_CAL=10
OP_VISUAL_ENTRY_CAL=11
OP_GET_PARAMS=14
ACK_OK=0
ACK_PENDING=1
ACK_BAD_FRAME=2
ACK_BAD_SESSION=3
ACK_STALE=4
ACK_DUPLICATE=5
ACK_OUT_OF_ORDER=6
ACK_NOT_PARKED=7
ACK_BUSY=8
ACK_DENIED=9
ACK_RANGE=10
ACK_UNSUPPORTED=11
ACK_BAD_PARAM=12
ACK_NOT_READY=15
PARAM_VISION_P=3
PARAM_VISION_D=17
VISUAL_ENTRY_RESET=0
VISUAL_ENTRY_MARK=1
VISUAL_ENTRY_COMMIT=2
# VISUAL_ENTRY_COMMIT ACK detail is a result bitmask.  Bit 0 means the
# visual document was written and read back; bit 1 means the independent
# encoder companion was written and read back.  A partial commit therefore
# remains observable as 1 or 2 instead of being reported as full success.
VISUAL_ENTRY_RESULT_VISUAL_READY = 1
VISUAL_ENTRY_RESULT_ENCODER_READY = 2
VISUAL_ENTRY_RESULT_MASK = (VISUAL_ENTRY_RESULT_VISUAL_READY |
                            VISUAL_ENTRY_RESULT_ENCODER_READY)

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


_VISUAL_ENTRY_REASON_CODES = {
    "MISSING": 1, "READY": 2, "READY_PREVIOUS": 3,
    "CALIBRATION_RESET": 4, "CALIBRATION_UPDATED": 5,
    "WAITING_FOR_INSIDE": 6, "ENTRY_GEOMETRY_WAIT": 7,
    "ENTRY_GEOMETRY_MATCH": 8, "ENTRY_GEOMETRY_CONFIRMING": 9,
    "ENTRY_ACCEPTED": 10, "FRAME_INVALID": 11,
    "SIDE_INVALID": 12, "SIDE_NOT_CALIBRATED": 13,
    "RING_OBSERVATION_RESET": 14,
}


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


class ProfileStore:
    """Six validated PD records, atomic replacement and CRC/readback."""
    def __init__(self, path):
        self.path = path
        self.rows = [[1000, 1000] for _ in range(6)]
        self.generation = 0
        self.load()

    def _body(self, rows, generation):
        return ("openmv-pd-v1|%d|" % generation +
                "|".join("%d,%d" % (r[0], r[1]) for r in rows)).encode()

    def _validate(self, record):
        rows, generation = record["profiles"], record["generation"]
        if (record["version"] != 1 or type(generation) is not int or
                not 0 <= generation <= 0xffffffff or len(rows) != 6 or
                any(len(r) != 2 or any(type(v) is not int or not 500 <= v <= 1500
                                       for v in r) for r in rows) or
                record["crc"] != crc32(self._body(rows, generation))):
            raise ValueError("invalid profile settings")
        return rows, generation

    def load(self):
        for candidate in (self.path, self.path + ".previous"):
            try:
                with open(candidate, "r") as stream:
                    rows, generation = self._validate(json.loads(stream.read()))
                self.rows, self.generation = rows, generation
                return True
            except (OSError, ValueError, KeyError, TypeError):
                pass
        return False

    def save(self, key, p, d):
        rows = [r[:] for r in self.rows]
        rows[key] = [p, d]
        generation = (self.generation + 1) & 0xffffffff
        record = dict(version=1, profiles=rows, generation=generation,
                      crc=crc32(self._body(rows, generation)))
        self._validate(record)
        temporary = self.path + ".new"
        with open(temporary, "w") as stream:
            stream.write(json.dumps(record))
        with open(temporary, "r") as stream:
            self._validate(json.loads(stream.read()))
        # FAT/OpenMV rename semantics may reject replacing an existing name.
        # Preserve the prior valid record until the new record is verified.
        backup = self.path + ".previous"
        try:
            os.remove(backup)
        except OSError:
            pass
        had_previous = False
        try:
            os.rename(self.path, backup)
            had_previous = True
        except OSError:
            pass
        try:
            os.rename(temporary, self.path)
            with open(self.path, "r") as stream:
                checked_rows, checked_generation = self._validate(json.loads(stream.read()))
        except Exception:
            if had_previous:
                try:
                    os.remove(self.path)
                except OSError:
                    pass
                os.rename(backup, self.path)
            raise
        self.rows, self.generation = checked_rows, checked_generation
        return generation


class WirelessSideband:
    def __init__(self, reverse_parser, store=None):
        self.reverse = reverse_parser
        self.store = store
        self.parser = OuterParser()
        self.one = bytearray(1)
        self.partial_ms = None
        self.session = 0
        self.key = 0
        self.last_hello = None
        self.last_hello_sequence = None
        self.offset = 0
        self.parked_hello = False
        self.command_sequence = None
        self.cached_ack = None
        self.serial = 0
        self.queue = []
        self.vision_p = self.vision_d = 1000
        self.generation = 0
        self.dropped = self.tx_errors = 0
        self.last_state_ms = None
        self.calibration_requested = False
        self.calibration_sequence = None
        self.last_calibration_progress_ms = None
        self.visual_entry_request = None
        self.visual_entry_sequence = None
        self.last_scaled_output = None

    def apply_control(self, info, limit):
        """Scale the actual P and D outputs, retaining the original PD history."""
        if self.vision_p == 1000 and self.vision_d == 1000:
            self.last_scaled_output = info["pd_output"]
            return
        p, d = info.get("position_p", 0), info.get("derivative_d", 0)
        p = (-1 if p < 0 else 1) * (abs(p) * self.vision_p // 1000)
        d = (-1 if d < 0 else 1) * (abs(d) * self.vision_d // 1000)
        info["position_p"], info["derivative_d"] = p, d
        output = max(-limit, min(limit, p + d))
        if info.get("duplicate") and info.get("reset_code", 0) == 0 and self.last_scaled_output is not None:
            output = self.last_scaled_output
        info["pd_output"] = self.last_scaled_output = output

    def _fresh(self, now):
        return self.last_hello is not None and 0 <= _ticks_diff(now, self.last_hello) < 500

    def parked(self, now):
        return bool(self._fresh(now) and self.parked_hello and
                    self.reverse.profile_is_fresh(now, _ticks_diff) and
                    not self.reverse.profile_session_active)

    def _send(self, kind, payload, now, priority=True):
        if len(self.queue) >= 4:
            if not priority:
                self.dropped += 1
                return
            # Never evict an operation ACK to insert another diagnostic.
            for i, item in enumerate(self.queue):
                if item[3] == TYPE_STATE:
                    self.queue.pop(i)
                    break
            else:
                self.dropped += 1
                return
        self.serial = (self.serial + 1) & 0xffffffff
        self.queue.append(encode_frame(kind, self.serial, self.session, now, payload))

    def _ack(self, status, opcode, sequence, now, detail=0, cache=True):
        payload = _pack_ack(status, opcode, TARGET_CAMERA, sequence, detail, self.generation)
        if cache:
            self.cached_ack = payload
        self._send(TYPE_ACK, payload, now)

    def _config(self, now):
        self._send(TYPE_CONFIG, _pack_camera_config(1, self.generation,
                                                   self.vision_p, self.vision_d), now)

    def _hello(self, frame, now):
        p = frame["payload"]
        if (len(p) != 8 or p[4] not in (0, 1) or p[5] >= 6 or p[6] or p[7] or
                not frame["session"] or _u32(p, 0) != frame["session"]):
            return
        if (frame["session"] == self.session and self.last_hello_sequence is not None and
                not _sequence_forward(frame["sequence"], self.last_hello_sequence)):
            return
        changed = self.session != frame["session"] or self.key != p[5]
        if changed:
            self.session, self.key = frame["session"], p[5]
            self.command_sequence = self.cached_ack = None
            self.queue[:] = []
            self.calibration_requested = False
            self.calibration_sequence = None
            self.visual_entry_request = None
            self.visual_entry_sequence = None
            self.vision_p, self.vision_d = (self.store.rows[self.key]
                                            if self.store else (1000, 1000))
            self.generation = self.store.generation if self.store else 0
            self.last_scaled_output = None
        self.offset = (now - frame["timestamp_ms"]) & 0xffffffff
        self.last_hello, self.last_hello_sequence = now, frame["sequence"]
        self.parked_hello = p[4] == 1
        self._send(TYPE_HELLO_REPLY, _pack_hello_reply(self.session, 0x19), now)

    def _command(self, frame, now):
        p, seq = frame["payload"], frame["sequence"]
        op = p[0] if p else 0
        if not self._fresh(now) or frame["session"] != self.session:
            self._ack(ACK_BAD_SESSION, op, seq, now, cache=False)
            return
        age = ((now - self.offset - frame["timestamp_ms"] + 0x80000000) & 0xffffffff) - 0x80000000
        if age < -50 or age > 1000:
            self._ack(ACK_STALE, op, seq, now, cache=False)
            return
        if self.command_sequence is not None:
            if seq == self.command_sequence:
                if self.cached_ack is not None:
                    self._send(TYPE_ACK, self.cached_ack, now)
                if op == OP_GET_PARAMS:
                    self._config(now)
                return
            if not _sequence_forward(seq, self.command_sequence):
                self._ack(ACK_OUT_OF_ORDER, op, seq, now, cache=False)
                return
        self.command_sequence = seq
        status = ACK_OK
        if len(p) < 4 or p[1] != TARGET_CAMERA or p[2] or p[3]:
            status = ACK_BAD_FRAME
        elif op in (OP_SAVE, OP_DEFAULT, OP_CAMERA_CAL,
                    OP_VISUAL_ENTRY_CAL) and not self.parked(now):
            status = ACK_NOT_PARKED
        elif op == OP_SET_PARAMS:
            if len(p) < 6 or p[4] not in (1, 2) or not 1 <= p[5] <= 2 or len(p) != 6 + p[5] * 5:
                status = ACK_BAD_PARAM
            else:
                candidate_p, candidate_d = self.vision_p, self.vision_d
                seen = 0
                for i in range(p[5]):
                    identifier, value = p[6+i*5], _i32(p, 7+i*5)
                    flag = 1 if identifier == PARAM_VISION_P else 2 if identifier == PARAM_VISION_D else 0
                    if not flag or seen & flag:
                        status = ACK_BAD_PARAM
                        break
                    seen |= flag
                    if not 500 <= value <= 1500:
                        status = ACK_RANGE
                        break
                    if flag == 1:
                        candidate_p = value
                    else:
                        candidate_d = value
                if status == ACK_OK:
                    self.vision_p, self.vision_d = candidate_p, candidate_d
                    self.generation = (self.generation + 1) & 0xffffffff
        elif op == OP_SAVE:
            try:
                if self.store is None:
                    raise OSError("no settings store")
                self.generation = self.store.save(self.key, self.vision_p, self.vision_d)
            except Exception:
                status = ACK_NOT_READY
        elif op == OP_DEFAULT:
            self.vision_p = self.vision_d = 1000
            self.generation = (self.generation + 1) & 0xffffffff
        elif op == OP_CAMERA_CAL:
            if self.calibration_sequence is not None:
                status = ACK_BUSY
            else:
                self.calibration_requested = True
                self.calibration_sequence = seq
                status = ACK_PENDING
        elif op == OP_VISUAL_ENTRY_CAL:
            # Payload: action, side, marker.  RESET and COMMIT use side/marker
            # zero; MARK captures the current camera geometry at that marker.
            if len(p) != 7 or p[4] not in (VISUAL_ENTRY_RESET,
                                           VISUAL_ENTRY_MARK,
                                           VISUAL_ENTRY_COMMIT) or \
                    p[5] not in (0, 1) or p[6] not in (0, 1, 2):
                status = ACK_BAD_PARAM
            elif self.visual_entry_sequence is not None:
                status = ACK_BUSY
            else:
                self.visual_entry_request = {
                    "action": p[4], "side": p[5], "marker": p[6],
                    "sequence": seq}
                self.visual_entry_sequence = seq
                status = ACK_PENDING
        elif op != OP_GET_PARAMS:
            status = ACK_UNSUPPORTED
        self._ack(status, op, seq, now)
        if op in (OP_GET_PARAMS, OP_SET_PARAMS, OP_DEFAULT) and status == ACK_OK:
            self._config(now)

    def feed_multiplexed(self, data, length, now_ms, reverse_parser=None):
        reverse = self.reverse if reverse_parser is None else reverse_parser
        if self.partial_ms is not None and _ticks_diff(now_ms, self.partial_ms) >= 250:
            self.parser.reset()
            self.partial_ms = None
        count = min(length, len(data), 64)
        if not self.parser.length and data.find(b"\xd3", 0, count) < 0:
            reverse.feed(data, count, now_ms)
            return
        for index in range(count):
            value = data[index]
            if self.parser.length or (value == MAGIC0 and not reverse.buffered_bytes):
                if not self.parser.length:
                    self.partial_ms = now_ms
                frame = self.parser.feed(value)
                if frame is not None:
                    self.partial_ms = None
                    if frame["type"] == TYPE_HELLO:
                        self._hello(frame, now_ms)
                    elif frame["type"] == TYPE_COMMAND:
                        self._command(frame, now_ms)
                # Never present an outer frame's payload to the control parser.
                continue
            self.one[0] = value
            reverse.feed(self.one, 1, now_ms)

    def calibration_progress(self, now, accepted, attempted, ready, done=False):
        if self.calibration_sequence is None:
            return
        if (not done and self.last_calibration_progress_ms is not None and
                _ticks_diff(now, self.last_calibration_progress_ms) < 500):
            return
        self.last_calibration_progress_ms = now
        payload = bytearray(8)
        payload[0], payload[1] = 1, int(ready)
        _put_u16(payload, 2, min(65535, accepted))
        _put_u16(payload, 4, min(65535, attempted))
        if done:
            self._ack(ACK_OK if ready else ACK_NOT_READY, OP_CAMERA_CAL,
                      self.calibration_sequence, now, accepted | (attempted << 16))
            self.calibration_sequence = None
        else:
            self._send(TYPE_EVENT, payload, now, priority=False)

    def visual_entry_complete(self, now, status, detail=0):
        """Complete one parked three-marker visual entrance command."""
        if self.visual_entry_sequence is None:
            return
        sequence = self.visual_entry_sequence
        self._ack(status, OP_VISUAL_ENTRY_CAL, sequence, now, detail)
        self.visual_entry_sequence = None
        self.visual_entry_request = None

    def publish(self, now, frame_id, valid, quality, source, ring_info,
                lateral, control, processing_ms, fps_x10, curve, slowdown):
        if not self._fresh(now) or (self.last_state_ms is not None and
                                   _ticks_diff(now, self.last_state_ms) < 100):
            return
        self.last_state_ms = now
        p = bytearray(36)
        _put_u32(p, 0, frame_id)
        p[4], p[5] = int(valid), min(100, max(0, quality))
        p[6] = {"road": 1, "single_left": 2, "single_right": 3,
                "ring_left": 4, "ring_right": 5}.get(source, 0)
        p[7] = ring_info.get("state_id", 0)
        track_side = ring_info.get("side", -1)
        p[8] = 1 if track_side == 0 else 2 if track_side == 1 else 0
        _put_i16(p, 9, lateral); _put_i16(p, 11, control)
        _put_u16(p, 13, min(65535, max(0, processing_ms)))
        _put_u16(p, 15, min(65535, max(0, fps_x10)))
        p[17] = curve
        _put_u16(p, 18, slowdown)
        p[20] = self.reverse.profile_id
        p[21] = int(self.reverse.profile_session_active)
        _put_u32(p, 22, self.reverse.session_token)
        # The remaining bytes are a compact visual-entry gate diagnostic.  It
        # is deliberately additive so older H743 firmware can continue to
        # parse the first 26 bytes of STATE frames.
        p[26] = int(bool(ring_info.get("entry_ready")))
        p[27] = _code(ring_info.get("entry_gate_state"), {})
        p[28] = int(bool(ring_info.get("entry_gate_calibration_ready")))
        p[29] = min(255, max(0, int(ring_info.get("entry_gate_match_frames", 0))))
        p[30] = int(bool(ring_info.get("entry_transition")))
        p[31] = int(bool(ring_info.get("ring_confirmed")))
        far_y = ring_info.get("entry_transition_far_y", -1)
        near_y = ring_info.get("entry_transition_near_y", -1)
        p[32] = 255 if far_y is None or int(far_y) < 0 else min(119, int(far_y))
        p[33] = 255 if near_y is None or int(near_y) < 0 else min(119, int(near_y))
        p[34] = _code(ring_info.get("entry_gate_reason"),
                      _VISUAL_ENTRY_REASON_CODES)
        p[35] = min(255, max(0, int(ring_info.get("entry_gate_side", -1) + 1)))
        self._send(TYPE_STATE, p, now, priority=False)

    def service_tx(self, uart, now):
        if not self._fresh(now):
            self.queue[:] = []
            return
        if not self.queue:
            return
        done = getattr(uart, "fifo_txdone", None)
        if callable(done) and done() is False:
            # The primary 16-byte FIFO frame has just been queued. Skipping
            # here every iteration starves HELLO/ACK/STATE on fast loops.
            # At 115200 8N1 its wire time is 1389 us. Bound the wait to 2 ms;
            # a stuck transmitter leaves the packet queued for a later try.
            ticks_us = getattr(time, "ticks_us", None)
            if not callable(ticks_us):
                return
            started = ticks_us()
            while done() is False:
                if _ticks_diff(ticks_us(), started) >= 2000:
                    return
        packet = self.queue.pop(0)
        try:
            if uart.write(packet) != len(packet):
                self.tx_errors += 1
        except Exception:
            self.tx_errors += 1


