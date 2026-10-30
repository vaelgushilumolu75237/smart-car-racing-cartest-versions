"""MicroPython-safe encoder for the strict smart-car vision UART V2 frame."""

FRAME_SIZE = 16
SYNC_BYTE_0 = 0xA5
SYNC_BYTE_1 = 0x5A
VERSION = 2
PAYLOAD_LENGTH = 8
FLAG_VALID = 0x01
FLAG_SLOPE_HINT = 0x02
KNOWN_FLAGS_MASK = FLAG_VALID | FLAG_SLOPE_HINT
MAX_CONFIDENCE = 100
MIN_LATERAL_ERROR = -1000
MAX_LATERAL_ERROR = 1000
CURVE_PHASE_STRAIGHT = 0
CURVE_PHASE_CANDIDATE = 1
CURVE_PHASE_TURN = 2
CURVE_PHASE_EXIT = 3
MAX_SLOWDOWN_REQUEST_PERMILLE = 1000


def _require_int(value, name):
    if not isinstance(value, int):
        raise TypeError(name + " must be an integer")


def _check_frame(sequence, valid, confidence, lateral_error, slope_hint,
                 curve_phase, slowdown_request_permille):
    _require_int(sequence, "sequence")
    _require_int(valid, "valid")
    _require_int(confidence, "confidence")
    _require_int(lateral_error, "lateral_error")
    _require_int(slope_hint, "slope_hint")
    _require_int(curve_phase, "curve_phase")
    _require_int(slowdown_request_permille, "slowdown_request_permille")

    if sequence < 0 or sequence > 0xFFFF:
        raise ValueError("sequence must be in 0..65535")
    if valid != 0 and valid != 1:
        raise ValueError("valid must be 0 or 1")
    if slope_hint != 0 and slope_hint != 1:
        raise ValueError("slope_hint must be 0 or 1")
    if confidence < 0 or confidence > MAX_CONFIDENCE:
        raise ValueError("confidence must be in 0..100")
    if lateral_error < MIN_LATERAL_ERROR or lateral_error > MAX_LATERAL_ERROR:
        raise ValueError("lateral_error must be in -1000..1000")
    if curve_phase < CURVE_PHASE_STRAIGHT or curve_phase > CURVE_PHASE_EXIT:
        raise ValueError("curve_phase must be in 0..3")
    if (slowdown_request_permille < 0 or
            slowdown_request_permille > MAX_SLOWDOWN_REQUEST_PERMILLE):
        raise ValueError("slowdown_request_permille must be in 0..1000")
    if valid == 0 and (confidence != 0 or lateral_error != 0 or slope_hint != 0 or
                       curve_phase != CURVE_PHASE_STRAIGHT or
                       slowdown_request_permille != 0):
        raise ValueError("invalid frames must zero every observation/control field")


def _crc16_ccitt_false_range(data, start, end):
    crc = 0xFFFF
    for index in range(start, end):
        crc ^= data[index] << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_false(data):
    """Return CRC-16/CCITT-FALSE for an iterable of byte values."""
    if data is None:
        raise TypeError("data must be a byte sequence")

    crc = 0xFFFF
    for value in data:
        _require_int(value, "data byte")
        if value < 0 or value > 0xFF:
            raise ValueError("data byte must be in 0..255")
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_into(out, sequence, valid, confidence, lateral_error,
              slope_hint=0, curve_phase=0,
              slowdown_request_permille=0):
    """Write a validated V2 frame into a caller-owned 16-byte bytearray.

    The supplied buffer may be larger than 16 bytes; only its first 16 bytes
    are changed. Invalid input raises TypeError or ValueError. The same buffer
    is returned on success.
    """
    _check_frame(sequence, valid, confidence, lateral_error, slope_hint,
                 curve_phase, slowdown_request_permille)
    if not isinstance(out, bytearray):
        raise TypeError("out must be a writable bytearray")
    if len(out) < FRAME_SIZE:
        raise ValueError("out must contain at least 16 bytes")

    out[0] = SYNC_BYTE_0
    out[1] = SYNC_BYTE_1
    out[2] = VERSION
    out[3] = PAYLOAD_LENGTH
    out[4] = sequence & 0xFF
    out[5] = (sequence >> 8) & 0xFF
    flags = FLAG_VALID if valid else 0
    if slope_hint:
        flags |= FLAG_SLOPE_HINT
    out[6] = flags
    out[7] = confidence

    raw_error = lateral_error & 0xFFFF
    out[8] = raw_error & 0xFF
    out[9] = (raw_error >> 8) & 0xFF
    out[10] = curve_phase
    out[11] = 0
    out[12] = slowdown_request_permille & 0xFF
    out[13] = (slowdown_request_permille >> 8) & 0xFF

    crc = _crc16_ccitt_false_range(out, 2, 14)
    out[14] = crc & 0xFF
    out[15] = (crc >> 8) & 0xFF
    return out


def pack_frame(sequence, valid, confidence, lateral_error, slope_hint=0,
               curve_phase=0, slowdown_request_permille=0):
    """Pack one validated observation into a 16-byte bytearray."""
    out = bytearray(FRAME_SIZE)
    return pack_into(out, sequence, valid, confidence, lateral_error,
                     slope_hint, curve_phase, slowdown_request_permille)


def pack(sequence, valid, confidence, lateral_error, slope_hint=0,
         curve_phase=0, slowdown_request_permille=0):
    """Short alias for pack_frame used by the vision loop."""
    return pack_frame(sequence, valid, confidence, lateral_error, slope_hint,
                      curve_phase, slowdown_request_permille)
