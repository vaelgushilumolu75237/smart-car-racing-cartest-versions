"""MicroPython-safe encoder for the smart-car vision UART protocol v1."""

FRAME_SIZE = 12
SYNC_BYTE_0 = 0xA5
SYNC_BYTE_1 = 0x5A
VERSION = 1
PAYLOAD_LENGTH = 4
FLAG_VALID = 0x01
FLAG_SLOPE_HINT = 0x02
KNOWN_FLAGS_MASK = FLAG_VALID | FLAG_SLOPE_HINT
MAX_CONFIDENCE = 100
MIN_LATERAL_ERROR = -1000
MAX_LATERAL_ERROR = 1000


def _require_int(value, name):
    if not isinstance(value, int):
        raise TypeError(name + " must be an integer")


def _check_frame(sequence, valid, confidence, lateral_error, slope_hint):
    _require_int(sequence, "sequence")
    _require_int(valid, "valid")
    _require_int(confidence, "confidence")
    _require_int(lateral_error, "lateral_error")
    _require_int(slope_hint, "slope_hint")

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
    if valid == 0 and (confidence != 0 or lateral_error != 0 or slope_hint != 0):
        raise ValueError(
            "invalid frames must carry zero confidence, error, and flags")


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


def pack_frame(sequence, valid, confidence, lateral_error, slope_hint=0):
    """Pack one validated observation into a 12-byte bytearray.

    ``slope_hint`` is an optional valid-frame class bit. It is diagnostic
    context only and does not encode a speed or drive command.
    """
    _check_frame(sequence, valid, confidence, lateral_error, slope_hint)

    out = bytearray(FRAME_SIZE)
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

    crc = crc16_ccitt_false(out[2:10])
    out[10] = crc & 0xFF
    out[11] = (crc >> 8) & 0xFF
    return out


def pack(sequence, valid, confidence, lateral_error, slope_hint=0):
    """Short alias for pack_frame used by the vision loop."""
    return pack_frame(sequence, valid, confidence, lateral_error, slope_hint)
