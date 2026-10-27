"""MicroPython-safe encoder for fixed-layout smart-car vision frames."""

FRAME_SIZE = 16
SYNC_BYTE_0 = 0xA5
SYNC_BYTE_1 = 0x5A
VERSION = 2
VERSION_V3 = 3
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

try:
    import micropython
except ImportError:  # CPython host tests
    class _HostMicroPython:
        @staticmethod
        def native(function):
            return function

    micropython = _HostMicroPython()

# CRC-16/CCITT-FALSE nibble table, computed for the 0x1021 polynomial.
# Kept as a fixed tuple so the runtime CRC path allocates no table or buffers.
_CRC16_NIBBLE_TABLE = (
    0x0000, 0x1021, 0x2042, 0x3063,
    0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B,
    0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
)


@micropython.native
def _require_int(value, name):
    if not isinstance(value, int):
        raise TypeError(name + " must be an integer")


@micropython.native
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


@micropython.native
def _crc16_ccitt_false_range(data, start, end):
    table = _CRC16_NIBBLE_TABLE
    crc = 0xFFFF
    for index in range(start, end):
        crc ^= data[index] << 8
        crc = ((crc << 4) ^
               table[(crc >> 12) & 0x0F]) & 0xFFFF
        crc = ((crc << 4) ^
               table[(crc >> 12) & 0x0F]) & 0xFFFF
    return crc


@micropython.native
def crc16_ccitt_false(data):
    """Return CRC-16/CCITT-FALSE for an iterable of byte values."""
    if data is None:
        raise TypeError("data must be a byte sequence")

    table = _CRC16_NIBBLE_TABLE
    crc = 0xFFFF
    for value in data:
        _require_int(value, "data byte")
        if value < 0 or value > 0xFF:
            raise ValueError("data byte must be in 0..255")
        crc ^= value << 8
        crc = ((crc << 4) ^
               table[(crc >> 12) & 0x0F]) & 0xFFFF
        crc = ((crc << 4) ^
               table[(crc >> 12) & 0x0F]) & 0xFFFF
    return crc


@micropython.native
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

    return _pack_into_version(
        out, sequence, valid, confidence, lateral_error, slope_hint,
        curve_phase, slowdown_request_permille, VERSION, 0)


@micropython.native
def _pack_into_version(out, sequence, valid, confidence, lateral_error,
                       slope_hint, curve_phase,
                       slowdown_request_permille, version, profile_ack):
    """Pack the common 16-byte layout with a selected version/byte 11."""
    _check_frame(sequence, valid, confidence, lateral_error, slope_hint,
                 curve_phase, slowdown_request_permille)
    if not isinstance(out, bytearray):
        raise TypeError("out must be a writable bytearray")
    if len(out) < FRAME_SIZE:
        raise ValueError("out must contain at least 16 bytes")
    if version not in (VERSION, VERSION_V3):
        raise ValueError("unsupported vision frame version")
    if (isinstance(profile_ack, bool) or not isinstance(profile_ack, int) or
            profile_ack < 0 or profile_ack > 3):
        raise ValueError("profile_ack must be in 0..3")

    out[0] = SYNC_BYTE_0
    out[1] = SYNC_BYTE_1
    out[2] = version
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
    out[11] = profile_ack if version == VERSION_V3 else 0
    out[12] = slowdown_request_permille & 0xFF
    out[13] = (slowdown_request_permille >> 8) & 0xFF

    crc = _crc16_ccitt_false_range(out, 2, 14)
    out[14] = crc & 0xFF
    out[15] = (crc >> 8) & 0xFF
    return out


def pack_into_v3(out, sequence, valid, confidence, lateral_error,
                 slope_hint=0, curve_phase=0,
                 slowdown_request_permille=0, profile_ack=0):
    """Pack a V3 frame; byte 11 confirms an acknowledged A/B/C profile."""
    return _pack_into_version(
        out, sequence, valid, confidence, lateral_error, slope_hint,
        curve_phase, slowdown_request_permille, VERSION_V3, profile_ack)


def pack_frame(sequence, valid, confidence, lateral_error, slope_hint=0,
               curve_phase=0, slowdown_request_permille=0):
    """Pack one validated observation into a 16-byte bytearray."""
    out = bytearray(FRAME_SIZE)
    return pack_into(out, sequence, valid, confidence, lateral_error,
                     slope_hint, curve_phase, slowdown_request_permille)


def pack_frame_v3(sequence, valid, confidence, lateral_error, slope_hint=0,
                  curve_phase=0, slowdown_request_permille=0,
                  profile_ack=0):
    """Pack one V3 observation into a new 16-byte bytearray."""
    out = bytearray(FRAME_SIZE)
    return pack_into_v3(
        out, sequence, valid, confidence, lateral_error, slope_hint,
        curve_phase, slowdown_request_permille, profile_ack)


def pack(sequence, valid, confidence, lateral_error, slope_hint=0,
         curve_phase=0, slowdown_request_permille=0):
    """Short alias for pack_frame used by the vision loop."""
    return pack_frame(sequence, valid, confidence, lateral_error, slope_hint,
                      curve_phase, slowdown_request_permille)


def pack_v3(sequence, valid, confidence, lateral_error, slope_hint=0,
            curve_phase=0, slowdown_request_permille=0, profile_ack=0):
    """Short alias for the production V3 encoder."""
    return pack_frame_v3(
        sequence, valid, confidence, lateral_error, slope_hint, curve_phase,
        slowdown_request_permille, profile_ack)

