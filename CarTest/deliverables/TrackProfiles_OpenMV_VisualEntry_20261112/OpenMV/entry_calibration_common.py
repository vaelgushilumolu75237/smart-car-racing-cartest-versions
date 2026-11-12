"""Shared hand-push encoder marker schema used by visual and encoder builds.

The visual camera records the wheel position as metadata beside each image;
the encoder build consumes the same three marker values.  This module keeps
the schema and CRC identical without making either runtime depend on the
other gate implementation.
"""

try:
    import binascii as _binascii
except ImportError:
    _binascii = None


SCHEMA = "ENCODER_ENTRY_V1"
MAX_THRESHOLD_MILLIREV = 60000
UINT32_MAX = 0xFFFFFFFF
DEFAULT_CALIBRATION_PATH = "ring_encoder_entry_calibration.json"
MARK_FIRST = 0
MARK_LOWER_OPENING = 1
MARK_UPPER_ENTRANCE = 2


def crc32(data):
    raw = data.encode("utf-8") if isinstance(data, str) else data
    if _binascii is not None:
        try:
            return int(_binascii.crc32(raw)) & 0xFFFFFFFF
        except Exception:
            pass
    value = 0xFFFFFFFF
    for byte in raw:
        value ^= int(byte)
        for _ in range(8):
            value = ((value >> 1) ^ 0xEDB88320) if value & 1 else value >> 1
    return value ^ 0xFFFFFFFF


def integer(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer" % name)
    if minimum is not None and value < minimum:
        raise ValueError("%s is below its minimum" % name)
    if maximum is not None and value > maximum:
        raise ValueError("%s is above its maximum" % name)
    return int(value)


def encoder_entry_crc(side, record):
    body = "%s|%d|%d|%d|%d|%d" % (
        SCHEMA, int(side), int(record["first_millirev"]),
        int(record["lower_millirev"]), int(record["upper_millirev"]),
        int(record["threshold_millirev"]))
    return crc32(body)


def validate_encoder_entry_record(source, side):
    if not isinstance(source, dict):
        raise ValueError("side record is missing")
    result = {}
    for name in ("first_millirev", "lower_millirev", "upper_millirev",
                 "threshold_millirev"):
        result[name] = integer(source.get(name), name, 0, UINT32_MAX)
        if result[name] == UINT32_MAX:
            raise ValueError("%s uses UINT32_MAX sentinel" % name)
    if not (result["first_millirev"] < result["lower_millirev"] <
            result["upper_millirev"]):
        raise ValueError("markers must increase first < lower < upper")
    expected = result["upper_millirev"] - result["first_millirev"]
    if expected != result["threshold_millirev"] or not (1 <= expected <=
                                                          MAX_THRESHOLD_MILLIREV):
        raise ValueError("threshold does not match first-to-upper distance")
    crc = integer(source.get("crc32"), "crc32", 0, 0xFFFFFFFF)
    if crc != encoder_entry_crc(side, result):
        raise ValueError("record CRC mismatch")
    result["crc32"] = crc
    return result


def _marker_value(source, marker):
    if isinstance(source, dict):
        value = source.get(marker)
        if value is None:
            value = source.get(str(marker))
        if isinstance(value, dict):
            value = value.get("mean_forward_wheel_millirev")
        return value
    if isinstance(source, (list, tuple)) and 0 <= marker < len(source):
        value = source[marker]
        if isinstance(value, dict):
            value = value.get("mean_forward_wheel_millirev")
        return value
    return None


def build_encoder_entry_record(markers, side=0):
    values = []
    for marker in (MARK_FIRST, MARK_LOWER_OPENING, MARK_UPPER_ENTRANCE):
        value = _marker_value(markers, marker)
        values.append(integer(value, "marker_%d_millirev" % marker,
                              0, UINT32_MAX))
        if values[-1] == UINT32_MAX:
            raise ValueError("marker_%d_millirev uses UINT32_MAX sentinel" %
                             marker)
    first, lower, upper = values
    if not (first < lower < upper):
        raise ValueError("markers must increase first < lower < upper")
    threshold = upper - first
    if not (1 <= threshold <= MAX_THRESHOLD_MILLIREV):
        raise ValueError("threshold is outside the supported range")
    result = {"first_millirev": first, "lower_millirev": lower,
              "upper_millirev": upper, "threshold_millirev": threshold}
    result["crc32"] = encoder_entry_crc(side, result)
    return result


def build_encoder_entry_document(generation, sides):
    result = {"format": 1, "schema": SCHEMA, "api": 1,
              "generation": integer(generation, "generation", 0),
              "ready": bool(sides), "left": None, "right": None}
    for side, key in ((0, "left"), (1, "right")):
        source = sides.get(side) if isinstance(sides, dict) else None
        if source is not None:
            checked = dict(source)
            checked["crc32"] = encoder_entry_crc(side, checked)
            result[key] = validate_encoder_entry_record(checked, side)
    return result


def validate_encoder_entry_document(source):
    """Validate and normalize one persisted encoder calibration document."""
    if not isinstance(source, dict):
        raise ValueError("encoder entry document is missing")
    if (source.get("format") != 1 or source.get("schema") != SCHEMA or
            source.get("api", 1) != 1):
        raise ValueError("encoder entry document format mismatch")
    generation = integer(source.get("generation", 0), "generation", 0)
    sides = {}
    for key, side in (("left", 0), ("right", 1)):
        record = source.get(key)
        if record is not None:
            sides[side] = validate_encoder_entry_record(record, side)
    ready = bool(source.get("ready"))
    if ready != bool(sides):
        raise ValueError("encoder entry ready flag mismatch")
    return build_encoder_entry_document(generation, sides)


__all__ = ("SCHEMA", "MAX_THRESHOLD_MILLIREV", "UINT32_MAX",
           "DEFAULT_CALIBRATION_PATH", "MARK_FIRST",
           "MARK_LOWER_OPENING", "MARK_UPPER_ENTRANCE", "encoder_entry_crc",
           "validate_encoder_entry_record", "build_encoder_entry_record",
           "build_encoder_entry_document", "validate_encoder_entry_document")
