"""Bind a new private width map to the K230 camera and mounting geometry.

The H750 JSON remains in private-baseline and is never relabelled as K230.
Both collection criteria and parked width validation remain the r24 rules.
"""
import ring_width_calibration as original

_base_type = original.RingWidthCalibration
_base_crc = original._crc_payload
_binding = None


def bind(sensor_type, binding):
    global _binding
    if not isinstance(sensor_type, int) or not binding:
        raise ValueError("Cannot bind unknown K230 sensor")
    _binding = binding
    original.SENSOR_ID = sensor_type
    original.FORMAT_VERSION = 3
    original._crc_payload = _crc_payload
    original.RingWidthCalibration = BoundCalibration


def _crc_payload(data):
    return _base_crc(data) + b"|k230-binding|" + data["camera_binding"].encode("utf-8")


class BoundCalibration(_base_type):
    def _payload(self):
        data = super()._payload()
        data["camera_binding"] = _binding
        return data

    @classmethod
    def load(cls, path):
        try:
            with open(path, "r") as source:
                data = original._json.loads(source.read())
        except OSError:
            return _base_type.load(path, cls())
        except Exception:
            result = cls()
            result._set_status(False, "INVALID_FILE")
            return result
        if (data.get("format_version") != 3 or not _binding or
                data.get("camera_binding") != _binding):
            result = cls()
            result._set_status(False, "GEOMETRY_MISMATCH")
            return result
        # CanMV v1.4.3 does not bind this inherited classmethod through super.
        # Supply the subclass instance explicitly and retain its bound payload.
        return _base_type.load(path, cls())

