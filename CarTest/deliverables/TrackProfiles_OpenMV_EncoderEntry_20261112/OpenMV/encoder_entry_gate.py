"""Encoder-distance roundabout entrance calibration and runtime gate.

The H743 sends a monotonic forward-wheel position in the reverse profile
frame.  A parked hand-push records three positions per roundabout direction:
the first stable observation, the lower opening, and the intended upper
entrance.  Runtime control stays on the ordinary route until the current
observation has advanced by the committed first-to-upper distance.
"""

try:
    import ujson as _json
except ImportError:
    import json as _json

try:
    import os as _os
except ImportError:
    _os = None

try:
    import binascii as _binascii
except ImportError:
    _binascii = None

try:
    from entry_calibration_common import (
        SCHEMA as _COMMON_SCHEMA,
        MAX_THRESHOLD_MILLIREV as _COMMON_MAX_THRESHOLD,
        MARK_FIRST as _COMMON_MARK_FIRST,
        MARK_LOWER_OPENING as _COMMON_MARK_LOWER_OPENING,
        MARK_UPPER_ENTRANCE as _COMMON_MARK_UPPER_ENTRANCE,
        encoder_entry_crc as _common_encoder_entry_crc,
        validate_encoder_entry_record as _common_validate_record)
except ImportError:
    _COMMON_SCHEMA = None
    _COMMON_MAX_THRESHOLD = None
    _COMMON_MARK_FIRST = None
    _COMMON_MARK_LOWER_OPENING = None
    _COMMON_MARK_UPPER_ENTRANCE = None
    _common_encoder_entry_crc = None
    _common_validate_record = None


API_VERSION = 1
FORMAT_VERSION = 1
DEFAULT_CALIBRATION_PATH = "ring_encoder_entry_calibration.json"
SCHEMA = (_COMMON_SCHEMA if _COMMON_SCHEMA is not None else
          "ENCODER_ENTRY_V1")
SIDE_LEFT = 0
SIDE_RIGHT = 1
VALID_SIDES = (SIDE_LEFT, SIDE_RIGHT)
MARK_FIRST = (_COMMON_MARK_FIRST if _COMMON_MARK_FIRST is not None else 0)
MARK_LOWER_OPENING = (_COMMON_MARK_LOWER_OPENING
                      if _COMMON_MARK_LOWER_OPENING is not None else 1)
MARK_UPPER_ENTRANCE = (_COMMON_MARK_UPPER_ENTRANCE
                       if _COMMON_MARK_UPPER_ENTRANCE is not None else 2)
MARK_COUNT = 3
MAX_THRESHOLD_MILLIREV = (_COMMON_MAX_THRESHOLD
                           if _COMMON_MAX_THRESHOLD is not None else 60000)
UINT32_MAX = 0xFFFFFFFF

# Runtime safety bounds. These are finite software guards, not vehicle
# geometry calibration values. A committed marker distance remains the only
# positive admission threshold.
DEFAULT_MAX_CANDIDATE_WAIT_FRAMES = 300
DEFAULT_MAX_ENTRY_OVERSHOOT_MILLIREV = 4000

# Kept here as well as in the wire-side module so the main-loop calibration
# service can validate a request without coupling persistence to framing.
ENTRY_CAL_RESET = 0
ENTRY_CAL_MARK = 1
ENTRY_CAL_COMMIT = 2

STATE_WAIT = 0
STATE_LEASE = 1
STATE_READY = 2
STATE_ACTIVE = 3


def _crc32(data):
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


def _integer(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer" % name)
    if minimum is not None and value < minimum:
        raise ValueError("%s is below its minimum" % name)
    if maximum is not None and value > maximum:
        raise ValueError("%s is above its maximum" % name)
    return int(value)


def _record_crc(side, record):
    if callable(_common_encoder_entry_crc):
        return int(_common_encoder_entry_crc(side, record)) & 0xFFFFFFFF
    body = "%s|%d|%d|%d|%d|%d" % (
        SCHEMA, int(side), int(record["first_millirev"]),
        int(record["lower_millirev"]), int(record["upper_millirev"]),
        int(record["threshold_millirev"]))
    return _crc32(body)


def _checked_record(source, side):
    side = _integer(side, "side", 0, 1)
    if callable(_common_validate_record):
        return _common_validate_record(source, side)
    if not isinstance(source, dict):
        raise ValueError("side record is missing")
    result = {}
    for name in ("first_millirev", "lower_millirev", "upper_millirev",
                 "threshold_millirev"):
        result[name] = _integer(source.get(name), name, 0, 0xFFFFFFFF)
    if not (result["first_millirev"] < result["lower_millirev"] <
            result["upper_millirev"]):
        raise ValueError("markers must increase first < lower < upper")
    expected = result["upper_millirev"] - result["first_millirev"]
    if expected != result["threshold_millirev"] or not (1 <= expected <=
                                                          MAX_THRESHOLD_MILLIREV):
        raise ValueError("threshold does not match first-to-upper distance")
    crc = _integer(source.get("crc32"), "crc32", 0, 0xFFFFFFFF)
    if crc != _record_crc(side, result):
        raise ValueError("record CRC mismatch")
    result["crc32"] = crc
    return result


class EncoderEntryCalibration(object):
    """Validated per-side persisted encoder entrance records."""

    def __init__(self, generation=0, sides=None, reason="MISSING"):
        self.generation = int(generation) if isinstance(generation, int) else 0
        self.sides = {}
        if isinstance(sides, dict):
            for side, source in sides.items():
                try:
                    side = int(side)
                    if side not in VALID_SIDES:
                        continue
                    self.sides[side] = _checked_record(source, side)
                except Exception:
                    continue
        self.reason = str(reason)

    @property
    def ready(self):
        return bool(self.sides)

    @classmethod
    def load(cls, path):
        if not path:
            return cls(reason="MISSING")
        candidates = [path, path + ".previous"]
        failure = "MISSING"
        for candidate in candidates:
            try:
                stream = open(candidate, "r")
                try:
                    data = _json.loads(stream.read())
                finally:
                    stream.close()
                if not isinstance(data, dict) or data.get("format") != FORMAT_VERSION or \
                        data.get("schema") != SCHEMA:
                    failure = "BAD_FORMAT"
                    continue
                generation = _integer(data.get("generation", 0), "generation", 0)
                sides = {}
                for key, side in (("left", SIDE_LEFT), ("right", SIDE_RIGHT)):
                    if data.get(key) is None:
                        continue
                    try:
                        sides[side] = _checked_record(data.get(key), side)
                    except Exception as exc:
                        failure = "INVALID_%s:%s" % (key, exc)
                if not sides:
                    failure = "NOT_READY"
                    continue
                return cls(generation, sides,
                           "READY_PREVIOUS" if candidate != path else "READY")
            except Exception as exc:
                # A valid main document that is merely NOT_READY should keep
                # that reason when its optional .previous fallback is absent.
                # Likewise, preserve the primary-file failure instead of
                # replacing it with a missing fallback error.
                if candidate == path or failure == "MISSING":
                    failure = "INVALID:%s" % exc
        return cls(reason=failure)

    def as_dict(self):
        def named(side):
            record = self.sides.get(side)
            if record is None:
                return None
            result = dict(record)
            result["crc32"] = _record_crc(side, result)
            return result
        return {"format": FORMAT_VERSION, "schema": SCHEMA,
                "api": API_VERSION, "generation": self.generation,
                "ready": self.ready, "left": named(SIDE_LEFT),
                "right": named(SIDE_RIGHT)}

    def save(self, path):
        if not path:
            raise ValueError("calibration path is empty")
        expected = self.as_dict()
        text = _json.dumps(expected)
        temporary = path + ".new"
        previous = path + ".previous"
        previous_stash = previous + ".old"

        def read_exact(candidate):
            stream = open(candidate, "r")
            try:
                return _json.loads(stream.read())
            finally:
                stream.close()

        def remove(candidate):
            if _os is None:
                return
            try:
                _os.remove(candidate)
            except Exception:
                pass

        try:
            stream = open(temporary, "w")
            try:
                stream.write(text)
                flush = getattr(stream, "flush", None)
                if callable(flush):
                    flush()
            finally:
                stream.close()
        except Exception:
            remove(temporary)
            raise
        try:
            if read_exact(temporary) != expected:
                raise ValueError("temporary calibration readback mismatch")
            # A ready document must pass the full schema/CRC validator before
            # it can replace the live file.  Empty reset documents are valid
            # serialized state even though load() reports NOT_READY.
            if expected.get("ready"):
                checked = EncoderEntryCalibration.load(temporary)
                if (not checked.ready or checked.as_dict() != expected):
                    raise ValueError("temporary calibration validation failed")
        except Exception:
            remove(temporary)
            raise

        if _os is None:
            remove(temporary)
            raise ValueError("atomic calibration rename unavailable")

        moved_previous = False
        moved_old_main = False
        try:
            # Keep the old fallback reachable during the rotation.  The stash
            # prevents replacing an existing .previous from creating a gap.
            remove(previous_stash)
            try:
                _os.rename(previous, previous_stash)
                moved_previous = True
            except Exception:
                moved_previous = False
            try:
                _os.rename(path, previous)
                moved_old_main = True
            except Exception:
                moved_old_main = False
            _os.rename(temporary, path)
            if read_exact(path) != expected:
                raise ValueError("committed calibration readback mismatch")
            if moved_previous:
                remove(previous_stash)
        except Exception:
            remove(temporary)
            if moved_old_main:
                remove(path)
                try:
                    _os.rename(previous, path)
                except Exception:
                    pass
            if moved_previous:
                remove(previous)
                try:
                    _os.rename(previous_stash, previous)
                except Exception:
                    pass
            raise

        checked = EncoderEntryCalibration.load(path)
        if checked.as_dict() != expected:
            # This should be unreachable after the exact raw readback above;
            # retain the explicit check so a fallback cannot be acknowledged.
            raise ValueError("saved calibration did not validate exactly")
        self.generation = checked.generation
        self.sides = checked.sides
        self.reason = checked.reason
        return checked

    def clear(self):
        self.generation = 0
        self.sides = {}
        self.reason = "CALIBRATION_RESET"

    def replace(self, side, record):
        side = _integer(side, "side", 0, 1)
        checked = _checked_record(record, side)
        sides = dict(self.sides)
        sides[side] = checked
        return EncoderEntryCalibration(self.generation + 1, sides, "READY")


class EncoderEntryCollector(object):
    """Collect FIRST/LOWER/UPPER millirev marks before COMMIT."""

    def __init__(self):
        self.markers = {}
        self.reason = "IDLE"

    def reset(self):
        self.markers = {}
        self.reason = "CALIBRATION_RESET"

    def mark(self, side, marker, millirev):
        side = _integer(side, "side", 0, 1)
        marker = _integer(marker, "marker", 0, MARK_COUNT - 1)
        millirev = _integer(millirev, "millirev", 0, 0xFFFFFFFF)
        if millirev == UINT32_MAX:
            self.reason = "POSITION_SENTINEL"
            return None
        row = self.markers.setdefault(side, {})
        row[marker] = millirev
        self.reason = "MARKED"
        return millirev

    def build(self, side):
        side = _integer(side, "side", 0, 1)
        row = self.markers.get(side, {})
        if any(marker not in row for marker in range(MARK_COUNT)):
            self.reason = "THREE_MARKERS_REQUIRED"
            return None
        first = row[MARK_FIRST]
        lower = row[MARK_LOWER_OPENING]
        upper = row[MARK_UPPER_ENTRANCE]
        threshold = upper - first
        if not (first < lower < upper):
            self.reason = "MARKERS_NOT_INCREASING"
            return None
        if not (1 <= threshold <= MAX_THRESHOLD_MILLIREV):
            self.reason = "THRESHOLD_OUT_OF_RANGE"
            return None
        result = {"first_millirev": first, "lower_millirev": lower,
                  "upper_millirev": upper,
                  "threshold_millirev": threshold}
        result["crc32"] = _record_crc(side, result)
        self.reason = "SIDE_READY"
        return result


class EncoderEntryGate(object):
    """Delay ring route/gyro authority until the committed distance is met."""

    def __init__(self, calibration=None, required=True,
                 max_candidate_wait_frames=DEFAULT_MAX_CANDIDATE_WAIT_FRAMES,
                 max_overshoot_millirev=
                 DEFAULT_MAX_ENTRY_OVERSHOOT_MILLIREV):
        if (isinstance(max_candidate_wait_frames, bool) or
                not isinstance(max_candidate_wait_frames, int) or
                max_candidate_wait_frames < 1):
            raise ValueError("max_candidate_wait_frames must be positive")
        if (isinstance(max_overshoot_millirev, bool) or
                not isinstance(max_overshoot_millirev, int) or
                max_overshoot_millirev < 0):
            raise ValueError("max_overshoot_millirev must be non-negative")
        self.calibration = calibration or EncoderEntryCalibration()
        self.required = bool(required)
        self.max_candidate_wait_frames = int(max_candidate_wait_frames)
        self.max_overshoot_millirev = int(max_overshoot_millirev)
        self.state = STATE_WAIT
        self.side = -1
        self.start_millirev = None
        self.progress_millirev = 0
        self.threshold_millirev = 0
        self.candidate_wait_frames = 0
        self.reason = self.calibration.reason if self.required else "OPTIONAL"
        self._identity = None
        self._skip_identity = None
        self._result = {"ready": False, "active": False,
                        "state": STATE_WAIT, "side": -1,
                        "entry_evidence": False,
                        "calibration_ready": bool(self.calibration.ready),
                        "threshold_millirev": 0, "progress_millirev": 0,
                        "start_millirev": 0, "candidate_wait_frames": 0,
                        "max_candidate_wait_frames":
                        self.max_candidate_wait_frames,
                        "reason": self.reason}

    def reset(self, reason="RESET"):
        self.state = STATE_WAIT
        self.side = -1
        self.start_millirev = None
        self.progress_millirev = 0
        self.threshold_millirev = 0
        self.candidate_wait_frames = 0
        self.reason = str(reason)
        self._skip_identity = None

    def _reject_candidate(self, identity, reason):
        self.state = STATE_WAIT
        self.side = -1
        self.start_millirev = None
        self.progress_millirev = 0
        self.threshold_millirev = 0
        self.candidate_wait_frames = 0
        self.reason = str(reason)
        self._skip_identity = identity
        return self._sync(False)

    @staticmethod
    def _state_id(ring_info):
        if not isinstance(ring_info, dict):
            return -1
        try:
            return int(ring_info.get("state_id", -1))
        except (TypeError, ValueError):
            return -1

    @classmethod
    def _inside(cls, ring_info):
        return (cls._state_id(ring_info) == 3 and
                isinstance(ring_info, dict) and
                bool(ring_info.get("ring_confirmed")))

    @classmethod
    def _candidate_present(cls, ring_info):
        if not isinstance(ring_info, dict):
            return False
        return bool(ring_info.get("candidate") or
                    ring_info.get("ring_confirmed") or
                    cls._state_id(ring_info) in (1, 2, 3, 4, 5))

    @classmethod
    def _candidate_cleared(cls, ring_info):
        if not isinstance(ring_info, dict):
            return False
        return (not bool(ring_info.get("candidate")) and
                not bool(ring_info.get("ring_confirmed")) and
                cls._state_id(ring_info) in (0, 1))

    def _sync(self, evidence=False):
        result = self._result
        result["ready"] = bool(self.state in (STATE_READY, STATE_ACTIVE))
        result["active"] = bool(self.state == STATE_ACTIVE)
        result["state"] = self.state
        result["side"] = self.side
        result["entry_evidence"] = bool(
            evidence and self.state == STATE_ACTIVE)
        result["calibration_ready"] = bool(self.calibration.ready)
        result["threshold_millirev"] = self.threshold_millirev
        result["progress_millirev"] = self.progress_millirev
        result["start_millirev"] = (self.start_millirev
                                     if self.start_millirev is not None else 0)
        result["candidate_wait_frames"] = self.candidate_wait_frames
        result["max_candidate_wait_frames"] = \
            self.max_candidate_wait_frames
        result["reason"] = self.reason
        return result

    def update(self, ring_info, valid=True, profile_id=0, session_token=0,
               millirev=0, position_fresh=True, session_active=None):
        if session_active is None:
            session_active = bool(session_token)
        identity = (int(profile_id), int(session_token),
                    bool(session_active))
        if self._identity is not None and identity != self._identity:
            self.reset("IDENTITY_CHANGED")
        self._identity = identity
        inside = self._inside(ring_info)
        candidate_present = self._candidate_present(ring_info)

        if self._skip_identity == identity:
            if self._candidate_cleared(ring_info):
                self.reset("CANDIDATE_CLEARED")
            else:
                return self._sync(False)

        if not session_active:
            if self.state != STATE_WAIT:
                self.reset("SESSION_INACTIVE")
            self.reason = "SESSION_INACTIVE"
            return self._sync(False)

        # Optional mode is only valid for explicit host fixtures. Production
        # main always supplies required=True and a validated record.
        if not self.required and not self.calibration.ready:
            self.state = STATE_READY
            self.side = -1
            self.start_millirev = None
            self.progress_millirev = 0
            self.threshold_millirev = 0
            self.reason = "OPTIONAL"
            return self._sync(False)

        if not self.calibration.ready:
            if candidate_present:
                return self._reject_candidate(identity,
                                              self.calibration.reason)
            self.reason = self.calibration.reason
            return self._sync(False)

        if self._candidate_cleared(ring_info):
            if self.state != STATE_WAIT:
                self.reset("CANDIDATE_CLEARED")
            else:
                self.reason = "WAITING_FOR_RING"
            return self._sync(False)

        if self.state in (STATE_LEASE, STATE_READY) and candidate_present:
            self.candidate_wait_frames += 1
            if self.candidate_wait_frames > self.max_candidate_wait_frames:
                return self._reject_candidate(identity, "ENTRY_GATE_EXPIRED")

        if not valid or not position_fresh:
            reason = "FRAME_INVALID" if not valid else "POSITION_STALE"
            if candidate_present or self.state != STATE_WAIT:
                return self._reject_candidate(identity, reason)
            self.reason = reason
            return self._sync(False)

        try:
            position = _integer(millirev, "millirev", 0, 0xFFFFFFFF)
        except Exception:
            if candidate_present or self.state != STATE_WAIT:
                return self._reject_candidate(identity, "POSITION_INVALID")
            self.reason = "POSITION_INVALID"
            return self._sync(False)
        if position == UINT32_MAX:
            if candidate_present or self.state != STATE_WAIT:
                return self._reject_candidate(identity, "POSITION_SENTINEL")
            self.reason = "POSITION_SENTINEL"
            return self._sync(False)

        if not inside:
            if self.state == STATE_ACTIVE:
                try:
                    observed_side = int(ring_info.get("side", -1))
                except (TypeError, ValueError, AttributeError):
                    observed_side = -1
                if observed_side not in VALID_SIDES:
                    return self._reject_candidate(
                        identity, "SIDE_NOT_CALIBRATED")
                if observed_side != self.side:
                    return self._reject_candidate(identity, "SIDE_CHANGED")
                return self._reject_candidate(
                    identity, "ENTRY_EVIDENCE_REVOKED")
            if self.state == STATE_WAIT:
                self.reason = "WAITING_FOR_INSIDE"
            elif self.state == STATE_LEASE:
                self.reason = "RING_OBSERVED_WAITING_FOR_INSIDE"
            elif self.state == STATE_READY:
                self.reason = "ENTRY_DISTANCE_REACHED_WAITING_FOR_EVIDENCE"
            else:
                self.reason = "ENTRY_ACTIVE_WAITING_FOR_EVIDENCE"
            return self._sync(False)

        try:
            side = int(ring_info.get("side", -1))
        except (TypeError, ValueError, AttributeError):
            side = -1
        record = self.calibration.sides.get(side)
        if side not in VALID_SIDES or record is None:
            return self._reject_candidate(identity, "SIDE_NOT_CALIBRATED")
        threshold = int(record["threshold_millirev"])
        if self.side != -1 and self.side != side:
            return self._reject_candidate(identity, "SIDE_CHANGED")
        if self.start_millirev is None:
            if not (ring_info.get("ring_first_stable") is True and
                    ring_info.get("ring_stable_observation") is True):
                self.reason = "WAITING_FOR_STABLE_RING_ORIGIN"
                return self._sync(False)
            self.side = side
            self.start_millirev = position
            self.progress_millirev = 0
            self.threshold_millirev = threshold
            self.candidate_wait_frames = 1
            self.state = STATE_LEASE
            self.reason = "RING_OBSERVED"
            return self._sync(False)

        delta = position - self.start_millirev
        if delta < 0 or delta > 0x7FFFFFFF:
            return self._reject_candidate(identity, "POSITION_BACKWARD")
        self.progress_millirev = delta
        self.threshold_millirev = threshold
        # The overshoot bound protects the pre-admission lease from a stale
        # candidate.  Once ACTIVE, continued wheel travel is expected while
        # the camera owns the ring; keep that authority until the ring ends
        # or the profile/session identity becomes invalid.
        if (self.state != STATE_ACTIVE and
                delta > threshold + self.max_overshoot_millirev):
            return self._reject_candidate(identity, "ENTRY_DISTANCE_OVERSHOT")
        if delta >= threshold:
            if self.state == STATE_LEASE:
                self.state = STATE_READY
                self.reason = "ENTRY_DISTANCE_REACHED"
                return self._sync(False)
            self.state = STATE_ACTIVE
            self.reason = "ENTRY_EVIDENCE_CONFIRMED"
            return self._sync(True)
        self.state = STATE_LEASE
        self.reason = "WAITING_FOR_ENTRY_DISTANCE"
        return self._sync(False)


__all__ = ("API_VERSION", "FORMAT_VERSION", "DEFAULT_CALIBRATION_PATH",
           "DEFAULT_MAX_CANDIDATE_WAIT_FRAMES",
           "DEFAULT_MAX_ENTRY_OVERSHOOT_MILLIREV",
           "SIDE_LEFT", "SIDE_RIGHT", "VALID_SIDES", "MARK_FIRST",
           "MARK_LOWER_OPENING", "MARK_UPPER_ENTRANCE", "MARK_COUNT",
           "STATE_WAIT", "STATE_LEASE", "STATE_READY", "STATE_ACTIVE",
           "UINT32_MAX", "ENTRY_CAL_RESET", "ENTRY_CAL_MARK",
           "ENTRY_CAL_COMMIT",
           "EncoderEntryCalibration", "EncoderEntryCollector",
           "EncoderEntryGate")
