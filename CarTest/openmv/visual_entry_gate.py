"""Fail-closed visual entrance gate for the roundabout controller.

The existing ring tracker can recognise a roundabout before the vehicle has
reached the physical entrance.  This small state machine separates that
observation from permission to hand the route to the ring controller.  The
calibration file is deliberately required: image coordinates for the user's
track cannot be inferred from a source snapshot or a generic threshold.

Calibration is a parked, hand-push artifact.  A record contains per-side
intervals for the transition's far/near rows and the number of consecutive
frames required to accept the entrance.  Missing, malformed, or unready data
never permits ring control.
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
        build_encoder_entry_document as _build_encoder_entry_document,
        build_encoder_entry_record as _build_encoder_entry_record,
        validate_encoder_entry_record as _validate_encoder_entry_record,
        validate_encoder_entry_document as _validate_encoder_entry_document,
        DEFAULT_CALIBRATION_PATH as _DEFAULT_ENCODER_CALIBRATION_PATH,
        UINT32_MAX as _UINT32_MAX)
except ImportError:
    _build_encoder_entry_document = None
    _build_encoder_entry_record = None
    _validate_encoder_entry_record = None
    _validate_encoder_entry_document = None
    _DEFAULT_ENCODER_CALIBRATION_PATH = "ring_encoder_entry_calibration.json"
    _UINT32_MAX = 0xFFFFFFFF


API_VERSION = 1
FORMAT_VERSION = 1
DEFAULT_CALIBRATION_PATH = "ring_visual_entry_calibration.json"
DEFAULT_ENCODER_CALIBRATION_PATH = _DEFAULT_ENCODER_CALIBRATION_PATH
# MARK snapshots are local SD-card evidence only.  They are never sent over
# the wireless sideband and are not part of the public firmware package.
DEFAULT_SNAPSHOT_DIR = "ring_entry_marks"
SIDE_LEFT = 0
SIDE_RIGHT = 1
VALID_SIDES = (SIDE_LEFT, SIDE_RIGHT)
STATE_WAIT = 0
STATE_READY = 1
STATE_ACTIVE = 2
STATE_REJECTED = 3
MARK_FIRST = 0
MARK_LOWER_OPENING = 1
MARK_UPPER_ENTRANCE = 2
MARK_COUNT = 3
ENTRY_Y_TOLERANCE = 3
DEFAULT_CANDIDATE_TIMEOUT_MS = 4000


def _safe_state_id(ring_info):
    try:
        return int(ring_info.get("state_id", 0))
    except (TypeError, ValueError, AttributeError):
        return 0


def _crc32(text):
    raw = text.encode("utf-8") if isinstance(text, str) else text
    if _binascii is not None:
        try:
            return int(_binascii.crc32(raw)) & 0xFFFFFFFF
        except Exception:
            pass
    value = 0xFFFFFFFF
    for byte in raw:
        value ^= int(byte)
        for _ in range(8):
            value = ((value >> 1) ^ 0xEDB88320) if (value & 1) else (value >> 1)
    return value ^ 0xFFFFFFFF


def _record_crc(side, record):
    profiles = ",".join(str(value) for value in record["profile_ids"])
    body = "%s|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d" % (
        "VISUAL_ENTRY_V1", side, profiles,
        record["far_y_min"], record["far_y_max"],
        record["near_y_min"], record["near_y_max"],
        record["required_frames"], record["marker_span"],
        record["first_far_y"], record["first_near_y"],
        record["lower_far_y"], record["lower_near_y"],
        record["upper_far_y"], record["upper_near_y"])
    return _crc32(body)


def _integer(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer" % name)
    if minimum is not None and value < minimum:
        raise ValueError("%s is below its minimum" % name)
    if maximum is not None and value > maximum:
        raise ValueError("%s is above its maximum" % name)
    return int(value)


def _side_record(source, side, require_crc=True):
    if not isinstance(source, dict):
        raise ValueError("side %s calibration is missing" % side)
    result = {}
    profile_ids = source.get("profile_ids", (2, 3))
    if not isinstance(profile_ids, (list, tuple)) or not profile_ids:
        raise ValueError("%s profile_ids is missing" % side)
    result["profile_ids"] = []
    for profile in profile_ids:
        result["profile_ids"].append(_integer(
            profile, "%s.profile_id" % side, 1, 3))
    for name in ("far_y_min", "far_y_max", "near_y_min", "near_y_max",
                 "first_far_y", "first_near_y", "lower_far_y",
                 "lower_near_y", "upper_far_y", "upper_near_y"):
        result[name] = _integer(source.get(name), "%s.%s" % (side, name),
                                0, 119)
    if result["far_y_min"] > result["far_y_max"]:
        raise ValueError("%s far_y interval is reversed" % side)
    if result["near_y_min"] > result["near_y_max"]:
        raise ValueError("%s near_y interval is reversed" % side)
    if result["far_y_max"] >= result["near_y_min"]:
        raise ValueError("%s transition intervals must be ordered" % side)
    result["required_frames"] = _integer(
        source.get("required_frames", 3), "%s.required_frames" % side, 1, 8)
    result["marker_span"] = _integer(
        source.get("marker_span"), "%s.marker_span" % side, 0, 119)
    if require_crc:
        crc = _integer(source.get("crc32"), "%s.crc32" % side, 0, 0xFFFFFFFF)
        if crc != _record_crc(side, result):
            raise ValueError("%s CRC does not match schema/profile/direction" % side)
    return result


class VisualEntryCalibration(object):
    """Validated persisted entrance geometry; invalid files remain unready."""

    def __init__(self, ready=False, generation=0, sides=None, reason="MISSING",
                 encoder_entry=None):
        self.generation = int(generation) if isinstance(generation, int) else 0
        self.sides = {}
        if isinstance(sides, dict):
            for side, source in sides.items():
                if not isinstance(source, dict):
                    continue
                record = dict(source)
                record.setdefault("profile_ids", [2, 3])
                self.sides[int(side)] = record
        self.ready = bool(ready and self.sides)
        self.reason = str(reason)
        self.encoder_entry = (dict(encoder_entry)
                              if isinstance(encoder_entry, dict) else None)

    @classmethod
    def load(cls, path):
        if not path or _os is None:
            return cls(reason="MISSING")
        candidates = [path, path + ".previous", path + ".previous.old"]
        failure = "MISSING"
        for candidate in candidates:
            try:
                stream = open(candidate, "r")
                try:
                    data = _json.loads(stream.read())
                finally:
                    stream.close()
                if not isinstance(data, dict) or data.get("format") != FORMAT_VERSION or \
                        data.get("schema") != "VISUAL_ENTRY_V1":
                    failure = "BAD_FORMAT"
                    continue
                sides = {}
                for key, side in (("left", SIDE_LEFT), ("right", SIDE_RIGHT)):
                    if data.get(key) is None:
                        continue
                    try:
                        sides[side] = _side_record(data.get(key), side)
                    except Exception as exc:
                        failure = "INVALID_%s:%s" % (key, exc)
                if not sides:
                    if not failure.startswith("INVALID_"):
                        failure = "NOT_READY"
                    continue
                generation = _integer(data.get("generation", 0), "generation", 0)
                encoder_entry = data.get("encoder_entry")
                if isinstance(encoder_entry, dict):
                    if _validate_encoder_entry_document is None:
                        raise ValueError("shared encoder schema unavailable")
                    encoder_entry = _validate_encoder_entry_document(
                        encoder_entry)
                else:
                    encoder_entry = None
                return cls(True, generation, sides,
                           "READY_PREVIOUS" if candidate != path else "READY",
                           encoder_entry)
            except Exception as exc:
                if ((candidate == path and not failure.startswith("INVALID_")) or
                        failure in ("MISSING", "BAD_FORMAT", "NOT_READY")):
                    failure = "INVALID:%s" % exc
        return cls(reason=failure)

    def as_dict(self):
        def named(side):
            item = self.sides.get(side)
            if not isinstance(item, dict):
                return None
            result = dict(item)
            result["crc32"] = _record_crc(side, result)
            return result
        result = {"format": FORMAT_VERSION, "schema": "VISUAL_ENTRY_V1",
                  "api": API_VERSION, "ready": bool(self.ready),
                  "generation": self.generation,
                  "left": named(SIDE_LEFT), "right": named(SIDE_RIGHT)}
        if isinstance(self.encoder_entry, dict):
            if _validate_encoder_entry_document is None:
                raise ValueError("shared encoder schema unavailable")
            result["encoder_entry"] = _validate_encoder_entry_document(
                self.encoder_entry)
        return result

    def save(self, path):
        """Atomically save a validated record and read it back."""
        if not path or not self.ready:
            raise ValueError("visual entry calibration is not ready")
        text = _json.dumps(self.as_dict())
        temporary = path + ".new"
        stream = open(temporary, "w")
        try:
            stream.write(text)
        finally:
            stream.close()
        checked = self.load(temporary)
        if not checked.ready:
            raise ValueError("visual entry calibration temporary readback failed")
        previous = path + ".previous"
        previous_stash = previous + ".old"
        moved_previous = False
        moved_old_main = False
        try:
            if _os is not None:
                # Keep the old fallback reachable throughout the commit.  A
                # second rename temporarily stashes an existing .previous so
                # that replacing it cannot create a power-loss gap.
                try:
                    _os.remove(previous_stash)
                except Exception:
                    pass
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
                if moved_previous:
                    try:
                        _os.remove(previous_stash)
                    except Exception:
                        pass
        except Exception:
            if _os is not None:
                try:
                    _os.remove(temporary)
                except Exception:
                    pass
                if moved_old_main:
                    try:
                        _os.remove(path)
                    except Exception:
                        pass
                    try:
                        _os.rename(previous, path)
                    except Exception:
                        pass
                if moved_previous:
                    try:
                        _os.remove(previous)
                    except Exception:
                        pass
                    try:
                        _os.rename(previous_stash, previous)
                    except Exception:
                        pass
            raise
        # Do not let a valid .previous file mask a failed replacement.  The
        # commit acknowledgement is only valid after the exact destination
        # contains the expected validated document.
        stream = open(path, "r")
        try:
            actual_data = _json.loads(stream.read())
        finally:
            stream.close()
        expected_data = self.as_dict()
        if actual_data != expected_data:
            raise ValueError("visual entry calibration destination mismatch")
        checked = self.load(path)
        if not checked.ready or checked.as_dict() != expected_data:
            raise ValueError("visual entry calibration readback mismatch")
        return checked

    def save_encoder(self, path):
        """Atomically save the independent encoder companion document."""
        if not path or not isinstance(self.encoder_entry, dict):
            raise ValueError("encoder entry calibration is not ready")
        if _validate_encoder_entry_document is None:
            raise ValueError("shared encoder schema unavailable")
        document = _validate_encoder_entry_document(self.encoder_entry)
        text = _json.dumps(document)
        temporary = path + ".new"
        stream = open(temporary, "w")
        try:
            stream.write(text)
        finally:
            stream.close()
        previous = path + ".previous"
        previous_stash = previous + ".old"
        moved_previous = False
        moved_old_main = False
        try:
            if _os is not None:
                try:
                    _os.remove(previous_stash)
                except Exception:
                    pass
                try:
                    _os.rename(previous, previous_stash)
                    moved_previous = True
                except Exception:
                    pass
                try:
                    _os.rename(path, previous)
                    moved_old_main = True
                except Exception:
                    pass
                _os.rename(temporary, path)
                if moved_previous:
                    try:
                        _os.remove(previous_stash)
                    except Exception:
                        pass
        except Exception:
            if _os is not None:
                try:
                    _os.remove(temporary)
                except Exception:
                    pass
                if moved_old_main:
                    try:
                        _os.remove(path)
                    except Exception:
                        pass
                    try:
                        _os.rename(previous, path)
                    except Exception:
                        pass
                if moved_previous:
                    try:
                        _os.remove(previous)
                    except Exception:
                        pass
                    try:
                        _os.rename(previous_stash, previous)
                    except Exception:
                        pass
            raise
        stream = open(path, "r")
        try:
            checked = _validate_encoder_entry_document(
                _json.loads(stream.read()))
        finally:
            stream.close()
        if checked != document:
            raise ValueError("encoder entry calibration destination mismatch")
        return checked


class VisualEntryCalibrationCollector(object):
    """Collect three parked hand-push markers and export one side record.

    The command path marks: FIRST_STABLE, LOWER_OPENING, and UPPER_ENTRANCE.
    A marker uses the current transition far/near rows; frames without those
    rows are rejected instead of inventing image coordinates.  The upper
    marker becomes the accepted interval, while the lower marker must be
    visibly distinct from it.
    """

    def __init__(self, profile_ids=(2, 3), snapshot_dir=DEFAULT_SNAPSHOT_DIR):
        self.profile_ids = tuple(int(value) for value in profile_ids)
        self.snapshot_dir = snapshot_dir
        self.markers = {}
        self.encoder_markers = {}
        # Every marker in one side must come from the same parked profile
        # session.  This prevents visual geometry and wheel positions from
        # being assembled across separate hand-push origins.
        self.origins = {}
        self.snapshots = {}
        self.reason = "EMPTY"

    def reset(self):
        self.markers.clear()
        self.encoder_markers.clear()
        self.origins.clear()
        self.snapshots.clear()
        self.reason = "RESET"

    @staticmethod
    def _image_dimension(image, name):
        getter = getattr(image, name, None)
        if not callable(getter):
            return -1
        try:
            value = int(getter())
        except Exception:
            return -1
        return value if value >= 0 else -1

    def _snapshot_paths(self, side, marker):
        prefix = "%s/side%d_mark%d" % (self.snapshot_dir, side, marker)
        return prefix + ".pgm", prefix + ".json"

    def _save_snapshot(self, side, marker, image, geometry, metadata=None):
        """Save the current grayscale frame and marker metadata locally.

        A MARK command used by the production camera path must provide an
        image.  If the OpenMV image object or its save API is unavailable we
        fail the marker instead of pretending that only far/near coordinates
        constitute a captured scene.  Host-side collector tests may omit the
        image by leaving ``require_snapshot`` false.
        """
        if image is None or _os is None:
            self.reason = "MARK_SNAPSHOT_UNAVAILABLE"
            return False
        save = getattr(image, "save", None)
        if not callable(save):
            self.reason = "MARK_SNAPSHOT_API_UNAVAILABLE"
            return False
        try:
            try:
                _os.mkdir(self.snapshot_dir)
            except Exception:
                # Existing directory is the normal path on OpenMV SD cards.
                pass
            image_path, metadata_path = self._snapshot_paths(side, marker)
            save(image_path)
            record = {
                "schema": "VISUAL_ENTRY_MARK_V1",
                "side": int(side), "marker": int(marker),
                "far_y": int(geometry[0]), "near_y": int(geometry[1]),
                "image_path": image_path,
                "width": self._image_dimension(image, "width"),
                "height": self._image_dimension(image, "height"),
            }
            if isinstance(metadata, dict):
                for key in ("frame_id", "profile_id", "session_token",
                            "mean_forward_wheel_millirev"):
                    if key in metadata:
                        try:
                            record[key] = int(metadata[key])
                        except (TypeError, ValueError):
                            pass
            temporary = metadata_path + ".new"
            stream = open(temporary, "w")
            try:
                stream.write(_json.dumps(record))
            finally:
                stream.close()
            if _os is not None:
                try:
                    _os.remove(metadata_path)
                except Exception:
                    pass
                _os.rename(temporary, metadata_path)
            self.snapshots[(int(side), int(marker))] = record
            return True
        except Exception:
            try:
                if _os is not None:
                    _os.remove(metadata_path)
            except Exception:
                pass
            self.reason = "MARK_SNAPSHOT_SAVE_FAILED"
            return False

    @staticmethod
    def _geometry(ring_info, marker=None):
        if not isinstance(ring_info, dict):
            return None
        # The confirmed upper entrance uses the entry-transition certificate.
        # During the first/lower hand-push markers the tracker may instead
        # expose the straight-cross geometry; accept that measured evidence
        # without manufacturing a coordinate.
        for far_key, near_key in (
                ("entry_transition_far_y", "entry_transition_near_y"),
                ("cross_far_y", "cross_near_y")):
            try:
                far_y = int(ring_info.get(far_key, -1))
                near_y = int(ring_info.get(near_key, -1))
            except (TypeError, ValueError):
                continue
            if 0 <= far_y < near_y <= 119:
                return far_y, near_y
        if marker == MARK_FIRST and ring_info.get("ring_first_stable") is True:
            try:
                far_y = int(ring_info.get("ring_stable_far_y", -1))
                near_y = int(ring_info.get("ring_stable_near_y", -1))
            except (TypeError, ValueError):
                return None
            if 0 <= far_y < near_y <= 119:
                return far_y, near_y
        return None

    @staticmethod
    def _origin(metadata):
        if not isinstance(metadata, dict):
            return None
        if "profile_id" not in metadata or "session_token" not in metadata:
            return None
        try:
            return (int(metadata["profile_id"]),
                    int(metadata["session_token"]))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _first_stable_ready(ring_info):
        return (isinstance(ring_info, dict) and
                ring_info.get("ring_first_stable") is True and
                ring_info.get("ring_stable_observation") is True and
                ring_info.get("parked_ring_confirmed") is True)

    def mark(self, side, marker, ring_info, far_y=None, near_y=None,
             image=None, require_snapshot=False, metadata=None,
             millirev=None, position_fresh=True):
        side = _integer(side, "side", 0, 1)
        marker = _integer(marker, "marker", 0, MARK_COUNT - 1)
        if require_snapshot and (not isinstance(ring_info, dict) or
                                 ring_info.get("side") != side):
            self.reason = "MARK_SIDE_MISMATCH"
            return False
        previous = self.markers.get(side, {})
        if marker > MARK_FIRST and (marker - 1) not in previous:
            self.reason = "MARK_ORDER_INVALID"
            return False
        if marker == MARK_FIRST and not self._first_stable_ready(ring_info):
            self.reason = "FIRST_STABLE_REQUIRED"
            return False
        geometry = self._geometry(ring_info, marker)
        if far_y is not None and near_y is not None:
            try:
                geometry = (int(far_y), int(near_y))
            except (TypeError, ValueError):
                geometry = None
            if geometry is not None and not (0 <= geometry[0] < geometry[1] <= 119):
                geometry = None
        if geometry is None:
            self.reason = "MARK_GEOMETRY_UNAVAILABLE"
            return False
        origin = self._origin(metadata)
        if side in self.origins and origin != self.origins[side]:
            self.reason = "MARK_ORIGIN_MISMATCH"
            return False
        if millirev is not None:
            try:
                millirev = _integer(millirev, "millirev", 0, _UINT32_MAX)
            except Exception:
                self.reason = "MARK_POSITION_INVALID"
                return False
            if not position_fresh:
                self.reason = "MARK_POSITION_STALE"
                return False
            if millirev == _UINT32_MAX:
                self.reason = "MARK_POSITION_SENTINEL"
                return False
        if require_snapshot and not self._save_snapshot(
                side, marker, image, geometry, metadata):
            return False
        if side not in self.origins:
            self.origins[side] = origin
        # Repeating an earlier marker invalidates the later samples from the
        # previous pass; they cannot silently survive a changed origin.
        for later in range(marker + 1, MARK_COUNT):
            self.markers.get(side, {}).pop(later, None)
            self.encoder_markers.get(side, {}).pop(later, None)
            self.snapshots.pop((side, later), None)
        self.markers.setdefault(side, {})[marker] = geometry
        if millirev is not None:
            self.encoder_markers.setdefault(side, {})[marker] = millirev
        self.reason = "MARK_%d_ACCEPTED" % marker
        return True

    def encoder_side_ready(self, side):
        values = self.encoder_markers.get(int(side), {})
        return all(marker in values for marker in range(MARK_COUNT))

    def build_encoder_side(self, side):
        side = _integer(side, "side", 0, 1)
        if not self.encoder_side_ready(side):
            self.reason = "THREE_ENCODER_MARKERS_REQUIRED"
            return None
        if _build_encoder_entry_record is None:
            self.reason = "SHARED_ENCODER_SCHEMA_UNAVAILABLE"
            return None
        try:
            record = _build_encoder_entry_record(
                self.encoder_markers[side], side=side)
        except Exception as exc:
            self.reason = "ENCODER_%s" % exc
            return None
        self.reason = "ENCODER_SIDE_READY"
        return record

    def build_encoder_document(self, generation=0, base=None, side=None,
                               require_markers=False):
        if _build_encoder_entry_document is None:
            self.reason = "SHARED_ENCODER_SCHEMA_UNAVAILABLE"
            return None
        sides = {}
        existing = getattr(base, "encoder_entry", base)
        if isinstance(existing, dict):
            for current, key in ((SIDE_LEFT, "left"), (SIDE_RIGHT, "right")):
                source = existing.get(key)
                if isinstance(source, dict):
                    try:
                        sides[current] = _validate_encoder_entry_record(
                            source, current)
                    except Exception:
                        pass
        selected = VALID_SIDES if side is None else (int(side),)
        for current in selected:
            if self.encoder_side_ready(current):
                record = self.build_encoder_side(current)
                if record is None:
                    return None
                sides[current] = record
            elif require_markers:
                self.reason = "THREE_ENCODER_MARKERS_REQUIRED"
                return None
        return _build_encoder_entry_document(int(generation), sides)

    def side_ready(self, side):
        values = self.markers.get(int(side), {})
        return all(marker in values for marker in range(MARK_COUNT))

    def build_side(self, side, required_frames=3):
        side = _integer(side, "side", 0, 1)
        if not self.side_ready(side):
            self.reason = "THREE_MARKERS_REQUIRED"
            return None
        first = self.markers[side][MARK_FIRST]
        lower = self.markers[side][MARK_LOWER_OPENING]
        upper = self.markers[side][MARK_UPPER_ENTRANCE]
        # The lower opening and the desired upper entrance must be separated;
        # otherwise this calibration cannot distinguish the two locations.
        span = max(abs(upper[0] - lower[0]), abs(upper[1] - lower[1]))
        if span < 4:
            self.reason = "LOWER_AND_UPPER_GEOMETRY_NOT_DISTINCT"
            return None
        if max(abs(first[0] - upper[0]), abs(first[1] - upper[1])) < 2:
            self.reason = "FIRST_AND_UPPER_GEOMETRY_NOT_DISTINCT"
            return None
        tolerance = ENTRY_Y_TOLERANCE
        # The upper entrance interval is the only positive admission window.
        # Keep both negative hand-push markers in the persisted record and
        # reject a submission when either marker's tolerance zone overlaps the
        # upper zone in both dimensions.
        def overlaps(a, b):
            return abs(int(a) - int(b)) <= tolerance * 2

        if ((overlaps(first[0], upper[0]) and overlaps(first[1], upper[1])) or
                (overlaps(lower[0], upper[0]) and
                 overlaps(lower[1], upper[1]))):
            self.reason = "UPPER_MARKER_EXCLUSION_OVERLAP"
            return None
        record = {"profile_ids": list(self.profile_ids),
                  "far_y_min": max(0, upper[0] - tolerance),
                  "far_y_max": min(119, upper[0] + tolerance),
                  "near_y_min": max(0, upper[1] - tolerance),
                  "near_y_max": min(119, upper[1] + tolerance),
                  "required_frames": _integer(required_frames,
                                                "required_frames", 1, 8),
                  "marker_span": span,
                  "first_far_y": first[0], "first_near_y": first[1],
                  "lower_far_y": lower[0], "lower_near_y": lower[1],
                  "upper_far_y": upper[0], "upper_near_y": upper[1]}
        if record["far_y_max"] >= record["near_y_min"]:
            self.reason = "UPPER_INTERVAL_ORDER_INVALID"
            return None
        self.reason = "SIDE_READY"
        return record

    def build_calibration(self, base=None, side=None, require_snapshots=False,
                          require_encoder=False):
        sides = dict(getattr(base, "sides", {}) or {})
        selected = VALID_SIDES if side is None else (int(side),)
        for current in selected:
            if require_snapshots and not all(
                    (current, marker) in self.snapshots
                    for marker in range(MARK_COUNT)):
                self.reason = "THREE_SNAPSHOTS_REQUIRED"
                return None
            record = self.build_side(current)
            if record is None:
                return None
            sides[current] = record
        generation = int(getattr(base, "generation", 0)) + 1
        encoder_entry = self.build_encoder_document(
            generation=generation, base=base, side=side,
            require_markers=require_encoder)
        if encoder_entry is None and require_encoder:
            return None
        return VisualEntryCalibration(
            True, generation, sides, "READY", encoder_entry)


class VisualEntryGate(object):
    """Gate ring route and gyro evidence until calibrated entrance geometry fits."""

    def __init__(self, calibration=None, required=True,
                 candidate_timeout_ms=DEFAULT_CANDIDATE_TIMEOUT_MS):
        self.calibration = calibration or VisualEntryCalibration()
        self.required = bool(required)
        try:
            candidate_timeout_ms = int(candidate_timeout_ms)
        except (TypeError, ValueError):
            candidate_timeout_ms = DEFAULT_CANDIDATE_TIMEOUT_MS
        if candidate_timeout_ms < 1:
            candidate_timeout_ms = DEFAULT_CANDIDATE_TIMEOUT_MS
        self.candidate_timeout_ms = candidate_timeout_ms
        self.state = STATE_WAIT
        self.side = -1
        self.match_frames = 0
        self.reason = self.calibration.reason if self.required else "OPTIONAL"
        self._identity = None
        self._active_identity = None
        self._skip_identity = None
        self._candidate_started_ms = None
        self._candidate_expired = False
        self._result = {"ready": False, "active": False, "state": STATE_WAIT,
                        "side": -1, "match_frames": 0,
                        "entry_evidence": False,
                        "candidate_elapsed_ms": 0,
                        "candidate_timeout_ms": self.candidate_timeout_ms,
                        "calibration_ready": bool(self.calibration.ready),
                        "reason": self.reason}

    def reset(self, reason="RESET"):
        self.state = STATE_WAIT
        self.side = -1
        self.match_frames = 0
        self.reason = str(reason)
        self._active_identity = None
        self._skip_identity = None
        self._candidate_started_ms = None
        self._candidate_expired = False

    @staticmethod
    def _candidate(ring_info):
        return (isinstance(ring_info, dict) and
                bool(ring_info.get("candidate")))

    @staticmethod
    def _elapsed_ms(now_ms, start_ms, ticks_diff=None):
        if now_ms is None or start_ms is None:
            return 0
        try:
            elapsed = (ticks_diff(now_ms, start_ms) if callable(ticks_diff)
                       else int(now_ms) - int(start_ms))
        except Exception:
            return 0
        return max(0, int(elapsed))

    @staticmethod
    def _inside(ring_info):
        if not isinstance(ring_info, dict):
            return False
        try:
            state_id = int(ring_info.get("state_id", -1))
        except (TypeError, ValueError):
            state_id = -1
        return state_id == 3 and bool(ring_info.get("ring_confirmed"))

    def _matches(self, ring_info, side, profile_id=0):
        record = self.calibration.sides.get(side)
        if not isinstance(record, dict):
            return False
        if profile_id and profile_id not in record.get("profile_ids", ()):
            return False
        try:
            far_y = int(ring_info.get("entry_transition_far_y", -1))
            near_y = int(ring_info.get("entry_transition_near_y", -1))
        except (TypeError, ValueError):
            return False
        return (bool(ring_info.get("entry_transition")) and
                record["far_y_min"] <= far_y <= record["far_y_max"] and
                record["near_y_min"] <= near_y <= record["near_y_max"])

    def update(self, ring_info, valid=True, profile_id=0, session_token=0,
               now_ms=None, ticks_diff=None, session_active=True):
        """Return a reusable status dictionary for the current camera frame."""
        result = self._result
        identity = (int(profile_id), int(session_token), bool(session_active))
        identity_changed = self._identity is not None and identity != self._identity
        if identity_changed:
            self.reset("IDENTITY_CHANGED")
        self._identity = identity
        if identity_changed:
            result["ready"] = False
            result["active"] = False
            result["state"] = self.state
            result["side"] = self.side
            result["match_frames"] = self.match_frames
            result["entry_evidence"] = False
            result["candidate_elapsed_ms"] = 0
            result["calibration_ready"] = bool(self.calibration.ready)
            result["reason"] = self.reason
            return result
        result["entry_evidence"] = False
        if not session_active:
            self.state = STATE_WAIT
            self.side = -1
            self.match_frames = 0
            self.reason = "SESSION_INACTIVE"
            self._active_identity = None
            self._candidate_started_ms = None
            self._candidate_expired = False
            result["ready"] = False
            result["active"] = False
            result["state"] = self.state
            result["side"] = self.side
            result["match_frames"] = self.match_frames
            result["candidate_elapsed_ms"] = 0
            result["calibration_ready"] = bool(self.calibration.ready)
            result["reason"] = self.reason
            return result
        candidate_present = self._candidate(ring_info)
        try:
            observed_side = int(ring_info.get("side", -1))
        except (TypeError, ValueError, AttributeError):
            observed_side = -1
        observed_record = self.calibration.sides.get(observed_side)
        active_side_valid = bool(
            observed_side == self.side and isinstance(observed_record, dict) and
            (not profile_id or profile_id in observed_record.get("profile_ids", ())))
        if (candidate_present and self._candidate_started_ms is None and
                self._skip_identity != identity and
                self.state != STATE_ACTIVE):
            self._candidate_started_ms = (0 if now_ms is None else now_ms)
            self._candidate_expired = False
            self.state = STATE_READY
            self.side = -1
            self.match_frames = 0
            self.reason = "ENTRY_CANDIDATE_WAIT"
        candidate_elapsed = self._elapsed_ms(
            now_ms, self._candidate_started_ms, ticks_diff)
        result["candidate_elapsed_ms"] = candidate_elapsed
        if not self.calibration.ready and self.required:
            self.state = STATE_WAIT
            self.side = -1
            self.match_frames = 0
            self.reason = self.calibration.reason
        elif (not candidate_present and self.state in (STATE_READY,
                                                        STATE_REJECTED) and
              isinstance(ring_info, dict) and
              not bool(ring_info.get("ring_confirmed")) and
              _safe_state_id(ring_info) in (0, 1)):
            # A timed-out candidate is latched until the tracker returns to a
            # genuine non-candidate observation. This prevents a late frame
            # from restarting the same physical approach indefinitely.
            self.reset("CANDIDATE_CLEARED")
            candidate_elapsed = 0
            result["candidate_elapsed_ms"] = 0
        elif (candidate_present and self._candidate_expired and
              self.state != STATE_ACTIVE):
            self.state = STATE_REJECTED
            self.match_frames = 0
            self.reason = "ENTRY_GATE_EXPIRED"
        elif (candidate_present and self.state != STATE_ACTIVE and
              self._candidate_started_ms is not None and
              candidate_elapsed >= self.candidate_timeout_ms):
            self._candidate_expired = True
            self._skip_identity = identity
            self.state = STATE_REJECTED
            self.match_frames = 0
            self.reason = "ENTRY_GATE_EXPIRED"
        elif (self.state == STATE_ACTIVE and isinstance(ring_info, dict) and
              not bool(ring_info.get("ring_confirmed")) and
              not bool(ring_info.get("candidate")) and
              _safe_state_id(ring_info) in (0, 1)):
            # The tracker has returned to OBSERVE/ENTRY_CONFIRM after the
            # previous ring.  Do not carry the old entrance lease into a new
            # roundabout in the same Follow session.
            self.reset("RING_OBSERVATION_RESET")
        elif (self.state == STATE_ACTIVE and
              (not isinstance(ring_info, dict) or candidate_present or
               bool(ring_info.get("ring_confirmed"))) and
              not active_side_valid):
            self.state = STATE_REJECTED
            self.side = -1
            self.match_frames = 0
            self._candidate_expired = True
            self._skip_identity = identity
            self.reason = "ENTRY_SIDE_CHANGED"
        elif not valid:
            if self.state == STATE_ACTIVE:
                self.state = STATE_REJECTED
                self.side = -1
                self.match_frames = 0
                self._candidate_expired = True
                self._skip_identity = identity
            self.match_frames = 0
            self.reason = "FRAME_INVALID"
        elif (not candidate_present or not self._inside(ring_info)):
            # Keep an accepted gate alive through the tracker exit states only
            # after an entrance was actually accepted. Before that, a stale
            # INSIDE lease cannot create an entry event.
            if self.state != STATE_ACTIVE:
                self.state = STATE_WAIT
                self.side = -1
                self.match_frames = 0
                self.reason = "WAITING_FOR_INSIDE"
        else:
            try:
                side = int(ring_info.get("side", -1))
            except (TypeError, ValueError):
                side = -1
            record = self.calibration.sides.get(side)
            side_ready = isinstance(record, dict) and (
                not profile_id or profile_id in record.get("profile_ids", ()))
            if side not in VALID_SIDES or not side_ready:
                if self.state == STATE_ACTIVE:
                    self.state = STATE_REJECTED
                    self.side = -1
                    self.match_frames = 0
                    self._candidate_expired = True
                    self._skip_identity = identity
                self.match_frames = 0
                self.reason = ("SIDE_INVALID" if side not in VALID_SIDES else
                               "SIDE_NOT_CALIBRATED")
            elif side != self.side:
                if self.state == STATE_ACTIVE:
                    self.state = STATE_REJECTED
                    self.side = -1
                    self.match_frames = 0
                    self._candidate_expired = True
                    self._skip_identity = identity
                    self.reason = "ENTRY_SIDE_CHANGED"
                else:
                    self.state = STATE_WAIT
                    self.side = side
                    self.match_frames = (1 if self._matches(
                        ring_info, side, profile_id) else 0)
                    needed = self.calibration.sides[side]["required_frames"]
                    if self.match_frames >= needed:
                        self.state = STATE_ACTIVE
                        self._active_identity = (int(profile_id),
                                                 int(session_token), side)
                        self.reason = "ENTRY_ACCEPTED"
                    else:
                        self.reason = "ENTRY_GEOMETRY_MATCH" if self.match_frames else \
                            "ENTRY_GEOMETRY_WAIT"
            elif self._matches(ring_info, side, profile_id):
                was_active = self.state == STATE_ACTIVE
                self.match_frames += 1
                needed = self.calibration.sides[side]["required_frames"]
                if self.match_frames >= needed:
                    self.state = STATE_ACTIVE
                    self._active_identity = (int(profile_id),
                                             int(session_token), side)
                    self.reason = "ENTRY_ACCEPTED"
                    result["entry_evidence"] = bool(was_active)
                else:
                    self.reason = "ENTRY_GEOMETRY_CONFIRMING"
            else:
                self.match_frames = 0
                self.reason = "ENTRY_GEOMETRY_WAIT"

        result["ready"] = bool(self.state == STATE_ACTIVE)
        result["active"] = bool(self.state == STATE_ACTIVE)
        result["state"] = self.state
        result["side"] = self.side
        result["match_frames"] = self.match_frames
        result["entry_evidence"] = bool(result["entry_evidence"])
        result["candidate_elapsed_ms"] = candidate_elapsed
        result["calibration_ready"] = bool(self.calibration.ready)
        result["reason"] = self.reason
        return result


__all__ = ("API_VERSION", "FORMAT_VERSION", "DEFAULT_CALIBRATION_PATH",
           "DEFAULT_SNAPSHOT_DIR", "DEFAULT_CANDIDATE_TIMEOUT_MS",
           "SIDE_LEFT", "SIDE_RIGHT",
           "STATE_WAIT", "STATE_READY", "STATE_ACTIVE", "STATE_REJECTED",
           "VisualEntryCalibration", "VisualEntryGate")
