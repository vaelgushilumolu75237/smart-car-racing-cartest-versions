"""Bounded per-ring approach history, stop freeze, and CRC persistence.

The recorder retains a fixed prehistory window before each ring approach and
bounded posthistory for up to eight episodes. It does not print or touch the
filesystem from ``record_frame``. A caller freezes and saves only after a
fresh actual Follow-to-stopped transition, while camera power is still on.

Format 3 adds per-frame target side/center, weighted and sent error, and
route-admission reason. Format 2/3 files use 20 ms timestamp ticks in the packed frame records. Format 1
files remain readable with their original millisecond timestamp units. Both
formats use a 32-byte ``RNGD`` header followed by a bounded build-id field,
episode metadata, packed signed 16-bit frame fields, and the 13 cumulative
rejection counters. The header CRC-32 covers the payload.
"""

import os

try:
    import micropython
    DIAGNOSTIC_KERNEL_BACKEND = "micropython-viper"
except ImportError:  # CPython host tests
    DIAGNOSTIC_KERNEL_BACKEND = "cpython-shim"

    class _HostMicropython:
        @staticmethod
        def native(function):
            return function

        @staticmethod
        def viper(function):
            return function

    micropython = _HostMicropython()

    def ptr8(buffer):
        return buffer

    def ptr16(buffer):
        return buffer

    class _HostPtr32:
        def __init__(self, buffer):
            self.buffer = buffer

        def __getitem__(self, index):
            return self.buffer[index] & 0xFFFFFFFF

    def ptr32(buffer):
        return _HostPtr32(buffer)

try:
    import ustruct as _struct
except ImportError:  # CPython host tests
    import struct as _struct

try:
    import binascii as _binascii
except ImportError:  # pragma: no cover
    _binascii = None

from array import array


FORMAT_VERSION = 3
LEGACY_FORMAT_VERSION = 1
TIMESTAMP_TICK_MS = 20
MAGIC = b"RNGD"
HEADER_SIZE = 32
EPISODE_CAPACITY = 8
PRE_FRAMES = 64
POST_FRAMES = 192
EPISODE_FRAME_CAPACITY = PRE_FRAMES + POST_FRAMES
REJECT_COUNT = 13
LEGACY_BASE_FIELD_COUNT = 48
BASE_FIELD_COUNT = 53  # adds target side/center and actual control direction
FRAME_RECORD_SIZE = BASE_FIELD_COUNT * 2 + REJECT_COUNT * 2
EPISODE_META_FORMAT = "<HHHHHIHHHHH"
EPISODE_META_SIZE = _struct.calcsize(EPISODE_META_FORMAT)
BUILD_ID_BYTES = 63
MAX_FILE_BYTES = 294912
STOP_SNAPSHOT_FLAG = 1

# The OpenMV build used by the camera may omit ``array.typecode`` and the
# memoryview format string even though the producer still creates an
# ``array('i')``.  Keep the signed/unsigned distinction explicit: the Viper
# serializer is valid only for signed 32-bit counts.  Unsigned high-bit
# values must remain on the Python path, where they clamp to 65535 rather
# than being reinterpreted as negative signed values and clamped to zero.
_REJECT_ARRAY_INVALID = 0
_REJECT_ARRAY_SIGNED_I = 1
_REJECT_ARRAY_UNSIGNED_I = 2


def _format_reject_array_kind(value):
    """Return the kind encoded by a typecode/format, or zero if unknown."""
    if value in ("i", "@i", "=i", "<i", ">i"):
        return _REJECT_ARRAY_SIGNED_I
    if value in ("I", "@I", "=I", "<I", ">I"):
        return _REJECT_ARRAY_UNSIGNED_I
    return _REJECT_ARRAY_INVALID


def _repr_reject_array_kind(values):
    """Conservatively identify an array kind when metadata is absent.

    This inspects only the typecode prefix emitted by the array object's
    representation.  It never infers signedness from an element value.
    """
    try:
        text = repr(values)
    except Exception:
        return _REJECT_ARRAY_INVALID
    if (text.startswith("array('i'") or
            text.startswith('array("i"')):
        return _REJECT_ARRAY_SIGNED_I
    if (text.startswith("array('I'") or
            text.startswith('array("I"')):
        return _REJECT_ARRAY_UNSIGNED_I
    return _REJECT_ARRAY_INVALID


def _classify_reject_counts_array(values):
    """Validate a reject-count producer and classify signedness once.

    The returned kind is safe to cache for the identity and current length of
    ``values``.  A non-array, short buffer, non-4-byte buffer, unsupported
    typecode, or conflicting metadata returns INVALID and must use the
    original bounded Python serializer.
    """
    if not isinstance(values, array):
        return _REJECT_ARRAY_INVALID
    try:
        count = len(values)
    except (TypeError, ValueError):
        return _REJECT_ARRAY_INVALID
    if count < REJECT_COUNT:
        return _REJECT_ARRAY_INVALID
    try:
        view = memoryview(values)
    except (TypeError, ValueError, AttributeError):
        return _REJECT_ARRAY_INVALID
    itemsize = getattr(view, "itemsize", None)
    if itemsize is None:
        # OpenMV v5 can omit memoryview.itemsize.  Verify physical width
        # without interpreting any element or guessing its signedness.
        try:
            if len(bytes(view)) != count * 4:
                return _REJECT_ARRAY_INVALID
        except (TypeError, ValueError, MemoryError):
            return _REJECT_ARRAY_INVALID
    elif itemsize != 4:
        return _REJECT_ARRAY_INVALID

    typecode = getattr(values, "typecode", None)
    type_kind = (_format_reject_array_kind(typecode)
                 if typecode is not None else _REJECT_ARRAY_INVALID)
    if typecode is not None and type_kind == _REJECT_ARRAY_INVALID:
        return _REJECT_ARRAY_INVALID

    view_format = getattr(view, "format", None)
    format_kind = (_format_reject_array_kind(view_format)
                   if view_format is not None else _REJECT_ARRAY_INVALID)
    if view_format is not None and format_kind == _REJECT_ARRAY_INVALID:
        return _REJECT_ARRAY_INVALID
    if (type_kind != _REJECT_ARRAY_INVALID and
            format_kind != _REJECT_ARRAY_INVALID and
            type_kind != format_kind):
        return _REJECT_ARRAY_INVALID

    repr_kind = _repr_reject_array_kind(values)
    if (type_kind != _REJECT_ARRAY_INVALID and
            repr_kind != _REJECT_ARRAY_INVALID and
            type_kind != repr_kind):
        return _REJECT_ARRAY_INVALID
    if (format_kind != _REJECT_ARRAY_INVALID and
            repr_kind != _REJECT_ARRAY_INVALID and
            format_kind != repr_kind):
        return _REJECT_ARRAY_INVALID

    if type_kind != _REJECT_ARRAY_INVALID:
        return type_kind
    if format_kind != _REJECT_ARRAY_INVALID:
        return format_kind
    return repr_kind


@micropython.viper
def _serialize_i16_fields_viper(values_object, destination_object,
                                count: int):
    values = ptr16(values_object)
    destination = ptr8(destination_object)
    index = 0
    while index < count:
        value = values[index]
        offset = index << 1
        destination[offset] = value & 0xFF
        destination[offset + 1] = (value >> 8) & 0xFF
        index += 1


@micropython.viper
def _serialize_i32_rejects_viper(values_object, destination_object,
                                 destination_offset: int,
                                 count: int) -> int:
    values = ptr32(values_object)
    destination = ptr8(destination_object)
    total = 0
    index = 0
    while index < count:
        value = values[index]
        if value < 0 or value >> 31:
            value = 0
        elif value > 65535:
            value = 65535
        total += value
        offset = destination_offset + (index << 1)
        destination[offset] = value & 0xFF
        destination[offset + 1] = (value >> 8) & 0xFF
        index += 1
    return total


@micropython.viper
def _copy_record_viper(destination_object, destination_offset: int,
                       source_object, source_offset: int,
                       count: int):
    destination = ptr8(destination_object)
    source = ptr8(source_object)
    byte_index = 0
    while byte_index < count:
        destination[destination_offset + byte_index] = \
            source[source_offset + byte_index]
        byte_index += 1


_SOURCE_CODES = {
    "NONE": 0, "road": 1, "single_left": 2, "single_right": 3,
    "ring_left": 4, "ring_right": 5,
}
_SELECTION_REASONS = (
    "", "OK", "RESET", "NO_CONVEX_ARC", "OBSERVING",
    "TOPOLOGY_NOT_CLEAR", "RING_TARGET_UNSUPPORTED",
    "RING_WIDTH_CACHE_TARGET", "FRAME_BUDGET", "CONTROL_TIMEOUT",
    "PROFILE_TIMEOUT", "REVERSE_RESET", "SESSION_INACTIVE",
    "PROFILE_UNAVAILABLE", "NO_FROZEN_WIDTH_PROFILE",
    "RING_ENTRY_TRANSITION_TARGET",
)
_TRACK_STATES = (
    "OBSERVE", "ENTRY_CONFIRM", "ENTER", "INSIDE",
    "EXIT_CONFIRM", "EXIT_REPAIR", "REACQUIRE",
)
_TRACK_REASONS = (
    "", "RESET", "OBSERVING", "TOPOLOGY_NOT_CLEAR",
    "NO_FROZEN_WIDTH_PROFILE", "ONE_SIDE_LOST_CANDIDATE",
    "CONFIRMING_ONE_SIDE_LOSS", "WAITING_FOR_RING_ARC",
    "RING_INSIDE_EXIT_UNOBSERVED", "EXIT_EXPERIMENTAL_CONFIRM",
    "EXIT_EXPERIMENTAL_REOBSERVATION",
    "WAITING_FOR_NORMAL_ROAD_REACQUIRE", "SESSION_INACTIVE",
    "PROFILE_UNAVAILABLE", "FRAME_INVALID", "CACHE_INVALID",
    "RING_TARGET_UNSUPPORTED", "TEST_RING_ACTIVE",
    "GROWING_ARC_CONFIRMED", "RING_ARC_CONFIRMED",
    "ORDINARY_ROUTE_REACQUIRED", "COMMON_MOTION_NOT_RING",
    "ORDINARY_CORRIDOR_VETO",
)
_REJECT_REASONS = (
    "NONE", "INPUT_INVALID", "FRAME_INVALID", "SESSION_INACTIVE",
    "PROFILE_UNAVAILABLE", "CACHE_INVALID", "ROAD_ROWS_INVALID",
    "SINGLE_ROWS_STALE", "TOPOLOGY_AMBIGUOUS", "SIDE_SUPPORT",
    "RING_SEQUENCE", "CROSS_EVIDENCE", "STATE_TIMEOUT",
)
_ROUTE_ADMISSION_REASONS = (
    "NOT_RUN", "RING_NOT_ELIGIBLE", "RING_WIDTH_CACHE_TARGET",
    "RING_WIDTH_PROVENANCE_REJECTED", "RING_TARGET_ROWS_OR_REFERENCE",
    "RING_ENTRY_TRANSITION_TARGET", "RING_ENTRY_TRANSITION_UNVERIFIED",
    "RING_ENTRY_TRANSITION_ANCHORS",
)
_FREEZE_REASONS = ("NONE", "REVERSE_RESET", "PROFILE_INACTIVE",
                   "EXPLICIT_STOP", "CONTROLLED_STOP")


@micropython.native
def _clamp(value, low, high, default=0):
    if isinstance(value, bool) or not isinstance(value, int):
        return default
    if value < low:
        return low
    if value > high:
        return high
    return value


@micropython.native
def _code(value, choices):
    if not isinstance(value, str):
        return _clamp(value, 0, 32767, 0)
    try:
        return choices.index(value)
    except ValueError:
        return 0


def _crc32(data):
    if _binascii is not None and callable(getattr(_binascii, "crc32", None)):
        return int(_binascii.crc32(data)) & 0xFFFFFFFF
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _bit in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def _put_u16(data, offset, value):
    value = int(value) & 0xFFFF
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF


def _put_u32(data, offset, value):
    value = int(value) & 0xFFFFFFFF
    data[offset] = value & 0xFF
    data[offset + 1] = (value >> 8) & 0xFF
    data[offset + 2] = (value >> 16) & 0xFF
    data[offset + 3] = (value >> 24) & 0xFF


class RingDiagnostics:
    """Fixed-capacity history of every ring candidate approach in a run."""

    def __init__(self):
        self._frames = bytearray(
            EPISODE_CAPACITY * EPISODE_FRAME_CAPACITY * FRAME_RECORD_SIZE)
        self._prehistory = bytearray(PRE_FRAMES * FRAME_RECORD_SIZE)
        self._scratch = bytearray(FRAME_RECORD_SIZE)
        self._values = array("h", [0] * BASE_FIELD_COUNT)
        self._episode_meta = array("i", [0] * (EPISODE_CAPACITY * 11))
        # The producer normally reuses one fixed array('i').  Cache the
        # validated classification by identity and length, so the normal
        # record path avoids repr/memoryview work while replacements are
        # revalidated before any ptr32 access.
        self._reject_counts_owner = None
        self._reject_counts_length = -1
        self._reject_counts_kind = _REJECT_ARRAY_INVALID
        self._episode_count = 0
        self._episode_next = 0
        self._episode_overflow = 0
        self._pre_count = 0
        self._pre_write = 0
        self._frame_index = 0
        self._first_ms = None
        self._ticks_diff = None
        self._open_episode = -1
        self._idle_frames = 0
        self._candidate_episodes = 0
        self._frozen = False
        self._freeze_reason = 0
        self._persisted = False
        self._last_saved_path = None
        self._record_errors = 0

    @property
    def frozen(self):
        return self._frozen

    @property
    def persisted(self):
        return self._persisted

    @property
    def last_saved_path(self):
        return self._last_saved_path

    @property
    def episode_count(self):
        return self._episode_count

    @property
    def episode_overflow(self):
        return self._episode_overflow

    @property
    def record_errors(self):
        return self._record_errors

    def _episode_offset(self, slot, frame):
        return ((slot * EPISODE_FRAME_CAPACITY + frame) *
                FRAME_RECORD_SIZE)

    @staticmethod
    def _copy_record(destination, destination_offset, source,
                     source_offset=0):
        if (destination_offset < 0 or source_offset < 0 or
                destination_offset + FRAME_RECORD_SIZE > len(destination) or
                source_offset + FRAME_RECORD_SIZE > len(source)):
            raise IndexError("ring diagnostic record copy out of bounds")
        _copy_record_viper(
            destination, destination_offset, source, source_offset,
            FRAME_RECORD_SIZE)

    def _copy_prehistory_to_episode(self, slot):
        count = self._pre_count
        if count > PRE_FRAMES:
            count = PRE_FRAMES
        first = self._pre_write - count
        if first < 0:
            first += PRE_FRAMES
        for index in range(count):
            ring_index = first + index
            if ring_index >= PRE_FRAMES:
                ring_index -= PRE_FRAMES
            self._copy_record(
                self._frames, self._episode_offset(slot, index),
                self._prehistory, ring_index * FRAME_RECORD_SIZE)
        meta = slot * 11
        self._episode_meta[meta] = count
        self._episode_meta[meta + 1] = count
        self._episode_meta[meta + 2] = 0
        return count

    def _start_episode(self, frame_index, sequence):
        if self._episode_count < EPISODE_CAPACITY:
            slot = self._episode_count
            self._episode_count += 1
        else:
            slot = self._episode_next
            self._episode_overflow += 1
        self._episode_next = (slot + 1) % EPISODE_CAPACITY
        meta = slot * 11
        for field in range(11):
            self._episode_meta[meta + field] = 0
        self._episode_meta[meta + 5] = frame_index
        self._episode_meta[meta + 6] = sequence
        self._episode_meta[meta + 7] = sequence
        pre_count = self._copy_prehistory_to_episode(slot)
        # Restore fields that are initialized independently of prehistory.
        self._episode_meta[meta + 5] = frame_index
        self._episode_meta[meta + 6] = sequence
        self._episode_meta[meta + 7] = sequence
        self._episode_meta[meta + 1] = pre_count
        self._episode_meta[meta + 2] = 0
        self._open_episode = slot
        self._candidate_episodes += 1

    def _append_episode_frame(self, slot, sequence, state_id,
                              candidate, confirmed):
        meta = slot * 11
        frame_count = self._episode_meta[meta]
        if frame_count >= EPISODE_FRAME_CAPACITY:
            self._episode_meta[meta + 10] += 1
            return False
        destination = self._episode_offset(slot, frame_count)
        self._copy_record(self._frames, destination, self._scratch)
        self._episode_meta[meta] = frame_count + 1
        self._episode_meta[meta + 2] += 1
        self._episode_meta[meta + 7] = sequence
        self._episode_meta[meta + 8] = state_id
        if candidate:
            self._episode_meta[meta + 3] += 1
        if confirmed:
            self._episode_meta[meta + 4] += 1
        return True

    def _record_prehistory(self):
        offset = self._pre_write * FRAME_RECORD_SIZE
        self._copy_record(self._prehistory, offset, self._scratch)
        self._pre_write += 1
        if self._pre_write >= PRE_FRAMES:
            self._pre_write = 0
        if self._pre_count < PRE_FRAMES:
            self._pre_count += 1

    @micropython.native
    def begin_session(self, now_ms, ticks_diff=None):
        """Set a fresh timebase for a verified active Follow session."""
        if self._frozen:
            return False
        if (self._frame_index or self._pre_count or self._episode_count or
                self._open_episode >= 0):
            raise RuntimeError("ring diagnostics session is already recording")
        self._first_ms = now_ms
        self._ticks_diff = ticks_diff if callable(ticks_diff) else None
        return True

    @micropython.native
    def record_frame(self, now_ms, sequence, profile_id, active_follow,
                     observation_valid, selection, tracker_result,
                     ticks_diff=None):
        """Record one completed frame with bounded copies and no I/O/prints."""
        if self._frozen:
            return False
        if not isinstance(selection, dict):
            selection = {}
        if not isinstance(tracker_result, dict):
            tracker_result = {}
        try:
            if self._first_ms is None:
                self.begin_session(now_ms, ticks_diff)
            diff_fn = (ticks_diff if callable(ticks_diff) else
                       self._ticks_diff)
            if callable(diff_fn):
                elapsed_ms = diff_fn(now_ms, self._first_ms)
            else:
                elapsed_ms = now_ms - self._first_ms
            if (isinstance(elapsed_ms, bool) or
                    not isinstance(elapsed_ms, int)):
                raise ValueError("ring diagnostic time delta is invalid")
            if elapsed_ms < 0:
                elapsed_ms = 0
            max_elapsed_ms = 32767 * TIMESTAMP_TICK_MS
            if elapsed_ms > max_elapsed_ms:
                elapsed_ms = max_elapsed_ms
            values = self._values
            values[0] = _clamp(
                elapsed_ms // TIMESTAMP_TICK_MS, 0, 32767)
            sequence16 = (
                (int(sequence) & 0xFFFF)
                if isinstance(sequence, int) and not isinstance(sequence, bool)
                else 0)
            if sequence16 >= 0x8000:
                sequence16 -= 0x10000
            values[1] = sequence16
            values[2] = _clamp(profile_id, 0, 3)
            values[3] = 1 if active_follow else 0
            values[4] = 1 if observation_valid else 0
            values[5] = _SOURCE_CODES.get(selection.get("source", "NONE"), 0)
            values[6] = _code(selection.get("reason", ""), _SELECTION_REASONS)
            values[7] = _clamp(tracker_result.get("state_id", 0), 0, 32767)
            values[8] = _code(tracker_result.get("reason", ""), _TRACK_REASONS)
            values[9] = 1 if tracker_result.get("candidate") else 0
            values[10] = 1 if tracker_result.get("ring_confirmed") else 0
            values[11] = _clamp(tracker_result.get("side", -1), -1, 32767, -1)
            values[12] = _clamp(tracker_result.get("arc_rows", 0), 0, 32767)
            values[13] = _clamp(tracker_result.get("path_y_span", 0), 0, 32767)
            values[14] = _clamp(tracker_result.get("target_count", 0), 0, 32767)
            values[15] = _clamp(tracker_result.get("observed_rows", 0), 0, 32767)
            values[16] = _clamp(tracker_result.get("inferred_rows", 0), 0, 32767)
            values[17] = _code(tracker_result.get("reject_reason", "NONE"),
                               _REJECT_REASONS)
            values[18] = 0
            values[19] = 1 if tracker_result.get("ring_calibration_ready") else 0
            values[20] = _clamp(tracker_result.get(
                "ring_calibration_check_frames", 0), 0, 32767)
            values[21] = _clamp(tracker_result.get(
                "ring_calibration_support_rows", 0), 0, 32767)
            values[22] = _clamp(tracker_result.get(
                "ring_calibration_span_rows", 0), 0, 32767)
            values[23] = _clamp(tracker_result.get(
                "ring_calibration_median_error_permille", -1),
                -32768, 32767, -1)
            values[24] = 1 if tracker_result.get("entry_geometry_ok") else 0
            values[25] = 1 if tracker_result.get("current_geometry_ok") else 0
            values[26] = 1 if tracker_result.get("route_authorized") else 0
            values[27] = _clamp(tracker_result.get("arc_y_min", -1),
                                -1, 32767, -1)
            values[28] = _clamp(tracker_result.get("arc_y_max", -1),
                                -1, 32767, -1)
            values[29] = _clamp(tracker_result.get("vertex_y", -1),
                                -1, 32767, -1)
            values[30] = _clamp(tracker_result.get("vertex_x", -1),
                                -1, 32767, -1)
            values[31] = _clamp(tracker_result.get("intrusion", 0), 0, 32767)
            values[32] = _clamp(tracker_result.get("port_x", -1),
                                -1, 32767, -1)
            values[33] = _clamp(tracker_result.get("port_y", -1),
                                -1, 32767, -1)
            ring_diag = tracker_result.get("ring_diagnostic_record")
            ring_diag_stride = tracker_result.get("ring_diagnostic_stride", 14)
            ring_diag_count = 0
            if (isinstance(ring_diag_stride, int) and
                    not isinstance(ring_diag_stride, bool) and
                    ring_diag_stride == 14 and ring_diag is not None and
                    len(ring_diag) >= ring_diag_stride):
                ring_diag_count = ring_diag_stride
            for index in range(14):
                value = ring_diag[index] if index < ring_diag_count else 0
                values[34 + index] = _clamp(value, -32768, 32767)
            values[48] = _clamp(tracker_result.get("target_side", -1), -1, 1, -1)
            values[49] = (_clamp(tracker_result.get("target_center2", -1), -1, 158, -1)
                          if tracker_result.get("target_valid") else -1)
            values[50] = _clamp(tracker_result.get("diagnostic_position_error", 0), -1000, 1000)
            values[51] = _clamp(tracker_result.get("diagnostic_sent_error", 0), -1000, 1000)
            admission = tracker_result.get("route_admission_reason", "NOT_RUN")
            values[52] = _code(admission, _ROUTE_ADMISSION_REASONS)
            reject_counts = tracker_result.get("reject_counts")
            reject_total = 0
            reject_offset = BASE_FIELD_COUNT * 2
            reject_count_length = (
                len(reject_counts) if reject_counts is not None else 0)
            if (reject_counts is not self._reject_counts_owner or
                    reject_count_length != self._reject_counts_length):
                self._reject_counts_owner = reject_counts
                self._reject_counts_length = reject_count_length
                self._reject_counts_kind = _classify_reject_counts_array(
                    reject_counts)
            if self._reject_counts_kind == _REJECT_ARRAY_SIGNED_I:
                reject_total = _serialize_i32_rejects_viper(
                    reject_counts, self._scratch, reject_offset,
                    REJECT_COUNT)
            else:
                for index in range(REJECT_COUNT):
                    value = 0
                    if (reject_counts is not None and
                            len(reject_counts) > index):
                        value = _clamp(reject_counts[index], 0, 65535)
                    reject_total += value
                    value_offset = reject_offset + index * 2
                    self._scratch[value_offset] = value & 0xFF
                    self._scratch[value_offset + 1] = (value >> 8) & 0xFF
            values[18] = _clamp(reject_total, 0, 32767)
            _serialize_i16_fields_viper(
                values, self._scratch, BASE_FIELD_COUNT)

            candidate = bool(tracker_result.get("candidate"))
            confirmed = bool(tracker_result.get("ring_confirmed"))
            state_id = values[7]
            state_active = 1 <= state_id <= 6
            event_active = candidate or confirmed or state_active
            frame_index = self._frame_index
            self._frame_index += 1
            if event_active and self._open_episode < 0:
                self._start_episode(frame_index, values[1] & 0xFFFF)
            if self._open_episode >= 0:
                self._append_episode_frame(
                    self._open_episode, values[1] & 0xFFFF,
                    state_id, candidate, confirmed)
                if event_active:
                    self._idle_frames = 0
                else:
                    self._idle_frames += 1
                    if self._idle_frames >= 3:
                        self._open_episode = -1
                        self._idle_frames = 0
            self._record_prehistory()
            return True
        except Exception:
            self._record_errors += 1
            return False

    def freeze(self, reason="EXPLICIT_STOP"):
        """Freeze only when the caller has confirmed a real stop transition."""
        if self._frozen:
            return False
        if self._episode_count == 0 and self._pre_count > 0:
            self._start_stop_snapshot()
        self._frozen = True
        self._freeze_reason = _code(reason, _FREEZE_REASONS)
        if self._open_episode >= 0:
            self._open_episode = -1
        return True

    def _start_stop_snapshot(self):
        """Persist the remaining prehistory when no candidate ever fired."""
        if self._pre_count <= 0 or self._episode_count > 0:
            return False
        if self._episode_count < EPISODE_CAPACITY:
            slot = self._episode_count
            self._episode_count += 1
        else:
            slot = self._episode_next
            self._episode_overflow += 1
        self._episode_next = (slot + 1) % EPISODE_CAPACITY
        meta = slot * 11
        for field in range(11):
            self._episode_meta[meta + field] = 0
        frame_count = self._copy_prehistory_to_episode(slot)
        first_frame = self._frame_index - frame_count
        if first_frame < 0:
            first_frame = 0
        self._episode_meta[meta] = frame_count
        self._episode_meta[meta + 1] = frame_count
        self._episode_meta[meta + 2] = 0
        self._episode_meta[meta + 5] = first_frame
        first_offset = self._episode_offset(slot, 0)
        last_offset = first_offset + (frame_count - 1) * FRAME_RECORD_SIZE
        self._episode_meta[meta + 6] = (
            self._frames[first_offset + 2] |
            (self._frames[first_offset + 3] << 8))
        self._episode_meta[meta + 7] = (
            self._frames[last_offset + 2] |
            (self._frames[last_offset + 3] << 8))
        self._episode_meta[meta + 9] = STOP_SNAPSHOT_FLAG
        return True

    def rearm(self):
        """Reset logical bounds after save and reuse the fixed payload buffers."""
        if self._frozen and not self._persisted:
            return False
        # Frame and prehistory bytes are intentionally left in place. Every
        # subsequent record overwrites the full scratch record before copying,
        # and the payload writer serializes only frames reachable through the
        # reset episode/prehistory counts below.
        for index in range(len(self._episode_meta)):
            self._episode_meta[index] = 0
        self._episode_count = 0
        self._episode_next = 0
        self._episode_overflow = 0
        self._pre_count = 0
        self._pre_write = 0
        self._frame_index = 0
        self._first_ms = None
        self._ticks_diff = None
        self._open_episode = -1
        self._idle_frames = 0
        self._candidate_episodes = 0
        self._frozen = False
        self._freeze_reason = 0
        self._persisted = False
        self._record_errors = 0
        return True

    def _payload(self, build_id):
        if self._episode_count <= 0:
            raise RuntimeError("ring diagnostics contain no stopped frames")
        if not isinstance(build_id, str):
            build_id = "UNKNOWN"
        build_bytes = build_id.encode("ascii", "replace")[:BUILD_ID_BYTES]
        payload = bytearray(BUILD_ID_BYTES + 1 + 2)
        payload[0] = len(build_bytes)
        for index in range(len(build_bytes)):
            payload[index + 1] = build_bytes[index]
        _put_u16(payload, BUILD_ID_BYTES + 1, self._episode_overflow)
        if self._episode_count < EPISODE_CAPACITY:
            start_slot = 0
        else:
            start_slot = self._episode_next
        for order in range(self._episode_count):
            slot = (start_slot + order) % EPISODE_CAPACITY
            meta_index = slot * 11
            frame_count = _clamp(
                self._episode_meta[meta_index], 0,
                EPISODE_FRAME_CAPACITY)
            pre_count = _clamp(self._episode_meta[meta_index + 1], 0,
                               PRE_FRAMES)
            post_count = _clamp(self._episode_meta[meta_index + 2], 0,
                                POST_FRAMES)
            meta = _struct.pack(
                EPISODE_META_FORMAT,
                frame_count, pre_count, post_count,
                _clamp(self._episode_meta[meta_index + 3], 0, 65535),
                _clamp(self._episode_meta[meta_index + 4], 0, 65535),
                _clamp(self._episode_meta[meta_index + 5], 0, 0xFFFFFFFF),
                _clamp(self._episode_meta[meta_index + 6], 0, 65535),
                _clamp(self._episode_meta[meta_index + 7], 0, 65535),
                _clamp(self._episode_meta[meta_index + 8], 0, 65535),
                _clamp(self._episode_meta[meta_index + 9], 0, 65535),
                _clamp(self._episode_meta[meta_index + 10], 0, 65535))
            payload.extend(meta)
            source_offset = self._episode_offset(slot, 0)
            end = source_offset + frame_count * FRAME_RECORD_SIZE
            payload.extend(self._frames[source_offset:end])
        return payload

    @staticmethod
    def decode_file(data):
        """Validate the bounded binary header and CRC without trusting size."""
        if not isinstance(data, (bytes, bytearray)) or len(data) < HEADER_SIZE:
            raise ValueError("ring diagnostic file is truncated")
        if data[:4] != MAGIC:
            raise ValueError("ring diagnostic magic mismatch")
        version = data[4] | (data[5] << 8)
        header_size = data[6] | (data[7] << 8)
        record_size = data[8] | (data[9] << 8)
        episode_count = data[14] | (data[15] << 8)
        payload_size = (data[24] | (data[25] << 8) |
                        (data[26] << 16) | (data[27] << 24))
        expected_crc = (data[28] | (data[29] << 8) |
                        (data[30] << 16) | (data[31] << 24))
        expected_record_size = (FRAME_RECORD_SIZE if version == FORMAT_VERSION else
                                LEGACY_BASE_FIELD_COUNT * 2 + REJECT_COUNT * 2)
        if (version not in (LEGACY_FORMAT_VERSION, 2, FORMAT_VERSION) or
                header_size != HEADER_SIZE or
                record_size != expected_record_size or
                episode_count > EPISODE_CAPACITY or
                (version >= 2 and episode_count == 0) or
                payload_size > MAX_FILE_BYTES - HEADER_SIZE or
                len(data) != HEADER_SIZE + payload_size):
            raise ValueError("ring diagnostic header is invalid")
        payload = data[HEADER_SIZE:]
        if _crc32(payload) != expected_crc:
            raise ValueError("ring diagnostic CRC mismatch")
        build_length = payload[0]
        if build_length > BUILD_ID_BYTES:
            raise ValueError("ring diagnostic build id is invalid")
        build_id = bytes(payload[1:1 + build_length]).decode(
            "ascii", "replace")
        offset = BUILD_ID_BYTES + 3
        stop_snapshot_count = 0
        for _episode in range(episode_count):
            if offset + EPISODE_META_SIZE > len(payload):
                raise ValueError("ring diagnostic episode metadata is truncated")
            meta = _struct.unpack_from(EPISODE_META_FORMAT, payload, offset)
            frame_count, pre_count, post_count = meta[0:3]
            flags = meta[9]
            if (frame_count > EPISODE_FRAME_CAPACITY or
                    pre_count > PRE_FRAMES or post_count > POST_FRAMES or
                    frame_count != pre_count + post_count):
                raise ValueError("ring diagnostic episode metadata is invalid")
            if version >= 2:
                if flags & ~STOP_SNAPSHOT_FLAG:
                    raise ValueError("ring diagnostic episode flags are invalid")
                if flags & STOP_SNAPSHOT_FLAG:
                    if (pre_count <= 0 or post_count != 0 or
                            meta[3] != 0 or meta[4] != 0):
                        raise ValueError(
                            "ring diagnostic stop snapshot metadata is invalid")
                    stop_snapshot_count += 1
            offset += EPISODE_META_SIZE + frame_count * record_size
            if offset > len(payload):
                raise ValueError("ring diagnostic episode records are truncated")
        if offset != len(payload):
            raise ValueError("ring diagnostic payload size is invalid")
        return {
            "format_version": version,
            "record_size": record_size,
            "timestamp_unit_ms": (
                TIMESTAMP_TICK_MS if version >= 2 else 1),
            "episode_count": episode_count,
            "stop_snapshot_count": stop_snapshot_count,
            "episode_overflow": payload[BUILD_ID_BYTES + 1] |
                (payload[BUILD_ID_BYTES + 2] << 8),
            "build_id": build_id,
            "payload_size": payload_size,
            "crc32": expected_crc,
        }

    @staticmethod
    def _split_path(path):
        separator = max(path.rfind("/"), path.rfind("\\"))
        if separator < 0:
            directory = "."
            name = path
        else:
            directory = path[:separator] or "/"
            name = path[separator + 1:]
        dot = name.rfind(".")
        if dot <= 0:
            return directory, name, ""
        return directory, name[:dot], name[dot:]

    @staticmethod
    def _directory_entries(directory):
        listdir = getattr(os, "listdir", None)
        if not callable(listdir):
            raise OSError("directory scan is unavailable")
        return listdir(directory)

    @classmethod
    def next_available_path(cls, base_path):
        """Choose max existing numbered file + 1, with a fixed upper bound."""
        base = str(base_path)
        directory, stem, extension = cls._split_path(base)
        prefix = stem + "_"
        entries = cls._directory_entries(directory)
        maximum = 0
        for name in entries:
            if (not name.startswith(prefix) or
                    (extension and not name.endswith(extension))):
                continue
            end = len(name) - len(extension) if extension else len(name)
            digits = name[len(prefix):end]
            if not digits:
                continue
            number = 0
            valid = True
            for character in digits:
                if character < "0" or character > "9":
                    valid = False
                    break
                number = number * 10 + ord(character) - 48
            if not valid:
                continue
            if number > 999999:
                raise OSError("ring diagnostic file number limit reached")
            if len(digits) != 6:
                continue
            if number > maximum:
                maximum = number
        if maximum >= 999999:
            raise OSError("ring diagnostic file number limit reached")
        number_text = "%06d" % (maximum + 1)
        name = prefix + number_text + extension
        return name if directory == "." else directory.rstrip("/\\") + "/" + name

    @classmethod
    def _path_exists(cls, path):
        directory, _stem, _extension = cls._split_path(path)
        name = str(path).split("/")[-1].split("\\")[-1]
        entries = cls._directory_entries(directory)
        return name in entries

    @classmethod
    def _atomic_create(cls, source, destination):
        if cls._path_exists(destination):
            raise OSError("ring diagnostic target already exists")
        rename = getattr(os, "rename", None)
        if not callable(rename):
            raise OSError("atomic rename is unavailable")
        rename(source, destination)

    def save(self, path, build_id):
        """Write once after freeze without replacing any existing file."""
        if not self._frozen:
            raise RuntimeError("ring diagnostics must be frozen before save")
        if self._persisted:
            raise RuntimeError("ring diagnostics were already persisted")
        if self._episode_count <= 0:
            raise RuntimeError("ring diagnostics contain no stopped frames")
        target = str(path)
        if self._path_exists(target):
            raise OSError("ring diagnostic target already exists")
        payload = self._payload(build_id)
        total_size = HEADER_SIZE + len(payload)
        if total_size > MAX_FILE_BYTES:
            raise ValueError("ring diagnostic file exceeds the fixed bound")
        header = bytearray(HEADER_SIZE)
        header[0:4] = MAGIC
        _put_u16(header, 4, FORMAT_VERSION)
        _put_u16(header, 6, HEADER_SIZE)
        _put_u16(header, 8, FRAME_RECORD_SIZE)
        _put_u16(header, 10, REJECT_COUNT)
        _put_u16(header, 12, EPISODE_CAPACITY)
        _put_u16(header, 14, self._episode_count)
        _put_u16(header, 16, PRE_FRAMES)
        _put_u16(header, 18, POST_FRAMES)
        _put_u16(header, 20, self._episode_overflow)
        _put_u16(header, 22, self._freeze_reason)
        _put_u32(header, 24, len(payload))
        _put_u32(header, 28, _crc32(payload))
        temporary = target + ".tmp"
        try:
            with open(temporary, "wb") as destination:
                destination.write(header)
                destination.write(payload)
                flush = getattr(destination, "flush", None)
                if callable(flush):
                    flush()
            self._atomic_create(temporary, target)
        except Exception:
            self._persisted = False
            try:
                os.remove(temporary)
            except Exception:
                pass
            raise
        self._persisted = True
        self._last_saved_path = target
        return True

    def save_next(self, base_path, build_id):
        """Save under the next numbered name without overwriting older runs."""
        target = self.next_available_path(base_path)
        self.save(target, build_id)
        return target

    def summary_lines(self, build_id):
        """Return a bounded post-stop summary (at most nine lines)."""
        lines = ["RING_DIAG version=%d build=%s frozen=%d episodes=%d "
                 "overflow=%d record_errors=%d persisted=%d" % (
                     FORMAT_VERSION, str(build_id),
                     1 if self._frozen else 0, self._episode_count,
                     self._episode_overflow, self._record_errors,
                     1 if self._persisted else 0)]
        if self._episode_count < EPISODE_CAPACITY:
            start_slot = 0
        else:
            start_slot = self._episode_next
        for order in range(self._episode_count):
            slot = (start_slot + order) % EPISODE_CAPACITY
            meta = slot * 11
            lines.append("RING_DIAG_EPISODE index=%d frames=%d pre=%d post=%d "
                         "candidates=%d confirmed=%d start_seq=%d end_seq=%d "
                         "dropped=%d" % (
                             order, self._episode_meta[meta],
                             self._episode_meta[meta + 1],
                             self._episode_meta[meta + 2],
                             self._episode_meta[meta + 3],
                             self._episode_meta[meta + 4],
                             self._episode_meta[meta + 6],
                             self._episode_meta[meta + 7],
                             self._episode_meta[meta + 10]))
        return lines


__all__ = ("BASE_FIELD_COUNT", "EPISODE_CAPACITY", "FORMAT_VERSION",
           "FRAME_RECORD_SIZE", "MAX_FILE_BYTES", "POST_FRAMES",
           "PRE_FRAMES", "REJECT_COUNT", "RingDiagnostics",
           "STOP_SNAPSHOT_FLAG", "TIMESTAMP_TICK_MS")
