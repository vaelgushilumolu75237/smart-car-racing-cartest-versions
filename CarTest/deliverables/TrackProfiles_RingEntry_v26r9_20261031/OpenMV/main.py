"""V7 fast grayscale detector and UART V3 publisher."""

import pyb
import sensor
import time
import gc
from array import array

try:
    import micropython
    _NATIVE_ACCELERATION_ENABLED = True
except ImportError:
    # The literal ``@micropython.native`` spelling is required for MicroPython
    # to recognize its compile-time native-code directive. CPython host tests
    # keep the same call path with an identity decorator.
    class _MicropythonHostCompat:
        @staticmethod
        def native(function):
            return function

        @staticmethod
        def viper(function):
            return function

    micropython = _MicropythonHostCompat()

    def ptr32(buffer):
        return buffer

    def ptr8(buffer):
        return buffer

    def ptr16(buffer):
        return buffer

    _NATIVE_ACCELERATION_ENABLED = False

import config
import vision_protocol

try:
    import cartest_accel
except Exception:
    # The native module is optional.  The existing gc.mem_free/mem_alloc
    # fallback remains the source of truth when it is absent or incompatible.
    cartest_accel = None

class _FreshCaptureFPS:
    """Count successful capture intervals in wrap-safe 1 s windows."""
    WINDOW_MS = 1000
    RESET_GAP_MS = 2000

    def __init__(self):
        self._window_start_ms = None
        self._last_capture_ms = None
        self._window_intervals = 0
        self._fps_value = 0.0

    def record_success(self, now_ms):
        """Record one successful snapshot return timestamp."""
        if self._last_capture_ms is None:
            self._window_start_ms = now_ms
            self._last_capture_ms = now_ms
            self._window_intervals = 0
            self._fps_value = 0.0
            return

        gap_ms = time.ticks_diff(now_ms, self._last_capture_ms)
        elapsed_ms = time.ticks_diff(now_ms, self._window_start_ms)
        if (gap_ms < 0 or elapsed_ms < 0 or
                gap_ms >= self.RESET_GAP_MS):
            self._window_start_ms = now_ms
            self._last_capture_ms = now_ms
            self._window_intervals = 0
            self._fps_value = 0.0
            return

        self._window_intervals += 1
        if elapsed_ms >= self.WINDOW_MS:
            self._fps_value = (self._window_intervals * 1000.0 /
                               elapsed_ms)
            self._window_start_ms = now_ms
            self._window_intervals = 0
        self._last_capture_ms = now_ms

    def fps(self, now_ms):
        """Return the current window rate without changing counter state."""
        if self._last_capture_ms is None:
            return 0.0
        age_ms = time.ticks_diff(now_ms, self._last_capture_ms)
        if age_ms < 0 or age_ms >= self.RESET_GAP_MS:
            return 0.0
        return self._fps_value



@micropython.viper
def _weighted_row_sums_viper(center_x2_object, row_y_object,
                             weights_object, count: int,
                             core_low: int, core_high: int,
                             output_object) -> int:
    """Sum accepted fixed-point row weights using preallocated int arrays."""
    centers = ptr32(center_x2_object)
    row_y = ptr32(row_y_object)
    weights = ptr32(weights_object)
    output = ptr32(output_object)
    weighted_x2 = 0
    total_weight = 0
    core_weight = 0
    index = 0
    while index < count:
        y = row_y[index]
        weight = weights[y]
        total_weight += weight
        weighted_x2 += centers[index] * weight
        if core_low <= y <= core_high:
            core_weight += weight
        index += 1
    output[0] = weighted_x2
    output[1] = total_weight
    output[2] = core_weight
    return 1


@micropython.native
def _road_preview_int32_array_valid(values):
    """Validate one private integer-array producer buffer for the Viper path."""
    if not isinstance(values, array):
        return False
    try:
        view = memoryview(values)
        itemsize = getattr(view, "itemsize", None)
    except (TypeError, AttributeError):
        return False
    if itemsize is None:
        # OpenMV v5 omits memoryview width/format. Check the physical width once
        # for each workspace-owned array identity, before entering the kernel.
        try:
            raw_bytes = bytes(view)
            valid_size = len(raw_bytes) == len(values) * 4
            del raw_bytes
        except (TypeError, ValueError, MemoryError):
            return False
        if not valid_size:
            return False
    elif itemsize != 4:
        return False
    view_format = getattr(view, "format", None)
    if (view_format is not None and
            view_format not in ("i", "l", "@i", "@l", "=i", "=l")):
        return False
    typecode = getattr(values, "typecode", None)
    if typecode is not None and typecode != "i":
        return False
    try:
        if len(values) == 0 or not isinstance(values[0], int):
            return False
    except (IndexError, TypeError):
        return False
    # The workspace creates these private arrays with array("i"). Ports that
    # omit typecode and view metadata rely on that producer contract plus the
    # physical-width and integer-sample checks above.
    return True


@micropython.viper
def _road_preview_segment_scan_viper(
        valid_rows_object, row_y_object, floor_y: int, height: int,
        band_low: int, band_high: int, gap_limit: int,
        output_object) -> int:
    """Find the unique reference-containing preview segment.

    This is the typed equivalent of the bounded scan formerly performed by
    ``_compute_road_preview``.  It writes status, selected start/end, row
    count, and matching-segment count to a preallocated int array.  The
    Python path remains the fallback for unusual host buffers or non-integer
    configuration values.
    """
    if (floor_y < 0 or height <= 0 or height > 60 or
            floor_y >= height or gap_limit < 0):
        return -1
    valid_rows = ptr8(valid_rows_object)
    row_y = ptr32(row_y_object)
    output = ptr32(output_object)
    selected_start_y = -1
    selected_end_y = -1
    selected_rows = 0
    matching_segments = 0
    segment_start_y = -1
    segment_end_y = -1
    segment_rows = 0
    segment_contains_reference = 0
    previous_y = -1
    slot = floor_y
    while slot < height:
        if valid_rows[slot] == 0:
            slot += 1
            continue
        current_y = row_y[slot]
        if (segment_start_y >= 0 and
                current_y - previous_y > gap_limit):
            if segment_contains_reference != 0:
                matching_segments += 1
                if matching_segments > 1:
                    output[0] = -1
                    output[1] = -1
                    output[2] = -1
                    output[3] = -1
                    output[4] = matching_segments
                    return -1
                selected_start_y = segment_start_y
                selected_end_y = segment_end_y
                selected_rows = segment_rows
            segment_start_y = current_y
            segment_rows = 1
            segment_contains_reference = (
                1 if band_low <= current_y <= band_high else 0)
        elif segment_start_y < 0:
            segment_start_y = current_y
            segment_rows = 1
            segment_contains_reference = (
                1 if band_low <= current_y <= band_high else 0)
        else:
            segment_rows += 1
            if band_low <= current_y <= band_high:
                segment_contains_reference = 1
        segment_end_y = current_y
        previous_y = current_y
        slot += 1

    if segment_start_y >= 0 and segment_contains_reference != 0:
        matching_segments += 1
        if matching_segments > 1:
            output[0] = -1
            output[1] = -1
            output[2] = -1
            output[3] = -1
            output[4] = matching_segments
            return -1
        selected_start_y = segment_start_y
        selected_end_y = segment_end_y
        selected_rows = segment_rows

    if selected_start_y < 0:
        output[0] = 0
        output[1] = -1
        output[2] = -1
        output[3] = 0
        output[4] = matching_segments
        return 0

    output[0] = 1
    output[1] = selected_start_y
    output[2] = selected_end_y
    output[3] = selected_rows
    output[4] = matching_segments
    return 1


@micropython.viper
def _road_preview_integer_geometry_viper(
        valid_rows_object, centers_object, row_y_object,
        selected_start_y: int, selected_end_y: int, endpoint_rows: int,
        scratch_object, middle_y_object, distances_object,
        output_object) -> int:
    """Collect exact endpoint and middle medians from bounded row arrays."""
    if (selected_start_y < 0 or selected_start_y >= 60 or
            selected_end_y < selected_start_y or selected_end_y >= 60 or
            endpoint_rows != 5):
        return -1

    valid_rows = ptr8(valid_rows_object)
    centers = ptr32(centers_object)
    row_y = ptr32(row_y_object)
    scratch = ptr32(scratch_object)
    middle_y = ptr32(middle_y_object)
    distances = ptr32(distances_object)
    output = ptr32(output_object)

    collected = 0
    far_y = -1
    far_band_end = -1
    slot = selected_start_y
    while slot <= selected_end_y:
        if valid_rows[slot] != 0:
            scratch[collected] = centers[slot]
            collected += 1
            if collected == 3:
                far_y = row_y[slot]
            if collected == 5:
                far_band_end = slot
                break
        slot += 1
    if collected != 5:
        return 0

    index = 1
    while index < 5:
        value = scratch[index]
        insert_at = index
        while insert_at > 0 and scratch[insert_at - 1] > value:
            scratch[insert_at] = scratch[insert_at - 1]
            insert_at -= 1
        scratch[insert_at] = value
        index += 1
    far_center_x2 = scratch[2]

    collected = 0
    near_y = -1
    near_band_start = -1
    slot = selected_end_y
    while slot >= selected_start_y:
        if valid_rows[slot] != 0:
            scratch[collected] = centers[slot]
            collected += 1
            if collected == 3:
                near_y = row_y[slot]
            if collected == 5:
                near_band_start = slot
                break
        slot -= 1
    if collected != 5:
        return 0

    index = 1
    while index < 5:
        value = scratch[index]
        insert_at = index
        while insert_at > 0 and scratch[insert_at - 1] > value:
            scratch[insert_at] = scratch[insert_at - 1]
            insert_at -= 1
        scratch[insert_at] = value
        index += 1
    near_center_x2 = scratch[2]

    middle_count = 0
    target_y2 = near_y + far_y
    slot = far_band_end + 1
    while slot < near_band_start:
        if valid_rows[slot] != 0:
            distance = slot * 2 - target_y2
            if distance < 0:
                distance = -distance
            insert_at = middle_count
            if insert_at >= 5:
                insert_at = 4
                if distance >= distances[insert_at]:
                    slot += 1
                    continue
            else:
                middle_count += 1
            while insert_at > 0 and distances[insert_at - 1] > distance:
                distances[insert_at] = distances[insert_at - 1]
                middle_y[insert_at] = middle_y[insert_at - 1]
                scratch[insert_at] = scratch[insert_at - 1]
                insert_at -= 1
            distances[insert_at] = distance
            middle_y[insert_at] = slot
            scratch[insert_at] = centers[slot]
        slot += 1
    if middle_count != 5:
        return 0

    index = 1
    while index < 5:
        value = scratch[index]
        insert_at = index
        while insert_at > 0 and scratch[insert_at - 1] > value:
            scratch[insert_at] = scratch[insert_at - 1]
            insert_at -= 1
        scratch[insert_at] = value
        index += 1
    middle_center_x2 = scratch[2]

    index = 1
    while index < 5:
        value = middle_y[index]
        insert_at = index
        while insert_at > 0 and middle_y[insert_at - 1] > value:
            middle_y[insert_at] = middle_y[insert_at - 1]
            insert_at -= 1
        middle_y[insert_at] = value
        index += 1
    middle_row = middle_y[2]

    output[0] = far_center_x2
    output[1] = far_y
    output[2] = near_center_x2
    output[3] = near_y
    output[4] = middle_center_x2
    output[5] = middle_row
    return 1


@micropython.viper
def _aggregate_selected_rows_viper(
        rows_object, count: int, stride: int, road_source: int,
        support_object, preview_valid_object, preview_centers_object,
        touched_rows_object, touched_count: int, centers_object,
        row_y_object, floor_y: int, width: int, height: int,
        preview_floor: int, core_low: int, core_high: int,
        reference_y: int, reference_tolerance: int,
        output_object) -> int:
    """Apply the validated-row gates and collect bounded aggregate metrics."""
    rows = ptr32(rows_object)
    support = ptr8(support_object)
    preview_valid = ptr8(preview_valid_object)
    preview_centers = ptr32(preview_centers_object)
    touched_rows = ptr32(touched_rows_object)
    centers = ptr32(centers_object)
    row_y = ptr32(row_y_object)
    output = ptr32(output_object)

    preview_count = 0
    conflicting_centers = 0
    accepted_count = 0
    core_count = 0
    core_min_y = height
    core_max_y = -1
    best_reference_index = -1
    best_reference_distance = reference_tolerance + 1
    reference_conflict = 0
    offset = 0
    index = 0

    while index < count:
        y = rows[offset]
        accepted = 1
        if road_source != 0:
            if support[index] != 1 or (rows[offset + 3] & 7) != 7:
                accepted = 0
            center_x2 = rows[offset + 1] + rows[offset + 2]
        else:
            center_x2 = rows[offset + 1]

        if (accepted != 0 and y >= floor_y and y < height and
                center_x2 >= 0 and center_x2 <= (width - 1) * 2):
            if preview_valid[y] != 0:
                if preview_centers[y] != center_x2:
                    conflicting_centers = 1
            else:
                preview_valid[y] = 1
                preview_centers[y] = center_x2
                touched_rows[touched_count] = y
                touched_count += 1
                if y >= preview_floor:
                    preview_count += 1

                centers[accepted_count] = center_x2
                row_y[accepted_count] = y
                accepted_count += 1
                if y >= core_low and y <= core_high:
                    core_count += 1
                    if y < core_min_y:
                        core_min_y = y
                    if y > core_max_y:
                        core_max_y = y

                distance = y - reference_y
                if distance < 0:
                    distance = -distance
                if distance <= reference_tolerance:
                    if distance < best_reference_distance:
                        best_reference_index = accepted_count - 1
                        best_reference_distance = distance
                        reference_conflict = 0
                    elif (distance == best_reference_distance and
                          best_reference_index >= 0 and
                          centers[best_reference_index] != center_x2):
                        reference_conflict = 1

        offset += stride
        index += 1

    output[0] = accepted_count
    output[1] = preview_count
    output[2] = conflicting_centers
    output[3] = core_count
    output[4] = core_min_y
    output[5] = core_max_y
    output[6] = best_reference_index
    output[7] = best_reference_distance
    output[8] = reference_conflict
    output[9] = touched_count
    return 1


@micropython.viper
def _crc16_control_extension_viper(data_object, offset: int,
                                   table_object) -> int:
    """Write the existing CRC-16/CCITT-FALSE checksum for one trace record."""
    data = ptr8(data_object)
    table = ptr16(table_object)
    crc = 0xFFFF
    index = 0
    while index < 30:
        crc = crc ^ (data[offset + index] << 8)
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        index += 1
    data[offset + 30] = crc & 0xFF
    data[offset + 31] = (crc >> 8) & 0xFF
    return crc


@micropython.viper
def _write_control_extension_viper(
        data_object, offset: int, table_object, version: int, flags: int,
        sequence: int, capture_ms: int, send_ms: int,
        position_p: int, derivative_d: int, pd_output: int,
        preview_candidate: int, preview_gain_milli: int,
        preview_applied: int, capture_dt_ms: int,
        preview_state: int, reset_code: int, sent_control: int) -> int:
    """Serialize a fully validated fixed-size control extension in one pass."""
    data = ptr8(data_object)
    table = ptr16(table_object)

    data[offset] = version & 0xFF
    data[offset + 1] = flags & 0xFF
    data[offset + 2] = sequence & 0xFF
    data[offset + 3] = (sequence >> 8) & 0xFF
    data[offset + 4] = capture_ms & 0xFF
    data[offset + 5] = (capture_ms >> 8) & 0xFF
    data[offset + 6] = (capture_ms >> 16) & 0xFF
    data[offset + 7] = (capture_ms >> 24) & 0xFF
    data[offset + 8] = send_ms & 0xFF
    data[offset + 9] = (send_ms >> 8) & 0xFF
    data[offset + 10] = (send_ms >> 16) & 0xFF
    data[offset + 11] = (send_ms >> 24) & 0xFF

    if position_p < -32768:
        position_p = -32768
    elif position_p > 32767:
        position_p = 32767
    if position_p < 0:
        position_p += 65536
    data[offset + 12] = position_p & 0xFF
    data[offset + 13] = (position_p >> 8) & 0xFF

    if derivative_d < -32768:
        derivative_d = -32768
    elif derivative_d > 32767:
        derivative_d = 32767
    if derivative_d < 0:
        derivative_d += 65536
    data[offset + 14] = derivative_d & 0xFF
    data[offset + 15] = (derivative_d >> 8) & 0xFF

    if pd_output < -32768:
        pd_output = -32768
    elif pd_output > 32767:
        pd_output = 32767
    if pd_output < 0:
        pd_output += 65536
    data[offset + 16] = pd_output & 0xFF
    data[offset + 17] = (pd_output >> 8) & 0xFF

    if preview_candidate < -32768:
        preview_candidate = -32768
    elif preview_candidate > 32767:
        preview_candidate = 32767
    if preview_candidate < 0:
        preview_candidate += 65536
    data[offset + 18] = preview_candidate & 0xFF
    data[offset + 19] = (preview_candidate >> 8) & 0xFF

    data[offset + 20] = preview_gain_milli & 0xFF
    data[offset + 21] = (preview_gain_milli >> 8) & 0xFF
    if preview_applied < -32768:
        preview_applied = -32768
    elif preview_applied > 32767:
        preview_applied = 32767
    if preview_applied < 0:
        preview_applied += 65536
    data[offset + 22] = preview_applied & 0xFF
    data[offset + 23] = (preview_applied >> 8) & 0xFF

    if capture_dt_ms < 0:
        capture_dt_ms = 0
    elif capture_dt_ms > 65535:
        capture_dt_ms = 65535
    data[offset + 24] = capture_dt_ms & 0xFF
    data[offset + 25] = (capture_dt_ms >> 8) & 0xFF
    if preview_state < 0:
        preview_state = 0
    elif preview_state > 255:
        preview_state = 255
    data[offset + 26] = preview_state
    if reset_code < 0:
        reset_code = 0
    elif reset_code > 255:
        reset_code = 255
    data[offset + 27] = reset_code

    if sent_control < -32768:
        sent_control = -32768
    elif sent_control > 32767:
        sent_control = 32767
    if sent_control < 0:
        sent_control += 65536
    data[offset + 28] = sent_control & 0xFF
    data[offset + 29] = (sent_control >> 8) & 0xFF

    crc = 0xFFFF
    index = 0
    while index < 30:
        crc = crc ^ (data[offset + index] << 8)
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
        index += 1
    data[offset + 30] = crc & 0xFF
    data[offset + 31] = (crc >> 8) & 0xFF
    return crc


@micropython.viper
def _write_camera_diag_record_viper(
        data_object, offset: int, sequence: int, flags: int,
        capture_ms: int, send_ms: int, position: int,
        preview_candidate: int, preview_applied: int, sent_error: int,
        raw_control: int, filtered_error: int, reference_error: int,
        row_count: int, source_code: int, reverse_generation: int) -> int:
    """Pack the 32-byte camera trace record without Python field writers."""
    data = ptr8(data_object)
    data[offset] = sequence & 0xFF
    data[offset + 1] = (sequence >> 8) & 0xFF
    data[offset + 2] = flags & 0xFF
    data[offset + 3] = 0

    data[offset + 4] = capture_ms & 0xFF
    data[offset + 5] = (capture_ms >> 8) & 0xFF
    data[offset + 6] = (capture_ms >> 16) & 0xFF
    data[offset + 7] = (capture_ms >> 24) & 0xFF
    data[offset + 8] = send_ms & 0xFF
    data[offset + 9] = (send_ms >> 8) & 0xFF
    data[offset + 10] = (send_ms >> 16) & 0xFF
    data[offset + 11] = (send_ms >> 24) & 0xFF

    if position < -32768:
        position = -32768
    elif position > 32767:
        position = 32767
    if position < 0:
        position += 65536
    data[offset + 12] = position & 0xFF
    data[offset + 13] = (position >> 8) & 0xFF

    if preview_candidate < -32768:
        preview_candidate = -32768
    elif preview_candidate > 32767:
        preview_candidate = 32767
    if preview_candidate < 0:
        preview_candidate += 65536
    data[offset + 14] = preview_candidate & 0xFF
    data[offset + 15] = (preview_candidate >> 8) & 0xFF

    if preview_applied < -32768:
        preview_applied = -32768
    elif preview_applied > 32767:
        preview_applied = 32767
    if preview_applied < 0:
        preview_applied += 65536
    data[offset + 16] = preview_applied & 0xFF
    data[offset + 17] = (preview_applied >> 8) & 0xFF

    if sent_error < -32768:
        sent_error = -32768
    elif sent_error > 32767:
        sent_error = 32767
    if sent_error < 0:
        sent_error += 65536
    data[offset + 18] = sent_error & 0xFF
    data[offset + 19] = (sent_error >> 8) & 0xFF

    if raw_control < -32768:
        raw_control = -32768
    elif raw_control > 32767:
        raw_control = 32767
    if raw_control < 0:
        raw_control += 65536
    data[offset + 20] = raw_control & 0xFF
    data[offset + 21] = (raw_control >> 8) & 0xFF

    if filtered_error < -32768:
        filtered_error = -32768
    elif filtered_error > 32767:
        filtered_error = 32767
    if filtered_error < 0:
        filtered_error += 65536
    data[offset + 22] = filtered_error & 0xFF
    data[offset + 23] = (filtered_error >> 8) & 0xFF

    if reference_error < -32768:
        reference_error = -32768
    elif reference_error > 32767:
        reference_error = 32767
    if reference_error < 0:
        reference_error += 65536
    data[offset + 24] = reference_error & 0xFF
    data[offset + 25] = (reference_error >> 8) & 0xFF

    if row_count < 0:
        row_count = 0
    elif row_count > 255:
        row_count = 255
    data[offset + 26] = row_count
    if source_code < 0 or source_code > 5:
        source_code = 0
    data[offset + 27] = source_code

    data[offset + 28] = reverse_generation & 0xFF
    data[offset + 29] = (reverse_generation >> 8) & 0xFF
    data[offset + 30] = (reverse_generation >> 16) & 0xFF
    data[offset + 31] = (reverse_generation >> 24) & 0xFF
    return 1

# V6 remains an explicit diagnostic/fallback entry. These modules are loaded
# only when that backend is selected, keeping both legacy detectors out of the
# default V7 runtime's import and heap footprint.
_V6_MODULE_CACHE = {}


class _LazyV6Module:
    """Load one compare-only module on first attribute access."""

    def __init__(self, name):
        self._module_name = name

    def __getattr__(self, attribute):
        name = self._module_name
        if name not in _V6_MODULE_CACHE:
            _load_v6_runtime()
        return getattr(_V6_MODULE_CACHE[name], attribute)


grayscale_adapter = _LazyV6Module("grayscale_adapter")
thresholding = _LazyV6Module("thresholding")
road_detector = _LazyV6Module("road_detector")
surface_detector = _LazyV6Module("surface_detector")
source_selector = _LazyV6Module("source_selector")
boundary_evidence = _LazyV6Module("boundary_evidence")
fast_gray = None
fast_path = None
fast_topology = None
fast_boundary = None
fast_single_edge = None
reverse_control = None
track_elements = None
contour_route = None
contour_trace = None
ring_width_calibration = None
ring_diagnostics = None
_RING_DIAGNOSTICS = None
_RING_DIAG_SESSION = None


class _LazyAdaptiveDetector:
    """Compatibility proxy for the explicit legacy path and old host tests."""

    def __getattr__(self, name):
        module = _load_adaptive_detector()
        return getattr(module, name)


def _load_adaptive_detector():
    global _adaptive_detector
    if _adaptive_detector is None:
        try:
            _adaptive_detector = __import__("adaptive_detector")
        except ImportError as exc:
            raise ImportError("legacy adaptive detector is unavailable: %s" % exc)
    return _adaptive_detector


adaptive_detector = _LazyAdaptiveDetector()
_adaptive_detector = None


ROI_BUILD_ID = "SIXTH_V4_BOUNDARY_GUARD_20261003"
V6_RUNTIME_API_VERSION = 6
V7_RUNTIME_API_VERSION = 7
V7_BUILD_ID = "CARTEST_OPENMV_VISION_V1_20261006"
SINGLEEDGE_RUNTIME_API_VERSION = 2
REVERSE_CONTROL_RUNTIME_API_VERSION = 2
CONTOUR_RUNTIME_API_VERSION = 1
TRACK_ELEMENTS_RUNTIME_API_VERSION = 2
RING_WIDTH_CALIBRATION_RUNTIME_API_VERSION = 1
CACHED_WIDTH_CONTROL_PROFILE_BUILD_IDS = tuple(
    config.STEERING_CONTROL_PROFILE_BUILD_IDS)
CACHED_WIDTH_BUILD_ID = dict(
    CACHED_WIDTH_CONTROL_PROFILE_BUILD_IDS).get(
        "pd", "BOTTOM80_POSPD_20261020")
CLIPPED_MIDPOINT_BUILD_ID = "CLIPPED_MIDPOINT_RING_20261014"
SLOPE_CLASSIFIER_RUNTIME_ENABLED = False
SINGLEEDGE_ROUTE_BUILD_IDS = (
    tuple(("cached_width", build_id)
          for _profile, build_id in
          CACHED_WIDTH_CONTROL_PROFILE_BUILD_IDS) +
    (("clipped_midpoint", CLIPPED_MIDPOINT_BUILD_ID),))


class _SteeringDiagBuffer:
    """Fixed-size pre/post-trigger RAM trace retained at the camera REPL."""

    CAPACITY = 256
    RECORD_SIZE = 32
    PRE_TRIGGER_CAPACITY = 64
    POST_TRIGGER_CAPACITY = 192
    EXTENSION_RECORD_SIZE = 16
    CONTROL_EXTENSION_VERSION = 2
    CONTROL_EXTENSION_RECORD_SIZE = 32
    _CRC16_NIBBLE_TABLE = (
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF)
    _CRC16_NIBBLE_TABLE_ARRAY = array("H", _CRC16_NIBBLE_TABLE)
    _CSV_HEADER = (
        "sequence,capture_ms,send_ms,valid,full_write,write_error,"
        "reverse_fresh,position_norm,preview_candidate_norm,"
        "preview_applied_norm,sent_error_norm,raw_control_norm,"
        "filtered_error_norm,reference_error_norm,row_count,source_code,"
        "reverse_generation,pair_reason,pair_common_rows,pair_common_span,"
        "pair_max_center_delta2,pair_selected_side,pair_selection,"
        "pair_left_support,pair_right_support,candidate_flags,"
        "result_reason_code,pending_side,pending_count,anchor_rejects,"
        "corridor_rejects,jump_breaks,gap_breaks,control_ext_v2_hex,"
        "curve_trace_v2_hex")

    def __init__(self):
        self.data = bytearray(self.CAPACITY * self.RECORD_SIZE)
        self.extension_data = bytearray(
            self.CAPACITY * self.EXTENSION_RECORD_SIZE)
        self.control_extension_data = bytearray(
            self.CAPACITY * self.CONTROL_EXTENSION_RECORD_SIZE)
        self.curve_trace_data = bytearray(
            self.CAPACITY * vision_protocol.FRAME_SIZE)
        self.pre_count = 0
        self.pre_oldest = 0
        self.write_index = 0
        self.count = 0
        self.overwritten = 0
        self.record_errors = 0
        self.frozen = False
        self.triggered = False
        self.post_count = 0
        self.freeze_reason = "RUNNING"
        self.dumped = False
        self.last_full_valid = False

    def rearm(self):
        self.write_index = 0
        self.pre_count = 0
        self.pre_oldest = 0
        self.count = 0
        self.overwritten = 0
        self.record_errors = 0
        self.frozen = False
        self.triggered = False
        self.post_count = 0
        self.freeze_reason = "RUNNING"
        self.dumped = False
        self.last_full_valid = False

    @staticmethod
    def _put_u16(data, offset, value):
        if not isinstance(value, int) or isinstance(value, bool):
            value = 0
        value &= 0xFFFF
        data[offset] = value & 0xFF
        data[offset + 1] = (value >> 8) & 0xFF

    @staticmethod
    def _put_i16(data, offset, value):
        if not isinstance(value, int) or isinstance(value, bool):
            value = 0
        if value < -32768:
            value = -32768
        elif value > 32767:
            value = 32767
        if value < 0:
            value += 0x10000
        data[offset] = value & 0xFF
        data[offset + 1] = (value >> 8) & 0xFF

    @staticmethod
    def _put_u32(data, offset, value):
        if not isinstance(value, int) or isinstance(value, bool):
            value = 0
        value &= 0xFFFFFFFF
        data[offset] = value & 0xFF
        data[offset + 1] = (value >> 8) & 0xFF
        data[offset + 2] = (value >> 16) & 0xFF
        data[offset + 3] = (value >> 24) & 0xFF

    def record(self, frame, full_write, write_error, reverse_fresh,
               capture_ms, send_ms, position, preview_candidate,
               preview_applied, raw_control, filtered_error,
               reference_error, row_count, source_code,
               reverse_generation, candidate_diagnostic=None,
               control_diagnostic=None):
        if self.frozen:
            return False
        if frame is None or len(frame) < vision_protocol.FRAME_SIZE:
            self.record_errors += 1
            return False

        actual_valid = bool(frame[6] & vision_protocol.FLAG_VALID)
        successful_write = bool(full_write and not write_error)
        trigger = bool(not self.triggered and successful_write and
                       not actual_valid and self.last_full_valid)
        if self.triggered:
            if self.post_count >= self.POST_TRIGGER_CAPACITY:
                self.freeze("CAPACITY_REACHED")
                return False
        elif trigger:
            self.pre_count = self.count
            self.pre_oldest = (self.write_index
                               if self.pre_count == self.PRE_TRIGGER_CAPACITY
                               else 0)
            slot = self.PRE_TRIGGER_CAPACITY
        else:
            slot = self.write_index
        if self.triggered:
            slot = self.PRE_TRIGGER_CAPACITY + self.post_count

        offset = slot * self.RECORD_SIZE
        data = self.data
        sequence = frame[4] | (frame[5] << 8)
        sent_error = frame[8] | (frame[9] << 8)
        if sent_error & 0x8000:
            sent_error -= 0x10000

        flags = 1 if actual_valid else 0
        if full_write:
            flags |= 2
        if write_error:
            flags |= 4
        if reverse_fresh:
            flags |= 8
        use_viper_record = (
            _NATIVE_ACCELERATION_ENABLED and type(data) is bytearray and
            type(sequence) is int and 0 <= sequence <= 0xFFFF and
            type(capture_ms) is int and
            -0x80000000 <= capture_ms <= 0x7FFFFFFF and
            type(send_ms) is int and
            -0x80000000 <= send_ms <= 0x7FFFFFFF and
            type(position) is int and
            -0x80000000 <= position <= 0x7FFFFFFF and
            type(preview_candidate) is int and
            -0x80000000 <= preview_candidate <= 0x7FFFFFFF and
            type(preview_applied) is int and
            -0x80000000 <= preview_applied <= 0x7FFFFFFF and
            type(sent_error) is int and
            -0x80000000 <= sent_error <= 0x7FFFFFFF and
            type(raw_control) is int and
            -0x80000000 <= raw_control <= 0x7FFFFFFF and
            type(filtered_error) is int and
            -0x80000000 <= filtered_error <= 0x7FFFFFFF and
            type(reference_error) is int and
            -0x80000000 <= reference_error <= 0x7FFFFFFF and
            type(row_count) is int and
            -0x80000000 <= row_count <= 0x7FFFFFFF and
            type(source_code) is int and
            -0x80000000 <= source_code <= 0x7FFFFFFF and
            type(reverse_generation) is int and
            -0x80000000 <= reverse_generation <= 0x7FFFFFFF)
        if use_viper_record:
            _write_camera_diag_record_viper(
                data, offset, sequence, flags, capture_ms, send_ms,
                position, preview_candidate, preview_applied, sent_error,
                raw_control, filtered_error, reference_error, row_count,
                source_code, reverse_generation)
        else:
            self._put_u16(data, offset, sequence)
            data[offset + 2] = flags
            data[offset + 3] = 0
            self._put_u32(data, offset + 4, capture_ms)
            self._put_u32(data, offset + 8, send_ms)
            self._put_i16(data, offset + 12, position)
            self._put_i16(data, offset + 14, preview_candidate)
            self._put_i16(data, offset + 16, preview_applied)
            self._put_i16(data, offset + 18, sent_error)
            self._put_i16(data, offset + 20, raw_control)
            self._put_i16(data, offset + 22, filtered_error)
            self._put_i16(data, offset + 24, reference_error)
            if not isinstance(row_count, int) or isinstance(row_count, bool):
                row_count = 0
            if row_count < 0:
                row_count = 0
            elif row_count > 255:
                row_count = 255
            data[offset + 26] = row_count
            if not isinstance(source_code, int) or isinstance(source_code, bool):
                source_code = 0
            if source_code < 0 or source_code > 5:
                source_code = 0
            data[offset + 27] = source_code
            self._put_u32(data, offset + 28, reverse_generation)
        self._write_extension(slot, candidate_diagnostic)
        self._write_control_extension(
            slot, control_diagnostic, sequence, capture_ms, send_ms,
            sent_error)
        curve_offset = slot * vision_protocol.FRAME_SIZE
        for index in range(vision_protocol.FRAME_SIZE):
            self.curve_trace_data[curve_offset + index] = frame[index]

        if self.triggered:
            self.count += 1
            self.post_count += 1
        elif trigger:
            self.count = self.pre_count + 1
            self.post_count = 1
            self.triggered = True
        else:
            if self.pre_count < self.PRE_TRIGGER_CAPACITY:
                self.pre_count += 1
                self.count = self.pre_count
            else:
                self.overwritten += 1
            self.write_index += 1
            if self.write_index >= self.PRE_TRIGGER_CAPACITY:
                self.write_index = 0

        # Only successful complete UART writes participate in the transition.
        if successful_write:
            self.last_full_valid = actual_valid
        if (self.triggered and
                self.post_count >= self.POST_TRIGGER_CAPACITY):
            self.freeze("POST_COMPLETE")
        return True

    def _write_extension(self, slot, candidate_diagnostic):
        """Store bounded candidate fields beside the matching frame."""
        offset = slot * self.EXTENSION_RECORD_SIZE
        data = self.extension_data
        if candidate_diagnostic is None:
            for index in range(self.EXTENSION_RECORD_SIZE):
                data[offset + index] = 0
            data[offset + 4] = 255
            data[offset + 10] = 255
            return
        for index in range(self.EXTENSION_RECORD_SIZE):
            value = int(candidate_diagnostic[index])
            if index in (4, 10) and value < 0:
                value = 255
            elif value < 0:
                value = 0
            elif value > 255:
                value = 255
            data[offset + index] = value

    def _write_control_extension(self, slot, diagnostic, sequence,
                                 capture_ms, send_ms, sent_control):
        """Store a sequence/time-bound PD/preview record with its own CRC."""
        record_size = self.CONTROL_EXTENSION_RECORD_SIZE
        offset = slot * record_size
        data = self.control_extension_data
        if (_NATIVE_ACCELERATION_ENABLED and
                type(record_size) is int and record_size == 32 and
                type(self.CONTROL_EXTENSION_VERSION) is int and
                0 <= self.CONTROL_EXTENSION_VERSION <= 255 and
                type(sequence) is int and
                -0x80000000 <= sequence <= 0x7FFFFFFF and
                type(capture_ms) is int and
                -0x80000000 <= capture_ms <= 0x7FFFFFFF and
                type(send_ms) is int and
                -0x80000000 <= send_ms <= 0x7FFFFFFF and
                type(sent_control) is int and
                -0x80000000 <= sent_control <= 0x7FFFFFFF):
            if diagnostic is None:
                flags = 0
                position_p = 0
                derivative_d = 0
                pd_output = 0
                preview_candidate = 0
                preview_gain_milli = 0
                preview_applied = 0
                capture_dt_ms = 0xFFFF
                preview_state = 0
                reset_code = 0
                fast_values_valid = True
            elif type(diagnostic) is dict:
                profile_code = diagnostic.get("profile_code", 0)
                pd_enabled = diagnostic.get("pd_enabled")
                preview_enabled = diagnostic.get("preview_enabled")
                preview_confirmed = diagnostic.get("preview_confirmed")
                preview_geometry_valid = diagnostic.get(
                    "preview_geometry_valid")
                preview_same_sign = diagnostic.get("preview_same_sign")
                duplicate = diagnostic.get("duplicate")
                position_p = diagnostic.get("position_p", 0)
                derivative_d = diagnostic.get("derivative_d", 0)
                pd_output = diagnostic.get("pd_output", 0)
                preview_candidate = diagnostic.get("preview_candidate", 0)
                preview_gain_milli = diagnostic.get("preview_gain_milli", 0)
                preview_applied = diagnostic.get("preview_applied", 0)
                capture_dt_ms = diagnostic.get("capture_dt_ms", 0xFFFF)
                preview_state_value = diagnostic.get("preview_state", 0)
                reset_code_value = diagnostic.get("reset_code", 0)

                fast_flags_valid = (
                    (type(profile_code) is int or
                     type(profile_code) is bool) and
                    (type(pd_enabled) is int or type(pd_enabled) is bool) and
                    (type(preview_enabled) is int or
                     type(preview_enabled) is bool) and
                    (type(preview_confirmed) is int or
                     type(preview_confirmed) is bool) and
                    (type(preview_geometry_valid) is int or
                     type(preview_geometry_valid) is bool) and
                    (type(preview_same_sign) is int or
                     type(preview_same_sign) is bool) and
                    (type(duplicate) is int or type(duplicate) is bool))
                fast_values_valid = (
                    type(position_p) is int and
                    -0x80000000 <= position_p <= 0x7FFFFFFF and
                    type(derivative_d) is int and
                    -0x80000000 <= derivative_d <= 0x7FFFFFFF and
                    type(pd_output) is int and
                    -0x80000000 <= pd_output <= 0x7FFFFFFF and
                    type(preview_candidate) is int and
                    -0x80000000 <= preview_candidate <= 0x7FFFFFFF and
                    type(preview_gain_milli) is int and
                    -0x80000000 <= preview_gain_milli <= 0x7FFFFFFF and
                    type(preview_applied) is int and
                    -0x80000000 <= preview_applied <= 0x7FFFFFFF and
                    (type(capture_dt_ms) is int or
                     type(capture_dt_ms) is bool) and
                    (type(preview_state_value) is int or
                     type(preview_state_value) is bool) and
                    (type(reset_code_value) is int or
                     type(reset_code_value) is bool))
                if fast_flags_valid and fast_values_valid:
                    flags = (1 if pd_enabled else 0)
                    if preview_enabled:
                        flags |= 2
                    if preview_confirmed:
                        flags |= 4
                    if preview_geometry_valid:
                        flags |= 8
                    if preview_same_sign:
                        flags |= 16
                    if duplicate:
                        flags |= 32
                    flags |= (profile_code & 3) << 6
                    if isinstance(capture_dt_ms, bool):
                        capture_dt_ms = 0xFFFF
                    else:
                        capture_dt_ms = max(
                            0, min(0xFFFF, capture_dt_ms))
                    preview_state = int(preview_state_value)
                    reset_code = int(reset_code_value)
                else:
                    fast_values_valid = False
            else:
                fast_values_valid = False

            if fast_values_valid:
                _write_control_extension_viper(
                    data, offset, self._CRC16_NIBBLE_TABLE_ARRAY,
                    self.CONTROL_EXTENSION_VERSION, flags, sequence,
                    capture_ms, send_ms, position_p, derivative_d,
                    pd_output, preview_candidate, preview_gain_milli,
                    preview_applied, capture_dt_ms, preview_state,
                    reset_code, sent_control)
                return

        for index in range(record_size):
            data[offset + index] = 0
        data[offset] = self.CONTROL_EXTENSION_VERSION
        self._put_u16(data, offset + 2, sequence)
        self._put_u32(data, offset + 4, capture_ms)
        self._put_u32(data, offset + 8, send_ms)
        self._put_i16(data, offset + 28, sent_control)
        if diagnostic is None:
            self._put_u16(data, offset + 24, 0xFFFF)
        else:
            profile_code = diagnostic.get("profile_code", 0) & 3
            flags = (1 if diagnostic.get("pd_enabled") else 0)
            if diagnostic.get("preview_enabled"):
                flags |= 2
            if diagnostic.get("preview_confirmed"):
                flags |= 4
            if diagnostic.get("preview_geometry_valid"):
                flags |= 8
            if diagnostic.get("preview_same_sign"):
                flags |= 16
            if diagnostic.get("duplicate"):
                flags |= 32
            flags |= profile_code << 6
            data[offset + 1] = flags
            self._put_i16(data, offset + 12,
                          diagnostic.get("position_p", 0))
            self._put_i16(data, offset + 14,
                          diagnostic.get("derivative_d", 0))
            self._put_i16(data, offset + 16,
                          diagnostic.get("pd_output", 0))
            self._put_i16(data, offset + 18,
                          diagnostic.get("preview_candidate", 0))
            self._put_u16(data, offset + 20,
                          diagnostic.get("preview_gain_milli", 0))
            self._put_i16(data, offset + 22,
                          diagnostic.get("preview_applied", 0))
            capture_dt_ms = diagnostic.get("capture_dt_ms", 0xFFFF)
            if not isinstance(capture_dt_ms, int) or isinstance(
                    capture_dt_ms, bool):
                capture_dt_ms = 0xFFFF
            else:
                capture_dt_ms = max(0, min(0xFFFF, capture_dt_ms))
            self._put_u16(data, offset + 24, capture_dt_ms)
            data[offset + 26] = max(0, min(255, int(
                diagnostic.get("preview_state", 0))))
            data[offset + 27] = max(0, min(255, int(
                diagnostic.get("reset_code", 0))))
        if _NATIVE_ACCELERATION_ENABLED:
            _crc16_control_extension_viper(
                data, offset, self._CRC16_NIBBLE_TABLE_ARRAY)
        else:
            crc = 0xFFFF
            table = self._CRC16_NIBBLE_TABLE
            for index in range(record_size - 2):
                crc ^= data[offset + index] << 8
                crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
                crc = ((crc << 4) ^ table[(crc >> 12) & 0x0F]) & 0xFFFF
            self._put_u16(data, offset + record_size - 2, crc)

    def freeze(self, reason="STOP"):
        if not self.frozen:
            self.frozen = True
            self.freeze_reason = reason
        return self.frozen

    def dump_once(self, build_id):
        """Dump once after an accepted reverse RESET ends a run."""
        if self.dumped or self.count <= 0:
            return False
        self.freeze(self.freeze_reason if self.freeze_reason != "RUNNING"
                    else "AUTO_DUMP")
        self.dumped = True
        self.dump(build_id)
        return True

    @staticmethod
    def _u16(data, offset):
        return data[offset] | (data[offset + 1] << 8)

    @staticmethod
    def _i16(data, offset):
        value = data[offset] | (data[offset + 1] << 8)
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _u32(data, offset):
        return (data[offset] | (data[offset + 1] << 8) |
                (data[offset + 2] << 16) |
                (data[offset + 3] << 24))

    def dump(self, build_id):
        data = self.data
        print("STEERING_DIAG_BEGIN build=%s format=3 capacity=256 "
              "record_size=32 extension_record_size=16 "
              "control_extension_v2_record_size=32 "
              "curve_trace_v2_record_size=16 pre_capacity=64 "
              "post_capacity=192 ticks_period_ms=%d count=%d frozen=%d "
              "triggered=%d pre_count=%d post_count=%d freeze_reason=%s overwritten=%d "
              "record_errors=%d" % (
                  build_id, _STEERING_DIAG_TICKS_PERIOD_MS, self.count,
                  1 if self.frozen else 0, 1 if self.triggered else 0,
                  self.pre_count, self.post_count, self.freeze_reason, self.overwritten,
                  self.record_errors))
        print("STEERING_DIAG_CODES build=%s extension=single_edge_candidate_v2 "
              "fields=0:pair_reason;1:common_rows;2:common_span;"
              "3:max_center_delta2;4:selected_side;5:selection;"
              "6:left_support;7:right_support;8:candidate_flags;"
              "9:result_reason_code;10:pending_side;11:pending_count;"
              "12:anchor_rejects;13:corridor_rejects;14:jump_breaks;"
              "15:gap_breaks;flag_bits=1:left_valid,2:right_valid,"
              "4:pair_checked,8:selected,16:published;"
              "pair_reason=0:NONE,1:SINGLE_LEFT,2:SINGLE_RIGHT,"
              "3:SAME_LANE,4:NO_COMMON_ROWS,5:TOO_FEW_ROWS,"
              "6:SHORT_SPAN,7:CENTER_DELTA,8:BOUNDARY_ORDER;"
              "selection=0:NONE,1:PREVIOUS_LEFT,2:PREVIOUS_RIGHT,"
              "3:SUPPORT_LEFT,4:SUPPORT_RIGHT,5:BOTTOM_LEFT,"
              "6:BOTTOM_RIGHT,7:TIE_LEFT;"
              "result=0:UNKNOWN,1:NOT_RUN,2:NO_FRESH_KEEP,"
              "3:NO_WIDTH_CACHE,4:INVALID_MASK_INFO,"
              "5:FRAME_SIZE_MISMATCH,6:TOPOLOGY_BRANCH_UNRESOLVED,"
              "7:TOPOLOGY_ISLAND_UNRESOLVED,"
              "8:TOPOLOGY_AMBIGUITY_UNRESOLVED,"
              "9:TOPOLOGY_COMPONENT_BOUNDS,10:AMBIGUOUS_BOTH_SIDES,"
              "11:SIDE_SWITCH_PENDING,12:SIDE_SWITCH_NO_OVERLAP,"
              "13:SIDE_SWITCH_CENTER_MISMATCH,"
              "14:AMBIGUOUS_SINGLE_EDGE_CHAIN,"
              "15:SINGLE_EDGE_CONTINUITY_OR_SUPPORT,"
              "16:SINGLE_EDGE_SUPPORT_ROWS,17:SINGLE_EDGE_SUPPORT_SPAN,"
              "18:NO_ANCHORED_SINGLE_EDGE,"
              "19:NO_CURRENT_BOUNDARY_OR_WHITE_CLIP,"
              "20:OK_SINGLE_EDGE_LEFT,21:OK_SINGLE_EDGE_RIGHT" % build_id)
        print("STEERING_DIAG_CONTROL_EXTENSION version=2 record_size=32 "
              "timestamps=camera_ticks_ms_snapshot_complete,uart_write_return "
              "fields="
              "0:version,1:flags,2-3:sequence_u16,4-7:capture_ms_u32,"
              "8-11:send_ms_u32,12-13:position_p_i16,"
              "14-15:derivative_d_i16_clamped_component,"
              "16-17:pd_output_i16,18-19:preview_candidate_i16,"
              "20-21:preview_gain_milli_u16,22-23:preview_applied_i16,"
              "24-25:capture_dt_ms_u16,26:preview_state,27:reset_code,"
              "28-29:sent_control_i16,30-31:crc16_ccitt_false_little_endian;"
              "flag_bits=1:pd_active,2:preview_profile,4:preview_confirmed,"
              "8:geometry_valid,16:position_same_sign,32:duplicate,"
              "64-128:profile_code;profile=0:no_pd,1:pd,2:pd_preview;"
              "preview_state=0:STRAIGHT,1:CANDIDATE,2:CONFIRMED,3:EXIT;"
              "reset=0:NONE,1:FIRST,2:INVALID,3:DT_GAP,4:IDENTITY,"
              "5:REFERENCE,6:CAPTURE_GAP,7:FRAME_BUDGET,"
              "8:REVERSE_RESET,9:UART_ERROR,10:CONTROL_TIMEOUT,"
              "11:DISABLED")
        print(self._CSV_HEADER)
        for record_index in range(self.count):
            if self.triggered:
                if record_index < self.pre_count:
                    slot = self.pre_oldest + record_index
                    if slot >= self.PRE_TRIGGER_CAPACITY:
                        slot -= self.PRE_TRIGGER_CAPACITY
                else:
                    slot = self.PRE_TRIGGER_CAPACITY + \
                        record_index - self.pre_count
            else:
                oldest = self.write_index - self.pre_count
                if oldest < 0:
                    oldest += self.PRE_TRIGGER_CAPACITY
                slot = oldest + record_index
                if slot >= self.PRE_TRIGGER_CAPACITY:
                    slot -= self.PRE_TRIGGER_CAPACITY
            offset = slot * self.RECORD_SIZE
            flags = data[offset + 2]
            base_text = ("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d" % (
                self._u16(data, offset),
                self._u32(data, offset + 4),
                self._u32(data, offset + 8),
                flags & 1,
                (flags >> 1) & 1,
                (flags >> 2) & 1,
                (flags >> 3) & 1,
                self._i16(data, offset + 12),
                self._i16(data, offset + 14),
                self._i16(data, offset + 16),
                self._i16(data, offset + 18),
                self._i16(data, offset + 20),
                self._i16(data, offset + 22),
                self._i16(data, offset + 24),
                data[offset + 26], data[offset + 27],
                self._u32(data, offset + 28)))
            extension_offset = slot * self.EXTENSION_RECORD_SIZE
            extension = self.extension_data
            extension_values = []
            for field_index in range(self.EXTENSION_RECORD_SIZE):
                value = extension[extension_offset + field_index]
                if field_index == 4 and value == 255:
                    value = -1
                elif field_index == 10 and value == 255:
                    value = -1
                extension_values.append(str(value))
            extension_text = ",".join(extension_values)
            control_offset = slot * self.CONTROL_EXTENSION_RECORD_SIZE
            control_data = self.control_extension_data
            control_text = "".join(
                "%02X" % control_data[control_offset + index]
                for index in range(self.CONTROL_EXTENSION_RECORD_SIZE))
            curve_offset = slot * vision_protocol.FRAME_SIZE
            curve_text = "".join(
                "%02X" % self.curve_trace_data[curve_offset + index]
                for index in range(vision_protocol.FRAME_SIZE))
            print(base_text + "," + extension_text + "," + control_text +
                  "," + curve_text)
        print("STEERING_DIAG_END")


def _make_steering_diag(enabled):
    if enabled is True:
        return _SteeringDiagBuffer()
    return None


class _SteeringDiagSession:
    """Track only a fresh, recognized H743 Follow session for diagnostics."""

    def __init__(self, diagnostic_buffer):
        self.diagnostic_buffer = diagnostic_buffer
        self.last_accepted_resets = 0
        self.last_accepted_keeps = 0
        self.session_active = False

    def observe(self, parser, tracker, build_id, reverse_fresh=False,
                now_ms=None, ticks_diff=None):
        accepted_resets = int(parser.accepted_resets)
        reset_changed = accepted_resets != self.last_accepted_resets
        accepted_keeps = int(parser.accepted_keeps)
        self.last_accepted_keeps = accepted_keeps
        profile_id = 0
        profile_fresh = False
        if now_ms is not None and callable(ticks_diff):
            try:
                profile_id = parser.profile_ack(now_ms, ticks_diff)
                profile_fresh = profile_id in (1, 2, 3)
            except Exception:
                profile_id = 0
                profile_fresh = False
        actual_follow = bool(
            reverse_fresh and parser.single_enabled and profile_fresh and
            parser.profile_session_active and parser.session_token != 0)
        stop_reason = None
        if self.session_active and reset_changed:
            stop_reason = "REVERSE_RESET"
        elif (self.session_active and profile_fresh and
              not parser.profile_session_active):
            stop_reason = "PROFILE_INACTIVE"
        if stop_reason is not None:
            diagnostic = self.diagnostic_buffer
            if diagnostic is not None and not diagnostic.frozen:
                if diagnostic.count > 0:
                    diagnostic.freeze(stop_reason)
                    freeze_snapshot = getattr(
                        tracker, "freeze_diagnostic_snapshot", None)
                    if callable(freeze_snapshot):
                        freeze_snapshot()
                else:
                    diagnostic.rearm()
            self.session_active = False

        if actual_follow and not self.session_active:
            diagnostic = self.diagnostic_buffer
            if diagnostic is not None:
                diagnostic.rearm()
            self.session_active = True

        self.last_accepted_resets = accepted_resets
        return stop_reason


def _ticks_period_ms(time_module):
    ticks_add = getattr(time_module, "ticks_add", None)
    if not callable(ticks_add):
        return 0
    try:
        period = ticks_add(0, -1) + 1
    except Exception:
        return 0
    if not isinstance(period, int) or isinstance(period, bool) or period <= 0:
        return 0
    return period


def _steering_diag_source_code(source):
    if source == "road":
        return 1
    if source == "single_left":
        return 2
    if source == "single_right":
        return 3
    if source == "ring_left":
        return 4
    if source == "ring_right":
        return 5
    return 0


def _ring_calibration_gate_blocked(profile_id, calibration_passed):
    """Keep ring calibration gating limited to the two ring profiles."""
    return bool(profile_id in config.RING_ENABLED_PROFILE_IDS and
                not calibration_passed)


_STEERING_DIAG = _make_steering_diag(
    getattr(config, "STEERING_DIAG_ENABLED", False))
_STEERING_DIAG_TICKS_PERIOD_MS = (
    _ticks_period_ms(time) if _STEERING_DIAG is not None else 0)


def steering_diag_dump():
    """Print retained frames after stopping the camera script at the REPL."""
    if _STEERING_DIAG is None:
        print("STEERING_DIAG_DISABLED build=%s enabled=0" % config.BUILD_ID)
        return False
    _STEERING_DIAG.dump(config.BUILD_ID)
    return True


def steering_diag_rearm():
    """Clear diagnostic metadata after the camera script has been stopped."""
    if _STEERING_DIAG is None:
        print("STEERING_DIAG_DISABLED build=%s enabled=0" % config.BUILD_ID)
        return False
    _STEERING_DIAG.rearm()
    return True


def ring_diag_freeze(reason="EXPLICIT_STOP"):
    """Freeze ring history after the camera loop has stopped at the REPL."""
    diagnostic = _RING_DIAGNOSTICS
    if diagnostic is None:
        print("RING_DIAG_DISABLED build=%s" % config.TRACK_PACKAGE_BUILD_ID)
        return False
    if diagnostic.frozen:
        return True
    return diagnostic.freeze(reason)


def ring_diag_dump():
    """Print only the bounded ring-history summary after stopping the loop."""
    diagnostic = _RING_DIAGNOSTICS
    if diagnostic is None:
        print("RING_DIAG_DISABLED build=%s" % config.TRACK_PACKAGE_BUILD_ID)
        return False
    if not diagnostic.frozen:
        print("RING_DIAG_NOT_FROZEN build=%s" %
              config.TRACK_PACKAGE_BUILD_ID)
        return False
    for line in diagnostic.summary_lines(config.TRACK_PACKAGE_BUILD_ID):
        print(line)
    return True


def ring_diag_persist():
    """Retry one frozen run using a new numbered file before power removal."""
    diagnostic = _RING_DIAGNOSTICS
    if diagnostic is None:
        print("RING_DIAG_DISABLED build=%s" % config.TRACK_PACKAGE_BUILD_ID)
        return False
    if not diagnostic.frozen:
        print("RING_DIAG_NOT_FROZEN build=%s" %
              config.TRACK_PACKAGE_BUILD_ID)
        return False
    if diagnostic.persisted:
        return True
    try:
        path = diagnostic.save_next(
            config.RING_DIAGNOSTICS_PATH,
            config.TRACK_PACKAGE_BUILD_ID)
    except Exception as exc:
        session = _RING_DIAG_SESSION
        if session is not None:
            session.blocked = True
        print("RING_DIAG_PERSIST_FAILED error=%s:%s" % (
            type(exc).__name__, str(exc)))
        return False
    session = _RING_DIAG_SESSION
    if session is not None:
        session.last_saved_path = path
        session.blocked = False
    print("RING_DIAG_PERSISTED path=%s episodes=%d crc32=1" % (
        path, diagnostic.episode_count))
    return True


class _RingDiagnosticsSession:
    """Gate recording and persistence on real Follow start/stop edges."""

    def __init__(self, diagnostic, base_path, build_id):
        self.diagnostic = diagnostic
        self.base_path = base_path
        self.build_id = build_id
        self.active = False
        self.blocked = False
        self.last_saved_path = None
        self.last_error = ""

    def observe(self, now_ms, sequence, profile_id, active_follow,
                observation_valid, selection, tracker_result, stop_reason,
                ticks_diff=None):
        diagnostic = self.diagnostic
        if diagnostic is None or self.blocked:
            return False
        if active_follow and not self.active:
            if diagnostic.frozen:
                if not diagnostic.persisted or not diagnostic.rearm():
                    self.blocked = True
                    return False
            try:
                diagnostic.begin_session(now_ms, ticks_diff)
            except Exception as exc:
                self.blocked = True
                self.last_error = "%s:%s" % (type(exc).__name__, str(exc))
                return False
            self.active = True
        if not self.active:
            return False
        diagnostic.record_frame(
            now_ms, sequence, profile_id, active_follow,
            observation_valid, selection, tracker_result, ticks_diff)
        if stop_reason is None:
            return True
        diagnostic.freeze(stop_reason)
        self.active = False
        try:
            path = diagnostic.save_next(self.base_path, self.build_id)
        except Exception as exc:
            self.blocked = True
            self.last_error = "%s:%s" % (type(exc).__name__, str(exc))
            print("RING_DIAG_PERSIST_FAILED error=%s" % self.last_error)
            return False
        self.last_saved_path = path
        print("RING_DIAG_PERSISTED path=%s episodes=%d crc32=1" % (
            path, diagnostic.episode_count))
        return True


# SINGLE_EDGE_EXT_DIAG_BEGIN
def single_edge_diag_dump():
    """Print aggregate single-edge counters after stopping the camera loop."""
    tracker = (fast_single_edge.last_tracker()
               if fast_single_edge is not None else None)
    if tracker is None:
        print("SINGLE_EDGE_EXT_DIAG_UNAVAILABLE build=%s" % config.BUILD_ID)
        return False
    return tracker.dump_extended_diagnostics(config.BUILD_ID)


def single_edge_diag_rearm():
    """Reset single-edge counters without changing tracker or camera state."""
    tracker = (fast_single_edge.last_tracker()
               if fast_single_edge is not None else None)
    if tracker is None:
        print("SINGLE_EDGE_EXT_DIAG_UNAVAILABLE build=%s" % config.BUILD_ID)
        return False
    return tracker.rearm_extended_diagnostics()
# SINGLE_EDGE_EXT_DIAG_END


_V6_RUNTIME_MODULES = ()
_V7_RUNTIME_MODULES = ()
_SINGLEEDGE_RUNTIME_MODULES = ()
_CONTOUR_RUNTIME_MODULES = ()


def _load_v6_runtime():
    global _V6_RUNTIME_MODULES
    if _V6_RUNTIME_MODULES:
        return _V6_RUNTIME_MODULES
    try:
        _V6_MODULE_CACHE["grayscale_adapter"] = __import__("grayscale_adapter")
        _V6_MODULE_CACHE["thresholding"] = __import__("thresholding")
        _V6_MODULE_CACHE["road_detector"] = __import__("road_detector")
        _V6_MODULE_CACHE["surface_detector"] = __import__("surface_detector")
        _V6_MODULE_CACHE["source_selector"] = __import__("source_selector")
        _V6_MODULE_CACHE["boundary_evidence"] = __import__("boundary_evidence")
    except ImportError as exc:
        raise ImportError(
            "V6 diagnostic bundle is incomplete (%s); install matching V6 files" % exc)
    _V6_RUNTIME_MODULES = (
        ("grayscale_adapter.py", _V6_MODULE_CACHE["grayscale_adapter"]),
        ("thresholding.py", _V6_MODULE_CACHE["thresholding"]),
        ("road_detector.py", _V6_MODULE_CACHE["road_detector"]),
        ("source_selector.py", _V6_MODULE_CACHE["source_selector"]),
        ("surface_detector.py", _V6_MODULE_CACHE["surface_detector"]),
        ("boundary_evidence.py", _V6_MODULE_CACHE["boundary_evidence"]),
    )
    return _V6_RUNTIME_MODULES


def _load_fast_runtime():
    global fast_gray, fast_path, fast_topology, fast_boundary
    global _V7_RUNTIME_MODULES
    try:
        if fast_gray is None:
            fast_gray = __import__("fast_gray")
        if fast_topology is None:
            fast_topology = __import__("fast_topology")
        if fast_path is None:
            fast_path = __import__("fast_path")
        if fast_boundary is None:
            fast_boundary = __import__("fast_boundary")
    except ImportError as exc:
        raise ImportError(
            "V7 bundle mismatch: fast grayscale/path/topology/boundary modules "
            "is missing or cannot be imported (%s)" % exc)
    _V7_RUNTIME_MODULES = (
        ("fast_gray.py", fast_gray),
        ("fast_path.py", fast_path),
        ("fast_topology.py", fast_topology),
        ("fast_boundary.py", fast_boundary),
    )
    return _V7_RUNTIME_MODULES


def _load_contour_runtime():
    """Load and validate the bounded contour-route and trace modules."""
    global contour_route, contour_trace, _CONTOUR_RUNTIME_MODULES
    try:
        if contour_route is None:
            contour_route = __import__("contour_route")
        if contour_trace is None:
            contour_trace = __import__("contour_trace")
    except ImportError as exc:
        raise ImportError(
            "contour recovery bundle is incomplete (%s); install matching "
            "contour_route.py and contour_trace.py" % exc)
    _CONTOUR_RUNTIME_MODULES = (
        ("contour_route.py", contour_route),
        ("contour_trace.py", contour_trace),
    )
    for filename, module in _CONTOUR_RUNTIME_MODULES:
        version = getattr(module, "RUNTIME_API_VERSION", None)
        backend = getattr(module, "COMPILER_BACKEND", None)
        if version != CONTOUR_RUNTIME_API_VERSION:
            raise RuntimeError(
                "contour bundle mismatch: %s API is %s, expected %d" % (
                    filename, version, CONTOUR_RUNTIME_API_VERSION))
        if backend not in ("python-bounded", "micropython-native-viper"):
            raise RuntimeError(
                "contour bundle mismatch: %s backend is %s, expected "
                "python-bounded or micropython-native-viper" % (
                    filename, backend))
    return _CONTOUR_RUNTIME_MODULES


def _load_singleedge_runtime():
    global fast_single_edge, reverse_control, _SINGLEEDGE_RUNTIME_MODULES
    try:
        if fast_single_edge is None:
            fast_single_edge = __import__("fast_single_edge")
        if reverse_control is None:
            reverse_control = __import__("reverse_control")
    except ImportError as exc:
        raise ImportError(
            "single-edge bundle mismatch: fast_single_edge/reverse_control "
            "is missing or cannot be imported (%s)" % exc)
    _SINGLEEDGE_RUNTIME_MODULES = (
        ("fast_single_edge.py", fast_single_edge),
        ("reverse_control.py", reverse_control),
    )
    if getattr(fast_single_edge, "RING_OBSERVATION_API_VERSION", None) != 2:
        raise ImportError("single-edge bundle mismatch: ring observation API 2 required")
    return _SINGLEEDGE_RUNTIME_MODULES


def _load_track_elements_runtime():
    """Load the bounded element observer used by the V7 camera loop."""
    global track_elements
    try:
        if track_elements is None:
            track_elements = __import__("track_elements")
    except ImportError as exc:
        raise ImportError(
            "V7 element bundle mismatch: track_elements.py is missing or "
            "cannot be imported (%s)" % exc)
    if getattr(track_elements, "RUNTIME_API_VERSION", None) != \
            TRACK_ELEMENTS_RUNTIME_API_VERSION:
        raise RuntimeError(
            "V7 element bundle mismatch: track_elements.py API is %s, "
            "expected %d" % (
                getattr(track_elements, "RUNTIME_API_VERSION", None),
                TRACK_ELEMENTS_RUNTIME_API_VERSION))
    if getattr(track_elements, "RING_OUTER_ROUTE_API_VERSION", None) != 2:
        raise RuntimeError("V7 element bundle mismatch: ring outer-route API 2 required")
    if getattr(track_elements, "RING_ISLAND_TOPOLOGY_API_VERSION", None) != 1:
        raise RuntimeError("V7 element bundle mismatch: ring island-topology API 1 required")
    if getattr(__import__("fast_gray"), "RING_DARK_BLOCK_API_VERSION", None) != 1:
        raise RuntimeError("V7 element bundle mismatch: ring dark-block API 1 required")
    return track_elements


def _load_ring_runtime():
    """Load the bounded width-calibration and stopped-run diagnostic files."""
    global ring_width_calibration, ring_diagnostics
    try:
        if ring_width_calibration is None:
            ring_width_calibration = __import__("ring_width_calibration")
        if ring_diagnostics is None:
            ring_diagnostics = __import__("ring_diagnostics")
    except ImportError as exc:
        raise ImportError(
            "ring runtime bundle is incomplete (%s); install matching "
            "ring_width_calibration.py and ring_diagnostics.py" % exc)
    if getattr(ring_width_calibration, "RUNTIME_API_VERSION", None) != \
            RING_WIDTH_CALIBRATION_RUNTIME_API_VERSION:
        raise RuntimeError(
            "ring runtime bundle mismatch: ring_width_calibration.py API "
            "is %s, expected %d" % (
                getattr(ring_width_calibration, "RUNTIME_API_VERSION", None),
                RING_WIDTH_CALIBRATION_RUNTIME_API_VERSION))
    if not callable(getattr(ring_diagnostics, "RingDiagnostics", None)):
        raise RuntimeError(
            "ring runtime bundle mismatch: ring_diagnostics.py API is missing")
    return ring_width_calibration, ring_diagnostics


class _FreshObservationFilter:
    """Small V7-only equivalent of the legacy fresh-frame EMA filter."""

    def __init__(self, alpha, max_gap_ms, min_confidence):
        self.alpha = float(alpha)
        self.max_gap_ms = int(max_gap_ms)
        self.min_confidence = int(min_confidence)
        self.value = None
        self.last_ms = None

    def reset(self):
        self.value = None
        self.last_ms = None

    def update(self, now_ms, valid, confidence, raw_value, ticks_diff):
        if not valid or confidence < self.min_confidence:
            self.reset()
            return 0
        if self.last_ms is not None:
            delta_ms = ticks_diff(now_ms, self.last_ms)
            if delta_ms == 0:
                if self.value is None:
                    return 0
                return (int(self.value + 0.5) if self.value >= 0.0 else
                        -int(-self.value + 0.5))
            if delta_ms < 0 or delta_ms > self.max_gap_ms:
                self.reset()
        if self.value is None:
            self.value = float(raw_value)
        else:
            self.value = (self.alpha * float(raw_value) +
                          (1.0 - self.alpha) * self.value)
        self.last_ms = now_ms
        if self.value >= 0.0:
            return int(self.value + 0.5)
        return -int(-self.value + 0.5)


_CONTROL_RESET_CODES = {
    "NONE": 0,
    "FIRST": 1,
    "INVALID": 2,
    "DT_GAP": 3,
    "IDENTITY": 4,
    "REFERENCE": 5,
    "CAPTURE_GAP": 6,
    "FRAME_BUDGET": 7,
    "REVERSE_RESET": 8,
    "UART_ERROR": 9,
    "CONTROL_TIMEOUT": 10,
    "DISABLED": 11,
    "PROFILE_SWITCH": 12,
    "SESSION_CHANGE": 13,
    "PROFILE_TIMEOUT": 14,
}


class _FastPositionPD:
    """Position PD workspace driven by the existing position EMA output."""

    def __init__(self, settings):
        self.tau_ms = int(settings.POSITION_PD_DERIVATIVE_TAU_MS)
        self.td_ms = int(settings.POSITION_PD_TIME_MS)
        self.d_limit = int(settings.POSITION_PD_D_LIMIT)
        self.max_dt_ms = int(settings.POSITION_PD_MAX_DT_MS)
        self.output_limit = int(settings.STEERING_OUTPUT_LIMIT)
        self.last_ms = None
        self.last_position = None
        self.filtered_rate = 0.0
        self.last_output = 0
        self.last_derivative = 0
        self.source = None
        self.boundary_source = None
        self.reference_kind = None
        self.reference_y = None
        self.pending_reset_code = _CONTROL_RESET_CODES["FIRST"]
        self.info = {
            "position_p": 0,
            "derivative_d": 0,
            "pd_output": 0,
            "preview_candidate": 0,
            "preview_gain_milli": 0,
            "preview_applied": 0,
            "capture_dt_ms": 0xFFFF,
            "reset_code": _CONTROL_RESET_CODES["FIRST"],
            "profile_code": 1,
            "pd_enabled": True,
            "preview_enabled": False,
            "preview_confirmed": False,
            "preview_geometry_valid": False,
            "preview_same_sign": False,
            "preview_state": 0,
            "duplicate": False,
        }

    def _clear_state(self, reason):
        self.last_ms = None
        self.last_position = None
        self.filtered_rate = 0.0
        self.last_output = 0
        self.last_derivative = 0
        self.source = None
        self.boundary_source = None
        self.reference_kind = None
        self.reference_y = None
        self.pending_reset_code = _CONTROL_RESET_CODES.get(
            reason, _CONTROL_RESET_CODES["IDENTITY"])

    def reset(self, reason):
        self._clear_state(reason)
        info = self.info
        info["position_p"] = 0
        info["derivative_d"] = 0
        info["pd_output"] = 0
        info["capture_dt_ms"] = 0xFFFF
        info["reset_code"] = self.pending_reset_code
        info["duplicate"] = False

    @micropython.native
    def update(self, capture_ms, capture_dt_ms, valid, position,
               source, boundary_source, reference_kind, reference_y,
               profile, ticks_diff):
        info = self.info
        profile_code = (0 if profile == "no_pd" else
                        2 if profile in (
                            "pd_preview", "geometry_perf_only",
                            "coordinated") else 1)
        pd_enabled = profile_code != 0
        info["profile_code"] = profile_code
        info["pd_enabled"] = pd_enabled
        info["preview_enabled"] = profile_code == 2
        info["preview_candidate"] = 0
        info["preview_gain_milli"] = 0
        info["preview_applied"] = 0
        info["preview_confirmed"] = False
        info["preview_geometry_valid"] = False
        info["preview_same_sign"] = False
        info["duplicate"] = False

        if not valid:
            if self.pending_reset_code in (
                    _CONTROL_RESET_CODES["NONE"],
                    _CONTROL_RESET_CODES["FIRST"]):
                self._clear_state("INVALID")
            info["position_p"] = 0
            info["derivative_d"] = 0
            info["pd_output"] = 0
            info["capture_dt_ms"] = (0xFFFF if capture_dt_ms is None else
                                     max(0, min(0xFFFE, capture_dt_ms)))
            info["reset_code"] = self.pending_reset_code
            return info

        was_duplicate = capture_dt_ms == 0
        reset_reason = None
        if self.last_ms is not None:
            if (source != self.source or
                    boundary_source != self.boundary_source or
                    reference_kind != self.reference_kind):
                reset_reason = "IDENTITY"
            elif reference_y != self.reference_y:
                reset_reason = "REFERENCE"
        if reset_reason is not None:
            self._clear_state(reset_reason)

        dt_ms = capture_dt_ms
        if dt_ms is not None and dt_ms < 0:
            self._clear_state("DT_GAP")
            dt_ms = None
        elif dt_ms is not None and dt_ms > self.max_dt_ms:
            self._clear_state("DT_GAP")

        p_term = int(position)
        if p_term < -self.output_limit:
            p_term = -self.output_limit
        elif p_term > self.output_limit:
            p_term = self.output_limit

        # A repeated capture must not change the EMA history or derivative
        # state. Preserve the output already paired with that capture unless a
        # newly detected identity requires a safe zero-derivative restart.
        if was_duplicate:
            info["position_p"] = p_term
            if self.last_ms is None:
                control = p_term
                self.last_output = control
            else:
                control = self.last_output
            info["derivative_d"] = self.last_derivative
            info["pd_output"] = control
            info["capture_dt_ms"] = 0
            info["reset_code"] = self.pending_reset_code
            info["duplicate"] = True
            return info

        reset_code = self.pending_reset_code
        derivative = 0
        if (pd_enabled and self.last_ms is not None and
                dt_ms is not None and dt_ms > 0):
            delta_position = p_term - self.last_position
            measured_rate = delta_position * 1000.0 / dt_ms
            alpha = self.tau_ms / float(self.tau_ms + dt_ms)
            self.filtered_rate = (alpha * self.filtered_rate +
                                  (1.0 - alpha) * measured_rate)
            derivative = _rounded_local(
                self.filtered_rate * self.td_ms / 1000.0)
            if derivative < -self.d_limit:
                derivative = -self.d_limit
            elif derivative > self.d_limit:
                derivative = self.d_limit
        else:
            self.filtered_rate = 0.0

        control = p_term + derivative if pd_enabled else p_term
        if control < -self.output_limit:
            control = -self.output_limit
        elif control > self.output_limit:
            control = self.output_limit

        self.last_ms = capture_ms
        self.last_position = p_term
        self.source = source
        self.boundary_source = boundary_source
        self.reference_kind = reference_kind
        self.reference_y = reference_y
        self.last_output = control
        self.last_derivative = derivative if pd_enabled else 0
        self.pending_reset_code = _CONTROL_RESET_CODES["NONE"]

        info["position_p"] = p_term
        info["derivative_d"] = self.last_derivative
        info["pd_output"] = control
        info["capture_dt_ms"] = (0xFFFF if dt_ms is None else
                                 max(0, min(0xFFFE, dt_ms)))
        info["reset_code"] = reset_code
        return info


class _FastAdaptivePreview:
    """Independent confirmation, gain, cap, and slew state for preview."""

    def __init__(self, settings):
        self.min_candidate = int(settings.PREVIEW_CANDIDATE_MIN_ABS)
        self.confirm_frames = int(settings.PREVIEW_CONFIRM_FRAMES)
        self.confirm_ms = int(settings.PREVIEW_CONFIRM_MS)
        self.position_min = int(settings.PREVIEW_POSITION_MIN_ABS)
        self.position_full_gain = int(
            settings.PREVIEW_POSITION_FULL_GAIN_ABS)
        self.max_gain = float(settings.PREVIEW_MAX_GAIN)
        self.applied_limit = int(settings.PREVIEW_APPLIED_LIMIT)
        self.slew_per_second = int(settings.PREVIEW_SLEW_PER_SECOND)
        self.max_dt_ms = int(settings.POSITION_PD_MAX_DT_MS)
        self.last_ms = None
        self.started_ms = None
        self.stable_sign = 0
        self.stable_count = 0
        self.confirmed = False
        self.applied = 0
        self.source = None
        self.boundary_source = None
        self.reference_kind = None
        self.reference_y = None
        self.exit_active = False

    def reset(self):
        self.last_ms = None
        self.started_ms = None
        self.stable_sign = 0
        self.stable_count = 0
        self.confirmed = False
        self.applied = 0
        self.source = None
        self.boundary_source = None
        self.reference_kind = None
        self.reference_y = None
        self.exit_active = False

    @micropython.native
    def update(self, capture_ms, capture_dt_ms, valid, enabled,
               source, boundary_source, reference_kind, reference_y,
               geometry_valid, candidate, position, ticks_diff, info):
        info["preview_candidate"] = int(candidate)
        info["preview_geometry_valid"] = bool(geometry_valid)
        info["preview_same_sign"] = bool(
            candidate != 0 and position != 0 and
            ((candidate > 0) == (position > 0)))
        info["preview_state"] = 0
        info["preview_enabled"] = bool(enabled)
        if not valid:
            self.reset()
            info["preview_gain_milli"] = 0
            info["preview_applied"] = 0
            info["preview_confirmed"] = False
            info["preview_state"] = 0
            return 0

        was_duplicate = capture_dt_ms == 0 and self.last_ms is not None

        identity_changed = (self.last_ms is not None and
                            (source != self.source or
                             boundary_source != self.boundary_source or
                             reference_kind != self.reference_kind))
        reference_changed = (self.last_ms is not None and
                             reference_y != self.reference_y)
        if identity_changed or reference_changed:
            self.reset()

        if not enabled:
            self.reset()
            info["preview_gain_milli"] = 0
            info["preview_applied"] = 0
            info["preview_confirmed"] = False
            info["preview_state"] = 0
            return 0

        dt_ms = capture_dt_ms
        if dt_ms is not None and (dt_ms < 0 or dt_ms > self.max_dt_ms):
            self.reset()
            dt_ms = None

        # Preserve the preview confirmation and slew state across a duplicate
        # sensor timestamp; the duplicate contributes no extra confirmation.
        if was_duplicate:
            info["preview_gain_milli"] = self._gain_milli(
                abs(position) if self.confirmed else 0)
            info["preview_applied"] = self.applied
            info["preview_confirmed"] = self.confirmed
            info["preview_state"] = (2 if self.confirmed else
                                     3 if self.exit_active and self.applied
                                     else 1 if self.stable_count else 0)
            return self.applied

        was_confirmed = self.confirmed
        candidate_stable = bool(
            geometry_valid and abs(candidate) >= self.min_candidate)
        if candidate_stable:
            candidate_sign = 1 if candidate > 0 else -1
            if candidate_sign != self.stable_sign:
                self.stable_sign = candidate_sign
                self.stable_count = 1
                self.started_ms = capture_ms
            else:
                self.stable_count += 1
        else:
            self.stable_sign = 0
            self.stable_count = 0
            self.started_ms = None

        position_ok = bool(
            abs(position) >= self.position_min and candidate != 0 and
            ((candidate > 0) == (position > 0)))
        confirmed = bool(candidate_stable and position_ok and
                         self.stable_count >= self.confirm_frames and
                         self.started_ms is not None and
                         ticks_diff(capture_ms, self.started_ms) >=
                         self.confirm_ms)
        self.confirmed = confirmed
        gain = 0.0
        target = 0
        if confirmed:
            magnitude = abs(position)
            progress = (magnitude - self.position_min) / float(
                self.position_full_gain - self.position_min)
            if progress < 0.0:
                progress = 0.0
            elif progress > 1.0:
                progress = 1.0
            gain = progress * self.max_gain
            target = _rounded_local(candidate * gain)
            if target < -self.applied_limit:
                target = -self.applied_limit
            elif target > self.applied_limit:
                target = self.applied_limit

        if self.last_ms is not None and dt_ms is not None and dt_ms > 0:
            max_step = self.slew_per_second * dt_ms // 1000
            delta = target - self.applied
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            self.applied += delta
        elif self.last_ms is None:
            # Start from zero; there is no elapsed interval to slew through.
            self.applied = 0

        self.exit_active = bool(
            (was_confirmed and not confirmed) or
            (not confirmed and self.applied != 0))

        self.last_ms = capture_ms
        self.source = source
        self.boundary_source = boundary_source
        self.reference_kind = reference_kind
        self.reference_y = reference_y
        info["preview_gain_milli"] = int(gain * 1000.0 + 0.5)
        info["preview_applied"] = self.applied
        info["preview_confirmed"] = confirmed
        info["preview_state"] = (2 if confirmed else
                                 3 if self.exit_active else
                                 1 if candidate_stable else 0)
        return self.applied

    def _gain_milli(self, magnitude):
        if magnitude <= self.position_min:
            return 0
        if magnitude >= self.position_full_gain:
            return int(self.max_gain * 1000.0 + 0.5)
        return int((magnitude - self.position_min) *
                   self.max_gain * 1000.0 /
                   (self.position_full_gain - self.position_min) + 0.5)


class _FastSpeedStage:
    """Independent candidate/turn slowdown latch for the H743 V2 request."""

    def __init__(self, settings):
        self.min_candidate = settings.SPEED_CANDIDATE_MIN_ABS
        self.confirm_frames = settings.SPEED_CANDIDATE_CONFIRM_FRAMES
        self.confirm_ms = settings.SPEED_CANDIDATE_CONFIRM_MS
        self.release_position_abs = settings.SPEED_RELEASE_POSITION_ABS
        self.release_candidate_abs = settings.SPEED_RELEASE_CANDIDATE_ABS
        self.release_ms = settings.SPEED_RELEASE_CONFIRM_MS
        self.exit_phase_ms = settings.SPEED_EXIT_PHASE_MS
        self.request_permille = settings.SPEED_SLOWDOWN_REQUEST_PERMILLE
        self.reset()

    def reset(self):
        self.pending_sign = 0
        self.pending_count = 0
        self.pending_start_ms = None
        self.active = False
        self.turn_seen = False
        self.active_sign = 0
        self.low_start_ms = None
        self.exit_start_ms = None
        self.last_fresh_ms = None
        self.last_frame_token = None

    def lifecycle_event(self, event):
        if event in ("STOP", "RESET"):
            self.reset()

    @micropython.native
    def update(self, now_ms, valid, position, candidate, candidate_valid,
               preview_state, ticks_diff, frame_token=None):
        """Return ``(curve_phase, slowdown_permille)`` for this V2 frame.

        Candidate debounce depends only on fresh geometry and candidate sign;
        the position sign is used only by the separate preview controller.
        Invalid observations publish zeros while preserving the slowdown latch.
        """
        duplicate_frame = (frame_token is not None and
                           frame_token == self.last_frame_token)
        if not duplicate_frame:
            elapsed = (None if self.last_fresh_ms is None else
                       ticks_diff(now_ms, self.last_fresh_ms))
            if elapsed is not None and (elapsed < 0 or elapsed > 100):
                self.pending_sign = 0
                self.pending_count = 0
                self.pending_start_ms = None
                self.low_start_ms = None
            self.last_fresh_ms = now_ms
            if frame_token is not None:
                self.last_frame_token = frame_token

        if not valid:
            self.pending_sign = 0
            self.pending_count = 0
            self.pending_start_ms = None
            self.low_start_ms = None
            return 0, 0

        if duplicate_frame:
            if self.active:
                return (2 if self.turn_seen else 1), self.request_permille
            if self.pending_sign:
                return 1, 0
            if self.exit_start_ms is not None and \
                    ticks_diff(now_ms, self.exit_start_ms) < self.exit_phase_ms:
                return 3, 0
            return 0, 0

        candidate_magnitude = abs(candidate) if candidate_valid else 0
        candidate_qualifies = bool(
            candidate_valid and candidate_magnitude >= self.min_candidate)
        if candidate_qualifies and not duplicate_frame:
            sign = 1 if candidate > 0 else -1
            if sign != self.pending_sign:
                self.pending_sign = sign
                self.pending_count = 1
                self.pending_start_ms = now_ms
            else:
                self.pending_count += 1
            if (self.pending_count >= self.confirm_frames and
                    self.pending_start_ms is not None and
                    ticks_diff(now_ms, self.pending_start_ms) >=
                    self.confirm_ms):
                if not self.active:
                    self.active = True
                    self.active_sign = sign
                    self.turn_seen = False
                    self.low_start_ms = None
                    self.exit_start_ms = None
        elif not candidate_qualifies:
            self.pending_sign = 0
            self.pending_count = 0
            self.pending_start_ms = None

        if self.active:
            if preview_state == 2 and not duplicate_frame:
                self.turn_seen = True
            can_release = (candidate_valid and
                           abs(position) < self.release_position_abs and
                           candidate_magnitude < self.release_candidate_abs)
            if can_release and not duplicate_frame:
                if self.low_start_ms is None:
                    self.low_start_ms = now_ms
                elif ticks_diff(now_ms, self.low_start_ms) >= self.release_ms:
                    self.active = False
                    self.active_sign = 0
                    self.turn_seen = False
                    self.low_start_ms = None
                    self.exit_start_ms = now_ms
                    return 3, 0
            else:
                self.low_start_ms = None
            return (2 if self.turn_seen else 1), self.request_permille

        if candidate_qualifies:
            self.exit_start_ms = None
            return 1, 0
        if self.exit_start_ms is not None:
            if ticks_diff(now_ms, self.exit_start_ms) < self.exit_phase_ms:
                return 3, 0
            self.exit_start_ms = None
        return 0, 0


class _FastRowWeightWorkspace:
    """Preallocated same-frame row aggregation for the fast controller path.

    Accepted rows use the configured fixed-point parabolic weight. Bilateral
    rows must carry the boundary validator's per-row pair bit; single rows
    arrive only from the tracker's accepted row_status==1 output.
    """

    _PREVIEW_FLOOR_Y = 18
    _PREVIEW_REFERENCE_TOLERANCE = 4
    _PREVIEW_MIN_ROWS = 15
    _PREVIEW_MIN_SPAN = 12
    _PREVIEW_ENDPOINT_ROWS = 5

    def __init__(self):
        self._center_x2 = array("i", [0] * 60)
        self._row_y = array("i", [0] * 60)
        self._weight_q = array("i", [0] * 60)
        self._weight_q_source = None
        self._weight_q_height = 0
        self._weight_q_kernel_ready = False
        self._weight_sums = array("i", [0] * 3)
        self._row_metrics = array("i", [0] * 10)
        self._preview_y = array("i", range(60))
        self._preview_center_x2 = array("i", [0] * 60)
        self._preview_valid_rows = bytearray(60)
        self._preview_touched_rows = array("i", [0] * 60)
        self._preview_touched_count = 0
        self._preview_median_x2 = array("i", [0] * 5)
        self._preview_middle_y = array("i", [0] * 5)
        self._preview_middle_distance = array("i", [0] * 5)
        self._preview_geometry = array("i", [0] * 6)
        self._preview_geometry_centers_owner = None
        self._preview_geometry_row_y_owner = None
        self._preview_geometry_scratch_owner = None
        self._preview_geometry_middle_y_owner = None
        self._preview_geometry_distances_owner = None
        self._preview_geometry_output_owner = None
        self._preview_geometry_arrays_valid = False
        self._preview_segment = array("i", [0] * 5)
        self._preview_segment_y_owner = None
        self._preview_segment_output_owner = None
        self._preview_segment_arrays_valid = False
        self.info = {
            "valid": False, "reason": "NOT_RUN", "source": "NONE",
            "center_x": None, "raw_error": 0,
            "near_count": 0, "far_count": 0,
            "near_weight": 0.0, "far_weight": 0.0,
            "missing_group": "NONE", "supported_rows": 0,
            "far_span": 0, "far_support_ok": False,
            "weight_mode": "NONE",
            "position_error": 0, "heading_error": 0,
            "heading_contribution": 0, "preview_candidate": 0,
            "steering_error": 0,
            "preview_rejection_reason": None,
            "reference_row_y": None, "reference_error": None,
            "preview_valid": False,
            "preview_near_x": None, "preview_near_y": None,
            "preview_far_x": None, "preview_far_y": None,
            "preview_curvature_cells": 0.0,
            "filtered_position": 0, "pd_error": 0,
            "preview_gain_milli": 0, "preview_applied": 0,
        }

    @micropython.native
    def _prepare_weight_kernel(self, weights_q, height):
        """Cache only immutable, bounded integer weights for the Viper loop."""
        if type(weights_q) is not tuple or height > len(self._weight_q):
            return False
        if (self._weight_q_source is weights_q and
                self._weight_q_height == height):
            return self._weight_q_kernel_ready
        self._weight_q_source = weights_q
        self._weight_q_height = height
        self._weight_q_kernel_ready = False
        for index in range(height):
            value = weights_q[index]
            if (not isinstance(value, int) or value < 0 or value > 1000):
                return False
            self._weight_q[index] = value
        self._weight_q_kernel_ready = True
        return True

    @micropython.native
    def clear(self, reason="NO_ACCEPTED_ROWS"):
        info = self.info
        info["valid"] = False
        info["reason"] = reason
        info["source"] = "NONE"
        info["center_x"] = None
        info["raw_error"] = 0
        info["near_count"] = 0
        info["far_count"] = 0
        info["near_weight"] = 0.0
        info["far_weight"] = 0.0
        info["missing_group"] = "NONE"
        info["supported_rows"] = 0
        info["far_span"] = 0
        info["far_support_ok"] = False
        info["weight_mode"] = "NONE"
        info["position_error"] = 0
        info["heading_error"] = 0
        info["heading_contribution"] = 0
        info["preview_candidate"] = 0
        info["steering_error"] = 0
        info["preview_rejection_reason"] = None
        info["reference_row_y"] = None
        info["reference_error"] = None
        info["preview_valid"] = False
        info["preview_near_x"] = None
        info["preview_near_y"] = None
        info["preview_far_x"] = None
        info["preview_far_y"] = None
        info["preview_curvature_cells"] = 0.0
        info["filtered_position"] = 0
        info["pd_error"] = 0
        info["preview_gain_milli"] = 0
        info["preview_applied"] = 0
        touched = self._preview_touched_rows
        for index in range(self._preview_touched_count):
            self._preview_valid_rows[touched[index]] = 0
        self._preview_touched_count = 0
        return info

    @micropython.native
    def _compute_road_preview(self, count, settings, conflicting_centers):
        """Use one current, continuous lower-image route for heading preview."""
        info = self.info
        centers = self._preview_center_x2
        valid_rows = self._preview_valid_rows
        if (conflicting_centers or count < self._PREVIEW_MIN_ROWS):
            return

        road_params = getattr(settings, "ROAD_PARAMS", None)
        if not isinstance(road_params, dict):
            return
        reference_y = road_params.get("ROAD_REFERENCE_Y", 40)
        max_gap_rows = road_params.get("ROAD_MAX_GAP_ROWS", 2)
        if (isinstance(reference_y, bool) or
                not isinstance(reference_y, (int, float)) or
                reference_y != reference_y or
                reference_y == float("inf") or
                reference_y == -float("inf") or
                isinstance(max_gap_rows, bool) or
                not isinstance(max_gap_rows, int) or
                max_gap_rows < 0):
            return
        width = getattr(settings, "SURFACE_GRID_WIDTH", 80)
        if (isinstance(width, bool) or not isinstance(width, int) or
                width < 2):
            return

        band_low = reference_y - self._PREVIEW_REFERENCE_TOLERANCE
        band_high = reference_y + self._PREVIEW_REFERENCE_TOLERANCE
        gap_limit = max_gap_rows + 1
        selected_start_y = -1
        selected_end_y = -1
        selected_rows = 0
        matching_segments = 0
        segment_scan_used = False
        preview_height = len(self._preview_y)
        # A short producer buffer is an invalid route input. Reject it before
        # either the typed kernel or the Python fallback indexes past it.
        if len(valid_rows) < preview_height:
            return
        if (_NATIVE_ACCELERATION_ENABLED and
                type(reference_y) is int and type(band_low) is int and
                type(band_high) is int and type(gap_limit) is int and
                isinstance(valid_rows, bytearray) and
                type(self._preview_y) is array and
                0 < preview_height <= 60 and
                self._PREVIEW_FLOOR_Y >= 0 and
                self._PREVIEW_FLOOR_Y < preview_height and
                type(self._preview_segment) is array and
                len(self._preview_segment) == 5):
            owners_match = (
                self._preview_y is self._preview_segment_y_owner and
                self._preview_segment is self._preview_segment_output_owner)
            if not owners_match:
                self._preview_segment_y_owner = self._preview_y
                self._preview_segment_output_owner = self._preview_segment
                self._preview_segment_arrays_valid = (
                    _road_preview_int32_array_valid(self._preview_y) and
                    _road_preview_int32_array_valid(self._preview_segment))
            if self._preview_segment_arrays_valid:
                _road_preview_segment_scan_viper(
                    valid_rows, self._preview_y, self._PREVIEW_FLOOR_Y,
                    len(self._preview_y), band_low, band_high, gap_limit,
                    self._preview_segment)
                segment_status = self._preview_segment[0]
                segment_scan_used = True
                if segment_status < 0:
                    return
                if segment_status > 0:
                    selected_start_y = self._preview_segment[1]
                    selected_end_y = self._preview_segment[2]
                    selected_rows = self._preview_segment[3]
                    matching_segments = self._preview_segment[4]

        if not segment_scan_used:
            segment_start_y = -1
            segment_end_y = -1
            segment_rows = 0
            segment_contains_reference = False
            previous_y = -1
            for slot in range(self._PREVIEW_FLOOR_Y, len(self._preview_y)):
                if not valid_rows[slot]:
                    continue
                row_y = self._preview_y[slot]
                if (segment_start_y >= 0 and
                        row_y - previous_y > gap_limit):
                    if segment_contains_reference:
                        matching_segments += 1
                        if matching_segments > 1:
                            return
                        selected_start_y = segment_start_y
                        selected_end_y = segment_end_y
                        selected_rows = segment_rows
                    segment_start_y = row_y
                    segment_rows = 1
                    segment_contains_reference = (
                        band_low <= row_y <= band_high)
                elif segment_start_y < 0:
                    segment_start_y = row_y
                    segment_rows = 1
                    segment_contains_reference = (
                        band_low <= row_y <= band_high)
                else:
                    segment_rows += 1
                    if band_low <= row_y <= band_high:
                        segment_contains_reference = True
                segment_end_y = row_y
                previous_y = row_y

            if segment_start_y >= 0 and segment_contains_reference:
                matching_segments += 1
                if matching_segments > 1:
                    return
                selected_start_y = segment_start_y
                selected_end_y = segment_end_y
                selected_rows = segment_rows

        if selected_start_y < 0:
            return
        y_span = selected_end_y - selected_start_y
        endpoint_rows = self._PREVIEW_ENDPOINT_ROWS
        if (selected_rows < self._PREVIEW_MIN_ROWS or
                y_span < self._PREVIEW_MIN_SPAN or
                selected_rows < endpoint_rows * 3):
            return

        geometry_status = -1
        valid_rows = self._preview_valid_rows
        centers = self._preview_center_x2
        row_y = self._preview_y
        scratch = self._preview_median_x2
        middle_y = self._preview_middle_y
        distances = self._preview_middle_distance
        output = self._preview_geometry
        if (type(selected_start_y) is int and
                type(selected_end_y) is int and
                0 <= selected_start_y <= selected_end_y < 60 and
                type(valid_rows) is bytearray and len(valid_rows) == 60 and
                isinstance(centers, array) and len(centers) == 60 and
                isinstance(row_y, array) and len(row_y) == 60 and
                isinstance(scratch, array) and len(scratch) >= endpoint_rows and
                isinstance(middle_y, array) and len(middle_y) >= endpoint_rows and
                isinstance(distances, array) and len(distances) >= endpoint_rows and
                isinstance(output, array) and len(output) >= 6):
            owners_match = (
                centers is self._preview_geometry_centers_owner and
                row_y is self._preview_geometry_row_y_owner and
                scratch is self._preview_geometry_scratch_owner and
                middle_y is self._preview_geometry_middle_y_owner and
                distances is self._preview_geometry_distances_owner and
                output is self._preview_geometry_output_owner)
            if not owners_match:
                self._preview_geometry_centers_owner = centers
                self._preview_geometry_row_y_owner = row_y
                self._preview_geometry_scratch_owner = scratch
                self._preview_geometry_middle_y_owner = middle_y
                self._preview_geometry_distances_owner = distances
                self._preview_geometry_output_owner = output
                self._preview_geometry_arrays_valid = (
                    _road_preview_int32_array_valid(centers) and
                    _road_preview_int32_array_valid(row_y) and
                    _road_preview_int32_array_valid(scratch) and
                    _road_preview_int32_array_valid(middle_y) and
                    _road_preview_int32_array_valid(distances) and
                    _road_preview_int32_array_valid(output))
            if self._preview_geometry_arrays_valid:
                geometry_status = _road_preview_integer_geometry_viper(
                    valid_rows, centers, row_y, selected_start_y,
                    selected_end_y, endpoint_rows, scratch, middle_y,
                    distances, output)

        if geometry_status == 0:
            return
        if geometry_status == 1:
            far_center_x2 = output[0]
            far_y = output[1]
            near_center_x2 = output[2]
            near_y = output[3]
            middle_center_x2 = output[4]
            middle_row = output[5]
        else:
            scratch = self._preview_median_x2
            collected = 0
            far_y = -1
            for slot in range(selected_start_y, selected_end_y + 1):
                if not valid_rows[slot]:
                    continue
                scratch[collected] = centers[slot]
                collected += 1
                if collected == endpoint_rows // 2 + 1:
                    far_y = self._preview_y[slot]
                if collected == endpoint_rows:
                    far_band_end = slot
                    break
            if collected != endpoint_rows:
                return
            for index in range(1, endpoint_rows):
                value = scratch[index]
                insert_at = index
                while insert_at > 0 and scratch[insert_at - 1] > value:
                    scratch[insert_at] = scratch[insert_at - 1]
                    insert_at -= 1
                scratch[insert_at] = value
            far_center_x2 = scratch[endpoint_rows // 2]
    
            collected = 0
            near_y = -1
            for slot in range(selected_end_y, selected_start_y - 1, -1):
                if not valid_rows[slot]:
                    continue
                scratch[collected] = centers[slot]
                collected += 1
                if collected == endpoint_rows // 2 + 1:
                    near_y = self._preview_y[slot]
                if collected == endpoint_rows:
                    near_band_start = slot
                    break
            if collected != endpoint_rows:
                return
            for index in range(1, endpoint_rows):
                value = scratch[index]
                insert_at = index
                while insert_at > 0 and scratch[insert_at - 1] > value:
                    scratch[insert_at] = scratch[insert_at - 1]
                    insert_at -= 1
                scratch[insert_at] = value
            near_center_x2 = scratch[endpoint_rows // 2]
    
            # A straight lane can be diagonal under camera yaw/perspective. Only
            # boost its heading when the forward part bends away from the local
            # near-to-middle tangent. Use three disjoint measured row windows.
            middle_y = self._preview_middle_y
            distances = self._preview_middle_distance
            middle_count = 0
            target_y2 = near_y + far_y
            for slot in range(far_band_end + 1, near_band_start):
                if not valid_rows[slot]:
                    continue
                distance = abs(slot * 2 - target_y2)
                insert_at = middle_count
                if insert_at >= endpoint_rows:
                    insert_at = endpoint_rows - 1
                    if distance >= distances[insert_at]:
                        continue
                else:
                    middle_count += 1
                while insert_at > 0 and distances[insert_at - 1] > distance:
                    distances[insert_at] = distances[insert_at - 1]
                    middle_y[insert_at] = middle_y[insert_at - 1]
                    scratch[insert_at] = scratch[insert_at - 1]
                    insert_at -= 1
                distances[insert_at] = distance
                middle_y[insert_at] = slot
                scratch[insert_at] = centers[slot]
            if middle_count != endpoint_rows:
                return
            for index in range(1, endpoint_rows):
                value = scratch[index]
                insert_at = index
                while insert_at > 0 and scratch[insert_at - 1] > value:
                    scratch[insert_at] = scratch[insert_at - 1]
                    insert_at -= 1
                scratch[insert_at] = value
            middle_center_x2 = scratch[endpoint_rows // 2]
            for index in range(1, endpoint_rows):
                value = middle_y[index]
                insert_at = index
                while insert_at > 0 and middle_y[insert_at - 1] > value:
                    middle_y[insert_at] = middle_y[insert_at - 1]
                    insert_at -= 1
                middle_y[insert_at] = value
            middle_row = middle_y[endpoint_rows // 2]
        local_span = near_y - middle_row
        if local_span <= 0:
            return
        curvature_numerator = (
            (far_center_x2 - near_center_x2) * local_span -
            (middle_center_x2 - near_center_x2) * (near_y - far_y))
        info["preview_curvature_cells"] = (
            curvature_numerator / float(2 * local_span))

        heading_error = _rounded_local(
            (far_center_x2 - near_center_x2) * 1000.0 / (width - 1))
        if heading_error < -1000:
            heading_error = -1000
        elif heading_error > 1000:
            heading_error = 1000
        contribution = heading_error * 2
        if contribution < -400:
            contribution = -400
        elif contribution > 400:
            contribution = 400
        # At least 1.5 grid cells beyond the local tangent, turning in the
        # same direction as that bend. No historical heading or extrapolated
        # route point is fed back into geometry or the width cache.
        if (abs(curvature_numerator) < 3 * local_span or
                curvature_numerator * heading_error <= 0):
            contribution = 0

        info["heading_error"] = heading_error
        info["preview_candidate"] = contribution
        info["preview_valid"] = True
        info["preview_near_x"] = near_center_x2 * 0.5
        info["preview_near_y"] = near_y
        info["preview_far_x"] = far_center_x2 * 0.5
        info["preview_far_y"] = far_y

    @micropython.native
    def aggregate(self, selection, road_result, settings):
        """Aggregate only accepted rows from the selected ordinary route.

        The fixed-point parabolic weights are indexed by 80x60 grid row. A
        full weighted estimate requires measured support in the center core;
        otherwise the selected same-lane chain's nearest measured row is used
        and marked DEGRADED_CORE_SUPPORT.
        """
        info = self.clear()
        if not selection or not selection.get("valid"):
            info["reason"] = "SELECTION_INVALID"
            return info
        source = selection.get("source")
        if source == "road":
            result = road_result
            stride = 4
            support = result.get("boundary_support_rows") if result else None
            support_count = result.get("boundary_support_count", -1) \
                if result else -1
        elif source in ("single_left", "single_right",
                        "ring_left", "ring_right"):
            result = selection.get("result")
            stride = 5
            support = None
            support_count = -1
        else:
            info["reason"] = "UNSUPPORTED_SOURCE"
            return info
        if not result:
            info["reason"] = "MISSING_ROW_RESULT"
            return info
        rows = result.get("selected_row_ints")
        count = result.get("selected_row_count")
        if (not isinstance(rows, array) or isinstance(count, bool) or
                not isinstance(count, int) or count < 1 or count > 60 or
                result.get("selected_row_stride") != stride or
                len(rows) < count * stride):
            info["reason"] = "INVALID_SELECTED_ROWS"
            return info
        if source == "road" and (
                not isinstance(support, bytearray) or
                support_count != count or len(support) < count):
            info["reason"] = "PAIR_SUPPORT_UNAVAILABLE"
            return info

        width = settings.SURFACE_GRID_WIDTH
        height = settings.SURFACE_GRID_HEIGHT
        floor_y = settings.SURFACE_ROI_FLOOR_Y
        weights_q = getattr(settings, "STEERING_ROW_WEIGHT_Q", None)
        weight_scale = getattr(settings, "STEERING_ROW_WEIGHT_Q_SCALE", None)
        if (not isinstance(weights_q, (tuple, list)) or
                len(weights_q) != height or
                not isinstance(weight_scale, int) or weight_scale <= 0):
            info["reason"] = "ROW_WEIGHT_CONFIG_INVALID"
            return info

        preview_count = 0
        preview_conflicting_centers = False
        preview_centers = self._preview_center_x2
        preview_valid_rows = self._preview_valid_rows
        centers = self._center_x2
        ys = self._row_y
        accepted_count = 0
        core_low = settings.STEERING_CORE_RAW_Y_MIN // \
            settings.SURFACE_PIXEL_SCALE_Y
        core_high = settings.STEERING_CORE_RAW_Y_MAX // \
            settings.SURFACE_PIXEL_SCALE_Y
        core_count = 0
        core_min_y = height
        core_max_y = -1
        # Avoid allocating a throwaway default dict on every frame; the None
        # fallback preserves the same missing-config reference_y=40 behavior.
        road_params = getattr(settings, "ROAD_PARAMS", None)
        reference_y = (road_params.get("ROAD_REFERENCE_Y", 40)
                       if isinstance(road_params, dict) else 40)
        best_reference_index = -1
        best_reference_distance = self._PREVIEW_REFERENCE_TOLERANCE + 1
        reference_conflict = False

        use_viper_rows = (
            _NATIVE_ACCELERATION_ENABLED and
            type(width) is int and type(height) is int and
            type(floor_y) is int and type(core_low) is int and
            type(core_high) is int and type(reference_y) is int and
            type(self._PREVIEW_FLOOR_Y) is int and
            type(self._PREVIEW_REFERENCE_TOLERANCE) is int)
        if use_viper_rows:
            _aggregate_selected_rows_viper(
                rows, count, stride, 1 if source == "road" else 0,
                support if source == "road" else preview_valid_rows,
                preview_valid_rows, preview_centers,
                self._preview_touched_rows, self._preview_touched_count,
                centers, ys, floor_y, width, height,
                self._PREVIEW_FLOOR_Y, core_low, core_high,
                reference_y, self._PREVIEW_REFERENCE_TOLERANCE,
                self._row_metrics)
            metrics = self._row_metrics
            accepted_count = metrics[0]
            preview_count = metrics[1]
            preview_conflicting_centers = metrics[2] != 0
            core_count = metrics[3]
            core_min_y = metrics[4]
            core_max_y = metrics[5]
            best_reference_index = metrics[6]
            best_reference_distance = metrics[7]
            reference_conflict = metrics[8] != 0
            self._preview_touched_count = metrics[9]
        else:
            # Host and unusual-configuration fallback mirrors the typed loop.
            # FastSingleEdgeTracker exports only accepted row_status==1.
            for index in range(count):
                offset = index * stride
                y = rows[offset]
                if source == "road":
                    flags = rows[offset + 3]
                    accepted = support[index] == 1 and (flags & 7) == 7
                    center_x2 = rows[offset + 1] + rows[offset + 2]
                else:
                    accepted = True
                    center_x2 = rows[offset + 1]
                if (not accepted or y < floor_y or y >= height or
                        center_x2 < 0 or center_x2 > (width - 1) * 2):
                    continue

                if preview_valid_rows[y]:
                    if preview_centers[y] != center_x2:
                        preview_conflicting_centers = True
                    # Duplicate selected rows never receive extra weight.
                    continue
                preview_valid_rows[y] = 1
                preview_centers[y] = center_x2
                self._preview_touched_rows[self._preview_touched_count] = y
                self._preview_touched_count += 1
                if y >= self._PREVIEW_FLOOR_Y:
                    preview_count += 1

                centers[accepted_count] = center_x2
                ys[accepted_count] = y
                accepted_count += 1
                if core_low <= y <= core_high:
                    core_count += 1
                    if y < core_min_y:
                        core_min_y = y
                    if y > core_max_y:
                        core_max_y = y
                distance = abs(y - reference_y)
                if distance <= self._PREVIEW_REFERENCE_TOLERANCE:
                    if distance < best_reference_distance:
                        best_reference_index = accepted_count - 1
                        best_reference_distance = distance
                        reference_conflict = False
                    elif (distance == best_reference_distance and
                          best_reference_index >= 0 and
                          centers[best_reference_index] != center_x2):
                        reference_conflict = True

        info["source"] = source
        info["supported_rows"] = accepted_count
        info["near_count"] = core_count
        info["far_count"] = accepted_count - core_count
        info["far_span"] = core_max_y - core_min_y if core_count else 0
        core_span = info["far_span"]
        core_ok = (core_count >= settings.STEERING_CORE_MIN_SUPPORT_ROWS and
                   core_span >= settings.STEERING_CORE_MIN_SUPPORT_SPAN_GRID)
        info["far_support_ok"] = core_ok
        if accepted_count == 0:
            info["reason"] = "NO_SUPPORTED_ROWS"
            return info
        total_weight = 0
        weighted_x2 = 0
        core_weight = 0
        if core_ok:
            if self._prepare_weight_kernel(weights_q, height):
                _weighted_row_sums_viper(
                    centers, ys, self._weight_q, accepted_count,
                    core_low, core_high, self._weight_sums)
                weighted_x2 = self._weight_sums[0]
                total_weight = self._weight_sums[1]
                core_weight = self._weight_sums[2]
            else:
                for index in range(accepted_count):
                    y = ys[index]
                    weight = int(weights_q[y])
                    total_weight += weight
                    weighted_x2 += centers[index] * weight
                    if core_low <= y <= core_high:
                        core_weight += weight
            if total_weight <= 0:
                info["reason"] = "ROW_WEIGHT_SUM_ZERO"
                return info
            center_x = weighted_x2 / float(2 * total_weight)
            info["weight_mode"] = "PARABOLIC_ROWS"
            info["near_weight"] = core_weight / float(total_weight)
            info["far_weight"] = 1.0 - info["near_weight"]
        else:
            nearest_index = -1
            nearest_distance = height + 1
            nearest_center_distance = height + 1
            core_midpoint = (core_low + core_high) // 2
            for index in range(accepted_count):
                y = ys[index]
                if y < core_low:
                    distance = core_low - y
                elif y > core_high:
                    distance = y - core_high
                else:
                    distance = 0
                center_distance = abs(y - core_midpoint)
                if (distance < nearest_distance or
                        (distance == nearest_distance and
                         center_distance < nearest_center_distance)):
                    nearest_index = index
                    nearest_distance = distance
                    nearest_center_distance = center_distance
            if nearest_index < 0:
                info["reason"] = "CORE_REFERENCE_UNAVAILABLE"
                return info
            center_x = centers[nearest_index] * 0.5
            info["weight_mode"] = "DEGRADED_CORE_SUPPORT"
            info["near_weight"] = 1.0
            info["far_weight"] = 0.0
            info["missing_group"] = "DEGRADED_CORE_SUPPORT"
            reference_row_y = ys[nearest_index]
            info["reference_row_y"] = reference_row_y
            info["reference_error"] = _normalize_fast_reference(
                center_x, width)

        try:
            raw_error = _normalize_fast_reference(center_x, width)
        except (TypeError, ValueError, OverflowError):
            info["reason"] = "WEIGHTED_CENTER_INVALID"
            info["source"] = "NONE"
            return info
        if raw_error < -1000 or raw_error > 1000:
            info["reason"] = "WEIGHTED_ERROR_OUT_OF_RANGE"
            info["source"] = "NONE"
            return info

        info["valid"] = True
        info["reason"] = "OK" if core_ok else "DEGRADED_CORE_SUPPORT"
        info["center_x"] = center_x
        info["raw_error"] = raw_error
        info["position_error"] = raw_error
        if core_ok:
            if best_reference_index >= 0 and not reference_conflict:
                info["reference_row_y"] = ys[best_reference_index]
                info["reference_error"] = _normalize_fast_reference(
                    centers[best_reference_index] * 0.5, width)
            self._compute_road_preview(
                preview_count, settings, preview_conflicting_centers)
            info["preview_rejection_reason"] = "AWAITING_CONTROL_PROFILE"
        else:
            info["preview_rejection_reason"] = "DEGRADED_CORE_SUPPORT"
        info["steering_error"] = info["position_error"]
        return info


def _validate_roi_config():
    if not hasattr(config, "ROI_PRESET"):
        raise RuntimeError(
            "ROI_PRESET missing; upload matching config.py and reboot")
    if getattr(config, "LEGACY_BUILD_ID", None) != ROI_BUILD_ID:
        raise RuntimeError(
            "ROI config/build mismatch; upload matching config.py and reboot")
    presets = getattr(config, "ROI_PRESETS", None)
    if not presets or config.ROI_PRESET not in presets:
        raise RuntimeError(
            "ROI_PRESET invalid; upload matching config.py and reboot")
    return presets[config.ROI_PRESET]


def _validate_scan_config():
    expected_rows, expected_reference, expected_span = \
        _validate_roi_config()
    rows = config.SCAN_ROWS
    radius = config.PROFILE_BAND_RADIUS
    if not rows or len(rows) > 9:
        raise ValueError("SCAN_ROWS must contain 1..9 rows")
    for index in range(1, len(rows)):
        if rows[index] <= rows[index - 1]:
            raise ValueError("SCAN_ROWS must be strictly increasing")
    if radius != 1:
        raise ValueError("PROFILE_BAND_RADIUS must be 1 for median3")
    if config.REFERENCE_Y < 0 or \
            config.REFERENCE_Y >= config.IMAGE_HEIGHT:
        raise ValueError("REFERENCE_Y is outside the image")
    if config.REFERENCE_Y < rows[0] or \
            config.REFERENCE_Y > rows[-1]:
        raise ValueError("REFERENCE_Y must be covered by SCAN_ROWS")
    for y in rows:
        if y - radius < 0 or y + radius >= config.IMAGE_HEIGHT:
            raise ValueError("SCAN_ROWS median band is outside the image")
    scene_rows = config.SCENE_CONTEXT_ROWS
    if not scene_rows or len(scene_rows) > config.SCENE_CONTEXT_MAX_ROWS:
        raise ValueError("SCENE_CONTEXT_ROWS must contain 1..max rows")
    for index in range(1, len(scene_rows)):
        if scene_rows[index] <= scene_rows[index - 1]:
            raise ValueError("SCENE_CONTEXT_ROWS must be strictly increasing")
    for y in scene_rows:
        if y - radius < 0 or y + radius >= config.IMAGE_HEIGHT:
            raise ValueError("SCENE_CONTEXT_ROWS median band is outside image")
    if len(set(rows).union(scene_rows)) > 18:
        raise ValueError("SCAN_ROWS and scene context exceed 18 unique rows")
    params = config.DETECTOR_PARAMS
    if tuple(rows) != tuple(expected_rows) or \
            config.REFERENCE_Y != expected_reference or \
            config.MIN_FIT_Y_SPAN_PX != expected_span or \
            tuple(params.get("scan_rows", ())) != tuple(expected_rows) or \
            params.get("reference_y") != expected_reference or \
            params.get("min_fit_y_span_px") != expected_span or \
            tuple(params.get("scene_context_rows", ())) != tuple(scene_rows) or \
            params.get("boundary_guard_enabled") != \
            config.BOUNDARY_GUARD_ENABLED:
        raise RuntimeError(
            "ROI_PRESET mismatch; upload matching config.py and reboot")


def _sample_profiles(img):
    """Take a vertical median-of-three profile at each configured row."""
    _validate_scan_config()
    return [_sample_profile_at(img, y) for y in config.SCAN_ROWS]


def _sample_profile_at(img, y):
    pixels = []
    radius = config.PROFILE_BAND_RADIUS
    for x in range(config.IMAGE_WIDTH):
        top = img.get_pixel((x, y - radius))
        middle = img.get_pixel((x, y))
        bottom = img.get_pixel((x, y + radius))
        pixels.append(adaptive_detector.median3(top, middle, bottom))
    return y, pixels


def _sample_frame_profiles(img):
    """Sample scan and scene rows once, reusing every overlapping profile."""
    _validate_scan_config()
    scan_rows = tuple(config.SCAN_ROWS)
    scene_rows = tuple(config.SCENE_CONTEXT_ROWS)
    sampled = {}
    for y in sorted(set(scan_rows).union(scene_rows)):
        sampled[y] = _sample_profile_at(img, y)[1]
    profiles = [(y, sampled[y]) for y in scan_rows]
    context = [(y, sampled[y]) for y in scene_rows]
    return profiles, context


def _short_reason(reason):
    return {
        "OK": "OK",
        "BAD_MODE": "BADMODE",
        "BAD_WIDTH": "WIDTH",
        "LOW_CONTRAST": "CONTRAST",
        "WEAK_CLASS": "CLASS",
        "NO_DARK": "NODARK",
        "NO_WIDTH_MATCH": "NOWIDTH",
        "NOISY_ROW": "NOISY",
        "ONE_EDGE": "ONEEDGE",
        "GAP_RANGE": "GAP",
        "INTERIOR_DARK": "INTERIOR",
        "MULTIPLE_PAIRS": "MULTI",
        "TOO_FEW_ROWS": "FEWROWS",
        "FIT_SUPPORT": "FITSUP",
        "FIT_RESIDUAL": "FITRES",
        "FIT_Y_SPAN": "YSPAN",
        "REFERENCE_UNCOVERED": "NOREF",
        "CENTER_OUT_OF_RANGE": "OUTRANGE",
        "LOCAL_RESIDUAL": "LOCALRES",
        "LOW_QUALITY": "LOWQUAL",
        "BAD_TRACK_MODEL": "BADMODEL",
        "MODEL_SUPPORT": "MODELSUP",
        "MODEL_GEOMETRY": "MODELGEOM",
        "MODEL_CURVATURE": "CURVE",
        "MODEL_SLOPE": "SLOPE",
        "MODEL_REFIT": "REFIT",
        "MODEL_AMBIGUOUS": "AMBIGMODEL",
        "SUPPORT_FRAGMENTED": "FRAGMENTED",
        "BAD_QUADRATIC_CONFIG": "BADCONFIG",
    }.get(reason, "MIX")


def _short_mode(mode):
    return {"white_corridor": "W", "black_edges": "B"}.get(mode, "?")


def _draw_result(img, result, valid, raw_error, filtered_error, quality,
                 mode, processing_ms, fps):
    details = result[6]
    points = result[5] if valid else []
    ordered = sorted(points, key=lambda point: point[0])
    for index in range(1, len(ordered)):
        before = ordered[index - 1]
        current = ordered[index]
        img.draw_line((int(before[3] + 0.5), before[0],
                       int(current[3] + 0.5), current[0]), color=255)
    for y, left_inner, right_inner, center in ordered:
        img.draw_cross((int(left_inner + 0.5), y), color=0)
        img.draw_cross((int(right_inner + 0.5), y), color=0)
    if valid:
        img.draw_cross((int(result[1] + 0.5), config.REFERENCE_Y), color=0)

    state = "VALID" if valid else "INVALID"
    error_text = "%+d" % filtered_error if valid else "0"
    raw_text = "%+d" % raw_error if valid else "0"
    img.draw_string((0, 0), "%s e=%s" % (state, error_text),
                    color=255, scale=1)
    img.draw_string((0, 12), "r=%s q=%d" % (raw_text, quality),
                    color=255, scale=1)
    img.draw_string((0, 24), "s=%d/%d p=%dms" %
                    (result[3], len(config.SCAN_ROWS), processing_ms),
                    color=255, scale=1)
    threshold_min = details["threshold_min"]
    threshold_max = details["threshold_max"]
    if threshold_min is None:
        threshold_text = "t=--"
    else:
        threshold_text = "t=%d-%d" % (threshold_min, threshold_max)
    img.draw_string((0, 36), "%s %s f=%d" %
                    (_short_mode(mode), threshold_text, int(fps)),
                    color=255, scale=1)


def _format_vdbg(mode, valid, result, raw_error, filtered_error,
                 processing_ms, fps):
    details = result[6]
    threshold_min = details["threshold_min"]
    threshold_max = details["threshold_max"]
    if threshold_min is None:
        threshold_text = "--"
    else:
        threshold_text = "%d..%d" % (threshold_min, threshold_max)
    scene_threshold = details.get("scene_threshold")
    scene_threshold_text = ("--" if scene_threshold is None else
                            str(int(scene_threshold)))
    scene_otsu_threshold = details.get("scene_otsu_threshold")
    scene_otsu_text = ("--" if scene_otsu_threshold is None else
                       str(int(scene_otsu_threshold)))
    row_reasons = {}
    for row in details["rows"]:
        reason = row["reason"]
        row_reasons[reason] = row_reasons.get(reason, 0) + 1
    ordered_reasons = sorted(row_reasons.items(), key=lambda item: item[1],
                             reverse=True)
    if ordered_reasons:
        row_reason_text = ",".join(
            "%s:%d" % (_short_reason(reason), count)
            for reason, count in ordered_reasons[:2])
    else:
        row_reason_text = "-"
    return ("VDBG mode=%s model=%s roi=%s ref=%d "
            "valid=%d q=%d sup=%d/%d hyp=%d amb=%d "
            "c=%.1f curv=%.4f t=%s scene_otsu_t=%s "
            "scene_t=%s scene_c=%.1f guard=%d "
            "r=%.1f lr=%.1f raw=%d filt=%d proc=%dms fps=%.1f "
            "why=%s rows=%s" % (
                mode, details.get("model", "linear"),
                config.ROI_PRESET, config.REFERENCE_Y,
                1 if valid else 0, details["confidence"], result[3],
                 len(config.SCAN_ROWS),
                 details.get("model_hypotheses", 0), result[4],
                 details["contrast_min"],
                details.get("curvature", 0.0), threshold_text,
                 scene_otsu_text, scene_threshold_text,
                 details.get("scene_contrast", 0.0),
                 1 if details.get("boundary_guard_enabled", True) else 0,
                details["fit_residual"],
                details["local_residual"], raw_error, filtered_error,
                processing_ms, fps, _short_reason(details["reason"]),
                row_reason_text))


def _format_vrow(row):
    """Format one bounded per-scan-row diagnostic without changing it."""
    threshold = row.get("threshold")
    threshold_text = "--" if threshold is None else str(int(threshold))
    local_threshold = row.get("local_threshold")
    local_threshold_text = ("--" if local_threshold is None else
                             str(int(local_threshold)))
    threshold_cap = row.get("threshold_cap")
    threshold_cap_text = ("--" if threshold_cap is None else
                          str(int(threshold_cap)))
    run_widths = row.get("runs", ())[:8]
    runs_text = ",".join(str(int(width)) for width in run_widths)
    if not runs_text:
        runs_text = "-"

    candidate = row.get("candidate")
    if candidate is None:
        candidate_text = "-"
    else:
        left, right, center, gap, _dark_count = candidate
        candidate_text = "%.0f,%.0f,%.1f,%d" % (
            left, right, center, gap)

    return ("VROW y=%d reason=%s t=%s localt=%s cap=%s "
            "contrast=%.1f classes=%d/%d "
            "runs=%s pairs=%d gaprej=%d intrej=%d cand=%s" % (
                row["y"], row["reason"], threshold_text,
                local_threshold_text, threshold_cap_text,
                row.get("contrast", 0.0), row.get("dark_class", 0),
                row.get("light_class", 0), runs_text,
                row.get("candidate_count", 0),
                row.get("gap_rejects", 0),
                row.get("interior_rejects", 0), candidate_text))


def _print_vrow_details(result):
    """Print at most the configured nine scan rows from this frame result."""
    for row in result[6].get("rows", ())[:9]:
        print(_format_vrow(row))


def _send_observation(uart, sequence, valid, confidence, error):
    frame = vision_protocol.pack_frame(
        sequence,
        1 if valid else 0,
        confidence if valid else 0,
        error if valid else 0,
    )
    written = uart.write(frame)
    if written is None or written != len(frame):
        raise OSError("short UART write")


def _configure_sensor(sensor_module):
    sensor_module.reset()
    sensor_module.set_pixformat(sensor_module.GRAYSCALE)
    sensor_module.set_framesize(getattr(sensor_module,
                                        config.FRAME_SIZE_NAME))
    sensor_module.set_windowing((config.SENSOR_WINDOW_X,
                                 config.SENSOR_WINDOW_Y,
                                 config.IMAGE_WIDTH,
                                 config.IMAGE_HEIGHT))
    sensor_module.set_hmirror(config.MIRROR_X)
    sensor_module.set_vflip(config.FLIP_Y)
    sensor_module.skip_frames(time=1000)
    sensor_module.set_auto_gain(False, gain_db=config.GAIN_DB)
    if config.AUTO_EXPOSURE:
        sensor_module.set_auto_exposure(True)
    else:
        sensor_module.set_auto_exposure(False,
                                        exposure_us=config.EXPOSURE_US)
    sensor_module.skip_frames(time=200)


def _main_legacy():
    _validate_scan_config()
    print("BUILD=%s model=%s mode=%s roi=%s ref=%d span=%d" % (
        config.BUILD_ID, config.TRACK_MODEL, config.DETECTION_MODE,
        config.ROI_PRESET, config.REFERENCE_Y, config.MIN_FIT_Y_SPAN_PX))
    _configure_sensor(sensor)
    uart = pyb.UART(config.UART_ID, config.UART_BAUDRATE,
                    bits=config.UART_BITS, parity=None,
                    stop=config.UART_STOP_BITS,
                    timeout=config.UART_TIMEOUT_MS,
                    timeout_char=config.UART_CHAR_TIMEOUT_MS)
    clock = time.clock()
    observation_filter = adaptive_detector.ObservationFilter(
        config.OBSERVATION_FILTER_ALPHA,
        config.OBSERVATION_FILTER_MAX_GAP_MS,
        config.MIN_CONFIDENCE)
    sequence = 0
    last_send_ms = time.ticks_ms()
    last_debug_ms = last_send_ms
    tx_error_count = 0

    while True:
        clock.tick()
        img = sensor.snapshot()
        if img.width() != config.IMAGE_WIDTH or \
                img.height() != config.IMAGE_HEIGHT:
            raise RuntimeError("image size differs from openmv/config.py")

        processing_start_ms = time.ticks_ms()
        profiles, context_profiles = _sample_frame_profiles(img)
        result = adaptive_detector.detect_track_center(
            profiles, config.IMAGE_WIDTH, config.DETECTOR_PARAMS,
            config.DETECTION_MODE, config.TRACK_MODEL, context_profiles)
        raw_valid = (result[0] != 0 and
                     result[2] >= config.MIN_CONFIDENCE)
        if raw_valid:
            try:
                raw_error = adaptive_detector.normalize_lateral_error(
                    result[1], config.IMAGE_WIDTH)
            except ValueError:
                raw_valid = False
                raw_error = 0
        else:
            raw_error = 0

        now_ms = time.ticks_ms()
        filtered_error = observation_filter.update(
            now_ms, raw_valid, result[2], raw_error, time.ticks_diff)
        valid = raw_valid
        quality = result[2]
        if not valid:
            raw_error = 0
            filtered_error = 0
        processing_ms = time.ticks_diff(now_ms, processing_start_ms)
        fps = clock.fps()

        if config.DEBUG_ENABLED and time.ticks_diff(
                now_ms, last_debug_ms) >= config.DEBUG_PERIOD_MS:
            print(_format_vdbg(config.DETECTION_MODE, valid, result,
                               raw_error, filtered_error,
                               processing_ms, fps))
            if config.DEBUG_ROW_DETAILS:
                _print_vrow_details(result)
            last_debug_ms = now_ms

        _draw_result(img, result, valid, raw_error, filtered_error,
                     quality if valid else 0, config.DETECTION_MODE,
                     processing_ms, fps)

        send_now_ms = time.ticks_ms()
        if time.ticks_diff(send_now_ms, last_send_ms) >= config.SEND_PERIOD_MS:
            try:
                _send_observation(uart, sequence, valid, quality,
                                  filtered_error)
            except Exception as exc:
                tx_error_count += 1
                print("UART_WRITE_ERROR=%d %s" % (tx_error_count, exc))
            sequence = (sequence + 1) & 0xFFFF
            last_send_ms = send_now_ms


def _validate_v6_gray_config():
    _validate_v6_runtime_apis()
    build_id = getattr(config, "BUILD_ID", None)
    configured_route = getattr(config, "SINGLE_EDGE_ROUTE_MODE", None)
    route_build_matches = False
    for route_mode, route_build_id in SINGLEEDGE_ROUTE_BUILD_IDS:
        if configured_route == route_mode and build_id == route_build_id:
            route_build_matches = True
            break
    if (build_id not in ("SIXTH_V6_GRAY_ROI_TOPOLOGY_20261004",
                         V7_BUILD_ID) and not route_build_matches):
        raise RuntimeError(
            "compare config/build mismatch; install a V6/V7 compatible bundle")
    if (build_id == V7_BUILD_ID or route_build_matches) and getattr(
            config, "V7_RUNTIME_API_VERSION", None) != \
            V7_RUNTIME_API_VERSION:
        raise RuntimeError("compare config V7 API version mismatch")
    if route_build_matches and getattr(
            config, "SINGLEEDGE_RUNTIME_API_VERSION", None) != \
            SINGLEEDGE_RUNTIME_API_VERSION:
        raise RuntimeError("compare config single-edge API version mismatch")
    if not isinstance(getattr(config, "OVERLAY_ENABLED", None), bool):
        raise ValueError("V6 OVERLAY_ENABLED must be bool")
    if getattr(config, "VISION_BACKEND", None) not in ("compare", "legacy"):
        raise ValueError("VISION_BACKEND must be compare or legacy")
    if getattr(config, "SURFACE_MODE", None) != "gray_unified":
        raise ValueError("V6 requires the single gray_unified camera path")
    preferred_source = getattr(
        config, "GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID", None)
    if preferred_source not in ("road", "surface"):
        raise ValueError(
            "V6 GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID must be road or surface")
    if (config.FRAME_SIZE_NAME != "QVGA" or
            config.IMAGE_WIDTH != 320 or config.IMAGE_HEIGHT != 120 or
            config.SENSOR_WINDOW_X != 0 or config.SENSOR_WINDOW_Y != 60 or
            config.SURFACE_GRID_WIDTH != 80 or
            config.SURFACE_GRID_HEIGHT != 60 or
            config.SURFACE_PIXEL_SCALE_X != 4 or
            config.SURFACE_PIXEL_SCALE_Y != 2):
        raise ValueError(
            "V6 requires QVGA center crop 320x120 and an 80x60 4x2 grid")
    if not 60 <= config.SURFACE_MIN_CONFIDENCE <= 100:
        raise ValueError("V6 minimum confidence must be in 60..100")
    if not 0 < config.SURFACE_FRAME_BUDGET_MS <= 200:
        raise ValueError("V6 frame budget must be in 1..200 ms")
    boundary_step = getattr(config, "GRAY_BOUNDARY_MIN_STEP", None)
    if (not isinstance(boundary_step, (int, float)) or
            isinstance(boundary_step, bool) or boundary_step <= 0 or
            boundary_step > 255 or boundary_step != boundary_step or
            boundary_step == float("inf")):
        raise ValueError(
            "V6 GRAY_BOUNDARY_MIN_STEP must be finite and in 1..255")
    boundary_fraction = getattr(
        config, "GRAY_BOUNDARY_MIN_STEP_FRACTION", None)
    if (not isinstance(boundary_fraction, (int, float)) or
            isinstance(boundary_fraction, bool) or boundary_fraction < 0 or
            boundary_fraction >= 1 or boundary_fraction != boundary_fraction or
            boundary_fraction == float("inf")):
        raise ValueError(
            "V6 GRAY_BOUNDARY_MIN_STEP_FRACTION must be finite in [0,1)")
    support_ratio = getattr(config, "GRAY_BOUNDARY_SUPPORT_RATIO", None)
    if (not isinstance(support_ratio, (int, float)) or
            isinstance(support_ratio, bool) or support_ratio <= 0 or
            support_ratio > 1 or support_ratio != support_ratio or
            support_ratio == float("inf")):
        raise ValueError(
            "V6 GRAY_BOUNDARY_SUPPORT_RATIO must be finite and in (0,1]")
    target_radius = getattr(
        config, "GRAY_BOUNDARY_TARGET_RADIUS_CELLS", None)
    if (not isinstance(target_radius, (int, float)) or
            isinstance(target_radius, bool) or target_radius <= 0 or
            target_radius > max(config.SURFACE_GRID_WIDTH,
                                config.SURFACE_GRID_HEIGHT) or
            target_radius != target_radius or
            target_radius == float("inf")):
        raise ValueError(
            "V6 GRAY_BOUNDARY_TARGET_RADIUS_CELLS must be finite and "
            "positive within the grid")
    target_support_bounds = (
        ("GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT",
         getattr(config, "GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT", None),
         config.SURFACE_TRACE_MAX_SECTIONS),
        ("GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT",
         getattr(config, "GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT", None),
         config.ROAD_PARAMS.get("ROAD_SEED_ROWS")),
    )
    for name, value, maximum in target_support_bounds:
        if not isinstance(maximum, int) or isinstance(maximum, bool):
            raise ValueError("V6 %s bound must be a positive integer" % name)
        if (not isinstance(value, int) or isinstance(value, bool) or
                value < 1 or value > maximum):
            raise ValueError(
                "V6 %s must be an integer in 1..%d" % (name, maximum))
    bounded_positive = (
        ("SURFACE_TRACE_MAX_SECTIONS", config.SURFACE_TRACE_MAX_SECTIONS, 32),
        ("SURFACE_RAY_MAX_STEPS", config.SURFACE_RAY_MAX_STEPS, 80),
        ("SURFACE_SEED_TRIALS_MAX", config.SURFACE_SEED_TRIALS_MAX, 6),
        ("SURFACE_TRACE_STEP_CELLS", config.SURFACE_TRACE_STEP_CELLS, 16),
        ("SURFACE_MIN_FRESH_SECTIONS", config.SURFACE_MIN_FRESH_SECTIONS, 32),
        ("ROAD_ROW_STEP", config.ROAD_PARAMS.get("ROAD_ROW_STEP"), 60),
        ("ROAD_SEED_ROWS", config.ROAD_PARAMS.get("ROAD_SEED_ROWS"), 20),
        ("ROAD_TRACK_BEAM", config.ROAD_PARAMS.get("ROAD_TRACK_BEAM"), 48),
        ("ROAD_BRANCH_OPTIONS", config.ROAD_PARAMS.get("ROAD_BRANCH_OPTIONS"), 8),
    )
    for name, value, maximum in bounded_positive:
        if (not isinstance(value, int) or isinstance(value, bool) or
                value < 1 or value > maximum):
            raise ValueError("V6 %s must be an integer in 1..%d" %
                             (name, maximum))
    road_params = config.ROAD_PARAMS
    if (road_params.get("ROAD_GRID_WIDTH") != 80 or
            road_params.get("ROAD_GRID_HEIGHT") != 60):
        raise ValueError("V6 road detector must use the fixed 80x60 grid")
    y_reset = config.GRAY_FILTER_REFERENCE_Y_RESET_CELLS
    if (not isinstance(y_reset, (int, float)) or y_reset <= 0 or
            y_reset != y_reset or y_reset == float("inf")):
        raise ValueError(
            "V6 reference-y filter reset threshold must be positive and finite")


def _validate_v6_runtime_apis():
    _load_v6_runtime()
    for filename, module in _V6_RUNTIME_MODULES:
        version = getattr(module, "RUNTIME_API_VERSION", None) \
            if module is not None else None
        if version != V6_RUNTIME_API_VERSION:
            raise RuntimeError(
                "V6 bundle mismatch: %s API is %s, expected %d; upload the "
                "matching V6 openmv bundle and reboot" % (
                    filename, version, V6_RUNTIME_API_VERSION))


def _validate_v7_runtime_apis():
    _load_fast_runtime()
    _load_track_elements_runtime()
    _load_ring_runtime()
    for filename, module in _V7_RUNTIME_MODULES:
        version = getattr(module, "RUNTIME_API_VERSION", None)
        if version != V7_RUNTIME_API_VERSION:
            raise RuntimeError(
                "V7 bundle mismatch: %s API is %s, expected %d; install a "
                "matching V7 bundle" % (
                    filename, version, V7_RUNTIME_API_VERSION))
    if (getattr(vision_protocol, "VERSION", None) != 2 or
            getattr(vision_protocol, "VERSION_V3", None) != 3 or
            getattr(vision_protocol, "FRAME_SIZE", None) != 16):
        raise RuntimeError("V7 requires the 16-byte UART V2/V3 protocol")


def _validate_fast_config():
    _validate_v7_runtime_apis()
    if not isinstance(getattr(config, "STEERING_DIAG_ENABLED", None), bool):
        raise ValueError("V7 STEERING_DIAG_ENABLED must be bool")
    if getattr(track_elements, "JOINT_ROUTE_API_VERSION", None) != 1:
        raise RuntimeError("v26 requires matching track_elements.py with route recovery")
    if getattr(track_elements, "ENTRY_TRANSITION_API_VERSION", None) != 1:
        raise RuntimeError("v26 requires matching track_elements.py with entry transition")
    if getattr(ring_diagnostics, "FORMAT_VERSION", None) != 3:
        raise RuntimeError("v26 requires matching ring_diagnostics.py format3")
    if getattr(config, "TRACK_PACKAGE_BUILD_ID", None) != \
            "CARTEST_H750_RING_ENTRY_V26R9_20261031":
        raise ValueError("V7 track package identity is missing or mismatched")
    if not isinstance(getattr(config, "RING_DIAGNOSTICS_ENABLED", None), bool):
        raise ValueError("V7 RING_DIAGNOSTICS_ENABLED must be bool")
    if getattr(config, "RING_WIDTH_CALIBRATION_REQUIRED", None) is not True:
        raise ValueError("V7 requires the parked ring-width calibration gate")
    for path_name in ("RING_WIDTH_CALIBRATION_PATH",
                      "RING_DIAGNOSTICS_PATH"):
        value = getattr(config, path_name, None)
        if not isinstance(value, str) or not value:
            raise ValueError("V7 %s must be a non-empty path" % path_name)
    control_profile = getattr(config, "STEERING_CONTROL_PROFILE", None)
    if control_profile not in (
            "no_pd", "pd", "pd_preview", "geometry_perf_only",
            "coordinated"):
        raise ValueError(
            "V7 STEERING_CONTROL_PROFILE must be no_pd, pd, pd_preview, "
            "geometry_perf_only or coordinated")
    expected_profile_build = dict(
        CACHED_WIDTH_CONTROL_PROFILE_BUILD_IDS).get(control_profile)
    if getattr(config, "BUILD_ID", None) != expected_profile_build:
        raise RuntimeError(
            "V7 control profile/build mismatch; install its matching config.py")
    contour_enabled = getattr(config, "CONTOUR_RECOVERY_ENABLED", None)
    if not isinstance(contour_enabled, bool):
        raise ValueError("V7 CONTOUR_RECOVERY_ENABLED must be bool")
    if contour_enabled:
        _load_contour_runtime()
    if getattr(fast_path, "CONFIDENCE_DIAGNOSTIC_ONLY", False) is not True:
        raise RuntimeError(
            "single-edge bundle mismatch: fast_path confidence gate mode")
    _load_singleedge_runtime()
    for filename, module in _SINGLEEDGE_RUNTIME_MODULES:
        version = getattr(module, "RUNTIME_API_VERSION", None)
        expected_version = (
            REVERSE_CONTROL_RUNTIME_API_VERSION
            if filename == "reverse_control.py" else
            SINGLEEDGE_RUNTIME_API_VERSION)
        if version != expected_version:
            raise RuntimeError(
                "single-edge bundle mismatch: %s API is %s, expected %d; "
                "install a matching bundle" % (
                    filename, version, expected_version))
    if (getattr(config, "PROFILE_STATE_REVISION", None) !=
            reverse_control.PROFILE_REVISION or
            getattr(config, "PROFILE_STATE_TIMEOUT_MS", None) !=
            reverse_control.PROFILE_TIMEOUT_MS or
            getattr(config, "RING_ENABLED_PROFILE_IDS", None) !=
            (reverse_control.PROFILE_B, reverse_control.PROFILE_C)):
        raise RuntimeError(
            "profile-state config/parser revision, timeout or ring IDs mismatch")
    configured_route = getattr(config, "SINGLE_EDGE_ROUTE_MODE", None)
    configured_build = getattr(config, "BUILD_ID", None)
    matched_build = False
    for route_mode, build_id in SINGLEEDGE_ROUTE_BUILD_IDS:
        if configured_route == route_mode and configured_build == build_id:
            matched_build = True
            break
    if getattr(config, "SINGLEEDGE_RUNTIME_API_VERSION", None) != \
            SINGLEEDGE_RUNTIME_API_VERSION or not matched_build or \
            getattr(config, "V7_RUNTIME_API_VERSION", None) != \
            V7_RUNTIME_API_VERSION:
        raise RuntimeError(
            "single-edge config/build mismatch (route/build mismatch): only "
            "supported matching route and build ID pairs are accepted")
    if getattr(config, "VISION_BACKEND", None) not in \
            ("fast", "compare", "legacy"):
        raise ValueError("VISION_BACKEND must be fast, compare or legacy")
    if not isinstance(getattr(config, "OVERLAY_ENABLED", None), bool):
        raise ValueError("V7 OVERLAY_ENABLED must be bool")
    overlay_period = getattr(config, "OVERLAY_PERIOD_MS", None)
    if (not isinstance(overlay_period, int) or isinstance(overlay_period, bool)
            or overlay_period < 1 or overlay_period > 200):
        raise ValueError("V7 OVERLAY_PERIOD_MS must be in 1..200 ms")
    usb_preview_enabled = getattr(config, "USB_PREVIEW_ENABLED", None)
    if not isinstance(usb_preview_enabled, bool):
        raise ValueError("USB_PREVIEW_ENABLED must be bool")
    if not isinstance(getattr(
            config, "PREVIEW_FPS_MARKER_ENABLED", None), bool):
        raise ValueError("PREVIEW_FPS_MARKER_ENABLED must be bool")
    usb_preview_quality = getattr(config, "USB_PREVIEW_QUALITY", None)
    if (not isinstance(usb_preview_quality, int) or
            isinstance(usb_preview_quality, bool) or
            usb_preview_quality < 1 or usb_preview_quality > 100):
        raise ValueError("USB_PREVIEW_QUALITY must be an integer in 1..100")
    if getattr(config, "USB_PREVIEW_OVERLAY_MODE", None) not in (
            "compact", "full"):
        raise ValueError("USB_PREVIEW_OVERLAY_MODE must be compact or full")
    if getattr(config, "USB_PREVIEW_BUILD_ID", None) !=             "OPENMV_USB60_CANDIDATE_20261024":
        raise RuntimeError("USB preview candidate config/build mismatch")
    if getattr(config, "SURFACE_MODE", None) != "gray_unified":
        raise ValueError("V7 requires the single gray_unified camera path")
    if (config.FRAME_SIZE_NAME != "QVGA" or
            config.IMAGE_WIDTH != 320 or config.IMAGE_HEIGHT != 120 or
            config.SENSOR_WINDOW_X != 0 or config.SENSOR_WINDOW_Y != 60 or
            config.SURFACE_GRID_WIDTH != 80 or
            config.SURFACE_GRID_HEIGHT != 60 or
            config.SURFACE_PIXEL_SCALE_X != 4 or
            config.SURFACE_PIXEL_SCALE_Y != 2):
        raise ValueError(
            "V7 requires QVGA center crop 320x120 and an 80x60 4x2 grid")
    expected_blocks = (
        (0, 8, 80, 20), (0, 20, 80, 32),
        (0, 32, 80, 46), (0, 46, 80, 60),
    )
    if (config.SURFACE_ROI_FLOOR_Y != 8 or
            config.ROAD_PARAMS.get("ROAD_FLOOR_Y") != 8 or
            tuple(config.GRAY_THRESHOLD_BLOCKS) != expected_blocks):
        raise ValueError(
            "turn-response ROI must use floor=8 and the four ordered blocks")
    if (getattr(config, "STEERING_CORE_RAW_Y_MIN", None) != 40 or
            getattr(config, "STEERING_CORE_RAW_Y_MAX", None) != 79 or
            getattr(config, "STEERING_CORE_MIN_SUPPORT_ROWS", None) != 3 or
            getattr(config, "STEERING_CORE_MIN_SUPPORT_SPAN_GRID", None) != 2 or
            len(getattr(config, "STEERING_ROW_WEIGHT_Q", ())) != 60 or
            getattr(config, "STEERING_ROW_WEIGHT_Q_SCALE", None) != 1000):
        raise ValueError("parabolic row-weight config/build mismatch")
    if (config.POSITION_PD_DERIVATIVE_TAU_MS != 60 or
            config.POSITION_PD_TIME_MS != 30 or
            config.POSITION_PD_D_LIMIT != 100 or
            config.POSITION_PD_MAX_DT_MS != 100 or
            config.STEERING_OUTPUT_LIMIT != 1000 or
            config.PREVIEW_CANDIDATE_MIN_ABS != 80 or
            config.PREVIEW_CONFIRM_FRAMES != 3 or
            config.PREVIEW_CONFIRM_MS != 60 or
            config.PREVIEW_POSITION_MIN_ABS != 50 or
            config.PREVIEW_POSITION_FULL_GAIN_ABS != 250 or
            config.PREVIEW_MAX_GAIN not in (0.35, 0.45) or
            config.PREVIEW_APPLIED_LIMIT not in (100, 120) or
            (config.PREVIEW_MAX_GAIN == 0.35 and
             config.PREVIEW_APPLIED_LIMIT != 100) or
            (config.PREVIEW_MAX_GAIN == 0.45 and
             config.PREVIEW_APPLIED_LIMIT != 120) or
            config.PREVIEW_SLEW_PER_SECOND != 500):
        raise ValueError("position PD/preview config/build mismatch")
    if (not isinstance(getattr(config, "SPEED_STAGE_ENABLED", None), bool) or
            config.SPEED_CANDIDATE_MIN_ABS != 80 or
            config.SPEED_CANDIDATE_CONFIRM_FRAMES != 3 or
            config.SPEED_CANDIDATE_CONFIRM_MS != 60 or
            config.SPEED_RELEASE_POSITION_ABS != 50 or
            config.SPEED_RELEASE_CANDIDATE_ABS != 80 or
            config.SPEED_RELEASE_CONFIRM_MS != 300 or
            config.SPEED_EXIT_PHASE_MS != 800 or
            config.SPEED_SLOWDOWN_REQUEST_PERMILLE != 1000):
        raise ValueError("speed-stage config/build mismatch")
    if getattr(config, "OBSERVATION_FILTER_ALPHA", None) != 0.70:
        raise ValueError("turn-response filter alpha must remain 0.70")
    if (isinstance(config.SURFACE_MIN_CONFIDENCE, bool) or
            not isinstance(config.SURFACE_MIN_CONFIDENCE, int) or
            not 0 <= config.SURFACE_MIN_CONFIDENCE <= 100):
        raise ValueError("quality diagnostic threshold must be in 0..100")
    source_shift = getattr(
        config, "SINGLE_EDGE_FILTER_REFERENCE_SHIFT_CELLS", None)
    if (not isinstance(source_shift, (int, float)) or
            isinstance(source_shift, bool) or source_shift < 0 or
            source_shift > config.SURFACE_GRID_WIDTH or
            source_shift != source_shift or source_shift == float("inf")):
        raise ValueError(
            "single-edge filter reference shift must be finite and bounded")
    exposure_cap = getattr(config, "AUTO_EXPOSURE_MAX_US", None)
    if config.AUTO_EXPOSURE and (not isinstance(exposure_cap, int) or
            isinstance(exposure_cap, bool) or
            exposure_cap < 1000 or exposure_cap > 100000):
        raise ValueError("V7 AUTO_EXPOSURE_MAX_US must be in 1000..100000")
    # Keep both controller-frame protections at or below the existing limit.
    if not 0 < config.SURFACE_FRAME_BUDGET_MS <= 200:
        raise ValueError("V7 frame budget must remain in 1..200 ms")
    if config.UART_ID != 3 or config.UART_BAUDRATE != 115200 or \
            config.UART_BITS != 8 or config.UART_STOP_BITS != 1:
        raise ValueError("V7 UART must remain UART3, 115200 8N1")
    if getattr(config, "PERF_WINDOW_MS", None) != 10000:
        raise ValueError("V7 performance windows must remain 10000 ms")
    gc_period = getattr(config, "PERF_GC_PERIOD_FRAMES", None)
    if (not isinstance(gc_period, int) or isinstance(gc_period, bool) or
            gc_period < 1 or gc_period > 1000):
        raise ValueError("V7 PERF_GC_PERIOD_FRAMES must be in 1..1000")
    gc_max_interval = getattr(
        config, "PERF_GC_MAX_INTERVAL_FRAMES", None)
    if (not isinstance(gc_max_interval, int) or
            isinstance(gc_max_interval, bool) or
            gc_max_interval < 1 or gc_max_interval > 120):
        raise ValueError("V7 GC max interval must be in 1..120 frames")
    gc_min_free = getattr(config, "PERF_GC_MIN_FREE_BYTES", None)
    if (not isinstance(gc_min_free, int) or isinstance(gc_min_free, bool) or
            gc_min_free < 1024 * 1024):
        raise ValueError("V7 GC minimum free heap must be at least 1 MiB")
    reserve = getattr(config, "V7_UART_WRITE_RESERVE_US", None)
    if (not isinstance(reserve, int) or isinstance(reserve, bool) or
            reserve < config.UART_TIMEOUT_MS * 1000 or
            reserve > config.UART_TIMEOUT_MS * 1000 + 2000):
        raise ValueError(
            "V7 UART reserve must include its timeout and at most 2 ms guard")
    boundary_step = getattr(config, "GRAY_BOUNDARY_MIN_STEP", None)
    boundary_fraction = getattr(
        config, "GRAY_BOUNDARY_MIN_STEP_FRACTION", None)
    if (not isinstance(boundary_step, (int, float)) or
            isinstance(boundary_step, bool) or boundary_step <= 0 or
            boundary_step > 255 or boundary_step != boundary_step or
            boundary_step == float("inf")):
        raise ValueError("V7 GRAY_BOUNDARY_MIN_STEP must be finite in 1..255")
    if (not isinstance(boundary_fraction, (int, float)) or
            isinstance(boundary_fraction, bool) or boundary_fraction < 0 or
            boundary_fraction >= 1 or boundary_fraction != boundary_fraction or
            boundary_fraction == float("inf")):
        raise ValueError(
            "V7 GRAY_BOUNDARY_MIN_STEP_FRACTION must be finite in [0,1)")
    if config.ROAD_PARAMS.get("ROAD_GRID_WIDTH") != 80 or \
            config.ROAD_PARAMS.get("ROAD_GRID_HEIGHT") != 60:
        raise ValueError("V7 fast path requires the fixed 80x60 grid")
    return True


def _validate_gray_config():
    """Compatibility validator selected by the explicit runtime backend."""
    if getattr(config, "VISION_BACKEND", None) == "fast":
        return _validate_fast_config()
    return _validate_v6_gray_config()


def _configure_gray_sensor(sensor_module, fast=False):
    """Start the grayscale path and return the camera's reported ID."""
    sensor_module.reset()
    sensor_id = None
    get_id = getattr(sensor_module, "get_id", None)
    if callable(get_id):
        try:
            sensor_id = get_id()
        except Exception:
            sensor_id = None
    sensor_module.set_pixformat(sensor_module.GRAYSCALE)
    sensor_module.set_framesize(getattr(sensor_module,
                                        config.FRAME_SIZE_NAME))
    sensor_module.set_windowing((config.SENSOR_WINDOW_X,
                                 config.SENSOR_WINDOW_Y,
                                 config.IMAGE_WIDTH,
                                 config.IMAGE_HEIGHT))
    sensor_module.set_hmirror(config.MIRROR_X)
    sensor_module.set_vflip(config.FLIP_Y)
    sensor_module.set_auto_gain(False, gain_db=config.GAIN_DB)
    if config.AUTO_EXPOSURE:
        exposure_cap = getattr(config, "AUTO_EXPOSURE_MAX_US", None) \
            if fast else None
        if exposure_cap is None:
            sensor_module.set_auto_exposure(True)
        else:
            sensor_module.set_auto_exposure(True,
                                            exposure_us=exposure_cap)
    else:
        sensor_module.set_auto_exposure(False,
                                        exposure_us=config.EXPOSURE_US)
    sensor_module.skip_frames(time=config.GRAY_SENSOR_WARMUP_MS)
    sensor_module.skip_frames(time=200)
    return sensor_id


def run_ring_width_calibration(raw_capture_path=None):
    """Manually build the private 45 cm ring-width map while stopped.

    With no path, collect fresh frames only after three fresh B/C profile
    status samples confirm no Follow lease and an unchanged wheel counter.
    ``raw_capture_path`` replays a CRC-verified R45R ground recording through
    the same camera image buffer and detector. Both modes transmit only
    invalid V3 frames with profile ACK 0. The JSON calibration is atomically
    saved only after 64 accepted stable frames pass all geometry checks.
    """
    _validate_fast_config()
    calibration_type = ring_width_calibration.RingWidthCalibration
    collector = calibration_type()
    expected_sensor_id = ring_width_calibration.SENSOR_ID
    sensor_id = _configure_gray_sensor(sensor, fast=True)
    if sensor_id != expected_sensor_id:
        result = {
            "ready": False, "reason": "SENSOR_ID_MISMATCH",
            "sensor_id": sensor_id,
        }
        print("RING_WIDTH_CALIBRATION ready=0 reason=%s sensor_id=%s" % (
            result["reason"], str(sensor_id)))
        return result

    raw_file = None
    raw_frame_count = None
    raw_expected_crc = None
    raw_crc = 0
    first_error = None
    rejection_names = (
        "PARKED_PROFILE_UNAVAILABLE", "ROAD_PATH_INVALID",
        "TOPOLOGY_NOT_CLEAR", "BOUNDARY_REJECTED",
        "REFERENCE_REJECTED", "BILATERAL_ROWS_MISSING",
        "WAITING_FOR_STABLE_STRAIGHT", "INSUFFICIENT_BILATERAL_ROWS",
        "WIDTH_OUT_OF_RANGE", "BAD_STRAIGHT_FIT", "ROW_GAP_TOO_LARGE",
        "INVALID_ROW_SHAPE", "INVALID_ROW_GEOMETRY",
        "COLLECTION_TIMEOUT", "COLLECTION_ERROR",
        "OTHER_CALIBRATION_REJECTION",
    )
    rejection_counts = array("i", [0] * len(rejection_names))
    attempted_frames = 0
    first_rejection = [None]

    def _reason_index(reason):
        if reason == "INVALID_ROW_SHAPE":
            return 11
        if reason == "INVALID_ROW_GEOMETRY":
            return 12
        if reason == "COLLECTION_TIMEOUT":
            return 13
        if reason == "COLLECTION_ERROR":
            return 14
        if reason == "WAITING_FOR_STABLE_STRAIGHT":
            return 6
        if reason == "INSUFFICIENT_BILATERAL_ROWS":
            return 7
        if reason == "WIDTH_OUT_OF_RANGE":
            return 8
        if reason == "BAD_STRAIGHT_FIT":
            return 9
        if reason == "ROW_GAP_TOO_LARGE":
            return 10
        return 15

    def _reject(index, reason=None, rows=None, count=0, stride=0):
        if reason is not None:
            index = _reason_index(reason)
        if 0 <= index < len(rejection_counts):
            rejection_counts[index] += 1
        if first_rejection[0] is None:
            prefix = array("i")
            if rows is not None:
                try:
                    limit = min(len(rows), 8)
                    for row_index in range(limit):
                        prefix.append(int(rows[row_index]))
                except (TypeError, ValueError, IndexError):
                    prefix = array("i")
            first_rejection[0] = {
                "attempt": attempted_frames,
                "reason": rejection_names[index]
                if 0 <= index < len(rejection_names) else "UNKNOWN",
                "source_reason": (str(reason)[:40]
                                  if reason is not None else ""),
                "row_count": count,
                "row_stride": stride,
                "row_prefix": tuple(prefix),
            }

    def _rejection_summary():
        summary = {
            "reject_reasons": rejection_names,
            "reject_counts": tuple(rejection_counts),
        }
        if first_rejection[0] is not None:
            summary["first_rejection"] = first_rejection[0]
        return summary

    def _compact_result(ready, reason, finalize_reason=None,
                        valid_rows=None, reference_width_cells=None):
        summary = {
            "ready": bool(ready), "reason": reason,
            "accepted_frames": collector.accepted_frames,
            "attempted_frames": attempted_frames,
            "first_error": first_error,
        }
        if finalize_reason is not None:
            summary["finalize_reason"] = finalize_reason
        if valid_rows is not None:
            summary["valid_rows"] = valid_rows
        if reference_width_cells is not None:
            summary["reference_width_cells"] = reference_width_cells
        if raw_capture_path is not None and raw_crc == raw_expected_crc:
            summary["raw_crc32"] = raw_crc
        summary.update(_rejection_summary())
        return summary

    def _print_summary(summary):
        first = summary.get("first_rejection")
        if first is None:
            first_text = "NONE"
        else:
            first_text = "%s@%d rows=%d/%d prefix=%s" % (
                first["reason"], first["attempt"], first["row_count"],
                first["row_stride"], str(first["row_prefix"]))
        print("RING_WIDTH_CALIBRATION ready=%d reason=%s accepted=%d/%d "
              "attempted=%d first=%s rejects=%s error=%s path=%s" % (
                  1 if summary.get("ready") else 0,
                  summary.get("reason", "UNKNOWN"),
                  summary.get("accepted_frames", 0),
                  ring_width_calibration.REQUIRED_FRAMES,
                  summary.get("attempted_frames", 0), first_text,
                  str(summary.get("reject_counts", ())),
                  str(summary.get("first_error"))[:128],
                  str(summary.get("path", ""))))

    mode = "LIVE"
    try:
        if raw_capture_path is not None:
            mode = "GROUND_R45R_REPLAY"
            raw_file = open(raw_capture_path, "rb")
            header = bytearray(12)
            if raw_file.readinto(header) != len(header):
                raise ValueError("raw capture header is truncated")
            if header[0:4] != b"R45R":
                raise ValueError("raw capture magic mismatch")
            raw_width = header[4] | (header[5] << 8)
            raw_height = header[6] | (header[7] << 8)
            raw_frame_count = header[8] | (header[9] << 8)
            raw_frame_bytes = header[10] | (header[11] << 8)
            if (raw_width != 320 or raw_height != 120 or
                    raw_frame_bytes != 320 * 120 or
                    raw_frame_count < ring_width_calibration.REQUIRED_FRAMES or
                    raw_frame_count > ring_width_calibration.MAX_ATTEMPTS):
                raise ValueError("raw capture geometry or frame count mismatch")
            manifest_path = str(raw_capture_path)
            if manifest_path.endswith(".bin"):
                manifest_path = manifest_path[:-4] + ".json"
            try:
                import ujson as raw_json
            except ImportError:
                import json as raw_json
            with open(manifest_path, "r") as manifest_file:
                manifest = raw_json.loads(manifest_file.read())
            if (manifest.get("wheel_state") != "on_ground_user_confirmed" or
                    manifest.get("purpose") !=
                    "stationary45cmstraight_ground_geometry" or
                    manifest.get("sensor_id") != expected_sensor_id or
                    manifest.get("width") != 320 or
                    manifest.get("height") != 120 or
                    manifest.get("frames") != raw_frame_count or
                    manifest.get("bytes_per_frame") != raw_frame_bytes or
                    manifest.get("crop") != [0, 60, 320, 120]):
                raise ValueError("raw capture manifest is not the ground setup")
            raw_expected_crc = manifest.get("raw_crc32")
            if (isinstance(raw_expected_crc, bool) or
                    not isinstance(raw_expected_crc, int) or
                    raw_expected_crc < 0 or raw_expected_crc > 0xFFFFFFFF):
                raise ValueError("raw capture manifest CRC is invalid")
            expected_file_size = 12 + raw_frame_count * raw_frame_bytes
            raw_file.seek(0, 2)
            if raw_file.tell() != expected_file_size:
                raise ValueError("raw capture length does not match its header")
            raw_file.seek(12)

        uart = _VisionUARTWriter(pyb.UART(
            config.UART_ID, config.UART_BAUDRATE,
            bits=config.UART_BITS, parity=None,
            stop=config.UART_STOP_BITS,
            timeout=config.UART_TIMEOUT_MS,
            timeout_char=config.UART_CHAR_TIMEOUT_MS))
        reverse_parser = reverse_control.ReverseControlParser(
            reverse_control.DEFAULT_TIMEOUT_MS,
            config.PROFILE_STATE_TIMEOUT_MS)
        reverse_buffer = bytearray(reverse_control.MAX_FEED_BYTES)
        reverse_drain_state = array("i", [0])
        packet = bytearray(vision_protocol.FRAME_SIZE)
        sequence = 0
        last_profile_sequence = None
        last_profile_id = None
        last_profile_token = None
        last_wheel_position = None
        stationary_status_frames = 0
        gray_builder = fast_gray.FastGrayMasks(config)
        path_detector = fast_path.FastPathDetector(config)
        boundary_validator = fast_boundary.FastBoundaryEvidence(config)

        # ``bytearray()`` is a by-reference framebuffer view in the pinned
        # camera firmware. Reusing this one image avoids retaining 96 frames.
        img = sensor.snapshot()
        if (img.width() != 320 or img.height() != 120):
            raise ValueError("calibration image must be exactly 320x120")
        image_bytes = img.bytearray()
        if len(image_bytes) != 320 * 120:
            raise ValueError("calibration framebuffer is not 38400 bytes")
        attempts = (raw_frame_count if raw_capture_path is not None else
                    ring_width_calibration.MAX_ATTEMPTS)

        print("RING_WIDTH_CALIBRATION_BEGIN mode=%s sensor=0x%04X "
              "crop=(0,60,320,120) grid=80x60 frames=%d ack=0" % (
                  mode, sensor_id, attempts))
        for attempt in range(attempts):
            attempted_frames = attempt + 1
            vision_protocol.pack_into_v3(
                packet, sequence, 0, 0, 0, 0, 0, 0, 0)
            written = uart.write(packet)
            if written != len(packet):
                raise OSError("invalid V3 frame write %s/16" % str(written))
            sequence = (sequence + 1) & 0xFFFF
            now_ms = time.ticks_ms()
            reverse_fresh = _service_reverse_control(
                uart, reverse_parser, reverse_buffer, reverse_drain_state,
                now_ms, time.ticks_diff)
            profile_id = reverse_parser.profile_ack(
                now_ms, time.ticks_diff)
            parked = raw_capture_path is not None
            if raw_capture_path is None:
                profile_sequence = getattr(
                    reverse_parser, "_profile_last_sequence", None)
                profile_token = reverse_parser.session_token
                wheel_position = reverse_parser.mean_forward_wheel_millirev
                if (profile_id not in config.RING_ENABLED_PROFILE_IDS or
                        reverse_parser.profile_session_active):
                    stationary_status_frames = 0
                    last_profile_sequence = None
                    last_profile_id = None
                    last_profile_token = None
                    last_wheel_position = None
                elif profile_sequence != last_profile_sequence:
                    if (profile_id == last_profile_id and
                            profile_token == last_profile_token and
                            wheel_position == last_wheel_position):
                        stationary_status_frames += 1
                    else:
                        stationary_status_frames = 1
                    last_profile_sequence = profile_sequence
                    last_profile_id = profile_id
                    last_profile_token = profile_token
                    last_wheel_position = wheel_position
                parked = stationary_status_frames >= 3

            if raw_capture_path is not None:
                received = raw_file.readinto(image_bytes)
                if received != len(image_bytes):
                    raise ValueError("raw capture frame %d is truncated" %
                                     attempt)
                try:
                    import binascii as raw_binascii
                    raw_crc = raw_binascii.crc32(image_bytes, raw_crc)
                except (ImportError, TypeError):
                    # The deployed MicroPython binascii API supports an
                    # incremental seed; hosts use the same API.
                    raise RuntimeError("incremental CRC32 is unavailable")
            else:
                img = sensor.snapshot()
                if img.width() != 320 or img.height() != 120:
                    raise ValueError("calibration frame is not exactly 320x120")
                image_bytes = img.bytearray()

            if collector.collection_complete:
                if raw_capture_path is None:
                    break
                # Replay still reads every recorded frame into the streaming
                # CRC before the calibration file can be published.
                continue
            if not parked:
                _reject(0)
                continue
            if raw_capture_path is not None:
                # A ground-confirmed manifest is the stationary evidence for
                # this replay; live profile state still receives ACK 0 above.
                pass
            elif reverse_parser.profile_session_active:
                _reject(0)
                continue

            mask_info = gray_builder.build(img)
            road_result = path_detector.detect(mask_info)
            topology = road_result.get("topology_evidence") or {}
            if not road_result.get("valid"):
                _reject(1)
                continue
            if (topology.get("reason") != "OK" or
                    not topology.get("complete") or
                    topology.get("ambiguous") or
                    topology.get("island_evidence") or
                    topology.get("branch_evidence")):
                _reject(2)
                continue
            boundary_validator.gate_fast_path_candidate(
                road_result, mask_info.get("luma"),
                mask_info.get("edge_mask"), mask_info.get("road_mask"),
                mask_info.get("known_region"), config)
            if not road_result.get("valid"):
                _reject(3)
                continue
            selection = {
                "valid": True, "source": "road", "result": road_result,
                "reason": road_result.get("reason", "OK"),
            }
            selection, reference, boundary_reference = \
                _validate_selected_fast_boundary(
                    selection, road_result, boundary_validator)
            if not selection.get("valid") or reference is None:
                _reject(4)
                continue
            approved = _approved_bilateral_cache_candidate(
                selection, road_result, boundary_reference, mask_info)
            if approved is None:
                _reject(5)
                continue
            rows = road_result.get("selected_row_ints")
            count = road_result.get("selected_row_count", 0)
            stride = road_result.get("selected_row_stride", 0)
            accepted = collector.accept_rows(rows, count, stride)
            if not accepted:
                reason = collector.reason
                _reject(0, reason, rows, count, stride)

            if (raw_capture_path is None and
                    collector.collection_complete):
                break

        if (raw_capture_path is not None and
                raw_crc != raw_expected_crc):
            raise ValueError("raw capture CRC mismatch")
        if not collector.collection_complete:
            finalized = collector.finalize()
            result = _compact_result(
                False, "COLLECTION_INCOMPLETE",
                finalize_reason=finalized.get("reason", "NOT_COMPLETE"))
            _print_summary(result)
            return result

        finalized = collector.finalize()
        if not finalized.get("ready"):
            result = _compact_result(
                False, finalized.get("reason", "FINALIZE_FAILED"))
            _print_summary(result)
            return result
        collector.save(config.RING_WIDTH_CALIBRATION_PATH)
        loaded = calibration_type.load(config.RING_WIDTH_CALIBRATION_PATH)
        if not loaded.ready:
            raise ValueError("saved calibration failed reload: %s" %
                             loaded.reason)
        result = _compact_result(
            True, "CALIBRATION_READY", valid_rows=loaded.valid_rows,
            reference_width_cells=loaded.width_cells[
                ring_width_calibration.REFERENCE_Y])
        result["path"] = config.RING_WIDTH_CALIBRATION_PATH
        _print_summary(result)
        return result
    except Exception as exc:
        first_error = "%s:%s" % (type(exc).__name__, str(exc))
        _reject(14, "COLLECTION_ERROR")
        result = _compact_result(False, "COLLECTION_ERROR")
        _print_summary(result)
        return result
    finally:
        if raw_file is not None:
            try:
                raw_file.close()
            except Exception:
                pass


def _surface_pixel(point):
    scale_x = config.SURFACE_PIXEL_SCALE_X
    scale_y = config.SURFACE_PIXEL_SCALE_Y
    return (int(point[0] * scale_x + scale_x * 0.5 + 0.5),
            int(point[1] * scale_y + scale_y * 0.5 + 0.5))


@micropython.native
def _rounded_local(value):
    return int(value + 0.5) if value >= 0 else int(value - 0.5)


# Keep the live route shape while limiting per-frame drawing calls on the
# 320x120 preview. Endpoints always remain in the evenly spaced sample set.
_PREVIEW_MAX_ROW_POINTS = 7
# The default preserves the existing detail; tests/REPL can lower this during
# display-only A/B checks without changing the vision or control pipeline.
_PREVIEW_MAX_BOUNDARY_SEGMENTS = 3


def _preview_row_sample_count(row_count):
    return row_count if row_count < _PREVIEW_MAX_ROW_POINTS else \
        _PREVIEW_MAX_ROW_POINTS


def _preview_row_sample_index(row_count, sample_index, sample_count):
    if sample_count <= 1:
        return 0
    return sample_index * (row_count - 1) // (sample_count - 1)


@micropython.native
def _draw_sparse_surface_contour(img, sections):
    count = len(sections)
    if count < 2:
        return
    step = (count + 2) // 3
    previous_index = 0
    index = step
    while index < count:
        before = sections[previous_index]
        current = sections[index]
        left0 = _surface_pixel(before["left"])
        left1 = _surface_pixel(current["left"])
        right0 = _surface_pixel(before["right"])
        right1 = _surface_pixel(current["right"])
        img.draw_line((left0[0], left0[1], left1[0], left1[1]), color=96)
        img.draw_line((right0[0], right0[1], right1[0], right1[1]), color=96)
        previous_index = index
        index += step
    if previous_index != count - 1:
        before = sections[previous_index]
        current = sections[count - 1]
        left0 = _surface_pixel(before["left"])
        left1 = _surface_pixel(current["left"])
        right0 = _surface_pixel(before["right"])
        right1 = _surface_pixel(current["right"])
        img.draw_line((left0[0], left0[1], left1[0], left1[1]), color=96)
        img.draw_line((right0[0], right0[1], right1[0], right1[1]), color=96)


def _road_boundary_point(row, side):
    boundary = row.get("boundary")
    measured = boundary.get(side) if isinstance(boundary, dict) else None
    if measured is not None and len(measured) >= 1:
        return _surface_pixel((measured[0], row["y"]))
    return _surface_pixel((row[side], row["y"]))


@micropython.native
def _draw_sparse_road_contour(img, rows):
    count = len(rows)
    if count < 2:
        return
    if _PREVIEW_MAX_BOUNDARY_SEGMENTS <= 1:
        before = rows[0]
        current = rows[count - 1]
        left0 = _road_boundary_point(before, "left")
        left1 = _road_boundary_point(current, "left")
        right0 = _road_boundary_point(before, "right")
        right1 = _road_boundary_point(current, "right")
        img.draw_line((left0[0], left0[1], left1[0], left1[1]), color=96)
        img.draw_line((right0[0], right0[1], right1[0], right1[1]), color=96)
        return
    step = (count + 2) // 3
    previous_index = 0
    index = step
    while index < count:
        before = rows[previous_index]
        current = rows[index]
        left0 = _road_boundary_point(before, "left")
        left1 = _road_boundary_point(current, "left")
        right0 = _road_boundary_point(before, "right")
        right1 = _road_boundary_point(current, "right")
        img.draw_line((left0[0], left0[1], left1[0], left1[1]), color=96)
        img.draw_line((right0[0], right0[1], right1[0], right1[1]), color=96)
        previous_index = index
        index += step
    if previous_index != count - 1:
        before = rows[previous_index]
        current = rows[count - 1]
        left0 = _road_boundary_point(before, "left")
        left1 = _road_boundary_point(current, "left")
        right0 = _road_boundary_point(before, "right")
        right1 = _road_boundary_point(current, "right")
        img.draw_line((left0[0], left0[1], left1[0], left1[1]), color=96)
        img.draw_line((right0[0], right0[1], right1[0], right1[1]), color=96)


def _draw_contour_trace_side(img, points, support, count, side):
    """Mark sampled trace points that carry current known-dark support."""
    if not isinstance(points, array) or not isinstance(support, bytearray):
        return
    if (isinstance(count, bool) or not isinstance(count, int) or count < 0):
        return
    count = min(count, len(support), len(points) // 2)
    supported_count = 0
    for index in range(count):
        offset = index * 2
        y = points[offset]
        x = points[offset + 1]
        if (support[index] == 1 and
                config.SURFACE_ROI_FLOOR_Y <= y < config.SURFACE_GRID_HEIGHT
                and 0 < x < config.SURFACE_GRID_WIDTH - 1):
            supported_count += 1
    sample_count = _preview_row_sample_count(supported_count)
    for sample_index in range(sample_count):
        target = _preview_row_sample_index(
            supported_count, sample_index, sample_count)
        ordinal = 0
        for index in range(count):
            offset = index * 2
            y = points[offset]
            x = points[offset + 1]
            if (support[index] != 1 or
                    y < config.SURFACE_ROI_FLOOR_Y or
                    y >= config.SURFACE_GRID_HEIGHT or x <= 0 or
                    x >= config.SURFACE_GRID_WIDTH - 1):
                continue
            if ordinal == target:
                boundary_x = float(x) - 0.5 if side == "left" else \
                    float(x) + 0.5
                img.draw_cross(_surface_pixel((boundary_x, y)), color=96)
                break
            ordinal += 1


def _draw_contour_route_overlay(img, result, trace_info):
    """Show current supported trace edges and validator-approved row mids."""
    if (not isinstance(trace_info, dict) or
            trace_info.get("used") is not True or
            not isinstance(result, dict) or not result.get("valid")):
        return
    _draw_contour_trace_side(
        img, trace_info.get("left_points"),
        trace_info.get("left_dark_support"), trace_info.get("Lcount", 0),
        "left")
    _draw_contour_trace_side(
        img, trace_info.get("right_points"),
        trace_info.get("right_dark_support"), trace_info.get("Rcount", 0),
        "right")
    rows = result.get("selected_row_ints")
    count = result.get("selected_row_count", 0)
    support = result.get("boundary_support_rows")
    if (not isinstance(rows, array) or isinstance(count, bool) or
            not isinstance(count, int) or count < 1 or count > 60 or
            result.get("selected_row_stride") != 4 or
            len(rows) < count * 4 or not isinstance(support, bytearray) or
            result.get("boundary_support_count") != count or
            len(support) < count):
        return
    supported_count = 0
    for index in range(count):
        if support[index] == 1:
            supported_count += 1
    sample_count = _preview_row_sample_count(supported_count)
    previous_midpoint = None
    for sample_index in range(sample_count):
        target = _preview_row_sample_index(
            supported_count, sample_index, sample_count)
        ordinal = 0
        for index in range(count):
            if support[index] != 1:
                continue
            if ordinal == target:
                offset = index * 4
                center_x = (rows[offset + 1] + rows[offset + 2]) * 0.5
                midpoint = _surface_pixel((center_x, rows[offset]))
                if previous_midpoint is not None:
                    img.draw_line((previous_midpoint[0], previous_midpoint[1],
                                   midpoint[0], midpoint[1]), color=255)
                previous_midpoint = midpoint
                break
            ordinal += 1


@micropython.native
def _normalize_fast_reference(x, width):
    """Normalize the exact controller point using the protocol's W-1 center."""
    half = (width - 1) * 0.5
    if half <= 0.0:
        raise ValueError("grid must contain at least two columns")
    normalized = ((float(x) - half) / half) * 1000.0
    return _rounded_local(normalized)


def _compose_ordinary_steering_error(position_error, preview_candidate,
                                     preview_enabled=True):
    """Compose ordinary control while retaining preview as a diagnostic.

    The optional argument keeps the host helper's historical default intact;
    the production aggregate path passes the explicit package setting.  A
    disabled preview therefore reports a zero applied contribution while the
    candidate remains available to logs and the overlay.
    """
    if preview_enabled:
        steering_error = position_error + preview_candidate
        applied_contribution = preview_candidate
    else:
        steering_error = position_error
        applied_contribution = 0
    if steering_error < -1000:
        steering_error = -1000
    elif steering_error > 1000:
        steering_error = 1000
    return steering_error, (None if preview_enabled else "DISABLED_BY_CONFIG"), \
        applied_contribution


def _fast_forward_white_continuation(mask_info, road_result, settings):
    """Require current supported bilateral rows on the near forward path."""
    if not mask_info or not road_result or not road_result.get("valid"):
        return False
    rows = road_result.get("selected_row_ints")
    count = road_result.get("selected_row_count", 0)
    support = road_result.get("boundary_support_rows")
    if (not isinstance(rows, array) or not isinstance(count, int) or
            isinstance(count, bool) or count < 1 or count > 60 or
            road_result.get("selected_row_stride") != 4 or
            len(rows) < count * 4 or not isinstance(support, bytearray) or
            len(support) < count or
            road_result.get("boundary_support_count") != count):
        return False
    road = mask_info.get("road_mask")
    known = mask_info.get("known_region")
    width = settings.SURFACE_GRID_WIDTH
    height = settings.SURFACE_GRID_HEIGHT
    if (not isinstance(road, bytearray) or not isinstance(known, bytearray) or
            len(road) != width * height or len(known) != width * height):
        return False
    supported_near = 0
    for index in range(count):
        base = index * 4
        y = rows[base]
        if (support[index] != 1 or rows[base + 3] & 7 != 7 or
                y < height - 10 or y >= height):
            continue
        center_x = (rows[base + 1] + rows[base + 2]) // 2
        cell = y * width + center_x
        if 0 <= center_x < width and known[cell] and road[cell]:
            supported_near += 1
    return supported_near >= 3


def _short_v7_detector_error(error, limit=96):
    """Return bounded printable ASCII for low-rate V7 detector diagnostics."""
    try:
        text = str(error)
    except Exception:
        text = "unprintable"
    result = []
    for character in text:
        code = ord(character)
        result.append(character if 32 <= code <= 126 else "_")
        if len(result) >= limit:
            break
    return "".join(result) or "-"


def _v7_valid_within_budget(valid, packet_ready_age_us, budget_us,
                            write_reserve_us):
    """Reserve the UART timeout before permitting a valid packet on wire."""
    if not valid:
        return False
    return packet_ready_age_us + write_reserve_us <= budget_us


@micropython.native
def _selected_reference(selection):
    """Return the exact grid point used to form the transmitted error."""
    if not selection or not selection.get("valid"):
        return None
    source = selection.get("source")
    result = selection.get("result") or {}
    if source == "surface":
        target = result.get("target")
        if target is None:
            return None
        return {"x": float(target[0]), "y": float(target[1]),
                "kind": "target"}

    if source in ("road", "ring_left", "ring_right"):
        comparison = selection.get("comparison") or {}
        comparison_y = comparison.get("target_y")
        comparison_x = comparison.get("road_center_x")
        if comparison_y is not None and comparison_x is not None:
            return {"x": float(comparison_x), "y": float(comparison_y),
                    "kind": "shared_y"}
        reference_x = result.get("reference_center_x")
        reference_y = result.get("reference_y")
        if reference_x is not None and reference_y is not None:
            return {"x": float(reference_x), "y": float(reference_y),
                    "kind": "reference_y_" + str(
                        result.get("reference_provenance", "OBSERVED")).lower()}

        # A road candidate without a center at the configured row uses its
        # weighted fresh-row center.  Keep the displayed marker tied to that
        # same weighted center and label it so it cannot be mistaken for a
        # fixed lookahead row.
        points = result.get("selected_rows", ())
        weights = result.get("weights", ())
        if points:
            if len(weights) != len(points):
                weights = [1.0 / len(points)] * len(points)
            center_y = 0.0
            for point, weight in zip(points, weights):
                center_y += float(point["y"]) * float(weight)
            center_x = result.get("center_x")
            if center_x is not None:
                return {"x": float(center_x), "y": center_y,
                        "kind": "weighted_center"}
    return None


def _validate_selected_boundary(selection, surface_result, road_result,
                                luma, edge_mask, road_mask, known_region):
    """Fail closed if selector's exact transmitted reference lacks fresh steps."""
    if not selection or not selection.get("valid"):
        return selection, None, {"valid": False, "reason": "SELECTION_INVALID"}
    reference = _selected_reference(selection)
    validation = boundary_evidence.validate_selected_reference(
        selection, reference, surface_result, road_result, config)
    if validation.get("valid"):
        return selection, reference, validation

    rejected_source = selection.get("source", "NONE")
    rejected = dict(selection)
    rejected.update({
        "valid": False,
        "quality": 0,
        "lateral_error": 0,
        "reason": "BOUNDARY_REFERENCE_UNSUPPORTED",
        "rejected_source": rejected_source,
        "source": "NONE",
        "result": None,
    })
    return rejected, None, validation


@micropython.native
def _validate_selected_fast_boundary(selection, road_result, validator):
    """Validate the exact V7 control point with the API-7 fast gate."""
    if not selection or not selection.get("valid"):
        return selection, None, {"valid": False,
                                 "reason": "SELECTION_INVALID"}
    reference = _selected_reference(selection)
    validation = validator.validate_selected_reference(
        selection, reference, None, road_result, config)
    if validation.get("valid"):
        return selection, reference, validation

    rejected_source = selection.get("source", "NONE")
    rejected = dict(selection)
    rejected.update({
        "valid": False,
        "quality": 0,
        "lateral_error": 0,
        "reason": "BOUNDARY_REFERENCE_UNSUPPORTED",
        "rejected_source": rejected_source,
        "source": "NONE",
        "result": None,
    })
    return rejected, None, validation


def _gate_contour_candidate(candidate, prior_result, validator, mask_info,
                            settings):
    """Keep the original path failure if contour rows miss the old gate."""
    try:
        validator.gate_fast_path_candidate(
            candidate, mask_info.get("luma"), mask_info.get("edge_mask"),
            mask_info.get("road_mask"), mask_info.get("known_region"),
            settings)
    except Exception as exc:
        candidate["valid"] = False
        candidate["quality"] = 0
        candidate["reason"] = "BOUNDARY_EXCEPTION"
        candidate["lateral_error"] = 0
        return prior_result, "BOUNDARY_EXCEPTION", exc
    if not candidate.get("valid"):
        return prior_result, "BOUNDARY_%s" % candidate.get(
            "reason", "REJECTED"), None
    return candidate, None, None


def _approved_bilateral_cache_candidate(selection, road_result,
                                        boundary_reference, mask_info):
    """Expose current road rows to the width cache only after both gates."""
    if (not selection or not selection.get("valid") or
            selection.get("source") != "road" or
            selection.get("result") is not road_result or
            not boundary_reference or
            not boundary_reference.get("valid") or mask_info is None or
            not road_result or not road_result.get("valid")):
        return None
    road_result["boundary_source"] = "OBSERVED_BOTH"
    road_result["inferred"] = False
    road_result["inference_flags"] = 0
    road_result["observed_both_rows"] = road_result.get(
        "selected_row_count", 0)
    road_result["observed_left_rows"] = road_result.get(
        "selected_row_count", 0)
    road_result["observed_right_rows"] = road_result.get(
        "selected_row_count", 0)
    road_result["inferred_rows"] = 0
    selection["boundary_source"] = "OBSERVED_BOTH"
    selection["inferred"] = False
    selection["inference_flags"] = 0
    return road_result


@micropython.native
def _fill_ring_control_route(element_result, element_tracker, edge_tracker,
                             road_accepted, single_rows_fresh,
                             route_result, route_rows, settings):
    """Convert current ring targets to the existing fixed single-row format."""
    if (not element_result or
            not element_result.get("control_eligible") or
            not element_result.get("target_valid")):
        route_result["valid"] = False
        route_result["reason"] = "RING_NOT_ELIGIBLE"
        route_result["selected_row_count"] = 0
        return 0

    target_rows = element_result.get("target_rows")
    target_count = element_result.get("target_count", 0)
    target_stride = element_result.get("target_stride")
    height = settings.SURFACE_GRID_HEIGHT
    if (not isinstance(target_rows, array) or
            isinstance(target_count, bool) or
            not isinstance(target_count, int) or
            target_count < element_tracker.min_target_rows or
            target_count > height or
            target_stride != track_elements.TARGET_ROW_STRIDE or
            len(target_rows) < target_count * target_stride):
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        return 0

    left_status = edge_tracker._row_status
    left_edges = edge_tracker._row_edge2
    right_base = height
    output_count = 0
    width_source_rejected = False
    target_side = element_result.get(
        "target_side", element_result.get("side"))
    if target_side not in (track_elements.SIDE_LEFT,
                           track_elements.SIDE_RIGHT):
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        return 0
    reference_y = settings.ROAD_PARAMS.get("ROAD_REFERENCE_Y", 40)
    reference_delta = height + 1
    reference_row_y = -1
    reference_center2 = 0
    reference_edge2 = 0
    reference_boundary = "NONE"
    observed_left = track_elements.TARGET_OBSERVED_LEFT
    observed_right = track_elements.TARGET_OBSERVED_RIGHT
    from_cache = track_elements.TARGET_WIDTH_FROM_CACHE
    from_calibration = track_elements.TARGET_WIDTH_FROM_CALIBRATION
    width_sources = from_cache | from_calibration
    calibration_verified = bool(
        getattr(element_tracker, "ring_calibration_ready", False) and
        getattr(element_tracker, "ring_calibration_check_ready", False))
    calibration_required = bool(getattr(
        element_tracker, "require_calibration", False))
    outer_requested = bool(element_result.get("outer_tracking"))
    outer_check = getattr(element_tracker, "outer_route_valid_for", None)
    outer_verified = bool(outer_requested and callable(outer_check) and
        getattr(track_elements, "RING_OUTER_ROUTE_API_VERSION", 0) == 2 and
        calibration_verified and single_rows_fresh and
        outer_check(element_result))
    if outer_requested and not outer_verified:
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        route_result["reason"] = "RING_OUTER_EDGE_UNVERIFIED"
        return 0
    transition_flag = getattr(track_elements, "TARGET_ENTRY_TRANSITION", 64)
    transition_requested = bool(element_result.get("entry_transition"))
    transition_check = getattr(element_tracker, "entry_transition_valid_for", None)
    transition_verified = bool(
        transition_requested and callable(transition_check) and
        getattr(track_elements, "ENTRY_TRANSITION_API_VERSION", 0) == 1 and
        transition_check(element_result) and calibration_verified and
        single_rows_fresh)
    transition_far_y = int(element_result.get("entry_transition_far_y", -1))
    transition_near_y = int(element_result.get("entry_transition_near_y", -1))
    if transition_requested and (not transition_verified or
            not (settings.SURFACE_ROI_FLOOR_Y <= transition_far_y <
                 reference_y < transition_near_y < height)):
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        route_result["reason"] = "RING_ENTRY_TRANSITION_UNVERIFIED"
        return 0
    transition_inner_count = 0
    transition_inner_run = 0
    transition_inner_longest = 0
    transition_inner_first = height
    transition_inner_last = -1
    transition_outer_count = 0
    transition_outer_run = 0
    transition_outer_longest = 0
    transition_outer_last = -2
    previous_transition_y = -1

    for index in range(target_count):
        source_offset = index * target_stride
        y = int(target_rows[source_offset])
        left_x = int(target_rows[source_offset + 1])
        right_x = int(target_rows[source_offset + 2])
        source_flags = int(target_rows[source_offset + 3])
        outside_view = bool(source_flags & track_elements.TARGET_OUTSIDE_VIEW)
        outside_geometry = left_x < 0 or right_x >= settings.SURFACE_GRID_WIDTH
        if (y < settings.SURFACE_ROI_FLOOR_Y or y >= height or
                left_x >= right_x):
            continue
        if outside_geometry or outside_view:
            if (not outside_view or not outside_geometry or not outer_verified or
                    left_x+right_x < 0 or
                    left_x+right_x > 2*(settings.SURFACE_GRID_WIDTH-1) or
                    (target_side == track_elements.SIDE_LEFT and
                     not (0 <= left_x < settings.SURFACE_GRID_WIDTH and
                          right_x >= settings.SURFACE_GRID_WIDTH)) or
                    (target_side == track_elements.SIDE_RIGHT and
                     not (left_x < 0 and
                          0 <= right_x < settings.SURFACE_GRID_WIDTH))):
                route_result["valid"] = False
                route_result["selected_row_count"] = 0
                route_result["reason"] = "RING_OUTSIDE_VIEW_UNVERIFIED"
                return 0

        has_left = bool(source_flags & observed_left)
        has_right = bool(source_flags & observed_right)
        transition_row = bool(source_flags & transition_flag)
        inferred_bridge = bool(
            transition_verified and transition_row and
            not has_left and not has_right and
            source_flags & track_elements.TARGET_INFERRED_LEFT and
            source_flags & track_elements.TARGET_INFERRED_RIGHT and
            transition_far_y < y < transition_near_y)
        if transition_row != transition_verified:
            route_result["valid"] = False
            route_result["selected_row_count"] = 0
            route_result["reason"] = "RING_ENTRY_TRANSITION_UNVERIFIED"
            return 0
        if transition_verified:
            if y <= previous_transition_y:
                route_result["valid"] = False
                route_result["selected_row_count"] = 0
                route_result["reason"] = "RING_ENTRY_TRANSITION_UNVERIFIED"
                return 0
            previous_transition_y = y
        width_source = source_flags & width_sources
        if ((not has_left and not has_right and not inferred_bridge) or
                width_source not in (from_cache, from_calibration) or
                (width_source == from_calibration and
                 not calibration_verified) or
                (calibration_required and width_source != from_calibration)):
            width_source_rejected = True
            continue
        if single_rows_fresh:
            if ((has_left and left_status[y] != 1) or
                    (has_right and left_status[right_base + y] != 1)):
                continue
            if (transition_verified or outer_verified) and (
                    (has_left and abs(left_x * 2 - 1 - int(left_edges[y])) > 2) or
                    (has_right and abs(right_x * 2 + 1 -
                                      int(left_edges[right_base + y])) > 2)):
                continue
        elif not road_accepted:
            continue

        if single_rows_fresh:
            left_edge2 = int(left_edges[y]) if has_left else left_x * 2 - 1
            right_edge2 = (int(left_edges[right_base + y]) if has_right
                           else right_x * 2 + 1)
        else:
            left_edge2 = left_x * 2 - 1
            right_edge2 = right_x * 2 + 1
        if inferred_bridge:
            observed_side = track_elements.SIDE_NONE
            edge2 = left_x + right_x
        elif has_left and not has_right:
            if not (source_flags & track_elements.TARGET_INFERRED_RIGHT):
                continue
            observed_side = track_elements.SIDE_LEFT
            edge2 = left_edge2
        elif has_right and not has_left:
            if not (source_flags & track_elements.TARGET_INFERRED_LEFT):
                continue
            observed_side = track_elements.SIDE_RIGHT
            edge2 = right_edge2
        elif has_left and has_right:
            observed_side = target_side
            if observed_side == track_elements.SIDE_LEFT:
                edge2 = left_edge2
            elif observed_side == track_elements.SIDE_RIGHT:
                edge2 = right_edge2
            else:
                continue
        else:
            continue

        if ((observed_side == track_elements.SIDE_LEFT and not has_left) or
                (observed_side == track_elements.SIDE_RIGHT and not has_right)):
            continue
        if not transition_verified and observed_side != target_side:
            continue
        if transition_verified and not inferred_bridge:
            expected_side = (target_side if y <= transition_far_y else
                             (track_elements.SIDE_RIGHT if target_side ==
                              track_elements.SIDE_LEFT else track_elements.SIDE_LEFT))
            if (transition_far_y < y < transition_near_y or
                    observed_side != expected_side or has_left == has_right):
                continue
            if y <= transition_far_y:
                transition_inner_count += 1
                if y == transition_inner_last + 1:
                    transition_inner_run += 1
                else:
                    transition_inner_run = 1
                transition_inner_longest = max(
                    transition_inner_longest, transition_inner_run)
                transition_inner_first = min(transition_inner_first, y)
                transition_inner_last = y
            elif y >= max(transition_near_y, height - element_tracker.near_rows):
                transition_outer_count += 1
                if y == transition_outer_last + 1:
                    transition_outer_run += 1
                else:
                    transition_outer_run = 1
                transition_outer_longest = max(
                    transition_outer_longest, transition_outer_run)
                transition_outer_last = y

        if inferred_bridge:
            boundary = "ENTRY_TRANSITION"
        elif has_left and has_right:
            boundary = "OBSERVED_BOTH"
        elif observed_side == track_elements.SIDE_LEFT:
            edge2 = left_edge2
            boundary = "LEFT_ONLY"
        else:
            edge2 = right_edge2
            boundary = "RIGHT_ONLY"

        center2 = left_x + right_x
        target_offset = output_count * fast_single_edge.ROW_STRIDE
        route_rows[target_offset] = y
        route_rows[target_offset + 1] = center2
        route_rows[target_offset + 2] = edge2
        # `side` is the boundary used as the single-edge identity in the
        # existing weighted-row format. Keep it paired with the actual edge
        # and observed flags even when the route's locked ring side differs.
        route_rows[target_offset + 3] = observed_side
        row_flags = 0
        if has_left:
            row_flags |= fast_single_edge.FLAG_OBSERVED_LEFT
        else:
            row_flags |= fast_single_edge.FLAG_INFERRED_LEFT
        if has_right:
            row_flags |= fast_single_edge.FLAG_OBSERVED_RIGHT
        else:
            row_flags |= fast_single_edge.FLAG_INFERRED_RIGHT
        if source_flags & width_sources:
            row_flags |= (fast_single_edge.FLAG_WIDTH_CACHE |
                          fast_single_edge.FLAG_IDENTITY_ANCHORED)
        route_rows[target_offset + 4] = row_flags
        output_count += 1

        delta = abs(y - reference_y)
        if delta < reference_delta:
            reference_delta = delta
            reference_row_y = y
            reference_center2 = center2
            reference_edge2 = edge2
            reference_boundary = (
                "ENTRY_TRANSITION" if inferred_bridge else
                "LEFT" if observed_side == track_elements.SIDE_LEFT
                else "RIGHT")

    if transition_verified and (
            transition_inner_count < element_tracker.ring_arc_min_rows or
            transition_inner_longest < element_tracker.ring_arc_min_contiguous_rows or
            transition_inner_last - transition_inner_first < element_tracker.ring_arc_min_span_rows or
            transition_outer_count < element_tracker.min_side_support or
            transition_outer_longest < element_tracker.min_side_contiguous):
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        route_result["reason"] = "RING_ENTRY_TRANSITION_ANCHORS"
        return 0
    if output_count < element_tracker.min_target_rows or \
            reference_delta > element_tracker.max_cache_gap + 2:
        route_result["reason"] = ("RING_WIDTH_PROVENANCE_REJECTED"
                                  if width_source_rejected else
                                  "RING_TARGET_ROWS_OR_REFERENCE")
        route_result["valid"] = False
        route_result["selected_row_count"] = 0
        return 0

    center_x = reference_center2 * 0.5
    quality = 40 + min(45 if transition_verified else 60, output_count * 2)
    route_result["valid"] = True
    route_result["quality"] = quality
    route_result["reason"] = ("RING_ENTRY_TRANSITION_TARGET" if transition_verified
                              else "RING_OUTER_EDGE_CONTINUATION_TARGET" if outer_verified
                              else "RING_WIDTH_CACHE_TARGET")
    route_result["scene"] = "RING_ACTIVE"
    route_result["lateral_error"] = _normalize_fast_reference(
        center_x, settings.SURFACE_GRID_WIDTH)
    route_result["selected_row_ints"] = route_rows
    route_result["selected_row_count"] = output_count
    route_result["selected_row_stride"] = fast_single_edge.ROW_STRIDE
    route_result["selected_rows"] = None
    route_result["reference_center_x"] = center_x
    route_result["reference_y"] = float(reference_row_y)
    route_result["reference_provenance"] = (
        "RING_ENTRY_INTERPOLATED" if reference_boundary == "ENTRY_TRANSITION" else
        "RING_WIDTH_CACHE_LEFT" if reference_boundary == "LEFT" else
        "RING_WIDTH_CACHE_RIGHT")
    route_result["boundary_source"] = route_result[
        "reference_provenance"]
    route_result["inferred"] = True
    route_result["inference_flags"] = (
        fast_single_edge.INFERENCE_USED_WIDTH_CACHE |
        fast_single_edge.INFERENCE_USED_CURRENT_EDGE |
        fast_single_edge.INFERENCE_USED_CONTINUITY)
    route_result["observed_both_rows"] = (0 if transition_verified else max(0,
        int(element_result.get("observed_rows", 0)) - int(
            element_result.get("inferred_rows", 0))))
    route_result["inferred_rows"] = int(
        element_result.get("inferred_rows", 0))
    route_result["actual_boundary"] = ("NONE" if reference_boundary ==
                                       "ENTRY_TRANSITION" else reference_boundary)
    route_result["actual_boundary_x"] = (None if reference_boundary ==
                                         "ENTRY_TRANSITION" else reference_edge2 * 0.5)
    route_result["entry_transition"] = transition_verified
    route_result["weak_support"] = False
    return output_count


@micropython.native
def _service_reverse_control(uart, parser, input_buffer, drain_state,
                             now_ms, ticks_diff):
    """Poll and read one bounded reverse-control chunk without allocations.

    On a heartbeat timeout or receive overflow, queued bytes are discarded in
    chunks no larger than ``MAX_FEED_BYTES``. This prevents old controls or
    profile states from creating a fresh lease after a camera/controller stall.
    """
    if parser.poll(now_ms, ticks_diff):
        drain_state[0] = 1
    available_fn = getattr(uart, "any", None)
    readinto_fn = getattr(uart, "readinto", None)
    if not callable(available_fn) or not callable(readinto_fn):
        return parser.single_enabled

    try:
        available = available_fn()
        if (isinstance(available, bool) or not isinstance(available, int) or
                available < 0):
            raise OSError("invalid UART.any result")
        if available > reverse_control.MAX_FEED_BYTES:
            parser.invalidate("RX_OVERFLOW")
            drain_state[0] = 1

        if drain_state[0]:
            if available == 0:
                drain_state[0] = 0
                return parser.single_enabled
            requested = min(available, reverse_control.MAX_FEED_BYTES)
            received = readinto_fn(input_buffer, requested)
            if received is None:
                received = 0
            if (isinstance(received, bool) or not isinstance(received, int) or
                    received < 0 or received > requested):
                raise OSError("invalid UART.readinto result")
            return parser.single_enabled

        if available == 0:
            return parser.single_enabled
        requested = min(available, reverse_control.MAX_FEED_BYTES)
        received = readinto_fn(input_buffer, requested)
        if received is None:
            received = 0
        if (isinstance(received, bool) or not isinstance(received, int) or
                received < 0 or received > requested):
            raise OSError("invalid UART.readinto result")
        parser.feed(input_buffer, received, now_ms)
    except Exception:
        parser.invalidate("RX_ERROR")
        drain_state[0] = 1
        return False
    return parser.single_enabled


def _single_edge_fallback_allowed(path_result):
    """Keep explicit ambiguous routes out of the cached single-edge path."""
    if not isinstance(path_result, dict):
        return False
    reason = path_result.get("reason", "")
    scene = path_result.get("scene", "")
    if scene in ("AMBIGUOUS_CANDIDATE", "AMBIGUOUS_TOPOLOGY"):
        return False
    if reason in ("AMBIGUOUS_ROAD", "ROW_CANDIDATE_OVERFLOW",
                  "CORRUPT_PATH_STATE", "TOPOLOGY_BRANCH_UNRESOLVED"):
        return False
    topology = path_result.get("topology_evidence") or {}
    if topology.get("branch_evidence") or topology.get("branch"):
        return False
    return True


def _format_single_edge_diagnostic(route_mode, reason, result):
    """Format the selected route, measured edge, clip end and midpoint."""
    if not isinstance(result, dict):
        return "route=%s fallback=%s" % (route_mode, reason)
    boundary = result.get("actual_boundary", "NONE")
    boundary_x = result.get("actual_boundary_x")
    endpoint = result.get("clipped_endpoint", "NONE")
    endpoint_x = result.get("clipped_endpoint_x")
    midpoint_x = result.get("reference_center_x")
    boundary_text = ("%s@%.1f" % (boundary, boundary_x)
                     if boundary_x is not None else str(boundary))
    endpoint_text = ("%s@%.1f" % (endpoint, endpoint_x)
                     if endpoint_x is not None else str(endpoint))
    midpoint_text = ("%.1f" % midpoint_x
                     if midpoint_x is not None else "NONE")
    return ("route=%s fallback=%s actual=%s clip=%s midpoint=%s" % (
        route_mode, reason, boundary_text, endpoint_text, midpoint_text))


def _selected_actual_boundaries(selection, road_result, reference, valid):
    """Return current measured left/right transition x values, when present."""
    left_x = None
    right_x = None
    if not valid or reference is None or reference.get("y") is None:
        return left_x, right_x

    source = selection.get("source")
    if source == "road":
        rows = road_result.get("selected_rows")
        reference_y = float(reference["y"])
        packed = road_result.get("selected_row_ints")
        count = road_result.get("selected_row_count", 0)
        stride = road_result.get("selected_row_stride")
        packed_valid = (
            isinstance(packed, array) and stride == 4 and
            isinstance(count, int) and not isinstance(count, bool) and
            0 <= count <= 60 and len(packed) >= count * 4)
        packed_view = (
            packed_valid and rows is not None and
            getattr(rows, "packed", None) is packed and
            getattr(rows, "active_count", None) == count)
        if packed_valid and (rows is None or packed_view):
            # The current fast-path view exposes each boundary as left-1 and
            # right+1. Read those exact values from its packed backing store,
            # without indexing the view and materializing row dictionaries.
            # A packed-only legacy result keeps its historical half-cell edge.
            edge_offset = -1.0 if rows is not None else -0.5
            opposite_edge_offset = 1.0 if rows is not None else 0.5
            for index in range(count):
                offset = index * 4
                if abs(float(packed[offset]) - reference_y) > 0.01:
                    continue
                left_x = float(packed[offset + 1]) + edge_offset
                right_x = float(packed[offset + 2]) + opposite_edge_offset
                break
            return left_x, right_x

        if rows is not None:
            for index in range(len(rows)):
                row = rows[index]
                if abs(float(row.get("y", -1000000)) - reference_y) > 0.01:
                    continue
                boundary = row.get("boundary")
                if isinstance(boundary, dict):
                    left = boundary.get("left")
                    right = boundary.get("right")
                    if left is not None and len(left) >= 1:
                        left_x = float(left[0])
                    if right is not None and len(right) >= 1:
                        right_x = float(right[0])
                break
    elif source in ("single_left", "single_right",
                    "ring_left", "ring_right"):
        result = selection.get("result") or {}
        boundary = result.get("actual_boundary")
        boundary_x = result.get("actual_boundary_x")
        if boundary_x is not None:
            if boundary == "LEFT":
                left_x = float(boundary_x)
            elif boundary == "RIGHT":
                right_x = float(boundary_x)
    return left_x, right_x


def _format_selected_actual_diagnostic(selection, road_result, reference,
                                       valid):
    """Describe only current selected-frame boundaries and controller point."""
    left_x, right_x = _selected_actual_boundaries(
        selection, road_result, reference, valid)
    left_text = "NONE" if left_x is None else "%.1f" % left_x
    right_text = "NONE" if right_x is None else "%.1f" % right_x
    if not valid or reference is None:
        reference_text = "NONE"
    elif reference.get("y") is None:
        reference_text = "%.1f" % reference["x"]
    else:
        reference_text = "%.1f,%.1f" % (
            reference["x"], reference["y"])
    return left_text, right_text, reference_text


def _same_fast_track_reference(previous_source, source,
                               previous_reference_x, reference_x,
                               previous_reference_y, reference_y):
    """Allow filter history across a verified road/single mode transition.

    Single-edge output is already anchored to bilateral cache and current
    continuity. Retain EMA history only when both modes refer to the same
    steering path, same controller row, and nearby center estimate.
    """
    if previous_source not in ("road", "single_left", "single_right") or \
            source not in ("road", "single_left", "single_right"):
        return False
    if (previous_reference_x is None or reference_x is None or
            previous_reference_y is None or reference_y is None):
        return False
    if abs(float(reference_y) - float(previous_reference_y)) > 0.01:
        return False
    maximum_shift = float(getattr(
        config, "SINGLE_EDGE_FILTER_REFERENCE_SHIFT_CELLS", 8.0))
    if (maximum_shift != maximum_shift or maximum_shift < 0.0 or
            maximum_shift > config.SURFACE_GRID_WIDTH):
        return False
    return abs(float(reference_x) - float(previous_reference_x)) <= \
        maximum_shift


def _build_gray_overlay_text_cache(selection, surface_result, road_result,
                                   reference, valid, raw_ctrl,
                                   filtered_error, processing_ms, fps,
                                   weight_info=None):
    """Format compact control and preview details, never frame geometry."""
    source = selection.get("source", "NONE") if valid else "NONE"
    track_profile = str(selection.get("track_profile_label", "UNK"))
    ring_state = str(selection.get("track_ring_state", "IDLE"))
    ring_reason = str(selection.get(
        "track_ring_reason", selection.get("reason", "NOT_RUN")))
    details0 = "TRK %s %s" % (track_profile, ring_state)
    details1 = "WHY %s" % ring_reason
    details2 = ""
    details3 = ""
    if valid and weight_info is not None and weight_info.get("valid"):
        position_error = weight_info.get("position_error", 0)
        heading_error = weight_info.get("heading_error", 0)
        contribution = weight_info.get("heading_contribution", 0)
        near_percent = int(weight_info.get("near_weight", 0.0) * 100 + 0.5)
        far_percent = int(weight_info.get("far_weight", 0.0) * 100 + 0.5)
        if source in ("ring_left", "ring_right"):
            details2 = "POS%+d RH%+d" % (
                position_error, heading_error)
            details3 = "CMD%+d P%+d W%d/%d" % (
                weight_info.get("pd_error", raw_ctrl),
                weight_info.get("filtered_position", filtered_error),
                far_percent, near_percent)
        else:
            details2 = "P%+d D%+d CAN%+d" % (
                weight_info.get("filtered_position", position_error),
                weight_info.get("derivative_d", 0),
                weight_info.get("preview_candidate", contribution))
            details3 = "CMD%+d EMA%+d" % (
                weight_info.get("pd_error", raw_ctrl),
                weight_info.get("filtered_position", filtered_error))
            if weight_info.get("preview_valid"):
                preview_mode = weight_info.get("profile_short", "PD")
                preview_state = weight_info.get("preview_state_name", "STR")
                details3 += " %s %s A%+d" % (
                    preview_mode, preview_state,
                    weight_info.get("preview_applied", 0))
            else:
                preview_mode = weight_info.get("profile_short", "PD")
                preview_state = weight_info.get("preview_state_name", "STR")
                details3 += " %s %s W%d/%d" % (
                    preview_mode, preview_state, far_percent, near_percent)
    elif valid:
        details2 = "POS UNAVAILABLE"
        details3 = "CMD%+d EMA%+d" % (raw_ctrl, filtered_error)
    else:
        details2 = "CMD0 EMA0"
        single_edge_reason = selection.get("single_edge_reason")
        if isinstance(single_edge_reason, str):
            details3 = single_edge_reason[:24]
        else:
            details3 = "%s NONE" % str(config.STEERING_CONTROL_PROFILE).upper()
    return (details0[:24], details1[:24], details2[:24], details3[:24])


@micropython.native
def _v7_overlay_status(overlay_state, selection, valid):
    """Reuse the live profile and element state until those fields change."""
    route_mode = getattr(config, "SINGLE_EDGE_ROUTE_MODE", "unknown")
    source = selection.get("source", "NONE") if valid else "NONE"
    track_profile = selection.get("track_profile_label", "UNK")
    ring_state = selection.get("track_ring_state", "IDLE")
    ring_reason = selection.get("track_ring_reason", "NOT_RUN")
    if (overlay_state.status_text is None or
            overlay_state.status_valid != bool(valid) or
            overlay_state.status_source != source or
            overlay_state.status_route_mode != route_mode or
            overlay_state.status_track_profile != track_profile or
            overlay_state.status_ring_state != ring_state or
            overlay_state.status_ring_reason != ring_reason):
        reason_code = _TRACK_STATUS_REASON_CODES.get(
            ring_reason, str(ring_reason)[:7])
        overlay_state.status_text = "TRK %s %s:%s" % (
            track_profile, str(ring_state)[:7], reason_code[:7])
        overlay_state.status_valid = bool(valid)
        overlay_state.status_source = source
        overlay_state.status_route_mode = route_mode
        overlay_state.status_track_profile = track_profile
        overlay_state.status_ring_state = ring_state
        overlay_state.status_ring_reason = ring_reason
    status_image = overlay_state.status_image
    status_text = overlay_state.status_text
    status_mask = overlay_state.status_mask
    if (status_image is not None and
            len(status_text) * 8 <= _OVERLAY_STATUS_IMAGE_WIDTH):
        if overlay_state.status_bitmap_text != status_text:
            status_image.clear()
            status_image.draw_string(
                (0, 0), status_text, color=255, scale=1)
            if status_mask is not None:
                status_mask.clear()
                status_mask.draw_string(
                    (0, 0), status_text, color=255, scale=1)
            overlay_state.status_bitmap_text = status_text
        overlay_state.status_bitmap_ready = True
    else:
        overlay_state.status_bitmap_ready = False
    return overlay_state.status_text


@micropython.native
def _road_overlay_packed_array_valid(packed):
    """Validate the reusable FastPathDetector int32 producer buffer once."""
    if not isinstance(packed, array):
        return False
    cache = _ROAD_OVERLAY_PACKED_CACHE
    if packed is cache[0] and len(packed) == cache[1]:
        return cache[2]
    cache[0] = packed
    cache[1] = -1
    cache[2] = False
    try:
        packed_length = len(packed)
        view = memoryview(packed)
        itemsize = getattr(view, "itemsize", None)
    except (TypeError, AttributeError):
        return False
    cache[1] = packed_length
    if itemsize is None:
        # OpenMV v5 omits view width/format. Check physical width once per
        # reusable producer array, then retain the result by object identity.
        try:
            raw_bytes = bytes(view)
            valid_size = len(raw_bytes) == packed_length * 4
            del raw_bytes
        except (TypeError, ValueError, MemoryError):
            return False
        if not valid_size:
            return False
    elif itemsize != 4:
        return False
    view_format = getattr(view, "format", None)
    if (view_format is not None and
            view_format not in ("i", "l", "@i", "@l", "=i", "=l")):
        return False
    typecode = getattr(packed, "typecode", None)
    if typecode is not None and typecode != "i":
        return False
    try:
        if packed_length == 0 or not isinstance(packed[0], int):
            return False
    except (IndexError, TypeError):
        return False
    # When OpenMV omits typecode and memoryview metadata, the exact _RowView
    # producer identity below guarantees FastPathDetector's array("i") source;
    # the physical-width and integer-sample checks above reject H/f buffers.
    cache[2] = True
    return True


@micropython.native
def _road_overlay_rows_unexposed(exposed, count):
    """Allow packed drawing only when aliases used by this overlay are fresh."""
    if (type(exposed) is not bytearray or count < 0 or
            count > len(exposed)):
        return False
    if count >= 2:
        if _PREVIEW_MAX_BOUNDARY_SEGMENTS <= 1:
            if exposed[0] or exposed[count - 1]:
                return False
        else:
            step = (count + 2) // 3
            previous_index = 0
            index = step
            while index < count:
                if exposed[previous_index] or exposed[index]:
                    return False
                previous_index = index
                index += step
            if previous_index != count - 1:
                if exposed[previous_index] or exposed[count - 1]:
                    return False
    sample_count = _preview_row_sample_count(count)
    for sample_index in range(sample_count):
        index = _preview_row_sample_index(
            count, sample_index, sample_count)
        if exposed[index]:
            return False
    return True


@micropython.native
def _draw_road_overlay_packed(img, packed, count):
    """Draw the same sparse road edges and sampled centerline from int rows."""
    if count >= 2:
        if _PREVIEW_MAX_BOUNDARY_SEGMENTS <= 1:
            previous_index = 0
            index = count - 1
            before_offset = previous_index * 4
            current_offset = index * 4
            left0 = _surface_pixel((packed[before_offset + 1] - 1,
                                    packed[before_offset]))
            left1 = _surface_pixel((packed[current_offset + 1] - 1,
                                    packed[current_offset]))
            right0 = _surface_pixel((packed[before_offset + 2] + 1,
                                     packed[before_offset]))
            right1 = _surface_pixel((packed[current_offset + 2] + 1,
                                     packed[current_offset]))
            img.draw_line((left0[0], left0[1], left1[0], left1[1]),
                          color=96)
            img.draw_line((right0[0], right0[1], right1[0], right1[1]),
                          color=96)
        else:
            step = (count + 2) // 3
            previous_index = 0
            index = step
            while index < count:
                before_offset = previous_index * 4
                current_offset = index * 4
                left0 = _surface_pixel((packed[before_offset + 1] - 1,
                                        packed[before_offset]))
                left1 = _surface_pixel((packed[current_offset + 1] - 1,
                                        packed[current_offset]))
                right0 = _surface_pixel((packed[before_offset + 2] + 1,
                                         packed[before_offset]))
                right1 = _surface_pixel((packed[current_offset + 2] + 1,
                                         packed[current_offset]))
                img.draw_line((left0[0], left0[1], left1[0], left1[1]),
                              color=96)
                img.draw_line((right0[0], right0[1], right1[0], right1[1]),
                              color=96)
                previous_index = index
                index += step
            if previous_index != count - 1:
                before_offset = previous_index * 4
                current_offset = (count - 1) * 4
                left0 = _surface_pixel((packed[before_offset + 1] - 1,
                                        packed[before_offset]))
                left1 = _surface_pixel((packed[current_offset + 1] - 1,
                                        packed[current_offset]))
                right0 = _surface_pixel((packed[before_offset + 2] + 1,
                                         packed[before_offset]))
                right1 = _surface_pixel((packed[current_offset + 2] + 1,
                                         packed[current_offset]))
                img.draw_line((left0[0], left0[1], left1[0], left1[1]),
                              color=96)
                img.draw_line((right0[0], right0[1], right1[0], right1[1]),
                              color=96)

    previous_center = None
    sample_count = _preview_row_sample_count(count)
    for sample_index in range(sample_count):
        index = _preview_row_sample_index(
            count, sample_index, sample_count)
        offset = index * 4
        center_x = (packed[offset + 1] + packed[offset + 2]) * 0.5
        center_pixel = _surface_pixel((center_x, packed[offset]))
        if previous_center is not None:
            img.draw_line((previous_center[0], previous_center[1],
                           center_pixel[0], center_pixel[1]), color=255)
        previous_center = center_pixel


_ROAD_OVERLAY_PACKED_CACHE = [None, -1, False]


@micropython.native
def _preview_marker_crc8(gray_value):
    """CRC-8/ATM over the three little-endian bytes of a Gray frame ID."""
    crc = 0
    for byte_index in range(3):
        crc ^= (gray_value >> (byte_index * 8)) & 0xFF
        for _bit_index in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@micropython.viper
def _draw_preview_marker_pixels_viper(buffer_object, width: int,
                                      gray_value: int,
                                      checksum: int) -> int:
    """Write the fixed 32-cell marker into a validated grayscale buffer."""
    pixels = ptr8(buffer_object)
    cell = 0
    while cell < 32:
        bit = 0
        if cell < 24:
            bit = (gray_value >> cell) & 1
        else:
            bit = (checksum >> (cell - 24)) & 1
        color = 0
        if bit:
            color = 255
        x = 4 + (cell * 5)
        row = 0
        while row < 4:
            offset = ((4 + row) * width) + x
            pixels[offset] = color
            pixels[offset + 1] = color
            pixels[offset + 2] = color
            pixels[offset + 3] = color
            row += 1
        cell += 1
    return 1


def _preview_marker_image_buffer(img):
    """Return only a full sensor-sized GRAYSCALE image's writable byte view."""
    format_reader = getattr(img, "format", None)
    width_reader = getattr(img, "width", None)
    height_reader = getattr(img, "height", None)
    buffer_reader = getattr(img, "bytearray", None)
    if (not callable(format_reader) or not callable(width_reader) or
            not callable(height_reader) or not callable(buffer_reader)):
        raise ValueError("preview marker requires an OpenMV Image")
    if format_reader() != sensor.GRAYSCALE:
        raise ValueError("preview marker requires GRAYSCALE format")
    width = width_reader()
    height = height_reader()
    if width != 320 or height != 120:
        raise ValueError("preview marker requires the full 320x120 frame")
    pixels = buffer_reader()
    if not isinstance(pixels, bytearray) or len(pixels) != width * height:
        raise ValueError("preview marker requires a full writable image buffer")
    return pixels, width


def _draw_preview_frame_marker(img, sequence):
    """Draw the opt-in 24-bit Gray ID plus CRC-8 preview watermark."""
    if (not getattr(config, "PREVIEW_FPS_MARKER_ENABLED", False) or
            img is None or sequence is None):
        return False
    frame_id = int(sequence) & 0xFFFFFF
    gray_value = (frame_id ^ (frame_id >> 1)) & 0xFFFFFF
    checksum = _preview_marker_crc8(gray_value)
    pixels, width = _preview_marker_image_buffer(img)
    return bool(_draw_preview_marker_pixels_viper(
        pixels, width, gray_value, checksum))


@micropython.native
def _draw_gray_result(img, selection, surface_result, road_result,
                      reference, valid, raw_ctrl, filtered_error,
                      processing_ms, fps, text_cache=None,
                      weight_info=None, contour_trace_info=None,
                      contour_used=False, detail_image=None,
                      detail_mask=None, current_status=None,
                      status_image=None, status_mask=None):
    """Draw only current-frame geometry and cached text on the current image."""
    if not config.OVERLAY_ENABLED:
        return
    source = selection.get("source", "NONE") if valid else "NONE"
    if valid:
        if source == "surface":
            _draw_sparse_surface_contour(
                img, surface_result.get("sections", ()))
        elif source == "road":
            if contour_used:
                _draw_contour_route_overlay(
                    img, road_result, contour_trace_info)
            else:
                selected_rows = road_result.get("selected_rows") or ()
                packed = road_result.get("selected_row_ints")
                packed_count = road_result.get("selected_row_count", -1)
                packed_stride = road_result.get("selected_row_stride", 0)
                row_count = len(selected_rows)
                packed_valid = (
                    isinstance(packed, array) and
                    packed_stride == 4 and
                    isinstance(packed_count, int) and
                    not isinstance(packed_count, bool) and
                    0 <= packed_count <= 60 and
                    len(packed) >= packed_count * 4)
                if packed_valid:
                    packed_valid = _road_overlay_packed_array_valid(packed)
                row_view_type = getattr(fast_path, "_RowView", None)
                packed_view = (
                    packed_valid and row_view_type is not None and
                    type(selected_rows) is row_view_type and
                    getattr(selected_rows, "packed", None) is packed and
                    getattr(selected_rows, "active_count", None) ==
                    packed_count and row_count == packed_count)
                exposed = getattr(selected_rows, "exposed", None)
                if (packed_view and isinstance(exposed, bytearray) and
                        len(exposed) >= packed_count and
                        _road_overlay_rows_unexposed(
                            exposed, packed_count)):
                    _draw_road_overlay_packed(
                        img, packed, packed_count)
                else:
                    selected_rows = road_result.get("selected_rows") or ()
                    _draw_sparse_road_contour(img, selected_rows)
                    previous_center = None
                    row_count = len(selected_rows)
                    sample_count = _preview_row_sample_count(row_count)
                    for sample_index in range(sample_count):
                        index = _preview_row_sample_index(
                            row_count, sample_index, sample_count)
                        row = selected_rows[index]
                        center_x = row.get("center")
                        if center_x is None:
                            center_x = (float(row["left"]) +
                                        float(row["right"])) * 0.5
                        center_pixel = _surface_pixel((center_x, row["y"]))
                        if previous_center is not None:
                            img.draw_line((previous_center[0], previous_center[1],
                                           center_pixel[0], center_pixel[1]),
                                          color=255)
                        previous_center = center_pixel
        elif source in ("ring_left", "ring_right"):
            candidate = selection.get("ring_candidate") or {}
            _draw_sparse_road_contour(
                img, road_result.get("selected_rows") or ())
            ring_result = selection.get("result") or {}
            ring_rows = ring_result.get("selected_row_ints")
            ring_count = ring_result.get("selected_row_count", 0)
            previous_ring_center = None
            if (isinstance(ring_rows, array) and
                    ring_result.get("selected_row_stride") == 5 and
                    isinstance(ring_count, int) and
                    not isinstance(ring_count, bool) and
                    0 < ring_count <= config.SURFACE_GRID_HEIGHT and
                    len(ring_rows) >= ring_count * 5):
                sample_count = _preview_row_sample_count(ring_count)
                for sample_index in range(sample_count):
                    index = _preview_row_sample_index(
                        ring_count, sample_index, sample_count)
                    offset = index * 5
                    row_y = ring_rows[offset]
                    center_x = ring_rows[offset + 1] * 0.5
                    boundary_x = ring_rows[offset + 2] * 0.5
                    img.draw_cross(_surface_pixel((boundary_x, row_y)),
                                   color=192)
                    center_pixel = _surface_pixel((center_x, row_y))
                    if previous_ring_center is not None:
                        img.draw_line((previous_ring_center[0],
                                       previous_ring_center[1],
                                       center_pixel[0], center_pixel[1]),
                                      color=255)
                    previous_ring_center = center_pixel
            path_x = candidate.get("path_x_rows")
            path_valid = candidate.get("path_valid_rows")
            if (isinstance(path_x, array) and
                    isinstance(path_valid, bytearray)):
                previous_path = None
                path_points = 0
                for row_y in range(config.SURFACE_ROI_FLOOR_Y,
                                   config.SURFACE_GRID_HEIGHT):
                    if path_valid[row_y] != 1:
                        continue
                    current_path = _surface_pixel((path_x[row_y], row_y))
                    if previous_path is not None:
                        img.draw_line((previous_path[0], previous_path[1],
                                       current_path[0], current_path[1]),
                                      color=96)
                    previous_path = current_path
                    path_points += 1
                if path_points > 0 and candidate.get("port_x") is not None:
                    img.draw_cross(_surface_pixel((
                        candidate["port_x"], candidate["port_y"])),
                        color=128)
                    # This marker is only the image crop endpoint; it is not
                    # a measured road boundary or a steering target.
                    img.draw_cross(_surface_pixel((
                        candidate["clip_endpoint_x"], candidate["port_y"])),
                        color=64)
        elif source in ("single_left", "single_right"):
            single_result = selection.get("result") or {}
            rows = single_result.get("selected_row_ints")
            count = single_result.get("selected_row_count", 0)
            stride = single_result.get("selected_row_stride", 0)
            if rows is not None and stride == 5:
                clipped = single_result.get("route_mode") == \
                    "clipped_midpoint"
                endpoint_x = (single_result.get("clipped_endpoint_x")
                              if clipped else None)
                previous_midpoint = None
                sample_count = _preview_row_sample_count(count)
                for sample_index in range(sample_count):
                    index = _preview_row_sample_index(
                        count, sample_index, sample_count)
                    offset = index * stride
                    row_y = rows[offset]
                    midpoint_x = rows[offset + 1] * 0.5
                    observed_edge_x = rows[offset + 2] * 0.5
                    img.draw_cross(_surface_pixel(
                        (observed_edge_x, row_y)), color=192)
                    midpoint_pixel = _surface_pixel((midpoint_x, row_y))
                    if previous_midpoint is not None:
                        img.draw_line((previous_midpoint[0],
                                       previous_midpoint[1],
                                       midpoint_pixel[0],
                                       midpoint_pixel[1]), color=255)
                    previous_midpoint = midpoint_pixel
                    if clipped and endpoint_x is not None:
                        img.draw_cross(_surface_pixel(
                            (endpoint_x, row_y)), color=96)
        if reference is not None and reference.get("y") is not None:
            img.draw_cross(_surface_pixel((reference["x"], reference["y"])),
                           color=160)
        if (weight_info is not None and weight_info.get("valid") and
                weight_info.get("center_x") is not None):
            # The weighted position center is a virtual column, not a
            # measured point on a selected camera row.
            raw_top = _surface_pixel((weight_info["center_x"],
                                      config.SURFACE_ROI_FLOOR_Y))
            bottom_x, bottom_y = _surface_pixel((
                weight_info["center_x"], config.SURFACE_GRID_HEIGHT - 1))
            img.draw_line((raw_top[0], raw_top[1], bottom_x, bottom_y),
                          color=192)
            if weight_info.get("preview_valid"):
                near_point = _surface_pixel((
                    weight_info["preview_near_x"],
                    weight_info["preview_near_y"]))
                far_point = _surface_pixel((
                    weight_info["preview_far_x"],
                    weight_info["preview_far_y"]))
                img.draw_line((far_point[0], far_point[1],
                               near_point[0], near_point[1]), color=64)
                img.draw_cross(near_point, color=128)
                img.draw_cross(far_point, color=128)
            control_x = (((raw_ctrl / 1000.0) *
                          ((config.SURFACE_GRID_WIDTH - 1) * 0.5)) +
                         (config.SURFACE_GRID_WIDTH - 1) * 0.5)
            control_start = _surface_pixel((
                control_x, config.SURFACE_ROI_FLOOR_Y))
            control_end = _surface_pixel((
                control_x, config.SURFACE_GRID_HEIGHT - 1))
            img.draw_line((control_start[0], control_start[1],
                           control_end[0], control_end[1]), color=224)
            filtered_x = (((filtered_error / 1000.0) *
                           ((config.SURFACE_GRID_WIDTH - 1) * 0.5)) +
                          (config.SURFACE_GRID_WIDTH - 1) * 0.5)
            filtered_start = _surface_pixel((
                filtered_x, config.SURFACE_ROI_FLOOR_Y))
            filtered_end = _surface_pixel((
                filtered_x, config.SURFACE_GRID_HEIGHT - 1))
            img.draw_line((filtered_start[0], filtered_start[1],
                           filtered_end[0], filtered_end[1]), color=255)

    if text_cache is None:
        text_cache = _build_gray_overlay_text_cache(
            selection, surface_result, road_result, reference, valid,
            raw_ctrl, filtered_error, processing_ms, fps, weight_info)
    if current_status is None:
        route_mode = getattr(config, "SINGLE_EDGE_ROUTE_MODE", "unknown")
        route_label = ("CLIP" if route_mode == "clipped_midpoint" else
                       "CACHE" if route_mode == "cached_width" else
                       str(route_mode).upper())
        source_label = (selection.get("source", "NONE").upper()
                        if valid else "NONE")
        current_status = ("NOW V %s/%s" % (route_label, source_label)
                          if valid else "NOW INVALID %s" % route_label)
    if status_image is not None and current_status is not None:
        if status_mask is None:
            img.draw_image(status_image, 0, 0)
        else:
            img.draw_image(status_image, 0, 0, mask=status_mask)
    else:
        img.draw_string((0, 0), current_status, color=255, scale=1)
    if detail_image is not None and detail_mask is not None:
        # The cached bitmap contains text only. Its mask keeps the scene and
        # current-frame route drawing visible beneath glyph pixels.
        img.draw_image(detail_image, 0, 10, mask=detail_mask)
    elif text_cache is not None:
        for index in range(len(text_cache)):
            img.draw_string((0, 10 + index * 10), text_cache[index],
                            color=192, scale=1)


class _V7OverlayState:
    """Preallocated per-frame preview state and reusable text bitmaps."""

    def __init__(self, now_ms, detail_image=None, detail_mask=None,
                 status_image=None, status_mask=None):
        self.last_text_ms = None
        self.text_cache = None
        self.detail_image = detail_image
        self.detail_mask = detail_mask
        self.status_text = None
        self.status_valid = None
        self.status_source = None
        self.status_route_mode = None
        self.status_track_profile = None
        self.status_ring_state = None
        self.status_ring_reason = None
        self.status_image = status_image
        self.status_mask = status_mask
        self.status_bitmap_text = None
        self.status_bitmap_ready = False
        self.draw_us = 0
        self.flush_us = 0


# Cached detail lines are capped at 24 characters. OpenMV's scale-1 bitmap
# font uses 8-pixel character cells; keep 16 pixels of spare width while
# avoiding a full-preview-width masked blit on every frame.
_OVERLAY_DETAIL_IMAGE_WIDTH = min(config.IMAGE_WIDTH, 24 * 8 + 16)
# Profile/state/reason status is capped at 24 cells. Longer vendor reason
# strings remain complete in the low-rate V7 diagnostic line.
_OVERLAY_STATUS_IMAGE_WIDTH = min(
    config.IMAGE_WIDTH, 24 * 8)
_OVERLAY_STATUS_IMAGE_HEIGHT = 10
_OVERLAY_STATUS_USE_MASK = False
_TRACK_PROFILE_LABELS = ("UNK", "A", "B", "C")
_TRACK_STATUS_REASON_CODES = {
    "OBSERVING": "OBS",
    "SESSION_INACTIVE": "IDLE",
    "PROFILE_UNAVAILABLE": "NO_PROFILE",
    "FRAME_INVALID": "BAD_FRAME",
    "RING_DISABLED_PROFILE_A": "A_OFF",
    "TOPOLOGY_NOT_CLEAR": "AMBIG",
    "NO_FROZEN_WIDTH_PROFILE": "NO_WIDTH",
    "ONE_SIDE_LOST_CANDIDATE": "SIDE_LOSS",
    "CONFIRMING_ONE_SIDE_LOSS": "ENTRY",
    "WAITING_FOR_RING_ARC": "ARC",
    "RING_INSIDE_EXIT_UNOBSERVED": "INSIDE",
    "EXIT_EXPERIMENTAL_CONFIRM": "EXIT_CONF",
    "EXIT_EXPERIMENTAL_REOBSERVATION": "EXIT_REOBS",
    "WAITING_FOR_NORMAL_ROAD_REACQUIRE": "REACQUIRE",
    "NO_CROSS_EVIDENCE": "NO_CROSS",
    "SYMMETRIC_OPENING_STRAIGHT_PATH": "CROSS_PATH",
    "CROSS_CANDIDATE_DEBOUNCE": "CROSS_WAIT",
    "CROSS_STRAIGHT_CONFIRMED": "CROSS_OK",
}


def _noop_preview_flush():
    """Keep RAM-fixture runs independent of the camera preview interface."""
    return None


def _native_preview_control_api_available():
    """Require both patched module markers before passing preview=False."""
    try:
        import csi
    except Exception:
        return False
    return (getattr(sensor, "PREVIEW_CONTROL_API", None) == 1 and
            getattr(csi, "PREVIEW_CONTROL_API", None) == 1)


def _open_usb_vcp_safely():
    """Return a USB CDC handle when this firmware exposes one."""
    constructor = getattr(pyb, "USB_VCP", None)
    if not callable(constructor):
        return None
    try:
        return constructor()
    except Exception:
        return None


def _fixture_usb_vcp(frame_source):
    """Return a fixture's explicit USB handle and whether it supplied one."""
    not_provided = object()
    for name in ("usb_vcp", "_usb_vcp"):
        try:
            usb_vcp = getattr(frame_source, name, not_provided)
        except Exception:
            # A present but failing handle must fail closed like a disconnected
            # CDC device; it must not fall back to unconditional fixture draw.
            return None, True
        if usb_vcp is not not_provided:
            return usb_vcp, True
    return None, False


def _usb_vcp_is_connected(usb_vcp):
    """Treat missing or failing USB CDC status as disconnected."""
    if usb_vcp is None:
        return False
    try:
        connected = getattr(usb_vcp, "isconnected", None)
        return bool(connected()) if callable(connected) else False
    except Exception:
        return False


@micropython.native
def _preview_codec_timeout_valid(value):
    return (isinstance(value, int) and not isinstance(value, bool) and
            1 <= value <= 1000)


@micropython.native
def _preview_codec_guard_info():
    """Return (setter, status, current_ms) for the optional codec guard."""
    if cartest_accel is None:
        return None, "missing-extension", None
    try:
        marker_present = hasattr(cartest_accel, "PREVIEW_CODEC_GUARD_API")
        timeout_present = hasattr(cartest_accel, "jpeg_encode_timeout_ms")
        if not marker_present and not timeout_present:
            return None, "missing-optional-guard", None
        if not marker_present or not timeout_present:
            return None, "incomplete-guard-api", None
        api_marker = getattr(cartest_accel, "PREVIEW_CODEC_GUARD_API", None)
        timeout_api = getattr(cartest_accel, "jpeg_encode_timeout_ms", None)
        if (not isinstance(api_marker, int) or isinstance(api_marker, bool) or
                api_marker != 1 or not callable(timeout_api)):
            return None, "extension-without-guard-api", None
        current_ms = timeout_api()
        if not _preview_codec_timeout_valid(current_ms):
            return None, "invalid-timeout-value", None
        return timeout_api, "available", current_ms
    except Exception:
        return None, "guard-api-error", None


def _preview_codec_guard_diagnostic():
    _timeout_api, status, current_ms = _preview_codec_guard_info()
    if status == "available":
        return ("USB_PREVIEW_CODEC_GUARD capable=1 current_ms=%d "
                "encode_budget_ms=5" % current_ms)
    legacy = "1" if status in (
        "missing-extension", "missing-optional-guard") else "0"
    return ("USB_PREVIEW_CODEC_GUARD capable=0 reason=%s "
            "legacy_fallback=%s" % (status, legacy))


def _publish_current_usb_preview_jpeg(img, usb_vcp, quality):
    """Publish only the current 320x120 frame as a copied full-size JPEG."""
    started = time.ticks_us()
    jpeg = None
    published = False
    failed = False
    timeout_api = None
    old_timeout_ms = None
    restore_timeout = False
    pending_keyboard_interrupt = False
    try:
        if _usb_vcp_is_connected(usb_vcp):
            if (img is None or img.width() != config.IMAGE_WIDTH or
                    img.height() != config.IMAGE_HEIGHT):
                failed = True
            else:
                if cartest_accel is not None:
                    timeout_api, guard_status, old_timeout_ms = (
                        _preview_codec_guard_info())
                    if guard_status == "missing-optional-guard":
                        # A pre-guard cartest_accel module is supported. The
                        # JPEG guard is optional, so preserve its old preview.
                        timeout_api = None
                    elif guard_status != "available":
                        # An installed but incompatible extension must not
                        # silently take the unbounded legacy encode path.
                        failed = True
                    if guard_status == "available":
                        # Mark restoration necessary before calling the setter:
                        # even a malformed extension may mutate and then fail.
                        restore_timeout = True
                        previous_timeout_ms = timeout_api(5)
                        if (not _preview_codec_timeout_valid(previous_timeout_ms)
                                or previous_timeout_ms != old_timeout_ms):
                            failed = True
                if not failed:
                    jpeg = img.to_jpeg(x_scale=1.0, y_scale=1.0,
                                       quality=quality, copy=True)
                    if jpeg is None:
                        failed = True
                    elif _usb_vcp_is_connected(usb_vcp):
                        # A cable disconnect during encoding must not flush the
                        # old image.
                        jpeg.flush()
                        published = True
    except KeyboardInterrupt:
        pending_keyboard_interrupt = True
        raise
    except Exception:
        failed = True
    finally:
        if restore_timeout:
            try:
                previous_timeout_ms = timeout_api(old_timeout_ms)
                if (not _preview_codec_timeout_valid(previous_timeout_ms) or
                        previous_timeout_ms != 5):
                    if not pending_keyboard_interrupt:
                        failed = True
            except KeyboardInterrupt:
                if not pending_keyboard_interrupt:
                    raise
            except Exception:
                if not pending_keyboard_interrupt:
                    failed = True
        jpeg = None
        if failed:
            published = False
    return (time.ticks_diff(time.ticks_us(), started), published, failed)


class _NativeUSBPreviewSource:
    """Capture and publish through one per-frame USB preview state object."""
    USB_PREVIEW_SOURCE_API = 1

    def __init__(self, sensor_module, usb_vcp, enabled=True, quality=10):
        if (getattr(sensor_module, "PREVIEW_CONTROL_API", None) != 1 or
                not _native_preview_control_api_available()):
            raise RuntimeError("native preview source requires API markers")
        self._sensor = sensor_module
        self._usb_vcp = usb_vcp
        self.enabled = bool(enabled)
        self.quality = quality
        self.frame_preview_active = False
        self.latest_image = None
        self.last_state = "NATIVE_HEADLESS"
        self.publish_failures = 0

    def configure(self, enabled, quality):
        if not isinstance(enabled, bool):
            raise ValueError("USB preview enabled state must be bool")
        if (not isinstance(quality, int) or isinstance(quality, bool) or
                quality < 1 or quality > 100):
            raise ValueError("USB preview quality must be in 1..100")
        self.enabled = enabled
        self.quality = quality

    def snapshot(self):
        self.clear()
        img = self._sensor.snapshot(preview=False)
        self.latest_image = img
        self.frame_preview_active = bool(
            self.enabled and _usb_vcp_is_connected(self._usb_vcp))
        if not self.enabled:
            self.last_state = "NATIVE_DISABLED"
        elif self.frame_preview_active:
            self.last_state = "NATIVE_ACTIVE"
        else:
            self.last_state = "NATIVE_HEADLESS"
        return img

    def flush(self):
        img = self.latest_image
        try:
            if not self.enabled:
                self.last_state = "NATIVE_DISABLED"
                return
            # The connection decision is latched by snapshot(). A host that
            # connects mid-frame waits for the next capture so an unannotated
            # cached image can never be published on reconnect.
            if not self.frame_preview_active:
                self.last_state = "NATIVE_HEADLESS"
                return
            if not _usb_vcp_is_connected(self._usb_vcp):
                self.last_state = "NATIVE_HEADLESS"
                return
            _, published, failed = _publish_current_usb_preview_jpeg(
                img, self._usb_vcp, self.quality)
            if published:
                self.last_state = "NATIVE_ACTIVE"
            elif failed:
                self.publish_failures += 1
                self.last_state = "NATIVE_ERROR"
            else:
                self.last_state = "NATIVE_HEADLESS"
        finally:
            self.clear()

    def clear(self):
        self.latest_image = None
        self.frame_preview_active = False


def _is_native_usb_preview_source(frame_source):
    return (getattr(frame_source, "USB_PREVIEW_SOURCE_API", None) == 1 and
            _native_preview_control_api_available() and
            callable(getattr(frame_source, "snapshot", None)) and
            callable(getattr(frame_source, "flush", None)) and
            callable(getattr(frame_source, "configure", None)) and
            callable(getattr(frame_source, "clear", None)) and
            hasattr(frame_source, "publish_failures"))


def _build_usb_preview_compact_text(selection,
                                     surface_result, road_result, reference,
                                     valid, control_command, filtered_error,
                                     processing_ms, fps, weight_info=None):
    """Keep the measured compact detail overlay to one cached FPS string."""
    return ("FPS %.1f" % fps,)


@micropython.native
def _render_v7_preview_frame(img, selection, surface_result, road_result,
                             reference, valid, control_command, filtered_error,
                             processing_ms, fps, overlay_state, now_ms,
                             flush_func=None, weight_info=None,
                             contour_trace_info=None, contour_used=False,
                             draw_enabled=True, preview_sequence=None):
    """Draw every captured frame, refresh details at the configured interval, then flush it.

    This runs only after detection and the observation UART write. The flush
    time is kept separate from capture-to-send age and drawing time.
    """
    overlay_state.draw_us = 0
    overlay_state.flush_us = 0
    if img is None:
        return
    if config.OVERLAY_ENABLED and draw_enabled:
        draw_start_us = time.ticks_us()
        current_status = _v7_overlay_status(
            overlay_state, selection, valid)
        compact_usb = bool(getattr(overlay_state, "compact_usb", False))
        if (overlay_state.text_cache is None or
                overlay_state.last_text_ms is None or
                time.ticks_diff(now_ms, overlay_state.last_text_ms) >=
                config.OVERLAY_PERIOD_MS):
            if compact_usb:
                overlay_state.text_cache = _build_usb_preview_compact_text(
                    selection, surface_result, road_result, reference, valid,
                    control_command, filtered_error, processing_ms, fps,
                    weight_info)
            else:
                overlay_state.text_cache = _build_gray_overlay_text_cache(
                    selection, surface_result, road_result, reference, valid,
                    control_command, filtered_error, processing_ms, fps,
                    weight_info)
            if (overlay_state.detail_image is not None and
                    overlay_state.detail_mask is not None):
                overlay_state.detail_image.clear()
                overlay_state.detail_mask.clear()
                for index in range(len(overlay_state.text_cache)):
                    y = index * 10
                    overlay_state.detail_image.draw_string(
                        (0, y), overlay_state.text_cache[index],
                        color=192, scale=1)
                    overlay_state.detail_mask.draw_string(
                        (0, y), overlay_state.text_cache[index],
                        color=255, scale=1)
            overlay_state.last_text_ms = now_ms
        _draw_gray_result(
            img, selection, surface_result, road_result, reference, valid,
            control_command, filtered_error, processing_ms, fps,
            overlay_state.text_cache, weight_info, contour_trace_info,
            contour_used, overlay_state.detail_image,
            overlay_state.detail_mask, current_status,
            (overlay_state.status_image
             if overlay_state.status_bitmap_ready else None),
            (overlay_state.status_mask
             if overlay_state.status_bitmap_ready else None))
        overlay_state.draw_us = time.ticks_diff(
            time.ticks_us(), draw_start_us)
    if config.PREVIEW_FPS_MARKER_ENABLED and draw_enabled:
        marker_start_us = time.ticks_us()
        _draw_preview_frame_marker(img, preview_sequence)
        overlay_state.draw_us += time.ticks_diff(
            time.ticks_us(), marker_start_us)
    if flush_func is None:
        flush_func = sensor.flush
    flush_start_us = time.ticks_us()
    flush_func()
    overlay_state.flush_us = time.ticks_diff(
        time.ticks_us(), flush_start_us)


def _format_gray_vdbg(selection, surface_result, road_result, mask_info,
                      reference, valid, raw_error, filtered_error,
                      sequence, tx_error_count, filter_reset_reason,
                      processing_ms, frame_age_ms, fps):
    if reference is None:
        reference_text = "-"
    elif reference.get("y") is None:
        reference_text = "%s:%.1f" % (reference["kind"], reference["x"])
    else:
        reference_text = "%s:%.1f,%.1f" % (
            reference["kind"], reference["x"], reference["y"])
    comparison = selection.get("comparison") or {}
    difference = comparison.get("difference_cells")
    difference_text = "--" if difference is None else "%.1f" % difference
    surface_boundary = surface_result.get("boundary_evidence") or {}
    road_boundary = road_result.get("boundary_evidence") or {}
    def boundary_text(boundary):
        if not boundary:
            return "-"
        measured = boundary.get("left_step_fraction_median")
        measured_right = boundary.get("right_step_fraction_median")
        measured_text = ("-/-" if measured is None or measured_right is None
                         else "%.2f/%.2f" % (measured, measured_right))
        return "%d/%d:%s:rel=%s/min=%0.2f" % (
            boundary.get("bilateral_supported", 0),
            boundary.get("pairs_tested", 0),
            boundary.get("reason", "?"), measured_text,
            config.GRAY_BOUNDARY_MIN_STEP_FRACTION)

    surface_boundary_text = boundary_text(surface_boundary)
    road_boundary_text = boundary_text(road_boundary)
    return ("VDBG backend=compare mode=gray_unified valid=%d q=%d "
            "source=%s ref=%s raw=%+d filt=%+d why=%s "
            "surface=%d:%s road=%d:%s bnd=S%s,R%s agree=%s blocks=%s:%d "
            "t=%s..%s seq=%d txerr=%d reset=%s "
            "proc=%dms age=%dms fps=%.1f" % (
                1 if valid else 0,
                selection.get("quality", 0) if valid else 0,
                selection.get("source", "NONE") if valid else
                    selection.get("rejected_source", "NONE"), reference_text,
                raw_error if valid else 0,
                filtered_error if valid else 0,
                selection.get("reason", "INVALID"),
                1 if surface_result.get("valid") else 0,
                surface_result.get("reason", "?"),
                1 if road_result.get("valid") else 0,
                road_result.get("reason", "?"),
                surface_boundary_text, road_boundary_text, difference_text,
                mask_info.get("reason", "?"),
                mask_info.get("trusted_block_count", 0),
                mask_info.get("threshold_min"),
                mask_info.get("threshold_max"), sequence, tx_error_count,
                filter_reset_reason,
                processing_ms,
                frame_age_ms, fps))


def _invalid_surface(reason):
    return {"valid": False, "quality": 0, "reason": reason,
            "scene": "UNKNOWN", "seed": None, "target": None,
            "sections": [], "fresh_sections": 0, "path_length": 0.0}


def _invalid_road(reason):
    return {"valid": False, "quality": 0, "reason": reason,
            "scene": "UNKNOWN", "reference_y": None,
            "center_x": None, "reference_center_x": None,
            "selected_rows": [], "lateral_error": 0}


_V7_PERF_STAGES = (
    "capture", "masks", "path_topology", "contour", "boundary", "ring",
    "row_weight",
    "reference_filter",
    "pack_uart", "capture_to_send", "diag_trace", "ring_diag", "overlay",
    "preview_flush", "debug",
    "summary_log", "gc", "total_cycle",
)


class _V7PerfWindow:
    """Bounded 1 ms histograms for the 10 s acceptance windows."""

    def __init__(self):
        self.bins = {}
        self.sums = {}
        self.maxima = {}
        self.counts = {}
        for stage in _V7_PERF_STAGES:
            self.bins[stage] = array("I", [0] * 202)
            self.sums[stage] = 0
            self.maxima[stage] = 0
            self.counts[stage] = 0
        self.frames = 0
        self.valid_frames = 0
        self.valid_full_writes = 0
        self.late_invalid_frames = 0
        self.write_attempts = 0
        self.full_writes = 0
        self.write_errors = 0
        self.last_sequence = -1
        self.heap_samples = 0
        self.heap_free_min = None
        self.heap_alloc_max = None
        self.heap_free_last = None
        self.heap_alloc_last = None

    @micropython.native
    def clear(self):
        for stage in _V7_PERF_STAGES:
            bins = self.bins[stage]
            for index in range(len(bins)):
                bins[index] = 0
            self.sums[stage] = 0
            self.maxima[stage] = 0
            self.counts[stage] = 0
        self.frames = 0
        self.valid_frames = 0
        self.valid_full_writes = 0
        self.late_invalid_frames = 0
        self.write_attempts = 0
        self.full_writes = 0
        self.write_errors = 0
        self.last_sequence = -1
        self.heap_samples = 0
        self.heap_free_min = None
        self.heap_alloc_max = None
        self.heap_free_last = None
        self.heap_alloc_last = None

    def record_heap(self, free_bytes, allocated_bytes):
        """Retain once-per-second heap samples in the active 10 s window."""
        self.heap_samples += 1
        self.heap_free_last = free_bytes
        self.heap_alloc_last = allocated_bytes
        if (self.heap_free_min is None or
                free_bytes < self.heap_free_min):
            self.heap_free_min = free_bytes
        if (self.heap_alloc_max is None or
                allocated_bytes > self.heap_alloc_max):
            self.heap_alloc_max = allocated_bytes

    @micropython.native
    def record(self, stage, elapsed_us):
        value = int(elapsed_us)
        if value < 0:
            value = 0
        bucket = value // 1000
        if bucket > 201:
            bucket = 201
        self.bins[stage][bucket] += 1
        self.sums[stage] += value
        if value > self.maxima[stage]:
            self.maxima[stage] = value
        self.counts[stage] += 1

    @micropython.native
    def _stage_summary(self, stage):
        count = self.counts[stage]
        if not count:
            return "%s:n=0,avg=0,p95_upper=0,max=0" % stage
        target = (count * 95 + 99) // 100
        accumulated = 0
        p95 = 202000
        bins = self.bins[stage]
        for index in range(len(bins)):
            accumulated += bins[index]
            if accumulated >= target:
                # The final bucket is open-ended. Report the measured maximum
                # as its conservative p95 upper bound instead of disguising a
                # 300/600 ms sample as 202 ms.
                p95 = (self.maxima[stage] if index == 201 else
                       (index + 1) * 1000)
                break
        return "%s:n=%d,avg=%d,p95_upper=%d,max=%d" % (
            stage, count, self.sums[stage] // count, p95,
            self.maxima[stage])

    def summary_line(self, run_id, window_index, attempted):
        stages = "|".join(self._stage_summary(stage)
                          for stage in _V7_PERF_STAGES)
        return ("V7_PERF_WINDOW run=%d index=%d start_ms=%d end_ms=%d "
                "distinct_captures=%d write_attempts=%d full_uart_writes=%d "
                "valid_frames=%d valid_full_writes=%d late_invalid_frames=%d "
                "uart_write_errors=%d "
                "attempted_by_loop=%d last_sequence=%d "
                "heap_samples=%d heap_free_min=%s heap_alloc_max=%s "
                "heap_free_last=%s heap_alloc_last=%s "
                "p95_method=1ms_bin_upper_overflow_uses_max stage_stats=%s" % (
                    run_id, window_index,
                    window_index * config.PERF_WINDOW_MS,
                    (window_index + 1) * config.PERF_WINDOW_MS,
                    self.frames, self.write_attempts, self.full_writes,
                    self.valid_frames, self.valid_full_writes,
                    self.late_invalid_frames, self.write_errors, attempted,
                    self.last_sequence, self.heap_samples,
                    str(self.heap_free_min), str(self.heap_alloc_max),
                    str(self.heap_free_last), str(self.heap_alloc_last),
                    stages))

    def summary_lines(self, run_id, window_index, attempted):
        """Return short stdout lines so the SDK's bounded channel cannot drop them."""
        lines = [
            ("V7_PERF_WINDOW run=%d index=%d start_ms=%d end_ms=%d "
             "distinct_captures=%d write_attempts=%d full_uart_writes=%d "
             "valid_frames=%d valid_full_writes=%d late_invalid_frames=%d "
             "uart_write_errors=%d attempted_by_loop=%d last_sequence=%d "
             "heap_samples=%d heap_free_min=%s heap_alloc_max=%s "
             "heap_free_last=%s heap_alloc_last=%s "
             "p95_method=1ms_bin_upper_overflow_uses_max" % (
                 run_id, window_index,
                 window_index * config.PERF_WINDOW_MS,
                 (window_index + 1) * config.PERF_WINDOW_MS,
                 self.frames, self.write_attempts, self.full_writes,
                 self.valid_frames, self.valid_full_writes,
                 self.late_invalid_frames, self.write_errors, attempted,
                 self.last_sequence, self.heap_samples,
                 str(self.heap_free_min), str(self.heap_alloc_max),
                  str(self.heap_free_last), str(self.heap_alloc_last)))]
        for stage in _V7_PERF_STAGES:
            lines.append("V7_PERF_STAGE run=%d index=%d %s" % (
                run_id, window_index, self._stage_summary(stage)))
        return lines


class _V7SummaryQueue:
    """Fixed-capacity FIFO; pending summary rows are never overwritten."""

    CAPACITY = 256

    def __init__(self):
        self.capacity = self.CAPACITY
        self.lines = [None] * self.capacity
        self.head = 0
        self.tail = 0
        self.count = 0
        self.overflow_windows = 0

    @property
    def pending(self):
        return self.count

    def enqueue(self, lines):
        line_count = len(lines)
        if line_count > self.capacity - self.count:
            self.overflow_windows += 1
            return False
        for line in lines:
            self.lines[self.tail] = line
            self.tail = (self.tail + 1) % self.capacity
            self.count += 1
        return True

    def pop(self):
        if not self.count:
            return None
        line = self.lines[self.head]
        self.lines[self.head] = None
        self.head = (self.head + 1) % self.capacity
        self.count -= 1
        return line


def _print_v7_summary(perf, run_id, window_index, attempted, queue=None):
    """Queue a window, or print its rows after a finite run has stopped."""
    lines = perf.summary_lines(run_id, window_index, attempted)
    if queue is not None:
        return queue.enqueue(lines)
    for line in lines:
        print(line)
    return len(lines)


def _drain_v7_summary_queue(queue, sleep_ms=None):
    """Print all retained rows; optional pacing is only safe after capture."""
    count = 0
    while queue.pending:
        print(queue.pop())
        count += 1
        if sleep_ms is not None:
            sleep_ms(5)
    return count


def _advance_v7_perf_window(send_end_us, window_start_us, window_us):
    """Advance bounded performance windows using wrap-safe tick operations."""
    advanced = 0
    next_start_us = time.ticks_add(window_start_us, window_us)
    while time.ticks_diff(send_end_us, next_start_us) >= 0:
        window_start_us = next_start_us
        next_start_us = time.ticks_add(window_start_us, window_us)
        advanced += 1
    return window_start_us, advanced


def _main_compare():
    _validate_v6_gray_config()
    sensor_id = _configure_gray_sensor(sensor)
    if isinstance(sensor_id, int):
        sensor_id_text = "0x%04X" % (sensor_id & 0xFFFF)
    else:
        sensor_id_text = "unavailable" if sensor_id is None else str(sensor_id)
    print("BUILD=%s backend=compare mode=gray_unified sensor_id=%s "
          "camera=320x120 crop=QVGA(0,60,320,120) grid=80x60 step_fraction=%.2f prefer=%s "
          "budget=%dms" % (
              config.BUILD_ID, sensor_id_text,
              config.GRAY_BOUNDARY_MIN_STEP_FRACTION,
              config.GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID,
              config.SURFACE_FRAME_BUDGET_MS))

    uart = pyb.UART(config.UART_ID, config.UART_BAUDRATE,
                    bits=config.UART_BITS, parity=None,
                    stop=config.UART_STOP_BITS,
                    timeout=config.UART_TIMEOUT_MS,
                    timeout_char=config.UART_CHAR_TIMEOUT_MS)
    clock = time.clock()
    observation_filter = adaptive_detector.ObservationFilter(
        config.OBSERVATION_FILTER_ALPHA,
        config.OBSERVATION_FILTER_MAX_GAP_MS,
        config.SURFACE_MIN_CONFIDENCE)
    sequence = 0
    last_send_ms = time.ticks_ms()
    last_debug_ms = last_send_ms
    tx_error_count = 0
    last_tx_sequence = -1
    previous_source = None
    previous_reference_kind = None
    previous_reference_y = None

    while True:
        clock.tick()
        frame_start_ms = time.ticks_ms()
        img = sensor.snapshot()
        if img.width() != config.IMAGE_WIDTH or \
                img.height() != config.IMAGE_HEIGHT:
            raise RuntimeError("image size differs from openmv/config.py")

        processing_start_ms = time.ticks_ms()
        mask_info = {"reason": "NOT_RUN", "trusted_block_count": 0}
        surface_result = _invalid_surface("NOT_RUN")
        road_result = _invalid_road("NOT_RUN")
        detector_error = None
        try:
            road_mask, edge_mask, known_region, mask_info = \
                grayscale_adapter.build_masks(img, config)
            surface_result = surface_detector.detect_path(
                edge_mask, config, mask_info, valid_region=known_region)
            road_result = road_detector.detect_road(
                road_mask, config.ROAD_PARAMS,
                valid_region=known_region)
            boundary_evidence.gate_candidates(
                surface_result, road_result, mask_info.get("luma"),
                edge_mask, road_mask, known_region, config)
            selection = source_selector.select_candidate(
                surface_result, road_result, config)
            selection, _selected, _boundary_reference = \
                _validate_selected_boundary(
                    selection, surface_result, road_result,
                    mask_info.get("luma"), edge_mask, road_mask,
                    known_region)
        except Exception as exc:
            detector_error = exc
            surface_result = _invalid_surface("DETECTOR_EXCEPTION")
            road_result = _invalid_road("DETECTOR_EXCEPTION")
            selection = source_selector.select_candidate(
                surface_result, road_result, config)

        raw_valid = bool(selection.get("valid"))
        raw_error = selection.get("lateral_error", 0) if raw_valid else 0
        reference = _selected_reference(selection) if raw_valid else None
        rejected_source = selection.get("source", "NONE")
        rejection_reason = None
        if raw_valid and selection.get("quality", 0) < \
                config.SURFACE_MIN_CONFIDENCE:
            rejection_reason = "LOW_CONFIDENCE"
        elif raw_valid and reference is None:
            rejection_reason = "REFERENCE_UNAVAILABLE"
        elif raw_valid and (raw_error < -1000 or raw_error > 1000):
            rejection_reason = "ERROR_OUT_OF_RANGE"
        if rejection_reason is not None:
            raw_valid = False
            raw_error = 0
            selection = dict(selection)
            selection.update({"valid": False, "quality": 0,
                              "lateral_error": 0,
                              "reason": rejection_reason,
                              "rejected_source": rejected_source,
                              "source": "NONE", "result": None})
            reference = None

        filter_reset_reason = "none"
        if raw_valid:
            source_now = selection.get("source")
            kind_now = reference["kind"]
            reference_y_now = reference.get("y")
            if previous_source is not None and (
                    source_now != previous_source or
                    kind_now != previous_reference_kind):
                observation_filter.reset()
                filter_reset_reason = "SOURCE_OR_REFERENCE_KIND"
            elif (previous_reference_y is not None and
                  reference_y_now is not None and
                  abs(reference_y_now - previous_reference_y) >
                  config.GRAY_FILTER_REFERENCE_Y_RESET_CELLS):
                observation_filter.reset()
                filter_reset_reason = "REFERENCE_Y_SHIFT"
        else:
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None

        now_ms = time.ticks_ms()
        filtered_error = observation_filter.update(
            now_ms, raw_valid, selection.get("quality", 0) if raw_valid else 0,
            raw_error, time.ticks_diff)
        valid = raw_valid
        if not valid:
            raw_error = 0
            filtered_error = 0
        else:
            previous_source = selection.get("source")
            previous_reference_kind = reference["kind"]
            previous_reference_y = reference.get("y")
        processing_ms = time.ticks_diff(now_ms, processing_start_ms)
        fps = clock.fps()

        # Capture plus detection must finish within the existing receiver
        # margin. A late frame immediately publishes an explicit invalid.
        pre_uart_ms = time.ticks_ms()
        frame_age_ms = time.ticks_diff(pre_uart_ms, frame_start_ms)
        if frame_age_ms > config.SURFACE_FRAME_BUDGET_MS:
            valid = False
            raw_valid = False
            raw_error = 0
            filtered_error = 0
            observation_filter.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            selection = dict(selection)
            selection.update({"valid": False, "quality": 0,
                              "lateral_error": 0,
                              "reason": "FRAME_BUDGET",
                              "rejected_source": selection.get("source", "NONE"),
                              "source": "NONE", "result": None})
            reference = None
            if filter_reset_reason == "none":
                filter_reset_reason = "FRAME_BUDGET"
        quality = selection.get("quality", 0) if valid else 0

        send_due = (time.ticks_diff(pre_uart_ms, last_send_ms) >=
                    config.SEND_PERIOD_MS)
        if send_due or frame_age_ms > config.SURFACE_FRAME_BUDGET_MS:
            last_tx_sequence = sequence
            try:
                _send_observation(uart, sequence, valid, quality,
                                  filtered_error)
            except Exception as exc:
                tx_error_count += 1
                print("UART_WRITE_ERROR=%d %s" % (tx_error_count, exc))
            sequence = (sequence + 1) & 0xFFFF
            last_send_ms = pre_uart_ms

        if detector_error is not None:
            print("DETECTOR_ERROR=%s" % detector_error)
        if config.DEBUG_ENABLED and time.ticks_diff(
                pre_uart_ms, last_debug_ms) >= config.DEBUG_PERIOD_MS:
            print(_format_gray_vdbg(selection, surface_result, road_result,
                                    mask_info, reference, valid, raw_error,
                                    filtered_error, last_tx_sequence,
                                    tx_error_count, filter_reset_reason,
                                    processing_ms, frame_age_ms, fps))
            last_debug_ms = pre_uart_ms

        _draw_gray_result(img, selection, surface_result, road_result,
                          reference, valid, raw_error, filtered_error,
                          processing_ms, fps)


class _VisionUARTWriter:
    """Select a single-call FIFO writer only when the UART advertises API 1."""

    FIFO_QUEUED = "FIFO_QUEUED"
    SYNCHRONOUS_FALLBACK = "SYNCHRONOUS_FALLBACK"

    def __init__(self, uart):
        self._uart = uart
        self._fifo_write = None
        self._fifo_txdone = None
        api_version = getattr(uart, "WRITE_FIFO_API", None)
        if api_version is None:
            api_version = getattr(type(uart), "WRITE_FIFO_API", None)
        api_v1 = type(api_version) is int and api_version == 1
        fifo_write = getattr(uart, "write_fifo", None)
        fifo_txdone = getattr(uart, "fifo_txdone", None)
        if api_v1 and callable(fifo_write) and callable(fifo_txdone):
            self._fifo_write = fifo_write
            self._fifo_txdone = fifo_txdone
            self.mode = self.FIFO_QUEUED
        else:
            self.mode = self.SYNCHRONOUS_FALLBACK

    def write(self, data):
        if self.mode == self.FIFO_QUEUED:
            return self._fifo_write(data)
        return self._uart.write(data)

    def any(self, *args, **kwargs):
        return self._uart.any(*args, **kwargs)

    def readinto(self, *args, **kwargs):
        return self._uart.readinto(*args, **kwargs)

    def fifo_txdone(self):
        """Probe FIFO completion only outside the frame hot path."""
        if self.mode != self.FIFO_QUEUED:
            return None
        return self._fifo_txdone()

    class CameraDiagnostics:
        """Rate-limited D1 sideband sender with isolated error accounting."""

        def __init__(self, build_code):
            self.packet = bytearray(vision_protocol.FRAME_SIZE)
            self.build_code = build_code
            self.sequence = 0
            self.last_attempt_ms = None
            self.capability_was_fresh = False
            self.identity_sent = False
            self.last_status_signature_valid = False
            self.last_status_flags = 0
            self.last_calibration_code = 0
            self.last_observation_code = 0
            self.last_reverse_code = 0
            self.last_profile_nibbles = 0
            self.next_kind = _CAMERA_DIAG_STATUS
            self.status_since_counters = 0
            self._attempt_ready = False
            self.last_sideband_elapsed_us = 0
            self.errors = 0

        def _record_error(self):
            if self.errors < 0xFFFF:
                self.errors += 1

        def _write(self, uart, kind, value1, value2, value3, value4,
                   value5, value6, value7):
            sequence = self.sequence
            self.sequence = (sequence + 1) & 0xFFFF
            vision_protocol.pack_diagnostic_into(
                self.packet, sequence, kind, value1, value2, value3, value4,
                value5, value6, value7)
            try:
                written = uart.write(self.packet)
            except Exception:
                self._record_error()
                return False
            if written != vision_protocol.FRAME_SIZE:
                self._record_error()
                return False
            return True

        def observe_capability(self, enabled):
            """Remember loss and reacquisition of the fresh H743 capability."""
            if not enabled:
                self.capability_was_fresh = False
                return
            if not self.capability_was_fresh:
                self.identity_sent = False
                self.last_status_signature_valid = False
                self.next_kind = _CAMERA_DIAG_STATUS
                self.status_since_counters = 0
            self.capability_was_fresh = True

        def begin_attempt(self, uart, now_ms, ticks_diff, interval_ms,
                          enabled, primary_full_write, primary_late,
                          remaining_us, uart_mode=None):
            """Cheap eligibility gate, run before preparing record fields."""
            self._attempt_ready = False
            self.observe_capability(enabled)
            if not enabled:
                return False
            if interval_ms < 1000:
                interval_ms = 1000
            if not primary_full_write or primary_late:
                return False
            if self.last_attempt_ms is not None:
                age_ms = ticks_diff(now_ms, self.last_attempt_ms)
                if age_ms < interval_ms:
                    return False

            mode = (getattr(uart, "mode", None) if uart_mode is None else
                    uart_mode)
            if mode == _VisionUARTWriter.FIFO_QUEUED:
                if remaining_us < _CAMERA_DIAG_FIFO_RESERVE_US:
                    return False
                try:
                    if not uart.fifo_txdone():
                        return False
                except Exception:
                    self.last_attempt_ms = now_ms
                    self._record_error()
                    return False
            elif (mode == _VisionUARTWriter.SYNCHRONOUS_FALLBACK or
                  mode == "SINK"):
                if remaining_us < _CAMERA_DIAG_SYNC_RESERVE_US:
                    return False
            else:
                return False

            self.last_attempt_ms = now_ms
            self._attempt_ready = True
            return True

        def send_prepared(self, uart, status_flags, calibration_code,
                          observation_code, reverse_code, raw_profile_ack,
                          outgoing_profile_ack, vision_sequence,
                          vision_tx_errors, detector_errors,
                          reverse_rejects):
            """Write one record after ``begin_attempt`` accepted the cycle."""
            if not self._attempt_ready:
                return False
            self._attempt_ready = False
            if not self.identity_sent:
                build_code = self.build_code
                sent = self._write(
                    uart, _CAMERA_DIAG_IDENTITY, 1,
                    build_code & 0xFF, (build_code >> 8) & 0xFF,
                    (build_code >> 16) & 0xFF, (build_code >> 24) & 0xFF,
                    1, 0)
                if sent:
                    self.identity_sent = True
                    self.next_kind = _CAMERA_DIAG_STATUS
                return sent

            if raw_profile_ack < 0 or raw_profile_ack > 3:
                raw_profile_ack = 0
            if outgoing_profile_ack < 0 or outgoing_profile_ack > 3:
                outgoing_profile_ack = 0
            profile_nibbles = (raw_profile_ack << 4) | outgoing_profile_ack
            kind = self.next_kind
            state_changed = bool(
                not self.last_status_signature_valid or
                self.last_status_flags != status_flags or
                self.last_calibration_code != calibration_code or
                self.last_observation_code != observation_code or
                self.last_reverse_code != reverse_code or
                self.last_profile_nibbles != profile_nibbles)
            if state_changed and self.next_kind == _CAMERA_DIAG_COUNTERS:
                if self.status_since_counters >= 2:
                    kind = _CAMERA_DIAG_COUNTERS
                else:
                    kind = _CAMERA_DIAG_STATUS
            elif state_changed:
                kind = _CAMERA_DIAG_STATUS

            if kind == _CAMERA_DIAG_STATUS:
                sent = self._write(
                    uart, kind, status_flags, calibration_code,
                    observation_code, reverse_code, profile_nibbles,
                    vision_sequence & 0xFF, (vision_sequence >> 8) & 0xFF)
                if sent:
                    if self.status_since_counters < 2:
                        self.status_since_counters += 1
                    self.last_status_signature_valid = True
                    self.last_status_flags = status_flags
                    self.last_calibration_code = calibration_code
                    self.last_observation_code = observation_code
                    self.last_reverse_code = reverse_code
                    self.last_profile_nibbles = profile_nibbles
                    self.next_kind = _CAMERA_DIAG_COUNTERS
                return sent

            saturated = 0
            if vision_tx_errors < 0:
                vision_tx_errors = 0
            elif vision_tx_errors > 0xFFFF:
                vision_tx_errors = 0xFFFF
                saturated = 1
            if detector_errors < 0:
                detector_errors = 0
            elif detector_errors > 0xFFFF:
                detector_errors = 0xFFFF
                saturated = 1
            if reverse_rejects < 0:
                reverse_rejects = 0
            elif reverse_rejects > 0xFFFF:
                reverse_rejects = 0xFFFF
                saturated = 1
            sent = self._write(
                uart, _CAMERA_DIAG_COUNTERS, saturated,
                vision_tx_errors & 0xFF, (vision_tx_errors >> 8) & 0xFF,
                detector_errors & 0xFF, (detector_errors >> 8) & 0xFF,
                reverse_rejects & 0xFF, (reverse_rejects >> 8) & 0xFF)
            if sent:
                self.next_kind = _CAMERA_DIAG_STATUS
                self.status_since_counters = 0
            return sent

        def send_after_primary(self, uart, now_ms, ticks_diff, interval_ms,
                               enabled, primary_full_write, primary_late,
                               remaining_us, status_flags, calibration_code,
                               observation_code, reverse_code, raw_profile_ack,
                               outgoing_profile_ack, vision_sequence,
                               vision_tx_errors, detector_errors,
                               reverse_rejects):
            """Combined test-friendly gate and sender entry point."""
            if not self.begin_attempt(
                    uart, now_ms, ticks_diff, interval_ms, enabled,
                    primary_full_write, primary_late, remaining_us,
                    getattr(uart, "mode", None)):
                return False
            return self.send_prepared(
                uart, status_flags, calibration_code, observation_code,
                reverse_code, raw_profile_ack, outgoing_profile_ack,
                vision_sequence, vision_tx_errors, detector_errors,
                reverse_rejects)

        @staticmethod
        def calibration_code(reason, sensor_identity_valid):
            if not sensor_identity_valid:
                return 6
            if reason == "CALIBRATION_READY":
                return 1
            if reason == "FILE_NOT_FOUND":
                return 2
            if reason == "CRC_MISMATCH":
                return 3
            if reason == "GEOMETRY_MISMATCH":
                return 4
            if reason in ("INVALID_CALIBRATION_DATA", "INVALID_FILE"):
                return 5
            if reason == "SENSOR_ID_MISMATCH":
                return 6
            if reason == "NO_CALIBRATION":
                return 7
            if reason in (
                    "INSUFFICIENT_SUPPORT",
                    "INSUFFICIENT_CALIBRATION_CHAIN",
                    "REFERENCE_WIDTH_UNAVAILABLE",
                    "NEAR_WIDTH_SUPPORT_INSUFFICIENT"):
                return 8
            if reason is None or reason == "UNKNOWN":
                return 0
            return 9

        @staticmethod
        def observation_code(reason, detector_error, primary_valid,
                             selection_source):
            if primary_valid:
                return 1
            if detector_error is not None or reason == "DETECTOR_EXCEPTION":
                return 3
            if reason == "RING_CALIBRATION_REQUIRED":
                return 2
            if reason == "FRAME_BUDGET":
                return 4
            if reason in ("NO_ROUTE", "NO_SUPPORTED_ROUTE"):
                return 5
            if reason in ("ELEMENT_SESSION_LEASE_EXPIRED", "SESSION_EXPIRED"):
                return 6
            if selection_source == "NONE":
                return 5
            if reason is None or reason == "UNKNOWN":
                return 0
            return 7

        @staticmethod
        def reverse_code(reason):
            if reason == "STARTUP":
                return 0
            if reason == "RESET":
                return 1
            if reason in ("KEEP", "DUPLICATE_KEEP"):
                return 2
            if reason == "CONTROL_TIMEOUT":
                return 3
            if reason == "KEEP_WITHOUT_RESET":
                return 4
            if reason in ("UART_ERROR", "RX_ERROR", "RX_OVERFLOW",
                          "OBSERVATION_UART_ERROR", "REVERSE_UART_ERROR"):
                return 5
            if reason in ("BAD_VERSION", "BAD_COMMAND", "BAD_CRC",
                          "SEQUENCE_ROLLBACK", "PROFILE_SEQUENCE_ROLLBACK",
                          "BAD_PROFILE_TYPE", "BAD_PROFILE_CRC",
                          "BAD_PROFILE_FLAGS", "PROFILE_PARTIAL_TIMEOUT"):
                return 6
            if reason is None:
                return 0
            return 7


_CAMERA_DIAG_STATUS = 1
_CAMERA_DIAG_COUNTERS = 2
_CAMERA_DIAG_IDENTITY = 3
_CAMERA_DIAG_FIFO_RESERVE_US = 1000
# Synchronous UART.write can block; 16 bytes at 115200/8N1 occupy 1.39 ms
# nominally. The reserve adds margin; actual hardware overhead is unverified.
_CAMERA_DIAG_SYNC_RESERVE_US = 22000


def _main_fast(duration_s=None, run_id=0, frame_source=None, uart_sink=None,
               fixture_sensor_id=None):
    """Process every frame through the V2 path, with optional RAM fixtures.

    Supplying either fixture requires both: this keeps runtime benchmarks off
    the physical camera and H743 UART while exercising the production path.
    """
    global _RING_DIAGNOSTICS, _RING_DIAG_SESSION
    _validate_fast_config()
    fixture_mode = frame_source is not None or uart_sink is not None
    if fixture_sensor_id is not None:
        if not fixture_mode:
            raise ValueError(
                "fixture_sensor_id is only valid in fixture mode")
        if (isinstance(fixture_sensor_id, bool) or
                not isinstance(fixture_sensor_id, int)):
            raise TypeError("fixture_sensor_id must be an integer sensor ID")
        if fixture_sensor_id != ring_width_calibration.SENSOR_ID:
            raise ValueError("fixture_sensor_id must identify sensor 0x1324")
    native_preview_api = False
    usb_vcp = None
    fixture_usb_vcp_provided = False
    preview_source = None
    if fixture_mode:
        if frame_source is None or uart_sink is None:
            raise ValueError("fixture mode requires frame_source and uart_sink")
        if not callable(getattr(frame_source, "snapshot", None)):
            raise TypeError("frame_source must provide snapshot()")
        if not callable(getattr(uart_sink, "write", None)):
            raise TypeError("uart_sink must provide write()")
        if (not callable(getattr(uart_sink, "any", None)) or
                not callable(getattr(uart_sink, "readinto", None))):
            raise TypeError("uart_sink must provide any() and readinto()")
        sensor_id = fixture_sensor_id
        preview_flush = getattr(frame_source, "flush", None)
        if not callable(preview_flush):
            preview_flush = _noop_preview_flush
    if fixture_mode:
        native_preview_api = _is_native_usb_preview_source(frame_source)
        if native_preview_api:
            preview_source = frame_source
            preview_source.configure(config.USB_PREVIEW_ENABLED,
                                     config.USB_PREVIEW_QUALITY)
            usb_vcp = preview_source._usb_vcp
            fixture_usb_vcp_provided = True
            preview_flush = preview_source.flush
        else:
            usb_vcp, fixture_usb_vcp_provided = _fixture_usb_vcp(
                frame_source)
            preview_flush = getattr(frame_source, "flush", None)
            if not callable(preview_flush):
                preview_flush = _noop_preview_flush
    else:
        sensor_id = _configure_gray_sensor(sensor, fast=True)
        native_preview_api = _native_preview_control_api_available()
        usb_vcp = (_open_usb_vcp_safely()
                   if config.USB_PREVIEW_ENABLED else None)
        if native_preview_api:
            # This object latches USB state at snapshot and publishes only
            # that same frame after processing, UART and overlay drawing.
            preview_source = _NativeUSBPreviewSource(
                sensor, usb_vcp, config.USB_PREVIEW_ENABLED,
                config.USB_PREVIEW_QUALITY)
            preview_flush = preview_source.flush
        else:
            # Older firmware keeps the original capture/flush behavior.
            preview_flush = sensor.flush
    compact_usb_overlay = bool(
        native_preview_api and config.USB_PREVIEW_ENABLED and
        config.USB_PREVIEW_OVERLAY_MODE == "compact")
    gray_builder = fast_gray.FastGrayMasks(config)
    path_detector = fast_path.FastPathDetector(config)
    boundary_validator = fast_boundary.FastBoundaryEvidence(config)
    single_edge_tracker = fast_single_edge.FastSingleEdgeTracker(config)
    contour_recovery = (
        contour_route.ContourRouteRecovery(config)
        if config.CONTOUR_RECOVERY_ENABLED else None)
    reverse_parser = reverse_control.ReverseControlParser(
        reverse_control.DEFAULT_TIMEOUT_MS,
        config.PROFILE_STATE_TIMEOUT_MS)
    steering_diag_session = _SteeringDiagSession(_STEERING_DIAG)
    sensor_get_exposure = (None if fixture_mode else
                           getattr(sensor, "get_exposure_us", None))
    exposure_us = None
    if callable(sensor_get_exposure):
        try:
            exposure_us = sensor_get_exposure()
        except Exception:
            exposure_us = None
    sensor_get_gain = (None if fixture_mode else
                       getattr(sensor, "get_gain_db", None))
    gain_db = None
    if callable(sensor_get_gain):
        try:
            gain_db = sensor_get_gain()
        except Exception:
            gain_db = None
    exposure_cap_us = getattr(config, "AUTO_EXPOSURE_MAX_US", None) \
        if config.AUTO_EXPOSURE else None
    if exposure_cap_us is not None and not fixture_mode:
        if exposure_us is None:
            raise RuntimeError(
                "V7 cannot verify the configured auto-exposure limit")
        if exposure_us > exposure_cap_us:
            raise RuntimeError(
                "V7 sensor exposure %dus exceeds %dus limit" %
                (exposure_us, exposure_cap_us))
    control_profile = config.STEERING_CONTROL_PROFILE

    sensor_id_text = ("0x%04X" % (sensor_id & 0xFFFF)
                      if isinstance(sensor_id, int) else
                      "unavailable" if sensor_id is None else str(sensor_id))
    contour_backend = (
        getattr(contour_trace, "COMPILER_BACKEND", "UNKNOWN")
        if contour_recovery is not None else "disabled")
    source_name = (getattr(frame_source, "name", "RAM_FIXTURE")
                   if fixture_mode else "SENSOR")
    print("V7_BUILD=%s backend=fast source=%s route=%s v7_api=%d single_api=%d "
          "ring_route=profile_gated_experimental slope_classifier=disabled "
          "slope_hint=always0 contour_api=%d contour_backend=%s "
          "sensor_id=%s "
          "camera=320x120 crop=QVGA(0,60,320,120) grid=80x60 floor=%d budget=%dms "
          "detail_text=%dms control_profile=%s preview=per_frame "
          "uart=3@115200/8N1 exposure_us=%s "
          "exposure_cap_us=%s gain_db=%s "
          "gray_backend=%s topology_backend=%s boundary_backend=%s "
          "single_backend=%s native_row_preview=%d "
          "reverse=v1-8+v2-20/ccitt-false profile_timeout_ms=%d " % (
              config.BUILD_ID, source_name, config.SINGLE_EDGE_ROUTE_MODE,
              V7_RUNTIME_API_VERSION,
              SINGLEEDGE_RUNTIME_API_VERSION,
              CONTOUR_RUNTIME_API_VERSION, contour_backend,
              sensor_id_text,
              config.SURFACE_ROI_FLOOR_Y, config.SURFACE_FRAME_BUDGET_MS,
              config.OVERLAY_PERIOD_MS,
              control_profile,
              str(exposure_us), str(exposure_cap_us), str(gain_db),
              getattr(fast_gray, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_topology, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_boundary, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_single_edge, "COMPILER_BACKEND", "UNKNOWN"),
              1 if _NATIVE_ACCELERATION_ENABLED else 0,
              config.PROFILE_STATE_TIMEOUT_MS))
    path_accel_status = "unavailable"
    try:
        path_accel = fast_path._path_accel_backend()
        if fast_path._path_accel_supported(path_detector, path_accel):
            path_accel_status = "cartest_path_accel-api1"
    except Exception:
        path_accel_status = "unverified"
    print("V25_CAPS path_accel=%s topology_accel=%s native_emit=%d "
          "native_preview=%d perf_ref_includes=ring,row_weight" % (
              path_accel_status,
              getattr(fast_topology, "NATIVE_ACCEL_BACKEND", "unavailable"),
              1 if _NATIVE_ACCELERATION_ENABLED else 0,
              1 if native_preview_api else 0))
    print("TRACK_BUILD=%s forward=V3/16 reverse=V1/8+V2/20 "
          "profile=awaiting_state ring=profile_gated_experimental "
          "cross=experimental" % config.TRACK_PACKAGE_BUILD_ID)

    if (fixture_mode and not native_preview_api and
            not fixture_usb_vcp_provided):
        preview_state = "RAM_FIXTURE"
    elif not config.USB_PREVIEW_ENABLED:
        preview_state = ("NATIVE_DISABLED" if native_preview_api else
                         "LEGACY_DISABLED")
    elif _usb_vcp_is_connected(usb_vcp):
        preview_state = ("NATIVE_ACTIVE" if native_preview_api else
                         "LEGACY_ACTIVE")
    else:
        preview_state = ("NATIVE_HEADLESS" if native_preview_api else
                         "LEGACY_HEADLESS")
    last_usb_preview_state = preview_state
    last_preview_error_ms = None
    print("USB_PREVIEW_CANDIDATE build=%s quality=Q%d mode=%s overlay=%s "
          "rate=unverified" % (
              config.USB_PREVIEW_BUILD_ID, config.USB_PREVIEW_QUALITY,
              preview_state, config.USB_PREVIEW_OVERLAY_MODE))
    print(_preview_codec_guard_diagnostic())

    if fixture_mode:
        uart = uart_sink
        uart_tx_mode = "SINK"
    else:
        hardware_uart = pyb.UART(
            config.UART_ID, config.UART_BAUDRATE,
            bits=config.UART_BITS, parity=None,
            stop=config.UART_STOP_BITS,
            timeout=config.UART_TIMEOUT_MS,
            timeout_char=config.UART_CHAR_TIMEOUT_MS)
        uart = _VisionUARTWriter(hardware_uart)
        uart_tx_mode = uart.mode
    print("UART_TX_MODE=%s" % uart_tx_mode)
    observation_filter = _FreshObservationFilter(
        config.OBSERVATION_FILTER_ALPHA,
        config.OBSERVATION_FILTER_MAX_GAP_MS,
        0)  # Quality is diagnostic only for this fast geometry path.
    row_weight_workspace = _FastRowWeightWorkspace()
    position_pd = _FastPositionPD(config)
    adaptive_preview = _FastAdaptivePreview(config)
    speed_stage = _FastSpeedStage(config)
    ring_calibration = ring_width_calibration.RingWidthCalibration.load(
        config.RING_WIDTH_CALIBRATION_PATH)
    sensor_identity_valid = ((fixture_mode and fixture_sensor_id is None) or
                             sensor_id == ring_width_calibration.SENSOR_ID)
    calibration_for_tracker = (ring_calibration
                               if sensor_identity_valid else None)
    element_tracker = track_elements.TrackElementsTracker(
        config, calibration_for_tracker,
        require_calibration=config.RING_WIDTH_CALIBRATION_REQUIRED)
    observe_element_route_outcome = getattr(
        element_tracker, "observe_route_outcome", None)
    ring_calibration_passed = bool(
        element_tracker.ring_calibration_ready and
        element_tracker.ring_calibration_check_ready)
    ring_stationary_status_frames = 0
    ring_stationary_last_profile_sequence = None
    ring_stationary_last_profile_id = None
    ring_stationary_last_profile_token = None
    ring_stationary_last_wheel_position = None
    ring_calibration_last_eligible = False
    _RING_DIAGNOSTICS = (
        ring_diagnostics.RingDiagnostics()
        if config.RING_DIAGNOSTICS_ENABLED else None)
    _RING_DIAG_SESSION = (
        _RingDiagnosticsSession(
            _RING_DIAGNOSTICS, config.RING_DIAGNOSTICS_PATH,
            config.TRACK_PACKAGE_BUILD_ID)
        if _RING_DIAGNOSTICS is not None else None)
    calibration_status = (
        element_tracker.ring_calibration_reason
        if sensor_identity_valid else "SENSOR_ID_MISMATCH")
    print("RING_WIDTH_GATE map=%s check=%s required=%d "
          "diagnostics=%d" % (
              calibration_status,
              "READY" if ring_calibration_passed else "PARKED_CHECK_REQUIRED",
              1 if config.RING_WIDTH_CALIBRATION_REQUIRED else 0,
              1 if _RING_DIAGNOSTICS is not None else 0))
    element_result = element_tracker.result
    element_input_row_status = getattr(
        single_edge_tracker, "_row_status", None)
    element_input_row_edge2 = getattr(
        single_edge_tracker, "_row_edge2", None)
    element_input_row_contrast = getattr(
        single_edge_tracker, "_row_contrast", None)
    element_input_cache_valid = getattr(
        single_edge_tracker, "_cache_valid", None)
    element_input_cache_width = getattr(
        single_edge_tracker, "_cache_width", None)
    element_input_cache_left2 = getattr(
        single_edge_tracker, "_cache_left2", None)
    element_input_cache_right2 = getattr(
        single_edge_tracker, "_cache_right2", None)
    grid_height = config.SURFACE_GRID_HEIGHT
    if element_input_row_status is None:
        element_input_row_status = bytearray(2 * grid_height)
    if element_input_row_edge2 is None:
        element_input_row_edge2 = array("i", [0] * (2 * grid_height))
    if element_input_row_contrast is None:
        element_input_row_contrast = array("i", [0] * (2 * grid_height))
    if element_input_cache_valid is None:
        element_input_cache_valid = bytearray(grid_height)
    if element_input_cache_width is None:
        element_input_cache_width = array("i", [0] * grid_height)
    if element_input_cache_left2 is None:
        element_input_cache_left2 = array("i", [0] * grid_height)
    if element_input_cache_right2 is None:
        element_input_cache_right2 = array("i", [0] * grid_height)
    ring_route_rows = array(
        "i", [0] * (config.SURFACE_GRID_HEIGHT *
                    fast_single_edge.ROW_STRIDE))
    ring_route_result = {
        "valid": False, "quality": 0, "reason": "RING_ROUTE_INACTIVE",
        "scene": "RING_ACTIVE", "lateral_error": 0,
        "selected_row_ints": ring_route_rows, "selected_row_count": 0,
        "selected_row_stride": fast_single_edge.ROW_STRIDE,
        "selected_rows": None, "reference_center_x": 0.0,
        "reference_y": 0.0, "reference_provenance": "RING_WIDTH_CACHE",
        "boundary_source": "RING_WIDTH_CACHE", "inferred": True,
        "inference_flags": 0, "observed_both_rows": 0,
        "inferred_rows": 0, "actual_boundary": "NONE",
        "actual_boundary_x": None, "weak_support": False,
    }
    ring_selection = {
        "valid": False, "quality": 0, "lateral_error": 0,
        "reason": "RING_ROUTE_INACTIVE", "source": "NONE",
        "rejected_source": None, "result": ring_route_result,
        "comparison": {}, "boundary_source": "RING_WIDTH_CACHE",
        "inferred": True, "inference_flags": 0,
        "ring_candidate": element_result,
    }
    cross_route_result = {
        "valid": False, "quality": 0,
        "reason": "CROSS_STRAIGHT_UNAVAILABLE", "scene": "CROSS_STRAIGHT",
        "lateral_error": 0, "selected_row_ints": None,
        "selected_row_count": 0, "selected_row_stride": 4,
        "selected_rows": None, "reference_center_x": 0.0,
        "reference_y": 0.0, "reference_provenance": "CROSS_STRAIGHT",
        "topology_evidence": None,
    }
    cross_selection = {
        "valid": False, "quality": 0, "lateral_error": 0,
        "reason": "CROSS_STRAIGHT_UNAVAILABLE", "source": "road",
        "rejected_source": "road", "result": cross_route_result,
        "comparison": {}, "boundary_source": "CROSS_STRAIGHT",
    }
    packet = bytearray(vision_protocol.FRAME_SIZE)
    camera_diagnostics = (
        _VisionUARTWriter.CameraDiagnostics(config.CAMERA_UART_DIAG_BUILD_CODE)
        if config.CAMERA_UART_DIAG_ENABLED else None)
    camera_diag_capability_check = getattr(
        reverse_parser, "profile_diagnostics_enabled", None)
    camera_diag_profile_ack = getattr(reverse_parser, "profile_ack", None)
    camera_diag_profile_fresh = getattr(reverse_parser, "profile_is_fresh", None)
    camera_diag_control_fresh = getattr(reverse_parser, "control_is_fresh", None)
    camera_diag_observe = (None if camera_diagnostics is None else
                           camera_diagnostics.observe_capability)
    camera_diag_begin = (None if camera_diagnostics is None else
                         camera_diagnostics.begin_attempt)
    camera_diag_send = (None if camera_diagnostics is None else
                        camera_diagnostics.send_prepared)
    camera_diag_calibration_code = (
        None if camera_diagnostics is None else
        _VisionUARTWriter.CameraDiagnostics.calibration_code)
    camera_diag_observation_code = (
        None if camera_diagnostics is None else
        _VisionUARTWriter.CameraDiagnostics.observation_code)
    camera_diag_reverse_code = (
        None if camera_diagnostics is None else
        _VisionUARTWriter.CameraDiagnostics.reverse_code)
    camera_diag_interval_ms = config.CAMERA_UART_DIAG_INTERVAL_MS
    if camera_diag_interval_ms < 1000:
        camera_diag_interval_ms = 1000
    reverse_input_buffer = bytearray(reverse_control.MAX_FEED_BYTES)
    reverse_drain_state = array("i", [0])
    reverse_generation = reverse_parser.generation
    profile_generation = reverse_parser.profile_generation
    last_capture_end_ms = None
    sequence = 0
    preview_frame_id = -1
    ring_stop_reason = None
    tx_error_count = 0
    last_tx_result = "none"
    last_debug_ms = time.ticks_ms()
    overlay_detail_image = None
    overlay_detail_mask = None
    overlay_status_image = None
    overlay_status_mask = None
    if config.OVERLAY_ENABLED:
        try:
            import image as image_module
            image_type = getattr(image_module, "Image", None)
            grayscale_format = getattr(image_module, "GRAYSCALE", None)
            if callable(image_type) and grayscale_format is not None:
                if not compact_usb_overlay:
                    overlay_detail_image = image_type(
                        _OVERLAY_DETAIL_IMAGE_WIDTH, 40, grayscale_format)
                    overlay_detail_mask = image_type(
                        _OVERLAY_DETAIL_IMAGE_WIDTH, 40, grayscale_format)
                try:
                    overlay_status_image = image_type(
                        _OVERLAY_STATUS_IMAGE_WIDTH,
                        _OVERLAY_STATUS_IMAGE_HEIGHT, grayscale_format)
                    if _OVERLAY_STATUS_USE_MASK:
                        overlay_status_mask = image_type(
                            _OVERLAY_STATUS_IMAGE_WIDTH,
                            _OVERLAY_STATUS_IMAGE_HEIGHT, grayscale_format)
                        if overlay_status_mask is None:
                            overlay_status_image = None
                except Exception:
                    # Keep the original full-string overlay when a status
                    # bitmap cannot be allocated on this camera build.
                    overlay_status_image = None
                    overlay_status_mask = None
        except ImportError:
            # Host-only injected frame sources may not provide OpenMV's image
            # module. Their test double uses the direct text fallback.
            pass
    overlay_state = _V7OverlayState(
        last_debug_ms, overlay_detail_image, overlay_detail_mask,
        overlay_status_image, overlay_status_mask)
    overlay_state.compact_usb = compact_usb_overlay
    previous_source = None
    previous_reference_kind = None
    previous_reference_y = None
    previous_reference_x = None
    detector_error_count = 0
    latest_detector_error = "-"
    no_surface_result = {
        "valid": False, "quality": 0,
        "reason": "FAST_BACKEND_SURFACE_DISABLED",
        "scene": "UNKNOWN", "sections": (), "target": None,
    }
    frame_count = 0
    gc_period_frames = config.PERF_GC_PERIOD_FRAMES
    if gc_period_frames > config.PERF_GC_MAX_INTERVAL_FRAMES:
        gc_period_frames = config.PERF_GC_MAX_INTERVAL_FRAMES
    capture_fps_counter = _FreshCaptureFPS()
    capture_fps_value = 0.0
    bench_start_us = time.ticks_us()
    bench_start_ms = time.ticks_ms()
    duration_ms = None if duration_s is None else int(duration_s) * 1000
    current_window = 0
    window_us = config.PERF_WINDOW_MS * 1000
    window_start_us = bench_start_us
    perf = _V7PerfWindow()
    summary_queue = _V7SummaryQueue()
    last_heap_sample_ms = None
    heap_free_fn = None
    heap_alloc_fn = None
    heap_stats_fn = None
    if cartest_accel is not None:
        try:
            if getattr(cartest_accel, "HEAP_STATS_API", 0) == 1:
                candidate_heap_stats = getattr(cartest_accel, "heap_stats", None)
                if callable(candidate_heap_stats):
                    heap_stats_fn = candidate_heap_stats
        except Exception:
            heap_stats_fn = None
    gc_cached_free_bytes = -1
    gc_low_memory_latched = False
    pending_cycle_start_us = None

    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), bench_start_ms) < duration_ms):
        cycle_start_us = time.ticks_us()
        # The previous frame's complete cycle ends at this start timestamp.
        # Record it before this frame can move the active send window.
        if pending_cycle_start_us is not None:
            perf.record("total_cycle", time.ticks_diff(
                cycle_start_us, pending_cycle_start_us))
            pending_cycle_start_us = None

        capture_start_us = time.ticks_us()
        try:
            if fixture_mode:
                img = frame_source.snapshot()
            elif native_preview_api:
                img = preview_source.snapshot()
            else:
                img = sensor.snapshot()
        except Exception:
            failed_capture_ms = time.ticks_ms()
            if last_capture_end_ms is not None:
                failed_capture_gap = time.ticks_diff(
                    failed_capture_ms, last_capture_end_ms)
                if failed_capture_gap < 0 or failed_capture_gap >= \
                        reverse_control.DEFAULT_TIMEOUT_MS:
                    reverse_parser.invalidate("CAPTURE_GAP")
                    reverse_drain_state[0] = 1
                    single_edge_tracker.clear("CAPTURE_GAP")
                    element_tracker.reset("CAPTURE_GAP")
                    observation_filter.reset()
                    position_pd.reset("CAPTURE_GAP")
                    adaptive_preview.reset()
                    previous_source = None
                    previous_reference_kind = None
                    previous_reference_y = None
                    previous_reference_x = None
            raise
        capture_end_us = time.ticks_us()
        preview_frame_id = (preview_frame_id + 1) & 0xFFFFFF
        single_edge_tracker.begin_frame()
        capture_us = time.ticks_diff(capture_end_us, capture_start_us)
        capture_end_ms = time.ticks_ms()
        capture_fps_counter.record_success(capture_end_ms)
        capture_fps_value = capture_fps_counter.fps(capture_end_ms)
        capture_interrupted = False
        capture_dt_ms = None
        if last_capture_end_ms is not None:
            capture_dt_ms = time.ticks_diff(
                capture_end_ms, last_capture_end_ms)
            if capture_dt_ms < 0 or capture_dt_ms >= \
                    reverse_control.DEFAULT_TIMEOUT_MS:
                capture_interrupted = True
                reverse_parser.invalidate("CAPTURE_GAP")
                reverse_drain_state[0] = 1
        last_capture_end_ms = capture_end_ms

        reverse_fresh = _service_reverse_control(
            uart, reverse_parser, reverse_input_buffer, reverse_drain_state,
            capture_end_ms, time.ticks_diff)
        ring_stop_reason = steering_diag_session.observe(
            reverse_parser, single_edge_tracker, config.BUILD_ID,
            reverse_fresh, capture_end_ms, time.ticks_diff)
        if camera_diag_observe is not None:
            capability_fresh = bool(
                callable(camera_diag_capability_check) and
                camera_diag_capability_check(capture_end_ms,
                                             time.ticks_diff))
            camera_diag_observe(capability_fresh)
        control_session_changed = (
            reverse_parser.generation != reverse_generation)
        profile_session_changed = (
            reverse_parser.profile_generation != profile_generation)
        if control_session_changed or profile_session_changed:
            reset_event = (reverse_parser.last_profile_event
                           if profile_session_changed else
                           reverse_parser.last_event)
            speed_stage.lifecycle_event(
                "RESET" if profile_session_changed else
                reverse_parser.last_event)
            single_edge_tracker.clear(reset_event)
            element_tracker.reset(reset_event)
            observation_filter.reset()
            reset_reason = (
                reverse_parser.last_profile_event
                if profile_session_changed else
                "CONTROL_TIMEOUT" if "TIMEOUT" in
                reverse_parser.last_event else "REVERSE_RESET")
            position_pd.reset(reset_reason)
            adaptive_preview.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            reverse_generation = reverse_parser.generation
            profile_generation = reverse_parser.profile_generation
        if capture_interrupted:
            single_edge_tracker.clear("CAPTURE_GAP")
            element_tracker.reset("CAPTURE_GAP")
            observation_filter.reset()
            position_pd.reset("CAPTURE_GAP")
            adaptive_preview.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            reverse_fresh = False
        try:
            image_size_valid = (img is not None and
                                img.width() == config.IMAGE_WIDTH and
                                img.height() == config.IMAGE_HEIGHT)
        except Exception:
            image_size_valid = False

        masks_start_us = time.ticks_us()
        mask_info = None
        detector_error = None
        if image_size_valid:
            try:
                mask_info = gray_builder.build(img)
                prior_ring = element_tracker.result
                if (reverse_fresh and reverse_parser.profile_session_active and
                        prior_ring.get("profile_id") in (2, 3) and
                        prior_ring.get("ring_confirmed") and
                        prior_ring.get("state_id") in (
                            track_elements.STATE_INSIDE,
                            track_elements.STATE_EXIT_CONFIRM,
                            track_elements.STATE_EXIT_REPAIR) and
                        element_tracker.ring_calibration_ready and
                        element_tracker.ring_calibration_check_ready and
                        getattr(element_tracker, "_ring_cache_frozen", False)):
                    gray_builder.resolve_confirmed_ring_dark_block(mask_info)
            except Exception as exc:
                detector_error = "MASK:%s" % exc
        else:
            detector_error = "FRAME_SIZE_MISMATCH"
        masks_end_us = time.ticks_us()

        path_start_us = time.ticks_us()
        road_result = None
        if mask_info is not None:
            try:
                road_result = path_detector.detect(mask_info)
            except Exception as exc:
                detector_error = "PATH:%s" % exc
        if road_result is None:
            road_result = _invalid_road("DETECTOR_EXCEPTION")
        path_end_us = time.ticks_us()

        # Keep all contour diagnostics and overlay eligibility frame-local;
        # the reusable recovery workspace itself retains its trace buffers.
        contour_candidate = None
        contour_trace_info = None
        contour_attempted = False
        contour_used = False
        contour_reason = ("DISABLED" if contour_recovery is None else
                          "NO_MASK" if mask_info is None else
                          "NOT_NEEDED" if road_result.get("valid") else
                          "NOT_RUN")
        contour_lcount = 0
        contour_rcount = 0
        contour_us = 0
        if (contour_recovery is not None and mask_info is not None and
                not road_result.get("valid")):
            contour_attempted = True
            contour_start_us = time.ticks_us()
            prior_road_result = road_result
            try:
                recovered_result = contour_recovery.detect(
                    mask_info, prior_road_result)
                contour_trace_info = contour_recovery.trace_info
                contour_reason = contour_recovery.reason
                contour_lcount = contour_recovery.Lcount
                contour_rcount = contour_recovery.Rcount
                if (recovered_result is not prior_road_result and
                        isinstance(recovered_result, dict) and
                        recovered_result.get("valid") and
                        contour_recovery.used):
                    contour_candidate = recovered_result
                    road_result = recovered_result
                    contour_used = True
            except Exception as exc:
                detector_error = "CONTOUR:%s" % exc
                contour_reason = "EXCEPTION"
            finally:
                contour_us = time.ticks_diff(
                    time.ticks_us(), contour_start_us)

        boundary_start_us = time.ticks_us()
        if (mask_info is not None and road_result.get("valid")):
            if contour_candidate is road_result:
                road_result, contour_gate_reason, contour_gate_error = \
                    _gate_contour_candidate(
                        road_result, prior_road_result, boundary_validator,
                        mask_info, config)
                if contour_gate_reason is not None:
                    contour_reason = contour_gate_reason
                    contour_candidate = None
                    contour_used = False
                if contour_gate_error is not None:
                    detector_error = "BOUNDARY:%s" % contour_gate_error
            else:
                try:
                    boundary_validator.gate_fast_path_candidate(
                        road_result, mask_info.get("luma"),
                        mask_info.get("edge_mask"),
                        mask_info.get("road_mask"),
                        mask_info.get("known_region"), config)
                except Exception as exc:
                    detector_error = "BOUNDARY:%s" % exc
                    road_result["valid"] = False
                    road_result["quality"] = 0
                    road_result["reason"] = "BOUNDARY_EXCEPTION"
                    road_result["lateral_error"] = 0
        boundary_end_us = time.ticks_us()

        reference_start_us = time.ticks_us()
        geometry_valid = bool(road_result.get("valid"))
        selection = {
            "valid": geometry_valid,
            "quality": road_result.get("quality", 0) if geometry_valid else 0,
            "lateral_error": road_result.get("lateral_error", 0)
                if geometry_valid else 0,
            "reason": road_result.get("reason", "INVALID"),
            "source": "road" if geometry_valid else "NONE",
            "rejected_source": None if geometry_valid else "road",
            "result": road_result if geometry_valid else None,
            "comparison": {},
        }
        reference = None
        boundary_reference = {"valid": False, "reason": "NOT_RUN"}
        if geometry_valid and mask_info is not None:
            selection, reference, boundary_reference = \
                _validate_selected_fast_boundary(
                    selection, road_result, boundary_validator)
        if (contour_candidate is road_result and
                not selection.get("valid")):
            contour_reason = "REFERENCE_%s" % boundary_reference.get(
                "reason", "REJECTED")
            road_result = prior_road_result
            contour_candidate = None
            contour_used = False

        single_candidate = None
        single_diagnostic_reason = (
            "NOT_NEEDED" if selection.get("valid") else "NOT_ATTEMPTED")
        prior_element = element_tracker.result
        ring_observation_only = bool(
            reverse_fresh and reverse_parser.profile_session_active and
            mask_info is not None and prior_element.get("ring_confirmed") and
            prior_element.get("state_id") in (
                track_elements.STATE_INSIDE,
                track_elements.STATE_EXIT_CONFIRM,
                track_elements.STATE_EXIT_REPAIR))
        if not selection.get("valid") and not ring_observation_only:
            if not reverse_fresh:
                if mask_info is not None:
                    single_candidate = single_edge_tracker.detect(
                        mask_info, False,
                        road_result.get("topology_evidence"))
                    single_diagnostic_reason = single_candidate.get(
                        "reason", "NO_FRESH_KEEP")
                else:
                    single_diagnostic_reason = "NO_FRESH_KEEP"
            elif mask_info is None:
                single_diagnostic_reason = "MASK_UNAVAILABLE"
            elif not _single_edge_fallback_allowed(road_result):
                topology = road_result.get("topology_evidence") or {}
                if topology.get("branch_evidence") or topology.get("branch"):
                    single_diagnostic_reason = "TOPOLOGY_BRANCH_UNRESOLVED"
                elif (getattr(config, "SINGLE_EDGE_ROUTE_MODE", None) ==
                      "clipped_midpoint" and
                      (topology.get("island_evidence") or
                       topology.get("island"))):
                    single_diagnostic_reason = "TOPOLOGY_ISLAND_UNRESOLVED"
                elif (topology.get("ambiguous") or road_result.get("scene") in
                      ("AMBIGUOUS_CANDIDATE", "AMBIGUOUS_TOPOLOGY")):
                    single_diagnostic_reason = \
                        "TOPOLOGY_AMBIGUITY_UNRESOLVED"
                else:
                    single_diagnostic_reason = "PATH_BLOCKED:%s" % \
                        road_result.get("reason", "AMBIGUOUS_PATH")
            else:
                single_candidate = single_edge_tracker.detect(
                    mask_info, True,
                    road_result.get("topology_evidence"))
                single_diagnostic_reason = single_candidate.get(
                    "reason", "SINGLE_EDGE_REJECTED")

            if single_candidate is not None and single_candidate.get("valid"):
                single_side = single_candidate["boundary_source"]
                selection = {
                    "valid": True,
                    "quality": single_candidate.get("quality", 0),
                    "lateral_error": single_candidate.get(
                        "lateral_error", 0),
                    "reason": single_candidate.get("reason", "OK_SINGLE"),
                    "source": ("single_left" if single_side == "LEFT_ONLY"
                               else "single_right"),
                    "rejected_source": "road",
                    "result": single_candidate,
                    "comparison": {},
                    "boundary_source": single_side,
                    "inferred": True,
                    "inference_flags": single_candidate.get(
                        "inference_flags", 0),
                    "single_edge_reason": single_diagnostic_reason,
                }
                reference = {
                    "x": single_candidate["reference_center_x"],
                    "y": single_candidate["reference_y"],
                    "kind": single_candidate["reference_provenance"],
                }
                boundary_reference = {
                    "valid": True,
                    "reason": ("CURRENT_EDGE_CLIPPED_MIDPOINT"
                               if single_candidate.get("route_mode") ==
                               "clipped_midpoint" else
                               "CURRENT_EDGE_WIDTH_CACHE_CONTINUITY"),
                    "source": single_side,
                }
            else:
                selection["single_edge_reason"] = single_diagnostic_reason
                selection["single_edge_result"] = single_candidate
                boundary_reference = {
                    "valid": False,
                    "reason": single_diagnostic_reason,
                    "source": "NONE",
                }

        # Once ring takeover is confirmed, a valid ordinary candidate must
        # not suppress this frame's independent boundary observation. Its
        # selected rows may cover only the far field. Do not select or commit
        # this diagnostic single-edge candidate as the ordinary route.
        if ring_observation_only:
            single_candidate = single_edge_tracker.observe_ring_boundaries(
                mask_info, True, road_result.get("topology_evidence"))
            single_diagnostic_reason = "CONFIRMED_RING_BOUNDARY_OBSERVATION"

        if single_candidate is None:
            single_edge_tracker.skip_frame(single_diagnostic_reason)

        bilateral_cache_candidate = _approved_bilateral_cache_candidate(
            selection, road_result, boundary_reference, mask_info)
        normal_cache_commit_allowed = bilateral_cache_candidate is not None

        # Recheck both independent leases immediately before element routing.
        # A stale profile must clear ring state without disabling a still-fresh
        # legacy V1 single-edge KEEP session.
        reverse_parser.poll(time.ticks_ms(), time.ticks_diff)
        control_expired = reverse_parser.generation != reverse_generation
        profile_expired = (
            reverse_parser.profile_generation != profile_generation)
        if control_expired or profile_expired:
            reset_event = (reverse_parser.last_profile_event
                           if profile_expired else reverse_parser.last_event)
            single_edge_tracker.clear(reset_event)
            element_tracker.reset(reset_event)
            observation_filter.reset()
            position_pd.reset(
                "PROFILE_TIMEOUT" if profile_expired else "CONTROL_TIMEOUT")
            adaptive_preview.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            reverse_generation = reverse_parser.generation
            profile_generation = reverse_parser.profile_generation
            if control_expired:
                reverse_fresh = False
                reverse_drain_state[0] = 1
            if selection.get("source") in (
                    "single_left", "single_right") and (
                        control_expired or profile_expired):
                single_diagnostic_reason = reset_event
                selection["valid"] = False
                selection["quality"] = 0
                selection["lateral_error"] = 0
                selection["reason"] = reset_event
                selection["single_edge_reason"] = reset_event
                selection["source"] = "NONE"
                selection["result"] = None
                reference = None
                bilateral_cache_candidate = None
                normal_cache_commit_allowed = False

        topology = road_result.get("topology_evidence") or {}
        profile_now_ms = time.ticks_ms()
        profile_id = reverse_parser.profile_ack(
            profile_now_ms, time.ticks_diff)
        profile_session_active = bool(
            reverse_fresh and profile_id != config.VEHICLE_PROFILE_UNKNOWN and
            reverse_parser.profile_session_active and
            reverse_parser.session_token != 0)
        road_rows = (road_result.get("selected_row_ints")
                     if normal_cache_commit_allowed else None)
        road_count = (road_result.get("selected_row_count", 0)
                      if normal_cache_commit_allowed else 0)
        road_stride = (road_result.get("selected_row_stride", 0)
                       if normal_cache_commit_allowed else 0)
        candidate_tracks = (road_result.get("candidate_tracks", 0)
                            if normal_cache_commit_allowed else 0)
        if not ring_calibration_passed:
            profile_sequence = getattr(
                reverse_parser, "_profile_last_sequence", None)
            profile_token = reverse_parser.session_token
            wheel_position = reverse_parser.mean_forward_wheel_millirev
            parked_profile = bool(
                profile_id in config.RING_ENABLED_PROFILE_IDS and
                not reverse_parser.profile_session_active)
            if parked_profile and profile_sequence is not None:
                if profile_sequence != ring_stationary_last_profile_sequence:
                    if (profile_id == ring_stationary_last_profile_id and
                            profile_token == ring_stationary_last_profile_token and
                            wheel_position == ring_stationary_last_wheel_position):
                        ring_stationary_status_frames += 1
                    else:
                        ring_stationary_status_frames = 1
                    ring_stationary_last_profile_sequence = profile_sequence
                    ring_stationary_last_profile_id = profile_id
                    ring_stationary_last_profile_token = profile_token
                    ring_stationary_last_wheel_position = wheel_position
            else:
                ring_stationary_status_frames = 0
                ring_stationary_last_profile_sequence = None
                ring_stationary_last_profile_id = None
                ring_stationary_last_profile_token = None
                ring_stationary_last_wheel_position = None
            calibration_rows_eligible = bool(
                element_tracker.ring_calibration_ready and
                ring_stationary_status_frames >= 3 and parked_profile and
                not capture_interrupted and normal_cache_commit_allowed and
                road_rows is not None and road_count > 0 and road_stride == 4 and
                road_result.get("valid") and topology.get("reason") == "OK" and
                topology.get("complete") and not topology.get("ambiguous") and
                not topology.get("island_evidence") and
                not topology.get("branch_evidence"))
            if calibration_rows_eligible:
                element_tracker.observe_ring_calibration(
                    road_rows, road_count, road_stride)
                ring_calibration_last_eligible = True
                if element_tracker.ring_calibration_check_ready:
                    ring_calibration_passed = True
            elif ring_calibration_last_eligible:
                element_tracker.reset_ring_calibration_check("PARKED_GATE")
                ring_calibration_last_eligible = False
        single_rows_fresh = bool(getattr(
            single_edge_tracker, "_frame_scan_done", False))
        ring_start_us = time.ticks_us()
        element_result = element_tracker.update(
            profile_now_ms, profile_id, profile_session_active,
            reverse_parser.session_token if profile_id else 0,
            bool(image_size_valid and mask_info is not None),
            normal_cache_commit_allowed, road_rows, road_count, road_stride,
            candidate_tracks, topology, single_rows_fresh,
            element_input_row_status, element_input_row_edge2,
            element_input_row_contrast, element_input_cache_valid,
            element_input_cache_width, element_input_cache_left2,
            element_input_cache_right2, mask_info,
            profile_seq=(reverse_parser._profile_last_sequence
                         if profile_id else 0),
            mean_forward_wheel_millirev=(
                reverse_parser.mean_forward_wheel_millirev
                if profile_id else 0))
        ring_detection_us = time.ticks_diff(time.ticks_us(), ring_start_us)
        ring_candidate = element_result
        ring_info = element_result
        control_road_result = road_result
        # Keep the already validated ordinary route untouched until the ring
        # admission/recovery decision is complete. No row buffers are copied.
        ordinary_selection = selection
        ordinary_reference = reference
        ordinary_boundary_reference = boundary_reference
        ordinary_route_valid = bool(
            normal_cache_commit_allowed and selection.get("valid") and
            selection.get("source") == "road" and reference is not None and
            boundary_reference.get("valid") and not capture_interrupted)
        element_route_selected = False
        ring_profile_enabled = bool(
            profile_session_active and
            reverse_parser.ring_profile_enabled(
                profile_now_ms, time.ticks_diff))

        element_result["route_admission_reason"] = "RING_NOT_ELIGIBLE"
        if ring_profile_enabled and element_result.get("control_eligible"):
            ring_count = _fill_ring_control_route(
                element_result, element_tracker, single_edge_tracker,
                normal_cache_commit_allowed, single_rows_fresh,
                ring_route_result, ring_route_rows, config)
            element_result["route_admission_reason"] = ring_route_result.get(
                "reason", "RING_TARGET_ROWS_OR_REFERENCE")
            if ring_count:
                ring_target_side = element_result.get(
                    "target_side", element_result.get("side"))
                if element_result.get("outer_tracking"):
                    # The observed boundary side differs from ring direction.
                    ring_target_side = element_result.get("side")
                ring_source = ("ring_left" if ring_target_side ==
                               track_elements.SIDE_LEFT else "ring_right")
                ring_selection["valid"] = True
                ring_selection["quality"] = ring_route_result["quality"]
                ring_selection["lateral_error"] = ring_route_result[
                    "lateral_error"]
                ring_selection["reason"] = ring_route_result["reason"]
                ring_selection["source"] = ring_source
                ring_selection["rejected_source"] = None
                ring_selection["boundary_source"] = ring_route_result[
                    "boundary_source"]
                ring_selection["inferred"] = True
                ring_selection["inference_flags"] = ring_route_result[
                    "inference_flags"]
                selection = ring_selection
                reference = _selected_reference(selection)
                element_route_selected = reference is not None
                boundary_reference = {
                    "valid": reference is not None,
                    "reason": "RING_CURRENT_EDGE_AND_CACHED_WIDTH",
                    "source": ring_source,
                }

        if observe_element_route_outcome is not None:
            recovered_ordinary = observe_element_route_outcome(
                profile_now_ms, element_route_selected, ordinary_route_valid)
            if recovered_ordinary:
                # The tracker requires three independent current-frame
                # counter-observations; restore only this validated frame.
                selection = ordinary_selection
                reference = ordinary_reference
                boundary_reference = ordinary_boundary_reference
                element_route_selected = False

        ring_confirmed_state = element_result.get("state_id") in (
            track_elements.STATE_INSIDE,
            track_elements.STATE_EXIT_CONFIRM,
            track_elements.STATE_EXIT_REPAIR)
        ring_target_blocked = bool(
            ring_profile_enabled and
            element_result.get("ring_confirmed") and
            ring_confirmed_state and not element_route_selected)
        if ring_target_blocked:
            selection["rejected_source"] = selection.get("source", "NONE")
            selection["valid"] = False
            selection["quality"] = 0
            selection["lateral_error"] = 0
            selection["reason"] = "RING_TARGET_UNSUPPORTED"
            selection["source"] = "NONE"
            selection["result"] = None
            reference = None

        if (not ring_target_blocked and
                selection.get("source") not in ("ring_left", "ring_right") and
                not selection.get("valid") and mask_info is not None and
                element_result.get("cross_route_usable")):
            cross_count = element_result.get("cross_route_count", 0)
            cross_rows = element_result.get("cross_route_rows")
            if (isinstance(cross_rows, array) and
                    isinstance(cross_count, int) and
                    not isinstance(cross_count, bool) and
                    0 < cross_count <= config.SURFACE_GRID_HEIGHT and
                    element_result.get("cross_route_stride") == 4 and
                    len(cross_rows) >= cross_count * 4):
                center2 = element_result.get(
                    "cross_reference_center2", 0)
                ref_y = element_result.get("cross_reference_y", -1)
                cross_route_result["valid"] = True
                cross_route_result["quality"] = min(
                    100, 40 + 2 * (element_result.get("cross_near_rows", 0) +
                                   element_result.get("cross_far_rows", 0)))
                cross_route_result["reason"] = "CROSS_STRAIGHT_CONFIRMED"
                cross_route_result["lateral_error"] = element_result.get(
                    "cross_lateral_error", 0)
                cross_route_result["selected_row_ints"] = cross_rows
                cross_route_result["selected_row_count"] = cross_count
                cross_route_result["selected_row_stride"] = 4
                cross_route_result["reference_center_x"] = center2 * 0.5
                cross_route_result["reference_y"] = float(ref_y)
                cross_route_result["boundary_evidence"] = None
                cross_route_result["topology_evidence"] = topology
                try:
                    boundary_validator.gate_fast_path_candidate(
                        cross_route_result, mask_info.get("luma"),
                        mask_info.get("edge_mask"),
                        mask_info.get("road_mask"),
                        mask_info.get("known_region"), config)
                except Exception as exc:
                    cross_route_result["valid"] = False
                    cross_route_result["reason"] = "CROSS_BOUNDARY_EXCEPTION"
                    detector_error = "CROSS_BOUNDARY:%s" % exc
                if cross_route_result.get("valid"):
                    cross_selection["valid"] = True
                    cross_selection["quality"] = cross_route_result[
                        "quality"]
                    cross_selection["lateral_error"] = cross_route_result[
                        "lateral_error"]
                    cross_selection["reason"] = "CROSS_STRAIGHT_CONFIRMED"
                    cross_selection["source"] = "road"
                    cross_selection["rejected_source"] = None
                    cross_selection["boundary_source"] = "CROSS_STRAIGHT"
                    selection, reference, boundary_reference = \
                        _validate_selected_fast_boundary(
                            cross_selection, cross_route_result,
                            boundary_validator)
                    if selection.get("valid"):
                        control_road_result = cross_route_result
                        element_route_selected = True
                    else:
                        cross_selection["reason"] = (
                            "CROSS_REFERENCE_%s" %
                            boundary_reference.get("reason", "REJECTED"))
                else:
                    cross_selection["valid"] = False
                    cross_selection["reason"] = cross_route_result.get(
                        "reason", "CROSS_BOUNDARY_REJECTED")

        # The candidate was computed from the ordinary route before element
        # routing. It may support current-frame diagnosis, but only a final
        # ordinary bilateral route can seed the persistent measured-width
        # cache after it is accepted and fully written.
        if element_route_selected or ring_target_blocked:
            normal_cache_commit_allowed = False

        track_profile_label = (
            _TRACK_PROFILE_LABELS[profile_id]
            if 0 <= profile_id < len(_TRACK_PROFILE_LABELS) else "UNK")
        selection["track_profile_label"] = track_profile_label
        selection["track_ring_state"] = element_result.get("state", "IDLE")
        selection["track_ring_reason"] = element_result.get(
            "reason", "NOT_RUN")

        row_weight_start_us = time.ticks_us()
        weight_info = row_weight_workspace.aggregate(
            selection, control_road_result, config)
        row_weight_end_us = time.ticks_us()
        raw_valid = bool(selection.get("valid"))
        raw_error = 0
        quality = selection.get("quality", 0) if raw_valid else 0
        if raw_valid:
            if reference is None:
                raw_valid = False
                selection["reason"] = "REFERENCE_UNAVAILABLE"
            elif not weight_info.get("valid"):
                raw_valid = False
                selection["reason"] = "ROW_WEIGHT_%s" % weight_info.get(
                    "reason", "INVALID")
            else:
                try:
                    # This is the same-frame row aggregate; `reference`
                    # remains the independently validated geometric safety
                    # reference used by source-switch and boundary checks.
                    raw_error = weight_info["position_error"]
                    selection["lateral_error"] = raw_error
                except (KeyError, TypeError, ValueError, OverflowError):
                    raw_valid = False
                    selection["reason"] = "ROW_WEIGHT_UNAVAILABLE"
                if raw_valid and (raw_error < -1000 or raw_error > 1000):
                    raw_valid = False
                    selection["reason"] = "ERROR_OUT_OF_RANGE"

        # Slope classification is intentionally disabled. Keep the protocol
        # encoder's optional bit for compatibility, but never raise it here.
        slope_hint = 0
        ring_us = ring_detection_us

        filter_reset_reason = "none"
        if raw_valid:
            source_now = selection.get("source")
            kind_now = reference["kind"]
            reference_x_now = reference.get("x")
            # EMA identity follows the same current supported weighted row
            # used by the controller, not the independent safety marker.
            reference_y_now = weight_info.get("reference_row_y")
            if (reference_y_now is None and source_now in
                    ("ring_left", "ring_right")):
                reference_y_now = reference.get("y")
            source_or_kind_changed = (previous_source is not None and
                                      (source_now != previous_source or
                                       kind_now != previous_reference_kind))
            if source_or_kind_changed and _same_fast_track_reference(
                    previous_source, source_now, previous_reference_x,
                    reference_x_now, previous_reference_y,
                    reference_y_now):
                # OBSERVED_BOTH/LEFT_ONLY/RIGHT_ONLY are observation modes
                # for one cache-anchored route, so keep scalar EMA history.
                filter_reset_reason = "SOURCE_MODE_SWITCH_RETAINED"
            elif source_or_kind_changed:
                observation_filter.reset()
                filter_reset_reason = "SOURCE_OR_REFERENCE_KIND"
            elif (previous_reference_y is not None and
                  reference_y_now is not None and
                  reference_y_now != previous_reference_y):
                observation_filter.reset()
                filter_reset_reason = "REFERENCE_ROW_CHANGED"
        else:
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None

        prewrite_us = time.ticks_diff(time.ticks_us(), cycle_start_us)
        safe_budget_us = (config.SURFACE_FRAME_BUDGET_MS * 1000 -
                          config.V7_UART_WRITE_RESERVE_US)
        late_invalid = False
        if prewrite_us > safe_budget_us:
            late_invalid = raw_valid
            raw_valid = False
            raw_error = 0
            quality = 0
            single_edge_tracker.clear("FRAME_BUDGET")
            element_tracker.reset("FRAME_BUDGET")
            observation_filter.reset()
            position_pd.reset("FRAME_BUDGET")
            adaptive_preview.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            selection["reason"] = "FRAME_BUDGET"
            selection["rejected_source"] = selection.get("source", "NONE")
            selection["source"] = "NONE"
            selection["valid"] = False
            selection["quality"] = 0
            selection["lateral_error"] = 0
            selection["result"] = None
            reference = None
            filter_reset_reason = "FRAME_BUDGET"

        now_ms = capture_end_ms
        filtered_error = observation_filter.update(
            now_ms, raw_valid, quality if raw_valid else 0,
            raw_error if raw_valid else 0, time.ticks_diff)
        valid = raw_valid
        if not valid:
            raw_error = 0
            filtered_error = 0
        control_source = selection.get("source", "NONE") if valid else "NONE"
        control_boundary_source = selection.get(
            "boundary_source", "OBSERVED_BOTH" if control_source == "road"
            else "NONE")
        control_reference_kind = (reference.get("kind")
                                  if reference is not None else "NONE")
        # The weighted supported row is the controller identity. The
        # geometric safety reference remains independent of this value.
        control_reference_y = weight_info.get("reference_row_y")
        if (control_reference_y is None and reference is not None and
                control_source in ("ring_left", "ring_right")):
            control_reference_y = reference.get("y")
        control_info = position_pd.update(
            capture_end_ms, capture_dt_ms, valid, filtered_error,
            control_source, control_boundary_source,
            control_reference_kind, control_reference_y,
            control_profile, time.ticks_diff)
        preview_candidate = weight_info.get("preview_candidate", 0)
        preview_geometry_valid = bool(weight_info.get("preview_valid"))
        preview_eligible_source = control_source in (
            "road", "single_left", "single_right")
        preview_enabled = bool(control_info.get("preview_enabled") and
                               preview_eligible_source)
        preview_applied = adaptive_preview.update(
            capture_end_ms, capture_dt_ms, valid, preview_enabled,
            control_source, control_boundary_source,
            control_reference_kind, control_reference_y,
            preview_geometry_valid, preview_candidate, filtered_error,
            time.ticks_diff, control_info)
        if config.SPEED_STAGE_ENABLED:
            curve_phase, slowdown_request_permille = speed_stage.update(
                capture_end_ms, valid, filtered_error,
                preview_candidate,
                bool(valid and weight_info.get("valid")),
                control_info.get("preview_state", 0), time.ticks_diff,
                frame_count & 0xFFFF)
        else:
            curve_phase = 0
            slowdown_request_permille = 0
        if (valid and element_route_selected and
                control_source in ("ring_left", "ring_right")):
            # H743 maps the full vision slowdown request to the selected
            # profile's outer-wheel ring target (B 4.0, C 4.4 rps).
            curve_phase = vision_protocol.CURVE_PHASE_TURN
            slowdown_request_permille = 1000
        elif (element_route_selected and control_source == "road" and
                control_boundary_source == "CROSS_STRAIGHT" and
                element_result.get("cross_suppress_slowdown")):
            # Only a fully confirmed and output-eligible straight cross route
            # suppresses the vision-side extra slowdown request.
            curve_phase = 0
            slowdown_request_permille = 0
        final_control = control_info.get("pd_output", 0) + preview_applied
        if final_control < -config.STEERING_OUTPUT_LIMIT:
            final_control = -config.STEERING_OUTPUT_LIMIT
        elif final_control > config.STEERING_OUTPUT_LIMIT:
            final_control = config.STEERING_OUTPUT_LIMIT
        if not valid:
            final_control = 0
        weight_info["filtered_position"] = filtered_error
        weight_info["pd_error"] = final_control
        weight_info["heading_contribution"] = preview_applied
        weight_info["preview_gain_milli"] = control_info.get(
            "preview_gain_milli", 0)
        weight_info["preview_applied"] = preview_applied
        weight_info["derivative_d"] = control_info.get("derivative_d", 0)
        weight_info["position_p"] = control_info.get("position_p", 0)
        weight_info["preview_state"] = control_info.get("preview_state", 0)
        preview_state_index = min(3, max(0, int(
            control_info.get("preview_state", 0))))
        weight_info["preview_state_name"] = (
            "STRAIGHT", "CANDIDATE", "CONFIRMED", "EXIT")[
                preview_state_index]
        weight_info["profile_short"] = {
            "no_pd": "NO_PD", "pd": "PD", "pd_preview": "PD+PREV",
            "geometry_perf_only": "GEOM PERF", "coordinated": "COORD",
        }.get(control_profile, "PD")
        weight_info["steering_error"] = final_control
        control_info["preview_applied"] = preview_applied
        control_info["preview_candidate"] = preview_candidate
        control_info["preview_geometry_valid"] = preview_geometry_valid
        control_info["preview_enabled"] = preview_enabled
        if not valid:
            raw_error = 0
            filtered_error = 0
            control_info["position_p"] = 0
            control_info["pd_output"] = 0
            control_info["derivative_d"] = 0
            control_info["preview_applied"] = 0
            weight_info["filtered_position"] = 0
            weight_info["pd_error"] = 0
            weight_info["steering_error"] = 0
            weight_info["heading_contribution"] = 0
        else:
            previous_source = selection.get("source")
            previous_reference_kind = reference["kind"]
            previous_reference_y = control_reference_y
            previous_reference_x = reference.get("x")
        reference_end_us = time.ticks_us()

        pack_start_us = reference_end_us
        write_attempts = 0
        full_write = False
        write_error = None
        profile_check_ms = time.ticks_ms()
        reverse_parser.poll(profile_check_ms, time.ticks_diff)
        profile_ack = reverse_parser.profile_ack(
            profile_check_ms, time.ticks_diff)
        ring_calibration_gate_blocked = _ring_calibration_gate_blocked(
            profile_id, ring_calibration_passed)
        if ring_calibration_gate_blocked:
            profile_ack = config.VEHICLE_PROFILE_UNKNOWN
            if valid:
                selection["rejected_source"] = selection.get("source", "NONE")
                selection["valid"] = False
                selection["quality"] = 0
                selection["lateral_error"] = 0
                selection["reason"] = "RING_CALIBRATION_REQUIRED"
                selection["source"] = "NONE"
                selection["result"] = None
                reference = None
            valid = False
            raw_valid = False
            raw_error = 0
            filtered_error = 0
            quality = 0
            final_control = 0
            preview_applied = 0
            curve_phase = 0
            slowdown_request_permille = 0
            observation_filter.reset()
            position_pd.reset("RING_CALIBRATION_GATE")
            adaptive_preview.reset()
            speed_stage.lifecycle_event("RESET")
            control_info["position_p"] = 0
            control_info["derivative_d"] = 0
            control_info["pd_output"] = 0
            control_info["preview_applied"] = 0
            control_info["preview_state"] = 0
            control_info["reset_code"] = _CONTROL_RESET_CODES[
                "DISABLED"]
            weight_info["position_p"] = 0
            weight_info["derivative_d"] = 0
            weight_info["preview_applied"] = 0
            weight_info["pd_error"] = 0
            weight_info["steering_error"] = 0
            weight_info["heading_contribution"] = 0
        if (element_route_selected and
                (not reverse_parser.single_enabled or
                 not reverse_parser.profile_session_active or
                 profile_ack != profile_id or
                 reverse_parser.session_token !=
                 element_result.get("session_token"))):
            # A status/session lease that expires while a candidate is being
            # processed cannot authorize one final element-derived command.
            valid = False
            raw_valid = False
            raw_error = 0
            quality = 0
            final_control = 0
            preview_applied = 0
            curve_phase = 0
            slowdown_request_permille = 0
            selection["valid"] = False
            selection["quality"] = 0
            selection["lateral_error"] = 0
            selection["reason"] = "ELEMENT_SESSION_LEASE_EXPIRED"
            selection["rejected_source"] = selection.get("source", "NONE")
            selection["source"] = "NONE"
            selection["result"] = None
            reference = None
            element_tracker.reset("ELEMENT_SESSION_LEASE_EXPIRED")
            single_edge_tracker.clear("ELEMENT_SESSION_LEASE_EXPIRED")
            observation_filter.reset()
            position_pd.reset("PROFILE_TIMEOUT")
            adaptive_preview.reset()
            speed_stage.lifecycle_event("RESET")
            control_info["position_p"] = 0
            control_info["derivative_d"] = 0
            control_info["pd_output"] = 0
            control_info["preview_applied"] = 0
            control_info["preview_state"] = 0
            control_info["reset_code"] = _CONTROL_RESET_CODES[
                "PROFILE_TIMEOUT"]
            weight_info["position_p"] = 0
            weight_info["derivative_d"] = 0
            weight_info["preview_applied"] = 0
            weight_info["pd_error"] = 0
            weight_info["steering_error"] = 0
            reverse_fresh = bool(reverse_parser.single_enabled)
        try:
            vision_protocol.pack_into_v3(
                packet, sequence, 1 if valid else 0,
                quality if valid else 0,
                final_control if valid else 0,
                0, curve_phase if valid else 0,
                slowdown_request_permille if valid else 0,
                profile_ack)
            packet_ready_us = time.ticks_us()
            packet_ready_age_us = time.ticks_diff(
                packet_ready_us, cycle_start_us)
            if valid and not _v7_valid_within_budget(
                    True, packet_ready_age_us,
                    config.SURFACE_FRAME_BUDGET_MS * 1000,
                    config.V7_UART_WRITE_RESERVE_US):
                # If packing consumed the remaining deadline margin, replace
                # this same sequence with an explicit invalid before writing.
                late_invalid = True
                valid = False
                raw_valid = False
                raw_error = 0
                filtered_error = 0
                quality = 0
                single_edge_tracker.clear("FRAME_BUDGET")
                element_tracker.reset("FRAME_BUDGET")
                observation_filter.reset()
                previous_source = None
                previous_reference_kind = None
                previous_reference_y = None
                previous_reference_x = None
                selection["valid"] = False
                selection["quality"] = 0
                selection["lateral_error"] = 0
                selection["rejected_source"] = selection.get("source", "NONE")
                selection["source"] = "NONE"
                selection["result"] = None
                selection["reason"] = "FRAME_BUDGET"
                reference = None
                filter_reset_reason = "FRAME_BUDGET_PACKET_READY"
                position_pd.reset("FRAME_BUDGET")
                adaptive_preview.reset()
                final_control = 0
                control_info["position_p"] = 0
                control_info["derivative_d"] = 0
                control_info["pd_output"] = 0
                control_info["preview_applied"] = 0
                control_info["preview_gain_milli"] = 0
                control_info["preview_confirmed"] = False
                control_info["preview_state"] = 0
                control_info["reset_code"] = _CONTROL_RESET_CODES[
                    "FRAME_BUDGET"]
                curve_phase = 0
                slowdown_request_permille = 0
                weight_info["position_p"] = 0
                weight_info["derivative_d"] = 0
                weight_info["preview_applied"] = 0
                weight_info["preview_gain_milli"] = 0
                weight_info["preview_state"] = 0
                weight_info["preview_state_name"] = "STRAIGHT"
                weight_info["pd_error"] = 0
                weight_info["steering_error"] = 0
                weight_info["heading_contribution"] = 0
                vision_protocol.pack_into_v3(
                    packet, sequence, 0, 0, 0, 0, 0, 0, profile_ack)
            if len(packet) < vision_protocol.FRAME_SIZE:
                raise ValueError("vision_protocol returned a short V3 frame")
            write_attempts = 1
            written = uart.write(packet)
            if written == len(packet):
                full_write = True
                last_tx_result = ("QUEUED_16" if
                                  uart_tx_mode == "FIFO_QUEUED" else
                                  "FULL_16")
            else:
                write_error = "SHORT_WRITE:%s/16" % str(written)
                last_tx_result = write_error
        except Exception as exc:
            write_error = "%s:%s" % (type(exc).__name__, exc)
            last_tx_result = "ERROR"
        # Preserve the primary V3 completion timestamp. D1 work is sideband
        # and total_cycle accounts for it separately after this point.
        send_end_us = time.ticks_us()
        send_end_ms = time.ticks_ms()
        pack_uart_us = time.ticks_diff(send_end_us, pack_start_us)
        capture_to_send_us = time.ticks_diff(
            send_end_us, cycle_start_us)
        # STEERING_DIAG_TRACE_BEGIN
        diag_trace_us = 0
        if _STEERING_DIAG is not None and write_attempts:
            diag_trace_start_us = time.ticks_us()
            try:
                diag_source = selection.get("source", "NONE")
                if diag_source == "NONE":
                    diag_source = selection.get("rejected_source", "NONE")
                diag_source_code = _steering_diag_source_code(diag_source)
                diag_row_count = 0
                if diag_source == "road":
                    diag_row_count = road_result.get(
                        "selected_row_count", 0)
                elif diag_source in ("single_left", "single_right"):
                    diag_result = selection.get("result")
                    if diag_result is None:
                        diag_result = single_candidate
                    if diag_result is not None:
                        diag_row_count = diag_result.get(
                            "selected_row_count", 0)
                elif (diag_source == "ring_left" or
                      diag_source == "ring_right") and \
                        ring_candidate is not None:
                    diag_result = selection.get("result")
                    diag_row_count = (diag_result.get(
                        "selected_row_count", 0) if diag_result else 0)
                diag_reference_error = 0
                if reference is not None:
                    diag_reference_error = weight_info.get(
                        "reference_error")
                    if diag_reference_error is None:
                        diag_reference_error = 0
                diag_was_frozen = _STEERING_DIAG.frozen
                candidate_diagnostic = getattr(
                    single_edge_tracker, "pair_diagnostic_record", None)
                _STEERING_DIAG.record(
                    packet, full_write, write_error is not None,
                    reverse_fresh, capture_end_ms, send_end_ms,
                    weight_info.get("position_error", 0),
                    weight_info.get("preview_candidate", 0),
                    weight_info.get("heading_contribution", 0),
                    raw_error, filtered_error, diag_reference_error,
                    diag_row_count, diag_source_code, reverse_generation,
                    candidate_diagnostic, control_info)
                if not diag_was_frozen and _STEERING_DIAG.frozen:
                    single_edge_tracker.freeze_diagnostic_snapshot()
            except Exception:
                # Diagnostics must never interrupt frame processing or UART.
                try:
                    _STEERING_DIAG.record_errors += 1
                except Exception:
                    pass
            diag_trace_us = time.ticks_diff(
                time.ticks_us(), diag_trace_start_us)
        # STEERING_DIAG_TRACE_END
        ring_diag_us = 0
        if _RING_DIAG_SESSION is not None:
            # Pair geometry with the actual signed value written in this V3,
            # not an earlier candidate that may have been rejected afterward.
            element_result["diagnostic_position_error"] = int(
                weight_info.get("position_error", 0))
            sent_error = packet[8] | (packet[9] << 8)
            if sent_error >= 32768:
                sent_error -= 65536
            element_result["diagnostic_sent_error"] = sent_error
            ring_diag_start_us = time.ticks_us()
            active_follow = bool(
                reverse_fresh and reverse_parser.single_enabled and
                profile_id in (config.VEHICLE_PROFILE_A,
                               config.VEHICLE_PROFILE_B,
                               config.VEHICLE_PROFILE_C) and
                reverse_parser.profile_session_active and
                reverse_parser.session_token != 0)
            _RING_DIAG_SESSION.observe(
                capture_end_ms, sequence, profile_id, active_follow,
                bool(valid and full_write), selection, element_result,
                ring_stop_reason, time.ticks_diff)
            ring_diag_us = time.ticks_diff(
                time.ticks_us(), ring_diag_start_us)
        sequence_sent = sequence
        sequence = (sequence + 1) & 0xFFFF
        if write_error is not None:
            tx_error_count += 1
            reverse_parser.invalidate("OBSERVATION_UART_ERROR")
            reverse_generation = reverse_parser.generation
            reverse_drain_state[0] = 1
            single_edge_tracker.clear("OBSERVATION_UART_ERROR")
            element_tracker.reset("OBSERVATION_UART_ERROR")
            observation_filter.reset()
            position_pd.reset("UART_ERROR")
            adaptive_preview.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
        elif full_write and valid:
            if (normal_cache_commit_allowed and reverse_fresh and
                    bilateral_cache_candidate is not None and
                    selection.get("valid") and
                    selection.get("source") == "road" and
                    selection.get("result") is road_result and
                    control_road_result is road_result and
                    selection.get("boundary_source") == "OBSERVED_BOTH"):
                single_edge_tracker.observe_bilateral(
                    bilateral_cache_candidate, mask_info.get("luma"),
                    mask_info.get("edge_mask"), mask_info.get("road_mask"),
                    mask_info.get("known_region"), True, True)
            elif (selection.get("source") in ("single_left", "single_right") and
                  single_candidate is not None and
                  single_candidate.get("valid")):
                single_edge_tracker.commit_single(single_candidate)

        heap_sample_us = 0
        heap_sample_now_ms = time.ticks_ms()
        if (last_heap_sample_ms is None or
                time.ticks_diff(heap_sample_now_ms,
                                last_heap_sample_ms) >= 1000):
            heap_sample_start_us = time.ticks_us()
            last_heap_sample_ms = heap_sample_now_ms
            gc_cached_free_bytes = -1
            gc_heap_sample_ok = False
            if callable(heap_stats_fn):
                try:
                    heap_values = heap_stats_fn()
                    if (isinstance(heap_values, tuple) and
                            len(heap_values) == 2 and
                            isinstance(heap_values[0], int) and
                            not isinstance(heap_values[0], bool) and
                            isinstance(heap_values[1], int) and
                            not isinstance(heap_values[1], bool) and
                            heap_values[0] >= 0 and heap_values[1] >= 0):
                        gc_cached_free_bytes = int(heap_values[0])
                        gc_alloc_bytes = int(heap_values[1])
                        gc_heap_sample_ok = True
                except Exception:
                    gc_heap_sample_ok = False
            if not gc_heap_sample_ok:
                # Keep the original two-call firmware path for absent,
                # incompatible, malformed, or failing optional extensions.
                try:
                    heap_free_fn = getattr(gc, "mem_free", None)
                    heap_alloc_fn = getattr(gc, "mem_alloc", None)
                    if callable(heap_free_fn):
                        gc_cached_free_bytes = int(heap_free_fn())
                        gc_heap_sample_ok = True
                    if gc_heap_sample_ok and callable(heap_alloc_fn):
                        gc_alloc_bytes = int(heap_alloc_fn())
                    else:
                        gc_heap_sample_ok = False
                except Exception:
                    gc_heap_sample_ok = False
            if gc_heap_sample_ok:
                perf.record_heap(gc_cached_free_bytes, gc_alloc_bytes)
            heap_sample_us = time.ticks_diff(
                time.ticks_us(), heap_sample_start_us)

        window_start_us, windows_advanced = _advance_v7_perf_window(
            send_end_us, window_start_us, window_us)
        summary_log_us = 0
        while windows_advanced:
            summary_start_us = time.ticks_us()
            _print_v7_summary(
                perf, run_id, current_window, perf.frames, summary_queue)
            summary_log_us += time.ticks_diff(time.ticks_us(), summary_start_us)
            current_window += 1
            perf.clear()
            windows_advanced -= 1
        if summary_queue.pending:
            summary_start_us = time.ticks_us()
            print(summary_queue.pop())
            summary_log_us += time.ticks_diff(
                time.ticks_us(), summary_start_us)

        perf.frames += 1
        perf.write_attempts += write_attempts
        if valid:
            perf.valid_frames += 1
        if late_invalid:
            perf.late_invalid_frames += 1
        if full_write:
            perf.full_writes += 1
            if valid:
                perf.valid_full_writes += 1
        if write_error is not None:
            perf.write_errors += 1
        perf.last_sequence = sequence_sent

        if detector_error is not None:
            detector_error_count += 1
            latest_detector_error = _short_v7_detector_error(detector_error)

        contour_selected = bool(
            contour_used and valid and selection.get("valid") and
            selection.get("source") == "road" and
            selection.get("result") is road_result and
            contour_candidate is road_result)

        render_profile_id = reverse_parser.profile_ack(
            time.ticks_ms(), time.ticks_diff)
        selection["track_profile_label"] = (
            _TRACK_PROFILE_LABELS[render_profile_id]
            if 0 <= render_profile_id < len(_TRACK_PROFILE_LABELS) else "UNK")
        selection["track_ring_state"] = ring_info.get("state", "IDLE")
        selection["track_ring_reason"] = ring_info.get("reason", "NOT_RUN")

        if not native_preview_api:
            usb_connected = _usb_vcp_is_connected(usb_vcp)
            fixture_without_usb_vcp = bool(
                fixture_mode and not fixture_usb_vcp_provided)
            fallback_draw_enabled = bool(
                config.USB_PREVIEW_ENABLED and
                (fixture_without_usb_vcp or usb_connected))
            _render_v7_preview_frame(
                img, selection, no_surface_result, control_road_result,
                reference,
                valid, final_control if valid else 0, filtered_error,
                capture_to_send_us // 1000, capture_fps_value, overlay_state,
                time.ticks_ms(), weight_info=weight_info,
                contour_trace_info=(contour_trace_info
                                    if contour_selected else None),
                contour_used=contour_selected, flush_func=preview_flush,
                draw_enabled=fallback_draw_enabled,
                preview_sequence=preview_frame_id)
            overlay_us = overlay_state.draw_us
            preview_flush_us = overlay_state.flush_us
            if fixture_without_usb_vcp:
                current_usb_preview_state = "RAM_FIXTURE"
            elif not config.USB_PREVIEW_ENABLED:
                current_usb_preview_state = "LEGACY_DISABLED"
            elif usb_connected:
                current_usb_preview_state = "LEGACY_ACTIVE"
            else:
                current_usb_preview_state = "LEGACY_HEADLESS"
        elif not config.USB_PREVIEW_ENABLED:
            # Keep capture, detection, tracing and UART; suppress the optional
            # preview-side render and JPEG work.
            overlay_state.draw_us = 0
            overlay_state.flush_us = 0
            overlay_us = 0
            preview_flush_us = 0
            current_usb_preview_state = "NATIVE_DISABLED"
            preview_source.last_state = current_usb_preview_state
            preview_source.clear()
        else:
            # Use the USB state latched by snapshot() for rendering. A cable
            # that connects mid-frame waits for the next captured image.
            usb_connected = _usb_vcp_is_connected(usb_vcp)
            frame_preview_active = bool(
                preview_source.frame_preview_active and usb_connected)
            if frame_preview_active:
                _render_v7_preview_frame(
                    img, selection, no_surface_result, control_road_result,
                    reference, valid, final_control if valid else 0,
                    filtered_error, capture_to_send_us // 1000,
                    capture_fps_value, overlay_state, time.ticks_ms(),
                    weight_info=weight_info,
                    contour_trace_info=(contour_trace_info
                                        if contour_selected else None),
                    contour_used=contour_selected,
                    flush_func=preview_source.flush,
                    preview_sequence=preview_frame_id)
                overlay_us = overlay_state.draw_us
                preview_flush_us = overlay_state.flush_us
                current_usb_preview_state = preview_source.last_state
            else:
                # Clear the captured image without rendering or encoding it.
                overlay_state.draw_us = 0
                overlay_state.flush_us = 0
                overlay_us = 0
                preview_flush_us = 0
                current_usb_preview_state = "NATIVE_HEADLESS"
                preview_source.last_state = current_usb_preview_state
                preview_source.clear()
        if current_usb_preview_state != last_usb_preview_state:
            print("USB_PREVIEW_STATE build=%s quality=Q%d mode=%s" % (
                config.USB_PREVIEW_BUILD_ID, config.USB_PREVIEW_QUALITY,
                current_usb_preview_state))
            last_usb_preview_state = current_usb_preview_state
        if (native_preview_api and
                current_usb_preview_state == "NATIVE_ERROR"):
            preview_error_now_ms = time.ticks_ms()
            if (last_preview_error_ms is None or
                    time.ticks_diff(preview_error_now_ms,
                                    last_preview_error_ms) >= 1000):
                print("USB_PREVIEW_ERROR build=%s failures=%d" % (
                    config.USB_PREVIEW_BUILD_ID,
                    preview_source.publish_failures))
                last_preview_error_ms = preview_error_now_ms

        debug_us = 0
        debug_now_ms = time.ticks_ms()
        if config.DEBUG_ENABLED and time.ticks_diff(
                debug_now_ms, last_debug_ms) >= config.DEBUG_PERIOD_MS:
            debug_start_us = time.ticks_us()
            diagnostic_profile_id = reverse_parser.profile_ack(
                debug_now_ms, time.ticks_diff)
            diagnostic_profile_label = (
                _TRACK_PROFILE_LABELS[diagnostic_profile_id]
                if 0 <= diagnostic_profile_id <
                len(_TRACK_PROFILE_LABELS) else "UNK")
            diagnostic_ring_enabled = bool(
                reverse_parser.ring_profile_enabled(
                    debug_now_ms, time.ticks_diff))
            diagnostic_session_active = bool(
                diagnostic_profile_id != config.VEHICLE_PROFILE_UNKNOWN and
                reverse_parser.profile_session_active and
                reverse_parser.session_token != 0)
            topology = road_result.get("topology_evidence") or {}
            boundary = road_result.get("boundary_evidence") or {}
            trusted_blocks = (mask_info.get("trusted_block_count", 0)
                              if mask_info is not None else 0)
            threshold_min = (mask_info.get("threshold_min")
                             if mask_info is not None else None)
            threshold_max = (mask_info.get("threshold_max")
                             if mask_info is not None else None)
            diagnostic_result = single_candidate
            if diagnostic_result is None:
                diagnostic_result = selection.get("single_edge_result")
            if (diagnostic_result is None and selection.get("source") in
                    ("single_left", "single_right")):
                diagnostic_result = selection.get("result")
            single_edge_diag = _format_single_edge_diagnostic(
                config.SINGLE_EDGE_ROUTE_MODE,
                selection.get("single_edge_reason",
                              single_diagnostic_reason),
                diagnostic_result)
            actual_left, actual_right, actual_reference = \
                _format_selected_actual_diagnostic(
                    selection, control_road_result, reference, valid)
            reference_row_y = weight_info.get("reference_row_y")
            reference_error = weight_info.get("reference_error")
            reference_row_text = ("-" if reference_row_y is None else
                                  str(reference_row_y))
            reference_error_text = ("-" if reference_error is None else
                                    "%+d" % reference_error)
            print("V7DBG run=%d seq=%d valid=%d q=%d source=%s "
                  "b=%s inferred=%d flags=%d "
                  "%s cache_rows=%d actual=%s/%s "
                  "safe_ref=%s wctrl=%s rows_nf=%d/%d eff_nf=%d/%d "
                  "reverse=%s/%s why=%s road_why=%s rows=%d scene=%s topology=%s "
                  "track=%s ack=%d enabled=%d follow=%d token=%08x "
                  "ring=%s:%s candidate=%d confirmed=%d side=%d rows=%d "
                  "cross=%s/%d usable=%d "
                  "slope=%s:%d "
                  "preview_mode=%s preview=%+d preview_candidate=%+d bend=%.2f "
                  "preview_block=- "
                  "contour=%d/%d:%s:%d/%d "
                  "rowref=%s reference_error=%s "
                  "boundary=%s:%d/%d ref=%s pos_weighted=%d raw_pos=%d "
                  "pos_ema=%d tx_cmd=%d "
                  "filter=%s "
                  "age_send_ms=%d trusted_block_count=%d "
                  "threshold_min=%s threshold_max=%s "
                  "tx=%s txerr=%d deterr=%d derr=%s fps=%.1f" % (
                      run_id, sequence_sent, 1 if valid else 0,
                      quality if valid else 0,
                      selection.get("source", "NONE"),
                      selection.get("boundary_source", "OBSERVED_BOTH"
                                    if selection.get("source") == "road"
                                    else "NONE"),
                      1 if selection.get("inferred") else 0,
                      selection.get("inference_flags", 0),
                      single_edge_diag,
                      single_edge_tracker.cache_rows,
                      actual_left, actual_right, actual_reference,
                      ("%.1f" % weight_info["center_x"]
                       if valid and weight_info.get("center_x") is not None
                       else "NONE"),
                      weight_info.get("near_count", 0),
                      weight_info.get("far_count", 0),
                      int(weight_info.get("near_weight", 0.0) * 100 + 0.5),
                      int(weight_info.get("far_weight", 0.0) * 100 + 0.5),
                      reverse_parser.last_event,
                      "FRESH" if reverse_parser.single_enabled else "STALE",
                      selection.get("reason", "INVALID"),
                      road_result.get("reason", "INVALID"),
                      road_result.get("active_row_count",
                                      road_result.get("observed_rows", 0)),
                      road_result.get("scene", "UNKNOWN"),
                      topology.get("reason", "NOT_RUN"),
                      diagnostic_profile_label, diagnostic_profile_id,
                      1 if diagnostic_ring_enabled else 0,
                      1 if diagnostic_session_active else 0,
                      reverse_parser.session_token & 0xFFFFFFFF,
                      ring_info.get("state", "IDLE"),
                      ring_info.get("reason", "NOT_RUN"),
                      1 if ring_info.get("candidate") else 0,
                      1 if ring_info.get("ring_confirmed") else 0,
                      ring_info.get("side", track_elements.SIDE_NONE),
                      ring_info.get("target_count", 0),
                      ring_info.get("cross_reason", "NOT_EVALUATED"),
                      ring_info.get("cross_route_count", 0),
                      1 if ring_info.get("cross_route_usable") else 0,
                      "DISABLED", slope_hint,
                       control_profile,
                      weight_info.get("heading_contribution", 0),
                      weight_info.get("preview_candidate", 0),
                      weight_info.get("preview_curvature_cells", 0.0),
                      1 if contour_attempted else 0,
                      1 if contour_selected else 0,
                      str(contour_reason)[:16],
                      contour_lcount, contour_rcount,
                      reference_row_text,
                      reference_error_text,
                      boundary.get("reason", "NOT_RUN"),
                      boundary.get("bilateral_supported", 0),
                      boundary.get("pairs_tested", 0),
                      "-" if reference is None else
                          "%.1f,%.1f" % (reference["x"], reference["y"]),
                      weight_info.get("position_error", 0) if valid else 0,
                      raw_error if valid else 0,
                      filtered_error if valid else 0,
                      final_control if valid else 0,
                      filter_reset_reason,
                      capture_to_send_us // 1000, trusted_blocks,
                      str(threshold_min), str(threshold_max), last_tx_result,
                      tx_error_count, detector_error_count,
                      latest_detector_error, capture_fps_value))
            print("V7CTRL seq=%d profile=%s p=%+d d_component=%+d pd=%+d tx_cmd=%+d "
                  "preview=%s:%+d candidate=%+d gain=%d dt=%d reset=%d" % (
                      sequence_sent, control_profile,
                      control_info.get("position_p", 0),
                      control_info.get("derivative_d", 0),
                      control_info.get("pd_output", 0),
                      final_control if valid else 0,
                      weight_info.get("preview_state_name", "STRAIGHT"),
                      weight_info.get("preview_applied", 0),
                      weight_info.get("preview_candidate", 0),
                      weight_info.get("preview_gain_milli", 0),
                      control_info.get("capture_dt_ms", 0xFFFF),
                      control_info.get("reset_code", 0)))
            ring_state = ring_info.get("state", "OBSERVE")
            ring_arc_rejection = ring_info.get("ring_arc_rejection", "NONE")
            ring_growth_rejected = bool(
                ring_info.get("growth_ready_but_target_rejected"))
            if (ring_state != "OBSERVE" or ring_arc_rejection != "NONE" or
                    ring_growth_rejected):
                print("V7RING seq=%d st=%s sd=%s arc=%d/%d near=%d/%d "
                      "gf=%d dx2=%+d path=%s tgt=%d rej=%s/%s "
                      "target_side=%s center2=%d cmd=%d source=%s admit=%s" % (
                          sequence_sent,
                          ring_state,
                          ring_info.get("side_name", "NONE"),
                          ring_info.get("ring_arc_support_rows", 0),
                          ring_info.get("ring_arc_span_rows", 0),
                          ring_info.get("ring_near_anchor_rows", 0),
                          ring_info.get("ring_near_anchor_run", 0),
                          ring_info.get("ring_arc_growth_frames", 0),
                          ring_info.get("ring_arc_growth_x2", 0),
                          ring_info.get("entry_confirmation_path", "NONE"),
                          1 if ring_info.get("target_valid") else 0,
                          ring_arc_rejection,
                          ring_info.get("growth_target_rejection", "NONE"),
                          ring_info.get("target_side_name", "NONE"),
                          int(ring_info.get("target_center2", -1))
                              if ring_info.get("target_valid") else -1,
                          final_control if valid else 0,
                          selection.get("source", "NONE"),
                          ring_info.get("route_admission_reason", "NOT_RUN")))
            if single_diagnostic_reason == "AMBIGUOUS_BOTH_SIDES":
                pair_record = getattr(
                    single_edge_tracker, "pair_diagnostic_record", None)
                if isinstance(pair_record, array) and len(pair_record) == 16:
                    pair_reason = pair_record[0]
                    pair_reason_names = getattr(
                        fast_single_edge, "_PAIR_REASON_NAMES", ())
                    pair_reason_name = "UNK"
                    if (isinstance(pair_reason_names, tuple) and
                            isinstance(pair_reason, int) and
                            0 <= pair_reason < len(pair_reason_names)):
                        pair_reason_name = pair_reason_names[pair_reason]
                    print("V7PAIR seq=%d rows=%d span=%d maxdx2=%d "
                          "left=%d right=%d reason=%d:%s" % (
                              sequence_sent, pair_record[1], pair_record[2],
                              pair_record[3], pair_record[6], pair_record[7],
                              pair_reason, pair_reason_name))
            debug_us = time.ticks_diff(time.ticks_us(), debug_start_us)
            last_debug_ms = debug_now_ms

        # The heap threshold is a sampled soft trigger. Automatic GC remains
        # enabled to handle allocations between the once-per-second samples.
        gc_low_memory_now = (
            gc_cached_free_bytes >= 0 and
            gc_cached_free_bytes < config.PERF_GC_MIN_FREE_BYTES)
        if not gc_low_memory_now:
            gc_low_memory_latched = False
        # Include the rare heap read/record work in the existing GC stage.
        gc_us = heap_sample_us
        frame_count += 1
        if (frame_count % gc_period_frames == 0 or
                (gc_low_memory_now and not gc_low_memory_latched)):
            gc_start_us = time.ticks_us()
            gc.collect()
            gc_us += time.ticks_diff(time.ticks_us(), gc_start_us)
            gc_low_memory_latched = gc_low_memory_now

        perf.record("capture", capture_us)
        perf.record("masks", time.ticks_diff(masks_end_us, masks_start_us))
        perf.record("path_topology", time.ticks_diff(path_end_us, path_start_us))
        perf.record("contour", contour_us)
        perf.record("boundary", time.ticks_diff(boundary_end_us, boundary_start_us))
        perf.record("ring", ring_us)
        perf.record("row_weight", time.ticks_diff(
            row_weight_end_us, row_weight_start_us))
        perf.record("reference_filter", time.ticks_diff(reference_end_us,
                                                          reference_start_us))
        perf.record("pack_uart", pack_uart_us)
        perf.record("capture_to_send", capture_to_send_us)
        perf.record("diag_trace", diag_trace_us)
        perf.record("ring_diag", ring_diag_us)
        perf.record("overlay", overlay_us)
        perf.record("preview_flush", preview_flush_us)
        perf.record("debug", debug_us)
        perf.record("summary_log", summary_log_us)
        perf.record("gc", gc_us)
        # The primary FIFO drains during existing overlay/debug/GC work.
        # Probe once here; never wait, replace a V3, or delay its completion.
        if (camera_diag_begin is not None and full_write and
                not late_invalid):
            diag_start_us = None
            try:
                diag_now_ms = time.ticks_ms()
                capability_fresh = bool(
                    callable(camera_diag_capability_check) and
                    camera_diag_capability_check(
                        diag_now_ms, time.ticks_diff))
                camera_diag_observe(capability_fresh)
                elapsed_us = time.ticks_diff(
                    time.ticks_us(), cycle_start_us)
                remaining_us = (
                    config.SURFACE_FRAME_BUDGET_MS * 1000 - elapsed_us)
                if camera_diag_begin(
                        uart, diag_now_ms, time.ticks_diff,
                        camera_diag_interval_ms,
                        capability_fresh, full_write, late_invalid,
                        remaining_us, uart_tx_mode):
                    diag_start_us = time.ticks_us()
                    raw_profile_ack = camera_diag_profile_ack(
                        diag_now_ms, time.ticks_diff)
                    profile_fresh = bool(
                        raw_profile_ack != config.VEHICLE_PROFILE_UNKNOWN and
                        callable(camera_diag_profile_fresh) and
                        camera_diag_profile_fresh(
                            diag_now_ms, time.ticks_diff))
                    reverse_keep_fresh = bool(
                        callable(camera_diag_control_fresh) and
                        camera_diag_control_fresh(
                            diag_now_ms, time.ticks_diff))
                    status_flags = (
                        (1 if element_tracker.ring_calibration_ready else 0) |
                        (2 if ring_calibration_passed else 0) |
                        (4 if reverse_keep_fresh else 0) |
                        (8 if profile_fresh else 0) |
                        (16 if _ring_calibration_gate_blocked(
                            raw_profile_ack, ring_calibration_passed) else 0) |
                        (32 if packet[6] & vision_protocol.FLAG_VALID else 0) |
                        (64 if uart_tx_mode == "FIFO_QUEUED" else 0))
                    calibration_code = camera_diag_calibration_code(
                        ring_calibration.reason, sensor_identity_valid)
                    observation_code = camera_diag_observation_code(
                        selection.get("reason"), detector_error,
                        bool(packet[6] & vision_protocol.FLAG_VALID),
                        selection.get("source", "NONE"))
                    reverse_code = camera_diag_reverse_code(
                        reverse_parser.last_event)
                    reverse_rejects = (
                        reverse_parser.rejected_frames +
                        reverse_parser.rejected_profile_states)
                    camera_diag_send(
                        uart, status_flags, calibration_code,
                        observation_code, reverse_code, raw_profile_ack,
                        packet[11], sequence_sent, tx_error_count,
                        detector_error_count, reverse_rejects)
                    diag_end_us = time.ticks_us()
                    camera_diagnostics.last_sideband_elapsed_us = time.ticks_diff(
                        diag_end_us, diag_start_us)
            except Exception:
                # Sideband diagnostics are isolated from primary control state.
                if diag_start_us is not None:
                    camera_diagnostics.last_sideband_elapsed_us = time.ticks_diff(
                        time.ticks_us(), diag_start_us)
                camera_diagnostics._record_error()

        # Defer total_cycle until the next iteration's start so its duration
        # includes all stage recording and loop overhead. The final iteration
        # is closed at the run boundary below.
        pending_cycle_start_us = cycle_start_us

    if pending_cycle_start_us is not None:
        # No next frame start exists for a finite run; close this final cycle
        # after all per-stage records have completed.
        perf.record("total_cycle", time.ticks_diff(
            time.ticks_us(), pending_cycle_start_us))
    _print_v7_summary(perf, run_id, current_window, perf.frames,
                      summary_queue)
    _drain_v7_summary_queue(
        summary_queue, time.sleep_ms if duration_s is not None else None)
    if summary_queue.overflow_windows:
        print("V7_PERF_QUEUE run=%d overflow_windows=%d capacity=%d" % (
            run_id, summary_queue.overflow_windows,
            summary_queue.capacity))
    print("V7_FAST_RUN_DONE run=%d duration_s=%s frames=%d "
          "board_storage_write=0 h743_uart_receive_verified=0 motors=0" % (
              run_id, str(duration_s), frame_count))
    if duration_s is not None:
        # Let the USB stdout channel drain before the finite RAM script exits
        # and the SDK soft-resets the REPL. This post-run wait is outside the
        # measured frame interval and is never used by the persistent loop.
        time.sleep_ms(500)


def main():
    backend = getattr(config, "VISION_BACKEND", "legacy")
    if backend == "fast":
        _main_fast()
    elif backend == "compare":
        _main_compare()
    elif backend == "legacy":
        _main_legacy()
    else:
        raise ValueError("VISION_BACKEND must be fast, compare or legacy")


if __name__ == "__main__":
    main()
