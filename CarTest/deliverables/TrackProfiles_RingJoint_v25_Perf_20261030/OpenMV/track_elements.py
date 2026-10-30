"""Conservative ring and cross observations over existing frame evidence.

The ring observer reads accepted road rows or fresh raw single-edge rows and
checks only its output corridors. A bounded Viper mask sampler runs solely on
the OPEN_BAND cross early-reject path. The module never owns or writes the
bilateral width cache; inferred opposite edges are output-only.
"""

from array import array
import time as _time

try:
    import micropython
    _IS_MICROPYTHON = True
except ImportError:  # Host tests only; OpenMV supplies the native decorator.
    _IS_MICROPYTHON = False

    class _HostMicropython:
        @staticmethod
        def viper(function):
            return function

        @staticmethod
        def native(function):
            return function

    micropython = _HostMicropython()

    def ptr8(buffer):
        return buffer

    def ptr32(buffer):
        return buffer


if hasattr(_time, "ticks_diff"):
    _ticks_diff = _time.ticks_diff
else:
    # OpenMV uses a 30-bit tick counter. Keep host wrap behavior identical to
    # time.ticks_diff so boundary and rollover cases are testable on CPython.
    _TICKS_PERIOD = 1 << 30
    _TICKS_HALF = _TICKS_PERIOD >> 1

    def _ticks_diff(new_tick, old_tick):
        return ((int(new_tick) - int(old_tick) + _TICKS_HALF) &
                (_TICKS_PERIOD - 1)) - _TICKS_HALF


WIDTH = 80
HEIGHT = 60
RING_REFERENCE_Y = 40
RUNTIME_API_VERSION = 2
JOINT_ROUTE_API_VERSION = 1
ROAD_ROW_STRIDE = 4
TARGET_ROW_STRIDE = 4

STATE_OBSERVE = 0
STATE_ENTRY_CONFIRM = 1
STATE_ENTER = 2
STATE_INSIDE = 3
STATE_EXIT_CONFIRM = 4
STATE_EXIT_REPAIR = 5
STATE_REACQUIRE = 6

_STATE_NAMES = (
    "OBSERVE", "ENTRY_CONFIRM", "ENTER", "INSIDE", "EXIT_CONFIRM",
    "EXIT_REPAIR", "REACQUIRE",
)

SIDE_NONE = -1
SIDE_LEFT = 0
SIDE_RIGHT = 1

# Target row flag bits. The input cache and detector buffers are never edited.
TARGET_OBSERVED_LEFT = 1
TARGET_OBSERVED_RIGHT = 2
TARGET_INFERRED_LEFT = 4
TARGET_INFERRED_RIGHT = 8
TARGET_WIDTH_FROM_CACHE = 16
TARGET_WIDTH_FROM_CALIBRATION = 32

RING_DIAG_STRIDE = 14

_RING_DIAG_REASON_NAMES = (
    "NONE", "CALIBRATION_REQUIRED", "BASELINE_NOT_READY", "INPUT_INVALID",
    "TOPOLOGY_UNCLEAR", "NO_RING_DROPOUT", "DROPOUT_CANDIDATE",
    "DROPOUT_CONFIRMED", "ARC_WAIT", "ARC_CONFIRMED", "EXIT_WAIT",
    "EXIT_FIT", "NORMAL_REACQUIRE",
)

EVENT_ENTRY_CANDIDATE = 1
EVENT_RING_CONFIRMED = 2
EVENT_EXIT_REOBSERVED = 3
EVENT_ABORT = 4
EVENT_CROSS_CONFIRMED = 5

EVENT_CAPACITY = 16
EVENT_STRIDE = 8  # frame, event, state, side, reason, edge2, time, target2

CROSS_ROUTE_MEASURED = 7  # FastBoundaryEvidence bilateral support bits.
CROSS_ROUTE_INTERPOLATED = 2


@micropython.viper
def _cross_sample_viper(road_object, edge_object, known_object,
                        left_object, right_object, valid_object,
                        multiple_object) -> int:
    """Extract one continuous, edge-supported road run per 80x60 row."""
    road = ptr8(road_object)
    edge = ptr8(edge_object)
    known = ptr8(known_object)
    left_output = ptr32(left_object)
    right_output = ptr32(right_object)
    valid = ptr8(valid_object)
    multiple = ptr8(multiple_object)
    total_valid = 0
    y = 0
    while y < 60:
        valid[y] = 0
        multiple[y] = 0
        left_output[y] = 0
        right_output[y] = 0
        row = y * 80
        x = 0
        run_count = 0
        run_left = -1
        run_right = -1
        while x < 80:
            index = row + x
            if known[index] != 0 and road[index] != 0:
                start = x
                while (x < 80 and known[row + x] != 0 and
                       road[row + x] != 0):
                    x += 1
                end = x - 1
                run_count += 1
                if run_count == 1:
                    run_left = start
                    run_right = end
                else:
                    multiple[y] = 1
            else:
                x += 1
        if run_count == 1 and run_left >= 1 and run_right <= 78:
            width = run_right - run_left + 1
            left_edge = row + run_left - 1
            right_edge = row + run_right + 1
            if (width >= 8 and width <= 70 and
                    known[left_edge] != 0 and known[right_edge] != 0 and
                    road[left_edge] == 0 and road[right_edge] == 0 and
                    edge[left_edge] != 0 and edge[right_edge] != 0):
                valid[y] = 1
                left_output[y] = run_left
                right_output[y] = run_right
                total_valid += 1
        y += 1
    return total_valid


@micropython.viper
def _cross_corridor_viper(road_object, known_object, far_y: int,
                          near_y: int, far_left: int, far_right: int,
                          near_left: int, near_right: int) -> int:
    """Require every pixel in an interpolated straight corridor to be road."""
    road = ptr8(road_object)
    known = ptr8(known_object)
    if near_y <= far_y or far_y < 0 or near_y >= 60:
        return -1
    span = near_y - far_y
    y = far_y
    while y <= near_y:
        offset = y - far_y
        left = far_left + ((near_left - far_left) * offset) // span
        right = far_right + ((near_right - far_right) * offset) // span
        if left < 0 or right >= 80 or left > right:
            return 0
        index = y * 80 + left
        while index <= y * 80 + right:
            if known[index] == 0 or road[index] == 0:
                return 0
            index += 1
        y += 1
    return 1


@micropython.viper
def _ring_corridor_viper(road_object, known_object, rows_object,
                         count: int, ring_side: int) -> int:
    """Check measured ring edge to centerline cells for known road support."""
    road = ptr8(road_object)
    known = ptr8(known_object)
    rows = ptr32(rows_object)
    row_index = 0
    while row_index < count:
        offset = row_index * 4
        y = rows[offset]
        left = rows[offset + 1]
        right = rows[offset + 2]
        if y < 0 or y >= 60 or left < 0 or right >= 80 or left > right:
            return 0
        center = (left + right) // 2
        if ring_side == 0:
            x = left
            end = center
        else:
            x = center
            end = right
        index = y * 80 + x
        last = y * 80 + end
        while index <= last:
            if known[index] == 0 or road[index] == 0:
                return 0
            index += 1
        row_index += 1
    return 1


@micropython.viper
def _prepare_nearest_cache_viper(valid_object, width_object,
                                 left_object, right_object, nearest_object,
                                 near_start: int) -> int:
    """Validate and index the 60-row measured cache in two fixed passes."""
    valid = ptr8(valid_object)
    widths = ptr32(width_object)
    left2 = ptr32(left_object)
    right2 = ptr32(right_object)
    nearest = ptr32(nearest_object)
    last = -1
    count = 0
    near_count = 0
    y = 0
    while y < 60:
        nearest[y] = -1
        width = widths[y]
        left = left2[y]
        right = right2[y]
        if (valid[y] != 0 and width >= 8 and width <= 70 and
                left >= 0 and right > left and right <= 158):
            last = y
            count += 1
            if y >= near_start:
                near_count += 1
        nearest[y] = last
        y += 1
    last = -1
    y = 59
    while y >= 0:
        width = widths[y]
        left = left2[y]
        right = right2[y]
        if (valid[y] != 0 and width >= 8 and width <= 70 and
                left >= 0 and right > left and right <= 158):
            last = y
        lower = nearest[y]
        if last >= 0 and (lower < 0 or last - y < y - lower):
            nearest[y] = last
        y -= 1
    return count * 256 + near_count


@micropython.viper
def _load_road_rows_viper(rows_object, count: int, floor_y: int,
                          status_object,
                          edge_object, seen_object) -> int:
    """Copy accepted road rows into side-major current measured evidence."""
    rows = ptr32(rows_object)
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    seen = ptr8(seen_object)
    index = 0
    while index < 120:
        status[index] = 0
        edges[index] = 0
        index += 1
    y = 0
    while y < 60:
        seen[y] = 0
        y += 1
    index = 0
    while index < count:
        source = index * 4
        row_y = rows[source]
        left = rows[source + 1]
        right = rows[source + 2]
        flags = rows[source + 3]
        if (row_y < floor_y or row_y >= 60 or left < 1 or right > 78 or
                left > right or (flags & 7) != 7 or seen[row_y] != 0):
            return -1
        seen[row_y] = 1
        status[row_y] = 1
        status[60 + row_y] = 1
        edges[row_y] = left * 2 - 1
        edges[60 + row_y] = right * 2 + 1
        index += 1
    return 1


@micropython.viper
def _copy_single_rows_viper(source_status_object, source_edge_object,
                            source_contrast_object, status_object,
                            edge_object, contrast_object) -> int:
    """Copy one fresh side-major scan without Python per-row dispatch."""
    source_status = ptr8(source_status_object)
    source_edges = ptr32(source_edge_object)
    source_contrast = ptr32(source_contrast_object)
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    contrast = ptr32(contrast_object)
    index = 0
    while index < 120:
        state = source_status[index]
        edge2 = source_edges[index]
        if (state > 2 or
                (state == 1 and (edge2 < 0 or edge2 > 159))):
            return -1
        status[index] = state
        edges[index] = edge2
        contrast[index] = source_contrast[index]
        index += 1
    return 1


@micropython.viper
def _measure_side_support_viper(status_object, edge_object, output_object,
                                height: int, floor_y: int, near_rows: int,
                                reference_y: int) -> int:
    """Collect near support and both reference-edge observations in one pass."""
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    output = ptr32(output_object)
    near_start = height - near_rows
    if near_start < floor_y:
        near_start = floor_y

    left_rows = 0
    right_rows = 0
    left_run = 0
    right_run = 0
    left_max_run = 0
    right_max_run = 0
    left_ambiguous = 0
    right_ambiguous = 0
    y = near_start
    while y < height:
        left_status = status[y]
        right_status = status[height + y]
        if left_status == 1:
            left_rows += 1
            left_run += 1
            if left_run > left_max_run:
                left_max_run = left_run
        else:
            if left_status == 2:
                left_ambiguous += 1
            left_run = 0
        if right_status == 1:
            right_rows += 1
            right_run += 1
            if right_run > right_max_run:
                right_max_run = right_run
        else:
            if right_status == 2:
                right_ambiguous += 1
            right_run = 0
        y += 1

    output[0] = left_rows
    output[1] = right_rows
    output[2] = left_max_run
    output[3] = right_max_run
    output[4] = left_ambiguous
    output[5] = right_ambiguous

    side = 0
    while side < 2:
        start = reference_y - 4
        if start < floor_y:
            start = floor_y
        end = reference_y + 4
        if end >= height:
            end = height - 1
        best_y = -1
        best_delta = height + 1
        best_edge = 0
        y = start
        base = side * height
        while y <= end:
            slot = base + y
            if status[slot] == 1:
                delta = y - reference_y
                if delta < 0:
                    delta = -delta
                if delta < best_delta:
                    best_delta = delta
                    best_y = y
                    best_edge = edges[slot]
            y += 1
        output[6 + side * 2] = best_y
        output[7 + side * 2] = best_edge
        side += 1
    return 0


@micropython.viper
def _measure_entry_dropout_viper(status_object, edge_object,
                                 baseline_valid_object,
                                 baseline_left_object, baseline_right_object,
                                 output_object, outer_shift_object,
                                 outer_shift_valid_object, height: int,
                                 floor_y: int, near_rows: int,
                                 reappear_shift2: int,
                                 opposite_tolerance2: int) -> int:
    """Measure both mirrored entry-drop candidates in one bounded scan."""
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    baseline_valid = ptr8(baseline_valid_object)
    baseline_left = ptr32(baseline_left_object)
    baseline_right = ptr32(baseline_right_object)
    output = ptr32(output_object)
    outer_shift = ptr32(outer_shift_object)
    outer_shift_valid = ptr8(outer_shift_valid_object)
    output[0] = 0
    output[1] = 0
    output[2] = 0
    output[3] = 0
    output[4] = 0
    output[5] = 0

    left_run = 0
    left_longest = 0
    left_missing = 0
    right_run = 0
    right_longest = 0
    right_missing = 0
    y = floor_y
    while y < 48:
        if baseline_valid[y] == 0:
            left_run = 0
            right_run = 0
        else:
            left_lost = status[y] == 0
            left_moved = False
            if status[y] == 1:
                outward = edges[y] - baseline_left[y]
                if status[height + y] == 1:
                    # Remove same-row motion shared with the outside edge.
                    outer_delta = edges[height + y] - baseline_right[y]
                    residual = outer_delta - outer_shift[1]
                    if residual < 0:
                        residual = -residual
                    outward -= outer_delta
                    if (residual <= opposite_tolerance2 and
                            outward >= reappear_shift2):
                        left_moved = True
                elif (status[height + y] == 0 and
                      outer_shift_valid[1] != 0):
                    # The far outside edge may be cropped; use its validated
                    # continuous near-anchor median shift instead.
                    outward -= outer_shift[1]
                    if outward >= reappear_shift2:
                        left_moved = True
            if status[height + y] == 2:
                # An ambiguous outside edge cannot anchor either dropout or
                # motion evidence for this row.
                left_lost = False
                left_moved = False
            if left_lost or left_moved:
                left_run += 1
                left_missing += 1
                if left_run > left_longest:
                    left_longest = left_run
            else:
                left_run = 0

            right_lost = status[height + y] == 0
            right_moved = False
            if status[height + y] == 1:
                outward = baseline_right[y] - edges[height + y]
                if status[y] == 1:
                    # For a right inner edge, rightward common motion has the
                    # opposite sign in the outward-left coordinate.
                    outer_delta = edges[y] - baseline_left[y]
                    residual = outer_delta - outer_shift[0]
                    if residual < 0:
                        residual = -residual
                    outward += outer_delta
                    if (residual <= opposite_tolerance2 and
                            outward >= reappear_shift2):
                        right_moved = True
                elif status[y] == 0 and outer_shift_valid[0] != 0:
                    outward += outer_shift[0]
                    if outward >= reappear_shift2:
                        right_moved = True
            if status[y] == 2:
                right_lost = False
                right_moved = False
            if right_lost or right_moved:
                right_run += 1
                right_missing += 1
                if right_run > right_longest:
                    right_longest = right_run
            else:
                right_run = 0
        y += 1

    near_start = height - near_rows
    if near_start < floor_y:
        near_start = floor_y
    left_anchor = 0
    left_anchor_run = 0
    left_anchor_longest = 0
    right_anchor = 0
    right_anchor_run = 0
    right_anchor_longest = 0
    y = near_start
    while y < height:
        if (outer_shift_valid[1] != 0 and baseline_valid[y] != 0 and
                status[height + y] == 1):
            delta = (edges[height + y] - baseline_right[y] -
                     outer_shift[1])
            if delta < 0:
                delta = -delta
            if delta <= opposite_tolerance2:
                left_anchor += 1
                left_anchor_run += 1
                if left_anchor_run > left_anchor_longest:
                    left_anchor_longest = left_anchor_run
            else:
                left_anchor_run = 0
        else:
            left_anchor_run = 0
        if (outer_shift_valid[0] != 0 and baseline_valid[y] != 0 and
                status[y] == 1):
            delta = edges[y] - baseline_left[y] - outer_shift[0]
            if delta < 0:
                delta = -delta
            if delta <= opposite_tolerance2:
                right_anchor += 1
                right_anchor_run += 1
                if right_anchor_run > right_anchor_longest:
                    right_anchor_longest = right_anchor_run
            else:
                right_anchor_run = 0
        else:
            right_anchor_run = 0
        y += 1

    output[0] = left_missing
    output[1] = left_longest
    output[2] = left_anchor
    output[3] = right_missing
    output[4] = right_longest
    output[5] = right_anchor
    output[6] = left_anchor_longest
    output[7] = right_anchor_longest
    return 0


@micropython.viper
def _clear_rows_viper(status_object, edge_object, contrast_object,
                      seen_object) -> int:
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    contrast = ptr32(contrast_object)
    seen = ptr8(seen_object)
    index = 0
    while index < 120:
        status[index] = 0
        edges[index] = 0
        contrast[index] = 0
        index += 1
    index = 0
    while index < 60:
        seen[index] = 0
        index += 1
    return 0


@micropython.viper
def _clear_i32_prefix_viper(buffer_object, count: int) -> int:
    values = ptr32(buffer_object)
    index = 0
    while index < count:
        values[index] = 0
        index += 1
    return 0

REJECT_NONE = 0
REJECT_INPUT_INVALID = 1
REJECT_FRAME_INVALID = 2
REJECT_SESSION_INACTIVE = 3
REJECT_PROFILE_UNAVAILABLE = 4
REJECT_CACHE_INVALID = 5
REJECT_ROAD_ROWS_INVALID = 6
REJECT_SINGLE_ROWS_STALE = 7
REJECT_TOPOLOGY_AMBIGUOUS = 8
REJECT_SIDE_SUPPORT = 9
REJECT_RING_SEQUENCE = 10
REJECT_CROSS_EVIDENCE = 11
REJECT_STATE_TIMEOUT = 12
REJECT_COUNT = 13

_REJECT_NAMES = (
    "NONE", "INPUT_INVALID", "FRAME_INVALID", "SESSION_INACTIVE",
    "PROFILE_UNAVAILABLE", "CACHE_INVALID", "ROAD_ROWS_INVALID",
    "SINGLE_ROWS_STALE", "TOPOLOGY_AMBIGUOUS", "SIDE_SUPPORT",
    "RING_SEQUENCE", "CROSS_EVIDENCE", "STATE_TIMEOUT",
)


def _setting(settings, name, default):
    if settings is None:
        return default
    try:
        return settings.get(name, default)
    except AttributeError:
        return getattr(settings, name, default)


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


@micropython.native
def _sort_prefix(values, count):
    index = 1
    while index < count:
        value = values[index]
        insert = index
        while insert > 0 and values[insert - 1] > value:
            values[insert] = values[insert - 1]
            insert -= 1
        values[insert] = value
        index += 1


@micropython.viper
def _ordinary_calibrated_corridor_viper(status_object, edge_object,
                                        valid_object, width_object,
                                        frozen_valid_object, frozen_width_object,
                                        use_frozen: int, floor: int,
                                        height: int) -> int:
    status = ptr8(status_object)
    edges = ptr32(edge_object)
    valid = ptr8(valid_object)
    widths = ptr32(width_object)
    frozen_valid = ptr8(frozen_valid_object)
    frozen_widths = ptr32(frozen_width_object)
    rows = 0
    min_y = height
    max_y = -1
    y = floor
    while y < height:
        if status[y] == 1 and status[height + y] == 1:
            left = edges[y]
            right = edges[height + y]
            if left < right:
                width = 0
                if valid[y] != 0:
                    width = widths[y]
                elif use_frozen != 0 and frozen_valid[y] != 0:
                    width = frozen_widths[y]
                # Legal measured edges are 0..159 half-grid coordinates.
                # Widths above 100 cannot satisfy the 20% test, and bounding
                # here also rejects corrupt signed values read through ptr32.
                if width >= 8 and width <= 100:
                    measured = (right - left) // 2
                    delta = measured - width
                    if delta < 0:
                        delta = -delta
                    # Exact integer equivalent of abs(delta)*5 <= width;
                    # division avoids int32 multiplication overflow.
                    if delta <= width // 5:
                        rows += 1
                        if y < min_y:
                            min_y = y
                        if y > max_y:
                            max_y = y
        y += 1
    if rows >= 8 and max_y - min_y >= 12:
        return 1
    return 0


class TrackElementsTracker:
    """Bounded temporal element observer with reusable buffers and results.

    `update()` consumes the already-extracted 80x60 observations. `road_rows`
    must be from the current, selected, boundary-validated road candidate.
    `row_status`/`row_edge2` must be the raw FastSingleEdgeTracker arrays and
    may be read only when `single_rows_fresh` is true. The inferred
    single-edge result rows are intentionally not accepted by this API.
    """

    def __init__(self, settings=None, calibration=None,
                 require_calibration=True):
        self.width = int(_setting(settings, "SURFACE_GRID_WIDTH", WIDTH))
        self.height = int(_setting(settings, "SURFACE_GRID_HEIGHT", HEIGHT))
        if self.width != WIDTH or self.height != HEIGHT:
            raise ValueError("TrackElementsTracker requires an 80x60 grid")

        self.floor_y = int(_setting(settings, "SURFACE_ROI_FLOOR_Y", 8))
        if self.floor_y < 0 or self.floor_y >= self.height:
            raise ValueError("SURFACE_ROI_FLOOR_Y must be in 0..59")
        self.reference_y = int(float(_setting(
            settings, "ROAD_REFERENCE_Y", 40)) + 0.5)
        if self.reference_y < self.floor_y or self.reference_y >= self.height:
            raise ValueError("ROAD_REFERENCE_Y must lie inside the ROI")

        self.near_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_NEAR_ROWS", 12, 4, self.height)
        self.min_near_support = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_NEAR_SUPPORT", 5, 2,
            self.near_rows)
        self.min_near_contiguous = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS", 4, 2,
            self.near_rows)
        self.ring_entry_min_near_support = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ENTRY_MIN_NEAR_SUPPORT", 4, 2,
            self.near_rows)
        self.ring_entry_min_near_contiguous = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ENTRY_MIN_NEAR_CONTIGUOUS", 3,
            2, self.near_rows)
        self.min_stable_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_STABLE_FRAMES", 3, 2, 20)
        self.min_side_support = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_SIDE_SUPPORT", 5, 2,
            self.near_rows)
        self.min_side_contiguous = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_SIDE_CONTIGUOUS", 4, 2,
            self.near_rows)
        self.max_missing_side_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_MISSING_SIDE_ROWS", 1, 0,
            self.min_side_support - 1)
        self.min_drop_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_DROP_FRAMES", 2, 2, 10)
        self.opposite_position_tolerance2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_OPPOSITE_POSITION_TOLERANCE_X2", 6,
            0, (self.width - 1) * 2)
        self.reappear_shift2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_REAPPEAR_SHIFT_X2", 8, 2,
            (self.width - 1) * 2)
        self.ring_entry_opposite_tolerance2 = self._bounded_setting(
            settings,
            "TRACK_ELEMENTS_RING_ENTRY_OPPOSITE_POSITION_TOLERANCE_X2",
            10, 0, (self.width - 1) * 2)
        self.ring_entry_reappear_shift2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ENTRY_REAPPEAR_SHIFT_X2", 6,
            2, (self.width - 1) * 2)
        self.peak_reversal2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_PEAK_REVERSAL_X2", 6, 2,
            (self.width - 1) * 2)
        self.peak_confirm_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_PEAK_CONFIRM_FRAMES", 2, 2, 10)
        self.ring_arc_min_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ARC_MIN_ROWS", 4, 2,
            self.height)
        self.ring_arc_min_contiguous_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ARC_MIN_CONTIGUOUS_ROWS", 3, 2,
            self.height)
        self.ring_arc_min_span_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ARC_MIN_SPAN_ROWS", 4, 1,
            self.height - 1)
        self.ring_arc_min_growth2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ARC_MIN_GROWTH_X2", 2, 1,
            (self.width - 1) * 2)
        self.ring_arc_growing_support_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_RING_ARC_GROWING_SUPPORT_FRAMES", 3,
            2, 10)
        self.min_target_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MIN_TARGET_ROWS", 5, 3, self.height)
        self.max_cache_gap = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_CACHE_GAP", 3, 0, 15)
        self.max_entry_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_ENTRY_FRAMES", 30, 2, 600)
        self.max_active_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_ACTIVE_FRAMES", 180, 2, 2000)
        self.max_entry_ms = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_ENTRY_MS", 4000, 1, 60000)
        self.max_active_ms = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_ACTIVE_MS", 16000, 1, 600000)
        self.exit_confirm_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_EXIT_CONFIRM_FRAMES", 4, 2, 30)
        self.exit_repair_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_EXIT_REPAIR_FRAMES", 4, 2, 30)
        self.exit_drop_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_EXIT_DROP_FRAMES", 3, 2, 30)
        self.exit_width_tolerance = self._bounded_setting(
            settings, "TRACK_ELEMENTS_EXIT_WIDTH_TOLERANCE_CELLS", 6,
            0, 30)
        self.exit_center_tolerance2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_EXIT_CENTER_TOLERANCE_X2", 12, 0,
            (self.width - 1) * 2)
        self.reacquire_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_REACQUIRE_FRAMES", 3, 2, 30)

        self.cross_confirm_frames = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_CONFIRM_FRAMES", 2, 2, 10)
        self.cross_min_open_rows = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_MIN_OPEN_ROWS", 3, 2,
            self.height)
        self.cross_open_min_cells = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_OPEN_MIN_CELLS", 3, 1, 20)
        self.cross_symmetry_tolerance = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_SYMMETRY_TOL_CELLS", 2, 0, 20)
        self.cross_center_tolerance2 = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_CENTER_TOLERANCE_X2", 6, 0,
            (self.width - 1) * 2)
        self.cross_width_tolerance = self._bounded_setting(
            settings, "TRACK_ELEMENTS_CROSS_WIDTH_TOLERANCE_CELLS", 6,
            0, 30)
        self.max_active_millirev = self._bounded_setting(
            settings, "TRACK_ELEMENTS_MAX_ACTIVE_MILLIREV", 60000, 0,
            2000000000)

        # These buffers are allocated once. Per-frame updates only overwrite
        # their fixed contents; no image, row, or event list is constructed.
        self._current_status = bytearray(2 * self.height)
        self._current_edge2 = array("i", [0] * (2 * self.height))
        self._current_contrast = array("i", [0] * (2 * self.height))
        self._side_summary = array("i", [0] * 10)
        self._entry_dropout_summary = array("i", [0] * 8)
        self._outer_shift_samples = array("i", [0] * self.near_rows)
        self._outer_shift_x2 = array("i", [0] * 2)
        self._outer_shift_valid = bytearray(2)
        self._outer_anchor_rows = array("i", [0] * 2)
        self._outer_anchor_run = array("i", [0] * 2)
        self._row_seen = bytearray(self.height)
        self._cross_open_rows = bytearray(self.height)
        self._cross_sample_valid = bytearray(self.height)
        self._cross_sample_support = bytearray(self.height)
        self._cross_sample_multiple = bytearray(self.height)
        self._cross_sample_left = array("i", [0] * self.height)
        self._cross_sample_right = array("i", [0] * self.height)
        self._nearest_cache_y = array("i", [-1] * self.height)
        self._ring_width_valid = bytearray(self.height)
        self._ring_width = array("i", [0] * self.height)
        self._ring_nearest_y = array("i", [-1] * self.height)
        self._ring_cache_frozen = False
        self._ring_width_calibrated = False
        self.require_calibration = bool(require_calibration)
        if self.require_calibration:
            self.exit_confirm_frames = 3
            self.reacquire_frames = 3
        self._calibration_source = None
        self._calibration_width_valid = None
        self._calibration_width_cells = None
        self._corridor_width_owner = None
        self._corridor_width_length = -1
        self._corridor_width_fast = False
        self._calibration_ready = False
        self._calibration_reason = "NO_CALIBRATION"
        self._cal_check_prev_valid = bytearray(self.height)
        self._cal_check_prev_width = array("i", [0] * self.height)
        self._cal_check_valid = bytearray(self.height)
        self._cal_check_width = array("i", [0] * self.height)
        self._cal_check_error = array("i", [0] * self.height)
        self._cal_check_stable_frames = 0
        self._cal_check_ready = False
        self._cal_check_reason = "NOT_CHECKED"
        self._cal_check_support_rows = 0
        self._cal_check_span_rows = 0
        self._cal_check_median_error_permille = -1
        self._ring_diag = array("i", [0] * RING_DIAG_STRIDE)
        self._ring_diag_arc_rows = 0
        self._ring_diag_anchor_rows = 0
        self._ring_diag_arc_span = 0
        self._ring_diag_anchor_run = 0
        self._ring_diag_arc_rejection = "NONE"
        self._ring_diag_growth_frames = 0
        self._ring_diag_growth_x2 = 0
        self._ring_diag_exit_rows = 0
        self._ring_diag_exit_span = 0
        self._ring_diag_exit_residual = -1
        self._ring_diag_reason_code = 0
        self._entry_growth_frames = 0
        self._entry_growth_origin_x2 = -1
        self._entry_growth_last_x2 = -1
        self._growth_confirmation_latched = False
        self._entry_confirmation_path = "NONE"
        self._scratch_left = array("i", [0] * self.height)
        self._scratch_right = array("i", [0] * self.height)
        self._scratch_y = array("i", [0] * self.height)
        self._band_summary = array("i", [0] * 12)
        self._cross_summary = array("i", [0] * 5)
        self._target_rows = array("i", [0] * (self.height * TARGET_ROW_STRIDE))
        self._cross_route_rows = array(
            "i", [0] * (self.height * TARGET_ROW_STRIDE))
        self._events = array("i", [0] * (EVENT_CAPACITY * EVENT_STRIDE))
        self._reject_counts = array("i", [0] * REJECT_COUNT)

        self._result = {
            "state": "OBSERVE",
            "state_id": STATE_OBSERVE,
            "reason": "RESET",
            "candidate": False,
            "ring_confirmed": False,
            "side": SIDE_NONE,
            "side_name": "NONE",
            "control_eligible": False,
            "target_valid": False,
            "target_rows": self._target_rows,
            "target_count": 0,
            "target_stride": TARGET_ROW_STRIDE,
            "target_side": SIDE_NONE,
            "target_side_name": "NONE",
            "target_left2": 0,
            "target_right2": 0,
            "target_center2": 0,
            "target_lateral_error": 0,
            "observed_rows": 0,
            "inferred_rows": 0,
            "exit_experimental": False,
            "exit_reobserved": False,
            "cross_candidate": False,
            "cross_confirmed": False,
            "cross_suppress_slowdown": False,
            "cross_reason": "NOT_EVALUATED",
            "cross_rows": 0,
            "cross_near_rows": 0,
            "cross_far_rows": 0,
            "cross_span": 0,
            "cross_near_y": -1,
            "cross_far_y": -1,
            "cross_near_center2": 0,
            "cross_far_center2": 0,
            "cross_reference_center2": 0,
            "cross_reference_y": -1,
            "cross_lateral_error": 0,
            "cross_route_usable": False,
            "cross_route_rows": self._cross_route_rows,
            "cross_route_count": 0,
            "cross_route_stride": TARGET_ROW_STRIDE,
            "cross_open_rows": 0,
            "cross_open_segments": 0,
            "cross_candidate_tracks": 0,
            "reject_reason": "NONE",
            "reject_counts": self._reject_counts,
            "event_log": self._events,
            "event_capacity": EVENT_CAPACITY,
            "event_stride": EVENT_STRIDE,
            "event_count": 0,
            "event_head": 0,
            "event_dropped": 0,
            "frame_index": 0,
            "profile_id": 0,
            "profile_seq": 0,
            "session_token": None,
            "state_frames": 0,
            "state_elapsed_ms": 0,
            "mean_forward_wheel_millirev": None,
            "millirev_delta": None,
            "millirev_guard_enabled": False,
            "ring_calibration_ready": False,
            "ring_calibration_reason": "NO_CALIBRATION",
            "ring_calibration_check_ready": False,
            "ring_calibration_check_reason": "NOT_CHECKED",
            "ring_calibration_check_frames": 0,
            "ring_calibration_support_rows": 0,
            "ring_calibration_span_rows": 0,
            "ring_calibration_median_error_permille": -1,
            "ring_diagnostic_record": self._ring_diag,
            "ring_diagnostic_stride": RING_DIAG_STRIDE,
            "ring_diagnostic_reason": "NONE",
            "ring_arc_support_rows": 0,
            "ring_arc_span_rows": 0,
            "ring_arc_rejection": "NONE",
            "ring_arc_growth_frames": 0,
            "ring_arc_growth_x2": 0,
            "growth_ready_but_target_rejected": False,
            "growth_target_rejection": "NONE",
            "entry_confirmation_path": "NONE",
            "ring_target_rejection": "NONE",
            "ring_near_anchor_rows": 0,
            "ring_near_anchor_run": 0,
            "ring_exit_fit_rows": 0,
            "ring_exit_span_rows": 0,
            "ring_exit_residual_x2": -1,
            "ring_width_source": "NONE",
        }

        self._identity_set = False
        self._profile_id = 0
        self._session_active = False
        self._session_token = None
        self._result["profile_id"] = 0
        self._result["profile_seq"] = 0
        self._result["session_token"] = None
        self._frame_index = 0
        self._state = STATE_OBSERVE
        self._reason = "RESET"
        self._side = SIDE_NONE
        self._state_frames = 0
        self._state_start_ms = 0
        self._last_now_ms = 0
        self._current_odom = None
        self._current_odom_valid = False
        self._ring_odom_start = None
        self._ring_odom_start_valid = False
        self._last_odom = None
        self._last_odom_valid = False
        self._event_count = 0
        self._event_head = 0
        self._event_dropped = 0
        self._stable_count = 0
        self._baseline_ready = False
        self._baseline_left2 = 0
        self._baseline_right2 = 0
        self._baseline_y = -1
        self._baseline_row_valid = bytearray(self.height)
        self._baseline_left_by_y2 = array("i", [-1] * self.height)
        self._baseline_right_by_y2 = array("i", [-1] * self.height)
        self._ring_cache_frozen = False
        self._release_ring_width()
        self._drop_count = 0
        self._reappear_count = 0
        self._extreme_x2 = 0
        self._reverse_count = 0
        self._reset_entry_growth()
        self._exit_count = 0
        self._exit_side = SIDE_NONE
        self._exit_drop_count = 0
        self._exit_reappear_count = 0
        self._repair_count = 0
        self._reacquire_count = 0
        self._cross_count = 0
        self._cross_was_confirmed = False
        self._current_valid = False
        self._current_road = False
        self._current_topology_clear = False
        self._route_outcome_count = 0
        self._route_outcome_last_frame = -1
        self._cache_valid = False
        self._active_cache_valid = None
        self._active_cache_width = None
        self._active_cache_left2 = None
        self._active_cache_right2 = None
        self._near_left_rows = 0
        self._near_right_rows = 0
        self._near_left_run = 0
        self._near_right_run = 0
        self._near_left_ambiguous = 0
        self._near_right_ambiguous = 0
        self._reference_left2 = 0
        self._reference_right2 = 0
        self._reference_left_y = -1
        self._reference_right_y = -1
        self._last_reject = REJECT_NONE
        if calibration is not None:
            self.set_ring_calibration(calibration, require_calibration)

    @staticmethod
    def _bounded_setting(settings, name, default, minimum, maximum):
        value = _setting(settings, name, default)
        if (not _is_int(value) or value < minimum or value > maximum):
            raise ValueError("%s must be an integer in %d..%d" % (
                name, minimum, maximum))
        return value

    def _reset_route_outcome(self):
        self._route_outcome_count = 0
        self._route_outcome_last_frame = -1

    @property
    def result(self):
        """Return the single reusable result dictionary."""
        return self._result

    @property
    def ring_calibration_ready(self):
        return self._calibration_ready

    @property
    def ring_calibration_reason(self):
        return self._calibration_reason

    @property
    def ring_calibration_check_ready(self):
        return self._cal_check_ready

    def set_ring_calibration(self, calibration, require_calibration=True):
        """Install a validated calibration map and reset its startup check.

        ``calibration`` may be a ``RingWidthCalibration`` instance or the
        dictionary returned by its ``finalize()`` method. This setter keeps
        the map separate from the ordinary per-session width cache.
        """
        self.require_calibration = bool(require_calibration)
        if self.require_calibration:
            self.exit_confirm_frames = 3
            self.reacquire_frames = 3
        self._calibration_source = calibration
        if isinstance(calibration, dict):
            ready = calibration.get("ready", calibration.get(
                "calibration_ready", False))
            reason = calibration.get("reason", calibration.get(
                "calibration_reason", "INVALID_CALIBRATION"))
            valid = calibration.get("width_valid", calibration.get(
                "width_valid60"))
            widths = calibration.get("width_cells", calibration.get(
                "width_cells60"))
        else:
            ready = getattr(calibration, "ready", getattr(
                calibration, "calibration_ready", False))
            reason = getattr(calibration, "reason", getattr(
                calibration, "calibration_reason", "INVALID_CALIBRATION"))
            valid = getattr(calibration, "width_valid", getattr(
                calibration, "width_valid60", None))
            widths = getattr(calibration, "width_cells", getattr(
                calibration, "width_cells60", None))
        map_ok = False
        support_reason = "INVALID_CALIBRATION"
        try:
            if (bool(ready) and len(valid) == self.height and
                    len(widths) == self.height and
                    self.reference_y == RING_REFERENCE_Y):
                map_ok = True
                support = 0
                min_y = self.height
                max_y = -1
                near_rows = 0
                near_run = 0
                max_near_run = 0
                nearest_reference = self.height
                for y in range(self.floor_y, self.height):
                    flag = int(valid[y])
                    width = int(widths[y])
                    if flag not in (0, 1) or (flag and
                            (width < 8 or width > 70)) or (not flag and width):
                        map_ok = False
                        break
                    if flag:
                        support += 1
                        if y < min_y:
                            min_y = y
                        if y > max_y:
                            max_y = y
                        delta = abs(y - RING_REFERENCE_Y)
                        if delta < nearest_reference:
                            nearest_reference = delta
                    if y >= self.height - 12:
                        if flag:
                            near_rows += 1
                            near_run += 1
                            if near_run > max_near_run:
                                max_near_run = near_run
                        else:
                            near_run = 0
                if map_ok and (support < 12 or max_y - min_y < 20):
                    map_ok = False
                    support_reason = "INSUFFICIENT_CALIBRATION_CHAIN"
                if map_ok and nearest_reference > 3:
                    map_ok = False
                    support_reason = "REFERENCE_WIDTH_UNAVAILABLE"
                if map_ok and (near_rows < 5 or max_near_run < 4):
                    map_ok = False
                    support_reason = "NEAR_WIDTH_SUPPORT_INSUFFICIENT"
            elif self.reference_y != RING_REFERENCE_Y:
                support_reason = "REFERENCE_ROW_MISMATCH"
        except (TypeError, AttributeError, IndexError):
            map_ok = False
        self._calibration_ready = map_ok
        if map_ok:
            self._calibration_reason = "CALIBRATION_READY"
        elif support_reason != "INVALID_CALIBRATION":
            self._calibration_reason = support_reason
        else:
            self._calibration_reason = (
                "INVALID_CALIBRATION" if str(reason) ==
                "CALIBRATION_READY" else str(reason))
        self._calibration_width_valid = valid if map_ok else None
        self._calibration_width_cells = widths if map_ok else None
        self._cal_check_stable_frames = 0
        self._cal_check_ready = False
        self._cal_check_reason = ("WAITING_FOR_CALIBRATION" if map_ok else
                                  self._calibration_reason)
        self._cal_check_support_rows = 0
        self._cal_check_span_rows = 0
        self._cal_check_median_error_permille = -1
        for y in range(self.height):
            self._cal_check_prev_valid[y] = 0
            self._cal_check_prev_width[y] = 0
        if hasattr(self, "_result"):
            self._sync_calibration_result()
        return map_ok

    def reset_ring_calibration_check(self, reason="PARKED_GATE"):
        """Reset only an incomplete startup width check after an ineligible frame.

        A passed check is sticky until a new calibration object is installed.
        This method does not touch route, ring, or ordinary width-cache state.
        """
        if self._cal_check_ready:
            return False
        self._cal_check_stable_frames = 0
        self._cal_check_ready = False
        self._cal_check_reason = str(reason)
        self._cal_check_support_rows = 0
        self._cal_check_span_rows = 0
        self._cal_check_median_error_permille = -1
        for y in range(self.height):
            self._cal_check_prev_valid[y] = 0
            self._cal_check_prev_width[y] = 0
            self._cal_check_valid[y] = 0
            self._cal_check_width[y] = 0
        self._sync_calibration_result()
        return True

    def observe_ring_calibration(self, rows, count, stride):
        """Check one parked, accepted bilateral width profile against map.

        The caller is responsible for calling this only on fresh, accepted
        road rows while the car is stationary and topology is clear. Records
        are ``[y,left_x,right_x,flags]`` and width is inclusive.
        """
        if not self._calibration_ready:
            self._cal_check_ready = False
            self._cal_check_stable_frames = 0
            self._cal_check_reason = self._calibration_reason
            self._cal_check_support_rows = 0
            self._cal_check_span_rows = 0
            self._cal_check_median_error_permille = -1
            self._sync_calibration_result()
            return False
        if self._cal_check_ready:
            return True
        for y in range(self.height):
            self._cal_check_valid[y] = 0
            self._cal_check_width[y] = 0
            self._cal_check_error[y] = 0
        if (not _is_int(count) or count < self.min_near_support or
                count > self.height or not _is_int(stride) or stride < 4):
            return self._calibration_check_failed("INVALID_ROW_SHAPE")
        try:
            if rows is None or len(rows) < count * stride:
                return self._calibration_check_failed("INVALID_ROW_SHAPE")
        except (TypeError, AttributeError):
            return self._calibration_check_failed("INVALID_ROW_SHAPE")
        support = 0
        min_y = self.height
        max_y = -1
        offset = 0
        index = 0
        while index < count:
            y = rows[offset]
            left = rows[offset + 1]
            right = rows[offset + 2]
            flags = rows[offset + 3]
            if (not _is_int(y) or not _is_int(left) or
                    not _is_int(right) or not _is_int(flags) or
                    (flags & 7) != 7 or y < self.floor_y or
                    y >= self.height or self._cal_check_error[y] != 0 or
                    left < 0 or right >= self.width or right <= left):
                return self._calibration_check_failed("INVALID_ROW_GEOMETRY")
            # Use the error scratch array as a row-indexed duplicate bitmap
            # while consuming arbitrary producer order. It is packed with
            # calibration errors only after the entire row set is validated.
            self._cal_check_error[y] = -1
            calibrated = int(self._calibration_width_cells[y])
            measured = right - left + 1
            if (self._calibration_width_valid[y] and
                    calibrated >= 8 and calibrated <= 70):
                self._cal_check_valid[y] = 1
                self._cal_check_width[y] = measured
                support += 1
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
            index += 1
            offset += stride
        span = max_y - min_y if support else 0
        self._cal_check_support_rows = support
        self._cal_check_span_rows = span
        if support < 5 or span < 6:
            return self._calibration_check_failed("CHECK_SUPPORT_INSUFFICIENT")
        ordered = self._cal_check_error
        error_index = 0
        y = self.floor_y
        while y < self.height:
            if self._cal_check_valid[y]:
                calibrated = int(self._calibration_width_cells[y])
                measured = int(self._cal_check_width[y])
                ordered[error_index] = (
                    abs(measured - calibrated) * 1000 // calibrated)
                error_index += 1
            y += 1
        index = 1
        while index < support:
            value = ordered[index]
            at = index
            while at > 0 and ordered[at - 1] > value:
                ordered[at] = ordered[at - 1]
                at -= 1
            ordered[at] = value
            index += 1
        median_error = int(ordered[support // 2])
        self._cal_check_median_error_permille = median_error
        if median_error > 200:
            return self._calibration_check_failed("WIDTH_MISMATCH")

        common = 0
        deltas = self._cal_check_error
        for y in range(self.floor_y, self.height):
            if self._cal_check_valid[y] and self._cal_check_prev_valid[y]:
                deltas[common] = abs(
                    self._cal_check_width[y] - self._cal_check_prev_width[y])
                common += 1
        stable = False
        if common >= 5:
            index = 1
            while index < common:
                value = deltas[index]
                at = index
                while at > 0 and deltas[at - 1] > value:
                    deltas[at] = deltas[at - 1]
                    at -= 1
                deltas[at] = value
                index += 1
            stable = deltas[common // 2] <= 1 and deltas[common - 1] <= 2
        for y in range(self.height):
            self._cal_check_prev_valid[y] = self._cal_check_valid[y]
            self._cal_check_prev_width[y] = self._cal_check_width[y]
        if stable:
            self._cal_check_stable_frames += 1
        else:
            self._cal_check_stable_frames = 1
        self._cal_check_ready = self._cal_check_stable_frames >= 3
        self._cal_check_reason = ("STARTUP_CHECK_READY" if
                                  self._cal_check_ready else
                                  "WAITING_FOR_STABLE_WIDTH_PROFILE")
        self._sync_calibration_result()
        return self._cal_check_ready

    def _calibration_check_failed(self, reason):
        self._cal_check_stable_frames = 0
        self._cal_check_ready = False
        self._cal_check_reason = reason
        for y in range(self.height):
            self._cal_check_prev_valid[y] = self._cal_check_valid[y]
            self._cal_check_prev_width[y] = self._cal_check_width[y]
        self._sync_calibration_result()
        return False

    def _sync_calibration_result(self):
        result = self._result
        if result is None:
            return
        result["ring_calibration_ready"] = self._calibration_ready
        result["ring_calibration_reason"] = self._calibration_reason
        result["ring_calibration_check_ready"] = self._cal_check_ready
        result["ring_calibration_check_reason"] = self._cal_check_reason
        result["ring_calibration_check_frames"] = self._cal_check_stable_frames
        result["ring_calibration_support_rows"] = self._cal_check_support_rows
        result["ring_calibration_span_rows"] = self._cal_check_span_rows
        result["ring_calibration_median_error_permille"] = \
            self._cal_check_median_error_permille

    def reset(self, reason="EXPLICIT_RESET"):
        """Clear temporal state, fixed counters, events, and output rows."""
        self._reset_route_outcome()
        self._clear_frame_output()
        self._identity_set = False
        self._profile_id = 0
        self._session_active = False
        self._session_token = None
        self._result["profile_id"] = 0
        self._result["profile_seq"] = 0
        self._result["session_token"] = None
        self._frame_index = 0
        self._state = STATE_OBSERVE
        self._reason = reason
        self._side = SIDE_NONE
        self._state_frames = 0
        self._state_start_ms = 0
        self._last_now_ms = 0
        self._current_topology_clear = False
        self._current_odom = None
        self._current_odom_valid = False
        self._ring_odom_start = None
        self._ring_odom_start_valid = False
        self._last_odom = None
        self._last_odom_valid = False
        self._stable_count = 0
        self._baseline_ready = False
        self._baseline_left2 = 0
        self._baseline_right2 = 0
        self._baseline_y = -1
        self._clear_baseline_rows()
        self._ring_cache_frozen = False
        self._release_ring_width()
        self._drop_count = 0
        self._reappear_count = 0
        self._extreme_x2 = 0
        self._reverse_count = 0
        self._reset_entry_growth()
        self._exit_count = 0
        self._exit_side = SIDE_NONE
        self._exit_drop_count = 0
        self._exit_reappear_count = 0
        self._repair_count = 0
        self._reacquire_count = 0
        self._cross_count = 0
        self._cross_was_confirmed = False
        self._event_count = 0
        self._event_head = 0
        self._event_dropped = 0
        self._last_reject = REJECT_NONE
        for index in range(REJECT_COUNT):
            self._reject_counts[index] = 0
        for index in range(len(self._events)):
            self._events[index] = 0
        for index in range(len(self._target_rows)):
            self._target_rows[index] = 0
            self._cross_route_rows[index] = 0
        self._clear_current_rows()
        self._result["reason"] = reason
        self._result["reject_reason"] = "NONE"
        self._publish()

    @micropython.native
    def update(self, now_ms, profile_id, session_active, session_token,
               frame_fresh, road_accepted, road_rows, road_count,
               road_stride, road_candidate_tracks, topology,
               single_rows_fresh, row_status, row_edge2, row_contrast,
               cache_valid, cache_width, cache_left2, cache_right2,
               mask_info, profile_seq=0,
               mean_forward_wheel_millirev=None):
        """Advance one captured-frame observation and return reusable output.

        `profile_id` is 1=A, 2=B, 3=C; 0 means profile status is stale.
        Road rows are `[y,left_x,right_x,reserved]` in grid cells. Cache edge
        positions and raw single-edge positions are doubled x coordinates.
        The wheel millirev input is used only for an optional configured
        maximum-progress abort and diagnostics. It never advances a state or
        declares an entry, cross, or exit.
        """
        self._frame_index = (self._frame_index + 1) & 0x7fffffff
        self._clear_frame_output()
        self._last_now_ms = int(now_ms)
        self._current_topology_clear = False

        changed = (self._identity_set and
                   (profile_id != self._profile_id or
                    bool(session_active) != self._session_active or
                    session_token != self._session_token))
        if changed:
            self.reset("IDENTITY_CHANGE")
            self._frame_index = 1
            self._last_now_ms = int(now_ms)
        self._profile_id = profile_id if _is_int(profile_id) else 0
        self._session_active = bool(session_active)
        self._session_token = session_token
        self._identity_set = True
        self._result["profile_id"] = self._profile_id
        self._result["profile_seq"] = profile_seq if _is_int(profile_seq) else 0
        self._result["session_token"] = session_token
        self._update_odom(mean_forward_wheel_millirev)

        if not session_active:
            if self._state != STATE_OBSERVE or self._baseline_ready:
                self._reset_temporal("SESSION_INACTIVE", now_ms)
            self._reject(REJECT_SESSION_INACTIVE)
            self._reason = "SESSION_INACTIVE"
            self._publish()
            return self._result
        if not _is_int(profile_id) or profile_id not in (1, 2, 3):
            self._reset_temporal("PROFILE_UNAVAILABLE", now_ms)
            self._reject(REJECT_PROFILE_UNAVAILABLE)
            self._reason = "PROFILE_UNAVAILABLE"
            self._publish()
            return self._result
        if not frame_fresh:
            self._reject(REJECT_FRAME_INVALID)
            if self._state not in (STATE_OBSERVE, STATE_REACQUIRE):
                self._abort("FRAME_INVALID", now_ms, REJECT_FRAME_INVALID)
            else:
                self._reason = "FRAME_INVALID"
            self._publish()
            return self._result

        cache_ok = self._prepare_cache(
            cache_valid, cache_width, cache_left2, cache_right2)
        if not cache_ok:
            self._reject(REJECT_CACHE_INVALID)
        road_ok = False
        if road_accepted:
            road_ok = self._load_road_rows(
                road_rows, road_count, road_stride, road_candidate_tracks)
            if not road_ok:
                self._reject(REJECT_ROAD_ROWS_INVALID)
        elif single_rows_fresh:
            road_ok = self._load_single_rows(
                row_status, row_edge2, row_contrast)
            if not road_ok:
                self._reject(REJECT_INPUT_INVALID)
        else:
            self._clear_current_rows()
            self._current_valid = False
            self._current_road = False
            if self._state in (STATE_ENTRY_CONFIRM, STATE_ENTER,
                               STATE_INSIDE, STATE_EXIT_CONFIRM,
                               STATE_EXIT_REPAIR):
                self._reject(REJECT_SINGLE_ROWS_STALE)

        self._current_road = bool(road_accepted and road_ok)
        self._current_valid = bool(road_ok)
        self._current_topology_clear = self._topology_clear(topology)
        self._update_side_support()
        self._update_stable_baseline(
            self._current_road, cache_ok, topology)

        ring_enabled = self._profile_id in (2, 3)
        aborted_this_frame = False
        if (ring_enabled and
                self._state not in (STATE_OBSERVE, STATE_REACQUIRE)):
            self._state_frames += 1
            elapsed_ms = _ticks_diff(int(now_ms), self._state_start_ms)
            if elapsed_ms < 0:
                elapsed_ms = 0
            limit_ms = (self.max_entry_ms if self._state in
                        (STATE_ENTRY_CONFIRM, STATE_ENTER) else
                        self.max_active_ms)
            if elapsed_ms > limit_ms:
                self._abort("STATE_TIMEOUT", now_ms, REJECT_STATE_TIMEOUT)
                aborted_this_frame = True
            elif self._odom_limit_exceeded():
                self._abort("ODOM_PROGRESS_LIMIT", now_ms,
                            REJECT_STATE_TIMEOUT)
                aborted_this_frame = True

        inside_target_built = False
        inside_target_attempted = False
        if not ring_enabled:
            # A is explicitly ring-off. Keep the accepted-road baseline alive
            # for the independent cross observer, without running ring states.
            self._reason = "RING_DISABLED_PROFILE_A"
        elif self._state == STATE_OBSERVE:
            self._observe_for_entry(now_ms, topology, cache_ok)
        elif self._state == STATE_ENTRY_CONFIRM:
            self._confirm_entry_drop(now_ms, topology)
        elif self._state == STATE_ENTER:
            self._observe_entry_arc(now_ms, topology)
        elif self._state == STATE_INSIDE:
            inside_target_built = self._build_target_rows(
                mask_info, topology, self._side)
            inside_target_attempted = True
            self._observe_inside_exit(
                now_ms, topology, cache_ok, inside_target_built)
        elif self._state == STATE_EXIT_CONFIRM:
            self._confirm_exit(now_ms, topology, cache_ok)
        elif self._state == STATE_EXIT_REPAIR:
            self._repair_exit(now_ms, topology, cache_ok)
        elif self._state == STATE_REACQUIRE and not aborted_this_frame:
            self._reacquire(now_ms, topology, cache_ok)

        self._update_cross(
            now_ms, self._current_road, road_ok, road_candidate_tracks,
            road_rows, road_count, road_stride, topology, cache_ok,
            mask_info)

        if (ring_enabled and self._ring_cache_frozen and
                self._current_valid):
            if (self._state == STATE_INSIDE and
                    not inside_target_attempted):
                inside_target_built = self._build_target_rows(
                    mask_info, topology, self._side)
                inside_target_attempted = True
            elif self._state == STATE_EXIT_REPAIR:
                exit_target_side = (SIDE_RIGHT if self._side == SIDE_LEFT else
                                    SIDE_LEFT)
                if self._result["target_side"] == SIDE_NONE:
                    self._build_target_rows(
                        mask_info, topology, exit_target_side)
        if (self._state == STATE_INSIDE and
                self._growth_confirmation_latched and
                not self._result["target_valid"]):
            self._result["growth_ready_but_target_rejected"] = True
            self._result["growth_target_rejection"] = (
                self._result["ring_target_rejection"]
                if inside_target_attempted else
                "TARGET_NOT_BUILT")
        self._publish()
        return self._result

    def _clear_current_rows(self):
        _clear_rows_viper(self._current_status, self._current_edge2,
                          self._current_contrast, self._row_seen)

    def _clear_target_output(self):
        result = self._result
        result["target_valid"] = False
        result["target_count"] = 0
        result["target_side"] = SIDE_NONE
        result["target_side_name"] = "NONE"
        result["target_left2"] = 0
        result["target_right2"] = 0
        result["target_center2"] = 0
        result["target_lateral_error"] = 0
        result["observed_rows"] = 0
        result["inferred_rows"] = 0

    def _clear_baseline_rows(self):
        for y in range(self.height):
            self._baseline_row_valid[y] = 0
            self._baseline_left_by_y2[y] = -1
            self._baseline_right_by_y2[y] = -1

    def _clear_frame_output(self):
        result = self._result
        previous_target_count = int(result["target_count"])
        previous_cross_count = int(result["cross_route_count"])
        result["target_valid"] = False
        result["target_count"] = 0
        result["target_side"] = SIDE_NONE
        result["target_left2"] = 0
        result["target_right2"] = 0
        result["target_center2"] = 0
        result["target_lateral_error"] = 0
        result["observed_rows"] = 0
        result["inferred_rows"] = 0
        result["exit_experimental"] = False
        result["exit_reobserved"] = False
        self._ring_diag_arc_rows = 0
        self._ring_diag_anchor_rows = 0
        self._ring_diag_arc_span = 0
        self._ring_diag_anchor_run = 0
        self._ring_diag_arc_rejection = "NONE"
        self._ring_diag_growth_frames = 0
        self._ring_diag_growth_x2 = 0
        self._ring_diag_exit_rows = 0
        self._ring_diag_exit_span = 0
        self._ring_diag_exit_residual = -1
        self._ring_diag_reason_code = 0
        result["ring_arc_rejection"] = "NONE"
        result["ring_arc_growth_frames"] = 0
        result["ring_arc_growth_x2"] = 0
        result["growth_ready_but_target_rejected"] = False
        result["growth_target_rejection"] = "NONE"
        result["ring_target_rejection"] = "NONE"
        result["ring_near_anchor_run"] = 0
        result["cross_candidate"] = False
        result["cross_confirmed"] = False
        result["cross_suppress_slowdown"] = False
        result["cross_route_usable"] = False
        result["cross_route_count"] = 0
        result["cross_reason"] = "NOT_EVALUATED"
        result["cross_rows"] = 0
        result["cross_near_rows"] = 0
        result["cross_far_rows"] = 0
        result["cross_span"] = 0
        result["cross_near_y"] = -1
        result["cross_far_y"] = -1
        result["cross_near_center2"] = 0
        result["cross_far_center2"] = 0
        result["cross_reference_center2"] = 0
        result["cross_reference_y"] = -1
        result["cross_lateral_error"] = 0
        result["cross_open_rows"] = 0
        result["cross_open_segments"] = 0
        result["cross_candidate_tracks"] = 0
        result["reject_reason"] = "NONE"
        if previous_target_count > 0:
            target_limit = previous_target_count * TARGET_ROW_STRIDE
            if target_limit > len(self._target_rows):
                target_limit = len(self._target_rows)
            _clear_i32_prefix_viper(self._target_rows, target_limit)
        if previous_cross_count > 0:
            cross_limit = previous_cross_count * TARGET_ROW_STRIDE
            if cross_limit > len(self._cross_route_rows):
                cross_limit = len(self._cross_route_rows)
            _clear_i32_prefix_viper(self._cross_route_rows, cross_limit)

    def _update_odom(self, value):
        valid = (_is_int(value) and value >= 0 and value <= 0xffffffff)
        if valid and self._last_odom_valid and value < self._last_odom:
            valid = False
            self._last_odom_valid = False
        elif valid:
            self._last_odom = value
            self._last_odom_valid = True
        self._current_odom_valid = bool(valid)
        self._current_odom = value if valid else None
        result = self._result
        result["mean_forward_wheel_millirev"] = self._current_odom
        result["millirev_guard_enabled"] = bool(
            self.max_active_millirev > 0)
        if (valid and self._ring_odom_start_valid and
                value >= self._ring_odom_start):
            result["millirev_delta"] = value - self._ring_odom_start
        else:
            result["millirev_delta"] = None

    def _odom_limit_exceeded(self):
        if self.max_active_millirev <= 0:
            return False
        delta = self._result["millirev_delta"]
        return _is_int(delta) and delta > self.max_active_millirev

    def _reset_temporal(self, reason, now_ms):
        self._route_outcome_count = 0
        self._state = STATE_OBSERVE
        self._reason = reason
        self._side = SIDE_NONE
        self._state_frames = 0
        self._state_start_ms = int(now_ms)
        self._stable_count = 0
        self._baseline_ready = False
        self._baseline_left2 = 0
        self._baseline_right2 = 0
        self._baseline_y = -1
        self._clear_baseline_rows()
        self._release_ring_width()
        self._drop_count = 0
        self._reappear_count = 0
        self._extreme_x2 = 0
        self._reverse_count = 0
        self._exit_count = 0
        self._exit_side = SIDE_NONE
        self._exit_drop_count = 0
        self._exit_reappear_count = 0
        self._repair_count = 0
        self._reacquire_count = 0
        self._cross_count = 0
        self._cross_was_confirmed = False
        self._clear_current_rows()

    def _prepare_cache(self, cache_valid, cache_width,
                       cache_left2, cache_right2):
        try:
            if (len(cache_valid) < self.height or
                    len(cache_width) < self.height or
                    len(cache_left2) < self.height or
                    len(cache_right2) < self.height):
                return False
        except (TypeError, AttributeError):
            return False

        self._active_cache_valid = cache_valid
        self._active_cache_width = cache_width
        self._active_cache_left2 = cache_left2
        self._active_cache_right2 = cache_right2

        packed_counts = _prepare_nearest_cache_viper(
            cache_valid, cache_width, cache_left2, cache_right2,
            self._nearest_cache_y, self.height - self.near_rows)
        valid_count = packed_counts >> 8
        near_count = packed_counts & 255
        nearest = self._nearest_cache_y
        reference_cache = int(nearest[self.reference_y])
        self._cache_valid = (
            valid_count >= self.min_near_support and
            near_count >= self.min_near_support and
            reference_cache >= 0 and
            abs(reference_cache - self.reference_y) <= self.max_cache_gap)
        return self._cache_valid

    def _load_road_rows(self, rows, count, stride, candidate_tracks):
        if (not _is_int(count) or count < self.min_near_support or
                count > self.height or stride != ROAD_ROW_STRIDE or
                not _is_int(candidate_tracks) or candidate_tracks < 1):
            return False
        try:
            if len(rows) < count * ROAD_ROW_STRIDE:
                return False
        except (TypeError, AttributeError):
            return False
        result = _load_road_rows_viper(
            rows, count, self.floor_y, self._current_status,
            self._current_edge2, self._row_seen)
        if result != 1:
            self._clear_current_rows()
            return False
        return True

    def _load_single_rows(self, row_status, row_edge2, row_contrast):
        try:
            if (len(row_status) < 2 * self.height or
                    len(row_edge2) < 2 * self.height or
                    len(row_contrast) < 2 * self.height):
                return False
        except (TypeError, AttributeError):
            return False
        result = _copy_single_rows_viper(
            row_status, row_edge2, row_contrast, self._current_status,
            self._current_edge2, self._current_contrast)
        if result != 1:
            self._clear_current_rows()
            return False
        return True

    def _update_side_support(self):
        summary = self._side_summary
        _measure_side_support_viper(
            self._current_status, self._current_edge2, summary,
            self.height, self.floor_y, self.near_rows, self.reference_y)
        self._near_left_rows = summary[0]
        self._near_right_rows = summary[1]
        self._near_left_run = summary[2]
        self._near_right_run = summary[3]
        self._near_left_ambiguous = summary[4]
        self._near_right_ambiguous = summary[5]
        self._reference_left_y = summary[6]
        self._reference_left2 = summary[7]
        self._reference_right_y = summary[8]
        self._reference_right2 = summary[9]

    def _update_stable_baseline(self, road_ok, cache_ok, topology):
        if self._state not in (STATE_OBSERVE, STATE_REACQUIRE):
            return
        if self._state == STATE_OBSERVE and self._baseline_ready:
            # Keep the first stable straight profile as the arc reference.
            # Updating it every frame would follow ordinary turns and erase
            # the geometric change the ring observer needs to measure.
            return
        if not road_ok:
            # Keep the last fresh, accepted bilateral baseline while the
            # current frame is a raw single-edge observation. That is the
            # evidence needed to detect a one-side dropout.
            return
        clear = self._topology_clear(topology)
        current_near = (
            self._near_left_rows >= self.min_near_support and
            self._near_right_rows >= self.min_near_support and
            self._near_left_run >= self.min_near_contiguous and
            self._near_right_run >= self.min_near_contiguous and
            self._near_left_ambiguous == 0 and
            self._near_right_ambiguous == 0)
        if not (cache_ok and clear and current_near):
            if self._state == STATE_OBSERVE:
                self._stable_count = 0
                self._baseline_ready = False
            return

        if self._stable_count < 255:
            self._stable_count += 1
        ref_y = self._reference_left_y
        if (ref_y >= 0 and ref_y == self._reference_right_y and
                abs(ref_y - self.reference_y) <= 4):
            left2 = int(self._current_edge2[ref_y])
            right2 = int(self._current_edge2[self.height + ref_y])
            if left2 < right2:
                self._baseline_left2 = left2
                self._baseline_right2 = right2
                self._baseline_y = ref_y
                baseline_rows = 0
                for y in range(self.floor_y, self.height):
                    self._baseline_row_valid[y] = 0
                    self._baseline_left_by_y2[y] = -1
                    self._baseline_right_by_y2[y] = -1
                    if (self._current_status[y] == 1 and
                            self._current_status[self.height + y] == 1):
                        row_left2 = int(self._current_edge2[y])
                        row_right2 = int(
                            self._current_edge2[self.height + y])
                        if row_left2 < row_right2:
                            self._baseline_row_valid[y] = 1
                            self._baseline_left_by_y2[y] = row_left2
                            self._baseline_right_by_y2[y] = row_right2
                            baseline_rows += 1
                if (self._stable_count >= self.min_stable_frames and
                        baseline_rows >= self.min_target_rows):
                    self._baseline_ready = True

    def _topology_clear(self, topology):
        if not isinstance(topology, dict):
            return False
        if (topology.get("ambiguous") or topology.get("branch_evidence") or
                topology.get("branch") or topology.get("island_evidence") or
                topology.get("island")):
            return False
        reason = topology.get("reason", "NOT_RUN")
        return reason == "OK" and bool(topology.get("complete"))

    def _topology_branch(self, topology):
        return bool(isinstance(topology, dict) and
                    (topology.get("branch_evidence") or
                     topology.get("branch")))

    def _opposite_matches_baseline(self, ring_side):
        if ring_side == SIDE_LEFT:
            edge2 = self._reference_right2
            edge_y = self._reference_right_y
            baseline2 = self._baseline_right2
        else:
            edge2 = self._reference_left2
            edge_y = self._reference_left_y
            baseline2 = self._baseline_left2
        if edge_y < 0 or edge_y != self._baseline_y:
            return False
        return abs(edge2 - baseline2) <= self.opposite_position_tolerance2

    def _entry_dropout_evidence(self, ring_side):
        """Return one side's status from the shared mirrored row scan."""
        if ring_side not in (SIDE_LEFT, SIDE_RIGHT):
            return 0, 0
        self._measure_entry_dropout()
        summary = self._entry_dropout_summary
        if ring_side == SIDE_LEFT:
            missing = summary[0]
            longest = summary[1]
            anchor_rows = summary[2]
            anchor_run = summary[6]
        else:
            missing = summary[3]
            longest = summary[4]
            anchor_rows = summary[5]
            anchor_run = summary[7]
        accepted = int(longest >= 4 and missing >= 4 and
                       anchor_rows >= self.ring_entry_min_near_support and
                       anchor_run >= self.ring_entry_min_near_contiguous)
        if accepted:
            self._ring_diag_arc_rows = missing
            self._ring_diag_anchor_rows = anchor_rows
            self._ring_diag_anchor_run = anchor_run
        return accepted, missing, anchor_rows

    def _find_entry_dropout_side(self):
        self._measure_entry_dropout()
        summary = self._entry_dropout_summary
        left_missing = summary[0]
        left_longest = summary[1]
        left_anchor = summary[2]
        left_anchor_run = summary[6]
        right_missing = summary[3]
        right_longest = summary[4]
        right_anchor = summary[5]
        right_anchor_run = summary[7]
        left_ok = (left_longest >= 4 and left_missing >= 4 and
                   left_anchor >= self.ring_entry_min_near_support and
                   left_anchor_run >= self.ring_entry_min_near_contiguous)
        right_ok = (right_longest >= 4 and right_missing >= 4 and
                    right_anchor >= self.ring_entry_min_near_support and
                    right_anchor_run >= self.ring_entry_min_near_contiguous)
        if left_ok and right_ok:
            self._ring_diag_arc_rows = 0
            self._ring_diag_anchor_rows = 0
            return SIDE_NONE
        if left_ok:
            self._ring_diag_arc_rows = left_missing
            self._ring_diag_anchor_rows = left_anchor
            self._ring_diag_anchor_run = left_anchor_run
            return SIDE_LEFT
        if right_ok:
            self._ring_diag_arc_rows = right_missing
            self._ring_diag_anchor_rows = right_anchor
            self._ring_diag_anchor_run = right_anchor_run
            return SIDE_RIGHT
        self._ring_diag_arc_rows = 0
        self._ring_diag_anchor_rows = max(left_anchor, right_anchor)
        self._ring_diag_anchor_run = max(left_anchor_run, right_anchor_run)
        return SIDE_NONE

    def _measure_entry_dropout(self):
        self._measure_outer_anchor_shift(SIDE_LEFT)
        self._measure_outer_anchor_shift(SIDE_RIGHT)
        _measure_entry_dropout_viper(
            self._current_status, self._current_edge2,
            self._baseline_row_valid, self._baseline_left_by_y2,
            self._baseline_right_by_y2, self._entry_dropout_summary,
            self._outer_shift_x2, self._outer_shift_valid,
            self.height, self.floor_y, self.near_rows,
            self.ring_entry_reappear_shift2,
            self.opposite_position_tolerance2)

    @micropython.native
    def _measure_outer_anchor_shift(self, outer_side):
        """Estimate shared lateral translation from stable near outer rows."""
        if outer_side not in (SIDE_LEFT, SIDE_RIGHT):
            return False
        slot = outer_side
        self._outer_shift_valid[slot] = 0
        self._outer_shift_x2[slot] = 0
        self._outer_anchor_rows[slot] = 0
        self._outer_anchor_run[slot] = 0
        if ((outer_side == SIDE_LEFT and self._near_left_ambiguous) or
                (outer_side == SIDE_RIGHT and self._near_right_ambiguous)):
            return False
        base = outer_side * self.height
        start = max(self.floor_y, self.height - self.near_rows)
        count = 0
        for y in range(start, self.height):
            if (self._baseline_row_valid[y] and
                    self._current_status[base + y] == 1):
                if outer_side == SIDE_LEFT:
                    delta = (int(self._current_edge2[base + y]) -
                             int(self._baseline_left_by_y2[y]))
                else:
                    delta = (int(self._current_edge2[base + y]) -
                             int(self._baseline_right_by_y2[y]))
                self._outer_shift_samples[count] = delta
                count += 1
        if count < self.ring_entry_min_near_support:
            return False
        _sort_prefix(self._outer_shift_samples, count)
        median = self._outer_shift_samples[count // 2]
        support = 0
        run = 0
        longest = 0
        for y in range(start, self.height):
            if (self._baseline_row_valid[y] and
                    self._current_status[base + y] == 1):
                if outer_side == SIDE_LEFT:
                    delta = (int(self._current_edge2[base + y]) -
                             int(self._baseline_left_by_y2[y]))
                else:
                    delta = (int(self._current_edge2[base + y]) -
                             int(self._baseline_right_by_y2[y]))
                if abs(delta - median) <= self.opposite_position_tolerance2:
                    support += 1
                    run += 1
                    if run > longest:
                        longest = run
                else:
                    run = 0
            else:
                run = 0
        self._outer_anchor_rows[slot] = support
        self._outer_anchor_run[slot] = longest
        if (support < self.ring_entry_min_near_support or
                longest < self.ring_entry_min_near_contiguous):
            return False
        self._outer_shift_x2[slot] = median
        self._outer_shift_valid[slot] = 1
        return True

    @micropython.native
    def _entry_arc_profile(self, ring_side):
        """Measure a continuous observed inner arc with a stable near anchor."""
        self._ring_diag_arc_rejection = "NONE"
        if ring_side not in (SIDE_LEFT, SIDE_RIGHT):
            self._ring_diag_arc_rejection = "NO_RING_SIDE"
            return 0, 0, 0, 0, 0
        inner_base = ring_side * self.height
        outer_side = SIDE_RIGHT if ring_side == SIDE_LEFT else SIDE_LEFT
        outer_base = outer_side * self.height
        outer_anchor_ok = self._measure_outer_anchor_shift(outer_side)
        outward_rows = 0
        run = 0
        max_run = 0
        first_y = -1
        last_y = -1
        peak = 0
        peak_edge2 = 0
        ambiguous_outer = False
        invalid_outer = False
        for y in range(self.floor_y, 48):
            if not self._baseline_row_valid[y]:
                run = 0
                continue
            inner_status = self._current_status[inner_base + y]
            outer_status = self._current_status[outer_base + y]
            if inner_status != 1:
                run = 0
                continue
            if outer_status == 2:
                # Ambiguous counterpart evidence is never treated as a crop.
                ambiguous_outer = True
                run = 0
                continue
            if outer_status not in (0, 1):
                invalid_outer = True
                run = 0
                continue
            inner_baseline = int(
                self._baseline_left_by_y2[y] if ring_side == SIDE_LEFT else
                self._baseline_right_by_y2[y])
            outer_baseline = int(
                self._baseline_right_by_y2[y] if
                outer_side == SIDE_RIGHT else
                self._baseline_left_by_y2[y])
            inner_edge = int(self._current_edge2[inner_base + y])
            if outer_status == 1:
                outer_edge = int(self._current_edge2[outer_base + y])
                correctly_ordered = (inner_edge < outer_edge if
                                     ring_side == SIDE_LEFT else
                                     outer_edge < inner_edge)
                outer_delta = outer_edge - outer_baseline
                if not correctly_ordered:
                    run = 0
                    continue
            else:
                if not outer_anchor_ok:
                    run = 0
                    continue
                outer_delta = int(self._outer_shift_x2[outer_side])
                outer_edge = outer_baseline + outer_delta
                correctly_ordered = (inner_edge < outer_edge if
                                     ring_side == SIDE_LEFT else
                                     outer_edge < inner_edge)
                if not correctly_ordered:
                    run = 0
                    continue
            outward = (inner_edge - inner_baseline if ring_side == SIDE_LEFT
                       else inner_baseline - inner_edge)
            if ring_side == SIDE_LEFT:
                outward -= outer_delta
            else:
                outward += outer_delta
            minimum_outward = (self.ring_entry_reappear_shift2 if
                               self._reappear_count == 0 else 1)
            if outward < minimum_outward:
                run = 0
                continue
            outward_rows += 1
            run += 1
            if first_y < 0:
                first_y = y
            last_y = y
            if run > max_run:
                max_run = run
            if outward > peak:
                peak = outward
                peak_edge2 = inner_edge
        span = last_y - first_y if first_y >= 0 else 0
        near_anchor = 0
        near_run = 0
        max_near_run = 0
        for y in range(max(self.floor_y, self.height - self.near_rows),
                        self.height):
            if (self._baseline_row_valid[y] and
                    self._current_status[outer_base + y] == 1 and
                    outer_anchor_ok and
                    abs((int(self._current_edge2[outer_base + y]) -
                         (int(self._baseline_right_by_y2[y])
                          if outer_side == SIDE_RIGHT else
                          int(self._baseline_left_by_y2[y]))) -
                        int(self._outer_shift_x2[outer_side])) <=
                    self.ring_entry_opposite_tolerance2):
                near_anchor += 1
                near_run += 1
                if near_run > max_near_run:
                    max_near_run = near_run
            else:
                near_run = 0
        self._ring_diag_arc_rows = outward_rows
        self._ring_diag_anchor_rows = near_anchor
        self._ring_diag_anchor_run = max_near_run
        if invalid_outer:
            self._ring_diag_arc_rejection = "OUTER_STATUS_INVALID"
            return outward_rows, peak, peak_edge2, span, 0
        if ambiguous_outer:
            self._ring_diag_arc_rejection = "OUTER_AMBIGUOUS"
            return outward_rows, peak, peak_edge2, span, 0
        if not outer_anchor_ok:
            self._ring_diag_arc_rejection = "OPPOSITE_NEAR_SUPPORT"
            return outward_rows, peak, peak_edge2, span, 0
        if (outward_rows < self.ring_arc_min_rows or
                max_run < self.ring_arc_min_contiguous_rows or
                span < self.ring_arc_min_span_rows):
            self._ring_diag_arc_rejection = "INNER_ARC_SUPPORT"
            return outward_rows, peak, peak_edge2, span, 0
        if (near_anchor < self.ring_entry_min_near_support or
                max_near_run < self.ring_entry_min_near_contiguous or
                (outer_side == SIDE_LEFT and
                 self._near_left_ambiguous != 0) or
                (outer_side == SIDE_RIGHT and
                 self._near_right_ambiguous != 0)):
            self._ring_diag_arc_rejection = "OPPOSITE_NEAR_SUPPORT"
            return outward_rows, peak, peak_edge2, span, 0
        return outward_rows, peak, peak_edge2, span, 1

    def _freeze_ring_width(self):
        calibrated = bool(self._calibration_ready and
                          self._calibration_width_valid is not None and
                          self._calibration_width_cells is not None)
        if self.require_calibration:
            if not calibrated or not self._cal_check_ready:
                return False
            source_valid = self._calibration_width_valid
            source_width = self._calibration_width_cells
        elif calibrated:
            source_valid = self._calibration_width_valid
            source_width = self._calibration_width_cells
        else:
            source_valid = self._active_cache_valid
            source_width = self._active_cache_width
        if source_valid is None or source_width is None:
            return False
        valid = self._ring_width_valid
        widths = self._ring_width
        nearest = self._ring_nearest_y
        count = 0
        near_count = 0
        last = -1
        near_start = self.height - self.near_rows
        for y in range(self.height):
            valid[y] = 0
            widths[y] = 0
            nearest[y] = -1
            if source_valid[y] != 0:
                width = int(source_width[y])
                if 8 <= width <= 70:
                    valid[y] = 1
                    widths[y] = width
                    last = y
                    count += 1
                    if y >= near_start:
                        near_count += 1
            nearest[y] = last
        last = -1
        for y in range(self.height - 1, -1, -1):
            if valid[y] != 0:
                last = y
            lower = int(nearest[y])
            if last >= 0 and (lower < 0 or last - y < y - lower):
                nearest[y] = last
        reference_y = int(nearest[self.reference_y])
        if (count < self.min_target_rows or
                near_count < self.min_near_support or reference_y < 0 or
                abs(reference_y - self.reference_y) > self.max_cache_gap):
            self._release_ring_width()
            return False
        self._ring_cache_frozen = True
        self._ring_width_calibrated = calibrated
        self._result["ring_width_source"] = (
            "CALIBRATED_45CM_ROW_MAP" if calibrated else
            "FIXTURE_CACHE_ONLY")
        return True

    def _release_ring_width(self):
        self._ring_cache_frozen = False
        self._ring_width_calibrated = False
        if hasattr(self, "_result"):
            self._result["ring_width_source"] = "NONE"
        for y in range(self.height):
            self._ring_width_valid[y] = 0
            self._ring_width[y] = 0
            self._ring_nearest_y[y] = -1
        self._ring_odom_start = None
        self._ring_odom_start_valid = False
        if hasattr(self, "_result"):
            self._result["millirev_delta"] = None

    def _observe_for_entry(self, now_ms, topology, cache_ok):
        self._reason = "OBSERVING"
        if self.require_calibration and not (
                self._calibration_ready and self._cal_check_ready):
            self._ring_diag_reason_code = 1
            self._reason = "RING_CALIBRATION_NOT_READY"
            return
        if not (self._baseline_ready and cache_ok):
            self._ring_diag_reason_code = 2
            self._reason = "OBSERVING"
            return
        if not self._current_valid:
            self._ring_diag_reason_code = 3
            self._reject(REJECT_SINGLE_ROWS_STALE)
            self._reason = "ROW_INPUT_INVALID"
            return
        if (self._near_left_ambiguous or self._near_right_ambiguous or
                self._topology_branch(topology) or
                not self._topology_clear(topology)):
            self._ring_diag_reason_code = 4
            self._reject(REJECT_TOPOLOGY_AMBIGUOUS)
            self._reason = "TOPOLOGY_NOT_CLEAR"
            return
        if self._ordinary_corridor_frame(False):
            self._reason = "ORDINARY_CORRIDOR_VETO"
            self._ring_diag_reason_code = 5
            return

        side = self._find_entry_dropout_side()
        if side == SIDE_NONE:
            self._ring_diag_reason_code = 5
            self._reason = "NO_MIDFAR_DROPOUT_WITH_NEAR_ANCHOR"
            return
        self._side = side
        if not self._freeze_ring_width():
            self._side = SIDE_NONE
            self._reject(REJECT_CACHE_INVALID)
            self._reason = "NO_FROZEN_CALIBRATED_WIDTH_PROFILE"
            self._ring_diag_reason_code = 1
            return
        self._ring_odom_start_valid = self._current_odom_valid
        self._ring_odom_start = (self._current_odom if
                                 self._current_odom_valid else None)
        self._drop_count = 1
        self._state = STATE_ENTRY_CONFIRM
        self._reason = "ONE_SIDE_LOST_CANDIDATE"
        self._ring_diag_reason_code = 6
        self._state_start_ms = int(now_ms)
        self._state_frames = 0
        self._record_event(EVENT_ENTRY_CANDIDATE,
                           REJECT_SIDE_SUPPORT,
                           self._baseline_left2 if self._side == SIDE_LEFT
                           else self._baseline_right2, now_ms)

    def _confirm_entry_drop(self, now_ms, topology):
        self._reason = "CONFIRMING_ONE_SIDE_LOSS"
        if not self._current_valid:
            self._reject(REJECT_SINGLE_ROWS_STALE)
            return
        if not self._topology_clear(topology):
            self._abort("ENTRY_TOPOLOGY_AMBIGUOUS", now_ms,
                        REJECT_TOPOLOGY_AMBIGUOUS)
            return
        if self._ordinary_corridor_frame(False):
            self._reset_to_observe("ORDINARY_CORRIDOR_VETO", now_ms)
            return
        missing, missing_rows, anchor_rows = \
            self._entry_dropout_evidence(self._side)
        self._ring_diag_arc_rows = missing_rows
        self._ring_diag_anchor_rows = anchor_rows
        if missing:
            self._drop_count += 1
            if self._drop_count >= self.min_drop_frames:
                self._set_state(STATE_ENTER, "DROPOUT_CONFIRMED", now_ms)
                self._ring_diag_reason_code = 7
            return
        # Reappearance without the required ring motion is an ordinary
        # recovery from a one-frame detector miss.
        self._reset_to_observe("DROPOUT_NOT_SUSTAINED", now_ms)
        self._reject(REJECT_RING_SEQUENCE)

    def _reset_entry_growth(self):
        self._entry_growth_frames = 0
        self._entry_growth_origin_x2 = -1
        self._entry_growth_last_x2 = -1
        self._ring_diag_growth_frames = 0
        self._ring_diag_growth_x2 = 0
        self._reverse_count = 0
        self._growth_confirmation_latched = False
        self._entry_confirmation_path = "NONE"

    def _start_entry_growth(self, outward):
        self._entry_growth_frames = 1
        self._entry_growth_origin_x2 = int(outward)
        self._entry_growth_last_x2 = int(outward)
        self._ring_diag_growth_frames = 1
        self._ring_diag_growth_x2 = 0

    def _observe_entry_arc(self, now_ms, topology):
        self._reason = "WAITING_FOR_RING_ARC"
        if not self._current_valid:
            self._reset_entry_growth()
            self._ring_diag_arc_rejection = "CURRENT_ROWS_INVALID"
            self._reject(REJECT_SINGLE_ROWS_STALE)
            return
        if not self._topology_clear(topology):
            self._reset_entry_growth()
            self._abort("ENTRY_TOPOLOGY_AMBIGUOUS", now_ms,
                        REJECT_TOPOLOGY_AMBIGUOUS)
            return
        if self._ordinary_corridor_frame(False):
            self._reset_to_observe("ORDINARY_CORRIDOR_VETO", now_ms)
            return
        if (not self._ring_cache_frozen or
                (self.require_calibration and
                 not (self._calibration_ready and
                      self._cal_check_ready and
                      self._ring_width_calibrated))):
            self._reset_entry_growth()
            self._ring_diag_arc_rejection = "WIDTH_NOT_READY"
            self._reject(REJECT_CACHE_INVALID)
            return
        rows, outward, edge2, span, arc_ok = self._entry_arc_profile(
            self._side)
        self._ring_diag_arc_rows = rows
        self._ring_diag_arc_span = span
        self._ring_diag_reason_code = 8
        if not arc_ok:
            self._reset_entry_growth()
            self._reject(REJECT_SIDE_SUPPORT)
            return
        if self._reappear_count == 0:
            self._reappear_count = 1
            self._extreme_x2 = outward
            self._reverse_count = 0
            self._start_entry_growth(outward)
            return

        if self._entry_growth_frames <= 0:
            self._start_entry_growth(outward)
        else:
            growth = outward - self._entry_growth_last_x2
            if growth >= 0:
                if (self._entry_growth_frames <
                        self.ring_arc_growing_support_frames):
                    self._entry_growth_frames += 1
                self._entry_growth_last_x2 = outward
                self._ring_diag_growth_frames = self._entry_growth_frames
                self._ring_diag_growth_x2 = max(
                    0, outward - self._entry_growth_origin_x2)
            else:
                # A negative step starts a new consecutive evidence run.
                self._start_entry_growth(outward)

        if (self._entry_growth_frames >=
                self.ring_arc_growing_support_frames and
                self._ring_diag_growth_x2 >= self.ring_arc_min_growth2):
            self._set_state(STATE_INSIDE, "GROWING_ARC_CONFIRMED", now_ms)
            self._growth_confirmation_latched = True
            self._entry_confirmation_path = "GROWING_ARC"
            self._ring_diag_reason_code = 9
            self._record_event(EVENT_RING_CONFIRMED, REJECT_NONE, edge2,
                               now_ms)
            return

        if outward > self._extreme_x2:
            self._extreme_x2 = outward
            self._reverse_count = 0
            return
        reversal = self._extreme_x2 - outward
        if reversal >= self.peak_reversal2:
            self._reverse_count += 1
        else:
            self._reverse_count = 0
        if self._reverse_count >= self.peak_confirm_frames:
            self._set_state(STATE_INSIDE, "RING_ARC_CONFIRMED", now_ms)
            self._entry_confirmation_path = "PEAK_REVERSAL"
            self._ring_diag_reason_code = 9
            self._record_event(EVENT_RING_CONFIRMED, REJECT_NONE, edge2,
                               now_ms)

    def _ring_topology_precheck(self, topology):
        if not isinstance(topology, dict) or not topology.get("complete"):
            return False
        if (topology.get("ambiguous") or topology.get("branch_evidence") or
                topology.get("branch") or
                topology.get("independent_branch_ambiguous") or
                topology.get("multi_branch_ambiguous") or
                topology.get("multiple_exits")):
            return False
        has_island = bool(topology.get("island_evidence") or
                          topology.get("island"))
        return has_island or topology.get("reason") == "OK"

    def _ring_topology_safe(self, topology, route_ok):
        if not self._ring_topology_precheck(topology):
            return False
        has_island = bool(topology.get("island_evidence") or
                          topology.get("island"))
        if has_island:
            # A ring may contain its own dark island. Accept that topology only
            # when this frame has a real locked-side route and its inner half
            # remains known road all the way to the computed centerline.
            return bool(route_ok and self._result["target_valid"])
        return True

    def _ring_boundary_dropout(self, missing_side, anchor_side):
        base = missing_side * self.height
        longest = 0
        run = 0
        missing_rows = 0
        for y in range(self.floor_y, 48):
            if not self._baseline_row_valid[y]:
                run = 0
                continue
            if self._current_status[base + y] != 1:
                run += 1
                missing_rows += 1
                if run > longest:
                    longest = run
            else:
                run = 0
        anchor_base = anchor_side * self.height
        anchor_rows = 0
        for y in range(self.height - self.near_rows, self.height):
            baseline = int(
                self._baseline_left_by_y2[y] if anchor_side == SIDE_LEFT else
                self._baseline_right_by_y2[y])
            if (self._baseline_row_valid[y] and
                    self._current_status[anchor_base + y] == 1 and
                    abs(int(self._current_edge2[anchor_base + y]) - baseline) <=
                    self.opposite_position_tolerance2):
                anchor_rows += 1
        return (missing_rows >= 4 and longest >= 4 and
                anchor_rows >= self.min_near_support,
                missing_rows, anchor_rows)

    def _exit_profile_fit(self, topology):
        if (not self._current_valid or not self._topology_clear(topology) or
                self._side not in (SIDE_LEFT, SIDE_RIGHT)):
            return False
        actual_side = SIDE_RIGHT if self._side == SIDE_LEFT else SIDE_LEFT
        base = actual_side * self.height
        rows = 0
        run = 0
        longest = 0
        min_y = self.height
        max_y = -1
        max_residual = 0
        for y in range(self.floor_y, self.height):
            if (not self._baseline_row_valid[y] or
                    self._current_status[base + y] != 1):
                run = 0
                continue
            baseline = int(
                self._baseline_right_by_y2[y] if actual_side == SIDE_RIGHT
                else self._baseline_left_by_y2[y])
            residual = abs(int(self._current_edge2[base + y]) - baseline)
            if residual > max_residual:
                max_residual = residual
            if residual > 4:
                run = 0
                continue
            rows += 1
            run += 1
            if run > longest:
                longest = run
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
        span = max_y - min_y if rows else 0
        self._ring_diag_exit_rows = rows
        self._ring_diag_exit_span = span
        self._ring_diag_exit_residual = max_residual
        if rows < 8 or longest < 8 or span < 12 or max_residual > 4:
            return False
        return True

    def _stable_exit_frame(self, topology, cache_ok):
        return self._exit_profile_fit(topology)

    def _paired_aligned_frame(self, topology):
        if (not self._current_road or not self._current_valid or
                not self._topology_clear(topology) or
                self._near_left_rows < self.min_near_support or
                self._near_right_rows < self.min_near_support or
                self._near_left_run < self.min_near_contiguous or
                self._near_right_run < self.min_near_contiguous or
                self._near_left_ambiguous or self._near_right_ambiguous):
            return False
        rows = 0
        min_y = self.height
        max_y = -1
        for y in range(self.floor_y, self.height):
            if (not self._baseline_row_valid[y] or
                    self._current_status[y] != 1 or
                    self._current_status[self.height + y] != 1):
                continue
            left2 = int(self._current_edge2[y])
            right2 = int(self._current_edge2[self.height + y])
            if (left2 >= right2 or
                    abs(left2 - self._baseline_left_by_y2[y]) > 4 or
                    abs(right2 - self._baseline_right_by_y2[y]) > 4):
                continue
            if self._calibration_ready:
                width = int(self._calibration_width_cells[y])
                measured = (right2 - left2) // 2
                if width < 8 or abs(measured - width) * 5 > width:
                    continue
            rows += 1
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
        return rows >= 8 and max_y - min_y >= 12

    @micropython.native
    def _ordinary_corridor_frame(self, use_frozen_width):
        """Check accepted bilateral geometry without baseline alignment."""
        if (not self._current_road or not self._current_valid or
                not self._current_topology_clear or
                self._near_left_rows < self.min_near_support or
                self._near_right_rows < self.min_near_support or
                self._near_left_run < self.min_near_contiguous or
                self._near_right_run < self.min_near_contiguous or
                self._near_left_ambiguous or self._near_right_ambiguous):
            return False
        if self.require_calibration and not (
                self._calibration_ready and self._cal_check_ready):
            return False
        # Production calibration owns exact typed buffers. Preserve the
        # object-safe reference path for list/dict calibration adapters.
        if self.require_calibration:
            valid = self._calibration_width_valid
            widths = self._calibration_width_cells
            width_length = len(widths) if widths is not None else -1
            if (widths is not self._corridor_width_owner or
                    width_length != self._corridor_width_length):
                self._corridor_width_owner = widths
                self._corridor_width_length = width_length
                # MicroPython omits array.typecode on this port. Exact array
                # type plus repr identifies signed i; available view metadata
                # must agree. This is classified only on owner/length changes.
                fast = False
                if type(widths) is array and width_length == self.height:
                    view = memoryview(widths)
                    typecode = getattr(widths, "typecode", "i")
                    itemsize = getattr(view, "itemsize", 4)
                    nbytes = getattr(view, "nbytes", self.height * 4)
                    fast = (typecode == "i" and itemsize == 4 and
                            nbytes == self.height * 4 and
                            repr(widths).startswith("array('i',"))
                self._corridor_width_fast = fast
            if (type(valid) is bytearray and len(valid) == self.height and
                    self._corridor_width_fast):
                return bool(_ordinary_calibrated_corridor_viper(
                    self._current_status, self._current_edge2, valid, widths,
                    self._ring_width_valid, self._ring_width,
                    1 if use_frozen_width else 0, self.floor_y, self.height))
        rows = 0
        min_y = self.height
        max_y = -1
        for y in range(self.floor_y, self.height):
            if (self._current_status[y] != 1 or
                    self._current_status[self.height + y] != 1):
                continue
            left2 = int(self._current_edge2[y])
            right2 = int(self._current_edge2[self.height + y])
            if left2 >= right2:
                continue
            if (self.require_calibration and
                    self._calibration_width_valid[y]):
                width = int(self._calibration_width_cells[y])
            elif (use_frozen_width and self._ring_width_valid[y]):
                width = int(self._ring_width[y])
            elif (not self.require_calibration and
                  self._active_cache_valid is not None and
                  self._active_cache_width is not None and
                  self._active_cache_valid[y]):
                width = int(self._active_cache_width[y])
            else:
                continue
            measured = (right2 - left2) // 2
            if width < 8 or abs(measured - width) * 5 > width:
                continue
            rows += 1
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
        return rows >= 8 and max_y - min_y >= 12

    @micropython.native
    def _ordinary_reacquired_frame(self):
        """Check current ordinary route against the active ring width source."""
        if self.require_calibration and not self._ring_width_calibrated:
            return False
        return self._ordinary_corridor_frame(True)

    @micropython.native
    def observe_route_outcome(self, now_ms, ring_route_selected,
                              ordinary_route_valid):
        """Abort a confirmed false ring only after three fresh ordinary routes.

        `ordinary_route_valid` must already mean the caller's standard
        bilateral selection passed its normal route and boundary checks.
        Returning True lets the caller restore that same-frame selection.
        """
        frame = self._frame_index
        if self._route_outcome_last_frame == frame:
            return False
        previous = self._route_outcome_last_frame
        self._route_outcome_last_frame = frame
        if previous >= 0 and frame != ((previous + 1) & 0x7fffffff):
            self._route_outcome_count = 0

        if (self._profile_id not in (2, 3) or not self._session_active or
                not self._result.get("ring_confirmed") or
                self._state not in (STATE_INSIDE, STATE_EXIT_CONFIRM,
                                    STATE_EXIT_REPAIR) or
                ring_route_selected or not ordinary_route_valid or
                not self._ordinary_reacquired_frame()):
            self._route_outcome_count = 0
            return False

        arc_rows, peak, edge2, span, arc_ok = self._entry_arc_profile(
            self._side)
        if arc_ok:
            self._route_outcome_count = 0
            self._publish()
            return False
        if self._route_outcome_count < 3:
            self._route_outcome_count += 1
        if self._route_outcome_count < 3:
            self._publish()
            return False

        self._route_outcome_count = 0
        self._abort("ORDINARY_ROUTE_REACQUIRED", now_ms,
                    REJECT_RING_SEQUENCE)
        self._clear_target_output()
        self._publish()
        return True

    def _observe_inside_exit(self, now_ms, topology, cache_ok, route_ok):
        self._reason = "RING_INSIDE_EXIT_UNOBSERVED"
        self._ring_diag_reason_code = 10
        if not self._ring_topology_safe(topology, route_ok):
            self._reject(REJECT_TOPOLOGY_AMBIGUOUS)
            return
        if not self._current_valid:
            self._reject(REJECT_SINGLE_ROWS_STALE)
            return
        actual_exit_side = (SIDE_RIGHT if self._side == SIDE_LEFT else
                            SIDE_LEFT)
        missing, missing_rows, anchor_rows = self._ring_boundary_dropout(
            actual_exit_side, self._side)
        self._ring_diag_arc_rows = missing_rows
        self._ring_diag_anchor_rows = anchor_rows
        if missing:
            if self._exit_side not in (SIDE_NONE, actual_exit_side):
                self._exit_drop_count = 0
            self._exit_side = actual_exit_side
            self._exit_drop_count += 1
            if self._exit_drop_count >= self.exit_drop_frames:
                self._set_state(STATE_EXIT_CONFIRM,
                                "EXIT_OPPOSITE_SIDE_DROPOUT_OBSERVED", now_ms)
                self._exit_reappear_count = 0
                self._result["exit_experimental"] = True
                self._ring_diag_reason_code = 10
                self._clear_target_output()
            return
        self._exit_drop_count = 0

    def _confirm_exit(self, now_ms, topology, cache_ok):
        self._reason = "EXIT_EXPERIMENTAL_CONFIRM"
        self._ring_diag_reason_code = 11
        self._result["exit_experimental"] = True
        if self._stable_exit_frame(topology, cache_ok):
            self._exit_reappear_count += 1
            if self._exit_reappear_count >= self.exit_confirm_frames:
                self._set_state(STATE_EXIT_REPAIR,
                                "EXIT_REOBSERVED_REPAIR_PENDING", now_ms)
                self._repair_count = 0
                self._result["exit_experimental"] = True
                self._ring_diag_reason_code = 11
        else:
            self._exit_reappear_count = 0
            self._result["exit_experimental"] = True

    def _repair_exit(self, now_ms, topology, cache_ok):
        self._reason = "EXIT_EXPERIMENTAL_REOBSERVATION"
        self._result["exit_experimental"] = True
        self._ring_diag_reason_code = 11
        if self._paired_aligned_frame(topology):
            self._set_state(STATE_REACQUIRE,
                            "EXIT_VISUALLY_REOBSERVED", now_ms)
            self._reacquire_count = 1
            self._record_event(EVENT_EXIT_REOBSERVED, REJECT_NONE, 0, now_ms)
            self._result["exit_experimental"] = True
            self._result["exit_reobserved"] = True

    def _reacquire(self, now_ms, topology, cache_ok):
        self._reason = "WAITING_FOR_NORMAL_ROAD_REACQUIRE"
        self._ring_diag_reason_code = 12
        if self._paired_aligned_frame(topology):
            self._reacquire_count += 1
            if self._reacquire_count >= self.reacquire_frames:
                self._reset_to_observe("NORMAL_ROAD_REACQUIRED", now_ms)
        else:
            self._reacquire_count = 0

    def _reset_to_observe(self, reason, now_ms):
        self._route_outcome_count = 0
        self._state = STATE_OBSERVE
        self._reason = reason
        self._side = SIDE_NONE
        self._state_frames = 0
        self._state_start_ms = int(now_ms)
        self._stable_count = 0
        self._baseline_ready = False
        self._baseline_left2 = 0
        self._baseline_right2 = 0
        self._baseline_y = -1
        self._clear_baseline_rows()
        self._release_ring_width()
        self._drop_count = 0
        self._reappear_count = 0
        self._extreme_x2 = 0
        self._reverse_count = 0
        self._reset_entry_growth()
        self._exit_count = 0
        self._exit_side = SIDE_NONE
        self._exit_drop_count = 0
        self._exit_reappear_count = 0
        self._repair_count = 0
        self._reacquire_count = 0
        self._cross_count = 0

    def _set_state(self, state, reason, now_ms):
        if state == STATE_ENTER:
            self._reset_entry_growth()
        elif state != STATE_INSIDE:
            self._growth_confirmation_latched = False
            self._entry_confirmation_path = "NONE"
        self._state = state
        self._reason = reason
        self._state_frames = 0
        self._state_start_ms = int(now_ms)
        if state in (STATE_OBSERVE, STATE_REACQUIRE):
            self._exit_count = 0
            self._release_ring_width()

    def _abort(self, reason, now_ms, reject_code):
        self._route_outcome_count = 0
        self._reject(reject_code)
        self._state = STATE_REACQUIRE
        self._reason = reason
        self._side = SIDE_NONE
        self._state_frames = 0
        self._state_start_ms = int(now_ms)
        self._stable_count = 0
        self._baseline_ready = False
        self._baseline_left2 = 0
        self._baseline_right2 = 0
        self._baseline_y = -1
        self._clear_baseline_rows()
        self._release_ring_width()
        self._drop_count = 0
        self._reappear_count = 0
        self._extreme_x2 = 0
        self._reverse_count = 0
        self._reset_entry_growth()
        self._exit_count = 0
        self._exit_side = SIDE_NONE
        self._exit_drop_count = 0
        self._exit_reappear_count = 0
        self._repair_count = 0
        self._reacquire_count = 0
        self._cross_count = 0
        self._record_event(EVENT_ABORT, reject_code, 0, now_ms)

    def _build_target_rows(self, mask_info, topology, target_side):
        self._result["ring_target_rejection"] = "NONE"
        if (not self._ring_cache_frozen or
                self._side not in (SIDE_LEFT, SIDE_RIGHT) or
                target_side not in (SIDE_LEFT, SIDE_RIGHT)):
            self._result["ring_target_rejection"] = "CACHE_OR_SIDE"
            return False
        if not self._ring_topology_precheck(topology):
            self._result["ring_target_rejection"] = "TOPOLOGY"
            self._reject(REJECT_TOPOLOGY_AMBIGUOUS)
            return False
        if not isinstance(mask_info, dict):
            self._result["ring_target_rejection"] = "INPUT"
            self._reject(REJECT_INPUT_INVALID)
            return False
        try:
            road_mask = mask_info.get("road_mask")
            known = mask_info.get("known_region")
            if (mask_info.get("valid", True) is False or
                    mask_info.get("width") != self.width or
                    mask_info.get("height") != self.height or
                    mask_info.get("floor") != self.floor_y or
                    not isinstance(road_mask, bytearray) or
                    not isinstance(known, bytearray) or
                    len(road_mask) != self.width * self.height or
                    len(known) != self.width * self.height):
                self._result["ring_target_rejection"] = "INPUT"
                self._reject(REJECT_INPUT_INVALID)
                return False
        except (TypeError, AttributeError):
            self._result["ring_target_rejection"] = "INPUT"
            self._reject(REJECT_INPUT_INVALID)
            return False
        count = 0
        near_observed = 0
        inferred = 0
        width_source_flag = (TARGET_WIDTH_FROM_CALIBRATION
                             if self._calibration_ready else
                             TARGET_WIDTH_FROM_CACHE)
        near_start = max(self.floor_y, self.height - self.near_rows)
        status = self._current_status
        edge2 = self._current_edge2
        for y in range(self.floor_y, self.height):
            if (self._ring_width_calibrated and
                    not self._ring_width_valid[y]):
                continue
            nearest_y = int(self._ring_nearest_y[y])
            if (nearest_y < 0 or abs(nearest_y - y) > self.max_cache_gap):
                continue
            width = int(self._ring_width[nearest_y])
            if width < 8 or width > 70:
                continue
            if target_side == SIDE_LEFT:
                if status[y] != 1:
                    continue
                left2 = int(edge2[y])
                right2 = left2 + 2 * width
                flags = (TARGET_OBSERVED_LEFT | TARGET_INFERRED_RIGHT |
                         width_source_flag)
            else:
                if status[self.height + y] != 1:
                    continue
                right2 = int(edge2[self.height + y])
                left2 = right2 - 2 * width
                flags = (TARGET_OBSERVED_RIGHT | TARGET_INFERRED_LEFT |
                         width_source_flag)
            if (left2 < -1 or right2 > (self.width - 1) * 2 + 1 or
                    left2 >= right2):
                continue
            left = (left2 + 1) // 2
            right = (right2 - 1) // 2
            if left < 0 or right >= self.width or left > right:
                continue
            offset = count * TARGET_ROW_STRIDE
            self._target_rows[offset] = y
            self._target_rows[offset + 1] = left
            self._target_rows[offset + 2] = right
            self._target_rows[offset + 3] = flags
            count += 1
            if y >= near_start:
                near_observed += 1
            inferred += 1

        if count < self.min_target_rows:
            self._result["ring_target_rejection"] = "MIN_ROWS"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        if near_observed < self.min_near_support:
            self._result["ring_target_rejection"] = "NEAR_SUPPORT"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        if (target_side == SIDE_LEFT and
                (self._near_left_rows < self.min_side_support or
                 self._near_left_run < self.min_side_contiguous)):
            self._result["ring_target_rejection"] = "LOCKED_SIDE_SUPPORT"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        if (target_side == SIDE_RIGHT and
                (self._near_right_rows < self.min_side_support or
                 self._near_right_run < self.min_side_contiguous)):
            self._result["ring_target_rejection"] = "LOCKED_SIDE_SUPPORT"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        if _ring_corridor_viper(road_mask, known, self._target_rows, count,
                                target_side) != 1:
            self._result["ring_target_rejection"] = "CORRIDOR"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        ref_index = -1
        best_delta = self.height + 1
        for index in range(count):
            offset = index * TARGET_ROW_STRIDE
            y = int(self._target_rows[offset])
            delta = y - self.reference_y
            if delta < 0:
                delta = -delta
            if delta < best_delta:
                best_delta = delta
                ref_index = index
        if ref_index < 0 or best_delta > self.max_cache_gap + 2:
            self._result["ring_target_rejection"] = "REFERENCE_GAP"
            self._reject(REJECT_SIDE_SUPPORT)
            return False
        offset = ref_index * TARGET_ROW_STRIDE
        left = int(self._target_rows[offset + 1])
        right = int(self._target_rows[offset + 2])
        left2 = left * 2 - 1
        right2 = right * 2 + 1
        center2 = left + right
        lateral = self._normalize_error(center2)
        result = self._result
        result["target_valid"] = True
        result["target_count"] = count
        result["target_side"] = target_side
        result["target_left2"] = left2
        result["target_right2"] = right2
        result["target_center2"] = center2
        result["target_lateral_error"] = lateral
        result["observed_rows"] = count
        result["inferred_rows"] = inferred
        return True

    def _normalize_error(self, center2):
        # Grid x spans 0..79; x2 therefore has center 79 and half-width 79.
        delta = center2 - (self.width - 1)
        scaled = delta * 1000
        if scaled >= 0:
            return (scaled + (self.width - 1) // 2) // (self.width - 1)
        return -((-scaled + (self.width - 1) // 2) // (self.width - 1))

    def _update_cross(self, now_ms, road_accepted, road_ok,
                      candidate_tracks, road_rows, road_count, road_stride,
                      topology, cache_ok, mask_info):
        result = self._result
        result["cross_reason"] = "NO_CROSS_EVIDENCE"
        if self._state not in (STATE_OBSERVE, STATE_REACQUIRE):
            self._reset_cross("RING_TRACK_ACTIVE")
            return
        if road_accepted:
            self._reset_cross("ROAD_PATH_NOT_EARLY_REJECTED")
            return
        if not (isinstance(topology, dict) and
                topology.get("reason") == "OPEN_BAND" and
                (topology.get("branch_evidence") or
                 topology.get("branch"))):
            self._reset_cross("NOT_OPEN_BAND")
            return
        if (not topology.get("complete") or
                topology.get("ambiguous") or
                topology.get("island_evidence") or topology.get("island") or
                topology.get("independent_branch_ambiguous") or
                topology.get("multi_branch_ambiguous") or
                topology.get("multiple_exits") or
                (isinstance(topology.get("exit_candidates"), int) and
                 topology.get("exit_candidates") > 1)):
            self._reset_cross("TOPOLOGY_NOT_RESOLVED")
            return
        if not cache_ok or self._stable_count < self.min_stable_frames:
            self._reset_cross("NO_STABLE_BILATERAL_BASELINE")
            return
        if not isinstance(mask_info, dict):
            self._reset_cross("MASK_UNAVAILABLE")
            return
        try:
            if (mask_info.get("valid", True) is False or
                    mask_info.get("width") != self.width or
                    mask_info.get("height") != self.height or
                    mask_info.get("floor") != self.floor_y):
                self._reset_cross("MASK_INVALID")
                return
            road_mask = mask_info.get("road_mask")
            edge_mask = mask_info.get("edge_mask")
            known = mask_info.get("known_region")
            if (not isinstance(road_mask, bytearray) or
                    not isinstance(edge_mask, bytearray) or
                    not isinstance(known, bytearray) or
                    len(road_mask) != self.width * self.height or
                    len(edge_mask) != self.width * self.height or
                    len(known) != self.width * self.height):
                self._reset_cross("MASK_INVALID")
                return
        except (TypeError, AttributeError):
            self._reset_cross("MASK_INVALID")
            return

        sampled = _cross_sample_viper(
            road_mask, edge_mask, known, self._cross_sample_left,
            self._cross_sample_right, self._cross_sample_valid,
            self._cross_sample_multiple)
        if sampled < 0:
            self._reset_cross("SAMPLE_KERNEL_BOUNDS")
            return

        for y in range(self.height):
            self._cross_sample_support[y] = 0
        # The helper takes its first bound as the largest (near) y and the
        # second as the smallest (far) y, then scans the inclusive band.
        self._measure_symmetric_opening(self.height - 1, self.floor_y)
        opening_count = int(self._cross_summary[0])
        longest_open = int(self._cross_summary[1])
        open_min_y = int(self._cross_summary[2])
        open_max_y = int(self._cross_summary[3])
        open_segments = int(self._cross_summary[4])
        result["cross_open_rows"] = opening_count
        result["cross_open_segments"] = open_segments
        if (opening_count < self.cross_min_open_rows or
                longest_open < self.cross_min_open_rows or
                open_segments != 1):
            self._reset_cross("OPENING_NOT_SYMMETRIC_OR_MULTIPLE")
            return

        far_start = self.floor_y
        far_end = self.floor_y + self.near_rows - 1
        if far_end >= self.height:
            far_end = self.height - 1
        near_start = self.height - self.near_rows
        if near_start < self.floor_y:
            near_start = self.floor_y
        near_end = self.height - 1
        self._measure_cross_band(far_start, far_end, 0)
        self._measure_cross_band(near_start, near_end, 6)
        summary = self._band_summary
        far_count = int(summary[0])
        far_y = int(summary[1])
        far_left = int(summary[2])
        far_right = int(summary[3])
        far_min_y = int(summary[4])
        far_max_y = int(summary[5])
        near_count = int(summary[6])
        near_y = int(summary[7])
        near_left = int(summary[8])
        near_right = int(summary[9])
        near_min_y = int(summary[10])
        near_max_y = int(summary[11])
        span = near_min_y - far_max_y
        result["cross_near_rows"] = near_count
        result["cross_far_rows"] = far_count
        result["cross_span"] = span
        result["cross_near_y"] = near_y
        result["cross_far_y"] = far_y
        result["cross_candidate_tracks"] = candidate_tracks
        if (near_count < 5 or far_count < 5 or span < 6 or
                near_y <= far_y):
            self._reset_cross("NEAR_FAR_SUPPORT_OR_SPAN")
            return
        if open_min_y < far_y or open_max_y > near_y:
            self._reset_cross("OPENING_OUTSIDE_STRAIGHT_SPAN")
            return

        near_center2 = near_left + near_right
        far_center2 = far_left + far_right
        result["cross_near_center2"] = near_center2
        result["cross_far_center2"] = far_center2
        result["cross_reference_center2"] = near_center2
        result["cross_reference_y"] = near_y
        result["cross_lateral_error"] = self._normalize_error(near_center2)
        near_width = near_right - near_left + 1
        far_width = far_right - far_left + 1
        if (abs(near_center2 - far_center2) >
                self.cross_center_tolerance2 or
                abs(near_width - far_width) > self.cross_width_tolerance):
            self._reset_cross("STRAIGHT_PATH_DISCONTINUITY")
            return

        result["cross_rows"] = sampled

        # Any second road run in the near-to-far span is an unselected route
        # that could be a separate exit; reject instead of choosing it.
        multiple = self._cross_sample_multiple
        for y in range(self.floor_y, self.height):
            if multiple[y] != 0:
                self._reset_cross("MULTIPLE_ROAD_RUNS")
                return

        corridor_ok = _cross_corridor_viper(
            road_mask, known, far_y, near_y, far_left, far_right,
            near_left, near_right)
        if corridor_ok != 1:
            self._reset_cross("CORRIDOR_NOT_CONTINUOUS")
            return

        route_count = self._build_cross_route(
            far_y, near_y, far_left, far_right, near_left, near_right)
        if route_count < 6:
            self._reset_cross("ROUTE_ROWS_INVALID")
            return

        result["cross_candidate"] = True
        result["cross_reason"] = "SYMMETRIC_OPENING_STRAIGHT_PATH"
        result["cross_route_count"] = route_count
        self._cross_count += 1
        if self._cross_count >= self.cross_confirm_frames:
            result["cross_confirmed"] = True
            result["cross_route_usable"] = True
            result["cross_suppress_slowdown"] = True
            result["cross_reason"] = "CROSS_STRAIGHT_CONFIRMED"
            if not self._cross_was_confirmed:
                self._record_event(EVENT_CROSS_CONFIRMED,
                                   REJECT_NONE, 0, now_ms,
                                   int(result["cross_near_center2"]))
                self._cross_was_confirmed = True
        else:
            result["cross_reason"] = "CROSS_CANDIDATE_DEBOUNCE"

    def _reset_cross(self, reason):
        self._cross_count = 0
        self._cross_was_confirmed = False
        self._result["cross_reason"] = reason
        self._result["cross_candidate"] = False
        self._result["cross_confirmed"] = False
        self._result["cross_route_usable"] = False
        self._result["cross_route_count"] = 0
        if reason not in ("RING_TRACK_ACTIVE", "ROAD_PATH_NOT_EARLY_REJECTED",
                          "NOT_OPEN_BAND"):
            self._reject(REJECT_CROSS_EVIDENCE)

    def _measure_cross_band(self, start_y, end_y, summary_offset):
        left_values = self._scratch_left
        right_values = self._scratch_right
        y_values = self._scratch_y
        valid = self._cross_sample_valid
        open_rows = self._cross_open_rows
        count = 0
        for y in range(start_y, end_y + 1):
            if valid[y] == 0 or open_rows[y] != 0:
                continue
            left_values[count] = int(self._cross_sample_left[y])
            right_values[count] = int(self._cross_sample_right[y])
            count += 1
        if count == 0:
            self._band_summary[summary_offset] = 0
            self._band_summary[summary_offset + 1] = -1
            self._band_summary[summary_offset + 2] = 0
            self._band_summary[summary_offset + 3] = 0
            self._band_summary[summary_offset + 4] = -1
            self._band_summary[summary_offset + 5] = -1
            return
        _sort_prefix(left_values, count)
        _sort_prefix(right_values, count)
        middle = count // 2
        median_left = int(left_values[middle])
        median_right = int(right_values[middle])
        support_count = 0
        min_y = self.height
        max_y = -1
        for y in range(start_y, end_y + 1):
            if valid[y] == 0 or open_rows[y] != 0:
                continue
            left_delta = int(self._cross_sample_left[y]) - median_left
            right_delta = int(self._cross_sample_right[y]) - median_right
            if (abs(left_delta) > 2 or abs(right_delta) > 2):
                continue
            self._cross_sample_support[y] = 1
            y_values[support_count] = y
            support_count += 1
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
        if support_count == 0:
            self._band_summary[summary_offset] = 0
            self._band_summary[summary_offset + 1] = -1
            self._band_summary[summary_offset + 2] = median_left
            self._band_summary[summary_offset + 3] = median_right
            self._band_summary[summary_offset + 4] = -1
            self._band_summary[summary_offset + 5] = -1
            return
        _sort_prefix(y_values, support_count)
        self._band_summary[summary_offset] = count
        self._band_summary[summary_offset] = support_count
        self._band_summary[summary_offset + 1] = y_values[support_count // 2]
        self._band_summary[summary_offset + 2] = median_left
        self._band_summary[summary_offset + 3] = median_right
        self._band_summary[summary_offset + 4] = min_y
        self._band_summary[summary_offset + 5] = max_y

    def _measure_symmetric_opening(self, near_y, far_y):
        open_rows = self._cross_open_rows
        cache_valid = self._active_cache_valid
        cache_left = self._active_cache_left2
        cache_right = self._active_cache_right2
        sample_left = self._cross_sample_left
        sample_right = self._cross_sample_right
        sample_valid = self._cross_sample_valid
        for y in range(self.height):
            open_rows[y] = 0
        count = 0
        longest = 0
        run = 0
        segments = 0
        in_segment = False
        min_y = self.height
        max_y = -1
        for y in range(far_y, near_y + 1):
            qualifies = False
            if sample_valid[y] != 0 and cache_valid[y] != 0:
                base_left = (int(cache_left[y]) + 1) // 2
                base_right = (int(cache_right[y]) - 1) // 2
                left_open = base_left - int(sample_left[y])
                right_open = int(sample_right[y]) - base_right
                if (left_open >= self.cross_open_min_cells and
                        right_open >= self.cross_open_min_cells and
                        abs(left_open - right_open) <=
                        self.cross_symmetry_tolerance and
                        abs((int(sample_left[y]) + int(sample_right[y])) -
                            (base_left + base_right)) <=
                        self.cross_center_tolerance2):
                    qualifies = True
            if qualifies:
                open_rows[y] = 1
                count += 1
                run += 1
                if not in_segment:
                    segments += 1
                    in_segment = True
                if run > longest:
                    longest = run
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
            else:
                run = 0
                in_segment = False
        self._cross_summary[0] = count
        self._cross_summary[1] = longest
        self._cross_summary[2] = min_y
        self._cross_summary[3] = max_y
        self._cross_summary[4] = segments

    def _build_cross_route(self, far_y, near_y, far_left, far_right,
                           near_left, near_right):
        span = near_y - far_y
        if span < 6:
            return 0
        route = self._cross_route_rows
        count = 0
        y = near_y
        while y >= far_y:
            offset = y - far_y
            left = far_left + ((near_left - far_left) * offset) // span
            right = far_right + ((near_right - far_right) * offset) // span
            flags = CROSS_ROUTE_INTERPOLATED
            sample_valid = self._cross_sample_valid[y] != 0
            sample_support = self._cross_sample_support[y] != 0
            is_open = self._cross_open_rows[y] != 0
            if sample_valid and not is_open:
                measured_left = int(self._cross_sample_left[y])
                measured_right = int(self._cross_sample_right[y])
                if sample_support or (abs(measured_left - left) <= 2 and
                        abs(measured_right - right) <= 2):
                    left = measured_left
                    right = measured_right
                    flags = CROSS_ROUTE_MEASURED
            if left < 0 or right >= self.width or left > right:
                return 0
            offset = count * TARGET_ROW_STRIDE
            route[offset] = y
            route[offset + 1] = left
            route[offset + 2] = right
            route[offset + 3] = flags
            count += 1
            y -= 1
        return count

    def _cache_left2_value(self, y):
        # Cache refs are captured for the current update by _prepare_cache.
        return self._active_cache_left2[y]

    def _cache_right2_value(self, y):
        return self._active_cache_right2[y]

    def _reject(self, code):
        if not _is_int(code) or code <= REJECT_NONE or code >= REJECT_COUNT:
            code = REJECT_INPUT_INVALID
        self._last_reject = code
        self._reject_counts[code] += 1
        self._result["reject_reason"] = _REJECT_NAMES[code]

    def _record_event(self, event, reason, edge2, now_ms=None,
                      target_center2=None):
        index = self._event_head * EVENT_STRIDE
        self._events[index] = self._frame_index
        self._events[index + 1] = event
        self._events[index + 2] = self._state
        self._events[index + 3] = self._side
        self._events[index + 4] = reason
        self._events[index + 5] = edge2
        self._events[index + 6] = (self._last_now_ms if now_ms is None else
                                    int(now_ms))
        if target_center2 is None:
            target_center2 = int(self._result["target_center2"])
        self._events[index + 7] = int(target_center2)
        self._event_head += 1
        if self._event_head >= EVENT_CAPACITY:
            self._event_head = 0
        if self._event_count < EVENT_CAPACITY:
            self._event_count += 1
        else:
            self._event_dropped += 1

    @micropython.native
    def _publish(self):
        result = self._result
        result["state"] = _STATE_NAMES[self._state]
        result["state_id"] = self._state
        result["reason"] = self._reason
        result["side"] = self._side
        if self._side == SIDE_LEFT:
            result["side_name"] = "LEFT"
        elif self._side == SIDE_RIGHT:
            result["side_name"] = "RIGHT"
        else:
            result["side_name"] = "NONE"
        target_side = result["target_side"]
        if target_side == SIDE_LEFT:
            result["target_side_name"] = "LEFT"
        elif target_side == SIDE_RIGHT:
            result["target_side_name"] = "RIGHT"
        else:
            result["target_side_name"] = "NONE"
        result["candidate"] = self._state in (
            STATE_ENTRY_CONFIRM, STATE_ENTER, STATE_INSIDE,
            STATE_EXIT_CONFIRM, STATE_EXIT_REPAIR)
        result["ring_confirmed"] = self._state in (
            STATE_INSIDE, STATE_EXIT_CONFIRM, STATE_EXIT_REPAIR)
        result["control_eligible"] = bool(
            self._profile_id in (2, 3) and
            self._state in (STATE_INSIDE, STATE_EXIT_CONFIRM,
                            STATE_EXIT_REPAIR) and
            result["target_valid"])
        result["target_rows"] = self._target_rows
        result["cross_route_rows"] = self._cross_route_rows
        result["reject_counts"] = self._reject_counts
        result["event_log"] = self._events
        result["event_count"] = self._event_count
        result["event_head"] = self._event_head
        result["event_dropped"] = self._event_dropped
        result["frame_index"] = self._frame_index
        result["state_frames"] = self._state_frames
        elapsed = _ticks_diff(int(self._last_now_ms),
                              int(self._state_start_ms))
        if elapsed < 0:
            elapsed = 0
        result["state_elapsed_ms"] = elapsed
        self._sync_ring_diagnostic()

    def _sync_ring_diagnostic(self):
        result = self._result
        reason_code = self._ring_diag_reason_code
        if reason_code < 0 or reason_code >= len(_RING_DIAG_REASON_NAMES):
            reason_code = 0
        result["ring_diagnostic_reason"] = \
            _RING_DIAG_REASON_NAMES[reason_code]
        result["ring_arc_support_rows"] = self._ring_diag_arc_rows
        result["ring_arc_span_rows"] = self._ring_diag_arc_span
        result["ring_arc_rejection"] = self._ring_diag_arc_rejection
        result["ring_arc_growth_frames"] = self._ring_diag_growth_frames
        result["ring_arc_growth_x2"] = self._ring_diag_growth_x2
        result["entry_confirmation_path"] = \
            self._entry_confirmation_path
        result["ring_near_anchor_rows"] = self._ring_diag_anchor_rows
        result["ring_near_anchor_run"] = self._ring_diag_anchor_run
        result["ring_exit_fit_rows"] = self._ring_diag_exit_rows
        result["ring_exit_span_rows"] = self._ring_diag_exit_span
        result["ring_exit_residual_x2"] = self._ring_diag_exit_residual
        record = self._ring_diag
        record[0] = reason_code
        record[1] = self._state
        record[2] = self._side
        record[3] = 1 if self._calibration_ready else 0
        record[4] = self._calibration_reason_code()
        record[5] = 1 if self._cal_check_ready else 0
        record[6] = self._cal_check_reason_code()
        record[7] = self._cal_check_stable_frames
        record[8] = self._cal_check_support_rows
        record[9] = self._cal_check_span_rows
        record[10] = self._cal_check_median_error_permille
        record[11] = self._ring_diag_arc_rows
        record[12] = self._ring_diag_anchor_rows
        record[13] = self._ring_diag_exit_rows

    def _calibration_reason_code(self):
        reason = self._calibration_reason
        if reason == "CALIBRATION_READY":
            return 0
        if reason in ("NO_CALIBRATION", "INVALID_CALIBRATION"):
            return 1
        if reason == "FILE_NOT_FOUND":
            return 2
        if reason == "CRC_MISMATCH":
            return 3
        if reason == "GEOMETRY_MISMATCH":
            return 4
        return 5

    def _cal_check_reason_code(self):
        reason = self._cal_check_reason
        if reason == "STARTUP_CHECK_READY":
            return 0
        if reason == "NOT_CHECKED":
            return 1
        if reason == "WAITING_FOR_CALIBRATION":
            return 2
        if reason == "CHECK_SUPPORT_INSUFFICIENT":
            return 3
        if reason == "WIDTH_MISMATCH":
            return 4
        if reason == "WAITING_FOR_STABLE_WIDTH_PROFILE":
            return 5
        if reason == "PARKED_GATE":
            return 7
        return 6

