"""Reusable Viper road-boundary evidence and selected-reference checks.

The API mirrors the road portion of ``boundary_evidence.py`` while keeping
per-row sampling and support counting in bounded, reusable integer buffers.
Returned masks, evidence, and support-point view are valid only until this
instance processes another candidate/frame. Diagnostic medians are deferred
unless the caller constructs the instance with ``diagnostics=True``.
"""

from array import array

try:
    import micropython
    _IS_MICROPYTHON = True
    COMPILER_BACKEND = "micropython-viper"
except ImportError:
    _IS_MICROPYTHON = False
    COMPILER_BACKEND = "cpython-shim"

    class _MicroPythonHostShim:
        def viper(self, function):
            return function

        def native(self, function):
            return function

        def const(self, value):
            return value

    micropython = _MicroPythonHostShim()
    ptr8 = int
    ptr32 = int


RUNTIME_API_VERSION = micropython.const(7)
WIDTH = micropython.const(80)
HEIGHT = micropython.const(60)
PIXELS = micropython.const(WIDTH * HEIGHT)
MAX_ROWS = micropython.const(HEIGHT)
# FastPathDetector's selected run centers and reference points lie on the
# half-cell grid. Twice-coordinate storage is exact for those inputs and keeps
# radius-squared arithmetic safely inside signed 32-bit Viper integers.
COORD_SCALE = micropython.const(2)
STEP_SCALE = micropython.const(1000)
FRACTION_SCALE = micropython.const(1000000)
_ROW_STRIDE = micropython.const(11)
_OUTPUT_STRIDE = micropython.const(9)
_NO_MEDIAN = micropython.const(-32768)


def _setting(settings, name, default):
    if isinstance(settings, dict):
        return settings.get(name, default)
    return getattr(settings, name, default)


def _round_cell(value):
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


def _round_fixed(value, scale):
    scaled = float(value) * scale
    if scaled >= 0.0:
        return int(scaled + 0.5)
    return int(scaled - 0.5)


def _ceil_scaled(value, scale):
    scaled = float(value) * scale
    integer = int(scaled)
    if scaled > integer:
        integer += 1
    return integer


def _floor_scaled_nonnegative(value, scale):
    """Round a nonnegative distance threshold inward, never outward."""
    return int(float(value) * scale)


def _finite(value):
    return value == value and value != float("inf") and \
        value != -float("inf")


def _coordinate_index(x, y, width, height):
    # Match boundary_evidence._index: reject floating coordinates before
    # half-away-from-zero rounding, then require the rounded cell in-bounds.
    if not _finite(x) or not _finite(y):
        return -1
    if x < 0.0 or x > width - 1 or y < 0.0 or y > height - 1:
        return -1
    ix = _round_cell(x)
    iy = _round_cell(y)
    if ix < 0 or ix >= width or iy < 0 or iy >= height:
        return -1
    return iy * width + ix


class _SupportPointView:
    """Stable read-only view over the current frame's support arrays."""

    __slots__ = ("owner",)

    def __init__(self, owner):
        self.owner = owner

    def __len__(self):
        return self.owner._support_count

    def __getitem__(self, index):
        count = self.owner._support_count
        if index < 0:
            index += count
        if index < 0 or index >= count:
            raise IndexError("support point index out of range")
        return (self.owner._support_x[index] / float(COORD_SCALE),
                self.owner._support_y[index] / float(COORD_SCALE))

    def __iter__(self):
        for index in range(self.owner._support_count):
            yield self[index]

    def __eq__(self, other):
        try:
            if len(other) != len(self):
                return False
            for index in range(len(self)):
                if self[index] != other[index]:
                    return False
            return True
        except (TypeError, IndexError):
            return False

    def __repr__(self):
        return repr(list(self))


@micropython.viper
def _measure_side_viper(self, row: int, side: int) -> int:
    """Read one dark hit and up to three known bright pixels."""
    luma = ptr8(self._current_luma)
    dark = ptr8(self._current_dark)
    white = ptr8(self._current_white)
    known = ptr8(self._current_known)
    records = ptr32(self._records)
    output = ptr32(self._row_output)
    width = int(self.width)
    height = int(self.height)
    row_stride = int(_ROW_STRIDE)
    output_stride = int(_OUTPUT_STRIDE)
    step_scale = int(STEP_SCALE)
    fraction_scale = int(FRACTION_SCALE)
    total = width * height
    record_base = row * row_stride
    output_base = row * output_stride + side * 4
    hit_offset = 1 + side * 4
    hit = int(records[record_base + hit_offset])
    output[output_base] = 0
    output[output_base + 1] = 0
    output[output_base + 2] = 0
    output[output_base + 3] = 0
    if hit < 0 or hit >= total:
        return 0
    if known[hit] == 0 or dark[hit] == 0:
        return 0

    value0 = 0
    value1 = 0
    value2 = 0
    sample_count = 0
    for sample in range(3):
        index = int(records[record_base + hit_offset + 1 + sample])
        if index < 0 or index >= total:
            continue
        if known[index] == 0 or white[index] == 0:
            continue
        value = int(luma[index])
        if sample_count == 0:
            value0 = value
        elif sample_count == 1:
            value1 = value
        else:
            value2 = value
        sample_count += 1
    if sample_count < 2:
        return 0

    if sample_count == 2:
        white2 = value0 + value1
    else:
        if value0 > value1:
            temp = value0
            value0 = value1
            value1 = temp
        if value1 > value2:
            temp = value1
            value1 = value2
            value2 = temp
        if value0 > value1:
            temp = value0
            value0 = value1
            value1 = temp
        white2 = value1 * 2

    step2 = white2 - int(luma[hit]) * 2
    denominator2 = white2
    if denominator2 < 2:
        denominator2 = 2
    output[output_base] = step2
    output[output_base + 1] = white2
    output[output_base + 2] = 1

    minimum_step_scaled = int(self._minimum_step_scaled)
    minimum_fraction_scaled = int(self._minimum_fraction_scaled)
    if step2 * step_scale < minimum_step_scaled * 2:
        return 0
    if step2 * fraction_scale < denominator2 * minimum_fraction_scaled:
        return 0
    output[output_base + 3] = 1
    return 1


@micropython.viper
def _gate_viper(self) -> int:
    """Run bounded evidence counts over staged selected-row coordinates."""
    records = ptr32(self._records)
    output = ptr32(self._row_output)
    metrics = ptr32(self._metrics)
    support_x = ptr32(self._support_x)
    support_y = ptr32(self._support_y)
    row_stride = int(_ROW_STRIDE)
    output_stride = int(_OUTPUT_STRIDE)
    count = int(self._current_count)
    for metric in range(8):
        metrics[metric] = 0
    support_count = 0

    for row in range(count):
        output_base = row * output_stride
        for field in range(output_stride):
            output[output_base + field] = 0
        record_base = row * row_stride
        if records[record_base] == 0:
            continue
        left_ok = int(_measure_side_viper(self, row, 0))
        right_ok = int(_measure_side_viper(self, row, 1))
        metrics[2] += left_ok
        metrics[3] += right_ok
        metrics[4] += int(output[output_base + 2])
        metrics[5] += int(output[output_base + 6])
        if left_ok != 0 and right_ok != 0:
            metrics[0] += 1
            support_x[support_count] = int(records[record_base + 9])
            support_y[support_count] = int(records[record_base + 10])
            if int(self._target_available) != 0:
                dx = int(self._target_x_q) - \
                    int(support_x[support_count])
                dy = int(self._target_y_q) - \
                    int(support_y[support_count])
                radius = int(self._target_radius_q)
                if dx <= radius and dx >= -radius and \
                        dy <= radius and dy >= -radius and \
                        dx * dx + dy * dy <= radius * radius:
                    metrics[1] += 1
            support_count += 1
        output[output_base + 8] = 1 if left_ok != 0 and right_ok != 0 else 0

    self._support_count = support_count
    return 0


@micropython.viper
def _summary_viper(self) -> int:
    """Sort only small integer diagnostic vectors and retain exact ratios."""
    output = ptr32(self._row_output)
    work = ptr32(self._summary_work)
    summary = ptr32(self._summary_values)
    max_rows = int(MAX_ROWS)
    output_stride = int(_OUTPUT_STRIDE)
    no_median = int(_NO_MEDIAN)
    count = int(self._current_count)

    # Groups 0/1 are side step2 values. Convert the median to quarter-levels
    # so the even-count median remains exact without floating-point work.
    for side in range(2):
        group = side
        work_base = group * max_rows
        found = 0
        for row in range(count):
            output_base = row * output_stride + side * 4
            if output[output_base + 2] != 0:
                work[work_base + found] = int(output[output_base])
                found += 1
        if found == 0:
            summary[side] = no_median
        else:
            for index in range(1, found):
                key = int(work[work_base + index])
                cursor = index - 1
                while cursor >= 0 and int(work[work_base + cursor]) > key:
                    work[work_base + cursor + 1] = \
                        int(work[work_base + cursor])
                    cursor -= 1
                work[work_base + cursor + 1] = key
            middle = found // 2
            if found & 1:
                summary[side] = int(work[work_base + middle]) * 2
            else:
                summary[side] = (int(work[work_base + middle - 1]) +
                                 int(work[work_base + middle]))

    # Groups 2/3 and 4/5 store exact fraction numerator/denominator pairs.
    for side in range(2):
        numerator_base = (2 + side * 2) * max_rows
        denominator_base = numerator_base + max_rows
        found = 0
        for row in range(count):
            output_base = row * output_stride + side * 4
            if output[output_base + 2] == 0:
                continue
            numerator = int(output[output_base])
            denominator = int(output[output_base + 1])
            if denominator < 2:
                denominator = 2
            work[numerator_base + found] = numerator
            work[denominator_base + found] = denominator
            found += 1
        if found == 0:
            summary[2 + side * 2] = no_median
            summary[3 + side * 2] = 0
            continue
        for index in range(1, found):
            key_n = int(work[numerator_base + index])
            key_d = int(work[denominator_base + index])
            cursor = index - 1
            while cursor >= 0 and \
                    int(work[numerator_base + cursor]) * key_d > \
                    key_n * int(work[denominator_base + cursor]):
                work[numerator_base + cursor + 1] = \
                    int(work[numerator_base + cursor])
                work[denominator_base + cursor + 1] = \
                    int(work[denominator_base + cursor])
                cursor -= 1
            work[numerator_base + cursor + 1] = key_n
            work[denominator_base + cursor + 1] = key_d
        middle = found // 2
        if found & 1:
            summary[2 + side * 2] = \
                int(work[numerator_base + middle])
            summary[3 + side * 2] = \
                int(work[denominator_base + middle])
        else:
            left_n = int(work[numerator_base + middle - 1])
            left_d = int(work[denominator_base + middle - 1])
            right_n = int(work[numerator_base + middle])
            right_d = int(work[denominator_base + middle])
            summary[2 + side * 2] = left_n * right_d + right_n * left_d
            summary[3 + side * 2] = 2 * left_d * right_d
    return 0


@micropython.viper
def _reference_viper(self) -> int:
    """Count existing bilateral points near one selected controller point."""
    support_x = ptr32(self._support_x)
    support_y = ptr32(self._support_y)
    count = int(self._support_count)
    target_x = int(self._target_x_q)
    target_y = int(self._target_y_q)
    radius = int(self._target_radius_q)
    supported = 0
    for index in range(count):
        dx = target_x - int(support_x[index])
        dy = target_y - int(support_y[index])
        if dx <= radius and dx >= -radius and \
                dy <= radius and dy >= -radius and \
                dx * dx + dy * dy <= radius * radius:
            supported += 1
    return supported


@micropython.viper
def _stage_path_viper(self) -> int:
    """Stage FastPathDetector's reusable [y,left,right,flags] integer rows."""
    rows = ptr32(self._current_path_rows)
    records = ptr32(self._records)
    row_stride = int(_ROW_STRIDE)
    count = int(self._current_count)
    width = int(self.width)
    height = int(self.height)
    stride = int(self._current_stride)
    for row in range(count):
        record_base = row * row_stride
        for field in range(row_stride):
            records[record_base + field] = 0
        source_base = row * stride
        y = int(rows[source_base])
        left = int(rows[source_base + 1])
        right = int(rows[source_base + 2])
        flags = int(rows[source_base + 3])
        if flags & 7 != 7:
            continue
        if y < 0 or y >= height or left < 0 or right < left or \
                right >= width:
            continue
        row_offset = y * width
        records[record_base] = 1
        if left > 0:
            records[record_base + 1] = row_offset + left - 1
        else:
            records[record_base + 1] = -1
        records[record_base + 2] = row_offset + left
        if left + 1 < width:
            records[record_base + 3] = row_offset + left + 1
        else:
            records[record_base + 3] = -1
        if left + 2 < width:
            records[record_base + 4] = row_offset + left + 2
        else:
            records[record_base + 4] = -1
        if right + 1 < width:
            records[record_base + 5] = row_offset + right + 1
        else:
            records[record_base + 5] = -1
        records[record_base + 6] = row_offset + right
        if right > 0:
            records[record_base + 7] = row_offset + right - 1
        else:
            records[record_base + 7] = -1
        if right > 1:
            records[record_base + 8] = row_offset + right - 2
        else:
            records[record_base + 8] = -1
        records[record_base + 9] = left + right
        records[record_base + 10] = y * 2
    return 0


class FastBoundaryEvidence:
    """V7 road-only replacement for the gate/reference part of API v6.

    ``gate_road_candidate`` mutates the detector candidate just like the
    legacy function. The evidence dictionary and support-point view are
    reused by this object. Call ``validate_selected_reference`` before this
    instance gates another candidate or frame.
    """

    def __init__(self, settings, diagnostics=False):
        self.settings = settings
        if not isinstance(diagnostics, bool):
            raise ValueError("diagnostics must be bool")
        self.diagnostics = diagnostics
        self.width = int(_setting(settings, "SURFACE_GRID_WIDTH", WIDTH))
        self.height = int(_setting(settings, "SURFACE_GRID_HEIGHT", HEIGHT))
        if self.width != WIDTH or self.height != HEIGHT:
            raise ValueError("FastBoundaryEvidence requires an 80x60 grid")

        self._records = array("i", [0] * (MAX_ROWS * _ROW_STRIDE))
        self._row_output = array("i", [0] * (MAX_ROWS * _OUTPUT_STRIDE))
        self._metrics = array("i", [0] * 8)
        self._support_x = array("i", [0] * MAX_ROWS)
        self._support_y = array("i", [0] * MAX_ROWS)
        self._summary_work = array("i", [0] * (6 * MAX_ROWS))
        self._summary_values = array("i", [_NO_MEDIAN] * 6)
        self._current_luma = None
        self._current_dark = None
        self._current_white = None
        self._current_known = None
        self._current_path_rows = None
        self._path_rows_owner = None
        self._path_rows_view = None
        self._current_count = 0
        self._current_stride = 4
        self._support_count = 0
        self._target_available = 0
        self._target_x_q = 0
        self._target_y_q = 0
        self._target_radius_q = COORD_SCALE * 5
        self._minimum_step_scaled = 10000
        self._minimum_fraction_scaled = 250000
        self._minimum_ratio_scaled = 750000
        self._minimum_target_support = 2
        self._limits = None
        self._point_view = _SupportPointView(self)
        self.evidence = {
            "source": "road", "status": "NOT_RUN", "reason": "NOT_RUN",
            "pairs_tested": 0, "bilateral_supported": 0,
            "left_supported": 0, "right_supported": 0,
            "support_ratio": 0.0, "target_support_count": 0,
            "minimum_step": 10.0, "minimum_step_fraction": 0.25,
            "left_step_median": None, "right_step_median": None,
            "left_step_fraction_median": None,
            "right_step_fraction_median": None,
            "support_points": self._point_view,
            "selected_reference_status": None,
            "selected_reference_support_count": 0,
            "selected_reference_reason": None,
            "preselection_status": None,
            "selected_reference": None,
        }

    @micropython.native
    def _read_limits(self, settings):
        minimum_step = float(_setting(settings, "GRAY_BOUNDARY_MIN_STEP", 10.0))
        minimum_fraction = float(_setting(
            settings, "GRAY_BOUNDARY_MIN_STEP_FRACTION", 0.25))
        minimum_ratio = float(_setting(
            settings, "GRAY_BOUNDARY_SUPPORT_RATIO", 0.75))
        radius = float(_setting(
            settings, "GRAY_BOUNDARY_TARGET_RADIUS_CELLS", 5.0))
        minimum_target_support = _setting(
            settings, "GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT", 2)
        if (not _finite(minimum_step) or minimum_step < 0.0 or
                not _finite(minimum_fraction) or
                minimum_fraction < 0.0 or minimum_fraction >= 1.0 or
                not _finite(minimum_ratio) or minimum_ratio < 0.0 or
                minimum_ratio > 1.0 or not _finite(radius) or radius < 0.0 or
                radius > max(self.width, self.height) or
                isinstance(minimum_target_support, bool) or
                not isinstance(minimum_target_support, int) or
                minimum_target_support < 1 or
                minimum_target_support > MAX_ROWS):
            raise ValueError("invalid fast boundary evidence settings")
        # No 8-bit gray step can exceed 255. Capping larger configured values
        # at 256 preserves their reject-all semantics while keeping Viper
        # signed products safely inside 32 bits.
        if minimum_step > 255.0:
            self._minimum_step_scaled = 256 * STEP_SCALE
        else:
            self._minimum_step_scaled = _ceil_scaled(minimum_step, STEP_SCALE)
        self._minimum_fraction_scaled = _ceil_scaled(
            minimum_fraction, FRACTION_SCALE)
        self._minimum_ratio_scaled = _ceil_scaled(
            minimum_ratio, FRACTION_SCALE)
        # The floor is conservative for arbitrary float configuration and is
        # exact for the default radius (5 cells) and selected half-cell grid.
        self._target_radius_q = _floor_scaled_nonnegative(
            radius, COORD_SCALE)
        self._minimum_target_support = minimum_target_support
        self.evidence["minimum_step"] = minimum_step
        self.evidence["minimum_step_fraction"] = minimum_fraction
        self.evidence["minimum_step_fraction"] = minimum_fraction
        self._limits = (minimum_step, minimum_fraction, minimum_ratio, radius,
                        minimum_target_support)

    @micropython.native
    def _reset_evidence(self, reason, pairs=0, bilateral=0, target=0):
        evidence = self.evidence
        evidence["source"] = "road"
        evidence["status"] = "REJECTED" if reason != "OK" else "PASS"
        evidence["reason"] = reason
        evidence["pairs_tested"] = pairs
        evidence["bilateral_supported"] = bilateral
        evidence["left_supported"] = 0
        evidence["right_supported"] = 0
        evidence["support_ratio"] = (float(bilateral) / pairs) if pairs else 0.0
        evidence["target_support_count"] = target
        evidence["left_step_median"] = None
        evidence["right_step_median"] = None
        evidence["left_step_fraction_median"] = None
        evidence["right_step_fraction_median"] = None
        evidence["selected_reference_status"] = None
        evidence["selected_reference_support_count"] = 0
        evidence["selected_reference_reason"] = None
        evidence["preselection_status"] = None
        evidence["selected_reference"] = None

    @micropython.native
    def _invalidate(self, candidate, evidence,
                    result_reason="BOUNDARY_EVIDENCE_REJECTED"):
        if "geometry_reason" not in candidate:
            candidate["geometry_reason"] = candidate.get("reason")
        candidate["boundary_evidence"] = evidence
        candidate["valid"] = False
        candidate["quality"] = 0
        candidate["reason"] = result_reason
        candidate["lateral_error"] = 0
        candidate["center_x"] = None
        candidate["reference_center_x"] = None
        candidate["reference_lateral_error"] = 0

    def _stage_side(self, record_base, field_base, hit_x, y, inward_sign):
        hit_index = _coordinate_index(
            hit_x, y, self.width, self.height)
        self._records[record_base + field_base] = hit_index
        for sample in range(3):
            sample_index = _coordinate_index(
                hit_x + inward_sign * (sample + 1), y,
                self.width, self.height)
            self._records[record_base + field_base + 1 + sample] = \
                sample_index

    def _stage_rows(self, rows, count, target):
        self._current_count = count
        self._support_count = 0
        self._stage_target(target)

        for row_index in range(count):
            record_base = row_index * _ROW_STRIDE
            for field in range(_ROW_STRIDE):
                self._records[record_base + field] = 0
            row = rows[row_index]
            left = row.get("left")
            right = row.get("right")
            y = row.get("y")
            if (left is None or right is None or y is None or
                    row.get("provenance") != "OBSERVED" or
                    row.get("left_status") != "OBSERVED" or
                    row.get("right_status") != "OBSERVED"):
                continue
            try:
                left_x = float(left)
                right_x = float(right)
                row_y = float(y)
                center_x = (left_x + right_x) * 0.5
                if (not _finite(left_x) or not _finite(right_x) or
                        not _finite(row_y) or not _finite(center_x)):
                    continue
                point_x_q = _round_fixed(center_x, COORD_SCALE)
                point_y_q = _round_fixed(row_y, COORD_SCALE)
            except (TypeError, ValueError, OverflowError):
                continue

            self._records[record_base] = 1
            self._records[record_base + 9] = point_x_q
            self._records[record_base + 10] = point_y_q
            left_hit_x = left_x - 1.0
            if center_x > left_hit_x:
                self._stage_side(record_base, 1, left_hit_x, row_y,
                                 1.0)
            else:
                self._records[record_base + 1] = -1
            right_hit_x = right_x + 1.0
            if center_x < right_hit_x:
                self._stage_side(record_base, 5, right_hit_x, row_y,
                                 -1.0)
            else:
                self._records[record_base + 5] = -1

    @micropython.native
    def _stage_target(self, target):
        """Quantize a bounded point without letting hostile floats overflow."""
        self._target_available = 0
        if target is None:
            return
        try:
            target_x = float(target[0])
            target_y = float(target[1])
        except (IndexError, TypeError, ValueError, OverflowError):
            return
        if not _finite(target_x) or not _finite(target_y):
            return
        radius = self._limits[3] if self._limits is not None else 5.0
        if (target_x < -radius or
                target_x > self.width - 1 + radius or
                target_y < -radius or
                target_y > self.height - 1 + radius):
            return
        self._target_x_q = _round_fixed(target_x, COORD_SCALE)
        self._target_y_q = _round_fixed(target_y, COORD_SCALE)
        self._target_available = 1

    def _stage_path_host(self):
        """CPython equivalent of the no-dict Viper row staging kernel."""
        rows = self._current_path_rows
        for row_index in range(self._current_count):
            record_base = row_index * _ROW_STRIDE
            for field in range(_ROW_STRIDE):
                self._records[record_base + field] = 0
            source_base = row_index * self._current_stride
            y = int(rows[source_base])
            left = int(rows[source_base + 1])
            right = int(rows[source_base + 2])
            flags = int(rows[source_base + 3])
            if (flags & 7) != 7 or y < 0 or y >= self.height or \
                    left < 0 or right < left or right >= self.width:
                continue
            row_offset = y * self.width
            self._records[record_base] = 1
            self._records[record_base + 1] = (
                row_offset + left - 1 if left > 0 else -1)
            self._records[record_base + 2] = row_offset + left
            self._records[record_base + 3] = (
                row_offset + left + 1 if left + 1 < self.width else -1)
            self._records[record_base + 4] = (
                row_offset + left + 2 if left + 2 < self.width else -1)
            self._records[record_base + 5] = (
                row_offset + right + 1 if right + 1 < self.width else -1)
            self._records[record_base + 6] = row_offset + right
            self._records[record_base + 7] = (
                row_offset + right - 1 if right > 0 else -1)
            self._records[record_base + 8] = (
                row_offset + right - 2 if right > 1 else -1)
            self._records[record_base + 9] = left + right
            self._records[record_base + 10] = y * 2

    def _host_measure_side(self, row, side):
        record_base = row * _ROW_STRIDE
        output_base = row * _OUTPUT_STRIDE + side * 4
        hit_offset = 1 + side * 4
        hit = self._records[record_base + hit_offset]
        total = self.width * self.height
        output = self._row_output
        output[output_base] = 0
        output[output_base + 1] = 0
        output[output_base + 2] = 0
        output[output_base + 3] = 0
        if hit < 0 or hit >= total or not self._current_known[hit] or \
                not self._current_dark[hit]:
            return 0

        value0 = 0
        value1 = 0
        value2 = 0
        sample_count = 0
        for sample in range(3):
            index = self._records[record_base + hit_offset + 1 + sample]
            if (index < 0 or index >= total or
                    not self._current_known[index] or
                    not self._current_white[index]):
                continue
            value = self._current_luma[index]
            if sample_count == 0:
                value0 = value
            elif sample_count == 1:
                value1 = value
            else:
                value2 = value
            sample_count += 1
        if sample_count < 2:
            return 0
        if sample_count == 2:
            white2 = value0 + value1
        else:
            values = (value0, value1, value2)
            low = min(values)
            high = max(values)
            white2 = (sum(values) - low - high) * 2
        step2 = white2 - 2 * self._current_luma[hit]
        denominator2 = max(2, white2)
        output[output_base] = step2
        output[output_base + 1] = white2
        output[output_base + 2] = 1
        if step2 * STEP_SCALE < self._minimum_step_scaled * 2:
            return 0
        if step2 * FRACTION_SCALE < \
                denominator2 * self._minimum_fraction_scaled:
            return 0
        output[output_base + 3] = 1
        return 1

    def _host_gate(self):
        metrics = self._metrics
        for index in range(8):
            metrics[index] = 0
        support_count = 0
        for row in range(self._current_count):
            output_base = row * _OUTPUT_STRIDE
            for field in range(_OUTPUT_STRIDE):
                self._row_output[output_base + field] = 0
            record_base = row * _ROW_STRIDE
            if self._records[record_base] == 0:
                continue
            left_ok = self._host_measure_side(row, 0)
            right_ok = self._host_measure_side(row, 1)
            metrics[2] += left_ok
            metrics[3] += right_ok
            metrics[4] += self._row_output[output_base + 2]
            metrics[5] += self._row_output[output_base + 6]
            if left_ok and right_ok:
                metrics[0] += 1
                point_x = self._records[record_base + 9]
                point_y = self._records[record_base + 10]
                self._support_x[support_count] = point_x
                self._support_y[support_count] = point_y
                if self._target_available:
                    dx = self._target_x_q - point_x
                    dy = self._target_y_q - point_y
                    radius = self._target_radius_q
                    if (abs(dx) <= radius and abs(dy) <= radius and
                            dx * dx + dy * dy <= radius * radius):
                        metrics[1] += 1
                support_count += 1
            self._row_output[output_base + 8] = int(
                bool(left_ok and right_ok))
        self._support_count = support_count

    def _host_summary(self):
        summary = self._summary_values
        output = self._row_output
        work = self._summary_work
        for side in range(2):
            count = 0
            work_base = side * MAX_ROWS
            for row in range(self._current_count):
                output_base = row * _OUTPUT_STRIDE + side * 4
                if output[output_base + 2]:
                    work[work_base + count] = output[output_base]
                    count += 1
            if not count:
                summary[side] = _NO_MEDIAN
            else:
                for index in range(1, count):
                    key = work[work_base + index]
                    cursor = index - 1
                    while cursor >= 0 and work[work_base + cursor] > key:
                        work[work_base + cursor + 1] = \
                            work[work_base + cursor]
                        cursor -= 1
                    work[work_base + cursor + 1] = key
                middle = count // 2
                if count & 1:
                    summary[side] = work[work_base + middle] * 2
                else:
                    summary[side] = (work[work_base + middle - 1] +
                                     work[work_base + middle])

        for side in range(2):
            numerator_base = (2 + side * 2) * MAX_ROWS
            denominator_base = numerator_base + MAX_ROWS
            count = 0
            for row in range(self._current_count):
                output_base = row * _OUTPUT_STRIDE + side * 4
                if not output[output_base + 2]:
                    continue
                numerator = output[output_base]
                denominator = max(2, output[output_base + 1])
                work[numerator_base + count] = numerator
                work[denominator_base + count] = denominator
                count += 1
            if not count:
                summary[2 + side * 2] = _NO_MEDIAN
                summary[3 + side * 2] = 0
                continue
            for index in range(1, count):
                key_n = work[numerator_base + index]
                key_d = work[denominator_base + index]
                cursor = index - 1
                while cursor >= 0 and \
                        work[numerator_base + cursor] * key_d > \
                        key_n * work[denominator_base + cursor]:
                    work[numerator_base + cursor + 1] = \
                        work[numerator_base + cursor]
                    work[denominator_base + cursor + 1] = \
                        work[denominator_base + cursor]
                    cursor -= 1
                work[numerator_base + cursor + 1] = key_n
                work[denominator_base + cursor + 1] = key_d
            middle = count // 2
            if count & 1:
                summary[2 + side * 2] = work[numerator_base + middle]
                summary[3 + side * 2] = work[denominator_base + middle]
            else:
                left_n = work[numerator_base + middle - 1]
                left_d = work[denominator_base + middle - 1]
                right_n = work[numerator_base + middle]
                right_d = work[denominator_base + middle]
                summary[2 + side * 2] = left_n * right_d + right_n * left_d
                summary[3 + side * 2] = 2 * left_d * right_d

    @micropython.native
    def _publish_summary(self, pair_count):
        metrics = self._metrics
        evidence = self.evidence
        bilateral = metrics[0]
        target_support = metrics[1]
        if pair_count < 4:
            reason = "INSUFFICIENT_FRESH_SUPPORT"
        elif bilateral * FRACTION_SCALE < \
                pair_count * self._minimum_ratio_scaled:
            reason = "LOW_PAIR_SUPPORT"
        elif target_support < self._minimum_target_support:
            reason = "NO_TARGET_SUPPORT"
        else:
            reason = "OK"
        self._reset_evidence(reason, pair_count, bilateral, target_support)
        evidence["left_supported"] = metrics[2]
        evidence["right_supported"] = metrics[3]
        summaries = self._summary_values
        if summaries[0] != _NO_MEDIAN:
            evidence["left_step_median"] = summaries[0] / 4.0
        if summaries[1] != _NO_MEDIAN:
            evidence["right_step_median"] = summaries[1] / 4.0
        if summaries[2] != _NO_MEDIAN and summaries[3] > 0:
            evidence["left_step_fraction_median"] = \
                summaries[2] / float(summaries[3])
        if summaries[4] != _NO_MEDIAN and summaries[5] > 0:
            evidence["right_step_fraction_median"] = \
                summaries[4] / float(summaries[5])
        return reason

    @micropython.native
    def _reset_candidate(self, candidate, reason):
        self._support_count = 0
        self._current_count = 0
        self._target_available = 0
        self._reset_evidence(reason)
        self._invalidate(candidate, self.evidence)
        return candidate

    @micropython.native
    def _check_masks(self, luma, edge_mask, road_mask, known_region):
        if (luma is None or edge_mask is None or road_mask is None or
                known_region is None):
            return "MISSING_FRAME_MASK"
        try:
            sizes = (len(luma), len(edge_mask), len(road_mask),
                     len(known_region))
        except TypeError:
            return "FRAME_SIZE_MISMATCH"
        if sizes != (PIXELS, PIXELS, PIXELS, PIXELS):
            return "FRAME_SIZE_MISMATCH"
        # The board path consumes FastGrayMasks bytearray planes directly.
        # Host list/sequence doubles remain supported by the scalar fallback.
        if _IS_MICROPYTHON:
            if (not isinstance(luma, bytearray) or
                    not isinstance(edge_mask, bytearray) or
                    not isinstance(road_mask, bytearray) or
                    not isinstance(known_region, bytearray)):
                return "FRAME_BUFFER_TYPE"
        return None

    @micropython.native
    def _run_staged_gate(self, pair_count):
        if _IS_MICROPYTHON:
            _gate_viper(self)
            if self.diagnostics:
                _summary_viper(self)
        else:
            self._host_gate()
            if self.diagnostics:
                self._host_summary()
        reason = self._publish_summary(pair_count)
        if reason != "OK":
            return reason
        return "OK"

    @micropython.native
    def _validated_path_array(self, rows):
        """Validate one reusable 32-bit array without assuming typecode attr.

        OpenMV MicroPython arrays expose the buffer protocol, but unlike
        CPython builds do not expose ``array.typecode`` as a Python attribute.
        Cache the memoryview for the array's lifetime and validate its element
        width once; FastPathDetector owns this buffer and guarantees its signed
        ``array('i')`` construction.
        """
        if not isinstance(rows, array):
            return False
        if rows is self._path_rows_owner:
            return self._path_rows_view is not None
        try:
            view = memoryview(rows)
            itemsize = getattr(view, "itemsize", None)
        except (TypeError, AttributeError):
            return False
        if itemsize is None:
            # OpenMV v5's memoryview omits itemsize/format. Its bytes() export
            # gives a one-time physical size check; the 960-byte copy is
            # discarded before the frame loop and the cached view is reused.
            try:
                raw_bytes = bytes(view)
                valid_size = len(raw_bytes) == len(rows) * 4
                del raw_bytes
                if not valid_size:
                    return False
            except (TypeError, ValueError, MemoryError):
                return False
        elif itemsize != 4:
            return False
        fmt = getattr(view, "format", None)
        if fmt is not None and fmt not in ("i", "l", "@i", "@l", "=i", "=l"):
            return False
        code = getattr(rows, "typecode", None)
        if code is not None and code != "i":
            return False
        try:
            if len(rows) == 0 or not isinstance(rows[0], int):
                return False
        except (IndexError, TypeError):
            return False
        # On ports that omit the metadata, acceptance relies on the documented
        # producer contract: FastPathDetector owns and reuses array('i').
        self._path_rows_owner = rows
        self._path_rows_view = view
        return True

    @micropython.native
    def gate_road_candidate(self, candidate, luma, edge_mask, road_mask,
                            known_region, settings=None):
        """Gate an already selected road path with same-frame raw gray steps."""
        if not candidate or not candidate.get("valid"):
            return candidate
        if candidate.get("selected_row_ints") is not None:
            return self.gate_fast_path_candidate(
                candidate, luma, edge_mask, road_mask, known_region, settings)
        source = self.settings if settings is None else settings
        self._read_limits(source)
        minimum_step, _fraction, _ratio, _radius, _minimum_support = \
            self._limits
        evidence = self.evidence
        evidence["minimum_step"] = minimum_step
        mask_error = self._check_masks(
            luma, edge_mask, road_mask, known_region)
        if mask_error is not None:
            self._reset_candidate(candidate, mask_error)
            return candidate

        rows = candidate.get("selected_rows") or ()
        pair_count = len(rows)
        if pair_count > MAX_ROWS:
            self._reset_candidate(candidate, "FRAME_SIZE_MISMATCH")
            return candidate
        target = self._road_preselection_reference(candidate)
        self._current_luma = luma
        self._current_dark = edge_mask
        self._current_white = road_mask
        self._current_known = known_region
        self._stage_rows(rows, pair_count, target)
        reason = self._run_staged_gate(pair_count)
        if reason != "OK":
            self._invalidate(candidate, evidence)
        else:
            candidate["boundary_evidence"] = evidence
        return candidate

    @micropython.native
    def gate_fast_path_candidate(self, candidate, luma, edge_mask,
                                 road_mask, known_region, settings=None):
        """Gate a candidate directly from FastPathDetector's int row plane.

        The detector's reusable array is read in place as
        ``[y,left,right,flags]`` records. The array must outlive this call;
        no selected-row dictionaries or per-row staging objects are built.
        """
        if not candidate or not candidate.get("valid"):
            return candidate
        source = self.settings if settings is None else settings
        self._read_limits(source)
        evidence = self.evidence
        mask_error = self._check_masks(
            luma, edge_mask, road_mask, known_region)
        if mask_error is not None:
            self._reset_candidate(candidate, mask_error)
            return candidate

        rows = candidate.get("selected_row_ints")
        count = candidate.get("selected_row_count")
        stride = candidate.get("selected_row_stride")
        if (rows is None or isinstance(count, bool) or
                not isinstance(count, int) or count < 0 or
                count > MAX_ROWS or isinstance(stride, bool) or
                not isinstance(stride, int) or stride != 4 or
                not self._validated_path_array(rows)):
            self._reset_candidate(candidate, "INVALID_SELECTED_ROWS")
            return candidate
        try:
            row_buffer_length = len(rows)
        except TypeError:
            self._reset_candidate(candidate, "INVALID_SELECTED_ROWS")
            return candidate
        if row_buffer_length < count * stride:
            self._reset_candidate(candidate, "INVALID_SELECTED_ROWS")
            return candidate
        selected_rows = candidate.get("selected_rows")
        if selected_rows is not None:
            try:
                if len(selected_rows) != count:
                    self._reset_candidate(candidate, "INVALID_SELECTED_ROWS")
                    return candidate
            except TypeError:
                self._reset_candidate(candidate, "INVALID_SELECTED_ROWS")
                return candidate

        self._current_luma = luma
        self._current_dark = edge_mask
        self._current_white = road_mask
        self._current_known = known_region
        self._current_path_rows = rows
        self._current_count = count
        self._current_stride = stride
        self._support_count = 0
        self._stage_target(self._road_preselection_reference(candidate))
        if _IS_MICROPYTHON:
            _stage_path_viper(self)
        else:
            self._stage_path_host()
        reason = self._run_staged_gate(count)
        if reason != "OK":
            self._invalidate(candidate, evidence)
        else:
            candidate["boundary_evidence"] = evidence
        return candidate

    @micropython.native
    def _road_preselection_reference(self, candidate):
        reference_x = candidate.get("reference_center_x")
        reference_y = candidate.get("reference_y")
        if reference_x is not None and reference_y is not None:
            return (float(reference_x), float(reference_y), "reference_y")
        points = candidate.get("selected_rows") or ()
        if not points:
            return None
        weights = candidate.get("weights") or ()
        if len(weights) != len(points):
            weight = 1.0 / len(points)
            center_y = 0.0
            for point in points:
                center_y += float(point["y"]) * weight
        else:
            center_y = 0.0
            for point, weight in zip(points, weights):
                center_y += float(point["y"]) * float(weight)
        center_x = candidate.get("center_x")
        if center_x is None:
            return None
        return (float(center_x), center_y, "weighted_center")

    @micropython.native
    def validate_selected_reference(self, selection, reference,
                                    surface_result, road_result,
                                    settings=None):
        """Check final road controller point against existing support only."""
        if not selection or not selection.get("valid"):
            return {"valid": False, "reason": "SELECTION_INVALID",
                    "support_count": 0}
        source = selection.get("source")
        candidate = road_result if source == "road" else None
        if candidate is None or not candidate.get("valid"):
            return {"valid": False, "reason": "SOURCE_CANDIDATE_INVALID",
                    "support_count": 0}
        if (not isinstance(reference, dict) or reference.get("x") is None or
                reference.get("y") is None):
            reason = "REFERENCE_UNAVAILABLE"
            evidence = candidate.get("boundary_evidence") or self.evidence
            evidence["status"] = "REJECTED"
            evidence["selected_reference_reason"] = reason
            evidence["selected_reference_support_count"] = 0
            self._invalidate(candidate, evidence,
                             "BOUNDARY_REFERENCE_UNSUPPORTED")
            return {"valid": False, "reason": reason, "support_count": 0}

        evidence = candidate.get("boundary_evidence")
        if evidence is not self.evidence or evidence.get("status") != "PASS":
            reason = "NO_PRESELECTION_EVIDENCE"
            if evidence is None:
                self._reset_evidence(reason)
                evidence = self.evidence
            self._invalidate(candidate, evidence,
                             "BOUNDARY_REFERENCE_UNSUPPORTED")
            return {"valid": False, "reason": reason, "support_count": 0}

        source_settings = self.settings if settings is None else settings
        self._read_limits(source_settings)
        try:
            target_x = float(reference["x"])
            target_y = float(reference["y"])
        except (KeyError, TypeError, ValueError, OverflowError):
            target_x = None
            target_y = None
        self._stage_target((target_x, target_y)
                           if target_x is not None and target_y is not None
                           else None)
        if not self._target_available:
            support_count = 0
        elif _IS_MICROPYTHON:
            support_count = _reference_viper(self)
        else:
            support_count = self._host_reference()

        if support_count < self._minimum_target_support:
            reason = "NO_SELECTED_REFERENCE_SUPPORT"
            evidence["preselection_status"] = evidence.get("status")
            evidence["status"] = "REJECTED"
            evidence["selected_reference_reason"] = reason
            evidence["selected_reference_support_count"] = support_count
            self._invalidate(candidate, evidence,
                             "BOUNDARY_REFERENCE_UNSUPPORTED")
            return {"valid": False, "reason": reason,
                    "support_count": support_count}

        evidence["selected_reference_status"] = "PASS"
        evidence["selected_reference_support_count"] = support_count
        evidence["selected_reference_reason"] = None
        evidence["selected_reference"] = (
            target_x, target_y, reference.get("kind"))
        return {"valid": True, "reason": "OK",
                "support_count": support_count}

    def _host_reference(self):
        radius = self._target_radius_q
        count = 0
        for index in range(self._support_count):
            dx = self._target_x_q - self._support_x[index]
            dy = self._target_y_q - self._support_y[index]
            if (abs(dx) <= radius and abs(dy) <= radius and
                    dx * dx + dy * dy <= radius * radius):
                count += 1
        return count


__all__ = ("FastBoundaryEvidence", "RUNTIME_API_VERSION", "COMPILER_BACKEND")
