"""Bounded Viper single-edge steering backed by accepted bilateral rows.

This module is a fallback for frames where the fast bilateral path has no
usable result. It searches current-frame dark-to-bright or bright-to-dark
transitions, anchors them to per-row boundaries learned only from an accepted
bilateral observation, and derives the missing edge from that row's smoothed
width. All pixel scans and cache updates use fixed-size buffers.
"""

from array import array

try:
    import micropython
    COMPILER_BACKEND = "micropython-viper"
    _IS_MICROPYTHON = True
except ImportError:  # Host tests only; OpenMV provides the real module.
    _IS_MICROPYTHON = False
    COMPILER_BACKEND = "cpython-shim"

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

    def ptr32(buffer):
        return buffer


RUNTIME_API_VERSION = 2
WIDTH = 80
HEIGHT = 60
MAX_ROWS = 60
ROW_STRIDE = 5

ROUTE_CACHED_WIDTH = "cached_width"
ROUTE_CLIPPED_MIDPOINT = "clipped_midpoint"
_ROUTE_CACHED_WIDTH = 0
_ROUTE_CLIPPED_MIDPOINT = 1

FLAG_OBSERVED_LEFT = 1
FLAG_OBSERVED_RIGHT = 2
FLAG_INFERRED_LEFT = 4
FLAG_INFERRED_RIGHT = 8
FLAG_WIDTH_CACHE = 16
FLAG_IDENTITY_ANCHORED = 32
FLAG_CLIPPED_FRAME_LEFT = 64
FLAG_CLIPPED_FRAME_RIGHT = 128
INFERENCE_USED_WIDTH_CACHE = 1
INFERENCE_USED_CURRENT_EDGE = 2
INFERENCE_USED_CONTINUITY = 4
INFERENCE_USED_FRAME_CLIP = 8

_NO_CANDIDATE = -1
_AMBIGUOUS = 2


def _setting(settings, name, default):
    if isinstance(settings, dict):
        return settings.get(name, default)
    return getattr(settings, name, default)


def _ceil_scaled(value, scale):
    scaled = float(value) * scale
    integer = int(scaled)
    if integer < scaled:
        integer += 1
    return integer


@micropython.viper
def _step_supported_viper(self, hit: int, direction: int) -> int:
    """Check one dark/bright transition with the frozen V7 step limits."""
    luma = ptr8(self._current_luma)
    road = ptr8(self._current_road)
    known = ptr8(self._current_known)
    width = int(self.width)
    total = width * int(self.height)
    if hit < 0 or hit >= total or known[hit] == 0:
        return 0
    dark = int(luma[hit])
    values = ptr32(self._sample_work)
    count = 0
    for index in range(3):
        sample = hit + direction * (index + 1)
        if sample < 0 or sample >= total:
            return 0
        if known[sample] == 0 or road[sample] == 0:
            return 0
        values[index] = int(luma[sample])
        count += 1
    value0 = int(values[0])
    value1 = int(values[1])
    value2 = int(values[2])
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
    step2 = white2 - dark * 2
    if step2 * 4 < int(self.minimum_step_scaled) * 2:
        return 0
    if step2 * 1000000 < white2 * int(self.minimum_fraction_scaled):
        return 0
    return 1


@micropython.viper
def _label_dark_components_viper(self) -> int:
    """Mark dark components that touch the known ROI/crop boundary.

    This bounded 8-connected pass runs only when the independent topology
    guard reports an island. Component classes are 1 for closed internal
    regions and 2 for regions connected to a visible/cropped ROI boundary.
    """
    edge = ptr8(self._current_edge)
    known = ptr8(self._current_known)
    classes = ptr8(self._component_class)
    queue = ptr16(self._component_queue)
    width = int(self.width)
    height = int(self.height)
    floor = int(self.floor_y)
    total = width * height

    for index in range(total):
        classes[index] = 0

    for y in range(floor, height):
        row = y * width
        for x in range(width):
            start = row + x
            if (known[start] == 0 or edge[start] == 0 or
                    classes[start] != 0):
                continue

            classes[start] = 3
            queue_count = 1
            queue[0] = start
            queue_head = 0
            touches_roi = 0
            while queue_head < queue_count:
                index = int(queue[queue_head])
                queue_head += 1
                pixel_y = index // width
                pixel_x = index - pixel_y * width
                if (pixel_x == 0 or pixel_x == width - 1 or
                        pixel_y == floor or pixel_y == height - 1):
                    touches_roi = 1

                for dy in range(-1, 2):
                    neighbor_y = pixel_y + dy
                    for dx in range(-1, 2):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor_x = pixel_x + dx
                        if (neighbor_x < 0 or neighbor_x >= width or
                                neighbor_y < floor or
                                neighbor_y >= height):
                            touches_roi = 1
                            continue
                        neighbor = neighbor_y * width + neighbor_x
                        if known[neighbor] == 0:
                            # A dark component cut by an unknown crop is an
                            # observed boundary, not an internal object.
                            touches_roi = 1
                            continue
                        if edge[neighbor] == 0 or classes[neighbor] != 0:
                            continue
                        if queue_count >= total:
                            return -1
                        classes[neighbor] = 3
                        queue[queue_count] = neighbor
                        queue_count += 1

            component_class = 2 if touches_roi != 0 else 1
            for queue_index in range(queue_count):
                classes[int(queue[queue_index])] = component_class
    return 0


@micropython.viper
def _observe_bilateral_viper(self) -> int:
    """Cache only current rows with measured, validated left and right steps."""
    rows = ptr32(self._observed_rows)
    luma = ptr8(self._current_luma)
    road = ptr8(self._current_road)
    edge = ptr8(self._current_edge)
    known = ptr8(self._current_known)
    cache_valid = ptr8(self._cache_valid)
    cache_width = ptr32(self._cache_width)
    cache_left2 = ptr32(self._cache_left2)
    cache_right2 = ptr32(self._cache_right2)
    anchor_left2 = ptr32(self._anchor_left2)
    anchor_right2 = ptr32(self._anchor_right2)
    width = int(self.width)
    height = int(self.height)
    floor = int(self.floor_y)
    count = int(self._observed_count)
    stride = int(self._observed_stride)
    minimum_width = int(self.min_width)
    maximum_width = int(self.max_width)
    total = width * height
    accepted = 0

    for row_index in range(count):
        base = row_index * stride
        y = int(rows[base])
        left = int(rows[base + 1])
        right = int(rows[base + 2])
        flags = int(rows[base + 3])
        if (y < floor or y >= height or left < 1 or right < left or
                right >= width - 1 or (flags & 7) != 7):
            continue
        run_width = right - left + 1
        if run_width < minimum_width or run_width > maximum_width:
            continue
        row = y * width
        left_hit = row + left - 1
        right_hit = row + right + 1
        if (left_hit < 0 or right_hit >= total or
                known[left_hit] == 0 or known[right_hit] == 0 or
                edge[left_hit] == 0 or edge[right_hit] == 0 or
                road[left_hit] != 0 or road[right_hit] != 0):
            continue
        left_ok = 1
        right_ok = 1
        for sample in range(3):
            left_sample = row + left + sample
            right_sample = row + right - sample
            if (left_sample >= width * (y + 1) or
                    right_sample < row or known[left_sample] == 0 or
                    known[right_sample] == 0 or road[left_sample] == 0 or
                    road[right_sample] == 0):
                left_ok = 0
                right_ok = 0
        if (left_ok == 0 or right_ok == 0 or
                int(_step_supported_viper(self, left_hit, 1)) == 0 or
                int(_step_supported_viper(self, right_hit, -1)) == 0):
            continue

        left2 = left * 2 - 1
        right2 = right * 2 + 1
        if cache_valid[y] == 0:
            cache_width[y] = run_width
            cache_left2[y] = left2
            cache_right2[y] = right2
            anchor_left2[y] = left2
            anchor_right2[y] = right2
            cache_valid[y] = 1
        else:
            delta = run_width - int(cache_width[y])
            if delta > 4:
                delta = 4
            elif delta < -4:
                delta = -4
            if delta >= 0:
                cache_width[y] = int(cache_width[y]) + (delta + 1) // 2
            else:
                cache_width[y] = int(cache_width[y]) - ((-delta + 1) // 2)

            delta = left2 - int(cache_left2[y])
            if delta > 8:
                delta = 8
            elif delta < -8:
                delta = -8
            if delta >= 0:
                cache_left2[y] = int(cache_left2[y]) + (delta + 1) // 2
            else:
                cache_left2[y] = int(cache_left2[y]) - ((-delta + 1) // 2)

            delta = right2 - int(cache_right2[y])
            if delta > 8:
                delta = 8
            elif delta < -8:
                delta = -8
            if delta >= 0:
                cache_right2[y] = int(cache_right2[y]) + (delta + 1) // 2
            else:
                cache_right2[y] = int(cache_right2[y]) - ((-delta + 1) // 2)

            anchor_left2[y] = cache_left2[y]
            anchor_right2[y] = cache_right2[y]
        accepted += 1
    return accepted


@micropython.viper
def _scan_single_edges_viper(self) -> int:
    """Find current-frame one-sided transitions for either route."""
    luma = ptr8(self._current_luma)
    road = ptr8(self._current_road)
    edge = ptr8(self._current_edge)
    known = ptr8(self._current_known)
    cache_valid = ptr8(self._cache_valid)
    cache_width = ptr32(self._cache_width)
    anchor_left2 = ptr32(self._anchor_left2)
    anchor_right2 = ptr32(self._anchor_right2)
    status = ptr8(self._row_status)
    row_edge2 = ptr32(self._row_edge2)
    row_center2 = ptr32(self._row_center2)
    row_contrast = ptr32(self._row_contrast)
    component_class = ptr8(self._component_class)
    width = int(self.width)
    height = int(self.height)
    floor = int(self.floor_y)
    route_mode = int(self._route_mode_id)
    cached_route = 1
    if route_mode != 0:
        cached_route = 0
    max_anchor2 = int(self.max_anchor2)
    ambiguity_margin2 = int(self.ambiguity_margin2)
    ambiguity_separation2 = int(self.ambiguity_separation2)
    island_check = int(self._island_check)
    total = width * height
    no_distance = int(2147483647)

    for index in range(2 * height):
        status[index] = 0
        row_edge2[index] = 0
        row_center2[index] = 0
        row_contrast[index] = 0

    for y in range(floor, height):
        if cached_route != 0 and cache_valid[y] == 0:
            continue
        row = y * width
        if cached_route != 0:
            cached_width = int(cache_width[y])
            if (cached_width < int(self.min_width) or
                    cached_width > int(self.max_width)):
                continue
        else:
            cached_width = 0

        for side in range(2):
            if cached_route != 0:
                anchor2 = (int(anchor_left2[y]) if side == 0 else
                           int(anchor_right2[y]))
            else:
                anchor2 = 0
            best_distance = no_distance
            second_distance = no_distance
            best_edge2 = 0
            second_edge2 = 0
            best_center2 = 0
            best_contrast = 0
            clipped_ambiguous = 0
            opposite_seen = 0

            for x in range(1, width - 1):
                hit = row + x
                if known[hit] == 0 or edge[hit] == 0:
                    continue
                if island_check != 0 and component_class[hit] != 2:
                    continue

                if side == 0:
                    if (x + 3 >= width or known[hit + 1] == 0 or
                            road[hit + 1] == 0):
                        continue
                    if (int(_step_supported_viper(self, hit, 1)) == 0):
                        continue
                    edge2 = x * 2 + 1
                    corridor_first = x + 1
                    if cached_route != 0:
                        opposite = x + cached_width + 1
                        opposite_seen = 0
                        for adjust in range(-2, 3):
                            other = opposite + adjust
                            if (other > 0 and other < width and
                                    known[row + other] != 0 and
                                    known[row + other - 1] != 0 and
                                    edge[row + other] != 0 and
                                    road[row + other - 1] != 0):
                                opposite_seen = 1
                        center2 = edge2 + cached_width
                        corridor_last = center2 // 2
                        anchor_edge2 = anchor2
                    else:
                        # edge2 is twice the measured dark/white transition
                        # coordinate. Round the midpoint to the nearest
                        # half-cell because the shared row buffer stores 2x.
                        center2 = (edge2 + (width - 1) * 2 + 1) // 2
                        corridor_last = width - 1
                        anchor_edge2 = 0
                else:
                    if (x < 3 or known[hit - 1] == 0 or
                            road[hit - 1] == 0):
                        continue
                    if (int(_step_supported_viper(self, hit, -1)) == 0):
                        continue
                    edge2 = x * 2 - 1
                    if cached_route != 0:
                        opposite = x - cached_width - 1
                        opposite_seen = 0
                        for adjust in range(-2, 3):
                            other = opposite + adjust
                            if (other > 0 and other < width - 1 and
                                    known[row + other] != 0 and
                                    known[row + other + 1] != 0 and
                                    edge[row + other] != 0 and
                                    road[row + other + 1] != 0):
                                opposite_seen = 1
                        center2 = edge2 - cached_width
                        corridor_first = center2 // 2
                        anchor_edge2 = anchor2
                    else:
                        center2 = (edge2 + 1) // 2
                        corridor_first = 0
                        anchor_edge2 = 0
                    corridor_last = x - 1

                if cached_route != 0 and opposite_seen != 0:
                    continue
                if center2 < 0 or center2 > (width - 1) * 2:
                    continue
                corridor_clear = 1
                for corridor_x in range(corridor_first,
                                        corridor_last + 1):
                    corridor_index = row + corridor_x
                    if (corridor_x < 0 or corridor_x >= width or
                            known[corridor_index] == 0 or
                            road[corridor_index] == 0):
                        corridor_clear = 0
                        break
                if corridor_clear == 0:
                    continue
                if cached_route != 0:
                    distance = edge2 - anchor_edge2
                    if distance < 0:
                        distance = -distance
                    if distance > max_anchor2:
                        continue

                    if distance < best_distance:
                        second_distance = best_distance
                        second_edge2 = best_edge2
                        best_distance = distance
                        best_edge2 = edge2
                        best_center2 = center2
                        best_contrast = int(luma[hit])
                    elif distance < second_distance:
                        second_distance = distance
                        second_edge2 = edge2
                elif best_distance == no_distance:
                    best_distance = 0
                    best_edge2 = edge2
                    best_center2 = center2
                    best_contrast = int(luma[hit])
                else:
                    separation = edge2 - best_edge2
                    if separation < 0:
                        separation = -separation
                    if separation >= ambiguity_separation2:
                        clipped_ambiguous = 1
                    elif int(luma[hit]) < best_contrast:
                        best_edge2 = edge2
                        best_center2 = center2
                        best_contrast = int(luma[hit])

            slot = side * height + y
            if best_distance == no_distance:
                continue
            if (cached_route == 0 and clipped_ambiguous != 0):
                status[slot] = 2
                continue
            if (cached_route != 0 and second_distance != no_distance and
                    int(abs(second_edge2 - best_edge2)) >=
                    ambiguity_separation2 and
                    second_distance - best_distance <= ambiguity_margin2):
                status[slot] = 2
                continue
            status[slot] = 1
            row_edge2[slot] = best_edge2
            row_center2[slot] = best_center2
            row_contrast[slot] = best_contrast
    return 0


class FastSingleEdgeTracker:
    """Keep validated bilateral widths and infer one current visible side."""

    def __init__(self, settings):
        self.settings = settings
        self.width = int(_setting(settings, "SURFACE_GRID_WIDTH", WIDTH))
        self.height = int(_setting(settings, "SURFACE_GRID_HEIGHT", HEIGHT))
        if self.width != WIDTH or self.height != HEIGHT:
            raise ValueError("FastSingleEdgeTracker requires an 80x60 grid")
        self.floor_y = int(_setting(settings, "SURFACE_ROI_FLOOR_Y", 20))
        if self.floor_y < 0 or self.floor_y >= self.height:
            raise ValueError("SURFACE_ROI_FLOOR_Y must be in 0..59")
        self.reference_y = float(_setting(settings, "ROAD_REFERENCE_Y", 40))
        if (self.reference_y != self.reference_y or
                self.reference_y < self.floor_y or
                self.reference_y >= self.height):
            raise ValueError("ROAD_REFERENCE_Y must lie inside the ROI")
        self.min_width = int(_setting(settings, "ROAD_MIN_WIDTH", 8))
        self.max_width = int(_setting(settings, "ROAD_MAX_WIDTH", 70))
        if (self.min_width < 8 or self.max_width > 70 or
                self.max_width < self.min_width):
            raise ValueError("ROAD width must stay within 8..70 cells")
        self.max_gap = int(_setting(settings, "ROAD_MAX_GAP_ROWS", 2))
        self.max_slope_milli = int(float(_setting(
            settings, "ROAD_MAX_CENTER_SLOPE", 2.5)) * 1000.0 + 0.5)
        self.link_tolerance_milli = int(float(_setting(
            settings, "ROAD_LINK_TOLERANCE", 2.5)) * 1000.0 + 0.5)
        self.max_anchor2 = int(float(_setting(
            settings, "SINGLE_EDGE_ANCHOR_RADIUS_CELLS", 10.0)) * 2.0 + 0.5)
        self.ambiguity_margin2 = int(float(_setting(
            settings, "SINGLE_EDGE_AMBIGUITY_MARGIN_CELLS", 2.0)) * 2.0 + 0.5)
        self.ambiguity_separation2 = int(float(_setting(
            settings, "SINGLE_EDGE_AMBIGUITY_SEPARATION_CELLS", 4.0)) *
            2.0 + 0.5)
        self.min_rows = int(_setting(settings, "SINGLE_EDGE_MIN_ROWS", 8))
        self.min_span = int(_setting(settings, "SINGLE_EDGE_MIN_SPAN", 12))
        self.reference_max_gap = int(_setting(
            settings, "SINGLE_EDGE_REFERENCE_MAX_GAP", 4))
        self.route_mode = _setting(
            settings, "SINGLE_EDGE_ROUTE_MODE", ROUTE_CACHED_WIDTH)
        if self.route_mode == ROUTE_CACHED_WIDTH:
            self._route_mode_id = _ROUTE_CACHED_WIDTH
        elif self.route_mode == ROUTE_CLIPPED_MIDPOINT:
            self._route_mode_id = _ROUTE_CLIPPED_MIDPOINT
        else:
            raise ValueError(
                "SINGLE_EDGE_ROUTE_MODE must be cached_width or "
                "clipped_midpoint")
        if (self.max_anchor2 < 2 or self.max_anchor2 > self.width * 2 or
                self.ambiguity_margin2 < 0 or
                self.ambiguity_separation2 < 2 or self.min_rows < 4 or
                self.min_rows > self.height - self.floor_y or
                self.min_span < 4 or
                self.min_span >= self.height - self.floor_y or
                self.reference_max_gap < 0 or
                self.reference_max_gap > self.height - 1):
            raise ValueError("invalid single-edge geometry settings")

        minimum_step = float(_setting(settings, "GRAY_BOUNDARY_MIN_STEP", 10.0))
        minimum_fraction = float(_setting(
            settings, "GRAY_BOUNDARY_MIN_STEP_FRACTION", 0.25))
        if (minimum_step <= 0.0 or minimum_step > 255.0 or
                minimum_fraction < 0.0 or minimum_fraction >= 1.0):
            raise ValueError("invalid single-edge boundary step settings")
        self.minimum_step_scaled = _ceil_scaled(minimum_step, 4)
        self.minimum_fraction_scaled = _ceil_scaled(minimum_fraction, 1000000)

        self._cache_valid = bytearray(self.height)
        self._cache_width = array("i", [0] * self.height)
        self._cache_left2 = array("i", [0] * self.height)
        self._cache_right2 = array("i", [0] * self.height)
        self._anchor_left2 = array("i", [0] * self.height)
        self._anchor_right2 = array("i", [0] * self.height)
        self._row_status = bytearray(2 * self.height)
        self._row_edge2 = array("i", [0] * (2 * self.height))
        self._row_center2 = array("i", [0] * (2 * self.height))
        self._row_contrast = array("i", [0] * (2 * self.height))
        self._side_valid = bytearray(2)
        self._side_ambiguous = bytearray(2)
        self._side_count = array("i", [0, 0])
        self._side_first = array("i", [0, 0])
        self._side_last = array("i", [0, 0])
        self._side_ref_y = array("i", [0, 0])
        self._side_ref_center2 = array("i", [0, 0])
        self._selected_row_ints = array("i", [0] * (self.height * ROW_STRIDE))
        self._sample_work = array("i", [0, 0, 0])
        # Preallocated topology scratch is used only when an island is
        # observed; no per-frame pixel arrays are allocated in detect().
        self._component_class = bytearray(self.width * self.height)
        self._component_queue = array("H", [0] * (self.width * self.height))
        self._island_check = 0
        self._current_luma = None
        self._current_road = None
        self._current_edge = None
        self._current_known = None
        self._observed_rows = None
        self._observed_count = 0
        self._observed_stride = 4
        self._cache_row_count = 0
        self._result = {
            "valid": False,
            "quality": 0,
            "reason": "NO_WIDTH_CACHE",
            "boundary_source": "NONE",
            "inferred": False,
            "inference_flags": 0,
            "observed_both_rows": 0,
            "observed_left_rows": 0,
            "observed_right_rows": 0,
            "inferred_rows": 0,
            "selected_row_ints": self._selected_row_ints,
            "selected_row_count": 0,
            "selected_row_stride": ROW_STRIDE,
            "reference_center_x": None,
            "reference_y": None,
            "reference_provenance": "NO_COMMON_REFERENCE",
            "lateral_error": 0,
            "topology_reason": "NOT_RUN",
            "topology_island_observed": False,
            "topology_identity_resolved": False,
            "cache_rows": 0,
            "route_mode": self.route_mode,
            "actual_boundary": "NONE",
            "actual_boundary_x": None,
            "clipped_endpoint": "NONE",
            "clipped_endpoint_x": None,
            "topology_checked": False,
        }
        self.last_clear_reason = "COLD_START"
        self.clear_count = 0
        self._last_single_side = -1

    @property
    def cache_rows(self):
        return self._cache_row_count

    @property
    def result(self):
        return self._result

    def clear(self, reason="CONTROL_RESET"):
        for y in range(self.height):
            self._cache_valid[y] = 0
            self._cache_width[y] = 0
            self._cache_left2[y] = 0
            self._cache_right2[y] = 0
            self._anchor_left2[y] = 0
            self._anchor_right2[y] = 0
        self._cache_row_count = 0
        self._last_single_side = -1
        self.clear_count += 1
        self.last_clear_reason = reason
        self._invalidate_result(reason)

    def _invalidate_result(self, reason):
        result = self._result
        result["valid"] = False
        result["quality"] = 0
        result["reason"] = reason
        result["boundary_source"] = "NONE"
        result["inferred"] = False
        result["inference_flags"] = 0
        result["observed_both_rows"] = 0
        result["observed_left_rows"] = 0
        result["observed_right_rows"] = 0
        result["inferred_rows"] = 0
        result["selected_row_count"] = 0
        result["reference_center_x"] = None
        result["reference_y"] = None
        result["reference_provenance"] = "NO_COMMON_REFERENCE"
        result["lateral_error"] = 0
        result["topology_island_observed"] = False
        result["topology_identity_resolved"] = False
        result["cache_rows"] = self._cache_row_count
        result["route_mode"] = self.route_mode
        result["actual_boundary"] = "NONE"
        result["actual_boundary_x"] = None
        result["clipped_endpoint"] = "NONE"
        result["clipped_endpoint_x"] = None
        result["topology_checked"] = False

    @micropython.native
    def observe_bilateral(self, candidate, luma, edge_mask, road_mask,
                          known_region, control_fresh, reference_valid):
        """Update width/edge anchors only after accepted dual-edge validation."""
        if not control_fresh or not reference_valid:
            return 0
        if not candidate or not candidate.get("valid"):
            return 0
        if self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT:
            # A successfully published bilateral frame ends the previous
            # one-sided identity. It never seeds or changes width cache data.
            self._last_single_side = -1
            return 0
        if self._route_mode_id != _ROUTE_CACHED_WIDTH:
            return 0
        rows = candidate.get("selected_row_ints")
        count = candidate.get("selected_row_count")
        stride = candidate.get("selected_row_stride")
        if (rows is None or isinstance(count, bool) or
                not isinstance(count, int) or count < 0 or count > MAX_ROWS or
                isinstance(stride, bool) or not isinstance(stride, int) or
                stride != 4):
            return 0
        try:
            if len(rows) < count * stride:
                return 0
        except TypeError:
            return 0
        if (not isinstance(luma, bytearray) or
                not isinstance(edge_mask, bytearray) or
                not isinstance(road_mask, bytearray) or
                not isinstance(known_region, bytearray) or
                len(luma) != self.width * self.height or
                len(edge_mask) != self.width * self.height or
                len(road_mask) != self.width * self.height or
                len(known_region) != self.width * self.height):
            return 0
        self._observed_rows = rows
        self._observed_count = count
        self._observed_stride = stride
        self._current_luma = luma
        self._current_edge = edge_mask
        self._current_road = road_mask
        self._current_known = known_region
        self._island_check = 0
        before = self._cache_row_count
        accepted = _observe_bilateral_viper(self)
        if _IS_MICROPYTHON:
            self._cache_row_count = 0
            for y in range(self.floor_y, self.height):
                if self._cache_valid[y]:
                    self._cache_row_count += 1
        else:
            self._cache_row_count = 0
            for y in range(self.floor_y, self.height):
                if self._cache_valid[y]:
                    self._cache_row_count += 1
        self._last_single_side = -1
        return accepted if accepted else max(0, self._cache_row_count - before)

    @micropython.native
    def commit_single(self, candidate):
        """Advance only the side actually observed in an accepted packet."""
        if not candidate or not candidate.get("valid"):
            return False
        source = candidate.get("boundary_source")
        side = 0 if source == "LEFT_ONLY" else 1 if source == "RIGHT_ONLY" \
            else -1
        rows = candidate.get("selected_row_ints")
        count = candidate.get("selected_row_count")
        stride = candidate.get("selected_row_stride")
        if (side < 0 or isinstance(count, bool) or
                not isinstance(count, int) or count < 1 or
                count > MAX_ROWS or stride != ROW_STRIDE or rows is None):
            return False
        if self._last_single_side >= 0 and self._last_single_side != side:
            return False
        before = self._last_single_side
        for index in range(count):
            offset = index * stride
            y = int(rows[offset])
            edge2 = int(rows[offset + 2])
            if (self._route_mode_id != _ROUTE_CACHED_WIDTH or
                    y < self.floor_y or y >= self.height or
                    not self._cache_valid[y]):
                continue
            anchors = self._anchor_left2 if side == 0 else self._anchor_right2
            delta = edge2 - int(anchors[y])
            if delta > 4:
                delta = 4
            elif delta < -4:
                delta = -4
            if delta >= 0:
                anchors[y] = int(anchors[y]) + (delta + 1) // 2
            else:
                anchors[y] = int(anchors[y]) - ((-delta + 1) // 2)
        self._last_single_side = side
        return before != side

    @micropython.native
    def _evaluate_side(self, side):
        """Validate one anchored candidate chain and choose its reference row."""
        height = self.height
        floor = self.floor_y
        base = side * height
        status = self._row_status
        centers = self._row_center2
        count = 0
        first_y = -1
        last_y = -1
        previous_y = -1
        previous_center2 = 0
        ambiguous = False
        continuity_failure = False
        reference_index = -1
        reference_delta = 1 << 30
        reference_y = self.reference_y

        for y in range(floor, height):
            row_status = status[base + y]
            if row_status == _AMBIGUOUS:
                ambiguous = True
                continue
            if row_status != 1:
                continue
            center2 = int(centers[base + y])
            if previous_y >= 0:
                dy = y - previous_y
                if dy > self.max_gap + 1:
                    continuity_failure = True
                    continue
                if abs(center2 - previous_center2) * 500 > (
                        self.max_slope_milli * dy +
                        self.link_tolerance_milli):
                    continuity_failure = True
            if first_y < 0:
                first_y = y
            last_y = y
            previous_y = y
            previous_center2 = center2
            count += 1
            delta = abs(float(y) - reference_y)
            if delta < reference_delta:
                reference_delta = delta
                reference_index = y

        self._side_count[side] = count
        self._side_first[side] = first_y if first_y >= 0 else 0
        self._side_last[side] = last_y if last_y >= 0 else 0
        self._side_ambiguous[side] = 1 if ambiguous else 0
        self._side_valid[side] = 0
        if ambiguous or continuity_failure:
            return False
        if (count < self.min_rows or first_y < 0 or last_y - first_y <
                self.min_span or reference_index < 0 or
                reference_delta > self.reference_max_gap):
            return False
        self._side_valid[side] = 1
        self._side_ref_y[side] = reference_index
        self._side_ref_center2[side] = int(
            centers[base + reference_index])
        return True

    @micropython.native
    def _publish_single(self, side, topology_reason,
                        topology_island_observed=False):
        result = self._result
        clipped_route = self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT
        height = self.height
        floor = self.floor_y
        base = side * height
        row_status = self._row_status
        row_edge2 = self._row_edge2
        row_center2 = self._row_center2
        output = self._selected_row_ints
        output_count = 0
        first_y = self._side_first[side]
        last_y = self._side_last[side]
        for y in range(first_y, last_y + 1):
            slot = base + y
            if row_status[slot] != 1:
                continue
            offset = output_count * ROW_STRIDE
            output[offset] = y
            output[offset + 1] = int(row_center2[slot])
            output[offset + 2] = int(row_edge2[slot])
            output[offset + 3] = side
            if side == 0:
                flags = FLAG_OBSERVED_LEFT | FLAG_INFERRED_RIGHT
                if clipped_route:
                    flags |= FLAG_CLIPPED_FRAME_RIGHT
                else:
                    flags |= FLAG_WIDTH_CACHE | FLAG_IDENTITY_ANCHORED
            else:
                flags = FLAG_OBSERVED_RIGHT | FLAG_INFERRED_LEFT
                if clipped_route:
                    flags |= FLAG_CLIPPED_FRAME_LEFT
                else:
                    flags |= FLAG_WIDTH_CACHE | FLAG_IDENTITY_ANCHORED
            output[offset + 4] = flags
            output_count += 1

        ref_y = int(self._side_ref_y[side])
        ref_center2 = int(self._side_ref_center2[side])
        if ref_center2 < 0 or ref_center2 > (self.width - 1) * 2:
            self._invalidate_result("INFERRED_CENTER_OUT_OF_FRAME")
            return self._result
        center = ref_center2 * 0.5
        half = (self.width - 1) * 0.5
        normalized = ((center - half) / half) * 1000.0
        if normalized >= 0.0:
            lateral_error = int(normalized + 0.5)
        else:
            lateral_error = -int(-normalized + 0.5)
        if lateral_error < -1000 or lateral_error > 1000:
            self._invalidate_result("INFERRED_ERROR_OUT_OF_RANGE")
            return self._result

        support_count = int(self._side_count[side])
        quality = 40 + support_count * 2
        if quality > 100:
            quality = 100
        result["valid"] = True
        result["quality"] = quality
        if clipped_route:
            result["reason"] = ("OK_CLIPPED_MIDPOINT_LEFT" if side == 0 else
                                 "OK_CLIPPED_MIDPOINT_RIGHT")
        else:
            result["reason"] = ("OK_SINGLE_EDGE_LEFT" if side == 0 else
                                 "OK_SINGLE_EDGE_RIGHT")
        result["boundary_source"] = "LEFT_ONLY" if side == 0 else \
            "RIGHT_ONLY"
        result["inferred"] = True
        if clipped_route:
            result["inference_flags"] = (
                INFERENCE_USED_CURRENT_EDGE | INFERENCE_USED_CONTINUITY |
                INFERENCE_USED_FRAME_CLIP)
        else:
            result["inference_flags"] = (
                INFERENCE_USED_WIDTH_CACHE | INFERENCE_USED_CURRENT_EDGE |
                INFERENCE_USED_CONTINUITY)
        result["observed_both_rows"] = 0
        result["observed_left_rows"] = support_count if side == 0 else 0
        result["observed_right_rows"] = support_count if side == 1 else 0
        result["inferred_rows"] = support_count
        result["selected_row_count"] = output_count
        result["reference_center_x"] = center
        result["reference_y"] = float(ref_y)
        result["reference_provenance"] = (
            ("CLIPPED_MIDPOINT_LEFT_TO_RIGHT_FRAME" if side == 0 else
             "CLIPPED_MIDPOINT_RIGHT_TO_LEFT_FRAME") if clipped_route else
            ("INFERRED_FROM_LEFT" if side == 0 else "INFERRED_FROM_RIGHT"))
        result["lateral_error"] = lateral_error
        result["topology_reason"] = topology_reason
        result["topology_island_observed"] = bool(
            topology_island_observed)
        result["actual_boundary"] = "LEFT" if side == 0 else "RIGHT"
        result["actual_boundary_x"] = int(
            row_edge2[base + ref_y]) * 0.5
        if clipped_route:
            result["clipped_endpoint"] = (
                "RIGHT_FRAME" if side == 0 else "LEFT_FRAME")
            result["clipped_endpoint_x"] = (
                self.width - 1 if side == 0 else 0)
            result["topology_identity_resolved"] = False
        else:
            # An island is not a blanket veto on the cached route: the
            # current measured side still has to follow its dual-seeded
            # identity anchor and retain a clear corridor to its inferred
            # center.
            result["topology_identity_resolved"] = bool(
                topology_island_observed)
        result["topology_checked"] = True
        result["cache_rows"] = self._cache_row_count
        return result

    @micropython.native
    def detect(self, mask_info, control_fresh, topology_evidence=None):
        """Return a single-edge result only inside a fresh RESET/KEEP session."""
        self._invalidate_result("NOT_RUN")
        if not control_fresh:
            self._invalidate_result("NO_FRESH_KEEP")
            return self._result
        if (self._route_mode_id == _ROUTE_CACHED_WIDTH and
                self._cache_row_count < self.min_rows):
            self._invalidate_result("NO_WIDTH_CACHE")
            return self._result
        if not isinstance(mask_info, dict):
            self._invalidate_result("INVALID_MASK_INFO")
            return self._result
        luma = mask_info.get("luma")
        road = mask_info.get("road_mask")
        edge = mask_info.get("edge_mask")
        known = mask_info.get("known_region")
        if (not isinstance(luma, bytearray) or
                not isinstance(road, bytearray) or
                not isinstance(edge, bytearray) or
                not isinstance(known, bytearray) or
                len(luma) != self.width * self.height or
                len(road) != self.width * self.height or
                len(edge) != self.width * self.height or
                len(known) != self.width * self.height or
                mask_info.get("width", self.width) != self.width or
                mask_info.get("height", self.height) != self.height or
                mask_info.get("floor", self.floor_y) != self.floor_y):
            self._invalidate_result("FRAME_SIZE_MISMATCH")
            return self._result
        topology = topology_evidence or {}
        self._result["topology_reason"] = topology.get("reason", "NOT_RUN")
        island = bool(topology.get("island_evidence") or topology.get("island"))
        ambiguous_topology = bool(topology.get("ambiguous"))
        if topology.get("branch_evidence") or topology.get("branch"):
            self._invalidate_result("TOPOLOGY_BRANCH_UNRESOLVED")
            self._result["topology_reason"] = topology.get("reason", "OPEN_BAND")
            return self._result
        if (self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT and island):
            self._invalidate_result("TOPOLOGY_ISLAND_UNRESOLVED")
            self._result["topology_reason"] = topology.get(
                "reason", "ISLAND")
            return self._result
        if (ambiguous_topology and
                (self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT or
                 not island)):
            self._invalidate_result("TOPOLOGY_AMBIGUITY_UNRESOLVED")
            self._result["topology_reason"] = topology.get(
                "reason", "AMBIGUOUS_TOPOLOGY")
            return self._result
        self._current_luma = luma
        self._current_road = road
        self._current_edge = edge
        self._current_known = known
        self._island_check = 1 if island else 0
        if island and _label_dark_components_viper(self) < 0:
            self._invalidate_result("TOPOLOGY_COMPONENT_BOUNDS")
            self._result["topology_reason"] = topology.get(
                "reason", "ISLAND")
            return self._result
        _scan_single_edges_viper(self)

        left_valid = self._evaluate_side(0)
        right_valid = self._evaluate_side(1)
        if self._side_ambiguous[0] or self._side_ambiguous[1]:
            self._invalidate_result("AMBIGUOUS_SINGLE_EDGE_IDENTITY")
            self._result["topology_reason"] = topology.get(
                "reason", "NOT_RUN")
            return self._result
        if left_valid and right_valid:
            self._invalidate_result("AMBIGUOUS_BOTH_SIDES")
            self._result["topology_reason"] = topology.get(
                "reason", "NOT_RUN")
            return self._result
        if left_valid:
            if self._last_single_side == 1:
                self._invalidate_result(
                    "SINGLE_EDGE_SIDE_SWITCH_REQUIRES_BILATERAL")
                self._result["topology_reason"] = topology.get(
                    "reason", "NOT_RUN")
                return self._result
            return self._publish_single(
                0, topology.get("reason", "NOT_RUN"), island)
        if right_valid:
            if self._last_single_side == 0:
                self._invalidate_result(
                    "SINGLE_EDGE_SIDE_SWITCH_REQUIRES_BILATERAL")
                self._result["topology_reason"] = topology.get(
                    "reason", "NOT_RUN")
                return self._result
            return self._publish_single(
                1, topology.get("reason", "NOT_RUN"), island)
        if self._side_count[0] or self._side_count[1]:
            self._invalidate_result("SINGLE_EDGE_CONTINUITY_OR_SUPPORT")
        elif self._route_mode_id == _ROUTE_CACHED_WIDTH and \
                self._cache_row_count < self.min_rows:
            self._invalidate_result("NO_WIDTH_CACHE")
        elif self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT:
            self._invalidate_result("NO_CURRENT_BOUNDARY_OR_WHITE_CLIP")
        else:
            self._invalidate_result("NO_ANCHORED_SINGLE_EDGE")
        self._result["topology_reason"] = topology.get("reason", "NOT_RUN")
        return self._result


__all__ = (
    "COMPILER_BACKEND", "FastSingleEdgeTracker",
    "FLAG_IDENTITY_ANCHORED", "FLAG_INFERRED_LEFT", "FLAG_INFERRED_RIGHT",
    "FLAG_OBSERVED_LEFT", "FLAG_OBSERVED_RIGHT", "FLAG_WIDTH_CACHE",
    "INFERENCE_USED_CONTINUITY", "INFERENCE_USED_CURRENT_EDGE",
    "INFERENCE_USED_WIDTH_CACHE", "RUNTIME_API_VERSION",
)
