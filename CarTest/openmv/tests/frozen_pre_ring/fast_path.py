"""Same-frame bounded bright-corridor detector for OpenMV."""

RUNTIME_API_VERSION = 7
# Required by the new package validator: old API-7 modules still enforced a
# minimum numerical quality score even though the wire field remains 0..100.
CONFIDENCE_DIAGNOSTIC_ONLY = True

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

    def ptr32(buffer):
        return buffer

from fast_topology import FastTopologyGuard


_DEFAULTS = {
    "SURFACE_GRID_WIDTH": 80,
    "SURFACE_GRID_HEIGHT": 60,
    "SURFACE_ROI_FLOOR_Y": 8,
    "ROAD_MIN_WIDTH": 8,
    "ROAD_MAX_WIDTH": 70,
    "ROAD_MAX_GAP_ROWS": 2,
    "ROAD_MAX_CENTER_SLOPE": 2.5,
    "ROAD_LINK_TOLERANCE": 2.5,
    "ROAD_MAX_WIDTH_CHANGE": 16,
    "ROAD_AMBIGUITY_MARGIN": 8.0,
    "ROAD_AMBIGUITY_SEPARATION": 8.0,
    "ROAD_REFERENCE_Y": 40,
    "ROAD_QUALITY_MIN": 60,
    "FAST_PATH_MAX_CANDIDATES_PER_ROW": 8,
    "FAST_PATH_MIN_OBSERVED_ROWS": 4,
    "FAST_PATH_MIN_VERTICAL_SPAN": 8,
}


def _value(settings, name, default=None):
    fallback = _DEFAULTS.get(name, default)
    if settings is None:
        return fallback
    if isinstance(settings, dict):
        if name in settings:
            return settings[name]
        road = settings.get("ROAD_PARAMS")
        if isinstance(road, dict) and name in road:
            return road[name]
        return fallback
    value = getattr(settings, name, None)
    if value is not None:
        return value
    road = getattr(settings, "ROAD_PARAMS", None)
    if isinstance(road, dict) and name in road:
        return road[name]
    return fallback


def _int_value(settings, name, default, minimum=1):
    value = _value(settings, name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(name + " must be an integer")
    if value < minimum:
        raise ValueError(name + " is below its supported minimum")
    return value


def _finite_number(settings, name, default, minimum=0.0):
    value = _value(settings, name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + " must be a finite number")
    value = float(value)
    if value != value or value == float("inf") or value == -float("inf"):
        raise ValueError(name + " must be a finite number")
    if value < minimum:
        raise ValueError(name + " is below its supported minimum")
    return value


def _round_cell(value):
    if value >= 0.0:
        return int(value + 0.5)
    return -int(-value + 0.5)


def _normalize_center(center_x, width):
    half = (width - 1) * 0.5
    if half <= 0.0:
        return 0
    return _round_cell(((center_x - half) / half) * 1000.0)


class _RowView:
    __slots__ = ("records", "active_count")

    def __init__(self, records):
        self.records = records
        self.active_count = 0

    def clear(self):
        self.active_count = 0

    def __len__(self):
        return self.active_count

    def __getitem__(self, index):
        if isinstance(index, slice):
            start, stop, step = index.indices(self.active_count)
            return [self.records[item] for item in range(start, stop, step)]
        if index < 0:
            index += self.active_count
        if index < 0 or index >= self.active_count:
            raise IndexError("selected row index out of range")
        return self.records[index]


class _StateFieldView:
    __slots__ = ("data", "stride", "field", "scale", "length")

    def __init__(self, data, stride, field, length, scale=1.0):
        self.data = data
        self.stride = stride
        self.field = field
        self.length = length
        self.scale = scale

    def __len__(self):
        return self.length

    def __getitem__(self, index):
        if index < 0:
            index += self.length
        if index < 0 or index >= self.length:
            raise IndexError("state view index out of range")
        return self.data[index * self.stride + self.field] / self.scale


@micropython.viper
def _reset_viper(self):
    counts = ptr8(self._counts)
    observed = ptr8(self._best_layer_observed)
    for layer in range(int(self.row_count)):
        counts[layer] = 0
        observed[layer] = 0
    control = ptr32(self._control)
    for index in range(12):
        control[index] = 0
    control[0] = -1


@micropython.viper
def _extract_viper(self) -> int:
    luma = ptr8(self._current_luma)
    road = ptr8(self._current_road)
    edge = ptr8(self._current_edge)
    known = ptr8(self._current_known)
    counts = ptr8(self._counts)
    candidates = ptr32(self._candidate_data)
    width = int(self.width)
    height = int(self.height)
    min_width = int(self.min_width)
    max_width = int(self.max_width)
    max_per_row = int(self.max_per_row)

    for layer in range(int(self.row_count)):
        y = height - 1 - layer
        row = y * width
        x = 0
        candidate_count = 0
        while x < width:
            index = row + x
            if known[index] == 0 or road[index] == 0:
                x += 1
                continue
            start = x
            bright_sum = 0
            while x < width:
                index = row + x
                if known[index] == 0 or road[index] == 0:
                    break
                bright_sum += int(luma[index])
                x += 1
            end = x - 1
            run_width = end - start + 1
            if run_width < min_width or run_width > max_width:
                continue
            left_hit = start - 1
            right_hit = end + 1
            if left_hit < 0 or right_hit >= width:
                continue
            left_index = row + left_hit
            right_index = row + right_hit
            if (known[left_index] == 0 or known[right_index] == 0 or
                    road[left_index] != 0 or road[right_index] != 0 or
                    edge[left_index] == 0 or edge[right_index] == 0):
                continue
            if candidate_count >= max_per_row:
                return -1
            node = layer * max_per_row + candidate_count
            offset = node * 4
            candidates[offset] = start
            candidates[offset + 1] = end
            candidates[offset + 2] = run_width
            bright_mean_milli = bright_sum * 1000 // run_width
            dark_mean_milli = (
                int(luma[left_index]) + int(luma[right_index])) * 500
            candidates[offset + 3] = bright_mean_milli - dark_mean_milli
            candidate_count += 1
        counts[layer] = candidate_count
    return 0


@micropython.viper
def _connect_viper(self) -> int:
    counts = ptr8(self._counts)
    candidates = ptr32(self._candidate_data)
    states = ptr32(self._state_data)
    width = int(self.width)
    height = int(self.height)
    layers = int(self.row_count)
    max_per_row = int(self.max_per_row)
    max_gap = int(self.max_gap)
    max_width_change = int(self.max_width_change)
    max_slope_milli = int(self.max_slope_milli)
    link_tolerance_milli = int(self.link_tolerance_milli)
    reference_y_milli = int(self.reference_y_milli)
    state_capacity = int(self.state_capacity)
    sentinel = int(-2147483647)

    for layer in range(layers):
        current_y = height - 1 - layer
        for slot in range(counts[layer]):
            node = layer * max_per_row + slot
            cbase = node * 4
            center_twice = int(candidates[cbase]) + int(candidates[cbase + 1])
            run_width = int(candidates[cbase + 2])
            contrast_milli = int(candidates[cbase + 3])
            if contrast_milli < 0:
                contrast_milli = 0
            elif contrast_milli > 80000:
                contrast_milli = 80000
            local_reward = 10000 + (contrast_milli * 25 + 500) // 1000

            first = node * 2
            first_base = first * 10
            second_base = (first + 1) * 10
            states[first_base] = sentinel
            states[first_base + 1] = -1
            states[first_base + 8] = -1
            states[first_base + 9] = 0
            states[second_base] = sentinel
            states[second_base + 1] = -1
            states[second_base + 8] = -1
            states[second_base + 9] = 0

            distance_limit = max_gap + 1
            if distance_limit > layer:
                distance_limit = layer
            for distance in range(1, distance_limit + 1):
                previous_layer = layer - distance
                previous_count = counts[previous_layer]
                if previous_count == 0:
                    continue
                old_y = height - 1 - previous_layer
                for previous_slot in range(previous_count):
                    previous_node = previous_layer * max_per_row + previous_slot
                    previous_center_twice = (
                        int(candidates[previous_node * 4]) +
                        int(candidates[previous_node * 4 + 1]))
                    previous_width = int(candidates[previous_node * 4 + 2])
                    width_delta = run_width - previous_width
                    if width_delta < 0:
                        width_delta = -width_delta
                    if width_delta > max_width_change * distance:
                        continue
                    center_step_twice = center_twice - previous_center_twice
                    if center_step_twice < 0:
                        center_step_twice = -center_step_twice
                    actual_step_milli = center_step_twice * 500
                    if actual_step_milli > (
                            max_slope_milli * distance +
                            link_tolerance_milli):
                        continue

                    for previous_slot_state in range(2):
                        previous_state = previous_node * 2 + previous_slot_state
                        previous_base = previous_state * 10
                        previous_score = int(states[previous_base])
                        if previous_score <= sentinel // 2:
                            continue
                        parent_state = int(states[previous_base + 1])
                        if parent_state < -1 or parent_state >= state_capacity:
                            continue

                        predicted_twice = previous_center_twice
                        if parent_state >= 0:
                            parent_node = parent_state // 2
                            prior_layer = parent_node // max_per_row
                            base_dy = previous_layer - prior_layer
                            if base_dy <= 0 or prior_layer >= layers:
                                continue
                            prior_center_twice = (
                                int(candidates[parent_node * 4]) +
                                int(candidates[parent_node * 4 + 1]))
                            prediction_delta = (
                                (previous_center_twice - prior_center_twice) *
                                distance)
                            if prediction_delta >= 0:
                                predicted_twice += prediction_delta // base_dy
                            else:
                                predicted_twice -= (-prediction_delta) // base_dy

                        center_error_twice = center_twice - predicted_twice
                        if center_error_twice < 0:
                            center_error_twice = -center_error_twice
                        center_error_milli = center_error_twice * 500
                        if center_error_milli > (
                                max_slope_milli * distance +
                                link_tolerance_milli):
                            continue

                        center_penalty = (
                            center_error_milli * 550 + 500) // 1000
                        width_penalty = (width_delta * 1925 + 5) // 10
                        gap_penalty = (distance - 1) * 2000
                        score = (previous_score + local_reward +
                                 distance * 550 - center_penalty -
                                 width_penalty - gap_penalty)

                        reference_delta = current_y * 1000 - reference_y_milli
                        if reference_delta < 0:
                            reference_delta = -reference_delta
                        old_reference_delta = int(states[previous_base + 5])
                        if reference_delta < old_reference_delta:
                            reference_node = node
                        else:
                            reference_node = int(states[previous_base + 4])
                            reference_delta = old_reference_delta
                        step_slope_milli = (
                            actual_step_milli + distance // 2) // distance
                        slope_sum = int(states[previous_base + 6]) + step_slope_milli
                        width_sum = int(states[previous_base + 7]) + width_delta
                        path_count = int(states[previous_base + 2]) + 1
                        bottom_y = int(states[previous_base + 3])
                        first_score = int(states[first_base])
                        if score > first_score:
                            for field in range(10):
                                states[second_base + field] = states[first_base + field]
                            states[first_base] = score
                            states[first_base + 1] = previous_state
                            states[first_base + 2] = path_count
                            states[first_base + 3] = bottom_y
                            states[first_base + 4] = reference_node
                            states[first_base + 5] = reference_delta
                            states[first_base + 6] = slope_sum
                            states[first_base + 7] = width_sum
                            states[first_base + 8] = -1
                            states[first_base + 9] = 0
                        elif (previous_state != int(states[first_base + 1]) and
                              score > int(states[second_base])):
                            states[second_base] = score
                            states[second_base + 1] = previous_state
                            states[second_base + 2] = path_count
                            states[second_base + 3] = bottom_y
                            states[second_base + 4] = reference_node
                            states[second_base + 5] = reference_delta
                            states[second_base + 6] = slope_sum
                            states[second_base + 7] = width_sum
                            states[second_base + 8] = -1
                            states[second_base + 9] = 0

            reference_delta = current_y * 1000 - reference_y_milli
            if reference_delta < 0:
                reference_delta = -reference_delta
            if local_reward > int(states[first_base]):
                for field in range(10):
                    states[second_base + field] = states[first_base + field]
                states[first_base] = local_reward
                states[first_base + 1] = -1
                states[first_base + 2] = 1
                states[first_base + 3] = current_y
                states[first_base + 4] = node
                states[first_base + 5] = reference_delta
                states[first_base + 6] = 0
                states[first_base + 7] = 0
                states[first_base + 8] = -1
                states[first_base + 9] = 0
            elif (int(states[first_base + 1]) != -1 and
                  local_reward > int(states[second_base])):
                states[second_base] = local_reward
                states[second_base + 1] = -1
                states[second_base + 2] = 1
                states[second_base + 3] = current_y
                states[second_base + 4] = node
                states[second_base + 5] = reference_delta
                states[second_base + 6] = 0
                states[second_base + 7] = 0
                states[second_base + 8] = -1
                states[second_base + 9] = 0
    return 0


@micropython.viper
def _select_viper(self) -> int:
    counts = ptr8(self._counts)
    candidates = ptr32(self._candidate_data)
    states = ptr32(self._state_data)
    observed = ptr8(self._best_layer_observed)
    best_centers = ptr32(self._best_centers_by_layer)
    trace = ptr32(self._trace)
    control = ptr32(self._control)
    layers = int(self.row_count)
    max_per_row = int(self.max_per_row)
    min_rows = int(self.min_rows)
    min_span = int(self.min_span)
    reference_max_gap_milli = int(self.reference_max_gap_milli)
    ambiguity_margin_milli = int(self.ambiguity_margin_milli)
    ambiguity_separation_milli = int(self.ambiguity_separation_milli)
    state_capacity = int(self.state_capacity)
    height = int(self.height)
    sentinel = int(-2147483647)
    best_state = -1
    best_score = sentinel
    viable_count = 0

    for layer in range(layers):
        current_y = height - 1 - layer
        for slot in range(counts[layer]):
            node = layer * max_per_row + slot
            for state_slot in range(2):
                state_id = node * 2 + state_slot
                base = state_id * 10
                score = int(states[base])
                if score <= sentinel // 2:
                    states[base + 8] = -1
                    continue
                path_count = int(states[base + 2])
                span = int(states[base + 3]) - current_y
                reference_delta = int(states[base + 5])
                if (path_count < min_rows or span < min_span or
                        reference_delta > reference_max_gap_milli):
                    states[base + 8] = -1
                    continue

                divisor = path_count - 1
                mean_slope_milli = int(states[base + 6]) // divisor
                mean_width = int(states[base + 7]) // divisor
                coverage_milli = path_count * 1000 // (span + 1)
                if coverage_milli > 1000:
                    coverage_milli = 1000
                support = 45 + path_count * 7
                if support > 100:
                    support = 100
                continuity_milli = (
                    100000 - mean_slope_milli * 16 - mean_width * 2000)
                if continuity_milli < 0:
                    continuity_milli = 0
                quality = (
                    support * 550 + coverage_milli * 25 +
                    continuity_milli // 5 + 500) // 1000
                states[base + 8] = quality
                viable_count += 1
                if score > best_score:
                    best_state = state_id
                    best_score = score

    control[0] = best_state
    control[1] = viable_count
    control[2] = 0
    control[3] = 0
    control[4] = 0
    control[5] = -1
    control[6] = 0
    control[7] = 0
    control[8] = 0
    control[9] = 0
    control[10] = 0
    if best_state < 0:
        return 0

    best_base = best_state * 10
    reference_node = int(states[best_base + 4])
    if reference_node < 0 or reference_node >= layers * max_per_row:
        return -1
    path_state = best_state
    path_count = 0
    center_sum_twice = 0
    while path_state >= 0 and path_count < layers:
        if path_state >= state_capacity:
            return -1
        path_base = path_state * 10
        node = path_state // 2
        layer = node // max_per_row
        if layer >= layers:
            return -1
        trace[path_count] = node
        center_twice = (
            int(candidates[node * 4]) + int(candidates[node * 4 + 1]))
        observed[layer] = 1
        best_centers[layer] = center_twice
        center_sum_twice += center_twice
        path_count += 1
        parent_state = int(states[path_base + 1])
        if parent_state < -1 or parent_state >= state_capacity:
            return -1
        path_state = parent_state
    if path_state >= 0 or path_count == 0:
        return -1
    ref_layer = reference_node // max_per_row
    if ref_layer >= layers or observed[ref_layer] == 0:
        return -1

    control[3] = path_count
    control[4] = center_sum_twice
    control[5] = reference_node
    control[6] = int(states[best_base + 8])
    best_node = best_state // 2
    best_layer = best_node // max_per_row
    control[7] = int(states[best_base + 3]) - (height - 1 - best_layer)
    control[8] = best_score
    control[9] = int(states[best_base + 2])
    control[10] = int(states[best_base + 5])

    # Propagate separation from the selected route once, in increasing layer
    # order. Every parent is in an earlier layer, so field 9 can hold an exact
    # ancestor summary and the competitor pass avoids repeated parent walks.
    # This still evaluates every viable near-score state, including routes
    # that diverged and later merged onto the same endpoint.
    for layer in range(layers):
        for slot in range(counts[layer]):
            node = layer * max_per_row + slot
            center_twice = (
                int(candidates[node * 4]) +
                int(candidates[node * 4 + 1]))
            for state_slot in range(2):
                state_id = node * 2 + state_slot
                base = state_id * 10
                if int(states[base]) <= sentinel // 2:
                    states[base + 9] = 0
                    continue
                parent_state = int(states[base + 1])
                if parent_state < -1 or parent_state >= state_capacity:
                    return -1
                separated = 0
                if parent_state >= 0:
                    parent_layer = (parent_state // 2) // max_per_row
                    if parent_layer >= layer:
                        return -1
                    parent_separated = int(
                        states[parent_state * 10 + 9])
                    if parent_separated < 0 or parent_separated > 1:
                        return -1
                    separated = parent_separated
                if observed[layer] != 0:
                    separation_twice = (
                        center_twice - int(best_centers[layer]))
                    if separation_twice < 0:
                        separation_twice = -separation_twice
                    if separation_twice * 500 >= ambiguity_separation_milli:
                        separated = 1
                states[base + 9] = separated

    # Walk every near-score viable route so a third lane or a merged route
    # cannot hide behind the two score slots at a single endpoint. The
    # divergence summary is equivalent to checking every shared observed row.
    for layer in range(layers):
        for slot in range(counts[layer]):
            node = layer * max_per_row + slot
            for state_slot in range(2):
                competitor = node * 2 + state_slot
                if competitor == best_state:
                    continue
                competitor_base = competitor * 10
                if int(states[competitor_base + 8]) < 0:
                    continue
                score_gap = best_score - int(states[competitor_base])
                if score_gap > ambiguity_margin_milli:
                    continue
                if int(states[competitor_base + 9]) != 0:
                    control[2] = 1
                    return 2
    return 1


class FastPathDetector:
    STATE_STRIDE = 10

    def __init__(self, config):
        self.config = config
        self.width = _int_value(config, "SURFACE_GRID_WIDTH", 80)
        self.height = _int_value(config, "SURFACE_GRID_HEIGHT", 60)
        if self.width != 80 or self.height != 60:
            raise ValueError("FastPathDetector requires an 80x60 grid")
        self.floor_y = _int_value(config, "SURFACE_ROI_FLOOR_Y", 8, 0)
        if self.floor_y >= self.height:
            raise ValueError("SURFACE_ROI_FLOOR_Y must be in 0..59")
        self.row_count = self.height - self.floor_y
        self.min_width = _int_value(config, "ROAD_MIN_WIDTH", 8)
        self.max_width = _int_value(config, "ROAD_MAX_WIDTH", 70)
        if self.min_width < 8 or self.max_width > 70 or self.max_width < self.min_width:
            raise ValueError("ROAD width must stay within 8..70 cells")
        self.max_gap = _int_value(config, "ROAD_MAX_GAP_ROWS", 2, 0)
        if self.max_gap > 2:
            raise ValueError("ROAD_MAX_GAP_ROWS must be in 0..2")
        self.max_slope = _finite_number(config, "ROAD_MAX_CENTER_SLOPE", 2.5)
        self.link_tolerance = _finite_number(config, "ROAD_LINK_TOLERANCE", 2.5)
        if self.max_slope > self.width or self.link_tolerance > self.width:
            raise ValueError("ROAD slope and link tolerance must be at most grid width")
        self.max_slope_milli = int(self.max_slope * 1000.0 + 0.5)
        self.link_tolerance_milli = int(self.link_tolerance * 1000.0 + 0.5)
        self.max_width_change = _int_value(config, "ROAD_MAX_WIDTH_CHANGE", 16)
        if self.max_width_change > 70:
            raise ValueError("ROAD_MAX_WIDTH_CHANGE must be at most 70")
        self.ambiguity_margin = _finite_number(config, "ROAD_AMBIGUITY_MARGIN", 8.0)
        self.ambiguity_separation = _finite_number(
            config, "ROAD_AMBIGUITY_SEPARATION", 8.0)
        if self.ambiguity_margin > 1000.0:
            raise ValueError("ROAD_AMBIGUITY_MARGIN must be in 0..1000")
        if self.ambiguity_separation <= 0.0 or self.ambiguity_separation > self.width - 1:
            raise ValueError("ROAD_AMBIGUITY_SEPARATION must be in 0..79")
        self.ambiguity_margin_milli = int(self.ambiguity_margin * 1000.0 + 0.5)
        self.ambiguity_separation_milli = int(
            self.ambiguity_separation * 1000.0 + 0.5)
        reference_y = _value(config, "ROAD_REFERENCE_Y", 40)
        if isinstance(reference_y, bool) or not isinstance(reference_y, (int, float)):
            raise ValueError("ROAD_REFERENCE_Y must be finite and in the ROI")
        self.reference_y = float(reference_y)
        if (self.reference_y != self.reference_y or
                self.reference_y == float("inf") or
                self.reference_y == -float("inf") or
                self.reference_y < self.floor_y or
                self.reference_y >= self.height):
            raise ValueError("ROAD_REFERENCE_Y must be finite and in the ROI")
        self.reference_y_milli = int(self.reference_y * 1000.0 + 0.5)
        self.min_quality = _int_value(config, "ROAD_QUALITY_MIN", 60, 0)
        if self.min_quality > 100:
            raise ValueError("ROAD_QUALITY_MIN must be in 0..100")
        self.max_per_row = _int_value(
            config, "FAST_PATH_MAX_CANDIDATES_PER_ROW", 8)
        if self.max_per_row > 8:
            raise ValueError("FAST_PATH_MAX_CANDIDATES_PER_ROW must be in 1..8")
        self.min_rows = _int_value(config, "FAST_PATH_MIN_OBSERVED_ROWS", 4, 4)
        if self.min_rows > self.row_count:
            raise ValueError("FAST_PATH_MIN_OBSERVED_ROWS exceeds ROI height")
        self.min_span = _int_value(config, "FAST_PATH_MIN_VERTICAL_SPAN", 8, 8)
        if self.min_span >= self.row_count:
            raise ValueError("FAST_PATH_MIN_VERTICAL_SPAN exceeds ROI span")
        self.reference_max_gap = _finite_number(
            config, "FAST_PATH_REFERENCE_MAX_GAP", float(self.row_count - 1))
        if self.reference_max_gap > self.row_count - 1:
            raise ValueError("FAST_PATH_REFERENCE_MAX_GAP exceeds ROI height")
        self.reference_max_gap_milli = int(
            self.reference_max_gap * 1000.0 + 0.5)

        # Fixed geometry and parameter caps keep every score/intermediate in int32.
        node_capacity = self.row_count * self.max_per_row
        self.state_capacity = node_capacity * 2
        self._counts = bytearray(self.row_count)
        self._candidate_data = array("i", [0] * (node_capacity * 4))
        self._state_data = array(
            "i", [0] * (self.state_capacity * self.STATE_STRIDE))
        self._control = array("i", [0] * 12)
        self._trace = array("i", [0] * self.row_count)
        self._best_centers_by_layer = array("i", [0] * self.row_count)
        self._best_layer_observed = bytearray(self.row_count)
        self.selected_row_ints = array("i", [0] * (self.row_count * 4))
        self.selected_row_stride = 4
        self.selected_row_count = 0
        self._current_luma = bytearray(self.width * self.height)
        self._current_road = bytearray(self.width * self.height)
        self._current_edge = bytearray(self.width * self.height)
        self._current_known = bytearray(self.width * self.height)
        self._scores = _StateFieldView(
            self._state_data, self.STATE_STRIDE, 0, self.state_capacity, 1000.0)
        self._parents = _StateFieldView(
            self._state_data, self.STATE_STRIDE, 1, self.state_capacity)
        self._state_quality = _StateFieldView(
            self._state_data, self.STATE_STRIDE, 8, self.state_capacity)
        self._row_records = []
        self._boundary_records = []
        self._selected_rows = _RowView(self._row_records)
        self._left_edges = ()
        self._right_edges = ()
        self._controller_reference = {
            "x": 0.0, "y": 0.0, "kind": "reference_y_observed",
            "target_y": self.reference_y, "shift_cells": 0.0}
        for _index in range(self.row_count):
            boundary = {"left": [0, 0], "right": [0, 0]}
            self._boundary_records.append(boundary)
            self._row_records.append({
                "y": 0, "left": 0, "right": 0, "center": 0.0,
                "width": 0, "pixels": 0, "observed": True,
                "boundary": boundary, "left_status": "OBSERVED",
                "right_status": "OBSERVED", "provenance": "OBSERVED",
            })
        self._result = {
            "valid": False, "quality": 0, "lateral_error": 0,
            "reason": "NOT_RUN", "scene": "UNKNOWN",
            "reference_y": None, "reference_target_y": self.reference_y,
            "reference_shift_cells": 0.0, "center_x": None,
            "reference_center_x": None,
            "reference_provenance": "NO_COMMON_REFERENCE",
            "reference_lateral_error": 0, "observed_rows": 0,
            "active_row_count": 0, "rows": (),
            "selected_rows": self._selected_rows,
            "selected_row_ints": self.selected_row_ints,
            "selected_row_count": 0, "selected_row_stride": 4,
            "left_edges": self._left_edges, "right_edges": self._right_edges,
            "corners": (), "interpolated": (), "width": self.width,
            "height": self.height, "weight_source": "NONE",
            "candidate_tracks": 0, "controller_reference": None,
            "topology_evidence": None, "boundary_evidence": None,
            "geometry_reason": None, "weights": (), "track_score": 0.0,
            "vertical_span": 0,
        }
        self.topology_guard = FastTopologyGuard(config)
        self._topology_evidence = None

    @micropython.native
    def _reset_work(self):
        self._topology_evidence = None
        _reset_viper(self)
        self._selected_rows.clear()
        self.selected_row_count = 0

    @micropython.native
    def _fail(self, reason, row_diag=(), scene="UNKNOWN"):
        self._selected_rows.clear()
        self.selected_row_count = 0
        result = self._result
        result["boundary_evidence"] = None
        result["geometry_reason"] = None
        result["valid"] = False
        result["quality"] = 0
        result["lateral_error"] = 0
        result["reason"] = reason
        result["scene"] = scene
        result["reference_y"] = None
        result["reference_target_y"] = self.reference_y
        result["reference_shift_cells"] = 0.0
        result["center_x"] = None
        result["reference_center_x"] = None
        result["reference_provenance"] = "NO_COMMON_REFERENCE"
        result["reference_lateral_error"] = 0
        result["observed_rows"] = 0
        result["active_row_count"] = 0
        result["rows"] = row_diag
        result["selected_row_count"] = 0
        result["weight_source"] = "NONE"
        result["candidate_tracks"] = 0
        result["controller_reference"] = None
        result["topology_evidence"] = self._topology_evidence
        result["track_score"] = 0.0
        result["vertical_span"] = 0
        return result

    def _candidate_node(self, layer, slot):
        return layer * self.max_per_row + slot

    def _state_id(self, node, slot):
        return node * 2 + slot

    def _extract_rows(self, mask_info):
        self._current_luma = mask_info["luma"]
        self._current_road = mask_info["road_mask"]
        self._current_edge = mask_info["edge_mask"]
        self._current_known = mask_info["known_region"]
        return _extract_viper(self)

    def _connect_all(self):
        return _connect_viper(self)

    def _select_best(self):
        return _select_viper(self)

    def _rows(self):
        result = []
        for layer in range(self.row_count):
            y = self.height - 1 - layer
            count = self._counts[layer]
            result.append({
                "y": y, "left_status": "OBSERVED" if count else "MISSING",
                "right_status": "OBSERVED" if count else "MISSING",
                "candidate_count": count,
                "provenance": "OBSERVED" if count else "NOT_OBSERVED",
            })
        return result

    def _competing_path_separates(self, state):
        steps = 0
        while state >= 0 and steps < self.row_count:
            node = state // 2
            layer = node // self.max_per_row
            if layer >= self.row_count:
                return False
            if self._best_layer_observed[layer]:
                offset = node * 4
                center_twice = (self._candidate_data[offset] +
                                self._candidate_data[offset + 1])
                if (abs(center_twice -
                        self._best_centers_by_layer[layer]) * 500 >=
                        self.ambiguity_separation_milli):
                    return True
            state = int(self._state_data[state * self.STATE_STRIDE + 1])
            steps += 1
        return False

    @micropython.native
    def _recover_output(self):
        control = self._control
        path_count = int(control[3])
        ref_node = int(control[5])
        ref_layer = ref_node // self.max_per_row
        ref_y = self.height - 1 - ref_layer
        ref_offset = ref_node * 4
        ref_center = (
            self._candidate_data[ref_offset] +
            self._candidate_data[ref_offset + 1]) * 0.5
        center_x = control[4] / float(path_count * 2)
        reference_shift = float(ref_y) - self.reference_y

        for index in range(path_count):
            node = self._trace[path_count - 1 - index]
            layer = node // self.max_per_row
            y = self.height - 1 - layer
            offset = node * 4
            left = int(self._candidate_data[offset])
            right = int(self._candidate_data[offset + 1])
            run_width = int(self._candidate_data[offset + 2])
            center = (left + right) * 0.5
            row = self._row_records[index]
            row["y"] = y
            row["left"] = left
            row["right"] = right
            row["center"] = center
            row["width"] = run_width
            row["pixels"] = run_width
            boundary = self._boundary_records[index]
            boundary["left"][0] = left - 1
            boundary["left"][1] = y
            boundary["right"][0] = right + 1
            boundary["right"][1] = y
            data_offset = index * 4
            self.selected_row_ints[data_offset] = y
            self.selected_row_ints[data_offset + 1] = left
            self.selected_row_ints[data_offset + 2] = right
            self.selected_row_ints[data_offset + 3] = 7

        self._selected_rows.active_count = path_count
        self.selected_row_count = path_count
        self._controller_reference["x"] = ref_center
        self._controller_reference["y"] = float(ref_y)
        self._controller_reference["kind"] = (
            "reference_y_observed" if reference_shift == 0.0 else
            "nearest_observed_y")
        self._controller_reference["target_y"] = self.reference_y
        self._controller_reference["shift_cells"] = reference_shift
        result = self._result
        result["boundary_evidence"] = None
        result["geometry_reason"] = None
        result["valid"] = True
        result["quality"] = int(control[6])
        result["lateral_error"] = _normalize_center(center_x, self.width)
        result["reason"] = "OK"
        result["scene"] = "ROAD_CANDIDATE"
        result["reference_y"] = float(ref_y)
        result["reference_target_y"] = self.reference_y
        result["reference_shift_cells"] = reference_shift
        result["center_x"] = center_x
        result["reference_center_x"] = ref_center
        result["reference_provenance"] = (
            "OBSERVED" if reference_shift == 0.0 else "NEAREST_OBSERVED")
        result["reference_lateral_error"] = _normalize_center(
            ref_center, self.width)
        result["controller_reference"] = self._controller_reference
        result["observed_rows"] = path_count
        result["active_row_count"] = path_count
        result["rows"] = ()
        result["selected_row_count"] = path_count
        result["weight_source"] = "STATIC"
        result["candidate_tracks"] = int(control[1])
        result["track_score"] = int(control[8]) / 1000.0
        result["vertical_span"] = int(control[7])
        result["topology_evidence"] = self._topology_evidence
        return result

    @micropython.native
    def detect(self, mask_info):
        if not isinstance(mask_info, dict):
            self._reset_work()
            return self._fail("MISSING_FRAME_MASK")
        width = mask_info.get("width", self.width)
        height = mask_info.get("height", self.height)
        if (isinstance(width, bool) or not isinstance(width, int) or
                isinstance(height, bool) or not isinstance(height, int)):
            self._reset_work()
            return self._fail("INVALID_FRAME_SIZE")
        if width != self.width or height != self.height:
            self._reset_work()
            return self._fail("FRAME_SIZE_MISMATCH")
        total = width * height
        luma = mask_info.get("luma")
        road = mask_info.get("road_mask")
        edge = mask_info.get("edge_mask")
        known = mask_info.get("known_region")
        if (not isinstance(luma, bytearray) or
                not isinstance(road, bytearray) or
                not isinstance(edge, bytearray) or
                not isinstance(known, bytearray)):
            self._reset_work()
            return self._fail("MISSING_FRAME_MASK")
        if (len(luma) != total or len(road) != total or
                len(edge) != total or len(known) != total):
            self._reset_work()
            return self._fail("FRAME_SIZE_MISMATCH")
        if mask_info.get("valid", True) is False:
            self._reset_work()
            return self._fail(mask_info.get("reason") or "MASK_INVALID")
        mask_floor = mask_info.get("floor", self.floor_y)
        if (isinstance(mask_floor, bool) or not isinstance(mask_floor, int) or
                mask_floor != self.floor_y):
            self._reset_work()
            return self._fail("ROI_FLOOR_MISMATCH")

        self._reset_work()
        topology = self.topology_guard.check(mask_info)
        self._topology_evidence = topology
        if topology.get("ambiguous"):
            return self._fail(topology.get("reason") or "AMBIGUOUS_TOPOLOGY",
                              (), "AMBIGUOUS_CANDIDATE")
        if topology.get("reason") in (
                "INVALID_MASK_INFO", "KERNEL_BOUNDS"):
            return self._fail(topology.get("reason"))

        if self._extract_rows(mask_info) < 0:
            rows = self._rows() if mask_info.get("debug") else ()
            return self._fail("ROW_CANDIDATE_OVERFLOW", rows,
                              "AMBIGUOUS_CANDIDATE")
        self._connect_all()
        select_status = self._select_best()
        if select_status == 2:
            rows = self._rows() if mask_info.get("debug") else ()
            return self._fail("AMBIGUOUS_ROAD", rows, "AMBIGUOUS_CANDIDATE")
        if select_status < 0:
            return self._fail("CORRUPT_PATH_STATE")
        if select_status == 0:
            reason = ("INSUFFICIENT_FRESH_ROWS"
                      if any(self._counts) else "NO_PAIRED_EDGES")
            rows = self._rows() if mask_info.get("debug") else ()
            return self._fail(reason, rows)
        # Quality is carried as a diagnostic field in the unchanged V1 packet.
        # Geometry, freshness, boundary evidence, and ambiguity decide validity.
        return self._recover_output()
