"""Bounded Viper single-edge steering backed by accepted bilateral rows.

This module is a fallback for frames where the fast bilateral path has no
usable result. It searches current-frame dark-to-bright or bright-to-dark
transitions, anchors them to per-row boundaries learned only from an accepted
bilateral observation, and derives the missing edge from that row's smoothed
width. All pixel scans and cache updates use fixed-size buffers.
"""

from array import array

try:
    import uctypes as _uctypes
except ImportError:  # Host tests only; OpenMV provides the real module.
    _uctypes = None

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
RING_OBSERVATION_API_VERSION = 1
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

_DIAG_CANDIDATE_EDGES = 0
_DIAG_AMBIGUOUS_ROWS = 1
_DIAG_ANCHOR_REJECTS = 2
_DIAG_CORRIDOR_REJECTS = 3
_DIAG_QUALIFIED_CHAINS = 4
_DIAG_REJECTED_CHAINS = 5
_DIAG_CHAIN_BREAKS = 6
_DIAG_GAP_BREAKS = 7
_DIAG_JUMP_BREAKS = 8
_DIAG_AMBIGUITY_BREAKS = 9
_DIAG_CHAIN_TIES = 10
_DIAG_SELECTED_CHAINS = 11
_DIAG_SWITCH_PENDING = 12
_DIAG_SWITCH_ACCEPTED = 13
_DIAG_SWITCH_REJECTED = 14
_DIAG_IDENTITY_GAP_REJECTS = 15
_DIAG_SUPPORT_ROW_REJECTS = 16
_DIAG_SUPPORT_SPAN_REJECTS = 17

PAIR_MIN_COMMON_ROWS = 5
PAIR_MIN_COMMON_SPAN = 6
PAIR_MAX_CENTER_DELTA2 = 6

_PAIR_REASON_NONE = 0
_PAIR_REASON_SINGLE_LEFT = 1
_PAIR_REASON_SINGLE_RIGHT = 2
_PAIR_REASON_SAME_LANE = 3
_PAIR_REASON_NO_COMMON_ROWS = 4
_PAIR_REASON_TOO_FEW_ROWS = 5
_PAIR_REASON_SHORT_SPAN = 6
_PAIR_REASON_CENTER_DELTA = 7
_PAIR_REASON_BOUNDARY_ORDER = 8

_PAIR_REASON_NAMES = (
    "NONE", "SINGLE_LEFT", "SINGLE_RIGHT", "SAME_LANE",
    "NO_COMMON_ROWS", "TOO_FEW_ROWS", "SHORT_SPAN",
    "CENTER_DELTA", "BOUNDARY_ORDER",
)

_PAIR_SELECT_NONE = 0
_PAIR_SELECT_PREVIOUS_LEFT = 1
_PAIR_SELECT_PREVIOUS_RIGHT = 2
_PAIR_SELECT_SUPPORT_LEFT = 3
_PAIR_SELECT_SUPPORT_RIGHT = 4
_PAIR_SELECT_BOTTOM_LEFT = 5
_PAIR_SELECT_BOTTOM_RIGHT = 6
_PAIR_SELECT_TIE_LEFT = 7

_PAIR_SELECT_NAMES = (
    "NONE", "PREVIOUS_LEFT", "PREVIOUS_RIGHT", "SUPPORT_LEFT",
    "SUPPORT_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT", "TIE_LEFT",
)

_PAIR_DIAG_REASON = 0
_PAIR_DIAG_COMMON_ROWS = 1
_PAIR_DIAG_SPAN = 2
_PAIR_DIAG_MAX_DELTA2 = 3
_PAIR_DIAG_SELECTED_SIDE = 4
_PAIR_DIAG_SELECTION = 5
_PAIR_DIAG_LEFT_SUPPORT = 6
_PAIR_DIAG_RIGHT_SUPPORT = 7
_PAIR_DIAG_CANDIDATE_FLAGS = 8
_PAIR_DIAG_RESULT_REASON = 9
_PAIR_DIAG_PENDING_SIDE = 10
_PAIR_DIAG_PENDING_COUNT = 11
_PAIR_DIAG_ANCHOR_REJECTS = 12
_PAIR_DIAG_CORRIDOR_REJECTS = 13
_PAIR_DIAG_JUMP_BREAKS = 14
_PAIR_DIAG_GAP_BREAKS = 15
_PAIR_DIAG_SIZE = 16

_CANDIDATE_LEFT_VALID = 1
_CANDIDATE_RIGHT_VALID = 2
_CANDIDATE_PAIR_CHECKED = 4
_CANDIDATE_SELECTED = 8
_CANDIDATE_PUBLISHED = 16

_RESULT_REASON_NAMES = (
    "UNKNOWN", "NOT_RUN", "NO_FRESH_KEEP", "NO_WIDTH_CACHE",
    "INVALID_MASK_INFO", "FRAME_SIZE_MISMATCH",
    "TOPOLOGY_BRANCH_UNRESOLVED", "TOPOLOGY_ISLAND_UNRESOLVED",
    "TOPOLOGY_AMBIGUITY_UNRESOLVED", "TOPOLOGY_COMPONENT_BOUNDS",
    "AMBIGUOUS_BOTH_SIDES", "SINGLE_EDGE_SIDE_SWITCH_PENDING",
    "SINGLE_EDGE_SIDE_SWITCH_NO_OVERLAP",
    "SINGLE_EDGE_SIDE_SWITCH_CENTER_MISMATCH",
    "AMBIGUOUS_SINGLE_EDGE_CHAIN", "SINGLE_EDGE_CONTINUITY_OR_SUPPORT",
    "SINGLE_EDGE_SUPPORT_ROWS", "SINGLE_EDGE_SUPPORT_SPAN",
    "NO_ANCHORED_SINGLE_EDGE", "NO_CURRENT_BOUNDARY_OR_WHITE_CLIP",
    "OK_SINGLE_EDGE_LEFT", "OK_SINGLE_EDGE_RIGHT",
)

# Preserve first-match tuple semantics while avoiding a Python-level scan
# on every no-detect frame. Non-string reasons retain the old loop.
_RESULT_REASON_CODE_BY_NAME = {}
for _reason_index in range(1, len(_RESULT_REASON_NAMES)):
    _reason_name = _RESULT_REASON_NAMES[_reason_index]
    if _reason_name not in _RESULT_REASON_CODE_BY_NAME:
        _RESULT_REASON_CODE_BY_NAME[_reason_name] = _reason_index

PAIR_DIAGNOSTIC_REASON_NAMES = _PAIR_REASON_NAMES
PAIR_DIAGNOSTIC_SELECTION_NAMES = _PAIR_SELECT_NAMES
PAIR_DIAGNOSTIC_RECORD_SIZE = _PAIR_DIAG_SIZE
PAIR_DIAGNOSTIC_RESULT_REASON_NAMES = _RESULT_REASON_NAMES

_DIAG_NAMES = (
    "candidate_edges", "ambiguous_rows", "anchor_rejects",
    "corridor_rejects", "qualified_chains", "rejected_chains",
    "chain_breaks", "gap_breaks", "jump_breaks",
    "ambiguity_breaks", "chain_ties", "selected_chains",
    "switch_pending", "switch_accepted", "switch_rejected",
    "identity_gap_rejects", "support_row_rejects",
    "support_span_rejects",
)

_LAST_TRACKER = None


def last_tracker():
    """Return the live tracker for explicit post-run REPL diagnostics."""
    return _LAST_TRACKER


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
    sample0 = hit + direction
    sample1 = sample0 + direction
    sample2 = sample1 + direction
    if (sample0 < 0 or sample0 >= total or
            sample1 < 0 or sample1 >= total or
            sample2 < 0 or sample2 >= total):
        return 0
    if (known[sample0] == 0 or known[sample1] == 0 or
            known[sample2] == 0 or road[sample0] == 0 or
            road[sample1] == 0 or road[sample2] == 0):
        return 0
    value0 = int(luma[sample0])
    value1 = int(luma[sample1])
    value2 = int(luma[sample2])
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
def _clear_accepted_chain_viper(self) -> int:
    """Clear per-row identity flags with aligned words when safely available."""
    height = int(self.height)
    if int(self._accepted_chain_word_clear) != 0:
        accepted_words = ptr32(self._accepted_chain_valid)
        for word in range(height >> 2):
            accepted_words[word] = 0
    else:
        accepted_chain = ptr8(self._accepted_chain_valid)
        for y in range(height):
            accepted_chain[y] = 0
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
    minimum_step_scaled = int(self.minimum_step_scaled)
    minimum_fraction_scaled = int(self.minimum_fraction_scaled)
    total = width * height
    accepted = 0
    new_cache_rows = 0

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

        left_sample0 = left_hit + 1
        left_sample1 = left_sample0 + 1
        left_sample2 = left_sample1 + 1
        right_sample0 = right_hit - 1
        right_sample1 = right_sample0 - 1
        right_sample2 = right_sample1 - 1
        if (left_sample0 < 0 or left_sample0 >= total or
                left_sample1 < 0 or left_sample1 >= total or
                left_sample2 < 0 or left_sample2 >= total or
                right_sample0 < 0 or right_sample0 >= total or
                right_sample1 < 0 or right_sample1 >= total or
                right_sample2 < 0 or right_sample2 >= total):
            continue
        if (known[left_sample0] == 0 or known[left_sample1] == 0 or
                known[left_sample2] == 0 or known[right_sample0] == 0 or
                known[right_sample1] == 0 or known[right_sample2] == 0 or
                road[left_sample0] == 0 or road[left_sample1] == 0 or
                road[left_sample2] == 0 or road[right_sample0] == 0 or
                road[right_sample1] == 0 or road[right_sample2] == 0):
            continue

        left_value0 = int(luma[left_sample0])
        left_value1 = int(luma[left_sample1])
        left_value2 = int(luma[left_sample2])
        if left_value0 > left_value1:
            temp = left_value0
            left_value0 = left_value1
            left_value1 = temp
        if left_value1 > left_value2:
            temp = left_value1
            left_value1 = left_value2
            left_value2 = temp
        if left_value0 > left_value1:
            temp = left_value0
            left_value0 = left_value1
            left_value1 = temp
        left_white2 = left_value1 * 2
        left_step2 = left_white2 - int(luma[left_hit]) * 2
        if (left_step2 * 4 < minimum_step_scaled * 2 or
                left_step2 * 1000000 <
                left_white2 * minimum_fraction_scaled):
            continue

        right_value0 = int(luma[right_sample0])
        right_value1 = int(luma[right_sample1])
        right_value2 = int(luma[right_sample2])
        if right_value0 > right_value1:
            temp = right_value0
            right_value0 = right_value1
            right_value1 = temp
        if right_value1 > right_value2:
            temp = right_value1
            right_value1 = right_value2
            right_value2 = temp
        if right_value0 > right_value1:
            temp = right_value0
            right_value0 = right_value1
            right_value1 = temp
        right_white2 = right_value1 * 2
        right_step2 = right_white2 - int(luma[right_hit]) * 2
        if (right_step2 * 4 < minimum_step_scaled * 2 or
                right_step2 * 1000000 <
                right_white2 * minimum_fraction_scaled):
            continue

        left2 = left * 2 - 1
        right2 = right * 2 + 1
        if cache_valid[y] == 0:
            new_cache_rows += 1
            cache_width[y] = run_width
            cache_left2[y] = left2
            cache_right2[y] = right2
            anchor_left2[y] = left2
            anchor_right2[y] = right2
            cache_valid[y] = 1
        else:
            if (cache_width[y] == run_width and
                    cache_left2[y] == left2 and
                    cache_right2[y] == right2 and
                    anchor_left2[y] == left2 and
                    anchor_right2[y] == right2):
                # All measured values and identity anchors already match, so
                # the zero-delta smoother would leave every cache word intact.
                accepted += 1
                continue
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
    # Six low bits hold the number of newly cached rows (0..60); higher bits
    # preserve the accepted-row count returned by the original kernel.
    return (accepted << 6) | new_cache_rows


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
    diagnostics = ptr32(self._diagnostic_counts)
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
    retain_paired_edges = int(self._ring_observation_mode)
    total = width * height
    no_distance = int(2147483647)
    diag_candidate_edges = int(0)
    diag_ambiguous_rows = int(1)
    diag_anchor_rejects = int(2)
    diag_corridor_rejects = int(3)
    diag_identity_gap_rejects = int(15)

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
                    diagnostics[diag_candidate_edges] += 1
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
                    diagnostics[diag_candidate_edges] += 1
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

                if (cached_route != 0 and opposite_seen != 0 and
                        retain_paired_edges == 0):
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
                    diagnostics[diag_corridor_rejects] += 1
                    continue
                if cached_route != 0:
                    distance = edge2 - anchor_edge2
                    if distance < 0:
                        distance = -distance
                    if distance > max_anchor2:
                        diagnostics[diag_anchor_rejects] += 1
                        continue

                    identity_clear = 1
                    if side == 0 and edge2 > anchor_edge2:
                        check_x = anchor_edge2 // 2 + 1
                        while check_x <= x:
                            check_index = row + check_x
                            if (known[check_index] == 0 or
                                    road[check_index] != 0):
                                identity_clear = 0
                                break
                            check_x += 1
                    elif side == 1 and edge2 < anchor_edge2:
                        check_x = x
                        anchor_x = anchor_edge2 // 2
                        while check_x <= anchor_x:
                            check_index = row + check_x
                            if (known[check_index] == 0 or
                                    road[check_index] != 0):
                                identity_clear = 0
                                break
                            check_x += 1
                    if identity_clear == 0:
                        diagnostics[diag_identity_gap_rejects] += 1
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
                diagnostics[diag_ambiguous_rows] += 1
                continue
            if (cached_route != 0 and second_distance != no_distance and
                    int(abs(second_edge2 - best_edge2)) >=
                    ambiguity_separation2 and
                    second_distance - best_distance <= ambiguity_margin2):
                status[slot] = 2
                diagnostics[diag_ambiguous_rows] += 1
                continue
            status[slot] = 1
            row_edge2[slot] = best_edge2
            row_center2[slot] = best_center2
            row_contrast[slot] = best_contrast
    return 0


class FastSingleEdgeTracker:
    """Keep validated bilateral widths and infer one current visible side."""

    def __init__(self, settings):
        global _LAST_TRACKER
        self.settings = settings
        self.width = int(_setting(settings, "SURFACE_GRID_WIDTH", WIDTH))
        self.height = int(_setting(settings, "SURFACE_GRID_HEIGHT", HEIGHT))
        if self.width != WIDTH or self.height != HEIGHT:
            raise ValueError("FastSingleEdgeTracker requires an 80x60 grid")
        self.floor_y = int(_setting(settings, "SURFACE_ROI_FLOOR_Y", 8))
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
        self.max_missing_rows = int(_setting(
            settings, "SINGLE_EDGE_MAX_MISSING_ROWS", 3))
        self.max_slope_milli = int(float(_setting(
            settings, "ROAD_MAX_CENTER_SLOPE", 2.5)) * 1000.0 + 0.5)
        self.link_tolerance_milli = int(float(_setting(
            settings, "ROAD_LINK_TOLERANCE", 2.5)) * 1000.0 + 0.5)
        self.max_anchor2 = int(float(_setting(
            settings, "SINGLE_EDGE_ANCHOR_RADIUS_CELLS", 16.0)) * 2.0 + 0.5)
        self.ambiguity_margin2 = int(float(_setting(
            settings, "SINGLE_EDGE_AMBIGUITY_MARGIN_CELLS", 2.0)) * 2.0 + 0.5)
        self.ambiguity_separation2 = int(float(_setting(
            settings, "SINGLE_EDGE_AMBIGUITY_SEPARATION_CELLS", 4.0)) *
            2.0 + 0.5)
        self.min_rows = int(_setting(settings, "SINGLE_EDGE_MIN_ROWS", 5))
        self.min_span = int(_setting(settings, "SINGLE_EDGE_MIN_SPAN", 6))
        self.weak_support_rows = int(_setting(
            settings, "SINGLE_EDGE_WEAK_SUPPORT_ROWS", 8))
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
                self.max_missing_rows < 0 or
                self.max_missing_rows > self.height - self.floor_y or
                self.weak_support_rows < self.min_rows or
                self.weak_support_rows > self.height - self.floor_y):
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
        self._ring_observation_mode = 0
        self._side_valid = bytearray(2)
        self._side_ambiguous = bytearray(2)
        self._side_ambiguous_rows = array("i", [0, 0])
        self._side_count = array("i", [0, 0])
        self._side_first = array("i", [0, 0])
        self._side_last = array("i", [0, 0])
        self._side_ref_y = array("i", [0, 0])
        self._side_ref_center2 = array("i", [0, 0])
        self._side_distance_sum = array("i", [0, 0])
        self._side_overlap_count = array("i", [0, 0])
        self._side_chain_count = array("i", [0, 0])
        self._side_reject_rows = array("i", [0, 0])
        self._side_reject_span = array("i", [0, 0])
        self._accepted_chain_valid = bytearray(self.height)
        self._accepted_chain_word_clear = 0
        if (_IS_MICROPYTHON and _uctypes is not None and
                (self.height & 3) == 0):
            try:
                if (_uctypes.addressof(self._accepted_chain_valid) & 3) == 0:
                    self._accepted_chain_word_clear = 1
            except Exception:
                # Keep the original byte-clear path if address inspection is
                # unavailable or raises on a different MicroPython port.
                pass
        self._accepted_center2 = array("i", [0] * self.height)
        self._pending_switch_side = -1
        self._pending_switch_count = 0
        self._pending_switch_frame = -1
        self._frame_serial = 0
        self._frame_open = False
        self._frame_scan_done = False
        self._frame_start_counts = array("i", [0] * len(_DIAG_NAMES))
        self._last_frame_counts = array("i", [0] * len(_DIAG_NAMES))
        self._frozen_frame_counts = array("i", [0] * len(_DIAG_NAMES))
        self._last_frame_serial = 0
        self._last_frame_detected = False
        self._last_frame_side = -1
        self._last_frame_first_y = -1
        self._last_frame_last_y = -1
        self._last_frame_rows = 0
        self._last_frame_reference_y = -1
        self._last_frame_weak_support = False
        self._last_frame_reason = "NOT_RUN"
        self._last_frame_pending_side = -1
        self._last_frame_pending_count = 0
        self._frozen_snapshot_valid = False
        self._frozen_frame_serial = 0
        self._frozen_frame_detected = False
        self._frozen_frame_side = -1
        self._frozen_frame_first_y = -1
        self._frozen_frame_last_y = -1
        self._frozen_frame_rows = 0
        self._frozen_frame_reference_y = -1
        self._frozen_frame_weak_support = False
        self._frozen_frame_reason = "NOT_FROZEN"
        self._frozen_frame_pending_side = -1
        self._frozen_frame_pending_count = 0
        self._switch_overlap_rows = 0
        self._switch_max_center_delta2 = 0
        self._pair_diagnostic = array("i", [0] * _PAIR_DIAG_SIZE)
        self._last_frame_pair_diagnostic = array(
            "i", [0] * _PAIR_DIAG_SIZE)
        self._frozen_frame_pair_diagnostic = array(
            "i", [0] * _PAIR_DIAG_SIZE)
        self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._last_frame_pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._frozen_frame_pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._diagnostic_counts = array("i", [0] * len(_DIAG_NAMES))
        self._selected_row_ints = array("i", [0] * (self.height * ROW_STRIDE))
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
            "weak_support": False,
            "side_switch": False,
            "switch_overlap_rows": 0,
            "switch_max_center_delta_cells": None,
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
        _LAST_TRACKER = self

    @property
    def cache_rows(self):
        return self._cache_row_count

    @property
    def result(self):
        return self._result

    @property
    def pair_diagnostic_record(self):
        """Return the fixed sixteen-integer per-frame diagnostic record."""
        return self._pair_diagnostic

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
        self._pending_switch_side = -1
        self._pending_switch_count = 0
        self._pending_switch_frame = -1
        self._frame_open = False
        self._switch_overlap_rows = 0
        self._switch_max_center_delta2 = 0
        for y in range(self.height):
            self._accepted_chain_valid[y] = 0
        self.clear_count += 1
        self.last_clear_reason = reason
        self._invalidate_result(reason)

    @micropython.native
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
        result["weak_support"] = False
        result["side_switch"] = False
        result["switch_overlap_rows"] = 0
        result["switch_max_center_delta_cells"] = None
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
            self._pending_switch_side = -1
            self._pending_switch_count = 0
            self._pending_switch_frame = -1
            _clear_accepted_chain_viper(self)
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
        observed_counts = int(_observe_bilateral_viper(self))
        accepted = observed_counts >> 6
        new_cache_rows = observed_counts & 63
        self._cache_row_count += new_cache_rows
        self._last_single_side = -1
        self._pending_switch_side = -1
        self._pending_switch_count = 0
        self._pending_switch_frame = -1
        _clear_accepted_chain_viper(self)
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
        side_switch = (self._last_single_side >= 0 and
                       self._last_single_side != side)
        if side_switch:
            overlap = candidate.get("switch_overlap_rows", 0)
            max_delta = candidate.get("switch_max_center_delta_cells")
            if (candidate.get("side_switch") is not True or
                    not isinstance(overlap, int) or overlap < 1 or
                    not isinstance(max_delta, (int, float)) or
                    max_delta > 12.0):
                return False
        before = self._last_single_side
        for y in range(self.height):
            self._accepted_chain_valid[y] = 0
        for index in range(count):
            offset = index * stride
            y = int(rows[offset])
            center2 = int(rows[offset + 1])
            edge2 = int(rows[offset + 2])
            if y < self.floor_y or y >= self.height:
                continue
            self._accepted_chain_valid[y] = 1
            self._accepted_center2[y] = center2
            if (self._route_mode_id == _ROUTE_CACHED_WIDTH and
                    self._cache_valid[y]):
                anchors = (self._anchor_left2 if side == 0 else
                           self._anchor_right2)
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
        self._pending_switch_side = -1
        self._pending_switch_count = 0
        self._pending_switch_frame = -1
        return before != side

    @micropython.native
    def _switch_overlap(self, side):
        """Measure only current selected rows shared with accepted support."""
        base = side * self.height
        status = self._row_status
        centers = self._row_center2
        first_y = self._side_first[side]
        last_y = self._side_last[side]
        overlap = 0
        maximum_delta2 = 0
        for y in range(first_y, last_y + 1):
            if (status[base + y] != 1 or
                    self._accepted_chain_valid[y] == 0):
                continue
            delta = abs(int(centers[base + y]) -
                        int(self._accepted_center2[y]))
            if delta > maximum_delta2:
                maximum_delta2 = delta
            overlap += 1
        self._switch_overlap_rows = overlap
        self._switch_max_center_delta2 = maximum_delta2
        return overlap

    @micropython.native
    def _allow_side_switch(self, side):
        """Require two fresh candidates and exact-row agreement to history."""
        if self._last_single_side < 0 or self._last_single_side == side:
            self._pending_switch_side = -1
            self._pending_switch_count = 0
            self._pending_switch_frame = -1
            self._switch_overlap_rows = 0
            self._switch_max_center_delta2 = 0
            return True
        if (self._pending_switch_side != side or
                self._pending_switch_frame != self._frame_serial - 1):
            self._pending_switch_side = side
            self._pending_switch_count = 1
        elif self._pending_switch_count < 2:
            self._pending_switch_count += 1
        self._pending_switch_frame = self._frame_serial
        if self._pending_switch_count < 2:
            self._diagnostic_counts[_DIAG_SWITCH_PENDING] += 1
            return False
        overlap = self._switch_overlap(side)
        if overlap == 0 or self._switch_max_center_delta2 > 24:
            self._diagnostic_counts[_DIAG_SWITCH_REJECTED] += 1
            return False
        return True

    def diagnostic_counts(self):
        """Return a low-rate snapshot for explicit REPL diagnostics."""
        return tuple((name, int(self._diagnostic_counts[index]))
                     for index, name in enumerate(_DIAG_NAMES))

    def dump_extended_diagnostics(self, build_id):
        """Print totals and latest/frozen frame snapshots on explicit demand."""
        print("SINGLE_EDGE_EXT_DIAG_BEGIN build=%s format=2" % build_id)
        print("PAIR_REASON_CODES_BEGIN version=1")
        for code, name in enumerate(_PAIR_REASON_NAMES):
            print("%d,%s" % (code, name))
        print("PAIR_REASON_CODES_END")
        print("PAIR_SELECTION_CODES_BEGIN version=1")
        for code, name in enumerate(_PAIR_SELECT_NAMES):
            print("%d,%s" % (code, name))
        print("PAIR_SELECTION_CODES_END")
        print("CANDIDATE_REASON_CODES_BEGIN version=1")
        for code, name in enumerate(_RESULT_REASON_NAMES):
            print("%d,%s" % (code, name))
        print("CANDIDATE_REASON_CODES_END")
        print("TOTAL_COUNTERS_BEGIN")
        for name, count in self.diagnostic_counts():
            print("%s,%d" % (name, count))
        self._print_frame_snapshot(
            "LAST", self._last_frame_serial, self._last_frame_detected,
            self._last_frame_side, self._last_frame_first_y,
            self._last_frame_last_y, self._last_frame_rows,
            self._last_frame_reference_y, self._last_frame_weak_support,
            self._last_frame_reason, self._last_frame_pending_side,
            self._last_frame_pending_count, self._last_frame_counts,
            self._last_frame_pair_diagnostic)
        if self._frozen_snapshot_valid:
            self._print_frame_snapshot(
                "FROZEN", self._frozen_frame_serial,
                self._frozen_frame_detected, self._frozen_frame_side,
                self._frozen_frame_first_y, self._frozen_frame_last_y,
                self._frozen_frame_rows, self._frozen_frame_reference_y,
                self._frozen_frame_weak_support, self._frozen_frame_reason,
                self._frozen_frame_pending_side,
                self._frozen_frame_pending_count,
                self._frozen_frame_counts,
                self._frozen_frame_pair_diagnostic)
        else:
            print("FROZEN_SNAPSHOT unavailable=1")
        print("SINGLE_EDGE_EXT_DIAG_END")
        return True

    def _print_frame_snapshot(self, label, frame_serial, detected, side,
                              first_y, last_y, rows, reference_y, weak,
                              reason, pending_side, pending_count, counts,
                              pair_diagnostic):
        if side == 0:
            side_name = "LEFT"
        elif side == 1:
            side_name = "RIGHT"
        else:
            side_name = "NONE"
        if pending_side == 0:
            pending_name = "LEFT"
        elif pending_side == 1:
            pending_name = "RIGHT"
        else:
            pending_name = "NONE"
        print("%s_FRAME frame=%d detected=%d side=%s first_y=%d last_y=%d "
              "rows=%d reference_y=%d weak_support=%d reason=%s "
              "pending_side=%s pending_count=%d" % (
                  label, frame_serial, 1 if detected else 0, side_name,
                  first_y, last_y, rows, reference_y,
                  1 if weak else 0, reason, pending_name, pending_count))
        print("%s_FRAME_COUNTERS_BEGIN" % label)
        for index, name in enumerate(_DIAG_NAMES):
            print("%s,%d" % (name, int(counts[index])))
        print("%s_FRAME_COUNTERS_END" % label)
        pair_reason = int(pair_diagnostic[_PAIR_DIAG_REASON])
        pair_selection = int(pair_diagnostic[_PAIR_DIAG_SELECTION])
        if pair_reason < 0 or pair_reason >= len(_PAIR_REASON_NAMES):
            pair_reason = _PAIR_REASON_NONE
        if pair_selection < 0 or pair_selection >= len(_PAIR_SELECT_NAMES):
            pair_selection = _PAIR_SELECT_NONE
        selected_side = int(pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE])
        if selected_side == 0:
            selected_name = "LEFT"
        elif selected_side == 1:
            selected_name = "RIGHT"
        else:
            selected_name = "NONE"
        print("%s_PAIR_FRAME reason_code=%d reason=%s common_rows=%d "
              "common_span=%d max_center_delta2=%d selected=%s "
              "selection_code=%d selection=%s left_support=%d "
              "right_support=%d" % (
                  label, pair_reason, _PAIR_REASON_NAMES[pair_reason],
                  int(pair_diagnostic[_PAIR_DIAG_COMMON_ROWS]),
                  int(pair_diagnostic[_PAIR_DIAG_SPAN]),
                  int(pair_diagnostic[_PAIR_DIAG_MAX_DELTA2]),
                  selected_name, pair_selection,
                  _PAIR_SELECT_NAMES[pair_selection],
                  int(pair_diagnostic[_PAIR_DIAG_LEFT_SUPPORT]),
                  int(pair_diagnostic[_PAIR_DIAG_RIGHT_SUPPORT])))
        result_reason = int(pair_diagnostic[_PAIR_DIAG_RESULT_REASON])
        if result_reason < 0 or result_reason >= len(_RESULT_REASON_NAMES):
            result_reason = 0
        pending_side = int(pair_diagnostic[_PAIR_DIAG_PENDING_SIDE])
        if pending_side == 0:
            pending_name = "LEFT"
        elif pending_side == 1:
            pending_name = "RIGHT"
        else:
            pending_name = "NONE"
        print("%s_CANDIDATE_FRAME flags=%d result_code=%d result=%s "
              "pending_side=%s pending_count=%d anchor_rejects=%d "
              "corridor_rejects=%d jump_breaks=%d gap_breaks=%d" % (
                  label,
                  int(pair_diagnostic[_PAIR_DIAG_CANDIDATE_FLAGS]),
                  result_reason, _RESULT_REASON_NAMES[result_reason],
                  pending_name,
                  int(pair_diagnostic[_PAIR_DIAG_PENDING_COUNT]),
                  int(pair_diagnostic[_PAIR_DIAG_ANCHOR_REJECTS]),
                  int(pair_diagnostic[_PAIR_DIAG_CORRIDOR_REJECTS]),
                  int(pair_diagnostic[_PAIR_DIAG_JUMP_BREAKS]),
                  int(pair_diagnostic[_PAIR_DIAG_GAP_BREAKS])))

    @micropython.native
    def _capture_frame_snapshot(self, detected, fallback_reason=None):
        side = -1
        first_y = -1
        last_y = -1
        rows = 0
        reference_y = -1
        weak = False
        result = self._result
        reason = result.get("reason", fallback_reason or "NOT_RUN")
        if fallback_reason is not None and not detected:
            reason = fallback_reason
        if self._frame_scan_done:
            if result.get("boundary_source") == "LEFT_ONLY":
                side = 0
            elif result.get("boundary_source") == "RIGHT_ONLY":
                side = 1
            elif (self._side_valid[0] and not self._side_ambiguous[0] and
                  not self._side_valid[1]):
                side = 0
            elif (self._side_valid[1] and not self._side_ambiguous[1] and
                  not self._side_valid[0]):
                side = 1
            if side >= 0:
                first_y = int(self._side_first[side])
                last_y = int(self._side_last[side])
                rows = int(self._side_count[side])
                reference_y = int(self._side_ref_y[side])
                weak = bool(
                    rows < self.weak_support_rows or
                    float(reference_y) != self.reference_y)
                if result.get("valid"):
                    weak = bool(result.get("weak_support"))
        self._last_frame_serial = self._frame_serial
        self._last_frame_detected = bool(detected)
        self._last_frame_side = side
        self._last_frame_first_y = first_y
        self._last_frame_last_y = last_y
        self._last_frame_rows = rows
        self._last_frame_reference_y = reference_y
        self._last_frame_weak_support = weak
        self._last_frame_reason = reason
        self._last_frame_pending_side = self._pending_switch_side
        self._last_frame_pending_count = self._pending_switch_count
        for index in range(len(self._diagnostic_counts)):
            delta = (int(self._diagnostic_counts[index]) -
                     int(self._frame_start_counts[index]))
            self._last_frame_counts[index] = max(0, delta)
        candidate_flags = 0
        if self._frame_scan_done:
            if (self._side_valid[0] and not self._side_ambiguous[0]):
                candidate_flags |= _CANDIDATE_LEFT_VALID
            if (self._side_valid[1] and not self._side_ambiguous[1]):
                candidate_flags |= _CANDIDATE_RIGHT_VALID
        if (candidate_flags & (_CANDIDATE_LEFT_VALID |
                               _CANDIDATE_RIGHT_VALID)) == (
                                   _CANDIDATE_LEFT_VALID |
                                   _CANDIDATE_RIGHT_VALID):
            candidate_flags |= _CANDIDATE_PAIR_CHECKED
        selected_side = int(
            self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE])
        if selected_side in (0, 1):
            candidate_flags |= _CANDIDATE_SELECTED
        if result.get("valid"):
            candidate_flags |= _CANDIDATE_PUBLISHED
        if type(reason) is str:
            reason_code = _RESULT_REASON_CODE_BY_NAME.get(reason, 0)
        else:
            reason_code = 0
            for index in range(1, len(_RESULT_REASON_NAMES)):
                if _RESULT_REASON_NAMES[index] == reason:
                    reason_code = index
                    break
        diagnostic = self._pair_diagnostic
        diagnostic[_PAIR_DIAG_CANDIDATE_FLAGS] = candidate_flags
        diagnostic[_PAIR_DIAG_RESULT_REASON] = reason_code
        diagnostic[_PAIR_DIAG_PENDING_SIDE] = self._pending_switch_side
        diagnostic[_PAIR_DIAG_PENDING_COUNT] = self._pending_switch_count
        diagnostic[_PAIR_DIAG_ANCHOR_REJECTS] = \
            self._last_frame_counts[_DIAG_ANCHOR_REJECTS]
        diagnostic[_PAIR_DIAG_CORRIDOR_REJECTS] = \
            self._last_frame_counts[_DIAG_CORRIDOR_REJECTS]
        diagnostic[_PAIR_DIAG_JUMP_BREAKS] = \
            self._last_frame_counts[_DIAG_JUMP_BREAKS]
        diagnostic[_PAIR_DIAG_GAP_BREAKS] = \
            self._last_frame_counts[_DIAG_GAP_BREAKS]
        for index in range(_PAIR_DIAG_SIZE):
            self._last_frame_pair_diagnostic[index] = diagnostic[index]

    def freeze_diagnostic_snapshot(self):
        """Keep the exact frame that first froze the fixed steering trace."""
        self._frozen_snapshot_valid = True
        self._frozen_frame_serial = self._last_frame_serial
        self._frozen_frame_detected = self._last_frame_detected
        self._frozen_frame_side = self._last_frame_side
        self._frozen_frame_first_y = self._last_frame_first_y
        self._frozen_frame_last_y = self._last_frame_last_y
        self._frozen_frame_rows = self._last_frame_rows
        self._frozen_frame_reference_y = self._last_frame_reference_y
        self._frozen_frame_weak_support = self._last_frame_weak_support
        self._frozen_frame_reason = self._last_frame_reason
        self._frozen_frame_pending_side = self._last_frame_pending_side
        self._frozen_frame_pending_count = self._last_frame_pending_count
        for index in range(_PAIR_DIAG_SIZE):
            self._frozen_frame_pair_diagnostic[index] = \
                self._last_frame_pair_diagnostic[index]
        for index in range(len(self._last_frame_counts)):
            self._frozen_frame_counts[index] = self._last_frame_counts[index]
        return True

    def rearm_extended_diagnostics(self):
        """Reset the extended counters without changing tracker state."""
        for index in range(len(self._diagnostic_counts)):
            self._diagnostic_counts[index] = 0
            self._frame_start_counts[index] = 0
            self._last_frame_counts[index] = 0
            self._frozen_frame_counts[index] = 0
        self._last_frame_serial = self._frame_serial
        self._last_frame_detected = False
        self._last_frame_side = -1
        self._last_frame_first_y = -1
        self._last_frame_last_y = -1
        self._last_frame_rows = 0
        self._last_frame_reference_y = -1
        self._last_frame_weak_support = False
        self._last_frame_reason = "REARMED"
        self._last_frame_pending_side = self._pending_switch_side
        self._last_frame_pending_count = self._pending_switch_count
        self._pair_diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_NONE
        self._pair_diagnostic[_PAIR_DIAG_COMMON_ROWS] = 0
        self._pair_diagnostic[_PAIR_DIAG_SPAN] = 0
        self._pair_diagnostic[_PAIR_DIAG_MAX_DELTA2] = 0
        self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._pair_diagnostic[_PAIR_DIAG_SELECTION] = _PAIR_SELECT_NONE
        self._pair_diagnostic[_PAIR_DIAG_LEFT_SUPPORT] = 0
        self._pair_diagnostic[_PAIR_DIAG_RIGHT_SUPPORT] = 0
        self._pair_diagnostic[_PAIR_DIAG_CANDIDATE_FLAGS] = 0
        self._pair_diagnostic[_PAIR_DIAG_RESULT_REASON] = 0
        self._pair_diagnostic[_PAIR_DIAG_PENDING_SIDE] = -1
        self._pair_diagnostic[_PAIR_DIAG_PENDING_COUNT] = 0
        self._pair_diagnostic[_PAIR_DIAG_ANCHOR_REJECTS] = 0
        self._pair_diagnostic[_PAIR_DIAG_CORRIDOR_REJECTS] = 0
        self._pair_diagnostic[_PAIR_DIAG_JUMP_BREAKS] = 0
        self._pair_diagnostic[_PAIR_DIAG_GAP_BREAKS] = 0
        for index in range(_PAIR_DIAG_SIZE):
            self._last_frame_pair_diagnostic[index] = 0
            self._frozen_frame_pair_diagnostic[index] = 0
        self._last_frame_pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._frozen_frame_pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._frozen_snapshot_valid = False
        self._frozen_frame_serial = 0
        self._frozen_frame_detected = False
        self._frozen_frame_side = -1
        self._frozen_frame_first_y = -1
        self._frozen_frame_last_y = -1
        self._frozen_frame_rows = 0
        self._frozen_frame_reference_y = -1
        self._frozen_frame_weak_support = False
        self._frozen_frame_reason = "NOT_FROZEN"
        self._frozen_frame_pending_side = -1
        self._frozen_frame_pending_count = 0
        return True

    @micropython.native
    def _reset_pending_side_switch(self):
        self._pending_switch_side = -1
        self._pending_switch_count = 0
        self._pending_switch_frame = -1

    @micropython.native
    def begin_frame(self):
        """Mark every newly captured frame, including detector skips."""
        if self._frame_open:
            # A captured frame that reached this point without a detect() call
            # interrupted any pending two-frame side change.
            self._reset_pending_side_switch()
        self._frame_serial += 1
        self._frame_open = True
        self._frame_scan_done = False
        self._pair_diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_NONE
        self._pair_diagnostic[_PAIR_DIAG_COMMON_ROWS] = 0
        self._pair_diagnostic[_PAIR_DIAG_SPAN] = 0
        self._pair_diagnostic[_PAIR_DIAG_MAX_DELTA2] = 0
        self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        self._pair_diagnostic[_PAIR_DIAG_SELECTION] = _PAIR_SELECT_NONE
        self._pair_diagnostic[_PAIR_DIAG_LEFT_SUPPORT] = 0
        self._pair_diagnostic[_PAIR_DIAG_RIGHT_SUPPORT] = 0
        self._pair_diagnostic[_PAIR_DIAG_CANDIDATE_FLAGS] = 0
        self._pair_diagnostic[_PAIR_DIAG_RESULT_REASON] = 0
        self._pair_diagnostic[_PAIR_DIAG_PENDING_SIDE] = -1
        self._pair_diagnostic[_PAIR_DIAG_PENDING_COUNT] = 0
        self._pair_diagnostic[_PAIR_DIAG_ANCHOR_REJECTS] = 0
        self._pair_diagnostic[_PAIR_DIAG_CORRIDOR_REJECTS] = 0
        self._pair_diagnostic[_PAIR_DIAG_JUMP_BREAKS] = 0
        self._pair_diagnostic[_PAIR_DIAG_GAP_BREAKS] = 0
        for index in range(len(self._diagnostic_counts)):
            self._frame_start_counts[index] = self._diagnostic_counts[index]
        return self._frame_serial

    @micropython.native
    def skip_frame(self, reason="DETECT_NOT_CALLED"):
        """End a captured frame with no detector call and break switch streaks."""
        if not self._frame_open:
            self.begin_frame()
        self._reset_pending_side_switch()
        self._frame_scan_done = False
        self._invalidate_result(reason)
        self._capture_frame_snapshot(False, reason)
        self._frame_open = False
        return self._result

    @micropython.native
    def _consider_side_chain(self, side, count, first_y, last_y,
                             reference_y, reference_center2, distance_sum,
                             overlap_count):
        """Rank one complete segment without allocating per-frame objects."""
        if count < self.min_rows or first_y < 0:
            self._side_reject_rows[side] += 1
            self._diagnostic_counts[_DIAG_REJECTED_CHAINS] += 1
            self._diagnostic_counts[_DIAG_SUPPORT_ROW_REJECTS] += 1
            return
        if last_y - first_y < self.min_span:
            self._side_reject_span[side] += 1
            self._diagnostic_counts[_DIAG_REJECTED_CHAINS] += 1
            self._diagnostic_counts[_DIAG_SUPPORT_SPAN_REJECTS] += 1
            return

        self._diagnostic_counts[_DIAG_QUALIFIED_CHAINS] += 1
        self._side_chain_count[side] += 1
        if self._side_count[side] == 0:
            better = True
            tied = False
        else:
            better = False
            tied = False
            if last_y > self._side_last[side]:
                better = True
            elif last_y == self._side_last[side]:
                if count > self._side_count[side]:
                    better = True
                elif count == self._side_count[side]:
                    best_span = self._side_last[side] - self._side_first[side]
                    span = last_y - first_y
                    if span > best_span:
                        better = True
                    elif span == best_span:
                        if self._last_single_side == side:
                            best_overlap = self._side_overlap_count[side]
                            if overlap_count > 0 and best_overlap == 0:
                                better = True
                            elif overlap_count == 0 and best_overlap > 0:
                                better = False
                            elif overlap_count > 0 and best_overlap > 0:
                                left = distance_sum * best_overlap
                                right = (self._side_distance_sum[side] *
                                         overlap_count)
                                if left < right:
                                    better = True
                                elif left == right:
                                    tied = True
                            else:
                                tied = True
                        else:
                            tied = True

        if better:
            self._side_count[side] = count
            self._side_first[side] = first_y
            self._side_last[side] = last_y
            self._side_ref_y[side] = reference_y
            self._side_ref_center2[side] = reference_center2
            self._side_distance_sum[side] = distance_sum
            self._side_overlap_count[side] = overlap_count
            self._side_ambiguous[side] = 0
            self._side_valid[side] = 1
        elif tied:
            self._side_ambiguous[side] = 1
            self._diagnostic_counts[_DIAG_CHAIN_TIES] += 1

    @micropython.native
    def _evaluate_side(self, side):
        """Build bottom-up segments and keep the nearest qualified one."""
        height = self.height
        floor = self.floor_y
        base = side * height
        status = self._row_status
        centers = self._row_center2
        self._side_count[side] = 0
        self._side_first[side] = 0
        self._side_last[side] = 0
        self._side_ref_y[side] = 0
        self._side_ref_center2[side] = 0
        self._side_distance_sum[side] = 0
        self._side_overlap_count[side] = 0
        self._side_chain_count[side] = 0
        self._side_reject_rows[side] = 0
        self._side_reject_span[side] = 0
        self._side_ambiguous_rows[side] = 0
        self._side_ambiguous[side] = 0
        self._side_valid[side] = 0

        chain_count = 0
        chain_first = -1
        chain_last = -1
        previous_y = -1
        previous_center2 = 0
        chain_ref_y = -1
        chain_ref_center2 = 0
        chain_ref_delta = 1 << 30
        chain_distance_sum = 0
        chain_overlap_count = 0
        missing_count = 0
        track_history = self._last_single_side == side
        reference_y = self.reference_y
        diagnostics = self._diagnostic_counts

        for y in range(height - 1, floor - 1, -1):
            slot = base + y
            row_status = status[slot]
            if row_status == _AMBIGUOUS:
                self._side_ambiguous_rows[side] += 1
                if chain_count:
                    diagnostics[_DIAG_CHAIN_BREAKS] += 1
                    diagnostics[_DIAG_AMBIGUITY_BREAKS] += 1
                    self._consider_side_chain(
                        side, chain_count, chain_first, chain_last,
                        chain_ref_y, chain_ref_center2, chain_distance_sum,
                        chain_overlap_count)
                    chain_count = 0
                    chain_first = -1
                    chain_last = -1
                    previous_y = -1
                    chain_distance_sum = 0
                    chain_overlap_count = 0
                missing_count = 0
                continue

            if row_status != 1:
                if chain_count:
                    missing_count += 1
                    if missing_count > self.max_missing_rows:
                        diagnostics[_DIAG_CHAIN_BREAKS] += 1
                        diagnostics[_DIAG_GAP_BREAKS] += 1
                        self._consider_side_chain(
                            side, chain_count, chain_first, chain_last,
                            chain_ref_y, chain_ref_center2,
                            chain_distance_sum, chain_overlap_count)
                        chain_count = 0
                        chain_first = -1
                        chain_last = -1
                        previous_y = -1
                        chain_distance_sum = 0
                        chain_overlap_count = 0
                        missing_count = 0
                continue

            center2 = int(centers[slot])
            if chain_count:
                dy = previous_y - y
                jump = abs(center2 - previous_center2) * 500 > (
                    self.max_slope_milli * dy +
                    self.link_tolerance_milli)
                if jump:
                    diagnostics[_DIAG_CHAIN_BREAKS] += 1
                    diagnostics[_DIAG_JUMP_BREAKS] += 1
                    self._consider_side_chain(
                        side, chain_count, chain_first, chain_last,
                        chain_ref_y, chain_ref_center2, chain_distance_sum,
                        chain_overlap_count)
                    chain_count = 0
                    chain_first = -1
                    chain_last = -1
                    chain_distance_sum = 0
                    chain_overlap_count = 0
                    missing_count = 0

            if chain_count == 0:
                chain_first = y
                chain_last = y
                chain_count = 1
                chain_ref_y = y
                chain_ref_center2 = center2
                chain_ref_delta = abs(float(y) - reference_y)
                chain_distance_sum = 0
                chain_overlap_count = 0
            else:
                chain_first = y
                chain_count += 1
                delta = abs(float(y) - reference_y)
                if delta < chain_ref_delta:
                    chain_ref_y = y
                    chain_ref_center2 = center2
                    chain_ref_delta = delta

            if track_history and self._accepted_chain_valid[y]:
                distance = abs(
                    center2 - int(self._accepted_center2[y]))
                chain_distance_sum += distance
                chain_overlap_count += 1
            previous_y = y
            previous_center2 = center2
            missing_count = 0

        if chain_count:
            self._consider_side_chain(
                side, chain_count, chain_first, chain_last, chain_ref_y,
                chain_ref_center2, chain_distance_sum, chain_overlap_count)

        return (self._side_valid[side] != 0 and
                self._side_ambiguous[side] == 0)

    @micropython.native
    def _resolve_same_lane_pair(self):
        """Require measured row agreement before resolving two side chains."""
        diagnostic = self._pair_diagnostic
        diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_NONE
        diagnostic[_PAIR_DIAG_COMMON_ROWS] = 0
        diagnostic[_PAIR_DIAG_SPAN] = 0
        diagnostic[_PAIR_DIAG_MAX_DELTA2] = 0
        diagnostic[_PAIR_DIAG_SELECTED_SIDE] = -1
        diagnostic[_PAIR_DIAG_SELECTION] = _PAIR_SELECT_NONE
        diagnostic[_PAIR_DIAG_LEFT_SUPPORT] = int(self._side_count[0])
        diagnostic[_PAIR_DIAG_RIGHT_SUPPORT] = int(self._side_count[1])

        first_y = int(self._side_first[0])
        if int(self._side_first[1]) > first_y:
            first_y = int(self._side_first[1])
        last_y = int(self._side_last[0])
        if int(self._side_last[1]) < last_y:
            last_y = int(self._side_last[1])
        common_count = 0
        common_first = self.height
        common_last = -1
        maximum_delta2 = 0
        wrong_order = 0
        status = self._row_status
        edges = self._row_edge2
        centers = self._row_center2
        height = self.height

        for y in range(first_y, last_y + 1):
            left_slot = y
            right_slot = height + y
            if status[left_slot] != 1 or status[right_slot] != 1:
                continue
            common_count += 1
            if y < common_first:
                common_first = y
            if y > common_last:
                common_last = y
            if int(edges[left_slot]) >= int(edges[right_slot]):
                wrong_order = 1
            delta2 = abs(int(centers[left_slot]) -
                         int(centers[right_slot]))
            if delta2 > maximum_delta2:
                maximum_delta2 = delta2

        span = common_last - common_first if common_count else 0
        diagnostic[_PAIR_DIAG_COMMON_ROWS] = common_count
        diagnostic[_PAIR_DIAG_SPAN] = span
        diagnostic[_PAIR_DIAG_MAX_DELTA2] = maximum_delta2
        if common_count == 0:
            diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_NO_COMMON_ROWS
            return -1
        if common_count < PAIR_MIN_COMMON_ROWS:
            diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_TOO_FEW_ROWS
            return -1
        if span < PAIR_MIN_COMMON_SPAN:
            diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_SHORT_SPAN
            return -1
        if maximum_delta2 > PAIR_MAX_CENTER_DELTA2:
            diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_CENTER_DELTA
            return -1
        if wrong_order:
            diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_BOUNDARY_ORDER
            return -1

        if self._last_single_side == 0 or self._last_single_side == 1:
            selected = self._last_single_side
            select_reason = (_PAIR_SELECT_PREVIOUS_LEFT if selected == 0 else
                             _PAIR_SELECT_PREVIOUS_RIGHT)
        elif self._side_count[0] > self._side_count[1]:
            selected = 0
            select_reason = _PAIR_SELECT_SUPPORT_LEFT
        elif self._side_count[1] > self._side_count[0]:
            selected = 1
            select_reason = _PAIR_SELECT_SUPPORT_RIGHT
        elif self._side_last[0] > self._side_last[1]:
            selected = 0
            select_reason = _PAIR_SELECT_BOTTOM_LEFT
        elif self._side_last[1] > self._side_last[0]:
            selected = 1
            select_reason = _PAIR_SELECT_BOTTOM_RIGHT
        else:
            selected = 0
            select_reason = _PAIR_SELECT_TIE_LEFT

        diagnostic[_PAIR_DIAG_REASON] = _PAIR_REASON_SAME_LANE
        diagnostic[_PAIR_DIAG_SELECTED_SIDE] = selected
        diagnostic[_PAIR_DIAG_SELECTION] = select_reason
        return selected

    @micropython.native
    def _publish_single(self, side, topology_reason,
                        topology_island_observed=False, side_switch=False):
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
        result["weak_support"] = bool(
            support_count < self.weak_support_rows or
            float(ref_y) != self.reference_y)
        result["side_switch"] = bool(side_switch)
        result["switch_overlap_rows"] = (
            self._switch_overlap_rows if side_switch else 0)
        result["switch_max_center_delta_cells"] = (
            self._switch_max_center_delta2 * 0.5 if side_switch else None)
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
        self._diagnostic_counts[_DIAG_SELECTED_CHAINS] += 1
        if side_switch:
            self._diagnostic_counts[_DIAG_SWITCH_ACCEPTED] += 1
        return result

    @micropython.native
    def _detect_current(self, mask_info, control_fresh,
                        topology_evidence=None):
        """Return a single-edge result only inside a fresh RESET/KEEP session."""
        self._invalidate_result("NOT_RUN")
        if not control_fresh:
            self._reset_pending_side_switch()
            self._invalidate_result("NO_FRESH_KEEP")
            return self._result
        if (self._route_mode_id == _ROUTE_CACHED_WIDTH and
                self._cache_row_count < self.min_rows):
            self._reset_pending_side_switch()
            self._invalidate_result("NO_WIDTH_CACHE")
            return self._result
        if not isinstance(mask_info, dict):
            self._reset_pending_side_switch()
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
            self._reset_pending_side_switch()
            self._invalidate_result("FRAME_SIZE_MISMATCH")
            return self._result
        topology = topology_evidence or {}
        self._result["topology_reason"] = topology.get("reason", "NOT_RUN")
        island = bool(topology.get("island_evidence") or topology.get("island"))
        ambiguous_topology = bool(topology.get("ambiguous"))
        if topology.get("branch_evidence") or topology.get("branch"):
            self._reset_pending_side_switch()
            self._invalidate_result("TOPOLOGY_BRANCH_UNRESOLVED")
            self._result["topology_reason"] = topology.get("reason", "OPEN_BAND")
            return self._result
        if (self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT and island):
            self._reset_pending_side_switch()
            self._invalidate_result("TOPOLOGY_ISLAND_UNRESOLVED")
            self._result["topology_reason"] = topology.get(
                "reason", "ISLAND")
            return self._result
        if (ambiguous_topology and
                (self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT or
                 not island)):
            self._reset_pending_side_switch()
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
            self._reset_pending_side_switch()
            self._invalidate_result("TOPOLOGY_COMPONENT_BOUNDS")
            self._result["topology_reason"] = topology.get(
                "reason", "ISLAND")
            return self._result
        _scan_single_edges_viper(self)
        self._frame_scan_done = True

        left_valid = self._evaluate_side(0)
        right_valid = self._evaluate_side(1)
        if left_valid and right_valid:
            selected_side = self._resolve_same_lane_pair()
        elif left_valid:
            self._pair_diagnostic[_PAIR_DIAG_REASON] = \
                _PAIR_REASON_SINGLE_LEFT
            self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = 0
            self._pair_diagnostic[_PAIR_DIAG_LEFT_SUPPORT] = int(
                self._side_count[0])
            selected_side = 0
        elif right_valid:
            self._pair_diagnostic[_PAIR_DIAG_REASON] = \
                _PAIR_REASON_SINGLE_RIGHT
            self._pair_diagnostic[_PAIR_DIAG_SELECTED_SIDE] = 1
            self._pair_diagnostic[_PAIR_DIAG_RIGHT_SUPPORT] = int(
                self._side_count[1])
            selected_side = 1
        else:
            selected_side = -1

        if left_valid and right_valid and selected_side < 0:
            self._reset_pending_side_switch()
            self._invalidate_result("AMBIGUOUS_BOTH_SIDES")
            self._result["topology_reason"] = topology.get(
                "reason", "NOT_RUN")
            return self._result
        if selected_side >= 0:
            side_switch = (self._last_single_side >= 0 and
                           self._last_single_side != selected_side)
            if not self._allow_side_switch(selected_side):
                if self._pending_switch_count < 2:
                    reason = "SINGLE_EDGE_SIDE_SWITCH_PENDING"
                elif self._switch_overlap_rows == 0:
                    reason = "SINGLE_EDGE_SIDE_SWITCH_NO_OVERLAP"
                else:
                    reason = "SINGLE_EDGE_SIDE_SWITCH_CENTER_MISMATCH"
                self._invalidate_result(reason)
                self._result["topology_reason"] = topology.get(
                    "reason", "NOT_RUN")
                return self._result
            return self._publish_single(
                selected_side, topology.get("reason", "NOT_RUN"), island,
                side_switch)
        self._reset_pending_side_switch()
        if (self._side_ambiguous[0] or self._side_ambiguous[1] or
                self._side_ambiguous_rows[0] or
                self._side_ambiguous_rows[1]):
            self._invalidate_result("AMBIGUOUS_SINGLE_EDGE_CHAIN")
            self._result["topology_reason"] = topology.get(
                "reason", "NOT_RUN")
            return self._result
        if self._side_count[0] or self._side_count[1]:
            self._invalidate_result("SINGLE_EDGE_CONTINUITY_OR_SUPPORT")
        elif self._side_reject_rows[0] or self._side_reject_rows[1]:
            self._invalidate_result("SINGLE_EDGE_SUPPORT_ROWS")
        elif self._side_reject_span[0] or self._side_reject_span[1]:
            self._invalidate_result("SINGLE_EDGE_SUPPORT_SPAN")
        elif self._route_mode_id == _ROUTE_CACHED_WIDTH and \
                self._cache_row_count < self.min_rows:
            self._invalidate_result("NO_WIDTH_CACHE")
        elif self._route_mode_id == _ROUTE_CLIPPED_MIDPOINT:
            self._invalidate_result("NO_CURRENT_BOUNDARY_OR_WHITE_CLIP")
        else:
            self._invalidate_result("NO_ANCHORED_SINGLE_EDGE")
        self._result["topology_reason"] = topology.get("reason", "NOT_RUN")
        return self._result

    def observe_ring_boundaries(self, mask_info, control_fresh,
                                topology_evidence=None):
        """Observe both current edges without the single-missing-side filter.

        This is not a fallback route selection. All transition, contrast,
        identity, ambiguity and known-road checks still run; only exclusion
        of an edge solely because its partner is visible is disabled.
        """
        self._ring_observation_mode = 1
        try:
            return self.detect(mask_info, control_fresh, topology_evidence)
        finally:
            self._ring_observation_mode = 0

    def detect(self, mask_info, control_fresh, topology_evidence=None):
        """Run one detector call and retain a fixed-size per-frame snapshot."""
        if not self._frame_open:
            self.begin_frame()
        self._frame_scan_done = False
        try:
            return self._detect_current(
                mask_info, control_fresh, topology_evidence)
        finally:
            self._capture_frame_snapshot(True)
            self._frame_open = False


__all__ = (
    "COMPILER_BACKEND", "FastSingleEdgeTracker",
    "FLAG_IDENTITY_ANCHORED", "FLAG_INFERRED_LEFT", "FLAG_INFERRED_RIGHT",
    "FLAG_OBSERVED_LEFT", "FLAG_OBSERVED_RIGHT", "FLAG_WIDTH_CACHE",
    "INFERENCE_USED_CONTINUITY", "INFERENCE_USED_CURRENT_EDGE",
    "INFERENCE_USED_WIDTH_CACHE", "RUNTIME_API_VERSION",
)
