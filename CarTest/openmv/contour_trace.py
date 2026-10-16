"""Bounded four-direction contour tracing for an 80x60 binary road mask.

The follower is a bounded Python port of the supplied left/right rules.  The
geometry helpers below are independent utilities; they are not part of the
original follower and make no camera or track-width calibration claim.
"""

from array import array
import math


WHITE = 255
UNKNOWN = 125
RUNTIME_API_VERSION = 1
COMPILER_BACKEND = "python-bounded"

TERM_NOT_RUN = "NOT_RUN"
TERM_EDGE = "EDGE"
TERM_DIRECTION_LIMIT = "DIRECTION_LIMIT"
TERM_CAPACITY = "CAPACITY"
TERM_ITERATION_BUDGET = "ITERATION_BUDGET"
TERM_REPEATED_STATE = "REPEATED_STATE"
TERM_COLLISION = "COLLISION"
TERM_BAD_SEED = "BAD_SEED"
TERM_STATE_TABLE_FULL = "STATE_TABLE_FULL"

SEED_OK = "OK"
SEED_NO_ROAD = "NO_CENTER_ROAD"
SEED_NO_DARK_BOUNDS = "NO_REAL_DARK_BOUNDS"
SEED_AMBIGUOUS = "AMBIGUOUS_CANDIDATES"
SEED_TOO_FAR = "ROAD_TOO_FAR_FROM_CENTER"

_DY = (-1, 0, 1, 0)
_DX = (0, 1, 0, -1)

# MicroPython v1.28 intentionally does not implement array * integer. The host
# interpreter does, which allowed the original fixed-storage allocations to
# pass local tests while failing on the camera. Keep zero-filled allocations
# byte-backed, using one fixed-length array allocation and a temporary zero
# bytearray; write non-zero initial values after allocation.
_ARRAY_ITEM_BYTES = {
    "b": 1,
    "B": 1,
    "h": 2,
    "H": 2,
    "i": 4,
    "I": 4,
    "f": 4,
}


def _array_fill(typecode, value, count):
    if not _valid_int(count) or count < 0:
        raise ValueError("array fill count must be a non-negative integer")
    try:
        item_bytes = _ARRAY_ITEM_BYTES[typecode]
    except KeyError:
        raise ValueError("unsupported array typecode")
    result = array(typecode, bytearray(count * item_bytes))
    if value != 0:
        for index in range(count):
            result[index] = value
    return result


def _info_value(info, name, default=None):
    getter = getattr(info, "get", None)
    if callable(getter):
        return getter(name, default)
    return getattr(info, name, default)


def _valid_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_masks(info, width, height):
    info_width = _info_value(info, "width", width)
    info_height = _info_value(info, "height", height)
    if (not _valid_int(info_width) or not _valid_int(info_height) or
            info_width != width or info_height != height):
        raise ValueError("mask dimensions do not match workspace")
    road = _info_value(info, "road_mask")
    known = _info_value(info, "known_region")
    total = width * height
    if road is None or known is None or len(road) != total or len(known) != total:
        raise ValueError("road_mask and known_region must match workspace size")
    return road, known


def _is_white(road, known, width, height, y, x):
    if x < 0 or x >= width or y < 0 or y >= height:
        return False
    index = y * width + x
    return known[index] != 0 and road[index] == WHITE


def _is_nonwhite(road, known, width, height, y, x):
    # Unknown and clipped samples follow the original 125 != 255 branch, but
    # callers must not treat them as evidence of a real dark boundary.
    return not _is_white(road, known, width, height, y, x)


def _get_image_sum(road, known, width, height, y, x):
    if x < 0 or x >= width or y < 0 or y >= height:
        return UNKNOWN
    index = y * width + x
    if known[index] == 0:
        return UNKNOWN
    return road[index]


def _dark_at(road, known, width, height, y, x):
    if x < 0 or x >= width or y < 0 or y >= height:
        return 0
    index = y * width + x
    return 1 if known[index] != 0 and road[index] == 0 else 0


def _side_direction(direction, side):
    return (direction + (3 if side == 0 else 1)) & 3


def _support_at(road, known, width, height, y, x, direction, side):
    side_dir = _side_direction(direction, side)
    return _dark_at(road, known, width, height,
                    y + _DY[side_dir], x + _DX[side_dir])


def _is_frame_edge(y, x, width, height):
    return y <= 0 or x <= 0 or y >= height - 1 or x >= width - 1


class ContourTraceWorkspace:
    """Reusable fixed-storage tracer. Coordinates are flattened ``(y, x)``."""

    def __init__(self, width=80, height=60, max_points=900,
                 iteration_budget=None, state_table_size=None,
                 record_events=False):
        if (not _valid_int(width) or not _valid_int(height) or
                width < 3 or height < 3 or width > 65535 or height > 65535):
            raise ValueError("width and height must be integers in 3..65535")
        if not _valid_int(max_points) or max_points < 1:
            raise ValueError("max_points must be a positive integer")
        if width * height > 0xFFFFFFFE:
            raise ValueError("workspace area must fit an unsigned 32-bit index")
        if iteration_budget is None:
            iteration_budget = max_points * 2
        if not _valid_int(iteration_budget) or iteration_budget < 1:
            raise ValueError("iteration_budget must be a positive integer")
        if state_table_size is None:
            state_table_size = 128
            wanted = iteration_budget * 2
            while state_table_size < wanted:
                state_table_size <<= 1
        if (not _valid_int(state_table_size) or state_table_size < 16 or
                (state_table_size & (state_table_size - 1)) != 0):
            raise ValueError("state_table_size must be a power of two >= 16")

        self.width = width
        self.height = height
        self.max_points = max_points
        self.iteration_budget = iteration_budget
        self.record_events = bool(record_events)
        self._state_mask = state_table_size - 1
        self._state_position = _array_fill("I", 0, state_table_size)
        self._state_code = bytearray(state_table_size)

        self.left_points = _array_fill("H", 0, max_points * 2)
        self.right_points = _array_fill("H", 0, max_points * 2)
        self.left_dark_support = bytearray(max_points)
        self.right_dark_support = bytearray(max_points)
        self.left_directions = bytearray(max_points)
        self.right_directions = bytearray(max_points)
        self.left_errors = _array_fill("b", 0, max_points)
        self.right_errors = _array_fill("b", 0, max_points)
        self.left_lost_history = _array_fill("H", 0, max_points)
        self.right_lost_history = _array_fill("H", 0, max_points)
        self.left_row_support = _array_fill("H", 0, height)
        self.right_row_support = _array_fill("H", 0, height)
        event_capacity = iteration_budget if self.record_events else 0
        self.event_side = bytearray(event_capacity)
        self.event_points = _array_fill("H", 0, event_capacity * 2)
        self.event_directions = bytearray(event_capacity)
        self.event_errors = _array_fill("b", 0, event_capacity)
        self.event_lost = _array_fill("H", 0, event_capacity)
        self.event_actions = bytearray(event_capacity)
        self.event_count = 0
        self.left_count = 0
        self.right_count = 0
        self.left_lost = 0
        self.right_lost = 0
        self.left_termination = TERM_NOT_RUN
        self.right_termination = TERM_NOT_RUN
        self.seed_status = SEED_NO_ROAD
        self.left_seed_y = -1
        self.left_seed_x = -1
        self.right_seed_y = -1
        self.right_seed_x = -1
        self.iterations = 0
        self.left_final_error = 0
        self.right_final_error = 0

    def _clear(self):
        self.left_count = 0
        self.right_count = 0
        self.left_lost = 0
        self.right_lost = 0
        self.left_termination = TERM_NOT_RUN
        self.right_termination = TERM_NOT_RUN
        self.iterations = 0
        self.event_count = 0
        self.left_final_error = 0
        self.right_final_error = 0
        for index in range(self.height):
            self.left_row_support[index] = 0
            self.right_row_support[index] = 0
        for index in range(len(self._state_position)):
            self._state_position[index] = 0
            self._state_code[index] = 0

    def _state_seen(self, y, x, direction, error, side):
        flat = y * self.width + x
        code = ((error + 10) * 4 + direction) * 2 + side
        stored_code = code + 1
        stored_position = flat + 1
        slot = (flat * 131 + code * 17) & self._state_mask
        for _probe in range(len(self._state_position)):
            seen_position = self._state_position[slot]
            if seen_position == 0:
                self._state_position[slot] = stored_position
                self._state_code[slot] = stored_code
                return 0
            if (seen_position == stored_position and
                    self._state_code[slot] == stored_code):
                return 1
            slot = (slot + 1) & self._state_mask
        return -1

    def _append(self, side, y, x, direction, error, road, known):
        if side == 0:
            count = self.left_count
            points = self.left_points
            support = self.left_dark_support
            directions = self.left_directions
            errors = self.left_errors
            lost_history = self.left_lost_history
            rows = self.left_row_support
        else:
            count = self.right_count
            points = self.right_points
            support = self.right_dark_support
            directions = self.right_directions
            errors = self.right_errors
            lost_history = self.right_lost_history
            rows = self.right_row_support
        offset = count * 2
        points[offset] = y
        points[offset + 1] = x
        evidence = _support_at(road, known, self.width, self.height,
                               y, x, direction, side)
        support[count] = evidence
        rows[y] += evidence
        if side == 0:
            lost_history[count] = self.left_lost
            self.left_directions[count] = direction
            errors[count] = error
            self.left_count += 1
        else:
            lost_history[count] = self.right_lost
            self.right_directions[count] = direction
            errors[count] = error
            self.right_count += 1

    def _record_event(self, side, y, x, direction, error, lost, action):
        if not self.record_events:
            return
        index = self.event_count
        self.event_side[index] = side
        self.event_points[index * 2] = y
        self.event_points[index * 2 + 1] = x
        self.event_directions[index] = direction
        self.event_errors[index] = error
        self.event_lost[index] = lost
        self.event_actions[index] = action
        self.event_count += 1

    def find_seeds(self, mask_info, center_x=39.5, row=None,
                   search_rows=8, max_center_distance=None):
        """Find the unique lower-image white run with two known dark borders.

        The default search starts at the bottom row and moves upward.
        A run touching an unknown or clipped cell is not a valid road seed.
        The method stores seed coordinates on the workspace and returns bool.
        """
        if (not isinstance(center_x, (int, float)) or
                isinstance(center_x, bool) or center_x != center_x or
                center_x == float("inf") or center_x == -float("inf")):
            raise ValueError("center_x must be finite")
        if not _valid_int(search_rows) or search_rows < 1:
            raise ValueError("search_rows must be a positive integer")
        if row is None:
            row = self.height - 1
        if not _valid_int(row) or row < 0 or row >= self.height:
            raise ValueError("row must be inside workspace")
        if max_center_distance is None:
            max_center_distance = self.width * 0.25
        if (not isinstance(max_center_distance, (int, float)) or
                isinstance(max_center_distance, bool) or
                max_center_distance < 0):
            raise ValueError("max_center_distance must be non-negative")
        road, known = _validate_masks(mask_info, self.width, self.height)
        first_y = min(row, self.height - 1)
        last_y = max(-1, first_y - search_rows)
        self.seed_status = SEED_NO_ROAD
        for y in range(first_y, last_y, -1):
            valid_count = 0
            invalid_white = 0
            selected_left = -1
            selected_right = -1
            x = 0
            while x < self.width:
                index = y * self.width + x
                if known[index] == 0 or road[index] != WHITE:
                    x += 1
                    continue
                start = x
                while (x < self.width and known[y * self.width + x] != 0 and
                       road[y * self.width + x] == WHITE):
                    x += 1
                end = x - 1
                left_dark = (start > 0 and
                             known[y * self.width + start - 1] != 0 and
                             road[y * self.width + start - 1] == 0)
                right_dark = (end + 1 < self.width and
                              known[y * self.width + end + 1] != 0 and
                              road[y * self.width + end + 1] == 0)
                if not left_dark or not right_dark:
                    invalid_white += 1
                    continue
                valid_count += 1
                selected_left = start
                selected_right = end
            if valid_count > 1:
                self.seed_status = SEED_AMBIGUOUS
                return False
            if valid_count == 1:
                distance = 0.0
                if center_x < selected_left:
                    distance = selected_left - center_x
                elif center_x > selected_right:
                    distance = center_x - selected_right
                if distance > max_center_distance:
                    self.seed_status = SEED_TOO_FAR
                    return False
                self.left_seed_y = y
                self.left_seed_x = selected_left
                self.right_seed_y = y
                self.right_seed_x = selected_right
                self.seed_status = SEED_OK
                return True
            if invalid_white:
                self.seed_status = SEED_NO_DARK_BOUNDS
        return False

    def _seed_value(self, seed):
        try:
            y = seed[0]
            x = seed[1]
        except (TypeError, IndexError, KeyError):
            return -1, -1
        if not _valid_int(y) or not _valid_int(x):
            return -1, -1
        return y, x

    def _collides(self, side, y, x):
        if side == 0:
            points = self.right_points
            count = self.right_count
        else:
            points = self.left_points
            count = self.left_count
        for point_index in range(count):
            offset = point_index * 2
            if points[offset] == y and points[offset + 1] == x:
                return True
        return False

    def trace(self, mask_info, left_seed=None, right_seed=None):
        """Trace both borders, preferring lower lost count and left on ties.

        Seeds are ``(y, x)`` pairs.  Omit both seeds to invoke ``find_seeds``.
        The workspace itself is returned and exposes both flat point arrays,
        counts, true-dark support flags, per-row support totals, and reasons.
        """
        self._clear()
        road, known = _validate_masks(mask_info, self.width, self.height)
        if left_seed is None and right_seed is None:
            if not self.find_seeds(mask_info):
                self.left_termination = TERM_BAD_SEED
                self.right_termination = TERM_BAD_SEED
                return self
            left_y = self.left_seed_y
            left_x = self.left_seed_x
            right_y = self.right_seed_y
            right_x = self.right_seed_x
        elif left_seed is None or right_seed is None:
            self.left_termination = TERM_BAD_SEED
            self.right_termination = TERM_BAD_SEED
            self.seed_status = "ONE_SEED_MISSING"
            return self
        else:
            left_y, left_x = self._seed_value(left_seed)
            right_y, right_x = self._seed_value(right_seed)
            self.seed_status = SEED_OK

        if (left_y < 0 or left_y >= self.height or left_x < 0 or
                left_x >= self.width or right_y < 0 or
                right_y >= self.height or right_x < 0 or
                right_x >= self.width or (left_y == right_y and
                                          left_x == right_x) or
                not _is_white(road, known, self.width, self.height,
                              left_y, left_x) or
                not _is_white(road, known, self.width, self.height,
                              right_y, right_x) or
                not _support_at(road, known, self.width, self.height,
                                left_y, left_x, 0, 0) or
                not _support_at(road, known, self.width, self.height,
                                right_y, right_x, 0, 1)):
            self.left_termination = TERM_BAD_SEED
            self.right_termination = TERM_BAD_SEED
            self.seed_status = "INVALID_OR_UNSUPPORTED_SEED"
            return self

        left_dir = 0
        right_dir = 0
        left_error = 0
        right_error = 0
        self._append(0, left_y, left_x, left_dir, left_error, road, known)
        self._append(1, right_y, right_x, right_dir, right_error, road, known)
        active_left = 1
        active_right = 1

        while active_left or active_right:
            if self.iterations >= self.iteration_budget:
                if active_left:
                    self.left_termination = TERM_ITERATION_BUDGET
                if active_right:
                    self.right_termination = TERM_ITERATION_BUDGET
                break
            if active_left and active_right:
                if self.left_lost < self.right_lost:
                    side = 0
                elif self.right_lost < self.left_lost:
                    side = 1
                else:
                    side = 0
            else:
                side = 0 if active_left else 1
            self.iterations += 1

            if side == 0:
                y = left_y
                x = left_x
                direction = left_dir
                error = left_error
            else:
                y = right_y
                x = right_x
                direction = right_dir
                error = right_error

            path_count = self.left_count if side == 0 else self.right_count
            seed_y = left_y if side == 0 else right_y
            initial_bottom_seed = (
                path_count == 1 and y == seed_y and y == self.height - 1)
            if (_is_frame_edge(y, x, self.width, self.height) and
                    not initial_bottom_seed):
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, 5)
                if side == 0:
                    self.left_termination = TERM_EDGE
                    active_left = 0
                else:
                    self.right_termination = TERM_EDGE
                    active_right = 0
                continue

            state_status = self._state_seen(y, x, direction, error, side)
            if state_status != 0:
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost,
                                   6 if state_status > 0 else 7)
                if side == 0:
                    self.left_termination = (
                        TERM_REPEATED_STATE if state_status > 0 else
                        TERM_STATE_TABLE_FULL)
                    active_left = 0
                else:
                    self.right_termination = (
                        TERM_REPEATED_STATE if state_status > 0 else
                        TERM_STATE_TABLE_FULL)
                    active_right = 0
                continue

            if side == 0:
                side_direction = (direction + 3) & 3
            else:
                side_direction = (direction + 1) & 3
            side_y = y + _DY[side_direction]
            side_x = x + _DX[side_direction]
            diagonal_direction = (direction + (3 if side == 0 else 1)) & 3
            diagonal_y = y + _DY[direction] + _DY[diagonal_direction]
            diagonal_x = x + _DX[direction] + _DX[diagonal_direction]
            front_y = y + _DY[direction]
            front_x = x + _DX[direction]
            side_wall = _is_nonwhite(road, known, self.width, self.height,
                                     side_y, side_x)
            diagonal_wall = _is_nonwhite(
                road, known, self.width, self.height,
                diagonal_y, diagonal_x)
            diagonal_white = _is_white(
                road, known, self.width, self.height,
                diagonal_y, diagonal_x)
            front_white = _is_white(
                road, known, self.width, self.height, front_y, front_x)

            move = 0
            step_action = 0
            next_y = y
            next_x = x
            if side_wall and diagonal_wall:
                if front_white:
                    next_y = front_y
                    next_x = front_x
                    move = 1
                    step_action = 1
                else:
                    direction = (direction + (1 if side == 0 else 3)) & 3
                    error += 1
                    step_action = 3 if side == 0 else 4
            elif side_wall and diagonal_white:
                next_y = diagonal_y
                next_x = diagonal_x
                direction = (direction + (3 if side == 0 else 1)) & 3
                error -= 1
                move = 1
                step_action = 4 if side == 0 else 3
            else:
                direction = (direction + (1 if side == 0 else 3)) & 3
                error += 1
                step_action = 3 if side == 0 else 4

            if error >= 10 or error <= -10:
                if side == 0:
                    self.left_final_error = error
                else:
                    self.right_final_error = error
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, step_action)
                if side == 0:
                    self.left_termination = TERM_DIRECTION_LIMIT
                    active_left = 0
                else:
                    self.right_termination = TERM_DIRECTION_LIMIT
                    active_right = 0
                continue
            if not move:
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, step_action)
                if side == 0:
                    left_dir = direction
                    left_error = error
                    self.left_final_error = error
                else:
                    right_dir = direction
                    right_error = error
                    self.right_final_error = error
                continue

            if (next_y < 0 or next_y >= self.height or next_x < 0 or
                    next_x >= self.width):
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, 5)
                if side == 0:
                    self.left_termination = TERM_EDGE
                    active_left = 0
                else:
                    self.right_termination = TERM_EDGE
                    active_right = 0
                continue
            if self._collides(side, next_y, next_x):
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, 8)
                if side == 0:
                    self.left_termination = TERM_COLLISION
                    active_left = 0
                    if active_right:
                        self.right_termination = TERM_COLLISION
                        active_right = 0
                else:
                    self.right_termination = TERM_COLLISION
                    active_right = 0
                    if active_left:
                        self.left_termination = TERM_COLLISION
                        active_left = 0
                continue
            count = self.left_count if side == 0 else self.right_count
            if count >= self.max_points:
                self._record_event(side, y, x, direction, error,
                                   self.left_lost if side == 0 else
                                   self.right_lost, 9)
                if side == 0:
                    self.left_termination = TERM_CAPACITY
                    active_left = 0
                else:
                    self.right_termination = TERM_CAPACITY
                    active_right = 0
                continue

            if _get_image_sum(road, known, self.width, self.height,
                              side_y, side_x) == UNKNOWN:
                if side == 0:
                    self.left_lost += 1
                else:
                    self.right_lost += 1
            self._append(side, next_y, next_x, direction, error, road, known)
            if side == 0:
                left_y = next_y
                left_x = next_x
                left_dir = direction
                left_error = error
                self.left_final_error = error
                self._record_event(0, left_y, left_x, left_dir, left_error,
                                   self.left_lost, step_action)
                if _is_frame_edge(left_y, left_x, self.width, self.height):
                    self.left_termination = TERM_EDGE
                    active_left = 0
            else:
                right_y = next_y
                right_x = next_x
                right_dir = direction
                right_error = error
                self.right_final_error = error
                self._record_event(1, right_y, right_x, right_dir, right_error,
                                   self.right_lost, step_action)
                if _is_frame_edge(right_y, right_x, self.width, self.height):
                    self.right_termination = TERM_EDGE
                    active_right = 0

        if active_left:
            self.left_termination = TERM_ITERATION_BUDGET
        if active_right:
            self.right_termination = TERM_ITERATION_BUDGET
        return self


class ContourPostWorkspace:
    """Preallocated bounded filter, arc sampler, offsetter, and corner picker."""

    def __init__(self, width=80, height=60, max_samples=900):
        if (not _valid_int(width) or not _valid_int(height) or width < 1 or
                height < 1 or not _valid_int(max_samples) or max_samples < 1):
            raise ValueError("invalid geometry workspace dimensions")
        self.width = width
        self.height = height
        self.max_samples = max_samples
        self.filtered_points = _array_fill("i", 0, 2 * max_samples)
        self.resampled_points = _array_fill("f", 0.0, 2 * max_samples)
        self.resample_breaks = bytearray(max_samples)
        self.offset_points = _array_fill("f", 0.0, 2 * max_samples)
        self.offset_valid = bytearray(max_samples)
        self.corner_indices = _array_fill("H", 0, max_samples)
        self.resample_truncated = False
        self.resampled_count = 0
        self.corner_count = 0

    def _check_path(self, points, count):
        if (not _valid_int(count) or count < 0 or
                count > self.max_samples or len(points) < count * 2):
            raise ValueError("invalid point buffer or count")

    def _point_known(self, points, index, known_region):
        y = int(points[index * 2])
        x = int(points[index * 2 + 1])
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return known_region[y * self.width + x] != 0

    def filter_path(self, points, count, known_region, radius=2):
        """Bounded moving average preserving endpoints and unknown barriers."""
        self._check_path(points, count)
        if len(known_region) != self.width * self.height:
            raise ValueError("known_region size mismatch")
        if not _valid_int(radius) or radius < 0:
            raise ValueError("radius must be a non-negative integer")
        out = self.filtered_points
        index = 0
        while index < count:
            if not self._point_known(points, index, known_region):
                out[index * 2] = int(points[index * 2])
                out[index * 2 + 1] = int(points[index * 2 + 1])
                index += 1
                continue
            segment_start = index
            index += 1
            while index < count and self._point_known(
                    points, index, known_region):
                index += 1
            segment_end = index - 1
            for point_index in range(segment_start, segment_end + 1):
                out_offset = point_index * 2
                if point_index == segment_start or point_index == segment_end:
                    out[out_offset] = int(points[out_offset])
                    out[out_offset + 1] = int(points[out_offset + 1])
                    continue
                first = max(segment_start, point_index - radius)
                last = min(segment_end, point_index + radius)
                sum_y = 0
                sum_x = 0
                samples = 0
                for sample in range(first, last + 1):
                    sample_offset = sample * 2
                    sum_y += int(points[sample_offset])
                    sum_x += int(points[sample_offset + 1])
                    samples += 1
                out[out_offset] = (sum_y + samples // 2) // samples
                out[out_offset + 1] = (sum_x + samples // 2) // samples
        return out, count

    def resample_path(self, points, count, known_region, spacing):
        """Equal arc-length samples, split at every unknown or clipped point."""
        self._check_path(points, count)
        if len(known_region) != self.width * self.height:
            raise ValueError("known_region size mismatch")
        if (not isinstance(spacing, (int, float)) or isinstance(spacing, bool)
                or spacing <= 0 or spacing != spacing):
            raise ValueError("spacing must be positive and finite")
        out = self.resampled_points
        breaks = self.resample_breaks
        out_count = 0
        self.resample_truncated = False
        for index in range(self.max_samples):
            breaks[index] = 0
        index = 0
        while index < count:
            if not self._point_known(points, index, known_region):
                index += 1
                continue
            segment_start = index
            index += 1
            while index < count and self._point_known(
                    points, index, known_region):
                index += 1
            segment_end = index - 1
            if out_count >= self.max_samples:
                self.resample_truncated = True
                break
            start_offset = segment_start * 2
            out[out_count * 2] = float(points[start_offset])
            out[out_count * 2 + 1] = float(points[start_offset + 1])
            breaks[out_count] = 1
            out_count += 1
            next_target = float(spacing)
            traveled = 0.0
            for point_index in range(segment_start, segment_end):
                first_offset = point_index * 2
                second_offset = first_offset + 2
                y0 = float(points[first_offset])
                x0 = float(points[first_offset + 1])
                y1 = float(points[second_offset])
                x1 = float(points[second_offset + 1])
                dy = y1 - y0
                dx = x1 - x0
                length = math.sqrt(dy * dy + dx * dx)
                if length <= 0.0:
                    continue
                while (next_target < traveled + length and
                       next_target < 0.0 + float("inf")):
                    if out_count >= self.max_samples:
                        self.resample_truncated = True
                        break
                    ratio = (next_target - traveled) / length
                    out[out_count * 2] = y0 + dy * ratio
                    out[out_count * 2 + 1] = x0 + dx * ratio
                    out_count += 1
                    next_target += float(spacing)
                if self.resample_truncated:
                    break
                traveled += length
            if self.resample_truncated:
                break
            if (out_count >= self.max_samples):
                self.resample_truncated = True
                break
            end_offset = segment_end * 2
            end_y = float(points[end_offset])
            end_x = float(points[end_offset + 1])
            if (out_count == 0 or out[out_count * 2] != end_y or
                    out[out_count * 2 + 1] != end_x):
                out[out_count * 2] = end_y
                out[out_count * 2 + 1] = end_x
                out_count += 1
        self.resampled_count = out_count
        return out, out_count, breaks

    def offset_inward(self, points, count, support, known_region, side,
                      width):
        """Offset supported boundary samples into road by explicit ``width``.

        ``side`` is ``"left"`` or ``"right"`` for a bottom-to-top trace.
        Output entries keep input indices; ``offset_valid`` marks entries
        whose source support and complete sampled offset segment are known.
        """
        self._check_path(points, count)
        if len(support) < count or len(known_region) != self.width * self.height:
            raise ValueError("support or known_region size mismatch")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if (not isinstance(width, (int, float)) or isinstance(width, bool) or
                width <= 0 or width != width or width == float("inf")):
            raise ValueError("width must be positive and finite")
        out = self.offset_points
        valid = self.offset_valid
        for index in range(count):
            valid[index] = 0
        for index in range(count):
            if support[index] == 0 or not self._point_known(
                    points, index, known_region):
                continue
            segment_start = index
            while (segment_start > 0 and support[segment_start - 1] and
                   self._point_known(points, segment_start - 1, known_region)):
                segment_start -= 1
            segment_end = index
            while (segment_end + 1 < count and support[segment_end + 1] and
                   self._point_known(points, segment_end + 1, known_region)):
                segment_end += 1
            if segment_start == segment_end:
                continue
            before = index - 1 if index > segment_start else index
            after = index + 1 if index < segment_end else index
            by = float(points[after * 2]) - float(points[before * 2])
            bx = float(points[after * 2 + 1]) - float(points[before * 2 + 1])
            length = math.sqrt(by * by + bx * bx)
            if length <= 0.0:
                continue
            if side == "left":
                offset_y = bx / length * width
                offset_x = -by / length * width
            else:
                offset_y = -bx / length * width
                offset_x = by / length * width
            source_y = float(points[index * 2])
            source_x = float(points[index * 2 + 1])
            target_y = source_y + offset_y
            target_x = source_x + offset_x
            samples = max(1, int(math.ceil(width * 2.0)))
            trusted = 1
            for step in range(1, samples + 1):
                ratio = float(step) / samples
                sample_y = int(source_y + offset_y * ratio + 0.5)
                sample_x = int(source_x + offset_x * ratio + 0.5)
                if (sample_x < 0 or sample_x >= self.width or sample_y < 0 or
                        sample_y >= self.height or
                        known_region[sample_y * self.width + sample_x] == 0):
                    trusted = 0
                    break
            if trusted:
                out[index * 2] = target_y
                out[index * 2 + 1] = target_x
                valid[index] = 1
        return out, valid, count

    def extract_corners(self, points, count, known_region, min_angle_deg=25.0):
        """Return indices whose local turn angle meets ``min_angle_deg``."""
        self._check_path(points, count)
        if len(known_region) != self.width * self.height:
            raise ValueError("known_region size mismatch")
        if (not isinstance(min_angle_deg, (int, float)) or
                isinstance(min_angle_deg, bool) or min_angle_deg < 0 or
                min_angle_deg > 180):
            raise ValueError("min_angle_deg must be in 0..180")
        corner_count = 0
        if count < 3:
            self.corner_count = 0
            return self.corner_indices, 0
        cosine_limit = math.cos(float(min_angle_deg) * math.pi / 180.0)
        for index in range(1, count - 1):
            if (not self._point_known(points, index - 1, known_region) or
                    not self._point_known(points, index, known_region) or
                    not self._point_known(points, index + 1, known_region)):
                continue
            first_y = float(points[index * 2] - points[(index - 1) * 2])
            first_x = float(points[index * 2 + 1] - points[(index - 1) * 2 + 1])
            second_y = float(points[(index + 1) * 2] - points[index * 2])
            second_x = float(points[(index + 1) * 2 + 1] - points[index * 2 + 1])
            first_len = math.sqrt(first_y * first_y + first_x * first_x)
            second_len = math.sqrt(second_y * second_y + second_x * second_x)
            if first_len <= 0.0 or second_len <= 0.0:
                continue
            cosine = ((first_y * second_y + first_x * second_x) /
                      (first_len * second_len))
            if cosine < -1.0:
                cosine = -1.0
            elif cosine > 1.0:
                cosine = 1.0
            if cosine <= cosine_limit:
                self.corner_indices[corner_count] = index
                corner_count += 1
        self.corner_count = corner_count
        return self.corner_indices, corner_count
