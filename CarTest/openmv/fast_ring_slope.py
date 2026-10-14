"""Bounded current-frame ring-port and flat-baseline slope hint evidence.

This module reports geometry candidates only. A ring route requires a fresh
control lease and three consecutive current-frame arc+port observations. A
slope hint requires ten stable straight frames to learn a flat baseline and
three current-frame candidates after that baseline exists.
"""

from array import array

try:
    import micropython
    _IS_MICROPYTHON = True
    COMPILER_BACKEND = "micropython-viper"
except ImportError:
    _IS_MICROPYTHON = False
    COMPILER_BACKEND = "cpython-reference"

    class _MicroPythonHostShim:
        def viper(self, function):
            return function

        def const(self, value):
            return value

    micropython = _MicroPythonHostShim()

    def ptr8(buffer):
        return buffer

    def ptr32(buffer):
        return buffer


RUNTIME_API_VERSION = 1
_WIDTH = micropython.const(80)
_HEIGHT = micropython.const(60)
_PIXELS = micropython.const(_WIDTH * _HEIGHT)
_MAX_ROWS = micropython.const(_HEIGHT)
_MAX_PIXELS = micropython.const(_PIXELS)
RING_NONE = 0
RING_LEFT = 1
RING_RIGHT = 2
_SIDE_TEXT = ("NONE", "LEFT", "RIGHT")
_STATE_IDLE = "IDLE"
_STATE_PREVIEW = "PREVIEW"
_STATE_ENTRY_LEFT = "ENTRY_LEFT"
_STATE_ENTRY_RIGHT = "ENTRY_RIGHT"
_STATE_INSIDE_LEFT = "INSIDE_LEFT"
_STATE_INSIDE_RIGHT = "INSIDE_RIGHT"
_STATE_EXIT = "EXIT"
_STATE_COOLDOWN = "COOLDOWN"
_ROW_STRIDE = 4


def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def _round_away(value):
    if value >= 0.0:
        return int(value + 0.5)
    return -int(-value + 0.5)


@micropython.viper
def _flood_component_viper(self, seed: int, port_start_y: int,
                            port_end_y: int, side_id: int) -> int:
    """Walk only known road pixels with measured-arc and dark-edge clearance."""
    road = ptr8(self._current_road_mask)
    edge = ptr8(self._current_edge_mask)
    known = ptr8(self._current_known_mask)
    visited = ptr8(self._visited)
    queue = ptr32(self._queue)
    parent = ptr32(self._parent)
    arc_x = ptr32(self._arc_x)
    port_rows = ptr8(self._port_rows)
    port_cell = ptr32(self._port_cell_by_row)
    width = int(self.width)
    height = int(self.height)
    floor_y = int(self.floor_y)
    port_inset_margin = int(self.port_inset_margin_cells)
    total = width * height
    for index in range(total):
        visited[index] = 0
    for y in range(height):
        port_rows[y] = 0
        port_cell[y] = -1
    head = 0
    tail = 1
    queue[0] = seed
    parent[seed] = -1
    visited[seed] = 1
    while head < tail:
        cell = int(queue[head])
        head += 1
        y = cell // width
        x = cell - y * width
        if port_start_y <= y <= port_end_y:
            if side_id == 2 and x >= width - 1 - port_inset_margin:
                port_rows[y] = 1
                current_port = int(port_cell[y])
                if (current_port == -1 or
                        x > current_port - (current_port // width) * width):
                    port_cell[y] = cell
            elif side_id == 1 and x <= port_inset_margin:
                port_rows[y] = 1
                current_port = int(port_cell[y])
                if (current_port == -1 or
                        x < current_port - (current_port // width) * width):
                    port_cell[y] = cell

        for direction in range(4):
            if direction == 0:
                nx = x - 1
                ny = y
                neighbor = cell - 1
                inside = x > 0
            elif direction == 1:
                nx = x + 1
                ny = y
                neighbor = cell + 1
                inside = x + 1 < width
            elif direction == 2:
                nx = x
                ny = y - 1
                neighbor = cell - width
                inside = y > floor_y
            else:
                nx = x
                ny = y + 1
                neighbor = cell + width
                inside = y + 1 < height
            if not inside or not known[neighbor] or not road[neighbor] or \
                    visited[neighbor]:
                continue
            boundary_x = int(arc_x[ny])
            if boundary_x >= 0:
                if side_id == 2 and nx > boundary_x - 2:
                    continue
                if side_id == 1 and nx < boundary_x + 2:
                    continue

            # Every in-frame route cell remains known and two cells from dark
            # pixels. Only coordinates outside the image are ignored.
            clear = True
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    abs_dx = dx if dx >= 0 else -dx
                    abs_dy = dy if dy >= 0 else -dy
                    if abs_dx + abs_dy > 2:
                        continue
                    check_x = nx + dx
                    check_y = ny + dy
                    if check_x < 0 or check_x >= width:
                        continue
                    if check_y < floor_y:
                        clear = False
                        break
                    if check_y >= height:
                        continue
                    check = check_y * width + check_x
                    if not known[check] or edge[check]:
                        clear = False
                        break
                if not clear:
                    break
            if not clear:
                continue
            if tail >= total:
                return -1
            visited[neighbor] = 1
            parent[neighbor] = cell
            queue[tail] = neighbor
            tail += 1
    return tail


class FastRingSlope:
    """Keep all candidate/state work bounded by the configured 80x60 grid."""

    def __init__(self, settings):
        self.width = int(settings.SURFACE_GRID_WIDTH)
        self.height = int(settings.SURFACE_GRID_HEIGHT)
        self.floor_y = int(settings.SURFACE_ROI_FLOOR_Y)
        self.port_inset_margin_cells = int(
            settings.RING_PORT_INSET_MARGIN_CELLS)
        if (self.width != 80 or self.height != 60 or self.floor_y < 0 or
                self.port_inset_margin_cells != self.width // 20):
            raise ValueError("ring/slope requires an 80x60 configured grid")

        # y, measured left boundary, measured right boundary, source row index.
        self._rows = array("i", [0] * (_MAX_ROWS * _ROW_STRIDE))
        self._row_valid = bytearray(self.height)
        self._row_count = 0
        self._arc_x = array("i", [-1] * self.height)
        self._visited = bytearray(_MAX_PIXELS)
        self._queue = array("i", [0] * _MAX_PIXELS)
        self._parent = array("i", [-1] * _MAX_PIXELS)
        self._port_rows = bytearray(self.height)
        self._port_cell_by_row = array("i", [-1] * self.height)
        self._path_x_rows = array("i", [0] * self.height)
        self._path_x_min = array("i", [0] * self.height)
        self._path_x_max = array("i", [0] * self.height)
        self._path_valid_rows = bytearray(self.height)
        self._candidate = {
            "valid": False, "ambiguous": False, "reason": "NOT_RUN",
            "arc_detected": False,
            "side": "NONE", "side_id": RING_NONE,
            "arc_rows": 0, "arc_y_min": None, "arc_y_max": None,
            "vertex_y": None, "vertex_x": None, "intrusion": 0,
            "near_inner_x": None,
            "near_seed_x": None, "near_seed_y": None,
            "near_seed_search_min_y": None,
            "partial_inner_current": False,
            "inner_edge_receding": False,
            "inner_edge_near_clip": False,
            "inner_edge_disappeared": False,
            "inner_edge_rows": 0,
            "outer_outlier_excluded_y": None,
            "port_x": None, "port_y": None, "clip_endpoint_x": None,
            "port_inset_margin_cells": self.port_inset_margin_cells,
            "port_search_y_min": None, "port_search_y_max": None,
            "port_segments": 0,
            "near_center_x": None, "near_center_y": None,
            "heading_error": 0,
            "boundary_source": "NONE", "planned_center_source": "NONE",
            "path_x_rows": self._path_x_rows,
            "path_valid_rows": self._path_valid_rows,
            "path_row_count": 0,
        }
        self._ring_state = _STATE_IDLE
        self._ring_side = RING_NONE
        self._pending_side = RING_NONE
        self._pending_port_y = -1
        self._pending_count = 0
        self._exit_count = 0
        self._cooldown_count = 0
        self._generation = None
        self._ring_reason = "RESET"
        self._ring_active = False
        self._last_inner_x = None
        self.ring = {
            "candidate": self._candidate,
            "state": _STATE_IDLE,
            "side_id": RING_NONE,
            "side": "NONE",
            "active": False,
            "reason": "RESET",
            "pending_count": 0,
            "exit_count": 0,
            "cooldown_count": 0,
        }

        # Slope fit points are integer grid coordinates in y=30..50.
        self._fit_y = array("i", [0] * _MAX_ROWS)
        self._fit_left = array("i", [0] * _MAX_ROWS)
        self._fit_right = array("i", [0] * _MAX_ROWS)
        self._fit_width = array("i", [0] * _MAX_ROWS)
        self._fit_center2 = array("i", [0] * _MAX_ROWS)
        self._fit_count = 0
        self._learn_h0 = array("i", [0] * 10)
        self._learn_slope_milli = array("i", [0] * 10)
        self._learn_count = 0
        self._baseline_h0 = None
        self._baseline_slope = None
        self._baseline_update_count = 0
        self._baseline_update_h0 = array("i", [0] * 10)
        self._baseline_update_slope = array("i", [0] * 10)
        self._slope_candidate_count = 0
        self._slope_exit_count = 0
        self._slope_hint = 0
        self._slope_reason = "NEED_BASELINE"
        self.slope = {
            "hint": 0, "stage": "NEED_BASELINE", "reason": "NEED_BASELINE",
            "fit_valid": False, "fit_rows": 0, "fit_y_min": None,
            "fit_y_max": None, "center_slope": None,
            "center_residual_max": None, "width_slope": None,
            "width_residual_max": None, "zero_width_y": None,
            "baseline_zero_width_y": None, "baseline_width_slope": None,
            "candidate_count": 0, "learn_count": 0,
        }

    def _clear_ring(self, reason):
        self._ring_state = _STATE_IDLE
        self._ring_side = RING_NONE
        self._pending_side = RING_NONE
        self._pending_port_y = -1
        self._pending_count = 0
        self._exit_count = 0
        self._cooldown_count = 0
        self._ring_reason = reason
        self._ring_active = False
        self._last_inner_x = None
        self._publish_ring_state()

    def _clear_slope_candidate(self, reason):
        self._slope_candidate_count = 0
        self._slope_exit_count = 0
        self._slope_hint = 0
        self._slope_reason = reason
        self.slope["hint"] = 0
        self.slope["candidate_count"] = 0
        self.slope["reason"] = reason
        self.slope["stage"] = (
            "NEED_BASELINE" if self._baseline_h0 is None else "BASELINE")

    def clear(self, reason="RESET"):
        """Clear ring state and learned slope baseline on session boundaries."""
        self._generation = None
        self._clear_ring(reason)
        self._learn_count = 0
        self._baseline_h0 = None
        self._baseline_slope = None
        self._baseline_update_count = 0
        self._clear_slope_candidate(reason)
        self.slope["baseline_zero_width_y"] = None
        self.slope["baseline_width_slope"] = None
        self.slope["learn_count"] = 0

    def _publish_ring_state(self):
        self.ring["state"] = self._ring_state
        self.ring["side_id"] = self._ring_side
        self.ring["side"] = _SIDE_TEXT[self._ring_side]
        self.ring["active"] = self._ring_active
        self.ring["reason"] = self._ring_reason
        self.ring["pending_count"] = self._pending_count
        self.ring["exit_count"] = self._exit_count
        self.ring["cooldown_count"] = self._cooldown_count

    def _reset_candidate(self, reason):
        candidate = self._candidate
        candidate["valid"] = False
        candidate["ambiguous"] = False
        candidate["reason"] = reason
        candidate["arc_detected"] = False
        candidate["side"] = "NONE"
        candidate["side_id"] = RING_NONE
        candidate["arc_rows"] = 0
        candidate["arc_y_min"] = None
        candidate["arc_y_max"] = None
        candidate["vertex_y"] = None
        candidate["vertex_x"] = None
        candidate["intrusion"] = 0
        candidate["near_inner_x"] = None
        candidate["near_seed_x"] = None
        candidate["near_seed_y"] = None
        candidate["near_seed_search_min_y"] = None
        candidate["partial_inner_current"] = False
        candidate["inner_edge_receding"] = False
        candidate["inner_edge_near_clip"] = False
        candidate["inner_edge_disappeared"] = False
        candidate["inner_edge_rows"] = 0
        candidate["outer_outlier_excluded_y"] = None
        candidate["port_x"] = None
        candidate["port_y"] = None
        candidate["clip_endpoint_x"] = None
        candidate["port_inset_margin_cells"] = self.port_inset_margin_cells
        candidate["port_search_y_min"] = None
        candidate["port_search_y_max"] = None
        candidate["port_segments"] = 0
        candidate["near_center_x"] = None
        candidate["near_center_y"] = None
        candidate["heading_error"] = 0
        candidate["boundary_source"] = "NONE"
        candidate["planned_center_source"] = "NONE"
        candidate["path_row_count"] = 0
        for index in range(self.height):
            self._path_valid_rows[index] = 0
        self._row_count = 0
        return candidate

    def _extract_supported_rows(self, road_result, support_rows,
                                support_count):
        self._row_count = 0
        for y in range(self.height):
            self._row_valid[y] = 0
        rows = road_result.get("selected_row_ints")
        count = road_result.get("selected_row_count", 0)
        stride = road_result.get("selected_row_stride", 0)
        if (not isinstance(rows, array) or not isinstance(count, int) or
                isinstance(count, bool) or count < 1 or count > _MAX_ROWS or
                stride != 4 or len(rows) < count * stride or
                not isinstance(support_rows, bytearray) or
                support_count != count or len(support_rows) < count):
            return False

        out = self._rows
        for index in range(count):
            base = index * stride
            y = int(rows[base])
            left = int(rows[base + 1])
            right = int(rows[base + 2])
            flags = int(rows[base + 3])
            if (support_rows[index] != 1 or flags & 7 != 7 or
                    y < self.floor_y or y >= self.height or left < 1 or
                    right < left or right >= self.width - 1):
                continue
            # Store by row index first; one bounded y scan below compacts in
            # image order without a Python insertion-sort path.
            record = y * _ROW_STRIDE
            if self._row_valid[y]:
                continue
            out[record] = y
            out[record + 1] = left - 1
            out[record + 2] = right + 1
            out[record + 3] = index
            self._row_valid[y] = 1

        row_count = 0
        for y in range(self.floor_y, self.height):
            if not self._row_valid[y]:
                continue
            source = y * _ROW_STRIDE
            target = row_count * _ROW_STRIDE
            row_y = out[source]
            left_x = out[source + 1]
            right_x = out[source + 2]
            source_index = out[source + 3]
            out[target] = row_y
            out[target + 1] = left_x
            out[target + 2] = right_x
            out[target + 3] = source_index
            row_count += 1
        self._row_count = row_count
        return row_count > 0

    def _arc_candidate(self, side_id):
        count = self._row_count
        rows = self._rows
        if count < 10 or rows[(count - 1) * _ROW_STRIDE] - rows[0] < 12:
            return None
        inner_offset = 1 if side_id == RING_LEFT else 2
        outer_offset = 2 if side_id == RING_LEFT else 1
        extreme = (self.width if side_id == RING_RIGHT else -1)
        first_extreme = -1
        last_extreme = -1
        for index in range(count):
            x = rows[index * _ROW_STRIDE + inner_offset]
            if ((side_id == RING_RIGHT and x < extreme) or
                    (side_id == RING_LEFT and x > extreme)):
                extreme = x
                first_extreme = index
                last_extreme = index
            elif x == extreme:
                if first_extreme < 0:
                    first_extreme = index
                last_extreme = index
        if first_extreme < 3 or count - 1 - last_extreme < 3:
            return None

        # One unique convex extremum is accepted. A second reversal anywhere
        # in the current supported profile rejects the ring candidate.
        for index in range(1, count):
            before = (index - 1) * _ROW_STRIDE
            current = index * _ROW_STRIDE
            dy = rows[current] - rows[before]
            x0 = rows[before + inner_offset]
            x1 = rows[current + inner_offset]
            if dy <= 0 or dy > 2:
                return None
            if index <= first_extreme:
                if ((side_id == RING_RIGHT and x1 > x0) or
                        (side_id == RING_LEFT and x1 < x0)):
                    return None
            elif index > last_extreme:
                if ((side_id == RING_RIGHT and x1 < x0) or
                        (side_id == RING_LEFT and x1 > x0)):
                    return None
        vertex_index = (first_extreme + last_extreme) // 2
        vertex_base = vertex_index * _ROW_STRIDE
        vertex_y = rows[vertex_base]
        vertex_x = extreme
        y_min = rows[0]
        y_max = rows[(count - 1) * _ROW_STRIDE]
        if (vertex_y - y_min < 6 or y_max - vertex_y < 6 or
                y_max - y_min < 12):
            return None
        endpoint_before = rows[inner_offset]
        endpoint_after = rows[(count - 1) * _ROW_STRIDE + inner_offset]
        if side_id == RING_RIGHT:
            depth_before = endpoint_before - vertex_x
            depth_after = endpoint_after - vertex_x
        else:
            depth_before = vertex_x - endpoint_before
            depth_after = vertex_x - endpoint_after
        intrusion = min(depth_before, depth_after)
        if intrusion < 6:
            return None

        # The opposite measured edge must remain continuous and plausible. A
        # single isolated row may be omitted from this check only when exactly
        # one step violates the existing limit, its neighbors bridge within
        # that same limit, and the row is at least 2.5 cells from their linear
        # midpoint. The row remains real evidence for the inner arc; nothing
        # is interpolated into the output.
        outer_violation = -1
        for index in range(1, count):
            before = (index - 1) * _ROW_STRIDE
            current = index * _ROW_STRIDE
            dy = rows[current] - rows[before]
            x0 = rows[before + outer_offset]
            x1 = rows[current + outer_offset]
            if dy <= 0 or dy > 2:
                return None
            if abs(x1 - x0) > 3 * dy:
                if outer_violation >= 0:
                    return None
                outer_violation = index
        excluded_outer_y = None
        if outer_violation >= 0:
            index = outer_violation
            if index <= 1 or index >= count - 1 or count - 1 < 10:
                return None
            before = (index - 1) * _ROW_STRIDE
            current = index * _ROW_STRIDE
            after = (index + 1) * _ROW_STRIDE
            y0 = rows[before]
            y1 = rows[current]
            y2 = rows[after]
            x0 = rows[before + outer_offset]
            x1 = rows[current + outer_offset]
            x2 = rows[after + outer_offset]
            bridge_dy = y2 - y0
            if (bridge_dy <= 0 or bridge_dy > 4 or
                    abs(x2 - x0) > 3 * bridge_dy or
                    abs(2 * x1 - x0 - x2) < 5):
                return None
            excluded_outer_y = y1
        return {
            "side_id": side_id,
            "vertex_index": vertex_index,
            "vertex_y": vertex_y,
            "vertex_x": vertex_x,
            "y_min": y_min,
            "y_max": y_max,
            "start_index": 0,
            "end_index": count - 1,
            "rows": count,
            "intrusion": intrusion,
            "outer_outlier_excluded_y": excluded_outer_y,
        }

    def _flood_port(self, mask_info, arc):
        width = self.width
        height = self.height
        total = width * height
        road = mask_info.get("road_mask")
        edge = mask_info.get("edge_mask")
        known = mask_info.get("known_region")
        if (not isinstance(road, bytearray) or
                not isinstance(edge, bytearray) or
                not isinstance(known, bytearray) or
                len(road) != total or len(edge) != total or
                len(known) != total):
            return False, -1, 0, "MASK_INVALID"

        for index in range(height):
            self._arc_x[index] = -1
            self._port_rows[index] = 0
            self._port_cell_by_row[index] = -1
            self._path_valid_rows[index] = 0
        rows = self._rows
        for index in range(arc["start_index"], arc["end_index"] + 1):
            base = index * _ROW_STRIDE
            y = rows[base]
            edge_field = 1 if arc["side_id"] == RING_LEFT else 2
            self._arc_x[y] = rows[base + edge_field]

        near_base = arc["end_index"] * _ROW_STRIDE
        near_y = rows[near_base]
        seed_min_y = max(self.floor_y, near_y - 4)
        self._candidate["near_seed_search_min_y"] = seed_min_y
        seed = -1
        seed_x = -1
        seed_y = -1
        had_current_white_seed = False
        # Prefer the nearest real, per-row-supported center, then step back by
        # at most four grid rows if bottom-border pixels make its full safety
        # clearance unusable. No boundary or center is interpolated.
        for row_index in range(arc["end_index"], -1, -1):
            base = row_index * _ROW_STRIDE
            row_y = rows[base]
            if row_y < seed_min_y:
                break
            row_seed_x = (rows[base + 1] + rows[base + 2]) // 2
            if row_seed_x < 0 or row_seed_x >= width:
                continue
            row_seed = row_y * width + row_seed_x
            if (row_seed < 0 or row_seed >= total or not known[row_seed] or
                    not road[row_seed]):
                continue
            had_current_white_seed = True
            if not self._dark_clear(row_seed_x, row_y, edge, known):
                continue
            seed = row_seed
            seed_x = row_seed_x
            seed_y = row_y
            break
        if seed < 0:
            return False, -1, 0, (
                "NO_SEED_BOUNDARY_CLEARANCE" if had_current_white_seed else
                "NO_NEAR_ROAD_SEED")
        self._candidate["near_seed_x"] = seed_x
        self._candidate["near_seed_y"] = seed_y

        side_id = arc["side_id"]
        # Image y increases toward the camera. Search the known current white
        # corridor from just before the measured arc through the far side of
        # its vertex. This permits a real port beside the far arc while the
        # mask, arc fence, and dark-edge clearance still validate every cell.
        port_start_y = max(self.floor_y, arc["y_min"] - 4)
        port_end_y = min(self.height - 1, arc["vertex_y"] - 1)
        if port_end_y < port_start_y:
            return False, -1, 0, "NO_FAR_ARC_PORT_BAND"

        self._current_road_mask = road
        self._current_edge_mask = edge
        self._current_known_mask = known
        self._port_start_y = port_start_y
        self._port_end_y = port_end_y
        self._candidate["port_search_y_min"] = port_start_y
        self._candidate["port_search_y_max"] = port_end_y
        if _IS_MICROPYTHON:
            walk_result = _flood_component_viper(
                self, seed, port_start_y, port_end_y, side_id)
        else:
            walk_result = self._flood_component_host(
                seed, port_start_y, port_end_y, side_id)
        if walk_result < 0:
            return False, -1, 0, "PORT_QUEUE_OVERFLOW"

        first_y = -1
        last_y = -1
        segments = 0
        in_segment = False
        for y in range(port_start_y, port_end_y + 1):
            if self._port_rows[y]:
                if not in_segment:
                    segments += 1
                    if first_y < 0:
                        first_y = y
                last_y = y
                in_segment = True
            else:
                in_segment = False
        if segments != 1:
            return False, -1, segments, (
                "NO_CONNECTED_PORT" if segments == 0 else "MULTIPLE_PORTS")
        port_y = (first_y + last_y) // 2
        port_cell = self._port_cell_by_row[port_y]
        if port_cell < 0:
            return False, -1, segments, "PORT_PATH_MISSING"
        if not self._build_path(port_cell, seed, road, known, side_id):
            return False, -1, segments, "PORT_PATH_INVALID"
        return True, port_y, segments, "OK"

    def _dark_clear(self, x, y, edge, known, allow_crop=False,
                    side_id=RING_NONE):
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                if abs(dx) + abs(dy) > 2:
                    continue
                check_x = x + dx
                check_y = y + dy
                if check_x < 0 or check_x >= self.width:
                    continue
                if check_y < self.floor_y:
                    return False
                if check_y >= self.height:
                    continue
                check = check_y * self.width + check_x
                if not known[check] or edge[check]:
                    return False
        return True

    def _flood_component_host(self, seed, port_start_y, port_end_y, side_id):
        """Scalar equivalent of the board Viper flood for host tests."""
        road = self._current_road_mask
        edge = self._current_edge_mask
        known = self._current_known_mask
        visited = self._visited
        parent = self._parent
        queue = self._queue
        port_rows = self._port_rows
        port_cell = self._port_cell_by_row
        total = self.width * self.height
        for index in range(total):
            visited[index] = 0
        for y in range(self.height):
            port_rows[y] = 0
            port_cell[y] = -1
        head = 0
        tail = 1
        queue[0] = seed
        parent[seed] = -1
        visited[seed] = 1
        while head < tail:
            cell = queue[head]
            head += 1
            y = cell // self.width
            x = cell - y * self.width
            if port_start_y <= y <= port_end_y:
                if (side_id == RING_RIGHT and
                        x >= self.width - 1 - self.port_inset_margin_cells):
                    port_rows[y] = 1
                    current_port = port_cell[y]
                    if (current_port < 0 or
                            x > current_port % self.width):
                        port_cell[y] = cell
                elif (side_id == RING_LEFT and
                      x <= self.port_inset_margin_cells):
                    port_rows[y] = 1
                    current_port = port_cell[y]
                    if (current_port < 0 or
                            x < current_port % self.width):
                        port_cell[y] = cell
            if x > 0:
                tail = self._enqueue_host(
                    cell - 1, y, x - 1, side_id, cell, tail,
                    road, edge, known)
            if x + 1 < self.width:
                tail = self._enqueue_host(
                    cell + 1, y, x + 1, side_id, cell, tail,
                    road, edge, known)
            if y > self.floor_y:
                tail = self._enqueue_host(
                    cell - self.width, y - 1, x, side_id, cell, tail,
                    road, edge, known)
            if y + 1 < self.height:
                tail = self._enqueue_host(
                    cell + self.width, y + 1, x, side_id, cell, tail,
                    road, edge, known)
            if tail > total:
                return -1
        return tail

    def _enqueue_host(self, neighbor, y, x, side_id, parent_cell, tail,
                      road, edge, known):
        if (not known[neighbor] or not road[neighbor] or
                self._visited[neighbor]):
            return tail
        boundary_x = self._arc_x[y]
        if boundary_x >= 0:
            if side_id == RING_RIGHT and x > boundary_x - 2:
                return tail
            if side_id == RING_LEFT and x < boundary_x + 2:
                return tail
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                if abs(dx) + abs(dy) > 2:
                    continue
                check_x = x + dx
                check_y = y + dy
                if check_x < 0 or check_x >= self.width:
                    continue
                if check_y < self.floor_y:
                    return tail
                if check_y >= self.height:
                    continue
                check = check_y * self.width + check_x
                if not known[check] or edge[check]:
                    return tail
        self._visited[neighbor] = 1
        if tail >= _MAX_PIXELS:
            return _MAX_PIXELS + 1
        self._parent[neighbor] = parent_cell
        self._queue[tail] = neighbor
        return tail + 1

    def _build_path(self, port_cell, seed, road, known, side_id):
        for y in range(self.height):
            self._path_x_min[y] = self.width
            self._path_x_max[y] = -1
            self._path_valid_rows[y] = 0
        parent = self._parent
        cell = port_cell
        steps = 0
        while cell >= 0 and steps < _MAX_PIXELS:
            y = cell // self.width
            x = cell - y * self.width
            if y < self.floor_y or y >= self.height or not known[cell] or \
                    not road[cell]:
                return False
            if x < self._path_x_min[y]:
                self._path_x_min[y] = x
            if x > self._path_x_max[y]:
                self._path_x_max[y] = x
            self._path_valid_rows[y] = 1
            if cell == seed:
                break
            cell = parent[cell]
            steps += 1
        if cell != seed:
            return False
        count = 0
        for y in range(self.floor_y, self.height):
            if self._path_valid_rows[y]:
                center = (self._path_x_min[y] + self._path_x_max[y]) // 2
                index = y * self.width + center
                if (not self._visited[index] or not known[index] or
                        not road[index] or
                        not self._dark_clear(
                            center, y, self._current_edge_mask, known,
                            True, side_id)):
                    return False
                self._path_x_rows[y] = center
                count += 1
        self._candidate["path_row_count"] = count
        return count >= 1

    def _current_inner_evidence(self, side_id, road_result,
                                support_rows, support_count,
                                single_selection=None):
        """Classify current near-edge evidence without reusing old masks."""
        if not isinstance(support_rows, bytearray):
            support_rows = None
        count = 0
        sum_x = 0
        rows = road_result.get("selected_row_ints")
        row_count = road_result.get("selected_row_count", 0)
        if (isinstance(rows, array) and isinstance(row_count, int) and
                not isinstance(row_count, bool) and 0 < row_count <= _MAX_ROWS
                and road_result.get("selected_row_stride") == 4 and
                len(rows) >= row_count * 4 and support_rows is not None and
                support_count == row_count and len(support_rows) >= row_count):
            for index in range(row_count):
                base = index * 4
                y = int(rows[base])
                if (support_rows[index] != 1 or int(rows[base + 3]) & 7 != 7
                        or y < self.height - 12 or y >= self.height):
                    continue
                # Keep current inner-edge coordinates aligned with the exact
                # measured dark/road transition used by _extract_supported_rows.
                boundary_x = (int(rows[base + 2]) + 1
                              if side_id == RING_RIGHT else
                              int(rows[base + 1]) - 1)
                sum_x += boundary_x
                count += 1
            if count >= 3:
                current_x = sum_x / float(count)
                candidate = self._candidate
                candidate["inner_edge_rows"] = count
                candidate["near_inner_x"] = current_x
                if self._last_inner_x is not None:
                    delta = current_x - self._last_inner_x
                    receding = (delta > 16.0 if side_id == RING_RIGHT else
                                delta < -16.0)
                    near_clip = (current_x >= self.width - 4
                                 if side_id == RING_RIGHT else current_x <= 3)
                    candidate["inner_edge_receding"] = receding
                    candidate["inner_edge_near_clip"] = near_clip
                    candidate["inner_edge_disappeared"] = bool(
                        receding and near_clip)
                    if abs(delta) <= 16.0:
                        candidate["partial_inner_current"] = True
                        # Keep _last_inner_x anchored to the last full current
                        # arc. Updating it on every partial frame would erase
                        # gradual outward movement before exit can confirm it.
                        return "CURRENT", current_x
                    if candidate["inner_edge_disappeared"]:
                        return "DISAPPEARED", current_x

        # The accepted single-edge tracker exports only current rows with
        # row_status==1. It can preserve an existing side identity, but it
        # cannot establish that a missing edge has cleared.
        if isinstance(single_selection, dict) and single_selection.get("valid"):
            source = single_selection.get("source")
            expected = ("single_left" if side_id == RING_LEFT else
                        "single_right")
            result = single_selection.get("result") or {}
            count = result.get("selected_row_count", 0)
            rows = result.get("selected_row_ints")
            if (source == expected and isinstance(rows, array) and
                    isinstance(count, int) and 0 < count <= _MAX_ROWS and
                    result.get("selected_row_stride") == 5 and
                    len(rows) >= count * 5):
                near_rows = 0
                edge_sum = 0
                for index in range(count):
                    base = index * 5
                    if int(rows[base]) >= self.height - 12:
                        near_rows += 1
                        edge_sum += int(rows[base + 2])
                if near_rows >= 3:
                    current_x = edge_sum / (2.0 * near_rows)
                    self._candidate["inner_edge_rows"] = near_rows
                    self._candidate["near_inner_x"] = current_x
                    if (self._last_inner_x is not None and
                            abs(current_x - self._last_inner_x) <= 16.0):
                        self._candidate["partial_inner_current"] = True
                        return "CURRENT", current_x
        return "UNKNOWN", None

    def detect_ring_candidate(self, mask_info, road_result,
                              support_rows=None, support_count=0,
                              single_selection=None):
        """Extract a unique current-frame convex arc with a connected port."""
        candidate = self._reset_candidate("NO_CURRENT_BOUNDARIES")
        if not self._extract_supported_rows(
                road_result or {}, support_rows, support_count):
            if self._ring_side != RING_NONE:
                self._current_inner_evidence(
                    self._ring_side, road_result or {}, support_rows,
                    support_count, single_selection)
            candidate["reason"] = "NO_SUPPORTED_BILATERAL_ROWS"
            return candidate

        topology = (road_result or {}).get("topology_evidence") or {}
        if (topology.get("branch_evidence") or topology.get("branch") or
                topology.get("ambiguous")):
            if self._ring_side != RING_NONE:
                self._current_inner_evidence(
                    self._ring_side, road_result or {}, support_rows,
                    support_count, single_selection)
            candidate["reason"] = "TOPOLOGY_BRANCH_OR_AMBIGUITY"
            return candidate
        left = self._arc_candidate(RING_LEFT)
        right = self._arc_candidate(RING_RIGHT)
        if left is not None and right is not None:
            candidate["ambiguous"] = True
            candidate["reason"] = "BOTH_RING_SIDES_CANDIDATE"
            return candidate
        arc = left if left is not None else right
        if arc is None:
            if self._ring_side != RING_NONE:
                self._current_inner_evidence(
                    self._ring_side, road_result or {}, support_rows,
                    support_count, single_selection)
            candidate["reason"] = "NO_CONVEX_ARC"
            return candidate
        if (self._ring_side != RING_NONE and
                arc["side_id"] != self._ring_side):
            candidate["ambiguous"] = True
            candidate["reason"] = "RING_SIDE_IDENTITY_CHANGED"
            return candidate

        candidate["arc_detected"] = True

        port_ok, port_y, port_segments, port_reason = self._flood_port(
            mask_info, arc)
        candidate["side_id"] = arc["side_id"]
        candidate["side"] = _SIDE_TEXT[arc["side_id"]]
        candidate["arc_rows"] = arc["rows"]
        candidate["arc_y_min"] = arc["y_min"]
        candidate["arc_y_max"] = arc["y_max"]
        candidate["vertex_y"] = arc["vertex_y"]
        candidate["vertex_x"] = arc["vertex_x"]
        candidate["intrusion"] = arc["intrusion"]
        candidate["outer_outlier_excluded_y"] = arc.get(
            "outer_outlier_excluded_y")
        rows = self._rows
        inner_offset = 1 if arc["side_id"] == RING_LEFT else 2
        candidate["near_inner_x"] = rows[
            arc["end_index"] * _ROW_STRIDE + inner_offset]
        candidate["partial_inner_current"] = True
        candidate["inner_edge_rows"] = arc["rows"]
        self._last_inner_x = candidate["near_inner_x"]
        candidate["port_segments"] = port_segments
        if not port_ok:
            candidate["reason"] = port_reason
            return candidate

        near_center = float(candidate["near_seed_x"])
        port_cell = self._port_cell_by_row[port_y]
        port_x = port_cell - port_y * self.width
        clip_endpoint_x = 0 if arc["side_id"] == RING_LEFT else self.width - 1
        heading = _round_away(
            (port_x - near_center) * 2000.0 / float(self.width - 1))
        candidate["valid"] = True
        candidate["reason"] = "ARC_AND_CONNECTED_PORT"
        candidate["port_x"] = port_x
        candidate["port_y"] = port_y
        candidate["clip_endpoint_x"] = clip_endpoint_x
        candidate["near_center_x"] = near_center
        candidate["near_center_y"] = candidate["near_seed_y"]
        candidate["heading_error"] = _clamp(heading, -600, 600)
        candidate["boundary_source"] = (
            "RING_INNER_LEFT" if arc["side_id"] == RING_LEFT else
            "RING_INNER_RIGHT")
        candidate["planned_center_source"] = "CURRENT_CONNECTED_WHITE_PATH"
        return candidate

    def update_ring_state(self, fresh_keep, generation, normal_route_valid,
                          forward_continuation):
        """Advance only on consecutive captured frames in one KEEP session."""
        if generation != self._generation:
            self.clear("CONTROL_GENERATION")
            self._generation = generation
        candidate = self._candidate
        if candidate["ambiguous"]:
            self._clear_ring("RING_SIDE_AMBIGUOUS")
            self._ring_reason = "RING_SIDE_AMBIGUOUS"
            self._publish_ring_state()
            return self.ring

        if not fresh_keep and self._ring_state in (
                _STATE_ENTRY_LEFT, _STATE_ENTRY_RIGHT, _STATE_INSIDE_LEFT,
                _STATE_INSIDE_RIGHT, _STATE_EXIT, _STATE_COOLDOWN):
            self._clear_ring("KEEP_STALE_STATE_CLEARED")
            self._ring_reason = "KEEP_STALE_STATE_CLEARED"
            self._publish_ring_state()
            return self.ring

        candidate_side = candidate["side_id"] if candidate["valid"] else RING_NONE
        candidate_port_y = candidate["port_y"] if candidate["valid"] else -1
        if self._ring_state in (_STATE_IDLE, _STATE_PREVIEW):
            if candidate_side == RING_NONE:
                self._pending_side = RING_NONE
                self._pending_port_y = -1
                self._pending_count = 0
                self._ring_state = (
                    _STATE_PREVIEW if candidate.get("arc_detected") else
                    _STATE_IDLE)
                self._ring_active = False
                self._ring_reason = (
                    "ARC_WITHOUT_CONNECTED_PORT" if
                    candidate.get("arc_detected") else candidate["reason"])
            elif not fresh_keep or not normal_route_valid:
                self._pending_side = RING_NONE
                self._pending_port_y = -1
                self._pending_count = 0
                self._ring_state = _STATE_PREVIEW
                self._ring_active = False
                self._ring_reason = (
                    "PREVIEW_REQUIRES_FRESH_KEEP" if not fresh_keep else
                    "PREVIEW_REQUIRES_VALIDATED_CURRENT_ROUTE")
            else:
                same = (candidate_side == self._pending_side and
                        abs(candidate_port_y - self._pending_port_y) <= 2)
                self._pending_count = self._pending_count + 1 if same else 1
                self._pending_side = candidate_side
                self._pending_port_y = candidate_port_y
                self._ring_state = _STATE_PREVIEW
                self._ring_reason = "CONFIRMING_ARC_PORT"
                if self._pending_count >= 3:
                    self._ring_side = candidate_side
                    self._ring_state = (
                        _STATE_ENTRY_LEFT if candidate_side == RING_LEFT else
                        _STATE_ENTRY_RIGHT)
                    self._ring_active = True
                    self._ring_reason = "ENTRY_CONFIRMED"
        elif self._ring_state in (_STATE_ENTRY_LEFT, _STATE_ENTRY_RIGHT):
            if not normal_route_valid:
                self._clear_ring("CURRENT_BOUNDARY_LOST")
            elif candidate_side == self._ring_side and fresh_keep:
                self._ring_state = (
                    _STATE_INSIDE_LEFT if self._ring_side == RING_LEFT else
                    _STATE_INSIDE_RIGHT)
                self._ring_active = True
                self._ring_reason = "CURRENT_RING_ARC"
            elif (candidate.get("partial_inner_current") and fresh_keep):
                self._ring_state = (
                    _STATE_INSIDE_LEFT if self._ring_side == RING_LEFT else
                    _STATE_INSIDE_RIGHT)
                self._ring_active = True
                self._ring_reason = "CURRENT_PARTIAL_INNER_EDGE"
            else:
                self._clear_ring("RING_ENTRY_NOT_CURRENT")
        elif self._ring_state in (_STATE_INSIDE_LEFT, _STATE_INSIDE_RIGHT):
            same_arc = (candidate_side == self._ring_side and
                        candidate.get("valid"))
            if not normal_route_valid:
                self._clear_ring("CURRENT_BOUNDARY_LOST")
            elif not fresh_keep:
                self._ring_active = False
                self._ring_reason = "STALE_KEEP_DIAGNOSTIC_ONLY"
            elif same_arc:
                self._exit_count = 0
                self._ring_active = True
                self._ring_reason = "CURRENT_RING_ARC"
            elif candidate.get("partial_inner_current"):
                self._exit_count = 0
                self._ring_active = True
                self._ring_reason = "CURRENT_PARTIAL_INNER_EDGE"
            elif (candidate.get("inner_edge_disappeared") and
                  candidate.get("inner_edge_receding") and
                  candidate.get("inner_edge_near_clip") and
                  forward_continuation):
                self._ring_active = False
                self._exit_count += 1
                self._ring_reason = "EXIT_EDGE_GONE_FORWARD_CURRENT"
                if self._exit_count >= 3:
                    self._ring_state = _STATE_EXIT
                    self._ring_active = False
                    self._ring_reason = "EXIT_CONFIRMED_CURRENT_GEOMETRY"
            else:
                # Exit evidence must be consecutive current frames; an
                # unknown edge or missing forward continuation breaks it.
                self._exit_count = 0
                self._ring_active = False
                self._ring_reason = "INSIDE_EDGE_IDENTITY_UNKNOWN"
        elif self._ring_state == _STATE_EXIT:
            if normal_route_valid and fresh_keep and forward_continuation:
                self._ring_state = _STATE_COOLDOWN
                self._cooldown_count = 1
                self._ring_reason = "COOLDOWN_STABLE_ROUTE"
            else:
                self._clear_ring("EXIT_ROUTE_NOT_STABLE")
        elif self._ring_state == _STATE_COOLDOWN:
            if normal_route_valid and fresh_keep and forward_continuation:
                self._cooldown_count += 1
                self._ring_active = False
                self._ring_reason = "COOLDOWN_STABLE_ROUTE"
                if self._cooldown_count >= 3:
                    self._clear_ring("COOLDOWN_COMPLETE")
            elif not normal_route_valid:
                self._clear_ring("COOLDOWN_ROUTE_LOST")
            else:
                self._cooldown_count = 0
                self._ring_reason = "COOLDOWN_WAITING_STABLE_ROUTE"

        # PREVIEW is diagnostic even when the lease is stale. It never marks
        # a ring route active or supplies an inferred steering target.
        self._publish_ring_state()
        return self.ring

    def _fit_current_bilateral(self, road_result, support_rows,
                               support_count):
        self._fit_count = 0
        rows = road_result.get("selected_row_ints")
        count = road_result.get("selected_row_count", 0)
        stride = road_result.get("selected_row_stride", 0)
        if (not isinstance(rows, array) or not isinstance(count, int) or
                isinstance(count, bool) or count < 1 or count > _MAX_ROWS or
                stride != 4 or len(rows) < count * stride or
                not isinstance(support_rows, bytearray) or
                support_count != count or len(support_rows) < count):
            return False
        point_count = 0
        y_min = self.height
        y_max = -1
        for index in range(count):
            base = index * stride
            y = int(rows[base])
            if (support_rows[index] != 1 or int(rows[base + 3]) & 7 != 7 or
                    y < 30 or y > 50):
                continue
            left = int(rows[base + 1]) - 1
            right = int(rows[base + 2]) + 1
            if left < 0 or right >= self.width or right <= left:
                continue
            self._fit_y[point_count] = y
            self._fit_left[point_count] = left
            self._fit_right[point_count] = right
            if y < y_min:
                y_min = y
            if y > y_max:
                y_max = y
            point_count += 1
        self._fit_count = point_count
        if point_count < 10 or y_max - y_min < 12:
            return False
        return True

    def _line_fit(self, values):
        count = self._fit_count
        sum_y = 0
        sum_v = 0
        sum_yy = 0
        sum_yv = 0
        for index in range(count):
            y = self._fit_y[index]
            value = values[index]
            sum_y += y
            sum_v += value
            sum_yy += y * y
            sum_yv += y * value
        denominator = count * sum_yy - sum_y * sum_y
        if denominator <= 0:
            return None, None, None
        slope = (count * sum_yv - sum_y * sum_v) / float(denominator)
        intercept = (sum_v - slope * sum_y) / float(count)
        max_residual = 0.0
        for index in range(count):
            residual = abs(values[index] -
                           (slope * self._fit_y[index] + intercept))
            if residual > max_residual:
                max_residual = residual
        return slope, intercept, max_residual

    def _fit_slope(self):
        width_values = self._fit_width
        center_values = self._fit_center2
        count = self._fit_count
        for index in range(count):
            left = self._fit_left[index]
            right = self._fit_right[index]
            width_values[index] = right - left
            center_values[index] = left + right
        center_slope, center_intercept, center_residual2 = self._line_fit(
            center_values)
        width_slope, width_intercept, width_residual = self._line_fit(
            width_values)
        center_residual = (center_residual2 * 0.5
                           if center_residual2 is not None else None)
        center_slope = (center_slope * 0.5
                        if center_slope is not None else None)
        if width_slope is None or width_slope <= 0.0:
            zero_width_y = None
        else:
            zero_width_y = -width_intercept / width_slope
        valid = (
            center_slope is not None and width_slope is not None and
            abs(center_slope) <= 0.15 and center_residual is not None and
            center_residual <= 2.0 and width_slope >= 0.20 and
            width_residual is not None and width_residual <= 4.0 and
            zero_width_y is not None)
        profile = self.slope
        profile["fit_valid"] = bool(valid)
        profile["fit_rows"] = count
        y_min = self.height
        y_max = -1
        for index in range(count):
            y = self._fit_y[index]
            if y < y_min:
                y_min = y
            if y > y_max:
                y_max = y
        profile["fit_y_min"] = y_min
        profile["fit_y_max"] = y_max
        profile["center_slope"] = center_slope
        profile["center_residual_max"] = center_residual
        profile["width_slope"] = width_slope
        profile["width_residual_max"] = width_residual
        profile["zero_width_y"] = zero_width_y
        return valid, zero_width_y, width_slope

    def update_slope(self, road_result, support_rows, support_count,
                     current_route_valid, fresh_keep, ring_state,
                     topology_evidence=None):
        """Learn a stable flat baseline, then emit a bounded class hint."""
        profile = self.slope
        profile["hint"] = 0
        profile["fit_valid"] = False
        profile["fit_rows"] = 0
        profile["fit_y_min"] = None
        profile["fit_y_max"] = None
        profile["center_slope"] = None
        profile["center_residual_max"] = None
        profile["width_slope"] = None
        profile["width_residual_max"] = None
        profile["zero_width_y"] = None
        profile["baseline_zero_width_y"] = self._baseline_h0
        profile["baseline_width_slope"] = self._baseline_slope

        topology = topology_evidence or {}
        topology_clear = not (
            topology.get("branch_evidence") or topology.get("branch") or
            topology.get("ambiguous") or topology.get("island_evidence") or
            topology.get("island"))
        ring_clear = ring_state == _STATE_IDLE
        if not current_route_valid or not fresh_keep or not topology_clear or \
                not ring_clear:
            reason = ("CURRENT_ROUTE_INVALID" if not current_route_valid else
                      "KEEP_STALE" if not fresh_keep else
                      "TOPOLOGY_AMBIGUOUS" if not topology_clear else
                      "RING_STATE_ACTIVE")
            self._learn_count = 0
            self._baseline_update_count = 0
            self._clear_slope_candidate(reason)
            profile["learn_count"] = self._learn_count
            return profile

        if not self._fit_current_bilateral(
                road_result or {}, support_rows, support_count):
            self._learn_count = 0
            self._baseline_update_count = 0
            self._clear_slope_candidate("INSUFFICIENT_FIXED_BAND_ROWS")
            profile["learn_count"] = self._learn_count
            return profile
        fit_valid, h0, width_slope = self._fit_slope()
        if not fit_valid:
            self._learn_count = 0
            self._baseline_update_count = 0
            self._clear_slope_candidate("NOT_STRAIGHT_BILATERAL_PROFILE")
            profile["learn_count"] = self._learn_count
            return profile

        if self._baseline_h0 is None or self._baseline_slope is None:
            if self._learn_count and abs(
                    h0 - self._learn_h0[self._learn_count - 1] /
                    1000.0) >= 3.0:
                self._learn_count = 0
            if self._learn_count < 10:
                self._learn_h0[self._learn_count] = _round_away(h0 * 1000)
                self._learn_slope_milli[self._learn_count] = _round_away(
                    width_slope * 1000)
                self._learn_count += 1
            profile["learn_count"] = self._learn_count
            self._clear_slope_candidate("LEARNING_FLAT_BASELINE")
            if self._learn_count < 10:
                profile["stage"] = "LEARNING_BASELINE"
                return profile
            h_values = self._learn_h0
            if max(h_values) - min(h_values) >= 3000:
                self._learn_count = 0
                profile["learn_count"] = 0
                profile["stage"] = "NEED_STABLE_BASELINE"
                profile["reason"] = "BASELINE_NOT_STABLE"
                return profile
            slope_values = self._learn_slope_milli
            self._baseline_h0 = sum(h_values) / 10000.0
            self._baseline_slope = sum(slope_values) / 10000.0
            profile["baseline_zero_width_y"] = self._baseline_h0
            profile["baseline_width_slope"] = self._baseline_slope
            profile["stage"] = "BASELINE"
            profile["reason"] = "BASELINE_READY"
            return profile

        h0_delta = h0 - self._baseline_h0
        slope_ratio = width_slope / self._baseline_slope \
            if self._baseline_slope > 0.0 else 0.0
        candidate = h0_delta >= 6.0 and slope_ratio >= 1.35
        if candidate:
            self._baseline_update_count = 0
            self._slope_exit_count = 0
            self._slope_candidate_count += 1
            self._slope_reason = "SLOPE_CANDIDATE"
            if self._slope_candidate_count >= 3:
                self._slope_hint = 1
                self._slope_reason = "SLOPE_CONFIRMED"
        else:
            self._slope_candidate_count = 0
            if self._slope_hint and abs(h0_delta) <= 3.0:
                self._slope_exit_count += 1
                if self._slope_exit_count >= 3:
                    self._slope_hint = 0
                    self._slope_reason = "SLOPE_EXIT_CONFIRMED"
                else:
                    self._slope_reason = "SLOPE_EXIT_PENDING"
            else:
                self._slope_exit_count = 0
                self._slope_hint = 0
                self._slope_reason = "AT_OR_NEAR_BASELINE"

            if abs(h0_delta) <= 3.0 and not self._slope_hint:
                slot = self._baseline_update_count
                if slot < 10:
                    self._baseline_update_h0[slot] = _round_away(h0 * 1000)
                    self._baseline_update_slope[slot] = _round_away(
                        width_slope * 1000)
                    self._baseline_update_count += 1
                if self._baseline_update_count >= 10:
                    h_values = self._baseline_update_h0
                    if max(h_values) - min(h_values) < 3000:
                        self._baseline_h0 = sum(h_values) / 10000.0
                        self._baseline_slope = sum(
                            self._baseline_update_slope) / 10000.0
                    self._baseline_update_count = 0
            else:
                self._baseline_update_count = 0

        profile["baseline_zero_width_y"] = self._baseline_h0
        profile["baseline_width_slope"] = self._baseline_slope
        profile["candidate_count"] = self._slope_candidate_count
        profile["hint"] = self._slope_hint
        profile["reason"] = self._slope_reason
        profile["stage"] = "SLOPE" if self._slope_hint else (
            "CONFIRMING_SLOPE" if candidate else "BASELINE")
        profile["learn_count"] = self._learn_count
        return profile


__all__ = ("FastRingSlope", "RING_LEFT", "RING_NONE", "RING_RIGHT",
           "RUNTIME_API_VERSION")
