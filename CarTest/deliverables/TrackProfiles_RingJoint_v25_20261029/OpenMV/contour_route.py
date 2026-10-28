"""Conservative row-mid adapter for the standalone contour follower.

This module is a bounded experimental fallback. Its valid result is a
candidate that still needs the existing selected-boundary validator; it does
not authorize steering and never invents a calibrated normal offset.
"""

from contour_trace import ContourTraceWorkspace, WHITE, _array_fill

try:
    import micropython
    _IS_MICROPYTHON = True
except ImportError:  # Host tests retain ordinary CPython execution.
    _IS_MICROPYTHON = False

    class _HostMicropython:
        @staticmethod
        def native(function):
            return function

    micropython = _HostMicropython()


RUNTIME_API_VERSION = 1
COMPILER_BACKEND = ("micropython-native-viper" if _IS_MICROPYTHON else
                    "python-bounded")


_ELIGIBLE_PRIOR_REASONS = (
    "INSUFFICIENT_FRESH_ROWS",
    "NO_PAIRED_EDGES",
    "NO_VALID_PATH",
    "NO_CONNECTED_PATH",
)


def _setting(settings, name, default):
    if isinstance(settings, dict):
        if name in settings:
            return settings[name]
        road = settings.get("ROAD_PARAMS")
        if isinstance(road, dict) and name in road:
            return road[name]
        return default
    value = getattr(settings, name, None)
    if value is not None:
        return value
    road = getattr(settings, "ROAD_PARAMS", None)
    if isinstance(road, dict) and name in road:
        return road[name]
    return default


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _finite_number(value):
    return (isinstance(value, (int, float)) and
            not isinstance(value, bool) and value == value and
            value != float("inf") and value != -float("inf"))


def _invalid_topology(topology):
    if not isinstance(topology, dict):
        return False
    if (topology.get("ambiguous") or topology.get("branch_evidence") or
            topology.get("branch") or topology.get("island_evidence") or
            topology.get("island")):
        return True
    return topology.get("reason") in (
        "ISLAND", "OPEN_BAND", "AMBIGUOUS", "UNKNOWN_REGION",
        "TOPOLOGY_BRANCH_UNRESOLVED", "TOPOLOGY_ISLAND_UNRESOLVED",
    )


def _round_cell(value):
    if value >= 0.0:
        return int(value + 0.5)
    return -int(-value + 0.5)


class ContourRouteRecovery:
    """Trace-backed paired-edge candidate recovery after explicit path misses."""

    def __init__(self, settings):
        self.settings = settings
        width = _setting(settings, "SURFACE_GRID_WIDTH",
                         _setting(settings, "ROAD_GRID_WIDTH", 80))
        height = _setting(settings, "SURFACE_GRID_HEIGHT",
                          _setting(settings, "ROAD_GRID_HEIGHT", 60))
        if width != 80 or height != 60:
            raise ValueError("ContourRouteRecovery requires an 80x60 grid")
        self.width = width
        self.height = height
        self.floor_y = _setting(settings, "SURFACE_ROI_FLOOR_Y",
                                _setting(settings, "ROAD_FLOOR_Y", 0))
        if not _is_int(self.floor_y) or self.floor_y < 0 or self.floor_y >= height:
            raise ValueError("invalid contour route floor")

        self.min_width = _setting(settings, "ROAD_MIN_WIDTH", 8)
        self.max_width = _setting(settings, "ROAD_MAX_WIDTH", 70)
        self.max_gap = _setting(settings, "ROAD_MAX_GAP_ROWS", 2)
        max_slope = _setting(settings, "ROAD_MAX_CENTER_SLOPE", 2.5)
        link_tolerance = _setting(settings, "ROAD_LINK_TOLERANCE", 2.5)
        self.max_width_change = _setting(settings, "ROAD_MAX_WIDTH_CHANGE", 16)
        min_rows = _setting(
            settings, "ROAD_MIN_OBSERVED_ROWS",
            _setting(settings, "FAST_PATH_MIN_OBSERVED_ROWS", 12))
        min_span = _setting(
            settings, "ROAD_MIN_VERTICAL_SPAN",
            _setting(settings, "FAST_PATH_MIN_VERTICAL_SPAN", 12))
        if not _is_int(min_rows) or not _is_int(min_span):
            raise ValueError("contour route row limits must be integers")
        self.min_rows = max(12, min_rows)
        self.min_span = max(12, min_span)
        reference_y = _setting(settings, "ROAD_REFERENCE_Y", 40)
        reference_gap = _setting(
            settings, "FAST_PATH_REFERENCE_MAX_GAP",
            _setting(settings, "ROAD_SHARED_REFERENCE_MAX_GAP_CELLS", 4.0))
        if not all(_finite_number(value) for value in
                   (max_slope, link_tolerance, reference_y, reference_gap)):
            raise ValueError("contour route float limits must be finite numbers")
        if (max_slope < 0 or link_tolerance < 0 or reference_gap < 0 or
                reference_y < 0 or reference_y > height - 1):
            raise ValueError("invalid contour route float limits")
        self.max_slope = float(max_slope)
        self.link_tolerance = float(link_tolerance)
        self.reference_y = float(reference_y)
        self.reference_gap = float(reference_gap)
        if (not _is_int(self.min_width) or not _is_int(self.max_width) or
                self.min_width < 1 or self.max_width < self.min_width or
                not _is_int(self.max_gap) or self.max_gap < 0 or
                not _is_int(self.max_width_change) or
                self.max_width_change < 0 or self.min_rows < 12 or
                self.min_rows > height or not _is_int(self.min_span) or
                self.min_span < 12 or self.min_span > height):
            raise ValueError("invalid contour route geometry limits")

        # Keep the runtime candidate bounded independently of the standalone
        # workspace's larger host/reference default.
        self.workspace = ContourTraceWorkspace(
            width=width, height=height, max_points=256,
            iteration_budget=1024, record_events=False)
        self.trace_info = {
            "workspace": self.workspace,
            "used": False,
            "attempted": False,
            "reason": "NOT_RUN",
            "Lcount": 0,
            "Rcount": 0,
            "left_points": self.workspace.left_points,
            "right_points": self.workspace.right_points,
            "left_dark_support": self.workspace.left_dark_support,
            "right_dark_support": self.workspace.right_dark_support,
            "left_row_support": self.workspace.left_row_support,
            "right_row_support": self.workspace.right_row_support,
            "left_termination": self.workspace.left_termination,
            "right_termination": self.workspace.right_termination,
            "left_lost": 0,
            "right_lost": 0,
            "seed_status": self.workspace.seed_status,
            "candidate_rows": 0,
            "ambiguous_row_count": 0,
            "selected_row_count": 0,
            "seed_y": -1,
            "seed_left_x": -1,
            "seed_right_x": -1,
        }
        self.used = False
        self.reason = "NOT_RUN"
        self.Lcount = 0
        self.Rcount = 0
        self._left_x = _array_fill("h", -1, height)
        self._right_x = _array_fill("h", -1, height)
        self._left_seen = bytearray(height)
        self._right_seen = bytearray(height)
        self._row_ambiguous = bytearray(height)
        self._row_valid = bytearray(height)
        self._row_width = _array_fill("H", 0, height)
        self._row_center_twice = _array_fill("i", 0, height)
        self.selected_row_ints = _array_fill("i", 0, height * 4)
        self.selected_row_stride = 4
        self.selected_row_count = 0
        self._prior_result = None
        self._result = {
            "valid": False,
            "quality": 0,
            "lateral_error": 0,
            "reason": "NOT_RUN",
            "scene": "ROAD_CANDIDATE",
            "reference_y": None,
            "reference_target_y": self.reference_y,
            "reference_shift_cells": 0.0,
            "center_x": None,
            "reference_center_x": None,
            "reference_provenance": "NO_COMMON_REFERENCE",
            "reference_lateral_error": 0,
            "observed_rows": 0,
            "active_row_count": 0,
            "rows": (),
            "selected_rows": None,
            "selected_row_ints": self.selected_row_ints,
            "selected_row_count": 0,
            "selected_row_stride": 4,
            "left_edges": (),
            "right_edges": (),
            "corners": (),
            "interpolated": (),
            "width": width,
            "height": height,
            "weight_source": "CURRENT_BILATERAL_ROW_MID",
            "candidate_tracks": 0,
            "controller_reference": None,
            "topology_evidence": None,
            "boundary_evidence": None,
            "geometry_reason": None,
            "weights": (),
            "track_score": 0.0,
            "vertical_span": 0,
            "prior_reason": None,
            "boundary_source": "OBSERVED_BOTH",
            "inferred": False,
            "inference_flags": 0,
            "observed_both_rows": 0,
            "observed_left_rows": 0,
            "observed_right_rows": 0,
            "trace_info": self.trace_info,
        }

    def _reset(self, reason="NOT_RUN"):
        self.used = False
        self.reason = reason
        self.Lcount = 0
        self.Rcount = 0
        self.selected_row_count = 0
        info = self.trace_info
        info["used"] = False
        info["attempted"] = False
        info["reason"] = reason
        info["Lcount"] = 0
        info["Rcount"] = 0
        info["left_termination"] = "NOT_RUN"
        info["right_termination"] = "NOT_RUN"
        info["left_lost"] = 0
        info["right_lost"] = 0
        info["seed_status"] = "NOT_RUN"
        info["candidate_rows"] = 0
        info["ambiguous_row_count"] = 0
        info["selected_row_count"] = 0
        info["prior_reason"] = None
        info["topology_evidence"] = None
        info["seed_y"] = -1
        info["seed_left_x"] = -1
        info["seed_right_x"] = -1
        result = self._result
        result["valid"] = False
        result["quality"] = 0
        result["lateral_error"] = 0
        result["reason"] = reason
        result["reference_y"] = None
        result["reference_target_y"] = self.reference_y
        result["reference_shift_cells"] = 0.0
        result["center_x"] = None
        result["reference_center_x"] = None
        result["reference_provenance"] = "NO_COMMON_REFERENCE"
        result["reference_lateral_error"] = 0
        result["observed_rows"] = 0
        result["active_row_count"] = 0
        result["rows"] = ()
        result["selected_rows"] = None
        result["selected_row_count"] = 0
        result["candidate_tracks"] = 0
        result["controller_reference"] = None
        result["topology_evidence"] = None
        result["boundary_evidence"] = None
        result["geometry_reason"] = None
        result["vertical_span"] = 0
        result["prior_reason"] = None
        result["observed_both_rows"] = 0
        result["observed_left_rows"] = 0
        result["observed_right_rows"] = 0

    def _clear_candidate_workspaces(self):
        """Clear private row buffers immediately before collecting a trace.

        Early failures publish zero counts and never expose these buffers in
        the result. Deferring the linear clear keeps those failures bounded by
        their actual gates while every attempted edge collection still starts
        from the same all-zero workspace state.
        """
        for y in range(self.height):
            self._left_x[y] = -1
            self._right_x[y] = -1
            self._left_seen[y] = 0
            self._right_seen[y] = 0
            self._row_ambiguous[y] = 0
            self._row_valid[y] = 0
            self._row_width[y] = 0
            self._row_center_twice[y] = 0
        for index in range(self.height * 4):
            self.selected_row_ints[index] = 0

    def _fail(self, reason, prior_reason=None, topology=None):
        self.reason = reason
        self.trace_info["reason"] = reason
        self.trace_info["prior_reason"] = prior_reason
        self.trace_info["topology_evidence"] = topology
        return self._prior_result

    @micropython.native
    def _collect_trace_edges(self, road, known, floor):
        workspace = self.workspace
        for side in (0, 1):
            points = workspace.left_points if side == 0 else workspace.right_points
            support = (workspace.left_dark_support if side == 0 else
                       workspace.right_dark_support)
            count = workspace.left_count if side == 0 else workspace.right_count
            for point_index in range(count):
                if support[point_index] == 0:
                    continue
                offset = point_index * 2
                y = points[offset]
                x = points[offset + 1]
                if y < floor or y >= self.height or x <= 0 or x >= self.width - 1:
                    continue
                index = y * self.width + x
                if known[index] == 0 or road[index] != WHITE:
                    continue
                edge_x = x - 1 if side == 0 else x + 1
                edge_index = y * self.width + edge_x
                if known[edge_index] == 0 or road[edge_index] != 0:
                    continue
                if side == 0:
                    values = self._left_x
                    seen = self._left_seen
                else:
                    values = self._right_x
                    seen = self._right_seen
                if not seen[y]:
                    seen[y] = 1
                    values[y] = x
                elif values[y] != x:
                    self._row_ambiguous[y] = 1

        candidate_rows = 0
        ambiguous_rows = 0
        for y in range(floor, self.height):
            if self._row_ambiguous[y]:
                ambiguous_rows += 1
                continue
            if not self._left_seen[y] or not self._right_seen[y]:
                continue
            left = self._left_x[y]
            right = self._right_x[y]
            if left >= right:
                self._row_ambiguous[y] = 1
                ambiguous_rows += 1
                continue
            run_width = right - left + 1
            if run_width < self.min_width or run_width > self.max_width:
                continue
            all_white_known = True
            for x in range(left, right + 1):
                index = y * self.width + x
                if known[index] == 0 or road[index] != WHITE:
                    all_white_known = False
                    break
            if not all_white_known:
                continue
            self._row_valid[y] = 1
            self._row_width[y] = run_width
            self._row_center_twice[y] = left + right
            candidate_rows += 1
        self.trace_info["candidate_rows"] = candidate_rows
        self.trace_info["ambiguous_row_count"] = ambiguous_rows
        return candidate_rows

    def _make_result(self, count, bottom_y, reference_index, topology,
                     prior_reason):
        ints = self.selected_row_ints
        reference_offset = reference_index * 4
        reference_y = ints[reference_offset]
        left = ints[reference_offset + 1]
        right = ints[reference_offset + 2]
        reference_center = (left + right) * 0.5
        center_x = reference_center
        lateral_error = _round_cell(
            ((center_x - (self.width - 1) * 0.5) /
             ((self.width - 1) * 0.5)) * 1000.0)
        span = bottom_y - ints[(count - 1) * 4]
        quality = int(50 + count * 50 / max(1, self.height - self.floor_y))
        if quality > 100:
            quality = 100
        result = self._result
        result["valid"] = True
        result["quality"] = quality  # Diagnostic only; evidence gates validity.
        result["lateral_error"] = lateral_error
        result["reason"] = "OK_CONTOUR_BILATERAL"
        result["scene"] = "ROAD_CANDIDATE"
        result["reference_y"] = float(reference_y)
        result["reference_target_y"] = self.reference_y
        result["reference_shift_cells"] = float(reference_y) - self.reference_y
        result["center_x"] = center_x
        result["reference_center_x"] = reference_center
        result["reference_provenance"] = "current_contour_bilateral"
        result["reference_lateral_error"] = lateral_error
        result["observed_rows"] = count
        result["active_row_count"] = count
        result["rows"] = ()
        result["selected_rows"] = None
        result["selected_row_count"] = count
        result["selected_row_stride"] = 4
        result["topology_evidence"] = topology
        result["boundary_evidence"] = None
        result["weight_source"] = "CURRENT_BILATERAL_ROW_MID"
        result["candidate_tracks"] = 1
        result["controller_reference"] = {
            "x": reference_center,
            "y": float(reference_y),
            "kind": "current_contour_bilateral",
            "target_y": self.reference_y,
            "shift_cells": float(reference_y) - self.reference_y,
        }
        result["track_score"] = float(count)
        result["vertical_span"] = span
        result["prior_reason"] = prior_reason
        result["boundary_source"] = "OBSERVED_BOTH"
        result["inferred"] = False
        result["inference_flags"] = 0
        result["observed_both_rows"] = count
        result["observed_left_rows"] = count
        result["observed_right_rows"] = count
        self.trace_info["selected_row_count"] = count
        self.trace_info["reason"] = "OK_CONTOUR_BILATERAL"
        return result

    def detect(self, mask_info, prior_result=None):
        """Return the prior result unless a guarded contour candidate succeeds."""
        self._reset()
        self._prior_result = prior_result
        if isinstance(prior_result, dict) and prior_result.get("valid"):
            self.reason = "PRIOR_ALREADY_VALID"
            self.trace_info["reason"] = self.reason
            return prior_result
        if not isinstance(prior_result, dict):
            return self._fail("PRIOR_RESULT_REQUIRED")
        prior_reason = prior_result.get("reason")
        topology = prior_result.get("topology_evidence")
        if (prior_result.get("scene") in
                ("AMBIGUOUS_CANDIDATE", "AMBIGUOUS_TOPOLOGY") or
                _invalid_topology(topology)):
            return self._fail("TOPOLOGY_UNRESOLVED", prior_reason, topology)
        if not isinstance(topology, dict):
            return self._fail("TOPOLOGY_EVIDENCE_MISSING", prior_reason,
                              topology)
        if (topology.get("reason") not in ("OK", "PARTIAL_KNOWN_ROI") or
                topology.get("island_evidence") is not False or
                topology.get("branch_evidence") is not False or
                topology.get("ambiguous") is not False):
            return self._fail("TOPOLOGY_NOT_CLEAR", prior_reason, topology)
        if prior_reason not in _ELIGIBLE_PRIOR_REASONS:
            return self._fail("PRIOR_REASON_NOT_ELIGIBLE", prior_reason,
                              topology)
        if not isinstance(mask_info, dict):
            return self._fail("MASK_INFO_INVALID", prior_reason, topology)

        width = mask_info.get("width")
        height = mask_info.get("height")
        floor = mask_info.get("floor", self.floor_y)
        road = mask_info.get("road_mask")
        known = mask_info.get("known_region")
        if (not _is_int(width) or not _is_int(height) or
                width != self.width or height != self.height):
            return self._fail("FRAME_SIZE_MISMATCH", prior_reason, topology)
        if (not _is_int(floor) or floor != self.floor_y or
                not isinstance(road, bytearray) or
                not isinstance(known, bytearray) or
                len(road) != width * height or len(known) != width * height):
            return self._fail("MASK_INFO_INVALID", prior_reason, topology)
        if mask_info.get("valid", True) is not True:
            return self._fail(mask_info.get("reason") or "MASK_INVALID",
                              prior_reason, topology)
        self.trace_info["attempted"] = True

        if not self.workspace.find_seeds(
                mask_info, center_x=(width - 1) * 0.5,
                row=height - 2, search_rows=8,
                max_center_distance=0.0):
            self.trace_info["seed_status"] = self.workspace.seed_status
            self.trace_info["reason"] = "SEED_" + self.workspace.seed_status
            return self._fail("SEED_" + self.workspace.seed_status,
                              prior_reason, topology)
        seed_y = self.workspace.left_seed_y
        seed_left_x = self.workspace.left_seed_x
        seed_right_x = self.workspace.right_seed_x
        self.trace_info["seed_y"] = seed_y
        self.trace_info["seed_left_x"] = seed_left_x
        self.trace_info["seed_right_x"] = seed_right_x
        trace = self.workspace.trace(
            mask_info, (seed_y, seed_left_x), (seed_y, seed_right_x))
        self.Lcount = trace.left_count
        self.Rcount = trace.right_count
        info = self.trace_info
        info["Lcount"] = trace.left_count
        info["Rcount"] = trace.right_count
        info["left_termination"] = trace.left_termination
        info["right_termination"] = trace.right_termination
        info["left_lost"] = trace.left_lost
        info["right_lost"] = trace.right_lost
        info["seed_status"] = trace.seed_status
        if (trace.left_termination in ("BAD_SEED", "DIRECTION_LIMIT",
                                       "CAPACITY", "ITERATION_BUDGET",
                                       "STATE_TABLE_FULL") or
                trace.right_termination in ("BAD_SEED", "DIRECTION_LIMIT",
                                            "CAPACITY", "ITERATION_BUDGET",
                                            "STATE_TABLE_FULL")):
            return self._fail("TRACE_TERMINATION_UNSAFE", prior_reason,
                              topology)

        self._clear_candidate_workspaces()
        if self._collect_trace_edges(road, known, floor) <= 0:
            return self._fail("NO_PAIRED_EDGES", prior_reason, topology)

        bottom_y = -1
        for y in range(self.height - 1, max(floor - 1, self.height - 5), -1):
            if self._row_valid[y]:
                bottom_y = y
                break
        if bottom_y < 0:
            return self._fail("NO_NEAR_SEED_PAIR", prior_reason, topology)
        if abs(seed_y - bottom_y) > 4:
            return self._fail("NO_CONNECTED_PATH", prior_reason, topology)
        if (abs(self._left_x[bottom_y] - seed_left_x) > 4 or
                abs(self._right_x[bottom_y] - seed_right_x) > 4):
            return self._fail("NO_CONNECTED_PATH", prior_reason, topology)

        selected_count = 1
        base = 0
        left = self._left_x[bottom_y]
        right = self._right_x[bottom_y]
        self.selected_row_ints[base] = bottom_y
        self.selected_row_ints[base + 1] = left
        self.selected_row_ints[base + 2] = right
        self.selected_row_ints[base + 3] = 7
        previous_y = bottom_y
        previous_center_twice = left + right
        previous_width = self._row_width[bottom_y]
        for y in range(bottom_y - 1, floor - 1, -1):
            if not self._row_valid[y]:
                continue
            distance = previous_y - y
            if distance <= 0:
                continue
            if distance > self.max_gap + 1:
                break
            current_width = self._row_width[y]
            width_delta = abs(current_width - previous_width)
            if width_delta > self.max_width_change * distance:
                break
            center_delta_twice = abs(
                self._row_center_twice[y] - previous_center_twice)
            center_error_milli = center_delta_twice * 500
            if center_error_milli > (
                    self.max_slope * 1000.0 * distance +
                    self.link_tolerance * 1000.0):
                break
            offset = selected_count * 4
            self.selected_row_ints[offset] = y
            self.selected_row_ints[offset + 1] = self._left_x[y]
            self.selected_row_ints[offset + 2] = self._right_x[y]
            self.selected_row_ints[offset + 3] = 7
            selected_count += 1
            previous_y = y
            previous_center_twice = self._row_center_twice[y]
            previous_width = current_width
        self.selected_row_count = selected_count
        self.trace_info["selected_row_count"] = selected_count
        span = bottom_y - self.selected_row_ints[(selected_count - 1) * 4]
        if selected_count < self.min_rows:
            return self._fail("INSUFFICIENT_FRESH_ROWS", prior_reason,
                              topology)
        if span < self.min_span:
            return self._fail("INSUFFICIENT_VERTICAL_SPAN", prior_reason,
                              topology)

        reference_index = -1
        reference_distance = float("inf")
        for index in range(selected_count):
            y = self.selected_row_ints[index * 4]
            distance = abs(float(y) - self.reference_y)
            if distance < reference_distance:
                reference_distance = distance
                reference_index = index
        if (reference_index < 0 or
                reference_distance > self.reference_gap):
            return self._fail("REFERENCE_ROW_MISSING", prior_reason,
                              topology)

        result = self._make_result(selected_count, bottom_y,
                                   reference_index, topology, prior_reason)
        self.used = True
        self.reason = result["reason"]
        info["used"] = True
        info["reason"] = result["reason"]
        result["trace_info"] = info
        return result
