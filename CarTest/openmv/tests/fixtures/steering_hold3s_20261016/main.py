"""V7 fast grayscale detector and UART v1 publisher."""

import pyb
import sensor
import time
import gc
from array import array

import config
import vision_protocol

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
fast_ring_slope = None
contour_route = None
contour_trace = None


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
REVERSE_CONTROL_RUNTIME_API_VERSION = 1
RING_SLOPE_RUNTIME_API_VERSION = 2
CONTOUR_RUNTIME_API_VERSION = 1
CACHED_WIDTH_BUILD_ID = "CONTOUR_ARRAYFIX_20261016"
CLIPPED_MIDPOINT_BUILD_ID = "CLIPPED_MIDPOINT_RING_20261014"
SLOPE_CLASSIFIER_RUNTIME_ENABLED = False
SINGLEEDGE_ROUTE_BUILD_IDS = (
    ("cached_width", CACHED_WIDTH_BUILD_ID),
    ("clipped_midpoint", CLIPPED_MIDPOINT_BUILD_ID),
)
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
    global fast_ring_slope
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
        if fast_ring_slope is None:
            fast_ring_slope = __import__("fast_ring_slope")
    except ImportError as exc:
        raise ImportError(
            "V7 bundle mismatch: fast grayscale/path/topology/boundary/"
            "ring-slope module "
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
        if backend != "python-bounded":
            raise RuntimeError(
                "contour bundle mismatch: %s backend is %s, expected "
                "python-bounded" % (filename, backend))
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
    return _SINGLEEDGE_RUNTIME_MODULES


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


class _FastRowWeightWorkspace:
    """Preallocated same-frame row aggregation for the fast controller path.

    Rows are grouped at grid y=18 (full-image y=36). Each available group is
    normalized internally, so row density cannot change its configured share.
    Bilateral rows must carry the boundary validator's per-row pair bit; single
    rows arrive only from the tracker's accepted row_status==1 output.
    """

    _WEIGHT_SCALE = 1000000
    _PREVIEW_FLOOR_Y = 18
    _PREVIEW_REFERENCE_TOLERANCE = 4
    _PREVIEW_MIN_ROWS = 15
    _PREVIEW_MIN_SPAN = 12
    _PREVIEW_ENDPOINT_ROWS = 5

    def __init__(self):
        self._center_x2 = array("i", [0] * 60)
        self._group = bytearray(60)
        self._row_weight = array("i", [0] * 60)
        self._preview_y = array("i", range(60))
        self._preview_center_x2 = array("i", [0] * 60)
        self._preview_valid_rows = bytearray(60)
        self._preview_median_x2 = array("i", [0] * 5)
        self._preview_middle_y = array("i", [0] * 5)
        self._preview_middle_distance = array("i", [0] * 5)
        self.info = {
            "valid": False, "reason": "NOT_RUN", "source": "NONE",
            "center_x": None, "raw_error": 0,
            "near_count": 0, "far_count": 0,
            "near_weight": 0.0, "far_weight": 0.0,
            "missing_group": "NONE", "supported_rows": 0,
            "position_error": 0, "heading_error": 0,
            "heading_contribution": 0, "preview_candidate": 0,
            "steering_error": 0,
            "preview_rejection_reason": None,
            "reference_row_y": None, "reference_error": None,
            "preview_valid": False,
            "preview_near_x": None, "preview_near_y": None,
            "preview_far_x": None, "preview_far_y": None,
            "preview_curvature_cells": 0.0,
        }

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
        for index in range(len(self._row_weight)):
            self._row_weight[index] = 0
            self._preview_valid_rows[index] = 0
        return info

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

    def aggregate(self, selection, road_result, settings,
                  ring_candidate=None):
        info = self.clear()
        if not selection or not selection.get("valid"):
            info["reason"] = "SELECTION_INVALID"
            return info
        source = selection.get("source")
        ring_route = source in ("ring_left", "ring_right")
        if source == "road":
            result = road_result
            stride = 4
            support = result.get("boundary_support_rows")
            support_count = result.get("boundary_support_count", -1)
        elif source in ("single_left", "single_right"):
            result = selection.get("result")
            stride = 5
            support = None
            support_count = -1
        elif ring_route:
            result = road_result
            stride = 4
            support = None
            support_count = -1
            if (not ring_candidate or not ring_candidate.get("valid") or
                    not ring_candidate.get("route_authorized") or
                    not ring_candidate.get("current_geometry_ok") or
                    ring_candidate.get("path_row_count", 0) < 8 or
                    ring_candidate.get("path_y_span", 0) < 12 or
                    not ring_candidate.get("path_near_anchor") or
                    ring_candidate.get("port_segments") != 1 or
                    not isinstance(ring_candidate.get("path_x_rows"), array)
                    or not isinstance(ring_candidate.get(
                        "path_valid_rows"), bytearray)):
                info["reason"] = "CURRENT_RING_PATH_UNAVAILABLE"
                return info
        else:
            info["reason"] = "UNSUPPORTED_SOURCE"
            return info
        if not result:
            info["reason"] = "MISSING_ROW_RESULT"
            return info
        rows = result.get("selected_row_ints")
        count = result.get("selected_row_count")
        if ring_route:
            rows = ring_candidate["path_x_rows"]
            path_valid = ring_candidate["path_valid_rows"]
            count = settings.SURFACE_GRID_HEIGHT
            if len(rows) < count or len(path_valid) < count:
                info["reason"] = "INVALID_RING_PATH_ROWS"
                return info
        elif (not isinstance(rows, array) or isinstance(count, bool) or
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

        split_y = settings.STEERING_SPLIT_GRID_Y
        floor_y = settings.SURFACE_ROI_FLOOR_Y
        width = settings.SURFACE_GRID_WIDTH
        height = settings.SURFACE_GRID_HEIGHT
        centers = self._center_x2
        groups = self._group
        accepted_count = 0
        preview_count = 0
        preview_conflicting_centers = False
        reference_row_y = None
        reference_center_x2 = 0
        reference_conflicting_centers = False
        preview_centers = self._preview_center_x2
        preview_valid_rows = self._preview_valid_rows
        near_count = 0
        far_count = 0
        for index in range(count):
            if ring_route:
                y = index
                accepted = path_valid[index] == 1
                center_x2 = rows[index] * 2
            else:
                offset = index * stride
                y = rows[offset]
            if not ring_route and source == "road":
                flags = rows[offset + 3]
                accepted = (support[index] == 1 and flags & 7 == 7)
                center_x2 = rows[offset + 1] + rows[offset + 2]
            elif not ring_route:
                # FastSingleEdgeTracker packs only rows with row_status==1.
                accepted = True
                center_x2 = rows[offset + 1]
            if (not accepted or y < floor_y or y >= height or
                    center_x2 < 0 or center_x2 > (width - 1) * 2):
                continue
            centers[accepted_count] = center_x2
            if not ring_route:
                reference_distance = abs(y - 40)
                if reference_distance <= self._PREVIEW_REFERENCE_TOLERANCE:
                    previous_distance = (None if reference_row_y is None else
                                         abs(reference_row_y - 40))
                    if (previous_distance is None or
                            reference_distance < previous_distance):
                        reference_row_y = y
                        reference_center_x2 = center_x2
                        reference_conflicting_centers = False
                    elif reference_distance == previous_distance:
                        if reference_center_x2 != center_x2:
                            reference_conflicting_centers = True
            if not ring_route and y >= self._PREVIEW_FLOOR_Y:
                if preview_valid_rows[y]:
                    if preview_centers[y] != center_x2:
                        preview_conflicting_centers = True
                else:
                    preview_valid_rows[y] = 1
                    preview_centers[y] = center_x2
                    preview_count += 1
            if y < split_y:
                groups[accepted_count] = 1
                far_count += 1
            else:
                groups[accepted_count] = 2
                near_count += 1
            accepted_count += 1

        if accepted_count == 0:
            info["reason"] = "NO_SUPPORTED_ROWS"
            return info
        info["source"] = source
        info["near_count"] = near_count
        info["far_count"] = far_count
        info["supported_rows"] = accepted_count
        if near_count and far_count:
            near_weight = int(settings.STEERING_NEAR_WEIGHT *
                              self._WEIGHT_SCALE + 0.5)
            far_weight = self._WEIGHT_SCALE - near_weight
            info["near_weight"] = near_weight / float(self._WEIGHT_SCALE)
            info["far_weight"] = far_weight / float(self._WEIGHT_SCALE)
        elif near_count:
            near_weight = self._WEIGHT_SCALE
            far_weight = 0
            info["near_weight"] = 1.0
            info["far_weight"] = 0.0
            info["missing_group"] = "FAR"
        else:
            near_weight = 0
            far_weight = self._WEIGHT_SCALE
            info["near_weight"] = 0.0
            info["far_weight"] = 1.0
            info["missing_group"] = "NEAR"

        row_weights = self._row_weight
        near_seen = 0
        far_seen = 0
        for index in range(accepted_count):
            group = groups[index]
            group_count = near_count if group == 2 else far_count
            group_weight = near_weight if group == 2 else far_weight
            quotient = group_weight // group_count
            remainder = group_weight - quotient * group_count
            # Deterministic remainder distribution keeps each group's row
            # weights summing exactly to its configured effective share.
            if group == 2:
                seen = near_seen
                near_seen += 1
            else:
                seen = far_seen
                far_seen += 1
            row_weights[index] = quotient + (1 if seen < remainder else 0)

        weighted_x2 = 0
        for index in range(accepted_count):
            weighted_x2 += centers[index] * row_weights[index]
        center_x = weighted_x2 / float(2 * self._WEIGHT_SCALE)
        try:
            raw_error = _normalize_fast_reference(center_x, width)
        except (TypeError, ValueError, OverflowError):
            info["reason"] = "WEIGHTED_CENTER_INVALID"
            info["source"] = "NONE"
            info["center_x"] = None
            info["near_weight"] = 0.0
            info["far_weight"] = 0.0
            return info
        info["position_error"] = raw_error
        if ring_route:
            heading_error = ring_candidate.get("heading_error", 0)
            if (isinstance(heading_error, bool) or
                    not isinstance(heading_error, int) or
                    heading_error < -600 or heading_error > 600):
                info["reason"] = "RING_HEADING_INVALID"
                info["source"] = "NONE"
                return info
            info["heading_error"] = heading_error
            # Port heading remains diagnostic only. Ring steering uses the
            # current connected-path rows through the ordinary row weights.
            info["heading_contribution"] = 0
        elif raw_error < -1000 or raw_error > 1000:
            info["reason"] = "WEIGHTED_ERROR_OUT_OF_RANGE"
            info["source"] = "NONE"
            info["center_x"] = None
            info["near_weight"] = 0.0
            info["far_weight"] = 0.0
            return info
        if raw_error < -1000:
            raw_error = -1000
        elif raw_error > 1000:
            raw_error = 1000
        info["valid"] = True
        info["reason"] = "OK"
        info["center_x"] = center_x
        info["raw_error"] = raw_error
        if (reference_row_y is not None and
                not reference_conflicting_centers):
            info["reference_row_y"] = reference_row_y
            info["reference_error"] = _normalize_fast_reference(
                reference_center_x2 * 0.5, width)
        if ring_route:
            info["steering_error"] = raw_error
        else:
            self._compute_road_preview(
                preview_count, settings, preview_conflicting_centers)
            steering_error, rejection_reason, applied_contribution = \
                _compose_ordinary_steering_error(
                    info["position_error"], info["preview_candidate"])
            info["heading_contribution"] = applied_contribution
            info["preview_rejection_reason"] = rejection_reason
            info["steering_error"] = steering_error
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
    if (config.FRAME_SIZE_NAME != "QQVGA" or
            config.IMAGE_WIDTH != 160 or config.IMAGE_HEIGHT != 120 or
            config.SURFACE_GRID_WIDTH != 80 or
            config.SURFACE_GRID_HEIGHT != 60 or
            config.SURFACE_PIXEL_SCALE != 2):
        raise ValueError(
            "V6 requires QQVGA 160x120 and the fixed 80x60 scale-2 grid")
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
    for filename, module in _V7_RUNTIME_MODULES:
        version = getattr(module, "RUNTIME_API_VERSION", None)
        if version != V7_RUNTIME_API_VERSION:
            raise RuntimeError(
                "V7 bundle mismatch: %s API is %s, expected %d; install a "
                "matching V7 bundle" % (
                    filename, version, V7_RUNTIME_API_VERSION))
    if getattr(fast_ring_slope, "RUNTIME_API_VERSION", None) != \
            RING_SLOPE_RUNTIME_API_VERSION:
        raise RuntimeError(
            "ring/slope bundle mismatch: fast_ring_slope.py API version")
    if getattr(vision_protocol, "VERSION", None) != 1 or \
            getattr(vision_protocol, "FRAME_SIZE", None) != 12:
        raise RuntimeError("V7 requires the unchanged 12-byte UART v1 protocol")


def _validate_fast_config():
    _validate_v7_runtime_apis()
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
            or overlay_period < 1 or overlay_period > 100):
        raise ValueError("V7 OVERLAY_PERIOD_MS must be in 1..100 ms")
    if getattr(config, "SURFACE_MODE", None) != "gray_unified":
        raise ValueError("V7 requires the single gray_unified camera path")
    if (config.FRAME_SIZE_NAME != "QQVGA" or
            config.IMAGE_WIDTH != 160 or config.IMAGE_HEIGHT != 120 or
            config.SURFACE_GRID_WIDTH != 80 or
            config.SURFACE_GRID_HEIGHT != 60 or
            config.SURFACE_PIXEL_SCALE != 2):
        raise ValueError(
            "V7 requires QQVGA 160x120 and the fixed 80x60 scale-2 grid")
    expected_blocks = (
        (0, 8, 80, 20), (0, 20, 80, 32),
        (0, 32, 80, 46), (0, 46, 80, 60),
    )
    if (config.SURFACE_ROI_FLOOR_Y != 8 or
            config.ROAD_PARAMS.get("ROAD_FLOOR_Y") != 8 or
            tuple(config.GRAY_THRESHOLD_BLOCKS) != expected_blocks):
        raise ValueError(
            "turn-response ROI must use floor=8 and the four ordered blocks")
    if (getattr(config, "STEERING_SPLIT_IMAGE_Y", None) != 36 or
            getattr(config, "STEERING_SPLIT_GRID_Y", None) != 18 or
            config.STEERING_SPLIT_IMAGE_Y !=
            config.STEERING_SPLIT_GRID_Y * config.SURFACE_PIXEL_SCALE):
        raise ValueError(
            "turn-response split must remain full y=36 / grid y=18")
    if (getattr(config, "STEERING_FAR_WEIGHT", None) != 0.20 or
            getattr(config, "STEERING_NEAR_WEIGHT", None) != 0.80 or
            getattr(config, "TURN_RESPONSE_VERSION", None) !=
            "LOWER70_UPPER30_V1"):
        raise ValueError("turn-response weighting config/build mismatch")
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


def _surface_pixel(point):
    scale = config.SURFACE_PIXEL_SCALE
    return (int(point[0] * scale + scale * 0.5 + 0.5),
            int(point[1] * scale + scale * 0.5 + 0.5))


def _rounded_local(value):
    return int(value + 0.5) if value >= 0 else int(value - 0.5)


_PREVIEW_MAX_ROW_POINTS = 13


def _preview_row_sample_count(row_count):
    return row_count if row_count < _PREVIEW_MAX_ROW_POINTS else \
        _PREVIEW_MAX_ROW_POINTS


def _preview_row_sample_index(row_count, sample_index, sample_count):
    if sample_count <= 1:
        return 0
    return sample_index * (row_count - 1) // (sample_count - 1)


def _draw_sparse_surface_contour(img, sections):
    count = len(sections)
    if count < 2:
        return
    step = (count + 4) // 5
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


def _draw_sparse_road_contour(img, rows):
    count = len(rows)
    if count < 2:
        return
    step = (count + 4) // 5
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


def _normalize_fast_reference(x, width):
    """Normalize the exact controller point using the protocol's W-1 center."""
    half = (width - 1) * 0.5
    if half <= 0.0:
        raise ValueError("grid must contain at least two columns")
    normalized = ((float(x) - half) / half) * 1000.0
    return _rounded_local(normalized)


def _compose_ordinary_steering_error(position_error, preview_candidate):
    """Add ordinary position and preview errors, bounded by protocol limits."""
    steering_error = position_error + preview_candidate
    if steering_error < -1000:
        steering_error = -1000
    elif steering_error > 1000:
        steering_error = 1000
    return steering_error, None, preview_candidate


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

    if source == "road":
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


def _service_reverse_control(uart, parser, input_buffer, drain_state,
                             now_ms, ticks_diff):
    """Poll and read one bounded reverse-control chunk without allocations.

    On a heartbeat timeout or receive overflow, queued bytes are discarded in
    chunks of at most 32 until the UART queue is empty. This prevents an old
    queued KEEP from creating a fresh lease after a camera/control stall.
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
            return False

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
    if source == "road" or source in ("ring_left", "ring_right"):
        rows = road_result.get("selected_rows")
        reference_y = float(reference["y"])
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
        else:
            packed = road_result.get("selected_row_ints")
            count = road_result.get("selected_row_count", 0)
            stride = road_result.get("selected_row_stride")
            if (isinstance(packed, array) and stride == 4 and
                    isinstance(count, int) and not isinstance(count, bool) and
                    0 <= count <= 60 and len(packed) >= count * 4):
                for index in range(count):
                    offset = index * 4
                    if abs(float(packed[offset]) - reference_y) > 0.01:
                        continue
                    left_x = float(packed[offset + 1]) - 0.5
                    right_x = float(packed[offset + 2]) + 0.5
                    break
    elif source in ("single_left", "single_right"):
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
    ring_state = selection.get("ring_state", "IDLE")
    details0 = "BP15 %s %s" % (
        source.upper(), str(ring_state)[:10])
    gate_reason = selection.get("ring_candidate_reason")
    if not gate_reason:
        gate_reason = (selection.get("single_edge_reason") or
                       selection.get("reason", "NO_CANDIDATE"))
    gate_codes = {
        "PARALLEL_CURVE": "PAR",
        "OPPOSITE_EDGE_CURVED": "OUTER",
        "ARC_AND_CONNECTED_PORT": "PORT",
        "NO_FRESH_KEEP": "KEEP",
    }
    gate_code = gate_codes.get(gate_reason, str(gate_reason)[:5])
    fit_modes = {
        "STRAIGHT_OUTER": "LINE",
        "INDEPENDENT_CURVATURE": "INDEP",
        "REJECT_PARALLEL_CURVE": "PAR",
        "REJECT_CURVED_OUTER": "CURVE",
    }
    fit_mode = fit_modes.get(selection.get("ring_fit_mode"), "--")
    outer_residual = selection.get("ring_outer_fit_residual_max")
    inner_residual = selection.get("ring_inner_fit_residual_max")
    outer_text = "-" if outer_residual is None else "%.1f" % float(
        outer_residual)
    inner_text = "-" if inner_residual is None else "%.1f" % float(
        inner_residual)
    correlation = selection.get("ring_residual_correlation")
    correlation_text = ("--" if correlation is None else
                        "%+.1f" % float(correlation))
    details1 = "G%s/%s O%s I%s" % (
        gate_code, fit_mode, outer_text, inner_text)
    details2 = "P%d/%d A%d G%d C%s" % (
        selection.get("ring_path_row_count", 0),
        selection.get("ring_path_y_span", 0),
        1 if selection.get("ring_route_authorized") else 0,
        1 if selection.get("ring_candidate_reason") == "ARC_AND_CONNECTED_PORT"
        else 0, correlation_text)
    if valid and weight_info is not None and weight_info.get("valid"):
        position_error = weight_info.get("position_error", 0)
        heading_error = weight_info.get("heading_error", 0)
        contribution = weight_info.get("heading_contribution", 0)
        near_percent = int(weight_info.get("near_weight", 0.0) * 100 + 0.5)
        far_percent = int(weight_info.get("far_weight", 0.0) * 100 + 0.5)
        if source in ("ring_left", "ring_right"):
            details1 = "POS%+d RINGH%+d" % (
                position_error, heading_error)
            details2 = "CTRL%+d FILT%+d" % (raw_ctrl, filtered_error)
            details3 = "W%d/%d %s" % (
                far_percent, near_percent, gate_reason)
        else:
            details1 = "POS%+d A%+d CAN%+d" % (
                position_error, contribution,
                weight_info.get("preview_candidate", contribution))
            details2 = "CTRL%+d FILT%+d" % (raw_ctrl, filtered_error)
            if weight_info.get("preview_valid"):
                details3 = "N%.1f,%d F%.1f,%d" % (
                    weight_info["preview_near_x"],
                    weight_info["preview_near_y"],
                    weight_info["preview_far_x"],
                    weight_info["preview_far_y"])
            else:
                details3 = "PREVIEW NONE W%d/%d" % (
                    far_percent, near_percent)
    elif valid:
        details1 = "POS UNAVAILABLE"
        details2 = "CTRL%+d FILT%+d" % (raw_ctrl, filtered_error)
        details3 = "PREVIEW NONE"
    else:
        details0 = "BP15 INVALID"
        details1 = "GATE %s" % gate_reason
        details2 = "CTRL0 FILT0"
        details3 = "PREVIEW NONE"
    return (details0[:24], details1[:24], details2[:24], details3[:24])


def _draw_gray_result(img, selection, surface_result, road_result,
                      reference, valid, raw_ctrl, filtered_error,
                      processing_ms, fps, text_cache=None,
                      weight_info=None, contour_trace_info=None,
                      contour_used=False):
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
    route_mode = getattr(config, "SINGLE_EDGE_ROUTE_MODE", "unknown")
    route_label = ("CLIP" if route_mode == "clipped_midpoint" else
                   "CACHE" if route_mode == "cached_width" else
                   str(route_mode).upper())
    source_label = selection.get("source", "NONE").upper() if valid else "NONE"
    current_status = ("NOW V %s/%s" % (route_label, source_label)
                      if valid else "NOW INVALID %s" % route_label)
    img.draw_string((0, 0), current_status, color=255, scale=1)
    if text_cache is not None:
        for index in range(len(text_cache)):
            img.draw_string((0, 10 + index * 10), text_cache[index],
                            color=192, scale=1)


class _V7OverlayState:
    """Preallocated per-frame preview state; detail text changes every 100 ms."""

    def __init__(self, now_ms):
        self.last_text_ms = None
        self.text_cache = None
        self.last_status = None
        self.draw_us = 0
        self.flush_us = 0


def _render_v7_preview_frame(img, selection, surface_result, road_result,
                             reference, valid, raw_ctrl, filtered_error,
                             processing_ms, fps, overlay_state, now_ms,
                             flush_func=None, weight_info=None,
                             contour_trace_info=None, contour_used=False):
    """Draw every captured frame, refresh details at 100 ms, then flush it.

    This runs only after detection and the observation UART write. The flush
    time is kept separate from capture-to-send age and drawing time.
    """
    overlay_state.draw_us = 0
    overlay_state.flush_us = 0
    if img is None:
        return
    if config.OVERLAY_ENABLED:
        draw_start_us = time.ticks_us()
        current_status = (bool(valid),
                          selection.get("source", "NONE") if valid else "NONE")
        if (overlay_state.text_cache is None or
                overlay_state.last_text_ms is None or
                overlay_state.last_status != current_status or
                time.ticks_diff(now_ms, overlay_state.last_text_ms) >=
                config.OVERLAY_PERIOD_MS):
            overlay_state.text_cache = _build_gray_overlay_text_cache(
                selection, surface_result, road_result, reference, valid,
                raw_ctrl, filtered_error, processing_ms, fps, weight_info)
            overlay_state.last_text_ms = now_ms
            overlay_state.last_status = current_status
        _draw_gray_result(
            img, selection, surface_result, road_result, reference, valid,
            raw_ctrl, filtered_error, processing_ms, fps,
            overlay_state.text_cache, weight_info, contour_trace_info,
            contour_used)
        overlay_state.draw_us = time.ticks_diff(
            time.ticks_us(), draw_start_us)
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
    "pack_uart", "capture_to_send", "overlay", "preview_flush", "debug",
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
                "p95_method=1ms_bin_upper_overflow_uses_max stage_stats=%s" % (
                    run_id, window_index,
                    window_index * config.PERF_WINDOW_MS,
                    (window_index + 1) * config.PERF_WINDOW_MS,
                    self.frames, self.write_attempts, self.full_writes,
                    self.valid_frames, self.valid_full_writes,
                    self.late_invalid_frames, self.write_errors, attempted,
                    self.last_sequence, stages))

    def summary_lines(self, run_id, window_index, attempted):
        """Return short stdout lines so the SDK's bounded channel cannot drop them."""
        lines = [
            ("V7_PERF_WINDOW run=%d index=%d start_ms=%d end_ms=%d "
             "distinct_captures=%d write_attempts=%d full_uart_writes=%d "
             "valid_frames=%d valid_full_writes=%d late_invalid_frames=%d "
             "uart_write_errors=%d attempted_by_loop=%d last_sequence=%d "
             "p95_method=1ms_bin_upper_overflow_uses_max" % (
                 run_id, window_index,
                 window_index * config.PERF_WINDOW_MS,
                 (window_index + 1) * config.PERF_WINDOW_MS,
                 self.frames, self.write_attempts, self.full_writes,
                 self.valid_frames, self.valid_full_writes,
                 self.late_invalid_frames, self.write_errors, attempted,
                 self.last_sequence))]
        for stage in _V7_PERF_STAGES:
            lines.append("V7_PERF_STAGE run=%d index=%d %s" % (
                run_id, window_index, self._stage_summary(stage)))
        return lines


def _print_v7_summary(perf, run_id, window_index, attempted):
    """Stream bounded stdout records with a short USB consumer interval."""
    for line in perf.summary_lines(run_id, window_index, attempted):
        print(line)
        # The official OpenMV v5.0 stdio ring is 1024 bytes and resets when a
        # single message exceeds available space. 5 ms per short record lets
        # the IDE/SDK drain without hiding this cost from the active cycle.
        time.sleep_ms(5)


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
          "camera=160x120 grid=80x60 step_fraction=%.2f prefer=%s "
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


def _main_fast(duration_s=None, run_id=0):
    """Run one detector over every new frame and publish one full v1 packet."""
    _validate_fast_config()
    sensor_id = _configure_gray_sensor(sensor, fast=True)
    gray_builder = fast_gray.FastGrayMasks(config)
    path_detector = fast_path.FastPathDetector(config)
    boundary_validator = fast_boundary.FastBoundaryEvidence(config)
    single_edge_tracker = fast_single_edge.FastSingleEdgeTracker(config)
    ring_slope_tracker = fast_ring_slope.FastRingSlope(config)
    contour_recovery = (
        contour_route.ContourRouteRecovery(config)
        if config.CONTOUR_RECOVERY_ENABLED else None)
    reverse_parser = reverse_control.ReverseControlParser(
        reverse_control.DEFAULT_TIMEOUT_MS)
    sensor_get_exposure = getattr(sensor, "get_exposure_us", None)
    exposure_us = None
    if callable(sensor_get_exposure):
        try:
            exposure_us = sensor_get_exposure()
        except Exception:
            exposure_us = None
    sensor_get_gain = getattr(sensor, "get_gain_db", None)
    gain_db = None
    if callable(sensor_get_gain):
        try:
            gain_db = sensor_get_gain()
        except Exception:
            gain_db = None
    exposure_cap_us = getattr(config, "AUTO_EXPOSURE_MAX_US", None) \
        if config.AUTO_EXPOSURE else None
    if exposure_cap_us is not None:
        if exposure_us is None:
            raise RuntimeError(
                "V7 cannot verify the configured auto-exposure limit")
        if exposure_us > exposure_cap_us:
            raise RuntimeError(
                "V7 sensor exposure %dus exceeds %dus limit" %
                (exposure_us, exposure_cap_us))

    sensor_id_text = ("0x%04X" % (sensor_id & 0xFFFF)
                      if isinstance(sensor_id, int) else
                      "unavailable" if sensor_id is None else str(sensor_id))
    contour_backend = (
        getattr(contour_trace, "COMPILER_BACKEND", "UNKNOWN")
        if contour_recovery is not None else "disabled")
    print("V7_BUILD=%s backend=fast route=%s v7_api=%d single_api=%d "
          "ring_slope_api=%d ring_slope_backend=%s slope_classifier=disabled "
          "slope_hint=always0 contour_api=%d contour_backend=%s "
          "sensor_id=%s "
          "camera=160x120 grid=80x60 floor=%d budget=%dms "
          "detail_text=%dms preview=per_frame uart=3@115200/8N1 exposure_us=%s "
          "exposure_cap_us=%s gain_db=%s "
          "gray_backend=%s topology_backend=%s boundary_backend=%s "
          "single_backend=%s reverse=8byte/ccitt-false" % (
              config.BUILD_ID, config.SINGLE_EDGE_ROUTE_MODE,
              V7_RUNTIME_API_VERSION,
              SINGLEEDGE_RUNTIME_API_VERSION,
              RING_SLOPE_RUNTIME_API_VERSION,
              getattr(fast_ring_slope, "COMPILER_BACKEND", "UNKNOWN"),
              CONTOUR_RUNTIME_API_VERSION, contour_backend,
              sensor_id_text,
              config.SURFACE_ROI_FLOOR_Y, config.SURFACE_FRAME_BUDGET_MS,
              config.OVERLAY_PERIOD_MS,
              str(exposure_us), str(exposure_cap_us), str(gain_db),
              getattr(fast_gray, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_topology, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_boundary, "COMPILER_BACKEND", "UNKNOWN"),
              getattr(fast_single_edge, "COMPILER_BACKEND", "UNKNOWN")))

    uart = pyb.UART(config.UART_ID, config.UART_BAUDRATE,
                    bits=config.UART_BITS, parity=None,
                    stop=config.UART_STOP_BITS,
                    timeout=config.UART_TIMEOUT_MS,
                    timeout_char=config.UART_CHAR_TIMEOUT_MS)
    clock = time.clock()
    observation_filter = _FreshObservationFilter(
        config.OBSERVATION_FILTER_ALPHA,
        config.OBSERVATION_FILTER_MAX_GAP_MS,
        0)  # Quality is diagnostic only for this fast geometry path.
    row_weight_workspace = _FastRowWeightWorkspace()
    reverse_input_buffer = bytearray(reverse_control.MAX_FEED_BYTES)
    reverse_drain_state = array("i", [0])
    reverse_generation = reverse_parser.generation
    last_capture_end_ms = None
    sequence = 0
    tx_error_count = 0
    last_tx_result = "none"
    last_debug_ms = time.ticks_ms()
    overlay_state = _V7OverlayState(last_debug_ms)
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
    bench_start_us = time.ticks_us()
    duration_us = None if duration_s is None else int(duration_s) * 1000000
    current_window = 0
    window_us = config.PERF_WINDOW_MS * 1000
    window_start_us = bench_start_us
    perf = _V7PerfWindow()
    pending_cycle_start_us = None

    while (duration_us is None or
           time.ticks_diff(time.ticks_us(), bench_start_us) < duration_us):
        cycle_start_us = time.ticks_us()
        # The previous frame's complete cycle ends at this start timestamp.
        # Record it before this frame can move the active send window.
        if pending_cycle_start_us is not None:
            perf.record("total_cycle", time.ticks_diff(
                cycle_start_us, pending_cycle_start_us))
            pending_cycle_start_us = None
        clock.tick()

        capture_start_us = time.ticks_us()
        try:
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
                    ring_slope_tracker.clear("CAPTURE_GAP")
                    observation_filter.reset()
                    previous_source = None
                    previous_reference_kind = None
                    previous_reference_y = None
                    previous_reference_x = None
            raise
        capture_end_us = time.ticks_us()
        capture_us = time.ticks_diff(capture_end_us, capture_start_us)
        capture_end_ms = time.ticks_ms()
        capture_interrupted = False
        if last_capture_end_ms is not None:
            capture_gap_ms = time.ticks_diff(
                capture_end_ms, last_capture_end_ms)
            if capture_gap_ms < 0 or capture_gap_ms >= \
                    reverse_control.DEFAULT_TIMEOUT_MS:
                capture_interrupted = True
                reverse_parser.invalidate("CAPTURE_GAP")
                reverse_drain_state[0] = 1
        last_capture_end_ms = capture_end_ms

        reverse_fresh = _service_reverse_control(
            uart, reverse_parser, reverse_input_buffer, reverse_drain_state,
            capture_end_ms, time.ticks_diff)
        if reverse_parser.generation != reverse_generation:
            single_edge_tracker.clear(reverse_parser.last_event)
            ring_slope_tracker.clear(reverse_parser.last_event)
            observation_filter.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            reverse_generation = reverse_parser.generation
        if capture_interrupted:
            single_edge_tracker.clear("CAPTURE_GAP")
            ring_slope_tracker.clear("CAPTURE_GAP")
            observation_filter.reset()
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
        if not selection.get("valid"):
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

        bilateral_cache_candidate = _approved_bilateral_cache_candidate(
            selection, road_result, boundary_reference, mask_info)

        if reverse_fresh and reverse_parser.poll(
                time.ticks_ms(), time.ticks_diff):
            reverse_fresh = False
            reverse_drain_state[0] = 1
            single_edge_tracker.clear(reverse_parser.last_event)
            observation_filter.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
            ring_slope_tracker.clear(reverse_parser.last_event)
            if selection.get("source") in ("single_left", "single_right"):
                single_diagnostic_reason = "CONTROL_TIMEOUT"
                selection["valid"] = False
                selection["quality"] = 0
                selection["lateral_error"] = 0
                selection["reason"] = "CONTROL_TIMEOUT"
                selection["single_edge_reason"] = "CONTROL_TIMEOUT"
                selection["source"] = "NONE"
                selection["result"] = None
                reference = None

        ring_start_us = time.ticks_us()
        support_rows = road_result.get("boundary_support_rows")
        support_count = road_result.get("boundary_support_count", 0)
        ring_candidate = ring_slope_tracker.detect_ring_candidate(
            mask_info, road_result, support_rows, support_count,
            selection if selection.get("source") in
            ("single_left", "single_right") else None)
        forward_continuation = _fast_forward_white_continuation(
            mask_info, road_result, config)
        topology = road_result.get("topology_evidence") or {}
        topology_clear = not (
            topology.get("branch_evidence") or topology.get("branch") or
            topology.get("ambiguous"))
        normal_route_valid = bool(
            selection.get("valid") and selection.get("source") == "road" and
            boundary_reference.get("valid") and topology_clear)
        ring_info = ring_slope_tracker.update_ring_state(
            reverse_fresh, reverse_parser.generation,
            normal_route_valid, forward_continuation, capture_end_ms,
            time.ticks_diff)
        ring_candidate["route_authorized"] = bool(
            ring_info.get("route_authorized"))
        ring_candidate["current_geometry_ok"] = bool(
            ring_info.get("current_geometry_ok"))
        if (ring_candidate.get("valid") and
                ring_candidate.get("entry_geometry_ok") and
                ring_candidate.get("current_geometry_ok") and
                ring_candidate.get("path_row_count", 0) >= 8 and
                ring_candidate.get("path_y_span", 0) >= 12 and
                ring_candidate.get("path_near_anchor") and
                ring_candidate.get("port_segments") == 1 and
                ring_info.get("active") and
                ring_info.get("route_authorized") and
                ring_info.get("current_geometry_ok") and
                reverse_fresh and selection.get("valid") and
                selection.get("source") == "road" and
                boundary_reference.get("valid") and topology_clear and
                ring_candidate.get("side_id") ==
                ring_info.get("side_id")):
            selection["valid"] = True
            selection["source"] = (
                "ring_left" if ring_candidate.get("side_id") ==
                fast_ring_slope.RING_LEFT else "ring_right")
            selection["reason"] = "CURRENT_RING_ARC_AND_CONNECTED_PORT"
            selection["boundary_source"] = ring_candidate.get(
                "boundary_source", "RING_INNER_CURRENT")
            selection["inferred"] = True
            selection["ring_candidate"] = ring_candidate
            reference = {
                "x": float(ring_candidate["near_center_x"]),
                "y": float(ring_candidate["near_center_y"]),
                "kind": "RING_CURRENT_MEASURED_NEAR_CENTER",
            }
        selection["ring_state"] = ring_info.get("state", "IDLE")
        selection["ring_reason"] = ring_info.get("reason", "NOT_RUN")
        selection["ring_route_authorized"] = bool(
            ring_info.get("route_authorized"))
        selection["ring_candidate_reason"] = ring_candidate.get(
            "rejection_reason") or ring_candidate.get("reason", "NOT_RUN")
        selection["ring_outer_fit_rows"] = ring_candidate.get(
            "outer_fit_rows", 0)
        selection["ring_outer_fit_residual_max"] = ring_candidate.get(
            "outer_fit_residual_max")
        selection["ring_inner_fit_residual_max"] = ring_candidate.get(
            "inner_fit_residual_max")
        selection["ring_residual_correlation"] = ring_candidate.get(
            "inner_outer_residual_correlation")
        selection["ring_residual_difference_max"] = ring_candidate.get(
            "residual_difference_max")
        selection["ring_width_range"] = ring_candidate.get("width_range")
        selection["ring_fit_mode"] = ring_candidate.get("fit_mode")
        selection["ring_path_row_count"] = ring_candidate.get(
            "path_row_count", 0)
        selection["ring_path_y_span"] = ring_candidate.get(
            "path_y_span", 0)
        ring_detection_us = time.ticks_diff(
            time.ticks_us(), ring_start_us)

        row_weight_start_us = time.ticks_us()
        weight_info = row_weight_workspace.aggregate(
            selection, road_result, config,
            ring_candidate if selection.get("source") in
            ("ring_left", "ring_right") else None)
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
                    raw_error = weight_info["steering_error"]
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
                  abs(reference_y_now - previous_reference_y) >
                  config.GRAY_FILTER_REFERENCE_Y_RESET_CELLS):
                observation_filter.reset()
                filter_reset_reason = "REFERENCE_Y_SHIFT"
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
            ring_slope_tracker.clear("FRAME_BUDGET")
            observation_filter.reset()
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

        now_ms = time.ticks_ms()
        filtered_error = observation_filter.update(
            now_ms, raw_valid, quality if raw_valid else 0,
            raw_error if raw_valid else 0, time.ticks_diff)
        valid = raw_valid
        if not valid:
            raw_error = 0
            filtered_error = 0
        else:
            previous_source = selection.get("source")
            previous_reference_kind = reference["kind"]
            previous_reference_y = reference.get("y")
            previous_reference_x = reference.get("x")
        reference_end_us = time.ticks_us()

        pack_start_us = reference_end_us
        write_attempts = 0
        full_write = False
        write_error = None
        try:
            packet = vision_protocol.pack_frame(
                sequence, 1 if valid else 0,
                quality if valid else 0,
                filtered_error if valid else 0,
                0)
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
                ring_slope_tracker.clear("FRAME_BUDGET")
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
                packet = vision_protocol.pack_frame(sequence, 0, 0, 0, 0)
            if len(packet) != 12:
                raise ValueError("vision_protocol returned a non-12-byte frame")
            write_attempts = 1
            written = uart.write(packet)
            if written == len(packet):
                full_write = True
                last_tx_result = "FULL_12"
            else:
                write_error = "SHORT_WRITE:%s/12" % str(written)
                last_tx_result = write_error
        except Exception as exc:
            write_error = "%s:%s" % (type(exc).__name__, exc)
            last_tx_result = "ERROR"
        send_end_us = time.ticks_us()
        pack_uart_us = time.ticks_diff(send_end_us, pack_start_us)
        capture_to_send_us = time.ticks_diff(send_end_us, cycle_start_us)
        sequence_sent = sequence
        sequence = (sequence + 1) & 0xFFFF
        if write_error is not None:
            tx_error_count += 1
            reverse_parser.invalidate("OBSERVATION_UART_ERROR")
            reverse_generation = reverse_parser.generation
            reverse_drain_state[0] = 1
            single_edge_tracker.clear("OBSERVATION_UART_ERROR")
            ring_slope_tracker.clear("OBSERVATION_UART_ERROR")
            observation_filter.reset()
            previous_source = None
            previous_reference_kind = None
            previous_reference_y = None
            previous_reference_x = None
        elif full_write and valid:
            if (selection.get("source") == "road" and reverse_fresh and
                    bilateral_cache_candidate is not None):
                single_edge_tracker.observe_bilateral(
                    bilateral_cache_candidate, mask_info.get("luma"),
                    mask_info.get("edge_mask"), mask_info.get("road_mask"),
                    mask_info.get("known_region"), True, True)
            elif (single_candidate is not None and
                  single_candidate.get("valid")):
                single_edge_tracker.commit_single(single_candidate)

        window_start_us, windows_advanced = _advance_v7_perf_window(
            send_end_us, window_start_us, window_us)
        summary_log_us = 0
        while windows_advanced:
            summary_start_us = time.ticks_us()
            _print_v7_summary(perf, run_id, current_window, perf.frames)
            summary_log_us += time.ticks_diff(time.ticks_us(), summary_start_us)
            current_window += 1
            perf.clear()
            windows_advanced -= 1

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

        _render_v7_preview_frame(
            img, selection, no_surface_result, road_result, reference,
            valid, raw_error, filtered_error,
            capture_to_send_us // 1000, clock.fps(), overlay_state,
            time.ticks_ms(), weight_info=weight_info,
            contour_trace_info=(contour_trace_info
                                if contour_selected else None),
            contour_used=contour_selected)
        overlay_us = overlay_state.draw_us
        preview_flush_us = overlay_state.flush_us

        debug_us = 0
        debug_now_ms = time.ticks_ms()
        if config.DEBUG_ENABLED and time.ticks_diff(
                debug_now_ms, last_debug_ms) >= config.DEBUG_PERIOD_MS:
            debug_start_us = time.ticks_us()
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
                    selection, road_result, reference, valid)
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
                  "ring=%s:%s/%d gate=%s fit=%s ofit=%d/%.2f ifit=%.2f "
                  "corr=%s diff=%s width=%s path=%d/%d auth=%d geo=%d "
                  "port=%s,%s seed=%s,%s "
                  "slope=%s:%d "
                  "preview=%+d preview_candidate=%+d bend=%.2f "
                  "preview_block=- "
                  "contour=%d/%d:%s:%d/%d "
                  "rowref=%s reference_error=%s "
                  "boundary=%s:%d/%d ref=%s pos_weighted=%d raw_ctrl=%d filt=%d "
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
                      ring_info.get("state", "IDLE"),
                      ring_info.get("reason", "NOT_RUN"),
                      ring_info.get("pending_count", 0),
                       selection.get("ring_candidate_reason", "NOT_RUN"),
                      ring_candidate.get("fit_mode", "NONE"),
                      ring_candidate.get("outer_fit_rows", 0),
                      -1.0 if ring_candidate.get(
                          "outer_fit_residual_max") is None else
                          ring_candidate.get("outer_fit_residual_max"),
                      -1.0 if ring_candidate.get(
                          "inner_fit_residual_max") is None else
                          ring_candidate.get("inner_fit_residual_max"),
                      "-" if ring_candidate.get(
                          "inner_outer_residual_correlation") is None else
                          "%.2f" % ring_candidate.get(
                              "inner_outer_residual_correlation"),
                      "-" if ring_candidate.get(
                          "residual_difference_max") is None else
                          "%.2f" % ring_candidate.get(
                              "residual_difference_max"),
                      "-" if ring_candidate.get("width_range") is None else
                          str(ring_candidate.get("width_range")),
                       ring_candidate.get("path_row_count", 0),
                       ring_candidate.get("path_y_span", 0),
                       1 if ring_info.get("route_authorized") else 0,
                       1 if ring_info.get("current_geometry_ok") else 0,
                      str(ring_candidate.get("port_x"))[:5],
                      str(ring_candidate.get("port_y"))[:5],
                      str(ring_candidate.get("near_seed_x"))[:5],
                      str(ring_candidate.get("near_seed_y"))[:5],
                      "DISABLED", slope_hint,
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
                      filter_reset_reason,
                      capture_to_send_us // 1000, trusted_blocks,
                      str(threshold_min), str(threshold_max), last_tx_result,
                      tx_error_count, detector_error_count,
                      latest_detector_error, clock.fps()))
            debug_us = time.ticks_diff(time.ticks_us(), debug_start_us)
            last_debug_ms = debug_now_ms

        gc_us = 0
        frame_count += 1
        if frame_count % config.PERF_GC_PERIOD_FRAMES == 0:
            gc_start_us = time.ticks_us()
            gc.collect()
            gc_us = time.ticks_diff(time.ticks_us(), gc_start_us)

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
        perf.record("overlay", overlay_us)
        perf.record("preview_flush", preview_flush_us)
        perf.record("debug", debug_us)
        perf.record("summary_log", summary_log_us)
        perf.record("gc", gc_us)
        # Defer total_cycle until the next iteration's start so its duration
        # includes all stage recording and loop overhead. The final iteration
        # is closed at the run boundary below.
        pending_cycle_start_us = cycle_start_us

    if pending_cycle_start_us is not None:
        # No next frame start exists for a finite run; close this final cycle
        # after all per-stage records have completed.
        perf.record("total_cycle", time.ticks_diff(
            time.ticks_us(), pending_cycle_start_us))
    _print_v7_summary(perf, run_id, current_window, perf.frames)
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
