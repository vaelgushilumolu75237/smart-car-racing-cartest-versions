"""Field-tuning values for the CarTest OpenMV vision package.

Pixel limits and contrast thresholds are starting values. Calibrate them with
recorded OpenMV images on the fixed camera mount, actual floor, and lighting.
"""

UART_ID = 3
UART_BAUDRATE = 115200
UART_BITS = 8
UART_STOP_BITS = 1
UART_TIMEOUT_MS = 20
UART_CHAR_TIMEOUT_MS = 20
SEND_PERIOD_MS = 50
# V7 publishes once after every new frame; SEND_PERIOD_MS is retained only for
# the explicit historical V4 entry and is not a V7 send gate.
PERF_WINDOW_MS = 10000
PERF_WINDOW_HISTOGRAM_MS = 1
PERF_GC_PERIOD_FRAMES = 30
V7_UART_WRITE_RESERVE_US = 22000  # UART 20 ms timeout plus 2 ms guard.
DEBUG_ENABLED = True
# Keep grayscale frames unmodified for calibration by setting this False.
OVERLAY_ENABLED = True
OVERLAY_PERIOD_MS = 100  # Detailed text refresh only; drawing is per frame.
DEBUG_PERIOD_MS = 1000
# Optional diagnostic output only.
DEBUG_ROW_DETAILS = True
BUILD_ID = "CACHED_WIDTH_BEND_PREVIEW_20261015"
V7_RUNTIME_API_VERSION = 7
SINGLEEDGE_RUNTIME_API_VERSION = 2
SINGLE_EDGE_ROUTE_MODE = "cached_width"
LEGACY_BUILD_ID = "SIXTH_V4_BOUNDARY_GUARD_20261003"
VISION_BACKEND = "fast"  # Explicit diagnostics: "compare"; V4: "legacy".

FRAME_SIZE_NAME = "QQVGA"
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120
AUTO_EXPOSURE = True
AUTO_EXPOSURE_MAX_US = 10000  # V7 exposure cap; compare keeps legacy AEC.
EXPOSURE_US = 10000  # Used only when AUTO_EXPOSURE is False.
GAIN_DB = 12.0
MIRROR_X = False
FLIP_Y = False

# The V6 runtime uses one grayscale path for both blue practice borders and
# narrow black competition markings; no field mode/color switch is required.
SURFACE_MODE = "gray_unified"
SURFACE_MODES = ("gray_unified",)
SURFACE_GRID_WIDTH = 80
SURFACE_GRID_HEIGHT = 60
SURFACE_PIXEL_SCALE = 2
SURFACE_ROI_FLOOR_Y = 8  # Full-resolution y=16; preserves the upper turn band.
# Full-frame weighting split: grid y=18 maps to full-resolution y=36.
STEERING_SPLIT_IMAGE_Y = 36
STEERING_SPLIT_GRID_Y = 18
STEERING_FAR_WEIGHT = 0.20
STEERING_NEAR_WEIGHT = 0.80
TURN_RESPONSE_VERSION = "LOWER70_UPPER30_V1"
# Ring entry ports may be within a 5% viewport inset (4 cells on the 80-cell
# grid). This margin is only a crop-side connection tolerance; it is unrelated
# to the 80/20 steering weights and does not relax road/clearance checks.
RING_PORT_INSET_MARGIN_CELLS = 4

# Frame-local Otsu starting values. Blocks span the whole road width and stay
# inside the geometry ROI; each block's trust bit becomes a separate known mask.
GRAY_THRESHOLD_MIN_CONTRAST = 18.0
GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
GRAY_THRESHOLD_BLOCKS = (
    (0, 8, 80, 20),
    (0, 20, 80, 32),
    (0, 32, 80, 46),
    (0, 46, 80, 60),
)
GRAY_SENSOR_WARMUP_MS = 1000
GRAY_MINPOOL_SCALE = 2
GRAY_SOURCE_AGREEMENT_CELLS = 6.0
GRAY_SHARED_REFERENCE_MAX_GAP_CELLS = 4.0
# Legacy selector diagnostic only; V6 does not use this tiebreak.
GRAY_SELECTION_TIEBREAK = "surface"
# Applies only when both candidates share a y and pass the agreement gate.
# Without a common y the selector always uses road's own fixed/weighted point.
GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID = "road"
GRAY_FILTER_REFERENCE_Y_RESET_CELLS = 5.0
# A verified road/single-edge mode transition may retain EMA history only
# when it uses the same reference row and moves within this center radius.
SINGLE_EDGE_FILTER_REFERENCE_SHIFT_CELLS = 8.0
# Local same-frame luma step starting value; measure and retune on the mounted
# camera. This is not a global scene-brightness limit or a proven calibration.
GRAY_BOUNDARY_MIN_STEP = 10.0
# Starting ratio only; not calibrated against a mounted camera capture.
GRAY_BOUNDARY_MIN_STEP_FRACTION = 0.25
GRAY_BOUNDARY_SUPPORT_RATIO = 0.75
GRAY_BOUNDARY_TARGET_RADIUS_CELLS = 5.0
GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT = 1
GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT = 2

# Blue-white chroma starting values (0..255 channel-share units).  These are
# relative tests, not calibrated values from a mounted H7Plus capture.
BLUE_NORMALIZED_MIN = 150
BLUE_NORMALIZED_EXCESS_MIN = 60
BLUE_SATURATION_MIN = 18
COLOR_AWB_WARMUP_MS = 1200

# Gray mask compares each pixel with its eight neighbors.  The frame-range
# fraction follows exposure changes; no fixed scene cap is used.
GRAY_FRAME_RANGE_MIN = 12
GRAY_LOCAL_CONTRAST_MIN = 28
GRAY_LOCAL_CONTRAST_FRACTION = 0.25

# Eight depth bands allow the search to move upward when a 22 cm mount sees
# less than road width near y=104. Values are initial geometry settings.
SURFACE_SEED_Y_RATIOS = (0.90, 0.75, 0.60, 0.45, 0.35, 0.25, 0.20, 0.15)
SURFACE_SEED_X_OFFSETS = (-12, 0, 12)
SURFACE_SEED_HEADINGS = (-60, -45, -30, -15, 0, 15, 30, 45, 60)
SURFACE_SEED_MAX_CORRECTION_CELLS = 10
SURFACE_SEED_HEADING_PENALTY = 1.5
SURFACE_SEED_CLUSTER_CELLS = 5
SURFACE_SEED_AMBIGUITY_MARGIN = 18.0
SURFACE_AMBIGUITY_SEPARATION_CELLS = 8
SURFACE_SEED_TRIAL_DEDUPE_CELLS = 5
SURFACE_SEED_TRIALS_MAX = 6
SURFACE_INTERNAL_COMPONENT_MIN_AREA = 12
SURFACE_INTERNAL_COMPONENT_MIN_EXTENT = 3
SURFACE_INTERNAL_COMPONENT_MAX_EXTENT = 18
SURFACE_INTERNAL_COMPONENT_MAX_AREA = 180
SURFACE_INTERNAL_HOLE_MIN_AREA = 4
SURFACE_OPEN_BAND_MIN_ROWS = 3
SURFACE_OPEN_BAND_SUPPORT_ROWS = 5
SURFACE_OPEN_BAND_MAX_RUN_WIDTH = 36
SURFACE_OPEN_BAND_MIN_PAIR_GAP = 8
SURFACE_OPEN_BAND_WIDE_ROWS = 4
SURFACE_WIDTH_MIN_CELLS = 8
SURFACE_WIDTH_MAX_CELLS = 70
SURFACE_RAY_MAX_STEPS = 80

# Local path expansion: seven headings around the current tangent, with a
# maximum of 32 trace sections. Quality is evidence-based, not probability.
SURFACE_TRACE_STEP_CELLS = 3
SURFACE_TRACE_HEADING_OFFSETS = (-30, -20, -10, 0, 10, 20, 30)
SURFACE_TRACE_MAX_ABS_HEADING = 75
SURFACE_TRACE_MAX_CORRECTION_CELLS = 8
SURFACE_TRACE_MIN_PROGRESS_CELLS = 0.75
SURFACE_WIDTH_STEP_CHANGE_MAX = 14
SURFACE_PATH_AMBIGUITY_MARGIN = 18.0
SURFACE_BRANCH_SEPARATION_CELLS = 3.5
SURFACE_TRACE_MAX_SECTIONS = 32
SURFACE_MIN_FRESH_SECTIONS = 3
SURFACE_MIN_PATH_LENGTH_CELLS = 8
SURFACE_TARGET_LOOKAHEAD_CELLS = 10
SURFACE_TRACE_UNTIL_CELLS = 13
# Confidence/quality remains a protocol diagnostic; acceptance is based on
# freshness, geometry, boundary evidence, and ambiguity checks.
SURFACE_MIN_CONFIDENCE = 60
# Keep 50 ms margin below the unchanged H743 vision timeout of 250 ms.
SURFACE_FRAME_BUDGET_MS = 200

# Independent binary-road candidate. Values are bounded initial settings, not
# camera calibration. Caller-supplied speed must state ROAD_SPEED_UNIT.
ROAD_PARAMS = {
    "ROAD_GRID_WIDTH": 80,
    "ROAD_GRID_HEIGHT": 60,
    "ROAD_FLOOR_Y": 8,
    "ROAD_ROW_STEP": 2,
    "ROAD_SEED_ROWS": 16,
    "ROAD_SEED_MARGIN_X": 8,
    "ROAD_MIN_WIDTH": 8,
    "ROAD_MAX_WIDTH": 70,
    "ROAD_MIN_OBSERVED_ROWS": 4,
    "ROAD_MIN_VERTICAL_SPAN": 8,
    "ROAD_MAX_GAP_ROWS": 2,
    "ROAD_MAX_CENTER_SLOPE": 2.5,
    "ROAD_LINK_TOLERANCE": 2.5,
    "ROAD_MAX_WIDTH_CHANGE": 16,
    "ROAD_TRACK_BEAM": 12,
    "ROAD_BRANCH_OPTIONS": 2,
    "ROAD_BRANCH_COST_WINDOW": 10.0,
    "ROAD_AMBIGUITY_MARGIN": 8.0,
    "ROAD_AMBIGUITY_SEPARATION": 8.0,
    "ROAD_REFERENCE_Y": 40,
    "ROAD_SHARED_REFERENCE_MAX_GAP_CELLS": 4.0,
    "ROAD_STATIC_WEIGHTS": (1.0,) * 20,
    "ROAD_DYNAMIC_SPEED_UNIT": "m_s",
    "ROAD_DYNAMIC_SPEED_MIN": 0.0,
    "ROAD_DYNAMIC_SPEED_MAX": 3.0,
    "ROAD_DYNAMIC_SLOW_WEIGHTS": (
        1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6,
        0.5, 0.4, 0.3, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
    ),
    "ROAD_DYNAMIC_FAST_WEIGHTS": (
        0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.4, 0.5, 0.6,
        0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6,
    ),
    "ROAD_CORNER_SLOPE_CHANGE": 0.45,
    "ROAD_QUALITY_MIN": 60,
}

# Keep a fully observed bilateral path close to the configured steering row.
# fast_path's reusable default remains broad for historical test fixtures.
FAST_PATH_REFERENCE_MAX_GAP = 4.0

# Single-boundary steering is permitted only after this run has established
# per-row width/edge anchors from a fully accepted bilateral observation and
# the independent reverse-control stream is fresh. Values are grid cells.
SINGLE_EDGE_ANCHOR_RADIUS_CELLS = 10.0
SINGLE_EDGE_AMBIGUITY_MARGIN_CELLS = 2.0
SINGLE_EDGE_AMBIGUITY_SEPARATION_CELLS = 4.0
SINGLE_EDGE_MIN_ROWS = 8
SINGLE_EDGE_MIN_SPAN = 12
SINGLE_EDGE_REFERENCE_MAX_GAP = 4

# The hand-held practice surface is a bright corridor between dark-blue areas.
# Set this to "black_edges" when inspecting the competition black boundary.
DETECTION_MODE = "white_corridor"
TRACK_MODEL = "quadratic"  # V3 default; set "linear" for A/B or fallback.

# Select an explicit scan preset. Compact is a field trial, not a measured fix:
# it narrows the sampled y span and lowers geometric resolution. `near` is an
# optional lower-image trial for the curved-scene observations, never an
# automatic fallback. Its 3-pixel row pitch makes adjacent median bands
# non-overlapping. Both short presets cover less vertical geometry than wide.
ROI_PRESETS = {
    "wide": ((48, 56, 64, 72, 80, 88, 96, 104, 112), 88, 32),
    "compact": ((64, 68, 72, 76, 80, 84, 88, 92, 96), 80, 16),
    "near": ((92, 95, 98, 101, 104, 107, 110, 113, 116), 104, 16),
}
ROI_PRESET = "near"
if ROI_PRESET not in ROI_PRESETS:
    raise ValueError("ROI_PRESET must be 'wide', 'compact', or 'near'")

# Median-of-three horizontal profiles, centered at preset-specific rows.
PROFILE_BAND_RADIUS = 1
SCAN_ROWS, REFERENCE_Y, MIN_FIT_Y_SPAN_PX = ROI_PRESETS[ROI_PRESET]

# Same-frame road context for the boundary guard. Rows overlapping SCAN_ROWS
# are sampled once and reused. The contrast and threshold margin are initial
# software settings that still need mounted-camera calibration.
BOUNDARY_GUARD_ENABLED = True
SCENE_CONTEXT_ROWS = (48, 56, 64, 72, 80, 88, 96, 104, 112)
SCENE_MIN_CONTRAST = 36
# Keep Otsu dark mean plus this fraction of raw contrast as a ceiling; it
# prevents a small gray seam from lifting the scene threshold.
SCENE_DARK_REFERENCE_FRACTION = 0.25
SCENE_ROW_THRESHOLD_MARGIN = 10
SCENE_CONTEXT_MAX_ROWS = 9
SCENE_MIN_DARK_CLASS_PIXELS = 8
SCENE_MIN_LIGHT_CLASS_PIXELS = 24
SCENE_MIN_DARK_CLASS_RATIO_PERCENT = 1
SCENE_MIN_LIGHT_CLASS_RATIO_PERCENT = 5

# Check each accepted boundary independently so equal-and-opposite edge jumps
# cannot hide behind a stable center estimate.
EDGE_MAX_SLOPE_PX_PER_Y = 4.0
EDGE_JUMP_TOLERANCE_PX = 2.0

# Per-profile Otsu classification and geometry. Values require camera capture
# calibration; no fixed grayscale threshold is used by the adaptive detector.
MIN_DARK_CLASS_PIXELS = 2
MIN_LIGHT_CLASS_PIXELS = 16
MIN_CLASS_CONTRAST = 18
CONTRAST_FULL_SCORE_AT = 36
BLACK_EDGE_WIDTH_MIN_PX = 1
BLACK_EDGE_WIDTH_MAX_PX = 30
WHITE_CORRIDOR_DARK_MIN_PX = 2
TRACK_GAP_MIN_PX = 16
TRACK_GAP_MAX_PX = 154
MAX_INTERIOR_DARK_RATIO_PERCENT = 8
MAX_INTERIOR_DARK_RUN_PX = 1
MAX_DARK_RUNS_PER_ROW = 24

# Robust row fit and fresh-observation quality gate.
MIN_VALID_ROWS = 5
MAX_FIT_RESIDUAL_PX = 8
MAX_LOCAL_FIT_RESIDUAL_PX = 8
LOCAL_FIT_MAX_POINTS = 5
LOCAL_FIT_MIN_POINTS = 3
MIN_CONFIDENCE = 60

# Deterministic bounded quadratic RANSAC starting values. `t` is normalized
# from image rows so derivative and curvature limits remain interpretable.
QUADRATIC_T_SCALE_PX = 16.0
QUADRATIC_INLIER_RESIDUAL_PX = 4.0
QUADRATIC_MAX_SLOPE_PX_PER_Y = 2.5
QUADRATIC_MAX_CURVATURE_PX_PER_Y2 = 0.08
QUADRATIC_MODEL_CENTER_AMBIGUITY_PX = 8.0
QUADRATIC_TIE_MEDIAN_RESIDUAL_MARGIN_PX = 0.5
QUADRATIC_TIE_SUM_RESIDUAL_MARGIN_PX = 2.0
QUADRATIC_TIE_MEAN_RESIDUAL_MARGIN_PX = 0.5
QUADRATIC_MIN_CONSECUTIVE_SUPPORT_ROWS = 3

# Temporal filtering smooths only fresh valid observations.
OBSERVATION_FILTER_ALPHA = 0.70
# Connect only fresh valid frames within 200 ms; invalid frames still clear
# immediately. This is not the UART send period or the STM32 timeout.
OBSERVATION_FILTER_MAX_GAP_MS = 200

DETECTOR_PARAMS = {
    "detection_mode": DETECTION_MODE,
    "track_model": TRACK_MODEL,
    "min_dark_class_pixels": MIN_DARK_CLASS_PIXELS,
    "min_light_class_pixels": MIN_LIGHT_CLASS_PIXELS,
    "min_class_contrast": MIN_CLASS_CONTRAST,
    "contrast_full_score_at": CONTRAST_FULL_SCORE_AT,
    "black_edge_width_min_px": BLACK_EDGE_WIDTH_MIN_PX,
    "black_edge_width_max_px": BLACK_EDGE_WIDTH_MAX_PX,
    "white_corridor_dark_min_px": WHITE_CORRIDOR_DARK_MIN_PX,
    "track_gap_min_px": TRACK_GAP_MIN_PX,
    "track_gap_max_px": TRACK_GAP_MAX_PX,
    "max_interior_dark_ratio_percent": MAX_INTERIOR_DARK_RATIO_PERCENT,
    "max_interior_dark_run_px": MAX_INTERIOR_DARK_RUN_PX,
    "max_dark_runs_per_row": MAX_DARK_RUNS_PER_ROW,
    "profile_band_radius": PROFILE_BAND_RADIUS,
    "image_height": IMAGE_HEIGHT,
    "min_valid_rows": MIN_VALID_ROWS,
    "min_fit_y_span_px": MIN_FIT_Y_SPAN_PX,
    "max_fit_residual_px": MAX_FIT_RESIDUAL_PX,
    "max_local_fit_residual_px": MAX_LOCAL_FIT_RESIDUAL_PX,
    "quadratic_t_scale_px": QUADRATIC_T_SCALE_PX,
    "quadratic_inlier_residual_px": QUADRATIC_INLIER_RESIDUAL_PX,
    "quadratic_max_slope_px_per_y": QUADRATIC_MAX_SLOPE_PX_PER_Y,
    "quadratic_max_curvature_px_per_y2":
        QUADRATIC_MAX_CURVATURE_PX_PER_Y2,
    "quadratic_model_center_ambiguity_px":
        QUADRATIC_MODEL_CENTER_AMBIGUITY_PX,
    "quadratic_tie_median_residual_margin_px":
        QUADRATIC_TIE_MEDIAN_RESIDUAL_MARGIN_PX,
    "quadratic_tie_sum_residual_margin_px":
        QUADRATIC_TIE_SUM_RESIDUAL_MARGIN_PX,
    "quadratic_tie_mean_residual_margin_px":
        QUADRATIC_TIE_MEAN_RESIDUAL_MARGIN_PX,
    "quadratic_min_consecutive_support_rows":
        QUADRATIC_MIN_CONSECUTIVE_SUPPORT_ROWS,
    "scan_rows": SCAN_ROWS,
    "local_fit_max_points": LOCAL_FIT_MAX_POINTS,
    "local_fit_min_points": LOCAL_FIT_MIN_POINTS,
    "reference_y": REFERENCE_Y,
    "min_confidence": MIN_CONFIDENCE,
    "boundary_guard_enabled": BOUNDARY_GUARD_ENABLED,
    "scene_context_rows": SCENE_CONTEXT_ROWS,
    "scene_context_max_rows": SCENE_CONTEXT_MAX_ROWS,
    "scene_min_contrast": SCENE_MIN_CONTRAST,
    "scene_dark_reference_fraction": SCENE_DARK_REFERENCE_FRACTION,
    "scene_row_threshold_margin": SCENE_ROW_THRESHOLD_MARGIN,
    "scene_min_dark_class_pixels": SCENE_MIN_DARK_CLASS_PIXELS,
    "scene_min_light_class_pixels": SCENE_MIN_LIGHT_CLASS_PIXELS,
    "scene_min_dark_class_ratio_percent":
        SCENE_MIN_DARK_CLASS_RATIO_PERCENT,
    "scene_min_light_class_ratio_percent":
        SCENE_MIN_LIGHT_CLASS_RATIO_PERCENT,
    "edge_max_slope_px_per_y": EDGE_MAX_SLOPE_PX_PER_Y,
    "edge_jump_tolerance_px": EDGE_JUMP_TOLERANCE_PX,
}

# Frozen starting values for the earlier single-row detector regression tests.
# The runtime uses DETECTOR_PARAMS above; line_detector.py remains unchanged.
LEGACY_SCAN_ROWS = (60, 72, 84, 96, 108)
LEGACY_BLACK_THRESHOLD = 55
LEGACY_BOUNDARY_WIDTH_MIN_PX = 3
LEGACY_BOUNDARY_WIDTH_MAX_PX = 15
LEGACY_BOUNDARY_WIDTH_TOLERANCE_PX = 6
LEGACY_TRACK_GAP_MIN_PX = 50
LEGACY_TRACK_GAP_MAX_PX = 154
LEGACY_MAX_GAP_BLACK_RATIO_PERCENT = 1
LEGACY_MAX_GAP_BLACK_PIXELS = 1
LEGACY_MIN_VALID_ROWS = 3
LEGACY_MAX_CENTER_DEVIATION_PX = 18
LEGACY_MIN_CONFIDENCE = 60

LEGACY_DETECTOR_PARAMS = {
    "detection_mode": "black_edges",
    "black_threshold": LEGACY_BLACK_THRESHOLD,
    "boundary_width_min_px": LEGACY_BOUNDARY_WIDTH_MIN_PX,
    "boundary_width_max_px": LEGACY_BOUNDARY_WIDTH_MAX_PX,
    "boundary_width_tolerance_px": LEGACY_BOUNDARY_WIDTH_TOLERANCE_PX,
    "track_gap_min_px": LEGACY_TRACK_GAP_MIN_PX,
    "track_gap_max_px": LEGACY_TRACK_GAP_MAX_PX,
    "max_gap_black_ratio_percent": LEGACY_MAX_GAP_BLACK_RATIO_PERCENT,
    "max_gap_black_pixels": LEGACY_MAX_GAP_BLACK_PIXELS,
    "min_valid_rows": LEGACY_MIN_VALID_ROWS,
    "max_center_deviation_px": LEGACY_MAX_CENTER_DEVIATION_PX,
    "image_height": IMAGE_HEIGHT,
    "white_corridor_dark_min_px": 3,
}
