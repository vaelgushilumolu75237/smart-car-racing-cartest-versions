"""Bounded binary-road observer for OpenMV MicroPython.

The detector consumes an 80x60 mask where nonzero pixels are candidate road.
It only treats a run as an observed corridor when both transitions to the
non-road class are inside the image. This module deliberately does not turn
image borders or interpolated geometry into fresh edge evidence.
"""

RUNTIME_API_VERSION = 6


ROAD_DEFAULTS = {
    "ROAD_GRID_WIDTH": 80,
    "ROAD_GRID_HEIGHT": 60,
    "ROAD_FLOOR_Y": 20,
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
    "ROAD_STATIC_WEIGHTS": (1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                            1.0, 1.0),
    "ROAD_DYNAMIC_SPEED_UNIT": "m_s",
    "ROAD_DYNAMIC_SPEED_MIN": 0.0,
    "ROAD_DYNAMIC_SPEED_MAX": 3.0,
    "ROAD_DYNAMIC_SLOW_WEIGHTS": (1.5, 1.4, 1.3, 1.2, 1.1, 1.0,
                                  0.9, 0.8, 0.7, 0.6, 0.5, 0.4,
                                  0.3, 0.2, 0.2, 0.2, 0.2, 0.2,
                                  0.2, 0.2),
    "ROAD_DYNAMIC_FAST_WEIGHTS": (0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
                                  0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
                                  0.9, 1.0, 1.1, 1.2, 1.3, 1.4,
                                  1.5, 1.6),
    "ROAD_CORNER_SLOPE_CHANGE": 0.45,
    "ROAD_QUALITY_MIN": 60,
}


def _value(settings, key, default=None):
    if settings is None:
        return ROAD_DEFAULTS.get(key, default)
    if isinstance(settings, dict):
        return settings.get(key, ROAD_DEFAULTS.get(key, default))
    return getattr(settings, key, ROAD_DEFAULTS.get(key, default))


def _positive_int(settings, key, default, minimum=1):
    value = _value(settings, key, default)
    try:
        value = int(value)
    except (TypeError, ValueError):
        return default
    return value if value >= minimum else default


def _row_candidates(mask, valid_region, width, y, min_width, max_width,
                    margin_x):
    """Return paired interior-white runs and row diagnostics."""
    runs = []
    partial = []
    x = 0
    row_offset = y * width
    has_unknown = False
    while x < width:
        index = row_offset + x
        if valid_region is not None and not valid_region[index]:
            has_unknown = True
            x += 1
            continue
        if not mask[index]:
            x += 1
            continue
        start = x
        x += 1
        while x < width:
            index = row_offset + x
            if valid_region is not None and not valid_region[index]:
                has_unknown = True
                break
            if not mask[index]:
                break
            x += 1
        end = x - 1
        left_known = (start > 0 and
                      (valid_region is None or
                       valid_region[row_offset + start - 1]))
        right_known = (end < width - 1 and
                       (valid_region is None or
                        valid_region[row_offset + end + 1]))
        if start > 0 and not left_known:
            has_unknown = True
        if end < width - 1 and not right_known:
            has_unknown = True
        left_observed = (start > 0 and left_known and
                         not mask[row_offset + start - 1])
        right_observed = (end < width - 1 and right_known and
                          not mask[row_offset + end + 1])
        run_width = end - start + 1
        candidate = {
            "y": y,
            "left": start,
            "right": end,
            "width": run_width,
            "center": (start + end) * 0.5,
            "left_status": "OBSERVED" if left_observed else "MISSING",
            "right_status": "OBSERVED" if right_observed else "MISSING",
            "provenance": "OBSERVED" if left_observed and right_observed
                          else "UNKNOWN_REGION" if has_unknown else "PARTIAL",
        }
        if left_observed and right_observed:
            if (run_width >= min_width and run_width <= max_width and
                    candidate["center"] >= margin_x and
                    candidate["center"] <= width - 1 - margin_x):
                runs.append(candidate)
            else:
                candidate["provenance"] = "OUT_OF_RANGE"
                partial.append(candidate)
        else:
            partial.append(candidate)
    return runs, partial, has_unknown


def _candidate_cost(previous, last, candidate, max_slope,
                    link_tolerance, max_width_change):
    dy = last["y"] - candidate["y"]
    if dy <= 0:
        return None
    if abs(candidate["width"] - last["width"]) > max_width_change:
        return None
    predicted = last["center"]
    if previous is not None:
        # Keep the signed image-y direction in both the slope denominator
        # and the extrapolation delta. Rows farther up the image have lower
        # y, so a rightward path has a negative x-per-y slope.
        base_dy = last["y"] - previous["y"]
        if base_dy != 0:
            slope = ((last["center"] - previous["center"]) /
                     float(base_dy))
            predicted += slope * (candidate["y"] - last["y"])
    maximum_delta = max_slope * dy + link_tolerance
    center_error = abs(candidate["center"] - predicted)
    if center_error > maximum_delta:
        return None
    width_error = abs(candidate["width"] - last["width"])
    return center_error + 0.35 * width_error


def _copy_track(track):
    return {
        "points": track["points"][:],
        "gap": track["gap"],
        "cost": track["cost"],
        "seed_y": track["seed_y"],
    }


def _track_score(track):
    points = track["points"]
    count = len(points)
    if count == 0:
        return -100000.0
    span = points[0]["y"] - points[-1]["y"]
    links = max(1, count - 1)
    curvature_cost = 0.0
    width_cost = 0.0
    if count >= 3:
        for index in range(2, count):
            a, b, c = points[index - 2:index + 1]
            dy0 = max(1, a["y"] - b["y"])
            dy1 = max(1, b["y"] - c["y"])
            slope0 = (b["center"] - a["center"]) / dy0
            slope1 = (c["center"] - b["center"]) / dy1
            curvature_cost += abs(slope1 - slope0)
    for index in range(1, count):
        width_cost += abs(points[index]["width"] -
                          points[index - 1]["width"])
    return (count * 10.0 + span * 0.55 - track["cost"] * 0.55 -
            curvature_cost * 1.25 - width_cost * 0.16 - track["gap"] * 2.0)


def _make_tracks(rows, row_candidates, seed_rows, settings):
    max_slope = float(_value(settings, "ROAD_MAX_CENTER_SLOPE", 2.5))
    tolerance = float(_value(settings, "ROAD_LINK_TOLERANCE", 2.5))
    max_width_change = _positive_int(settings, "ROAD_MAX_WIDTH_CHANGE", 16)
    max_gap = _positive_int(settings, "ROAD_MAX_GAP_ROWS", 2, 0)
    beam = _positive_int(settings, "ROAD_TRACK_BEAM", 48)
    branch_options = _positive_int(settings, "ROAD_BRANCH_OPTIONS", 3)
    branch_window = float(_value(settings, "ROAD_BRANCH_COST_WINDOW", 10.0))
    tracks = []

    for seed_index in range(min(seed_rows, len(rows))):
        for seed in row_candidates[seed_index]:
            active = [{"points": [seed], "gap": 0, "cost": 0.0,
                       "seed_y": seed["y"]}]
            completed = []
            for row_index in range(seed_index + 1, len(rows)):
                options = row_candidates[row_index]
                expanded = []
                for track in active:
                    points = track["points"]
                    previous = points[-2] if len(points) > 1 else None
                    last = points[-1]
                    links = []
                    for candidate in options:
                        cost = _candidate_cost(
                            previous, last, candidate, max_slope,
                            tolerance, max_width_change)
                        if cost is not None:
                            links.append((cost, candidate))
                    links.sort(key=lambda item: item[0])
                    if links:
                        cheapest = links[0][0]
                        accepted = 0
                        for cost, candidate in links:
                            if cost > cheapest + branch_window:
                                break
                            next_track = _copy_track(track)
                            next_track["points"].append(candidate)
                            next_track["gap"] = 0
                            next_track["cost"] += cost
                            expanded.append(next_track)
                            accepted += 1
                            if accepted >= branch_options:
                                break
                    elif track["gap"] < max_gap:
                        skipped = _copy_track(track)
                        skipped["gap"] += 1
                        skipped["cost"] += 1.5
                        expanded.append(skipped)
                    else:
                        completed.append(track)
                expanded.sort(key=_track_score, reverse=True)
                active = expanded[:beam]
                if len(completed) > beam:
                    completed.sort(key=_track_score, reverse=True)
                    completed = completed[:beam]
                if not active:
                    break
            # A track that ended on an unknown, clipped, or unsupported row
            # is still a fresh same-frame trace. Preserve it for the caller's
            # existing row-count, span, quality, and ambiguity gates.
            tracks.extend(completed)
            tracks.extend(active)

    # Seeded at adjacent rows, the same geometric corridor produces duplicate
    # paths. Deduplicate by target center and observed extent, keeping stronger.
    tracks.sort(key=_track_score, reverse=True)
    unique = []
    target_y = int(_value(settings, "ROAD_REFERENCE_Y", 40))
    for track in tracks:
        points = track["points"]
        if len(points) < 2:
            continue
        nearest = min(points, key=lambda point: abs(point["y"] - target_y))
        duplicate = False
        for retained in unique:
            retained_points = retained["points"]
            retained_nearest = min(
                retained_points,
                key=lambda point: abs(point["y"] - target_y))
            if (abs(nearest["center"] - retained_nearest["center"]) < 4.0 and
                    abs(points[0]["center"] -
                        retained_points[0]["center"]) < 5.0 and
                    abs(points[-1]["center"] -
                        retained_points[-1]["center"]) < 5.0):
                duplicate = True
                break
        if not duplicate:
            unique.append(track)
        if len(unique) >= beam:
            break
    return unique


def _static_weights(settings, count):
    configured = _value(settings, "ROAD_STATIC_WEIGHTS", ())
    weights = []
    for index in range(count):
        try:
            value = float(configured[index])
        except (IndexError, TypeError, ValueError):
            value = 1.0
        weights.append(value if value > 0.0 else 0.0)
    if sum(weights) <= 0.0:
        return [1.0] * count
    return weights


def weighted_observation(points, settings=None, speed_value=None,
                         speed_unit=None):
    """Return weighted center and weight-source metadata for fresh rows only.

    `speed_unit` must match the configured unit before dynamic weights are
    considered. Any missing/invalid speed falls back to static fresh-row weights.
    """
    ordered = sorted(points, key=lambda point: point["y"], reverse=True)
    if not ordered:
        return {"center_x": None, "weight_source": "NONE", "weights": []}

    count = len(ordered)
    static = _static_weights(settings, count)
    weights = static
    source = "STATIC"
    minimum = _value(settings, "ROAD_DYNAMIC_SPEED_MIN", 0.0)
    maximum = _value(settings, "ROAD_DYNAMIC_SPEED_MAX", 0.0)
    expected_unit = _value(settings, "ROAD_DYNAMIC_SPEED_UNIT", None)
    try:
        speed = float(speed_value)
        low = float(minimum)
        high = float(maximum)
    except (TypeError, ValueError):
        speed = None
        low = high = 0.0
    slow = _value(settings, "ROAD_DYNAMIC_SLOW_WEIGHTS", ())
    fast = _value(settings, "ROAD_DYNAMIC_FAST_WEIGHTS", ())
    if (speed is not None and speed_unit == expected_unit and
            high > low and low <= speed <= high and len(slow) > 0 and
            len(fast) > 0):
        ratio = (speed - low) / (high - low)
        dynamic = []
        for index in range(count):
            try:
                slow_value = float(slow[index])
                fast_value = float(fast[index])
            except (IndexError, TypeError, ValueError):
                # A short configured window is clipped; no fixed-size index
                # is read beyond it and missing positions use static weights.
                dynamic.append(static[index])
                continue
            dynamic.append(slow_value + (fast_value - slow_value) * ratio)
        if any(value > 0.0 for value in dynamic):
            weights = [value if value > 0.0 else 0.0
                       for value in dynamic]
            if sum(weights) > 0.0:
                source = "DYNAMIC"
            else:
                weights = static
    total = sum(weights)
    if total <= 0.0:
        weights = [1.0] * count
        total = float(count)
    # Normalize after clipping to this frame's fresh rows. This keeps the
    # effective observation scale independent of how many rows survived.
    weights = [value / total for value in weights]
    center_x = 0.0
    for index in range(count):
        center_x += ordered[index]["center"] * weights[index]
    return {"center_x": center_x, "weight_source": source,
            "weights": weights}


def _quality(points, rows, settings):
    count = len(points)
    if count == 0:
        return 0
    span = points[0]["y"] - points[-1]["y"]
    possible = max(1, span // _positive_int(settings, "ROAD_ROW_STEP", 2) + 1)
    coverage = min(1.0, float(count) / float(possible))
    slope_changes = []
    width_changes = []
    for index in range(1, count):
        previous, current = points[index - 1], points[index]
        dy = max(1, previous["y"] - current["y"])
        slope_changes.append(abs(current["center"] -
                                 previous["center"]) / float(dy))
        width_changes.append(abs(current["width"] - previous["width"]))
    mean_slope = sum(slope_changes) / max(1, len(slope_changes))
    mean_width_change = sum(width_changes) / max(1, len(width_changes))
    support = min(100.0, 45.0 + count * 7.0)
    continuity = max(0.0, 100.0 - mean_slope * 16.0 -
                     mean_width_change * 2.0)
    value = support * 0.55 + coverage * 100.0 * 0.25 + continuity * 0.20
    return int(value + 0.5)


def _scene(points, settings):
    if len(points) < 3:
        return "ROAD_CANDIDATE"
    signs = []
    slopes = []
    for index in range(1, len(points)):
        a, b = points[index - 1], points[index]
        dy = max(1, a["y"] - b["y"])
        slope = (b["center"] - a["center"]) / float(dy)
        slopes.append(slope)
        if slope >= 0.20:
            signs.append(1)
        elif slope <= -0.20:
            signs.append(-1)
    if 1 in signs and -1 in signs:
        return "S_BEND_CANDIDATE"
    if not slopes or max([abs(value) for value in slopes]) < 0.20:
        return "STRAIGHT_CANDIDATE"
    # Image coordinates only: left/right labels are diagnostic directions in
    # the image, not claims about a world-frame course turn.
    final_slope = 0.0
    for slope in reversed(slopes):
        if abs(slope) >= 0.20:
            final_slope = slope
            break
    return "IMAGE_RIGHT_BEND_CANDIDATE" if final_slope > 0 else \
           "IMAGE_LEFT_BEND_CANDIDATE"


def _corners(points, settings):
    """Find observed centerline slope changes; never add or repair points."""
    threshold = float(_value(settings, "ROAD_CORNER_SLOPE_CHANGE", 0.45))
    corners = []
    for index in range(1, len(points) - 1):
        before, point, after = points[index - 1:index + 2]
        dy0 = max(1, before["y"] - point["y"])
        dy1 = max(1, point["y"] - after["y"])
        slope0 = (point["center"] - before["center"]) / float(dy0)
        slope1 = (after["center"] - point["center"]) / float(dy1)
        change = slope1 - slope0
        if abs(change) >= threshold:
            corners.append({"x": point["center"], "y": point["y"],
                            "turn": 1 if change > 0 else -1,
                            "provenance": "OBSERVED"})
    return corners


def _fail(reason, height, width, rows=None, scene="UNKNOWN"):
    if rows is None:
        rows = []
    return {
        "valid": False,
        "quality": 0,
        "lateral_error": 0,
        "reason": reason,
        "scene": scene,
        "reference_y": None,
        "center_x": None,
        "reference_center_x": None,
        "reference_provenance": "NO_COMMON_REFERENCE",
        "reference_lateral_error": 0,
        "observed_rows": 0,
        "rows": rows,
        "selected_rows": [],
        "left_edges": [],
        "right_edges": [],
        "corners": [],
        "interpolated": [],
        "width": width,
        "height": height,
        "weight_source": "NONE",
        "candidate_tracks": 0,
    }


def _normalize_center(center_x, width):
    scaled = ((center_x - (width - 1) * 0.5) /
              (width * 0.5)) * 1000.0
    if scaled >= 0.0:
        return int(scaled + 0.5)
    return -int(-scaled + 0.5)


def center_at_y(points, y, max_gap=4.0):
    """Return a fresh observed or short-gap interpolated center at image y.

    Interpolation is suitable for comparing two algorithms at a common image
    row; it does not add an observation to the selected road trace.
    """
    exact = None
    below = None
    above = None
    for point in points:
        point_y = point["y"]
        if abs(point_y - y) < 0.000001:
            exact = point
            break
        if point_y < y and (below is None or point_y > below["y"]):
            below = point
        if point_y > y and (above is None or point_y < above["y"]):
            above = point
    if exact is not None:
        return exact["center"], "OBSERVED"
    if below is None or above is None:
        return None, "NO_COMMON_REFERENCE"
    gap = above["y"] - below["y"]
    if gap <= 0 or gap > max_gap:
        return None, "NO_COMMON_REFERENCE"
    ratio = (y - below["y"]) / float(gap)
    center = below["center"] + (above["center"] - below["center"]) * ratio
    return center, "INTERPOLATED"


def detect_road(road_mask, settings=None, speed_value=None, speed_unit=None,
                valid_region=None):
    """Detect a fresh, paired-edge binary road path in an 80x60 mask.

    A valid center requires at least four bilateral observed rows with a
    bounded vertical span. Invalid frames return zero quality and zero error.
    Speed-dependent weights are used only when callers state a matching unit.
    """
    width = _positive_int(settings, "ROAD_GRID_WIDTH", 80)
    height = _positive_int(settings, "ROAD_GRID_HEIGHT", 60)
    if len(road_mask) != width * height:
        raise ValueError("road mask size differs from configured grid")
    if valid_region is not None and len(valid_region) != width * height:
        raise ValueError("road valid-region size differs from configured grid")

    floor_y = _positive_int(settings, "ROAD_FLOOR_Y", 20, 0)
    floor_y = min(height - 1, floor_y)
    row_step = _positive_int(settings, "ROAD_ROW_STEP", 2)
    min_width = _positive_int(settings, "ROAD_MIN_WIDTH", 8)
    max_width = _positive_int(settings, "ROAD_MAX_WIDTH", width - 2)
    max_width = min(width - 2, max_width)
    margin_x = _positive_int(settings, "ROAD_SEED_MARGIN_X", 8, 0)
    rows = list(range(height - 1, floor_y - 1, -row_step))
    row_candidates = []
    row_partials = []
    row_unknown = []
    for y in rows:
        candidates, partials, unknown = _row_candidates(
            road_mask, valid_region, width, y, min_width, max_width, margin_x)
        row_candidates.append(candidates)
        row_partials.append(partials)
        row_unknown.append(unknown)

    row_diag_by_y = {}
    for index, y in enumerate(rows):
        candidates = row_candidates[index]
        partials = row_partials[index]
        if candidates:
            row_diag_by_y[y] = {
                "y": y,
                "left_status": "OBSERVED",
                "right_status": "OBSERVED",
                "candidate_count": len(candidates),
                "candidates": candidates,
                "provenance": "OBSERVED",
            }
        elif partials:
            part = partials[0]
            row_diag_by_y[y] = {
                "y": y,
                "left_status": part["left_status"],
                "right_status": part["right_status"],
                "candidate_count": 0,
                "candidates": [],
                "provenance": part["provenance"],
            }
        else:
            row_diag_by_y[y] = {
                "y": y,
                "left_status": "MISSING",
                "right_status": "MISSING",
                "candidate_count": 0,
                "candidates": [],
                "provenance": "UNKNOWN_REGION" if row_unknown[index] else
                              "NOT_OBSERVED",
            }
    row_diag = []
    for y in range(height - 1, -1, -1):
        if y in row_diag_by_y:
            row_diag.append(row_diag_by_y[y])
        else:
            row_diag.append({
                "y": y,
                "left_status": "NOT_SCANNED",
                "right_status": "NOT_SCANNED",
                "candidate_count": 0,
                "candidates": [],
                "provenance": "NOT_SCANNED",
            })

    seed_rows = _positive_int(settings, "ROAD_SEED_ROWS", 16)
    tracks = _make_tracks(rows, row_candidates, seed_rows, settings)
    min_rows = _positive_int(settings, "ROAD_MIN_OBSERVED_ROWS", 4)
    min_span = _positive_int(settings, "ROAD_MIN_VERTICAL_SPAN", 8)
    viable = []
    for track in tracks:
        points = track["points"]
        span = points[0]["y"] - points[-1]["y"] if points else 0
        if len(points) >= min_rows and span >= min_span:
            viable.append(track)
    if not viable:
        any_bilateral = any(row_candidates)
        reason = "INSUFFICIENT_FRESH_ROWS" if any_bilateral else "NO_PAIRED_EDGES"
        return _fail(reason, height, width, row_diag)

    viable.sort(key=_track_score, reverse=True)
    best = viable[0]
    best_points = best["points"]
    reference_y = float(_value(settings, "ROAD_REFERENCE_Y", 40))
    best_obs = min(best_points, key=lambda point: abs(point["y"] - reference_y))
    best_center = best_obs["center"]
    ambiguity_margin = float(_value(settings, "ROAD_AMBIGUITY_MARGIN", 8.0))
    ambiguity_sep = float(_value(settings, "ROAD_AMBIGUITY_SEPARATION", 8.0))
    best_score = _track_score(best)
    for alternative in viable[1:]:
        if best_score - _track_score(alternative) > ambiguity_margin:
            break
        alt_points = alternative["points"]
        alt_obs = min(alt_points,
                      key=lambda point: abs(point["y"] - reference_y))
        if abs(alt_obs["center"] - best_center) >= ambiguity_sep:
            return _fail("AMBIGUOUS_ROAD", height, width, row_diag,
                         "AMBIGUOUS_CANDIDATE")

    # Weight only the selected track's fresh paired-edge observations. Row
    # weights are clipped to the available observations and then normalized.
    weighted = weighted_observation(best_points, settings, speed_value,
                                    speed_unit)
    center_x = weighted["center_x"]
    reference_y = float(_value(settings, "ROAD_REFERENCE_Y", 40))
    reference_gap = float(_value(
        settings, "ROAD_SHARED_REFERENCE_MAX_GAP_CELLS", 4.0))
    reference_center_x, reference_provenance = center_at_y(
        best_points, reference_y, reference_gap)
    quality = _quality(best_points, rows, settings)
    min_quality = int(_value(settings, "ROAD_QUALITY_MIN", 60))
    if quality < min_quality:
        return _fail("LOW_QUALITY", height, width, row_diag,
                     _scene(best_points, settings))

    # Keep the established surface_v1 scale: (center_x - 39.5) / 40 * 1000.
    lateral_error = _normalize_center(center_x, width)
    selected_points = []
    for point in best_points:
        selected_points.append({
            "y": point["y"],
            "left": point["left"],
            "right": point["right"],
            "center": point["center"],
            "width": point["width"],
            "left_status": "OBSERVED",
            "right_status": "OBSERVED",
            "provenance": "OBSERVED",
        })
    left_edges = [(point["left"], point["y"]) for point in best_points]
    right_edges = [(point["right"], point["y"]) for point in best_points]
    return {
        "valid": True,
        "quality": quality,
        "lateral_error": lateral_error,
        "reason": "OK",
        "scene": _scene(best_points, settings),
        "reference_y": reference_y,
        "center_x": center_x,
        "reference_center_x": reference_center_x,
        "reference_provenance": reference_provenance,
        "reference_lateral_error": (
            _normalize_center(reference_center_x, width)
            if reference_center_x is not None else 0),
        "observed_rows": len(best_points),
        "rows": row_diag,
        "selected_rows": selected_points,
        "left_edges": left_edges,
        "right_edges": right_edges,
        "corners": _corners(best_points, settings),
        "interpolated": [],
        "width": width,
        "height": height,
        "weight_source": weighted["weight_source"],
        "weights": weighted["weights"],
        "candidate_tracks": len(viable),
        "track_score": best_score,
        "vertical_span": best_points[0]["y"] - best_points[-1]["y"],
    }
