"""Adaptive grayscale corridor detector for OpenMV MicroPython.

Rows are classified independently with Otsu thresholds. An explicit model
setting selects either the V2 global/local Theil-Sen line fit or a bounded,
deterministic quadratic consensus model for modest curvature.
"""


SUPPORTED_MODES = ("white_corridor", "black_edges")


def median3(a, b, c):
    if a > b:
        a, b = b, a
    if b > c:
        b, c = c, b
    if a > b:
        a, b = b, a
    return b


def _median(values):
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return 0.0
    middle = count // 2
    if count & 1:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def _otsu(pixels, min_dark_pixels, min_light_pixels, min_contrast):
    histogram = [0] * 256
    total_sum = 0
    for value in pixels:
        histogram[value] += 1
        total_sum += value

    total = len(pixels)
    if total < min_dark_pixels + min_light_pixels:
        return None, "WEAK_CLASS", 0, 0, 0.0

    dark_count = 0
    dark_sum = 0
    best_threshold = None
    best_variance = -1.0
    for threshold in range(255):
        dark_count += histogram[threshold]
        dark_sum += threshold * histogram[threshold]
        light_count = total - dark_count
        if dark_count < min_dark_pixels or light_count < min_light_pixels:
            continue
        dark_mean = float(dark_sum) / float(dark_count)
        light_mean = float(total_sum - dark_sum) / float(light_count)
        contrast = light_mean - dark_mean
        if contrast < min_contrast:
            continue
        # Otsu's between-class variance, scaled without dividing by total.
        difference = float(dark_sum) * light_count - \
            float(total_sum - dark_sum) * dark_count
        variance = difference * difference / \
            (float(dark_count) * float(light_count))
        if variance > best_variance:
            best_variance = variance
            best_threshold = threshold

    if best_threshold is None:
        minimum = min(pixels)
        maximum = max(pixels)
        if maximum == minimum or maximum - minimum < min_contrast:
            return None, "LOW_CONTRAST", 0, 0, 0.0
        return None, "WEAK_CLASS", 0, 0, 0.0

    dark_count = 0
    dark_sum = 0
    for value in pixels:
        if value <= best_threshold:
            dark_count += 1
            dark_sum += value
    light_count = total - dark_count
    light_sum = total_sum - dark_sum
    contrast = (float(light_sum) / float(light_count) -
                float(dark_sum) / float(dark_count))
    return best_threshold, "OK", dark_count, light_count, contrast


def _class_metrics(pixels, threshold):
    dark_count = 0
    dark_sum = 0
    light_count = 0
    light_sum = 0
    for value in pixels:
        if value <= threshold:
            dark_count += 1
            dark_sum += value
        else:
            light_count += 1
            light_sum += value
    if dark_count == 0 or light_count == 0:
        return dark_count, light_count, 0.0
    contrast = (float(light_sum) / float(light_count) -
                float(dark_sum) / float(dark_count))
    return dark_count, light_count, contrast


def _scene_context_summary(context_profiles, image_width, params):
    """Measure current-frame evidence for separate dark/light road classes."""
    try:
        profiles = list(context_profiles)
    except (TypeError, ValueError):
        profiles = []
    maximum_rows = params.get("scene_context_max_rows", 9)
    if not profiles:
        return {"reason": "SCENE_NO_CONTEXT", "threshold": None,
                "otsu_threshold": None,
                "contrast": 0.0, "dark_class": 0, "light_class": 0,
                "dark_ratio": 0.0, "light_ratio": 0.0}
    if len(profiles) > maximum_rows:
        return {"reason": "SCENE_BAD_CONTEXT", "threshold": None,
                "otsu_threshold": None,
                "contrast": 0.0, "dark_class": 0, "light_class": 0,
                "dark_ratio": 0.0, "light_ratio": 0.0}

    pixels = []
    for item in profiles:
        if len(item) != 2 or len(item[1]) != image_width:
            return {"reason": "SCENE_BAD_WIDTH", "threshold": None,
                    "otsu_threshold": None,
                    "contrast": 0.0, "dark_class": 0, "light_class": 0,
                    "dark_ratio": 0.0, "light_ratio": 0.0}
        pixels.extend(item[1])

    total = len(pixels)
    dark_ratio_min = params.get("scene_min_dark_class_ratio_percent", 1)
    light_ratio_min = params.get("scene_min_light_class_ratio_percent", 5)
    min_dark = max(
        params.get("scene_min_dark_class_pixels", 8),
        (total * dark_ratio_min + 99) // 100)
    min_light = max(
        params.get("scene_min_light_class_pixels", 24),
        (total * light_ratio_min + 99) // 100)
    otsu_threshold, reason, dark_count, light_count, contrast = _otsu(
        pixels, min_dark, min_light,
        params.get("scene_min_contrast", 36))
    if otsu_threshold is None:
        return {"reason": "SCENE_" + reason, "threshold": None,
                "otsu_threshold": None,
                "contrast": contrast, "dark_class": dark_count,
                "light_class": light_count,
                "dark_ratio": 0.0, "light_ratio": 0.0}

    raw_dark_sum = 0
    raw_dark_count = 0
    for value in pixels:
        if value <= otsu_threshold:
            raw_dark_sum += value
            raw_dark_count += 1
    raw_dark_mean = float(raw_dark_sum) / float(raw_dark_count)
    reference_fraction = params.get(
        "scene_dark_reference_fraction", 0.25)
    threshold = min(
        otsu_threshold,
        int(raw_dark_mean + reference_fraction * contrast))
    dark_count, light_count, contrast = _class_metrics(pixels, threshold)
    dark_ratio = (100.0 * dark_count / total) if total else 0.0
    light_ratio = (100.0 * light_count / total) if total else 0.0
    if dark_count < min_dark or light_count < min_light:
        reason = "SCENE_WEAK_CLASS"
        threshold = None
    elif contrast < params.get("scene_min_contrast", 36):
        reason = "SCENE_LOW_CONTRAST"
        threshold = None
    elif dark_ratio < dark_ratio_min or light_ratio < light_ratio_min:
        reason = "SCENE_WEAK_CLASS"
        threshold = None
    else:
        reason = "OK"
    return {"reason": reason, "threshold": threshold,
            "otsu_threshold": otsu_threshold,
            "contrast": contrast, "dark_class": dark_count,
            "light_class": light_count, "dark_ratio": dark_ratio,
            "light_ratio": light_ratio}


def _bridge_single_light_gaps(mask):
    repaired = mask[:]
    count = 0
    for index in range(1, len(mask) - 1):
        if mask[index - 1] and not mask[index] and mask[index + 1]:
            repaired[index] = True
            count += 1
    return repaired, count


def _runs(mask):
    result = []
    start = -1
    width = len(mask)
    for x in range(width + 1):
        dark = x < width and mask[x]
        if dark and start < 0:
            start = x
        elif not dark and start >= 0:
            result.append((start, x - 1, x - start))
            start = -1
    return result


def _matching_runs(raw_runs, params, mode):
    if mode == "white_corridor":
        minimum = params["white_corridor_dark_min_px"]
        return [run for run in raw_runs if run[2] >= minimum]
    minimum = params["black_edge_width_min_px"]
    maximum = params["black_edge_width_max_px"]
    return [run for run in raw_runs if minimum <= run[2] <= maximum]


def _pair_candidates(mask, raw_runs, runs, params):
    pairs = []
    rejected_gap = 0
    rejected_interior = 0
    prefix_dark = [0] * (len(mask) + 1)
    for x, dark in enumerate(mask):
        prefix_dark[x + 1] = prefix_dark[x] + (1 if dark else 0)

    for left_index in range(len(runs)):
        left = runs[left_index]
        for right_index in range(left_index + 1, len(runs)):
            right = runs[right_index]
            gap = right[0] - left[1] - 1
            if gap < params["track_gap_min_px"] or \
                    gap > params["track_gap_max_px"]:
                rejected_gap += 1
                continue

            dark_count = prefix_dark[right[0]] - prefix_dark[left[1] + 1]
            if dark_count * 100 > \
                    gap * params["max_interior_dark_ratio_percent"]:
                rejected_interior += 1
                continue

            longest_internal = 0
            for run in raw_runs:
                if run[0] <= left[1]:
                    continue
                if run[1] >= right[0]:
                    break
                if run[2] > longest_internal:
                    longest_internal = run[2]
            if longest_internal > params["max_interior_dark_run_px"]:
                rejected_interior += 1
                continue

            # Both modes define boundaries at the two inward-facing edges.
            left_inner = float(left[1])
            right_inner = float(right[0])
            pairs.append((left_inner, right_inner, gap, dark_count))

    return pairs, rejected_gap, rejected_interior


def diagnose_profile(y, pixels, params, mode, scene_threshold=None,
                     apply_boundary_guard=False):
    """Return a row candidate and measurements for VDBG/tests.

    When enabled, the row-local Otsu threshold is capped by the threshold
    measured from this frame's scene context. Class counts and contrast are
    then recomputed at the threshold actually used for the row.
    """
    base = {"y": y, "threshold": None, "local_threshold": None,
            "threshold_cap": None, "contrast": 0.0, "dark_class": 0,
            "light_class": 0, "runs": (), "repairs": 0,
            "gap_rejects": 0, "interior_rejects": 0,
            "candidate_count": 0, "candidate": None}
    if mode not in SUPPORTED_MODES:
        base["reason"] = "BAD_MODE"
        return base
    if not pixels:
        base["reason"] = "BAD_WIDTH"
        return base

    local_threshold, threshold_reason, _dark, _light, _contrast = _otsu(
        pixels, params["min_dark_class_pixels"],
        params["min_light_class_pixels"], params["min_class_contrast"])
    base["local_threshold"] = local_threshold
    if local_threshold is None:
        base["reason"] = threshold_reason
        return base

    if scene_threshold is not None:
        threshold_cap = (scene_threshold +
                         params.get("scene_row_threshold_margin", 10))
        base["threshold_cap"] = threshold_cap
    else:
        threshold_cap = None

    if apply_boundary_guard:
        if scene_threshold is None:
            base["reason"] = "SCENE_INVALID"
            return base
        threshold = min(local_threshold, threshold_cap)
    else:
        threshold = local_threshold
    dark_class, light_class, contrast = _class_metrics(pixels, threshold)
    base.update({"threshold": threshold, "contrast": contrast,
                 "dark_class": dark_class, "light_class": light_class})
    if dark_class < params["min_dark_class_pixels"] or \
            light_class < params["min_light_class_pixels"]:
        base["reason"] = "WEAK_CLASS"
        return base
    if contrast < params["min_class_contrast"]:
        base["reason"] = "LOW_CONTRAST"
        return base

    mask = [value <= threshold for value in pixels]
    mask, repairs = _bridge_single_light_gaps(mask)
    raw_runs = _runs(mask)
    matching = _matching_runs(raw_runs, params, mode)
    max_runs = params["max_dark_runs_per_row"]
    if len(matching) > max_runs:
        base.update({"reason": "NOISY_ROW", "runs":
                     tuple(run[2] for run in raw_runs[:8]),
                     "repairs": repairs})
        return base

    pairs, gap_rejects, interior_rejects = _pair_candidates(
        mask, raw_runs, matching, params)
    if len(pairs) > 1:
        reason = "MULTIPLE_PAIRS"
        candidate = None
    elif len(pairs) == 1:
        reason = "OK"
        left_inner, right_inner, gap, dark_count = pairs[0]
        candidate = (left_inner, right_inner,
                     (left_inner + right_inner) * 0.5, gap, dark_count)
    elif not raw_runs:
        reason = "NO_DARK"
        candidate = None
    elif not matching:
        reason = "NO_WIDTH_MATCH"
        candidate = None
    elif len(matching) == 1:
        reason = "ONE_EDGE"
        candidate = None
    elif interior_rejects:
        reason = "INTERIOR_DARK"
        candidate = None
    elif gap_rejects:
        reason = "GAP_RANGE"
        candidate = None
    else:
        reason = "NO_PAIR"
        candidate = None

    base.update({"reason": reason,
                 "runs": tuple(run[2] for run in raw_runs[:8]),
                 "repairs": repairs, "gap_rejects": gap_rejects,
                 "interior_rejects": interior_rejects,
                 "candidate_count": len(pairs), "candidate": candidate})
    return base


def _theil_sen(points):
    slopes = []
    count = len(points)
    for left_index in range(count):
        y0, x0 = points[left_index]
        for right_index in range(left_index + 1, count):
            y1, x1 = points[right_index]
            if y1 != y0:
                slopes.append((x1 - x0) / float(y1 - y0))
    if not slopes:
        return 0.0, _median([point[1] for point in points])
    slope = _median(slopes)
    intercept = _median([x - slope * y for y, x in points])
    return slope, intercept


def _local_reference_center(points, reference_y, max_points,
                             min_points):
    ordered = sorted(points, key=lambda point: abs(point[0] - reference_y))
    local = sorted(ordered[:max_points])
    if len(local) >= min_points and local[0][0] < reference_y < local[-1][0]:
        slope, intercept = _theil_sen(local)
        predicted = slope * reference_y + intercept
        residual = max(abs(x - (slope * y + intercept)) for y, x in local)
        return predicted, residual, "LOCAL_THEIL_SEN"

    before = [point for point in points if point[0] < reference_y]
    after = [point for point in points if point[0] > reference_y]
    exact = [point for point in points if point[0] == reference_y]
    if exact:
        return exact[0][1], 0.0, "REFERENCE_ROW"
    if before and after:
        low = max(before)
        high = min(after)
        ratio = float(reference_y - low[0]) / float(high[0] - low[0])
        value = low[1] + ratio * (high[1] - low[1])
        return value, 0.0, "BRACKET_INTERPOLATION"
    return None, 0.0, "REFERENCE_UNCOVERED"


def _confidence(support, total_rows, residual, contrast, params,
               residual_limit=None):
    if residual_limit is None:
        residual_limit = params["max_fit_residual_px"]
    support_score = 100.0 * float(support) / float(total_rows)
    residual_score = 100.0 * max(
        0.0, 1.0 - float(residual) / float(residual_limit))
    contrast_score = min(
        100.0,
        100.0 * float(contrast) /
        float(params["contrast_full_score_at"]))
    score = (0.50 * support_score + 0.25 * residual_score +
             0.25 * contrast_score)
    return max(0, min(100, int(score + 0.5)))


def _quadratic_coefficients(points, reference_y, scale):
    """Solve x=a+b*t+c*t**2 for one deterministic three-row seed."""
    y0, x0 = points[0]
    y1, x1 = points[1]
    y2, x2 = points[2]
    t0 = float(y0 - reference_y) / scale
    t1 = float(y1 - reference_y) / scale
    t2 = float(y2 - reference_y) / scale
    if t0 == t1 or t1 == t2 or t0 == t2:
        return None
    slope01 = float(x1 - x0) / (t1 - t0)
    slope12 = float(x2 - x1) / (t2 - t1)
    c = (slope12 - slope01) / (t2 - t0)
    b = slope01 - c * (t0 + t1)
    a = float(x0) - b * t0 - c * t0 * t0
    return a, b, c


def _quadratic_predict(coefficients, y, reference_y, scale):
    t = float(y - reference_y) / scale
    a, b, c = coefficients
    return a + b * t + c * t * t


def _quadratic_inliers(points, coefficients, reference_y, scale,
                       residual_limit):
    inliers = []
    residuals = []
    for y, x in points:
        residual = abs(float(x) - _quadratic_predict(
            coefficients, y, reference_y, scale))
        if residual <= residual_limit:
            inliers.append((y, x))
            residuals.append(residual)
    return inliers, residuals


def _longest_consecutive_support(inliers, scan_rows):
    supported_y = set(point[0] for point in inliers)
    longest = 0
    current = 0
    for y in sorted(scan_rows):
        if y in supported_y:
            current += 1
            if current > longest:
                longest = current
        else:
            current = 0
    return longest


def _quadratic_geometry_reason(coefficients, inliers, reference_y,
                               scale, image_width, params):
    y_values = [point[0] for point in inliers]
    if _longest_consecutive_support(
            inliers, params["scan_rows"]) < \
            params["quadratic_min_consecutive_support_rows"]:
        return "SUPPORT_FRAGMENTED"
    if not y_values or min(y_values) > reference_y or \
            max(y_values) < reference_y:
        return "REFERENCE_UNCOVERED"
    if max(y_values) - min(y_values) < params["min_fit_y_span_px"]:
        return "FIT_Y_SPAN"

    a, b, c = coefficients
    if a < 0.0 or a > image_width - 1:
        return "CENTER_OUT_OF_RANGE"

    curvature = 2.0 * c / (scale * scale)
    if abs(curvature) > params["quadratic_max_curvature_px_per_y2"]:
        return "MODEL_CURVATURE"

    first_t = float(min(y_values) - reference_y) / scale
    last_t = float(max(y_values) - reference_y) / scale
    first_slope = (b + 2.0 * c * first_t) / scale
    last_slope = (b + 2.0 * c * last_t) / scale
    if max(abs(first_slope), abs(last_slope)) > \
            params["quadratic_max_slope_px_per_y"]:
        return "MODEL_SLOPE"
    return "OK"


def _quadratic_model(candidates, summaries, ambiguous,
                     image_width, params):
    reference_y = params["reference_y"]
    scale = float(params["quadratic_t_scale_px"])
    residual_limit = params["quadratic_inlier_residual_px"]
    min_support = params["min_valid_rows"]
    center_points = [(item[0], item[3]) for item in candidates]
    models = []
    sufficient_support_seen = False
    peak_support = 0
    rejected_reasons = {}

    count = len(center_points)
    tested_hypotheses = 0
    for first in range(count - 2):
        for second in range(first + 1, count - 1):
            for third in range(second + 1, count):
                coefficients = _quadratic_coefficients(
                    (center_points[first], center_points[second],
                     center_points[third]), reference_y, scale)
                if coefficients is None:
                    continue
                tested_hypotheses += 1
                inliers, residuals = _quadratic_inliers(
                    center_points, coefficients, reference_y, scale,
                    residual_limit)
                if len(inliers) > peak_support:
                    peak_support = len(inliers)
                if len(inliers) < min_support:
                    continue
                sufficient_support_seen = True
                geometry = _quadratic_geometry_reason(
                    coefficients, inliers, reference_y, scale,
                    image_width, params)
                if geometry != "OK":
                    rejected_reasons[geometry] = \
                        rejected_reasons.get(geometry, 0) + 1
                    continue
                models.append({
                    "coefficients": coefficients,
                    "inliers": inliers,
                    "inlier_y": tuple(point[0] for point in inliers),
                    "support": len(inliers),
                    "median_residual": _median(residuals),
                    "sum_residual": sum(residuals),
                    "max_residual": max(residuals),
                })

    if not models:
        if rejected_reasons:
            reason_order = ("MODEL_CURVATURE", "MODEL_SLOPE",
                            "SUPPORT_FRAGMENTED", "FIT_Y_SPAN",
                            "REFERENCE_UNCOVERED",
                            "CENTER_OUT_OF_RANGE")
            reason = "MODEL_SUPPORT"
            for candidate_reason in reason_order:
                if rejected_reasons.get(candidate_reason, 0):
                    reason = candidate_reason
                    break
        elif sufficient_support_seen:
            reason = "MODEL_GEOMETRY"
        else:
            reason = "MODEL_SUPPORT"
        return _failure(reason, summaries, candidates,
                        support=peak_support, ambiguous=ambiguous,
                        model="quadratic",
                        model_hypotheses=tested_hypotheses)

    models.sort(key=lambda item: (-item["support"],
                                  item["median_residual"],
                                  item["sum_residual"],
                                  item["coefficients"][0],
                                  item["coefficients"][1],
                                  item["coefficients"][2]))
    best = models[0]
    coefficients = best["coefficients"]
    inliers = best["inliers"]
    stable = False

    # Robustly aggregate all seed fits with the same consensus set. Median
    # coefficients avoid least-squares leverage from an isolated bad row.
    for _ in range(3):
        inlier_y = tuple(point[0] for point in inliers)
        consensus_models = [item for item in models
                            if item["inlier_y"] == inlier_y]
        if not consensus_models:
            break
        coefficients = (
            _median([item["coefficients"][0] for item in consensus_models]),
            _median([item["coefficients"][1] for item in consensus_models]),
            _median([item["coefficients"][2] for item in consensus_models]),
        )
        inliers, residuals = _quadratic_inliers(
            center_points, coefficients, reference_y, scale, residual_limit)
        if len(inliers) < min_support:
            return _failure("MODEL_REFIT", summaries, candidates,
                            support=len(inliers), ambiguous=ambiguous,
                            residual=max(residuals) if residuals else 0.0,
                            model="quadratic", coefficients=coefficients,
                            model_hypotheses=tested_hypotheses)
        geometry = _quadratic_geometry_reason(
            coefficients, inliers, reference_y, scale, image_width, params)
        if geometry != "OK":
            return _failure(geometry, summaries, candidates,
                            support=len(inliers), ambiguous=ambiguous,
                            residual=max(residuals) if residuals else 0.0,
                            model="quadratic", coefficients=coefficients,
                            model_hypotheses=tested_hypotheses)
        new_inlier_y = tuple(point[0] for point in inliers)
        if new_inlier_y == inlier_y:
            stable = True
            break

    if not stable:
        return _failure("MODEL_REFIT", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        model="quadratic", coefficients=coefficients,
                        model_hypotheses=tested_hypotheses)

    inliers, residuals = _quadratic_inliers(
        center_points, coefficients, reference_y, scale, residual_limit)
    if len(inliers) < min_support:
        return _failure("MODEL_REFIT", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        model="quadratic", coefficients=coefficients,
                        model_hypotheses=tested_hypotheses)
    geometry = _quadratic_geometry_reason(
        coefficients, inliers, reference_y, scale, image_width, params)
    if geometry != "OK":
        return _failure(geometry, summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        residual=max(residuals) if residuals else 0.0,
                        model="quadratic", coefficients=coefficients,
                        model_hypotheses=tested_hypotheses)

    final_support = len(inliers)
    final_median = _median(residuals)
    final_sum = sum(residuals)
    final_mean = final_sum / float(final_support)
    final_center = coefficients[0]
    center_ambiguity = params["quadratic_model_center_ambiguity_px"]
    median_margin = params["quadratic_tie_median_residual_margin_px"]
    sum_margin = params["quadratic_tie_sum_residual_margin_px"]
    mean_margin = params["quadratic_tie_mean_residual_margin_px"]
    for competing in models:
        if competing["support"] < final_support:
            continue
        if competing["support"] == final_support:
            if abs(competing["median_residual"] - final_median) > \
                    median_margin:
                continue
            if abs(competing["sum_residual"] - final_sum) > sum_margin:
                continue
        else:
            if competing["median_residual"] > \
                    final_median + median_margin:
                continue
            competing_mean = competing["sum_residual"] / \
                float(competing["support"])
            if competing_mean > final_mean + mean_margin:
                continue
        if abs(competing["coefficients"][0] - final_center) > \
                center_ambiguity:
            return _failure("MODEL_AMBIGUOUS", summaries, candidates,
                            support=final_support, ambiguous=ambiguous,
                            residual=max(residuals) if residuals else 0.0,
                            slope=coefficients[1] / scale,
                            intercept=coefficients[0], model="quadratic",
                            coefficients=coefficients,
                            curvature=2.0 * coefficients[2] / (scale * scale),
                            model_hypotheses=tested_hypotheses)

    fit_residual = max(residuals) if residuals else 0.0
    curvature = 2.0 * coefficients[2] / (scale * scale)
    slope_at_reference = coefficients[1] / scale
    support_y = [point[0] for point in inliers]
    inlier_by_y = {item[0]: item for item in candidates}
    inlier_candidates = [inlier_by_y[y] for y in support_y]
    edge_jump = _edge_jump(inlier_candidates, params)
    if edge_jump is not None:
        return _failure("EDGE_JUMP", summaries, candidates,
                        support=final_support, ambiguous=ambiguous,
                        residual=fit_residual, slope=slope_at_reference,
                        intercept=final_center, model="quadratic",
                        coefficients=coefficients, curvature=curvature,
                        model_hypotheses=tested_hypotheses,
                        edge_jump=edge_jump)
    contrast_min = min(item[4] for item in inlier_candidates)
    confidence = _confidence(
        final_support, len(summaries), fit_residual, contrast_min,
        params, residual_limit=residual_limit)
    if confidence < params["min_confidence"]:
        return _failure("LOW_QUALITY", summaries, candidates,
                        support=final_support, ambiguous=ambiguous,
                        residual=fit_residual, slope=slope_at_reference,
                        intercept=final_center, model="quadratic",
                        coefficients=coefficients, curvature=curvature,
                        confidence=confidence,
                        model_hypotheses=tested_hypotheses)

    points_for_draw = [(item[0], item[1], item[2], item[3])
                       for item in inlier_candidates]
    thresholds = [item["threshold"] for item in summaries
                  if item["threshold"] is not None]
    contrasts = [item[4] for item in inlier_candidates]
    details = {
        "reason": "OK",
        "model": "quadratic",
        "coefficients": coefficients,
        "curvature": curvature,
        "threshold_min": min(thresholds) if thresholds else None,
        "threshold_max": max(thresholds) if thresholds else None,
        "contrast_min": contrast_min,
        "contrast_avg": sum(contrasts) / float(len(contrasts)),
        "fit_residual": fit_residual,
        "local_residual": 0.0,
        "slope": slope_at_reference,
        "intercept": final_center,
        "local_method": "QUADRATIC_MEDIAN_REFIT",
        "candidate_rows": len(candidates),
        "model_hypotheses": tested_hypotheses,
        "model_count": len(models),
        "confidence": confidence,
        "rows": summaries,
    }
    for key in ("scene_threshold", "scene_otsu_threshold",
                "scene_contrast", "scene_dark_class",
                "scene_light_class", "scene_dark_ratio",
                "scene_light_ratio", "scene_reason",
                "boundary_guard_enabled", "scene_threshold_cap"):
        if key in summaries[0]:
            details[key] = summaries[0][key]
    return (1, final_center, confidence, final_support, ambiguous,
            points_for_draw, details)


class ObservationFilter:
    """Fresh-frame EMA which clears on invalid data or a long frame gap."""

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


def _failure(reason, summaries, candidates, support=0, ambiguous=0,
             slope=0.0, intercept=0.0, residual=0.0, local_residual=0.0,
             local_method="NONE", confidence=0, model="linear",
             curvature=0.0, coefficients=None, model_hypotheses=0,
             edge_jump=None):
    thresholds = [item["threshold"] for item in summaries
                  if item["threshold"] is not None]
    contrasts = [item["contrast"] for item in summaries
                 if item["threshold"] is not None]
    details = {
        "reason": reason,
        "threshold_min": min(thresholds) if thresholds else None,
        "threshold_max": max(thresholds) if thresholds else None,
        "contrast_min": min(contrasts) if contrasts else 0.0,
        "contrast_avg": (sum(contrasts) / float(len(contrasts))
                         if contrasts else 0.0),
        "fit_residual": residual,
        "local_residual": local_residual,
        "slope": slope,
        "intercept": intercept,
        "local_method": local_method,
        "model": model,
        "curvature": curvature,
        "coefficients": coefficients,
        "candidate_rows": len(candidates),
        "model_hypotheses": model_hypotheses,
        "confidence": confidence,
        "rows": summaries,
    }
    if summaries:
        first = summaries[0]
        for key in ("scene_threshold", "scene_otsu_threshold",
                    "scene_contrast",
                    "scene_dark_class", "scene_light_class",
                    "scene_dark_ratio", "scene_light_ratio",
                    "scene_reason", "boundary_guard_enabled",
                    "scene_threshold_cap"):
            if key in first:
                details[key] = first[key]
    if edge_jump is not None:
        details["edge_jump"] = edge_jump
    return 0, 0.0, 0, support, ambiguous, [], details


def _stamp_scene(summary, scene, boundary_guard_enabled, params):
    summary.update({
        "scene_threshold": scene["threshold"],
        "scene_otsu_threshold": scene["otsu_threshold"],
        "scene_contrast": scene["contrast"],
        "scene_dark_class": scene["dark_class"],
        "scene_light_class": scene["light_class"],
        "scene_dark_ratio": scene["dark_ratio"],
        "scene_light_ratio": scene["light_ratio"],
        "scene_reason": scene["reason"],
        "boundary_guard_enabled": boundary_guard_enabled,
        "scene_threshold_cap": (
            scene["threshold"] +
            params.get("scene_row_threshold_margin", 10)
            if scene["threshold"] is not None else None),
    })
    return summary


def _edge_jump(inliers, params):
    """Find adjacent accepted rows with a discontinuous left or right edge."""
    if not params.get("boundary_guard_enabled", True):
        return None
    ordered = sorted(inliers, key=lambda item: item[0])
    max_slope = params.get("edge_max_slope_px_per_y", 4.0)
    tolerance = params.get("edge_jump_tolerance_px", 2.0)
    for index in range(1, len(ordered)):
        before = ordered[index - 1]
        current = ordered[index]
        delta_y = current[0] - before[0]
        if delta_y <= 0:
            return (before[0], current[0], 0.0, 0.0, 0.0)
        limit = max_slope * delta_y + tolerance
        left_delta = current[1] - before[1]
        right_delta = current[2] - before[2]
        if abs(left_delta) > limit or abs(right_delta) > limit:
            return (before[0], current[0], left_delta, right_delta, limit)
    return None


def _quadratic_config_is_valid(rows, params):
    scan_rows = params.get("scan_rows", ())
    image_height = params.get("image_height", 0)
    radius = params.get("profile_band_radius", 0)
    reference_y = params.get("reference_y", -1)
    if not scan_rows or len(scan_rows) > 9 or \
            len(set(scan_rows)) != len(scan_rows):
        return False
    for index in range(1, len(scan_rows)):
        if scan_rows[index] <= scan_rows[index - 1]:
            return False
    if len(rows) > 9 or len(set(item[0] for item in rows)) != len(rows):
        return False
    if image_height < 1 or radius < 0 or \
            reference_y < 0 or reference_y >= image_height:
        return False
    if min(scan_rows) > reference_y or max(scan_rows) < reference_y:
        return False
    for y in scan_rows:
        if y - radius < 0 or y + radius >= image_height:
            return False
    for item in rows:
        y = item[0]
        if y - radius < 0 or y + radius >= image_height:
            return False
    return True


def detect_track_center(rows, image_width, params, mode=None, model=None,
                        context_profiles=None):
    """Return fit-gated center at reference_y and diagnostics.

    Result fields are (valid, center, quality, support, ambiguous, points,
    details). `points` contains accepted (y, left_inner, right_inner, center)
    rows for drawing. Quality is a 0..100 score, not a probability.
    """
    if mode is None:
        mode = params.get("detection_mode", "black_edges")
    if mode not in SUPPORTED_MODES:
        return _failure("BAD_MODE", [], [])
    if model is None:
        model = params.get("track_model", "linear")
    if model not in ("linear", "quadratic"):
        return _failure("BAD_TRACK_MODEL", [], [], model=model)

    rows = list(rows)
    if model == "quadratic":
        if not _quadratic_config_is_valid(rows, params):
            return _failure("BAD_QUADRATIC_CONFIG", [], [],
                            model="quadratic")

    boundary_guard_enabled = bool(
        params.get("boundary_guard_enabled", True))
    scene_source = rows if context_profiles is None else context_profiles
    scene = _scene_context_summary(scene_source, image_width, params)
    scene_threshold = (scene["threshold"]
                       if scene["reason"] == "OK" else None)

    summaries = []
    candidates = []
    ambiguous = 0
    for y, pixels in rows:
        if len(pixels) != image_width:
            summary = {"y": y, "reason": "BAD_WIDTH", "threshold": None,
                       "local_threshold": None, "threshold_cap": None,
                       "contrast": 0.0, "dark_class": 0,
                       "light_class": 0, "runs": (), "repairs": 0,
                       "gap_rejects": 0, "interior_rejects": 0,
                       "candidate_count": 0, "candidate": None}
        else:
            summary = diagnose_profile(
                y, pixels, params, mode, scene_threshold,
                boundary_guard_enabled)
        _stamp_scene(summary, scene, boundary_guard_enabled, params)
        summaries.append(summary)
        if summary["reason"] == "MULTIPLE_PAIRS":
            ambiguous += 1
        if summary["candidate"] is not None:
            left_inner, right_inner, center, gap, dark_count = \
                summary["candidate"]
            candidates.append((y, left_inner, right_inner, center,
                               summary["contrast"], summary["threshold"],
                               gap, dark_count))

    if boundary_guard_enabled and scene["reason"] != "OK":
        return _failure(scene["reason"], summaries, [],
                        ambiguous=ambiguous, model=model)

    min_support = params["min_valid_rows"]
    if len(candidates) < min_support:
        return _failure("TOO_FEW_ROWS", summaries, candidates,
                        support=len(candidates), ambiguous=ambiguous,
                        model=model)
    if model == "quadratic":
        return _quadratic_model(
            candidates, summaries, ambiguous, image_width, params)

    center_points = [(item[0], item[3]) for item in candidates]
    active = center_points[:]
    slope = 0.0
    intercept = 0.0
    for _ in range(3):
        slope, intercept = _theil_sen(active)
        next_active = [point for point in center_points
                       if abs(point[1] -
                              (slope * point[0] + intercept)) <=
                       params["max_fit_residual_px"]]
        if len(next_active) < min_support:
            return _failure("FIT_SUPPORT", summaries, candidates,
                            support=len(next_active), ambiguous=ambiguous,
                            slope=slope, intercept=intercept)
        if next_active == active:
            active = next_active
            break
        active = next_active

    slope, intercept = _theil_sen(active)
    active_set = set(point[0] for point in active)
    inliers = [item for item in candidates if item[0] in active_set]
    residuals = [abs(item[3] - (slope * item[0] + intercept))
                 for item in inliers]
    fit_residual = max(residuals) if residuals else 0.0
    if fit_residual > params["max_fit_residual_px"]:
        return _failure("FIT_RESIDUAL", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual)

    y_values = [item[0] for item in inliers]
    y_min = min(y_values)
    y_max = max(y_values)
    if y_max - y_min < params["min_fit_y_span_px"]:
        return _failure("FIT_Y_SPAN", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual)

    reference_y = params["reference_y"]
    if reference_y < y_min or reference_y > y_max:
        return _failure("REFERENCE_UNCOVERED", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual)

    reference_center, local_residual, local_method = _local_reference_center(
        [(item[0], item[3]) for item in inliers], reference_y,
        params["local_fit_max_points"], params["local_fit_min_points"])
    if reference_center is None:
        return _failure("REFERENCE_UNCOVERED", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual)
    if local_residual > params["max_local_fit_residual_px"]:
        return _failure("LOCAL_RESIDUAL", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual,
                        local_residual=local_residual,
                        local_method=local_method)
    if reference_center < 0 or reference_center > image_width - 1:
        return _failure("CENTER_OUT_OF_RANGE", summaries, candidates,
                        support=len(inliers), ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual,
                        local_residual=local_residual,
                        local_method=local_method)

    support = len(inliers)
    min_contrast = min(item[4] for item in inliers)
    confidence = _confidence(
        support, len(summaries), max(fit_residual, local_residual),
        min_contrast, params)
    if confidence < params["min_confidence"]:
        return _failure("LOW_QUALITY", summaries, candidates,
                        support=support, ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual,
                        local_residual=local_residual,
                        local_method=local_method,
                        confidence=confidence)

    edge_jump = _edge_jump(inliers, params)
    if edge_jump is not None:
        return _failure("EDGE_JUMP", summaries, candidates,
                        support=support, ambiguous=ambiguous,
                        slope=slope, intercept=intercept,
                        residual=fit_residual,
                        local_residual=local_residual,
                        local_method=local_method, model="linear",
                        edge_jump=edge_jump)

    points = [(item[0], item[1], item[2], item[3]) for item in inliers]
    details = {
        "reason": "OK",
        "threshold_min": min(item["threshold"] for item in summaries
                              if item["threshold"] is not None),
        "threshold_max": max(item["threshold"] for item in summaries
                              if item["threshold"] is not None),
        "contrast_min": min_contrast,
        "contrast_avg": sum(item[4] for item in inliers) /
        float(support),
        "fit_residual": fit_residual,
        "local_residual": local_residual,
        "slope": slope,
        "intercept": intercept,
        "local_method": local_method,
        "model": "linear",
        "curvature": 0.0,
        "coefficients": None,
        "candidate_rows": len(candidates),
        "confidence": confidence,
        "rows": summaries,
    }
    for key in ("scene_threshold", "scene_otsu_threshold",
                "scene_contrast", "scene_dark_class",
                "scene_light_class", "scene_dark_ratio",
                "scene_light_ratio", "scene_reason",
                "boundary_guard_enabled", "scene_threshold_cap"):
        if key in summaries[0]:
            details[key] = summaries[0][key]
    return 1, reference_center, confidence, support, ambiguous, points, details


def normalize_lateral_error(target_x, image_width):
    if image_width < 2:
        raise ValueError("image_width must be at least 2")
    if target_x < 0 or target_x > image_width - 1:
        raise ValueError("target_x is outside the normalized image")
    center = (image_width - 1) * 0.5
    scaled = (float(target_x) - center) * 1000.0 / center
    if scaled >= 0.0:
        return int(scaled + 0.5)
    return -int(-scaled + 0.5)
