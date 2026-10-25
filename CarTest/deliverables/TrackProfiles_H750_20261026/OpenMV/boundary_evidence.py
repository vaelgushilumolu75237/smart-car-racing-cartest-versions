"""Local raw-grayscale step evidence for detected bilateral road edges.

This module is intentionally small and MicroPython-compatible. It only checks
same-frame pixels near already selected geometric boundaries; it does not
change threshold masks, infer clipped edges, or claim material semantics.
"""

RUNTIME_API_VERSION = 6

import math


_DEFAULTS = {
    "GRAY_BOUNDARY_MIN_STEP": 10.0,
    "GRAY_BOUNDARY_MIN_STEP_FRACTION": 0.25,
    "GRAY_BOUNDARY_SUPPORT_RATIO": 0.75,
    "GRAY_BOUNDARY_TARGET_RADIUS_CELLS": 5.0,
    "GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT": 1,
    "GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT": 2,
}


def _value(settings, name, default=None):
    fallback = _DEFAULTS.get(name, default)
    if isinstance(settings, dict):
        return settings.get(name, fallback)
    return getattr(settings, name, fallback)


def _round_cell(value):
    if value >= 0.0:
        return int(value + 0.5)
    return int(value - 0.5)


def _index(x, y, width, height, known_region):
    # Reject floating-point coordinates outside the image before rounding so
    # a slightly negative ray cannot masquerade as the left image-edge pixel.
    if x < 0.0 or x > width - 1 or y < 0.0 or y > height - 1:
        return None
    ix = _round_cell(x)
    iy = _round_cell(y)
    if ix < 0 or ix >= width or iy < 0 or iy >= height:
        return None
    index = iy * width + ix
    if not known_region[index]:
        return None
    return index


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def _measure_side_details(hit, center, luma, dark_mask, white_mask,
                          known_region, width, height):
    if (not isinstance(hit, (tuple, list)) or len(hit) != 2 or
            not isinstance(center, (tuple, list)) or len(center) != 2):
        return None
    hit_x, hit_y = float(hit[0]), float(hit[1])
    center_x, center_y = float(center[0]), float(center[1])
    hit_index = _index(hit_x, hit_y, width, height, known_region)
    if hit_index is None or not dark_mask[hit_index]:
        return None

    dx = center_x - hit_x
    dy = center_y - hit_y
    length = math.sqrt(dx * dx + dy * dy)
    if length <= 0.0:
        return None
    unit_x = dx / length
    unit_y = dy / length
    sample_values = []
    seen = {}
    for distance in (1.0, 2.0, 3.0):
        sample_x = hit_x + unit_x * distance
        sample_y = hit_y + unit_y * distance
        sample_index = _index(sample_x, sample_y, width, height,
                              known_region)
        if sample_index is None or not white_mask[sample_index]:
            continue
        if sample_index in seen:
            continue
        seen[sample_index] = True
        sample_values.append(luma[sample_index])
    if len(sample_values) < 2:
        return None

    # Median only the known white samples between this hit and its own section
    # center. Never average the exterior side of a narrow black line.
    white_median = _median(sample_values)
    dark_luma = luma[hit_index]
    step = white_median - dark_luma
    return {
        "step": step,
        "dark_luma": dark_luma,
        "white_median": white_median,
        "sample_count": len(sample_values),
        "step_fraction": step / max(1.0, float(white_median)),
    }


def _measure_side(hit, center, luma, dark_mask, white_mask, known_region,
                  width, height):
    """Compatibility scalar API; detailed users should sample only once."""
    details = _measure_side_details(
        hit, center, luma, dark_mask, white_mask, known_region, width, height)
    return None if details is None else details["step"]


def _side_supported(details, minimum_step, minimum_fraction):
    return (details is not None and details["step"] >= minimum_step and
            details["step_fraction"] >= minimum_fraction)


def _stats(source, status, reason, minimum_step, minimum_fraction=0.25,
           pair_count=0,
           bilateral=0, left_values=(), right_values=(), target_support=0,
           support_points=(), left_fractions=(), right_fractions=()):
    ratio = (float(bilateral) / pair_count) if pair_count else 0.0
    return {
        "source": source,
        "status": status,
        "reason": reason,
        "pairs_tested": pair_count,
        "bilateral_supported": bilateral,
        "left_supported": sum(
            1 for value, fraction in zip(left_values, left_fractions)
            if value >= minimum_step and fraction >= minimum_fraction),
        "right_supported": sum(
            1 for value, fraction in zip(right_values, right_fractions)
            if value >= minimum_step and fraction >= minimum_fraction),
        "support_ratio": ratio,
        "target_support_count": target_support,
        "minimum_step": minimum_step,
        "minimum_step_fraction": minimum_fraction,
        "left_step_median": _median(list(left_values)),
        "right_step_median": _median(list(right_values)),
        "left_step_fraction_median": _median(list(left_fractions)),
        "right_step_fraction_median": _median(list(right_fractions)),
        # These are references to already-counted fresh observations. The
        # post-selector check may inspect them but never adds support.
        "support_points": list(support_points),
    }


def _invalidate(candidate, source, reason, evidence,
                result_reason="BOUNDARY_EVIDENCE_REJECTED"):
    if "geometry_reason" not in candidate:
        candidate["geometry_reason"] = candidate.get("reason")
    candidate["boundary_evidence"] = evidence
    candidate["valid"] = False
    candidate["quality"] = 0
    candidate["reason"] = result_reason
    if source == "surface":
        candidate["target"] = None
    else:
        candidate["lateral_error"] = 0
        candidate["center_x"] = None
        candidate["reference_center_x"] = None
        candidate["reference_lateral_error"] = 0


def _prepare(candidate, luma, dark_mask, white_mask, known_region, settings,
             source):
    if not candidate or not candidate.get("valid"):
        return None, None
    minimum_step = float(_value(settings, "GRAY_BOUNDARY_MIN_STEP"))
    minimum_fraction = float(_value(
        settings, "GRAY_BOUNDARY_MIN_STEP_FRACTION"))
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    total = width * height
    reason = None
    if (luma is None or dark_mask is None or white_mask is None or
            known_region is None):
        reason = "MISSING_FRAME_MASK"
    elif (len(luma) != total or len(dark_mask) != total or
          len(white_mask) != total or len(known_region) != total):
        reason = "FRAME_SIZE_MISMATCH"
    if reason is not None:
        evidence = _stats(source, "REJECTED", reason, minimum_step,
                          minimum_fraction)
        _invalidate(candidate, source, reason, evidence)
        return None, None
    return (width, height, minimum_step, minimum_fraction), None


def _finish(candidate, source, pair_count, bilateral, left_values,
            right_values, target_support, min_pairs, min_target_support,
            support_points, settings, left_fractions=(), right_fractions=()):
    minimum_step = float(_value(settings, "GRAY_BOUNDARY_MIN_STEP"))
    minimum_fraction = float(_value(
        settings, "GRAY_BOUNDARY_MIN_STEP_FRACTION"))
    minimum_ratio = float(_value(settings, "GRAY_BOUNDARY_SUPPORT_RATIO"))
    ratio = (float(bilateral) / pair_count) if pair_count else 0.0
    if pair_count < min_pairs:
        reason = "INSUFFICIENT_FRESH_SUPPORT"
    elif ratio < minimum_ratio:
        reason = "LOW_PAIR_SUPPORT"
    elif target_support < min_target_support:
        reason = "NO_TARGET_SUPPORT"
    else:
        reason = "OK"
    evidence = _stats(
        source, "PASS" if reason == "OK" else "REJECTED", reason,
        minimum_step, minimum_fraction, pair_count, bilateral, left_values,
        right_values, target_support, support_points, left_fractions,
        right_fractions)
    if reason == "OK":
        candidate["boundary_evidence"] = evidence
    else:
        _invalidate(candidate, source, reason, evidence)
    return candidate


def _target_near(target, point, radius):
    dx = target[0] - point[0]
    dy = target[1] - point[1]
    return dx * dx + dy * dy <= radius * radius


def _road_preselection_reference(candidate):
    reference_x = candidate.get("reference_center_x")
    reference_y = candidate.get("reference_y")
    if reference_x is not None and reference_y is not None:
        return (float(reference_x), float(reference_y), "reference_y")

    # Match main._selected_reference's weighted-center fallback exactly.
    points = candidate.get("selected_rows") or ()
    if not points:
        return None
    weights = candidate.get("weights") or ()
    if len(weights) != len(points):
        weights = [1.0 / len(points)] * len(points)
    center_y = 0.0
    for point, weight in zip(points, weights):
        value = float(weight)
        center_y += float(point["y"]) * value
    center_x = candidate.get("center_x")
    if center_x is None:
        return None
    return (float(center_x), center_y, "weighted_center")


def gate_surface_candidate(candidate, luma, edge_mask, road_mask,
                           known_region, settings):
    """Require local gray steps at both hits on fresh raycast sections."""
    prepared, _unused = _prepare(candidate, luma, edge_mask, road_mask,
                                 known_region, settings, "surface")
    if prepared is None:
        # Geometry failures, especially explicit AMBIGUOUS topology, remain
        # untouched; only previously valid candidates are photometrically gated.
        return candidate
    width, height, minimum_step, minimum_fraction = prepared
    sections = candidate.get("sections") or ()
    fresh = sections[1:]  # Seed is initialization, not fresh support.
    pair_count = len(fresh)
    left_values = []
    right_values = []
    left_fractions = []
    right_fractions = []
    bilateral = 0
    target_support = 0
    support_points = []
    target = candidate.get("target")
    radius = float(_value(settings,
                          "GRAY_BOUNDARY_TARGET_RADIUS_CELLS"))
    for section in fresh:
        center = (section.get("x"), section.get("y"))
        left_details = _measure_side_details(
            section.get("left"), center, luma, edge_mask, road_mask,
            known_region, width, height)
        right_details = _measure_side_details(
            section.get("right"), center, luma, edge_mask, road_mask,
            known_region, width, height)
        left_ok = _side_supported(left_details, minimum_step,
                                  minimum_fraction)
        right_ok = _side_supported(right_details, minimum_step,
                                   minimum_fraction)
        if left_details is not None:
            left_values.append(left_details["step"])
            left_fractions.append(left_details["step_fraction"])
        if right_details is not None:
            right_values.append(right_details["step"])
            right_fractions.append(right_details["step_fraction"])
        if left_ok and right_ok:
            bilateral += 1
            support_points.append((center[0], center[1]))
            if (isinstance(target, (tuple, list)) and len(target) == 2 and
                    _target_near(target, (center[0], center[1]), radius)):
                target_support += 1
    return _finish(
        candidate, "surface", pair_count, bilateral, left_values,
        right_values, target_support, 3,
        int(_value(settings,
                   "GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT")),
        support_points, settings, left_fractions, right_fractions)


def gate_road_candidate(candidate, luma, edge_mask, road_mask,
                        known_region, settings):
    """Require raw dark/white steps on each fresh paired road row."""
    prepared, _unused = _prepare(candidate, luma, edge_mask, road_mask,
                                 known_region, settings, "road")
    if prepared is None:
        return candidate
    width, height, minimum_step, minimum_fraction = prepared
    rows = candidate.get("selected_rows") or ()
    pair_count = len(rows)
    left_values = []
    right_values = []
    left_fractions = []
    right_fractions = []
    bilateral = 0
    target_support = 0
    support_points = []
    target = _road_preselection_reference(candidate)
    radius = float(_value(settings,
                          "GRAY_BOUNDARY_TARGET_RADIUS_CELLS"))
    for row in rows:
        y = row.get("y")
        left = row.get("left")
        right = row.get("right")
        if (left is None or right is None or y is None or
                row.get("provenance") != "OBSERVED" or
                row.get("left_status") != "OBSERVED" or
                row.get("right_status") != "OBSERVED"):
            continue
        center = ((float(left) + float(right)) * 0.5, float(y))
        # Road runs are inclusive bright pixels; adjacent dark hits are
        # left-1/right+1. No image-edge or unknown-region hit is synthesized.
        left_hit = (float(left) - 1.0, float(y))
        right_hit = (float(right) + 1.0, float(y))
        left_details = _measure_side_details(
            left_hit, center, luma, edge_mask, road_mask, known_region,
            width, height)
        right_details = _measure_side_details(
            right_hit, center, luma, edge_mask, road_mask, known_region,
            width, height)
        left_ok = _side_supported(left_details, minimum_step,
                                  minimum_fraction)
        right_ok = _side_supported(right_details, minimum_step,
                                   minimum_fraction)
        if left_details is not None:
            left_values.append(left_details["step"])
            left_fractions.append(left_details["step_fraction"])
        if right_details is not None:
            right_values.append(right_details["step"])
            right_fractions.append(right_details["step_fraction"])
        if left_ok and right_ok:
            bilateral += 1
            point = (center[0], center[1])
            support_points.append(point)
            if (target is not None and
                    _target_near((target[0], target[1]), point, radius)):
                target_support += 1
    return _finish(
        candidate, "road", pair_count, bilateral, left_values,
        right_values, target_support, 4,
        int(_value(settings,
                   "GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT")),
        support_points, settings, left_fractions, right_fractions)


def validate_selected_reference(selection, reference, surface_result,
                                road_result, settings):
    """Recheck final controller point against existing fresh evidence only.

    The support point list is not expanded here. This catches source-selector
    choices such as a shared-y road center that differ from its pre-gate
    fixed/weighted reference.
    """
    if not selection or not selection.get("valid"):
        return {"valid": False, "reason": "SELECTION_INVALID",
                "support_count": 0}
    source = selection.get("source")
    candidate = (surface_result if source == "surface" else
                 road_result if source == "road" else None)
    if candidate is None or not candidate.get("valid"):
        return {"valid": False, "reason": "SOURCE_CANDIDATE_INVALID",
                "support_count": 0}
    if (not isinstance(reference, dict) or reference.get("x") is None or
            reference.get("y") is None):
        reason = "REFERENCE_UNAVAILABLE"
        evidence = candidate.get("boundary_evidence") or _stats(
            source, "REJECTED", reason,
            float(_value(settings, "GRAY_BOUNDARY_MIN_STEP")),
            float(_value(settings,
                         "GRAY_BOUNDARY_MIN_STEP_FRACTION")))
        evidence["status"] = "REJECTED"
        evidence["selected_reference_reason"] = reason
        _invalidate(candidate, source, reason, evidence,
                    "BOUNDARY_REFERENCE_UNSUPPORTED")
        return {"valid": False, "reason": reason, "support_count": 0}

    evidence = candidate.get("boundary_evidence")
    if not evidence or evidence.get("status") != "PASS":
        reason = "NO_PRESELECTION_EVIDENCE"
        if evidence is None:
            evidence = _stats(
                source, "REJECTED", reason,
                float(_value(settings, "GRAY_BOUNDARY_MIN_STEP")),
                float(_value(settings,
                             "GRAY_BOUNDARY_MIN_STEP_FRACTION")))
        _invalidate(candidate, source, reason, evidence,
                    "BOUNDARY_REFERENCE_UNSUPPORTED")
        return {"valid": False, "reason": reason, "support_count": 0}

    radius = float(_value(settings,
                          "GRAY_BOUNDARY_TARGET_RADIUS_CELLS"))
    target = (float(reference["x"]), float(reference["y"]))
    support_count = 0
    for point in evidence.get("support_points", ()):
        if _target_near(target, point, radius):
            support_count += 1
    if source == "surface":
        minimum = int(_value(
            settings, "GRAY_BOUNDARY_MIN_SURFACE_TARGET_SUPPORT"))
    else:
        minimum = int(_value(
            settings, "GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT"))
    if support_count < minimum:
        reason = "NO_SELECTED_REFERENCE_SUPPORT"
        evidence["preselection_status"] = evidence.get("status")
        evidence["status"] = "REJECTED"
        evidence["selected_reference_reason"] = reason
        evidence["selected_reference_support_count"] = support_count
        _invalidate(candidate, source, reason, evidence,
                    "BOUNDARY_REFERENCE_UNSUPPORTED")
        return {"valid": False, "reason": reason,
                "support_count": support_count}

    evidence["selected_reference_status"] = "PASS"
    evidence["selected_reference_support_count"] = support_count
    evidence["selected_reference"] = (
        target[0], target[1], reference.get("kind"))
    return {"valid": True, "reason": "OK",
            "support_count": support_count}


def gate_candidates(surface_result, road_result, luma, edge_mask,
                    road_mask, known_region, settings):
    """Apply same-frame local gray evidence before source selection."""
    gate_surface_candidate(surface_result, luma, edge_mask, road_mask,
                           known_region, settings)
    gate_road_candidate(road_result, luma, edge_mask, road_mask,
                        known_region, settings)
    return surface_result, road_result
