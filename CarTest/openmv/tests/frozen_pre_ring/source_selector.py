"""Same-frame comparison for independent grayscale road candidates."""

RUNTIME_API_VERSION = 6

import road_detector


def _value(settings, name, default):
    if isinstance(settings, dict):
        return settings.get(name, default)
    return getattr(settings, name, default)


def _preferred_source(settings):
    """Read and strictly validate the both-valid diagnostic preference."""
    preference = _value(
        settings, "GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID", "road")
    if not isinstance(preference, str) or preference not in ("road", "surface"):
        raise ValueError(
            "GRAY_PREFERRED_SOURCE_WHEN_BOTH_VALID must be 'road' or 'surface'")
    return preference


def _error(center_x, width):
    scaled = ((center_x - (width - 1) * 0.5) /
              (width * 0.5)) * 1000.0
    if scaled >= 0.0:
        return int(scaled + 0.5)
    return -int(-scaled + 0.5)


def _local_support(result, target_y, half_window=4.0):
    points = result.get("sections", ())
    fresh = 0
    for point in points:
        if abs(point["y"] - target_y) <= half_window:
            fresh += 1
    return min(1.0, fresh / 3.0)


def _road_support(result, target_y, half_window=4.0):
    fresh = 0
    for point in result.get("selected_rows", ()):
        if (point.get("provenance") == "OBSERVED" and
                point.get("left_status") == "OBSERVED" and
                point.get("right_status") == "OBSERVED" and
                abs(point["y"] - target_y) <= half_window):
            fresh += 1
    return min(1.0, fresh / 5.0)


def _invalid(reason, surface, road, comparison=None):
    return {
        "valid": False,
        "quality": 0,
        "lateral_error": 0,
        "reason": reason,
        "source": "NONE",
        "result": None,
        "surface": surface,
        "road": road,
        "comparison": comparison,
    }


def _surface_topology_ambiguity(result):
    """Return explicit current-frame topology evidence, if any.

    A geometric failure such as ROI clipping or low quality can still permit
    a separately valid paired-edge road fallback. An explicit surface
    ambiguity, internal island, or branch is different: it means the image
    contains competing topology and the road-only answer is unsafe to select.
    """
    if not result:
        return None
    reason = result.get("reason")
    island = bool(result.get("island_evidence"))
    branch = bool(result.get("branch_evidence"))
    if reason != "AMBIGUOUS" and not island and not branch:
        return None
    return {
        "surface_reason": reason,
        "island_evidence": island,
        "branch_evidence": branch,
    }


def select_candidate(surface_result, road_result, settings=None):
    """Select one geometry-gated candidate without blending control errors.

    If both candidates are valid, compare their centers at the surface
    candidate's target y. The road center may be interpolated only between
    two fresh observed rows within a bounded vertical gap; that interpolated
    comparison point is not added to road support. If they agree within the
    configured tolerance, prefer the road's paired-row observation by default;
    an explicit surface preference is a diagnostic counterfactual. Local and
    paired support counts remain diagnostic only. No common y means no
    consistency claim and always uses the road's own fixed/weighted reference;
    the both-valid preference applies only when a common y was compared.
    """
    surface_valid = bool(surface_result and surface_result.get("valid"))
    road_valid = bool(road_result and road_result.get("valid"))
    width = int(_value(settings, "SURFACE_GRID_WIDTH", 80))
    agreement_limit = float(_value(
        settings, "GRAY_SOURCE_AGREEMENT_CELLS", 6.0))
    max_gap = float(_value(
        settings, "GRAY_SHARED_REFERENCE_MAX_GAP_CELLS", 4.0))
    preferred_source = _preferred_source(settings)
    # Kept as a read-only compatibility diagnostic. This legacy setting must
    # not let local section counts or an old surface default displace paired
    # observed road rows.
    legacy_tiebreak = _value(settings, "GRAY_SELECTION_TIEBREAK", None)

    if not surface_valid and not road_valid:
        return _invalid("NO_VALID_SOURCE", surface_result, road_result)
    if surface_valid and not road_valid:
        target = surface_result.get("target")
        if target is None:
            return _invalid("SURFACE_TARGET_MISSING", surface_result,
                            road_result)
        return {
            "valid": True,
            "quality": surface_result.get("quality", 0),
            "lateral_error": _error(target[0], width),
            "reason": "SINGLE_VALID_SURFACE",
            "source": "surface",
            "result": surface_result,
            "surface": surface_result,
            "road": road_result,
            "comparison": None,
        }
    if road_valid and not surface_valid:
        topology = _surface_topology_ambiguity(surface_result)
        if topology is not None:
            return _invalid("SURFACE_TOPOLOGY_AMBIGUOUS", surface_result,
                            road_result, topology)
        reference = road_result.get("reference_center_x")
        if reference is not None:
            error = _error(reference, width)
        else:
            error = road_result.get("lateral_error", 0)
        return {
            "valid": True,
            "quality": road_result.get("quality", 0),
            "lateral_error": error,
            "reason": "SINGLE_VALID_ROAD",
            "source": "road",
            "result": road_result,
            "surface": surface_result,
            "road": road_result,
            "comparison": None,
        }

    surface_target = surface_result.get("target")
    if surface_target is None:
        return _invalid("SURFACE_TARGET_MISSING", surface_result,
                        road_result)
    target_x, target_y = surface_target
    road_center, provenance = road_detector.center_at_y(
        road_result.get("selected_rows", ()), target_y, max_gap)
    if road_center is None:
        # The preference flag compares sources only when they share a usable
        # y. With no common reference there is no consistency result, so the
        # road candidate must retain its own fixed/weighted reference even
        # when surface is requested for an explicit shared-y counterfactual.
        chosen_source = "road"
        chosen = road_result
        reference = road_result.get("reference_center_x")
        error = (_error(reference, width) if reference is not None else
                 road_result.get("lateral_error", 0))
        return {
            "valid": True,
            "quality": chosen.get("quality", 0),
            "lateral_error": error,
            "reason": "NO_COMMON_REFERENCE",
            "source": chosen_source,
            "result": chosen,
            "surface": surface_result,
            "road": road_result,
            "comparison": {
                "target_y": target_y,
                "road_center_x": None,
                "road_provenance": provenance,
                "consistency_checked": False,
                "consistency": "NOT_CHECKED_NO_COMMON_REFERENCE",
                "both_valid_common_reference_preference": preferred_source,
                "legacy_tiebreak_ignored": legacy_tiebreak,
            },
        }

    difference = abs(road_center - target_x)
    comparison = {
        "target_y": target_y,
        "surface_center_x": target_x,
        "road_center_x": road_center,
        "road_provenance": provenance,
        "difference_cells": difference,
        "tolerance_cells": agreement_limit,
    }
    if difference > agreement_limit:
        return _invalid("AMBIGUOUS_SOURCE", surface_result, road_result,
                        comparison)

    surface_support = _local_support(surface_result, target_y)
    road_support = _road_support(road_result, target_y)
    chosen_source = preferred_source
    chosen = surface_result if chosen_source == "surface" else road_result
    chosen_center = target_x if chosen_source == "surface" else road_center
    return {
        "valid": True,
        "quality": chosen.get("quality", 0),
        "lateral_error": _error(chosen_center, width),
        "reason": ("PAIRED_ROW_PREFERRED" if chosen_source == "road" else
                   "SURFACE_PREFERRED"),
        "source": chosen_source,
        "result": chosen,
        "surface": surface_result,
        "road": road_result,
        "comparison": comparison,
        "surface_support": surface_support,
        "road_support": road_support,
        "support_stats_role": "DIAGNOSTIC_ONLY_NOT_USED_FOR_SELECTION",
        "selection_preference": preferred_source,
        "legacy_tiebreak_ignored": legacy_tiebreak,
    }
