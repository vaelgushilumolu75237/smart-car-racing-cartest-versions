"""Small MicroPython-compatible two-boundary detector and pure helpers."""


def _median(values):
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return 0.0
    middle = count // 2
    if count & 1:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def _resolve_mode(params, mode):
    if mode is not None:
        return mode
    return params.get("detection_mode", "black_edges")


def _black_runs(pixels, threshold, width_min, width_max):
    runs = []
    start = -1
    width = len(pixels)
    for x in range(width + 1):
        black = x < width and pixels[x] <= threshold
        if black and start < 0:
            start = x
        elif not black and start >= 0:
            run_width = x - start
            if width_min <= run_width <= width_max:
                runs.append((start, x - 1, run_width))
            start = -1
    return runs


def _matching_runs(raw_runs, params, mode):
    if mode == "black_edges":
        return [
            run for run in raw_runs
            if params["boundary_width_min_px"] <= run[2] <=
            params["boundary_width_max_px"]
        ]
    if mode == "white_corridor":
        minimum = params["white_corridor_dark_min_px"]
        return [run for run in raw_runs if run[2] >= minimum]
    return []


def _pair_for_row(pixels, params, mode=None):
    mode = _resolve_mode(params, mode)
    if mode not in ("black_edges", "white_corridor"):
        return None, 0
    raw_runs = _all_black_runs(pixels, params["black_threshold"])
    runs = _matching_runs(raw_runs, params, mode)
    pairs = _find_pairs(pixels, runs, params, mode=mode)
    if len(pairs) == 1:
        return pairs[0], 0
    return None, 1 if len(pairs) > 1 else 0


def _find_pairs(pixels, runs, params, details=None, mode="black_edges"):
    """Apply the detector's unchanged pair checks; optionally collect stats."""
    pairs = []
    for left_index in range(len(runs)):
        left = runs[left_index]
        for right_index in range(left_index + 1, len(runs)):
            right = runs[right_index]
            gap = right[0] - left[1] - 1
            width_delta = abs(left[2] - right[2])
            gap_ok = (params["track_gap_min_px"] <= gap <=
                      params["track_gap_max_px"])
            width_ok = (mode != "black_edges" or
                        width_delta <=
                        params["boundary_width_tolerance_px"])
            gap_black_count = 0
            for x in range(left[1] + 1, right[0]):
                if pixels[x] <= params["black_threshold"]:
                    gap_black_count += 1
            interior_ok = (
                gap_black_count <= params["max_gap_black_pixels"] and
                gap_black_count * 100 <=
                gap * params["max_gap_black_ratio_percent"])
            if gap_ok and width_ok and interior_ok:
                if mode == "white_corridor":
                    # Use the inward-facing edges; dark area widths may vary.
                    left_edge = float(left[1])
                    right_edge = float(right[0])
                else:
                    left_edge = (left[0] + left[1]) * 0.5
                    right_edge = (right[0] + right[1]) * 0.5
                pairs.append((left_edge, right_edge))
                outcome = "OK"
            else:
                outcome = "REJECTED"
                if details is not None:
                    reasons = []
                    if not gap_ok:
                        reasons.append("GAP_RANGE")
                        details["rejected"]["GAP_RANGE"] += 1
                    if not width_ok:
                        reasons.append("WIDTH_DIFFERENCE")
                        details["rejected"]["WIDTH_DIFFERENCE"] += 1
                    if not interior_ok:
                        reasons.append("INTERIOR_BLACK")
                        details["rejected"]["INTERIOR_BLACK"] += 1
                    outcome = "+".join(reasons)

            if details is not None:
                details["tested_pairs"] += 1
                if outcome == "OK":
                    details["accepted_pairs"] += 1
                if len(details["pair_samples"]) < details["sample_limit"]:
                    details["pair_samples"].append((
                        left[0], left[2], gap, right[0], right[2], outcome))
                else:
                    details["pair_samples_omitted"] += 1
    return pairs


def _all_black_runs(pixels, threshold):
    """Return every black run, including widths excluded by the detector."""
    runs = []
    start = -1
    width = len(pixels)
    for x in range(width + 1):
        black = x < width and pixels[x] <= threshold
        if black and start < 0:
            start = x
        elif not black and start >= 0:
            runs.append((start, x - 1, x - start))
            start = -1
    return runs


def diagnose_row(y, pixels, params, mode=None):
    """Summarize one raw scanline without changing detection or thresholds."""
    mode = _resolve_mode(params, mode)
    ordered = sorted(pixels)
    count = len(ordered)
    if count == 0:
        gray_min = None
        p10 = None
        p90 = None
        gray_max = None
    else:
        gray_min = ordered[0]
        p10 = ordered[((count - 1) * 10) // 100]
        p90 = ordered[((count - 1) * 90) // 100]
        gray_max = ordered[-1]

    raw_runs = _all_black_runs(pixels, params["black_threshold"])
    matching_runs = _matching_runs(raw_runs, params, mode)
    details = {
        "tested_pairs": 0,
        "accepted_pairs": 0,
        "rejected": {
            "GAP_RANGE": 0,
            "WIDTH_DIFFERENCE": 0,
            "INTERIOR_BLACK": 0,
        },
        "pair_samples": [],
        "pair_samples_omitted": 0,
        "sample_limit": 3,
    }
    if mode in ("black_edges", "white_corridor"):
        pairs = _find_pairs(pixels, matching_runs, params, details, mode)
    else:
        pairs = []

    if mode not in ("black_edges", "white_corridor"):
        reason = "CONFIG_MODE"
    elif not raw_runs:
        reason = "NO_DARK" if mode == "white_corridor" else "NO_BLACK"
    elif not matching_runs:
        reason = "NO_WIDTH_MATCH"
    elif len(matching_runs) == 1:
        reason = "ONE_EDGE"
    elif len(pairs) == 1:
        reason = "OK"
    elif len(pairs) > 1:
        reason = "MULTIPLE_PAIRS"
    else:
        reason = "GAP_RANGE"
        for candidate_reason in ("GAP_RANGE", "WIDTH_DIFFERENCE",
                                 "INTERIOR_BLACK"):
            if details["rejected"][candidate_reason]:
                reason = candidate_reason
                break

    raw_width_limit = 8
    return {
        "y": y,
        "mode": mode,
        "gray": (gray_min, p10, p90, gray_max),
        "raw_run_count": len(raw_runs),
        "raw_widths": tuple(run[2] for run in raw_runs[:raw_width_limit]),
        "raw_widths_omitted": max(0, len(raw_runs) - raw_width_limit),
        "width_match_count": len(matching_runs),
        "tested_pairs": details["tested_pairs"],
        "accepted_pairs": len(pairs),
        "rejected": details["rejected"],
        "pair_samples": tuple(details["pair_samples"]),
        "pair_samples_omitted": details["pair_samples_omitted"],
        "reason": reason,
    }


def format_row_diagnostic(summary):
    """Create one bounded console line for a diagnosed scan row."""
    widths = ",".join(str(value) for value in summary["raw_widths"])
    if not widths:
        widths = "-"
    if summary["raw_widths_omitted"]:
        widths += ",+%d" % summary["raw_widths_omitted"]

    gaps = []
    for left_x, left_width, gap, right_x, right_width, result in \
            summary["pair_samples"]:
        gaps.append("%d/%d-%d:%d/%d=%s" % (
            left_x, left_width, gap, right_x, right_width, result))
    if summary["pair_samples_omitted"]:
        gaps.append("+%d" % summary["pair_samples_omitted"])
    if not gaps:
        gaps_text = "-"
    else:
        gaps_text = ";".join(gaps)

    rejected = []
    for reason in ("GAP_RANGE", "WIDTH_DIFFERENCE", "INTERIOR_BLACK"):
        value = summary["rejected"][reason]
        if value:
            rejected.append("%s:%d" % (reason, value))
    rejected_text = ",".join(rejected) if rejected else "none"
    gray = summary["gray"]
    gray_text = "-" if gray[0] is None else "%d/%d/%d/%d" % gray
    return ("VDBG y=%d gray=%s raw=%d[w=%s] match=%d pairs=%d "
            "gap=%s why=%s reject=%s" % (
                summary["y"], gray_text, summary["raw_run_count"], widths,
                summary["width_match_count"], summary["accepted_pairs"],
                gaps_text, summary["reason"], rejected_text))


def summarize_diagnostics(summaries, detector_result=None,
                          min_valid_rows=None):
    """Return the dominant row reason, or MIXED when row reasons tie."""
    if not summaries:
        return "WAIT"
    if (detector_result is not None and min_valid_rows is not None and
            detector_result[0] == 0 and
            detector_result[3] < min_valid_rows and
            all(summary["reason"] == "OK" for summary in summaries)):
        return "FEWROWS"
    counts = {}
    for summary in summaries:
        reason = summary["reason"]
        counts[reason] = counts.get(reason, 0) + 1
    maximum = max(counts.values())
    dominant = [reason for reason in counts if counts[reason] == maximum]
    return dominant[0] if len(dominant) == 1 else "MIXED"


def detect_track_center(rows, image_width, params, mode=None):
    """Return (valid, center_x, confidence, support_count, ambiguous_count, pairs).

    rows is an iterable of (y, one-dimensional grayscale pixel sequence).
    pairs contains (y, left_boundary_center, right_boundary_center) for the
    rows that survive multi-row consistency filtering.
    """
    mode = _resolve_mode(params, mode)
    if mode not in ("black_edges", "white_corridor"):
        return 0, 0.0, 0, 0, 0, []

    candidates = []
    ambiguous_count = 0
    for y, pixels in rows:
        if len(pixels) != image_width:
            continue
        pair, ambiguous = _pair_for_row(pixels, params, mode)
        ambiguous_count += ambiguous
        if pair is not None:
            left_center, right_center = pair
            candidates.append((y, left_center, right_center,
                               (left_center + right_center) * 0.5))

    if not candidates or ambiguous_count != 0:
        return 0, 0.0, 0, 0, ambiguous_count, []

    median_center = _median([item[3] for item in candidates])
    consistent = [
        item for item in candidates
        if abs(item[3] - median_center) <=
        params["max_center_deviation_px"]
    ]
    support = len(consistent)
    if support < params["min_valid_rows"]:
        return 0, 0.0, 0, support, ambiguous_count, []

    weighted_sum = 0.0
    total_weight = 0.0
    for item in consistent:
        # Lower rows are closer to the vehicle and receive greater weight.
        weight = 1.0 + float(item[0]) / float(params["image_height"])
        weighted_sum += item[3] * weight
        total_weight += weight
    center = weighted_sum / total_weight
    confidence = int((support * 100 + len(rows) // 2) // len(rows))
    return 1, center, confidence, support, ambiguous_count, [
        (item[0], item[1], item[2]) for item in consistent
    ]


def normalize_lateral_error(target_x, image_width):
    """Map x to protocol half-width thousandths, with half values away from 0."""
    if image_width < 2:
        raise ValueError("image_width must be at least 2")
    if target_x < 0 or target_x > image_width - 1:
        raise ValueError("target_x is outside the normalized image")
    center = (image_width - 1) * 0.5
    half_width = center
    scaled = (float(target_x) - center) * 1000.0 / half_width
    if scaled >= 0.0:
        result = int(scaled + 0.5)
    else:
        result = -int(-scaled + 0.5)
    return result

