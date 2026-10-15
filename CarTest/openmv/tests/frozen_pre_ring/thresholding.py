"""Small, frame-local Otsu thresholding for OpenMV MicroPython.

The input is an 8-bit luminance plane.  Each ROI block receives its own
256-level Otsu scan; no scene-wide cap or cross-frame threshold state is used.
The returned mask uses 255 for the brighter class (road candidate) and 0 for
the darker class.  A block without two separable classes is marked untrusted
and left black in the mask so callers can fail closed.

Rectangles are half-open ``(x0, y0, x1, y1)`` coordinates.  ``blocks`` may be
an upper/middle/lower set supplied by the caller.  By default, no threshold
smoothing is applied; ``smooth_neighbors=True`` enables an explicit second
pass over trusted blocks that share an edge.
"""

RUNTIME_API_VERSION = 6


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _check_rect(rect, width, height, limit=None):
    if not isinstance(rect, (tuple, list)) or len(rect) != 4:
        raise ValueError("rectangle must be (x0, y0, x1, y1)")
    x0, y0, x1, y1 = rect
    if not (_is_int(x0) and _is_int(y0) and _is_int(x1) and _is_int(y1)):
        raise ValueError("rectangle coordinates must be integers")
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        raise ValueError("rectangle is outside the image")
    # A zero-area ROI/block is representable so empty regions become
    # diagnostics with reason EMPTY instead of an arithmetic exception.
    if x0 > x1 or y0 > y1:
        raise ValueError("rectangle coordinates must be ordered")
    if limit is not None:
        lx0, ly0, lx1, ly1 = limit
        if x0 < lx0 or y0 < ly0 or x1 > lx1 or y1 > ly1:
            raise ValueError("block rectangle is outside the ROI")
    return (x0, y0, x1, y1)


def _overlap_or_touch(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    if ax0 >= ax1 or ay0 >= ay1 or bx0 >= bx1 or by0 >= by1:
        return False
    vertical_touch = (ax1 == bx0 or bx1 == ax0) and min(ay1, by1) > max(ay0, by0)
    horizontal_touch = (ay1 == by0 or by1 == ay0) and min(ax1, bx1) > max(ax0, bx0)
    return vertical_touch or horizontal_touch


def _rectangles_overlap(a, b):
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


def _histogram(luma, width, rect):
    x0, y0, x1, y1 = rect
    histogram = [0] * 256
    count = 0
    value_sum = 0
    minimum = 256
    maximum = -1
    for y in range(y0, y1):
        offset = y * width
        for x in range(x0, x1):
            value = luma[offset + x]
            if not _is_int(value) or value < 0 or value > 255:
                raise ValueError("luma pixels must be integers in 0..255")
            histogram[value] += 1
            count += 1
            value_sum += value
            if value < minimum:
                minimum = value
            if value > maximum:
                maximum = value
    return histogram, count, value_sum, minimum, maximum


def _otsu(histogram, count, value_sum):
    """Return (threshold, low_count, high_count, low_mean, high_mean).

    The score is the between-class variance.  Every gray cutoff 0..255 is
    inspected; empty classes are skipped instead of divided by zero.
    """
    low_count = 0
    low_sum = 0
    best_score = -1.0
    best_threshold = None
    best_counts = (0, 0)
    best_means = (0.0, 0.0)
    scale = float(count * count)

    for threshold in range(256):
        amount = histogram[threshold]
        low_count += amount
        low_sum += threshold * amount
        high_count = count - low_count
        if low_count == 0 or high_count == 0:
            continue
        high_sum = value_sum - low_sum
        low_mean = low_sum / float(low_count)
        high_mean = high_sum / float(high_count)
        difference = high_mean - low_mean
        score = (low_count * high_count / scale) * difference * difference
        if score > best_score:
            best_score = score
            best_threshold = threshold
            best_counts = (low_count, high_count)
            best_means = (low_mean, high_mean)

    if best_threshold is None:
        return None, (count, 0), (0.0, 0.0), 0.0
    contrast = best_means[1] - best_means[0]
    return best_threshold, best_counts, best_means, contrast


def _class_stats(histogram, threshold):
    low_count = 0
    low_sum = 0
    high_count = 0
    high_sum = 0
    for value in range(256):
        amount = histogram[value]
        if value <= threshold:
            low_count += amount
            low_sum += value * amount
        else:
            high_count += amount
            high_sum += value * amount
    low_mean = low_sum / float(low_count) if low_count else 0.0
    high_mean = high_sum / float(high_count) if high_count else 0.0
    return (low_count, high_count), (low_mean, high_mean), high_mean - low_mean


def _median(values):
    ordered = sorted(values)
    size = len(ordered)
    middle = size // 2
    if size % 2:
        return ordered[middle]
    return int((ordered[middle - 1] + ordered[middle]) / 2.0 + 0.5)


def build_mask(luma, width=80, height=60, roi=None, blocks=None,
               min_contrast=0.0, smooth_neighbors=False):
    """Build a binary high-luminance candidate mask and block diagnostics.

    Args:
        luma: flat row-major bytes/bytearray (or integer sequence), 0..255.
        width, height: dimensions of ``luma``.
        roi: optional half-open image rectangle.
        blocks: optional list of non-overlapping half-open rectangles inside
            ``roi``.  A single ROI block is used when omitted.
        min_contrast: optional minimum measured high-minus-low class mean.
            Zero still requires two non-empty classes with positive contrast.
        smooth_neighbors: optional second pass replacing each trusted block's
            threshold with the median of itself and trusted edge-adjacent
            blocks, clamped so the block retains both classes.

    Returns:
        ``(mask, diagnostics)``.  The mask has ``width * height`` bytes, with
        white candidates 255 and all other/untrusted pixels 0.  Diagnostics
        include a ``blocks`` list in the input order; each item reports the
        effective threshold, raw threshold, contrast, ``(low, high)`` class
        counts, and trust/reason flags.  ``valid`` means at least one block is
        trusted; ``complete`` means every requested block is trusted.
    """
    if not _is_int(width) or not _is_int(height) or width <= 0 or height <= 0:
        raise ValueError("width and height must be positive integers")
    if len(luma) != width * height:
        raise ValueError("luma length must equal width * height")
    if not isinstance(min_contrast, (int, float)) or min_contrast < 0:
        raise ValueError("min_contrast must be a non-negative number")
    if min_contrast != min_contrast or min_contrast == float("inf"):
        raise ValueError("min_contrast must be finite")

    if roi is None:
        roi = (0, 0, width, height)
    roi = _check_rect(roi, width, height)
    if blocks is None:
        rects = [roi]
    else:
        if not isinstance(blocks, (tuple, list)) or len(blocks) == 0:
            raise ValueError("blocks must be a non-empty sequence")
        rects = [_check_rect(rect, width, height, roi) for rect in blocks]
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                if _rectangles_overlap(rects[i], rects[j]):
                    raise ValueError("block rectangles must not overlap")

    records = []
    for rect in rects:
        histogram, count, value_sum, minimum, maximum = _histogram(luma, width, rect)
        record = {
            "rect": rect,
            "pixel_count": count,
            "minimum": minimum,
            "maximum": maximum,
            "raw_threshold": None,
            "threshold": None,
            "contrast": 0.0,
            "classes": (count, 0),
            "class_means": (0.0, 0.0),
            "trusted": False,
            "reason": "EMPTY",
        }
        if smooth_neighbors:
            record["histogram"] = histogram
        if count:
            if minimum == maximum:
                record["classes"] = (count, 0)
                record["class_means"] = (float(minimum), 0.0)
                record["reason"] = "CONSTANT"
            else:
                threshold, classes, means, contrast = _otsu(histogram, count, value_sum)
                record["raw_threshold"] = threshold
                record["threshold"] = threshold
                record["classes"] = classes
                record["class_means"] = means
                record["contrast"] = contrast
                if threshold is None or classes[0] == 0 or classes[1] == 0:
                    record["reason"] = "NO_CLASS_SPLIT"
                elif contrast <= 0.0:
                    record["reason"] = "NO_CONTRAST"
                elif contrast < min_contrast:
                    record["reason"] = "LOW_CONTRAST"
                else:
                    record["trusted"] = True
                    record["reason"] = "OK"
        records.append(record)

    if smooth_neighbors:
        raw_thresholds = [record["threshold"] for record in records]
        for i, record in enumerate(records):
            if not record["trusted"]:
                continue
            neighbor_thresholds = [raw_thresholds[i]]
            for j, other in enumerate(records):
                if i != j and other["trusted"] and _overlap_or_touch(record["rect"], other["rect"]):
                    neighbor_thresholds.append(raw_thresholds[j])
            proposed = _median(neighbor_thresholds)
            # Keep a non-empty split within the observed gray range.
            proposed = max(record["minimum"], min(proposed, record["maximum"] - 1))
            classes, means, contrast = _class_stats(record["histogram"], proposed)
            record["threshold"] = proposed
            record["classes"] = classes
            record["class_means"] = means
            record["contrast"] = contrast
            if classes[0] == 0 or classes[1] == 0 or contrast <= 0.0:
                record["trusted"] = False
                record["reason"] = "SMOOTHED_NO_CLASS_SPLIT"
            elif contrast < min_contrast:
                record["trusted"] = False
                record["reason"] = "SMOOTHED_LOW_CONTRAST"
            else:
                record["reason"] = "OK_SMOOTHED" if proposed != raw_thresholds[i] else "OK"

    mask = bytearray(width * height)
    for record in records:
        if not record["trusted"]:
            continue
        x0, y0, x1, y1 = record["rect"]
        threshold = record["threshold"]
        for y in range(y0, y1):
            offset = y * width
            for x in range(x0, x1):
                if luma[offset + x] > threshold:
                    mask[offset + x] = 255

    trusted_count = 0
    diagnostic_blocks = []
    thresholds = []
    contrasts = []
    classes = []
    trusted_flags = []
    for record in records:
        if record["trusted"]:
            trusted_count += 1
        thresholds.append(record["threshold"] if record["trusted"] else None)
        contrasts.append(record["contrast"])
        classes.append(record["classes"])
        trusted_flags.append(record["trusted"])
        diagnostic_blocks.append({
            "rect": record["rect"],
            "pixel_count": record["pixel_count"],
            "raw_threshold": record["raw_threshold"],
            "threshold": record["threshold"] if record["trusted"] else None,
            "contrast": record["contrast"],
            "classes": record["classes"],
            "class_means": record["class_means"],
            "trusted": record["trusted"],
            "reason": record["reason"],
        })
    all_trusted = trusted_count == len(records)
    if all_trusted:
        reason = "OK"
    elif trusted_count:
        reason = "PARTIAL"
    else:
        reason = "NO_TRUSTED_BLOCK"
    diagnostics = {
        "valid": trusted_count > 0,
        "complete": all_trusted,
        "reason": reason,
        "roi": roi,
        "block_count": len(records),
        "trusted_block_count": trusted_count,
        "thresholds": thresholds,
        "contrast": contrasts,
        "classes": classes,
        "trusted": trusted_flags,
        "blocks": diagnostic_blocks,
    }
    return mask, diagnostics


otsu_mask = build_mask
