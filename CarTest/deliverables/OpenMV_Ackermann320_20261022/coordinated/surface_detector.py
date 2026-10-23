"""Bounded two-dimensional road-surface detector for OpenMV MicroPython.

The blue mask is a chroma cue for a white practice lane.  The grayscale mask
uses local contrast for a black competition boundary.  All thresholds here
are starting values; they are not a camera calibration.
"""

RUNTIME_API_VERSION = 6

import math

_ANGLE_VECTOR_CACHE = {}


def _value(settings, name, default):
    return getattr(settings, name, default)


def _rounded(value):
    if value >= 0:
        return int(value + 0.5)
    return int(value - 0.5)


def _gray_value(pixel):
    if isinstance(pixel, (tuple, list)):
        if len(pixel) < 3:
            return int(pixel[0])
        red, green, blue = pixel[0], pixel[1], pixel[2]
        return (77 * int(red) + 150 * int(green) +
                29 * int(blue)) >> 8
    return int(pixel)


def _blue_pixel(pixel, blue_min, blue_excess_min, saturation_min):
    if not isinstance(pixel, (tuple, list)) or len(pixel) < 3:
        return False
    red = int(pixel[0])
    green = int(pixel[1])
    blue = int(pixel[2])
    maximum = max(red, green, blue)
    minimum = min(red, green, blue)
    if maximum <= 0:
        return False

    # Compare normalized channel shares so exposure changes affect all three
    # channels together.  White, gray seams, and neutral shadows fail the
    # blue-excess test without relying on an absolute brightness cap.
    red_share = red * 255 // maximum
    green_share = green * 255 // maximum
    blue_share = blue * 255 // maximum
    if blue_share < blue_min:
        return False
    if blue_share - red_share < blue_excess_min:
        return False
    if blue_share - green_share < blue_excess_min:
        return False
    return maximum - minimum >= saturation_min


def _work_image(image, settings):
    """Return an 80x60 image or original source with explicit x/y strides."""
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    scale_x = _value(settings, "SURFACE_PIXEL_SCALE_X", 4)
    scale_y = _value(settings, "SURFACE_PIXEL_SCALE_Y", 2)
    image_width = image.width()
    image_height = image.height()
    if image_width == width and image_height == height:
        return image, 1, 1, "grid"

    if (image_width != width * scale_x or
            image_height != height * scale_y):
        raise ValueError("image size differs from surface grid config")

    try:
        half = image.copy(x_scale=1.0 / scale_x, y_scale=1.0 / scale_y)
        if half.width() == width and half.height() == height:
            return half, 1, 1, "copy"
    except Exception:
        pass

    # Compare-only legacy fallback. Production masks use fast_gray's exact
    # 4x2 minpool and inspect every source pixel in each grid cell.
    return image, scale_x, scale_y, "stride4x2-compare"


def build_mask(image, mode, settings):
    """Build a bounded 80x60 semantic boundary mask from one image.

    Returns ``(mask, diagnostics)``.  Mask values are 1 for a boundary and 0
    for the traversable surface.  The diagnostics include the actual sampling
    path so the host tests can exercise copy and fallback behavior.
    """
    if mode not in ("blue_white", "black_white"):
        raise ValueError("mode must be blue_white or black_white")

    work, stride_x, stride_y, sampling = _work_image(image, settings)
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    mask = bytearray(width * height)
    row_counts = bytearray(height)
    mask_count = 0
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)

    if mode == "blue_white":
        blue_min = _value(settings, "BLUE_NORMALIZED_MIN", 150)
        blue_excess_min = _value(
            settings, "BLUE_NORMALIZED_EXCESS_MIN", 60)
        saturation_min = _value(settings, "BLUE_SATURATION_MIN", 18)
        for y in range(floor_y, height):
            source_y = y if stride_y == 1 else y * stride_y
            row_count = 0
            for x in range(width):
                source_x = x if stride_x == 1 else x * stride_x
                pixel = work.get_pixel((source_x, source_y))
                if _blue_pixel(pixel, blue_min, blue_excess_min,
                               saturation_min):
                    mask[y * width + x] = 1
                    mask_count += 1
                    row_count += 1
            row_counts[y] = row_count
        return mask, {"sampling": sampling,
                      "mask_pixels": mask_count,
                      "row_counts": row_counts,
                      "reason": "OK" if mask_count else "NO_COLOR_MASK"}

    luma = bytearray(width * height)
    low = 255
    high = 0
    for y in range(floor_y, height):
        source_y = y if stride_y == 1 else y * stride_y
        row = y * width
        for x in range(width):
            source_x = x if stride_x == 1 else x * stride_x
            value = _gray_value(work.get_pixel((source_x, source_y)))
            if value < 0:
                value = 0
            elif value > 255:
                value = 255
            luma[row + x] = value
            if value < low:
                low = value
            if value > high:
                high = value

    contrast_span = high - low
    min_span = _value(settings, "GRAY_FRAME_RANGE_MIN", 12)
    if contrast_span < min_span:
        return mask, {"sampling": sampling, "mask_pixels": 0,
                      "row_counts": row_counts,
                      "reason": "LOW_FRAME_CONTRAST",
                      "contrast_span": contrast_span}

    # A 3x3 local mean catches narrow black paint while rejecting gradual
    # floor shading.  The threshold scales with the current frame range; it is
    # not tied to one scene-wide gray cap.
    radius = 1
    min_delta = _value(settings, "GRAY_LOCAL_CONTRAST_MIN", 28)
    fraction = _value(settings, "GRAY_LOCAL_CONTRAST_FRACTION", 0.25)
    delta = max(min_delta, int(contrast_span * fraction))
    for y in range(floor_y, height):
        row_count = 0
        for x in range(width):
            total = 0
            center = luma[y * width + x]
            count = 0
            for dy in (-1, 0, 1):
                neighbor_y = y + dy
                if neighbor_y < floor_y or neighbor_y >= height:
                    continue
                row = neighbor_y * width
                for dx in (-1, 0, 1):
                    if not dx and not dy:
                        continue
                    neighbor_x = x + dx
                    if neighbor_x < 0 or neighbor_x >= width:
                        continue
                    total += luma[row + neighbor_x]
                    count += 1
            if count == 0:
                continue
            local_mean = total // count
            if local_mean - center >= delta:
                mask[y * width + x] = 1
                mask_count += 1
                row_count += 1
        row_counts[y] = row_count
    return mask, {"sampling": sampling, "mask_pixels": mask_count,
                  "row_counts": row_counts,
                  "reason": "OK" if mask_count else "NO_DARK_CONTRAST",
                  "contrast_span": contrast_span,
                  "local_delta": delta}


def _inside(mask, x, y, width, height, floor_y, valid_region=None):
    if x < 0 or x >= width or y < floor_y or y >= height:
        return False
    index = y * width + x
    if valid_region is not None and not valid_region[index]:
        return False
    return mask[index] == 0


def _angle_vector(heading):
    try:
        return _ANGLE_VECTOR_CACHE[heading]
    except KeyError:
        radians = math.radians(heading)
        vector = (math.cos(radians), math.sin(radians))
        # Runtime headings come from the configured 5-degree angle lattice,
        # so this cache remains small and avoids repeated MicroPython trig.
        if len(_ANGLE_VECTOR_CACHE) >= 64:
            _ANGLE_VECTOR_CACHE.clear()
        _ANGLE_VECTOR_CACHE[heading] = vector
        return vector


def _raycast(mask, x, y, dx, dy, direction, settings, valid_region=None):
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    limit = _value(settings, "SURFACE_RAY_MAX_STEPS", 80)
    for step in range(1, limit + 1):
        sample_x = _rounded(x + direction * dx * step)
        sample_y = _rounded(y + direction * dy * step)
        if sample_x < 0 or sample_x >= width or \
                sample_y < floor_y or sample_y >= height:
            return None, True
        index = sample_y * width + sample_x
        if valid_region is not None and not valid_region[index]:
            # A block without trustworthy contrast is an unknown ray stop,
            # never a synthetic dark boundary.
            return None, False
        if mask[index]:
            return (float(sample_x), float(sample_y)), False
    return None, False


def _boundary_runs(mask, left, right, settings, valid_region=None):
    """Count mask islands between the first left/right boundary hits."""
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    x0, y0 = left
    x1, y1 = right
    span = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    steps = min(80, max(2, int(span + 0.5)))
    runs = 0
    in_run = False
    for index in range(steps + 1):
        ratio = float(index) / steps
        x = _rounded(x0 + (x1 - x0) * ratio)
        y = _rounded(y0 + (y1 - y0) * ratio)
        index = y * width + x
        if valid_region is not None and not valid_region[index]:
            return -1
        current = bool(mask[index])
        if current and not in_run:
            runs += 1
        in_run = current
    return runs


def _cross_section(mask, x, y, heading, settings, valid_region=None):
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    ix = _rounded(x)
    iy = _rounded(y)
    if not _inside(mask, ix, iy, width, height, floor_y, valid_region):
        return None, False, False

    normal_x, normal_y = _angle_vector(heading)
    left, left_clipped = _raycast(
        mask, x, y, normal_x, normal_y, -1, settings, valid_region)
    right, right_clipped = _raycast(
        mask, x, y, normal_x, normal_y, 1, settings, valid_region)
    if left is None or right is None:
        return None, left_clipped or right_clipped, False

    dx = right[0] - left[0]
    dy = right[1] - left[1]
    road_width = math.sqrt(dx * dx + dy * dy)
    min_width = _value(settings, "SURFACE_WIDTH_MIN_CELLS", 8)
    max_width = _value(settings, "SURFACE_WIDTH_MAX_CELLS", 70)
    if road_width < min_width or road_width > max_width:
        return None, False, road_width > max_width

    center_x = (left[0] + right[0]) * 0.5
    center_y = (left[1] + right[1]) * 0.5
    correction = math.sqrt((center_x - x) * (center_x - x) +
                           (center_y - y) * (center_y - y))
    extra_runs = _boundary_runs(mask, left, right, settings, valid_region)
    if extra_runs < 0:
        return None, False, False
    section = {
        "x": center_x,
        "y": center_y,
        "width": road_width,
        "heading": float(heading),
        "left": left,
        "right": right,
        "correction": correction,
        "boundary_runs": extra_runs,
    }
    return section, False, False


def _forward_vector(heading):
    normal_x, normal_y = _angle_vector(heading)
    return normal_y, -normal_x


def _point_distance(first, second):
    dx = first["x"] - second["x"]
    dy = first["y"] - second["y"]
    return math.sqrt(dx * dx + dy * dy)


def _seed_rows(settings):
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    ratios = _value(settings, "SURFACE_SEED_Y_RATIOS",
                    (0.90, 0.75, 0.60, 0.45, 0.35))
    result = []
    for ratio in ratios:
        row = _rounded(floor_y + (height - 1 - floor_y) * ratio)
        if row not in result:
            result.append(row)
    return result


def _seed_score(section, seed_x, seed_y, settings):
    correction = section["correction"]
    heading_cost = abs(section["heading"]) * _value(
        settings, "SURFACE_SEED_HEADING_PENALTY", 1.5)
    near_cost = (seed_y - _value(settings, "SURFACE_ROI_FLOOR_Y", 20)) * 0.08
    return 100.0 - correction * 4.0 - heading_cost + near_cost


def _component_encloses_hole(mask, settings, bounds, visited, queue):
    """Check a compact boundary component for an enclosed interior region."""
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    x0, y0, x1, y1 = bounds
    x0 = max(0, x0 - 1)
    y0 = max(_value(settings, "SURFACE_ROI_FLOOR_Y", 20), y0 - 1)
    x1 = min(width - 1, x1 + 1)
    y1 = min(_value(settings, "SURFACE_GRID_HEIGHT", 60) - 1, y1 + 1)
    min_hole_area = _value(settings,
                           "SURFACE_INTERNAL_HOLE_MIN_AREA", 4)

    # Mark all zero pixels connected to the local box border as exterior.
    queue_count = 0
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if x != x0 and x != x1 and y != y0 and y != y1:
                continue
            index = y * width + x
            if mask[index] or visited[index]:
                continue
            visited[index] = 1
            offset = queue_count * 2
            queue[offset] = index & 255
            queue[offset + 1] = index >> 8
            queue_count += 1

    head = 0
    while head < queue_count:
        offset = head * 2
        index = queue[offset] | (queue[offset + 1] << 8)
        head += 1
        x = index % width
        y = index // width
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if nx < x0 or nx > x1 or ny < y0 or ny > y1:
                continue
            neighbor = ny * width + nx
            if mask[neighbor] or visited[neighbor]:
                continue
            visited[neighbor] = 1
            offset = queue_count * 2
            queue[offset] = neighbor & 255
            queue[offset + 1] = neighbor >> 8
            queue_count += 1

    for y in range(y0 + 1, y1):
        for x in range(x0 + 1, x1):
            start = y * width + x
            if mask[start] or visited[start]:
                continue
            visited[start] = 1
            queue_count = 1
            queue[0] = start & 255
            queue[1] = start >> 8
            head = 0
            area = 0
            while head < queue_count:
                offset = head * 2
                index = queue[offset] | (queue[offset + 1] << 8)
                head += 1
                px = index % width
                py = index // width
                area += 1
                for nx, ny in ((px - 1, py), (px + 1, py),
                               (px, py - 1), (px, py + 1)):
                    if nx <= x0 or nx >= x1 or ny <= y0 or ny >= y1:
                        continue
                    neighbor = ny * width + nx
                    if mask[neighbor] or visited[neighbor]:
                        continue
                    visited[neighbor] = 1
                    offset = queue_count * 2
                    queue[offset] = neighbor & 255
                    queue[offset + 1] = neighbor >> 8
                    queue_count += 1
            if area >= min_hole_area:
                return True
    return False


def _has_internal_mask_component(mask, settings):
    """Find a compact closed contour enclosing an interior mask hole.

    Long detached line components can be the two cropped outer boundaries in
    grayscale mode.  They still close valid cross sections, so a component is
    island evidence only when it has bounded dimensions and an enclosed
    zero-mask region.  Compact speckles and short line fragments do not count.
    """
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    min_area = _value(settings, "SURFACE_INTERNAL_COMPONENT_MIN_AREA", 12)
    min_extent = _value(settings, "SURFACE_INTERNAL_COMPONENT_MIN_EXTENT", 3)
    max_extent = _value(settings, "SURFACE_INTERNAL_COMPONENT_MAX_EXTENT", 18)
    max_area = _value(settings, "SURFACE_INTERNAL_COMPONENT_MAX_AREA", 180)
    visited = bytearray(width * height)
    queue = bytearray(width * height * 2)
    neighbors = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0),
                 (-1, 1), (0, 1), (1, 1))
    for start_y in range(floor_y, height):
        for start_x in range(width):
            start = start_y * width + start_x
            if not mask[start] or visited[start]:
                continue
            visited[start] = 1
            queue_count = 1
            queue[0] = start & 255
            queue[1] = start >> 8
            queue_head = 0
            area = 0
            touches_roi = False
            min_x = width
            max_x = -1
            min_y = height
            max_y = -1
            while queue_head < queue_count:
                queue_offset = queue_head * 2
                index = queue[queue_offset] | \
                    (queue[queue_offset + 1] << 8)
                queue_head += 1
                x = index % width
                y = index // width
                area += 1
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
                if x == 0 or x == width - 1 or \
                        y == floor_y or y == height - 1:
                    touches_roi = True
                for dx, dy in neighbors:
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or nx >= width or ny < floor_y or \
                            ny >= height:
                        continue
                    neighbor = ny * width + nx
                    if mask[neighbor] and not visited[neighbor]:
                        visited[neighbor] = 1
                        queue_offset = queue_count * 2
                        queue[queue_offset] = neighbor & 255
                        queue[queue_offset + 1] = neighbor >> 8
                        queue_count += 1
            box_width = max_x - min_x + 1
            box_height = max_y - min_y + 1
            if touches_roi or area < min_area or area > max_area:
                continue
            if box_width < min_extent or box_height < min_extent:
                continue
            if box_width > max_extent or box_height > max_extent:
                continue
            if _component_encloses_hole(
                    mask, settings, (min_x, min_y, max_x, max_y),
                    visited, queue):
                return True
    return False


def _has_internal_open_band(mask, settings, row_counts=None):
    """Detect a multi-row open-surface band bounded by edge evidence.

    A crossbar/junction can remove both side boundaries across several image
    rows.  Ignore an empty band touching the ROI or frame edge, where ordinary
    horizon/crop loss occurs; require nearby boundary pixels to show that a
    tracked corridor entered or left the band.
    """
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    min_rows = _value(settings, "SURFACE_OPEN_BAND_MIN_ROWS", 3)
    support_distance = _value(settings, "SURFACE_OPEN_BAND_SUPPORT_ROWS", 5)
    max_run_width = _value(settings,
                           "SURFACE_OPEN_BAND_MAX_RUN_WIDTH", 36)
    min_pair_gap = _value(settings,
                          "SURFACE_OPEN_BAND_MIN_PAIR_GAP", 8)
    wide_rows = _value(settings, "SURFACE_OPEN_BAND_WIDE_ROWS", 4)

    def row_count(y):
        if row_counts is not None and y < len(row_counts):
            return row_counts[y]
        offset = y * width
        count = 0
        for x in range(width):
            count += 1 if mask[offset + x] else 0
        return count

    def paired_boundary_row(y):
        """True when two separated edge runs straddle the image center."""
        offset = y * width
        runs = []
        x = 0
        while x < width:
            if not mask[offset + x]:
                x += 1
                continue
            start = x
            while x < width and mask[offset + x]:
                x += 1
            end = x - 1
            if end - start + 1 <= max_run_width:
                runs.append((start, end))
        center = (width - 1) * 0.5
        for first_index in range(len(runs)):
            left_run = runs[first_index]
            if left_run[0] >= center:
                continue
            for second_index in range(first_index + 1, len(runs)):
                right_run = runs[second_index]
                if right_run[1] <= center:
                    continue
                if right_run[0] - left_run[1] - 1 >= min_pair_gap:
                    return True
        return False

    def has_paired_support(start_y, end_y):
        for near_y in range(start_y, end_y + 1):
            if paired_boundary_row(near_y):
                return True
        return False

    def has_wide_band(start_y, end_y, backwards):
        y = start_y
        step = -1 if backwards else 1
        streak = 0
        wide_limit = int(width * 0.75)
        while (y >= end_y if backwards else y <= end_y):
            if row_count(y) >= wide_limit:
                streak += 1
                if streak >= wide_rows:
                    # A broad dark region continuous to a crop boundary is
                    # incomplete evidence: it cannot establish an internally
                    # bounded bar separating two observed surfaces.
                    edge_y = floor_y if backwards else height - 1
                    probe_y = y
                    edge_connected = True
                    while probe_y != edge_y:
                        probe_y += step
                        if row_count(probe_y) < wide_limit:
                            edge_connected = False
                            break
                    if not edge_connected:
                        return True
                    # Since this run reaches the crop edge, there cannot be
                    # another disjoint run farther in the scan direction.
                    return False
            else:
                streak = 0
            y += step
        return False

    y = floor_y
    while y < height:
        if row_count(y) != 0:
            y += 1
            continue
        start = y
        while y < height and row_count(y) == 0:
            y += 1
        end = y - 1
        if end - start + 1 < min_rows:
            continue
        if start <= floor_y or end >= height - 1:
            continue
        before = max(floor_y, start - support_distance)
        after = min(height - 1, end + support_distance)
        paired_before = has_paired_support(before, start - 1)
        paired_after = has_paired_support(end + 1, after)
        wide_before = has_wide_band(start - 1, before, True)
        wide_after = has_wide_band(end + 1, after, False)
        # A brief loss of narrow lines can come from half-resolution aliasing
        # in an ordinary straight.  Do not call a gap a junction unless a
        # broad transverse region is visible on one side and separated road
        # edges reappear on the other side.
        supported = ((wide_before and paired_after) or
                     (paired_before and wide_after))
        if supported:
            return True
    return False


def _add_seed_cluster(clusters, candidate, settings):
    separation = _value(settings, "SURFACE_SEED_CLUSTER_CELLS", 5)
    for index in range(len(clusters)):
        prior = clusters[index]
        if _point_distance(prior["section"], candidate["section"]) <= separation:
            if candidate["score"] > prior["score"]:
                clusters[index] = candidate
            return
    clusters.append(candidate)
    if len(clusters) > 3:
        worst = min(range(len(clusters)), key=lambda item: clusters[item]["score"])
        del clusters[worst]


def _find_seed(mask, settings, row_counts=None, valid_region=None,
               region_complete=True):
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    x_center = (width - 1) * 0.5
    offsets = _value(settings, "SURFACE_SEED_X_OFFSETS", (-12, 0, 12))
    headings = _value(settings, "SURFACE_SEED_HEADINGS",
                       (-60, -45, -30, -15, 0, 15, 30, 45, 60))
    max_correction = _value(settings, "SURFACE_SEED_MAX_CORRECTION_CELLS", 10)
    by_row = []
    clipped_evidence = False
    wide_evidence = False
    island_evidence = (region_complete and
                       _has_internal_mask_component(mask, settings))
    branch_evidence = (region_complete and
                       _has_internal_open_band(mask, settings, row_counts))
    hypotheses = 0

    for seed_y in _seed_rows(settings):
        clusters = []
        for offset in offsets:
            seed_x = x_center + offset
            if not _inside(mask, _rounded(seed_x), seed_y, width,
                           _value(settings, "SURFACE_GRID_HEIGHT", 60),
                           floor_y, valid_region):
                continue
            for heading in headings:
                hypotheses += 1
                section, clipped, too_wide = _cross_section(
                    mask, seed_x, seed_y, heading, settings, valid_region)
                clipped_evidence = clipped_evidence or clipped
                wide_evidence = wide_evidence or too_wide
                if section is None:
                    continue
                if section["boundary_runs"] > 2:
                    continue
                if section["correction"] > max_correction:
                    continue
                section["seed_y"] = seed_y
                section["seed_x"] = seed_x
                candidate = {
                    "section": section,
                    "score": _seed_score(section, seed_x, seed_y, settings),
                    "row": seed_y,
                }
                _add_seed_cluster(clusters, candidate, settings)
        clusters.sort(key=lambda item: item["score"], reverse=True)
        by_row.append(clusters)

    all_candidates = []
    for clusters in by_row:
        all_candidates.extend(clusters)
    if not all_candidates:
        if island_evidence:
            return None, "AMBIGUOUS", {
                "wide_evidence": wide_evidence,
                "island_evidence": True,
                "clipped_evidence": clipped_evidence,
                "hypotheses": hypotheses,
            }
        if branch_evidence:
            return None, "AMBIGUOUS", {
                "wide_evidence": True,
                "island_evidence": False,
                "branch_evidence": True,
                "clipped_evidence": clipped_evidence,
                "hypotheses": hypotheses,
            }
        if wide_evidence:
            return None, "AMBIGUOUS", {
                "wide_evidence": True,
                "island_evidence": False,
                "branch_evidence": False,
                "clipped_evidence": clipped_evidence,
                "hypotheses": hypotheses,
            }
        return None, "NO_CLOSED_SECTION", {
            "wide_evidence": False,
            "island_evidence": False,
            "branch_evidence": False,
            "clipped_evidence": clipped_evidence,
            "hypotheses": hypotheses,
        }

    all_candidates.sort(key=lambda item: item["score"], reverse=True)
    best = all_candidates[0]
    margin = _value(settings, "SURFACE_SEED_AMBIGUITY_MARGIN", 18.0)
    min_separation = _value(settings, "SURFACE_AMBIGUITY_SEPARATION_CELLS", 8)
    for competitor in all_candidates[1:]:
        if competitor["row"] != best["row"]:
            continue
        if best["score"] - competitor["score"] > margin:
            continue
        if _point_distance(best["section"], competitor["section"]) >= \
                min_separation:
            return None, "AMBIGUOUS", {
                "wide_evidence": wide_evidence,
                "island_evidence": island_evidence,
                "branch_evidence": branch_evidence,
                "clipped_evidence": clipped_evidence,
                "hypotheses": hypotheses,
            }
    starts = []
    dedupe_distance = _value(
        settings, "SURFACE_SEED_TRIAL_DEDUPE_CELLS", 5)
    for candidate in all_candidates:
        duplicate = False
        for selected in starts:
            if abs(candidate["row"] - selected["row"]) <= 3 and \
                    _point_distance(candidate["section"],
                                    selected["section"]) < dedupe_distance:
                duplicate = True
                break
        if not duplicate:
            starts.append(candidate)
        if len(starts) >= _value(settings, "SURFACE_SEED_TRIALS_MAX", 3):
            break
    return starts, "OK", {
        "wide_evidence": wide_evidence,
        "island_evidence": island_evidence,
        "branch_evidence": branch_evidence,
        "clipped_evidence": clipped_evidence,
        "hypotheses": hypotheses,
    }


def _next_section(mask, prior, settings, valid_region=None):
    step_size = _value(settings, "SURFACE_TRACE_STEP_CELLS", 3)
    step_x, step_y = _forward_vector(prior["heading"])
    predicted_x = prior["x"] + step_x * step_size
    predicted_y = prior["y"] + step_y * step_size
    offsets = _value(settings, "SURFACE_TRACE_HEADING_OFFSETS",
                     (-30, -20, -10, 0, 10, 20, 30))
    max_heading = _value(settings, "SURFACE_TRACE_MAX_ABS_HEADING", 75)
    max_correction = _value(settings, "SURFACE_TRACE_MAX_CORRECTION_CELLS", 8)
    max_width_change = _value(settings, "SURFACE_WIDTH_STEP_CHANGE_MAX", 14)
    min_progress = _value(settings, "SURFACE_TRACE_MIN_PROGRESS_CELLS", 0.75)
    candidates = []
    clipped = False
    wide = False
    island = False
    backward = False

    for delta in offsets:
        heading = prior["heading"] + delta
        if abs(heading) > max_heading:
            continue
        section, was_clipped, too_wide = _cross_section(
            mask, predicted_x, predicted_y, heading, settings, valid_region)
        clipped = clipped or was_clipped
        wide = wide or too_wide
        if section is None:
            continue
        if section["boundary_runs"] > 2:
            island = True
            continue
        # The current trace convention advances toward smaller image y. An
        # equal rounded row is possible at shallow headings, but a section
        # whose center moves back down the image must not be fresh progress.
        if (abs(prior["heading"]) <= max_heading and step_y < 0.0 and
                section["y"] > prior["y"]):
            backward = True
            continue
        if section["correction"] > max_correction:
            continue
        progress = ((section["x"] - prior["x"]) * step_x +
                    (section["y"] - prior["y"]) * step_y)
        if progress < min_progress:
            continue
        if _point_distance(section, prior) < min_progress:
            continue
        width_change = abs(section["width"] - prior["width"])
        allowed_width_change = max(
            max_width_change, prior["width"] * 0.30)
        if width_change > allowed_width_change:
            continue
        angle_change = abs(heading - prior["heading"])
        # A candidate whose normal follows the corridor should recenter the
        # predicted point.  Weight that geometric evidence above a small
        # turn regularizer; the previous weighting preferred a zero-angle
        # section even while its center drifted sideways at every step.
        score = (100.0 - angle_change * 0.20 -
                 section["correction"] * 8.0 - width_change * 0.50)
        section["score"] = score
        section["width_change"] = width_change
        section["angle_change"] = angle_change
        candidates.append(section)

    if island:
        return None, "TRACE_SHORT", {"wide": wide, "clipped": clipped,
                                      "island": False}
    if not candidates:
        reason = ("TRACE_BACKWARD" if backward else
                  "ROI_CLIP" if clipped else "TRACE_SHORT")
        if wide:
            reason = "AMBIGUOUS"
        return None, reason, {"wide": wide, "clipped": clipped,
                              "backward": backward,
                              "island": False}

    candidates.sort(key=lambda item: item["score"], reverse=True)
    best = candidates[0]
    margin = _value(settings, "SURFACE_PATH_AMBIGUITY_MARGIN", 18.0)
    min_separation = _value(settings, "SURFACE_AMBIGUITY_SEPARATION_CELLS", 8)
    for competitor in candidates[1:]:
        if best["score"] - competitor["score"] > margin:
            continue
        angle_gap = abs(best["heading"] - competitor["heading"])
        center_gap = _point_distance(best, competitor)
        # Headings around one center are competing descriptions of the same
        # corridor normal.  A branch requires both a materially different
        # tangent and a spatially separated fresh center.
        branch_separation = _value(
            settings, "SURFACE_BRANCH_SEPARATION_CELLS", 3.5)
        if angle_gap >= 25 and center_gap >= branch_separation:
            # Competing local cross sections are fresh branch evidence.  A
            # single attractive arm must not be silently selected at a fork.
            return None, "AMBIGUOUS", {"wide": wide, "clipped": clipped,
                                       "island": False}
    return best, "OK", {"wide": wide, "clipped": clipped,
                         "backward": backward, "island": False}


def _target_at_distance(sections, distance):
    if not sections:
        return None
    if distance <= 0:
        return sections[0]["x"], sections[0]["y"]
    travelled = 0.0
    previous = sections[0]
    for current in sections[1:]:
        segment = _point_distance(previous, current)
        if travelled + segment >= distance and segment > 0:
            ratio = (distance - travelled) / segment
            return (previous["x"] +
                    (current["x"] - previous["x"]) * ratio,
                    previous["y"] +
                    (current["y"] - previous["y"]) * ratio)
        travelled += segment
        previous = current
    return sections[-1]["x"], sections[-1]["y"]


def _quality(sections, fresh_sections, settings):
    if fresh_sections <= 0:
        return 0
    width_changes = []
    heading_changes = []
    corrections = []
    for index in range(1, len(sections)):
        width_changes.append(abs(sections[index]["width"] -
                                 sections[index - 1]["width"]))
        heading_changes.append(abs(sections[index]["heading"] -
                                   sections[index - 1]["heading"]))
        corrections.append(sections[index]["correction"])
    support_score = min(100.0, 45.0 + fresh_sections * 11.0)
    avg_width_change = sum(width_changes) / max(1, len(width_changes))
    avg_heading_change = sum(heading_changes) / max(1, len(heading_changes))
    avg_correction = sum(corrections) / max(1, len(corrections))
    width_score = max(0.0, 100.0 - avg_width_change * 4.0)
    heading_score = max(0.0, 100.0 - avg_heading_change * 1.5)
    correction_score = max(0.0, 100.0 - avg_correction * 8.0)
    value = (support_score * 0.40 + width_score * 0.25 +
             heading_score * 0.20 + correction_score * 0.15)
    return int(value + 0.5)


def _scene(sections, evidence):
    if evidence.get("island_evidence"):
        return "ROUNDABOUT_CANDIDATE"
    if evidence.get("wide_evidence") or evidence.get("branch_evidence"):
        return "WIDE_JUNCTION_CANDIDATE"
    heading_trends = []
    for index in range(1, len(sections)):
        change = sections[index]["heading"] - sections[index - 1]["heading"]
        if change >= 5:
            heading_trends.append(1)
        elif change <= -5:
            heading_trends.append(-1)
    if 1 in heading_trends and -1 in heading_trends:
        return "S_BEND"
    if not sections or max([abs(item["heading"]) for item in sections]) < 12:
        return "STRAIGHT"
    final_heading = sections[-1]["heading"]
    return "RIGHT_CURVE" if final_heading > 0 else "LEFT_CURVE"


def _failure(reason, evidence, sections=None, path_length=0.0, seed=None):
    attempted_count = max(0, len(sections or ()) - 1)
    seed_point = None
    if seed is not None:
        seed_point = (seed["x"], seed["y"])
    return {
        "valid": False,
        "quality": 0,
        "reason": reason,
        "scene": _scene(sections or [], evidence),
        "seed": seed_point,
        "target": None,
        "sections": [],
        "fresh_sections": attempted_count,
        "path_length": path_length,
        "hypotheses": evidence.get("hypotheses", 0),
        "wide_evidence": evidence.get("wide_evidence", False),
        "island_evidence": evidence.get("island_evidence", False),
        "branch_evidence": evidence.get("branch_evidence", False),
        "clipped_evidence": evidence.get("clipped_evidence", False),
    }


def _trace_seed(mask, seed, evidence, settings, valid_region=None):
    sections = [seed]
    fresh_sections = 0
    path_length = 0.0
    trace_limit = _value(settings, "SURFACE_TRACE_MAX_SECTIONS", 32)
    lookahead = _value(settings, "SURFACE_TARGET_LOOKAHEAD_CELLS", 10)
    trace_until = _value(settings, "SURFACE_TRACE_UNTIL_CELLS", 13)
    last_reason = "TRACE_SHORT"

    while fresh_sections < trace_limit and len(sections) < trace_limit:
        if path_length >= trace_until:
            break
        next_section, next_reason, trace_evidence = _next_section(
            mask, sections[-1], settings, valid_region)
        if next_section is None:
            local_evidence = dict(evidence)
            if trace_evidence.get("island"):
                local_evidence["island_evidence"] = True
            if trace_evidence.get("wide"):
                local_evidence["wide_evidence"] = True
            if trace_evidence.get("clipped"):
                local_evidence["clipped_evidence"] = True
            if next_reason == "AMBIGUOUS":
                return _failure("AMBIGUOUS", local_evidence, sections,
                                path_length, seed)
            last_reason = next_reason
            evidence = local_evidence
            break
        path_length += _point_distance(sections[-1], next_section)
        sections.append(next_section)
        fresh_sections += 1

    min_fresh = _value(settings, "SURFACE_MIN_FRESH_SECTIONS", 3)
    min_length = _value(settings, "SURFACE_MIN_PATH_LENGTH_CELLS", 8)
    if fresh_sections < min_fresh or path_length < min_length:
        return _failure(last_reason, evidence, sections, path_length, seed)

    target = _target_at_distance(sections, lookahead)
    if target is None:
        return _failure("TRACE_SHORT", evidence, sections, path_length, seed)
    quality = _quality(sections, fresh_sections, settings)
    min_quality = _value(settings, "SURFACE_MIN_CONFIDENCE", 60)
    if quality < min_quality:
        return _failure("LOW_QUALITY", evidence, sections, path_length, seed)

    return {
        "valid": True,
        "quality": quality,
        "reason": "OK",
        "scene": _scene(sections, evidence),
        "seed": (seed["x"], seed["y"]),
        "target": target,
        "sections": sections,
        "fresh_sections": fresh_sections,
        "path_length": path_length,
        "hypotheses": evidence.get("hypotheses", 0),
        "wide_evidence": evidence.get("wide_evidence", False),
        "island_evidence": evidence.get("island_evidence", False),
        "branch_evidence": evidence.get("branch_evidence", False),
        "clipped_evidence": evidence.get("clipped_evidence", False),
    }


def detect_path(mask, settings, mask_info=None, valid_region=None):
    """Find a closed seed section and trace fresh 2-D sections forward."""
    width = _value(settings, "SURFACE_GRID_WIDTH", 80)
    height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
    if len(mask) != width * height:
        raise ValueError("surface mask size differs from configured grid")
    if valid_region is not None and len(valid_region) != width * height:
        raise ValueError("surface valid-region size differs from configured grid")

    row_counts = (mask_info.get("row_counts")
                  if mask_info is not None else None)
    floor_y = _value(settings, "SURFACE_ROI_FLOOR_Y", 20)
    region_complete = True
    if valid_region is not None:
        for y in range(floor_y, height):
            offset = y * width
            for x in range(width):
                if not valid_region[offset + x]:
                    region_complete = False
                    break
            if not region_complete:
                break
    seeds, reason, evidence = _find_seed(
        mask, settings, row_counts, valid_region, region_complete)
    if seeds is None:
        return _failure(reason, evidence)
    if evidence.get("island_evidence") or evidence.get("branch_evidence"):
        # A closed internal boundary may be a ring or another road element.
        # Image-only input cannot select a safe lap or exit policy.
        return _failure("AMBIGUOUS", evidence)

    last_failure = None
    for candidate in seeds:
        trial = _trace_seed(mask, candidate["section"], evidence, settings,
                            valid_region)
        if trial["reason"] == "AMBIGUOUS":
            return trial
        if trial["valid"]:
            # Widths seen only in unrelated seed hypotheses can come from
            # perspective magnification at the bottom of a normal road.  A
            # accepted bounded path is not itself evidence of a junction.
            trial["wide_evidence"] = False
            scene_evidence = dict(evidence)
            scene_evidence["wide_evidence"] = False
            trial["scene"] = _scene(trial["sections"], scene_evidence)
            return trial
        if last_failure is None or \
                trial["fresh_sections"] > last_failure["fresh_sections"]:
            last_failure = trial
    if last_failure is not None:
        return last_failure
    return _failure("NO_CLOSED_SECTION", evidence)
