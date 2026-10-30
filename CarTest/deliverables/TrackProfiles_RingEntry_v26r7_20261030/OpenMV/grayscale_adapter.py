"""Convert one windowed grayscale frame into bounded, known-region masks.

The 4x2 minimum pool preserves narrow dark paint in the 80x60 working grid.
Unknown/untrusted Otsu blocks remain unknown in ``known_region``; their zero
fill is never promoted to a measured wall or road boundary.
"""

RUNTIME_API_VERSION = 6

import thresholding


def _gray(pixel):
    if isinstance(pixel, (tuple, list)):
        if len(pixel) < 3:
            return int(pixel[0])
        red, green, blue = int(pixel[0]), int(pixel[1]), int(pixel[2])
        return (77 * red + 150 * green + 29 * blue) >> 8
    return int(pixel)


def _read_min_pool(image, width, height, scale_x, scale_y):
    image_width = image.width()
    image_height = image.height()
    if image_width == width and image_height == height:
        pool_x = 1
        pool_y = 1
        sampling = "grid"
    elif (image_width == width * scale_x and
          image_height == height * scale_y):
        pool_x = scale_x
        pool_y = scale_y
        sampling = "min%dx%d" % (scale_x, scale_y)
    else:
        raise ValueError("image size differs from grayscale grid config")

    # On OpenMV, image.bytearray() exposes a by-reference view of the frame
    # buffer. Read it only (never write through the view), and accept it only
    # when the dimensions and one-byte-per-pixel length agree exactly. This
    # excludes RGB565 buffers and lets RGB host fakes use tuple get_pixel.
    bytearray_reader = getattr(image, "bytearray", None)
    if callable(bytearray_reader):
        try:
            raw = bytearray_reader()
            if len(raw) == image_width * image_height:
                luma = bytearray(width * height)
                valid_buffer = True
                for y in range(height):
                    source_y = y * pool_y
                    row = y * width
                    for x in range(width):
                        source_x = x * pool_x
                        value = 255
                        for dy in range(pool_y):
                            base = ((source_y + dy) * image_width +
                                    source_x)
                            for dx in range(pool_x):
                                pixel = raw[base + dx]
                                if not isinstance(pixel, int) or \
                                        pixel < 0 or pixel > 255:
                                    valid_buffer = False
                                    break
                                if pixel < value:
                                    value = pixel
                            if not valid_buffer:
                                break
                        if not valid_buffer:
                            break
                        luma[row + x] = value
                    if not valid_buffer:
                        break
                if valid_buffer:
                    return luma, sampling
        except Exception:
            # API absence, unsupported format, or an unexpected host wrapper
            # falls through to the public tuple-coordinate pixel API.
            pass

    luma = bytearray(width * height)
    for y in range(height):
        source_y = y * pool_y
        row = y * width
        for x in range(width):
            source_x = x * pool_x
            value = 255
            for dy in range(pool_y):
                for dx in range(pool_x):
                    pixel = _gray(image.get_pixel((source_x + dx,
                                                   source_y + dy)))
                    if pixel < value:
                        value = pixel
            luma[row + x] = max(0, min(255, value))
    return luma, sampling


def _row_counts(mask, width, height, floor_y):
    counts = bytearray(height)
    for y in range(floor_y, height):
        offset = y * width
        count = 0
        for x in range(width):
            count += 1 if mask[offset + x] else 0
        counts[y] = count
    return counts


def build_masks(image, settings):
    """Return ``(road_mask, edge_mask, known_region, diagnostics)``.

    ``road_mask`` uses 255 for the brighter candidate surface. ``edge_mask``
    uses 1 for the darker class, but only inside trusted blocks. Consumers
    must also receive ``known_region`` and stop geometry at unknown pixels.
    """
    width = getattr(settings, "SURFACE_GRID_WIDTH", 80)
    height = getattr(settings, "SURFACE_GRID_HEIGHT", 60)
    floor_y = getattr(settings, "SURFACE_ROI_FLOOR_Y", 20)
    scale_x = getattr(settings, "SURFACE_PIXEL_SCALE_X", 4)
    scale_y = getattr(settings, "SURFACE_PIXEL_SCALE_Y", 2)
    roi = (0, floor_y, width, height)
    blocks = getattr(settings, "GRAY_THRESHOLD_BLOCKS", None)
    if blocks is None:
        split_a = floor_y + (height - floor_y) // 3
        split_b = floor_y + 2 * (height - floor_y) // 3
        blocks = ((0, floor_y, width, split_a),
                  (0, split_a, width, split_b),
                  (0, split_b, width, height))

    luma, sampling = _read_min_pool(
        image, width, height, scale_x, scale_y)
    road_mask, threshold_info = thresholding.build_mask(
        luma, width, height, roi=roi, blocks=blocks,
        min_contrast=getattr(settings, "GRAY_THRESHOLD_MIN_CONTRAST", 0),
        smooth_neighbors=getattr(settings,
                                 "GRAY_THRESHOLD_SMOOTH_NEIGHBORS", False))
    known_region = bytearray(width * height)
    for block in threshold_info["blocks"]:
        if not block["trusted"]:
            continue
        x0, y0, x1, y1 = block["rect"]
        for y in range(y0, y1):
            offset = y * width
            for x in range(x0, x1):
                known_region[offset + x] = 1

    edge_mask = bytearray(width * height)
    road_pixels = 0
    edge_pixels = 0
    for index in range(width * height):
        if not known_region[index]:
            road_mask[index] = 0
            continue
        if road_mask[index]:
            road_pixels += 1
        else:
            edge_mask[index] = 1
            edge_pixels += 1

    contrast_min = None
    contrast_max = None
    threshold_min = None
    threshold_max = None
    for block in threshold_info["blocks"]:
        if not block["trusted"]:
            continue
        contrast = block["contrast"]
        threshold = block["threshold"]
        contrast_min = (contrast if contrast_min is None else
                        min(contrast_min, contrast))
        contrast_max = (contrast if contrast_max is None else
                        max(contrast_max, contrast))
        threshold_min = (threshold if threshold_min is None else
                         min(threshold_min, threshold))
        threshold_max = (threshold if threshold_max is None else
                         max(threshold_max, threshold))

    diagnostics = {
        "sampling": sampling,
        # Retain this frame's immutable-by-convention min-pooled grayscale for
        # the later local edge-step check; consumers must not write through it.
        "luma": luma,
        "reason": threshold_info["reason"],
        "blocks": threshold_info["blocks"],
        "trusted_block_count": sum(
            1 for block in threshold_info["blocks"] if block["trusted"]),
        "complete": threshold_info["complete"],
        "valid": threshold_info["valid"],
        "road_pixels": road_pixels,
        "edge_pixels": edge_pixels,
        "mask_pixels": edge_pixels,
        "contrast_min": contrast_min,
        "contrast_max": contrast_max,
        "threshold_min": threshold_min,
        "threshold_max": threshold_max,
        "row_counts": _row_counts(edge_mask, width, height, floor_y),
        "road_row_counts": _row_counts(road_mask, width, height, floor_y),
    }
    return road_mask, edge_mask, known_region, diagnostics
