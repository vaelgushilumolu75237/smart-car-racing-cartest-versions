"""OpenMV board differential for the integrated production histogram path."""

import config
import fast_gray
import time


class _PartialWidthSettings:
    SURFACE_GRID_WIDTH = config.SURFACE_GRID_WIDTH
    SURFACE_GRID_HEIGHT = config.SURFACE_GRID_HEIGHT
    SURFACE_PIXEL_SCALE_X = config.SURFACE_PIXEL_SCALE_X
    SURFACE_PIXEL_SCALE_Y = config.SURFACE_PIXEL_SCALE_Y
    SURFACE_ROI_FLOOR_Y = config.SURFACE_ROI_FLOOR_Y
    GRAY_THRESHOLD_MIN_CONTRAST = config.GRAY_THRESHOLD_MIN_CONTRAST
    GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
    GRAY_THRESHOLD_BLOCKS = (
        (3, 8, 77, 20),
        (0, 24, 80, 36),
    )


class _EmptyBlocksSettings:
    SURFACE_GRID_WIDTH = config.SURFACE_GRID_WIDTH
    SURFACE_GRID_HEIGHT = config.SURFACE_GRID_HEIGHT
    SURFACE_PIXEL_SCALE_X = config.SURFACE_PIXEL_SCALE_X
    SURFACE_PIXEL_SCALE_Y = config.SURFACE_PIXEL_SCALE_Y
    SURFACE_ROI_FLOOR_Y = config.SURFACE_ROI_FLOOR_Y
    GRAY_THRESHOLD_MIN_CONTRAST = config.GRAY_THRESHOLD_MIN_CONTRAST
    GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
    GRAY_THRESHOLD_BLOCKS = (
        (0, 8, 80, 8),
        (0, 10, 80, 10),
    )


def _fill_luma(builder, style, seed):
    if style == "constant":
        for index in range(len(builder.luma)):
            builder.luma[index] = seed
        return
    state = seed & 0xFFFFFFFF
    for index in range(len(builder.luma)):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        builder.luma[index] = (state >> 24) & 0xFF


def _seed_scratch(builder):
    for index in range(len(builder._histograms)):
        builder._histograms[index] = 0xA5A5A5A5
    for index in range(len(builder._nonzero_counts)):
        builder._nonzero_counts[index] = 0xA5A5A5A5
    for index in range(len(builder._block_stats)):
        builder._block_stats[index] = 0xA5A5A5A5
    for index in range(len(builder._nonzero_values)):
        builder._nonzero_values[index] = 0xA5


def _snapshot(builder):
    return (tuple(builder._histograms), bytes(builder._nonzero_values),
            tuple(builder._nonzero_counts), tuple(builder._block_stats))


def _compare_production_path(builder, style, seed, expected_asm_ready):
    if builder._histogram_asm_ready != expected_asm_ready:
        raise AssertionError("unexpected production histogram guard state")
    _fill_luma(builder, style, seed)
    _seed_scratch(builder)
    fast_gray._histogram_viper(builder)
    expected = _snapshot(builder)

    _seed_scratch(builder)
    builder._calculate_thresholds()
    actual = _snapshot(builder)
    if actual != expected:
        raise AssertionError("production histogram differs for %s/%d" %
                             (style, seed))
    print("HIST_PRODUCTION_CASE", builder._block_count, style, seed,
          "matched asm_ready=", expected_asm_ready)
    time.sleep_ms(20)


def run():
    full_width = fast_gray.FastGrayMasks(config)
    if full_width._block_count != 4:
        raise AssertionError("expected all four production histogram blocks")
    if not full_width._histogram_asm_ready:
        raise AssertionError("production full-width ASM guard did not enable")
    for style, seed in (
            ("random", 0x13579BDF),
            ("constant", 0),
            ("random", 0x2468ACE1),
            ("constant", 90),
            ("constant", 255),
            ("random", 0xC001D00D)):
        _compare_production_path(full_width, style, seed, True)

    # Turn a previously populated block into an empty span on the same
    # instance. This checks per-frame clearing and the zero-work ASM case.
    full_width._rectangles[15] = full_width._rectangles[13]
    _compare_production_path(full_width, "random", 0xBADC0FFE, True)
    _compare_production_path(full_width, "constant", 0, True)

    partial_width = fast_gray.FastGrayMasks(_PartialWidthSettings)
    _compare_production_path(partial_width, "random", 0xDEADBEEF, False)
    _compare_production_path(partial_width, "constant", 73, False)

    empty_blocks = fast_gray.FastGrayMasks(_EmptyBlocksSettings)
    _compare_production_path(empty_blocks, "random", 0xF00DCAFE, True)
    _compare_production_path(empty_blocks, "constant", 0, True)
    print("HIST_PRODUCTION_PASS cases=12 blocks=4+partial+empty-reuse")
    return 0


if __name__ == "__main__":
    run()
