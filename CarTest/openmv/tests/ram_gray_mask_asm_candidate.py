"""RAM differential for a scalar Thumb mask-row kernel candidate.

The candidate changes only how a trusted row is classified and counted. It
keeps the production region geometry, threshold/trust decisions, and all mask
planes unchanged. Run on OpenMV with the usual 80x60 FastGrayMasks settings.
"""

from array import array

import config
import fast_gray

try:
    import micropython
    import time as _time
    import uctypes as _uctypes
    _HAS_TARGET = True
except ImportError:
    micropython = fast_gray.micropython
    _time = None
    _uctypes = None
    _HAS_TARGET = False


if _HAS_TARGET and _uctypes is not None:
    @micropython.asm_thumb
    def _mask_row_asm(r0, r1, r2, r3):
        # r0=luma row, r1=road row, r2=edge row,
        # r3=[known row pointer, threshold, row width].
        ldr(r4, [r3, 0])
        ldr(r5, [r3, 4])
        ldr(r6, [r3, 8])
        mov(r7, 0)

        label(pixel_loop)
        cmp(r6, 0)
        beq(done)
        ldrb(r3, [r0, 0])
        cmp(r3, r5)
        ble(edge_pixel)

        mov(r3, 255)
        strb(r3, [r1, 0])
        mov(r3, 1)
        strb(r3, [r4, 0])
        b(advance)

        label(edge_pixel)
        mov(r3, 1)
        strb(r3, [r2, 0])
        strb(r3, [r4, 0])
        add(r7, r7, 1)

        label(advance)
        add(r0, r0, 1)
        add(r1, r1, 1)
        add(r2, r2, 1)
        add(r4, r4, 1)
        sub(r6, r6, 1)
        b(pixel_loop)

        label(done)
        mov(r0, r7)
else:
    _mask_row_asm = None


@micropython.viper
def _clear_planes(builder) -> int:
    road = ptr8(builder.road_mask)
    edge = ptr8(builder.edge_mask)
    known = ptr8(builder.known_region)
    row_counts = ptr8(builder.row_counts)
    road_row_counts = ptr8(builder.road_row_counts)
    totals = ptr32(builder._pixel_totals)
    width = int(builder.width)
    height = int(builder.height)
    total = width * height
    for index in range(total):
        road[index] = 0
        edge[index] = 0
        known[index] = 0
    for y in range(height):
        row_counts[y] = 0
        road_row_counts[y] = 0
    totals[0] = 0
    totals[1] = 0
    return 0


@micropython.viper
def _mask_viper_local_counts(builder) -> int:
    """Candidate: word-clear planes and update row/totals once per row."""
    luma = ptr8(builder.luma)
    road = ptr8(builder.road_mask)
    edge = ptr8(builder.edge_mask)
    known = ptr8(builder.known_region)
    road_words = ptr32(builder.road_mask)
    edge_words = ptr32(builder.edge_mask)
    known_words = ptr32(builder.known_region)
    rectangles = ptr32(builder._rectangles)
    thresholds = ptr32(builder._threshold_values)
    trusted = ptr32(builder._trusted_values)
    row_counts = ptr8(builder.row_counts)
    road_row_counts = ptr8(builder.road_row_counts)
    totals = ptr32(builder._pixel_totals)
    width = int(builder.width)
    height = int(builder.height)
    block_count = int(builder._block_count)
    total = width * height

    # The three planes are bytearrays allocated by this instance. The board
    # test verifies word alignment before selecting this kernel.
    for word_index in range(total >> 2):
        road_words[word_index] = 0
        edge_words[word_index] = 0
        known_words[word_index] = 0
    for y in range(height):
        row_counts[y] = 0
        road_row_counts[y] = 0
    totals[0] = 0
    totals[1] = 0

    road_total = 0
    edge_total = 0
    for block in range(block_count):
        if trusted[block] == 0:
            continue
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        threshold = thresholds[block]
        for y in range(y0, y1):
            row = y * width
            row_road_count = 0
            row_edge_count = 0
            for x in range(x0, x1):
                index = row + x
                known[index] = 1
                if luma[index] > threshold:
                    road[index] = 255
                    row_road_count += 1
                else:
                    edge[index] = 1
                    row_edge_count += 1
            row_counts[y] += row_edge_count
            road_row_counts[y] += row_road_count
            road_total += row_road_count
            edge_total += row_edge_count
    totals[0] = road_total
    totals[1] = edge_total
    return 0


def _mask_local_counts_host(builder):
    """Host model for equivalence tests; target runs the typed Viper kernel."""
    total = builder.width * builder.height
    for index in range(total):
        builder.road_mask[index] = 0
        builder.edge_mask[index] = 0
        builder.known_region[index] = 0
    for y in range(builder.height):
        builder.row_counts[y] = 0
        builder.road_row_counts[y] = 0
    builder._pixel_totals[0] = 0
    builder._pixel_totals[1] = 0
    road_total = 0
    edge_total = 0
    for block in range(builder._block_count):
        if not builder._trusted_values[block]:
            continue
        rect_base = block * 4
        x0 = builder._rectangles[rect_base]
        y0 = builder._rectangles[rect_base + 1]
        x1 = builder._rectangles[rect_base + 2]
        y1 = builder._rectangles[rect_base + 3]
        threshold = builder._threshold_values[block]
        for y in range(y0, y1):
            row = y * builder.width
            row_road_count = 0
            row_edge_count = 0
            for x in range(x0, x1):
                index = row + x
                builder.known_region[index] = 1
                if builder.luma[index] > threshold:
                    builder.road_mask[index] = 255
                    row_road_count += 1
                else:
                    builder.edge_mask[index] = 1
                    row_edge_count += 1
            builder.row_counts[y] += row_edge_count
            builder.road_row_counts[y] += row_road_count
            road_total += row_road_count
            edge_total += row_edge_count
    builder._pixel_totals[0] = road_total
    builder._pixel_totals[1] = edge_total


@micropython.native
def _mask_asm_candidate(builder, workspace, workspace_ptr):
    _clear_planes(builder)
    luma_ptr = _uctypes.addressof(builder.luma)
    road_ptr = _uctypes.addressof(builder.road_mask)
    edge_ptr = _uctypes.addressof(builder.edge_mask)
    known_ptr = _uctypes.addressof(builder.known_region)
    meta = workspace
    rectangles = builder._rectangles
    thresholds = builder._threshold_values
    trusted = builder._trusted_values
    totals = builder._pixel_totals
    width = builder.width

    for block in range(builder._block_count):
        if trusted[block] == 0:
            continue
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        threshold = thresholds[block]
        row_width = x1 - x0
        for y in range(y0, y1):
            row_index = y * width + x0
            meta[0] = known_ptr + row_index
            meta[1] = threshold
            meta[2] = row_width
            edge_count = _mask_row_asm(
                luma_ptr + row_index,
                road_ptr + row_index,
                edge_ptr + row_index,
                workspace_ptr)
            road_count = row_width - edge_count
            builder.row_counts[y] += edge_count
            builder.road_row_counts[y] += road_count
            totals[0] += road_count
            totals[1] += edge_count
    return 0


class _GapSettings:
    SURFACE_GRID_WIDTH = 80
    SURFACE_GRID_HEIGHT = 60
    SURFACE_PIXEL_SCALE_X = 4
    SURFACE_PIXEL_SCALE_Y = 2
    SURFACE_ROI_FLOOR_Y = 8
    GRAY_THRESHOLD_MIN_CONTRAST = 18.0
    GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
    GRAY_THRESHOLD_BLOCKS = (
        (0, 8, 31, 24),
        (38, 12, 80, 31),
        (5, 35, 75, 49),
        (0, 52, 80, 60),
    )


def _fill_luma(builder, style, seed):
    luma = builder.luma
    if style == "constant_low":
        for index in range(len(luma)):
            luma[index] = 0
        return
    if style == "constant_high":
        for index in range(len(luma)):
            luma[index] = 255
        return
    state = seed & 0xFFFFFFFF
    for index in range(len(luma)):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        if style == "boundary_mix":
            luma[index] = (0, 64, 127, 128, 254, 255)[(state >> 29) % 6]
        else:
            luma[index] = (state >> 24) & 0xFF


def _prime_decisions(builder, threshold_seed, trust_mask):
    for block in range(builder._block_count):
        builder._threshold_values[block] = (
            threshold_seed + block * 53) & 0xFF
        builder._trusted_values[block] = (trust_mask >> block) & 1


def _snapshot(builder):
    return (bytes(builder.road_mask), bytes(builder.edge_mask),
            bytes(builder.known_region), bytes(builder.row_counts),
            bytes(builder.road_row_counts), tuple(builder._pixel_totals))


def _assert_same(expected, actual, label):
    names = ("road", "edge", "known", "row_counts",
             "road_row_counts", "pixel_totals")
    for name, left, right in zip(names, expected, actual):
        if left != right:
            if name == "pixel_totals":
                raise AssertionError("%s %s expected=%s actual=%s" %
                                     (label, name, left, right))
            first = -1
            differences = 0
            for index in range(len(left)):
                if left[index] != right[index]:
                    differences += 1
                    if first < 0:
                        first = index
            raise AssertionError("%s %s diffs=%d first=%d" %
                                 (label, name, differences, first))


def _run_case(builder, workspace, workspace_ptr,
              style, seed, threshold_seed, trust_mask):
    _fill_luma(builder, style, seed)
    _prime_decisions(builder, threshold_seed, trust_mask)
    fast_gray._mask_viper(builder)
    expected = _snapshot(builder)

    # Give the candidate stale values to ensure it clears all outputs first.
    builder.road_mask[:] = bytes([17]) * len(builder.road_mask)
    builder.edge_mask[:] = bytes([19]) * len(builder.edge_mask)
    builder.known_region[:] = bytes([23]) * len(builder.known_region)
    builder.row_counts[:] = bytes([29]) * len(builder.row_counts)
    builder.road_row_counts[:] = bytes([31]) * len(builder.road_row_counts)
    builder._pixel_totals[0] = 1001
    builder._pixel_totals[1] = 1003
    _mask_asm_candidate(builder, workspace, workspace_ptr)
    _assert_same(expected, _snapshot(builder),
                 "%s/%d/trust=%x/asm" % (style, seed, trust_mask))
    builder.road_mask[:] = bytes([17]) * len(builder.road_mask)
    builder.edge_mask[:] = bytes([19]) * len(builder.edge_mask)
    builder.known_region[:] = bytes([23]) * len(builder.known_region)
    builder.row_counts[:] = bytes([29]) * len(builder.row_counts)
    builder.road_row_counts[:] = bytes([31]) * len(builder.road_row_counts)
    builder._pixel_totals[0] = 1001
    builder._pixel_totals[1] = 1003
    _mask_viper_local_counts(builder)
    _assert_same(expected, _snapshot(builder),
                 "%s/%d/trust=%x/local" % (style, seed, trust_mask))
    print("MASK_ASM_CASE", style, seed, "trust=", trust_mask,
          "totals=", tuple(builder._pixel_totals))
    _time.sleep_ms(20)


def run(repeats=300):
    if _mask_row_asm is None or _uctypes is None or _time is None:
        raise RuntimeError("run this validation on OpenMV MicroPython")
    builders = (fast_gray.FastGrayMasks(config),
                fast_gray.FastGrayMasks(_GapSettings))
    builder = builders[1]
    workspace = array("I", [0, 0, 0])
    workspace_ptr = _uctypes.addressof(workspace)
    if workspace_ptr & 3:
        raise RuntimeError("mask row workspace is not word-aligned")
    for plane in (builder.road_mask, builder.edge_mask,
                  builder.known_region):
        if _uctypes.addressof(plane) & 3:
            raise RuntimeError("mask plane is not word-aligned")

    cases = (
        ("noise", 0x13579BDF, 30, 0xF),
        ("boundary_mix", 0x2468ACE1, 0, 0xF),
        ("constant_low", 1, 0, 0xF),
        ("constant_high", 2, 255, 0xF),
        ("noise", 0xC001D00D, 127, 0x5),
        ("noise", 0xDEADBEEF, 220, 0xA),
        ("boundary_mix", 0x10293847, 128, 0x0),
        ("noise", 0xA5A5F00D, 254, 0x9),
    )
    print("MASK_ASM_START cases=", len(cases),
          "blocks=", builder._block_count)
    for style, seed, threshold_seed, trust_mask in cases:
        _run_case(builder, workspace, workspace_ptr,
                  style, seed, threshold_seed, trust_mask)

    # Benchmark on a dense trusted pattern and compare all outputs before and
    # after timing so timing cannot hide a correctness mismatch.
    _fill_luma(builder, "noise", 0x13579BDF)
    _prime_decisions(builder, 93, 0xF)
    fast_gray._mask_viper(builder)
    expected = _snapshot(builder)
    _mask_asm_candidate(builder, workspace, workspace_ptr)
    _assert_same(expected, _snapshot(builder), "benchmark-input")
    _mask_viper_local_counts(builder)
    _assert_same(expected, _snapshot(builder), "benchmark-input/local")

    start = _time.ticks_us()
    for _ in range(repeats):
        fast_gray._mask_viper(builder)
    viper_us = _time.ticks_diff(_time.ticks_us(), start)
    start = _time.ticks_us()
    for _ in range(repeats):
        _mask_asm_candidate(builder, workspace, workspace_ptr)
    asm_us = _time.ticks_diff(_time.ticks_us(), start)
    start = _time.ticks_us()
    for _ in range(repeats):
        _mask_viper_local_counts(builder)
    local_us = _time.ticks_diff(_time.ticks_us(), start)
    print("MASK_KERNEL_BENCH repeats=%d viper_us=%d asm_us=%d local_viper_us=%d" %
          (repeats, viper_us, asm_us, local_us))
    _assert_same(expected, _snapshot(builder), "post-benchmark")
    return 0


if __name__ == "__main__":
    run()
