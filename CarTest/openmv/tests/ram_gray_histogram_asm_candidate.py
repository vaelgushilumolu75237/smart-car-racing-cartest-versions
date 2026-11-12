"""RAM differential for a flat, full-width Thumb histogram candidate.

The ASM pass only increments the same 256-bin counters for each luma byte.
The existing Viper compaction and per-block statistics remain unchanged. If a
configured block is not full-width or its area is not divisible by four, the
candidate falls back to the existing generic Viper kernel.
"""

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
    def _hist_block_flat4_asm(r0, r1, r2):
        # r0=contiguous luma bytes, r1=histogram block, r2=groups of 4.
        label(loop)
        cmp(r2, 0)
        beq(done)

        ldrb(r3, [r0, 0])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 1])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 2])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 3])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        add(r0, r0, 4)
        sub(r2, r2, 1)
        b(loop)

        label(done)
        mov(r0, 0)
else:
    _hist_block_flat4_asm = None


@micropython.native
def _histogram_asm_count_candidate(builder):
    if _hist_block_flat4_asm is None or _uctypes is None:
        return False

    width = builder.width
    block_count = builder._block_count
    rectangles = builder._rectangles
    for block in range(block_count):
        rect_base = block * 4
        x0 = rectangles[rect_base]
        x1 = rectangles[rect_base + 2]
        y0 = rectangles[rect_base + 1]
        y1 = rectangles[rect_base + 3]
        pixels = (x1 - x0) * (y1 - y0)
        if x0 != 0 or x1 != width or (pixels & 3):
            return False

    _histogram_clear(builder)
    luma_ptr = _uctypes.addressof(builder.luma)
    histograms_ptr = _uctypes.addressof(builder._histograms)
    for block in range(block_count):
        rect_base = block * 4
        y0 = rectangles[rect_base + 1]
        y1 = rectangles[rect_base + 3]
        hist_ptr = histograms_ptr + block * 256 * 4
        source_ptr = luma_ptr + y0 * width
        pixel_count = (y1 - y0) * width
        _hist_block_flat4_asm(
            source_ptr, hist_ptr, pixel_count >> 2)
    return True


@micropython.native
def _histogram_asm_compact_candidate(builder):
    # Retain the exact existing ascending-bin compaction and statistics.
    nonzero_values = builder._nonzero_values
    nonzero_counts = builder._nonzero_counts
    block_stats = builder._block_stats
    histograms = builder._histograms
    block_count = builder._block_count
    for block in range(block_count):
        hist_base = block * 256
        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = histograms[hist_base + value]
            if amount != 0:
                nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        block_stats[stats_base] = count
        block_stats[stats_base + 1] = value_sum
        block_stats[stats_base + 2] = minimum
        block_stats[stats_base + 3] = maximum


@micropython.viper
def _histogram_compact_viper_candidate(builder) -> int:
    """Typed version of the existing ascending-bin compaction and stats."""
    histograms = ptr32(builder._histograms)
    nonzero_values = ptr8(builder._nonzero_values)
    nonzero_counts = ptr32(builder._nonzero_counts)
    block_stats = ptr32(builder._block_stats)
    block_count = int(builder._block_count)
    for block in range(block_count):
        hist_base = block * 256
        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = histograms[hist_base + value]
            if amount != 0:
                nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        block_stats[stats_base] = count
        block_stats[stats_base + 1] = value_sum
        block_stats[stats_base + 2] = minimum
        block_stats[stats_base + 3] = maximum
    return 0


@micropython.native
def _histogram_asm_candidate(builder):
    if not _histogram_asm_count_candidate(builder):
        fast_gray._histogram_viper(builder)
        return False
    _histogram_asm_compact_candidate(builder)
    return True


@micropython.native
def _histogram_asm_viper_candidate(builder):
    if not _histogram_asm_count_candidate(builder):
        fast_gray._histogram_viper(builder)
        return False
    _histogram_compact_viper_candidate(builder)
    return True


@micropython.viper
def _histogram_clear(builder) -> int:
    histograms = ptr32(builder._histograms)
    total = int(builder._block_count) * 256
    for index in range(total):
        histograms[index] = 0
    return 0


def _histogram_candidate_host(builder):
    """Host model for the flat ASM pass, with generic Viper-equivalent fallback."""
    width = builder.width
    rectangles = builder._rectangles
    for block in range(builder._block_count):
        rect_base = block * 4
        if (rectangles[rect_base] != 0 or
                rectangles[rect_base + 2] != width or
                (((rectangles[rect_base + 2] - rectangles[rect_base]) *
                  (rectangles[rect_base + 3] - rectangles[rect_base + 1])) & 3)):
            fast_gray._histogram_host(
                builder.luma, builder._histograms, rectangles, width,
                builder._block_count, builder._nonzero_values,
                builder._nonzero_counts, builder._block_stats)
            return False

    _histogram_clear_host(builder)
    for block in range(builder._block_count):
        rect_base = block * 4
        hist_base = block * 256
        y0 = rectangles[rect_base + 1]
        y1 = rectangles[rect_base + 3]
        source_base = y0 * width
        for index in range(source_base, source_base + (y1 - y0) * width):
            value = builder.luma[index]
            builder._histograms[hist_base + value] += 1
    _histogram_compact_host(builder)
    return True


def _histogram_clear_host(builder):
    for index in range(builder._block_count * 256):
        builder._histograms[index] = 0


def _histogram_compact_host(builder):
    for block in range(builder._block_count):
        hist_base = block * 256
        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = builder._histograms[hist_base + value]
            if amount:
                builder._nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        builder._nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        builder._block_stats[stats_base] = count
        builder._block_stats[stats_base + 1] = value_sum
        builder._block_stats[stats_base + 2] = minimum
        builder._block_stats[stats_base + 3] = maximum


def _snapshot(builder):
    return (tuple(builder._histograms), bytes(builder._nonzero_values),
            tuple(builder._nonzero_counts), tuple(builder._block_stats))


def _seed_scratch(builder):
    for index in range(len(builder._histograms)):
        builder._histograms[index] = 0xA5A5A5A5
    for index in range(len(builder._nonzero_counts)):
        builder._nonzero_counts[index] = 0xA5A5A5A5
    for index in range(len(builder._block_stats)):
        builder._block_stats[index] = 0xA5A5A5A5
    for index in range(len(builder._nonzero_values)):
        builder._nonzero_values[index] = 0xA5


def _fill_luma(builder, style, seed):
    if style == "constant":
        for index in range(len(builder.luma)):
            builder.luma[index] = seed
        return
    state = seed & 0xFFFFFFFF
    for index in range(len(builder.luma)):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        builder.luma[index] = (state >> 24) & 0xFF


def run(repeats=300):
    if _hist_block_flat4_asm is None or _uctypes is None or _time is None:
        raise RuntimeError("run this validation on OpenMV MicroPython")
    builder = fast_gray.FastGrayMasks(config)
    workspace_addresses = (
        _uctypes.addressof(builder.luma),
        _uctypes.addressof(builder._histograms),
    )
    if workspace_addresses[0] & 3 or workspace_addresses[1] & 3:
        raise RuntimeError("histogram ASM inputs are not word-aligned")

    cases = (
        ("random", 0x13579BDF),
        ("random", 0x2468ACE1),
        ("random", 0xC001D00D),
        ("constant", 0),
        ("constant", 90),
        ("constant", 255),
    )
    print("HIST_ASM_START cases=", len(cases),
          "blocks=", builder._block_count)
    for style, seed in cases:
        _fill_luma(builder, style, seed)
        _seed_scratch(builder)
        fast_gray._histogram_viper(builder)
        expected = _snapshot(builder)
        # Use the same stale scratch before the candidate so trailing
        # nonzero-list bytes are compared exactly as well.
        _seed_scratch(builder)
        used = _histogram_asm_candidate(builder)
        if not used:
            raise AssertionError("configured full-width blocks used fallback")
        actual = _snapshot(builder)
        if actual != expected:
            raise AssertionError("ASM histogram differs for %s/%d" %
                                 (style, seed))
        _seed_scratch(builder)
        used = _histogram_asm_viper_candidate(builder)
        if not used:
            raise AssertionError("configured full-width blocks used fallback")
        if _snapshot(builder) != expected:
            raise AssertionError("ASM/Viper compact differs for %s/%d" %
                                 (style, seed))
        print("HIST_ASM_CASE", style, seed, "native+viper matched")
        _time.sleep_ms(20)

    _fill_luma(builder, "random", 0xDEADBEEF)
    _seed_scratch(builder)
    fast_gray._histogram_viper(builder)
    expected = _snapshot(builder)

    # Time each phase independently. Count includes histogram clearing; the
    # compaction input is the correct histogram produced by the Viper kernel.
    start = _time.ticks_us()
    for _ in range(repeats):
        if not _histogram_asm_count_candidate(builder):
            raise AssertionError("unexpected generic histogram fallback")
    count_us = _time.ticks_diff(_time.ticks_us(), start)

    start = _time.ticks_us()
    for _ in range(repeats):
        _histogram_clear(builder)
    clear_us = _time.ticks_diff(_time.ticks_us(), start)

    # Restore the correct histogram after the clear-only measurement.
    fast_gray._histogram_viper(builder)
    start = _time.ticks_us()
    for _ in range(repeats):
        _histogram_asm_compact_candidate(builder)
    compact_native_us = _time.ticks_diff(_time.ticks_us(), start)

    # Restore correct bins before independently timing typed compaction.
    fast_gray._histogram_viper(builder)
    start = _time.ticks_us()
    for _ in range(repeats):
        _histogram_compact_viper_candidate(builder)
    compact_viper_us = _time.ticks_diff(_time.ticks_us(), start)

    start = _time.ticks_us()
    for _ in range(repeats):
        fast_gray._histogram_viper(builder)
    viper_us = _time.ticks_diff(_time.ticks_us(), start)

    start = _time.ticks_us()
    for _ in range(repeats):
        if not _histogram_asm_candidate(builder):
            raise AssertionError("unexpected generic histogram fallback")
    asm_native_us = _time.ticks_diff(_time.ticks_us(), start)
    start = _time.ticks_us()
    for _ in range(repeats):
        if not _histogram_asm_viper_candidate(builder):
            raise AssertionError("unexpected generic histogram fallback")
    asm_viper_us = _time.ticks_diff(_time.ticks_us(), start)
    if _snapshot(builder) != expected:
        raise AssertionError("histogram changed during benchmark")
    print("HIST_ASM_PHASES repeats=%d clear_us=%d asm_count_clear_us=%d "
          "compact_native_us=%d compact_viper_us=%d viper_full_us=%d "
          "asm_native_full_us=%d asm_viper_full_us=%d" %
          (repeats, clear_us, count_us, compact_native_us,
           compact_viper_us, viper_us, asm_native_us, asm_viper_us))
    return 0


if __name__ == "__main__":
    run()
