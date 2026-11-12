"""Production and isolated exact 4x2 min-pool ASM RAM differential.

This fixed-size experiment maps the 320x120 camera crop to the existing 80x60
grid. Pass a writable 320x120 grayscale bytearray to run() from the OpenMV REPL.
"""

import fast_gray
import config

try:
    import micropython
    import uctypes as _uctypes
    import time as _time
    _HAS_MICROPYTHON = True
except ImportError:
    micropython = fast_gray.micropython
    _uctypes = None
    _time = None
    _HAS_MICROPYTHON = False


if _HAS_MICROPYTHON and _uctypes is not None:
    @micropython.asm_thumb
    def _min_pool_asm(r0, r1):
        # The camera crop is 320x120 and the fixed output is 80x60.
        mov(r2, r0)
        mov(r0, 80)
        lsl(r0, r0, 2)
        add(r3, r2, r0)
        mov(r4, 60)

        label(row_loop)
        mov(r5, 80)

        label(column_loop)
        # Load the eight pixels in the exact 4x2 source cell and keep their
        # scalar unsigned minimum. Grayscale bytes are 0..255, so BGE after
        # CMP has the same ordering as an unsigned comparison here.
        ldrb(r6, [r2, 0])
        ldrb(r7, [r2, 1])
        cmp(r7, r6)
        bge(top1_done)
        mov(r6, r7)
        label(top1_done)
        ldrb(r7, [r2, 2])
        cmp(r7, r6)
        bge(top2_done)
        mov(r6, r7)
        label(top2_done)
        ldrb(r7, [r2, 3])
        cmp(r7, r6)
        bge(top3_done)
        mov(r6, r7)
        label(top3_done)
        ldrb(r7, [r3, 0])
        cmp(r7, r6)
        bge(bottom0_done)
        mov(r6, r7)
        label(bottom0_done)
        ldrb(r7, [r3, 1])
        cmp(r7, r6)
        bge(bottom1_done)
        mov(r6, r7)
        label(bottom1_done)
        ldrb(r7, [r3, 2])
        cmp(r7, r6)
        bge(bottom2_done)
        mov(r6, r7)
        label(bottom2_done)
        ldrb(r7, [r3, 3])
        cmp(r7, r6)
        bge(bottom3_done)
        mov(r6, r7)
        label(bottom3_done)
        strb(r6, [r1, 0])

        add(r2, r2, 4)
        add(r3, r3, 4)
        add(r1, r1, 1)
        sub(r5, r5, 1)
        cmp(r5, 0)
        bne(column_loop)

        # Skip the next source row after consuming two rows for this output.
        mov(r0, 80)
        lsl(r0, r0, 2)
        add(r2, r2, r0)
        add(r3, r3, r0)
        sub(r4, r4, 1)
        cmp(r4, 0)
        bne(row_loop)
        mov(r0, 0)
else:
    _min_pool_asm = None


class _PoolState:
    width = 80
    height = 60
    scale_y = 2

    def __init__(self, luma):
        self.luma = luma


class _BufferImage:
    def __init__(self, raw):
        self.raw = raw

    def width(self):
        return 320

    def height(self):
        return 120

    def bytearray(self):
        return self.raw


def _first_difference(left, right):
    mismatch_count = 0
    first_index = -1
    for index in range(4800):
        if left[index] != right[index]:
            mismatch_count += 1
            if first_index < 0:
                first_index = index
    return mismatch_count, first_index


def _fill_constant(source, value):
    for index in range(320 * 120):
        source[index] = value


def _fill_minimum_position(source, minimum_position):
    seen = 0
    for y in range(60):
        source_top = y * 2 * 320
        for x in range(80):
            cell = y * 80 + x
            minimum = (cell * 13 + 7) % 255
            left = x * 4
            for position in range(8):
                row = source_top + (position // 4) * 320
                column = left + position % 4
                source[row + column] = (
                    minimum if position == minimum_position else 255)
            seen |= 1 << minimum_position
    return seen


def _fill_random(source, seed):
    state = seed & 0xFFFFFFFF
    for index in range(320 * 120):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        source[index] = (state >> 24) & 0xFF


def _compare_case(source, asm_output, viper_output, reference_output,
                  state, builder, frame, source_ptr, target_ptr,
                  label, case):
    fast_gray._min_pool_host(
        source, reference_output, 320, 80, 60, 4, 2)
    fast_gray._min_pool_words_viper(state, source, 320)
    _min_pool_asm(source_ptr, target_ptr)
    frame.raw = source
    builder._read_frame(frame)

    asm_diffs, asm_first = _first_difference(reference_output, asm_output)
    viper_diffs, viper_first = _first_difference(reference_output, viper_output)
    production_diffs, production_first = _first_difference(
        reference_output, builder.luma)
    print("MINPOOL_ASM_CASE", label, case,
          "asm=", asm_diffs, asm_first,
          "viper=", viper_diffs, viper_first,
          "production=", production_diffs, production_first)
    _time.sleep_ms(20)
    if asm_diffs or viper_diffs or production_diffs:
        raise AssertionError("min-pool output differs from exact 8-pixel minimum")


def run(source, repeats=300):
    """Compare patterns against reference/Viper/production, then time both."""
    if _min_pool_asm is None or _uctypes is None or _time is None:
        raise RuntimeError("run this validation on OpenMV MicroPython")
    if len(source) != 320 * 120:
        raise ValueError("source must be the 320x120 grayscale crop")
    source_ptr = _uctypes.addressof(source)
    if source_ptr & 3:
        raise ValueError("source grayscale buffer must be word-aligned")

    asm_output = bytearray(80 * 60)
    viper_output = bytearray(80 * 60)
    reference_output = bytearray(80 * 60)
    state = _PoolState(viper_output)
    target_ptr = _uctypes.addressof(asm_output)
    builder = fast_gray.FastGrayMasks(config)
    if not builder._min_pool_asm_ready:
        raise RuntimeError("production min-pool ASM pointers are unavailable")
    if builder._min_pool_target_ptr != _uctypes.addressof(builder.luma):
        raise AssertionError("production output pointer is not instance-local")
    frame = _BufferImage(source)

    seen = 0
    _fill_constant(source, 0)
    _compare_case(source, asm_output, viper_output, reference_output,
                  state, builder, frame, source_ptr, target_ptr,
                  "constant", 0)
    _fill_constant(source, 255)
    _compare_case(source, asm_output, viper_output, reference_output,
                  state, builder, frame, source_ptr, target_ptr,
                  "constant", 255)
    for position in range(8):
        seen |= _fill_minimum_position(source, position)
        _compare_case(source, asm_output, viper_output, reference_output,
                      state, builder, frame, source_ptr, target_ptr,
                      "minimum-position", position)
    if seen != 0xFF:
        raise AssertionError("not all eight source positions were tested")
    for seed in (0x13579BDF, 0x2468ACE1, 0xC001D00D, 0xDEADBEEF):
        _fill_random(source, seed)
        _compare_case(source, asm_output, viper_output, reference_output,
                      state, builder, frame, source_ptr, target_ptr,
                      "random-seed", seed)

    # Exercise the byte-oriented fallback with a deliberately offset view.
    # The production dispatcher uses this kernel whenever an image buffer's
    # data pointer is unaligned; current MicroPython bytearrays are usually
    # word-aligned, so construct the condition explicitly here.
    misaligned_storage = bytearray(320 * 120 + 4)
    storage_ptr = _uctypes.addressof(misaligned_storage)
    offset = (1 - (storage_ptr & 3)) & 3
    if offset == 0:
        offset = 1
    misaligned = memoryview(misaligned_storage)[offset:offset + 320 * 120]
    if (_uctypes.addressof(misaligned) & 3) == 0:
        raise AssertionError("test view unexpectedly remains aligned")
    _fill_random(misaligned, 0xA5A5F00D)
    fast_gray._min_pool_host(
        misaligned, reference_output, 320, 80, 60, 4, 2)
    fast_gray._min_pool_viper(builder, misaligned, 320)
    fallback_diffs, fallback_first = _first_difference(
        reference_output, builder.luma)
    print("MINPOOL_UNALIGNED_FALLBACK diffs=", fallback_diffs,
          "first=", fallback_first)
    if fallback_diffs:
        raise AssertionError("byte-oriented unaligned fallback differs")

    start = _time.ticks_us()
    for _ in range(repeats):
        fast_gray._min_pool_words_viper(state, source, 320)
    viper_us = _time.ticks_diff(_time.ticks_us(), start)

    start = _time.ticks_us()
    for _ in range(repeats):
        _min_pool_asm(source_ptr, target_ptr)
    asm_us = _time.ticks_diff(_time.ticks_us(), start)
    print("MINPOOL_ASM_BENCH repeats=%d viper_us=%d asm_us=%d" %
          (repeats, viper_us, asm_us))
    return 0
