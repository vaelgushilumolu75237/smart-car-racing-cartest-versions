"""Experimental FPU Otsu sweep for bounded OpenMV RAM validation.

This is an isolated candidate. Production fast_gray._otsu_native remains the
reference and is not changed or called by this script. Run on an OpenMV target
with the sibling openmv/fast_gray.py available; CPython cannot execute the
inline assembler body.
"""

from array import array

import fast_gray

try:
    import micropython
    _HAS_MICROPYTHON_MODULE = True
except ImportError:
    micropython = fast_gray.micropython
    _HAS_MICROPYTHON_MODULE = False

try:
    import time as _time
    import uctypes as _uctypes
    import ustruct as _struct
except ImportError:
    _time = None
    _uctypes = None
    import struct as _struct


@micropython.native
def _otsu_old_native(histograms, nonzero_values, nonzero_count: int,
                     count: int, value_sum: int):
    """Exact pre-optimization native Otsu arithmetic and tie behavior."""
    low_count = 0
    low_sum = 0
    best_score = -1.0
    best_threshold = 0
    best_low_count = 0
    best_high_count = 0
    best_low_mean = 0.0
    best_high_mean = 0.0
    scale = float(count * count)
    for item in range(nonzero_count):
        value = nonzero_values[item]
        amount = histograms[value]
        low_count += amount
        low_sum += value * amount
        high_count = count - low_count
        if low_count == 0 or high_count == 0:
            continue
        high_sum = value_sum - low_sum
        low_mean = low_sum / float(low_count)
        high_mean = high_sum / float(high_count)
        difference = high_mean - low_mean
        score = ((low_count * high_count / scale) * difference) * difference
        if score > best_score:
            best_score = score
            best_threshold = value
            best_low_count = low_count
            best_high_count = high_count
            best_low_mean = low_mean
            best_high_mean = high_mean
    contrast = best_high_mean - best_low_mean
    return (best_threshold, best_low_count, best_high_count,
            best_low_mean, best_high_mean, contrast)


_ASM_AVAILABLE = (_HAS_MICROPYTHON_MODULE and _uctypes is not None)

if _ASM_AVAILABLE:
    @micropython.asm_thumb
    def _otsu_threshold_asm(r0, r1, r2, r3):
        # r0=histogram, r1=nonzero values, r2=[count,sum,nz_count],
        # r3=16-word workspace. The inline-assembler wrapper saves/restores
        # r1-r7 and lr around this body.
        ldr(r4, [r2, 0])
        ldr(r5, [r2, 4])
        ldr(r6, [r2, 8])
        str(r0, [r3, 24])
        str(r1, [r3, 28])
        mov(r0, 0)
        str(r0, [r3, 4])
        str(r0, [r3, 8])
        str(r0, [r3, 12])
        mov(r7, 0)

        label(loop)
        cmp(r7, r6)
        blt(process)
        b(done)

        label(process)

        # Accumulate the next populated value into the low class.
        ldr(r0, [r3, 28])
        add(r0, r0, r7)
        ldrb(r1, [r0, 0])
        str(r1, [r3, 16])
        lsl(r2, r1, 2)
        ldr(r0, [r3, 24])
        add(r0, r0, r2)
        ldr(r1, [r0, 0])
        str(r1, [r3, 20])
        ldr(r0, [r3, 4])
        add(r0, r0, r1)
        str(r0, [r3, 4])
        ldr(r2, [r3, 8])
        ldr(r0, [r3, 16])
        ldr(r1, [r3, 20])
        mul(r0, r1)
        add(r2, r2, r0)
        str(r2, [r3, 8])

        # Empty high class is skipped exactly as in the native reference.
        ldr(r0, [r3, 4])
        sub(r1, r4, r0)
        cmp(r1, 0)
        bne(nonempty_high)
        b(skip)

        label(nonempty_high)
        str(r1, [r3, 36])
        ldr(r0, [r3, 8])
        sub(r0, r5, r0)
        str(r0, [r3, 32])

        # low_mean = low_sum / float(low_count).  OBJ_REPR_C stores inline
        # floats with two payload bits displaced, then reconstructs those bits
        # from bits 2..3 when unboxing.  Reproduce that decode after every
        # boxed float conversion/operation used by the native reference:
        # u = (u & ~3) | ((u >> 2) & 3).
        ldr(r0, [r3, 8])
        vmov(s0, r0)
        vcvt_f32_s32(s0, s0)
        ldr(r0, [r3, 4])
        vmov(s1, r0)
        vcvt_f32_s32(s1, s1)
        mov(r0, 3)
        vmov(r1, s1)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s1, r1)
        vdiv(s0, s0, s1)
        mov(r0, 3)
        vmov(r1, s0)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s0, r1)

        # high_mean = high_sum / float(high_count)
        ldr(r0, [r3, 32])
        vmov(s1, r0)
        vcvt_f32_s32(s1, s1)
        ldr(r0, [r3, 36])
        vmov(s2, r0)
        vcvt_f32_s32(s2, s2)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)
        vdiv(s1, s1, s2)
        mov(r0, 3)
        vmov(r1, s1)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s1, r1)
        vsub(s2, s1, s0)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)

        # Preserve the old left-to-right expression:
        # ((low_count * high_count / float(count * count)) * diff) * diff.
        ldr(r0, [r3, 4])
        ldr(r1, [r3, 36])
        mul(r0, r1)
        vmov(s3, r0)
        vcvt_f32_s32(s3, s3)
        mov(r0, r4)
        mul(r0, r4)
        vmov(s4, r0)
        vcvt_f32_s32(s4, s4)
        # scale is an explicit float() result, so it is boxed and unboxed.
        mov(r0, 3)
        vmov(r1, s4)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s4, r1)
        vdiv(s3, s3, s4)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)
        vmul(s3, s3, s2)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)
        vmul(s3, s3, s2)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)

        # Strict greater-than keeps the earliest cutoff on a score tie.
        vldr(s4, [r3, 0])
        vcmp(s3, s4)
        vmrs(APSR_nzcv, FPSCR)
        ble(skip)
        vstr(s3, [r3, 0])
        ldr(r0, [r3, 16])
        str(r0, [r3, 12])
        ldr(r0, [r3, 4])
        str(r0, [r3, 40])
        ldr(r0, [r3, 36])
        str(r0, [r3, 44])
        vstr(s0, [r3, 48])
        vstr(s1, [r3, 52])
        vstr(s2, [r3, 56])

        label(skip)
        add(r7, r7, 1)
        b(loop)

        label(done)
        # The native result computes contrast after the winning means have
        # been retained, so quantize this final subtraction as well.
        vldr(s0, [r3, 48])
        vldr(s1, [r3, 52])
        vsub(s2, s1, s0)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)
        vstr(s2, [r3, 56])
        ldr(r0, [r3, 12])
else:
    _otsu_threshold_asm = None


_ASM_META = array("I", [0, 0, 0]) if _ASM_AVAILABLE else None
_ASM_WORKSPACE = array("I", [0xBF800000] + [0] * 15) \
    if _ASM_AVAILABLE else None


def _otsu_asm_candidate(histograms, nonzero_values, nonzero_count,
                        count, value_sum):
    if _otsu_threshold_asm is None:
        raise RuntimeError("this script requires OpenMV asm_thumb and uctypes")
    _ASM_META[0] = count
    _ASM_META[1] = value_sum
    _ASM_META[2] = nonzero_count
    # The first word is the IEEE-754 single-precision encoding of -1.0.
    _ASM_WORKSPACE[0] = 0xBF800000
    _ASM_WORKSPACE[3] = 0
    _ASM_WORKSPACE[10] = 0
    _ASM_WORKSPACE[11] = 0
    _ASM_WORKSPACE[12] = 0
    _ASM_WORKSPACE[13] = 0
    _ASM_WORKSPACE[14] = 0
    hist_ptr = _uctypes.addressof(histograms)
    values_ptr = _uctypes.addressof(nonzero_values)
    meta_ptr = _uctypes.addressof(_ASM_META)
    workspace_ptr = _uctypes.addressof(_ASM_WORKSPACE)
    if (hist_ptr | values_ptr | meta_ptr | workspace_ptr) & 3:
        raise RuntimeError("word-addressed Otsu buffers are not aligned")
    threshold = _otsu_threshold_asm(
        hist_ptr, values_ptr, meta_ptr, workspace_ptr)
    low_mean = _struct.unpack_from("<f", _ASM_WORKSPACE, 48)[0]
    high_mean = _struct.unpack_from("<f", _ASM_WORKSPACE, 52)[0]
    contrast = _struct.unpack_from("<f", _ASM_WORKSPACE, 56)[0]
    return (threshold, _ASM_WORKSPACE[10], _ASM_WORKSPACE[11],
            low_mean, high_mean, contrast)


def _next(state):
    return (state * 1664525 + 1013904223) & 0xFFFFFFFF


def _clear(histogram):
    for value in range(256):
        histogram[value] = 0


def _build_sparse(histogram, case):
    state = (0x13579BDF + case * 0x9E3779B9) & 0xFFFFFFFF
    for _ in range(10):
        state = _next(state)
        value = (state >> 16) & 0xFF
        amount = 1 + ((state >> 24) % 240)
        histogram[value] += amount
    histogram[case & 0x7F] += 1 + case % 97
    histogram[128 + (case & 0x7F)] += 1 + (case * 7) % 89


def _build_dense(histogram, case):
    state = (0x2468ACE1 + case * 0x85EBCA6B) & 0xFFFFFFFF
    for _ in range(4800):
        state = _next(state)
        histogram[(state >> 24) & 0xFF] += 1


def _build_symmetric(histogram, case):
    low = case & 0x7F
    high = 255 - low
    amount = 1 + (case * 37) % 2400
    histogram[low] = amount
    histogram[high] = amount


def _build_near_tie(histogram, case):
    low = 24 + case % 88
    high = 255 - low
    delta = case % 9 - 4
    histogram[low - 1] = 1 + case % 7
    histogram[low] = 2200 + delta
    histogram[low + 1] = 1 + (case * 7) % 5
    histogram[high - 1] = 1 + (case * 3) % 5
    histogram[high] = 2200 - delta
    histogram[high + 1] = 1 + (case * 5) % 7


def _prepare(histogram, nonzero_values):
    count = 0
    value_sum = 0
    nonzero_count = 0
    for value in range(256):
        amount = histogram[value]
        if amount:
            nonzero_values[nonzero_count] = value
            nonzero_count += 1
            count += amount
            value_sum += value * amount
    return nonzero_count, count, value_sum


def _compare(histogram, nonzero_values):
    nonzero_count, count, value_sum = _prepare(histogram, nonzero_values)
    old = _otsu_old_native(histogram, nonzero_values,
                           nonzero_count, count, value_sum)
    asm = _otsu_asm_candidate(histogram, nonzero_values,
                              nonzero_count, count, value_sum)
    return old, asm


def _benchmark(histogram, nonzero_values, repeats=300):
    nonzero_count, count, value_sum = _prepare(histogram, nonzero_values)
    start = _time.ticks_us()
    for _ in range(repeats):
        _otsu_old_native(histogram, nonzero_values,
                         nonzero_count, count, value_sum)
    old_us = _time.ticks_diff(_time.ticks_us(), start)
    start = _time.ticks_us()
    for _ in range(repeats):
        _otsu_asm_candidate(histogram, nonzero_values,
                            nonzero_count, count, value_sum)
    asm_us = _time.ticks_diff(_time.ticks_us(), start)
    print("OTSU_ASM_BENCH repeats=%d bins=%d old_us=%d asm_us=%d" %
          (repeats, nonzero_count, old_us, asm_us))


def _run_smoke(histogram, nonzero_values):
    _clear(histogram)
    histogram[32] = 2400
    histogram[220] = 2400
    old, asm = _compare(histogram, nonzero_values)
    print("OTSU_ASM_SMOKE two_bin old=", old, "asm=", asm)
    if old != asm:
        raise AssertionError("two-bin FPU Otsu smoke differs from baseline")

    _clear(histogram)
    _build_dense(histogram, 0)
    old, asm = _compare(histogram, nonzero_values)
    print("OTSU_ASM_SMOKE dense old=", old, "asm=", asm)
    if old != asm:
        raise AssertionError("dense FPU Otsu smoke differs from baseline")


def run(full=False):
    if _otsu_threshold_asm is None or _time is None:
        raise RuntimeError("run this validation on OpenMV MicroPython")
    print("OTSU_ASM_START full=", full)
    if hasattr(_time, "sleep_ms"):
        _time.sleep_ms(100)
    histogram = array("I", [0] * 256)
    nonzero_values = bytearray(256)
    _run_smoke(histogram, nonzero_values)
    if not full:
        print("OTSU_ASM_SMOKE_SUMMARY cases=2 mismatches=0")
        return 0

    mismatch_count = 0
    cases = 0
    for style, builder, limit in (
            ("sparse", _build_sparse, 256),
            ("dense", _build_dense, 1024),
            ("symmetric", _build_symmetric, 208),
            ("near_tie", _build_near_tie, 512)):
        style_mismatches = 0
        for case in range(limit):
            _clear(histogram)
            builder(histogram, case)
            old, asm = _compare(histogram, nonzero_values)
            cases += 1
            if old != asm:
                mismatch_count += 1
                style_mismatches += 1
                if mismatch_count <= 8:
                    print("OTSU_ASM_DIFF", style, case,
                          "old=", old, "asm=", asm)
            if (case + 1) % 64 == 0:
                print("OTSU_ASM_PROGRESS", style, case + 1, "/", limit)
        print("OTSU_ASM_STYLE", style, "cases=", limit,
              "mismatches=", style_mismatches)

    # Exercise stale-workspace clearing after populated dense inputs.
    _clear(histogram)
    old, asm = _compare(histogram, nonzero_values)
    cases += 1
    if old != asm:
        mismatch_count += 1
        print("OTSU_ASM_DIFF", "empty", "old=", old, "asm=", asm)
    else:
        print("OTSU_ASM_DEGENERATE", "empty", "matched")

    _clear(histogram)
    histogram[90] = 4800
    old, asm = _compare(histogram, nonzero_values)
    cases += 1
    if old != asm:
        mismatch_count += 1
        print("OTSU_ASM_DIFF", "one_bin", "old=", old, "asm=", asm)
    else:
        print("OTSU_ASM_DEGENERATE", "one_bin", "matched")

    _clear(histogram)
    _build_dense(histogram, 17)
    _benchmark(histogram, nonzero_values)
    _clear(histogram)
    _build_sparse(histogram, 17)
    _benchmark(histogram, nonzero_values)
    print("OTSU_ASM_SUMMARY cases=%d mismatches=%d" %
          (cases, mismatch_count))
    if mismatch_count:
        raise AssertionError("FPU Otsu candidate differs from native baseline")
    return mismatch_count


if __name__ == "__main__":
    run()
