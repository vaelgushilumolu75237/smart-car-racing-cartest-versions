"""RAM differential for the production FastGrayMasks ASM Otsu path."""

import config
import fast_gray
import micropython
import time


@micropython.native
def _otsu_reference(histograms, hist_base, count, value_sum):
    low_count = 0
    low_sum = 0
    best_score = -1.0
    best_threshold = 0
    best_low_count = 0
    best_high_count = 0
    best_low_mean = 0.0
    best_high_mean = 0.0
    scale = float(count * count)
    for value in range(256):
        amount = histograms[hist_base + value]
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


def _clear_segment(histograms, hist_base):
    for value in range(256):
        histograms[hist_base + value] = 0


def _fill_segment(histograms, hist_base, style, case):
    _clear_segment(histograms, hist_base)
    if style == "empty":
        return
    if style == "one_bin":
        histograms[hist_base + 90] = 4800
        return
    if style == "symmetric":
        low = 32 + (case & 31)
        high = 255 - low
        amount = 300 + case * 11
        histograms[hist_base + low] = amount
        histograms[hist_base + high] = amount
        return
    if style == "near_tie":
        low = 24 + case % 88
        high = 255 - low
        delta = case % 9 - 4
        histograms[hist_base + low - 1] = 1 + case % 7
        histograms[hist_base + low] = 2200 + delta
        histograms[hist_base + low + 1] = 1 + (case * 7) % 5
        histograms[hist_base + high - 1] = 1 + (case * 3) % 5
        histograms[hist_base + high] = 2200 - delta
        histograms[hist_base + high + 1] = 1 + (case * 5) % 7
        return
    if style == "dense":
        state = (0x2468ACE1 + case * 0x85EBCA6B) & 0xFFFFFFFF
        for _ in range(4800):
            state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
            histograms[hist_base + ((state >> 24) & 0xFF)] += 1
        return
    if style == "sparse":
        for index in range(12):
            value = (case * 17 + index * 21) & 0xFF
            amount = 1 + (case * 37 + index * 53) % 200
            histograms[hist_base + value] += amount
        return
    raise ValueError("unknown histogram style")


def _prepare_segment(builder, hist_base, nonzero_base):
    nonzero_count = 0
    count = 0
    value_sum = 0
    for value in range(256):
        amount = builder._histograms[hist_base + value]
        if amount:
            builder._nonzero_values[nonzero_base + nonzero_count] = value
            nonzero_count += 1
            count += amount
            value_sum += value * amount
    return nonzero_count, count, value_sum


def _check_case(builder, block, style, case):
    hist_base = block * 256
    nonzero_base = hist_base
    _fill_segment(builder._histograms, hist_base, style, case)
    nonzero_count, count, value_sum = _prepare_segment(
        builder, hist_base, nonzero_base)
    expected = _otsu_reference(
        builder._histograms, hist_base, count, value_sum)
    actual = builder._otsu_for_block(
        hist_base, nonzero_base, nonzero_count, count, value_sum)
    if actual != expected:
        print("OTSU_PROD_ASM_DIFF", "block=", block, "style=", style,
              "case=", case, "expected=", expected, "actual=", actual)
        return 1
    return 0


def run():
    if not fast_gray._OTSU_ASM_SUPPORTED:
        raise RuntimeError("OBJ_REPR_C float probe or asm_thumb guard failed")
    builder = fast_gray.FastGrayMasks(config)
    if not builder._otsu_asm_ready:
        raise RuntimeError("production Otsu ASM buffers are unavailable")
    if builder._block_count != 4:
        raise RuntimeError("test requires the four configured histogram blocks")

    cases = (
        (0, "sparse", 5),
        (1, "dense", 17),
        (2, "near_tie", 31),
        (3, "symmetric", 23),
        (0, "empty", 0),
        (0, "one_bin", 0),
        (0, "dense", 41),
        (1, "empty", 0),
        (1, "sparse", 12),
        (2, "one_bin", 0),
        (2, "dense", 64),
        (3, "empty", 0),
        (3, "near_tie", 77),
        (3, "sparse", 19),
    )
    print("OTSU_PROD_ASM_START cases=", len(cases),
          "blocks=", builder._block_count)
    mismatches = 0
    for index, (block, style, case) in enumerate(cases):
        case_mismatches = _check_case(builder, block, style, case)
        mismatches += case_mismatches
        print("OTSU_PROD_ASM_CASE", index, "block=", block,
              "style=", style, "case=", case,
              "diffs=", case_mismatches)
        time.sleep_ms(20)
    print("OTSU_PROD_ASM_SUMMARY cases=%d mismatches=%d" %
          (len(cases), mismatches))
    if mismatches:
        raise AssertionError("production ASM Otsu differs from native reference")
    return 0


if __name__ == "__main__":
    run()
