"""Bounded OpenMV RAM comparison of pre-change and candidate native Otsu.

Run on the camera with fast_gray.py available in the current working directory.
The old and candidate implementations below are both decorated with the real
MicroPython native compiler. This prints only mismatches and one final row.
"""

from array import array
try:
    import micropython
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import fast_gray
    micropython = fast_gray.micropython
else:
    import fast_gray


@micropython.native
def _otsu_old_native(histograms, nonzero_values, nonzero_count: int,
                     count: int, value_sum: int):
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
        score = (low_count * high_count / scale) * difference * difference
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


def _compare(histogram, nonzero_values):
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
    old = _otsu_old_native(histogram, nonzero_values,
                           nonzero_count, count, value_sum)
    production = fast_gray._otsu_native(
        histogram, nonzero_values, 0, nonzero_count, count, value_sum)
    return old, production


def run():
    histogram = array("I", [0] * 256)
    nonzero_values = bytearray(256)
    mismatches = 0
    production_mismatches = 0
    cases = 0
    for style, builder, limit in (
            ("sparse", _build_sparse, 256),
            ("dense", _build_dense, 1024),
            ("symmetric", _build_symmetric, 208),
            ("near_tie", _build_near_tie, 512)):
        for case in range(limit):
            _clear(histogram)
            builder(histogram, case)
            old, production = _compare(histogram, nonzero_values)
            cases += 1
            production_differs = production != old
            if production_differs:
                mismatches += 1
                production_mismatches += 1 if production_differs else 0
                if mismatches <= 8:
                    print("OTSU_DIFF", style, case,
                          "old=", old,
                          "production=", production)
    print("OTSU_DIFF_SUMMARY cases=%d mismatches=%d" %
          (cases, production_mismatches))
    return mismatches


if run():
    raise AssertionError("Otsu native implementations differ")
