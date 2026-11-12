"""Otsu equivalence checks against the independent full 256-bin sweep."""

import random
import sys
import unittest
from array import array
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import fast_gray
import thresholding
import config


def _compare(histogram):
    count = sum(histogram)
    value_sum = sum(value * amount
                    for value, amount in enumerate(histogram))
    populated = bytearray(value for value, amount in enumerate(histogram)
                          if amount)
    actual = fast_gray._otsu_native(
        array("I", histogram), populated, 0, len(populated), count, value_sum)
    expected_threshold, expected_counts, expected_means, expected_contrast = \
        thresholding._otsu(histogram, count, value_sum)
    expected = (expected_threshold, expected_counts[0], expected_counts[1],
                expected_means[0], expected_means[1], expected_contrast)
    return actual, expected


class FastGrayOtsuEquivalenceTests(unittest.TestCase):
    def test_cpython_guard_keeps_native_fallback_for_segmented_histograms(self):
        if fast_gray._IS_MICROPYTHON:
            self.skipTest("host-only fallback check")
        self.assertFalse(fast_gray._OTSU_ASM_SUPPORTED)
        builder = fast_gray.FastGrayMasks(config)
        self.assertFalse(builder._otsu_asm_ready)

        hist_base = 256
        nonzero_base = 256
        samples = ((8, 1200), (86, 1400), (220, 1600))
        local_histogram = [0] * 256
        count = 0
        value_sum = 0
        for index, (value, amount) in enumerate(samples):
            builder._histograms[hist_base + value] = amount
            builder._nonzero_values[nonzero_base + index] = value
            local_histogram[value] = amount
            count += amount
            value_sum += value * amount

        actual = builder._otsu_for_block(
            hist_base, nonzero_base, len(samples), count, value_sum)
        threshold, counts, means, contrast = thresholding._otsu(
            local_histogram, count, value_sum)
        expected = (threshold, counts[0], counts[1], means[0], means[1],
                    contrast)
        self.assertEqual(actual, expected)

    def test_random_sparse_and_dense_histograms_match_full_sweep(self):
        rng = random.Random(0xA11CE)
        for case in range(4000):
            histogram = [0] * 256
            if case % 3 == 0:
                bins = rng.sample(range(256), rng.randint(2, 16))
                for value in bins:
                    histogram[value] = rng.randint(1, 250)
            else:
                sample_count = rng.randint(300, 4800)
                for _ in range(sample_count):
                    histogram[rng.randrange(256)] += 1
            actual, expected = _compare(histogram)
            self.assertEqual(actual, expected, "histogram case %d" % case)

    def test_symmetric_two_class_ties_keep_the_earliest_cutoff(self):
        for amount in (1, 2, 3, 7, 31, 255, 2400):
            for low in range(128):
                high = 255 - low
                histogram = [0] * 256
                histogram[low] = amount
                histogram[high] = amount
                actual, expected = _compare(histogram)
                self.assertEqual(actual, expected,
                                 "symmetric bins %d/%d amount %d" %
                                 (low, high, amount))


if __name__ == "__main__":
    unittest.main(verbosity=2)
