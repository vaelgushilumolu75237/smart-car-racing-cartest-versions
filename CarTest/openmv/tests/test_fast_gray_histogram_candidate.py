"""Host differential for the isolated flat histogram-pass candidate."""

import sys
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
TEST_DIR = Path(__file__).resolve().parent
for path in (str(OPENMV_DIR), str(TEST_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

import config
import fast_gray
from ram_gray_histogram_asm_candidate import (
    _fill_luma,
    _histogram_candidate_host,
    _seed_scratch,
    _snapshot,
)


class _PartialWidthSettings:
    SURFACE_GRID_WIDTH = 80
    SURFACE_GRID_HEIGHT = 60
    SURFACE_PIXEL_SCALE_X = 4
    SURFACE_PIXEL_SCALE_Y = 2
    SURFACE_ROI_FLOOR_Y = 8
    GRAY_THRESHOLD_MIN_CONTRAST = 18.0
    GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
    GRAY_THRESHOLD_BLOCKS = (
        (3, 8, 77, 20),
        (0, 24, 80, 36),
    )


class FastGrayHistogramCandidateTests(unittest.TestCase):
    def _assert_model_matches_reference(self, builder, style, seed, expected_use):
        _fill_luma(builder, style, seed)
        _seed_scratch(builder)
        fast_gray._histogram_host(
            builder.luma, builder._histograms, builder._rectangles,
            builder.width, builder._block_count, builder._nonzero_values,
            builder._nonzero_counts, builder._block_stats)
        expected = _snapshot(builder)

        _seed_scratch(builder)
        used = _histogram_candidate_host(builder)
        self.assertEqual(used, expected_use)
        self.assertEqual(_snapshot(builder), expected,
                         (style, seed, expected_use))

    def test_full_width_model_matches_all_bins_and_stats_after_reuse(self):
        builder = fast_gray.FastGrayMasks(config)
        for style, seed in (
                ("random", 0x13579BDF),
                ("random", 0x2468ACE1),
                ("random", 0xC001D00D),
                ("constant", 0),
                ("constant", 90),
                ("constant", 255)):
            with self.subTest(style=style, seed=seed):
                self._assert_model_matches_reference(
                    builder, style, seed, True)

    def test_partial_width_blocks_keep_generic_fallback(self):
        builder = fast_gray.FastGrayMasks(_PartialWidthSettings)
        for style, seed in (("random", 0xDEADBEEF), ("constant", 73)):
            with self.subTest(style=style, seed=seed):
                self._assert_model_matches_reference(
                    builder, style, seed, False)

    def test_production_guard_accepts_only_full_width_flat_spans(self):
        full_width = fast_gray.FastGrayMasks(config)
        partial_width = fast_gray.FastGrayMasks(_PartialWidthSettings)
        self.assertTrue(fast_gray._histogram_asm_geometry_supported(
            full_width._rectangles, full_width.width,
            full_width._block_count))
        self.assertFalse(fast_gray._histogram_asm_geometry_supported(
            partial_width._rectangles, partial_width.width,
            partial_width._block_count))
        empty_rectangles = (0, 8, 80, 8, 0, 8, 80, 8)
        self.assertTrue(fast_gray._histogram_asm_geometry_supported(
            empty_rectangles, 80, 2))
        self.assertFalse(full_width._histogram_asm_ready)
        self.assertFalse(partial_width._histogram_asm_ready)

    def test_production_host_path_reuses_histogram_scratch_exactly(self):
        builder = fast_gray.FastGrayMasks(config)
        for style, seed in (
                ("random", 0x13579BDF),
                ("constant", 0),
                ("constant", 255),
                ("random", 0x2468ACE1)):
            with self.subTest(style=style, seed=seed):
                _fill_luma(builder, style, seed)
                _seed_scratch(builder)
                fast_gray._histogram_host(
                    builder.luma, builder._histograms, builder._rectangles,
                    builder.width, builder._block_count,
                    builder._nonzero_values, builder._nonzero_counts,
                    builder._block_stats)
                expected = _snapshot(builder)

                _seed_scratch(builder)
                builder._calculate_thresholds()
                self.assertEqual(_snapshot(builder), expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
