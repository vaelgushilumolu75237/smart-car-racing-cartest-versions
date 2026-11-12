"""Host differential for the isolated local-count mask candidate."""

import sys
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
TEST_DIR = Path(__file__).resolve().parent
for path in (str(OPENMV_DIR), str(TEST_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

import fast_gray
from ram_gray_mask_asm_candidate import (
    _GapSettings,
    _mask_local_counts_host,
)


class FastGrayMaskLocalCountTests(unittest.TestCase):
    def setUp(self):
        self.builder = fast_gray.FastGrayMasks(_GapSettings)

    def _snapshot(self):
        builder = self.builder
        return (
            bytes(builder.road_mask),
            bytes(builder.edge_mask),
            bytes(builder.known_region),
            bytes(builder.row_counts),
            bytes(builder.road_row_counts),
            tuple(builder._pixel_totals),
        )

    def test_local_row_count_model_matches_reference_with_gaps_and_trust(self):
        builder = self.builder
        for seed in (0x13579BDF, 0x2468ACE1):
            state = seed
            for index in range(len(builder.luma)):
                state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
                builder.luma[index] = (state >> 24) & 0xFF

            for trust_mask in (0xF, 0x5, 0xA, 0x1, 0x0):
                for threshold_base in (0, 127, 255):
                    for block in range(builder._block_count):
                        builder._threshold_values[block] = (
                            threshold_base + block * 53) & 0xFF
                        builder._trusted_values[block] = (
                            trust_mask >> block) & 1

                    fast_gray._mask_host(
                        builder.luma, builder.road_mask, builder.edge_mask,
                        builder.known_region, builder._rectangles,
                        builder._threshold_values, builder._trusted_values,
                        builder.row_counts, builder.road_row_counts,
                        builder._pixel_totals, builder.width, builder.height,
                        builder._block_count)
                    expected = self._snapshot()

                    # Seed every reusable output to ensure both algorithms
                    # clear stale values, including untrusted gaps.
                    builder.road_mask[:] = bytes([17]) * len(builder.road_mask)
                    builder.edge_mask[:] = bytes([19]) * len(builder.edge_mask)
                    builder.known_region[:] = bytes([23]) * len(
                        builder.known_region)
                    builder.row_counts[:] = bytes([29]) * len(
                        builder.row_counts)
                    builder.road_row_counts[:] = bytes([31]) * len(
                        builder.road_row_counts)
                    builder._pixel_totals[0] = 1001
                    builder._pixel_totals[1] = 1003
                    _mask_local_counts_host(builder)
                    self.assertEqual(self._snapshot(), expected,
                                     (seed, trust_mask, threshold_base))


if __name__ == "__main__":
    unittest.main(verbosity=2)
