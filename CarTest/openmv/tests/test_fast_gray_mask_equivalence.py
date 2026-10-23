"""Full output-plane equivalence tests for the reusable grayscale builder."""

import sys
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import fast_gray
import grayscale_adapter


class _Settings:
    SURFACE_GRID_WIDTH = 80
    SURFACE_GRID_HEIGHT = 60
    SURFACE_PIXEL_SCALE_X = 4
    SURFACE_PIXEL_SCALE_Y = 2
    SURFACE_ROI_FLOOR_Y = 8
    GRAY_THRESHOLD_MIN_CONTRAST = 18.0
    GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
    # Leave gaps and use unequal block shapes so the sentinel area is tested.
    GRAY_THRESHOLD_BLOCKS = (
        (0, 8, 32, 30),
        (40, 8, 80, 30),
        (0, 35, 80, 60),
    )


class _BufferImage:
    def __init__(self, raw):
        self._raw = raw

    def width(self):
        return 320

    def height(self):
        return 120

    def bytearray(self):
        return self._raw


def _expanded_frame(cell_values):
    """Expand an 80x60 byte plane into uniform 4x2 source cells."""
    raw = bytearray(320 * 120)
    for y in range(60):
        source_row = (y * 2) * 320
        source_next_row = source_row + 320
        cell_row = y * 80
        for x in range(80):
            value = cell_values[cell_row + x]
            source_x = x * 4
            for dx in range(4):
                raw[source_row + source_x + dx] = value
                raw[source_next_row + source_x + dx] = value
    return raw


def _two_tone_values(constant_blocks=()):
    values = bytearray([90] * (80 * 60))
    for block_index, (x0, y0, x1, y1) in enumerate(
            _Settings.GRAY_THRESHOLD_BLOCKS):
        if block_index in constant_blocks:
            continue
        split = x0 + (x1 - x0) // 2
        for y in range(y0, y1):
            row = y * 80
            for x in range(x0, x1):
                values[row + x] = 24 if x < split else 224
    return values


def _noise_values(seed):
    values = bytearray(80 * 60)
    state = seed
    for index in range(len(values)):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        values[index] = (state >> 16) & 0xFF
    return values


class FastGrayMaskEquivalenceTests(unittest.TestCase):
    def setUp(self):
        self.builder = fast_gray.FastGrayMasks(_Settings)

    def test_cpython_keeps_word_clear_fast_path_disabled(self):
        self.assertFalse(self.builder._mask_local_counts_ready)

    def _assert_matches_reference(self, raw):
        expected_road, expected_edge, expected_known, expected_info = \
            grayscale_adapter.build_masks(_BufferImage(raw), _Settings)
        actual = self.builder.build(_BufferImage(raw))

        self.assertEqual(bytes(actual["road_mask"]), bytes(expected_road))
        self.assertEqual(bytes(actual["edge_mask"]), bytes(expected_edge))
        self.assertEqual(bytes(actual["known_region"]), bytes(expected_known))
        self.assertEqual(bytes(actual["row_counts"]),
                         bytes(expected_info["row_counts"]))
        self.assertEqual(bytes(actual["road_row_counts"]),
                         bytes(expected_info["road_row_counts"]))
        expected_blocks = expected_info["blocks"]
        comparisons = {
            "valid": expected_info["valid"],
            "complete": expected_info["complete"],
            "reason": expected_info["reason"],
            "trusted_block_count": expected_info["trusted_block_count"],
            "road_pixels": expected_info["road_pixels"],
            "edge_pixels": expected_info["edge_pixels"],
            "thresholds": [block["threshold"] for block in expected_blocks],
            "contrast": [block["contrast"] for block in expected_blocks],
            "classes": [block["classes"] for block in expected_blocks],
            "trusted": [block["trusted"] for block in expected_blocks],
            "contrast_min": expected_info["contrast_min"],
            "contrast_max": expected_info["contrast_max"],
            "threshold_min": expected_info["threshold_min"],
            "threshold_max": expected_info["threshold_max"],
        }
        for key, expected in comparisons.items():
            self.assertEqual(actual[key], expected, key)
        self.assertEqual(bytes(actual["luma"]), bytes(_cell_minima(raw)))

    def test_two_tone_constant_partial_and_random_frames(self):
        frames = (
            _expanded_frame(_two_tone_values()),
            _expanded_frame(bytearray([90] * (80 * 60))),
            _expanded_frame(_two_tone_values(constant_blocks=(1,))),
            _expanded_frame(_noise_values(0x2468ACE1)),
            _expanded_frame(_noise_values(0x10293847)),
        )
        for frame_index, raw in enumerate(frames):
            with self.subTest(frame_index=frame_index):
                self._assert_matches_reference(raw)

    def test_every_call_rewrites_all_reusable_planes_and_counts(self):
        self._assert_matches_reference(_expanded_frame(_two_tone_values()))
        self.builder.road_mask[:] = bytes([17]) * len(self.builder.road_mask)
        self.builder.edge_mask[:] = bytes([19]) * len(self.builder.edge_mask)
        self.builder.known_region[:] = bytes([23]) * len(self.builder.known_region)
        self.builder.row_counts[:] = bytes([29]) * len(self.builder.row_counts)
        self.builder.road_row_counts[:] = bytes([31]) * len(
            self.builder.road_row_counts)
        self._assert_matches_reference(
            _expanded_frame(bytearray([90] * (80 * 60))))


def _cell_minima(raw):
    values = bytearray(80 * 60)
    for y in range(60):
        source_y = y * 2 * 320
        for x in range(80):
            left = x * 4
            minimum = 255
            for dy in range(2):
                base = source_y + dy * 320 + left
                for dx in range(4):
                    value = raw[base + dx]
                    if value < minimum:
                        minimum = value
            values[y * 80 + x] = minimum
    return values


if __name__ == "__main__":
    unittest.main(verbosity=2)
