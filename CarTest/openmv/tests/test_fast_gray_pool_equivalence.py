"""Independent 4x2 grayscale min-pool equivalence tests.

These host tests exercise the CPython implementation and compare it with a
separate scalar reference. They do not execute OpenMV's Viper machine code.
"""

import sys
import struct
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import config
import fast_gray


GRID_WIDTH = config.SURFACE_GRID_WIDTH
GRID_HEIGHT = config.SURFACE_GRID_HEIGHT
SCALE_X = config.SURFACE_PIXEL_SCALE_X
SCALE_Y = config.SURFACE_PIXEL_SCALE_Y
SOURCE_WIDTH = GRID_WIDTH * SCALE_X
SOURCE_HEIGHT = GRID_HEIGHT * SCALE_Y
SOURCE_SIZE = SOURCE_WIDTH * SOURCE_HEIGHT


class _BufferImage:
    def __init__(self, raw):
        self._raw = raw

    def width(self):
        return SOURCE_WIDTH

    def height(self):
        return SOURCE_HEIGHT

    def bytearray(self):
        return self._raw


def _reference_min_pool(raw):
    """Independent nested-loop reference that examines every 4x2 cell."""
    result = bytearray(GRID_WIDTH * GRID_HEIGHT)
    for y in range(GRID_HEIGHT):
        source_y = y * SCALE_Y * SOURCE_WIDTH
        for x in range(GRID_WIDTH):
            source_x = x * SCALE_X
            minimum = 255
            for dy in range(SCALE_Y):
                row = source_y + dy * SOURCE_WIDTH + source_x
                for dx in range(SCALE_X):
                    value = raw[row + dx]
                    if value < minimum:
                        minimum = value
            result[y * GRID_WIDTH + x] = minimum
    return result


def _aligned_word_min_pool(raw):
    """Host model of the little-endian pair-of-word Viper kernel."""
    result = bytearray(GRID_WIDTH * GRID_HEIGHT)
    for y in range(GRID_HEIGHT):
        source_top = y * SCALE_Y * SOURCE_WIDTH
        source_bottom = source_top + SOURCE_WIDTH
        for x in range(GRID_WIDTH):
            source_x = x * SCALE_X
            top_word = struct.unpack_from(
                "<I", raw, source_top + source_x)[0]
            bottom_word = struct.unpack_from(
                "<I", raw, source_bottom + source_x)[0]
            minimum = 255
            for word in (top_word, bottom_word):
                for shift in (0, 8, 16, 24):
                    value = (word >> shift) & 0xFF
                    if value < minimum:
                        minimum = value
            result[y * GRID_WIDTH + x] = minimum
    return result


def _rotating_minimum_frame():
    """Make every source position win repeatedly, with distinct cell values."""
    raw = bytearray(SOURCE_SIZE)
    seen = set()
    for y in range(GRID_HEIGHT):
        top = y * SCALE_Y * SOURCE_WIDTH
        for x in range(GRID_WIDTH):
            left = x * SCALE_X
            minimum_position = (x + 3 * y) & 7
            seen.add(minimum_position)
            minimum = (x * 17 + y * 29) % 248
            for position in range(8):
                source_row = top + (position // 4) * SOURCE_WIDTH
                source_column = left + position % 4
                raw[source_row + source_column] = (
                    minimum + ((position - minimum_position) & 7))
    return raw, seen


def _deterministic_noise_frame():
    raw = bytearray(SOURCE_SIZE)
    state = 0x13579BDF
    for index in range(SOURCE_SIZE):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        raw[index] = (state >> 16) & 0xFF
    return raw


class FastGrayPoolEquivalenceTests(unittest.TestCase):
    def setUp(self):
        self.builder = fast_gray.FastGrayMasks(config)
        self.luma = self.builder.luma

    def _assert_frame_matches_reference(self, raw):
        expected = _reference_min_pool(raw)
        source = _BufferImage(raw)
        self.assertEqual(self.builder._read_frame(source), "buffer")
        self.assertIs(self.builder.luma, self.luma)
        self.assertEqual(bytes(self.builder.luma), bytes(expected))

    def test_cpython_keeps_the_native_and_assembly_paths_disabled(self):
        self.assertFalse(fast_gray._MIN_POOL_ASM_SUPPORTED)
        self.assertFalse(self.builder._min_pool_asm_ready)
        self.assertIsNone(self.builder._min_pool_target_ptr)

    def test_all_eight_source_positions_can_determine_the_output(self):
        raw, minimum_positions = _rotating_minimum_frame()
        self.assertEqual(minimum_positions, set(range(8)))
        self._assert_frame_matches_reference(raw)

    def test_deterministic_full_range_noise_matches_independent_reference(self):
        self._assert_frame_matches_reference(_deterministic_noise_frame())

    def test_aligned_word_byte_extraction_matches_all_eight_source_samples(self):
        rotating, minimum_positions = _rotating_minimum_frame()
        self.assertEqual(minimum_positions, set(range(8)))
        for raw in (rotating, _deterministic_noise_frame()):
            self.assertEqual(bytes(_aligned_word_min_pool(raw)),
                             bytes(_reference_min_pool(raw)))

    def test_reused_builder_replaces_every_previous_luma_pixel(self):
        first = bytearray([255] * SOURCE_SIZE)
        second = _deterministic_noise_frame()
        self._assert_frame_matches_reference(first)
        self._assert_frame_matches_reference(second)


if __name__ == "__main__":
    unittest.main(verbosity=2)
