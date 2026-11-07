"""Fast grayscale mask equivalence and buffer-reuse checks."""

import sys
import unittest
from pathlib import Path


TEST_ROOT = Path(__file__).resolve().parents[1]
OPENMV_DIR = TEST_ROOT / "openmv"
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import config
import fast_gray
import grayscale_adapter
import thresholding


GRID_WIDTH = config.SURFACE_GRID_WIDTH
GRID_HEIGHT = config.SURFACE_GRID_HEIGHT
INPUT_WIDTH = GRID_WIDTH * config.SURFACE_PIXEL_SCALE
INPUT_HEIGHT = GRID_HEIGHT * config.SURFACE_PIXEL_SCALE
PIXELS = INPUT_WIDTH * INPUT_HEIGHT


def _raw_from_grid(grid):
    raw = bytearray(PIXELS)
    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            value = grid[y * GRID_WIDTH + x]
            input_row = y * 2 * INPUT_WIDTH
            input_column = x * 2
            raw[input_row + input_column] = value
            raw[input_row + input_column + 1] = value
            raw[input_row + INPUT_WIDTH + input_column] = value
            raw[input_row + INPUT_WIDTH + input_column + 1] = value
    return raw


def _lane_grid(dark=34, bright=218, floor=20):
    grid = bytearray(GRID_WIDTH * GRID_HEIGHT)
    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            grid[y * GRID_WIDTH + x] = (
                bright if y >= floor and 22 <= x <= 57 else dark)
    return grid


class BufferImage:
    def __init__(self, raw):
        self.raw = raw
        self.bytearray_calls = 0
        self.width_calls = 0
        self.height_calls = 0

    def width(self):
        self.width_calls += 1
        return INPUT_WIDTH

    def height(self):
        self.height_calls += 1
        return INPUT_HEIGHT

    def bytearray(self):
        self.bytearray_calls += 1
        return self.raw

    def get_pixel(self, _coordinates):
        raise AssertionError("valid grayscale bytearray must be used")


class PixelImage:
    """Host-only fake matching the old tuple-coordinate get_pixel API."""

    def __init__(self, pixel_fn, bad_buffer=None):
        self.pixel_fn = pixel_fn
        self.bad_buffer = bad_buffer
        self.reads = 0

    def width(self):
        return INPUT_WIDTH

    def height(self):
        return INPUT_HEIGHT

    def bytearray(self):
        if self.bad_buffer is None:
            raise RuntimeError("no packed frame buffer in host fake")
        return self.bad_buffer

    def get_pixel(self, coordinates):
        if not isinstance(coordinates, tuple) or len(coordinates) != 2:
            raise TypeError("get_pixel requires one (x, y) tuple")
        x, y = coordinates
        self.reads += 1
        return self.pixel_fn(x, y)


class InvalidFormatImage(BufferImage):
    def format(self):
        # Deliberately unknown numeric pixel format with a misleadingly valid
        # one-byte buffer length.  It cannot pass as verified grayscale.
        return 91


class FastGrayMaskTests(unittest.TestCase):
    def setUp(self):
        self.fast = fast_gray.FastGrayMasks(config)

    def _assert_matches_old(self, image, fast=None):
        builder = fast or self.fast
        actual = builder.build(image)
        old_road, old_edge, old_known, old_details = \
            grayscale_adapter.build_masks(image, config)
        self.assertEqual(bytes(actual["luma"]), bytes(old_details["luma"]))
        self.assertEqual(bytes(actual["road_mask"]), bytes(old_road))
        self.assertEqual(bytes(actual["edge_mask"]), bytes(old_edge))
        self.assertEqual(bytes(actual["known_region"]), bytes(old_known))
        self.assertEqual(actual["reason"], old_details["reason"])
        self.assertEqual(actual["trusted_block_count"],
                         old_details["trusted_block_count"])
        self.assertEqual(actual["complete"], old_details["complete"])
        self.assertEqual(actual["valid"], old_details["valid"])
        self.assertEqual(actual["thresholds"],
                         old_details["blocks_thresholds"]
                         if "blocks_thresholds" in old_details else
                         [block["threshold"]
                          for block in old_details["blocks"]])
        self.assertEqual(actual["contrasts"],
                         [block["contrast"]
                          for block in old_details["blocks"]])
        self.assertEqual(actual["classes"],
                         [block["classes"]
                          for block in old_details["blocks"]])
        return actual, old_details

    def test_blue_gray_boundary_and_narrow_black_pixel_match_old_masks(self):
        grid = _lane_grid(dark=31, bright=224)
        # A one-pixel dark marking must survive the 2x2 minimum pool.
        grid[42 * GRID_WIDTH + 40] = 4
        image = BufferImage(_raw_from_grid(grid))
        actual, _old = self._assert_matches_old(image)
        self.assertEqual(actual["luma"][42 * GRID_WIDTH + 40], 4)
        self.assertEqual(actual["sampling_source"], "buffer")
        self.assertEqual(image.bytearray_calls, 2)
        self.assertEqual(image.width_calls, 2)
        self.assertEqual(image.height_calls, 2)

    def test_min_pool_matches_independent_reference_for_distinct_2x2_pixels(self):
        """Host check with four different source pixels in every 2x2 cell.

        This verifies the input/output mapping independently of the helper
        that builds repeated-pixel fixtures. It exercises the CPython path;
        it does not claim to execute or validate OpenMV's Viper kernel.
        """
        raw = bytearray(PIXELS)
        expected = bytearray(GRID_WIDTH * GRID_HEIGHT)
        minimum_positions = set()

        for y in range(GRID_HEIGHT):
            source_y = y * 2 * INPUT_WIDTH
            for x in range(GRID_WIDTH):
                # Keep all four values distinct while rotating the minimum
                # through the four source positions. This catches code that
                # accidentally copies only the top-left sample.
                base = (x * 37 + y * 73 + (x * y) % 83) % 252
                minimum_position = (x + 2 * y) & 3
                minimum_positions.add(minimum_position)
                output_index = y * GRID_WIDTH + x
                for position in range(4):
                    source_x = x * 2 + (position & 1)
                    source_row = source_y + (position >> 1) * INPUT_WIDTH
                    value = base + ((position - minimum_position) & 3)
                    raw[source_row + source_x] = value

                # Independent scalar reference over the four source bytes.
                top = source_y + x * 2
                bottom = top + INPUT_WIDTH
                expected[output_index] = min(
                    raw[top], raw[top + 1], raw[bottom], raw[bottom + 1])

        self.assertEqual(minimum_positions, {0, 1, 2, 3})
        image = BufferImage(raw)
        self.assertEqual(self.fast._read_frame(image), "buffer")
        self.assertEqual(bytes(self.fast.luma), bytes(expected))

    def test_shadow_and_brightness_scaled_frames_match_old_masks(self):
        grid = bytearray(GRID_WIDTH * GRID_HEIGHT)
        for y in range(GRID_HEIGHT):
            shadow = 24 if y < 37 else 60
            bright = 198 if y < 37 else 176
            for x in range(GRID_WIDTH):
                grid[y * GRID_WIDTH + x] = (
                    bright if y >= config.SURFACE_ROI_FLOOR_Y and
                    18 <= x <= 61 else shadow)
        for multiplier, offset in ((1.0, 0), (0.72, 8), (1.08, 0)):
            with self.subTest(multiplier=multiplier, offset=offset):
                scaled = bytearray(
                    min(255, max(0, int(value * multiplier + offset)))
                    for value in grid)
                self._assert_matches_old(
                    BufferImage(_raw_from_grid(scaled)))

    def test_full_white_dark_and_low_contrast_scenes_fail_closed(self):
        scenes = (("white", 255), ("dark", 0))
        for name, value in scenes:
            with self.subTest(name=name):
                actual, _old = self._assert_matches_old(
                    BufferImage(_raw_from_grid(bytearray(
                        [value] * (GRID_WIDTH * GRID_HEIGHT)))))
                self.assertEqual(actual["reason"], "NO_TRUSTED_BLOCK")
                self.assertFalse(actual["valid"])
                self.assertFalse(actual["complete"])
                self.assertFalse(any(actual["known_region"]))

        low = bytearray(GRID_WIDTH * GRID_HEIGHT)
        for y in range(config.SURFACE_ROI_FLOOR_Y, GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                low[y * GRID_WIDTH + x] = 100 if x < 40 else 112
        actual, old = self._assert_matches_old(BufferImage(_raw_from_grid(low)))
        self.assertEqual(actual["reason"], "NO_TRUSTED_BLOCK")
        self.assertTrue(all(block["reason"] == "LOW_CONTRAST"
                            for block in old["blocks"]))
        self.assertFalse(any(actual["known_region"]))

    def test_unknown_middle_block_boundary_stays_unknown(self):
        grid = bytearray(GRID_WIDTH * GRID_HEIGHT)
        for y in range(20, 32):
            for x in range(GRID_WIDTH):
                grid[y * GRID_WIDTH + x] = 25 if x < 40 else 210
        for y in range(32, 46):
            for x in range(GRID_WIDTH):
                grid[y * GRID_WIDTH + x] = 145
        for y in range(46, GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                grid[y * GRID_WIDTH + x] = 40 if x < 35 else 230
        actual, old = self._assert_matches_old(BufferImage(_raw_from_grid(grid)))
        self.assertEqual(actual["reason"], "PARTIAL")
        self.assertEqual(actual["trusted_block_count"], 2)
        self.assertTrue(all(actual["known_region"][20 * GRID_WIDTH:
                                                    32 * GRID_WIDTH]))
        self.assertFalse(any(actual["known_region"][32 * GRID_WIDTH:
                                                     46 * GRID_WIDTH]))
        self.assertTrue(all(actual["known_region"][46 * GRID_WIDTH:]))
        self.assertEqual(old["blocks"][1]["reason"], "CONSTANT")

    def test_histograms_cover_every_byte_value_in_all_three_blocks(self):
        grid = bytearray((index & 0xFF)
                         for index in range(GRID_WIDTH * GRID_HEIGHT))
        actual, _old = self._assert_matches_old(BufferImage(_raw_from_grid(grid)))
        for block, (x0, y0, x1, y1) in enumerate(config.GRAY_THRESHOLD_BLOCKS):
            hist_base = block * 256
            histogram = self.fast._histograms[hist_base:hist_base + 256]
            pixels = (x1 - x0) * (y1 - y0)
            self.assertEqual(min(histogram), pixels // 256)
            self.assertEqual(max(histogram), (pixels + 255) // 256)
            self.assertEqual(sum(histogram), pixels)
            stats_base = block * 4
            self.assertEqual(tuple(self.fast._block_stats[
                stats_base:stats_base + 4]),
                (pixels, sum(value * count for value, count in
                             enumerate(histogram)), 0, 255))
            self.assertEqual(self.fast._nonzero_counts[block], 256)
            self.assertEqual(list(self.fast._nonzero_values[
                hist_base:hist_base + 256]), list(range(256)))
        self.assertTrue(all(value is not None for value in actual["thresholds"]))

    def test_repeated_frames_reuse_arrays_and_clear_all_previous_masks(self):
        ids = (id(self.fast.luma), id(self.fast.road_mask),
               id(self.fast.edge_mask), id(self.fast.known_region),
               id(self.fast._histograms), id(self.fast.row_counts),
               id(self.fast._pixel_totals), id(self.fast._nonzero_values),
               id(self.fast._nonzero_counts), id(self.fast._block_stats))
        first = self.fast.build(BufferImage(_raw_from_grid(_lane_grid())))
        first_road = bytes(first["road_mask"])
        self.assertTrue(any(first["known_region"]))

        uniform = bytearray([200] * (GRID_WIDTH * GRID_HEIGHT))
        second = self.fast.build(BufferImage(_raw_from_grid(uniform)))
        self.assertIs(first, second)
        self.assertEqual(bytes(second["road_mask"]), bytes(GRID_WIDTH * GRID_HEIGHT))
        self.assertEqual(bytes(second["edge_mask"]), bytes(GRID_WIDTH * GRID_HEIGHT))
        self.assertEqual(bytes(second["known_region"]), bytes(GRID_WIDTH * GRID_HEIGHT))
        self.assertEqual(bytes(second["row_counts"]), bytes(GRID_HEIGHT))
        self.assertEqual(bytes(second["road_row_counts"]), bytes(GRID_HEIGHT))

        third = self.fast.build(BufferImage(_raw_from_grid(_lane_grid())))
        self.assertEqual(bytes(third["road_mask"]), first_road)
        self.assertEqual((id(self.fast.luma), id(self.fast.road_mask),
                          id(self.fast.edge_mask), id(self.fast.known_region),
                          id(self.fast._histograms), id(self.fast.row_counts),
                          id(self.fast._pixel_totals),
                          id(self.fast._nonzero_values),
                          id(self.fast._nonzero_counts),
                          id(self.fast._block_stats)),
                         ids)

    def test_all_256_luma_values_match_old_otsu_cutoffs(self):
        grid = bytearray(GRID_WIDTH * GRID_HEIGHT)
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                grid[y * GRID_WIDTH + x] = (x + y * 79) & 0xFF
        image = BufferImage(_raw_from_grid(grid))
        actual, _old = self._assert_matches_old(image)
        for block in range(3):
            counts = self.fast._histograms[block * 256:block * 256 + 256]
            self.assertTrue(all(count > 0 for count in counts))
        self.assertEqual(len(actual["luma"]), 4800)

    def test_host_get_pixel_fallback_is_explicit_and_equivalent(self):
        grid = _lane_grid()

        def pixel(x, y):
            return grid[(y // 2) * GRID_WIDTH + (x // 2)]

        image = PixelImage(pixel, bad_buffer=bytes(PIXELS * 2))
        actual, _old = self._assert_matches_old(image)
        self.assertEqual(actual["sampling"], "min2")
        self.assertEqual(actual["sampling_source"], "host-get-pixel-fallback")
        self.assertEqual(image.reads, PIXELS * 2)

    def test_invalid_advertised_format_and_image_dimensions_are_rejected(self):
        valid_length = bytearray(PIXELS)
        bad_format = InvalidFormatImage(valid_length)
        with self.assertRaisesRegex(ValueError, "not GRAYSCALE|validate"):
            self.fast.build(bad_format)
        self.assertEqual(bad_format.bytearray_calls, 0)

        class WrongSize(BufferImage):
            def width(self):
                return INPUT_WIDTH - 2

        with self.assertRaisesRegex(ValueError, "image size"):
            self.fast.build(WrongSize(valid_length))

    def test_debug_blocks_are_optional_and_stable(self):
        compact = self.fast.build(BufferImage(_raw_from_grid(_lane_grid())))
        self.assertNotIn("blocks", compact)
        debug_builder = fast_gray.FastGrayMasks(config, debug=True)
        first = debug_builder.build(BufferImage(_raw_from_grid(_lane_grid())))
        blocks = first["blocks"]
        records = tuple(id(record) for record in blocks)
        second = debug_builder.build(BufferImage(_raw_from_grid(
            bytearray([255] * (GRID_WIDTH * GRID_HEIGHT)))))
        self.assertIs(blocks, second["blocks"])
        self.assertEqual(tuple(id(record) for record in blocks), records)
        self.assertEqual([block["reason"] for block in blocks],
                         ["CONSTANT"] * 3)

    def test_viper_compilation_is_not_claimed_by_cpython_shim(self):
        if sys.implementation.name == "cpython":
            self.assertEqual(fast_gray.COMPILER_BACKEND, "cpython-shim")
            self.assertFalse(fast_gray._IS_MICROPYTHON)

    def test_configuration_rejects_smoothing_and_nonfixed_geometry(self):
        class Settings:
            SURFACE_GRID_WIDTH = 80
            SURFACE_GRID_HEIGHT = 60
            SURFACE_ROI_FLOOR_Y = 20
            SURFACE_PIXEL_SCALE = 2
            GRAY_THRESHOLD_MIN_CONTRAST = 18
            GRAY_THRESHOLD_SMOOTH_NEIGHBORS = True
            GRAY_THRESHOLD_BLOCKS = config.GRAY_THRESHOLD_BLOCKS

        with self.assertRaisesRegex(ValueError, "SMOOTH_NEIGHBORS"):
            fast_gray.FastGrayMasks(Settings)
        Settings.GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
        Settings.SURFACE_PIXEL_SCALE = 1
        with self.assertRaisesRegex(ValueError, "scale 2"):
            fast_gray.FastGrayMasks(Settings)

        Settings.SURFACE_PIXEL_SCALE = 2
        for bad_floor in (True, -1, 60, 12.0):
            with self.subTest(floor=bad_floor):
                Settings.SURFACE_ROI_FLOOR_Y = bad_floor
                with self.assertRaisesRegex(ValueError, "ROI_FLOOR_Y"):
                    fast_gray.FastGrayMasks(Settings)

    def test_configurable_floor_matches_old_masks_and_clears_unknown_roi(self):
        class CalibratedSettings:
            SURFACE_GRID_WIDTH = 80
            SURFACE_GRID_HEIGHT = 60
            SURFACE_ROI_FLOOR_Y = 12
            SURFACE_PIXEL_SCALE = 2
            GRAY_THRESHOLD_MIN_CONTRAST = 18.0
            GRAY_THRESHOLD_SMOOTH_NEIGHBORS = False
            GRAY_THRESHOLD_BLOCKS = (
                (0, 12, 80, 28),
                (0, 28, 80, 44),
                (0, 44, 80, 60),
            )

        grid = bytearray(GRID_WIDTH * GRID_HEIGHT)
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                if y < 12:
                    value = 180
                elif y < 28:
                    value = 30 if x < 40 else 215
                elif y < 44:
                    value = 150
                else:
                    value = 45 if x < 36 else 228
                grid[y * GRID_WIDTH + x] = value

        image = BufferImage(_raw_from_grid(grid))
        builder = fast_gray.FastGrayMasks(CalibratedSettings)
        actual = builder.build(image)
        old_road, old_edge, old_known, old_details = \
            grayscale_adapter.build_masks(image, CalibratedSettings)
        self.assertEqual(bytes(actual["luma"]), bytes(old_details["luma"]))
        self.assertEqual(bytes(actual["road_mask"]), bytes(old_road))
        self.assertEqual(bytes(actual["edge_mask"]), bytes(old_edge))
        self.assertEqual(bytes(actual["known_region"]), bytes(old_known))
        self.assertEqual(actual["floor"], 12)
        self.assertEqual(actual["reason"], "PARTIAL")
        self.assertFalse(any(actual["known_region"][:12 * GRID_WIDTH]))
        self.assertTrue(all(actual["known_region"][12 * GRID_WIDTH:
                                                    28 * GRID_WIDTH]))
        self.assertFalse(any(actual["known_region"][28 * GRID_WIDTH:
                                                     44 * GRID_WIDTH]))
        self.assertTrue(all(actual["known_region"][44 * GRID_WIDTH:]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
