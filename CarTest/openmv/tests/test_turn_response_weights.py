"""Turn-response weighting and expanded-ROI host regressions.

These tests use synthetic grayscale buffers and host masks. They do not claim
camera, OpenMV-board, H743, or vehicle validation.
"""

import sys
import types
import unittest
from array import array
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import fast_boundary
import fast_gray
import fast_path
import main


WIDTH = config.SURFACE_GRID_WIDTH
HEIGHT = config.SURFACE_GRID_HEIGHT
FLOOR = config.SURFACE_ROI_FLOOR_Y


class _BufferImage:
    def __init__(self, raw):
        self.raw = raw

    @staticmethod
    def width():
        return 160

    @staticmethod
    def height():
        return 120

    def bytearray(self):
        return self.raw


def _corridor_image():
    raw = bytearray(160 * 120)
    for grid_y in range(HEIGHT):
        for grid_x in range(WIDTH):
            value = (220 if grid_y >= FLOOR and 24 <= grid_x <= 48
                     else 25)
            top = grid_y * 2 * 160 + grid_x * 2
            raw[top] = value
            raw[top + 1] = value
            raw[top + 160] = value
            raw[top + 161] = value
    return _BufferImage(raw)


def _path_candidate(rows, reference_y=40, reference_x=39.5):
    packed = array("i")
    for y, left, right in rows:
        packed.extend((y, left, right, 7))
    return {
        "valid": True,
        "quality": 90,
        "reason": "OK",
        "reference_y": reference_y,
        "reference_center_x": reference_x,
        "selected_row_ints": packed,
        "selected_row_count": len(rows),
        "selected_row_stride": 4,
    }


class TurnResponseWeightTests(unittest.TestCase):
    def setUp(self):
        self.workspace = main._FastRowWeightWorkspace()

    def test_new_package_identity_and_full_frame_split(self):
        self.assertEqual(config.BUILD_ID,
                         "CACHED_WIDTH_BEND_PREVIEW_20261015")
        self.assertEqual(config.SURFACE_ROI_FLOOR_Y, 8)
        self.assertEqual(config.STEERING_SPLIT_IMAGE_Y, 36)
        self.assertEqual(config.STEERING_SPLIT_GRID_Y, 18)
        self.assertEqual(config.STEERING_SPLIT_GRID_Y *
                         config.SURFACE_PIXEL_SCALE,
                         config.STEERING_SPLIT_IMAGE_Y)
        self.assertEqual(config.STEERING_FAR_WEIGHT, 0.20)
        self.assertEqual(config.STEERING_NEAR_WEIGHT, 0.80)
        self.assertEqual(len(config.GRAY_THRESHOLD_BLOCKS), 4)
        self.assertEqual(config.GRAY_THRESHOLD_BLOCKS, (
            (0, 8, 80, 20), (0, 20, 80, 32),
            (0, 32, 80, 46), (0, 46, 80, 60)))
        self.assertEqual(main.SINGLEEDGE_ROUTE_BUILD_IDS, (
            ("cached_width", "CACHED_WIDTH_BEND_PREVIEW_20261015"),
            ("clipped_midpoint", "CLIPPED_MIDPOINT_RING_20261014")))
        self.assertTrue(main._validate_fast_config())

    def test_actual_gray_builder_and_path_accept_upper_rows_from_new_roi(self):
        masks = fast_gray.FastGrayMasks(config).build(_corridor_image())
        self.assertEqual(masks["floor"], 8)
        self.assertEqual(masks["block_count"], 4)
        self.assertEqual(masks["trusted_block_count"], 4)
        self.assertFalse(any(masks["known_region"][:8 * WIDTH]))
        self.assertTrue(all(masks["known_region"][8 * WIDTH:]))

        path = fast_path.FastPathDetector(config).detect(masks)
        self.assertTrue(path["valid"], path.get("reason"))
        row_y = [path["selected_row_ints"][index * 4]
                 for index in range(path["selected_row_count"])]
        self.assertTrue(any(y < config.STEERING_SPLIT_GRID_Y for y in row_y))
        validator = fast_boundary.FastBoundaryEvidence(config)
        validator.gate_fast_path_candidate(
            path, masks["luma"], masks["edge_mask"], masks["road_mask"],
            masks["known_region"], config)
        self.assertTrue(path["valid"], path.get("reason"))
        self.assertEqual(path["boundary_support_count"],
                         path["selected_row_count"])
        self.assertTrue(any(path["boundary_support_rows"][index]
                            for index, y in enumerate(row_y)
                            if y < config.STEERING_SPLIT_GRID_Y))

    def test_uneven_row_counts_keep_exact_group_shares(self):
        # One far row versus four near rows must still mean 20/80, not 1/5.
        rows = [(10, 10, 30)]
        rows.extend((y, 40, 60) for y in (18, 24, 40, 58))
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray([1] * len(rows))
        candidate["boundary_support_count"] = len(rows)
        selection = {"valid": True, "source": "road", "result": candidate}

        info = self.workspace.aggregate(selection, candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual((info["far_count"], info["near_count"]), (1, 4))
        self.assertEqual((info["far_weight"], info["near_weight"]),
                         (0.2, 0.8))
        self.assertAlmostEqual(info["center_x"], 44.0)
        self.assertEqual(info["raw_error"],
                         main._normalize_fast_reference(44.0, WIDTH))

    def test_missing_far_rows_renormalize_without_inventing_far_weight(self):
        rows = [(10, 10, 30)]
        rows.extend((y, 40, 60) for y in (18, 24, 40, 58))
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray((0, 1, 1, 1, 1))
        candidate["boundary_support_count"] = len(rows)
        selection = {"valid": True, "source": "road", "result": candidate}

        info = self.workspace.aggregate(selection, candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["far_count"], 0)
        self.assertEqual(info["near_count"], 4)
        self.assertEqual(info["missing_group"], "FAR")
        self.assertEqual((info["far_weight"], info["near_weight"]),
                         (0.0, 1.0))
        self.assertEqual(info["center_x"], 50.0)
        self.assertEqual(info["raw_error"],
                         main._normalize_fast_reference(50.0, WIDTH))

    def test_both_single_route_modes_use_same_weighted_rows(self):
        rows = array("i", [
            10, 40, 20, 0, 3,
            18, 100, 30, 0, 3,
            45, 100, 30, 0, 3,
        ])
        result = {"selected_row_ints": rows, "selected_row_count": 3,
                   "selected_row_stride": 5}
        seen_errors = []
        for source in ("single_left", "single_right"):
            selection = {"valid": True, "source": source, "result": result}
            info = self.workspace.aggregate(selection, {}, config)
            self.assertTrue(info["valid"], info)
            self.assertEqual((info["far_count"], info["near_count"]), (1, 2))
            self.assertEqual((info["far_weight"], info["near_weight"]),
                             (0.2, 0.8))
            seen_errors.append(info["raw_error"])
        self.assertEqual(seen_errors[0], seen_errors[1])
        self.assertEqual(seen_errors[0],
                         main._normalize_fast_reference(44.0, WIDTH))

    def test_global_75_percent_gate_does_not_weight_unsupported_far_row(self):
        candidate = _path_candidate([
            (12, 24, 48), (39, 24, 48), (40, 24, 48), (41, 24, 48)],
            reference_y=40)
        pixels = WIDTH * HEIGHT
        luma = bytearray([25]) * pixels
        dark = bytearray([1]) * pixels
        white = bytearray(pixels)
        known = bytearray([1]) * pixels
        for y in (12, 39, 40, 41):
            row = y * WIDTH
            for x in range(24, 49):
                white[row + x] = 1
                dark[row + x] = 0
                luma[row + x] = 220
        # The upper/far row has no current left dark-to-white step. The other
        # three rows keep the existing global support gate exactly at 75%.
        dark[12 * WIDTH + 23] = 0
        validator = fast_boundary.FastBoundaryEvidence(config)
        validator.gate_fast_path_candidate(
            candidate, luma, dark, white, known, config)
        self.assertTrue(candidate["valid"], candidate.get("reason"))
        self.assertEqual(candidate["boundary_evidence"]["bilateral_supported"], 3)
        self.assertEqual(candidate["boundary_evidence"]["pairs_tested"], 4)
        self.assertEqual(list(candidate["boundary_support_rows"][:4]),
                         [0, 1, 1, 1])

        info = self.workspace.aggregate(
            {"valid": True, "source": "road", "result": candidate},
            candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["far_count"], 0)
        self.assertEqual(info["near_count"], 3)
        self.assertEqual(info["missing_group"], "FAR")
        self.assertEqual((info["far_weight"], info["near_weight"]),
                         (0.0, 1.0))

    def test_invalid_or_ambiguous_selection_cannot_reuse_row_data(self):
        rows = [(10, 10, 30), (40, 40, 60)]
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray([1, 1])
        candidate["boundary_support_count"] = 2
        selection = {"valid": False, "source": "NONE",
                     "reason": "AMBIGUOUS_PATH", "result": None}
        info = self.workspace.aggregate(selection, candidate, config)
        self.assertFalse(info["valid"])
        self.assertEqual(info["reason"], "SELECTION_INVALID")
        self.assertEqual(info["supported_rows"], 0)
        self.assertEqual(info["raw_error"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
