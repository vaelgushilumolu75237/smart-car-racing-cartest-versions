"""Turn-response weighting and expanded-ROI host regressions.

These tests use synthetic grayscale buffers and host masks. They do not claim
camera, OpenMV-board, H743, or vehicle validation.
"""

import sys
import types
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


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


class _FullClearReferenceWorkspace(main._FastRowWeightWorkspace):
    """Test-only reference for the pre-optimization 60-row array clear."""

    def clear(self, reason="NO_ACCEPTED_ROWS"):
        # The production clear() has already recorded touched rows from the
        # previous call. A full reset subsumes that short clear list.
        self._preview_touched_count = 0
        info = super().clear(reason)
        for index in range(len(self._row_weight)):
            self._row_weight[index] = 0
            self._preview_valid_rows[index] = 0
        return info


class TurnResponseWeightTests(unittest.TestCase):
    def setUp(self):
        self.workspace = main._FastRowWeightWorkspace()

    def test_new_package_identity_and_full_frame_split(self):
        self.assertEqual(config.BUILD_ID,
                         "BOTTOM80_POSPD_20261020")
        self.assertEqual(config.SURFACE_ROI_FLOOR_Y, 8)
        self.assertEqual(config.STEERING_SPLIT_IMAGE_Y, 24)
        self.assertEqual(config.STEERING_SPLIT_GRID_Y, 12)
        self.assertEqual(config.STEERING_SPLIT_GRID_Y *
                         config.SURFACE_PIXEL_SCALE,
                         config.STEERING_SPLIT_IMAGE_Y)
        self.assertEqual(config.STEERING_FAR_WEIGHT, 0.20)
        self.assertEqual(config.STEERING_NEAR_WEIGHT, 0.80)
        self.assertEqual(config.STEERING_FAR_MIN_SUPPORT_ROWS, 3)
        self.assertEqual(config.STEERING_FAR_MIN_SUPPORT_SPAN, 2)
        self.assertEqual(config.TURN_RESPONSE_VERSION,
                         "BOTTOM80_TOP20_Y24_V2")
        self.assertEqual(len(config.GRAY_THRESHOLD_BLOCKS), 4)
        self.assertEqual(config.GRAY_THRESHOLD_BLOCKS, (
            (0, 8, 80, 20), (0, 20, 80, 32),
            (0, 32, 80, 46), (0, 46, 80, 60)))
        self.assertEqual(main.SINGLEEDGE_ROUTE_BUILD_IDS, (
            ("cached_width", "BOTTOM80_NO_PD_20261020"),
            ("cached_width", "BOTTOM80_POSPD_20261020"),
            ("cached_width", "BOTTOM80_PD_PREVIEW_20261020"),
            ("clipped_midpoint", "CLIPPED_MIDPOINT_RING_20261014")))
        self.assertTrue(main._validate_fast_config())

    def test_control_profile_selector_accepts_only_published_profiles(self):
        with patch.object(config, "STEERING_CONTROL_PROFILE", None):
            with self.assertRaisesRegex(ValueError,
                                        "STEERING_CONTROL_PROFILE"):
                main._validate_fast_config()

        profile_builds = dict(config.STEERING_CONTROL_PROFILE_BUILD_IDS)
        for profile in ("no_pd", "pd", "pd_preview"):
            with patch.object(config, "STEERING_CONTROL_PROFILE", profile), \
                    patch.object(config, "BUILD_ID", profile_builds[profile]):
                self.assertTrue(main._validate_fast_config())
        with patch.object(config, "STEERING_CONTROL_PROFILE", "unknown"):
            with self.assertRaisesRegex(ValueError, "STEERING_CONTROL_PROFILE"):
                main._validate_fast_config()

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

    def test_insufficient_far_support_falls_back_to_near_only(self):
        # A single far row cannot claim the configured 20% share.
        rows = [(10, 10, 30)]
        rows.extend((y, 40, 60) for y in (18, 24, 40, 58))
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray([1] * len(rows))
        candidate["boundary_support_count"] = len(rows)
        selection = {"valid": True, "source": "road", "result": candidate}

        info = self.workspace.aggregate(selection, candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual((info["far_count"], info["near_count"]), (1, 4))
        self.assertFalse(info["far_support_ok"])
        self.assertEqual(info["weight_mode"], "NEAR_ONLY")
        self.assertEqual((info["far_weight"], info["near_weight"]),
                         (0.0, 1.0))
        self.assertAlmostEqual(info["center_x"], 50.0)
        self.assertEqual(info["raw_error"],
                         main._normalize_fast_reference(50.0, WIDTH))

    def test_far_share_requires_three_supported_rows_spanning_two_rows(self):
        rows = [(8, 10, 30), (9, 10, 30), (10, 10, 30)]
        rows.extend((y, 40, 60) for y in (18, 24, 40, 58))
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray([1] * len(rows))
        candidate["boundary_support_count"] = len(rows)
        info = self.workspace.aggregate(
            {"valid": True, "source": "road"}, candidate, config)
        self.assertTrue(info["valid"], info)
        self.assertEqual(info["far_count"], 3)
        self.assertEqual(info["far_span"], 2)
        self.assertTrue(info["far_support_ok"])
        self.assertEqual(info["weight_mode"], "NEAR_FAR_80_20")
        self.assertEqual((info["far_weight"], info["near_weight"]),
                         (0.2, 0.8))

        close_rows = [(8, 10, 30), (9, 10, 30), (9, 10, 30)]
        close_rows.extend((y, 40, 60) for y in (18, 24, 40, 58))
        close_candidate = _path_candidate(close_rows)
        close_candidate["boundary_support_rows"] = bytearray(
            [1] * len(close_rows))
        close_candidate["boundary_support_count"] = len(close_rows)
        close_info = self.workspace.aggregate(
            {"valid": True, "source": "road"}, close_candidate, config)
        self.assertTrue(close_info["valid"], close_info)
        self.assertFalse(close_info["far_support_ok"])
        self.assertEqual((close_info["far_weight"], close_info["near_weight"]),
                         (0.0, 1.0))

    def test_no_near_rows_invalidates_the_position(self):
        rows = [(8, 40, 60), (9, 40, 60), (10, 40, 60)]
        candidate = _path_candidate(rows)
        candidate["boundary_support_rows"] = bytearray([1] * len(rows))
        candidate["boundary_support_count"] = len(rows)
        info = self.workspace.aggregate(
            {"valid": True, "source": "road"}, candidate, config)
        self.assertFalse(info["valid"])
        self.assertEqual(info["reason"], "NO_NEAR_SUPPORTED_ROWS")
        self.assertEqual(info["missing_group"], "NEAR")

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
                             (0.0, 1.0))
            seen_errors.append(info["raw_error"])
        self.assertEqual(seen_errors[0], seen_errors[1])
        self.assertEqual(seen_errors[0],
                         main._normalize_fast_reference(50.0, WIDTH))

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

    def test_touched_row_clear_matches_full_clear_over_mixed_frames(self):
        optimized = main._FastRowWeightWorkspace()
        reference = _FullClearReferenceWorkspace()
        rows_by_frame = (
            [(y, 22 + (y % 4), 54 + (y % 4))
             for y in range(FLOOR, HEIGHT)],
            [(8, 18, 42), (10, 18, 42), (20, 28, 52),
             (24, 28, 52), (40, 30, 54), (58, 32, 56)],
            [(20, 22, 54), (20, 24, 56), (39, 24, 56),
             (40, 24, 56), (41, 24, 56), (55, 25, 57)],
            [(10, 20, 50), (40, 24, 54)],
        )
        support_by_frame = (
            None,
            [1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1],
            [0, 0],
        )

        for frame_index, (rows, support) in enumerate(
                zip(rows_by_frame, support_by_frame)):
            candidate = _path_candidate(rows)
            candidate["boundary_support_rows"] = bytearray(
                support if support is not None else [1] * len(rows))
            candidate["boundary_support_count"] = len(rows)
            selection = {"valid": True, "source": "road",
                         "result": candidate}
            actual = dict(optimized.aggregate(selection, candidate, config))
            expected = dict(reference.aggregate(selection, candidate, config))
            self.assertEqual(actual, expected, "frame %d" % frame_index)

            if support is None:
                expected_rows = {y for y, _left, _right in rows
                                 if y >= optimized._PREVIEW_FLOOR_Y}
            else:
                expected_rows = {
                    y for index, (y, _left, _right) in enumerate(rows)
                    if support[index] and y >= optimized._PREVIEW_FLOOR_Y
                }
            actual_rows = {
                y for y, value in enumerate(optimized._preview_valid_rows)
                if value
            }
            self.assertEqual(actual_rows, expected_rows,
                             "stale preview rows after frame %d" % frame_index)

        invalid = {"valid": False, "source": "NONE", "result": None}
        actual = dict(optimized.aggregate(invalid, {}, config))
        expected = dict(reference.aggregate(invalid, {}, config))
        self.assertEqual(actual, expected)
        self.assertFalse(any(optimized._preview_valid_rows))


if __name__ == "__main__":
    unittest.main(verbosity=2)
