"""Host semantic comparison for the bounded V7 road-boundary kernel."""

import copy
import sys
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch


TEST_ROOT = Path(__file__).resolve().parents[1]
OPENMV_DIR = TEST_ROOT / "openmv"
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import boundary_evidence
import config
import fast_boundary


WIDTH = 80
HEIGHT = 60
PIXELS = WIDTH * HEIGHT


class _ArrayWithoutTypecode(array):
    @property
    def typecode(self):
        raise AttributeError("board arrays do not expose typecode")


class _MetadataFreeMemoryview:
    """OpenMV v5-style view: buffer bytes but no itemsize/format fields."""

    __slots__ = ("_view",)

    def __init__(self, value):
        self._view = memoryview(value)

    def __bytes__(self):
        return bytes(self._view)


def frame_masks(edge_gray=25, inside_gray=220):
    luma = bytearray([inside_gray]) * PIXELS
    edge_mask = bytearray(PIXELS)
    road_mask = bytearray([255]) * PIXELS
    known = bytearray([1]) * PIXELS
    for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
        for x in (20, 60):
            index = y * WIDTH + x
            luma[index] = edge_gray
            edge_mask[index] = 1
            road_mask[index] = 0
    return luma, edge_mask, road_mask, known


def road_candidate(y_values=(55, 53, 51, 49, 47, 45, 43, 41, 39, 37)):
    rows = []
    for y in y_values:
        rows.append({
            "y": y, "left": 21, "right": 59, "center": 40.0,
            "width": 39, "left_status": "OBSERVED",
            "right_status": "OBSERVED", "provenance": "OBSERVED",
        })
    return {
        "valid": True, "quality": 90, "reason": "OK",
        "lateral_error": 123, "center_x": 40.0,
        "reference_y": 40.0, "reference_center_x": 40.0,
        "reference_lateral_error": 13, "selected_rows": rows,
    }


def with_fast_path_rows(candidate, flags=7):
    candidate = copy.deepcopy(candidate)
    rows = candidate["selected_rows"]
    packed = array("i", [0] * (HEIGHT * 4))
    for index, row in enumerate(rows):
        base = index * 4
        packed[base] = int(row["y"])
        packed[base + 1] = int(row["left"])
        packed[base + 2] = int(row["right"])
        packed[base + 3] = flags
    candidate["selected_row_ints"] = packed
    candidate["selected_row_count"] = len(rows)
    candidate["selected_row_stride"] = 4
    return candidate


def parity_candidate(candidate):
    old = copy.deepcopy(candidate)
    new = copy.deepcopy(candidate)
    return old, new


def run_gate(candidate, masks, fast):
    luma, edge, road, known = masks
    boundary_evidence.gate_road_candidate(
        candidate, luma, edge, road, known, config)
    fast.gate_road_candidate(candidate, luma, edge, road, known, config)


class FastBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.fast = fast_boundary.FastBoundaryEvidence(
            config, diagnostics=True)

    def _assert_gate_parity(self, old, new):
        self.assertEqual(new["valid"], old["valid"])
        for key in ("quality", "reason", "geometry_reason", "lateral_error",
                    "center_x", "reference_center_x",
                    "reference_lateral_error"):
            self.assertEqual(new.get(key), old.get(key), key)
        old_evidence = old.get("boundary_evidence")
        new_evidence = new.get("boundary_evidence")
        self.assertIsNotNone(new_evidence)
        for key in ("source", "status", "reason", "pairs_tested",
                    "bilateral_supported", "left_supported",
                    "right_supported", "support_ratio",
                    "target_support_count", "minimum_step",
                    "minimum_step_fraction", "left_step_median",
                    "right_step_median", "left_step_fraction_median",
                    "right_step_fraction_median"):
            left = old_evidence.get(key)
            right = new_evidence.get(key)
            if isinstance(left, float):
                self.assertAlmostEqual(left, right, places=12, msg=key)
            else:
                self.assertEqual(right, left, key)
        self.assertEqual(list(new_evidence["support_points"]),
                         old_evidence["support_points"])

    def test_default_backend_is_host_shim_and_buffers_are_preallocated(self):
        fast = fast_boundary.FastBoundaryEvidence(config)
        self.assertEqual(fast.RUNTIME_API_VERSION if hasattr(
            fast, "RUNTIME_API_VERSION") else
            fast_boundary.RUNTIME_API_VERSION, 7)
        self.assertEqual(fast_boundary.COMPILER_BACKEND, "cpython-shim")
        self.assertEqual(len(fast._records), 60 * 11)
        self.assertEqual(len(fast._row_output), 60 * 9)
        self.assertEqual(len(fast._support_x), 60)
        self.assertEqual(len(fast._support_y), 60)
        with self.assertRaisesRegex(ValueError, "diagnostics must be bool"):
            fast_boundary.FastBoundaryEvidence(config, diagnostics=1)

    def test_road_gate_matches_old_steps_ratio_support_and_diagnostics(self):
        masks = frame_masks(25, 220)
        luma = masks[0]
        for index, y in enumerate((55, 53, 51, 49, 47, 45, 43, 41, 39, 37)):
            # Exercise median-of-three and median-of-two side samples.
            luma[y * WIDTH + 21] = 210 + (index % 3) * 5
            luma[y * WIDTH + 22] = 220
            luma[y * WIDTH + 23] = 230 - (index % 3) * 5
            luma[y * WIDTH + 59] = 230 - (index % 3) * 5
            luma[y * WIDTH + 58] = 220
            luma[y * WIDTH + 57] = 210 + (index % 3) * 5
        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, *masks[:1], masks[1], masks[2], masks[3], config)
        self.fast.gate_road_candidate(
            new, masks[0], masks[1], masks[2], masks[3], config)
        self._assert_gate_parity(old, new)
        self.assertTrue(new["valid"])
        self.assertEqual(new["boundary_evidence"]["bilateral_supported"], 10)

    def test_low_contrast_and_unknown_pixels_fail_closed_like_old_gate(self):
        cases = []
        cases.append(("low_step", frame_masks(100, 100)))
        masks = frame_masks(25, 220)
        for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
            masks[3][y * WIDTH + 21] = 0
            masks[3][y * WIDTH + 22] = 0
        cases.append(("unknown_inward", masks))
        for name, masks in cases:
            with self.subTest(name=name):
                old, new = parity_candidate(road_candidate())
                boundary_evidence.gate_road_candidate(
                    old, masks[0], masks[1], masks[2], masks[3], config)
                self.fast.gate_road_candidate(
                    new, masks[0], masks[1], masks[2], masks[3], config)
                self._assert_gate_parity(old, new)
                self.assertFalse(new["valid"])
                self.assertEqual(new["lateral_error"], 0)
                self.assertIsNone(new["reference_center_x"])

    def test_negative_step_diagnostics_remain_signed_like_old_gate(self):
        masks = frame_masks(edge_gray=200, inside_gray=80)
        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], config)
        new = with_fast_path_rows(new)
        self.fast.gate_fast_path_candidate(
            new, masks[0], masks[1], masks[2], masks[3], config)
        self._assert_gate_parity(old, new)
        evidence = new["boundary_evidence"]
        self.assertEqual(evidence["left_step_median"], -120.0)
        self.assertEqual(evidence["right_step_median"], -120.0)
        self.assertEqual(evidence["bilateral_supported"], 0)
        self.assertEqual(evidence["reason"], "LOW_PAIR_SUPPORT")

    def test_fraction_cutoff_is_signed_exact_and_reread_each_frame(self):
        masks = frame_masks(edge_gray=150, inside_gray=200)
        settings = dict(config.__dict__)
        settings["GRAY_BOUNDARY_MIN_STEP_FRACTION"] = 0.25

        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], settings)
        new = with_fast_path_rows(new)
        self.fast.gate_fast_path_candidate(
            new, masks[0], masks[1], masks[2], masks[3], settings)
        self._assert_gate_parity(old, new)
        evidence = new.get("boundary_evidence")
        self.assertIsNotNone(evidence)
        self.assertIs(evidence, self.fast.evidence)
        self.assertTrue(new["valid"])
        self.assertEqual(evidence["minimum_step_fraction"], 0.25)
        self.assertEqual(evidence["left_step_fraction_median"], 0.25)
        self.assertEqual(evidence["right_step_fraction_median"], 0.25)

        # The same validator instance must reread the dynamic limit. A value
        # just above the observed ratio rejects the exact same signed step.
        settings["GRAY_BOUNDARY_MIN_STEP_FRACTION"] = 0.2500001
        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], settings)
        new = with_fast_path_rows(new)
        self.fast.gate_fast_path_candidate(
            new, masks[0], masks[1], masks[2], masks[3], settings)
        self._assert_gate_parity(old, new)
        evidence = new.get("boundary_evidence")
        self.assertIsNotNone(evidence)
        self.assertIs(evidence, self.fast.evidence)
        self.assertFalse(new["valid"])
        self.assertEqual(evidence["reason"], "LOW_PAIR_SUPPORT")
        self.assertEqual(evidence["minimum_step_fraction"], 0.2500001)
        self.assertEqual(evidence["left_step_fraction_median"], 0.25)

    def test_exact_seventy_five_percent_pair_gate_and_small_row_rejection(self):
        masks = frame_masks()
        candidate = road_candidate((55, 53, 51, 49))
        candidate["reference_y"] = 52.0
        for y in (49,):
            masks[3][y * WIDTH + 21] = 0
            masks[3][y * WIDTH + 22] = 0
        old, new = parity_candidate(candidate)
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], config)
        self.fast.gate_road_candidate(
            new, masks[0], masks[1], masks[2], masks[3], config)
        self._assert_gate_parity(old, new)
        self.assertTrue(new["valid"])
        self.assertEqual(new["boundary_evidence"]["bilateral_supported"], 3)
        self.assertEqual(new["boundary_evidence"]["support_ratio"], 0.75)

        few_old, few_new = parity_candidate(road_candidate((53, 51, 49)))
        boundary_evidence.gate_road_candidate(
            few_old, *masks, config)
        self.fast.gate_road_candidate(few_new, *masks, config)
        self._assert_gate_parity(few_old, few_new)
        self.assertEqual(few_new["boundary_evidence"]["reason"],
                         "INSUFFICIENT_FRESH_SUPPORT")

    def test_edge_crop_missing_masks_and_row_bounds_are_rejected(self):
        masks = frame_masks()
        cropped = road_candidate((55, 53, 51, 49))
        for row in cropped["selected_rows"]:
            row["left"] = 0
        self.fast.gate_road_candidate(cropped, *masks, config)
        self.assertFalse(cropped["valid"])
        self.assertEqual(cropped["boundary_evidence"]["left_supported"], 0)

        missing = road_candidate()
        self.fast.gate_road_candidate(missing, None, None, None, None, config)
        self.assertFalse(missing["valid"])
        self.assertEqual(missing["boundary_evidence"]["reason"],
                         "MISSING_FRAME_MASK")

        short = road_candidate()
        self.fast.gate_road_candidate(
            short, bytearray(PIXELS - 1), masks[1], masks[2], masks[3], config)
        self.assertEqual(short["boundary_evidence"]["reason"],
                         "FRAME_SIZE_MISMATCH")

        too_many = road_candidate()
        too_many["selected_rows"] = too_many["selected_rows"] * 7
        self.fast.gate_road_candidate(too_many, *masks, config)
        self.assertEqual(too_many["boundary_evidence"]["reason"],
                         "FRAME_SIZE_MISMATCH")

    def test_selected_reference_matches_old_gate_without_new_support(self):
        masks = frame_masks()
        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], config)
        self.fast.gate_road_candidate(
            new, masks[0], masks[1], masks[2], masks[3], config)
        old_support = list(old["boundary_evidence"]["support_points"])
        new_support = list(new["boundary_evidence"]["support_points"])
        self.assertEqual(old_support, new_support)

        selection_old = {"valid": True, "source": "road"}
        selection_new = {"valid": True, "source": "road"}
        near = {"x": 40.0, "y": 42.0, "kind": "shared_y"}
        old_check = boundary_evidence.validate_selected_reference(
            selection_old, near, None, old, config)
        new_check = self.fast.validate_selected_reference(
            selection_new, near, None, new, config)
        self.assertEqual(new_check, old_check)
        self.assertTrue(new_check["valid"])
        self.assertEqual(list(new["boundary_evidence"]["support_points"]),
                         new_support)

        far = {"x": 40.0, "y": 30.0, "kind": "shared_y"}
        old, new = parity_candidate(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], config)
        self.fast.gate_road_candidate(
            new, masks[0], masks[1], masks[2], masks[3], config)
        old_check = boundary_evidence.validate_selected_reference(
            {"valid": True, "source": "road"}, far, None, old, config)
        new_check = self.fast.validate_selected_reference(
            {"valid": True, "source": "road"}, far, None, new, config)
        self.assertEqual(new_check, old_check)
        self.assertEqual(new["reason"], "BOUNDARY_REFERENCE_UNSUPPORTED")
        self.assertEqual(new["boundary_evidence"]["reason"], "OK")
        self.assertEqual(new["boundary_evidence"]["selected_reference_reason"],
                         "NO_SELECTED_REFERENCE_SUPPORT")

    def test_weighted_reference_and_unknown_next_frame_clear_support_view(self):
        masks = frame_masks()
        candidate = road_candidate()
        candidate["reference_y"] = None
        candidate["reference_center_x"] = None
        candidate["weights"] = [0.1] * len(candidate["selected_rows"])
        self.fast.gate_road_candidate(candidate, *masks, config)
        self.assertTrue(candidate["valid"], candidate["boundary_evidence"])
        self.assertEqual(candidate["boundary_evidence"]["target_support_count"],
                         6)
        previous_view = candidate["boundary_evidence"]["support_points"]
        self.assertEqual(len(previous_view), 10)

        unknown = list(frame_masks())
        unknown[3] = bytearray(PIXELS)
        next_candidate = road_candidate()
        self.fast.gate_road_candidate(next_candidate, *unknown, config)
        self.assertFalse(next_candidate["valid"])
        self.assertEqual(len(previous_view), 0)
        stale = self.fast.validate_selected_reference(
            {"valid": True, "source": "road"},
            {"x": 40.0, "y": 46.0, "kind": "weighted_center"},
            None, candidate, config)
        self.assertFalse(stale["valid"])
        self.assertEqual(stale["reason"], "NO_PRESELECTION_EVIDENCE")

    def test_packed_fast_path_rows_match_legacy_and_reuse_same_kernel(self):
        masks = frame_masks()
        old, general = parity_candidate(road_candidate())
        fast = with_fast_path_rows(road_candidate())
        boundary_evidence.gate_road_candidate(
            old, masks[0], masks[1], masks[2], masks[3], config)
        self.fast.gate_road_candidate(
            general, masks[0], masks[1], masks[2], masks[3], config)
        self.fast.gate_fast_path_candidate(
            fast, masks[0], masks[1], masks[2], masks[3], config)
        self._assert_gate_parity(old, general)
        self._assert_gate_parity(old, fast)
        self.assertTrue(fast["valid"])
        self.assertEqual(fast["boundary_evidence"]["bilateral_supported"],
                         10)
        self.assertEqual(fast["boundary_evidence"]["support_points"],
                         general["boundary_evidence"]["support_points"])

        # The legacy entry point dispatches to the packed array whenever it is
        # present, keeping integration callers from accidentally using Python
        # dictionary staging on the production path.
        dispatched = with_fast_path_rows(road_candidate())
        self.fast.gate_road_candidate(
            dispatched, masks[0], masks[1], masks[2], masks[3], config)
        self._assert_gate_parity(old, dispatched)

    def test_packed_rows_validate_count_stride_buffer_and_flags(self):
        masks = frame_masks()
        invalid_cases = (
            ("bool count", lambda c: c.update(selected_row_count=True)),
            ("large count", lambda c: c.update(selected_row_count=61)),
            ("stride", lambda c: c.update(selected_row_stride=3)),
            ("short array", lambda c: c.update(
                selected_row_ints=array("i", [0] * 4))),
            ("wrong array", lambda c: c.update(
                selected_row_ints=bytearray(60 * 4))),
        )
        for name, corrupt in invalid_cases:
            with self.subTest(name=name):
                candidate = with_fast_path_rows(road_candidate())
                corrupt(candidate)
                self.fast.gate_fast_path_candidate(
                    candidate, masks[0], masks[1], masks[2], masks[3], config)
                self.assertFalse(candidate["valid"])
                self.assertEqual(candidate["boundary_evidence"]["reason"],
                                 "INVALID_SELECTED_ROWS")

        # A structurally valid but unobserved row contributes to the same
        # denominator as the original selected_rows API, while its zeroed
        # per-row record cannot leak prior-frame white/dark evidence.
        candidate = with_fast_path_rows(road_candidate(), flags=0)
        self.fast.gate_fast_path_candidate(
            candidate, masks[0], masks[1], masks[2], masks[3], config)
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["boundary_evidence"]["pairs_tested"], 10)
        self.assertEqual(candidate["boundary_evidence"]["bilateral_supported"],
                         0)

    def test_array_validation_uses_buffer_width_not_public_typecode(self):
        masks = frame_masks()
        candidate = with_fast_path_rows(road_candidate())
        regular = candidate["selected_row_ints"]
        hidden_typecode = _ArrayWithoutTypecode("i", regular)
        candidate["selected_row_ints"] = hidden_typecode
        calls = []
        def metadata_free(value):
            calls.append(value)
            return _MetadataFreeMemoryview(value)

        with patch.object(fast_boundary, "memoryview", metadata_free,
                          create=True):
            self.fast.gate_fast_path_candidate(
                candidate, masks[0], masks[1], masks[2], masks[3], config)
            self.fast.gate_fast_path_candidate(
                candidate, masks[0], masks[1], masks[2], masks[3], config)
        self.assertTrue(candidate["valid"])
        self.assertEqual(len(calls), 1)
        self.assertIs(self.fast._path_rows_owner, hidden_typecode)

        unsigned = with_fast_path_rows(road_candidate())
        unsigned["selected_row_ints"] = array(
            "I", unsigned["selected_row_ints"])
        self.fast.gate_fast_path_candidate(
            unsigned, masks[0], masks[1], masks[2], masks[3], config)
        self.assertFalse(unsigned["valid"])
        self.assertEqual(unsigned["boundary_evidence"]["reason"],
                         "INVALID_SELECTED_ROWS")

        narrow = with_fast_path_rows(road_candidate())
        narrow["selected_row_ints"] = array(
            "h", narrow["selected_row_ints"])
        self.fast.gate_fast_path_candidate(
            narrow, masks[0], masks[1], masks[2], masks[3], config)
        self.assertFalse(narrow["valid"])

        floating = with_fast_path_rows(road_candidate())
        floating["selected_row_ints"] = array(
            "f", floating["selected_row_ints"])
        self.fast.gate_fast_path_candidate(
            floating, masks[0], masks[1], masks[2], masks[3], config)
        self.assertFalse(floating["valid"])

    def test_target_quantization_is_bounded_and_radius_never_expands(self):
        masks = frame_masks()
        too_large_radius = dict(config.__dict__)
        too_large_radius["GRAY_BOUNDARY_TARGET_RADIUS_CELLS"] = 80.01
        with self.assertRaisesRegex(ValueError, "invalid fast boundary"):
            self.fast.gate_fast_path_candidate(
                with_fast_path_rows(road_candidate()), *masks,
                too_large_radius)

        too_large_radius["GRAY_BOUNDARY_TARGET_RADIUS_CELLS"] = 5.3
        candidate = with_fast_path_rows(road_candidate())
        self.fast.gate_fast_path_candidate(
            candidate, *masks, too_large_radius)
        # The half-cell quantizer floors configured radius, while preserving
        # the production default radius 5 exactly.
        self.assertEqual(self.fast._target_radius_q, 10)
        check = self.fast.validate_selected_reference(
            {"valid": True, "source": "road"},
            {"x": 1e300, "y": 1e300, "kind": "outside"}, None,
            candidate, too_large_radius)
        self.assertFalse(check["valid"])
        self.assertEqual(check["support_count"], 0)

        bad_support = dict(config.__dict__)
        bad_support["GRAY_BOUNDARY_MIN_ROAD_TARGET_SUPPORT"] = True
        with self.assertRaisesRegex(ValueError, "invalid fast boundary"):
            fast_boundary.FastBoundaryEvidence(bad_support).gate_fast_path_candidate(
                with_fast_path_rows(road_candidate()), *masks, bad_support)


if __name__ == "__main__":
    unittest.main(verbosity=2)
