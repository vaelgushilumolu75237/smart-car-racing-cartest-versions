"""Full-result differential tests for fast-boundary hot-loop changes."""

import copy
import importlib.util
import sys
import unittest
from array import array
from pathlib import Path


OPENMV = Path(__file__).resolve().parents[1]
if str(OPENMV) not in sys.path:
    sys.path.insert(0, str(OPENMV))

import config
import fast_boundary


BASELINE_PATH = (Path(__file__).resolve().parent /
                 "boundary_hotspot_reference_20261023" /
                 "fast_boundary.py")
BASELINE_SPEC = importlib.util.spec_from_file_location(
    "fast_boundary_pre_hotspot_20261023", BASELINE_PATH)
BASELINE = importlib.util.module_from_spec(BASELINE_SPEC)
BASELINE_SPEC.loader.exec_module(BASELINE)

WIDTH = 80
HEIGHT = 60
PIXELS = WIDTH * HEIGHT
ROWS = (55, 53, 51, 49, 47, 45, 43, 41, 39, 37)


def _frame_masks(edge_gray=25, inside_gray=220, unknown=()):
    luma = bytearray([inside_gray]) * PIXELS
    edge = bytearray(PIXELS)
    road = bytearray([255]) * PIXELS
    known = bytearray([1]) * PIXELS
    for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
        for x in (20, 60):
            index = y * WIDTH + x
            luma[index] = edge_gray
            edge[index] = 1
            road[index] = 0
    for y, x in unknown:
        known[y * WIDTH + x] = 0
    return luma, edge, road, known


def _candidate(packed=True, flags=7, y_values=ROWS,
               left=21, right=59):
    selected_rows = [{
        "y": y, "left": left, "right": right,
        "center": (left + right) * 0.5,
        "width": right - left + 1, "left_status": "OBSERVED",
        "right_status": "OBSERVED", "provenance": "OBSERVED",
    } for y in y_values]
    result = {
        "valid": True, "quality": 90, "reason": "OK",
        "lateral_error": 123, "center_x": 40.0,
        "reference_y": 40.0, "reference_center_x": 40.0,
        "reference_lateral_error": 13, "selected_rows": selected_rows,
    }
    if packed:
        rows = array("i", [0] * (HEIGHT * 4))
        for index, row in enumerate(selected_rows):
            base = index * 4
            rows[base] = row["y"]
            rows[base + 1] = row["left"]
            rows[base + 2] = row["right"]
            rows[base + 3] = flags
        result["selected_row_ints"] = rows
        result["selected_row_count"] = len(selected_rows)
        result["selected_row_stride"] = 4
    return result


def _snapshot(value):
    if isinstance(value, dict):
        return {key: _snapshot(item) for key, item in value.items()}
    if isinstance(value, array):
        return ("array", value.typecode, tuple(value))
    if isinstance(value, (bytearray, bytes)):
        return bytes(value)
    if isinstance(value, (tuple, list)):
        return tuple(_snapshot(item) for item in value)
    if isinstance(value, (str, int, float, bool, type(None))):
        return value
    if hasattr(value, "__iter__"):
        return tuple(_snapshot(item) for item in value)
    return value


class FastBoundaryHotspotEquivalenceTests(unittest.TestCase):
    def _compare_gate_sequence(self, diagnostics, use_fast_path):
        settings = config
        before = BASELINE.FastBoundaryEvidence(
            settings, diagnostics=diagnostics)
        after = fast_boundary.FastBoundaryEvidence(
            settings, diagnostics=diagnostics)
        cases = (
            ("valid", _candidate(packed=use_fast_path),
             _frame_masks(), settings),
            ("threshold_reload", _candidate(packed=use_fast_path),
             _frame_masks(), dict(
                 config.__dict__, GRAY_BOUNDARY_MIN_STEP=200.0)),
            ("all_unobserved", _candidate(
                packed=use_fast_path, flags=0), _frame_masks(), settings),
            ("unknown_samples", _candidate(packed=use_fast_path),
             _frame_masks(unknown=tuple((y, x) for y in ROWS
                                        for x in (21, 22, 58, 59))), settings),
            ("two_known_samples", _candidate(packed=use_fast_path),
             _frame_masks(unknown=tuple((y, x) for y in ROWS
                                        for x in (23, 57))), settings),
            ("left_under_sampled", _candidate(packed=use_fast_path),
             _frame_masks(unknown=tuple((y, x) for y in ROWS
                                        for x in (22, 23))), settings),
            ("left_hit_outside", _candidate(
                packed=use_fast_path, left=0, right=19),
             _frame_masks(), settings),
            ("low_contrast", _candidate(packed=use_fast_path),
             _frame_masks(edge_gray=100, inside_gray=100), settings),
        )

        for label, template, masks, source_settings in cases:
            with self.subTest(case=label, diagnostics=diagnostics,
                              packed=use_fast_path):
                old_candidate = copy.deepcopy(template)
                new_candidate = copy.deepcopy(template)
                if use_fast_path:
                    before.gate_fast_path_candidate(
                        old_candidate, *masks, source_settings)
                    after.gate_fast_path_candidate(
                        new_candidate, *masks, source_settings)
                else:
                    before.gate_road_candidate(
                        old_candidate, *masks, source_settings)
                    after.gate_road_candidate(
                        new_candidate, *masks, source_settings)
                self.assertEqual(_snapshot(new_candidate),
                                 _snapshot(old_candidate))

        # Exercise stale support-view behavior and selected-reference checks
        # across consecutive calls on the same reusable detector instances.
        old_candidate = copy.deepcopy(_candidate(packed=use_fast_path))
        new_candidate = copy.deepcopy(_candidate(packed=use_fast_path))
        masks = _frame_masks()
        gate = "gate_fast_path_candidate" if use_fast_path else \
            "gate_road_candidate"
        getattr(before, gate)(old_candidate, *masks, settings)
        getattr(after, gate)(new_candidate, *masks, settings)
        for instance, candidate in ((before, old_candidate),
                                    (after, new_candidate)):
            result = instance.validate_selected_reference(
                {"valid": True, "source": "road"},
                {"x": 40.0, "y": 42.0, "kind": "shared_y"},
                None, candidate, settings)
            if instance is before:
                old_check = result
            else:
                new_check = result
        self.assertEqual(_snapshot(new_check), _snapshot(old_check))
        self.assertEqual(_snapshot(new_candidate), _snapshot(old_candidate))

    def test_packed_path_matches_preserved_baseline_on_host(self):
        self._compare_gate_sequence(diagnostics=False, use_fast_path=True)
        self._compare_gate_sequence(diagnostics=True, use_fast_path=True)

    def test_dict_path_matches_preserved_baseline_on_host(self):
        self._compare_gate_sequence(diagnostics=False, use_fast_path=False)
        self._compare_gate_sequence(diagnostics=True, use_fast_path=False)

    def test_packed_viper_gate_matches_preserved_host_baseline(self):
        old_backend = fast_boundary._IS_MICROPYTHON
        old_ptr8 = fast_boundary.ptr8
        old_ptr32 = fast_boundary.ptr32
        try:
            fast_boundary._IS_MICROPYTHON = True
            fast_boundary.ptr8 = lambda value: value
            fast_boundary.ptr32 = lambda value: value
            self._compare_gate_sequence(
                diagnostics=False, use_fast_path=True)
            self._compare_gate_sequence(
                diagnostics=True, use_fast_path=True)
            self._compare_gate_sequence(
                diagnostics=False, use_fast_path=False)
            self._compare_gate_sequence(
                diagnostics=True, use_fast_path=False)
        finally:
            fast_boundary._IS_MICROPYTHON = old_backend
            fast_boundary.ptr8 = old_ptr8
            fast_boundary.ptr32 = old_ptr32


if __name__ == "__main__":
    unittest.main()
