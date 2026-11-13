"""Guard contour buffer setup against MicroPython's array multiplication gap."""

import array as array_module
import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

import config  # noqa: E402
import contour_route  # noqa: E402
import contour_trace  # noqa: E402
from contour_route import ContourRouteRecovery  # noqa: E402
from contour_trace import ContourPostWorkspace, ContourTraceWorkspace  # noqa: E402


class StrictArray(array_module.array):
    """Host array that models MicroPython by rejecting array repetition."""

    def __mul__(self, _count):
        raise TypeError("unsupported types for __mul__: array, int")

    def __rmul__(self, _count):
        raise TypeError("unsupported types for __mul__: int, array")


def _straight_mask():
    width = 80
    height = 60
    road = bytearray(width * height)
    known = bytearray([1]) * (width * height)
    for y in range(height - 1):
        for x in range(25, 55):
            road[y * width + x] = 255
    return {
        "width": width,
        "height": height,
        "floor": config.SURFACE_ROI_FLOOR_Y,
        "road_mask": road,
        "known_region": known,
        "edge_mask": bytearray([1]) * (width * height),
        "row_counts": bytearray([width]) * height,
    }


def _prior_result():
    return {
        "valid": False,
        "reason": "INSUFFICIENT_FRESH_ROWS",
        "scene": "ROAD_CANDIDATE",
        "topology_evidence": {
            "ambiguous": False,
            "branch_evidence": False,
            "island_evidence": False,
            "reason": "PARTIAL_KNOWN_ROI",
        },
    }


class ContourArrayCompatibilityTests(unittest.TestCase):
    def test_legacy_array_repetition_fixture_reproduces_type_error(self):
        with self.assertRaisesRegex(
                TypeError, "unsupported types for __mul__") as caught:
            StrictArray("I", [0]) * 128
        print("ARRAY_COMPAT_LEGACY_FIXTURE_REPRO: %s" % caught.exception)

    def test_strict_array_construction_and_synthetic_trace_and_route(self):
        with patch.object(contour_trace, "array", StrictArray):
            default_trace = ContourTraceWorkspace()
            self.assertEqual((80, 60),
                             (default_trace.width, default_trace.height))
            self.assertEqual(1800, len(default_trace.left_points))
            self.assertEqual(0, default_trace.left_points[0])
            self.assertEqual(0, default_trace._state_position[0])
            self.assertEqual(0, len(default_trace.event_points))

            event_trace = ContourTraceWorkspace(record_events=True)
            self.assertEqual(event_trace.iteration_budget,
                             len(event_trace.event_side))
            self.assertEqual(event_trace.iteration_budget * 2,
                             len(event_trace.event_points))
            self.assertEqual(0, event_trace.event_points[0])

            original_trace = ContourTraceWorkspace(
                width=320, height=240, max_points=300,
                iteration_budget=700, record_events=True)
            self.assertEqual((320, 240),
                             (original_trace.width, original_trace.height))
            self.assertEqual(600, len(original_trace.right_points))
            self.assertEqual(240, len(original_trace.left_row_support))
            self.assertEqual(1400, len(original_trace.event_points))

            post_default = ContourPostWorkspace()
            post_original = ContourPostWorkspace(
                width=320, height=240, max_samples=300)
            self.assertTrue(all(value == 0.0
                                for value in post_default.resampled_points))
            self.assertTrue(all(value == 0.0
                                for value in post_original.offset_points))
            self.assertEqual(600, len(post_original.filtered_points))

            route = ContourRouteRecovery(config)
            self.assertTrue(all(value == -1 for value in route._left_x))
            self.assertTrue(all(value == -1 for value in route._right_x))
            self.assertTrue(all(value == 0 for value in route._row_width))
            self.assertEqual(240, len(route.selected_row_ints))

            mask = _straight_mask()
            trace = default_trace.trace(mask)
            self.assertEqual(59, trace.left_count)
            self.assertEqual(59, trace.right_count)
            self.assertTrue(all(trace.left_dark_support[:trace.left_count]))
            self.assertTrue(all(trace.right_dark_support[:trace.right_count]))

            candidate = route.detect(mask, _prior_result())
            self.assertTrue(candidate["valid"], candidate["reason"])
            self.assertEqual("OK_CONTOUR_BILATERAL", candidate["reason"])
            self.assertGreaterEqual(candidate["selected_row_count"],
                                    route.min_rows)

    def test_all_runtime_modules_avoid_direct_array_multiplication(self):
        offenders = []
        for source in sorted(PACKAGE_DIR.glob("*.py")):
            tree = ast.parse(source.read_text(encoding="utf-8"),
                             filename=str(source))
            for node in ast.walk(tree):
                if not isinstance(node, ast.BinOp) or not isinstance(
                        node.op, ast.Mult):
                    continue
                operands = (node.left, node.right)
                if any(isinstance(operand, ast.Call) and
                       isinstance(operand.func, ast.Name) and
                       operand.func.id == "array"
                       for operand in operands):
                    offenders.append("%s:%d" %
                                     (source.name, node.lineno))
        self.assertEqual([], offenders,
                         "array object multiplication remains: %s" %
                         ", ".join(offenders))


if __name__ == "__main__":
    unittest.main()
