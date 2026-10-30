"""Regression guard for constant-width ordinary curves with a side port.

The fixture is synthetic host data from ``test_ring_slope_host._ring_fixture``.
It models parallel curved boundaries 24 cells apart; it is not a camera or
vehicle recording. On the pre-fix runtime, both mirrored cases falsely enter
the ring route after three fresh KEEP frames. This is retained as the negative
control for the fix.
"""

import sys
import types
import unittest
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
TEST_DIR = OPENMV_DIR / "tests"
for _path in (OPENMV_DIR, TEST_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import fast_ring_slope
from test_ring_slope_host import _ordinary_band_fixture, _ring_fixture


class OrdinaryCurveRingGuardTests(unittest.TestCase):
    def test_parallel_curves_do_not_enter_ring_on_three_fresh_keep_frames(self):
        for side in (fast_ring_slope.RING_LEFT,
                     fast_ring_slope.RING_RIGHT):
            with self.subTest(side=side):
                masks, road_result, support = _ring_fixture(side, port=True)
                tracker = fast_ring_slope.FastRingSlope(config)
                candidate_history = []
                state_history = []

                for frame_number in (1, 2, 3):
                    candidate = tracker.detect_ring_candidate(
                        masks, road_result, support, len(support))
                    candidate_history.append({
                        "valid": candidate["valid"],
                        "reason": candidate["reason"],
                        "side": candidate["side"],
                        "port_x": candidate["port_x"],
                        "port_y": candidate["port_y"],
                        "arc_rows": candidate["arc_rows"],
                        "intrusion": candidate["intrusion"],
                        "outer_fit_rows": candidate["outer_fit_rows"],
                        "outer_fit_residual_max": candidate[
                            "outer_fit_residual_max"],
                        "rejection_reason": candidate["rejection_reason"],
                    })
                    state = tracker.update_ring_state(
                        fresh_keep=True,
                        generation=1,
                        normal_route_valid=True,
                        forward_continuation=True,
                        now_ms=frame_number * 100,
                        ticks_diff=lambda new, old: new - old,
                    )
                    state_history.append({
                        "state": state["state"],
                        "active": state["active"],
                        "pending_count": state["pending_count"],
                        "reason": state["reason"],
                    })

                self.assertFalse(
                    state_history[-1]["active"],
                    "constant-width ordinary curve entered ring route: "
                    "side=%d candidates=%r states=%r" %
                    (side, candidate_history, state_history),
                )
                self.assertEqual(
                    [item["rejection_reason"] for item in candidate_history],
                    ["PARALLEL_CURVE"] * 3,
                )
                self.assertGreaterEqual(
                    candidate_history[-1]["outer_fit_rows"], 10)
                self.assertGreater(
                    candidate_history[-1]["outer_fit_residual_max"], 1.5)
                self.assertEqual(state_history[-1]["pending_count"], 0)

    def test_straight_bend_s_curve_reverse_s_and_u_profiles_stay_ordinary(self):
        profiles = ("straight", "bend", "sharp_bend", "s_curve",
                    "reverse_s", "u_curve")
        for side in (fast_ring_slope.RING_LEFT,
                     fast_ring_slope.RING_RIGHT):
            for profile in profiles:
                for crop_side_edge in (None,
                                       1 if side == fast_ring_slope.RING_LEFT
                                       else 78):
                    with self.subTest(side=side, profile=profile,
                                      crop_side_edge=crop_side_edge):
                        masks, road_result, support = _ordinary_band_fixture(
                            side, profile, port=True,
                            crop_side_edge=crop_side_edge)
                        tracker = fast_ring_slope.FastRingSlope(config)
                        history = []
                        for frame_number in (1, 2, 3):
                            candidate = tracker.detect_ring_candidate(
                                masks, road_result, support, len(support))
                            state = tracker.update_ring_state(
                                fresh_keep=True,
                                generation=1,
                                normal_route_valid=True,
                                forward_continuation=True,
                                now_ms=frame_number * 100,
                                ticks_diff=lambda new, old: new - old,
                            )
                            history.append((candidate, state))
                        self.assertFalse(
                            history[-1][1]["active"],
                            "ordinary profile latched as ring: side=%d "
                            "profile=%s crop=%s candidate=%r state=%r" % (
                                side, profile, crop_side_edge,
                                history[-1][0], history[-1][1]))
                        self.assertEqual(history[-1][1]["pending_count"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
