"""Integration checks for the visual gate, route admission, and gyro evidence."""
from pathlib import Path
import sys
from array import array


ROOT = Path(__file__).resolve().parents[1]
OPENMV = ROOT / "openmv"
sys.path.insert(0, str(OPENMV))
sys.path.insert(0, str(OPENMV / "tests"))

import gyro_link
import reverse_control
import wireless_sideband
from visual_entry_gate import (MARK_FIRST, MARK_LOWER_OPENING,
                               MARK_UPPER_ENTRANCE,
                               VisualEntryCalibration,
                               VisualEntryCalibrationCollector,
                               VisualEntryGate)
from test_track_elements import TrackElementsHarness


def _calibration():
    return VisualEntryCalibration(True, 1, {1: {
        "far_y_min": 20, "far_y_max": 30, "near_y_min": 70,
        "near_y_max": 85, "required_frames": 2}}, "READY")


def _ring(evidence=False):
    return {"state": "INSIDE", "state_id": 3, "ring_confirmed": True,
            "side": 1, "entry_transition": True,
            "entry_transition_far_y": 25,
            "entry_transition_near_y": 75,
            "candidate": True, "entry_evidence": evidence}


def _ticks(new, old):
    return new - old


def test_main_requires_visual_gate_before_ring_route_and_block():
    source = (OPENMV / "main.py").read_text(encoding="utf-8")
    route_condition = source[source.index("if (ring_profile_enabled"):source.index(
        "ring_count = _fill_ring_control_route", source.index(
            "if (ring_profile_enabled"))]
    assert "element_result.get(\"entry_ready\")" in route_condition
    block_condition = source[source.index("ring_target_blocked = bool"):source.index(
        "if ring_target_blocked", source.index("ring_target_blocked = bool"))]
    assert "element_result.get(\"entry_ready\")" in block_condition
    source = (OPENMV / "main.py").read_text(encoding="utf-8")
    assert 'prior_ring.get("entry_ready")' in source
    assert "VISUAL_ENTRY_RESULT_VISUAL_READY" in source
    assert "VISUAL_ENTRY_RESULT_ENCODER_READY" in source


def test_parked_straight_baseline_cannot_be_used_as_first_ring_marker():
    harness = TrackElementsHarness()
    rows = array("i")
    for y in range(8, 60):
        rows.extend((y, 25, 55, 7))
    result = None
    for frame in range(3):
        result = harness.tracker.observe_parked(
            (frame + 1) * 30, 2, 77, True, True, rows, 52, 4, 2,
            {"reason": "OK", "complete": True}, False,
            bytearray(120), array("i", [0] * 120), array("i", [0] * 120),
            harness.cache_valid, harness.cache_width, harness.cache_left2,
            harness.cache_right2, harness.mask(), frame + 1, 100)
    assert result["baseline_ready"] is True
    assert result["ring_first_stable"] is False
    collector = VisualEntryCalibrationCollector()
    assert not collector.mark(1, MARK_FIRST, result,
                              metadata={"profile_id": 2,
                                        "session_token": 77},
                              millirev=100)
    assert collector.reason == "FIRST_STABLE_REQUIRED"


def test_parked_ring_marker_sequence_exports_visual_and_encoder_records():
    collector = VisualEntryCalibrationCollector()
    origin = {"profile_id": 2, "session_token": 77}
    first = {"ring_first_stable": True,
             "ring_stable_observation": True,
             "parked_ring_confirmed": True,
             "ring_stable_far_y": 10,
             "ring_stable_near_y": 60}
    lower = {"entry_transition_far_y": 40,
             "entry_transition_near_y": 90}
    upper = {"entry_transition_far_y": 25,
             "entry_transition_near_y": 75}
    assert collector.mark(1, MARK_FIRST, first, metadata=origin,
                          millirev=100)
    assert collector.mark(1, MARK_LOWER_OPENING, lower, metadata=origin,
                          millirev=220)
    assert collector.mark(1, MARK_UPPER_ENTRANCE, upper, metadata=origin,
                          millirev=430)
    candidate = collector.build_calibration(require_encoder=True, side=1)
    assert candidate is not None and candidate.ready
    assert candidate.encoder_entry["ready"] is True
    assert candidate.encoder_entry["right"]["first_millirev"] == 100
    assert candidate.encoder_entry["right"]["upper_millirev"] == 430
    assert candidate.encoder_entry["right"]["threshold_millirev"] == 330


def test_commit_detail_bits_are_independent():
    assert wireless_sideband.VISUAL_ENTRY_RESULT_VISUAL_READY == 1
    assert wireless_sideband.VISUAL_ENTRY_RESULT_ENCODER_READY == 2
    assert wireless_sideband.VISUAL_ENTRY_RESULT_MASK == 3


def test_gate_and_gyro_require_fresh_post_acceptance_evidence():
    parser = reverse_control.ReverseControlParser(250, 250)
    parser.feed(reverse_control.pack_frame(0, 100), 8, 0)
    parser.feed(reverse_control.pack_frame(1, 101), 8, 1)
    parser.feed(reverse_control.pack_profile_state(
        0, 2, reverse_control.PROFILE_SESSION_ACTIVE |
        reverse_control.PROFILE_GYRO_SUPPORTED, session_token=1), 20, 2)
    gate = VisualEntryGate(_calibration())
    link = gyro_link.GyroLink()
    first = gate.update(_ring(), profile_id=2, session_token=1)
    assert not first["ready"]
    second = gate.update(_ring(), profile_id=2, session_token=1)
    assert second["ready"] and not second["entry_evidence"]
    info = _ring(second["entry_evidence"])
    info["entry_ready"] = second["ready"]
    assert link.update(parser, info, True, 3, _ticks)["phase"] == 0
    third = gate.update(_ring(), profile_id=2, session_token=1)
    assert third["entry_evidence"]
    info = _ring(True)
    info["entry_ready"] = third["ready"]
    assert link.update(parser, info, True, 4, _ticks)["phase"] == 1


if __name__ == "__main__":
    test_main_requires_visual_gate_before_ring_route_and_block()
    test_parked_straight_baseline_cannot_be_used_as_first_ring_marker()
    test_parked_ring_marker_sequence_exports_visual_and_encoder_records()
    test_commit_detail_bits_are_independent()
    test_gate_and_gyro_require_fresh_post_acceptance_evidence()
    print("visual entry integration: 5 tests passed")
