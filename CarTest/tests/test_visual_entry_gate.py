"""Host tests for the visual upper-entrance gate."""
import json
import pathlib
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1] / "openmv"
sys.path.insert(0, str(ROOT))

from visual_entry_gate import (MARK_FIRST, MARK_LOWER_OPENING,
                               MARK_UPPER_ENTRANCE, STATE_ACTIVE,
                               VisualEntryCalibration,
                               VisualEntryCalibrationCollector,
                               VisualEntryGate)
from entry_calibration_common import (UINT32_MAX,
                                      build_encoder_entry_record)


def _calibration():
    return VisualEntryCalibration(
        True, 1,
        {0: _record(required_frames=2), 1: _record(required_frames=2)},
        "READY")


def _record(**overrides):
    value = {"far_y_min": 20, "far_y_max": 30,
             "near_y_min": 70, "near_y_max": 85,
             "required_frames": 2, "marker_span": 20,
             "first_far_y": 10, "first_near_y": 60,
             "lower_far_y": 40, "lower_near_y": 90,
             "upper_far_y": 25, "upper_near_y": 75}
    value.update(overrides)
    return value


def _ring(**overrides):
    value = {"state_id": 3, "state": "INSIDE", "ring_confirmed": True,
             "side": 1, "entry_transition": True,
             "entry_transition_far_y": 25, "entry_transition_near_y": 75,
             "candidate": True}
    value.update(overrides)
    return value


def _stable(far_y=10, near_y=60, **overrides):
    value = {"ring_first_stable": True,
             "ring_stable_observation": True,
             "parked_ring_confirmed": True,
             "ring_stable_far_y": far_y,
             "ring_stable_near_y": near_y}
    value.update(overrides)
    return value


def test_missing_calibration_is_fail_closed():
    gate = VisualEntryGate(VisualEntryCalibration(), required=True)
    assert gate.update(_ring())["ready"] is False
    assert gate.update(_ring())["calibration_ready"] is False


def test_wrong_geometry_does_not_take_ring_route():
    gate = VisualEntryGate(_calibration())
    status = gate.update(_ring(entry_transition_far_y=45))
    assert status["ready"] is False
    assert status["reason"] == "ENTRY_GEOMETRY_WAIT"


def test_calibrated_geometry_needs_consecutive_frames():
    gate = VisualEntryGate(_calibration())
    assert gate.update(_ring())["ready"] is False
    status = gate.update(_ring())
    assert status["ready"] is True
    assert status["entry_evidence"] is False
    assert status["state"] == STATE_ACTIVE
    assert gate.update(_ring())["entry_evidence"] is True


def test_invalid_frame_revokes_active_admission_until_candidate_clears():
    gate = VisualEntryGate(_calibration())
    gate.update(_ring())
    assert gate.update(_ring())["ready"] is True
    invalid = gate.update(_ring(), valid=False)
    assert invalid["ready"] is False
    assert invalid["reason"] == "FRAME_INVALID"
    assert gate.update(_ring())["ready"] is False
    cleared = gate.update({"state_id": 0, "candidate": False,
                           "ring_confirmed": False})
    assert cleared["reason"] == "CANDIDATE_CLEARED"


def test_candidate_times_out_and_latches_until_tracker_clears_it():
    calibration = VisualEntryCalibration(True, 1, {1: _record(
        required_frames=1)}, "READY")
    gate = VisualEntryGate(calibration, candidate_timeout_ms=4000)
    waiting = gate.update({"state_id": 1, "candidate": True,
                           "ring_confirmed": False, "side": 1},
                          now_ms=100)
    assert waiting["ready"] is False
    expired = gate.update({"state_id": 1, "candidate": True,
                           "ring_confirmed": False, "side": 1},
                          now_ms=4100)
    assert expired["ready"] is False
    assert expired["reason"] == "ENTRY_GATE_EXPIRED"
    still_expired = gate.update(_ring(), now_ms=4200)
    assert still_expired["ready"] is False
    assert still_expired["reason"] == "ENTRY_GATE_EXPIRED"
    cleared = gate.update({"state_id": 0, "candidate": False,
                           "ring_confirmed": False}, now_ms=4300)
    assert cleared["reason"] == "CANDIDATE_CLEARED"
    accepted = gate.update(_ring(), now_ms=4400)
    assert accepted["ready"] is True


def test_old_marker_records_fail_closed_without_negative_marker_geometry():
    with tempfile.TemporaryDirectory() as directory:
        path = pathlib.Path(directory) / "entry.json"
        path.write_text(json.dumps({
            "format": 1, "schema": "VISUAL_ENTRY_V1", "generation": 1,
            "ready": True, "left": None,
            "right": {"profile_ids": [2, 3], "far_y_min": 20,
                      "far_y_max": 30, "near_y_min": 70,
                      "near_y_max": 85, "required_frames": 1,
                      "marker_span": 20, "crc32": 0}}))
        loaded = VisualEntryCalibration.load(str(path))
        assert loaded.ready is False
        assert loaded.reason.startswith("INVALID_right")


def test_new_observation_cannot_reuse_old_entry_lease():
    gate = VisualEntryGate(_calibration())
    gate.update(_ring())
    gate.update(_ring())
    status = gate.update({"state_id": 0, "ring_confirmed": False,
                          "candidate": False})
    assert status["ready"] is False


def test_save_and_readback_is_validated():
    with tempfile.TemporaryDirectory() as directory:
        path = pathlib.Path(directory) / "entry.json"
        saved = _calibration().save(str(path))
        assert saved.ready
        loaded = VisualEntryCalibration.load(str(path))
        assert loaded.ready
        assert loaded.sides[1]["near_y_max"] == 85
        parsed = json.loads(path.read_text())
        assert parsed["format"] == 1


def test_previous_copy_is_used_after_main_file_corruption():
    with tempfile.TemporaryDirectory() as directory:
        path = pathlib.Path(directory) / "entry.json"
        calibration = _calibration()
        calibration.save(str(path))
        calibration.generation = 2
        calibration.save(str(path))
        path.write_text("{broken")
        loaded = VisualEntryCalibration.load(str(path))
        assert loaded.ready
        assert loaded.reason == "READY_PREVIOUS"


def test_previous_old_copy_is_used_after_main_and_previous_corruption():
    with tempfile.TemporaryDirectory() as directory:
        path = pathlib.Path(directory) / "entry.json"
        calibration = _calibration()
        payload = json.dumps(calibration.as_dict())
        path.write_text("{broken")
        pathlib.Path(str(path) + ".previous").write_text("[broken]")
        pathlib.Path(str(path) + ".previous.old").write_text(payload)
        loaded = VisualEntryCalibration.load(str(path))
        assert loaded.ready
        assert loaded.reason == "READY_PREVIOUS"


def test_one_direction_can_be_ready_independently():
    only_right = VisualEntryCalibration(True, 1, {1: {
        "far_y_min": 20, "far_y_max": 30, "near_y_min": 70,
        "near_y_max": 85, "required_frames": 1,
        "marker_span": 20, "first_far_y": 10, "first_near_y": 60,
        "lower_far_y": 40, "lower_near_y": 90,
        "upper_far_y": 25, "upper_near_y": 75}}, "READY")
    assert VisualEntryGate(only_right).update(_ring(side=1))["ready"]
    assert not VisualEntryGate(only_right).update(_ring(side=0))["ready"]


def test_identity_and_side_changes_revoke_old_entry():
    calibration = VisualEntryCalibration(True, 1, {1: {
        "far_y_min": 20, "far_y_max": 30, "near_y_min": 70,
        "near_y_max": 85, "required_frames": 1,
        "marker_span": 20, "first_far_y": 10, "first_near_y": 60,
        "lower_far_y": 40, "lower_near_y": 90,
        "upper_far_y": 25, "upper_near_y": 75}}, "READY")
    gate = VisualEntryGate(calibration)
    assert gate.update(_ring(side=1), profile_id=3,
                       session_token=11)["ready"]
    assert not gate.update(_ring(side=1), profile_id=3,
                           session_token=12)["ready"]
    assert gate.update(_ring(side=1), profile_id=3,
                       session_token=12)["ready"]


def test_three_markers_reject_same_lower_opening():
    collector = VisualEntryCalibrationCollector()
    same = {"entry_transition_far_y": 25,
            "entry_transition_near_y": 75}
    assert collector.mark(1, MARK_FIRST, _stable(25, 75))
    assert collector.mark(1, MARK_LOWER_OPENING, same)
    assert collector.mark(1, MARK_UPPER_ENTRANCE, same)
    assert collector.build_side(1) is None
    assert collector.reason == "LOWER_AND_UPPER_GEOMETRY_NOT_DISTINCT"


def test_first_marker_requires_parked_stable_baseline_evidence():
    collector = VisualEntryCalibrationCollector()
    assert not collector.mark(1, MARK_FIRST, {
        "entry_transition_far_y": 10,
        "entry_transition_near_y": 60})
    assert collector.reason == "FIRST_STABLE_REQUIRED"


def test_encoder_marker_sentinel_is_rejected():
    try:
        build_encoder_entry_record([UINT32_MAX, 20, 40], side=1)
    except ValueError as exc:
        assert "sentinel" in str(exc)
    else:
        raise AssertionError("UINT32_MAX marker must be rejected")


def test_marker_origin_must_remain_one_parked_session():
    collector = VisualEntryCalibrationCollector()
    origin = {"profile_id": 2, "session_token": 11}
    assert collector.mark(1, MARK_FIRST, _stable(10, 60),
                          metadata=origin)
    assert not collector.mark(
        1, MARK_LOWER_OPENING,
        {"entry_transition_far_y": 35, "entry_transition_near_y": 85},
        metadata={"profile_id": 2, "session_token": 12})
    assert collector.reason == "MARK_ORIGIN_MISMATCH"


def test_three_markers_export_side_record():
    collector = VisualEntryCalibrationCollector()
    assert collector.mark(1, MARK_FIRST, _stable(10, 60))
    assert collector.mark(1, MARK_LOWER_OPENING,
                          {"entry_transition_far_y": 35,
                           "entry_transition_near_y": 85})
    assert collector.mark(1, MARK_UPPER_ENTRANCE,
                          {"entry_transition_far_y": 20,
                           "entry_transition_near_y": 70})
    record = collector.build_side(1)
    assert record["profile_ids"] == [2, 3]
    assert record["far_y_min"] == 17
    assert record["near_y_max"] == 73
    assert record["first_far_y"] == 10
    assert record["lower_near_y"] == 85


def test_upper_tolerance_zone_cannot_overlap_negative_marker():
    collector = VisualEntryCalibrationCollector()
    assert collector.mark(1, MARK_FIRST, _stable(20, 70))
    assert collector.mark(1, MARK_LOWER_OPENING,
                          {"entry_transition_far_y": 40,
                           "entry_transition_near_y": 90})
    assert collector.mark(1, MARK_UPPER_ENTRANCE,
                          {"entry_transition_far_y": 25,
                           "entry_transition_near_y": 75})
    assert collector.build_side(1) is None
    assert collector.reason == "UPPER_MARKER_EXCLUSION_OVERLAP"


def test_mark_can_require_private_grayscale_snapshot_and_metadata():
    class _Image:
        def width(self):
            return 320

        def height(self):
            return 120

        def save(self, path):
            pathlib.Path(path).write_bytes(b"P5\\n1 1\\n255\\n\\x00")

    with tempfile.TemporaryDirectory() as directory:
        collector = VisualEntryCalibrationCollector(
            snapshot_dir=str(pathlib.Path(directory) / "marks"))
        image = _Image()
        geometry = {"entry_transition_far_y": 15,
                    "entry_transition_near_y": 65}
        assert collector.mark(1, MARK_FIRST, _stable(15, 65), image=image,
                              require_snapshot=True,
                              metadata={"frame_id": 7, "profile_id": 3})
        assert collector.snapshots[(1, MARK_FIRST)]["width"] == 320
        assert (pathlib.Path(directory) / "marks" /
                "side1_mark0.pgm").exists()
        metadata = pathlib.Path(directory) / "marks" / "side1_mark0.json"
        parsed = json.loads(metadata.read_text())
        assert parsed["schema"] == "VISUAL_ENTRY_MARK_V1"
        assert parsed["frame_id"] == 7
        assert not collector.mark(1, MARK_LOWER_OPENING, geometry,
                                  require_snapshot=True)
        assert collector.reason == "MARK_SNAPSHOT_UNAVAILABLE"


if __name__ == "__main__":
    tests = [value for name, value in globals().items()
             if name.startswith("test_") and callable(value)]
    for test in tests:
        test()
    print("visual entry gate: %d tests passed" % len(tests))
