from pathlib import Path
import sys
import tempfile
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import encoder_entry_gate as gate


def _calibration():
    collector = gate.EncoderEntryCollector()
    collector.mark(gate.SIDE_LEFT, gate.MARK_FIRST, 100)
    collector.mark(gate.SIDE_LEFT, gate.MARK_LOWER_OPENING, 150)
    collector.mark(gate.SIDE_LEFT, gate.MARK_UPPER_ENTRANCE, 300)
    record = collector.build(gate.SIDE_LEFT)
    return gate.EncoderEntryCalibration(1, {gate.SIDE_LEFT: record}, "READY")


def _inside():
    return {"state_id": 3, "ring_confirmed": True,
            "candidate": True, "side": gate.SIDE_LEFT,
            "ring_first_stable": True, "ring_stable_observation": True}


class EncoderEntryGateTests(unittest.TestCase):
    def test_no_origin_from_plain_confirmed_or_straight_baseline(self):
        runtime = gate.EncoderEntryGate(_calibration())
        info = _inside()
        info.pop("ring_first_stable")
        info.pop("ring_stable_observation")
        info.update(first_stable=True, baseline_ready=True)
        state = runtime.update(info, profile_id=2, session_token=7,
                               millirev=1000, session_active=True)
        self.assertIsNone(runtime.start_millirev)
        self.assertFalse(state["ready"])
        state = runtime.update(_inside(), profile_id=2, session_token=7,
                               millirev=1100, session_active=True)
        self.assertEqual(runtime.start_millirev, 1100)

    def test_backward_stale_sentinel_and_timeout_latch_skip(self):
        for kwargs in (dict(millirev=999), dict(millirev=gate.UINT32_MAX),
                       dict(millirev=1001, position_fresh=False)):
            runtime = gate.EncoderEntryGate(_calibration())
            runtime.update(_inside(), profile_id=2, session_token=7,
                           millirev=1000, session_active=True)
            self.assertFalse(runtime.update(_inside(), profile_id=2,
                session_token=7, session_active=True, **kwargs)["ready"])
            self.assertFalse(runtime.update(_inside(), profile_id=2,
                session_token=7, millirev=1200, session_active=True)["ready"])
        runtime = gate.EncoderEntryGate(_calibration(), max_candidate_wait_frames=2)
        for position in (1000, 1001, 1002, 1200, 1201):
            self.assertFalse(runtime.update(_inside(), profile_id=2,
                session_token=7, millirev=position, session_active=True)["ready"])

    def test_active_gate_keeps_authority_after_overshoot(self):
        runtime = gate.EncoderEntryGate(
            _calibration(), max_overshoot_millirev=10)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1000, session_active=True)
        ready = dict(runtime.update(
            _inside(), profile_id=2, session_token=7,
            millirev=1200, session_active=True))
        active = dict(runtime.update(
            _inside(), profile_id=2, session_token=7,
            millirev=1201, session_active=True))
        overshot = dict(runtime.update(
            _inside(), profile_id=2, session_token=7,
            millirev=2000, session_active=True))
        self.assertEqual(ready["state"], gate.STATE_READY)
        self.assertEqual(active["state"], gate.STATE_ACTIVE)
        self.assertTrue(active["entry_evidence"])
        self.assertEqual(overshot["state"], gate.STATE_ACTIVE)
        self.assertTrue(overshot["entry_evidence"])

    def test_wait_gate_rejects_overshoot_before_admission(self):
        runtime = gate.EncoderEntryGate(
            _calibration(), max_overshoot_millirev=10)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1000, session_active=True)
        result = runtime.update(
            _inside(), profile_id=2, session_token=7,
            millirev=1211, session_active=True)
        self.assertEqual(result["state"], gate.STATE_WAIT)
        self.assertEqual(result["reason"], "ENTRY_DISTANCE_OVERSHOT")

    def test_active_gate_revokes_on_unconfirmed_or_changed_side(self):
        runtime = gate.EncoderEntryGate(_calibration())
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1000, session_active=True)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1200, session_active=True)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1201, session_active=True)
        revoked = runtime.update(
            {"state_id": 4, "ring_confirmed": True,
             "candidate": True, "side": gate.SIDE_LEFT},
            profile_id=2, session_token=7, millirev=1210,
            session_active=True)
        self.assertEqual(revoked["state"], gate.STATE_WAIT)
        self.assertEqual(revoked["reason"], "ENTRY_EVIDENCE_REVOKED")

        runtime.reset("TEST")
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1000, session_active=True)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1200, session_active=True)
        runtime.update(_inside(), profile_id=2, session_token=7,
                       millirev=1201, session_active=True)
        changed = runtime.update(
            {"state_id": 4, "ring_confirmed": True,
             "candidate": True, "side": gate.SIDE_RIGHT},
            profile_id=2, session_token=7, millirev=1210,
            session_active=True)
        self.assertEqual(changed["state"], gate.STATE_WAIT)
        self.assertEqual(changed["reason"], "SIDE_CHANGED")

    def test_collector_rejects_sentinel_marker(self):
        collector = gate.EncoderEntryCollector()
        result = collector.mark(gate.SIDE_LEFT, gate.MARK_FIRST,
                                gate.UINT32_MAX)
        self.assertIsNone(result)
        self.assertEqual(collector.reason, "POSITION_SENTINEL")

    def test_save_round_trip_and_last_good_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "entry.json")
            first = _calibration()
            self.assertEqual(first.save(path).generation, 1)
            second_collector = gate.EncoderEntryCollector()
            second_collector.mark(gate.SIDE_LEFT, gate.MARK_FIRST, 200)
            second_collector.mark(gate.SIDE_LEFT,
                                  gate.MARK_LOWER_OPENING, 260)
            second_collector.mark(gate.SIDE_LEFT,
                                  gate.MARK_UPPER_ENTRANCE, 450)
            second = first.replace(
                gate.SIDE_LEFT, second_collector.build(gate.SIDE_LEFT))
            self.assertEqual(second.save(path).generation, 2)
            self.assertEqual(
                gate.EncoderEntryCalibration.load(path).generation, 2)
            self.assertEqual(
                gate.EncoderEntryCalibration.load(path + ".previous").generation,
                1)


if __name__ == "__main__":
    unittest.main()
