"""Release contracts for one parked pass and independent entrance authority."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import entry_calibration_common as common
import visual_entry_gate as visual


class GrayFrame:
    def width(self):
        return 320

    def height(self):
        return 120

    def save(self, path):
        Path(path).write_bytes(b'P5\n320 120\n255\n' + bytes(320 * 120))


def observation(far=30, near=52, side=1):
    return dict(state_id=3, candidate=True, ring_confirmed=True, side=side,
                entry_transition=True, entry_transition_far_y=far,
                entry_transition_near_y=near, ring_first_stable=True,
                ring_stable_observation=True, parked_ring_confirmed=True)


def collect(directory, side=1):
    collector = visual.VisualEntryCalibrationCollector(
        snapshot_dir=str(Path(directory) / 'marks'))
    for marker, far, near, position in ((0, 5, 25, 100),
                                       (1, 15, 35, 250),
                                       (2, 30, 52, 600)):
        accepted = collector.mark(
            side, marker, observation(far, near, side), image=GrayFrame(),
            require_snapshot=True, millirev=position, position_fresh=True,
            metadata=dict(frame_id=marker, profile_id=3, session_token=7,
                          mean_forward_wheel_millirev=position))
        if not accepted:
            raise AssertionError(collector.reason)
    calibration = collector.build_calibration(
        side=side, require_snapshots=True, require_encoder=True)
    if calibration is None:
        raise AssertionError(collector.reason)
    return collector, calibration


class EntryReleaseContracts(unittest.TestCase):
    def test_parked_observer_uses_real_ring_inputs_without_control_authority(self):
        from test_ring_entry_transition import RingEntryTransitionTests
        import track_elements as te
        for side in (0, 1):
            h = RingEntryTransitionTests()._harness()
            def parked(now, profile, _active, token, *args):
                return h.tracker.observe_parked(now, profile, 7, *args)
            h.tracker.update = parked
            h.prime()
            h.lose(side)
            for outward in (6, 8, 10):
                status, edges, mask = h.entry_arc_inputs(side, outward)
                result = h.frame(road_accepted=False, raw_status=status,
                    raw_edge2=edges, mask_info=mask)
            self.assertEqual(result['state_id'], te.STATE_INSIDE, result['reason'])
            self.assertFalse(result['control_eligible'])
            self.assertFalse(result['ring_confirmed'])
            self.assertFalse(result['candidate'])
            self.assertTrue(result['parked_ring_confirmed'])
            self.assertTrue(result['ring_first_stable'])
            collector = visual.VisualEntryCalibrationCollector()
            self.assertTrue(collector.mark(side, 0, result, millirev=100))

    def test_one_pass_saves_three_images_and_two_independent_crc_documents(self):
        with tempfile.TemporaryDirectory() as directory:
            collector, calibration = collect(directory)
            visual_path = str(Path(directory) / 'visual.json')
            encoder_path = str(Path(directory) / 'encoder.json')
            checked = calibration.save(visual_path)
            encoder = calibration.save_encoder(encoder_path)
            self.assertTrue(checked.ready)
            self.assertEqual(encoder['right']['threshold_millirev'], 500)
            self.assertEqual(common.validate_encoder_entry_document(encoder), encoder)
            self.assertEqual(len(list((Path(directory) / 'marks').glob('*.pgm'))), 3)
            metadata = json.loads((Path(directory) / 'marks/side1_mark2.json').read_text())
            self.assertEqual(metadata['mean_forward_wheel_millirev'], 600)
            data = json.loads(Path(visual_path).read_text())
            data['right']['upper_far_y'] += 1
            Path(visual_path).write_text(json.dumps(data))
            self.assertFalse(visual.VisualEntryCalibration.load(visual_path).ready)

    def test_plain_straight_baseline_cannot_be_first_ring_marker(self):
        collector = visual.VisualEntryCalibrationCollector()
        self.assertFalse(collector.mark(1, 0, dict(
            first_stable=True, stable_observation=True, baseline_ready=True,
            cross_far_y=5, cross_near_y=25), millirev=100))
        self.assertEqual(collector.reason, 'FIRST_STABLE_REQUIRED')

    def test_mixed_parked_sessions_cannot_commit_a_distance(self):
        collector = visual.VisualEntryCalibrationCollector()
        self.assertTrue(collector.mark(1, 0, observation(5, 25), millirev=100,
            metadata=dict(profile_id=3, session_token=7)))
        self.assertFalse(collector.mark(1, 1, observation(15, 35), millirev=250,
            metadata=dict(profile_id=3, session_token=8)))
        self.assertEqual(collector.reason, 'MARK_ORIGIN_MISMATCH')

    def test_lower_opening_cannot_take_over_but_upper_requires_consecutive_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            _, calibration = collect(directory)
            gate = visual.VisualEntryGate(calibration)
            for index in range(6):
                status = gate.update(observation(15, 35), profile_id=3,
                                     session_token=7, now_ms=index * 10)
                self.assertFalse(status['ready'])
            for index in range(2):
                self.assertFalse(gate.update(observation(), profile_id=3,
                    session_token=7, now_ms=100 + index * 10)['ready'])
            self.assertTrue(gate.update(observation(), profile_id=3,
                session_token=7, now_ms=120)['ready'])
            self.assertFalse(gate.update(observation(), valid=False, profile_id=3,
                session_token=7, now_ms=130)['ready'])
            self.assertFalse(gate.update(observation(), profile_id=3,
                session_token=7, now_ms=140)['ready'])

    def test_expired_approach_cannot_rearm_at_a_late_upper_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            _, calibration = collect(directory)
            gate = visual.VisualEntryGate(calibration, candidate_timeout_ms=100)
            gate.update(observation(15, 35), profile_id=3, session_token=7, now_ms=0)
            for now in (100, 110, 120, 130, 140):
                self.assertFalse(gate.update(observation(), profile_id=3,
                    session_token=7, now_ms=now)['ready'])

    def test_active_exit_state_cannot_switch_to_uncalibrated_direction(self):
        with tempfile.TemporaryDirectory() as directory:
            _, calibration = collect(directory)
            gate = visual.VisualEntryGate(calibration)
            for now in (0, 10, 20, 30):
                gate.update(observation(), profile_id=3, session_token=7, now_ms=now)
            changed = observation(side=0)
            changed['state_id'] = 4
            self.assertFalse(gate.update(changed, profile_id=3,
                session_token=7, now_ms=40)['ready'])

    def test_rename_failure_never_acknowledges_fallback_as_new_save(self):
        with tempfile.TemporaryDirectory() as directory:
            _, calibration = collect(directory)
            path = str(Path(directory) / 'visual.json')
            calibration.save(path)
            original_generation = calibration.generation
            calibration.generation += 1
            original_rename = visual._os.rename

            def fail_replacement(source, destination):
                if source.endswith('.new'):
                    raise OSError('injected SD replacement failure')
                return original_rename(source, destination)

            with patch.object(visual._os, 'rename', side_effect=fail_replacement):
                with self.assertRaises(OSError):
                    calibration.save(path)
            self.assertEqual(visual.VisualEntryCalibration.load(path).generation,
                             original_generation)


if __name__ == '__main__':
    unittest.main()
