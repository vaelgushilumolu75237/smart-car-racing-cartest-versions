"""r16: fresh exit geometry, immutable wire anchor and final UART admission."""
from array import array
import unittest
import test_ring_exit_guidance as guidance
import track_elements as te


class ExitAdmissionTests(unittest.TestCase):
    def fixture(self, side):
        return guidance.ExitGuidanceTests().tracker(side)

    def shifted(self, h, shift, paired=False):
        return h.frame(left=25+shift, right=55+shift,
            road_accepted=paired, raw_side=None if paired else h.tracker._side,
            mask_info=h.mask(left_by_y=lambda y:25+shift,
                             right_by_y=lambda y:55+shift))

    def test_small_steps_cannot_walk_exit_anchor_toward_straight(self):
        for side in (0, 1):
            h = self.fixture(side)
            t = h.tracker
            frozen = list(t._wire_centers2)
            anchor_time = t._wire_route_ms
            for shift in (2, 4, 6):
                r = self.shifted(h, shift)
                self.assertTrue(r['target_valid'])
                self.assertTrue(t.commit_transmitted_route(h.now, True, True))
            r = self.shifted(h, 8)
            self.assertFalse(r['target_valid'])
            self.assertEqual(r['ring_target_rejection'], 'EXIT_GUIDANCE_UNTRUSTED')
            self.assertFalse(t.commit_transmitted_route(h.now, False, True))
            self.assertEqual(list(t._wire_centers2), frozen)
            self.assertEqual(t._wire_route_ms, anchor_time)

    def test_current_bilateral_exit_can_legitimately_straighten(self):
        for side in (0, 1):
            h = self.fixture(side)
            t = h.tracker
            for i in range(t.exit_repair_frames):
                r = self.shifted(h, 8, paired=True)
                self.assertEqual(r['target_valid'], i+1 == t.exit_repair_frames)
            self.assertEqual(r['exit_admission_reason'], 'EXIT_CURRENT_PAIRED')
            self.assertEqual(r['target_side'], side)
            for _ in range(10):
                r = self.shifted(h, 8, paired=True)
            self.assertEqual(r['state'], 'OBSERVE')

    def test_interrupted_pair_evidence_does_not_release_anchor(self):
        h = self.fixture(1)
        for _ in range(3):
            self.shifted(h, 8, True)
        self.shifted(h, 8, False)
        for _ in range(3):
            self.assertFalse(self.shifted(h, 8, True)['target_valid'])

    def test_same_reference_cannot_hide_changed_weighted_rows(self):
        h = self.fixture(1)
        t = h.tracker
        r = self.shifted(h, 0)
        self.assertTrue(r['target_valid'])
        for i in range(r['target_count']):
            off = i*4
            if r['target_rows'][off] != t.reference_y:
                r['target_rows'][off+1] += 8
                r['target_rows'][off+2] += 8
        self.assertFalse(t._exit_target_admitted())

    def test_invalid_short_write_and_duplicate_cannot_promote_reference(self):
        h = self.fixture(0)
        t = h.tracker
        self.shifted(h, 2)
        before = (t._selected_route_ms, t._selected_route_center2)
        for valid, full in ((False, True), (True, False), (False, False)):
            self.assertFalse(t.commit_transmitted_route(h.now, valid, full))
            self.assertEqual((t._selected_route_ms,t._selected_route_center2),before)
        # Candidate selection also cannot promote history.
        t.observe_route_outcome(h.now, True, False)
        self.assertEqual((t._selected_route_ms,t._selected_route_center2),before)
        self.assertTrue(t.commit_transmitted_route(h.now, True, True))
        self.assertFalse(t.commit_transmitted_route(h.now, True, True))

    def test_missing_stale_anchor_and_session_reset_fail_closed(self):
        for kind in ('missing', 'stale'):
            h = self.fixture(1)
            t = h.tracker
            if kind == 'missing':
                t._wire_route_ms = None
            else:
                t._wire_route_ms = h.now-2001
            self.assertFalse(self.shifted(h, 0)['target_valid'])
        t.reset('TEST')
        self.assertIsNone(t._wire_route_ms)
        self.assertTrue(all(v == -1 for v in t._wire_centers2))

    def test_inside_current_geometry_is_not_limited_by_exit_anchor(self):
        h = self.fixture(1)
        h.tracker._state = te.STATE_INSIDE
        self.assertTrue(self.shifted(h, 8)['target_valid'])

    def test_paired_shape_jump_resets_count_even_if_reference_stays_still(self):
        h = self.fixture(1)
        t = h.tracker
        for _ in range(3):
            self.shifted(h, 8, True)
        # Simulate the previous paired far rows on a different branch while
        # keeping the measured reference point and current geometry unchanged.
        for y in range(t.floor_y, t.reference_y-4):
            t._exit_pair_centers2[y] -= t.handoff_max_center_delta2+1
        self.assertFalse(self.shifted(h, 8, True)['target_valid'])
        self.assertEqual(t._exit_reobserve_count, 1)

    def test_rejected_route_sends_invalid_uart_and_never_cached_valid(self):
        import config
        import test_fast_main_ring_slope as wire
        h = self.fixture(1)
        result = dict(self.shifted(h, 8))
        result.update(profile_id=3, profile_seq=11, session_token=1)
        class RejectedExit(wire._ActiveRingElements):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.result = result
        packet, log, _ = wire.FastMainRingSlopeWireTests()._run_profile_element_case(
            RejectedExit, config.VEHICLE_PROFILE_C)
        self.assertFalse(packet[6] & 1, log)
        self.assertEqual(int.from_bytes(packet[8:10], 'little', signed=True), 0)

    def test_main_promotes_only_after_full_valid_packet(self):
        import config
        import test_fast_main_ring_slope as wire
        calls = []
        class WireExit(wire._ActiveRingElements):
            def commit_transmitted_route(self, now, valid, full):
                calls.append((valid, full))
        packet, _, _ = wire.FastMainRingSlopeWireTests()._run_profile_element_case(
            WireExit, config.VEHICLE_PROFILE_C)
        self.assertTrue(packet[6] & 1)
        self.assertEqual(calls, [(True, True)])


if __name__ == '__main__':
    unittest.main()
