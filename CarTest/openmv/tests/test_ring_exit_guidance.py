"""Measured inner/outer exit guidance and guarded ordinary handover."""
from array import array
import unittest
import test_ring_exit_pending_route as pending
import track_elements as te


class ExitGuidanceTests(unittest.TestCase):
    def tracker(self, side, state=te.STATE_EXIT_REPAIR):
        h = pending.PendingExitRouteTests().tracker(side)
        t = h.tracker
        t._state = state
        t._exit_side = 1-side
        t._exit_drop_count = t.exit_drop_frames
        t._inner_route_established = True
        t._selected_route_ms = h.now
        t._selected_route_center2 = 80
        t._selected_route_side = side
        return h

    def seed_previous(self, h, center2=80, side=None):
        t = h.tracker
        t._selected_route_ms = h.now
        t._selected_route_center2 = center2
        if side is not None:
            t._selected_route_side = side

    def test_repair_keeps_inner_and_obeys_configured_count_both_sides(self):
        for side in (0, 1):
            h = self.tracker(side)
            t = h.tracker
            for i in range(t.exit_repair_frames):
                self.seed_previous(h)
                r = h.frame()
                self.assertEqual(r['target_side'], side)
                self.assertTrue(r['target_valid'])
                self.assertEqual(r['state_id'], te.STATE_REACQUIRE if
                                 i+1 == t.exit_repair_frames else te.STATE_EXIT_REPAIR)
            self.assertTrue(r['ring_confirmed'])
            self.assertTrue(t._ring_cache_frozen)
            self.assertEqual(r['target_center2'], 80)

    def test_failed_repair_resets_counter_and_does_not_unlock(self):
        for side in (0, 1):
            h = self.tracker(side)
            self.seed_previous(h)
            h.frame()
            self.assertEqual(h.tracker._repair_count, 1)
            r = h.frame(road_accepted=False, raw_side=side)
            self.assertEqual(h.tracker._repair_count, 0)
            self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)
            self.assertTrue(h.tracker._ring_cache_frozen)

    def test_outer_fallback_uses_calibrated_width_and_continuous_handoff(self):
        for state in (te.STATE_EXIT_CONFIRM, te.STATE_EXIT_REPAIR, te.STATE_REACQUIRE):
            for side in (0, 1):
                h = self.tracker(side, state)
                r = h.frame(road_accepted=False, raw_side=1-side)
                self.assertTrue(r['target_valid'], (state, side, r['outer_rejection']))
                self.assertTrue(r['outer_tracking'])
                self.assertEqual(r['target_side'], 1-side)
                self.assertEqual(r['target_center2'], 80)
                for i in range(r['target_count']):
                    _, left, right, _ = r['target_rows'][i*4:i*4+4]
                    self.assertEqual(right-left+1, 31)

    def test_jump_stale_missing_unknown_or_branch_fails_closed(self):
        for kind in ('jump', 'stale', 'missing', 'unknown', 'branch'):
            for side in (0, 1):
                h = self.tracker(side)
                args = dict(road_accepted=False, raw_side=1-side)
                if kind == 'jump': h.tracker._selected_route_center2 = 40
                if kind == 'stale': h.tracker._selected_route_ms = h.now-2001
                if kind == 'missing':
                    args = dict(road_accepted=False, raw_status=bytearray(120),
                                raw_edge2=array('i', [0]*120))
                if kind == 'unknown':
                    mask = h.mask()
                    mask['known_region'][:] = bytearray(4800)
                    args['mask_info'] = mask
                if kind == 'branch':
                    args['topology'] = dict(reason='BRANCH', complete=True,
                                            branch_evidence=True)
                r = h.frame(**args)
                self.assertFalse(r['target_valid'], (kind, side))
                self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)

    def test_inner_restoration_cannot_jump_from_outer_route(self):
        for side in (0, 1):
            h = self.tracker(side)
            self.seed_previous(h, 40, 1-side)
            r = h.frame(road_accepted=False, raw_side=side)
            self.assertFalse(r['target_valid'])
            self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)

    def test_bilateral_jump_and_old_history_cannot_release_exit(self):
        for kind in ('jump', 'stale'):
            h = self.tracker(1)
            h.tracker._selected_route_center2 = 40 if kind == 'jump' else 80
            if kind == 'stale': h.tracker._selected_route_ms = h.now-2001
            for _ in range(h.tracker.exit_repair_frames-1):
                r = h.frame()
                self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)
                self.assertEqual(h.tracker._repair_count, 0)
            # Fresh independent paired evidence permits a bounded restart,
            # but still cannot skip the configured repair/reacquire sequence.
            r = h.frame()
            self.assertEqual(h.tracker._exit_reobserve_count,
                             h.tracker.exit_repair_frames)
            self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)
            for _ in range(h.tracker.exit_repair_frames): r = h.frame()
            self.assertEqual(r['state_id'], te.STATE_REACQUIRE)
            self.assertTrue(h.tracker._ring_cache_frozen)

    def test_timeout_and_false_entry_recovery_do_not_bypass_exit(self):
        h = self.tracker(1)
        h.tracker._state_start_ms = h.now-h.tracker.max_active_ms-1
        r = h.frame()
        self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)
        self.assertFalse(r['target_valid'])
        self.assertTrue(r['ring_confirmed'])
        self.assertTrue(h.tracker._ring_cache_frozen)
        for _ in range(5):
            self.assertFalse(h.tracker.observe_route_outcome(h.now, False, True))
        self.assertEqual(h.tracker._state, te.STATE_EXIT_REPAIR)

    def test_bilateral_reobservation_breaks_on_gap_width_or_branch(self):
        for kind in ('time', 'width', 'branch', 'single', 'shift'):
            h = self.tracker(1)
            self.seed_previous(h, 40, 0)
            for _ in range(3): h.frame()
            self.assertEqual(h.tracker._exit_reobserve_count, 3)
            args = {}
            if kind == 'time': args['advance_ms'] = 251
            if kind == 'width': args['right'] = 64
            if kind == 'branch':
                args['topology'] = dict(reason='BRANCH', complete=True,
                                        branch_evidence=True)
            if kind == 'single': args.update(road_accepted=False, raw_side=1)
            if kind == 'shift':
                args.update(left=33, right=63,
                    mask_info=h.mask(left_by_y=lambda y:33,
                                     right_by_y=lambda y:63))
            r = h.frame(**args)
            self.assertLess(h.tracker._exit_reobserve_count, 4)
            self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR)
            self.assertEqual(h.tracker._repair_count, 0)

    def test_current_paired_reobservation_cannot_authorize_unseen_inner_edge(self):
        h = self.tracker(1)
        self.seed_previous(h, 40, 0)
        for _ in range(3): h.frame()
        r = h.frame(road_accepted=False, raw_side=0)
        self.assertEqual(h.tracker._exit_reobserve_count, 0)
        self.assertFalse(r['target_valid'])

    def test_actual_inner_targets_reach_final_uart_with_mirrored_errors(self):
        import config
        import test_fast_main_ring_slope as wire
        runner = wire.FastMainRingSlopeWireTests()
        for side, shift in ((0, -4), (1, 4)):
            h = self.tracker(side)
            self.seed_previous(h, 80+2*shift)
            result = dict(h.frame(left=25+shift, right=55+shift,
                mask_info=h.mask(left_by_y=lambda y:25+shift,
                                 right_by_y=lambda y:55+shift)))
            result['target_rows'] = array('i', result['target_rows'])
            result.update(profile_id=3, profile_seq=11, session_token=1)
            self.assertTrue(result['target_valid'])
            self.assertEqual(result['target_side'], side)
            class ExitElements(wire._ActiveRingElements):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.result = result
            packet, log, _ = runner._run_profile_element_case(
                ExitElements, config.VEHICLE_PROFILE_C)
            self.assertTrue(packet[6] & 1, log)
            self.assertEqual(packet[11], 3)
            error = int.from_bytes(packet[8:10], 'little', signed=True)
            self.assertGreater(error*shift, 0)

    def test_reacquire_without_target_cannot_leak_an_ordinary_uart_route(self):
        import config
        import test_fast_main_ring_slope as wire
        class GuardedReacquire(wire._ConfirmedRingNoTarget):
            STATE_ID_NAME = 'STATE_REACQUIRE'
            STATE = 'REACQUIRE'
        packet, log, _ = wire.FastMainRingSlopeWireTests()._run_profile_element_case(
            GuardedReacquire, config.VEHICLE_PROFILE_C)
        self.assertFalse(packet[6] & 1, log)
        self.assertEqual(int.from_bytes(packet[8:10], 'little', signed=True), 0)
        self.assertIn('RING_TARGET_UNSUPPORTED', log)


if __name__ == '__main__':
    unittest.main()
