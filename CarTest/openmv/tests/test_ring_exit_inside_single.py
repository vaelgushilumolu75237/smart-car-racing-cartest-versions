"""Enter a ring through measured frames, then admit its single-edge exit."""
from array import array
import unittest
import test_ring_entry_transition as entry
import test_ring_exit_single as single
import track_elements as te


class InsideSingleExitTests(unittest.TestCase):
    def fixture(self, side):
        h, result, _, _, _ = entry.RingEntryTransitionTests()._prepare_inside(
            side, near_inner_rows=5)
        self.assertEqual(result['state_id'], te.STATE_INSIDE)
        self.assertTrue(result['ring_confirmed'])
        # Real current geometry and a complete transmission establish the
        # lane. Do not seed EXIT_CONFIRM, an exit gap or a frozen wire anchor.
        result = h.frame()
        self.assertTrue(result['target_valid'], result)
        self.assertTrue(h.tracker.commit_transmitted_route(h.now, True, True))
        self.assertFalse(h.tracker._entry_transition_active)
        self.assertEqual(h.tracker._exit_side, te.SIDE_NONE)
        return h

    def outer(self, h, shift=0, **kwargs):
        args = dict(left=25+shift, right=55+shift, road_accepted=False,
                    raw_side=1-h.tracker._side,
                    mask_info=h.mask(lambda y:25+shift, lambda y:55+shift))
        args.update(kwargs)
        if 'raw_status' in kwargs:
            args['raw_side'] = None
        return h.frame(**args)

    def test_real_entry_then_three_outer_frames_admit_exit_without_inner_anchor(self):
        for side in (0, 1):
            h = self.fixture(side)
            for i in range(3):
                r = self.outer(h)
                self.assertEqual(r['exit_single_count'], i+1)
                self.assertEqual(r['state_id'], te.STATE_EXIT_CONFIRM if i == 2
                                 else te.STATE_INSIDE)
                self.assertEqual(h.tracker._near_left_rows if side == 0 else
                                 h.tracker._near_right_rows, 0)
                self.assertTrue(r['target_valid'])
                h.tracker.commit_transmitted_route(h.now, True, True)
            self.assertEqual(r['reason'], 'EXIT_SINGLE_CORRIDOR_OBSERVED')
            self.assertEqual(r['exit_admission_reason'], 'EXIT_CURRENT_SINGLE')
            self.assertEqual(h.tracker._exit_side, 1-side)
            self.assertTrue(h.tracker.outer_route_valid_for(r))
            self.assertTrue(h.tracker._ring_cache_frozen)

    def test_progress_to_reacquire_then_require_bilateral_release(self):
        for side in (0, 1):
            h = self.fixture(side)
            states = []
            for _ in range(15):
                r = self.outer(h)
                states.append(r['state_id'])
                if r['target_valid']:
                    h.tracker.commit_transmitted_route(h.now, True, True)
            for state in (te.STATE_EXIT_CONFIRM, te.STATE_EXIT_REPAIR, te.STATE_REACQUIRE):
                self.assertIn(state, states)
            self.assertTrue(r['ring_confirmed'])
            for _ in range(12):
                r = h.frame()
                if r['target_valid']:
                    h.tracker.commit_transmitted_route(h.now, True, True)
            self.assertEqual(r['state_id'], te.STATE_OBSERVE)
            self.assertFalse(h.tracker._ring_cache_frozen)

    def test_entry_lease_or_no_actual_write_cannot_arm_exit(self):
        for kind in ('lease','no_write','stale'):
            h = self.fixture(1);t = h.tracker
            if kind == 'lease': t._start_entry_transition(h.now)
            if kind == 'no_write': t._inner_route_established=False
            if kind == 'stale': t._selected_route_ms=h.now-2001
            for _ in range(5): r=self.outer(h)
            self.assertEqual(r['state_id'], te.STATE_INSIDE, kind)
            self.assertEqual(r['exit_single_count'], 0, kind)

    def test_inside_timeout_is_not_extended_by_single_geometry(self):
        h=self.fixture(1);t=h.tracker
        t._state_start_ms=h.now-t.max_active_ms-1
        started=t._state_start_ms
        for _ in range(5):
            r=self.outer(h)
            self.assertEqual(r['state_id'],te.STATE_INSIDE)
            self.assertEqual(r['reason'],'STATE_TIMEOUT')
            self.assertEqual(r['exit_single_reason'],'INSIDE_TIMEOUT')
            self.assertFalse(r['target_valid'])
        self.assertEqual(t._state_start_ms, started)
        self.assertEqual(t.max_active_ms,16000)

    def test_missing_or_branch_interrupts_confirmation(self):
        for kind in ('rows','branch','time','unknown'):
            h=self.fixture(1)
            self.outer(h);self.outer(h)
            kwargs={}
            if kind=='rows': kwargs.update(raw_side=None,single_fresh=False)
            if kind=='branch':kwargs['topology']=dict(complete=True,reason='BRANCH',branch_evidence=True)
            if kind=='time':kwargs['advance_ms']=250
            if kind=='unknown':
                m=h.mask();m['known_region'][:]=bytearray(4800);kwargs['mask_info']=m
            self.outer(h,**kwargs)
            r=self.outer(h)
            self.assertEqual(r['state_id'],te.STATE_INSIDE,kind)
            self.assertLess(r['exit_single_count'],3,kind)

    def test_curved_outer_edge_does_not_admit_exit(self):
        for side in (0,1):
            h=self.fixture(side)
            s=bytearray(120);e=array('i',[0]*120)
            for y in range(36,60):
                s[(1-side)*60+y]=1
                e[(1-side)*60+y]=(49 if side else 111)+(y-48)**2//5
            for _ in range(5):r=self.outer(h,raw_status=s,raw_edge2=e)
            self.assertEqual(r['state_id'],te.STATE_INSIDE)
            self.assertFalse(h.tracker._exit_single_ready())

    def uart_packet(self, side, shift):
        # Reuse the real main-loop converter/wire assertion, replacing its
        # fixture with the unprimed exit from measured ring entry above.
        case=single.SingleExitTests()
        case.fixture=self.fixture
        return case.uart_packet(side,shift)

    def test_real_entry_exit_reaches_actual_main_uart_both_directions(self):
        for side,shift in ((0,-8),(1,7)):
            packet=self.uart_packet(side,shift)
            self.assertTrue(packet[6]&1)
            self.assertEqual(packet[11],3)
            error=int.from_bytes(packet[8:10],'little',signed=True)
            self.assertGreater(error*shift,0)
