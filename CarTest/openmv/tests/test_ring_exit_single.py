"""Current single exit geometry must reach the real main-loop UART path."""
from array import array
import unittest
import test_ring_exit_guidance as guidance
import track_elements as te


class SingleExitTests(unittest.TestCase):
    def fixture(self, side):
        h = guidance.ExitGuidanceTests().tracker(side, te.STATE_EXIT_CONFIRM)
        return h

    def single(self, h, shift=0, **kwargs):
        return h.frame(left=25+shift, right=55+shift,
            road_accepted=False, raw_side=1-h.tracker._side,
            mask_info=h.mask(left_by_y=lambda y:25+shift,
                             right_by_y=lambda y:55+shift), **kwargs)

    def ready(self, side):
        h = self.fixture(side)
        for _ in range(3):
            r = self.single(h)
            if r['target_valid']:
                h.tracker.commit_transmitted_route(h.now, True, True)
        return h

    def test_three_new_single_frames_confirm_without_bilateral(self):
        for side in (0, 1):
            h = self.fixture(side)
            for i in range(3):
                r = self.single(h)
                self.assertEqual(r['exit_single_count'], i+1)
                self.assertEqual(r['state_id'], te.STATE_EXIT_REPAIR if i == 2
                                 else te.STATE_EXIT_CONFIRM)
            self.assertTrue(r['target_valid'])
            self.assertTrue(r['outer_tracking'])
            self.assertEqual(r['target_side'], 1-side)
            self.assertEqual(r['exit_admission_reason'], 'EXIT_CURRENT_SINGLE')
            self.assertEqual(h.tracker._exit_reobserve_count, 0)

    def test_single_guidance_progresses_past_frozen_anchor_only_with_new_edges(self):
        for side in (0, 1):
            h = self.ready(side)
            t = h.tracker
            frozen = list(t._wire_centers2)
            for shift in (2, 4, 6, 8, 10):
                r = self.single(h, shift if side else -shift)
                self.assertTrue(r['target_valid'], r['exit_single_reason'])
                self.assertTrue(t.commit_transmitted_route(h.now, True, True))
                self.assertTrue(t.outer_route_valid_for(r))
            self.assertEqual(list(t._wire_centers2), frozen)
            self.assertEqual(r['target_center2'], 100 if side else 60)
            self.assertTrue(t._ring_cache_frozen)

    def test_single_repair_keeps_ring_identity_until_bilateral_reacquisition(self):
        h = self.ready(1)
        for _ in range(h.tracker.exit_repair_frames+5):
            r = self.single(h)
            self.assertTrue(r['target_valid'])
            h.tracker.commit_transmitted_route(h.now, True, True)
        self.assertEqual(r['state_id'], te.STATE_REACQUIRE)
        self.assertTrue(r['ring_confirmed'])
        for _ in range(12):
            r = h.frame()
            if r['target_valid']:
                h.tracker.commit_transmitted_route(h.now, True, True)
        self.assertEqual(r['state_id'], te.STATE_OBSERVE)
        self.assertFalse(h.tracker._ring_cache_frozen)

    def test_interruption_restarts_three_frame_confirmation(self):
        for kind in ('gap', 'duplicate_time', 'branch', 'missing', 'invalid'):
            h = self.fixture(1)
            self.single(h); self.single(h)
            if kind == 'gap': self.single(h, advance_ms=250)
            elif kind == 'duplicate_time': self.single(h, advance_ms=0)
            elif kind == 'branch':
                self.single(h, topology=dict(complete=True, reason='BRANCH',
                                             branch_evidence=True))
            elif kind == 'missing': h.frame(road_accepted=False)
            else:
                t = h.tracker
                t.update(h.now+1, 2, True, t._session_token, False, False,
                         None, 0, 0, 0, {}, False,
                         None, None, None, None, None, None, None, None)
            self.assertLess(h.tracker._exit_single_count, 3, kind)
            r = self.single(h)
            self.assertLess(r['exit_single_count'], 3, kind)

    def test_curve_unknown_dark_island_and_unanchored_line_do_not_certify(self):
        for kind in ('curve', 'unknown', 'dark', 'island', 'stale', 'jump',
                     'ambiguous', 'wrong_width', 'no_gap', 'no_calibration'):
            h = self.fixture(1); t = h.tracker
            args = {}
            if kind == 'unknown':
                m=h.mask();m['known_region'][:]=bytearray(4800);args['mask_info']=m
            if kind == 'dark': args['mask_info']=h.mask(dark_hole=(55,25,55))
            if kind == 'island':
                args['topology']=dict(complete=True,reason='ISLAND',island=True,
                                     ambiguous=True, independent_branch_checked=True)
            if kind == 'stale': t._selected_route_ms=h.now-2001
            if kind == 'jump':
                for y in range(t.height): t._wire_centers2[y]-=30
            if kind == 'no_gap': t._exit_side=te.SIDE_NONE
            if kind == 'no_calibration': t._cal_check_ready=False
            if kind in ('curve', 'ambiguous', 'wrong_width'):
                status=bytearray(120); edges=array('i',[0]*120)
                for y in range(36,60):
                    status[y]=1
                    edges[y]=49+((y-48)**2//5 if kind=='curve' else 0)
                    if kind=='ambiguous':status[60+y]=2
                    if kind=='wrong_width':
                        status[60+y]=1;edges[60+y]=75
                args.update(raw_status=status,raw_edge2=edges)
            for _ in range(5):
                if 'raw_status' in args:
                    r=h.frame(road_accepted=False,**args)
                else:
                    call=dict(left=25,right=55,road_accepted=False,raw_side=0,
                              mask_info=h.mask());call.update(args);r=h.frame(**call)
            # One blocked row can be discarded by the pre-existing visible
            # corridor filter; it cannot be represented as an observed target.
            if kind=='dark':
                self.assertTrue(all(r['target_rows'][i*4]!=55
                                    for i in range(r['target_count'])))
            else:
                self.assertFalse(t._exit_single_ready(),kind)

    def test_no_valid_write_or_old_geometry_cannot_extend_last_sent_deadline(self):
        h=self.ready(1);t=h.tracker;stamp=t._selected_route_ms
        for _ in range(70):
            self.single(h)
            self.assertFalse(t.commit_transmitted_route(h.now, True, False))
        self.assertEqual(t._selected_route_ms,stamp)
        self.assertFalse(t._exit_single_ready())

    def test_inside_without_transmitted_inner_route_cannot_invent_exit(self):
        h=self.fixture(1);h.tracker._state=te.STATE_INSIDE
        h.tracker._inner_route_established=False
        for _ in range(6):r=self.single(h)
        self.assertEqual(r['state_id'],te.STATE_INSIDE)
        self.assertFalse(h.tracker._exit_single_ready())

    def test_certificate_rejects_copied_stale_or_mutated_target(self):
        h=self.ready(1);t=h.tracker;r=t.result
        self.assertTrue(t.outer_route_valid_for(r))
        self.assertFalse(t.outer_route_valid_for(dict(r)))
        # The continuity shortcut must still check the full current shape.
        t._selected_route_center2=-100
        off=0
        t._target_rows[off+1]+=20;t._target_rows[off+2]+=20
        self.assertFalse(t.outer_route_valid_for(r))
        t._target_rows[off+1]-=20;t._target_rows[off+2]-=20
        t._frame_index+=1
        self.assertFalse(t.outer_route_valid_for(r))

    def test_identity_change_and_invalid_frame_clear_confirmation(self):
        h=self.ready(1);t=h.tracker
        r=h.frame(active=False)
        self.assertFalse(t._exit_single_ready())
        self.assertFalse(t.outer_route_valid_for(r))
        h=self.ready(1);t=h.tracker
        r=t.update(h.now+1,2,True,t._session_token,False,False,
                   None,0,0,0,{},False,
                   None,None,None,None,None,None,None,None)
        self.assertEqual(r['exit_single_count'],0)
        self.assertFalse(r['target_valid'])
        self.assertFalse(t.outer_route_valid_for(r))

    def uart_packet(self, side, shift):
        import config
        import test_fast_main_ring_slope as wire
        h=self.ready(side);t=h.tracker
        steps=list(range(2,abs(shift)+1,2))
        if not steps or steps[-1]!=abs(shift):steps.append(abs(shift))
        for amount in steps:
            result=self.single(h, amount if shift>0 else -amount)
            self.assertTrue(result['target_valid'])
            t.commit_transmitted_route(h.now, True, True)
        t._profile_id=3;t._session_token=1
        result.update(profile_id=3,profile_seq=11,session_token=1)
        class ActualExit(wire._ActiveRingElements):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.result=result
                self.require_calibration=True
            def outer_route_valid_for(self,r):return t.outer_route_valid_for(r)
        # Supply the actual observed left/right edge rows to the final converter.
        from unittest.mock import patch
        original=wire._SingleEdgeTracker.__init__
        def init(instance,*a,**kw):
            original(instance,*a,**kw)
            instance._row_status[:]=t._current_status
            instance._row_edge2[:]=t._current_edge2
            instance._frame_scan_done=True
        with patch.object(wire._SingleEdgeTracker,'__init__',init):
            packet,log,_=wire.FastMainRingSlopeWireTests()._run_profile_element_case(
                ActualExit,config.VEHICLE_PROFILE_C)
        self.assertTrue(packet[6]&1,log)
        self.assertEqual(packet[11],3)
        return bytes(packet)

    def test_actual_single_exit_targets_reach_uart_with_mirrored_errors(self):
        errors=[]
        # Grid centre is x2=79; the harness's 25..55 lane has x2=80.
        for side,shift in ((0,-8),(1,7)):
            packet=self.uart_packet(side,shift)
            error=int.from_bytes(packet[8:10],'little',signed=True)
            self.assertGreater(error*shift,0)
            errors.append(error)
        self.assertEqual(errors[0],-errors[1])


if __name__=='__main__':unittest.main()
