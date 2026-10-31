"""Exit discovery and ownership across fresh frame / UART sequences."""
import unittest
from array import array
import test_ring_exit_single as single
import track_elements as te

class ExitOwnershipTests(unittest.TestCase):
    def fixture(self):
        h=single.SingleExitTests().fixture(te.SIDE_RIGHT)
        h.tracker._state=te.STATE_INSIDE
        h.tracker._exit_side=te.SIDE_NONE
        return h

    def left(self,h,shift=0,**kw):
        return single.SingleExitTests().single(h,shift,**kw)

    def depart(self,h):
        for _ in range(3):
            status=bytearray([0]*8+[1]*52+[0]*8+[1]*52)
            edges=array('i',[0]*120)
            for y in range(8,60):
                edges[y]=9 if y<32 else 49
                edges[60+y]=111
            r=h.frame(road_accepted=False,raw_status=status,raw_edge2=edges,
                mask_info=h.mask(left_by_y=lambda y:5 if y<32 else 25))
            self.assertTrue(r['target_valid'])
            self.assertEqual(r['target_side'],te.SIDE_RIGHT)
            h.tracker.commit_transmitted_route(h.now,True,True)
        self.assertTrue(h.tracker._exit_search_armed)

    def test_entry_straight_does_not_exit(self):
        h=self.fixture()
        for _ in range(8):
            r=self.left(h)
            self.assertNotEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')
            self.assertFalse(h.tracker._exit_search_armed)

    def test_departure_then_first_exit_changes_target_without_old_anchor(self):
        h=self.fixture();self.depart(h);t=h.tracker
        for y in range(t.height):t._wire_centers2[y]-=30
        r=self.left(h,-7)
        self.assertTrue(r['target_valid'])
        self.assertEqual(r['state_id'],te.STATE_EXIT_CONFIRM)
        self.assertEqual(r['target_side'],te.SIDE_LEFT)
        self.assertEqual(r['exit_single_count'],1)
        self.assertEqual(r['target_center2'],66)
        self.assertTrue(t.outer_route_valid_for(r))

    def test_discontinuous_ambiguous_stale_or_unestablished_frames_do_not_arm(self):
        for kind in ('gap','branch','ordinary','unknown','stale','no_inner'):
            h=self.fixture();t=h.tracker
            if kind=='stale':t._selected_route_ms=h.now-3001
            if kind=='no_inner':t._inner_route_established=False
            for _ in range(4):
                kw=dict(road_accepted=False,raw_side=te.SIDE_RIGHT)
                if kind=='gap':kw['advance_ms']=250
                if kind=='branch':kw['topology']=dict(complete=True,reason='BRANCH',branch_evidence=True)
                if kind=='ordinary':kw={}
                if kind=='unknown':
                    m=h.mask();m['known_region'][:]=bytearray(4800);kw['mask_info']=m
                h.frame(**kw)
            self.assertFalse(t._exit_search_armed,kind)

    def test_only_full_valid_uart_commit_locks_exit(self):
        h=self.fixture();self.depart(h);t=h.tracker
        self.assertTrue(self.left(h,-7)['target_valid'])
        self.assertFalse(t.commit_transmitted_route(h.now,True,False))
        self.assertFalse(t._exit_guidance_latched)
        self.assertTrue(t.commit_transmitted_route(h.now,True,True))
        self.assertTrue(t._exit_guidance_latched)

    def test_gap_cannot_select_inner_or_refresh_deadline_and_left_recovers(self):
        h=self.fixture();self.depart(h);t=h.tracker
        self.left(h,-7);t.commit_transmitted_route(h.now,True,True)
        stamp=t._selected_route_ms
        for _ in range(5):
            r=h.frame(road_accepted=False,raw_side=te.SIDE_RIGHT)
            self.assertFalse(r['target_valid'])
            self.assertEqual(r['exit_admission_reason'],'EXIT_WAIT_CURRENT_EDGE')
            self.assertFalse(t.commit_transmitted_route(h.now,False,True))
        self.assertEqual(stamp,t._selected_route_ms)
        r=self.left(h,-4)
        self.assertTrue(r['target_valid']);self.assertEqual(r['target_side'],te.SIDE_LEFT)

    def test_expired_exit_cannot_restart_or_select_inner(self):
        h=self.fixture();self.depart(h);t=h.tracker
        self.left(h);t.commit_transmitted_route(h.now,True,True);stamp=t._selected_route_ms
        self.assertFalse(self.left(h,advance_ms=3001)['target_valid'])
        self.assertEqual(stamp,t._selected_route_ms)
        self.assertFalse(h.frame(road_accepted=False,raw_side=te.SIDE_RIGHT)['target_valid'])

    def test_single_continues_then_stable_ordinary_releases(self):
        h=self.fixture();self.depart(h);t=h.tracker
        for _ in range(12):
            self.assertTrue(self.left(h)['target_valid'])
            t.commit_transmitted_route(h.now,True,True)
        self.assertEqual(t._state,te.STATE_REACQUIRE)
        self.assertTrue(t._exit_guidance_latched)
        for _ in range(12):
            r=h.frame()
            if r['target_valid']:t.commit_transmitted_route(h.now,True,True)
        self.assertEqual(t._state,te.STATE_OBSERVE)
        self.assertFalse(t._exit_guidance_latched)
        self.assertFalse(t._exit_search_armed)
        self.assertFalse(t._ring_cache_frozen)

    def test_session_reset_clears_ownership(self):
        h=self.fixture();self.depart(h);t=h.tracker
        self.left(h);t.commit_transmitted_route(h.now,True,True)
        h.frame(active=False)
        self.assertFalse(t._exit_guidance_latched)
        self.assertFalse(t._exit_search_armed)

    def uart_packet(self, shift):
        import config
        import test_fast_main_ring_slope as wire
        from unittest.mock import patch
        h=self.fixture();self.depart(h);t=h.tracker
        result=self.left(h,shift)
        self.assertEqual(result['exit_admission_reason'],'EXIT_CURRENT_SINGLE')
        t._profile_id=3;t._session_token=1
        result.update(profile_id=3,profile_seq=11,session_token=1)
        class Actual(wire._ActiveRingElements):
            def __init__(self,*a,**kw):
                super().__init__(*a,**kw);self.result=result;self.require_calibration=True
            def outer_route_valid_for(self,r):return t.outer_route_valid_for(r)
        original=wire._SingleEdgeTracker.__init__
        def init(instance,*a,**kw):
            original(instance,*a,**kw)
            instance._row_status[:]=t._current_status
            instance._row_edge2[:]=t._current_edge2
            instance._frame_scan_done=True
        with patch.object(wire._SingleEdgeTracker,'__init__',init):
            packet,log,_=wire.FastMainRingSlopeWireTests()._run_profile_element_case(Actual,config.VEHICLE_PROFILE_C)
        self.assertTrue(packet[6]&1,log)
        self.assertEqual(packet[11],3)
        error=int.from_bytes(packet[8:10],'little',signed=True)
        self.assertGreater(error*shift,0)
        return bytes(packet)

    def test_inside_exit_reaches_actual_uart_both_error_signs(self):
        for shift in (-8,7):self.uart_packet(shift)

    def test_stable_bilateral_can_restore_after_gap_without_inner_retake(self):
        h=self.fixture();self.depart(h);t=h.tracker
        self.left(h);t.commit_transmitted_route(h.now,True,True)
        self.assertFalse(self.left(h,advance_ms=3001)['target_valid'])
        for i in range(t.exit_repair_frames):
            r=h.frame()
            self.assertEqual(r['target_valid'],i+1==t.exit_repair_frames)
        self.assertEqual(r['target_side'],te.SIDE_LEFT)
        self.assertEqual(r['exit_admission_reason'],'EXIT_CURRENT_PAIRED')
        self.assertTrue(t.outer_route_valid_for(r))
