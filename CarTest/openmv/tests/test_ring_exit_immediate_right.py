"""r20-derived: first qualified exit left edge owns right-ring guidance."""
from array import array
import unittest
from unittest.mock import patch
import test_ring_exit_single as single
import test_fast_main_ring_slope as wire
import config
import track_elements as te

class ImmediateRightExitTests(unittest.TestCase):
    def fixture(self, state=te.STATE_EXIT_CONFIRM):
        h=single.SingleExitTests().fixture(te.SIDE_RIGHT)
        h.tracker._state=state
        return h

    def left(self,h,shift=0,**kw):
        return single.SingleExitTests().single(h,shift,**kw)

    def test_first_fresh_left_frame_admits_all_exit_states(self):
        for state in (te.STATE_EXIT_CONFIRM,te.STATE_EXIT_REPAIR,te.STATE_REACQUIRE):
            h=self.fixture(state);r=self.left(h)
            self.assertTrue(r['target_valid'])
            self.assertEqual(r['exit_single_count'],1)
            self.assertEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')
            self.assertEqual(r['target_side'],te.SIDE_LEFT)
            self.assertTrue(h.tracker.outer_route_valid_for(r))
            self.assertTrue(h.tracker._ring_cache_frozen)

    def test_current_exit_geometry_replaces_displaced_frozen_anchor(self):
        h=self.fixture();t=h.tracker
        for y in range(t.height):
            if t._wire_centers2[y]>=0:t._wire_centers2[y]-=30
        r=self.left(h)
        self.assertTrue(r['target_valid'])
        self.assertEqual(r['target_center2'],80)

    def test_no_left_or_unknown_branch_curve_and_bad_width_reject(self):
        for kind in ('missing','unknown','branch','curve','width','calibration','session','expired'):
            h=self.fixture();t=h.tracker;kw={}
            if kind=='missing':kw.update(raw_side=None,single_fresh=False)
            if kind=='unknown':
                m=h.mask();m['known_region'][:]=bytearray(4800);kw['mask_info']=m
            if kind=='branch':kw['topology']=dict(complete=True,reason='BRANCH',branch_evidence=True)
            if kind in ('curve','width'):
                status=bytearray(120);edges=array('i',[0]*120)
                for y in range(36,60):
                    status[y]=1;edges[y]=49+((y-48)**2//5 if kind=='curve' else 0)
                    if kind=='width':status[60+y]=1;edges[60+y]=75
                kw.update(raw_status=status,raw_edge2=edges,raw_side=None)
            if kind=='calibration':t._cal_check_ready=False
            if kind=='session':kw['active']=False
            if kind=='expired':t._selected_route_ms=h.now-3001
            # explicit raw arguments replace the helper's defaults
            args=dict(left=25,right=55,road_accepted=False,raw_side=0,mask_info=h.mask())
            args.update(kw);r=h.frame(**args)
            self.assertFalse(t._exit_single_ready(),kind)
            self.assertNotEqual(r.get('exit_admission_reason'),'EXIT_CURRENT_SINGLE',kind)

    def test_last_write_budget_is_three_seconds_and_not_renewed_on_failure(self):
        h=self.fixture();t=h.tracker;t._selected_route_ms=h.now-2500
        r=self.left(h);self.assertTrue(r['target_valid'])
        stamp=t._selected_route_ms
        self.assertFalse(t.commit_transmitted_route(h.now,True,False))
        self.assertEqual(t._selected_route_ms,stamp)
        r=self.left(h,advance_ms=600)
        self.assertFalse(t._exit_single_ready())

    def test_inside_and_left_ring_retain_three_frame_confirmation(self):
        h=single.SingleExitTests().fixture(0)
        r=single.SingleExitTests().single(h)
        self.assertEqual(r['exit_single_count'],1)
        self.assertFalse(h.tracker._exit_single_ready())
        h=self.fixture(te.STATE_INSIDE);r=self.left(h)
        self.assertLess(r['exit_single_count'],3)
        self.assertEqual(r['state_id'],te.STATE_INSIDE)

    def test_first_frame_route_reaches_real_uart_without_forcing_center(self):
        for shift in (-8,7):
            h=self.fixture();t=h.tracker;result=self.left(h,shift)
            self.assertTrue(result['target_valid'])
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

if __name__=='__main__':unittest.main()
