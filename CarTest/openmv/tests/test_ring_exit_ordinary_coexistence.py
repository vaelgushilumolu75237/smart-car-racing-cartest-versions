"""An accepted ordinary route must not mask an independently qualified exit.

The ordinary observation is intentionally separate from raw camera edges.
No recorded vehicle outcome is assumed by these synthetic geometry tests.
"""
from array import array
import unittest
import test_ring_exit_ownership as ownership
import track_elements as te


class ExitOrdinaryCoexistenceTests(unittest.TestCase):
    def fixture(self, armed=True):
        helper=ownership.ExitOwnershipTests()
        h=helper.fixture()
        if armed:helper.depart(h)
        return h

    def frame(self,h,ordinary=True,shift=-7,raw=True,kind=None):
        t=h.tracker
        status=bytearray(120);edges=array('i',[0]*120)
        left,right=25+shift,55+shift
        for y in range(8,60):
            status[y]=1;edges[y]=2*left-1
            if kind=='curve':edges[y]+=(y-35)**2//12
            if kind=='ambiguous':status[60+y]=2
            if kind=='width':status[60+y]=1;edges[60+y]=2*(left+10)+1
        if kind=='missing':status[:]=bytearray(120)
        rows=array('i')
        for y in range(8,60):rows.extend((y,left,right,7))
        mask=h.mask(lambda y:left,lambda y:right)
        if kind=='unknown':mask['known_region'][:]=bytearray(4800)
        topology=dict(complete=True,reason='OK')
        if kind=='branch':topology.update(reason='BRANCH',branch_evidence=True)
        h.now+=30;h.profile_seq+=1;h.odom+=10
        return t.update(h.now,2,True,'session-1',True,ordinary,
            rows,52,4,2 if ordinary else 0,topology,raw,
            status,edges,array('i',[20]*120),h.cache_valid,h.cache_width,
            h.cache_left2,h.cache_right2,mask,h.profile_seq,h.odom)

    def test_qualified_exit_is_not_vetoed_by_ordinary_detector(self):
        for state in (te.STATE_INSIDE,te.STATE_EXIT_CONFIRM,
                      te.STATE_EXIT_REPAIR,te.STATE_REACQUIRE):
            with self.subTest(state=state):
                h=self.fixture();t=h.tracker
                t._state=state
                if state!=te.STATE_INSIDE:t._exit_side=te.SIDE_LEFT
                r=self.frame(h)
                self.assertTrue(t._ordinary_snapshot_active)
                self.assertTrue(r['target_valid'])
                self.assertEqual(r['target_side'],te.SIDE_LEFT)
                self.assertEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')
                self.assertTrue(t.outer_route_valid_for(r))

    def test_same_current_geometry_has_same_target_with_or_without_ordinary(self):
        values=[]
        for ordinary in (False,True):
            h=self.fixture();r=self.frame(h,ordinary)
            self.assertTrue(r['target_valid'])
            values.append((r['target_side'],r['target_center2'],
                list(r['target_rows'][:r['target_count']*4])))
        self.assertEqual(values[0],values[1])

    def test_entry_ordinary_road_cannot_skip_departure(self):
        h=self.fixture(False)
        for _ in range(6):
            r=self.frame(h,shift=0)
            self.assertNotEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')
            self.assertFalse(h.tracker._exit_search_armed)
            self.assertEqual(r['state_id'],te.STATE_INSIDE)

    def test_ordinary_flag_does_not_substitute_for_current_raw_edges(self):
        for raw,kind in ((False,None),(True,'missing'),(True,'curve'),
                         (True,'unknown'),(True,'width'),(True,'branch'),
                         (True,'ambiguous')):
            with self.subTest(raw=raw,kind=kind):
                h=self.fixture();r=self.frame(h,raw=raw,kind=kind)
                self.assertFalse(h.tracker._exit_single_ready())
                self.assertNotEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')

    def test_no_transmission_cannot_latch_and_gap_keeps_last_sent_exit(self):
        h=self.fixture();t=h.tracker
        r=self.frame(h)
        self.assertTrue(r['target_valid'])
        self.assertFalse(t.commit_transmitted_route(h.now,True,False))
        self.assertFalse(t._exit_guidance_latched)
        self.assertTrue(t.commit_transmitted_route(h.now,True,True))
        self.assertTrue(t._exit_guidance_latched)
        stamp=t._selected_route_ms
        r=h.frame(road_accepted=False,raw_side=te.SIDE_RIGHT)
        self.assertFalse(r['target_valid'])
        self.assertFalse(t.commit_transmitted_route(h.now,False,True))
        self.assertEqual(t._selected_route_ms,stamp)

    def test_no_departure_or_wrong_side_keeps_existing_ordinary_guard(self):
        for side in (te.SIDE_LEFT,te.SIDE_RIGHT):
            h=self.fixture(False);t=h.tracker
            t._side=side;t._state=te.STATE_EXIT_CONFIRM;t._exit_side=1-side
            self.frame(h,shift=0)
            self.assertFalse(t._exit_single_ready())

    def uart_packet(self,shift):
        from unittest.mock import patch
        import config
        import test_fast_main_ring_slope as wire
        h=self.fixture();t=h.tracker;result=self.frame(h,shift=shift)
        self.assertTrue(result['target_valid'])
        self.assertTrue(result['exit_ordinary_coexistence'])
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
            packet,log,_=wire.FastMainRingSlopeWireTests()._run_profile_element_case(Actual,3)
        self.assertTrue(packet[6]&1,log);self.assertEqual(packet[11],3)
        error=int.from_bytes(packet[8:10],'little',signed=True)
        self.assertGreater(error*shift,0)
        return bytes(packet)

    def test_ordinary_coexistence_reaches_real_uart_both_error_signs(self):
        for shift in (-7,7):self.uart_packet(shift)

    def test_coexistence_cannot_bypass_calibration_history_or_deadlines(self):
        for kind in ('calibration','no_inner','expired','state_timeout','entry_lease'):
            with self.subTest(kind=kind):
                h=self.fixture();t=h.tracker
                if kind=='calibration':t._cal_check_ready=False
                if kind=='no_inner':t._inner_route_established=False
                if kind=='expired':t._selected_route_ms=h.now-3001
                if kind=='state_timeout':t._state_start_ms=h.now-t.max_active_ms-1
                if kind=='entry_lease':t._start_entry_transition(h.now)
                r=self.frame(h)
                self.assertFalse(t._exit_single_ready())
                self.assertNotEqual(r['exit_admission_reason'],'EXIT_CURRENT_SINGLE')

    def test_stable_bilateral_recovery_releases_new_exit_ownership(self):
        h=self.fixture();t=h.tracker
        self.frame(h);self.assertTrue(t.commit_transmitted_route(h.now,True,True))
        for _ in range(15):
            r=self.frame(h)
            if r['target_valid']:t.commit_transmitted_route(h.now,True,True)
        # The exit is still laterally displaced; an accepted ordinary result
        # alone must not release guidance before the vehicle aligns.
        self.assertEqual(t._state,te.STATE_REACQUIRE)
        self.assertTrue(t._exit_guidance_latched)
        for _ in range(12):
            r=h.frame()
            if r['target_valid']:t.commit_transmitted_route(h.now,True,True)
        self.assertEqual(t._state,te.STATE_OBSERVE)
        self.assertFalse(t._exit_guidance_latched)
        self.assertFalse(t._ring_cache_frozen)

    def test_mixed_r23_config_or_tracker_is_rejected(self):
        from unittest.mock import patch
        import test_fast_main_ring_slope
        import main,config
        main._load_fast_runtime();main._load_track_elements_runtime()
        with patch.object(te,'EXIT_IMMEDIATE_RIGHT_API_VERSION',2):
            with self.assertRaisesRegex(RuntimeError,'v26-r24'):
                main._validate_fast_config()
        with patch.object(config,'TRACK_PACKAGE_BUILD_ID','TRACK_RING_EXIT_V26_R23_20261101'):
            with self.assertRaisesRegex(ValueError,'identity'):
                main._validate_fast_config()


if __name__=='__main__':unittest.main()
