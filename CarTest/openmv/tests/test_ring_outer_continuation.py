"""A selected inner route can continue on a verified, continuous outer edge."""
from array import array
import types
import unittest
from unittest.mock import patch
import test_ring_exit_pending_route as pending
import track_elements as te


class OuterContinuationTests(unittest.TestCase):
    def frame(self, h, side, observed, *, shift=0, advance_ms=30, blocked=False):
        status = bytearray(120)
        edges = array('i', [0]*120)
        # Mirror both the physical ring direction and its observed boundary.
        left, right = ((30, 60) if side == te.SIDE_RIGHT else (19, 49))
        left += shift
        right += shift
        for y in range(8, 60):
            status[observed*60+y] = 1
            edges[observed*60+y] = left*2-1 if observed == 0 else right*2+1
        mask = h.mask(left_by_y=lambda y: 0, right_by_y=lambda y: 79)
        if blocked:
            for y in range(48,60):
                mask['known_region'][y*80+40] = 0
        h.now += advance_ms
        result = h.tracker.update(h.now, 2, True, 'session-1', True, False,
            None, 0, 4, 0, {'complete': True, 'reason': 'OK'}, True,
            status, edges, array('i', [20]*120), h.cache_valid, h.cache_width,
            h.cache_left2, h.cache_right2, mask, h.now & 65535, 0)
        return result, types.SimpleNamespace(_row_status=status, _row_edge2=edges)

    def setup_inner(self, side, selected=True):
        h = pending.PendingExitRouteTests().tracker(side)
        h.tracker._state = te.STATE_INSIDE
        result, edges = self.frame(h, side, side)
        self.assertTrue(result['target_valid'])
        if selected:
            h.tracker.commit_transmitted_route(h.now, True, True)
        return h

    def outside_outer(self, side):
        h=pending.PendingExitRouteTests().tracker(side)
        h.tracker._state=te.STATE_INSIDE
        direction=1 if side==te.SIDE_RIGHT else -1
        result,_=self.frame(h,side,side,shift=18*direction)
        self.assertTrue(result['target_valid'])
        h.tracker.commit_transmitted_route(h.now,True,True)
        result,edges=self.frame(h,side,1-side,shift=22*direction)
        return h,result,edges

    def test_virtual_opposite_edge_keeps_the_geometric_center_on_both_sides(self):
        for side in (0,1):
            h,r,_=self.outside_outer(side)
            self.assertTrue(r['target_valid'])
            self.assertTrue(r['outer_tracking'])
            self.assertTrue(h.tracker.outer_route_valid_for(r))
            rows=r['target_rows']
            for i in range(r['target_count']):
                y,left,right,flags=rows[i*4:i*4+4]
                self.assertTrue(flags&te.TARGET_OUTSIDE_VIEW)
                self.assertTrue(left<0 if side==0 else right>=80)
                self.assertEqual(left+right,24 if side==0 else 134)

    def test_virtual_rows_do_not_hide_unknown_visible_cells_or_offscreen_center(self):
        for side in (0,1):
            h,r,_=self.outside_outer(side)
            original=h.mask
            def mask(*a,**kw):
                m=original(*a,**kw)
                for y in range(48,60):
                    m['known_region'][y*80+(0 if side==0 else 79)]=0
                return m
            h.mask=mask
            r,_=self.frame(h,side,1-side,shift=(-22 if side==0 else 22))
            self.assertFalse(r['target_valid'])
            h,r,_=self.outside_outer(side)
            r,_=self.frame(h,side,1-side,shift=(-60 if side==0 else 60))
            self.assertFalse(r['target_valid'])

    def test_main_rejects_unflagged_virtual_rows_and_changed_observed_identity(self):
        import test_fast_main_ring_slope
        import main,config
        main._load_fast_runtime();main._load_singleedge_runtime();main._load_track_elements_runtime()
        for side in (0,1):
            h,r,edges=self.outside_outer(side)
            for i in range(r['target_count']):
                r['target_rows'][i*4+3]&=~te.TARGET_OUTSIDE_VIEW
            result={}
            self.assertEqual(main._fill_ring_control_route(r,h.tracker,edges,
                False,True,result,array('i',[0]*300),config),0)
            self.assertEqual(result['reason'],'RING_OUTSIDE_VIEW_UNVERIFIED')
            h,r,edges=self.outside_outer(side)
            # Separate raw arrays, so a same-object certificate alone cannot
            # make different observed edge coordinates pass main admission.
            edges=types.SimpleNamespace(_row_status=bytearray(edges._row_status),
                _row_edge2=array('i',edges._row_edge2))
            for y in range(60):edges._row_edge2[(1-side)*60+y]+=10
            self.assertEqual(main._fill_ring_control_route(r,h.tracker,edges,
                False,True,{},array('i',[0]*300),config),0)

    def test_selected_inner_can_continue_on_either_outer_boundary(self):
        for side in (0, 1):
            h = self.setup_inner(side)
            result, edges = self.frame(h, side, 1-side)
            self.assertTrue(result['target_valid'], result)
            self.assertTrue(result['outer_tracking'])
            self.assertEqual(result['side'], side)
            self.assertEqual(result['target_side'], 1-side)
            self.assertTrue(h.tracker.outer_route_valid_for(result))
            self.assertFalse(h.tracker.outer_route_valid_for(result.copy()))

    def test_outer_route_requires_a_previously_admitted_inner_route(self):
        for side in (0, 1):
            h = self.setup_inner(side, selected=False)
            result, _ = self.frame(h, side, 1-side)
            self.assertFalse(result['target_valid'])
            self.assertFalse(result['outer_tracking'])

    def test_expired_jump_or_unknown_corridor_cannot_handoff(self):
        for side in (0, 1):
            for kwargs in ({'advance_ms': 2001}, {'shift': 8}, {'blocked': True}):
                h = self.setup_inner(side)
                result, _ = self.frame(h, side, 1-side, **kwargs)
                self.assertFalse(result['target_valid'], (side, kwargs, result))
                self.assertFalse(result['outer_tracking'])

    def test_selected_outer_refreshes_history_and_inner_can_return(self):
        for side in (0, 1):
            h = self.setup_inner(side)
            for _ in range(3):
                result, _ = self.frame(h, side, 1-side, advance_ms=600)
                self.assertTrue(result['outer_tracking'])
                h.tracker.commit_transmitted_route(h.now, True, True)
            result, _ = self.frame(h, side, side)
            self.assertTrue(result['target_valid'])
            self.assertFalse(result['outer_tracking'])
            h.tracker.reset('new session')
            self.assertFalse(h.tracker._inner_route_established)

    def test_return_to_inner_cannot_make_an_unbounded_target_jump(self):
        for side in (0, 1):
            h = self.setup_inner(side)
            result, _ = self.frame(h, side, 1-side)
            h.tracker.commit_transmitted_route(h.now, True, True)
            result, _ = self.frame(h, side, side, shift=8)
            self.assertFalse(result['target_valid'])

    def test_expired_outer_history_does_not_block_fresh_inner_reacquisition(self):
        for side in (0, 1):
            h=self.setup_inner(side)
            result,_=self.frame(h,side,1-side)
            h.tracker.commit_transmitted_route(h.now,True,True)
            result,_=self.frame(h,side,side,advance_ms=2001)
            self.assertTrue(result['target_valid'])
            self.assertFalse(result['outer_tracking'])

    def test_qualified_outer_can_return_within_the_existing_two_second_hold(self):
        for side in (0,1):
            h=self.setup_inner(side)
            result,_=self.frame(h,side,1-side,advance_ms=1800)
            self.assertTrue(result['outer_tracking'])

    def test_a_bad_row_is_omitted_and_remaining_corridors_stay_verified(self):
        h=self.setup_inner(1)
        original=h.mask
        def mask(*a,**kw):
            value=original(*a,**kw)
            value['known_region'][45*80+55]=0
            return value
        h.mask=mask
        result,_=self.frame(h,1,0)
        self.assertTrue(result['outer_tracking'])
        rows=result['target_rows']
        self.assertNotIn(45,[rows[i*4] for i in range(result['target_count'])])

    def test_main_admission_checks_certificate_freshness_and_final_uart(self):
        import test_fast_main_ring_slope as runtime
        import main, config, vision_protocol
        main._load_fast_runtime(); main._load_singleedge_runtime(); main._load_track_elements_runtime()
        for side, outside in ((0,False),(1,False),(0,True),(1,True)):
            if outside:
                h,element,edges=self.outside_outer(side)
            else:
                h = self.setup_inner(side)
                element, edges = self.frame(h, side, 1-side)
            args=(element,h.tracker,edges,False,True,{},array('i',[0]*300),config)
            self.assertGreater(main._fill_ring_control_route(*args), 0)
            bad=list(args);bad[0]=element.copy();bad[5]={}
            self.assertEqual(main._fill_ring_control_route(*bad), 0)
            stale=list(args);stale[4]=False;stale[5]={}
            self.assertEqual(main._fill_ring_control_route(*stale), 0)
            h.tracker._profile_id=3
            h.tracker._session_token=1
            element.update(profile_id=3, session_token=1, profile_seq=11)

            class Element(runtime._ActiveRingElements):
                def __init__(self,*a,**kw):
                    super().__init__(*a,**kw)
                    self.min_target_rows=h.tracker.min_target_rows
                    self.max_cache_gap=h.tracker.max_cache_gap
                    self.require_calibration=True
                    self.near_rows=12
                    self.result=element
                    self.outer_route_valid_for=h.tracker.outer_route_valid_for

            class Edges(runtime._SingleEdgeTracker):
                def observe_ring_boundaries(self,*a):
                    self._row_status[:]=edges._row_status
                    self._row_edge2[:]=edges._row_edge2
                    self._frame_scan_done=True
                    return {'valid':False,'reason':'OBSERVATION_ONLY'}

            runner=runtime.FastMainRingSlopeWireTests(methodName=
                'test_fresh_profile_b_ring_target_reaches_the_existing_uart_chain')
            with patch.object(runtime,'_SingleEdgeTracker',Edges):
                packet,log,commits=runner._run_profile_element_case(Element,3)
            self.assertTrue(packet[6]&vision_protocol.FLAG_VALID,log)
            self.assertEqual(packet[11],3)
            error=int.from_bytes(packet[8:10],'little',signed=True)
            self.assertLess(error,0) if side==0 else self.assertGreater(error,0)
            self.assertIn('source=ring_left' if side==0 else 'source=ring_right',log)


if __name__ == '__main__':
    unittest.main()
