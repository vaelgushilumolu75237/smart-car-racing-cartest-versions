"""Confirmed ring boundaries and ordinary counter-evidence stay independent."""
from array import array
import unittest
from unittest.mock import patch
from test_track_elements import TrackElementsHarness
import test_ring_exit_pending_route as pending
import track_elements as te


class FreshRingObservationTests(unittest.TestCase):
    def setup_ring(self, side):
        h = pending.PendingExitRouteTests().tracker(side)
        h.tracker._state = te.STATE_INSIDE
        return h

    def update(self, h, side, *, ordinary_near=False, raw_both=False,
               fresh=True, malformed=False, raw_empty=False):
        rows = array('i')
        for y in range(8, 60 if ordinary_near else 48):
            rows.extend((y, 25, 55, 7))
        status = bytearray(120)
        edges = array('i', [0]*120)
        if not raw_empty:
            for y in range(8, 60):
                for observed in ((0, 1) if raw_both else (side,)):
                    status[observed*60+y] = 1
                    edges[observed*60+y] = 49 if observed == 0 else 111
        if malformed:
            status[50] = 3
        h.now += 30
        return h.tracker.update(h.now, 2, True, 'session-1', True, True,
            rows, len(rows)//4, 4, 2, {'complete': True, 'reason': 'OK'},
            fresh, status, edges, array('i', [20]*120), h.cache_valid,
            h.cache_width, h.cache_left2, h.cache_right2, h.mask(),
            profile_seq=h.now & 65535, mean_forward_wheel_millirev=0)

    def test_near_raw_edge_survives_accepted_far_only_ordinary_route(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.setup_ring(side)
            result = self.update(h, side)
            self.assertTrue(result['target_valid'], result)
            self.assertEqual(result['target_side'], side)
            self.assertTrue(h.tracker._ordinary_snapshot_active)
            self.assertEqual(h.tracker._ordinary_side_summary[0], 0)
            self.assertEqual(h.tracker._ordinary_side_summary[1], 0)
            self.assertFalse(h.tracker._ordinary_reacquired_frame())
            self.assertIn('FRESH_RAW', result['ring_observation_source'])

    def test_raw_bilateral_rows_cannot_fill_missing_ordinary_counterevidence(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.setup_ring(side)
            for _ in range(3):
                self.update(h, side, raw_both=True)
                self.assertFalse(h.tracker._ordinary_reacquired_frame())
                self.assertFalse(h.tracker.observe_route_outcome(
                    h.now, False, True))
            self.assertNotEqual(h.tracker._state, te.STATE_OBSERVE)

    def test_qualified_ordinary_route_survives_raw_ring_rejection(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.setup_ring(side)
            for i in range(3):
                result = self.update(h, side, ordinary_near=True, raw_empty=True)
                self.assertFalse(result['target_valid'])
                self.assertTrue(h.tracker._ordinary_reacquired_frame())
                recovered = h.tracker.observe_route_outcome(h.now, False, True)
                self.assertEqual(recovered, i == 2)
            self.assertEqual(h.tracker._state, te.STATE_REACQUIRE)
            self.assertEqual(h.tracker._reason, 'ORDINARY_ROUTE_REACQUIRED')

    def test_stale_or_malformed_raw_rows_do_not_create_a_ring_route(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            for kwargs in ({'fresh': False}, {'malformed': True}):
                h = self.setup_ring(side)
                result = self.update(h, side, **kwargs)
                self.assertFalse(result['target_valid'], (side, kwargs, result))

    def test_pending_exit_also_uses_current_raw_inner_boundary(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.setup_ring(side)
            self.update(h, side)
            self.assertTrue(h.tracker.commit_transmitted_route(h.now, True, True))
            h.tracker._state = te.STATE_EXIT_CONFIRM
            result = self.update(h, side)
            self.assertEqual(result['state_id'], te.STATE_EXIT_CONFIRM)
            self.assertTrue(result['target_valid'])

    def test_real_main_scans_accepted_road_and_emits_mirrored_ring_packets(self):
        import test_fast_main_ring_slope as runtime
        from test_main_entry_transition import transition_vector
        import vision_protocol
        sent = []
        for side in (0, 1):
            tracker, element, observed = transition_vector(side)
            calls = []

            class Entry(runtime._ActiveRingElements):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    for key, value in vars(tracker).items():
                        setattr(self, key, value)
                    self.rows = element['target_rows']
                    self.result = element

                def update(self, *args, **kwargs):
                    self_test.assertTrue(args[5])  # ordinary candidate retained
                    self_test.assertTrue(args[11])  # raw scan is fresh too
                    return self.result

            class ObservedEdges(runtime._SingleEdgeTracker):
                def detect(self, *args, **kwargs):
                    calls.append(True)
                    self._row_status[:] = observed._row_status
                    self._row_edge2[:] = observed._row_edge2
                    self._frame_scan_done = True
                    return {'valid': False, 'reason': 'OBSERVATION_ONLY'}

            self_test = self
            runner = runtime.FastMainRingSlopeWireTests(methodName=
                'test_fresh_profile_b_ring_target_reaches_the_existing_uart_chain')
            with patch.object(runtime, '_SingleEdgeTracker', ObservedEdges):
                packet, log, commits = runner._run_profile_element_case(Entry, 3)
            self.assertEqual(calls, [True])
            self.assertEqual(commits, [])
            self.assertTrue(packet[6] & vision_protocol.FLAG_VALID, log)
            self.assertEqual(packet[11], 3)
            error = int.from_bytes(packet[8:10], 'little', signed=True)
            self.assertLess(error, 0) if side == 0 else self.assertGreater(error, 0)
            sent.append(error)
        self.assertLessEqual(abs(sum(sent)), 1)


if __name__ == '__main__':
    unittest.main()
