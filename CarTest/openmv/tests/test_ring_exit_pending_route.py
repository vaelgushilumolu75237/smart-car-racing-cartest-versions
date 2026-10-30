"""An unconfirmed exit must not discard a freshly verified inner route."""
from array import array
import unittest
from test_track_elements import TrackElementsHarness
import track_elements as te


class PendingExitRouteTests(unittest.TestCase):
    def tracker(self, side):
        h = TrackElementsHarness()
        class Calibration:
            ready = True
            reason = 'CALIBRATION_READY'
            width_valid = bytearray([0]*8+[1]*52)
            width_cells = array('i', [0]*8+[31]*52)
        h.tracker = te.TrackElementsTracker({}, Calibration(), True)
        rows = array('i')
        for y in range(8,60): rows.extend((y,25,55,7))
        for _ in range(3): h.tracker.observe_ring_calibration(rows,52,4)
        h.prime()
        self.assertTrue(h.tracker._freeze_ring_width())
        h.tracker._side = side
        h.tracker._state = te.STATE_EXIT_CONFIRM
        return h

    def test_pending_exit_retains_fresh_inner_route_on_both_sides(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.tracker(side)
            result = h.frame(road_accepted=False, raw_side=side)
            self.assertEqual(result['state_id'], te.STATE_EXIT_CONFIRM)
            self.assertTrue(result['target_valid'])
            self.assertEqual(result['target_side'], side)
            self.assertFalse(result['entry_transition'])

    def test_pending_exit_still_rejects_missing_or_blocked_inner_evidence(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = self.tracker(side)
            result = h.frame(road_accepted=False, raw_status=bytearray(120),
                             raw_edge2=array('i',[0]*120))
            self.assertFalse(result['target_valid'])
            h = self.tracker(side)
            mask = h.mask()
            x = 28 if side == te.SIDE_LEFT else 52
            mask['road_mask'][40*80+x] = 0
            result = h.frame(road_accepted=False, raw_side=side, mask_info=mask)
            self.assertFalse(result['target_valid'])


if __name__ == '__main__':
    unittest.main()
