"""Ring observation retains paired edges; ordinary fallback still excludes them."""
import unittest
from unittest.mock import patch
from test_fast_single_edge import _mask, _warm_tracker
import fast_single_edge as se
import config


class PairedRingObservationTests(unittest.TestCase):
    def curved_mask(self, side=0, break_y=None, jump=False):
        m=_mask('left')
        for y in range(8,60):
            boundary=27+max(0,54-y)
            if jump and y<50:boundary+=12
            for x in range(80):
                white=x>=boundary if side==0 else x<=79-boundary
                i=y*80+x
                m['road_mask'][i]=1 if white else 0
                m['edge_mask'][i]=0 if white else 1
                m['luma'][i]=220 if white else 25
            m['row_counts'][y]=sum(m['edge_mask'][y*80:(y+1)*80])
        if break_y is not None:
            m['known_region'][break_y*80:(break_y+1)*80]=bytearray(80)
        return m

    def test_current_near_chain_can_extend_past_straight_anchor_on_both_sides(self):
        for side in (0,1):
            t=self.tracker();m=self.curved_mask(side)
            t.observe_ring_boundaries(m,True)
            self.assertEqual(t._row_status[side*60+34],1)
            self.assertEqual(t._row_edge2[side*60+34],93 if side==0 else 65)
            t=self.tracker();t.detect(m,True)
            self.assertNotEqual(t._row_status[side*60+34],1)

    def test_chain_extension_does_not_bridge_unknown_rows_or_a_large_jump(self):
        for side in (0,1):
            for kw in ({'break_y':40},{'jump':True}):
                t=self.tracker();t.observe_ring_boundaries(self.curved_mask(side,**kw),True)
                self.assertNotEqual(t._row_status[side*60+34],1)

    def tracker(self):
        t = se.FastSingleEdgeTracker(config)
        _warm_tracker(t)
        return t

    def test_paired_edges_are_retained_only_in_ring_observation(self):
        t = self.tracker()
        mask = _mask('both')
        ordinary = t.detect(mask, True)
        self.assertFalse(ordinary['valid'])
        self.assertEqual(sum(s == 1 for s in t._row_status), 0)
        t.observe_ring_boundaries(mask, True)
        self.assertTrue(t._frame_scan_done)
        for base in (0, 60):
            self.assertEqual(sum(s == 1 for s in t._row_status[base+8:base+60]), 52)
        self.assertEqual(t._ring_observation_mode, 0)
        again = t.detect(mask, True)
        self.assertFalse(again['valid'])
        self.assertEqual(sum(s == 1 for s in t._row_status), 0)

    def test_mode_is_released_when_detection_raises(self):
        t = self.tracker()
        with patch.object(t, 'detect', side_effect=RuntimeError('probe')):
            with self.assertRaises(RuntimeError):
                t.observe_ring_boundaries(_mask('both'), True)
        self.assertEqual(t._ring_observation_mode, 0)

    def test_stale_control_cannot_make_fresh_ring_rows(self):
        t = self.tracker()
        t.observe_ring_boundaries(_mask('both'), False)
        self.assertFalse(t._frame_scan_done)
        self.assertEqual(t._ring_observation_mode, 0)

    def test_unknown_or_dark_corridor_still_rejects_edges(self):
        for key in ('known_region', 'road_mask'):
            t = self.tracker()
            mask = _mask('both')
            mask[key][:] = bytearray(len(mask[key]))
            t.observe_ring_boundaries(mask, True)
            self.assertEqual(sum(s == 1 for s in t._row_status), 0)
            self.assertEqual(t._ring_observation_mode, 0)

    def test_missing_ring_observation_api_rejects_a_mixed_bundle(self):
        import test_fast_main_ring_slope
        import main
        with patch.object(se, 'RING_OBSERVATION_API_VERSION', 0):
            with self.assertRaisesRegex(ImportError, 'ring observation API'):
                main._load_singleedge_runtime()


if __name__ == '__main__':
    unittest.main()
