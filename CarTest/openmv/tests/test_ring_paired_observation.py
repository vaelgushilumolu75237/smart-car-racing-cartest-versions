"""Ring observation retains paired edges; ordinary fallback still excludes them."""
import unittest
from unittest.mock import patch
from test_fast_single_edge import _mask, _warm_tracker
import fast_single_edge as se
import config


class PairedRingObservationTests(unittest.TestCase):
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
