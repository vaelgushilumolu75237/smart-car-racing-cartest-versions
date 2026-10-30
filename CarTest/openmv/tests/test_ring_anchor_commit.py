"""Temporal ring anchors follow only accepted, actually measured route rows."""
from array import array
import unittest
from unittest.mock import patch
from test_fast_single_edge import _mask, _warm_tracker
import fast_single_edge as se
import config


def curved(side, shift):
    m = _mask('left')
    for y in range(8, 60):
        boundary = 27 + shift + 59 - y
        for x in range(80):
            white = x >= boundary if side == 0 else x <= 79-boundary
            i = y*80+x
            m['road_mask'][i] = int(white)
            m['edge_mask'][i] = int(not white)
            m['luma'][i] = 220 if white else 25
        m['row_counts'][y] = sum(m['edge_mask'][y*80:(y+1)*80])
    return m


def route(t, side):
    rows = array('i')
    for y in range(8, 60):
        slot = side*60+y
        if t._row_status[slot] == 1:
            rows.extend((y, t._row_center2[slot], t._row_edge2[slot],
                         side, 1 << side))
    return dict(valid=True, selected_row_ints=rows,
                selected_row_count=len(rows)//5, selected_row_stride=5)


class RingAnchorCommitTests(unittest.TestCase):
    def tracker(self):
        t = se.FastSingleEdgeTracker(config)
        _warm_tracker(t)
        return t

    def test_uncommitted_four_to_three_seed_cliff_reproduces_both_sides(self):
        for side in (0, 1):
            t = self.tracker()
            t.observe_ring_boundaries(curved(side, 13), True)
            self.assertGreater(route(t, side)['selected_row_count'], 20)
            t.observe_ring_boundaries(curved(side, 14), True)
            self.assertEqual(route(t, side)['selected_row_count'], 3)

    def test_sent_observation_tracks_curve_without_changing_frozen_width(self):
        for side in (0, 1):
            t = self.tracker()
            frozen = (list(t._cache_width), list(t._cache_left2),
                      list(t._cache_right2))
            opposite = list(t._anchor_right2 if side == 0 else t._anchor_left2)
            for shift in range(13, 18):
                t.observe_ring_boundaries(curved(side, shift), True)
                r = route(t, side)
                self.assertGreater(r['selected_row_count'], 20)
                before = list(t._anchor_left2 if side == 0 else t._anchor_right2)
                self.assertEqual(t.commit_ring_observation(r), r['selected_row_count'])
                after = list(t._anchor_left2 if side == 0 else t._anchor_right2)
                self.assertTrue(all(abs(a-b) <= 2 for a,b in zip(after,before)))
            self.assertEqual(frozen, (list(t._cache_width), list(t._cache_left2),
                                      list(t._cache_right2)))
            self.assertEqual(opposite, list(t._anchor_right2 if side == 0 else t._anchor_left2))

    def test_invalid_stale_short_forged_or_malformed_route_cannot_move_anchors(self):
        for kind in ('invalid', 'stale', 'short', 'edge', 'side', 'duplicate', 'stride', 'narrow', 'float'):
            t = self.tracker()
            t.observe_ring_boundaries(curved(0, 13), True)
            r = route(t, 0)
            before = list(t._anchor_left2)
            if kind == 'invalid': r['valid'] = False
            if kind == 'stale': t._frame_scan_done = False
            if kind == 'short': r['selected_row_count'] = 3
            if kind == 'edge': r['selected_row_ints'][-3] += 2
            if kind == 'side': r['selected_row_ints'][-2] = 1
            if kind == 'duplicate': r['selected_row_ints'][-5] = r['selected_row_ints'][-10]
            if kind == 'stride': r['selected_row_stride'] = 4
            if kind == 'narrow': r['selected_row_ints'] = array('h', r['selected_row_ints'])
            if kind == 'float': r['selected_row_ints'] = array('f', r['selected_row_ints'])
            self.assertEqual(t.commit_ring_observation(r), 0, kind)
            self.assertEqual(before, list(t._anchor_left2), kind)

    def test_unknown_row_still_breaks_extension_and_ordinary_scan_stays_bounded(self):
        for side in (0, 1):
            t = self.tracker()
            m = curved(side, 13)
            m['known_region'][54*80:55*80] = bytearray(80)
            t.observe_ring_boundaries(m, True)
            self.assertNotEqual(t._row_status[side*60+53], 1)
            t = self.tracker()
            t.detect(curved(side, 13), True)
            self.assertEqual(route(t, side)['selected_row_count'], 4)

    def test_inferred_bridge_does_not_become_an_anchor(self):
        t = self.tracker()
        t.observe_ring_boundaries(curved(0, 13), True)
        r = route(t, 0)
        y = r['selected_row_ints'][0]
        anchor = t._anchor_left2[y]
        r['selected_row_ints'][3] = -1
        r['selected_row_ints'][4] = se.FLAG_INFERRED_LEFT | se.FLAG_INFERRED_RIGHT
        self.assertEqual(t.commit_ring_observation(r), r['selected_row_count']-1)
        self.assertEqual(t._anchor_left2[y], anchor)

    def test_actual_main_commits_only_full_valid_ring_uart_writes(self):
        import test_fast_main_ring_slope as fixture
        runner = fixture.FastMainRingSlopeWireTests()
        original_write = fixture._UART.write
        for mode in ('full', 'short', 'error', 'invalid'):
            commits = []
            def commit(_self, candidate):
                commits.append(candidate['selected_row_count'])
                return 0
            def write(uart, frame):
                result = original_write(uart, frame)
                if mode == 'short': return len(frame)-1
                if mode == 'error': raise OSError('test UART failure')
                return result
            elements = (fixture._ConfirmedRingNoTarget if mode == 'invalid'
                        else fixture._ActiveRingElements)
            with patch.object(fixture._SingleEdgeTracker, 'commit_ring_observation', commit), \
                    patch.object(fixture._UART, 'write', write):
                runner._run_profile_element_case(elements, config.VEHICLE_PROFILE_B)
            self.assertEqual(commits, [52] if mode == 'full' else [], mode)


if __name__ == '__main__':
    unittest.main()
