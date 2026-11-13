"""Exit evidence follows the transmitted lane, and aborts retain ring ownership."""
import unittest
import test_ring_exit_guidance as guidance
import track_elements as te


class CurrentExitAnchorTests(unittest.TestCase):
    def fixture(self, side):
        h = guidance.ExitGuidanceTests().tracker(side, te.STATE_INSIDE)
        h.tracker._exit_side = te.SIDE_NONE
        h.tracker._exit_drop_count = 0
        shift = -8 if side == te.SIDE_LEFT else 8
        args = dict(left=25+shift, right=55+shift, road_accepted=False,
                    raw_side=side, mask_info=h.mask(
                        left_by_y=lambda y: 25+shift,
                        right_by_y=lambda y: 55+shift))
        # Commit measured ring guidance, rather than fabricating a new anchor.
        r = h.frame(**args)
        self.assertTrue(r['target_valid'])
        self.assertTrue(h.tracker.commit_transmitted_route(h.now, True, True))
        h.tracker._exit_drop_count = 0
        return h, args

    def test_shifted_inner_edge_confirms_exit_against_actual_wire_route(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h, args = self.fixture(side)
            for i in range(h.tracker.exit_drop_frames):
                r = h.frame(**args)
                self.assertEqual(r['state_id'], te.STATE_EXIT_CONFIRM if
                                 i+1 == h.tracker.exit_drop_frames else te.STATE_INSIDE)
                self.assertTrue(r['target_valid'])
                self.assertEqual(r['target_side'], side)
                self.assertTrue(h.tracker.commit_transmitted_route(h.now, True, True))
            self.assertGreaterEqual(r['ring_near_anchor_rows'], h.tracker.min_near_support)
            # The geometric offset keeps the mirrored centre in the white lane.
            self.assertEqual(r['target_center2'], 64 if side == 0 else 96)

    def test_stale_missing_or_jumping_wire_reference_cannot_confirm(self):
        for kind in ('stale', 'missing', 'jump'):
            h, args = self.fixture(te.SIDE_RIGHT)
            t = h.tracker
            if kind == 'stale': t._wire_route_ms = h.now-2001
            if kind == 'missing': t._wire_route_ms = None
            if kind == 'jump':
                for y in range(t.height):
                    if t._wire_centers2[y] >= 0: t._wire_centers2[y] -= 30
            for _ in range(t.exit_drop_frames+2): r = h.frame(**args)
            self.assertEqual(r['state_id'], te.STATE_INSIDE, kind)
            self.assertEqual(t._exit_drop_count, 0, kind)

    def test_topology_or_row_evidence_interruption_restarts_dropout_streak(self):
        for kind in ('topology', 'rows', 'frame'):
            h, args = self.fixture(te.SIDE_RIGHT)
            h.frame(**args)
            self.assertEqual(h.tracker._exit_drop_count, 1)
            interrupted = dict(args)
            if kind == 'topology':
                interrupted['topology'] = dict(complete=False, reason='PARTIAL_KNOWN_ROI')
            elif kind == 'rows':
                interrupted['raw_side'] = None
                interrupted['single_fresh'] = False
            if kind == 'frame':
                t = h.tracker
                t.update(h.now+1, 2, True, t._session_token, False,
                         False, None, 0, 0, 0, {}, False,
                         None, None, None, None, None, None, None, None)
            else:
                h.frame(**interrupted)
            self.assertEqual(h.tracker._exit_drop_count, 0)
            r = h.frame(**args)
            self.assertEqual(r['state_id'], te.STATE_INSIDE)
            self.assertEqual(h.tracker._exit_drop_count, 1)

    def test_ambiguous_outer_rows_are_not_exit_dropout(self):
        from array import array
        h, args = self.fixture(te.SIDE_RIGHT)
        h.frame(**args)
        status = bytearray(h.tracker._current_status)
        edges = array('i', h.tracker._current_edge2)
        for y in range(h.tracker.floor_y, h.tracker.height):
            if status[y] == 0:
                status[y] = 2
        args.update(raw_status=status, raw_edge2=edges)
        args.pop('raw_side')
        for _ in range(h.tracker.exit_drop_frames+1):
            r = h.frame(**args)
        self.assertEqual(r['state_id'], te.STATE_INSIDE)
        self.assertEqual(h.tracker._exit_drop_count, 0)

    def test_inside_timeout_cannot_release_to_ordinary_single_edge(self):
        for side in (0, 1):
            h, args = self.fixture(side)
            t = h.tracker
            t._state_start_ms = h.now-t.max_active_ms-1
            frozen = list(t._ring_width)
            r = h.frame(**args)
            self.assertEqual(r['state_id'], te.STATE_INSIDE)
            self.assertEqual(r['reason'], 'STATE_TIMEOUT')
            self.assertTrue(r['ring_confirmed'])
            self.assertFalse(r['control_eligible'])
            self.assertFalse(r['target_valid'])
            self.assertTrue(t._ring_cache_frozen)
            self.assertEqual(list(t._ring_width), frozen)
            for _ in range(4):
                r = h.frame()
                self.assertFalse(t.observe_route_outcome(h.now, False, True))
            self.assertTrue(r['ring_confirmed'])
            self.assertEqual(r['side'], side)

    def test_invalid_frame_cannot_release_confirmed_ring(self):
        h, args = self.fixture(1)
        t = h.tracker
        r = t.update(h.now+30, 2, True, t._session_token, False,
                     False, None, 0, 0, 0, {}, False,
                     None, None, None, None, None, None, None, None)
        self.assertEqual(r['state_id'], te.STATE_INSIDE)
        self.assertTrue(r['ring_confirmed'])
        self.assertFalse(r['target_valid'])
        self.assertTrue(t._ring_cache_frozen)


if __name__ == '__main__':
    unittest.main()
