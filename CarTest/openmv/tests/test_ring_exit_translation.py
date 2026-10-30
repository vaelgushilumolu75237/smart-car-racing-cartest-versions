"""Exit reappearance permits shared translation, never one-edge drift."""
import unittest
from test_track_elements import TrackElementsHarness
import track_elements as te


class ExitTranslationTests(unittest.TestCase):
    def pending(self, side):
        h = TrackElementsHarness()
        if side == te.SIDE_LEFT:
            h.complete_left_ring()
        else:
            h.complete_right_ring()
        h.lose(1 - side)
        self.assertEqual(h.tracker._state, te.STATE_EXIT_CONFIRM)
        return h

    def test_mirrored_common_shift_reacquires_without_timeout(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            for shift in (-8, 8):
                with self.subTest(side=side, shift=shift):
                    h = self.pending(side)
                    baseline = list(h.tracker._baseline_left_by_y2)
                    widths = list(h.tracker._ring_width)
                    for i in range(3):
                        result = h.frame(left=25+shift, right=55+shift)
                        self.assertEqual(result['state'],
                                         'EXIT_REPAIR' if i == 2 else 'EXIT_CONFIRM')
                    self.assertEqual(list(h.tracker._baseline_left_by_y2), baseline)
                    self.assertEqual(list(h.tracker._ring_width), widths)
                    # Match the actual moved corridor in the raw mask too.
                    for _ in range(h.tracker.exit_repair_frames):
                        result = h.frame(left=25+shift, right=55+shift,
                            mask_info=h.mask(left_by_y=lambda y:25+shift,
                                             right_by_y=lambda y:55+shift))
                    self.assertEqual(result['state'], 'REACQUIRE')
                    self.assertTrue(result['exit_reobserved'])
                    for _ in range(2):
                        result = h.frame(left=25+shift, right=55+shift,
                            mask_info=h.mask(left_by_y=lambda y:25+shift,
                                             right_by_y=lambda y:55+shift))
                    self.assertEqual(result['state'], 'OBSERVE')
                    self.assertFalse(h.tracker._ring_cache_frozen)

    def test_visibility_inside_without_dropout_never_exits(self):
        for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            h = TrackElementsHarness()
            (h.complete_left_ring if side == te.SIDE_LEFT else h.complete_right_ring)()
            for _ in range(6):
                result = h.frame(left=33, right=63)
            self.assertEqual(result['state'], 'INSIDE')

    def test_single_edge_drift_unaccepted_ambiguous_and_curved_reject(self):
        for kind in ('width', 'unaccepted', 'ambiguous', 'curve'):
            for side in (te.SIDE_LEFT, te.SIDE_RIGHT):
                with self.subTest(kind=kind, side=side):
                    h = self.pending(side)
                    args = dict(left=33, right=63)
                    if kind == 'width': args['right'] = 66
                    if kind == 'unaccepted': args.update(road_accepted=False)
                    if kind == 'ambiguous': args['topology'] = dict(reason='BRANCH', complete=True, branch_evidence=True)
                    if kind == 'curve':
                        args.update(left_by_y=lambda y: 25+(y-8)//4,
                                    right_by_y=lambda y: 55+(y-8)//4)
                    for _ in range(4): result = h.frame(**args)
                    self.assertEqual(result['state'], 'EXIT_CONFIRM')
                    self.assertFalse(result['exit_reobserved'])

    def test_interruption_resets_continuous_reappearance_count(self):
        h = self.pending(te.SIDE_RIGHT)
        for _ in range(2): h.frame(left=33, right=63)
        self.assertEqual(h.tracker._exit_reappear_count, 2)
        h.frame(left=33, right=66)
        self.assertEqual(h.tracker._exit_reappear_count, 0)
        for _ in range(2): result = h.frame(left=33, right=63)
        self.assertEqual(result['state'], 'EXIT_CONFIRM')
        result = h.frame(left=33, right=63)
        self.assertEqual(result['state'], 'EXIT_REPAIR')

if __name__ == '__main__': unittest.main()
