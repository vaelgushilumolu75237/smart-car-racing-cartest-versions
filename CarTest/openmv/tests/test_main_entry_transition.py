"""Entry interpolation must retain current anchors through real main V3 output."""
from array import array
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
for location in (HERE, HERE.parent):
    if str(location) not in sys.path:
        sys.path.insert(0, str(location))
import test_fast_main_ring_slope as runtime
import config
import main
import vision_protocol


def transition_vector(side):
    main._load_fast_runtime()
    main._load_singleedge_runtime()
    main._load_track_elements_runtime()
    te = main.track_elements
    rows = array('i')
    status = bytearray(120)
    edges = array('i', [0]*120)
    for y in range(8, 60):
        center2 = 100 if y <= 20 else 80 if y >= 48 else 100-(y-20)*20//28
        left = (center2-40)//2
        right = left+40
        if side == te.SIDE_LEFT:
            left, right = 79-right, 79-left
        if y <= 20:
            observed = side
        elif y >= 48:
            observed = 1-side
        else:
            observed = -1
        flags = te.TARGET_WIDTH_FROM_CALIBRATION | te.TARGET_ENTRY_TRANSITION
        if observed == te.SIDE_LEFT:
            flags |= te.TARGET_OBSERVED_LEFT | te.TARGET_INFERRED_RIGHT
            status[y] = 1
            edges[y] = left*2-1
        elif observed == te.SIDE_RIGHT:
            flags |= te.TARGET_OBSERVED_RIGHT | te.TARGET_INFERRED_LEFT
            status[60+y] = 1
            edges[60+y] = right*2+1
        else:
            flags |= te.TARGET_INFERRED_LEFT | te.TARGET_INFERRED_RIGHT
        rows.extend((y, left, right, flags))
    element = dict(control_eligible=True, target_valid=True, side=side, target_side=side,
                   state='INSIDE', state_id=te.STATE_INSIDE, ring_confirmed=True,
                   target_rows=rows, target_count=52, target_stride=4,
                   entry_transition=True, entry_transition_frame=1,
                   entry_transition_far_y=20, entry_transition_near_y=48,
                   session_token=1, profile_id=3, profile_seq=11)
    tracker = types.SimpleNamespace(min_target_rows=5, max_cache_gap=3, near_rows=12,
        require_calibration=True, ring_calibration_ready=True, ring_calibration_check_ready=True,
        ring_arc_min_rows=4, ring_arc_min_contiguous_rows=3, ring_arc_min_span_rows=4,
        min_side_support=5, min_side_contiguous=4)
    tracker.entry_transition_valid_for = lambda value: value is element
    edge_tracker = types.SimpleNamespace(_row_status=status, _row_edge2=edges)
    return tracker, element, edge_tracker


def convert(tracker, element, edges):
    result = {}
    output = array('i', [0]*300)
    count = main._fill_ring_control_route(element, tracker, edges,
        False, True, result, output, config)
    return count, result, output


class MainEntryTransitionTests(unittest.TestCase):
    def test_bridge_is_explicitly_inferred_and_has_correct_weighted_direction(self):
        for side in (0, 1):
            tracker, element, edges = transition_vector(side)
            count, result, output = convert(tracker, element, edges)
            self.assertEqual(count, 52)
            self.assertEqual(result['actual_boundary'], 'NONE')
            self.assertIsNone(result['actual_boundary_x'])
            row40 = (40-8)*5
            self.assertEqual(output[row40+3], main.track_elements.SIDE_NONE)
            self.assertEqual(output[row40+4] & 3, 0)  # no fabricated observed edge
            selection = dict(valid=True, source='ring_left' if side == 0 else 'ring_right',
                             result=result)
            info = main._FastRowWeightWorkspace().aggregate(selection, None, config)
            self.assertTrue(info['valid'])
            error = info.get('position_error', info.get('weighted_position_error'))
            self.assertIsNotNone(error)
            self.assertLess(error, 0) if side == 0 else self.assertGreater(error, 0)

    def test_certificate_and_current_endpoint_evidence_are_required(self):
        tracker, element, edges = transition_vector(1)
        self.assertEqual(convert(tracker, element.copy(), edges)[0], 0)
        for missing in ('far', 'near'):
            tracker, element, edges = transition_vector(1)
            if missing == 'far':
                edges._row_status[60:81] = bytearray(21)
            else:
                edges._row_status[48:60] = bytearray(12)
            self.assertEqual(convert(tracker, element, edges)[0], 0)
        tracker, element, edges = transition_vector(1)
        tracker.ring_calibration_check_ready = False
        self.assertEqual(convert(tracker, element, edges)[0], 0)

    def test_existing_ordinary_main_loop_emits_mirrored_entry_v3_packets(self):
        original_edges = runtime._SingleEdgeTracker
        sent_errors = []
        for side in (0, 1):
            tracker, element, edges = transition_vector(side)

            class Entry(runtime._ActiveRingElements):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    for key, value in vars(tracker).items():
                        setattr(self, key, value)
                    self.rows = element['target_rows']
                    self.result = element

            class CurrentEdges(original_edges):
                def __init__(self, *args):
                    super().__init__(*args)
                    self._row_status[:] = edges._row_status
                    self._row_edge2[:] = edges._row_edge2
                    self._frame_scan_done = True

            runner = runtime.FastMainRingSlopeWireTests(methodName=
                'test_fresh_profile_b_ring_target_reaches_the_existing_uart_chain')
            with patch.object(runtime, '_SingleEdgeTracker', CurrentEdges):
                packet, log, cache_commits = runner._run_profile_element_case(Entry, 3)
            self.assertEqual(packet[11], 3)
            self.assertTrue(packet[6] & vision_protocol.FLAG_VALID, log)
            self.assertEqual(vision_protocol.crc16_ccitt_false(packet[2:14]),
                             int.from_bytes(packet[14:16], 'little'))
            error = int.from_bytes(packet[8:10], 'little', signed=True)
            sent_errors.append(error)
            self.assertLess(error, 0) if side == 0 else self.assertGreater(error, 0)
            self.assertEqual(cache_commits, [])
            self.assertIn('RING_ENTRY_TRANSITION_TARGET', log)
        self.assertLessEqual(abs(sum(sent_errors)), 1)


if __name__ == '__main__':
    unittest.main()
