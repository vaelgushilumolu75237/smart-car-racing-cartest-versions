"""Ring dropout evidence must retain the dense baseline used before sampling."""
from array import array
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'openmv'), str(ROOT / 'openmv/tests'),
                str(ROOT / 'tests/k230_port')]
import config
import main
import fast_path
import k230_accel as accel
import track_elements as te
from test_fast_single_edge import _mask
from test_native60 import ThresholdOracle
from test_track_elements import TrackElementsHarness


class RingBaselineSamplingTests(unittest.TestCase):
    def _dense_decision(self, tracker):
        tree = ast.parse((ROOT / 'openmv/main.py').read_text(encoding='utf-8-sig'))
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == '_main_fast')
        decision = next(n for n in ast.walk(fn) if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Name) and
                                t.id == '_dense_width_rows' for t in n.targets))
        scope = dict(vars(main))
        scope.update(ring_calibration_passed=True, reverse_fresh=True,
                     wireless_service=SimpleNamespace(camera_calibration_controller=
                                                       SimpleNamespace(active=False)),
                     single_edge_tracker=SimpleNamespace(route_mode='cached_width',
                                                         _cache_row_count=52),
                     reverse_parser=SimpleNamespace(profile_id=3,
                                                     profile_session_active=True),
                     element_tracker=tracker)
        return eval(compile(ast.Expression(decision.value), '<production-sampling>', 'eval'), scope)

    def test_sampled_main_baseline_preserves_left_and_right_entry(self):
        for ring_side in (te.SIDE_LEFT, te.SIDE_RIGHT):
            with self.subTest(ring_side=ring_side):
                h = TrackElementsHarness()
                d = fast_path.FastPathDetector(config)
                mask = _mask('both', left=25, right=55)
                dense_frames = []
                for frame in range(6):
                    dense = self._dense_decision(h.tracker)
                    dense_frames.append(dense)
                    d.path_row_step = 1 if dense else 3
                    d.path_row_phase = 0 if dense else frame % 3
                    with patch.object(fast_path, '_K230_ACCEL', accel), \
                            patch.object(accel, '_cv', ThresholdOracle):
                        road = d.detect(mask)
                    self.assertTrue(road['valid'], road['reason'])
                    h.now += 30
                    h.tracker.update(h.now, 3, True, 'session-1', True, True,
                        road['selected_row_ints'], road['selected_row_count'], 4,
                        road['candidate_tracks'], {'complete':True, 'reason':'OK'},
                        False, bytearray(120), array('i', [0]*120),
                        array('i', [0]*120), h.cache_valid, h.cache_width,
                        h.cache_left2, h.cache_right2, mask, frame+1, 100+frame*10)
                self.assertTrue(h.tracker._baseline_ready)
                self.assertEqual(sum(h.tracker._baseline_row_valid), 52,
                    'Sparse baseline breaks the required four consecutive dropout rows')
                self.assertEqual(dense_frames[:3], [True, True, True])
                self.assertFalse(dense_frames[-1], 'Resume sampling after baseline is ready')
                visible_side = te.SIDE_RIGHT if ring_side == te.SIDE_LEFT else te.SIDE_LEFT
                first = h.frame(profile_id=3, road_accepted=False, raw_side=visible_side)
                self.assertEqual(first['state_id'], te.STATE_ENTRY_CONFIRM)
                second = h.frame(profile_id=3, road_accepted=False, raw_side=visible_side)
                self.assertEqual(second['state_id'], te.STATE_ENTER)


if __name__ == '__main__':
    unittest.main()
