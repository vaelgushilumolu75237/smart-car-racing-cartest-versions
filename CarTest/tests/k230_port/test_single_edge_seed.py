import sys,unittest
from pathlib import Path
from array import array
sys.path[:0]=[str(Path(__file__).resolve().parents[2]/'openmv'),str(Path(__file__).resolve().parents[2]/'openmv/tests')]
import main,config,fast_single_edge
import test_fast_single_edge as fixtures

class DenseSingleSeedTests(unittest.TestCase):
 def run_case(self,side,dense):
  tracker=fast_single_edge.FastSingleEdgeTracker(config)
  both=fixtures._mask('both');candidate=fixtures._bilateral_candidate()
  if not dense:
   records=candidate['selected_row_ints'];sampled=[]
   for i in range(candidate['selected_row_count']):
    if (59-records[i*4])%3==1:sampled.extend(records[i*4:i*4+4])
   candidate=dict(candidate,selected_row_ints=array('i',sampled),selected_row_count=len(sampled)//4)
  tracker.observe_bilateral(candidate,both['luma'],both['edge_mask'],both['road_mask'],both['known_region'],True,True)
  one=fixtures._mask(side)
  for y in range(8,60,4):fixtures._remove_edge_rows(one,y,y)
  return tracker,tracker.detect(one,True,{'reason':'OK','complete':True})
 def test_reproduces_sparse_cache_failure_for_both_sides(self):
  for side in ('left','right'):
   tracker,result=self.run_case(side,False)
   self.assertFalse(result['valid'])
   self.assertEqual(result['reason'],'SINGLE_EDGE_SUPPORT_ROWS')
 def test_dense_measured_seed_recovers_fragmented_left_and_right_edges(self):
  for side in ('left','right'):
   tracker,result=self.run_case(side,True)
   self.assertTrue(result['valid'],result['reason'])
   self.assertEqual(result['boundary_source'],'LEFT_ONLY' if side=='left' else 'RIGHT_ONLY')
   self.assertGreaterEqual(result['selected_row_count'],30)
   self.assertFalse(main._needs_dense_single_edge_seed(tracker,True))
 def test_reset_requires_new_dense_seed_but_stale_control_cannot_seed(self):
  tracker,result=self.run_case('left',True)
  tracker.clear('RESET')
  self.assertTrue(main._needs_dense_single_edge_seed(tracker,True))
  self.assertFalse(main._needs_dense_single_edge_seed(tracker,False))
 def test_dense_seed_never_bypasses_fresh_control_requirement(self):
  tracker,result=self.run_case('left',True)
  lost=tracker.detect(fixtures._mask('left'),False,{'reason':'OK','complete':True})
  self.assertFalse(lost['valid']);self.assertEqual(lost['reason'],'NO_FRESH_KEEP')
