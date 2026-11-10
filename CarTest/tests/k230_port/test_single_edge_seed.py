import sys,unittest
from pathlib import Path
from array import array
sys.path[:0]=[str(Path(__file__).resolve().parents[2]/'openmv'),str(Path(__file__).resolve().parents[2]/'openmv/tests')]
import main,config,fast_single_edge,fast_path,k230_accel
from unittest.mock import patch
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

 def test_sparse_scanline_dropouts_retry_same_frame_with_original_gap_limit(self):
  # Exactly the sampled rows are weak: dense still observes two rows in three.
  mask=fixtures._mask('both')
  mask.update(valid=True,width=80,height=60,floor=8)
  # Topology inspects all pixels; a weak boundary sample is not a fake hole.
  for y in range(8,60):
   if (59-y)%3==1 and y%9!=4:
    fixtures._remove_edge_rows(mask,y,y)
  mask['row_counts']=bytearray(sum(mask['edge_mask'][y*80:(y+1)*80]) for y in range(60))
  detector=fast_path.FastPathDetector(config)
  with patch.object(fast_path,'_K230_ACCEL',k230_accel),patch.object(k230_accel,'_cv',object()):
   detector.path_row_step=3;detector.path_row_phase=1
   sparse=detector.detect(mask)
   self.assertFalse(sparse['valid'])
   result=main._detect_sampled_road(detector,mask,False,3,1)
  self.assertTrue(result['valid'],result['reason'])
  self.assertEqual(detector.path_row_step,1)
  self.assertEqual(detector.max_gap,2)

 def test_native_transition_scan_matches_all_original_candidate_gates(self):
  for side in ('left','right','both'):
   for gaps in (False,True):
    reference=fast_single_edge.FastSingleEdgeTracker(config)
    native=fast_single_edge.FastSingleEdgeTracker(config)
    fixtures._warm_tracker(reference);fixtures._warm_tracker(native)
    mask=fixtures._mask(side)
    if gaps:
     for y in range(8,60,4):fixtures._remove_edge_rows(mask,y,y)
    topology={'reason':'OK','complete':True}
    with patch.object(fast_single_edge,'_K230_ACCEL',None):
     expected=dict(reference.detect(mask,True,topology))
    with patch.object(fast_single_edge,'_K230_ACCEL',k230_accel):
     actual=dict(native.detect(mask,True,topology))
    for key in ('valid','reason','selected_row_count','boundary_source','lateral_error'):
     self.assertEqual(actual.get(key),expected.get(key),(side,gaps,key))
    for key in ('_row_status','_row_edge2','_row_center2','_row_contrast','_diagnostic_counts'):
     self.assertEqual(getattr(native,key),getattr(reference,key),(side,gaps,key))

 def test_sparse_boundary_dropout_retries_dense_without_weakening_gate(self):
  import fast_boundary
  for weak_all in (False,True):
   mask=fixtures._mask('both')
   mask.update(valid=True,width=80,height=60,floor=8)
   mask['row_counts']=bytearray(sum(mask['edge_mask'][y*80:(y+1)*80]) for y in range(60))
   for y in range(8,60):
    if weak_all or (35<=y<=45 and ((59-y)%3==1 or y==40)):
     # Same-frame masks still delimit a corridor, but these gray steps do
     # not qualify as boundary support. Other rows remain measurable.
     mask['luma'][y*80+fixtures.LEFT-1]=210
     mask['luma'][y*80+fixtures.RIGHT+1]=210
   detector=fast_path.FastPathDetector(config)
   validator=fast_boundary.FastBoundaryEvidence(config)
   with patch.object(fast_path,'_K230_ACCEL',k230_accel),patch.object(k230_accel,'_cv',object()):
    detector.path_row_step=3;detector.path_row_phase=1
    road=detector.detect(mask)
    self.assertTrue(road['valid'],road['reason'])
    result=main._gate_sampled_road_boundary(detector,road,mask,validator,config)
   self.assertEqual(result['valid'],not weak_all,result['reason'])
   self.assertEqual(detector.path_row_step,1)
   if weak_all:self.assertEqual(result['reason'],'BOUNDARY_EVIDENCE_REJECTED')

 def test_rotating_bilateral_rows_keep_curve_anchors_current_for_both_sides(self):
  for direction,side in ((-1,'right'),(1,'left')):
   results=[]
   for rotate in (False,True):
    tracker=fast_single_edge.FastSingleEdgeTracker(config)
    fixtures._warm_tracker(tracker)
    for frame in range(60):
     shift=direction*(frame//3)
     mask=fixtures._mask('both',left=fixtures.LEFT+shift,right=fixtures.RIGHT+shift)
     phase=(1+frame)%3 if rotate else 1
     candidate=fixtures._bilateral_candidate_rows([
      (y,fixtures.LEFT+shift,fixtures.RIGHT+shift)
      for y in range(fixtures.FLOOR,60) if (59-y)%3==phase])
     tracker.observe_bilateral(candidate,mask['luma'],mask['edge_mask'],mask['road_mask'],mask['known_region'],True,True)
    mask=fixtures._mask(side,left=fixtures.LEFT+direction*19,right=fixtures.RIGHT+direction*19)
    for y in range(fixtures.FLOOR,60,4):fixtures._remove_edge_rows(mask,y,y)
    result=dict(tracker.detect(mask,True,{'reason':'OK','complete':True}))
    results.append(result)
   self.assertFalse(results[0]['valid'],results[0]['reason'])
   self.assertTrue(results[1]['valid'],results[1]['reason'])
   self.assertGreaterEqual(results[1]['selected_row_count'],30)
