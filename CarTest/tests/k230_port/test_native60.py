"""Native backend contracts; real cv_lite timings are measured on the board."""
import sys, unittest, types
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openmv'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openmv/tests'))
import config, fast_gray, fast_path, k230_settings, k230_accel as accel

class ThresholdOracle:
 @staticmethod
 def rgb888_erode(shape,rgb,kernel,iterations,threshold):
  gray=rgb.reshape((-1,3))[:,0]
  hist=np.bincount(gray,minlength=256)
  cumulative=np.cumsum(hist);weighted=np.cumsum(hist*np.arange(256))
  denominator=cumulative*(len(gray)-cumulative)
  scores=np.full(256,-1.0)
  good=denominator>0
  scores[good]=(weighted[-1]*cumulative[good]-weighted[good]*len(gray))**2/denominator[good]
  cut=int(np.argmax(scores))
  return np.array(gray>cut,dtype=np.uint8).reshape(tuple(shape)+(1,))*255

class UlabAxisSums:
 """Model the board's dtype-preserving axis reduction, including uint8 wrap."""
 def __getattr__(self,name):return getattr(np,name)
 def sum(self,values,axis=None):
  return np.sum(values,axis=axis,dtype=values.dtype if axis is not None else None)

class NativeContracts(unittest.TestCase):
 def calculate(self,builder):
  with patch.object(fast_gray,'_K230_ACCEL',accel),patch.object(accel,'_cv',ThresholdOracle):
   builder._calculate_thresholds();builder._write_masks()
 def test_native_class_statistics_and_cached_masks_are_per_builder(self):
  a=fast_gray.FastGrayMasks(config);b=fast_gray.FastGrayMasks(config)
  a.luma[:]=bytes(30 if i%80<30 else 220 for i in range(4800))
  b.luma[:]=bytes(70 if i%80<45 else 180 for i in range(4800))
  self.calculate(a);saved=bytes(a.road_mask);self.calculate(b)
  with patch.object(accel,'_cv',ThresholdOracle):self.assertTrue(accel.native_masks(a))
  self.assertEqual(bytes(a.road_mask),saved)
  for block in range(a._block_count):
   x0,y0,x1,y1=a._rectangles[block*4:block*4+4]
   samples=[a.luma[y*80+x] for y in range(y0,y1) for x in range(x0,x1)]
   self.assertEqual(a._block_stats[block*4],len(samples))
   self.assertEqual(a._block_stats[block*4+1],sum(samples))
   self.assertEqual(sum(a.classes[block]),len(samples))
   self.assertAlmostEqual(a.class_means[block][0],30)
   self.assertAlmostEqual(a.class_means[block][1],220)
  self.assertNotEqual(a.road_mask,b.road_mask)

 def test_batched_native_statistics_match_independent_current_pixel_reductions(self):
  rng=np.random.default_rng(880027)
  builder=fast_gray.FastGrayMasks(config)
  for trial in range(60):
   values=rng.integers(0,256,size=4800,dtype=np.uint8)
   if trial%5==0:values[:]=trial
   if trial%7==0:values[::2]=20;values[1::2]=220
   builder.luma[:]=values.tobytes()
   with patch.object(accel,'_cv',ThresholdOracle),patch.object(accel,'np',UlabAxisSums()):
    actual=accel.native_thresholds(builder)
   for block in range(builder._block_count):
    x0,y0,x1,y1=builder._rectangles[block*4:block*4+4]
    gray=values.reshape((60,80))[y0:y1,x0:x1]
    binary=ThresholdOracle.rgb888_erode(gray.shape,np.repeat(gray.ravel()[:,None],3,axis=1),1,1,0).reshape(gray.shape)
    dark=gray[binary==0];light=gray[binary!=0]
    expected=(0,0,0,0.0,0.0,0.0) if not len(dark) or not len(light) else (
     int(np.max(dark)),len(dark),len(light),float(np.mean(dark)),float(np.mean(light)),float(np.mean(light))-float(np.mean(dark)))
    self.assertEqual(actual[block],expected,(trial,block))
    self.assertEqual(list(builder._block_stats[block*4:block*4+4]),[
     gray.size,int(np.sum(gray)),int(np.min(gray)),int(np.max(gray))])
 def test_constant_and_low_contrast_cannot_leave_old_masks(self):
  a=fast_gray.FastGrayMasks(config)
  for low,high in ((20,220),(80,80),(80,81)):
   a.luma[:]=bytes(low if i%80<40 else high for i in range(4800))
   self.calculate(a)
   if high-low<2:
    self.assertFalse(any(a.trusted))
    self.assertFalse(any(a.known_region))
    self.assertFalse(any(a.road_mask))
    self.assertFalse(any(a.edge_mask))
 def test_sampled_rows_match_reference_and_clear_stale_counts(self):
  for bend in (0,1,-1):
   a=fast_path.FastPathDetector(config);b=fast_path.FastPathDetector(config)
   luma=bytearray([20]*4800);road=bytearray(4800);known=bytearray([1]*4800);edge=bytearray([1]*4800)
   for y in range(60):
    start=20+bend*(y//10);stop=start+35
    road[y*80+start:y*80+stop]=bytes([255])*35
    edge[y*80+start:y*80+stop]=bytes(35)
    luma[y*80+start:y*80+stop]=bytes([210])*35
   for d in (a,b):
    d._current_luma=luma;d._current_road=road;d._current_known=known;d._current_edge=edge
   fast_path._extract_viper(a)
   b._counts[:]=bytes([9])*len(b._counts)
   with patch.object(accel,'_cv',ThresholdOracle):accel.extract_rows(b)
   for row in range(b.row_count):
    selected=row%3==1 or row<config.TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS
    self.assertEqual(b._counts[row],a._counts[row] if selected else 0)
    if selected:
     start=row*b.max_per_row*4;stop=start+a._counts[row]*4
     self.assertEqual(b._candidate_data[start:stop],a._candidate_data[start:stop])
   road[:]=bytes(4800)
   with patch.object(accel,'_cv',ThresholdOracle):accel.extract_rows(b)
   self.assertFalse(any(b._counts))
 def test_calibration_override_restores_dense_rows_then_fast_sampling(self):
  d=fast_path.FastPathDetector(config)
  road=bytearray(4800);edge=bytearray([1]*4800);known=bytearray([1]*4800);luma=bytearray([20]*4800)
  for y in range(60):
   road[y*80+20:y*80+55]=bytes([255])*35
   edge[y*80+20:y*80+55]=bytes(35)
   luma[y*80+20:y*80+55]=bytes([210])*35
  d._current_luma=luma;d._current_road=road;d._current_known=known;d._current_edge=edge
  with patch.object(accel,'_cv',ThresholdOracle):
   d.path_row_step=1;d.path_row_phase=0;accel.extract_rows(d)
   self.assertGreaterEqual(sum(bool(n) for n in d._counts),20)
   d.path_row_step=3;d.path_row_phase=1;accel.extract_rows(d)
   self.assertLessEqual(sum(bool(n) for n in d._counts),21)
 def test_invalid_sampling_is_rejected(self):
  d=fast_path.FastPathDetector(config)
  for name in ('luma','road','known','edge'):setattr(d,'_current_'+name,bytearray(4800))
  with patch.object(accel,'_cv',ThresholdOracle),patch.object(k230_settings,'PATH_ROW_STEP',4):
   with self.assertRaises(ValueError):accel.extract_rows(d)

 def test_rotating_scan_keeps_actual_reference_row_and_control_stable(self):
  from test_fast_single_edge import _mask
  detector=fast_path.FastPathDetector(config)
  mask=_mask('both')
  references=[]
  for phase in range(3):
   detector.path_row_step=3;detector.path_row_phase=phase
   with patch.object(fast_path,'_K230_ACCEL',accel),patch.object(accel,'_cv',ThresholdOracle):
    result=detector.detect(mask)
   self.assertTrue(result['valid'],result['reason'])
   references.append((result['reference_y'],result['reference_center_x']))
  self.assertEqual(references,[(40.0,39.5)]*3)

 def test_rotating_rows_preserve_ring_near_field_baseline(self):
  from test_fast_single_edge import _mask
  from test_track_elements import TrackElementsHarness
  from array import array
  harness=TrackElementsHarness()
  detector=fast_path.FastPathDetector(config)
  mask=_mask('both')
  for frame in range(6):
   detector.path_row_step=3;detector.path_row_phase=frame%3
   with patch.object(fast_path,'_K230_ACCEL',accel),patch.object(accel,'_cv',ThresholdOracle):
    road=detector.detect(mask)
   self.assertTrue(road['valid'])
   rows=road['selected_row_ints'];count=road['selected_row_count']
   near={rows[i*4] for i in range(count)}
   self.assertTrue({56,57,58,59}.issubset(near))
   harness.now+=30
   harness.tracker.update(harness.now,2,True,'session-1',True,True,
    rows,count,4,road['candidate_tracks'],{'complete':True,'reason':'OK'},
    False,bytearray(120),array('i',[0]*120),array('i',[0]*120),
    harness.cache_valid,harness.cache_width,harness.cache_left2,
    harness.cache_right2,mask,frame+1,100+frame*10)
  self.assertTrue(harness.tracker._baseline_ready)

if __name__=='__main__':unittest.main()
