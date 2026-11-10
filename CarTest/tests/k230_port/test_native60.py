"""Native backend contracts; real cv_lite timings are measured on the board."""
import sys, unittest, types
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openmv'))
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
    selected=row%3==1
    self.assertEqual(b._counts[row],a._counts[row] if selected else 0)
    if selected:
     start=row*b.max_per_row*4;stop=start+a._counts[row]*4
     self.assertEqual(b._candidate_data[start:stop],a._candidate_data[start:stop])
   road[:]=bytes(4800)
   with patch.object(accel,'_cv',ThresholdOracle):accel.extract_rows(b)
   self.assertFalse(any(b._counts))
 def test_invalid_sampling_is_rejected(self):
  d=fast_path.FastPathDetector(config)
  for name in ('luma','road','known','edge'):setattr(d,'_current_'+name,bytearray(4800))
  with patch.object(accel,'_cv',ThresholdOracle),patch.object(k230_settings,'PATH_ROW_STEP',4):
   with self.assertRaises(ValueError):accel.extract_rows(d)

if __name__=='__main__':unittest.main()
