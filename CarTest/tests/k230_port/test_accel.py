import sys,random,unittest
import types
from collections import deque
import numpy as np
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openmv'))
import config,fast_gray,fast_topology,fast_path,k230_accel as accel
from array import array

class NativeImageOracle:
    GRAYSCALE=1
    class Image:
        def __init__(self,width,height,fmt,data):
            self.width=width;self.height=height;self.data=bytearray(data)
        def get_histogram(self,roi,bins):
            x,y,w,h=roi
            pixels=np.frombuffer(self.data,dtype=np.uint8).reshape((self.height,self.width))[y:y+h,x:x+w]
            histogram=np.bincount(pixels.ravel(),minlength=256).astype(np.float32)/np.float32(w*h)
            return types.SimpleNamespace(bins=lambda:histogram.tolist())
        def binary(self,thresholds):
            low,high=thresholds[0]
            self.data[:]=bytes(255 if low<=v<=high else 0 for v in self.data)
        def bytearray(self):return self.data
        def find_blobs(self,thresholds,roi,x_stride,y_stride,pixels_threshold,area_threshold,merge):
            x0,y0,w,h=roi;seen=set();blobs=[]
            for y in range(y0,y0+h):
                for x in range(x0,x0+w):
                    if (x,y) in seen or self.data[y*self.width+x]!=0:continue
                    q=deque([(x,y)]);seen.add((x,y));points=[]
                    while q:
                        px,py=q.popleft();points.append((px,py))
                        for nx,ny in ((px-1,py),(px+1,py),(px,py-1),(px,py+1)):
                            if x0<=nx<x0+w and y0<=ny<y0+h and (nx,ny) not in seen and self.data[ny*self.width+nx]==0:
                                seen.add((nx,ny));q.append((nx,ny))
                    if len(points)>=pixels_threshold:
                        xa=min(p[0] for p in points);xb=max(p[0] for p in points)
                        ya=min(p[1] for p in points);yb=max(p[1] for p in points)
                        rect=(xa,ya,xb-xa+1,yb-ya+1)
                        blobs.append(types.SimpleNamespace(rect=lambda rect=rect:rect))
            return blobs

class AccelerationEquivalence(unittest.TestCase):
    def test_pool_histogram_and_masks(self):
        rng=random.Random(1946)
        for trial in range(16):
            raw=bytearray(rng.randrange(256) for _ in range(320*120))
            expected=bytearray(4800);actual=bytearray(4800)
            fast_gray._min_pool_host(raw,expected,320,80,60,4,2)
            accel.min_pool(raw,actual,320,80,60)
            self.assertEqual(actual,expected)
            reference=fast_gray.FastGrayMasks(config)
            candidate=fast_gray.FastGrayMasks(config)
            reference.luma[:]=expected;candidate.luma[:]=actual
            with patch.object(fast_gray,'_K230_ACCEL',None):
                reference._calculate_thresholds();reference._write_masks()
            with patch.object(fast_gray,'_K230_ACCEL',accel),patch.object(accel,'_image',NativeImageOracle):
                candidate._calculate_thresholds();candidate._write_masks()
            for name in ('_histograms','_block_stats','_nonzero_counts','thresholds','trusted','road_mask','edge_mask','known_region','row_counts','road_row_counts','_pixel_totals'):
                self.assertEqual(getattr(candidate,name),getattr(reference,name),(trial,name))

    def test_row_extraction_exact(self):
        rng=random.Random(3928)
        for trial in range(80):
            a=fast_path.FastPathDetector(config);b=fast_path.FastPathDetector(config)
            luma=bytearray(rng.randrange(256) for _ in range(4800))
            road=bytearray(4800);known=bytearray([1]*4800);edge=bytearray([1]*4800)
            for y in range(8,60):
                for _ in range(rng.randrange(1,5)):
                    x0=rng.randrange(78);x1=rng.randrange(x0+1,81)
                    road[y*80+x0:y*80+x1]=bytes([255])*(x1-x0)
                    edge[y*80+x0:y*80+x1]=bytes(x1-x0)
            if trial%3==0:known[1000:1100]=bytes(100)
            for detector in (a,b):
                detector._current_luma=luma;detector._current_road=road
                detector._current_known=known;detector._current_edge=edge
            expected=fast_path._extract_viper(a);actual=accel.extract_rows(b)
            self.assertEqual(actual,expected,trial)
            self.assertEqual(a._counts,b._counts,trial)
            self.assertEqual(a._candidate_data,b._candidate_data,trial)

    def test_topology_random_rectangles_and_holes(self):
        rng=random.Random(19231)
        for trial in range(160):
            mask=bytearray(4800);known=bytearray([1]*4800)
            for _ in range(rng.randrange(1,18)):
                x0=rng.randrange(79);x1=rng.randrange(x0+1,81)
                y0=rng.randrange(8,59);y1=rng.randrange(y0+1,61)
                value=rng.randrange(2)
                for y in range(y0,y1):mask[y*80+x0:y*80+x1]=bytes([value])*(x1-x0)
            if trial%5==0:
                y=rng.randrange(8,60);known[y*80:(y+1)*80]=bytes(80)
            if trial%7==0:known[1000]=0
            rows=bytearray(sum(1 for x in mask[y*80:(y+1)*80] if x) for y in range(60))
            info=dict(width=80,height=60,floor=8,edge_mask=mask,known_region=known,row_counts=rows)
            ref=fast_topology.FastTopologyGuard(config);test=fast_topology.FastTopologyGuard(config)
            with patch.object(fast_topology,'_K230_ACCEL',None):expected=dict(ref.check(info))
            with patch.object(fast_topology,'_K230_ACCEL',accel),patch.object(accel,'_image',NativeImageOracle):actual=dict(test.check(info))
            self.assertEqual(actual,expected,trial)

if __name__=='__main__':unittest.main()
