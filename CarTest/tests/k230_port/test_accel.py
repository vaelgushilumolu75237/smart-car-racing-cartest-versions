import sys,random,unittest
import types
from collections import deque
import numpy as np
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openmv'))
import config,fast_gray,fast_topology,fast_path,k230_accel as accel
from array import array

def components(pixels, diagonal=False):
    height,width=pixels.shape;seen=set();result=[]
    for y in range(height):
        for x in range(width):
            if not pixels[y,x] or (x,y) in seen:continue
            pending=[(x,y)];seen.add((x,y));points=set()
            while pending:
                px,py=pending.pop();points.add((px,py))
                for dy in (-1,0,1):
                    for dx in (-1,0,1):
                        if (dx==dy==0) or (not diagonal and dx and dy):continue
                        nx,ny=px+dx,py+dy
                        if (0<=nx<width and 0<=ny<height and pixels[ny,nx] and
                                (nx,ny) not in seen):
                            seen.add((nx,ny));pending.append((nx,ny))
            result.append(frozenset(points))
    return set(result)

class CVExternalContoursOracle:
    @staticmethod
    def grayscale_find_blobs(shape,pixels,*args):
        pixels=pixels.reshape(shape)
        background=components(pixels==0)
        height,width=pixels.shape
        exterior=set().union(*(points for points in background if any(
            x in (0,width-1) or y in (0,height-1) for x,y in points))) if background else set()
        result=[]
        for points in components(pixels,diagonal=True):
            if not any(x in (0,width-1) or y in (0,height-1) or any(
                (x+dx,y+dy) in exterior for dx,dy in ((-1,0),(1,0),(0,-1),(0,1)))
                for x,y in points):continue
            xs=[p[0] for p in points];ys=[p[1] for p in points]
            result.extend((min(xs),min(ys),max(xs)-min(xs)+1,max(ys)-min(ys)+1))
        return result

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
    def test_white_run_proof_includes_nested_four_connected_components(self):
        rng = random.Random(880028)
        fixtures = []
        for bits in range(512):
            fixtures.append(np.array([(bits >> bit) & 1 for bit in range(9)],
                                     dtype=np.uint8).reshape((3, 3)))
        for _ in range(300):
            fixtures.append(np.array([rng.randrange(2) for _ in range(48)],
                                     dtype=np.uint8).reshape((6, 8)))
        nested = np.zeros((20, 20), dtype=np.uint8)
        nested[3:17, 3:17] = 1
        nested[5:15, 5:15] = 0
        nested[8:12, 8:12] = 1
        fixtures.append(nested)
        for dark in fixtures:
            height, width = dark.shape
            expected = all(any(x in (0, width - 1) or y in (0, height - 1)
                               for x, y in component)
                           for component in components(dark == 0))
            self.assertEqual(accel._white_exterior_runs(
                dark.tobytes(), width, 0, height), expected)
        checker = np.indices((60, 80)).sum(axis=0) % 2
        self.assertFalse(accel._white_exterior_runs(
            checker.astype(np.uint8).tobytes(), 80, 8, 60))

    def test_native_prefilter_preserves_topology_for_diagonals_nested_holes_and_noise(self):
        rng=random.Random(880025)
        fixtures=[]
        for trial in range(30):
            mask=bytearray(4800)
            if trial<4:
                for i in range(12):
                    x=20+i;y=24+i if trial%2 else 36-i
                    mask[y*80+x]=1
                if trial>=2:
                    for y in range(18,44):
                        for x in range(14,40):
                            if y in (18,43) or x in (14,39):mask[y*80+x]=1
            else:
                for _ in range(20):
                    x=rng.randrange(80);y=rng.randrange(8,60)
                    w=rng.randrange(1,12);h=rng.randrange(1,12)
                    for row in range(y,min(60,y+h)):
                        mask[row*80+x:row*80+min(80,x+w)]=bytes([1])*min(w,80-x)
            fixtures.append(mask)
        for trial,mask in enumerate(fixtures):
            info=dict(width=80,height=60,floor=8,edge_mask=mask,
                      known_region=bytearray([1]*4800),
                      row_counts=bytearray(sum(bool(x) for x in mask[y*80:(y+1)*80]) for y in range(60)))
            ref=fast_topology.FastTopologyGuard(config);candidate=fast_topology.FastTopologyGuard(config)
            with patch.object(fast_topology,'_K230_ACCEL',None):expected=dict(ref.check(info))
            with patch.object(fast_topology,'_K230_ACCEL',accel),patch.object(accel,'_cv',CVExternalContoursOracle):
                actual=dict(candidate.check(info))
            self.assertEqual(actual,expected,trial)

    def test_binary_pack_matches_wire_bits_for_short_long_and_random_masks(self):
        rng = random.Random(2026111024)
        for length in (0, 1, 7, 8, 9, 4799, 4800, 4801, 10000):
            for trial in range(8):
                data = bytearray(rng.randrange(256) if trial else 0
                                 for _ in range(length))
                expected = bytearray(600)
                for index, value in enumerate(data[:4800]):
                    if value:
                        expected[index // 8] |= 1 << (index % 8)
                self.assertEqual(accel.pack_binary(data), expected,
                                 (length, trial))

    def test_portable_connect_two_best_states_match_reference(self):
        rng=random.Random(20261110)
        for trial in range(160):
            reference=fast_path.FastPathDetector(config)
            candidate=fast_path.FastPathDetector(config)
            for layer in range(reference.row_count):
                count=rng.randrange(4) if trial%3 else 1
                reference._counts[layer]=count
                for slot in range(count):
                    at=(layer*reference.max_per_row+slot)*4
                    center=80+rng.randrange(-18,19)
                    reference._candidate_data[at:at+4]=array('i',[
                        15,center,45+rng.randrange(-6,7),rng.randrange(-1000,100001)])
            candidate._counts[:]=reference._counts
            candidate._candidate_data[:]=reference._candidate_data
            fast_path._connect_viper(reference)
            accel.connect_rows(candidate)
            self.assertEqual(candidate._state_data,reference._state_data,trial)

    def test_single_candidate_selection_matches_independent_historical_kernel(self):
        import importlib.util
        path=Path(__file__).resolve().parents[2]/'openmv/tests/hotloop_reference_20261023/fast_path.py'
        spec=importlib.util.spec_from_file_location('single_selection_reference',path)
        historical=importlib.util.module_from_spec(spec);spec.loader.exec_module(historical)
        rng=random.Random(880026)
        for trial in range(150):
            reference=historical.FastPathDetector(config)
            candidate=fast_path.FastPathDetector(config)
            for layer in range(reference.row_count):
                reference._counts[layer]=int(rng.randrange(6)!=0)
                offset=layer*reference.max_per_row*4
                reference._candidate_data[offset:offset+4]=array('i',[
                    10,70+rng.randrange(20),35+rng.randrange(10),rng.randrange(80000)])
            candidate._counts[:]=reference._counts
            candidate._candidate_data[:]=reference._candidate_data
            # The older kernel stores right x; the current kernel stores
            # left+right in the second candidate field.
            for layer in range(reference.row_count):
                offset=layer*reference.max_per_row*4
                reference._candidate_data[offset+1]-=reference._candidate_data[offset]
            historical._connect_viper(reference);accel.connect_rows(candidate)
            self.assertEqual(fast_path._select_viper(candidate),historical._select_viper(reference),trial)
            for name in ('_control','_state_data','_best_layer_observed','_best_centers_by_layer','_trace'):
                self.assertEqual(getattr(candidate,name),getattr(reference,name),(trial,name))

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

    def test_batch_otsu_exact_random_and_constant_histograms(self):
        rng=random.Random(320060)
        builder=fast_gray.FastGrayMasks(config)
        for trial in range(1000):
            for block in range(builder._block_count):
                hist=[0]*256
                if trial%17==0:
                    hist[rng.randrange(256)]=1200
                elif trial%19:
                    for _ in range(2+rng.randrange(220)):
                        hist[rng.randrange(256)]+=1+rng.randrange(9)
                values=[i for i in range(256) if hist[i]]
                base=block*256
                builder._histograms[base:base+256]=array('I',hist)
                builder._nonzero_values[base:base+len(values)]=bytes(values)
                builder._nonzero_counts[block]=len(values)
                builder._block_stats[block*4:block*4+4]=array('I',[
                    sum(hist),sum(i*h for i,h in enumerate(hist)),
                    values[0] if values else 256,values[-1] if values else 0])
            actual=accel.batch_otsu(builder)
            expected=[fast_gray._otsu_native(
                builder._histograms,builder._nonzero_values,block*256,
                builder._nonzero_counts[block],builder._block_stats[block*4],
                builder._block_stats[block*4+1])
                for block in range(builder._block_count)]
            self.assertEqual(actual,expected,trial)

    def test_native_histogram_bad_counts_recount_current_pixels(self):
        class CorruptHistogram(NativeImageOracle):
            class Image(NativeImageOracle.Image):
                def get_histogram(self,roi,bins):
                    return types.SimpleNamespace(bins=lambda:[0.5]*256)
        rng=random.Random(671)
        reference=fast_gray.FastGrayMasks(config)
        candidate=fast_gray.FastGrayMasks(config)
        pixels=bytearray(rng.randrange(256) for _ in range(4800))
        reference.luma[:]=pixels;candidate.luma[:]=pixels
        with patch.object(fast_gray,'_K230_ACCEL',None):
            reference._calculate_thresholds();reference._write_masks()
        with patch.object(fast_gray,'_K230_ACCEL',accel),patch.object(accel,'_image',CorruptHistogram):
            candidate._calculate_thresholds();candidate._write_masks()
        for name in ('_histograms','_block_stats','_nonzero_counts','thresholds','trusted','road_mask','edge_mask','known_region','row_counts','road_row_counts','_pixel_totals'):
            self.assertEqual(getattr(candidate,name),getattr(reference,name),name)

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
