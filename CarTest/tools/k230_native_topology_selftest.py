import sys,os
sys.path.insert(0,'/sdcard/cartest-k230');os.chdir('/sdcard/cartest-k230')
import config,fast_topology,k230_accel
seed=19231
def rand(n):
 global seed
 seed=(seed*1103515245+12345)&0x7fffffff
 return seed%n
for trial in range(500):
 mask=bytearray(4800);known=bytearray([1]*4800)
 for rect in range(1+rand(17)):
  x0=rand(79);x1=x0+1+rand(80-x0)
  y0=8+rand(51);y1=y0+1+rand(60-y0)
  value=rand(2)
  for y in range(y0,y1):mask[y*80+x0:y*80+x1]=bytes([value])*(x1-x0)
 if trial%3==0:
  x0=10+rand(35);y0=15+rand(20);size=5+rand(12)
  for y in range(y0,y0+size):
   for x in range(x0,x0+size):mask[y*80+x]=int(y in (y0,y0+size-1) or x in (x0,x0+size-1))
  if trial%2:mask[y0*80+x0]=0
 if trial%5==0:
  y=8+rand(52);known[y*80:(y+1)*80]=bytes(80)
 if trial%7==0:known[1000]=0
 rows=bytearray(sum(1 for x in mask[y*80:(y+1)*80] if x) for y in range(60))
 info=dict(width=80,height=60,floor=8,edge_mask=mask,known_region=known,row_counts=rows)
 ref=fast_topology.FastTopologyGuard(config);candidate=fast_topology.FastTopologyGuard(config)
 fast_topology._K230_ACCEL=None;expected=dict(ref.check(info))
 fast_topology._K230_ACCEL=k230_accel;actual=dict(candidate.check(info))
 if expected!=actual:raise ValueError((trial,expected,actual))
print('DEVICE_TOPOLOGY_EQUIVALENCE cases=500 mismatches=0 native_bitmap=True')
print('PROFILE_DONE')
