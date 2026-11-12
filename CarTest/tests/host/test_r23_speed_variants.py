"""Compile actual wheel-target control at all supported C speed percentages."""
from pathlib import Path
import argparse,subprocess,json

def run(zig,out):
 root=Path(__file__).resolve().parents[2];out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True);zig=str(Path(zig).resolve());reports=[]
 defs=['-DCARTEST_TRACK_PROFILE=3','-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600','-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400']
 previous={}
 for pct in (100,120,130,140):
  exe=out/('speed'+str(pct)+'.exe');cmd=[zig,'cc','-std=c11','-I',str(root/'Inc'),*defs,'-DCARTEST_SPEED_PERCENT='+str(pct),str(root/'tests/host/test_r23_speed_variants.c'),str(root/'App/ackermann_control.c'),'-o',str(exe)]
  r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
  r=subprocess.run([str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
  rows=[list(map(int,x.split(','))) for x in r.stdout.splitlines()]
  if pct==100:previous={(x[0],x[1]):x for x in rows}
  for x in rows:
   base=previous[(x[0],x[1])];assert x[5]==base[5]
   for k in (2,3,4):assert abs(x[k]-base[k]*pct/100)<=1.5
  (out/('speed'+str(pct)+'.log')).write_text(r.stdout);reports.append(dict(percent=pct,cases=len(rows),passed=True))
 # Header guards must reject unsupported scales and non-C speed variants.
 for pct,profile in ((110,3),(120,0),(130,1),(140,2)):
  src=out/'guard.c';src.write_text('#include "car_track_profile.h"\nint main(void){return 0;}\n')
  fastest,slowest=((5600,4400) if profile in (1,3) else (4800,4000))
  r=subprocess.run([zig,'cc','-I',str(root/'Inc'),'-DCARTEST_TRACK_PROFILE='+str(profile),'-DCARTEST_SPEED_PERCENT='+str(pct),'-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS='+str(fastest),'-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS='+str(slowest),'-fsyntax-only',str(src)],capture_output=True,text=True)
  assert r.returncode!=0,(pct,profile)
  (out/('reject-%s-%s.log'%(pct,profile))).write_text(r.stderr)
 (out/'summary.json').write_text(json.dumps(dict(targets=reports,negative_compile_guards=4),indent=2));print(json.dumps(reports))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--zig',required=True);p.add_argument('--output',required=True);a=p.parse_args();run(a.zig,a.output)
