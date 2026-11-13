"""Verify all three actual C ring/control variants without device access."""
import argparse, subprocess, tempfile, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def main():
 p=argparse.ArgumentParser();p.add_argument('--zig',required=True);a=p.parse_args()
 def run(cmd):subprocess.run([str(x) for x in cmd],cwd=ROOT,check=True)
 with tempfile.TemporaryDirectory() as temp:
  probe=Path(temp)/'probe.c'
  probe.write_text('#include "gyro_ring.h"\n#include "ackermann_control.h"\n_Static_assert(CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS==EXPECTED_SPEED,"speed");\n_Static_assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS==EXPECTED_RATE,"rate");\n_Static_assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS==5280,"exit unchanged");\n_Static_assert(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS==EXPECTED_STRAIGHT,"straight");\n_Static_assert(CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS==4400,"bend");\n_Static_assert(CAR_GYRO_RING_ENTRY_STRAIGHT_MS==EXPECTED_ENTRY_MS,"entry delay");\nint main(void){return 0;}')
  for tune,speed,rate,entry_ms in ((0,2800,100000,1100),(3,4200,145000,800),(4,7600,282000,700)):
   flags=['-DCARTEST_TRACK_PROFILE=3','-DCARTEST_SPEED_PERCENT=100','-DCARTEST_GYRO350_ENABLED=1','-DCARTEST_FOLLOW_SPEED_PI_ENABLED=1','-DCARTEST_FOLLOW_INCREMENTAL_PI_ENABLED=1','-DCARTEST_RED_OBSTACLE_ENABLED=1','-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600','-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400',f'-DCARTEST_RING_TUNE={tune}']
   for name,src in [('pi_limit',['App/follow_incremental_pi.c','App/follow_speed_pi.c','App/speed_pi.c','App/ackermann_control.c','tests/ring_profiles/test_pi_limit.c']),('probe',[probe]),('gyro',['App/icm42688.c','App/gyro_ring.c','tests/gyro350/test_gyro350.c']),('authority',['App/commissioning.c','App/gyro350_authority.c','App/gyro_ring.c','App/obstacle_authority.c','App/obstacle_bypass.c','tests/obstacle/test_authority.c']),('obstacle',['App/obstacle_bypass.c','tests/obstacle/test_obstacle.c'])]:
    exe=Path(temp)/f'{name}{tune}.exe'
    run([a.zig,'cc','-std=c11','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-I','Inc',*flags,f'-DEXPECTED_SPEED={speed}',f'-DEXPECTED_RATE={rate}',f'-DEXPECTED_ENTRY_MS={entry_ms}',f'-DEXPECTED_STRAIGHT={7600 if tune==4 else 5600}',*src,'-o',exe]);run([exe])
   print('Ring tune',tune,'PASS',flush=True)
if __name__=='__main__':main()
