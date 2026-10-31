"""Decode actual camera mirror-case V3 packets and apply the real H743 mapping.

Only pure parser and extracted production mapping run here. HAL/timer hardware,
physical servo direction and the real ring video are not exercised.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def run(zig, output):
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/"openmv/tests"))
    sys.path.insert(0,str(root/"tests/host"))
    import test_ring_joint_route as camera
    from test_follow_output_path import extract_function
    case=camera.CalibratedRingRouteTests("test_mirrored_calibrated_routes_reach_actual_main_v3_uart")
    case.test_mirrored_calibrated_routes_reach_actual_main_v3_uart()
    packets=list(case.wire_frames)
    # Exit confirmation uses the actual current-anchor tracker output, then
    # the production route converter and weighted error, for both directions.
    import test_ring_exit_current_anchor as exit_anchor
    for side in (0, 1):
        fixture = exit_anchor.CurrentExitAnchorTests()
        h, args = fixture.fixture(side)
        for _ in range(h.tracker.exit_drop_frames):
            result = h.frame(**args)
            assert result['target_valid']
            assert h.tracker.commit_transmitted_route(h.now, True, True)
        assert result['state_id'] == exit_anchor.te.STATE_EXIT_CONFIRM
        row = result['target_rows']
        error = camera.converted_error(side, row[1], row[2])
        assert error < 0 if side == 0 else error > 0
        packets.append(camera.vision_protocol.pack_frame_v3(200+side, True, 99,
                                                          error, 0, profile_ack=3))
    import test_ring_exit_single as exit_single
    for side, shift in ((0, -8), (1, 7)):
        packets.append(exit_single.SingleExitTests().uart_packet(side, shift))
    for error in (-400,0,400):
        packets.append(camera.vision_protocol.pack_frame_v3(100+error%100,True,99,error,0,
                                                       profile_ack=3))
    source=(root/"App/commissioning.c").read_text(encoding="utf-8-sig")
    functions="\n\n".join(extract_function(source,n)[0] for n in
                             ("clamp_error","follow_servo_target"))
    prefix='''#include <cstdio>
#include "commissioning.h"
#include "car_calibration.h"
#include "vision_protocol.h"
'''
    body=r'''
int main() {
    car_config_t c={};
    c.servo_center_us=CARTEST_SERVO_CENTER_US;
    c.follow_servo_left_us=CARTEST_SERVO_LEFT_US;
    c.follow_servo_right_us=CARTEST_SERVO_RIGHT_US;
    c.follow_kp_permille=CARTEST_STEERING_GAIN_PERMILLE;
    unsigned value; unsigned cases=0; unsigned failures=0;
    for (;;) {
        uint8_t bytes[VP_FRAME_SIZE];
        if (std::scanf("%x",&value)!=1) break;
        bytes[0]=(uint8_t)value;
        for (unsigned i=1;i<VP_FRAME_SIZE;++i) {
            if(std::scanf("%x",&value)!=1)return 2;
            bytes[i]=(uint8_t)value;
        }
        vp_parser p; vp_frame frame={};vp_parser_init(&p);unsigned accepted=0;
        for(auto byte:bytes)accepted+=vp_parser_feed(&p,byte,&frame);
        if(accepted!=1 || frame.version!=VP_VERSION_V3 || frame.profile_ack!=3 || !(frame.flags&VP_FLAG_VALID))return 3;
        const uint16_t target=follow_servo_target(&c,frame.lateral_error);
        if((frame.lateral_error<0 && target>=c.servo_center_us) ||
           (frame.lateral_error>0 && target<=c.servo_center_us) ||
           (frame.lateral_error==0 && target!=c.servo_center_us))++failures;
        std::printf("error=%d target_us=%u\n",frame.lateral_error,target);++cases;
    }
    std::printf("RING_DIRECTION_BRIDGE cases=%u failures=%u (host parser/mapping only)\n",cases,failures);
    return failures || cases!=9;
}
'''
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/"camera-v3-packets.json").write_text(json.dumps([bytes(x).hex() for x in packets],indent=2))
    cpp=output/"ring_direction.cpp";cpp.write_text(prefix+functions+body)
    exe=output/"ring_direction.exe"
    result=subprocess.run([str(Path(zig).resolve()),"c++","-std=c++17","-I",str(root/"Inc"),
                           str(cpp),str(root/"App/vision_protocol.cpp"),"-o",str(exe)],capture_output=True,text=True)
    (output/"compile.log").write_text(result.stdout+result.stderr)
    if result.returncode:print(result.stderr);return result.returncode
    data="\n".join(" ".join(f"{b:02x}" for b in x) for x in packets)+"\n"
    result=subprocess.run([str(exe)],input=data,capture_output=True,text=True)
    (output/"run.log").write_text(result.stdout+result.stderr);print(result.stdout)
    return result.returncode

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--zig",required=True);p.add_argument("--output-dir",required=True)
    a=p.parse_args();raise SystemExit(run(a.zig,a.output_dir))
