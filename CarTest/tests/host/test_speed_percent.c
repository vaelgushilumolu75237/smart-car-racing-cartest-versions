#include "ackermann_control.h"
#include "ackermann_snapshot.h"
#include "follow_incremental_pi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    car_ackermann_control_state_t s;
    car_ackermann_control_output_t o;
    const int hi=CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS*CARTEST_SPEED_PERCENT/100;
    const int lo=CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS*CARTEST_SPEED_PERCENT/100;
    assert(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS==hi);
    assert(CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS==lo);
    assert(CAR_FOLLOW_SPEED_PI_MAX_TARGET_MILLIRPS==hi);
    assert(CAR_FOLLOW_INCREMENTAL_PI_TARGET_MILLIRPS==hi);
    assert(CAR_ACKERMANN_REQUEST_RAMP_DOWN_MS==200);
    assert(CAR_ACKERMANN_REQUEST_RAMP_UP_MS==800);
    for (int pulse=1100;pulse<=2500;pulse+=20) {
        car_ackermann_control_reset(&s);
        assert(car_ackermann_control_update(&s,0,0,0,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==hi);
        assert(o.left_target_millirps>0 && o.left_target_millirps<=hi);
        assert(o.right_target_millirps>0 && o.right_target_millirps<=hi);
        assert(car_ackermann_control_update(&s,10,2,1000,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==hi);
        assert(car_ackermann_control_update(&s,110,2,1000,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==(hi+lo)/2);
        assert(car_ackermann_control_update(&s,210,2,1000,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==lo);
        assert(car_ackermann_control_update(&s,220,3,0,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==lo);
        assert(car_ackermann_control_update(&s,1020,0,0,pulse,1530,1100,2500,&o));
        assert(o.fastest_target_millirps==hi);
    }
    car_follow_incremental_pi_state_t pi;
    car_follow_speed_pi_input_t in={0};
    car_follow_incremental_pi_reset(&pi);
    in.enabled=in.follow_running=in.startup_qualified=in.actual_motor_outputs=1;
    in.servo_ready=in.speed_sample_valid=in.new_speed_sample=1;
    in.sample_ms=10;in.sample_elapsed_ms=10;
    in.nominal_left_permille=in.nominal_right_permille=500;
    in.applied_left_permille=in.applied_right_permille=500;
    in.target_override_valid=1;in.target_left_millirps=hi;in.target_right_millirps=hi*3/4;
    assert(car_follow_incremental_pi_update(&pi,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    assert(pi.target_left_millirps==hi && pi.target_right_millirps==hi*3/4);
    in.vision_hold=1;in.new_speed_sample=0;
    assert(car_follow_incremental_pi_update(&pi,&in)==CAR_FOLLOW_SPEED_PI_STATUS_HOLD);
    assert(pi.target_left_millirps==hi);
    _Alignas(32) unsigned char bytes[64]={0};
    car_ackermann_snapshot_values_t snap={0};
    snap.ratio_q15=32768;snap.servo_pulse_us=1530;snap.fastest_target_millirps=hi;
    snap.left_target_millirps=hi;snap.right_target_millirps=hi;
    car_ackermann_snapshot_publish(bytes,&snap);
    assert(car_ackermann_snapshot_validate(bytes));
    unsigned char prior[64];memcpy(prior,bytes,64);
    snap.fastest_target_millirps=CAR_ACKERMANN_SNAPSHOT_MAX_TARGET_MILLIRPS+1;
    car_ackermann_snapshot_publish(bytes,&snap);
    assert(memcmp(bytes,prior,64)==0);
    printf("PASS profile=%d percent=%d straight=%d slowdown=%d PI=accepted hold=preserved snapshot=valid\n",CARTEST_TRACK_PROFILE,CARTEST_SPEED_PERCENT,hi,lo);
    return 0;
}
