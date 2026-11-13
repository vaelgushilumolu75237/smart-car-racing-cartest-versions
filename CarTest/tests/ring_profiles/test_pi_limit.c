#include <assert.h>
#include <stdio.h>
#include "follow_incremental_pi.h"
#include "gyro_ring.h"
static car_follow_speed_pi_input_t input(void) {
    car_follow_speed_pi_input_t in = {0};
    in.enabled=1; in.follow_running=1; in.startup_qualified=1;
    in.actual_motor_outputs=1; in.servo_ready=1; in.speed_sample_valid=1;
    in.new_speed_sample=1; in.sample_ms=100; in.sample_elapsed_ms=20;
    in.measured_left_millirps=3000; in.measured_right_millirps=3000;
    in.nominal_left_permille=500; in.nominal_right_permille=500;
    in.applied_left_permille=400; in.applied_right_permille=400;
    in.target_override_valid=1;
    in.target_left_millirps=CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS;
    in.target_right_millirps=CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS;
    return in;
}
int main(void) {
    car_follow_incremental_pi_state_t state;
    car_follow_speed_pi_state_t normal;
    car_follow_speed_pi_input_t in=input();
    uint16_t left,right;
    car_follow_incremental_pi_reset(&state);
    assert(CAR_FOLLOW_INCREMENTAL_PI_TARGET_MILLIRPS==(CARTEST_RING_TUNE==4?7600:5600));
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    assert(state.target_left_millirps==in.target_left_millirps);
    assert(state.target_right_millirps==in.target_right_millirps);
    for (unsigned i=0;i<50;i++) {
        in.sample_ms+=20;
        assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
        assert(state.proposal_left_permille<=1000 && state.proposal_right_permille<=1000);
        assert(car_follow_incremental_pi_ramp(&state,1,40,400,400,&left,&right));
        assert(left<=440 && right<=440);
    }
    /* Left/right Ackermann target pairs remain accepted. */
    in.target_right_millirps/=2; in.sample_ms+=20;
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    in.target_left_millirps=in.target_right_millirps;
    in.target_right_millirps=CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS;in.sample_ms+=20;
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    /* Real Ackermann ordinary straight/bend endpoints and ramp remain bounded. */
    car_ackermann_control_state_t ack;
    car_ackermann_control_output_t out;
    car_ackermann_control_reset(&ack);
    assert(car_ackermann_control_update(&ack,0,0,0,1530,1530,1100,2500,&out));
    assert(out.left_target_millirps==EXPECTED_STRAIGHT);
    assert(car_ackermann_control_update(&ack,10,2,1000,1530,1530,1100,2500,&out));
    assert(car_ackermann_control_update(&ack,210,2,1000,1530,1530,1100,2500,&out));
    assert(out.left_target_millirps==EXPECTED_BEND);
    assert(car_ackermann_control_update(&ack,220,0,0,1530,1530,1100,2500,&out));
    assert(car_ackermann_control_update(&ack,1020,0,0,1530,1530,1100,2500,&out));
    assert(out.left_target_millirps==EXPECTED_STRAIGHT);
    in.target_left_millirps=out.left_target_millirps;
    in.target_right_millirps=out.right_target_millirps;in.sample_ms+=20;
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    in.sample_age_ms=101;
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
    car_follow_incremental_pi_reset(&state);in=input();
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    in.sample_ms+=20;in.target_left_millirps=7601;
    assert(car_follow_incremental_pi_update(&state,&in)==CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
    in=input();car_follow_speed_pi_reset(&normal);
    assert(car_follow_speed_pi_update(&normal,&in)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    in.target_right_millirps=7601;in.sample_ms+=20;
    assert(car_follow_speed_pi_update(&normal,&in)==CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
    puts("ring speed to real incremental/normal PI, caps, ramp and stale guard: PASS");
    return 0;
}
