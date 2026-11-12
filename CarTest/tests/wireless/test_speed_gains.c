#include <assert.h>
#include <stdio.h>
#include "follow_incremental_pi.h"
#include "wireless_runtime.h"
static wireless_settings_t settings;
const wireless_settings_t *car_wireless_settings(void) { return &settings; }
int main(void)
{
    unsigned p,i;
    for(p=500;p<=1500;p+=500) for(i=500;i<=1500;i+=500) {
        car_follow_incremental_pi_state_t state;
        car_follow_speed_pi_input_t input={0};
        int64_t expected, original;
        settings.speed_p_gain_permille=(uint16_t)p;
        settings.speed_i_gain_permille=(uint16_t)i;
        input.enabled=input.follow_running=input.startup_qualified=1;
        input.actual_motor_outputs=input.servo_ready=input.speed_sample_valid=1;
        input.new_speed_sample=input.target_override_valid=1;
        input.target_left_millirps=input.target_right_millirps=2000;
        input.measured_left_millirps=input.measured_right_millirps=2000;
        input.nominal_left_permille=input.nominal_right_permille=300;
        input.applied_left_permille=input.applied_right_permille=300;
        input.sample_ms=20;input.sample_elapsed_ms=20;
        car_follow_incremental_pi_reset(&state);
        assert(car_follow_incremental_pi_update(&state,&input)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
        input.sample_ms=40;
        input.measured_left_millirps=input.measured_right_millirps=1900;
        assert(car_follow_incremental_pi_update(&state,&input)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
        expected=INT64_C(300)*65536+
            ((int64_t)CAR_FOLLOW_INCREMENTAL_PI_KP_Q16*p/1000)*100+
            ((int64_t)CAR_FOLLOW_INCREMENTAL_PI_KI_Q16*i/1000)*100*20/1000;
        assert(state.left.accumulator_q16==expected && state.right.accumulator_q16==expected);
        original=state.left.accumulator_q16;
        settings.speed_p_gain_permille=settings.speed_i_gain_permille=1000;
        assert(car_follow_incremental_pi_update(&state,&input)==CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
        assert(state.left.accumulator_q16==original);
    }
    puts("wireless speed PI gains: 9 actual output combinations and duplicate-sample guards PASS");
    return 0;
}
