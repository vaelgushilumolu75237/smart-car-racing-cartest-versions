#include "ackermann_control.h"
#include "gyro_ring.h"
#include "follow_incremental_pi.h"
#include <assert.h>
#include <stdio.h>
#if CARTEST_WIRELESS_ENABLED
#include "wireless_settings.h"
const wireless_settings_t *car_wireless_settings(void)
{
    static const wireless_settings_t settings = {
        .speed_p_gain_permille = 1000,
        .speed_i_gain_permille = 1000
    };
    return &settings;
}
#endif

static car_ackermann_control_output_t settled(uint16_t request, uint16_t pwm)
{
    car_ackermann_control_state_t state = {0};
    car_ackermann_control_output_t out = {0};
    car_ackermann_control_reset(&state);
    assert(car_ackermann_control_update(&state, 100, request ? 2 : 0,
        request, pwm, 1530, 1100, 2500, &out));
    assert(car_ackermann_control_update(&state, 300, request ? 2 : 0,
        request, pwm, 1530, 1100, 2500, &out));
    return out;
}

int main(void)
{
    car_ackermann_control_output_t straight = settled(0, 1530);
    car_ackermann_control_output_t left = settled(1000, 1100);
    car_ackermann_control_output_t right = settled(1000, 2500);
    car_ackermann_control_output_t partial = settled(500, 1530);
    car_ackermann_control_state_t state = {0};
    car_ackermann_control_output_t out = {0};
    unsigned straight_expected = CARTEST_WIRELESS_ENABLED ? 5600 : 7840;
    unsigned expected = CARTEST_WIRELESS_ENABLED ? 4400 : 6160;
    assert(straight.fastest_target_millirps == (int)straight_expected);
    assert(left.fastest_target_millirps == (int)expected);
    assert(right.fastest_target_millirps == (int)expected);
    assert(left.right_target_millirps == (int)expected);
    assert(right.left_target_millirps == (int)expected);
    assert(left.left_target_millirps == right.right_target_millirps);
    assert(partial.fastest_target_millirps == (int)((straight_expected + expected + 1) / 2));
    car_ackermann_control_reset(&state);
    assert(car_ackermann_control_update(&state, 100, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)straight_expected);
    assert(car_ackermann_control_update(&state, 200, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)((straight_expected + expected + 1) / 2));
    assert(car_ackermann_control_update(&state, 300, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)expected);
    assert(car_ackermann_control_update(&state, 400, 0, 0, 1530, 1530,
        1100, 2500, &out));
    assert(car_ackermann_control_update(&state, 1200, 0, 0, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)straight_expected);
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 7392);
    printf("NEW_A_SPEED_V30 wireless=%u straight=%u turn=%u ring=7392 passed\n",
        (unsigned)CARTEST_WIRELESS_ENABLED, straight_expected, expected);
    {
        car_follow_incremental_pi_state_t pi;
        car_follow_speed_pi_input_t input = {0};
        car_follow_incremental_pi_reset(&pi);
        input.enabled = input.follow_running = input.startup_qualified = 1;
        input.actual_motor_outputs = input.servo_ready = input.speed_sample_valid = 1;
        input.new_speed_sample = input.target_override_valid = 1;
        input.sample_ms = input.sample_elapsed_ms = 20;
        input.measured_left_millirps = input.measured_right_millirps = 1000;
        input.nominal_left_permille = input.nominal_right_permille = 500;
        input.applied_left_permille = input.applied_right_permille = 500;
        input.target_left_millirps = input.target_right_millirps = 7392;
        assert(CAR_FOLLOW_INCREMENTAL_PI_MAX_TARGET_MILLIRPS == 7840);
        assert(car_follow_incremental_pi_update(&pi, &input) == CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
        assert(pi.target_left_millirps == 7392 && pi.target_right_millirps == 7392);
        input.target_left_millirps = 7841;
        input.sample_ms = 40;
        assert(car_follow_incremental_pi_update(&pi, &input) == CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
        puts("RING_PI_V30 independent 7392 target accepted; original 7840 ceiling retained");
    }
    return 0;
}
