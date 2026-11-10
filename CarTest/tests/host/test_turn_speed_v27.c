#include "ackermann_control.h"
#include "gyro_ring.h"
#include <assert.h>
#include <stdio.h>

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
    unsigned expected = CARTEST_WIRELESS_ENABLED ? 4928 : 6160;
    assert(straight.fastest_target_millirps == 7840);
    assert(left.fastest_target_millirps == (int)expected);
    assert(right.fastest_target_millirps == (int)expected);
    assert(left.right_target_millirps == (int)expected);
    assert(right.left_target_millirps == (int)expected);
    assert(left.left_target_millirps == right.right_target_millirps);
    assert(partial.fastest_target_millirps == (int)((7840 + expected) / 2));
    car_ackermann_control_reset(&state);
    assert(car_ackermann_control_update(&state, 100, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == 7840);
    assert(car_ackermann_control_update(&state, 200, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)((7840 + expected) / 2));
    assert(car_ackermann_control_update(&state, 300, 2, 1000, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)expected);
    assert(car_ackermann_control_update(&state, 400, 0, 0, 1530, 1530,
        1100, 2500, &out));
    assert(car_ackermann_control_update(&state, 1200, 0, 0, 1530, 1530,
        1100, 2500, &out));
    assert(out.fastest_target_millirps == 7840);
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 7392);
    printf("TURN_SPEED_V27 wireless=%u straight=7840 turn=%u ring=7392 passed\n",
        (unsigned)CARTEST_WIRELESS_ENABLED, expected);
    return 0;
}
