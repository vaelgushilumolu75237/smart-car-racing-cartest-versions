#include <assert.h>
#include <stdio.h>
#include "ackermann_control.h"
#include "reverse_control.h"
#include "gyro_ring.h"

static void check_speed_ramp(void)
{
    car_ackermann_control_state_t state;
    car_ackermann_control_output_t out;
    const unsigned bend = CARTEST_PREVIEW95_BEND110 ? 4840u : 4400u;
    car_ackermann_control_reset(&state);
    assert(CARTEST_EFFECTIVE_SLOWEST_MILLIRPS == 4400u); /* bypass unchanged */
    assert(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS == 5600u);
    assert(CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS == bend);
    assert(CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS == 2800u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 100000);
    assert(CAR_GYRO_RING_ENTRY_STRAIGHT_MS == 1100u);
    assert(CAR_GYRO_RING_TARGET_PROGRESS_MDEG == 340000);
    assert(CAR_GYRO_RING_TURN_SERVO_PERMILLE == 1875u);
    assert(car_ackermann_control_update(&state, 0, 0, 0, 1530, 1530, 1100, 2500, &out));
    assert(out.left_target_millirps == 5600 && out.right_target_millirps == 5600);
    assert(car_ackermann_control_update(&state, 10, 2, 1000, 2000, 1530, 1100, 2500, &out));
    assert(out.fastest_target_millirps == 5600);
    assert(car_ackermann_control_update(&state, 110, 2, 1000, 2000, 1530, 1100, 2500, &out));
    assert(out.fastest_target_millirps == (5600 + (int)bend) / 2);
    assert(car_ackermann_control_update(&state, 210, 2, 1000, 2000, 1530, 1100, 2500, &out));
    assert(out.fastest_target_millirps == (int)bend);
    assert(out.left_target_millirps == (int)bend && out.right_target_millirps < (int)bend);
    assert(car_ackermann_control_update(&state, 220, 3, 0, 1530, 1530, 1100, 2500, &out));
    assert(car_ackermann_control_update(&state, 1020, 0, 0, 1530, 1530, 1100, 2500, &out));
    assert(out.fastest_target_millirps == 5600);
}

static void check_production_profile_wire(void)
{
    car_reverse_control_tx_t tx;
    unsigned count = 0;
    unsigned i;
    unsigned found = 0;
    unsigned char wire[256];
    unsigned char byte;
    car_reverse_control_init(&tx, 0);
    car_reverse_control_set_profile_state(&tx, 3, 1, 42, 1234);
    for (unsigned t = 0; t < 150; t++) {
        while (car_reverse_control_next_byte_paired(&tx, t, 0, &byte)) {
            assert(count < sizeof wire);
            wire[count++] = byte;
        }
    }
    for (i = 0; i + 20 <= count; i++) {
        if (wire[i] != 0xa6 || wire[i+1] != 0x5a || wire[i+2] != 2) continue;
        assert(wire[i+3] == 2 && wire[i+6] == 3);
        assert((wire[i+7] & CAR_REVERSE_PROFILE_PREVIEW95) ==
               (CARTEST_PREVIEW95_BEND110 ? CAR_REVERSE_PROFILE_PREVIEW95 : 0));
        assert((wire[i+7] & 7u) == 7u);
        assert(car_reverse_control_crc16(wire+i, 18) == (wire[i+18] | (wire[i+19] << 8)));
        found++;
    }
    assert(found >= 1);
}
int main(void)
{
    check_speed_ramp();
    check_production_profile_wire();
    puts("preview profile speed/ramp, ring invariants and production wire passed");
    return 0;
}
