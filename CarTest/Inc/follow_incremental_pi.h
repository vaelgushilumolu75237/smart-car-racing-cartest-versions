#ifndef CARTEST_FOLLOW_INCREMENTAL_PI_H
#define CARTEST_FOLLOW_INCREMENTAL_PI_H

#include <stdint.h>

#include "follow_speed_pi.h"

#ifndef CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
#define CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_FOLLOW_INCREMENTAL_PI_TARGET_MILLIRPS =
        CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS,
    CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE = 1000u,
    CAR_FOLLOW_INCREMENTAL_PI_TRACKING_TIME_MS = 200u,
    CAR_FOLLOW_INCREMENTAL_PI_KP_Q16 = 13107u,
    CAR_FOLLOW_INCREMENTAL_PI_KI_Q16 = 26214u
};

typedef struct {
    int64_t accumulator_q16;
    int32_t previous_error_millirps;
    uint16_t previous_feedforward_permille;
    uint16_t previous_request_permille;
    uint8_t initialized;
    uint8_t saturated;
} car_follow_incremental_pi_axis_t;

typedef struct {
    car_follow_incremental_pi_axis_t left;
    car_follow_incremental_pi_axis_t right;
    uint32_t last_sample_ms;
    uint32_t status;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
    uint16_t proposal_left_permille;
    uint16_t proposal_right_permille;
    uint8_t armed;
    uint8_t have_sample;
    uint8_t hold_seen;
    uint8_t saturated_left;
    uint8_t saturated_right;
} car_follow_incremental_pi_state_t;

void car_follow_incremental_pi_reset(car_follow_incremental_pi_state_t *state);

/* Pure, per-wheel incremental PI proposal. Each distinct observer sample is
   consumed at most once. The target pair is normalized together so the larger
   nominal side is at most the selected profile's straight target. Hold freezes
   the accumulator; the first post-hold sample seeds from actual output without
   a control step. */
uint32_t car_follow_incremental_pi_update(
    car_follow_incremental_pi_state_t *state,
    const car_follow_speed_pi_input_t *input);

/* Slew a stored PI proposal from the actual applied duty. */
uint8_t car_follow_incremental_pi_ramp(
    const car_follow_incremental_pi_state_t *state,
    uint8_t ramp_due,
    uint16_t step_permille,
    uint16_t applied_left_permille,
    uint16_t applied_right_permille,
    uint16_t *command_left_permille,
    uint16_t *command_right_permille);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_FOLLOW_INCREMENTAL_PI_H */
