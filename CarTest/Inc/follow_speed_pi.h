#ifndef CARTEST_FOLLOW_SPEED_PI_H
#define CARTEST_FOLLOW_SPEED_PI_H

#include <stdint.h>

#include "speed_pi.h"

#ifndef CARTEST_FOLLOW_SPEED_PI_ENABLED
#define CARTEST_FOLLOW_SPEED_PI_ENABLED 0
#endif
#ifndef CARTEST_FOLLOW_SPEED_PI_OUTPUT_CAP_PERMILLE
#define CARTEST_FOLLOW_SPEED_PI_OUTPUT_CAP_PERMILLE 1000u
#endif
#if CARTEST_FOLLOW_SPEED_PI_OUTPUT_CAP_PERMILLE != 1000u
#error "Follow speed PI output cap is fixed at 1000 permille."
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_FOLLOW_SPEED_PI_TARGET_MILLIRPS = 2400u,
    CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE =
        CARTEST_FOLLOW_SPEED_PI_OUTPUT_CAP_PERMILLE,
    CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS = 100u,
    CAR_FOLLOW_SPEED_PI_TRACKING_TIME_MS = 200u,
    CAR_FOLLOW_SPEED_PI_RAMP_STEP_PERMILLE = 40u,
    /* Provisional starting gains. They require vehicle tuning. */
    CAR_FOLLOW_SPEED_PI_KP_Q16 = 13107u,
    CAR_FOLLOW_SPEED_PI_KI_Q16 = 26214u,
    CAR_FOLLOW_SPEED_PI_STATUS_STOPPED = 0u,
    CAR_FOLLOW_SPEED_PI_STATUS_DISABLED = 1u,
    CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY = 2u,
    CAR_FOLLOW_SPEED_PI_STATUS_RUNNING = 3u,
    CAR_FOLLOW_SPEED_PI_STATUS_HOLD = 4u,
    CAR_FOLLOW_SPEED_PI_STATUS_INVALID = 5u
};

typedef struct {
    uint8_t enabled;
    uint8_t follow_running;
    uint8_t vision_hold;
    uint8_t startup_qualified;
    uint8_t actual_motor_outputs;
    uint8_t servo_ready;
    uint8_t speed_sample_valid;
    uint8_t new_speed_sample;
    uint32_t sample_ms;
    uint32_t sample_age_ms;
    uint32_t sample_elapsed_ms;
    int32_t measured_left_millirps;
    int32_t measured_right_millirps;
    uint16_t nominal_left_permille;
    uint16_t nominal_right_permille;
    uint16_t applied_left_permille;
    uint16_t applied_right_permille;
    /* Optional wheel-speed target pair. When false, retain the legacy target
       derived from the nominal duty pair. */
    uint8_t target_override_valid;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
} car_follow_speed_pi_input_t;

typedef struct {
    car_speed_pi_state_t left;
    car_speed_pi_state_t right;
    uint32_t last_sample_ms;
    uint32_t status;
    int32_t target_left_millirps;
    int32_t target_right_millirps;
    uint16_t proposal_left_permille;
    uint16_t proposal_right_permille;
    uint8_t armed;
    uint8_t have_sample;
    uint8_t hold_seen;
    uint8_t hold_was_armed;
    uint8_t saturated_left;
    uint8_t saturated_right;
} car_follow_speed_pi_state_t;

void car_follow_speed_pi_reset(car_follow_speed_pi_state_t *state);

/* Computes PI only once per new observer sample. Output is a proposal only. */
uint32_t car_follow_speed_pi_update(
    car_follow_speed_pi_state_t *state,
    const car_follow_speed_pi_input_t *input);

/* Apply one output-ramp step from the software-applied value to the stored PI
   proposal. No step occurs until the existing 10 ms ramp is due. */
uint8_t car_follow_speed_pi_ramp(
    const car_follow_speed_pi_state_t *state,
    uint8_t ramp_due,
    uint16_t applied_left_permille,
    uint16_t applied_right_permille,
    uint16_t *command_left_permille,
    uint16_t *command_right_permille);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_FOLLOW_SPEED_PI_H */
