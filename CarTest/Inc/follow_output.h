#ifndef CARTEST_FOLLOW_OUTPUT_H
#define CARTEST_FOLLOW_OUTPUT_H

#include <stdint.h>

#ifndef CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE
#define CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE 600u
#endif
#ifndef CARTEST_FOLLOW_HARD_LIMIT_PERMILLE
#define CARTEST_FOLLOW_HARD_LIMIT_PERMILLE 700u
#endif
#if CARTEST_FOLLOW_HARD_LIMIT_PERMILLE != 700u
#error "Follow hard output limit is fixed at 700 permille."
#endif
#ifndef CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE
#define CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE 10u
#endif
#ifndef CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE
#define CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE 10u
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_FOLLOW_BASE_TARGET_PERMILLE = 500u,
    CAR_FOLLOW_NORMAL_MAX_PERMILLE = CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE,
    CAR_FOLLOW_HARD_MAX_PERMILLE = 700u,
    CAR_FOLLOW_DIFFERENTIAL_MAX_PERMILLE = 200u,
    CAR_FOLLOW_OUTPUT_STARTUP_RAMP_STEP_PERMILLE =
        CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE,
    CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE =
        CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE,
    CAR_FOLLOW_OUTPUT_RAMP_PERIOD_MS = 10u,
    CAR_FOLLOW_STEERING_GAIN_PERMILLE = 700u
};

typedef enum {
    CAR_FOLLOW_OUTPUT_PHASE_IDLE = 0u,
    CAR_FOLLOW_OUTPUT_PHASE_BASE_RAMP = 1u,
    CAR_FOLLOW_OUTPUT_PHASE_WAIT_SERVO_FRAME = 2u,
    CAR_FOLLOW_OUTPUT_PHASE_WAIT_BASE_DUTY = 3u,
    CAR_FOLLOW_OUTPUT_PHASE_DIFFERENTIAL = 4u,
    CAR_FOLLOW_OUTPUT_PHASE_VISION_HOLD = 5u
} car_follow_output_phase_t;

typedef struct {
    uint16_t target_left_permille;
    uint16_t target_right_permille;
    uint16_t command_left_permille;
    uint16_t command_right_permille;
    uint16_t differential_u_permille;
    uint16_t outer_delta_permille;
    uint16_t inner_delta_permille;
    uint32_t last_ramp_ms;
    uint8_t have_ramp_time;
    uint8_t startup_qualified;
    uint8_t phase;
} car_follow_output_state_t;

void car_follow_output_reset(car_follow_output_state_t *state);

/* Freeze the exact software-applied side duties during a qualifying vision
   hold.  This preserves the startup latch and moves the ramp clock forward. */
void car_follow_output_hold(car_follow_output_state_t *state,
                            uint32_t now_ms,
                            uint16_t left_applied_permille,
                            uint16_t right_applied_permille);

/* Prepare one normal Follow output step from the already submitted servo
   pulse.  Before the startup qualification latch, both sides target the same
   500-permille base ramp.  Once qualified, the selected differential is
   computed from the applied pulse and each side advances at most one
   profile-selected step per 10 ms. */
uint8_t car_follow_output_update(
    car_follow_output_state_t *state,
    uint32_t now_ms,
    uint16_t base_target_permille,
    uint16_t outer_differential_permille,
    uint16_t inner_differential_permille,
    uint16_t servo_applied_us,
    uint16_t servo_center_us,
    uint16_t servo_left_endpoint_us,
    uint16_t servo_right_endpoint_us,
    uint8_t servo_output_active,
    uint8_t servo_frame_ready,
    uint8_t motor_outputs_active,
    uint16_t left_applied_permille,
    uint16_t right_applied_permille,
    uint16_t *left_command_permille,
    uint16_t *right_command_permille);

#ifdef __cplusplus
}
#endif

#endif
