#ifndef CARTEST_FOLLOW_OUTPUT_H
#define CARTEST_FOLLOW_OUTPUT_H

#include <stdint.h>

#ifndef CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE
#define CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE 1000u
#endif
#ifndef CARTEST_FOLLOW_HARD_LIMIT_PERMILLE
#define CARTEST_FOLLOW_HARD_LIMIT_PERMILLE 1000u
#endif
#if CARTEST_FOLLOW_HARD_LIMIT_PERMILLE != 1000u
#error "Follow hard output limit is fixed at 1000 permille."
#endif
#ifndef CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE
#define CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE 10u
#endif
#ifndef CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE
#define CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE 40u
#endif
#ifndef CARTEST_FOLLOW_TURN_PROFILE_COMPILED
#define CARTEST_FOLLOW_TURN_PROFILE_COMPILED 0
#endif
#ifndef CARTEST_VISION_HOLD_RAMP_MS
#define CARTEST_VISION_HOLD_RAMP_MS 300u
#endif
#if CARTEST_FOLLOW_TURN_PROFILE_COMPILED && \
    (CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE != 1000u)
#error "FollowTurn requires the 1000-permille normal output cap."
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_FOLLOW_BASE_TARGET_PERMILLE = 500u,
    CAR_FOLLOW_SLOPE_ASSIST_TARGET_PERMILLE = 700u,
    CAR_FOLLOW_SLOPE_ASSIST_STRAIGHT_U_MAX_PERMILLE = 100u,
    CAR_FOLLOW_NORMAL_MAX_PERMILLE = CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE,
    CAR_FOLLOW_HARD_MAX_PERMILLE = CARTEST_FOLLOW_HARD_LIMIT_PERMILLE,
    CAR_FOLLOW_DIFFERENTIAL_MAX_PERMILLE = 200u,
    CAR_FOLLOW_OUTPUT_STARTUP_RAMP_STEP_PERMILLE =
        CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE,
    CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE =
        CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE,
    CAR_FOLLOW_OUTPUT_RAMP_PERIOD_MS = 10u,
    CAR_FOLLOW_VISION_HOLD_RAMP_MS = CARTEST_VISION_HOLD_RAMP_MS,
    CAR_FOLLOW_VISION_HOLD_TARGET_PERMILLE = 400u,
    CAR_FOLLOW_VISION_HOLD_MIN_TARGET_PERMILLE = 300u,
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
    uint16_t hold_entry_left_permille;
    uint16_t hold_entry_right_permille;
    uint32_t hold_entry_generation;
    uint32_t last_ramp_ms;
    uint32_t hold_started_ms;
    uint8_t have_ramp_time;
    uint8_t startup_qualified;
    /* Latched command state; hold preserves it with the frozen output. */
    uint8_t slope_assist_active;
    uint8_t phase;
} car_follow_output_state_t;

/* Main-loop eligibility snapshot. This is transient state, not a log/wire ABI. */
typedef struct {
    uint8_t follow_running;
    uint8_t manual_run_request;
    uint8_t follow_session_active;
    uint8_t road_drive_allowed;
    uint8_t fault_or_stop;
    uint8_t trusted_slope_hint;
    uint8_t vision_valid;
    uint8_t vision_fresh;
    uint8_t startup_qualified;
    uint8_t servo_output_active;
    uint8_t servo_frame_ready;
    uint8_t motor_outputs_active;
    uint8_t applied_outputs_nonzero;
    int16_t trusted_vision_error_norm;
} car_follow_slope_assist_gate_t;

uint8_t car_follow_output_slope_assist_request(
    const car_follow_slope_assist_gate_t *gate);

void car_follow_output_reset(car_follow_output_state_t *state);

/* Ramp each exact software-applied side duty toward its fixed hold target.
   hold_started_ms and hold_entry_generation come from the controller entry,
   so delayed service and a new hold immediately after recovery stay distinct. */
void car_follow_output_hold_at(car_follow_output_state_t *state,
                               uint32_t now_ms,
                               uint32_t hold_started_ms,
                               uint32_t hold_entry_generation,
                               uint16_t left_applied_permille,
                               uint16_t right_applied_permille);

/* Convenience form for isolated callers whose hold starts at now_ms. */
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

/* FollowTurn-only overload. The helper independently enforces startup,
   software-servo straightness, profile cap, and the fixed 1000 hard ceiling. */
uint8_t car_follow_output_update_with_slope_assist(
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
    uint8_t slope_assist_requested,
    uint16_t *left_command_permille,
    uint16_t *right_command_permille);

#ifdef __cplusplus
}
#endif

#endif
