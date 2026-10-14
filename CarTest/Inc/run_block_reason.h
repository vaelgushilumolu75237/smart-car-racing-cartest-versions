#ifndef CARTEST_RUN_BLOCK_REASON_H
#define CARTEST_RUN_BLOCK_REASON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAR_RUN_BLOCK_RUNNING_WITH_MOTOR_OUTPUT = 0,
    CAR_RUN_BLOCK_WAIT_STARTUP_RELEASE = 1,
    CAR_RUN_BLOCK_FAULT = 2,
    CAR_RUN_BLOCK_NO_FRESH_VALID_VISION = 3,
    CAR_RUN_BLOCK_RECOVERY_PENDING = 4,
    CAR_RUN_BLOCK_REVERSE_RESET_KEEP_GATE = 5,
    CAR_RUN_BLOCK_WAIT_PE7_PRESS_EDGE = 6,
    CAR_RUN_BLOCK_SOFTWARE_PERMISSION_OFF = 7,
    CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME = 8,
    CAR_RUN_BLOCK_SERVO_STOP_PENDING = 9,
    CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD = 10,
    CAR_RUN_BLOCK_HOLD_EXPIRED_WAIT_ROUTE = 11
} car_run_block_reason_t;

typedef struct {
    uint8_t startup_release_seen;
    uint8_t fault_present;
    uint8_t recovery_pending;
    uint8_t vision_fresh_valid;
    uint8_t reverse_reset_keep_ready;
    uint8_t profile_output_permission;
    uint8_t follow_mode_active;
    uint8_t follow_session_active;
    uint8_t follow_output_permission;
    uint8_t servo_stop_pending;
    uint8_t servo_output_active;
    uint8_t servo_first_frame_ready;
    uint8_t motor_left_applied;
    uint8_t motor_right_applied;
    uint8_t manual_run_request;
    uint8_t vision_hold_active;
    uint8_t vision_hold_expired;
} car_run_block_reason_input_t;

/* Read-only classification of the first currently observed output gate. */
car_run_block_reason_t car_run_block_reason_evaluate(
    const car_run_block_reason_input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_RUN_BLOCK_REASON_H */
