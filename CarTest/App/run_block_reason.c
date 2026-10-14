#include "run_block_reason.h"

car_run_block_reason_t car_run_block_reason_evaluate(
    const car_run_block_reason_input_t *input)
{
    if (input == 0) {
        return CAR_RUN_BLOCK_FAULT;
    }
    if (input->startup_release_seen == 0u) {
        return CAR_RUN_BLOCK_WAIT_STARTUP_RELEASE;
    }
    if (input->fault_present != 0u) {
        return CAR_RUN_BLOCK_FAULT;
    }
    if (input->recovery_pending != 0u) {
        return CAR_RUN_BLOCK_RECOVERY_PENDING;
    }
    if (input->vision_hold_expired != 0u) {
        return CAR_RUN_BLOCK_HOLD_EXPIRED_WAIT_ROUTE;
    }
    if (input->vision_fresh_valid == 0u) {
        return CAR_RUN_BLOCK_NO_FRESH_VALID_VISION;
    }
    if (input->reverse_reset_keep_ready == 0u) {
        return CAR_RUN_BLOCK_REVERSE_RESET_KEEP_GATE;
    }
    if (input->profile_output_permission == 0u) {
        return CAR_RUN_BLOCK_SOFTWARE_PERMISSION_OFF;
    }
    if (input->manual_run_request == 0u) {
        return CAR_RUN_BLOCK_WAIT_PE7_PRESS_EDGE;
    }
    if (input->follow_mode_active == 0u ||
        input->follow_session_active == 0u) {
        return CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD;
    }
    if (input->follow_output_permission == 0u) {
        return CAR_RUN_BLOCK_SOFTWARE_PERMISSION_OFF;
    }
    if (input->servo_stop_pending != 0u) {
        return CAR_RUN_BLOCK_SERVO_STOP_PENDING;
    }
    if (input->servo_output_active == 0u ||
        input->servo_first_frame_ready == 0u) {
        return CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME;
    }
    if (input->motor_left_applied != 0u &&
        input->motor_right_applied != 0u) {
        return CAR_RUN_BLOCK_RUNNING_WITH_MOTOR_OUTPUT;
    }
    return CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD;
}
