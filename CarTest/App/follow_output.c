#include "follow_output.h"

static uint16_t clamp_u16(uint32_t value, uint16_t maximum)
{
    return value > maximum ? maximum : (uint16_t)value;
}

static uint16_t approach_by_step(uint16_t current, uint16_t target)
{
    if (current < target) {
        uint16_t delta = (uint16_t)(target - current);
        return (uint16_t)(current +
            (delta > CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE
                 ? CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE : delta));
    }
    if (current > target) {
        uint16_t delta = (uint16_t)(current - target);
        return (uint16_t)(current -
            (delta > CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE
                 ? CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE : delta));
    }
    return current;
}

static uint8_t steering_amount_permille(uint16_t servo_applied_us,
                                        uint16_t servo_center_us,
                                        uint16_t servo_left_endpoint_us,
                                        uint16_t servo_right_endpoint_us,
                                        uint16_t *amount_permille)
{
    uint32_t delta_us;
    uint32_t span_us;
    uint64_t numerator;
    uint32_t denominator;

    if (amount_permille == 0 ||
        servo_left_endpoint_us >= servo_center_us ||
        servo_right_endpoint_us <= servo_center_us ||
        servo_applied_us < servo_left_endpoint_us ||
        servo_applied_us > servo_right_endpoint_us) {
        return 0u;
    }
    if (servo_applied_us == servo_center_us) {
        *amount_permille = 0u;
        return 1u;
    }

    if (servo_applied_us < servo_center_us) {
        delta_us = (uint32_t)servo_center_us - servo_applied_us;
        span_us = (uint32_t)servo_center_us - servo_left_endpoint_us;
    } else {
        delta_us = (uint32_t)servo_applied_us - servo_center_us;
        span_us = (uint32_t)servo_right_endpoint_us - servo_center_us;
    }
    denominator = span_us * CAR_FOLLOW_STEERING_GAIN_PERMILLE;
    if (denominator == 0u) {
        return 0u;
    }
    numerator = (uint64_t)delta_us * 1000000u;
    *amount_permille = clamp_u16(
        (uint32_t)(numerator / denominator), 1000u);
    return 1u;
}

void car_follow_output_reset(car_follow_output_state_t *state)
{
    if (state != 0) {
        *state = (car_follow_output_state_t){0};
    }
}

void car_follow_output_hold(car_follow_output_state_t *state,
                            uint32_t now_ms,
                            uint16_t left_applied_permille,
                            uint16_t right_applied_permille)
{
    if (state == 0) {
        return;
    }
    state->target_left_permille = left_applied_permille;
    state->target_right_permille = right_applied_permille;
    state->command_left_permille = left_applied_permille;
    state->command_right_permille = right_applied_permille;
    state->last_ramp_ms = now_ms;
    state->have_ramp_time = 1u;
    state->phase = CAR_FOLLOW_OUTPUT_PHASE_VISION_HOLD;
}

uint8_t car_follow_output_update(
    car_follow_output_state_t *state,
    uint32_t now_ms,
    uint16_t base_target_permille,
    uint16_t differential_permille,
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
    uint16_t *right_command_permille)
{
    uint16_t next_left_target;
    uint16_t next_right_target;
    uint16_t steering_u = 0u;
    uint32_t delta = 0u;
    uint8_t servo_ready;

    if (state == 0 || left_command_permille == 0 ||
        right_command_permille == 0 ||
        base_target_permille > CAR_FOLLOW_BASE_TARGET_PERMILLE ||
        differential_permille > CAR_FOLLOW_DIFFERENTIAL_MAX_PERMILLE ||
        servo_left_endpoint_us >= servo_center_us ||
        servo_right_endpoint_us <= servo_center_us) {
        return 0u;
    }

    servo_ready = servo_output_active != 0u && servo_frame_ready != 0u;
    if (servo_ready != 0u &&
        steering_amount_permille(servo_applied_us, servo_center_us,
                                 servo_left_endpoint_us,
                                 servo_right_endpoint_us,
                                 &steering_u) == 0u) {
        return 0u;
    }

    if (state->startup_qualified == 0u &&
        base_target_permille == CAR_FOLLOW_BASE_TARGET_PERMILLE &&
        servo_ready != 0u && motor_outputs_active != 0u &&
        left_applied_permille == CAR_FOLLOW_BASE_TARGET_PERMILLE &&
        right_applied_permille == CAR_FOLLOW_BASE_TARGET_PERMILLE) {
        state->startup_qualified = 1u;
    }

    next_left_target = base_target_permille;
    next_right_target = base_target_permille;
    if (state->startup_qualified != 0u) {
        delta = ((uint32_t)differential_permille * steering_u) / 1000u;
        if (servo_applied_us > servo_center_us) {
            /* Right turn: left is the outer side and receives more duty. */
            next_left_target = (uint16_t)(base_target_permille + delta);
            next_right_target = (uint16_t)(base_target_permille - delta);
        } else if (servo_applied_us < servo_center_us) {
            /* Left turn mirrors the right-turn output. */
            next_left_target = (uint16_t)(base_target_permille - delta);
            next_right_target = (uint16_t)(base_target_permille + delta);
        }
        if (next_left_target > CAR_FOLLOW_NORMAL_MAX_PERMILLE ||
            next_right_target > CAR_FOLLOW_NORMAL_MAX_PERMILLE ||
            next_left_target > CAR_FOLLOW_HARD_MAX_PERMILLE ||
            next_right_target > CAR_FOLLOW_HARD_MAX_PERMILLE) {
            return 0u;
        }
    }

    state->target_left_permille = next_left_target;
    state->target_right_permille = next_right_target;
    state->differential_u_permille = steering_u;
    if (state->startup_qualified != 0u) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_DIFFERENTIAL;
    } else if (base_target_permille < CAR_FOLLOW_BASE_TARGET_PERMILLE) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_BASE_RAMP;
    } else if (servo_ready == 0u) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_WAIT_SERVO_FRAME;
    } else {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_WAIT_BASE_DUTY;
    }

    if (state->have_ramp_time == 0u) {
        state->last_ramp_ms = now_ms;
        state->have_ramp_time = 1u;
        state->command_left_permille = left_applied_permille;
        state->command_right_permille = right_applied_permille;
    } else if ((uint32_t)(now_ms - state->last_ramp_ms) >=
               CAR_FOLLOW_OUTPUT_RAMP_PERIOD_MS) {
        state->last_ramp_ms = now_ms;
        state->command_left_permille = approach_by_step(
            left_applied_permille, next_left_target);
        state->command_right_permille = approach_by_step(
            right_applied_permille, next_right_target);
    } else {
        state->command_left_permille = left_applied_permille;
        state->command_right_permille = right_applied_permille;
    }

    *left_command_permille = state->command_left_permille;
    *right_command_permille = state->command_right_permille;
    return 1u;
}
