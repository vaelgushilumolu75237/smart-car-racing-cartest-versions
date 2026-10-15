#include "follow_output.h"

static uint16_t clamp_u16(uint32_t value, uint16_t maximum)
{
    return value > maximum ? maximum : (uint16_t)value;
}

static uint16_t approach_by_step(uint16_t current, uint16_t target,
                                 uint16_t step_permille)
{
    if (current < target) {
        uint16_t delta = (uint16_t)(target - current);
        return (uint16_t)(current +
            (delta > step_permille ? step_permille : delta));
    }
    if (current > target) {
        uint16_t delta = (uint16_t)(current - target);
        return (uint16_t)(current -
            (delta > step_permille ? step_permille : delta));
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

uint8_t car_follow_output_slope_assist_request(
    const car_follow_slope_assist_gate_t *gate)
{
#if CARTEST_FOLLOW_TURN_PROFILE_COMPILED
    int32_t steering_error;
    if (gate == 0 || gate->follow_running == 0u ||
        gate->manual_run_request == 0u ||
        gate->follow_session_active == 0u ||
        gate->road_drive_allowed == 0u || gate->fault_or_stop != 0u ||
        gate->trusted_slope_hint == 0u || gate->vision_valid == 0u ||
        gate->vision_fresh == 0u || gate->startup_qualified == 0u ||
        gate->servo_output_active == 0u || gate->servo_frame_ready == 0u ||
        gate->motor_outputs_active == 0u ||
        gate->applied_outputs_nonzero == 0u) {
        return 0u;
    }
    steering_error = gate->trusted_vision_error_norm;
    if (steering_error < 0) {
        steering_error = -steering_error;
    }
    return steering_error <= 100 ? 1u : 0u;
#else
    (void)gate;
    return 0u;
#endif
}

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
    uint16_t *right_command_permille)
{
    uint16_t next_left_target;
    uint16_t next_right_target;
    uint16_t effective_base_target;
    uint16_t steering_u = 0u;
    uint16_t ramp_step_permille;
    uint32_t delta = 0u;
    uint8_t servo_ready;
    uint8_t slope_assist_active = 0u;

    if (state == 0 || left_command_permille == 0 ||
        right_command_permille == 0 ||
        base_target_permille > CAR_FOLLOW_BASE_TARGET_PERMILLE ||
        outer_differential_permille > CAR_FOLLOW_DIFFERENTIAL_MAX_PERMILLE ||
        inner_differential_permille > CAR_FOLLOW_DIFFERENTIAL_MAX_PERMILLE ||
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

    effective_base_target = base_target_permille;
#if CARTEST_FOLLOW_TURN_PROFILE_COMPILED
    if (slope_assist_requested != 0u &&
        state->startup_qualified != 0u && servo_ready != 0u &&
        motor_outputs_active != 0u && left_applied_permille != 0u &&
        right_applied_permille != 0u &&
        left_applied_permille <= CAR_FOLLOW_HARD_MAX_PERMILLE &&
        right_applied_permille <= CAR_FOLLOW_HARD_MAX_PERMILLE &&
        steering_u <= CAR_FOLLOW_SLOPE_ASSIST_STRAIGHT_U_MAX_PERMILLE) {
        effective_base_target = CAR_FOLLOW_SLOPE_ASSIST_TARGET_PERMILLE;
        slope_assist_active = 1u;
    }
#else
    (void)slope_assist_requested;
#endif

    next_left_target = effective_base_target;
    next_right_target = effective_base_target;
    state->outer_delta_permille = 0u;
    state->inner_delta_permille = 0u;
    if (state->startup_qualified != 0u && slope_assist_active == 0u) {
        delta = ((uint32_t)outer_differential_permille * steering_u) / 1000u;
        if (servo_applied_us > servo_center_us) {
            /* Right turn: left is the outer side and receives more duty. */
            next_left_target = (uint16_t)(effective_base_target + delta);
            state->outer_delta_permille = (uint16_t)delta;
            delta = ((uint32_t)inner_differential_permille * steering_u) /
                    1000u;
            if (delta > effective_base_target) {
                return 0u;
            }
            next_right_target = (uint16_t)(effective_base_target - delta);
            state->inner_delta_permille = (uint16_t)delta;
        } else if (servo_applied_us < servo_center_us) {
            /* Left turn mirrors the right-turn output. */
            delta = ((uint32_t)inner_differential_permille * steering_u) /
                    1000u;
            if (delta > effective_base_target) {
                return 0u;
            }
            next_left_target = (uint16_t)(effective_base_target - delta);
            state->inner_delta_permille = (uint16_t)delta;
            delta = ((uint32_t)outer_differential_permille * steering_u) /
                    1000u;
            next_right_target = (uint16_t)(effective_base_target + delta);
            state->outer_delta_permille = (uint16_t)delta;
        }
    }
    /* Apply the configured and fixed ceilings after selecting either the
       ordinary differential or the optional slope-assist target. */
    if (next_left_target > CAR_FOLLOW_NORMAL_MAX_PERMILLE ||
        next_right_target > CAR_FOLLOW_NORMAL_MAX_PERMILLE ||
        next_left_target > CAR_FOLLOW_HARD_MAX_PERMILLE ||
        next_right_target > CAR_FOLLOW_HARD_MAX_PERMILLE) {
        return 0u;
    }

    state->target_left_permille = next_left_target;
    state->target_right_permille = next_right_target;
    state->differential_u_permille = steering_u;
    state->slope_assist_active = slope_assist_active;
    if (state->startup_qualified != 0u) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_DIFFERENTIAL;
    } else if (effective_base_target < CAR_FOLLOW_BASE_TARGET_PERMILLE) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_BASE_RAMP;
    } else if (servo_ready == 0u) {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_WAIT_SERVO_FRAME;
    } else {
        state->phase = CAR_FOLLOW_OUTPUT_PHASE_WAIT_BASE_DUTY;
    }
    ramp_step_permille = state->startup_qualified != 0u
        ? CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE
        : CAR_FOLLOW_OUTPUT_STARTUP_RAMP_STEP_PERMILLE;

    if (state->have_ramp_time == 0u) {
        state->last_ramp_ms = now_ms;
        state->have_ramp_time = 1u;
        state->command_left_permille = left_applied_permille;
        state->command_right_permille = right_applied_permille;
    } else if ((uint32_t)(now_ms - state->last_ramp_ms) >=
               CAR_FOLLOW_OUTPUT_RAMP_PERIOD_MS) {
        state->last_ramp_ms = now_ms;
        state->command_left_permille = approach_by_step(
            left_applied_permille, next_left_target, ramp_step_permille);
        state->command_right_permille = approach_by_step(
            right_applied_permille, next_right_target, ramp_step_permille);
    } else {
        state->command_left_permille = left_applied_permille;
        state->command_right_permille = right_applied_permille;
    }

    *left_command_permille = state->command_left_permille;
    *right_command_permille = state->command_right_permille;
    return 1u;
}

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
    uint16_t *right_command_permille)
{
    return car_follow_output_update_with_slope_assist(
        state, now_ms, base_target_permille, outer_differential_permille,
        inner_differential_permille, servo_applied_us, servo_center_us,
        servo_left_endpoint_us, servo_right_endpoint_us, servo_output_active,
        servo_frame_ready, motor_outputs_active, left_applied_permille,
        right_applied_permille, 0u, left_command_permille,
        right_command_permille);
}
