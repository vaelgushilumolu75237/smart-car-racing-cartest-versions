#include "follow_speed_pi.h"
#include "speed_observer.h"

#include <string.h>

static const car_speed_pi_config_t speed_pi_config = {
    1u,
    CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE,
    CAR_FOLLOW_SPEED_PI_KP_Q16,
    CAR_FOLLOW_SPEED_PI_KI_Q16,
    CAR_FOLLOW_SPEED_PI_TRACKING_TIME_MS,
    0u
};

static uint16_t approach_by_step(uint16_t current, uint16_t target)
{
    if (current < target) {
        const uint16_t delta = (uint16_t)(target - current);
        return (uint16_t)(current +
            (delta > CAR_FOLLOW_SPEED_PI_RAMP_STEP_PERMILLE
                 ? CAR_FOLLOW_SPEED_PI_RAMP_STEP_PERMILLE : delta));
    }
    if (current > target) {
        const uint16_t delta = (uint16_t)(current - target);
        return (uint16_t)(current -
            (delta > CAR_FOLLOW_SPEED_PI_RAMP_STEP_PERMILLE
                 ? CAR_FOLLOW_SPEED_PI_RAMP_STEP_PERMILLE : delta));
    }
    return current;
}

static uint8_t valid_input(const car_follow_speed_pi_input_t *input)
{
    return input != 0 && input->sample_age_ms <=
               CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS &&
           input->speed_sample_valid != 0u &&
           input->sample_elapsed_ms != 0u &&
           input->sample_elapsed_ms <= CAR_SPEED_OBSERVER_MAX_GAP_MS &&
           input->measured_left_millirps >= 0 &&
           input->measured_right_millirps >= 0 &&
           input->nominal_left_permille <= CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE &&
           input->nominal_right_permille <= CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE &&
           input->applied_left_permille <= CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE &&
           input->applied_right_permille <= CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE &&
           input->target_override_valid <= 1u &&
           (input->target_override_valid == 0u ||
            (input->target_left_millirps >= 0 &&
             input->target_left_millirps <= 4800 &&
             input->target_right_millirps >= 0 &&
             input->target_right_millirps <= 4800));
}

static uint16_t nominal_to_feedforward(uint16_t nominal_permille)
{
    uint32_t value = nominal_permille;
    return value > CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE
        ? CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE : (uint16_t)value;
}

static int32_t nominal_to_target_millirps(uint16_t nominal_permille)
{
    return (int32_t)(((uint32_t)CAR_FOLLOW_SPEED_PI_TARGET_MILLIRPS *
                      nominal_permille) / 500u);
}

static int32_t input_target_millirps(
    const car_follow_speed_pi_input_t *input,
    uint8_t left_wheel)
{
    if (input->target_override_valid != 0u) {
        return left_wheel != 0u ? input->target_left_millirps
                                : input->target_right_millirps;
    }
    return nominal_to_target_millirps(left_wheel != 0u
        ? input->nominal_left_permille : input->nominal_right_permille);
}

static void reset_axis_history(car_speed_pi_state_t *axis)
{
    if (axis != 0) {
        axis->previous_proposal_valid = 0u;
        axis->motor_permission = 0u;
    }
}

static void invalidate_state(car_follow_speed_pi_state_t *state)
{
    car_follow_speed_pi_reset(state);
    state->status = CAR_FOLLOW_SPEED_PI_STATUS_INVALID;
}

void car_follow_speed_pi_reset(car_follow_speed_pi_state_t *state)
{
    if (state != 0) {
        (void)memset(state, 0, sizeof(*state));
        car_speed_pi_reset(&state->left);
        car_speed_pi_reset(&state->right);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_STOPPED;
    }
}

uint32_t car_follow_speed_pi_update(
    car_follow_speed_pi_state_t *state,
    const car_follow_speed_pi_input_t *input)
{
    car_speed_pi_input_t left_input;
    car_speed_pi_input_t right_input;
    car_speed_pi_output_t left_output;
    car_speed_pi_output_t right_output;
    uint8_t sample_is_new;

    if (state == 0) {
        return CAR_FOLLOW_SPEED_PI_STATUS_INVALID;
    }
    if (input == 0) {
        invalidate_state(state);
        return state->status;
    }
    if (input->enabled == 0u || CARTEST_FOLLOW_SPEED_PI_ENABLED == 0) {
        car_follow_speed_pi_reset(state);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_DISABLED;
        return state->status;
    }
    if (input->vision_hold != 0u) {
        if (state->hold_seen == 0u) {
            state->hold_was_armed = state->armed;
            reset_axis_history(&state->left);
            reset_axis_history(&state->right);
        }
        state->hold_seen = 1u;
        state->have_sample = 0u;
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_HOLD;
        return state->status;
    }
    if (input->follow_running == 0u || input->startup_qualified == 0u) {
        car_follow_speed_pi_reset(state);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }
    if (valid_input(input) == 0u) {
        invalidate_state(state);
        return state->status;
    }
    if (input->actual_motor_outputs == 0u || input->servo_ready == 0u) {
        state->status = state->armed != 0u
            ? CAR_FOLLOW_SPEED_PI_STATUS_INVALID
            : CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        if (state->armed != 0u) {
            invalidate_state(state);
        }
        return state->status;
    }

    if (state->armed == 0u && input->new_speed_sample == 0u) {
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }

    if (state->hold_seen != 0u) {
        if (state->hold_was_armed == 0u) {
            car_follow_speed_pi_reset(state);
            state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
            return state->status;
        }
        /* Resume at current applied outputs, and wait for a post-hold sample. */
        reset_axis_history(&state->left);
        reset_axis_history(&state->right);
        state->proposal_left_permille = input->applied_left_permille;
        state->proposal_right_permille = input->applied_right_permille;
        state->last_sample_ms = input->sample_ms;
        state->have_sample = 1u;
        state->hold_seen = 0u;
        state->hold_was_armed = 0u;
        state->armed = 1u;
        state->target_left_millirps = input_target_millirps(input, 1u);
        state->target_right_millirps = input_target_millirps(input, 0u);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_RUNNING;
        return state->status;
    }

    sample_is_new = state->have_sample == 0u ||
                    input->sample_ms != state->last_sample_ms;
    if (sample_is_new == 0u) {
        state->status = state->armed != 0u
            ? CAR_FOLLOW_SPEED_PI_STATUS_RUNNING
            : CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }

    left_input = (car_speed_pi_input_t){
        .lifecycle = CAR_SPEED_PI_LIFECYCLE_RUNNING,
        .speed_sample_valid = input->speed_sample_valid,
        .downstream_applied_valid = state->armed,
        .downstream_applied_permille = input->applied_left_permille,
        .feedforward_permille = nominal_to_feedforward(
            input->nominal_left_permille),
        .dt_ms = input->sample_elapsed_ms,
        .target_millirps = input_target_millirps(input, 1u),
        .measured_millirps = input->measured_left_millirps
    };
    right_input = left_input;
    right_input.downstream_applied_permille = input->applied_right_permille;
    right_input.feedforward_permille = nominal_to_feedforward(
        input->nominal_right_permille);
    right_input.target_millirps = input_target_millirps(input, 0u);
    right_input.measured_millirps = input->measured_right_millirps;

    (void)car_speed_pi_update(&state->left, &speed_pi_config, &left_input,
                              &left_output);
    (void)car_speed_pi_update(&state->right, &speed_pi_config, &right_input,
                              &right_output);
    if (left_output.status != CAR_SPEED_PI_STATUS_RUNNING ||
        right_output.status != CAR_SPEED_PI_STATUS_RUNNING) {
        invalidate_state(state);
        return state->status;
    }

    state->proposal_left_permille = left_output.requested_permille;
    state->proposal_right_permille = right_output.requested_permille;
    state->target_left_millirps = left_input.target_millirps;
    state->target_right_millirps = right_input.target_millirps;
    state->saturated_left = left_output.saturated;
    state->saturated_right = right_output.saturated;
    state->last_sample_ms = input->sample_ms;
    state->have_sample = 1u;
    state->armed = 1u;
    state->status = CAR_FOLLOW_SPEED_PI_STATUS_RUNNING;
    return state->status;
}

uint8_t car_follow_speed_pi_ramp(
    const car_follow_speed_pi_state_t *state,
    uint8_t ramp_due,
    uint16_t applied_left_permille,
    uint16_t applied_right_permille,
    uint16_t *command_left_permille,
    uint16_t *command_right_permille)
{
    if (state == 0 || command_left_permille == 0 ||
        command_right_permille == 0 || state->armed == 0u ||
        state->status != CAR_FOLLOW_SPEED_PI_STATUS_RUNNING ||
        state->proposal_left_permille >
            CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE ||
        state->proposal_right_permille >
            CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE ||
        applied_left_permille > CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE ||
        applied_right_permille > CAR_FOLLOW_SPEED_PI_MAX_OUTPUT_PERMILLE) {
        return 0u;
    }
    *command_left_permille = ramp_due != 0u
        ? approach_by_step(applied_left_permille,
                           state->proposal_left_permille)
        : applied_left_permille;
    *command_right_permille = ramp_due != 0u
        ? approach_by_step(applied_right_permille,
                           state->proposal_right_permille)
        : applied_right_permille;
    return 1u;
}
