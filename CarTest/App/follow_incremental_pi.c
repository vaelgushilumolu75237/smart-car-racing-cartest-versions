#include "follow_incremental_pi.h"
#include "speed_observer.h"

#include <string.h>

#define Q16_ONE INT64_C(65536)

static int64_t clamp_i64(int64_t value, int64_t low, int64_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static uint16_t approach_by_step(uint16_t current, uint16_t target,
                                 uint16_t step_permille)
{
    if (current < target) {
        const uint16_t delta = (uint16_t)(target - current);
        return (uint16_t)(current +
            (delta > step_permille ? step_permille : delta));
    }
    if (current > target) {
        const uint16_t delta = (uint16_t)(current - target);
        return (uint16_t)(current -
            (delta > step_permille ? step_permille : delta));
    }
    return current;
}

static int32_t nominal_to_target_millirps(uint16_t nominal_permille)
{
    return (int32_t)(((uint32_t)CAR_FOLLOW_INCREMENTAL_PI_TARGET_MILLIRPS *
                      nominal_permille) / 500u);
}

static uint8_t valid_input(const car_follow_speed_pi_input_t *input)
{
    return input != 0 && input->speed_sample_valid != 0u &&
           input->sample_age_ms <= CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS &&
           input->sample_elapsed_ms != 0u &&
           input->sample_elapsed_ms <= CAR_SPEED_OBSERVER_MAX_GAP_MS &&
           input->measured_left_millirps >= 0 &&
           input->measured_right_millirps >= 0 &&
           input->nominal_left_permille <=
               CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE &&
           input->nominal_right_permille <=
               CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE &&
           input->applied_left_permille <=
               CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE &&
           input->applied_right_permille <=
               CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE;
}

static void seed_axis(car_follow_incremental_pi_axis_t *axis,
                      uint16_t applied_permille,
                      uint16_t feedforward_permille,
                      int32_t target_millirps,
                      int32_t measured_millirps)
{
    if (axis == 0) {
        return;
    }
    axis->accumulator_q16 = (int64_t)applied_permille * Q16_ONE;
    axis->previous_error_millirps = target_millirps - measured_millirps;
    axis->previous_feedforward_permille = feedforward_permille;
    axis->previous_request_permille = applied_permille;
    axis->initialized = 1u;
    axis->saturated = 0u;
}

static uint16_t update_axis(car_follow_incremental_pi_axis_t *axis,
                            uint16_t nominal_permille,
                            uint16_t applied_permille,
                            int32_t target_millirps,
                            int32_t measured_millirps,
                            uint32_t elapsed_ms)
{
    const int64_t cap_q16 =
        (int64_t)CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE * Q16_ONE;
    const int32_t error = target_millirps - measured_millirps;
    const int64_t error_delta = (int64_t)error -
                                (int64_t)axis->previous_error_millirps;
    const int64_t delta_feedforward_q16 =
        ((int64_t)nominal_permille -
         axis->previous_feedforward_permille) * Q16_ONE;
    const int64_t proportional_q16 =
        (int64_t)CAR_FOLLOW_INCREMENTAL_PI_KP_Q16 * error_delta;
    const int64_t integral_delta_q16 =
        ((int64_t)CAR_FOLLOW_INCREMENTAL_PI_KI_Q16 * error * elapsed_ms) /
        1000;
    const int64_t tracking_q16 =
        ((int64_t)((int32_t)applied_permille -
                   (int32_t)axis->previous_request_permille) * Q16_ONE *
         elapsed_ms) / CAR_FOLLOW_INCREMENTAL_PI_TRACKING_TIME_MS;
    const int64_t raw_without_integral_q16 =
        axis->accumulator_q16 + delta_feedforward_q16 +
        proportional_q16 + tracking_q16;
    const int64_t raw_with_integral_q16 =
        raw_without_integral_q16 + integral_delta_q16;
    int64_t raw_q16 = raw_with_integral_q16;
    uint8_t saturated =
        (raw_with_integral_q16 > cap_q16 ||
         (raw_with_integral_q16 >= cap_q16 && error > 0) ||
         raw_with_integral_q16 < 0) ? 1u : 0u;

    /* The output accumulator itself is bounded, so allow integration to fill
       the remaining rail headroom. Once the stored output is already at a
       rail, suppress only an outward integral step. */
    if ((axis->accumulator_q16 >= cap_q16 && error > 0 &&
         integral_delta_q16 > 0 && raw_without_integral_q16 >= cap_q16) ||
        (axis->accumulator_q16 <= 0 && error < 0 &&
         integral_delta_q16 < 0 && raw_without_integral_q16 <= 0)) {
        raw_q16 = raw_without_integral_q16;
    }
    axis->accumulator_q16 = clamp_i64(raw_q16, 0, cap_q16);
    axis->previous_error_millirps = error;
    axis->previous_feedforward_permille = nominal_permille;
    axis->previous_request_permille =
        (uint16_t)(axis->accumulator_q16 / Q16_ONE);
    axis->saturated = saturated;
    return axis->previous_request_permille;
}

static void invalidate_state(car_follow_incremental_pi_state_t *state)
{
    car_follow_incremental_pi_reset(state);
    state->status = CAR_FOLLOW_SPEED_PI_STATUS_INVALID;
}

void car_follow_incremental_pi_reset(car_follow_incremental_pi_state_t *state)
{
    if (state != 0) {
        (void)memset(state, 0, sizeof(*state));
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_STOPPED;
    }
}

uint32_t car_follow_incremental_pi_update(
    car_follow_incremental_pi_state_t *state,
    const car_follow_speed_pi_input_t *input)
{
    uint8_t sample_is_new;
    int32_t target_left;
    int32_t target_right;

    if (state == 0) {
        return CAR_FOLLOW_SPEED_PI_STATUS_INVALID;
    }
    if (input == 0) {
        invalidate_state(state);
        return state->status;
    }
    if (input->enabled == 0u ||
        CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED == 0) {
        car_follow_incremental_pi_reset(state);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_DISABLED;
        return state->status;
    }
    if (input->vision_hold != 0u) {
        state->hold_seen = 1u;
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_HOLD;
        return state->status;
    }
    if (input->follow_running == 0u || input->startup_qualified == 0u) {
        car_follow_incremental_pi_reset(state);
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }
    if (input->actual_motor_outputs == 0u || input->servo_ready == 0u) {
        if (state->armed != 0u) {
            invalidate_state(state);
            return state->status;
        }
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }

    if (valid_input(input) == 0u) {
        if (state->armed == 0u) {
            state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
            return state->status;
        }
        invalidate_state(state);
        return state->status;
    }

    /* Rebase to what the output path actually applied when leaving hold.
       The held sample is discarded and a later fresh sample seeds history. */
    if (state->hold_seen != 0u) {
        if (state->armed == 0u) {
            car_follow_incremental_pi_reset(state);
            state->status = CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
            return state->status;
        }
        target_left = nominal_to_target_millirps(
            input->nominal_left_permille);
        target_right = nominal_to_target_millirps(
            input->nominal_right_permille);
        seed_axis(&state->left, input->applied_left_permille,
                  input->nominal_left_permille, target_left,
                  input->measured_left_millirps);
        seed_axis(&state->right, input->applied_right_permille,
                  input->nominal_right_permille, target_right,
                  input->measured_right_millirps);
        state->proposal_left_permille = input->applied_left_permille;
        state->proposal_right_permille = input->applied_right_permille;
        state->target_left_millirps = target_left;
        state->target_right_millirps = target_right;
        state->last_sample_ms = input->sample_ms;
        state->have_sample = 1u;
        state->hold_seen = 0u;
        state->saturated_left = 0u;
        state->saturated_right = 0u;
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_RUNNING;
        return state->status;
    }

    sample_is_new = input->new_speed_sample != 0u &&
        (state->have_sample == 0u || input->sample_ms != state->last_sample_ms);
    if (sample_is_new == 0u) {
        state->status = state->armed != 0u
            ? CAR_FOLLOW_SPEED_PI_STATUS_RUNNING
            : CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY;
        return state->status;
    }

    target_left = nominal_to_target_millirps(input->nominal_left_permille);
    target_right = nominal_to_target_millirps(input->nominal_right_permille);
    if (state->armed == 0u) {
        seed_axis(&state->left, input->applied_left_permille,
                  input->nominal_left_permille, target_left,
                  input->measured_left_millirps);
        seed_axis(&state->right, input->applied_right_permille,
                  input->nominal_right_permille, target_right,
                  input->measured_right_millirps);
        state->proposal_left_permille = input->applied_left_permille;
        state->proposal_right_permille = input->applied_right_permille;
        state->target_left_millirps = target_left;
        state->target_right_millirps = target_right;
        state->last_sample_ms = input->sample_ms;
        state->have_sample = 1u;
        state->armed = 1u;
        state->saturated_left = 0u;
        state->saturated_right = 0u;
        state->status = CAR_FOLLOW_SPEED_PI_STATUS_RUNNING;
        return state->status;
    }

    state->proposal_left_permille = update_axis(
        &state->left, input->nominal_left_permille,
        input->applied_left_permille, target_left,
        input->measured_left_millirps, input->sample_elapsed_ms);
    state->proposal_right_permille = update_axis(
        &state->right, input->nominal_right_permille,
        input->applied_right_permille, target_right,
        input->measured_right_millirps, input->sample_elapsed_ms);
    state->target_left_millirps = target_left;
    state->target_right_millirps = target_right;
    state->last_sample_ms = input->sample_ms;
    state->have_sample = 1u;
    state->armed = 1u;
    state->saturated_left = state->left.saturated;
    state->saturated_right = state->right.saturated;
    state->status = CAR_FOLLOW_SPEED_PI_STATUS_RUNNING;
    return state->status;
}

uint8_t car_follow_incremental_pi_ramp(
    const car_follow_incremental_pi_state_t *state,
    uint8_t ramp_due,
    uint16_t step_permille,
    uint16_t applied_left_permille,
    uint16_t applied_right_permille,
    uint16_t *command_left_permille,
    uint16_t *command_right_permille)
{
    if (state == 0 || command_left_permille == 0 ||
        command_right_permille == 0 || state->armed == 0u ||
        state->status != CAR_FOLLOW_SPEED_PI_STATUS_RUNNING ||
        state->proposal_left_permille >
            CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE ||
        state->proposal_right_permille >
            CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE ||
        applied_left_permille >
            CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE ||
        applied_right_permille >
            CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE ||
        (ramp_due != 0u && step_permille == 0u)) {
        return 0u;
    }
    *command_left_permille = ramp_due != 0u
        ? approach_by_step(applied_left_permille,
                           state->proposal_left_permille, step_permille)
        : applied_left_permille;
    *command_right_permille = ramp_due != 0u
        ? approach_by_step(applied_right_permille,
                           state->proposal_right_permille, step_permille)
        : applied_right_permille;
    return 1u;
}
