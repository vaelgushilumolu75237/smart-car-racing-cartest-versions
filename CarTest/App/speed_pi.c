#include "speed_pi.h"

#include <limits.h>
#include <string.h>

#define Q16_ONE INT64_C(65536)

static int64_t clamp_i64(int64_t value, int64_t low, int64_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint16_t clamp_output(int64_t raw_q16, uint16_t cap,
                             uint8_t *saturated)
{
    int64_t maximum = (int64_t)cap * Q16_ONE;
    int64_t bounded = raw_q16;
    if (bounded < 0) {
        bounded = 0;
        if (saturated != 0) *saturated = 1u;
    } else if (bounded > maximum) {
        bounded = maximum;
        if (saturated != 0) *saturated = 1u;
    }
    return (uint16_t)(bounded / Q16_ONE);
}

static uint8_t valid_parameters(const car_speed_pi_config_t *config,
                                const car_speed_pi_input_t *input)
{
    if (config == 0 || input == 0 || config->configured == 0u ||
        config->output_cap_permille == 0u ||
        config->output_cap_permille > CAR_SPEED_PI_MAX_OUTPUT_PERMILLE ||
        config->kp_q16 > CAR_SPEED_PI_MAX_GAIN_Q16 ||
        config->ki_q16 > CAR_SPEED_PI_MAX_GAIN_Q16 ||
        config->tracking_time_ms > CAR_SPEED_PI_MAX_TRACKING_TIME_MS ||
        input->speed_sample_valid == 0u || input->dt_ms == 0u ||
        input->dt_ms > CAR_SPEED_PI_MAX_DT_MS ||
        input->target_millirps <= 0 ||
        (uint32_t)input->target_millirps >
            CAR_SPEED_PI_MAX_SPEED_MILLIRPS ||
        input->measured_millirps < 0 ||
        (uint32_t)input->measured_millirps >
            CAR_SPEED_PI_MAX_SPEED_MILLIRPS ||
        input->feedforward_permille > config->output_cap_permille ||
        (input->downstream_applied_valid != 0u &&
         input->downstream_applied_permille > config->output_cap_permille)) {
        return 0u;
    }
    return 1u;
}

void car_speed_pi_reset(car_speed_pi_state_t *state)
{
    if (state != 0) {
        (void)memset(state, 0, sizeof(*state));
        state->status = CAR_SPEED_PI_STATUS_STOPPED;
    }
}

uint32_t car_speed_pi_update(car_speed_pi_state_t *state,
                             const car_speed_pi_config_t *config,
                             const car_speed_pi_input_t *input,
                             car_speed_pi_output_t *output)
{
    int64_t error;
    int64_t proportional_q16;
    int64_t candidate_integral_q16;
    int64_t integral_delta_q16;
    int64_t cap_q16;
    int64_t raw_q16;
    uint8_t saturated = 0u;
    uint32_t status;

    if (state == 0 || output == 0) {
        return CAR_SPEED_PI_STATUS_INVALID;
    }
    output->requested_permille = 0u;
    output->motor_permission = 0u;
    output->saturated = 0u;
    output->status = CAR_SPEED_PI_STATUS_INVALID;
    if (input == 0) {
        car_speed_pi_reset(state);
        state->status = CAR_SPEED_PI_STATUS_INVALID;
        return state->status;
    }

    if (input->lifecycle == CAR_SPEED_PI_LIFECYCLE_STOPPED) {
        car_speed_pi_reset(state);
        state->status = CAR_SPEED_PI_STATUS_STOPPED;
        status = state->status;
        goto done;
    }
    if (input->lifecycle == CAR_SPEED_PI_LIFECYCLE_STARTUP) {
        car_speed_pi_reset(state);
        state->status = CAR_SPEED_PI_STATUS_STARTUP;
        status = state->status;
        goto done;
    }
    if (input->lifecycle == CAR_SPEED_PI_LIFECYCLE_HOLD) {
        state->requested_permille = 0u;
        state->motor_permission = 0u;
        state->saturated = 0u;
        state->previous_proposal_valid = 0u;
        state->status = CAR_SPEED_PI_STATUS_HOLD;
        status = state->status;
        goto done;
    }
    if (input->lifecycle != CAR_SPEED_PI_LIFECYCLE_RUNNING) {
        car_speed_pi_reset(state);
        state->status = CAR_SPEED_PI_STATUS_INVALID;
        status = state->status;
        goto done;
    }
    if (valid_parameters(config, input) == 0u) {
        const uint32_t invalid_status =
            config != 0 && config->configured == 0u
                ? CAR_SPEED_PI_STATUS_DISABLED : CAR_SPEED_PI_STATUS_INVALID;
        car_speed_pi_reset(state);
        state->status = invalid_status;
        status = state->status;
        goto done;
    }

    cap_q16 = (int64_t)config->output_cap_permille * Q16_ONE;
    state->integral_q16 = clamp_i64(state->integral_q16, -cap_q16, cap_q16);
    if (input->downstream_applied_valid != 0u &&
        state->previous_proposal_valid != 0u &&
        state->motor_permission != 0u) {
        int64_t tracking_q16 =
            ((int64_t)input->downstream_applied_permille -
             state->requested_permille) * Q16_ONE;
        if (config->tracking_time_ms != 0u) {
            uint32_t tracking_dt_ms = input->dt_ms;
            if (tracking_dt_ms > config->tracking_time_ms) {
                tracking_dt_ms = config->tracking_time_ms;
            }
            tracking_q16 = tracking_q16 * tracking_dt_ms /
                           config->tracking_time_ms;
        }
        state->integral_q16 = clamp_i64(state->integral_q16 + tracking_q16,
                                        -cap_q16, cap_q16);
    } else {
        state->previous_proposal_valid = 0u;
    }

    error = (int64_t)input->target_millirps - input->measured_millirps;
    proportional_q16 = (int64_t)config->kp_q16 * error;
    integral_delta_q16 =
        ((int64_t)config->ki_q16 * error * input->dt_ms) / 1000;
    candidate_integral_q16 = clamp_i64(
        state->integral_q16 + integral_delta_q16, -cap_q16, cap_q16);
    raw_q16 = (int64_t)input->feedforward_permille * Q16_ONE +
              proportional_q16 + candidate_integral_q16;

    /* Conditional integration: do not wind farther into either rail. */
    if ((raw_q16 > cap_q16 && error > 0) || (raw_q16 < 0 && error < 0)) {
        candidate_integral_q16 = state->integral_q16;
        raw_q16 = (int64_t)input->feedforward_permille * Q16_ONE +
                  proportional_q16 + candidate_integral_q16;
    } else {
        state->integral_q16 = candidate_integral_q16;
    }
    state->requested_permille = clamp_output(raw_q16,
                                               config->output_cap_permille,
                                               &saturated);

    state->motor_permission = 1u;
    state->saturated = saturated;
    state->previous_proposal_valid = 1u;
    state->status = CAR_SPEED_PI_STATUS_RUNNING;
    status = state->status;

done:
    output->requested_permille = state->requested_permille;
    output->motor_permission = state->motor_permission;
    output->saturated = state->saturated;
    output->status = status;
    return status;
}
