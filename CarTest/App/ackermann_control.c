#include "ackermann_control.h"

#include <string.h>

/* tan(degrees) * 1,000,000 at integer-degree increments. Linear
   interpolation keeps the firmware independent of libm and deterministic. */
static const uint32_t tangent_micro[21] = {
    0u, 17455u, 34921u, 52408u, 69927u, 87489u, 105104u,
    122785u, 140541u, 158384u, 176327u, 194380u, 212557u,
    230868u, 249328u, 267949u, 286745u, 305731u, 324920u,
    344328u, 363970u
};

static uint32_t abs_i32(int32_t value)
{
    return (uint32_t)(value < 0 ? -value : value);
}

static uint32_t tangent_micro_for_cdeg(uint16_t angle_cdeg)
{
    uint16_t whole_degrees = (uint16_t)(angle_cdeg / 100u);
    uint16_t fractional_cdeg = (uint16_t)(angle_cdeg % 100u);
    uint32_t lower;
    uint32_t upper;

    if (whole_degrees >= 20u) {
        return tangent_micro[20];
    }
    lower = tangent_micro[whole_degrees];
    upper = tangent_micro[whole_degrees + 1u];
    return lower + (uint32_t)(((uint64_t)(upper - lower) *
                               fractional_cdeg + 50u) / 100u);
}

static uint8_t ratio_for_angle(int16_t angle_cdeg, uint16_t *ratio_q15)
{
    uint32_t angle_abs;
    uint32_t tangent_value_micro;
    uint64_t numerator;
    uint64_t denominator;

    if (ratio_q15 == 0) {
        return 0u;
    }
    angle_abs = abs_i32(angle_cdeg);
    if (angle_abs > CAR_ACKERMANN_MAX_STEER_CDEG) {
        return 0u;
    }
    if (angle_abs == 0u) {
        *ratio_q15 = CAR_ACKERMANN_RATIO_ONE_Q15;
        return 1u;
    }

    tangent_value_micro = tangent_micro_for_cdeg((uint16_t)angle_abs);
    if (tangent_value_micro == 0u) {
        return 0u;
    }
    /* Algebraically cancel the radius division:
       (R - t/2) / (R + t/2) = (2L - t*tan) / (2L + t*tan).
       Keeping the full fixed-point precision avoids radius-rounding loss. */
    denominator = (uint64_t)2u * CAR_ACKERMANN_WHEELBASE_TENTH_MM *
                      1000000u +
                  (uint64_t)CAR_ACKERMANN_TRACK_TENTH_MM *
                      tangent_value_micro;
    numerator = (uint64_t)2u * CAR_ACKERMANN_WHEELBASE_TENTH_MM *
                    1000000u -
                (uint64_t)CAR_ACKERMANN_TRACK_TENTH_MM *
                    tangent_value_micro;
    if (numerator == 0u || denominator == 0u) {
        return 0u;
    }
    *ratio_q15 = (uint16_t)((numerator * CAR_ACKERMANN_RATIO_ONE_Q15 +
                             denominator / 2u) / denominator);
    return *ratio_q15 != 0u ? 1u : 0u;
}

static uint16_t ramp_value_at(const car_ackermann_control_state_t *state,
                              uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t delta;

    if (state->ramp_active == 0u || state->ramp_duration_ms == 0u) {
        return state->fastest_target_millirps;
    }
    elapsed_ms = (uint32_t)(now_ms - state->ramp_started_ms);
    if (elapsed_ms >= state->ramp_duration_ms) {
        return state->ramp_target_millirps;
    }
    if (state->ramp_target_millirps < state->ramp_start_millirps) {
        delta = (uint32_t)(state->ramp_start_millirps -
                           state->ramp_target_millirps);
        return (uint16_t)(state->ramp_start_millirps -
            (delta * elapsed_ms) / state->ramp_duration_ms);
    }
    delta = (uint32_t)(state->ramp_target_millirps -
                       state->ramp_start_millirps);
    return (uint16_t)(state->ramp_start_millirps +
        (delta * elapsed_ms) / state->ramp_duration_ms);
}

static uint16_t requested_fastest_target(uint16_t request_permille)
{
    uint32_t reduction =
        ((uint32_t)(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS -
                    CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS) *
         request_permille + 500u) / CAR_ACKERMANN_REQUEST_MAX_PERMILLE;
    return (uint16_t)(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS - reduction);
}

void car_ackermann_control_reset(car_ackermann_control_state_t *state)
{
    if (state == 0) {
        return;
    }
    (void)memset(state, 0, sizeof(*state));
    state->ramp_start_millirps =
        CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS;
    state->ramp_target_millirps =
        CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS;
    state->fastest_target_millirps =
        CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS;
    state->initialized = 1u;
}

uint8_t car_ackermann_model_geometry(uint16_t applied_pwm_us,
                                     uint16_t center_pwm_us,
                                     uint16_t left_endpoint_us,
                                     uint16_t right_endpoint_us,
                                     int16_t *model_angle_cdeg,
                                     uint16_t *ratio_q15)
{
    uint32_t delta_us;
    uint32_t span_us;
    uint32_t angle_abs_cdeg = 0u;
    int16_t signed_angle;
    uint16_t ratio;

    if (model_angle_cdeg == 0 || ratio_q15 == 0 ||
        left_endpoint_us >= center_pwm_us ||
        center_pwm_us >= right_endpoint_us ||
        applied_pwm_us < left_endpoint_us ||
        applied_pwm_us > right_endpoint_us) {
        return 0u;
    }
    if (applied_pwm_us < center_pwm_us) {
        delta_us = (uint32_t)center_pwm_us - applied_pwm_us;
        span_us = (uint32_t)center_pwm_us - left_endpoint_us;
        angle_abs_cdeg = (delta_us * CAR_ACKERMANN_MAX_STEER_CDEG +
                          span_us / 2u) / span_us;
        signed_angle = (int16_t)-(int16_t)angle_abs_cdeg;
    } else if (applied_pwm_us > center_pwm_us) {
        delta_us = (uint32_t)applied_pwm_us - center_pwm_us;
        span_us = (uint32_t)right_endpoint_us - center_pwm_us;
        angle_abs_cdeg = (delta_us * CAR_ACKERMANN_MAX_STEER_CDEG +
                          span_us / 2u) / span_us;
        signed_angle = (int16_t)angle_abs_cdeg;
    } else {
        signed_angle = 0;
    }
    if (angle_abs_cdeg > CAR_ACKERMANN_MAX_STEER_CDEG &&
        (applied_pwm_us < center_pwm_us || applied_pwm_us > center_pwm_us)) {
        return 0u;
    }
    if (ratio_for_angle(signed_angle, &ratio) == 0u) {
        return 0u;
    }
    *model_angle_cdeg = signed_angle;
    *ratio_q15 = ratio;
    return 1u;
}

uint8_t car_ackermann_control_update(
    car_ackermann_control_state_t *state,
    uint32_t now_ms,
    uint8_t curve_phase,
    uint16_t slowdown_request_permille,
    uint16_t applied_pwm_us,
    uint16_t center_pwm_us,
    uint16_t left_endpoint_us,
    uint16_t right_endpoint_us,
    car_ackermann_control_output_t *output)
{
    uint16_t current;
    uint16_t desired;
    uint16_t ratio;
    int16_t angle;

    if (state == 0 || output == 0 ||
        curve_phase > CAR_ACKERMANN_PHASE_MAX ||
        slowdown_request_permille >
            CAR_ACKERMANN_REQUEST_MAX_PERMILLE ||
        car_ackermann_model_geometry(applied_pwm_us, center_pwm_us,
                                     left_endpoint_us, right_endpoint_us,
                                     &angle, &ratio) == 0u) {
        return 0u;
    }
    if (state->initialized == 0u) {
        car_ackermann_control_reset(state);
    }

    current = ramp_value_at(state, now_ms);
    if (state->ramp_active != 0u &&
        (uint32_t)(now_ms - state->ramp_started_ms) >=
            state->ramp_duration_ms) {
        state->fastest_target_millirps = state->ramp_target_millirps;
        state->ramp_active = 0u;
    } else {
        state->fastest_target_millirps = current;
    }

    desired = requested_fastest_target(slowdown_request_permille);
    if (slowdown_request_permille != state->request_permille) {
        state->request_permille = slowdown_request_permille;
        if (desired != state->fastest_target_millirps) {
            state->ramp_start_millirps = state->fastest_target_millirps;
            state->ramp_target_millirps = desired;
            state->ramp_started_ms = now_ms;
            state->ramp_duration_ms = desired < state->ramp_start_millirps
                ? CAR_ACKERMANN_REQUEST_RAMP_DOWN_MS
                : CAR_ACKERMANN_REQUEST_RAMP_UP_MS;
            state->ramp_active = 1u;
        } else {
            state->ramp_start_millirps = desired;
            state->ramp_target_millirps = desired;
            state->ramp_duration_ms = 0u;
            state->ramp_active = 0u;
        }
    }
    state->fastest_target_millirps = ramp_value_at(state, now_ms);
    if (state->ramp_active != 0u &&
        (uint32_t)(now_ms - state->ramp_started_ms) >=
            state->ramp_duration_ms) {
        state->fastest_target_millirps = state->ramp_target_millirps;
        state->ramp_active = 0u;
    }

    output->model_angle_cdeg = angle;
    output->ratio_q15 = ratio;
    output->request_permille = slowdown_request_permille;
    output->curve_phase = curve_phase;
    output->request_active = slowdown_request_permille != 0u;
    output->fastest_target_millirps = state->fastest_target_millirps;
    if (angle > 0) {
        output->left_target_millirps =
            state->fastest_target_millirps;
        output->right_target_millirps = (int32_t)(
            ((uint32_t)state->fastest_target_millirps * ratio + 16384u) /
            CAR_ACKERMANN_RATIO_ONE_Q15);
    } else if (angle < 0) {
        output->left_target_millirps = (int32_t)(
            ((uint32_t)state->fastest_target_millirps * ratio + 16384u) /
            CAR_ACKERMANN_RATIO_ONE_Q15);
        output->right_target_millirps =
            state->fastest_target_millirps;
    } else {
        output->left_target_millirps =
            state->fastest_target_millirps;
        output->right_target_millirps =
            state->fastest_target_millirps;
    }
    return 1u;
}

uint16_t car_ackermann_scale_nominal_permille(uint16_t base_permille,
                                              uint16_t ratio_q15,
                                              uint8_t inner_side)
{
    if (inner_side == 0u) {
        return base_permille;
    }
    return (uint16_t)(((uint32_t)base_permille * ratio_q15 + 16384u) /
                      CAR_ACKERMANN_RATIO_ONE_Q15);
}
