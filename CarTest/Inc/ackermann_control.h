#ifndef CARTEST_ACKERMANN_CONTROL_H
#define CARTEST_ACKERMANN_CONTROL_H

#include <stdint.h>
#include "car_track_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_ACKERMANN_PHASE_STRAIGHT = 0u,
    CAR_ACKERMANN_PHASE_CANDIDATE = 1u,
    CAR_ACKERMANN_PHASE_TURN = 2u,
    CAR_ACKERMANN_PHASE_EXIT = 3u,
    CAR_ACKERMANN_PHASE_MAX = CAR_ACKERMANN_PHASE_EXIT,
    CAR_ACKERMANN_WHEELBASE_TENTH_MM = 1435u,
    CAR_ACKERMANN_TRACK_TENTH_MM = 1670u,
    CAR_ACKERMANN_MAX_STEER_CDEG = 2000u,
    CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS =
        CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS,
    CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS =
        CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS,
    CAR_ACKERMANN_REQUEST_MAX_PERMILLE = 1000u,
    CAR_ACKERMANN_REQUEST_RAMP_DOWN_MS = 200u,
    CAR_ACKERMANN_REQUEST_RAMP_UP_MS = 800u,
    CAR_ACKERMANN_RATIO_ONE_Q15 = 32768u
};

typedef struct {
    uint32_t ramp_started_ms;
    uint16_t ramp_start_millirps;
    uint16_t ramp_target_millirps;
    uint16_t fastest_target_millirps;
    uint16_t request_permille;
    uint16_t ramp_duration_ms;
    uint8_t initialized;
    uint8_t ramp_active;
} car_ackermann_control_state_t;

typedef struct {
    int16_t model_angle_cdeg;
    uint16_t ratio_q15;
    uint16_t request_permille;
    uint8_t curve_phase;
    uint8_t request_active;
    int32_t fastest_target_millirps;
    int32_t left_target_millirps;
    int32_t right_target_millirps;
} car_ackermann_control_output_t;

void car_ackermann_control_reset(car_ackermann_control_state_t *state);

/* Uses the currently applied PWM pulse and the full measured center-to-limit
   span. No steering gain is applied here. Returns false for an invalid span
   or a pulse outside that span. */
uint8_t car_ackermann_model_geometry(uint16_t applied_pwm_us,
                                     uint16_t center_pwm_us,
                                     uint16_t left_endpoint_us,
                                     uint16_t right_endpoint_us,
                                     int16_t *model_angle_cdeg,
                                     uint16_t *ratio_q15);

/* Advance the profile-selected fastest/slowest request ramp using wrap-safe
   elapsed time. Repeated identical requests preserve ramp start and progress. */
uint8_t car_ackermann_control_update(
    car_ackermann_control_state_t *state,
    uint32_t now_ms,
    uint8_t curve_phase,
    uint16_t slowdown_request_permille,
    uint16_t applied_pwm_us,
    uint16_t center_pwm_us,
    uint16_t left_endpoint_us,
    uint16_t right_endpoint_us,
    car_ackermann_control_output_t *output);

/* Scale only the PI feed-forward side. Target speed is carried separately. */
uint16_t car_ackermann_scale_nominal_permille(uint16_t base_permille,
                                              uint16_t ratio_q15,
                                              uint8_t inner_side);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_ACKERMANN_CONTROL_H */
