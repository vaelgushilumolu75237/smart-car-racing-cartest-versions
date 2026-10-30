#ifndef CARTEST_SPEED_PI_H
#define CARTEST_SPEED_PI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_SPEED_PI_MAX_OUTPUT_PERMILLE = 1000u,
    CAR_SPEED_PI_MAX_GAIN_Q16 = 50u * 65536u,
    CAR_SPEED_PI_MAX_SPEED_MILLIRPS = 100000000u,
    CAR_SPEED_PI_MAX_DT_MS = 1000u,
    CAR_SPEED_PI_MAX_TRACKING_TIME_MS = 10000u,
    CAR_SPEED_PI_LIFECYCLE_STOPPED = 0u,
    CAR_SPEED_PI_LIFECYCLE_STARTUP = 1u,
    CAR_SPEED_PI_LIFECYCLE_RUNNING = 2u,
    CAR_SPEED_PI_LIFECYCLE_HOLD = 3u
};

typedef struct {
    uint8_t configured;
    uint16_t output_cap_permille;
    /* Q16.16 gains: Kp = permille / (milli-RPS), Ki = permille / (milli-RPS*s). */
    uint32_t kp_q16;
    uint32_t ki_q16;
    /* Zero preserves legacy full tracking; nonzero scales correction by dt/Tt. */
    uint16_t tracking_time_ms;
    uint16_t reserved;
} car_speed_pi_config_t;

typedef struct {
    uint32_t lifecycle;
    uint8_t speed_sample_valid;
    uint8_t downstream_applied_valid;
    uint16_t downstream_applied_permille;
    uint16_t feedforward_permille;
    uint16_t reserved;
    uint32_t dt_ms;
    int32_t target_millirps;
    int32_t measured_millirps;
} car_speed_pi_input_t;

typedef struct {
    int64_t integral_q16;
    uint16_t requested_permille;
    uint8_t motor_permission;
    uint8_t saturated;
    uint8_t previous_proposal_valid;
    uint8_t reserved;
    uint32_t status;
} car_speed_pi_state_t;

typedef struct {
    uint16_t requested_permille;
    uint8_t motor_permission;
    uint8_t saturated;
    uint32_t status;
} car_speed_pi_output_t;

enum {
    CAR_SPEED_PI_STATUS_DISABLED = 0u,
    CAR_SPEED_PI_STATUS_RUNNING = 1u,
    CAR_SPEED_PI_STATUS_HOLD = 2u,
    CAR_SPEED_PI_STATUS_INVALID = 3u,
    CAR_SPEED_PI_STATUS_STOPPED = 4u,
    CAR_SPEED_PI_STATUS_STARTUP = 5u
};

void car_speed_pi_reset(car_speed_pi_state_t *state);

/*
 * Pure host-testable PI proposal. It never writes hardware or motor outputs.
 * downstream_applied_permille, when valid, must be the measured/slew-limited
 * output corresponding to the previous proposal stored in state.
 */
uint32_t car_speed_pi_update(car_speed_pi_state_t *state,
                             const car_speed_pi_config_t *config,
                             const car_speed_pi_input_t *input,
                             car_speed_pi_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SPEED_PI_H */
