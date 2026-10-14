#ifndef CARTEST_SPEED_OBSERVER_H
#define CARTEST_SPEED_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_SPEED_OBSERVER_PERIOD_MS = 10u,
    CAR_SPEED_OBSERVER_MAX_GAP_MS = 100u,
    CAR_SPEED_OBSERVER_STALL_MS = 300u,
    CAR_SPEED_OBSERVER_STATUS_NOT_READY = 0u,
    CAR_SPEED_OBSERVER_STATUS_WAITING = 1u,
    CAR_SPEED_OBSERVER_STATUS_SAMPLE = 2u,
    CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG = 3u,
    CAR_SPEED_OBSERVER_STATUS_CLOCK_GAP = 4u,
    CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE = 5u,
    CAR_SPEED_OBSERVER_STATUS_REVERSE_DELTA = 6u,
    CAR_SPEED_OBSERVER_STATUS_STALL = 7u
};

typedef struct {
    uint32_t last_sample_ms;
    int64_t previous_left_counts;
    int64_t previous_right_counts;
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    int8_t left_sign;
    int8_t right_sign;
    uint8_t initialized;
    uint8_t valid;
    uint8_t expected_motion_since_sample;
    uint8_t motion_expected_continuous_since_sample;
    uint8_t reserved;
    uint32_t last_call_ms;
    uint32_t left_zero_expected_ms;
    uint32_t right_zero_expected_ms;
    uint32_t elapsed_ms;
    int32_t left_millirps;
    int32_t right_millirps;
    int32_t left_rpm;
    int32_t right_rpm;
    uint32_t status;
} car_speed_observer_t;

typedef struct {
    uint8_t valid;
    uint32_t status;
    uint32_t sample_ms;
    uint32_t elapsed_ms;
    int32_t left_millirps;
    int32_t right_millirps;
    int32_t left_rpm;
    int32_t right_rpm;
} car_speed_observer_result_t;

void car_speed_observer_init(car_speed_observer_t *observer,
                             uint32_t now_ms,
                             int64_t left_counts,
                             int64_t right_counts,
                             uint32_t left_counts_per_rev,
                             int8_t left_sign,
                             uint32_t right_counts_per_rev,
                             int8_t right_sign);

/*
 * Call from the main loop with signed accumulated encoder counts. Sampling is
 * at most once per call after 10 ms; elapsed time is used directly and missed
 * periods are never replayed. Motion expectation must reflect software-applied
 * nonzero motor output after startup qualification. 300 ms of expected motion
 * with zero counts invalidates the estimate as a likely stall/disconnection.
 */
uint32_t car_speed_observer_update(car_speed_observer_t *observer,
                                   uint32_t now_ms,
                                   int64_t left_counts,
                                   int64_t right_counts,
                                   uint8_t motion_expected,
                                   car_speed_observer_result_t *result);

void car_speed_observer_invalidate(car_speed_observer_t *observer,
                                   uint32_t now_ms,
                                   int64_t left_counts,
                                   int64_t right_counts,
                                   uint32_t status);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SPEED_OBSERVER_H */
