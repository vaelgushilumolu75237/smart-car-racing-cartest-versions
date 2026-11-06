#include "speed_observer.h"

#include <limits.h>
#include <string.h>

static uint8_t subtract_i64(int64_t current, int64_t previous, int64_t *delta)
{
    if (delta == 0 ||
        (previous > 0 && current < INT64_MIN + previous) ||
        (previous < 0 && current > INT64_MAX + previous)) {
        return 0u;
    }
    *delta = current - previous;
    return 1u;
}

static uint8_t counts_to_millirps(int64_t normalized_delta,
                                  uint32_t counts_per_rev,
                                  uint32_t elapsed_ms,
                                  int32_t *millirps)
{
    int64_t numerator;
    uint64_t denominator;
    int64_t value;
    if (millirps == 0 || normalized_delta < 0 || counts_per_rev == 0u ||
        elapsed_ms == 0u ||
        normalized_delta > INT64_MAX / INT64_C(1000000)) {
        return 0u;
    }
    numerator = normalized_delta * INT64_C(1000000);
    denominator = (uint64_t)counts_per_rev * elapsed_ms;
    if (denominator == 0u || denominator > (uint64_t)INT64_MAX) {
        return 0u;
    }
    value = numerator / (int64_t)denominator;
    if (value > INT32_MAX) {
        return 0u;
    }
    *millirps = (int32_t)value;
    return 1u;
}

static void copy_result(const car_speed_observer_t *observer,
                        car_speed_observer_result_t *result)
{
    if (observer == 0 || result == 0) {
        return;
    }
    result->valid = observer->valid;
    result->status = observer->status;
    result->sample_ms = observer->last_sample_ms;
    result->elapsed_ms = observer->elapsed_ms;
    result->left_millirps = observer->left_millirps;
    result->right_millirps = observer->right_millirps;
    result->left_rpm = observer->left_rpm;
    result->right_rpm = observer->right_rpm;
}

void car_speed_observer_init(car_speed_observer_t *observer,
                             uint32_t now_ms,
                             int64_t left_counts,
                             int64_t right_counts,
                             uint32_t left_counts_per_rev,
                             int8_t left_sign,
                             uint32_t right_counts_per_rev,
                             int8_t right_sign)
{
    if (observer == 0) {
        return;
    }
    (void)memset(observer, 0, sizeof(*observer));
    observer->last_sample_ms = now_ms;
    observer->last_call_ms = now_ms;
    observer->previous_left_counts = left_counts;
    observer->previous_right_counts = right_counts;
    observer->left_counts_per_rev = left_counts_per_rev;
    observer->right_counts_per_rev = right_counts_per_rev;
    observer->left_sign = left_sign;
    observer->right_sign = right_sign;
    observer->initialized = 1u;
    observer->motion_expected_continuous_since_sample = 1u;
    observer->status =
        left_counts_per_rev != 0u && right_counts_per_rev != 0u &&
                (left_sign == 1 || left_sign == -1) &&
                (right_sign == 1 || right_sign == -1)
            ? CAR_SPEED_OBSERVER_STATUS_NOT_READY
            : CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG;
    if (observer->status == CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG) {
        observer->initialized = 0u;
    }
}

void car_speed_observer_invalidate(car_speed_observer_t *observer,
                                   uint32_t now_ms,
                                   int64_t left_counts,
                                   int64_t right_counts,
                                   uint32_t status)
{
    if (observer == 0) {
        return;
    }
    observer->last_sample_ms = now_ms;
    observer->last_call_ms = now_ms;
    observer->previous_left_counts = left_counts;
    observer->previous_right_counts = right_counts;
    observer->valid = 0u;
    observer->expected_motion_since_sample = 0u;
    observer->motion_expected_continuous_since_sample = 0u;
    observer->left_zero_expected_ms = 0u;
    observer->right_zero_expected_ms = 0u;
    observer->left_millirps = 0;
    observer->right_millirps = 0;
    observer->left_rpm = 0;
    observer->right_rpm = 0;
    observer->elapsed_ms = 0u;
    observer->status = status;
}

uint32_t car_speed_observer_update(car_speed_observer_t *observer,
                                   uint32_t now_ms,
                                   int64_t left_counts,
                                   int64_t right_counts,
                                   uint8_t motion_expected,
                                   car_speed_observer_result_t *result)
{
    uint32_t elapsed_ms;
    uint32_t call_elapsed_ms;
    int64_t left_raw_delta;
    int64_t right_raw_delta;
    int64_t left_delta;
    int64_t right_delta;
    int32_t left_millirps;
    int32_t right_millirps;
    uint8_t sampled_motion_expected;
    uint8_t sampled_motion_continuous;
    if (observer == 0) {
        return CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG;
    }
    if (observer->initialized == 0u) {
        observer->status = CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG;
        copy_result(observer, result);
        return observer->status;
    }

    call_elapsed_ms = (uint32_t)(now_ms - observer->last_call_ms);
    if (call_elapsed_ms >= 0x80000000u) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_CLOCK_GAP);
        copy_result(observer, result);
        return observer->status;
    }
    observer->last_call_ms = now_ms;
    if (motion_expected != 0u) {
        observer->expected_motion_since_sample = 1u;
    } else {
        observer->expected_motion_since_sample = 0u;
        observer->motion_expected_continuous_since_sample = 0u;
        observer->left_zero_expected_ms = 0u;
        observer->right_zero_expected_ms = 0u;
    }
    elapsed_ms = (uint32_t)(now_ms - observer->last_sample_ms);
    if (elapsed_ms >= 0x80000000u ||
        elapsed_ms > CAR_SPEED_OBSERVER_MAX_GAP_MS) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_CLOCK_GAP);
        copy_result(observer, result);
        return observer->status;
    }
    if (elapsed_ms < CAR_SPEED_OBSERVER_PERIOD_MS) {
        if (!(observer->status == CAR_SPEED_OBSERVER_STATUS_STALL &&
              motion_expected != 0u &&
              (observer->left_zero_expected_ms >=
                   CAR_SPEED_OBSERVER_STALL_MS ||
               observer->right_zero_expected_ms >=
                   CAR_SPEED_OBSERVER_STALL_MS))) {
            observer->status = observer->valid != 0u
                ? CAR_SPEED_OBSERVER_STATUS_WAITING
                : CAR_SPEED_OBSERVER_STATUS_NOT_READY;
        }
        copy_result(observer, result);
        return observer->status;
    }

    sampled_motion_expected = observer->expected_motion_since_sample;
    sampled_motion_continuous =
        observer->motion_expected_continuous_since_sample;
    observer->expected_motion_since_sample = 0u;
    observer->motion_expected_continuous_since_sample = 1u;
    if (subtract_i64(left_counts, observer->previous_left_counts,
                     &left_raw_delta) == 0u ||
        subtract_i64(right_counts, observer->previous_right_counts,
                     &right_raw_delta) == 0u) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE);
        copy_result(observer, result);
        return observer->status;
    }
    observer->previous_left_counts = left_counts;
    observer->previous_right_counts = right_counts;
    observer->last_sample_ms = now_ms;
    observer->elapsed_ms = elapsed_ms;

    if ((observer->left_sign < 0 && left_raw_delta == INT64_MIN) ||
        (observer->right_sign < 0 && right_raw_delta == INT64_MIN)) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE);
        copy_result(observer, result);
        return observer->status;
    }
    left_delta = observer->left_sign < 0 ? -left_raw_delta : left_raw_delta;
    right_delta = observer->right_sign < 0 ? -right_raw_delta : right_raw_delta;
    if (left_delta < 0 || right_delta < 0) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_REVERSE_DELTA);
        copy_result(observer, result);
        return observer->status;
    }
    if (counts_to_millirps(left_delta, observer->left_counts_per_rev,
                           elapsed_ms, &left_millirps) == 0u ||
        counts_to_millirps(right_delta, observer->right_counts_per_rev,
                           elapsed_ms, &right_millirps) == 0u) {
        car_speed_observer_invalidate(
            observer, now_ms, left_counts, right_counts,
            CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE);
        copy_result(observer, result);
        return observer->status;
    }

    if (left_delta != 0) {
        observer->left_zero_expected_ms = 0u;
    } else if (sampled_motion_expected != 0u &&
               sampled_motion_continuous != 0u) {
        uint32_t room = CAR_SPEED_OBSERVER_STALL_MS -
                        observer->left_zero_expected_ms;
        observer->left_zero_expected_ms +=
            elapsed_ms > room ? room : elapsed_ms;
    }
    if (right_delta != 0) {
        observer->right_zero_expected_ms = 0u;
    } else if (sampled_motion_expected != 0u &&
               sampled_motion_continuous != 0u) {
        uint32_t room = CAR_SPEED_OBSERVER_STALL_MS -
                        observer->right_zero_expected_ms;
        observer->right_zero_expected_ms +=
            elapsed_ms > room ? room : elapsed_ms;
    }
    if (sampled_motion_expected == 0u) {
        observer->left_zero_expected_ms = 0u;
        observer->right_zero_expected_ms = 0u;
    }
    if (observer->left_zero_expected_ms >= CAR_SPEED_OBSERVER_STALL_MS ||
        observer->right_zero_expected_ms >= CAR_SPEED_OBSERVER_STALL_MS) {
        observer->valid = 0u;
        observer->left_millirps = 0;
        observer->right_millirps = 0;
        observer->left_rpm = 0;
        observer->right_rpm = 0;
        observer->status = CAR_SPEED_OBSERVER_STATUS_STALL;
        copy_result(observer, result);
        return observer->status;
    }

    observer->left_millirps = left_millirps;
    observer->right_millirps = right_millirps;
    observer->left_rpm = (int32_t)(((int64_t)left_millirps * 60) / 1000);
    observer->right_rpm = (int32_t)(((int64_t)right_millirps * 60) / 1000);
    observer->elapsed_ms = elapsed_ms;
    observer->valid = 1u;
    observer->status = CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    copy_result(observer, result);
    return observer->status;
}
