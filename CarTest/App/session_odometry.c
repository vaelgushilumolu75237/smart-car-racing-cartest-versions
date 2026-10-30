#include "session_odometry.h"

#include <stddef.h>

uint8_t car_encoder_counts_to_millirev(
    uint64_t counts,
    uint32_t counts_per_rev,
    uint64_t *fractional_numerator_remainder,
    uint64_t *millirev)
{
    const uint64_t scale = UINT64_C(1000);
    uint64_t whole_revolutions;
    uint64_t remaining_counts;
    uint64_t prior_remainder;
    uint64_t whole_millirev;
    uint64_t fractional_numerator;
    uint64_t fractional_millirev;

    if (counts_per_rev == 0u || fractional_numerator_remainder == NULL ||
        millirev == NULL) {
        return 0u;
    }
    prior_remainder = *fractional_numerator_remainder;
    if (prior_remainder >= counts_per_rev) {
        return 0u;
    }

    whole_revolutions = counts / counts_per_rev;
    remaining_counts = counts % counts_per_rev;
    if (whole_revolutions > UINT64_MAX / scale ||
        remaining_counts > (UINT64_MAX - prior_remainder) / scale) {
        return 0u;
    }
    whole_millirev = whole_revolutions * scale;
    fractional_numerator = remaining_counts * scale + prior_remainder;
    fractional_millirev = fractional_numerator / counts_per_rev;
    if (whole_millirev > UINT64_MAX - fractional_millirev) {
        return 0u;
    }

    *millirev = whole_millirev + fractional_millirev;
    *fractional_numerator_remainder =
        fractional_numerator % counts_per_rev;
    return 1u;
}
