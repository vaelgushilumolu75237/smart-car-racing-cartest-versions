#ifndef CARTEST_SESSION_ODOMETRY_H
#define CARTEST_SESSION_ODOMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Convert a forward encoder-count delta to millirevolutions. The fractional
 * numerator is carried between accepted samples, so split intervals produce
 * the same result as converting their combined count total. */
uint8_t car_encoder_counts_to_millirev(
    uint64_t counts,
    uint32_t counts_per_rev,
    uint64_t *fractional_numerator_remainder,
    uint64_t *millirev);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SESSION_ODOMETRY_H */
