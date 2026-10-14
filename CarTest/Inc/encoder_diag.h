#ifndef CARTEST_ENCODER_DIAG_H
#define CARTEST_ENCODER_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t left_raw;
    uint16_t right_raw;
    uint8_t initialized;
    int64_t left_raw_delta_sum;
    int64_t right_raw_delta_sum;
    uint32_t ambiguous_delta_count;
} car_encoder_diag_t;

void car_encoder_diag_init(car_encoder_diag_t *diag,
                           uint32_t left_raw,
                           uint16_t right_raw);
/* Returns 0 and increments ambiguous_delta_count on an exact half-range jump. */
uint8_t car_encoder_diag_update(car_encoder_diag_t *diag,
                                uint32_t left_raw,
                                uint16_t right_raw);

#ifdef __cplusplus
}
#endif

#endif
