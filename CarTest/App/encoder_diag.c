#include "encoder_diag.h"

#include <string.h>

static int32_t delta_u32(uint32_t previous, uint32_t current, uint8_t *ok)
{
    uint32_t delta = current - previous;
    if (delta == 0x80000000u) {
        *ok = 0u;
        return 0;
    }
    *ok = 1u;
    if (delta < 0x80000000u) {
        return (int32_t)delta;
    }
    return (int32_t)((int64_t)delta - 0x100000000LL);
}

static int32_t delta_u16(uint16_t previous, uint16_t current, uint8_t *ok)
{
    uint16_t delta = (uint16_t)(current - previous);
    if (delta == 0x8000u) {
        *ok = 0u;
        return 0;
    }
    *ok = 1u;
    if (delta < 0x8000u) {
        return (int32_t)delta;
    }
    return (int32_t)delta - 0x10000;
}

void car_encoder_diag_init(car_encoder_diag_t *diag,
                           uint32_t left_raw,
                           uint16_t right_raw)
{
    if (diag == 0) {
        return;
    }
    (void)memset(diag, 0, sizeof(*diag));
    diag->left_raw = left_raw;
    diag->right_raw = right_raw;
    diag->initialized = 1u;
}

uint8_t car_encoder_diag_update(car_encoder_diag_t *diag,
                                uint32_t left_raw,
                                uint16_t right_raw)
{
    uint8_t left_ok;
    uint8_t right_ok;
    int32_t left_delta;
    int32_t right_delta;
    if (diag == 0) {
        return 0u;
    }
    if (diag->initialized == 0u) {
        car_encoder_diag_init(diag, left_raw, right_raw);
        return 1u;
    }

    left_delta = delta_u32(diag->left_raw, left_raw, &left_ok);
    right_delta = delta_u16(diag->right_raw, right_raw, &right_ok);
    if (left_ok == 0u || right_ok == 0u) {
        ++diag->ambiguous_delta_count;
        return 0u;
    }

    diag->left_raw = left_raw;
    diag->right_raw = right_raw;
    diag->left_raw_delta_sum += left_delta;
    diag->right_raw_delta_sum += right_delta;
    return 1u;
}
