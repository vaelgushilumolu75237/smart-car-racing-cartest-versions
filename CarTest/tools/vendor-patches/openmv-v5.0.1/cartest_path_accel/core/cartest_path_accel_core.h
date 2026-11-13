#ifndef CARTEST_PATH_ACCEL_CORE_H
#define CARTEST_PATH_ACCEL_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARTEST_PATH_ACCEL_API_VERSION 1
#define CARTEST_PATH_ACCEL_WIDTH 80
#define CARTEST_PATH_ACCEL_HEIGHT 60
#define CARTEST_PATH_ACCEL_MAX_PER_ROW 8
#define CARTEST_PATH_ACCEL_CANDIDATE_STRIDE 4
#define CARTEST_PATH_ACCEL_STATE_STRIDE 10
#define CARTEST_PATH_ACCEL_INVALID (-100)

#if defined(_WIN32)
#define CARTEST_PATH_ACCEL_EXPORT __declspec(dllexport)
#else
#define CARTEST_PATH_ACCEL_EXPORT __attribute__((visibility("default")))
#endif

CARTEST_PATH_ACCEL_EXPORT int32_t cartest_path_accel_extract(
    const uint8_t *luma, size_t luma_len,
    const uint8_t *road, size_t road_len,
    const uint8_t *edge, size_t edge_len,
    const uint8_t *known, size_t known_len,
    uint8_t *counts, size_t counts_len,
    int32_t *candidates, size_t candidates_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t min_width, int32_t max_width, int32_t max_per_row);

CARTEST_PATH_ACCEL_EXPORT int32_t cartest_path_accel_connect(
    const uint8_t *counts, size_t counts_len,
    const int32_t *candidates, size_t candidates_bytes,
    int32_t *states, size_t states_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t max_gap, int32_t max_width_change,
    int32_t max_slope_milli, int32_t link_tolerance_milli,
    int32_t reference_y_milli, int32_t max_per_row,
    int32_t state_capacity);

#ifdef __cplusplus
}
#endif

#endif
