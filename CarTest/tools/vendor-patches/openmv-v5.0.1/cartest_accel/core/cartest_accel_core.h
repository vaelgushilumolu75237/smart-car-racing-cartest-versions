#ifndef CARTEST_ACCEL_CORE_H
#define CARTEST_ACCEL_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARTEST_ACCEL_API_VERSION 1
#define CARTEST_ACCEL_WIDTH 80
#define CARTEST_ACCEL_HEIGHT 60
#define CARTEST_ACCEL_PIXELS (CARTEST_ACCEL_WIDTH * CARTEST_ACCEL_HEIGHT)
#define CARTEST_ACCEL_QUEUE_ENTRIES 4800

/* Invalid pointers, lengths, geometry, overlap, alignment, or settings. */
#define CARTEST_ACCEL_INVALID (-100)

#if defined(_WIN32)
#define CARTEST_ACCEL_EXPORT __declspec(dllexport)
#else
#define CARTEST_ACCEL_EXPORT __attribute__((visibility("default")))
#endif

/* Result codes match fast_topology's Viper kernel for valid calls. */
CARTEST_ACCEL_EXPORT int32_t cartest_accel_region_segments(
    const uint8_t *mask, size_t mask_len,
    const uint8_t *known, size_t known_len,
    const uint8_t *row_counts, size_t row_counts_len,
    int32_t width, int32_t height, int32_t floor,
    uint8_t *trusted_rows, size_t trusted_rows_len,
    uint8_t *segment_starts, size_t segment_starts_len,
    uint8_t *segment_ends, size_t segment_ends_len,
    int32_t *segment_count_out, size_t segment_count_bytes);

CARTEST_ACCEL_EXPORT int32_t cartest_accel_internal_component(
    const uint8_t *mask, size_t mask_len,
    uint8_t *visited, size_t visited_len,
    int32_t *visit_epoch, size_t visit_epoch_bytes,
    uint16_t *queue, size_t queue_bytes, size_t queue_capacity,
    int32_t width, int32_t height, int32_t floor, int32_t band_end,
    int32_t min_area, int32_t max_area,
    int32_t min_extent, int32_t max_extent, int32_t min_hole_area);

#ifdef __cplusplus
}
#endif

#endif
