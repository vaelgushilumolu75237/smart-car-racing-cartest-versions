#include "cartest_accel_core.h"

#include <stdbool.h>
#include <stdint.h>

static bool pointer_range_overflows(const void *ptr, size_t length) {
    uintptr_t begin;
    if (ptr == NULL || length == 0) {
        return true;
    }
    begin = (uintptr_t) ptr;
    return length > (size_t) (UINTPTR_MAX - begin);
}

static bool ranges_overlap(const void *left, size_t left_len,
                           const void *right, size_t right_len) {
    uintptr_t left_begin;
    uintptr_t right_begin;
    uintptr_t left_end;
    uintptr_t right_end;
    if (pointer_range_overflows(left, left_len) ||
        pointer_range_overflows(right, right_len)) {
        return true;
    }
    left_begin = (uintptr_t) left;
    right_begin = (uintptr_t) right;
    left_end = left_begin + left_len;
    right_end = right_begin + right_len;
    return left_begin < right_end && right_begin < left_end;
}

static bool aligned_for(const void *ptr, size_t alignment) {
    return ptr != NULL && (((uintptr_t) ptr) % alignment) == 0;
}

static bool output_disjoint_from_inputs(
    const void *output, size_t output_len,
    const void *input0, size_t input0_len,
    const void *input1, size_t input1_len,
    const void *input2, size_t input2_len) {
    return !ranges_overlap(output, output_len, input0, input0_len) &&
           !ranges_overlap(output, output_len, input1, input1_len) &&
           !ranges_overlap(output, output_len, input2, input2_len);
}

int32_t cartest_accel_region_segments(
    const uint8_t *mask, size_t mask_len,
    const uint8_t *known, size_t known_len,
    const uint8_t *row_counts, size_t row_counts_len,
    int32_t width, int32_t height, int32_t floor,
    uint8_t *trusted_rows, size_t trusted_rows_len,
    uint8_t *segment_starts, size_t segment_starts_len,
    uint8_t *segment_ends, size_t segment_ends_len,
    int32_t *segment_count_out, size_t segment_count_bytes) {
    const size_t total = CARTEST_ACCEL_PIXELS;
    const size_t count_bytes = 2U * sizeof(int32_t);
    int32_t segment_count = 0;
    int32_t segment_start = -1;
    int32_t all_known = 1;
    int32_t partial_rows = 0;
    int32_t trusted_row_count = 0;

    /* Validate every pointer, length, geometry, alignment, and write range
     * before the first buffer read or write. Read-only inputs may alias. */
    if (width != CARTEST_ACCEL_WIDTH || height != CARTEST_ACCEL_HEIGHT ||
        floor < 0 || floor >= height ||
        mask == NULL || mask_len != total ||
        known == NULL || known_len != total ||
        row_counts == NULL || row_counts_len != (size_t) height ||
        trusted_rows == NULL || trusted_rows_len != (size_t) height ||
        segment_starts == NULL || segment_starts_len != (size_t) height ||
        segment_ends == NULL || segment_ends_len != (size_t) height ||
        segment_count_out == NULL || segment_count_bytes != count_bytes ||
        !aligned_for(segment_count_out, _Alignof(int32_t))) {
        return CARTEST_ACCEL_INVALID;
    }
    if (!output_disjoint_from_inputs(trusted_rows, trusted_rows_len,
                                     mask, mask_len, known, known_len,
                                     row_counts, row_counts_len) ||
        !output_disjoint_from_inputs(segment_starts, segment_starts_len,
                                     mask, mask_len, known, known_len,
                                     row_counts, row_counts_len) ||
        !output_disjoint_from_inputs(segment_ends, segment_ends_len,
                                     mask, mask_len, known, known_len,
                                     row_counts, row_counts_len) ||
        !output_disjoint_from_inputs(segment_count_out, count_bytes,
                                     mask, mask_len, known, known_len,
                                     row_counts, row_counts_len) ||
        ranges_overlap(trusted_rows, trusted_rows_len,
                       segment_starts, segment_starts_len) ||
        ranges_overlap(trusted_rows, trusted_rows_len,
                       segment_ends, segment_ends_len) ||
        ranges_overlap(trusted_rows, trusted_rows_len,
                       segment_count_out, count_bytes) ||
        ranges_overlap(segment_starts, segment_starts_len,
                       segment_ends, segment_ends_len) ||
        ranges_overlap(segment_starts, segment_starts_len,
                       segment_count_out, count_bytes) ||
        ranges_overlap(segment_ends, segment_ends_len,
                       segment_count_out, count_bytes)) {
        return CARTEST_ACCEL_INVALID;
    }

    segment_count_out[0] = 0;
    segment_count_out[1] = 0;
    for (int32_t y = floor; y < height; ++y) {
        size_t offset = (size_t) y * (size_t) width;
        int32_t count = 0;
        int32_t known_count = 0;
        int32_t complete_row = 1;

        trusted_rows[y] = 0;
        for (int32_t x = 0; x < width; ++x) {
            size_t index = offset + (size_t) x;
            if (mask[index] != 0) {
                ++count;
            }
            if (known[index] == 0) {
                complete_row = 0;
                all_known = 0;
            } else {
                ++known_count;
            }
        }
        if (row_counts[y] != (uint8_t) count) {
            return -1;
        }
        if (known_count != 0 && known_count != width) {
            partial_rows = 1;
        }
        if (complete_row != 0) {
            trusted_rows[y] = 1;
            ++trusted_row_count;
            if (segment_start < 0) {
                segment_start = y;
            }
        } else if (segment_start >= 0) {
            segment_starts[segment_count] = (uint8_t) segment_start;
            segment_ends[segment_count] = (uint8_t) y;
            ++segment_count;
            segment_start = -1;
        }
    }
    if (segment_start >= 0) {
        segment_starts[segment_count] = (uint8_t) segment_start;
        segment_ends[segment_count] = (uint8_t) height;
        ++segment_count;
    }
    if (partial_rows != 0) {
        segment_count_out[0] = 0;
        return -2;
    }
    segment_count_out[0] = segment_count;
    segment_count_out[1] = trusted_row_count;
    if (segment_count == 0) {
        return 0;
    }
    return all_known != 0 ? 1 : 0;
}

static uint32_t load_u32_le(const uint8_t *bytes) {
    return (uint32_t) bytes[0] |
           ((uint32_t) bytes[1] << 8) |
           ((uint32_t) bytes[2] << 16) |
           ((uint32_t) bytes[3] << 24);
}

static bool internal_args_valid(
    const uint8_t *mask, size_t mask_len,
    uint8_t *visited, size_t visited_len,
    int32_t *visit_epoch, size_t visit_epoch_bytes,
    uint16_t *queue, size_t queue_bytes, size_t queue_capacity,
    int32_t width, int32_t height, int32_t floor, int32_t band_end,
    int32_t min_area, int32_t max_area,
    int32_t min_extent, int32_t max_extent, int32_t min_hole_area) {
    const size_t total = CARTEST_ACCEL_PIXELS;
    if (width != CARTEST_ACCEL_WIDTH || height != CARTEST_ACCEL_HEIGHT ||
        floor < 0 || floor >= height || band_end <= floor ||
        band_end > height ||
        mask == NULL || mask_len != total ||
        visited == NULL || visited_len != total ||
        visit_epoch == NULL || visit_epoch_bytes != sizeof(int32_t) ||
        queue == NULL || (queue_bytes % sizeof(uint16_t)) != 0 ||
        queue_capacity < 2 ||
        queue_capacity > CARTEST_ACCEL_QUEUE_ENTRIES ||
        queue_capacity > queue_bytes / sizeof(uint16_t) ||
        !aligned_for(visit_epoch, _Alignof(int32_t)) ||
        !aligned_for(queue, _Alignof(uint16_t)) ||
        min_area < 1 || max_area < min_area || max_area > (int32_t) total ||
        min_extent < 1 || max_extent < min_extent ||
        max_extent > width || min_hole_area < 1 ||
        min_hole_area > (int32_t) total) {
        return false;
    }
    if (ranges_overlap(mask, mask_len, visited, visited_len) ||
        ranges_overlap(mask, mask_len, visit_epoch, visit_epoch_bytes) ||
        ranges_overlap(mask, mask_len, queue, queue_bytes) ||
        ranges_overlap(visited, visited_len, visit_epoch, visit_epoch_bytes) ||
        ranges_overlap(visited, visited_len, queue, queue_bytes) ||
        ranges_overlap(visit_epoch, visit_epoch_bytes, queue, queue_bytes)) {
        return false;
    }
    return true;
}

static int32_t find_internal_component(
    const uint8_t *mask, uint8_t *visited,
    int32_t *visit_epoch, uint16_t *queue, size_t queue_capacity,
    int32_t floor, int32_t band_end,
    int32_t min_area, int32_t max_area,
    int32_t min_extent, int32_t max_extent, int32_t min_hole_area) {
    const int32_t width = CARTEST_ACCEL_WIDTH;
    const int32_t height = CARTEST_ACCEL_HEIGHT;
    const int32_t total = CARTEST_ACCEL_PIXELS;
    int32_t epoch = visit_epoch[0] + 1;
    int32_t visit_stamp;
    uint32_t visit_word_stamp;

    if (epoch >= 64) {
        for (int32_t index = 0; index < total; ++index) {
            visited[index] = 0;
        }
        epoch = 1;
    }
    visit_epoch[0] = epoch;
    visit_stamp = epoch << 2;
    visit_word_stamp = (uint32_t) visit_stamp * UINT32_C(0x01010101);

    for (int32_t start_y = floor; start_y < band_end; ++start_y) {
        int32_t start_row_offset = start_y * width;
        int32_t start_x = 0;
        while (start_x < width) {
            if ((start_x & 3) == 0 && start_x + 3 < width) {
                int32_t word_offset = start_row_offset + start_x;
                uint32_t mask_word = load_u32_le(mask + word_offset);
                uint32_t visited_word = load_u32_le(visited + word_offset);
                if (mask_word == 0U ||
                    (visited_word & UINT32_C(0xFCFCFCFC)) ==
                        visit_word_stamp) {
                    start_x += 4;
                    continue;
                }
            }
            int32_t start = start_row_offset + start_x;
            if (mask[start] == 0 ||
                ((uint32_t) visited[start] >> 2) == (uint32_t) epoch) {
                ++start_x;
                continue;
            }

            int32_t row_offset = start_row_offset;
            int32_t run_left = start_x;
            while (run_left > 0 && mask[row_offset + run_left - 1] != 0) {
                --run_left;
            }
            int32_t run_right = start_x;
            while (run_right + 1 < width &&
                   mask[row_offset + run_right + 1] != 0) {
                ++run_right;
            }
            int32_t seed_run_right = run_right;
            for (int32_t x = run_left; x <= run_right; ++x) {
                visited[row_offset + x] = (uint8_t) (visit_stamp | 1);
            }
            int32_t queue_count = 1;
            queue[0] = (uint16_t) (row_offset + run_left);
            queue[1] = (uint16_t) (row_offset + run_right);
            int32_t queue_head = 0;
            int32_t area = 0;
            int32_t touches_roi = 0;
            int32_t min_x = width;
            int32_t max_x = -1;
            int32_t min_y = height;
            int32_t max_y = -1;

            while (queue_head < queue_count) {
                int32_t queue_slot = queue_head * 2;
                int32_t index = queue[queue_slot];
                int32_t run_end_index = queue[queue_slot + 1];
                ++queue_head;
                int32_t y = index / width;
                row_offset = y * width;
                run_left = index - row_offset;
                run_right = run_end_index - row_offset;
                area += run_right - run_left + 1;
                if (run_left < min_x) min_x = run_left;
                if (run_right > max_x) max_x = run_right;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (run_left == 0 || run_right == width - 1 ||
                    y == floor || y == band_end - 1) {
                    touches_roi = 1;
                }

                for (int32_t direction = 0; direction < 2; ++direction) {
                    int32_t ny = direction == 0 ? y - 1 : y + 1;
                    if (ny < floor || ny >= band_end) continue;
                    int32_t neighbor_row = ny * width;
                    int32_t nx = run_left - 1;
                    if (nx < 0) nx = 0;
                    int32_t nx_limit = run_right + 1;
                    if (nx_limit >= width) nx_limit = width - 1;
                    while (nx <= nx_limit) {
                        int32_t neighbor = neighbor_row + nx;
                        if (mask[neighbor] != 0 &&
                            ((uint32_t) visited[neighbor] >> 2) !=
                                (uint32_t) epoch) {
                            int32_t next_left = nx;
                            while (next_left > 0 &&
                                   mask[neighbor_row + next_left - 1] != 0) {
                                --next_left;
                            }
                            int32_t next_right = nx;
                            while (next_right + 1 < width &&
                                   mask[neighbor_row + next_right + 1] != 0) {
                                ++next_right;
                            }
                            if ((size_t) queue_count * 2U + 1U >=
                                queue_capacity) {
                                return -1;
                            }
                            int32_t queue_slot = queue_count * 2;
                            queue[queue_slot] = (uint16_t) (
                                neighbor_row + next_left);
                            queue[queue_slot + 1] = (uint16_t) (
                                neighbor_row + next_right);
                            ++queue_count;
                            for (int32_t px = next_left; px <= next_right; ++px) {
                                visited[neighbor_row + px] =
                                    (uint8_t) (visit_stamp | 1);
                            }
                            nx = next_right + 1;
                        } else {
                            ++nx;
                        }
                    }
                }
            }

            int32_t box_width = max_x - min_x + 1;
            int32_t box_height = max_y - min_y + 1;
            if (touches_roi != 0 || area < min_area || area > max_area ||
                box_width < min_extent || box_height < min_extent ||
                box_width > max_extent || box_height > max_extent) {
                start_x = seed_run_right + 1;
                continue;
            }

            int32_t x0 = min_x - 1;
            if (x0 < 0) x0 = 0;
            int32_t y0 = min_y - 1;
            if (y0 < floor) y0 = floor;
            int32_t x1 = max_x + 1;
            if (x1 >= width) x1 = width - 1;
            int32_t y1 = max_y + 1;
            if (y1 >= band_end) y1 = band_end - 1;

            queue_count = 0;
            for (int32_t y = y0; y <= y1; ++y) {
                for (int32_t x = x0; x <= x1; ++x) {
                    if (x != x0 && x != x1 && y != y0 && y != y1) continue;
                    int32_t index = y * width + x;
                    if (mask[index] != 0 ||
                        ((uint32_t) visited[index] >> 2) == (uint32_t) epoch) {
                        continue;
                    }
                    int32_t run_left = x;
                    while (run_left > x0) {
                        int32_t neighbor = y * width + run_left - 1;
                        if (mask[neighbor] != 0 ||
                            ((uint32_t) visited[neighbor] >> 2) ==
                                (uint32_t) epoch) break;
                        --run_left;
                    }
                    int32_t run_right = x;
                    while (run_right < x1) {
                        int32_t neighbor = y * width + run_right + 1;
                        if (mask[neighbor] != 0 ||
                            ((uint32_t) visited[neighbor] >> 2) ==
                                (uint32_t) epoch) break;
                        ++run_right;
                    }
                    if ((size_t) queue_count * 2U + 1U >= queue_capacity) {
                        return -1;
                    }
                    queue[queue_count * 2] = (uint16_t) (
                        y * width + run_left);
                    queue[queue_count * 2 + 1] = (uint16_t) (
                        y * width + run_right);
                    ++queue_count;
                    for (int32_t px = run_left; px <= run_right; ++px) {
                        visited[y * width + px] = (uint8_t) (visit_stamp | 1);
                    }
                }
            }

            queue_head = 0;
            while (queue_head < queue_count) {
                int32_t queue_slot = queue_head * 2;
                int32_t index = queue[queue_slot];
                int32_t run_end_index = queue[queue_slot + 1];
                ++queue_head;
                int32_t y = index / width;
                row_offset = y * width;
                run_left = index - row_offset;
                run_right = run_end_index - row_offset;

                for (int32_t direction = 0; direction < 2; ++direction) {
                    int32_t ny = direction == 0 ? y - 1 : y + 1;
                    if (ny < y0 || ny > y1) continue;
                    int32_t neighbor_row = ny * width;
                    int32_t nx = run_left;
                    while (nx <= run_right) {
                        int32_t neighbor = neighbor_row + nx;
                        if (mask[neighbor] == 0 &&
                            ((uint32_t) visited[neighbor] >> 2) !=
                                (uint32_t) epoch) {
                            int32_t next_left = nx;
                            while (next_left > x0) {
                                neighbor = neighbor_row + next_left - 1;
                                if (mask[neighbor] != 0 ||
                                    ((uint32_t) visited[neighbor] >> 2) ==
                                        (uint32_t) epoch) break;
                                --next_left;
                            }
                            int32_t next_right = nx;
                            while (next_right < x1) {
                                neighbor = neighbor_row + next_right + 1;
                                if (mask[neighbor] != 0 ||
                                    ((uint32_t) visited[neighbor] >> 2) ==
                                        (uint32_t) epoch) break;
                                ++next_right;
                            }
                            if ((size_t) queue_count * 2U + 1U >=
                                queue_capacity) {
                                return -1;
                            }
                            queue[queue_count * 2] = (uint16_t) (
                                neighbor_row + next_left);
                            queue[queue_count * 2 + 1] = (uint16_t) (
                                neighbor_row + next_right);
                            ++queue_count;
                            for (int32_t px = next_left; px <= next_right; ++px) {
                                visited[neighbor_row + px] =
                                    (uint8_t) (visit_stamp | 1);
                            }
                            nx = next_right + 1;
                        } else {
                            ++nx;
                        }
                    }
                }
            }

            int32_t inner_left = x0 + 1;
            int32_t inner_right = x1 - 1;
            for (int32_t y = y0 + 1; y < y1; ++y) {
                int32_t x = inner_left;
                while (x <= inner_right) {
                    int32_t start = y * width + x;
                    if (mask[start] != 0 ||
                        ((uint32_t) visited[start] >> 2) == (uint32_t) epoch) {
                        ++x;
                        continue;
                    }
                    int32_t run_left = x;
                    while (run_left > inner_left) {
                        int32_t neighbor = y * width + run_left - 1;
                        if (mask[neighbor] != 0 ||
                            ((uint32_t) visited[neighbor] >> 2) ==
                                (uint32_t) epoch) break;
                        --run_left;
                    }
                    int32_t run_right = x;
                    while (run_right < inner_right) {
                        int32_t neighbor = y * width + run_right + 1;
                        if (mask[neighbor] != 0 ||
                            ((uint32_t) visited[neighbor] >> 2) ==
                                (uint32_t) epoch) break;
                        ++run_right;
                    }
                    int32_t seed_right = run_right;
                    for (int32_t px = run_left; px <= run_right; ++px) {
                        visited[y * width + px] = (uint8_t) (visit_stamp | 1);
                    }
                    queue_count = 1;
                    queue[0] = (uint16_t) (y * width + run_left);
                    queue[1] = (uint16_t) (y * width + run_right);
                    queue_head = 0;
                    int32_t hole_area = 0;
                    while (queue_head < queue_count) {
                        int32_t queue_slot = queue_head * 2;
                        int32_t index = queue[queue_slot];
                        int32_t run_end_index = queue[queue_slot + 1];
                        ++queue_head;
                        int32_t py = index / width;
                        row_offset = py * width;
                        run_left = index - row_offset;
                        run_right = run_end_index - row_offset;
                        hole_area += run_right - run_left + 1;

                        for (int32_t direction = 0; direction < 2; ++direction) {
                            int32_t ny = direction == 0 ? py - 1 : py + 1;
                            if (ny <= y0 || ny >= y1) continue;
                            int32_t neighbor_row = ny * width;
                            int32_t nx = run_left;
                            while (nx <= run_right) {
                                int32_t neighbor = neighbor_row + nx;
                                if (mask[neighbor] == 0 &&
                                    ((uint32_t) visited[neighbor] >> 2) !=
                                        (uint32_t) epoch) {
                                    int32_t next_left = nx;
                                    while (next_left > inner_left) {
                                        neighbor = neighbor_row + next_left - 1;
                                        if (mask[neighbor] != 0 ||
                                            ((uint32_t) visited[neighbor] >> 2) ==
                                                (uint32_t) epoch) break;
                                        --next_left;
                                    }
                                    int32_t next_right = nx;
                                    while (next_right < inner_right) {
                                        neighbor = neighbor_row + next_right + 1;
                                        if (mask[neighbor] != 0 ||
                                            ((uint32_t) visited[neighbor] >> 2) ==
                                                (uint32_t) epoch) break;
                                        ++next_right;
                                    }
                                    if ((size_t) queue_count * 2U + 1U >=
                                        queue_capacity) {
                                        return -1;
                                    }
                                    queue[queue_count * 2] = (uint16_t) (
                                        neighbor_row + next_left);
                                    queue[queue_count * 2 + 1] = (uint16_t) (
                                        neighbor_row + next_right);
                                    ++queue_count;
                                    for (int32_t px = next_left;
                                         px <= next_right; ++px) {
                                        visited[neighbor_row + px] =
                                            (uint8_t) (visit_stamp | 1);
                                    }
                                    nx = next_right + 1;
                                } else {
                                    ++nx;
                                }
                            }
                        }
                    }
                    if (hole_area >= min_hole_area) {
                        return 1;
                    }
                    x = seed_right + 1;
                }
            }
            ++start_x;
        }
    }
    return 0;
}

int32_t cartest_accel_internal_component(
    const uint8_t *mask, size_t mask_len,
    uint8_t *visited, size_t visited_len,
    int32_t *visit_epoch, size_t visit_epoch_bytes,
    uint16_t *queue, size_t queue_bytes, size_t queue_capacity,
    int32_t width, int32_t height, int32_t floor, int32_t band_end,
    int32_t min_area, int32_t max_area,
    int32_t min_extent, int32_t max_extent, int32_t min_hole_area) {
    if (!internal_args_valid(mask, mask_len, visited, visited_len,
                             visit_epoch, visit_epoch_bytes,
                             queue, queue_bytes, queue_capacity,
                             width, height, floor, band_end,
                             min_area, max_area,
                             min_extent, max_extent, min_hole_area)) {
        return CARTEST_ACCEL_INVALID;
    }
    /* State validation occurs after pointer/length/geometry validation and
     * before any output mutation. The runtime writes only epochs 0..63. */
    if (visit_epoch[0] < 0 || visit_epoch[0] > 63) {
        return CARTEST_ACCEL_INVALID;
    }
    return find_internal_component(mask, visited, visit_epoch, queue,
                                   queue_capacity, floor, band_end,
                                   min_area, max_area,
                                   min_extent, max_extent, min_hole_area);
}
