#include "cartest_path_accel_core.h"

#include <stdbool.h>
#include <stdint.h>

static bool range_bad(const void *ptr, size_t length) {
    uintptr_t begin;
    if (ptr == NULL || length == 0U) {
        return true;
    }
    begin = (uintptr_t) ptr;
    return length > (size_t) (UINTPTR_MAX - begin);
}

static bool overlap(const void *left, size_t left_len,
                    const void *right, size_t right_len) {
    uintptr_t left_begin;
    uintptr_t right_begin;
    uintptr_t left_end;
    uintptr_t right_end;
    if (range_bad(left, left_len) || range_bad(right, right_len)) {
        return true;
    }
    left_begin = (uintptr_t) left;
    right_begin = (uintptr_t) right;
    left_end = left_begin + left_len;
    right_end = right_begin + right_len;
    return left_begin < right_end && right_begin < left_end;
}

static bool aligned4(const void *ptr) {
    return ptr != NULL && (((uintptr_t) ptr) & 3U) == 0U;
}

static bool extract_args_valid(
    const uint8_t *luma, size_t luma_len,
    const uint8_t *road, size_t road_len,
    const uint8_t *edge, size_t edge_len,
    const uint8_t *known, size_t known_len,
    uint8_t *counts, size_t counts_len,
    int32_t *candidates, size_t candidates_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t min_width, int32_t max_width, int32_t max_per_row) {
    size_t pixels = (size_t) CARTEST_PATH_ACCEL_WIDTH *
                    (size_t) CARTEST_PATH_ACCEL_HEIGHT;
    size_t layers;
    size_t nodes;
    size_t candidate_bytes;
    if (width != CARTEST_PATH_ACCEL_WIDTH ||
        height != CARTEST_PATH_ACCEL_HEIGHT || floor < 0 || floor >= height ||
        min_width < 8 || max_width < min_width || max_width > 70 ||
        max_per_row < 1 || max_per_row > CARTEST_PATH_ACCEL_MAX_PER_ROW ||
        luma == NULL || luma_len != pixels || road == NULL ||
        road_len != pixels || edge == NULL || edge_len != pixels ||
        known == NULL || known_len != pixels || counts == NULL ||
        candidates == NULL || !aligned4(candidates)) {
        return false;
    }
    layers = (size_t) (height - floor);
    nodes = layers * (size_t) max_per_row;
    candidate_bytes = nodes * CARTEST_PATH_ACCEL_CANDIDATE_STRIDE *
                      sizeof(int32_t);
    if (counts_len != layers || candidates_bytes != candidate_bytes) {
        return false;
    }
    if (overlap(counts, counts_len, luma, luma_len) ||
        overlap(counts, counts_len, road, road_len) ||
        overlap(counts, counts_len, edge, edge_len) ||
        overlap(counts, counts_len, known, known_len) ||
        overlap(counts, counts_len, candidates, candidates_bytes) ||
        overlap(candidates, candidates_bytes, luma, luma_len) ||
        overlap(candidates, candidates_bytes, road, road_len) ||
        overlap(candidates, candidates_bytes, edge, edge_len) ||
        overlap(candidates, candidates_bytes, known, known_len)) {
        return false;
    }
    return true;
}

int32_t cartest_path_accel_extract(
    const uint8_t *luma, size_t luma_len,
    const uint8_t *road, size_t road_len,
    const uint8_t *edge, size_t edge_len,
    const uint8_t *known, size_t known_len,
    uint8_t *counts, size_t counts_len,
    int32_t *candidates, size_t candidates_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t min_width, int32_t max_width, int32_t max_per_row) {
    int32_t layers;

    if (!extract_args_valid(luma, luma_len, road, road_len, edge, edge_len,
                            known, known_len, counts, counts_len,
                            candidates, candidates_bytes, width, height, floor,
                            min_width, max_width, max_per_row)) {
        return CARTEST_PATH_ACCEL_INVALID;
    }
    layers = height - floor;
    for (int32_t layer = 0; layer < layers; ++layer) {
        int32_t y = height - 1 - layer;
        size_t row = (size_t) y * (size_t) width;
        int32_t x = 0;
        int32_t candidate_count = 0;
        while (x < width) {
            size_t index = row + (size_t) x;
            int32_t start;
            int32_t end;
            int32_t run_width;
            int32_t left_hit;
            int32_t right_hit;
            size_t left_index;
            size_t right_index;
            int32_t bright_sum = 0;
            int32_t bright_mean_milli;
            int32_t dark_mean_milli;
            size_t offset;
            if (known[index] == 0U || road[index] == 0U) {
                ++x;
                continue;
            }
            start = x;
            while (x < width) {
                index = row + (size_t) x;
                if (known[index] == 0U || road[index] == 0U) {
                    break;
                }
                bright_sum += (int32_t) luma[index];
                ++x;
            }
            end = x - 1;
            run_width = end - start + 1;
            if (run_width < min_width || run_width > max_width) {
                continue;
            }
            left_hit = start - 1;
            right_hit = end + 1;
            if (left_hit < 0 || right_hit >= width) {
                continue;
            }
            left_index = row + (size_t) left_hit;
            right_index = row + (size_t) right_hit;
            if (known[left_index] == 0U || known[right_index] == 0U ||
                road[left_index] != 0U || road[right_index] != 0U ||
                edge[left_index] == 0U || edge[right_index] == 0U) {
                continue;
            }
            if (candidate_count >= max_per_row) {
                return -1;
            }
            offset = ((size_t) layer * (size_t) max_per_row +
                      (size_t) candidate_count) *
                     CARTEST_PATH_ACCEL_CANDIDATE_STRIDE;
            candidates[offset] = start;
            candidates[offset + 1U] = start + end;
            candidates[offset + 2U] = run_width;
            bright_mean_milli = (bright_sum * 1000) / run_width;
            dark_mean_milli = ((int32_t) luma[left_index] +
                               (int32_t) luma[right_index]) * 500;
            candidates[offset + 3U] = bright_mean_milli - dark_mean_milli;
            ++candidate_count;
        }
        counts[layer] = (uint8_t) candidate_count;
    }
    return 0;
}

static bool connect_args_valid(
    const uint8_t *counts, size_t counts_len,
    const int32_t *candidates, size_t candidates_bytes,
    int32_t *states, size_t states_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t max_gap, int32_t max_width_change,
    int32_t max_slope_milli, int32_t link_tolerance_milli,
    int32_t reference_y_milli, int32_t max_per_row,
    int32_t state_capacity) {
    size_t layers;
    size_t nodes;
    size_t candidate_bytes;
    size_t expected_state_capacity;
    size_t state_bytes;
    (void) reference_y_milli;
    if (width != CARTEST_PATH_ACCEL_WIDTH ||
        height != CARTEST_PATH_ACCEL_HEIGHT || floor < 0 || floor >= height ||
        max_gap < 0 || max_gap > 2 || max_width_change < 1 ||
        max_width_change > 70 || max_slope_milli < 0 ||
        max_slope_milli > 80000 || link_tolerance_milli < 0 ||
        link_tolerance_milli > 80000 || reference_y_milli < 0 ||
        reference_y_milli >= CARTEST_PATH_ACCEL_HEIGHT * 1000 ||
        max_per_row < 1 ||
        max_per_row > CARTEST_PATH_ACCEL_MAX_PER_ROW ||
        counts == NULL || candidates == NULL || states == NULL ||
        !aligned4(candidates) || !aligned4(states)) {
        return false;
    }
    layers = (size_t) (height - floor);
    nodes = layers * (size_t) max_per_row;
    candidate_bytes = nodes * CARTEST_PATH_ACCEL_CANDIDATE_STRIDE *
                      sizeof(int32_t);
    expected_state_capacity = nodes * 2U;
    state_bytes = expected_state_capacity *
                  CARTEST_PATH_ACCEL_STATE_STRIDE * sizeof(int32_t);
    if (counts_len != layers || candidates_bytes != candidate_bytes ||
        state_capacity != (int32_t) expected_state_capacity ||
        states_bytes != state_bytes) {
        return false;
    }
    if (overlap(states, states_bytes, counts, counts_len) ||
        overlap(states, states_bytes, candidates, candidates_bytes)) {
        return false;
    }
    /* Validate every count and candidate relationship before the first state
     * write. This makes malformed host/module input fail closed instead of
     * allowing an oversized count to read past candidates or states. */
    for (size_t layer = 0; layer < layers; ++layer) {
        uint8_t count = counts[layer];
        if (count > (uint8_t) max_per_row) {
            return false;
        }
        for (size_t slot = 0; slot < count; ++slot) {
            size_t base = (layer * (size_t) max_per_row + slot) * 4U;
            int32_t start = candidates[base];
            int32_t center_twice = candidates[base + 1U];
            int32_t run_width = candidates[base + 2U];
            int32_t contrast = candidates[base + 3U];
            if (run_width < 1 || run_width > width || start < 0 ||
                start > width - run_width ||
                center_twice != start * 2 + run_width - 1 ||
                contrast < -255000 || contrast > 255000) {
                return false;
            }
        }
    }
    return true;
}

static void reset_state(int32_t *states, size_t base, int32_t sentinel) {
    states[base] = sentinel;
    states[base + 1U] = -1;
    states[base + 8U] = -1;
    states[base + 9U] = 0;
}

static void copy_state(int32_t *states, size_t destination,
                       size_t source) {
    for (size_t field = 0; field < CARTEST_PATH_ACCEL_STATE_STRIDE; ++field) {
        states[destination + field] = states[source + field];
    }
}

int32_t cartest_path_accel_connect(
    const uint8_t *counts, size_t counts_len,
    const int32_t *candidates, size_t candidates_bytes,
    int32_t *states, size_t states_bytes,
    int32_t width, int32_t height, int32_t floor,
    int32_t max_gap, int32_t max_width_change,
    int32_t max_slope_milli, int32_t link_tolerance_milli,
    int32_t reference_y_milli, int32_t max_per_row,
    int32_t state_capacity) {
    const int32_t sentinel = -2147483647;
    int32_t layers;
    if (!connect_args_valid(counts, counts_len, candidates, candidates_bytes,
                            states, states_bytes, width, height, floor,
                            max_gap, max_width_change, max_slope_milli,
                            link_tolerance_milli, reference_y_milli,
                            max_per_row, state_capacity)) {
        return CARTEST_PATH_ACCEL_INVALID;
    }
    layers = height - floor;
    for (int32_t layer = 0; layer < layers; ++layer) {
        int32_t current_y = height - 1 - layer;
        for (int32_t slot = 0; slot < (int32_t) counts[layer]; ++slot) {
            int32_t node = layer * max_per_row + slot;
            size_t cbase = (size_t) node * 4U;
            int32_t center_twice = candidates[cbase + 1U];
            int32_t run_width = candidates[cbase + 2U];
            int32_t contrast_milli = candidates[cbase + 3U];
            int32_t local_reward;
            size_t first = (size_t) node * 2U;
            size_t first_base = first * 10U;
            size_t second_base = (first + 1U) * 10U;
            int32_t distance_limit = max_gap + 1;
            if (contrast_milli < 0) contrast_milli = 0;
            if (contrast_milli > 80000) contrast_milli = 80000;
            local_reward = 10000 + (contrast_milli * 25 + 500) / 1000;
            reset_state(states, first_base, sentinel);
            reset_state(states, second_base, sentinel);
            if (distance_limit > layer) distance_limit = layer;
            for (int32_t distance = 1; distance <= distance_limit; ++distance) {
                int32_t previous_layer = layer - distance;
                int32_t previous_count = counts[previous_layer];
                if (previous_count == 0) continue;
                for (int32_t previous_slot = 0;
                     previous_slot < previous_count; ++previous_slot) {
                    int32_t previous_node = previous_layer * max_per_row +
                                            previous_slot;
                    int32_t previous_center_twice =
                        candidates[(size_t) previous_node * 4U + 1U];
                    int32_t previous_width =
                        candidates[(size_t) previous_node * 4U + 2U];
                    int32_t width_delta = run_width - previous_width;
                    int32_t center_step_twice = center_twice -
                                                previous_center_twice;
                    int32_t actual_step_milli;
                    if (width_delta < 0) width_delta = -width_delta;
                    if (width_delta > max_width_change * distance) continue;
                    if (center_step_twice < 0) center_step_twice =
                        -center_step_twice;
                    actual_step_milli = center_step_twice * 500;
                    if (actual_step_milli > max_slope_milli * distance +
                        link_tolerance_milli) continue;
                    for (int32_t previous_slot_state = 0;
                         previous_slot_state < 2; ++previous_slot_state) {
                        int32_t previous_state = previous_node * 2 +
                                                  previous_slot_state;
                        size_t previous_base = (size_t) previous_state * 10U;
                        int32_t previous_score = states[previous_base];
                        int32_t parent_state;
                        int32_t predicted_twice;
                        int32_t center_error_twice;
                        int32_t center_error_milli;
                        int32_t center_penalty;
                        int32_t width_penalty;
                        int32_t gap_penalty;
                        int32_t score;
                        int32_t reference_delta;
                        int32_t old_reference_delta;
                        int32_t reference_node;
                        int32_t step_slope_milli;
                        int32_t slope_sum;
                        int32_t width_sum;
                        int32_t path_count;
                        int32_t bottom_y;
                        if (previous_score <= sentinel / 2) continue;
                        parent_state = states[previous_base + 1U];
                        if (parent_state < -1 || parent_state >= state_capacity)
                            continue;
                        predicted_twice = previous_center_twice;
                        if (parent_state >= 0) {
                            int32_t parent_node = parent_state / 2;
                            int32_t prior_layer = parent_node / max_per_row;
                            int32_t base_dy = previous_layer - prior_layer;
                            int32_t prior_center_twice;
                            int32_t prediction_delta;
                            if (base_dy <= 0 || prior_layer >= layers)
                                continue;
                            prior_center_twice = candidates[
                                (size_t) parent_node * 4U + 1U];
                            prediction_delta = (previous_center_twice -
                                                prior_center_twice) * distance;
                            if (prediction_delta >= 0)
                                predicted_twice += prediction_delta / base_dy;
                            else
                                predicted_twice -= (-prediction_delta) / base_dy;
                        }
                        center_error_twice = center_twice - predicted_twice;
                        if (center_error_twice < 0) center_error_twice =
                            -center_error_twice;
                        center_error_milli = center_error_twice * 500;
                        if (center_error_milli > max_slope_milli * distance +
                            link_tolerance_milli) continue;
                        center_penalty = (center_error_milli * 550 + 500) /
                                         1000;
                        width_penalty = (width_delta * 1925 + 5) / 10;
                        gap_penalty = (distance - 1) * 2000;
                        score = previous_score + local_reward + distance * 550 -
                                center_penalty - width_penalty - gap_penalty;
                        reference_delta = current_y * 1000 - reference_y_milli;
                        if (reference_delta < 0) reference_delta =
                            -reference_delta;
                        old_reference_delta = states[previous_base + 5U];
                        if (reference_delta < old_reference_delta) {
                            reference_node = node;
                        } else {
                            reference_node = states[previous_base + 4U];
                            reference_delta = old_reference_delta;
                        }
                        step_slope_milli = (actual_step_milli + distance / 2) /
                                           distance;
                        slope_sum = states[previous_base + 6U] +
                                    step_slope_milli;
                        width_sum = states[previous_base + 7U] + width_delta;
                        path_count = states[previous_base + 2U] + 1;
                        bottom_y = states[previous_base + 3U];
                        if (score > states[first_base]) {
                            copy_state(states, second_base, first_base);
                            states[first_base] = score;
                            states[first_base + 1U] = previous_state;
                            states[first_base + 2U] = path_count;
                            states[first_base + 3U] = bottom_y;
                            states[first_base + 4U] = reference_node;
                            states[first_base + 5U] = reference_delta;
                            states[first_base + 6U] = slope_sum;
                            states[first_base + 7U] = width_sum;
                            states[first_base + 8U] = -1;
                            states[first_base + 9U] = 0;
                        } else if (previous_state != states[first_base + 1U] &&
                                   score > states[second_base]) {
                            states[second_base] = score;
                            states[second_base + 1U] = previous_state;
                            states[second_base + 2U] = path_count;
                            states[second_base + 3U] = bottom_y;
                            states[second_base + 4U] = reference_node;
                            states[second_base + 5U] = reference_delta;
                            states[second_base + 6U] = slope_sum;
                            states[second_base + 7U] = width_sum;
                            states[second_base + 8U] = -1;
                            states[second_base + 9U] = 0;
                        }
                    }
                }
            }
            {
                int32_t reference_delta = current_y * 1000 -
                                           reference_y_milli;
                if (reference_delta < 0) reference_delta = -reference_delta;
                if (local_reward > states[first_base]) {
                    copy_state(states, second_base, first_base);
                    states[first_base] = local_reward;
                    states[first_base + 1U] = -1;
                    states[first_base + 2U] = 1;
                    states[first_base + 3U] = current_y;
                    states[first_base + 4U] = node;
                    states[first_base + 5U] = reference_delta;
                    states[first_base + 6U] = 0;
                    states[first_base + 7U] = 0;
                    states[first_base + 8U] = -1;
                    states[first_base + 9U] = 0;
                } else if (states[first_base + 1U] != -1 &&
                           local_reward > states[second_base]) {
                    states[second_base] = local_reward;
                    states[second_base + 1U] = -1;
                    states[second_base + 2U] = 1;
                    states[second_base + 3U] = current_y;
                    states[second_base + 4U] = node;
                    states[second_base + 5U] = reference_delta;
                    states[second_base + 6U] = 0;
                    states[second_base + 7U] = 0;
                    states[second_base + 8U] = -1;
                    states[second_base + 9U] = 0;
                }
            }
        }
    }
    return 0;
}
