#include "session_odometry.h"

#include <stdio.h>
#include <stdint.h>

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        printf("FAIL %s\n", message); \
        return 1; \
    } \
} while (0)

static uint8_t convert(uint64_t counts, uint32_t counts_per_rev,
                       uint64_t *remainder, uint64_t *total_millirev)
{
    uint64_t delta_millirev;

    if (car_encoder_counts_to_millirev(counts, counts_per_rev, remainder,
                                       &delta_millirev) == 0u ||
        delta_millirev > UINT64_MAX - *total_millirev) {
        return 0u;
    }
    *total_millirev += delta_millirev;
    return 1u;
}

int main(void)
{
    const uint32_t counts_per_rev = 60000u;
    const uint32_t guard_millirev = 60000u;
    uint64_t remainder = 0u;
    uint64_t total_millirev = 0u;
    uint64_t delta_millirev = UINT64_MAX;
    uint64_t threshold_counts = UINT64_C(60) * counts_per_rev;
    uint64_t converted_counts = 0u;
    uint64_t left_carry = 0u;
    uint64_t right_carry = 0u;
    uint64_t left_total = 0u;
    uint64_t right_total = 0u;
    uint64_t left_counts = 0u;
    uint64_t right_counts = 0u;
    uint64_t aggregate_carry = 0u;
    uint64_t aggregate_millirev = 0u;
    uint32_t i;

    CHECK(car_encoder_counts_to_millirev(0u, counts_per_rev, &remainder,
                                         &delta_millirev) != 0u &&
              delta_millirev == 0u,
          "zero-count conversion failed");
    CHECK(car_encoder_counts_to_millirev(counts_per_rev, counts_per_rev,
                                         &remainder, &delta_millirev) != 0u &&
              delta_millirev == 1000u,
          "one full revolution is not 1000 millirevolutions");
    remainder = 0u;
    CHECK(car_encoder_counts_to_millirev(counts_per_rev / 2u,
                                         counts_per_rev, &remainder,
                                         &delta_millirev) != 0u &&
              delta_millirev == 500u,
          "half revolution conversion failed");

    remainder = 0u;
    total_millirev = 0u;
    for (i = 0u; i < counts_per_rev; ++i) {
        CHECK(convert(1u, counts_per_rev, &remainder, &total_millirev) != 0u,
              "repeated sub-revolution carry conversion failed");
    }
    CHECK(total_millirev == 1000u && remainder == 0u,
          "sub-count fractions were lost across accepted samples");

    remainder = 0u;
    total_millirev = 0u;
    while (converted_counts < threshold_counts) {
        const uint64_t remaining = threshold_counts - converted_counts;
        const uint64_t chunk = remaining < 137u ? remaining : 137u;

        CHECK(convert(chunk, counts_per_rev, &remainder, &total_millirev) !=
                  0u,
              "threshold-boundary conversion failed");
        converted_counts += chunk;
        CHECK(total_millirev < guard_millirev ||
                  converted_counts == threshold_counts,
              "60,000 millirevolution guard crossed before 60 turns");
    }
    CHECK(total_millirev == guard_millirev,
          "60,000 millirevolution guard does not correspond to 60 turns");

    remainder = 0u;
    delta_millirev = 55u;
    CHECK(car_encoder_counts_to_millirev(1u, 0u, &remainder,
                                         &delta_millirev) == 0u &&
              delta_millirev == 55u,
          "invalid calibration changed the output");

    remainder = counts_per_rev;
    delta_millirev = 55u;
    CHECK(car_encoder_counts_to_millirev(1u, counts_per_rev, &remainder,
                                         &delta_millirev) == 0u &&
              remainder == counts_per_rev && delta_millirev == 55u,
          "invalid fractional carry changed the carry or output");
    remainder = 0u;
    delta_millirev = 55u;
    CHECK(car_encoder_counts_to_millirev(UINT64_MAX, 1u, &remainder,
                                         &delta_millirev) == 0u &&
              remainder == 0u && delta_millirev == 55u,
          "overflowing conversion changed the carry or output");

    for (i = 0u; i < 10000u; ++i) {
        const uint64_t left_chunk = (i % 17u) + 1u;
        const uint64_t right_chunk = (i % 23u) + 1u;

        CHECK(convert(left_chunk, 59995u, &left_carry, &left_total) != 0u &&
                  convert(right_chunk, 60239u, &right_carry,
                          &right_total) != 0u,
              "asymmetric calibrated conversion failed");
        left_counts += left_chunk;
        right_counts += right_chunk;
    }
    CHECK(car_encoder_counts_to_millirev(left_counts, 59995u,
                                         &aggregate_carry,
                                         &aggregate_millirev) != 0u &&
              left_total == aggregate_millirev,
          "left 59,995 CPR carry diverged from aggregate conversion");
    aggregate_carry = 0u;
    aggregate_millirev = 0u;
    CHECK(car_encoder_counts_to_millirev(right_counts, 60239u,
                                         &aggregate_carry,
                                         &aggregate_millirev) != 0u &&
              right_total == aggregate_millirev,
          "right 60,239 CPR carry diverged from aggregate conversion");

    puts("PASS session odometry conversion, carry, and 60-revolution units");
    return 0;
}
