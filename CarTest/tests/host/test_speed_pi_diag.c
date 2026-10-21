#include "speed_pi_diag.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        printf("FAIL %s\n", message); \
        return 1; \
    } \
} while (0)

static uint8_t snapshot_crc_valid(void)
{
    return (uint8_t)(g_speed_pi_diag_snapshot.crc32 ==
                     car_speed_pi_diag_snapshot_crc32(
                         &g_speed_pi_diag_snapshot));
}

static car_speed_pi_diag_values_t sample_values(uint32_t now_ms,
                                                 uint32_t sample_ms)
{
    car_speed_pi_diag_values_t values;

    memset(&values, 0, sizeof(values));
    values.now_ms = now_ms;
    values.sample_ms = sample_ms;
    values.target_left_millirps = 2400;
    values.target_right_millirps = 2300;
    values.measured_left_millirps = 2100;
    values.measured_right_millirps = 2050;
    values.nominal_ff_left_permille = 500u;
    values.nominal_ff_right_permille = 490u;
    values.requested_left_permille = 1000u;
    values.requested_right_permille = 1000u;
    values.applied_left_permille = 680u;
    values.applied_right_permille = 670u;
    values.status = 3u;
    values.sample_elapsed_ms = 10u;
    values.enabled = 1u;
    values.armed = 1u;
    values.new_speed_sample = 1u;
    values.speed_sample_valid = 1u;
    values.saturated_left = 1u;
    values.saturated_right = 1u;
    values.pi_proposal_valid = 1u;
    return values;
}

int main(void)
{
    static const uint8_t crc_text[] = "123456789";
    car_speed_pi_diag_values_t values;
    car_speed_pi_diag_snapshot_t torn;
    uint32_t generation;
    int32_t retained_measured_left;
    uint16_t retained_requested_left;

    CHECK(sizeof(g_speed_pi_diag_snapshot) == 128u,
          "snapshot ABI size is not 128 bytes");
    CHECK(((uintptr_t)&g_speed_pi_diag_snapshot & 31u) == 0u,
          "snapshot object is not 32-byte aligned");
    CHECK(car_speed_pi_diag_crc32(crc_text, 9u) == 0xCBF43926u,
          "CRC32/ISO-HDLC vector mismatch");

    car_speed_pi_diag_init();
    CHECK(snapshot_crc_valid(), "IDLE snapshot CRC is invalid");
    CHECK(g_speed_pi_diag_snapshot.generation != 0u &&
              (g_speed_pi_diag_snapshot.generation & 1u) == 0u,
          "IDLE snapshot generation is not stable/even");
    CHECK(g_speed_pi_diag_snapshot.flags == 0u,
          "IDLE snapshot flags are not clear");

    car_speed_pi_diag_begin(7u, 100u,
                            CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL);
    CHECK(snapshot_crc_valid(), "begin snapshot CRC is invalid");
    CHECK(g_speed_pi_diag_snapshot.start_token == 7u &&
              g_speed_pi_diag_snapshot.start_ms == 100u &&
              g_speed_pi_diag_snapshot.pi_type ==
                  CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL,
          "begin did not publish the session identity");
    CHECK((g_speed_pi_diag_snapshot.flags &
           CAR_SPEED_PI_DIAG_FLAG_ACTIVE) != 0u,
          "begin did not activate the PI session");

    values = sample_values(110u, 110u);
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "first valid speed sample was not published");
    CHECK(snapshot_crc_valid(), "valid-sample snapshot CRC is invalid");
    CHECK(g_speed_pi_diag_snapshot.measured_left_millirps == 2100 &&
              g_speed_pi_diag_snapshot.target_right_millirps == 2300 &&
              g_speed_pi_diag_snapshot.nominal_ff_left_permille == 500u &&
              g_speed_pi_diag_snapshot.requested_left_permille == 1000u,
          "valid sample values were not published");
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 10u &&
              g_speed_pi_diag_snapshot.saturation_right_ms == 10u,
          "upper-cap saturation time did not accumulate");

    generation = g_speed_pi_diag_snapshot.generation;
    values.now_ms = 111u;
    CHECK(car_speed_pi_diag_update(&values) == 0u,
          "unchanged duplicate sample caused a redundant publish");
    CHECK(g_speed_pi_diag_snapshot.generation == generation,
          "unchanged duplicate sample advanced generation");

    values.now_ms = 112u;
    values.applied_left_permille = 681u;
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "changed applied output was not published");
    CHECK(g_speed_pi_diag_snapshot.generation == generation + 2u,
          "changed output did not advance generation exactly once");
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 10u,
          "same sample_ms accumulated saturation more than once");

    {
        const int32_t retained_target =
            g_speed_pi_diag_snapshot.target_left_millirps;
        const uint16_t retained_ff =
            g_speed_pi_diag_snapshot.nominal_ff_left_permille;
        const uint16_t retained_request =
            g_speed_pi_diag_snapshot.requested_left_permille;
        values = sample_values(115u, 115u);
        values.armed = 0u;
        values.pi_proposal_valid = 0u;
        values.saturated_left = 0u;
        values.saturated_right = 0u;
        values.target_left_millirps = 0;
        values.nominal_ff_left_permille = 0u;
        values.requested_left_permille = 0u;
        values.measured_left_millirps = 2222;
        CHECK(car_speed_pi_diag_update(&values) == 1u,
              "valid observer sample while unarmed was not published");
        CHECK(g_speed_pi_diag_snapshot.measured_left_millirps == 2222 &&
                  g_speed_pi_diag_snapshot.target_left_millirps == retained_target &&
                  g_speed_pi_diag_snapshot.nominal_ff_left_permille == retained_ff &&
                  g_speed_pi_diag_snapshot.requested_left_permille == retained_request,
              "unarmed observer sample erased the last valid PI proposal");
    }

    values = sample_values(120u, 120u);
    values.saturated_right = 0u;
    values.requested_right_permille = 0u;
    values.applied_right_permille = 0u;
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "second valid speed sample was not published");
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 20u &&
              g_speed_pi_diag_snapshot.saturation_right_ms == 10u,
          "per-side cap saturation accounting is incorrect");

    {
        const int32_t retained_target =
            g_speed_pi_diag_snapshot.target_left_millirps;
        const uint16_t retained_ff =
            g_speed_pi_diag_snapshot.nominal_ff_left_permille;
        const uint16_t retained_request =
            g_speed_pi_diag_snapshot.requested_left_permille;
        values = sample_values(130u, 130u);
        values.hold_active = 1u;
        values.status = 4u;
        values.target_left_millirps = 0;
        values.nominal_ff_left_permille = 0u;
        values.requested_left_permille = 0u;
        CHECK(car_speed_pi_diag_update(&values) == 1u,
              "hold-state transition was not published");
        CHECK(g_speed_pi_diag_snapshot.target_left_millirps == retained_target &&
                  g_speed_pi_diag_snapshot.nominal_ff_left_permille == retained_ff &&
                  g_speed_pi_diag_snapshot.requested_left_permille == retained_request,
              "hold observer sample erased the last valid PI proposal");
    }
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 20u &&
              g_speed_pi_diag_snapshot.saturation_right_ms == 10u &&
              (g_speed_pi_diag_snapshot.flags &
               CAR_SPEED_PI_DIAG_FLAG_HOLD_ACTIVE) != 0u,
          "hold interval was counted as upper-cap saturation");

    retained_measured_left = g_speed_pi_diag_snapshot.measured_left_millirps;
    retained_requested_left = g_speed_pi_diag_snapshot.requested_left_permille;
    values = sample_values(140u, 140u);
    values.new_speed_sample = 1u;
    values.speed_sample_valid = 0u;
    values.status = 5u;
    values.measured_left_millirps = 0;
    values.requested_left_permille = 0u;
    values.applied_left_permille = 0u;
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "invalid-state transition was not published");
    CHECK(g_speed_pi_diag_snapshot.measured_left_millirps ==
              retained_measured_left &&
              g_speed_pi_diag_snapshot.requested_left_permille ==
                  retained_requested_left,
          "invalid sample erased the last valid measure/request");

    /* A saturated flag at the lower/zero output does not mean upper-cap
       saturation and must not increase either duration. */
    values = sample_values(150u, 150u);
    values.requested_left_permille = 0u;
    values.requested_right_permille = 0u;
    values.applied_left_permille = 0u;
    values.applied_right_permille = 0u;
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "zero-output sample state was not published");
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 20u &&
              g_speed_pi_diag_snapshot.saturation_right_ms == 10u,
          "zero-output saturation was incorrectly counted as upper-cap time");

    /* Force a near-overflow counter image and prove the next valid interval
       saturates instead of wrapping. */
    g_speed_pi_diag_snapshot.saturation_left_ms = 0xFFFFFFFAu;
    g_speed_pi_diag_snapshot.crc32 = 0u;
    g_speed_pi_diag_snapshot.crc32 = car_speed_pi_diag_snapshot_crc32(
        &g_speed_pi_diag_snapshot);
    values = sample_values(160u, 160u);
    values.hold_active = 0u;
    values.saturated_right = 0u;
    values.requested_right_permille = 0u;
    CHECK(car_speed_pi_diag_update(&values) == 1u,
          "post-hold valid sample was not published");
    CHECK(g_speed_pi_diag_snapshot.saturation_left_ms == 0xFFFFFFFFu,
          "saturation counter wrapped instead of saturating");

    retained_measured_left = g_speed_pi_diag_snapshot.measured_left_millirps;
    retained_requested_left = g_speed_pi_diag_snapshot.requested_left_permille;
    generation = g_speed_pi_diag_snapshot.generation;
    car_speed_pi_diag_freeze(170u, 0x1234u);
    CHECK(snapshot_crc_valid(), "frozen snapshot CRC is invalid");
    CHECK((g_speed_pi_diag_snapshot.flags &
           CAR_SPEED_PI_DIAG_FLAG_FROZEN) != 0u &&
              (g_speed_pi_diag_snapshot.flags &
               CAR_SPEED_PI_DIAG_FLAG_ACTIVE) == 0u &&
              g_speed_pi_diag_snapshot.termination_fault == 0x1234u,
          "freeze state or termination fault was not retained");
    CHECK(g_speed_pi_diag_snapshot.measured_left_millirps ==
              retained_measured_left &&
              g_speed_pi_diag_snapshot.requested_left_permille ==
                  retained_requested_left,
          "freeze lost the final valid measure/request");

    values = sample_values(180u, 180u);
    CHECK(car_speed_pi_diag_update(&values) == 0u,
          "update after freeze changed the snapshot");
    car_speed_pi_diag_freeze(190u, 0u);
    CHECK(g_speed_pi_diag_snapshot.generation == generation + 2u,
          "repeated freeze changed the frozen generation");

    torn = g_speed_pi_diag_snapshot;
    torn.generation |= 1u;
    CHECK((torn.generation & 1u) != 0u,
          "test did not construct an odd/torn generation");
    torn = g_speed_pi_diag_snapshot;
    torn.measured_left_millirps ^= 1;
    CHECK(torn.crc32 != car_speed_pi_diag_snapshot_crc32(&torn),
          "torn payload unexpectedly passed the CRC check");

    car_speed_pi_diag_begin(8u, 210u,
                            CAR_SPEED_PI_DIAG_TYPE_POSITION);
    CHECK(snapshot_crc_valid() &&
              g_speed_pi_diag_snapshot.start_token == 8u &&
              g_speed_pi_diag_snapshot.saturation_left_ms == 0u &&
              g_speed_pi_diag_snapshot.measured_left_millirps == 0,
          "new session did not reset old diagnostic values");

    printf("SPEED_PI_DIAG_PASS abi=128 crc=valid freeze=stable saturation=upper-cap\n");
    return 0;
}
