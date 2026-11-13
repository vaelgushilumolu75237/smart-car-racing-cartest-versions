#include "receive_diag.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        printf("FAIL %s\n", message); \
        return 1; \
    } \
} while (0)

int main(void)
{
    static const uint8_t crc_text[] = "123456789";
    const uint32_t start = UINT32_MAX - 499u;
    uint32_t publish_ms;

    CHECK(sizeof(g_receive_diag_snapshot) == CAR_RECEIVE_DIAG_SIZE,
          "snapshot ABI size changed");
    CHECK(((uintptr_t)&g_receive_diag_snapshot & 31u) == 0u,
          "snapshot object is not cache-line aligned");
    CHECK(car_receive_diag_crc32(crc_text, 9u) == 0xCBF43926u,
          "CRC32/ISO-HDLC vector mismatch");

    car_receive_diag_init(start);
    CHECK(car_rx_frame_rate_millifps == 0u &&
              car_rx_invalid_observation_permille == 0u &&
              car_rx_frame_rate_valid == 0u,
          "rate watch must start unavailable and zeroed");
    CHECK(g_receive_diag_snapshot.window_duration_ms == 0u &&
              (g_receive_diag_snapshot.flags &
               CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID) == 0u,
          "initial window must be marked invalid");
    car_receive_diag_note_frame(65534u, 1u, start + 10u);
    car_receive_diag_note_frame(65535u, 0u, start + 20u);
    car_receive_diag_note_frame(0u, 1u, start + 30u);
    car_receive_diag_note_frame(2u, 1u, start + 40u);
    car_receive_diag_note_frame(2u, 1u, start + 50u);
    car_receive_diag_note_frame(1u, 1u, start + 60u);
    publish_ms = start + CAR_RECEIVE_DIAG_WINDOW_MS;
    car_receive_diag_publish(publish_ms);

    CHECK(g_receive_diag_snapshot.snapshot_ms == publish_ms &&
              g_receive_diag_snapshot.window_duration_ms ==
                  CAR_RECEIVE_DIAG_WINDOW_MS,
          "window duration did not use rollover-safe actual elapsed time");
    CHECK(g_receive_diag_snapshot.accepted_total == 6u &&
              g_receive_diag_snapshot.accepted_window == 6u,
          "parser-accepted packet counter mismatch");
    CHECK(car_rx_frame_rate_valid == 1u &&
              car_rx_frame_rate_millifps == 6000u,
          "one-second accepted-frame rate is not expressed in millifps");
    CHECK(car_rx_invalid_observation_permille == 166u,
          "invalid-observation ratio should be rounded down in permille");
    CHECK(g_receive_diag_snapshot.invalid_observation_total == 1u &&
              g_receive_diag_snapshot.invalid_observation_window == 1u,
          "invalid-observation counter mismatch");
    CHECK(g_receive_diag_snapshot.missing_sequence_total == 1u &&
              g_receive_diag_snapshot.missing_sequence_window == 1u,
          "forward sequence gap estimate mismatch");
    CHECK(g_receive_diag_snapshot.sequence_reject_total == 2u &&
              g_receive_diag_snapshot.sequence_reject_window == 2u &&
              g_receive_diag_snapshot.latest_sequence == 2u,
          "duplicate/backward sequence observations moved the anchor");
    CHECK((g_receive_diag_snapshot.flags &
           (CAR_RECEIVE_DIAG_FLAG_HAVE_SEQUENCE |
            CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID)) ==
              (CAR_RECEIVE_DIAG_FLAG_HAVE_SEQUENCE |
               CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID),
          "sequence/window validity flags are incorrect");
    CHECK(g_receive_diag_snapshot.crc32 ==
              car_receive_diag_snapshot_crc32(&g_receive_diag_snapshot),
          "published snapshot CRC is invalid");

    car_receive_diag_note_frame(4u, 1u,
                                publish_ms + 2u * CAR_RECEIVE_DIAG_WINDOW_MS);
    CHECK(g_receive_diag_snapshot.latest_sequence == 2u,
          "new sequence should update on publication, not note_frame");
    car_receive_diag_publish(publish_ms + 2u * CAR_RECEIVE_DIAG_WINDOW_MS);
    CHECK(g_receive_diag_snapshot.latest_sequence == 4u &&
              g_receive_diag_snapshot.missing_sequence_total == 1u,
          "two-second receive outage did not begin a fresh sequence epoch");
    CHECK(g_receive_diag_snapshot.window_duration_ms ==
              2u * CAR_RECEIVE_DIAG_WINDOW_MS,
          "late window must expose its actual elapsed duration");
    CHECK(car_rx_frame_rate_millifps == 500u &&
              car_rx_invalid_observation_permille == 0u,
          "rate watch must use the actual late-window duration");
    CHECK(g_receive_diag_snapshot.crc32 ==
              car_receive_diag_snapshot_crc32(&g_receive_diag_snapshot),
          "post-outage snapshot CRC is invalid");

    car_receive_diag_init(7000u);
    car_receive_diag_publish(8000u);
    CHECK(g_receive_diag_snapshot.accepted_window == 0u &&
              g_receive_diag_snapshot.invalid_observation_window == 0u &&
              g_receive_diag_snapshot.window_duration_ms == 1000u &&
              (g_receive_diag_snapshot.flags &
               CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID) != 0u,
          "empty completed window must report a valid zero-rate sample");
    CHECK(car_rx_frame_rate_valid == 1u &&
              car_rx_frame_rate_millifps == 0u &&
              car_rx_invalid_observation_permille == 0u,
          "empty completed window must be valid with a zero rate");

    puts("PASS receive diagnostic rates, invalid observations, sequence wrap/gap, outage rebase, and CRC");
    return 0;
}
