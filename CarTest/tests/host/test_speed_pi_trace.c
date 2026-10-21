#include "speed_pi_trace.h"
#include "speed_pi_diag.h"

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
    static car_speed_pi_trace_sample_t chronological[
        CAR_SPEED_PI_TRACE_CAPACITY];
    car_speed_pi_trace_values_t values = {0};
    car_speed_pi_trace_sample_t tampered;
    uint8_t build_id_nonzero = 0u;
    uint32_t i;

    CHECK(sizeof(g_speed_pi_trace) == CAR_SPEED_PI_TRACE_SECTION_SIZE &&
              CAR_SPEED_PI_TRACE_SECTION_SIZE == 98368u,
          "trace section size changed");
    CHECK(((uintptr_t)&g_speed_pi_trace & 31u) == 0u,
          "trace object is not cache-line aligned");
    CHECK(car_speed_pi_trace_crc32(crc_text, 9u) == 0xCBF43926u,
          "CRC32/ISO-HDLC vector mismatch");

    car_speed_pi_trace_init();
    CHECK(g_speed_pi_trace.header.state == CAR_SPEED_PI_TRACE_STATE_IDLE &&
              g_speed_pi_trace.header.header_crc32 ==
                  car_speed_pi_trace_header_crc32(&g_speed_pi_trace.header),
          "IDLE header CRC/state invalid");
    car_speed_pi_trace_begin(0x10203040u, 100u,
                             CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL);
    CHECK(g_speed_pi_trace.header.state == CAR_SPEED_PI_TRACE_STATE_ACTIVE &&
              g_speed_pi_trace.header.start_token == 0x10203040u,
          "trace session identity was not recorded");
    for (i = 0u; i < sizeof(g_speed_pi_trace.header.build_id); ++i) {
        build_id_nonzero |= g_speed_pi_trace.header.build_id[i];
    }
    CHECK(build_id_nonzero != 0u,
          "trace build ID was not linked into the snapshot");

    values.record_ms = 140u;
    values.sample_ms = 120u;
    values.vision_sequence = 65535u;
    values.vision_arrival_ms = 135u;
    values.pi_type = CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL;
    values.speed_valid = 1u;
    values.armed = 1u;
    values.proposal_valid = 1u;
    values.target_left_millirps = 4800;
    values.target_right_millirps = 2584;
    values.measured_left_millirps = 4210;
    values.measured_right_millirps = 2500;
    values.requested_left_permille = 920u;
    values.requested_right_permille = 600u;
    values.applied_left_permille = 900u;
    values.applied_right_permille = 580u;
    car_speed_pi_trace_append(&values);
    values.record_ms = 160u;
    values.sample_ms = 140u;
    values.vision_sequence = 0u;
    values.vision_arrival_ms = 155u;
    values.applied_left_permille = 920u;
    car_speed_pi_trace_append(&values);

    CHECK(g_speed_pi_trace.header.state == CAR_SPEED_PI_TRACE_STATE_ACTIVE,
          "active trace was incorrectly marked frozen");
    CHECK(g_speed_pi_trace.samples[0].record_ms == 140u &&
              g_speed_pi_trace.samples[0].sample_ms == 120u &&
              g_speed_pi_trace.samples[0].vision_arrival_ms == 135u &&
              g_speed_pi_trace.samples[0].vision_sequence == 65535u,
          "record, speed-sample, and vision timestamps were conflated");
    CHECK(g_speed_pi_trace.samples[1].vision_sequence == 0u &&
              g_speed_pi_trace.samples[1].sample_index == 1u,
          "sequence rollover/sample indexing failed");
    CHECK(g_speed_pi_trace.samples[0].crc32 ==
              car_speed_pi_trace_sample_crc32(&g_speed_pi_trace.samples[0]) &&
              g_speed_pi_trace.samples[1].crc32 ==
              car_speed_pi_trace_sample_crc32(&g_speed_pi_trace.samples[1]),
          "per-sample CRC invalid");

    car_speed_pi_trace_freeze(170u);
    CHECK(g_speed_pi_trace.header.state == CAR_SPEED_PI_TRACE_STATE_FROZEN &&
              g_speed_pi_trace.header.stop_ms == 170u &&
              g_speed_pi_trace.header.count == 2u &&
              g_speed_pi_trace.header.total_samples == 2u &&
              g_speed_pi_trace.header.next_index == 2u &&
              g_speed_pi_trace.header.lost_samples == 0u,
          "freeze metadata is inconsistent");
    CHECK(g_speed_pi_trace.header.records_crc32 ==
              car_speed_pi_trace_crc32(
                  g_speed_pi_trace.samples,
                  2u * CAR_SPEED_PI_TRACE_SAMPLE_SIZE),
          "frozen chronological records CRC mismatch");
    CHECK(g_speed_pi_trace.header.header_crc32 ==
              car_speed_pi_trace_header_crc32(&g_speed_pi_trace.header),
          "frozen header CRC invalid");
    car_speed_pi_trace_append(&values);
    CHECK(g_speed_pi_trace.header.total_samples == 2u,
          "frozen trace accepted a later sample");

    car_speed_pi_trace_begin(0xAABBCCDDu, 200u,
                             CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL);
    CHECK(g_speed_pi_trace.header.start_token == 0xAABBCCDDu &&
              g_speed_pi_trace.header.state == CAR_SPEED_PI_TRACE_STATE_ACTIVE,
          "new session did not replace the frozen session identity");
    values.record_ms = 200u;
    values.vision_arrival_ms = 195u;
    for (i = 0u; i < CAR_SPEED_PI_TRACE_CAPACITY + 5u; ++i) {
        values.record_ms = 200u + i * 20u;
        values.sample_ms = 195u + i * 20u;
        values.vision_sequence = (uint16_t)i;
        car_speed_pi_trace_append(&values);
    }
    car_speed_pi_trace_freeze(200u +
                              (CAR_SPEED_PI_TRACE_CAPACITY + 5u) * 20u);
    CHECK(g_speed_pi_trace.header.count == CAR_SPEED_PI_TRACE_CAPACITY &&
              g_speed_pi_trace.header.total_samples ==
                  CAR_SPEED_PI_TRACE_CAPACITY + 5u &&
              g_speed_pi_trace.header.next_index == 5u &&
              g_speed_pi_trace.header.lost_samples == 5u,
          "full-ring overwrite accounting is incorrect");
    for (i = 0u; i < CAR_SPEED_PI_TRACE_CAPACITY; ++i) {
        const uint16_t ring_index = (uint16_t)(
            (g_speed_pi_trace.header.next_index + i) %
            CAR_SPEED_PI_TRACE_CAPACITY);
        chronological[i] = g_speed_pi_trace.samples[ring_index];
        CHECK(chronological[i].sample_index == i + 5u,
              "full-ring chronological order is incorrect");
        CHECK(chronological[i].crc32 ==
                  car_speed_pi_trace_sample_crc32(&chronological[i]),
              "full-ring sample CRC is invalid");
    }
    CHECK(g_speed_pi_trace.header.records_crc32 ==
              car_speed_pi_trace_crc32(
                  chronological,
                  CAR_SPEED_PI_TRACE_CAPACITY *
                      CAR_SPEED_PI_TRACE_SAMPLE_SIZE),
          "full-ring chronological CRC mismatch");
    tampered = chronological[0];
    tampered.applied_left_permille ^= 1u;
    CHECK(tampered.crc32 != car_speed_pi_trace_sample_crc32(&tampered),
          "record mutation was not detected by its per-sample CRC");
    CHECK(g_speed_pi_trace.header.header_crc32 ==
              car_speed_pi_trace_header_crc32(&g_speed_pi_trace.header),
          "wrapped frozen header CRC invalid");

    car_speed_pi_trace_begin(0x55667788u, 90000u,
                             CAR_SPEED_PI_DIAG_TYPE_POSITION);
    CHECK(g_speed_pi_trace.header.total_samples == 0u &&
              g_speed_pi_trace.header.count == 0u &&
              g_speed_pi_trace.header.next_index == 0u &&
              g_speed_pi_trace.header.start_token == 0x55667788u,
          "begin-new-session did not reset prior ring accounting");

    puts("PASS speed PI trace ABI, timestamp pairing, session/build ID, freeze and CRC");
    return 0;
}
