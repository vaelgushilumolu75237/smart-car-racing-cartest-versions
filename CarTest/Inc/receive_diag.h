#ifndef CARTEST_RECEIVE_DIAG_H
#define CARTEST_RECEIVE_DIAG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_RECEIVE_DIAG_MAGIC = 0x47414952u, /* Little-endian "RIAG". */
    CAR_RECEIVE_DIAG_VERSION = 1u,
    CAR_RECEIVE_DIAG_SIZE = 96u,
    CAR_RECEIVE_DIAG_WINDOW_MS = 1000u,
    CAR_RECEIVE_DIAG_FLAG_HAVE_SEQUENCE = 1u << 0,
    CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID = 1u << 1
};

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define CAR_RECEIVE_DIAG_PACKED
#else
#define CAR_RECEIVE_DIAG_PACKED __attribute__((packed))
#endif

typedef struct CAR_RECEIVE_DIAG_PACKED {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t generation;
    uint32_t crc32;
    uint32_t snapshot_ms;
    uint32_t window_start_ms;
    uint32_t window_duration_ms;
    uint32_t accepted_total;
    uint32_t accepted_window;
    uint32_t invalid_observation_total;
    uint32_t invalid_observation_window;
    uint32_t missing_sequence_total;
    uint32_t missing_sequence_window;
    uint32_t sequence_reject_total;
    uint32_t sequence_reject_window;
    uint32_t parser_error_total;
    uint16_t latest_sequence;
    uint16_t flags;
    uint32_t reserved[7];
} car_receive_diag_snapshot_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef CAR_RECEIVE_DIAG_PACKED

extern car_receive_diag_snapshot_t g_receive_diag_snapshot;

/* Directly readable last-completed window summary. The rate counts frames
   accepted by the vision parser, including invalid observations; it is not a
   measurement of the camera sensor's acquisition FPS. */
extern volatile uint32_t car_rx_frame_rate_millifps;
extern volatile uint32_t car_rx_invalid_observation_permille;
extern volatile uint32_t car_rx_frame_rate_valid;

/* This diagnostic is independent of the PE7 run session and has one main-line
   writer. Accepted frames mean CRC/format-valid protocol packets delivered to
   the controller, not the camera's advertised acquisition rate. */
void car_receive_diag_init(uint32_t now_ms);
void car_receive_diag_note_frame(uint16_t sequence,
                                 uint8_t valid_observation,
                                 uint32_t arrival_ms);
void car_receive_diag_note_parser_error(uint32_t now_ms);
void car_receive_diag_publish(uint32_t now_ms);
uint32_t car_receive_diag_crc32(const void *bytes, uint32_t length);
uint32_t car_receive_diag_snapshot_crc32(
    const car_receive_diag_snapshot_t *snapshot);

#if defined(__cplusplus)
static_assert(sizeof(car_receive_diag_snapshot_t) == CAR_RECEIVE_DIAG_SIZE,
              "receive diagnostic ABI must remain 96 bytes");
#else
_Static_assert(sizeof(car_receive_diag_snapshot_t) == CAR_RECEIVE_DIAG_SIZE,
               "receive diagnostic ABI must remain 96 bytes");
#endif

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_RECEIVE_DIAG_H */
