#ifndef CARTEST_REVERSE_CONTROL_H
#define CARTEST_REVERSE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_REVERSE_CONTROL_FRAME_SIZE = 8u,
    CAR_REVERSE_CONTROL_RESET = 0u,
    CAR_REVERSE_CONTROL_KEEP = 1u,
    CAR_REVERSE_CONTROL_RESET_PHASE_MS = 250u,
    CAR_REVERSE_CONTROL_INTERVAL_MS = 50u,
    CAR_REVERSE_CONTROL_RESET_FRAMES = 5u
};

typedef struct {
    uint16_t sequence;
    uint8_t frame[CAR_REVERSE_CONTROL_FRAME_SIZE];
    uint8_t frame_offset;
    uint8_t active_command;
    uint8_t frame_active;
    uint8_t reset_pending;
    uint8_t reset_frames_remaining;
    uint8_t reset_phase_complete;
    uint8_t keep_active;
    uint32_t next_due_ms;
    uint32_t reset_generation;
} car_reverse_control_tx_t;

void car_reverse_control_init(car_reverse_control_tx_t *tx, uint32_t now_ms);

/* Finish an in-progress frame before beginning the new RESET phase. */
void car_reverse_control_request_reset(car_reverse_control_tx_t *tx,
                                       uint32_t now_ms);

/* Poll one byte at a time. The caller writes it only when USART TXE is set. */
uint8_t car_reverse_control_next_byte(car_reverse_control_tx_t *tx,
                                      uint32_t now_ms,
                                      uint8_t keep_permitted,
                                      uint8_t *byte_out);

void car_reverse_control_encode(uint8_t command,
                                uint16_t sequence,
                                uint8_t frame[CAR_REVERSE_CONTROL_FRAME_SIZE]);

uint16_t car_reverse_control_crc16(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_REVERSE_CONTROL_H */
