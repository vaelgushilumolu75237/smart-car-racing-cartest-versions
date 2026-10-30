#ifndef CARTEST_REVERSE_CONTROL_H
#define CARTEST_REVERSE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_REVERSE_CONTROL_FRAME_SIZE = 8u,
    CAR_REVERSE_PROFILE_STATE_FRAME_SIZE = 20u,
    CAR_REVERSE_PROFILE_STATE_INTERVAL_MS = 100u,
    CAR_REVERSE_PROFILE_STATE_REVISION = 1u,
    CAR_REVERSE_PROFILE_CAP_CAMERA_DIAGNOSTICS = 0x02u,
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
    uint8_t profile_frame[CAR_REVERSE_PROFILE_STATE_FRAME_SIZE];
    uint8_t profile_frame_offset;
    uint8_t profile_frame_active;
    uint8_t profile_state_enabled;
    uint16_t profile_sequence;
    uint8_t profile_id;
    uint8_t profile_flags;
    uint32_t profile_session_token;
    uint32_t profile_mean_forward_wheel_millirev;
    uint32_t profile_next_due_ms;
    uint32_t profile_last_tx_ms;
    uint32_t profile_tx_count;
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

/* Update the latest profile pairing/status values used by V2 frames. */
void car_reverse_control_set_profile_state(
    car_reverse_control_tx_t *tx,
    uint8_t profile_id,
    uint8_t session_active,
    uint32_t session_token,
    uint32_t mean_forward_wheel_millirev);

/* V1 RESET/KEEP and V2 profile-state share one byte stream without interleave. */
uint8_t car_reverse_control_next_byte_paired(
    car_reverse_control_tx_t *tx,
    uint32_t now_ms,
    uint8_t keep_permitted,
    uint8_t *byte_out);

void car_reverse_profile_state_encode(
    uint16_t sequence,
    uint8_t profile_id,
    uint8_t session_active,
    uint32_t session_token,
    uint32_t mean_forward_wheel_millirev,
    uint8_t frame[CAR_REVERSE_PROFILE_STATE_FRAME_SIZE]);

void car_reverse_control_encode(uint8_t command,
                                uint16_t sequence,
                                uint8_t frame[CAR_REVERSE_CONTROL_FRAME_SIZE]);

uint16_t car_reverse_control_crc16(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_REVERSE_CONTROL_H */
