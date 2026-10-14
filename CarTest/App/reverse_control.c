#include "reverse_control.h"

#define REVERSE_SYNC0 0xA6u
#define REVERSE_SYNC1 0x5Au
#define REVERSE_VERSION 0x01u

uint16_t car_reverse_control_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFu;
    uint32_t index;
    uint8_t bit;

    if (data == 0) {
        return crc;
    }
    for (index = 0u; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8u;
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) != 0u
                      ? (uint16_t)((crc << 1u) ^ 0x1021u)
                      : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

void car_reverse_control_encode(
    uint8_t command,
    uint16_t sequence,
    uint8_t frame[CAR_REVERSE_CONTROL_FRAME_SIZE])
{
    uint16_t crc;

    if (frame == 0) {
        return;
    }
    frame[0] = REVERSE_SYNC0;
    frame[1] = REVERSE_SYNC1;
    frame[2] = REVERSE_VERSION;
    frame[3] = command;
    frame[4] = (uint8_t)(sequence & 0xFFu);
    frame[5] = (uint8_t)(sequence >> 8u);
    crc = car_reverse_control_crc16(frame, 6u);
    frame[6] = (uint8_t)(crc & 0xFFu);
    frame[7] = (uint8_t)(crc >> 8u);
}

void car_reverse_control_init(car_reverse_control_tx_t *tx, uint32_t now_ms)
{
    if (tx == 0) {
        return;
    }
    *tx = (car_reverse_control_tx_t){0};
    tx->reset_pending = 1u;
    tx->next_due_ms = now_ms;
}

void car_reverse_control_request_reset(car_reverse_control_tx_t *tx,
                                       uint32_t now_ms)
{
    if (tx == 0) {
        return;
    }
    ++tx->reset_generation;
    tx->reset_pending = 1u;
    tx->reset_frames_remaining = 0u;
    tx->reset_phase_complete = 0u;
    tx->keep_active = 0u;
    tx->next_due_ms = now_ms;
}

static uint8_t time_due(uint32_t now_ms, uint32_t due_ms)
{
    return (uint8_t)((int32_t)(now_ms - due_ms) >= 0 ? 1u : 0u);
}

static uint8_t begin_frame(car_reverse_control_tx_t *tx,
                           uint32_t now_ms,
                           uint8_t keep_permitted)
{
    uint8_t command;

    if (tx->reset_pending != 0u) {
        tx->reset_pending = 0u;
        tx->reset_frames_remaining = CAR_REVERSE_CONTROL_RESET_FRAMES;
        tx->reset_phase_complete = 0u;
        tx->keep_active = 0u;
    }

    if (tx->reset_frames_remaining != 0u) {
        if (time_due(now_ms, tx->next_due_ms) == 0u) {
            return 0u;
        }
        command = CAR_REVERSE_CONTROL_RESET;
        tx->next_due_ms = now_ms + CAR_REVERSE_CONTROL_INTERVAL_MS;
    } else {
        tx->reset_phase_complete = 1u;
        if (keep_permitted == 0u ||
            time_due(now_ms, tx->next_due_ms) == 0u) {
            return 0u;
        }
        command = CAR_REVERSE_CONTROL_KEEP;
        tx->next_due_ms = now_ms + CAR_REVERSE_CONTROL_INTERVAL_MS;
    }

    tx->active_command = command;
    tx->frame_offset = 0u;
    tx->frame_active = 1u;
    car_reverse_control_encode(command, tx->sequence, tx->frame);
    ++tx->sequence;
    return 1u;
}

uint8_t car_reverse_control_next_byte(car_reverse_control_tx_t *tx,
                                      uint32_t now_ms,
                                      uint8_t keep_permitted,
                                      uint8_t *byte_out)
{
    if (tx == 0 || byte_out == 0) {
        return 0u;
    }
    if (tx->frame_active == 0u &&
        begin_frame(tx, now_ms, keep_permitted) == 0u) {
        return 0u;
    }

    *byte_out = tx->frame[tx->frame_offset];
    ++tx->frame_offset;
    if (tx->frame_offset >= CAR_REVERSE_CONTROL_FRAME_SIZE) {
        tx->frame_active = 0u;
        tx->frame_offset = 0u;
        if (tx->active_command == CAR_REVERSE_CONTROL_RESET) {
            if (tx->reset_pending == 0u &&
                tx->reset_frames_remaining != 0u) {
                --tx->reset_frames_remaining;
            }
            if (tx->reset_pending == 0u &&
                tx->reset_frames_remaining == 0u) {
                tx->reset_phase_complete = 1u;
            }
        } else {
            tx->keep_active = tx->reset_pending == 0u &&
                                      tx->reset_frames_remaining == 0u &&
                                      keep_permitted != 0u
                                  ? 1u : 0u;
        }
    }
    return 1u;
}
