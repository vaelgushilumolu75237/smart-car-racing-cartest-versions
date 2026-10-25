#include "reverse_control.h"

#define REVERSE_SYNC0 0xA6u
#define REVERSE_SYNC1 0x5Au
#define REVERSE_VERSION 0x01u
#define REVERSE_PROFILE_STATE_VERSION 0x02u

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
    tx->profile_next_due_ms = now_ms;
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

void car_reverse_profile_state_encode(
    uint16_t sequence,
    uint8_t profile_id,
    uint8_t session_active,
    uint32_t session_token,
    uint32_t mean_forward_wheel_millirev,
    uint8_t frame[CAR_REVERSE_PROFILE_STATE_FRAME_SIZE])
{
    uint16_t crc;

    if (frame == 0) {
        return;
    }
    frame[0] = REVERSE_SYNC0;
    frame[1] = REVERSE_SYNC1;
    frame[2] = REVERSE_PROFILE_STATE_VERSION;
    frame[3] = REVERSE_PROFILE_STATE_VERSION;
    frame[4] = (uint8_t)(sequence & 0xFFu);
    frame[5] = (uint8_t)(sequence >> 8u);
    frame[6] = profile_id <= 3u ? profile_id : 0u;
    frame[7] = session_active != 0u ? 0x01u : 0u;
    frame[8] = (uint8_t)(CAR_REVERSE_PROFILE_STATE_REVISION & 0xFFu);
    frame[9] = (uint8_t)(CAR_REVERSE_PROFILE_STATE_REVISION >> 8u);
    frame[10] = (uint8_t)(session_token & 0xFFu);
    frame[11] = (uint8_t)((session_token >> 8u) & 0xFFu);
    frame[12] = (uint8_t)((session_token >> 16u) & 0xFFu);
    frame[13] = (uint8_t)((session_token >> 24u) & 0xFFu);
    frame[14] = (uint8_t)(mean_forward_wheel_millirev & 0xFFu);
    frame[15] = (uint8_t)((mean_forward_wheel_millirev >> 8u) & 0xFFu);
    frame[16] = (uint8_t)((mean_forward_wheel_millirev >> 16u) & 0xFFu);
    frame[17] = (uint8_t)((mean_forward_wheel_millirev >> 24u) & 0xFFu);
    crc = car_reverse_control_crc16(frame, 18u);
    frame[18] = (uint8_t)(crc & 0xFFu);
    frame[19] = (uint8_t)(crc >> 8u);
}

void car_reverse_control_set_profile_state(
    car_reverse_control_tx_t *tx,
    uint8_t profile_id,
    uint8_t session_active,
    uint32_t session_token,
    uint32_t mean_forward_wheel_millirev)
{
    if (tx == 0) {
        return;
    }
    tx->profile_id = profile_id <= 3u ? profile_id : 0u;
    tx->profile_flags = session_active != 0u ? 0x01u : 0u;
    tx->profile_session_token = session_token;
    tx->profile_mean_forward_wheel_millirev =
        mean_forward_wheel_millirev;
    tx->profile_state_enabled = 1u;
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

static uint8_t emit_v1_byte(car_reverse_control_tx_t *tx,
                            uint8_t keep_permitted,
                            uint8_t *byte_out)
{
    if (tx == 0 || byte_out == 0 || tx->frame_active == 0u) {
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

static uint8_t emit_profile_state_byte(car_reverse_control_tx_t *tx,
                                       uint32_t now_ms,
                                       uint8_t *byte_out)
{
    if (tx == 0 || byte_out == 0 || tx->profile_frame_active == 0u) {
        return 0u;
    }
    *byte_out = tx->profile_frame[tx->profile_frame_offset];
    ++tx->profile_frame_offset;
    if (tx->profile_frame_offset >=
        CAR_REVERSE_PROFILE_STATE_FRAME_SIZE) {
        tx->profile_frame_active = 0u;
        tx->profile_frame_offset = 0u;
        tx->profile_last_tx_ms = now_ms;
        ++tx->profile_tx_count;
    }
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
    return emit_v1_byte(tx, keep_permitted, byte_out);
}

uint8_t car_reverse_control_next_byte_paired(
    car_reverse_control_tx_t *tx,
    uint32_t now_ms,
    uint8_t keep_permitted,
    uint8_t *byte_out)
{
    if (tx == 0 || byte_out == 0) {
        return 0u;
    }
    if (tx->frame_active != 0u) {
        return emit_v1_byte(tx, keep_permitted, byte_out);
    }
    if (tx->profile_frame_active != 0u) {
        return emit_profile_state_byte(tx, now_ms, byte_out);
    }

    /* RESET has first priority; profile status may use the gaps between its
       50 ms frames. KEEP then gets priority over status when both are due. */
    if (begin_frame(tx, now_ms, 0u) != 0u) {
        return emit_v1_byte(tx, 0u, byte_out);
    }
    if (tx->reset_frames_remaining == 0u && tx->reset_pending == 0u &&
        begin_frame(tx, now_ms, keep_permitted) != 0u) {
        return emit_v1_byte(tx, keep_permitted, byte_out);
    }

    if (tx->profile_state_enabled != 0u &&
        time_due(now_ms, tx->profile_next_due_ms) != 0u) {
        car_reverse_profile_state_encode(
            tx->profile_sequence, tx->profile_id, tx->profile_flags,
            tx->profile_session_token,
            tx->profile_mean_forward_wheel_millirev,
            tx->profile_frame);
        ++tx->profile_sequence;
        tx->profile_frame_offset = 0u;
        tx->profile_frame_active = 1u;
        tx->profile_next_due_ms =
            now_ms + CAR_REVERSE_PROFILE_STATE_INTERVAL_MS;
        return emit_profile_state_byte(tx, now_ms, byte_out);
    }

    return 0u;
}
