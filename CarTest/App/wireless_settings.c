#include "wireless_settings.h"

#include <string.h>

#include "car_track_profile.h"
#include "gyro350_config.h"

#define BUILD_GYRO_RATE_MAX_MDPS \
    (CAR_GYRO350_TARGET_YAW_RATE_MDPS > 180000 ? \
     CAR_GYRO350_TARGET_YAW_RATE_MDPS : 180000)

enum {
    SETTINGS_MAGIC = 0x31534757u, /* WGS1 */
    SETTINGS_PAYLOAD_OFFSET = 20u,
    SETTINGS_PAYLOAD_BYTES = 50u,
    SETTINGS_PAYLOAD_CRC_OFFSET = SETTINGS_PAYLOAD_OFFSET + SETTINGS_PAYLOAD_BYTES,
    SETTINGS_RECORD_CRC_OFFSET = SETTINGS_PAYLOAD_CRC_OFFSET + 4u,
    SETTINGS_MIN_RECORD_BYTES = SETTINGS_RECORD_CRC_OFFSET + 4u
};

static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8u);
}

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8u);
    out[2] = (uint8_t)(value >> 16u);
    out[3] = (uint8_t)(value >> 24u);
}

static uint16_t get_u16(const uint8_t *in)
{
    return (uint16_t)in[0] | (uint16_t)((uint16_t)in[1] << 8u);
}

static uint32_t get_u32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8u) |
           ((uint32_t)in[2] << 16u) | ((uint32_t)in[3] << 24u);
}

static void encode_payload(const wireless_settings_t *settings, uint8_t *out)
{
    put_u16(out + 0u, settings->wheel_scale_permille);
    put_u32(out + 2u, settings->gyro_rate_mdps);
    put_u16(out + 6u, settings->vision_gain_permille);
    put_u16(out + 8u, settings->speed_p_gain_permille);
    put_u16(out + 10u, settings->speed_i_gain_permille);
    put_u16(out + 12u, settings->gyro_p_gain_permille);
    put_u16(out + 14u, settings->gyro_i_gain_permille);
    put_u32(out + 16u, settings->ring_progress_mdeg);
    put_u16(out + 20u, settings->ring_exit_ms);
    put_u16(out + 22u, settings->ring_search_ms);
    put_u16(out + 24u, settings->servo_left_us);
    put_u16(out + 26u, settings->servo_right_us);
    put_u32(out + 28u, settings->encoder_left_cpr);
    put_u32(out + 32u, settings->encoder_right_cpr);
    out[36] = (uint8_t)settings->encoder_left_sign;
    out[37] = (uint8_t)settings->encoder_right_sign;
    put_u16(out + 38u, settings->vision_d_gain_permille);
    memset(out + 40u, 0, SETTINGS_PAYLOAD_BYTES - 40u);
}

static void decode_payload(const uint8_t *in, wireless_settings_t *settings)
{
    settings->wheel_scale_permille = get_u16(in + 0u);
    settings->gyro_rate_mdps = get_u32(in + 2u);
    settings->vision_gain_permille = get_u16(in + 6u);
    settings->speed_p_gain_permille = get_u16(in + 8u);
    settings->speed_i_gain_permille = get_u16(in + 10u);
    settings->gyro_p_gain_permille = get_u16(in + 12u);
    settings->gyro_i_gain_permille = get_u16(in + 14u);
    settings->ring_progress_mdeg = get_u32(in + 16u);
    settings->ring_exit_ms = get_u16(in + 20u);
    settings->ring_search_ms = get_u16(in + 22u);
    settings->servo_left_us = get_u16(in + 24u);
    settings->servo_right_us = get_u16(in + 26u);
    settings->encoder_left_cpr = get_u32(in + 28u);
    settings->encoder_right_cpr = get_u32(in + 32u);
    settings->encoder_left_sign = (int8_t)in[36];
    settings->encoder_right_sign = (int8_t)in[37];
    settings->vision_d_gain_permille = get_u16(in + 38u);
}

void wireless_settings_defaults(wireless_settings_t *settings)
{
    if (settings == NULL) return;
    *settings = (wireless_settings_t){
        .wheel_scale_permille = 1000u,
        .gyro_rate_mdps = 120000u,
        .vision_gain_permille = 1000u,
        .vision_d_gain_permille = 1000u,
        .speed_p_gain_permille = 1000u,
        .speed_i_gain_permille = 1000u,
        .gyro_p_gain_permille = 1000u,
        .gyro_i_gain_permille = 1000u,
        .ring_progress_mdeg = 280000u,
        .ring_exit_ms = 300u,
        .ring_search_ms = 3000u,
        .servo_left_us = 1100u,
        .servo_right_us = 2500u,
        .encoder_left_cpr = 59995u,
        .encoder_right_cpr = 60239u,
        .encoder_left_sign = 1,
        .encoder_right_sign = -1
    };
}

void wireless_settings_defaults_for_profile(wireless_settings_t *settings,
                                            uint8_t profile_key)
{
    uint32_t yaw_mdps = 120000u;

    if (settings == NULL) return;
    wireless_settings_defaults(settings);
    /* Current C builds use their source-selected ring rate. Legacy speed
       profiles retain their original defaults. The runtime rebases the rate
       after loading Flash so saved legacy values cannot mask source edits. */
    switch (profile_key) {
    case WIRELESS_PROFILE_B_RING_BASE:
        yaw_mdps = 64935u;
        break;
    case WIRELESS_PROFILE_C_RING:
#if CARTEST_TRACK_PROFILE == 3 && CARTEST_SPEED_PERCENT == 100
        yaw_mdps = CAR_GYRO350_TARGET_YAW_RATE_MDPS;
#else
        yaw_mdps = 85714u;
#endif
        break;
    case WIRELESS_PROFILE_C_RING_SPEED120:
        yaw_mdps = 102857u;
        break;
    case WIRELESS_PROFILE_C_RING_SPEED130:
        yaw_mdps = 111429u;
        break;
    case WIRELESS_PROFILE_C_RING_SPEED140:
        yaw_mdps = 120000u;
        break;
    case WIRELESS_PROFILE_A_NO_RING:
    default:
        /* A does not use gyro authority, but keeps a valid diagnostic value. */
        yaw_mdps = 120000u;
        break;
    }
    settings->gyro_rate_mdps = yaw_mdps;
}

uint8_t wireless_settings_build_profile_key(void)
{
#if CARTEST_TRACK_PROFILE == 1
    return WIRELESS_PROFILE_A_NO_RING;
#elif CARTEST_TRACK_PROFILE == 2
    return WIRELESS_PROFILE_B_RING_BASE;
#elif CARTEST_TRACK_PROFILE == 3
#if CARTEST_SPEED_PERCENT == 120u
    return WIRELESS_PROFILE_C_RING_SPEED120;
#elif CARTEST_SPEED_PERCENT == 130u
    return WIRELESS_PROFILE_C_RING_SPEED130;
#elif CARTEST_SPEED_PERCENT == 140u
    return WIRELESS_PROFILE_C_RING_SPEED140;
#else
    return WIRELESS_PROFILE_C_RING;
#endif
#else
    /* Host tests and legacy builds use the ring-base record. */
    return WIRELESS_PROFILE_B_RING_BASE;
#endif
}

static uint8_t u16_range(uint16_t value, uint16_t low, uint16_t high)
{
    return (value >= low && value <= high) ? 1u : 0u;
}

static uint8_t u32_range(uint32_t value, uint32_t low, uint32_t high)
{
    return (value >= low && value <= high) ? 1u : 0u;
}

uint8_t wireless_settings_is_valid(const wireless_settings_t *settings)
{
    if (settings == NULL) return 0u;
    return (u16_range(settings->wheel_scale_permille, 250u, 1000u) &&
            u32_range(settings->gyro_rate_mdps, 30000u, BUILD_GYRO_RATE_MAX_MDPS) &&
            u16_range(settings->vision_gain_permille, 500u, 1500u) &&
            u16_range(settings->vision_d_gain_permille, 500u, 1500u) &&
            u16_range(settings->speed_p_gain_permille, 500u, 1500u) &&
            u16_range(settings->speed_i_gain_permille, 500u, 1500u) &&
            u16_range(settings->gyro_p_gain_permille, 500u, 1500u) &&
            u16_range(settings->gyro_i_gain_permille, 500u, 1500u) &&
            u32_range(settings->ring_progress_mdeg, 180000u, 540000u) &&
            u16_range(settings->ring_exit_ms, 100u, 1000u) &&
            u16_range(settings->ring_search_ms, 500u, 5000u) &&
            u16_range(settings->servo_left_us, 1100u, 2500u) &&
            u16_range(settings->servo_right_us, 1100u, 2500u) &&
            settings->servo_left_us < 1530u && settings->servo_right_us > 1530u &&
            settings->encoder_left_cpr > 0u && settings->encoder_left_cpr <= 200000u &&
            settings->encoder_right_cpr > 0u && settings->encoder_right_cpr <= 200000u &&
            (settings->encoder_left_sign == 1 || settings->encoder_left_sign == -1) &&
            (settings->encoder_right_sign == 1 || settings->encoder_right_sign == -1)) ? 1u : 0u;
}

uint8_t wireless_settings_param_is_moving_allowed(uint8_t id)
{
    return (id == WIRELESS_PARAM_WHEEL_SCALE_PERMILLE ||
            id == WIRELESS_PARAM_GYRO_RATE_MDPS ||
            id == WIRELESS_PARAM_VISION_GAIN_PERMILLE ||
            id == WIRELESS_PARAM_VISION_D_GAIN_PERMILLE ||
            id == WIRELESS_PARAM_SPEED_P_GAIN_PERMILLE ||
            id == WIRELESS_PARAM_SPEED_I_GAIN_PERMILLE ||
            id == WIRELESS_PARAM_GYRO_P_GAIN_PERMILLE ||
            id == WIRELESS_PARAM_GYRO_I_GAIN_PERMILLE) ? 1u : 0u;
}

static uint8_t group_accepts(uint8_t group, uint8_t id)
{
    if (group == WIRELESS_SETTINGS_GROUP_DRIVING) {
        return id == WIRELESS_PARAM_WHEEL_SCALE_PERMILLE ||
               id == WIRELESS_PARAM_VISION_GAIN_PERMILLE ||
               id == WIRELESS_PARAM_VISION_D_GAIN_PERMILLE ||
               id == WIRELESS_PARAM_SPEED_P_GAIN_PERMILLE ||
               id == WIRELESS_PARAM_SPEED_I_GAIN_PERMILLE;
    }
    if (group == WIRELESS_SETTINGS_GROUP_GYRO) {
        return id == WIRELESS_PARAM_GYRO_RATE_MDPS ||
               id == WIRELESS_PARAM_GYRO_P_GAIN_PERMILLE ||
               id == WIRELESS_PARAM_GYRO_I_GAIN_PERMILLE;
    }
    if (group == WIRELESS_SETTINGS_GROUP_PARKED_RING) {
        return id == WIRELESS_PARAM_RING_PROGRESS_MDEG ||
               id == WIRELESS_PARAM_RING_EXIT_MS ||
               id == WIRELESS_PARAM_RING_SEARCH_MS;
    }
    if (group == WIRELESS_SETTINGS_GROUP_HARDWARE) {
        return id >= WIRELESS_PARAM_SERVO_LEFT_US &&
               id <= WIRELESS_PARAM_ENCODER_RIGHT_SIGN;
    }
    return 0u;
}

static uint8_t raw_value_is_valid(uint8_t id, int32_t value)
{
    switch (id) {
    case WIRELESS_PARAM_WHEEL_SCALE_PERMILLE: return value>=250 && value<=1000;
    case WIRELESS_PARAM_GYRO_RATE_MDPS: return value>=30000 && value<=BUILD_GYRO_RATE_MAX_MDPS;
    case WIRELESS_PARAM_VISION_GAIN_PERMILLE:
    case WIRELESS_PARAM_VISION_D_GAIN_PERMILLE:
    case WIRELESS_PARAM_SPEED_P_GAIN_PERMILLE:
    case WIRELESS_PARAM_SPEED_I_GAIN_PERMILLE:
    case WIRELESS_PARAM_GYRO_P_GAIN_PERMILLE:
    case WIRELESS_PARAM_GYRO_I_GAIN_PERMILLE: return value>=500 && value<=1500;
    case WIRELESS_PARAM_RING_PROGRESS_MDEG: return value>=180000 && value<=540000;
    case WIRELESS_PARAM_RING_EXIT_MS: return value>=100 && value<=1000;
    case WIRELESS_PARAM_RING_SEARCH_MS: return value>=500 && value<=5000;
    case WIRELESS_PARAM_SERVO_LEFT_US: return value>=1100 && value<1530;
    case WIRELESS_PARAM_SERVO_RIGHT_US: return value>1530 && value<=2500;
    case WIRELESS_PARAM_ENCODER_LEFT_CPR:
    case WIRELESS_PARAM_ENCODER_RIGHT_CPR: return value>0 && value<=200000;
    case WIRELESS_PARAM_ENCODER_LEFT_SIGN:
    case WIRELESS_PARAM_ENCODER_RIGHT_SIGN: return value==1 || value==-1;
    default: return 0u;
    }
}

static uint8_t set_one(wireless_settings_t *settings, uint8_t id, int32_t value)
{
    switch (id) {
    case WIRELESS_PARAM_WHEEL_SCALE_PERMILLE: settings->wheel_scale_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_GYRO_RATE_MDPS: settings->gyro_rate_mdps = (uint32_t)value; return 1u;
    case WIRELESS_PARAM_VISION_GAIN_PERMILLE: settings->vision_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_VISION_D_GAIN_PERMILLE: settings->vision_d_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_SPEED_P_GAIN_PERMILLE: settings->speed_p_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_SPEED_I_GAIN_PERMILLE: settings->speed_i_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_GYRO_P_GAIN_PERMILLE: settings->gyro_p_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_GYRO_I_GAIN_PERMILLE: settings->gyro_i_gain_permille = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_RING_PROGRESS_MDEG: settings->ring_progress_mdeg = (uint32_t)value; return 1u;
    case WIRELESS_PARAM_RING_EXIT_MS: settings->ring_exit_ms = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_RING_SEARCH_MS: settings->ring_search_ms = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_SERVO_LEFT_US: settings->servo_left_us = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_SERVO_RIGHT_US: settings->servo_right_us = (uint16_t)value; return 1u;
    case WIRELESS_PARAM_ENCODER_LEFT_CPR: settings->encoder_left_cpr = (uint32_t)value; return 1u;
    case WIRELESS_PARAM_ENCODER_RIGHT_CPR: settings->encoder_right_cpr = (uint32_t)value; return 1u;
    case WIRELESS_PARAM_ENCODER_LEFT_SIGN: settings->encoder_left_sign = (int8_t)value; return 1u;
    case WIRELESS_PARAM_ENCODER_RIGHT_SIGN: settings->encoder_right_sign = (int8_t)value; return 1u;
    default: return 0u;
    }
}

wireless_settings_status_t wireless_settings_apply_group(
    wireless_settings_t *destination, uint8_t group,
    const wireless_param_entry_t *entries, uint8_t count, uint8_t moving)
{
    wireless_settings_t candidate;
    uint8_t i;
    if (destination == NULL || entries == NULL || count == 0u || count > 16u ||
        group == 0u || group > WIRELESS_SETTINGS_GROUP_COUNT) {
        return group == 0u || group > WIRELESS_SETTINGS_GROUP_COUNT
                   ? WIRELESS_SETTINGS_BAD_GROUP : WIRELESS_SETTINGS_BAD_PARAM;
    }
    if (moving != 0u && group != WIRELESS_SETTINGS_GROUP_DRIVING &&
        group != WIRELESS_SETTINGS_GROUP_GYRO) return WIRELESS_SETTINGS_NOT_PARKED;
    candidate = *destination;
    for (i = 0u; i < count; ++i) {
        uint8_t previous;
        for (previous = 0u; previous < i; ++previous) {
            if (entries[previous].id == entries[i].id)
                return WIRELESS_SETTINGS_BAD_PARAM;
        }
        if (!group_accepts(group, entries[i].id) ||
            (moving != 0u && !wireless_settings_param_is_moving_allowed(entries[i].id))) {
            return WIRELESS_SETTINGS_BAD_PARAM;
        }
        /* Validate the signed wire value before narrowing to uint16/int8.
           An oversized or negative value must not wrap into an allowed one. */
        if (!raw_value_is_valid(entries[i].id, entries[i].value))
            return WIRELESS_SETTINGS_OUT_OF_RANGE;
        if (!set_one(&candidate, entries[i].id, entries[i].value))
            return WIRELESS_SETTINGS_BAD_PARAM;
    }
    if (!wireless_settings_is_valid(&candidate)) return WIRELESS_SETTINGS_OUT_OF_RANGE;
    *destination = candidate;
    return WIRELESS_SETTINGS_OK;
}

uint32_t wireless_settings_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffu;
    size_t i;
    if (data == NULL) return 0u;
    for (i = 0u; i < length; ++i) {
        uint32_t bit;
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit)
            crc = (crc & 1u) ? (crc >> 1u) ^ 0xedb88320u : crc >> 1u;
    }
    return crc ^ 0xffffffffu;
}

size_t wireless_settings_encode_slot(const wireless_settings_t *settings,
                                     uint32_t generation, uint8_t *out,
                                     size_t capacity)
{
    uint32_t payload_crc;
    if (settings == NULL || out == NULL || capacity < WIRELESS_SETTINGS_SLOT_BYTES ||
        !wireless_settings_is_valid(settings)) return 0u;
    memset(out, 0xff, WIRELESS_SETTINGS_SLOT_BYTES);
    put_u32(out + 0u, SETTINGS_MAGIC);
    put_u32(out + 4u, WIRELESS_SETTINGS_SCHEMA);
    put_u32(out + 8u, generation);
    put_u32(out + 12u, SETTINGS_PAYLOAD_BYTES);
    encode_payload(settings, out + SETTINGS_PAYLOAD_OFFSET);
    payload_crc = wireless_settings_crc32(out + SETTINGS_PAYLOAD_OFFSET, SETTINGS_PAYLOAD_BYTES);
    put_u32(out + SETTINGS_PAYLOAD_CRC_OFFSET, payload_crc);
    put_u32(out + SETTINGS_RECORD_CRC_OFFSET,
            wireless_settings_crc32(out, SETTINGS_RECORD_CRC_OFFSET));
    return WIRELESS_SETTINGS_SLOT_BYTES;
}

uint8_t wireless_settings_decode_slot(const uint8_t *record, size_t length,
                                       wireless_settings_t *settings,
                                       uint32_t *generation)
{
    wireless_settings_t candidate;
    if (record == NULL || settings == NULL || generation == NULL ||
        length < SETTINGS_MIN_RECORD_BYTES ||
        get_u32(record + 0u) != SETTINGS_MAGIC ||
        get_u32(record + 4u) != WIRELESS_SETTINGS_SCHEMA ||
        get_u32(record + 12u) != SETTINGS_PAYLOAD_BYTES ||
        get_u32(record + SETTINGS_RECORD_CRC_OFFSET) !=
            wireless_settings_crc32(record, SETTINGS_RECORD_CRC_OFFSET) ||
        get_u32(record + SETTINGS_PAYLOAD_CRC_OFFSET) !=
            wireless_settings_crc32(record + SETTINGS_PAYLOAD_OFFSET, SETTINGS_PAYLOAD_BYTES)) {
        return 0u;
    }
    decode_payload(record + SETTINGS_PAYLOAD_OFFSET, &candidate);
    if (!wireless_settings_is_valid(&candidate)) return 0u;
    *settings = candidate;
    *generation = get_u32(record + 8u);
    return 1u;
}

uint8_t wireless_settings_generation_is_newer(uint32_t candidate, uint32_t current)
{
    uint32_t delta = candidate - current;
    return delta != 0u && delta < 0x80000000u;
}

uint8_t wireless_settings_select_slot(const uint8_t *slot_a, const uint8_t *slot_b,
                                      size_t length, wireless_settings_t *settings,
                                      uint32_t *generation, uint8_t *selected_slot)
{
    wireless_settings_t a, b;
    uint32_t ga = 0u, gb = 0u;
    uint8_t va = wireless_settings_decode_slot(slot_a, length, &a, &ga);
    uint8_t vb = wireless_settings_decode_slot(slot_b, length, &b, &gb);
    if (!va && !vb) return 0u;
    if (vb && (!va || wireless_settings_generation_is_newer(gb, ga))) {
        *settings = b; *generation = gb; if (selected_slot) *selected_slot = 1u;
    } else {
        *settings = a; *generation = ga; if (selected_slot) *selected_slot = 0u;
    }
    return 1u;
}
