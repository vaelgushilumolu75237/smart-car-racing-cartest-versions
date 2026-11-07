#ifndef CARTEST_WIRELESS_SETTINGS_H
#define CARTEST_WIRELESS_SETTINGS_H

/*
 * Runtime settings exposed by the wireless service.  Values are deliberately
 * integer based: the same representation is used by the MCU command parser,
 * the persistent record and the desktop tool.  There is no floating point or
 * target ABI dependency in this interface.
 */
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WIRELESS_SETTINGS_SCHEMA = 1u,
    WIRELESS_SETTINGS_SLOT_BYTES = 128u,
    WIRELESS_SETTINGS_GROUP_DRIVING = 1u,
    WIRELESS_SETTINGS_GROUP_GYRO = 2u,
    WIRELESS_SETTINGS_GROUP_PARKED_RING = 3u,
    WIRELESS_SETTINGS_GROUP_HARDWARE = 4u,
    WIRELESS_SETTINGS_GROUP_COUNT = 4u
};

typedef enum {
    WIRELESS_PARAM_WHEEL_SCALE_PERMILLE = 1u,
    WIRELESS_PARAM_GYRO_RATE_MDPS = 2u,
    WIRELESS_PARAM_VISION_GAIN_PERMILLE = 3u,
    WIRELESS_PARAM_SPEED_P_GAIN_PERMILLE = 4u,
    WIRELESS_PARAM_SPEED_I_GAIN_PERMILLE = 5u,
    WIRELESS_PARAM_GYRO_P_GAIN_PERMILLE = 6u,
    WIRELESS_PARAM_GYRO_I_GAIN_PERMILLE = 7u,
    WIRELESS_PARAM_RING_PROGRESS_MDEG = 8u,
    WIRELESS_PARAM_RING_EXIT_MS = 9u,
    WIRELESS_PARAM_RING_SEARCH_MS = 10u,
    WIRELESS_PARAM_SERVO_LEFT_US = 11u,
    WIRELESS_PARAM_SERVO_RIGHT_US = 12u,
    WIRELESS_PARAM_ENCODER_LEFT_CPR = 13u,
    WIRELESS_PARAM_ENCODER_RIGHT_CPR = 14u,
    WIRELESS_PARAM_ENCODER_LEFT_SIGN = 15u,
    WIRELESS_PARAM_ENCODER_RIGHT_SIGN = 16u,
    /* Appended to preserve the established IDs 1..16. */
    WIRELESS_PARAM_VISION_D_GAIN_PERMILLE = 17u
} wireless_param_id_t;

typedef struct {
    uint16_t wheel_scale_permille;
    uint32_t gyro_rate_mdps;
    uint16_t vision_gain_permille;
    uint16_t vision_d_gain_permille;
    uint16_t speed_p_gain_permille;
    uint16_t speed_i_gain_permille;
    uint16_t gyro_p_gain_permille;
    uint16_t gyro_i_gain_permille;
    uint32_t ring_progress_mdeg;
    uint16_t ring_exit_ms;
    uint16_t ring_search_ms;
    uint16_t servo_left_us;
    uint16_t servo_right_us;
    uint32_t encoder_left_cpr;
    uint32_t encoder_right_cpr;
    int8_t encoder_left_sign;
    int8_t encoder_right_sign;
} wireless_settings_t;

typedef enum {
    WIRELESS_SETTINGS_OK = 0u,
    WIRELESS_SETTINGS_BAD_GROUP = 1u,
    WIRELESS_SETTINGS_BAD_PARAM = 2u,
    WIRELESS_SETTINGS_OUT_OF_RANGE = 3u,
    WIRELESS_SETTINGS_NOT_PARKED = 4u,
    WIRELESS_SETTINGS_BAD_RECORD = 5u
} wireless_settings_status_t;

/* One parameter entry in a SET_PARAMS request. */
typedef struct {
    uint8_t id;
    int32_t value;
} wireless_param_entry_t;

void wireless_settings_defaults(wireless_settings_t *settings);
uint8_t wireless_settings_is_valid(const wireless_settings_t *settings);

/* Validate and atomically apply a complete group.  The destination is left
   unchanged when any entry is invalid or outside its safety range. */
wireless_settings_status_t wireless_settings_apply_group(
    wireless_settings_t *destination,
    uint8_t group,
    const wireless_param_entry_t *entries,
    uint8_t count,
    uint8_t moving);

/* A moving request may contain only the bounded live controls. */
uint8_t wireless_settings_param_is_moving_allowed(uint8_t id);

/* Portable, fixed-size dual-slot record helpers.  The target flash adapter is
   intentionally separate; these functions can be tested without hardware. */
uint32_t wireless_settings_crc32(const uint8_t *data, size_t length);
size_t wireless_settings_encode_slot(const wireless_settings_t *settings,
                                     uint32_t generation,
                                     uint8_t *out,
                                     size_t capacity);
uint8_t wireless_settings_decode_slot(const uint8_t *record,
                                       size_t length,
                                       wireless_settings_t *settings,
                                       uint32_t *generation);
uint8_t wireless_settings_generation_is_newer(uint32_t candidate,
                                              uint32_t current);
uint8_t wireless_settings_select_slot(const uint8_t *slot_a,
                                      const uint8_t *slot_b,
                                      size_t length,
                                      wireless_settings_t *settings,
                                      uint32_t *generation,
                                      uint8_t *selected_slot);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_WIRELESS_SETTINGS_H */
