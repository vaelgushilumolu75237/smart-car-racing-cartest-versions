#include "wireless_control.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8u);
}

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8u);
    out[2] = (uint8_t)(value >> 16u); out[3] = (uint8_t)(value >> 24u);
}

static wireless_ack_status_t command(wireless_control_t *control,
                                     uint32_t seq, uint32_t now,
                                     uint8_t *payload, size_t length,
                                     wireless_control_ack_t *ack)
{
    return wireless_control_command(control, seq, 0xabcdu, now, now,
                                    payload, length, ack);
}

static void test_settings(void)
{
    wireless_settings_t settings, original;
    wireless_param_entry_t entries[2];
    uint8_t a[WIRELESS_SETTINGS_SLOT_BYTES], b[WIRELESS_SETTINGS_SLOT_BYTES];
    uint8_t selected = 99u;
    uint32_t generation = 0u;
    wireless_settings_defaults(&settings);
    assert(wireless_settings_is_valid(&settings));
    entries[0] = (wireless_param_entry_t){WIRELESS_PARAM_VISION_GAIN_PERMILLE, 1200};
    entries[1] = (wireless_param_entry_t){WIRELESS_PARAM_VISION_D_GAIN_PERMILLE, 800};
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_DRIVING,
                                          entries, 2u, 1u) == WIRELESS_SETTINGS_OK);
    assert(settings.vision_gain_permille == 1200u && settings.vision_d_gain_permille == 800u);
    entries[1].value = 2000;
    original = settings;
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_DRIVING,
                                          entries, 2u, 1u) == WIRELESS_SETTINGS_OUT_OF_RANGE);
    assert(memcmp(&settings, &original, sizeof(settings)) == 0);
    entries[0] = (wireless_param_entry_t){WIRELESS_PARAM_RING_EXIT_MS, 500};
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_PARKED_RING,
                                          entries, 1u, 1u) == WIRELESS_SETTINGS_NOT_PARKED);
    assert(wireless_settings_encode_slot(&settings, 7u, a, sizeof(a)) == sizeof(a));
    assert(wireless_settings_encode_slot(&settings, 8u, b, sizeof(b)) == sizeof(b));
    b[40] ^= 0x80u;
    assert(wireless_settings_select_slot(a, b, sizeof(a), &original, &generation,
                                         &selected) != 0u);
    assert(selected == 0u && generation == 7u && original.vision_d_gain_permille == 800u);
    a[0] ^= 1u;
    assert(!wireless_settings_decode_slot(a, sizeof(a), &original, &generation));
}

static void test_control(void)
{
    wireless_control_t control;
    wireless_control_runtime_t runtime = {1u, 1u, 1u, 0u, 0u, 1u, 0u, 0u};
    wireless_control_ack_t ack;
    uint8_t payload[32] = {0};
    wireless_control_init(&control, NULL, 3u);
    wireless_control_set_runtime(&control, &runtime);
    assert(wireless_control_open_session(&control, 0xabcd, 100u));
    payload[0] = WIRELESS_OP_START;
    assert(command(&control, 1u, 100u, payload, 4u, &ack) == WIRELESS_ACK_PENDING);
    assert(wireless_control_take_start(&control));
    assert(command(&control, 1u, 100u, payload, 4u, &ack) == WIRELESS_ACK_PENDING);
    assert((ack.flags & WIRELESS_ACK_FLAG_DUPLICATE) != 0u);
    assert(!wireless_control_take_start(&control));
    assert(command(&control, 0u, 100u, payload, 4u, &ack) == WIRELESS_ACK_OUT_OF_ORDER);
    assert(wireless_control_command(&control, 2u, 0xabcd, 100u, 1200u,
                                    payload, 4u, &ack) == WIRELESS_ACK_STALE);
    assert(command(&control, 1u, 100u, payload, 4u, &ack) == WIRELESS_ACK_PENDING);
    assert((ack.flags & WIRELESS_ACK_FLAG_DUPLICATE) != 0u &&
           ack.request_seq == 1u && ack.status == WIRELESS_ACK_PENDING);

    payload[0] = WIRELESS_OP_SET_PARAMS;
    payload[4] = WIRELESS_SETTINGS_GROUP_DRIVING; payload[5] = 2u;
    payload[6] = WIRELESS_PARAM_VISION_GAIN_PERMILLE; put_u32(payload + 7u, 1100u);
    payload[11] = WIRELESS_PARAM_VISION_D_GAIN_PERMILLE; put_u32(payload + 12u, 900u);
    assert(command(&control, 2u, 200u, payload, 16u, &ack) == WIRELESS_ACK_OK);
    assert(control.settings.vision_gain_permille == 1100u);
    runtime.follow_active = 1u;
    wireless_control_set_runtime(&control, &runtime);
    payload[4] = WIRELESS_SETTINGS_GROUP_PARKED_RING; payload[5] = 1u;
    payload[6] = WIRELESS_PARAM_RING_EXIT_MS; put_u32(payload + 7u, 400u);
    assert(command(&control, 3u, 220u, payload, 11u, &ack) == WIRELESS_ACK_NOT_PARKED);
    runtime.follow_active = 0u;
    wireless_control_set_runtime(&control, &runtime);
    payload[0] = WIRELESS_OP_TEST_MOTOR; payload[4] = 3u;
    put_u16(payload + 5u, 400u); put_u16(payload + 7u, 5000u);
    assert(command(&control, 4u, 300u, payload, 9u, &ack) == WIRELESS_ACK_OK);
    assert(wireless_control_test(&control).active != 0u);
    assert(!wireless_control_open_session(&control, 0x1111u, 301u));
    payload[0] = WIRELESS_OP_SAVE;
    assert(command(&control, 5u, 302u, payload, 4u, &ack) == WIRELESS_ACK_BUSY);
    payload[0] = WIRELESS_OP_INFO;
    assert(command(&control, 6u, 303u, payload, 4u, &ack) == WIRELESS_ACK_OK);
    wireless_control_tick(&control, 5300u);
    assert(wireless_control_test(&control).active == 0u);
    runtime.capture_busy = 1u;
    wireless_control_set_runtime(&control, &runtime);
    payload[0] = WIRELESS_OP_SET_PARAMS;
    payload[4] = WIRELESS_SETTINGS_GROUP_DRIVING; payload[5] = 2u;
    payload[6] = WIRELESS_PARAM_VISION_GAIN_PERMILLE; put_u32(payload + 7u, 1100u);
    payload[11] = WIRELESS_PARAM_VISION_D_GAIN_PERMILLE; put_u32(payload + 12u, 900u);
    assert(command(&control, 7u, 5301u, payload, 16u, &ack) == WIRELESS_ACK_BUSY);
    runtime.capture_busy = 0u;
    wireless_control_set_runtime(&control, &runtime);
    payload[0] = WIRELESS_OP_TEST_SERVO; put_u16(payload + 4u, 1100u);
    put_u16(payload + 6u, 1000u);
    assert(command(&control, 8u, 5400u, payload, 8u, &ack) == WIRELESS_ACK_OK);
    payload[0] = WIRELESS_OP_STOP;
    assert(command(&control, 9u, 5500u, payload, 4u, &ack) == WIRELESS_ACK_OK);
    assert(wireless_control_take_stop(&control));

    /* Camera commands use a separate permission surface.  Only the camera's
       vision gain group may be forwarded while moving; parking operations and
       calibration remain stopped-only, and actuator/unknown opcodes never
       reach the K230. */
    payload[1] = WIRELESS_TARGET_CAMERA;
    payload[0] = WIRELESS_OP_SET_PARAMS;
    payload[4] = WIRELESS_SETTINGS_GROUP_DRIVING; payload[5] = 2u;
    payload[6] = WIRELESS_PARAM_VISION_GAIN_PERMILLE; put_u32(payload + 7u, 1200u);
    payload[11] = WIRELESS_PARAM_VISION_D_GAIN_PERMILLE; put_u32(payload + 12u, 800u);
    assert(command(&control, 10u, 5600u, payload, 16u, &ack) == WIRELESS_ACK_FORWARD);
    runtime.follow_active = 1u;
    wireless_control_set_runtime(&control, &runtime);
    payload[0] = WIRELESS_OP_SAVE;
    assert(command(&control, 11u, 5601u, payload, 4u, &ack) == WIRELESS_ACK_NOT_PARKED);
    payload[0] = WIRELESS_OP_DEFAULT;
    assert(command(&control, 12u, 5602u, payload, 4u, &ack) == WIRELESS_ACK_NOT_PARKED);
    payload[0] = WIRELESS_OP_CAPTURE;
    assert(command(&control, 13u, 5603u, payload, 4u, &ack) == WIRELESS_ACK_NOT_PARKED);
    runtime.follow_active = 0u;
    wireless_control_set_runtime(&control, &runtime);
    payload[0] = WIRELESS_OP_START;
    assert(command(&control, 14u, 5604u, payload, 4u, &ack) == WIRELESS_ACK_DENIED);
    payload[0] = 0xf0u;
    assert(command(&control, 15u, 5605u, payload, 4u, &ack) == WIRELESS_ACK_UNSUPPORTED);
    payload[0] = WIRELESS_OP_SET_PARAMS;
    payload[4] = WIRELESS_SETTINGS_GROUP_GYRO; payload[5] = 1u;
    payload[6] = WIRELESS_PARAM_GYRO_RATE_MDPS; put_u32(payload + 7u, 120000u);
    assert(command(&control, 16u, 5606u, payload, 11u, &ack) == WIRELESS_ACK_UNSUPPORTED);
    payload[0] = WIRELESS_OP_CAMERA_CAL;
    assert(command(&control, 17u, 5607u, payload, 4u, &ack) == WIRELESS_ACK_FORWARD);
    payload[0] = WIRELESS_OP_GET_PARAMS;
    assert(command(&control, 18u, 5608u, payload, 4u, &ack) == WIRELESS_ACK_FORWARD);
}

int main(void)
{
    test_settings();
    test_control();
    puts("wireless control: PASS");
    return 0;
}
