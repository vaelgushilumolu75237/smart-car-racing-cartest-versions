#include "wireless_control.h"
#include "wireless_protocol.h"
#include "wireless_settings.h"

#include <assert.h>
#include <string.h>

static wireless_ack_status_t command(wireless_control_t *control,
                                     uint32_t sequence, uint32_t now,
                                     const uint8_t *payload, size_t length,
                                     wireless_control_ack_t *ack)
{
    return wireless_control_command(control, sequence, control->session,
                                     now, now, payload, length, ack);
}

static void test_framing(void)
{
    car_wireless_frame_t input = {
        .type = CAR_WIRELESS_STATE,
        .payload_len = 3u,
        .sequence = 17u,
        .session = 0x12345678u,
        .timestamp_ms = 901u,
        .payload = {1u, 2u, 3u}
    };
    car_wireless_frame_t output = {0};
    car_wireless_parser_t parser;
    uint8_t bytes[CAR_WIRELESS_MAX_FRAME];
    size_t written = 0u;
    size_t i;

    assert(car_wireless_encode(&input, bytes, sizeof(bytes), &written));
    assert(written == CAR_WIRELESS_HEADER_SIZE + 3u + CAR_WIRELESS_CRC_SIZE);
    assert(car_wireless_decode(bytes, written, &output));
    assert(output.type == input.type && output.sequence == input.sequence);
    assert(output.session == input.session && output.timestamp_ms == input.timestamp_ms);
    assert(memcmp(output.payload, input.payload, input.payload_len) == 0);

    car_wireless_parser_init(&parser);
    for (i = 0u; i < written; ++i)
        assert(car_wireless_parser_feed(&parser, bytes[i], &output) ==
               (i + 1u == written ? CAR_WIRELESS_PARSE_FRAME : CAR_WIRELESS_PARSE_NONE));
    assert(parser.frames_ok == 1u);

    bytes[written - 1u] ^= 1u;
    car_wireless_parser_init(&parser);
    for (i = 0u; i < written; ++i)
        (void)car_wireless_parser_feed(&parser, bytes[i], &output);
    assert(parser.crc_errors == 1u);
    assert(car_wireless_sequence_forward(0u, UINT32_MAX));
    assert(!car_wireless_sequence_forward(UINT32_MAX, 0u));
}

static void test_seekfree(void)
{
    uint8_t bytes[80] = {0};
    float values[2] = {1.0f, -2.0f};
    size_t written = 0u;

    assert(car_seekfree_oscilloscope_config(bytes, sizeof(bytes), 2u, 0u) == 8u);
    assert(bytes[0] == CAR_SEEKFREE_SEND_HEAD && bytes[2] == CAR_SEEKFREE_CMD_OSCILLOSCOPE);
    assert(car_seekfree_oscilloscope_frame(bytes, sizeof(bytes), values, 2u, &written));
    assert(written == 16u && bytes[0] == CAR_SEEKFREE_SEND_HEAD);
    assert(car_seekfree_debug_param_frame(bytes, sizeof(bytes), 1u, 1.5f));
    assert(bytes[0] == CAR_SEEKFREE_RECEIVE_HEAD && bytes[2] == CAR_SEEKFREE_CMD_DEBUG_PARAM);
}

static void test_settings(void)
{
    wireless_settings_t settings;
    wireless_settings_t defaults;
    wireless_param_entry_t entry;

    wireless_settings_defaults(&settings);
    assert(wireless_settings_is_valid(&settings));
    entry.id = WIRELESS_PARAM_GYRO_RATE_MDPS;
    entry.value = 180000;
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_GYRO,
                                         &entry, 1u, 1u) == WIRELESS_SETTINGS_OK);
    entry.value = 200000;
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_GYRO,
                                         &entry, 1u, 1u) == WIRELESS_SETTINGS_OUT_OF_RANGE);
    entry.id = WIRELESS_PARAM_RING_PROGRESS_MDEG;
    entry.value = 360000;
    assert(wireless_settings_apply_group(&settings, WIRELESS_SETTINGS_GROUP_PARKED_RING,
                                         &entry, 1u, 1u) == WIRELESS_SETTINGS_NOT_PARKED);

    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_A_NO_RING);
    assert(defaults.gyro_rate_mdps == 120000u);
    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_B_RING_BASE);
    assert(defaults.gyro_rate_mdps == 64935u);
    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_C_RING);
    assert(defaults.gyro_rate_mdps == 85714u);
    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_C_RING_SPEED120);
    assert(defaults.gyro_rate_mdps == 102857u);
    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_C_RING_SPEED130);
    assert(defaults.gyro_rate_mdps == 111429u);
    wireless_settings_defaults_for_profile(&defaults, WIRELESS_PROFILE_C_RING_SPEED140);
    assert(defaults.gyro_rate_mdps == 120000u);
}

static void test_control_gates(void)
{
    wireless_control_t control;
    wireless_control_runtime_t runtime = {
        .parked = 1u,
        .local_button_released = 1u,
        .startup_release_confirmed = 1u,
        .motor_hardware_accepted = 1u
    };
    wireless_control_ack_t ack;
    uint8_t start[] = {WIRELESS_OP_START, WIRELESS_TARGET_MCU, 0u, 0u};
    uint8_t stop[] = {WIRELESS_OP_STOP, WIRELESS_TARGET_MCU, 0u, 0u};

    wireless_control_init(&control, NULL, 3u);
    wireless_control_set_runtime(&control, &runtime);
    assert(wireless_control_open_session(&control, 0x42u, 100u));
    assert(command(&control, 1u, 101u, start, sizeof(start), &ack) == WIRELESS_ACK_PENDING);
    assert(wireless_control_take_start(&control));
    assert(command(&control, 1u, 102u, start, sizeof(start), &ack) == WIRELESS_ACK_PENDING);
    assert((ack.flags & WIRELESS_ACK_FLAG_DUPLICATE) != 0u);

    runtime.hard_fault = 1u;
    wireless_control_set_runtime(&control, &runtime);
    assert(command(&control, 2u, 103u, start, sizeof(start), &ack) == WIRELESS_ACK_FAULT);
    assert(command(&control, 3u, 104u, stop, sizeof(stop), &ack) == WIRELESS_ACK_OK);
}

static void test_parameter_narrowing_cannot_bypass_limits(void)
{
    wireless_settings_t settings, original;
    wireless_param_entry_t entry;
    wireless_settings_defaults(&settings); original=settings;
    entry=(wireless_param_entry_t){WIRELESS_PARAM_WHEEL_SCALE_PERMILLE,65536+1000};
    assert(wireless_settings_apply_group(&settings,1,&entry,1,1)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    assert(!memcmp(&settings,&original,sizeof(settings)));
    entry.value=1000-65536;
    assert(wireless_settings_apply_group(&settings,1,&entry,1,1)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    entry=(wireless_param_entry_t){WIRELESS_PARAM_ENCODER_LEFT_SIGN,257};
    assert(wireless_settings_apply_group(&settings,4,&entry,1,0)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    entry=(wireless_param_entry_t){WIRELESS_PARAM_SERVO_LEFT_US,1600};
    assert(wireless_settings_apply_group(&settings,4,&entry,1,0)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    entry=(wireless_param_entry_t){WIRELESS_PARAM_RING_EXIT_MS,300};
    assert(wireless_settings_apply_group(&settings,3,&entry,1,1)==WIRELESS_SETTINGS_NOT_PARKED);
    assert(!memcmp(&settings,&original,sizeof(settings)));
}

int main(void)
{
    test_framing();
    test_seekfree();
    test_settings();
    test_control_gates();
    test_parameter_narrowing_cannot_bypass_limits();
    return 0;
}
