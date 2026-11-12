#include "wireless_runtime.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static car_control_t control;
static car_config_t config;
static car_wireless_snapshot_t snapshot;
static uint32_t camera_sequence;
static uint32_t camera_epoch;

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8u);
}

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8u);
    p[2] = (uint8_t)(value >> 16u);
    p[3] = (uint8_t)(value >> 24u);
}

static void service(uint32_t now_ms)
{
    snapshot.now_ms = now_ms;
    car_wireless_service(&snapshot, &control, &config);
}

static void feed_bytes(const uint8_t *bytes, size_t length, uint32_t now_ms)
{
    size_t index;
    for (index = 0u; index < length; ++index)
        car_wireless_uart_rx_isr(bytes[index], now_ms);
    service(now_ms);
}

static void feed_text(const char *text, uint32_t now_ms)
{
    feed_bytes((const uint8_t *)text, strlen(text), now_ms);
}

static int next_radio_frame(car_wireless_frame_t *out)
{
    car_wireless_parser_t parser;
    uint8_t byte;
    unsigned raw = 0u;
    int found = 0;
    car_wireless_parser_init(&parser);
    while (car_wireless_radio_tx_byte(&byte)) {
        ++raw;
        if (car_wireless_parser_feed(&parser, byte, out) ==
            CAR_WIRELESS_PARSE_FRAME) found = 1;
    }
    if (!found) fprintf(stderr, "radio raw=%u parser_len=%u crc=%u sync=%u len=%u\n",
                        raw, parser.length, parser.crc_errors, parser.sync_errors,
                        parser.length_errors);
    return found;
}

static int next_camera_frame(car_wireless_frame_t *out)
{
    car_wireless_parser_t parser;
    uint8_t byte;
    car_wireless_parser_init(&parser);
    while (car_wireless_camera_tx_byte(&byte, 1u)) {
        if (car_wireless_parser_feed(&parser, byte, out) ==
            CAR_WIRELESS_PARSE_FRAME) return 1;
    }
    return 0;
}

static void make_camera_ready(uint32_t now_ms)
{
    car_wireless_frame_t hello = {0};
    car_wireless_frame_t reply = {0};
    assert(next_camera_frame(&hello));
    assert(hello.type == CAR_WIRELESS_HELLO);
    assert(hello.payload_len == 8u);
    assert(hello.payload[5] < WIRELESS_SETTINGS_PROFILE_COUNT);
    camera_epoch = hello.session;
    reply.type = CAR_WIRELESS_HELLO_REPLY;
    reply.session = camera_epoch;
    reply.sequence = ++camera_sequence;
    reply.payload_len = 8u;
    put32(reply.payload, camera_epoch);
    put32(reply.payload + 4u, 0x1fu);
    car_wireless_camera_frame(&reply, now_ms);
}

static void send_camera_state(uint32_t now_ms)
{
    car_wireless_frame_t frame = {0};
    frame.type = CAR_WIRELESS_STATE;
    frame.session = camera_epoch;
    frame.sequence = ++camera_sequence;
    frame.payload_len = 36u;
    put32(frame.payload + 0u, 17u);
    frame.payload[4] = 1u;
    frame.payload[5] = 92u;
    frame.payload[6] = 4u;
    frame.payload[7] = 2u;
    frame.payload[8] = 1u;
    put16(frame.payload + 9u, (uint16_t)(int16_t)-12);
    put16(frame.payload + 11u, (uint16_t)(int16_t)34);
    put16(frame.payload + 13u, 7u);
    put16(frame.payload + 15u, 653u);
    frame.payload[17] = 3u;
    put16(frame.payload + 18u, 800u);
    frame.payload[20] = 5u;
    frame.payload[21] = 1u;
    put32(frame.payload + 22u, 0x1234u);
    put16(frame.payload + 26u, 9u);
    car_wireless_camera_frame(&frame, now_ms);
}

static void send_camera_config(uint32_t now_ms)
{
    car_wireless_frame_t frame = {0};
    frame.type = CAR_WIRELESS_CONFIG;
    frame.session = camera_epoch;
    frame.sequence = ++camera_sequence;
    frame.payload_len = 16u;
    put32(frame.payload + 0u, 1u);
    put32(frame.payload + 4u, 3u);
    put16(frame.payload + 8u, 1120u);
    put16(frame.payload + 10u, 880u);
    car_wireless_camera_frame(&frame, now_ms);
}

static void send_camera_ack(uint32_t now_ms)
{
    car_wireless_frame_t frame = {0};
    frame.type = CAR_WIRELESS_ACK;
    frame.session = camera_epoch;
    frame.sequence = ++camera_sequence;
    frame.payload_len = 16u;
    frame.payload[0] = WIRELESS_ACK_OK;
    frame.payload[1] = WIRELESS_OP_SET_PARAMS;
    frame.payload[2] = WIRELESS_TARGET_CAMERA;
    put32(frame.payload + 8u, 0x55u);
    put32(frame.payload + 12u, 3u);
    car_wireless_camera_frame(&frame, now_ms);
}

static void select_scope_page(uint8_t page, uint32_t now_ms)
{
    uint8_t command[8];
    float value = (float)page;
    assert(car_seekfree_debug_param_frame(command, sizeof(command), 7u, value));
    feed_bytes(command, sizeof(command), now_ms);
}

static int next_scope(float values[16])
{
    uint8_t bytes[72];
    size_t count = 0u;
    uint8_t byte;
    while (car_wireless_radio_tx_byte(&byte)) {
        if (count == 0u && byte != CAR_SEEKFREE_SEND_HEAD) continue;
        bytes[count++] = byte;
        if (count == sizeof(bytes)) {
            if (bytes[2] == CAR_SEEKFREE_CMD_OSCILLOSCOPE && bytes[3] == 16u) {
                memcpy(values, bytes + 8u, sizeof(float) * 16u);
                return 1;
            }
            count = 0u;
        }
    }
    return 0;
}

static void test_runtime_link(void)
{
    car_wireless_frame_t frame;
    float scope[16];

    (void)memset(&control, 0, sizeof(control));
    (void)memset(&config, 0, sizeof(config));
    (void)memset(&snapshot, 0, sizeof(snapshot));
    snapshot.button_released = 1u;
    snapshot.startup_released = 1u;
    config.motor_hardware_accepted = 1u;
    car_wireless_init(100u, 0x12345678u);
    service(100u);
    make_camera_ready(101u);
    send_camera_state(120u);
    send_camera_config(121u);
    send_camera_ack(122u);
    select_scope_page(4u, 160u);
    service(170u);
    assert(next_scope(scope));
    assert(scope[0] == 65.3f);
    assert(scope[1] == 1.0f);
    assert(scope[2] == 2.0f);
    assert(scope[3] == 9.0f);
    assert(scope[4] == 0.0f);
    assert(scope[5] == 0.0f);
    assert(scope[7] == 0.0f && scope[8] == 0.0f);
    assert(scope[13] == 3.0f);
    assert(scope[14] == 5.0f);
    assert(scope[15] == 7.0f);

    feed_text("ARM\n", 200u);
    service(201u);
    feed_text("READ\n", 202u);
    assert(next_radio_frame(&frame));
    assert(frame.type == CAR_WIRELESS_ACK || frame.type == CAR_WIRELESS_CONFIG);

    feed_text("DEFAULT\n", 203u);
    assert(next_radio_frame(&frame));
    assert(frame.type == CAR_WIRELESS_ACK || frame.type == CAR_WIRELESS_CONFIG);

    /* READ/DEFAULT already consumed command numbers. START must use the
       next one rather than replaying sequence 1. */
    service(250u);
    assert(next_scope(scope));
    assert(scope[0] == 2.0f && scope[1] == (float)WIRELESS_ACK_OK);
    {
        char start[32];
        snprintf(start, sizeof(start), "START %07u\n", (unsigned)scope[2]);
        feed_text(start, 251u);
        assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_START);
        car_wireless_effect_result(CAR_WIRELESS_EFFECT_START, 1u);
        service(310u);
        assert(next_scope(scope));
        assert(scope[0] == 0.0f);
        feed_text(start, 312u);
        assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_NONE);
    }
    (void)next_radio_frame(&frame);

    feed_text("IMUCAL\n", 260u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_IMU_CAL);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_IMU_CAL, 1u);
    (void)next_radio_frame(&frame);

    control.follow_session_active = 1u;
    snapshot.outputs_active = 1u;
    frame = (car_wireless_frame_t){0};
    frame.type = CAR_WIRELESS_COMMAND;
    frame.payload_len = 4u;
    frame.payload[0] = WIRELESS_OP_SAVE;
    frame.session = car_wireless_session;
    frame.sequence = 5u;
    frame.timestamp_ms = 270u;
    {
        uint8_t bytes[CAR_WIRELESS_MAX_FRAME];
        size_t length = 0u;
        assert(car_wireless_encode(&frame, bytes, sizeof(bytes), &length));
        feed_bytes(bytes, length, 270u);
    }
    assert(next_radio_frame(&frame));
    assert(frame.type == CAR_WIRELESS_ACK);
    assert(frame.payload_len == 16u);
    assert(frame.payload[0] == WIRELESS_ACK_NOT_PARKED);
}

static void test_assistant_camera_read_and_explicit_save(void)
{
    car_wireless_frame_t request, ack = {0};
    int found;
    memset(&control, 0, sizeof(control));
    memset(&config, 0, sizeof(config));
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.button_released = snapshot.startup_released = 1u;
    config.motor_hardware_accepted = 1u;
    car_wireless_init(100u, 0x76543210u);
    service(100u);
    make_camera_ready(101u);
    feed_text("ARM\n", 110u);
    feed_text("READCAM\n", 120u);
    assert(next_camera_frame(&request));
    assert(request.type == CAR_WIRELESS_COMMAND);
    assert(request.payload[0] == WIRELESS_OP_GET_PARAMS);
    assert(request.payload[1] == WIRELESS_TARGET_CAMERA);
    (void)next_radio_frame(&ack);

    /* A save cannot claim camera success before its actual PD ACK. */
    feed_text("SAVECAM\n", 130u);
    assert(!next_camera_frame(&request));
    send_camera_state(2100u);
    service(2110u);
    found = 0;
    while (next_camera_frame(&request)) {
        if (request.type == CAR_WIRELESS_COMMAND &&
            request.payload[0] == WIRELESS_OP_SET_PARAMS) {
            ack = (car_wireless_frame_t){0};
            ack.type = CAR_WIRELESS_ACK;
            ack.session = camera_epoch;
            ack.sequence = ++camera_sequence;
            ack.payload_len = 16u;
            ack.payload[0] = WIRELESS_ACK_OK;
            ack.payload[1] = WIRELESS_OP_SET_PARAMS;
            ack.payload[2] = WIRELESS_TARGET_CAMERA;
            put32(ack.payload + 4u, request.sequence);
            car_wireless_camera_frame(&ack, 2111u);
            found = 1;
        }
    }
    assert(found);
    feed_text("SAVECAM\n", 2120u);
    assert(next_camera_frame(&request));
    assert(request.type == CAR_WIRELESS_COMMAND);
    assert(request.payload[0] == WIRELESS_OP_SAVE);
    assert(request.payload[1] == WIRELESS_TARGET_CAMERA);
}

static void test_assistant_visual_entry_calibration_forward(void)
{
    car_wireless_frame_t request, ack;
    float scope[16];

    memset(&control, 0, sizeof(control));
    memset(&config, 0, sizeof(config));
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.button_released = snapshot.startup_released = 1u;
    config.motor_hardware_accepted = 1u;
    car_wireless_init(100u, 0x13572468u);
    service(100u);
    make_camera_ready(101u);
    feed_text("ARM\n", 110u);

    /* The marker protocol is deliberately parked/session gated and is
       forwarded unchanged to the camera. */
    feed_text("VCALRESET\n", 120u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_ENTRY_RESET);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_ENTRY_RESET, 1u);
    assert(next_camera_frame(&request));
    assert(request.type == CAR_WIRELESS_COMMAND);
    assert(request.payload_len == 7u);
    assert(request.payload[0] == WIRELESS_OP_VISUAL_ENTRY_CAL);
    assert(request.payload[1] == WIRELESS_TARGET_CAMERA);
    assert(request.payload[4] == 0u && request.payload[5] == 0u &&
           request.payload[6] == 0u);

    memset(&ack, 0, sizeof(ack));
    ack.type = CAR_WIRELESS_ACK;
    ack.session = camera_epoch;
    ack.sequence = ++camera_sequence;
    ack.payload_len = 16u;
    ack.payload[0] = WIRELESS_ACK_OK;
    ack.payload[1] = WIRELESS_OP_VISUAL_ENTRY_CAL;
    ack.payload[2] = WIRELESS_TARGET_CAMERA;
    put32(ack.payload + 4u, request.sequence);
    car_wireless_camera_frame(&ack, 121u);
    service(171u);
    assert(next_scope(scope));
    assert(scope[0] == 3.0f && scope[1] == (float)WIRELESS_ACK_OK);

    feed_text("VCALMARK12\n", 180u);
    assert(next_camera_frame(&request));
    assert(request.payload_len == 7u);
    assert(request.payload[0] == WIRELESS_OP_VISUAL_ENTRY_CAL);
    assert(request.payload[4] == 1u && request.payload[5] == 1u &&
           request.payload[6] == 2u);
    ack.sequence = ++camera_sequence;
    ack.payload[0] = WIRELESS_ACK_NOT_READY;
    put32(ack.payload + 8u, 1u); /* visual saved, encoder failed */
    put32(ack.payload + 4u, request.sequence);
    car_wireless_camera_frame(&ack, 181u);
    service(231u);
    assert(next_scope(scope));
    assert(scope[0] == 4.0f && scope[1] == (float)WIRELESS_ACK_NOT_READY);
    assert(scope[14] == 1.0f);
}

static void test_startcheck_is_read_only_and_failed_start_exposes_detail(void)
{
    float scope[16];
    char start[32];
    memset(&control,0,sizeof(control));memset(&config,0,sizeof(config));
    memset(&snapshot,0,sizeof(snapshot));
    config.motor_hardware_accepted=1u;
    snapshot.button_released=snapshot.startup_released=1u;
    snapshot.start_blockers=65536u|8u;
    car_wireless_init(100u,0x12345678u);service(100u);
    feed_text("STARTCHECK\n",110u);service(160u);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assert(next_scope(scope));
    assert(scope[0]==3.0f && scope[1]==0.0f && scope[14]==65544.0f);
    feed_text("ARM\n",170u);service(220u);assert(next_scope(scope));
    snprintf(start,sizeof(start),"START %07u\n",(unsigned)scope[2]);
    feed_text(start,230u);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    car_wireless_effect_result_detail(CAR_WIRELESS_EFFECT_START,0u,65544u);
    service(280u);assert(next_scope(scope));
    assert(scope[0]==4.0f && scope[1]==15.0f && scope[14]==65544.0f);
    assert(!control.road_drive_allowed);
}

int main(void)
{
    test_startcheck_is_read_only_and_failed_start_exposes_detail();
    test_assistant_camera_read_and_explicit_save();
    test_assistant_visual_entry_calibration_forward();
    test_runtime_link();
    return 0;
}
