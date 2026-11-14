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
    assert(scope[3] == 3.0f);
    assert(scope[4] == 5.0f);
    assert(scope[5] == 1.0f);
    assert(scope[7] == 1120.0f && scope[8] == 880.0f);
    assert(scope[9] == (float)WIRELESS_ACK_OK);
    assert(scope[10] == (float)WIRELESS_OP_SET_PARAMS);

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
        feed_text(start, 252u);
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

static void test_parked_width_bootstrap_keep_never_authorizes_drive(void)
{
    memset(&control,0,sizeof(control));memset(&config,0,sizeof(config));
    memset(&snapshot,0,sizeof(snapshot));
    config.motor_hardware_accepted=1u;
    snapshot.button_released=snapshot.startup_released=1u;
    car_wireless_init(100u,0x12345678u);service(100u);
    assert(!car_wireless_parked_camera_keep_allowed(100u,1u,0u));
    make_camera_ready(101u);service(102u);
    /* No vision, no profile ACK, no ARM session: only the fresh handshake. */
    assert(!control.vision_valid && !control.latest_rx_have_frame);
    assert(car_wireless_parked_camera_keep_allowed(103u,1u,0u));
    assert(!control.road_drive_allowed && !control.follow_session_active);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assert(!car_wireless_parked_camera_keep_allowed(103u,0u,0u));
    assert(car_wireless_parked_camera_keep_allowed(103u,1u,1u));
    assert(!control.road_drive_allowed && !control.follow_session_active);
    assert(!car_wireless_parked_camera_keep_allowed(1300u,1u,0u));
    snapshot.outputs_active=1u;service(104u);
    assert(!car_wireless_parked_camera_keep_allowed(105u,1u,0u));
    snapshot.outputs_active=0u;control.fault=CAR_FAULT_UART;service(106u);
    assert(!car_wireless_parked_camera_keep_allowed(107u,1u,0u));
    control.fault=CAR_FAULT_NONE;control.follow_session_active=1u;service(108u);
    assert(!car_wireless_parked_camera_keep_allowed(109u,1u,0u));
    control.follow_session_active=0u;service(110u);car_wireless_local_stop(111u);
    assert(!car_wireless_parked_camera_keep_allowed(111u,1u,0u));
}

static void test_pe7_start_coexists_with_wireless_service(void)
{
    uint32_t now;
    memset(&control, 0, sizeof(control));
    memset(&config, 0, sizeof(config));
    memset(&snapshot, 0, sizeof(snapshot));
    car_wireless_init(0u, 0x12345678u);
    /* A held button at boot must not generate a repeated NORMAL_STOP. */
    service(1u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_NONE);
    snapshot.startup_released = 1u;
    control.manual_run_request = 1u; /* The real PE7 edge owns this request. */
    for (now = 2u; now < 102u; ++now) {
        service(now);
        assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_NONE);
        assert(control.manual_run_request == 1u);
    }
    snapshot.button_released = 1u;
    service(102u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_NONE);
    car_wireless_local_stop(103u); /* The second PE7 press is still STOP. */
    service(103u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_STOP);
    service(104u);
    assert(car_wireless_take_effect() == CAR_WIRELESS_EFFECT_NONE);
}

int main(void)
{
    test_pe7_start_coexists_with_wireless_service();
    test_parked_width_bootstrap_keep_never_authorizes_drive();
    test_assistant_camera_read_and_explicit_save();
    test_runtime_link();
    return 0;
}
