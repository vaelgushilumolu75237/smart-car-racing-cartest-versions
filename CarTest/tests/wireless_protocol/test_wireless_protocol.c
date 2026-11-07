#include "wireless_protocol.h"

#include <assert.h>
#include <string.h>

static void test_frame(void)
{
    car_wireless_frame_t in = {CAR_WIRELESS_COMMAND, 3u, 0x01020304u,
                               0xAABBCCDDu, 123456u, {1u, 2u, 3u}};
    car_wireless_frame_t out;
    car_wireless_parser_t parser;
    uint8_t bytes[CAR_WIRELESS_MAX_FRAME];
    size_t length = 0u;
    size_t i;
    assert(car_wireless_encode(&in, bytes, sizeof(bytes), &length));
    assert(length == 27u);
    {
        static const uint8_t expected[] = {
            0xD3u, 0x91u, 0x01u, 0x03u, 0x03u, 0x00u, 0x00u, 0x00u,
            0x04u, 0x03u, 0x02u, 0x01u, 0xDDu, 0xCCu, 0xBBu, 0xAAu,
            0x40u, 0xE2u, 0x01u, 0x00u, 0x01u, 0x02u, 0x03u,
            0x92u, 0x7Fu, 0x74u, 0xBBu};
        assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    }
    assert(car_wireless_decode(bytes, length, &out));
    assert(out.type == in.type && out.payload_len == in.payload_len);
    assert(out.sequence == in.sequence && out.session == in.session);
    assert(out.timestamp_ms == in.timestamp_ms);
    assert(memcmp(out.payload, in.payload, 3u) == 0);
    car_wireless_parser_init(&parser);
    assert(car_wireless_parser_feed(&parser, 0x00u, &out) ==
           CAR_WIRELESS_PARSE_NONE);
    for (i = 0u; i < length; ++i) {
        car_wireless_parse_result_t result =
            car_wireless_parser_feed(&parser, bytes[i], &out);
        if (i + 1u == length) assert(result == CAR_WIRELESS_PARSE_FRAME);
    }
    assert(parser.frames_ok == 1u);
    bytes[length - 1u] ^= 1u;
    car_wireless_parser_init(&parser);
    for (i = 0u; i < length; ++i)
        (void)car_wireless_parser_feed(&parser, bytes[i], &out);
    assert(parser.crc_errors == 1u);
    assert(car_wireless_sequence_forward(0u, 0xFFFFFFFFu));
    assert(!car_wireless_sequence_forward(0xFFFFFFFFu, 0u));
}

static void test_seekfree(void)
{
    uint8_t bytes[12u + 64u];
    float values[2] = {1.0f, -2.0f};
    size_t length = 0u;
    assert(car_seekfree_oscilloscope_config(bytes, sizeof(bytes), 2u, 0u) == 8u);
    assert(bytes[0] == 0xAAu && bytes[1] == 0xBCu && bytes[2] == 0x10u);
    assert(car_seekfree_oscilloscope_frame(bytes, sizeof(bytes), values, 2u,
                                           &length));
    assert(length == 16u && bytes[0] == 0xAAu && bytes[2] == 0x10u);
    {
        static const uint8_t expected[] = {
            0xAAu, 0x3Bu, 0x10u, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u,
            0x00u, 0x00u, 0x80u, 0x3Fu, 0x00u, 0x00u, 0x00u, 0xC0u};
        assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    }
    assert(car_seekfree_debug_param_frame(bytes, sizeof(bytes), 1u, 1.5f));
    {
        static const uint8_t expected[] =
            {0x55u, 0x74u, 0x20u, 0x01u, 0x00u, 0x00u, 0xC0u, 0x3Fu};
        assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    }
    assert(car_seekfree_camera_frame(bytes, sizeof(bytes),
                                     CAR_SEEKFREE_CAMERA_BINARY, 8u, 1u,
                                     (const uint8_t *)"\xA5", 1u, &length));
    {
        static const uint8_t expected[] = {
            0xAAu, 0xB7u, 0x02u, 0x01u, 0x08u, 0x00u, 0x01u, 0x00u,
            0x01u, 0x00u, 0x00u, 0x00u, 0xA5u};
        assert(length == sizeof(expected));
        assert(memcmp(bytes, expected, sizeof(expected)) == 0);
    }
    assert(!car_seekfree_camera_frame(bytes, sizeof(bytes),
                                      CAR_SEEKFREE_CAMERA_GRAY, 8u, 1u,
                                      (const uint8_t *)"\xA5", 1u, &length));
}

int main(void)
{
    test_frame();
    test_seekfree();
    return 0;
}
