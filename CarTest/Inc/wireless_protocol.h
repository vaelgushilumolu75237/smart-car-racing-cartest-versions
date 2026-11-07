#ifndef CARTEST_WIRELESS_PROTOCOL_H
#define CARTEST_WIRELESS_PROTOCOL_H

/* Bounded, allocation-free link framing shared by the H743 and K230. */
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_WIRELESS_MAGIC0 = 0xD3u,
    CAR_WIRELESS_MAGIC1 = 0x91u,
    CAR_WIRELESS_VERSION = 1u,
    CAR_WIRELESS_HEADER_SIZE = 20u,
    CAR_WIRELESS_CRC_SIZE = 4u,
    CAR_WIRELESS_MAX_PAYLOAD = 160u,
    CAR_WIRELESS_MAX_FRAME = CAR_WIRELESS_HEADER_SIZE +
                             CAR_WIRELESS_MAX_PAYLOAD + CAR_WIRELESS_CRC_SIZE,
    CAR_WIRELESS_IMAGE_MAX_BYTES = 76800u
};

typedef enum {
    CAR_WIRELESS_HELLO = 1u,
    CAR_WIRELESS_HELLO_REPLY = 2u,
    CAR_WIRELESS_COMMAND = 3u,
    CAR_WIRELESS_ACK = 4u,
    CAR_WIRELESS_STATE = 16u,
    CAR_WIRELESS_DIAG = 17u,
    CAR_WIRELESS_IMAGE = 18u,
    CAR_WIRELESS_CONFIG = 19u,
    CAR_WIRELESS_EVENT = 20u
} car_wireless_type_t;

typedef enum {
    CAR_WIRELESS_PARSE_NONE = 0u,
    CAR_WIRELESS_PARSE_FRAME = 1u,
    CAR_WIRELESS_PARSE_SYNC_ERROR = 2u,
    CAR_WIRELESS_PARSE_LENGTH_ERROR = 3u,
    CAR_WIRELESS_PARSE_CRC_ERROR = 4u,
    CAR_WIRELESS_PARSE_RESERVED_ERROR = 5u
} car_wireless_parse_result_t;

typedef struct {
    uint8_t type;
    uint16_t payload_len;
    uint32_t sequence;
    uint32_t session;
    uint32_t timestamp_ms;
    uint8_t payload[CAR_WIRELESS_MAX_PAYLOAD];
} car_wireless_frame_t;

typedef struct {
    uint8_t buffer[CAR_WIRELESS_MAX_FRAME];
    uint16_t length;
    uint16_t expected;
    uint32_t frames_ok;
    uint32_t sync_errors;
    uint32_t length_errors;
    uint32_t crc_errors;
    uint32_t reserved_errors;
} car_wireless_parser_t;

uint32_t car_wireless_crc32(const void *data, size_t length);

/* Encode one complete frame. Returns zero on invalid input/capacity. */
uint8_t car_wireless_encode(const car_wireless_frame_t *frame,
                            uint8_t *output, size_t capacity,
                            size_t *written);

/* Decode one complete frame. It accepts no trailing bytes. */
uint8_t car_wireless_decode(const uint8_t *bytes, size_t length,
                            car_wireless_frame_t *frame);

void car_wireless_parser_init(car_wireless_parser_t *parser);
car_wireless_parse_result_t car_wireless_parser_feed(
    car_wireless_parser_t *parser, uint8_t byte, car_wireless_frame_t *frame);

/* Half-range sequence ordering used for command and telemetry streams. */
uint8_t car_wireless_sequence_forward(uint32_t sequence,
                                      uint32_t previous);

/* Official SeekFree assistant compatibility helpers. These bytes are sent on
   the wireless UART in compatibility mode, independently of the D3/91 link. */
enum {
    CAR_SEEKFREE_SEND_HEAD = 0xAAu,
    CAR_SEEKFREE_RECEIVE_HEAD = 0x55u,
    CAR_SEEKFREE_CMD_CAMERA = 0x02u,
    CAR_SEEKFREE_CMD_OSCILLOSCOPE = 0x10u,
    CAR_SEEKFREE_CMD_DEBUG_PARAM = 0x20u,
    CAR_SEEKFREE_CAMERA_BINARY = 0x01u,
    CAR_SEEKFREE_CAMERA_GRAY = 0x02u,
    CAR_SEEKFREE_CAMERA_RGB565 = 0x03u,
    CAR_SEEKFREE_MAX_OSCILLOSCOPE_CHANNELS = 16u,
    CAR_SEEKFREE_MAX_DEBUG_PARAMS = 8u
};

uint8_t car_seekfree_sum8(const uint8_t *bytes, size_t length);
/* 12-byte config followed by channel_count*4 raw IEEE754 float bytes. */
uint8_t car_seekfree_oscilloscope_config(uint8_t *output, size_t capacity,
                                         uint8_t channel_count,
                                         uint8_t data_type);
uint8_t car_seekfree_oscilloscope_frame(uint8_t *output, size_t capacity,
                                        const float *values,
                                        uint8_t channel_count,
                                        size_t *written);
/* 8-byte command: 55, checksum, 20, index, float32, reserved. */
uint8_t car_seekfree_debug_param_frame(uint8_t *output, size_t capacity,
                                       uint8_t channel, float value);
/* 12-byte camera config followed by packed image bytes. */
uint8_t car_seekfree_camera_frame(uint8_t *output, size_t capacity,
                                  uint8_t camera_type, uint16_t width,
                                  uint16_t height, const uint8_t *image,
                                  size_t image_len, size_t *written);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_WIRELESS_PROTOCOL_H */
