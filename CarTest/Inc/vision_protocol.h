#ifndef SMART_CAR_VISION_PROTOCOL_H
#define SMART_CAR_VISION_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VP_FRAME_SIZE 16u
#define VP_SYNC_BYTE_0 0xA5u
#define VP_SYNC_BYTE_1 0x5Au
#define VP_VERSION 2u
#define VP_VERSION_V1 1u
#define VP_PAYLOAD_LENGTH 8u
#define VP_FLAG_VALID 0x01u
#define VP_FLAG_SLOPE_HINT 0x02u
#define VP_FLAG_ALLOWED_MASK (VP_FLAG_VALID | VP_FLAG_SLOPE_HINT)
#define VP_MAX_CONFIDENCE 100u
#define VP_MIN_LATERAL_ERROR (-1000)
#define VP_MAX_LATERAL_ERROR 1000
#define VP_CURVE_PHASE_STRAIGHT 0u
#define VP_CURVE_PHASE_CANDIDATE 1u
#define VP_CURVE_PHASE_TURN 2u
#define VP_CURVE_PHASE_EXIT 3u
#define VP_CURVE_PHASE_MAX VP_CURVE_PHASE_EXIT
#define VP_MAX_SLOWDOWN_REQUEST_PERMILLE 1000u

typedef enum {
    VP_STATUS_NONE = 0u,
    VP_STATUS_FRAME = 1u,
    VP_STATUS_VERSION_MISMATCH = 2u,
    VP_STATUS_BAD_ARGUMENT = 3u,
    VP_STATUS_BAD_HEADER = 4u,
    VP_STATUS_BAD_CRC = 5u,
    VP_STATUS_BAD_FIELDS = 6u
} vp_status;

typedef struct {
    uint16_t sequence;
    uint8_t flags;
    uint8_t confidence;
    int16_t lateral_error;
    uint8_t curve_phase;
    uint16_t slowdown_request_permille;
} vp_frame;

/* The parser owns this fixed-size storage; it performs no allocation. */
typedef struct {
    uint8_t buffer[VP_FRAME_SIZE];
    uint8_t length;
    uint8_t last_status;
    uint8_t last_received_version;
} vp_parser;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0. */
uint16_t vp_crc16_ccitt_false(const uint8_t* data, uint8_t length);

/* Pack/unpack the strict V2 wire format. Return 1/FRAME on success. */
uint8_t vp_pack(const vp_frame* frame, uint8_t out[VP_FRAME_SIZE]);
vp_status vp_unpack_ex(const uint8_t in[VP_FRAME_SIZE], vp_frame* frame);
uint8_t vp_unpack(const uint8_t in[VP_FRAME_SIZE], vp_frame* frame);

void vp_parser_init(vp_parser* parser);

/* Feed one byte. Return 1 only when a complete valid frame is written to out. */
uint8_t vp_parser_feed(vp_parser* parser, uint8_t byte, vp_frame* out);

/* Status describes only the most recent feed call; V1/other versions report
 * VERSION_MISMATCH immediately after the version byte is received. */
vp_status vp_parser_last_status(const vp_parser* parser);
uint8_t vp_parser_last_version(const vp_parser* parser);

#ifdef __cplusplus
}
#endif

#endif
