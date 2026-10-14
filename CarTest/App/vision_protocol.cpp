#include "vision_protocol.h"

namespace {

uint8_t valid_fields(uint8_t flags, uint8_t confidence, int16_t lateral_error) {
    if ((flags & static_cast<uint8_t>(~VP_FLAG_VALID)) != 0u) return 0;
    if ((flags & VP_FLAG_VALID) == 0u)
        return confidence == 0u && lateral_error == 0 ? 1u : 0u;
    return confidence <= VP_MAX_CONFIDENCE &&
           lateral_error >= VP_MIN_LATERAL_ERROR &&
           lateral_error <= VP_MAX_LATERAL_ERROR ? 1u : 0u;
}

int16_t decode_error(const uint8_t low, const uint8_t high) {
    const uint16_t raw = static_cast<uint16_t>(low) |
                         (static_cast<uint16_t>(high) << 8);
    const int32_t signed_value = raw < 0x8000u
                                     ? static_cast<int32_t>(raw)
                                     : static_cast<int32_t>(raw) - 0x10000;
    return static_cast<int16_t>(signed_value);
}

void write_crc(uint8_t bytes[VP_FRAME_SIZE]) {
    const uint16_t crc = vp_crc16_ccitt_false(bytes + 2, 8);
    bytes[10] = static_cast<uint8_t>(crc & 0xFFu);
    bytes[11] = static_cast<uint8_t>(crc >> 8);
}

void shift_one(vp_parser* parser) {
    if (parser->length == 0u) return;
    for (uint8_t i = 1u; i < parser->length; ++i)
        parser->buffer[i - 1u] = parser->buffer[i];
    --parser->length;
}

/*
 * Rejecting a candidate advances one byte at a time. This retains an A5 at
 * the tail and any A5 5A header embedded in the rejected candidate.
 */
void realign(vp_parser* parser) {
    while (parser->length != 0u) {
        if (parser->buffer[0] == VP_SYNC_BYTE_0 &&
            (parser->length == 1u || parser->buffer[1] == VP_SYNC_BYTE_1))
            return;
        shift_one(parser);
    }
}

}  // namespace

uint16_t vp_crc16_ccitt_false(const uint8_t* data, uint8_t length) {
    if (data == 0 && length != 0u) return 0u;

    uint16_t crc = 0xFFFFu;
    for (uint8_t i = 0u; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) != 0u
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint8_t vp_pack(const vp_frame* frame, uint8_t out[VP_FRAME_SIZE]) {
    if (frame == 0 || out == 0 ||
        !valid_fields(frame->flags, frame->confidence, frame->lateral_error))
        return 0u;

    out[0] = VP_SYNC_BYTE_0;
    out[1] = VP_SYNC_BYTE_1;
    out[2] = VP_VERSION;
    out[3] = VP_PAYLOAD_LENGTH;
    out[4] = static_cast<uint8_t>(frame->sequence & 0xFFu);
    out[5] = static_cast<uint8_t>(frame->sequence >> 8);
    out[6] = frame->flags;
    out[7] = frame->confidence;

    const uint16_t raw_error = static_cast<uint16_t>(frame->lateral_error);
    out[8] = static_cast<uint8_t>(raw_error & 0xFFu);
    out[9] = static_cast<uint8_t>(raw_error >> 8);
    write_crc(out);
    return 1u;
}

uint8_t vp_unpack(const uint8_t in[VP_FRAME_SIZE], vp_frame* frame) {
    if (in == 0 || frame == 0 || in[0] != VP_SYNC_BYTE_0 ||
        in[1] != VP_SYNC_BYTE_1 || in[2] != VP_VERSION ||
        in[3] != VP_PAYLOAD_LENGTH)
        return 0u;

    const uint16_t expected_crc = vp_crc16_ccitt_false(in + 2, 8);
    const uint16_t received_crc = static_cast<uint16_t>(in[10]) |
                                  (static_cast<uint16_t>(in[11]) << 8);
    if (expected_crc != received_crc) return 0u;

    const uint8_t flags = in[6];
    const uint8_t confidence = in[7];
    const int16_t lateral_error = decode_error(in[8], in[9]);
    if (!valid_fields(flags, confidence, lateral_error)) return 0u;

    frame->sequence = static_cast<uint16_t>(in[4]) |
                      (static_cast<uint16_t>(in[5]) << 8);
    frame->flags = flags;
    frame->confidence = confidence;
    frame->lateral_error = lateral_error;
    return 1u;
}

void vp_parser_init(vp_parser* parser) {
    if (parser != 0) parser->length = 0u;
}

uint8_t vp_parser_feed(vp_parser* parser, uint8_t byte, vp_frame* out) {
    if (parser == 0) return 0u;

    if (parser->length == 0u) {
        if (byte == VP_SYNC_BYTE_0) {
            parser->buffer[0] = byte;
            parser->length = 1u;
        }
        return 0u;
    }

    if (parser->length == 1u) {
        if (byte == VP_SYNC_BYTE_1) {
            parser->buffer[1] = byte;
            parser->length = 2u;
        } else if (byte == VP_SYNC_BYTE_0) {
            parser->buffer[0] = byte;
        } else {
            parser->length = 0u;
        }
        return 0u;
    }

    parser->buffer[parser->length] = byte;
    ++parser->length;
    if (parser->length != VP_FRAME_SIZE) return 0u;

    vp_frame candidate;
    if (vp_unpack(parser->buffer, &candidate)) {
        parser->length = 0u;
        if (out == 0) return 0u;
        *out = candidate;
        return 1u;
    }

    /* Drop the rejected candidate's first byte before realigning. */
    shift_one(parser);
    realign(parser);
    return 0u;
}
