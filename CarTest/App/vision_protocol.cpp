#include "vision_protocol.h"

namespace {

uint8_t valid_fields(const vp_frame* frame) {
    if (frame == 0 ||
        (frame->flags & static_cast<uint8_t>(~VP_FLAG_ALLOWED_MASK)) != 0u)
        return 0u;

    if ((frame->flags & VP_FLAG_VALID) == 0u) {
        return frame->flags == 0u && frame->confidence == 0u &&
                       frame->lateral_error == 0 &&
                       frame->curve_phase == VP_CURVE_PHASE_STRAIGHT &&
                       frame->slowdown_request_permille == 0u
                   ? 1u : 0u;
    }

    return frame->confidence <= VP_MAX_CONFIDENCE &&
                   frame->lateral_error >= VP_MIN_LATERAL_ERROR &&
                   frame->lateral_error <= VP_MAX_LATERAL_ERROR &&
                   frame->curve_phase <= VP_CURVE_PHASE_MAX &&
                   frame->slowdown_request_permille <=
                       VP_MAX_SLOWDOWN_REQUEST_PERMILLE
               ? 1u : 0u;
}

int16_t decode_i16(const uint8_t low, const uint8_t high) {
    const uint16_t raw = static_cast<uint16_t>(low) |
                         (static_cast<uint16_t>(high) << 8);
    const int32_t signed_value = raw < 0x8000u
                                     ? static_cast<int32_t>(raw)
                                     : static_cast<int32_t>(raw) - 0x10000;
    return static_cast<int16_t>(signed_value);
}

void write_crc(uint8_t bytes[VP_FRAME_SIZE]) {
    const uint16_t crc = vp_crc16_ccitt_false(bytes + 2, 12u);
    bytes[14] = static_cast<uint8_t>(crc & 0xFFu);
    bytes[15] = static_cast<uint8_t>(crc >> 8);
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

void reject_candidate(vp_parser* parser, vp_status status,
                      uint8_t received_version) {
    parser->last_status = static_cast<uint8_t>(status);
    parser->last_received_version = received_version;
    shift_one(parser);
    realign(parser);
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
    if (frame == 0 || out == 0 || !valid_fields(frame)) return 0u;

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
    out[10] = frame->curve_phase;
    out[11] = 0u;
    out[12] = static_cast<uint8_t>(
        frame->slowdown_request_permille & 0xFFu);
    out[13] = static_cast<uint8_t>(
        frame->slowdown_request_permille >> 8);
    write_crc(out);
    return 1u;
}

vp_status vp_unpack_ex(const uint8_t in[VP_FRAME_SIZE], vp_frame* frame) {
    if (in == 0 || frame == 0) return VP_STATUS_BAD_ARGUMENT;
    if (in[0] != VP_SYNC_BYTE_0 || in[1] != VP_SYNC_BYTE_1)
        return VP_STATUS_BAD_HEADER;
    if (in[2] != VP_VERSION) return VP_STATUS_VERSION_MISMATCH;
    if (in[3] != VP_PAYLOAD_LENGTH || in[11] != 0u)
        return VP_STATUS_BAD_HEADER;

    const uint16_t expected_crc = vp_crc16_ccitt_false(in + 2, 12u);
    const uint16_t received_crc = static_cast<uint16_t>(in[14]) |
                                  (static_cast<uint16_t>(in[15]) << 8);
    if (expected_crc != received_crc) return VP_STATUS_BAD_CRC;

    vp_frame candidate{};
    candidate.sequence = static_cast<uint16_t>(in[4]) |
                         (static_cast<uint16_t>(in[5]) << 8);
    candidate.flags = in[6];
    candidate.confidence = in[7];
    candidate.lateral_error = decode_i16(in[8], in[9]);
    candidate.curve_phase = in[10];
    candidate.slowdown_request_permille = static_cast<uint16_t>(in[12]) |
        (static_cast<uint16_t>(in[13]) << 8);
    if (!valid_fields(&candidate)) return VP_STATUS_BAD_FIELDS;

    *frame = candidate;
    return VP_STATUS_FRAME;
}

uint8_t vp_unpack(const uint8_t in[VP_FRAME_SIZE], vp_frame* frame) {
    return vp_unpack_ex(in, frame) == VP_STATUS_FRAME ? 1u : 0u;
}

void vp_parser_init(vp_parser* parser) {
    if (parser == 0) return;
    parser->length = 0u;
    parser->last_status = VP_STATUS_NONE;
    parser->last_received_version = 0u;
}

uint8_t vp_parser_feed(vp_parser* parser, uint8_t byte, vp_frame* out) {
    if (parser == 0) return 0u;
    parser->last_status = VP_STATUS_NONE;
    parser->last_received_version = 0u;

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

    /* Report an old/unknown frame as soon as its version byte is available. */
    if (parser->length == 3u && parser->buffer[2] != VP_VERSION) {
        reject_candidate(parser, VP_STATUS_VERSION_MISMATCH,
                         parser->buffer[2]);
        return 0u;
    }
    if (parser->length == 4u && parser->buffer[3] != VP_PAYLOAD_LENGTH) {
        reject_candidate(parser, VP_STATUS_BAD_HEADER, parser->buffer[2]);
        return 0u;
    }
    if (parser->length != VP_FRAME_SIZE) return 0u;

    vp_frame candidate{};
    const vp_status status = vp_unpack_ex(parser->buffer, &candidate);
    if (status == VP_STATUS_FRAME) {
        parser->length = 0u;
        parser->last_received_version = VP_VERSION;
        if (out == 0) {
            parser->last_status = VP_STATUS_BAD_ARGUMENT;
            return 0u;
        }
        parser->last_status = VP_STATUS_FRAME;
        *out = candidate;
        return 1u;
    }

    reject_candidate(parser, status, parser->buffer[2]);
    return 0u;
}

vp_status vp_parser_last_status(const vp_parser* parser) {
    return parser == 0 ? VP_STATUS_BAD_ARGUMENT
                       : static_cast<vp_status>(parser->last_status);
}

uint8_t vp_parser_last_version(const vp_parser* parser) {
    return parser == 0 ? 0u : parser->last_received_version;
}
