#include "camera_diag.h"
#include "car_vision.h"
#include "commissioning.h"
#include "reverse_control.h"
#include "vision_protocol.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

unsigned g_failures = 0u;
unsigned g_control_frames = 0u;
unsigned g_control_faults = 0u;
unsigned g_receive_frames = 0u;
unsigned g_receive_parser_errors = 0u;
uint16_t g_last_control_sequence = 0u;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        std::printf("FAIL %s\n", message);
    }
}

uint16_t read_u16(const uint8_t *bytes)
{
    return static_cast<uint16_t>(bytes[0]) |
           static_cast<uint16_t>(bytes[1] << 8u);
}

uint32_t read_u32(const uint8_t *bytes)
{
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8u) |
           (static_cast<uint32_t>(bytes[2]) << 16u) |
           (static_cast<uint32_t>(bytes[3]) << 24u);
}

uint32_t crc32_ieee(const uint8_t *bytes, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0u; index < length; ++index) {
        uint8_t value = bytes[index];
        if (index >= CAR_CAMERA_DIAG_OFFSET_CRC32 &&
            index < CAR_CAMERA_DIAG_OFFSET_CRC32 + 4u) {
            value = 0u;
        }
        crc ^= value;
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 1u) != 0u
                      ? (crc >> 1u) ^ 0xEDB88320u
                      : crc >> 1u;
        }
    }
    return ~crc;
}

std::array<uint8_t, VP_FRAME_SIZE> make_diag(
    uint16_t sequence, const std::array<uint8_t, 8> &payload)
{
    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    bytes[0] = VP_SYNC_BYTE_0;
    bytes[1] = VP_SYNC_BYTE_1;
    bytes[2] = VP_VERSION_D1;
    bytes[3] = VP_PAYLOAD_LENGTH;
    bytes[4] = static_cast<uint8_t>(sequence & 0xFFu);
    bytes[5] = static_cast<uint8_t>(sequence >> 8u);
    std::memcpy(bytes.data() + 6u, payload.data(), payload.size());
    const uint16_t crc = vp_crc16_ccitt_false(bytes.data() + 2u, 12u);
    bytes[14] = static_cast<uint8_t>(crc & 0xFFu);
    bytes[15] = static_cast<uint8_t>(crc >> 8u);
    return bytes;
}

std::array<uint8_t, VP_FRAME_SIZE> make_v3(uint16_t sequence)
{
    vp_frame frame{};
    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    frame.sequence = sequence;
    frame.flags = VP_FLAG_VALID;
    frame.confidence = 85u;
    frame.lateral_error = -24;
    frame.curve_phase = VP_CURVE_PHASE_STRAIGHT;
    frame.profile_ack = VP_PROFILE_ACK_DEBUG_FAST_NO_RING;
    check(vp_pack_v3(&frame, bytes.data()) != 0u,
          "valid normal V3 frame packs");
    return bytes;
}

std::array<uint8_t, VP_FRAME_SIZE> make_v2(uint16_t sequence)
{
    vp_frame frame{};
    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    frame.sequence = sequence;
    frame.flags = VP_FLAG_VALID;
    frame.confidence = 85u;
    frame.lateral_error = 24;
    frame.curve_phase = VP_CURVE_PHASE_STRAIGHT;
    check(vp_pack(&frame, bytes.data()) != 0u,
          "valid normal V2 frame packs");
    return bytes;
}

void enqueue(car_vision_rx_t *rx, const uint8_t *bytes, size_t count,
             uint32_t arrival_ms)
{
    for (size_t index = 0u; index < count; ++index) {
        car_vision_rx_isr_byte(rx, bytes[index], arrival_ms);
    }
}

void poll(car_vision_rx_t *rx, car_control_t *control,
          const car_config_t *config, uint32_t now_ms)
{
    car_vision_rx_poll(rx, control, config, rx->head, now_ms, 1u);
}

void enqueue_and_poll(car_vision_rx_t *rx, car_control_t *control,
                      const car_config_t *config,
                      const std::array<uint8_t, VP_FRAME_SIZE> &bytes,
                      uint32_t arrival_ms)
{
    enqueue(rx, bytes.data(), bytes.size(), arrival_ms);
    poll(rx, control, config, arrival_ms);
}

void reset_stubs()
{
    g_control_frames = 0u;
    g_control_faults = 0u;
    g_receive_frames = 0u;
    g_receive_parser_errors = 0u;
    g_last_control_sequence = 0u;
}

void check_snapshot_valid()
{
    const uint8_t *snapshot = g_camera_diag_snapshot;
    check(read_u32(snapshot + CAR_CAMERA_DIAG_OFFSET_MAGIC) == 0x47444943u,
          "snapshot magic is CIDG");
    check(read_u16(snapshot + CAR_CAMERA_DIAG_OFFSET_VERSION) == 1u,
          "snapshot version is one");
    check(read_u16(snapshot + CAR_CAMERA_DIAG_OFFSET_SIZE) ==
              CAR_CAMERA_DIAG_SNAPSHOT_SIZE,
          "snapshot size is 352 bytes");
    check((read_u32(snapshot + CAR_CAMERA_DIAG_OFFSET_GENERATION) & 1u) == 0u,
          "snapshot generation is committed and even");
    check(read_u32(snapshot + CAR_CAMERA_DIAG_OFFSET_CRC32) ==
              crc32_ieee(snapshot, CAR_CAMERA_DIAG_SNAPSHOT_SIZE),
          "snapshot CRC32 matches bytes with its CRC field zeroed");
}

void test_initial_snapshot_and_mixed_frames()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    check((reinterpret_cast<uintptr_t>(g_camera_diag_snapshot) & 31u) == 0u,
          "snapshot global is 32-byte aligned");
    check_snapshot_valid();
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 0u,
          "snapshot initializes with accepted count zero");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 0u,
          "snapshot initializes with rejected count zero");

    const std::array<uint8_t, 8> status =
        {{1u, 0x0Fu, 1u, 2u, 3u, 0x23u, 0x34u, 0x12u}};
    const auto diag = make_diag(7u, status);
    enqueue(&rx, diag.data(), 7u, 100u);
    poll(&rx, &control, &config, 100u);
    check(rx.partial_active != 0u,
          "split D1 frame remains a parser partial");

    enqueue(&rx, diag.data() + 7u, diag.size() - 7u, 101u);
    const auto normal = make_v3(321u);
    enqueue(&rx, normal.data(), normal.size(), 101u);
    poll(&rx, &control, &config, 101u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 1u,
          "valid split D1 status is accepted once");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_LATEST_ARRIVAL_MS) == 101u,
          "D1 arrival uses the final byte timestamp");
    check(read_u16(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_LATEST_SEQUENCE) == 7u,
          "D1 has its independent sequence");
    check(g_control_frames == 1u && g_last_control_sequence == 321u,
          "concatenated V3 reaches control exactly once");
    check(g_receive_frames == 1u,
          "only normal V3 increments receive-frame diagnostics");
    check(g_control_faults == 0u && rx.protocol_error_count == 0u,
          "valid D1 frame does not create a receiver fault");
    check(read_u16(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_RING_COUNT) == 1u,
          "accepted D1 record enters the ring");
    check_snapshot_valid();
}

void test_bad_diag_and_normal_corruption()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);

    std::array<uint8_t, 8> bad_status =
        {{1u, 0u, 10u, 1u, 1u, 0u, 0u, 0u}};
    auto bad_fields = make_diag(1u, bad_status);
    enqueue_and_poll(&rx, &control, &config, bad_fields, 10u);
    auto bad_crc = make_diag(2u, {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}});
    bad_crc[14] ^= 0x80u;
    enqueue_and_poll(&rx, &control, &config, bad_crc, 11u);
    auto bad_length = make_diag(3u, {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}});
    bad_length[3] = 7u;
    enqueue(&rx, bad_length.data(), bad_length.size(), 12u);
    const auto after_bad_length = make_v3(70u);
    enqueue(&rx, after_bad_length.data(), after_bad_length.size(), 12u);
    poll(&rx, &control, &config, 12u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 3u,
          "bad D1 fields, CRC, and fixed-frame header each count a rejection");
    check(rx.protocol_error_count == 0u && rx.protocol_fault == 0u &&
              rx.protocol_crc_error_count == 0u &&
              g_receive_parser_errors == 0u && g_control_faults == 0u,
          "invalid D1 frames do not affect control faults or parser counters");

    auto normal_bad_crc = make_v3(77u);
    normal_bad_crc[14] ^= 1u;
    enqueue_and_poll(&rx, &control, &config, normal_bad_crc, 12u);
    check(rx.protocol_error_count == 1u && rx.protocol_fault != 0u &&
              rx.protocol_crc_error_count == 1u,
          "corrupted normal V3 retains protocol fault accounting");
    check(g_receive_parser_errors == 1u && g_control_faults == 1u,
          "normal V3 corruption still reports parser and control faults");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 3u,
          "normal V3 corruption does not increment D1 rejection count");
    check(g_control_frames == 1u && g_last_control_sequence == 70u,
          "fixed-size D1 rejection resynchronizes at the following V3 frame");
}

void test_diagnostic_sequence_flush_and_absence()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    const std::array<uint8_t, 8> status =
        {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}};
    enqueue_and_poll(&rx, &control, &config, make_diag(100u, status), 10u);
    enqueue_and_poll(&rx, &control, &config, make_diag(100u, status), 11u);
    enqueue_and_poll(&rx, &control, &config, make_diag(99u, status), 12u);
    enqueue_and_poll(&rx, &control, &config, make_diag(101u, status), 13u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 2u,
          "forward diagnostic sequence advances normally");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 2u,
          "duplicate and rollback diagnostic sequences are rejected");

    enqueue_and_poll(&rx, &control, &config, make_diag(3u, status), 5013u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 3u,
          "sequence baseline resets after 5000 ms without accepted D1");
    const uint32_t first_arrival = read_u32(
        g_camera_diag_snapshot + CAR_CAMERA_DIAG_OFFSET_RING);
    const uint16_t prior_ring_count = read_u16(
        g_camera_diag_snapshot + CAR_CAMERA_DIAG_OFFSET_RING_COUNT);
    car_vision_rx_clear(&rx);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_LATEST_ARRIVAL_MS) == 5013u &&
              read_u16(g_camera_diag_snapshot +
                       CAR_CAMERA_DIAG_OFFSET_RING_COUNT) == prior_ring_count &&
              read_u32(g_camera_diag_snapshot +
                       CAR_CAMERA_DIAG_OFFSET_RING) == first_arrival,
          "receiver flush preserves diagnostic ages and ring history");
    enqueue_and_poll(&rx, &control, &config, make_diag(1u, status), 5014u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 4u,
          "receiver flush resets only the diagnostic sequence gate");
    check(g_control_frames == 0u && g_receive_frames == 0u &&
              g_control_faults == 0u,
          "sequence checks and flush remain isolated from control");
}

void test_partial_timeout_is_diagnostic_only()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    const std::array<uint8_t, 8> status =
        {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}};
    const auto diag = make_diag(5u, status);
    enqueue(&rx, diag.data(), 8u, 20u);
    poll(&rx, &control, &config, 20u);
    poll(&rx, &control, &config, 40u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 1u,
          "D1 partial timeout increments diagnostic rejection count");
    check(rx.diagnostic_discard_remaining == 8u &&
              rx.partial_timeout_count == 0u && rx.stale_fault == 0u &&
              rx.protocol_error_count == 0u && g_control_faults == 0u,
          "D1 partial timeout does not create a control recovery fault");

    enqueue(&rx, diag.data() + 8u, 8u, 41u);
    const auto normal = make_v3(88u);
    enqueue(&rx, normal.data(), normal.size(), 41u);
    poll(&rx, &control, &config, 41u);
    check(g_control_frames == 1u && g_last_control_sequence == 88u,
          "resynchronization accepts the next normal V3 after D1 timeout");
    check(g_receive_frames == 1u && g_control_faults == 0u,
          "timed-out D1 tail does not affect receive or fault counters");
}

void test_d1_prefix_timeouts_and_bad_crc_quarantine()
{
    const std::array<uint8_t, 8> status =
        {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}};
    for (size_t prefix_length = 3u; prefix_length < VP_FRAME_SIZE;
         ++prefix_length) {
        car_vision_rx_t rx{};
        car_control_t control{};
        car_config_t config{};
        reset_stubs();
        car_vision_rx_init(&rx);
        auto diagnostic = make_diag(20u, status);
        diagnostic[14] = 0u;
        diagnostic[15] = 0u;
        enqueue(&rx, diagnostic.data(), prefix_length, 100u);
        poll(&rx, &control, &config, 100u);
        poll(&rx, &control, &config, 120u);
        check(read_u32(g_camera_diag_snapshot +
                       CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 1u,
              "known D1 prefix timeout counts one diagnostic rejection");
        check(rx.diagnostic_discard_remaining ==
                  VP_FRAME_SIZE - prefix_length &&
                  rx.diagnostic_discard_until_sync == 0u &&
                  rx.partial_timeout_count == 0u && rx.stale_fault == 0u &&
                  g_control_faults == 0u,
              "D1 prefix timeout records the exact continuation boundary");

        const size_t tail_length = VP_FRAME_SIZE - prefix_length;
        const size_t first_split = tail_length / 2u;
        if (first_split != 0u) {
            enqueue(&rx, diagnostic.data() + prefix_length, first_split, 121u);
            poll(&rx, &control, &config, 121u);
        }
        if (tail_length > first_split) {
            enqueue(&rx, diagnostic.data() + prefix_length + first_split,
                    tail_length - first_split, 122u);
            poll(&rx, &control, &config, 122u);
        }
        check(rx.diagnostic_discard_remaining == 0u &&
                  rx.diagnostic_discard_until_sync != 0u,
              "exact D1 tail completion advances to sync quarantine");
        const uint8_t garbage_a[] = {0x10u, 0x20u};
        enqueue(&rx, garbage_a, sizeof(garbage_a), 123u);
        poll(&rx, &control, &config, 123u);
        const uint8_t garbage_b[] = {0x30u, 0x40u};
        enqueue(&rx, garbage_b, sizeof(garbage_b), 124u);
        poll(&rx, &control, &config, 124u);
        check(rx.protocol_error_count == 0u && rx.stale_fault == 0u &&
                  g_control_faults == 0u,
              "D1 tail split across polls remains outside protocol faults");

        const auto normal = make_v3(200u +
                                    static_cast<uint16_t>(prefix_length));
        enqueue(&rx, normal.data(), normal.size(), 125u);
        poll(&rx, &control, &config, 125u);
        check(g_control_frames == 1u && g_last_control_sequence ==
                  200u + static_cast<uint16_t>(prefix_length),
              "valid V3 after each D1 prefix timeout reaches control");
        check(g_control_faults == 0u && g_receive_frames == 1u,
              "known D1 prefix continuation never changes control/FPS errors");
    }

    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    auto bad_crc = make_diag(44u, status);
    bad_crc[14] ^= 0x01u;
    enqueue(&rx, bad_crc.data(), bad_crc.size(), 200u);
    poll(&rx, &control, &config, 200u);
    check(rx.diagnostic_discard_until_sync != 0u,
          "bad D1 CRC starts persistent stream quarantine");
    const uint8_t garbage_a[] = {0x11u, 0x22u};
    enqueue(&rx, garbage_a, sizeof(garbage_a), 201u);
    poll(&rx, &control, &config, 201u);
    const uint8_t garbage_b[] = {0x33u, 0x44u};
    enqueue(&rx, garbage_b, sizeof(garbage_b), 202u);
    poll(&rx, &control, &config, 202u);
    const auto normal = make_v3(300u);
    enqueue(&rx, normal.data(), normal.size(), 203u);
    poll(&rx, &control, &config, 203u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 1u,
          "bad D1 CRC counts one reject across arbitrarily split polls");
    check(g_control_frames == 1u && g_last_control_sequence == 300u &&
              g_control_faults == 0u && rx.protocol_error_count == 0u,
          "bad D1 CRC garbage quarantine ends at the next valid V3 sync");
}

void test_stale_diagnostic_does_not_trip_queue_recovery()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    const std::array<uint8_t, 8> status =
        {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}};
    const auto diag = make_diag(1u, status);
    enqueue(&rx, diag.data(), diag.size(), 0u);
    const auto normal = make_v3(12u);
    enqueue(&rx, normal.data(), normal.size(), 100u);
    poll(&rx, &control, &config, 100u);
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT) == 1u,
          "aged queued D1 frame is rejected as diagnostic data");
    check(rx.queue_age_reject_count == 0u && rx.stale_fault == 0u &&
              g_control_faults == 0u,
          "aged D1 frame does not trigger queue recovery");
    check(g_control_frames == 1u && g_last_control_sequence == 12u,
          "fresh V3 after an aged D1 frame reaches control");
}

void test_stale_v3_is_not_hidden_by_diagnostic_quarantine()
{
    const std::array<uint8_t, 8> status =
        {{1u, 0u, 1u, 1u, 1u, 0u, 0u, 0u}};
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    auto bad_crc = make_diag(4u, status);
    bad_crc[14] ^= 1u;
    enqueue(&rx, bad_crc.data(), bad_crc.size(), 10u);
    poll(&rx, &control, &config, 10u);
    check(rx.diagnostic_discard_until_sync != 0u,
          "rejected full D1 frame enters sync quarantine");

    const auto stale_v3 = make_v3(90u);
    enqueue(&rx, stale_v3.data(), stale_v3.size(), 10u);
    poll(&rx, &control, &config, 110u);
    check(rx.stale_fault != 0u && rx.queue_age_reject_count == 1u &&
              g_control_faults == 1u,
          "clearly framed stale V3 keeps the original RX_STALE path");
    check(g_control_frames == 0u && g_receive_frames == 0u,
          "stale V3 is rejected before control or receive-FPS delivery");

    car_vision_rx_t v2_rx{};
    car_control_t v2_control{};
    car_config_t v2_config{};
    reset_stubs();
    car_vision_rx_init(&v2_rx);
    bad_crc = make_diag(6u, status);
    bad_crc[14] ^= 1u;
    enqueue(&v2_rx, bad_crc.data(), bad_crc.size(), 10u);
    poll(&v2_rx, &v2_control, &v2_config, 10u);
    const auto stale_v2 = make_v2(89u);
    enqueue(&v2_rx, stale_v2.data(), stale_v2.size(), 10u);
    poll(&v2_rx, &v2_control, &v2_config, 110u);
    check(v2_rx.stale_fault != 0u && v2_rx.queue_age_reject_count == 1u &&
              g_control_faults == 1u,
          "clearly framed stale V2 also keeps the original RX_STALE path");
    check(g_control_frames == 0u && g_receive_frames == 0u,
          "stale V2 is rejected before control or receive-FPS delivery");

    car_vision_rx_t partial_rx{};
    car_control_t partial_control{};
    car_config_t partial_config{};
    reset_stubs();
    car_vision_rx_init(&partial_rx);
    auto partial_diag = make_diag(5u, status);
    partial_diag[14] = 0u;
    partial_diag[15] = 0u;
    enqueue(&partial_rx, partial_diag.data(), 8u, 0u);
    poll(&partial_rx, &partial_control, &partial_config, 0u);
    poll(&partial_rx, &partial_control, &partial_config, 20u);
    check(partial_rx.diagnostic_discard_remaining == 8u,
          "partial D1 timeout tracks only the known frame tail");
    enqueue(&partial_rx, partial_diag.data() + 8u, 8u, 30u);
    const auto stale_after_partial = make_v3(91u);
    enqueue(&partial_rx, stale_after_partial.data(),
            stale_after_partial.size(), 30u);
    poll(&partial_rx, &partial_control, &partial_config, 130u);
    check(partial_rx.stale_fault != 0u &&
              partial_rx.queue_age_reject_count == 1u &&
              g_control_faults == 1u,
          "stale V3 after an exact partial-D1 tail still triggers RX_STALE");
    check(g_control_frames == 0u && g_receive_frames == 0u,
          "stale V3 after partial D1 quarantine is never delivered");

    car_vision_rx_t fresh_rx{};
    car_control_t fresh_control{};
    car_config_t fresh_config{};
    reset_stubs();
    car_vision_rx_init(&fresh_rx);
    enqueue(&fresh_rx, partial_diag.data(), 8u, 200u);
    poll(&fresh_rx, &fresh_control, &fresh_config, 200u);
    poll(&fresh_rx, &fresh_control, &fresh_config, 220u);
    const auto fresh_after_partial = make_v3(92u);
    enqueue(&fresh_rx, fresh_after_partial.data(),
            fresh_after_partial.size(), 221u);
    poll(&fresh_rx, &fresh_control, &fresh_config, 221u);
    check(g_control_frames == 1u && g_last_control_sequence == 92u &&
              g_control_faults == 0u && fresh_rx.stale_fault == 0u,
          "complete fresh V3 at the partial-tail boundary ends quarantine");
}

void test_snapshot_kind_fields_ring_wrap_and_export()
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    reset_stubs();
    car_vision_rx_init(&rx);
    const std::array<uint8_t, 8> status =
        {{1u, 0x0Fu, 1u, 2u, 3u, 0x23u, 0x34u, 0x12u}};
    const std::array<uint8_t, 8> counters =
        {{2u, 1u, 0x34u, 0x12u, 0x78u, 0x56u, 0xBCu, 0x9Au}};
    const std::array<uint8_t, 8> identity =
        {{3u, 1u, 0x29u, 0x10u, 0x26u, 0x20u, 1u, 0u}};
    enqueue_and_poll(&rx, &control, &config, make_diag(0u, status), 100u);
    enqueue_and_poll(&rx, &control, &config, make_diag(1u, counters), 110u);
    enqueue_and_poll(&rx, &control, &config, make_diag(2u, identity), 120u);
    check((g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_SEEN_FLAGS] & 7u) == 7u,
          "status, counters, and identity all publish seen flags");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_STATUS_ARRIVAL_MS) == 100u &&
              read_u32(g_camera_diag_snapshot +
                       CAR_CAMERA_DIAG_OFFSET_COUNTERS_ARRIVAL_MS) == 110u &&
              read_u32(g_camera_diag_snapshot +
                       CAR_CAMERA_DIAG_OFFSET_IDENTITY_ARRIVAL_MS) == 120u,
          "kind-specific arrival times are retained");
    check(std::memcmp(g_camera_diag_snapshot +
                          CAR_CAMERA_DIAG_OFFSET_LATEST_STATUS,
                      status.data(), status.size()) == 0 &&
              std::memcmp(g_camera_diag_snapshot +
                              CAR_CAMERA_DIAG_OFFSET_LATEST_COUNTERS,
                          counters.data(), counters.size()) == 0 &&
              std::memcmp(g_camera_diag_snapshot +
                              CAR_CAMERA_DIAG_OFFSET_LATEST_IDENTITY,
                          identity.data(), identity.size()) == 0,
          "latest payloads match all three diagnostic kinds");

    for (uint16_t sequence = 3u; sequence < 20u; ++sequence) {
        const auto frame = make_diag(sequence, status);
        enqueue_and_poll(&rx, &control, &config, frame,
                         120u + sequence);
    }
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT) == 20u,
          "ring test accepts all 20 forward diagnostic records");
    check(read_u16(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_RING_COUNT) == 16u,
          "diagnostic ring count is bounded at 16");
    check(read_u16(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_RING_NEXT) == 4u,
          "diagnostic ring next index wraps modulo 16");
    check(read_u32(g_camera_diag_snapshot +
                   CAR_CAMERA_DIAG_OFFSET_OVERWRITTEN_COUNT) == 4u,
          "diagnostic ring reports four overwritten records");
    const uint8_t *first_chronological = g_camera_diag_snapshot +
        CAR_CAMERA_DIAG_OFFSET_RING + 4u * CAR_CAMERA_DIAG_RECORD_SIZE;
    const uint8_t *last_chronological = g_camera_diag_snapshot +
        CAR_CAMERA_DIAG_OFFSET_RING + 3u * CAR_CAMERA_DIAG_RECORD_SIZE;
    check(read_u16(first_chronological + 4u) == 4u &&
              read_u16(last_chronological + 4u) == 19u,
          "wrapped ring stores chronological bounded records");
    for (uint16_t index = 0u; index < CAR_CAMERA_DIAG_RING_CAPACITY; ++index) {
        const uint8_t *record = g_camera_diag_snapshot +
            CAR_CAMERA_DIAG_OFFSET_RING +
            index * CAR_CAMERA_DIAG_RECORD_SIZE;
        check(read_u16(record + 6u) == 0u,
              "ring record reserved field remains zero");
    }
    check_snapshot_valid();
    check(g_control_frames == 0u && g_receive_frames == 0u &&
              g_control_faults == 0u,
          "diagnostic kind scheduling does not reach control or FPS callbacks");
}

void test_profile_capability_preserves_public_encoder()
{
    std::array<uint8_t, CAR_REVERSE_PROFILE_STATE_FRAME_SIZE> legacy{};
    car_reverse_profile_state_encode(0u, 2u, 1u, 0x12345678u, 0x01020304u,
                                     legacy.data());
    check(legacy[7] == 1u,
          "public profile encoder remains boolean session-only");

    car_reverse_control_tx_t tx{};
    car_reverse_control_init(&tx, 0u);
    car_reverse_control_set_profile_state(
        &tx, 2u, 1u, 0x12345678u, 0x01020304u);
    std::vector<uint8_t> stream;
    for (uint32_t now_ms = 0u; now_ms < 300u; ++now_ms) {
        uint8_t byte = 0u;
        if (car_reverse_control_next_byte_paired(
                &tx, now_ms, 0u, &byte) != 0u) {
            stream.push_back(byte);
        }
    }
    bool found_production_state = false;
    for (size_t offset = 0u; offset +
             CAR_REVERSE_PROFILE_STATE_FRAME_SIZE <= stream.size(); ++offset) {
        if (stream[offset] == 0xA6u && stream[offset + 1u] == 0x5Au &&
            stream[offset + 2u] == 2u) {
            const uint16_t expected = car_reverse_control_crc16(
                stream.data() + offset, 18u);
            const uint16_t received = static_cast<uint16_t>(
                stream[offset + 18u] |
                (static_cast<uint16_t>(stream[offset + 19u]) << 8u));
            check(stream[offset + 7u] == 3u,
                  "production profile flags include session and diagnostics");
            check(received == expected,
                  "production capability flag recomputes profile-state CRC");
            found_production_state = true;
            break;
        }
    }
    check(found_production_state,
          "paired production scheduler emits a profile-state heartbeat");
}

bool write_snapshot(const std::string &path)
{
    car_vision_rx_t rx{};
    car_control_t control{};
    car_config_t config{};
    car_vision_rx_init(&rx);
    const std::array<uint8_t, 8> status =
        {{1u, 0x0Fu, 1u, 2u, 3u, 0x23u, 0x34u, 0x12u}};
    const std::array<uint8_t, 8> counters =
        {{2u, 1u, 0x34u, 0x12u, 0x78u, 0x56u, 0xBCu, 0x9Au}};
    const std::array<uint8_t, 8> identity =
        {{3u, 1u, 0x29u, 0x10u, 0x26u, 0x20u, 1u, 0u}};
    enqueue_and_poll(&rx, &control, &config, make_diag(1u, status), 100u);
    enqueue_and_poll(&rx, &control, &config, make_diag(2u, counters), 110u);
    enqueue_and_poll(&rx, &control, &config, make_diag(3u, identity), 120u);
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char *>(g_camera_diag_snapshot),
                 CAR_CAMERA_DIAG_SNAPSHOT_SIZE);
    return output.good();
}

} // namespace

extern "C" void car_receive_diag_note_parser_error(uint32_t)
{
    ++g_receive_parser_errors;
}

extern "C" void car_receive_diag_note_frame(uint16_t sequence,
                                             uint8_t,
                                             uint32_t)
{
    ++g_receive_frames;
    (void)sequence;
}

extern "C" void car_control_note_fault(car_control_t *, car_fault_t)
{
    ++g_control_faults;
}

extern "C" void car_control_on_vision_frame_at_with_flags_curve_and_profile(
    car_control_t *, const car_config_t *, uint16_t sequence, uint8_t,
    uint8_t, int16_t, uint8_t, uint16_t, uint8_t, uint8_t, uint32_t,
    uint32_t, uint8_t)
{
    ++g_control_frames;
    g_last_control_sequence = sequence;
}

int main(int argc, char **argv)
{
    if (argc == 3 && std::string(argv[1]) == "--write-snapshot") {
        if (!write_snapshot(argv[2])) {
            std::fprintf(stderr, "failed to write snapshot: %s\n", argv[2]);
            return 2;
        }
        std::puts("SNAPSHOT_EXPORT bytes=352 kinds=status,counters,identity");
        return 0;
    }

    test_initial_snapshot_and_mixed_frames();
    test_bad_diag_and_normal_corruption();
    test_diagnostic_sequence_flush_and_absence();
    test_partial_timeout_is_diagnostic_only();
    test_d1_prefix_timeouts_and_bad_crc_quarantine();
    test_stale_diagnostic_does_not_trip_queue_recovery();
    test_stale_v3_is_not_hidden_by_diagnostic_quarantine();
    test_snapshot_kind_fields_ring_wrap_and_export();
    test_profile_capability_preserves_public_encoder();
    if (g_failures != 0u) {
        std::printf("RESULT FAIL failures=%u\n", g_failures);
        return 1;
    }
    std::puts("RESULT PASS camera_uart_diag_v21_mcu");
    return 0;
}
