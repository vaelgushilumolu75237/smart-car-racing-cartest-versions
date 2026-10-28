#include "car_vision.h"

#include <string.h>

#include "receive_diag.h"
#if defined(_MSC_VER)
#include <intrin.h>
#endif

#if defined(_MSC_VER)
__declspec(align(32)) uint8_t
    g_camera_diag_snapshot[CAR_CAMERA_DIAG_SNAPSHOT_SIZE];
#elif defined(__GNUC__)
uint8_t g_camera_diag_snapshot[CAR_CAMERA_DIAG_SNAPSHOT_SIZE]
    __attribute__((aligned(32), used));
#else
_Alignas(32) uint8_t g_camera_diag_snapshot[CAR_CAMERA_DIAG_SNAPSHOT_SIZE];
#endif

static void publish_barrier(void);

#define CAMERA_DIAG_MAGIC 0x47444943u /* Little-endian "CIDG". */
#define CAMERA_DIAG_VERSION 1u
#define CAMERA_DIAG_BUILD_CODE 0x20261029u
#define CAMERA_DIAG_FEATURES 0x0001u

static uint16_t camera_diag_read_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] |
                      ((uint16_t)bytes[1] << 8u));
}

static uint32_t camera_diag_read_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) |
           ((uint32_t)bytes[3] << 24u);
}

static uint32_t camera_diag_read_generation(void)
{
#if defined(__GNUC__) || defined(__clang__)
    return __atomic_load_n(
        (const uint32_t *)(const void *)&g_camera_diag_snapshot[
            CAR_CAMERA_DIAG_OFFSET_GENERATION], __ATOMIC_ACQUIRE);
#elif defined(_MSC_VER)
    return (uint32_t)_InterlockedCompareExchange(
        (volatile long *)(void *)&g_camera_diag_snapshot[
            CAR_CAMERA_DIAG_OFFSET_GENERATION], 0, 0);
#else
    return camera_diag_read_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_GENERATION]);
#endif
}

static void camera_diag_write_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFu);
    bytes[1] = (uint8_t)(value >> 8u);
}

static void camera_diag_write_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)(value & 0xFFu);
    bytes[1] = (uint8_t)((value >> 8u) & 0xFFu);
    bytes[2] = (uint8_t)((value >> 16u) & 0xFFu);
    bytes[3] = (uint8_t)(value >> 24u);
}

static void camera_diag_write_generation(uint32_t value)
{
#if defined(__GNUC__) || defined(__clang__)
    __atomic_store_n(
        (uint32_t *)(void *)&g_camera_diag_snapshot[
            CAR_CAMERA_DIAG_OFFSET_GENERATION], value, __ATOMIC_RELEASE);
#elif defined(_MSC_VER)
    (void)_InterlockedExchange(
        (volatile long *)(void *)&g_camera_diag_snapshot[
            CAR_CAMERA_DIAG_OFFSET_GENERATION], (long)value);
#else
    camera_diag_write_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_GENERATION], value);
#endif
}

static uint32_t camera_diag_crc32(uint32_t committed_generation)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t index;
    uint8_t bit;

    for (index = 0u; index < CAR_CAMERA_DIAG_SNAPSHOT_SIZE; ++index) {
        uint8_t value;
        if (index >= CAR_CAMERA_DIAG_OFFSET_GENERATION &&
            index < CAR_CAMERA_DIAG_OFFSET_GENERATION + 4u) {
            value = (uint8_t)(committed_generation >>
                              ((index - CAR_CAMERA_DIAG_OFFSET_GENERATION) * 8u));
        } else if (index >= CAR_CAMERA_DIAG_OFFSET_CRC32 &&
                   index < CAR_CAMERA_DIAG_OFFSET_CRC32 + 4u) {
            value = 0u;
        } else {
            value = g_camera_diag_snapshot[index];
        }
        crc ^= value;
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 1u) != 0u
                      ? (crc >> 1u) ^ 0xEDB88320u
                      : crc >> 1u;
        }
    }
    return ~crc;
}

static uint32_t camera_diag_begin_snapshot_write(void)
{
    uint32_t committed = camera_diag_read_generation();
    uint32_t odd_generation;

    if ((committed & 1u) != 0u) {
        ++committed;
    }
    odd_generation = committed + 1u;
    camera_diag_write_generation(odd_generation);
    publish_barrier();
    return odd_generation + 1u;
}

static void camera_diag_commit_snapshot_write(uint32_t committed_generation)
{
    const uint32_t crc = camera_diag_crc32(committed_generation);
    camera_diag_write_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_CRC32], crc);
    publish_barrier();
    camera_diag_write_generation(committed_generation);
    publish_barrier();
}

static uint32_t saturating_increment_u32(uint32_t value)
{
    return value == 0xFFFFFFFFu ? value : value + 1u;
}

static void camera_diag_snapshot_init(void)
{
    uint32_t committed_generation;
    (void)memset(g_camera_diag_snapshot, 0,
                 sizeof(g_camera_diag_snapshot));
    committed_generation = camera_diag_begin_snapshot_write();
    camera_diag_write_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_MAGIC],
        CAMERA_DIAG_MAGIC);
    camera_diag_write_u16(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_VERSION],
        CAMERA_DIAG_VERSION);
    camera_diag_write_u16(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_SIZE],
        CAR_CAMERA_DIAG_SNAPSHOT_SIZE);
    camera_diag_commit_snapshot_write(committed_generation);
}

static void camera_diag_note_reject(void)
{
    uint32_t committed_generation = camera_diag_begin_snapshot_write();
    uint32_t rejected = camera_diag_read_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT]);
    camera_diag_write_u32(
        &g_camera_diag_snapshot[CAR_CAMERA_DIAG_OFFSET_REJECTED_COUNT],
        saturating_increment_u32(rejected));
    camera_diag_commit_snapshot_write(committed_generation);
}

static void camera_diag_reject_rx(car_vision_rx_t *rx)
{
    camera_diag_note_reject();
    if (rx != 0) {
        rx->diagnostic_discard_remaining = 0u;
        rx->diagnostic_discard_until_sync = 1u;
    }
}

static void camera_diag_reject_partial_rx(car_vision_rx_t *rx)
{
    uint8_t parser_length = rx->parser.length;
    camera_diag_note_reject();
    rx->diagnostic_discard_remaining =
        (uint8_t)(VP_FRAME_SIZE - parser_length);
    rx->diagnostic_discard_until_sync = 0u;
}

static uint8_t camera_diag_payload_valid(const uint8_t payload[8])
{
    if (payload == 0) {
        return 0u;
    }
    if (payload[0] == 1u) {
        return (uint8_t)((payload[1] & 0x80u) == 0u &&
                         payload[2] <= 9u && payload[3] <= 7u &&
                         payload[4] <= 7u && (payload[5] >> 4u) <= 3u &&
                         (payload[5] & 0x0Fu) <= 3u);
    }
    if (payload[0] == 2u) {
        return (uint8_t)((payload[1] & 0xFEu) == 0u);
    }
    if (payload[0] == 3u) {
        return (uint8_t)(payload[1] == 1u &&
                         camera_diag_read_u32(&payload[2]) ==
                             CAMERA_DIAG_BUILD_CODE &&
                         camera_diag_read_u16(&payload[6]) ==
                             CAMERA_DIAG_FEATURES);
    }
    return 0u;
}

static uint8_t camera_diag_sequence_valid(const car_vision_rx_t *rx,
                                          uint16_t sequence,
                                          uint32_t arrival_ms)
{
    uint16_t delta;
    if (rx->diagnostic_sequence_seen == 0u ||
        (uint32_t)(arrival_ms - rx->diagnostic_last_sequence_ms) >=
            CAR_CAMERA_DIAG_STALE_MS) {
        return 1u;
    }
    delta = (uint16_t)(sequence - rx->diagnostic_last_sequence);
    return (uint8_t)(delta != 0u && delta < 0x8000u);
}

static void camera_diag_accept(car_vision_rx_t *rx,
                               const vp_frame *frame,
                               uint32_t arrival_ms)
{
    uint32_t committed_generation = camera_diag_begin_snapshot_write();
    uint8_t *snapshot = g_camera_diag_snapshot;
    const uint8_t *payload = frame->diagnostic_payload;
    uint16_t ring_count = camera_diag_read_u16(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_RING_COUNT]);
    uint16_t ring_next = camera_diag_read_u16(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_RING_NEXT]);
    uint32_t accepted = camera_diag_read_u32(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT]);
    uint8_t *record;

    camera_diag_write_u32(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_ACCEPTED_COUNT],
        saturating_increment_u32(accepted));
    camera_diag_write_u32(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_LATEST_ARRIVAL_MS], arrival_ms);
    camera_diag_write_u16(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_LATEST_SEQUENCE], frame->sequence);
    if (payload[0] == 1u) {
        snapshot[CAR_CAMERA_DIAG_OFFSET_SEEN_FLAGS] |=
            CAR_CAMERA_DIAG_SEEN_STATUS;
        camera_diag_write_u32(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_STATUS_ARRIVAL_MS], arrival_ms);
        (void)memcpy(&snapshot[CAR_CAMERA_DIAG_OFFSET_LATEST_STATUS],
                     payload, VP_PAYLOAD_LENGTH);
    } else if (payload[0] == 2u) {
        snapshot[CAR_CAMERA_DIAG_OFFSET_SEEN_FLAGS] |=
            CAR_CAMERA_DIAG_SEEN_COUNTERS;
        camera_diag_write_u32(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_COUNTERS_ARRIVAL_MS], arrival_ms);
        (void)memcpy(&snapshot[CAR_CAMERA_DIAG_OFFSET_LATEST_COUNTERS],
                     payload, VP_PAYLOAD_LENGTH);
    } else {
        snapshot[CAR_CAMERA_DIAG_OFFSET_SEEN_FLAGS] |=
            CAR_CAMERA_DIAG_SEEN_IDENTITY;
        camera_diag_write_u32(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_IDENTITY_ARRIVAL_MS], arrival_ms);
        (void)memcpy(&snapshot[CAR_CAMERA_DIAG_OFFSET_LATEST_IDENTITY],
                     payload, VP_PAYLOAD_LENGTH);
    }

    if (ring_count >= CAR_CAMERA_DIAG_RING_CAPACITY) {
        uint32_t overwritten = camera_diag_read_u32(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_OVERWRITTEN_COUNT]);
        camera_diag_write_u32(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_OVERWRITTEN_COUNT],
            saturating_increment_u32(overwritten));
    } else {
        ++ring_count;
        camera_diag_write_u16(
            &snapshot[CAR_CAMERA_DIAG_OFFSET_RING_COUNT], ring_count);
    }
    record = &snapshot[CAR_CAMERA_DIAG_OFFSET_RING +
                       (uint32_t)ring_next * CAR_CAMERA_DIAG_RECORD_SIZE];
    camera_diag_write_u32(record, arrival_ms);
    camera_diag_write_u16(record + 4u, frame->sequence);
    camera_diag_write_u16(record + 6u, 0u);
    (void)memcpy(record + 8u, payload, VP_PAYLOAD_LENGTH);
    ring_next = (uint16_t)((ring_next + 1u) % CAR_CAMERA_DIAG_RING_CAPACITY);
    camera_diag_write_u16(
        &snapshot[CAR_CAMERA_DIAG_OFFSET_RING_NEXT], ring_next);
    rx->diagnostic_last_sequence = frame->sequence;
    rx->diagnostic_last_sequence_ms = arrival_ms;
    rx->diagnostic_sequence_seen = 1u;
    camera_diag_commit_snapshot_write(committed_generation);
}

static void publish_barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("dmb" ::: "memory");
#elif defined(__GNUC__)
    __asm volatile ("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif
}

static void note_protocol_error(car_vision_rx_t *rx,
                                car_control_t *control,
                                uint8_t discarded_byte,
                                uint32_t now_ms)
{
    ++rx->protocol_error_count;
    car_receive_diag_note_parser_error(now_ms);
    if (discarded_byte != 0u) {
        ++rx->discarded_byte_count;
    }
    rx->protocol_fault = 1u;
    ++rx->fault_generation;
    (void)control;
}

static void note_parser_status(car_vision_rx_t *rx,
                               car_control_t *control,
                               vp_status status,
                               uint8_t received_version,
                               uint8_t discarded_byte,
                               uint32_t now_ms)
{
    if (rx == 0 || status == VP_STATUS_NONE || status == VP_STATUS_FRAME) {
        return;
    }
    if (received_version == VP_VERSION_D1) {
        camera_diag_reject_rx(rx);
        return;
    }
    rx->last_protocol_status = (uint8_t)status;
    rx->last_received_protocol_version = received_version;
    if (status == VP_STATUS_VERSION_MISMATCH) {
        ++rx->protocol_version_mismatch_count;
    } else if (status == VP_STATUS_BAD_CRC) {
        ++rx->protocol_crc_error_count;
    } else if (status == VP_STATUS_BAD_FIELDS) {
        ++rx->protocol_field_error_count;
    } else {
        ++rx->protocol_format_error_count;
    }
    note_protocol_error(rx, control, discarded_byte, now_ms);
}

static void dispatch_new_rx_faults(car_vision_rx_t *rx,
                                   car_control_t *control)
{
    uint32_t parser_generation;
    uint32_t isr_generation;
    uint8_t uart_fault;
    uint8_t overflow_fault;
    uint8_t protocol_fault;
    uint8_t stale_fault;

    parser_generation = rx->fault_generation;
    isr_generation = rx->isr_fault_generation;
    if (parser_generation == rx->fault_generation_reported &&
        isr_generation == rx->isr_fault_generation_reported) {
        return;
    }

    /* Each counter has one writer, so an interrupt cannot lose an increment. */
    rx->fault_generation_reported = parser_generation;
    rx->isr_fault_generation_reported = isr_generation;
    uart_fault = rx->uart_fault;
    overflow_fault = rx->overflow_fault;
    protocol_fault = rx->protocol_fault;
    stale_fault = rx->stale_fault;

    if (uart_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_UART);
    }
    if (overflow_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_RX_OVERFLOW);
    }
    if (protocol_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_PROTOCOL);
    }
    if (stale_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_RX_STALE);
    }
}

static uint8_t parser_has_diagnostic_version(const vp_parser *parser)
{
    return (uint8_t)(parser != 0 && parser->length >= 3u &&
                     parser->buffer[2] == VP_VERSION_D1);
}

static uint16_t queued_bytes_available(const car_vision_rx_t *rx,
                                       uint16_t head_snapshot)
{
    return (uint16_t)(head_snapshot - rx->tail) & CAR_VISION_RX_RING_MASK;
}

static uint8_t queued_byte_at(const car_vision_rx_t *rx,
                              uint16_t head_snapshot,
                              uint16_t offset,
                              uint8_t *byte_out)
{
    const uint16_t available = queued_bytes_available(rx, head_snapshot);
    const uint16_t index = (uint16_t)(rx->tail + offset) &
                           CAR_VISION_RX_RING_MASK;
    if (byte_out == 0 || offset >= available) {
        return 0u;
    }
    *byte_out = rx->ring[index].byte;
    return 1u;
}

static uint8_t queue_starts_control_header(const car_vision_rx_t *rx,
                                           uint16_t head_snapshot)
{
    uint8_t first;
    uint8_t second;
    uint8_t version;
    return (uint8_t)(queued_byte_at(rx, head_snapshot, 0u, &first) != 0u &&
                     first == VP_SYNC_BYTE_0 &&
                     queued_byte_at(rx, head_snapshot, 1u, &second) != 0u &&
                     second == VP_SYNC_BYTE_1 &&
                     queued_byte_at(rx, head_snapshot, 2u, &version) != 0u &&
                     (version == VP_VERSION_V2 ||
                      version == VP_VERSION_V3));
}

static uint8_t queue_has_complete_valid_control_frame(
    const car_vision_rx_t *rx,
    uint16_t head_snapshot)
{
    uint8_t bytes[VP_FRAME_SIZE];
    vp_frame frame;
    uint16_t index;
    if (queued_bytes_available(rx, head_snapshot) < VP_FRAME_SIZE ||
        queue_starts_control_header(rx, head_snapshot) == 0u) {
        return 0u;
    }
    for (index = 0u; index < VP_FRAME_SIZE; ++index) {
        if (queued_byte_at(rx, head_snapshot, index, &bytes[index]) == 0u) {
            return 0u;
        }
    }
    return (uint8_t)(vp_unpack_ex(bytes, &frame) == VP_STATUS_FRAME &&
                     (frame.version == VP_VERSION_V2 ||
                      frame.version == VP_VERSION_V3));
}

static uint8_t queue_has_incomplete_sync_prefix(
    const car_vision_rx_t *rx,
    uint16_t head_snapshot)
{
    uint8_t first;
    uint8_t second;
    const uint16_t available = queued_bytes_available(rx, head_snapshot);
    if (available == 0u || available >= 3u ||
        queued_byte_at(rx, head_snapshot, 0u, &first) == 0u ||
        first != VP_SYNC_BYTE_0) {
        return 0u;
    }
    if (available == 1u) {
        return 1u;
    }
    return (uint8_t)(queued_byte_at(rx, head_snapshot, 1u, &second) != 0u &&
                     second == VP_SYNC_BYTE_1);
}

static uint8_t queue_starts_diagnostic(const car_vision_rx_t *rx,
                                      uint16_t head_snapshot)
{
    uint8_t first;
    uint8_t second;
    uint8_t third;
    const uint8_t length = rx->parser.length;

    if (parser_has_diagnostic_version(&rx->parser) != 0u) {
        return 1u;
    }
    if (length == 0u) {
        return (uint8_t)(queued_byte_at(rx, head_snapshot, 0u, &first) != 0u &&
                         first == VP_SYNC_BYTE_0 &&
                         queued_byte_at(rx, head_snapshot, 1u, &second) != 0u &&
                         second == VP_SYNC_BYTE_1 &&
                         queued_byte_at(rx, head_snapshot, 2u, &third) != 0u &&
                         third == VP_VERSION_D1);
    }
    if (length == 1u && rx->parser.buffer[0] == VP_SYNC_BYTE_0) {
        return (uint8_t)(queued_byte_at(rx, head_snapshot, 0u, &first) != 0u &&
                         first == VP_SYNC_BYTE_1 &&
                         queued_byte_at(rx, head_snapshot, 1u, &second) != 0u &&
                         second == VP_VERSION_D1);
    }
    if (length == 2u && rx->parser.buffer[0] == VP_SYNC_BYTE_0 &&
        rx->parser.buffer[1] == VP_SYNC_BYTE_1) {
        return (uint8_t)(queued_byte_at(rx, head_snapshot, 0u, &first) != 0u &&
                         first == VP_VERSION_D1);
    }
    return 0u;
}

static void discard_stale_diagnostic_frame(car_vision_rx_t *rx,
                                           uint16_t head_snapshot)
{
    uint16_t remaining = (uint16_t)(VP_FRAME_SIZE - rx->parser.length);
    uint16_t available = queued_bytes_available(rx, head_snapshot);
    camera_diag_note_reject();
    while (remaining != 0u && available != 0u &&
           rx->tail != head_snapshot) {
        if (queue_starts_control_header(rx, head_snapshot) != 0u) {
            if (available < VP_FRAME_SIZE) {
                break;
            }
            /* A full normal frame at the current sync is a new stream item. */
            break;
        }
        rx->tail = (uint16_t)((rx->tail + 1u) &
                              CAR_VISION_RX_RING_MASK);
        --remaining;
        --available;
    }
    rx->partial_active = 0u;
    vp_parser_init(&rx->parser);
    rx->diagnostic_discard_remaining = (uint8_t)remaining;
    rx->diagnostic_discard_until_sync = remaining == 0u ? 1u : 0u;
}

void car_vision_rx_init(car_vision_rx_t *rx)
{
    if (rx == 0) {
        return;
    }
    (void)memset(rx, 0, sizeof(*rx));
    vp_parser_init(&rx->parser);
    camera_diag_snapshot_init();
}

void car_vision_rx_isr_byte(car_vision_rx_t *rx,
                            uint8_t byte,
                            uint32_t arrival_ms)
{
    uint16_t head;
    uint16_t next;
    if (rx == 0) {
        return;
    }
    ++rx->rx_byte_count;
    head = rx->head;
    publish_barrier();
    next = (uint16_t)((head + 1u) & CAR_VISION_RX_RING_MASK);
    if (next == rx->tail) {
        ++rx->queue_overflow_count;
        rx->overflow_fault = 1u;
        ++rx->isr_fault_generation;
        return;
    }
    rx->ring[head].byte = byte;
    rx->ring[head].arrival_ms = arrival_ms;
    publish_barrier();
    rx->head = next;
}

void car_vision_rx_isr_error(car_vision_rx_t *rx)
{
    if (rx != 0) {
        ++rx->uart_error_count;
        rx->uart_fault = 1u;
        ++rx->isr_fault_generation;
    }
}

void car_vision_rx_poll(car_vision_rx_t *rx,
                        car_control_t *control,
                        const car_config_t *config,
                        uint16_t head_snapshot,
                        uint32_t now_ms,
                        uint8_t pe7_raw_high)
{
    uint16_t tail;
    uint8_t byte;
    uint8_t parser_length_before;
    uint8_t discard_until_sync = 0u;
    uint32_t arrival_ms;
    vp_status parser_status;
    vp_frame frame;

    if (rx == 0 || control == 0 || config == 0) {
        return;
    }

    dispatch_new_rx_faults(rx, control);
    if (control->rx_recovery_flush_required != 0u) {
        return;
    }

    while (rx->tail != head_snapshot) {
        dispatch_new_rx_faults(rx, control);
        if (control->rx_recovery_flush_required != 0u) {
            return;
        }
        tail = rx->tail;
        publish_barrier();
        arrival_ms = rx->ring[tail].arrival_ms;
        if ((uint32_t)(now_ms - arrival_ms) >= CAR_VISION_MAX_QUEUE_AGE_MS) {
            if (rx->diagnostic_discard_remaining != 0u) {
                if (queue_starts_control_header(rx, head_snapshot) != 0u &&
                    queue_has_complete_valid_control_frame(
                        rx, head_snapshot) != 0u) {
                    rx->diagnostic_discard_remaining = 0u;
                    rx->diagnostic_discard_until_sync = 0u;
                } else if (queue_starts_control_header(rx, head_snapshot) != 0u &&
                           queued_bytes_available(rx, head_snapshot) <
                               VP_FRAME_SIZE) {
                    break;
                } else if (queue_has_incomplete_sync_prefix(
                               rx, head_snapshot) != 0u) {
                    break;
                } else {
                    rx->tail = (uint16_t)((tail + 1u) &
                                          CAR_VISION_RX_RING_MASK);
                    --rx->diagnostic_discard_remaining;
                    if (rx->diagnostic_discard_remaining == 0u) {
                        rx->diagnostic_discard_until_sync = 1u;
                    }
                    continue;
                }
            }
            if (rx->diagnostic_discard_until_sync != 0u &&
                queue_starts_control_header(rx, head_snapshot) != 0u) {
                ++rx->queue_age_reject_count;
                rx->stale_fault = 1u;
                ++rx->fault_generation;
                rx->tail = rx->head;
                rx->partial_active = 0u;
                rx->diagnostic_discard_remaining = 0u;
                rx->diagnostic_discard_until_sync = 0u;
                vp_parser_init(&rx->parser);
                dispatch_new_rx_faults(rx, control);
                return;
            }
            if (rx->diagnostic_discard_until_sync != 0u &&
                queue_starts_diagnostic(rx, head_snapshot) != 0u) {
                discard_stale_diagnostic_frame(rx, head_snapshot);
                continue;
            }
            if (rx->diagnostic_discard_until_sync != 0u &&
                queue_has_incomplete_sync_prefix(rx, head_snapshot) != 0u) {
                break;
            }
            if (rx->diagnostic_discard_until_sync != 0u) {
                rx->tail = (uint16_t)((tail + 1u) &
                                      CAR_VISION_RX_RING_MASK);
                continue;
            }
            if (queue_starts_diagnostic(rx, head_snapshot) != 0u) {
                discard_stale_diagnostic_frame(rx, head_snapshot);
                continue;
            }
            ++rx->queue_age_reject_count;
            rx->stale_fault = 1u;
            ++rx->fault_generation;
            rx->tail = rx->head;
            rx->partial_active = 0u;
            vp_parser_init(&rx->parser);
            dispatch_new_rx_faults(rx, control);
            return;
        }
        if (rx->diagnostic_discard_remaining != 0u) {
            if (queue_starts_control_header(rx, head_snapshot) != 0u &&
                queue_has_complete_valid_control_frame(
                    rx, head_snapshot) != 0u) {
                rx->diagnostic_discard_remaining = 0u;
                rx->diagnostic_discard_until_sync = 0u;
            } else if (queue_starts_control_header(rx, head_snapshot) != 0u &&
                       queued_bytes_available(rx, head_snapshot) <
                           VP_FRAME_SIZE) {
                break;
            } else if (queue_has_incomplete_sync_prefix(
                           rx, head_snapshot) != 0u) {
                break;
            } else {
                rx->tail = (uint16_t)((tail + 1u) &
                                      CAR_VISION_RX_RING_MASK);
                --rx->diagnostic_discard_remaining;
                if (rx->diagnostic_discard_remaining == 0u) {
                    rx->diagnostic_discard_until_sync = 1u;
                }
                continue;
            }
        }

        if (rx->diagnostic_discard_until_sync != 0u) {
            if (queue_starts_control_header(rx, head_snapshot) != 0u ||
                queue_starts_diagnostic(rx, head_snapshot) != 0u) {
                rx->diagnostic_discard_until_sync = 0u;
            } else if (queue_has_incomplete_sync_prefix(
                           rx, head_snapshot) != 0u) {
                break;
            } else {
                rx->tail = (uint16_t)((tail + 1u) &
                                      CAR_VISION_RX_RING_MASK);
                continue;
            }
        }

        byte = rx->ring[tail].byte;
        publish_barrier();
        rx->tail = (uint16_t)((tail + 1u) & CAR_VISION_RX_RING_MASK);

        if (discard_until_sync != 0u) {
            if (byte != VP_SYNC_BYTE_0) {
                ++rx->discarded_byte_count;
                continue;
            }
            discard_until_sync = 0u;
        }

        if (rx->partial_active != 0u &&
            (uint32_t)(arrival_ms - rx->partial_last_byte_ms) >=
                CAR_VISION_PARTIAL_TIMEOUT_MS) {
            if (parser_has_diagnostic_version(&rx->parser) != 0u) {
                camera_diag_reject_partial_rx(rx);
                rx->partial_active = 0u;
                vp_parser_init(&rx->parser);
                if (byte == VP_SYNC_BYTE_0) {
                    uint8_t next_byte;
                    uint8_t next_version;
                    const uint16_t queued =
                        queued_bytes_available(rx, head_snapshot);
                    if (queued == 0u ||
                        (queued_byte_at(rx, head_snapshot, 0u,
                                        &next_byte) != 0u &&
                         next_byte == VP_SYNC_BYTE_1 && queued < 2u)) {
                        rx->tail = tail;
                        break;
                    }
                    if (queued >= 2u &&
                        queued_byte_at(rx, head_snapshot, 0u,
                                       &next_byte) != 0u &&
                        queued_byte_at(rx, head_snapshot, 1u,
                                       &next_version) != 0u &&
                        next_byte == VP_SYNC_BYTE_1 &&
                        (next_version == VP_VERSION_V2 ||
                         next_version == VP_VERSION_V3)) {
                        rx->diagnostic_discard_remaining = 0u;
                        rx->diagnostic_discard_until_sync = 0u;
                    } else {
                        if (rx->diagnostic_discard_remaining != 0u) {
                            --rx->diagnostic_discard_remaining;
                        }
                        if (rx->diagnostic_discard_remaining == 0u) {
                            rx->diagnostic_discard_until_sync = 1u;
                        }
                        continue;
                    }
                } else {
                    if (rx->diagnostic_discard_remaining != 0u) {
                        --rx->diagnostic_discard_remaining;
                    }
                    if (rx->diagnostic_discard_remaining == 0u) {
                        rx->diagnostic_discard_until_sync = 1u;
                    }
                    continue;
                }
            } else {
                ++rx->partial_timeout_count;
                rx->stale_fault = 1u;
                ++rx->fault_generation;
                rx->partial_active = 0u;
                vp_parser_init(&rx->parser);
                dispatch_new_rx_faults(rx, control);
                return;
            }
        }
        parser_length_before = rx->parser.length;

        if (parser_length_before == 0u && byte != VP_SYNC_BYTE_0) {
            note_parser_status(rx, control, VP_STATUS_BAD_HEADER, 0u,
                               1u, now_ms);
            dispatch_new_rx_faults(rx, control);
            discard_until_sync = 1u;
            continue;
        }
        if (parser_length_before == 1u && byte != VP_SYNC_BYTE_1 &&
            byte != VP_SYNC_BYTE_0) {
            note_parser_status(rx, control, VP_STATUS_BAD_HEADER, 0u,
                               1u, now_ms);
            dispatch_new_rx_faults(rx, control);
            discard_until_sync = 1u;
            continue;
        }

        if (vp_parser_feed(&rx->parser, byte, &frame) != 0u) {
            rx->partial_active = 0u;
            dispatch_new_rx_faults(rx, control);
            if (control->rx_recovery_flush_required != 0u) {
                return;
            }
            if (frame.version == VP_VERSION_D1) {
                if (camera_diag_payload_valid(frame.diagnostic_payload) == 0u ||
                    camera_diag_sequence_valid(rx, frame.sequence,
                                               arrival_ms) == 0u) {
                    camera_diag_reject_rx(rx);
                } else {
                    camera_diag_accept(rx, &frame, arrival_ms);
                }
            } else {
                car_receive_diag_note_frame(
                    frame.sequence, (frame.flags & VP_FLAG_VALID) != 0u,
                    arrival_ms);
                car_control_on_vision_frame_at_with_flags_curve_and_profile(
                    control, config, frame.sequence, frame.flags,
                    frame.confidence, frame.lateral_error, frame.curve_phase,
                    frame.slowdown_request_permille, frame.version,
                    frame.profile_ack, arrival_ms, now_ms, pe7_raw_high);
            }
            dispatch_new_rx_faults(rx, control);
        } else if ((parser_status = vp_parser_last_status(&rx->parser)) !=
                   VP_STATUS_NONE) {
            note_parser_status(rx, control, parser_status,
                               vp_parser_last_version(&rx->parser),
                               0u, now_ms);
            dispatch_new_rx_faults(rx, control);
            discard_until_sync = 1u;
        }

        if (rx->parser.length != 0u) {
            rx->partial_active = 1u;
            rx->partial_last_byte_ms = arrival_ms;
        } else {
            rx->partial_active = 0u;
        }
        if (control->rx_recovery_flush_required != 0u) {
            return;
        }
    }

    if (rx->partial_active != 0u &&
        (uint32_t)(now_ms - rx->partial_last_byte_ms) >=
            CAR_VISION_PARTIAL_TIMEOUT_MS) {
        if (parser_has_diagnostic_version(&rx->parser) != 0u) {
            camera_diag_reject_partial_rx(rx);
        } else {
            ++rx->partial_timeout_count;
            rx->stale_fault = 1u;
            ++rx->fault_generation;
        }
        rx->partial_active = 0u;
        vp_parser_init(&rx->parser);
        dispatch_new_rx_faults(rx, control);
    }
}

void car_vision_rx_clear(car_vision_rx_t *rx)
{
    if (rx == 0) {
        return;
    }
    rx->tail = rx->head;
    rx->uart_fault = 0u;
    rx->overflow_fault = 0u;
    rx->protocol_fault = 0u;
    rx->stale_fault = 0u;
    rx->fault_generation_reported = rx->fault_generation;
    rx->isr_fault_generation_reported = rx->isr_fault_generation;
    rx->partial_active = 0u;
    rx->partial_last_byte_ms = 0u;
    rx->diagnostic_sequence_seen = 0u;
    rx->diagnostic_last_sequence = 0u;
    rx->diagnostic_last_sequence_ms = 0u;
    rx->diagnostic_discard_until_sync = 0u;
    rx->diagnostic_discard_remaining = 0u;
    vp_parser_init(&rx->parser);
}
