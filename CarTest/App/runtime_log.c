#include "runtime_log.h"

#include "commissioning.h"

#include <stddef.h>
#include <string.h>

#ifndef CAR_RUNTIME_LOG_HOST_TEST
#include "runtime_log_build_id.h"
#include "stm32h7xx_hal.h"
#endif

#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "The exported CarTest runtime log ABI requires little-endian storage."
#endif

#define CAR_RUNTIME_LOG_HEADER_MAGIC 0x474F4C52u /* bytes RLOG */
#define CAR_RUNTIME_LOG_DESCRIPTOR_MAGIC 0x53444C52u /* bytes RLDS */
#define CAR_RUNTIME_LOG_DESCRIPTOR_VERSION 1u
#define CAR_RUNTIME_LOG_ENDIAN_TAG 0x12345678u

#if defined(CAR_RUNTIME_LOG_HOST_TEST)
#if defined(CAR_RUNTIME_LOG_HOST_TEST_USE_TARGET_BUILD_ID)
#include "runtime_log_build_id.h"
#else
#define CAR_RUNTIME_LOG_BUILD_ID_BYTES \
    { 0x43u, 0x41u, 0x52u, 0x54u, 0x45u, 0x53u, 0x54u, 0x2Du, \
      0x48u, 0x4Fu, 0x53u, 0x54u, 0x2Du, 0x56u, 0x31u, 0x00u }
#endif
#endif

#if defined(__GNUC__)
#define CAR_RUNTIME_LOG_RAM_OBJECT \
    __attribute__((section(".runtime_log"), aligned(32), used))
#define CAR_RUNTIME_LOG_ROM_OBJECT \
    __attribute__((section(".runtime_log_descriptor"), aligned(4), used))
#else
#define CAR_RUNTIME_LOG_RAM_OBJECT
#define CAR_RUNTIME_LOG_ROM_OBJECT
#endif

#if defined(CAR_RUNTIME_LOG_HOST_TEST)
#define LOG_DMB() ((void)0)
#define LOG_DSB() ((void)0)
#else
#define LOG_DMB() __DMB()
#define LOG_DSB() __DSB()
#endif

_Static_assert(sizeof(car_runtime_log_record_t) ==
                   CAR_RUNTIME_LOG_RECORD_SIZE,
               "runtime log record ABI size changed");
_Static_assert(offsetof(car_runtime_log_record_t, time_ms) == 0u,
               "runtime log record time offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, session_id) == 4u,
               "runtime log record session offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, left_encoder_counts) == 8u,
               "runtime log record left encoder offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, right_encoder_counts) == 16u,
               "runtime log record right encoder offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, fault_generation) == 24u,
               "runtime log record fault offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, servo_requested_us) == 28u,
               "runtime log record servo request offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, motor_requested_permille) == 34u,
               "runtime log record motor request offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, left_applied_permille) == 36u,
               "runtime log record left duty offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, right_applied_permille) == 38u,
               "runtime log record right duty offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, vision_error_norm) == 48u,
               "runtime log record vision offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, auto_recovery_count) == 52u,
               "runtime log record recovery offset changed");
_Static_assert(offsetof(car_runtime_log_record_t, event_flags) == 56u,
               "runtime log record event offset changed");
_Static_assert(offsetof(car_runtime_log_header_t, count) == 16u,
               "runtime log header count offset changed");
_Static_assert(offsetof(car_runtime_log_header_t, records_crc32) == 36u,
               "runtime log record CRC offset changed");
_Static_assert(offsetof(car_runtime_log_header_t, header_crc32) == 40u,
               "runtime log header CRC offset changed");
_Static_assert(offsetof(car_runtime_log_header_t, build_id) == 44u,
               "runtime log build ID offset changed");
_Static_assert(sizeof(car_runtime_log_header_t) ==
                   CAR_RUNTIME_LOG_HEADER_SIZE,
               "runtime log header ABI size changed");
_Static_assert(offsetof(car_runtime_log_descriptor_t, build_id) == 28u,
               "runtime log descriptor build ID offset changed");
_Static_assert(sizeof(car_runtime_log_descriptor_t) == 64u,
               "runtime log descriptor ABI size changed");
_Static_assert(sizeof(car_runtime_log_storage_t) ==
                   CAR_RUNTIME_LOG_BUFFER_SIZE,
               "runtime log storage footprint changed");
_Static_assert((CAR_RUNTIME_LOG_RECORD_SIZE % CAR_RUNTIME_LOG_ALIGNMENT) == 0u,
               "runtime log records must occupy whole cache lines");

car_runtime_log_storage_t car_runtime_log_storage CAR_RUNTIME_LOG_RAM_OBJECT;

const car_runtime_log_descriptor_t car_runtime_log_rom_descriptor
    CAR_RUNTIME_LOG_ROM_OBJECT = {
        .magic = CAR_RUNTIME_LOG_DESCRIPTOR_MAGIC,
        .descriptor_version = CAR_RUNTIME_LOG_DESCRIPTOR_VERSION,
        .descriptor_size = sizeof(car_runtime_log_descriptor_t),
        .log_version = CAR_RUNTIME_LOG_VERSION,
        .header_size = CAR_RUNTIME_LOG_HEADER_SIZE,
        .record_size = CAR_RUNTIME_LOG_RECORD_SIZE,
        .capacity = CAR_RUNTIME_LOG_CAPACITY,
        .period_ms = CAR_RUNTIME_LOG_PERIOD_MS,
        .alignment = CAR_RUNTIME_LOG_ALIGNMENT,
        .buffer_size = CAR_RUNTIME_LOG_BUFFER_SIZE,
        .endian_tag = CAR_RUNTIME_LOG_ENDIAN_TAG,
        .build_id = CAR_RUNTIME_LOG_BUILD_ID_BYTES,
        .reserved = {0u, 0u, 0u, 0u, 0u}
    };

static uint32_t record_crc_state = 0xFFFFFFFFu;
static uint32_t session_counter;
static uint32_t last_periodic_ms;
static uint32_t observed_fault_generation;
static uint32_t observed_auto_recovery_count;
static uint8_t observed_mode;
static uint8_t observed_fault;
static uint8_t observed_recovery_active;
static uint8_t observed_recovery_frame_count;
static uint8_t observed_manual_run_request;
static uint8_t observed_vision_hold_active;
static uint8_t observed_vision_hold_expired;
static uint8_t observed_state_valid;
static uint8_t stop_pending;
static uint8_t pending_stop_reason;
static uint16_t pending_event_flags;
static uint8_t active_session_kind;
static uint8_t recording_enabled;

static uint32_t crc32_update(uint32_t crc, const uint8_t *bytes,
                             uint32_t length)
{
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < length; ++i) {
        crc ^= bytes[i];
        for (bit = 0u; bit < 8u; ++bit) {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return crc;
}

uint32_t car_runtime_log_crc32(const void *bytes, uint32_t length)
{
    if (bytes == 0 && length != 0u) {
        return 0u;
    }
    return crc32_update(0xFFFFFFFFu, (const uint8_t *)bytes, length) ^
           0xFFFFFFFFu;
}

#ifndef CAR_RUNTIME_LOG_HOST_TEST
static void clean_committed_range(const void *address, uint32_t length)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u && length != 0u) {
        const uintptr_t mask = (uintptr_t)(CAR_RUNTIME_LOG_ALIGNMENT - 1u);
        const uintptr_t start = (uintptr_t)address & ~mask;
        const uintptr_t end = ((uintptr_t)address + length + mask) & ~mask;
        SCB_CleanDCache_by_Addr((uint32_t *)start,
                               (int32_t)(end - start));
    }
    LOG_DSB();
}
#else
static void clean_committed_range(const void *address, uint32_t length)
{
    (void)address;
    (void)length;
    LOG_DSB();
}
#endif

static uint32_t header_crc(const car_runtime_log_header_t *header)
{
    car_runtime_log_header_t copy = *header;
    copy.header_crc32 = 0u;
    return car_runtime_log_crc32(&copy, sizeof(copy));
}

/* The committed record and metadata are visible before count is published. */
static void publish_header(const car_runtime_log_header_t *next)
{
    uint8_t *destination = (uint8_t *)&car_runtime_log_storage.header;
    const uint8_t *source = (const uint8_t *)next;
    const uint32_t count_offset =
        (uint32_t)offsetof(car_runtime_log_header_t, count);
    const uint32_t count_end = count_offset + sizeof(next->count);

    memcpy(destination, source, count_offset);
    memcpy(destination + count_end, source + count_end,
           sizeof(*next) - count_end);
    LOG_DMB();
    car_runtime_log_storage.header.count = next->count;
    LOG_DMB();
    clean_committed_range(&car_runtime_log_storage.header, sizeof(*next));
}

static void prepare_header(car_runtime_log_header_t *header)
{
    header->magic = CAR_RUNTIME_LOG_HEADER_MAGIC;
    header->version = CAR_RUNTIME_LOG_VERSION;
    header->header_size = CAR_RUNTIME_LOG_HEADER_SIZE;
    header->record_size = CAR_RUNTIME_LOG_RECORD_SIZE;
    header->capacity = CAR_RUNTIME_LOG_CAPACITY;
    header->period_ms = CAR_RUNTIME_LOG_PERIOD_MS;
    header->header_crc32 = 0u;
    header->header_crc32 = header_crc(header);
}

void car_runtime_log_init(void)
{
    car_runtime_log_header_t initial = {0};

    session_counter = 0u;
    record_crc_state = 0xFFFFFFFFu;
    active_session_kind = CAR_RUNTIME_LOG_SESSION_NONE;
    last_periodic_ms = 0u;
    observed_fault_generation = 0u;
    observed_auto_recovery_count = 0u;
    observed_mode = 0u;
    observed_fault = 0u;
    observed_recovery_active = 0u;
    observed_recovery_frame_count = 0u;
    observed_manual_run_request = 0u;
    observed_vision_hold_active = 0u;
    observed_vision_hold_expired = 0u;
    observed_state_valid = 0u;
    stop_pending = 0u;
    pending_stop_reason = CAR_RUNTIME_LOG_STOP_NONE;
    pending_event_flags = 0u;
    recording_enabled = 1u;

    initial.state = CAR_RUNTIME_LOG_STATE_IDLE;
    memcpy(initial.build_id, car_runtime_log_rom_descriptor.build_id,
           sizeof(initial.build_id));
    prepare_header(&initial);
    publish_header(&initial);
}

void car_runtime_log_invalidate(void)
{
    const car_runtime_log_header_t invalid = {0};

    session_counter = 0u;
    record_crc_state = 0xFFFFFFFFu;
    last_periodic_ms = 0u;
    observed_fault_generation = 0u;
    observed_auto_recovery_count = 0u;
    observed_mode = 0u;
    observed_fault = 0u;
    observed_recovery_active = 0u;
    observed_recovery_frame_count = 0u;
    observed_manual_run_request = 0u;
    observed_vision_hold_active = 0u;
    observed_vision_hold_expired = 0u;
    observed_state_valid = 0u;
    stop_pending = 0u;
    pending_stop_reason = CAR_RUNTIME_LOG_STOP_NONE;
    pending_event_flags = 0u;
    active_session_kind = CAR_RUNTIME_LOG_SESSION_NONE;
    recording_enabled = 0u;
    publish_header(&invalid);
}

uint8_t car_runtime_log_state(void)
{
    return car_runtime_log_storage.header.state;
}

static void begin_session(uint32_t now_ms,
                          car_runtime_log_session_kind_t session_kind)
{
    car_runtime_log_header_t next = {0};

    if (session_counter == 0xFFFFFFFFu) {
        session_counter = 1u;
    } else {
        ++session_counter;
        if (session_counter == 0u) {
            session_counter = 1u;
        }
    }

    next.state = CAR_RUNTIME_LOG_STATE_ACTIVE;
    next.session_id = session_counter;
    next.start_ms = now_ms;
    memcpy(next.build_id, car_runtime_log_rom_descriptor.build_id,
           sizeof(next.build_id));
    prepare_header(&next);
    publish_header(&next);

    record_crc_state = 0xFFFFFFFFu;
    last_periodic_ms = now_ms;
    active_session_kind = (uint8_t)session_kind;
    observed_state_valid = 0u;
    stop_pending = 0u;
    pending_stop_reason = CAR_RUNTIME_LOG_STOP_NONE;
    pending_event_flags = 0u;
}

static void make_record(car_runtime_log_record_t *record,
                        const car_runtime_log_sample_t *sample,
                        uint16_t event_flags)
{
    record->time_ms = sample->now_ms;
    record->session_id = car_runtime_log_storage.header.session_id;
    record->left_encoder_counts = sample->left_encoder_counts;
    record->right_encoder_counts = sample->right_encoder_counts;
    record->fault_generation = sample->fault_generation;
    record->servo_requested_us = sample->servo_requested_us;
    record->servo_limited_us = sample->servo_limited_us;
    record->servo_applied_us = sample->servo_applied_us;
    record->motor_requested_permille = sample->motor_requested_permille;
    record->left_applied_permille = sample->left_applied_permille;
    record->right_applied_permille = sample->right_applied_permille;
    record->motor_ccr1_counts = sample->motor_ccr1_counts;
    record->motor_ccr2_counts = sample->motor_ccr2_counts;
    record->motor_ccr3_counts = sample->motor_ccr3_counts;
    record->motor_ccr4_counts = sample->motor_ccr4_counts;
    record->vision_error_norm = sample->vision_error_norm;
    record->vision_sequence = sample->vision_sequence;
    record->auto_recovery_count = sample->auto_recovery_count;
    record->event_flags = event_flags;
    record->mode = sample->mode;
    record->fault = sample->fault;
    record->vision_confidence = sample->vision_confidence;
    record->vision_flags = sample->vision_flags;
    record->recovery_frame_count = sample->recovery_frame_count;
    record->recovery_active = sample->recovery_active;
}

static void publish_dropped_sample(void)
{
    car_runtime_log_header_t next = car_runtime_log_storage.header;

    next.flags |= (CAR_RUNTIME_LOG_FLAG_FULL |
                   CAR_RUNTIME_LOG_FLAG_TRUNCATED);
    if (next.dropped_count != 0xFFFFFFFFu) {
        ++next.dropped_count;
    }
    prepare_header(&next);
    publish_header(&next);
}

/* Normal events/periodic rows may not consume the final-stop reserve slot. */
static uint8_t append_record(const car_runtime_log_sample_t *sample,
                             uint16_t event_flags,
                             uint8_t final_record)
{
    car_runtime_log_header_t next;
    car_runtime_log_record_t record;
    const uint32_t count = car_runtime_log_storage.header.count;
    const uint32_t usable_normal_count = CAR_RUNTIME_LOG_CAPACITY - 1u;

    if (count >= CAR_RUNTIME_LOG_CAPACITY ||
        (final_record == 0u && count >= usable_normal_count)) {
        publish_dropped_sample();
        return 0u;
    }

    make_record(&record, sample, event_flags);
    memcpy(&car_runtime_log_storage.records[count], &record, sizeof(record));
    LOG_DMB();
    clean_committed_range(&car_runtime_log_storage.records[count],
                          sizeof(record));

    record_crc_state = crc32_update(record_crc_state,
                                    (const uint8_t *)&record,
                                    sizeof(record));
    next = car_runtime_log_storage.header;
    next.records_crc32 = record_crc_state ^ 0xFFFFFFFFu;
    next.count = count + 1u;
    if (next.count >= CAR_RUNTIME_LOG_CAPACITY - 1u) {
        next.flags |= CAR_RUNTIME_LOG_FLAG_FULL;
    }
    prepare_header(&next);
    publish_header(&next);
    return 1u;
}

static uint8_t sample_is_active(const car_runtime_log_sample_t *sample)
{
    if (active_session_kind == CAR_RUNTIME_LOG_SESSION_FOLLOW) {
        return sample->manual_run_request != 0u &&
               sample->hard_fault_latched == 0u ? 1u : 0u;
    }
    if (active_session_kind == CAR_RUNTIME_LOG_SESSION_BENCH) {
        if (sample->hard_fault_latched != 0u ||
            sample->mode == (uint8_t)CAR_MODE_BENCH_COMPLETE) {
            return 0u;
        }
        return sample->mode == (uint8_t)CAR_MODE_BENCH_RUNNING ? 1u : 0u;
    }
    return 0u;
}

static uint8_t stopped_outputs_are_zero(
    const car_runtime_log_sample_t *sample)
{
    return (uint8_t)(sample->servo_applied_us == 0u &&
                     sample->left_applied_permille == 0u &&
                     sample->right_applied_permille == 0u &&
                     sample->motor_ccr1_counts == 0u &&
                     sample->motor_ccr2_counts == 0u &&
                     sample->motor_ccr3_counts == 0u &&
                     sample->motor_ccr4_counts == 0u);
}

static uint8_t get_stop_reason(const car_runtime_log_sample_t *sample)
{
    if (sample->hard_fault_latched != 0u) {
        return CAR_RUNTIME_LOG_STOP_HARD_FAULT;
    }
    if (sample->mode == (uint8_t)CAR_MODE_BENCH_COMPLETE) {
        return CAR_RUNTIME_LOG_STOP_BENCH_COMPLETE;
    }
    if (sample->fault == (uint8_t)CAR_FAULT_SWITCH_RELEASE) {
        return CAR_RUNTIME_LOG_STOP_MANUAL_RELEASE;
    }
    if (active_session_kind == CAR_RUNTIME_LOG_SESSION_FOLLOW &&
        sample->manual_run_request == 0u) {
        return CAR_RUNTIME_LOG_STOP_MANUAL_RELEASE;
    }
    return CAR_RUNTIME_LOG_STOP_SESSION_ENDED;
}

static uint16_t changed_event_flags(const car_runtime_log_sample_t *sample)
{
    uint16_t events = 0u;

    if (observed_state_valid == 0u) {
        if (sample->fault != (uint8_t)CAR_FAULT_NONE ||
            sample->hard_fault_latched != 0u ||
            sample->fault_generation != 0u) {
            events |= CAR_RUNTIME_LOG_EVENT_FAULT;
        }
        observed_state_valid = 1u;
    } else {
        if (sample->fault_generation != observed_fault_generation ||
            sample->fault != observed_fault ||
            sample->hard_fault_latched != 0u) {
            events |= CAR_RUNTIME_LOG_EVENT_FAULT;
        }
        if (sample->mode != observed_mode) {
            events |= CAR_RUNTIME_LOG_EVENT_MODE;
        }
        if (sample->recovery_active != observed_recovery_active ||
            sample->recovery_frame_count != observed_recovery_frame_count) {
            events |= CAR_RUNTIME_LOG_EVENT_RECOVERY;
        }
        if (sample->auto_recovery_count != observed_auto_recovery_count) {
            events |= CAR_RUNTIME_LOG_EVENT_AUTO_RECOVERY;
        }
        if (sample->manual_run_request != observed_manual_run_request) {
            events |= sample->manual_run_request != 0u
                          ? CAR_RUNTIME_LOG_EVENT_BUTTON_START
                          : CAR_RUNTIME_LOG_EVENT_BUTTON_STOP;
        }
        if (sample->vision_hold_active != observed_vision_hold_active) {
            if (sample->vision_hold_active != 0u) {
                events |= CAR_RUNTIME_LOG_EVENT_HOLD_ENTER;
            } else {
                events |= CAR_RUNTIME_LOG_EVENT_HOLD_EXIT;
                events |= (uint16_t)
                    (((uint16_t)sample->vision_hold_exit_cause & 7u)
                     << CAR_RUNTIME_LOG_EVENT_HOLD_CAUSE_SHIFT);
            }
        }
        if (sample->vision_hold_expired != observed_vision_hold_expired &&
            sample->vision_hold_expired != 0u) {
            events |= CAR_RUNTIME_LOG_EVENT_HOLD_DEADLINE;
        }
    }

    observed_fault_generation = sample->fault_generation;
    observed_fault = sample->fault;
    observed_mode = sample->mode;
    observed_recovery_active = sample->recovery_active;
    observed_recovery_frame_count = sample->recovery_frame_count;
    observed_manual_run_request = sample->manual_run_request;
    observed_vision_hold_active = sample->vision_hold_active;
    observed_vision_hold_expired = sample->vision_hold_expired;
    observed_auto_recovery_count = sample->auto_recovery_count;
    return events;
}

static void freeze_session(const car_runtime_log_sample_t *sample)
{
    car_runtime_log_header_t next = car_runtime_log_storage.header;

    next.state = CAR_RUNTIME_LOG_STATE_FROZEN;
    next.stop_ms = sample->now_ms;
    next.stop_reason = pending_stop_reason;
    prepare_header(&next);
    publish_header(&next);
    active_session_kind = CAR_RUNTIME_LOG_SESSION_NONE;
    stop_pending = 0u;
    pending_event_flags = 0u;
}

void car_runtime_log_update(const car_runtime_log_sample_t *sample,
                            car_runtime_log_session_kind_t start_kind)
{
    uint16_t events;
    uint8_t still_active;
    uint8_t is_periodic;

    if (sample == 0) {
        return;
    }
    if (recording_enabled == 0u) {
        return;
    }

    if (car_runtime_log_storage.header.state != CAR_RUNTIME_LOG_STATE_ACTIVE &&
        (start_kind == CAR_RUNTIME_LOG_SESSION_FOLLOW ||
         start_kind == CAR_RUNTIME_LOG_SESSION_BENCH)) {
        begin_session(sample->now_ms, start_kind);
        events = (uint16_t)(changed_event_flags(sample) |
                            sample->application_event_flags |
                            CAR_RUNTIME_LOG_EVENT_START);
        (void)append_record(sample, events, 0u);
        last_periodic_ms = sample->now_ms;
        if (sample_is_active(sample) == 0u) {
            stop_pending = 1u;
            pending_stop_reason = get_stop_reason(sample);
            pending_event_flags = CAR_RUNTIME_LOG_EVENT_STOP;
            if (stopped_outputs_are_zero(sample) != 0u) {
                if (append_record(sample, pending_event_flags, 1u) != 0u) {
                    freeze_session(sample);
                }
            }
        }
        return;
    }

    if (car_runtime_log_storage.header.state != CAR_RUNTIME_LOG_STATE_ACTIVE) {
        return;
    }

    events = (uint16_t)(changed_event_flags(sample) |
                        sample->application_event_flags);
    still_active = sample_is_active(sample);
    if (still_active == 0u) {
        if (stop_pending == 0u) {
            stop_pending = 1u;
            pending_stop_reason = get_stop_reason(sample);
        }
        pending_event_flags |= events;
        pending_event_flags |= CAR_RUNTIME_LOG_EVENT_STOP;
        if (stopped_outputs_are_zero(sample) != 0u) {
            if (append_record(sample, pending_event_flags, 1u) != 0u) {
                freeze_session(sample);
            }
        }
        return;
    }

    if (stop_pending != 0u) {
        pending_event_flags |= events;
        pending_event_flags |= CAR_RUNTIME_LOG_EVENT_STOP;
        if (stopped_outputs_are_zero(sample) != 0u) {
            if (append_record(sample, pending_event_flags, 1u) != 0u) {
                freeze_session(sample);
            }
        }
        return;
    }

    is_periodic = (uint8_t)((uint32_t)(sample->now_ms - last_periodic_ms) >=
                            CAR_RUNTIME_LOG_PERIOD_MS);
    if (is_periodic != 0u) {
        events |= CAR_RUNTIME_LOG_EVENT_PERIODIC;
        last_periodic_ms = sample->now_ms;
    }
    if (events != 0u) {
        (void)append_record(sample, events, 0u);
    }
}

