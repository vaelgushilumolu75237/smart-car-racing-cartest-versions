#include "runtime_diag.h"

#include <limits.h>
#include <string.h>

#if !defined(CAR_RUNTIME_DIAG_HOST_TEST)
#include "stm32h7xx_hal.h"
#endif

#if defined(CAR_RUNTIME_DIAG_HOST_TEST)
#if defined(_MSC_VER)
__declspec(align(32)) car_runtime_diag_snapshot_t g_runtime_diag_snapshot;
#else
car_runtime_diag_snapshot_t g_runtime_diag_snapshot
    __attribute__((aligned(32), used));
#endif
#define DIAG_DMB() ((void)0)
#define DIAG_DSB() ((void)0)
#else
car_runtime_diag_snapshot_t g_runtime_diag_snapshot
    __attribute__((section(".runtime_diag"), aligned(32), used));
#define DIAG_DMB() __DMB()
#define DIAG_DSB() __DSB()
#endif

static uint32_t generation_state;
static uint32_t log_update_calls;
static uint32_t last_publish_ms;
static car_runtime_diag_values_t last_values;
static uint8_t have_published;

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

uint32_t car_runtime_diag_crc32(const void *bytes, uint32_t length)
{
    if (bytes == 0 && length != 0u) {
        return 0u;
    }
    return crc32_update(0xFFFFFFFFu, (const uint8_t *)bytes, length) ^
           0xFFFFFFFFu;
}

uint32_t car_runtime_diag_snapshot_crc32(
    const car_runtime_diag_snapshot_t *snapshot)
{
    car_runtime_diag_snapshot_t copy;

    if (snapshot == 0) {
        return 0u;
    }
    memcpy(&copy, snapshot, sizeof(copy));
    copy.crc32 = 0u;
    return car_runtime_diag_crc32(&copy, sizeof(copy));
}

uint32_t car_runtime_diag_next_even_generation(uint32_t stable_generation)
{
    uint32_t next;

    if ((stable_generation & 1u) != 0u) {
        stable_generation &= ~1u;
    }
    next = stable_generation + 2u;
    if (next == 0u) {
        next = 2u;
    }
    return next;
}

#if defined(CAR_RUNTIME_DIAG_HOST_TEST)
static void clean_committed_range(const void *address, uint32_t length)
{
    (void)address;
    (void)length;
    DIAG_DSB();
}
#else
static void clean_committed_range(const void *address, uint32_t length)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u && length != 0u) {
        const uintptr_t mask = 31u;
        const uintptr_t start = (uintptr_t)address & ~mask;
        const uintptr_t end = ((uintptr_t)address + length + mask) & ~mask;
        SCB_CleanDCache_by_Addr((uint32_t *)start,
                               (int32_t)(end - start));
    }
    DIAG_DSB();
}
#endif

static int16_t output_to_i16(uint16_t output_permille)
{
    if (output_permille > (uint16_t)INT16_MAX) {
        return INT16_MAX;
    }
    return (int16_t)output_permille;
}

static uint8_t values_changed(const car_runtime_diag_values_t *values)
{
    return (uint8_t)(have_published == 0u ||
                     values->start_events != last_values.start_events ||
                     values->stop_events != last_values.stop_events ||
                     values->start_token != last_values.start_token ||
                     values->cpu_header_state !=
                         last_values.cpu_header_state ||
                     values->cpu_header_count !=
                         last_values.cpu_header_count ||
                     values->header_session_id !=
                         last_values.header_session_id ||
                     values->log_enabled != last_values.log_enabled);
}

static void publish_snapshot(const car_runtime_diag_snapshot_t *desired)
{
    volatile uint8_t *destination =
        (volatile uint8_t *)&g_runtime_diag_snapshot;
    const uint8_t *source = (const uint8_t *)desired;
    const uint32_t next_generation = desired->generation;
    const uint32_t odd_generation = next_generation - 1u;
    uint32_t i;

    /* Make the in-progress marker visible before touching any payload byte. */
    g_runtime_diag_snapshot.generation = odd_generation;
    clean_committed_range(&g_runtime_diag_snapshot.generation,
                          sizeof(g_runtime_diag_snapshot.generation));
    DIAG_DMB();

    /* Copy all fields except generation. CRC already covers the intended final
       even generation, so a torn/stale generation cannot validate. */
    for (i = 0u; i < sizeof(*desired); ++i) {
        if (i < offsetof(car_runtime_diag_snapshot_t, generation) ||
            i >= offsetof(car_runtime_diag_snapshot_t, generation) +
                     sizeof(desired->generation)) {
            destination[i] = source[i];
        }
    }
    DIAG_DMB();
    clean_committed_range(&g_runtime_diag_snapshot,
                          sizeof(g_runtime_diag_snapshot));
    DIAG_DSB();

    /* Publish the committed sequence only after the complete CRC image is in
       SRAM and visible beyond the H743 data cache. */
    g_runtime_diag_snapshot.generation = next_generation;
    clean_committed_range(&g_runtime_diag_snapshot.generation,
                          sizeof(g_runtime_diag_snapshot.generation));
    DIAG_DSB();
    generation_state = next_generation;
}

void car_runtime_diag_init(uint32_t reset_flags, uint8_t log_enabled)
{
    car_runtime_diag_values_t initial = {0};

    generation_state = 0u;
    log_update_calls = 0u;
    last_publish_ms = 0u;
    last_values = (car_runtime_diag_values_t){0};
    have_published = 0u;
    memset(&g_runtime_diag_snapshot, 0, sizeof(g_runtime_diag_snapshot));
    clean_committed_range(&g_runtime_diag_snapshot,
                          sizeof(g_runtime_diag_snapshot));
    initial.reset_flags = reset_flags;
    initial.log_enabled = log_enabled != 0u ? 1u : 0u;
    (void)car_runtime_diag_update(&initial);
}

void car_runtime_diag_note_log_update_call(void)
{
    ++log_update_calls;
}

uint8_t car_runtime_diag_update(const car_runtime_diag_values_t *values)
{
    car_runtime_diag_snapshot_t next = {0};
    uint8_t changed;

    if (values == 0) {
        return 0u;
    }
    changed = values_changed(values);
    if (changed == 0u && have_published != 0u &&
        (uint32_t)(values->uptime_ms - last_publish_ms) <
            CAR_RUNTIME_DIAG_PERIOD_MS) {
        return 0u;
    }

    next.magic = CAR_RUNTIME_DIAG_MAGIC;
    next.version = CAR_RUNTIME_DIAG_VERSION;
    next.size = CAR_RUNTIME_DIAG_SIZE;
    next.generation =
        car_runtime_diag_next_even_generation(generation_state);
    next.start_events = values->start_events;
    next.stop_events = values->stop_events;
    next.log_update_calls = log_update_calls;
    next.start_token = values->start_token;
    next.cpu_header_state = values->cpu_header_state;
    next.cpu_header_count = values->cpu_header_count;
    next.uptime_ms = values->uptime_ms;
    next.reset_flags = values->reset_flags;
    next.left_output_permille = output_to_i16(
        values->left_output_permille);
    next.right_output_permille = output_to_i16(
        values->right_output_permille);
    next.servo_pulse_us = values->servo_pulse_us;
    next.flags = values->log_enabled != 0u
        ? CAR_RUNTIME_DIAG_FLAG_LOG_ENABLED : 0u;
    next.header_session_id = values->header_session_id;
    next.reserved = 0u;
    next.crc32 = car_runtime_diag_snapshot_crc32(&next);

    publish_snapshot(&next);
    last_values = *values;
    last_publish_ms = values->uptime_ms;
    have_published = 1u;
    return 1u;
}
