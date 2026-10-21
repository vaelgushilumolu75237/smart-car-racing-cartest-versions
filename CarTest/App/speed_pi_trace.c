#include "speed_pi_trace.h"

#include <string.h>

#include "runtime_log_build_id.h"

#if !defined(CAR_SPEED_PI_TRACE_HOST_TEST)
#include "stm32h7xx_hal.h"
#endif

#if defined(CAR_SPEED_PI_TRACE_HOST_TEST)
#if defined(_MSC_VER)
__declspec(align(32)) car_speed_pi_trace_t g_speed_pi_trace;
#else
car_speed_pi_trace_t g_speed_pi_trace
    __attribute__((aligned(32), used));
#endif
#define TRACE_DMB() ((void)0)
#define TRACE_DSB() ((void)0)
#else
car_speed_pi_trace_t g_speed_pi_trace
    __attribute__((section(".speed_pi_trace"), aligned(32), used));
#define TRACE_DMB() __DMB()
#define TRACE_DSB() __DSB()
#endif

static uint32_t generation_state;
static uint32_t total_samples;
static uint16_t next_index;
static uint16_t sample_count;

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

uint32_t car_speed_pi_trace_crc32(const void *bytes, uint32_t length)
{
    if (bytes == 0 && length != 0u) {
        return 0u;
    }
    return crc32_update(0xFFFFFFFFu, (const uint8_t *)bytes, length) ^
           0xFFFFFFFFu;
}

uint32_t car_speed_pi_trace_header_crc32(
    const car_speed_pi_trace_header_t *header)
{
    car_speed_pi_trace_header_t copy;

    if (header == 0) {
        return 0u;
    }
    memcpy(&copy, header, sizeof(copy));
    copy.header_crc32 = 0u;
    return car_speed_pi_trace_crc32(&copy, (uint32_t)sizeof(copy));
}

uint32_t car_speed_pi_trace_sample_crc32(
    const car_speed_pi_trace_sample_t *sample)
{
    car_speed_pi_trace_sample_t copy;

    if (sample == 0) {
        return 0u;
    }
    memcpy(&copy, sample, sizeof(copy));
    copy.crc32 = 0u;
    return car_speed_pi_trace_crc32(&copy, (uint32_t)sizeof(copy));
}

static uint32_t next_even_generation(uint32_t generation)
{
    if ((generation & 1u) != 0u) {
        generation &= ~1u;
    }
    generation += 2u;
    return generation == 0u ? 2u : generation;
}

#if defined(CAR_SPEED_PI_TRACE_HOST_TEST)
static void clean_range(const void *address, uint32_t length)
{
    (void)address;
    (void)length;
    TRACE_DSB();
}
#else
static void clean_range(const void *address, uint32_t length)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u && length != 0u) {
        const uintptr_t mask = 31u;
        const uintptr_t start = (uintptr_t)address & ~mask;
        const uintptr_t end = ((uintptr_t)address + length + mask) & ~mask;
        SCB_CleanDCache_by_Addr((uint32_t *)start,
                               (int32_t)(end - start));
    }
    TRACE_DSB();
}
#endif

static void publish_header(const car_speed_pi_trace_header_t *desired)
{
    volatile uint8_t *destination =
        (volatile uint8_t *)&g_speed_pi_trace.header;
    const uint8_t *source = (const uint8_t *)desired;
    const uint32_t next_generation = desired->generation;
    uint32_t index;

    g_speed_pi_trace.header.generation = next_generation - 1u;
    clean_range(&g_speed_pi_trace.header.generation,
                (uint32_t)sizeof(g_speed_pi_trace.header.generation));
    TRACE_DMB();
    for (index = 0u; index < (uint32_t)sizeof(*desired); ++index) {
        if (index < offsetof(car_speed_pi_trace_header_t, generation) ||
            index >= offsetof(car_speed_pi_trace_header_t, generation) +
                         sizeof(desired->generation)) {
            destination[index] = source[index];
        }
    }
    TRACE_DMB();
    clean_range(&g_speed_pi_trace.header,
                (uint32_t)sizeof(g_speed_pi_trace.header));
    TRACE_DSB();
    g_speed_pi_trace.header.generation = next_generation;
    clean_range(&g_speed_pi_trace.header.generation,
                (uint32_t)sizeof(g_speed_pi_trace.header.generation));
    TRACE_DSB();
    generation_state = next_generation;
}

static void init_header(uint8_t state, uint32_t start_token,
                        uint32_t start_ms, uint32_t stop_ms,
                        uint8_t pi_type, uint32_t records_crc32)
{
    car_speed_pi_trace_header_t next = {0};

    next.magic = CAR_SPEED_PI_TRACE_MAGIC;
    next.version = CAR_SPEED_PI_TRACE_VERSION;
    next.size = CAR_SPEED_PI_TRACE_HEADER_SIZE;
    next.generation = next_even_generation(generation_state);
    next.records_crc32 = records_crc32;
    next.start_token = start_token;
    next.start_ms = start_ms;
    next.stop_ms = stop_ms;
    next.capacity = CAR_SPEED_PI_TRACE_CAPACITY;
    next.count = sample_count;
    next.next_index = next_index;
    next.pi_type = pi_type;
    next.state = state;
    next.total_samples = total_samples;
    next.lost_samples = total_samples > sample_count
        ? total_samples - sample_count : 0u;
    {
        const uint8_t build_id[16] = CAR_RUNTIME_LOG_BUILD_ID_BYTES;
        memcpy(next.build_id, build_id, sizeof(next.build_id));
    }
    next.header_crc32 = car_speed_pi_trace_header_crc32(&next);
    publish_header(&next);
}

void car_speed_pi_trace_init(void)
{
    generation_state = 0u;
    total_samples = 0u;
    next_index = 0u;
    sample_count = 0u;
    memset(&g_speed_pi_trace.header, 0, sizeof(g_speed_pi_trace.header));
    clean_range(&g_speed_pi_trace.header,
                (uint32_t)sizeof(g_speed_pi_trace.header));
    init_header(CAR_SPEED_PI_TRACE_STATE_IDLE, 0u, 0u, 0u, 0u, 0u);
}

void car_speed_pi_trace_begin(uint32_t start_token, uint32_t start_ms,
                              uint8_t pi_type)
{
    total_samples = 0u;
    next_index = 0u;
    sample_count = 0u;
    init_header(CAR_SPEED_PI_TRACE_STATE_ACTIVE, start_token, start_ms,
                0u, pi_type, 0u);
}

void car_speed_pi_trace_append(const car_speed_pi_trace_values_t *values)
{
    car_speed_pi_trace_sample_t sample = {0};
    car_speed_pi_trace_sample_t *destination;

    if (values == 0 ||
        g_speed_pi_trace.header.state != CAR_SPEED_PI_TRACE_STATE_ACTIVE) {
        return;
    }

    sample.sample_index = total_samples;
    sample.record_ms = values->record_ms;
    sample.sample_ms = values->sample_ms;
    sample.vision_sequence = values->vision_sequence;
    sample.vision_arrival_ms = values->vision_arrival_ms;
    sample.pi_type = values->pi_type;
    sample.flags = (values->speed_valid != 0u
                        ? CAR_SPEED_PI_TRACE_FLAG_SPEED_VALID : 0u) |
                   (values->armed != 0u
                        ? CAR_SPEED_PI_TRACE_FLAG_ARMED : 0u) |
                   (values->hold_active != 0u
                        ? CAR_SPEED_PI_TRACE_FLAG_HOLD : 0u) |
                   (values->proposal_valid != 0u
                        ? CAR_SPEED_PI_TRACE_FLAG_PROPOSAL_VALID : 0u);
    sample.target_left_millirps = values->target_left_millirps;
    sample.target_right_millirps = values->target_right_millirps;
    sample.measured_left_millirps = values->measured_left_millirps;
    sample.measured_right_millirps = values->measured_right_millirps;
    sample.requested_left_permille = values->requested_left_permille;
    sample.requested_right_permille = values->requested_right_permille;
    sample.applied_left_permille = values->applied_left_permille;
    sample.applied_right_permille = values->applied_right_permille;
    sample.crc32 = car_speed_pi_trace_sample_crc32(&sample);

    destination = &g_speed_pi_trace.samples[next_index];
    memcpy(destination, &sample, sizeof(sample));
    TRACE_DMB();
    clean_range(destination, (uint32_t)sizeof(*destination));
    TRACE_DSB();

    if (total_samples != UINT32_MAX) {
        ++total_samples;
    }
    next_index = (uint16_t)((next_index + 1u) %
                            CAR_SPEED_PI_TRACE_CAPACITY);
    if (sample_count < CAR_SPEED_PI_TRACE_CAPACITY) {
        ++sample_count;
    }
}

void car_speed_pi_trace_freeze(uint32_t stop_ms)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    const uint16_t first = sample_count == CAR_SPEED_PI_TRACE_CAPACITY
        ? next_index : 0u;
    const uint8_t pi_type = g_speed_pi_trace.header.pi_type;
    const uint32_t start_token = g_speed_pi_trace.header.start_token;
    const uint32_t start_ms = g_speed_pi_trace.header.start_ms;

    if (g_speed_pi_trace.header.state != CAR_SPEED_PI_TRACE_STATE_ACTIVE) {
        return;
    }
    TRACE_DSB();
    clean_range(g_speed_pi_trace.samples,
                (uint32_t)CAR_SPEED_PI_TRACE_CAPACITY *
                    CAR_SPEED_PI_TRACE_SAMPLE_SIZE);
    for (i = 0u; i < sample_count; ++i) {
        const uint16_t index = (uint16_t)(
            (first + i) % CAR_SPEED_PI_TRACE_CAPACITY);
        crc = crc32_update(crc,
            (const uint8_t *)&g_speed_pi_trace.samples[index],
            CAR_SPEED_PI_TRACE_SAMPLE_SIZE);
    }
    init_header(CAR_SPEED_PI_TRACE_STATE_FROZEN, start_token, start_ms,
                stop_ms, pi_type, crc ^ 0xFFFFFFFFu);
}
