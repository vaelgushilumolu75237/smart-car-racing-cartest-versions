#include "speed_pi_diag.h"

#include <limits.h>
#include <string.h>

#if !defined(CAR_SPEED_PI_DIAG_HOST_TEST)
#include "stm32h7xx_hal.h"
#endif

#if defined(CAR_SPEED_PI_DIAG_HOST_TEST)
#if defined(_MSC_VER)
__declspec(align(32)) car_speed_pi_diag_snapshot_t g_speed_pi_diag_snapshot;
#else
car_speed_pi_diag_snapshot_t g_speed_pi_diag_snapshot
    __attribute__((aligned(32), used));
#endif
#define PI_DIAG_DMB() ((void)0)
#define PI_DIAG_DSB() ((void)0)
#else
car_speed_pi_diag_snapshot_t g_speed_pi_diag_snapshot
    __attribute__((section(".speed_pi_diag"), aligned(32), used));
#define PI_DIAG_DMB() __DMB()
#define PI_DIAG_DSB() __DSB()
#endif

static uint32_t generation_state;
static uint8_t session_active;
static uint8_t initialized;

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

uint32_t car_speed_pi_diag_crc32(const void *bytes, uint32_t length)
{
    if (bytes == 0 && length != 0u) {
        return 0u;
    }
    return crc32_update(0xFFFFFFFFu, (const uint8_t *)bytes, length) ^
           0xFFFFFFFFu;
}

uint32_t car_speed_pi_diag_snapshot_crc32(
    const car_speed_pi_diag_snapshot_t *snapshot)
{
    car_speed_pi_diag_snapshot_t copy;

    if (snapshot == 0) {
        return 0u;
    }
    memcpy(&copy, snapshot, sizeof(copy));
    copy.crc32 = 0u;
    return car_speed_pi_diag_crc32((const uint8_t *)&copy,
                                   (uint32_t)sizeof(copy));
}

uint32_t car_speed_pi_diag_next_even_generation(uint32_t stable_generation)
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

#if defined(CAR_SPEED_PI_DIAG_HOST_TEST)
static void clean_committed_range(const void *address, uint32_t length)
{
    (void)address;
    (void)length;
    PI_DIAG_DSB();
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
    PI_DIAG_DSB();
}
#endif

static void publish_snapshot(const car_speed_pi_diag_snapshot_t *desired)
{
    volatile uint8_t *destination =
        (volatile uint8_t *)&g_speed_pi_diag_snapshot;
    const uint8_t *source = (const uint8_t *)desired;
    const uint32_t next_generation = desired->generation;
    const uint32_t odd_generation = next_generation - 1u;
    uint32_t i;

    /* Expose the in-progress marker before changing any payload bytes. */
    g_speed_pi_diag_snapshot.generation = odd_generation;
    clean_committed_range(&g_speed_pi_diag_snapshot.generation,
                          (uint32_t)sizeof(g_speed_pi_diag_snapshot.generation));
    PI_DIAG_DMB();

    /* The CRC covers the final even generation. Leave its live field odd until
       the whole CRC image has reached SRAM beyond the H743 data cache. */
    for (i = 0u; i < (uint32_t)sizeof(*desired); ++i) {
        if (i < offsetof(car_speed_pi_diag_snapshot_t, generation) ||
            i >= offsetof(car_speed_pi_diag_snapshot_t, generation) +
                     sizeof(desired->generation)) {
            destination[i] = source[i];
        }
    }
    PI_DIAG_DMB();
    clean_committed_range(&g_speed_pi_diag_snapshot,
                          (uint32_t)sizeof(g_speed_pi_diag_snapshot));
    PI_DIAG_DSB();

    g_speed_pi_diag_snapshot.generation = next_generation;
    clean_committed_range(&g_speed_pi_diag_snapshot.generation,
                          (uint32_t)sizeof(g_speed_pi_diag_snapshot.generation));
    PI_DIAG_DSB();
    generation_state = next_generation;
}

static void publish_current(uint32_t now_ms)
{
    car_speed_pi_diag_snapshot_t next;

    memcpy(&next, &g_speed_pi_diag_snapshot, sizeof(next));
    next.magic = CAR_SPEED_PI_DIAG_MAGIC;
    next.version = CAR_SPEED_PI_DIAG_VERSION;
    next.size = CAR_SPEED_PI_DIAG_SIZE;
    next.generation =
        car_speed_pi_diag_next_even_generation(generation_state);
    next.snapshot_ms = now_ms;
    memset(next.reserved, 0, sizeof(next.reserved));
    next.reserved[0] = CAR_SPEED_PI_DIAG_OUTPUT_CAP_PERMILLE;
    next.crc32 = 0u;
    next.crc32 = car_speed_pi_diag_snapshot_crc32(&next);
    publish_snapshot(&next);
}

void car_speed_pi_diag_init(void)
{
    memset(&g_speed_pi_diag_snapshot, 0,
           sizeof(g_speed_pi_diag_snapshot));
    generation_state = 0u;
    session_active = 0u;
    initialized = 1u;
    clean_committed_range(&g_speed_pi_diag_snapshot,
                          (uint32_t)sizeof(g_speed_pi_diag_snapshot));
    publish_current(0u);
}

void car_speed_pi_diag_begin(uint32_t start_token, uint32_t start_ms,
                             uint32_t pi_type)
{
    car_speed_pi_diag_snapshot_t next;

    if (initialized == 0u) {
        car_speed_pi_diag_init();
    }
    memset(&next, 0, sizeof(next));
    next.magic = CAR_SPEED_PI_DIAG_MAGIC;
    next.version = CAR_SPEED_PI_DIAG_VERSION;
    next.size = CAR_SPEED_PI_DIAG_SIZE;
    next.generation =
        car_speed_pi_diag_next_even_generation(generation_state);
    next.start_token = start_token;
    next.start_ms = start_ms;
    next.snapshot_ms = start_ms;
    next.flags = CAR_SPEED_PI_DIAG_FLAG_ACTIVE;
    next.pi_type = pi_type;
    next.reserved[0] = CAR_SPEED_PI_DIAG_OUTPUT_CAP_PERMILLE;
    next.crc32 = car_speed_pi_diag_snapshot_crc32(&next);

    session_active = 1u;
    publish_snapshot(&next);
}

static uint32_t add_saturation_elapsed(uint32_t total, uint32_t elapsed)
{
    if (UINT32_MAX - total < elapsed) {
        return UINT32_MAX;
    }
    return total + elapsed;
}

uint8_t car_speed_pi_diag_update(const car_speed_pi_diag_values_t *values)
{
    car_speed_pi_diag_snapshot_t next;
    uint8_t distinct_sample = 0u;
    uint8_t changed = 0u;
    uint32_t desired_flags;

    if (values == 0 || initialized == 0u || session_active == 0u ||
        (g_speed_pi_diag_snapshot.flags & CAR_SPEED_PI_DIAG_FLAG_FROZEN) != 0u) {
        return 0u;
    }

    memcpy(&next, &g_speed_pi_diag_snapshot, sizeof(next));
    desired_flags = next.flags &
        ~(CAR_SPEED_PI_DIAG_FLAG_ENABLED |
          CAR_SPEED_PI_DIAG_FLAG_ARMED |
          CAR_SPEED_PI_DIAG_FLAG_HOLD_ACTIVE);
    if (values->enabled != 0u) {
        desired_flags |= CAR_SPEED_PI_DIAG_FLAG_ENABLED;
    }
    if (values->armed != 0u) {
        desired_flags |= CAR_SPEED_PI_DIAG_FLAG_ARMED;
    }
    if (values->hold_active != 0u) {
        desired_flags |= CAR_SPEED_PI_DIAG_FLAG_HOLD_ACTIVE;
    }

    if (values->new_speed_sample != 0u &&
        values->speed_sample_valid != 0u &&
        (next.flags & CAR_SPEED_PI_DIAG_FLAG_LAST_SAMPLE_VALID) == 0u) {
        distinct_sample = 1u;
    } else if (values->new_speed_sample != 0u &&
               values->speed_sample_valid != 0u &&
               values->sample_ms != next.sample_ms) {
        distinct_sample = 1u;
    }

    if (desired_flags != next.flags) {
        next.flags = desired_flags;
        changed = 1u;
    }
    if (next.status != values->status) {
        next.status = values->status;
        changed = 1u;
    }
    if (next.applied_left_permille != values->applied_left_permille ||
        next.applied_right_permille != values->applied_right_permille) {
        next.applied_left_permille = values->applied_left_permille;
        next.applied_right_permille = values->applied_right_permille;
        changed = 1u;
    }

    if (distinct_sample != 0u) {
        next.sample_ms = values->sample_ms;
        next.measured_left_millirps = values->measured_left_millirps;
        next.measured_right_millirps = values->measured_right_millirps;
        next.flags |= CAR_SPEED_PI_DIAG_FLAG_LAST_SAMPLE_VALID;
        changed = 1u;

        if (values->enabled != 0u && values->armed != 0u &&
            values->hold_active == 0u && values->pi_proposal_valid != 0u) {
            next.target_left_millirps = values->target_left_millirps;
            next.target_right_millirps = values->target_right_millirps;
            next.nominal_ff_left_permille = values->nominal_ff_left_permille;
            next.nominal_ff_right_permille = values->nominal_ff_right_permille;
            next.requested_left_permille = values->requested_left_permille;
            next.requested_right_permille = values->requested_right_permille;
        }

        {
            const uint8_t count_upper_left =
                values->enabled != 0u && values->armed != 0u &&
                values->hold_active == 0u &&
                values->pi_proposal_valid != 0u &&
                values->saturated_left != 0u &&
                values->requested_left_permille >=
                    next.reserved[0];
            const uint8_t count_upper_right =
                values->enabled != 0u && values->armed != 0u &&
                values->hold_active == 0u &&
                values->pi_proposal_valid != 0u &&
                values->saturated_right != 0u &&
                values->requested_right_permille >=
                    next.reserved[0];

            if (count_upper_left != 0u) {
                next.saturation_left_ms = add_saturation_elapsed(
                    next.saturation_left_ms, values->sample_elapsed_ms);
            }
            if (count_upper_right != 0u) {
                next.saturation_right_ms = add_saturation_elapsed(
                    next.saturation_right_ms, values->sample_elapsed_ms);
            }
        }
    }

    if (changed == 0u) {
        return 0u;
    }
    next.generation =
        car_speed_pi_diag_next_even_generation(generation_state);
    next.snapshot_ms = values->now_ms;
    next.crc32 = 0u;
    next.crc32 = car_speed_pi_diag_snapshot_crc32(&next);
    publish_snapshot(&next);
    return 1u;
}

void car_speed_pi_diag_freeze(uint32_t now_ms, uint32_t termination_fault)
{
    car_speed_pi_diag_snapshot_t next;

    if (initialized == 0u || session_active == 0u ||
        (g_speed_pi_diag_snapshot.flags & CAR_SPEED_PI_DIAG_FLAG_FROZEN) != 0u) {
        return;
    }
    memcpy(&next, &g_speed_pi_diag_snapshot, sizeof(next));
    next.generation =
        car_speed_pi_diag_next_even_generation(generation_state);
    next.snapshot_ms = now_ms;
    next.termination_fault = termination_fault;
    next.flags &= ~CAR_SPEED_PI_DIAG_FLAG_ACTIVE;
    next.flags |= CAR_SPEED_PI_DIAG_FLAG_FROZEN;
    next.crc32 = 0u;
    next.crc32 = car_speed_pi_diag_snapshot_crc32(&next);
    session_active = 0u;
    publish_snapshot(&next);
}
