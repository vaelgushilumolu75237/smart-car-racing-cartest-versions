#include "receive_diag.h"

#include <limits.h>
#include <string.h>

#if !defined(CAR_RECEIVE_DIAG_HOST_TEST)
#include "stm32h7xx_hal.h"
#endif

#if defined(CAR_RECEIVE_DIAG_HOST_TEST)
#if defined(_MSC_VER)
__declspec(align(32)) car_receive_diag_snapshot_t g_receive_diag_snapshot;
#else
car_receive_diag_snapshot_t g_receive_diag_snapshot
    __attribute__((aligned(32), used));
#endif
#define RX_DIAG_DMB() ((void)0)
#define RX_DIAG_DSB() ((void)0)
#else
car_receive_diag_snapshot_t g_receive_diag_snapshot
    __attribute__((section(".receive_diag"), aligned(32), used));
#define RX_DIAG_DMB() __DMB()
#define RX_DIAG_DSB() __DSB()
#endif

volatile uint32_t car_rx_frame_rate_millifps;
volatile uint32_t car_rx_invalid_observation_permille;
volatile uint32_t car_rx_frame_rate_valid;

static uint32_t generation_state;
static uint32_t window_start_ms;
static uint32_t accepted_total;
static uint32_t accepted_window;
static uint32_t invalid_observation_total;
static uint32_t invalid_observation_window;
static uint32_t missing_sequence_total;
static uint32_t missing_sequence_window;
static uint32_t sequence_reject_total;
static uint32_t sequence_reject_window;
static uint32_t parser_error_total;
static uint32_t last_sequence_ms;
static uint16_t latest_sequence;
static uint8_t have_sequence;
static uint8_t window_valid;

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

uint32_t car_receive_diag_crc32(const void *bytes, uint32_t length)
{
    if (bytes == 0 && length != 0u) {
        return 0u;
    }
    return crc32_update(0xFFFFFFFFu, (const uint8_t *)bytes, length) ^
           0xFFFFFFFFu;
}

uint32_t car_receive_diag_snapshot_crc32(
    const car_receive_diag_snapshot_t *snapshot)
{
    car_receive_diag_snapshot_t copy;

    if (snapshot == 0) {
        return 0u;
    }
    memcpy(&copy, snapshot, sizeof(copy));
    copy.crc32 = 0u;
    return car_receive_diag_crc32(&copy, (uint32_t)sizeof(copy));
}

static uint32_t next_even_generation(uint32_t generation)
{
    if ((generation & 1u) != 0u) {
        generation &= ~1u;
    }
    generation += 2u;
    return generation == 0u ? 2u : generation;
}

#if defined(CAR_RECEIVE_DIAG_HOST_TEST)
static void clean_range(const void *address, uint32_t length)
{
    (void)address;
    (void)length;
    RX_DIAG_DSB();
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
    RX_DIAG_DSB();
}
#endif

static void publish_snapshot(uint32_t now_ms, uint32_t duration_ms,
                             uint32_t window_accepted,
                             uint32_t window_invalid,
                             uint32_t window_missing,
                             uint32_t window_rejected)
{
    car_receive_diag_snapshot_t next = {0};
    volatile uint8_t *destination = (volatile uint8_t *)&g_receive_diag_snapshot;
    const uint8_t *source = (const uint8_t *)&next;
    uint32_t index;

    next.magic = CAR_RECEIVE_DIAG_MAGIC;
    next.version = CAR_RECEIVE_DIAG_VERSION;
    next.size = CAR_RECEIVE_DIAG_SIZE;
    next.generation = next_even_generation(generation_state);
    next.snapshot_ms = now_ms;
    next.window_start_ms = window_start_ms;
    next.window_duration_ms = duration_ms;
    next.accepted_total = accepted_total;
    next.accepted_window = window_accepted;
    next.invalid_observation_total = invalid_observation_total;
    next.invalid_observation_window = window_invalid;
    next.missing_sequence_total = missing_sequence_total;
    next.missing_sequence_window = window_missing;
    next.sequence_reject_total = sequence_reject_total;
    next.sequence_reject_window = window_rejected;
    next.parser_error_total = parser_error_total;
    next.latest_sequence = latest_sequence;
    next.flags = (have_sequence != 0u
                      ? CAR_RECEIVE_DIAG_FLAG_HAVE_SEQUENCE : 0u) |
                 (window_valid != 0u
                      ? CAR_RECEIVE_DIAG_FLAG_WINDOW_VALID : 0u);
    next.crc32 = car_receive_diag_snapshot_crc32(&next);

    g_receive_diag_snapshot.generation = next.generation - 1u;
    clean_range(&g_receive_diag_snapshot.generation,
                (uint32_t)sizeof(g_receive_diag_snapshot.generation));
    RX_DIAG_DMB();
    for (index = 0u; index < (uint32_t)sizeof(next); ++index) {
        if (index < offsetof(car_receive_diag_snapshot_t, generation) ||
            index >= offsetof(car_receive_diag_snapshot_t, generation) +
                         sizeof(next.generation)) {
            destination[index] = source[index];
        }
    }
    RX_DIAG_DMB();
    clean_range(&g_receive_diag_snapshot,
                (uint32_t)sizeof(g_receive_diag_snapshot));
    RX_DIAG_DSB();
    g_receive_diag_snapshot.generation = next.generation;
    clean_range(&g_receive_diag_snapshot.generation,
                (uint32_t)sizeof(g_receive_diag_snapshot.generation));
    RX_DIAG_DSB();
    generation_state = next.generation;
}

static uint32_t saturating_add(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

void car_receive_diag_init(uint32_t now_ms)
{
    car_rx_frame_rate_millifps = 0u;
    car_rx_invalid_observation_permille = 0u;
    car_rx_frame_rate_valid = 0u;
    generation_state = 0u;
    window_start_ms = now_ms;
    accepted_total = 0u;
    accepted_window = 0u;
    invalid_observation_total = 0u;
    invalid_observation_window = 0u;
    missing_sequence_total = 0u;
    missing_sequence_window = 0u;
    sequence_reject_total = 0u;
    sequence_reject_window = 0u;
    parser_error_total = 0u;
    last_sequence_ms = now_ms;
    latest_sequence = 0u;
    have_sequence = 0u;
    window_valid = 0u;
    memset(&g_receive_diag_snapshot, 0, sizeof(g_receive_diag_snapshot));
    clean_range(&g_receive_diag_snapshot,
                (uint32_t)sizeof(g_receive_diag_snapshot));
    publish_snapshot(now_ms, 0u, 0u, 0u, 0u, 0u);
}

void car_receive_diag_note_frame(uint16_t sequence,
                                 uint8_t valid_observation,
                                 uint32_t arrival_ms)
{
    accepted_total = saturating_add(accepted_total, 1u);
    accepted_window = saturating_add(accepted_window, 1u);
    if (valid_observation == 0u) {
        invalid_observation_total =
            saturating_add(invalid_observation_total, 1u);
        invalid_observation_window =
            saturating_add(invalid_observation_window, 1u);
    }
    /* A two-second receive outage starts a new sequence epoch. This avoids
       calling a camera reboot a huge packet-loss burst. Sequence resets that
       happen during a shorter outage remain ambiguous and use modulo deltas. */
    if (have_sequence != 0u &&
        (uint32_t)(arrival_ms - last_sequence_ms) >=
            (2u * CAR_RECEIVE_DIAG_WINDOW_MS)) {
        have_sequence = 0u;
    }
    if (have_sequence != 0u) {
        const uint16_t delta = (uint16_t)(sequence - latest_sequence);
        if (delta == 0u || delta >= 0x8000u) {
            sequence_reject_total = saturating_add(sequence_reject_total, 1u);
            sequence_reject_window =
                saturating_add(sequence_reject_window, 1u);
        } else if (delta > 1u) {
            const uint32_t missing = (uint32_t)delta - 1u;
            missing_sequence_total =
                saturating_add(missing_sequence_total, missing);
            missing_sequence_window =
                saturating_add(missing_sequence_window, missing);
        }
    }
    if (have_sequence == 0u ||
        (uint16_t)(sequence - latest_sequence) < 0x8000u) {
        latest_sequence = sequence;
        have_sequence = 1u;
    }
    last_sequence_ms = arrival_ms;
}

void car_receive_diag_note_parser_error(uint32_t now_ms)
{
    (void)now_ms;
    parser_error_total = saturating_add(parser_error_total, 1u);
}

void car_receive_diag_publish(uint32_t now_ms)
{
    const uint32_t elapsed_ms = (uint32_t)(now_ms - window_start_ms);

    if (elapsed_ms < CAR_RECEIVE_DIAG_WINDOW_MS) {
        return;
    }

    window_valid = 1u;
    if (elapsed_ms != 0u) {
        const uint64_t rate =
            ((uint64_t)accepted_window * 1000000u) / elapsed_ms;
        car_rx_frame_rate_millifps = rate > UINT32_MAX
            ? UINT32_MAX : (uint32_t)rate;
    } else {
        car_rx_frame_rate_millifps = 0u;
    }
    car_rx_invalid_observation_permille = accepted_window == 0u
        ? 0u
        : (uint32_t)(((uint64_t)invalid_observation_window * 1000u) /
                     accepted_window);
    car_rx_frame_rate_valid = 1u;
    publish_snapshot(now_ms, elapsed_ms, accepted_window,
                     invalid_observation_window, missing_sequence_window,
                     sequence_reject_window);
    window_start_ms = now_ms;
    accepted_window = 0u;
    invalid_observation_window = 0u;
    missing_sequence_window = 0u;
    sequence_reject_window = 0u;
}
