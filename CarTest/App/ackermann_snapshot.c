#include "ackermann_snapshot.h"

#include <stddef.h>
#include <stdint.h>

#if !defined(CAR_ACKERMANN_SNAPSHOT_HOST_TEST)
#include "stm32h7xx_hal.h"
#endif

#if defined(CAR_ACKERMANN_SNAPSHOT_HOST_TEST)
#if defined(_MSC_VER)
__declspec(align(32)) volatile uint8_t
    g_ackermann_snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE];
#else
volatile uint8_t g_ackermann_snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE]
    __attribute__((aligned(32), used));
#endif
#else
volatile uint8_t g_ackermann_snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE]
    __attribute__((section(".ackermann_snapshot"), aligned(32), used));
#endif

#if defined(__arm__) || defined(__thumb__)
#define ACKERMANN_DMB() __asm volatile ("dmb" ::: "memory")
#define ACKERMANN_DSB() __asm volatile ("dsb" ::: "memory")
#elif defined(__GNUC__)
#define ACKERMANN_DMB() __asm volatile ("" ::: "memory")
#define ACKERMANN_DSB() __asm volatile ("" ::: "memory")
#elif defined(_MSC_VER)
#include <intrin.h>
#define ACKERMANN_DMB() _ReadWriteBarrier()
#define ACKERMANN_DSB() _ReadWriteBarrier()
#else
#define ACKERMANN_DMB() ((void)0)
#define ACKERMANN_DSB() ((void)0)
#endif

#if defined(CAR_ACKERMANN_SNAPSHOT_HOST_TEST)
static void clean_committed_range(const volatile void *address,
                                  uint32_t length)
{
    (void)address;
    (void)length;
    ACKERMANN_DSB();
}
#else
static void clean_committed_range(const volatile void *address,
                                  uint32_t length)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u && length != 0u) {
        const uintptr_t mask = 31u;
        const uintptr_t start = (uintptr_t)address & ~mask;
        const uintptr_t end = ((uintptr_t)address + length + mask) & ~mask;
        SCB_CleanDCache_by_Addr((uint32_t *)start,
                               (int32_t)(end - start));
    }
    ACKERMANN_DSB();
}
#endif

static uint16_t read_u16(const uint8_t *bytes, uint32_t offset)
{
    return (uint16_t)((uint16_t)bytes[offset] |
                      ((uint16_t)bytes[offset + 1u] << 8));
}

static uint32_t read_u32(const uint8_t *bytes, uint32_t offset)
{
    return (uint32_t)bytes[offset] |
           ((uint32_t)bytes[offset + 1u] << 8) |
           ((uint32_t)bytes[offset + 2u] << 16) |
           ((uint32_t)bytes[offset + 3u] << 24);
}

static uint32_t read_volatile_u32(
    const volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE],
    uint32_t offset)
{
    return (uint32_t)bytes[offset] |
           ((uint32_t)bytes[offset + 1u] << 8) |
           ((uint32_t)bytes[offset + 2u] << 16) |
           ((uint32_t)bytes[offset + 3u] << 24);
}

static void write_volatile_u32(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE],
    uint32_t offset,
    uint32_t value)
{
    bytes[offset] = (uint8_t)value;
    bytes[offset + 1u] = (uint8_t)(value >> 8);
    bytes[offset + 2u] = (uint8_t)(value >> 16);
    bytes[offset + 3u] = (uint8_t)(value >> 24);
}

static void write_volatile_generation(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE],
    uint32_t value)
{
    volatile uint8_t *address = bytes + 8u;
    if (((uintptr_t)address & 3u) == 0u) {
        /* The H743 is little-endian and this aligned volatile word store is
           atomic; the raw ABI remains bytes with generation at offset 8. */
        *((volatile uint32_t *)(uintptr_t)address) = value;
    } else {
        /* Keep the helper useful for unaligned host fixtures. */
        write_volatile_u32(bytes, 8u, value);
    }
}

static void copy_to_volatile(
    volatile uint8_t *destination,
    uint32_t offset,
    const uint8_t *source,
    uint32_t length)
{
    uint32_t index;
    for (index = 0u; index < length; ++index) {
        destination[offset + index] = source[index];
    }
}

static void write_le16(uint8_t *bytes, uint32_t offset, uint16_t value)
{
    bytes[offset] = (uint8_t)value;
    bytes[offset + 1u] = (uint8_t)(value >> 8);
}

static void write_le32(uint8_t *bytes, uint32_t offset, uint32_t value)
{
    bytes[offset] = (uint8_t)value;
    bytes[offset + 1u] = (uint8_t)(value >> 8);
    bytes[offset + 2u] = (uint8_t)(value >> 16);
    bytes[offset + 3u] = (uint8_t)(value >> 24);
}

uint32_t car_ackermann_snapshot_crc32(const uint8_t *bytes,
                                      uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t index;

    if (bytes == 0) {
        return 0u;
    }
    for (index = 0u; index < length; ++index) {
        uint8_t bit;
        crc ^= bytes[index];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1) ^
                ((crc & 1u) != 0u ? 0xEDB88320u : 0u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

void car_ackermann_snapshot_reset(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE])
{
    uint32_t index;
    if (bytes == 0) {
        return;
    }
    for (index = 0u; index < CAR_ACKERMANN_SNAPSHOT_SIZE; ++index) {
        bytes[index] = 0u;
    }
    ACKERMANN_DMB();
}

void car_ackermann_snapshot_publish(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE],
    const car_ackermann_snapshot_values_t *values)
{
    uint8_t record[CAR_ACKERMANN_SNAPSHOT_SIZE] = {0u};
    uint32_t previous_generation;
    uint32_t stable_generation;
    uint32_t crc;

    if (bytes == 0 || values == 0 ||
        (values->flags & (uint16_t)~CAR_ACKERMANN_SNAPSHOT_FLAGS_MASK) != 0u ||
        values->curve_phase > 3u || values->slowdown_request_permille > 1000u ||
        values->ratio_q15 > 32768u ||
        values->model_angle_cdeg < -2000 ||
        values->model_angle_cdeg > 2000 ||
        values->fastest_target_millirps < 0 ||
        values->fastest_target_millirps > 4800 ||
        values->left_target_millirps < 0 || values->left_target_millirps > 4800 ||
        values->right_target_millirps < 0 || values->right_target_millirps > 4800 ||
        values->left_duty_permille > 1000u ||
        values->right_duty_permille > 1000u) {
        return;
    }

    previous_generation = read_volatile_u32(bytes, 8u) & 0xFFFFFFFEu;
    stable_generation = previous_generation + 2u;
    write_volatile_generation(bytes, stable_generation - 1u);
    clean_committed_range(bytes + 8u, 4u);
    ACKERMANN_DMB();

    record[0] = 'A';
    record[1] = 'C';
    record[2] = 'K';
    record[3] = '1';
    record[4] = CAR_ACKERMANN_SNAPSHOT_VERSION;
    record[5] = CAR_ACKERMANN_SNAPSHOT_SIZE;
    write_le16(record, 6u, values->flags);
    write_le32(record, 8u, stable_generation);
    write_le32(record, 12u, values->timestamp_ms);
    write_le16(record, 16u, values->vision_sequence);
    record[18] = values->curve_phase;
    write_le16(record, 20u, values->slowdown_request_permille);
    write_le16(record, 24u, (uint16_t)values->model_angle_cdeg);
    write_le16(record, 26u, values->ratio_q15);
    write_le16(record, 28u, (uint16_t)values->applied_pwm_delta_us);
    write_le16(record, 30u, values->servo_pulse_us);
    write_le32(record, 32u, (uint32_t)values->fastest_target_millirps);
    write_le32(record, 36u, (uint32_t)values->left_target_millirps);
    write_le32(record, 40u, (uint32_t)values->right_target_millirps);
    write_le32(record, 44u, (uint32_t)values->left_measured_millirps);
    write_le32(record, 48u, (uint32_t)values->right_measured_millirps);
    write_le16(record, 52u, values->left_duty_permille);
    write_le16(record, 54u, values->right_duty_permille);
    record[56] = values->run_state;
    record[57] = values->fault_reason;

    /* The local image already contains the final even generation. Compute its
       CRC before publishing any bytes, then keep the volatile generation odd
       until payload and CRC are visible in SRAM. */
    crc = car_ackermann_snapshot_crc32(
        record, CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET);
    write_le32(record, CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET, crc);

    copy_to_volatile(bytes, 0u, record, 8u);
    copy_to_volatile(bytes, 12u, record + 12u, 48u);
    copy_to_volatile(bytes, CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET,
                     record + CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET, 4u);
    ACKERMANN_DMB();
    clean_committed_range(bytes, CAR_ACKERMANN_SNAPSHOT_SIZE);
    ACKERMANN_DMB();
    write_volatile_generation(bytes, stable_generation);
    ACKERMANN_DMB();
    clean_committed_range(bytes + 8u, 4u);
    ACKERMANN_DMB();
}

uint8_t car_ackermann_snapshot_validate(
    const uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE])
{
    uint32_t expected_crc;
    const uint16_t flags = bytes != 0 ? read_u16(bytes, 6u) : 0u;
    const uint16_t slowdown = bytes != 0 ? read_u16(bytes, 20u) : 0u;

    if (bytes == 0 || bytes[0] != 'A' || bytes[1] != 'C' ||
        bytes[2] != 'K' || bytes[3] != '1' ||
        bytes[4] != CAR_ACKERMANN_SNAPSHOT_VERSION ||
        bytes[5] != CAR_ACKERMANN_SNAPSHOT_SIZE ||
        (read_u32(bytes, 8u) & 1u) != 0u ||
        (flags & (uint16_t)~CAR_ACKERMANN_SNAPSHOT_FLAGS_MASK) != 0u ||
        (flags & (CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE |
                  CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN)) ==
            (CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE |
             CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN) ||
        bytes[18] > 3u || bytes[19] != 0u ||
        slowdown > 1000u ||
        (((flags & CAR_ACKERMANN_SNAPSHOT_FLAG_REQUEST_ACTIVE) != 0u) !=
         (slowdown != 0u)) ||
        bytes[22] != 0u || bytes[23] != 0u ||
        (int16_t)read_u16(bytes, 24u) < -2000 ||
        (int16_t)read_u16(bytes, 24u) > 2000 ||
        read_u16(bytes, 26u) == 0u || read_u16(bytes, 26u) > 32768u ||
        (int16_t)read_u16(bytes, 28u) < -430 ||
        (int16_t)read_u16(bytes, 28u) > 970 ||
        read_u16(bytes, 30u) < 1100u || read_u16(bytes, 30u) > 2500u ||
        bytes[56] > 12u || bytes[57] > 15u ||
        bytes[58] != 0u || bytes[59] != 0u ||
        (int32_t)read_u32(bytes, 32u) < 0 ||
        (int32_t)read_u32(bytes, 32u) > 4800 ||
        (int32_t)read_u32(bytes, 36u) < 0 ||
        (int32_t)read_u32(bytes, 36u) > 4800 ||
        (int32_t)read_u32(bytes, 40u) < 0 ||
        (int32_t)read_u32(bytes, 40u) > 4800 ||
        read_u16(bytes, 52u) > 1000u || read_u16(bytes, 54u) > 1000u) {
        return 0u;
    }
    expected_crc = car_ackermann_snapshot_crc32(
        bytes, CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET);
    return expected_crc == read_u32(bytes, CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET)
        ? 1u : 0u;
}

_Static_assert(sizeof(((car_ackermann_snapshot_values_t *)0)->flags) == 2u,
               "ackermann snapshot flags width changed");
