#ifndef CARTEST_ACKERMANN_SNAPSHOT_H
#define CARTEST_ACKERMANN_SNAPSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_ACKERMANN_SNAPSHOT_SIZE = 64u,
    CAR_ACKERMANN_SNAPSHOT_VERSION = 1u,
    CAR_ACKERMANN_SNAPSHOT_CRC_OFFSET = 60u,
    CAR_ACKERMANN_SNAPSHOT_FLAG_COORD_ENABLED = 1u << 0,
    CAR_ACKERMANN_SNAPSHOT_FLAG_REQUEST_ACTIVE = 1u << 1,
    CAR_ACKERMANN_SNAPSHOT_FLAG_IN_HOLD = 1u << 2,
    CAR_ACKERMANN_SNAPSHOT_FLAG_FAULT_LOCKED = 1u << 3,
    CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE = 1u << 4,
    CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN = 1u << 5,
    CAR_ACKERMANN_SNAPSHOT_FLAGS_MASK = 0x003Fu
};

/* Raw bytes are the ABI. Fields are serialized explicitly as little-endian,
   so this remains exactly 64 bytes on the H743 and on host test machines. */
typedef struct {
    uint16_t flags;
    uint32_t timestamp_ms;
    uint16_t vision_sequence;
    uint8_t curve_phase;
    uint16_t slowdown_request_permille;
    int16_t model_angle_cdeg;
    uint16_t ratio_q15;
    int16_t applied_pwm_delta_us;
    uint16_t servo_pulse_us;
    int32_t fastest_target_millirps;
    int32_t left_target_millirps;
    int32_t right_target_millirps;
    int32_t left_measured_millirps;
    int32_t right_measured_millirps;
    uint16_t left_duty_permille;
    uint16_t right_duty_permille;
    uint8_t run_state;
    uint8_t fault_reason;
} car_ackermann_snapshot_values_t;

/* Independent SWD-readable storage in the dedicated .ackermann_snapshot
   NOLOAD section. A zeroed buffer means no sample has been published. */
extern volatile uint8_t g_ackermann_snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE];

void car_ackermann_snapshot_reset(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE]);
void car_ackermann_snapshot_publish(
    volatile uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE],
    const car_ackermann_snapshot_values_t *values);
uint32_t car_ackermann_snapshot_crc32(const uint8_t *bytes,
                                      uint32_t length);
uint8_t car_ackermann_snapshot_validate(
    const uint8_t bytes[CAR_ACKERMANN_SNAPSHOT_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_ACKERMANN_SNAPSHOT_H */
