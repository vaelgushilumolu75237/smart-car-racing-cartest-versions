#ifndef CARTEST_GYRO_RING_H
#define CARTEST_GYRO_RING_H

#include <stdint.h>
#include "gyro350_config.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_GYRO_RING_OFF = 0u,
    CAR_GYRO_RING_ENTRY = 1u,
    CAR_GYRO_RING_ORBIT = 2u,
    CAR_GYRO_RING_CENTER = 3u,
    CAR_GYRO_RING_EXIT_STRAIGHT = 4u,
    CAR_GYRO_RING_SEARCH = 5u,
    CAR_GYRO_RING_WAIT_ROUTE = 6u,
    CAR_GYRO_RING_REARM = 7u,
    CAR_GYRO_RING_FAULT = 8u
};

enum {
    CAR_GYRO_RING_FAULT_NONE = 0u,
    CAR_GYRO_RING_FAULT_IMU = 1u << 0,
    CAR_GYRO_RING_FAULT_TRANSPORT = 1u << 1,
    CAR_GYRO_RING_FAULT_TIMEOUT = 1u << 2,
    CAR_GYRO_RING_FAULT_INPUT = 1u << 3
};

enum {
    CAR_GYRO_RING_CENTER_SERVO_US = 1530u,
    CAR_GYRO_RING_OUTER_TARGET_MILLIRPS = CAR_GYRO350_OUTER_TARGET_MILLIRPS,
    CAR_GYRO_RING_ENTRY_STRAIGHT_MS = 1500u,
    CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS =
        CAR_GYRO_RING_OUTER_TARGET_MILLIRPS * 75u / 100u / 2u,
    CAR_GYRO_RING_TURN_SERVO_PERMILLE = 1875u,
    CAR_GYRO_RING_EXIT_STRAIGHT_MS = 300u,
    CAR_GYRO_RING_SEARCH_LIMIT_MS = 3000u,
    CAR_GYRO_RING_ENTRY_LIMIT_MS = 4000u,
    CAR_GYRO_RING_ORBIT_LIMIT_MS = 16000u,
    CAR_GYRO_RING_CENTER_LIMIT_MS = 1000u,
    CAR_GYRO_RING_TARGET_YAW_RATE_MDPS = CAR_GYRO350_TARGET_YAW_RATE_MDPS,
    CAR_GYRO_RING_TARGET_PROGRESS_MDEG = 350000,
    CAR_GYRO_RING_RECOVERY_FRAMES = 3u,
    /* Three frames do not remain valid indefinitely if the camera stops. */
    CAR_GYRO_RING_RECOVERY_MAX_AGE_MS = 250u
};

typedef struct {
    uint32_t now_ms;
    uint8_t enabled;
    uint8_t run_active;
    uint8_t imu_ready;
    uint8_t imu_fault;
    uint8_t transport_fresh;
    uint32_t imu_sample_us;
    int32_t gyro_z_mdps;
    int64_t yaw_mdeg;
    uint8_t camera_phase;
    int8_t camera_side;
    uint8_t camera_fresh;
    uint8_t ordinary_fresh;
    uint16_t camera_sequence;
    uint16_t applied_servo_us;
} car_gyro_ring_input_t;

typedef struct {
    uint8_t active;
    uint8_t fault;
    uint8_t phase;
    uint8_t allow_rearm;
    uint8_t hold_ramp;
    int32_t control_error_norm;
    uint16_t servo_target_us;
    uint16_t outer_target_millirps;
    uint32_t search_elapsed_ms;
    int64_t progress_mdeg;
    uint32_t last_event_ms;

    /* Private, allocation-free state. */
    int8_t entry_side;
    uint8_t entry_straight_done;
    uint8_t pending_orbit;
    uint8_t recovery_count;
    uint8_t have_camera_sequence;
    uint8_t have_ordinary_sequence;
    uint8_t have_imu_sample;
    uint8_t state;
    uint8_t fault_latched;
    uint8_t imu_integral_initialized;
    uint16_t last_camera_sequence;
    uint16_t last_ordinary_sequence;
    uint32_t state_started_ms;
    uint32_t recovery_last_ms;
    uint32_t last_imu_sample_us;
    int64_t yaw_origin_mdeg;
    int32_t imu_integral_permille;
    int64_t imu_integral_remainder;
    int32_t last_control_error;
} car_gyro_ring_t;

void car_gyro_ring_init(car_gyro_ring_t *ring);
void car_gyro_ring_step(car_gyro_ring_t *ring,
                        const car_gyro_ring_input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_GYRO_RING_H */
