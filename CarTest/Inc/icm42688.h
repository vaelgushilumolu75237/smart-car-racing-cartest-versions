#ifndef CARTEST_ICM42688_H
#define CARTEST_ICM42688_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CAR_IMU_YAW_SIGN
#define CAR_IMU_YAW_SIGN 1
#endif

#if (CAR_IMU_YAW_SIGN != 1) && (CAR_IMU_YAW_SIGN != -1)
#error "CAR_IMU_YAW_SIGN must be +1 or -1"
#endif

enum {
    CAR_IMU_FAULT_NONE = 0u,
    CAR_IMU_FAULT_BUS = 1u << 0,
    CAR_IMU_FAULT_WHOAMI = 1u << 1,
    CAR_IMU_FAULT_STALE = 1u << 2,
    CAR_IMU_FAULT_DT = 1u << 3,
    CAR_IMU_FAULT_CONFIG = 1u << 4
};

typedef struct {
    void *context;
    /* Bus callbacks return 1 on success and 0 on failure. */
    int (*read)(void *context, uint8_t reg, uint8_t *out, uint16_t len);
    int (*write)(void *context, uint8_t reg, uint8_t value);
} car_imu_bus_t;

typedef struct {
    /* Public measurements and status consumed by the application. */
    uint8_t ready;
    uint8_t calibrated;
    uint8_t fault;
    uint8_t sample_valid;
    /* Raw WHO_AM_I evidence from the most recent successful register read. */
    uint8_t who_am_i;
    uint8_t who_am_i_valid;
    uint32_t who_am_i_reads;
    uint32_t sample_us;
    int32_t gyro_z_mdps;
    int64_t yaw_mdeg;
    uint32_t sample_count;
    uint32_t calibration_count;
    int32_t bias_z_raw_q16;

    /* Private state: kept in the struct so the driver allocates no memory. */
    car_imu_bus_t bus;
    uint8_t state;
    uint8_t have_sample;
    uint8_t have_previous_rate;
    uint8_t have_previous_accel;
    uint32_t init_us;
    uint32_t state_deadline_us;
    uint32_t next_poll_us;
    uint32_t last_good_us;
    uint32_t calibration_start_us;
    uint32_t previous_sample_us;
    int64_t yaw_remainder;
    uint16_t calibration_samples;
    uint16_t reserved;
    int64_t calibration_sum_z_raw;
    int32_t previous_rate_mdps;
    int16_t previous_accel[3];
    int32_t gyro_raw[3];
    int32_t accel_raw[3];
} car_imu_t;

void car_imu_init(car_imu_t *imu, const car_imu_bus_t *bus,
                  uint32_t now_us);
void car_imu_poll(car_imu_t *imu, uint32_t now_us,
                  uint8_t stationary_allowed);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_ICM42688_H */
