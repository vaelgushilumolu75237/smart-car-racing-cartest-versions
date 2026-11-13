#include "icm42688.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* ICM-42688-P register map, bank 0. */
enum {
    REG_INT_STATUS = 0x2Du,
    REG_ACCEL_DATA_X1 = 0x1Fu,
    REG_DEVICE_CONFIG = 0x11u,
    REG_PWR_MGMT0 = 0x4Eu,
    REG_GYRO_CONFIG0 = 0x4Fu,
    REG_ACCEL_CONFIG0 = 0x50u,
    REG_WHO_AM_I = 0x75u,
    WHO_AM_I_VALUE = 0x47u,
    INT_STATUS_DATA_READY = 0x08u,
    DEVICE_SOFT_RESET = 0x01u,
    PWR_ACCEL_GYRO_LOW_NOISE = 0x0Fu,
    CONFIG_FS_MAX_200HZ = 0x07u
};

enum {
    IMU_STATE_RESET_WAIT = 1u,
    IMU_STATE_OFF_WAIT = 2u,
    IMU_STATE_SETTLE_WAIT = 3u,
    IMU_STATE_RUNNING = 4u,
    RESET_WAIT_US = 1000u,
    WHOAMI_TIMEOUT_US = 50000u,
    WHOAMI_RETRY_US = 5000u,
    POWER_MODE_WRITE_GUARD_US = 200u,
    GYRO_STARTUP_WAIT_US = 45000u,
    SAMPLE_POLL_PERIOD_US = 5000u,
    SAMPLE_STALE_TIMEOUT_US = 50000u,
    SAMPLE_MAX_DT_US = 25000u,
    CALIBRATION_TIME_US = 2000000u,
    CALIBRATION_MIN_SAMPLES = 360u,
    CALIBRATION_MAX_GYRO_RAW = 80,
    CALIBRATION_MIN_ACCEL_RAW = 1536u,
    CALIBRATION_MAX_ACCEL_RAW = 2560u,
    CALIBRATION_MAX_ACCEL_STEP_RAW = 350
};

#define YAW_DENOMINATOR INT64_C(2000000)
#define GYRO_MDPS_NUMERATOR INT64_C(2000000)
#define RAW_Q16_ONE INT64_C(65536)

static uint8_t time_reached(uint32_t now, uint32_t deadline)
{
    return ((uint32_t)(now - deadline) < UINT32_C(0x80000000)) ? 1u : 0u;
}

static uint8_t bus_write(car_imu_t *imu, uint8_t reg, uint8_t value)
{
    if (imu->bus.write == NULL ||
        imu->bus.write(imu->bus.context, reg, value) == 0) {
        imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_BUS);
        imu->ready = 0u;
        imu->sample_valid = 0u;
        return 0u;
    }
    return 1u;
}

static uint8_t bus_read(car_imu_t *imu, uint8_t reg,
                        uint8_t *out, uint16_t len)
{
    if (imu->bus.read == NULL || out == NULL ||
        imu->bus.read(imu->bus.context, reg, out, len) == 0) {
        imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_BUS);
        imu->ready = 0u;
        imu->sample_valid = 0u;
        return 0u;
    }
    return 1u;
}

static int32_t decode_i16_be(const uint8_t *bytes)
{
    uint16_t value = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    if ((value & UINT16_C(0x8000)) != 0u) {
        return (int32_t)value - INT32_C(65536);
    }
    return (int32_t)value;
}

static int64_t abs_i64(int64_t value)
{
    if (value < 0) {
        return -value;
    }
    return value;
}

static int32_t divide_round_i64(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }
    return (int32_t)(numerator / denominator);
}

static int32_t raw_to_mdps_q16(int32_t raw, int32_t bias_raw_q16)
{
    const int64_t effective_raw_q16 =
        (int64_t)raw * RAW_Q16_ONE - bias_raw_q16;
    const int64_t numerator = effective_raw_q16 * GYRO_MDPS_NUMERATOR;
    const int64_t denominator = INT64_C(32768) * RAW_Q16_ONE;
    return divide_round_i64(numerator, denominator);
}

static uint8_t calibration_sample_is_stationary(const car_imu_t *imu)
{
    uint64_t accel_norm_sq = 0u;
    uint32_t axis;
    int32_t min_axis = INT32_MAX;
    int32_t max_axis = INT32_MIN;

    for (axis = 0u; axis < 3u; ++axis) {
        int64_t value = imu->accel_raw[axis];
        int32_t gyro_abs = (int32_t)abs_i64(imu->gyro_raw[axis]);
        uint32_t accel_abs = (uint32_t)abs_i64(value);

        if (gyro_abs > CALIBRATION_MAX_GYRO_RAW) {
            return 0u;
        }
        accel_norm_sq += (uint64_t)accel_abs * accel_abs;

        if (imu->have_previous_accel != 0u) {
            int32_t delta = (int32_t)imu->accel_raw[axis] -
                            (int32_t)imu->previous_accel[axis];
            int32_t delta_abs = delta < 0 ? -delta : delta;
            if (delta_abs > CALIBRATION_MAX_ACCEL_STEP_RAW) {
                return 0u;
            }
        }
        if (imu->accel_raw[axis] < min_axis) {
            min_axis = imu->accel_raw[axis];
        }
        if (imu->accel_raw[axis] > max_axis) {
            max_axis = imu->accel_raw[axis];
        }
    }

    (void)min_axis;
    (void)max_axis;
    if (accel_norm_sq <
            (uint64_t)CALIBRATION_MIN_ACCEL_RAW * CALIBRATION_MIN_ACCEL_RAW ||
        accel_norm_sq >
            (uint64_t)CALIBRATION_MAX_ACCEL_RAW * CALIBRATION_MAX_ACCEL_RAW) {
        return 0u;
    }
    return 1u;
}

static void reset_calibration(car_imu_t *imu)
{
    imu->calibration_start_us = 0u;
    imu->calibration_samples = 0u;
    imu->calibration_count = 0u;
    imu->calibration_sum_z_raw = 0;
    imu->have_previous_accel = 0u;
}

static void update_calibration(car_imu_t *imu, uint32_t now_us,
                               uint8_t stationary_allowed)
{
    uint32_t axis;

    if (imu->calibrated != 0u) {
        return;
    }
    if (stationary_allowed == 0u ||
        calibration_sample_is_stationary(imu) == 0u) {
        reset_calibration(imu);
        return;
    }

    if (imu->calibration_samples == 0u) {
        imu->calibration_start_us = now_us;
        imu->calibration_sum_z_raw = 0;
    }
    imu->calibration_sum_z_raw += imu->gyro_raw[2];
    if (imu->calibration_samples < UINT16_MAX) {
        ++imu->calibration_samples;
    }
    imu->calibration_count = imu->calibration_samples;

    for (axis = 0u; axis < 3u; ++axis) {
        imu->previous_accel[axis] = (int16_t)imu->accel_raw[axis];
    }
    imu->have_previous_accel = 1u;

    if (imu->calibration_samples >= CALIBRATION_MIN_SAMPLES &&
        (uint32_t)(now_us - imu->calibration_start_us) >=
            CALIBRATION_TIME_US) {
        int64_t mean_q16 = imu->calibration_sum_z_raw * RAW_Q16_ONE /
                           imu->calibration_samples;
        if (mean_q16 > INT32_MAX || mean_q16 < INT32_MIN) {
            imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_CONFIG);
            imu->ready = 0u;
            imu->sample_valid = 0u;
            return;
        }
        imu->bias_z_raw_q16 = (int32_t)mean_q16;
        imu->calibrated = 1u;
        imu->have_previous_rate = 0u;
        imu->yaw_remainder = 0;
    }
}

static void integrate_yaw(car_imu_t *imu, uint32_t now_us,
                          uint32_t dt_us, int32_t current_rate_mdps)
{
    int64_t numerator;

    if (imu->have_previous_rate == 0u) {
        imu->previous_rate_mdps = current_rate_mdps;
        imu->have_previous_rate = 1u;
        imu->previous_sample_us = now_us;
        return;
    }

    numerator = ((int64_t)imu->previous_rate_mdps + current_rate_mdps) *
                dt_us;
    /* current_rate_mdps already uses the configured vehicle yaw sign. */
    numerator += imu->yaw_remainder;
    imu->yaw_mdeg += numerator / YAW_DENOMINATOR;
    imu->yaw_remainder = numerator % YAW_DENOMINATOR;
    imu->previous_rate_mdps = current_rate_mdps;
    imu->previous_sample_us = now_us;
}

static void fault_dt(car_imu_t *imu)
{
    imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_DT);
    imu->ready = 0u;
    imu->sample_valid = 0u;
}

static void read_new_sample(car_imu_t *imu, uint32_t now_us,
                            uint8_t stationary_allowed)
{
    uint8_t status;
    uint8_t data[12];
    uint32_t dt_us = 0u;
    uint32_t axis;
    int32_t current_rate_mdps;

    if (bus_read(imu, REG_INT_STATUS, &status, 1u) == 0u) {
        return;
    }
    if ((status & INT_STATUS_DATA_READY) == 0u) {
        uint32_t reference_us = imu->have_sample != 0u
                                    ? imu->last_good_us
                                    : imu->state_deadline_us;
        if ((uint32_t)(now_us - reference_us) > SAMPLE_STALE_TIMEOUT_US) {
            imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_STALE);
            imu->ready = 0u;
            imu->sample_valid = 0u;
        }
        return;
    }
    if (bus_read(imu, REG_ACCEL_DATA_X1, data, (uint16_t)sizeof(data)) == 0u) {
        return;
    }

    for (axis = 0u; axis < 3u; ++axis) {
        imu->accel_raw[axis] = decode_i16_be(&data[axis * 2u]);
        imu->gyro_raw[axis] = decode_i16_be(&data[6u + axis * 2u]);
    }

    if (imu->have_sample != 0u) {
        dt_us = (uint32_t)(now_us - imu->sample_us);
        if (dt_us == 0u || dt_us > SAMPLE_MAX_DT_US) {
            fault_dt(imu);
            return;
        }
    }

    update_calibration(imu, now_us, stationary_allowed);
    if (imu->fault != CAR_IMU_FAULT_NONE) {
        return;
    }

    current_rate_mdps = raw_to_mdps_q16(imu->gyro_raw[2],
                                        imu->bias_z_raw_q16) *
                        CAR_IMU_YAW_SIGN;
    imu->gyro_z_mdps = current_rate_mdps;
    imu->sample_us = now_us;
    imu->last_good_us = now_us;
    imu->have_sample = 1u;
    imu->sample_valid = 1u;
    if (imu->sample_count < UINT32_MAX) {
        ++imu->sample_count;
    }

    if (imu->calibrated != 0u && dt_us != 0u) {
        integrate_yaw(imu, now_us, dt_us, current_rate_mdps);
    } else {
        imu->have_previous_rate = 0u;
        imu->yaw_remainder = 0;
    }
}

void car_imu_init(car_imu_t *imu, const car_imu_bus_t *bus,
                  uint32_t now_us)
{
    if (imu == NULL) {
        return;
    }
    (void)memset(imu, 0, sizeof(*imu));
    imu->init_us = now_us;
    imu->state = IMU_STATE_RESET_WAIT;
    imu->state_deadline_us = now_us + RESET_WAIT_US;
    imu->next_poll_us = imu->state_deadline_us;
    if (bus != NULL) {
        imu->bus = *bus;
    }
    if (imu->bus.write == NULL ||
        imu->bus.write(imu->bus.context, REG_DEVICE_CONFIG,
                       DEVICE_SOFT_RESET) == 0) {
        imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_BUS);
    }
}

void car_imu_poll(car_imu_t *imu, uint32_t now_us,
                  uint8_t stationary_allowed)
{
    uint8_t who_am_i;

    if (imu == NULL || imu->fault != CAR_IMU_FAULT_NONE) {
        return;
    }

    if (imu->state == IMU_STATE_RESET_WAIT) {
        if (time_reached(now_us, imu->next_poll_us) == 0u) {
            return;
        }
        if (bus_read(imu, REG_WHO_AM_I, &who_am_i, 1u) == 0u) {
            return;
        }
        /* Keep the raw bus evidence separate from the expected-value check.
         * A failed read is not a valid zero/FF sample, while a successful
         * mismatch must remain observable even when WHO_AM_I latches fault. */
        imu->who_am_i = who_am_i;
        imu->who_am_i_valid = 1u;
        if (imu->who_am_i_reads != UINT32_MAX) {
            ++imu->who_am_i_reads;
        }
        if (who_am_i != WHO_AM_I_VALUE) {
            if ((uint32_t)(now_us - imu->init_us) >= WHOAMI_TIMEOUT_US) {
                imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_WHOAMI);
            } else {
                imu->next_poll_us = now_us + WHOAMI_RETRY_US;
            }
            return;
        }
        if (bus_write(imu, REG_PWR_MGMT0, 0u) == 0u) {
            return;
        }
        imu->state = IMU_STATE_OFF_WAIT;
        imu->state_deadline_us = now_us + POWER_MODE_WRITE_GUARD_US;
        return;
    }

    if (imu->state == IMU_STATE_OFF_WAIT) {
        if (time_reached(now_us, imu->state_deadline_us) == 0u) {
            return;
        }
        if (bus_write(imu, REG_GYRO_CONFIG0, CONFIG_FS_MAX_200HZ) == 0u ||
            bus_write(imu, REG_ACCEL_CONFIG0, CONFIG_FS_MAX_200HZ) == 0u ||
            bus_write(imu, REG_PWR_MGMT0,
                      PWR_ACCEL_GYRO_LOW_NOISE) == 0u) {
            return;
        }
        imu->state = IMU_STATE_SETTLE_WAIT;
        imu->state_deadline_us = now_us + GYRO_STARTUP_WAIT_US;
        return;
    }

    if (imu->state == IMU_STATE_SETTLE_WAIT) {
        if (time_reached(now_us, imu->state_deadline_us) == 0u) {
            return;
        }
        imu->state = IMU_STATE_RUNNING;
        imu->ready = 1u;
        imu->state_deadline_us = now_us;
        imu->next_poll_us = now_us;
    }

    if (imu->state != IMU_STATE_RUNNING || imu->ready == 0u) {
        imu->fault = (uint8_t)(imu->fault | CAR_IMU_FAULT_CONFIG);
        imu->ready = 0u;
        return;
    }

    if (stationary_allowed == 0u && imu->calibrated == 0u) {
        reset_calibration(imu);
    }
    if (time_reached(now_us, imu->next_poll_us) == 0u) {
        return;
    }
    imu->next_poll_us = now_us + SAMPLE_POLL_PERIOD_US;
    read_new_sample(imu, now_us, stationary_allowed);
}
