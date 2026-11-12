#ifndef CAR_ICM42688_STM32_H
#define CAR_ICM42688_STM32_H
#include "icm42688.h"
enum {
    CAR_IMU_SPI_ERROR_NONE = 0u,
    CAR_IMU_SPI_ERROR_NOT_OPEN = 1u,
    CAR_IMU_SPI_ERROR_ARGUMENT = 2u,
    CAR_IMU_SPI_ERROR_INTERRUPTS_MASKED = 3u,
    CAR_IMU_SPI_ERROR_CLOCK = 4u,
    CAR_IMU_SPI_ERROR_COUNTER_STOPPED = 5u,
    CAR_IMU_SPI_ERROR_DEADLINE = 6u
};
typedef struct {
    uint32_t attempts, completed, failures;
    uint32_t last_error, failed_command, failed_length, failed_bits;
    uint32_t failed_elapsed_cycles, last_elapsed_cycles, max_elapsed_cycles;
} car_imu_spi_diag_t;
/* Read-only debug evidence. Last failure survives later successful exchanges. */
extern volatile car_imu_spi_diag_t car_imu_spi_diag;
int car_imu_spi2_open(car_imu_bus_t *bus);
#endif
