#ifndef CARTEST_OBSTACLE_BYPASS_H
#define CARTEST_OBSTACLE_BYPASS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_OBSTACLE_IDLE = 0,
    CAR_OBSTACLE_LEFT_OUT = 1,
    CAR_OBSTACLE_DIAGONAL_LEFT = 2,
    CAR_OBSTACLE_RIGHT_FORWARD = 3,
    CAR_OBSTACLE_PASS_FORWARD = 4,
    CAR_OBSTACLE_RIGHT_IN = 5,
    CAR_OBSTACLE_DIAGONAL_RIGHT = 6,
    CAR_OBSTACLE_LEFT_ALIGN = 7,
    CAR_OBSTACLE_REACQUIRE = 8,
    CAR_OBSTACLE_FAULT = 9,
    CAR_OBSTACLE_SEGMENTS = 7
};
enum {
    CAR_OBSTACLE_ERROR_NONE = 0,
    CAR_OBSTACLE_ERROR_UNCALIBRATED = 1,
    CAR_OBSTACLE_ERROR_IMU = 2,
    CAR_OBSTACLE_ERROR_ENCODER = 3,
    CAR_OBSTACLE_ERROR_TIMEOUT = 4,
    CAR_OBSTACLE_ERROR_INPUT = 5
};
/* Stage directions are fixed: L, straight, R, straight, R, straight, L.
 * Distance unit: mean forward wheel millirevolutions, NOT millimetres.
 * Turn angle: positive magnitude in millidegrees. Rate: positive mdps.
 * Zero route values are invalid. Supplied nonzero candidates need track tuning. */
typedef struct {
    uint32_t angle_mdeg;
    uint32_t distance_millirev;
    uint16_t outer_speed_millirps;
    uint32_t yaw_rate_mdps;
    uint32_t timeout_ms;
} car_obstacle_segment_t;
typedef struct {
    car_obstacle_segment_t segment[CAR_OBSTACLE_SEGMENTS];
    uint16_t heading_kp_permille; /* mdps / mdeg, scaled by 1000 */
} car_obstacle_config_t;
typedef struct {
    uint32_t now_ms;
    uint8_t run_active, trigger, imu_ready, imu_fault, odometry_valid;
    uint32_t imu_sample_us;
    int32_t gyro_z_mdps;
    int64_t yaw_mdeg;
    uint64_t forward_millirev;
    uint8_t ordinary_fresh;
    uint16_t camera_sequence;
} car_obstacle_input_t;
typedef struct {
    uint8_t active, state, error, resumed;
    uint16_t outer_target_millirps;
    int32_t control_error_norm;
    uint32_t stage_started_ms;
    uint64_t stage_start_millirev, distance_millirev;
    int64_t heading_target_mdeg;
    uint8_t recovery_count, recovery_have_sequence;
    uint16_t recovery_sequence;
    uint32_t recovery_last_ms;
    uint32_t accepted_triggers, ignored_triggers;
} car_obstacle_t;
uint8_t car_obstacle_config_valid(const car_obstacle_config_t *config);
void car_obstacle_init(car_obstacle_t *state);
void car_obstacle_step(car_obstacle_t *state, const car_obstacle_config_t *config,
                       const car_obstacle_input_t *input);
#ifdef __cplusplus
}
#endif
#endif
