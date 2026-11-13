#include "commissioning.h"
#if CARTEST_RED_OBSTACLE_ENABLED
#include "obstacle_bypass_config.h"
#include "vision_protocol.h"
static uint16_t servo_from_error(const car_config_t *config, int32_t error) {
    int32_t endpoint = error < 0 ? config->follow_servo_left_us : config->follow_servo_right_us;
    int32_t value = config->servo_center_us + (int32_t)(
        (int64_t)(endpoint - config->servo_center_us) * (error < 0 ? -error : error) *
        config->follow_kp_permille / 1000000);
    if (value < config->servo_min_us) value = config->servo_min_us;
    if (value > config->servo_max_us) value = config->servo_max_us;
    return (uint16_t)value;
}
void car_control_update_obstacle(car_control_t *control, const car_config_t *config, uint32_t now_ms) {
    car_obstacle_input_t input = {0};
    if (!control || !config) return;
    if (control->obstacle.error && control->fault_latched) {
        control->obstacle_trigger = 0;
        return;
    }
    input.now_ms = now_ms;
    input.run_active = config->follow_mode_compiled &&
        control->follow_session_active && control->manual_run_request &&
        !control->fault_latched && control->fault == CAR_FAULT_NONE &&
        !control->isr_stop_request && !control->isr_release_seen &&
        control->isr_stop_fault == CAR_FAULT_NONE;
    input.trigger = control->obstacle_trigger;
    control->obstacle_trigger = 0;
    input.imu_ready = control->imu_ready;
    input.imu_fault = control->imu_fault;
    input.imu_sample_us = control->imu_sample_us;
    input.gyro_z_mdps = control->imu_z_mdps;
    input.yaw_mdeg = control->imu_yaw_mdeg;
    input.odometry_valid = control->obstacle_odometry_valid;
    input.forward_millirev = control->obstacle_forward_millirev;
    input.ordinary_fresh = control->latest_rx_have_frame && control->latest_rx_valid &&
        control->latest_rx_sequence == control->vision_last_frame_sequence &&
        (uint32_t)(now_ms - control->latest_rx_arrival_ms) < config->vision_timeout_ms &&
        car_control_camera_profile_ack_ready(control, config, now_ms) &&
        control->gyro_camera_phase == 0 && control->gyro_ordinary &&
        control->latest_rx_protocol_version == VP_VERSION_V4;
    input.camera_sequence = control->latest_rx_sequence;
    car_obstacle_step(&control->obstacle, &car_obstacle_route_config, &input);
    control->obstacle_servo_us = servo_from_error(config, control->obstacle.control_error_norm);
    if (control->obstacle.error && control->obstacle.active) {
        car_control_note_fault(control,
            control->obstacle.error == CAR_OBSTACLE_ERROR_UNCALIBRATED ? CAR_FAULT_CONFIG :
            control->obstacle.error == CAR_OBSTACLE_ERROR_IMU ? CAR_FAULT_IMU :
            control->obstacle.error == CAR_OBSTACLE_ERROR_ENCODER ? CAR_FAULT_ENCODER_INIT :
            CAR_FAULT_OBSTACLE);
    }
    /* Preserve the existing motor PI and ramp across the moving handover. */
}
#endif
