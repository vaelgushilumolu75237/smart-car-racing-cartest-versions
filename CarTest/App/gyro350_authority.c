#include "commissioning.h"
#include "vision_protocol.h"

#if CARTEST_GYRO350_ENABLED
static uint16_t map_error(const car_config_t *config, int32_t error)
{
    int32_t endpoint, value;
    if (error < -1000) error = -1000;
    if (error > 1000) error = 1000;
    endpoint = error < 0 ? config->follow_servo_left_us : config->follow_servo_right_us;
    value = config->servo_center_us + (int32_t)(
        (int64_t)(endpoint - config->servo_center_us) * (error < 0 ? -error : error) *
        config->follow_kp_permille / 1000000);
    if (value < config->servo_min_us) value = config->servo_min_us;
    if (value > config->servo_max_us) value = config->servo_max_us;
    return (uint16_t)value;
}

void car_control_update_gyro(car_control_t *control, const car_config_t *config, uint32_t now_ms)
{
    car_gyro_ring_input_t input = {0};
    uint8_t was_active;
    if (!control || !config || !config->gyro_enabled) return;
#if CARTEST_RED_OBSTACLE_ENABLED
    if (control->obstacle.active) {
        car_gyro_ring_init(&control->gyro);
        return;
    }
#endif
    /* Keep the reason visible while the controller's hard fault is latched.
       Its own revoked run request must not erase the ring fault next tick.
       The normal physical acknowledgement clears the latch, then OFF resets. */
    if (control->gyro.fault && control->fault_latched) return;
    was_active = control->gyro.active;
    input.now_ms = now_ms;
    input.enabled = 1u;
    input.run_active = control->follow_session_active && control->manual_run_request &&
        !control->fault_latched && control->fault == CAR_FAULT_NONE &&
        !control->isr_stop_request && !control->isr_release_seen &&
        control->isr_stop_fault == CAR_FAULT_NONE;
    input.imu_ready = control->imu_ready;
    input.imu_fault = control->imu_fault ||
        (control->imu_ready && (uint32_t)(now_ms * 1000u - control->imu_sample_us) >= 50000u);
    input.imu_sample_us = control->imu_sample_us;
    input.gyro_z_mdps = control->imu_z_mdps;
    input.yaw_mdeg = control->imu_yaw_mdeg;
    input.transport_fresh = control->vision_have_frame_sequence &&
        (uint32_t)(now_ms - control->vision_last_frame_alive_ms) < config->vision_timeout_ms &&
        car_control_camera_profile_ack_ready(control, config, now_ms);
    input.camera_fresh = input.transport_fresh && control->latest_rx_protocol_version == VP_VERSION_V4 &&
        control->latest_rx_sequence == control->vision_last_frame_sequence;
    /* Invalid images keep the existing gyro controller alive, but cannot
       start entry or move the yaw origin into ORBIT. */
    input.camera_phase = control->latest_rx_valid ? control->gyro_camera_phase : 0u;
    input.camera_side = control->gyro_camera_side;
    input.camera_sequence = control->vision_last_frame_sequence;
    input.ordinary_fresh = input.camera_fresh && control->gyro_ordinary && control->latest_rx_valid;
    input.applied_servo_us = control->applied_servo_output_active &&
        control->applied_servo_frame_ready ? control->applied_servo_pulse_us : 0u;
    car_gyro_ring_step(&control->gyro, &input);
    control->gyro_servo_us = control->gyro.servo_target_us ? control->gyro.servo_target_us :
        map_error(config, control->gyro.control_error_norm);
    if (control->gyro.active &&
        ((control->gyro.phase == CAR_GYRO_RING_ENTRY && control->gyro.entry_straight_done) ||
         control->gyro.phase == CAR_GYRO_RING_ORBIT)) {
        int32_t pulse = CAR_GYRO_RING_CENTER_SERVO_US +
            ((int32_t)control->gyro_servo_us - CAR_GYRO_RING_CENTER_SERVO_US) *
            CAR_GYRO_RING_TURN_SERVO_PERCENT / 100;
        if (pulse < config->servo_min_us) pulse = config->servo_min_us;
        if (pulse > config->servo_max_us) pulse = config->servo_max_us;
        control->gyro_servo_us = (uint16_t)pulse;
    }
    if (control->gyro.fault) {
        car_control_note_fault(control, input.imu_fault ? CAR_FAULT_IMU : CAR_FAULT_GYRO_RING);
    }
    if (control->gyro.active && !was_active) {
        /* A ring may take over an existing, manually started Follow session;
           it can never arm/start a session by itself. */
        if (!control->applied_servo_frame_ready || !control->applied_servo_output_active ||
            (!control->applied_motor_left_permille && !control->applied_motor_right_permille))
            car_control_note_fault(control, CAR_FAULT_GYRO_RING);
    }
    if (control->gyro.active && !control->gyro.outer_target_millirps &&
        control->gyro.phase != control->gyro_last_phase) {
        control->vision_hold_restart_required = 1u;
    }
    control->gyro_last_phase = control->gyro.phase;
}
#endif
