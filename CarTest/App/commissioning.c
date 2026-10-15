#include "commissioning.h"
#include "car_calibration.h"

#define CAR_DEFAULT_VISION_TIMEOUT_MS 250u
#define CAR_HARD_BENCH_LIMIT_PM       400u
#define CAR_HARD_FOLLOW_LIMIT_PM      700u
#define CAR_FOLLOW_NORMAL_LIMIT_PM    600u
#define CAR_FOLLOW_BASE_TARGET_PM     500u
#define CAR_MAX_BENCH_RUN_MS          5500u
#define CAR_MAX_SERVO_PULSE_US         20000u
#define CAR_RAMP_PERIOD_MS            10u
#define CAR_BENCH_TARGET_HOLD_MS      CARTEST_BENCH_HOLD_MS
#define CAR_RECOVERY_REQUIRED_FRAMES  CAR_CONTROL_RECOVERY_REQUIRED_FRAMES
#define CAR_RECOVERY_MAX_GAP_MS       250u
#define CAR_RECOVERY_MAX_QUEUE_MS     100u
#define CAR_FOLLOW_KP_PERMILLE        700u
#define CAR_FOLLOW_RAMP_STEP_PM       10u

void car_config_init_application(car_config_t *config,
                                 uint8_t bench_mode_compiled,
                                 uint8_t debug_steering_mode_compiled,
                                 uint8_t follow_mode_compiled,
                                 uint8_t motor_hardware_accepted)
{
    if (config == 0) {
        return;
    }

    *config = (car_config_t){
        .bench_mode_compiled = bench_mode_compiled,
        .servo_limits_measured = CARTEST_SERVO_LIMITS_MEASURED,
        .servo_min_us = CARTEST_SERVO_MIN_US,
        .servo_max_us = CARTEST_SERVO_MAX_US,
        .servo_center_us = CARTEST_SERVO_CENTER_US,
        .bench_duty_limit_permille = CAR_HARD_BENCH_LIMIT_PM,
        .bench_ramp_step_permille = 10u,
        .bench_run_limit_ms = CARTEST_BENCH_RUN_LIMIT_MS,
        .vision_timeout_ms = CAR_DEFAULT_VISION_TIMEOUT_MS,
        .motor_hardware_accepted = motor_hardware_accepted,
        .debug_steering_mode_compiled = debug_steering_mode_compiled,
        .follow_mode_compiled = follow_mode_compiled,
        .follow_calibration_ready = CARTEST_FOLLOW_CALIBRATION_COMPLETE,
        .follow_positive_error_right_measured =
            CARTEST_STEERING_DIRECTION_CONFIRMED,
        .follow_servo_left_us = CARTEST_SERVO_LEFT_US,
        .follow_servo_right_us = CARTEST_SERVO_RIGHT_US,
        .follow_left_encoder_counts_per_rev =
            CARTEST_ENCODER_LEFT_COUNTS_PER_REV,
        .follow_right_encoder_counts_per_rev =
            CARTEST_ENCODER_RIGHT_COUNTS_PER_REV,
        .follow_left_encoder_sign = CARTEST_ENCODER_LEFT_SIGN,
        .follow_right_encoder_sign = CARTEST_ENCODER_RIGHT_SIGN,
        .follow_kp_permille = CARTEST_STEERING_GAIN_PERMILLE,
        .follow_max_duty_permille = CAR_FOLLOW_NORMAL_LIMIT_PM,
        .follow_ramp_step_permille = CAR_FOLLOW_RAMP_STEP_PM
    };
}

static uint8_t config_is_valid(const car_config_t *config)
{
    int32_t left_delta;
    int32_t right_delta;

    if (config == 0 || config->vision_timeout_ms == 0u ||
        config->vision_timeout_ms > 0x7fffffffu ||
        config->motor_hardware_accepted > 1u) {
        return 0u;
    }

    if (config->debug_steering_mode_compiled > 1u ||
        (uint32_t)(config->bench_mode_compiled != 0u) +
            (uint32_t)(config->follow_mode_compiled != 0u) +
            (uint32_t)(config->debug_steering_mode_compiled != 0u) > 1u) {
        return 0u;
    }

    if (config->bench_mode_compiled != 0u &&
        (config->bench_duty_limit_permille == 0u ||
         config->bench_duty_limit_permille > CAR_HARD_BENCH_LIMIT_PM ||
         config->bench_ramp_step_permille == 0u ||
         config->bench_run_limit_ms == 0u ||
         config->bench_run_limit_ms > CAR_MAX_BENCH_RUN_MS)) {
        return 0u;
    }

    if (config->servo_limits_measured != 0u &&
        (config->servo_min_us == 0u ||
         config->servo_min_us >= config->servo_max_us ||
         config->servo_max_us >= CAR_MAX_SERVO_PULSE_US ||
         config->servo_center_us < config->servo_min_us ||
         config->servo_center_us > config->servo_max_us)) {
        return 0u;
    }

    if (config->debug_steering_mode_compiled != 0u &&
        (config->servo_limits_measured == 0u ||
         config->servo_min_us != CARTEST_SERVO_MIN_US ||
         config->servo_max_us != CARTEST_SERVO_MAX_US ||
         config->servo_center_us != CARTEST_SERVO_CENTER_US ||
         config->vision_timeout_ms > CAR_DEFAULT_VISION_TIMEOUT_MS ||
         config->follow_positive_error_right_measured == 0u ||
         config->follow_kp_permille != CARTEST_STEERING_GAIN_PERMILLE ||
         CARTEST_STEERING_GAIN_PERMILLE != CAR_FOLLOW_KP_PERMILLE)) {
        return 0u;
    }

    if (config->follow_mode_compiled != 0u) {
        if (config->follow_calibration_ready == 0u ||
            config->follow_positive_error_right_measured == 0u ||
            config->servo_limits_measured == 0u ||
            config->follow_servo_left_us == 0u ||
            config->follow_servo_right_us == 0u ||
            config->follow_left_encoder_counts_per_rev == 0u ||
            config->follow_right_encoder_counts_per_rev == 0u ||
            (config->follow_left_encoder_sign != -1 &&
             config->follow_left_encoder_sign != 1) ||
            (config->follow_right_encoder_sign != -1 &&
             config->follow_right_encoder_sign != 1) ||
            config->follow_kp_permille != CAR_FOLLOW_KP_PERMILLE ||
            config->follow_max_duty_permille != CAR_FOLLOW_NORMAL_LIMIT_PM ||
            config->follow_ramp_step_permille != CAR_FOLLOW_RAMP_STEP_PM) {
            return 0u;
        }
        if (config->vision_timeout_ms > CAR_DEFAULT_VISION_TIMEOUT_MS ||
            config->servo_center_us != 1530u) {
            return 0u;
        }

        if (config->follow_servo_left_us < config->servo_min_us ||
            config->follow_servo_left_us > config->servo_max_us ||
            config->follow_servo_right_us < config->servo_min_us ||
            config->follow_servo_right_us > config->servo_max_us) {
            return 0u;
        }
        left_delta = (int32_t)config->follow_servo_left_us -
                     (int32_t)config->servo_center_us;
        right_delta = (int32_t)config->follow_servo_right_us -
                      (int32_t)config->servo_center_us;
        if (left_delta == 0 || right_delta == 0 ||
            (left_delta < 0) == (right_delta < 0)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t is_hard_fault(car_fault_t fault)
{
    switch (fault) {
    case CAR_FAULT_WATCHDOG_RESET:
    case CAR_FAULT_LOOP_GAP:
    case CAR_FAULT_ENCODER_INIT:
    case CAR_FAULT_CLOCK:
    case CAR_FAULT_TIMER:
    case CAR_FAULT_CONFIG:
    case CAR_FAULT_WATCHDOG_INIT:
        return 1u;
    default:
        return 0u;
    }
}

static void stop_bench(car_control_t *control)
{
    control->bench_output_duty_permille = 0u;
    control->bench_target_reached_ms = 0u;
    control->bench_target_hold_active = 0u;
    if (control->mode == CAR_MODE_BENCH_RUNNING) {
        control->mode = CAR_MODE_BENCH_COMPLETE;
    }
}

static uint8_t bench_channel_mask_for_selection(uint8_t motor,
                                                uint8_t input)
{
    if (motor == CAR_MOTOR_LEFT) {
        if (input == CAR_MOTOR_IN1) return 0x01u;
        if (input == CAR_MOTOR_IN2) return 0x02u;
        return 0u;
    }
    if (motor == CAR_MOTOR_RIGHT) {
        if (input == CAR_MOTOR_IN1) return 0x04u;
        if (input == CAR_MOTOR_IN2) return 0x08u;
        return 0u;
    }
    if (motor == CAR_MOTOR_BOTH_FORWARD && input == CAR_MOTOR_IN1) {
        return 0x06u;
    }
    return 0u;
}

uint8_t car_control_bench_channel_mask(const car_control_t *control)
{
    if (control == 0) {
        return 0u;
    }
    return bench_channel_mask_for_selection(control->bench_motor,
                                            control->bench_input);
}

static void clear_trusted_observation(car_control_t *control)
{
    control->vision_valid = 0u;
    control->vision_timeout = 1u;
    control->vision_confidence = 0u;
    control->vision_error_norm = 0;
    control->vision_sequence = 0u;
    control->have_sequence = 0u;
    control->have_valid_vision = 0u;
}

static void request_reverse_control_reset(car_control_t *control)
{
    if (control == 0) {
        return;
    }
    ++control->reverse_control_reset_generation;
    control->reverse_control_keep_seen = 0u;
    control->reverse_control_fresh_frames = 0u;
}

static void note_reverse_control_frame(car_control_t *control)
{
    if (control != 0 && control->reverse_control_keep_seen != 0u &&
        control->reverse_control_fresh_frames < CAR_RECOVERY_REQUIRED_FRAMES) {
        ++control->reverse_control_fresh_frames;
    }
}

void car_control_note_applied_outputs(car_control_t *control,
                                      uint8_t servo_frame_ready,
                                      uint8_t servo_output_active,
                                      uint16_t servo_applied_us,
                                      uint16_t motor_left_applied_permille,
                                      uint16_t motor_right_applied_permille)
{
    if (control == 0) {
        return;
    }
    control->applied_servo_frame_ready = servo_frame_ready != 0u ? 1u : 0u;
    control->applied_servo_output_active =
        servo_output_active != 0u ? 1u : 0u;
    control->applied_servo_pulse_us = servo_output_active != 0u
                                          ? servo_applied_us : 0u;
    control->applied_motor_left_permille = motor_left_applied_permille;
    control->applied_motor_right_permille = motor_right_applied_permille;
}

static void stop_outputs(car_control_t *control)
{
    stop_bench(control);
    control->bench_motor = CAR_MOTOR_NONE;
    control->bench_requested_duty_permille = 0u;
    control->bench_output_duty_permille = 0u;
    control->follow_output_duty_permille = 0u;
    control->road_drive_allowed = 0u;
    control->steering_session_active = 0u;
}

static void finish_vision_hold(car_control_t *control,
                               car_vision_hold_exit_cause_t cause)
{
    if (control == 0 || (control->vision_hold_active == 0u &&
                         control->vision_hold_expired == 0u)) {
        return;
    }
    control->vision_hold_active = 0u;
    control->vision_hold_expired = 0u;
    control->vision_hold_exit_cause = (uint8_t)cause;
    control->vision_hold_valid_count = 0u;
    control->vision_hold_have_sequence = 0u;
}

static uint8_t follow_startup_release_ready(const car_config_t *config,
                                           const car_inputs_t *inputs)
{
    if (config == 0 || inputs == 0) {
        return 0u;
    }
    if (config->follow_mode_compiled != 0u &&
        config->bench_mode_compiled == 0u &&
        config->debug_steering_mode_compiled == 0u) {
        return inputs->manual_button_startup_release_confirmed;
    }
    return inputs->pe7_high_stable;
}

static void enter_hard_fault(car_control_t *control, car_fault_t fault)
{
    uint8_t is_new_latch;
    if (control == 0 || fault == CAR_FAULT_NONE) {
        return;
    }

    /* A hard reason replaces an earlier soft receive reason. */
    is_new_latch = (uint8_t)(control->fault_latched == 0u ||
        is_hard_fault(control->fault) == 0u);
    if (control->fault_latched == 0u ||
        is_hard_fault(control->fault) == 0u) {
        control->fault = fault;
    }
    if (is_new_latch != 0u) {
        ++control->fault_generation;
        control->hard_ack_press_seen = 0u;
        control->rx_recovery_flush_required = 1u;
        request_reverse_control_reset(control);
    }
    control->fault_latched = 1u;
    finish_vision_hold(control, CAR_VISION_HOLD_EXIT_HARD_FAULT);
    control->manual_run_request = 0u;
    control->mode = CAR_MODE_FAULT_LATCHED;
    control->follow_session_active = 0u;
    control->recovery_active = 0u;
    control->recovery_frame_count = 0u;
    control->recovery_have_sequence = 0u;
    control->recovery_was_faulted = 0u;
    stop_outputs(control);
    clear_trusted_observation(control);
}

static void enter_soft_fault(car_control_t *control, car_fault_t fault)
{
    uint8_t same_unserviced_fault;

    if (control == 0 || fault == CAR_FAULT_NONE ||
        control->fault_latched != 0u) {
        return;
    }

    /* Level-triggered copies of the same sticky RX flag are one event. */
    same_unserviced_fault =
        (uint8_t)(control->rx_recovery_flush_required != 0u &&
                  control->recovery_active != 0u &&
                  control->fault == fault);
    if (same_unserviced_fault != 0u) {
        return;
    }

    control->fault = fault;
    finish_vision_hold(control, CAR_VISION_HOLD_EXIT_COMM_FAULT);
    control->fault_latched = 0u;
    ++control->fault_generation;
    if (fault != CAR_FAULT_SWITCH_RELEASE) {
        ++control->rx_fault_generation;
    }
    control->mode = CAR_MODE_RECOVERY_WAIT;
    control->recovery_active = 1u;
    control->recovery_was_faulted =
        fault == CAR_FAULT_SWITCH_RELEASE ? 0u : 1u;
    control->recovery_frame_count = 0u;
    control->recovery_have_sequence = 0u;
    control->rx_recovery_flush_required = 1u;
    request_reverse_control_reset(control);
    ++control->rx_error_count;
    stop_outputs(control);
    clear_trusted_observation(control);
}

static int16_t clamp_error(int16_t error_norm)
{
    if (error_norm > 1000) {
        return 1000;
    }
    if (error_norm < -1000) {
        return -1000;
    }
    return error_norm;
}

uint16_t car_debug_steering_target_us(int16_t error_norm)
{
    int32_t error = (int32_t)error_norm;
    int32_t pulse;
    int32_t endpoint_delta;
    int64_t offset;

    if (error > 1000) {
        error = 1000;
    } else if (error < -1000) {
        error = -1000;
    }

    endpoint_delta = error < 0
        ? (int32_t)CARTEST_SERVO_LEFT_US -
              (int32_t)CARTEST_SERVO_CENTER_US
        : (int32_t)CARTEST_SERVO_RIGHT_US -
              (int32_t)CARTEST_SERVO_CENTER_US;
    offset = ((int64_t)endpoint_delta *
              (int64_t)(error < 0 ? -error : error) *
              (int64_t)CARTEST_STEERING_GAIN_PERMILLE) / 1000000;
    pulse = (int32_t)CARTEST_SERVO_CENTER_US + (int32_t)offset;

    if (pulse < (int32_t)CARTEST_SERVO_MIN_US) {
        pulse = (int32_t)CARTEST_SERVO_MIN_US;
    } else if (pulse > (int32_t)CARTEST_SERVO_MAX_US) {
        pulse = (int32_t)CARTEST_SERVO_MAX_US;
    }
    return (uint16_t)pulse;
}

static uint16_t follow_servo_target(const car_config_t *config,
                                    int16_t error_norm)
{
    int32_t error = (int32_t)clamp_error(error_norm);
    int32_t endpoint_delta;
    int64_t offset;

    if (error == 0) {
        return config->servo_center_us;
    }

    endpoint_delta = error > 0
        ? (int32_t)config->follow_servo_right_us -
              (int32_t)config->servo_center_us
        : (int32_t)config->follow_servo_left_us -
              (int32_t)config->servo_center_us;
    offset = ((int64_t)endpoint_delta *
              (int64_t)(error > 0 ? error : -error) *
              (int64_t)config->follow_kp_permille) / 1000000;
    return (uint16_t)((int32_t)config->servo_center_us +
                      (int32_t)offset);
}

static void store_trusted_observation(car_control_t *control,
                                      const car_config_t *config,
                                      uint16_t sequence,
                                      uint8_t confidence,
                                      int16_t error_norm,
                                      uint32_t arrival_ms)
{
    control->vision_valid = 1u;
    control->vision_timeout = 0u;
    control->vision_confidence = confidence;
    control->vision_error_norm = clamp_error(error_norm);
    control->vision_sequence = sequence;
    control->have_sequence = 1u;
    control->have_valid_vision = 1u;
    control->vision_last_valid_ms = arrival_ms;
    if (config->follow_mode_compiled != 0u &&
        config->follow_calibration_ready != 0u) {
        control->follow_servo_target_us =
            follow_servo_target(config, control->vision_error_norm);
    }
    if (config->debug_steering_mode_compiled != 0u) {
        control->steering_servo_target_us =
            car_debug_steering_target_us(control->vision_error_norm);
    }
}

static void complete_observation_recovery(car_control_t *control,
                                          const car_config_t *config,
                                          uint32_t now_ms)
{
    control->recovery_frame_count = CAR_RECOVERY_REQUIRED_FRAMES;
    control->fault = CAR_FAULT_NONE;
    control->fault_latched = 0u;
    if (control->recovery_was_faulted != 0u) {
        ++control->auto_recovery_count;
    }
    control->recovery_was_faulted = 0u;

    if (control->follow_session_active != 0u &&
        config->follow_mode_compiled != 0u &&
        config->follow_calibration_ready != 0u &&
        config->motor_hardware_accepted != 0u &&
        control->manual_run_request != 0u &&
        (control->reverse_control_keep_seen == 0u ||
         control->reverse_control_fresh_frames < CAR_RECOVERY_REQUIRED_FRAMES)) {
        /* Keep the existing Follow session stopped while the independent
           reverse cache lease is established and three new frames arrive. */
        control->recovery_active = 1u;
        control->mode = CAR_MODE_RECOVERY_WAIT;
        control->road_drive_allowed = 0u;
        control->follow_output_duty_permille = 0u;
        return;
    }

    control->recovery_active = 0u;

    if (config->debug_steering_mode_compiled != 0u) {
        /* Steering resumes only after this fresh three-frame validation. */
        control->mode = CAR_MODE_DEBUG_STEERING_RUNNING;
        control->steering_session_active = 1u;
        control->steering_servo_target_us =
            car_debug_steering_target_us(control->vision_error_norm);
        control->road_drive_allowed = 0u;
    } else if (control->follow_session_active != 0u &&
        config->follow_mode_compiled != 0u &&
        config->follow_calibration_ready != 0u &&
        config->motor_hardware_accepted != 0u &&
        control->manual_run_request != 0u) {
        /* Third fresh observation already set the target; ramp starts at 0. */
        control->mode = CAR_MODE_FOLLOW_RUNNING;
        control->follow_output_duty_permille = 0u;
        control->follow_last_ramp_ms = now_ms;
        control->road_drive_allowed = 1u;
    } else {
        control->follow_session_active = 0u;
        control->mode = CAR_MODE_WAIT_START_EDGE;
        control->frame_edge_count = control->button_low_edge_count;
        control->road_drive_allowed = 0u;
    }
}

void car_control_init(car_control_t *control,
                      const car_config_t *config,
                      uint32_t now_ms,
                      uint8_t pe7_raw_high,
                      uint8_t watchdog_reset_detected)
{
    if (control == 0) {
        return;
    }
    *control = (car_control_t){0};
    control->road_drive_allowed = 0u;
    control->vision_timeout = 1u;
    control->debug_steering_mode_compiled =
        config != 0 ? config->debug_steering_mode_compiled : 0u;
    control->startup_release_seen =
        control->debug_steering_mode_compiled != 0u ||
        (config != 0 && config->follow_mode_compiled == 0u &&
         pe7_raw_high != 0u)
            ? 1u : 0u;
    control->startup_frame_gate_pending =
        control->debug_steering_mode_compiled != 0u ? 0u : 1u;
    control->switch_was_high = pe7_raw_high != 0u ? 1u : 0u;
    control->vision_last_valid_ms = now_ms;
    control->startup_release_ms = now_ms;
    control->reverse_control_reset_generation = 1u;
    control->follow_servo_target_us =
        config != 0 ? config->servo_center_us : 0u;
    control->steering_servo_target_us =
        (uint16_t)CARTEST_SERVO_CENTER_US;
    control->mode = control->debug_steering_mode_compiled != 0u
                        ? CAR_MODE_RECOVERY_WAIT
                        : (control->startup_release_seen != 0u
                               ? CAR_MODE_RECOVERY_WAIT
                               : CAR_MODE_WAIT_SWITCH_RELEASE);
    control->recovery_active =
        control->debug_steering_mode_compiled != 0u ||
        pe7_raw_high != 0u ? 1u : 0u;
    if (config_is_valid(config) == 0u) {
        enter_hard_fault(control, CAR_FAULT_CONFIG);
    }
    if (watchdog_reset_detected != 0u) {
        enter_hard_fault(control, CAR_FAULT_WATCHDOG_RESET);
    }
}

uint8_t car_control_vision_fresh(const car_control_t *control,
                                 uint32_t now_ms,
                                 uint32_t timeout_ms)
{
    if (control == 0 || timeout_ms == 0u || timeout_ms > 0x7fffffffu ||
        control->have_valid_vision == 0u || control->vision_valid == 0u) {
        return 0u;
    }
    return (uint32_t)(now_ms - control->vision_last_valid_ms) < timeout_ms
               ? 1u
               : 0u;
}

void car_control_note_fault(car_control_t *control, car_fault_t fault)
{
    if (control == 0 || fault == CAR_FAULT_NONE) {
        return;
    }
    if (fault == CAR_FAULT_SWITCH_RELEASE &&
        control->debug_steering_mode_compiled != 0u) {
        return;
    }
    control->steering_session_active = 0u;
    if (fault == CAR_FAULT_SWITCH_RELEASE) {
        control->follow_session_active = 0u;
    }
    if (is_hard_fault(fault) != 0u) {
        enter_hard_fault(control, fault);
    } else {
        enter_soft_fault(control, fault);
    }
}

void car_control_note_switch_release_isr(car_control_t *control)
{
    if (control != 0) {
        if (control->debug_steering_mode_compiled != 0u) {
            return;
        }
        control->isr_stop_request = 1u;
        control->isr_release_seen = 1u;
        ++control->isr_switch_generation;
        /* A momentary high cancels the run session even if it falls before main. */
        control->follow_session_active = 0u;
        control->follow_output_duty_permille = 0u;
        control->steering_session_active = 0u;
        control->road_drive_allowed = 0u;
    }
}

void car_control_note_fast_stop_isr(car_control_t *control, car_fault_t fault)
{
    uint8_t old_fault;
    if (control == 0 || fault == CAR_FAULT_NONE) {
        return;
    }
    if (fault == CAR_FAULT_SWITCH_RELEASE) {
        car_control_note_switch_release_isr(control);
        return;
    }

    old_fault = control->isr_stop_fault;
    ++control->isr_uart_generation;
    /* A servo-only session is never resumed after an interrupt-level fault. */
    control->steering_session_active = 0u;
    if (old_fault == CAR_FAULT_NONE ||
        (is_hard_fault(fault) != 0u &&
         is_hard_fault((car_fault_t)old_fault) == 0u)) {
        control->isr_stop_fault = (uint8_t)fault;
    }
    if (is_hard_fault(fault) != 0u) {
        control->follow_session_active = 0u;
    }
}

car_fault_t car_control_take_fast_stop_isr(car_control_t *control)
{
    car_fault_t fault;
    uint8_t release_seen;
    if (control == 0) {
        return CAR_FAULT_NONE;
    }
    fault = (car_fault_t)control->isr_stop_fault;
    control->isr_stop_fault = CAR_FAULT_NONE;
    control->isr_stop_request = 0u;
    release_seen = control->isr_release_seen;
    control->isr_release_seen = 0u;
    if (release_seen != 0u &&
        control->debug_steering_mode_compiled == 0u) {
        control->follow_session_active = 0u;
        control->steering_session_active = 0u;
        if (fault == CAR_FAULT_NONE) {
            fault = CAR_FAULT_SWITCH_RELEASE;
        }
    }
    return fault;
}

void car_control_rx_recovery_flush_complete(car_control_t *control)
{
    if (control == 0) {
        return;
    }
    control->rx_recovery_flush_required = 0u;
    control->vision_have_frame_sequence = 0u;
    control->vision_last_frame_sequence = 0u;
    control->vision_last_frame_alive_ms = 0u;
    if (control->fault_latched != 0u) {
        control->recovery_active = 0u;
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        return;
    }
    if (control->recovery_active != 0u) {
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        control->mode = CAR_MODE_RECOVERY_WAIT;
    }
}

void car_control_reverse_control_reset_started(car_control_t *control)
{
    if (control != 0) {
        control->reverse_control_keep_seen = 0u;
        control->reverse_control_fresh_frames = 0u;
        control->reverse_reset_transport_gate_open = 0u;
        control->reverse_transport_recovery_frame_count = 0u;
        control->reverse_transport_have_sequence = 0u;
        control->reverse_transport_last_sequence = 0u;
        control->reverse_transport_last_arrival_ms = 0u;
    }
}

void car_control_reverse_control_reset_phase_complete(car_control_t *control)
{
    if (control != 0 && control->reverse_reset_transport_gate_open == 0u) {
        control->reverse_reset_transport_gate_open = 1u;
        control->reverse_transport_recovery_frame_count = 0u;
        control->reverse_transport_have_sequence = 0u;
        control->reverse_transport_last_sequence = 0u;
        control->reverse_transport_last_arrival_ms = 0u;
    }
}

void car_control_reverse_control_keep_started(car_control_t *control)
{
    if (control != 0) {
        control->reverse_control_keep_seen = 1u;
        control->reverse_control_fresh_frames = 0u;
    }
}

void car_control_on_vision_frame_at(car_control_t *control,
                                    const car_config_t *config,
                                    uint16_t sequence,
                                    uint8_t valid,
                                    uint8_t confidence,
                                    int16_t error_norm,
                                    uint32_t arrival_ms,
                                    uint32_t now_ms,
                                    uint8_t pe7_raw_high)
{
    uint16_t delta;
    uint32_t interval;
    uint32_t queue_age;

    if (control == 0) {
        return;
    }

    /* CRC/format-validated data remains visible during every control fault. */
    control->latest_rx_valid = valid != 0u ? 1u : 0u;
    control->latest_rx_confidence = confidence;
    control->latest_rx_error_norm = error_norm;
    control->latest_rx_sequence = sequence;
    control->latest_rx_arrival_ms = arrival_ms;
    control->latest_rx_have_frame = 1u;
    ++control->latest_rx_frame_count;
    ++control->rx_frame_count;

    if (config_is_valid(config) == 0u) {
        enter_hard_fault(control, CAR_FAULT_CONFIG);
        return;
    }

    if (control->fault_latched != 0u ||
        control->mode == CAR_MODE_FAULT_LATCHED) {
        return;
    }

    queue_age = (uint32_t)(now_ms - arrival_ms);
    if (queue_age >= CAR_RECOVERY_MAX_QUEUE_MS) {
        car_control_note_fault(control, CAR_FAULT_RX_STALE);
        return;
    }

    /* Transport liveness is separate from route validity. Only a fresh,
       strictly forward sequence from the CRC/format-checked parser counts. */
    if (control->vision_have_frame_sequence != 0u) {
        delta = (uint16_t)(sequence - control->vision_last_frame_sequence);
        if (delta == 0u || delta >= 0x8000u) {
            ++control->sequence_error_count;
            car_control_note_fault(control, CAR_FAULT_SEQUENCE);
            return;
        }
        if (config->follow_mode_compiled != 0u &&
            (uint32_t)(arrival_ms - control->vision_last_frame_alive_ms) >=
                config->vision_timeout_ms) {
            /* A late frame cannot retroactively erase the transport outage.
               Route validity is deliberately not used for this timeout. */
            car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
            return;
        }
    }
    control->vision_have_frame_sequence = 1u;
    control->vision_last_frame_sequence = sequence;
    control->vision_last_frame_alive_ms = arrival_ms;

    if (control->reverse_reset_transport_gate_open != 0u) {
        if (control->reverse_transport_have_sequence != 0u) {
            interval = (uint32_t)(arrival_ms -
                                  control->reverse_transport_last_arrival_ms);
            if (interval < CAR_RECOVERY_MAX_GAP_MS) {
                if (control->reverse_transport_recovery_frame_count <
                    CAR_RECOVERY_REQUIRED_FRAMES) {
                    ++control->reverse_transport_recovery_frame_count;
                }
            } else {
                control->reverse_transport_recovery_frame_count = 1u;
            }
        } else {
            control->reverse_transport_have_sequence = 1u;
            control->reverse_transport_recovery_frame_count = 1u;
        }
        control->reverse_transport_last_sequence = sequence;
        control->reverse_transport_last_arrival_ms = arrival_ms;
    }

    if (control->startup_release_seen == 0u) {
        (void)pe7_raw_high;
        return;
    }
    if (control->startup_frame_gate_pending != 0u) {
        /* Flush establishes the startup boundary; this is a fresh frame after it. */
        control->startup_frame_gate_pending = 0u;
    }

    if (valid == 0u) {
        ++control->invalid_vision_count;
        control->vision_valid = 0u;
        control->vision_timeout = 0u;
        if (control->recovery_active != 0u ||
            control->mode == CAR_MODE_RECOVERY_WAIT ||
            control->mode == CAR_MODE_WAIT_NEW_FRAME) {
            control->recovery_frame_count = 0u;
            control->recovery_have_sequence = 0u;
        }
        if (control->vision_hold_active != 0u ||
            control->vision_hold_expired != 0u) {
            control->vision_hold_valid_count = 0u;
            control->vision_hold_have_sequence = 0u;
            return;
        }
        if (control->mode == CAR_MODE_FOLLOW_RUNNING &&
            control->follow_session_active != 0u) {
            if (control->manual_run_request == 0u ||
                control->applied_servo_frame_ready == 0u ||
                control->applied_servo_output_active == 0u ||
                control->applied_servo_pulse_us < config->servo_min_us ||
                control->applied_servo_pulse_us > config->servo_max_us ||
                control->applied_motor_left_permille == 0u ||
                control->applied_motor_right_permille == 0u ||
                control->applied_motor_left_permille >
                    config->follow_max_duty_permille ||
                control->applied_motor_right_permille >
                    config->follow_max_duty_permille) {
                car_control_note_fault(control, CAR_FAULT_VISION_INVALID);
                return;
            }
            control->mode = CAR_MODE_FOLLOW_VISION_HOLD;
            control->vision_hold_active = 1u;
            control->vision_hold_expired = 0u;
            control->vision_hold_restart_required = 0u;
            control->vision_hold_valid_count = 0u;
            control->vision_hold_have_sequence = 0u;
            /* This is the route-trust timestamp that defines the 1000 ms
               deadline, not the later valid=0 detector-arrival time. */
            control->vision_hold_started_ms = control->vision_last_valid_ms;
            control->vision_hold_exit_cause = CAR_VISION_HOLD_EXIT_NONE;
            ++control->vision_hold_entries;
            control->vision_hold_servo_pulse_us =
                control->applied_servo_pulse_us;
            control->vision_hold_motor_left_permille =
                control->applied_motor_left_permille;
            control->vision_hold_motor_right_permille =
                control->applied_motor_right_permille;
            control->vision_hold_base_duty_permille =
                control->follow_output_duty_permille;
            control->road_drive_allowed = 1u;
            return;
        }
        if (control->mode == CAR_MODE_BENCH_RUNNING ||
            control->steering_session_active != 0u) {
            car_control_note_fault(control, CAR_FAULT_VISION_INVALID);
        }
        /* A line-absent observation while idle/recovering is fresh transport,
           not a request to reset the reverse cache repeatedly. */
        return;
    }

    if (control->vision_hold_active != 0u ||
        control->vision_hold_expired != 0u) {
        if (control->manual_run_request == 0u) {
            car_control_note_fault(control, CAR_FAULT_SWITCH_RELEASE);
            return;
        }
        if (control->vision_hold_have_sequence != 0u) {
            interval = (uint32_t)(arrival_ms -
                                  control->vision_hold_last_arrival_ms);
            if (interval < CAR_RECOVERY_MAX_GAP_MS &&
                control->vision_hold_valid_count < CAR_RECOVERY_REQUIRED_FRAMES) {
                ++control->vision_hold_valid_count;
            } else {
                control->vision_hold_valid_count = 1u;
            }
        } else {
            control->vision_hold_have_sequence = 1u;
            control->vision_hold_valid_count = 1u;
        }
        control->vision_hold_last_sequence = sequence;
        control->vision_hold_last_arrival_ms = arrival_ms;
        note_reverse_control_frame(control);
        if (control->vision_hold_valid_count < CAR_RECOVERY_REQUIRED_FRAMES) {
            return;
        }

        interval = (uint32_t)(now_ms - control->vision_last_valid_ms);
        if (interval >= CAR_CONTROL_VISION_HOLD_MS) {
            /* Deadline has already stopped outputs. Recovery re-enters the
               normal first-servo-frame / motor-ramp gate from zero. */
            control->vision_hold_restart_required = 1u;
            control->follow_output_duty_permille = 0u;
        } else {
            control->follow_output_duty_permille =
                control->vision_hold_base_duty_permille;
        }
        store_trusted_observation(control, config, sequence, confidence,
                                  error_norm, arrival_ms);
        control->vision_hold_active = 0u;
        control->vision_hold_expired = 0u;
        control->vision_hold_exit_cause = CAR_VISION_HOLD_EXIT_RECOVERED;
        ++control->auto_recovery_count;
        control->mode = CAR_MODE_FOLLOW_RUNNING;
        control->recovery_active = 0u;
        control->recovery_frame_count = CAR_RECOVERY_REQUIRED_FRAMES;
        control->recovery_have_sequence = 0u;
        control->follow_last_ramp_ms = now_ms;
        control->follow_session_active = 1u;
        control->road_drive_allowed = 1u;
        return;
    }

    if (control->recovery_active != 0u ||
        control->mode == CAR_MODE_RECOVERY_WAIT ||
        control->mode == CAR_MODE_WAIT_NEW_FRAME) {
        if (control->rx_recovery_flush_required != 0u) {
            return;
        }

        if (control->recovery_have_sequence != 0u) {
            delta = (uint16_t)(sequence - control->recovery_sequence);
            interval = (uint32_t)(arrival_ms -
                                  control->recovery_last_arrival_ms);
            if (delta == 0u || delta >= 0x8000u) {
                ++control->sequence_error_count;
                car_control_note_fault(control, CAR_FAULT_SEQUENCE);
                return;
            }
            if (interval >= CAR_RECOVERY_MAX_GAP_MS) {
                car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
                return;
            }
            if (control->recovery_frame_count < CAR_RECOVERY_REQUIRED_FRAMES) {
                ++control->recovery_frame_count;
            }
        } else {
            control->recovery_have_sequence = 1u;
            control->recovery_frame_count = 1u;
        }

        note_reverse_control_frame(control);

        control->recovery_sequence = sequence;
        control->recovery_last_arrival_ms = arrival_ms;
        if (control->recovery_frame_count < CAR_RECOVERY_REQUIRED_FRAMES) {
            return;
        }

        store_trusted_observation(control, config, sequence, confidence,
                                  error_norm, arrival_ms);
        if (control->mode == CAR_MODE_WAIT_NEW_FRAME) {
            control->ack_waiting_new_frame = 0u;
        }
        complete_observation_recovery(control, config, now_ms);
        return;
    }

    if (config->follow_mode_compiled == 0u &&
        control->have_valid_vision != 0u) {
        interval = (uint32_t)(arrival_ms - control->vision_last_valid_ms);
        if (interval >= config->vision_timeout_ms) {
            car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
            return;
        }
    }

    note_reverse_control_frame(control);
    store_trusted_observation(control, config, sequence, confidence,
                              error_norm, arrival_ms);
}

void car_control_on_vision_frame(car_control_t *control,
                                 const car_config_t *config,
                                 uint16_t sequence,
                                 uint8_t valid,
                                 uint8_t confidence,
                                 int16_t error_norm,
                                 uint32_t arrival_ms,
                                 uint8_t pe7_raw_high)
{
    car_control_on_vision_frame_at(control, config, sequence, valid,
                                   confidence, error_norm, arrival_ms,
                                   arrival_ms, pe7_raw_high);
}

static uint32_t reject_command_stop(car_control_t *control)
{
    uint32_t effects = CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    uint8_t was_bench_running;
    uint8_t was_follow_session;

    if (control == 0) {
        return CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    }
    ++control->command_reject_count;
    was_bench_running =
        (uint8_t)(control->mode == CAR_MODE_BENCH_RUNNING);
    was_follow_session = (uint8_t)(
        control->mode == CAR_MODE_FOLLOW_RUNNING ||
        control->follow_session_active != 0u);

    stop_outputs(control);
    control->follow_session_active = 0u;
    control->bench_motor = CAR_MOTOR_NONE;
    control->bench_requested_duty_permille = 0u;
    if (control->fault_latched != 0u) {
        control->mode = CAR_MODE_FAULT_LATCHED;
    } else if (was_bench_running != 0u) {
        /* A rejected command ends this one-shot bench run. */
        control->mode = CAR_MODE_BENCH_COMPLETE;
        ++control->fault_generation;
    } else if (was_follow_session != 0u) {
        /* Re-arm only after a new flush, three frames, and a new PE7 press. */
        control->mode = CAR_MODE_RECOVERY_WAIT;
        control->recovery_active = 1u;
        control->recovery_was_faulted = 0u;
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        control->rx_recovery_flush_required = 1u;
        ++control->fault_generation;
        clear_trusted_observation(control);
        effects |= CAR_EFFECT_CLEAR_RX | CAR_EFFECT_RESET_PARSER;
    }
    return effects;
}

static uint32_t accept_ack(car_control_t *control,
                           const car_config_t *config,
                           const car_inputs_t *inputs,
                           uint8_t reset_sequence)
{
    car_fault_t hard_reason = CAR_FAULT_NONE;

    if (config_is_valid(config) == 0u) {
        enter_hard_fault(control, CAR_FAULT_CONFIG);
        ++control->command_reject_count;
        return CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    }
    if (inputs != 0) {
        if (inputs->loop_gap_fault != 0u) {
            hard_reason = CAR_FAULT_LOOP_GAP;
        } else if (inputs->encoder_fault != 0u) {
            hard_reason = CAR_FAULT_ENCODER_INIT;
        } else if (inputs->clock_or_timer_fault != 0u) {
            hard_reason = CAR_FAULT_CLOCK;
        }
    }
    if (hard_reason == CAR_FAULT_NONE &&
        control->isr_stop_fault != CAR_FAULT_NONE &&
        is_hard_fault((car_fault_t)control->isr_stop_fault) != 0u) {
        hard_reason = (car_fault_t)control->isr_stop_fault;
    }
    if (hard_reason != CAR_FAULT_NONE) {
        enter_hard_fault(control, hard_reason);
        ++control->command_reject_count;
        return CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    }
    if (inputs == 0 || inputs->pe7_raw_high == 0u ||
        inputs->pe7_high_stable == 0u ||
        control->isr_stop_request != 0u || control->isr_release_seen != 0u ||
        control->mode == CAR_MODE_BENCH_RUNNING ||
        control->fault_latched != 0u ||
        control->mode == CAR_MODE_FAULT_LATCHED) {
        return reject_command_stop(control);
    }

    stop_outputs(control);
    control->fault = CAR_FAULT_NONE;
    control->fault_latched = 0u;
    control->follow_session_active = 0u;
    control->hard_ack_press_seen = 0u;
    control->ack_waiting_new_frame = 1u;
    control->ack_seen_low = 0u;
    control->bench_motor = CAR_MOTOR_NONE;
    control->bench_requested_duty_permille = 0u;
    clear_trusted_observation(control);
    control->recovery_active = 1u;
    control->recovery_was_faulted = 0u;
    control->recovery_frame_count = 0u;
    control->recovery_have_sequence = 0u;
    control->rx_recovery_flush_required = 1u;
    request_reverse_control_reset(control);
    control->mode = CAR_MODE_RECOVERY_WAIT;
    if (reset_sequence != 0u) {
        control->have_sequence = 0u;
        control->vision_sequence = 0u;
        ++control->camera_restart_count;
    }
    return CAR_EFFECT_CLEAR_RX | CAR_EFFECT_RESET_PARSER |
           CAR_EFFECT_STOP_OUTPUTS;
}

uint32_t car_control_command(car_control_t *control,
                             const car_config_t *config,
                             const car_inputs_t *inputs,
                             car_command_t command,
                             uint8_t selected_motor,
                             uint8_t selected_input,
                             uint16_t requested_duty_permille)
{
    if (control == 0) {
        return CAR_EFFECT_STOP_OUTPUTS;
    }
    if (inputs == 0) {
        return reject_command_stop(control);
    }
    control->command_effects = CAR_EFFECT_NONE;

    if (command != CAR_CMD_NONE && command != CAR_CMD_NORMAL_STOP &&
        config_is_valid(config) == 0u) {
        enter_hard_fault(control, CAR_FAULT_CONFIG);
        return CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    }

    switch (command) {
    case CAR_CMD_NONE:
        return CAR_EFFECT_NONE;

    case CAR_CMD_LOCAL_ACK:
        control->command_effects = accept_ack(control, config, inputs, 0u);
        return control->command_effects;

    case CAR_CMD_CAMERA_RESTART_ACK:
        control->command_effects = accept_ack(control, config, inputs, 1u);
        return control->command_effects;

    case CAR_CMD_NORMAL_STOP:
        finish_vision_hold(control, CAR_VISION_HOLD_EXIT_MANUAL_STOP);
        request_reverse_control_reset(control);
        stop_outputs(control);
        control->follow_session_active = 0u;
        control->manual_run_request = 0u;
        control->hard_ack_press_seen = 0u;
        control->fault = control->fault_latched != 0u
                             ? control->fault : CAR_FAULT_NONE;
        control->recovery_active = control->fault_latched == 0u ? 1u : 0u;
        control->recovery_was_faulted = 0u;
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        control->rx_recovery_flush_required = 1u;
        clear_trusted_observation(control);
        control->mode = control->fault_latched != 0u
                            ? CAR_MODE_FAULT_LATCHED
                            : CAR_MODE_RECOVERY_WAIT;
        control->command_effects = CAR_EFFECT_CLEAR_RX |
                                   CAR_EFFECT_RESET_PARSER |
                                   CAR_EFFECT_STOP_OUTPUTS |
                                   CAR_EFFECT_DISABLE_SERVO;
        return control->command_effects;

    case CAR_CMD_SELECT_BENCH_OUTPUT:
        if (config_is_valid(config) == 0u ||
            config->bench_mode_compiled == 0u ||
            config->motor_hardware_accepted == 0u ||
            bench_channel_mask_for_selection(selected_motor,
                                             selected_input) == 0u ||
            requested_duty_permille == 0u ||
            requested_duty_permille > config->bench_duty_limit_permille ||
            inputs->pe7_raw_high == 0u || inputs->pe7_high_stable == 0u ||
            control->mode == CAR_MODE_BENCH_RUNNING) {
            return reject_command_stop(control);
        }
        control->command_effects = accept_ack(control, config, inputs, 0u);
        if ((control->command_effects & CAR_EFFECT_CLEAR_RX) == 0u) {
            return control->command_effects;
        }
        control->bench_input = selected_input;
        control->bench_requested_duty_permille = requested_duty_permille;
        control->bench_motor = selected_motor;
        control->profile_command_issued = 1u;
        return control->command_effects;

    default:
        ++control->command_reject_count;
        enter_hard_fault(control, CAR_FAULT_CONFIG);
        return CAR_EFFECT_STOP_OUTPUTS | CAR_EFFECT_DISABLE_SERVO;
    }
}

static uint8_t bench_start_gate_clear(const car_control_t *control,
                                      const car_config_t *config,
                                      const car_inputs_t *inputs,
                                      uint8_t rx_path_healthy)
{
    if (control == 0 || config == 0 || inputs == 0 ||
        config_is_valid(config) == 0u || config->bench_mode_compiled == 0u ||
        config->motor_hardware_accepted == 0u ||
        config->follow_mode_compiled != 0u ||
        config->debug_steering_mode_compiled != 0u ||
        rx_path_healthy == 0u || control->fault != CAR_FAULT_NONE ||
        control->fault_latched != 0u ||
        control->isr_stop_request != 0u ||
        control->isr_stop_fault != CAR_FAULT_NONE ||
        control->isr_release_seen != 0u ||
        control->rx_recovery_flush_required != 0u ||
        control->mode != CAR_MODE_WAIT_START_EDGE ||
        control->startup_release_seen == 0u ||
        control->startup_frame_gate_pending != 0u ||
        control->recovery_active != 0u ||
        control->recovery_frame_count != CAR_RECOVERY_REQUIRED_FRAMES ||
        control->vision_valid == 0u || control->latest_rx_valid == 0u ||
        car_control_vision_fresh(control, inputs->now_ms,
                                 config->vision_timeout_ms) == 0u ||
        inputs->pe7_raw_high == 0u || inputs->pe7_high_stable == 0u ||
        inputs->pe7_low_stable != 0u || inputs->loop_gap_fault != 0u ||
        inputs->uart_fault != 0u || inputs->rx_overflow_fault != 0u ||
        inputs->rx_stale_fault != 0u || inputs->protocol_fault != 0u ||
        inputs->encoder_fault != 0u ||
        inputs->clock_or_timer_fault != 0u ||
        control->road_drive_allowed != 0u ||
        control->follow_session_active != 0u ||
        control->bench_output_duty_permille != 0u) {
        return 0u;
    }
    return 1u;
}

uint32_t car_control_auto_select_bench_output(
    car_control_t *control,
    const car_config_t *config,
    const car_inputs_t *inputs,
    uint8_t rx_path_healthy)
{
    uint32_t effects;

    if (control == 0 || control->profile_command_issued != 0u ||
        bench_start_gate_clear(control, config, inputs,
                               rx_path_healthy) == 0u) {
        return CAR_EFFECT_NONE;
    }

    effects = car_control_command(
        control, config, inputs, CAR_CMD_SELECT_BENCH_OUTPUT,
        (uint8_t)CARTEST_BENCH_MOTOR, (uint8_t)CARTEST_BENCH_INPUT,
        (uint16_t)CARTEST_BENCH_DUTY_PERMILLE);
    if ((effects & (CAR_EFFECT_CLEAR_RX | CAR_EFFECT_STOP_OUTPUTS)) ==
        (CAR_EFFECT_CLEAR_RX | CAR_EFFECT_STOP_OUTPUTS)) {
        control->profile_command_issued = 1u;
    }
    return effects;
}

uint8_t car_control_bench_ready(const car_control_t *control,
                                const car_config_t *config,
                                const car_inputs_t *inputs,
                                uint8_t rx_path_healthy)
{
    if (control == 0 || control->profile_command_issued == 0u ||
        bench_start_gate_clear(control, config, inputs,
                               rx_path_healthy) == 0u ||
        control->bench_motor != (uint8_t)CARTEST_BENCH_MOTOR ||
        control->bench_input != (uint8_t)CARTEST_BENCH_INPUT ||
        control->bench_requested_duty_permille !=
            (uint16_t)CARTEST_BENCH_DUTY_PERMILLE) {
        return 0u;
    }
    return 1u;
}

static uint8_t hard_inputs_clear(const car_control_t *control,
                                 const car_config_t *config,
                                 const car_inputs_t *inputs)
{
    if (config_is_valid(config) == 0u || inputs == 0 ||
        inputs->loop_gap_fault != 0u || inputs->encoder_fault != 0u ||
        inputs->clock_or_timer_fault != 0u ||
        (control->isr_stop_fault != CAR_FAULT_NONE &&
         is_hard_fault((car_fault_t)control->isr_stop_fault) != 0u)) {
        return 0u;
    }
    return 1u;
}

static void acknowledge_hard_fault(car_control_t *control,
                                   const car_config_t *config,
                                   const car_inputs_t *inputs)
{
    if (hard_inputs_clear(control, config, inputs) == 0u ||
        inputs->pe7_raw_high == 0u || inputs->pe7_high_stable == 0u) {
        control->hard_ack_press_seen = 0u;
        return;
    }
    control->fault = CAR_FAULT_NONE;
    control->fault_latched = 0u;
    control->follow_session_active = 0u;
    control->hard_ack_press_seen = 0u;
    control->recovery_active = 1u;
    control->recovery_was_faulted = 0u;
    control->recovery_frame_count = 0u;
    control->recovery_have_sequence = 0u;
    control->rx_recovery_flush_required = 1u;
    control->mode = CAR_MODE_RECOVERY_WAIT;
    stop_outputs(control);
    clear_trusted_observation(control);
}

static void update_button_state(car_control_t *control,
                                const car_inputs_t *inputs,
                                uint8_t *pressed_edge,
                                uint8_t *released_edge)
{
    *pressed_edge = 0u;
    *released_edge = 0u;
    control->switch_was_high = inputs->pe7_raw_high != 0u ? 1u : 0u;

    /* Debounce settles after the EXTI edge; qualify the stable transition. */
    if (control->stable_switch_known == 0u) {
        if (inputs->pe7_high_stable != 0u) {
            control->stable_switch_known = 1u;
            control->stable_switch_high = 1u;
        } else if (inputs->pe7_low_stable != 0u) {
            control->stable_switch_known = 1u;
            control->stable_switch_high = 0u;
        }
        return;
    }

    if (inputs->pe7_low_stable != 0u &&
        control->stable_switch_high != 0u) {
        control->stable_switch_high = 0u;
        ++control->button_low_edge_count;
        *pressed_edge = 1u;
    } else if (inputs->pe7_high_stable != 0u &&
               control->stable_switch_high == 0u) {
        control->stable_switch_high = 1u;
        *released_edge = 1u;
    }
}

void car_control_tick(car_control_t *control,
                      const car_config_t *config,
                      const car_inputs_t *inputs)
{
    uint8_t raw_high;
    uint8_t pressed_edge;
    uint8_t released_edge;
    uint32_t elapsed;
    uint32_t steps;
    uint32_t ramped;

    if (control == 0 || inputs == 0) {
        return;
    }
    control->manual_run_request = inputs->manual_run_request != 0u ? 1u : 0u;
    control->road_drive_allowed = 0u;
    raw_high = inputs->pe7_raw_high != 0u ? 1u : 0u;
    update_button_state(control, inputs, &pressed_edge, &released_edge);

    if (config_is_valid(config) == 0u) {
        enter_hard_fault(control, CAR_FAULT_CONFIG);
        control->bench_output_duty_permille = 0u;
        control->follow_output_duty_permille = 0u;
        return;
    }

    /* The production acceptance constant defaults false. If an unexpected
       state claims motor drive while locked, clear software outputs without
       creating a recurring fault that would starve diagnostics. */
    if (config->motor_hardware_accepted == 0u &&
        (control->mode == CAR_MODE_BENCH_RUNNING ||
         control->mode == CAR_MODE_FOLLOW_RUNNING)) {
        stop_outputs(control);
        control->follow_session_active = 0u;
        control->bench_motor = CAR_MOTOR_NONE;
        control->bench_requested_duty_permille = 0u;
        control->mode = CAR_MODE_WAIT_START_EDGE;
        control->frame_edge_count = control->button_low_edge_count;
    }

    /* Hard faults always win even if an RX error is present in the same tick. */
    if (inputs->loop_gap_fault != 0u) {
        enter_hard_fault(control, CAR_FAULT_LOOP_GAP);
    }
    if (inputs->encoder_fault != 0u) {
        enter_hard_fault(control, CAR_FAULT_ENCODER_INIT);
    }
    if (inputs->clock_or_timer_fault != 0u) {
        enter_hard_fault(control, CAR_FAULT_CLOCK);
    }
    if (control->fault_latched == 0u) {
        if (inputs->uart_fault != 0u) {
            enter_soft_fault(control, CAR_FAULT_UART);
        }
        if (inputs->rx_overflow_fault != 0u) {
            enter_soft_fault(control, CAR_FAULT_RX_OVERFLOW);
        }
        if (inputs->rx_stale_fault != 0u) {
            enter_soft_fault(control, CAR_FAULT_RX_STALE);
        }
        if (inputs->protocol_fault != 0u) {
            enter_soft_fault(control, CAR_FAULT_PROTOCOL);
        }
    }

    if (raw_high != 0u && control->steering_session_active != 0u) {
        car_control_note_fault(control, CAR_FAULT_SWITCH_RELEASE);
    }

    if (control->debug_steering_mode_compiled == 0u &&
        control->startup_release_seen == 0u &&
        (config->follow_mode_compiled != 0u || raw_high != 0u) &&
        follow_startup_release_ready(config, inputs) != 0u) {
        request_reverse_control_reset(control);
        control->startup_release_seen = 1u;
        control->startup_frame_gate_pending = 1u;
        control->startup_release_ms = inputs->now_ms;
        /* Discard frames and partial packets received before initial arming. */
        control->rx_recovery_flush_required = 1u;
        control->recovery_active = 1u;
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        control->mode = CAR_MODE_RECOVERY_WAIT;
        control->frame_edge_count = control->button_low_edge_count;
    }

    if (control->debug_steering_mode_compiled == 0u &&
        control->mode == CAR_MODE_WAIT_SWITCH_RELEASE &&
        (config->follow_mode_compiled != 0u || raw_high != 0u) &&
        follow_startup_release_ready(config, inputs) != 0u) {
        request_reverse_control_reset(control);
        control->mode = CAR_MODE_RECOVERY_WAIT;
        control->recovery_active = 1u;
        control->recovery_frame_count = 0u;
        control->recovery_have_sequence = 0u;
        control->startup_release_seen = 1u;
        control->startup_frame_gate_pending = 1u;
        control->startup_release_ms = inputs->now_ms;
    }

    if (control->fault_latched != 0u ||
        control->mode == CAR_MODE_FAULT_LATCHED) {
        control->mode = CAR_MODE_FAULT_LATCHED;
        if (pressed_edge != 0u && inputs->pe7_low_stable != 0u) {
            control->hard_ack_press_seen = 1u;
        }
        if (control->hard_ack_press_seen != 0u && released_edge != 0u &&
            inputs->pe7_high_stable != 0u) {
            acknowledge_hard_fault(control, config, inputs);
        }
        control->bench_output_duty_permille = 0u;
        control->follow_output_duty_permille = 0u;
        control->road_drive_allowed = 0u;
        return;
    }

    if (control->mode == CAR_MODE_WAIT_NEW_FRAME && raw_high == 0u) {
        control->ack_seen_low = 1u;
    }
    if (control->mode == CAR_MODE_WAIT_START_RELEASE && raw_high != 0u &&
        inputs->pe7_high_stable != 0u) {
        control->mode = CAR_MODE_WAIT_START_EDGE;
    }

    if (control->mode == CAR_MODE_BENCH_RUNNING) {
        if (raw_high != 0u) {
            car_control_note_fault(control, CAR_FAULT_SWITCH_RELEASE);
        } else if (car_control_vision_fresh(control, inputs->now_ms,
                   config->vision_timeout_ms) == 0u) {
            car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
        } else {
            elapsed = (uint32_t)(inputs->now_ms - control->bench_started_ms);
            if (elapsed >= config->bench_run_limit_ms) {
                stop_bench(control);
                control->mode = CAR_MODE_BENCH_COMPLETE;
                control->bench_motor = CAR_MOTOR_NONE;
                control->bench_requested_duty_permille = 0u;
            } else {
                if ((uint32_t)(inputs->now_ms - control->last_ramp_ms) >=
                    CAR_RAMP_PERIOD_MS) {
                    steps = (uint32_t)(inputs->now_ms - control->last_ramp_ms) /
                            CAR_RAMP_PERIOD_MS;
                    ramped = control->bench_output_duty_permille +
                             steps * config->bench_ramp_step_permille;
                    control->last_ramp_ms += steps * CAR_RAMP_PERIOD_MS;
                    if (ramped > control->bench_requested_duty_permille) {
                        ramped = control->bench_requested_duty_permille;
                    }
                    if (ramped > CAR_HARD_BENCH_LIMIT_PM) {
                        ramped = CAR_HARD_BENCH_LIMIT_PM;
                    }
                    control->bench_output_duty_permille = (uint16_t)ramped;
                }

                if (control->bench_output_duty_permille >=
                    control->bench_requested_duty_permille) {
                    if (control->bench_target_hold_active == 0u) {
                        /* This records the software duty command, not a pin waveform. */
                        control->bench_target_reached_ms = inputs->now_ms;
                        control->bench_target_hold_active = 1u;
                    } else if ((uint32_t)(inputs->now_ms -
                                          control->bench_target_reached_ms) >=
                               CAR_BENCH_TARGET_HOLD_MS) {
                        stop_bench(control);
                        control->mode = CAR_MODE_BENCH_COMPLETE;
                        control->bench_motor = CAR_MOTOR_NONE;
                        control->bench_requested_duty_permille = 0u;
                    }
                }
            }
        }
    }

    if ((control->mode == CAR_MODE_FOLLOW_RUNNING ||
         control->mode == CAR_MODE_FOLLOW_VISION_HOLD) &&
        control->follow_session_active != 0u &&
        control->vision_have_frame_sequence != 0u &&
        (uint32_t)(inputs->now_ms - control->vision_last_frame_alive_ms) >=
            config->vision_timeout_ms) {
        control->vision_timeout = 1u;
        car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
    } else if (((control->mode == CAR_MODE_BENCH_RUNNING ||
                 control->mode == CAR_MODE_DEBUG_STEERING_RUNNING) &&
                control->have_valid_vision != 0u &&
                (uint32_t)(inputs->now_ms - control->vision_last_valid_ms) >=
                    config->vision_timeout_ms) ||
               (config->follow_mode_compiled != 0u &&
                control->mode == CAR_MODE_WAIT_START_EDGE &&
                control->vision_have_frame_sequence != 0u &&
                (uint32_t)(inputs->now_ms -
                           control->vision_last_frame_alive_ms) >=
                    config->vision_timeout_ms)) {
        control->vision_timeout = 1u;
        car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
    }

    if (control->mode == CAR_MODE_RECOVERY_WAIT &&
        control->recovery_active == 0u) {
        control->mode = CAR_MODE_WAIT_START_EDGE;
    }

    if (control->debug_steering_mode_compiled == 0u &&
        control->mode == CAR_MODE_WAIT_START_EDGE &&
        car_control_vision_fresh(control, inputs->now_ms,
                                  config->vision_timeout_ms) != 0u) {
        if (config->bench_mode_compiled != 0u &&
            config->motor_hardware_accepted != 0u &&
            pressed_edge != 0u && raw_high == 0u &&
            inputs->pe7_low_stable != 0u &&
            control->button_low_edge_count != control->frame_edge_count &&
            car_control_bench_channel_mask(control) != 0u &&
            control->bench_requested_duty_permille != 0u) {
            control->mode = CAR_MODE_BENCH_RUNNING;
            control->bench_started_ms = inputs->now_ms;
            control->last_ramp_ms = inputs->now_ms;
            control->bench_output_duty_permille = 0u;
            control->bench_target_reached_ms = 0u;
            control->bench_target_hold_active = 0u;
        } else if (config->follow_mode_compiled != 0u &&
                   config->follow_calibration_ready != 0u &&
                   config->motor_hardware_accepted != 0u &&
                   control->manual_run_request != 0u &&
                   control->reverse_control_keep_seen != 0u &&
                   control->reverse_control_fresh_frames >=
                       CAR_RECOVERY_REQUIRED_FRAMES) {
            control->mode = CAR_MODE_FOLLOW_RUNNING;
            control->follow_session_active = 1u;
            control->follow_last_ramp_ms = inputs->now_ms;
            control->follow_output_duty_permille = 0u;
            control->follow_servo_target_us = follow_servo_target(
                config, control->vision_error_norm);
        }
    }

    if (control->mode == CAR_MODE_FOLLOW_RUNNING) {
        if (control->manual_run_request == 0u) {
            car_control_note_fault(control, CAR_FAULT_SWITCH_RELEASE);
        } else if (config->follow_mode_compiled == 0u ||
            config->follow_calibration_ready == 0u ||
            config->motor_hardware_accepted == 0u ||
            control->reverse_control_keep_seen == 0u ||
            control->reverse_control_fresh_frames <
                CAR_RECOVERY_REQUIRED_FRAMES ||
            control->follow_session_active == 0u ||
            control->fault_latched != 0u ||
            car_control_vision_fresh(control, inputs->now_ms,
                                     config->vision_timeout_ms) == 0u) {
            car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
        } else {
            elapsed = (uint32_t)(inputs->now_ms -
                                 control->follow_last_ramp_ms);
            if (elapsed >= CAR_RAMP_PERIOD_MS) {
                steps = elapsed / CAR_RAMP_PERIOD_MS;
                ramped = control->follow_output_duty_permille +
                         steps * config->follow_ramp_step_permille;
                control->follow_last_ramp_ms += steps * CAR_RAMP_PERIOD_MS;
                if (ramped > CAR_FOLLOW_BASE_TARGET_PM) {
                    ramped = CAR_FOLLOW_BASE_TARGET_PM;
                }
                if (ramped > CAR_HARD_FOLLOW_LIMIT_PM) {
                    ramped = CAR_HARD_FOLLOW_LIMIT_PM;
                }
                control->follow_output_duty_permille = (uint16_t)ramped;
            }
            control->road_drive_allowed = 1u;
        }
    }

    if (control->mode == CAR_MODE_FOLLOW_VISION_HOLD) {
        if (control->manual_run_request == 0u ||
            control->follow_session_active == 0u ||
            control->fault_latched != 0u) {
            finish_vision_hold(control, CAR_VISION_HOLD_EXIT_MANUAL_STOP);
            stop_outputs(control);
            control->follow_session_active = 0u;
            control->recovery_active = 1u;
            control->recovery_frame_count = 0u;
            control->recovery_have_sequence = 0u;
            control->rx_recovery_flush_required = 1u;
            request_reverse_control_reset(control);
            clear_trusted_observation(control);
            control->mode = CAR_MODE_RECOVERY_WAIT;
        } else if (control->vision_hold_expired != 0u) {
            control->follow_output_duty_permille = 0u;
            control->road_drive_allowed = 0u;
        } else if ((uint32_t)(inputs->now_ms -
                              control->vision_last_valid_ms) >=
                   CAR_CONTROL_VISION_HOLD_MS) {
            control->vision_hold_active = 0u;
            control->vision_hold_expired = 1u;
            control->vision_hold_restart_required = 1u;
            control->vision_hold_exit_cause = CAR_VISION_HOLD_EXIT_DEADLINE;
            control->follow_output_duty_permille = 0u;
            control->road_drive_allowed = 0u;
        } else {
            control->road_drive_allowed = 1u;
            control->follow_output_duty_permille =
                control->vision_hold_base_duty_permille;
        }
    }

    if (control->mode == CAR_MODE_DEBUG_STEERING_RUNNING) {
        if (config->debug_steering_mode_compiled == 0u ||
            control->steering_session_active == 0u ||
            control->fault_latched != 0u ||
            car_control_vision_fresh(control, inputs->now_ms,
                                     config->vision_timeout_ms) == 0u) {
            car_control_note_fault(control, CAR_FAULT_VISION_TIMEOUT);
        } else {
            /* The debug-steering profile never grants motor drive. */
            control->road_drive_allowed = 0u;
        }
    }

    if (control->mode == CAR_MODE_BENCH_COMPLETE && raw_high != 0u &&
        inputs->pe7_high_stable != 0u) {
        control->mode = CAR_MODE_WAIT_LOCAL_ACK;
    }
    if (control->mode != CAR_MODE_FOLLOW_RUNNING &&
        !(control->mode == CAR_MODE_FOLLOW_VISION_HOLD &&
          control->vision_hold_active != 0u)) {
        control->follow_output_duty_permille = 0u;
        control->road_drive_allowed = 0u;
    }
}
