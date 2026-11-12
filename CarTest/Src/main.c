#include "car_vision.h"
#include "car_calibration.h"
#include "commissioning.h"
#include "encoder_diag.h"
#include "follow_output.h"
#include "follow_incremental_pi.h"
#include "follow_speed_pi.h"
#include "manual_run_button.h"
#include "reverse_control.h"
#include "run_block_reason.h"
#include "runtime_diag.h"
#include "runtime_log.h"
#include "session_odometry.h"
#include "receive_diag.h"
#include "servo_safety.h"
#include "speed_pi_diag.h"
#include "speed_pi_trace.h"
#include "speed_observer.h"
#include "ackermann_control.h"
#include "ackermann_snapshot.h"
#include "uart_rx_service.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc_ex.h"

#include <stdint.h>

#if !defined(CARTEST_BENCH_MODE_COMPILED)
#error "CMake must explicitly select the motor bench profile."
#endif
#if !defined(CARTEST_SERVO_LIMITS_MEASURED)
#error "CMake must explicitly select the measured-servo-limit profile."
#endif
#if !defined(CARTEST_FOLLOW_MODE_COMPILED)
#error "CMake must explicitly select the vision-follow profile."
#endif
#if !defined(CARTEST_STEERING_MODE_COMPILED)
#error "CMake must explicitly select the debug-steering profile."
#endif
#if !defined(CARTEST_HARDWARE_ACCEPTED)
#error "CMake must explicitly select the hardware motor-acceptance lock."
#endif
#if !defined(CARTEST_RUNTIME_LOG_ENABLED)
#error "CMake must explicitly select whether runtime logging is enabled."
#endif
#if !defined(CARTEST_FOLLOW_SPEED_PI_ENABLED)
#error "CMake must explicitly select whether the Follow speed PI trial is enabled."
#endif
#if !defined(CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED) || \
    !defined(CARTEST_ACKERMANN_COORDINATION_ENABLED)
#error "CMake must explicitly select incremental PI and Ackermann coordination profiles."
#endif
#if !defined(CARTEST_FOLLOW_DIFF_PERMILLE)
#error "CMake must explicitly select the Follow differential level."
#endif
#if !defined(CARTEST_FOLLOW_INNER_DIFF_PERMILLE) || \
    !defined(CARTEST_FOLLOW_TURN_PROFILE_COMPILED) || \
    !defined(CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE) || \
    !defined(CARTEST_FOLLOW_HARD_LIMIT_PERMILLE) || \
    !defined(CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE) || \
    !defined(CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE) || \
    !defined(CARTEST_FOLLOW_SERVO_SLEW_STEP_US)
#error "CMake must explicitly select the Follow turn response fingerprint."
#endif
#if CARTEST_FOLLOW_TURN_PROFILE_COMPILED
#if !CARTEST_FOLLOW_MODE_COMPILED || \
    ((CARTEST_FOLLOW_DIFF_PERMILLE != 150) && \
     (CARTEST_FOLLOW_DIFF_PERMILLE != 200)) || \
    (CARTEST_FOLLOW_INNER_DIFF_PERMILLE != CARTEST_FOLLOW_DIFF_PERMILLE) || \
    (CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE != 1000) || \
    (CARTEST_FOLLOW_HARD_LIMIT_PERMILLE != 1000) || \
    (CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE != 10) || \
    (CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE != 40) || \
    (CARTEST_FOLLOW_SERVO_SLEW_STEP_US != \
        (CARTEST_ACKERMANN_COORDINATION_ENABLED ? 160 : 120))
#error "FollowTurn requires a 65/35 or 70/30 differential with the selected 120/160us steering slew and 40-permille/10ms motor ramp."
#endif
#if CARTEST_TRACK_PROFILE == 3
#if CARTEST_VISION_HOLD_MS != 3000 || CARTEST_VISION_HOLD_RAMP_MS != 300
#error "r22 C requires a 3000 ms hold deadline and 300 ms deceleration."
#endif
#else
#if CARTEST_VISION_HOLD_MS != 2000 || CARTEST_VISION_HOLD_RAMP_MS != 200
#error "FollowTurn requires a 2000 ms hold deadline and 200 ms deceleration."
#endif
#endif
#elif CARTEST_FOLLOW_MODE_COMPILED
#if (CARTEST_FOLLOW_DIFF_PERMILLE != 0) && \
    (CARTEST_FOLLOW_DIFF_PERMILLE != 50) && \
    (CARTEST_FOLLOW_DIFF_PERMILLE != 100) || \
    (CARTEST_FOLLOW_INNER_DIFF_PERMILLE != CARTEST_FOLLOW_DIFF_PERMILLE) || \
    (CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE != 1000) || \
    (CARTEST_FOLLOW_HARD_LIMIT_PERMILLE != 1000) || \
    (CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE != 10) || \
    (CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE != 40) || \
    (CARTEST_FOLLOW_SERVO_SLEW_STEP_US != 120) || \
    (CARTEST_VISION_HOLD_MS != 2000 || CARTEST_VISION_HOLD_RAMP_MS != 200)
#error "Follow requires 1000-permille caps, 40/10 ms motor ramp, 120/20 us servo slew, and a 2000/200 ms hold profile."
#endif
#else
#if CARTEST_FOLLOW_DIFF_PERMILLE != 0 || \
    CARTEST_FOLLOW_INNER_DIFF_PERMILLE != 0 || \
    CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE != 600 || \
    CARTEST_FOLLOW_HARD_LIMIT_PERMILLE != 1000 || \
    CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE != 10 || \
    CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE != 10 || \
    CARTEST_FOLLOW_SERVO_SLEW_STEP_US != 40 || \
    CARTEST_VISION_HOLD_MS != 3000 || \
    CARTEST_VISION_HOLD_RAMP_MS != 300
#error "Non-Follow profiles must preserve the 600-permille legacy output cap, 10-permille motor ramp, 40-us servo slew, and 3000/300 ms hold profile."
#endif
#endif
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED && \
    (!CARTEST_FOLLOW_SPEED_PI_ENABLED || \
     !CARTEST_FOLLOW_TURN_PROFILE_COMPILED || \
     CARTEST_FOLLOW_DIFF_PERMILLE != 150 || \
     !CARTEST_HARDWARE_ACCEPTED || \
     !CARTEST_RUNTIME_LOG_ENABLED)
#error "Incremental PI is reserved for the hardware-accepted, logged DebugFollowTurn65 C profile."
#endif
#if CARTEST_ACKERMANN_COORDINATION_ENABLED && \
    (!CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED || \
     !CARTEST_FOLLOW_SPEED_PI_ENABLED || \
     !CARTEST_FOLLOW_TURN_PROFILE_COMPILED || \
     CARTEST_FOLLOW_DIFF_PERMILLE != 150 || \
     CARTEST_FOLLOW_SERVO_SLEW_STEP_US != \
        160)
#error "Ackermann coordination requires the incremental-PI profile and its selected 120/160us servo slew."
#endif
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED && \
    !CARTEST_ACKERMANN_COORDINATION_ENABLED && \
    CARTEST_FOLLOW_SERVO_SLEW_STEP_US != 120
#error "The incremental PI baseline must preserve 120us/20ms servo slew."
#endif
#if !CARTEST_FOLLOW_MODE_COMPILED && (CARTEST_FOLLOW_DIFF_PERMILLE != 0)
#error "Follow differential is only valid in a Follow profile."
#endif
#if CARTEST_FOLLOW_SPEED_PI_ENABLED && \
    (!CARTEST_FOLLOW_MODE_COMPILED || !CARTEST_FOLLOW_TURN_PROFILE_COMPILED || \
     !CARTEST_HARDWARE_ACCEPTED || \
     CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE != 1000 || \
     CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE != 40 || \
     CARTEST_FOLLOW_SERVO_SLEW_STEP_US != \
        (CARTEST_ACKERMANN_COORDINATION_ENABLED ? 160 : 120) || \
     CARTEST_FOLLOW_SPEED_PI_TARGET_MILLIRPS != 2400 || \
     CARTEST_FOLLOW_SPEED_PI_KP_Q16 != 13107 || \
     CARTEST_FOLLOW_SPEED_PI_KI_Q16 != 26214 || \
     CARTEST_FOLLOW_SPEED_PI_OUTPUT_CAP_PERMILLE != 1000 || \
     CARTEST_FOLLOW_SPEED_PI_TRACKING_TIME_MS != 200)
#error "The speed PI trial requires the exact accepted FollowTurn profile and provisional PI parameters."
#endif
#if CARTEST_TRACK_PROFILE != CAR_TRACK_PROFILE_LEGACY && \
    (!CARTEST_FOLLOW_MODE_COMPILED || \
     !CARTEST_FOLLOW_TURN_PROFILE_COMPILED || \
     !CARTEST_FOLLOW_SPEED_PI_ENABLED || \
     !CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED || \
     !CARTEST_ACKERMANN_COORDINATION_ENABLED || \
     !CARTEST_HARDWARE_ACCEPTED || !CARTEST_RUNTIME_LOG_ENABLED)
#error "Paired track profiles require logged, hardware-accepted incremental-PI Ackermann Follow."
#endif
#if (CARTEST_BENCH_MODE_COMPILED + CARTEST_FOLLOW_MODE_COMPILED + \
     CARTEST_STEERING_MODE_COMPILED) > 1
#error "Actuator profiles are mutually exclusive."
#endif

#define APP_VISION_TIMEOUT_MS 250u
#define APP_MAIN_LOOP_GAP_LIMIT_MS 50u
#define APP_SWITCH_DEBOUNCE_MS 30u
#define APP_BENCH_LIMIT_PERMILLE 400u
#define APP_FOLLOW_NORMAL_LIMIT_PERMILLE \
    CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE
#define APP_FOLLOW_HARD_LIMIT_PERMILLE CARTEST_FOLLOW_HARD_LIMIT_PERMILLE
#define APP_BENCH_RAMP_STEP_PERMILLE 10u
#define APP_USART_BAUDRATE 115200u
#define APP_IWDG_RELOAD_COUNTS 500u
#define APP_IWDG_PRESCALER_BITS 4u /* LSI / 64. */
#define APP_IWDG_WINDOW_DISABLED 0x0FFFu
#define APP_IWDG_LSI_START_TIMEOUT_MS 1000u
#define APP_IWDG_REGISTER_UPDATE_TIMEOUT_MS 100u
#define APP_MOTOR_GPIOA_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#define APP_MOTOR_GPIOB_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#define APP_SERVO_GPIO_PIN GPIO_PIN_8
#define APP_SWITCH_GPIO_PIN GPIO_PIN_7
#define APP_MOTOR_CHANNEL_MASK (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)

#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
#define APP_FOLLOW_PI_ARMED (follow_incremental_pi_state.armed)
#define APP_FOLLOW_PI_STATUS (follow_incremental_pi_state.status)
#define APP_FOLLOW_PI_TARGET_LEFT (follow_incremental_pi_state.target_left_millirps)
#define APP_FOLLOW_PI_TARGET_RIGHT (follow_incremental_pi_state.target_right_millirps)
#define APP_FOLLOW_PI_REQUEST_LEFT (follow_incremental_pi_state.proposal_left_permille)
#define APP_FOLLOW_PI_REQUEST_RIGHT (follow_incremental_pi_state.proposal_right_permille)
#define APP_FOLLOW_PI_SATURATED_LEFT (follow_incremental_pi_state.saturated_left)
#define APP_FOLLOW_PI_SATURATED_RIGHT (follow_incremental_pi_state.saturated_right)
#else
#define APP_FOLLOW_PI_ARMED (follow_speed_pi_state.armed)
#define APP_FOLLOW_PI_STATUS (follow_speed_pi_state.status)
#define APP_FOLLOW_PI_TARGET_LEFT (follow_speed_pi_state.target_left_millirps)
#define APP_FOLLOW_PI_TARGET_RIGHT (follow_speed_pi_state.target_right_millirps)
#define APP_FOLLOW_PI_REQUEST_LEFT (follow_speed_pi_state.proposal_left_permille)
#define APP_FOLLOW_PI_REQUEST_RIGHT (follow_speed_pi_state.proposal_right_permille)
#define APP_FOLLOW_PI_SATURATED_LEFT (follow_speed_pi_state.saturated_left)
#define APP_FOLLOW_PI_SATURATED_RIGHT (follow_speed_pi_state.saturated_right)
#endif

/* Fixed harness phase labels are reversed relative to timer channel order. */
#define APP_ENCODER_LEFT_A_GPIO_PIN GPIO_PIN_1  /* PA1 / TIM2_CH2 */
#define APP_ENCODER_LEFT_B_GPIO_PIN GPIO_PIN_0  /* PA0 / TIM2_CH1 */
#define APP_ENCODER_RIGHT_A_GPIO_PIN GPIO_PIN_7 /* PC7 / TIM8_CH2 */
#define APP_ENCODER_RIGHT_B_GPIO_PIN GPIO_PIN_6 /* PC6 / TIM8_CH1 */

/* Exporters use these ROM symbols to require a frozen speed trace for the
   two PI builds even though the older 128-byte snapshot is C-only. */
#if CARTEST_FOLLOW_SPEED_PI_ENABLED && CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
const uint8_t g_speed_pi_trace_profile_incremental
    __attribute__((used)) = 2u;
#elif CARTEST_FOLLOW_SPEED_PI_ENABLED
const uint8_t g_speed_pi_trace_profile_position
    __attribute__((used)) = 1u;
#endif

/* These numeric values are intentionally stable for Live Watch diagnosis. */
typedef enum {
    CAR_INIT_STAGE_RESET_SAFE = 0u,
    CAR_INIT_STAGE_CLOCK_OSC = 1u,
    CAR_INIT_STAGE_CLOCK_TREE = 2u,
    CAR_INIT_STAGE_CLOCK_VERIFY = 3u,
    CAR_INIT_STAGE_SWITCH_GPIO = 4u,
    CAR_INIT_STAGE_MOTOR_TIM3_BASE = 5u,
    CAR_INIT_STAGE_MOTOR_TIM3_CH1 = 6u,
    CAR_INIT_STAGE_MOTOR_TIM3_CH2 = 7u,
    CAR_INIT_STAGE_MOTOR_TIM3_CH3 = 8u,
    CAR_INIT_STAGE_MOTOR_TIM3_CH4 = 9u,
    CAR_INIT_STAGE_MOTOR_TIM3_VERIFY = 10u,
    CAR_INIT_STAGE_SERVO_TIM1_BASE = 11u,
    CAR_INIT_STAGE_SERVO_TIM1_CH1 = 12u,
    CAR_INIT_STAGE_SERVO_TIM1_VERIFY = 13u,
    CAR_INIT_STAGE_ENCODER_TIM2_CONFIG = 14u,
    CAR_INIT_STAGE_ENCODER_TIM8_CONFIG = 15u,
    CAR_INIT_STAGE_ENCODER_TIM2_START = 16u,
    CAR_INIT_STAGE_ENCODER_TIM8_START = 17u,
    CAR_INIT_STAGE_CONTROLLER_INIT = 18u,
    CAR_INIT_STAGE_USART1_VERIFY = 19u,
    CAR_INIT_STAGE_USART1_ENABLE = 20u,
    CAR_INIT_STAGE_IWDG_ENABLE = 21u,
    CAR_INIT_STAGE_IWDG_LSI_WAIT = 22u,
    CAR_INIT_STAGE_IWDG_WRITE_ACCESS = 23u,
    CAR_INIT_STAGE_IWDG_CONFIGURE = 24u,
    CAR_INIT_STAGE_IWDG_UPDATE_WAIT = 25u,
    CAR_INIT_STAGE_IWDG_VERIFY = 26u,
    CAR_INIT_STAGE_READY = 27u
} car_init_stage_t;

typedef enum {
    CAR_INIT_ERROR_NONE = 0u,
    CAR_INIT_ERROR_CLOCK_OSC = 1u,
    CAR_INIT_ERROR_CLOCK_TREE = 2u,
    CAR_INIT_ERROR_CLOCK_VERIFY = 3u,
    CAR_INIT_ERROR_SWITCH_GPIO = 4u,
    CAR_INIT_ERROR_MOTOR_TIM3_BASE = 5u,
    CAR_INIT_ERROR_MOTOR_TIM3_CH1 = 6u,
    CAR_INIT_ERROR_MOTOR_TIM3_CH2 = 7u,
    CAR_INIT_ERROR_MOTOR_TIM3_CH3 = 8u,
    CAR_INIT_ERROR_MOTOR_TIM3_CH4 = 9u,
    CAR_INIT_ERROR_MOTOR_TIM3_VERIFY = 10u,
    CAR_INIT_ERROR_SERVO_TIM1_BASE = 11u,
    CAR_INIT_ERROR_SERVO_TIM1_CH1 = 12u,
    CAR_INIT_ERROR_SERVO_TIM1_VERIFY = 13u,
    CAR_INIT_ERROR_ENCODER_TIM2_CONFIG = 14u,
    CAR_INIT_ERROR_ENCODER_TIM8_CONFIG = 15u,
    CAR_INIT_ERROR_ENCODER_TIM2_START = 16u,
    CAR_INIT_ERROR_ENCODER_TIM8_START = 17u,
    CAR_INIT_ERROR_USART1_VERIFY = 19u,
    CAR_INIT_ERROR_USART1_ENABLE = 20u,
    CAR_INIT_ERROR_IWDG_LSI_NOT_READY = 22u,
    CAR_INIT_ERROR_IWDG_REGISTER_UPDATE_TIMEOUT = 25u,
    CAR_INIT_ERROR_IWDG_READBACK = 26u,
    CAR_INIT_ERROR_TRACE_SRAM = 27u,
    CAR_INIT_ERROR_UNCLASSIFIED = 0x7FFFFFFFu
} car_init_error_t;

/* Global names are intentionally stable for STM32CubeIDE Live Watch. */
volatile uint32_t car_command_mailbox;
volatile uint32_t car_selected_motor;
volatile uint32_t car_selected_input;
volatile uint32_t car_requested_duty_permille;
volatile uint32_t car_mode;
volatile uint32_t car_fault;
volatile uint32_t car_run_block_reason;
volatile uint32_t car_runtime_log_enabled;
volatile uint32_t car_manual_run_request;
volatile uint32_t car_button_press_event_count;
volatile uint32_t car_button_start_event_count;
volatile uint32_t car_button_stop_generation;
volatile uint32_t car_button_armed;
volatile uint32_t car_start_pending;
volatile uint32_t car_vision_hold_active;
volatile uint32_t car_vision_hold_elapsed_ms;
volatile uint32_t car_vision_hold_entries;
volatile uint32_t car_vision_hold_exit_cause;
volatile uint32_t car_drive_allowed;
volatile uint32_t car_pe7_raw_high;
volatile uint32_t car_pe7_high_stable;
volatile uint32_t car_pe7_low_stable;
volatile uint32_t car_vision_valid;
volatile uint32_t car_vision_timeout;
volatile uint32_t car_vision_confidence;
volatile int32_t car_vision_error_norm;
volatile uint32_t car_vision_sequence;
volatile uint32_t car_vision_rx_frames;
volatile uint32_t car_latest_rx_have_frame;
volatile uint32_t car_latest_rx_valid;
volatile uint32_t car_latest_rx_slope_hint;
volatile uint32_t car_latest_rx_confidence;
volatile int32_t car_latest_rx_error_norm;
volatile uint32_t car_latest_rx_sequence;
volatile uint32_t car_latest_rx_arrival_ms;
volatile uint32_t car_latest_rx_frame_count;
volatile uint32_t car_vision_slope_hint;
volatile uint32_t car_slope_assist_active;
volatile uint32_t car_recovery_frame_count;
volatile uint32_t car_recovery_active;
volatile uint32_t car_auto_recovery_count;
volatile uint32_t car_follow_session_active;
volatile uint32_t car_steering_session_active;
volatile uint32_t car_steering_servo_target_us;
volatile uint32_t car_rx_flush_pending;
volatile uint32_t car_servo_output_active;
volatile uint32_t car_servo_applied_pulse_us;
volatile uint32_t car_servo_requested_us;
volatile uint32_t car_servo_limited_us;
volatile uint32_t car_servo_ccr1_software_us;
volatile uint32_t car_servo_last_update_compare_us_estimate;
volatile uint32_t car_servo_pa8_idr_high;
volatile uint32_t car_servo_stop_pending;
volatile uint32_t car_servo_start_cnt_snapshot;
volatile uint32_t car_servo_stop_request_cnt_snapshot;
volatile uint32_t car_servo_stop_complete_cnt_snapshot;
volatile uint32_t car_servo_timer_psc;
volatile uint32_t car_servo_timer_arr;
volatile uint32_t car_servo_fault_latched;
volatile uint32_t car_servo_sticky_fault_code;
volatile uint32_t car_servo_frame_ready;
volatile uint32_t car_servo_last_rejected_request_us;
volatile uint32_t car_servo_fault_snapshot_cr1;
volatile uint32_t car_servo_fault_snapshot_cr2;
volatile uint32_t car_servo_fault_snapshot_smcr;
volatile uint32_t car_servo_fault_snapshot_dier;
volatile uint32_t car_servo_fault_snapshot_ccmr1;
volatile uint32_t car_servo_fault_snapshot_ccer;
volatile uint32_t car_servo_fault_snapshot_bdtr;
volatile uint32_t car_servo_fault_snapshot_psc;
volatile uint32_t car_servo_fault_snapshot_arr;
volatile uint32_t car_servo_fault_snapshot_rcr;
volatile uint32_t car_servo_fault_snapshot_ccr1;
volatile uint32_t car_servo_fault_snapshot_rcc_cr;
volatile uint32_t car_servo_fault_snapshot_rcc_cfgr;
volatile uint32_t car_servo_fault_snapshot_d1cfgr;
volatile uint32_t car_servo_fault_snapshot_d2cfgr;
volatile uint32_t car_servo_fault_snapshot_gpioa_moder;
volatile uint32_t car_servo_fault_snapshot_gpioa_afr1;
volatile uint32_t car_servo_ccr_is_software_request_only = 1u;
volatile uint32_t car_servo_real_pwm_verified;
volatile uint32_t car_motor_hardware_accepted = CARTEST_HARDWARE_ACCEPTED;
volatile uint32_t car_motor_hardware_lock_reason;
volatile uint32_t car_reverse_control_sequence;
volatile uint32_t car_reverse_control_command;
volatile uint32_t car_reverse_control_reset_generation;
volatile uint32_t car_reverse_control_reset_phase_complete;
volatile uint32_t car_reverse_control_keep_active;
volatile uint32_t car_reverse_control_fresh_frames;
volatile uint32_t car_camera_profile_required_ack;
volatile uint32_t car_camera_profile_latest_ack;
volatile uint32_t car_camera_profile_latest_version;
volatile uint32_t car_camera_profile_ack_ready;
volatile uint32_t car_camera_profile_ack_seen_after_reset;
volatile uint32_t car_camera_profile_ack_age_ms;
volatile uint32_t car_camera_profile_last_ack_ms;
volatile uint32_t car_camera_profile_mismatch_count;
volatile uint32_t car_profile_state_sequence;
volatile uint32_t car_profile_state_last_tx_ms;
volatile uint32_t car_profile_state_tx_count;
volatile uint32_t car_profile_state_id;
volatile uint32_t car_profile_state_session_active;
volatile uint32_t car_session_token;
volatile uint32_t car_session_mean_forward_wheel_millirev;
volatile uint32_t car_session_odom_valid_samples;
volatile uint32_t car_session_odom_invalid_samples;
volatile uint32_t car_follow_servo_target_us;
volatile uint32_t car_follow_motor_duty_permille;
volatile uint32_t car_follow_diff_mode_permille;
volatile uint32_t car_follow_inner_diff_permille;
volatile uint32_t car_follow_diff_u_permille;
volatile uint32_t car_follow_profile_id;
volatile uint32_t car_follow_outer_delta_permille;
volatile uint32_t car_follow_inner_delta_permille;
volatile uint32_t car_follow_normal_cap_permille;
volatile uint32_t car_follow_motor_normal_step_permille;
volatile uint32_t car_follow_servo_normal_step_us;
volatile uint32_t car_follow_output_phase;
volatile uint32_t car_follow_startup_qualified;
volatile uint32_t car_follow_base_target_permille;
volatile uint32_t car_follow_left_target_permille;
volatile uint32_t car_follow_right_target_permille;
volatile uint32_t car_follow_left_command_permille;
volatile uint32_t car_follow_right_command_permille;
volatile uint32_t car_uart_rx_bytes;
volatile uint32_t car_uart_error_count;
volatile uint32_t car_uart_parity_error_count;
volatile uint32_t car_uart_framing_error_count;
volatile uint32_t car_uart_noise_error_count;
volatile uint32_t car_uart_overrun_error_count;
volatile uint32_t car_uart_rx_fifo_enabled;
volatile uint32_t car_uart_rx_irq_max_batch;
volatile uint32_t car_pe7_release_event_count;
volatile uint32_t car_uart_overflow_count;
volatile uint32_t car_protocol_error_count;
volatile uint32_t car_uart_partial_timeout_count;
volatile uint32_t car_uart_queue_age_reject_count;
volatile uint32_t car_uart_discarded_byte_count;
volatile uint32_t car_encoder_left_raw;
volatile uint32_t car_encoder_right_raw;
volatile int64_t car_encoder_left_raw_delta_sum;
volatile int64_t car_encoder_right_raw_delta_sum;
volatile uint32_t car_encoder_ambiguous_delta_count;
volatile uint32_t car_speed_observer_valid;
volatile uint32_t car_speed_observer_status;
volatile uint32_t car_speed_observer_elapsed_ms;
volatile uint32_t car_speed_observer_sample_age_ms;
volatile int32_t car_speed_left_millirps;
volatile int32_t car_speed_right_millirps;
volatile int32_t car_speed_left_rpm;
volatile int32_t car_speed_right_rpm;
volatile uint32_t car_speed_pi_actuation_enabled;
volatile uint32_t car_speed_pi_compiled = CARTEST_FOLLOW_SPEED_PI_ENABLED;
volatile uint32_t car_speed_pi_status;
volatile uint32_t car_speed_pi_type;
volatile uint32_t car_speed_pi_target_left_millirps;
volatile uint32_t car_speed_pi_target_right_millirps;
volatile uint32_t car_speed_pi_requested_left_permille;
volatile uint32_t car_speed_pi_requested_right_permille;
volatile uint32_t car_speed_pi_saturated_left;
volatile uint32_t car_speed_pi_saturated_right;
volatile int64_t car_speed_pi_integral_left_q16;
volatile int64_t car_speed_pi_integral_right_q16;
/* These expose the active incremental controller's output accumulators;
   car_speed_pi_integral_* retain their legacy position-PI meaning. */
volatile int64_t car_speed_pi_accumulator_left_q16;
volatile int64_t car_speed_pi_accumulator_right_q16;
volatile uint32_t car_enc_tim8_cnt_hw;
volatile uint32_t car_enc_tim8_cen;
volatile uint32_t car_enc_tim8_clock_enabled;
volatile uint32_t car_enc_tim8_smcr;
volatile uint32_t car_enc_tim8_ccmr1;
volatile uint32_t car_enc_tim8_ccer;
volatile uint32_t car_motor_left_in1_output_permille;
volatile uint32_t car_motor_left_in2_output_permille;
volatile uint32_t car_motor_right_in1_output_permille;
volatile uint32_t car_motor_right_in2_output_permille;
volatile uint32_t car_motor_pwm_ccr2;
volatile uint32_t car_motor_pwm_ccr3;
volatile uint32_t car_motor_pwm_ccer;
volatile uint32_t car_motor_pwm_enabled;
volatile uint32_t car_bench_ready;
volatile uint32_t car_bench_max_applied_duty_permille;
volatile uint32_t car_main_loop_heartbeat_ms;
volatile uint32_t car_main_loop_gap_fault;
volatile uint32_t car_iwdg_reset_seen;
volatile uint32_t car_system_sysclk_hz;
volatile uint32_t car_system_hclk_hz;
volatile uint32_t car_system_pclk1_hz;
volatile uint32_t car_system_pclk2_hz;
volatile uint32_t car_tim3_prescaler;
volatile uint32_t car_tim3_period;
volatile uint32_t car_tim1_prescaler;
volatile uint32_t car_tim1_period;
volatile uint32_t car_init_stage = CAR_INIT_STAGE_RESET_SAFE;
volatile uint32_t car_init_hal_status = HAL_OK;
volatile uint32_t car_init_error = CAR_INIT_ERROR_NONE;
volatile uint32_t car_init_rcc_csr_snapshot;
volatile uint32_t car_init_rcc_cr_snapshot;
volatile uint32_t car_init_rcc_cfgr_snapshot;
volatile uint32_t car_init_rcc_rsr_reset_snapshot;
volatile uint32_t car_init_iwdg_sr_snapshot;
volatile uint32_t car_init_iwdg_pr_snapshot;
volatile uint32_t car_init_iwdg_rlr_snapshot;
volatile uint32_t car_init_iwdg_winr_snapshot;
volatile uint32_t car_init_lsi_wait_ms;
volatile uint32_t car_init_iwdg_update_wait_ms;

car_control_t car_control_state;
car_vision_rx_t car_vision_rx;
car_encoder_diag_t car_encoder_diag;
static car_speed_observer_t speed_observer;
static car_speed_observer_result_t speed_observer_result;
static car_follow_speed_pi_state_t follow_speed_pi_state;
static car_follow_incremental_pi_state_t follow_incremental_pi_state;
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
static car_ackermann_control_state_t ackermann_control_state;
static car_ackermann_control_output_t ackermann_control_output = {
    .model_angle_cdeg = 0,
    .ratio_q15 = CAR_ACKERMANN_RATIO_ONE_Q15,
    .request_permille = 0u,
    .curve_phase = CAR_ACKERMANN_PHASE_STRAIGHT,
    .request_active = 0u,
    .fastest_target_millirps = CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS,
    .left_target_millirps = CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS,
    .right_target_millirps = CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS
};
static uint8_t ackermann_control_session_ready;
#endif
static uint8_t ackermann_snapshot_session_active;
static uint32_t ackermann_snapshot_last_publish_ms;
static uint16_t ackermann_snapshot_last_valid_servo_pulse_us;
#endif
static volatile uint8_t follow_speed_pi_reset_pending;
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
static uint8_t speed_pi_diag_session_active;
static uint32_t speed_pi_diag_start_token;
static uint8_t speed_pi_diag_have_values;
static uint8_t speed_pi_diag_have_observer_sample;
static uint8_t speed_pi_diag_last_armed;
static uint8_t speed_pi_diag_last_hold;
static uint32_t speed_pi_diag_last_observer_sample_ms;
static uint32_t speed_pi_diag_last_status;
static uint16_t speed_pi_diag_last_request_left;
static uint16_t speed_pi_diag_last_request_right;
static uint16_t speed_pi_diag_last_applied_left;
static uint16_t speed_pi_diag_last_applied_right;
#endif
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
static uint8_t speed_pi_trace_session_active;
static uint32_t speed_pi_trace_start_token;
static uint8_t speed_pi_trace_have_sample;
static uint32_t speed_pi_trace_last_sample_ms;
#endif

static car_config_t app_config;

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim8;
static volatile uint8_t motor_outputs_active;
static volatile uint8_t servo_output_active;
static volatile uint8_t servo_stop_pending;
static uint16_t applied_motor_duty_permille;
static uint16_t applied_motor_follow_left_permille;
static uint16_t applied_motor_follow_right_permille;
static uint8_t applied_motor_channel_mask;
static uint8_t applied_follow_outputs;

typedef struct {
    uint16_t left_in1_permille;
    uint16_t left_in2_permille;
    uint16_t right_in1_permille;
    uint16_t right_in2_permille;
} motor_applied_channels_t;

static uint16_t applied_servo_pulse_us;
static car_servo_safety_t servo_safety;
static car_follow_output_state_t follow_output_state;
static car_servo_safety_io_t servo_safety_io;
static car_reverse_control_tx_t reverse_control_tx;
static uint32_t follow_session_token;
static uint64_t session_left_millirev;
static uint64_t session_right_millirev;
static uint64_t session_mean_forward_millirev;
static uint64_t odometry_left_fractional_remainder;
static uint64_t odometry_right_fractional_remainder;
static int64_t odometry_last_left_raw_counts;
static int64_t odometry_last_right_raw_counts;
static uint8_t odometry_have_baseline;
static uint32_t odometry_valid_sample_count;
static uint32_t odometry_rejected_sample_count;
static uint32_t reverse_control_seen_generation;
static uint32_t reverse_control_seen_rx_fault_generation;
static uint8_t reverse_control_ready;
static uint8_t reverse_comm_outage;
static uint8_t reverse_keep_notified;
static uint8_t reverse_manual_stop_holdoff;
static uint8_t follow_ramp_started;
static uint8_t iwdg_started;
static car_manual_button_t manual_run_button;
static uint32_t manual_stop_consumed_generation;

typedef struct {
    uint32_t control_fault;
    uint32_t control_rx_fault;
    uint32_t control_isr_switch;
    uint32_t control_isr_uart;
    uint32_t rx_parser_fault;
    uint32_t rx_isr_fault;
} output_generation_snapshot_t;

typedef struct {
    uint8_t candidate_level;
    uint8_t stable_level;
    uint8_t stable_valid;
    uint32_t candidate_since_ms;
} switch_debounce_t;

static switch_debounce_t switch_debounce;

static HAL_StatusTypeDef system_clock_config(void);
static HAL_StatusTypeDef verify_system_and_timer_clocks(void);
static HAL_StatusTypeDef motor_pwm_init(void);
static HAL_StatusTypeDef servo_pwm_init(void);
static HAL_StatusTypeDef encoder_init(void);
static HAL_StatusTypeDef usart1_init(void);
static HAL_StatusTypeDef switch_init(void);
static HAL_StatusTypeDef iwdg_start(void);
static void init_stage_begin(car_init_stage_t stage);
static HAL_StatusTypeDef init_status_record(car_init_error_t error,
                                            HAL_StatusTypeDef status);
static void init_capture_registers(void);
static void init_failure(car_fault_t fault, car_init_error_t fallback_error,
                         HAL_StatusTypeDef status);
void Car_FatalException(void);
static void emergency_stop_hardware(void);
static void emergency_stop_hardware_immediate(void);
static void servo_hardware_stop(void *context);
static void servo_hardware_stop_now(void);
static uint8_t servo_stop_timer_config_valid(uint8_t expect_stop_pending);
static void servo_latch_stop_timer_fault(void);
static void servo_capture_snapshot(void *context,
                                   car_servo_timer_snapshot_t *snapshot);
static uint8_t servo_write_compare(void *context, uint16_t pulse_us);
static uint32_t servo_read_compare(void *context);
static uint8_t servo_hardware_start(void *context);
static uint8_t servo_take_completed_update(void *context);
static void servo_safety_fault_to_controller(void);
static void service_reverse_control(uint32_t now_ms);
static void reset_session_odometry(void);
static void update_session_odometry(void);
static void enforce_motor_hardware_lock(void);
static void consume_fast_stop_latch(void);
static void flush_uart_and_parser(void);
static uint8_t profile_rx_path_healthy(void);
static void service_physical_profile_operation(const car_inputs_t *inputs);
static void publish_encoder_hardware_snapshot(void);
static motor_applied_channels_t motor_applied_channels_snapshot(void);
static void publish_motor_channel_debug_snapshot(void);
static void publish_debug_snapshot(const car_inputs_t *inputs);
static void protected_uart_error_note(uint32_t status);
static void protected_uart_byte_note(uint8_t byte, uint32_t arrival_ms);
static uint16_t vision_rx_snapshot(uint32_t *now_ms);
static uint8_t pe7_is_high(void);
static car_manual_button_event_t manual_button_sample(uint8_t raw_high,
                                                       uint32_t now_ms);
static void cancel_manual_request_on_fault(void);
static void consume_manual_stop_request(const car_inputs_t *inputs);
static void apply_manual_stop_effects(uint32_t effects);
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
static void speed_pi_diag_publish(uint32_t now_ms, uint8_t allow_new_sample,
                                  uint8_t force);
static void speed_pi_diag_freeze_if_stopped(uint32_t now_ms);
#endif
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
static void speed_pi_trace_publish_sample(void);
static void speed_pi_trace_freeze_if_stopped(uint32_t now_ms);
#endif
#if CARTEST_RUNTIME_LOG_ENABLED
static car_runtime_log_session_kind_t runtime_log_start_edge(
    car_mode_t mode_before_tick);
static void runtime_log_publish_sample(uint32_t now_ms,
                                       car_runtime_log_session_kind_t
                                           start_kind);
#endif
static void record_uart_error_flags(uint32_t status);
static void note_uart_error_locked(uint32_t status);

static uint32_t uart_error_icr_mask(uint32_t status)
{
    uint32_t clear_flags = 0u;
    if ((status & USART_ISR_PE) != 0u) clear_flags |= USART_ICR_PECF;
    if ((status & USART_ISR_FE) != 0u) clear_flags |= USART_ICR_FECF;
    if ((status & USART_ISR_NE) != 0u) clear_flags |= USART_ICR_NECF;
    if ((status & USART_ISR_ORE) != 0u) clear_flags |= USART_ICR_ORECF;
    return clear_flags;
}

static uint32_t uart_service_read_status(void *context)
{
    (void)context;
    return USART1->ISR;
}

static uint8_t uart_service_read_data(void *context)
{
    (void)context;
    return (uint8_t)(USART1->RDR & 0xFFu);
}

static void uart_service_stop_receive(void)
{
    USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE;
    USART1->RQR = USART_RQR_RXFRQ;
    __DSB();
}

static void uart_service_on_error(void *context, uint32_t status)
{
    (void)context;
    protected_uart_error_note(status);
    emergency_stop_hardware();
    uart_service_stop_receive();
}

static uint8_t uart_service_on_byte(void *context, uint8_t byte)
{
    (void)context;
    protected_uart_byte_note(byte, HAL_GetTick());
    return car_vision_rx.overflow_fault != 0u ? 1u : 0u;
}

static void uart_service_on_abort(void *context)
{
    (void)context;
    emergency_stop_hardware();
    uart_service_stop_receive();
}

/* These wrappers serialize the small ISR-owned counter/latch updates against
   higher-priority EXTI and USART handlers. Do not include HAL waits here. */
static void protected_switch_release_note(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    car_control_note_switch_release_isr(&car_control_state);
    __set_PRIMASK(primask);
}

static void protected_fast_stop_note(car_fault_t fault)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    car_control_note_fast_stop_isr(&car_control_state, fault);
    __set_PRIMASK(primask);
}

static void record_uart_error_flags(uint32_t status)
{
    if ((status & USART_ISR_PE) != 0u) {
        ++car_uart_parity_error_count;
    }
    if ((status & USART_ISR_FE) != 0u) {
        ++car_uart_framing_error_count;
    }
    if ((status & USART_ISR_NE) != 0u) {
        ++car_uart_noise_error_count;
    }
    if ((status & USART_ISR_ORE) != 0u) {
        ++car_uart_overrun_error_count;
    }
}

/* Caller holds PRIMASK. Counts describe observed/handled error flag bits,
   not a one-to-one count of electrical line faults. Clear only the bits in
   the captured status so a later error remains available for the next pass. */
static void note_uart_error_locked(uint32_t status)
{
    const uint32_t errors = status & (USART_ISR_PE | USART_ISR_FE |
                                      USART_ISR_NE | USART_ISR_ORE);
    if (errors == 0u) {
        return;
    }
    record_uart_error_flags(errors);
    car_vision_rx_isr_error(&car_vision_rx);
    car_control_note_fast_stop_isr(&car_control_state, CAR_FAULT_UART);
    USART1->ICR = uart_error_icr_mask(errors);
}

static void protected_uart_error_note(uint32_t status)
{
    uint32_t primask = __get_PRIMASK();
    if ((status & (USART_ISR_PE | USART_ISR_FE |
                   USART_ISR_NE | USART_ISR_ORE)) == 0u) {
        return;
    }
    __disable_irq();
    /* Note and clear only the captured errors once; the caller then stops
       outputs before disabling RX service and flushing the FIFO. */
    note_uart_error_locked(status);
    __set_PRIMASK(primask);
}

static void protected_uart_byte_note(uint8_t byte, uint32_t arrival_ms)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t overflow_before;
    __disable_irq();
    overflow_before = car_vision_rx.overflow_fault;
    car_vision_rx_isr_byte(&car_vision_rx, byte, arrival_ms);
    if (overflow_before == 0u && car_vision_rx.overflow_fault != 0u) {
        car_control_note_fast_stop_isr(&car_control_state,
                                       CAR_FAULT_RX_OVERFLOW);
    }
    __set_PRIMASK(primask);
}

static void init_stage_begin(car_init_stage_t stage)
{
    car_init_stage = (uint32_t)stage;
    car_init_hal_status = (uint32_t)HAL_BUSY;
    car_init_error = (uint32_t)CAR_INIT_ERROR_NONE;
}

static HAL_StatusTypeDef init_status_record(car_init_error_t error,
                                            HAL_StatusTypeDef status)
{
    car_init_hal_status = (uint32_t)status;
    car_init_error = (status == HAL_OK) ? (uint32_t)CAR_INIT_ERROR_NONE
                                        : (uint32_t)error;
    return status;
}

static void init_capture_registers(void)
{
    car_init_rcc_csr_snapshot = RCC->CSR;
    car_init_rcc_cr_snapshot = RCC->CR;
    car_init_rcc_cfgr_snapshot = RCC->CFGR;
    car_init_iwdg_sr_snapshot = IWDG1->SR;
    car_init_iwdg_pr_snapshot = IWDG1->PR;
    car_init_iwdg_rlr_snapshot = IWDG1->RLR;
    car_init_iwdg_winr_snapshot = IWDG1->WINR;
    __DSB();
}

static void init_failure(car_fault_t fault, car_init_error_t fallback_error,
                         HAL_StatusTypeDef status)
{
    if (car_init_error == (uint32_t)CAR_INIT_ERROR_NONE) {
        car_init_error = (uint32_t)fallback_error;
    }
    car_init_hal_status = (uint32_t)status;
    car_fault = (uint32_t)fault;
    init_capture_registers();
    Car_FatalException();
}

static void early_motor_pins_low(void)
{
    const uint32_t a_mask = (3u << 12u) | (3u << 14u) | (3u << 16u);
    const uint32_t b_mask = (3u << 0u) | (3u << 2u);
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN;
    (void)RCC->AHB4ENR;
    GPIOA->BSRR = (uint32_t)(APP_MOTOR_GPIOA_PINS | APP_SERVO_GPIO_PIN) << 16u;
    GPIOB->BSRR = (uint32_t)APP_MOTOR_GPIOB_PINS << 16u;
    GPIOA->MODER = (GPIOA->MODER & ~a_mask) | (1u << 12u) | (1u << 14u) | (1u << 16u);
    GPIOB->MODER = (GPIOB->MODER & ~b_mask) | (1u << 0u) | (1u << 2u);
    GPIOA->OTYPER &= ~(APP_MOTOR_GPIOA_PINS | APP_SERVO_GPIO_PIN);
    GPIOB->OTYPER &= ~APP_MOTOR_GPIOB_PINS;
    GPIOA->PUPDR &= ~a_mask;
    GPIOB->PUPDR &= ~b_mask;
    __DSB();
}

static void set_motor_gpio_channels_af(uint32_t channels)
{
    const uint32_t a_mode_mask = (3u << 12u) | (3u << 14u);
    const uint32_t b_mode_mask = (3u << 0u) | (3u << 2u);
    GPIOA->BSRR = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7) << 16u;
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_0 | GPIO_PIN_1) << 16u;
    GPIOA->MODER = (GPIOA->MODER & ~a_mode_mask) |
                   (1u << 12u) | (1u << 14u);
    GPIOB->MODER = (GPIOB->MODER & ~b_mode_mask) |
                   (1u << 0u) | (1u << 2u);
    GPIOA->OTYPER &= ~(GPIO_PIN_6 | GPIO_PIN_7);
    GPIOB->OTYPER &= ~(GPIO_PIN_0 | GPIO_PIN_1);
    GPIOA->PUPDR &= ~a_mode_mask;
    GPIOB->PUPDR &= ~b_mode_mask;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((15u << 24u) | (15u << 28u))) |
                    (2u << 24u) | (2u << 28u);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~0xFFu) | 0x22u;
    __DSB();
    if ((channels & TIM_CCER_CC1E) != 0u) {
        GPIOA->MODER = (GPIOA->MODER & ~(3u << 12u)) | (2u << 12u);
    }
    if ((channels & TIM_CCER_CC2E) != 0u) {
        GPIOA->MODER = (GPIOA->MODER & ~(3u << 14u)) | (2u << 14u);
    }
    if ((channels & TIM_CCER_CC3E) != 0u) {
        GPIOB->MODER = (GPIOB->MODER & ~(3u << 0u)) | (2u << 0u);
    }
    if ((channels & TIM_CCER_CC4E) != 0u) {
        GPIOB->MODER = (GPIOB->MODER & ~(3u << 2u)) | (2u << 2u);
    }
    __DSB();
}

static void set_servo_gpio_af(void)
{
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFu) | 1u;
    __DSB();
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 16u)) | (2u << 16u);
}

static void set_servo_gpio_low(void)
{
    GPIOA->BSRR = (uint32_t)APP_SERVO_GPIO_PIN << 16u;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 16u)) | (1u << 16u);
    __DSB();
}

static void servo_capture_snapshot(
    void *context, car_servo_timer_snapshot_t *snapshot)
{
    (void)context;
    if (snapshot == 0) {
        return;
    }
    snapshot->rcc_cr = RCC->CR;
    snapshot->rcc_cfgr = RCC->CFGR;
    snapshot->rcc_d1cfgr = RCC->D1CFGR;
    snapshot->rcc_d2cfgr = RCC->D2CFGR;
    snapshot->rcc_apb2enr = RCC->APB2ENR;
    (void)car_servo_safety_apply_fixed_hsi64_profile(snapshot);
    snapshot->tim_cr1 = TIM1->CR1;
    snapshot->tim_cr2 = TIM1->CR2;
    snapshot->tim_smcr = TIM1->SMCR;
    snapshot->tim_dier = TIM1->DIER;
    snapshot->tim_ccmr1 = TIM1->CCMR1;
    snapshot->tim_ccer = TIM1->CCER;
    snapshot->tim_bdtr = TIM1->BDTR;
    snapshot->tim_psc = TIM1->PSC;
    snapshot->tim_arr = TIM1->ARR;
    snapshot->tim_rcr = TIM1->RCR;
    snapshot->tim_ccr1 = TIM1->CCR1;
    snapshot->gpioa_moder = GPIOA->MODER;
    snapshot->gpioa_odr = GPIOA->ODR;
    snapshot->gpioa_afr1 = GPIOA->AFR[1];
}

static uint8_t servo_write_compare(void *context, uint16_t pulse_us)
{
    (void)context;
    if (pulse_us < CAR_SERVO_SAFETY_MIN_US ||
        pulse_us > CAR_SERVO_SAFETY_MAX_US) {
        return 0u;
    }
    /* This is the sole nonzero TIM1 CCR1 write site. */
    TIM1->CCR1 = pulse_us;
    __DSB();
    return TIM1->CCR1 == pulse_us ? 1u : 0u;
}

static uint32_t servo_read_compare(void *context)
{
    (void)context;
    return TIM1->CCR1;
}

static uint8_t servo_stop_timer_config_valid(uint8_t expect_stop_pending)
{
    const uint32_t ccmr_mask = (3u << 0u) | (1u << 2u) | (1u << 3u) |
                               (7u << 4u) | (1u << 16u);
    const uint32_t ccmr_expected = (1u << 3u) | (6u << 4u);
    const uint32_t expected_dier = expect_stop_pending != 0u
                                       ? TIM_DIER_CC1IE : 0u;
    return ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0u &&
            (RCC->CR & 0x00000005u) == 0x00000005u &&
            (RCC->CR & 0x00000018u) == 0u &&
            (RCC->CFGR & 0x0000803Fu) == 0u &&
            (RCC->D1CFGR & 0x00000F0Fu) == 0u &&
            (RCC->D2CFGR & (7u << 8u)) == 0u &&
            TIM1->PSC == 63u && TIM1->ARR == 19999u &&
            TIM1->RCR == 0u && TIM1->CNT <= TIM1->ARR &&
            TIM1->CR1 == TIM_CR1_CEN && TIM1->CR2 == 0u &&
            TIM1->SMCR == 0u && TIM1->DIER == expected_dier &&
            (TIM1->CCMR1 & ccmr_mask) == ccmr_expected &&
            TIM1->CCER == TIM_CCER_CC1E &&
            TIM1->BDTR == TIM_BDTR_MOE &&
            TIM1->CCR1 >= CAR_SERVO_SAFETY_MIN_US &&
            TIM1->CCR1 <= CAR_SERVO_SAFETY_MAX_US &&
            ((GPIOA->MODER >> 16u) & 3u) == 2u &&
            (GPIOA->AFR[1] & 0xFu) == 1u) ? 1u : 0u;
}

static uint8_t servo_hardware_start(void *context)
{
    (void)context;
    if (servo_stop_pending != 0u || servo_output_active != 0u ||
        TIM1->CCR1 != CAR_SERVO_SAFETY_CENTER_US ||
        TIM1->ARR != 19999u) {
        return 0u;
    }
    servo_stop_pending = 0u;
    car_servo_stop_pending = 0u;
    car_servo_stop_request_cnt_snapshot = 0u;
    car_servo_stop_complete_cnt_snapshot = 0u;
    TIM1->DIER = 0u;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER = 0u;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->SR = 0u;
    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    set_servo_gpio_af();
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->EGR = TIM_EGR_UG;
    __DSB();
    /* Start with a low phase at ARR.  The first timer tick wraps to zero and
       begins a complete centre pulse; no old compare or static-high wait is
       exposed before CEN runs. */
    TIM1->CNT = TIM1->ARR;
    /* Discard the software UG update; readiness requires a timer overflow. */
    TIM1->SR = 0u;
    TIM1->CCER = TIM_CCER_CC1E;
    TIM1->BDTR = TIM_BDTR_MOE;
    __DSB();
    TIM1->CR1 |= TIM_CR1_CEN;
    __DSB();
    car_servo_start_cnt_snapshot = TIM1->CNT;
    car_servo_last_update_compare_us_estimate = TIM1->CCR1;
    return ((TIM1->CR1 & TIM_CR1_CEN) != 0u &&
            (TIM1->CCER & TIM_CCER_CC1E) != 0u &&
            (TIM1->BDTR & TIM_BDTR_MOE) != 0u &&
            ((GPIOA->MODER >> 16u) & 3u) == 2u &&
            (GPIOA->AFR[1] & 0xFu) == 1u) ? 1u : 0u;
}

static uint8_t servo_take_completed_update(void *context)
{
    (void)context;
    if ((TIM1->SR & TIM_SR_UIF) == 0u) {
        return 0u;
    }
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    if (servo_output_active != 0u && servo_stop_pending == 0u) {
        car_servo_last_update_compare_us_estimate = TIM1->CCR1;
    }
    return 1u;
}

static void set_motor_gpio_low(void)
{
    GPIOA->BSRR = (uint32_t)APP_MOTOR_GPIOA_PINS << 16u;
    GPIOB->BSRR = (uint32_t)APP_MOTOR_GPIOB_PINS << 16u;
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 12u) | (3u << 14u))) |
                   (1u << 12u) | (1u << 14u);
    GPIOB->MODER = (GPIOB->MODER & ~((3u << 0u) | (3u << 2u))) |
                   (1u << 0u) | (1u << 2u);
}

static void stop_motor_outputs_hardware(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    (void)RCC->APB1LENR;
    TIM3->CCER &= ~APP_MOTOR_CHANNEL_MASK;
    TIM3->CCR1 = 0u;
    TIM3->CCR2 = 0u;
    TIM3->CCR3 = 0u;
    TIM3->CCR4 = 0u;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    __DSB();
    set_motor_gpio_low();
    motor_outputs_active = 0u;
    applied_motor_duty_permille = 0u;
    applied_motor_follow_left_permille = 0u;
    applied_motor_follow_right_permille = 0u;
    applied_motor_channel_mask = 0u;
    applied_follow_outputs = 0u;
    follow_ramp_started = 0u;
    car_follow_output_reset(&follow_output_state);
    follow_speed_pi_reset_pending = 1u;
    __DSB();
}

static void servo_hardware_stop_now(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    car_servo_stop_complete_cnt_snapshot = TIM1->CNT;
    TIM1->DIER = 0u;
    TIM1->SR = 0u;
    TIM1->CCER = 0u;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0u;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    __DSB();
    set_servo_gpio_low();
    servo_stop_pending = 0u;
    car_servo_stop_pending = 0u;
    servo_output_active = 0u;
    applied_servo_pulse_us = 0u;
    follow_ramp_started = 0u;
    car_follow_output_reset(&follow_output_state);
    follow_speed_pi_reset_pending = 1u;
    __DSB();
}

static void servo_latch_stop_timer_fault(void)
{
    const uint8_t first_fault = servo_safety.fault_latched == 0u ? 1u : 0u;
    /* This path is short and avoids HAL snapshots from the compare ISR. */
    if (first_fault != 0u) {
        car_servo_timer_snapshot_t *snapshot =
            &servo_safety.last_fault_snapshot;
        *snapshot = (car_servo_timer_snapshot_t){0};
        snapshot->rcc_cr = RCC->CR;
        snapshot->rcc_cfgr = RCC->CFGR;
        snapshot->rcc_d1cfgr = RCC->D1CFGR;
        snapshot->rcc_d2cfgr = RCC->D2CFGR;
        snapshot->rcc_apb2enr = RCC->APB2ENR;
        snapshot->tim_cr1 = TIM1->CR1;
        snapshot->tim_cr2 = TIM1->CR2;
        snapshot->tim_smcr = TIM1->SMCR;
        snapshot->tim_dier = TIM1->DIER;
        snapshot->tim_ccmr1 = TIM1->CCMR1;
        snapshot->tim_ccer = TIM1->CCER;
        snapshot->tim_bdtr = TIM1->BDTR;
        snapshot->tim_psc = TIM1->PSC;
        snapshot->tim_arr = TIM1->ARR;
        snapshot->tim_rcr = TIM1->RCR;
        snapshot->tim_ccr1 = TIM1->CCR1;
        snapshot->gpioa_moder = GPIOA->MODER;
        snapshot->gpioa_odr = GPIOA->ODR;
        snapshot->gpioa_afr1 = GPIOA->AFR[1];
        servo_safety.last_rejected_request_us = servo_safety.requested_us;
    }
    car_servo_safety_stop(&servo_safety, 0);
    if (first_fault != 0u) {
        servo_safety.fault_latched = 1u;
        servo_safety.sticky_fault_code = CAR_SERVO_FAULT_TIMER_CONFIG;
    }
    car_servo_fault_latched = servo_safety.fault_latched;
    car_servo_sticky_fault_code = servo_safety.sticky_fault_code;
    car_control_note_fast_stop_isr(
        &car_control_state,
        servo_safety.sticky_fault_code == CAR_SERVO_FAULT_TIMER_CLOCK
            ? CAR_FAULT_CLOCK : CAR_FAULT_TIMER);
}

static void servo_hardware_stop(void *context)
{
    uint32_t primask;
    uint32_t counter;
    uint32_t arr;
    uint8_t pin_high;
    uint8_t compare_event;
    car_servo_stop_action_t action;
    (void)context;

    primask = __get_PRIMASK();
    __disable_irq();
    if (servo_stop_pending != 0u) {
        __set_PRIMASK(primask);
        return;
    }
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    car_servo_stop_request_cnt_snapshot = TIM1->CNT;
    if (servo_output_active == 0u &&
        (TIM1->CR1 & TIM_CR1_CEN) == 0u &&
        (TIM1->CCER & TIM_CCER_CC1E) == 0u &&
        (TIM1->BDTR & TIM_BDTR_MOE) == 0u) {
        /* Inactive output is already safe; enforce the low GPIO state. */
        servo_hardware_stop_now();
        __set_PRIMASK(primask);
        return;
    }
    if (servo_output_active == 0u ||
        servo_stop_timer_config_valid(0u) == 0u) {
        servo_latch_stop_timer_fault();
        servo_hardware_stop_now();
        __set_PRIMASK(primask);
        return;
    }

    /* Clear only an old compare flag.  If the real falling edge occurs after
       this write, CC1IF remains set until CC1IE is enabled below. */
    TIM1->DIER &= ~TIM_DIER_CC1IE;
    TIM1->SR = ~TIM_SR_CC1IF;
    __DSB();
    counter = TIM1->CNT;
    arr = TIM1->ARR;
    pin_high = (GPIOA->IDR & APP_SERVO_GPIO_PIN) != 0u ? 1u : 0u;
    compare_event = (TIM1->SR & TIM_SR_CC1IF) != 0u ? 1u : 0u;
    action = car_servo_safety_stop_action(0u, compare_event, pin_high,
                                         counter, arr);
    if (action == CAR_SERVO_STOP_ACTION_CLOSE_NOW) {
        servo_hardware_stop_now();
    } else {
        servo_stop_pending = 1u;
        car_servo_stop_pending = 1u;
        __DMB();
        TIM1->DIER |= TIM_DIER_CC1IE;
        __DSB();
    }
    __set_PRIMASK(primask);
}

void TIM_CC_IRQHandler(void)
{
    uint32_t status = TIM1->SR;
    uint32_t counter;
    uint32_t arr;
    uint8_t pin_high;
    car_servo_stop_action_t action;

    if ((TIM1->DIER & TIM_DIER_CC1IE) == 0u ||
        (status & TIM_SR_CC1IF) == 0u) {
        return;
    }
    if (servo_stop_pending == 0u) {
        TIM1->DIER &= ~TIM_DIER_CC1IE;
        TIM1->SR = ~TIM_SR_CC1IF;
        __DSB();
        return;
    }
    if (servo_stop_timer_config_valid(1u) == 0u) {
        /* A changed clock/timer setup invalidates phase timing; fail low. */
        servo_latch_stop_timer_fault();
        servo_hardware_stop_now();
        return;
    }

    counter = TIM1->CNT;
    arr = TIM1->ARR;
    pin_high = (GPIOA->IDR & APP_SERVO_GPIO_PIN) != 0u ? 1u : 0u;
    action = car_servo_safety_stop_action(1u, 1u, pin_high, counter, arr);
    if (action == CAR_SERVO_STOP_ACTION_CLOSE_NOW) {
        servo_hardware_stop_now();
    } else {
        /* A stale compare request must not close the output in a new high
           pulse or at ARR wrap. Keep CC1IE armed for its next falling edge. */
        TIM1->SR = ~TIM_SR_CC1IF;
        __DSB();
        /* If the falling edge landed between the first IDR sample and the
           flag clear, finish now using the post-clear pin/phase state. An
           edge after this second sample leaves CC1IF pending for re-entry. */
        counter = TIM1->CNT;
        arr = TIM1->ARR;
        pin_high = (GPIOA->IDR & APP_SERVO_GPIO_PIN) != 0u ? 1u : 0u;
        action = car_servo_safety_stop_action(1u, pin_high == 0u ? 1u : 0u,
                                              pin_high,
                                              counter, arr);
        if (action == CAR_SERVO_STOP_ACTION_CLOSE_NOW) {
            servo_hardware_stop_now();
        }
    }
}

static void stop_servo_outputs_hardware(void)
{
    if (servo_safety_io.stop_output != 0) {
        car_servo_safety_stop(&servo_safety, &servo_safety_io);
    } else {
        servo_hardware_stop(0);
    }
}

static void emergency_stop_hardware(void)
{
    stop_motor_outputs_hardware();
    stop_servo_outputs_hardware();
    __DSB();
}

static void emergency_stop_hardware_immediate(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    stop_motor_outputs_hardware();
    car_servo_safety_stop(&servo_safety, 0);
    servo_hardware_stop_now();
    __DSB();
    __set_PRIMASK(primask);
}

void Car_FatalException(void)
{
    /* Fatal CPU/clock paths force the pin low immediately.  They cannot
       promise a complete pulse if the timer or execution clock has failed. */
    emergency_stop_hardware_immediate();
    for (;;) {
        __NOP();
    }
}

void Error_Handler(void)
{
    if (car_init_error == (uint32_t)CAR_INIT_ERROR_NONE) {
        car_init_error = (uint32_t)CAR_INIT_ERROR_UNCLASSIFIED;
    }
    if (car_init_hal_status == (uint32_t)HAL_OK) {
        car_init_hal_status = (uint32_t)HAL_ERROR;
    }
    if (car_fault == (uint32_t)CAR_FAULT_NONE) {
        car_fault = (uint32_t)CAR_FAULT_CONFIG;
    }
    init_capture_registers();
    Car_FatalException();
}

static void consume_fast_stop_latch(void)
{
    uint32_t primask = __get_PRIMASK();
    car_fault_t reason;
    __disable_irq();
    reason = car_control_take_fast_stop_isr(&car_control_state);
    __set_PRIMASK(primask);
    if (reason != CAR_FAULT_NONE) {
        car_control_note_fault(&car_control_state, reason);
    }
}

void NMI_Handler(void)
{
    car_fault = CAR_FAULT_CLOCK;
    Car_FatalException();
}

void HardFault_Handler(void)
{
    car_fault = CAR_FAULT_CONFIG;
    Car_FatalException();
}

void MemManage_Handler(void)
{
    car_fault = CAR_FAULT_CONFIG;
    Car_FatalException();
}

void BusFault_Handler(void)
{
    car_fault = CAR_FAULT_CONFIG;
    Car_FatalException();
}

void UsageFault_Handler(void)
{
    car_fault = CAR_FAULT_CONFIG;
    Car_FatalException();
}

void SysTick_Handler(void)
{
    uint32_t now_ms;
    uint8_t follow_active;
    uint8_t bench_active;
    uint8_t steering_active;
    HAL_IncTick();
    now_ms = HAL_GetTick();
    follow_active = app_config.follow_mode_compiled != 0u &&
        (manual_run_button.run_request != 0u ||
         car_control_state.follow_session_active != 0u ||
         applied_follow_outputs != 0u ||
         car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD);
    bench_active = app_config.bench_mode_compiled != 0u &&
        (car_control_state.mode == CAR_MODE_BENCH_RUNNING ||
         (motor_outputs_active != 0u && applied_follow_outputs == 0u));
    steering_active =
        app_config.debug_steering_mode_compiled != 0u &&
        car_control_state.steering_session_active != 0u;
    if (follow_active != 0u) {
        if (manual_run_button.run_request == 0u ||
            manual_run_button.stop_pending != 0u) {
            emergency_stop_hardware();
        } else if ((uint32_t)(now_ms - car_main_loop_heartbeat_ms) >=
                   APP_MAIN_LOOP_GAP_LIMIT_MS) {
            protected_fast_stop_note(CAR_FAULT_LOOP_GAP);
            emergency_stop_hardware();
        }
    } else if (bench_active != 0u) {
        if (pe7_is_high() != 0u) {
            protected_switch_release_note();
            emergency_stop_hardware();
        } else if ((uint32_t)(now_ms - car_main_loop_heartbeat_ms) >=
                   APP_MAIN_LOOP_GAP_LIMIT_MS) {
            protected_fast_stop_note(CAR_FAULT_LOOP_GAP);
            emergency_stop_hardware();
        }
    } else if (steering_active != 0u ||
               (app_config.debug_steering_mode_compiled != 0u &&
                servo_output_active != 0u)) {
        /* DebugSteering follows vision; PE7 does not gate its servo. */
        if (steering_active == 0u && pe7_is_high() != 0u) {
            emergency_stop_hardware();
        } else if ((uint32_t)(now_ms - car_main_loop_heartbeat_ms) >=
                   APP_MAIN_LOOP_GAP_LIMIT_MS) {
            protected_fast_stop_note(CAR_FAULT_LOOP_GAP);
            emergency_stop_hardware();
        }
    }
    HAL_SYSTICK_IRQHandler();
}

static HAL_StatusTypeDef system_clock_config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    HAL_StatusTypeDef status;

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    init_stage_begin(CAR_INIT_STAGE_CLOCK_OSC);
    status = init_status_record(CAR_INIT_ERROR_CLOCK_OSC,
                                HAL_RCC_OscConfig(&oscillator));
    if (status != HAL_OK) {
        return status;
    }

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                       RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV1;
    clocks.APB3CLKDivider = RCC_APB3_DIV1;
    clocks.APB1CLKDivider = RCC_APB1_DIV1;
    clocks.APB2CLKDivider = RCC_APB2_DIV1;
    clocks.APB4CLKDivider = RCC_APB4_DIV1;
    init_stage_begin(CAR_INIT_STAGE_CLOCK_TREE);
    status = init_status_record(CAR_INIT_ERROR_CLOCK_TREE,
                                HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2));
    if (status != HAL_OK) {
        return status;
    }
    __HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_DESACTIVATED);
    return HAL_OK;
}

static HAL_StatusTypeDef verify_system_and_timer_clocks(void)
{
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t flash_latency = 0u;
    uint8_t matches;
    HAL_StatusTypeDef status;

    init_stage_begin(CAR_INIT_STAGE_CLOCK_VERIFY);
    car_system_sysclk_hz = HAL_RCC_GetSysClockFreq();
    car_system_hclk_hz = HAL_RCC_GetHCLKFreq();
    car_system_pclk1_hz = HAL_RCC_GetPCLK1Freq();
    car_system_pclk2_hz = HAL_RCC_GetPCLK2Freq();
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    matches = (car_system_sysclk_hz == 64000000u &&
               car_system_hclk_hz == 64000000u &&
               car_system_pclk1_hz == 64000000u &&
               car_system_pclk2_hz == 64000000u &&
               clocks.SYSCLKSource == RCC_SYSCLKSOURCE_STATUS_HSI &&
               clocks.SYSCLKDivider == RCC_SYSCLK_DIV1 &&
               clocks.AHBCLKDivider == RCC_HCLK_DIV1 &&
               clocks.APB1CLKDivider == RCC_APB1_DIV1 &&
               clocks.APB2CLKDivider == RCC_APB2_DIV1 &&
               (RCC->CFGR & RCC_CFGR_TIMPRE) == 0u) ? 1u : 0u;
    status = init_status_record(CAR_INIT_ERROR_CLOCK_VERIFY,
                                matches != 0u ? HAL_OK : HAL_ERROR);
    return status;
}

static HAL_StatusTypeDef motor_pwm_init(void)
{
    TIM_OC_InitTypeDef output = {0};
    HAL_StatusTypeDef status;
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 63u;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999u;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_BASE);
    status = init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_BASE,
                                HAL_TIM_PWM_Init(&htim3));
    if (status != HAL_OK) {
        return status;
    }
    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0u;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;
    output.OCIdleState = TIM_OCIDLESTATE_RESET;
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_CH1);
    status = init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_CH1,
                                HAL_TIM_PWM_ConfigChannel(&htim3, &output, TIM_CHANNEL_1));
    if (status != HAL_OK) {
        return status;
    }
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_CH2);
    status = init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_CH2,
                                HAL_TIM_PWM_ConfigChannel(&htim3, &output, TIM_CHANNEL_2));
    if (status != HAL_OK) {
        return status;
    }
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_CH3);
    status = init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_CH3,
                                HAL_TIM_PWM_ConfigChannel(&htim3, &output, TIM_CHANNEL_3));
    if (status != HAL_OK) {
        return status;
    }
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_CH4);
    status = init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_CH4,
                                HAL_TIM_PWM_ConfigChannel(&htim3, &output, TIM_CHANNEL_4));
    if (status != HAL_OK) {
        return status;
    }
    TIM3->CCER = 0u;
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);
    TIM3->CCR1 = 0u;
    TIM3->CCR2 = 0u;
    TIM3->CCR3 = 0u;
    TIM3->CCR4 = 0u;
    TIM3->CNT = 0u;
    TIM3->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_ARPE);
    car_tim3_prescaler = TIM3->PSC;
    car_tim3_period = TIM3->ARR;
    init_stage_begin(CAR_INIT_STAGE_MOTOR_TIM3_VERIFY);
    return init_status_record(CAR_INIT_ERROR_MOTOR_TIM3_VERIFY,
                              (car_tim3_prescaler == 63u && car_tim3_period == 999u)
                                  ? HAL_OK : HAL_ERROR);
}

static HAL_StatusTypeDef servo_pwm_init(void)
{
    TIM_OC_InitTypeDef output = {0};
    HAL_StatusTypeDef status;
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63u;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999u;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0u;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    init_stage_begin(CAR_INIT_STAGE_SERVO_TIM1_BASE);
    status = init_status_record(CAR_INIT_ERROR_SERVO_TIM1_BASE,
                                HAL_TIM_PWM_Init(&htim1));
    if (status != HAL_OK) {
        return status;
    }
    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0u;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;
    output.OCIdleState = TIM_OCIDLESTATE_RESET;
    output.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    init_stage_begin(CAR_INIT_STAGE_SERVO_TIM1_CH1);
    status = init_status_record(CAR_INIT_ERROR_SERVO_TIM1_CH1,
                                HAL_TIM_PWM_ConfigChannel(&htim1, &output, TIM_CHANNEL_1));
    if (status != HAL_OK) {
        return status;
    }
    TIM1->CCER &= ~TIM_CCER_CC1E;
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCR1 = 0u;
    TIM1->CNT = 0u;
    TIM1->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_ARPE);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    NVIC_SetPriority(TIM1_CC_IRQn, 0u);
    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    car_tim1_prescaler = TIM1->PSC;
    car_tim1_period = TIM1->ARR;
    init_stage_begin(CAR_INIT_STAGE_SERVO_TIM1_VERIFY);
    return init_status_record(CAR_INIT_ERROR_SERVO_TIM1_VERIFY,
                              (car_tim1_prescaler == 63u && car_tim1_period == 19999u)
                                  ? HAL_OK : HAL_ERROR);
}

static HAL_StatusTypeDef encoder_init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef status;
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = APP_ENCODER_LEFT_A_GPIO_PIN | APP_ENCODER_LEFT_B_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = APP_ENCODER_RIGHT_A_GPIO_PIN | APP_ENCODER_RIGHT_B_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 0u;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 0u;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0u;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFu;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    init_stage_begin(CAR_INIT_STAGE_ENCODER_TIM2_CONFIG);
    status = init_status_record(CAR_INIT_ERROR_ENCODER_TIM2_CONFIG,
                                HAL_TIM_Encoder_Init(&htim2, &encoder));
    if (status != HAL_OK) {
        return status;
    }
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0u;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFFu;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0u;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    init_stage_begin(CAR_INIT_STAGE_ENCODER_TIM8_CONFIG);
    status = init_status_record(CAR_INIT_ERROR_ENCODER_TIM8_CONFIG,
                                HAL_TIM_Encoder_Init(&htim8, &encoder));
    if (status != HAL_OK) {
        return status;
    }
    init_stage_begin(CAR_INIT_STAGE_ENCODER_TIM2_START);
    status = init_status_record(CAR_INIT_ERROR_ENCODER_TIM2_START,
                                HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL));
    if (status != HAL_OK) {
        return status;
    }
    init_stage_begin(CAR_INIT_STAGE_ENCODER_TIM8_START);
    status = init_status_record(CAR_INIT_ERROR_ENCODER_TIM8_START,
                                HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL));
    if (status != HAL_OK) {
        return status;
    }
    car_encoder_diag_init(&car_encoder_diag, TIM2->CNT, (uint16_t)TIM8->CNT);
    car_speed_observer_init(
        &speed_observer, HAL_GetTick(), car_encoder_diag.left_raw_delta_sum,
        car_encoder_diag.right_raw_delta_sum,
        CARTEST_ENCODER_LEFT_COUNTS_PER_REV, CARTEST_ENCODER_LEFT_SIGN,
        CARTEST_ENCODER_RIGHT_COUNTS_PER_REV, CARTEST_ENCODER_RIGHT_SIGN);
    return HAL_OK;
}

static HAL_StatusTypeDef usart1_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pclk;
    HAL_StatusTypeDef status;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    /* USART1 kernel clock is explicitly PCLK2; do not inherit stale RCC state. */
    RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USART16SEL;
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    init_stage_begin(CAR_INIT_STAGE_USART1_VERIFY);
    pclk = HAL_RCC_GetPCLK2Freq();
    status = init_status_record(
        CAR_INIT_ERROR_USART1_VERIFY,
        (pclk == 64000000u &&
         (RCC->D2CCIP2R & RCC_D2CCIP2R_USART16SEL) == 0u)
            ? HAL_OK : HAL_ERROR);
    if (status != HAL_OK) {
        return status;
    }
    /* FIFOEN must be set while UE is clear. RXNEIE_RXFNEIE remains the
       nonempty-level interrupt; no FIFO threshold interrupt is enabled. */
    USART1->CR1 = 0u;
    USART1->CR1 = USART_CR1_FIFOEN;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;
    USART1->PRESC = 0u;
    USART1->BRR = (pclk + (APP_USART_BAUDRATE / 2u)) / APP_USART_BAUDRATE;
    USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    USART1->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE |
                  USART_CR1_RXNEIE_RXFNEIE;
    car_uart_rx_fifo_enabled =
        (USART1->CR1 & USART_CR1_FIFOEN) != 0u ? 1u : 0u;
    USART1->CR1 |= USART_CR1_UE;
    init_stage_begin(CAR_INIT_STAGE_USART1_ENABLE);
    status = init_status_record(CAR_INIT_ERROR_USART1_ENABLE,
                                ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE |
                                                 USART_CR1_RE | USART_CR1_FIFOEN |
                                                 USART_CR1_RXNEIE_RXFNEIE)) ==
                                 (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |
                                  USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE) &&
                                 (USART1->CR3 & USART_CR3_OVRDIS) == 0u &&
                                 USART1->BRR == 556u)
                                    ? HAL_OK : HAL_ERROR);
    if (status != HAL_OK) {
        return status;
    }
    NVIC_SetPriority(USART1_IRQn, 1u);
    NVIC_EnableIRQ(USART1_IRQn);
    return HAL_OK;
}

static HAL_StatusTypeDef switch_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef status;
    init_stage_begin(CAR_INIT_STAGE_SWITCH_GPIO);
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    gpio.Pin = APP_SWITCH_GPIO_PIN;
    gpio.Mode = app_config.follow_mode_compiled != 0u
                    ? GPIO_MODE_IT_RISING_FALLING : GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);
    __HAL_GPIO_EXTI_CLEAR_IT(APP_SWITCH_GPIO_PIN);
    NVIC_SetPriority(EXTI9_5_IRQn, 0u);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    status = init_status_record(CAR_INIT_ERROR_SWITCH_GPIO, HAL_OK);
    return status;
}

static HAL_StatusTypeDef iwdg_start(void)
{
    uint32_t start_ms;
    uint32_t status;
    const uint32_t busy_mask = IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU;

    /* Match ST's HAL_IWDG_Init sequence: enable first, then configure. */
    __HAL_DBGMCU_UnFreeze_IWDG1();
    init_stage_begin(CAR_INIT_STAGE_IWDG_ENABLE);
    IWDG1->KR = 0xCCCCu;
    (void)init_status_record(CAR_INIT_ERROR_NONE, HAL_OK);

    start_ms = HAL_GetTick();
    init_stage_begin(CAR_INIT_STAGE_IWDG_LSI_WAIT);
    car_init_lsi_wait_ms = 0u;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0u &&
           (uint32_t)(HAL_GetTick() - start_ms) < APP_IWDG_LSI_START_TIMEOUT_MS) {
        car_init_lsi_wait_ms = (uint32_t)(HAL_GetTick() - start_ms);
        car_init_rcc_csr_snapshot = RCC->CSR;
    }
    car_init_lsi_wait_ms = (uint32_t)(HAL_GetTick() - start_ms);
    if ((RCC->CSR & RCC_CSR_LSIRDY) == 0u) {
        return init_status_record(CAR_INIT_ERROR_IWDG_LSI_NOT_READY,
                                  HAL_TIMEOUT);
    }

    init_stage_begin(CAR_INIT_STAGE_IWDG_WRITE_ACCESS);
    IWDG1->KR = 0x5555u;
    (void)init_status_record(CAR_INIT_ERROR_NONE, HAL_OK);

    init_stage_begin(CAR_INIT_STAGE_IWDG_CONFIGURE);
    IWDG1->PR = APP_IWDG_PRESCALER_BITS;
    IWDG1->RLR = APP_IWDG_RELOAD_COUNTS - 1u;
    IWDG1->WINR = APP_IWDG_WINDOW_DISABLED;
    (void)init_status_record(CAR_INIT_ERROR_NONE, HAL_OK);

    start_ms = HAL_GetTick();
    init_stage_begin(CAR_INIT_STAGE_IWDG_UPDATE_WAIT);
    car_init_iwdg_update_wait_ms = 0u;
    car_init_iwdg_sr_snapshot = IWDG1->SR;
    while ((IWDG1->SR & busy_mask) != 0u &&
           (uint32_t)(HAL_GetTick() - start_ms) <
               APP_IWDG_REGISTER_UPDATE_TIMEOUT_MS) {
        car_init_iwdg_sr_snapshot = IWDG1->SR;
        car_init_rcc_csr_snapshot = RCC->CSR;
        car_init_iwdg_update_wait_ms = (uint32_t)(HAL_GetTick() - start_ms);
    }
    car_init_iwdg_update_wait_ms = (uint32_t)(HAL_GetTick() - start_ms);
    status = IWDG1->SR;
    car_init_iwdg_sr_snapshot = status;
    if ((status & busy_mask) != 0u) {
        return init_status_record(CAR_INIT_ERROR_IWDG_REGISTER_UPDATE_TIMEOUT,
                                  HAL_TIMEOUT);
    }

    init_stage_begin(CAR_INIT_STAGE_IWDG_VERIFY);
    car_init_iwdg_pr_snapshot = IWDG1->PR;
    car_init_iwdg_rlr_snapshot = IWDG1->RLR;
    car_init_iwdg_winr_snapshot = IWDG1->WINR;
    if (car_init_iwdg_pr_snapshot != APP_IWDG_PRESCALER_BITS ||
        car_init_iwdg_rlr_snapshot != (APP_IWDG_RELOAD_COUNTS - 1u) ||
        car_init_iwdg_winr_snapshot != APP_IWDG_WINDOW_DISABLED) {
        return init_status_record(CAR_INIT_ERROR_IWDG_READBACK, HAL_ERROR);
    }

    IWDG1->KR = 0xAAAAu;
    (void)init_status_record(CAR_INIT_ERROR_NONE, HAL_OK);
    iwdg_started = 1u;
    return HAL_OK;
}

static void iwdg_refresh(void)
{
    if (iwdg_started != 0u) {
        IWDG1->KR = 0xAAAAu;
    }
}

static uint8_t pe7_is_high(void)
{
    return (GPIOE->IDR & APP_SWITCH_GPIO_PIN) != 0u ? 1u : 0u;
}

static void debounce_update(uint32_t now_ms, uint8_t raw_high)
{
    if (raw_high != switch_debounce.candidate_level) {
        switch_debounce.candidate_level = raw_high;
        switch_debounce.candidate_since_ms = now_ms;
    } else if ((uint32_t)(now_ms - switch_debounce.candidate_since_ms) >= APP_SWITCH_DEBOUNCE_MS) {
        switch_debounce.stable_level = raw_high;
        switch_debounce.stable_valid = 1u;
    }
}

static car_inputs_t make_inputs(uint32_t now_ms, uint8_t raw_high, uint8_t loop_gap_fault)
{
    car_inputs_t inputs = {0};
    inputs.now_ms = now_ms;
    inputs.pe7_raw_high = raw_high;
    inputs.pe7_high_stable = switch_debounce.stable_valid != 0u &&
                             switch_debounce.stable_level != 0u;
    inputs.pe7_low_stable = switch_debounce.stable_valid != 0u &&
                            switch_debounce.stable_level == 0u;
    inputs.loop_gap_fault = loop_gap_fault;
    inputs.uart_fault = car_vision_rx.uart_fault;
    inputs.rx_overflow_fault = car_vision_rx.overflow_fault;
    inputs.rx_stale_fault = car_vision_rx.stale_fault;
    inputs.protocol_fault = car_vision_rx.protocol_fault;
    inputs.manual_run_request =
        app_config.follow_mode_compiled != 0u
            ? manual_run_button.run_request : 0u;
    inputs.manual_stop_pending =
        app_config.follow_mode_compiled != 0u
            ? manual_run_button.stop_pending : 0u;
    inputs.manual_button_startup_release_confirmed =
        app_config.follow_mode_compiled != 0u
            ? manual_run_button.startup_release_confirmed : 0u;
    return inputs;
}

static void note_applied_output_snapshot(void)
{
    car_control_note_applied_outputs(
        &car_control_state,
        servo_safety.frame_ready,
        servo_output_active,
        applied_servo_pulse_us,
        motor_outputs_active != 0u && applied_follow_outputs != 0u
            ? applied_motor_follow_left_permille : 0u,
        motor_outputs_active != 0u && applied_follow_outputs != 0u
            ? applied_motor_follow_right_permille : 0u);
}

static car_manual_button_event_t manual_button_sample(uint8_t raw_high,
                                                       uint32_t now_ms)
{
    uint32_t primask;
    car_manual_button_event_t event = CAR_MANUAL_BUTTON_EVENT_NONE;

    if (app_config.follow_mode_compiled == 0u) {
        return event;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    car_manual_button_poll_high(&manual_run_button, raw_high, now_ms);
    if (raw_high == 0u) {
        event = car_manual_button_note_low_edge(
            &manual_run_button, raw_high,
            car_control_state.fault_latched);
    }
    __set_PRIMASK(primask);

    if (event == CAR_MANUAL_BUTTON_EVENT_STOP) {
        /* Motors stop immediately; the existing servo path finishes its
           queued pulse before disabling PA8. Main consumes the full state. */
        emergency_stop_hardware();
    }
    return event;
}

static void cancel_manual_request_on_fault(void)
{
    uint32_t primask;
    if (app_config.follow_mode_compiled == 0u ||
        car_control_state.fault_latched == 0u) {
        return;
    }
    reverse_manual_stop_holdoff = 1u;
    primask = __get_PRIMASK();
    __disable_irq();
    car_manual_button_cancel_request(&manual_run_button);
    __set_PRIMASK(primask);
}

static void apply_manual_stop_effects(uint32_t effects)
{
    if ((effects & CAR_EFFECT_CLEAR_RX) != 0u) {
        flush_uart_and_parser();
    }
    if ((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u) {
        emergency_stop_hardware();
    }
    if ((effects & CAR_EFFECT_DISABLE_SERVO) != 0u) {
        stop_servo_outputs_hardware();
    }
}

static void consume_manual_stop_request(const car_inputs_t *inputs)
{
    uint32_t primask;
    uint32_t generation;
    uint32_t effects;

    if (inputs == 0 || app_config.follow_mode_compiled == 0u) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    generation = manual_run_button.stop_generation;
    __set_PRIMASK(primask);
    if (generation == manual_stop_consumed_generation) {
        return;
    }

    reverse_manual_stop_holdoff = 1u;
    effects = car_control_command(&car_control_state, &app_config, inputs,
                                  CAR_CMD_NORMAL_STOP, 0u, 0u, 0u);
    apply_manual_stop_effects(effects);
    follow_ramp_started = 0u;
    manual_stop_consumed_generation = generation;

    primask = __get_PRIMASK();
    __disable_irq();
    car_manual_button_stop_consumed(&manual_run_button);
    __set_PRIMASK(primask);
}

static void reset_session_odometry(void)
{
    session_left_millirev = 0u;
    session_right_millirev = 0u;
    session_mean_forward_millirev = 0u;
    odometry_left_fractional_remainder = 0u;
    odometry_right_fractional_remainder = 0u;
    odometry_have_baseline = 0u;
    odometry_valid_sample_count = 0u;
    odometry_rejected_sample_count = 0u;
}

static uint8_t normalize_forward_encoder_delta(
    int64_t current,
    int64_t previous,
    int8_t sign,
    uint64_t *forward_delta)
{
    int64_t delta;

    if (forward_delta == 0 || (sign != 1 && sign != -1) ||
        (previous > 0 && current < INT64_MIN + previous) ||
        (previous < 0 && current > INT64_MAX + previous)) {
        return 0u;
    }
    delta = current - previous;
    if (sign < 0) {
        if (delta == INT64_MIN) {
            return 0u;
        }
        delta = -delta;
    }
    if (delta < 0) {
        return 0u;
    }
    *forward_delta = (uint64_t)delta;
    return 1u;
}

static void update_session_odometry(void)
{
    const int64_t left_counts = car_encoder_diag.left_raw_delta_sum;
    const int64_t right_counts = car_encoder_diag.right_raw_delta_sum;
    uint64_t left_delta;
    uint64_t right_delta;
    uint64_t left_millirev;
    uint64_t right_millirev;
    uint64_t left_fractional_remainder =
        odometry_left_fractional_remainder;
    uint64_t right_fractional_remainder =
        odometry_right_fractional_remainder;

    if (car_control_state.follow_session_active == 0u) {
        odometry_have_baseline = 0u;
        return;
    }
    if (speed_observer_result.valid == 0u) {
        odometry_have_baseline = 0u;
        return;
    }
    if (speed_observer_result.status == CAR_SPEED_OBSERVER_STATUS_WAITING) {
        return;
    }
    if (speed_observer_result.status != CAR_SPEED_OBSERVER_STATUS_SAMPLE) {
        odometry_have_baseline = 0u;
        return;
    }
    if (odometry_have_baseline == 0u) {
        odometry_last_left_raw_counts = left_counts;
        odometry_last_right_raw_counts = right_counts;
        odometry_have_baseline = 1u;
        return;
    }
    if (normalize_forward_encoder_delta(
            left_counts, odometry_last_left_raw_counts,
            app_config.follow_left_encoder_sign, &left_delta) == 0u ||
        normalize_forward_encoder_delta(
            right_counts, odometry_last_right_raw_counts,
            app_config.follow_right_encoder_sign, &right_delta) == 0u ||
        car_encoder_counts_to_millirev(
            left_delta, app_config.follow_left_encoder_counts_per_rev,
            &left_fractional_remainder, &left_millirev) == 0u ||
        car_encoder_counts_to_millirev(
            right_delta, app_config.follow_right_encoder_counts_per_rev,
            &right_fractional_remainder, &right_millirev) == 0u ||
        left_millirev > UINT64_MAX - session_left_millirev ||
        right_millirev > UINT64_MAX - session_right_millirev) {
        odometry_have_baseline = 0u;
        ++odometry_rejected_sample_count;
        return;
    }

    odometry_last_left_raw_counts = left_counts;
    odometry_last_right_raw_counts = right_counts;
    odometry_left_fractional_remainder = left_fractional_remainder;
    odometry_right_fractional_remainder = right_fractional_remainder;
    session_left_millirev += left_millirev;
    session_right_millirev += right_millirev;
    session_mean_forward_millirev =
        (session_left_millirev / 2u) +
        (session_right_millirev / 2u) +
        (((session_left_millirev & 1u) +
          (session_right_millirev & 1u)) / 2u);
    ++odometry_valid_sample_count;
}

static void flush_uart_and_parser(void)
{
    uint32_t primask;
    uint32_t usart_status;
    uint8_t session_active;
    uint8_t pending_switch_release;
    uint8_t flush_pass;
    car_fault_t pending_fault;

    /* Close the outputs before establishing a new receive boundary. */
    emergency_stop_hardware();
    primask = __get_PRIMASK();
    __disable_irq();
    USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE;

    session_active = car_control_state.follow_session_active != 0u ||
                     car_control_state.mode == CAR_MODE_BENCH_RUNNING ||
                     motor_outputs_active != 0u;
    pending_switch_release =
        (EXTI->PR1 & APP_SWITCH_GPIO_PIN) != 0u ? 1u : 0u;
    usart_status = USART1->ISR;
    if ((usart_status & (USART_ISR_PE | USART_ISR_FE |
                         USART_ISR_NE | USART_ISR_ORE)) != 0u) {
        note_uart_error_locked(usart_status);
    }
    if (session_active != 0u && pending_switch_release != 0u &&
        app_config.follow_mode_compiled == 0u && pe7_is_high() != 0u) {
        /* Bench keeps its released-high stop; Follow handles PE7 separately. */
        car_control_note_switch_release_isr(&car_control_state);
    }
    /* Clear an old NVIC request before flushing the peripheral. A new byte
       arriving during this masked section will then pend a fresh interrupt. */
    NVIC_ClearPendingIRQ(USART1_IRQn);
    USART1->RQR = USART_RQR_RXFRQ;

    /* Capture any error that arrived during the flush before clearing flags. */
    for (flush_pass = 0u; flush_pass < 2u; ++flush_pass) {
        usart_status = USART1->ISR;
        if ((usart_status & (USART_ISR_PE | USART_ISR_FE |
                             USART_ISR_NE | USART_ISR_ORE)) == 0u) {
            break;
        }
        note_uart_error_locked(usart_status);
        USART1->RQR = USART_RQR_RXFRQ;
    }

    car_vision_rx_clear(&car_vision_rx);
    pending_fault = car_control_take_fast_stop_isr(&car_control_state);
    if (pending_fault != CAR_FAULT_NONE) {
        car_control_note_fault(&car_control_state, pending_fault);
    }
    car_control_rx_recovery_flush_complete(&car_control_state);
    /* FIFO flush and parser reset form the receive boundary. Re-enable only
       after stale hardware/software bytes and captured error flags are gone. */
    USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    __DSB();
    __set_PRIMASK(primask);
}

/* Capture a producer head, then read time while RX IRQ is briefly masked.
   Every byte consumed by this poll therefore has arrival_ms <= now_ms. */
static uint16_t vision_rx_snapshot(uint32_t *now_ms)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t head;
    __disable_irq();
    head = car_vision_rx.head;
    __DMB();
    *now_ms = HAL_GetTick();
    __set_PRIMASK(primask);
    return head;
}

static uint8_t profile_rx_path_healthy(void)
{
    return (uint8_t)(car_control_state.rx_recovery_flush_required == 0u &&
                     car_control_state.isr_stop_request == 0u &&
                     car_control_state.isr_stop_fault == CAR_FAULT_NONE &&
                     car_control_state.isr_release_seen == 0u &&
                     car_vision_rx.head == car_vision_rx.tail &&
                     car_vision_rx.partial_active == 0u &&
                     car_vision_rx.fault_generation ==
                         car_vision_rx.fault_generation_reported &&
                     car_vision_rx.isr_fault_generation ==
                         car_vision_rx.isr_fault_generation_reported &&
                     car_vision_rx.uart_fault == 0u &&
                     car_vision_rx.overflow_fault == 0u &&
                     car_vision_rx.protocol_fault == 0u &&
                     car_vision_rx.stale_fault == 0u);
}

static void service_physical_profile_operation(const car_inputs_t *inputs)
{
    uint32_t primask;
    uint32_t effects;

    if (inputs == 0 || CARTEST_BENCH_MODE_COMPILED == 0 ||
        car_control_state.profile_command_issued != 0u) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    effects = car_control_auto_select_bench_output(
        &car_control_state, &app_config, inputs,
        profile_rx_path_healthy());
    __set_PRIMASK(primask);

    if ((effects & CAR_EFFECT_CLEAR_RX) != 0u) {
        flush_uart_and_parser();
    }
    if ((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u) {
        emergency_stop_hardware();
    }
    if ((effects & CAR_EFFECT_DISABLE_SERVO) != 0u) {
        stop_servo_outputs_hardware();
    }
}

/* Keep the optional debugger mailbox for diagnosis and controlled stop/ACK
   commands. No normal operation depends on a Live Watch write. */
static void service_command_mailbox(const car_inputs_t *inputs)
{
    uint32_t primask;
    uint32_t mailbox;
    uint8_t selected_motor;
    uint8_t selected_input;
    uint16_t duty;
    uint32_t effects;

    if (inputs == 0) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    mailbox = car_command_mailbox;
    car_command_mailbox = CAR_CMD_NONE;
    selected_motor = (uint8_t)car_selected_motor;
    selected_input = (uint8_t)car_selected_input;
    duty = (uint16_t)car_requested_duty_permille;
    __set_PRIMASK(primask);
    if (mailbox == CAR_CMD_NONE) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    effects = car_control_command(&car_control_state, &app_config, inputs,
                                  (car_command_t)mailbox, selected_motor,
                                  selected_input, duty);
    __set_PRIMASK(primask);
    if ((effects & CAR_EFFECT_CLEAR_RX) != 0u) {
        flush_uart_and_parser();
    }
    if ((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u) {
        emergency_stop_hardware();
    }
    if ((effects & CAR_EFFECT_DISABLE_SERVO) != 0u) {
        stop_servo_outputs_hardware();
    }
}

static uint32_t motor_channel_enable_mask(uint8_t channel_mask)
{
    uint32_t enable_mask = 0u;

    if ((channel_mask & 0xF0u) != 0u) return 0u;
    if ((channel_mask & 0x01u) != 0u) enable_mask |= TIM_CCER_CC1E;
    if ((channel_mask & 0x02u) != 0u) enable_mask |= TIM_CCER_CC2E;
    if ((channel_mask & 0x04u) != 0u) enable_mask |= TIM_CCER_CC3E;
    if ((channel_mask & 0x08u) != 0u) enable_mask |= TIM_CCER_CC4E;
    return enable_mask;
}

static void output_generation_snapshot(output_generation_snapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == 0) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->control_fault = car_control_state.fault_generation;
    snapshot->control_rx_fault = car_control_state.rx_fault_generation;
    snapshot->control_isr_switch = car_control_state.isr_switch_generation;
    snapshot->control_isr_uart = car_control_state.isr_uart_generation;
    snapshot->rx_parser_fault = car_vision_rx.fault_generation;
    snapshot->rx_isr_fault = car_vision_rx.isr_fault_generation;
    __set_PRIMASK(primask);
}

static uint8_t motor_output_guard(
    uint8_t follow_mode,
    uint8_t expected_bench_channel_mask,
    const output_generation_snapshot_t *snapshot)
{
    const uint32_t now_ms = HAL_GetTick();
    uint32_t usart_status = USART1->ISR;
    uint8_t active_session;
    uint8_t common_safe;

#if !CARTEST_HARDWARE_ACCEPTED
    (void)follow_mode;
    (void)expected_bench_channel_mask;
    (void)snapshot;
    return 0u;
#endif

    if (snapshot == 0) {
        return 0u;
    }
    active_session = car_control_state.follow_session_active != 0u ||
                     car_control_state.mode == CAR_MODE_BENCH_RUNNING ||
                     motor_outputs_active != 0u;
    if ((usart_status & (USART_ISR_PE | USART_ISR_FE |
                         USART_ISR_NE | USART_ISR_ORE)) != 0u) {
        protected_uart_error_note(usart_status);
        return 0u;
    }
    if (follow_mode != 0u) {
        if (manual_run_button.run_request == 0u ||
            manual_run_button.stop_pending != 0u) {
            return 0u;
        }
    } else {
        if ((EXTI->PR1 & APP_SWITCH_GPIO_PIN) != 0u) {
            if (active_session != 0u && pe7_is_high() != 0u) {
                car_control_note_switch_release_isr(&car_control_state);
            }
            /* Bench release edges remain fail-safe in this non-Follow path. */
            return 0u;
        }
        if (pe7_is_high() != 0u) {
            if (active_session != 0u) {
                car_control_note_switch_release_isr(&car_control_state);
            }
            return 0u;
        }
    }

    common_safe =
        car_control_state.fault_latched == 0u &&
        car_control_state.fault == CAR_FAULT_NONE &&
        car_control_state.isr_stop_request == 0u &&
        car_control_state.isr_stop_fault == CAR_FAULT_NONE &&
        car_control_state.isr_release_seen == 0u &&
        car_control_state.rx_recovery_flush_required == 0u &&
        car_control_state.fault_generation == snapshot->control_fault &&
        car_control_state.rx_fault_generation == snapshot->control_rx_fault &&
        car_control_state.isr_switch_generation == snapshot->control_isr_switch &&
        car_control_state.isr_uart_generation == snapshot->control_isr_uart &&
        car_vision_rx.fault_generation == snapshot->rx_parser_fault &&
        car_vision_rx.isr_fault_generation == snapshot->rx_isr_fault &&
        car_vision_rx.fault_generation ==
            car_vision_rx.fault_generation_reported &&
        car_vision_rx.isr_fault_generation ==
            car_vision_rx.isr_fault_generation_reported &&
        car_vision_rx.uart_fault == 0u &&
        car_vision_rx.overflow_fault == 0u &&
        car_vision_rx.protocol_fault == 0u &&
        car_vision_rx.stale_fault == 0u &&
        servo_stop_pending == 0u &&
        app_config.motor_hardware_accepted != 0u;
    if (common_safe == 0u) {
        return 0u;
    }
    if (follow_mode != 0u) {
        if (app_config.follow_mode_compiled == 0u ||
            app_config.follow_calibration_ready == 0u ||
            app_config.follow_positive_error_right_measured == 0u ||
            car_control_state.follow_session_active == 0u ||
            car_control_state.road_drive_allowed == 0u ||
            car_control_state.manual_run_request == 0u) {
            return 0u;
        }
        if (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING) {
            return car_control_vision_fresh(&car_control_state, now_ms,
                                             app_config.vision_timeout_ms);
        }
        if (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
            car_control_state.vision_hold_active != 0u &&
            (uint32_t)(now_ms - car_control_state.vision_last_valid_ms) <
                CAR_CONTROL_VISION_HOLD_MS &&
            (uint32_t)(now_ms - car_control_state.vision_last_frame_alive_ms) <
                app_config.vision_timeout_ms) {
            return 1u;
        }
        return 0u;
    }
    if (app_config.bench_mode_compiled == 0u ||
        car_control_state.mode != CAR_MODE_BENCH_RUNNING ||
        expected_bench_channel_mask == 0u ||
        car_control_bench_channel_mask(&car_control_state) !=
            expected_bench_channel_mask) {
        car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
        return 0u;
    }
    return car_control_state.road_drive_allowed == 0u &&
           car_control_state.follow_session_active == 0u &&
           car_control_state.vision_valid != 0u &&
           car_control_vision_fresh(&car_control_state, now_ms,
                                    app_config.vision_timeout_ms) != 0u;
}

#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
static void speed_pi_diag_publish(uint32_t now_ms,
                                  uint8_t allow_new_sample,
                                  uint8_t force)
{
    car_speed_pi_diag_values_t values = {0};
    const uint8_t hold_active = car_control_state.vision_hold_active;
    const uint8_t observer_sample_seen =
        speed_observer_result.status == CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    const uint8_t observer_sample_new = observer_sample_seen != 0u &&
        (speed_pi_diag_have_observer_sample == 0u ||
         speed_observer_result.sample_ms !=
             speed_pi_diag_last_observer_sample_ms);
    const uint8_t proposal_valid =
        follow_incremental_pi_state.armed != 0u &&
        follow_incremental_pi_state.status ==
            CAR_FOLLOW_SPEED_PI_STATUS_RUNNING &&
        follow_speed_pi_reset_pending == 0u;
    const uint8_t new_sample = observer_sample_new != 0u &&
        allow_new_sample != 0u && hold_active == 0u ? 1u : 0u;
    const uint8_t armed = follow_incremental_pi_state.armed;
    const uint32_t status = follow_incremental_pi_state.status;
    const uint16_t request_left =
        follow_incremental_pi_state.proposal_left_permille;
    const uint16_t request_right =
        follow_incremental_pi_state.proposal_right_permille;
    const uint16_t applied_left = applied_motor_follow_left_permille;
    const uint16_t applied_right = applied_motor_follow_right_permille;
    const uint8_t changed = force != 0u || new_sample != 0u ||
        speed_pi_diag_have_values == 0u || status != speed_pi_diag_last_status ||
        armed != speed_pi_diag_last_armed ||
        hold_active != speed_pi_diag_last_hold ||
        request_left != speed_pi_diag_last_request_left ||
        request_right != speed_pi_diag_last_request_right ||
        applied_left != speed_pi_diag_last_applied_left ||
        applied_right != speed_pi_diag_last_applied_right;

    if (observer_sample_seen != 0u) {
        speed_pi_diag_have_observer_sample = 1u;
        speed_pi_diag_last_observer_sample_ms =
            speed_observer_result.sample_ms;
    }
    if (speed_pi_diag_session_active == 0u || changed == 0u) {
        return;
    }

    values.now_ms = now_ms;
    values.sample_ms = speed_observer_result.sample_ms;
    values.target_left_millirps =
        follow_incremental_pi_state.target_left_millirps;
    values.target_right_millirps =
        follow_incremental_pi_state.target_right_millirps;
    values.measured_left_millirps = speed_observer_result.left_millirps;
    values.measured_right_millirps = speed_observer_result.right_millirps;
    values.nominal_ff_left_permille =
        follow_incremental_pi_state.left.previous_feedforward_permille;
    values.nominal_ff_right_permille =
        follow_incremental_pi_state.right.previous_feedforward_permille;
    values.requested_left_permille = request_left;
    values.requested_right_permille = request_right;
    values.applied_left_permille = applied_left;
    values.applied_right_permille = applied_right;
    values.status = status;
    values.sample_elapsed_ms = speed_observer_result.elapsed_ms;
    values.enabled = 1u;
    values.armed = armed;
    values.new_speed_sample = new_sample;
    values.speed_sample_valid = speed_observer_result.valid;
    values.saturated_left = follow_incremental_pi_state.saturated_left;
    values.saturated_right = follow_incremental_pi_state.saturated_right;
    values.hold_active = hold_active;
    values.pi_proposal_valid = proposal_valid;
    (void)car_speed_pi_diag_update(&values);

    speed_pi_diag_have_values = 1u;
    speed_pi_diag_last_status = status;
    speed_pi_diag_last_armed = armed;
    speed_pi_diag_last_hold = hold_active;
    speed_pi_diag_last_request_left = request_left;
    speed_pi_diag_last_request_right = request_right;
    speed_pi_diag_last_applied_left = applied_left;
    speed_pi_diag_last_applied_right = applied_right;
}

static void speed_pi_diag_freeze_if_stopped(uint32_t now_ms)
{
    uint32_t termination_fault;
    /* Match the runtime logger's active Follow-session predicate exactly:
       recoverable vision/transport faults keep the manual session open. */
    const uint8_t terminal_state =
        manual_run_button.run_request == 0u ||
        car_control_state.fault_latched != 0u;

    if (speed_pi_diag_session_active == 0u || terminal_state == 0u) {
        return;
    }

    speed_pi_diag_publish(now_ms, 0u, 1u);
    termination_fault = (uint32_t)car_control_state.fault;
    if (termination_fault == (uint32_t)CAR_FAULT_NONE &&
        car_control_state.isr_stop_fault != CAR_FAULT_NONE) {
        termination_fault = (uint32_t)car_control_state.isr_stop_fault;
    }
    car_speed_pi_diag_freeze(now_ms, termination_fault);
    speed_pi_diag_session_active = 0u;
}
#endif

#if CARTEST_FOLLOW_SPEED_PI_ENABLED
static void speed_pi_trace_publish_sample(void)
{
    car_speed_pi_trace_values_t values = {0};
    const uint32_t primask = __get_PRIMASK();
    const uint8_t new_sample =
        speed_observer_result.status == CAR_SPEED_OBSERVER_STATUS_SAMPLE &&
        (speed_pi_trace_have_sample == 0u ||
         speed_observer_result.sample_ms != speed_pi_trace_last_sample_ms);

    if (speed_observer_result.status == CAR_SPEED_OBSERVER_STATUS_SAMPLE) {
        speed_pi_trace_have_sample = 1u;
        speed_pi_trace_last_sample_ms = speed_observer_result.sample_ms;
    }
    if (speed_pi_trace_session_active == 0u || new_sample == 0u) {
        return;
    }

    __disable_irq();
    values.record_ms = HAL_GetTick();
    values.sample_ms = speed_observer_result.sample_ms;
    values.vision_sequence = car_control_state.latest_rx_have_frame != 0u
        ? car_control_state.latest_rx_sequence : 0u;
    values.vision_arrival_ms = car_control_state.latest_rx_have_frame != 0u
        ? car_control_state.latest_rx_arrival_ms : 0u;
    values.pi_type = CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
        ? CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL
        : CAR_SPEED_PI_DIAG_TYPE_POSITION;
    values.speed_valid = speed_observer_result.valid;
    values.armed = APP_FOLLOW_PI_ARMED;
    values.hold_active = car_control_state.vision_hold_active;
    values.proposal_valid = APP_FOLLOW_PI_ARMED != 0u &&
        APP_FOLLOW_PI_STATUS == CAR_FOLLOW_SPEED_PI_STATUS_RUNNING &&
        follow_speed_pi_reset_pending == 0u;
    values.target_left_millirps = APP_FOLLOW_PI_TARGET_LEFT;
    values.target_right_millirps = APP_FOLLOW_PI_TARGET_RIGHT;
    values.measured_left_millirps = speed_observer_result.left_millirps;
    values.measured_right_millirps = speed_observer_result.right_millirps;
    values.requested_left_permille = APP_FOLLOW_PI_REQUEST_LEFT;
    values.requested_right_permille = APP_FOLLOW_PI_REQUEST_RIGHT;
    values.applied_left_permille = applied_motor_follow_left_permille;
    values.applied_right_permille = applied_motor_follow_right_permille;
    __set_PRIMASK(primask);
    car_speed_pi_trace_append(&values);
}

static void speed_pi_trace_freeze_if_stopped(uint32_t now_ms)
{
    const uint8_t terminal_state =
        manual_run_button.run_request == 0u ||
        car_control_state.fault_latched != 0u;

    if (speed_pi_trace_session_active == 0u || terminal_state == 0u) {
        return;
    }
    /* CRC work is limited per call so serial and main-loop safety servicing
       continue after outputs stop. The trace remains ACTIVE/pending until its
       complete chronological CRC is published. */
    if (car_speed_pi_trace_freeze(now_ms) != 0u) {
        speed_pi_trace_session_active = 0u;
    }
}
#endif

static void consume_follow_speed_pi_reset_request(void)
{
    const uint32_t primask = __get_PRIMASK();
    uint8_t reset_requested;
    __disable_irq();
    reset_requested = follow_speed_pi_reset_pending;
    follow_speed_pi_reset_pending = 0u;
    __set_PRIMASK(primask);
    if (reset_requested != 0u) {
        car_follow_speed_pi_reset(&follow_speed_pi_state);
        car_follow_incremental_pi_reset(&follow_incremental_pi_state);
    }
}

static void prepare_follow_motor_targets(uint32_t now_ms)
{
    uint8_t fault_or_stop;
    uint8_t follow_ramp_due;
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
    uint8_t startup_qualified_before_update;
#endif

#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
    speed_pi_diag_freeze_if_stopped(now_ms);
#endif
    consume_follow_speed_pi_reset_request();

    fault_or_stop = car_control_state.fault_latched != 0u ||
                    car_control_state.fault != CAR_FAULT_NONE ||
                    car_control_state.isr_stop_request != 0u ||
                    car_control_state.isr_stop_fault != CAR_FAULT_NONE ||
                    car_control_state.isr_release_seen != 0u ||
                    servo_safety.fault_latched != 0u ||
                    servo_stop_pending != 0u;

    if (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
        car_control_state.vision_hold_active != 0u &&
        car_control_state.manual_run_request != 0u && fault_or_stop == 0u) {
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
        if (follow_output_state.startup_qualified != 0u &&
            (speed_observer_result.status >=
                 CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG ||
             (uint32_t)(now_ms - speed_observer_result.sample_ms) >
                 CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS ||
             (APP_FOLLOW_PI_ARMED != 0u &&
              speed_observer_result.valid == 0u) ||
             (speed_observer_result.valid == 0u &&
              speed_observer_result.status !=
                  CAR_SPEED_OBSERVER_STATUS_NOT_READY &&
              speed_observer_result.status !=
                  CAR_SPEED_OBSERVER_STATUS_WAITING))) {
            car_control_note_fault(&car_control_state,
                                   CAR_FAULT_ENCODER_INIT);
            emergency_stop_hardware();
            return;
        }
#endif
        follow_output_state.slope_assist_active = 0u;
        car_follow_output_hold_at(
            &follow_output_state, now_ms,
            car_control_state.vision_hold_started_ms,
            car_control_state.vision_hold_entries,
            car_control_state.vision_hold_motor_left_permille,
            car_control_state.vision_hold_motor_right_permille);
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
        {
            const car_follow_speed_pi_input_t pi_input = {
                .enabled = 1u,
                .vision_hold = 1u,
                .sample_ms = speed_observer_result.sample_ms
            };
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
            (void)car_follow_incremental_pi_update(
                &follow_incremental_pi_state, &pi_input);
#else
            (void)car_follow_speed_pi_update(&follow_speed_pi_state,
                                             &pi_input);
#endif
        }
#endif
        return;
    }

    if (car_control_state.mode != CAR_MODE_FOLLOW_RUNNING ||
        car_control_state.manual_run_request == 0u ||
        car_control_state.follow_session_active == 0u ||
        car_control_state.road_drive_allowed == 0u || fault_or_stop != 0u) {
        car_follow_output_reset(&follow_output_state);
        car_follow_speed_pi_reset(&follow_speed_pi_state);
        car_follow_incremental_pi_reset(&follow_incremental_pi_state);
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
        ackermann_control_session_ready = 0u;
#endif
        return;
    }

    follow_ramp_due = follow_output_state.have_ramp_time != 0u &&
        (uint32_t)(now_ms - follow_output_state.last_ramp_ms) >=
            CAR_FOLLOW_OUTPUT_RAMP_PERIOD_MS;
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
    startup_qualified_before_update = follow_output_state.startup_qualified;
#endif

#if CARTEST_ACKERMANN_COORDINATION_ENABLED
    {
        uint16_t steering_model_pulse_us = applied_servo_pulse_us;

        /* Servo startup deliberately begins with software-applied pulse 0.
           Until a complete TIM1 frame is confirmed, model straight steering
           at the legal configured center; the independent full-frame and
           motor-ramp gates below remain authoritative. */
        if (servo_output_active == 0u || servo_safety.frame_ready == 0u) {
            steering_model_pulse_us = app_config.servo_center_us;
        } else if (steering_model_pulse_us < app_config.follow_servo_left_us ||
                   steering_model_pulse_us > app_config.follow_servo_right_us) {
            car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
            emergency_stop_hardware();
            car_follow_speed_pi_reset(&follow_speed_pi_state);
            car_follow_incremental_pi_reset(&follow_incremental_pi_state);
            return;
        }
    if (ackermann_control_session_ready == 0u) {
        car_ackermann_control_reset(&ackermann_control_state);
        ackermann_control_session_ready = 1u;
    }
    if (car_ackermann_control_update(
            &ackermann_control_state, now_ms,
            car_control_state.vision_curve_phase,
            car_control_state.vision_slowdown_request_permille,
            steering_model_pulse_us, app_config.servo_center_us,
            app_config.follow_servo_left_us,
            app_config.follow_servo_right_us,
            &ackermann_control_output) == 0u) {
        car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
        emergency_stop_hardware();
        car_follow_speed_pi_reset(&follow_speed_pi_state);
        car_follow_incremental_pi_reset(&follow_incremental_pi_state);
        return;
    }
    }
#endif

    if (car_follow_output_update(
            &follow_output_state, now_ms,
            car_control_state.follow_output_duty_permille,
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
            0u, 0u,
#else
            CARTEST_FOLLOW_DIFF_PERMILLE,
            CARTEST_FOLLOW_INNER_DIFF_PERMILLE,
#endif
            applied_servo_pulse_us,
            app_config.servo_center_us, app_config.follow_servo_left_us,
            app_config.follow_servo_right_us, servo_output_active,
            servo_safety.frame_ready,
            motor_outputs_active != 0u && applied_follow_outputs != 0u,
            applied_motor_follow_left_permille,
            applied_motor_follow_right_permille,
            &follow_output_state.command_left_permille,
            &follow_output_state.command_right_permille) == 0u) {
        car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
        emergency_stop_hardware();
        car_follow_speed_pi_reset(&follow_speed_pi_state);
        car_follow_incremental_pi_reset(&follow_incremental_pi_state);
        follow_speed_pi_reset_pending = 0u;
        return;
    }

#if CARTEST_FOLLOW_SPEED_PI_ENABLED
    {
        const uint32_t sample_age_ms =
            (uint32_t)(now_ms - speed_observer_result.sample_ms);
        const uint8_t startup_qualified =
            follow_output_state.startup_qualified;
        const uint8_t outputs_active =
            motor_outputs_active != 0u && applied_follow_outputs != 0u;
        const uint8_t servo_ready = servo_output_active != 0u &&
                                   servo_safety.frame_ready != 0u &&
                                   servo_stop_pending == 0u;

        /* Once startup is qualified, encoder failures stop the trial instead
           of silently returning to open-loop output. */
        if (startup_qualified != 0u &&
            (speed_observer_result.status >=
                 CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG ||
             sample_age_ms > CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS ||
             (APP_FOLLOW_PI_ARMED != 0u &&
              speed_observer_result.valid == 0u) ||
             (speed_observer_result.valid == 0u &&
              speed_observer_result.status !=
                  CAR_SPEED_OBSERVER_STATUS_NOT_READY &&
              speed_observer_result.status !=
                  CAR_SPEED_OBSERVER_STATUS_WAITING))) {
            car_speed_pi_actuation_enabled = 0u;
            car_control_note_fault(&car_control_state,
                                   CAR_FAULT_ENCODER_INIT);
            emergency_stop_hardware();
            return;
        }

        {
            const car_follow_speed_pi_input_t pi_input = {
                .enabled = 1u,
                .follow_running = 1u,
                /* Preserve the first full qualified servo frame at the
                   legacy 500-permille startup command. PI arms next sample. */
                .startup_qualified = startup_qualified != 0u &&
                                     startup_qualified_before_update != 0u &&
                                     (speed_observer_result.valid != 0u ||
                                      APP_FOLLOW_PI_ARMED != 0u),
                .actual_motor_outputs = outputs_active,
                .servo_ready = servo_ready,
                .speed_sample_valid = speed_observer_result.valid,
                .new_speed_sample =
                    speed_observer_result.status ==
                        CAR_SPEED_OBSERVER_STATUS_SAMPLE,
                .sample_ms = speed_observer_result.sample_ms,
                .sample_age_ms = sample_age_ms,
                .sample_elapsed_ms = speed_observer_result.elapsed_ms,
                .measured_left_millirps =
                    speed_observer_result.left_millirps,
                .measured_right_millirps =
                    speed_observer_result.right_millirps,
                .nominal_left_permille =
                    follow_output_state.target_left_permille,
                .nominal_right_permille =
                    follow_output_state.target_right_permille,
                .applied_left_permille =
                    applied_motor_follow_left_permille,
                .applied_right_permille =
                    applied_motor_follow_right_permille
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
                , .target_override_valid = 1u,
                .target_left_millirps =
                    ackermann_control_output.left_target_millirps,
                .target_right_millirps =
                    ackermann_control_output.right_target_millirps
#endif
            };
            uint32_t pi_status;
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
            pi_status = car_follow_incremental_pi_update(
                &follow_incremental_pi_state, &pi_input);
#else
            pi_status = car_follow_speed_pi_update(
                &follow_speed_pi_state, &pi_input);
#endif

            if (pi_status == CAR_FOLLOW_SPEED_PI_STATUS_INVALID) {
                car_speed_pi_actuation_enabled = 0u;
                car_control_note_fault(&car_control_state,
                                       CAR_FAULT_ENCODER_INIT);
                emergency_stop_hardware();
                return;
            }
            if (pi_status == CAR_FOLLOW_SPEED_PI_STATUS_RUNNING) {
                uint8_t ramp_ok;
                car_speed_pi_actuation_enabled =
                    APP_FOLLOW_PI_ARMED != 0u ? 1u : 0u;
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
                ramp_ok = car_follow_incremental_pi_ramp(
                        &follow_incremental_pi_state, follow_ramp_due,
                        CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE,
                        applied_motor_follow_left_permille,
                        applied_motor_follow_right_permille,
                        &follow_output_state.command_left_permille,
                        &follow_output_state.command_right_permille);
#else
                ramp_ok = car_follow_speed_pi_ramp(
                        &follow_speed_pi_state, follow_ramp_due,
                        applied_motor_follow_left_permille,
                        applied_motor_follow_right_permille,
                        &follow_output_state.command_left_permille,
                        &follow_output_state.command_right_permille);
#endif
                if (ramp_ok == 0u) {
                    car_speed_pi_actuation_enabled = 0u;
                    car_control_note_fault(&car_control_state,
                                           CAR_FAULT_ENCODER_INIT);
                    emergency_stop_hardware();
                    return;
                }
            } else {
                car_speed_pi_actuation_enabled = 0u;
            }
        }
    }
#else
    (void)follow_ramp_due;
#endif
}

static void motor_apply(void)
{
    uint32_t primask;
    output_generation_snapshot_t generations;
    uint16_t duty;
    uint16_t left_duty;
    uint16_t right_duty;
    uint8_t channel_mask = 0u;
    uint8_t pi_zero_coast_allowed = 0u;
    uint32_t cc_enable;
    uint8_t follow_mode = 0u;
    uint8_t remap_channels;
#if !CARTEST_HARDWARE_ACCEPTED
    app_config.motor_hardware_accepted = 0u;
    car_control_state.road_drive_allowed = 0u;
    car_control_state.follow_output_duty_permille = 0u;
    car_control_state.bench_output_duty_permille = 0u;
    if (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
        car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD ||
        car_control_state.mode == CAR_MODE_BENCH_RUNNING) {
        car_control_state.mode = CAR_MODE_WAIT_START_EDGE;
        car_control_state.follow_session_active = 0u;
        car_control_state.bench_motor = CAR_MOTOR_NONE;
        car_control_state.bench_requested_duty_permille = 0u;
        car_control_state.frame_edge_count =
            car_control_state.button_low_edge_count;
    }
    car_motor_hardware_lock_reason = 1u;
    primask = __get_PRIMASK();
    __disable_irq();
    stop_motor_outputs_hardware();
    __set_PRIMASK(primask);
    return;
#endif
    if (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
        (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
         car_control_state.vision_hold_active != 0u)) {
        follow_mode = 1u;
        left_duty = follow_output_state.command_left_permille;
        right_duty = follow_output_state.command_right_permille;
        duty = left_duty > right_duty ? left_duty : right_duty;
        cc_enable = TIM_CCER_CC2E | TIM_CCER_CC3E;
    } else if (car_control_state.mode == CAR_MODE_BENCH_RUNNING) {
        duty = car_control_state.bench_output_duty_permille;
        left_duty = duty;
        right_duty = duty;
        channel_mask = car_control_bench_channel_mask(&car_control_state);
        cc_enable = motor_channel_enable_mask(channel_mask);
    } else {
        primask = __get_PRIMASK();
        __disable_irq();
        stop_motor_outputs_hardware();
        __set_PRIMASK(primask);
        return;
    }

    if (duty == 0u && follow_mode != 0u && motor_outputs_active == 0u) {
        return;
    }
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
    pi_zero_coast_allowed =
        follow_mode != 0u && duty == 0u &&
        APP_FOLLOW_PI_ARMED != 0u &&
        APP_FOLLOW_PI_STATUS == CAR_FOLLOW_SPEED_PI_STATUS_RUNNING &&
        follow_output_state.startup_qualified != 0u &&
        car_control_state.mode == CAR_MODE_FOLLOW_RUNNING &&
        car_control_state.manual_run_request != 0u &&
        car_control_state.follow_session_active != 0u &&
        car_control_state.road_drive_allowed != 0u &&
        motor_outputs_active != 0u && applied_follow_outputs != 0u &&
        servo_output_active != 0u && servo_safety.frame_ready != 0u &&
        servo_stop_pending == 0u;
#endif
    if (cc_enable == 0u ||
        duty > (follow_mode != 0u ? APP_FOLLOW_HARD_LIMIT_PERMILLE
                                  : APP_BENCH_LIMIT_PERMILLE) ||
        left_duty > (follow_mode != 0u ? APP_FOLLOW_HARD_LIMIT_PERMILLE
                                       : APP_BENCH_LIMIT_PERMILLE) ||
        right_duty > (follow_mode != 0u ? APP_FOLLOW_HARD_LIMIT_PERMILLE
                                        : APP_BENCH_LIMIT_PERMILLE) ||
        (follow_mode != 0u &&
         (app_config.follow_max_duty_permille !=
              APP_FOLLOW_NORMAL_LIMIT_PERMILLE ||
          left_duty > app_config.follow_max_duty_permille ||
          right_duty > app_config.follow_max_duty_permille))) {
        car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
        emergency_stop_hardware();
        return;
    }
    if (duty == 0u && pi_zero_coast_allowed == 0u) {
        if (motor_outputs_active != 0u) {
            primask = __get_PRIMASK();
            __disable_irq();
            stop_motor_outputs_hardware();
            __set_PRIMASK(primask);
        }
        return;
    }
    output_generation_snapshot(&generations);
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
        emergency_stop_hardware();
        __set_PRIMASK(primask);
        return;
    }
    if (follow_mode != 0u &&
        (servo_output_active == 0u || servo_safety.frame_ready == 0u)) {
        if (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
            car_control_state.vision_hold_active != 0u) {
            emergency_stop_hardware();
        }
        __set_PRIMASK(primask);
        return;
    }

    if (follow_mode != 0u && motor_outputs_active == 0u &&
        follow_ramp_started == 0u) {
        car_control_state.follow_output_duty_permille = 0u;
        car_control_state.follow_last_ramp_ms = HAL_GetTick();
        follow_ramp_started = 1u;
        car_follow_output_reset(&follow_output_state);
        __set_PRIMASK(primask);
        return;
    }

    if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
        emergency_stop_hardware();
        __set_PRIMASK(primask);
        return;
    }

    if (follow_mode != 0u) {
        if (motor_outputs_active != 0u && applied_follow_outputs != 0u &&
            left_duty == applied_motor_follow_left_permille &&
            right_duty == applied_motor_follow_right_permille) {
            __set_PRIMASK(primask);
            return;
        }
    } else if (motor_outputs_active != 0u && applied_follow_outputs == 0u &&
               duty == applied_motor_duty_permille &&
               channel_mask == applied_motor_channel_mask) {
        __set_PRIMASK(primask);
        return;
    }

    remap_channels = motor_outputs_active == 0u ||
        ((follow_mode != 0u) != (applied_follow_outputs != 0u)) ||
        (follow_mode == 0u && channel_mask != applied_motor_channel_mask);
    if (remap_channels != 0u) {
        TIM3->CCER = 0u;
        TIM3->CCR1 = 0u;
        TIM3->CCR2 = 0u;
        TIM3->CCR3 = 0u;
        TIM3->CCR4 = 0u;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->CNT = 0u;
        set_motor_gpio_low();
        motor_outputs_active = 0u;
        set_motor_gpio_channels_af(cc_enable);
        if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
            emergency_stop_hardware();
            __set_PRIMASK(primask);
            return;
        }
    }

    if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
        emergency_stop_hardware();
        __set_PRIMASK(primask);
        return;
    }
    if ((cc_enable & TIM_CCER_CC1E) != 0u) TIM3->CCR1 = duty;
    else if (motor_outputs_active == 0u || applied_follow_outputs != 0u) TIM3->CCR1 = 0u;
    if ((cc_enable & TIM_CCER_CC2E) != 0u) TIM3->CCR2 = follow_mode != 0u ? left_duty : duty;
    else if (motor_outputs_active == 0u || applied_follow_outputs != 0u) TIM3->CCR2 = 0u;
    if ((cc_enable & TIM_CCER_CC3E) != 0u) TIM3->CCR3 = follow_mode != 0u ? right_duty : duty;
    else if (motor_outputs_active == 0u || applied_follow_outputs != 0u) TIM3->CCR3 = 0u;
    if ((cc_enable & TIM_CCER_CC4E) != 0u) TIM3->CCR4 = duty;
    else if (motor_outputs_active == 0u || applied_follow_outputs != 0u) TIM3->CCR4 = 0u;

    if (motor_outputs_active == 0u) {
        /* Run the counter with outputs disabled, then recheck immediately
           before the PWM pins are connected to the driver inputs. */
        TIM3->CNT = 0u;
        TIM3->CCER = 0u;
        TIM3->CR1 |= TIM_CR1_CEN;
        if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
            emergency_stop_hardware();
            __set_PRIMASK(primask);
            return;
        }
        TIM3->CCER = cc_enable;
    } else {
        TIM3->CCER = cc_enable;
    }
    if (motor_output_guard(follow_mode, channel_mask, &generations) == 0u) {
        emergency_stop_hardware();
        __set_PRIMASK(primask);
        return;
    }
    motor_outputs_active = 1u;
    applied_motor_duty_permille = duty;
    applied_motor_follow_left_permille = follow_mode != 0u ? left_duty : 0u;
    applied_motor_follow_right_permille = follow_mode != 0u ? right_duty : 0u;
    applied_motor_channel_mask = follow_mode != 0u ? 0xFFu : channel_mask;
    applied_follow_outputs = follow_mode;
    if (follow_mode == 0u && app_config.bench_mode_compiled != 0u &&
        car_control_state.mode == CAR_MODE_BENCH_RUNNING &&
        motor_outputs_active != 0u &&
        (TIM3->CCER & cc_enable) == cc_enable &&
        duty > car_bench_max_applied_duty_permille) {
        car_bench_max_applied_duty_permille = duty;
    }
    __DSB();
    __set_PRIMASK(primask);
}

static uint8_t debug_steering_output_guard(
    const output_generation_snapshot_t *snapshot)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t usart_status = USART1->ISR;

    if (snapshot == 0) {
        return 0u;
    }
    if ((usart_status & (USART_ISR_PE | USART_ISR_FE |
                         USART_ISR_NE | USART_ISR_ORE)) != 0u) {
        protected_uart_error_note(usart_status);
        return 0u;
    }

    /* This independent servo gate intentionally ignores PE7, EXTI pending,
       isr_release_seen, and the switch generation. All non-switch fault
       evidence remains mandatory. */
    if (app_config.debug_steering_mode_compiled == 0u ||
        car_control_state.mode != CAR_MODE_DEBUG_STEERING_RUNNING ||
        car_control_state.steering_session_active == 0u ||
        car_control_state.follow_session_active != 0u ||
        car_control_state.road_drive_allowed != 0u ||
        car_control_state.fault_latched != 0u ||
        car_control_state.fault != CAR_FAULT_NONE ||
        car_control_state.isr_stop_fault != CAR_FAULT_NONE ||
        car_control_state.rx_recovery_flush_required != 0u ||
        car_control_state.fault_generation != snapshot->control_fault ||
        car_control_state.rx_fault_generation != snapshot->control_rx_fault ||
        car_control_state.isr_uart_generation != snapshot->control_isr_uart ||
        car_vision_rx.fault_generation != snapshot->rx_parser_fault ||
        car_vision_rx.isr_fault_generation != snapshot->rx_isr_fault ||
        car_vision_rx.fault_generation !=
            car_vision_rx.fault_generation_reported ||
        car_vision_rx.isr_fault_generation !=
            car_vision_rx.isr_fault_generation_reported ||
        car_vision_rx.uart_fault != 0u ||
        car_vision_rx.overflow_fault != 0u ||
        car_vision_rx.protocol_fault != 0u ||
        car_vision_rx.stale_fault != 0u ||
        car_control_state.vision_valid == 0u ||
        car_control_vision_fresh(&car_control_state, now_ms,
                                 app_config.vision_timeout_ms) == 0u ||
        motor_outputs_active != 0u ||
        (TIM3->CCER & APP_MOTOR_CHANNEL_MASK) != 0u ||
        TIM3->CCR1 != 0u || TIM3->CCR2 != 0u ||
        TIM3->CCR3 != 0u || TIM3->CCR4 != 0u ||
        (TIM3->CR1 & TIM_CR1_CEN) != 0u) {
        return 0u;
    }
    return 1u;
}

static void servo_apply(void)
{
    uint32_t primask;
    const uint16_t follow_pulse =
        car_control_follow_servo_output_target_us(&car_control_state);
    const uint16_t debug_pulse = car_control_state.steering_servo_target_us;
    output_generation_snapshot_t generations;
    const uint8_t follow_mode =
        car_control_state.follow_session_active != 0u &&
        car_control_state.manual_run_request != 0u &&
        (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
         (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
          car_control_state.vision_hold_active != 0u));
    const uint8_t steering_mode =
        app_config.debug_steering_mode_compiled != 0u &&
        car_control_state.mode == CAR_MODE_DEBUG_STEERING_RUNNING &&
        car_control_state.steering_session_active != 0u;
    const uint16_t slew_step_us =
        follow_mode != 0u && follow_output_state.startup_qualified != 0u
            ? CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US
            : CAR_SERVO_SAFETY_SLEW_STEP_US;
    const car_servo_source_t selected_source =
        follow_mode != 0u ? CAR_SERVO_SOURCE_FOLLOW
                          : CAR_SERVO_SOURCE_DEBUG_STEERING;

    /* A pending stop owns TIM1 through its next complete falling edge. */
    if (servo_stop_pending != 0u) {
        return;
    }

#if !CARTEST_HARDWARE_ACCEPTED
    if (follow_mode != 0u) {
        primask = __get_PRIMASK();
        __disable_irq();
        (void)car_servo_safety_request_selected_with_slew(
            &servo_safety, selected_source, follow_pulse, debug_pulse,
            0u, HAL_GetTick(), slew_step_us, &servo_safety_io);
        if (servo_safety.fault_latched != 0u) {
            servo_safety_fault_to_controller();
        }
        __set_PRIMASK(primask);
        return;
    }
#endif

    if (follow_mode != 0u) {
        uint16_t low;
        uint16_t high;
        low = app_config.follow_servo_left_us < app_config.follow_servo_right_us
                  ? app_config.follow_servo_left_us
                  : app_config.follow_servo_right_us;
        high = app_config.follow_servo_left_us > app_config.follow_servo_right_us
                   ? app_config.follow_servo_left_us
                   : app_config.follow_servo_right_us;
        if (app_config.follow_mode_compiled == 0u ||
            app_config.follow_calibration_ready == 0u ||
            app_config.follow_positive_error_right_measured == 0u ||
            low < CAR_SERVO_SAFETY_MIN_US ||
            high > CAR_SERVO_SAFETY_MAX_US || low > high) {
            (void)car_servo_safety_request_selected_with_slew(
                &servo_safety, selected_source, follow_pulse, debug_pulse,
                0u, HAL_GetTick(), slew_step_us, &servo_safety_io);
            if (servo_safety.fault_latched != 0u) {
                servo_safety_fault_to_controller();
            }
            car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
            emergency_stop_hardware();
            return;
        }

        output_generation_snapshot(&generations);
        primask = __get_PRIMASK();
        __disable_irq();
        if (motor_output_guard(1u, 0u, &generations) == 0u) {
            (void)car_servo_safety_request_selected_with_slew(
                &servo_safety, selected_source, follow_pulse, debug_pulse,
                0u, HAL_GetTick(), slew_step_us, &servo_safety_io);
            if (servo_safety.fault_latched != 0u) {
                servo_safety_fault_to_controller();
            }
            emergency_stop_hardware();
            __set_PRIMASK(primask);
            return;
        }
    } else if (steering_mode != 0u) {
        output_generation_snapshot(&generations);
        primask = __get_PRIMASK();
        __disable_irq();
        if (debug_steering_output_guard(&generations) == 0u) {
            if (motor_outputs_active != 0u ||
                (TIM3->CCER & APP_MOTOR_CHANNEL_MASK) != 0u ||
                TIM3->CCR1 != 0u || TIM3->CCR2 != 0u ||
                TIM3->CCR3 != 0u || TIM3->CCR4 != 0u) {
                stop_motor_outputs_hardware();
                car_control_note_fault(&car_control_state, CAR_FAULT_CONFIG);
            } else if (car_control_state.fault == CAR_FAULT_NONE &&
                       car_control_vision_fresh(
                           &car_control_state, HAL_GetTick(),
                           app_config.vision_timeout_ms) == 0u) {
                car_control_note_fault(&car_control_state,
                                       CAR_FAULT_VISION_TIMEOUT);
            }
            (void)car_servo_safety_request_selected_with_slew(
                &servo_safety, selected_source, follow_pulse, debug_pulse,
                0u, HAL_GetTick(), slew_step_us, &servo_safety_io);
            if (servo_safety.fault_latched != 0u) {
                servo_safety_fault_to_controller();
            }
            stop_servo_outputs_hardware();
            __set_PRIMASK(primask);
            return;
        }
    } else {
        if (servo_safety.active != 0u || servo_output_active != 0u ||
            (TIM1->CR1 & TIM_CR1_CEN) != 0u ||
            (TIM1->CCER & TIM_CCER_CC1E) != 0u ||
            (TIM1->BDTR & TIM_BDTR_MOE) != 0u) {
            primask = __get_PRIMASK();
            __disable_irq();
            stop_servo_outputs_hardware();
            __set_PRIMASK(primask);
        }
        return;
    }

    if (servo_safety.fault_latched != 0u ||
        car_servo_safety_request_selected_with_slew(
            &servo_safety, selected_source, follow_pulse, debug_pulse,
            1u, HAL_GetTick(), slew_step_us, &servo_safety_io) == 0u) {
        if (servo_safety.fault_latched != 0u) {
            servo_safety_fault_to_controller();
        }
        __set_PRIMASK(primask);
        return;
    }
    servo_output_active = servo_safety.active;
    applied_servo_pulse_us = servo_safety.compare_us;
    __DSB();
    __set_PRIMASK(primask);
}

static void servo_safety_fault_to_controller(void)
{
    car_fault_t fault = servo_safety.sticky_fault_code ==
                                CAR_SERVO_FAULT_TIMER_CLOCK
                            ? CAR_FAULT_CLOCK : CAR_FAULT_TIMER;
    if (car_control_state.fault_latched == 0u ||
        car_control_state.fault != fault) {
        car_control_note_fault(&car_control_state, fault);
    }
    if (servo_output_active != 0u ||
        (TIM1->CR1 & TIM_CR1_CEN) != 0u ||
        (TIM1->CCER & TIM_CCER_CC1E) != 0u ||
        (TIM1->BDTR & TIM_BDTR_MOE) != 0u) {
        emergency_stop_hardware();
    }
}

static void enforce_motor_hardware_lock(void)
{
#if CARTEST_HARDWARE_ACCEPTED
    app_config.motor_hardware_accepted = 1u;
    car_motor_hardware_lock_reason = 0u;
#else
    app_config.motor_hardware_accepted = 0u;
    car_motor_hardware_lock_reason = 1u;
    car_control_state.road_drive_allowed = 0u;
    car_control_state.follow_output_duty_permille = 0u;
    car_control_state.bench_output_duty_permille = 0u;
    if (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
        car_control_state.mode == CAR_MODE_BENCH_RUNNING) {
        car_control_state.follow_session_active = 0u;
        car_control_state.bench_motor = CAR_MOTOR_NONE;
        car_control_state.bench_requested_duty_permille = 0u;
        car_control_state.mode = CAR_MODE_WAIT_START_EDGE;
        car_control_state.frame_edge_count =
            car_control_state.button_low_edge_count;
    }
#endif
}

static void service_reverse_control(uint32_t now_ms)
{
    uint8_t prepared;
    uint8_t keep_permitted;
    uint8_t soft_session_fault;
    uint8_t byte;

    if (reverse_control_ready == 0u) {
        return;
    }

    car_reverse_control_set_profile_state(
        &reverse_control_tx, (uint8_t)CARTEST_TRACK_PROFILE,
        car_control_state.follow_session_active, follow_session_token,
        session_mean_forward_millirev > UINT32_MAX
            ? UINT32_MAX : (uint32_t)session_mean_forward_millirev);

    if (car_control_state.reverse_control_reset_generation !=
        reverse_control_seen_generation) {
        reverse_control_seen_generation =
            car_control_state.reverse_control_reset_generation;
        car_reverse_control_request_reset(&reverse_control_tx, now_ms);
        car_control_reverse_control_reset_started(&car_control_state);
        reverse_keep_notified = 0u;
    }

    if (car_control_state.rx_fault_generation !=
        reverse_control_seen_rx_fault_generation) {
        reverse_control_seen_rx_fault_generation =
            car_control_state.rx_fault_generation;
        reverse_comm_outage = 1u;
    }
    if (reverse_control_tx.reset_phase_complete != 0u) {
        car_control_reverse_control_reset_phase_complete(&car_control_state);
    }
    if (reverse_comm_outage != 0u &&
        car_control_state.reverse_reset_transport_gate_open != 0u &&
        car_control_state.reverse_transport_recovery_frame_count >=
            CAR_CONTROL_RECOVERY_REQUIRED_FRAMES &&
        car_control_state.vision_have_frame_sequence != 0u &&
        (uint32_t)(now_ms - car_control_state.vision_last_frame_alive_ms) <
            app_config.vision_timeout_ms) {
        reverse_comm_outage = 0u;
    }

    prepared = (uint8_t)(app_config.debug_steering_mode_compiled != 0u ||
                         car_control_state.startup_release_seen != 0u);
    soft_session_fault = (uint8_t)(
        car_control_state.fault == CAR_FAULT_UART ||
        car_control_state.fault == CAR_FAULT_RX_OVERFLOW ||
        car_control_state.fault == CAR_FAULT_RX_STALE ||
        car_control_state.fault == CAR_FAULT_PROTOCOL ||
        car_control_state.fault == CAR_FAULT_SEQUENCE ||
        car_control_state.fault == CAR_FAULT_VISION_INVALID ||
        car_control_state.fault == CAR_FAULT_VISION_TIMEOUT);
    keep_permitted = (uint8_t)(prepared != 0u &&
        reverse_comm_outage == 0u &&
        reverse_manual_stop_holdoff == 0u &&
        car_control_state.fault_latched == 0u &&
        (car_control_state.fault == CAR_FAULT_NONE ||
         (manual_run_button.run_request != 0u &&
          soft_session_fault != 0u)));

    /* With FIFOEN, TXE_TXFNF means "FIFO has space" rather than the old
       single-byte DR-empty condition. Preserve the prior bounded pacing by
       enqueueing at most one protocol byte per main-loop service call. */
    if ((USART1->ISR & USART_ISR_TXE_TXFNF) != 0u &&
        car_reverse_control_next_byte_paired(
            &reverse_control_tx, now_ms, keep_permitted, &byte) != 0u) {
        USART1->TDR = byte;
        if (reverse_control_tx.keep_active != 0u &&
            reverse_keep_notified == 0u) {
            reverse_keep_notified = 1u;
            car_control_reverse_control_keep_started(&car_control_state);
        }
    }

    car_reverse_control_sequence = reverse_control_tx.sequence;
    car_reverse_control_command = reverse_control_tx.active_command;
    car_reverse_control_reset_generation =
        reverse_control_tx.reset_generation;
    car_reverse_control_reset_phase_complete =
        reverse_control_tx.reset_phase_complete;
    car_reverse_control_keep_active = reverse_control_tx.keep_active;
    car_reverse_control_fresh_frames =
        car_control_state.reverse_control_fresh_frames;
    car_profile_state_sequence = reverse_control_tx.profile_sequence;
    car_profile_state_last_tx_ms = reverse_control_tx.profile_last_tx_ms;
    car_profile_state_tx_count = reverse_control_tx.profile_tx_count;
    car_profile_state_id = reverse_control_tx.profile_id;
    car_profile_state_session_active = reverse_control_tx.profile_flags & 1u;
    car_session_token = follow_session_token;
    car_session_mean_forward_wheel_millirev =
        session_mean_forward_millirev > UINT32_MAX
            ? UINT32_MAX : (uint32_t)session_mean_forward_millirev;
    car_session_odom_valid_samples = odometry_valid_sample_count;
    car_session_odom_invalid_samples = odometry_rejected_sample_count;
}

static void publish_encoder_hardware_snapshot(void)
{
    const uint32_t apb2_enr = RCC->APB2ENR;

    /* Read live hardware independently of encoder_diag's accepted software
       snapshot. These registers are part of normal encoder fault diagnosis. */
    car_enc_tim8_cnt_hw = (uint32_t)TIM8->CNT;
    car_enc_tim8_cen = (TIM8->CR1 & TIM_CR1_CEN) != 0u ? 1u : 0u;
    car_enc_tim8_clock_enabled =
        (apb2_enr & RCC_APB2ENR_TIM8EN) != 0u ? 1u : 0u;
    car_enc_tim8_smcr = TIM8->SMCR;
    car_enc_tim8_ccmr1 = TIM8->CCMR1;
    car_enc_tim8_ccer = TIM8->CCER;
}

/* Callers hold the interrupt mask while pairing these committed software
   outputs with the rest of a debug or runtime-log snapshot. */
static motor_applied_channels_t motor_applied_channels_snapshot(void)
{
    motor_applied_channels_t channels = {0};

    if (motor_outputs_active == 0u) {
        return channels;
    }
    if (applied_follow_outputs != 0u) {
        channels.left_in2_permille = applied_motor_follow_left_permille;
        channels.right_in1_permille = applied_motor_follow_right_permille;
        return channels;
    }

    if ((applied_motor_channel_mask & 0x01u) != 0u) {
        channels.left_in1_permille = applied_motor_duty_permille;
    }
    if ((applied_motor_channel_mask & 0x02u) != 0u) {
        channels.left_in2_permille = applied_motor_duty_permille;
    }
    if ((applied_motor_channel_mask & 0x04u) != 0u) {
        channels.right_in1_permille = applied_motor_duty_permille;
    }
    if ((applied_motor_channel_mask & 0x08u) != 0u) {
        channels.right_in2_permille = applied_motor_duty_permille;
    }
    return channels;
}

static void publish_motor_channel_debug_snapshot(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const motor_applied_channels_t channels =
        motor_applied_channels_snapshot();

    car_motor_left_in1_output_permille = channels.left_in1_permille;
    car_motor_left_in2_output_permille = channels.left_in2_permille;
    car_motor_right_in1_output_permille = channels.right_in1_permille;
    car_motor_right_in2_output_permille = channels.right_in2_permille;
    car_motor_pwm_ccr2 = TIM3->CCR2;
    car_motor_pwm_ccr3 = TIM3->CCR3;
    car_motor_pwm_ccer = TIM3->CCER;
    car_motor_pwm_enabled =
        (TIM3->CR1 & TIM_CR1_CEN) != 0u &&
        (TIM3->CCER & APP_MOTOR_CHANNEL_MASK) != 0u ? 1u : 0u;
    __set_PRIMASK(primask);
}

static void publish_debug_snapshot(const car_inputs_t *inputs)
{
    car_run_block_reason_input_t block_reason = {0};
    uint32_t snapshot_now_ms = HAL_GetTick();
    publish_encoder_hardware_snapshot();
    car_mode = (uint32_t)car_control_state.mode;
    car_fault = (uint32_t)car_control_state.fault;
#if CARTEST_HARDWARE_ACCEPTED
    car_drive_allowed = car_control_state.road_drive_allowed;
#else
    car_drive_allowed = 0u;
#endif
    car_pe7_high_stable = switch_debounce.stable_valid != 0u &&
                          switch_debounce.stable_level != 0u;
    car_pe7_low_stable = switch_debounce.stable_valid != 0u &&
                         switch_debounce.stable_level == 0u;
    car_vision_valid = car_control_state.vision_valid;
    car_vision_timeout = car_control_state.vision_timeout;
    car_vision_confidence = car_control_state.vision_confidence;
    car_vision_error_norm = car_control_state.vision_error_norm;
    car_vision_sequence = car_control_state.vision_sequence;
    car_vision_rx_frames = car_control_state.latest_rx_frame_count;
    car_latest_rx_have_frame = car_control_state.latest_rx_have_frame;
    car_latest_rx_valid = car_control_state.latest_rx_valid;
    car_latest_rx_slope_hint = car_control_state.latest_rx_slope_hint;
    car_vision_slope_hint = car_control_state.vision_slope_hint;
    car_slope_assist_active = 0u;
    car_latest_rx_confidence = car_control_state.latest_rx_confidence;
    car_latest_rx_error_norm = car_control_state.latest_rx_error_norm;
    car_latest_rx_sequence = car_control_state.latest_rx_sequence;
    car_latest_rx_arrival_ms = car_control_state.latest_rx_arrival_ms;
    car_latest_rx_frame_count = car_control_state.latest_rx_frame_count;
    car_camera_profile_required_ack =
        app_config.required_camera_profile_ack;
    car_camera_profile_latest_ack =
        car_control_state.latest_rx_profile_ack;
    car_camera_profile_latest_version =
        car_control_state.latest_rx_protocol_version;
    car_camera_profile_ack_ready =
        car_control_camera_profile_ack_ready(
            &car_control_state, &app_config, snapshot_now_ms);
    car_camera_profile_ack_seen_after_reset =
        car_control_state.profile_ack_seen_after_reset;
    car_camera_profile_ack_age_ms =
        car_control_state.latest_rx_have_frame != 0u
            ? snapshot_now_ms -
                car_control_state.latest_rx_profile_ack_arrival_ms
            : 0u;
    car_camera_profile_last_ack_ms =
        car_control_state.latest_rx_profile_ack_arrival_ms;
    car_camera_profile_mismatch_count =
        car_control_state.profile_ack_mismatch_count;
    car_recovery_frame_count = car_control_state.recovery_frame_count;
    car_recovery_active = car_control_state.recovery_active;
    car_auto_recovery_count = car_control_state.auto_recovery_count;
    car_follow_session_active = car_control_state.follow_session_active;
    car_steering_session_active = car_control_state.steering_session_active;
    car_manual_run_request = manual_run_button.run_request;
    car_button_press_event_count = manual_run_button.press_event_count;
    car_button_start_event_count = manual_run_button.start_event_count;
    car_button_stop_generation = manual_run_button.stop_generation;
    car_button_armed = manual_run_button.armed;
    /* Pending means the latched request has not reached ready Follow output:
       session, road permission, first PWM readiness, and both applied sides. */
    car_start_pending =
        manual_run_button.run_request != 0u &&
        !(car_control_state.follow_session_active != 0u &&
          car_control_state.road_drive_allowed != 0u &&
          servo_safety.frame_ready != 0u && servo_output_active != 0u &&
          motor_outputs_active != 0u && applied_follow_outputs != 0u &&
          applied_motor_follow_left_permille != 0u &&
          applied_motor_follow_right_permille != 0u);
    car_vision_hold_active = car_control_state.vision_hold_active;
    car_vision_hold_elapsed_ms = 0u;
    if (car_control_state.vision_hold_active != 0u) {
        uint32_t hold_elapsed = (uint32_t)(snapshot_now_ms -
                                      car_control_state.vision_last_valid_ms);
        if (hold_elapsed > CAR_CONTROL_VISION_HOLD_MS) {
            hold_elapsed = CAR_CONTROL_VISION_HOLD_MS;
        }
        car_vision_hold_elapsed_ms = hold_elapsed;
    } else if (car_control_state.vision_hold_expired != 0u) {
        car_vision_hold_elapsed_ms = CAR_CONTROL_VISION_HOLD_MS;
    }
    car_vision_hold_entries = car_control_state.vision_hold_entries;
    car_vision_hold_exit_cause = car_control_state.vision_hold_exit_cause;
    car_steering_servo_target_us = car_control_state.steering_servo_target_us;
    car_rx_flush_pending = car_control_state.rx_recovery_flush_required;
    car_uart_rx_bytes = car_vision_rx.rx_byte_count;
    car_uart_error_count = car_vision_rx.uart_error_count;
    car_uart_overflow_count = car_vision_rx.queue_overflow_count;
    car_protocol_error_count = car_vision_rx.protocol_error_count;
    car_uart_partial_timeout_count = car_vision_rx.partial_timeout_count;
    car_uart_queue_age_reject_count = car_vision_rx.queue_age_reject_count;
    car_uart_discarded_byte_count = car_vision_rx.discarded_byte_count;
    car_encoder_left_raw = car_encoder_diag.left_raw;
    car_encoder_right_raw = car_encoder_diag.right_raw;
    car_encoder_left_raw_delta_sum = car_encoder_diag.left_raw_delta_sum;
    car_encoder_right_raw_delta_sum = car_encoder_diag.right_raw_delta_sum;
    car_encoder_ambiguous_delta_count = car_encoder_diag.ambiguous_delta_count;
    car_speed_observer_valid = speed_observer_result.valid;
    car_speed_observer_status = speed_observer_result.status;
    car_speed_observer_elapsed_ms = speed_observer_result.elapsed_ms;
    car_speed_observer_sample_age_ms =
        snapshot_now_ms - speed_observer_result.sample_ms;
    car_speed_left_millirps = speed_observer_result.valid != 0u
        ? speed_observer_result.left_millirps : 0;
    car_speed_right_millirps = speed_observer_result.valid != 0u
        ? speed_observer_result.right_millirps : 0;
    car_speed_left_rpm = speed_observer_result.valid != 0u
        ? speed_observer_result.left_rpm : 0;
    car_speed_right_rpm = speed_observer_result.valid != 0u
        ? speed_observer_result.right_rpm : 0;
    car_speed_pi_actuation_enabled =
        CARTEST_FOLLOW_SPEED_PI_ENABLED != 0 &&
        CARTEST_HARDWARE_ACCEPTED != 0 &&
        follow_speed_pi_reset_pending == 0u &&
        APP_FOLLOW_PI_ARMED != 0u &&
        APP_FOLLOW_PI_STATUS == CAR_FOLLOW_SPEED_PI_STATUS_RUNNING &&
        follow_output_state.startup_qualified != 0u &&
        car_control_state.mode == CAR_MODE_FOLLOW_RUNNING &&
        car_control_state.manual_run_request != 0u &&
        car_control_state.follow_session_active != 0u &&
        car_control_state.road_drive_allowed != 0u &&
        car_control_state.fault == CAR_FAULT_NONE &&
        car_control_state.fault_latched == 0u &&
        car_control_state.isr_stop_request == 0u &&
        car_control_state.isr_stop_fault == CAR_FAULT_NONE &&
        car_control_state.isr_release_seen == 0u &&
        motor_outputs_active != 0u && applied_follow_outputs != 0u &&
        servo_output_active != 0u && servo_safety.frame_ready != 0u &&
        servo_stop_pending == 0u && servo_safety.fault_latched == 0u &&
        speed_observer_result.valid != 0u &&
        (uint32_t)(snapshot_now_ms - speed_observer_result.sample_ms) <=
            CAR_FOLLOW_SPEED_PI_MAX_SAMPLE_AGE_MS;
    car_speed_pi_compiled = CARTEST_FOLLOW_SPEED_PI_ENABLED;
    car_speed_pi_status = APP_FOLLOW_PI_STATUS;
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
    car_speed_pi_type = CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL;
    car_speed_pi_accumulator_left_q16 =
        follow_incremental_pi_state.left.accumulator_q16;
    car_speed_pi_accumulator_right_q16 =
        follow_incremental_pi_state.right.accumulator_q16;
#else
    car_speed_pi_type = CARTEST_FOLLOW_SPEED_PI_ENABLED
        ? CAR_SPEED_PI_DIAG_TYPE_POSITION : CAR_SPEED_PI_DIAG_TYPE_NONE;
    car_speed_pi_accumulator_left_q16 = 0;
    car_speed_pi_accumulator_right_q16 = 0;
#endif
    car_speed_pi_target_left_millirps =
        APP_FOLLOW_PI_TARGET_LEFT > 0
            ? (uint32_t)APP_FOLLOW_PI_TARGET_LEFT : 0u;
    car_speed_pi_target_right_millirps =
        APP_FOLLOW_PI_TARGET_RIGHT > 0
            ? (uint32_t)APP_FOLLOW_PI_TARGET_RIGHT : 0u;
    car_speed_pi_requested_left_permille =
        APP_FOLLOW_PI_REQUEST_LEFT;
    car_speed_pi_requested_right_permille =
        APP_FOLLOW_PI_REQUEST_RIGHT;
    car_speed_pi_saturated_left = APP_FOLLOW_PI_SATURATED_LEFT;
    car_speed_pi_saturated_right = APP_FOLLOW_PI_SATURATED_RIGHT;
    car_speed_pi_integral_left_q16 = follow_speed_pi_state.left.integral_q16;
    car_speed_pi_integral_right_q16 = follow_speed_pi_state.right.integral_q16;
    publish_motor_channel_debug_snapshot();
    car_servo_output_active = servo_output_active;
    car_servo_stop_pending = servo_stop_pending;
    car_servo_pa8_idr_high =
        (GPIOA->IDR & APP_SERVO_GPIO_PIN) != 0u ? 1u : 0u;
    car_servo_applied_pulse_us = applied_servo_pulse_us;
    car_servo_requested_us = servo_safety.requested_us;
    car_servo_limited_us = servo_safety.limited_us;
    car_servo_ccr1_software_us = servo_safety.compare_us;
    car_servo_timer_psc = TIM1->PSC;
    car_servo_timer_arr = TIM1->ARR;
    car_servo_fault_latched = servo_safety.fault_latched;
    car_servo_sticky_fault_code = servo_safety.sticky_fault_code;
    car_servo_frame_ready = servo_safety.frame_ready;
    car_servo_last_rejected_request_us =
        servo_safety.last_rejected_request_us;
    car_pe7_release_event_count =
        car_control_state.isr_switch_generation;
    car_servo_fault_snapshot_cr1 = servo_safety.last_fault_snapshot.tim_cr1;
    car_servo_fault_snapshot_cr2 = servo_safety.last_fault_snapshot.tim_cr2;
    car_servo_fault_snapshot_smcr = servo_safety.last_fault_snapshot.tim_smcr;
    car_servo_fault_snapshot_dier = servo_safety.last_fault_snapshot.tim_dier;
    car_servo_fault_snapshot_ccmr1 =
        servo_safety.last_fault_snapshot.tim_ccmr1;
    car_servo_fault_snapshot_ccer = servo_safety.last_fault_snapshot.tim_ccer;
    car_servo_fault_snapshot_bdtr = servo_safety.last_fault_snapshot.tim_bdtr;
    car_servo_fault_snapshot_psc = servo_safety.last_fault_snapshot.tim_psc;
    car_servo_fault_snapshot_arr = servo_safety.last_fault_snapshot.tim_arr;
    car_servo_fault_snapshot_rcr = servo_safety.last_fault_snapshot.tim_rcr;
    car_servo_fault_snapshot_ccr1 = servo_safety.last_fault_snapshot.tim_ccr1;
    car_servo_fault_snapshot_rcc_cr =
        servo_safety.last_fault_snapshot.rcc_cr;
    car_servo_fault_snapshot_rcc_cfgr =
        servo_safety.last_fault_snapshot.rcc_cfgr;
    car_servo_fault_snapshot_d1cfgr =
        servo_safety.last_fault_snapshot.rcc_d1cfgr;
    car_servo_fault_snapshot_d2cfgr =
        servo_safety.last_fault_snapshot.rcc_d2cfgr;
    car_servo_fault_snapshot_gpioa_moder =
        servo_safety.last_fault_snapshot.gpioa_moder;
    car_servo_fault_snapshot_gpioa_afr1 =
        servo_safety.last_fault_snapshot.gpioa_afr1;
    car_motor_hardware_accepted = CARTEST_HARDWARE_ACCEPTED;
    car_servo_real_pwm_verified = 0u;
    car_follow_servo_target_us = car_control_state.follow_servo_target_us;
    car_follow_motor_duty_permille =
#if CARTEST_HARDWARE_ACCEPTED
        car_control_state.follow_output_duty_permille;
#else
        0u;
#endif
    car_follow_diff_mode_permille = CARTEST_FOLLOW_DIFF_PERMILLE;
    car_follow_inner_diff_permille = CARTEST_FOLLOW_INNER_DIFF_PERMILLE;
    car_follow_profile_id = CARTEST_TRACK_PROFILE !=
                                    CAR_TRACK_PROFILE_LEGACY
        ? 6u + CARTEST_TRACK_PROFILE :
        (CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED ? 6u :
         (CARTEST_FOLLOW_SPEED_PI_ENABLED ? 4u :
          (CARTEST_FOLLOW_TURN_PROFILE_COMPILED ?
              (CARTEST_FOLLOW_DIFF_PERMILLE == 150 ? 5u : 3u) :
           (CARTEST_FOLLOW_DIFF_PERMILLE == 0 ? 0u :
            (CARTEST_FOLLOW_DIFF_PERMILLE == 50 ? 1u : 2u)))));
    car_follow_diff_u_permille = follow_output_state.differential_u_permille;
    car_follow_outer_delta_permille =
        follow_output_state.outer_delta_permille;
    car_follow_inner_delta_permille =
        follow_output_state.inner_delta_permille;
    car_follow_normal_cap_permille = APP_FOLLOW_NORMAL_LIMIT_PERMILLE;
    car_follow_motor_normal_step_permille =
        CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE;
    car_follow_servo_normal_step_us =
        CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US;
    car_follow_output_phase = follow_output_state.phase;
    car_follow_startup_qualified = follow_output_state.startup_qualified;
    car_follow_base_target_permille =
        car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
        car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD
            ? car_control_state.follow_output_duty_permille : 0u;
    car_follow_left_target_permille =
        follow_output_state.target_left_permille;
    car_follow_right_target_permille =
        follow_output_state.target_right_permille;
    car_follow_left_command_permille =
        follow_output_state.command_left_permille;
    car_follow_right_command_permille =
        follow_output_state.command_right_permille;
    car_bench_ready = car_control_bench_ready(
        &car_control_state, &app_config, inputs,
        inputs != 0 ? profile_rx_path_healthy() : 0u);
    block_reason.startup_release_seen =
        car_control_state.startup_release_seen;
    block_reason.fault_present =
        (car_control_state.fault != CAR_FAULT_NONE ||
         car_control_state.fault_latched != 0u ||
         car_control_state.isr_stop_request != 0u ||
         car_control_state.isr_stop_fault != CAR_FAULT_NONE ||
         car_control_state.isr_release_seen != 0u ||
         servo_safety.fault_latched != 0u) ? 1u : 0u;
    block_reason.recovery_pending =
        ((car_control_state.recovery_active != 0u ||
          car_control_state.mode == CAR_MODE_RECOVERY_WAIT ||
          car_control_state.mode == CAR_MODE_WAIT_NEW_FRAME) &&
         car_control_state.recovery_frame_count <
             CAR_CONTROL_RECOVERY_REQUIRED_FRAMES) ? 1u : 0u;
    block_reason.vision_hold_active =
        (car_control_state.vision_hold_active != 0u && inputs != 0 &&
         (uint32_t)(inputs->now_ms -
                    car_control_state.vision_last_valid_ms) <
             CAR_CONTROL_VISION_HOLD_MS &&
         (uint32_t)(inputs->now_ms -
                    car_control_state.vision_last_frame_alive_ms) <
             app_config.vision_timeout_ms) ? 1u : 0u;
    block_reason.vision_hold_expired =
        (car_control_state.vision_hold_expired != 0u ||
         (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
          car_control_state.vision_hold_active == 0u)) ? 1u : 0u;
    block_reason.vision_fresh_valid =
        (block_reason.vision_hold_active != 0u ||
         (car_control_state.vision_valid != 0u &&
         car_control_state.vision_timeout == 0u && inputs != 0 &&
         car_control_vision_fresh(&car_control_state, inputs->now_ms,
                                  app_config.vision_timeout_ms) != 0u))
            ? 1u : 0u;
    block_reason.reverse_reset_keep_ready =
        (reverse_control_tx.reset_phase_complete != 0u &&
         car_control_state.reverse_reset_transport_gate_open != 0u &&
         car_control_state.reverse_transport_recovery_frame_count >=
             CAR_CONTROL_RECOVERY_REQUIRED_FRAMES &&
         reverse_control_tx.keep_active != 0u &&
         car_control_state.reverse_control_keep_seen != 0u &&
         car_control_state.reverse_control_fresh_frames >=
             CAR_CONTROL_RECOVERY_REQUIRED_FRAMES) ? 1u : 0u;
    block_reason.profile_output_permission =
        (CARTEST_HARDWARE_ACCEPTED != 0 &&
         car_motor_hardware_accepted != 0u &&
         app_config.follow_mode_compiled != 0u &&
         app_config.motor_hardware_accepted != 0u) ? 1u : 0u;
    block_reason.follow_mode_active =
        (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
         block_reason.vision_hold_active != 0u) ? 1u : 0u;
    block_reason.follow_session_active =
        car_control_state.follow_session_active;
    block_reason.follow_output_permission =
        (car_control_state.road_drive_allowed != 0u &&
         car_control_state.rx_recovery_flush_required == 0u) ? 1u : 0u;
    block_reason.servo_stop_pending = servo_stop_pending;
    block_reason.servo_output_active = servo_output_active;
    block_reason.servo_first_frame_ready = servo_safety.frame_ready;
    block_reason.motor_left_applied =
        (motor_outputs_active != 0u && applied_follow_outputs != 0u &&
         car_motor_left_in2_output_permille != 0u) ? 1u : 0u;
    block_reason.motor_right_applied =
        (motor_outputs_active != 0u && applied_follow_outputs != 0u &&
         car_motor_right_in1_output_permille != 0u) ? 1u : 0u;
    block_reason.manual_run_request =
        manual_run_button.run_request;
    car_run_block_reason = (uint32_t)
        car_run_block_reason_evaluate(&block_reason);
    car_main_loop_gap_fault = car_control_state.fault == CAR_FAULT_LOOP_GAP;
    car_main_loop_heartbeat_ms = snapshot_now_ms;
}

#if CARTEST_RUNTIME_LOG_ENABLED
static car_runtime_log_session_kind_t runtime_log_start_edge(
    car_mode_t mode_before_tick)
{
    if (car_control_state.mode == CAR_MODE_BENCH_RUNNING &&
        mode_before_tick != CAR_MODE_BENCH_RUNNING) {
        return CAR_RUNTIME_LOG_SESSION_BENCH;
    }
    return CAR_RUNTIME_LOG_SESSION_NONE;
}

static uint16_t runtime_log_max_u16(uint32_t first, uint32_t second)
{
    const uint32_t maximum = first > second ? first : second;
    return maximum > 0xFFFFu ? 0xFFFFu : (uint16_t)maximum;
}

static car_runtime_log_sample_t runtime_log_capture_sample(uint32_t now_ms)
{
    car_runtime_log_sample_t sample = {0};
    motor_applied_channels_t applied_channels;
    uint32_t primask;
    uint32_t vision_last_valid_ms;
    uint8_t vision_fresh;

    primask = __get_PRIMASK();
    __disable_irq();
    applied_channels = motor_applied_channels_snapshot();
    vision_fresh = car_control_vision_fresh(
        &car_control_state, now_ms, app_config.vision_timeout_ms);
    sample.now_ms = now_ms;
    sample.left_encoder_counts = car_encoder_diag.left_raw_delta_sum;
    sample.right_encoder_counts = car_encoder_diag.right_raw_delta_sum;
    sample.fault_generation = car_control_state.fault_generation;
    sample.auto_recovery_count = car_control_state.auto_recovery_count;
    sample.servo_requested_us = (uint16_t)car_servo_requested_us;
    sample.servo_limited_us = (uint16_t)car_servo_limited_us;
    sample.servo_applied_us = car_servo_output_active != 0u
                                  ? (uint16_t)car_servo_applied_pulse_us
                                  : 0u;
    sample.motor_requested_permille =
        app_config.follow_mode_compiled != 0u
            ? car_control_state.follow_output_duty_permille
            : car_control_state.bench_output_duty_permille;
    sample.left_applied_permille = runtime_log_max_u16(
        applied_channels.left_in1_permille,
        applied_channels.left_in2_permille);
    sample.right_applied_permille = runtime_log_max_u16(
        applied_channels.right_in1_permille,
        applied_channels.right_in2_permille);
    sample.motor_ccr1_counts = (uint16_t)TIM3->CCR1;
    sample.motor_ccr2_counts = (uint16_t)TIM3->CCR2;
    sample.motor_ccr3_counts = (uint16_t)TIM3->CCR3;
    sample.motor_ccr4_counts = (uint16_t)TIM3->CCR4;
    sample.vision_error_norm = car_control_state.latest_rx_error_norm;
    sample.vision_sequence = car_control_state.latest_rx_sequence;
    sample.mode = (uint8_t)car_control_state.mode;
    sample.fault = (uint8_t)car_control_state.fault;
    sample.vision_confidence = car_control_state.latest_rx_confidence;
    sample.vision_flags =
        (car_control_state.latest_rx_valid != 0u
             ? CAR_RUNTIME_LOG_VISION_LATEST_VALID : 0u) |
        (vision_fresh != 0u && car_control_state.vision_valid != 0u
             ? CAR_RUNTIME_LOG_VISION_USABLE_FRESH : 0u) |
        (car_control_state.vision_timeout != 0u
             ? CAR_RUNTIME_LOG_VISION_TIMEOUT : 0u) |
        (car_control_state.latest_rx_have_frame != 0u
             ? CAR_RUNTIME_LOG_VISION_LATEST_FRAME_PRESENT : 0u) |
        (car_control_state.vision_hold_active != 0u
             ? CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE : 0u) |
        (car_control_state.vision_hold_expired != 0u
             ? CAR_RUNTIME_LOG_VISION_HOLD_EXPIRED : 0u) |
        (car_control_state.latest_rx_slope_hint != 0u
             ? CAR_RUNTIME_LOG_VISION_LATEST_SLOPE_HINT : 0u) |
        (follow_output_state.slope_assist_active != 0u
             ? CAR_RUNTIME_LOG_VISION_SLOPE_ASSIST_COMMANDED : 0u);
    sample.recovery_frame_count = car_control_state.recovery_frame_count;
    sample.recovery_active = car_control_state.recovery_active;
    sample.follow_session_active = car_control_state.follow_session_active;
    sample.hard_fault_latched = car_control_state.fault_latched;
    sample.manual_run_request = manual_run_button.run_request;
    sample.follow_profile_compiled = app_config.follow_mode_compiled;
    sample.button_start_event_count = manual_run_button.start_event_count;
    sample.button_stop_generation = manual_run_button.stop_generation;
    sample.vision_hold_active = car_control_state.vision_hold_active;
    sample.vision_hold_expired = car_control_state.vision_hold_expired;
    sample.vision_hold_exit_cause = car_control_state.vision_hold_exit_cause;
    sample.vision_hold_valid_count =
        car_control_state.vision_hold_valid_count;
    vision_last_valid_ms = car_control_state.vision_last_valid_ms;
    __set_PRIMASK(primask);

    if (sample.mode == (uint8_t)CAR_MODE_FOLLOW_VISION_HOLD) {
        uint32_t hold_elapsed = (uint32_t)(now_ms - vision_last_valid_ms);
        if (hold_elapsed > CAR_CONTROL_VISION_HOLD_MS) {
            hold_elapsed = CAR_CONTROL_VISION_HOLD_MS;
        }
        sample.vision_hold_elapsed_ms = (uint16_t)hold_elapsed;
        /* Mode 12 retains its 64-byte ABI: 50 ms progress ticks occupy the
           high six bits and the low two bits retain consecutive recovery. */
        sample.recovery_frame_count = car_runtime_log_pack_hold_progress(
            hold_elapsed, sample.vision_hold_valid_count);
    }
    return sample;
}

static void runtime_log_publish_sample(
    uint32_t now_ms,
    car_runtime_log_session_kind_t start_kind)
{
    car_runtime_log_sample_t sample = runtime_log_capture_sample(now_ms);

    car_runtime_diag_note_log_update_call();
    car_runtime_log_update(&sample, start_kind);
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
    if (start_kind == CAR_RUNTIME_LOG_SESSION_FOLLOW) {
        const uint32_t start_token = car_runtime_log_last_start_token();
        if (start_token != 0u &&
            (speed_pi_trace_session_active == 0u ||
             start_token != speed_pi_trace_start_token)) {
            const uint8_t pi_type = CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
                ? CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL
                : CAR_SPEED_PI_DIAG_TYPE_POSITION;
            car_speed_pi_trace_begin(start_token, now_ms, pi_type);
            speed_pi_trace_session_active = 1u;
            speed_pi_trace_start_token = start_token;
            speed_pi_trace_have_sample = 0u;
        }
    }
#endif
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
    if (start_kind == CAR_RUNTIME_LOG_SESSION_FOLLOW) {
        const uint32_t start_token = car_runtime_log_last_start_token();
        if (start_token != 0u &&
            (speed_pi_diag_session_active == 0u ||
             start_token != speed_pi_diag_start_token)) {
            car_speed_pi_diag_begin(start_token, now_ms,
                                    CAR_SPEED_PI_DIAG_TYPE_INCREMENTAL);
            speed_pi_diag_session_active = 1u;
            speed_pi_diag_start_token = start_token;
            speed_pi_diag_have_values = 0u;
            speed_pi_diag_have_observer_sample = 0u;
        }
    }
#endif
}
#endif

static void runtime_diag_publish_snapshot(uint32_t now_ms)
{
    car_runtime_diag_values_t values = {0};
    uint32_t primask = __get_PRIMASK();

    /* Capture ISR-visible event counters and applied outputs as one bounded
       software snapshot. The CRC/cache publication itself runs with IRQs on. */
    __disable_irq();
    values.start_events = manual_run_button.start_event_count;
    values.stop_events = manual_run_button.stop_generation;
    values.left_output_permille = applied_motor_follow_left_permille;
    values.right_output_permille = applied_motor_follow_right_permille;
    values.servo_pulse_us = applied_servo_pulse_us;
    __set_PRIMASK(primask);

    values.start_token = car_runtime_log_last_start_token();
    values.cpu_header_state = car_runtime_log_storage.header.state;
    values.cpu_header_count = car_runtime_log_storage.header.count;
    values.header_session_id = car_runtime_log_storage.header.session_id;
    values.uptime_ms = now_ms;
    values.reset_flags = car_init_rcc_rsr_reset_snapshot;
    values.log_enabled = (uint8_t)CARTEST_RUNTIME_LOG_ENABLED;
    (void)car_runtime_diag_update(&values);
}

#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
static void ackermann_snapshot_update(uint32_t now_ms)
{
    car_ackermann_snapshot_values_t values = {0};
    uint16_t snapshot_servo_pulse_us = applied_servo_pulse_us;
    const uint8_t in_hold =
        car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
        car_control_state.vision_hold_active != 0u;
    const uint8_t fault_locked = car_control_state.fault_latched != 0u ||
                                 car_control_state.fault != CAR_FAULT_NONE;
    const uint8_t active =
        car_control_state.manual_run_request != 0u &&
        car_control_state.follow_session_active != 0u &&
        fault_locked == 0u &&
        (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
         in_hold != 0u);
    int16_t angle_cdeg = 0;
    uint16_t ratio_q15 = CAR_ACKERMANN_RATIO_ONE_Q15;
    int32_t applied_delta;

    if (active == 0u && ackermann_snapshot_session_active == 0u) {
        return;
    }
    if (active != 0u && ackermann_snapshot_session_active == 0u) {
        ackermann_snapshot_last_valid_servo_pulse_us = 0u;
    }
    if (active != 0u && ackermann_snapshot_session_active != 0u &&
        (uint32_t)(now_ms - ackermann_snapshot_last_publish_ms) < 10u) {
        return;
    }

    if (active != 0u && servo_output_active != 0u &&
        servo_safety.frame_ready != 0u &&
        snapshot_servo_pulse_us >= app_config.follow_servo_left_us &&
        snapshot_servo_pulse_us <= app_config.follow_servo_right_us) {
        ackermann_snapshot_last_valid_servo_pulse_us =
            snapshot_servo_pulse_us;
    } else if (ackermann_snapshot_last_valid_servo_pulse_us != 0u) {
        /* Freeze/servo restart records retain the last physically published
           pulse; a stopped TIM1 compare of zero is not a valid model input. */
        snapshot_servo_pulse_us =
            ackermann_snapshot_last_valid_servo_pulse_us;
    } else if (active != 0u) {
        /* Do not claim an applied steering sample before the first full frame. */
        return;
    } else {
        snapshot_servo_pulse_us = app_config.servo_center_us;
    }

    if (car_ackermann_model_geometry(
            snapshot_servo_pulse_us, app_config.servo_center_us,
            app_config.follow_servo_left_us,
            app_config.follow_servo_right_us,
            &angle_cdeg, &ratio_q15) == 0u) {
        angle_cdeg = 0;
        ratio_q15 = CAR_ACKERMANN_RATIO_ONE_Q15;
    }
    applied_delta = (int32_t)snapshot_servo_pulse_us -
                    (int32_t)app_config.servo_center_us;

    values.flags = 0u;
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
    values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_COORD_ENABLED;
#endif
    values.curve_phase = car_control_state.vision_curve_phase;
    values.slowdown_request_permille =
        car_control_state.vision_slowdown_request_permille;
    if (values.slowdown_request_permille != 0u) {
        values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_REQUEST_ACTIVE;
    }
    if (in_hold != 0u) {
        values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_IN_HOLD;
    }
    if (fault_locked != 0u) {
        values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_FAULT_LOCKED;
    }
    if (active != 0u) {
        values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE;
    } else {
        values.flags |= CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN;
    }
    values.timestamp_ms = now_ms;
    values.vision_sequence = car_control_state.have_sequence != 0u
        ? car_control_state.vision_sequence : 0u;
    values.model_angle_cdeg = angle_cdeg;
    values.ratio_q15 = ratio_q15;
    values.applied_pwm_delta_us = (int16_t)applied_delta;
    values.servo_pulse_us = snapshot_servo_pulse_us;
#if CARTEST_ACKERMANN_COORDINATION_ENABLED
    values.fastest_target_millirps =
        ackermann_control_output.fastest_target_millirps;
    values.left_target_millirps =
        ackermann_control_output.left_target_millirps;
    values.right_target_millirps =
        ackermann_control_output.right_target_millirps;
#else
    values.left_target_millirps = APP_FOLLOW_PI_TARGET_LEFT;
    values.right_target_millirps = APP_FOLLOW_PI_TARGET_RIGHT;
    values.fastest_target_millirps =
        values.left_target_millirps > values.right_target_millirps
            ? values.left_target_millirps : values.right_target_millirps;
#endif
    values.left_measured_millirps = speed_observer_result.left_millirps;
    values.right_measured_millirps = speed_observer_result.right_millirps;
    values.left_duty_permille = applied_motor_follow_left_permille;
    values.right_duty_permille = applied_motor_follow_right_permille;
    values.run_state = (uint8_t)car_control_state.mode;
    values.fault_reason = (uint8_t)car_control_state.fault;
    car_ackermann_snapshot_publish(g_ackermann_snapshot, &values);
    ackermann_snapshot_last_publish_ms = now_ms;
    ackermann_snapshot_session_active = active;
}
#endif

static HAL_StatusTypeDef speed_pi_trace_sram_enable(void)
{
    uint32_t enabled;

    /* D2 SRAM1 is clock-gated on this device. The trace section is NOLOAD, so
       explicitly enable and read back its clock before any trace access. */
    RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN;
    enabled = RCC->AHB2ENR;
    __DSB();
    return (enabled & RCC_AHB2ENR_D2SRAM1EN) != 0u ? HAL_OK : HAL_ERROR;
}

int main(void)
{
    uint8_t watchdog_reset;
    uint8_t raw_high;
    HAL_StatusTypeDef init_status;
    uint32_t now_ms;
    uint32_t previous_loop_ms;
    uint8_t loop_gap_fault;
    uint32_t left_encoder_raw;
    uint16_t right_encoder_raw;
#if CARTEST_RUNTIME_LOG_ENABLED
    car_runtime_log_session_kind_t runtime_log_start_kind;
    car_mode_t mode_before_tick;
#endif
    car_inputs_t inputs;

    car_ackermann_snapshot_reset(g_ackermann_snapshot);
    car_runtime_log_enabled = (uint32_t)CARTEST_RUNTIME_LOG_ENABLED;
    car_config_init_application(
        &app_config, (uint8_t)CARTEST_BENCH_MODE_COMPILED,
        (uint8_t)CARTEST_STEERING_MODE_COMPILED,
        (uint8_t)CARTEST_FOLLOW_MODE_COMPILED,
        (uint8_t)CARTEST_HARDWARE_ACCEPTED);
    car_camera_profile_required_ack =
        app_config.required_camera_profile_ack;
    reset_session_odometry();
    car_servo_safety_init(&servo_safety);
    servo_safety_io = (car_servo_safety_io_t){
        .context = 0,
        .capture = servo_capture_snapshot,
        .write_compare = servo_write_compare,
        .read_compare = servo_read_compare,
        .start_output = servo_hardware_start,
        .take_completed_update = servo_take_completed_update,
        .stop_output = servo_hardware_stop,
    };
#if CARTEST_HARDWARE_ACCEPTED
    car_motor_hardware_lock_reason = 0u;
#else
    car_motor_hardware_lock_reason = 1u;
#endif
    car_bench_ready = 0u;
    car_bench_max_applied_duty_permille = 0u;

    init_stage_begin(CAR_INIT_STAGE_RESET_SAFE);
    early_motor_pins_low();
    car_init_rcc_rsr_reset_snapshot = RCC->RSR;
    watchdog_reset = (car_init_rcc_rsr_reset_snapshot & RCC_RSR_IWDG1RSTF) != 0u
                         ? 1u : 0u;
    init_capture_registers();
    __HAL_RCC_CLEAR_RESET_FLAGS();
    HAL_Init();
    car_init_hal_status = (uint32_t)HAL_OK;

    init_status = system_clock_config();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_CLOCK, CAR_INIT_ERROR_CLOCK_OSC, init_status);
    }
    init_status = verify_system_and_timer_clocks();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_CLOCK, CAR_INIT_ERROR_CLOCK_VERIFY, init_status);
    }

    init_status = switch_init();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_CONFIG, CAR_INIT_ERROR_SWITCH_GPIO, init_status);
    }
    raw_high = pe7_is_high();
    car_pe7_raw_high = raw_high;

    init_status = motor_pwm_init();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_TIMER, CAR_INIT_ERROR_MOTOR_TIM3_BASE, init_status);
    }
    init_status = servo_pwm_init();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_TIMER, CAR_INIT_ERROR_SERVO_TIM1_BASE, init_status);
    }
    init_status = encoder_init();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_ENCODER_INIT,
                     CAR_INIT_ERROR_ENCODER_TIM2_CONFIG, init_status);
    }
    emergency_stop_hardware();
    car_vision_rx_init(&car_vision_rx);
    raw_high = pe7_is_high();
    switch_debounce.candidate_level = raw_high;
    switch_debounce.stable_level = raw_high;
    switch_debounce.stable_valid = 0u;
    switch_debounce.candidate_since_ms = HAL_GetTick();
    car_manual_button_init(&manual_run_button, raw_high, HAL_GetTick());
    manual_stop_consumed_generation = 0u;
    init_stage_begin(CAR_INIT_STAGE_CONTROLLER_INIT);
    car_control_init(&car_control_state, &app_config, HAL_GetTick(),
                     raw_high, watchdog_reset);
    enforce_motor_hardware_lock();
    (void)init_status_record(CAR_INIT_ERROR_NONE, HAL_OK);

    init_status = usart1_init();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_UART, CAR_INIT_ERROR_USART1_VERIFY, init_status);
    }
    car_reverse_control_init(&reverse_control_tx, HAL_GetTick());
    reverse_control_ready = 1u;
    reverse_control_seen_generation =
        car_control_state.reverse_control_reset_generation;
    reverse_control_seen_rx_fault_generation =
        car_control_state.rx_fault_generation;
    car_control_reverse_control_reset_started(&car_control_state);
    car_iwdg_reset_seen = watchdog_reset;
    init_status = iwdg_start();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_WATCHDOG_INIT,
                     CAR_INIT_ERROR_IWDG_REGISTER_UPDATE_TIMEOUT, init_status);
    }

#if CARTEST_RUNTIME_LOG_ENABLED
    car_runtime_log_init();
#else
    car_runtime_log_invalidate();
#endif
    init_status = speed_pi_trace_sram_enable();
    if (init_status != HAL_OK) {
        init_failure(CAR_FAULT_CONFIG, CAR_INIT_ERROR_TRACE_SRAM, init_status);
    }
    car_speed_pi_trace_init();
    car_receive_diag_init(HAL_GetTick());
    car_speed_pi_diag_init();
    car_runtime_diag_init(car_init_rcc_rsr_reset_snapshot,
                          (uint8_t)CARTEST_RUNTIME_LOG_ENABLED);

    car_init_stage = (uint32_t)CAR_INIT_STAGE_READY;
    car_init_hal_status = (uint32_t)HAL_OK;
    car_init_error = (uint32_t)CAR_INIT_ERROR_NONE;
    previous_loop_ms = HAL_GetTick();
    car_main_loop_heartbeat_ms = previous_loop_ms;
    for (;;) {
        uint8_t follow_start_event = 0u;
#if CARTEST_RUNTIME_LOG_ENABLED
        runtime_log_start_kind = CAR_RUNTIME_LOG_SESSION_NONE;
#endif
        enforce_motor_hardware_lock();
        now_ms = HAL_GetTick();
        raw_high = pe7_is_high();
        (void)manual_button_sample(raw_high, now_ms);
        if (app_config.follow_mode_compiled != 0u &&
            manual_run_button.start_pending != 0u) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (manual_run_button.start_pending != 0u) {
                follow_start_event = 1u;
                reverse_manual_stop_holdoff = 0u;
                car_manual_button_start_consumed(&manual_run_button);
            }
            __set_PRIMASK(primask);
        }
#if CARTEST_RUNTIME_LOG_ENABLED
        if (follow_start_event != 0u) {
            runtime_log_start_kind = CAR_RUNTIME_LOG_SESSION_FOLLOW;
        }
#else
        (void)follow_start_event;
#endif
        debounce_update(now_ms, raw_high);
        loop_gap_fault = (uint32_t)(now_ms - previous_loop_ms) >=
                         APP_MAIN_LOOP_GAP_LIMIT_MS ? 1u : 0u;
        previous_loop_ms = now_ms;

        left_encoder_raw = TIM2->CNT;
        right_encoder_raw = (uint16_t)TIM8->CNT;
        if (car_encoder_diag_update(&car_encoder_diag, left_encoder_raw,
                                    right_encoder_raw) == 0u) {
            car_control_note_fault(&car_control_state, CAR_FAULT_ENCODER_INIT);
            car_speed_observer_invalidate(
                &speed_observer, now_ms,
                car_encoder_diag.left_raw_delta_sum,
                car_encoder_diag.right_raw_delta_sum,
                CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE);
            speed_observer_result.valid = 0u;
            speed_observer_result.status =
                CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE;
            speed_observer_result.sample_ms = now_ms;
            speed_observer_result.elapsed_ms = 0u;
            speed_observer_result.left_millirps = 0;
            speed_observer_result.right_millirps = 0;
            speed_observer_result.left_rpm = 0;
            speed_observer_result.right_rpm = 0;
        } else {
            const uint8_t observer_base_expected =
                (car_control_state.mode == CAR_MODE_FOLLOW_RUNNING ||
                 (car_control_state.mode == CAR_MODE_FOLLOW_VISION_HOLD &&
                  car_control_state.vision_hold_active != 0u)) &&
                car_control_state.follow_session_active != 0u &&
                car_control_state.road_drive_allowed != 0u &&
                car_control_state.fault_latched == 0u &&
                follow_output_state.startup_qualified != 0u &&
                servo_output_active != 0u && servo_safety.frame_ready != 0u &&
                motor_outputs_active != 0u && applied_follow_outputs != 0u;
            uint8_t observer_motion_expected = observer_base_expected != 0u &&
                applied_motor_follow_left_permille != 0u &&
                applied_motor_follow_right_permille != 0u;
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
            if (observer_base_expected != 0u &&
                APP_FOLLOW_PI_ARMED != 0u &&
                APP_FOLLOW_PI_TARGET_LEFT > 0 &&
                APP_FOLLOW_PI_TARGET_RIGHT > 0) {
                /* A zero-coast correction must not hide an encoder stall. */
                observer_motion_expected = 1u;
            }
#endif
            (void)car_speed_observer_update(
                &speed_observer, now_ms,
                car_encoder_diag.left_raw_delta_sum,
                car_encoder_diag.right_raw_delta_sum,
                observer_motion_expected, &speed_observer_result);
        }
        update_session_odometry();

        inputs = make_inputs(now_ms, raw_high, loop_gap_fault);
        consume_fast_stop_latch();
        cancel_manual_request_on_fault();
        consume_manual_stop_request(&inputs);
        inputs = make_inputs(now_ms, raw_high, loop_gap_fault);
#if CARTEST_RUNTIME_LOG_ENABLED
        uint8_t servo_ack_before_tick = car_control_state.fault_latched;
        mode_before_tick = car_control_state.mode;
        car_control_tick(&car_control_state, &app_config, &inputs);
        if (runtime_log_start_kind == CAR_RUNTIME_LOG_SESSION_NONE) {
            runtime_log_start_kind = runtime_log_start_edge(
                mode_before_tick);
        }
#else
        uint8_t servo_ack_before_tick = car_control_state.fault_latched;
        car_control_tick(&car_control_state, &app_config, &inputs);
        cancel_manual_request_on_fault();
#endif
        if (servo_ack_before_tick != 0u &&
            car_control_state.fault_latched == 0u) {
            car_servo_safety_acknowledge(&servo_safety);
        }
        enforce_motor_hardware_lock();
        if (car_control_state.rx_recovery_flush_required != 0u) {
            flush_uart_and_parser();
        }
        raw_high = pe7_is_high();
        (void)manual_button_sample(raw_high, HAL_GetTick());
        inputs = make_inputs(HAL_GetTick(), raw_high, loop_gap_fault);
        service_physical_profile_operation(&inputs);
        service_command_mailbox(&inputs);
        if (car_control_state.rx_recovery_flush_required != 0u) {
            flush_uart_and_parser();
        }

        uint16_t rx_head_snapshot = vision_rx_snapshot(&now_ms);
        raw_high = pe7_is_high();
        /* Account for ISR/main-line fast stops before a newly parsed valid=0
           frame is allowed to begin the camera-loss hold. */
        note_applied_output_snapshot();
        car_vision_rx_poll(&car_vision_rx, &car_control_state, &app_config,
                           rx_head_snapshot, now_ms, raw_high);
        raw_high = pe7_is_high();
        (void)manual_button_sample(raw_high, now_ms);
        inputs = make_inputs(now_ms, raw_high, loop_gap_fault);
        consume_fast_stop_latch();
        cancel_manual_request_on_fault();
        consume_manual_stop_request(&inputs);
        inputs = make_inputs(now_ms, raw_high, loop_gap_fault);
        servo_ack_before_tick = car_control_state.fault_latched;
#if CARTEST_RUNTIME_LOG_ENABLED
        mode_before_tick = car_control_state.mode;
#endif
        car_control_tick(&car_control_state, &app_config, &inputs);
        cancel_manual_request_on_fault();
#if CARTEST_RUNTIME_LOG_ENABLED
        if (runtime_log_start_kind == CAR_RUNTIME_LOG_SESSION_NONE) {
            runtime_log_start_kind = runtime_log_start_edge(
                mode_before_tick);
        }
#endif
        if (servo_ack_before_tick != 0u &&
            car_control_state.fault_latched == 0u) {
            car_servo_safety_acknowledge(&servo_safety);
        }
        if (follow_start_event != 0u &&
            app_config.follow_mode_compiled != 0u) {
            uint8_t manual_start_accepted = 0u;
            uint32_t start_event_primask = __get_PRIMASK();
            __disable_irq();
            /* Rebuild the inputs while the ISR button/fast-stop state is
               stable; the earlier loop snapshot may predate a physical stop. */
            inputs = make_inputs(HAL_GetTick(), pe7_is_high(), loop_gap_fault);
            if (manual_run_button.run_request != 0u &&
                manual_run_button.stop_pending == 0u &&
                manual_run_button.start_pending == 0u) {
                manual_start_accepted =
                    car_control_begin_manual_follow_start(
                        &car_control_state, &app_config, &inputs);
            }
            __set_PRIMASK(start_event_primask);
            if (manual_start_accepted != 0u) {
                ++follow_session_token;
                if (follow_session_token == 0u) {
                    ++follow_session_token;
                }
                reset_session_odometry();
                /* A previous local KEEP may have left applied outputs active. */
                emergency_stop_hardware();
            }
        }
        enforce_motor_hardware_lock();
        if (car_control_state.rx_recovery_flush_required != 0u) {
            flush_uart_and_parser();
        }

        service_reverse_control(HAL_GetTick());
        if (car_control_state.vision_hold_restart_required != 0u) {
            /* A late third route frame may resume the session logically, but
               it cannot reuse outputs from the expired hold. Stop TIM3 now,
               complete the servo pulse stop path, then require a new first
               servo frame and fresh motor ramp. Close controller permission
               in this loop before any output stage can reuse the old frame. */
            car_control_state.road_drive_allowed = 0u;
            car_control_state.follow_output_duty_permille = 0u;
            emergency_stop_hardware();
            follow_ramp_started = 0u;
            car_follow_output_reset(&follow_output_state);
            car_control_state.vision_hold_restart_required = 0u;
        }
        servo_apply();
        prepare_follow_motor_targets(HAL_GetTick());
        motor_apply();
        note_applied_output_snapshot();
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
        ackermann_snapshot_update(HAL_GetTick());
#endif
        publish_debug_snapshot(&inputs);
#if CARTEST_RUNTIME_LOG_ENABLED
        runtime_log_publish_sample(HAL_GetTick(), runtime_log_start_kind);
#endif
#if CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED
        speed_pi_diag_publish(HAL_GetTick(), 1u, 0u);
#endif
#if CARTEST_FOLLOW_SPEED_PI_ENABLED
        speed_pi_trace_publish_sample();
        speed_pi_trace_freeze_if_stopped(HAL_GetTick());
#endif
        car_receive_diag_publish(HAL_GetTick());
        runtime_diag_publish_snapshot(HAL_GetTick());
        iwdg_refresh();
    }
}

void USART1_IRQHandler(void)
{
    static const car_uart_rx_service_io_t service = {
        .context = 0,
        .rxne_mask = USART_ISR_RXNE_RXFNE,
        .error_mask = USART_ISR_PE | USART_ISR_FE | USART_ISR_NE |
                      USART_ISR_ORE,
        .read_status = uart_service_read_status,
        .read_data = uart_service_read_data,
        .on_error = uart_service_on_error,
        .on_byte = uart_service_on_byte,
        .on_abort = uart_service_on_abort
    };
    const car_uart_rx_service_result_t result =
        car_uart_rx_service_drain(&service);
    if (result.bytes_drained > car_uart_rx_irq_max_batch) {
        car_uart_rx_irq_max_batch = result.bytes_drained;
    }
}

void EXTI9_5_IRQHandler(void)
{
    if ((EXTI->PR1 & APP_SWITCH_GPIO_PIN) != 0u) {
        EXTI->PR1 = APP_SWITCH_GPIO_PIN;
        if (app_config.follow_mode_compiled != 0u) {
            (void)manual_button_sample(pe7_is_high(), HAL_GetTick());
        } else if (app_config.debug_steering_mode_compiled == 0u &&
                   (car_control_state.mode == CAR_MODE_BENCH_RUNNING ||
                    (motor_outputs_active != 0u &&
                     applied_follow_outputs == 0u)) &&
                   pe7_is_high() != 0u) {
            /* Bench keeps the original released-high safety stop. */
            protected_switch_release_note();
            emergency_stop_hardware();
        }
    }
}

