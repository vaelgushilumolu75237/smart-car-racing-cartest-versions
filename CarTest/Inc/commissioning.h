#ifndef CARTEST_COMMISSIONING_H
#define CARTEST_COMMISSIONING_H

#include <stdint.h>

#ifndef CARTEST_VISION_HOLD_MS
#define CARTEST_VISION_HOLD_MS 3000u
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_CONTROL_RECOVERY_REQUIRED_FRAMES = 3u
};

#define CAR_CONTROL_VISION_HOLD_MS CARTEST_VISION_HOLD_MS

/* D1 .bss RAM snapshot for CubeIDE Live Watch / debugger reads. Entry is the
   publication token and is written after the associated data fields. */
extern volatile uint32_t car_vision_hold_snapshot_entry;
extern volatile int32_t car_vision_hold_snapshot_error_norm;
extern volatile uint32_t car_vision_hold_snapshot_sequence;
extern volatile uint32_t car_vision_hold_snapshot_source_rx_ms;
extern volatile uint32_t car_vision_hold_snapshot_entered_ms;
extern volatile uint32_t car_vision_hold_snapshot_target_us;
extern volatile uint32_t car_vision_hold_snapshot_applied_us;
extern volatile uint32_t car_vision_hold_snapshot_left_permille;
extern volatile uint32_t car_vision_hold_snapshot_right_permille;

/* Keep existing Live Watch numbers stable; append runtime states only. */
typedef enum {
    CAR_MODE_BOOT_SAFE = 0,
    CAR_MODE_WAIT_SWITCH_RELEASE,
    CAR_MODE_WAIT_LOCAL_ACK,
    CAR_MODE_WAIT_NEW_FRAME,
    CAR_MODE_WAIT_START_RELEASE,
    CAR_MODE_WAIT_START_EDGE,
    CAR_MODE_BENCH_RUNNING,
    CAR_MODE_BENCH_COMPLETE,
    CAR_MODE_FAULT_LATCHED,
    CAR_MODE_RECOVERY_WAIT = 9,
    CAR_MODE_FOLLOW_RUNNING = 10,
    CAR_MODE_DEBUG_STEERING_RUNNING = 11,
    CAR_MODE_FOLLOW_VISION_HOLD = 12
} car_mode_t;

typedef enum {
    CAR_FAULT_NONE = 0,
    CAR_FAULT_WATCHDOG_RESET,
    CAR_FAULT_LOOP_GAP,
    CAR_FAULT_UART,
    CAR_FAULT_RX_OVERFLOW,
    CAR_FAULT_RX_STALE,
    CAR_FAULT_PROTOCOL,
    CAR_FAULT_SEQUENCE,
    CAR_FAULT_VISION_INVALID,
    CAR_FAULT_VISION_TIMEOUT,
    CAR_FAULT_SWITCH_RELEASE,
    CAR_FAULT_ENCODER_INIT,
    CAR_FAULT_CLOCK,
    CAR_FAULT_TIMER,
    CAR_FAULT_CONFIG,
    CAR_FAULT_WATCHDOG_INIT
} car_fault_t;

typedef enum {
    CAR_MOTOR_NONE = 0,
    CAR_MOTOR_LEFT = 1,
    CAR_MOTOR_RIGHT = 2,
    /* Fixed forward pair: left IN2 plus right IN1. */
    CAR_MOTOR_BOTH_FORWARD = 3
} car_motor_t;

/* Driver input index, not a guessed forward/reverse wheel direction. */
typedef enum {
    CAR_MOTOR_IN1 = 0,
    CAR_MOTOR_IN2 = 1
} car_motor_input_t;

typedef enum {
    CAR_CMD_NONE = 0,
    CAR_CMD_LOCAL_ACK = 0x41434B31u,
    CAR_CMD_CAMERA_RESTART_ACK = 0x43414D52u,
    CAR_CMD_NORMAL_STOP = 0x53544F50u,
    CAR_CMD_SELECT_BENCH_OUTPUT = 0x53454C31u
} car_command_t;

enum {
    CAR_EFFECT_NONE = 0u,
    CAR_EFFECT_CLEAR_RX = 1u << 0,
    CAR_EFFECT_RESET_PARSER = 1u << 1,
    CAR_EFFECT_STOP_OUTPUTS = 1u << 2,
    CAR_EFFECT_DISABLE_SERVO = 1u << 3
};

typedef struct {
    uint8_t bench_mode_compiled;
    uint8_t servo_limits_measured;
    uint16_t servo_min_us;
    uint16_t servo_max_us;
    uint16_t servo_center_us; /* User-measured centre is 1530 us. */
    uint16_t bench_duty_limit_permille;
    uint16_t bench_ramp_step_permille;
    uint16_t bench_run_limit_ms;
    uint32_t vision_timeout_ms;
    uint8_t motor_hardware_accepted;
    /* Independent servo-only camera check; all motor paths remain disabled. */
    uint8_t debug_steering_mode_compiled;
    /* Follow mode remains locked unless every field below is measured. */
    uint8_t follow_mode_compiled;
    uint8_t follow_calibration_ready;
    uint8_t follow_positive_error_right_measured;
    uint16_t follow_servo_left_us;
    uint16_t follow_servo_right_us;
    uint32_t follow_left_encoder_counts_per_rev;
    uint32_t follow_right_encoder_counts_per_rev;
    int8_t follow_left_encoder_sign;
    int8_t follow_right_encoder_sign;
    uint16_t follow_kp_permille;
    uint16_t follow_max_duty_permille;
    uint16_t follow_ramp_step_permille;
} car_config_t;

/* Shared production profile configuration used by target main and host tests. */
void car_config_init_application(car_config_t *config,
                                 uint8_t bench_mode_compiled,
                                 uint8_t debug_steering_mode_compiled,
                                 uint8_t follow_mode_compiled,
                                 uint8_t motor_hardware_accepted);

typedef struct {
    uint32_t now_ms;
    uint8_t pe7_raw_high;       /* High means released; ignored by DebugSteering. */
    uint8_t pe7_high_stable;    /* Debounce is only used to acknowledge. */
    uint8_t pe7_low_stable;
    uint8_t loop_gap_fault;
    uint8_t uart_fault;
    uint8_t rx_overflow_fault;
    uint8_t rx_stale_fault;
    uint8_t protocol_fault;
    uint8_t encoder_fault;
    uint8_t clock_or_timer_fault;
    uint8_t manual_run_request;
    uint8_t manual_stop_pending;
    uint8_t manual_button_startup_release_confirmed;
} car_inputs_t;

typedef struct {
    car_mode_t mode;
    car_fault_t fault;
    uint8_t debug_steering_mode_compiled;
    uint8_t fault_latched;
    uint8_t road_drive_allowed;
    uint8_t vision_valid;
    /* Trusted only after the normal sequence, age, and recovery gates. */
    uint8_t vision_slope_hint;
    uint8_t vision_curve_phase;
    uint16_t vision_slowdown_request_permille;
    uint8_t vision_timeout;
    uint8_t vision_confidence;
    int16_t vision_error_norm;  /* Normalized image half-width / 1000. */
    uint16_t vision_sequence;
    uint8_t have_sequence;
    uint8_t have_valid_vision;
    uint32_t vision_last_valid_ms;
    uint32_t rx_frame_count;
    uint32_t rx_error_count;
    uint32_t sequence_error_count;
    uint32_t invalid_vision_count;
    uint32_t command_reject_count;
    uint32_t camera_restart_count;
    uint8_t bench_motor;
    uint8_t bench_input;
    uint16_t bench_requested_duty_permille;
    uint16_t bench_output_duty_permille;
    uint32_t bench_started_ms;
    uint32_t bench_target_reached_ms;
    uint8_t bench_target_hold_active;
    uint32_t button_low_edge_count;
    uint32_t frame_edge_count;
    uint8_t ack_seen_low;
    uint8_t switch_was_high;
    volatile uint8_t isr_stop_request;
    /* ISR sets only a cause latch; the main loop converts it to full state. */
    volatile uint8_t isr_stop_fault;
    uint8_t ack_waiting_new_frame;
    uint32_t command_effects;
    uint32_t last_ramp_ms;
    /* Latest CRC/format-valid UART observation, independent of trust. */
    uint8_t latest_rx_valid;
    uint8_t latest_rx_slope_hint;
    uint8_t latest_rx_curve_phase;
    uint16_t latest_rx_slowdown_request_permille;
    uint8_t latest_rx_confidence;
    int16_t latest_rx_error_norm;
    uint16_t latest_rx_sequence;
    uint32_t latest_rx_arrival_ms;
    uint32_t latest_rx_frame_count;
    uint8_t latest_rx_have_frame;
    /* Recovery accepts only a new uninterrupted set of fresh observations. */
    uint8_t recovery_frame_count;
    uint32_t auto_recovery_count;
    uint8_t recovery_active;
    uint8_t recovery_was_faulted;
    volatile uint8_t rx_recovery_flush_required;
    uint16_t recovery_sequence;
    uint32_t recovery_last_arrival_ms;
    uint8_t recovery_have_sequence;
    /* Reverse-link RESET/KEEP is cache lifecycle control, never drive input. */
    uint32_t reverse_control_reset_generation;
    uint8_t reverse_control_keep_seen;
    uint8_t reverse_control_fresh_frames;
    /* Set by a physical PE7 press; retained through recoverable faults. */
    volatile uint8_t follow_session_active;
    uint16_t follow_servo_target_us;
    uint16_t follow_output_duty_permille;
    uint32_t follow_last_ramp_ms;
    /* Separate session: cleared on every fault and never auto-resumed. */
    volatile uint8_t steering_session_active;
    uint16_t steering_servo_target_us;
    uint8_t startup_release_seen;
    uint8_t startup_frame_gate_pending;
    uint32_t startup_release_ms;
    uint8_t stable_switch_known;
    uint8_t stable_switch_high;
    uint8_t hard_ack_press_seen;
    volatile uint8_t isr_release_seen;
    uint32_t fault_generation;       /* Main-loop writer only. */
    uint32_t rx_fault_generation;    /* Main-loop writer only. */
    volatile uint32_t isr_switch_generation; /* PE7 EXTI writer only. */
    volatile uint32_t isr_uart_generation;   /* USART1 ISR writer only. */
    /* Set once per reset when the physical Bench target is selected. */
    uint8_t profile_command_issued;
    /* Follow camera-loss hold freezes the last trusted steering source and
       the two currently applied wheel outputs at one qualified entry. */
    uint8_t manual_run_request;
    uint8_t vision_hold_active;
    uint8_t vision_hold_expired;
    uint8_t vision_hold_restart_required;
    uint8_t vision_hold_valid_count;
    uint8_t vision_hold_have_sequence;
    uint16_t vision_hold_last_sequence;
    uint32_t vision_hold_last_arrival_ms;
    uint32_t vision_hold_entries;
    uint32_t vision_hold_started_ms; /* Hold entry time; deadline uses valid RX. */
    int16_t vision_hold_error_norm;
    uint16_t vision_hold_source_sequence;
    uint32_t vision_hold_source_rx_ms;
    uint8_t vision_hold_exit_cause;
    uint32_t vision_last_frame_alive_ms;
    uint16_t vision_last_frame_sequence;
    uint8_t vision_have_frame_sequence;
    uint8_t applied_servo_frame_ready;
    uint8_t applied_servo_output_active;
    uint16_t applied_servo_pulse_us;
    uint16_t applied_motor_left_permille;
    uint16_t applied_motor_right_permille;
    uint16_t vision_hold_servo_pulse_us; /* Held target mapped from saved error. */
    uint16_t vision_hold_applied_servo_pulse_us; /* Entry pulse, diagnostics only. */
    uint16_t vision_hold_base_duty_permille;
    uint16_t vision_hold_motor_left_permille;
    uint16_t vision_hold_motor_right_permille;
    uint8_t reverse_reset_transport_gate_open;
    uint8_t reverse_transport_recovery_frame_count;
    uint16_t reverse_transport_last_sequence;
    uint32_t reverse_transport_last_arrival_ms;
    uint8_t reverse_transport_have_sequence;
} car_control_t;

typedef enum {
    CAR_VISION_HOLD_EXIT_NONE = 0u,
    CAR_VISION_HOLD_EXIT_RECOVERED = 1u,
    CAR_VISION_HOLD_EXIT_DEADLINE = 2u,
    CAR_VISION_HOLD_EXIT_MANUAL_STOP = 3u,
    CAR_VISION_HOLD_EXIT_COMM_FAULT = 4u,
    CAR_VISION_HOLD_EXIT_HARD_FAULT = 5u
} car_vision_hold_exit_cause_t;

void car_control_init(car_control_t *control,
                      const car_config_t *config,
                      uint32_t now_ms,
                      uint8_t pe7_raw_high,
                      uint8_t watchdog_reset_detected);

/* P0.7 asymmetric image-error mapping, clamped to the measured servo span. */
uint16_t car_debug_steering_target_us(int16_t error_norm);

/* Called only from the main loop, never from an ISR. */
void car_control_tick(car_control_t *control,
                      const car_config_t *config,
                      const car_inputs_t *inputs);

/* Main loop publishes software-applied output snapshots for eligibility and
   hold freezing. These values describe register/software state, not pin tests. */
void car_control_note_applied_outputs(car_control_t *control,
                                      uint8_t servo_frame_ready,
                                      uint8_t servo_output_active,
                                      uint16_t servo_applied_us,
                                      uint16_t motor_left_applied_permille,
                                      uint16_t motor_right_applied_permille);

/* Follow's servo safety layer keeps slewing toward the saved route target
   throughout hold; this does not authorize output by itself. */
uint16_t car_control_follow_servo_output_target_us(
    const car_control_t *control);

/* Feed only a complete vp_unpack()-validated frame from the UART parser. */
void car_control_on_vision_frame(car_control_t *control,
                                 const car_config_t *config,
                                 uint16_t sequence,
                                 uint8_t valid,
                                 uint8_t confidence,
                                 int16_t error_norm,
                                 uint32_t arrival_ms,
                                 uint8_t pe7_raw_high);

/* Parser path supplies both the final-byte arrival and current poll time. */
void car_control_on_vision_frame_at(car_control_t *control,
                                    const car_config_t *config,
                                    uint16_t sequence,
                                    uint8_t valid,
                                    uint8_t confidence,
                                    int16_t error_norm,
                                    uint32_t arrival_ms,
                                    uint32_t now_ms,
                                    uint8_t pe7_raw_high);

/* Parser path preserving all CRC-validated V1 flags. The legacy entry points
   above intentionally synthesize valid-only observations without a slope hint. */
void car_control_on_vision_frame_at_with_flags(
    car_control_t *control,
    const car_config_t *config,
    uint16_t sequence,
    uint8_t flags,
    uint8_t confidence,
    int16_t error_norm,
    uint32_t arrival_ms,
    uint32_t now_ms,
    uint8_t pe7_raw_high);

/* V2 parser path also supplies the strictly validated Ackermann request. */
void car_control_on_vision_frame_at_with_flags_and_curve(
    car_control_t *control,
    const car_config_t *config,
    uint16_t sequence,
    uint8_t flags,
    uint8_t confidence,
    int16_t error_norm,
    uint8_t curve_phase,
    uint16_t slowdown_request_permille,
    uint32_t arrival_ms,
    uint32_t now_ms,
    uint8_t pe7_raw_high);

/* Call only after outputs are stopped and RX/UART queues are atomically flushed. */
void car_control_rx_recovery_flush_complete(car_control_t *control);

/* Main-loop calls after the reverse RESET phase / first complete KEEP frame. */
void car_control_reverse_control_reset_started(car_control_t *control);
void car_control_reverse_control_keep_started(car_control_t *control);
void car_control_reverse_control_reset_phase_complete(car_control_t *control);

/* Begin a new Follow session from one consumed physical PE7 start event. */
uint8_t car_control_begin_manual_follow_start(
    car_control_t *control,
    const car_config_t *config,
    const car_inputs_t *inputs);

/* Recoverable receive errors stop outputs and require three fresh observations. */
void car_control_note_fault(car_control_t *control, car_fault_t fault);

/* Consume one command mailbox value. Caller must clear the mailbox first. */
uint32_t car_control_command(car_control_t *control,
                             const car_config_t *config,
                             const car_inputs_t *inputs,
                             car_command_t command,
                             uint8_t selected_motor,
                             uint8_t selected_input,
                             uint16_t requested_duty_permille);

/* HAL-independent Bench preparation path shared by main and host tests. */
uint32_t car_control_auto_select_bench_output(
    car_control_t *control,
    const car_config_t *config,
    const car_inputs_t *inputs,
    uint8_t rx_path_healthy);
/* Logical TIM3 mask: bit 0..3 select CH1..CH4; both-forward is CH2|CH3. */
uint8_t car_control_bench_channel_mask(const car_control_t *control);
uint8_t car_control_bench_ready(const car_control_t *control,
                                const car_config_t *config,
                                const car_inputs_t *inputs,
                                uint8_t rx_path_healthy);

/*
 * ISR fast-stop latch. The hardware ISR also disables TIM3 channels directly.
 * Target callers must save PRIMASK, disable IRQs around every hook call, and
 * restore PRIMASK afterward so higher-priority IRQs cannot nest these RMWs.
 * Main-loop simulations of an ISR event need the same short critical section.
 */
void car_control_note_switch_release_isr(car_control_t *control);
void car_control_note_fast_stop_isr(car_control_t *control, car_fault_t fault);
/* Caller must mask interrupts around this exchange on target hardware. */
car_fault_t car_control_take_fast_stop_isr(car_control_t *control);

/* Snapshot helpers for IRQ/main shared debug values. */
uint8_t car_control_vision_fresh(const car_control_t *control,
                                 uint32_t now_ms,
                                 uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_COMMISSIONING_H */
