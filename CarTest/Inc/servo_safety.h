#ifndef CARTEST_SERVO_SAFETY_H
#define CARTEST_SERVO_SAFETY_H

#include <stdint.h>

#ifndef CARTEST_FOLLOW_SERVO_SLEW_STEP_US
#define CARTEST_FOLLOW_SERVO_SLEW_STEP_US 120u
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_SERVO_SAFETY_MIN_US = 1100u,
    CAR_SERVO_SAFETY_CENTER_US = 1530u,
    CAR_SERVO_SAFETY_MAX_US = 2500u,
    CAR_SERVO_SAFETY_SLEW_STEP_US = 10u,
    CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US =
        CARTEST_FOLLOW_SERVO_SLEW_STEP_US,
    CAR_SERVO_SAFETY_SLEW_PERIOD_MS = 20u,
    CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US = 128u
};

typedef enum {
    CAR_SERVO_STOP_ACTION_CLOSE_NOW = 0,
    CAR_SERVO_STOP_ACTION_WAIT = 1,
    CAR_SERVO_STOP_ACTION_CLEAR_COMPARE_AND_WAIT = 2
} car_servo_stop_action_t;

typedef enum {
    CAR_SERVO_FAULT_NONE = 0,
    CAR_SERVO_FAULT_REQUEST_RANGE = 1,
    CAR_SERVO_FAULT_TIMER_CLOCK = 2,
    CAR_SERVO_FAULT_TIMER_CONFIG = 3,
    CAR_SERVO_FAULT_COMPARE_WRITE = 4,
    CAR_SERVO_FAULT_START_OUTPUT = 5
} car_servo_fault_t;

typedef enum {
    CAR_SERVO_SOURCE_FOLLOW = 1,
    CAR_SERVO_SOURCE_DEBUG_STEERING = 2
} car_servo_source_t;

/* Read-only copy of the clock basis and live TIM1 registers. */
typedef struct {
    uint32_t hclk_hz;
    uint32_t pclk2_hz;
    uint32_t sysclk_hz;
    uint32_t rcc_cr;
    uint32_t rcc_cfgr;
    uint32_t rcc_d1cfgr;
    uint32_t rcc_d2cfgr;
    uint32_t rcc_apb2enr;
    uint32_t tim_cr1;
    uint32_t tim_cr2;
    uint32_t tim_smcr;
    uint32_t tim_dier;
    uint32_t tim_ccmr1;
    uint32_t tim_ccer;
    uint32_t tim_bdtr;
    uint32_t tim_psc;
    uint32_t tim_arr;
    uint32_t tim_rcr;
    uint32_t tim_ccr1;
    uint32_t gpioa_moder;
    uint32_t gpioa_odr;
    uint32_t gpioa_afr1;
} car_servo_timer_snapshot_t;

typedef struct {
    void *context;
    void (*capture)(void *context, car_servo_timer_snapshot_t *snapshot);
    uint8_t (*write_compare)(void *context, uint16_t pulse_us);
    uint32_t (*read_compare)(void *context);
    uint8_t (*start_output)(void *context);
    uint8_t (*take_completed_update)(void *context);
    void (*stop_output)(void *context);
} car_servo_safety_io_t;

typedef struct {
    uint16_t requested_us;
    uint16_t limited_us;
    uint16_t compare_us;
    uint16_t last_rejected_request_us;
    uint8_t active;
    uint8_t frame_ready;
    uint8_t discard_first_update;
    uint8_t fault_latched;
    uint8_t sticky_fault_code;
    uint32_t last_slew_ms;
    uint8_t have_slew_time;
    car_servo_timer_snapshot_t last_fault_snapshot;
} car_servo_safety_t;

void car_servo_safety_init(car_servo_safety_t *state);

/* One guard covers every nonzero TIM1 CCR write, including the centre start. */
uint8_t car_servo_safety_request(car_servo_safety_t *state,
                                 uint16_t requested_us,
                                 uint32_t now_ms,
                                 const car_servo_safety_io_t *io);

/* Production mode-selection entry: validates even when output authorization
   is denied, but only a selected and authorized profile can start/write PWM. */
uint8_t car_servo_safety_request_selected(
    car_servo_safety_t *state,
    car_servo_source_t source,
    uint16_t follow_target_us,
    uint16_t debug_target_us,
    uint8_t output_permitted,
    uint32_t now_ms,
    const car_servo_safety_io_t *io);

/* Follow may use the larger step only after its first-PWM and base-motor
   qualification latch.  All other callers keep the 10 us default above. */
uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_t *state,
    car_servo_source_t source,
    uint16_t follow_target_us,
    uint16_t debug_target_us,
    uint8_t output_permitted,
    uint32_t now_ms,
    uint16_t slew_step_us,
    const car_servo_safety_io_t *io);

/* Stop is always permitted and does not erase sticky fault evidence. */
void car_servo_safety_stop(car_servo_safety_t *state,
                           const car_servo_safety_io_t *io);

/* Call only after the controller's existing physical hard-fault ACK succeeds. */
void car_servo_safety_acknowledge(car_servo_safety_t *state);

/* Bounded policy used by the TIM1 stop path.  The guard protects both sides
   of ARR wrap; a stale compare flag during a new high pulse is cleared and
   waits for the next falling edge instead of truncating it. */
car_servo_stop_action_t car_servo_safety_stop_action(
    uint8_t stop_pending,
    uint8_t compare_event,
    uint8_t pin_high,
    uint32_t counter,
    uint32_t arr);

uint32_t car_servo_safety_timer_clock_hz(
    const car_servo_timer_snapshot_t *snapshot);
/* Populate the accepted fixed HSI/64 MHz clock values only when the captured
   live RCC registers match that profile; on mismatch all derived values are
   zeroed so normal timer validation latches a clock fault. */
uint8_t car_servo_safety_apply_fixed_hsi64_profile(
    car_servo_timer_snapshot_t *snapshot);
car_servo_fault_t car_servo_safety_validate_timer(
    const car_servo_timer_snapshot_t *snapshot,
    uint8_t expected_active);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_SERVO_SAFETY_H */
