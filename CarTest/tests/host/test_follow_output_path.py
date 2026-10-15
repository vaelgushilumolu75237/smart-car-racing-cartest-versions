"""Compile the real main.c Follow output gates and apply paths against HAL mocks.

This test extracts production function bodies from Src/main.c. It does not
reimplement motor_output_guard(), motor_apply(), servo_apply(), or the stop
decision. Timer/GPIO/UART registers and lower-level HAL/controller callbacks
are mocked so the C harness can exercise those exact bodies on the host.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_PATH = PROJECT_ROOT / "Src" / "main.c"
DEFAULT_EVIDENCE_DIR = (
    PROJECT_ROOT / "test-data" / "turn-response-20261014" / "control" / "output-path"
)
FUNCTION_NAMES = (
    "prepare_follow_motor_targets",
    "uart_error_icr_mask",
    "record_uart_error_flags",
    "note_uart_error_locked",
    "protected_uart_error_note",
    "motor_channel_enable_mask",
    "output_generation_snapshot",
    "motor_output_guard",
    "motor_apply",
    "debug_steering_output_guard",
    "servo_apply",
    "servo_safety_fault_to_controller",
    "emergency_stop_hardware",
)


PRELUDE = r'''#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "follow_output.h"

#ifndef CARTEST_HARDWARE_ACCEPTED
#define CARTEST_HARDWARE_ACCEPTED 1
#endif

#define CAR_SERVO_SAFETY_MIN_US 1100u
#define CAR_SERVO_SAFETY_MAX_US 2500u
#define CAR_CONTROL_VISION_HOLD_MS 1000u
#define APP_MOTOR_CHANNEL_MASK 0x0fu
#define APP_SWITCH_GPIO_PIN 0x00000080u
#define APP_BENCH_LIMIT_PERMILLE 400u
#define APP_FOLLOW_NORMAL_LIMIT_PERMILLE CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE
#define APP_FOLLOW_HARD_LIMIT_PERMILLE CARTEST_FOLLOW_HARD_LIMIT_PERMILLE
#define CAR_SERVO_SAFETY_SLEW_STEP_US 10u
#define CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US CARTEST_FOLLOW_SERVO_SLEW_STEP_US
#define TIM_CR1_CEN 0x0001u
#define TIM_CCER_CC1E 0x0001u
#define TIM_CCER_CC2E 0x0004u
#define TIM_CCER_CC3E 0x0010u
#define TIM_CCER_CC4E 0x0040u
#define TIM_BDTR_MOE 0x8000u
#define USART_ISR_PE 0x00000001u
#define USART_ISR_FE 0x00000002u
#define USART_ISR_NE 0x00000004u
#define USART_ISR_ORE 0x00000008u
#define USART_ICR_PECF USART_ISR_PE
#define USART_ICR_FECF USART_ISR_FE
#define USART_ICR_NECF USART_ISR_NE
#define USART_ICR_ORECF USART_ISR_ORE
#define CAR_SERVO_FAULT_TIMER_CLOCK 2u

typedef enum {
    CAR_MODE_IDLE = 0,
    CAR_MODE_FOLLOW_RUNNING = 1,
    CAR_MODE_FOLLOW_VISION_HOLD = 2,
    CAR_MODE_BENCH_RUNNING = 3,
    CAR_MODE_DEBUG_STEERING_RUNNING = 4,
    CAR_MODE_WAIT_START_EDGE = 5
} car_mode_t;

typedef enum {
    CAR_FAULT_NONE = 0,
    CAR_FAULT_CONFIG = 1,
    CAR_FAULT_VISION_TIMEOUT = 2,
    CAR_FAULT_UART = 3,
    CAR_FAULT_RX_OVERFLOW = 4,
    CAR_FAULT_CLOCK = 5,
    CAR_FAULT_TIMER = 6
} car_fault_t;

typedef enum { CAR_MOTOR_NONE = 0 } car_motor_t;
typedef enum {
    CAR_SERVO_SOURCE_FOLLOW = 1,
    CAR_SERVO_SOURCE_DEBUG_STEERING = 2
} car_servo_source_t;

typedef struct {
    car_mode_t mode;
    uint8_t follow_session_active;
    uint8_t steering_session_active;
    uint8_t vision_hold_active;
    uint8_t road_drive_allowed;
    uint8_t manual_run_request;
    uint8_t vision_valid;
    uint8_t fault_latched;
    uint8_t isr_stop_request;
    uint8_t isr_release_seen;
    uint8_t rx_recovery_flush_required;
    uint8_t bench_motor;
    uint16_t follow_output_duty_permille;
    uint16_t bench_output_duty_permille;
    uint16_t vision_hold_motor_left_permille;
    uint16_t vision_hold_motor_right_permille;
    uint16_t vision_hold_base_duty_permille;
    uint16_t follow_servo_target_us;
    uint16_t vision_hold_servo_pulse_us;
    uint16_t steering_servo_target_us;
    uint16_t bench_requested_duty_permille;
    uint32_t vision_last_valid_ms;
    uint32_t vision_last_frame_alive_ms;
    uint32_t follow_last_ramp_ms;
    uint32_t fault_generation;
    uint32_t rx_fault_generation;
    uint32_t isr_switch_generation;
    uint32_t isr_uart_generation;
    uint32_t button_low_edge_count;
    uint32_t frame_edge_count;
    car_fault_t fault;
    car_fault_t isr_stop_fault;
} car_control_state_t;

typedef struct {
    uint8_t debug_steering_mode_compiled;
    uint8_t follow_mode_compiled;
    uint8_t follow_calibration_ready;
    uint8_t follow_positive_error_right_measured;
    uint8_t motor_hardware_accepted;
    uint8_t bench_mode_compiled;
    uint16_t follow_servo_left_us;
    uint16_t follow_servo_right_us;
    uint16_t servo_center_us;
    uint16_t follow_max_duty_permille;
    uint16_t vision_timeout_ms;
} app_config_t;

typedef struct {
    uint32_t fault_generation;
    uint32_t isr_fault_generation;
    uint32_t fault_generation_reported;
    uint32_t isr_fault_generation_reported;
    uint8_t uart_fault;
    uint8_t overflow_fault;
    uint8_t protocol_fault;
    uint8_t stale_fault;
} car_vision_rx_t;

typedef struct {
    uint8_t run_request;
    uint8_t stop_pending;
    uint8_t raw_high;
} car_manual_button_t;

typedef struct {
    uint8_t fault_latched;
    uint8_t active;
    uint8_t frame_ready;
    uint16_t compare_us;
    uint32_t sticky_fault_code;
} car_servo_safety_t;

typedef struct { uint32_t marker; } car_servo_safety_io_t;
typedef struct {
    uint32_t control_fault;
    uint32_t control_rx_fault;
    uint32_t control_isr_switch;
    uint32_t control_isr_uart;
    uint32_t rx_parser_fault;
    uint32_t rx_isr_fault;
} output_generation_snapshot_t;
typedef struct {
    uint32_t CR1, CCER, CNT, CCR1, CCR2, CCR3, CCR4, BDTR;
} tim_regs_t;
typedef struct { uint32_t ISR, ICR; } usart_regs_t;
typedef struct { uint32_t PR1; } exti_regs_t;

static car_control_state_t car_control_state;
static app_config_t app_config;
static car_vision_rx_t car_vision_rx;
static car_manual_button_t manual_run_button;
static car_servo_safety_t servo_safety;
static car_follow_output_state_t follow_output_state;
static car_servo_safety_io_t servo_safety_io;
static tim_regs_t tim1_regs, tim3_regs;
static usart_regs_t usart1_regs;
static exti_regs_t exti_regs;
#define TIM1 (&tim1_regs)
#define TIM3 (&tim3_regs)
#define USART1 (&usart1_regs)
#define EXTI (&exti_regs)

static uint32_t primask_state;
static uint32_t mock_now_ms;
static uint8_t bump_switch_generation_on_snapshot_restore;
static uint8_t pe7_high;
static uint8_t servo_output_active;
static uint8_t motor_outputs_active;
static uint8_t servo_stop_pending;
static uint8_t follow_ramp_started;
static uint8_t applied_follow_outputs;
static uint16_t applied_servo_pulse_us;
static uint16_t applied_motor_duty_permille;
static uint16_t applied_motor_follow_left_permille;
static uint16_t applied_motor_follow_right_permille;
static uint8_t applied_motor_channel_mask;
static uint16_t car_motor_hardware_lock_reason;
static uint16_t car_bench_max_applied_duty_permille;
static uint32_t car_uart_parity_error_count, car_uart_framing_error_count;
static uint32_t car_uart_noise_error_count, car_uart_overrun_error_count;
static uint32_t motor_stop_calls, servo_stop_calls;
static uint32_t servo_request_calls, servo_request_permitted;
static uint32_t servo_request_source;
static uint16_t servo_request_follow_target, servo_request_debug_target;
static uint16_t servo_request_slew_step_us;
static uint32_t failures, checks, cases_run;

static uint32_t __get_PRIMASK(void);
static void __disable_irq(void);
static void __set_PRIMASK(uint32_t value);
static uint32_t HAL_GetTick(void);
static void __DSB(void);
static void protected_uart_error_note(uint32_t status);
static uint8_t pe7_is_high(void);
static void set_motor_gpio_low(void);
static void set_motor_gpio_channels_af(uint32_t mask);
static void stop_motor_outputs_hardware(void);
static void stop_servo_outputs_hardware(void);
static void emergency_stop_hardware(void);
static void servo_safety_fault_to_controller(void);
static void output_generation_snapshot(output_generation_snapshot_t *snapshot);
static uint8_t motor_output_guard(uint8_t follow_mode, uint8_t channel_mask,
                                  const output_generation_snapshot_t *snapshot);
static void prepare_follow_motor_targets(uint32_t now_ms);
static uint8_t debug_steering_output_guard(
    const output_generation_snapshot_t *snapshot);
static uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, uint16_t slew_step_us, car_servo_safety_io_t *io);
static void car_control_note_fault(car_control_state_t *state,
                                   car_fault_t fault);
static void car_control_note_fast_stop_isr(car_control_state_t *state,
                                           car_fault_t fault);
static void car_control_note_switch_release_isr(car_control_state_t *state);
static void car_vision_rx_isr_error(car_vision_rx_t *rx);
static uint8_t car_control_vision_fresh(const car_control_state_t *state,
                                        uint32_t now_ms, uint16_t timeout_ms);
static uint8_t car_control_bench_channel_mask(
    const car_control_state_t *state);
'''


SUPPORT = r'''static uint32_t __get_PRIMASK(void) { return primask_state; }
static void __disable_irq(void) { primask_state = 1u; }
static void __set_PRIMASK(uint32_t value)
{
    primask_state = value;
    if (bump_switch_generation_on_snapshot_restore != 0u) {
        bump_switch_generation_on_snapshot_restore = 0u;
        ++car_control_state.isr_switch_generation;
    }
}
static uint32_t HAL_GetTick(void) { return mock_now_ms; }
static void __DSB(void) { }
static uint8_t pe7_is_high(void) { return pe7_high; }
static void set_motor_gpio_low(void) { }
static void set_motor_gpio_channels_af(uint32_t mask) { (void)mask; }

/* HAL stop callbacks clear the mocked registers like the production low-level
   stop helpers. The extracted emergency_stop_hardware() ordering is real. */
static void stop_motor_outputs_hardware(void)
{
    ++motor_stop_calls;
    TIM3->CCER = 0u; TIM3->CCR1 = 0u; TIM3->CCR2 = 0u;
    TIM3->CCR3 = 0u; TIM3->CCR4 = 0u; TIM3->CR1 &= ~TIM_CR1_CEN;
    motor_outputs_active = 0u; applied_motor_duty_permille = 0u;
    applied_motor_follow_left_permille = 0u;
    applied_motor_follow_right_permille = 0u;
    applied_motor_channel_mask = 0u; applied_follow_outputs = 0u;
}
static void stop_servo_outputs_hardware(void)
{
    ++servo_stop_calls;
    TIM1->CCER = 0u; TIM1->CCR1 = 0u; TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    servo_output_active = 0u; applied_servo_pulse_us = 0u;
    servo_safety.active = 0u; servo_safety.compare_us = 0u;
}

static uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, uint16_t slew_step_us, car_servo_safety_io_t *io)
{
    (void)now_ms; (void)io;
    ++servo_request_calls;
    servo_request_permitted = permitted;
    servo_request_source = (uint32_t)source;
    servo_request_follow_target = follow_target;
    servo_request_debug_target = debug_target;
    servo_request_slew_step_us = slew_step_us;
    if (permitted == 0u || safety->fault_latched != 0u) return 0u;
    safety->active = 1u;
    {
        const uint16_t target = source == CAR_SERVO_SOURCE_FOLLOW
                                    ? follow_target : debug_target;
        if (safety->compare_us == 0u) safety->compare_us = target;
        else if (target > safety->compare_us) {
            uint16_t delta = (uint16_t)(target - safety->compare_us);
            safety->compare_us = (uint16_t)(safety->compare_us +
                (delta > slew_step_us ? slew_step_us : delta));
        } else if (target < safety->compare_us) {
            uint16_t delta = (uint16_t)(safety->compare_us - target);
            safety->compare_us = (uint16_t)(safety->compare_us -
                (delta > slew_step_us ? slew_step_us : delta));
        }
    }
    return 1u;
}
static void car_control_note_fault(car_control_state_t *state,
                                   car_fault_t fault)
{
    state->fault = fault; state->fault_latched = 1u;
}
static void car_control_note_fast_stop_isr(car_control_state_t *state,
                                           car_fault_t fault)
{
    state->isr_stop_fault = fault;
    ++state->isr_uart_generation;
}
static void car_control_note_switch_release_isr(car_control_state_t *state)
{
    state->isr_release_seen = 1u;
    ++state->isr_switch_generation;
}
static void car_vision_rx_isr_error(car_vision_rx_t *rx)
{
    rx->uart_fault = 1u;
    ++rx->isr_fault_generation;
}
static uint8_t car_control_vision_fresh(const car_control_state_t *state,
                                        uint32_t now_ms, uint16_t timeout_ms)
{
    (void)now_ms; (void)timeout_ms;
    return state->vision_valid != 0u ? 1u : 0u;
}
static uint8_t car_control_bench_channel_mask(
    const car_control_state_t *state)
{
    return state->bench_motor == CAR_MOTOR_NONE ? 0u : 1u;
}

static void expect_u32(const char *name, const char *field,
                       uint32_t expected, uint32_t actual)
{
    ++checks;
    if (expected != actual) {
        ++failures;
        printf("FAIL case=%s field=%s expected=%lu actual=%lu\n", name,
               field, (unsigned long)expected, (unsigned long)actual);
    }
}
static void expect_primask(const char *name, uint32_t caller)
{
    expect_u32(name, "primask_restored", caller, primask_state);
}
static void reset_mock(uint32_t caller)
{
    memset(&car_control_state, 0, sizeof(car_control_state));
    memset(&app_config, 0, sizeof(app_config));
    memset(&car_vision_rx, 0, sizeof(car_vision_rx));
    memset(&manual_run_button, 0, sizeof(manual_run_button));
    memset(&servo_safety, 0, sizeof(servo_safety));
    memset(&follow_output_state, 0, sizeof(follow_output_state));
    memset(&tim1_regs, 0, sizeof(tim1_regs));
    memset(&tim3_regs, 0, sizeof(tim3_regs));
    memset(&usart1_regs, 0, sizeof(usart1_regs));
    memset(&exti_regs, 0, sizeof(exti_regs));
    primask_state = caller; mock_now_ms = 10000u; pe7_high = 0u;
    bump_switch_generation_on_snapshot_restore = 0u;
    servo_output_active = motor_outputs_active = servo_stop_pending = 0u;
    follow_ramp_started = applied_follow_outputs = 0u;
    applied_servo_pulse_us = applied_motor_duty_permille = 0u;
    applied_motor_follow_left_permille = applied_motor_follow_right_permille = 0u;
    applied_motor_channel_mask = 0u; car_motor_hardware_lock_reason = 0u;
    car_bench_max_applied_duty_permille = 0u;
    car_uart_parity_error_count = car_uart_framing_error_count = 0u;
    car_uart_noise_error_count = car_uart_overrun_error_count = 0u;
    motor_stop_calls = servo_stop_calls = 0u;
    servo_request_calls = servo_request_permitted = servo_request_source = 0u;
    servo_request_follow_target = servo_request_debug_target = 0u;
    servo_request_slew_step_us = 0u;
    app_config.follow_mode_compiled = 1u;
    app_config.follow_calibration_ready = 1u;
    app_config.follow_positive_error_right_measured = 1u;
    app_config.motor_hardware_accepted = 1u;
    app_config.follow_servo_left_us = 1200u;
    app_config.follow_servo_right_us = 2300u;
    app_config.servo_center_us = 1530u;
    app_config.follow_max_duty_permille = APP_FOLLOW_NORMAL_LIMIT_PERMILLE;
    app_config.vision_timeout_ms = 250u;
    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    car_control_state.follow_session_active = 1u;
    car_control_state.road_drive_allowed = 1u;
    car_control_state.manual_run_request = 1u;
    car_control_state.vision_valid = 1u;
    car_control_state.follow_output_duty_permille = 80u;
    car_control_state.follow_servo_target_us = 1520u;
    manual_run_button.run_request = 1u;
    servo_safety_io.marker = 1u;
}
static void start_existing_motor_outputs(uint16_t left, uint16_t right)
{
    motor_outputs_active = 1u; applied_follow_outputs = 1u;
    applied_motor_follow_left_permille = left;
    applied_motor_follow_right_permille = right;
    applied_motor_duty_permille = left > right ? left : right;
    applied_motor_channel_mask = 0xffu;
    TIM3->CR1 = TIM_CR1_CEN;
    TIM3->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM3->CCR2 = left; TIM3->CCR3 = right;
}
static void start_existing_servo(uint16_t pulse)
{
    servo_output_active = 1u; servo_safety.active = 1u;
    servo_safety.frame_ready = 1u; servo_safety.compare_us = pulse;
    applied_servo_pulse_us = pulse;
    TIM1->CR1 = TIM_CR1_CEN; TIM1->CCER = TIM_CCER_CC1E;
    TIM1->BDTR = TIM_BDTR_MOE; TIM1->CCR1 = pulse;
}

static void case_first_frame_wait(uint32_t caller)
{
    reset_mock(caller); ++cases_run;
    servo_apply(); expect_primask("first_frame_servo_request", caller);
    expect_u32("first_frame_servo_request", "startup servo step remains 10us",
               CAR_SERVO_SAFETY_SLEW_STEP_US, servo_request_slew_step_us);
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("first_frame_servo_request", "request_count", 1u,
               servo_request_calls);
    expect_u32("first_frame_servo_request", "servo_active", 1u,
               servo_output_active);
    expect_u32("first_frame_servo_request", "frame_ready_initial", 0u,
               servo_safety.frame_ready);
    motor_apply(); expect_primask("first_frame_wait", caller);
    expect_u32("first_frame_wait", "motor_stop_calls", 0u,
               motor_stop_calls);
    expect_u32("first_frame_wait", "servo_stop_calls", 0u, servo_stop_calls);
    expect_u32("first_frame_wait", "servo_stays_active", 1u,
               servo_output_active);
    expect_u32("first_frame_wait", "ramp_not_started_early", 0u,
               follow_ramp_started);
    expect_u32("first_frame_wait", "motor_active", 0u, motor_outputs_active);
    expect_u32("first_frame_wait", "motor_ccer", 0u, TIM3->CCER);
    expect_u32("first_frame_wait", "ccr1_zero", 0u, TIM3->CCR1);
    expect_u32("first_frame_wait", "left_pwm", 0u, TIM3->CCR2);
    expect_u32("first_frame_wait", "right_pwm", 0u, TIM3->CCR3);
    expect_u32("first_frame_wait", "ccr4_zero", 0u, TIM3->CCR4);

    /* Mock the completed 20 ms update edge; production gate reads frame_ready. */
    servo_safety.frame_ready = 1u;
    mock_now_ms += 10u;
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask("first_complete_frame_ramp_start", caller);
    expect_u32("first_complete_frame_ramp_start", "ramp_started", 1u,
               follow_ramp_started);
    expect_u32("first_complete_frame_ramp_start", "duty_reset_for_ramp", 0u,
               car_control_state.follow_output_duty_permille);
    expect_u32("first_complete_frame_ramp_start", "motor_not_early", 0u,
               motor_outputs_active);
    car_control_state.follow_output_duty_permille = 10u; /* mocked base-ramp tick */
    mock_now_ms += 10u;
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask("ramp_first_tick_wait", caller);
    expect_u32("ramp_first_tick_wait", "first output still waits one bounded step",
               0u, motor_outputs_active);
    mock_now_ms += 10u;
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask("ramp_commit", caller);
    expect_u32("ramp_commit", "motor_active", 1u, motor_outputs_active);
    expect_u32("ramp_commit", "left_pwm", 10u, TIM3->CCR2);
    expect_u32("ramp_commit", "right_pwm", 10u, TIM3->CCR3);
    expect_u32("ramp_commit", "channels_enabled",
               TIM_CCER_CC2E | TIM_CCER_CC3E, TIM3->CCER);
    expect_u32("ramp_commit", "no_motor_stop", 0u, motor_stop_calls);
}

static void case_profile_differential_commit(uint32_t caller,
                                             uint16_t outer_differential,
                                             uint16_t inner_differential)
{
    reset_mock(caller); ++cases_run;
    app_config.follow_servo_left_us = 1100u;
    app_config.follow_servo_right_us = 2500u;
    car_control_state.follow_output_duty_permille = 500u;
    car_control_state.follow_servo_target_us = 2500u;
    follow_ramp_started = 1u;
    start_existing_servo(2500u);
    start_existing_motor_outputs(500u, 500u);

    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("diff_startup", "actual servo frame qualifies startup",
               1u, follow_output_state.startup_qualified);
    expect_u32("diff_right_turn", "right endpoint targets outer left",
               500u + outer_differential, follow_output_state.target_left_permille);
    expect_u32("diff_right_turn", "right endpoint targets inner right",
               500u - inner_differential, follow_output_state.target_right_permille);
    motor_apply();
    mock_now_ms += 10u;
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply();
    expect_u32("diff_right_turn", "left side advances one bounded step",
               outer_differential == 0u ? 500u :
                   (uint32_t)(500u + CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE),
               applied_motor_follow_left_permille);
    expect_u32("diff_right_turn", "right side advances independently",
               inner_differential == 0u ? 500u :
                   (uint32_t)(500u - CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE),
               applied_motor_follow_right_permille);

    /* The controller uses the applied, slew-limited compare. Requesting the
       opposite endpoint must not make the output jump before servo_apply has
       actually committed that endpoint. */
    car_control_state.follow_servo_target_us = 1100u;
    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("diff_applied_servo", "opposite request advances by only one normal-slew step",
               2500u - CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US,
               applied_servo_pulse_us);
    expect_u32("diff_applied_servo", "diff remains tied to the committed right-turn pulse",
               500u + outer_differential, follow_output_state.target_left_permille);
}

static void case_active_hold_frozen(uint32_t caller)
{
    const uint16_t left = 91u, right = 107u, pulse = 1584u;
    reset_mock(caller); ++cases_run;
    mock_now_ms = 5000u;
    car_control_state.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    car_control_state.vision_hold_active = 1u;
    car_control_state.vision_valid = 0u;
    car_control_state.vision_last_valid_ms = 4001u; /* age 999 ms */
    car_control_state.vision_last_frame_alive_ms = 4751u; /* age 249 ms */
    car_control_state.vision_hold_motor_left_permille = left;
    car_control_state.vision_hold_motor_right_permille = right;
    car_control_state.vision_hold_servo_pulse_us = pulse;
    follow_output_state.startup_qualified = 1u;
    follow_ramp_started = 1u;
    start_existing_motor_outputs(left, right); start_existing_servo(pulse);
    servo_apply(); expect_primask("active_hold_servo", caller);
    expect_u32("active_hold_servo", "qualified short hold retains normal servo rate",
               CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US,
               servo_request_slew_step_us);
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask("active_hold_motor", caller);
    expect_u32("active_hold", "vision_invalid", 0u,
               car_control_state.vision_valid);
    expect_u32("active_hold", "held_servo_request", pulse,
               servo_request_follow_target);
    expect_u32("active_hold", "held_servo_compare", pulse,
               applied_servo_pulse_us);
    expect_u32("active_hold", "held_left_duty", left,
               applied_motor_follow_left_permille);
    expect_u32("active_hold", "held_right_duty", right,
               applied_motor_follow_right_permille);
    expect_u32("active_hold", "held_left_pwm", left, TIM3->CCR2);
    expect_u32("active_hold", "held_right_pwm", right, TIM3->CCR3);
    expect_u32("active_hold", "motor_remains_active", 1u, motor_outputs_active);
    expect_u32("active_hold", "no_motor_stop", 0u, motor_stop_calls);
}

static void case_hold_expired(uint32_t caller, uint8_t transport_expired)
{
    const char *name = transport_expired ? "hold_transport_250ms" :
                                           "hold_valid_1000ms";
    reset_mock(caller); ++cases_run;
    mock_now_ms = 6000u;
    car_control_state.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    car_control_state.vision_hold_active = 1u;
    car_control_state.vision_valid = 0u;
    car_control_state.vision_last_valid_ms = transport_expired ? 5501u : 5000u;
    car_control_state.vision_last_frame_alive_ms = transport_expired ? 5750u : 5751u;
    car_control_state.vision_hold_motor_left_permille = 90u;
    car_control_state.vision_hold_motor_right_permille = 100u;
    follow_ramp_started = 1u;
    start_existing_motor_outputs(90u, 100u); start_existing_servo(1550u);
    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask(name, caller);
    expect_u32(name, "motor_stopped", 0u, motor_outputs_active);
    expect_u32(name, "left_pwm_zero", 0u, TIM3->CCR2);
    expect_u32(name, "right_pwm_zero", 0u, TIM3->CCR3);
    expect_u32(name, "channels_disabled", 0u, TIM3->CCER);
    expect_u32(name, "motor_stop_callback_called", 1u,
               motor_stop_calls != 0u ? 1u : 0u);
    expect_u32(name, "servo_stop_callback_called", 1u,
               servo_stop_calls != 0u ? 1u : 0u);
    expect_u32(name, "servo_stopped", 0u, servo_output_active);
}

static void case_normal_raw_release(uint32_t caller)
{
    reset_mock(caller); ++cases_run;
    manual_run_button.raw_high = 1u; /* normal release edge, still request=1 */
    manual_run_button.run_request = 1u;
    EXTI->PR1 = APP_SWITCH_GPIO_PIN;
    pe7_high = 1u;
    start_existing_motor_outputs(80u, 80u); start_existing_servo(1520u);
    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask("normal_raw_release", caller);
    expect_u32("normal_raw_release", "request_still_active", 1u,
               manual_run_button.run_request);
    expect_u32("normal_raw_release", "no_stop_pending", 0u,
               manual_run_button.stop_pending);
    expect_u32("normal_raw_release", "motor_remains_active", 1u,
               motor_outputs_active);
    expect_u32("normal_raw_release", "no_motor_stop", 0u,
               motor_stop_calls);
}

static void case_manual_stop(uint32_t caller, uint8_t stop_pending)
{
    const char *name = stop_pending ? "helper_stop_pending" :
                                      "helper_run_request_zero";
    reset_mock(caller); ++cases_run;
    manual_run_button.run_request = stop_pending ? 1u : 0u;
    manual_run_button.stop_pending = stop_pending;
    car_control_state.manual_run_request = stop_pending ? 1u : 0u;
    start_existing_motor_outputs(80u, 80u); start_existing_servo(1520u);
    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask(name, caller);
    expect_u32(name, "motor_stopped", 0u, motor_outputs_active);
    expect_u32(name, "pwm_zero", 0u, TIM3->CCR2 + TIM3->CCR3);
    expect_u32(name, "motor_stop_callback_called", 1u,
               motor_stop_calls != 0u ? 1u : 0u);
    expect_u32(name, "servo_stop_callback_called", 1u,
               servo_stop_calls != 0u ? 1u : 0u);
}

static void case_follow_guard_rejection(uint32_t caller, uint8_t reason)
{
    const char *name = reason == 0u ? "follow_manual_request_zero_500" :
                       reason == 1u ? "follow_generation_changed_500" :
                                      "follow_uart_ore_500";
    reset_mock(caller); ++cases_run;
    car_control_state.follow_output_duty_permille = 500u;
    follow_output_state.command_left_permille = 500u;
    follow_output_state.command_right_permille = 500u;
    follow_ramp_started = 1u;
    start_existing_motor_outputs(500u, 500u);
    start_existing_servo(1530u);

    if (reason == 0u) {
        manual_run_button.run_request = 0u;
        car_control_state.manual_run_request = 0u;
    } else if (reason == 1u) {
        /* output_generation_snapshot() captures first; this models a switch
           ISR arriving as it restores the caller's enabled IRQ state. */
        bump_switch_generation_on_snapshot_restore = 1u;
    } else {
        USART1->ISR = USART_ISR_ORE;
    }

    expect_u32(name, "initial_follow_output_500", 500u,
               applied_motor_duty_permille);
    motor_apply(); expect_primask(name, caller);
    expect_u32(name, "fault_remains_none", CAR_FAULT_NONE,
               car_control_state.fault);
    expect_u32(name, "fault_not_latched", 0u,
               car_control_state.fault_latched);
    expect_u32(name, "motor_stopped", 0u, motor_outputs_active);
    expect_u32(name, "pwm_zero", 0u, TIM3->CCR2 + TIM3->CCR3);
    expect_u32(name, "channels_disabled", 0u, TIM3->CCER);
    expect_u32(name, "motor_stop_callback_called", 1u,
               motor_stop_calls != 0u ? 1u : 0u);
    if (reason == 1u) {
        expect_u32(name, "switch_generation_changed", 1u,
                   car_control_state.isr_switch_generation);
    } else if (reason == 2u) {
        expect_u32(name, "uart_fault_preserved", CAR_FAULT_UART,
                   car_control_state.isr_stop_fault);
        expect_u32(name, "uart_error_recorded", 1u,
                   car_vision_rx.uart_fault);
        expect_u32(name, "uart_error_cleared", USART_ICR_ORECF,
                   USART1->ICR);
    }
}

static void case_follow_hard_limit_rejected(uint32_t caller)
{
    reset_mock(caller); ++cases_run;
    car_control_state.follow_output_duty_permille = 500u;
    follow_output_state.command_left_permille =
        APP_FOLLOW_HARD_LIMIT_PERMILLE + 1u;
    follow_output_state.command_right_permille = 500u;
    start_existing_motor_outputs(500u, 500u);
    start_existing_servo(1530u);
    motor_apply(); expect_primask("follow_over_700_config", caller);
    expect_u32("follow_over_700_config", "config_fault", CAR_FAULT_CONFIG,
               car_control_state.fault);
    expect_u32("follow_over_700_config", "fault_latched", 1u,
               car_control_state.fault_latched);
    expect_u32("follow_over_700_config", "motor_stopped", 0u,
               motor_outputs_active);
    expect_u32("follow_over_700_config", "pwm_zero", 0u,
               TIM3->CCR2 + TIM3->CCR3);
}

static void case_bench_over_limit_rejected(uint32_t caller)
{
    reset_mock(caller); ++cases_run;
    app_config.bench_mode_compiled = 1u;
    car_control_state.mode = CAR_MODE_BENCH_RUNNING;
    car_control_state.follow_session_active = 0u;
    car_control_state.road_drive_allowed = 0u;
    car_control_state.bench_motor = (car_motor_t)1;
    car_control_state.bench_output_duty_permille =
        APP_BENCH_LIMIT_PERMILLE + 1u;
    start_existing_motor_outputs(APP_BENCH_LIMIT_PERMILLE,
                                 APP_BENCH_LIMIT_PERMILLE);
    motor_apply(); expect_primask("bench_over_400_config", caller);
    expect_u32("bench_over_400_config", "config_fault", CAR_FAULT_CONFIG,
               car_control_state.fault);
    expect_u32("bench_over_400_config", "fault_latched", 1u,
               car_control_state.fault_latched);
    expect_u32("bench_over_400_config", "motor_stopped", 0u,
               motor_outputs_active);
    expect_u32("bench_over_400_config", "pwm_zero", 0u,
               TIM3->CCR2 + TIM3->CCR3);
}

static void case_fault_or_uart(uint32_t caller, uint8_t uart_error)
{
    const char *name = uart_error ? "uart_error_priority" : "latched_fault_priority";
    reset_mock(caller); ++cases_run;
    if (uart_error != 0u) USART1->ISR = USART_ISR_ORE;
    else { car_control_state.fault_latched = 1u;
           car_control_state.fault = CAR_FAULT_CONFIG; }
    start_existing_motor_outputs(80u, 80u); start_existing_servo(1520u);
    servo_apply();
    prepare_follow_motor_targets(mock_now_ms);
    motor_apply(); expect_primask(name, caller);
    expect_u32(name, "motor_stopped", 0u, motor_outputs_active);
    expect_u32(name, "pwm_zero", 0u, TIM3->CCR2 + TIM3->CCR3);
    expect_u32(name, "motor_stop_callback_called", 1u,
               motor_stop_calls != 0u ? 1u : 0u);
    expect_u32(name, "servo_stop_callback_called", 1u,
               servo_stop_calls != 0u ? 1u : 0u);
    if (uart_error != 0u) {
        expect_u32(name, "uart_fault_noted", 1u, car_vision_rx.uart_fault);
        expect_u32(name, "uart_error_cleared", USART_ICR_ORECF, USART1->ICR);
        expect_u32(name, "uart_fault_code", CAR_FAULT_UART,
                   car_control_state.isr_stop_fault);
        expect_u32(name, "overrun_count", 1u, car_uart_overrun_error_count);
    }
}

static void case_hardware_locked(uint32_t caller)
{
    reset_mock(caller); ++cases_run;
    car_control_state.follow_output_duty_permille = 120u;
    car_control_state.follow_session_active = 1u;
    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    start_existing_motor_outputs(120u, 120u);
    motor_apply(); expect_primask("hardware_locked_macro", caller);
    expect_u32("hardware_locked_macro", "motor_outputs_disabled", 0u,
               motor_outputs_active);
    expect_u32("hardware_locked_macro", "pwm_zero", 0u,
               TIM3->CCR2 + TIM3->CCR3);
    expect_u32("hardware_locked_macro", "lock_reason", 1u,
               car_motor_hardware_lock_reason);
    expect_u32("hardware_locked_macro", "road_drive_closed", 0u,
               car_control_state.road_drive_allowed);
    expect_u32("hardware_locked_macro", "run_mode_closed", 1u,
               car_control_state.mode == CAR_MODE_WAIT_START_EDGE ? 1u : 0u);
}

int main(void)
{
    uint32_t caller;
#if CARTEST_HARDWARE_ACCEPTED
    for (caller = 0u; caller <= 1u; ++caller) {
        case_first_frame_wait(caller);
        case_profile_differential_commit(
            caller, (uint16_t)CARTEST_FOLLOW_DIFF_PERMILLE,
            (uint16_t)CARTEST_FOLLOW_INNER_DIFF_PERMILLE);
        case_active_hold_frozen(caller);
        case_hold_expired(caller, 0u);
        case_hold_expired(caller, 1u);
        case_normal_raw_release(caller);
        case_manual_stop(caller, 1u);
        case_manual_stop(caller, 0u);
        case_follow_guard_rejection(caller, 0u);
        if (caller == 0u) case_follow_guard_rejection(caller, 1u);
        case_follow_guard_rejection(caller, 2u);
        case_follow_hard_limit_rejected(caller);
        case_bench_over_limit_rejected(caller);
        case_fault_or_uart(caller, 0u);
        case_fault_or_uart(caller, 1u);
    }
#else
    for (caller = 0u; caller <= 1u; ++caller) case_hardware_locked(caller);
#endif
    printf("FOLLOW_OUTPUT_PATH_SUMMARY macro=%u cases=%lu checks=%lu failures=%lu\n",
           (unsigned)CARTEST_HARDWARE_ACCEPTED, (unsigned long)cases_run,
           (unsigned long)checks, (unsigned long)failures);
    return failures == 0u ? 0 : 1;
}
'''


SERVO_APPLY_16_CASE_RUNNER = r'''from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

root = Path(__PROJECT_ROOT_LITERAL__)
new_path = root / "tests" / "host" / "test_follow_output_path.py"
old_path = root / "tests" / "host" / "test_servo_apply_primask.py"
new_spec = spec_from_file_location("follow_output_path", new_path)
new_module = module_from_spec(new_spec)
new_spec.loader.exec_module(new_module)
old_spec = spec_from_file_location("servo_primask", old_path)
old_module = module_from_spec(old_spec)
old_spec.loader.exec_module(old_module)
# Preserve the servo_apply scenarios/assertions while using a brace-aware extractor
# for main.c's forward declaration and initializing the live Follow request.
old_module.extract_function = lambda text: new_module.extract_function(text, "servo_apply")[0]
old_module.SUPPORT = old_module.SUPPORT.replace(
    "car_control_state.follow_session_active = 1u;",
    "car_control_state.follow_session_active = 1u;\n"
    "        car_control_state.manual_run_request = 1u;")
rc, _ = old_module.compile_and_run(root / "Src" / "main.c", "cl", expect_negative=False)
raise SystemExit(rc)
'''


def extract_function(source: str, name: str) -> tuple[str, int]:
    pattern = re.compile(r"(?m)^static\b[^;{}]*\b" + re.escape(name) + r"\s*\(")
    marker = None
    opening = -1
    for candidate in pattern.finditer(source):
        candidate_open = source.find("{", candidate.end())
        candidate_semicolon = source.find(";", candidate.end())
        if candidate_open >= 0 and (candidate_semicolon < 0 or candidate_open < candidate_semicolon):
            marker = candidate
            opening = candidate_open
            break
    if marker is None:
        raise ValueError(f"static function definition not found: {name}")
    start = marker.start()
    if opening < 0:
        raise ValueError(f"function body not found: {name}")
    depth = 0
    state = "normal"
    index = opening
    while index < len(source):
        char = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "normal":
            if char == "/" and following == "/":
                state = "line_comment"; index += 2; continue
            if char == "/" and following == "*":
                state = "block_comment"; index += 2; continue
            if char == '"': state = "string"
            elif char == "'": state = "char"
            elif char == "{": depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    return source[start:index + 1], source.count("\n", 0, start) + 1
        elif state == "line_comment":
            if char == "\n": state = "normal"
        elif state == "block_comment":
            if char == "*" and following == "/":
                state = "normal"; index += 2; continue
        elif state in ("string", "char"):
            if char == "\\": index += 2; continue
            if (state == "string" and char == '"') or (state == "char" and char == "'"):
                state = "normal"
        index += 1
    raise ValueError(f"function braces did not balance: {name}")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def find_vsdevcmd() -> Path | None:
    candidates = [
        Path(r"D:\vs2026\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"),
    ]
    return next((path for path in candidates if path.is_file()), None)


def compiler_command(compiler: str, args: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    found = shutil.which(compiler)
    if found:
        return subprocess.run([found, *args], cwd=cwd, capture_output=True,
                              text=True, errors="replace", check=False)
    vsdevcmd = find_vsdevcmd()
    if vsdevcmd is None:
        raise RuntimeError("MSVC compiler was not found on PATH and VsDevCmd.bat was not found")
    cl_path = Path(r"D:\vs2026\VC\Tools\MSVC\14.51.36231\bin\Hostx64\x64\cl.exe")
    if not cl_path.is_file():
        cl_path = Path(compiler)
    cmdline = (f'call "{vsdevcmd}" -no_logo -arch=x64 >nul && '
               + subprocess.list2cmdline([str(cl_path), *args]))
    return subprocess.run(cmdline, cwd=cwd, capture_output=True, text=True,
                          errors="replace", check=False, shell=True)


def compile_and_run(macro_value: int, diff_permille: int, c_text: str, compiler: str,
                    evidence: Path) -> tuple[int, int, str, str]:
    label = ("hardware-accepted" if macro_value else "hardware-locked") + \
        f"-diff{diff_permille}"
    with tempfile.TemporaryDirectory(prefix="CarTestFollowOutput-") as temp_name:
        temp = Path(temp_name)
        c_path = temp / "follow_output_path.c"
        exe_path = temp / "follow_output_path.exe"
        c_path.write_text(c_text, encoding="ascii")
        compile_args = ["/nologo", "/std:c11", "/W4", "/TC",
                        "/I" + str(PROJECT_ROOT / "Inc"),
                        "/DCARTEST_FOLLOW_DIFF_PERMILLE=" + str(diff_permille),
                        f"/DCARTEST_HARDWARE_ACCEPTED={macro_value}",
                        "/Fe:" + str(exe_path), str(c_path),
                        str(PROJECT_ROOT / "App" / "follow_output.c")]
        if diff_permille == 200:
            compile_args.extend([
                "/DCARTEST_FOLLOW_INNER_DIFF_PERMILLE=200",
                "/DCARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE=700",
                "/DCARTEST_FOLLOW_HARD_LIMIT_PERMILLE=700",
                "/DCARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE=10",
                "/DCARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE=20",
                "/DCARTEST_FOLLOW_SERVO_SLEW_STEP_US=60",
                "/DCARTEST_FOLLOW_TURN_PROFILE_COMPILED=1",
            ])
        else:
            compile_args.extend([
                f"/DCARTEST_FOLLOW_INNER_DIFF_PERMILLE={diff_permille}",
                "/DCARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE=600",
                "/DCARTEST_FOLLOW_HARD_LIMIT_PERMILLE=700",
                "/DCARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE=10",
                "/DCARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE=10",
                "/DCARTEST_FOLLOW_SERVO_SLEW_STEP_US=40",
                "/DCARTEST_FOLLOW_TURN_PROFILE_COMPILED=0",
            ])
        built = compiler_command(compiler, compile_args, temp)
        compile_log = built.stdout + built.stderr
        (evidence / f"{label}-compile.log").write_text(compile_log, encoding="utf-8")
        (evidence / f"{label}-compile.exitcode").write_text(
            str(built.returncode) + "\n", encoding="ascii")
        if built.returncode != 0:
            (evidence / f"{label}-run.log").write_text(
                "NOT_RUN: MSVC compilation failed\n", encoding="utf-8")
            (evidence / f"{label}-run.exitcode").write_text(
                "-999\n", encoding="ascii")
            return built.returncode, -999, compile_log, ""
        run = subprocess.run([str(exe_path)], cwd=temp, capture_output=True,
                             text=True, errors="replace", check=False)
        run_log = run.stdout + run.stderr
        (evidence / f"{label}-run.log").write_text(run_log, encoding="utf-8")
        (evidence / f"{label}-run.exitcode").write_text(
            str(run.returncode) + "\n", encoding="ascii")
        return built.returncode, run.returncode, compile_log, run_log


def run_servo_apply_16_case(compiler: str, evidence: Path) -> tuple[int, str]:
    runner = evidence / "run_servo_apply_16case.py"
    runner.write_text(
        SERVO_APPLY_16_CASE_RUNNER.replace(
            "__PROJECT_ROOT_LITERAL__", repr(str(PROJECT_ROOT))),
        encoding="utf-8")
    python_args = [sys.executable, "-B", str(runner)]
    if shutil.which(compiler):
        result = subprocess.run(python_args, cwd=PROJECT_ROOT,
                                capture_output=True, text=True,
                                errors="replace", check=False)
    else:
        vsdevcmd = find_vsdevcmd()
        if vsdevcmd is None:
            return 127, "MSVC environment is unavailable for legacy 14-case test\n"
        command = (f'call "{vsdevcmd}" -no_logo -arch=x64 >nul && '
                   + subprocess.list2cmdline(python_args))
        result = subprocess.run(command, cwd=PROJECT_ROOT, shell=True,
                                capture_output=True, text=True,
                                errors="replace", check=False)
    output = result.stdout + result.stderr
    (evidence / "servo-apply-primask-16case.log").write_text(
        output, encoding="utf-8")
    (evidence / "servo-apply-primask-16case.exitcode").write_text(
        str(result.returncode) + "\n", encoding="ascii")
    return result.returncode, output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=SOURCE_PATH)
    parser.add_argument("--compiler", default="cl")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_EVIDENCE_DIR)
    args = parser.parse_args()
    source_path = args.source.resolve()
    evidence_dir = args.output_dir.resolve()
    if not source_path.is_file():
        raise SystemExit(f"production source not found: {source_path}")
    evidence_dir.mkdir(parents=True, exist_ok=True)
    source_bytes = source_path.read_bytes()
    source_text = source_bytes.decode("utf-8-sig")
    extracted = []
    provenance = []
    for name in FUNCTION_NAMES:
        fn, line = extract_function(source_text, name)
        extracted.append(fn)
        provenance.append({"name": name, "source_line": line,
                           "sha256": sha256(fn.encode("utf-8"))})
    c_text = PRELUDE + "\n" + "\n\n".join(extracted) + "\n\n" + SUPPORT
    harness_path = evidence_dir / "follow_output_path_harness.c"
    harness_path.write_text(c_text, encoding="ascii")
    record = {
        "source_path": str(source_path),
        "source_sha256": sha256(source_bytes),
        "test_script_path": str(Path(__file__).resolve()),
        "test_script_sha256": sha256(Path(__file__).read_bytes()),
        "compiler_requested": args.compiler,
        "functions_extracted_from_production_main": provenance,
        "harness_sha256": sha256(c_text.encode("ascii")),
        "evidence_boundary": (
            "The extracted main.c request/gate/commit/emergency-stop decisions are real. "
            "Timer/GPIO/UART registers and lower-level HAL/controller callbacks are host mocks; "
            "no hardware behavior is established by this test."
        ),
    }
    (evidence_dir / "provenance.json").write_text(
        json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    failures = []
    version_result = compiler_command(args.compiler, ["/Bv", "/?"], evidence_dir)
    version_output = version_result.stdout + version_result.stderr
    (evidence_dir / "msvc-version.log").write_text(
        version_output, encoding="utf-8")
    (evidence_dir / "msvc-version.exitcode").write_text(
        str(version_result.returncode) + "\n", encoding="ascii")
    version_match = re.search(
        r"Microsoft \(R\) C/C\+\+ Optimizing Compiler Version ([^\r\n]+)",
        version_output)
    if version_result.returncode != 0 or version_match is None:
        failures.append("MSVC version evidence command failed or returned no banner")
    variants = []
    for macro, diff_permille in ((1, 0), (1, 50), (1, 100), (1, 200), (0, 0)):
        compile_rc, run_rc, _, run_log = compile_and_run(
            macro, diff_permille, c_text, args.compiler, evidence_dir)
        label = ("hardware-accepted" if macro else "hardware-locked") + \
            f"-diff{diff_permille}"
        variants.append({
            "name": label, "hardware_accepted": macro,
            "diff_permille": diff_permille,
            "compile_exit_code": int((evidence_dir / f"{label}-compile.exitcode").read_text().strip()),
            "run_exit_code": int((evidence_dir / f"{label}-run.exitcode").read_text().strip()),
        })
        if compile_rc != 0 or run_rc != 0:
            failures.append(f"{label}: compile={compile_rc} run={run_rc}")
        elif "failures=0" not in run_log:
            failures.append(f"{label}: missing zero-failure summary")
    legacy_rc, legacy_log = run_servo_apply_16_case(args.compiler, evidence_dir)
    if legacy_rc != 0 or "SERVO_APPLY_PRIMASK_SUMMARY cases=16 failures=0" not in legacy_log:
        failures.append(f"servo_apply PRIMASK 16-case invariant failed: exit={legacy_rc}")
    report = {
        **record,
        "variants": variants,
        "servo_apply_primask_16_case": {
            "exit_code": legacy_rc,
            "summary": "SERVO_APPLY_PRIMASK_SUMMARY cases=16 failures=0" in legacy_log,
            "log": "servo-apply-primask-16case.log",
        },
        "compiler": {
            "name": "MSVC C/C++ x64",
            "version_banner": version_match.group(1).strip() if version_match else None,
            "version_exit_code": version_result.returncode,
            "version_log": "msvc-version.log",
        },
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
    }
    (evidence_dir / "REPORT.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("SOURCE {} SHA256 {}".format(source_path, record["source_sha256"]))
    print("COMPILER {}".format(
        version_match.group(1).strip() if version_match else "unavailable"))
    for variant in report["variants"]:
        print("VARIANT {} compile={} run={}".format(
            variant["name"], variant["compile_exit_code"], variant["run_exit_code"]))
        print((evidence_dir / (variant["name"] + "-run.log")).read_text(encoding="utf-8"), end="")
    print("REPORT {}".format(evidence_dir / "REPORT.json"))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
