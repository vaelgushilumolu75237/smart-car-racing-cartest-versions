"""Compile the exact servo_apply() body extracted from Src/main.c with HAL mocks."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import subprocess
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CARTEST_HARDWARE_ACCEPTED 1u
#define CAR_SERVO_SAFETY_MIN_US 1100u
#define CAR_SERVO_SAFETY_MAX_US 2500u
#define CAR_SERVO_SAFETY_SLEW_STEP_US 10u
#define CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US __FOLLOW_NORMAL_SLEW_STEP_US__u
#define APP_MOTOR_CHANNEL_MASK 0x0fu
#define TIM_CR1_CEN 0x0001u
#define TIM_CCER_CC1E 0x0001u
#define TIM_BDTR_MOE 0x8000u

typedef enum {
    CAR_MODE_IDLE = 0,
    CAR_MODE_FOLLOW_RUNNING = 1,
    CAR_MODE_DEBUG_STEERING_RUNNING = 2,
    CAR_MODE_FOLLOW_VISION_HOLD = 12
} car_mode_t;

typedef enum {
    CAR_FAULT_NONE = 0,
    CAR_FAULT_CONFIG = 1,
    CAR_FAULT_VISION_TIMEOUT = 2
} car_fault_t;

typedef enum {
    CAR_SERVO_SOURCE_FOLLOW = 1,
    CAR_SERVO_SOURCE_DEBUG_STEERING = 2
} car_servo_source_t;

typedef struct {
    car_mode_t mode;
    uint8_t follow_session_active;
    uint8_t steering_session_active;
    uint16_t follow_servo_target_us;
    uint16_t steering_servo_target_us;
    uint8_t vision_hold_active;
    uint16_t vision_hold_servo_pulse_us;
    uint8_t manual_run_request;
    car_fault_t fault;
} car_control_state_t;

typedef struct {
    uint8_t debug_steering_mode_compiled;
    uint8_t follow_mode_compiled;
    uint8_t follow_calibration_ready;
    uint8_t follow_positive_error_right_measured;
    uint16_t follow_servo_left_us;
    uint16_t follow_servo_right_us;
    uint16_t vision_timeout_ms;
} app_config_t;

typedef struct {
    uint8_t fault_latched;
    uint8_t active;
    uint16_t compare_us;
} car_servo_safety_state_t;

typedef struct {
    uint8_t startup_qualified;
} car_follow_output_state_t;

typedef struct {
    uint32_t marker;
} output_generation_snapshot_t;

typedef struct {
    uint32_t CR1;
    uint32_t CCER;
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
    uint32_t BDTR;
} tim_regs_t;

static car_control_state_t car_control_state;
static app_config_t app_config;
static car_servo_safety_state_t servo_safety;
static car_follow_output_state_t follow_output_state;
static tim_regs_t tim1_regs;
static tim_regs_t tim3_regs;
#define TIM1 (&tim1_regs)
#define TIM3 (&tim3_regs)

static uint32_t primask_state;
static uint32_t primask_get_count;
static uint32_t disable_count;
static uint32_t restore_count;
static uint32_t last_restore_value;
static uint32_t motor_guard_result;
static uint32_t steering_guard_result;
static uint32_t request_result;
static uint32_t request_count;
static uint32_t request_allowed;
static uint32_t request_slew_step_us;
static car_servo_source_t request_source;
static uint32_t motor_guard_count;
static uint32_t steering_guard_count;
static uint32_t guard_irq_disabled;
static uint32_t request_irq_disabled;
static uint32_t commit_irq_disabled;
static uint32_t emergency_stop_count;
static uint32_t stop_motor_count;
static uint32_t stop_servo_count;
static uint32_t motor_outputs_active;
static uint32_t servo_output_active;
static uint16_t applied_servo_pulse_us;
static uint32_t servo_stop_pending;
static void *servo_safety_io;

static uint32_t __get_PRIMASK(void);
static void __disable_irq(void);
static void __set_PRIMASK(uint32_t value);
static uint32_t HAL_GetTick(void);
static void output_generation_snapshot(output_generation_snapshot_t *snapshot);
static uint8_t motor_output_guard(uint8_t a, uint8_t b,
                                  const output_generation_snapshot_t *snapshot);
static uint8_t debug_steering_output_guard(
    const output_generation_snapshot_t *snapshot);
static uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_state_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, uint16_t slew_step_us, void *io);
static uint8_t car_servo_safety_request_selected(
    car_servo_safety_state_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, void *io);
static void servo_safety_fault_to_controller(void);
static void emergency_stop_hardware(void);
static void stop_motor_outputs_hardware(void);
static void stop_servo_outputs_hardware(void);
static void car_control_note_fault(car_control_state_t *state, car_fault_t fault);
static uint8_t car_control_vision_fresh(const car_control_state_t *state,
                                        uint32_t now_ms, uint16_t timeout_ms);
static uint16_t car_control_follow_servo_output_target_us(
    const car_control_state_t *state);
#define __DSB() ((void)0)
'''

SUPPORT = r'''
static uint32_t __get_PRIMASK(void)
{
    ++primask_get_count;
    return primask_state;
}

static void __disable_irq(void)
{
    ++disable_count;
    primask_state = 1u;
}

static void __set_PRIMASK(uint32_t value)
{
    ++restore_count;
    last_restore_value = value;
    if (servo_output_active != 0u || applied_servo_pulse_us != 0u) {
        commit_irq_disabled = primask_state == 1u ? 1u : 0u;
    }
    primask_state = value;
}

static uint32_t HAL_GetTick(void)
{
    return 1234u;
}

static uint16_t car_control_follow_servo_output_target_us(
    const car_control_state_t *state)
{
    return state->follow_servo_target_us;
}

static void output_generation_snapshot(output_generation_snapshot_t *snapshot)
{
    snapshot->marker = 1u;
}

static uint8_t motor_output_guard(uint8_t a, uint8_t b,
                                  const output_generation_snapshot_t *snapshot)
{
    (void)a;
    (void)b;
    (void)snapshot;
    ++motor_guard_count;
    guard_irq_disabled &= primask_state == 1u ? 1u : 0u;
    return (uint8_t)motor_guard_result;
}

static uint8_t debug_steering_output_guard(
    const output_generation_snapshot_t *snapshot)
{
    (void)snapshot;
    ++steering_guard_count;
    guard_irq_disabled &= primask_state == 1u ? 1u : 0u;
    return (uint8_t)steering_guard_result;
}

static uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_state_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, uint16_t slew_step_us, void *io)
{
    (void)now_ms;
    (void)io;
    ++request_count;
    request_allowed = permitted;
    request_slew_step_us = slew_step_us;
    request_source = source;
    request_irq_disabled &= primask_state == 1u ? 1u : 0u;
    if (permitted != 0u && request_result != 0u) {
        safety->active = 1u;
        safety->compare_us = source == CAR_SERVO_SOURCE_FOLLOW
                                 ? follow_target : debug_target;
    }
    return (uint8_t)request_result;
}

static uint8_t car_servo_safety_request_selected(
    car_servo_safety_state_t *safety, car_servo_source_t source,
    uint16_t follow_target, uint16_t debug_target, uint8_t permitted,
    uint32_t now_ms, void *io)
{
    return car_servo_safety_request_selected_with_slew(
        safety, source, follow_target, debug_target, permitted, now_ms,
        CAR_SERVO_SAFETY_SLEW_STEP_US, io);
}

static void servo_safety_fault_to_controller(void)
{
}

static void emergency_stop_hardware(void)
{
    ++emergency_stop_count;
}

static void stop_motor_outputs_hardware(void)
{
    ++stop_motor_count;
    motor_outputs_active = 0u;
}

static void stop_servo_outputs_hardware(void)
{
    ++stop_servo_count;
    servo_output_active = 0u;
}

static void car_control_note_fault(car_control_state_t *state, car_fault_t fault)
{
    state->fault = fault;
}

static uint8_t car_control_vision_fresh(const car_control_state_t *state,
                                        uint32_t now_ms, uint16_t timeout_ms)
{
    (void)state;
    (void)now_ms;
    (void)timeout_ms;
    return 1u;
}

typedef enum {
    CASE_FOLLOW_SUCCESS = 0,
    CASE_FOLLOW_GUARD_REJECT = 1,
    CASE_FOLLOW_REQUEST_FAILURE = 2,
    CASE_STEERING_SUCCESS = 3,
    CASE_STEERING_GUARD_REJECT = 4,
    CASE_IDLE = 5,
    CASE_STOP_PENDING = 6,
    CASE_FOLLOW_SUCCESS_NORMAL_SLEW = 7
} scenario_t;

static uint32_t failures;
static uint32_t cases_run;

static void expect_value(const char *case_name, const char *property,
                         uint32_t expected, uint32_t actual)
{
    if (expected != actual) {
        ++failures;
        printf("FAIL %s: %s expected=%u actual=%u\n",
               case_name, property, expected, actual);
    }
}

static int run_case(scenario_t scenario, uint32_t caller_primask)
{
    const char *base_name = "unknown";
    char case_name[96];
    uint32_t expected_request = 0u;
    uint32_t expected_motor_guard = 0u;
    uint32_t expected_steering_guard = 0u;
    uint32_t expected_allow = 0u;
    uint32_t expected_output = 0u;
    uint32_t expected_source = CAR_SERVO_SOURCE_FOLLOW;
    uint32_t expected_emergency_stop = 0u;
    uint32_t expected_servo_stop = 0u;
    uint32_t expected_critical = 0u;
    uint32_t expected_request_success = 1u;
    uint32_t expected_slew_step_us = CAR_SERVO_SAFETY_SLEW_STEP_US;

    memset(&car_control_state, 0, sizeof(car_control_state));
    memset(&app_config, 0, sizeof(app_config));
    memset(&servo_safety, 0, sizeof(servo_safety));
    memset(&follow_output_state, 0, sizeof(follow_output_state));
    memset(&tim1_regs, 0, sizeof(tim1_regs));
    memset(&tim3_regs, 0, sizeof(tim3_regs));
    primask_state = caller_primask;
    primask_get_count = disable_count = restore_count = last_restore_value = 0u;
    motor_guard_result = steering_guard_result = request_result = 1u;
    request_count = request_allowed = request_slew_step_us =
        motor_guard_count = steering_guard_count = 0u;
    request_source = CAR_SERVO_SOURCE_FOLLOW;
    guard_irq_disabled = request_irq_disabled = commit_irq_disabled = 1u;
    emergency_stop_count = stop_motor_count = stop_servo_count = 0u;
    motor_outputs_active = servo_output_active = servo_stop_pending = 0u;
    applied_servo_pulse_us = 0u;

    car_control_state.follow_servo_target_us = 1620u;
    car_control_state.steering_servo_target_us = 1470u;
    app_config.follow_mode_compiled = 1u;
    app_config.follow_calibration_ready = 1u;
    app_config.follow_positive_error_right_measured = 1u;
    app_config.debug_steering_mode_compiled = 1u;
    app_config.follow_servo_left_us = 1100u;
    app_config.follow_servo_right_us = 2500u;
    app_config.vision_timeout_ms = 100u;

    switch (scenario) {
    case CASE_FOLLOW_SUCCESS:
        base_name = "follow_success";
        car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
        car_control_state.follow_session_active = 1u;
        car_control_state.manual_run_request = 1u;
        expected_request = expected_motor_guard = expected_allow = 1u;
        expected_output = expected_critical = 1u;
        break;
    case CASE_FOLLOW_GUARD_REJECT:
        base_name = "follow_guard_reject";
        car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
        car_control_state.follow_session_active = 1u;
        car_control_state.manual_run_request = 1u;
        motor_guard_result = 0u;
        expected_request = expected_motor_guard = expected_critical = 1u;
        expected_emergency_stop = 1u;
        break;
    case CASE_FOLLOW_REQUEST_FAILURE:
        base_name = "follow_request_failure";
        car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
        car_control_state.follow_session_active = 1u;
        car_control_state.manual_run_request = 1u;
        request_result = 0u;
        expected_request = expected_motor_guard = expected_allow = 1u;
        expected_critical = 1u;
        expected_request_success = 0u;
        break;
    case CASE_FOLLOW_SUCCESS_NORMAL_SLEW:
        base_name = "follow_success_normal_slew";
        car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
        car_control_state.follow_session_active = 1u;
        car_control_state.manual_run_request = 1u;
        follow_output_state.startup_qualified = 1u;
        expected_request = expected_motor_guard = expected_allow = 1u;
        expected_output = expected_critical = 1u;
        expected_slew_step_us = CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US;
        break;
    case CASE_STEERING_SUCCESS:
        base_name = "steering_success";
        car_control_state.mode = CAR_MODE_DEBUG_STEERING_RUNNING;
        car_control_state.steering_session_active = 1u;
        expected_request = expected_steering_guard = expected_allow = 1u;
        expected_source = CAR_SERVO_SOURCE_DEBUG_STEERING;
        expected_output = expected_critical = 1u;
        break;
    case CASE_STEERING_GUARD_REJECT:
        base_name = "steering_guard_reject";
        car_control_state.mode = CAR_MODE_DEBUG_STEERING_RUNNING;
        car_control_state.steering_session_active = 1u;
        steering_guard_result = 0u;
        expected_request = expected_steering_guard = expected_critical = 1u;
        expected_source = CAR_SERVO_SOURCE_DEBUG_STEERING;
        expected_servo_stop = 1u;
        break;
    case CASE_IDLE:
        base_name = "idle";
        break;
    case CASE_STOP_PENDING:
        base_name = "stop_pending";
        car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
        car_control_state.follow_session_active = 1u;
        servo_stop_pending = 1u;
        break;
    }
    (void)snprintf(case_name, sizeof(case_name), "%s caller=%u",
                   base_name, (unsigned)caller_primask);
    ++cases_run;

    servo_apply();

    expect_value(case_name, "final_primask", caller_primask, primask_state);
    expect_value(case_name, "primask_reads", expected_critical, primask_get_count);
    expect_value(case_name, "disable_calls", expected_critical, disable_count);
    expect_value(case_name, "restore_calls", expected_critical, restore_count);
    if (expected_critical != 0u) {
        expect_value(case_name, "restore_value", caller_primask, last_restore_value);
    }
    expect_value(case_name, "motor_guard_calls", expected_motor_guard,
                 motor_guard_count);
    expect_value(case_name, "steering_guard_calls", expected_steering_guard,
                 steering_guard_count);
    expect_value(case_name, "request_calls", expected_request, request_count);
    expect_value(case_name, "request_permitted", expected_allow, request_allowed);
    if (expected_request != 0u) {
        expect_value(case_name, "request_source", expected_source, request_source);
        expect_value(case_name, "request_slew_step_us",
                     expected_slew_step_us, request_slew_step_us);
        expect_value(case_name, "guard_ran_irq_disabled", 1u,
                     guard_irq_disabled);
        expect_value(case_name, "request_ran_irq_disabled", 1u,
                     request_irq_disabled);
    }
    expect_value(case_name, "servo_output_active", expected_output,
                 servo_output_active);
    expect_value(case_name, "pwm_commit_irq_disabled",
                 expected_output, expected_output == 0u ? 0u : commit_irq_disabled);
    expect_value(case_name, "applied_pulse",
                 expected_output != 0u
                     ? (expected_source == CAR_SERVO_SOURCE_FOLLOW ? 1620u : 1470u)
                     : 0u,
                 applied_servo_pulse_us);
    expect_value(case_name, "emergency_stop_calls", expected_emergency_stop,
                 emergency_stop_count);
    expect_value(case_name, "servo_stop_calls", expected_servo_stop,
                 stop_servo_count);
    if (expected_request != 0u) {
        expect_value(case_name, "request_return_expected",
                     expected_request_success,
                     request_result);
    }
    return failures == 0u ? 1 : 0;
}

int main(void)
{
    uint32_t caller;
    for (caller = 0u; caller <= 1u; ++caller) {
        (void)run_case(CASE_FOLLOW_SUCCESS, caller);
        (void)run_case(CASE_FOLLOW_GUARD_REJECT, caller);
        (void)run_case(CASE_FOLLOW_REQUEST_FAILURE, caller);
        (void)run_case(CASE_FOLLOW_SUCCESS_NORMAL_SLEW, caller);
        (void)run_case(CASE_STEERING_SUCCESS, caller);
        (void)run_case(CASE_STEERING_GUARD_REJECT, caller);
        (void)run_case(CASE_IDLE, caller);
        (void)run_case(CASE_STOP_PENDING, caller);
    }
    printf("SERVO_APPLY_PRIMASK_SUMMARY cases=%u failures=%u\n",
           (unsigned)cases_run, (unsigned)failures);
    return failures == 0u ? 0 : 1;
}
'''


def extract_function(source: str) -> str:
    marker = "static void servo_apply(void)"
    start = 0
    opening = -1
    while True:
        start = source.find(marker, start)
        if start < 0:
            raise ValueError("static void servo_apply(void) definition was not found")
        brace = source.find("{", start + len(marker))
        semicolon = source.find(";", start + len(marker))
        if brace >= 0 and (semicolon < 0 or brace < semicolon):
            opening = brace
            break
        start += len(marker)

    depth = 0
    state = "normal"
    index = opening
    while index < len(source):
        char = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "normal":
            if char == "/" and following == "/":
                state = "line_comment"
                index += 2
                continue
            if char == "/" and following == "*":
                state = "block_comment"
                index += 2
                continue
            if char == '"':
                state = "string"
            elif char == "'":
                state = "char"
            elif char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    return source[start:index + 1]
        elif state == "line_comment":
            if char == "\n":
                state = "normal"
        elif state == "block_comment":
            if char == "*" and following == "/":
                state = "normal"
                index += 2
                continue
        elif state in ("string", "char"):
            if char == "\\":
                index += 2
                continue
            if (state == "string" and char == '"') or (state == "char" and char == "'"):
                state = "normal"
        index += 1

    raise ValueError("servo_apply function braces did not balance")


def compile_and_run(source_path: Path, compiler: str, expect_negative: bool,
                    follow_slew_step_us: int) -> tuple[int, str]:
    source_text = source_path.read_text(encoding="utf-8-sig")
    function = extract_function(source_text)
    source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix="CarTestServoApplyPrimask-") as temp_name:
        temp = Path(temp_name)
        c_file = temp / "servo_apply_primask.c"
        executable = temp / "servo_apply_primask.exe"
        prelude = PRELUDE.replace(
            "__FOLLOW_NORMAL_SLEW_STEP_US__", str(follow_slew_step_us)
        )
        c_file.write_text(prelude + "\n" + function + "\n" + SUPPORT, encoding="ascii")
        built = subprocess.run(
            [compiler, "/nologo", "/std:c11", "/W4", "/TC",
             "/Fe:" + str(executable), str(c_file)],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
        )
        if built.returncode != 0:
            raise RuntimeError("Harness compile failed for {}:\n{}\n{}".format(
                source_path, built.stdout, built.stderr))
        run = subprocess.run(
            [str(executable)],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
        )
        output = run.stdout + run.stderr
        print("SOURCE {} SHA256 {}".format(source_path, source_hash))
        print(output, end="")
        if expect_negative:
            required = (
                "FAIL follow_success caller=0: final_primask expected=0 actual=1",
                "FAIL steering_success caller=0: final_primask expected=0 actual=1",
            )
            if run.returncode == 0 or any(token not in output for token in required):
                raise RuntimeError(
                    "Baseline negative control did not detect both success-path PRIMASK leaks.")
            print("EXPECTED_NEGATIVE_CONTROL_FAILURE: baseline leaks PRIMASK on Follow and Steering success.")
            return run.returncode, output
        if run.returncode != 0:
            raise RuntimeError("Fixed servo_apply harness failed with exit {}.".format(run.returncode))
        if "SERVO_APPLY_PRIMASK_SUMMARY cases=16 failures=0" not in output:
            raise RuntimeError("Fixed servo_apply harness did not report all 16 cases passing.")
        print("FIXED_PRODUCTION_SOURCE_PASS: extracted servo_apply preserved caller PRIMASK.")
        return run.returncode, output


def configured_follow_servo_slew_step_us(build_preset: str | None = None) -> int:
    selected_preset = build_preset or os.environ.get(
        "CARTEST_RUNTIME_LOG_BUILD_PRESET", "DebugFollow"
    )
    if selected_preset not in ("DebugFollow", "DebugFollowTurn"):
        raise SystemExit("build preset must be DebugFollow or DebugFollowTurn")
    build_ninja = PROJECT_ROOT / "build" / selected_preset / "build.ninja"
    if not build_ninja.is_file():
        raise SystemExit("Build the selected preset before running this test: " + str(build_ninja))
    match = re.search(
        r"-DCARTEST_FOLLOW_SERVO_SLEW_STEP_US=(\d+)",
        build_ninja.read_text(encoding="utf-8-sig", errors="replace"),
    )
    if match is None:
        raise SystemExit("Selected build has no CARTEST_FOLLOW_SERVO_SLEW_STEP_US definition")
    return int(match.group(1))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--negative-source", required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument("--compiler", default="cl")
    parser.add_argument("--build-preset", default=None,
                        help="read the selected follow-servo slew value from build/<preset>/build.ninja")
    args = parser.parse_args()

    build_preset = args.build_preset or os.environ.get(
        "CARTEST_RUNTIME_LOG_BUILD_PRESET", "DebugFollow"
    )
    follow_slew_step_us = configured_follow_servo_slew_step_us(build_preset)
    print("FOLLOW_SERVO_SLEW_STEP_US={} BUILD_PRESET={}".format(
        follow_slew_step_us, build_preset
    ))

    negative_source = (PROJECT_ROOT / args.negative_source).resolve()
    fixed_source = (PROJECT_ROOT / args.source).resolve()
    if not negative_source.is_file():
        raise SystemExit("Negative-control source not found: {}".format(negative_source))
    if not fixed_source.is_file():
        raise SystemExit("Production source not found: {}".format(fixed_source))

    compile_and_run(negative_source, args.compiler, expect_negative=True,
                    follow_slew_step_us=follow_slew_step_us)
    compile_and_run(fixed_source, args.compiler, expect_negative=False,
                    follow_slew_step_us=follow_slew_step_us)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
