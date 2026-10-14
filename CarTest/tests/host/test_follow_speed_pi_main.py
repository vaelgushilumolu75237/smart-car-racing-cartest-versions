"""Run the exact production Follow PI target-preparation helper with host mocks."""

from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "commissioning.h"
#include "follow_output.h"
#include "follow_speed_pi.h"
#include "servo_safety.h"
#include "speed_observer.h"

static car_control_t car_control_state;
static car_config_t app_config;
static car_servo_safety_t servo_safety;
static car_follow_output_state_t follow_output_state;
static car_speed_observer_result_t speed_observer_result;
static car_follow_speed_pi_state_t follow_speed_pi_state;
static volatile uint8_t follow_speed_pi_reset_pending;
static volatile uint8_t motor_outputs_active;
static volatile uint8_t servo_output_active;
static volatile uint8_t servo_stop_pending;
static volatile uint32_t car_speed_pi_actuation_enabled;
static uint8_t applied_follow_outputs;
static uint16_t applied_servo_pulse_us;
static uint16_t applied_motor_follow_left_permille;
static uint16_t applied_motor_follow_right_permille;
static uint32_t test_primask;
static uint32_t test_emergency_stops;
static uint32_t test_last_fault;

static uint32_t __get_PRIMASK(void) { return test_primask; }
static void __disable_irq(void) { test_primask = 1u; }
static void __set_PRIMASK(uint32_t value) { test_primask = value; }

void car_control_note_fault(car_control_t *control, car_fault_t fault)
{
    control->fault = fault;
    control->fault_latched = 1u;
    control->manual_run_request = 0u;
    control->follow_session_active = 0u;
    control->road_drive_allowed = 0u;
    control->mode = CAR_MODE_FAULT_LATCHED;
    test_last_fault = (uint32_t)fault;
}

static void emergency_stop_hardware(void)
{
    ++test_emergency_stops;
    motor_outputs_active = 0u;
    applied_follow_outputs = 0u;
    applied_motor_follow_left_permille = 0u;
    applied_motor_follow_right_permille = 0u;
    servo_output_active = 0u;
    follow_speed_pi_reset_pending = 1u;
}
'''


def extract_function(source: str, name: str) -> str:
    marker = f"static void {name}("
    start = source.find(marker)
    if start < 0:
        raise ValueError(f"static void {name} definition was not found")
    opening = source.find("{", start + len(marker))
    if opening < 0:
        raise ValueError(f"static void {name} opening brace was not found")
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
        else:
            if char == "\\":
                index += 2
                continue
            if (state == "string" and char == '"') or \
                    (state == "char" and char == "'"):
                state = "normal"
        index += 1
    raise ValueError(f"static void {name} braces did not balance")


HARNESS = r'''
#define REQUIRE(label, expression) do { \
    if (!(expression)) { \
        printf("FAIL %s line=%d\n", label, __LINE__); \
        ++failures; \
    } else { \
        ++checks; \
    } \
} while (0)

static unsigned checks;
static unsigned failures;

static void init_running(uint32_t now_ms)
{
    memset(&car_control_state, 0, sizeof(car_control_state));
    memset(&app_config, 0, sizeof(app_config));
    memset(&servo_safety, 0, sizeof(servo_safety));
    memset(&follow_output_state, 0, sizeof(follow_output_state));
    memset(&speed_observer_result, 0, sizeof(speed_observer_result));
    car_follow_speed_pi_reset(&follow_speed_pi_state);
    follow_speed_pi_reset_pending = 0u;
    motor_outputs_active = 1u;
    servo_output_active = 1u;
    servo_stop_pending = 0u;
    applied_follow_outputs = 1u;
    applied_servo_pulse_us = 1530u;
    applied_motor_follow_left_permille = 500u;
    applied_motor_follow_right_permille = 500u;
    test_primask = 0u;
    test_emergency_stops = 0u;
    test_last_fault = CAR_FAULT_NONE;

    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    car_control_state.fault = CAR_FAULT_NONE;
    car_control_state.manual_run_request = 1u;
    car_control_state.follow_session_active = 1u;
    car_control_state.road_drive_allowed = 1u;
    car_control_state.follow_output_duty_permille = 500u;
    app_config.servo_center_us = 1530u;
    app_config.follow_servo_left_us = 1100u;
    app_config.follow_servo_right_us = 2500u;
    servo_safety.frame_ready = 1u;
    follow_output_state.startup_qualified = 1u;
    follow_output_state.have_ramp_time = 1u;
    follow_output_state.last_ramp_ms = now_ms - 10u;
    follow_output_state.command_left_permille = 500u;
    follow_output_state.command_right_permille = 500u;
    speed_observer_result.valid = 1u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    speed_observer_result.sample_ms = now_ms;
    speed_observer_result.elapsed_ms = 10u;
    speed_observer_result.left_millirps = 1800;
    speed_observer_result.right_millirps = 2400;
}

int main(void)
{
    uint32_t before_integral;
    init_running(90u);
    follow_output_state.startup_qualified = 0u;
    prepare_follow_motor_targets(100u);
    REQUIRE("first qualified servo frame preserves legacy 500 command",
            follow_output_state.command_left_permille == 500u &&
            follow_output_state.command_right_permille == 500u);
    REQUIRE("PI stays disarmed on the qualification frame",
            follow_speed_pi_state.armed == 0u);
    speed_observer_result.sample_ms = 110u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    prepare_follow_motor_targets(110u);
    REQUIRE("PI arms on the following fresh sample",
            follow_speed_pi_state.armed != 0u);

    init_running(100u);
    prepare_follow_motor_targets(100u);
    REQUIRE("qualified fresh observer sample arms PI", follow_speed_pi_state.armed != 0u);
    REQUIRE("left target is derived from nominal 500", follow_speed_pi_state.target_left_millirps == 2400);
    REQUIRE("load drop creates greater left proposal", follow_speed_pi_state.proposal_left_permille > 500u);
    REQUIRE("steady right side keeps nominal proposal", follow_speed_pi_state.proposal_right_permille == 500u);
    REQUIRE("first PI command is one 20 permille step", follow_output_state.command_left_permille == 520u);
    REQUIRE("servo and motor startup gates remain represented", servo_output_active != 0u && motor_outputs_active != 0u && servo_safety.frame_ready != 0u);

    applied_motor_follow_left_permille = follow_output_state.command_left_permille;
    applied_motor_follow_right_permille = follow_output_state.command_right_permille;
    before_integral = (uint32_t)follow_speed_pi_state.left.integral_q16;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_WAITING;
    speed_observer_result.left_millirps = 0;
    prepare_follow_motor_targets(101u);
    REQUIRE("duplicate sample does not update integral", (uint32_t)follow_speed_pi_state.left.integral_q16 == before_integral);
    REQUIRE("duplicate sample retains PI proposal", follow_speed_pi_state.proposal_left_permille > 500u);

    prepare_follow_motor_targets(110u);
    REQUIRE("stored proposal ramps on next 10 ms tick", follow_output_state.command_left_permille == 540u);
    REQUIRE("per-side command stays within one step", follow_output_state.command_right_permille == 500u);

    before_integral = (uint32_t)follow_speed_pi_state.left.integral_q16;
    car_control_state.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    car_control_state.vision_hold_active = 1u;
    car_control_state.vision_hold_motor_left_permille = 540u;
    car_control_state.vision_hold_motor_right_permille = 500u;
    car_control_state.vision_hold_servo_pulse_us = 1530u;
    prepare_follow_motor_targets(111u);
    REQUIRE("vision hold freezes left motor output", follow_output_state.command_left_permille == 540u);
    REQUIRE("vision hold freezes right motor output", follow_output_state.command_right_permille == 500u);
    REQUIRE("vision hold does not integrate speed error", (uint32_t)follow_speed_pi_state.left.integral_q16 == before_integral);
    REQUIRE("PI adapter reports exact hold", follow_speed_pi_state.status == CAR_FOLLOW_SPEED_PI_STATUS_HOLD);

    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    car_control_state.vision_hold_active = 0u;
    applied_motor_follow_left_permille = 540u;
    applied_motor_follow_right_permille = 500u;
    prepare_follow_motor_targets(112u);
    REQUIRE("hold resume starts from actual applied left output", follow_output_state.command_left_permille == 540u);
    REQUIRE("hold resume preserves integral", (uint32_t)follow_speed_pi_state.left.integral_q16 == before_integral);

    follow_speed_pi_state.left.integral_q16 = 1200;
    follow_speed_pi_reset_pending = 1u;
    car_control_state.mode = CAR_MODE_WAIT_START_EDGE;
    prepare_follow_motor_targets(113u);
    REQUIRE("deferred stop reset is consumed in main context", follow_speed_pi_reset_pending == 0u);
    REQUIRE("deferred reset clears PI history", follow_speed_pi_state.left.integral_q16 == 0);
    REQUIRE("deferred reset revokes PI arm", follow_speed_pi_state.armed == 0u);

    init_running(200u);
    speed_observer_result.valid = 0u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_REVERSE_DELTA;
    prepare_follow_motor_targets(200u);
    REQUIRE("qualified reverse encoder faults with encoder code", test_last_fault == CAR_FAULT_ENCODER_INIT);
    REQUIRE("qualified encoder fault immediately requests stop", test_emergency_stops == 1u);
    REQUIRE("fault does not retain road permission", car_control_state.road_drive_allowed == 0u);

    init_running(100u);
    speed_observer_result.valid = 0u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_NOT_READY;
    speed_observer_result.sample_ms = 0u;
    prepare_follow_motor_targets(100u);
    REQUIRE("qualified PI waits through the 100 ms initial NOT_READY window",
            test_last_fault == CAR_FAULT_NONE && test_emergency_stops == 0u &&
            follow_speed_pi_state.armed == 0u);

    init_running(101u);
    speed_observer_result.valid = 0u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_NOT_READY;
    speed_observer_result.sample_ms = 0u;
    prepare_follow_motor_targets(101u);
    REQUIRE("initial observer silence faults after the 100 ms bound",
            test_last_fault == CAR_FAULT_ENCODER_INIT &&
            test_emergency_stops == 1u);

    printf("FOLLOW_SPEED_PI_MAIN_SUMMARY assertions=%u failures=%u\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()
    source_path = args.source.resolve()
    source_text = source_path.read_text(encoding="utf-8-sig")
    function_names = ("consume_follow_speed_pi_reset_request",
                      "prepare_follow_motor_targets")
    functions = "\n\n".join(extract_function(source_text, name)
                             for name in function_names)
    motor_apply = extract_function(source_text, "motor_apply")

    coast_tokens = (
        "follow_speed_pi_state.armed != 0u",
        "CAR_FOLLOW_SPEED_PI_STATUS_RUNNING",
        "follow_output_state.startup_qualified != 0u",
        "car_control_state.manual_run_request != 0u",
        "car_control_state.follow_session_active != 0u",
        "car_control_state.road_drive_allowed != 0u",
        "servo_safety.frame_ready != 0u",
        "servo_stop_pending == 0u",
        "duty == 0u && pi_zero_coast_allowed == 0u",
        "motor_output_guard(follow_mode, channel_mask, &generations)",
    )
    missing = [token for token in coast_tokens if token not in motor_apply]
    if missing:
        raise SystemExit("PI zero-coast safety checks are missing: " + ", ".join(missing))
    if motor_apply.find("duty == 0u && pi_zero_coast_allowed == 0u") > \
            motor_apply.find("motor_output_guard(follow_mode, channel_mask, &generations)"):
        raise SystemExit("PI zero-coast bypass moved past the independent output guard")

    observer_motion = re.search(
        r"if\s*\(observer_base_expected[^\)]*\).*?observer_motion_expected\s*=\s*1u",
        source_text,
        re.DOTALL,
    )
    if not observer_motion or "follow_speed_pi_state.target_left_millirps > 0" not in observer_motion.group(0) or \
            "follow_speed_pi_state.target_right_millirps > 0" not in observer_motion.group(0):
        raise SystemExit("PI zero-coast requests do not keep stall observation active")
    if source_text.find("prepare_follow_motor_targets(HAL_GetTick());") > \
            source_text.find("motor_apply();", source_text.find("prepare_follow_motor_targets(HAL_GetTick());")):
        raise SystemExit("Production main loop must prepare the PI proposal before motor_apply")

    source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix="CarTestFollowSpeedPI-") as temp_name:
        temp = Path(temp_name)
        c_file = temp / "follow_speed_pi_main.c"
        executable = temp / "follow_speed_pi_main.exe"
        c_file.write_text(PRELUDE + "\n" + functions + "\n" + HARNESS,
                          encoding="ascii")
        built = subprocess.run(
            ["cl", "/nologo", "/std:c11", "/W4", "/TC",
             "/DCARTEST_FOLLOW_SPEED_PI_ENABLED=1",
             "/DCARTEST_FOLLOW_TURN_PROFILE_COMPILED=1",
             "/DCARTEST_FOLLOW_DIFF_PERMILLE=200",
             "/DCARTEST_FOLLOW_INNER_DIFF_PERMILLE=200",
             "/DCARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE=700",
             "/DCARTEST_FOLLOW_HARD_LIMIT_PERMILLE=700",
             "/DCARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE=20",
             "/DCARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE=10",
             "/I" + str(PROJECT_ROOT / "Inc"),
             "/Fe:" + str(executable), str(c_file),
             str(PROJECT_ROOT / "App" / "follow_output.c"),
             str(PROJECT_ROOT / "App" / "speed_pi.c"),
             str(PROJECT_ROOT / "App" / "follow_speed_pi.c")],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            errors="replace",
            check=False,
        )
        if built.returncode != 0:
            raise RuntimeError("Production main harness compile failed:\n" +
                               built.stdout + built.stderr)
        run = subprocess.run([str(executable)], cwd=PROJECT_ROOT,
                             capture_output=True, text=True, errors="replace",
                             check=False)
        output = run.stdout + run.stderr
        print("SOURCE {} SHA256 {}".format(source_path, source_hash))
        print(output, end="")
        if run.returncode != 0 or \
                "FOLLOW_SPEED_PI_MAIN_SUMMARY assertions=27 failures=0" not in output:
            raise RuntimeError("Production Follow speed PI main harness failed.")
    print("FOLLOW_SPEED_PI_MAIN_PRODUCTION_SOURCE_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
