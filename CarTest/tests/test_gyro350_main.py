"""Execute production main.c wheel-target helper with actual PI/Ackermann code.

HAL registers and the output state are simulated; this cannot verify a motor.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("pi_main", ROOT / "tests/host/test_follow_speed_pi_main.py")
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)

EXTRA = r'''
#include "ackermann_control.h"
#include "car_vision.h"
#include "manual_run_button.h"
#define APP_SWITCH_GPIO_PIN 128u
#define USART_ISR_PE 1u
#define USART_ISR_FE 2u
#define USART_ISR_NE 4u
#define USART_ISR_ORE 8u
typedef struct {uint32_t ISR;} mock_uart_t;
typedef struct {uint32_t PR1;} mock_exti_t;
static mock_uart_t uart_regs;
static mock_exti_t exti_regs;
#define USART1 (&uart_regs)
#define EXTI (&exti_regs)
static uint32_t test_now;
static uint8_t test_authority;
static car_manual_button_t manual_run_button;
static car_vision_rx_t car_vision_rx;
typedef struct {
    uint32_t control_fault, control_rx_fault, control_isr_switch, control_isr_uart;
    uint32_t rx_parser_fault, rx_isr_fault;
} output_generation_snapshot_t;
static uint32_t HAL_GetTick(void) { return test_now; }
static uint8_t pe7_is_high(void) { return 1; }
static void protected_uart_error_note(uint32_t status) { (void)status; }
void car_control_note_switch_release_isr(car_control_t *c) { c->isr_release_seen = 1; }
uint8_t car_control_vision_fresh(const car_control_t *c, uint32_t n, uint32_t t) {
    (void)c; (void)n; (void)t; return 0;
}
uint8_t car_control_bench_channel_mask(const car_control_t *c) { (void)c; return 0; }
uint8_t car_control_follow_authority_ready(const car_control_t *c, const car_config_t *cfg, uint32_t n) {
    (void)c; (void)cfg; (void)n; return test_authority;
}
static car_ackermann_control_state_t ackermann_control_state;
static car_ackermann_control_output_t ackermann_control_output;
static uint8_t ackermann_control_session_ready;
static uint8_t gyro_hold_seen;
static uint16_t gyro_hold_left, gyro_hold_right;
static uint32_t gyro_hold_generation, gyro_hold_started_ms;
static void speed_pi_diag_freeze_if_stopped(uint32_t now_ms) { (void)now_ms; }
'''

HARNESS = r'''
static void running(uint32_t now_ms) {
    init_running(now_ms);
    car_follow_incremental_pi_reset(&follow_incremental_pi_state);
    car_ackermann_control_reset(&ackermann_control_state);
    ackermann_control_session_ready = 0;
    gyro_hold_seen = 0;
    car_control_state.gyro.active = 1;
    car_control_state.gyro.phase = CAR_GYRO_RING_ORBIT;
    car_control_state.gyro.outer_target_millirps = 7392;
    /* An invalid camera route must not determine the ring wheel target. */
    car_control_state.vision_valid = 0;
    car_control_state.vision_curve_phase = 2;
    car_control_state.vision_slowdown_request_permille = 1000;
}
int main(void) {
    running(100);
    applied_servo_pulse_us = 1100;
    prepare_follow_motor_targets(100);
    REQUIRE("left orbit outer is 7392", ackermann_control_output.right_target_millirps == 7392);
    REQUIRE("left orbit inner is lower", ackermann_control_output.left_target_millirps < 7392);
    REQUIRE("real PI receives gyro outer", follow_incremental_pi_state.target_right_millirps == 7392);
    REQUIRE("real PI receives inner", follow_incremental_pi_state.target_left_millirps == ackermann_control_output.left_target_millirps);
    REQUIRE("no visual validity fabricated", car_control_state.vision_valid == 0);
    REQUIRE("no fault", test_emergency_stops == 0);
    running(100);
    applied_servo_pulse_us = 2500;
    prepare_follow_motor_targets(100);
    REQUIRE("right orbit mirrored", ackermann_control_output.left_target_millirps == 7392 && ackermann_control_output.right_target_millirps < 7392);
    REQUIRE("right PI mirrored", follow_incremental_pi_state.target_left_millirps == 7392);
    running(100);
    car_control_state.gyro.phase = CAR_GYRO_RING_EXIT_STRAIGHT;
    prepare_follow_motor_targets(100);
    REQUIRE("center straight equal wheel targets", ackermann_control_output.left_target_millirps == 7392 && ackermann_control_output.right_target_millirps == 7392);

    running(100);
    car_control_state.gyro.phase = CAR_GYRO_RING_SEARCH;
    car_control_state.gyro.hold_ramp = 1;
    applied_motor_follow_left_permille = 650;
    applied_motor_follow_right_permille = 650;
    prepare_follow_motor_targets(100);
    REQUIRE("search freezes actual duties", follow_output_state.command_left_permille == 650 && follow_output_state.command_right_permille == 650);
    REQUIRE("search keeps PI paused", follow_incremental_pi_state.status == CAR_FOLLOW_SPEED_PI_STATUS_HOLD);
    uint32_t generation = follow_output_state.hold_entry_generation;
    /* Later actual outputs and fresh camera callbacks cannot restart ramp. */
    car_control_state.gyro.search_elapsed_ms = 150;
    speed_observer_result.sample_ms = 250;
    applied_motor_follow_left_permille = 500;
    applied_motor_follow_right_permille = 500;
    prepare_follow_motor_targets(250);
    REQUIRE("search midpoint", follow_output_state.command_left_permille == 525);
    REQUIRE("search fixed generation", follow_output_state.hold_entry_generation == generation && follow_output_state.hold_started_ms == 100);
    car_control_state.gyro.search_elapsed_ms = 300;
    speed_observer_result.sample_ms = 400;
    prepare_follow_motor_targets(400);
    REQUIRE("search original 300ms ramp target", follow_output_state.command_left_permille == 400 && follow_output_state.command_right_permille == 400);
    car_control_state.gyro.search_elapsed_ms = 2999;
    speed_observer_result.sample_ms = 3099;
    prepare_follow_motor_targets(3099);
    REQUIRE("search holds reduced duty", follow_output_state.command_left_permille == 400 && follow_output_state.hold_started_ms == 100);
    car_control_state.gyro.hold_ramp = 0;
    car_control_state.gyro.phase = CAR_GYRO_RING_WAIT_ROUTE;
    car_control_state.road_drive_allowed = 0;
    prepare_follow_motor_targets(3100);
    REQUIRE("deadline removes outputs", follow_output_state.command_left_permille == 0 && follow_output_state.command_right_permille == 0);
    REQUIRE("deadline disarms PI", follow_incremental_pi_state.armed == 0);
    running(100);
    car_control_state.gyro.hold_ramp = 1;
    speed_observer_result.valid = 0;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_INVALID_CONFIG;
    prepare_follow_motor_targets(100);
    REQUIRE("encoder fault stops search", test_emergency_stops == 1 && test_last_fault == CAR_FAULT_ENCODER_INIT);
    /* Exact last-moment production hardware guard; authority callback itself
       is exercised against the full controller in the integration suite. */
    running(100);
    output_generation_snapshot_t snapshot = {0};
    app_config.motor_hardware_accepted = 1;
    app_config.follow_mode_compiled = 1;
    app_config.follow_calibration_ready = 1;
    app_config.follow_positive_error_right_measured = 1;
    manual_run_button.run_request = 1;
    test_authority = 1;
    REQUIRE("gyro authority reaches hardware guard despite invalid route", motor_output_guard(1, 0, &snapshot));
    test_authority = 0;
    REQUIRE("missing authority revokes motor", !motor_output_guard(1, 0, &snapshot));
    test_authority = 1;
    manual_run_button.stop_pending = 1;
    REQUIRE("PE7 pending stop overrides gyro", !motor_output_guard(1, 0, &snapshot));
    manual_run_button.stop_pending = 0;
    uart_regs.ISR = USART_ISR_ORE;
    REQUIRE("UART fault overrides gyro", !motor_output_guard(1, 0, &snapshot));
    uart_regs.ISR = 0;
    car_vision_rx.fault_generation = 1;
    REQUIRE("new parser fault overrides gyro", !motor_output_guard(1, 0, &snapshot));
    car_vision_rx.fault_generation = 0;
    car_control_state.isr_stop_request = 1;
    REQUIRE("ISR stop overrides gyro", !motor_output_guard(1, 0, &snapshot));
    printf("GYRO350_MAIN assertions=%u failures=%u\n", checks, failures);
    return failures ? 1 : 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zig", type=Path, required=True)
    args = parser.parse_args()
    source = ROOT / "Src/main.c"
    text = source.read_text(encoding="utf-8-sig")
    prelude = helper.PRELUDE.replace("(follow_speed_pi_state.", "(follow_incremental_pi_state.")
    setup = helper.HARNESS.split("int main(void)")[0]
    functions = "\n".join(helper.extract_function(text, name) for name in
        ("consume_follow_speed_pi_reset_request", "prepare_follow_motor_targets"))
    spec2 = importlib.util.spec_from_file_location("output_path", ROOT / "tests/host/test_follow_output_path.py")
    output_helper = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(output_helper)
    functions += "\n" + output_helper.extract_function(text, "motor_output_guard")[0]
    flags = {"CARTEST_HARDWARE_ACCEPTED": 1, "CARTEST_GYRO350_ENABLED": 1, "CARTEST_FOLLOW_SPEED_PI_ENABLED": 1,
             "CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED": 1, "CARTEST_ACKERMANN_COORDINATION_ENABLED": 1,
             "CARTEST_FOLLOW_TURN_PROFILE_COMPILED": 1, "CARTEST_TRACK_PROFILE": 3,
             "CARTEST_SPEED_PERCENT": 140, "CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS": 5600,
             "CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS": 4400, "CARTEST_VISION_HOLD_RAMP_MS": 300}
    out = ROOT.parent / "validation"; out.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="cartest-gyro-main-") as tmp:
        c = Path(tmp) / "main.c"; exe = Path(tmp) / "test.exe"
        c.write_text(prelude + EXTRA + functions + setup + HARNESS, encoding="ascii")
        units = ["follow_output.c", "speed_pi.c", "follow_speed_pi.c", "follow_incremental_pi.c", "ackermann_control.c"]
        built = subprocess.run([str(args.zig), "cc", "-std=c11", "-I", str(ROOT / "Inc"),
            *[f"-D{k}={v}" for k, v in flags.items()], str(c),
            *[str(ROOT / "App" / name) for name in units], "-o", str(exe)], capture_output=True, text=True)
        if built.returncode:
            raise RuntimeError(built.stdout + built.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        (out / "gyro-main.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout)
        if result.returncode:
            raise RuntimeError("Production gyro target harness failed")
        (out / "GYRO_MAIN.json").write_text(json.dumps({"passed": True,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "summary": result.stdout.strip(), "hardware": "not_connected"}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
