"""Exercise the real main.c startup/output functions with mocked registers."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("output_path", ROOT / "tests/host/test_follow_output_path.py")
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)

CASES = r'''
static void straight_fixture(void)
{
    reset_mock(0u);
    gyro_hold_seen = 0u;
    start_existing_servo(app_config.servo_center_us);
    car_control_state.follow_output_duty_permille = 0u;
    speed_observer_result.valid = 1u;
    speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    speed_observer_result.elapsed_ms = 10u;
}
int main(void)
{
    straight_fixture();
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("straight", "first left command", EXPECT_START, follow_output_state.command_left_permille);
    expect_u32("straight", "first right command", EXPECT_START, follow_output_state.command_right_permille);
    expect_u32("straight", "wheel target", EXPECT_TARGET, ackermann_control_output.fastest_target_millirps);
#if CARTEST_RING_TUNE == 4
    expect_u32("straight", "cannot claim applied qualification", 0u, follow_output_state.startup_qualified);
    motor_apply();
    expect_u32("straight", "existing first-frame output boundary", 0u, motor_outputs_active);
    ++mock_now_ms; speed_observer_result.sample_ms = mock_now_ms;
    prepare_follow_motor_targets(mock_now_ms); motor_apply();
    expect_u32("straight", "left PWM applied after boundary", 500u, TIM3->CCR2);
    expect_u32("straight", "right PWM applied after boundary", 500u, TIM3->CCR3);
    ++mock_now_ms; speed_observer_result.sample_ms = mock_now_ms;
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("straight", "qualification uses actual applied PWM", 1u, follow_output_state.startup_qualified);
    mock_now_ms += 10u; speed_observer_result.sample_ms = mock_now_ms;
    prepare_follow_motor_targets(mock_now_ms);
    expect_u32("straight", "encoder PI armed", 1u, follow_incremental_pi_state.armed);
    expect_u32("straight", "PI target", 7600u, follow_incremental_pi_state.target_left_millirps);
    car_control_state.isr_stop_request = 1u; motor_apply();
    expect_u32("stop", "motor disabled immediately", 0u, motor_outputs_active);
    expect_u32("stop", "PWM cleared", 0u, TIM3->CCR2);
#endif
    for (unsigned gate = 0u; gate < 10u; ++gate) {
        straight_fixture();
        switch (gate) {
        case 0: servo_safety.frame_ready = 0u; break;
        case 1: speed_observer_result.valid = 0u; break;
        case 2: speed_observer_result.sample_ms -= 101u; break;
        case 3: car_control_state.vision_curve_phase = CAR_ACKERMANN_PHASE_TURN; break;
        case 4: car_control_state.vision_slowdown_request_permille = 1u; break;
        case 5: car_control_state.gyro.active = 1u; car_control_state.gyro.outer_target_millirps = 7600u; break;
        case 6: car_control_state.obstacle.active = 1u; car_control_state.obstacle.outer_target_millirps = 4400u; break;
        case 7: car_control_state.manual_run_request = 0u; break;
        case 8: car_control_state.fault_latched = 1u; break;
        case 9: speed_observer_result.status = CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE; break;
        }
        prepare_follow_motor_targets(mock_now_ms);
        expect_u32("gated", "no direct launch", 0u, follow_output_state.command_left_permille);
    }
    printf("DIRECT_STRAIGHT_START tune=%d checks=%lu failures=%lu\n", CARTEST_RING_TUNE,
           (unsigned long)checks, (unsigned long)failures);
    return failures ? 1 : 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zig", required=True)
    args = parser.parse_args()
    prelude = host.PRELUDE.replace("} car_control_state_t;", """
    struct { uint8_t active, hold_ramp, fault; uint32_t search_elapsed_ms;
             uint16_t outer_target_millirps; } gyro;
    struct { uint8_t active, error; uint16_t outer_target_millirps; } obstacle;
    uint16_t gyro_servo_us, obstacle_servo_us;
} car_control_state_t;""").replace("CARTEST_HOST_HOLD_DEADLINE_MS", "3000u")
    prelude += """
static uint8_t gyro_hold_seen;
static uint16_t gyro_hold_left, gyro_hold_right;
static uint32_t gyro_hold_started_ms, gyro_hold_generation;
static uint8_t car_control_follow_authority_ready(const car_control_state_t *s,
    const app_config_t *c, uint32_t now) {
    return car_control_vision_fresh(s, now, c->vision_timeout_ms);
}
"""
    source = (ROOT / "Src/main.c").read_text(encoding="utf-8-sig")
    selector = host.extract_function((ROOT / "App/commissioning.c").read_text(encoding="utf-8-sig"),
                                    "car_control_follow_servo_output_target_us")[0]
    functions = selector + "\n" + "\n".join(host.extract_function(source, n)[0] for n in host.FUNCTION_NAMES)
    support = host.SUPPORT[:host.SUPPORT.index("int main(void)")]
    text = prelude + functions + support + CASES
    with tempfile.TemporaryDirectory(prefix="CarTestStraightStart-") as directory:
        temp = Path(directory)
        harness = temp / "harness.c"
        harness.write_text(text, encoding="ascii")
        for tune, target, first in ((0, 5600, 0), (3, 5600, 0), (4, 7600, 500)):
            exe = temp / f"test{tune}.exe"
            defines = dict(CARTEST_RING_TUNE=tune, CARTEST_TRACK_PROFILE=3,
                CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600,
                CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=5100 if tune == 4 else 4400,
                CARTEST_FOLLOW_SPEED_PI_ENABLED=1, CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED=1,
                CARTEST_ACKERMANN_COORDINATION_ENABLED=1, CARTEST_GYRO350_ENABLED=1,
                CARTEST_RED_OBSTACLE_ENABLED=1, CARTEST_FOLLOW_TURN_PROFILE_COMPILED=1,
                CARTEST_FOLLOW_SERVO_SLEW_STEP_US=160, CARTEST_FOLLOW_DIFF_PERMILLE=150,
                CARTEST_FOLLOW_INNER_DIFF_PERMILLE=150, EXPECT_START=first, EXPECT_TARGET=target)
            subprocess.run([args.zig, "cc", "-std=c11", "-Wno-unused-function", "-I", str(ROOT / "Inc"),
                *[f"-D{k}={v}" for k, v in defines.items()], str(harness),
                *[str(ROOT / "App" / (name + ".c")) for name in
                  ("follow_output", "speed_pi", "follow_speed_pi", "follow_incremental_pi", "ackermann_control")],
                "-o", str(exe)], check=True, cwd=ROOT)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
