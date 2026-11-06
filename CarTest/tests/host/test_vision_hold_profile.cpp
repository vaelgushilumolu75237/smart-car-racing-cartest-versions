#include "car_calibration.h"
#include "commissioning.h"
#include "follow_output.h"
#include "vision_protocol.h"

#include <cstdint>
#include <cstdio>

#if CARTEST_VISION_HOLD_MS != 2000u || CARTEST_VISION_HOLD_RAMP_MS != 200u
#error "This host fixture must compile with the B/C hold profile values."
#endif

namespace {

unsigned g_failures;

void check(bool condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        std::printf("FAIL %s\n", message);
    }
}

car_inputs_t inputs_at(uint32_t now_ms)
{
    car_inputs_t inputs{};
    inputs.now_ms = now_ms;
    inputs.pe7_raw_high = 0u;
    inputs.pe7_low_stable = 1u;
    inputs.manual_run_request = 1u;
    return inputs;
}

void send_frame(car_control_t *control, const car_config_t *config,
                uint16_t sequence, uint8_t flags, uint8_t confidence,
                int16_t error, uint32_t time_ms, uint8_t pe7_raw_high)
{
    car_control_on_vision_frame_at_with_flags(
        control, config, sequence, flags, confidence, error, time_ms,
        time_ms, pe7_raw_high);
}

void enter_hold(car_control_t *control, const car_config_t *config,
                uint32_t start_ms)
{
    car_control_init(control, config, start_ms, 1u, 0u);

    car_inputs_t released{};
    released.now_ms = start_ms;
    released.pe7_raw_high = 1u;
    released.pe7_high_stable = 1u;
    released.manual_button_startup_release_confirmed = 1u;
    car_control_tick(control, config, &released);
    if (control->rx_recovery_flush_required != 0u) {
        car_control_rx_recovery_flush_complete(control);
    }
    car_control_reverse_control_keep_started(control);
    car_control_tick(control, config, &released);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t frame_ms = start_ms + 1u + offset;
        send_frame(control, config, static_cast<uint16_t>(100u + offset),
                   VP_FLAG_VALID, 80u, 0, frame_ms, 1u);
    }

    car_inputs_t press_edge{};
    press_edge.now_ms = start_ms + 4u;
    press_edge.pe7_raw_high = 0u;
    press_edge.pe7_high_stable = 1u;
    car_control_tick(control, config, &press_edge);
    car_inputs_t press_stable = inputs_at(start_ms + 24u);
    press_stable.pe7_high_stable = 0u;
    car_control_tick(control, config, &press_stable);

    send_frame(control, config, 103u, VP_FLAG_VALID, 80u, 545,
               start_ms + 27u, 0u);
    car_control_note_applied_outputs(control, 1u, 1u, 1600u, 120u, 180u);
    send_frame(control, config, 104u, 0u, 0u, 0,
               start_ms + 28u, 0u);
}

} // namespace

int main()
{
    car_config_t config{};
    car_control_t control{};
    car_config_init_application(&config, 0u, 0u, 1u, 1u);
    config.follow_calibration_ready = 1u;
    config.follow_positive_error_right_measured = 1u;
    config.servo_limits_measured = 1u;
    config.servo_min_us = CARTEST_SERVO_LEFT_US < CARTEST_SERVO_RIGHT_US
        ? CARTEST_SERVO_LEFT_US : CARTEST_SERVO_RIGHT_US;
    config.servo_max_us = CARTEST_SERVO_LEFT_US > CARTEST_SERVO_RIGHT_US
        ? CARTEST_SERVO_LEFT_US : CARTEST_SERVO_RIGHT_US;
    config.follow_servo_left_us = CARTEST_SERVO_LEFT_US;
    config.follow_servo_right_us = CARTEST_SERVO_RIGHT_US;
    config.follow_left_encoder_counts_per_rev = 59995u;
    config.follow_right_encoder_counts_per_rev = 60239u;
    config.follow_left_encoder_sign = 1;
    config.follow_right_encoder_sign = -1;

    constexpr uint32_t start_ms = 1000u;
    enter_hold(&control, &config, start_ms);
    check(control.vision_hold_active != 0u,
          "profile fixture enters Follow vision hold");
    const uint32_t last_valid_ms = control.vision_last_valid_ms;
    const uint16_t held_target =
        car_control_follow_servo_output_target_us(&control);
    check(control.vision_hold_error_norm == 545,
          "exit line loss holds the last trusted signed error");
    uint16_t sequence = 105u;
    for (uint32_t elapsed_ms = 200u; elapsed_ms <= 1800u;
         elapsed_ms += 200u) {
        const uint32_t frame_ms = last_valid_ms + elapsed_ms;
        send_frame(&control, &config, sequence++, 0u, 0u, 0,
                   frame_ms, 0u);
        check(control.vision_hold_error_norm == 545 &&
                  car_control_follow_servo_output_target_us(&control) == held_target &&
                  control.vision_last_valid_ms == last_valid_ms,
              "continued exit invalid frames neither centre steering nor renew deadline");
    }

    car_inputs_t before_deadline = inputs_at(last_valid_ms + 1999u);
    car_control_tick(&control, &config, &before_deadline);
    check(control.vision_hold_active != 0u &&
              control.vision_hold_expired == 0u,
          "1999 ms after last valid route remains in active hold");
    car_inputs_t exact_deadline = inputs_at(last_valid_ms + 2000u);
    car_control_tick(&control, &config, &exact_deadline);
    check(control.vision_hold_active == 0u &&
              control.vision_hold_expired != 0u,
          "2000 ms after last valid route expires hold exactly");
    check(control.vision_hold_restart_required != 0u &&
              control.follow_output_duty_permille == 0u,
          "profile deadline requests restart from stopped output");

    car_follow_output_state_t output{};
    car_follow_output_hold_at(&output, 0u, 0u, 1u, 600u, 500u);
    car_follow_output_hold_at(&output, 199u, 0u, 1u, 0u, 0u);
    check(output.command_left_permille == 401u &&
              output.command_right_permille == 334u,
          "200 ms deceleration is still one permille short at 199 ms");
    car_follow_output_hold_at(&output, 200u, 0u, 1u, 0u, 0u);
    check(output.command_left_permille == 400u &&
              output.command_right_permille == 333u,
          "200 ms deceleration reaches both held targets at 200 ms");

    std::printf("VISION_HOLD_PROFILE_SUMMARY failures=%u hold_ms=%u ramp_ms=%u\n",
                g_failures, CARTEST_VISION_HOLD_MS,
                CAR_FOLLOW_VISION_HOLD_RAMP_MS);
    return g_failures == 0u ? 0 : 1;
}
