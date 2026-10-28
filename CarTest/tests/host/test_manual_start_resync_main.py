"""Check production PE7 1 ms sampling, EXTI stop, and guarded motor handoff."""

from __future__ import annotations

import argparse
from pathlib import Path


def block_after(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise ValueError(f"unterminated block after {marker!r}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="Src/main.c")
    args = parser.parse_args()
    source = Path(args.source).read_text(encoding="utf-8")

    main_block = block_after(source, "int main(void)\n{")
    init_button = main_block.index("car_manual_button_init(&manual_run_button")
    ready_flag = main_block.index("manual_button_gpio_ready = 1u;")
    loop = main_block.index("for (;;)")
    assert init_button < ready_flag < loop

    tick_sample = block_after(
        source,
        "static car_manual_button_event_t manual_button_tick_sample(uint32_t now_ms)\n{",
    )
    tick_lock = tick_sample.index("__disable_irq();")
    tick_call = tick_sample.index("car_manual_button_tick_1ms(")
    tick_unlock = tick_sample.index("__set_PRIMASK(primask);")
    tick_emergency = tick_sample.index("emergency_stop_hardware();")
    assert tick_lock < tick_call < tick_unlock < tick_emergency
    assert "car_control_state.fault_latched" in tick_sample[tick_call:tick_unlock]
    assert "CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT" in tick_sample

    systick = block_after(source, "void SysTick_Handler(void)\n{")
    inc_tick = systick.index("HAL_IncTick();")
    ready_check = systick.index("manual_button_gpio_ready != 0u")
    sample_call = systick.index("manual_button_tick_sample(now_ms);")
    assert inc_tick < ready_check < sample_call
    assert "app_config.follow_mode_compiled != 0u" in systick

    switch_init = block_after(
        source,
        "static HAL_StatusTypeDef switch_init(void)\n{",
    )
    assert "gpio.Mode = GPIO_MODE_IT_RISING;" in switch_init
    assert "GPIO_MODE_IT_RISING_FALLING" not in switch_init

    stop_sample = block_after(
        source,
        "static car_manual_button_event_t manual_button_stop_on_high_sample(\n"
        "    uint8_t raw_high,\n"
        "    uint32_t now_ms)\n{",
    )
    stop_call = stop_sample.index("car_manual_button_stop_on_high(")
    stop_unlock = stop_sample.index("__set_PRIMASK(primask);")
    assert stop_sample.index("__disable_irq();") < stop_call < stop_unlock
    assert stop_sample.index("if (event == CAR_MANUAL_BUTTON_EVENT_STOP)") < stop_sample.index(
        "emergency_stop_hardware();"
    )

    exti = block_after(source, "void EXTI9_5_IRQHandler(void)\n{")
    assert "manual_button_stop_on_high_sample(" in exti
    assert "manual_button_tick_sample(" not in exti
    assert "car_manual_button_tick_1ms(" not in exti
    assert "start_pending" not in exti

    start_event = block_after(
        source,
        "if (app_config.follow_mode_compiled != 0u &&\n"
        "            manual_run_button.start_pending != 0u)",
    )
    assert "follow_start_event = 1u;" in start_event
    assert "car_manual_button_start_consumed(&manual_run_button);" in start_event

    resync = block_after(
        source,
        "if (follow_start_event != 0u &&\n"
        "            app_config.follow_mode_compiled != 0u)",
    )
    disable = resync.index("__disable_irq();")
    rebuild = resync.index("inputs = make_inputs(")
    state_check = resync.index(
        "manual_run_button.state == CAR_MANUAL_BUTTON_RUNNING"
    )
    run_check = resync.index("manual_run_button.run_request != 0u")
    stop_check = resync.index("manual_run_button.stop_pending == 0u")
    pending_check = resync.index("manual_run_button.start_pending == 0u")
    begin = resync.index("car_control_begin_manual_follow_start(")
    restore = resync.index("__set_PRIMASK(start_event_primask);")
    emergency = resync.index("emergency_stop_hardware();")
    assert disable < rebuild < state_check < run_check < stop_check
    assert stop_check < pending_check < begin < restore < emergency

    guard = block_after(
        source,
        "static uint8_t motor_output_guard(\n"
        "    uint8_t follow_mode,\n"
        "    uint8_t expected_bench_channel_mask,\n"
        "    const output_generation_snapshot_t *snapshot)\n{",
    )
    assert "manual_run_button.state != CAR_MANUAL_BUTTON_RUNNING" in guard
    assert "manual_run_button.run_request == 0u" in guard
    assert "manual_run_button.stop_pending != 0u" in guard
    for safety_gate in (
        "car_control_state.fault_latched == 0u",
        "car_control_state.road_drive_allowed == 0u",
        "app_config.follow_calibration_ready == 0u",
        "car_control_state.manual_run_request == 0u",
        "car_control_vision_fresh(",
        "servo_stop_pending == 0u",
    ):
        assert safety_gate in guard

    stop_motor = block_after(
        source,
        "static void stop_motor_outputs_hardware(void)\n{",
    )
    for stop_action in (
        "TIM3->CCER &= ~APP_MOTOR_CHANNEL_MASK;",
        "TIM3->CCR1 = 0u;",
        "TIM3->CCR2 = 0u;",
        "TIM3->CCR3 = 0u;",
        "TIM3->CCR4 = 0u;",
        "TIM3->CR1 &= ~TIM_CR1_CEN;",
        "set_motor_gpio_low();",
    ):
        assert stop_action in stop_motor
    emergency_stop = block_after(
        source,
        "static void emergency_stop_hardware(void)\n{",
    )
    assert emergency_stop.index("stop_motor_outputs_hardware();") < emergency_stop.index(
        "stop_servo_outputs_hardware();"
    )

    motor_snapshot = block_after(
        source,
        "static void publish_motor_channel_debug_snapshot(void)\n{",
    )
    for register_snapshot in (
        "car_motor_pwm_ccr2 = TIM3->CCR2;",
        "car_motor_pwm_ccr3 = TIM3->CCR3;",
        "car_motor_pwm_ccer = TIM3->CCER;",
        "(TIM3->CR1 & TIM_CR1_CEN)",
    ):
        assert register_snapshot in motor_snapshot

    for live_watch in (
        "volatile uint32_t car_button_state;",
        "volatile uint32_t car_pe7_sample_count;",
        "volatile uint32_t car_pe7_last_sample_ms;",
        "volatile uint32_t car_pe7_sample_gap_max_ms;",
        "car_button_state = (uint32_t)manual_run_button.state;",
        "++car_pe7_sample_count;",
    ):
        assert live_watch in source

    obsolete_api_parts = (
        ("manual_button_", "sample("),
        ("car_manual_button_", "poll_high("),
        ("car_manual_button_", "note_low_edge("),
    )
    for obsolete_api in ("".join(parts) for parts in obsolete_api_parts):
        assert obsolete_api not in source

    print("PASS PE7 SysTick FSM, rising EXTI stop, deferred-start guard, and TIM3 stop wiring")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
