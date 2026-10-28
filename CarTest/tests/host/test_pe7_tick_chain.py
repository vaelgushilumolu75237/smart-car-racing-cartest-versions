"""Execute extracted production SysTick/EXTI-to-TIM3 stop code on fake registers."""
from pathlib import Path
import argparse
import subprocess
import tempfile


def function(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
        end += 1
    return source[start:end]


PREFIX = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "manual_run_button.h"
#include "commissioning.h"
#include "follow_output.h"
typedef struct { uint32_t CR1, CCER, CCR1, CCR2, CCR3, CCR4; } Timer;
typedef struct { uint32_t APB1LENR; } Clocks;
typedef struct { uint32_t PR1; } Exti;
static Timer timer; static Clocks clocks; static Exti exti;
#define TIM3 (&timer)
#define RCC (&clocks)
#define EXTI (&exti)
#define RCC_APB1LENR_TIM3EN 1u
#define TIM_CR1_CEN 1u
#define APP_MOTOR_CHANNEL_MASK 0x1111u
#define APP_SWITCH_GPIO_PIN 0x80u
#define APP_MAIN_LOOP_GAP_LIMIT_MS 200u
static car_config_t app_config;
static car_control_t car_control_state;
static car_manual_button_t manual_run_button;
static car_follow_output_state_t follow_output_state;
static uint32_t now, mask, raw_high=1u, gpio_low, servo_stops, faults;
static volatile uint8_t manual_button_gpio_ready;
static uint8_t manual_button_have_sample;
static uint32_t car_pe7_raw_high, car_button_state, car_manual_run_request;
static uint32_t car_button_press_event_count, car_button_start_event_count;
static uint32_t car_button_stop_generation, car_button_armed;
static uint32_t car_pe7_sample_count, car_pe7_last_sample_ms, car_pe7_sample_gap_max_ms;
static uint32_t car_main_loop_heartbeat_ms;
static uint8_t motor_outputs_active, applied_follow_outputs, servo_output_active;
static uint8_t follow_ramp_started, follow_speed_pi_reset_pending;
static uint16_t applied_motor_duty_permille, applied_motor_follow_left_permille;
static uint16_t applied_motor_follow_right_permille;
static uint8_t applied_motor_channel_mask;
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __set_PRIMASK(uint32_t value) { mask=value; }
static void __DSB(void) {}
static void HAL_IncTick(void) { ++now; }
static uint32_t HAL_GetTick(void) { return now; }
static void HAL_SYSTICK_IRQHandler(void) {}
static uint8_t pe7_is_high(void) { return (uint8_t)raw_high; }
static void set_motor_gpio_low(void) { gpio_low=1; }
static void stop_servo_outputs_hardware(void) { ++servo_stops; }
static void protected_fast_stop_note(car_fault_t reason) { faults=(uint32_t)reason; }
static void protected_switch_release_note(void) { faults=CAR_FAULT_SWITCH_RELEASE; }
'''

CHECKS = r'''
static unsigned failures, checks;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
static void tick(uint8_t level, unsigned count) {
    raw_high=level;
    while(count--) { car_main_loop_heartbeat_ms=now; SysTick_Handler(); }
}
static void pretend_applied(void) {
    car_control_state.follow_session_active=1;
    car_control_state.mode=CAR_MODE_FOLLOW_RUNNING;
    timer.CR1=TIM_CR1_CEN; timer.CCER=APP_MOTOR_CHANNEL_MASK;
    timer.CCR1=timer.CCR2=timer.CCR3=timer.CCR4=123;
    motor_outputs_active=applied_follow_outputs=1;
    applied_motor_duty_permille=applied_motor_follow_left_permille=123;
    applied_motor_follow_right_permille=123; gpio_low=0;
}
static void stopped(void) {
    CHECK((timer.CR1 & TIM_CR1_CEN)==0);
    CHECK((timer.CCER & APP_MOTOR_CHANNEL_MASK)==0);
    CHECK(timer.CCR1==0 && timer.CCR2==0 && timer.CCR3==0 && timer.CCR4==0);
    CHECK(gpio_low==1 && motor_outputs_active==0 && applied_follow_outputs==0);
    CHECK(applied_motor_duty_permille==0 && applied_motor_follow_left_permille==0 && applied_motor_follow_right_permille==0);
    CHECK(manual_run_button.run_request==0 && manual_run_button.start_pending==0);
}
int main(void) {
    app_config.follow_mode_compiled=1;
    car_manual_button_init(&manual_run_button,1,0);
    tick(0,50); CHECK(car_pe7_sample_count==0); /* No uninitialized GPIO reads. */
    now=0; manual_button_gpio_ready=1;
    tick(1,19); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_HIGH);
    tick(1,1); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_LOW);
    /* No falling EXTI call is made: periodic sampling alone must start. */
    tick(0,20); CHECK(manual_run_button.run_request==0);
    tick(0,1); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_RUNNING);
    CHECK(manual_run_button.start_event_count==1 && manual_run_button.start_pending==1);
    tick(0,40); CHECK(manual_run_button.start_event_count==1);
    car_manual_button_start_consumed(&manual_run_button); pretend_applied();
    raw_high=1; exti.PR1=APP_SWITCH_GPIO_PIN; EXTI9_5_IRQHandler();
    stopped(); CHECK(manual_run_button.stop_generation==1);
    CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_HIGH && servo_stops>0);
    /* Stop bounce and a low hold cannot restart before a new high dwell. */
    car_manual_button_stop_consumed(&manual_run_button);
    tick(0,30); CHECK(manual_run_button.run_request==0);
    tick(1,20); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_HIGH);
    tick(1,1); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_LOW);
    tick(0,21); CHECK(manual_run_button.start_event_count==2);
    pretend_applied();
    /* A rising EXTI is not required for the 1ms high-stop fallback. */
    tick(1,1); stopped(); CHECK(manual_run_button.stop_generation==2);
    car_manual_button_stop_consumed(&manual_run_button);
    tick(1,20); tick(0,21); CHECK(manual_run_button.run_request==1);
    pretend_applied(); car_control_state.fault_latched=1;
    tick(0,1); stopped(); CHECK(manual_run_button.state==CAR_MANUAL_BUTTON_WAIT_HIGH);
    car_control_state.fault_latched=0; tick(0,100);
    CHECK(manual_run_button.run_request==0);
    CHECK(car_pe7_sample_gap_max_ms==1 && car_pe7_sample_count>100);
    CHECK(mask==0);
    printf("PE7_CHAIN_TEST checks=%u failures=%u (fake registers; no hardware)\n",checks,failures);
    return failures ? 1 : 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", required=True)
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source = (root / "Src/main.c").read_text(encoding="utf-8")
    markers = (
        "static void manual_button_publish_live_locked(uint8_t raw_high)\n{",
        "static void stop_motor_outputs_hardware(void)\n{",
        "static void emergency_stop_hardware(void)\n{",
        "static car_manual_button_event_t manual_button_tick_sample(uint32_t now_ms)\n{",
        "static car_manual_button_event_t manual_button_stop_on_high_sample(\n"
        "    uint8_t raw_high,\n    uint32_t now_ms)\n{",
        "void SysTick_Handler(void)\n{", "void EXTI9_5_IRQHandler(void)\n{",
    )
    with tempfile.TemporaryDirectory(prefix="cartest-pe7-chain-") as temp:
        out = Path(args.output_dir or temp).resolve()
        out.mkdir(parents=True, exist_ok=True)
        c_file, executable = out / "pe7_chain.c", out / "pe7_chain.exe"
        c_file.write_text(PREFIX + "\n".join(function(source, m) for m in markers) + CHECKS,
                          encoding="utf-8")
        command = [str(Path(args.zig).resolve()), "cc", "-std=c11", "-I", str(root / "Inc"),
                   str(c_file), str(root / "App/manual_run_button.c"),
                   str(root / "App/follow_output.c"), "-o", str(executable)]
        result = subprocess.run(command, capture_output=True, text=True)
        (out / "compile.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            print(result.stderr)
            return result.returncode
        result = subprocess.run([str(executable)], capture_output=True, text=True)
        (out / "run.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout, end="")
        return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
