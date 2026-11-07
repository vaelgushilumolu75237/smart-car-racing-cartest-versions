from __future__ import annotations
import hashlib
import json
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'Src' / 'main.c'
ZIG = Path(r'D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe')
EVIDENCE = ROOT.parent / 'validation' / 'WIRELESS_MAIN.json'

def extract_function(source: str, name: str) -> str:
    pattern = r'static\s+[^\n{;]+\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{'
    match = re.search(pattern, source)
    if match is None:
        raise AssertionError(f'missing production function: {name}')
    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise AssertionError(f'unterminated production function: {name}')

def extract_systick_wireless_guard(source: str) -> str:
    start_marker = '    if ((wireless_test_owned || wireless_return_center) && (motor_outputs_active || servo_output_active)) {'
    end_marker = '    follow_active = app_config.follow_mode_compiled != 0u &&'
    start = source.find(start_marker)
    end = source.find(end_marker, start)
    if start < 0 or end < 0:
        raise AssertionError('missing production SysTick wireless guard')
    block = source[start:end]
    end_if = block.rfind('#endif')
    if end_if >= 0:
        block = block[:end_if]
    return block

PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CARTEST_HARDWARE_ACCEPTED 1
#define CARTEST_SERVO_CENTER_US 1530u
#define CAR_SERVO_SOURCE_DEBUG_STEERING 2u
#define CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US 160u
#define APP_SWITCH_GPIO_PIN 128u
#define APP_MAIN_LOOP_GAP_LIMIT_MS 50u
#define CAR_FAULT_LOOP_GAP 77
#define WIRELESS_TEST_MOTOR 1u
#define WIRELESS_TEST_SERVO 2u
#define TIM_CR1_CEN 1u
#define TIM_CCER_CC2E 4u
#define TIM_CCER_CC3E 16u
#define USART_ISR_PE 1u
#define USART_ISR_FE 2u
#define USART_ISR_NE 4u
#define USART_ISR_ORE 8u
#define CAR_FAULT_NONE 0
typedef struct { uint8_t active,motor; uint16_t duty_permille,servo_us; uint32_t started_ms,deadline_ms; uint8_t kind; } wireless_control_test_t;
typedef struct { uint32_t control_fault,control_rx_fault,control_isr_switch,control_isr_uart,rx_parser_fault,rx_isr_fault; } output_generation_snapshot_t;
typedef struct { uint8_t startup_release_confirmed,run_request,start_pending,stop_pending; } car_manual_button_t;
typedef struct { uint8_t follow_session_active,fault_latched,rx_recovery_flush_required; int fault; uint8_t imu_fault,isr_stop_request,isr_release_seen; int isr_stop_fault; uint32_t fault_generation,rx_fault_generation,isr_switch_generation,isr_uart_generation,vision_last_frame_alive_ms; } car_control_t;
typedef struct { uint8_t motor_hardware_accepted; uint16_t vision_timeout_ms; } app_config_t;
typedef struct { uint32_t fault_generation,isr_fault_generation; uint8_t uart_fault,overflow_fault,protocol_fault,stale_fault; } car_vision_rx_t;
typedef struct { uint8_t fault_latched,active,frame_ready; uint16_t compare_us; } car_servo_safety_t;
typedef struct { uint32_t CR1,CCER,CNT,CCR1,CCR2,CCR3,CCR4; } tim_regs_t;
typedef struct { uint32_t ISR; } usart_regs_t;
typedef struct { uint32_t PR1; } exti_regs_t;
typedef struct { uint32_t marker; } car_servo_safety_io_t;
'''

PRELUDE += r'''
static car_control_t car_control_state; static app_config_t app_config;
static car_vision_rx_t car_vision_rx; static car_manual_button_t manual_run_button;
static car_servo_safety_t servo_safety; static car_servo_safety_io_t servo_safety_io;
static wireless_control_test_t wireless_test_state; static tim_regs_t tim3_regs;
static usart_regs_t usart1_regs; static exti_regs_t exti_regs;
static uint32_t test_now,primask_state,car_main_loop_heartbeat_ms;
static uint8_t pe7_high,servo_stop_pending,servo_output_active,motor_outputs_active,applied_follow_outputs;
static uint16_t applied_servo_pulse_us,applied_motor_duty_permille;
static uint16_t applied_motor_follow_left_permille,applied_motor_follow_right_permille;
static uint8_t applied_motor_channel_mask,wireless_test_owned,wireless_return_center;
static uint32_t wireless_return_started_ms,local_stop_calls,emergency_stop_calls;
static uint32_t motor_stop_calls,servo_stop_calls,last_af_channels;
static uint8_t servo_request_fail;
static uint32_t protected_fast_stop_calls,protected_fast_stop_fault;
#define TIM3 (&tim3_regs)
#define USART1 (&usart1_regs)
#define EXTI (&exti_regs)
static uint32_t HAL_GetTick(void) { return test_now; }
static void __DSB(void) {}
static uint32_t __get_PRIMASK(void) { return primask_state; }
static void __disable_irq(void) { primask_state=1u; }
static void __set_PRIMASK(uint32_t value) { primask_state=value; }
static uint8_t pe7_is_high(void) { return pe7_high; }
static wireless_control_test_t car_wireless_test(void) { return wireless_test_state; }
static uint8_t car_wireless_test_allowed(uint32_t now) {
  return wireless_test_state.active && (uint32_t)(now-wireless_test_state.started_ms)<5000u;
}
static void output_generation_snapshot(output_generation_snapshot_t *s) {
  s->control_fault=car_control_state.fault_generation;
  s->control_rx_fault=car_control_state.rx_fault_generation;
  s->control_isr_switch=car_control_state.isr_switch_generation;
  s->control_isr_uart=car_control_state.isr_uart_generation;
  s->rx_parser_fault=car_vision_rx.fault_generation;
  s->rx_isr_fault=car_vision_rx.isr_fault_generation;
}
static uint8_t car_control_camera_profile_ack_ready(const car_control_t *c,
  const app_config_t *a,uint32_t n) { (void)c;(void)a;(void)n;return 1u; }
static uint8_t car_servo_safety_request_selected_with_slew(car_servo_safety_t *s,
  uint8_t source,uint16_t target,uint16_t fallback,uint8_t permit,uint32_t now,
  uint16_t step,car_servo_safety_io_t *io) {
  (void)source;(void)fallback;(void)permit;(void)now;(void)step;(void)io;
  if(servo_request_fail)return 0u;
  s->active=1u;s->frame_ready=1u;s->compare_us=target;return 1u;
}
static void servo_safety_fault_to_controller(void) {}
static void set_motor_gpio_low(void) {}
static void set_motor_gpio_channels_af(uint32_t channels) { last_af_channels=channels; }
static void stop_motor_outputs_hardware(void) {
  ++motor_stop_calls;motor_outputs_active=0u;applied_follow_outputs=0u;
  TIM3->CCER=0u;TIM3->CCR1=0u;TIM3->CCR2=0u;TIM3->CCR3=0u;TIM3->CCR4=0u;
  TIM3->CR1&=~TIM_CR1_CEN;
}
static void stop_servo_outputs_hardware(void) { ++servo_stop_calls;servo_output_active=0u; }
static void emergency_stop_hardware(void) { ++emergency_stop_calls;stop_motor_outputs_hardware(); }
static void car_wireless_local_stop(uint32_t now) { (void)now;++local_stop_calls; }
static void protected_fast_stop_note(int fault) { ++protected_fast_stop_calls; protected_fast_stop_fault=(uint32_t)fault; }
'''

HARNESS = r'''
static unsigned assertions,failures,first_failure;
static void require(int condition) { ++assertions; if(!condition) { ++failures; if(!first_failure) first_failure=assertions; } }
static void reset_mock(void) {
  memset(&car_control_state,0,sizeof(car_control_state));
  memset(&app_config,0,sizeof(app_config));memset(&car_vision_rx,0,sizeof(car_vision_rx));
  memset(&manual_run_button,0,sizeof(manual_run_button));memset(&servo_safety,0,sizeof(servo_safety));
  memset(&wireless_test_state,0,sizeof(wireless_test_state));memset(&tim3_regs,0,sizeof(tim3_regs));
  memset(&usart1_regs,0,sizeof(usart1_regs));memset(&exti_regs,0,sizeof(exti_regs));
  test_now=400u;primask_state=0u;pe7_high=1u;servo_stop_pending=0u;
  servo_output_active=0u;motor_outputs_active=0u;applied_follow_outputs=0u;
  applied_servo_pulse_us=0u;applied_motor_duty_permille=0u;
  applied_motor_follow_left_permille=0u;applied_motor_follow_right_permille=0u;
  applied_motor_channel_mask=0u;wireless_test_owned=0u;wireless_return_center=0u;
  wireless_return_started_ms=0u;local_stop_calls=0u;emergency_stop_calls=0u;
  motor_stop_calls=0u;servo_stop_calls=0u;last_af_channels=0u;servo_request_fail=0u;
  app_config.motor_hardware_accepted=1u;app_config.vision_timeout_ms=1000u;
  manual_run_button.startup_release_confirmed=1u;car_control_state.fault=CAR_FAULT_NONE;
  car_control_state.isr_stop_fault=CAR_FAULT_NONE;
  wireless_test_state.active=1u;wireless_test_state.kind=WIRELESS_TEST_MOTOR;wireless_test_state.motor=3u;
  wireless_test_state.duty_permille=400u;wireless_test_state.started_ms=0u;wireless_test_state.deadline_ms=5000u;
  car_main_loop_heartbeat_ms=400u;protected_fast_stop_calls=0u;protected_fast_stop_fault=0u;
}
int main(void) {
  reset_mock();require(wireless_apply_test() && TIM3->CCR2==400u && TIM3->CCR3==400u && TIM3->CCER==(TIM_CCER_CC2E|TIM_CCER_CC3E));
  reset_mock();wireless_test_state.motor=1u;require(wireless_apply_test() && TIM3->CCR2==400u && TIM3->CCR3==0u && TIM3->CCER==TIM_CCER_CC2E);
  reset_mock();wireless_test_state.motor=2u;require(wireless_apply_test() && TIM3->CCR2==0u && TIM3->CCR3==400u && TIM3->CCER==TIM_CCER_CC3E);
  reset_mock();car_control_state.imu_fault=1u;require(wireless_apply_test() && TIM3->CCER==0u && local_stop_calls==1u && emergency_stop_calls==1u);
  reset_mock();pe7_high=0u;require(wireless_apply_test() && TIM3->CCER==0u);
  reset_mock();test_now=5000u;require(wireless_apply_test() && TIM3->CCER==0u && local_stop_calls==1u);
  reset_mock();require(wireless_apply_test() && motor_outputs_active!=0u);wireless_test_state.active=0u;test_now=500u;
  require(wireless_apply_test() && wireless_return_center==0u && servo_stop_calls==1u && motor_outputs_active==0u && TIM3->CCER==0u);
  reset_mock();wireless_test_owned=1u;motor_outputs_active=1u;pe7_high=0u;wireless_systick_guard_test();
  require(local_stop_calls==1u && emergency_stop_calls==1u);
  reset_mock();wireless_test_owned=1u;motor_outputs_active=1u;wireless_test_state.active=0u;wireless_systick_guard_test();
  require(local_stop_calls==1u && emergency_stop_calls==1u);
  reset_mock();wireless_test_owned=1u;motor_outputs_active=1u;test_now=451u;car_main_loop_heartbeat_ms=400u;wireless_systick_guard_test();
  require(protected_fast_stop_calls==1u && protected_fast_stop_fault==CAR_FAULT_LOOP_GAP && emergency_stop_calls==1u);
  reset_mock();wireless_test_owned=1u;motor_outputs_active=1u;test_now=5000u;car_main_loop_heartbeat_ms=5000u;wireless_systick_guard_test();
  require(motor_stop_calls==1u && local_stop_calls==0u && emergency_stop_calls==0u && motor_outputs_active==0u);
  return failures==0u && assertions==12u ? 0 : (int)first_failure;
}
'''

def main() -> int:
    source_text = SOURCE.read_text(encoding='utf-8')
    production = '\n\n'.join(extract_function(source_text, name)
                             for name in ('wireless_output_guard', 'wireless_apply_test'))
    systick_guard = extract_systick_wireless_guard(source_text)
    production += '\n\nstatic void wireless_systick_guard_test(void) { uint32_t now_ms = HAL_GetTick();\n' + systick_guard + '\n}\n'
    source = PRELUDE + production + HARNESS
    with tempfile.TemporaryDirectory(prefix='cartest-wireless-main-') as temp:
        temp_path = Path(temp)
        c_path = temp_path / 'wireless_main_harness.c'
        exe_path = temp_path / 'wireless_main_harness.exe'
        c_path.write_text(source, encoding='utf-8')
        build = subprocess.run([str(ZIG), 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                str(c_path), '-o', str(exe_path)],
                               cwd=ROOT, capture_output=True, text=True, check=False)
        if build.returncode:
            print(build.stdout, end='')
            print(build.stderr, end='')
            return build.returncode
        run = subprocess.run([str(exe_path)], capture_output=True, text=True, check=False)
        if run.returncode:
            print(run.stdout, end='')
            print(run.stderr, end='')
            print(f'exit={run.returncode}')
            return run.returncode
    EVIDENCE.parent.mkdir(parents=True, exist_ok=True)
    EVIDENCE.write_text(json.dumps({
        'passed': True,
        'runner': 'zig cc -std=c11 -Wall -Wextra -Werror',
        'cases': 12,
        'assertions': 12,
        'coverage': ['production wireless_output_guard safety gates',
                     'production wireless_apply_test CC2/CC3 mapping',
                     'IMU fault and PE7 stop rejection',
                     'five second deadline rejection',
                     'servo center and motor stop after expiry',
                     'production SysTick wireless-owned PE7 and authorization guard',
                     'production SysTick main-loop heartbeat gap fast stop',
                     'production SysTick expiry motor stop preserves main-loop return-center path'],
        'tested_source_sha256': {
            'Src/main.c': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'tests/test_wireless_main.py': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        },
    }, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'evidence: {EVIDENCE}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
