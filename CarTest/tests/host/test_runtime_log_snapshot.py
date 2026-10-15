"""Compile the exact runtime-log main.c helpers with IRQ and timer mocks."""

from __future__ import annotations

import argparse
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
#include "encoder_diag.h"
#include "follow_output.h"
#include "manual_run_button.h"
#include "runtime_log.h"

typedef struct {
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} runtime_log_test_tim_t;

typedef struct {
    uint16_t left_in1_permille;
    uint16_t left_in2_permille;
    uint16_t right_in1_permille;
    uint16_t right_in2_permille;
} motor_applied_channels_t;

static car_control_t car_control_state;
static car_encoder_diag_t car_encoder_diag;
static car_config_t app_config;
static car_manual_button_t manual_run_button;
static car_follow_output_state_t follow_output_state;
static volatile uint32_t car_servo_requested_us;
static volatile uint32_t car_servo_limited_us;
static volatile uint32_t car_servo_applied_pulse_us;
static volatile uint32_t car_servo_output_active;
static volatile uint32_t car_motor_left_in1_output_permille;
static volatile uint32_t car_motor_left_in2_output_permille;
static volatile uint32_t car_motor_right_in1_output_permille;
static volatile uint32_t car_motor_right_in2_output_permille;
static volatile uint8_t motor_outputs_active;
static uint16_t applied_motor_duty_permille;
static uint16_t applied_motor_follow_left_permille;
static uint16_t applied_motor_follow_right_permille;
static uint8_t applied_motor_channel_mask;
static uint8_t applied_follow_outputs;
static runtime_log_test_tim_t runtime_log_test_tim3_regs;

static uint32_t runtime_log_test_primask;
static uint32_t runtime_log_test_primask_reads;
static uint32_t runtime_log_test_disable_calls;
static uint32_t runtime_log_test_restore_calls;
static uint32_t runtime_log_test_restore_value;
static uint32_t runtime_log_test_snapshot_entry_primask;
static uint32_t runtime_log_test_capture_debug_at_restore;
static uint32_t runtime_log_test_restore_debug_primask;
static uint32_t runtime_log_test_restore_left_in1;
static uint32_t runtime_log_test_restore_left_in2;
static uint32_t runtime_log_test_restore_right_in1;
static uint32_t runtime_log_test_restore_right_in2;
static uint32_t runtime_log_test_irq_pending;
static uint32_t runtime_log_test_irq_count;
static uint32_t runtime_log_test_update_count;
static uint32_t runtime_log_test_update_primask;
static car_runtime_log_session_kind_t runtime_log_test_update_kind;
static car_runtime_log_sample_t runtime_log_test_last_sample;
static uint32_t runtime_log_test_sample_coherent;

static void runtime_log_test_apply_irq(void);
static void runtime_log_test_unmask(uint32_t value);
static runtime_log_test_tim_t *runtime_log_test_get_tim3(void);
static motor_applied_channels_t motor_applied_channels_snapshot(void);
static motor_applied_channels_t runtime_log_test_snapshot_channels(void);
static void publish_motor_channel_debug_snapshot(void);
static uint32_t __get_PRIMASK(void);
static void __disable_irq(void);
static void __set_PRIMASK(uint32_t value);
#define TIM3 (runtime_log_test_get_tim3())

uint8_t car_control_vision_fresh(const car_control_t *control,
                                 uint32_t now_ms,
                                 uint32_t timeout_ms)
{
    return (uint8_t)(control != 0 && timeout_ms != 0u &&
                     control->have_valid_vision != 0u &&
                     control->vision_valid != 0u &&
                     (uint32_t)(now_ms - control->vision_last_valid_ms) <
                         timeout_ms);
}

void car_runtime_log_update(const car_runtime_log_sample_t *sample,
                            car_runtime_log_session_kind_t start_kind)
{
    runtime_log_test_last_sample = *sample;
    runtime_log_test_update_kind = start_kind;
    runtime_log_test_update_primask = runtime_log_test_primask;
    ++runtime_log_test_update_count;
}

static uint32_t __get_PRIMASK(void)
{
    ++runtime_log_test_primask_reads;
    return runtime_log_test_primask;
}

static void __disable_irq(void)
{
    ++runtime_log_test_disable_calls;
    runtime_log_test_primask = 1u;
}

static void __set_PRIMASK(uint32_t value)
{
    ++runtime_log_test_restore_calls;
    runtime_log_test_restore_value = value;
    if (runtime_log_test_capture_debug_at_restore != 0u) {
        runtime_log_test_restore_debug_primask = runtime_log_test_primask;
        runtime_log_test_restore_left_in1 = car_motor_left_in1_output_permille;
        runtime_log_test_restore_left_in2 = car_motor_left_in2_output_permille;
        runtime_log_test_restore_right_in1 = car_motor_right_in1_output_permille;
        runtime_log_test_restore_right_in2 = car_motor_right_in2_output_permille;
        runtime_log_test_capture_debug_at_restore = 0u;
    }
    runtime_log_test_unmask(value);
}

static motor_applied_channels_t runtime_log_test_snapshot_channels(void)
{
    runtime_log_test_snapshot_entry_primask = runtime_log_test_primask;
    return motor_applied_channels_snapshot();
}

static void runtime_log_test_unmask(uint32_t value)
{
    runtime_log_test_primask = value;
    if (value == 0u && runtime_log_test_irq_pending != 0u) {
        runtime_log_test_irq_pending = 0u;
        runtime_log_test_apply_irq();
    }
}

static runtime_log_test_tim_t *runtime_log_test_get_tim3(void)
{
    if (runtime_log_test_irq_count == 0u) {
        if (runtime_log_test_primask == 0u) {
            runtime_log_test_apply_irq();
        } else {
            runtime_log_test_irq_pending = 1u;
        }
    }
    return &runtime_log_test_tim3_regs;
}

static void runtime_log_test_apply_irq(void)
{
    ++runtime_log_test_irq_count;
    car_control_state.mode = CAR_MODE_FAULT_LATCHED;
    car_control_state.fault = CAR_FAULT_TIMER;
    car_control_state.fault_latched = 1u;
    ++car_control_state.fault_generation;
    car_servo_output_active = 0u;
    car_servo_applied_pulse_us = 0u;
    car_motor_left_in1_output_permille = 0u;
    car_motor_left_in2_output_permille = 0u;
    car_motor_right_in1_output_permille = 0u;
    car_motor_right_in2_output_permille = 0u;
    runtime_log_test_tim3_regs.CCR1 = 0u;
    runtime_log_test_tim3_regs.CCR2 = 0u;
    runtime_log_test_tim3_regs.CCR3 = 0u;
    runtime_log_test_tim3_regs.CCR4 = 0u;
}

static uint32_t runtime_log_test_is_sample_coherent(void)
{
    const car_runtime_log_sample_t *sample = &runtime_log_test_last_sample;
    return (uint32_t)(sample->mode == (uint8_t)CAR_MODE_FOLLOW_RUNNING &&
                      sample->fault == (uint8_t)CAR_FAULT_NONE &&
                      sample->hard_fault_latched == 0u &&
                      sample->servo_applied_us == 1530u &&
                      sample->left_applied_permille == 200u &&
                      sample->right_applied_permille == 210u &&
                      sample->motor_ccr1_counts == 321u &&
                      sample->motor_ccr4_counts == 322u);
}

static void runtime_log_test_init_snapshot(void)
{
    memset(&car_control_state, 0, sizeof(car_control_state));
    memset(&car_encoder_diag, 0, sizeof(car_encoder_diag));
    memset(&app_config, 0, sizeof(app_config));
    memset(&manual_run_button, 0, sizeof(manual_run_button));
    memset(&runtime_log_test_tim3_regs, 0, sizeof(runtime_log_test_tim3_regs));
    memset(&runtime_log_test_last_sample, 0, sizeof(runtime_log_test_last_sample));
    runtime_log_test_irq_pending = 0u;
    runtime_log_test_irq_count = 0u;
    runtime_log_test_primask_reads = 0u;
    runtime_log_test_disable_calls = 0u;
    runtime_log_test_restore_calls = 0u;
    runtime_log_test_restore_value = 0xFFFFFFFFu;
    runtime_log_test_snapshot_entry_primask = 0xFFFFFFFFu;
    runtime_log_test_capture_debug_at_restore = 0u;
    runtime_log_test_restore_debug_primask = 0xFFFFFFFFu;
    runtime_log_test_restore_left_in1 = 0xFFFFFFFFu;
    runtime_log_test_restore_left_in2 = 0xFFFFFFFFu;
    runtime_log_test_restore_right_in1 = 0xFFFFFFFFu;
    runtime_log_test_restore_right_in2 = 0xFFFFFFFFu;
    runtime_log_test_primask = 0u;
    runtime_log_test_update_count = 0u;
    runtime_log_test_update_primask = 0xFFFFFFFFu;
    runtime_log_test_update_kind = CAR_RUNTIME_LOG_SESSION_NONE;
    app_config.follow_mode_compiled = 1u;
    app_config.vision_timeout_ms = 250u;
    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    car_control_state.fault = CAR_FAULT_NONE;
    car_control_state.follow_session_active = 1u;
    car_control_state.manual_run_request = 1u;
    car_control_state.have_valid_vision = 1u;
    car_control_state.vision_valid = 1u;
    car_control_state.vision_last_valid_ms = 10u;
    car_control_state.follow_output_duty_permille = 400u;
    car_encoder_diag.left_raw_delta_sum = 123;
    car_encoder_diag.right_raw_delta_sum = -456;
    car_servo_requested_us = 1530u;
    car_servo_limited_us = 1530u;
    car_servo_applied_pulse_us = 1530u;
    car_servo_output_active = 1u;
    car_motor_left_in1_output_permille = 200u;
    car_motor_left_in2_output_permille = 210u; /* stale max from old publisher */
    car_motor_right_in1_output_permille = 210u;
    car_motor_right_in2_output_permille = 0u;
    motor_outputs_active = 1u;
    applied_motor_duty_permille = 210u;
    applied_motor_follow_left_permille = 200u;
    applied_motor_follow_right_permille = 210u;
    applied_motor_channel_mask = 0xFFu;
    applied_follow_outputs = 1u;
    runtime_log_test_tim3_regs.CCR2 = 200u;
    runtime_log_test_tim3_regs.CCR3 = 210u;
    runtime_log_test_tim3_regs.CCR1 = 321u;
    runtime_log_test_tim3_regs.CCR4 = 322u;
}

'''


def extract_function(source: str, name: str) -> str:
    pattern = re.compile(
        r"\bstatic\s+[^;{}]*?\b" + re.escape(name) + r"\s*\([^;{}]*?\)\s*\{",
        re.DOTALL,
    )
    matches = list(pattern.finditer(source))
    if not matches:
        return ""
    match = matches[-1]
    brace = source.find("{", match.start(), match.end())
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise AssertionError("Unbalanced function body for " + name)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=PROJECT_ROOT / "Src/main.c")
    parser.add_argument("--expect-mixed", action="store_true",
                        help="positive control for a saved pre-fix main.c")
    parser.add_argument("--expect-legacy-channel-mismatch", action="store_true",
                        help="prove a saved legacy main.c logs max duty on both Follow sides")
    args = parser.parse_args()
    source_path = args.source.resolve()
    source = source_path.read_text(encoding="utf-8")

    max_helper = extract_function(source, "runtime_log_max_u16")
    start_edge = extract_function(source, "runtime_log_start_edge")
    channels_helper = extract_function(source, "motor_applied_channels_snapshot")
    publish_channels = extract_function(source, "publish_motor_channel_debug_snapshot")
    debug_snapshot = extract_function(source, "publish_debug_snapshot")
    capture = extract_function(source, "runtime_log_capture_sample")
    publish = extract_function(source, "runtime_log_publish_sample")
    if args.expect_legacy_channel_mismatch:
        if not (max_helper and capture):
            raise SystemExit("Legacy channel control is missing runtime-log capture helpers")
    elif not (max_helper and start_edge and channels_helper and publish_channels and
              debug_snapshot and publish):
        raise SystemExit("Could not extract the production runtime logger helpers from " + str(source_path))

    if not args.expect_legacy_channel_mismatch:
        if "publish_motor_channel_debug_snapshot();" not in debug_snapshot:
            raise SystemExit("Production debug snapshot no longer publishes committed per-channel outputs")
        if "__disable_irq" in debug_snapshot or "__set_PRIMASK" in debug_snapshot:
            raise SystemExit("Debug publisher masks interrupts beyond the bounded motor-channel snapshot")
        ordered_tokens = (
            "__get_PRIMASK()", "__disable_irq()", "motor_applied_channels_snapshot()",
            "car_motor_left_in1_output_permille =", "car_motor_left_in2_output_permille =",
            "car_motor_right_in1_output_permille =", "car_motor_right_in2_output_permille =",
            "__set_PRIMASK(primask)",
        )
        positions = [publish_channels.find(token) for token in ordered_tokens]
        if any(position < 0 for position in positions) or positions != sorted(positions):
            raise SystemExit("Motor-channel debug snapshot does not bound its read/publication with PRIMASK")
        if "motor_applied_channels_snapshot()" not in capture:
            raise SystemExit("Runtime log capture no longer snapshots committed per-channel outputs")
        if "car_motor_left_in1_output_permille" in capture or \
                "car_motor_right_in1_output_permille" in capture:
            raise SystemExit("Runtime log capture reads cached debug outputs instead of committed outputs")

    if capture and ("crc32" in capture.lower() or "cache" in capture.lower() or
                    "append_record" in capture):
        raise SystemExit("Snapshot helper contains CRC/cache/append work")
    if not re.search(
            r"#if\s+CARTEST_RUNTIME_LOG_ENABLED\s+runtime_log_publish_sample\([^;]*\);\s+#endif",
            source,
            re.DOTALL,
    ):
        raise SystemExit("No-log main-loop compile guard around runtime logging was lost")

    old_edge_signature = "follow_session_was_active" in start_edge
    publish_channels = publish_channels.replace(
        "motor_applied_channels_snapshot()",
        "runtime_log_test_snapshot_channels()",
        1,
    )
    function_text = "\n\n".join(part for part in (
        max_helper, start_edge, channels_helper, publish_channels, capture, publish
    ) if part)
    expected_mixed = 1 if args.expect_mixed else 0
    harness_main = r'''
static int motor_mapping_case(const char *label,
                              uint8_t active,
                              uint8_t follow,
                              uint8_t channel_mask,
                              uint16_t scalar_duty,
                              uint16_t follow_left,
                              uint16_t follow_right,
                              uint16_t expected_left_in1,
                              uint16_t expected_left_in2,
                              uint16_t expected_right_in1,
                              uint16_t expected_right_in2)
{
    car_runtime_log_sample_t sample;
    uint16_t expected_left = expected_left_in1 > expected_left_in2
                                 ? expected_left_in1 : expected_left_in2;
    uint16_t expected_right = expected_right_in1 > expected_right_in2
                                  ? expected_right_in1 : expected_right_in2;

    runtime_log_test_init_snapshot();
    motor_outputs_active = active;
    applied_follow_outputs = follow;
    applied_motor_channel_mask = channel_mask;
    applied_motor_duty_permille = scalar_duty;
    applied_motor_follow_left_permille = follow_left;
    applied_motor_follow_right_permille = follow_right;
    car_motor_left_in1_output_permille = 999u;
    car_motor_left_in2_output_permille = 999u;
    car_motor_right_in1_output_permille = 999u;
    car_motor_right_in2_output_permille = 999u;
    runtime_log_test_tim3_regs.CCR2 = follow != 0u ? follow_left :
        ((channel_mask & 0x02u) != 0u ? scalar_duty : 0u);
    runtime_log_test_tim3_regs.CCR3 = follow != 0u ? follow_right :
        ((channel_mask & 0x04u) != 0u ? scalar_duty : 0u);

    publish_motor_channel_debug_snapshot();
    if (car_motor_left_in1_output_permille != expected_left_in1 ||
        car_motor_left_in2_output_permille != expected_left_in2 ||
        car_motor_right_in1_output_permille != expected_right_in1 ||
        car_motor_right_in2_output_permille != expected_right_in2) {
        printf("FAIL %s debug channels=(%u,%u,%u,%u) expected=(%u,%u,%u,%u)\n",
               label,
               (unsigned)car_motor_left_in1_output_permille,
               (unsigned)car_motor_left_in2_output_permille,
               (unsigned)car_motor_right_in1_output_permille,
               (unsigned)car_motor_right_in2_output_permille,
               (unsigned)expected_left_in1, (unsigned)expected_left_in2,
               (unsigned)expected_right_in1, (unsigned)expected_right_in2);
        return 1;
    }

    runtime_log_test_irq_count = 1u; /* Keep this case free of IRQ injection. */
    runtime_log_test_primask = 0u;
    sample = runtime_log_capture_sample(11u);
    if (sample.left_applied_permille != expected_left ||
        sample.right_applied_permille != expected_right) {
        printf("FAIL %s log sides=(%u,%u) expected=(%u,%u)\n",
               label, (unsigned)sample.left_applied_permille,
               (unsigned)sample.right_applied_permille,
               (unsigned)expected_left, (unsigned)expected_right);
        return 1;
    }
    return 0;
}

static int test_motor_output_channel_mapping(void)
{
    if (motor_mapping_case("follow-507-493", 1u, 1u, 0xFFu, 507u,
                           507u, 493u, 0u, 507u, 493u, 0u) != 0 ||
        motor_mapping_case("follow-486-514", 1u, 1u, 0xFFu, 514u,
                           486u, 514u, 0u, 486u, 514u, 0u) != 0 ||
        motor_mapping_case("follow-700-300", 1u, 1u, 0xFFu, 700u,
                           700u, 300u, 0u, 700u, 300u, 0u) != 0 ||
        motor_mapping_case("idle-stopped-zero", 0u, 1u, 0xFFu, 700u,
                           700u, 300u, 0u, 0u, 0u, 0u) != 0 ||
        motor_mapping_case("hardware-locked-zero", 0u, 0u, 0x0Fu, 400u,
                           400u, 400u, 0u, 0u, 0u, 0u) != 0 ||
        motor_mapping_case("bench-single-left-in1", 1u, 0u, 0x01u, 347u,
                           700u, 300u, 347u, 0u, 0u, 0u) != 0 ||
        motor_mapping_case("bench-single-left-in2", 1u, 0u, 0x02u, 347u,
                           700u, 300u, 0u, 347u, 0u, 0u) != 0 ||
        motor_mapping_case("bench-single-right-in1", 1u, 0u, 0x04u, 347u,
                           700u, 300u, 0u, 0u, 347u, 0u) != 0 ||
        motor_mapping_case("bench-single-right-in2", 1u, 0u, 0x08u, 347u,
                           700u, 300u, 0u, 0u, 0u, 347u) != 0 ||
        motor_mapping_case("bench-paired", 1u, 0u, 0x09u, 347u,
                           700u, 300u, 347u, 0u, 0u, 347u) != 0) {
        return 1;
    }
    printf("MOTOR_OUTPUT_CHANNELS_PASS follow=507/493,486/514,700/300 bench=single+paired stopped=zero\n");
    return 0;
}

static int test_motor_debug_snapshot_primask(void)
{
    uint32_t incoming;
    for (incoming = 0u; incoming <= 1u; ++incoming) {
        runtime_log_test_init_snapshot();
        motor_outputs_active = 1u;
        applied_follow_outputs = 1u;
        applied_motor_follow_left_permille = 507u;
        applied_motor_follow_right_permille = 493u;
        runtime_log_test_primask = incoming;
        runtime_log_test_irq_pending = 1u;
        runtime_log_test_capture_debug_at_restore = 1u;

        publish_motor_channel_debug_snapshot();

        if (runtime_log_test_snapshot_entry_primask != 1u ||
            runtime_log_test_restore_debug_primask != 1u ||
            runtime_log_test_restore_left_in1 != 0u ||
            runtime_log_test_restore_left_in2 != 507u ||
            runtime_log_test_restore_right_in1 != 493u ||
            runtime_log_test_restore_right_in2 != 0u ||
            runtime_log_test_primask != incoming ||
            runtime_log_test_restore_calls != 1u ||
            runtime_log_test_restore_value != incoming) {
            printf("FAIL channel snapshot PRIMASK incoming=%u entry=%u restore=%u final=%u\n",
                   (unsigned)incoming,
                   (unsigned)runtime_log_test_snapshot_entry_primask,
                   (unsigned)runtime_log_test_restore_debug_primask,
                   (unsigned)runtime_log_test_primask);
            return 1;
        }
        if (incoming == 0u) {
            if (runtime_log_test_irq_count != 1u ||
                runtime_log_test_irq_pending != 0u) {
                printf("FAIL enabled caller did not service deferred channel-snapshot IRQ\n");
                return 1;
            }
        } else if (runtime_log_test_irq_count != 0u ||
                   runtime_log_test_irq_pending == 0u) {
            printf("FAIL masked caller changed pending channel-snapshot IRQ\n");
            return 1;
        }
    }
    printf("MOTOR_DEBUG_SNAPSHOT_PRIMASK_PASS incoming=0,1\n");
    return 0;
}

int main(void)
{
    uint32_t incoming;
    uint32_t mixed;
    uint32_t expected_mixed = EXPECT_MIXED;
    car_runtime_log_session_kind_t kind;

    car_control_state.mode = CAR_MODE_FOLLOW_RUNNING;
    car_control_state.follow_session_active = 1u;
    kind = START_EDGE_CALL;
    if (kind != (expected_mixed != 0u
                     ? CAR_RUNTIME_LOG_SESSION_FOLLOW
                     : CAR_RUNTIME_LOG_SESSION_NONE)) {
        printf("FAIL automatic Follow transition start_kind=%u\n", (unsigned)kind);
        return 1;
    }
    car_control_state.mode = CAR_MODE_BENCH_RUNNING;
    kind = BENCH_EDGE_CALL;
    if (kind != CAR_RUNTIME_LOG_SESSION_BENCH) {
        printf("FAIL explicit Bench transition start_kind=%u\n", (unsigned)kind);
        return 1;
    }

    for (incoming = 0u; incoming <= 1u; ++incoming) {
        const uint32_t expected_mixed_this_call =
            expected_mixed != 0u && incoming == 0u ? 1u : 0u;
        runtime_log_test_init_snapshot();
        runtime_log_test_primask = incoming;
        runtime_log_publish_sample(11u, CAR_RUNTIME_LOG_SESSION_NONE);
        mixed = runtime_log_test_is_sample_coherent() == 0u ? 1u : 0u;
        if (mixed != expected_mixed_this_call) {
            printf("FAIL snapshot expected_mixed=%u actual_mixed=%u incoming_primask=%u\n",
                   (unsigned)expected_mixed_this_call, (unsigned)mixed,
                   (unsigned)incoming);
            return 1;
        }
        if (runtime_log_test_update_count != 1u ||
            runtime_log_test_update_primask != incoming) {
            printf("FAIL logger update mask expected=%u actual=%u\n",
                   (unsigned)incoming, (unsigned)runtime_log_test_update_primask);
            return 1;
        }
        if (expected_mixed == 0u &&
            (runtime_log_test_primask_reads != 1u ||
             runtime_log_test_disable_calls != 1u ||
             runtime_log_test_restore_calls != 1u ||
             runtime_log_test_restore_value != incoming)) {
            printf("FAIL snapshot did not restore incoming PRIMASK exactly\n");
            return 1;
        }
        if (incoming == 0u) {
            if (runtime_log_test_primask != 0u ||
                runtime_log_test_irq_count != 1u ||
                runtime_log_test_irq_pending != 0u) {
                printf("FAIL unmasked caller was not restored before logger update\n");
                return 1;
            }
            if (expected_mixed_this_call == 0u &&
                (car_control_state.fault_latched == 0u ||
                 car_motor_left_in1_output_permille != 0u)) {
                printf("FAIL injected IRQ did not run after a coherent snapshot\n");
                return 1;
            }
        } else {
            if (runtime_log_test_primask != 1u ||
                runtime_log_test_irq_count != 0u ||
                runtime_log_test_irq_pending == 0u) {
                printf("FAIL masked caller state or pending IRQ was changed\n");
                return 1;
            }
            runtime_log_test_unmask(0u);
            if (runtime_log_test_irq_count != 1u ||
                car_control_state.fault_latched == 0u) {
                printf("FAIL pending IRQ did not run after caller unmasked\n");
                return 1;
            }
        }
    }
    if (test_motor_output_channel_mapping() != 0) {
        return 1;
    }
    if (test_motor_debug_snapshot_primask() != 0) {
        return 1;
    }
    printf("RUNTIME_LOG_SNAPSHOT_PASS expected_mixed=%u primask_inputs=0,1\n",
           (unsigned)expected_mixed);
    return 0;
}
'''.replace("EXPECT_MIXED", str(expected_mixed))
    if old_edge_signature:
        harness_main = harness_main.replace(
            "START_EDGE_CALL",
            "runtime_log_start_edge(0u, CAR_MODE_RECOVERY_WAIT)",
        ).replace(
            "BENCH_EDGE_CALL",
            "runtime_log_start_edge(0u, CAR_MODE_BOOT_SAFE)",
        )
    else:
        harness_main = harness_main.replace(
            "START_EDGE_CALL", "runtime_log_start_edge(CAR_MODE_RECOVERY_WAIT)"
        ).replace(
            "BENCH_EDGE_CALL", "runtime_log_start_edge(CAR_MODE_BOOT_SAFE)"
        )

    if args.expect_legacy_channel_mismatch:
        function_text = "\n\n".join(part for part in (max_helper, capture) if part)
        harness_main = r'''
int main(void)
{
    car_runtime_log_sample_t sample;
    runtime_log_test_init_snapshot();
    motor_outputs_active = 1u;
    applied_follow_outputs = 1u;
    applied_motor_duty_permille = 507u;
    applied_motor_follow_left_permille = 507u;
    applied_motor_follow_right_permille = 493u;
    applied_motor_channel_mask = 0xFFu;
    car_motor_left_in1_output_permille = 0u;
    car_motor_left_in2_output_permille = 507u;
    car_motor_right_in1_output_permille = 507u;
    car_motor_right_in2_output_permille = 0u;
    runtime_log_test_tim3_regs.CCR2 = 507u;
    runtime_log_test_tim3_regs.CCR3 = 493u;
    runtime_log_test_irq_count = 1u;
    runtime_log_test_primask = 1u;
    sample = runtime_log_capture_sample(11u);
    if (sample.left_applied_permille == 507u &&
        sample.right_applied_permille == 507u &&
        sample.motor_ccr2_counts == 507u &&
        sample.motor_ccr3_counts == 493u) {
        printf("LEGACY_CHANNEL_BUG_REPRODUCED csv=507/507 CCR2/CCR3=507/493 expected=507/493\n");
        return 0;
    }
    printf("FAIL legacy-control result sides=%u/%u CCR2/CCR3=%u/%u\n",
           (unsigned)sample.left_applied_permille,
           (unsigned)sample.right_applied_permille,
           (unsigned)sample.motor_ccr2_counts,
           (unsigned)sample.motor_ccr3_counts);
    return 1;
}
'''

    with tempfile.TemporaryDirectory(prefix="CarTestRuntimeLogSnapshot-") as temp:
        temp_path = Path(temp)
        c_path = temp_path / "runtime_log_snapshot_harness.c"
        exe_path = temp_path / "runtime_log_snapshot_harness.exe"
        c_path.write_text(
            PRELUDE + function_text + "\n\n" + harness_main,
            encoding="utf-8-sig",
        )
        compile_result = subprocess.run(
            ["cl", "/nologo", "/std:c11", "/W4", "/TC",
             "/I" + str(PROJECT_ROOT / "Inc"),
             "/Fe:" + str(exe_path), str(c_path)],
            cwd=PROJECT_ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        print("SOURCE {}".format(source_path))
        print(compile_result.stdout, end="")
        if compile_result.returncode != 0:
            return compile_result.returncode
        run_result = subprocess.run(
            [str(exe_path)], text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, encoding="utf-8-sig", errors="replace",
            check=False,
        )
        print(run_result.stdout, end="")
        return run_result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
