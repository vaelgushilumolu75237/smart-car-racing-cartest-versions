#ifndef CARTEST_MOTOR_MAX_TEST_H
#define CARTEST_MOTOR_MAX_TEST_H
#include <stdint.h>
#ifndef CARTEST_MOTOR_MAX_TEST_ENABLED
#define CARTEST_MOTOR_MAX_TEST_ENABLED 0
#endif
/* Isolated overhead test: open-loop forward PWM, not a wheel-speed demand. */
enum {
    MOTOR_MAX_RAMP_MS = 2000u, MOTOR_MAX_HOLD_MS = 5000u,
    MOTOR_MAX_RUN_MS = MOTOR_MAX_RAMP_MS + MOTOR_MAX_HOLD_MS,
    MOTOR_MAX_WINDOW_MS = 500u, MOTOR_MAX_TRACE_CAPACITY = 800u,
    MOTOR_MAX_READY = 0u, MOTOR_MAX_RAMP = 1u, MOTOR_MAX_HOLD = 2u,
    MOTOR_MAX_DONE = 3u, MOTOR_MAX_FAULT = 4u,
    MOTOR_MAX_STOP_BUTTON = 1u, MOTOR_MAX_STOP_ENCODER = 2u,
    MOTOR_MAX_STOP_LOOP = 3u, MOTOR_MAX_STOP_RESET = 4u,
    MOTOR_MAX_STOP_HARDWARE = 5u
};
typedef struct {
    uint32_t time_ms, phase, duty_permille;
    int32_t left_millirps, right_millirps;
    uint32_t elapsed_ms;
} motor_max_sample_t;
/* Fixed 32-bit debugger ABI. Results and trace remain until the next reset. */
typedef struct {
    uint32_t magic, version, phase, stop_reason, now_ms, started_ms;
    uint32_t full_duty_ms, finished_ms, duty_permille, sample_count;
    int32_t peak_left_millirps, peak_right_millirps;
    int32_t peak500_left_millirps, peak500_right_millirps;
    int32_t mean_full_left_millirps, mean_full_right_millirps;
    uint32_t full_sample_ms, window_ms;
    int64_t full_left_sum, full_right_sum, window_left_sum, window_right_sum;
    motor_max_sample_t samples[MOTOR_MAX_TRACE_CAPACITY];
} motor_max_test_t;
void motor_max_init(volatile motor_max_test_t *s);
void motor_max_start(volatile motor_max_test_t *s, uint32_t now);
void motor_max_stop(volatile motor_max_test_t *s, uint32_t now, uint32_t reason);
void motor_max_tick(volatile motor_max_test_t *s, uint32_t now,
                    uint8_t run, uint32_t fault);
void motor_max_sample(volatile motor_max_test_t *s, uint32_t now,
    uint32_t dt, int32_t left, int32_t right, uint32_t applied_duty);
#endif
