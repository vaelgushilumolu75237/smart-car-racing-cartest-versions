#include "motor_max_test.h"
#include <stddef.h>
_Static_assert(sizeof(motor_max_sample_t) == 24u, "trace ABI");
_Static_assert(offsetof(motor_max_test_t, samples) == 104u, "header ABI");
void motor_max_init(volatile motor_max_test_t *s)
{
    *s = (motor_max_test_t){0};
    s->magic = 0x4d415857u; s->version = 1u;
}
void motor_max_start(volatile motor_max_test_t *s, uint32_t now)
{
    if (s->phase != MOTOR_MAX_READY) return;
    s->started_ms = now; s->now_ms = now; s->phase = MOTOR_MAX_RAMP;
}
void motor_max_stop(volatile motor_max_test_t *s, uint32_t now, uint32_t reason)
{
    if (s->phase == MOTOR_MAX_DONE || s->phase == MOTOR_MAX_FAULT) return;
    s->duty_permille = 0u; s->finished_ms = now; s->stop_reason = reason;
    s->phase = reason ? MOTOR_MAX_FAULT : MOTOR_MAX_DONE;
}
void motor_max_tick(volatile motor_max_test_t *s, uint32_t now,
                    uint8_t run, uint32_t fault)
{
    s->now_ms = now;
    if (fault) { motor_max_stop(s, now, fault); return; }
    if (s->phase != MOTOR_MAX_RAMP && s->phase != MOTOR_MAX_HOLD) return;
    if (!run) { motor_max_stop(s, now, MOTOR_MAX_STOP_BUTTON); return; }
    uint32_t elapsed = now - s->started_ms;
    if (elapsed >= MOTOR_MAX_RUN_MS) { motor_max_stop(s, now, 0u); return; }
    if (s->phase == MOTOR_MAX_RAMP) {
        if (elapsed >= MOTOR_MAX_RAMP_MS) {
            s->phase = MOTOR_MAX_HOLD; s->full_duty_ms = now;
        }
        s->duty_permille = elapsed >= MOTOR_MAX_RAMP_MS ? 1000u :
                          elapsed * 1000u / MOTOR_MAX_RAMP_MS;
    }
    if (s->phase == MOTOR_MAX_HOLD) s->duty_permille = 1000u;
}
void motor_max_sample(volatile motor_max_test_t *s, uint32_t now,
    uint32_t dt, int32_t left, int32_t right, uint32_t applied_duty)
{
    if ((s->phase != MOTOR_MAX_RAMP && s->phase != MOTOR_MAX_HOLD) ||
        !dt || dt > 100u || left < 0 || right < 0 || applied_duty > 1000u) return;
    uint32_t n = s->sample_count;
    if (n < MOTOR_MAX_TRACE_CAPACITY) {
        s->samples[n] = (motor_max_sample_t){now, s->phase, applied_duty, left, right, dt};
        s->sample_count = n + 1u;
    }
    if (!applied_duty) return;
    if (left > s->peak_left_millirps) s->peak_left_millirps = left;
    if (right > s->peak_right_millirps) s->peak_right_millirps = right;
    /* Exclude the first 500 ms at full PWM from the steady-speed statistics. */
    if (applied_duty != 1000u || s->phase != MOTOR_MAX_HOLD ||
        now - s->full_duty_ms < MOTOR_MAX_WINDOW_MS + dt) return;
    s->full_left_sum += (int64_t)left * dt;
    s->full_right_sum += (int64_t)right * dt;
    s->full_sample_ms += dt;
    s->mean_full_left_millirps = (int32_t)(s->full_left_sum / s->full_sample_ms);
    s->mean_full_right_millirps = (int32_t)(s->full_right_sum / s->full_sample_ms);
    s->window_left_sum += (int64_t)left * dt;
    s->window_right_sum += (int64_t)right * dt;
    s->window_ms += dt;
    if (s->window_ms >= MOTOR_MAX_WINDOW_MS) {
        int32_t l = (int32_t)(s->window_left_sum / s->window_ms);
        int32_t r = (int32_t)(s->window_right_sum / s->window_ms);
        if (l > s->peak500_left_millirps) s->peak500_left_millirps = l;
        if (r > s->peak500_right_millirps) s->peak500_right_millirps = r;
        s->window_left_sum = 0; s->window_right_sum = 0; s->window_ms = 0u;
    }
}
