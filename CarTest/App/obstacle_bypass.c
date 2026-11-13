#include "obstacle_bypass.h"
#include <limits.h>
#ifndef CARTEST_VISION_HOLD_MS
#define CARTEST_VISION_HOLD_MS 3000u
#endif
#include <stddef.h>
#include <string.h>
/* Reuse the original gyro controller's proportional rate gain and turn
 * feed-forward. These are existing controller values, not route measurements. */
enum { RATE_KP = 2, TURN_FEEDFORWARD = 250, RECOVERY_FRAMES = 3,
       RECOVERY_MAX_AGE_MS = 250, IMU_MAX_AGE_US = 50000 };
static const int8_t turn_direction[7] = {1, 0, -1, 0, -1, 0, 1};
static int32_t clamp(int64_t v, int32_t limit) {
    return v > limit ? limit : v < -limit ? -limit : (int32_t)v;
}
uint8_t car_obstacle_config_valid(const car_obstacle_config_t *c) {
    uint32_t i;
    if (!c || !c->heading_kp_permille) return 0;
    for (i = 0; i < 7; ++i) {
        const car_obstacle_segment_t *s = &c->segment[i];
        if (!s->outer_speed_millirps || !s->yaw_rate_mdps ||
            s->yaw_rate_mdps > INT32_MAX || !s->timeout_ms ||
            s->timeout_ms > INT32_MAX) return 0;
        if (turn_direction[i]) {
            if (!s->angle_mdeg || s->angle_mdeg > INT32_MAX || s->distance_millirev) return 0;
        } else if (!s->distance_millirev || s->angle_mdeg) return 0;
    }
    return 1; /* Independently configured relative turns need not sum to zero. */
}
void car_obstacle_init(car_obstacle_t *s) {
    if (s) memset(s, 0, sizeof(*s));
}
static void fail(car_obstacle_t *s, uint8_t error) {
    s->active = 1; s->state = CAR_OBSTACLE_FAULT; s->error = error;
    s->outer_target_millirps = 0; s->control_error_norm = 0;
}
static uint8_t enter_stage(car_obstacle_t *s, const car_obstacle_config_t *c,
                           const car_obstacle_input_t *in, uint8_t stage) {
    int64_t delta;
    s->state = stage; s->stage_started_ms = in->now_ms;
    s->stage_start_millirev = in->forward_millirev; s->distance_millirev = 0;
    if (stage == CAR_OBSTACLE_REACQUIRE) {
        /* Keep the last segment speed while collecting normal route frames. */
        s->control_error_norm = 0;
        s->recovery_count = 0; s->recovery_have_sequence = 1;
        s->recovery_sequence = in->camera_sequence; s->recovery_last_ms = in->now_ms;
        return 1;
    }
    delta = (int64_t)turn_direction[stage - 1] * c->segment[stage - 1].angle_mdeg;
    if ((delta > 0 && s->heading_target_mdeg > INT64_MAX - delta) ||
        (delta < 0 && s->heading_target_mdeg < INT64_MIN - delta)) {
        fail(s, CAR_OBSTACLE_ERROR_INPUT); return 0;
    }
    s->heading_target_mdeg += delta;
    return 1;
}
static void recovery(car_obstacle_t *s, const car_obstacle_input_t *in) {
    uint16_t delta;
    if (s->recovery_have_sequence &&
        (uint32_t)(in->now_ms - s->recovery_last_ms) >= RECOVERY_MAX_AGE_MS) {
        s->recovery_count = 0; s->recovery_have_sequence = 0;
    }
    if (!in->ordinary_fresh) {
        s->recovery_count = 0; s->recovery_have_sequence = 0;
        return;
    }
    if (s->recovery_have_sequence) {
        delta = (uint16_t)(in->camera_sequence - s->recovery_sequence);
        if (!delta || delta >= 0x8000u) return;
        s->recovery_count = delta == 1u ? (uint8_t)(s->recovery_count + 1u) : 1u;
    } else s->recovery_count = 1;
    s->recovery_sequence = in->camera_sequence;
    s->recovery_last_ms = in->now_ms; s->recovery_have_sequence = 1;
    if (s->recovery_count >= RECOVERY_FRAMES) {
        s->active = 0; s->state = CAR_OBSTACLE_IDLE; s->resumed = 1;
    }
}
void car_obstacle_step(car_obstacle_t *s, const car_obstacle_config_t *c,
                       const car_obstacle_input_t *in) {
    const car_obstacle_segment_t *segment;
    int8_t direction;
    int64_t heading_error, target_rate, command;
    uint8_t done;
    if (!s) return;
    s->resumed = 0;
    if (!in) { fail(s, CAR_OBSTACLE_ERROR_INPUT); return; }
    if (!in->run_active) {
        s->active = 0; s->state = CAR_OBSTACLE_IDLE;
        s->outer_target_millirps = 0; s->control_error_norm = 0;
        s->recovery_count = 0; s->recovery_have_sequence = 0;
        return;
    }
    if (s->active) {
        if (in->trigger) ++s->ignored_triggers;
    } else if (in->trigger) {
        ++s->accepted_triggers;
        s->active = 1; s->error = 0;
        if (!car_obstacle_config_valid(c)) { fail(s, CAR_OBSTACLE_ERROR_UNCALIBRATED); return; }
        s->heading_target_mdeg = in->yaw_mdeg;
        if (!enter_stage(s, c, in, CAR_OBSTACLE_LEFT_OUT)) return;
    } else return;
    if (s->state == CAR_OBSTACLE_FAULT) return;
    if (!in->imu_ready || in->imu_fault ||
        (uint32_t)(in->now_ms * 1000u - in->imu_sample_us) >= IMU_MAX_AGE_US) {
        fail(s, CAR_OBSTACLE_ERROR_IMU); return;
    }
    if (s->state == CAR_OBSTACLE_REACQUIRE) {
        if (!in->odometry_valid) { fail(s, CAR_OBSTACLE_ERROR_ENCODER); return; }
        if ((uint32_t)(in->now_ms - s->stage_started_ms) >= CARTEST_VISION_HOLD_MS) {
            fail(s, CAR_OBSTACLE_ERROR_TIMEOUT); return;
        }
        recovery(s, in); return;
    }
    if (!in->odometry_valid || in->forward_millirev < s->stage_start_millirev) {
        fail(s, CAR_OBSTACLE_ERROR_ENCODER); return;
    }
    segment = &c->segment[s->state - 1];
    if ((uint32_t)(in->now_ms - s->stage_started_ms) >= segment->timeout_ms) {
        fail(s, CAR_OBSTACLE_ERROR_TIMEOUT); return;
    }
    /* Avoid overflow when subtracting untrusted diagnostic yaw values. */
    if ((in->yaw_mdeg < 0 && s->heading_target_mdeg > INT64_MAX + in->yaw_mdeg) ||
        (in->yaw_mdeg > 0 && s->heading_target_mdeg < INT64_MIN + in->yaw_mdeg)) {
        fail(s, CAR_OBSTACLE_ERROR_INPUT); return;
    }
    heading_error = s->heading_target_mdeg - in->yaw_mdeg;
    direction = turn_direction[s->state - 1];
    s->distance_millirev = in->forward_millirev - s->stage_start_millirev;
    done = direction ? (direction > 0 ? heading_error <= 0 : heading_error >= 0) :
        s->distance_millirev >= segment->distance_millirev;
    if (done) {
        /* User route: after stage 3 immediately turn inward at stage 5.
         * Stage 4 (pass-forward) is deliberately never entered. */
        uint8_t next = s->state == CAR_OBSTACLE_RIGHT_FORWARD ?
            CAR_OBSTACLE_RIGHT_IN : (uint8_t)(s->state + 1);
        if (!enter_stage(s, c, in, next)) return;
        /* One stage transition per tick. New setpoint applies this tick. */
        if (s->state == CAR_OBSTACLE_REACQUIRE) return;
        segment = &c->segment[s->state - 1];
        direction = turn_direction[s->state - 1];
        /* Target changed by at most INT32_MAX; reject impossible subtraction. */
        if ((in->yaw_mdeg < 0 && s->heading_target_mdeg > INT64_MAX + in->yaw_mdeg) ||
            (in->yaw_mdeg > 0 && s->heading_target_mdeg < INT64_MIN + in->yaw_mdeg)) {
            fail(s, CAR_OBSTACLE_ERROR_INPUT); return;
        }
        heading_error = s->heading_target_mdeg - in->yaw_mdeg;
    }
    if (direction) target_rate = (int64_t)direction * segment->yaw_rate_mdps;
    else {
        int32_t bounded_error = clamp(heading_error, INT32_MAX);
        target_rate = clamp((int64_t)bounded_error * c->heading_kp_permille / 1000,
                            (int32_t)segment->yaw_rate_mdps);
    }
    command = (target_rate - in->gyro_z_mdps) * RATE_KP / 1000;
    if (direction) command += (int64_t)direction * TURN_FEEDFORWARD;
    s->control_error_norm = clamp(-command, 1000);
    s->outer_target_millirps = segment->outer_speed_millirps;
}
