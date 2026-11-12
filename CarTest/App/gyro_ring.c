#include "gyro_ring.h"
#include "gyro350_config.h"
#if CARTEST_WIRELESS_ENABLED
#include "wireless_runtime.h"
#define RING_RATE (car_wireless_settings()->gyro_rate_mdps)
#define RING_PROGRESS (car_wireless_settings()->ring_progress_mdeg)
#define RING_EXIT_MS (car_wireless_settings()->ring_exit_ms)
#define RING_SEARCH_MS (car_wireless_settings()->ring_search_ms)
#define GYRO_P_SCALE (car_wireless_settings()->gyro_p_gain_permille)
#define GYRO_I_SCALE (car_wireless_settings()->gyro_i_gain_permille)
#else
#define RING_RATE CAR_GYRO_RING_TARGET_YAW_RATE_MDPS
#define RING_PROGRESS CAR_GYRO_RING_TARGET_PROGRESS_MDEG
#define RING_EXIT_MS CAR_GYRO_RING_EXIT_STRAIGHT_MS
#define RING_SEARCH_MS CAR_GYRO_RING_SEARCH_LIMIT_MS
#define GYRO_P_SCALE 1000
#define GYRO_I_SCALE 1000
#endif

#include <limits.h>
#include <stddef.h>
#include <string.h>

enum {
    ENTRY_FEEDFORWARD_PERMILLE = 250,
    ENTRY_KP_PER_DPS_NUMERATOR = 2,
    ENTRY_KI_PER_DPS_SECOND_NUMERATOR = 1,
    ENTRY_INTEGRAL_LIMIT_PERMILLE = 250,
    IMU_FRESH_LIMIT_US = 50000u,
    PI_MAX_SAMPLE_DT_US = 50000u
};

#define PI_INTEGRAL_DENOMINATOR INT64_C(1000000000)

static uint8_t is_active_state(uint8_t state)
{
    return (state == CAR_GYRO_RING_ENTRY ||
            state == CAR_GYRO_RING_ORBIT ||
            state == CAR_GYRO_RING_CENTER ||
            state == CAR_GYRO_RING_EXIT_STRAIGHT ||
            state == CAR_GYRO_RING_SEARCH ||
            state == CAR_GYRO_RING_WAIT_ROUTE ||
            state == CAR_GYRO_RING_FAULT) ? 1u : 0u;
}

static void set_state(car_gyro_ring_t *ring, uint8_t state,
                      uint32_t now_ms)
{
    ring->state = state;
    ring->phase = state;
    ring->state_started_ms = now_ms;
    ring->last_event_ms = now_ms;
}

static int32_t clamp_i32(int64_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return (int32_t)value;
}

static uint8_t sequence_is_forward(uint16_t sequence, uint16_t previous)
{
    uint16_t delta = (uint16_t)(sequence - previous);
    return (delta != 0u && delta < UINT16_C(0x8000)) ? 1u : 0u;
}

/* A camera sequence is consumed once whether it carries an ordinary frame,
   a phase event, or both. Half-range arithmetic rejects old packets at wrap. */
static uint8_t consume_new_sequence(car_gyro_ring_t *ring,
                                    const car_gyro_ring_input_t *input)
{
    if (input->camera_fresh == 0u && input->ordinary_fresh == 0u) {
        return 0u;
    }
    if (ring->have_camera_sequence != 0u &&
        sequence_is_forward(input->camera_sequence,
                            ring->last_camera_sequence) == 0u) {
        return 0u;
    }
    ring->last_camera_sequence = input->camera_sequence;
    ring->have_camera_sequence = 1u;
    return 1u;
}

static void reset_recovery(car_gyro_ring_t *ring)
{
    ring->recovery_count = 0u;
    ring->have_ordinary_sequence = 0u;
}

static void update_recovery(car_gyro_ring_t *ring,
                            const car_gyro_ring_input_t *input,
                            uint8_t new_sequence)
{
    uint16_t delta;

    if (ring->have_ordinary_sequence != 0u &&
        (uint32_t)(input->now_ms - ring->recovery_last_ms) >
            CAR_GYRO_RING_RECOVERY_MAX_AGE_MS) {
        reset_recovery(ring);
    }
    if (new_sequence == 0u) {
        return;
    }
    if (input->camera_phase != 0u || input->ordinary_fresh == 0u) {
        reset_recovery(ring);
        return;
    }

    if (ring->have_ordinary_sequence == 0u) {
        ring->recovery_count = 1u;
    } else {
        delta = (uint16_t)(input->camera_sequence -
                           ring->last_ordinary_sequence);
        if (delta == 1u) {
            if (ring->recovery_count < CAR_GYRO_RING_RECOVERY_FRAMES) {
                ++ring->recovery_count;
            }
        } else {
            ring->recovery_count = 1u;
        }
    }
    ring->last_ordinary_sequence = input->camera_sequence;
    ring->have_ordinary_sequence = 1u;
    ring->recovery_last_ms = input->now_ms;
}

static uint8_t recovery_is_fresh(const car_gyro_ring_t *ring,
                                 const car_gyro_ring_input_t *input)
{
    return (ring->recovery_count >= CAR_GYRO_RING_RECOVERY_FRAMES &&
            ring->have_ordinary_sequence != 0u &&
            (uint32_t)(input->now_ms - ring->recovery_last_ms) <=
                CAR_GYRO_RING_RECOVERY_MAX_AGE_MS) ? 1u : 0u;
}

static void clear_integrator(car_gyro_ring_t *ring)
{
    ring->imu_integral_initialized = 0u;
    ring->last_imu_sample_us = 0u;
    ring->imu_integral_permille = 0;
    ring->imu_integral_remainder = 0;
}

static void latch_fault(car_gyro_ring_t *ring, uint8_t reason,
                        uint32_t now_ms)
{
    ring->fault = reason;
    ring->fault_latched = 1u;
    ring->active = 1u;
    ring->allow_rearm = 0u;
    ring->control_error_norm = 0;
    ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
    ring->outer_target_millirps = 0u;
    ring->search_elapsed_ms = 0u;
    ring->hold_ramp = 0u;
    ring->state = CAR_GYRO_RING_FAULT;
    ring->phase = CAR_GYRO_RING_FAULT;
    ring->last_event_ms = now_ms;
}

static uint8_t imu_is_fresh(const car_gyro_ring_input_t *input)
{
    uint32_t now_us = input->now_ms * 1000u;
    uint32_t age_us = (uint32_t)(now_us - input->imu_sample_us);
    return (age_us <= IMU_FRESH_LIMIT_US) ? 1u : 0u;
}

static uint8_t active_inputs_valid(const car_gyro_ring_input_t *input)
{
    return (input->transport_fresh != 0u && input->imu_ready != 0u &&
            input->imu_fault == 0u && imu_is_fresh(input) != 0u) ? 1u : 0u;
}

static void reset_session(car_gyro_ring_t *ring)
{
    (void)memset(ring, 0, sizeof(*ring));
    ring->state = CAR_GYRO_RING_OFF;
    ring->phase = CAR_GYRO_RING_OFF;
    ring->servo_target_us = 0u;
}

static void finish_ring(car_gyro_ring_t *ring, uint32_t now_ms)
{
    ring->active = 0u;
    ring->fault = CAR_GYRO_RING_FAULT_NONE;
    ring->fault_latched = 0u;
    ring->allow_rearm = 1u;
    ring->control_error_norm = 0;
    ring->servo_target_us = 0u;
    ring->outer_target_millirps = 0u;
    ring->hold_ramp = 0u;
    ring->search_elapsed_ms = 0u;
    ring->entry_side = 0;
    clear_integrator(ring);
    reset_recovery(ring);
    set_state(ring, CAR_GYRO_RING_REARM, now_ms);
}

static void start_entry(car_gyro_ring_t *ring,
                        const car_gyro_ring_input_t *input)
{
    ring->active = 1u;
    ring->fault = CAR_GYRO_RING_FAULT_NONE;
    ring->fault_latched = 0u;
    ring->allow_rearm = 0u;
    ring->entry_side = input->camera_side;
    ring->control_error_norm = 0;
    ring->servo_target_us = 0u;
    ring->outer_target_millirps =
        CAR_GYRO_RING_ENTRY_ORBIT_OUTER_TARGET_MILLIRPS;
    ring->progress_mdeg = 0;
    ring->yaw_origin_mdeg = 0;
    ring->search_elapsed_ms = 0u;
    ring->hold_ramp = 0u;
    clear_integrator(ring);
    reset_recovery(ring);
    set_state(ring, CAR_GYRO_RING_ENTRY, input->now_ms);
}

static int64_t saturated_sub_i64(int64_t left, int64_t right)
{
    if (right > 0 && left < INT64_MIN + right) {
        return INT64_MIN;
    }
    if (right < 0 && left > INT64_MAX + right) {
        return INT64_MAX;
    }
    return left - right;
}

static int64_t progress_for_side(int8_t side, int64_t yaw,
                                 int64_t origin)
{
    int64_t delta = saturated_sub_i64(yaw, origin);
    if (side < 0) {
        return delta;
    }
    if (delta == INT64_MIN) {
        return INT64_MAX;
    }
    return -delta;
}

static void update_entry_pi(car_gyro_ring_t *ring,
                            const car_gyro_ring_input_t *input)
{
    int64_t rate_error;
    int64_t turn_command;
    int64_t integral_numerator;
    uint32_t sample_dt_us;
    int32_t target_rate_mdps =
        -((int32_t)ring->entry_side *
          (int32_t)RING_RATE);

    /* The PI integrator consumes each physical IMU sample timestamp once. */
    if (ring->imu_integral_initialized == 0u) {
        ring->last_imu_sample_us = input->imu_sample_us;
        ring->imu_integral_initialized = 1u;
    } else if (input->imu_sample_us != ring->last_imu_sample_us) {
        sample_dt_us = (uint32_t)(input->imu_sample_us -
                                  ring->last_imu_sample_us);
        if (sample_dt_us != 0u && sample_dt_us <= PI_MAX_SAMPLE_DT_US) {
            rate_error = (int64_t)target_rate_mdps - input->gyro_z_mdps;
            integral_numerator = rate_error * sample_dt_us * GYRO_I_SCALE / 1000 +
                                 ring->imu_integral_remainder;
            ring->imu_integral_permille +=
                (int32_t)(integral_numerator / PI_INTEGRAL_DENOMINATOR);
            ring->imu_integral_remainder =
                integral_numerator % PI_INTEGRAL_DENOMINATOR;
            if (ring->imu_integral_permille > ENTRY_INTEGRAL_LIMIT_PERMILLE) {
                ring->imu_integral_permille = ENTRY_INTEGRAL_LIMIT_PERMILLE;
                ring->imu_integral_remainder = 0;
            } else if (ring->imu_integral_permille <
                       -ENTRY_INTEGRAL_LIMIT_PERMILLE) {
                ring->imu_integral_permille = -ENTRY_INTEGRAL_LIMIT_PERMILLE;
                ring->imu_integral_remainder = 0;
            }
        }
        ring->last_imu_sample_us = input->imu_sample_us;
    }

    rate_error = (int64_t)target_rate_mdps - input->gyro_z_mdps;
    turn_command = -(int64_t)ring->entry_side *
                   ENTRY_FEEDFORWARD_PERMILLE +
                   rate_error * ENTRY_KP_PER_DPS_NUMERATOR * GYRO_P_SCALE / 1000000 +
                   ring->imu_integral_permille;

    /* Normalized vision error is positive to the right; gyro yaw has the
       opposite sign, so the output mapping is mirrored here. */
    ring->control_error_norm = clamp_i32(-turn_command, -1000, 1000);
}

static void apply_state_outputs(car_gyro_ring_t *ring,
                                const car_gyro_ring_input_t *input)
{
    uint32_t elapsed_ms =
        (uint32_t)(input->now_ms - ring->state_started_ms);

    ring->active = is_active_state(ring->state);
    ring->phase = ring->state;
    ring->hold_ramp = 0u;
    ring->search_elapsed_ms = 0u;

    switch (ring->state) {
    case CAR_GYRO_RING_ENTRY:
        if (elapsed_ms < CAR_GYRO_RING_APPROACH_MS) {
            ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
            ring->control_error_norm = 0;
            ring->outer_target_millirps = CAR_GYRO_RING_ENTRY_ORBIT_OUTER_TARGET_MILLIRPS;
            clear_integrator(ring);
            break;
        }
        ring->servo_target_us = 0u;
        ring->outer_target_millirps =
            CAR_GYRO_RING_ENTRY_ORBIT_OUTER_TARGET_MILLIRPS;
        update_entry_pi(ring, input);
        break;
    case CAR_GYRO_RING_ORBIT:
        ring->servo_target_us = 0u;
        ring->outer_target_millirps =
            CAR_GYRO_RING_ENTRY_ORBIT_OUTER_TARGET_MILLIRPS;
        update_entry_pi(ring, input);
        ring->progress_mdeg = progress_for_side(
            ring->entry_side, input->yaw_mdeg, ring->yaw_origin_mdeg);
        break;
    case CAR_GYRO_RING_CENTER:
    case CAR_GYRO_RING_EXIT_STRAIGHT:
        ring->control_error_norm = 0;
        ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
        ring->outer_target_millirps = CAR_GYRO_RING_OUTER_TARGET_MILLIRPS;
        break;
    case CAR_GYRO_RING_SEARCH:
        ring->control_error_norm = 0;
        ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
        ring->outer_target_millirps = CAR_GYRO_RING_OUTER_TARGET_MILLIRPS;
        ring->hold_ramp = 1u;
        ring->search_elapsed_ms =
            elapsed_ms > RING_SEARCH_MS
                ? RING_SEARCH_MS : elapsed_ms;
        break;
    case CAR_GYRO_RING_WAIT_ROUTE:
        ring->control_error_norm = 0;
        ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
        ring->outer_target_millirps = 0u;
        break;
    case CAR_GYRO_RING_FAULT:
        ring->active = 1u;
        ring->control_error_norm = 0;
        ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
        ring->outer_target_millirps = 0u;
        break;
    case CAR_GYRO_RING_REARM:
    case CAR_GYRO_RING_OFF:
    default:
        ring->active = 0u;
        ring->control_error_norm = 0;
        ring->servo_target_us = 0u;
        ring->outer_target_millirps = 0u;
        break;
    }
}

void car_gyro_ring_init(car_gyro_ring_t *ring)
{
    if (ring != NULL) {
        reset_session(ring);
    }
}

void car_gyro_ring_step(car_gyro_ring_t *ring,
                        const car_gyro_ring_input_t *input)
{
    uint8_t new_sequence;

    if (ring == NULL) {
        return;
    }
    if (input == NULL) {
        latch_fault(ring, CAR_GYRO_RING_FAULT_INPUT, 0u);
        return;
    }

    /* A physical stop explicitly clears authorization/fault state and does
       not interpret the current camera packet as an entry or recovery frame. */
    if (input->run_active == 0u) {
        reset_session(ring);
        return;
    }
    if (ring->fault_latched != 0u) {
        apply_state_outputs(ring, input);
        return;
    }
    if (input->enabled == 0u) {
        if (ring->active != 0u) {
            latch_fault(ring, CAR_GYRO_RING_FAULT_INPUT, input->now_ms);
        } else {
            reset_session(ring);
        }
        return;
    }

    new_sequence = consume_new_sequence(ring, input);
    if (ring->active != 0u &&
        active_inputs_valid(input) == 0u) {
        uint8_t reason = input->transport_fresh == 0u
                             ? CAR_GYRO_RING_FAULT_TRANSPORT
                             : CAR_GYRO_RING_FAULT_IMU;
        latch_fault(ring, reason, input->now_ms);
        return;
    }

    switch (ring->state) {
    case CAR_GYRO_RING_OFF:
    case CAR_GYRO_RING_REARM:
        update_recovery(ring, input, new_sequence);
        if (recovery_is_fresh(ring, input) != 0u &&
            input->camera_phase == 0u) {
            ring->allow_rearm = 1u;
            ring->state = CAR_GYRO_RING_REARM;
            ring->phase = CAR_GYRO_RING_REARM;
        }
        if (ring->state == CAR_GYRO_RING_REARM &&
            ring->allow_rearm != 0u && new_sequence != 0u &&
            input->camera_fresh != 0u && input->camera_phase == 1u &&
            (input->camera_side == -1 || input->camera_side == 1)) {
            start_entry(ring, input);
        }
        break;

    case CAR_GYRO_RING_ENTRY:
        if ((uint32_t)(input->now_ms - ring->state_started_ms) >=
            CAR_GYRO_RING_ENTRY_LIMIT_MS) {
            latch_fault(ring, CAR_GYRO_RING_FAULT_TIMEOUT, input->now_ms);
            return;
        }
        if ((uint32_t)(input->now_ms - ring->state_started_ms) >= CAR_GYRO_RING_APPROACH_MS &&
            new_sequence != 0u && input->camera_fresh != 0u &&
            input->camera_phase == 2u &&
            input->camera_side == ring->entry_side) {
            ring->yaw_origin_mdeg = input->yaw_mdeg;
            ring->progress_mdeg = 0;
            clear_integrator(ring);
            set_state(ring, CAR_GYRO_RING_ORBIT, input->now_ms);
        }
        break;

    case CAR_GYRO_RING_ORBIT:
        ring->progress_mdeg = progress_for_side(
            ring->entry_side, input->yaw_mdeg, ring->yaw_origin_mdeg);
        if (ring->progress_mdeg >= RING_PROGRESS) {
            set_state(ring, CAR_GYRO_RING_CENTER, input->now_ms);
            ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
            ring->control_error_norm = 0;
        } else if ((uint32_t)(input->now_ms - ring->state_started_ms) >=
                   CAR_GYRO_RING_ORBIT_LIMIT_MS) {
            latch_fault(ring, CAR_GYRO_RING_FAULT_TIMEOUT, input->now_ms);
            return;
        }
        break;

    case CAR_GYRO_RING_CENTER:
        if ((uint32_t)(input->now_ms - ring->state_started_ms) >=
            CAR_GYRO_RING_CENTER_LIMIT_MS) {
            latch_fault(ring, CAR_GYRO_RING_FAULT_TIMEOUT, input->now_ms);
            return;
        }
        if (input->applied_servo_us == CAR_GYRO_RING_CENTER_SERVO_US) {
            ring->control_error_norm = 0;
            ring->servo_target_us = CAR_GYRO_RING_CENTER_SERVO_US;
            ring->outer_target_millirps =
                CAR_GYRO_RING_OUTER_TARGET_MILLIRPS;
            set_state(ring, CAR_GYRO_RING_EXIT_STRAIGHT, input->now_ms);
        }
        break;

    case CAR_GYRO_RING_EXIT_STRAIGHT:
        update_recovery(ring, input, new_sequence);
        if ((uint32_t)(input->now_ms - ring->state_started_ms) >=
            RING_EXIT_MS) {
            if (recovery_is_fresh(ring, input) != 0u &&
                input->camera_phase == 0u) {
                finish_ring(ring, input->now_ms);
            } else {
                reset_recovery(ring);
                set_state(ring, CAR_GYRO_RING_SEARCH, input->now_ms);
            }
        }
        break;

    case CAR_GYRO_RING_SEARCH:
        if ((uint32_t)(input->now_ms - ring->state_started_ms) >=
            RING_SEARCH_MS) {
            reset_recovery(ring);
            set_state(ring, CAR_GYRO_RING_WAIT_ROUTE, input->now_ms);
            break;
        }
        update_recovery(ring, input, new_sequence);
        if (recovery_is_fresh(ring, input) != 0u &&
            input->camera_phase == 0u) {
            finish_ring(ring, input->now_ms);
        }
        break;

    case CAR_GYRO_RING_WAIT_ROUTE:
        update_recovery(ring, input, new_sequence);
        if (recovery_is_fresh(ring, input) != 0u &&
            input->camera_phase == 0u) {
            finish_ring(ring, input->now_ms);
        }
        break;

    case CAR_GYRO_RING_FAULT:
    default:
        latch_fault(ring, ring->fault == 0u
                                ? CAR_GYRO_RING_FAULT_INPUT : ring->fault,
                    input->now_ms);
        return;
    }

    if (ring->active != 0u && active_inputs_valid(input) == 0u) {
        uint8_t reason = input->transport_fresh == 0u
                             ? CAR_GYRO_RING_FAULT_TRANSPORT
                             : CAR_GYRO_RING_FAULT_IMU;
        latch_fault(ring, reason, input->now_ms);
        return;
    }
    apply_state_outputs(ring, input);
}
