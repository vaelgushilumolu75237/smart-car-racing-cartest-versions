#include "ackermann_control.h"
#include "ackermann_snapshot.h"
#include "follow_incremental_pi.h"
#include "servo_safety.h"

#include <stdint.h>
#include <stdio.h>

static unsigned checks;
static unsigned failures;

#define CHECK(condition, message) do { \
    ++checks; \
    if (!(condition)) { \
        ++failures; \
        (void)printf("FAIL: %s (line %u)\n", message, (unsigned)__LINE__); \
    } \
} while (0)

#if defined(_MSC_VER)
__declspec(align(32)) static uint8_t snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE];
#else
_Alignas(32) static uint8_t snapshot[CAR_ACKERMANN_SNAPSHOT_SIZE];
#endif

static uint16_t expected_fastest_target(void)
{
#if CARTEST_TRACK_PROFILE == 1 || CARTEST_TRACK_PROFILE == 3
    return 5600u;
#else
    return 4800u;
#endif
}

static uint16_t expected_slowest_target(void)
{
#if CARTEST_TRACK_PROFILE == 1 || CARTEST_TRACK_PROFILE == 3
    return 4400u;
#else
    return 4000u;
#endif
}

static void test_ackermann_profile_targets(void)
{
    car_ackermann_control_state_t straight_state;
    car_ackermann_control_state_t ring_state;
    car_ackermann_control_state_t curve_state;
    car_ackermann_control_output_t straight;
    car_ackermann_control_output_t ring;
    car_ackermann_control_output_t curve;
    const uint16_t fastest = expected_fastest_target();
    const uint16_t slowest = expected_slowest_target();

    CHECK(CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS == fastest,
          "selected profile exposes its exact straight target");
    CHECK(CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS == slowest,
          "selected profile exposes its exact full-slowdown target");
    CHECK(CAR_FOLLOW_INCREMENTAL_PI_TARGET_MILLIRPS == fastest,
          "incremental PI per-wheel cap follows the selected profile");
    CHECK(CAR_FOLLOW_SPEED_PI_MAX_TARGET_MILLIRPS == fastest,
          "positional PI diagnostic cap follows the selected profile");
    CHECK(CAR_FOLLOW_INCREMENTAL_PI_OUTPUT_CAP_PERMILLE == 1000u,
          "per-wheel output cap remains 100 percent");
    CHECK(CAR_FOLLOW_INCREMENTAL_PI_KP_Q16 == 13107u &&
              CAR_FOLLOW_INCREMENTAL_PI_KI_Q16 == 26214u &&
              CAR_FOLLOW_INCREMENTAL_PI_TRACKING_TIME_MS == 200u,
          "incremental PI gains and 200 ms tracking remain unchanged");
    CHECK(CARTEST_FOLLOW_SERVO_SLEW_STEP_US == 160u &&
              CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US == 160u &&
              CAR_SERVO_SAFETY_SLEW_PERIOD_MS == 20u,
          "selected coordinated profile retains 160us/20ms servo slew");

    car_ackermann_control_reset(&straight_state);
    CHECK(straight_state.fastest_target_millirps == fastest,
          "startup reset seeds the selected profile target");
    CHECK(car_ackermann_control_update(
              &straight_state, 10u, CAR_ACKERMANN_PHASE_STRAIGHT, 0u,
              1530u, 1530u, 1100u, 2500u, &straight) != 0u,
          "straight Ackermann target is accepted");
    CHECK(straight.fastest_target_millirps == fastest &&
              straight.left_target_millirps == fastest &&
              straight.right_target_millirps == fastest,
          "straight startup mirrors the selected per-wheel target");

    car_ackermann_control_reset(&ring_state);
    car_ackermann_control_reset(&curve_state);
    CHECK(car_ackermann_control_update(
              &ring_state, 100u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &ring) != 0u,
          "ring-like full slowdown target starts its bounded ramp");
    CHECK(car_ackermann_control_update(
              &curve_state, 100u, CAR_ACKERMANN_PHASE_CANDIDATE, 1000u,
              2209u, 1530u, 1100u, 2500u, &curve) != 0u,
          "ordinary curve full slowdown target starts its bounded ramp");
    CHECK(car_ackermann_control_update(
              &ring_state, 300u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &ring) != 0u,
          "ring-like target reaches full-request endpoint");
    CHECK(car_ackermann_control_update(
              &curve_state, 300u, CAR_ACKERMANN_PHASE_CANDIDATE, 1000u,
              2209u, 1530u, 1100u, 2500u, &curve) != 0u,
          "curve target reaches full-request endpoint");
    CHECK(ring.fastest_target_millirps == slowest &&
              curve.fastest_target_millirps == slowest &&
              ring.fastest_target_millirps == curve.fastest_target_millirps,
          "ring fastest wheel uses the same curved target, with no extra boost");
    CHECK(ring.left_target_millirps == slowest &&
              ring.right_target_millirps < ring.left_target_millirps,
          "right turn mirrors the slower inner wheel only");

    CHECK(car_ackermann_control_update(
              &ring_state, 300u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              1229u, 1530u, 1100u, 2500u, &ring) != 0u,
          "left turn target uses the same selected profile ceiling");
    CHECK(ring.right_target_millirps == slowest &&
              ring.left_target_millirps < ring.right_target_millirps,
          "left turn mirrors the slower inner wheel only");
}

static void test_pi_override_and_snapshot_version(void)
{
    car_follow_incremental_pi_state_t pi;
    car_follow_speed_pi_input_t input = {0};
    car_ackermann_snapshot_values_t values = {0};
    const uint16_t fastest = expected_fastest_target();

    car_follow_incremental_pi_reset(&pi);
    input.enabled = 1u;
    input.follow_running = 1u;
    input.startup_qualified = 1u;
    input.actual_motor_outputs = 1u;
    input.servo_ready = 1u;
    input.speed_sample_valid = 1u;
    input.new_speed_sample = 1u;
    input.sample_ms = 20u;
    input.sample_elapsed_ms = 20u;
    input.measured_left_millirps = 1000;
    input.measured_right_millirps = 1100;
    input.nominal_left_permille = 500u;
    input.nominal_right_permille = 500u;
    input.applied_left_permille = 500u;
    input.applied_right_permille = 500u;
    input.target_override_valid = 1u;
    input.target_left_millirps = fastest;
    input.target_right_millirps = fastest;
    CHECK(car_follow_incremental_pi_update(&pi, &input) ==
              CAR_FOLLOW_SPEED_PI_STATUS_RUNNING,
          "incremental PI accepts the selected per-wheel target override");
    CHECK(pi.target_left_millirps == fastest &&
              pi.target_right_millirps == fastest,
          "incremental PI retains the full selected target for each wheel");

    CHECK(sizeof(snapshot) == 64u &&
              CAR_ACKERMANN_SNAPSHOT_SIZE == 64u,
          "ackermann diagnostic snapshot layout remains 64 bytes");
#if CARTEST_TRACK_PROFILE == CAR_TRACK_PROFILE_LEGACY
    CHECK(CAR_ACKERMANN_SNAPSHOT_VERSION == 1u &&
              CAR_ACKERMANN_SNAPSHOT_MAX_TARGET_MILLIRPS == 4800u,
          "legacy snapshot remains V1 and capped at 4800");
#else
    CHECK(CAR_ACKERMANN_SNAPSHOT_VERSION == 2u &&
              CAR_ACKERMANN_SNAPSHOT_MAX_TARGET_MILLIRPS == 5600u,
          "paired profiles use V2 while preserving the 64-byte layout");
#endif

    values.timestamp_ms = 20u;
    values.vision_sequence = 1u;
    values.curve_phase = CAR_ACKERMANN_PHASE_STRAIGHT;
    values.ratio_q15 = CAR_ACKERMANN_RATIO_ONE_Q15;
    values.servo_pulse_us = 1530u;
    values.fastest_target_millirps = fastest;
    values.left_target_millirps = fastest;
    values.right_target_millirps = fastest;
    values.left_measured_millirps = 1000;
    values.right_measured_millirps = 1100;
    values.left_duty_permille = 500u;
    values.right_duty_permille = 500u;
    car_ackermann_snapshot_reset(snapshot);
    car_ackermann_snapshot_publish(snapshot, &values);
    CHECK(car_ackermann_snapshot_validate(snapshot) != 0u,
          "selected profile target serializes in a valid snapshot");
    CHECK(snapshot[4] == CAR_ACKERMANN_SNAPSHOT_VERSION &&
              snapshot[5] == CAR_ACKERMANN_SNAPSHOT_SIZE,
          "snapshot version and fixed-size header remain explicit");

    values.fastest_target_millirps = 5600;
    values.left_target_millirps = 5600;
    values.right_target_millirps = 5600;
    car_ackermann_snapshot_reset(snapshot);
    car_ackermann_snapshot_publish(snapshot, &values);
#if CARTEST_TRACK_PROFILE == CAR_TRACK_PROFILE_LEGACY
    CHECK(car_ackermann_snapshot_validate(snapshot) == 0u,
          "legacy snapshot rejects values above its 4800 cap");
#else
    CHECK(car_ackermann_snapshot_validate(snapshot) != 0u,
          "new-version snapshot accepts its 5600 diagnostic ceiling");
#endif
}

int main(void)
{
    test_ackermann_profile_targets();
    test_pi_override_and_snapshot_version();
    (void)printf("TRACK_PROFILE_TARGETS profile=%u fastest=%u slowest=%u snapshot_v=%u checks=%u failures=%u\n",
                 (unsigned)CARTEST_TRACK_PROFILE,
                 (unsigned)CAR_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS,
                 (unsigned)CAR_ACKERMANN_SLOWEST_REQUEST_MILLIRPS,
                 (unsigned)CAR_ACKERMANN_SNAPSHOT_VERSION,
                 checks, failures);
    return failures == 0u ? 0 : 1;
}
