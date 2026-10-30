#include "ackermann_control.h"
#include "ackermann_snapshot.h"
#include "follow_incremental_pi.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

static unsigned checks;
static unsigned failures;

static uint16_t read_le16(const uint8_t *bytes, unsigned offset)
{
    return (uint16_t)((uint16_t)bytes[offset] |
                      ((uint16_t)bytes[offset + 1u] << 8));
}

static uint32_t read_le32(const uint8_t *bytes, unsigned offset)
{
    return (uint32_t)bytes[offset] |
           ((uint32_t)bytes[offset + 1u] << 8) |
           ((uint32_t)bytes[offset + 2u] << 16) |
           ((uint32_t)bytes[offset + 3u] << 24);
}

static void write_le32(uint8_t *bytes, unsigned offset, uint32_t value)
{
    bytes[offset] = (uint8_t)value;
    bytes[offset + 1u] = (uint8_t)(value >> 8);
    bytes[offset + 2u] = (uint8_t)(value >> 16);
    bytes[offset + 3u] = (uint8_t)(value >> 24);
}

static uint16_t expected_ratio_q15(uint16_t angle_cdeg)
{
    const double pi = 3.14159265358979323846;
    const double angle = (double)angle_cdeg * pi / 18000.0;
    if (angle_cdeg == 0u) {
        return CAR_ACKERMANN_RATIO_ONE_Q15;
    }
    const double tangent = tan(angle);
    const double radius_mm = 143.5 / tangent;
    const double ratio = (radius_mm - 83.5) / (radius_mm + 83.5);
    return (uint16_t)floor(ratio * 32768.0 + 0.5);
}

static void test_geometry_and_tangent_lut(void)
{
    uint16_t angle;
    int16_t model_angle;
    uint16_t ratio;
    uint16_t max_error = 0u;

    CHECK(car_ackermann_model_geometry(30000u, 30000u, 10000u, 50000u,
                                       &model_angle, &ratio) != 0u,
          "center geometry accepted");
    CHECK(model_angle == 0, "center angle is zero");
    CHECK(ratio == CAR_ACKERMANN_RATIO_ONE_Q15,
          "zero angle keeps both wheels at the same speed");

    for (angle = 0u; angle <= 2000u; ++angle) {
        uint16_t expected;
        uint16_t error;
        const uint16_t pulse = (uint16_t)(30000u + angle * 10u);
        const uint16_t left_pulse = (uint16_t)(30000u - angle * 10u);
        CHECK(car_ackermann_model_geometry(
                  pulse, 30000u, 10000u, 50000u,
                  &model_angle, &ratio) != 0u,
              "right geometry accepted throughout 0..20 degrees");
        CHECK(model_angle == (int16_t)angle,
              "right PWM maps linearly to the requested centidegree");
        expected = expected_ratio_q15(angle);
        error = ratio > expected ? (uint16_t)(ratio - expected)
                                 : (uint16_t)(expected - ratio);
        if (error > max_error) max_error = error;
        CHECK(error <= 1u,
              "interpolated tangent LUT matches math reference within 1 Q15");

        CHECK(car_ackermann_model_geometry(
                  left_pulse, 30000u, 10000u, 50000u,
                  &model_angle, &ratio) != 0u,
              "left geometry accepted throughout 0..20 degrees");
        CHECK(model_angle == -(int16_t)angle,
              "left PWM maps linearly to the requested centidegree");
    }

    CHECK(car_ackermann_model_geometry(2209u, 1530u, 1100u, 2500u,
                                       &model_angle, &ratio) != 0u,
          "asymmetric right steering geometry accepted");
    CHECK(model_angle == 1400,
          "1530 to 2500 full span maps 679 us to 14 degrees");
    CHECK(ratio == expected_ratio_q15(1400u),
          "14 degree right turn ratio matches model");
    CHECK(car_ackermann_model_geometry(1229u, 1530u, 1100u, 2500u,
                                       &model_angle, &ratio) != 0u,
          "asymmetric left steering geometry accepted");
    CHECK(model_angle == -1400,
          "1530 to 1100 full span maps 301 us to 14 degrees");
    CHECK(ratio == expected_ratio_q15(1400u),
          "left/right angle sign does not change the Ackermann ratio");
    CHECK(car_ackermann_model_geometry(2500u, 1530u, 1100u, 2500u,
                                       &model_angle, &ratio) != 0u &&
              model_angle == 2000,
          "right commercial endpoint maps to 20 degrees");
    CHECK(car_ackermann_model_geometry(1100u, 1530u, 1100u, 2500u,
                                       &model_angle, &ratio) != 0u &&
              model_angle == -2000,
          "left commercial endpoint maps to 20 degrees");
    CHECK(max_error <= 1u,
          "all tangent-reference comparisons stay within one Q15 step");
}

static void test_turn_sides_and_request_ramps(void)
{
    car_ackermann_control_state_t state;
    car_ackermann_control_output_t output;
    uint32_t start = 50u;

    car_ackermann_control_reset(&state);
    CHECK(car_ackermann_control_update(
              &state, start, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &output) != 0u,
          "initial full slowdown request accepted");
    CHECK(output.fastest_target_millirps == 4800,
          "down-ramp begins from 4800 milli-RPS without a step");
    CHECK(output.left_target_millirps > output.right_target_millirps,
          "right steering turn slows the right inner wheel");

    CHECK(car_ackermann_control_update(
              &state, start + 50u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4600,
          "repeated request advances 200 ms down-ramp linearly");
    CHECK(car_ackermann_control_update(
              &state, start + 100u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4400,
          "repeated request does not restart the down-ramp");
    CHECK(car_ackermann_control_update(
              &state, start + 150u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4200,
          "down-ramp reaches 4200 milli-RPS at 150 ms");
    CHECK(car_ackermann_control_update(
              &state, start + 200u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              2209u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4000,
          "full slowdown request reaches 4000 milli-RPS at 200 ms");

    CHECK(car_ackermann_control_update(
              &state, start + 200u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1229u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4000,
          "request release begins from the current target");
    CHECK(output.right_target_millirps > output.left_target_millirps,
          "left steering turn slows the left inner wheel");
    CHECK(car_ackermann_control_update(
              &state, start + 400u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1229u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4200,
          "release rises linearly over the 800 ms ramp");
    CHECK(car_ackermann_control_update(
              &state, start + 800u, CAR_ACKERMANN_PHASE_STRAIGHT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4600,
          "release reaches 4600 at 600 ms elapsed");
    CHECK(output.left_target_millirps == output.right_target_millirps,
          "zero steering remains equal-speed despite an active ramp");
    CHECK(car_ackermann_control_update(
              &state, start + 1000u, CAR_ACKERMANN_PHASE_STRAIGHT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4800,
          "release reaches 4800 milli-RPS at 800 ms");

    car_ackermann_control_reset(&state);
    start = 0xFFFFFF80u;
    CHECK(car_ackermann_control_update(
              &state, start, CAR_ACKERMANN_PHASE_CANDIDATE, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u,
          "wraparound down-ramp starts");
    CHECK(car_ackermann_control_update(
              &state, start + 100u, CAR_ACKERMANN_PHASE_CANDIDATE, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4400,
          "wrap-safe elapsed time advances the ramp across uint32 rollover");
    CHECK(car_ackermann_control_update(
              &state, start + 200u, CAR_ACKERMANN_PHASE_CANDIDATE, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4000,
          "wrap-safe down-ramp completes at 200 ms");

    car_ackermann_control_reset(&state);
    start = 700u;
    CHECK(car_ackermann_control_update(
              &state, start, CAR_ACKERMANN_PHASE_TURN, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4800,
          "changed-request test starts at the fastest target");
    CHECK(car_ackermann_control_update(
              &state, start + 50u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4600,
          "changed-request test reaches an intermediate target");
    CHECK(car_ackermann_control_update(
              &state, start + 50u, CAR_ACKERMANN_PHASE_TURN, 500u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4600,
          "request change starts continuously from the current ramp value");
    CHECK(car_ackermann_control_update(
              &state, start + 150u, CAR_ACKERMANN_PHASE_TURN, 500u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4500,
          "partial slowdown continues linearly from the changed request");
    CHECK(car_ackermann_control_update(
              &state, start + 250u, CAR_ACKERMANN_PHASE_TURN, 500u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4400,
          "partial slowdown reaches its mapped target after 200 ms");
    CHECK(car_ackermann_control_update(
              &state, start + 250u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4400,
          "release starts without a speed-target discontinuity");
    CHECK(car_ackermann_control_update(
              &state, start + 650u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4600,
          "partial request release reaches its midpoint at 400 ms");
    CHECK(car_ackermann_control_update(
              &state, start + 1050u, CAR_ACKERMANN_PHASE_STRAIGHT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4800,
          "partial request release reaches the maximum after 800 ms");

    car_ackermann_control_reset(&state);
    start = 0xFFFFFF80u;
    CHECK(car_ackermann_control_update(
              &state, start, CAR_ACKERMANN_PHASE_TURN, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u,
          "wrap-safe release test starts the down-ramp");
    CHECK(car_ackermann_control_update(
              &state, start + 200u, CAR_ACKERMANN_PHASE_TURN, 1000u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4000,
          "wrap-safe release test reaches the slow target");
    CHECK(car_ackermann_control_update(
              &state, start + 200u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4000,
          "wrap-safe release ramp begins continuously");
    CHECK(car_ackermann_control_update(
              &state, start + 600u, CAR_ACKERMANN_PHASE_EXIT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4400,
          "wrap-safe release ramp reaches its midpoint");
    CHECK(car_ackermann_control_update(
              &state, start + 1000u, CAR_ACKERMANN_PHASE_STRAIGHT, 0u,
              1530u, 1530u, 1100u, 2500u, &output) != 0u &&
              output.fastest_target_millirps == 4800,
          "wrap-safe release ramp completes across uint32 rollover");
}

static void test_incremental_pi_hold_rebase(void)
{
    car_follow_incremental_pi_state_t state;
    car_follow_speed_pi_input_t input = {0};
    const int64_t before_hold_left = 500 * INT64_C(65536);
    const int64_t before_hold_right = 500 * INT64_C(65536);

    car_follow_incremental_pi_reset(&state);
    input.enabled = 1u;
    input.follow_running = 1u;
    input.startup_qualified = 1u;
    input.actual_motor_outputs = 1u;
    input.servo_ready = 1u;
    input.speed_sample_valid = 1u;
    input.new_speed_sample = 1u;
    input.sample_ms = 10u;
    input.sample_age_ms = 0u;
    input.sample_elapsed_ms = 10u;
    input.measured_left_millirps = 1000;
    input.measured_right_millirps = 1000;
    input.nominal_left_permille = 500u;
    input.nominal_right_permille = 500u;
    input.applied_left_permille = 500u;
    input.applied_right_permille = 500u;
    input.target_override_valid = 1u;
    input.target_left_millirps = 4800;
    input.target_right_millirps = 3600;
    CHECK(car_follow_incremental_pi_update(&state, &input) ==
              CAR_FOLLOW_SPEED_PI_STATUS_RUNNING,
          "PI target override arms on the first qualified sample");
    CHECK(state.target_left_millirps == 4800 &&
              state.target_right_millirps == 3600,
          "coordinated target pair is accepted independently of feedforward");
    CHECK(state.left.accumulator_q16 == before_hold_left &&
              state.right.accumulator_q16 == before_hold_right,
          "first sample seeds the PI accumulator from actual duty");

    input.vision_hold = 1u;
    input.new_speed_sample = 0u;
    input.target_left_millirps = 0;
    input.target_right_millirps = 0;
    CHECK(car_follow_incremental_pi_update(&state, &input) ==
              CAR_FOLLOW_SPEED_PI_STATUS_HOLD,
          "vision hold takes priority over a changed Ackermann target");
    CHECK(state.left.accumulator_q16 == before_hold_left &&
              state.right.accumulator_q16 == before_hold_right &&
              state.target_left_millirps == 4800 &&
              state.target_right_millirps == 3600,
          "hold changes neither PI accumulators nor wheel targets");

    input.vision_hold = 0u;
    input.new_speed_sample = 1u;
    input.sample_ms = 20u;
    input.sample_elapsed_ms = 10u;
    input.target_left_millirps = 4400;
    input.target_right_millirps = 3300;
    input.applied_left_permille = 430u;
    input.applied_right_permille = 460u;
    CHECK(car_follow_incremental_pi_update(&state, &input) ==
              CAR_FOLLOW_SPEED_PI_STATUS_RUNNING,
          "first valid post-hold sample rebases without a PI step");
    CHECK(state.proposal_left_permille == 430u &&
              state.proposal_right_permille == 460u,
          "recovery starts at the actual applied motor outputs");
    CHECK(state.left.accumulator_q16 == 430 * INT64_C(65536) &&
              state.right.accumulator_q16 == 460 * INT64_C(65536),
          "post-hold integrators are seeded from actual outputs");
    CHECK(state.target_left_millirps == 4400 &&
              state.target_right_millirps == 3300,
          "recovery adopts the fresh coordinated target pair");
}

static void test_snapshot_crc_generation_and_freeze(void)
{
    car_ackermann_snapshot_values_t values = {0};
    uint8_t unchanged[CAR_ACKERMANN_SNAPSHOT_SIZE];
    const uint8_t crc_vector[] = "123456789";

    CHECK(car_ackermann_snapshot_crc32(crc_vector, 9u) == 0xCBF43926u,
          "snapshot CRC matches CRC-32/ISO-HDLC check vector");
    car_ackermann_snapshot_reset(snapshot);
    CHECK(car_ackermann_snapshot_validate(snapshot) == 0u,
          "zeroed snapshot has no valid sample");

    values.flags = CAR_ACKERMANN_SNAPSHOT_FLAG_COORD_ENABLED |
                   CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE |
                   CAR_ACKERMANN_SNAPSHOT_FLAG_REQUEST_ACTIVE;
    values.timestamp_ms = 123456u;
    values.vision_sequence = 65535u;
    values.curve_phase = CAR_ACKERMANN_PHASE_TURN;
    values.slowdown_request_permille = 1000u;
    values.model_angle_cdeg = 1400;
    values.ratio_q15 = expected_ratio_q15(1400u);
    values.applied_pwm_delta_us = 679;
    values.servo_pulse_us = 2209u;
    values.fastest_target_millirps = 4000;
    values.left_target_millirps = 4000;
    values.right_target_millirps = 2984;
    values.left_measured_millirps = 1000;
    values.right_measured_millirps = 900;
    values.left_duty_permille = 450u;
    values.right_duty_permille = 420u;
    values.run_state = 10u;
    values.fault_reason = 0u;
    car_ackermann_snapshot_publish(snapshot, &values);
    CHECK(car_ackermann_snapshot_validate(snapshot) != 0u,
          "active snapshot validates after its first publication");
    CHECK(read_le32(snapshot, 8u) == 2u,
          "first snapshot publishes an even generation");
    CHECK(snapshot[0] == 'A' && snapshot[1] == 'C' &&
              snapshot[2] == 'K' && snapshot[3] == '1' &&
              snapshot[4] == 1u && snapshot[5] == 64u,
          "ACK1 version 1 header is explicitly serialized");
    CHECK(read_le16(snapshot, 20u) == 1000u &&
              read_le16(snapshot, 30u) == 2209u,
          "snapshot request and pulse offsets are little-endian");

    values.timestamp_ms += 10u;
    car_ackermann_snapshot_publish(snapshot, &values);
    CHECK(read_le32(snapshot, 8u) == 4u &&
              car_ackermann_snapshot_validate(snapshot) != 0u,
          "later publication advances stable generation by two");

    write_le32(snapshot, 8u, 0xFFFFFFFEu);
    values.flags = CAR_ACKERMANN_SNAPSHOT_FLAG_COORD_ENABLED |
                   CAR_ACKERMANN_SNAPSHOT_FLAG_REQUEST_ACTIVE |
                   CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN;
    values.timestamp_ms += 1u;
    values.run_state = 8u;
    values.fault_reason = 13u;
    car_ackermann_snapshot_publish(snapshot, &values);
    CHECK(read_le32(snapshot, 8u) == 0u &&
              car_ackermann_snapshot_validate(snapshot) != 0u,
          "generation wrap to even zero remains a valid frozen snapshot");
    CHECK((read_le16(snapshot, 6u) &
           CAR_ACKERMANN_SNAPSHOT_FLAG_FROZEN) != 0u &&
              (read_le16(snapshot, 6u) &
               CAR_ACKERMANN_SNAPSHOT_FLAG_SESSION_ACTIVE) == 0u,
          "frozen snapshot closes the active session flag");

    (void)memcpy(unchanged, snapshot, sizeof(unchanged));
    values.model_angle_cdeg = 2001;
    car_ackermann_snapshot_publish(snapshot, &values);
    CHECK(memcmp(snapshot, unchanged, sizeof(unchanged)) == 0,
          "invalid snapshot data leaves the last stable record untouched");

    snapshot[60] ^= 1u;
    CHECK(car_ackermann_snapshot_validate(snapshot) == 0u,
          "CRC corruption is rejected");
    (void)memcpy(snapshot, unchanged, sizeof(snapshot));
    snapshot[8] |= 1u;
    CHECK(car_ackermann_snapshot_validate(snapshot) == 0u,
          "odd generation is rejected as an in-progress record");
}

int main(void)
{
    test_geometry_and_tangent_lut();
    test_turn_sides_and_request_ramps();
    test_incremental_pi_hold_rebase();
    test_snapshot_crc_generation_and_freeze();
    (void)printf("Ackermann control host checks: %u; failures: %u\n",
                 checks, failures);
    return failures == 0u ? 0 : 1;
}
