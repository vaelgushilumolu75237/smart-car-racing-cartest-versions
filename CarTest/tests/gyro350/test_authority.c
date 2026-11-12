#include "commissioning.h"
#include "vision_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    car_config_t config;
    car_control_t control;
    uint32_t now_ms;
    uint16_t sequence;
} authority_fixture_t;

static void check(uint8_t condition, const char *message)
{
    if (condition == 0u) {
        fprintf(stderr, "authority test failed: %s\n", message);
        assert(condition != 0u);
    }
}

static void fixture_init(authority_fixture_t *fixture)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    car_config_init_application(&fixture->config, 0u, 0u, 1u, 1u);
    car_control_init(&fixture->control, &fixture->config, 100u, 1u, 0u);

    /* Model an already-started Follow session with a valid reverse lease and
       software-applied outputs. The test never starts hardware outputs. */
    fixture->now_ms = 100u;
    fixture->sequence = 100u;
    fixture->control.startup_release_seen = 1u;
    fixture->control.startup_frame_gate_pending = 0u;
    fixture->control.mode = CAR_MODE_FOLLOW_RUNNING;
    fixture->control.follow_session_active = 1u;
    fixture->control.manual_run_request = 1u;
    fixture->control.reverse_control_keep_seen = 1u;
    fixture->control.reverse_control_fresh_frames =
        CAR_CONTROL_RECOVERY_REQUIRED_FRAMES;
    fixture->control.applied_servo_frame_ready = 1u;
    fixture->control.applied_servo_output_active = 1u;
    fixture->control.applied_servo_pulse_us = fixture->config.servo_center_us;
    fixture->control.applied_motor_left_permille = 400u;
    fixture->control.applied_motor_right_permille = 400u;
    fixture->control.follow_output_duty_permille = 400u;
    fixture->control.imu_sample_us = fixture->now_ms * 1000u;
}

static void send_frame(authority_fixture_t *fixture,
                       uint8_t valid,
                       uint8_t protocol_version,
                       uint8_t ring_phase,
                       int8_t ring_side,
                       uint8_t ordinary_route,
                       uint32_t now_ms)
{
    ++fixture->sequence;
    fixture->now_ms = now_ms;
    fixture->control.imu_sample_us = now_ms * 1000u;
    car_control_on_vision_frame_at_with_ring(
        &fixture->control, &fixture->config, fixture->sequence,
        valid != 0u ? VP_FLAG_VALID : 0u, 90u, 0, VP_CURVE_PHASE_STRAIGHT,
        1000u, protocol_version, fixture->config.required_camera_profile_ack,
        now_ms, now_ms, 1u, ring_phase, ring_side, ordinary_route);
}

#if CARTEST_TRACK_PROFILE >= 2
static void seed_ordinary_route(authority_fixture_t *fixture)
{
    uint32_t now_ms = fixture->now_ms;
    send_frame(fixture, 1u, VP_VERSION_V4, 0u, 0, 1u, now_ms + 10u);
    send_frame(fixture, 1u, VP_VERSION_V4, 0u, 0, 1u, now_ms + 20u);
    send_frame(fixture, 1u, VP_VERSION_V4, 0u, 0, 1u, now_ms + 30u);
}

static void make_gyro_ready(authority_fixture_t *fixture)
{
    fixture->control.imu_ready = 1u;
    fixture->control.imu_fault = 0u;
    fixture->control.imu_sample_us = fixture->now_ms * 1000u;
    fixture->control.imu_z_mdps = 0;
}
#endif

static void test_profile_gate(void)
{
    authority_fixture_t fixture;

    fixture_init(&fixture);
#if CARTEST_TRACK_PROFILE >= 2
    check(fixture.config.gyro_enabled == 1u,
          "B/C profile enables gyro authority");
    check(car_control_follow_authority_ready(&fixture.control,
                                              &fixture.config,
                                              fixture.now_ms) == 0u,
          "B/C rejects authority before healthy IMU and ordinary V4 route");

    make_gyro_ready(&fixture);
    seed_ordinary_route(&fixture);
    check(car_control_follow_authority_ready(&fixture.control,
                                              &fixture.config,
                                              fixture.now_ms) != 0u,
          "B/C accepts healthy IMU and fresh ordinary V4 route");

    fixture_init(&fixture);
    make_gyro_ready(&fixture);
    send_frame(&fixture, 1u, VP_VERSION_V3, 0u, 0, 1u, 110u);
    check(fixture.control.fault == CAR_FAULT_PROTOCOL,
          "B/C rejects V3 camera frame");
#else
    check(fixture.config.gyro_enabled == 0u,
          "A profile keeps gyro authority disabled");
    send_frame(&fixture, 1u, VP_VERSION_V4, 0u, 0, 1u, 110u);
    check(fixture.control.fault != CAR_FAULT_IMU &&
              fixture.control.fault != CAR_FAULT_GYRO_RING,
          "A accepts ordinary V4 without IMU");
#endif
}

#if CARTEST_GYRO350_ENABLED && CARTEST_TRACK_PROFILE >= 2
static void start_ring(authority_fixture_t *fixture)
{
    make_gyro_ready(fixture);
    seed_ordinary_route(fixture);
    send_frame(fixture, 1u, VP_VERSION_V4, 1u, -1, 0u,
               fixture->now_ms + 10u);
}

static void test_ring_authority_and_invalid_route(void)
{
#if CARTEST_TRACK_PROFILE >= 2
    authority_fixture_t fixture;

    fixture_init(&fixture);
    start_ring(&fixture);
    check(fixture.control.gyro.active != 0u,
          "fresh V4 entry evidence starts gyro authority");
    check(fixture.control.gyro.phase == CAR_GYRO_RING_ENTRY,
          "entry evidence selects gyro ENTRY phase");
    check(fixture.control.gyro.outer_target_millirps ==
              CAR_GYRO_RING_OUTER_TARGET_MILLIRPS,
          "entry uses profile outer wheel target");
    check(fixture.control.vision_hold_active == 0u &&
              fixture.control.vision_hold_entries == 0u,
          "entry does not enter visual hold");

    send_frame(&fixture, 0u, VP_VERSION_V4, 1u, -1, 0u,
               fixture.now_ms + 10u);
    check(fixture.control.gyro.active != 0u &&
              fixture.control.vision_hold_active == 0u &&
              fixture.control.vision_hold_entries == 0u,
          "fresh invalid route keeps gyro authority without visual hold");
#endif
}

static void test_invalid_image_cannot_advance_ring_phase(void)
{
#if CARTEST_TRACK_PROFILE >= 2
    authority_fixture_t fixture;
    fixture_init(&fixture);
    make_gyro_ready(&fixture);
    seed_ordinary_route(&fixture);
    send_frame(&fixture, 0u, VP_VERSION_V4, 1u, -1, 0u,
               fixture.now_ms + 10u);
    check(fixture.control.gyro.active == 0u,
          "invalid entry event cannot start gyro authority");
    send_frame(&fixture, 1u, VP_VERSION_V4, 1u, -1, 0u,
               fixture.now_ms + 10u);
    check(fixture.control.gyro.phase == CAR_GYRO_RING_ENTRY,
          "fresh valid entry can follow rejected invalid event");
    send_frame(&fixture, 0u, VP_VERSION_V4, 2u, -1, 0u,
               fixture.now_ms + 10u);
    check(fixture.control.gyro.phase == CAR_GYRO_RING_ENTRY,
          "invalid orbit event cannot reset yaw origin");
    send_frame(&fixture, 1u, VP_VERSION_V4, 2u, -1, 0u,
               fixture.now_ms + 10u);
    check(fixture.control.gyro.phase == CAR_GYRO_RING_ORBIT,
          "fresh valid orbit event advances entry");
#endif
}

static void test_manual_stop_and_faults(void)
{
#if CARTEST_TRACK_PROFILE >= 2
    authority_fixture_t fixture;

    fixture_init(&fixture);
    start_ring(&fixture);
    car_control_note_switch_release_isr(&fixture.control);
    fixture.now_ms += 1u;
    fixture.control.imu_sample_us = fixture.now_ms * 1000u;
    car_control_update_gyro(&fixture.control, &fixture.config,
                            fixture.now_ms);
    check(fixture.control.gyro.active == 0u &&
              fixture.control.gyro.phase == CAR_GYRO_RING_OFF,
          "PE7 release revokes gyro authority");

    fixture_init(&fixture);
    start_ring(&fixture);
    fixture.now_ms += fixture.config.vision_timeout_ms + 1u;
    fixture.control.imu_sample_us = fixture.now_ms * 1000u;
    car_control_update_gyro(&fixture.control, &fixture.config,
                            fixture.now_ms);
    check(fixture.control.gyro.fault == CAR_GYRO_RING_FAULT_TRANSPORT &&
              fixture.control.fault == CAR_FAULT_GYRO_RING,
          "camera transport outage faults and stops gyro authority");

    fixture_init(&fixture);
    start_ring(&fixture);
    fixture.control.imu_fault = 1u;
    fixture.now_ms += 1u;
    fixture.control.imu_sample_us = fixture.now_ms * 1000u;
    car_control_update_gyro(&fixture.control, &fixture.config,
                            fixture.now_ms);
    check(fixture.control.gyro.fault == CAR_GYRO_RING_FAULT_IMU &&
              fixture.control.fault == CAR_FAULT_IMU,
          "IMU fault stops gyro authority");
#endif
}
#endif

int main(void)
{
    test_profile_gate();
#if CARTEST_GYRO350_ENABLED && CARTEST_TRACK_PROFILE >= 2
    test_ring_authority_and_invalid_route();
    test_invalid_image_cannot_advance_ring_phase();
    test_manual_stop_and_faults();
#endif
    (void)puts("gyro350 authority tests: PASS");
    return 0;
}
