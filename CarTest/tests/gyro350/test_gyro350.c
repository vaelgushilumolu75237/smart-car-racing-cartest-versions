#include "gyro_ring.h"
#include "icm42688.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    TEST_REG_INT_STATUS = 0x2Du,
    TEST_REG_ACCEL_DATA_X1 = 0x1Fu,
    TEST_REG_DEVICE_CONFIG = 0x11u,
    TEST_REG_PWR_MGMT0 = 0x4Eu,
    TEST_REG_GYRO_CONFIG0 = 0x4Fu,
    TEST_REG_ACCEL_CONFIG0 = 0x50u,
    TEST_REG_WHO_AM_I = 0x75u,
    TEST_AGC_READY = 0x01u,
    TEST_DATA_READY = 0x08u
};

typedef struct {
    uint8_t registers[128];
    uint8_t data[12];
    uint8_t data_ready;
    uint8_t interrupt_status;
    uint8_t fail_read_reg;
    uint8_t fail_read_once;
    uint8_t fail_write_reg;
    uint8_t fail_write_once;
    uint32_t read_calls;
    uint32_t write_calls;
} fake_imu_bus_t;

static void fake_set_i16_be(uint8_t *bytes, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    bytes[0] = (uint8_t)(raw >> 8);
    bytes[1] = (uint8_t)raw;
}

static void fake_set_sample(fake_imu_bus_t *fake,
                            int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz)
{
    fake_set_i16_be(&fake->data[0], ax);
    fake_set_i16_be(&fake->data[2], ay);
    fake_set_i16_be(&fake->data[4], az);
    fake_set_i16_be(&fake->data[6], gx);
    fake_set_i16_be(&fake->data[8], gy);
    fake_set_i16_be(&fake->data[10], gz);
    fake->data_ready = 1u;
    fake->interrupt_status = TEST_DATA_READY;
}

static int fake_read(void *context, uint8_t reg, uint8_t *out, uint16_t len)
{
    fake_imu_bus_t *fake = (fake_imu_bus_t *)context;
    ++fake->read_calls;
    if (fake->fail_read_once != 0u && fake->fail_read_reg == reg) {
        fake->fail_read_once = 0u;
        return 0;
    }
    if (reg == TEST_REG_INT_STATUS && len == 1u) {
        out[0] = fake->interrupt_status;
        return 1;
    }
    if (reg == TEST_REG_ACCEL_DATA_X1 && len == sizeof(fake->data)) {
        (void)memcpy(out, fake->data, sizeof(fake->data));
        fake->data_ready = 0u;
        fake->interrupt_status = 0u;
        return 1;
    }
    if ((uint16_t)reg + len > sizeof(fake->registers)) {
        return 0;
    }
    (void)memcpy(out, &fake->registers[reg], len);
    return 1;
}

static int fake_write(void *context, uint8_t reg, uint8_t value)
{
    fake_imu_bus_t *fake = (fake_imu_bus_t *)context;
    ++fake->write_calls;
    if (fake->fail_write_once != 0u && fake->fail_write_reg == reg) {
        fake->fail_write_once = 0u;
        return 0;
    }
    if (reg >= sizeof(fake->registers)) {
        return 0;
    }
    fake->registers[reg] = value;
    if (reg == TEST_REG_DEVICE_CONFIG && (value & 1u) != 0u) {
        fake->data_ready = 0u;
        fake->interrupt_status = 0u;
    }
    return 1;
}

static car_imu_bus_t fake_bus(fake_imu_bus_t *fake)
{
    car_imu_bus_t bus;
    bus.context = fake;
    bus.read = fake_read;
    bus.write = fake_write;
    return bus;
}

static uint32_t imu_to_ready(car_imu_t *imu, fake_imu_bus_t *fake,
                             uint32_t start_us)
{
    car_imu_bus_t bus = fake_bus(fake);
    car_imu_init(imu, &bus, start_us);
    assert(imu->fault == CAR_IMU_FAULT_NONE);
    assert(fake->registers[TEST_REG_DEVICE_CONFIG] == 1u);

    car_imu_poll(imu, start_us + 1000u, 0u);
    assert(imu->ready == 0u);
    assert(fake->registers[TEST_REG_PWR_MGMT0] == 0u);

    car_imu_poll(imu, start_us + 1200u, 0u);
    assert(fake->registers[TEST_REG_GYRO_CONFIG0] == 0x07u);
    assert(fake->registers[TEST_REG_ACCEL_CONFIG0] == 0x07u);
    assert(fake->registers[TEST_REG_PWR_MGMT0] == 0x0Fu);

    car_imu_poll(imu, start_us + 46200u, 0u);
    assert(imu->ready == 1u);
    assert(imu->fault == CAR_IMU_FAULT_NONE);
    assert(imu->who_am_i_valid == 1u);
    assert(imu->who_am_i == 0x47u);
    assert(imu->who_am_i_reads >= 1u);
    return start_us + 46200u;
}

static uint32_t imu_sample(car_imu_t *imu, fake_imu_bus_t *fake,
                           uint32_t now_us, uint8_t stationary_allowed,
                           int16_t gz_raw)
{
    fake_set_sample(fake, 0, 0, 2048, 0, 0, gz_raw);
    car_imu_poll(imu, now_us, stationary_allowed);
    return now_us;
}

static void test_imu_init_calibration_and_variable_dt(void)
{
    fake_imu_bus_t fake;
    car_imu_t imu;
    uint32_t now_us;
    uint32_t index;
    uint32_t samples_before;
    int64_t yaw_before;

    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    now_us = imu_to_ready(&imu, &fake, 10000u);

    /* A stationary interruption must restart the full two-second window. */
    for (index = 0u; index < 100u; ++index) {
        now_us += 5000u;
        (void)imu_sample(&imu, &fake, now_us, 1u, 40);
    }
    assert(imu.calibrated == 0u);
    assert(imu.calibration_count == 100u);
    now_us += 5000u;
    car_imu_poll(&imu, now_us, 0u);
    assert(imu.calibration_count == 0u);

    for (index = 0u; index < 401u; ++index) {
        now_us += 5000u;
        (void)imu_sample(&imu, &fake, now_us, 1u, 40);
    }
    assert(imu.calibrated == 1u);
    assert(imu.calibration_count >= 400u);
    assert(imu.bias_z_raw_q16 > 39 * 65536);
    assert(imu.bias_z_raw_q16 < 41 * 65536);
    assert(imu.gyro_z_mdps == 0);
    assert(fake.registers[TEST_REG_GYRO_CONFIG0] == 0x07u);
    assert(fake.registers[TEST_REG_ACCEL_CONFIG0] == 0x07u);

    /* Seed the trapezoid at zero rate, then use 7 ms and 13 ms intervals. */
    now_us += 5000u;
    (void)imu_sample(&imu, &fake, now_us, 1u, 40);
    assert(imu.yaw_mdeg == 0);
    now_us += 7000u;
    (void)imu_sample(&imu, &fake, now_us, 1u, 204);
    now_us += 13000u;
    (void)imu_sample(&imu, &fake, now_us, 1u, 40);
    assert(imu.gyro_z_mdps == 0);
#if CAR_IMU_YAW_SIGN > 0
    assert(imu.yaw_mdeg >= 99 && imu.yaw_mdeg <= 101);
#else
    assert(imu.yaw_mdeg <= -99 && imu.yaw_mdeg >= -101);
#endif

    samples_before = imu.sample_count;
    yaw_before = imu.yaw_mdeg;
    fake.interrupt_status = TEST_AGC_READY;
    now_us += 5000u;
    car_imu_poll(&imu, now_us, 1u); /* AGC_RDY is not DATA_RDY */
    assert(imu.sample_count == samples_before);
    car_imu_poll(&imu, now_us + 1000u, 1u);
    assert(imu.sample_count == samples_before);
    assert(imu.yaw_mdeg == yaw_before);
}

static void test_imu_whoami_diagnostics(void)
{
    static const uint8_t mismatch_values[] = {0x00u, 0xFFu};
    fake_imu_bus_t fake;
    car_imu_bus_t bus;
    car_imu_t imu;
    uint32_t index;
    uint32_t now_us;

    /* Successful mismatches retain the actual byte and eventually latch the
       existing WHO_AM_I fault; neither value is replaced by the expected ID. */
    for (index = 0u; index < sizeof(mismatch_values); ++index) {
        (void)memset(&fake, 0, sizeof(fake));
        fake.registers[TEST_REG_WHO_AM_I] = mismatch_values[index];
        bus = fake_bus(&fake);
        car_imu_init(&imu, &bus, 1000u);
        car_imu_poll(&imu, 2000u, 0u);
        assert(imu.who_am_i_valid == 1u);
        assert(imu.who_am_i == mismatch_values[index]);
        assert(imu.who_am_i_reads == 1u);
        for (now_us = 7000u; now_us < 62000u && imu.fault == 0u;
             now_us += 5000u) {
            car_imu_poll(&imu, now_us, 0u);
        }
        assert((imu.fault & CAR_IMU_FAULT_WHOAMI) != 0u);
        assert(imu.who_am_i_valid == 1u);
        assert(imu.who_am_i == mismatch_values[index]);
        assert(imu.who_am_i_reads > 1u);
    }

    /* A successful expected-ID read is the only path that makes the raw
       diagnostic valid; init itself must not fabricate a value. */
    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    bus = fake_bus(&fake);
    car_imu_init(&imu, &bus, 1000u);
    assert(imu.who_am_i_valid == 0u);
    assert(imu.who_am_i == 0u);
    assert(imu.who_am_i_reads == 0u);
    car_imu_poll(&imu, 2000u, 0u);
    assert(imu.who_am_i_valid == 1u);
    assert(imu.who_am_i == 0x47u);
    assert(imu.who_am_i_reads == 1u);

    /* A bus failure must not turn an uninitialised local byte into a valid
       WHO_AM_I result, and the existing bus fault latch must prevent a later
       poll from fabricating a diagnostic value. */
    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    fake.fail_read_reg = TEST_REG_WHO_AM_I;
    fake.fail_read_once = 1u;
    bus = fake_bus(&fake);
    car_imu_init(&imu, &bus, 1000u);
    car_imu_poll(&imu, 2000u, 0u);
    assert(imu.who_am_i_valid == 0u);
    assert(imu.who_am_i == 0u);
    assert(imu.who_am_i_reads == 0u);
    assert((imu.fault & CAR_IMU_FAULT_BUS) != 0u);
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    car_imu_poll(&imu, 7000u, 0u);
    assert(imu.who_am_i_valid == 0u);
    assert(imu.who_am_i == 0u);
    assert(imu.who_am_i_reads == 0u);
    assert((imu.fault & CAR_IMU_FAULT_BUS) != 0u);
}

static void test_imu_faults(void)
{
    fake_imu_bus_t fake;
    car_imu_t imu;
    uint32_t now_us;
    uint32_t index;
    uint32_t count_before;

    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    now_us = imu_to_ready(&imu, &fake, 50000u);
    now_us += 5000u;
    (void)imu_sample(&imu, &fake, now_us, 0u, 0);
    count_before = imu.sample_count;

    fake.fail_read_reg = TEST_REG_INT_STATUS;
    fake.fail_read_once = 1u;
    now_us += 5000u;
    car_imu_poll(&imu, now_us, 0u);
    assert((imu.fault & CAR_IMU_FAULT_BUS) != 0u);
    assert(imu.sample_count == count_before);
    assert(imu.sample_valid == 0u);

    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    now_us = imu_to_ready(&imu, &fake, 100000u);
    for (index = 0u; index < 12u && imu.fault == 0u; ++index) {
        now_us += 5000u;
        car_imu_poll(&imu, now_us, 0u);
    }
    assert((imu.fault & CAR_IMU_FAULT_STALE) != 0u);

    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x47u;
    now_us = imu_to_ready(&imu, &fake, 200000u);
    now_us += 5000u;
    (void)imu_sample(&imu, &fake, now_us, 0u, 0);
    now_us += 30000u;
    (void)imu_sample(&imu, &fake, now_us, 0u, 200);
    assert((imu.fault & CAR_IMU_FAULT_DT) != 0u);
    assert(imu.sample_count == 1u);

    (void)memset(&fake, 0, sizeof(fake));
    fake.registers[TEST_REG_WHO_AM_I] = 0x46u;
    {
        car_imu_bus_t bus = fake_bus(&fake);
        car_imu_init(&imu, &bus, UINT32_MAX - 10000u);
    }
    for (index = 0u; index < 12u && imu.fault == 0u; ++index) {
        car_imu_poll(&imu, (uint32_t)(UINT32_MAX - 9000u + index * 5000u),
                     0u);
    }
    assert((imu.fault & CAR_IMU_FAULT_WHOAMI) != 0u);
}

typedef struct {
    car_gyro_ring_t state;
    car_gyro_ring_input_t input;
} ring_fixture_t;

static void ring_set_time(ring_fixture_t *fixture, uint32_t now_ms)
{
    fixture->input.now_ms = now_ms;
    fixture->input.imu_sample_us = now_ms * 1000u;
}

static void ring_step(ring_fixture_t *fixture)
{
    car_gyro_ring_step(&fixture->state, &fixture->input);
}

static void ring_init(ring_fixture_t *fixture, uint32_t now_ms,
                      uint16_t sequence)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    car_gyro_ring_init(&fixture->state);
    fixture->input.enabled = 1u;
    fixture->input.run_active = 1u;
    fixture->input.imu_ready = 1u;
    fixture->input.transport_fresh = 1u;
    fixture->input.camera_phase = 0u;
    fixture->input.applied_servo_us = 1530u;
    fixture->input.camera_sequence = sequence;
    fixture->input.camera_fresh = 1u;
    fixture->input.ordinary_fresh = 1u;
    ring_set_time(fixture, now_ms);
    ring_step(fixture);
}

static void ring_ordinary_frame(ring_fixture_t *fixture, uint32_t now_ms)
{
    ring_set_time(fixture, now_ms);
    ++fixture->input.camera_sequence;
    fixture->input.camera_phase = 0u;
    fixture->input.camera_side = 0;
    fixture->input.camera_fresh = 1u;
    fixture->input.ordinary_fresh = 1u;
    ring_step(fixture);
    fixture->input.camera_fresh = 0u;
    fixture->input.ordinary_fresh = 0u;
}

static void ring_trigger(ring_fixture_t *fixture, uint32_t start_ms,
                     uint16_t sequence, int8_t side)
{
    /* Three ordinary frames precede the entry marker. Keep the marker a
       strictly newer packet so the production half-range check is exercised. */
    ring_init(fixture, start_ms, (uint16_t)(sequence - 4u));
    ring_ordinary_frame(fixture, start_ms + 10u);
    ring_ordinary_frame(fixture, start_ms + 20u);
    ring_ordinary_frame(fixture, start_ms + 30u);
    assert(fixture->state.allow_rearm == 1u);
    assert(fixture->state.active == 0u);
    fixture->input.camera_sequence = sequence;
    fixture->input.camera_phase = 1u;
    fixture->input.camera_side = side;
    fixture->input.camera_fresh = 1u;
    fixture->input.ordinary_fresh = 0u;
    ring_set_time(fixture, start_ms + 40u);
    ring_step(fixture);
    assert(fixture->state.phase == CAR_GYRO_RING_ENTRY);
    fixture->input.camera_fresh = 0u;
}

static void ring_arm(ring_fixture_t *fixture, uint32_t start_ms,
                     uint16_t sequence, int8_t side)
{
    ring_trigger(fixture, start_ms - CAR_GYRO_RING_ENTRY_STRAIGHT_MS, sequence, side);
    ring_set_time(fixture, start_ms + 40u);
    ring_step(fixture);
    assert(fixture->state.entry_straight_done == 1u);
}

static void ring_phase2(ring_fixture_t *fixture, uint32_t now_ms,
                        int8_t side, int64_t yaw)
{
    ring_set_time(fixture, now_ms);
    ++fixture->input.camera_sequence;
    fixture->input.camera_phase = 2u;
    fixture->input.camera_side = side;
    fixture->input.camera_fresh = 1u;
    fixture->input.ordinary_fresh = 0u;
    fixture->input.yaw_mdeg = yaw;
    ring_step(fixture);
    fixture->input.camera_fresh = 0u;
}

static void ring_quiet_step(ring_fixture_t *fixture, uint32_t now_ms,
                            int64_t yaw)
{
    ring_set_time(fixture, now_ms);
    fixture->input.camera_fresh = 0u;
    fixture->input.ordinary_fresh = 0u;
    fixture->input.yaw_mdeg = yaw;
    ring_step(fixture);
}

static void test_ring_direction_zero_and_progress_boundary(int8_t side)
{
    ring_fixture_t fixture;
    const uint32_t start = 100u;
    const int64_t origin = INT64_C(1000000);
    const int64_t signed_progress = side < 0 ? 1 : -1;
    const int32_t expected_error =
        250 + (CAR_GYRO_RING_TARGET_YAW_RATE_MDPS * 2) / 1000;

    ring_arm(&fixture, start, 50u, side);
    assert(fixture.state.control_error_norm ==
           (side < 0 ? -expected_error : expected_error));
    ring_phase2(&fixture, start + 50u, side, origin);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    assert(fixture.state.progress_mdeg == 0);
    assert(fixture.state.yaw_origin_mdeg == origin);

    ring_quiet_step(&fixture, start + 60u,
                    origin + signed_progress * 349999);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    assert(fixture.state.progress_mdeg == 349999);

    ring_quiet_step(&fixture, start + 70u,
                    origin + signed_progress * 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_CENTER);
    assert(fixture.state.progress_mdeg == 350000);
    assert(fixture.state.servo_target_us == 1530u);

    fixture.input.applied_servo_us = 1500u;
    ring_quiet_step(&fixture, start + 80u,
                    origin + signed_progress * 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_CENTER);
    assert(fixture.state.last_event_ms == start + 70u);

    fixture.input.applied_servo_us = 1530u;
    ring_quiet_step(&fixture, start + 90u,
                    origin + signed_progress * 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_EXIT_STRAIGHT);
    assert(fixture.state.last_event_ms == start + 90u);

    ring_ordinary_frame(&fixture, start + 180u);
    ring_ordinary_frame(&fixture, start + 270u);
    ring_ordinary_frame(&fixture, start + 390u);
    assert(fixture.state.phase == CAR_GYRO_RING_REARM);
    assert(fixture.state.active == 0u);
    assert(fixture.state.allow_rearm == 1u);
}

static void test_ring_search_wait_and_recovery(void)
{
    ring_fixture_t fixture;
    uint32_t exit_start;
    uint16_t sequence;
    int64_t origin = INT64_C(2000000);

    ring_arm(&fixture, 1000u, 100u, -1);
    ring_phase2(&fixture, 1050u, -1, origin);
    ring_quiet_step(&fixture, 1060u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_CENTER);
    fixture.input.applied_servo_us = 1530u;
    ring_quiet_step(&fixture, 1070u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_EXIT_STRAIGHT);
    exit_start = fixture.state.last_event_ms;
    sequence = fixture.input.camera_sequence;

    ring_quiet_step(&fixture, exit_start + 299u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_EXIT_STRAIGHT);
    ring_quiet_step(&fixture, exit_start + 300u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_SEARCH);
    assert(fixture.state.search_elapsed_ms == 0u);
    assert(fixture.state.hold_ramp == 1u);
    assert(fixture.state.servo_target_us == 1530u);
    assert(fixture.state.outer_target_millirps ==
           CAR_GYRO_RING_OUTER_TARGET_MILLIRPS);

    ring_quiet_step(&fixture, exit_start + 3299u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_SEARCH);
    assert(fixture.state.search_elapsed_ms == 2999u);
    ring_quiet_step(&fixture, exit_start + 3300u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_WAIT_ROUTE);
    assert(fixture.state.search_elapsed_ms == 0u);
    assert(fixture.state.hold_ramp == 0u);
    assert(fixture.state.outer_target_millirps == 0u);

    fixture.input.camera_sequence = sequence;
    ring_ordinary_frame(&fixture, exit_start + 3301u);
    ring_ordinary_frame(&fixture, exit_start + 3302u);
    ring_ordinary_frame(&fixture, exit_start + 3303u);
    assert(fixture.state.phase == CAR_GYRO_RING_REARM);
    assert(fixture.state.active == 0u);
    assert(fixture.state.allow_rearm == 1u);
}

static void test_entry_straight_delay(int8_t side, uint32_t start)
{
    ring_fixture_t fixture;
    uint32_t trigger;
    ring_trigger(&fixture, start, 50u, side);
    trigger = fixture.input.now_ms;
    assert(fixture.state.servo_target_us == 1530u);
    assert(fixture.state.control_error_norm == 0);
    assert(fixture.state.outer_target_millirps == CAR_GYRO_RING_TURN_OUTER_TARGET_MILLIRPS);
    /* Early valid phase2 is retained but cannot shorten the straight. */
    ring_phase2(&fixture, trigger + 1u, side, 1000);
    ring_quiet_step(&fixture, trigger + 1499u, 2000);
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);
    assert(fixture.state.servo_target_us == 1530u);
    assert(fixture.state.control_error_norm == 0);
    /* No fresh image or new marker is required at expiry. */
    ring_quiet_step(&fixture, trigger + 1500u, 3000);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    assert(fixture.state.servo_target_us == 0u);
    assert(fixture.state.control_error_norm * side > 0);
    assert(fixture.state.progress_mdeg == 0);
    assert(fixture.state.yaw_origin_mdeg == 3000);
    assert(fixture.state.outer_target_millirps == CAR_GYRO_RING_OUTER_TARGET_MILLIRPS * 3u / 8u);

    ring_trigger(&fixture, start, 50u, side);
    trigger = fixture.input.now_ms;
    ring_quiet_step(&fixture, trigger + 1500u, 0);
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);
    assert(fixture.state.entry_straight_done == 1u);
    assert(fixture.state.control_error_norm * side > 0);
}

static void test_profile_targets(void)
{
#if CARTEST_TRACK_PROFILE == 2
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 4000u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 64935);
#elif CARTEST_TRACK_PROFILE == 3 && CARTEST_SPEED_PERCENT == 100
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 5280u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 85714);
#elif CARTEST_TRACK_PROFILE == 3 && CARTEST_SPEED_PERCENT == 120
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 6336u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 102857);
#elif CARTEST_TRACK_PROFILE == 3 && CARTEST_SPEED_PERCENT == 130
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 6864u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 111429);
#else
    assert(CAR_GYRO_RING_OUTER_TARGET_MILLIRPS == 7392u);
    assert(CAR_GYRO_RING_TARGET_YAW_RATE_MDPS == 120000);
#endif
}

static void test_ring_rejects_unconfirmed_old_and_wrong_side(void)
{
    ring_fixture_t fixture;
    uint16_t entry_sequence;

    ring_init(&fixture, 100u, 1u);
    fixture.input.camera_phase = 2u;
    fixture.input.camera_side = -1;
    fixture.input.camera_sequence = 2u;
    ring_set_time(&fixture, 101u);
    ring_step(&fixture);
    assert(fixture.state.phase == CAR_GYRO_RING_OFF);
    assert(fixture.state.active == 0u);

    ring_arm(&fixture, 200u, 30u, -1);
    entry_sequence = fixture.input.camera_sequence;
    fixture.input.camera_phase = 2u;
    fixture.input.camera_side = -1;
    fixture.input.camera_fresh = 1u;
    ring_set_time(&fixture, 250u);
    ring_step(&fixture); /* duplicate sequence */
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);

    fixture.input.camera_sequence = (uint16_t)(entry_sequence - 1u);
    ring_set_time(&fixture, 260u);
    ring_step(&fixture); /* backward sequence */
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);

    fixture.input.camera_sequence = (uint16_t)(entry_sequence + 1u);
    fixture.input.camera_side = 1;
    ring_set_time(&fixture, 270u);
    ring_step(&fixture); /* fresh event, wrong side */
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);

    fixture.input.camera_sequence = (uint16_t)(entry_sequence + 2u);
    fixture.input.camera_side = -1;
    ring_set_time(&fixture, 280u);
    ring_step(&fixture);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    assert(fixture.state.yaw_origin_mdeg == fixture.input.yaw_mdeg);
}

static void test_ring_backward_sway_and_sequence_wrap(void)
{
    ring_fixture_t fixture;
    const int64_t origin = 700000;

    ring_arm(&fixture, 500u, 65533u, -1);
    /* The three recovery frames crossed 65535 -> 0 without losing order. */
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);
    ring_phase2(&fixture, 550u, -1, origin);
    ring_quiet_step(&fixture, 560u, origin + 200000);
    assert(fixture.state.progress_mdeg == 200000);
    ring_quiet_step(&fixture, 570u, origin + 100000);
    assert(fixture.state.progress_mdeg == 100000);
    ring_quiet_step(&fixture, 580u, origin + 349999);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    ring_quiet_step(&fixture, 590u, origin + 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_CENTER);

    /* A subsequent entry sequence advancing through uint16 wrap is forward. */
    car_gyro_ring_init(&fixture.state);
    fixture.input.run_active = 1u;
    fixture.input.enabled = 1u;
    fixture.input.imu_ready = 1u;
    fixture.input.imu_fault = 0u;
    fixture.input.transport_fresh = 1u;
    fixture.input.camera_phase = 0u;
    fixture.input.camera_fresh = 1u;
    fixture.input.ordinary_fresh = 1u;
    fixture.input.camera_sequence = 65534u;
    ring_set_time(&fixture, 1000u);
    ring_step(&fixture);
    ring_ordinary_frame(&fixture, 1001u); /* 65535 */
    ring_ordinary_frame(&fixture, 1002u); /* 0 */
    assert(fixture.state.allow_rearm == 1u);
}

static void test_ring_stop_fault_and_session_rearm(void)
{
    ring_fixture_t fixture;
    uint16_t sequence;

    ring_arm(&fixture, 100u, 50u, -1);
    sequence = fixture.input.camera_sequence;
    fixture.input.transport_fresh = 0u;
    ring_set_time(&fixture, 150u);
    ring_step(&fixture);
    assert(fixture.state.fault_latched == 1u);
    assert(fixture.state.active == 1u);
    assert(fixture.state.outer_target_millirps == 0u);

    fixture.input.transport_fresh = 1u;
    ring_set_time(&fixture, 160u);
    ring_step(&fixture);
    assert(fixture.state.fault_latched == 1u);

    fixture.input.run_active = 0u;
    fixture.input.camera_phase = 1u;
    fixture.input.camera_side = -1;
    fixture.input.camera_fresh = 1u;
    fixture.input.camera_sequence = (uint16_t)(sequence + 1u);
    ring_set_time(&fixture, 170u);
    ring_step(&fixture);
    assert(fixture.state.phase == CAR_GYRO_RING_OFF);
    assert(fixture.state.active == 0u);
    assert(fixture.state.fault_latched == 0u);
    assert(fixture.state.have_camera_sequence == 0u);

    fixture.input.run_active = 1u;
    ring_set_time(&fixture, 180u);
    ring_step(&fixture); /* stale phase1 ignored until three ordinary frames */
    assert(fixture.state.phase == CAR_GYRO_RING_OFF);
    ring_ordinary_frame(&fixture, 190u);
    ring_ordinary_frame(&fixture, 200u);
    ring_ordinary_frame(&fixture, 210u);
    assert(fixture.state.allow_rearm == 1u);
}

static void test_ring_pi_sample_gate_and_time_wrap(void)
{
    ring_fixture_t fixture;
    uint32_t wrap_start = UINT32_MAX - 100u;
    int32_t integral;
    int64_t remainder;
    uint32_t imu_sample;

    ring_arm(&fixture, wrap_start, 100u, -1);
    assert(fixture.state.control_error_norm ==
           -(250 + (CAR_GYRO_RING_TARGET_YAW_RATE_MDPS * 2) / 1000));
    imu_sample = fixture.input.imu_sample_us;
    ring_set_time(&fixture, wrap_start + 41u);
    fixture.input.imu_sample_us = imu_sample;
    ring_step(&fixture);
    integral = fixture.state.imu_integral_permille;
    remainder = fixture.state.imu_integral_remainder;
    ring_set_time(&fixture, wrap_start + 42u);
    fixture.input.imu_sample_us = imu_sample;
    ring_step(&fixture);
    assert(fixture.state.imu_integral_permille == integral);
    assert(fixture.state.imu_integral_remainder == remainder);

    ring_set_time(&fixture, wrap_start + 90u);
    fixture.input.imu_sample_us =
        (uint32_t)(imu_sample + 50000u);
    ring_step(&fixture);
    assert(fixture.state.imu_integral_permille != integral);

    /* Entry timeout uses unsigned elapsed time across millisecond wrap. */
    ring_set_time(&fixture, wrap_start + 4040u);
    fixture.input.imu_sample_us = fixture.input.now_ms * 1000u;
    ring_step(&fixture);
    assert(fixture.state.fault_latched == 1u);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_TIMEOUT);
}

static void test_ring_stale_and_imu_fault(void)
{
    ring_fixture_t fixture;

    ring_arm(&fixture, 1000u, 10u, -1);
    fixture.input.imu_fault = 1u;
    ring_set_time(&fixture, 1050u);
    ring_step(&fixture);
    assert(fixture.state.fault_latched == 1u);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_IMU);

    car_gyro_ring_init(&fixture.state);
    fixture.input.imu_fault = 0u;
    fixture.input.run_active = 1u;
    fixture.input.enabled = 1u;
    fixture.input.imu_ready = 1u;
    fixture.input.transport_fresh = 1u;
    fixture.input.camera_phase = 0u;
    fixture.input.camera_fresh = 1u;
    fixture.input.ordinary_fresh = 1u;
    fixture.input.camera_sequence = 100u;
    ring_set_time(&fixture, 2000u);
    ring_step(&fixture);
    ring_ordinary_frame(&fixture, 2010u);
    ring_ordinary_frame(&fixture, 2020u);
    ring_ordinary_frame(&fixture, 2030u);
    fixture.input.camera_sequence++;
    fixture.input.camera_phase = 1u;
    fixture.input.camera_side = 1;
    fixture.input.camera_fresh = 1u;
    fixture.input.ordinary_fresh = 0u;
    ring_set_time(&fixture, 2040u);
    ring_step(&fixture);
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);
    fixture.input.camera_fresh = 0u;
    ring_set_time(&fixture, 2091u);
    fixture.input.imu_sample_us = (uint32_t)(fixture.input.now_ms * 1000u -
                                             50001u);
    ring_step(&fixture);
    assert(fixture.state.fault_latched == 1u);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_IMU);
}

static void test_ring_phase_timeouts(void)
{
    ring_fixture_t fixture;
    uint32_t start;

    ring_arm(&fixture, 10u, 10u, -1);
    start = fixture.state.last_event_ms;
    ring_quiet_step(&fixture, start + 3999u, 0);
    assert(fixture.state.phase == CAR_GYRO_RING_ENTRY);
    ring_quiet_step(&fixture, start + 4000u, 0);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_TIMEOUT);

    ring_arm(&fixture, 10000u, 100u, -1);
    ring_phase2(&fixture, 10050u, -1, 0);
    start = fixture.state.last_event_ms;
    ring_quiet_step(&fixture, start + 15999u, 0);
    assert(fixture.state.phase == CAR_GYRO_RING_ORBIT);
    ring_quiet_step(&fixture, start + 16000u, 0);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_TIMEOUT);

    ring_arm(&fixture, 30000u, 200u, -1);
    ring_phase2(&fixture, 30050u, -1, 0);
    ring_quiet_step(&fixture, 30060u, 350000);
    fixture.input.applied_servo_us = 1500u;
    start = fixture.state.last_event_ms;
    ring_quiet_step(&fixture, start + 999u, 350000);
    assert(fixture.state.phase == CAR_GYRO_RING_CENTER);
    ring_quiet_step(&fixture, start + 1000u, 350000);
    assert(fixture.state.fault == CAR_GYRO_RING_FAULT_TIMEOUT);
}

int main(void)
{
    test_profile_targets();
    test_entry_straight_delay(-1, 100u);
    test_entry_straight_delay(1, UINT32_MAX - 100u);
    test_imu_init_calibration_and_variable_dt();
    test_imu_whoami_diagnostics();
    test_imu_faults();
    test_ring_direction_zero_and_progress_boundary(-1);
    test_ring_direction_zero_and_progress_boundary(1);
    test_ring_search_wait_and_recovery();
    test_ring_rejects_unconfirmed_old_and_wrong_side();
    test_ring_backward_sway_and_sequence_wrap();
    test_ring_stop_fault_and_session_rearm();
    test_ring_pi_sample_gate_and_time_wrap();
    test_ring_stale_and_imu_fault();
    test_ring_phase_timeouts();
    (void)puts("gyro350 pure-C tests: PASS");
    return 0;
}
