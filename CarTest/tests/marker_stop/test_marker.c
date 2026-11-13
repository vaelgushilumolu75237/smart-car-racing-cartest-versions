#define main previous_authority_main
#include "../gyro350/test_authority.c"
#undef main

static void init_running(authority_fixture_t *f)
{
    fixture_init(f);
    f->control.imu_ready = 1u;
}

static void marker_frame(authority_fixture_t *f, int marker, uint8_t valid)
{
    vp_frame wire = {0}, decoded = {0};
    uint8_t bytes[VP_FRAME_SIZE];
    wire.sequence = ++f->sequence;
    wire.flags = valid ? VP_FLAG_VALID : 0u;
    if (marker >= 0) wire.flags |= VP_FLAG_MARKER_KNOWN;
    if (marker > 0) wire.flags |= VP_FLAG_MARKER_PRESENT;
    wire.confidence = valid ? 90u : 0u;
    wire.profile_ack = f->config.required_camera_profile_ack;
    wire.ordinary_route = valid;
    assert(vp_pack_v4(&wire, bytes));
    assert(vp_unpack(bytes, &decoded));
    f->now_ms += 10u;
    f->control.imu_sample_us = f->now_ms * 1000u;
    car_control_on_vision_frame_at_with_ring(
        &f->control, &f->config, decoded.sequence, decoded.flags,
        decoded.confidence, 0, 0, 0, VP_VERSION_V4, decoded.profile_ack,
        f->now_ms, f->now_ms, 1u, 0, 0, decoded.ordinary_route);
}

static void leave_marker(authority_fixture_t *f)
{
    for (int i = 0; i < 3; ++i) marker_frame(f, 0, 1u);
}

static car_inputs_t tick_at(authority_fixture_t *f, uint32_t now)
{
    car_inputs_t in = {0};
    in.now_ms = now;
    in.pe7_raw_high = in.pe7_high_stable = 1u;
    in.manual_run_request = in.manual_button_startup_release_confirmed = 1u;
    f->control.imu_sample_us = now * 1000u;
    car_control_tick(&f->control, &f->config, &in);
    return in;
}

static void third_pass(authority_fixture_t *f)
{
    for (int pass = 1; pass <= 3; ++pass) {
        marker_frame(f, 1, 1u);
        assert(f->control.marker_count == pass);
        for (int i = 0; i < 100; ++i) marker_frame(f, 1, 1u);
        assert(f->control.marker_count == pass);
        assert(!f->control.marker_stop_pending);
        marker_frame(f, 0, 1u);
        marker_frame(f, 1, 1u); /* A missed frame cannot split a pass. */
        assert(f->control.marker_count == pass);
        leave_marker(f);
        assert(f->control.marker_stop_pending == (pass == 3));
    }
}

static void test_three_passes_deadline_and_permanent_latch(void)
{
    authority_fixture_t f;
    init_running(&f);
    third_pass(&f);
    uint32_t start = f.control.marker_left_ms;
    /* Continue real ordinary vision/ticks during the whole delay. */
    for (int i = 0; i < 99; ++i) {
        marker_frame(&f, 0, 1u);
        tick_at(&f, f.now_ms);
        assert(f.control.road_drive_allowed);
        assert(!f.control.marker_stop_latched);
        assert(f.control.mode == CAR_MODE_FOLLOW_RUNNING);
    }
    tick_at(&f, start + 999u);
    assert(f.control.road_drive_allowed && !f.control.marker_stop_latched);
    car_inputs_t in = tick_at(&f, start + 1000u);
    assert(f.control.marker_stop_latched && f.control.fault_latched);
    assert(f.control.fault == CAR_FAULT_FINISH_MARKER);
    assert(!f.control.road_drive_allowed && !f.control.follow_output_duty_permille);
    assert(!f.control.follow_session_active && !f.control.manual_run_request);
    assert(!car_control_begin_manual_follow_start(&f.control, &f.config, &in));
    car_control_command(&f.control, &f.config, &in, CAR_CMD_NORMAL_STOP, 0, 0, 0);
    car_control_command(&f.control, &f.config, &in, CAR_CMD_LOCAL_ACK, 0, 0, 0);
    car_control_command(&f.control, &f.config, &in, CAR_CMD_CAMERA_RESTART_ACK, 0, 0, 0);
    for (int i = 0; i < 10; ++i) {
        in.now_ms += 10u;
        in.pe7_raw_high = in.pe7_high_stable = i & 1;
        in.pe7_low_stable = !(i & 1);
        car_control_tick(&f.control, &f.config, &in);
        assert(f.control.marker_stop_latched && f.control.fault_latched);
        assert(!f.control.road_drive_allowed);
    }
    assert(!car_control_follow_authority_ready(&f.control, &f.config, in.now_ms));
}

static void test_unknown_gaps_parked_and_invalid_route(void)
{
    authority_fixture_t f;
    init_running(&f);
    f.control.follow_session_active = 0u;
    marker_frame(&f, 1, 1u);
    assert(f.control.marker_count == 0u);
    f.control.follow_session_active = 1u;
    marker_frame(&f, 1, 0u); /* The marking can obscure the normal route. */
    assert(f.control.marker_count == 1u);
    marker_frame(&f, 0, 1u);
    marker_frame(&f, 0, 1u);
    marker_frame(&f, -1, 1u); /* Unknown perception is not an absence. */
    marker_frame(&f, 0, 1u);
    marker_frame(&f, 0, 1u);
    assert(f.control.marker_present);
    f.sequence += 2u; /* Lost camera frames break disappearance confirmation. */
    marker_frame(&f, 0, 1u);
    assert(f.control.marker_present && f.control.marker_absent_frames == 1u);
    marker_frame(&f, 0, 1u);
    assert(f.control.marker_present);
    marker_frame(&f, 0, 1u);
    assert(!f.control.marker_present && !f.control.marker_stop_pending);
    marker_frame(&f, 1, 1u);
    assert(f.control.marker_count == 2u);
}

static void test_rejected_frames_legacy_and_crc(void)
{
    authority_fixture_t f;
    init_running(&f);
    marker_frame(&f, 1, 1u);
    car_control_on_vision_frame_at_with_ring(&f.control, &f.config, ++f.sequence,
        VP_FLAG_VALID | VP_FLAG_MARKER_KNOWN, 90, 0, 0, 0, VP_VERSION_V4,
        f.config.required_camera_profile_ack, f.now_ms, f.now_ms + 200u, 1, 0, 0, 1);
    assert(f.control.marker_present && f.control.marker_absent_frames == 0u);
    vp_frame frame = {0}, decoded = {0};
    uint8_t bytes[VP_FRAME_SIZE];
    frame.flags = VP_FLAG_MARKER_PRESENT; /* Invalid without KNOWN. */
    assert(!vp_pack_v4(&frame, bytes));
    frame.flags |= VP_FLAG_MARKER_KNOWN;
    assert(vp_pack_v4(&frame, bytes));
    assert(!vp_pack(&frame, bytes) && !vp_pack_v3(&frame, bytes));
    assert(vp_pack_v4(&frame, bytes));
    bytes[6] ^= VP_FLAG_MARKER_PRESENT;
    assert(vp_unpack_ex(bytes, &decoded) == VP_STATUS_BAD_CRC);
    init_running(&f);
    for (int i = 0; i < 20; ++i) send_frame(&f, 1, VP_VERSION_V3, 0, 0, 1, f.now_ms + 10u);
    assert(!f.control.marker_count && !f.control.marker_stop_pending);
}

static void test_deadline_survives_other_stop_and_tick_wrap(void)
{
    authority_fixture_t f;
    init_running(&f);
    third_pass(&f);
    car_inputs_t in = tick_at(&f, f.now_ms);
    car_control_command(&f.control, &f.config, &in, CAR_CMD_NORMAL_STOP, 0, 0, 0);
    assert(f.control.marker_stop_pending);
    f.control.marker_left_ms = UINT32_MAX - 500u;
    tick_at(&f, 498u); /* elapsed 999 ms */
    assert(!f.control.marker_stop_latched);
    tick_at(&f, 499u);
    assert(f.control.marker_stop_latched && !f.control.road_drive_allowed);
}

int main(void)
{
    previous_authority_main();
    test_three_passes_deadline_and_permanent_latch();
    test_unknown_gaps_parked_and_invalid_route();
    test_rejected_frames_legacy_and_crc();
    test_deadline_survives_other_stop_and_tick_wrap();
    puts("three-pass marker protocol/count/timer/permanent-stop tests: PASS");
    return 0;
}
