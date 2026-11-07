#include "car_vision.h"
#include "car_calibration.h"
#include "commissioning.h"
#include "vision_protocol.h"
#if CARTEST_WIRELESS_ENABLED
#include "wireless_runtime.h"
#include <cstring>
#endif

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {

unsigned g_checks = 0u;
unsigned g_failures = 0u;

void check(bool condition, const char *label)
{
    ++g_checks;
    if (!condition) {
        ++g_failures;
        std::printf("FAIL %s\n", label);
    }
}

struct Rig {
    car_config_t config{};
    car_control_t control{};
    car_vision_rx_t rx{};
    uint16_t sequence = 0u;
    uint32_t now_ms = 0u;

    void init()
    {
        car_config_init_application(&config, 0u, 0u, 1u, 1u);
        car_control_init(&control, &config, 0u, 1u, 0u);
        car_vision_rx_init(&rx);
    }

    void tick(uint32_t now, uint8_t manual_run)
    {
        car_inputs_t inputs{};
        now_ms = now;
        inputs.now_ms = now;
        inputs.pe7_raw_high = 1u;
        inputs.pe7_high_stable = 1u;
        inputs.manual_run_request = manual_run;
        inputs.manual_button_startup_release_confirmed = 1u;
        car_control_tick(&control, &config, &inputs);
    }

    void set_imu(uint8_t ready, int64_t yaw_mdeg, int32_t z_mdps = 0)
    {
        control.imu_ready = ready;
        control.imu_fault = 0u;
        control.imu_sample_us = now_ms * 1000u;
        control.imu_yaw_mdeg = yaw_mdeg;
        control.imu_z_mdps = z_mdps;
    }

    bool send(uint32_t now, uint8_t valid, uint8_t ring_phase,
              int8_t ring_side, uint8_t ordinary, int64_t yaw_mdeg,
              uint8_t confidence = 90u)
    {
        vp_frame frame{};
        uint8_t bytes[VP_FRAME_SIZE]{};
        frame.sequence = ++sequence;
        frame.flags = valid != 0u ? VP_FLAG_VALID : 0u;
        frame.confidence = valid != 0u ? confidence : 0u;
        frame.lateral_error = 0;
        frame.curve_phase = VP_CURVE_PHASE_STRAIGHT;
        frame.profile_ack = VP_PROFILE_ACK_DEBUG_RING_FAST;
        frame.ring_phase = ring_phase;
        frame.ring_side = ring_side;
        frame.ordinary_route = ordinary;
        if (!vp_pack_v4(&frame, bytes)) {
            return false;
        }
        now_ms = now;
        control.imu_sample_us = now * 1000u;
        control.imu_yaw_mdeg = yaw_mdeg;
        for (uint8_t byte : bytes) {
            car_vision_rx_isr_byte(&rx, byte, now);
        }
        car_vision_rx_poll(&rx, &control, &config, rx.head, now, 1u);
        return control.latest_rx_sequence == frame.sequence;
    }

    void note_applied(uint16_t servo = CARTEST_SERVO_CENTER_US)
    {
        car_control_note_applied_outputs(&control, 1u, 1u, servo,
                                         1000u, 1000u);
    }
};

void test_startup_gate_and_session(Rig &rig)
{
    rig.init();
    check(rig.config.gyro_enabled != 0u, "gyro profile enabled");
    check(rig.config.required_camera_profile_ack ==
              VP_PROFILE_ACK_DEBUG_RING_FAST,
          "profile C acknowledgement required");

    rig.tick(0u, 0u);
    check(rig.control.startup_release_seen != 0u,
          "PE7 startup release accepted");
    check(rig.control.rx_recovery_flush_required != 0u,
          "startup flush required");
    car_inputs_t before_start{};
    before_start.now_ms = 0u;
    before_start.manual_run_request = 1u;
    check(car_control_begin_manual_follow_start(&rig.control, &rig.config,
                                                &before_start) == 0u,
          "start rejected before IMU and camera readiness");

    car_control_rx_recovery_flush_complete(&rig.control);
    rig.now_ms = 100u;
    rig.set_imu(1u, 0);
    check(rig.send(100u, 1u, 0u, 0, 1u, 0),
          "startup ordinary frame 1 parsed");
    check(rig.send(110u, 1u, 0u, 0, 1u, 0),
          "startup ordinary frame 2 parsed");
    check(rig.send(120u, 1u, 0u, 0, 1u, 0),
          "startup ordinary frame 3 parsed");

    car_inputs_t start{};
    start.now_ms = 120u;
    start.manual_run_request = 1u;
    start.manual_button_startup_release_confirmed = 1u;
    check(car_control_begin_manual_follow_start(&rig.control, &rig.config,
                                                &start) != 0u,
          "manual follow start accepted after IMU calibration");
    car_control_rx_recovery_flush_complete(&rig.control);
    car_control_reverse_control_reset_phase_complete(&rig.control);
    car_control_reverse_control_keep_started(&rig.control);

    check(rig.send(130u, 1u, 0u, 0, 1u, 0), "recovery frame 1");
    check(rig.send(140u, 1u, 0u, 0, 1u, 0), "recovery frame 2");
    check(rig.send(150u, 1u, 0u, 0, 1u, 0), "recovery frame 3");
    rig.tick(150u, 1u);
    check(rig.control.follow_session_active != 0u,
          "ordinary recovery starts follow session");
    check(rig.control.fault == CAR_FAULT_NONE &&
              rig.control.fault_latched == 0u,
          "startup/session remains fault free");
    rig.note_applied();
}

void prime_ring(Rig &rig)
{
    const uint32_t base = rig.now_ms;
    check(rig.send(base + 10u, 1u, 0u, 0, 1u, 0),
          "ring rearm frame 1");
    check(rig.send(base + 20u, 1u, 0u, 0, 1u, 0),
          "ring rearm frame 2");
    check(rig.send(base + 30u, 1u, 0u, 0, 1u, 0),
          "ring rearm frame 3");
    check(rig.control.gyro.phase == CAR_GYRO_RING_REARM,
          "three ordinary frames arm ring rearm state");
    check(rig.control.gyro.active == 0u,
          "rearm state does not drive ring");
}

void test_left_ring_and_search(Rig &rig)
{
    prime_ring(rig);
    rig.note_applied();
    check(rig.send(rig.now_ms + 10u, 1u, 1u, -1, 0u, 0),
          "left entry evidence");
    check(rig.control.gyro.phase == CAR_GYRO_RING_ENTRY &&
              rig.control.gyro.active != 0u,
          "left entry takes gyro authority");
    rig.note_applied();

    check(rig.send(rig.now_ms + 10u, 1u, 2u, -1, 0u, 500000),
          "inside confirmation after nonzero yaw");
    check(rig.control.gyro.phase == CAR_GYRO_RING_ORBIT &&
              rig.control.gyro.progress_mdeg == 0,
          "inside confirmation zeros relative yaw");
    rig.note_applied();

    check(rig.send(rig.now_ms + 10u, 1u, 2u, -1, 0u, 650000),
          "left orbit progress");
    check(rig.control.gyro.progress_mdeg == 150000,
          "left progress uses signed yaw");
    check(rig.send(rig.now_ms + 10u, 1u, 2u, -1, 0u, 850000),
          "left orbit reaches 350 degrees");
    check(rig.control.gyro.progress_mdeg == 350000 &&
              rig.control.gyro.phase == CAR_GYRO_RING_CENTER,
          "left 350 degree threshold enters center");

    rig.note_applied(1530u);
    check(rig.send(rig.now_ms + 10u, 0u, 0u, 0, 0u, 850000),
          "invalid route does not break gyro exit");
    check(rig.control.gyro.phase == CAR_GYRO_RING_EXIT_STRAIGHT,
          "software center starts fixed straight stage");
    check(rig.control.gyro.servo_target_us == 1530u,
          "center stage holds 1530 microseconds");

    const uint32_t straight_deadline = rig.now_ms +
        CAR_GYRO_RING_EXIT_STRAIGHT_MS;
    for (uint32_t now = rig.now_ms + 100u;
         now < straight_deadline; now += 100u) {
        check(rig.send(now, 0u, 0u, 0, 0u, 850000),
              "invalid frame keeps straight-stage transport fresh");
    }
    check(rig.send(straight_deadline, 0u, 0u, 0, 0u, 850000),
          "invalid frame at straight deadline");
    check(rig.control.gyro.phase == CAR_GYRO_RING_SEARCH,
          "missing route enters bounded search");
    const uint32_t search_deadline = rig.now_ms +
        CAR_GYRO_RING_SEARCH_LIMIT_MS;
    for (uint32_t now = rig.now_ms + 100u;
         now + 1u < search_deadline; now += 100u) {
        check(rig.send(now, 0u, 0u, 0, 0u, 850000),
              "invalid frame keeps gyro transport fresh");
    }
    check(rig.send(search_deadline - 1u, 0u, 0u, 0, 0u, 850000),
          "invalid frame before search timeout");
    check(rig.control.gyro.phase == CAR_GYRO_RING_SEARCH,
          "invalid frames do not count as ordinary recovery");
    check(rig.send(search_deadline, 0u, 0u, 0, 0u, 850000),
          "invalid frame at search timeout");
    check(rig.control.gyro.phase == CAR_GYRO_RING_WAIT_ROUTE &&
              rig.control.gyro.outer_target_millirps == 0u,
          "search timeout stops ring target");

    check(rig.send(search_deadline + 10u, 1u, 0u, 0, 1u, 850000),
          "ordinary route recovery frame 1");
    check(rig.send(search_deadline + 20u, 1u, 0u, 0, 1u, 850000),
          "ordinary route recovery frame 2");
    check(rig.send(search_deadline + 30u, 1u, 0u, 0, 1u, 850000),
          "ordinary route recovery frame 3");
    check(rig.control.gyro.phase == CAR_GYRO_RING_REARM &&
              rig.control.gyro.active == 0u,
          "three fresh ordinary frames release ring authority");
}

void test_right_signed_progress(Rig &rig)
{
    rig.tick(3600u, 1u);
    rig.note_applied();
    prime_ring(rig);
    check(rig.send(rig.now_ms + 10u, 1u, 1u, 1, 0u, 0),
          "right entry evidence");
    check(rig.control.gyro.entry_side == 1,
          "right entry side latched");
    check(rig.send(rig.now_ms + 10u, 1u, 2u, 1, 0u, -200000),
          "right inside confirmation");
    check(rig.control.gyro.phase == CAR_GYRO_RING_ORBIT &&
              rig.control.gyro.progress_mdeg == 0,
          "right inside zeros relative yaw");
    check(rig.send(rig.now_ms + 10u, 1u, 2u, 1, 0u, -550000),
          "right signed progress");
    check(rig.control.gyro.progress_mdeg == 350000,
          "right mirrored yaw reaches 350 degrees");
}

void test_fault_and_stop(Rig &rig)
{
    check(rig.control.gyro.phase == CAR_GYRO_RING_CENTER,
          "right threshold reaches center before fault test");
    rig.control.imu_fault = 1u;
    rig.now_ms += 10u;
    car_control_update_gyro(&rig.control, &rig.config, rig.now_ms);
    check(rig.control.gyro.fault != 0u && rig.control.fault == CAR_FAULT_IMU,
          "IMU fault stops gyro authority");

    car_inputs_t stop{};
    stop.now_ms = rig.now_ms + 10u;
    stop.pe7_raw_high = 1u;
    stop.pe7_high_stable = 1u;
    stop.manual_button_startup_release_confirmed = 1u;
    car_control_tick(&rig.control, &rig.config, &stop);
    check(rig.control.follow_session_active == 0u,
          "PE7 stop clears follow session");
    check(rig.control.fault_latched != 0u &&
              rig.control.gyro.phase == CAR_GYRO_RING_FAULT,
          "PE7 stop preserves latched gyro fault");
}

}  // namespace

int main()
{
#if CARTEST_WIRELESS_ENABLED
    car_wireless_init(0,123);
    /* Valid and corrupt side-band envelopes containing a complete V4 frame
       must never inject that payload as a route observation. */
    Rig mix;
    mix.init();
    mix.tick(0,0);
    car_control_rx_recovery_flush_complete(&mix.control);
    car_wireless_frame_t extra{};
    extra.type=CAR_WIRELESS_IMAGE;extra.payload_len=160;
    vp_frame embedded{};embedded.sequence=500;embedded.flags=VP_FLAG_VALID;
    embedded.confidence=90;embedded.profile_ack=VP_PROFILE_ACK_DEBUG_RING_FAST;
    embedded.ordinary_route=1;
    check(vp_pack_v4(&embedded,extra.payload+20)!=0,"embedded V4 fixture");
    uint8_t packet[CAR_WIRELESS_MAX_FRAME];size_t n=0;
    check(car_wireless_encode(&extra,packet,sizeof packet,&n)!=0,"side-band fixture");
    for(unsigned corrupt=0;corrupt<2;corrupt++){
        if(corrupt)packet[n-1]^=1;
        for(size_t i=0;i<n;i++)car_vision_rx_isr_byte(&mix.rx,packet[i],10+corrupt);
        car_vision_rx_poll(&mix.rx,&mix.control,&mix.config,mix.rx.head,10+corrupt,1);
        check(mix.control.latest_rx_sequence!=500,"image V4 bytes cannot gain route authority");
    }
    check(mix.send(20,1,0,0,1,0),"normal V4 accepted after image frames");
#endif
    Rig rig;
    test_startup_gate_and_session(rig);
    if (g_failures == 0u) {
        test_left_ring_and_search(rig);
    }
    if (g_failures == 0u) {
        test_right_signed_progress(rig);
    }
    if (g_failures == 0u) {
        test_fault_and_stop(rig);
    }
    std::printf("GYRO350_INTEGRATION checks=%u failures=%u\n",
                g_checks, g_failures);
    return g_failures == 0u ? EXIT_SUCCESS : EXIT_FAILURE;
}
