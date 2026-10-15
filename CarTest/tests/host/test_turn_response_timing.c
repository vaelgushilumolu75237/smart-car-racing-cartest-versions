#include "follow_output.h"
#include "servo_safety.h"

#include <stdint.h>
#include <stdio.h>

#ifndef CARTEST_FOLLOW_DIFF_PERMILLE
#error "The comparison profile must select its outer differential."
#endif
#ifndef CARTEST_FOLLOW_INNER_DIFF_PERMILLE
#error "The comparison profile must select its inner differential."
#endif

enum { REVERSE_LEFT_US = 1229u, CENTER_US = 1530u, REVERSE_RIGHT_US = 2209u };
enum { CALIBRATED_LEFT_US = 1100u, CALIBRATED_RIGHT_US = 2500u };

typedef struct {
    car_servo_timer_snapshot_t snapshot;
} fake_servo_t;

static void capture(void *context, car_servo_timer_snapshot_t *snapshot)
{
    *snapshot = ((fake_servo_t *)context)->snapshot;
}

static uint8_t write_compare(void *context, uint16_t pulse_us)
{
    ((fake_servo_t *)context)->snapshot.tim_ccr1 = pulse_us;
    return 1u;
}

static uint32_t read_compare(void *context)
{
    return ((fake_servo_t *)context)->snapshot.tim_ccr1;
}

static uint8_t start_output(void *context)
{
    (void)context;
    return 1u;
}

static uint8_t take_completed_update(void *context)
{
    (void)context;
    return 0u;
}

static void stop_output(void *context)
{
    fake_servo_t *fake = (fake_servo_t *)context;
    fake->snapshot.tim_cr1 = 0u;
    fake->snapshot.tim_ccer = 0u;
    fake->snapshot.tim_bdtr = 0u;
    fake->snapshot.tim_ccr1 = 0u;
}

static car_servo_safety_io_t make_io(fake_servo_t *fake)
{
    car_servo_safety_io_t io = {0};
    io.context = fake;
    io.capture = capture;
    io.write_compare = write_compare;
    io.read_compare = read_compare;
    io.start_output = start_output;
    io.take_completed_update = take_completed_update;
    io.stop_output = stop_output;
    return io;
}

static fake_servo_t make_active_right_endpoint(void)
{
    fake_servo_t fake = {0};
    fake.snapshot.sysclk_hz = 64000000u;
    fake.snapshot.hclk_hz = 64000000u;
    fake.snapshot.pclk2_hz = 64000000u;
    fake.snapshot.rcc_cr = 0x5u;
    fake.snapshot.rcc_apb2enr = 0x1u;
    fake.snapshot.tim_cr1 = 0x1u;
    fake.snapshot.tim_ccmr1 = (1u << 3u) | (6u << 4u);
    fake.snapshot.tim_ccer = 0x1u;
    fake.snapshot.tim_bdtr = 1u << 15u;
    fake.snapshot.tim_psc = 63u;
    fake.snapshot.tim_arr = 19999u;
    fake.snapshot.tim_ccr1 = REVERSE_RIGHT_US;
    fake.snapshot.gpioa_moder = 2u << 16u;
    fake.snapshot.gpioa_afr1 = 0x1u;
    return fake;
}

static uint16_t abs_delta(uint16_t left, uint16_t right)
{
    return left > right ? (uint16_t)(left - right)
                        : (uint16_t)(right - left);
}

int main(void)
{
    fake_servo_t fake = make_active_right_endpoint();
    car_servo_safety_io_t io = make_io(&fake);
    car_servo_safety_t servo = {0};
    car_follow_output_state_t output = {0};
    uint16_t applied_left = (uint16_t)(500u + CARTEST_FOLLOW_DIFF_PERMILLE);
    uint16_t applied_right =
        (uint16_t)(500u - CARTEST_FOLLOW_INNER_DIFF_PERMILLE);
    uint16_t left_command = applied_left;
    uint16_t right_command = applied_right;
    uint16_t max_pwm_step = 0u;
    uint16_t max_servo_step = 0u;
    int first_limited_ms = -1;
    int servo_endpoint_ms = -1;
    int motor_endpoint_ms = -1;
    int failed = 0;
    uint32_t now_ms;

    servo.active = 1u;
    servo.frame_ready = 1u;
    servo.limited_us = REVERSE_RIGHT_US;
    servo.compare_us = REVERSE_RIGHT_US;
    servo.have_slew_time = 1u;
    servo.last_slew_ms = 0u;

    output.startup_qualified = 1u;
    output.target_left_permille = applied_left;
    output.target_right_permille = applied_right;
    output.command_left_permille = applied_left;
    output.command_right_permille = applied_right;
    output.have_ramp_time = 1u;
    output.last_ramp_ms = 0u;

    for (now_ms = 0u; now_ms <= 1000u; now_ms += 10u) {
        uint16_t previous_servo = servo.limited_us;
        uint16_t previous_left = applied_left;
        uint16_t previous_right = applied_right;
        uint16_t step = CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US;

        if (car_servo_safety_request_selected_with_slew(
                &servo, CAR_SERVO_SOURCE_FOLLOW, REVERSE_LEFT_US, CENTER_US,
                1u, now_ms, step, &io) == 0u) {
            printf("FAIL servo request at %lu ms; fault=%u\n",
                   (unsigned long)now_ms, servo.sticky_fault_code);
            failed = 1;
            break;
        }
        if (servo.limited_us != previous_servo) {
            uint16_t delta = abs_delta(servo.limited_us, previous_servo);
            if (delta > max_servo_step) max_servo_step = delta;
            if (first_limited_ms < 0) first_limited_ms = (int)now_ms;
        }
        if (car_follow_output_update(
                &output, now_ms, 500u,
                CARTEST_FOLLOW_DIFF_PERMILLE,
                CARTEST_FOLLOW_INNER_DIFF_PERMILLE,
                servo.limited_us, CENTER_US,
                CALIBRATED_LEFT_US, CALIBRATED_RIGHT_US,
                servo.active, servo.frame_ready, 1u,
                applied_left, applied_right,
                &left_command, &right_command) == 0u) {
            printf("FAIL Follow output update at %lu ms\n",
                   (unsigned long)now_ms);
            failed = 1;
            break;
        }
        if (abs_delta(left_command, previous_left) > max_pwm_step) {
            max_pwm_step = abs_delta(left_command, previous_left);
        }
        if (abs_delta(right_command, previous_right) > max_pwm_step) {
            max_pwm_step = abs_delta(right_command, previous_right);
        }
        applied_left = left_command;
        applied_right = right_command;

        if (servo_endpoint_ms < 0 && servo.limited_us == REVERSE_LEFT_US) {
            servo_endpoint_ms = (int)now_ms;
        }
        if (motor_endpoint_ms < 0 && applied_left ==
                (uint16_t)(500u - CARTEST_FOLLOW_INNER_DIFF_PERMILLE) &&
            applied_right ==
                (uint16_t)(500u + CARTEST_FOLLOW_DIFF_PERMILLE)) {
            motor_endpoint_ms = (int)now_ms;
        }
    }

    if (servo.requested_us != REVERSE_LEFT_US ||
        servo.limited_us != REVERSE_LEFT_US ||
        servo_endpoint_ms < 0 || motor_endpoint_ms < 0 ||
        max_servo_step > CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US ||
        max_pwm_step > CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE) {
        printf("FAIL timing bound or reversal endpoint\n");
        failed = 1;
    }

    {
        fake_servo_t delayed_fake = make_active_right_endpoint();
        car_servo_safety_io_t delayed_io = make_io(&delayed_fake);
        car_servo_safety_t delayed_servo = {0};
        car_follow_output_state_t delayed = {0};
        uint16_t delayed_left = 500u;
        uint16_t delayed_right = 500u;
        delayed_servo.active = 1u;
        delayed_servo.frame_ready = 1u;
        delayed_servo.limited_us = REVERSE_RIGHT_US;
        delayed_servo.compare_us = REVERSE_RIGHT_US;
        delayed_servo.have_slew_time = 1u;
        delayed_servo.last_slew_ms = 0u;
        delayed.startup_qualified = 1u;
        delayed.have_ramp_time = 1u;
        delayed.last_ramp_ms = 0u;
        if (car_servo_safety_request_selected_with_slew(
                &delayed_servo, CAR_SERVO_SOURCE_FOLLOW, REVERSE_LEFT_US,
                CENTER_US, 1u, 1000u,
                CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US,
                &delayed_io) == 0u ||
            abs_delta(delayed_servo.limited_us, REVERSE_RIGHT_US) !=
                CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US ||
            car_follow_output_update(
                &delayed, 1000u, 500u,
                CARTEST_FOLLOW_DIFF_PERMILLE,
                CARTEST_FOLLOW_INNER_DIFF_PERMILLE,
                REVERSE_RIGHT_US, CENTER_US,
                CALIBRATED_LEFT_US, CALIBRATED_RIGHT_US,
                1u, 1u, 1u, 500u, 500u,
                &delayed_left, &delayed_right) == 0u ||
            abs_delta(delayed_left, 500u) !=
                CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE ||
            abs_delta(delayed_right, 500u) !=
                CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE) {
            printf("FAIL delayed loop caught up multiple motor steps\n");
            failed = 1;
        }
    }

    {
        car_follow_output_state_t startup = {0};
        uint16_t startup_left = 0u;
        uint16_t startup_right = 0u;
        startup.have_ramp_time = 1u;
        startup.last_ramp_ms = 0u;
        if (car_follow_output_update(
                &startup, 1000u, 500u,
                CARTEST_FOLLOW_DIFF_PERMILLE,
                CARTEST_FOLLOW_INNER_DIFF_PERMILLE,
                CENTER_US, CENTER_US,
                CALIBRATED_LEFT_US, CALIBRATED_RIGHT_US,
                1u, 1u, 1u, 0u, 0u,
                &startup_left, &startup_right) == 0u ||
            startup.startup_qualified != 0u ||
            startup_left != CAR_FOLLOW_OUTPUT_STARTUP_RAMP_STEP_PERMILLE ||
            startup_right != CAR_FOLLOW_OUTPUT_STARTUP_RAMP_STEP_PERMILLE) {
            printf("FAIL unqualified startup/recovery did not stay at 10 permille\n");
            failed = 1;
        }
    }

    printf("TURN_RESPONSE profile_diff=%u inner_diff=%u servo_step_us=%u "
           "motor_step_permille=%u requested_at_ms=0 requested_us=%u "
           "first_limited_ms=%d servo_endpoint_ms=%d motor_endpoint_ms=%d "
           "start_left=%u start_right=%u final_left=%u final_right=%u "
           "max_servo_step_us=%u max_actual_side_pwm_step_permille=%u "
           "delayed_loop_single_step=1 result=%s\n",
           (unsigned)CARTEST_FOLLOW_DIFF_PERMILLE,
           (unsigned)CARTEST_FOLLOW_INNER_DIFF_PERMILLE,
           (unsigned)CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US,
           (unsigned)CAR_FOLLOW_OUTPUT_RAMP_STEP_PERMILLE,
           (unsigned)REVERSE_LEFT_US,
           first_limited_ms, servo_endpoint_ms, motor_endpoint_ms,
           (unsigned)(500u + CARTEST_FOLLOW_DIFF_PERMILLE),
           (unsigned)(500u - CARTEST_FOLLOW_INNER_DIFF_PERMILLE),
           (unsigned)applied_left, (unsigned)applied_right,
           (unsigned)max_servo_step, (unsigned)max_pwm_step,
           failed ? "FAIL" : "PASS");
    return failed ? 1 : 0;
}
