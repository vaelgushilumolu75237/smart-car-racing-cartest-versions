#include "motor_max_test.h"
#include "manual_run_button.h"
#include "speed_observer.h"
#include <assert.h>
#include <stdio.h>
static motor_max_test_t s;
static void timing_and_latches(void) {
    motor_max_init(&s); motor_max_tick(&s, 100, 0, 0); assert(s.phase==MOTOR_MAX_READY);
    motor_max_start(&s,100); motor_max_tick(&s,1100,1,0); assert(s.duty_permille==500);
    motor_max_tick(&s,2099,1,0); assert(s.duty_permille==999);
    motor_max_tick(&s,2100,1,0); assert(s.phase==MOTOR_MAX_HOLD && s.duty_permille==1000);
    motor_max_tick(&s,7099,1,0); assert(s.duty_permille==1000);
    motor_max_tick(&s,7100,1,0); assert(s.phase==MOTOR_MAX_DONE && !s.duty_permille);
    motor_max_start(&s,8000); motor_max_tick(&s,9000,1,0); assert(s.phase==MOTOR_MAX_DONE);
    motor_max_init(&s); motor_max_start(&s,0); motor_max_tick(&s,100,1,0);
    motor_max_tick(&s,101,0,0); assert(s.phase==MOTOR_MAX_FAULT && !s.duty_permille && s.stop_reason==1);
    motor_max_start(&s,200); assert(s.phase==MOTOR_MAX_FAULT);
    for(unsigned f=2;f<=5;++f) {
        motor_max_init(&s); motor_max_start(&s,0); motor_max_tick(&s,100,1,f);
        assert(s.phase==MOTOR_MAX_FAULT && !s.duty_permille && s.stop_reason==f);
    }
    motor_max_init(&s); motor_max_start(&s,0xfffff000u);
    motor_max_tick(&s,0xfffff000u+2000u,1,0); assert(s.duty_permille==1000);
    motor_max_tick(&s,0xfffff000u+7000u,1,0); assert(s.phase==MOTOR_MAX_DONE);
}
static void stats_and_trace(void) {
    motor_max_init(&s); motor_max_start(&s,0); motor_max_tick(&s,2000,1,0);
    motor_max_sample(&s,2490,10,99999,99999,1000); assert(!s.full_sample_ms);
    for (unsigned t=2510;t<=3000;t+=10) motor_max_sample(&s,t,10,12345,11000,1000);
    assert(s.full_sample_ms==500 && s.peak500_left_millirps==12345 && s.peak500_right_millirps==11000);
    assert(s.mean_full_left_millirps==12345 && s.mean_full_right_millirps==11000);
    unsigned n=s.sample_count;
    motor_max_sample(&s,3010,0,10,10,1000); motor_max_sample(&s,3010,101,10,10,1000);
    motor_max_sample(&s,3010,10,-1,10,1000); motor_max_sample(&s,3010,10,10,10,1001);
    assert(s.sample_count==n);
    for(unsigned i=0;i<1000;++i) motor_max_sample(&s,3010+i*10,10,12000,10000,1000);
    assert(s.sample_count==MOTOR_MAX_TRACE_CAPACITY);
    motor_max_stop(&s,7000,0); n=s.sample_count;
    motor_max_sample(&s,7010,10,999999,999999,1000); assert(s.sample_count==n);
}
static void button_and_actual_observer(void) {
    car_manual_button_t b; car_manual_button_init(&b,0,0);
    assert(car_manual_button_note_low_edge(&b,0,0)==CAR_MANUAL_BUTTON_EVENT_NONE);
    car_manual_button_poll_high(&b,1,10); car_manual_button_poll_high(&b,1,30);
    assert(car_manual_button_note_low_edge(&b,0,0)==CAR_MANUAL_BUTTON_EVENT_START);
    assert(car_manual_button_note_low_edge(&b,0,0)==CAR_MANUAL_BUTTON_EVENT_NONE);
    car_manual_button_poll_high(&b,1,40); car_manual_button_poll_high(&b,1,60);
    assert(car_manual_button_note_low_edge(&b,0,0)==CAR_MANUAL_BUTTON_EVENT_STOP);
    car_speed_observer_t o; car_speed_observer_result_t r;
    car_speed_observer_init(&o,0,0,0,60000,1,60000,-1);
    assert(car_speed_observer_update(&o,10,7200,-6600,1,&r)==CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    assert(r.valid && r.left_millirps==12000 && r.right_millirps==11000);
    for(unsigned t=20;t<=340;t+=10) car_speed_observer_update(&o,t,7200,-6600,1,&r);
    assert(!r.valid && r.status==CAR_SPEED_OBSERVER_STATUS_STALL);
}
int main(void) {timing_and_latches();stats_and_trace();button_and_actual_observer();puts("motor max test PASS: timing, one-shot, faults, wrap, stats, trace, button and encoder stall");return 0;}
