#include "ackermann_control.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    const unsigned requests[] = {0,267,1000};
    const unsigned pwms[] = {1100,1315,1530,2015,2500};
    const unsigned expected_base[] = {5600,5280,4400};
    car_ackermann_control_state_t s;
    car_ackermann_control_output_t o;
    for(unsigned r=0;r<3;++r) for(unsigned a=0;a<5;++a) {
        car_ackermann_control_reset(&s);
        assert(s.fastest_target_millirps==5600u*CARTEST_SPEED_PERCENT/100u);
        assert(car_ackermann_control_update(&s,0,2,requests[r],pwms[a],1530,1100,2500,&o));
        assert(car_ackermann_control_update(&s,200,2,requests[r],pwms[a],1530,1100,2500,&o));
        assert(o.fastest_target_millirps==expected_base[r]*CARTEST_SPEED_PERCENT/100u);
        assert(o.left_target_millirps>0 && o.right_target_millirps>0);
        assert(o.left_target_millirps<=o.fastest_target_millirps);
        assert(o.right_target_millirps<=o.fastest_target_millirps);
        if(a==2) assert(o.left_target_millirps==o.right_target_millirps);
        printf("%u,%u,%ld,%ld,%ld,%u\n",requests[r],pwms[a],(long)o.fastest_target_millirps,(long)o.left_target_millirps,(long)o.right_target_millirps,o.ratio_q15);
    }
    car_ackermann_control_reset(&s);
    assert(!car_ackermann_control_update(&s,0,2,1001,1530,1530,1100,2500,&o));
    assert(CAR_ACKERMANN_REQUEST_RAMP_DOWN_MS==200u && CAR_ACKERMANN_REQUEST_RAMP_UP_MS==800u);
    return 0;
}
