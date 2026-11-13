#include "wireless_runtime.h"
#define main original_gyro_tests_main
#include "../gyro350/test_gyro350.c"
#undef main
unsigned test_saved_settings;
int main(void) {
    wireless_settings_t settings;
    wireless_param_entry_t entry={WIRELESS_PARAM_GYRO_RATE_MDPS,0};
    const unsigned rate=CAR_GYRO_RING_TARGET_YAW_RATE_MDPS;
    const unsigned maximum=rate>180000?rate:180000;
    for (unsigned saved=0;saved<2;saved++) {
        test_saved_settings=saved;
        car_wireless_init(0,1234);
        assert(car_wireless_settings()->gyro_rate_mdps==rate);
        assert(car_wireless_settings()->wheel_scale_permille==(saved?750:1000));
        assert(wireless_settings_is_valid(car_wireless_settings()));
        for (int side=-1;side<=1;side+=2) {
            ring_fixture_t fixture;
            ring_arm(&fixture,100,50,(int8_t)side);
            const int expected=250+(int)rate*2/1000;
            assert(fixture.state.control_error_norm==(side<0?-expected:expected));
        }
        test_entry_straight_delay(-1,100);
        test_entry_straight_delay(1,UINT32_MAX-100);
    }
    settings=*car_wireless_settings();entry.value=(int32_t)maximum;
    assert(wireless_settings_apply_group(&settings,WIRELESS_SETTINGS_GROUP_GYRO,&entry,1,1)==WIRELESS_SETTINGS_OK);
    entry.value=(int32_t)maximum+1;
    assert(wireless_settings_apply_group(&settings,WIRELESS_SETTINGS_GROUP_GYRO,&entry,1,1)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    assert(settings.gyro_rate_mdps==maximum);
    entry.value=29999;
    assert(wireless_settings_apply_group(&settings,WIRELESS_SETTINGS_GROUP_GYRO,&entry,1,1)==WIRELESS_SETTINGS_OUT_OF_RANGE);
    unsigned char slot[WIRELESS_SETTINGS_SLOT_BYTES];
    uint32_t generation=0;
    settings=*car_wireless_settings();
    assert(wireless_settings_encode_slot(&settings,12,slot,sizeof slot));
    assert(wireless_settings_decode_slot(slot,sizeof slot,&settings,&generation));
    assert(settings.gyro_rate_mdps==rate && generation==12);
    printf("wireless boot/default/saved/rebase/rate bound/real ring target: %u PASS\n",rate);
    return 0;
}
