#include "wireless_runtime.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static car_control_t control;
static car_config_t config;
static car_wireless_snapshot_t snapshot;
static uint32_t seq=0, camera_seq=0, epoch;
static void put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(i*8));}
static uint32_t get32(const uint8_t *p){return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void service(uint32_t now){snapshot.now_ms=now;car_wireless_service(&snapshot,&control,&config);}
static void feed(car_wireless_frame_t *f,uint32_t now){
    uint8_t bytes[CAR_WIRELESS_MAX_FRAME];size_t n;assert(car_wireless_encode(f,bytes,sizeof bytes,&n));
    for(size_t i=0;i<n;i++)car_wireless_uart_rx_isr(bytes[i],now);
    service(now);
}
static car_wireless_frame_t cmd(uint8_t op,uint32_t now){
    car_wireless_frame_t f={0};f.type=CAR_WIRELESS_COMMAND;f.payload_len=4;
    f.payload[0]=op;f.session=car_wireless_session;f.sequence=++seq;f.timestamp_ms=now;return f;
}
static unsigned drain(uint8_t type,unsigned length){
    car_wireless_parser_t p;car_wireless_frame_t f;uint8_t b;unsigned found=0;
    car_wireless_parser_init(&p);
    while(car_wireless_radio_tx_byte(&b))if(car_wireless_parser_feed(&p,b,&f)==CAR_WIRELESS_PARSE_FRAME)
        if(f.type==type && f.payload_len==length)found++;
    return found;
}
static void camera_ready(uint32_t now){
    car_wireless_parser_t p;car_wireless_frame_t f;uint8_t b;
    car_wireless_parser_init(&p);
    while(car_wireless_camera_tx_byte(&b,1))if(car_wireless_parser_feed(&p,b,&f)==CAR_WIRELESS_PARSE_FRAME && f.type==CAR_WIRELESS_HELLO)epoch=f.session;
    assert(epoch);memset(&f,0,sizeof f);f.type=CAR_WIRELESS_HELLO_REPLY;f.session=epoch;f.sequence=++camera_seq;
    car_wireless_camera_frame(&f,now);
}
int main(void){
    snapshot.button_released=snapshot.startup_released=1;config.motor_hardware_accepted=1;
    car_wireless_init(100,12345);service(100);camera_ready(100);
    car_wireless_frame_t f={0};f.type=CAR_WIRELESS_HELLO;f.payload_len=4;put32(f.payload,456);
    feed(&f,101);assert(car_wireless_session);drain(0,0);
    f=cmd(WIRELESS_OP_MODE,102);f.payload_len=5;f.payload[4]=WIRELESS_TOOL_MODE_COMPLETE;feed(&f,102);drain(0,0);
    f=cmd(WIRELESS_OP_START,103);feed(&f,103);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    feed(&f,104);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_START,1);drain(0,0);
    f=cmd(WIRELESS_OP_MODE,105);f.payload_len=5;f.payload[4]=WIRELESS_TOOL_MODE_ASSISTANT;feed(&f,105);
    uint8_t tuning[8];assert(car_seekfree_debug_param_frame(tuning,sizeof tuning,0,422.0f));
    /* 422.0 contains D3 in its IEEE754 payload; it is not an outer-frame start. */
    for(unsigned i=0;i<8;i++)car_wireless_uart_rx_isr(tuning[i],106);
    service(106);assert(car_wireless_settings()->wheel_scale_permille==422);
    f=cmd(WIRELESS_OP_MODE,107);f.payload_len=5;f.payload[4]=WIRELESS_TOOL_MODE_COMPLETE;feed(&f,107);drain(0,0);
    f=cmd(WIRELESS_OP_TEST_MOTOR,110);f.payload_len=9;f.payload[4]=3;f.payload[5]=144;f.payload[6]=1;f.payload[7]=100;f.payload[8]=0;
    feed(&f,110);assert(car_wireless_test().active);assert(car_wireless_test_allowed(110));
    snapshot.outputs_active=1;feed(&f,150);assert(car_wireless_test().started_ms==110);assert(car_wireless_test_allowed(150));
    service(210);assert(!car_wireless_test().active);assert(!car_wireless_test_allowed(210));snapshot.outputs_active=0;
    service(211);drain(0,0);
    f=cmd(WIRELESS_OP_TEST_SERVO,220);f.payload_len=8;f.payload[4]=0xc4;f.payload[5]=9;f.payload[6]=0xe8;f.payload[7]=3;
    feed(&f,220);assert(car_wireless_test().active);car_wireless_local_stop(221);service(221);
    assert(!car_wireless_test_allowed(221));assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    service(222);drain(0,0);
    f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_STATE;f.session=epoch;f.sequence=++camera_seq;f.payload_len=36;
    car_wireless_camera_frame(&f,223);assert(drain(CAR_WIRELESS_STATE,36)==1);
    f=cmd(WIRELESS_OP_CAPTURE,230);f.payload[1]=1;feed(&f,230);assert(car_wireless_capture_busy());
    /* A successful camera ACK with no image must not leave capture busy forever. */
    car_wireless_parser_t p;car_wireless_frame_t out;uint8_t b;uint32_t capture_seq=0;
    car_wireless_parser_init(&p);
    while(car_wireless_camera_tx_byte(&b,1))if(car_wireless_parser_feed(&p,b,&out)==CAR_WIRELESS_PARSE_FRAME && out.type==CAR_WIRELESS_COMMAND && out.payload[0]==WIRELESS_OP_CAPTURE)capture_seq=out.sequence;
    assert(capture_seq);f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_ACK;f.session=epoch;f.sequence=++camera_seq;f.payload_len=16;f.payload[0]=WIRELESS_ACK_OK;put32(f.payload+4,capture_seq);
    car_wireless_camera_frame(&f,231);
    f.type=CAR_WIRELESS_HELLO_REPLY;f.sequence=++camera_seq;
    car_wireless_camera_frame(&f,60230);service(60231);assert(!car_wireless_capture_busy());
    /* Calibration may collect for 60 seconds; keep the MCU busy beyond 30. */
    f=cmd(WIRELESS_OP_CAMERA_CAL,60240);f.payload[1]=1;feed(&f,60240);
    assert(car_wireless_capture_busy());
    uint32_t cal_seq=0;car_wireless_parser_init(&p);
    while(car_wireless_camera_tx_byte(&b,1))if(car_wireless_parser_feed(&p,b,&out)==CAR_WIRELESS_PARSE_FRAME && out.type==CAR_WIRELESS_COMMAND && out.payload[0]==WIRELESS_OP_CAMERA_CAL)cal_seq=out.sequence;
    assert(cal_seq);f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_HELLO_REPLY;f.session=epoch;f.sequence=++camera_seq;
    car_wireless_camera_frame(&f,91240);service(91240);assert(car_wireless_capture_busy());
    f.type=CAR_WIRELESS_ACK;f.sequence=++camera_seq;f.payload_len=16;
    f.payload[0]=WIRELESS_ACK_OK;f.payload[1]=WIRELESS_OP_CAMERA_CAL;put32(f.payload+4,cal_seq);
    car_wireless_camera_frame(&f,91241);assert(!car_wireless_capture_busy());
    assert(get32((uint8_t[]){1,2,3,4})==0x04030201u);
    puts("wireless runtime: replay, motor deadline, local stop, camera state and missing-image timeout passed");
    return 0;
}
