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
static void text_feed(const char *text,uint32_t now){
    for(size_t i=0;text[i];++i)car_wireless_uart_rx_isr((uint8_t)text[i],now);
    service(now);
}
static void raw_feed(const uint8_t *bytes,size_t length,uint32_t now){
    for(size_t i=0;i<length;++i)car_wireless_uart_rx_isr(bytes[i],now);
    service(now);
}
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
static uint32_t drain_camera_command(uint8_t opcode){
    car_wireless_parser_t p;car_wireless_frame_t f;uint8_t b;uint32_t sequence=0;
    car_wireless_parser_init(&p);
    while(car_wireless_camera_tx_byte(&b,1))
        if(car_wireless_parser_feed(&p,b,&f)==CAR_WIRELESS_PARSE_FRAME &&
           f.type==CAR_WIRELESS_COMMAND && f.payload_len>=1u && f.payload[0]==opcode)
            sequence=f.sequence;
    return sequence;
}
static void camera_ack(uint32_t sequence,uint8_t status,uint32_t detail,uint32_t now){
    car_wireless_frame_t f={0};f.type=CAR_WIRELESS_ACK;f.session=epoch;
    f.sequence=++camera_seq;f.payload_len=16;f.payload[0]=status;
    f.payload[1]=WIRELESS_OP_CAMERA_CAL;put32(f.payload+4,sequence);
    put32(f.payload+8,detail);car_wireless_camera_frame(&f,now);
}
static float last_scope[16];
static float drain_scope_value(unsigned index){
    uint8_t packet[72],b;size_t count=0;float value=0.0f;
    while(car_wireless_radio_tx_byte(&b)){
        if(!count && b!=0xaau)continue;
        packet[count++]=b;
        if(count==sizeof(packet)){
            if(packet[2]==0x10u && packet[3]==16u && index<16u) {
                memcpy(last_scope,packet+8u,sizeof last_scope);
                memcpy(&value,packet+8u+index*4u,sizeof(value));
            }
            count=0;
        }
    }
    return value;
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
    /* CALWIDTH is an assistant-only text trigger. Split input and CRLF are
       accepted, while a retransmitted accepted command is ignored. */
    car_wireless_init(92000,12346);service(92000);camera_ready(92000);
    text_feed("CAL",92010);text_feed("WIDTH",92011);text_feed("\r\n",92012);
    uint32_t ascii_cal_seq=drain_camera_command(WIRELESS_OP_CAMERA_CAL);assert(ascii_cal_seq);
    text_feed("CALWIDTH\n",92013);assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_EVENT;f.session=epoch;
    f.sequence=++camera_seq;f.payload_len=16;f.payload[0]=1;f.payload[1]=0;
    f.payload[2]=10;f.payload[4]=30;f.payload[6]=0x34;f.payload[7]=0x12;
    car_wireless_camera_frame(&f,92014);
    service(92050);assert(drain_scope_value(0)==1.0f);
    assert(last_scope[2]==10.0f && last_scope[3]==30.0f && last_scope[15]==4660.0f);
    camera_ack(ascii_cal_seq,WIRELESS_ACK_OK,64u|(70u<<16),92060);
    uint8_t page[8];assert(car_seekfree_debug_param_frame(page,sizeof page,7,3.0f));
    raw_feed(page,sizeof page,92061);service(92100);assert(drain_scope_value(0)==2.0f);
    assert(last_scope[2]==64.0f && last_scope[3]==70.0f);
    text_feed("CALWIDTH\n",92101);assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    service(92150);assert(drain_scope_value(0)==2.0f);
    assert(last_scope[2]==64.0f && last_scope[3]==70.0f);
    uint16_t before=car_wireless_settings()->wheel_scale_permille;
    assert(car_seekfree_debug_param_frame(page,sizeof page,0,500.0f));
    raw_feed(page,sizeof page,92151);assert(car_wireless_settings()->wheel_scale_permille==before);
    /* A new boot permits a new request, but output-active prerequisites
       reject it without forwarding a camera command. */
    car_wireless_init(93000,12347);service(93000);camera_ready(93000);
    snapshot.outputs_active=1;text_feed("CALWIDTH\n",93001);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));snapshot.outputs_active=0;
    /* Faulted and disconnected camera states are rejected before forwarding;
       a malformed C-prefix and binary bytes do not become CALWIDTH. */
    car_wireless_init(94000,12348);service(94000);camera_ready(94000);
    control.fault_latched=1;text_feed("CALWIDTH\n",94001);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));control.fault_latched=0;
    car_wireless_init(95000,12349);service(95000);
    text_feed("CALX",95001);uint8_t binary_noise[8]={0x55,0,0x20,7,0,0,0,0};
    raw_feed(binary_noise,sizeof binary_noise,95002);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    /* Once accepted, a camera failure is reported as failed and a silent
       camera is reported as timeout in the page-3 status channel. */
    car_wireless_init(96000,12350);service(96000);camera_ready(96000);
    text_feed("CALWIDTH\n",96001);uint32_t failed_seq=drain_camera_command(WIRELESS_OP_CAMERA_CAL);
    assert(failed_seq);camera_ack(failed_seq,WIRELESS_ACK_FAULT,0,96002);
    service(96050);assert(drain_scope_value(0)==4.0f);
    car_wireless_init(97000,12351);service(97000);camera_ready(97000);
    text_feed("CALWIDTH\n",97001);assert(drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    service(99000);assert(drain_scope_value(0)==4.0f);
    /* A live but non-finishing camera times out at the independent 65s deadline. */
    car_wireless_init(100000,12352);service(100000);camera_ready(100000);
    text_feed("CALWIDTH\n",100001);assert(drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    for(uint32_t at=101000;at<=166000;at+=1000){
        f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_HELLO_REPLY;
        f.session=epoch;f.sequence=++camera_seq;car_wireless_camera_frame(&f,at);
        service(at);drain_camera_command(0);if(at<166000)drain_scope_value(0);
    }
    drain_scope_value(0);service(166050);assert(drain_scope_value(0)==5.0f);
    /* No execution during a driving session; prerequisites can then be retried. */
    car_wireless_init(170000,12353);service(170000);camera_ready(170000);
    control.follow_session_active=1;text_feed("CALWIDTH\n",170001);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    control.follow_session_active=0;text_feed("CALWIDTH\n",170002);
    assert(drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    /* A valid D3 packet containing command text is never interpreted as ASCII. */
    car_wireless_init(180000,12354);service(180000);camera_ready(180000);
    f=(car_wireless_frame_t){0};f.type=CAR_WIRELESS_EVENT;
    f.payload_len=9;memcpy(f.payload,"CALWIDTH\n",9);feed(&f,180001);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    text_feed("CAL",180010);text_feed("WIDTH\n",180600);
    assert(!drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    text_feed("CAL",180700);text_feed("WIDTH\n",180800);
    assert(drain_camera_command(WIRELESS_OP_CAMERA_CAL));
    /* Raw transport diagnostics are read-only and preserve ordinary pages. */
    car_wireless_init(190000,12355);service(190000);camera_ready(190000);
    snapshot.imu_who_am_i_valid=1;snapshot.imu_who_am_i=255;
    snapshot.imu_who_am_i_reads=10;snapshot.camera_uart_rx_bytes=123;
    snapshot.camera_uart_errors=2;snapshot.camera_uart_crc_errors=3;
    snapshot.camera_uart_overflows=4;snapshot.camera_uart_tx_bytes=456;
    assert(car_seekfree_debug_param_frame(page,sizeof page,7,4.0f));
    raw_feed(page,sizeof page,190001);service(190050);drain_scope_value(0);
    assert(last_scope[0]==1 && last_scope[1]==255 && last_scope[2]==71);
    assert(last_scope[3]==10 && last_scope[6]==123 && last_scope[7]==2);
    assert(last_scope[8]==3 && last_scope[9]==4 && last_scope[10]==456 && last_scope[14]==4);
    before=car_wireless_settings()->wheel_scale_permille;
    assert(car_seekfree_debug_param_frame(page,sizeof page,0,500.0f));
    raw_feed(page,sizeof page,190051);assert(car_wireless_settings()->wheel_scale_permille==before);
    assert(get32((uint8_t[]){1,2,3,4})==0x04030201u);
    puts("wireless runtime: legacy replay/output gates; CALWIDTH split/CRLF/replay, parked/fault gates, progress, save ACK, disconnect, 65s timeout and binary isolation passed");
    return 0;
}
