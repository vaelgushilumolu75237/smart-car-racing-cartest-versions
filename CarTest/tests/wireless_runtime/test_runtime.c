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
static void assistant_fixture(uint32_t now,uint32_t boot_seed){
    memset(&control,0,sizeof control);memset(&snapshot,0,sizeof snapshot);
    memset(&config,0,sizeof config);snapshot.button_released=snapshot.startup_released=1;
    control.imu_ready=1;config.motor_hardware_accepted=1;
    car_wireless_init(now,boot_seed);service(now);drain(0,0);
}
static uint32_t assistant_arm(uint32_t now){
    text_feed("ARM\r\n",now);service(now+50u);drain_scope_value(2);
    assert(last_scope[0]==1 && last_scope[14]==5);
    uint32_t ticket=(uint32_t)last_scope[2];
    assert(ticket>=1000000u && ticket<=9999999u && (float)ticket==last_scope[2]);
    return ticket;
}
static void assistant_start(uint32_t ticket,uint32_t now){
    char line[32];snprintf(line,sizeof line,"START %07u\r\n",ticket);text_feed(line,now);
}
static void assistant_tests(void){
    uint32_t ticket,new_ticket;car_wireless_frame_t f={0};uint8_t page[8];
    assistant_fixture(200000,30000);
    assistant_start(1234567u,200001);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    ticket=assistant_arm(200100);assistant_start(ticket,200160);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_START,0);
    service(200210);drain_scope_value(0);assert(last_scope[0]==4 && last_scope[1]==15);
    assistant_start(ticket,200220);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    /* A consumed ticket never queues a deferred start on retry/readiness. */
    new_ticket=assistant_arm(200300);assert(new_ticket!=ticket);ticket=new_ticket;
    char line[32];snprintf(line,sizeof line,"RT %07u\n",ticket);
    text_feed("STA",200360);text_feed(line,200370);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_START,1);
    service(200420);drain_scope_value(0);assert(last_scope[0]==3 && last_scope[1]==0);
    assistant_start(ticket,200430);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    control.follow_session_active=1;snapshot.outputs_active=1;
    text_feed("STOP\n",200440);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_STOP,1);
    control.follow_session_active=0;snapshot.outputs_active=0;
    service(200500);drain_scope_value(0);assert(last_scope[0]==5 && last_scope[15]==0);
    assistant_start(ticket,200510);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    text_feed("STOP\n",200520);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_STOP,1);
    /* Expiry, PE7, faults, power-up release, tests/calibration and hardware
       acceptance cannot be bypassed by either ARM or a previously issued code. */
    assistant_fixture(210000,30001);ticket=assistant_arm(210001);
    assistant_start(ticket,240001);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    service(240051);drain_scope_value(0);assert(last_scope[2]==0);
    assistant_fixture(250000,30002);ticket=assistant_arm(250001);
    car_wireless_local_stop(250060);service(250060);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    assistant_start(ticket,250061);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(260000,30003);snapshot.button_released=0;text_feed("ARM\n",260001);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    snapshot.button_released=1;service(260010);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_start(1234567u,260011);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(270000,30004);ticket=assistant_arm(270001);
    control.fault_latched=1;assistant_start(ticket,270060);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    control.fault_latched=0;assistant_start(ticket,270061);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(280000,30005);snapshot.startup_released=0;text_feed("ARM\n",280001);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    service(280051);drain_scope_value(0);assert(last_scope[2]==0);
    assistant_fixture(290000,30006);config.motor_hardware_accepted=0;text_feed("ARM\n",290001);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    service(290051);drain_scope_value(0);assert(last_scope[0]==4 && last_scope[1]==9);
    assistant_fixture(300000,30007);snapshot.calibration_busy=1;text_feed("ARM\n",300001);
    service(300051);drain_scope_value(0);assert(last_scope[0]==4 && last_scope[1]==8);
    assistant_fixture(310000,30008);ticket=assistant_arm(310001);
    control.follow_session_active=1;assistant_start(ticket,310060);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(320000,30009);ticket=assistant_arm(320001);
    snprintf(line,sizeof line,"START %07u\n",ticket);
    for(size_t i=0;line[i];++i)car_wireless_uart_rx_isr((uint8_t)line[i],320060);
    car_wireless_uart_error_isr();service(320060);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_start(ticket,320061);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(330000,30010);ticket=assistant_arm(330001);
    for(unsigned i=0;i<600;++i)car_wireless_uart_rx_isr('A',330060);
    service(330060);assistant_start(ticket,330061);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    /* Exact bounded line parsing, partial timeout, stale RX, native parameter
       isolation, and a D3 payload containing STOP must not create commands. */
    assistant_fixture(340000,30011);ticket=assistant_arm(340001);
    text_feed("ASTOP\n",340060);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    text_feed("AAAAAAAAAAAAAAAAAAAAAAAAAAAASTOP\n",340061);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    snprintf(line,sizeof line,"RT %07u\n",ticket);
    text_feed("STA",340062);text_feed(line,340700);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    f.type=CAR_WIRELESS_EVENT;f.payload_len=5;memcpy(f.payload,"STOP\n",5);feed(&f,340701);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assert(car_seekfree_debug_param_frame(page,sizeof page,7,5.0f));raw_feed(page,sizeof page,340702);
    snprintf(line,sizeof line,"START %07u\n",ticket);
    for(size_t i=0;line[i];++i)car_wireless_uart_rx_isr((uint8_t)line[i],340703);
    service(340804);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_start(ticket,340805);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_START,1);
    /* A new binary session invalidates the assistant code. Text in complete
       mode cannot trigger operations. */
    assistant_fixture(350000,30012);ticket=assistant_arm(350001);
    memset(&f,0,sizeof f);f.type=CAR_WIRELESS_HELLO;f.payload_len=4;put32(f.payload,789);
    feed(&f,350060);assistant_start(ticket,350061);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    f=cmd(WIRELESS_OP_MODE,350062);f.payload_len=5;f.payload[4]=WIRELESS_TOOL_MODE_COMPLETE;
    feed(&f,350062);text_feed("STOP\nARM\n",350063);
    assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
    assistant_fixture(0xffffff80u,30013);ticket=assistant_arm(0xffffff81u);
    assistant_start(ticket,0x20u);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_START);
    car_wireless_effect_result(CAR_WIRELESS_EFFECT_START,1);
    puts("SeekFree text: one-use ticket, result, STOP/replay, 30s expiry, PE7/fault/release/busy/motor gates, UART/overflow, exact/split/stale/binary isolation, session replacement and clock wrap passed");
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
    /* A recoverable receive fault stops physical outputs in commissioning.
       Telemetry must not turn it into NORMAL_STOP and erase the driver's
       request, including when no PC session has ever been opened. */
    const car_fault_t recoverable[] = {CAR_FAULT_PROTOCOL, CAR_FAULT_SEQUENCE,
        CAR_FAULT_VISION_INVALID, CAR_FAULT_VISION_TIMEOUT, CAR_FAULT_UART,
        CAR_FAULT_RX_OVERFLOW, CAR_FAULT_RX_STALE};
    for(unsigned i=0;i<sizeof(recoverable)/sizeof(recoverable[0]);++i){
        assistant_fixture(300000u+i*1000u,40000u+i);
        control.manual_run_request=control.follow_session_active=1u;
        control.mode=CAR_MODE_RECOVERY_WAIT;control.fault=recoverable[i];
        service(300001u+i*1000u);
        assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
        assert(control.manual_run_request && control.follow_session_active);
        text_feed("ARM\n",300002u+i*1000u);
        assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
        control.fault=CAR_FAULT_NONE;service(300003u+i*1000u);
        assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_NONE);
        /* An explicit operator stop still cancels during recovery. */
        text_feed("STOP\n",300004u+i*1000u);
        assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    }
    assistant_fixture(320000,40010);
    control.manual_run_request=control.follow_session_active=1;
    control.fault=CAR_FAULT_ENCODER_INIT;control.fault_latched=1;
    service(320001);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    assistant_fixture(321000,40011);
    control.manual_run_request=control.follow_session_active=1;
    control.fault=CAR_FAULT_PROTOCOL;snapshot.button_released=0;
    service(321001);assert(car_wireless_take_effect()==CAR_WIRELESS_EFFECT_STOP);
    assistant_tests();
    puts("wireless runtime: legacy replay/output gates; CALWIDTH split/CRLF/replay, parked/fault gates, progress, save ACK, disconnect, 65s timeout and binary isolation passed");
    return 0;
}
