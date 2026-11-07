#include "wireless_runtime.h"
#if CARTEST_WIRELESS_ENABLED
#include "wireless_flash_stm32.h"
#include <string.h>
#include <math.h>

/* All queues contain whole packets. An active packet is never interleaved. */
enum { RX_N=512, TX_N=16, HI_N=4, CAM_N=4, IMAGE_HEADER=18, IMAGE_TTL_MS=60000 };
typedef struct { uint16_t size, pos; uint8_t data[CAR_WIRELESS_MAX_FRAME]; } packet_t;
typedef struct { uint8_t head, tail, count; packet_t packet[TX_N]; } queue_t;
typedef struct { uint32_t pc_sequence, camera_sequence, deadline;
    uint8_t opcode, active; uint16_t vision_p, vision_d; } forwarding_t;
static queue_t radio_queue, ack_queue, camera_queue, image_queue;
static packet_t radio_active, camera_active;
static volatile uint16_t rx_head, rx_tail;
static uint8_t rx_data[RX_N];
static uint32_t rx_time[RX_N];
static car_wireless_parser_t radio_parser;
static wireless_control_t engine;
static car_wireless_snapshot_t last_snapshot;
static car_control_t *vehicle;
static uint32_t seed, serial, session_counter, last_rx_ms, last_state_ms;
static uint32_t last_diag_ms, camera_epoch, camera_last_ms, camera_rx_seq;
static uint32_t camera_tx_seq, camera_last_hello_ms, camera_count;
static uint8_t camera_have_seq, camera_ready, initialized, scope_page;
static volatile uint8_t local_stop_latched;
static car_wireless_effect_t effect;
static wireless_control_ack_t effect_ack;
static forwarding_t pending[4];
static wireless_flash_writer_t flash_writer;
static uint8_t flash_selected_slot, effect_ack_valid;
static uint32_t flash_generation, last_vision_generation;
static uint16_t synced_vision_p, synced_vision_d;
static wireless_control_ack_t save_ack;
static uint8_t seekfree_rx[8], seekfree_count;
/* Separate D2 SRAM2 region, initialized by metadata rather than .bss clearing. */
#if defined(__arm__) || defined(__thumb__)
__attribute__((section(".wireless_image"),aligned(32)))
#endif
static uint8_t image_data[76812];
static uint32_t image_id, image_total, image_offset, image_started_ms;
static uint32_t capture_started_ms, vision_sync_attempt_ms;
static uint32_t image_wire_size, image_wire_pos;
static uint16_t image_width, image_height;
static uint8_t image_format, image_receiving, capture_pending;
volatile uint32_t car_wireless_rx_overflow, car_wireless_uart_errors;
volatile uint32_t car_wireless_tx_dropped, car_wireless_camera_dropped;
volatile uint32_t car_wireless_session, car_wireless_packets;

static uint16_t u16(const uint8_t *p) { return p[0] | (uint16_t)p[1]<<8; }
static uint32_t u32(const uint8_t *p) { return u16(p) | (uint32_t)u16(p+2)<<16; }
static void put32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);
}
static uint8_t push(queue_t *q, unsigned limit, const uint8_t *p, size_t n) {
    packet_t *slot;
    if (!n || n>CAR_WIRELESS_MAX_FRAME || q->count>=limit) return 0;
    slot=&q->packet[q->head];memcpy(slot->data,p,n);slot->size=(uint16_t)n;slot->pos=0;
    q->head=(q->head+1u)%limit;++q->count;return 1;
}
static uint8_t pop(queue_t *q, unsigned limit, packet_t *p) {
    if (!q->count) return 0;
    *p=q->packet[q->tail];q->tail=(q->tail+1u)%limit;--q->count;return 1;
}
static uint8_t send_frame(queue_t *q,unsigned limit,car_wireless_frame_t *f) {
    uint8_t bytes[CAR_WIRELESS_MAX_FRAME];size_t n;
    if (!car_wireless_encode(f,bytes,sizeof(bytes),&n)) return 0;
    return push(q,limit,bytes,n);
}
static car_wireless_frame_t frame(uint8_t type,uint32_t now) {
    car_wireless_frame_t f={0};f.type=type;f.sequence=++serial;
    f.session=engine.session;f.timestamp_ms=now;return f;
}
static void emit_ack(wireless_control_ack_t *ack,uint32_t now) {
    car_wireless_frame_t f=frame(CAR_WIRELESS_ACK,now);f.payload_len=16;
    f.payload[0]=ack->status;f.payload[1]=ack->opcode;f.payload[2]=ack->target;f.payload[3]=ack->flags;
    put32(f.payload+4,ack->request_seq);put32(f.payload+8,ack->detail);
    put32(f.payload+12,ack->settings_generation);
    if (!send_frame(&ack_queue,HI_N,&f)) ++car_wireless_tx_dropped;
}
static void cache_final(wireless_control_ack_t *ack,uint32_t now) {
    if (engine.last_command_seq==ack->request_seq) engine.cached_ack=*ack;
    emit_ack(ack,now);
}
static uint8_t forward_camera(car_wireless_frame_t *f,uint32_t pc_seq,uint8_t opcode,uint32_t now) {
    unsigned i;
    if (!camera_ready || now-camera_last_ms>1000u) return 0;
    for (i=0;i<4 && pending[i].active;++i) {}
    if (i==4) return 0;
    f->type=CAR_WIRELESS_COMMAND;f->session=camera_epoch;f->sequence=++camera_tx_seq;
    f->timestamp_ms=now;f->payload[1]=WIRELESS_TARGET_CAMERA;
    if (!send_frame(&camera_queue,CAM_N,f)) return 0;
    pending[i]=(forwarding_t){pc_seq,f->sequence,
        now+(opcode==WIRELESS_OP_CAMERA_CAL?65000u:2000u),opcode,1,0,0};
    if (pc_seq==0 && opcode==WIRELESS_OP_SET_PARAMS) {
        pending[i].vision_p=engine.settings.vision_gain_permille;
        pending[i].vision_d=engine.settings.vision_d_gain_permille;
    }
    return 1;
}
static void get_settings(uint32_t now) {
    car_wireless_frame_t f=frame(CAR_WIRELESS_CONFIG,now);
    /* Portable record; GUI uses the same schema and CRC as persistent storage. */
    f.payload_len=(uint16_t)wireless_settings_encode_slot(&engine.settings,
        engine.settings_generation,f.payload,sizeof(f.payload));
    if (!send_frame(&ack_queue,HI_N,&f)) ++car_wireless_tx_dropped;
}
static void command_frame(car_wireless_frame_t *f,uint32_t now) {
    wireless_control_ack_t ack;
    wireless_ack_status_t status;
    if (f->payload_len>=4 && f->payload[0]==WIRELESS_OP_PAGE) {
        /* PAGE uses the normal replay/session gate; no output permission. */
        uint8_t original=f->payload[0];f->payload[0]=WIRELESS_OP_INFO;
        status=wireless_control_command(&engine,f->sequence,f->session,f->timestamp_ms,now,
            f->payload,f->payload_len,&ack);f->payload[0]=original;ack.opcode=original;
        if (status==WIRELESS_ACK_OK && f->payload_len==5 && f->payload[4]<3) scope_page=f->payload[4];
        else if (status==WIRELESS_ACK_OK) ack.status=WIRELESS_ACK_RANGE;
        cache_final(&ack,now);return;
    }
    status=wireless_control_command(&engine,f->sequence,f->session,f->timestamp_ms,now,
        f->payload,f->payload_len,&ack);
    /* A cached PENDING acknowledgement is not a second execution request. */
    if (ack.flags & WIRELESS_ACK_FLAG_DUPLICATE) { emit_ack(&ack,now); return; }
    if (status==WIRELESS_ACK_FORWARD ||
        (status==WIRELESS_ACK_PENDING && (ack.opcode==WIRELESS_OP_CAMERA_CAL || ack.opcode==WIRELESS_OP_CAPTURE))) {
        (void)wireless_control_take_camera_cal(&engine);(void)wireless_control_take_capture(&engine);
        if (ack.opcode==WIRELESS_OP_CAPTURE && (!engine.runtime.parked || engine.test.active)) ack.status=WIRELESS_ACK_NOT_PARKED;
        else if (forward_camera(f,ack.request_seq,ack.opcode,now)) {
            ack.status=WIRELESS_ACK_PENDING;
            if (ack.opcode==WIRELESS_OP_CAPTURE) {capture_pending=1;capture_started_ms=now;}
        } else ack.status=WIRELESS_ACK_NOT_READY;
        cache_final(&ack,now);return;
    }
    if (status==WIRELESS_ACK_PENDING && (ack.opcode==WIRELESS_OP_START || ack.opcode==WIRELESS_OP_IMU_CAL)) {
        if (effect!=CAR_WIRELESS_EFFECT_NONE || car_wireless_capture_busy() || engine.test.active) {
            ack.status=WIRELESS_ACK_BUSY;(void)wireless_control_take_start(&engine);
            (void)wireless_control_take_imu_cal(&engine);
        } else {effect=ack.opcode==WIRELESS_OP_START?CAR_WIRELESS_EFFECT_START:CAR_WIRELESS_EFFECT_IMU_CAL;effect_ack=ack;effect_ack_valid=1;}
    }
    if (status==WIRELESS_ACK_OK && ack.opcode==WIRELESS_OP_STOP) {
        effect=CAR_WIRELESS_EFFECT_STOP;effect_ack=ack;effect_ack_valid=1;ack.status=WIRELESS_ACK_PENDING;
    }
    if (status==WIRELESS_ACK_PENDING && ack.opcode==WIRELESS_OP_SAVE && wireless_control_take_save(&engine)) {
        if(wireless_flash_begin_save(&flash_writer,&engine.settings,flash_generation+1u,(uint8_t)(flash_selected_slot^1u)))save_ack=ack;
        else ack.status=WIRELESS_ACK_BUSY;
    }
    /* Asynchronous SAVE is completed by the flash adapter, never by this ACK. */
    if (status==WIRELESS_ACK_OK && (ack.opcode==WIRELESS_OP_GET || ack.opcode==WIRELESS_OP_INFO ||
        ack.opcode==WIRELESS_OP_SET_PARAMS || ack.opcode==WIRELESS_OP_DEFAULT)) get_settings(now);
    cache_final(&ack,now);
}
static void seekfree_byte(uint8_t b,uint32_t now) {
    float value;uint8_t checksum;unsigned i,id=0,group=0;
    static const uint8_t ids[3][7]={{1,2,3,17,4,5,6},{7,8,9,10,11,12,13},{14,15,16,0,0,0,0}};
    wireless_param_entry_t entry;
    if (!seekfree_count && b!=0x55) return;
    seekfree_rx[seekfree_count++]=b;
    if (seekfree_count<8) return;
    seekfree_count=0;checksum=0xff;
    for (i=0;i<8;++i) if (i!=1) checksum=(uint8_t)(checksum+seekfree_rx[i]);
    if (checksum!=seekfree_rx[1] || seekfree_rx[2]!=0x20 || seekfree_rx[3]>7) return;
    memcpy(&value,seekfree_rx+4,4);if (!isfinite(value)) return;
    if (seekfree_rx[3]==7) {if(value>=0 && value<=2 && value==(float)(int)value) scope_page=(uint8_t)value;return;}
    id=ids[scope_page][seekfree_rx[3]];if(!id || value < -1000000 || value>1000000 || value!=(float)(int32_t)value) return;
    group=(id==2 || id==6 || id==7)?2:(id>=8 && id<=10)?3:(id>=11 && id<=16)?4:1;
    entry.id=(uint8_t)id;entry.value=(int32_t)value;
    if ((group>=3 && !engine.runtime.parked) || engine.test.active || car_wireless_capture_busy()) return;
    if (wireless_settings_apply_group(&engine.settings,(uint8_t)group,&entry,1,engine.runtime.follow_active)==WIRELESS_SETTINGS_OK)
        ++engine.settings_generation;
    (void)now;
}
void car_wireless_init(uint32_t now,uint32_t boot_seed) {
    wireless_settings_t settings;uint32_t gen=0;
    memset(&radio_queue,0,sizeof(radio_queue));memset(&ack_queue,0,sizeof(ack_queue));
    memset(&image_queue,0,sizeof(image_queue));
    memset(&camera_queue,0,sizeof(camera_queue));memset(pending,0,sizeof(pending));
    radio_active.size=camera_active.size=0;rx_head=rx_tail=0;seekfree_count=0;
    wireless_settings_defaults(&settings);
    /* Adapter validates both slots before returning any persistent settings. */
    flash_selected_slot=1u;
    (void)wireless_flash_load(&settings,&gen,&flash_selected_slot);
    flash_generation=gen;wireless_flash_writer_init(&flash_writer);
    wireless_control_init(&engine,&settings,gen);car_wireless_parser_init(&radio_parser);
    seed=boot_seed;camera_epoch=boot_seed^0x4b323330u^now;if(!camera_epoch)camera_epoch=1;
    camera_tx_seq=0;serial=0;session_counter=0;camera_ready=0;camera_have_seq=0;
    scope_page=0;effect=CAR_WIRELESS_EFFECT_NONE;effect_ack_valid=0;local_stop_latched=0;
    synced_vision_p=synced_vision_d=0;last_vision_generation=UINT32_MAX;
    image_receiving=capture_pending=0;image_wire_size=image_wire_pos=0;
    last_state_ms=last_diag_ms=now;camera_last_hello_ms=now-200u;initialized=1;
}
void car_wireless_uart_rx_isr(uint8_t b,uint32_t now) {
    uint16_t next=(rx_head+1u)&(RX_N-1u);
    if(next==rx_tail){++car_wireless_rx_overflow;return;}
    rx_data[rx_head]=b;rx_time[rx_head]=now;
#if defined(__GNUC__)
    __asm volatile("" ::: "memory");
#endif
    rx_head=next;
}
void car_wireless_uart_error_isr(void){++car_wireless_uart_errors;}
void car_wireless_local_stop(uint32_t now){(void)now;local_stop_latched=1;}
const wireless_settings_t *car_wireless_settings(void){return &engine.settings;}
wireless_control_test_t car_wireless_test(void){return engine.test;}
uint8_t car_wireless_capture_busy(void){
    for(unsigned i=0;i<4;i++)if(pending[i].active && pending[i].opcode==WIRELESS_OP_CAMERA_CAL)return 1;
    return capture_pending || (image_receiving && image_format==2) ||
        (image_wire_size!=0 && image_format==2) || wireless_flash_busy(&flash_writer);
}
uint8_t car_wireless_test_allowed(uint32_t now){return initialized && !local_stop_latched &&
    now-last_snapshot.now_ms<20u && engine.test.active && engine.runtime.parked &&
    engine.runtime.local_button_released && engine.runtime.startup_release_confirmed &&
    !engine.runtime.hard_fault && !engine.runtime.follow_active && !engine.runtime.calibration_busy &&
    !car_wireless_capture_busy() && engine.runtime.motor_hardware_accepted &&
    now-engine.test.started_ms<engine.test.deadline_ms-engine.test.started_ms;}
car_wireless_effect_t car_wireless_take_effect(void){car_wireless_effect_t result=effect;effect=CAR_WIRELESS_EFFECT_NONE;return result;}
void car_wireless_effect_result(car_wireless_effect_t e,uint8_t success){
    (void)e;if(!effect_ack_valid)return;effect_ack_valid=0;
    effect_ack.status=success?WIRELESS_ACK_OK:WIRELESS_ACK_NOT_READY;
    cache_final(&effect_ack,last_snapshot.now_ms);
}
uint8_t car_wireless_radio_tx_byte(uint8_t *b){
    if(!b)return 0;
    if(image_wire_size && image_wire_pos) {
        *b=image_data[image_wire_pos++];if(image_wire_pos>=image_wire_size)image_wire_size=image_wire_pos=0;return 1;
    }
    if(radio_active.pos>=radio_active.size) {
        radio_active.size=radio_active.pos=0;
        if(!pop(&ack_queue,HI_N,&radio_active) && !pop(&radio_queue,TX_N,&radio_active) &&
           !pop(&image_queue,HI_N,&radio_active)) {
            if(image_wire_size){*b=image_data[image_wire_pos++];return 1;}return 0;
        }
    }
    *b=radio_active.data[radio_active.pos++];return 1;
}
uint8_t car_wireless_camera_tx_byte(uint8_t *b,uint8_t start_allowed){
    if(!b)return 0;
    if(camera_active.pos>=camera_active.size){camera_active.size=camera_active.pos=0;
        if(!start_allowed || !pop(&camera_queue,CAM_N,&camera_active))return 0;}
    *b=camera_active.data[camera_active.pos++];return 1;
}
static void telemetry(const car_wireless_snapshot_t *s,uint8_t type) {
    car_wireless_frame_t f=frame(type,s->now_ms);uint32_t v[32];unsigned i;
    float scope[16];uint8_t packed[80];size_t written;
    memset(v,0,sizeof(v));v[0]=s->now_ms;v[1]=vehicle->mode;v[2]=vehicle->fault;
    v[3]=vehicle->fault_latched;v[4]=vehicle->follow_session_active;v[5]=vehicle->road_drive_allowed;
    v[6]=vehicle->vision_valid;v[7]=vehicle->vision_sequence;v[8]=(uint32_t)(int32_t)vehicle->vision_error_norm;
    v[9]=vehicle->vision_confidence;v[10]=(uint32_t)s->wheel_left_millirps;v[11]=(uint32_t)s->wheel_right_millirps;
    v[12]=(uint32_t)s->target_left_millirps;v[13]=(uint32_t)s->target_right_millirps;v[14]=s->servo_us;
    v[15]=s->motor_left_permille;v[16]=s->motor_right_permille;v[17]=vehicle->imu_ready;
    v[18]=vehicle->imu_fault;v[19]=(uint32_t)vehicle->imu_z_mdps;v[20]=(uint32_t)vehicle->imu_yaw_mdeg;
    v[21]=vehicle->gyro.phase;v[22]=(uint32_t)vehicle->gyro.progress_mdeg;v[23]=vehicle->gyro.fault;
    v[24]=camera_ready?s->now_ms-camera_last_ms:UINT32_MAX;v[25]=camera_count;v[26]=engine.settings_generation;
    v[27]=car_wireless_rx_overflow;v[28]=car_wireless_tx_dropped;v[29]=car_wireless_camera_dropped;
    v[30]=scope_page;v[31]=s->now_ms;
    if(engine.tool_mode==WIRELESS_TOOL_MODE_ASSISTANT){
        if(type!=CAR_WIRELESS_STATE)return;
        for(i=0;i<16;++i)scope[i]=(float)(int32_t)v[(scope_page*8u+i)%32u];
        if(car_seekfree_oscilloscope_frame(packed,sizeof(packed),scope,16,&written) &&
           !push(&radio_queue,TX_N,packed,written))++car_wireless_tx_dropped;
        return;
    }
    f.payload_len=128;for(i=0;i<32;++i)put32(f.payload+4*i,v[i]);
    if(!send_frame(&radio_queue,TX_N,&f))++car_wireless_tx_dropped;
}
void car_wireless_camera_frame(const car_wireless_frame_t *f,uint32_t now) {
    car_wireless_frame_t out;uint32_t id,offset,total;uint16_t w,h;unsigned i;size_t n;
    if(!initialized || !f || f->session!=camera_epoch)return;
    if(camera_have_seq && !car_wireless_sequence_forward(f->sequence,camera_rx_seq))return;
    camera_have_seq=1;camera_rx_seq=f->sequence;camera_last_ms=now;++camera_count;
    if(f->type==CAR_WIRELESS_HELLO_REPLY){camera_ready=1;return;}
    if(f->type==CAR_WIRELESS_ACK && f->payload_len>=16){
        uint32_t seq=u32(f->payload+4);
        for(i=0;i<4;++i)if(pending[i].active && pending[i].camera_sequence==seq){
            wireless_control_ack_t ack={f->payload[0],pending[i].opcode,1,f->payload[3],pending[i].pc_sequence,u32(f->payload+8),u32(f->payload+12)};
            if(ack.status==WIRELESS_ACK_OK && pending[i].pc_sequence==0 && pending[i].opcode==WIRELESS_OP_SET_PARAMS){
                synced_vision_p=pending[i].vision_p;synced_vision_d=pending[i].vision_d;
            }
            if(ack.status!=WIRELESS_ACK_PENDING){pending[i].active=0;if(ack.status!=WIRELESS_ACK_OK)capture_pending=0;}
            cache_final(&ack,now);return;
        }
        return;
    }
    if(f->type==CAR_WIRELESS_STATE || f->type==CAR_WIRELESS_DIAG || f->type==CAR_WIRELESS_EVENT || f->type==CAR_WIRELESS_CONFIG){
        if(engine.tool_mode==WIRELESS_TOOL_MODE_COMPLETE){out=*f;out.session=engine.session;out.sequence=++serial;out.timestamp_ms=now;
            if(!send_frame(&radio_queue,TX_N,&out))++car_wireless_camera_dropped;}return;
    }
    if(f->type!=CAR_WIRELESS_IMAGE || f->payload_len<IMAGE_HEADER)return;
    id=u32(f->payload);offset=u32(f->payload+4);total=u32(f->payload+8);
    w=u16(f->payload+12);h=u16(f->payload+14);n=f->payload_len-IMAGE_HEADER;
    if(!total || total>76800 || !n || offset>total || n>total-offset ||
       !((f->payload[16]==1 && w==80 && h==60 && total==600) ||
         (f->payload[16]==2 && w==320 && h==240 && total==76800))) {++car_wireless_camera_dropped;return;}
    if(offset==0){
        if(image_wire_size || (image_receiving && image_format==2 && now-image_started_ms<IMAGE_TTL_MS)) {++car_wireless_camera_dropped;return;}
        if(f->payload[16]==2 && (!capture_pending || !engine.runtime.parked))return;
        image_id=id;image_total=total;image_offset=0;image_started_ms=now;image_width=w;image_height=h;
        image_format=f->payload[16];image_receiving=1;
    }
    if(!image_receiving || id!=image_id || offset!=image_offset || total!=image_total ||
       w!=image_width || h!=image_height || f->payload[16]!=image_format || now-image_started_ms>IMAGE_TTL_MS){
        ++car_wireless_camera_dropped;return;}
    memcpy(image_data+12+offset,f->payload+IMAGE_HEADER,n);image_offset+=(uint32_t)n;
    if(engine.tool_mode==WIRELESS_TOOL_MODE_COMPLETE){out=*f;out.sequence=++serial;out.session=engine.session;out.timestamp_ms=now;
        if(!send_frame(&image_queue,HI_N,&out))++car_wireless_camera_dropped;}
    if(image_offset==image_total){image_receiving=capture_pending=0;
        if(engine.tool_mode==WIRELESS_TOOL_MODE_ASSISTANT){
            image_data[0]=0xaa;image_data[1]=0;image_data[2]=2;image_data[3]=image_format;
            image_data[4]=(uint8_t)w;image_data[5]=(uint8_t)(w>>8);image_data[6]=(uint8_t)h;image_data[7]=(uint8_t)(h>>8);
            image_data[8]=1;image_data[9]=image_data[10]=image_data[11]=0;
            image_data[1]=car_seekfree_sum8(image_data,12);image_wire_size=12+image_total;image_wire_pos=0;
        }
    }
}
void car_wireless_service(const car_wireless_snapshot_t *s,car_control_t *c,car_config_t *config) {
    unsigned budget=128,i;car_wireless_frame_t f;uint32_t now;wireless_control_runtime_t state;
    if(!initialized || !s || !c || !config)return;
    now=s->now_ms;last_snapshot=*s;vehicle=c;
    state=(wireless_control_runtime_t){0};state.parked=!c->follow_session_active && !c->manual_run_request &&
        (!s->outputs_active || engine.test.active);
    state.local_button_released=s->button_released;state.startup_release_confirmed=s->startup_released;
    state.hard_fault=c->fault_latched || c->fault!=CAR_FAULT_NONE || c->isr_stop_request || c->isr_stop_fault || s->servo_fault || c->imu_fault;
    state.follow_active=c->follow_session_active;state.motor_hardware_accepted=config->motor_hardware_accepted;
    state.calibration_busy=s->calibration_busy || wireless_flash_busy(&flash_writer);state.capture_busy=car_wireless_capture_busy();
    wireless_control_set_runtime(&engine,&state);
    if(local_stop_latched){wireless_control_revoke_outputs(&engine,now);local_stop_latched=0;}
    wireless_control_tick(&engine,now);
    if(camera_ready && now-camera_last_ms>1500u){
        /* A camera reboot must negotiate a new epoch, never replay queued commands. */
        camera_ready=camera_have_seq=0;
        camera_epoch+=0x9e3779b9u;if(!camera_epoch)camera_epoch=1;
        memset(&camera_queue,0,sizeof(camera_queue));
        image_receiving=capture_pending=0;
        synced_vision_p=synced_vision_d=0;
        for(i=0;i<4;i++)if(pending[i].active){
            wireless_control_ack_t ack={WIRELESS_ACK_NOT_READY,pending[i].opcode,1,0,pending[i].pc_sequence,0,0};
            pending[i].active=0;cache_final(&ack,now);
        }
    }
    if(wireless_flash_busy(&flash_writer)) {
        wireless_flash_state_t result=wireless_flash_service(&flash_writer);
        if(result==WIRELESS_FLASH_DONE){flash_generation=flash_writer.generation;flash_selected_slot=flash_writer.target_slot;
            save_ack.status=WIRELESS_ACK_OK;save_ack.detail=flash_generation;cache_final(&save_ack,now);
            (void)wireless_flash_take_done(&flash_writer);}
        else if(result==WIRELESS_FLASH_ERROR){save_ack.status=WIRELESS_ACK_FAULT;save_ack.detail=wireless_flash_error(&flash_writer);cache_final(&save_ack,now);}
    }
    while(rx_tail!=rx_head && budget--){
        uint8_t b=rx_data[rx_tail];uint32_t at=rx_time[rx_tail];rx_tail=(rx_tail+1u)&(RX_N-1u);
        if(now-at>100u){car_wireless_parser_init(&radio_parser);seekfree_count=0;continue;}
        if(at-last_rx_ms>50u){car_wireless_parser_init(&radio_parser);seekfree_count=0;}last_rx_ms=at;
        if(engine.tool_mode==WIRELESS_TOOL_MODE_ASSISTANT &&
           (seekfree_count || (!radio_parser.length && b==0x55))) {
            seekfree_byte(b,now);continue;
        }
        if(car_wireless_parser_feed(&radio_parser,b,&f)!=CAR_WIRELESS_PARSE_FRAME)continue;
        ++car_wireless_packets;
        if(f.type==CAR_WIRELESS_HELLO && f.payload_len==4 && !car_wireless_capture_busy() && effect==CAR_WIRELESS_EFFECT_NONE){
            uint32_t token=seed^(++session_counter*0x9e3779b9u)^now^u32(f.payload);if(!token)token=1;
            if(wireless_control_open_session(&engine,token,now)){
                memset(pending,0,sizeof(pending));
                memset(&ack_queue,0,sizeof(ack_queue));
                memset(&radio_queue,0,sizeof(radio_queue));
                memset(&image_queue,0,sizeof(image_queue));
                car_wireless_frame_t reply=frame(CAR_WIRELESS_HELLO_REPLY,now);car_wireless_session=token;
                reply.payload_len=12;memcpy(reply.payload,f.payload,4);put32(reply.payload+4,engine.settings_generation);put32(reply.payload+8,0x3fu);
                if(!send_frame(&ack_queue,HI_N,&reply))++car_wireless_tx_dropped;
            }
        }else if(f.type==CAR_WIRELESS_COMMAND){
            command_frame(&f,now);
            /* Refresh parked/busy/output state before the next command. */
            break;
        }
        /* Apply a requested start/stop/calibration before admitting another operation. */
        if(effect!=CAR_WIRELESS_EFFECT_NONE)break;
    }
    if(wireless_control_take_stop(&engine) && effect!=CAR_WIRELESS_EFFECT_STOP)effect=CAR_WIRELESS_EFFECT_STOP;
    (void)wireless_control_take_start(&engine);(void)wireless_control_take_imu_cal(&engine);
    if(now-camera_last_hello_ms>=200u && camera_queue.count<CAM_N){
        f=frame(CAR_WIRELESS_HELLO,now);f.session=camera_epoch;f.sequence=++camera_tx_seq;f.payload_len=8;
        put32(f.payload,camera_epoch);f.payload[4]=state.parked && !engine.test.active && !s->outputs_active;
        f.payload[5]=f.payload[6]=f.payload[7]=0;
        if(send_frame(&camera_queue,CAM_N,&f))camera_last_hello_ms=now;
    }
    if(now-last_state_ms>=50u){last_state_ms=now;telemetry(s,CAR_WIRELESS_STATE);}
    if(now-last_diag_ms>=500u){last_diag_ms=now;telemetry(s,CAR_WIRELESS_DIAG);}
    if(image_receiving && now-image_started_ms>IMAGE_TTL_MS){image_receiving=capture_pending=0;++car_wireless_camera_dropped;}
    if(capture_pending && now-capture_started_ms>IMAGE_TTL_MS){capture_pending=image_receiving=0;++car_wireless_camera_dropped;}
    for(i=0;i<4;++i)if(pending[i].active && (int32_t)(now-pending[i].deadline)>=0){
        wireless_control_ack_t ack={WIRELESS_ACK_NOT_READY,pending[i].opcode,1,0,pending[i].pc_sequence,0,engine.settings_generation};
        pending[i].active=0;cache_final(&ack,now);if(ack.opcode==WIRELESS_OP_CAPTURE)capture_pending=0;
    }
    /* MCU settings hold desired vision gains; the camera reports its own ACK.
       No camera success is fabricated when the bridge is unavailable. */
    if(camera_ready && (synced_vision_p!=engine.settings.vision_gain_permille || synced_vision_d!=engine.settings.vision_d_gain_permille) &&
       now-vision_sync_attempt_ms>=2000u && camera_queue.count<CAM_N) {
        f=frame(CAR_WIRELESS_COMMAND,now);f.payload_len=16;
        f.payload[0]=WIRELESS_OP_SET_PARAMS;f.payload[1]=1;f.payload[2]=f.payload[3]=0;
        f.payload[4]=WIRELESS_SETTINGS_GROUP_DRIVING;f.payload[5]=2;
        f.payload[6]=WIRELESS_PARAM_VISION_GAIN_PERMILLE;put32(f.payload+7,engine.settings.vision_gain_permille);
        f.payload[11]=WIRELESS_PARAM_VISION_D_GAIN_PERMILLE;put32(f.payload+12,engine.settings.vision_d_gain_permille);
        if(forward_camera(&f,0,WIRELESS_OP_SET_PARAMS,now)) {
            vision_sync_attempt_ms=now;
            last_vision_generation=engine.settings_generation;
        }
    }
    config->follow_servo_left_us=engine.settings.servo_left_us;config->follow_servo_right_us=engine.settings.servo_right_us;
    config->follow_left_encoder_counts_per_rev=engine.settings.encoder_left_cpr;config->follow_right_encoder_counts_per_rev=engine.settings.encoder_right_cpr;
    config->follow_left_encoder_sign=engine.settings.encoder_left_sign;config->follow_right_encoder_sign=engine.settings.encoder_right_sign;
}
#endif
