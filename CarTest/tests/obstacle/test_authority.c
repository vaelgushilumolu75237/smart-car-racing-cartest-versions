#define main original_authority_main
#include "../gyro350/test_authority.c"
#undef main
#if CARTEST_RED_OBSTACLE_ENABLED
static void obstacle_seed(authority_fixture_t *f) {
 fixture_init(f);f->control.imu_ready=1;f->control.obstacle_odometry_valid=1;
 for(int i=0;i<3;i++)send_frame(f,1,VP_VERSION_V4,0,0,1,f->now_ms+10);
 f->control.obstacle_trigger=1;car_control_update_obstacle(&f->control,&f->config,f->now_ms);
 assert(f->control.obstacle.active && f->control.obstacle.state==1);
 assert(car_control_follow_servo_output_target_us(&f->control)<f->config.servo_center_us);
}
static void obstacle_tick(authority_fixture_t *f, uint32_t now, uint8_t running) {
 car_inputs_t inputs={0};inputs.now_ms=now;inputs.pe7_raw_high=1;inputs.pe7_high_stable=1;
 inputs.manual_run_request=running;inputs.manual_button_startup_release_confirmed=1;
 f->control.imu_sample_us=now*1000u;
 car_control_tick(&f->control,&f->config,&inputs);
}
int main(void) {
 authority_fixture_t f;original_authority_main();obstacle_seed(&f);
 assert(car_control_follow_authority_ready(&f.control,&f.config,f.now_ms));
 send_frame(&f,0,VP_VERSION_V4,0,0,0,f.now_ms+10);
 assert(f.control.obstacle.active&&!f.control.vision_hold_active);
 assert(car_control_follow_authority_ready(&f.control,&f.config,f.now_ms));
 assert(!car_control_follow_authority_ready(&f.control,&f.config,f.now_ms+300));
 car_control_update_gyro(&f.control,&f.config,f.now_ms);assert(!f.control.gyro.active);
 f.control.manual_run_request=0;car_control_update_obstacle(&f.control,&f.config,f.now_ms);assert(!f.control.obstacle.active);
 obstacle_seed(&f);f.control.imu_ready=0;car_control_update_obstacle(&f.control,&f.config,f.now_ms);
 assert(f.control.fault==CAR_FAULT_IMU && f.control.fault_latched);
 obstacle_seed(&f);f.control.obstacle.state=CAR_OBSTACLE_REACQUIRE;f.control.obstacle.outer_target_millirps=0;
 for(int i=0;i<3;i++){send_frame(&f,1,VP_VERSION_V4,0,0,1,f.now_ms+10);car_control_update_obstacle(&f.control,&f.config,f.now_ms);}
 assert(!f.control.obstacle.active && f.control.vision_hold_restart_required);
 obstacle_seed(&f);send_frame(&f,0,VP_VERSION_V4,0,0,0,f.now_ms+10);
 obstacle_tick(&f,f.now_ms,1);
 assert(f.control.road_drive_allowed && f.control.fault==CAR_FAULT_NONE);
 obstacle_tick(&f,f.now_ms+300,1);
 assert(!f.control.road_drive_allowed && f.control.fault!=CAR_FAULT_NONE);
 obstacle_seed(&f);f.control.obstacle.state=CAR_OBSTACLE_REACQUIRE;f.control.obstacle.outer_target_millirps=0;
 obstacle_tick(&f,f.now_ms,1);assert(!f.control.road_drive_allowed && !f.control.follow_output_duty_permille);
 obstacle_seed(&f);obstacle_tick(&f,f.now_ms,0);assert(!f.control.road_drive_allowed);
 /* Keep IMU fresh to isolate the route timeout from IMU health. */
 obstacle_seed(&f);f.control.imu_sample_us=(f.now_ms+8000)*1000u;
 car_control_update_obstacle(&f.control,&f.config,f.now_ms+8000);
 assert(f.control.obstacle.error==CAR_OBSTACLE_ERROR_TIMEOUT);
 assert(f.control.fault==CAR_FAULT_OBSTACLE && f.control.fault_latched && !f.control.road_drive_allowed);
 puts("obstacle integration authority tests passed");return 0;
}
#endif
