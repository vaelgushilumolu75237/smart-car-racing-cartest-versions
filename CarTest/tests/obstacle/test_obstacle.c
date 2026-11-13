#include "obstacle_bypass.h"
#include "obstacle_bypass_config.h"
#include <assert.h>
#include <stdio.h>
#include <limits.h>
static car_obstacle_t s;
static car_obstacle_input_t in;
static car_obstacle_config_t c;
static void step(void) { in.imu_sample_us=in.now_ms*1000u; car_obstacle_step(&s,&c,&in); }
static void start(void) {
 car_obstacle_init(&s); c=car_obstacle_route_config;
 assert(c.segment[1].distance_millirev==250 && c.segment[3].distance_millirev==1000 && c.segment[5].distance_millirev==250);
 in=(car_obstacle_input_t){.now_ms=100,.run_active=1,.trigger=1,.imu_ready=1,.odometry_valid=1};step();in.trigger=0;
 assert(s.state==1 && s.control_error_norm<0);
 assert(s.outer_target_millirps==CARTEST_EFFECTIVE_SLOWEST_MILLIRPS);
}
static void complete(void) {
 for (int stage=1;stage<=7;stage++) {
  if(stage==4)continue;
  assert(s.state==stage);
  in.now_ms+=10;
  if (stage%2) in.yaw_mdeg=s.heading_target_mdeg;
  else in.forward_millirev+=c.segment[stage-1].distance_millirev;
  step();
  assert(s.state==(stage==3?5:stage+1));
  if(stage==3)assert(s.heading_target_mdeg==-30000 && s.control_error_norm>0);
 }
 assert(s.state==8 && s.outer_target_millirps==CARTEST_EFFECTIVE_SLOWEST_MILLIRPS && s.heading_target_mdeg==0);
}
int main(void) {
 start(); in.trigger=1; in.now_ms++;step();in.trigger=0;
 assert(s.ignored_triggers==1 && s.accepted_triggers==1 && s.stage_started_ms==100);
 in.yaw_mdeg=-30000;in.forward_millirev=99999;step();assert(s.state==1);
 in.forward_millirev=0;in.yaw_mdeg=0;in.camera_sequence=65534;complete();
 assert(c.segment[0].angle_mdeg==30000 && c.segment[2].angle_mdeg==30000 && c.segment[4].angle_mdeg==30000 && c.segment[6].angle_mdeg==30000);
 in.ordinary_fresh=1;in.camera_sequence=65535;step();step();assert(s.recovery_count==1);
 in.camera_sequence=0;step();assert(s.recovery_count==2);
 in.ordinary_fresh=0;step();assert(!s.recovery_count);
 in.ordinary_fresh=1;in.camera_sequence=1;step();in.camera_sequence=3;step();assert(s.recovery_count==1);
 in.camera_sequence=4;step();in.now_ms+=250;in.camera_sequence=5;step();assert(s.recovery_count==1);
 in.camera_sequence=6;step();in.camera_sequence=7;step();assert(!s.active&&s.resumed);
 step();assert(!s.resumed);
 start();complete();in.now_ms+=3000;step();assert(s.error==CAR_OBSTACLE_ERROR_TIMEOUT && !s.outer_target_millirps);
 start();in.run_active=0;step();assert(!s.active&&!s.outer_target_millirps);in.run_active=1;step();assert(!s.active);
 start();in.imu_ready=0;step();assert(s.error==CAR_OBSTACLE_ERROR_IMU&&!s.outer_target_millirps);
 start();in.odometry_valid=0;step();assert(s.error==CAR_OBSTACLE_ERROR_ENCODER);
 start();in.now_ms+=8000;step();assert(s.error==CAR_OBSTACLE_ERROR_TIMEOUT);
 start();in.now_ms=UINT32_MAX-10;s.stage_started_ms=in.now_ms;step();in.now_ms=20;step();assert(!s.error);
 start();in.yaw_mdeg=INT64_MIN;step();assert(s.error==CAR_OBSTACLE_ERROR_INPUT);
 start();in.yaw_mdeg=31000;step();assert(s.state==2 && s.control_error_norm>0); /* hold planned 30, not overshoot */
 in.now_ms++;in.forward_millirev=249;step();assert(s.state==2);
 in.forward_millirev=250;step();assert(s.state==3&&s.heading_target_mdeg==0&&s.control_error_norm>0);
 in.forward_millirev=249;step();assert(s.error==CAR_OBSTACLE_ERROR_ENCODER);
 c=car_obstacle_route_config;c.segment[0].angle_mdeg=0;assert(!car_obstacle_config_valid(&c));
 c=car_obstacle_route_config;c.segment[0].angle_mdeg++;assert(car_obstacle_config_valid(&c));
 c=car_obstacle_route_config;c.segment[1].distance_millirev=0;assert(!car_obstacle_config_valid(&c));
 c=car_obstacle_route_config;c.segment[0].outer_speed_millirps=0;assert(!car_obstacle_config_valid(&c));
 start();in.trigger=1;c.segment[0].angle_mdeg=0;s.active=0;step();assert(s.error==CAR_OBSTACLE_ERROR_UNCALIBRATED);
 start();in.now_ms+=50;car_obstacle_step(&s,&c,&in);assert(s.error==CAR_OBSTACLE_ERROR_IMU);
 puts("obstacle state/threshold/stop/timeout/recovery tests passed");return 0;
}
