#ifndef CARTEST_OBSTACLE_BYPASS_CONFIG_H
#define CARTEST_OBSTACLE_BYPASS_CONFIG_H
#include "obstacle_bypass.h"
#include "car_track_profile.h"
/* USER-AUTHORIZED CANDIDATES, NOT MEASURED ROUTE VALUES.
 * Columns: turn angle mdeg, forward mean wheel-millirev, outer speed
 * milli-RPS, yaw-rate mdps, maximum stage duration ms.
 * Speed reuses this profile's existing ordinary straight/bend targets.
 * Wireless wheel-scale remains applied by the original motor controller.
 * Turning is forward motion, not a pivot about the stationary rear axle. */
static const car_obstacle_config_t car_obstacle_route_config = {
    .segment = {
        {30000,    0, CARTEST_EFFECTIVE_SLOWEST_MILLIRPS, 60000, 8000}, /* 1 left */
        {    0, 1000, CARTEST_EFFECTIVE_FASTEST_MILLIRPS, 30000, 6000}, /* 2 diagonal left */
        {45000,    0, CARTEST_EFFECTIVE_SLOWEST_MILLIRPS, 60000, 8000}, /* 3 right */
        {    0, 1000, CARTEST_EFFECTIVE_FASTEST_MILLIRPS, 30000, 8000}, /* 4 reserved, skipped by state machine */
        {45000,    0, CARTEST_EFFECTIVE_SLOWEST_MILLIRPS, 60000, 8000}, /* 5 right */
        {    0, 1500, CARTEST_EFFECTIVE_FASTEST_MILLIRPS, 30000, 6000}, /* 6 diagonal right */
        {45000,    0, CARTEST_EFFECTIVE_SLOWEST_MILLIRPS, 60000, 8000}  /* 7 left align */
    },
    .heading_kp_permille = 1500 /* candidate heading gain, NOT track-validated */
};
#endif
