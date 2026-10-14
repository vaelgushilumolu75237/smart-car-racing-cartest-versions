#ifndef CARTEST_CAR_CALIBRATION_H
#define CARTEST_CAR_CALIBRATION_H

/* User-confirmed smooth steering span, with margin at both ends. */
#define CARTEST_SERVO_LEFT_US 1100u
#define CARTEST_SERVO_CENTER_US 1530u
#define CARTEST_SERVO_RIGHT_US 2500u
#define CARTEST_SERVO_LIMITS_MEASURED 1u

/* Preserve the existing P0.7 steering gain across the measured servo span. */
#define CARTEST_STEERING_GAIN_PERMILLE 700u

/* Positive image error maps right. */
#define CARTEST_STEERING_DIRECTION_CONFIRMED 1u

/* User-reported counts from continuous, un-cleared three-turn captures. */
#define CARTEST_ENCODER_LEFT_COUNTS_PER_REV 59995u
#define CARTEST_ENCODER_RIGHT_COUNTS_PER_REV 60239u
#define CARTEST_ENCODER_LEFT_SIGN 1
#define CARTEST_ENCODER_RIGHT_SIGN (-1)

/* Confirmed servo, steering direction, and encoder measurements enable FOLLOW. */
#define CARTEST_FOLLOW_CALIBRATION_COMPLETE 1u

#if CARTEST_SERVO_LIMITS_MEASURED && \
    (CARTEST_SERVO_LEFT_US > 0u) && (CARTEST_SERVO_RIGHT_US > 0u)
#define CARTEST_SERVO_MIN_US \
    ((CARTEST_SERVO_LEFT_US < CARTEST_SERVO_RIGHT_US) ? \
         CARTEST_SERVO_LEFT_US : CARTEST_SERVO_RIGHT_US)
#define CARTEST_SERVO_MAX_US \
    ((CARTEST_SERVO_LEFT_US > CARTEST_SERVO_RIGHT_US) ? \
         CARTEST_SERVO_LEFT_US : CARTEST_SERVO_RIGHT_US)
#else
#define CARTEST_SERVO_MIN_US 0u
#define CARTEST_SERVO_MAX_US 0u
#endif

#if CARTEST_FOLLOW_CALIBRATION_COMPLETE
#if !CARTEST_SERVO_LIMITS_MEASURED || \
    !CARTEST_STEERING_DIRECTION_CONFIRMED || \
    (CARTEST_SERVO_LEFT_US == 0u) || \
    (CARTEST_SERVO_RIGHT_US == 0u) || \
    (CARTEST_ENCODER_LEFT_COUNTS_PER_REV == 0u) || \
    (CARTEST_ENCODER_RIGHT_COUNTS_PER_REV == 0u) || \
    ((CARTEST_ENCODER_LEFT_SIGN != 1) && (CARTEST_ENCODER_LEFT_SIGN != -1)) || \
    ((CARTEST_ENCODER_RIGHT_SIGN != 1) && (CARTEST_ENCODER_RIGHT_SIGN != -1))
#error "Follow calibration is marked complete but required measurements are missing."
#endif
#endif

/* One-shot overhead paired-forward selection; input 0 is its legal marker. */
#define CARTEST_BENCH_MOTOR 3u       /* CAR_MOTOR_BOTH_FORWARD */
#define CARTEST_BENCH_INPUT 0u       /* CAR_MOTOR_IN1 selector marker */
#define CARTEST_BENCH_DUTY_PERMILLE 400u
#define CARTEST_BENCH_HOLD_MS 5000u
#define CARTEST_BENCH_RUN_LIMIT_MS 5500u

#endif /* CARTEST_CAR_CALIBRATION_H */
