#ifndef CARTEST_WIRELESS_RUNTIME_H
#define CARTEST_WIRELESS_RUNTIME_H
#include "gyro350_config.h"
#if CARTEST_WIRELESS_ENABLED
#include "commissioning.h"
#include "wireless_protocol.h"
#include "wireless_control.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t now_ms;
    int32_t wheel_left_millirps, wheel_right_millirps;
    int32_t target_left_millirps, target_right_millirps;
    uint16_t servo_us, motor_left_permille, motor_right_permille;
    uint8_t outputs_active, button_released, startup_released;
    uint8_t servo_fault, calibration_busy;
    uint8_t imu_who_am_i, imu_who_am_i_valid;
    uint32_t imu_who_am_i_reads, imu_sample_count;
    uint32_t camera_uart_rx_bytes, camera_uart_errors, camera_uart_crc_errors;
    uint32_t camera_uart_overflows, camera_uart_tx_bytes;
} car_wireless_snapshot_t;
typedef enum { CAR_WIRELESS_EFFECT_NONE=0, CAR_WIRELESS_EFFECT_START=1,
    CAR_WIRELESS_EFFECT_STOP=2, CAR_WIRELESS_EFFECT_IMU_CAL=3 } car_wireless_effect_t;
void car_wireless_init(uint32_t now_ms, uint32_t boot_seed);
void car_wireless_uart_rx_isr(uint8_t byte, uint32_t now_ms);
void car_wireless_uart_error_isr(void);
void car_wireless_service(const car_wireless_snapshot_t *snapshot,
                          car_control_t *control, car_config_t *config);
car_wireless_effect_t car_wireless_take_effect(void);
void car_wireless_effect_result(car_wireless_effect_t effect, uint8_t success);
void car_wireless_camera_frame(const car_wireless_frame_t *frame, uint32_t now_ms);
uint8_t car_wireless_radio_tx_byte(uint8_t *byte);
uint8_t car_wireless_camera_tx_byte(uint8_t *byte, uint8_t start_allowed);
void car_wireless_local_stop(uint32_t now_ms);
const wireless_settings_t *car_wireless_settings(void);
wireless_control_test_t car_wireless_test(void);
uint8_t car_wireless_test_allowed(uint32_t now_ms);
uint8_t car_wireless_capture_busy(void);
extern volatile uint32_t car_wireless_rx_overflow, car_wireless_uart_errors;
extern volatile uint32_t car_wireless_tx_dropped, car_wireless_camera_dropped;
extern volatile uint32_t car_wireless_session, car_wireless_packets;
#ifdef __cplusplus
}
#endif
#endif
#endif
