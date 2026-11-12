#ifndef CARTEST_WIRELESS_CONTROL_H
#define CARTEST_WIRELESS_CONTROL_H

#include <stddef.h>
#include <stdint.h>

#include "wireless_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WIRELESS_TARGET_MCU = 0u,
    WIRELESS_TARGET_CAMERA = 1u
};

typedef enum {
    WIRELESS_OP_INFO = 0x01u,
    WIRELESS_OP_START = 0x02u,
    WIRELESS_OP_STOP = 0x03u,
    WIRELESS_OP_SET_PARAMS = 0x04u,
    WIRELESS_OP_SAVE = 0x05u,
    WIRELESS_OP_DEFAULT = 0x06u,
    WIRELESS_OP_TEST_MOTOR = 0x07u,
    WIRELESS_OP_TEST_SERVO = 0x08u,
    WIRELESS_OP_IMU_CAL = 0x09u,
    WIRELESS_OP_CAMERA_CAL = 0x0au,
    WIRELESS_OP_CAPTURE = 0x0bu,
    WIRELESS_OP_MODE = 0x0cu,
    WIRELESS_OP_PAGE = 0x0du,
    WIRELESS_OP_GET_PARAMS = 0x0eu,
    WIRELESS_OP_GET = WIRELESS_OP_GET_PARAMS
} wireless_opcode_t;

enum {
    WIRELESS_TOOL_MODE_ASSISTANT = 0u,
    WIRELESS_TOOL_MODE_COMPLETE = 1u
};

typedef enum {
    WIRELESS_ACK_OK = 0u,
    WIRELESS_ACK_PENDING = 1u,
    WIRELESS_ACK_BAD_FRAME = 2u,
    WIRELESS_ACK_BAD_SESSION = 3u,
    WIRELESS_ACK_STALE = 4u,
    WIRELESS_ACK_DUPLICATE = 5u,
    WIRELESS_ACK_OUT_OF_ORDER = 6u,
    WIRELESS_ACK_NOT_PARKED = 7u,
    WIRELESS_ACK_BUSY = 8u,
    WIRELESS_ACK_DENIED = 9u,
    WIRELESS_ACK_RANGE = 10u,
    WIRELESS_ACK_UNSUPPORTED = 11u,
    WIRELESS_ACK_BAD_PARAM = 12u,
    WIRELESS_ACK_FAULT = 13u,
    WIRELESS_ACK_FORWARD = 14u,
    WIRELESS_ACK_NOT_READY = 15u
} wireless_ack_status_t;

enum {
    WIRELESS_ACK_FLAG_DUPLICATE = 1u << 0
};

typedef enum {
    WIRELESS_TEST_NONE = 0u,
    WIRELESS_TEST_MOTOR = 1u,
    WIRELESS_TEST_SERVO = 2u
} wireless_test_kind_t;

typedef struct {
    uint8_t status;
    uint8_t opcode;
    uint8_t target;
    uint8_t flags;
    uint32_t request_seq;
    uint32_t detail;
    uint32_t settings_generation;
} wireless_control_ack_t;

typedef struct {
    uint8_t parked;
    uint8_t local_button_released;
    uint8_t startup_release_confirmed;
    uint8_t hard_fault;
    uint8_t follow_active;
    uint8_t motor_hardware_accepted;
    uint8_t calibration_busy;
    uint8_t capture_busy;
    /* Recoverable vehicle fault: block new operations, let commissioning
       retain the existing Follow request and perform its own recovery. */
    uint8_t command_inhibited;
} wireless_control_runtime_t;

typedef struct {
    uint8_t kind;
    uint8_t motor; /* 1 left, 2 right, 3 both forward */
    uint16_t duty_permille;
    uint16_t servo_us;
    uint32_t started_ms;
    uint32_t deadline_ms;
    uint8_t active;
} wireless_control_test_t;

typedef struct {
    wireless_settings_t settings;
    wireless_settings_t defaults;
    wireless_control_runtime_t runtime;
    wireless_control_test_t test;
    uint32_t session;
    uint32_t last_command_seq;
    uint8_t have_command_seq;
    uint8_t session_ready;
    uint8_t start_requested;
    uint8_t imu_cal_requested;
    uint8_t camera_cal_requested;
    uint8_t capture_requested;
    uint8_t stop_requested;
    uint8_t tool_mode;
    uint8_t parameter_page;
    uint8_t save_requested;
    uint8_t default_requested;
    uint32_t settings_generation;
    uint32_t last_ack_seq;
    wireless_control_ack_t cached_ack;
} wireless_control_t;

void wireless_control_init(wireless_control_t *control,
                           const wireless_settings_t *initial,
                           uint32_t generation);
/* Replace the profile defaults used by the parking-only DEFAULT command. */
void wireless_control_set_defaults(wireless_control_t *control,
                                   const wireless_settings_t *defaults);
void wireless_control_set_runtime(wireless_control_t *control,
                                  const wireless_control_runtime_t *runtime);
/* A new session invalidates old commands. It is rejected while a timed test
   is active so a reconnect cannot extend an actuator operation. */
uint8_t wireless_control_open_session(wireless_control_t *control,
                                      uint32_t session,
                                      uint32_t now_ms);
void wireless_control_revoke_outputs(wireless_control_t *control,
                                     uint32_t now_ms);
void wireless_control_tick(wireless_control_t *control, uint32_t now_ms);

/* payload is the COMMAND payload after the outer wireless frame. The first
   four bytes are opcode, target and little-endian reserved=0. */
wireless_ack_status_t wireless_control_command(
    wireless_control_t *control,
    uint32_t sequence,
    uint32_t session,
    uint32_t command_timestamp_ms,
    uint32_t now_ms,
    const uint8_t *payload,
    size_t payload_length,
    wireless_control_ack_t *ack);

const wireless_settings_t *wireless_control_settings(
    const wireless_control_t *control);
wireless_control_test_t wireless_control_test(const wireless_control_t *control);
uint8_t wireless_control_take_start(wireless_control_t *control);
uint8_t wireless_control_take_stop(wireless_control_t *control);
uint8_t wireless_control_take_imu_cal(wireless_control_t *control);
uint8_t wireless_control_take_camera_cal(wireless_control_t *control);
uint8_t wireless_control_take_capture(wireless_control_t *control);
uint8_t wireless_control_take_save(wireless_control_t *control);
uint8_t wireless_control_take_default(wireless_control_t *control);
uint8_t wireless_control_set_tool_mode(wireless_control_t *control,
                                       uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_WIRELESS_CONTROL_H */
