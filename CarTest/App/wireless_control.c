#include "wireless_control.h"

#include <string.h>

enum {
    COMMAND_HEADER_BYTES = 4u,
    SET_GROUP_HEADER_BYTES = 2u,
    SET_ENTRY_BYTES = 5u,
    MAX_SET_ENTRIES = 16u,
    COMMAND_MAX_AGE_MS = 1000u,
    MOTOR_MAX_DUTY_PERMILLE = 400u,
    MOTOR_MAX_DURATION_MS = 5000u,
    SERVO_MIN_US = 1100u,
    SERVO_MAX_US = 2500u,
    SERVO_CENTER_US = 1530u,
    SERVO_MAX_DURATION_MS = 1000u
};

static uint16_t get_u16(const uint8_t *in)
{
    return (uint16_t)in[0] | (uint16_t)((uint16_t)in[1] << 8u);
}

static uint32_t get_u32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8u) |
           ((uint32_t)in[2] << 16u) | ((uint32_t)in[3] << 24u);
}

static int32_t get_i32(const uint8_t *in)
{
    return (int32_t)get_u32(in);
}

static uint8_t seq_newer(uint32_t candidate, uint32_t previous)
{
    uint32_t delta = candidate - previous;
    return delta != 0u && delta < 0x80000000u;
}

static uint8_t elapsed_ok(uint32_t now_ms, uint32_t timestamp_ms)
{
    int32_t elapsed = (int32_t)(now_ms - timestamp_ms);
    return elapsed >= 0 && (uint32_t)elapsed <= COMMAND_MAX_AGE_MS;
}

static void ack_init(wireless_control_ack_t *ack, uint32_t sequence,
                     uint8_t opcode, uint8_t target)
{
    *ack = (wireless_control_ack_t){
        .status = WIRELESS_ACK_BAD_FRAME,
        .opcode = opcode,
        .target = target,
        .flags = 0u,
        .request_seq = sequence,
        .detail = 0u,
        .settings_generation = 0u
    };
}

static void remember_ack(wireless_control_t *control,
                         const wireless_control_ack_t *ack)
{
    control->cached_ack = *ack;
    control->last_ack_seq = ack->request_seq;
}

static wireless_ack_status_t finish(wireless_control_t *control,
                                    wireless_control_ack_t *ack,
                                    wireless_ack_status_t status,
                                    uint32_t detail)
{
    ack->status = (uint8_t)status;
    ack->detail = detail;
    ack->settings_generation = control->settings_generation;
    remember_ack(control, ack);
    return status;
}

static wireless_ack_status_t finish_uncached(
    wireless_control_t *control, wireless_control_ack_t *ack,
    wireless_ack_status_t status, uint32_t detail)
{
    ack->status = (uint8_t)status;
    ack->detail = detail;
    ack->settings_generation = control->settings_generation;
    return status;
}

static uint8_t is_parked_and_safe(const wireless_control_t *control)
{
    const wireless_control_runtime_t *runtime = &control->runtime;
    return (runtime->parked != 0u && runtime->local_button_released != 0u &&
            runtime->startup_release_confirmed != 0u && runtime->hard_fault == 0u &&
            runtime->command_inhibited == 0u &&
            runtime->follow_active == 0u && runtime->calibration_busy == 0u &&
            runtime->capture_busy == 0u && control->test.active == 0u) ? 1u : 0u;
}

static uint8_t command_allowed_while_test_active(uint8_t opcode)
{
    return opcode == WIRELESS_OP_INFO || opcode == WIRELESS_OP_STOP ||
           opcode == WIRELESS_OP_GET_PARAMS || opcode == WIRELESS_OP_PAGE ||
           opcode == WIRELESS_OP_MODE;
}

static wireless_ack_status_t validate_camera_set_params(
    const uint8_t *data, size_t length)
{
    uint8_t group;
    uint8_t count;
    uint8_t index;

    if (data == NULL || length < SET_GROUP_HEADER_BYTES) return WIRELESS_ACK_BAD_PARAM;
    group = data[0];
    count = data[1];
    /* The K230 side owns only the two vision gains.  Keeping this as a small
       whitelist prevents an MCU parameter group from being acknowledged as a
       camera setting by accident. */
    if (group != WIRELESS_SETTINGS_GROUP_DRIVING || count == 0u || count > 2u ||
        length != SET_GROUP_HEADER_BYTES + (size_t)count * SET_ENTRY_BYTES)
        return WIRELESS_ACK_UNSUPPORTED;
    for (index = 0u; index < count; ++index) {
        uint8_t id = data[2u + (size_t)index * SET_ENTRY_BYTES];
        int32_t value = get_i32(data + 3u + (size_t)index * SET_ENTRY_BYTES);
        uint8_t previous;
        if (id != WIRELESS_PARAM_VISION_GAIN_PERMILLE &&
            id != WIRELESS_PARAM_VISION_D_GAIN_PERMILLE)
            return WIRELESS_ACK_UNSUPPORTED;
        if (value < 500 || value > 1500) return WIRELESS_ACK_RANGE;
        for (previous = 0u; previous < index; ++previous) {
            if (data[2u + (size_t)previous * SET_ENTRY_BYTES] == id)
                return WIRELESS_ACK_BAD_PARAM;
        }
    }
    return WIRELESS_ACK_FORWARD;
}

static wireless_ack_status_t camera_target_command(
    const wireless_control_t *control, uint8_t opcode,
    const uint8_t *payload, size_t payload_length)
{
    const size_t data_length = payload_length - COMMAND_HEADER_BYTES;
    const uint8_t *data = payload + COMMAND_HEADER_BYTES;

    switch (opcode) {
    case WIRELESS_OP_INFO:
    case WIRELESS_OP_STOP:
    case WIRELESS_OP_GET_PARAMS:
        return WIRELESS_ACK_FORWARD;
    case WIRELESS_OP_SET_PARAMS:
        return validate_camera_set_params(data, data_length);
    case WIRELESS_OP_SAVE:
    case WIRELESS_OP_DEFAULT:
    case WIRELESS_OP_CAMERA_CAL:
    case WIRELESS_OP_CAPTURE:
        return is_parked_and_safe(control) ? WIRELESS_ACK_FORWARD
                                            : WIRELESS_ACK_NOT_PARKED;
    case WIRELESS_OP_START:
    case WIRELESS_OP_TEST_MOTOR:
    case WIRELESS_OP_TEST_SERVO:
    case WIRELESS_OP_IMU_CAL:
        return WIRELESS_ACK_DENIED;
    default:
        return WIRELESS_ACK_UNSUPPORTED;
    }
}

void wireless_control_init(wireless_control_t *control,
                           const wireless_settings_t *initial,
                           uint32_t generation)
{
    if (control == NULL) return;
    (void)memset(control, 0, sizeof(*control));
    wireless_settings_defaults(&control->defaults);
    control->settings = control->defaults;
    if (initial != NULL && wireless_settings_is_valid(initial)) control->settings = *initial;
    control->settings_generation = generation;
    control->runtime.parked = 1u;
    control->runtime.local_button_released = 1u;
    control->runtime.startup_release_confirmed = 1u;
    control->tool_mode = WIRELESS_TOOL_MODE_ASSISTANT;
}

void wireless_control_set_runtime(wireless_control_t *control,
                                  const wireless_control_runtime_t *runtime)
{
    if (control == NULL || runtime == NULL) return;
    control->runtime = *runtime;
    if (runtime->hard_fault != 0u || runtime->local_button_released == 0u)
        wireless_control_revoke_outputs(control, 0u);
    else if (runtime->command_inhibited != 0u) {
        control->test = (wireless_control_test_t){.servo_us = SERVO_CENTER_US};
        control->start_requested = 0u;
    }
}

uint8_t wireless_control_open_session(wireless_control_t *control,
                                      uint32_t session, uint32_t now_ms)
{
    (void)now_ms;
    if (control == NULL || session == 0u || control->test.active != 0u) return 0u;
    control->session = session;
    control->session_ready = 1u;
    control->have_command_seq = 0u;
    control->last_command_seq = 0u;
    control->last_ack_seq = 0u;
    (void)memset(&control->cached_ack, 0, sizeof(control->cached_ack));
    return 1u;
}

void wireless_control_revoke_outputs(wireless_control_t *control, uint32_t now_ms)
{
    (void)now_ms;
    if (control == NULL) return;
    control->test.active = 0u;
    control->test.kind = WIRELESS_TEST_NONE;
    control->test.duty_permille = 0u;
    control->test.servo_us = SERVO_CENTER_US;
    control->start_requested = 0u;
    control->stop_requested = 1u;
}

void wireless_control_tick(wireless_control_t *control, uint32_t now_ms)
{
    if (control == NULL) return;
    if (control->runtime.hard_fault != 0u ||
        control->runtime.local_button_released == 0u ||
        control->runtime.startup_release_confirmed == 0u) {
        wireless_control_revoke_outputs(control, now_ms);
        return;
    }
    if (control->runtime.command_inhibited != 0u) {
        control->test = (wireless_control_test_t){.servo_us = SERVO_CENTER_US};
        control->start_requested = 0u;
        return;
    }
    if (control->test.active != 0u &&
        (uint32_t)(now_ms - control->test.started_ms) >=
            (uint32_t)(control->test.deadline_ms - control->test.started_ms)) {
        control->test.active = 0u;
        control->test.kind = WIRELESS_TEST_NONE;
        control->test.duty_permille = 0u;
        control->test.servo_us = SERVO_CENTER_US;
    }
}

static wireless_ack_status_t command_motor_test(
    wireless_control_t *control, const uint8_t *data, size_t length,
    uint32_t now_ms, wireless_control_ack_t *ack)
{
    uint16_t duty, duration;
    if (length != 5u) return WIRELESS_ACK_BAD_PARAM;
    duty = get_u16(data + 1u);
    duration = get_u16(data + 3u);
    if (data[0] < 1u || data[0] > 3u || duty > MOTOR_MAX_DUTY_PERMILLE ||
        duration == 0u || duration > MOTOR_MAX_DURATION_MS) return WIRELESS_ACK_RANGE;
    if (!is_parked_and_safe(control) || !control->runtime.motor_hardware_accepted)
        return control->runtime.hard_fault ? WIRELESS_ACK_FAULT : WIRELESS_ACK_NOT_PARKED;
    if (control->test.active != 0u) return WIRELESS_ACK_BUSY;
    control->test = (wireless_control_test_t){
        .kind = WIRELESS_TEST_MOTOR, .motor = data[0], .duty_permille = duty,
        .servo_us = SERVO_CENTER_US, .started_ms = now_ms,
        .deadline_ms = now_ms + duration, .active = 1u
    };
    ack->detail = ((uint32_t)data[0] << 24u) | ((uint32_t)duty << 8u) | duration;
    return WIRELESS_ACK_OK;
}

static wireless_ack_status_t command_servo_test(
    wireless_control_t *control, const uint8_t *data, size_t length,
    uint32_t now_ms, wireless_control_ack_t *ack)
{
    uint16_t pulse, duration;
    if (length != 4u) return WIRELESS_ACK_BAD_PARAM;
    pulse = get_u16(data);
    duration = get_u16(data + 2u);
    if (pulse < SERVO_MIN_US || pulse > SERVO_MAX_US || duration == 0u ||
        duration > SERVO_MAX_DURATION_MS) return WIRELESS_ACK_RANGE;
    if (!is_parked_and_safe(control) || !control->runtime.motor_hardware_accepted)
        return control->runtime.hard_fault ? WIRELESS_ACK_FAULT : WIRELESS_ACK_NOT_PARKED;
    if (control->test.active != 0u) return WIRELESS_ACK_BUSY;
    control->test = (wireless_control_test_t){
        .kind = WIRELESS_TEST_SERVO, .servo_us = pulse, .started_ms = now_ms,
        .deadline_ms = now_ms + duration, .active = 1u
    };
    ack->detail = ((uint32_t)pulse << 16u) | duration;
    return WIRELESS_ACK_OK;
}

static wireless_ack_status_t command_set_params(
    wireless_control_t *control, const uint8_t *data, size_t length,
    wireless_control_ack_t *ack)
{
    wireless_param_entry_t entries[MAX_SET_ENTRIES];
    uint8_t group, count, i;
    wireless_settings_status_t status;
    if (length < SET_GROUP_HEADER_BYTES) return WIRELESS_ACK_BAD_PARAM;
    group = data[0];
    count = data[1];
    if (count == 0u || count > MAX_SET_ENTRIES ||
        length != SET_GROUP_HEADER_BYTES + (size_t)count * SET_ENTRY_BYTES)
        return WIRELESS_ACK_BAD_PARAM;
    for (i = 0u; i < count; ++i) {
        entries[i].id = data[2u + (size_t)i * SET_ENTRY_BYTES];
        entries[i].value = get_i32(data + 3u + (size_t)i * SET_ENTRY_BYTES);
    }
    status = wireless_settings_apply_group(&control->settings, group, entries,
                                           count,
                                           (uint8_t)(control->runtime.follow_active != 0u ||
                                                     control->runtime.parked == 0u));
    if (status != WIRELESS_SETTINGS_OK) {
        if (status == WIRELESS_SETTINGS_NOT_PARKED) return WIRELESS_ACK_NOT_PARKED;
        if (status == WIRELESS_SETTINGS_OUT_OF_RANGE) return WIRELESS_ACK_RANGE;
        return WIRELESS_ACK_BAD_PARAM;
    }
    ++control->settings_generation;
    ack->detail = ((uint32_t)group << 16u) | count;
    return WIRELESS_ACK_OK;
}

wireless_ack_status_t wireless_control_command(
    wireless_control_t *control, uint32_t sequence, uint32_t session,
    uint32_t command_timestamp_ms, uint32_t now_ms,
    const uint8_t *payload, size_t payload_length, wireless_control_ack_t *ack)
{
    uint8_t opcode = payload != NULL && payload_length > 0u ? payload[0] : 0u;
    uint8_t target = payload != NULL && payload_length > 1u ? payload[1] : 0u;
    wireless_ack_status_t status;
    if (control == NULL || ack == NULL) return WIRELESS_ACK_BAD_FRAME;
    ack_init(ack, sequence, opcode, target);
    if (!control->session_ready || session != control->session)
        return finish_uncached(control, ack, WIRELESS_ACK_BAD_SESSION, 0u);
    if (!elapsed_ok(now_ms, command_timestamp_ms))
        return finish_uncached(control, ack, WIRELESS_ACK_STALE, 0u);
    if (payload == NULL || payload_length < COMMAND_HEADER_BYTES ||
        payload[2] != 0u || payload[3] != 0u || target > WIRELESS_TARGET_CAMERA)
        return finish_uncached(control, ack, WIRELESS_ACK_BAD_FRAME, 0u);
    if (control->have_command_seq && sequence == control->last_command_seq) {
        *ack = control->cached_ack;
        ack->flags |= WIRELESS_ACK_FLAG_DUPLICATE;
        return (wireless_ack_status_t)ack->status;
    }
    if (control->have_command_seq && !seq_newer(sequence, control->last_command_seq))
        return finish_uncached(control, ack, WIRELESS_ACK_OUT_OF_ORDER, 0u);
    control->have_command_seq = 1u;
    control->last_command_seq = sequence;
    if (control->test.active != 0u &&
        !command_allowed_while_test_active(opcode))
        return finish(control, ack, WIRELESS_ACK_BUSY, 0u);
    if (target == WIRELESS_TARGET_CAMERA)
        return finish(control, ack,
                      camera_target_command(control, opcode, payload,
                                            payload_length), 0u);

    switch (opcode) {
    case WIRELESS_OP_INFO:
        status = WIRELESS_ACK_OK;
        ack->detail = control->test.active != 0u ? 1u : 0u;
        break;
    case WIRELESS_OP_PAGE:
        if (payload_length != 5u || payload[4] > 3u) status = WIRELESS_ACK_RANGE;
        else { control->parameter_page = payload[4]; ack->detail = payload[4];
               status = WIRELESS_ACK_OK; }
        break;
    case WIRELESS_OP_GET_PARAMS:
        status = WIRELESS_ACK_OK;
        ack->detail = control->parameter_page;
        break;
    case WIRELESS_OP_START:
        if (control->runtime.hard_fault != 0u ||
            control->runtime.command_inhibited != 0u) status = WIRELESS_ACK_FAULT;
        else { control->start_requested = 1u; status = WIRELESS_ACK_PENDING; }
        break;
    case WIRELESS_OP_STOP:
        wireless_control_revoke_outputs(control, now_ms);
        status = WIRELESS_ACK_OK;
        break;
    case WIRELESS_OP_SET_PARAMS:
        if (control->runtime.calibration_busy != 0u ||
            control->runtime.capture_busy != 0u)
            status = WIRELESS_ACK_BUSY;
        else
            status = command_set_params(control, payload + COMMAND_HEADER_BYTES,
                                        payload_length - COMMAND_HEADER_BYTES, ack);
        break;
    case WIRELESS_OP_SAVE:
        if (!is_parked_and_safe(control)) status = WIRELESS_ACK_NOT_PARKED;
        else { control->save_requested = 1u; status = WIRELESS_ACK_PENDING; }
        break;
    case WIRELESS_OP_DEFAULT:
        if (!is_parked_and_safe(control)) status = WIRELESS_ACK_NOT_PARKED;
        else { control->settings = control->defaults; ++control->settings_generation;
               control->default_requested = 1u; status = WIRELESS_ACK_OK; }
        break;
    case WIRELESS_OP_TEST_MOTOR:
        status = command_motor_test(control, payload + COMMAND_HEADER_BYTES,
                                    payload_length - COMMAND_HEADER_BYTES, now_ms, ack);
        break;
    case WIRELESS_OP_TEST_SERVO:
        status = command_servo_test(control, payload + COMMAND_HEADER_BYTES,
                                    payload_length - COMMAND_HEADER_BYTES, now_ms, ack);
        break;
    case WIRELESS_OP_IMU_CAL:
        if (!is_parked_and_safe(control)) status = WIRELESS_ACK_NOT_PARKED;
        else { control->imu_cal_requested = 1u; status = WIRELESS_ACK_PENDING; }
        break;
    case WIRELESS_OP_CAMERA_CAL:
        if (!is_parked_and_safe(control)) status = WIRELESS_ACK_NOT_PARKED;
        else { control->camera_cal_requested = 1u; status = WIRELESS_ACK_PENDING; }
        break;
    case WIRELESS_OP_CAPTURE:
        if (!is_parked_and_safe(control)) status = WIRELESS_ACK_NOT_PARKED;
        else if (control->capture_requested) status = WIRELESS_ACK_BUSY;
        else { control->capture_requested = 1u; status = WIRELESS_ACK_PENDING; }
        break;
    case WIRELESS_OP_MODE:
        if (payload_length != 5u || payload[4] > WIRELESS_TOOL_MODE_COMPLETE)
            status = WIRELESS_ACK_BAD_PARAM;
        else { control->tool_mode = payload[4]; status = WIRELESS_ACK_OK; }
        break;
    default:
        status = WIRELESS_ACK_UNSUPPORTED;
        break;
    }
    return finish(control, ack, status, ack->detail);
}

const wireless_settings_t *wireless_control_settings(const wireless_control_t *control)
{
    return control == NULL ? NULL : &control->settings;
}

wireless_control_test_t wireless_control_test(const wireless_control_t *control)
{
    return control == NULL ? (wireless_control_test_t){0} : control->test;
}

#define TAKE_FLAG(field) \
    uint8_t value = control != NULL ? control->field : 0u; \
    if (control != NULL) control->field = 0u; \
    return value

uint8_t wireless_control_take_start(wireless_control_t *control) { TAKE_FLAG(start_requested); }
uint8_t wireless_control_take_stop(wireless_control_t *control) { TAKE_FLAG(stop_requested); }
uint8_t wireless_control_take_imu_cal(wireless_control_t *control) { TAKE_FLAG(imu_cal_requested); }
uint8_t wireless_control_take_camera_cal(wireless_control_t *control) { TAKE_FLAG(camera_cal_requested); }
uint8_t wireless_control_take_capture(wireless_control_t *control) { TAKE_FLAG(capture_requested); }
uint8_t wireless_control_take_save(wireless_control_t *control) { TAKE_FLAG(save_requested); }
uint8_t wireless_control_take_default(wireless_control_t *control) { TAKE_FLAG(default_requested); }

uint8_t wireless_control_set_tool_mode(wireless_control_t *control, uint8_t mode)
{
    if (control == NULL || mode > WIRELESS_TOOL_MODE_COMPLETE) return 0u;
    control->tool_mode = mode;
    return 1u;
}
