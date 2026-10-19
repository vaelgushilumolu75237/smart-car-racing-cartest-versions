#include "car_calibration.h"
#include "car_vision.h"
#include "commissioning.h"
#include "manual_run_button.h"
#include "encoder_diag.h"
#include "follow_output.h"
#include "follow_speed_pi.h"
#include "reverse_control.h"
#include "run_block_reason.h"
#include "runtime_diag.h"
#include "runtime_log.h"
#include "servo_safety.h"
#include "speed_observer.h"
#include "speed_pi.h"
#include "uart_rx_service.h"
#include "vision_protocol.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

extern "C" uint8_t car_follow_output_turn_slope_assist_request(
    const car_follow_slope_assist_gate_t *gate);
extern "C" uint8_t car_follow_output_turn_update_with_slope_assist(
    car_follow_output_state_t *state,
    uint32_t now_ms,
    uint16_t base_target_permille,
    uint16_t outer_differential_permille,
    uint16_t inner_differential_permille,
    uint16_t servo_applied_us,
    uint16_t servo_center_us,
    uint16_t servo_left_endpoint_us,
    uint16_t servo_right_endpoint_us,
    uint8_t servo_output_active,
    uint8_t servo_frame_ready,
    uint8_t motor_outputs_active,
    uint16_t left_applied_permille,
    uint16_t right_applied_permille,
    uint8_t slope_assist_requested,
    uint16_t *left_command_permille,
    uint16_t *right_command_permille);
extern "C" void car_follow_output_turn_hold_at(
    car_follow_output_state_t *state,
    uint32_t now_ms,
    uint32_t hold_started_ms,
    uint32_t hold_entry_generation,
    uint16_t left_applied_permille,
    uint16_t right_applied_permille);
extern "C" void car_servo_turn_safety_init(car_servo_safety_t *state);
extern "C" uint8_t car_servo_turn_safety_request_selected_with_slew(
    car_servo_safety_t *state,
    car_servo_source_t source,
    uint16_t follow_target_us,
    uint16_t debug_target_us,
    uint8_t output_permitted,
    uint32_t now_ms,
    uint16_t slew_step_us,
    const car_servo_safety_io_t *io);

namespace {

struct WireVector {
    std::string name;
    uint16_t sequence = 0;
    uint8_t valid = 0;
    uint8_t confidence = 0;
    int16_t error = 0;
    uint8_t slope_hint = 0;
    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
};

std::vector<WireVector> g_vectors;
struct ReverseVector {
    std::string name;
    uint16_t sequence = 0;
    uint8_t command = 0;
    std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE> bytes{};
};
std::vector<ReverseVector> g_reverse_vectors;
unsigned g_case_count = 0;
unsigned g_check_count = 0;
unsigned g_fail_count = 0;

struct UartFifoMock {
    std::vector<uint8_t> fifo;
    size_t cursor = 0u;
    std::vector<uint8_t> accepted;
    uint32_t error_flags = 0u;
    size_t reads = 0u;
    size_t error_after_reads = 0u;
    size_t inject_after_reads = 0u;
    std::vector<uint8_t> injected_bytes;
    uint8_t injection_done = 0u;
    size_t abort_after_accepted = 0u;
    unsigned error_calls = 0u;
    unsigned abort_calls = 0u;
    unsigned flush_calls = 0u;
    uint8_t rx_irq_enabled = 1u;
    car_control_t *control = nullptr;
};

constexpr uint32_t UART_MOCK_RXFNE = 1u << 0u;
constexpr uint32_t UART_MOCK_ORE = 1u << 1u;

uint32_t uart_mock_read_status(void *context)
{
    auto *mock = static_cast<UartFifoMock *>(context);
    if (mock->error_after_reads != 0u &&
        mock->reads >= mock->error_after_reads) {
        mock->error_flags |= UART_MOCK_ORE;
    }
    return mock->error_flags |
           (mock->cursor < mock->fifo.size() ? UART_MOCK_RXFNE : 0u);
}

uint8_t uart_mock_read_data(void *context)
{
    auto *mock = static_cast<UartFifoMock *>(context);
    if (mock->cursor >= mock->fifo.size()) {
        return 0u;
    }
    const uint8_t byte = mock->fifo[mock->cursor++];
    ++mock->reads;
    if (mock->inject_after_reads != 0u && mock->injection_done == 0u &&
        mock->reads == mock->inject_after_reads) {
        mock->fifo.insert(mock->fifo.end(), mock->injected_bytes.begin(),
                          mock->injected_bytes.end());
        mock->injection_done = 1u;
    }
    return byte;
}

void uart_mock_on_error(void *context, uint32_t status)
{
    auto *mock = static_cast<UartFifoMock *>(context);
    ++mock->error_calls;
    mock->error_flags &= ~(status & (UART_MOCK_ORE | UART_MOCK_RXFNE));
    mock->rx_irq_enabled = 0u;
    mock->fifo.clear();
    mock->cursor = 0u;
    ++mock->flush_calls;
    if (mock->control != nullptr) {
        car_control_note_fast_stop_isr(mock->control, CAR_FAULT_UART);
    }
}

uint8_t uart_mock_on_byte(void *context, uint8_t byte)
{
    auto *mock = static_cast<UartFifoMock *>(context);
    mock->accepted.push_back(byte);
    return mock->abort_after_accepted != 0u &&
                   mock->accepted.size() >= mock->abort_after_accepted
               ? 1u : 0u;
}

void uart_mock_on_abort(void *context)
{
    auto *mock = static_cast<UartFifoMock *>(context);
    ++mock->abort_calls;
    mock->rx_irq_enabled = 0u;
    mock->fifo.clear();
    mock->cursor = 0u;
    ++mock->flush_calls;
    if (mock->control != nullptr) {
        car_control_note_fast_stop_isr(mock->control, CAR_FAULT_RX_OVERFLOW);
    }
}

car_uart_rx_service_io_t uart_mock_io(UartFifoMock *mock)
{
    car_uart_rx_service_io_t io{};
    io.context = mock;
    io.rxne_mask = UART_MOCK_RXFNE;
    io.error_mask = UART_MOCK_ORE;
    io.read_status = uart_mock_read_status;
    io.read_data = uart_mock_read_data;
    io.on_error = uart_mock_on_error;
    io.on_byte = uart_mock_on_byte;
    io.on_abort = uart_mock_on_abort;
    return io;
}

void check(bool condition, const char *name, const char *file, int line)
{
    ++g_check_count;
    if (!condition) {
        ++g_fail_count;
        std::printf("FAIL %s (%s:%d)\n", name, file, line);
    }
}

#define CHECK(expression, name) \
    check((expression), (name), __FILE__, __LINE__)

#define CHECK_EQ(name, actual, expected) \
    check(static_cast<long long>(actual) == static_cast<long long>(expected), \
          (name), __FILE__, __LINE__)

void run_case(const char *name, void (*test)())
{
    const unsigned failures_before = g_fail_count;
    ++g_case_count;
    test();
    std::printf("CASE %s %s\n", name,
                g_fail_count == failures_before ? "PASS" : "FAIL");
}

bool parse_csv_vectors(const char *path)
{
    std::ifstream input(path);
    std::string line;
    if (!input.is_open()) {
        std::printf("FAIL cannot open shared vector file: %s\n", path);
        return false;
    }

    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#' || line.rfind("name,", 0) == 0) {
            continue;
        }
        std::stringstream row(line);
        std::array<std::string, 7> fields{};
        for (std::string &field : fields) {
            if (!std::getline(row, field, ',')) {
                std::printf("FAIL malformed CSV row: %s\n", line.c_str());
                return false;
            }
        }
        try {
            WireVector vector;
            vector.name = fields[0];
            vector.sequence = static_cast<uint16_t>(std::stoul(fields[1]));
            vector.valid = static_cast<uint8_t>(std::stoul(fields[2]));
            vector.confidence = static_cast<uint8_t>(std::stoul(fields[3]));
            vector.error = static_cast<int16_t>(std::stoi(fields[4]));
            vector.slope_hint = static_cast<uint8_t>(std::stoul(fields[5]));
            if (vector.slope_hint > 1u ||
                (vector.valid == 0u && vector.slope_hint != 0u)) {
                std::printf("FAIL invalid slope hint in vector: %s\n",
                            vector.name.c_str());
                return false;
            }
            if (fields[6].size() != VP_FRAME_SIZE * 2u) {
                std::printf("FAIL wrong hex byte count in vector: %s\n",
                            vector.name.c_str());
                return false;
            }
            for (size_t i = 0; i < VP_FRAME_SIZE; ++i) {
                vector.bytes[i] = static_cast<uint8_t>(
                    std::stoul(fields[6].substr(i * 2u, 2u), nullptr, 16));
            }
            g_vectors.push_back(vector);
        } catch (const std::exception &exception) {
            std::printf("FAIL parsing CSV row '%s': %s\n", line.c_str(),
                        exception.what());
            return false;
        }
    }
    if (!input.eof() || g_vectors.size() != 10u) {
        std::printf("FAIL shared vector file must contain exactly ten rows; found %zu\n",
                    g_vectors.size());
        return false;
    }
    return true;
}

bool parse_reverse_vectors(const char *path)
{
    std::ifstream input(path);
    std::string line;
    if (!input.is_open()) {
        std::printf("FAIL cannot open shared reverse-control vector file: %s\n",
                    path);
        return false;
    }
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#' || line.rfind("case,", 0) == 0) {
            continue;
        }
        std::stringstream row(line);
        std::array<std::string, 4> fields{};
        for (std::string &field : fields) {
            if (!std::getline(row, field, ',')) {
                std::printf("FAIL malformed reverse CSV row: %s\n", line.c_str());
                return false;
            }
        }
        try {
            ReverseVector vector;
            vector.name = fields[0];
            vector.sequence = static_cast<uint16_t>(std::stoul(fields[1]));
            vector.command = static_cast<uint8_t>(
                fields[2] == "RESET" ? CAR_REVERSE_CONTROL_RESET
                                      : CAR_REVERSE_CONTROL_KEEP);
            if ((fields[2] != "RESET" && fields[2] != "KEEP") ||
                fields[3].size() != CAR_REVERSE_CONTROL_FRAME_SIZE * 2u) {
                std::printf("FAIL invalid reverse vector: %s\n",
                            vector.name.c_str());
                return false;
            }
            for (size_t i = 0; i < CAR_REVERSE_CONTROL_FRAME_SIZE; ++i) {
                vector.bytes[i] = static_cast<uint8_t>(
                    std::stoul(fields[3].substr(i * 2u, 2u), nullptr, 16));
            }
            g_reverse_vectors.push_back(vector);
        } catch (const std::exception &exception) {
            std::printf("FAIL parsing reverse CSV row '%s': %s\n",
                        line.c_str(), exception.what());
            return false;
        }
    }
    if (!input.eof() || g_reverse_vectors.size() != 4u) {
        std::printf("FAIL reverse vector file must contain four rows; found %zu\n",
                    g_reverse_vectors.size());
        return false;
    }
    return true;
}

car_config_t make_config(bool bench = false, bool servo_measured = false)
{
    car_config_t config{};
    config.bench_mode_compiled = bench ? 1u : 0u;
    config.servo_limits_measured = servo_measured ? 1u : 0u;
    config.servo_min_us = servo_measured ? CARTEST_SERVO_MIN_US : 0u;
    config.servo_max_us = servo_measured ? CARTEST_SERVO_MAX_US : 0u;
    config.servo_center_us = CARTEST_SERVO_CENTER_US;
    config.bench_duty_limit_permille = bench ? 400u : 0u;
    config.bench_ramp_step_permille = bench ? 100u : 0u;
    config.bench_run_limit_ms = bench ? 55u : 0u;
    config.vision_timeout_ms = 250u;
    config.motor_hardware_accepted = 1u;
    return config;
}

/* Synthetic fixture only: these values are chosen to exercise the host state
   machine and are not physical servo/encoder measurements. */
car_config_t make_follow_config(bool calibrated = true,
                                bool positive_error_right = true,
                                uint16_t left_us = 1460u,
                                uint16_t right_us = 1610u)
{
    car_config_t config{};
    car_config_init_application(&config, 0u, 0u, 1u, 1u);
    config.follow_calibration_ready = calibrated ? 1u : 0u;
    config.follow_positive_error_right_measured =
        positive_error_right ? 1u : 0u;
    config.servo_limits_measured = 1u;
    config.servo_min_us = left_us < right_us ? left_us : right_us;
    config.servo_max_us = left_us > right_us ? left_us : right_us;
    config.follow_servo_left_us = left_us;
    config.follow_servo_right_us = right_us;
    config.follow_left_encoder_counts_per_rev = 360u;
    config.follow_right_encoder_counts_per_rev = 420u;
    config.follow_left_encoder_sign = 1;
    config.follow_right_encoder_sign = -1;
    return config;
}

car_config_t make_hold_follow_config()
{
    /* Hold behavior is exercised against the checked-in measured Follow
       endpoints so error_norm=545 maps to 1900 us at the production gain. */
    return make_follow_config(true, true, CARTEST_SERVO_LEFT_US,
                              CARTEST_SERVO_RIGHT_US);
}

car_config_t make_debug_steering_config()
{
    car_config_t config = make_config(false, true);
    config.debug_steering_mode_compiled = 1u;
    config.follow_mode_compiled = 0u;
    config.follow_calibration_ready = CARTEST_FOLLOW_CALIBRATION_COMPLETE;
    config.follow_positive_error_right_measured =
        CARTEST_STEERING_DIRECTION_CONFIRMED;
    config.follow_kp_permille = 700u;
    return config;
}

car_inputs_t make_inputs(uint32_t now_ms, bool switch_high)
{
    car_inputs_t inputs{};
    inputs.now_ms = now_ms;
    inputs.pe7_raw_high = switch_high ? 1u : 0u;
    inputs.pe7_high_stable = switch_high ? 1u : 0u;
    inputs.pe7_low_stable = switch_high ? 0u : 1u;
    /* Legacy controller tests model one already-consumed low press while the
       line is held low. Dedicated button tests exercise the production
       high-20ms rearm / low-edge toggle helper and release-does-not-stop. */
    inputs.manual_run_request = switch_high ? 0u : 1u;
    inputs.manual_button_startup_release_confirmed = switch_high ? 1u : 0u;
    return inputs;
}

car_inputs_t make_debounced_inputs(uint32_t now_ms,
                                   bool raw_switch_high,
                                   bool stable_high,
                                   bool stable_low)
{
    car_inputs_t inputs = make_inputs(now_ms, raw_switch_high);
    inputs.pe7_high_stable = stable_high ? 1u : 0u;
    inputs.pe7_low_stable = stable_low ? 1u : 0u;
    inputs.manual_run_request = stable_low ? 1u : 0u;
    inputs.manual_button_startup_release_confirmed = stable_high ? 1u : 0u;
    return inputs;
}

void confirm_follow_startup_release(car_control_t *control,
                                    const car_config_t *config,
                                    uint32_t now_ms)
{
    car_inputs_t released = make_inputs(now_ms, true);
    released.manual_button_startup_release_confirmed = 1u;
    car_control_tick(control, config, &released);
    if (control->rx_recovery_flush_required != 0u) {
        car_control_rx_recovery_flush_complete(control);
    }
}

void send_frame(car_control_t *control,
                const car_config_t *config,
                uint16_t sequence,
                bool valid,
                uint8_t confidence,
                int16_t error,
                uint32_t arrival_ms,
                bool switch_high)
{
    car_control_on_vision_frame(control, config, sequence,
                                valid ? 1u : 0u, confidence, error,
                                arrival_ms, switch_high ? 1u : 0u);
}

void send_frame_at(car_control_t *control,
                   const car_config_t *config,
                   uint16_t sequence,
                   bool valid,
                   uint8_t confidence,
                   int16_t error,
                   uint32_t arrival_ms,
                   uint32_t now_ms,
                   bool switch_high,
                   bool slope_hint = false)
{
    const uint8_t flags = (valid ? VP_FLAG_VALID : 0u) |
                          (slope_hint ? VP_FLAG_SLOPE_HINT : 0u);
    car_control_on_vision_frame_at_with_flags(
        control, config, sequence, flags, confidence, error, arrival_ms,
        now_ms, switch_high ? 1u : 0u);
}

void establish_fresh_vision(car_control_t *control,
                            const car_config_t *config,
                            uint32_t now_ms)
{
    car_control_init(control, config, now_ms - 3u, 1u, 0u);
    car_control_rx_recovery_flush_complete(control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t frame_ms = now_ms - 2u + offset;
        send_frame_at(control, config, static_cast<uint16_t>(offset + 1u),
                      true, 80u, 0, frame_ms, frame_ms, true);
    }
}

vp_frame make_wire_frame(uint16_t sequence,
                         bool valid = true,
                         uint8_t confidence = 80u,
                         int16_t error = 0,
                         bool slope_hint = false)
{
    vp_frame frame{};
    frame.sequence = sequence;
    frame.flags = (valid ? VP_FLAG_VALID : 0u) |
                  (slope_hint ? VP_FLAG_SLOPE_HINT : 0u);
    frame.confidence = confidence;
    frame.lateral_error = error;
    return frame;
}

struct FakeServoRegisters {
    car_servo_timer_snapshot_t snapshot{};
    uint8_t update_pending = 0u;
    uint8_t fail_write = 0u;
    uint8_t corrupt_readback = 0u;
    uint8_t fail_start = 0u;
    unsigned write_count = 0u;
    unsigned start_count = 0u;
    unsigned stop_count = 0u;
};

FakeServoRegisters make_valid_fake_servo()
{
    FakeServoRegisters fake{};
    fake.snapshot.sysclk_hz = 64000000u;
    fake.snapshot.hclk_hz = 64000000u;
    fake.snapshot.pclk2_hz = 64000000u;
    fake.snapshot.rcc_cr = 0x5u; /* HSI on/ready, DIV1. */
    fake.snapshot.rcc_apb2enr = 0x1u; /* TIM1EN. */
    fake.snapshot.tim_ccmr1 = (1u << 3u) | (6u << 4u); /* OC1PE, PWM1. */
    fake.snapshot.tim_psc = 63u;
    fake.snapshot.tim_arr = 19999u;
    fake.snapshot.gpioa_moder = (1u << 16u); /* PA8 low GPIO output. */
    return fake;
}

void fake_servo_capture(void *context,
                        car_servo_timer_snapshot_t *snapshot)
{
    *snapshot = static_cast<FakeServoRegisters *>(context)->snapshot;
}

uint8_t fake_servo_write(void *context, uint16_t pulse_us)
{
    auto *fake = static_cast<FakeServoRegisters *>(context);
    ++fake->write_count;
    if (fake->fail_write != 0u) {
        return 0u;
    }
    fake->snapshot.tim_ccr1 = pulse_us;
    return 1u;
}

uint32_t fake_servo_read(void *context)
{
    const auto *fake = static_cast<const FakeServoRegisters *>(context);
    return fake->snapshot.tim_ccr1 + fake->corrupt_readback;
}

uint8_t fake_servo_start(void *context)
{
    auto *fake = static_cast<FakeServoRegisters *>(context);
    ++fake->start_count;
    if (fake->fail_start != 0u) {
        return 0u;
    }
    fake->snapshot.gpioa_moder =
        (fake->snapshot.gpioa_moder & ~(3u << 16u)) | (2u << 16u);
    fake->snapshot.gpioa_afr1 =
        (fake->snapshot.gpioa_afr1 & ~0xFu) | 1u;
    fake->snapshot.tim_cr1 |= 1u; /* CEN. */
    fake->snapshot.tim_ccer = 1u; /* CC1E. */
    fake->snapshot.tim_bdtr = 1u << 15u; /* MOE. */
    fake->update_pending = 0u; /* Startup UG is not a completed frame. */
    return 1u;
}

uint8_t fake_servo_take_update(void *context)
{
    auto *fake = static_cast<FakeServoRegisters *>(context);
    const uint8_t pending = fake->update_pending;
    fake->update_pending = 0u;
    return pending;
}

void fake_servo_stop(void *context)
{
    auto *fake = static_cast<FakeServoRegisters *>(context);
    ++fake->stop_count;
    fake->snapshot.tim_cr1 &= ~1u;
    fake->snapshot.tim_ccer = 0u;
    fake->snapshot.tim_bdtr = 0u;
    fake->snapshot.tim_ccr1 = 0u;
    fake->snapshot.gpioa_moder =
        (fake->snapshot.gpioa_moder & ~(3u << 16u)) | (1u << 16u);
    fake->snapshot.gpioa_odr &= ~(1u << 8u);
}

car_servo_safety_io_t fake_servo_io(FakeServoRegisters *fake)
{
    car_servo_safety_io_t io{};
    io.context = fake;
    io.capture = fake_servo_capture;
    io.write_compare = fake_servo_write;
    io.read_compare = fake_servo_read;
    io.start_output = fake_servo_start;
    io.take_completed_update = fake_servo_take_update;
    io.stop_output = fake_servo_stop;
    return io;
}

bool pack_frame(const vp_frame &frame,
                std::array<uint8_t, VP_FRAME_SIZE> *bytes)
{
    return vp_pack(&frame, bytes->data()) != 0u;
}

void enqueue_frame(car_vision_rx_t *rx,
                   const vp_frame &frame,
                   uint32_t arrival_ms,
                   uint32_t spacing_ms = 0u)
{
    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    if (!pack_frame(frame, &bytes)) {
        check(false, "test helper packs valid frame", __FILE__, __LINE__);
        return;
    }
    for (size_t i = 0; i < bytes.size(); ++i) {
        car_vision_rx_isr_byte(rx, bytes[i],
                              arrival_ms + static_cast<uint32_t>(i) * spacing_ms);
    }
}

void poll_rx_current_snapshot(car_vision_rx_t *rx,
                              car_control_t *control,
                              const car_config_t *config,
                              uint32_t now_ms,
                              bool switch_high)
{
    const uint16_t head_snapshot = rx->head;
    car_vision_rx_poll(rx, control, config, head_snapshot, now_ms,
                       switch_high ? 1u : 0u);
}

uint32_t select_bench(car_control_t *control,
                      const car_config_t *config,
                      uint8_t motor = CARTEST_BENCH_MOTOR,
                      uint8_t input = CAR_MOTOR_IN1,
                      uint16_t duty = CARTEST_BENCH_DUTY_PERMILLE)
{
    const car_inputs_t inputs = make_inputs(0u, true);
    return car_control_command(control, config, &inputs,
                               CAR_CMD_SELECT_BENCH_OUTPUT,
                               motor, input, duty);
}

void start_bench_with_fresh_frame(car_control_t *control,
                                  const car_config_t *config,
                                  uint16_t sequence = 1u)
{
    car_control_init(control, config, 0u, 1u, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(control, config, &released);
    (void)select_bench(control, config);
    car_control_rx_recovery_flush_complete(control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t frame_ms = 1u + offset;
        send_frame_at(control, config, static_cast<uint16_t>(sequence + offset),
                      true, 80u, 0, frame_ms, frame_ms, true);
    }
    const car_inputs_t press_edge = make_debounced_inputs(4u, false, true, false);
    car_control_tick(control, config, &press_edge);
    const car_inputs_t press_stable = make_debounced_inputs(24u, false, false, true);
    car_control_tick(control, config, &press_stable);
}

void start_bench_with_fresh_frame_at(car_control_t *control,
                                     const car_config_t *config,
                                     uint32_t start_ms,
                                     uint16_t sequence = 1u)
{
    car_control_init(control, config, start_ms, 1u, 0u);
    car_inputs_t released = make_inputs(start_ms, true);
    car_control_tick(control, config, &released);
    const uint32_t select_effects = car_control_command(
        control, config, &released, CAR_CMD_SELECT_BENCH_OUTPUT,
        CARTEST_BENCH_MOTOR, CARTEST_BENCH_INPUT,
        static_cast<uint16_t>(CARTEST_BENCH_DUTY_PERMILLE));
    CHECK((select_effects & CAR_EFFECT_CLEAR_RX) != 0u,
          "wrapped-time start selects Bench and flushes RX");
    car_control_rx_recovery_flush_complete(control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t frame_ms = start_ms + 1u + offset;
        send_frame_at(control, config,
                      static_cast<uint16_t>(sequence + offset),
                      true, 80u, 0, frame_ms, frame_ms, true);
    }
    const car_inputs_t press_edge =
        make_debounced_inputs(start_ms + 4u, false, true, false);
    car_control_tick(control, config, &press_edge);
    const car_inputs_t press_stable =
        make_debounced_inputs(start_ms + 24u, false, false, true);
    car_control_tick(control, config, &press_stable);
}

void test_shared_protocol_vectors()
{
    CHECK_EQ("ten shared vectors loaded", g_vectors.size(), 10u);
    for (const WireVector &vector : g_vectors) {
        vp_frame frame{};
        frame.sequence = vector.sequence;
        frame.flags = (vector.valid != 0u ? VP_FLAG_VALID : 0u) |
                      (vector.slope_hint != 0u ? VP_FLAG_SLOPE_HINT : 0u);
        frame.confidence = vector.confidence;
        frame.lateral_error = vector.error;

        std::array<uint8_t, VP_FRAME_SIZE> packed{};
        CHECK(vp_pack(&frame, packed.data()) != 0u, vector.name.c_str());
        for (size_t i = 0; i < VP_FRAME_SIZE; ++i) {
            CHECK_EQ("shared vector exact wire byte", packed[i], vector.bytes[i]);
        }

        vp_frame decoded{};
        CHECK(vp_unpack(vector.bytes.data(), &decoded) != 0u, vector.name.c_str());
        CHECK_EQ("shared vector sequence", decoded.sequence, vector.sequence);
        CHECK_EQ("shared vector valid flag", decoded.flags,
                 (vector.valid != 0u ? VP_FLAG_VALID : 0u) |
                     (vector.slope_hint != 0u ? VP_FLAG_SLOPE_HINT : 0u));
        CHECK_EQ("shared vector confidence", decoded.confidence, vector.confidence);
        CHECK_EQ("shared vector lateral error", decoded.lateral_error, vector.error);
    }
}

void test_reverse_control_shared_vectors_and_lifecycle()
{
    for (const ReverseVector &vector : g_reverse_vectors) {
        std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE> encoded{};
        car_reverse_control_encode(vector.command, vector.sequence,
                                  encoded.data());
        CHECK(encoded == vector.bytes,
              "reverse-control packer matches shared RESET/KEEP vector");
    }

    car_reverse_control_tx_t tx{};
    car_reverse_control_init(&tx, 0u);
    std::vector<std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE>> frames;
    std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE> frame{};
    for (uint32_t timestamp = 0u; timestamp <= 200u; timestamp += 50u) {
        for (uint8_t index = 0u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
            uint8_t byte = 0u;
            CHECK_EQ("RESET byte is available at scheduled boundary",
                     car_reverse_control_next_byte(&tx, timestamp, 0u, &byte), 1u);
            frame[index] = byte;
        }
        frames.push_back(frame);
    }
    CHECK_EQ("boot emits five complete RESET frames", frames.size(), 5u);
    for (size_t index = 0; index < frames.size(); ++index) {
        CHECK_EQ("boot reset frame command is RESET", frames[index][3],
                 CAR_REVERSE_CONTROL_RESET);
        CHECK_EQ("boot reset sequence increments", frames[index][4], index);
        CHECK_EQ("boot reset sequence high byte remains zero", frames[index][5], 0u);
    }
    CHECK_EQ("RESET span covers the 250 ms lease expiry boundary",
             tx.reset_phase_complete, 1u);
    uint8_t byte = 0u;
    CHECK_EQ("no early KEEP before 250 ms",
             car_reverse_control_next_byte(&tx, 249u, 1u, &byte), 0u);
    CHECK_EQ("KEEP waits when acquisition is not permitted",
             car_reverse_control_next_byte(&tx, 250u, 0u, &byte), 0u);
    for (uint8_t index = 0u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
        CHECK_EQ("KEEP is sent after lease and permission",
                 car_reverse_control_next_byte(&tx, 250u, 1u, &byte), 1u);
        frame[index] = byte;
    }
    CHECK_EQ("KEEP command follows RESET phase", frame[3],
             CAR_REVERSE_CONTROL_KEEP);
    CHECK_EQ("KEEP sequence continues after RESET frames", frame[4], 5u);
    CHECK_EQ("KEEP marks lease active only after complete frame",
             tx.keep_active, 1u);
    CHECK_EQ("KEEP period does not catch up early",
             car_reverse_control_next_byte(&tx, 299u, 1u, &byte), 0u);

    car_reverse_control_request_reset(&tx, 301u);
    CHECK_EQ("new lifecycle RESET clears active lease",
             tx.keep_active, 0u);
    CHECK_EQ("RESET request increments generation", tx.reset_generation, 1u);

    car_reverse_control_tx_t partial{};
    car_reverse_control_init(&partial, 100u);
    std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE> first{};
    for (uint8_t index = 0u; index < 3u; ++index) {
        CHECK_EQ("partial frame begins", car_reverse_control_next_byte(
                     &partial, 100u, 0u, &first[index]), 1u);
    }
    car_reverse_control_request_reset(&partial, 101u);
    for (uint8_t index = 3u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
        CHECK_EQ("pending RESET does not cut a partial frame",
                 car_reverse_control_next_byte(&partial, 101u, 0u,
                                               &first[index]), 1u);
    }
    CHECK_EQ("partial frame remains an intact RESET", first[3],
             CAR_REVERSE_CONTROL_RESET);
    CHECK_EQ("next byte starts the queued RESET lifecycle",
             car_reverse_control_next_byte(&partial, 101u, 0u, &byte), 1u);
    CHECK_EQ("queued RESET starts at sync byte", byte, 0xA6u);

    car_reverse_control_tx_t partial_keep{};
    car_reverse_control_init(&partial_keep, 0u);
    for (uint32_t timestamp = 0u; timestamp <= 200u; timestamp += 50u) {
        for (uint8_t index = 0u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
            CHECK_EQ("RESET phase completes before partial KEEP fixture",
                     car_reverse_control_next_byte(&partial_keep, timestamp,
                                                   0u, &byte), 1u);
        }
    }
    for (uint8_t index = 0u; index < 3u; ++index) {
        CHECK_EQ("partial KEEP frame begins", car_reverse_control_next_byte(
                     &partial_keep, 250u, 1u, &byte), 1u);
    }
    car_reverse_control_request_reset(&partial_keep, 251u);
    for (uint8_t index = 3u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
        CHECK_EQ("pending RESET lets an old KEEP finish intact",
                 car_reverse_control_next_byte(&partial_keep, 251u, 1u,
                                               &byte), 1u);
    }
    CHECK_EQ("completed pre-RESET KEEP cannot reopen lease",
             partial_keep.keep_active, 0u);
    CHECK_EQ("new RESET starts immediately after old KEEP ends",
             car_reverse_control_next_byte(&partial_keep, 251u, 1u,
                                           &byte), 1u);
    CHECK_EQ("new lifecycle begins with RESET sync byte", byte, 0xA6u);
}

void test_servo_live_timer_validation()
{
    FakeServoRegisters valid = make_valid_fake_servo();
    CHECK_EQ("valid live HSI/TIM1 snapshot passes inactive guard",
             car_servo_safety_validate_timer(&valid.snapshot, 0u),
             CAR_SERVO_FAULT_NONE);
    CHECK_EQ("live APB timer formula resolves to 64 MHz",
             car_servo_safety_timer_clock_hz(&valid.snapshot), 64000000u);

    std::array<FakeServoRegisters, 14> invalid{};
    for (auto &fake : invalid) {
        fake = make_valid_fake_servo();
    }
    invalid[0].snapshot.rcc_cr |= 1u << 3u; /* HSI DIV2, stale Hz fields. */
    invalid[1].snapshot.rcc_cfgr = 0x12u; /* cached HCLK, live PLL SWS. */
    invalid[2].snapshot.rcc_d1cfgr = 8u; /* D1 HPRE changed. */
    invalid[3].snapshot.rcc_d2cfgr = 4u << 8u; /* APB2 prescaler. */
    invalid[4].snapshot.rcc_cfgr = 1u << 15u; /* TIMPRE. */
    invalid[5].snapshot.tim_psc = 62u;
    invalid[6].snapshot.tim_arr = 19998u;
    invalid[7].snapshot.tim_rcr = 1u;
    invalid[8].snapshot.tim_smcr = 1u; /* Slave/external clock mode. */
    invalid[9].snapshot.tim_dier = 1u << 8u; /* UDE/DMA write path. */
    invalid[10].snapshot.tim_ccmr1 = (1u << 3u) | (7u << 4u); /* PWM2. */
    invalid[11].snapshot.tim_ccer = 1u << 1u; /* Active-low polarity. */
    invalid[12].snapshot.gpioa_moder = 2u << 16u;
    invalid[12].snapshot.gpioa_afr1 = 2u; /* Wrong PA8 alternate function. */
    invalid[13].snapshot.rcc_d1cfgr = 1u << 8u; /* D1CPRE changed. */
    for (const auto &fake : invalid) {
        CHECK(car_servo_safety_validate_timer(&fake.snapshot, 0u) !=
                  CAR_SERVO_FAULT_NONE,
              "timer guard rejects a changed RCC/timer/pin route");
    }

    FakeServoRegisters active = make_valid_fake_servo();
    active.snapshot.tim_cr1 = 1u;
    active.snapshot.tim_ccer = 1u;
    active.snapshot.tim_bdtr = 1u << 15u;
    active.snapshot.gpioa_moder = 2u << 16u;
    active.snapshot.gpioa_afr1 = 1u;
    CHECK_EQ("valid active timer and PA8 AF1 pass", car_servo_safety_validate_timer(
                 &active.snapshot, 1u), CAR_SERVO_FAULT_NONE);
    active.snapshot.tim_cr1 = 0u;
    CHECK_EQ("active software state rejects an unexpectedly stopped timer",
             car_servo_safety_validate_timer(&active.snapshot, 1u),
             CAR_SERVO_FAULT_TIMER_CONFIG);

    FakeServoRegisters unexpectedly_active = make_valid_fake_servo();
    unexpectedly_active.snapshot.tim_cr1 = 1u;
    CHECK_EQ("inactive software state rejects an unexpectedly running timer",
             car_servo_safety_validate_timer(&unexpectedly_active.snapshot, 0u),
             CAR_SERVO_FAULT_TIMER_CONFIG);
    unexpectedly_active.snapshot.tim_cr1 = 0u;
    unexpectedly_active.snapshot.tim_ccer = 1u;
    CHECK_EQ("inactive software state rejects an enabled channel",
             car_servo_safety_validate_timer(&unexpectedly_active.snapshot, 0u),
             CAR_SERVO_FAULT_TIMER_CONFIG);
    unexpectedly_active.snapshot.tim_ccer = 0u;
    unexpectedly_active.snapshot.tim_bdtr = 1u << 15u;
    CHECK_EQ("inactive software state rejects enabled advanced output",
             car_servo_safety_validate_timer(&unexpectedly_active.snapshot, 0u),
             CAR_SERVO_FAULT_TIMER_CONFIG);
}

void test_servo_live_clock_profile_derivation()
{
    FakeServoRegisters valid = make_valid_fake_servo();
    valid.snapshot.sysclk_hz = 0u;
    valid.snapshot.hclk_hz = 0u;
    valid.snapshot.pclk2_hz = 0u;
    CHECK_EQ("live HSI64 profile is accepted",
             car_servo_safety_apply_fixed_hsi64_profile(&valid.snapshot), 1u);
    CHECK_EQ("accepted RCC profile derives SYSCLK from live registers",
             valid.snapshot.sysclk_hz, 64000000u);
    CHECK_EQ("accepted RCC profile derives HCLK from live registers",
             valid.snapshot.hclk_hz, 64000000u);
    CHECK_EQ("accepted RCC profile derives PCLK2 from live registers",
             valid.snapshot.pclk2_hz, 64000000u);

    std::array<car_servo_timer_snapshot_t, 8> invalid{};
    for (auto &snapshot : invalid) {
        snapshot = make_valid_fake_servo().snapshot;
        snapshot.sysclk_hz = 123u;
        snapshot.hclk_hz = 456u;
        snapshot.pclk2_hz = 789u;
    }
    invalid[0].rcc_cr &= ~(1u << 2u); /* HSIRDY clear. */
    invalid[1].rcc_cr |= 1u << 3u; /* HSI divided. */
    invalid[2].rcc_cfgr |= 1u; /* SW not HSI. */
    invalid[3].rcc_cfgr |= 1u << 3u; /* SWS not HSI. */
    invalid[4].rcc_cfgr |= 1u << 15u; /* TIMPRE. */
    invalid[5].rcc_d1cfgr |= 1u; /* HPRE. */
    invalid[6].rcc_d1cfgr |= 1u << 8u; /* D1CPRE. */
    invalid[7].rcc_d2cfgr |= 1u << 8u; /* PPRE2. */
    for (auto &snapshot : invalid) {
        CHECK_EQ("nonmatching live RCC profile is rejected",
                 car_servo_safety_apply_fixed_hsi64_profile(&snapshot), 0u);
        CHECK_EQ("rejected profile cannot reuse SYSCLK cache",
                 snapshot.sysclk_hz, 0u);
        CHECK_EQ("rejected profile cannot reuse HCLK cache",
                 snapshot.hclk_hz, 0u);
        CHECK_EQ("rejected profile cannot reuse PCLK2 cache",
                 snapshot.pclk2_hz, 0u);
    }
    CHECK_EQ("null clock snapshot is rejected",
             car_servo_safety_apply_fixed_hsi64_profile(nullptr), 0u);
}

void test_follow_output_differential_and_independent_ramps()
{
    car_follow_output_state_t state{};
    uint16_t left = 0u;
    uint16_t right = 0u;

    CHECK_EQ("centered applied servo pulse has no differential",
             car_follow_output_update(&state, 0u, 500u, 100u, 100u, 1530u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      0u, 0u, 0u, &left, &right), 1u);
    CHECK_EQ("startup waits for applied motor base before qualifying",
             state.startup_qualified, 0u);
    CHECK_EQ("unstarted left command remains actual zero", left, 0u);
    CHECK_EQ("unstarted right command remains actual zero", right, 0u);
    CHECK_EQ("base startup target is straight on left",
             state.target_left_permille, 500u);
    CHECK_EQ("base startup target is straight on right",
             state.target_right_permille, 500u);

    car_follow_output_reset(&state);
    CHECK_EQ("a complete active pulse is still required before qualification",
             car_follow_output_update(&state, 0u, 500u, 100u, 100u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 0u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("motor duties alone cannot qualify before full servo frame",
             state.startup_qualified, 0u);
    CHECK_EQ("pre-frame output stays straight despite an endpoint request",
             state.target_left_permille, 500u);

    CHECK_EQ("first full PWM and both actual base duties qualify startup",
             car_follow_output_update(&state, 10u, 500u, 100u, 100u, 1530u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("startup qualification is latched only at actual 500/500",
             state.startup_qualified, 1u);
    CHECK_EQ("zero differential command remains 500/500", left, 500u);
    CHECK_EQ("center gives normalized steering zero",
             state.differential_u_permille, 0u);

    CHECK_EQ("right-turn endpoint updates from applied servo pulse",
             car_follow_output_update(&state, 20u, 500u, 100u, 100u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("right-turn endpoint has full normalized amount",
             state.differential_u_permille, 1000u);
    CHECK_EQ("right turn targets outer left at the 600 normal ceiling",
             state.target_left_permille, 600u);
    CHECK_EQ("right turn targets inner right at 400",
             state.target_right_permille, 400u);
    CHECK_EQ("left side advances only one 10-permille step", left, 510u);
    CHECK_EQ("right side independently advances one 10-permille step",
             right, 490u);

    CHECK_EQ("left turn mirrors the right-turn target allocation",
             car_follow_output_update(&state, 30u, 500u, 100u, 100u, 1100u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("left turn targets inner left at 400", state.target_left_permille,
             400u);
    CHECK_EQ("left turn targets outer right at 600", state.target_right_permille,
             600u);
    CHECK_EQ("left-side target change is independently ramp limited",
             left, 490u);
    CHECK_EQ("right-side target change is independently ramp limited",
             right, 510u);

    CHECK_EQ("medium differential endpoint remains symmetric",
             car_follow_output_update(&state, 40u, 500u, 50u, 50u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("medium right-turn outer target is 550", state.target_left_permille,
             550u);
    CHECK_EQ("medium right-turn inner target is 450", state.target_right_permille,
             450u);

    CHECK_EQ("long scheduling delay still advances at most one ramp step",
             car_follow_output_update(&state, 1000u, 500u, 50u, 50u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("delayed left command does not catch up in one tick", left, 510u);
    CHECK_EQ("delayed right command does not catch up in one tick", right,
             490u);

    car_follow_output_hold(&state, 1001u, 237u, 418u);
    CHECK_EQ("hold freezes the applied left side independently",
             state.command_left_permille, 237u);
    CHECK_EQ("hold freezes the applied right side independently",
             state.command_right_permille, 418u);
    CHECK_EQ("hold retains completed startup qualification",
             state.startup_qualified, 1u);
    CHECK_EQ("hold phase remains visible", state.phase,
             CAR_FOLLOW_OUTPUT_PHASE_VISION_HOLD);

    car_follow_output_reset(&state);
    car_follow_output_hold_at(&state, 1150u, 1000u, 7u, 600u, 500u);
    CHECK_EQ("delayed hold service uses the actual 150 ms entry age",
             state.command_left_permille, 500u);
    CHECK_EQ("delayed hold service interpolates each side independently",
             state.command_right_permille, 417u);
    CHECK_EQ("delayed hold service retains the controller entry time",
             state.hold_started_ms, 1000u);
    CHECK_EQ("delayed hold service retains the unique entry generation",
             state.hold_entry_generation, 7u);
    car_follow_output_hold_at(&state, 1299u, 1000u, 7u, 100u, 100u);
    CHECK_EQ("isolated valid frame does not restart the fixed 300 ms ramp",
             state.command_left_permille, 401u);
    CHECK_EQ("isolated valid frame cannot replace the captured entry output",
             state.hold_entry_left_permille, 600u);
    car_follow_output_hold_at(&state, 1300u, 1000u, 7u, 100u, 100u);
    CHECK_EQ("hold reaches its fixed left target at 300 ms",
             state.command_left_permille, 400u);
    CHECK_EQ("hold reaches its fixed right target at 300 ms",
             state.command_right_permille, 333u);

    car_follow_output_hold_at(&state, 1300u, 1300u, 8u, 470u, 220u);
    CHECK_EQ("new hold entry token recaptures actual output immediately",
             state.command_left_permille, 470u);
    CHECK_EQ("new hold entry token replaces the prior entry generation",
             state.hold_entry_generation, 8u);
    car_follow_output_hold_at(&state, 1450u, 1300u, 8u, 0u, 0u);
    CHECK_EQ("new hold is interpolated from its own entry output",
             state.command_left_permille, 435u);
    CHECK_EQ("entry side below 300 permille is never increased",
             state.command_right_permille, 220u);

    car_follow_output_reset(&state);
    car_follow_output_hold_at(&state, 49u, UINT32_MAX - 100u, 9u,
                              600u, 500u);
    CHECK_EQ("hold ramp timer crosses uint32 wrap at 150 ms",
             state.command_left_permille, 500u);
    car_follow_output_hold_at(&state, 199u, UINT32_MAX - 100u, 9u,
                              0u, 0u);
    CHECK_EQ("hold ramp timer reaches 300 ms across uint32 wrap",
             state.command_left_permille, 400u);

    car_follow_output_reset(&state);
    CHECK_EQ("stop reset clears startup qualification", state.startup_qualified,
             0u);
    CHECK_EQ("stop reset clears both side commands", state.command_left_permille,
             0u);

    car_follow_output_reset(&state);
    CHECK_EQ("wrap fixture starts with completed 500/500 outputs",
             car_follow_output_update(&state, UINT32_MAX - 5u, 500u, 0u, 0u,
                                      1530u, 1530u, 1100u, 2500u,
                                      1u, 1u, 1u, 500u, 500u,
                                      &left, &right), 1u);
    CHECK_EQ("zero differential qualifies the base latch once",
             state.startup_qualified, 1u);
    CHECK_EQ("elapsed-time arithmetic crosses uint32 wrap exactly once",
             car_follow_output_update(&state, 4u, 500u, 100u, 100u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 1u);
    CHECK_EQ("wrapped ramp advances left by one step", left, 510u);
    CHECK_EQ("wrapped ramp advances right by one step", right, 490u);

    CHECK_EQ("normal helper rejects base requests above 500",
             car_follow_output_update(&state, 14u, 501u, 0u, 0u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 0u);
    CHECK_EQ("normal helper rejects differential above the implementation bound",
             car_follow_output_update(&state, 14u, 500u, 201u, 201u, 2500u,
                                      1530u, 1100u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 0u);
    CHECK_EQ("degenerate servo endpoint calibration is rejected",
             car_follow_output_update(&state, 14u, 500u, 0u, 0u, 1530u,
                                      1530u, 1530u, 2500u, 1u, 1u,
                                      1u, 500u, 500u, &left, &right), 0u);
    car_follow_slope_assist_gate_t old_profile_gate{};
    old_profile_gate.follow_running = 1u;
    old_profile_gate.manual_run_request = 1u;
    old_profile_gate.follow_session_active = 1u;
    old_profile_gate.road_drive_allowed = 1u;
    old_profile_gate.trusted_slope_hint = 1u;
    old_profile_gate.vision_valid = 1u;
    old_profile_gate.vision_fresh = 1u;
    old_profile_gate.startup_qualified = 1u;
    old_profile_gate.servo_output_active = 1u;
    old_profile_gate.servo_frame_ready = 1u;
    old_profile_gate.motor_outputs_active = 1u;
    old_profile_gate.applied_outputs_nonzero = 1u;
    CHECK_EQ("non-Turn profiles cannot request slope assist",
             car_follow_output_slope_assist_request(&old_profile_gate), 0u);
}

void test_runtime_diag_snapshot_abi_crc_and_schedule()
{
    const char crc_fixture[] = "123456789";
    car_runtime_diag_values_t values{};

    CHECK_EQ("runtime diagnostic uses CRC-32/ISO-HDLC",
             car_runtime_diag_crc32(crc_fixture, 9u), 0xCBF43926u);
    CHECK_EQ("runtime diagnostic generation wraps to nonzero even 2",
             car_runtime_diag_next_even_generation(0xFFFFFFFEu), 2u);

    car_runtime_diag_init(0xA55A1234u, 1u);
    CHECK_EQ("runtime diagnostic snapshot has fixed magic",
             g_runtime_diag_snapshot.magic, CAR_RUNTIME_DIAG_MAGIC);
    CHECK_EQ("runtime diagnostic snapshot has ABI version 1",
             g_runtime_diag_snapshot.version, CAR_RUNTIME_DIAG_VERSION);
    CHECK_EQ("runtime diagnostic snapshot has fixed 64-byte size",
             g_runtime_diag_snapshot.size, CAR_RUNTIME_DIAG_SIZE);
    CHECK_EQ("first published runtime diagnostic generation is nonzero even",
             g_runtime_diag_snapshot.generation, 2u);
    CHECK_EQ("runtime diagnostic records the captured reset flags",
             g_runtime_diag_snapshot.reset_flags, 0xA55A1234u);
    CHECK_EQ("runtime diagnostic initially reports logging enabled",
             g_runtime_diag_snapshot.flags,
             CAR_RUNTIME_DIAG_FLAG_LOG_ENABLED);
    CHECK_EQ("runtime diagnostic CRC covers the complete ABI image",
             g_runtime_diag_snapshot.crc32,
             car_runtime_diag_snapshot_crc32(&g_runtime_diag_snapshot));
    CHECK_EQ("runtime diagnostic starts with zeroed reserved bytes",
             g_runtime_diag_snapshot.reserved, 0u);

    values.start_events = 1u;
    values.stop_events = 0u;
    values.start_token = 1u;
    values.cpu_header_state = CAR_RUNTIME_LOG_STATE_ACTIVE;
    values.cpu_header_count = 1u;
    values.uptime_ms = 25u;
    values.reset_flags = 0xA55A1234u;
    values.header_session_id = 1u;
    values.left_output_permille = 499u;
    values.right_output_permille = 501u;
    values.servo_pulse_us = 1530u;
    values.log_enabled = 1u;
    car_runtime_diag_note_log_update_call();
    CHECK_EQ("PE7 and logger header change publishes immediately",
             car_runtime_diag_update(&values), 1u);
    CHECK_EQ("published snapshot advances by two generations",
             g_runtime_diag_snapshot.generation, 4u);
    CHECK_EQ("snapshot includes the actual logger update call count",
             g_runtime_diag_snapshot.log_update_calls, 1u);
    CHECK_EQ("snapshot preserves distinct logger start token and session id",
             g_runtime_diag_snapshot.start_token, 1u);
    CHECK_EQ("snapshot mirrors the runtime-log session id",
             g_runtime_diag_snapshot.header_session_id, 1u);
    CHECK_EQ("snapshot stores applied left output as signed 16-bit",
             g_runtime_diag_snapshot.left_output_permille, 499);
    CHECK_EQ("snapshot stores software servo compare pulse",
             g_runtime_diag_snapshot.servo_pulse_us, 1530u);
    CHECK_EQ("published active snapshot has valid CRC",
             g_runtime_diag_snapshot.crc32,
             car_runtime_diag_snapshot_crc32(&g_runtime_diag_snapshot));

    values.uptime_ms = 124u;
    CHECK_EQ("periodic snapshot is bounded below 100 ms",
             car_runtime_diag_update(&values), 0u);
    values.uptime_ms = 125u;
    CHECK_EQ("periodic snapshot publishes at 100 ms",
             car_runtime_diag_update(&values), 1u);
    CHECK_EQ("periodic snapshot has next even generation",
             g_runtime_diag_snapshot.generation, 6u);

    values.uptime_ms = 126u;
    values.cpu_header_state = CAR_RUNTIME_LOG_STATE_FROZEN;
    values.cpu_header_count = 2u;
    car_runtime_diag_note_log_update_call();
    CHECK_EQ("final frozen header state/count publishes in the same loop",
             car_runtime_diag_update(&values), 1u);
    CHECK_EQ("frozen snapshot contains current logger call total",
             g_runtime_diag_snapshot.log_update_calls, 2u);
    CHECK_EQ("frozen CPU header state uses raw enum value 2",
             g_runtime_diag_snapshot.cpu_header_state,
             CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("frozen snapshot contains final header count",
             g_runtime_diag_snapshot.cpu_header_count, 2u);

    car_runtime_diag_init(0u, 0u);
    values = {};
    values.uptime_ms = 0xFFFFFFCEu;
    CHECK_EQ("disabled logger still publishes an independent idle snapshot",
             car_runtime_diag_update(&values), 1u);
    values.start_events = 2u;
    CHECK_EQ("disabled logger snapshot publishes PE7 event changes",
             car_runtime_diag_update(&values), 1u);
    CHECK_EQ("disabled logger flag is clear in the snapshot",
             g_runtime_diag_snapshot.flags, 0u);
    values.start_events = 2u;
    values.uptime_ms = 50u;
    CHECK_EQ("100 ms periodic update remains wrap-safe",
             car_runtime_diag_update(&values), 1u);
    CHECK_EQ("disabled snapshot still has a valid CRC",
             g_runtime_diag_snapshot.crc32,
             car_runtime_diag_snapshot_crc32(&g_runtime_diag_snapshot));

    {
        car_runtime_diag_snapshot_t corrupted = g_runtime_diag_snapshot;
        ++corrupted.generation;
        CHECK(corrupted.crc32 !=
                  car_runtime_diag_snapshot_crc32(&corrupted),
              "generation is included in runtime snapshot CRC");
    }
}

void test_turn_slope_assist_gate_and_per_side_slew()
{
    car_follow_slope_assist_gate_t gate{};
    gate.follow_running = 1u;
    gate.manual_run_request = 1u;
    gate.follow_session_active = 1u;
    gate.road_drive_allowed = 1u;
    gate.trusted_slope_hint = 1u;
    gate.vision_valid = 1u;
    gate.vision_fresh = 1u;
    gate.startup_qualified = 1u;
    gate.servo_output_active = 1u;
    gate.servo_frame_ready = 1u;
    gate.motor_outputs_active = 1u;
    gate.applied_outputs_nonzero = 1u;
    gate.trusted_vision_error_norm = -100;
    CHECK_EQ("Turn slope gate accepts -100 normalized error",
             car_follow_output_turn_slope_assist_request(&gate), 1u);
    gate.trusted_vision_error_norm = 100;
    CHECK_EQ("Turn slope gate accepts +100 normalized error",
             car_follow_output_turn_slope_assist_request(&gate), 1u);
    gate.trusted_vision_error_norm = 101;
    CHECK_EQ("Turn slope gate rejects steering error above 100",
             car_follow_output_turn_slope_assist_request(&gate), 0u);
    gate.trusted_vision_error_norm = 0;

    uint8_t *required_gates[] = {
        &gate.follow_running, &gate.manual_run_request,
        &gate.follow_session_active, &gate.road_drive_allowed,
        &gate.trusted_slope_hint, &gate.vision_valid, &gate.vision_fresh,
        &gate.startup_qualified, &gate.servo_output_active,
        &gate.servo_frame_ready, &gate.motor_outputs_active,
        &gate.applied_outputs_nonzero
    };
    for (size_t index = 0u;
         index < sizeof(required_gates) / sizeof(required_gates[0]);
         ++index) {
        uint8_t *mutable_gate = required_gates[index];
        const uint8_t saved = *mutable_gate;
        *mutable_gate = 0u;
        CHECK_EQ("every independent startup/route/output gate is required",
                 car_follow_output_turn_slope_assist_request(&gate), 0u);
        *mutable_gate = saved;
    }
    gate.fault_or_stop = 1u;
    CHECK_EQ("fault or stop blocks Turn slope request",
             car_follow_output_turn_slope_assist_request(&gate), 0u);
    gate.fault_or_stop = 0u;

    car_follow_output_state_t state{};
    uint16_t left = 0u;
    uint16_t right = 0u;
    CHECK_EQ("pre-start slope request remains at zero output",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 0u, 500u, 200u, 200u, 1530u, 1530u, 1100u,
                 2500u, 1u, 1u, 1u, 0u, 0u, 1u, &left, &right), 1u);
    CHECK_EQ("slope request cannot skip 500/500 startup target",
             state.target_left_permille, 500u);
    CHECK_EQ("pre-start request is not marked commanded",
             state.slope_assist_active, 0u);
    CHECK_EQ("pre-start motor command remains zero", left, 0u);

    CHECK_EQ("completed actual 500/500 and full servo frame latch startup",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 10u, 500u, 200u, 200u, 1530u, 1530u, 1100u,
                 2500u, 1u, 1u, 1u, 500u, 500u, 0u, &left, &right), 1u);
    CHECK_EQ("startup qualifies only at actual straight 500/500",
             state.startup_qualified, 1u);

    CHECK_EQ("trusted straight slope request selects 700/700 targets",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 20u, 500u, 200u, 200u, 1530u, 1530u, 1100u,
                 2500u, 1u, 1u, 1u, 500u, 500u, 1u, &left, &right), 1u);
    CHECK_EQ("slope target is 700 left", state.target_left_permille, 700u);
    CHECK_EQ("slope target is 700 right", state.target_right_permille, 700u);
    CHECK_EQ("assist state describes the selected command", state.slope_assist_active, 1u);
    CHECK_EQ("assist enters through one 40-permille left step", left, 540u);
    CHECK_EQ("assist enters through one 40-permille right step", right, 540u);
    CHECK(state.target_left_permille <= 700u &&
              state.target_right_permille <= 700u,
          "both assisted side targets respect the fixed 700 hard ceiling");
    CHECK(state.target_left_permille <= CAR_FOLLOW_HARD_MAX_PERMILLE,
          "left slope target passes the fixed hard-bound check");
    CHECK(state.target_right_permille <= CAR_FOLLOW_HARD_MAX_PERMILLE,
          "right slope target passes the fixed hard-bound check");

    CHECK_EQ("long delay still advances uphill output by one step only",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 1000u, 500u, 200u, 200u, 1530u, 1530u,
                 1100u, 2500u, 1u, 1u, 1u, 540u, 540u, 1u,
                 &left, &right), 1u);
    CHECK_EQ("delayed assisted left output has no catch-up", left, 580u);
    CHECK_EQ("delayed assisted right output has no catch-up", right, 580u);

    CHECK_EQ("right turn above u=100 withdraws assist immediately",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 1010u, 500u, 200u, 200u, 1599u, 1530u,
                 1100u, 2500u, 1u, 1u, 1u, 700u, 700u, 1u,
                 &left, &right), 1u);
    CHECK_EQ("right turn restores the existing 70/30 differential outer target",
             state.target_left_permille, 520u);
    CHECK_EQ("right turn restores the existing 70/30 differential inner target",
             state.target_right_permille, 480u);
    CHECK_EQ("turn withdraws commanded-assist diagnostic immediately",
             state.slope_assist_active, 0u);
    CHECK_EQ("left motor decreases by no more than 40 on withdrawal", left, 660u);
    CHECK_EQ("right motor decreases by no more than 40 on withdrawal", right, 660u);

    CHECK_EQ("left turn above u=100 also withdraws assist",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 1020u, 500u, 200u, 200u, 1461u, 1530u,
                 1100u, 2500u, 1u, 1u, 1u, 700u, 700u, 1u,
                 &left, &right), 1u);
    CHECK_EQ("left turn mirrors lower inner duty target",
             state.target_left_permille, 455u);
    CHECK_EQ("left turn mirrors upper outer duty target",
             state.target_right_permille, 545u);
    CHECK_EQ("left-turn ramp remains per-side limited", left, 660u);
    CHECK_EQ("left-turn ramp remains per-side limited right", right, 660u);

    car_follow_output_reset(&state);
    car_follow_output_turn_hold_at(&state, 0u, 0u, 13u, 700u, 300u);
    CHECK_EQ("70/30 hold preserves the actual left entry output at time zero",
             state.command_left_permille, 700u);
    CHECK_EQ("70/30 hold preserves the actual right entry output at time zero",
             state.command_right_permille, 300u);
    car_follow_output_turn_hold_at(&state, 150u, 0u, 13u, 0u, 0u);
    CHECK_EQ("70/30 hold reaches 50/30 at 150 ms", state.command_left_permille,
             550u);
    CHECK_EQ("70/30 hold does not increase its 300-permille side",
             state.command_right_permille, 300u);
    car_follow_output_turn_hold_at(&state, 300u, 0u, 13u, 0u, 0u);
    CHECK_EQ("70/30 hold reaches its fixed 40/30 target at 300 ms",
             state.command_left_permille, 400u);
    CHECK_EQ("70/30 hold retains its 30 target at 300 ms",
             state.command_right_permille, 300u);
    state.startup_qualified = 1u;
    CHECK_EQ("same-tick recovery keeps the decelerated 40/30 output",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 300u, 500u, 200u, 200u, 1530u, 1530u, 1100u,
                 2500u, 1u, 1u, 1u, 400u, 300u, 0u, &left, &right), 1u);
    CHECK_EQ("same-tick recovery does not jump from the decelerated output",
             left, 400u);
    CHECK_EQ("first recovered left slew step remains at most 40 permille",
             car_follow_output_turn_update_with_slope_assist(
                 &state, 310u, 500u, 200u, 200u, 1530u, 1530u, 1100u,
                 2500u, 1u, 1u, 1u, 400u, 300u, 0u, &left, &right), 1u);
    CHECK_EQ("70/30 recovered left output advances by one existing step",
             left, 440u);
    CHECK_EQ("70/30 recovered right output advances by one existing step",
             right, 340u);

    car_follow_output_reset(&state);
    car_follow_output_turn_hold_at(&state, 0u, 0u, 14u, 500u, 500u);
    CHECK_EQ("50/50 hold begins at its actual entry outputs",
             state.command_left_permille, 500u);
    car_follow_output_turn_hold_at(&state, 150u, 0u, 14u, 0u, 0u);
    CHECK_EQ("50/50 hold reaches 45/45 at 150 ms",
             state.command_left_permille, 450u);
    CHECK_EQ("50/50 hold keeps both sides symmetrical at 150 ms",
             state.command_right_permille, 450u);
    car_follow_output_turn_hold_at(&state, 300u, 0u, 14u, 0u, 0u);
    CHECK_EQ("50/50 hold reaches its fixed 40/40 target at 300 ms",
             state.command_left_permille, 400u);
    CHECK_EQ("50/50 hold reaches its fixed right target at 300 ms",
             state.command_right_permille, 400u);
}

void test_servo_center_frame_slew_fault_and_sticky_snapshot()
{
    FakeServoRegisters fake = make_valid_fake_servo();
    const car_servo_safety_io_t io = fake_servo_io(&fake);
    car_servo_safety_t servo{};
    car_servo_safety_init(&servo);
    CHECK_EQ("first servo request starts guarded output",
             car_servo_safety_request(&servo, 2500u, 100u, &io), 1u);
    CHECK_EQ("first applied request is always measured centre",
             servo.compare_us, CAR_SERVO_SAFETY_CENTER_US);
    CHECK_EQ("start begins from centre even for an endpoint request",
             fake.snapshot.tim_ccr1, CAR_SERVO_SAFETY_CENTER_US);
    CHECK_EQ("startup marks the ARR-seed wrap as unqualified",
             servo.discard_first_update, 1u);
    CHECK_EQ("software UG does not mark a real PWM frame ready",
             servo.frame_ready, 0u);
    CHECK_EQ("first nonzero write count is exactly the centre pulse",
             fake.write_count, 1u);

    fake.update_pending = 1u;
    CHECK_EQ("the first ARR-seed wrap is consumed but does not unlock motor",
             car_servo_safety_request(&servo, 2500u, 101u, &io), 1u);
    CHECK_EQ("startup wrap leaves motor readiness closed", servo.frame_ready, 0u);
    CHECK_EQ("startup wrap qualification clears once", servo.discard_first_update, 0u);
    CHECK_EQ("elapsed time alone cannot fake a completed PWM frame",
             car_servo_safety_request(&servo, 2500u, 119u, &io), 1u);
    CHECK_EQ("UIF absence keeps motor readiness closed", servo.frame_ready, 0u);
    fake.update_pending = 1u;
    CHECK_EQ("the full 20 ms update qualifies the first complete pulse",
             car_servo_safety_request(&servo, 2500u, 120u, &io), 1u);
    CHECK_EQ("real update advances frame readiness", servo.frame_ready, 1u);
    CHECK_EQ("first slew step follows a full PWM period",
             servo.limited_us, 1540u);
    CHECK_EQ("target change before next interval succeeds",
             car_servo_safety_request(&servo, 1100u, 122u, &io), 1u);
    CHECK_EQ("target change does not reset completed-frame readiness",
             servo.frame_ready, 1u);
    CHECK_EQ("zero elapsed time does not slew", servo.limited_us, 1540u);
    CHECK_EQ("one interval moves one step toward new target",
             car_servo_safety_request(&servo, 1100u, 141u, &io), 1u);
    CHECK_EQ("target change slews by ten microseconds", servo.limited_us, 1530u);
    CHECK_EQ("delayed loop advances by only one step",
             car_servo_safety_request(&servo, 1100u, 1000u, &io), 1u);
    CHECK_EQ("delayed-loop step is bounded", servo.limited_us, 1520u);
    CHECK_EQ("late iteration does not accumulate catch-up steps",
             car_servo_safety_request(&servo, 1100u, 1001u, &io), 1u);
    CHECK_EQ("no second step occurs before a fresh interval", servo.limited_us,
             1520u);

    FakeServoRegisters wrap_fake = make_valid_fake_servo();
    const car_servo_safety_io_t wrap_io = fake_servo_io(&wrap_fake);
    car_servo_safety_t wrap_servo{};
    car_servo_safety_init(&wrap_servo);
    CHECK_EQ("wrap fixture starts", car_servo_safety_request(
                 &wrap_servo, 2500u, 0xFFFFFFF0u, &wrap_io), 1u);
    CHECK_EQ("wrap-safe elapsed 19 ms does not slew",
             car_servo_safety_request(&wrap_servo, 2500u, 3u, &wrap_io), 1u);
    CHECK_EQ("wrap-safe 20 ms interval advances one step",
             car_servo_safety_request(&wrap_servo, 2500u, 4u, &wrap_io), 1u);
    CHECK_EQ("wrap-safe step is bounded", wrap_servo.limited_us, 1540u);

    for (uint16_t rejected : std::array<uint16_t, 2>{1099u, 2501u}) {
        FakeServoRegisters rejected_fake = make_valid_fake_servo();
        const car_servo_safety_io_t rejected_io =
            fake_servo_io(&rejected_fake);
        car_servo_safety_t rejected_state{};
        car_servo_safety_init(&rejected_state);
        CHECK_EQ("out-of-range request fails closed",
                 car_servo_safety_request(&rejected_state, rejected, 5u,
                                          &rejected_io), 0u);
        CHECK_EQ("range failure latches a hard safety fault",
                 rejected_state.fault_latched, 1u);
        CHECK_EQ("range failure preserves rejected request",
                 rejected_state.last_rejected_request_us, rejected);
        CHECK_EQ("range failure preserves pre-stop clock snapshot",
                 rejected_state.last_fault_snapshot.rcc_cr, 0x5u);
        CHECK_EQ("range failure leaves output stopped",
                 rejected_fake.snapshot.tim_ccr1, 0u);
        CHECK_EQ("range failure does not write a nonzero CCR",
                 rejected_fake.write_count, 0u);
    }

    FakeServoRegisters corrupt = make_valid_fake_servo();
    corrupt.snapshot.tim_smcr = 2u;
    car_servo_safety_t corrupt_state{};
    car_servo_safety_init(&corrupt_state);
    const car_servo_safety_io_t corrupt_io = fake_servo_io(&corrupt);
    CHECK_EQ("slave-clock mode fault stops request",
             car_servo_safety_request(&corrupt_state, 1530u, 0u,
                                      &corrupt_io), 0u);
    CHECK_EQ("sticky snapshot keeps SMCR before stop",
             corrupt_state.last_fault_snapshot.tim_smcr, 2u);
    CHECK_EQ("timer-mode fault latch persists after stop",
             corrupt_state.fault_latched, 1u);
    car_servo_safety_stop(&corrupt_state, &corrupt_io);
    CHECK_EQ("ordinary stop leaves sticky evidence intact",
             corrupt_state.sticky_fault_code, CAR_SERVO_FAULT_TIMER_CONFIG);
    car_servo_safety_acknowledge(&corrupt_state);
    CHECK_EQ("physical ACK clears latch only, not evidence",
             corrupt_state.fault_latched, 0u);
    CHECK_EQ("sticky fault code survives ACK",
             corrupt_state.sticky_fault_code, CAR_SERVO_FAULT_TIMER_CONFIG);

    FakeServoRegisters readback = make_valid_fake_servo();
    readback.corrupt_readback = 1u;
    car_servo_safety_t readback_state{};
    car_servo_safety_init(&readback_state);
    const car_servo_safety_io_t readback_io = fake_servo_io(&readback);
    CHECK_EQ("compare readback mismatch rejects output",
             car_servo_safety_request(&readback_state, 1530u, 0u,
                                      &readback_io), 0u);
    CHECK_EQ("compare readback mismatch has sticky fault code",
             readback_state.sticky_fault_code,
             CAR_SERVO_FAULT_COMPARE_WRITE);
}

void test_servo_stop_phase_and_pending_policy()
{
    constexpr uint32_t arr = 19999u;
    constexpr uint32_t period_ticks = arr + 1u;
    constexpr uint32_t centre_pulse = CAR_SERVO_SAFETY_CENTER_US;
    bool request_phase_sweep_passed = true;
    bool compare_phase_sweep_passed = true;

    for (uint32_t counter = 0u; counter <= arr; ++counter) {
        const uint8_t pin_high = counter < centre_pulse ? 1u : 0u;
        const bool edge_window =
            counter < CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US ||
            counter > arr - CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US;
        const auto request_action = car_servo_safety_stop_action(
            0u, 0u, pin_high, counter, arr);
        const auto request_expected =
            pin_high != 0u || edge_window
                ? CAR_SERVO_STOP_ACTION_WAIT
                : CAR_SERVO_STOP_ACTION_CLOSE_NOW;
        if (request_action != request_expected) {
            request_phase_sweep_passed = false;
        }

        const auto compare_action = car_servo_safety_stop_action(
            1u, 1u, pin_high, counter, arr);
        const auto compare_expected =
            pin_high != 0u || edge_window
                ? CAR_SERVO_STOP_ACTION_CLEAR_COMPARE_AND_WAIT
                : CAR_SERVO_STOP_ACTION_CLOSE_NOW;
        if (compare_action != compare_expected) {
            compare_phase_sweep_passed = false;
        }
    }
    CHECK(request_phase_sweep_passed,
          "all 20000 initial-stop phases preserve high and wrap-edge pulses");
    CHECK(compare_phase_sweep_passed,
          "all 20000 compare-IRQ phases reject high/stale-wrap truncation");

    CHECK_EQ("pending stop without a compare remains pending",
             car_servo_safety_stop_action(1u, 0u, 0u, 1600u, arr),
             CAR_SERVO_STOP_ACTION_WAIT);
    CHECK_EQ("stale compare during a new high pulse is cleared, not closed",
             car_servo_safety_stop_action(1u, 1u, 1u, 50u, arr),
             CAR_SERVO_STOP_ACTION_CLEAR_COMPARE_AND_WAIT);
    CHECK_EQ("low phase near ARR defers through wrap",
             car_servo_safety_stop_action(0u, 0u, 0u, arr - 1u, arr),
             CAR_SERVO_STOP_ACTION_WAIT);
    CHECK_EQ("low phase in the first wrap window defers",
             car_servo_safety_stop_action(0u, 0u, 0u, 1u, arr),
             CAR_SERVO_STOP_ACTION_WAIT);
    CHECK_EQ("upper guard boundary outside the final 128 ticks closes low",
             car_servo_safety_stop_action(
                 0u, 0u, 0u, arr - CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US, arr),
             CAR_SERVO_STOP_ACTION_CLOSE_NOW);
    CHECK_EQ("a normal low phase closes without waiting a whole frame",
             car_servo_safety_stop_action(0u, 0u, 0u, 1600u, arr),
             CAR_SERVO_STOP_ACTION_CLOSE_NOW);
    CHECK_EQ("pending compare at a normal falling edge closes",
             car_servo_safety_stop_action(1u, 1u, 0u, 1600u, arr),
             CAR_SERVO_STOP_ACTION_CLOSE_NOW);
    CHECK_EQ("invalid short ARR is treated as an edge window",
             car_servo_safety_stop_action(0u, 0u, 0u, 5u, 100u),
             CAR_SERVO_STOP_ACTION_WAIT);

    /* Discrete PWM timeline model: active compare is deliberately different
       from the queued preload. A wrap transfers that already-queued value;
       pending stop makes no additional compare write. Stop decisions call
       the production phase helper using the modeled PA8 IDR state. */
    constexpr std::array<uint32_t, 3> active_compares{1100u, 1530u, 2500u};
    constexpr std::array<uint32_t, 4> isr_delays{0u, 1u, 128u, 2500u};
    bool event_model_passed = true;
    uint32_t maximum_close_delay = 0u;
    for (uint32_t active_compare : active_compares) {
        for (uint32_t stop_phase = 0u; stop_phase <= arr; ++stop_phase) {
            for (uint32_t irq_delay : isr_delays) {
                const uint32_t preload_compare =
                    active_compare == 2500u ? 1100u : 2500u;
                uint32_t elapsed = 0u;
                uint32_t compare_absolute = 0u;
                bool closed = false;
                bool pending = true;
                uint32_t completed_pulses = 0u;
                bool pulse_widths_valid = true;
                const bool high = stop_phase < active_compare;
                const auto initial = car_servo_safety_stop_action(
                    0u, 0u, high ? 1u : 0u, stop_phase, arr);
                if (initial == CAR_SERVO_STOP_ACTION_CLOSE_NOW) {
                    closed = true;
                    pending = false;
                } else if (stop_phase < active_compare) {
                    compare_absolute = active_compare - stop_phase;
                } else {
                    compare_absolute = period_ticks - stop_phase +
                                       preload_compare;
                }

                for (uint32_t edge = 0u; !closed && edge < 3u; ++edge) {
                    const uint32_t service_absolute =
                        compare_absolute + irq_delay;
                    const uint32_t counter =
                        (stop_phase + service_absolute) % period_ticks;
                    const uint32_t active_at_service =
                        stop_phase + service_absolute >= period_ticks
                            ? preload_compare : active_compare;
                    const bool pin_high = counter < active_at_service;
                    elapsed = service_absolute;
                    const uint32_t pulse_width =
                        stop_phase < active_compare && completed_pulses == 0u
                            ? active_compare : preload_compare;
                    if (pulse_width < CAR_SERVO_SAFETY_MIN_US ||
                        pulse_width > CAR_SERVO_SAFETY_MAX_US ||
                        pulse_width != (stop_phase < active_compare &&
                                        completed_pulses == 0u
                                            ? active_compare
                                            : preload_compare)) {
                        pulse_widths_valid = false;
                    }
                    completed_pulses++;
                    const auto isr_action = car_servo_safety_stop_action(
                        1u, 1u, pin_high ? 1u : 0u, counter, arr);
                    if (isr_action == CAR_SERVO_STOP_ACTION_CLOSE_NOW) {
                        if (pin_high) {
                            pulse_widths_valid = false;
                        }
                        closed = true;
                        pending = false;
                        break;
                    }
                    if (counter < active_at_service) {
                        compare_absolute = service_absolute +
                                           active_at_service - counter;
                    } else {
                        compare_absolute = service_absolute + period_ticks -
                                           counter + preload_compare;
                    }
                }
                const uint32_t maximum_expected =
                    CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US +
                    (active_compare > preload_compare
                         ? active_compare : preload_compare) + irq_delay;
                if (!closed || pending ||
                    completed_pulses > 2u ||
                    !pulse_widths_valid || elapsed > maximum_expected) {
                    event_model_passed = false;
                }
                if (elapsed > maximum_close_delay) {
                    maximum_close_delay = elapsed;
                }
                if (preload_compare == active_compare) {
                    event_model_passed = false;
                }
            }
        }
    }
    CHECK(event_model_passed,
          "PWM event model preserves complete active and queued-preload pulses across every stop phase");
    CHECK(maximum_close_delay <= 128u + 2500u + 2500u,
          "stop completion stays within 128-tick wrap guard + queued pulse + modeled IRQ delay");
}

void test_servo_production_profile_selection_rejects_before_output()
{
    const std::array<std::pair<car_servo_source_t, uint16_t>, 4> cases{{
        {CAR_SERVO_SOURCE_DEBUG_STEERING, static_cast<uint16_t>(1099u)},
        {CAR_SERVO_SOURCE_DEBUG_STEERING, static_cast<uint16_t>(2501u)},
        {CAR_SERVO_SOURCE_FOLLOW, static_cast<uint16_t>(1099u)},
        {CAR_SERVO_SOURCE_FOLLOW, static_cast<uint16_t>(2501u)}
    }};

    for (const auto &entry : cases) {
        FakeServoRegisters fake = make_valid_fake_servo();
        const car_servo_safety_io_t io = fake_servo_io(&fake);
        car_servo_safety_t servo{};
        car_servo_safety_init(&servo);
        const uint16_t follow_target =
            entry.first == CAR_SERVO_SOURCE_FOLLOW ? entry.second : 1530u;
        const uint16_t debug_target =
            entry.first == CAR_SERVO_SOURCE_DEBUG_STEERING
                ? entry.second : 1530u;

        CHECK_EQ("selected production profile rejects invalid pulse even when output gate is closed",
                 car_servo_safety_request_selected(
                     &servo, entry.first, follow_target, debug_target,
                     0u, 10u, &io), 0u);
        CHECK_EQ("selected profile range violation latches hard fault",
                 servo.fault_latched, 1u);
        CHECK_EQ("selected profile preserves rejected request",
                 servo.last_rejected_request_us, entry.second);
        CHECK_EQ("selected profile preserves pre-stop TIM1 state",
                 servo.last_fault_snapshot.tim_psc, 63u);
        CHECK_EQ("selected profile emits no nonzero CCR write",
                 fake.write_count, 0u);
        CHECK_EQ("selected profile fail-closed compare is zero",
                 fake.snapshot.tim_ccr1, 0u);
    }
}

void test_motor_hardware_acceptance_default_lock()
{
    car_config_t bench_config = make_config(true, true);
    bench_config.motor_hardware_accepted = 0u;
    car_control_t bench{};
    car_control_init(&bench, &bench_config, 0u, 1u, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(&bench, &bench_config, &released);
    CHECK_EQ("production-default Bench acceptance is locked",
             bench_config.motor_hardware_accepted, 0u);
    CHECK_EQ("locked Bench does not begin in a fault loop",
             bench.fault_latched, 0u);

    car_control_rx_recovery_flush_complete(&bench);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame_at(&bench, &bench_config, sequence, true, 20u, 0,
                      sequence, sequence, true);
    }
    (void)select_bench(&bench, &bench_config);
    car_inputs_t press_edge = make_debounced_inputs(4u, false, true, false);
    car_control_tick(&bench, &bench_config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(24u, false, false, true);
    car_control_tick(&bench, &bench_config, &press_stable);
    CHECK(bench.mode != CAR_MODE_BENCH_RUNNING,
          "Bench acceptance lock cannot be bypassed by PE7");
    CHECK_EQ("locked Bench keeps commanded road permission clear",
             bench.road_drive_allowed, 0u);
    CHECK_EQ("locked Bench keeps software duty zero",
             bench.bench_output_duty_permille, 0u);
    CHECK_EQ("locked Bench does not repeatedly raise CONFIG",
             bench.fault, CAR_FAULT_NONE);

    car_config_t follow_config = make_follow_config();
    follow_config.motor_hardware_accepted = 0u;
    car_control_t follow{};
    car_control_init(&follow, &follow_config, 0u, 1u, 0u);
    car_control_tick(&follow, &follow_config, &released);
    car_control_rx_recovery_flush_complete(&follow);
    car_control_reverse_control_keep_started(&follow);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame_at(&follow, &follow_config, sequence, true, 20u, 0,
                      sequence, sequence, true);
    }
    car_inputs_t follow_press_edge =
        make_debounced_inputs(4u, false, true, false);
    car_control_tick(&follow, &follow_config, &follow_press_edge);
    car_inputs_t follow_press_stable =
        make_debounced_inputs(24u, false, false, true);
    car_control_tick(&follow, &follow_config, &follow_press_stable);
    CHECK(follow.mode != CAR_MODE_FOLLOW_RUNNING,
          "Follow acceptance lock blocks start after PE7 and fresh vision");
    CHECK_EQ("locked Follow keeps drive permission clear",
             follow.road_drive_allowed, 0u);
    CHECK_EQ("locked Follow keeps commanded duty zero",
             follow.follow_output_duty_permille, 0u);
    CHECK_EQ("locked Follow does not repeatedly raise CONFIG",
             follow.fault, CAR_FAULT_NONE);
}

void test_protocol_validation_and_crc()
{
    const uint8_t known[] = {'1','2','3','4','5','6','7','8','9'};
    CHECK_EQ("CRC16 CCITT-FALSE standard check", 
             vp_crc16_ccitt_false(known, 9u), 0x29B1u);

    uint8_t output[VP_FRAME_SIZE]{};
    vp_frame frame = make_wire_frame(2u);
    frame.flags = static_cast<uint8_t>(VP_FLAG_VALID | VP_FLAG_SLOPE_HINT);
    CHECK_EQ("valid slope hint remains wire-valid", vp_pack(&frame, output), 1u);
    CHECK_EQ("slope hint round-trips with valid bit",
             vp_unpack(output, &frame), 1u);
    CHECK_EQ("slope hint bit is retained by parser",
             frame.flags,
             static_cast<uint8_t>(VP_FLAG_VALID | VP_FLAG_SLOPE_HINT));
    frame = make_wire_frame(2u, true, 80u, 0);
    frame.flags = 0x04u;
    CHECK_EQ("unknown higher flag bit rejected", vp_pack(&frame, output), 0u);
    frame = make_wire_frame(2u, false, 0u, 0);
    frame.flags = VP_FLAG_SLOPE_HINT;
    CHECK_EQ("slope hint on invalid observation rejected",
             vp_pack(&frame, output), 0u);
    frame = make_wire_frame(2u, true, 101u, 0);
    CHECK_EQ("confidence above protocol maximum rejected", vp_pack(&frame, output), 0u);
    frame = make_wire_frame(2u, true, 80u, 1001);
    CHECK_EQ("error above protocol maximum rejected", vp_pack(&frame, output), 0u);
    frame = make_wire_frame(2u, true, 80u, -1001);
    CHECK_EQ("error below protocol minimum rejected", vp_pack(&frame, output), 0u);
    frame = make_wire_frame(2u, false, 1u, 0);
    CHECK_EQ("invalid frame must zero confidence rejected", vp_pack(&frame, output), 0u);

    CHECK_EQ("all-zero invalid vector remains protocol-valid",
             vp_unpack(g_vectors[5].bytes.data(), &frame), 1u);
    std::array<uint8_t, VP_FRAME_SIZE> corrupt = g_vectors[9].bytes;
    corrupt[10] ^= 0x01u;
    CHECK_EQ("CRC-corrupted slope hint frame rejected",
             vp_unpack(corrupt.data(), &frame), 0u);
    corrupt = g_vectors[0].bytes;
    corrupt[2] = static_cast<uint8_t>(VP_VERSION + 1u);
    CHECK_EQ("unsupported protocol version rejected", vp_unpack(corrupt.data(), &frame), 0u);
}

void test_parser_split_concatenation_and_resync()
{
    vp_parser parser{};
    vp_frame decoded{};
    vp_parser_init(&parser);
    unsigned produced = 0u;
    const auto &first = g_vectors[0].bytes;
    for (size_t i = 0; i < 5u; ++i) {
        produced += vp_parser_feed(&parser, first[i], &decoded);
    }
    CHECK_EQ("parser retains incomplete fragment", parser.length, 5u);
    for (size_t i = 5u; i < first.size(); ++i) {
        produced += vp_parser_feed(&parser, first[i], &decoded);
    }
    CHECK_EQ("split frame yields one result", produced, 1u);
    CHECK_EQ("split frame sequence", decoded.sequence, g_vectors[0].sequence);

    vp_parser_init(&parser);
    produced = 0u;
    for (const WireVector &vector : g_vectors) {
        for (uint8_t byte : vector.bytes) {
            produced += vp_parser_feed(&parser, byte, &decoded);
        }
    }
    CHECK_EQ("nine adjacent frames all parsed", produced, g_vectors.size());
    CHECK_EQ("parser ends between frames", parser.length, 0u);

    vp_parser_init(&parser);
    produced = 0u;
    const uint8_t noise[] = {0x00u, 0xA5u, 0x00u, 0x5Au, 0x33u};
    for (uint8_t byte : noise) {
        produced += vp_parser_feed(&parser, byte, &decoded);
    }
    for (uint8_t byte : g_vectors[1].bytes) {
        produced += vp_parser_feed(&parser, byte, &decoded);
    }
    CHECK_EQ("garbage prefix resynchronizes", produced, 1u);
    CHECK_EQ("resync returns following sequence", decoded.sequence, g_vectors[1].sequence);

    vp_parser_init(&parser);
    produced = 0u;
    std::array<uint8_t, VP_FRAME_SIZE> bad = g_vectors[0].bytes;
    bad[11] ^= 0x80u;
    for (uint8_t byte : bad) {
        produced += vp_parser_feed(&parser, byte, &decoded);
    }
    for (uint8_t byte : g_vectors[2].bytes) {
        produced += vp_parser_feed(&parser, byte, &decoded);
    }
    CHECK_EQ("bad CRC is skipped and next frame recovered", produced, 1u);
    CHECK_EQ("recovered frame after CRC failure", decoded.sequence, g_vectors[2].sequence);

    vp_parser_init(&parser);
    produced = 0u;
    for (uint8_t byte : g_vectors[8].bytes) {
        produced += vp_parser_feed(&parser, byte, &decoded);
    }
    CHECK_EQ("sync bytes embedded in sequence do not break parsing", produced, 1u);
    CHECK_EQ("embedded sync sequence decoded", decoded.sequence, 0x5AA5u);
}

void test_default_boot_and_switch_do_not_drive()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_control_init(&control, &config, 0u, 0u, 0u);
    CHECK_EQ("boot with PE7 low waits for release", control.mode,
             CAR_MODE_WAIT_SWITCH_RELEASE);
    CHECK_EQ("road driving compiled state remains disabled", control.road_drive_allowed, 0u);

    car_inputs_t low = make_inputs(1u, false);
    car_control_tick(&control, &config, &low);
    send_frame(&control, &config, 1u, true, 90u, 0, 2u, false);
    CHECK_EQ("valid vision alone cannot leave safe boot", control.mode,
             CAR_MODE_WAIT_SWITCH_RELEASE);
    CHECK_EQ("no bench PWM in default config", control.bench_output_duty_permille, 0u);

    const uint32_t low_select = car_control_command(
        &control, &config, &low, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 100u);
    CHECK((low_select & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "bench selection rejected with PE7 low");
    CHECK_EQ("bench mode is not compiled in default profile",
             control.bench_output_duty_permille, 0u);

    car_inputs_t high = make_inputs(3u, true);
    car_control_tick(&control, &config, &high);
    const uint32_t disabled_select = car_control_command(
        &control, &config, &high, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 100u);
    CHECK((disabled_select & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "bench selection rejected when bench code is not compiled");
    CHECK_EQ("default profile never authorizes road drive", control.road_drive_allowed, 0u);
    CHECK_EQ("all default motor outputs remain zero", control.bench_output_duty_permille, 0u);
}

void test_manual_run_button_toggle_lifecycle()
{
    car_manual_button_t button{};
    car_manual_button_init(&button, 0u, 0u);
    CHECK_EQ("power-up held-low is not armed", button.armed, 0u);
    CHECK_EQ("power-up held-low is not a start request",
             button.run_request, 0u);
    CHECK_EQ("power-up held-low has not confirmed release",
             button.startup_release_confirmed, 0u);

    car_manual_button_poll_high(&button, 1u, 10u);
    car_manual_button_poll_high(&button, 1u, 29u);
    CHECK_EQ("19 ms high is below the rearm boundary", button.armed, 0u);
    car_manual_button_poll_high(&button, 1u, 30u);
    CHECK_EQ("20 ms high arms the first press", button.armed, 1u);
    CHECK_EQ("20 ms high confirms startup release",
             button.startup_release_confirmed, 1u);

    CHECK_EQ("first low toggles the manual request on",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_START);
    CHECK_EQ("start event latches request", button.run_request, 1u);
    CHECK_EQ("start event is separately consumable", button.start_pending, 1u);
    CHECK_EQ("long low cannot retrigger the same press",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_NONE);
    car_manual_button_start_consumed(&button);
    car_manual_button_poll_high(&button, 1u, 31u);
    car_manual_button_poll_high(&button, 1u, 50u);
    CHECK_EQ("release does not clear the run request", button.run_request, 1u);
    CHECK_EQ("19 ms release does not rearm a second press", button.armed, 0u);
    car_manual_button_poll_high(&button, 1u, 51u);
    CHECK_EQ("20 ms release rearms the next press", button.armed, 1u);

    CHECK_EQ("second low toggles manual request off",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_STOP);
    CHECK_EQ("stop immediately revokes run request", button.run_request, 0u);
    CHECK_EQ("stop is pending for main-loop state conversion",
             button.stop_pending, 1u);
    CHECK_EQ("stop generation advances exactly once",
             button.stop_generation, 1u);
    car_manual_button_poll_high(&button, 1u, 52u);
    car_manual_button_poll_high(&button, 1u, 72u);
    CHECK_EQ("low before stop consumption cannot start",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_STOP);
    CHECK_EQ("stop priority leaves request off", button.run_request, 0u);
    car_manual_button_stop_consumed(&button);
    car_manual_button_poll_high(&button, 1u, 73u);
    car_manual_button_poll_high(&button, 1u, 93u);
    CHECK_EQ("new low starts only after stop consumed and rearmed",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_START);
    car_manual_button_t faulted_button{};
    car_manual_button_init(&faulted_button, 1u, 0u);
    car_manual_button_poll_high(&faulted_button, 1u, 20u);
    CHECK_EQ("hard fault blocks a new request",
             car_manual_button_note_low_edge(&faulted_button, 0u, 1u),
             CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT);
    CHECK_EQ("hard fault leaves manual request off",
             faulted_button.run_request, 0u);
}

void test_bench_channel_mask_mapping()
{
    car_control_t control{};

    control.bench_motor = CAR_MOTOR_LEFT;
    control.bench_input = CAR_MOTOR_IN1;
    CHECK_EQ("left IN1 maps to logical TIM3 CH1", 
             car_control_bench_channel_mask(&control), 0x01u);
    control.bench_input = CAR_MOTOR_IN2;
    CHECK_EQ("left IN2 maps to logical TIM3 CH2",
             car_control_bench_channel_mask(&control), 0x02u);
    control.bench_motor = CAR_MOTOR_RIGHT;
    control.bench_input = CAR_MOTOR_IN1;
    CHECK_EQ("right IN1 maps to logical TIM3 CH3",
             car_control_bench_channel_mask(&control), 0x04u);
    control.bench_input = CAR_MOTOR_IN2;
    CHECK_EQ("right IN2 maps to logical TIM3 CH4",
             car_control_bench_channel_mask(&control), 0x08u);
    control.bench_motor = CAR_MOTOR_BOTH_FORWARD;
    control.bench_input = CAR_MOTOR_IN1;
    CHECK_EQ("both-forward selects only left IN2 and right IN1",
             car_control_bench_channel_mask(&control), 0x06u);
    control.bench_input = CAR_MOTOR_IN2;
    CHECK_EQ("both-forward rejects an ambiguous IN2 marker",
             car_control_bench_channel_mask(&control), 0u);
    control.bench_motor = 4u;
    control.bench_input = CAR_MOTOR_IN1;
    CHECK_EQ("unknown motor enum maps to no output",
             car_control_bench_channel_mask(&control), 0u);
    CHECK_EQ("null selection maps to no output",
             car_control_bench_channel_mask(nullptr), 0u);
}

void test_select_ack_flushes_old_rx_before_new_frame()
{
    const car_config_t config = make_config(true, false);
    car_control_t control{};
    car_vision_rx_t rx{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    enqueue_frame(&rx, make_wire_frame(40u), 1u);
    CHECK_EQ("old complete frame queued before ack", rx.head, VP_FRAME_SIZE);

    const uint32_t effects = select_bench(&control, &config,
                                           CAR_MOTOR_RIGHT, CAR_MOTOR_IN2, 300u);
    CHECK((effects & CAR_EFFECT_CLEAR_RX) != 0u, "selection acts as local raised-car ack");
    CHECK((effects & CAR_EFFECT_RESET_PARSER) != 0u, "selection resets protocol parser");
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u, "ack keeps outputs stopped");
    CHECK_EQ("ack waits in the stopped three-frame recovery state",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("selected motor is retained for bench only", control.bench_motor, CAR_MOTOR_RIGHT);
    car_vision_rx_clear(&rx);
    car_control_rx_recovery_flush_complete(&control);
    CHECK_EQ("old ring queue is empty after clear", rx.tail, rx.head);
    CHECK_EQ("parser reset after clear", rx.parser.length, 0u);

    for (uint16_t sequence = 41u; sequence <= 43u; ++sequence) {
        enqueue_frame(&rx, make_wire_frame(sequence), sequence - 39u);
        poll_rx_current_snapshot(&rx, &control, &config, sequence - 39u, true);
    }
    CHECK_EQ("only post-ack frames counted", control.rx_frame_count, 3u);
    CHECK_EQ("third post-ack sequence trusted", control.vision_sequence, 43u);
    CHECK_EQ("fresh frame unlocks start-edge wait only", control.mode,
             CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("fresh frame does not start motor", control.bench_output_duty_permille, 0u);
}

void test_switch_edge_must_follow_new_frame()
{
    const car_config_t config = make_config(true, false);
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_inputs_t initial_high = make_inputs(0u, true);
    car_control_tick(&control, &config, &initial_high);
    (void)select_bench(&control, &config);
    car_control_rx_recovery_flush_complete(&control);

    car_inputs_t low_edge_early = make_debounced_inputs(1u, false, true, false);
    car_control_tick(&control, &config, &low_edge_early);
    car_inputs_t low_stable_early = make_debounced_inputs(21u, false, false, true);
    car_control_tick(&control, &config, &low_stable_early);
    CHECK_EQ("switch low before recovered observations does not start", control.mode,
             CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("switch low before frame leaves output zero",
             control.bench_output_duty_permille, 0u);

    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = 21u + sequence;
        send_frame_at(&control, &config, sequence, true, 80u, 5,
                      now, now, false);
    }
    CHECK_EQ("three frames recover observations but not the earlier press",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("recovered frames still keep motor off",
             control.bench_output_duty_permille, 0u);

    car_inputs_t release_edge = make_debounced_inputs(25u, true, false, true);
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t release_stable = make_debounced_inputs(45u, true, true, false);
    car_control_tick(&control, &config, &release_stable);
    CHECK_EQ("stable release arms an independent later press", control.mode,
             CAR_MODE_WAIT_START_EDGE);
    car_inputs_t press_edge = make_debounced_inputs(46u, false, true, false);
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(66u, false, false, true);
    car_control_tick(&control, &config, &press_stable);
    CHECK_EQ("new debounced low edge after recovery starts bench sequence", control.mode,
             CAR_MODE_BENCH_RUNNING);
    CHECK_EQ("first start tick has zero duty before ramp", control.bench_output_duty_permille, 0u);
    CHECK_EQ("stage one road-drive path remains disabled", control.road_drive_allowed, 0u);
}

void test_bench_ramp_cap_duration_and_stop()
{
    car_config_t config = make_config(true, false);
    config.bench_ramp_step_permille = 10u;
    config.bench_run_limit_ms = 500u;
    car_control_t control{};
    start_bench_with_fresh_frame(&control, &config, 1u);
    CHECK_EQ("independent start reaches bench-running state", control.mode,
             CAR_MODE_BENCH_RUNNING);
    CHECK_EQ("bench begins from zero duty", control.bench_output_duty_permille, 0u);

    for (uint32_t step = 1u; step <= 50u; ++step) {
        const uint32_t now = 24u + step * 10u;
        if (now == 204u) {
            send_frame_at(&control, &config, 4u, true, 80u, 0,
                          now, now, false);
        } else if (now == 404u) {
            send_frame_at(&control, &config, 5u, true, 80u, 0,
                          now, now, false);
        }
        car_inputs_t low = make_inputs(now, false);
        car_control_tick(&control, &config, &low);
        if (step < 50u) {
            const uint16_t expected =
                (uint16_t)(step < 40u ? step * 10u : 400u);
            CHECK_EQ("10ms ramp reaches 40 percent at 400ms",
                     control.bench_output_duty_permille, expected);
        }
        CHECK(control.bench_output_duty_permille <= 400u,
              "bench duty never exceeds 40 percent");
        CHECK_EQ("road drive remains disabled while bench pulses", control.road_drive_allowed, 0u);
        if (step == 40u) {
            CHECK_EQ("paired channels share the 40 percent duty command",
                     control.bench_output_duty_permille, 400u);
            CHECK_EQ("paired selection remains left IN2 plus right IN1",
                     car_control_bench_channel_mask(&control), 0x06u);
        }
    }

    car_inputs_t expired = make_inputs(524u, false);
    CHECK_EQ("500ms total bench duration ends at its deadline",
             control.mode, CAR_MODE_BENCH_COMPLETE);
    CHECK_EQ("run limit stops PWM", control.bench_output_duty_permille, 0u);
    /* Bench completion stops the one-shot; it retains the last observation for diagnostics. */
    CHECK_EQ("completed bench pulse retains diagnostic trusted observation",
             control.have_valid_vision, 1u);
    car_control_tick(&control, &config, &expired);
    CHECK_EQ("completed pulse cannot restart automatically", control.mode, CAR_MODE_BENCH_COMPLETE);

    const uint32_t stop_effects = car_control_command(
        &control, &config, &expired, CAR_CMD_NORMAL_STOP,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((stop_effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "normal stop directly requests output stop");
    CHECK_EQ("normal stop requires a new stopped recovery batch",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("normal stop requires the atomic RX flush before recovery",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("normal stop never enables road mode", control.road_drive_allowed, 0u);
}

void test_bench_auto_select_ready_and_one_shot_start()
{
    car_config_t config = make_config(true, false);
    config.bench_ramp_step_permille = 10u;
    config.bench_run_limit_ms = 500u;

    car_control_t boot_pressed{};
    car_control_init(&boot_pressed, &config, 0u, 0u, 0u);
    car_inputs_t pressed_at_boot = make_inputs(1u, false);
    uint32_t effects = car_control_auto_select_bench_output(
        &boot_pressed, &config, &pressed_at_boot, 1u);
    CHECK_EQ("boot-held PE7 does not auto-select Bench", effects,
             CAR_EFFECT_NONE);
    CHECK_EQ("boot-held PE7 leaves Bench unissued",
             boot_pressed.profile_command_issued, 0u);

    car_control_t no_vision{};
    car_control_init(&no_vision, &config, 0u, 1u, 0u);
    car_control_rx_recovery_flush_complete(&no_vision);
    car_inputs_t released_no_vision = make_inputs(1u, true);
    effects = car_control_auto_select_bench_output(
        &no_vision, &config, &released_no_vision, 1u);
    CHECK_EQ("missing fresh vision does not auto-select Bench", effects,
             CAR_EFFECT_NONE);
    CHECK_EQ("missing vision leaves Bench unissued",
             no_vision.profile_command_issued, 0u);

    car_control_t control{};
    establish_fresh_vision(&control, &config, 104u);
    car_inputs_t released = make_inputs(105u, true);
    car_control_tick(&control, &config, &released);
    CHECK_EQ("three boot frames leave the controller waiting for a new edge",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("Bench is not ready before its one-time SELECT",
             car_control_bench_ready(&control, &config, &released, 1u), 0u);
    effects = car_control_auto_select_bench_output(
        &control, &config, &released, 0u);
    CHECK_EQ("unhealthy RX path blocks automatic SELECT", effects,
             CAR_EFFECT_NONE);
    effects = car_control_auto_select_bench_output(
        &control, &config, &released, 1u);
    CHECK((effects & CAR_EFFECT_CLEAR_RX) != 0u,
          "automatic SELECT establishes a clean RX boundary");
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "automatic SELECT leaves physical outputs stopped");
    CHECK_EQ("automatic SELECT records its one-shot state",
             control.profile_command_issued, 1u);
    CHECK_EQ("automatic target is the dedicated both-forward selection",
             control.bench_motor, CAR_MOTOR_BOTH_FORWARD);
    CHECK_EQ("both-forward uses IN1 only as its legal marker",
             control.bench_input, CAR_MOTOR_IN1);
    CHECK_EQ("automatic pair maps to left IN2 and right IN1",
             car_control_bench_channel_mask(&control), 0x06u);
    CHECK_EQ("automatic target requests 40 percent",
             control.bench_requested_duty_permille, 400u);
    CHECK_EQ("automatic SELECT never starts the motor",
             control.bench_output_duty_permille, 0u);
    CHECK_EQ("automatic SELECT never grants road drive",
             control.road_drive_allowed, 0u);
    CHECK_EQ("post-SELECT recovery keeps ready low",
             car_control_bench_ready(&control, &config, &released, 1u), 0u);

    car_control_rx_recovery_flush_complete(&control);
    car_inputs_t early_press_edge =
        make_debounced_inputs(106u, false, true, false);
    car_control_tick(&control, &config, &early_press_edge);
    car_inputs_t early_press_held =
        make_debounced_inputs(126u, false, false, true);
    car_control_tick(&control, &config, &early_press_held);
    for (uint16_t sequence = 4u; sequence <= 6u; ++sequence) {
        const uint32_t now = 130u + (uint32_t)(sequence - 4u) * 10u;
        send_frame_at(&control, &config, sequence, true, 80u, 0,
                      now, now, false);
    }
    CHECK_EQ("second fresh batch completes auto-SELECT recovery",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    car_inputs_t held_after_recovery = make_inputs(151u, false);
    car_control_tick(&control, &config, &held_after_recovery);
    CHECK_EQ("a press held across recovery cannot start Bench",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("a held press keeps ready low",
             car_control_bench_ready(&control, &config,
                                     &held_after_recovery, 1u), 0u);
    CHECK_EQ("held press still cannot request automatic re-SELECT",
             car_control_auto_select_bench_output(
                 &control, &config, &held_after_recovery, 1u),
             CAR_EFFECT_NONE);

    car_inputs_t release_edge =
        make_debounced_inputs(152u, true, false, true);
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t released_ready =
        make_debounced_inputs(172u, true, true, false);
    car_control_tick(&control, &config, &released_ready);
    CHECK_EQ("released PE7 and fresh recovery frames expose ready",
             car_control_bench_ready(&control, &config,
                                     &released_ready, 1u), 1u);
    CHECK_EQ("ready remains low if the RX path is unhealthy",
             car_control_bench_ready(&control, &config,
                                     &released_ready, 0u), 0u);

    control.bench_input = CAR_MOTOR_IN2;
    CHECK_EQ("ready rejects an illegal paired IN2 marker",
             car_control_bench_ready(&control, &config,
                                     &released_ready, 1u), 0u);
    control.bench_input = CAR_MOTOR_IN1;
    control.bench_motor = CAR_MOTOR_LEFT;
    CHECK_EQ("ready rejects a single-wheel selection for the paired profile",
             car_control_bench_ready(&control, &config,
                                     &released_ready, 1u), 0u);
    control.bench_motor = CAR_MOTOR_BOTH_FORWARD;
    control.bench_requested_duty_permille = 399u;
    CHECK_EQ("ready requires the exact 40 percent request",
             car_control_bench_ready(&control, &config,
                                     &released_ready, 1u), 0u);
    control.bench_requested_duty_permille = 400u;

    car_inputs_t new_press_edge =
        make_debounced_inputs(173u, false, true, false);
    car_control_tick(&control, &config, &new_press_edge);
    CHECK_EQ("new press edge waits for the 30ms debounce",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    car_inputs_t new_press_stable =
        make_debounced_inputs(193u, false, false, true);
    car_control_tick(&control, &config, &new_press_stable);
    CHECK_EQ("one new debounced press starts Bench",
             control.mode, CAR_MODE_BENCH_RUNNING);
    CHECK_EQ("both channels start from the same ramped duty state",
             car_control_bench_channel_mask(&control), 0x06u);
    CHECK_EQ("Bench run begins from zero duty",
             control.bench_output_duty_permille, 0u);
    CHECK_EQ("starting remains isolated from road drive",
             control.road_drive_allowed, 0u);
    CHECK_EQ("ready drops after the run starts",
             car_control_bench_ready(&control, &config,
                                     &new_press_stable, 1u), 0u);

    car_control_t completion = control;
    send_frame_at(&completion, &config, 7u, true, 80u, 0,
                  350u, 350u, false);
    send_frame_at(&completion, &config, 8u, true, 80u, 0,
                  550u, 550u, false);
    car_inputs_t reached_40 = make_inputs(593u, false);
    car_control_tick(&completion, &config, &reached_40);
    CHECK_EQ("10ms 1 percent ramp reaches 40 percent at 400ms",
             completion.bench_output_duty_permille, 400u);
    car_inputs_t before_deadline = make_inputs(683u, false);
    car_control_tick(&completion, &config, &before_deadline);
    CHECK_EQ("40 percent remains applied until the 500ms deadline",
             completion.bench_output_duty_permille, 400u);
    car_inputs_t at_deadline = make_inputs(693u, false);
    car_control_tick(&completion, &config, &at_deadline);
    CHECK_EQ("single Bench run ends at 500ms from start",
             completion.mode, CAR_MODE_BENCH_COMPLETE);
    CHECK_EQ("completion stops Bench output",
             completion.bench_output_duty_permille, 0u);
    CHECK_EQ("completion retains the reset-scoped issue marker",
             completion.profile_command_issued, 1u);
    CHECK_EQ("completed run cannot auto-SELECT again",
             car_control_auto_select_bench_output(
                 &completion, &config, &at_deadline, 1u),
             CAR_EFFECT_NONE);

    car_control_t stopped = control;
    car_inputs_t stop_inputs = make_inputs(194u, false);
    effects = car_control_command(&stopped, &config, &stop_inputs,
                                  CAR_CMD_NORMAL_STOP, CAR_MOTOR_NONE,
                                  CAR_MOTOR_IN1, 0u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "normal STOP stops the one-shot run");
    CHECK_EQ("STOP retains the reset-scoped issue marker",
             stopped.profile_command_issued, 1u);
    car_control_rx_recovery_flush_complete(&stopped);
    for (uint16_t sequence = 7u; sequence <= 9u; ++sequence) {
        const uint32_t now = 201u + (uint32_t)(sequence - 7u) * 10u;
        send_frame_at(&stopped, &config, sequence, true, 80u, 0,
                      now, now, true);
    }
    CHECK_EQ("STOP recovery still needs three frames",
             stopped.mode, CAR_MODE_WAIT_START_EDGE);
    car_inputs_t stopped_ready = make_inputs(222u, true);
    CHECK_EQ("STOP cannot re-prepare Bench without reset",
             car_control_auto_select_bench_output(
                 &stopped, &config, &stopped_ready, 1u),
             CAR_EFFECT_NONE);
    CHECK_EQ("STOP leaves the physical target cleared",
             stopped.bench_motor, CAR_MOTOR_NONE);

    car_control_t faulted = control;
    car_control_note_fault(&faulted, CAR_FAULT_LOOP_GAP);
    car_inputs_t faulted_inputs = make_inputs(194u, true);
    CHECK_EQ("faulted one-shot cannot auto-SELECT again",
             car_control_auto_select_bench_output(
                 &faulted, &config, &faulted_inputs, 1u),
             CAR_EFFECT_NONE);

    car_control_t soft_faulted = control;
    car_control_note_fault(&soft_faulted, CAR_FAULT_UART);
    CHECK_EQ("UART soft fault stops the active one-shot",
             soft_faulted.bench_output_duty_permille, 0u);
    CHECK_EQ("UART soft fault clears the selected motor",
             soft_faulted.bench_motor, CAR_MOTOR_NONE);
    CHECK_EQ("UART soft fault preserves the issue marker",
             soft_faulted.profile_command_issued, 1u);
    car_control_rx_recovery_flush_complete(&soft_faulted);
    for (uint16_t sequence = 7u; sequence <= 9u; ++sequence) {
        const uint32_t now = 201u + (uint32_t)(sequence - 7u) * 10u;
        send_frame_at(&soft_faulted, &config, sequence, true, 80u, 0,
                      now, now, true);
    }
    CHECK_EQ("UART soft fault recovers after the new three-frame batch",
             soft_faulted.mode, CAR_MODE_WAIT_START_EDGE);
    car_inputs_t soft_released = make_inputs(222u, true);
    car_control_tick(&soft_faulted, &config, &soft_released);
    CHECK_EQ("UART recovery cannot auto-select a replacement target",
             car_control_auto_select_bench_output(
                 &soft_faulted, &config, &soft_released, 1u),
             CAR_EFFECT_NONE);
    car_inputs_t after_soft_press_edge =
        make_debounced_inputs(223u, false, true, false);
    car_control_tick(&soft_faulted, &config, &after_soft_press_edge);
    car_inputs_t after_soft_press_stable =
        make_debounced_inputs(243u, false, false, true);
    car_control_tick(&soft_faulted, &config, &after_soft_press_stable);
    CHECK_EQ("new press after UART recovery cannot repulse without reset",
             soft_faulted.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("UART recovery leaves Bench output at zero",
             soft_faulted.bench_output_duty_permille, 0u);
}

void test_bench_five_second_target_hold_and_wraparound()
{
    car_config_t config = make_config(true, false);
    config.bench_ramp_step_permille = 10u;
    config.bench_run_limit_ms =
        static_cast<uint16_t>(CARTEST_BENCH_RUN_LIMIT_MS);
    /* After the 24ms press debounce and 400ms ramp, the target timestamp is
       exactly zero. The explicit active flag must distinguish this from idle. */
    const uint32_t release_start = 0xFFFFFE58u;
    car_control_t control{};
    start_bench_with_fresh_frame_at(&control, &config, release_start);
    CHECK_EQ("wrapped-time physical press enters Bench running",
             control.mode, CAR_MODE_BENCH_RUNNING);
    CHECK_EQ("5-second hold starts from a zero duty command",
             control.bench_output_duty_permille, 0u);
    const uint32_t bench_start = control.bench_started_ms;

    uint16_t sequence = 4u;
    for (uint32_t step = 1u; step <= 40u; ++step) {
        const uint32_t now = bench_start + step * 10u;
        if ((step % 20u) == 0u) {
            send_frame_at(&control, &config, sequence++, true, 80u, 0,
                          now, now, false);
        }
        const car_inputs_t inputs = make_inputs(now, false);
        car_control_tick(&control, &config, &inputs);
        CHECK_EQ("ramp reaches 40 percent at 400ms across tick wrap",
                 control.bench_output_duty_permille,
                 static_cast<uint16_t>(step * 10u));
    }

    CHECK_EQ("target-hold timer is active at the software 400 command",
             control.bench_target_hold_active, 1u);
    CHECK_EQ("target hold may start at timestamp zero after uint32 wrap",
             control.bench_target_reached_ms, 0u);
    CHECK_EQ("software target hold starts on the first 400 command",
             static_cast<uint32_t>(control.bench_target_reached_ms - bench_start),
             400u);
    CHECK_EQ("road-drive path remains disabled during the target hold",
             control.road_drive_allowed, 0u);
    const uint32_t target_time = control.bench_target_reached_ms;

    car_control_t early_release = control;
    const car_inputs_t release_inputs = make_inputs(target_time + 10u, true);
    car_control_tick(&early_release, &config, &release_inputs);
    CHECK_EQ("PE7 release stops during the 5-second hold",
             early_release.bench_output_duty_permille, 0u);
    CHECK_EQ("PE7 release clears the hold timer",
             early_release.bench_target_hold_active, 0u);
    CHECK_EQ("PE7 release clears the stored hold timestamp",
             early_release.bench_target_reached_ms, 0u);
    CHECK_EQ("PE7 release faults the active Bench pulse",
             early_release.fault, CAR_FAULT_SWITCH_RELEASE);

    car_control_t uart_fault = control;
    car_control_note_fault(&uart_fault, CAR_FAULT_UART);
    CHECK_EQ("UART fault stops during the target hold",
             uart_fault.bench_output_duty_permille, 0u);
    CHECK_EQ("UART fault clears the target hold timer",
             uart_fault.bench_target_hold_active, 0u);
    CHECK_EQ("UART fault clears the stored hold timestamp",
             uart_fault.bench_target_reached_ms, 0u);

    car_control_t vision_timeout = control;
    const car_inputs_t stale_vision =
        make_inputs(target_time + config.vision_timeout_ms, false);
    car_control_tick(&vision_timeout, &config, &stale_vision);
    CHECK_EQ("stale vision stops during the target hold",
             vision_timeout.bench_output_duty_permille, 0u);
    CHECK_EQ("stale vision faults the active Bench pulse",
             vision_timeout.fault, CAR_FAULT_VISION_TIMEOUT);
    CHECK_EQ("vision fault clears the stored hold timestamp",
             vision_timeout.bench_target_reached_ms, 0u);

    /* Keep accepted valid frames flowing; this exercises freshness rather than
       extending or bypassing the production vision timeout. */
    car_control_t completion = control;
    for (uint32_t hold_elapsed = 10u;
         hold_elapsed < CARTEST_BENCH_HOLD_MS;
         hold_elapsed += 10u) {
        const uint32_t now = target_time + hold_elapsed;
        if ((hold_elapsed % 200u) == 0u) {
            send_frame_at(&completion, &config, sequence++, true, 80u, 0,
                          now, now, false);
        }
        const car_inputs_t inputs = make_inputs(now, false);
        car_control_tick(&completion, &config, &inputs);
        CHECK_EQ("400 duty remains through 4,990ms target hold",
                 completion.bench_output_duty_permille, 400u);
        CHECK_EQ("run remains active until the full target hold elapses",
                 completion.mode, CAR_MODE_BENCH_RUNNING);
    }

    const uint32_t hold_deadline = target_time + CARTEST_BENCH_HOLD_MS;
    send_frame_at(&completion, &config, sequence++, true, 80u, 0,
                  hold_deadline, hold_deadline, false);
    const car_inputs_t hold_deadline_inputs = make_inputs(hold_deadline, false);
    car_control_tick(&completion, &config, &hold_deadline_inputs);
    CHECK_EQ("5000ms hold after the 400 command completes the one-shot",
             completion.mode, CAR_MODE_BENCH_COMPLETE);
    CHECK_EQ("completed target hold stops the motor command",
             completion.bench_output_duty_permille, 0u);
    CHECK_EQ("normal ramp plus hold ends after 5400ms from start",
             static_cast<uint32_t>(hold_deadline - bench_start), 5400u);
    CHECK_EQ("completed target hold clears its active timer",
             completion.bench_target_hold_active, 0u);
    CHECK_EQ("completed target hold clears its timestamp",
             completion.bench_target_reached_ms, 0u);
    CHECK_EQ("completed hold cannot automatically select another pulse",
             car_control_auto_select_bench_output(
                 &completion, &config, &hold_deadline_inputs, 1u),
             CAR_EFFECT_NONE);

    /* Model a delayed scheduler reaching the target just before the absolute
       cap. The independent 5.5s cap still wins; no late ramp earns more time. */
    car_control_t delayed_schedule{};
    start_bench_with_fresh_frame_at(&delayed_schedule, &config, 1000u, 1u);
    const uint32_t delayed_start = delayed_schedule.bench_started_ms;
    uint16_t delayed_sequence = 4u;
    for (uint32_t observation_age_ms = 200u;
         observation_age_ms < CARTEST_BENCH_RUN_LIMIT_MS - 100u;
         observation_age_ms += 200u) {
        const uint32_t observation_ms = delayed_start + observation_age_ms;
        send_frame_at(&delayed_schedule, &config, delayed_sequence++, true,
                      80u, 0, observation_ms, observation_ms, false);
    }
    const uint32_t late_target = delayed_start +
        static_cast<uint32_t>(CARTEST_BENCH_RUN_LIMIT_MS - 100u);
    send_frame_at(&delayed_schedule, &config, delayed_sequence++, true, 80u, 0,
                  late_target, late_target, false);
    const car_inputs_t late_target_inputs = make_inputs(late_target, false);
    car_control_tick(&delayed_schedule, &config, &late_target_inputs);
    CHECK_EQ("late scheduler still caps the software target at 40 percent",
             delayed_schedule.bench_output_duty_permille, 400u);
    CHECK_EQ("late target command starts a fresh hold timer",
             static_cast<uint32_t>(delayed_schedule.bench_target_reached_ms -
                                   delayed_start),
             CARTEST_BENCH_RUN_LIMIT_MS - 100u);
    CHECK_EQ("late scheduler cannot claim an already completed hold",
             delayed_schedule.mode, CAR_MODE_BENCH_RUNNING);
    const uint32_t absolute_deadline =
        delayed_start + CARTEST_BENCH_RUN_LIMIT_MS;
    send_frame_at(&delayed_schedule, &config, delayed_sequence++, true, 80u, 0,
                  absolute_deadline, absolute_deadline, false);
    const car_inputs_t absolute_deadline_inputs =
        make_inputs(absolute_deadline, false);
    car_control_tick(&delayed_schedule, &config, &absolute_deadline_inputs);
    CHECK_EQ("5.5-second absolute limit ends a late target hold",
             delayed_schedule.mode, CAR_MODE_BENCH_COMPLETE);
    CHECK_EQ("absolute limit stops before the hold can extend the run",
             delayed_schedule.bench_output_duty_permille, 0u);
    CHECK_EQ("absolute run deadline is measured with unsigned wrap-safe delta",
             static_cast<uint32_t>(absolute_deadline - delayed_start),
             CARTEST_BENCH_RUN_LIMIT_MS);
}

void test_invalid_and_illegal_commands_revoke_output()
{
    car_config_t invalid_config = make_config(true, false);
    invalid_config.bench_duty_limit_permille = 401u;
    car_control_t invalid_control{};
    car_control_init(&invalid_control, &invalid_config, 0u, 1u, 0u);
    CHECK_EQ("bench limit above hard 40 percent fails closed",
             invalid_control.fault, CAR_FAULT_CONFIG);
    CHECK_EQ("invalid bench config leaves PWM zero",
             invalid_control.bench_output_duty_permille, 0u);

    const car_config_t config = make_config(true, false);
    car_control_t control{};
    start_bench_with_fresh_frame(&control, &config, 1u);
    car_inputs_t low = make_inputs(34u, false);
    car_control_tick(&control, &config, &low);
    CHECK_EQ("bench has nonzero ramp before illegal command",
             control.bench_output_duty_permille, 100u);

    const uint32_t effects = car_control_command(
        &control, &config, &low, static_cast<car_command_t>(0xDEADBEEFu),
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 400u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "unknown command immediately requests output stop");
    CHECK_EQ("illegal command latches configuration fault", control.fault,
             CAR_FAULT_CONFIG);
    CHECK_EQ("illegal command drops PWM immediately", control.bench_output_duty_permille, 0u);
    CHECK_EQ("illegal command clears selected motor", control.bench_motor, CAR_MOTOR_NONE);
    CHECK_EQ("illegal command cannot authorize road drive", control.road_drive_allowed, 0u);

    const car_inputs_t high = make_inputs(0u, true);
    car_control_t invalid_pair{};
    car_control_init(&invalid_pair, &config, 0u, 1u, 0u);
    const uint32_t invalid_pair_effects = car_control_command(
        &invalid_pair, &config, &high, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_BOTH_FORWARD, CAR_MOTOR_IN2, 400u);
    CHECK((invalid_pair_effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "both-forward with IN2 is rejected and stopped");
    CHECK_EQ("illegal paired marker cannot select any output",
             car_control_bench_channel_mask(&invalid_pair), 0u);

    car_control_t invalid_motor{};
    car_control_init(&invalid_motor, &config, 0u, 1u, 0u);
    const uint32_t invalid_motor_effects = car_control_command(
        &invalid_motor, &config, &high, CAR_CMD_SELECT_BENCH_OUTPUT,
        4u, CAR_MOTOR_IN1, 400u);
    CHECK((invalid_motor_effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "unknown motor enum is rejected and stopped");

    car_control_t reject_control{};
    car_control_init(&reject_control, &config, 0u, 1u, 0u);
    const uint32_t rejected = car_control_command(
        &reject_control, &config, &high, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 401u);
    CHECK((rejected & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "requested duty above configured 40 percent is rejected");
    CHECK_EQ("over-limit request never creates PWM", reject_control.bench_output_duty_permille, 0u);
    CHECK_EQ("over-limit request counted", reject_control.command_reject_count, 1u);
}

void test_switch_release_isr_stops_and_latches()
{
    const car_config_t config = make_config(true, false);
    car_control_t control{};
    start_bench_with_fresh_frame(&control, &config, 1u);
    car_inputs_t low = make_inputs(34u, false);
    car_control_tick(&control, &config, &low);
    CHECK_EQ("motor pulse active before switch release", control.bench_output_duty_permille, 100u);

    car_control_note_switch_release_isr(&control);
    car_inputs_t released = make_inputs(35u, true);
    car_control_tick(&control, &config, &released);
    CHECK_EQ("PE7 release is a recoverable vision-session fault",
             control.fault, CAR_FAULT_SWITCH_RELEASE);
    CHECK_EQ("switch release waits for fresh observations", control.mode,
             CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("switch release does not masquerade as a hard latch",
             control.fault_latched, 0u);
    CHECK_EQ("switch release stops PWM", control.bench_output_duty_permille, 0u);
    CHECK_EQ("switch release revokes bench motor", control.bench_motor, CAR_MOTOR_NONE);
}

void test_fast_stop_fault_latch_blocks_ack_until_consumed()
{
    const car_config_t config = make_config(true, false);
    car_control_t control{};
    start_bench_with_fresh_frame(&control, &config, 1u);
    car_inputs_t low = make_inputs(34u, false);
    car_control_tick(&control, &config, &low);
    CHECK_EQ("motor pulse active before fast fault", control.bench_output_duty_permille, 100u);

    car_control_note_fast_stop_isr(&control, CAR_FAULT_LOOP_GAP);
    CHECK_EQ("ISR stores fast-stop reason", control.isr_stop_fault, CAR_FAULT_LOOP_GAP);
    const car_inputs_t high = make_inputs(35u, true);
    const uint32_t rejected_ack = car_control_command(
        &control, &config, &high, CAR_CMD_LOCAL_ACK,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((rejected_ack & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "local ACK cannot erase an unconsumed ISR fault");
    CHECK_EQ("rejected ACK preserves ISR reason", control.isr_stop_fault,
             CAR_FAULT_LOOP_GAP);

    const car_fault_t consumed = car_control_take_fast_stop_isr(&control);
    CHECK_EQ("main-loop consumer receives ISR reason", consumed, CAR_FAULT_LOOP_GAP);
    car_control_note_fault(&control, consumed);
    CHECK_EQ("consumed fast fault is promoted to latched state", control.fault,
             CAR_FAULT_LOOP_GAP);
    CHECK_EQ("fast fault stops software bench output",
             control.bench_output_duty_permille, 0u);
    CHECK_EQ("fast fault revokes selected motor", control.bench_motor, CAR_MOTOR_NONE);

    car_inputs_t stable_release = make_debounced_inputs(55u, true, true, false);
    car_control_tick(&control, &config, &stable_release);
    CHECK_EQ("hard fault requires a stable new press before release ACK",
             control.hard_ack_press_seen, 0u);
    car_inputs_t press_edge = make_debounced_inputs(56u, false, true, false);
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(76u, false, false, true);
    car_control_tick(&control, &config, &press_stable);
    CHECK_EQ("stable post-fault press arms physical acknowledgement",
             control.hard_ack_press_seen, 1u);
    car_inputs_t release_edge = make_debounced_inputs(77u, true, false, true);
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t release_stable = make_debounced_inputs(97u, true, true, false);
    car_control_tick(&control, &config, &release_stable);
    CHECK_EQ("stable press-release acknowledges only after fault source clears",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("hard ACK creates a fresh atomic RX boundary",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("hard ACK never restarts a motor output",
             control.bench_output_duty_permille, 0u);
}

void test_soft_fault_recovers_without_auto_start()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    CHECK_EQ("startup has no automatic recovery count", control.auto_recovery_count, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(&control, &config, &released);
    send_frame_at(&control, &config, 10u, true, 80u, 0, 1u, 1u, true);
    car_control_note_fault(&control, CAR_FAULT_UART);
    CHECK_EQ("soft UART fault requests a clean RX boundary",
             control.rx_recovery_flush_required, 1u);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 11u, true, 80u, 200, 10u, 10u, true);
    send_frame_at(&control, &config, 12u, true, 80u, 100, 20u, 20u, true);
    CHECK_EQ("two fresh frames are not enough to recover",
             control.recovery_frame_count, 2u);
    CHECK_EQ("observation recovery never starts an unstarted FOLLOW session",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("no FOLLOW session exists to resume", control.follow_session_active, 0u);
    CHECK_EQ("fault remains until all three recovery frames arrive",
             control.fault, CAR_FAULT_UART);
    CHECK_EQ("soft UART fault is recoverable, not a hard latch",
             control.fault_latched, 0u);
    CHECK_EQ("recovery wait keeps FOLLOW output zero",
             control.follow_output_duty_permille, 0u);

    send_frame_at(&control, &config, 13u, true, 80u, -100, 30u, 30u, true);
    CHECK_EQ("third fresh frame completes soft observation recovery",
             control.recovery_frame_count, 3u);
    CHECK_EQ("third soft-recovery frame increments counter once",
             control.auto_recovery_count, 1u);
    CHECK_EQ("soft fault unlocks observation after three frames",
             control.recovery_active, 0u);
    CHECK_EQ("observation recovery clears only the recoverable fault",
             control.fault_latched, 0u);
    CHECK_EQ("observation recovery still does not start FOLLOW",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("trusted observation is refreshed after recovery",
             control.have_valid_vision, 1u);
    CHECK_EQ("road output remains gated before a separate PE7 press",
             control.follow_output_duty_permille, 0u);
    car_control_reverse_control_keep_started(&control);
    send_frame_at(&control, &config, 14u, true, 80u, 0, 31u, 31u, true);
    send_frame_at(&control, &config, 15u, true, 80u, 0, 32u, 32u, true);
    send_frame_at(&control, &config, 16u, true, 80u, 0, 33u, 33u, true);
    CHECK_EQ("later fresh observations do not count the same recovery again",
             control.auto_recovery_count, 1u);

    car_inputs_t press_edge = make_debounced_inputs(34u, false, true, false);
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(54u, false, false, true);
    car_control_tick(&control, &config, &press_stable);
    CHECK_EQ("new physical press starts only after observation recovery",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("manual start creates the resumable FOLLOW session",
             control.follow_session_active, 1u);
    CHECK_EQ("first FOLLOW output starts from zero after separate press",
             control.follow_output_duty_permille, 0u);
}

void start_follow_by_physical_pe7(car_control_t *control,
                                  const car_config_t *config,
                                  uint16_t first_sequence,
                                  uint32_t start_ms)
{
    car_control_init(control, config, start_ms, 1u, 0u);
    confirm_follow_startup_release(control, config, start_ms);
    /* Host fixture models the completed reverse RESET phase and KEEP lease. */
    car_control_reverse_control_keep_started(control);
    car_inputs_t released = make_inputs(start_ms, true);
    car_control_tick(control, config, &released);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t frame_ms = start_ms + 1u + offset;
        send_frame_at(control, config,
                      static_cast<uint16_t>(first_sequence + offset),
                      true, 80u, 0, frame_ms, frame_ms, true);
    }
    car_inputs_t press_edge = make_debounced_inputs(
        start_ms + 4u, false, true, false);
    car_control_tick(control, config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(
        start_ms + 24u, false, false, true);
    car_control_tick(control, config, &press_stable);
}

void enter_follow_visual_hold(car_control_t *control,
                              const car_config_t *config,
                              uint16_t first_sequence,
                              uint32_t start_ms)
{
    start_follow_by_physical_pe7(control, config, first_sequence, start_ms);
    send_frame_at(control, config,
                  static_cast<uint16_t>(first_sequence + 3u), true, 80u, 545,
                  start_ms + 27u, start_ms + 27u, false);
    car_control_note_applied_outputs(control, 1u, 1u, 1600u, 120u, 180u);
    send_frame_at(control, config,
                  static_cast<uint16_t>(first_sequence + 4u), false, 0u, 0,
                  start_ms + 28u, start_ms + 28u, false);
}

void test_debug_steering_mapping_and_automatic_tracking()
{
    CHECK_EQ("servo left calibration", CARTEST_SERVO_LEFT_US, 1100u);
    CHECK_EQ("servo center calibration", CARTEST_SERVO_CENTER_US, 1530u);
    CHECK_EQ("servo right calibration", CARTEST_SERVO_RIGHT_US, 2500u);
    CHECK_EQ("measured servo limits remain explicit", CARTEST_SERVO_LIMITS_MEASURED, 1u);
    CHECK_EQ("P0.7 negative endpoint map", car_debug_steering_target_us(-1000), 1229u);
    CHECK_EQ("P0.7 center map", car_debug_steering_target_us(0), 1530u);
    CHECK_EQ("P0.7 positive endpoint map", car_debug_steering_target_us(1000), 2209u);
    CHECK_EQ("negative overrange clamps", car_debug_steering_target_us(-2000), 1229u);
    CHECK_EQ("positive overrange clamps", car_debug_steering_target_us(2000), 2209u);
    CHECK_EQ("int16 minimum clamps", car_debug_steering_target_us(INT16_MIN), 1229u);
    CHECK_EQ("int16 maximum clamps", car_debug_steering_target_us(INT16_MAX), 2209u);
    CHECK_EQ("measured Follow calibration is enabled",
             CARTEST_FOLLOW_CALIBRATION_COMPLETE, 1u);
    CHECK_EQ("left encoder count remains", CARTEST_ENCODER_LEFT_COUNTS_PER_REV, 59995u);
    CHECK_EQ("right encoder count remains", CARTEST_ENCODER_RIGHT_COUNTS_PER_REV, 60239u);
    CHECK_EQ("left encoder sign remains", CARTEST_ENCODER_LEFT_SIGN, 1);
    CHECK_EQ("right encoder sign remains", CARTEST_ENCODER_RIGHT_SIGN, -1);

    uint16_t previous = car_debug_steering_target_us(-1000);
    for (int32_t error = -999; error <= 1000; ++error) {
        const uint16_t target =
            car_debug_steering_target_us(static_cast<int16_t>(error));
        CHECK(target >= previous, "steering map is monotonic");
        CHECK(target >= CARTEST_SERVO_MIN_US &&
              target <= CARTEST_SERVO_MAX_US,
              "every mapped pulse stays inside hard limits");
        previous = target;
    }

    car_config_t config = make_debug_steering_config();
    CHECK_EQ("DebugSteering carries the measured Follow calibration flag",
             config.follow_calibration_ready, 1u);
    CHECK_EQ("DebugSteering has no Follow profile", config.follow_mode_compiled, 0u);
    CHECK_EQ("DebugSteering config is valid without Follow readiness",
             config.debug_steering_mode_compiled, 1u);

    for (uint8_t initial_pe7 = 0u; initial_pe7 <= 1u; ++initial_pe7) {
        car_control_t control{};
        car_control_init(&control, &config, 0u, initial_pe7, 0u);
        CHECK_EQ("boot waits for camera recovery validation",
                 control.mode, CAR_MODE_RECOVERY_WAIT);
        CHECK_EQ("PE7 level does not block DebugSteering startup",
                 control.startup_release_seen, 1u);
        CHECK_EQ("no servo session before trusted data",
                 control.steering_session_active, 0u);
        CHECK_EQ("no motor permission at boot", control.road_drive_allowed, 0u);

        const bool first_switch = initial_pe7 != 0u;
        send_frame_at(&control, &config, 1u, true, 80u, -1000,
                      10u, 10u, first_switch);
        send_frame_at(&control, &config, 2u, true, 80u, 0,
                      20u, 20u, !first_switch);
        CHECK_EQ("two trusted frames cannot start the servo",
                 control.steering_session_active, 0u);
        car_control_note_switch_release_isr(&control);
        CHECK_EQ("PE7 release does not latch a stop in DebugSteering",
                 control.isr_stop_request, 0u);
        send_frame_at(&control, &config, 3u, true, 80u, -1000,
                      30u, 30u, !first_switch);
        CHECK_EQ("third trusted frame automatically starts steering",
                 control.mode, CAR_MODE_DEBUG_STEERING_RUNNING);
        CHECK_EQ("independent steering session becomes active",
                 control.steering_session_active, 1u);
        CHECK_EQ("Follow session remains inactive",
                 control.follow_session_active, 0u);
        CHECK_EQ("automatic start uses the third frame target",
                 control.steering_servo_target_us, 1229u);
        CHECK_EQ("drive stays prohibited", control.road_drive_allowed, 0u);
        CHECK_EQ("bench motor duty stays zero", control.bench_output_duty_permille, 0u);
        CHECK_EQ("Follow motor duty stays zero", control.follow_output_duty_permille, 0u);

        car_inputs_t pe7_release = make_inputs(31u, true);
        car_control_tick(&control, &config, &pe7_release);
        CHECK_EQ("PE7 release does not stop steering",
                 control.steering_session_active, 1u);
        CHECK_EQ("PE7 release does not fault steering", control.fault, CAR_FAULT_NONE);
        car_inputs_t pe7_press = make_inputs(32u, false);
        car_control_tick(&control, &config, &pe7_press);
        CHECK_EQ("PE7 press does not stop or restart steering",
                 control.steering_session_active, 1u);
        send_frame_at(&control, &config, 4u, true, 80u, 1000,
                      33u, 33u, false);
        CHECK_EQ("each new trusted deviation updates target immediately",
                 control.steering_servo_target_us, 2209u);
        CHECK_EQ("vision tracking never grants motor drive",
                 control.road_drive_allowed, 0u);
    }

    car_config_t locked_follow = make_follow_config(false);
    car_control_t follow_control{};
    car_control_init(&follow_control, &locked_follow, 0u, 1u, 0u);
    CHECK_EQ("Follow profile remains hard locked without calibration",
             follow_control.fault, CAR_FAULT_CONFIG);
    CHECK_EQ("locked Follow cannot authorize drive",
             follow_control.road_drive_allowed, 0u);
    CHECK_EQ("locked Follow session remains inactive",
             follow_control.follow_session_active, 0u);
}

void test_debug_steering_fault_recovery_and_hard_lock()
{
    const car_config_t config = make_debug_steering_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 0u, 0u);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = static_cast<uint32_t>(sequence) * 10u;
        send_frame_at(&control, &config, sequence, true, 80u, 250,
                      now, now, (sequence & 1u) != 0u);
    }
    CHECK_EQ("startup starts only after three fresh valid observations",
             control.steering_session_active, 1u);
    CHECK_EQ("startup target maps trusted image error",
             control.steering_servo_target_us,
             car_debug_steering_target_us(250));

    send_frame_at(&control, &config, 4u, false, 80u, 0,
                  31u, 31u, true);
    CHECK_EQ("invalid frame stops steering", control.steering_session_active, 0u);
    CHECK_EQ("invalid frame enters recovery", control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("invalid frame fault is retained", control.fault, CAR_FAULT_VISION_INVALID);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 5u, true, 80u, -500, 32u, 32u, false);
    send_frame_at(&control, &config, 6u, true, 80u, -250, 42u, 42u, true);
    CHECK_EQ("two recovery frames do not restart steering",
             control.steering_session_active, 0u);
    send_frame_at(&control, &config, 7u, true, 80u, -750, 52u, 52u, false);
    CHECK_EQ("third recovery frame automatically resumes steering",
             control.steering_session_active, 1u);
    CHECK_EQ("recovery immediately applies latest trusted target",
             control.steering_servo_target_us,
             car_debug_steering_target_us(-750));

    send_frame_at(&control, &config, 8u, true, 59u, 700, 53u, 53u, true);
    CHECK_EQ("valid low-confidence frame keeps steering session active",
             control.steering_session_active, 1u);
    CHECK_EQ("confidence does not gate an otherwise valid CRC frame",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("low confidence remains visible as diagnostic data",
             control.vision_confidence, 59u);
    send_frame_at(&control, &config, 9u, true, 80u, 500, 54u, 54u, true);
    send_frame_at(&control, &config, 10u, true, 80u, 400, 64u, 64u, false);
    send_frame_at(&control, &config, 11u, true, 80u, 300, 74u, 74u, true);
    CHECK_EQ("valid later frames keep steering active without a score gate",
             control.steering_session_active, 1u);
    CHECK_EQ("steering uses latest valid image target",
             control.steering_servo_target_us,
             car_debug_steering_target_us(300));

    car_inputs_t timeout = make_inputs(324u, true);
    car_control_tick(&control, &config, &timeout);
    CHECK_EQ("vision timeout stops steering", control.steering_session_active, 0u);
    CHECK_EQ("vision timeout enters recovery", control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("vision timeout fault retained", control.fault, CAR_FAULT_VISION_TIMEOUT);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 12u, true, 80u, -100, 325u, 325u, false);
    send_frame_at(&control, &config, 13u, true, 80u, -200, 335u, 335u, true);
    CHECK_EQ("timeout recovery requires a third fresh frame",
             control.steering_session_active, 0u);
    send_frame_at(&control, &config, 14u, true, 80u, -300, 345u, 345u, false);
    CHECK_EQ("timeout recovery restarts after three fresh frames",
             control.steering_session_active, 1u);
    CHECK_EQ("timeout recovery target is latest observation",
             control.steering_servo_target_us,
             car_debug_steering_target_us(-300));

    car_control_note_fast_stop_isr(&control, CAR_FAULT_UART);
    CHECK_EQ("UART ISR error immediately clears steering session",
             control.steering_session_active, 0u);
    const car_fault_t uart_fault = car_control_take_fast_stop_isr(&control);
    CHECK_EQ("UART ISR cause is retained", uart_fault, CAR_FAULT_UART);
    car_control_note_fault(&control, uart_fault);
    CHECK_EQ("UART error enters visual recovery", control.mode,
             CAR_MODE_RECOVERY_WAIT);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 15u, true, 80u, 200, 347u, 347u, true);
    send_frame_at(&control, &config, 16u, true, 80u, 100, 357u, 357u, false);
    CHECK_EQ("UART recovery remains stopped before frame three",
             control.steering_session_active, 0u);
    send_frame_at(&control, &config, 17u, true, 80u, 50, 367u, 367u, true);
    CHECK_EQ("UART recovery automatically resumes after three fresh frames",
             control.steering_session_active, 1u);

    send_frame_at(&control, &config, 17u, true, 80u, 900, 368u, 368u, false);
    CHECK_EQ("sequence replay stops DebugSteering",
             control.steering_session_active, 0u);
    CHECK_EQ("sequence replay is reported", control.fault, CAR_FAULT_SEQUENCE);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 18u, true, 80u, 600, 369u, 369u, true);
    send_frame_at(&control, &config, 19u, true, 80u, 700, 379u, 379u, false);
    CHECK_EQ("replay recovery requires the third fresh sequence",
             control.steering_session_active, 0u);
    send_frame_at(&control, &config, 20u, true, 80u, 800, 389u, 389u, true);
    CHECK_EQ("replay recovery resumes only after three new sequences",
             control.steering_session_active, 1u);

    car_inputs_t hard_fault = make_inputs(390u, false);
    hard_fault.encoder_fault = 1u;
    car_control_tick(&control, &config, &hard_fault);
    CHECK_EQ("encoder fault hard-latches DebugSteering",
             control.mode, CAR_MODE_FAULT_LATCHED);
    CHECK_EQ("hard fault immediately clears steering session",
             control.steering_session_active, 0u);
    send_frame_at(&control, &config, 21u, true, 80u, 900, 391u, 391u, true);
    send_frame_at(&control, &config, 22u, true, 80u, 800, 401u, 401u, false);
    send_frame_at(&control, &config, 23u, true, 80u, 700, 411u, 411u, true);
    CHECK_EQ("trusted frames cannot clear a hard fault",
             control.mode, CAR_MODE_FAULT_LATCHED);
    CHECK_EQ("hard fault cannot auto-restart servo",
             control.steering_session_active, 0u);
    CHECK_EQ("hard fault never grants motor drive", control.road_drive_allowed, 0u);
}



void test_rejected_commands_revoke_active_outputs()
{
    const car_config_t bench_config = make_config(true, false);
    const car_inputs_t bench_tick = make_inputs(34u, false);
    car_control_t control{};

    start_bench_with_fresh_frame(&control, &bench_config, 1u);
    car_control_tick(&control, &bench_config, &bench_tick);
    CHECK_EQ("bench reject fixture has active output",
             control.bench_output_duty_permille, 100u);
    uint32_t effects = car_control_command(
        &control, &bench_config, &bench_tick, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 401u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "over-limit SELECT during active bench requests hardware stop");
    CHECK_EQ("rejected SELECT immediately clears bench duty",
             control.bench_output_duty_permille, 0u);
    CHECK_EQ("rejected SELECT leaves BENCH_RUNNING",
             control.mode == CAR_MODE_BENCH_RUNNING, 0u);
    car_inputs_t after_select = make_inputs(35u, false);
    car_control_tick(&control, &bench_config, &after_select);
    CHECK_EQ("next tick cannot re-enable rejected bench SELECT",
             control.bench_output_duty_permille, 0u);

    start_bench_with_fresh_frame(&control, &bench_config, 10u);
    car_control_tick(&control, &bench_config, &bench_tick);
    CHECK_EQ("second bench reject fixture has active output",
             control.bench_output_duty_permille, 100u);
    effects = car_control_command(
        &control, &bench_config, &bench_tick,
        static_cast<car_command_t>(0xDEAD0001u),
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "unknown command during bench requests hardware stop");
    car_inputs_t after_servo_reject = make_inputs(35u, false);
    car_control_tick(&control, &bench_config, &after_servo_reject);
    CHECK_EQ("next tick cannot re-enable output after rejected SERVO",
             control.bench_output_duty_permille, 0u);
    const car_config_t follow_config = make_follow_config();
    start_follow_by_physical_pe7(&control, &follow_config, 100u, 0u);
    car_inputs_t follow_tick = make_inputs(34u, false);
    car_control_tick(&control, &follow_config, &follow_tick);
    CHECK_EQ("FOLLOW reject fixture has active limited output",
             control.follow_output_duty_permille, 10u);
    effects = car_control_command(
        &control, &follow_config, &follow_tick, CAR_CMD_SELECT_BENCH_OUTPUT,
        CAR_MOTOR_LEFT, CAR_MOTOR_IN1, 100u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "uncompiled SELECT during FOLLOW requests hardware stop");
    CHECK_EQ("rejected SELECT cancels FOLLOW session",
             control.follow_session_active, 0u);
    car_inputs_t follow_after_select = make_inputs(35u, false);
    car_control_tick(&control, &follow_config, &follow_after_select);
    CHECK_EQ("next tick cannot re-enable FOLLOW after rejected SELECT",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("rejected SELECT leaves road-drive gate closed",
             control.road_drive_allowed, 0u);

    start_follow_by_physical_pe7(&control, &follow_config, 200u, 100u);
    car_inputs_t follow_tick2 = make_inputs(134u, false);
    car_control_tick(&control, &follow_config, &follow_tick2);
    CHECK_EQ("second FOLLOW reject fixture has active limited output",
             control.follow_output_duty_permille, 10u);
    effects = car_control_command(
        &control, &follow_config, &follow_tick2,
        static_cast<car_command_t>(0xDEAD0002u),
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "unknown command during FOLLOW requests hardware stop");
    CHECK_EQ("unknown command cancels FOLLOW session",
             control.follow_session_active, 0u);
    car_inputs_t follow_after_servo = make_inputs(135u, false);
    car_control_tick(&control, &follow_config, &follow_after_servo);
    CHECK_EQ("next tick cannot re-enable FOLLOW after unknown command",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("unknown command keeps road-drive gate closed",
             control.road_drive_allowed, 0u);
}

void test_follow_boot_release_three_frames_and_ramp()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 0u, 0u);
    CHECK_EQ("boot-held switch does not count as automatic recovery",
             control.auto_recovery_count, 0u);
    car_inputs_t held = make_inputs(0u, false);
    car_control_tick(&control, &config, &held);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = static_cast<uint32_t>(sequence);
        send_frame_at(&control, &config, sequence, true, 80u, 250,
                      now, now, false);
    }
    CHECK_EQ("boot-held PE7 and three pre-release frames cannot start",
             control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("pre-release frames do not create FOLLOW session",
             control.follow_session_active, 0u);

    car_inputs_t release_edge = make_debounced_inputs(4u, true, false, true);
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t release_stable = make_debounced_inputs(24u, true, true, false);
    car_control_tick(&control, &config, &release_stable);
    car_control_rx_recovery_flush_complete(&control);
    CHECK_EQ("release alone cannot start FOLLOW", control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 25u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(4u + offset),
                      true, 80u, 250, now, now, true);
    }
    CHECK_EQ("three post-release frames arm but do not auto-start",
             control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("no motor output before the independent start press",
             control.follow_output_duty_permille, 0u);

    car_control_reverse_control_keep_started(&control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 28u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(7u + offset),
                      true, 80u, 250, now, now, true);
    }

    car_inputs_t press_edge = make_debounced_inputs(31u, false, true, false);
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(51u, false, false, true);
    car_control_tick(&control, &config, &press_stable);
    CHECK_EQ("fresh press after release and three frames starts FOLLOW",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("manual PE7 edge marks session eligible for soft recovery",
             control.follow_session_active, 1u);
    CHECK_EQ("first FOLLOW tick starts at zero duty",
             control.follow_output_duty_permille, 0u);

    for (uint32_t step = 1u; step <= 43u; ++step) {
        const uint32_t now = 51u + step * 10u;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(9u + step), true, 80u, 400,
                      now - 1u, now, false);
        car_inputs_t tick = make_inputs(now, false);
        car_control_tick(&control, &config, &tick);
        const uint16_t expected = step < 50u
                                      ? static_cast<uint16_t>(step * 10u)
                                      : 500u;
        CHECK_EQ("FOLLOW duty rises by ten permille every ten ms",
                 control.follow_output_duty_permille, expected);
        CHECK(control.follow_output_duty_permille <= 500u,
              "FOLLOW base ramp never exceeds the 500 permille straight target");
        CHECK_EQ("FOLLOW software path remains separately gated",
                 control.road_drive_allowed, 1u);
    }

    send_frame_at(&control, &config, 53u, true, 80u, 800,
                  482u, 482u, false);
    const uint16_t positive_target = control.follow_servo_target_us;
    send_frame_at(&control, &config, 54u, true, 80u, -800,
                  483u, 483u, false);
    CHECK(positive_target > 1530u,
          "measured positive-right mapping steers toward the right endpoint");
    CHECK(control.follow_servo_target_us < 1530u,
          "negative error steers toward the left endpoint");
    CHECK(control.follow_servo_target_us >= config.follow_servo_left_us &&
              control.follow_servo_target_us <= config.follow_servo_right_us,
          "servo target stays inside synthetic asymmetric endpoints");

    car_config_t inverse = make_follow_config(true, true, 1620u, 1440u);
    car_control_t inverse_control{};
    start_follow_by_physical_pe7(&inverse_control, &inverse, 100u, 500u);
    send_frame_at(&inverse_control, &inverse, 103u, true, 80u, 800,
                  525u, 525u, false);
    CHECK(inverse_control.follow_servo_target_us < 1530u,
          "inverse asymmetric servo endpoints are honored without direction assumptions");
    send_frame_at(&inverse_control, &inverse, 104u, true, 80u, -800,
                  526u, 526u, false);
    CHECK(inverse_control.follow_servo_target_us > 1530u,
          "opposite error maps through the opposite measured endpoint");

    car_config_t wide = make_follow_config(true, true, 700u, 6000u);
    car_control_t wide_control{};
    start_follow_by_physical_pe7(&wide_control, &wide, 300u, 600u);
    send_frame_at(&wide_control, &wide, 303u, true, 80u, 1000,
                  625u, 625u, false);
    CHECK(wide_control.follow_servo_target_us > 1530u &&
              wide_control.follow_servo_target_us < wide.follow_servo_right_us,
          "wide synthetic servo span uses safe wide intermediate math");
    send_frame_at(&wide_control, &wide, 304u, true, 80u, -1000,
                  626u, 626u, false);
    CHECK(wide_control.follow_servo_target_us >= wide.follow_servo_left_us &&
              wide_control.follow_servo_target_us < 1530u,
          "wide negative mapping stays between left endpoint and centre");
}

void test_recovery_confidence_age_gap_wrap_and_bad_sequence()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);

    send_frame_at(&control, &config, 65535u, true, 60u, 0,
                  100u, 199u, true);
    CHECK_EQ("confidence 60 and queue age 99 ms count as first candidate",
             control.recovery_frame_count, 1u);
    send_frame_at(&control, &config, 0u, true, 60u, 0,
                  349u, 448u, true);
    CHECK_EQ("modulo sequence wrap and 249 ms frame gap remain consecutive",
             control.recovery_frame_count, 2u);
    send_frame_at(&control, &config, 1u, true, 60u, 0,
                  598u, 697u, true);
    CHECK_EQ("third q60/age99/gap249 observation completes recovery",
             control.recovery_active, 0u);
    CHECK_EQ("accepted camera sequence wraps through zero", control.recovery_sequence, 1u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 10u, true, 59u, 0,
                  100u, 100u, true);
    CHECK_EQ("valid confidence 59 counts as recovery candidate one",
             control.recovery_frame_count, 1u);
    CHECK_EQ("confidence alone does not request a flush boundary",
             control.rx_recovery_flush_required, 0u);
    send_frame_at(&control, &config, 11u, true, 60u, 0,
                  110u, 110u, true);
    send_frame_at(&control, &config, 12u, true, 60u, 0,
                  120u, 120u, true);
    CHECK_EQ("low and ordinary confidence valid frames recover consecutively",
             control.recovery_active, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 1u, true, 80u, 0,
                  200u, 300u, true);
    CHECK_EQ("queue age exactly 100 ms is rejected", control.recovery_frame_count, 0u);
    CHECK_EQ("age boundary raises a recoverable stale fault",
             control.fault, CAR_FAULT_RX_STALE);
    CHECK_EQ("age boundary keeps the recovery flush closed",
             control.rx_recovery_flush_required, 1u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 10u, true, 80u, 0,
                  100u, 100u, true);
    send_frame_at(&control, &config, 11u, true, 80u, 0,
                  350u, 350u, true);
    CHECK_EQ("250 ms inter-frame gap breaks the consecutive run",
             control.recovery_frame_count, 0u);
    CHECK_EQ("exact gap boundary remains stopped", control.follow_output_duty_permille, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 7u, true, 80u, 0,
                  1u, 1u, true);
    send_frame_at(&control, &config, 7u, true, 80u, 0,
                  2u, 2u, true);
    CHECK_EQ("duplicate sequence resets recovery observation",
             control.recovery_frame_count, 0u);
    CHECK_EQ("duplicate sequence requests a fresh flush boundary",
             control.rx_recovery_flush_required, 1u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 20u, true, 80u, 0,
                  10u, 10u, true);
    CHECK_EQ("first ordered recovery sequence becomes baseline",
             control.recovery_frame_count, 1u);
    send_frame_at(&control, &config, 19u, true, 80u, 0,
                  11u, 11u, true);
    CHECK_EQ("out-of-order recovery sequence discards candidate run",
             control.recovery_frame_count, 0u);
    CHECK_EQ("out-of-order recovery sequence requires another flush",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("out-of-order recovery sequence is diagnosed",
             control.fault, CAR_FAULT_SEQUENCE);

    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    send_frame_at(&control, &config, 50000u, true, 80u, 0,
                  1u, 1u, true);
    send_frame_at(&control, &config, 50001u, true, 80u, 0,
                  2u, 2u, true);
    send_frame_at(&control, &config, 50002u, true, 80u, 0,
                  3u, 3u, true);
    car_inputs_t timed_out = make_inputs(253u, true);
    car_control_tick(&control, &config, &timed_out);
    CHECK_EQ("vision outage stops after trusted observations age out",
             control.fault, CAR_FAULT_VISION_TIMEOUT);
    car_inputs_t camera_restarted = make_inputs(10004u, true);
    car_control_tick(&control, &config, &camera_restarted);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 100u, true, 80u, 0,
                  10005u, 10005u, true);
    send_frame_at(&control, &config, 101u, true, 80u, 0,
                  10015u, 10015u, true);
    send_frame_at(&control, &config, 102u, true, 80u, 0,
                  10025u, 10025u, true);
    CHECK_EQ("camera restart after ten seconds accepts a new stopped sequence baseline",
             control.recovery_active, 0u);
    CHECK_EQ("new camera sequence does not trigger old-sequence rejection",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("camera restart observation alone cannot start motors",
             control.follow_output_duty_permille, 0u);
}

void test_raw_rx_is_diagnostic_and_flush_preserves_new_uart_error()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_vision_rx_t rx{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_vision_rx_init(&rx);

    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = sequence;
        send_frame_at(&control, &config, sequence, true, 59u, 300,
                      now, now, true);
    }
    CHECK_EQ("CRC-valid low-confidence frame updates raw diagnostics",
             control.latest_rx_have_frame, 1u);
    CHECK_EQ("raw frame count includes CRC-valid observations",
             control.latest_rx_frame_count, 3u);
    CHECK_EQ("raw frame preserves valid flag", control.latest_rx_valid, 1u);
    CHECK_EQ("raw frame preserves confidence", control.latest_rx_confidence, 59u);
    CHECK_EQ("low-confidence valid frame can become trusted vision",
             control.have_valid_vision, 1u);
    CHECK_EQ("low-confidence valid error is retained for steering",
             control.vision_error_norm, 300);

    car_control_t invalid_status{};
    car_control_init(&invalid_status, &config, 0u, 0u, 0u);
    send_frame_at(&invalid_status, &config, 2u, false, 0u, 0,
                  2u, 2u, false);
    CHECK_EQ("CRC-valid valid=0 frame updates raw diagnostics",
             invalid_status.latest_rx_frame_count, 1u);
    CHECK_EQ("raw diagnostics preserve valid=0", invalid_status.latest_rx_valid, 0u);
    CHECK_EQ("invalid visual status never updates trusted steering",
             invalid_status.have_valid_vision, 0u);

    car_control_t parsed{};
    car_vision_rx_t parsed_rx{};
    car_control_init(&parsed, &config, 0u, 0u, 0u);
    car_vision_rx_init(&parsed_rx);
    enqueue_frame(&parsed_rx, make_wire_frame(20u, true, 80u, 300), 5u);
    poll_rx_current_snapshot(&parsed_rx, &parsed, &config, 5u, false);
    CHECK_EQ("complete CRC-valid frame reaches raw diagnostic state",
             parsed.latest_rx_frame_count, 1u);
    CHECK_EQ("valid CRC frame is not required to become trusted during boot recovery",
             parsed.have_valid_vision, 0u);
    std::array<uint8_t, VP_FRAME_SIZE> corrupt{};
    CHECK(pack_frame(make_wire_frame(21u, true, 80u, 400), &corrupt),
          "pack CRC corruption fixture");
    corrupt[10] ^= 0x01u;
    for (size_t index = 0u; index < corrupt.size(); ++index) {
        car_vision_rx_isr_byte(&parsed_rx, corrupt[index], 6u);
    }
    poll_rx_current_snapshot(&parsed_rx, &parsed, &config, 6u, false);
    CHECK_EQ("bad CRC does not update raw latest frame count",
             parsed.latest_rx_frame_count, 1u);
    CHECK_EQ("bad CRC revokes trusted steering until recovery", parsed.have_valid_vision, 0u);
    CHECK_EQ("CRC/parser error advances parser-owned RX generation",
             parsed_rx.fault_generation, 1u);
    CHECK_EQ("CRC/parser error generation is reported to control",
             parsed_rx.fault_generation_reported, 1u);

    std::array<uint8_t, VP_FRAME_SIZE> bad_format{};
    CHECK(pack_frame(make_wire_frame(22u, true, 90u, -250), &bad_format),
          "pack format rejection fixture");
    bad_format[2] = static_cast<uint8_t>(VP_VERSION + 1u);
    const uint16_t bad_format_crc = vp_crc16_ccitt_false(bad_format.data() + 2, 8u);
    bad_format[10] = static_cast<uint8_t>(bad_format_crc & 0xFFu);
    bad_format[11] = static_cast<uint8_t>(bad_format_crc >> 8u);
    car_control_t bad_format_control{};
    car_vision_rx_t bad_format_rx{};
    car_control_init(&bad_format_control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&bad_format_rx);
    for (size_t index = 0u; index < bad_format.size(); ++index) {
        car_vision_rx_isr_byte(&bad_format_rx, bad_format[index], 7u);
    }
    poll_rx_current_snapshot(&bad_format_rx, &bad_format_control,
                             &config, 7u, true);
    CHECK_EQ("unsupported frame format does not update raw count",
             bad_format_control.latest_rx_frame_count, 0u);

    car_control_t recovery{};
    car_vision_rx_t recovery_rx{};
    car_control_init(&recovery, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&recovery, &config, 0u);
    car_vision_rx_init(&recovery_rx);
    car_control_note_fault(&recovery, CAR_FAULT_UART);
    CHECK_EQ("soft fault increments main-owned fault generation",
             recovery.fault_generation, 1u);
    CHECK_EQ("soft RX fault increments main-owned RX fault generation",
             recovery.rx_fault_generation, 1u);
    CHECK_EQ("main soft-fault path does not increment ISR-owned generation",
             recovery.isr_switch_generation, 0u);
    CHECK_EQ("main soft-fault path does not increment USART ISR generation",
             recovery.isr_uart_generation, 0u);
    car_vision_rx_isr_error(&recovery_rx);
    CHECK_EQ("UART ISR advances only its ISR-owned RX generation",
             recovery_rx.isr_fault_generation, 1u);
    CHECK_EQ("UART ISR does not write parser-owned RX generation",
             recovery_rx.fault_generation, 0u);
    car_vision_rx_clear(&recovery_rx); /* Existing fault is part of this flush. */
    CHECK_EQ("flush snapshots existing UART ISR generation",
             recovery_rx.isr_fault_generation_reported, 1u);
    car_control_rx_recovery_flush_complete(&recovery);
    car_vision_rx_poll(&recovery_rx, &recovery, &config,
                       recovery_rx.head, 10u, 1u);
    CHECK_EQ("flush consumes the old UART error generation",
             recovery.recovery_frame_count, 0u);

    send_frame_at(&recovery, &config, 40u, true, 80u, 20,
                  12u, 12u, true);
    send_frame_at(&recovery, &config, 41u, true, 80u, 30,
                  13u, 13u, true);
    CHECK_EQ("two clean observations begin a candidate run after old error clear",
             recovery.recovery_frame_count, 2u);

    car_vision_rx_isr_error(&recovery_rx); /* New error after the flush boundary. */
    CHECK_EQ("post-flush UART error advances ISR generation again",
             recovery_rx.isr_fault_generation, 2u);
    car_vision_rx_poll(&recovery_rx, &recovery, &config,
                       recovery_rx.head, 11u, 1u);
    CHECK_EQ("fresh UART error after clear remains visible",
             recovery_rx.uart_fault, 1u);
    CHECK_EQ("post-flush ISR generation remains unreported until poll",
             recovery_rx.isr_fault_generation_reported, 2u);
    CHECK_EQ("fresh UART error resets recovery and requests another flush",
             recovery.rx_recovery_flush_required, 1u);
    CHECK_EQ("new same-source UART error resets recovery candidates",
             recovery.recovery_frame_count, 0u);
    CHECK_EQ("new same-source UART error advances main fault generation",
             recovery.fault_generation, 2u);
    CHECK_EQ("new same-source UART error advances RX fault generation",
             recovery.rx_fault_generation, 2u);
    CHECK_EQ("fresh UART error leaves outputs stopped",
             recovery.follow_output_duty_permille, 0u);

    car_control_t partial_control{};
    car_vision_rx_t partial_rx{};
    car_control_init(&partial_control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&partial_rx);
    std::array<uint8_t, VP_FRAME_SIZE> partial_frame{};
    CHECK(pack_frame(make_wire_frame(30u, true, 90u, 0), &partial_frame),
          "pack partial old-queue fixture");
    for (size_t index = 0u; index < 5u; ++index) {
        car_vision_rx_isr_byte(&partial_rx, partial_frame[index], 20u);
    }
    poll_rx_current_snapshot(&partial_rx, &partial_control, &config, 20u, true);
    CHECK_EQ("old partial frame is present before recovery flush",
             partial_rx.parser.length, 5u);
    car_control_note_fault(&partial_control, CAR_FAULT_UART);
    car_vision_rx_clear(&partial_rx);
    car_control_rx_recovery_flush_complete(&partial_control);
    CHECK_EQ("recovery flush clears partial parser bytes",
             partial_rx.parser.length, 0u);
    enqueue_frame(&partial_rx, make_wire_frame(31u, true, 90u, 0), 21u);
    poll_rx_current_snapshot(&partial_rx, &partial_control, &config, 21u, true);
    CHECK_EQ("old partial bytes cannot prefix the first post-flush frame",
             partial_control.latest_rx_sequence, 31u);
    CHECK_EQ("only complete post-flush frame is counted raw",
             partial_control.latest_rx_frame_count, 1u);

    car_control_t old_queue_control{};
    car_vision_rx_t old_queue_rx{};
    car_control_init(&old_queue_control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&old_queue_rx);
    car_control_note_fault(&old_queue_control, CAR_FAULT_UART);
    enqueue_frame(&old_queue_rx, make_wire_frame(400u, true, 90u, 0), 30u);
    car_vision_rx_clear(&old_queue_rx);
    car_control_rx_recovery_flush_complete(&old_queue_control);
    poll_rx_current_snapshot(&old_queue_rx, &old_queue_control, &config, 31u, true);
    CHECK_EQ("complete frame queued before recovery boundary is discarded",
             old_queue_control.latest_rx_frame_count, 0u);
}

void test_slope_hint_trust_recovery_and_hold()
{
    const car_config_t config = make_follow_config();
    car_control_t recovering{};
    car_vision_rx_t rx{};
    car_control_init(&recovering, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&recovering, &config, 0u);
    car_vision_rx_init(&rx);
    car_control_note_fault(&recovering, CAR_FAULT_UART);
    car_vision_rx_clear(&rx);
    car_control_rx_recovery_flush_complete(&recovering);

    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        enqueue_frame(&rx,
                      make_wire_frame(sequence, true, 0u, 0, true),
                      sequence);
        poll_rx_current_snapshot(&rx, &recovering, &config, sequence, true);
        CHECK_EQ("wire parser preserves the latest CRC-valid slope flag",
                 recovering.latest_rx_slope_hint, 1u);
        if (sequence < 3u) {
            CHECK_EQ("one or two recovery frames cannot authorize slope",
                     recovering.vision_slope_hint, 0u);
        }
    }
    CHECK_EQ("third fresh forward frame commits slope hint",
             recovering.vision_slope_hint, 1u);
    CHECK_EQ("slope hint adds no confidence threshold",
             recovering.vision_confidence, 0u);

    enqueue_frame(&rx, make_wire_frame(3u, true, 0u, 0, true), 4u);
    poll_rx_current_snapshot(&rx, &recovering, &config, 4u, true);
    CHECK_EQ("duplicate slope frame remains visible in raw diagnostics",
             recovering.latest_rx_slope_hint, 1u);
    CHECK_EQ("duplicate sequence clears the trusted slope hint",
             recovering.vision_slope_hint, 0u);
    CHECK_EQ("duplicate sequence faults the normal controller path",
             recovering.fault, CAR_FAULT_SEQUENCE);

    car_control_t held{};
    start_follow_by_physical_pe7(&held, &config, 400u, 0u);
    send_frame_at(&held, &config, 403u, true, 0u, 0, 25u, 25u, false,
                  true);
    CHECK_EQ("fresh slope observation is trusted during Follow",
             held.vision_slope_hint, 1u);
    car_control_note_applied_outputs(&held, 1u, 1u, 1530u, 500u, 500u);
    send_frame_at(&held, &config, 404u, false, 0u, 0, 26u, 26u, false);
    CHECK_EQ("invalid frame clears trusted slope before entering hold",
             held.vision_slope_hint, 0u);
    CHECK_EQ("invalid no-hint frame enters the existing hold path",
             held.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    send_frame_at(&held, &config, 405u, true, 0u, 0, 27u, 27u, false,
                  true);
    send_frame_at(&held, &config, 406u, true, 0u, 0, 28u, 28u, false,
                  true);
    CHECK_EQ("first two hold recovery frames do not commit slope",
             held.vision_slope_hint, 0u);
    send_frame_at(&held, &config, 407u, true, 0u, 0, 29u, 29u, false,
                  true);
    CHECK_EQ("third hold recovery frame commits its fresh slope hint",
             held.vision_slope_hint, 1u);
    CHECK_EQ("hold recovery preserves the established session",
             held.mode, CAR_MODE_FOLLOW_RUNNING);
}

void test_follow_resume_is_session_scoped_and_pe7_isr_cancels()
{
    const car_config_t config = make_follow_config();
    car_control_t uart_only{};
    start_follow_by_physical_pe7(&uart_only, &config, 90u, 0u);
    car_control_note_fast_stop_isr(&uart_only, CAR_FAULT_UART);
    CHECK_EQ("UART-only fast stop preserves the manual Follow session",
             uart_only.follow_session_active, 1u);
    const car_fault_t uart_only_fault =
        car_control_take_fast_stop_isr(&uart_only);
    car_control_note_fault(&uart_only, uart_only_fault);
    CHECK_EQ("UART-only soft fault remains in session recovery",
             uart_only.follow_session_active, 1u);
    CHECK_EQ("UART-only soft fault waits for eligible recovery",
             uart_only.mode, CAR_MODE_RECOVERY_WAIT);

    car_control_t control{};
    start_follow_by_physical_pe7(&control, &config, 100u, 0u);
    CHECK_EQ("test setup starts a manually initiated FOLLOW session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    send_frame_at(&control, &config, 103u, true, 80u, 200,
                  34u, 34u, false);
    car_inputs_t ramp = make_inputs(34u, false);
    car_control_tick(&control, &config, &ramp);
    CHECK_EQ("manual FOLLOW has begun its first limited ramp step",
             control.follow_output_duty_permille, 10u);

    car_control_note_fault(&control, CAR_FAULT_UART);
    CHECK_EQ("soft fault immediately clears follow duty",
             control.follow_output_duty_permille, 0u);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 104u, true, 80u, 400,
                  44u, 44u, false);
    send_frame_at(&control, &config, 105u, true, 80u, 500,
                  54u, 54u, false);
    CHECK_EQ("two recovery frames do not resume FOLLOW",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    send_frame_at(&control, &config, 106u, true, 80u, 600,
                  64u, 64u, false);
    CHECK_EQ("three trusted frames wait for a new reverse KEEP lease",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("reverse lease wait keeps road output closed",
             control.road_drive_allowed, 0u);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 65u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(107u + offset),
                      true, 80u, static_cast<int16_t>(700u + offset),
                      now, now, false);
    }
    CHECK_EQ("three fresh post-KEEP frames resume only prior Follow session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("automatic resume starts the output ramp from zero",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("automatic resume count increments exactly once",
             control.auto_recovery_count, 1u);
    CHECK_EQ("latest post-KEEP error replaces pre-fault steering value",
             control.vision_error_norm, 702);

    car_control_note_fault(&control, CAR_FAULT_UART);
    car_control_rx_recovery_flush_complete(&control);
    car_control_note_switch_release_isr(&control);
    car_inputs_t transient_press = make_inputs(68u, false);
    car_control_tick(&control, &config, &transient_press);
    CHECK_EQ("release ISR between ticks cancels a re-press before main observes high",
             control.follow_session_active, 0u);
    CHECK_EQ("ISR release leaves FOLLOW output zero",
             control.follow_output_duty_permille, 0u);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 69u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(110u + offset),
                      true, 80u, 0, now, now, false);
    }
    CHECK_EQ("three frames cannot auto-resume after PE7 release canceled session",
             control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("release ISR cause remains latched for the main loop",
             control.isr_release_seen, 1u);
    CHECK_EQ("PE7 release advances only the PE7 ISR generation",
             control.isr_switch_generation, 1u);
    CHECK_EQ("PE7 release does not write the USART ISR generation",
             control.isr_uart_generation, 0u);

    car_control_t uart_isr_probe{};
    car_control_init(&uart_isr_probe, &config, 0u, 1u, 0u);
    car_control_note_fast_stop_isr(&uart_isr_probe, CAR_FAULT_UART);
    CHECK_EQ("USART fast-stop event advances the USART-owned control generation",
             uart_isr_probe.isr_uart_generation, 1u);
    CHECK_EQ("USART fast-stop event leaves PE7 generation unchanged",
             uart_isr_probe.isr_switch_generation, 0u);

    car_control_t stopped{};
    start_follow_by_physical_pe7(&stopped, &config, 200u, 100u);
    car_inputs_t high = make_inputs(125u, true);
    const uint32_t effects = car_control_command(
        &stopped, &config, &high, CAR_CMD_NORMAL_STOP,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "normal STOP requests output disable");
    CHECK_EQ("normal STOP cancels FOLLOW session",
             stopped.follow_session_active, 0u);
    car_control_rx_recovery_flush_complete(&stopped);
    send_frame_at(&stopped, &config, 203u, true, 80u, 0,
                  126u, 126u, true);
    send_frame_at(&stopped, &config, 204u, true, 80u, 0,
                  127u, 127u, true);
    send_frame_at(&stopped, &config, 205u, true, 80u, 0,
                  128u, 128u, true);
    CHECK_EQ("normal STOP cannot be undone by fresh frames",
             stopped.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
}

void test_uart_fifo_service_batches_errors_and_follow_recovery()
{
    auto drain = [](UartFifoMock *mock) {
        const car_uart_rx_service_io_t io = uart_mock_io(mock);
        return car_uart_rx_service_drain(&io);
    };

    UartFifoMock two{};
    two.fifo = {0x11u, 0x12u};
    CHECK_EQ("two queued RX bytes are drained in one service call",
             drain(&two).bytes_drained, 2u);
    CHECK_EQ("two-byte batch preserves input order", two.accepted.size(), 2u);
    CHECK_EQ("two-byte batch keeps first byte", two.accepted[0], 0x11u);
    CHECK_EQ("two-byte batch keeps second byte", two.accepted[1], 0x12u);

    UartFifoMock twelve{};
    const vp_frame expected_frame = make_wire_frame(73u, true, 91u, -327);
    std::array<uint8_t, VP_FRAME_SIZE> expected_wire{};
    CHECK_EQ("valid V1 frame packs for FIFO ordering test",
             vp_pack(&expected_frame, expected_wire.data()), 1u);
    twelve.fifo.assign(expected_wire.begin(), expected_wire.end());
    CHECK_EQ("twelve queued RX bytes are drained in one service call",
             drain(&twelve).bytes_drained, 12u);
    CHECK_EQ("twelve-byte batch preserves every byte", twelve.accepted.size(), 12u);
    vp_frame decoded_frame{};
    CHECK_EQ("drained bytes match packed V1 wire order",
             twelve.accepted.size() == expected_wire.size() &&
                 std::equal(twelve.accepted.begin(), twelve.accepted.end(),
                            expected_wire.begin()),
             1u);
    CHECK_EQ("drained twelve-byte FIFO payload passes CRC unpack",
             vp_unpack(twelve.accepted.data(), &decoded_frame), 1u);
    CHECK_EQ("FIFO parser input retains frame sequence", decoded_frame.sequence,
             expected_frame.sequence);
    CHECK_EQ("FIFO parser input retains signed error", decoded_frame.lateral_error,
             expected_frame.lateral_error);

    UartFifoMock arrival{};
    for (uint8_t index = 0u; index < 12u; ++index) {
        arrival.fifo.push_back(index);
    }
    arrival.inject_after_reads = 4u;
    arrival.injected_bytes = {12u, 13u, 14u, 15u};
    const car_uart_rx_service_result_t arrival_result = drain(&arrival);
    CHECK_EQ("bytes arriving during service are drained in the same bounded batch",
             arrival_result.bytes_drained, 16u);
    CHECK_EQ("arrival during service preserves all sixteen bytes",
             arrival.accepted.size(), 16u);
    std::vector<uint8_t> expected_arrival_order{};
    for (uint8_t index = 0u; index < 16u; ++index) {
        expected_arrival_order.push_back(index);
    }
    CHECK(std::equal(arrival.accepted.begin(), arrival.accepted.end(),
                     expected_arrival_order.begin()),
          "bytes arriving during the batch retain FIFO order");
    CHECK_EQ("full FIFO-sized pass reports bounded-budget exhaustion",
             arrival_result.budget_exhausted, 1u);

    UartFifoMock reentrant{};
    for (uint8_t index = 0u; index < 12u; ++index) {
        reentrant.fifo.push_back(index);
    }
    reentrant.inject_after_reads = 4u;
    reentrant.injected_bytes = {12u, 13u, 14u, 15u,
                                16u, 17u, 18u, 19u};
    const car_uart_rx_service_result_t first_batch = drain(&reentrant);
    CHECK_EQ("first pass is capped at physical FIFO depth",
             first_batch.bytes_drained, CAR_UART_RX_SERVICE_MAX_BATCH);
    CHECK_EQ("pending FIFO data remains for level-triggered re-entry",
             reentrant.fifo.size() - reentrant.cursor, 4u);
    CHECK_EQ("second service entry drains the remaining pending bytes",
             drain(&reentrant).bytes_drained, 4u);
    CHECK_EQ("re-entry eventually consumes all queued bytes",
             reentrant.accepted.size(), 20u);
    std::vector<uint8_t> expected_reentrant_order{};
    for (uint8_t index = 0u; index < 20u; ++index) {
        expected_reentrant_order.push_back(index);
    }
    CHECK(std::equal(reentrant.accepted.begin(), reentrant.accepted.end(),
                     expected_reentrant_order.begin()),
          "bounded re-entry preserves the entire FIFO stream");

    UartFifoMock error_at_entry{};
    error_at_entry.fifo = {1u, 2u, 3u, 4u};
    error_at_entry.error_flags = UART_MOCK_ORE;
    const car_uart_rx_service_result_t entry_error = drain(&error_at_entry);
    CHECK_EQ("ORE is handled before any queued byte reaches the parser",
             entry_error.bytes_drained, 0u);
    CHECK_EQ("ORE invokes one immediate error/stop callback",
             error_at_entry.error_calls, 1u);
    CHECK_EQ("ORE callback disables RX service and flushes old FIFO data",
             error_at_entry.flush_calls, 1u);
    CHECK_EQ("ORE path does not accept stale FIFO bytes",
             error_at_entry.accepted.size(), 0u);

    UartFifoMock error_during_batch{};
    error_during_batch.fifo = {1u, 2u, 3u, 4u, 5u};
    error_during_batch.error_after_reads = 2u;
    const car_uart_rx_service_result_t mid_error = drain(&error_during_batch);
    CHECK_EQ("new ORE is checked between FIFO reads",
             mid_error.stopped_for_error, 1u);
    CHECK_EQ("bytes behind a newly observed ORE are never accepted",
             error_during_batch.accepted.size(), 2u);
    CHECK_EQ("mid-batch ORE discards the remaining old FIFO contents",
             error_during_batch.fifo.size(), 0u);

    UartFifoMock overflow{};
    overflow.fifo = {0u, 1u, 2u, 3u, 4u, 5u};
    overflow.abort_after_accepted = 3u;
    const car_uart_rx_service_result_t overflow_result = drain(&overflow);
    CHECK_EQ("software RX queue overflow aborts the bounded drain",
             overflow_result.stopped_for_abort, 1u);
    CHECK_EQ("overflow does not consume bytes after the first faulting point",
             overflow.accepted.size(), 3u);
    CHECK_EQ("overflow path clears remaining peripheral FIFO data",
             overflow.fifo.size(), 0u);

    const car_config_t config = make_follow_config();
    car_control_t control{};
    start_follow_by_physical_pe7(&control, &config, 800u, 100u);
    CHECK_EQ("integration fixture begins with a manual Follow session",
             control.follow_session_active, 1u);
    UartFifoMock controller_ore{};
    controller_ore.control = &control;
    controller_ore.fifo = {0xA5u, 0x5Au, 0x01u, 0x02u};
    controller_ore.error_flags = UART_MOCK_ORE;
    const car_uart_rx_service_result_t controller_error = drain(&controller_ore);
    CHECK_EQ("actual shared service reports the captured ORE",
             controller_error.stopped_for_error, 1u);
    const car_fault_t uart_fault = car_control_take_fast_stop_isr(&control);
    CHECK_EQ("ORE stop latch retains UART fault identity", uart_fault,
             CAR_FAULT_UART);
    car_control_note_fault(&control, uart_fault);
    CHECK_EQ("UART-only ORE parks outputs without cancelling Follow session",
             control.follow_session_active, 1u);
    CHECK_EQ("UART-only ORE enters soft recovery, not hard lock",
             control.fault_latched, 0u);
    CHECK_EQ("UART-only ORE keeps road drive closed during recovery",
             control.road_drive_allowed, 0u);
    car_control_rx_recovery_flush_complete(&control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 130u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(803u + offset), true, 80u,
                      static_cast<int16_t>(offset), now, now, false);
    }
    CHECK_EQ("three fresh frames alone do not bypass reverse KEEP recovery",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("recovery still owns the prior Follow session",
             control.follow_session_active, 1u);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t offset = 0u; offset < 3u; ++offset) {
        const uint32_t now = 140u + offset;
        send_frame_at(&control, &config,
                      static_cast<uint16_t>(806u + offset), true, 80u,
                      static_cast<int16_t>(offset), now, now, false);
    }
    CHECK_EQ("UART-only ORE resumes only after KEEP and three more new frames",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
}

void test_application_follow_profile_recovers_and_runs_past_bench_limit()
{
    car_config_t config{};
    car_config_init_application(&config, 0u, 0u, 1u, 1u);
    CHECK_EQ("application profile compiles Follow", config.follow_mode_compiled, 1u);
    CHECK_EQ("application profile enables measured Follow calibration",
             config.follow_calibration_ready, 1u);
    CHECK_EQ("application servo left endpoint", config.follow_servo_left_us, 1100u);
    CHECK_EQ("application servo center", config.servo_center_us, 1530u);
    CHECK_EQ("application servo right endpoint", config.follow_servo_right_us, 2500u);
    CHECK_EQ("application left encoder calibration",
             config.follow_left_encoder_counts_per_rev, 59995u);
    CHECK_EQ("application right encoder calibration",
             config.follow_right_encoder_counts_per_rev, 60239u);
    CHECK_EQ("application left encoder sign", config.follow_left_encoder_sign, 1);
    CHECK_EQ("application right encoder sign", config.follow_right_encoder_sign, -1);
    CHECK_EQ("application Follow normal output ceiling is 60 percent",
             config.follow_max_duty_permille, 600u);
    CHECK_EQ("production Bench retains its separate 5.5 second limit",
             config.bench_run_limit_ms, 5500u);

    car_control_t unstarted{};
    car_control_init(&unstarted, &config, 0u, 0u, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(&unstarted, &config, &released);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame_at(&unstarted, &config, sequence, true, 80u, 0,
                      sequence, sequence, true);
    }
    CHECK_EQ("production Follow frames cannot auto-start without a new PE7 press",
             unstarted.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("production Follow remains unstarted until manual press",
             unstarted.follow_session_active, 0u);

    car_control_t control{};
    start_follow_by_physical_pe7(&control, &config, 100u, 100u);
    CHECK_EQ("production profile starts only after the manual PE7 edge",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("manual production run owns an active session",
             control.follow_session_active, 1u);

    car_control_note_fault(&control, CAR_FAULT_VISION_TIMEOUT);
    CHECK_EQ("production soft vision timeout immediately zeros Follow duty",
             control.follow_output_duty_permille, 0u);
    car_control_rx_recovery_flush_complete(&control);
    send_frame_at(&control, &config, 1u, true, 80u, 400,
                  10135u, 10135u, false);
    send_frame_at(&control, &config, 2u, true, 80u, 400,
                  10145u, 10145u, false);
    send_frame_at(&control, &config, 3u, true, 80u, 400,
                  10155u, 10155u, false);
    CHECK_EQ("production Follow waits for sideband KEEP after trusted recovery",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("pre-KEEP recovery frames do not reopen road drive",
             control.road_drive_allowed, 0u);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t sequence = 4u; sequence <= 6u; ++sequence) {
        const uint32_t now = 10155u + (uint32_t)(sequence - 3u) * 10u;
        send_frame_at(&control, &config, sequence, true, 80u, 400,
                      now, now, false);
    }
    CHECK_EQ("production Follow resumes only after three new post-KEEP frames",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("production recovery retains only the prior manual session",
             control.follow_session_active, 1u);
    CHECK_EQ("production auto-resume begins with a zero motor ramp",
             control.follow_output_duty_permille, 0u);
    CHECK(control.follow_servo_target_us > config.servo_center_us,
          "production recovery uses the latest servo target before motor ramp");

    for (uint16_t index = 1u; index <= 60u; ++index) {
        const uint32_t now = 10185u + (uint32_t)index * 100u;
        const uint16_t sequence = static_cast<uint16_t>(6u + index);
        send_frame_at(&control, &config, sequence, true, 80u, 400,
                      now, now, false);
        car_inputs_t active = make_inputs(now, false);
        car_control_tick(&control, &config, &active);
    }
    CHECK_EQ("production Follow stays active beyond Bench's 5.5 second limit",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("production Follow base output reaches the straight 500 target",
             control.follow_output_duty_permille, 500u);
    CHECK_EQ("production Follow retains road-drive permission during valid operation",
             control.road_drive_allowed, 1u);

    car_control_note_switch_release_isr(&control);
    car_inputs_t released_switch = make_inputs(16156u, true);
    car_control_tick(&control, &config, &released_switch);
    CHECK_EQ("production PE7 release ends the continuous Follow session",
             control.follow_session_active, 0u);
    CHECK_EQ("production PE7 release immediately removes Follow duty",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("production PE7 release removes road-drive permission",
             control.road_drive_allowed, 0u);
}

void test_debug_and_hard_faults_never_auto_enable_follow()
{
    const car_config_t config = make_follow_config();
    car_config_t debug = make_config(false, false);
    car_control_t control{};
    car_control_init(&control, &debug, 0u, 1u, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(&control, &debug, &released);
    send_frame_at(&control, &debug, 1u, true, 80u, 0, 1u, 1u, true);
    send_frame_at(&control, &debug, 2u, true, 80u, 0, 2u, 2u, true);
    send_frame_at(&control, &debug, 3u, true, 80u, 0, 3u, 3u, true);
    car_inputs_t press = make_inputs(4u, false);
    car_control_tick(&control, &debug, &press);
    CHECK_EQ("Debug profile cannot enter FOLLOW even with valid frames and PE7",
             control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("Debug profile follow output stays zero",
             control.follow_output_duty_permille, 0u);

    car_config_t uncalibrated = make_follow_config(false);
    car_control_init(&control, &uncalibrated, 0u, 1u, 0u);
    car_control_rx_recovery_flush_complete(&control);
    car_inputs_t uncalibrated_initial_release = make_inputs(0u, true);
    car_control_tick(&control, &uncalibrated, &uncalibrated_initial_release);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = sequence;
        send_frame_at(&control, &uncalibrated, sequence, true, 90u, 0,
                      now, now, true);
    }
    car_inputs_t uncalibrated_press_edge = make_debounced_inputs(
        4u, false, true, false);
    car_control_tick(&control, &uncalibrated, &uncalibrated_press_edge);
    car_inputs_t uncalibrated_press_stable = make_debounced_inputs(
        24u, false, false, true);
    car_control_tick(&control, &uncalibrated, &uncalibrated_press_stable);
    car_inputs_t uncalibrated_release_edge = make_debounced_inputs(
        25u, true, false, true);
    car_control_tick(&control, &uncalibrated, &uncalibrated_release_edge);
    car_inputs_t uncalibrated_release_stable = make_debounced_inputs(
        45u, true, true, false);
    car_control_tick(&control, &uncalibrated, &uncalibrated_release_stable);
    CHECK_EQ("missing endpoint/direction/encoder calibration latches CONFIG",
             control.fault, CAR_FAULT_CONFIG);
    CHECK_EQ("physical press-release ACK and fresh vision cannot bypass missing calibration",
             control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    CHECK_EQ("invalid calibration blocks physical hard-fault acknowledgement",
             control.fault_latched, 1u);
    CHECK_EQ("uncalibrated follow output remains zero",
             control.follow_output_duty_permille, 0u);

    const car_fault_t hard_faults[] = {
        CAR_FAULT_WATCHDOG_RESET,
        CAR_FAULT_LOOP_GAP,
        CAR_FAULT_ENCODER_INIT,
        CAR_FAULT_CLOCK,
        CAR_FAULT_TIMER,
        CAR_FAULT_CONFIG,
        CAR_FAULT_WATCHDOG_INIT
    };
    for (car_fault_t hard_fault : hard_faults) {
        car_control_init(&control, &config, 0u, 1u, 0u);
        car_control_note_fault(&control, hard_fault);
        car_control_rx_recovery_flush_complete(&control);
        for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
            const uint32_t now = sequence;
            send_frame_at(&control, &config, sequence, true, 90u, 0,
                          now, now, true);
        }
        CHECK_EQ("hard-priority fault remains latched through recovery frames",
                 control.fault_latched, 1u);
        CHECK_EQ("hard-priority fault blocks FOLLOW output",
                 control.follow_output_duty_permille, 0u);
        CHECK_EQ("hard-priority fault blocks session auto-resume",
                 control.mode == CAR_MODE_FOLLOW_RUNNING, 0u);
    }

    car_control_init(&control, &config, 0u, 1u, 1u);
    CHECK_EQ("watchdog reset is latched at boot", control.fault,
             CAR_FAULT_WATCHDOG_RESET);
    car_control_rx_recovery_flush_complete(&control);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = sequence;
        send_frame_at(&control, &config, sequence, true, 90u, 0,
                      now, now, true);
    }
    CHECK_EQ("watchdog reset cannot clear automatically from vision",
             control.fault_latched, 1u);
    CHECK_EQ("watchdog reset cannot enable FOLLOW output",
             control.follow_output_duty_permille, 0u);
}

void test_hard_ack_is_rejected_while_source_remains_active()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_control_note_fault(&control, CAR_FAULT_LOOP_GAP);

    car_inputs_t press_edge = make_debounced_inputs(1u, false, true, false);
    press_edge.loop_gap_fault = 1u;
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(21u, false, false, true);
    press_stable.loop_gap_fault = 1u;
    car_control_tick(&control, &config, &press_stable);

    car_control_note_switch_release_isr(&control);
    car_inputs_t release_edge = make_debounced_inputs(22u, true, false, true);
    release_edge.loop_gap_fault = 1u;
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t release_stable = make_debounced_inputs(42u, true, true, false);
    release_stable.loop_gap_fault = 1u;
    car_control_tick(&control, &config, &release_stable);

    CHECK_EQ("physical ACK cannot clear loop-gap fault while source is active",
             control.fault, CAR_FAULT_LOOP_GAP);
    CHECK_EQ("hard fault remains latched after debounced ACK attempt",
             control.fault_latched, 1u);
    CHECK_EQ("hard ACK attempt keeps recovery boundary closed",
             control.recovery_active, 0u);
    CHECK_EQ("hard ACK attempt keeps FOLLOW output stopped",
             control.follow_output_duty_permille, 0u);
}

void test_hard_fault_requires_physical_ack_three_frames_and_new_press()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_inputs_t released = make_inputs(0u, true);
    car_control_tick(&control, &config, &released);
    car_control_note_fault(&control, CAR_FAULT_LOOP_GAP);

    const uint32_t local_ack = car_control_command(
        &control, &config, &released, CAR_CMD_LOCAL_ACK,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((local_ack & CAR_EFFECT_CLEAR_RX) == 0u,
          "legacy local ACK command cannot clear a hard latch");
    const uint32_t camera_ack = car_control_command(
        &control, &config, &released, CAR_CMD_CAMERA_RESTART_ACK,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((camera_ack & CAR_EFFECT_CLEAR_RX) == 0u,
          "camera restart command cannot clear a hard latch");
    CHECK_EQ("command ACK attempts keep hard fault latched",
             control.fault_latched, 1u);

    car_inputs_t press_edge = make_debounced_inputs(1u, false, true, false);
    car_control_tick(&control, &config, &press_edge);
    car_inputs_t press_stable = make_debounced_inputs(21u, false, false, true);
    car_control_tick(&control, &config, &press_stable);
    CHECK_EQ("stable physical press arms hard-fault ACK",
             control.hard_ack_press_seen, 1u);
    car_inputs_t release_edge = make_debounced_inputs(22u, true, false, true);
    car_control_tick(&control, &config, &release_edge);
    car_inputs_t release_stable = make_debounced_inputs(42u, true, true, false);
    car_control_tick(&control, &config, &release_stable);
    CHECK_EQ("physical press-release clears the now-absent hard cause",
             control.fault_latched, 0u);
    CHECK_EQ("physical ACK still waits in recovery and requests RX flush",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("physical ACK does not auto-start FOLLOW",
             control.mode, CAR_MODE_RECOVERY_WAIT);

    car_control_rx_recovery_flush_complete(&control);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        const uint32_t now = 42u + sequence;
        send_frame_at(&control, &config, sequence, true, 80u, 100,
                      now, now, true);
    }
    CHECK_EQ("three post-ACK frames recover observation only",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("post-ACK frames do not restore a prior session",
             control.follow_session_active, 0u);
    CHECK_EQ("post-ACK frames keep FOLLOW output stopped",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("hard ACK and its three frames do not count as auto recovery",
             control.auto_recovery_count, 0u);

    car_inputs_t separate_press_edge = make_debounced_inputs(
        46u, false, true, false);
    car_control_tick(&control, &config, &separate_press_edge);
    car_inputs_t separate_press_stable = make_debounced_inputs(
        66u, false, false, true);
    car_control_tick(&control, &config, &separate_press_stable);
    CHECK_EQ("separate post-recovery PE7 press starts a new FOLLOW session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("new hard-fault-cleared session starts from zero output",
             control.follow_output_duty_permille, 0u);
}

void test_invalid_quality_and_sequence_rejections()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame(&control, &config, sequence, true, 59u, 0,
                   sequence, true);
    }
    CHECK_EQ("valid frame is accepted without a numeric confidence gate",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("low confidence remains diagnostic and does not increment invalid count",
             control.invalid_vision_count, 0u);
    CHECK_EQ("valid low-confidence frame is trusted", control.have_valid_vision, 1u);
    CHECK_EQ("low confidence leaves road disabled", control.road_drive_allowed, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    send_frame(&control, &config, 1u, false, 0u, 0, 1u, true);
    CHECK_EQ("idle valid=0 is a hold-eligible observation, not a reset request",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("invalid visual status does not retain usable vision",
             control.have_valid_vision, 0u);
    CHECK_EQ("idle invalid visual status cannot authorize drive",
             control.road_drive_allowed, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    send_frame(&control, &config, 7u, true, 70u, 0, 1u, true);
    send_frame(&control, &config, 7u, true, 70u, 0, 2u, true);
    CHECK_EQ("repeated sequence rejected", control.fault, CAR_FAULT_SEQUENCE);
    CHECK_EQ("repeated sequence increments diagnostic", control.sequence_error_count, 1u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    send_frame(&control, &config, 20u, true, 70u, 0, 1u, true);
    send_frame(&control, &config, 19u, true, 70u, 0, 2u, true);
    CHECK_EQ("descending sequence rejected", control.fault, CAR_FAULT_SEQUENCE);

    car_control_init(&control, &config, 0u, 1u, 0u);
    send_frame(&control, &config, 65535u, true, 70u, 0, 1u, true);
    send_frame(&control, &config, 0u, true, 70u, 0, 2u, true);
    send_frame(&control, &config, 1u, true, 70u, 0, 3u, true);
    CHECK_EQ("normal uint16 sequence wrap accepted", control.fault, CAR_FAULT_NONE);
    CHECK_EQ("sequence wrap becomes trusted after the third fresh frame",
             control.vision_sequence, 1u);
    CHECK_EQ("sequence wrap recovery requires three accepted frames",
             control.rx_frame_count, 3u);
}

void test_exact_vision_timeout_and_late_frame()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    start_follow_by_physical_pe7(&control, &config, 90u, 0u);
    send_frame_at(&control, &config, 93u, true, 80u, 0, 10u, 10u, false);
    send_frame_at(&control, &config, 94u, true, 80u, 0, 20u, 20u, false);
    send_frame_at(&control, &config, 95u, true, 80u, 0, 30u, 30u, false);
    CHECK_EQ("vision remains fresh one ms before timeout",
             car_control_vision_fresh(&control, 279u, config.vision_timeout_ms), 1u);
    CHECK_EQ("vision is stale at exact 250 ms boundary",
             car_control_vision_fresh(&control, 280u, config.vision_timeout_ms), 0u);

    car_inputs_t timeout = make_inputs(280u, false);
    car_control_tick(&control, &config, &timeout);
    CHECK_EQ("no-frame timeout stops at exact 250 ms", control.fault,
             CAR_FAULT_VISION_TIMEOUT);
    CHECK_EQ("timeout stops/keeps motor disabled", control.bench_output_duty_permille, 0u);
    CHECK_EQ("vision timeout is recoverable after a clean UART boundary",
             control.fault_latched, 0u);

    start_follow_by_physical_pe7(&control, &config, 190u, 0u);
    send_frame_at(&control, &config, 193u, true, 80u, 0, 10u, 10u, false);
    send_frame_at(&control, &config, 194u, true, 80u, 0, 20u, 20u, false);
    send_frame_at(&control, &config, 195u, true, 80u, 0, 30u, 30u, false);
    send_frame_at(&control, &config, 196u, true, 80u, 0, 280u, 280u, false);
    CHECK_EQ("late new frame faults before it can renew permission",
             control.fault, CAR_FAULT_VISION_TIMEOUT);
    CHECK_EQ("late frame cannot erase the recoverable timeout boundary",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("late new frame does not become a hard latch", control.fault_latched, 0u);
    CHECK_EQ("late frame cannot restore trusted vision", control.have_valid_vision, 0u);
    CHECK_EQ("late frame remains invalid after timeout fault", control.vision_valid, 0u);
    CHECK_EQ("late new frame does not authorize motor", control.bench_output_duty_permille, 0u);
}

void test_follow_toggle_drives_manual_controller_session()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_manual_button_t button{};
    car_manual_button_init(&button, 0u, 0u);
    car_control_init(&control, &config, 0u, 1u, 0u);

    car_manual_button_poll_high(&button, 1u, 0u);
    car_manual_button_poll_high(&button, 1u, 19u);
    CHECK_EQ("button startup release is not confirmed before 20 ms",
             button.startup_release_confirmed, 0u);
    car_manual_button_poll_high(&button, 1u, 20u);
    CHECK_EQ("button and controller share the 20 ms startup release",
             button.startup_release_confirmed, 1u);

    car_inputs_t released = make_inputs(20u, true);
    released.manual_button_startup_release_confirmed =
        button.startup_release_confirmed;
    released.manual_run_request = button.run_request;
    car_control_tick(&control, &config, &released);
    car_control_rx_recovery_flush_complete(&control);
    car_control_reverse_control_reset_phase_complete(&control);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame_at(&control, &config, sequence, true, 80u, 0,
                      20u + sequence, 20u + sequence, true);
    }
    CHECK_EQ("camera frames alone cannot start a manual Follow session",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    CHECK_EQ("camera frames alone leave manual request off",
             control.manual_run_request, 0u);
    CHECK_EQ("camera frames alone leave Follow session inactive",
             control.follow_session_active, 0u);

    CHECK_EQ("first real low edge latches the manual start request",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_START);
    car_inputs_t pressed = make_inputs(24u, false);
    pressed.manual_run_request = button.run_request;
    pressed.manual_button_startup_release_confirmed =
        button.startup_release_confirmed;
    car_control_tick(&control, &config, &pressed);
    car_manual_button_start_consumed(&button);
    CHECK_EQ("button request starts Follow after all route and KEEP gates",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("started controller records the manual request",
             control.manual_run_request, 1u);
    CHECK_EQ("start begins with the existing zero motor ramp",
             control.follow_output_duty_permille, 0u);

    car_manual_button_poll_high(&button, 1u, 30u);
    car_manual_button_poll_high(&button, 1u, 49u);
    CHECK_EQ("release under 20 ms does not rearm the stop toggle",
             button.armed, 0u);
    CHECK_EQ("release leaves the latched manual request on",
             button.run_request, 1u);
    car_manual_button_poll_high(&button, 1u, 50u);
    car_inputs_t released_again = make_inputs(50u, true);
    released_again.manual_run_request = button.run_request;
    released_again.manual_button_startup_release_confirmed =
        button.startup_release_confirmed;
    car_control_tick(&control, &config, &released_again);
    CHECK_EQ("20 ms release does not stop the active Follow session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);

    CHECK_EQ("second armed low edge requests a stop",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_STOP);
    CHECK_EQ("stop edge immediately revokes the request",
             button.run_request, 0u);
    car_inputs_t stopped = make_inputs(51u, false);
    stopped.manual_run_request = button.run_request;
    const uint32_t effects = car_control_command(
        &control, &config, &stopped, CAR_CMD_NORMAL_STOP,
        CAR_MOTOR_NONE, CAR_MOTOR_IN1, 0u);
    CHECK((effects & CAR_EFFECT_STOP_OUTPUTS) != 0u,
          "second press uses the normal stop effects");
    CHECK((effects & CAR_EFFECT_RESET_PARSER) != 0u,
          "second press clears cached visual state");
    car_control_note_applied_outputs(&control, 0u, 0u, 0u, 0u, 0u);
    car_manual_button_stop_consumed(&button);
    CHECK_EQ("second press cancels the session", control.follow_session_active, 0u);
    CHECK_EQ("second press returns to stopped recovery", control.mode,
             CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("stopped request cannot restart from the same low level",
             control.manual_run_request, 0u);
}

void test_manual_follow_start_restarts_reverse_resync()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_manual_button_t button{};
    start_follow_by_physical_pe7(&control, &config, 40u, 100u);
    CHECK_EQ("fixture begins with a running stale Follow session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("fixture has the old local KEEP gate", control.reverse_control_keep_seen, 1u);
    CHECK_EQ("fixture has old local fresh observations",
             control.reverse_control_fresh_frames,
             CAR_CONTROL_RECOVERY_REQUIRED_FRAMES);

    car_manual_button_init(&button, 1u, 0u);
    car_manual_button_poll_high(&button, 1u, 20u);
    CHECK_EQ("one armed PE7 low edge creates one physical start event",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_START);
    car_manual_button_start_consumed(&button);
    const uint32_t generation_before = control.reverse_control_reset_generation;
    car_inputs_t start = make_inputs(125u, false);
    start.manual_run_request = button.run_request;
    start.manual_stop_pending = button.stop_pending;
    start.manual_button_startup_release_confirmed =
        button.startup_release_confirmed;
    CHECK_EQ("physical Follow start requests one new reverse RESET lifecycle",
             car_control_begin_manual_follow_start(&control, &config, &start),
             1u);
    CHECK_EQ("new start increments reset generation once",
             control.reverse_control_reset_generation, generation_before + 1u);
    CHECK_EQ("new start clears old local KEEP gate",
             control.reverse_control_keep_seen, 0u);
    CHECK_EQ("new start clears old local fresh-frame gate",
             control.reverse_control_fresh_frames, 0u);
    CHECK_EQ("new start closes the transport recovery gate",
             control.reverse_reset_transport_gate_open, 0u);
    CHECK_EQ("new start flushes stale queued camera data",
             control.rx_recovery_flush_required, 1u);
    CHECK_EQ("new start immediately removes the previous drive permission",
             control.road_drive_allowed, 0u);
    CHECK_EQ("new start ends the previous Follow session",
             control.follow_session_active, 0u);
    CHECK_EQ("manual request remains latched during RESET/KEEP/fresh wait",
             control.manual_run_request, 1u);
    CHECK_EQ("one button request remains latched while waiting",
             button.run_request, 1u);

    CHECK_EQ("held low level cannot create another start event",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_NONE);
    CHECK_EQ("held button leaves no second pending start",
             button.start_pending, 0u);
    car_control_tick(&control, &config, &start);
    CHECK_EQ("held request cannot reuse old KEEP/fresh gates",
             control.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("held request does not repeat RESET",
             control.reverse_control_reset_generation, generation_before + 1u);

    car_control_rx_recovery_flush_complete(&control);
    car_control_reverse_control_reset_started(&control);
    car_reverse_control_tx_t tx{};
    car_reverse_control_init(&tx, 1000u);
    car_reverse_control_request_reset(&tx, 1000u);
    std::array<uint8_t, CAR_REVERSE_CONTROL_FRAME_SIZE> frame{};
    for (uint32_t timestamp = 1000u; timestamp <= 1200u; timestamp += 50u) {
        for (uint8_t index = 0u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
            uint8_t byte = 0u;
            CHECK_EQ("new lifecycle emits complete scheduled RESET bytes",
                     car_reverse_control_next_byte(&tx, timestamp, 0u, &byte),
                     1u);
            frame[index] = byte;
        }
        CHECK_EQ("each startup resync frame carries RESET",
                 frame[3], CAR_REVERSE_CONTROL_RESET);
        CHECK_EQ("RESET phase stays closed until all five frames complete",
                 control.reverse_reset_transport_gate_open, 0u);
        if (tx.reset_phase_complete != 0u) {
            car_control_reverse_control_reset_phase_complete(&control);
        }
    }
    CHECK_EQ("five RESET frames finish the independent reverse phase",
             tx.reset_phase_complete, 1u);
    CHECK_EQ("RESET completion opens only the transport recovery gate",
             control.reverse_reset_transport_gate_open, 1u);
    uint8_t byte = 0u;
    CHECK_EQ("KEEP cannot begin before the RESET interval expires",
             car_reverse_control_next_byte(&tx, 1249u, 1u, &byte), 0u);
    for (uint8_t index = 0u; index < CAR_REVERSE_CONTROL_FRAME_SIZE; ++index) {
        CHECK_EQ("KEEP follows the completed five-frame RESET phase",
                 car_reverse_control_next_byte(&tx, 1250u, 1u, &byte), 1u);
        frame[index] = byte;
    }
    CHECK_EQ("new lease is KEEP", frame[3], CAR_REVERSE_CONTROL_KEEP);
    car_control_reverse_control_keep_started(&control);
    CHECK_EQ("controller records KEEP only after its complete frame",
             control.reverse_control_keep_seen, 1u);
    CHECK_EQ("KEEP begins with a fresh three-observation gate",
             control.reverse_control_fresh_frames, 0u);

    const uint32_t reset_generation_after_start =
        control.reverse_control_reset_generation;
    send_frame_at(&control, &config, 1u, false, 0u, 0,
                  1251u, 1251u, false);
    CHECK_EQ("invalid visual observations do not start another RESET",
             control.reverse_control_reset_generation,
             reset_generation_after_start);
    CHECK_EQ("invalid visual observations do not count as fresh route frames",
             control.reverse_control_fresh_frames, 0u);

    for (uint16_t sequence = 2u; sequence <= 4u; ++sequence) {
        const uint32_t timestamp = 1250u + sequence;
        send_frame_at(&control, &config, sequence, true, 80u, 0,
                      timestamp, timestamp, false);
        CHECK_EQ("Follow remains stopped until three post-KEEP observations",
                 control.follow_session_active, 0u);
        CHECK_EQ("post-KEEP observations advance the fresh gate one at a time",
                 control.reverse_control_fresh_frames,
                 static_cast<uint8_t>(sequence - 1u));
    }
    car_inputs_t latched_request = make_inputs(1260u, false);
    latched_request.manual_run_request = button.run_request;
    latched_request.manual_button_startup_release_confirmed =
        button.startup_release_confirmed;
    car_control_tick(&control, &config, &latched_request);
    CHECK_EQ("one original physical request starts after RESET/KEEP/three frames",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("fresh startup restores Follow session permission",
             control.road_drive_allowed, 1u);

    car_manual_button_poll_high(&button, 1u, 1300u);
    car_manual_button_poll_high(&button, 1u, 1320u);
    CHECK_EQ("rearmed second low edge becomes a stop",
             car_manual_button_note_low_edge(&button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_STOP);
    car_inputs_t stopped = latched_request;
    stopped.manual_run_request = button.run_request;
    stopped.manual_stop_pending = button.stop_pending;
    const uint32_t stopped_generation = control.reverse_control_reset_generation;
    CHECK_EQ("fresh stop state cancels a stale queued start event",
             car_control_begin_manual_follow_start(&control, &config, &stopped),
             0u);
    CHECK_EQ("stop rejection does not enqueue a start RESET",
             control.reverse_control_reset_generation, stopped_generation);

    car_control_t faulted = control;
    faulted.fault = CAR_FAULT_ENCODER_INIT;
    faulted.fault_latched = 1u;
    faulted.mode = CAR_MODE_FAULT_LATCHED;
    car_inputs_t fault_start = make_inputs(1330u, false);
    fault_start.manual_run_request = 1u;
    fault_start.manual_button_startup_release_confirmed = 1u;
    CHECK_EQ("hard fault rejects a physical start resync",
             car_control_begin_manual_follow_start(
                 &faulted, &config, &fault_start),
             0u);
    CHECK_EQ("hard fault rejection leaves reset generation unchanged",
             faulted.reverse_control_reset_generation, stopped_generation);

    car_control_t stop_race_control{};
    car_manual_button_t stop_race_button{};
    start_follow_by_physical_pe7(&stop_race_control, &config, 80u, 100u);
    car_manual_button_init(&stop_race_button, 1u, 200u);
    car_manual_button_poll_high(&stop_race_button, 1u, 220u);
    CHECK_EQ("race fixture captures a physical start event",
             car_manual_button_note_low_edge(&stop_race_button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_START);
    car_manual_button_start_consumed(&stop_race_button);
    car_inputs_t stale_start_snapshot = make_inputs(201u, false);
    stale_start_snapshot.manual_run_request = stop_race_button.run_request;
    stale_start_snapshot.manual_stop_pending = stop_race_button.stop_pending;
    car_manual_button_poll_high(&stop_race_button, 1u, 210u);
    car_manual_button_poll_high(&stop_race_button, 1u, 230u);
    CHECK_EQ("a stop arriving before deferred start handoff wins",
             car_manual_button_note_low_edge(&stop_race_button, 0u, 0u),
             CAR_MANUAL_BUTTON_EVENT_STOP);
    car_inputs_t raced_stop = make_inputs(231u, false);
    raced_stop.manual_run_request = stop_race_button.run_request;
    raced_stop.manual_stop_pending = stop_race_button.stop_pending;
    const uint32_t race_generation =
        stop_race_control.reverse_control_reset_generation;
    CHECK_EQ("deferred start handoff rejects the fresh physical stop state",
             car_control_begin_manual_follow_start(
                 &stop_race_control, &config, &raced_stop),
             0u);
    CHECK_EQ("start-stop race does not enqueue a RESET after stop",
             stop_race_control.reverse_control_reset_generation,
             race_generation);
    car_manual_button_stop_consumed(&stop_race_button);
    car_inputs_t refreshed_after_stop = make_inputs(232u, false);
    refreshed_after_stop.manual_run_request = stop_race_button.run_request;
    refreshed_after_stop.manual_stop_pending = stop_race_button.stop_pending;
    uint8_t handoff_accepted_after_consumed_stop = 0u;
    if (stop_race_button.run_request != 0u &&
        stop_race_button.stop_pending == 0u &&
        stop_race_button.start_pending == 0u) {
        handoff_accepted_after_consumed_stop =
            car_control_begin_manual_follow_start(
                &stop_race_control, &config, &refreshed_after_stop);
    }
    CHECK_EQ("captured pre-stop input is stale after stop consumption",
             stale_start_snapshot.manual_run_request, 1u);
    CHECK_EQ("live physical request remains cleared after stop consumption",
             refreshed_after_stop.manual_run_request, 0u);
    CHECK_EQ("start handoff recheck skips a consumed stop race",
             handoff_accepted_after_consumed_stop, 0u);
    CHECK_EQ("consumed stop race still does not enqueue RESET",
             stop_race_control.reverse_control_reset_generation,
             race_generation);
}

void test_follow_visual_hold_requires_real_outputs_and_full_recovery()
{
    const car_config_t config = make_hold_follow_config();
    car_control_t control{};
    enter_follow_visual_hold(&control, &config, 300u, 300u);
    CHECK_EQ("hold fixture reaches manually started Follow", control.mode,
             CAR_MODE_FOLLOW_VISION_HOLD);
    const uint32_t reset_generation = control.reverse_control_reset_generation;
    CHECK_EQ("valid=0 after an applied Follow output enters mode 12",
             control.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    CHECK_EQ("545 trusted error maps to the held steering target",
             control.vision_hold_servo_pulse_us, 1900u);
    CHECK_EQ("hold retains the trusted signed error independently of V1 zero",
             control.vision_hold_error_norm, 545);
    CHECK_EQ("hold retains the source valid sequence",
             control.vision_hold_source_sequence, 303u);
    CHECK_EQ("hold retains the source valid arrival time",
             control.vision_hold_source_rx_ms, 327u);
    CHECK_EQ("hold records its entry time separately from the source arrival",
             control.vision_hold_started_ms, 328u);
    CHECK_EQ("hold output selector returns the held target, not applied pulse",
             car_control_follow_servo_output_target_us(&control), 1900u);
    const uint16_t hold_target_us =
        car_control_follow_servo_output_target_us(&control);
    CHECK_EQ("applied servo pulse remains a separate entry snapshot",
             control.vision_hold_applied_servo_pulse_us, 1600u);
    CHECK_EQ("snapshot entry marker is written for this hold entry",
             car_vision_hold_snapshot_entry, 1u);
    CHECK_EQ("published snapshot retains the trusted signed error",
             car_vision_hold_snapshot_error_norm, 545);
    CHECK_EQ("published snapshot retains source sequence independently",
             car_vision_hold_snapshot_sequence, 303u);
    CHECK_EQ("published snapshot retains valid RX arrival separately",
             car_vision_hold_snapshot_source_rx_ms, 327u);
    CHECK_EQ("published snapshot retains hold entry time",
             car_vision_hold_snapshot_entered_ms, 328u);
    CHECK_EQ("published snapshot target is separate from applied pulse",
             car_vision_hold_snapshot_target_us, 1900u);
    CHECK_EQ("published snapshot applied pulse records the 1600 us entry",
             car_vision_hold_snapshot_applied_us, 1600u);
    CHECK_EQ("published snapshot preserves left entry duty independently",
             car_vision_hold_snapshot_left_permille, 120u);
    CHECK_EQ("published snapshot preserves right entry duty independently",
             car_vision_hold_snapshot_right_permille, 180u);
    CHECK_EQ("hold snapshot records left duty independently",
             control.vision_hold_motor_left_permille, 120u);
    CHECK_EQ("hold snapshot records right duty independently",
             control.vision_hold_motor_right_permille, 180u);
    CHECK_EQ("hold preserves the nominal base request independently of side outputs",
             control.vision_hold_base_duty_permille, 0u);

    send_frame_at(&control, &config, 305u, true, 80u, -300,
                  329u, 329u, false);
    send_frame_at(&control, &config, 306u, true, 80u, -300,
                  330u, 330u, false);
    CHECK_EQ("one or two valid frames do not release hold",
             control.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    send_frame_at(&control, &config, 307u, false, 0u, 0,
                  331u, 331u, false);
    CHECK_EQ("valid=0 breaks the consecutive recovery route count",
             control.vision_hold_valid_count, 0u);
    send_frame_at(&control, &config, 308u, true, 80u, -300,
                  332u, 332u, false);
    send_frame_at(&control, &config, 309u, true, 80u, -300,
                  333u, 333u, false);
    CHECK_EQ("recovery still requires three consecutive valid frames",
             control.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    send_frame_at(&control, &config, 310u, true, 80u, -300,
                  334u, 334u, false);
    CHECK_EQ("third in-deadline valid frame resumes the saved session",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("in-deadline hold recovery counts once as automatic recovery",
             control.auto_recovery_count, 1u);
    send_frame_at(&control, &config, 311u, true, 80u, -280,
                  335u, 335u, false);
    CHECK_EQ("continuing valid frames do not recount the same hold episode",
             control.auto_recovery_count, 1u);
    CHECK_EQ("early resume restores the nominal base request, not a side maximum",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("early visual hold recovery does not RESET reverse cache",
             control.reverse_control_reset_generation, reset_generation);
    CHECK_EQ("recovery does not rewrite the latched entry source error",
             control.vision_hold_error_norm, 545);
    CHECK_EQ("recovery does not rewrite the latched entry source sequence",
             control.vision_hold_source_sequence, 303u);
    CHECK_EQ("recovery does not rewrite the held entry target",
             control.vision_hold_servo_pulse_us, 1900u);
    CHECK_EQ("recovery does not rewrite the published entry snapshot",
             car_vision_hold_snapshot_error_norm, 545);
    CHECK_EQ("entry marker persists after hold recovery",
             car_vision_hold_snapshot_entry, 1u);
    CHECK_EQ("source sequence snapshot persists after hold recovery",
             car_vision_hold_snapshot_sequence, 303u);
    CHECK_EQ("source RX snapshot persists after hold recovery",
             car_vision_hold_snapshot_source_rx_ms, 327u);
    CHECK_EQ("hold-entry time snapshot persists after recovery",
             car_vision_hold_snapshot_entered_ms, 328u);
    CHECK_EQ("mapped target snapshot persists after recovery",
             car_vision_hold_snapshot_target_us, 1900u);
    CHECK_EQ("entry applied pulse snapshot persists after recovery",
             car_vision_hold_snapshot_applied_us, 1600u);
    CHECK_EQ("left motor snapshot persists after recovery",
             car_vision_hold_snapshot_left_permille, 120u);
    CHECK_EQ("right motor snapshot persists after recovery",
             car_vision_hold_snapshot_right_permille, 180u);

    /* The held target still passes through the FollowTurn 120 us/20 ms servo
       limiter; it is never replaced by the 1600 us entry pulse. */
    FakeServoRegisters fast_fake = make_valid_fake_servo();
    const car_servo_safety_io_t fast_io = fake_servo_io(&fast_fake);
    car_servo_safety_t fast_servo{};
    car_servo_turn_safety_init(&fast_servo);
    CHECK_EQ("fast-profile servo starts at centre before the hold target",
             car_servo_turn_safety_request_selected_with_slew(
                 &fast_servo, CAR_SERVO_SOURCE_FOLLOW, 1600u, 1530u, 1u,
                 0u, 120u, &fast_io), 1u);
    fast_fake.update_pending = 1u;
    (void)car_servo_turn_safety_request_selected_with_slew(
        &fast_servo, CAR_SERVO_SOURCE_FOLLOW, 1600u, 1530u, 1u,
        1u, 120u, &fast_io);
    fast_fake.update_pending = 1u;
    (void)car_servo_turn_safety_request_selected_with_slew(
        &fast_servo, CAR_SERVO_SOURCE_FOLLOW, 1600u, 1530u, 1u,
        20u, 120u, &fast_io);
    fast_fake.update_pending = 1u;
    (void)car_servo_turn_safety_request_selected_with_slew(
        &fast_servo, CAR_SERVO_SOURCE_FOLLOW, 1600u, 1530u, 1u,
        40u, 120u, &fast_io);
    CHECK_EQ("fixture reaches the separately captured 1600 us applied pulse",
             fast_servo.limited_us, 1600u);
    const uint16_t expected_turn_slew_us[] = {
        1720u, 1720u, 1840u, 1840u, 1900u
    };
    for (uint32_t now_ms = 60u; now_ms <= 100u; now_ms += 10u) {
        fast_fake.update_pending = 1u;
        CHECK_EQ("hold steering request remains accepted by servo guard",
                 car_servo_turn_safety_request_selected_with_slew(
                     &fast_servo, CAR_SERVO_SOURCE_FOLLOW,
                     hold_target_us,
                     1530u, 1u, now_ms, 120u, &fast_io), 1u);
        CHECK(fast_servo.limited_us >= 1600u &&
                  fast_servo.limited_us <= 1900u,
              "held steering slew stays between entry pulse and mapped target");
        CHECK_EQ("FollowTurn advances only on 20 ms boundaries by at most 120 us",
                 fast_servo.limited_us,
                 expected_turn_slew_us[(now_ms - 60u) / 10u]);
    }
    CHECK_EQ("qualified fast-profile steering reaches the mapped held target",
             fast_servo.limited_us, 1900u);

    FakeServoRegisters startup_fake = make_valid_fake_servo();
    const car_servo_safety_io_t startup_io = fake_servo_io(&startup_fake);
    car_servo_safety_t startup_servo{};
    car_servo_safety_init(&startup_servo);
    CHECK_EQ("startup servo starts with its original 10 us limiter",
             car_servo_safety_request_selected_with_slew(
                 &startup_servo, CAR_SERVO_SOURCE_FOLLOW, 1900u, 1530u,
                 1u, 0u, 10u, &startup_io), 1u);
    startup_fake.update_pending = 1u;
    (void)car_servo_safety_request_selected_with_slew(
        &startup_servo, CAR_SERVO_SOURCE_FOLLOW, 1900u, 1530u,
        1u, 1u, 10u, &startup_io);
    startup_fake.update_pending = 1u;
    (void)car_servo_safety_request_selected_with_slew(
        &startup_servo, CAR_SERVO_SOURCE_FOLLOW, 1900u, 1530u,
        1u, 20u, 10u, &startup_io);
    CHECK_EQ("startup first qualified 20 ms step remains 10 us",
             startup_servo.limited_us, 1540u);

    /* A new run proves that an invalid startup snapshot cannot enter hold. */
    car_control_t not_ready{};
    start_follow_by_physical_pe7(&not_ready, &config, 400u, 0u);
    car_control_note_applied_outputs(&not_ready, 0u, 0u, 0u, 0u, 0u);
    send_frame_at(&not_ready, &config, 403u, false, 0u, 0,
                  25u, 25u, false);
    CHECK_EQ("no completed/active servo or motor output cannot start hold",
             not_ready.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("invalid output snapshot is a soft visual fault", not_ready.fault,
             CAR_FAULT_VISION_INVALID);
    CHECK_EQ("unready output remains zero", not_ready.follow_output_duty_permille, 0u);
}

void test_follow_visual_hold_exact_deadline_and_transport_split()
{
    const car_config_t config = make_hold_follow_config();
    car_control_t control{};
    constexpr uint32_t start_ms = 0xFFFFFF00u;
    enter_follow_visual_hold(&control, &config, 500u, start_ms);
    CHECK_EQ("deadline fixture enters active hold", control.vision_hold_active, 1u);
    const uint32_t reset_generation = control.reverse_control_reset_generation;
    const uint32_t source_valid_ms = control.vision_hold_source_rx_ms;
    const uint32_t hold_entry_ms = control.vision_hold_started_ms;
    uint16_t sequence = 505u;
    for (uint32_t hold_elapsed = 200u; hold_elapsed <= 2800u;
         hold_elapsed += 200u, ++sequence) {
        const uint32_t now_ms = hold_entry_ms + hold_elapsed;
        const bool scattered_valid = hold_elapsed == 1000u;
        send_frame_at(&control, &config, sequence, scattered_valid,
                      scattered_valid ? 80u : 0u,
                      scattered_valid ? -500 : 0,
                      now_ms, now_ms, false);
    }
    CHECK_EQ("scattered valid frame does not refresh the trusted source time",
             control.vision_last_valid_ms, source_valid_ms);
    CHECK_EQ("later invalid V1 payload does not overwrite hold error",
             control.vision_hold_error_norm, 545);
    CHECK_EQ("hold target remains mapped from the source error",
             control.vision_hold_servo_pulse_us, 1900u);

    send_frame_at(&control, &config, sequence++, false, 0u, 0,
                  source_valid_ms + 2999u, source_valid_ms + 2999u, false);
    car_inputs_t before_deadline = make_inputs(source_valid_ms + 2999u, false);
    car_control_tick(&control, &config, &before_deadline);
    CHECK_EQ("2999 ms from the last trusted valid route stays in hold",
             control.vision_hold_active, 1u);
    CHECK_EQ("2999 ms hold preserves each captured side output independently",
             control.vision_hold_motor_left_permille, 120u);
    CHECK_EQ("2999 ms hold preserves the captured right output",
             control.vision_hold_motor_right_permille, 180u);
    CHECK_EQ("2999 ms hold keeps nominal base separate from side outputs",
             control.follow_output_duty_permille, 0u);

    car_inputs_t deadline = make_inputs(source_valid_ms + 3000u, false);
    car_control_tick(&control, &config, &deadline);
    CHECK_EQ("exact 3000 ms boundary expires visual hold",
             control.vision_hold_active, 0u);
    CHECK_EQ("expired hold remains visible while waiting for route",
             control.vision_hold_expired, 1u);
    CHECK_EQ("deadline requests a real hardware stop and restart gate",
             control.vision_hold_restart_required, 1u);
    CHECK_EQ("deadline immediately zeros motor software output",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("deadline keeps manual session latched for safe recovery",
             control.follow_session_active, 1u);
    CHECK_EQ("deadline itself does not RESET reverse cache",
             control.reverse_control_reset_generation, reset_generation);

    /* Mirror main's immediate stop/first-PWM restart boundary. */
    control.vision_hold_restart_required = 0u;
    car_control_note_applied_outputs(&control, 0u, 0u, 0u, 0u, 0u);
    send_frame_at(&control, &config, sequence++, false, 0u, 0,
                  hold_entry_ms + 3100u, hold_entry_ms + 3100u, false);
    CHECK_EQ("fresh valid=0 transport keeps expired wait without reset",
             control.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    CHECK_EQ("expired wait does not restart outputs for missing route",
             control.follow_output_duty_permille, 0u);
    CHECK_EQ("expired valid=0 frames preserve reverse cache generation",
             control.reverse_control_reset_generation, reset_generation);

    send_frame_at(&control, &config, sequence++, true, 80u, 100,
                  hold_entry_ms + 3200u, hold_entry_ms + 3200u, false);
    send_frame_at(&control, &config, sequence++, true, 80u, 100,
                  hold_entry_ms + 3210u, hold_entry_ms + 3210u, false);
    CHECK_EQ("expired wait requires all three new valid route frames",
             control.vision_hold_valid_count, 2u);
    CHECK_EQ("expired wait remains zero through its first two recovery frames",
             control.follow_output_duty_permille, 0u);
    send_frame_at(&control, &config, sequence++, true, 80u, 100,
                  hold_entry_ms + 3220u, hold_entry_ms + 3220u, false);
    CHECK_EQ("third post-deadline route frame re-enters normal Follow gate",
             control.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("post-deadline hold recovery counts once as automatic recovery",
             control.auto_recovery_count, 1u);
    CHECK_EQ("late recovery cannot reuse pre-stop duty",
             control.follow_output_duty_permille, 0u);
    send_frame_at(&control, &config, sequence++, true, 80u, 100,
                  hold_entry_ms + 3230u, hold_entry_ms + 3230u, false);
    CHECK_EQ("post-deadline continuation does not recount hold recovery",
             control.auto_recovery_count, 1u);
    CHECK_EQ("late recovery explicitly requests the normal restart gate",
             control.vision_hold_restart_required, 1u);

    car_run_block_reason_input_t restart_gate{};
    restart_gate.startup_release_seen = 1u;
    restart_gate.vision_fresh_valid = 1u;
    restart_gate.reverse_reset_keep_ready = 1u;
    restart_gate.profile_output_permission = 1u;
    restart_gate.follow_mode_active = 1u;
    restart_gate.follow_session_active = 1u;
    restart_gate.follow_output_permission = 1u;
    restart_gate.manual_run_request = 1u;
    CHECK_EQ("post-deadline recovery still waits for its first servo PWM",
             car_run_block_reason_evaluate(&restart_gate),
             CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME);
    restart_gate.servo_output_active = 1u;
    restart_gate.servo_first_frame_ready = 1u;
    CHECK_EQ("first PWM alone does not skip the normal motor ramp gate",
             car_run_block_reason_evaluate(&restart_gate),
             CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD);
    restart_gate.motor_left_applied = 1u;
    restart_gate.motor_right_applied = 1u;
    CHECK_EQ("restarted Follow is reported only after both motor sides apply",
             car_run_block_reason_evaluate(&restart_gate),
             CAR_RUN_BLOCK_RUNNING_WITH_MOTOR_OUTPUT);

    /* Exercise the receive path at the exact source-valid deadline before a
       controller tick can mark the hold expired. The main-loop restart flag
       must still force a stop and invalidate the previous servo-frame gate. */
    car_control_t exact_deadline_recovery{};
    constexpr uint32_t exact_start_ms = 0xFFFFF000u;
    enter_follow_visual_hold(&exact_deadline_recovery, &config, 700u,
                             exact_start_ms);
    const uint32_t exact_source_ms =
        exact_deadline_recovery.vision_hold_source_rx_ms;
    uint16_t exact_sequence = 705u;
    /* Keep the UART lease alive with invalid route frames; the 250 ms
       communication timeout must stay distinct from the 3000 ms valid-route
       hold deadline under test here. */
    for (uint32_t elapsed_ms = 200u; elapsed_ms <= 2600u;
         elapsed_ms += 200u) {
        const uint32_t now_ms = exact_source_ms + elapsed_ms;
        send_frame_at(&exact_deadline_recovery, &config, exact_sequence++,
                      false, 0u, 0, now_ms, now_ms, false);
    }
    CHECK_EQ("continuous invalid transport does not refresh trusted RX time",
             exact_deadline_recovery.vision_last_valid_ms, exact_source_ms);
    CHECK_EQ("continuous invalid route keeps the vision hold active",
             exact_deadline_recovery.vision_hold_active, 1u);
    send_frame_at(&exact_deadline_recovery, &config, exact_sequence++, true,
                  80u, 100, exact_source_ms + 2800u,
                  exact_source_ms + 2800u, false);
    send_frame_at(&exact_deadline_recovery, &config, exact_sequence++, true,
                  80u, 100, exact_source_ms + 2900u,
                  exact_source_ms + 2900u, false);
    CHECK_EQ("two late valid frames are queued for exact-deadline recovery",
             exact_deadline_recovery.vision_hold_valid_count, 2u);
    const uint32_t exact_deadline_ms = exact_source_ms + 3000u;
    send_frame_at(&exact_deadline_recovery, &config, exact_sequence, true,
                  80u, 100, exact_deadline_ms, exact_deadline_ms, false);
    CHECK_EQ("third valid frame at exact deadline resumes only logically",
             exact_deadline_recovery.mode, CAR_MODE_FOLLOW_RUNNING);
    CHECK_EQ("exact-deadline recovery requests the same-loop hardware restart",
             exact_deadline_recovery.vision_hold_restart_required, 1u);
    CHECK_EQ("exact-deadline recovery keeps road permission closed for this loop",
             exact_deadline_recovery.road_drive_allowed, 0u);
    CHECK_EQ("exact-deadline recovery cannot reuse the held motor base",
             exact_deadline_recovery.follow_output_duty_permille, 0u);
    CHECK_EQ("accepted recovery frame becomes the new trusted RX timestamp",
             exact_deadline_recovery.vision_last_valid_ms, exact_deadline_ms);
    car_run_block_reason_input_t exact_restart_gate{};
    exact_restart_gate.startup_release_seen = 1u;
    exact_restart_gate.vision_fresh_valid = 1u;
    exact_restart_gate.reverse_reset_keep_ready = 1u;
    exact_restart_gate.profile_output_permission = 1u;
    exact_restart_gate.follow_mode_active = 1u;
    exact_restart_gate.follow_session_active = 1u;
    exact_restart_gate.follow_output_permission = 1u;
    exact_restart_gate.manual_run_request = 1u;
    exact_restart_gate.servo_output_active = 1u;
    exact_restart_gate.servo_first_frame_ready = 0u;
    CHECK_EQ("exact-deadline restart rejects the old frame-ready state",
             car_run_block_reason_evaluate(&exact_restart_gate),
             CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME);
    exact_restart_gate.servo_first_frame_ready = 1u;
    CHECK_EQ("exact-deadline restart still requires the normal motor ramp",
             car_run_block_reason_evaluate(&exact_restart_gate),
             CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD);
}

void test_follow_visual_hold_comm_fault_and_manual_stop_priority()
{
    const car_config_t config = make_hold_follow_config();
    car_control_t timed_out{};
    enter_follow_visual_hold(&timed_out, &config, 100u, 1000u);
    const uint32_t hold_entries = timed_out.vision_hold_entries;
    const int16_t held_error = timed_out.vision_hold_error_norm;
    const uint16_t held_sequence = timed_out.vision_hold_source_sequence;

    car_inputs_t just_before_timeout = make_inputs(
        timed_out.vision_hold_started_ms + 249u, false);
    car_control_tick(&timed_out, &config, &just_before_timeout);
    CHECK_EQ("249 ms without a transport frame remains below comm timeout",
             timed_out.mode, CAR_MODE_FOLLOW_VISION_HOLD);
    CHECK_EQ("249 ms no-frame interval still leaves hold active",
             timed_out.vision_hold_active, 1u);
    car_inputs_t comm_timeout = make_inputs(
        timed_out.vision_hold_started_ms + 250u, false);
    car_control_tick(&timed_out, &config, &comm_timeout);
    CHECK_EQ("250 ms without any frame takes priority over 3 s line hold",
             timed_out.fault, CAR_FAULT_VISION_TIMEOUT);
    CHECK_EQ("250 ms communication timeout exits active hold",
             timed_out.vision_hold_active, 0u);
    CHECK_EQ("communication fault stops road output immediately",
             timed_out.road_drive_allowed, 0u);
    CHECK_EQ("communication fault does not erase the entry snapshot",
             timed_out.vision_hold_entries, hold_entries);
    CHECK_EQ("communication fault preserves source error for diagnosis",
             timed_out.vision_hold_error_norm, held_error);
    CHECK_EQ("communication fault preserves source sequence for diagnosis",
             timed_out.vision_hold_source_sequence, held_sequence);

    car_control_t manually_stopped{};
    enter_follow_visual_hold(&manually_stopped, &config, 200u, 2000u);
    const uint32_t stop_reset_generation =
        manually_stopped.reverse_control_reset_generation;
    const uint32_t stop_generation = manually_stopped.isr_switch_generation;
    car_control_note_switch_release_isr(&manually_stopped);
    CHECK_EQ("physical PE7 ISR advances the stop generation",
             manually_stopped.isr_switch_generation, stop_generation + 1u);
    CHECK_EQ("physical PE7 ISR closes Follow session before main-loop work",
             manually_stopped.follow_session_active, 0u);
    CHECK_EQ("physical PE7 ISR immediately revokes road permission",
             manually_stopped.road_drive_allowed, 0u);
    const car_fault_t stop_fault =
        car_control_take_fast_stop_isr(&manually_stopped);
    CHECK_EQ("PE7 stop is not replaced by a communication fault",
             stop_fault, CAR_FAULT_SWITCH_RELEASE);
    car_control_note_fault(&manually_stopped, stop_fault);
    CHECK_EQ("manual PE7 stop wins over the remaining visual hold",
             manually_stopped.mode, CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("manual PE7 stop records the dedicated hold exit cause",
             manually_stopped.vision_hold_exit_cause,
             CAR_VISION_HOLD_EXIT_MANUAL_STOP);
    CHECK_EQ("manual PE7 stop preserves the hold entry source error",
             manually_stopped.vision_hold_error_norm, 545);
    CHECK_EQ("manual PE7 stop preserves the hold entry timestamp",
             manually_stopped.vision_hold_source_rx_ms, 2027u);
    CHECK_EQ("manual PE7 stop leaves the published entry marker intact",
             car_vision_hold_snapshot_entry, hold_entries);
    CHECK_EQ("manual PE7 stop leaves the published target intact",
             car_vision_hold_snapshot_target_us, 1900u);
    CHECK_EQ("manual PE7 stop leaves the published applied pulse intact",
             car_vision_hold_snapshot_applied_us, 1600u);
    CHECK(manually_stopped.reverse_control_reset_generation >
              stop_reset_generation,
          "physical PE7 stop requests the existing reverse-cache resync");
}

void test_follow_valid_zero_keeps_transport_alive_without_session()
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    confirm_follow_startup_release(&control, &config, 0u);
    car_control_reverse_control_reset_phase_complete(&control);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t sequence = 1u; sequence <= 3u; ++sequence) {
        send_frame_at(&control, &config, sequence, true, 80u, 50,
                      sequence, sequence, true);
    }
    CHECK_EQ("three camera routes do not auto-start without a button request",
             control.mode, CAR_MODE_WAIT_START_EDGE);
    const uint32_t reset_generation = control.reverse_control_reset_generation;
    for (uint16_t sequence = 4u; sequence <= 7u; ++sequence) {
        const uint32_t now_ms = 3u + (uint32_t)(sequence - 3u) * 100u;
        send_frame_at(&control, &config, sequence, false, 0u, 0,
                      now_ms, now_ms, true);
        car_inputs_t tick = make_inputs(now_ms, true);
        car_control_tick(&control, &config, &tick);
    }
    CHECK_EQ("fresh valid=0 frames preserve idle Follow transport",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("fresh valid=0 frames do not reset reverse cached geometry",
             control.reverse_control_reset_generation, reset_generation);
    send_frame_at(&control, &config, 8u, true, 80u, -50,
                  510u, 510u, true);
    CHECK_EQ("new valid route after continuous valid=0 transport is accepted",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("route recovery still has no manual run request",
             control.manual_run_request, 0u);
    CHECK_EQ("route recovery cannot auto-start the motor session",
             control.follow_session_active, 0u);
    CHECK_EQ("route recovery keeps Follow output zero",
             control.follow_output_duty_permille, 0u);
}

void test_uart_fragment_gap_and_exact_partial_timeout()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_vision_rx_t rx{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);

    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    CHECK(pack_frame(make_wire_frame(30u), &bytes), "pack split-frame test data");
    for (size_t i = 0; i < 6u; ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 10u);
    }
    poll_rx_current_snapshot(&rx, &control, &config, 10u, true);
    CHECK_EQ("first UART fragment retained", rx.parser.length, 6u);
    for (size_t i = 6u; i < bytes.size(); ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 29u);
    }
    poll_rx_current_snapshot(&rx, &control, &config, 29u, true);
    CHECK_EQ("split under 20 ms completes", control.rx_frame_count, 1u);
    CHECK_EQ("19 ms split does not create stale fault", rx.stale_fault, 0u);
    CHECK_EQ("completed split frame appears in raw diagnostic state",
             control.latest_rx_sequence, 30u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    CHECK(pack_frame(make_wire_frame(31u), &bytes), "pack partial-timeout test data");
    for (size_t i = 0; i < 6u; ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 10u);
    }
    poll_rx_current_snapshot(&rx, &control, &config, 10u, true);
    for (size_t i = 6u; i < bytes.size(); ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 30u);
    }
    poll_rx_current_snapshot(&rx, &control, &config, 30u, true);
    CHECK_EQ("exact 20 ms partial gap rejected", rx.partial_timeout_count, 1u);
    CHECK_EQ("exact partial timeout becomes stale fault", rx.stale_fault, 1u);
    CHECK_EQ("exact partial timeout is visible to controller",
             control.fault, CAR_FAULT_RX_STALE);
    CHECK_EQ("timed-out fragment cannot form vision frame", control.rx_frame_count, 0u);
}

void test_uart_poll_consumes_only_captured_head_snapshot()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_vision_rx_t rx{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);

    std::array<uint8_t, VP_FRAME_SIZE> bytes{};
    CHECK(pack_frame(make_wire_frame(32u), &bytes),
          "pack snapshot-race test data");
    for (size_t i = 0; i < 2u; ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 100u);
    }

    /* Mirror the target critical section: capture head, then sample now. */
    const uint16_t captured_head = rx.head;
    const uint32_t captured_now_ms = 101u;
    for (size_t i = 2u; i < bytes.size(); ++i) {
        car_vision_rx_isr_byte(&rx, bytes[i], 102u);
    }

    car_vision_rx_poll(&rx, &control, &config, captured_head,
                       captured_now_ms, 1u);
    CHECK_EQ("poll stops at captured head despite later ISR bytes",
             rx.tail, captured_head);
    CHECK(rx.head != captured_head,
          "post-snapshot ISR bytes remain queued for next poll");
    CHECK_EQ("post-snapshot bytes are not parsed with earlier now",
             control.rx_frame_count, 0u);
    CHECK_EQ("post-snapshot future arrival is not called stale",
             rx.queue_age_reject_count, 0u);
    CHECK_EQ("first fragment remains partial after snapshot poll",
             rx.parser.length, 2u);

    car_vision_rx_poll(&rx, &control, &config, rx.head, 102u, 1u);
    CHECK_EQ("next poll consumes remaining bytes", rx.tail, rx.head);
    CHECK_EQ("completed frame is accepted on next snapshot",
             control.rx_frame_count, 1u);
    CHECK_EQ("snapshot-race sequence appears in raw diagnostics",
             control.latest_rx_sequence, 32u);
    CHECK_EQ("normal capture race creates no timeout fault",
             control.fault, CAR_FAULT_NONE);
}

void test_uart_queue_age_overflow_clear_and_error()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_vision_rx_t rx{};

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    enqueue_frame(&rx, make_wire_frame(50u), 0u);
    poll_rx_current_snapshot(&rx, &control, &config, 99u, true);
    CHECK_EQ("queue younger than 100 ms accepted", control.rx_frame_count, 1u);
    CHECK_EQ("99 ms queued frame is not stale", rx.stale_fault, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    enqueue_frame(&rx, make_wire_frame(51u), 0u);
    poll_rx_current_snapshot(&rx, &control, &config, 100u, true);
    CHECK_EQ("queue at exact 100 ms rejected", rx.queue_age_reject_count, 1u);
    CHECK_EQ("old queue sets stale fault", rx.stale_fault, 1u);
    CHECK_EQ("old queue fault reaches control", control.fault, CAR_FAULT_RX_STALE);
    CHECK_EQ("stale queued frame not parsed", control.rx_frame_count, 0u);
    CHECK_EQ("stale queue flushed", rx.tail, rx.head);

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    for (unsigned i = 0; i < CAR_VISION_RX_RING_SIZE; ++i) {
        car_vision_rx_isr_byte(&rx, 0x00u, 1u);
    }
    CHECK_EQ("256th byte detects one-slot-empty ring overflow",
             rx.queue_overflow_count, 1u);
    CHECK_EQ("all incoming bytes counted through overflow", rx.rx_byte_count,
             CAR_VISION_RX_RING_SIZE);
    CHECK_EQ("ring stores only its 255 usable slots",
             (rx.head - rx.tail) & CAR_VISION_RX_RING_MASK,
             CAR_VISION_RX_RING_SIZE - 1u);
    poll_rx_current_snapshot(&rx, &control, &config, 1u, true);
    CHECK_EQ("ring overflow locks controller", control.fault, CAR_FAULT_RX_OVERFLOW);
    CHECK_EQ("ring overflow output remains disabled", control.bench_output_duty_permille, 0u);
    car_vision_rx_clear(&rx);
    CHECK_EQ("explicit local-clear helper drains ring", rx.tail, rx.head);
    CHECK_EQ("explicit clear resets overflow sticky flag", rx.overflow_fault, 0u);
    CHECK_EQ("explicit clear resets parser", rx.parser.length, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    car_vision_rx_isr_error(&rx);
    poll_rx_current_snapshot(&rx, &control, &config, 1u, true);
    CHECK_EQ("UART framing/error interrupt locks controller", control.fault, CAR_FAULT_UART);
    CHECK_EQ("UART error counter increments", rx.uart_error_count, 1u);
}

void test_uart_crc_noise_resync_and_recovery_lock()
{
    const car_config_t config = make_config(false, false);
    car_control_t control{};
    car_vision_rx_t rx{};
    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    std::array<uint8_t, VP_FRAME_SIZE> bad{};
    CHECK(pack_frame(make_wire_frame(60u), &bad), "pack CRC test frame");
    bad[10] ^= 0x10u;
    for (uint8_t byte : bad) {
        car_vision_rx_isr_byte(&rx, byte, 1u);
    }
    poll_rx_current_snapshot(&rx, &control, &config, 1u, true);
    CHECK_EQ("CRC error requests a stopped recovery boundary",
             control.fault, CAR_FAULT_PROTOCOL);
    CHECK_EQ("CRC error counter increments", rx.protocol_error_count, 1u);
    CHECK_EQ("CRC error never reaches accepted frame count", control.rx_frame_count, 0u);
    CHECK_EQ("CRC error closes RX until the caller atomically flushes",
             control.rx_recovery_flush_required, 1u);

    enqueue_frame(&rx, make_wire_frame(61u), 2u);
    poll_rx_current_snapshot(&rx, &control, &config, 2u, true);
    CHECK_EQ("queued post-error frame is held until the flush boundary",
             control.rx_frame_count, 0u);
    car_vision_rx_clear(&rx);
    car_control_rx_recovery_flush_complete(&control);
    for (uint16_t sequence = 61u; sequence <= 63u; ++sequence) {
        enqueue_frame(&rx, make_wire_frame(sequence), sequence - 58u);
        poll_rx_current_snapshot(&rx, &control, &config,
                                 sequence - 58u, true);
    }
    CHECK_EQ("three fresh frames after CRC flush recover observation",
             control.rx_frame_count, 3u);
    CHECK_EQ("three-frame recovery clears only the soft protocol fault",
             control.fault, CAR_FAULT_NONE);
    CHECK_EQ("default Debug config still has no motor output",
             control.follow_output_duty_permille, 0u);

    car_control_init(&control, &config, 0u, 1u, 0u);
    car_vision_rx_init(&rx);
    car_vision_rx_isr_byte(&rx, 0x77u, 1u);
    enqueue_frame(&rx, make_wire_frame(62u), 1u);
    poll_rx_current_snapshot(&rx, &control, &config, 1u, true);
    CHECK_EQ("garbage before sync is reported", rx.protocol_error_count, 1u);
    CHECK_EQ("garbage parser fault holds following bytes for the flush",
             control.rx_frame_count, 0u);
    CHECK_EQ("garbage parser fault is soft and output-gated",
             control.fault, CAR_FAULT_PROTOCOL);
}

void test_encoder_wrap_and_half_range_rejection()
{
    car_encoder_diag_t diag{};
    car_encoder_diag_init(&diag, 0xFFFFFFFDu, 65534u);
    CHECK_EQ("encoder init marks baseline", diag.initialized, 1u);
    CHECK_EQ("32-bit raw counter forward wrap accepted",
             car_encoder_diag_update(&diag, 1u, 2u), 1u);
    CHECK_EQ("32-bit wrap delta sums +4", diag.left_raw_delta_sum, 4);
    CHECK_EQ("16-bit raw counter forward wrap delta sums +4", diag.right_raw_delta_sum, 4);

    CHECK_EQ("encoder reverse wrap accepted",
             car_encoder_diag_update(&diag, 0xFFFFFFFDu, 65534u), 1u);
    CHECK_EQ("32-bit reverse wrap delta sums back to zero", diag.left_raw_delta_sum, 0);
    CHECK_EQ("16-bit reverse wrap delta sums back to zero", diag.right_raw_delta_sum, 0);

    car_encoder_diag_init(&diag, 10u, 10u);
    CHECK_EQ("exact 32-bit half-range is ambiguous",
             car_encoder_diag_update(&diag, 0x8000000Au, 11u), 0u);
    CHECK_EQ("ambiguous 32-bit movement increments count", diag.ambiguous_delta_count, 1u);
    CHECK_EQ("ambiguous 32-bit movement does not advance baseline", diag.left_raw, 10u);
    CHECK_EQ("ambiguous movement does not add a guessed delta", diag.right_raw_delta_sum, 0);

    car_encoder_diag_init(&diag, 10u, 10u);
    CHECK_EQ("exact 16-bit half-range is ambiguous",
             car_encoder_diag_update(&diag, 11u, static_cast<uint16_t>(10u + 0x8000u)), 0u);
    CHECK_EQ("ambiguous 16-bit movement increments count", diag.ambiguous_delta_count, 1u);
    CHECK_EQ("ambiguous 16-bit movement leaves baseline unchanged", diag.right_raw, 10u);
    CHECK_EQ("ambiguous 16-bit movement adds no left delta", diag.left_raw_delta_sum, 0);
}

car_runtime_log_sample_t make_runtime_log_follow_sample(uint32_t now_ms)
{
    car_runtime_log_sample_t sample{};
    sample.now_ms = now_ms;
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.follow_session_active = 1u;
    sample.manual_run_request = 1u;
    sample.follow_profile_compiled = 1u;
    sample.servo_requested_us = 1530u;
    sample.servo_limited_us = 1530u;
    sample.servo_applied_us = 1530u;
    sample.motor_requested_permille = 400u;
    sample.left_applied_permille = 120u;
    sample.right_applied_permille = 120u;
    sample.motor_ccr1_counts = 120u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 120u;
    sample.vision_flags = CAR_RUNTIME_LOG_VISION_LATEST_VALID |
                          CAR_RUNTIME_LOG_VISION_USABLE_FRESH;
    sample.vision_confidence = 90u;
    sample.vision_sequence = 7u;
    return sample;
}

void test_runtime_log_keeps_manual_start_resync_wait()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(100u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("resync-wait log fixture starts a Follow session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);

    sample.now_ms = 150u;
    sample.mode = CAR_MODE_RECOVERY_WAIT;
    sample.follow_session_active = 0u;
    sample.recovery_active = 1u;
    sample.recovery_frame_count = 0u;
    sample.manual_run_request = 1u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    sample.vision_flags = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("manual start RESET wait does not freeze the RAM log",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("RESET wait publishes its zero-output recovery row",
             car_runtime_log_storage.records[1].mode,
             CAR_MODE_RECOVERY_WAIT);
    CHECK_EQ("RESET wait record shows no applied left duty",
             car_runtime_log_storage.records[1].left_applied_permille, 0u);
    CHECK((car_runtime_log_storage.records[1].event_flags &
           CAR_RUNTIME_LOG_EVENT_STOP) == 0u,
          "retained manual request is not logged as a stop");
}

void test_runtime_log_session_fault_recovery_and_stop()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(100u);

    CHECK_EQ("runtime log starts idle", car_runtime_log_state(),
             CAR_RUNTIME_LOG_STATE_IDLE);
    CHECK_EQ("runtime log starts with no records",
             car_runtime_log_storage.header.count, 0u);
    sample.mode = CAR_MODE_DEBUG_STEERING_RUNNING;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("automatic steering does not start a RAM session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_IDLE);

    sample = make_runtime_log_follow_sample(100u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("manual Follow edge starts the session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("start event is first published record",
             car_runtime_log_storage.records[0].event_flags,
             CAR_RUNTIME_LOG_EVENT_START);
    CHECK_EQ("header start time matches first record",
             car_runtime_log_storage.header.start_ms,
             car_runtime_log_storage.records[0].time_ms);

    sample.now_ms = 149u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("periodic sample waits for full 50 ms interval",
             car_runtime_log_storage.header.count, 1u);

    sample.now_ms = 150u;
    sample.left_encoder_counts = INT64_C(0x123456789);
    sample.right_encoder_counts = -INT64_C(0x123456789);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("periodic row appears at exact 50 ms interval",
             car_runtime_log_storage.header.count, 2u);
    CHECK_EQ("large positive encoder total is preserved",
             car_runtime_log_storage.records[1].left_encoder_counts,
             INT64_C(0x123456789));
    CHECK_EQ("large negative encoder total is preserved",
             car_runtime_log_storage.records[1].right_encoder_counts,
             -INT64_C(0x123456789));

    sample.now_ms = 151u;
    sample.mode = CAR_MODE_RECOVERY_WAIT;
    sample.fault = CAR_FAULT_PROTOCOL;
    sample.fault_generation = 1u;
    sample.recovery_active = 1u;
    sample.recovery_frame_count = 1u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("soft vision fault keeps Follow logging active",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK((car_runtime_log_storage.records[2].event_flags &
           (CAR_RUNTIME_LOG_EVENT_FAULT | CAR_RUNTIME_LOG_EVENT_MODE |
            CAR_RUNTIME_LOG_EVENT_RECOVERY)) ==
              (CAR_RUNTIME_LOG_EVENT_FAULT | CAR_RUNTIME_LOG_EVENT_MODE |
               CAR_RUNTIME_LOG_EVENT_RECOVERY),
          "fault, mode, and recovery transitions are event logged");
    CHECK_EQ("soft fault row retains zero applied software duty",
             car_runtime_log_storage.records[2].left_applied_permille, 0u);

    sample.now_ms = 152u;
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.fault = CAR_FAULT_NONE;
    sample.recovery_active = 0u;
    sample.recovery_frame_count = 0u;
    sample.auto_recovery_count = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("three-frame recovery does not create a new session",
             car_runtime_log_storage.header.session_id, 1u);
    CHECK((car_runtime_log_storage.records[3].event_flags &
           (CAR_RUNTIME_LOG_EVENT_RECOVERY |
            CAR_RUNTIME_LOG_EVENT_AUTO_RECOVERY)) ==
              (CAR_RUNTIME_LOG_EVENT_RECOVERY |
               CAR_RUNTIME_LOG_EVENT_AUTO_RECOVERY),
          "recovery completion and auto-recovery count are event logged");
    CHECK_EQ("recovered Follow session remains active",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);

    sample.now_ms = 153u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    sample.servo_applied_us = 0u;
    sample.motor_requested_permille = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("manual stop freezes the session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("manual switch release is retained as stop reason",
             car_runtime_log_storage.header.stop_reason,
             CAR_RUNTIME_LOG_STOP_MANUAL_RELEASE);
    CHECK_EQ("final stop record timestamp matches header",
             car_runtime_log_storage.records[
                 car_runtime_log_storage.header.count - 1u].time_ms,
             car_runtime_log_storage.header.stop_ms);
    CHECK((car_runtime_log_storage.records[
               car_runtime_log_storage.header.count - 1u].event_flags &
           CAR_RUNTIME_LOG_EVENT_STOP) != 0u,
          "final row has stop event flag");
    CHECK_EQ("final row verifies software servo output is stopped",
             car_runtime_log_storage.records[
                 car_runtime_log_storage.header.count - 1u].servo_applied_us,
             0u);
    CHECK_EQ("incremental records CRC matches committed rows",
             car_runtime_log_storage.header.records_crc32,
             car_runtime_log_crc32(
                 car_runtime_log_storage.records,
                 car_runtime_log_storage.header.count *
                     CAR_RUNTIME_LOG_RECORD_SIZE));
    car_runtime_log_header_t header_copy = car_runtime_log_storage.header;
    const uint32_t header_crc = header_copy.header_crc32;
    header_copy.header_crc32 = 0u;
    CHECK_EQ("header CRC covers frozen metadata",
             header_crc,
             car_runtime_log_crc32(&header_copy, sizeof(header_copy)));
}

void test_runtime_log_captured_start_then_same_iteration_hard_fault()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(77u);
    sample.mode = CAR_MODE_FAULT_LATCHED;
    sample.fault = CAR_FAULT_TIMER;
    sample.fault_generation = 1u;
    sample.follow_session_active = 0u;
    sample.hard_fault_latched = 1u;
    sample.manual_run_request = 0u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;

    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("captured manual start survives same-iteration hard fault",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("hard fault snapshot contains start and final rows",
             car_runtime_log_storage.header.count, 2u);
    CHECK((car_runtime_log_storage.records[0].event_flags &
           CAR_RUNTIME_LOG_EVENT_START) != 0u,
          "first row records captured start edge");
    CHECK((car_runtime_log_storage.records[1].event_flags &
           CAR_RUNTIME_LOG_EVENT_STOP) != 0u,
          "second row records same-iteration stop");
    CHECK_EQ("hard fault determines stop reason",
             car_runtime_log_storage.header.stop_reason,
             CAR_RUNTIME_LOG_STOP_HARD_FAULT);
}

void test_runtime_log_bench_one_shot_freezes()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(500u);
    sample.mode = CAR_MODE_BENCH_RUNNING;
    sample.follow_session_active = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_BENCH);
    CHECK_EQ("manual Bench edge starts the logger",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);

    sample.now_ms = 550u;
    sample.mode = CAR_MODE_BENCH_COMPLETE;
    sample.fault = CAR_FAULT_NONE;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("one-shot Bench completion freezes instead of waiting forever",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("Bench completion gets its own stop reason",
             car_runtime_log_storage.header.stop_reason,
             CAR_RUNTIME_LOG_STOP_BENCH_COMPLETE);
}

void test_runtime_log_pending_stop_waits_for_applied_zero()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(1000u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);

    sample.now_ms = 1001u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    /* Simulate the sample hook observing one last nonzero PWM before apply. */
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("stop remains pending until software PWM is zero",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("pending stop does not publish a false final row",
             car_runtime_log_storage.header.count, 1u);

    sample.now_ms = 1002u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("pending stop freezes when post-apply snapshot is zero",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("final row stores zero applied outputs",
             car_runtime_log_storage.records[
                 car_runtime_log_storage.header.count - 1u].right_applied_permille,
             0u);
}

void test_runtime_log_hold_elapsed_does_not_emit_recovery()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(0u);
    sample.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    sample.vision_hold_active = 1u;
    sample.recovery_frame_count = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);

    sample.now_ms = 50u;
    sample.vision_hold_elapsed_ms = 50u;
    sample.recovery_frame_count = car_runtime_log_pack_hold_progress(
        sample.vision_hold_elapsed_ms, 0u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("50 ms hold timer emits one periodic row", 
             car_runtime_log_storage.header.count, 2u);
    CHECK((car_runtime_log_storage.records[1].event_flags &
           CAR_RUNTIME_LOG_EVENT_RECOVERY) == 0u,
          "elapsed hold ticks alone do not look like route recovery");
    CHECK((car_runtime_log_storage.records[1].event_flags &
           CAR_RUNTIME_LOG_EVENT_PERIODIC) != 0u,
          "mode 12 progress is sampled at the 50 ms production period");

    sample.now_ms = 100u;
    sample.vision_hold_elapsed_ms = 100u;
    sample.recovery_frame_count = car_runtime_log_pack_hold_progress(
        sample.vision_hold_elapsed_ms, 1u);
    sample.vision_hold_valid_count = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("one valid recovery frame appends one event row",
             car_runtime_log_storage.header.count, 3u);
    CHECK((car_runtime_log_storage.records[2].event_flags &
           CAR_RUNTIME_LOG_EVENT_RECOVERY) != 0u,
          "hold recovery-frame change remains observable");

    CHECK_EQ("2999 ms quantizes to the 59th 50 ms hold tick",
             car_runtime_log_pack_hold_progress(2999u, 3u),
             static_cast<uint8_t>((59u << 2u) | 3u));
    CHECK_EQ("3000 ms quantizes to the 60th 50 ms hold tick",
             car_runtime_log_pack_hold_progress(3000u, 2u),
             static_cast<uint8_t>((60u << 2u) | 2u));
    CHECK_EQ("hold log progress saturates at 3000 ms",
             car_runtime_log_pack_hold_progress(0xFFFFFFFFu, 1u),
             static_cast<uint8_t>((60u << 2u) | 1u));
}

void test_runtime_log_slope_flags_keep_fixed_record_abi()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(0u);
    sample.button_start_event_count = 1u;
    sample.left_applied_permille = 507u;
    sample.right_applied_permille = 493u;
    sample.vision_error_norm = 545;
    sample.vision_sequence = 0x1234u;
    sample.motor_ccr2_counts = 507u;
    sample.motor_ccr3_counts = 493u;
    sample.vision_flags |= CAR_RUNTIME_LOG_VISION_LATEST_SLOPE_HINT |
                           CAR_RUNTIME_LOG_VISION_SLOPE_ASSIST_COMMANDED;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("fixed runtime record remains 64 bytes",
             sizeof(car_runtime_log_record_t), 64u);
    CHECK_EQ("new runtime records use version 2",
             CAR_RUNTIME_LOG_VERSION, 2u);
    CHECK_EQ("production log period remains 50 ms",
             CAR_RUNTIME_LOG_PERIOD_MS, 50u);
    CHECK_EQ("ROM descriptor format remains version 1",
             car_runtime_log_rom_descriptor.descriptor_version, 1u);
    CHECK_EQ("ROM descriptor advertises runtime log version 2",
             car_runtime_log_rom_descriptor.log_version, 2u);
    CHECK_EQ("runtime header advertises version 2",
             car_runtime_log_storage.header.version, 2u);
    CHECK_EQ("runtime header keeps its existing 50 ms period",
             car_runtime_log_storage.header.period_ms, 50u);
    CHECK_EQ("vision flags remain at their existing record byte",
             offsetof(car_runtime_log_record_t, vision_flags), 61u);
    CHECK_EQ("latest RX error stays at offset 48 in version 2",
             offsetof(car_runtime_log_record_t, vision_error_norm), 48u);
    CHECK_EQ("latest RX sequence stays at offset 50 in version 2",
             offsetof(car_runtime_log_record_t, vision_sequence), 50u);
    CHECK_EQ("latest CRC-valid error remains independently serialized",
             car_runtime_log_storage.records[0].vision_error_norm, 545);
    CHECK_EQ("latest CRC-valid sequence remains independently serialized",
             car_runtime_log_storage.records[0].vision_sequence, 0x1234u);
    CHECK_EQ("new diagnostic bits are copied into byte 61 unchanged",
             car_runtime_log_storage.records[0].vision_flags,
             sample.vision_flags);
    CHECK_EQ("left Follow output is serialized from its own applied value",
             car_runtime_log_storage.records[0].left_applied_permille, 507u);
    CHECK_EQ("right Follow output is serialized independently",
             car_runtime_log_storage.records[0].right_applied_permille, 493u);

    sample.now_ms = 50u;
    sample.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    sample.vision_hold_active = 1u;
    sample.vision_flags |= CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE;
    sample.vision_error_norm = 0;
    sample.vision_sequence = 0x5678u;
    sample.vision_hold_elapsed_ms = 50u;
    sample.recovery_frame_count = car_runtime_log_pack_hold_progress(50u, 0u);
    sample.left_applied_permille = 486u;
    sample.right_applied_permille = 514u;
    sample.motor_ccr2_counts = 486u;
    sample.motor_ccr3_counts = 514u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK((car_runtime_log_storage.records[1].vision_flags &
           CAR_RUNTIME_LOG_VISION_SLOPE_ASSIST_COMMANDED) != 0u,
          "hold record retains the inherited assist-command diagnostic");
    CHECK_EQ("mode 12 keeps offset 48 as the latest invalid V1 error",
             car_runtime_log_storage.records[1].vision_error_norm, 0);
    CHECK_EQ("mode 12 keeps offset 50 as the latest V1 sequence",
             car_runtime_log_storage.records[1].vision_sequence, 0x5678u);
    CHECK_EQ("second asymmetric pair remains left=486 in the fixed record",
             car_runtime_log_storage.records[1].left_applied_permille, 486u);
    CHECK_EQ("second asymmetric pair remains right=514 in the fixed record",
             car_runtime_log_storage.records[1].right_applied_permille, 514u);

    sample.now_ms = 100u;
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.vision_hold_active = 0u;
    sample.vision_flags &= (uint8_t)~CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE;
    sample.left_applied_permille = 700u;
    sample.right_applied_permille = 300u;
    sample.motor_ccr2_counts = 700u;
    sample.motor_ccr3_counts = 300u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("third one-sided case preserves 700 left in 64-byte record",
             car_runtime_log_storage.records[2].left_applied_permille, 700u);
    CHECK_EQ("third one-sided case preserves 300 right in 64-byte record",
             car_runtime_log_storage.records[2].right_applied_permille, 300u);
}

void test_runtime_log_fault_cancel_is_not_physical_button_stop()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(10u);
    sample.button_start_event_count = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);

    CHECK((car_runtime_log_storage.records[0].event_flags &
           CAR_RUNTIME_LOG_EVENT_BUTTON_START) != 0u,
          "consumed physical PE7 start event is recorded");

    sample.now_ms = 11u;
    sample.mode = CAR_MODE_FAULT_LATCHED;
    sample.fault = CAR_FAULT_TIMER;
    sample.fault_generation = 1u;
    sample.hard_fault_latched = 1u;
    sample.manual_run_request = 0u; /* hard-fault cancellation, not PE7 */
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);

    const uint16_t final_events = car_runtime_log_storage.records[
        car_runtime_log_storage.header.count - 1u].event_flags;
    CHECK_EQ("fault-cancelled request still freezes its session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK((final_events & CAR_RUNTIME_LOG_EVENT_STOP) != 0u,
          "fault-cancelled request retains the session stop event");
    CHECK((final_events & CAR_RUNTIME_LOG_EVENT_BUTTON_STOP) == 0u,
          "request cancellation is not mislabeled as a physical PE7 stop");
}

void test_runtime_log_physical_button_counter_edges()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(100u);

    /* Reconcile the real physical counter even when the one-loop token was
       lost before the logger's first publication call. */
    sample.button_start_event_count = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("physical PE7 counter starts an idle Follow logger",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("reconciled physical start publishes its first row",
             car_runtime_log_storage.header.count, 1u);
    const uint16_t first_events =
        car_runtime_log_storage.records[0].event_flags;
    CHECK((first_events & CAR_RUNTIME_LOG_EVENT_START) != 0u,
          "reconciled row carries the session START event");
    CHECK((first_events & CAR_RUNTIME_LOG_EVENT_BUTTON_START) != 0u,
          "reconciled row retains the physical BUTTON_START event");
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("late explicit token cannot start a duplicate session",
             car_runtime_log_storage.header.session_id, 1u);
    CHECK_EQ("late explicit token does not duplicate the first row",
             car_runtime_log_storage.header.count, 1u);

    sample.now_ms = 101u;
    sample.button_start_event_count = 2u;
    sample.button_stop_generation = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    const uint16_t events = car_runtime_log_storage.records[1].event_flags;
    CHECK((events & CAR_RUNTIME_LOG_EVENT_BUTTON_START) != 0u,
          "start counter delta is logged independently");
    CHECK((events & CAR_RUNTIME_LOG_EVENT_BUTTON_STOP) != 0u,
          "stop generation delta is logged even when both counters change");
}

void test_runtime_log_automatic_follow_never_starts_or_replaces_session()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(200u);
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.follow_session_active = 1u;

    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("automatic Follow state does not start logging",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_IDLE);

    sample.button_start_event_count = 1u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("explicit physical Follow start creates session one",
             car_runtime_log_storage.header.session_id, 1u);

    sample.now_ms = 201u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    sample.button_stop_generation = 1u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("physical stop freezes the one recorded session",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);

    sample.now_ms = 202u;
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.fault = CAR_FAULT_NONE;
    sample.follow_session_active = 1u;
    sample.manual_run_request = 1u;
    sample.servo_applied_us = 1530u;
    sample.left_applied_permille = 120u;
    sample.right_applied_permille = 120u;
    sample.motor_ccr1_counts = 120u;
    sample.motor_ccr4_counts = 120u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("automatic Follow recovery cannot replace frozen RAM evidence",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("automatic Follow recovery preserves prior session identity",
             car_runtime_log_storage.header.session_id, 1u);

    sample.now_ms = 203u;
    sample.mode = CAR_MODE_FOLLOW_RUNNING;
    sample.fault = CAR_FAULT_NONE;
    sample.follow_session_active = 1u;
    sample.manual_run_request = 1u;
    sample.button_start_event_count = 2u;
    sample.button_stop_generation = 1u;
    sample.servo_applied_us = 1530u;
    sample.left_applied_permille = 120u;
    sample.right_applied_permille = 120u;
    sample.motor_ccr1_counts = 120u;
    sample.motor_ccr4_counts = 120u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("new physical start counter begins the next frozen-session log",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("new physical start increments session identity once",
             car_runtime_log_storage.header.session_id, 2u);
    CHECK((car_runtime_log_storage.records[0].event_flags &
           CAR_RUNTIME_LOG_EVENT_BUTTON_START) != 0u,
          "new session records the actual button start");
}

void test_runtime_log_restart_overwrites_header_without_buffer_clear()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(2000u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    sample.now_ms = 2001u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("precondition first session freezes",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("precondition first session id is one",
             car_runtime_log_storage.header.session_id, 1u);

    car_runtime_log_storage.records[100].time_ms = 0xA55AA55Au;
    sample = make_runtime_log_follow_sample(3000u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    CHECK_EQ("new manual session starts after prior freeze",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
    CHECK_EQ("new session advances session id",
             car_runtime_log_storage.header.session_id, 2u);
    CHECK_EQ("new session exposes only its new start row",
             car_runtime_log_storage.header.count, 1u);
    CHECK_EQ("restart does not clear the entire RAM buffer",
             car_runtime_log_storage.records[100].time_ms, 0xA55AA55Au);
    car_runtime_log_header_t header_copy = car_runtime_log_storage.header;
    const uint32_t header_crc = header_copy.header_crc32;
    header_copy.header_crc32 = 0u;
    CHECK_EQ("new session header CRC is valid after reset",
             header_crc,
             car_runtime_log_crc32(&header_copy, sizeof(header_copy)));
}

void test_runtime_log_capacity_rate_limits_dropped_rows()
{
    car_runtime_log_init();
    car_runtime_log_sample_t sample = make_runtime_log_follow_sample(0u);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);

    for (uint32_t index = 1u;
         index < CAR_RUNTIME_LOG_CAPACITY - 1u; ++index) {
        sample.now_ms = index * CAR_RUNTIME_LOG_PERIOD_MS;
        car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    }
    CHECK_EQ("normal rows leave exactly one final-stop slot",
             car_runtime_log_storage.header.count,
             CAR_RUNTIME_LOG_CAPACITY - 1u);
    CHECK((car_runtime_log_storage.header.flags & CAR_RUNTIME_LOG_FLAG_FULL) != 0u,
          "normal capacity sets full before a sample is dropped");
    CHECK((car_runtime_log_storage.header.flags &
           CAR_RUNTIME_LOG_FLAG_TRUNCATED) == 0u,
          "full capacity alone does not claim a dropped sample");
    CHECK_EQ("full capacity begins with zero dropped count",
             car_runtime_log_storage.header.dropped_count, 0u);

    const uint32_t base_ms = sample.now_ms;
    for (uint32_t delta = 1u; delta <= 200u; ++delta) {
        sample.now_ms = base_ms + delta;
        car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    }
    CHECK_EQ("full-buffer periodic drops remain limited to 20 Hz",
             car_runtime_log_storage.header.dropped_count, 4u);
    CHECK((car_runtime_log_storage.header.flags &
           CAR_RUNTIME_LOG_FLAG_TRUNCATED) != 0u,
          "dropped periodic rows set truncated flag");
    CHECK_EQ("overflow does not consume reserved final stop slot",
             car_runtime_log_storage.header.count,
             CAR_RUNTIME_LOG_CAPACITY - 1u);

    sample.now_ms = base_ms + 201u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    sample.servo_applied_us = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);
    CHECK_EQ("reserved final slot records and freezes stop",
             car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_FROZEN);
    CHECK_EQ("final stop may fill physical capacity",
             car_runtime_log_storage.header.count,
             CAR_RUNTIME_LOG_CAPACITY);
    CHECK_EQ("final physical capacity retains full marker",
             car_runtime_log_storage.header.flags & CAR_RUNTIME_LOG_FLAG_FULL,
             CAR_RUNTIME_LOG_FLAG_FULL);
}

void test_run_block_reason_reports_first_gate()
{
    car_run_block_reason_input_t input{};
    input.startup_release_seen = 1u;
    input.vision_fresh_valid = 1u;
    input.reverse_reset_keep_ready = 1u;
    input.profile_output_permission = 1u;
    input.follow_mode_active = 1u;
    input.follow_session_active = 1u;
    input.follow_output_permission = 1u;
    input.servo_output_active = 1u;
    input.servo_first_frame_ready = 1u;
    input.manual_run_request = 1u;

    CHECK_EQ("null input is conservatively reported as a fault",
             car_run_block_reason_evaluate(nullptr), CAR_RUN_BLOCK_FAULT);
    input.startup_release_seen = 0u;
    CHECK_EQ("startup release gate has reason 1",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_WAIT_STARTUP_RELEASE);
    input.startup_release_seen = 1u;
    input.fault_present = 1u;
    CHECK_EQ("fault has reason 2",
             car_run_block_reason_evaluate(&input), CAR_RUN_BLOCK_FAULT);
    input.fault_present = 0u;
    input.recovery_pending = 1u;
    CHECK_EQ("three-frame recovery gate has reason 4",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_RECOVERY_PENDING);
    input.recovery_pending = 0u;
    input.vision_fresh_valid = 0u;
    CHECK_EQ("vision freshness gate has reason 3",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_NO_FRESH_VALID_VISION);
    input.vision_fresh_valid = 1u;
    input.vision_hold_expired = 1u;
    CHECK_EQ("expired vision hold is separately reported",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_HOLD_EXPIRED_WAIT_ROUTE);
    input.vision_hold_expired = 0u;
    input.reverse_reset_keep_ready = 0u;
    CHECK_EQ("camera RESET/KEEP gate has reason 5",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_REVERSE_RESET_KEEP_GATE);
    input.reverse_reset_keep_ready = 1u;
    input.profile_output_permission = 0u;
    CHECK_EQ("compiled or accepted output permission has reason 7",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_SOFTWARE_PERMISSION_OFF);
    input.profile_output_permission = 1u;
    input.follow_mode_active = 0u;
    input.manual_run_request = 0u;
    CHECK_EQ("fresh PE7 press edge gate has reason 6",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_WAIT_PE7_PRESS_EDGE);
    input.manual_run_request = 1u;
    input.follow_mode_active = 1u;
    input.follow_session_active = 0u;
    input.manual_run_request = 0u;
    CHECK_EQ("inactive PE7 session has reason 6",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_WAIT_PE7_PRESS_EDGE);
    input.manual_run_request = 1u;
    input.follow_session_active = 1u;
    input.follow_output_permission = 0u;
    CHECK_EQ("dynamic software output permission has reason 7",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_SOFTWARE_PERMISSION_OFF);
    input.follow_output_permission = 1u;
    input.servo_stop_pending = 1u;
    CHECK_EQ("pending safe servo stop has reason 9",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_SERVO_STOP_PENDING);
    input.servo_stop_pending = 0u;
    input.servo_output_active = 0u;
    CHECK_EQ("inactive servo output has reason 8",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME);
    input.servo_output_active = 1u;
    input.servo_first_frame_ready = 0u;
    CHECK_EQ("first servo PWM frame gate has reason 8",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_WAIT_SERVO_FIRST_FRAME);
    input.servo_first_frame_ready = 1u;
    CHECK_EQ("ramp or another output guard has reason 10",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD);
    input.motor_left_applied = 1u;
    CHECK_EQ("one side alone is not reported as both outputs released",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_RAMP_OR_OTHER_OUTPUT_GUARD);
    input.motor_right_applied = 1u;
    CHECK_EQ("reason zero means both Follow software outputs are applied",
             car_run_block_reason_evaluate(&input),
             CAR_RUN_BLOCK_RUNNING_WITH_MOTOR_OUTPUT);
}

struct LoggingToggleTrace {
    car_mode_t mode;
    car_fault_t fault;
    uint8_t fault_latched;
    uint8_t follow_session_active;
    uint8_t road_drive_allowed;
    uint8_t recovery_active;
    uint8_t recovery_frame_count;
    uint32_t auto_recovery_count;
    uint16_t follow_output_duty_permille;
};

LoggingToggleTrace run_follow_recovery_trace(bool logging_enabled)
{
    const car_config_t config = make_follow_config();
    car_control_t control{};
    start_follow_by_physical_pe7(&control, &config, 1u, 100u);

    if (logging_enabled) {
        car_runtime_log_init();
    } else {
        car_runtime_log_storage.records[77].time_ms = 0xA55AA55Au;
        car_runtime_log_invalidate();
    }

    auto update_log = [&](uint32_t now_ms,
                          car_runtime_log_session_kind_t start_kind) {
        if (!logging_enabled) {
            return;
        }
        car_runtime_log_sample_t sample =
            make_runtime_log_follow_sample(now_ms);
        sample.mode = static_cast<uint8_t>(control.mode);
        sample.fault = static_cast<uint8_t>(control.fault);
        sample.follow_session_active = control.follow_session_active;
        sample.recovery_active = control.recovery_active;
        sample.recovery_frame_count = control.recovery_frame_count;
        sample.auto_recovery_count = control.auto_recovery_count;
        sample.hard_fault_latched = control.fault_latched;
        if (control.follow_session_active == 0u) {
            sample.servo_applied_us = 0u;
            sample.left_applied_permille = 0u;
            sample.right_applied_permille = 0u;
            sample.motor_ccr1_counts = 0u;
            sample.motor_ccr2_counts = 0u;
            sample.motor_ccr3_counts = 0u;
            sample.motor_ccr4_counts = 0u;
        }
        car_runtime_log_update(&sample, start_kind);
    };

    update_log(124u, CAR_RUNTIME_LOG_SESSION_FOLLOW);
    for (uint16_t sequence = 4u; sequence <= 6u; ++sequence) {
        const uint32_t now_ms = 125u + sequence;
        send_frame_at(&control, &config, sequence, true, 80u, 200,
                      now_ms - 1u, now_ms, false);
        car_inputs_t tick = make_inputs(now_ms, false);
        car_control_tick(&control, &config, &tick);
        update_log(now_ms, CAR_RUNTIME_LOG_SESSION_NONE);
    }

    car_control_note_fault(&control, CAR_FAULT_PROTOCOL);
    car_control_rx_recovery_flush_complete(&control);
    car_control_reverse_control_keep_started(&control);
    for (uint16_t sequence = 7u; sequence <= 9u; ++sequence) {
        const uint32_t now_ms = 140u + sequence;
        send_frame_at(&control, &config, sequence, true, 80u, 100,
                      now_ms - 1u, now_ms, false);
        car_inputs_t tick = make_inputs(now_ms, false);
        car_control_tick(&control, &config, &tick);
        update_log(now_ms, CAR_RUNTIME_LOG_SESSION_NONE);
    }

    if (logging_enabled) {
        CHECK_EQ("ON variant records the manual session",
                 car_runtime_log_state(), CAR_RUNTIME_LOG_STATE_ACTIVE);
        CHECK(car_runtime_log_storage.header.count >= 2u,
              "ON variant records runtime rows during the trace");
    } else {
        const auto *header_bytes = reinterpret_cast<const uint8_t *>(
            &car_runtime_log_storage.header);
        CHECK(std::all_of(header_bytes,
                          header_bytes + sizeof(car_runtime_log_header_t),
                          [](uint8_t value) { return value == 0u; }),
              "OFF variant keeps the header invalid after initialization");
        CHECK_EQ("OFF variant never creates an active/frozen session",
                 car_runtime_log_state(), 0u);
        CHECK_EQ("OFF variant leaves prior records untouched",
                 car_runtime_log_storage.records[77].time_ms,
                 0xA55AA55Au);
        car_runtime_log_sample_t unexpected_start =
            make_runtime_log_follow_sample(200u);
        car_runtime_log_update(&unexpected_start,
                               CAR_RUNTIME_LOG_SESSION_FOLLOW);
        CHECK_EQ("OFF logger rejects even an accidental session update",
                 car_runtime_log_state(), 0u);
        CHECK_EQ("OFF logger preserves old records after accidental update",
                 car_runtime_log_storage.records[77].time_ms,
                 0xA55AA55Au);
    }

    return LoggingToggleTrace{
        control.mode,
        control.fault,
        control.fault_latched,
        control.follow_session_active,
        control.road_drive_allowed,
        control.recovery_active,
        control.recovery_frame_count,
        control.auto_recovery_count,
        control.follow_output_duty_permille
    };
}

void test_logging_toggle_does_not_change_follow_recovery()
{
    const LoggingToggleTrace logging_on = run_follow_recovery_trace(true);
    const LoggingToggleTrace logging_off = run_follow_recovery_trace(false);
    CHECK_EQ("logging toggle does not change controller mode",
             logging_off.mode, logging_on.mode);
    CHECK_EQ("logging toggle does not change controller fault",
             logging_off.fault, logging_on.fault);
    CHECK_EQ("logging toggle does not change hard-fault latch",
             logging_off.fault_latched, logging_on.fault_latched);
    CHECK_EQ("logging toggle does not change manual session state",
             logging_off.follow_session_active, logging_on.follow_session_active);
    CHECK_EQ("logging toggle does not change drive permission",
             logging_off.road_drive_allowed, logging_on.road_drive_allowed);
    CHECK_EQ("logging toggle does not change recovery active state",
             logging_off.recovery_active, logging_on.recovery_active);
    CHECK_EQ("logging toggle does not change recovery frame count",
             logging_off.recovery_frame_count, logging_on.recovery_frame_count);
    CHECK_EQ("logging toggle does not change auto recovery count",
             logging_off.auto_recovery_count, logging_on.auto_recovery_count);
    CHECK_EQ("logging toggle does not change Follow duty",
             logging_off.follow_output_duty_permille,
             logging_on.follow_output_duty_permille);
}

void test_speed_observer_signs_jitter_and_clock_wrap()
{
    car_speed_observer_t observer{};
    car_speed_observer_result_t result{};
    const uint32_t start = 0xFFFFFFF0u;
    car_speed_observer_init(&observer, start, 100, 200,
                            60000u, 1, 60000u, -1);

    CHECK_EQ("speed observer waits for 10ms", car_speed_observer_update(
                 &observer, start + 5u, 100, 200, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_NOT_READY);
    CHECK_EQ("speed observer samples across wrap", car_speed_observer_update(
                 &observer, start + 20u, 120, 170, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("left forward sign gives positive mRPS",
             result.left_millirps, 16);
    CHECK_EQ("right inverted sign gives positive mRPS",
             result.right_millirps, 25);
    CHECK_EQ("actual elapsed interval recorded", result.elapsed_ms, 20u);

    CHECK_EQ("jittered sample uses 15ms actual dt",
             car_speed_observer_update(&observer, start + 35u, 135, 155,
                                       1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("jittered speed remains calibrated", result.left_millirps, 16);
    CHECK_EQ("jittered elapsed interval", result.elapsed_ms, 15u);

    CHECK_EQ("delayed loop produces one sample only",
             car_speed_observer_update(&observer, start + 65u, 165, 125,
                                       1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("delayed loop does not replay missed periods",
             result.elapsed_ms, 30u);
}

void test_speed_observer_invalid_inputs_and_stall()
{
    car_speed_observer_t observer{};
    car_speed_observer_result_t result{};
    car_speed_observer_init(&observer, 0u, 0, 0,
                            60000u, 1, 60000u, -1);
    CHECK_EQ("encoderless zero window at 10ms",
             car_speed_observer_update(&observer, 10u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("jittered 20ms stall window",
             car_speed_observer_update(&observer, 30u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("jittered 100ms stall window",
             car_speed_observer_update(&observer, 130u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("second 100ms stall window",
             car_speed_observer_update(&observer, 230u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("299ms continuous zero is not yet a stall",
             car_speed_observer_update(&observer, 299u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("zero timer reaches 299ms", observer.left_zero_expected_ms, 299u);
    CHECK_EQ("one millisecond cannot bypass 10ms sampler cadence",
             car_speed_observer_update(&observer, 300u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_WAITING);
    CHECK_EQ("stall is reported when next eligible sample crosses 300ms",
             car_speed_observer_update(&observer, 309u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("stall revokes speed validity", result.valid, 0u);
    CHECK_EQ("stall remains visible before next 10ms sample",
             car_speed_observer_update(&observer, 310u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("persistent broken encoder remains invalid after 100ms",
             car_speed_observer_update(&observer, 409u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("persistent stall remains visible after another 100ms",
             car_speed_observer_update(&observer, 509u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("persistent stall remains invalid at 600ms",
             car_speed_observer_update(&observer, 600u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("persistent stall speed stays invalid", result.valid, 0u);
    CHECK_EQ("real positive encoder counts restore observation",
             car_speed_observer_update(&observer, 610u, 10, -10, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("movement restores observer validity", result.valid, 1u);

    car_speed_observer_init(&observer, 0u, 0, 0,
                            60000u, 1, 60000u, -1);
    (void)car_speed_observer_update(&observer, 10u, 0, 0, 1u, &result);
    (void)car_speed_observer_update(&observer, 30u, 0, 0, 1u, &result);
    (void)car_speed_observer_update(&observer, 130u, 0, 0, 1u, &result);
    (void)car_speed_observer_update(&observer, 230u, 0, 0, 1u, &result);
    CHECK_EQ("10/20/100ms jitter reaches exact 300ms stall boundary",
             car_speed_observer_update(&observer, 300u, 0, 0, 1u, &result),
             CAR_SPEED_OBSERVER_STATUS_STALL);
    CHECK_EQ("standby clears persistent stall diagnostic",
             car_speed_observer_update(&observer, 310u, 0, 0, 0u, &result),
             CAR_SPEED_OBSERVER_STATUS_SAMPLE);
    CHECK_EQ("standby clears accumulated zero time",
             observer.left_zero_expected_ms, 0u);

    car_speed_observer_init(&observer, 0u, 0, 0,
                            60000u, 1, 60000u, -1);
    (void)car_speed_observer_update(&observer, 100u, 0, 0, 1u, &result);
    CHECK_EQ("zero timer accumulates only while output expects motion",
             observer.left_zero_expected_ms, 100u);
    (void)car_speed_observer_update(&observer, 110u, 0, 0, 0u, &result);
    CHECK_EQ("stop clears stall time before restart",
             observer.left_zero_expected_ms, 0u);
    (void)car_speed_observer_update(&observer, 120u, 0, 0, 1u, &result);
    CHECK_EQ("restart begins new zero interval",
             observer.left_zero_expected_ms, 10u);

    car_speed_observer_init(&observer, 0u, 0, 0,
                            60000u, 1, 60000u, -1);
    CHECK_EQ("normalized reverse count is invalid",
             car_speed_observer_update(&observer, 10u, -1, 0, 0u, &result),
             CAR_SPEED_OBSERVER_STATUS_REVERSE_DELTA);

    car_speed_observer_init(&observer, 0u, 0, 0,
                            60000u, 1, 60000u, -1);
    CHECK_EQ("long loop gap invalidates baseline",
             car_speed_observer_update(&observer, 101u, 5, -5, 0u, &result),
             CAR_SPEED_OBSERVER_STATUS_CLOCK_GAP);
    CHECK_EQ("gap invalidates speed", result.valid, 0u);

    car_speed_observer_init(&observer, 100u, INT64_MIN, 0,
                            60000u, 1, 60000u, -1);
    CHECK_EQ("cumulative-count subtraction overflow is rejected",
             car_speed_observer_update(&observer, 110u, INT64_MAX, 0, 0u,
                                       &result),
             CAR_SPEED_OBSERVER_STATUS_COUNT_RANGE);

    car_speed_observer_init(&observer, 0x100u, 0, 0,
                            60000u, 1, 60000u, -1);
    CHECK_EQ("backward half-range clock is rejected",
             car_speed_observer_update(&observer, 0xFFFFFF00u, 0, 0, 0u,
                                       &result),
             CAR_SPEED_OBSERVER_STATUS_CLOCK_GAP);
}

void test_speed_pi_shadow_math_gates_and_antiwindup()
{
    car_speed_pi_state_t state{};
    car_speed_pi_output_t output{};
    car_speed_pi_config_t config{1u, 700u, 65536u, 65536u};
    car_speed_pi_input_t input{};
    input.lifecycle = CAR_SPEED_PI_LIFECYCLE_RUNNING;
    input.speed_sample_valid = 1u;
    input.feedforward_permille = 100u;
    input.dt_ms = 100u;
    input.target_millirps = 100;
    input.measured_millirps = 100;
    CHECK_EQ("PI steady baseline proposal is feedforward",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    const uint16_t baseline_duty = output.requested_permille;
    input.measured_millirps = 80;
    CHECK_EQ("PI loadstep proposal is computed",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("PI loadstep includes feedforward and proportional term",
             output.requested_permille, 122u);
    CHECK(output.requested_permille > baseline_duty,
          "simulated load-induced speed drop increases proposal");
    CHECK_EQ("PI library proposal is permission-gated until configured/run",
             output.motor_permission, 1u);

    config.kp_q16 = CAR_SPEED_PI_MAX_GAIN_Q16;
    config.ki_q16 = CAR_SPEED_PI_MAX_GAIN_Q16;
    input.feedforward_permille = 690u;
    input.target_millirps = 1000;
    input.measured_millirps = 0;
    state.integral_q16 = 0;
    CHECK_EQ("PI output saturates at independent 700 cap",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("PI cap remains 700", output.requested_permille, 700u);
    CHECK_EQ("PI reports saturation", output.saturated, 1u);
    CHECK_EQ("conditional integration prevents upper windup",
             state.integral_q16, INT64_C(0));
    input.measured_millirps = 1010;
    CHECK_EQ("PI recovers below cap when error reverses",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK(output.requested_permille < 700u,
          "reverse error lowers requested output");

    input.measured_millirps = 0;
    input.downstream_applied_valid = 1u;
    input.downstream_applied_permille = 400u;
    const int64_t integral_before_tracking = state.integral_q16;
    const int64_t expected_tracked_integral = integral_before_tracking +
        ((int64_t)input.downstream_applied_permille -
         state.requested_permille) * INT64_C(65536);
    (void)car_speed_pi_update(&state, &config, &input, &output);
    CHECK_EQ("tracking compares prior applied duty with prior proposal",
             state.integral_q16, expected_tracked_integral);

    input.target_millirps = 0;
    CHECK_EQ("zero target has no permission",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_INVALID);
    CHECK_EQ("zero target output is zero", output.motor_permission, 0u);
    CHECK_EQ("zero target clears stored integral", state.integral_q16,
             INT64_C(0));

    input.target_millirps = 100;
    input.lifecycle = CAR_SPEED_PI_LIFECYCLE_HOLD;
    state.integral_q16 = 12345;
    CHECK_EQ("hold freezes PI integrator and revokes permission",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_HOLD);
    CHECK_EQ("hold preserves integral", state.integral_q16, INT64_C(12345));
    CHECK_EQ("hold does not permit motor output", output.motor_permission, 0u);

    input.lifecycle = CAR_SPEED_PI_LIFECYCLE_STARTUP;
    CHECK_EQ("startup resets PI state",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_STARTUP);
    CHECK_EQ("startup clears integral", state.integral_q16, INT64_C(0));
    CHECK_EQ("startup has no motor permission", output.motor_permission, 0u);

    input.lifecycle = CAR_SPEED_PI_LIFECYCLE_RUNNING;
    input.speed_sample_valid = 0u;
    CHECK_EQ("invalid encoder sample blocks PI",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_INVALID);
    CHECK_EQ("invalid encoder sample has no permission",
             output.motor_permission, 0u);
    CHECK_EQ("invalid encoder sample clears stale integral",
             state.integral_q16, INT64_C(0));

    config.configured = 0u;
    input.speed_sample_valid = 1u;
    CHECK_EQ("unconfigured gains block PI",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_DISABLED);
    CHECK_EQ("unconfigured PI cannot request motor output",
             output.motor_permission, 0u);
    CHECK_EQ("unconfigured PI clears stored integral", state.integral_q16,
             INT64_C(0));
    config.configured = 1u;
    config.output_cap_permille = 701u;
    CHECK_EQ("cap above independent 700 rejected",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_INVALID);
    CHECK_EQ("invalid cap has no permission", output.motor_permission, 0u);

    config.output_cap_permille = CAR_SPEED_PI_MAX_OUTPUT_PERMILLE;
    config.kp_q16 = CAR_SPEED_PI_MAX_GAIN_Q16;
    config.ki_q16 = CAR_SPEED_PI_MAX_GAIN_Q16;
    input.target_millirps = CAR_SPEED_PI_MAX_SPEED_MILLIRPS;
    input.measured_millirps = 0;
    input.dt_ms = CAR_SPEED_PI_MAX_DT_MS;
    input.feedforward_permille = 0u;
    input.downstream_applied_valid = 0u;
    CHECK_EQ("maximum bounded PI inputs remain defined",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("maximum bounded PI input saturates at 700",
             output.requested_permille, 700u);
    CHECK_EQ("maximum bounded PI input reports saturation",
             output.saturated, 1u);

    config.kp_q16 = 0u;
    config.ki_q16 = 0u;
    config.output_cap_permille = 700u;
    input.target_millirps = 100;
    input.measured_millirps = 100;
    input.dt_ms = 10u;
    input.feedforward_permille = 0u;
    input.downstream_applied_valid = 0u;
    state.integral_q16 = INT64_C(699) * INT64_C(65536);
    state.previous_proposal_valid = 0u;
    CHECK_EQ("large-cap PI state is valid before cap change",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    config.output_cap_permille = 400u;
    CHECK_EQ("lowered cap bounds existing PI state immediately",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK(state.integral_q16 <= INT64_C(400) * INT64_C(65536),
          "stored integral respects newly lowered cap");
    CHECK(output.requested_permille <= 400u,
          "proposal respects newly lowered cap");

    car_speed_pi_reset(&state);
    config = car_speed_pi_config_t{1u, 700u, 0u, 0u, 200u, 0u};
    input = car_speed_pi_input_t{};
    input.lifecycle = CAR_SPEED_PI_LIFECYCLE_RUNNING;
    input.speed_sample_valid = 1u;
    input.feedforward_permille = 100u;
    input.dt_ms = 1000u;
    input.target_millirps = 100;
    input.measured_millirps = 100;
    CHECK_EQ("tracking-time controller establishes its prior proposal",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    input.downstream_applied_valid = 1u;
    input.downstream_applied_permille = 300u;
    CHECK_EQ("tracking-time back-calculation accepts bounded long sample",
             car_speed_pi_update(&state, &config, &input, &output),
             CAR_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("tracking correction clamps dt to tracking time",
             state.integral_q16, INT64_C(200) * INT64_C(65536));
}

void test_follow_speed_pi_adapter_sample_slew_hold_and_recovery()
{
    car_follow_speed_pi_state_t state{};
    car_follow_speed_pi_input_t input{};
    uint16_t command_left = 0u;
    uint16_t command_right = 0u;

    input.enabled = 1u;
    CHECK_EQ("PI adapter waits behind existing startup qualification",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_NOT_READY);
    CHECK_EQ("startup gate leaves PI unarmed", state.armed, 0u);

    input.follow_running = 1u;
    input.startup_qualified = 1u;
    input.actual_motor_outputs = 1u;
    input.servo_ready = 1u;
    input.speed_sample_valid = 1u;
    input.new_speed_sample = 1u;
    input.sample_ms = 10u;
    input.sample_elapsed_ms = 10u;
    input.sample_age_ms = 0u;
    input.nominal_left_permille = 500u;
    input.nominal_right_permille = 500u;
    input.applied_left_permille = 500u;
    input.applied_right_permille = 500u;
    input.measured_left_millirps = 2400;
    input.measured_right_millirps = 2400;
    CHECK_EQ("qualified fresh sample arms the independent PI adapter",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("straight speed target is 2400 milli-RPS",
             state.target_left_millirps, 2400);
    CHECK_EQ("steady speed proposal stays at 500 permille",
             state.proposal_left_permille, 500u);
    CHECK_EQ("steady right proposal stays at 500 permille",
             state.proposal_right_permille, 500u);

    const uint16_t proposal_before_duplicate = state.proposal_left_permille;
    const int64_t integral_before_duplicate = state.left.integral_q16;
    input.new_speed_sample = 0u;
    input.sample_age_ms = 5u;
    input.measured_left_millirps = 1000;
    CHECK_EQ("duplicate observer timestamp does not update PI",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("duplicate timestamp retains proposal",
             state.proposal_left_permille, proposal_before_duplicate);
    CHECK_EQ("duplicate timestamp retains integral",
             state.left.integral_q16, integral_before_duplicate);

    input.new_speed_sample = 1u;
    input.sample_ms = 20u;
    input.sample_age_ms = 0u;
    input.measured_left_millirps = 1800;
    input.measured_right_millirps = 2400;
    input.applied_left_permille = 480u;
    input.applied_right_permille = 500u;
    CHECK_EQ("load drop raises only the affected side proposal",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK(state.proposal_left_permille > 500u,
          "left load drop increases left proposal");
    CHECK_EQ("unloaded side remains at nominal proposal",
             state.proposal_right_permille, 500u);
    CHECK_EQ("nominal left target remains 2400 milli-RPS",
             state.target_left_millirps, 2400);

    CHECK_EQ("PI output ramp advances one 20-permille step",
             car_follow_speed_pi_ramp(&state, 1u, 500u, 500u,
                                      &command_left, &command_right), 1u);
    CHECK_EQ("load correction slew is bounded to 20",
             command_left, 520u);
    CHECK_EQ("unloaded side has no output step",
             command_right, 500u);
    CHECK_EQ("PI output ramp does not step before its 10 ms deadline",
             car_follow_speed_pi_ramp(&state, 0u, 520u, 500u,
                                      &command_left, &command_right), 1u);
    CHECK_EQ("not-due step leaves the applied left output unchanged",
             command_left, 520u);

    input.sample_ms = 30u;
    input.measured_left_millirps = 5000;
    input.measured_right_millirps = 5000;
    input.applied_left_permille = 520u;
    input.applied_right_permille = 500u;
    CHECK_EQ("overspeed sample reduces both side proposals",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK(state.proposal_left_permille < 500u,
          "overspeed reduces left request below feedforward");
    CHECK(state.proposal_right_permille < 500u,
          "overspeed reduces right request below feedforward");

    input.sample_ms = 40u;
    input.measured_left_millirps = 10000;
    input.measured_right_millirps = 10000;
    input.applied_left_permille = 20u;
    input.applied_right_permille = 20u;
    CHECK_EQ("strong overspeed can request zero on both sides",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("zero correction is permitted for left request",
             state.proposal_left_permille, 0u);
    CHECK_EQ("zero correction is permitted for right request",
             state.proposal_right_permille, 0u);
    CHECK_EQ("zero correction keeps PI state armed",
             state.armed, 1u);
    CHECK_EQ("zero correction still ramps without resetting startup",
             car_follow_speed_pi_ramp(&state, 1u, 20u, 20u,
                                      &command_left, &command_right), 1u);
    CHECK_EQ("zero correction reaches zero within the allowed slew",
             command_left, 0u);
    CHECK_EQ("zero correction reaches zero on the second side",
             command_right, 0u);

    input.sample_ms = 50u;
    input.measured_left_millirps = 0;
    input.measured_right_millirps = 0;
    input.applied_left_permille = 0u;
    input.applied_right_permille = 0u;
    CHECK_EQ("underspeed after zero-coast recovers without a startup reset",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("underspeed ramp recovers from actual zero on left",
             car_follow_speed_pi_ramp(&state, 1u, 0u, 0u,
                                      &command_left, &command_right), 1u);
    CHECK_EQ("underspeed recovery starts with a 20-permille left step",
             command_left, 20u);
    CHECK_EQ("underspeed recovery starts with a 20-permille right step",
             command_right, 20u);
    CHECK_EQ("zero-coast recovery remains armed", state.armed, 1u);

    const int64_t hold_integral = state.left.integral_q16;
    input.vision_hold = 1u;
    input.applied_left_permille = command_left;
    input.applied_right_permille = command_right;
    CHECK_EQ("vision hold freezes PI updates", car_follow_speed_pi_update(
                 &state, &input), CAR_FOLLOW_SPEED_PI_STATUS_HOLD);
    CHECK_EQ("vision hold preserves bounded integral",
             state.left.integral_q16, hold_integral);
    CHECK_EQ("vision hold preserves stored proposal",
             state.proposal_left_permille, 700u);

    input.vision_hold = 0u;
    input.new_speed_sample = 0u;
    input.sample_ms = 80u;
    CHECK_EQ("hold resume clears stale proposal history",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_RUNNING);
    CHECK_EQ("hold resume begins at actual applied left output",
             state.proposal_left_permille, command_left);
    CHECK_EQ("hold resume preserves integral without old tracking",
             state.left.integral_q16, hold_integral);

    input.new_speed_sample = 1u;
    input.sample_ms = 90u;
    input.sample_age_ms = 101u;
    CHECK_EQ("stale qualified encoder data invalidates PI",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
    CHECK_EQ("stale data clears PI arm and stored request", state.armed, 0u);
    CHECK_EQ("stale data clears old integral", state.left.integral_q16,
             INT64_C(0));
    CHECK_EQ("invalid PI state cannot ramp a stale command",
             car_follow_speed_pi_ramp(&state, 1u, 200u, 200u,
                                      &command_left, &command_right), 0u);

    input.sample_age_ms = 0u;
    input.speed_sample_valid = 0u;
    CHECK_EQ("invalid duplicate sample also fails closed",
             car_follow_speed_pi_update(&state, &input),
             CAR_FOLLOW_SPEED_PI_STATUS_INVALID);
    CHECK_EQ("invalid sample clears stale target", state.target_left_millirps,
             0);
}

bool emit_runtime_log_fixture(const char *path)
{
    car_runtime_log_sample_t sample =
        make_runtime_log_follow_sample(0xFFFFFFF0u);
    car_runtime_log_init();
    sample.button_start_event_count = 1u;
    sample.left_encoder_counts = INT64_C(0x123456789);
    sample.right_encoder_counts = -INT64_C(0x123456789);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_FOLLOW);

    sample.now_ms = 0x00000022u; /* +50 ms across the 32-bit clock wrap. */
    sample.left_encoder_counts = INT64_C(0x123456789ABCDEF);
    sample.right_encoder_counts = -INT64_C(0x123456789ABCDEF);
    sample.vision_sequence = 8u;
    sample.mode = CAR_MODE_FOLLOW_VISION_HOLD;
    sample.vision_flags |= CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE;
    sample.vision_hold_active = 1u;
    sample.vision_hold_elapsed_ms = 3000u;
    sample.vision_hold_valid_count = 2u;
    sample.recovery_frame_count = car_runtime_log_pack_hold_progress(
        sample.vision_hold_elapsed_ms, sample.vision_hold_valid_count);
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);

    sample.now_ms = 0x00000023u;
    sample.mode = CAR_MODE_WAIT_SWITCH_RELEASE;
    sample.fault = CAR_FAULT_SWITCH_RELEASE;
    sample.follow_session_active = 0u;
    sample.manual_run_request = 0u;
    sample.button_stop_generation = 1u;
    sample.vision_flags &= static_cast<uint8_t>(
        ~CAR_RUNTIME_LOG_VISION_HOLD_ACTIVE);
    sample.vision_hold_active = 0u;
    sample.vision_hold_elapsed_ms = 0u;
    sample.vision_hold_valid_count = 0u;
    sample.vision_hold_exit_cause = CAR_VISION_HOLD_EXIT_RECOVERED;
    sample.servo_applied_us = 0u;
    sample.motor_requested_permille = 0u;
    sample.left_applied_permille = 0u;
    sample.right_applied_permille = 0u;
    sample.motor_ccr1_counts = 0u;
    sample.motor_ccr2_counts = 0u;
    sample.motor_ccr3_counts = 0u;
    sample.motor_ccr4_counts = 0u;
    car_runtime_log_update(&sample, CAR_RUNTIME_LOG_SESSION_NONE);

    const car_runtime_log_header_t &header = car_runtime_log_storage.header;
    if (header.state != CAR_RUNTIME_LOG_STATE_FROZEN || header.count != 3u ||
        header.start_ms != 0xFFFFFFF0u || header.stop_ms != 0x00000023u ||
        header.stop_reason != CAR_RUNTIME_LOG_STOP_MANUAL_RELEASE ||
        header.version != 2u || header.period_ms != 50u ||
        car_runtime_log_rom_descriptor.descriptor_version != 1u ||
        car_runtime_log_storage.records[1].recovery_frame_count !=
            static_cast<uint8_t>((60u << 2u) | 2u) ||
        car_runtime_log_storage.records[1].vision_error_norm !=
            sample.vision_error_norm ||
        car_runtime_log_storage.records[1].vision_sequence != 8u ||
        header.records_crc32 != car_runtime_log_crc32(
            car_runtime_log_storage.records,
            header.count * CAR_RUNTIME_LOG_RECORD_SIZE)) {
        std::printf("FAIL fixture logger did not freeze with expected rows\n");
        return false;
    }
    car_runtime_log_header_t header_copy = header;
    const uint32_t expected_header_crc = header_copy.header_crc32;
    header_copy.header_crc32 = 0u;
    if (expected_header_crc != car_runtime_log_crc32(
            &header_copy, CAR_RUNTIME_LOG_HEADER_SIZE)) {
        std::printf("FAIL fixture logger version-2 header CRC mismatch\n");
        return false;
    }
    const size_t length = CAR_RUNTIME_LOG_HEADER_SIZE +
                          header.count * CAR_RUNTIME_LOG_RECORD_SIZE;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
        std::printf("FAIL cannot open runtime logger fixture output: %s\n", path);
        return false;
    }
    output.write(reinterpret_cast<const char *>(&car_runtime_log_storage),
                 static_cast<std::streamsize>(length));
    output.close();
    if (!output) {
        std::printf("FAIL failed writing runtime logger fixture: %s\n", path);
        return false;
    }
    std::printf("RUNTIME_LOG_FIXTURE path=%s bytes=%zu records=%u build_id=",
                path, length, header.count);
    for (uint8_t value : header.build_id) {
        std::printf("%02X", value);
    }
    std::printf("\n");
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc == 3 && std::string(argv[1]) == "--emit-runtime-log-fixture") {
        return emit_runtime_log_fixture(argv[2]) ? 0 : 1;
    }

    if (!parse_csv_vectors("tests/data/vision-protocol-v1.csv")) {
        return 2;
    }
    if (!parse_reverse_vectors("openmv/tests/data/reverse-control-v1.csv")) {
        return 2;
    }

    run_case("shared_protocol_vectors", test_shared_protocol_vectors);
    run_case("reverse_control_vectors_lifecycle",
             test_reverse_control_shared_vectors_and_lifecycle);
    run_case("servo_live_timer_validation", test_servo_live_timer_validation);
    run_case("servo_live_clock_profile_derivation",
             test_servo_live_clock_profile_derivation);
    run_case("follow_output_differential_and_independent_ramps",
             test_follow_output_differential_and_independent_ramps);
    run_case("runtime_diag_snapshot_abi_crc_and_schedule",
             test_runtime_diag_snapshot_abi_crc_and_schedule);
    run_case("turn_slope_assist_gate_and_per_side_slew",
             test_turn_slope_assist_gate_and_per_side_slew);
    run_case("servo_center_frame_slew_sticky_fault",
             test_servo_center_frame_slew_fault_and_sticky_snapshot);
    run_case("servo_stop_phase_and_pending_policy",
             test_servo_stop_phase_and_pending_policy);
    run_case("servo_selected_profile_range_rejection",
             test_servo_production_profile_selection_rejects_before_output);
    run_case("motor_hardware_acceptance_default_lock",
             test_motor_hardware_acceptance_default_lock);
    run_case("protocol_validation_and_crc", test_protocol_validation_and_crc);
    run_case("parser_split_concatenation_resync", test_parser_split_concatenation_and_resync);
    run_case("default_boot_switch_safe", test_default_boot_and_switch_do_not_drive);
    run_case("manual_run_button_toggle_lifecycle",
             test_manual_run_button_toggle_lifecycle);
    run_case("bench_channel_mask_mapping", test_bench_channel_mask_mapping);
    run_case("select_ack_flush_old_rx", test_select_ack_flushes_old_rx_before_new_frame);
    run_case("switch_edge_after_new_frame", test_switch_edge_must_follow_new_frame);
    run_case("bench_ramp_cap_duration_stop", test_bench_ramp_cap_duration_and_stop);
    run_case("bench_auto_select_ready_one_shot",
             test_bench_auto_select_ready_and_one_shot_start);
    run_case("bench_5s_target_hold_wrap_and_absolute_cap",
             test_bench_five_second_target_hold_and_wraparound);
    run_case("illegal_commands_revoke_output", test_invalid_and_illegal_commands_revoke_output);
    run_case("switch_release_isr_stop", test_switch_release_isr_stops_and_latches);
    run_case("fast_stop_fault_ack_order", test_fast_stop_fault_latch_blocks_ack_until_consumed);
    run_case("soft_fault_recovery_no_auto_start", test_soft_fault_recovers_without_auto_start);
    run_case("rejected_commands_revoke_active_output", test_rejected_commands_revoke_active_outputs);
    run_case("follow_boot_release_three_frames_ramp", test_follow_boot_release_three_frames_and_ramp);
    run_case("debug_steering_mapping_auto_tracking", test_debug_steering_mapping_and_automatic_tracking);
    run_case("debug_steering_fault_recovery", test_debug_steering_fault_recovery_and_hard_lock);
    run_case("recovery_confidence_age_gap_wrap", test_recovery_confidence_age_gap_wrap_and_bad_sequence);
    run_case("raw_rx_flush_generation", test_raw_rx_is_diagnostic_and_flush_preserves_new_uart_error);
    run_case("slope_hint_trust_recovery_and_hold",
             test_slope_hint_trust_recovery_and_hold);
    run_case("follow_session_resume_pe7_cancel", test_follow_resume_is_session_scoped_and_pe7_isr_cancels);
    run_case("uart_fifo_service_batches_recovery",
             test_uart_fifo_service_batches_errors_and_follow_recovery);
    run_case("application_follow_recovery_unbounded_run",
             test_application_follow_profile_recovers_and_runs_past_bench_limit);
    run_case("debug_hard_fault_follow_lock", test_debug_and_hard_faults_never_auto_enable_follow);
    run_case("hard_ack_active_source", test_hard_ack_is_rejected_while_source_remains_active);
    run_case("hard_ack_physical_recovery", test_hard_fault_requires_physical_ack_three_frames_and_new_press);
    run_case("vision_quality_sequence_rejection", test_invalid_quality_and_sequence_rejections);
    run_case("exact_vision_timeout", test_exact_vision_timeout_and_late_frame);
    run_case("follow_toggle_actual_controller",
             test_follow_toggle_drives_manual_controller_session);
    run_case("manual_start_reverse_resync",
             test_manual_follow_start_restarts_reverse_resync);
    run_case("follow_hold_requires_applied_outputs",
             test_follow_visual_hold_requires_real_outputs_and_full_recovery);
    run_case("follow_hold_exact_deadline_transport",
             test_follow_visual_hold_exact_deadline_and_transport_split);
    run_case("follow_hold_comm_fault_and_manual_stop",
             test_follow_visual_hold_comm_fault_and_manual_stop_priority);
    run_case("follow_valid0_transport_idle_no_reset",
             test_follow_valid_zero_keeps_transport_alive_without_session);
    run_case("uart_split_timeout", test_uart_fragment_gap_and_exact_partial_timeout);
    run_case("uart_head_snapshot_race", test_uart_poll_consumes_only_captured_head_snapshot);
    run_case("uart_age_overflow_clear_error", test_uart_queue_age_overflow_clear_and_error);
    run_case("uart_crc_noise_resync", test_uart_crc_noise_resync_and_recovery_lock);
    run_case("encoder_wrap_half_range", test_encoder_wrap_and_half_range_rejection);
    run_case("runtime_log_session_fault_recovery_stop",
             test_runtime_log_session_fault_recovery_and_stop);
      run_case("runtime_log_manual_start_resync_wait",
               test_runtime_log_keeps_manual_start_resync_wait);
    run_case("runtime_log_captured_start_same_iteration_hard_fault",
             test_runtime_log_captured_start_then_same_iteration_hard_fault);
    run_case("runtime_log_bench_one_shot_freeze",
             test_runtime_log_bench_one_shot_freezes);
    run_case("runtime_log_stop_waits_for_zero_output",
             test_runtime_log_pending_stop_waits_for_applied_zero);
    run_case("runtime_log_hold_timer_not_recovery_event",
             test_runtime_log_hold_elapsed_does_not_emit_recovery);
    run_case("runtime_log_slope_flags_fixed_abi",
             test_runtime_log_slope_flags_keep_fixed_record_abi);
    run_case("runtime_log_fault_cancel_not_button_stop",
             test_runtime_log_fault_cancel_is_not_physical_button_stop);
    run_case("runtime_log_physical_button_counter_edges",
             test_runtime_log_physical_button_counter_edges);
    run_case("runtime_log_auto_follow_no_session_replacement",
             test_runtime_log_automatic_follow_never_starts_or_replaces_session);
    run_case("runtime_log_restart_header_only",
             test_runtime_log_restart_overwrites_header_without_buffer_clear);
    run_case("runtime_log_capacity_drop_rate_limit",
             test_runtime_log_capacity_rate_limits_dropped_rows);
    run_case("run_block_reason_first_gate",
             test_run_block_reason_reports_first_gate);
    run_case("runtime_log_toggle_control_invariant",
             test_logging_toggle_does_not_change_follow_recovery);
    run_case("speed_observer_signs_jitter_clock_wrap",
             test_speed_observer_signs_jitter_and_clock_wrap);
    run_case("speed_observer_invalid_inputs_stall",
             test_speed_observer_invalid_inputs_and_stall);
    run_case("speed_pi_shadow_math_gates_antiwindup",
             test_speed_pi_shadow_math_gates_and_antiwindup);
    run_case("follow_speed_pi_adapter_sample_slew_hold_recovery",
             test_follow_speed_pi_adapter_sample_slew_hold_and_recovery);

    std::printf("HOST_TEST_SUMMARY cases=%u assertions=%u failures=%u vectors=%u\n",
                g_case_count, g_check_count, g_fail_count,
                static_cast<unsigned>(g_vectors.size()));
    return g_fail_count == 0u ? 0 : 1;
}
