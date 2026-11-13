#include "servo_safety.h"

#define SERVO_RCC_TIMPRE_BIT       (1u << 15u)
#define SERVO_RCC_D2PPRE2_MASK     (7u << 8u)
#define SERVO_RCC_TIM1EN_BIT      (1u << 0u)

#define SERVO_TIM_CR1_CEN_BIT     (1u << 0u)
#define SERVO_TIM_CR1_ARPE_BIT    (1u << 7u)
#define SERVO_TIM_CCMR1_CC1S_MASK (3u << 0u)
#define SERVO_TIM_CCMR1_OC1FE     (1u << 2u)
#define SERVO_TIM_CCMR1_OC1PE     (1u << 3u)
#define SERVO_TIM_CCMR1_OC1M_MASK (7u << 4u)
#define SERVO_TIM_CCMR1_OC1M_PWM1 (6u << 4u)
#define SERVO_TIM_CCMR1_OC1M3     (1u << 16u)
#define SERVO_TIM_CCER_CC1E       (1u << 0u)
#define SERVO_TIM_CCER_CC1P       (1u << 1u)
#define SERVO_TIM_CCER_CC1NP      (1u << 3u)
#define SERVO_TIM_BDTR_MOE        (1u << 15u)
#define SERVO_GPIOA_PA8_MODE_MASK (3u << 16u)
#define SERVO_GPIOA_PA8_MODE_OUTPUT (1u << 16u)
#define SERVO_GPIOA_PA8_MODE_AF     (2u << 16u)
#define SERVO_GPIOA_PA8_ODR_BIT     (1u << 8u)

#define SERVO_RCC_CR_HSI_ON_READY_MASK 0x00000005u
#define SERVO_RCC_CR_HSIDIV_MASK       0x00000018u
#define SERVO_RCC_CFGR_SW_SWS_MASK     0x0000003Fu
#define SERVO_RCC_CFGR_TIMPRE_BIT      (1u << 15u)
#define SERVO_RCC_D1_PRESCALER_MASK    0x00000F0Fu

static uint32_t apb2_divisor(uint32_t rcc_d2cfgr)
{
    switch ((rcc_d2cfgr & SERVO_RCC_D2PPRE2_MASK) >> 8u) {
    case 0u:
        return 1u;
    case 4u:
        return 2u;
    case 5u:
        return 4u;
    case 6u:
        return 8u;
    case 7u:
        return 16u;
    default:
        return 0u;
    }
}

uint32_t car_servo_safety_timer_clock_hz(
    const car_servo_timer_snapshot_t *snapshot)
{
    uint32_t divisor;
    uint32_t multiplier;
    uint32_t clock_hz;

    if (snapshot == 0 || snapshot->pclk2_hz == 0u ||
        snapshot->hclk_hz == 0u) {
        return 0u;
    }

    divisor = apb2_divisor(snapshot->rcc_d2cfgr);
    if (divisor == 0u) {
        return 0u;
    }

    if ((snapshot->rcc_cfgr & SERVO_RCC_TIMPRE_BIT) == 0u) {
        multiplier = divisor == 1u ? 1u : 2u;
        clock_hz = snapshot->pclk2_hz;
    } else if (divisor <= 4u) {
        multiplier = 1u;
        clock_hz = snapshot->hclk_hz;
    } else {
        multiplier = 4u;
        clock_hz = snapshot->pclk2_hz;
    }

    if (clock_hz > UINT32_MAX / multiplier) {
        return 0u;
    }
    return clock_hz * multiplier;
}

uint8_t car_servo_safety_apply_fixed_hsi64_profile(
    car_servo_timer_snapshot_t *snapshot)
{
    uint8_t matches;

    if (snapshot == 0) {
        return 0u;
    }

    matches = (uint8_t)(
        (snapshot->rcc_cr & SERVO_RCC_CR_HSI_ON_READY_MASK) ==
            SERVO_RCC_CR_HSI_ON_READY_MASK &&
        (snapshot->rcc_cr & SERVO_RCC_CR_HSIDIV_MASK) == 0u &&
        (snapshot->rcc_cfgr & SERVO_RCC_CFGR_SW_SWS_MASK) == 0u &&
        (snapshot->rcc_cfgr & SERVO_RCC_CFGR_TIMPRE_BIT) == 0u &&
        (snapshot->rcc_d1cfgr & SERVO_RCC_D1_PRESCALER_MASK) == 0u &&
        (snapshot->rcc_d2cfgr & SERVO_RCC_D2PPRE2_MASK) == 0u);

    snapshot->sysclk_hz = matches != 0u ? 64000000u : 0u;
    snapshot->hclk_hz = snapshot->sysclk_hz;
    snapshot->pclk2_hz = snapshot->sysclk_hz;
    return matches;
}

car_servo_fault_t car_servo_safety_validate_timer(
    const car_servo_timer_snapshot_t *snapshot,
    uint8_t expected_active)
{
    uint32_t ccmr1_mask;
    uint32_t ccmr1_value;
    uint8_t timer_active;
    uint8_t channel_active;
    uint8_t main_output_active;
    uint8_t gpio_mode;
    uint8_t pin_active;

    if (snapshot == 0 ||
        (snapshot->rcc_cr & 0x00000005u) != 0x00000005u ||
        (snapshot->rcc_cr & 0x00000018u) != 0u ||
        (snapshot->rcc_cfgr & 0x0000003Fu) != 0u ||
        (snapshot->rcc_cfgr & SERVO_RCC_TIMPRE_BIT) != 0u ||
        (snapshot->rcc_d1cfgr & 0x00000F0Fu) != 0u ||
        (snapshot->rcc_d2cfgr & SERVO_RCC_D2PPRE2_MASK) != 0u ||
        snapshot->sysclk_hz != 64000000u ||
        snapshot->hclk_hz != 64000000u ||
        snapshot->pclk2_hz != 64000000u ||
        (snapshot->rcc_apb2enr & SERVO_RCC_TIM1EN_BIT) == 0u ||
        car_servo_safety_timer_clock_hz(snapshot) != 64000000u) {
        return CAR_SERVO_FAULT_TIMER_CLOCK;
    }

    if (snapshot->tim_psc != 63u || snapshot->tim_arr != 19999u ||
        snapshot->tim_rcr != 0u) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }

    timer_active = (snapshot->tim_cr1 & SERVO_TIM_CR1_CEN_BIT) != 0u
                       ? 1u : 0u;
    channel_active = (snapshot->tim_ccer & SERVO_TIM_CCER_CC1E) != 0u
                         ? 1u : 0u;
    main_output_active = (snapshot->tim_bdtr & SERVO_TIM_BDTR_MOE) != 0u
                             ? 1u : 0u;
    pin_active = (uint8_t)(expected_active != 0u);
    if (timer_active != pin_active || channel_active != pin_active ||
        main_output_active != pin_active ||
        (snapshot->tim_smcr != 0u) || (snapshot->tim_dier != 0u)) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }

    /* CEN is dynamic; all other CR1 bits must match the configured upcounter
       with ARR preload disabled. CR2 stays at reset to avoid trigger/output
       remapping. */
    if ((snapshot->tim_cr1 & ~(SERVO_TIM_CR1_CEN_BIT)) != 0u ||
        (snapshot->tim_cr1 & SERVO_TIM_CR1_ARPE_BIT) != 0u ||
        snapshot->tim_cr2 != 0u) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }

    ccmr1_mask = SERVO_TIM_CCMR1_CC1S_MASK | SERVO_TIM_CCMR1_OC1FE |
                 SERVO_TIM_CCMR1_OC1PE | SERVO_TIM_CCMR1_OC1M_MASK |
                 SERVO_TIM_CCMR1_OC1M3;
    ccmr1_value = SERVO_TIM_CCMR1_OC1PE | SERVO_TIM_CCMR1_OC1M_PWM1;
    if ((snapshot->tim_ccmr1 & ccmr1_mask) != ccmr1_value ||
        (snapshot->tim_ccer & ~(SERVO_TIM_CCER_CC1E)) != 0u ||
        (snapshot->tim_ccer &
         (SERVO_TIM_CCER_CC1P | SERVO_TIM_CCER_CC1NP)) != 0u ||
        (snapshot->tim_bdtr & ~SERVO_TIM_BDTR_MOE) != 0u) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }

    gpio_mode = (uint8_t)((snapshot->gpioa_moder &
                           SERVO_GPIOA_PA8_MODE_MASK) >> 16u);
    if (expected_active != 0u) {
        if (gpio_mode != 2u || (snapshot->gpioa_afr1 & 0xFu) != 1u) {
            return CAR_SERVO_FAULT_TIMER_CONFIG;
        }
    } else if (gpio_mode != 1u ||
               (snapshot->gpioa_odr & SERVO_GPIOA_PA8_ODR_BIT) != 0u) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }

    if (snapshot->tim_ccr1 != 0u &&
        (snapshot->tim_ccr1 < CAR_SERVO_SAFETY_MIN_US ||
         snapshot->tim_ccr1 > CAR_SERVO_SAFETY_MAX_US)) {
        return CAR_SERVO_FAULT_TIMER_CONFIG;
    }
    return CAR_SERVO_FAULT_NONE;
}

void car_servo_safety_init(car_servo_safety_t *state)
{
    if (state == 0) {
        return;
    }
    *state = (car_servo_safety_t){0};
}

static void capture_snapshot(const car_servo_safety_io_t *io,
                             car_servo_timer_snapshot_t *snapshot)
{
    *snapshot = (car_servo_timer_snapshot_t){0};
    io->capture(io->context, snapshot);
}

static void latch_fault(car_servo_safety_t *state,
                        car_servo_fault_t fault,
                        uint16_t rejected_request_us,
                        const car_servo_timer_snapshot_t *snapshot,
                        const car_servo_safety_io_t *io)
{
    if (state->fault_latched == 0u) {
        state->last_rejected_request_us = rejected_request_us;
        state->sticky_fault_code = (uint8_t)fault;
        if (snapshot != 0) {
            state->last_fault_snapshot = *snapshot;
        }
    }
    state->fault_latched = 1u;
    state->frame_ready = 0u;
    state->active = 0u;
    state->limited_us = 0u;
    state->compare_us = 0u;
    state->discard_first_update = 0u;
    state->have_slew_time = 0u;
    if (io != 0 && io->stop_output != 0) {
        io->stop_output(io->context);
    }
}

static uint8_t write_checked(car_servo_safety_t *state,
                             uint16_t pulse_us,
                             uint16_t rejected_request_us,
                             const car_servo_timer_snapshot_t *snapshot,
                             const car_servo_safety_io_t *io)
{
    if (io->write_compare == 0 || io->read_compare == 0 ||
        io->write_compare(io->context, pulse_us) == 0u ||
        io->read_compare(io->context) != pulse_us) {
        car_servo_timer_snapshot_t failed_snapshot = *snapshot;
        if (io->read_compare != 0) {
            failed_snapshot.tim_ccr1 = io->read_compare(io->context);
        }
        latch_fault(state, CAR_SERVO_FAULT_COMPARE_WRITE,
                    rejected_request_us, &failed_snapshot, io);
        return 0u;
    }
    state->compare_us = pulse_us;
    return 1u;
}

static uint8_t request_internal(car_servo_safety_t *state,
                                uint16_t requested_us,
                                uint8_t output_permitted,
                                uint32_t now_ms,
                                uint16_t slew_step_us,
                                const car_servo_safety_io_t *io)
{
    car_servo_timer_snapshot_t snapshot;
    car_servo_fault_t timer_fault;
    uint16_t next_us;
    uint16_t delta;

    if (state == 0 || io == 0 || io->capture == 0 ||
        io->stop_output == 0) {
        return 0u;
    }
    state->requested_us = requested_us;
    if (state->fault_latched != 0u) {
        return 0u;
    }

    capture_snapshot(io, &snapshot);
    if (slew_step_us == 0u ||
        slew_step_us > CAR_SERVO_SAFETY_FOLLOW_NORMAL_SLEW_STEP_US) {
        latch_fault(state, CAR_SERVO_FAULT_TIMER_CONFIG, requested_us,
                    &snapshot, io);
        return 0u;
    }
    if (requested_us < CAR_SERVO_SAFETY_MIN_US ||
        requested_us > CAR_SERVO_SAFETY_MAX_US) {
        latch_fault(state, CAR_SERVO_FAULT_REQUEST_RANGE, requested_us,
                    &snapshot, io);
        return 0u;
    }

    timer_fault = car_servo_safety_validate_timer(&snapshot, state->active);
    if (timer_fault != CAR_SERVO_FAULT_NONE) {
        latch_fault(state, timer_fault, requested_us, &snapshot, io);
        return 0u;
    }

    if (output_permitted == 0u) {
        car_servo_safety_stop(state, io);
        state->requested_us = requested_us;
        return 0u;
    }

    if (state->active == 0u) {
        if (write_checked(state, CAR_SERVO_SAFETY_CENTER_US,
                          requested_us, &snapshot, io) == 0u) {
            return 0u;
        }
        if (io->start_output == 0 ||
            io->start_output(io->context) == 0u) {
            car_servo_timer_snapshot_t failed_snapshot;
            capture_snapshot(io, &failed_snapshot);
            latch_fault(state, CAR_SERVO_FAULT_START_OUTPUT,
                        requested_us, &failed_snapshot, io);
            return 0u;
        }
        state->active = 1u;
        state->limited_us = CAR_SERVO_SAFETY_CENTER_US;
        state->frame_ready = 0u;
        state->discard_first_update = 1u;
        capture_snapshot(io, &snapshot);
        timer_fault = car_servo_safety_validate_timer(&snapshot, 1u);
        if (timer_fault != CAR_SERVO_FAULT_NONE) {
            latch_fault(state, timer_fault, requested_us, &snapshot, io);
            return 0u;
        }
        state->last_slew_ms = now_ms;
        state->have_slew_time = 1u;
        return 1u;
    }

    if (io->take_completed_update != 0 &&
        io->take_completed_update(io->context) != 0u) {
        if (state->discard_first_update != 0u) {
            state->discard_first_update = 0u;
        } else {
            state->frame_ready = 1u;
        }
    }

    if (state->have_slew_time == 0u) {
        state->last_slew_ms = now_ms;
        state->have_slew_time = 1u;
        return 1u;
    }
    if ((uint32_t)(now_ms - state->last_slew_ms) <
        CAR_SERVO_SAFETY_SLEW_PERIOD_MS) {
        return 1u;
    }

    next_us = state->limited_us;
    if (requested_us > next_us) {
        delta = (uint16_t)(requested_us - next_us);
        next_us = (uint16_t)(next_us +
            (delta > slew_step_us ? slew_step_us : delta));
    } else if (requested_us < next_us) {
        delta = (uint16_t)(next_us - requested_us);
        next_us = (uint16_t)(next_us -
            (delta > slew_step_us ? slew_step_us : delta));
    }

    /* A delayed loop takes one step, then starts a fresh 20 ms interval. */
    state->last_slew_ms = now_ms;
    if (next_us != state->limited_us) {
        if (write_checked(state, next_us, requested_us, &snapshot, io) == 0u) {
            return 0u;
        }
        state->limited_us = next_us;
    }
    return 1u;
}

uint8_t car_servo_safety_request(car_servo_safety_t *state,
                                 uint16_t requested_us,
                                 uint32_t now_ms,
                                 const car_servo_safety_io_t *io)
{
    return request_internal(state, requested_us, 1u, now_ms,
                            CAR_SERVO_SAFETY_SLEW_STEP_US, io);
}

uint8_t car_servo_safety_request_selected(
    car_servo_safety_t *state,
    car_servo_source_t source,
    uint16_t follow_target_us,
    uint16_t debug_target_us,
    uint8_t output_permitted,
    uint32_t now_ms,
    const car_servo_safety_io_t *io)
{
    return car_servo_safety_request_selected_with_slew(
        state, source, follow_target_us, debug_target_us, output_permitted,
        now_ms, CAR_SERVO_SAFETY_SLEW_STEP_US, io);
}

uint8_t car_servo_safety_request_selected_with_slew(
    car_servo_safety_t *state,
    car_servo_source_t source,
    uint16_t follow_target_us,
    uint16_t debug_target_us,
    uint8_t output_permitted,
    uint32_t now_ms,
    uint16_t slew_step_us,
    const car_servo_safety_io_t *io)
{
    uint16_t requested_us;

    if (source == CAR_SERVO_SOURCE_FOLLOW) {
        requested_us = follow_target_us;
    } else if (source == CAR_SERVO_SOURCE_DEBUG_STEERING) {
        requested_us = debug_target_us;
    } else {
        car_servo_timer_snapshot_t snapshot;
        if (state == 0 || io == 0 || io->capture == 0 ||
            io->stop_output == 0) {
            return 0u;
        }
        if (state->fault_latched == 0u) {
            capture_snapshot(io, &snapshot);
            latch_fault(state, CAR_SERVO_FAULT_TIMER_CONFIG, 0u,
                        &snapshot, io);
        }
        return 0u;
    }

    return request_internal(state, requested_us, output_permitted,
                            now_ms, slew_step_us, io);
}

void car_servo_safety_stop(car_servo_safety_t *state,
                           const car_servo_safety_io_t *io)
{
    if (state == 0) {
        return;
    }
    if (io != 0 && io->stop_output != 0) {
        io->stop_output(io->context);
    }
    state->active = 0u;
    state->frame_ready = 0u;
    state->discard_first_update = 0u;
    state->limited_us = 0u;
    state->compare_us = 0u;
    state->have_slew_time = 0u;
}

void car_servo_safety_acknowledge(car_servo_safety_t *state)
{
    if (state != 0) {
        state->fault_latched = 0u;
    }
}

car_servo_stop_action_t car_servo_safety_stop_action(
    uint8_t stop_pending,
    uint8_t compare_event,
    uint8_t pin_high,
    uint32_t counter,
    uint32_t arr)
{
    const uint32_t guard = CAR_SERVO_SAFETY_STOP_EDGE_GUARD_US;
    const uint8_t edge_window =
        arr <= guard * 2u || counter < guard || counter > arr - guard;

    if (stop_pending == 0u) {
        return (pin_high != 0u || edge_window != 0u)
                   ? CAR_SERVO_STOP_ACTION_WAIT
                   : CAR_SERVO_STOP_ACTION_CLOSE_NOW;
    }
    if (compare_event == 0u) {
        return CAR_SERVO_STOP_ACTION_WAIT;
    }
    return (pin_high != 0u || edge_window != 0u)
               ? CAR_SERVO_STOP_ACTION_CLEAR_COMPARE_AND_WAIT
               : CAR_SERVO_STOP_ACTION_CLOSE_NOW;
}
