#include "manual_run_button.h"

#include <stddef.h>

static void manual_button_enter_wait_high(car_manual_button_t *button,
                                          uint8_t raw_high,
                                          uint32_t now_ms)
{
    button->state = CAR_MANUAL_BUTTON_WAIT_HIGH;
    button->armed = 0u;
    button->run_request = 0u;
    button->start_pending = 0u;
    button->high_tracking = raw_high != 0u ? 1u : 0u;
    button->high_since_ms = now_ms;
    button->low_tracking = 0u;
}

void car_manual_button_init(car_manual_button_t *button,
                            uint8_t raw_high,
                            uint32_t now_ms)
{
    if (button == 0) {
        return;
    }
    button->state = CAR_MANUAL_BUTTON_WAIT_HIGH;
    button->armed = 0u;
    button->run_request = 0u;
    button->stop_pending = 0u;
    button->start_pending = 0u;
    button->high_tracking = raw_high != 0u ? 1u : 0u;
    button->low_tracking = 0u;
    button->startup_release_confirmed = 0u;
    button->high_since_ms = now_ms;
    button->low_since_ms = now_ms;
    button->press_event_count = 0u;
    button->start_event_count = 0u;
    button->stop_generation = 0u;
}

car_manual_button_event_t car_manual_button_stop_on_high(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint32_t now_ms)
{
    if (button == 0 || raw_high == 0u) {
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    button->low_tracking = 0u;
    if (button->state == CAR_MANUAL_BUTTON_RUNNING ||
        button->run_request != 0u || button->start_pending != 0u) {
        button->state = CAR_MANUAL_BUTTON_WAIT_HIGH;
        button->armed = 0u;
        button->run_request = 0u;
        button->start_pending = 0u;
        button->stop_pending = 1u;
        button->high_tracking = 1u;
        button->high_since_ms = now_ms;
        ++button->stop_generation;
        return CAR_MANUAL_BUTTON_EVENT_STOP;
    }

    if (button->state == CAR_MANUAL_BUTTON_WAIT_HIGH &&
        button->high_tracking == 0u) {
        button->high_tracking = 1u;
        button->high_since_ms = now_ms;
    }
    return CAR_MANUAL_BUTTON_EVENT_NONE;
}

car_manual_button_event_t car_manual_button_tick_1ms(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint8_t hard_fault_latched,
    uint32_t now_ms)
{
    uint8_t request_active;

    if (button == 0) {
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    if (raw_high != 0u &&
        (button->state == CAR_MANUAL_BUTTON_RUNNING ||
         button->run_request != 0u || button->start_pending != 0u)) {
        return car_manual_button_stop_on_high(button, raw_high, now_ms);
    }

    request_active = button->state == CAR_MANUAL_BUTTON_RUNNING ||
                     button->run_request != 0u ||
                     button->start_pending != 0u;
    if (hard_fault_latched != 0u) {
        const car_manual_button_state_t previous_state = button->state;
        manual_button_enter_wait_high(button, 0u, now_ms);
        if (request_active != 0u ||
            previous_state != CAR_MANUAL_BUTTON_WAIT_HIGH) {
            return CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT;
        }
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    switch (button->state) {
    case CAR_MANUAL_BUTTON_WAIT_HIGH:
        if (raw_high == 0u) {
            button->high_tracking = 0u;
            button->armed = 0u;
            return CAR_MANUAL_BUTTON_EVENT_NONE;
        }
        button->low_tracking = 0u;
        if (button->high_tracking == 0u) {
            button->high_tracking = 1u;
            button->high_since_ms = now_ms;
        } else if (button->stop_pending == 0u &&
                   (uint32_t)(now_ms - button->high_since_ms) >=
                       CAR_MANUAL_BUTTON_HIGH_STABLE_MS) {
            button->state = CAR_MANUAL_BUTTON_WAIT_LOW;
            button->armed = 1u;
            button->startup_release_confirmed = 1u;
        }
        return CAR_MANUAL_BUTTON_EVENT_NONE;

    case CAR_MANUAL_BUTTON_WAIT_LOW:
        if (raw_high != 0u) {
            button->low_tracking = 0u;
            button->armed = 1u;
            return CAR_MANUAL_BUTTON_EVENT_NONE;
        }
        button->high_tracking = 0u;
        if (button->low_tracking == 0u) {
            button->low_tracking = 1u;
            button->low_since_ms = now_ms;
        } else if ((uint32_t)(now_ms - button->low_since_ms) >=
                   CAR_MANUAL_BUTTON_LOW_STABLE_MS) {
            button->state = CAR_MANUAL_BUTTON_RUNNING;
            button->armed = 0u;
            button->run_request = 1u;
            button->start_pending = 1u;
            button->low_tracking = 0u;
            ++button->press_event_count;
            ++button->start_event_count;
            return CAR_MANUAL_BUTTON_EVENT_START;
        }
        return CAR_MANUAL_BUTTON_EVENT_NONE;

    case CAR_MANUAL_BUTTON_RUNNING:
        button->low_tracking = 0u;
        return CAR_MANUAL_BUTTON_EVENT_NONE;

    default:
        manual_button_enter_wait_high(button, raw_high, now_ms);
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }
}

void car_manual_button_cancel_request(car_manual_button_t *button)
{
    if (button != 0) {
        manual_button_enter_wait_high(button, 0u, button->high_since_ms);
    }
}

void car_manual_button_start_consumed(car_manual_button_t *button)
{
    if (button != 0) {
        button->start_pending = 0u;
    }
}

void car_manual_button_stop_consumed(car_manual_button_t *button)
{
    if (button != 0) {
        button->stop_pending = 0u;
    }
}
