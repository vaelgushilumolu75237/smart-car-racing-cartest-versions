#include "manual_run_button.h"

#include <stddef.h>

void car_manual_button_init(car_manual_button_t *button,
                            uint8_t raw_high,
                            uint32_t now_ms)
{
    if (button == 0) {
        return;
    }
    button->armed = 0u;
    button->run_request = 0u;
    button->stop_pending = 0u;
    button->start_pending = 0u;
    button->high_tracking = raw_high != 0u ? 1u : 0u;
    button->startup_release_confirmed = 0u;
    button->high_since_ms = now_ms;
    button->press_event_count = 0u;
    button->start_event_count = 0u;
    button->stop_generation = 0u;
}

car_manual_button_event_t car_manual_button_poll_high(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint32_t now_ms)
{
    if (button == 0) {
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }
    if (raw_high == 0u) {
        button->high_tracking = 0u;
    } else if (button->high_tracking == 0u) {
        button->high_tracking = 1u;
        button->high_since_ms = now_ms;
    } else if ((uint32_t)(now_ms - button->high_since_ms) >=
               CAR_MANUAL_BUTTON_REARM_MS) {
        button->startup_release_confirmed = 1u;
        if (button->stop_pending == 0u) {
            button->armed = 1u;
        }
    }

    if (raw_high != 0u && button->stop_pending == 0u &&
        (button->run_request != 0u || button->start_pending != 0u)) {
        button->run_request = 0u;
        button->start_pending = 0u;
        button->armed = 0u;
        button->stop_pending = 1u;
        ++button->stop_generation;
        return CAR_MANUAL_BUTTON_EVENT_STOP;
    }

    /* Keep tracking the high interval while a queued stop owns the switch. */
    return CAR_MANUAL_BUTTON_EVENT_NONE;
}

car_manual_button_event_t car_manual_button_note_low_edge(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint8_t hard_fault_latched)
{
    if (button == 0 || raw_high != 0u) {
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    if (button->stop_pending != 0u) {
        /* A queued high-level stop outranks any later low sample. */
        return CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_STOP;
    }
    if (button->armed == 0u) {
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    /* Consume one RUN request at the first armed low sample. A long hold and
       bounce cannot repeat it; stable high is required before another start. */
    button->armed = 0u;
    button->high_tracking = 0u;
    ++button->press_event_count;
    if (hard_fault_latched != 0u) {
        return CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT;
    }
    if (button->run_request != 0u || button->start_pending != 0u) {
        /* Low is only a RUN request. A high sample owns the STOP transition. */
        return CAR_MANUAL_BUTTON_EVENT_NONE;
    }

    button->run_request = 1u;
    button->start_pending = 1u;
    ++button->start_event_count;
    return CAR_MANUAL_BUTTON_EVENT_START;
}

void car_manual_button_cancel_request(car_manual_button_t *button)
{
    if (button != 0) {
        button->run_request = 0u;
        button->start_pending = 0u;
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
