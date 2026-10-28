#ifndef CARTEST_MANUAL_RUN_BUTTON_H
#define CARTEST_MANUAL_RUN_BUTTON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_MANUAL_BUTTON_REARM_MS = 20u,
    CAR_MANUAL_BUTTON_LOW_QUALIFY_MS = 20u
};

typedef enum {
    CAR_MANUAL_BUTTON_EVENT_NONE = 0u,
    CAR_MANUAL_BUTTON_EVENT_START = 1u,
    CAR_MANUAL_BUTTON_EVENT_STOP = 2u,
    CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT = 3u,
    CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_STOP = 4u
} car_manual_button_event_t;

/* PE7 is a maintained switch: a stable-high startup/release arms a RUN
   request, which requires 20 ms of continuous low. The first high sample
   while running requests STOP. EXTI and main-loop callers serialize updates
   with the caller's PRIMASK. */
typedef struct {
    volatile uint8_t armed;
    volatile uint8_t run_request;
    volatile uint8_t stop_pending;
    volatile uint8_t start_pending;
    uint8_t high_tracking;
    uint8_t low_tracking;
    uint8_t low_qualified;
    volatile uint8_t startup_release_confirmed;
    uint32_t high_since_ms;
    uint32_t low_since_ms;
    volatile uint32_t press_event_count;
    volatile uint32_t start_event_count;
    volatile uint32_t stop_generation;
} car_manual_button_t;

void car_manual_button_init(car_manual_button_t *button,
                            uint8_t raw_high,
                            uint32_t now_ms);
car_manual_button_event_t car_manual_button_poll_high(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint32_t now_ms);
car_manual_button_event_t car_manual_button_note_low_edge(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint8_t hard_fault_latched);
void car_manual_button_cancel_request(car_manual_button_t *button);
void car_manual_button_start_consumed(car_manual_button_t *button);
void car_manual_button_stop_consumed(car_manual_button_t *button);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_MANUAL_RUN_BUTTON_H */
