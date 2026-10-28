#ifndef CARTEST_MANUAL_RUN_BUTTON_H
#define CARTEST_MANUAL_RUN_BUTTON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CAR_MANUAL_BUTTON_HIGH_STABLE_MS = 20u,
    CAR_MANUAL_BUTTON_LOW_STABLE_MS = 50u,
    CAR_MANUAL_BUTTON_HIGH_IGNORE_MS = 1000u
};

typedef enum {
    CAR_MANUAL_BUTTON_WAIT_HIGH = 0u,
    CAR_MANUAL_BUTTON_WAIT_LOW = 1u,
    CAR_MANUAL_BUTTON_RUNNING = 2u
} car_manual_button_state_t;

typedef enum {
    CAR_MANUAL_BUTTON_EVENT_NONE = 0u,
    CAR_MANUAL_BUTTON_EVENT_START = 1u,
    CAR_MANUAL_BUTTON_EVENT_STOP = 2u,
    CAR_MANUAL_BUTTON_EVENT_BLOCKED_BY_FAULT = 3u
} car_manual_button_event_t;

/* PE7 runs as a three-state maintained switch. Each tick uses the 1 ms
   sample cadence; EXTI only stops on high after the one-shot1000ms start
   window. Fault and emergency stops remain independent. Target
   callers serialize updates with the caller's PRIMASK. */
typedef struct {
    volatile car_manual_button_state_t state;
    volatile uint8_t armed;
    volatile uint8_t run_request;
    volatile uint8_t stop_pending;
    volatile uint8_t start_pending;
    uint8_t high_stop_guard_active;
    uint8_t high_tracking;
    uint8_t low_tracking;
    volatile uint8_t startup_release_confirmed;
    uint32_t run_started_ms;
    uint32_t high_since_ms;
    uint32_t low_since_ms;
    volatile uint32_t press_event_count;
    volatile uint32_t start_event_count;
    volatile uint32_t stop_generation;
} car_manual_button_t;

void car_manual_button_init(car_manual_button_t *button,
                            uint8_t raw_high,
                            uint32_t now_ms);
car_manual_button_event_t car_manual_button_tick_1ms(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint8_t hard_fault_latched,
    uint32_t now_ms);
car_manual_button_event_t car_manual_button_stop_on_high(
    car_manual_button_t *button,
    uint8_t raw_high,
    uint32_t now_ms);
void car_manual_button_cancel_request(car_manual_button_t *button);
void car_manual_button_start_consumed(car_manual_button_t *button);
void car_manual_button_stop_consumed(car_manual_button_t *button);

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_MANUAL_RUN_BUTTON_H */
