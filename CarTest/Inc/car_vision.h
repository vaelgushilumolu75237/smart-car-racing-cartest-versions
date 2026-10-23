#ifndef CARTEST_CAR_VISION_H
#define CARTEST_CAR_VISION_H

#include "commissioning.h"
#include "vision_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAR_VISION_RX_RING_SIZE 256u
#define CAR_VISION_RX_RING_MASK (CAR_VISION_RX_RING_SIZE - 1u)
#define CAR_VISION_PARTIAL_TIMEOUT_MS 20u
#define CAR_VISION_MAX_QUEUE_AGE_MS 100u

typedef struct {
    uint32_t arrival_ms;
    uint8_t byte;
} car_vision_rx_slot_t;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t rx_byte_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t queue_overflow_count;
    volatile uint32_t protocol_error_count;
    volatile uint32_t protocol_version_mismatch_count;
    volatile uint32_t protocol_format_error_count;
    volatile uint32_t protocol_crc_error_count;
    volatile uint32_t protocol_field_error_count;
    volatile uint32_t discarded_byte_count;
    volatile uint32_t partial_timeout_count;
    volatile uint32_t queue_age_reject_count;
    /* Main parser and UART ISR own separate generations to avoid RMW races. */
    uint32_t fault_generation;
    uint32_t fault_generation_reported;
    volatile uint32_t isr_fault_generation;
    uint32_t isr_fault_generation_reported;
    volatile uint8_t uart_fault;
    volatile uint8_t overflow_fault;
    volatile uint8_t protocol_fault;
    volatile uint8_t stale_fault;
    car_vision_rx_slot_t ring[CAR_VISION_RX_RING_SIZE];
    vp_parser parser;
    uint32_t partial_last_byte_ms;
    uint8_t partial_active;
    uint8_t last_received_protocol_version;
    uint8_t last_protocol_status;
} car_vision_rx_t;

void car_vision_rx_init(car_vision_rx_t *rx);
/* Target caller must mask IRQs around each call to protect ISR-only counters
   and ring publication from nested USART/EXTI/SysTick activity. */
void car_vision_rx_isr_byte(car_vision_rx_t *rx,
                            uint8_t byte,
                            uint32_t arrival_ms);
void car_vision_rx_isr_error(car_vision_rx_t *rx);
void car_vision_rx_poll(car_vision_rx_t *rx,
                        car_control_t *control,
                        const car_config_t *config,
                        uint16_t head_snapshot,
                        uint32_t now_ms,
                        uint8_t pe7_raw_high);
/* Caller masks RX IRQ while flushing, then flushes the USART hardware RDR. */
void car_vision_rx_clear(car_vision_rx_t *rx);

#ifdef __cplusplus
}
#endif

#endif
