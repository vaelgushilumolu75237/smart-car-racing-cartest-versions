#ifndef CARTEST_CAR_VISION_H
#define CARTEST_CAR_VISION_H

#include "commissioning.h"
#include "camera_diag.h"
#include "vision_protocol.h"
#include "car_rx_fastpath.h"
#if defined(_MSC_VER)
#include <intrin.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if CARTEST_WIRELESS_ENABLED
/* Control plus bounded image/diagnostic bursts share this dedicated UART.
   Preserve headroom while the main loop parses the prior snapshot. */
#define CAR_VISION_RX_RING_SIZE 1024u
#else
#define CAR_VISION_RX_RING_SIZE 256u
#endif
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
    uint32_t diagnostic_last_sequence_ms;
    uint16_t diagnostic_last_sequence;
    uint8_t diagnostic_sequence_seen;
    uint8_t diagnostic_discard_until_sync;
    uint8_t diagnostic_discard_remaining;
} car_vision_rx_t;

static CAR_RX_INLINE void car_vision_rx_publish_barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("dmb" ::: "memory");
#elif defined(__GNUC__)
    __asm volatile ("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif
}

/* Caller masks IRQs as for car_vision_rx_isr_byte. Shared with its host-tested
   wrapper, so the fast target path retains the same overflow and publication
   semantics without an out-of-line Debug call for every received byte. */
static CAR_RX_INLINE void car_vision_rx_isr_byte_inline(
    car_vision_rx_t *rx, uint8_t byte, uint32_t arrival_ms)
{
    uint16_t head, next;
    if (rx == 0) return;
    ++rx->rx_byte_count;
    head = rx->head;
    car_vision_rx_publish_barrier();
    next = (uint16_t)((head + 1u) & CAR_VISION_RX_RING_MASK);
    if (next == rx->tail) {
        ++rx->queue_overflow_count;
        rx->overflow_fault = 1u;
        ++rx->isr_fault_generation;
        return;
    }
    rx->ring[head].byte = byte;
    rx->ring[head].arrival_ms = arrival_ms;
    car_vision_rx_publish_barrier();
    rx->head = next;
}

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
