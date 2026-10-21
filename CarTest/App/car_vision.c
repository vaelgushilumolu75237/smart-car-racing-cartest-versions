#include "car_vision.h"

#include <string.h>

#include "receive_diag.h"
#if defined(_MSC_VER)
#include <intrin.h>
#endif

static void publish_barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile ("dmb" ::: "memory");
#elif defined(__GNUC__)
    __asm volatile ("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#endif
}

static void note_protocol_error(car_vision_rx_t *rx,
                                car_control_t *control,
                                uint8_t discarded_byte,
                                uint32_t now_ms)
{
    ++rx->protocol_error_count;
    car_receive_diag_note_parser_error(now_ms);
    if (discarded_byte != 0u) {
        ++rx->discarded_byte_count;
    }
    rx->protocol_fault = 1u;
    ++rx->fault_generation;
    (void)control;
}

static void dispatch_new_rx_faults(car_vision_rx_t *rx,
                                   car_control_t *control)
{
    uint32_t parser_generation;
    uint32_t isr_generation;
    uint8_t uart_fault;
    uint8_t overflow_fault;
    uint8_t protocol_fault;
    uint8_t stale_fault;

    parser_generation = rx->fault_generation;
    isr_generation = rx->isr_fault_generation;
    if (parser_generation == rx->fault_generation_reported &&
        isr_generation == rx->isr_fault_generation_reported) {
        return;
    }

    /* Each counter has one writer, so an interrupt cannot lose an increment. */
    rx->fault_generation_reported = parser_generation;
    rx->isr_fault_generation_reported = isr_generation;
    uart_fault = rx->uart_fault;
    overflow_fault = rx->overflow_fault;
    protocol_fault = rx->protocol_fault;
    stale_fault = rx->stale_fault;

    if (uart_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_UART);
    }
    if (overflow_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_RX_OVERFLOW);
    }
    if (protocol_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_PROTOCOL);
    }
    if (stale_fault != 0u) {
        car_control_note_fault(control, CAR_FAULT_RX_STALE);
    }
}

void car_vision_rx_init(car_vision_rx_t *rx)
{
    if (rx == 0) {
        return;
    }
    (void)memset(rx, 0, sizeof(*rx));
    vp_parser_init(&rx->parser);
}

void car_vision_rx_isr_byte(car_vision_rx_t *rx,
                            uint8_t byte,
                            uint32_t arrival_ms)
{
    uint16_t head;
    uint16_t next;
    if (rx == 0) {
        return;
    }
    ++rx->rx_byte_count;
    head = rx->head;
    publish_barrier();
    next = (uint16_t)((head + 1u) & CAR_VISION_RX_RING_MASK);
    if (next == rx->tail) {
        ++rx->queue_overflow_count;
        rx->overflow_fault = 1u;
        ++rx->isr_fault_generation;
        return;
    }
    rx->ring[head].byte = byte;
    rx->ring[head].arrival_ms = arrival_ms;
    publish_barrier();
    rx->head = next;
}

void car_vision_rx_isr_error(car_vision_rx_t *rx)
{
    if (rx != 0) {
        ++rx->uart_error_count;
        rx->uart_fault = 1u;
        ++rx->isr_fault_generation;
    }
}

void car_vision_rx_poll(car_vision_rx_t *rx,
                        car_control_t *control,
                        const car_config_t *config,
                        uint16_t head_snapshot,
                        uint32_t now_ms,
                        uint8_t pe7_raw_high)
{
    uint16_t tail;
    uint8_t byte;
    uint8_t parser_length_before;
    uint32_t arrival_ms;
    vp_frame frame;

    if (rx == 0 || control == 0 || config == 0) {
        return;
    }

    dispatch_new_rx_faults(rx, control);
    if (control->rx_recovery_flush_required != 0u) {
        return;
    }

    while (rx->tail != head_snapshot) {
        dispatch_new_rx_faults(rx, control);
        if (control->rx_recovery_flush_required != 0u) {
            return;
        }
        tail = rx->tail;
        publish_barrier();
        arrival_ms = rx->ring[tail].arrival_ms;
        if ((uint32_t)(now_ms - arrival_ms) >= CAR_VISION_MAX_QUEUE_AGE_MS) {
            ++rx->queue_age_reject_count;
            rx->stale_fault = 1u;
            ++rx->fault_generation;
            rx->tail = rx->head;
            rx->partial_active = 0u;
            vp_parser_init(&rx->parser);
            dispatch_new_rx_faults(rx, control);
            return;
        }
        byte = rx->ring[tail].byte;
        publish_barrier();
        rx->tail = (uint16_t)((tail + 1u) & CAR_VISION_RX_RING_MASK);

        if (rx->partial_active != 0u &&
            (uint32_t)(arrival_ms - rx->partial_last_byte_ms) >=
                CAR_VISION_PARTIAL_TIMEOUT_MS) {
            ++rx->partial_timeout_count;
            rx->stale_fault = 1u;
            ++rx->fault_generation;
            rx->partial_active = 0u;
            vp_parser_init(&rx->parser);
            dispatch_new_rx_faults(rx, control);
            return;
        }
        parser_length_before = rx->parser.length;

        if ((parser_length_before == 0u && byte != VP_SYNC_BYTE_0) ||
            (parser_length_before == 1u && byte != VP_SYNC_BYTE_1)) {
            note_protocol_error(rx, control, 1u, now_ms);
            dispatch_new_rx_faults(rx, control);
            return;
        }

        if (vp_parser_feed(&rx->parser, byte, &frame) != 0u) {
            rx->partial_active = 0u;
            dispatch_new_rx_faults(rx, control);
            if (control->rx_recovery_flush_required != 0u) {
                return;
            }
            car_receive_diag_note_frame(
                frame.sequence, (frame.flags & VP_FLAG_VALID) != 0u,
                arrival_ms);
            car_control_on_vision_frame_at_with_flags(
                control, config, frame.sequence, frame.flags,
                frame.confidence, frame.lateral_error, arrival_ms,
                now_ms, pe7_raw_high);
            dispatch_new_rx_faults(rx, control);
        } else if (parser_length_before == (VP_FRAME_SIZE - 1u)) {
            note_protocol_error(rx, control, 0u, now_ms);
            dispatch_new_rx_faults(rx, control);
            return;
        }

        if (rx->parser.length != 0u) {
            rx->partial_active = 1u;
            rx->partial_last_byte_ms = arrival_ms;
        } else {
            rx->partial_active = 0u;
        }
        if (control->rx_recovery_flush_required != 0u) {
            return;
        }
    }

    if (rx->partial_active != 0u &&
        (uint32_t)(now_ms - rx->partial_last_byte_ms) >=
            CAR_VISION_PARTIAL_TIMEOUT_MS) {
        ++rx->partial_timeout_count;
        rx->stale_fault = 1u;
        ++rx->fault_generation;
        rx->partial_active = 0u;
        vp_parser_init(&rx->parser);
        dispatch_new_rx_faults(rx, control);
    }
}

void car_vision_rx_clear(car_vision_rx_t *rx)
{
    if (rx == 0) {
        return;
    }
    rx->tail = rx->head;
    rx->uart_fault = 0u;
    rx->overflow_fault = 0u;
    rx->protocol_fault = 0u;
    rx->stale_fault = 0u;
    rx->fault_generation_reported = rx->fault_generation;
    rx->isr_fault_generation_reported = rx->isr_fault_generation;
    rx->partial_active = 0u;
    rx->partial_last_byte_ms = 0u;
    vp_parser_init(&rx->parser);
}
