#ifndef CARTEST_UART_RX_SERVICE_H
#define CARTEST_UART_RX_SERVICE_H

#include <stdint.h>
#include "car_rx_fastpath.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    /* STM32H743 USART RX FIFO depth is 16 entries. Keep each IRQ bounded to
       one FIFO's worth so a level-pending RXFNE interrupt can re-enter. */
    CAR_UART_RX_SERVICE_MAX_BATCH = 16u
};

typedef struct {
    void *context;
    uint32_t rxne_mask;
    uint32_t error_mask;
    uint32_t (*read_status)(void *context);
    uint8_t (*read_data)(void *context);
    /* Must latch/count the captured error, stop outputs, disable RX service,
       and discard the peripheral FIFO before returning. */
    void (*on_error)(void *context, uint32_t status);
    /* Return nonzero when the software RX queue overflowed. */
    uint8_t (*on_byte)(void *context, uint8_t byte);
    /* Must stop outputs, disable RX service, and discard remaining FIFO. */
    void (*on_abort)(void *context);
} car_uart_rx_service_io_t;

typedef struct {
    uint8_t bytes_drained;
    uint8_t budget_exhausted;
    uint8_t stopped_for_error;
    uint8_t stopped_for_abort;
} car_uart_rx_service_result_t;

/* Shared production/host algorithm. Errors are checked before every data
   read. A full batch returns immediately; RXFNE remains level asserted if
   more data is pending and the interrupt can re-enter. */
car_uart_rx_service_result_t car_uart_rx_service_drain(
    const car_uart_rx_service_io_t *io);

/* Same bounded/error-first algorithm used by the host-visible wrapper. With
   constant target callbacks the compiler removes the indirect calls. */
static CAR_RX_INLINE car_uart_rx_service_result_t
car_uart_rx_service_drain_inline(const car_uart_rx_service_io_t *io)
{
    car_uart_rx_service_result_t result = {0u, 0u, 0u, 0u};
    if (io == 0 || io->read_status == 0 || io->read_data == 0) {
        return result;
    }
    while (result.bytes_drained < CAR_UART_RX_SERVICE_MAX_BATCH) {
        const uint32_t status = io->read_status(io->context);
        if ((status & io->error_mask) != 0u) {
            result.stopped_for_error = 1u;
            if (io->on_error != 0) io->on_error(io->context, status);
            return result;
        }
        if ((status & io->rxne_mask) == 0u) return result;
        {
            const uint8_t byte = io->read_data(io->context);
            ++result.bytes_drained;
            if (io->on_byte != 0 && io->on_byte(io->context, byte) != 0u) {
                result.stopped_for_abort = 1u;
                if (io->on_abort != 0) io->on_abort(io->context);
                return result;
            }
        }
    }
    result.budget_exhausted = 1u;
    return result;
}

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_UART_RX_SERVICE_H */
