#ifndef CARTEST_UART_RX_SERVICE_H
#define CARTEST_UART_RX_SERVICE_H

#include <stdint.h>

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

#ifdef __cplusplus
}
#endif

#endif /* CARTEST_UART_RX_SERVICE_H */
