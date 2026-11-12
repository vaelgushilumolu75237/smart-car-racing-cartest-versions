#include "uart_rx_service.h"

car_uart_rx_service_result_t car_uart_rx_service_drain(
    const car_uart_rx_service_io_t *io)
{
    return car_uart_rx_service_drain_inline(io);
}
