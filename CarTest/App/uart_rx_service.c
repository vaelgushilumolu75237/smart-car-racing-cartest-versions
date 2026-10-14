#include "uart_rx_service.h"

car_uart_rx_service_result_t car_uart_rx_service_drain(
    const car_uart_rx_service_io_t *io)
{
    car_uart_rx_service_result_t result = {0u, 0u, 0u, 0u};

    if (io == 0 || io->read_status == 0 || io->read_data == 0) {
        return result;
    }

    while (result.bytes_drained < CAR_UART_RX_SERVICE_MAX_BATCH) {
        const uint32_t status = io->read_status(io->context);
        if ((status & io->error_mask) != 0u) {
            result.stopped_for_error = 1u;
            if (io->on_error != 0) {
                io->on_error(io->context, status);
            }
            return result;
        }
        if ((status & io->rxne_mask) == 0u) {
            return result;
        }

        {
            const uint8_t byte = io->read_data(io->context);
            ++result.bytes_drained;
            if (io->on_byte != 0 && io->on_byte(io->context, byte) != 0u) {
                result.stopped_for_abort = 1u;
                if (io->on_abort != 0) {
                    io->on_abort(io->context);
                }
                return result;
            }
        }
    }

    result.budget_exhausted = 1u;
    return result;
}
