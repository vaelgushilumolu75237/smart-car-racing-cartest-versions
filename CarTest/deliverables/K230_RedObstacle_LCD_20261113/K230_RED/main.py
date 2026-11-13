"""Auxiliary red camera only; keep the existing OpenMV line camera connected."""
from machine import UART, FPIOA
from media.sensor import Sensor
from media.media import MediaManager
from media.display import Display
import image
from settings import (WIDTH, HEIGHT, UART_ID, UART_TX_PIN, UART_BAUDRATE,
                      RED_LAB_THRESHOLD, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                      DISPLAY_FPS)
from red_detector import RedEvent, count_red_pixels


def run():
    fpioa = FPIOA()
    # Yahboom factory firmware names this UART1_TXD; accept TX on other releases.
    tx_function = getattr(FPIOA, "UART%d_TXD" % UART_ID, None)
    if tx_function is None:
        tx_function = getattr(FPIOA, "UART%d_TX" % UART_ID)
    fpioa.set_function(UART_TX_PIN, tx_function, ie=0, oe=1)
    uart = UART(UART_ID, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    sensor = Sensor()
    media_ready = False
    display_ready = False
    try:
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        # Display is independent of capture: detection always sees 320x240.
        display_ready = True
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT,
                     to_ide=True, fps=DISPLAY_FPS)
        preview = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)
        scale = min(DISPLAY_WIDTH / WIDTH, DISPLAY_HEIGHT / HEIGHT)
        x = (DISPLAY_WIDTH - int(WIDTH * scale)) // 2
        y = (DISPLAY_HEIGHT - int(HEIGHT * scale)) // 2
        media_ready = True
        MediaManager.init()
        sensor.run()
        event = RedEvent()
        while True:
            frame = sensor.snapshot()
            event.update(count_red_pixels(frame, RED_LAB_THRESHOLD), uart)
            # Count and transmit before rendering; never draw on detection data.
            preview.clear()
            preview.draw_image(frame, x, y, x_scale=scale, y_scale=scale)
            Display.show_image(preview, x=0, y=0)
    finally:
        try:
            sensor.stop()
        finally:
            try:
                if display_ready:
                    Display.deinit()
            finally:
                try:
                    if media_ready:
                        MediaManager.deinit()
                finally:
                    uart.deinit()


if __name__ == "__main__":
    run()
