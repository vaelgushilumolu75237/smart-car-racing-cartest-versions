"""Auxiliary red camera only; keep the existing OpenMV line camera connected."""
from machine import UART, FPIOA
from media.sensor import Sensor
from media.media import MediaManager
from settings import (WIDTH, HEIGHT, UART_ID, UART_TX_PIN, UART_BAUDRATE,
                      RED_LAB_THRESHOLD)
from red_detector import RedEvent, count_red_pixels


def run():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TX, ie=0, oe=1)
    uart = UART(UART_ID, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    sensor = Sensor()
    media_ready = False
    try:
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        MediaManager.init()
        media_ready = True
        sensor.run()
        event = RedEvent()
        while True:
            frame = sensor.snapshot()
            event.update(count_red_pixels(frame, RED_LAB_THRESHOLD), uart)
    finally:
        try:
            sensor.stop()
        finally:
            if media_ready:
                MediaManager.deinit()
            uart.deinit()


if __name__ == "__main__":
    run()
