"""Yahboom K230 vision module deluxe, not the generic K230 development board.

Factory communication socket: UART1, TX GPIO9, RX GPIO10.
Source: https://www.yahboom.net/study/K230/14977
All camera mounting changes require a new private calibration.
"""
PORT_BUILD = "CARTEST_K230_GYRO350_20261106"
UART_ID = 1
UART_TX_PIN = 9
UART_RX_PIN = 10
SENSOR_CSI_ID = None  # Use Yahboom firmware's default CSI (normally CSI2).
SENSOR_INPUT_WIDTH = 1920
SENSOR_INPUT_HEIGHT = 1080
SENSOR_FPS = 60  # Requested rate; not a measured processing/display rate.
MIRROR_X = False
FLIP_Y = False
DISPLAY_MODE = "lcd"  # lcd (ST7701 + IDE), ide (VIRT), or off.
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_FPS = 60
DISPLAY_QUALITY = 50
DATA_DIRECTORY = "/sdcard/cartest-k230"
MOUNT_REVISION = "mount-1"  # Change after camera/lens/height/pitch adjustment.

