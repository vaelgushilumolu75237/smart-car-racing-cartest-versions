# Yahboom K230 auxiliary camera. LAB red classification is a tuning candidate.
WIDTH = 320
HEIGHT = 240
UART_ID = 1
UART_TX_PIN = 9
UART_RX_PIN = 10  # Required by UART initialization, even with no RX wire.
UART_BAUDRATE = 115200
RED_LAB_THRESHOLD = (0, 100, 25, 127, -20, 127)
# Yahboom on-board ST7701 LCD. Only the preview is enlarged to 640x480.
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_FPS = 30
