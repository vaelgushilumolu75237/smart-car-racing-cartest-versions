"""Manual 45cm straight width-map capture; sends only invalid ACK0 frames.

Requires parked B/C profile state from the matched H743 as in original r24.
The operator starts this separately, with the driving session stopped.
"""
import sys

PACKAGE_ROOT = "/sdcard/cartest-k230"
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)


def run():
    import k230_hal
    import main as vision
    k230_hal.run(vision.run_ring_width_calibration)


if __name__ == "__main__":
    run()

