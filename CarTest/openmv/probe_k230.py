"""User-run board/camera/preview probe, with no UART initialization or writes.

Import does nothing. Does not measure the detector or establish 60FPS.
"""
import sys

PACKAGE_ROOT = "/sdcard/cartest-k230"
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)


def run(seconds=10):
    import os
    import time
    import gc
    import k230_hal
    try:
        print("K230_SYSTEM", os.uname())
        print("K230_PYTHON", sys.implementation)
        camera = k230_hal.sensor
        camera.configure()
        started = time.ticks_ms()
        count = 0
        while time.ticks_diff(time.ticks_ms(), started) < seconds * 1000:
            frame = camera.snapshot()
            camera.flush()
            count += 1
        elapsed = time.ticks_diff(time.ticks_ms(), started)
        print("K230_PROBE type=%s frames=%d elapsed_ms=%d capture_preview_fps=%.2f "
              "detector=not_run uart=not_opened heap=%d" % (
                  camera.sensor_type, count, elapsed, count * 1000 / max(1, elapsed), gc.mem_free()))
        print("K230_CAMERA_BINDING", camera.binding)
    finally:
        k230_hal.sensor.close()


if __name__ == "__main__":
    run()

