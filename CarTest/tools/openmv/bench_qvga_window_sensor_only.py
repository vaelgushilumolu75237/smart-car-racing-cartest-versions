"""Sensor-only preview and capture counter for OpenMV H7 Plus.

Run this as a standalone script from the OpenMV IDE. It intentionally does not
import the production camera program, configure UART, or draw overlays. The
sensor is QVGA with an explicit 320x120 center crop: source rows 60..179.
The loop runs for 30 seconds by default and prints one summary every 10 s.
"""

import gc
import sensor
import time

try:
    import machine
except ImportError:
    machine = None


WIDTH = 320
HEIGHT = 120
WINDOW_Y = 60
RUN_MS = 30000
REPORT_MS = 10000
EXPOSURE_CAP_US = 10000
GAIN_DB = 12.0
BENCH_FRAME_MARKER = False
MARKER_SIZE = 4
MARKER_X = 2
MARKER_Y = 2


def _read(obj, name):
    value = getattr(obj, name, None)
    if value is None:
        return "NA"
    if not callable(value):
        return value
    try:
        return value()
    except Exception:
        return "ERR"


def _cpu_hz():
    if machine is None:
        return "NA"
    fn = getattr(machine, "freq", None)
    if fn is None:
        return "NA"
    try:
        value = fn()
        if isinstance(value, (tuple, list)):
            return value[0]
        return value
    except Exception:
        return "ERR"


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((0, WINDOW_Y, WIDTH, HEIGHT))
sensor.set_auto_gain(False, gain_db=GAIN_DB)
sensor.set_auto_exposure(True, exposure_us=EXPOSURE_CAP_US)
sensor.skip_frames(time=1200)

try:
    import omv
    board_name = getattr(omv, "board_type", "NA")
except Exception:
    board_name = "NA"

print("SENSOR_ONLY_BEGIN board=%s id=%s framesize=QVGA window=(0,%d,%d,%d) "
      "cpu_hz=%s exposure_us=%s gain_db=%s gc_free=%s gc_alloc=%s run_ms=%d "
      "frame_marker=%d" % (
          board_name, _read(sensor, "get_id"), WINDOW_Y, WIDTH, HEIGHT,
          _cpu_hz(), _read(sensor, "get_exposure_us"),
          _read(sensor, "get_gain_db"), gc.mem_free(), gc.mem_alloc(), RUN_MS,
          1 if BENCH_FRAME_MARKER else 0))

start_ms = time.ticks_ms()
next_report_ms = REPORT_MS
total_frames = 0
interval_frames = 0
last_report_ms = 0
clock = time.clock()
marker_value = 0


def _draw_frame_marker(image, value):
    """Draw a 16-bit frame counter as sixteen 4x4 black/white blocks."""
    for bit in range(16):
        color = 255 if value & (1 << bit) else 0
        image.draw_rectangle(
            (MARKER_X + bit * MARKER_SIZE, MARKER_Y,
             MARKER_SIZE, MARKER_SIZE), color=color, fill=True)

try:
    while time.ticks_diff(time.ticks_ms(), start_ms) < RUN_MS:
        clock.tick()
        image = sensor.snapshot()
        if total_frames == 0 and (image.width() != WIDTH or
                                  image.height() != HEIGHT):
            raise RuntimeError("sensor returned %dx%d, expected %dx%d" % (
                image.width(), image.height(), WIDTH, HEIGHT))
        if BENCH_FRAME_MARKER:
            _draw_frame_marker(image, marker_value)
            marker_value = (marker_value + 1) & 0xFFFF
        flush = getattr(sensor, "flush", None)
        if callable(flush):
            flush()
        # Keep a live reference only until the next capture; no image processing.
        total_frames += 1
        interval_frames += 1
        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
        if elapsed_ms >= next_report_ms:
            interval_ms = elapsed_ms - last_report_ms
            interval_fps = (interval_frames * 1000.0 / interval_ms
                            if interval_ms > 0 else 0.0)
            print("SENSOR_ONLY_INTERVAL elapsed_ms=%d frames=%d interval_fps=%.2f "
                  "clock_fps=%.2f gc_free=%d" % (
                      elapsed_ms, interval_frames, interval_fps,
                      clock.fps(), gc.mem_free()))
            interval_frames = 0
            last_report_ms = elapsed_ms
            next_report_ms += REPORT_MS
except KeyboardInterrupt:
    print("SENSOR_ONLY_STOP reason=KEYBOARD_INTERRUPT")

elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
average_fps = (total_frames * 1000.0 / elapsed_ms if elapsed_ms > 0 else 0.0)
print("SENSOR_ONLY_END elapsed_ms=%d frames=%d average_fps=%.2f "
      "gc_free=%d gc_alloc=%d" % (
          elapsed_ms, total_frames, average_fps, gc.mem_free(), gc.mem_alloc()))
