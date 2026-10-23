"""RAM-only USB preview probe requiring the optional OpenMV vendor patch.

Run from the OpenMV IDE with the patched firmware and production files. It
keeps the detector input at the configured 320x120 grayscale crop, sends valid
RESET/KEEP control frames into the production path, discards UART output, and
publishes an 80x30 JPEG every fourth frame. It does not connect to the H743.

This is a candidate comparison workload. The preview is smaller and decimated,
so its FPS is not equivalent to full-resolution, every-frame IDE preview.
"""

import gc
import time

import main
import reverse_control
import sensor
import csi


if getattr(sensor, "PREVIEW_CONTROL_API", None) != 1:
    raise RuntimeError(
        "preview probe requires patched sensor module with "
        "PREVIEW_CONTROL_API=1")
if getattr(csi, "PREVIEW_CONTROL_API", None) != 1:
    raise RuntimeError(
        "preview probe requires patched csi module with "
        "PREVIEW_CONTROL_API=1")


RUN_SECONDS = 30
RUN_ID = 20261024
PREVIEW_EVERY_N = 4
SCALE = 0.25
PREVIEW_QUALITY = 50

if PREVIEW_EVERY_N < 1:
    raise ValueError("PREVIEW_EVERY_N must be at least 1")
if SCALE <= 0.0 or SCALE > 1.0:
    raise ValueError("SCALE must be in (0, 1]")


class _Source:
    name = "LIVE_SENSOR_PREVIEW_FALSE_SCALED_DECIMATED"

    def __init__(self):
        self.frame_count = 0
        self.latest_image = None

    def snapshot(self):
        # The patched API suppresses automatic full-frame IDE preview for this
        # capture while returning the untouched detector input image.
        self.latest_image = sensor.snapshot(preview=False)
        self.frame_count += 1
        return self.latest_image

    def flush(self):
        if (self.latest_image is None or
                self.frame_count % PREVIEW_EVERY_N):
            return

        preview = self.latest_image.to_jpeg(
            x_scale=SCALE,
            y_scale=SCALE,
            quality=PREVIEW_QUALITY,
            copy=True)
        preview.flush()


class _Sink:
    """RESET/KEEP input and discard output, matching RAM running bench."""

    def __init__(self):
        self.pending = reverse_control.pack_frame(
            reverse_control.COMMAND_RESET, 0)
        self.seq = 1
        self.due = time.ticks_ms()

    def write(self, packet):
        return len(packet)

    def any(self):
        if not self.pending and time.ticks_diff(
                time.ticks_ms(), self.due) >= 0:
            self.pending = reverse_control.pack_frame(
                reverse_control.COMMAND_KEEP, self.seq)
            self.seq = (self.seq + 1) & 65535
            self.due = time.ticks_add(time.ticks_ms(), 50)
        return len(self.pending)

    def readinto(self, buffer, requested=None):
        count = min(len(buffer), len(self.pending))
        if requested is not None:
            count = min(count, requested)
        for index in range(count):
            buffer[index] = self.pending[index]
        self.pending = self.pending[count:]
        return count


print("RAM_USB_PREVIEW_PROBE start: input=production 320x120 grayscale, "
      "preview scale=", SCALE, " every=", PREVIEW_EVERY_N,
      " frames, UART=RESET/KEEP sink")
gc.collect()
main._configure_gray_sensor(sensor, fast=True)
main._main_fast(
    duration_s=RUN_SECONDS,
    run_id=RUN_ID,
    frame_source=_Source(),
    uart_sink=_Sink())
