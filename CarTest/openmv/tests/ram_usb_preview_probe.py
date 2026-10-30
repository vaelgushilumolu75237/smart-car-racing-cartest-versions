"""RAM-only USB preview probe requiring the optional OpenMV vendor patch.

Run from the OpenMV IDE with the patched firmware and production files. It
keeps the detector input at the configured 320x120 grayscale crop, sends valid
RESET/KEEP control frames into the production path, discards UART output, and
publishes a full-size JPEG every frame by default. It does not connect to the
H743.

The 32-bit binary frame marker (D3 magic, sequence16, CRC-8/ATM) is painted
only after detector processing and the UART sink write. It adds per-frame
marker work and covers an eight-row strip near the bottom of the preview.
Board-side counters and PC-observed marker IDs are separate measurements; do
not infer unique displayed frames from image hashes.
"""

import gc
import micropython
import time

import main
import reverse_control
import sensor
try:
    import csi
except ImportError:
    csi = None


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
PREVIEW_EVERY_N = 1
SCALE = 1.0
PREVIEW_QUALITY = 30

if PREVIEW_EVERY_N < 1:
    raise ValueError("PREVIEW_EVERY_N must be at least 1")
if SCALE <= 0.0 or SCALE > 1.0:
    raise ValueError("SCALE must be in (0, 1]")
if PREVIEW_QUALITY < 1 or PREVIEW_QUALITY > 100:
    raise ValueError("PREVIEW_QUALITY must be between 1 and 100")


@micropython.viper
def _write_preview_marker_viper(raw, sequence: int) -> int:
    pixels = ptr8(raw)
    seq = sequence & 0xFFFF
    crc = 0
    for byte_index in range(3):
        if byte_index == 0:
            value = 0xD3
        elif byte_index == 1:
            value = (seq >> 8) & 0xFF
        else:
            value = seq & 0xFF
        crc = crc ^ value
        for crc_bit in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    for bit_index in range(32):
        if bit_index < 8:
            bit = (0xD3 >> (7 - bit_index)) & 1
        elif bit_index < 24:
            bit = (seq >> (23 - bit_index)) & 1
        else:
            bit = (crc >> (31 - bit_index)) & 1
        value = 0
        if bit:
            value = 255
        x = 32 + bit_index * 8
        for dy in range(8):
            row = (104 + dy) * 320 + x
            for dx in range(8):
                pixels[row + dx] = value
    return crc


class _Source:
    name = "LIVE_SENSOR_PREVIEW_FALSE_FULLSIZE_EVERY_FRAME"

    def __init__(self):
        self.frame_count = 0
        self.preview_count = 0
        self.marker_writes = 0
        self.marker_total_us = 0
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

        # _main_fast calls this after processing and the UART sink write.
        # This binary marker matches tools/openmv_preview_marker.py exactly.
        started = time.ticks_us()
        _write_preview_marker_viper(
            self.latest_image.bytearray(), self.frame_count)
        self.marker_total_us += time.ticks_diff(time.ticks_us(), started)
        self.marker_writes += 1

        # OpenMV v5.0.0 defaults image.to_jpeg(copy=False). That mode can
        # convert the source image in place. copy=True allocates an independent
        # dynamic Image, preserving latest_image for any remaining processing
        # and the next framebuffer acquisition.
        preview = self.latest_image.to_jpeg(
            x_scale=SCALE,
            y_scale=SCALE,
            quality=PREVIEW_QUALITY,
            copy=True)
        preview.flush()
        self.preview_count += 1
        self.latest_image = None


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


print("RAM_USB_PREVIEW_PROBE start: detector=320x120 grayscale, "
      "preview=%dx%d JPEG q=%d" % (int(320 * SCALE), int(120 * SCALE),
                                    PREVIEW_QUALITY),
      " scale=", SCALE, " every=", PREVIEW_EVERY_N,
      " frame(s), marker=D3/seq16/crc8 x=32..287 y=104..111, "
      "UART=RESET/KEEP sink")
gc.collect()
main._configure_gray_sensor(sensor, fast=True)
source = _Source()
main._main_fast(
    duration_s=RUN_SECONDS,
    run_id=RUN_ID,
    frame_source=source,
    uart_sink=_Sink())
print("BOARD_PROBE_COUNTS input_frames=%d preview_publishes=%d "
      "marker_writes=%d marker_total_us=%d" %
      (source.frame_count, source.preview_count,
       source.marker_writes, source.marker_total_us))

print("RAM_USB_PREVIEW_PROBE_FINISHED")
# Keep the protocol alive briefly so the host can drain final counters.
time.sleep_ms(2000)
