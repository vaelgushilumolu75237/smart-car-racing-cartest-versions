"""Decode the preview probe's binary sequence marker from host RGB888 frames.

Freshness is counted from valid forward sequence transitions. Image content
hashes are deliberately not used here because an unchanged scene can produce
identical pixels for many distinct camera frames.
"""

from __future__ import annotations


FRAME_WIDTH = 320
FRAME_HEIGHT = 120
MARKER_X = 32
MARKER_Y = 104
CELL_SIZE = 8
BIT_COUNT = 32
MAGIC = 0xD3
SAMPLE_START = 2
SAMPLE_SIZE = 4


def crc8_atm(data):
    """Return CRC-8/ATM (poly 0x07, init/xorout 0) for bytes-like input."""
    crc = 0
    for value in data:
        crc ^= int(value)
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def marker_payload(sequence):
    """Return the 32 marker bits as four bytes: magic, seq16, CRC-8."""
    sequence = int(sequence) & 0xFFFF
    prefix = bytes((MAGIC, sequence >> 8, sequence & 0xFF))
    return prefix + bytes((crc8_atm(prefix),))


def _failure(status, **fields):
    result = {"status": status, "sequence": None,
              "magic": None, "crc_expected": None, "crc_actual": None}
    result.update(fields)
    return result


def decode_preview_marker_rgb888(data, width, height):
    """Decode marker bits from one packed RGB888 frame.

    The decoder samples a centered 4x4 area inside each solid 8x8 marker cell
    and averages integer-weighted luma. This leaves a two-pixel JPEG ringing
    margin around every cell edge. Returns a status of ``ok``, ``shape_error``,
    ``magic_mismatch``, or ``crc_fail`` and never infers freshness from pixels.
    """
    try:
        width = int(width)
        height = int(height)
        pixels = memoryview(data).cast("B")
    except (TypeError, ValueError, OverflowError):
        return _failure("shape_error")
    if (width < MARKER_X + BIT_COUNT * CELL_SIZE or
            height < MARKER_Y + CELL_SIZE or
            len(pixels) != width * height * 3):
        return _failure("shape_error", width=width, height=height,
                        rgb888_bytes=len(pixels))

    marker = 0
    for bit_index in range(BIT_COUNT):
        cell_x = MARKER_X + bit_index * CELL_SIZE
        luminance_sum = 0
        sample_count = SAMPLE_SIZE * SAMPLE_SIZE
        for dy in range(SAMPLE_START, SAMPLE_START + SAMPLE_SIZE):
            row_offset = (MARKER_Y + dy) * width * 3
            for dx in range(SAMPLE_START, SAMPLE_START + SAMPLE_SIZE):
                offset = row_offset + (cell_x + dx) * 3
                red = pixels[offset]
                green = pixels[offset + 1]
                blue = pixels[offset + 2]
                luminance_sum += 299 * red + 587 * green + 114 * blue
        bit = 1 if luminance_sum >= 128000 * sample_count else 0
        marker = (marker << 1) | bit

    magic = (marker >> 24) & 0xFF
    sequence = (marker >> 8) & 0xFFFF
    actual_crc = marker & 0xFF
    prefix = bytes((magic, sequence >> 8, sequence & 0xFF))
    expected_crc = crc8_atm(prefix)
    result = {
        "status": "ok",
        "sequence": sequence,
        "magic": magic,
        "crc_expected": expected_crc,
        "crc_actual": actual_crc,
    }
    if magic != MAGIC:
        result["status"] = "magic_mismatch"
    elif actual_crc != expected_crc:
        result["status"] = "crc_fail"
    return result


def paint_marker_grayscale_for_test(frame, sequence, width=FRAME_WIDTH,
                                    height=FRAME_HEIGHT):
    """Paint the marker into a mutable grayscale test image.

    This helper is host-test-only. The board harness uses a Viper loop to write
    the same cells after the detector has consumed the original grayscale data.
    """
    width = int(width)
    height = int(height)
    pixels = memoryview(frame).cast("B")
    if len(pixels) != width * height:
        raise ValueError("grayscale buffer must be width*height bytes")
    if (width < MARKER_X + BIT_COUNT * CELL_SIZE or
            height < MARKER_Y + CELL_SIZE):
        raise ValueError("image is too small for the preview marker")
    payload = marker_payload(sequence)
    for bit_index in range(BIT_COUNT):
        byte_value = payload[bit_index // 8]
        bit = (byte_value >> (7 - (bit_index % 8))) & 1
        value = 255 if bit else 0
        x0 = MARKER_X + bit_index * CELL_SIZE
        for y in range(MARKER_Y, MARKER_Y + CELL_SIZE):
            row = y * width
            pixels[row + x0:row + x0 + CELL_SIZE] = bytes((value,)) * CELL_SIZE
    return frame


class PreviewMarkerTracker:
    """Track valid marker sequence transitions in host-returned RGB888 frames."""

    def __init__(self):
        self.marker_read_attempts = 0
        self.valid_marker_reads = 0
        self.new_sequence_reads = 0
        self.repeated_sequence_reads = 0
        self.out_of_order_sequence_reads = 0
        self.sequence_gap_events = 0
        self.missing_sequence_values = 0
        self.sequence_wraps = 0
        self.crc_failures = 0
        self.magic_failures = 0
        self.shape_failures = 0
        self.last_sequence = None

    def observe_rgb888(self, data, width, height):
        decoded = decode_preview_marker_rgb888(data, width, height)
        self.marker_read_attempts += 1
        status = decoded["status"]
        if status == "crc_fail":
            self.crc_failures += 1
            return decoded
        if status == "magic_mismatch":
            self.magic_failures += 1
            return decoded
        if status != "ok":
            self.shape_failures += 1
            return decoded

        self.valid_marker_reads += 1
        sequence = decoded["sequence"]
        if self.last_sequence is None:
            self.new_sequence_reads += 1
            self.last_sequence = sequence
            return decoded

        delta = (sequence - self.last_sequence) & 0xFFFF
        if delta == 0:
            self.repeated_sequence_reads += 1
            return decoded
        if delta >= 0x8000:
            self.out_of_order_sequence_reads += 1
            return decoded

        if sequence < self.last_sequence:
            self.sequence_wraps += 1
        if delta > 1:
            self.sequence_gap_events += 1
            self.missing_sequence_values += delta - 1
        self.new_sequence_reads += 1
        self.last_sequence = sequence
        return decoded

    def report(self):
        return {
            "marker_read_attempts": self.marker_read_attempts,
            "valid_marker_reads": self.valid_marker_reads,
            "new_sequence_reads": self.new_sequence_reads,
            "repeated_sequence_reads": self.repeated_sequence_reads,
            "out_of_order_sequence_reads": self.out_of_order_sequence_reads,
            "sequence_gap_events": self.sequence_gap_events,
            "missing_sequence_values": self.missing_sequence_values,
            "sequence_wraps": self.sequence_wraps,
            "crc_failures": self.crc_failures,
            "magic_failures": self.magic_failures,
            "shape_failures": self.shape_failures,
            "last_sequence": self.last_sequence,
            "fresh_frame_metric_basis": (
                "valid forward marker sequence transitions; image hashes are "
                "not used to count fresh frames"),
            "camera_fps_claimed_by_host": False,
        }
