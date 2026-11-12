import io
import unittest

from PIL import Image

from tools.openmv_preview_marker import (
    FRAME_HEIGHT,
    FRAME_WIDTH,
    PreviewMarkerTracker,
    decode_preview_marker_rgb888,
    paint_marker_grayscale_for_test,
)


def jpeg_rgb888(sequence, *, corrupt_crc=False, corrupt_magic=False):
    gray = bytearray([96]) * (FRAME_WIDTH * FRAME_HEIGHT)
    paint_marker_grayscale_for_test(gray, sequence)
    if corrupt_crc or corrupt_magic:
        # Flip the center of one solid marker cell; JPEG quality 30 must retain
        # enough contrast for the decoder to reject the altered codeword.
        bit_index = 0 if corrupt_magic else 31
        x0 = 32 + bit_index * 8 + 2
        y0 = 104 + 2
        value = 0 if gray[y0 * FRAME_WIDTH + x0] > 127 else 255
        for y in range(y0, y0 + 4):
            for x in range(x0, x0 + 4):
                gray[y * FRAME_WIDTH + x] = value
    image = Image.frombytes("L", (FRAME_WIDTH, FRAME_HEIGHT), bytes(gray))
    jpeg = io.BytesIO()
    image.save(jpeg, format="JPEG", quality=30)
    decoded = Image.open(io.BytesIO(jpeg.getvalue())).convert("RGB")
    return decoded.tobytes()


class OpenMVPreviewMarkerTests(unittest.TestCase):
    def test_synthetic_quality_30_jpeg_decodes_sequence_and_crc(self):
        for sequence in (0, 1, 0x1234, 0xFFFF):
            with self.subTest(sequence=sequence):
                rgb = jpeg_rgb888(sequence)
                decoded = decode_preview_marker_rgb888(
                    rgb, FRAME_WIDTH, FRAME_HEIGHT)
                self.assertEqual(decoded["status"], "ok")
                self.assertEqual(decoded["sequence"], sequence)
                self.assertEqual(decoded["crc_actual"],
                                 decoded["crc_expected"])

    def test_bad_shape_magic_and_crc_are_distinct(self):
        self.assertEqual(decode_preview_marker_rgb888(b"short", 320, 120)[
            "status"], "shape_error")
        bad_magic = decode_preview_marker_rgb888(
            jpeg_rgb888(12, corrupt_magic=True), 320, 120)
        bad_crc = decode_preview_marker_rgb888(
            jpeg_rgb888(12, corrupt_crc=True), 320, 120)
        self.assertEqual(bad_magic["status"], "magic_mismatch")
        self.assertEqual(bad_crc["status"], "crc_fail")

    def test_tracker_counts_wrap_gap_repeat_out_of_order_and_crc_failure(self):
        tracker = PreviewMarkerTracker()
        for sequence in (65534, 65535, 0, 3, 3, 2):
            tracker.observe_rgb888(jpeg_rgb888(sequence), 320, 120)
        tracker.observe_rgb888(jpeg_rgb888(4, corrupt_crc=True), 320, 120)
        report = tracker.report()
        self.assertEqual(report["marker_read_attempts"], 7)
        self.assertEqual(report["valid_marker_reads"], 6)
        self.assertEqual(report["new_sequence_reads"], 4)
        self.assertEqual(report["sequence_wraps"], 1)
        self.assertEqual(report["sequence_gap_events"], 1)
        self.assertEqual(report["missing_sequence_values"], 2)
        self.assertEqual(report["repeated_sequence_reads"], 1)
        self.assertEqual(report["out_of_order_sequence_reads"], 1)
        self.assertEqual(report["crc_failures"], 1)
        self.assertEqual(report["last_sequence"], 3)
        self.assertFalse(report["camera_fps_claimed_by_host"])


if __name__ == "__main__":
    unittest.main()
