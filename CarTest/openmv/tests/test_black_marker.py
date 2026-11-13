"""Synthetic texture/packet tests; no claim of mounted-camera acceptance."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
import fast_gray
import vision_protocol as vp


class BlackMarkerTests(unittest.TestCase):
    def setUp(self):
        self.builder = fast_gray.FastGrayMasks(config)
        self.builder.luma[:] = b'\xdc' * 4800
        self.builder.thresholds[-1] = 55

    def paint(self, y, count=8, offset=0):
        # Several separated short dark strokes, not an exact-eight contract.
        for bar in range(count):
            x = 12 + offset + bar * 56 // count
            for col in range(x, min(x + 3, 68)):
                self.builder.luma[y * 80 + col] = 25

    def test_seven_eight_ten_strokes_detected_without_exact_counting(self):
        for count in (7, 8, 10):
            with self.subTest(count=count):
                self.setUp()
                self.paint(54, count)
                self.assertEqual(self.builder.black_marker_present(), 1)

    def test_only_fixed_bottom_roi_is_read_and_buffers_unchanged(self):
        self.paint(47)
        self.assertEqual(self.builder.black_marker_present(), 0)
        self.paint(59)
        before = bytes(self.builder.luma)
        edge = bytes(self.builder.edge_mask)
        self.assertEqual(self.builder.black_marker_present(), 1)
        self.assertEqual(bytes(self.builder.luma), before)
        self.assertEqual(bytes(self.builder.edge_mask), edge)
        self.builder.luma[59 * 80:60 * 80] = b'\xdc' * 80
        self.assertEqual(self.builder.black_marker_present(), 0)

    def test_ordinary_borders_bends_solid_crossbar_and_blank_rejected(self):
        self.assertEqual(self.builder.black_marker_present(), 0)
        for y in range(48, 60):
            for x in (14, 15, 64, 65):
                self.builder.luma[y * 80 + x] = 25
        self.assertEqual(self.builder.black_marker_present(), 0)
        for y in range(48, 60):
            for x in range(12, 35 + (y - 48)):
                self.builder.luma[y * 80 + x] = 25
        self.assertEqual(self.builder.black_marker_present(), 0)
        self.builder.luma[:] = b'\x19' * 4800
        self.assertEqual(self.builder.black_marker_present(), 0)

    def test_reuses_bottom_threshold_and_legacy_fallback(self):
        self.paint(54)
        self.builder.thresholds[-1] = 20
        self.assertEqual(self.builder.black_marker_present(), 0)
        self.builder.thresholds[-1] = 55
        self.assertEqual(self.builder.black_marker_present(), 1)
        self.builder.thresholds[-1] = None
        self.assertEqual(self.builder.black_marker_present(), 1)

    def test_example_blue_sides_white_lane_and_eight_parallel_black_bars(self):
        for y in range(60):
            for x in range(80):
                self.builder.luma[y * 80 + x] = 220 if 10 <= x < 70 else 30
        self.assertEqual(self.builder.black_marker_present(), 0)
        for y in range(48, 57):
            self.paint(y)
        self.assertEqual(self.builder.black_marker_present(), 1)

    def test_invalid_roi_cannot_expand_scan_to_full_image(self):
        for roi in ((0, 0, 80, 60), (12, 48, 60, 12), (12, 48, 56, 13)):
            with self.assertRaises(ValueError):
                fast_gray.FastGrayMasks(SimpleNamespace(BLACK_MARKER_GRID_ROI=roi))

    def test_v4_marker_is_independent_of_route_validity_crc_and_size(self):
        for valid in (0, 1):
            for present, flags in ((None, 0), (0, 4), (1, 12)):
                packet = bytearray(16)
                vp.pack_into_v4(packet, 65535, valid, 90 if valid else 0,
                                123 if valid else 0, profile_ack=3,
                                ordinary_route=valid, marker_present=present)
                self.assertEqual(packet[6], valid | flags)
                self.assertEqual(len(packet), 16)
                self.assertEqual(packet[10], valid << 6)
                self.assertEqual(vp.crc16_ccitt_false(packet[2:14]),
                                 packet[14] | packet[15] << 8)
        with self.assertRaises(ValueError):
            vp.pack_into_v4(bytearray(16), 0, 0, 0, 0, marker_present=2)

    def test_same_frame_integration_and_no_extra_uart_write(self):
        source = (Path(__file__).resolve().parents[1] / 'main.py').read_text(encoding='utf-8')
        self.assertIn('marker_present = gray_builder.black_marker_present()', source)
        self.assertEqual(source.count('marker_present=marker_present'), 2)


if __name__ == '__main__':
    unittest.main()
