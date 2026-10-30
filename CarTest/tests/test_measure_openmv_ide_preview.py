from __future__ import annotations

import argparse
import ast
import ctypes
from contextlib import redirect_stderr
import io
from pathlib import Path
import time
import unittest
from ctypes import wintypes

from PIL import Image, ImageDraw


HELPER_PATH = (Path(__file__).resolve().parents[1] / "tmp" /
               "measure_openmv_ide_preview.py")


def load_portable_marker_helpers():
    """Load marker math/CLI functions without importing Windows user32."""
    tree = ast.parse(HELPER_PATH.read_text(encoding="utf-8"))
    selected = {
        "parse_box", "gray_to_binary", "crc8_atm", "sample_marker",
        "sample_marker_bgra", "decode_marker_bits", "decode_marker_at",
        "foreground_window_handle", "request_window_foreground", "parse_args",
    }
    definitions = [
        node for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and
        node.name in selected]
    if {node.name for node in definitions} != selected:
        raise AssertionError("screen helper marker functions changed")
    namespace = {"argparse": argparse, "Path": Path, "time": time}
    module = ast.Module(body=definitions, type_ignores=[])
    exec(compile(module, str(HELPER_PATH), "exec"), namespace)
    return namespace


def load_dib_capture_class(user32, gdi32):
    tree = ast.parse(HELPER_PATH.read_text(encoding="utf-8"))
    selected = {
        "_RGBQUAD", "_BITMAPINFOHEADER", "_BITMAPINFO",
        "FixedDibScreenCapture",
    }
    definitions = [
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name in selected]
    if {node.name for node in definitions} != selected:
        raise AssertionError("fixed DIB capture classes changed")
    namespace = {
        "ctypes": ctypes,
        "wintypes": wintypes,
        "USER32": user32,
        "GDI32": gdi32,
        "BI_RGB": 0,
        "DIB_RGB_COLORS": 0,
        "SRCCOPY": 0x00CC0020,
        "CAPTUREBLT": 0x40000000,
    }
    module = ast.Module(body=definitions, type_ignores=[])
    exec(compile(module, str(HELPER_PATH), "exec"), namespace)
    return namespace["FixedDibScreenCapture"]


def marker_image(frame_id: int, corrupt_crc: bool = False) -> Image.Image:
    frame_id &= 0xFFFFFF
    gray_value = (frame_id ^ (frame_id >> 1)) & 0xFFFFFF
    gray_bytes = bytes((gray_value & 0xFF,
                        (gray_value >> 8) & 0xFF,
                        (gray_value >> 16) & 0xFF))
    namespace = load_portable_marker_helpers()
    checksum = namespace["crc8_atm"](gray_bytes)
    encoded = gray_value | (checksum << 24)
    if corrupt_crc:
        encoded ^= 1 << 31

    tile = 8
    pitch = 10
    image = Image.new("L", (31 * pitch + tile, tile), color=96)
    draw = ImageDraw.Draw(image)
    for bit_index in range(32):
        value = 255 if (encoded >> bit_index) & 1 else 0
        x = bit_index * pitch
        draw.rectangle((x, 0, x + tile - 1, tile - 1), fill=value)
    return image


class MeasureOpenMVIdePreviewTests(unittest.TestCase):
    def test_gray_crc8_marker_decodes_lsb_first_and_checks_crc(self):
        helpers = load_portable_marker_helpers()
        frame_id = 0x123456
        image = marker_image(frame_id)
        decoded, bits, crc_ok = helpers["sample_marker"](
            image, (0, 0), pitch=10, tile=8, bit_count=32,
            threshold=128, encoding="gray-crc8")
        self.assertEqual(decoded, frame_id)
        self.assertEqual(len(bits), 32)
        self.assertTrue(crc_ok)
        self.assertEqual(
            helpers["decode_marker_at"](
                image, (0, 0), pitch=10, tile=8, threshold=128),
            frame_id)

        invalid, invalid_bits, invalid_crc = helpers["sample_marker"](
            marker_image(frame_id, corrupt_crc=True), (0, 0), 10, 8, 32,
            128, "gray-crc8")
        self.assertIsNone(invalid)
        self.assertEqual(len(invalid_bits), 32)
        self.assertFalse(invalid_crc)

        gray = image.tobytes()
        bgra = bytearray(len(gray) * 4)
        for index, value in enumerate(gray):
            offset = index * 4
            bgra[offset:offset + 4] = bytes((value, value, value, 0))
        decoded_bgra, bgra_bits, bgra_crc_ok = helpers["sample_marker_bgra"](
            bgra, image.width, image.height, (0, 0), 10, 8, 32, 128,
            "gray-crc8")
        self.assertEqual(decoded_bgra, frame_id)
        self.assertEqual(bgra_bits, bits)
        self.assertTrue(bgra_crc_ok)

    def test_fixed_dib_capture_reuses_buffer_and_blits_only_requested_strip(self):
        class FakeUser32:
            def __init__(self):
                self.released = []

            def GetDC(self, _window):
                return 101

            def ReleaseDC(self, window, device_context):
                self.released.append((window, device_context))
                return 1

        class FakeGDI32:
            def __init__(self):
                self.buffer = None
                self.info_size = None
                self.info_width = None
                self.info_height = None
                self.blit_calls = []
                self.deleted_objects = []
                self.deleted_dcs = []

            def CreateCompatibleDC(self, _screen_dc):
                return 202

            def CreateDIBSection(self, _screen_dc, info, _colors, bits_out,
                                 _section, _offset):
                header = info._obj.bmiHeader
                self.info_size = header.biSizeImage
                self.info_width = header.biWidth
                self.info_height = header.biHeight
                self.buffer = (ctypes.c_ubyte * self.info_size)()
                ctypes.cast(bits_out, ctypes.POINTER(ctypes.c_void_p))[0] = (
                    ctypes.addressof(self.buffer))
                return 303

            def SelectObject(self, _memory_dc, selected):
                return 404 if selected == 303 else 303

            def BitBlt(self, _dest, _x, _y, width, height, _source,
                       screen_x, screen_y, raster_op):
                self.blit_calls.append(
                    (width, height, screen_x, screen_y, raster_op))
                ctypes.memset(ctypes.addressof(self.buffer), 77,
                              self.info_size)
                return 1

            def GdiFlush(self):
                return 1

            def DeleteObject(self, obj):
                self.deleted_objects.append(obj)
                return 1

            def DeleteDC(self, dc):
                self.deleted_dcs.append(dc)
                return 1

        user32 = FakeUser32()
        gdi32 = FakeGDI32()
        capture_type = load_dib_capture_class(user32, gdi32)
        capture = capture_type(318, 8)
        first_buffer = capture.grab(111, 222)
        first_identity = id(first_buffer)
        second_buffer = capture.grab(333, 444)
        self.assertEqual(id(second_buffer), first_identity)
        self.assertEqual(gdi32.info_width, 318)
        self.assertEqual(gdi32.info_height, -8)
        self.assertEqual(gdi32.info_size, 318 * 8 * 4)
        self.assertEqual(gdi32.blit_calls, [
            (318, 8, 111, 222, 0x00CC0020 | 0x40000000),
            (318, 8, 333, 444, 0x00CC0020 | 0x40000000),
        ])
        image = capture.to_pillow_image()
        self.assertEqual(image.size, (318, 8))
        self.assertEqual(image.getpixel((0, 0)), (77, 77, 77))
        capture.close()
        self.assertEqual(user32.released, [(None, 101)])
        self.assertEqual(gdi32.deleted_objects, [303])
        self.assertEqual(gdi32.deleted_dcs, [202])

    def test_cli_accepts_visible_probe_layout_and_rejects_bad_width(self):
        parse_args = load_portable_marker_helpers()["parse_args"]
        args = parse_args([
            "--pid", "1234", "--auto-locate-marker",
            "--activate-window",
            "--bit-count", "32", "--marker-encoding", "gray-crc8",
            "--output-dir", "probe-output",
        ])
        self.assertEqual(args.bit_count, 32)
        self.assertEqual(args.marker_encoding, "gray-crc8")
        self.assertEqual(args.marker_tile_px, 8)
        self.assertEqual(args.marker_pitch_px, 10)
        self.assertTrue(args.activate_window)
        defaults = parse_args([
            "--pid", "1234", "--marker-origin", "8,8,8,10",
            "--bit-count", "32", "--marker-encoding", "gray-crc8",
            "--output-dir", "probe-output",
        ])
        self.assertFalse(defaults.activate_window)

        stderr = io.StringIO()
        with redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
            parse_args([
                "--pid", "1234", "--auto-locate-marker",
                "--bit-count", "24", "--marker-encoding", "gray-crc8",
                "--output-dir", "probe-output",
            ])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("gray-crc8 requires --bit-count 32", stderr.getvalue())

    def test_foreground_activation_requires_observed_target_handle(self):
        helpers = load_portable_marker_helpers()

        class FakeUser32:
            def __init__(self, activates):
                self.foreground = 11
                self.activates = activates

            def GetForegroundWindow(self):
                return self.foreground

            def SetForegroundWindow(self, target):
                if self.activates:
                    self.foreground = target
                return True

        helpers["USER32"] = FakeUser32(True)
        result = helpers["request_window_foreground"](22)
        self.assertTrue(result["set_foreground_window_returned"])
        self.assertEqual(result["foreground_window_after_request"], 22)

        class ImmediateFakeTime:
            now = 0.0

            @classmethod
            def perf_counter(cls):
                return cls.now

            @classmethod
            def sleep(cls, seconds):
                cls.now += seconds

        helpers["time"] = ImmediateFakeTime
        helpers["USER32"] = FakeUser32(False)
        with self.assertRaisesRegex(RuntimeError,
                                    "did not grant IDE foreground activation"):
            helpers["request_window_foreground"](22)

        helpers["USER32"] = type("EmptyForegroundUser32", (), {
            "GetForegroundWindow": lambda self: None})()
        self.assertEqual(helpers["foreground_window_handle"](), 0)


if __name__ == "__main__":
    unittest.main()
