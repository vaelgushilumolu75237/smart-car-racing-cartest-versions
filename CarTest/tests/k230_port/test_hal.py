import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openmv"))
import k230_hal as hal
import ring_width_calibration as calibration
import k230_calibration as binding


class UARTDevice:
    EIGHTBITS = 8
    PARITY_NONE = 0
    STOPBITS_ONE = 0
    last = None

    def __init__(self, *args, **kwargs):
        type(self).last = self
        self.args, self.kwargs = args, kwargs
        self.rx = [b"\xa6\x5a\x00\xff", None]
        self.writes = []
        self.result = 16
        self.closed = False

    def read(self, count):
        assert count == 256
        return self.rx.pop(0) if self.rx else None

    def readinto(self, buffer):
        return None

    def write(self, data):
        self.writes.append(bytes(data))
        return self.result

    def deinit(self):
        self.closed = True


class FPIOA:
    UART1_TXD = 101
    UART1_RXD = 102
    mappings = []

    def set_function(self, *args, **kwargs):
        self.mappings.append((args, kwargs))


class PreviewImage:
    GRAYSCALE = 1
    created = []

    def __init__(self, width, height, pixel_format):
        self.width_value = width
        self.height_value = height
        self.pixel_format = pixel_format
        self.clears = 0
        self.draws = []
        type(self).created.append(self)

    def clear(self):
        self.clears += 1

    def draw_image(self, source, x, y, **kwargs):
        self.draws.append((source, x, y, kwargs))


class HALTests(unittest.TestCase):
    def setUp(self):
        hal._uarts[:] = []
        FPIOA.mappings[:] = []
        self.machine = types.SimpleNamespace(UART=UARTDevice, FPIOA=FPIOA)
        self.patch = patch.dict(sys.modules, {"machine": self.machine})
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        hal._uarts[:] = []
        PreviewImage.created[:] = []

    def test_factory_pin_mapping_uses_one_stop_not_openmv_numeric_one(self):
        hal.UARTAdapter(1, 115200)
        self.assertEqual(UARTDevice.last.kwargs,
                         dict(baudrate=115200, bits=8, parity=0, stop=0))
        self.assertEqual([9, 10], [m[0][0] for m in FPIOA.mappings])
        self.assertEqual([dict(ie=0, oe=1), dict(ie=1, oe=0)],
                         [m[1] for m in FPIOA.mappings])

    def test_no_any_binary_prefetch_preserves_split_frames_and_zero_bytes(self):
        uart = hal.UARTAdapter(1, 115200)
        self.assertEqual(uart.any(), 4)
        self.assertEqual(uart.any(), 4)
        small = bytearray(2)
        self.assertEqual(uart.readinto(small), 2)
        self.assertEqual(small, b"\xa6\x5a")
        self.assertEqual(uart.any(), 2)
        self.assertEqual(uart.readinto(small), 2)
        self.assertEqual(small, b"\x00\xff")
        self.assertEqual(uart.any(), 0)

    def test_none_and_short_write_are_not_promoted_to_success(self):
        uart = hal.UARTAdapter(1, 115200)
        for result in (None, 0, 7, 16):
            uart.device.result = result
            self.assertEqual(uart.write(b"x" * 16), result)

    def test_requested_read_preserves_prefetched_tail_and_buffer_boundary(self):
        uart = hal.UARTAdapter(1, 115200)
        self.assertEqual(uart.any(), 4)
        buffer = bytearray(b"????????")
        self.assertEqual(uart.readinto(buffer, 2), 2)
        self.assertEqual(buffer, b"\xa6\x5a??????")
        self.assertEqual(uart.any(), 2)
        self.assertEqual(uart.readinto(buffer, 1), 1)
        self.assertEqual(buffer, b"\x00\x5a??????")
        self.assertEqual(uart.any(), 1)
        self.assertEqual(uart.readinto(buffer, 0), 0)
        self.assertEqual(uart.any(), 1)
        self.assertEqual(uart.readinto(buffer, 99), 1)
        self.assertEqual(buffer[0], 0xff)

    def test_requested_read_uses_native_one_argument_readinto(self):
        uart = hal.UARTAdapter(1, 115200)
        pending = bytearray(b"abcdef")
        calls = []

        def native_readinto(buffer):
            calls.append(len(buffer))
            count = min(len(buffer), len(pending))
            buffer[:count] = pending[:count]
            del pending[:count]
            return count

        uart.device.any = lambda: len(pending)
        uart.device.readinto = native_readinto
        buffer = bytearray(b"????????")
        self.assertEqual(uart.any(), 6)
        self.assertEqual(uart.readinto(buffer, 3), 3)
        self.assertEqual(buffer, b"abc?????")
        self.assertEqual(pending, b"def")
        self.assertEqual(uart.readinto(buffer, 0), 0)
        self.assertEqual(calls, [3])
        self.assertEqual(uart.readinto(buffer), 3)
        self.assertEqual(calls, [3, 8])
        uart.device.readinto = lambda buffer: None
        self.assertIsNone(uart.readinto(buffer, 1))

    def test_production_receive_service_accepts_profile_and_wireless_hello(self):
        import main as camera_main
        import reverse_control as reverse
        import wireless_link as wireless

        epoch = 0x12345678
        profile = reverse.pack_profile_state(
            1, reverse.PROFILE_C,
            reverse.PROFILE_SESSION_ACTIVE | reverse.PROFILE_GYRO350_SUPPORTED)
        hello = wireless.encode_frame(wireless.TYPE_HELLO, 1, epoch, 1000,
                                     epoch.to_bytes(4, "little") + b"\x01\x00\x00\x00")
        for native_any in (False, True):
            with self.subTest(native_any=native_any), \
                    patch.object(camera_main, "reverse_control", reverse):
                uart = hal.UARTAdapter(1, 115200)
                stream = bytearray(profile + hello)
                if native_any:
                    uart.device.any = lambda: len(stream)

                    def native_readinto(buffer):
                        count = min(len(buffer), len(stream))
                        buffer[:count] = stream[:count]
                        del stream[:count]
                        return count

                    uart.device.readinto = native_readinto
                else:
                    uart.device.rx = [bytes(stream[:9]), bytes(stream[9:])]
                parser = reverse.ReverseControlParser()
                link = wireless.WirelessCameraLink(enabled=True, parked_cb=lambda: True)
                drain = bytearray(1)
                for step in range(3):
                    camera_main._service_reverse_control(
                        uart, parser, bytearray(reverse.MAX_FEED_BYTES), drain,
                        1000 + step, lambda now, then: now - then, link)
                self.assertEqual(drain[0], 0)
                self.assertTrue(parser.profile_session_active)
                self.assertEqual(link.session, epoch)
                uart.device.write = lambda data: (
                    uart.device.writes.append(bytes(data)) or len(data))
                link.service_tx(uart, 1004)
                reply = wireless.decode_frame(uart.device.writes[-1])
                self.assertEqual(reply["type"], wireless.TYPE_HELLO_REPLY)
                self.assertEqual(reply["session"], epoch)

    def test_production_drain_keeps_hello_but_drops_stale_profile(self):
        import main as camera_main
        import reverse_control as reverse
        import wireless_link as wireless

        uart = hal.UARTAdapter(1, 115200)
        epoch = 17
        profile = reverse.pack_profile_state(
            1, reverse.PROFILE_C, reverse.PROFILE_SESSION_ACTIVE)
        hello = wireless.encode_frame(wireless.TYPE_HELLO, 1, epoch, 1000,
                                     epoch.to_bytes(4, "little") + b"\x01\x00\x00\x00")
        # A burst over the existing 64-byte limit enters the drain path. The
        # adapter must retain the unread suffix, including a split HELLO.
        uart.device.rx = [b"\x00" * 32 + profile + hello]
        parser = reverse.ReverseControlParser()
        link = wireless.WirelessCameraLink(enabled=True, parked_cb=lambda: True)
        drain = bytearray(1)
        with patch.object(camera_main, "reverse_control", reverse):
            for step in range(3):
                camera_main._service_reverse_control(
                    uart, parser, bytearray(reverse.MAX_FEED_BYTES), drain,
                    1000 + step, lambda now, then: now - then, link)
        self.assertEqual(drain[0], 0)
        self.assertFalse(parser.profile_session_active)
        self.assertEqual(link.session, epoch)

    def test_oversized_or_nonbinary_receive_is_rejected(self):
        uart = hal.UARTAdapter(1, 115200)
        for data in (b"x" * 257, "text"):
            uart.device.rx = [data]
            with self.assertRaises(ValueError):
                uart.any()

    def test_camera_crops_without_resizing_and_does_not_reuse_failed_capture(self):
        events = []
        cropped = types.SimpleNamespace(width=lambda: 320, height=lambda: 120,
                     format=lambda: 1, bytearray=lambda: bytearray(38400))
        full = types.SimpleNamespace(width=lambda: 320, height=lambda: 240,
                                    format=lambda: 1,
                                    copy=lambda **kw: events.append(kw) or cropped)
        camera = hal.Camera()
        camera.GRAYSCALE = 1
        camera.device = types.SimpleNamespace(snapshot=lambda: full)
        self.assertEqual(len(camera.snapshot().bytearray()), 38400)
        self.assertEqual(events, [dict(roi=(0, 60, 320, 120))])
        camera.device.snapshot = lambda: None
        with self.assertRaises(RuntimeError):
            camera.snapshot()
        self.assertIsNone(camera.latest)

    def test_lcd_preview_reuses_scaled_buffer_and_ide_keeps_raw_frame(self):
        raw = object()
        display = types.SimpleNamespace(shown=[])
        display.show_image = lambda frame, **kwargs: display.shown.append(
            (frame, kwargs))
        camera = hal.Camera()
        camera.latest = types.SimpleNamespace(image=raw)
        camera.display = display

        image_module = types.SimpleNamespace(
            Image=PreviewImage, GRAYSCALE=PreviewImage.GRAYSCALE)
        with patch.dict(sys.modules, {"image": image_module}), \
                patch.object(hal.settings, "DISPLAY_MODE", "lcd"), \
                patch.object(hal.settings, "DISPLAY_WIDTH", 640), \
                patch.object(hal.settings, "DISPLAY_HEIGHT", 480):
            camera.flush()
            camera.flush()

            self.assertEqual(len(PreviewImage.created), 1)
            preview = PreviewImage.created[0]
            self.assertEqual((preview.width_value, preview.height_value),
                             (640, 240))
            self.assertEqual(len(preview.draws), 2)
            self.assertIs(preview.draws[0][0], raw)
            self.assertEqual(preview.draws[0][1:3], (0, 0))
            self.assertEqual(preview.draws[0][3],
                             {"x_scale": 2.0, "y_scale": 2.0})
            self.assertIs(display.shown[0][0], preview)
            self.assertEqual(display.shown[0][1], {"x": 0, "y": 120})
            self.assertIs(display.shown[1][0], preview)

        display.shown[:] = []
        with patch.object(hal.settings, "DISPLAY_MODE", "ide"):
            camera.flush()
        self.assertIs(display.shown[0][0], raw)
        self.assertEqual(display.shown[0][1], {"x": 0, "y": 60})

        camera.close()
        self.assertIsNone(camera._lcd_preview)

    def test_rgb_or_wrong_frame_shape_is_rejected(self):
        camera = hal.Camera()
        camera.GRAYSCALE = 1
        for width, fmt in ((320, 2), (640, 1)):
            camera.device = types.SimpleNamespace(snapshot=lambda: types.SimpleNamespace(
                width=lambda: width, height=lambda: 240, format=lambda: fmt))
            with self.assertRaises(RuntimeError):
                camera.snapshot()

    def test_resource_cleanup_continues_after_sensor_stop_error(self):
        camera = hal.Camera()
        events = []
        def broken():
            events.append("sensor")
            raise OSError("stop")
        camera.device = types.SimpleNamespace(stop=broken)
        camera.display = types.SimpleNamespace(deinit=lambda: events.append("display"))
        camera.media = types.SimpleNamespace(deinit=lambda: events.append("media"))
        camera.display_started = camera.media_started = True
        camera.close()
        self.assertEqual(events, ["sensor", "display", "media"])
        self.assertIsNone(camera.device)


class CalibrationBindingTests(unittest.TestCase):
    def setUp(self):
        self.old = calibration.SENSOR_ID, calibration.FORMAT_VERSION, calibration._crc_payload, calibration.RingWidthCalibration

    def tearDown(self):
        calibration.SENSOR_ID, calibration.FORMAT_VERSION, calibration._crc_payload, calibration.RingWidthCalibration = self.old

    def test_h750_file_is_not_accepted_by_changing_a_label(self):
        binding.bind(24, "board-a|mount-1")
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cal.json"
            p.write_text(json.dumps(dict(format_version=2, sensor_id=0x1324)))
            result = calibration.RingWidthCalibration.load(str(p))
            self.assertFalse(result.ready)
            self.assertEqual(result.reason, "GEOMETRY_MISMATCH")

    def test_new_calibration_roundtrip_and_mount_sensor_crc_rejection(self):
        binding.bind(24, "board-a|mount-1")
        collector = calibration.RingWidthCalibration()
        rows = [v for y in range(8, 60) for v in (y, 25, 54, 7)]
        for _ in range(66):
            collector.accept_rows(rows, 52, 4)
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cal.json"
            data = collector.save(str(p))
            self.assertEqual(data["format_version"], 3)
            self.assertEqual(data["sensor_id"], 24)
            self.assertTrue(calibration.RingWidthCalibration.load(str(p)).ready)
            binding.bind(24, "board-a|mount-2")
            self.assertFalse(calibration.RingWidthCalibration.load(str(p)).ready)
            binding.bind(25, "board-a|mount-1")
            self.assertFalse(calibration.RingWidthCalibration.load(str(p)).ready)
            binding.bind(24, "board-a|mount-1")
            data["width_cells"][20] += 1
            p.write_text(json.dumps(data))
            self.assertEqual(calibration.RingWidthCalibration.load(str(p)).reason, "CRC_MISMATCH")


if __name__ == "__main__":
    unittest.main()

