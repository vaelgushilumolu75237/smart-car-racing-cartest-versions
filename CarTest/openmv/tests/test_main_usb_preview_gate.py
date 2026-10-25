"""Exercise USB preview gating through the production _main_fast loop."""

import contextlib
import io
import unittest
from unittest.mock import patch

import test_fast_main_ring_slope as fixtures


main = fixtures.main
config = fixtures.config


class _DrawingImage(fixtures._Image):
    def __init__(self):
        super().__init__()
        self.draw_calls = []

    def __getattr__(self, name):
        if not name.startswith("draw_"):
            raise AttributeError(name)

        def record(*_args, **_kwargs):
            self.draw_calls.append(name)

        return record


class _RecordingSensor:
    def __init__(self, image=None):
        self.image = image if image is not None else _DrawingImage()
        self.snapshot_calls = 0
        self.flush_calls = 0

    @staticmethod
    def get_exposure_us():
        return 5000

    @staticmethod
    def get_gain_db():
        return 0.0

    def snapshot(self):
        self.snapshot_calls += 1
        return self.image

    def flush(self):
        self.flush_calls += 1


class _USBVCP:
    def __init__(self, connected):
        self.connected = connected

    def isconnected(self):
        return self.connected


class _FixtureSource:
    name = "RAM_FIXTURE"

    def __init__(self, image, usb_attribute=None, usb_vcp=None):
        self.image = image
        self.snapshot_calls = 0
        self.flush_calls = 0
        if usb_attribute is not None:
            setattr(self, usb_attribute, usb_vcp)

    def snapshot(self):
        self.snapshot_calls += 1
        return self.image

    def flush(self):
        self.flush_calls += 1


class MainUSBPreviewGateTests(unittest.TestCase):
    def _run(self, *, connected=False, usb_preview_enabled=True,
             frame_source=None):
        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()

        fake_time = fixtures._Time()
        sensor = _RecordingSensor()
        uart = fixtures._UART(fake_time)
        usb_vcp = _USBVCP(connected)
        build_calls = []

        class RecordingGrayBuilder(fixtures._GrayBuilder):
            def build(self, image):
                build_calls.append(image)
                return super().build(image)

        output = io.StringIO()
        patches = [
            patch.object(main, "sensor", sensor),
            patch.object(main, "time", fake_time),
            patch.object(main, "_native_preview_control_api_available",
                         return_value=False),
            patch.object(main, "_configure_gray_sensor", return_value=1),
            patch.object(main, "_service_reverse_control",
                         return_value=True),
            patch.object(main.fast_gray, "FastGrayMasks",
                         RecordingGrayBuilder),
            patch.object(main.fast_path, "FastPathDetector",
                         fixtures._PathDetector),
            patch.object(main.fast_boundary, "FastBoundaryEvidence",
                         fixtures._BoundaryValidator),
            patch.object(main.fast_single_edge, "FastSingleEdgeTracker",
                         fixtures._SingleEdgeTracker),
            patch.object(main.pyb, "UART", return_value=uart, create=True),
            patch.object(main, "_validate_selected_fast_boundary",
                         side_effect=lambda selection, _road, _validator: (
                             selection,
                             {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                             {"valid": True, "reason": "TEST_VALID"})),
            patch.object(config, "DEBUG_ENABLED", True),
            patch.object(config, "OVERLAY_ENABLED", True),
            patch.object(config, "USB_PREVIEW_ENABLED",
                         usb_preview_enabled),
            patch.object(main, "_open_usb_vcp_safely",
                         return_value=usb_vcp),
        ]
        with contextlib.ExitStack() as stack:
            opened_usb = stack.enter_context(patches[-1])
            for patcher in patches[:-1]:
                stack.enter_context(patcher)
            with contextlib.redirect_stdout(output):
                if frame_source is None:
                    main._main_fast(duration_s=1)
                else:
                    main._main_fast(duration_s=1,
                                    frame_source=frame_source,
                                    uart_sink=uart)
        return sensor, uart, output.getvalue(), build_calls, opened_usb

    def test_stock_non_native_camera_draws_when_usb_cdc_is_connected(self):
        sensor, uart, output, build_calls, opened_usb = self._run(
            connected=True)

        self.assertEqual(opened_usb.call_count, 1)
        self.assertEqual(sensor.snapshot_calls, 1)
        self.assertEqual(len(build_calls), 1)
        self.assertGreater(len(sensor.image.draw_calls), 0)
        self.assertEqual(sensor.flush_calls, 1)
        self.assertEqual(len(uart.frames), 1)
        self.assertIn("V7DBG ", output)

    def test_stock_headless_and_preview_disabled_keep_the_frame_pipeline(self):
        for enabled, connected in ((True, False), (False, True)):
            with self.subTest(usb_preview_enabled=enabled,
                              usb_connected=connected):
                sensor, uart, output, build_calls, opened_usb = self._run(
                    connected=connected,
                    usb_preview_enabled=enabled)

                self.assertEqual(opened_usb.call_count, 1 if enabled else 0)
                self.assertEqual(sensor.snapshot_calls, 1)
                self.assertEqual(len(build_calls), 1)
                self.assertEqual(sensor.image.draw_calls, [])
                self.assertEqual(sensor.flush_calls, 1)
                self.assertEqual(len(uart.frames), 1)
                self.assertIn("V7DBG ", output)

    def test_explicit_fixture_usb_state_gates_drawing_but_still_flushes(self):
        cases = (("usb_vcp", _USBVCP(False), False),
                 ("_usb_vcp", None, False),
                 ("usb_vcp", _USBVCP(True), True))
        for attribute, usb_vcp, should_draw in cases:
            with self.subTest(attribute=attribute,
                              usb_connected=(usb_vcp is not None and
                                             usb_vcp.isconnected())):
                image = _DrawingImage()
                source = _FixtureSource(image, attribute, usb_vcp)
                _sensor, uart, output, build_calls, opened_usb = self._run(
                    frame_source=source)

                self.assertEqual(opened_usb.call_count, 0)
                self.assertEqual(source.snapshot_calls, 1)
                self.assertEqual(len(build_calls), 1)
                self.assertEqual(bool(image.draw_calls), should_draw)
                self.assertEqual(source.flush_calls, 1)
                self.assertEqual(len(uart.frames), 1)
                self.assertIn("V7DBG ", output)

    def test_host_fixture_without_usb_handle_keeps_legacy_overlay_behavior(self):
        image = _DrawingImage()
        source = _FixtureSource(image)

        _sensor, uart, output, build_calls, _opened_usb = self._run(
            frame_source=source)

        self.assertEqual(source.snapshot_calls, 1)
        self.assertEqual(len(build_calls), 1)
        self.assertGreater(len(image.draw_calls), 0)
        self.assertEqual(source.flush_calls, 1)
        self.assertEqual(len(uart.frames), 1)
        self.assertIn("V7DBG ", output)


if __name__ == "__main__":
    unittest.main()
