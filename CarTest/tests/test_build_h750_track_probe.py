from __future__ import annotations

import ast
from array import array
from contextlib import redirect_stderr
import hashlib
import io
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest

from tools import build_h750_track_probe as builder
from openmv import reverse_control, vision_protocol


class _FakeTime:
    def __init__(self, now=0):
        self.now = now & 0xFFFF

    def ticks_ms(self):
        return self.now

    @staticmethod
    def ticks_add(value, delta):
        return (value + delta) & 0xFFFF

    @staticmethod
    def ticks_diff(value, before):
        return ((value - before + 0x8000) & 0xFFFF) - 0x8000


class _FakeMicroPython:
    @staticmethod
    def native(function):
        return function

    @staticmethod
    def viper(function):
        return function


class _FakeVisionUARTWriter:
    FIFO_QUEUED = "FIFO_QUEUED"
    SYNCHRONOUS_FALLBACK = "SYNCHRONOUS_FALLBACK"

    def __init__(self, uart):
        self.uart = uart
        if (getattr(uart, "WRITE_FIFO_API", None) == 1 and
                callable(getattr(uart, "write_fifo", None)) and
                callable(getattr(uart, "fifo_txdone", None))):
            self.mode = self.FIFO_QUEUED
        else:
            self.mode = self.SYNCHRONOUS_FALLBACK

    def write(self, packet):
        if self.mode == self.FIFO_QUEUED:
            return self.uart.write_fifo(packet)
        return self.uart.write(packet)

    def fifo_txdone(self):
        if self.mode != self.FIFO_QUEUED:
            return None
        return self.uart.fifo_txdone()


class _FakePhysicalUART:
    def __init__(self, incoming=b"", fifo=False, txdone=True):
        self.incoming = bytearray(incoming)
        self.writes = []
        self.WRITE_FIFO_API = 1 if fifo else None
        self.txdone = txdone
        self.txdone_checks = 0

    def any(self):
        return len(self.incoming)

    def readinto(self, destination, requested=None):
        count = min(len(destination), len(self.incoming))
        if requested is not None:
            count = min(count, requested)
        for index in range(count):
            destination[index] = self.incoming[index]
        del self.incoming[:count]
        return count

    def write(self, packet):
        self.writes.append(bytes(packet))
        return len(packet)

    def write_fifo(self, packet):
        self.writes.append(bytes(packet))
        return len(packet)

    def fifo_txdone(self):
        self.txdone_checks += 1
        return self.txdone


class _FakePerf:
    def __init__(self, stages):
        self.bins = {name: array("I", [0] * 202) for name in stages}
        self.sums = {name: 0 for name in stages}
        self.maxima = {name: 0 for name in stages}
        self.counts = {name: 0 for name in stages}
        self.frames = 10
        self.write_attempts = 10
        self.full_writes = 10
        self.valid_frames = 4
        self.valid_full_writes = 4
        self.late_invalid_frames = 0
        self.write_errors = 0
        self.last_sequence = 9
        self.heap_samples = 2
        self.heap_free_min = 1000
        self.heap_alloc_max = 2000
        self.heap_free_last = 1200
        self.heap_alloc_last = 1800

        for name in stages:
            self.counts[name] = 10
            self.sums[name] = 12345
            self.maxima[name] = 1900
            self.bins[name][1] = 9
            self.bins[name][2] = 1


class BuildH750TrackProbeTests(unittest.TestCase):
    def _helper_namespace(self):
        crc_table = array("H", (
            0x0000, 0x1021, 0x2042, 0x3063,
            0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B,
            0xC18C, 0xD1AD, 0xE1CE, 0xF1EF))
        fake_main = types.SimpleNamespace(
            _SteeringDiagBuffer=types.SimpleNamespace(
                _CRC16_NIBBLE_TABLE_ARRAY=crc_table),
            _VisionUARTWriter=_FakeVisionUARTWriter)
        namespace = {
            "array": array,
            "micropython": _FakeMicroPython,
            "main": fake_main,
            "ptr8": lambda value: value,
            "ptr16": lambda value: value,
        }
        exec(builder.CALIBRATION_PROOF_HELPER, namespace)
        exec(builder.WIRE_HELPERS, namespace)
        exec(builder.SYNTHETIC_REVERSE_HELPER, namespace)
        exec(builder.SAFE_UART_HELPER, namespace)
        exec(builder.render_window_helpers(), namespace)
        return namespace

    def test_generated_source_is_small_explicit_and_real_sensor_ordered(self):
        camera_root = "/flash/track_trial_01"
        source = builder.build_probe_source(
            profile="B", camera_root=camera_root)
        compile(source, "h750-track-probe-B.py", "exec")
        self.assertEqual(len(builder.RUNTIME_NAMES), 23)
        self.assertEqual(builder.EXPECTED_RUNTIME_MODULE_COUNT, 23)
        self.assertTrue({
            "ring_width_calibration.py",
            "ring_diagnostics.py",
            "fast_ring_slope.py",
        }.issubset(set(builder.RUNTIME_NAMES)))
        self.assertIn("_PROFILE_ID = 2", source)
        self.assertIn("_PROFILE_ID = __PROFILE_ID__", builder.PROBE_BODY)
        self.assertIn("_CAMERA_ROOT = '/flash/track_trial_01'", source)
        self.assertIn("import pyb, sensor, uhashlib", source)
        self.assertIn("import pyb, sensor, uhashlib, micropython", source)
        self.assertEqual(source.count("@micropython.native"), 4)
        self.assertIn("handle.read(1024)", source)
        self.assertIn("hasher.update(block)", source)
        self.assertIn("module_path != expected_prefix + filename", source)
        self.assertNotIn("__main__", source)
        self.assertIn("production_valid_control=0", source)
        self.assertIn("profile_sequence=10parked_then_active", source)
        self.assertIn("fixture_sensor_id=_probe_sensor_id", source)
        self.assertIn("_verify_calibration_probe_proof(_stats, _STAT_INDEX)",
                      source)
        self.assertIn("PARKED_PROFILE_STATUS_FRAMES = 10", source)
        self.assertIn("synthetic_inactive_profile_frames", source)
        self.assertIn("synthetic_active_profile_frames", source)
        self.assertIn("landed ring calibration not ready", source)
        self.assertIn("preview_source=native marker=gray24crc8_opt_in", source)
        self.assertIn("main._NativeUSBPreviewSource.snapshot(self)", source)
        self.assertIn("main._NativeUSBPreviewSource.flush(self)", source)
        self.assertIn("main._is_native_usb_preview_source(_probe_source)",
                      source)
        self.assertIn("sensor.snapshot(preview=False)",
                      (builder.OPENMV_ROOT / "main.py").read_text(
                          encoding="utf-8"))
        self.assertIn("config.PREVIEW_FPS_MARKER_ENABLED = True", source)
        self.assertIn("_original_marker_setting", source)
        self.assertIn("native_preview_active_frames", source)
        self.assertIn("native_preview_headless_frames", source)
        self.assertIn("main._VisionUARTWriter(physical_uart)", source)
        self.assertIn('_UART_MODE = "production"', source)
        self.assertIn("def finish(self):", source)
        self.assertIn("@micropython.viper", builder.WIRE_HELPERS)
        self.assertIn("main._SteeringDiagBuffer._CRC16_NIBBLE_TABLE_ARRAY",
                      builder.WIRE_HELPERS)
        self.assertIn("time.sleep_ms(1000)", source)
        self.assertIn("time.sleep_ms(50)", source)
        self.assertNotIn("self.sensor.flush()", source)
        self.assertNotIn("image = self.sensor.snapshot()", source)
        self.assertNotIn("_ram_sources", source)
        self.assertNotIn("deflate.DeflateIO", source)

        tree = ast.parse(source)
        configure_lines = [
            node.lineno for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "main"
            and node.func.attr == "_configure_gray_sensor"]
        run_lines = [
            node.lineno for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "main"
            and node.func.attr == "_main_fast"]
        self.assertEqual(len(configure_lines), 1)
        self.assertEqual(len(run_lines), 1)
        self.assertLess(configure_lines[0], run_lines[0])
        self.assertIn("duration_s=_RUN_SECONDS", source)
        self.assertIn("_RUN_SECONDS = 35", source)
        path_insert = source.index("sys.path.insert(0, _CAMERA_ROOT)")
        hash_check = source.index("_verify_camera_runtime_hashes()\n")
        first_module_import = source.index(
            "import config, main, reverse_control, vision_protocol")
        path_check = source.index("_verify_loaded_module_paths()\n")
        self.assertLess(path_insert, hash_check)
        self.assertLess(hash_check, first_module_import)
        self.assertLess(first_module_import, path_check)

    def test_sync_uart_source_is_explicit_and_unknown_mode_is_rejected(self):
        sync_source = builder.build_probe_source(
            profile="B", camera_root="/flash/track_trial_01",
            uart_mode="synchronous")
        compile(sync_source, "h750-track-probe-B-sync.py", "exec")
        self.assertIn('_UART_MODE = "synchronous"', sync_source)
        self.assertIn("self.physical_uart.write(self.wire_buffer)",
                      sync_source)
        with self.assertRaisesRegex(ValueError, "uart_mode"):
            builder.build_probe_source(
                profile="B", camera_root="/flash/track_trial_01",
                uart_mode="raw")

        bootstrap_tree = ast.parse(builder.RUNTIME_IMPORT_BOOTSTRAP)
        bootstrap_imports = [
            alias.name for node in bootstrap_tree.body
            if isinstance(node, ast.Import) for alias in node.names]
        self.assertEqual(bootstrap_imports,
                         ["config", "main", "reverse_control",
                          "vision_protocol"])
        bootstrap_calls = [node for node in ast.walk(bootstrap_tree)
                           if isinstance(node, ast.Call)]
        self.assertFalse(any(isinstance(node.func, ast.Name) and
                             node.func.id == "__import__"
                             for node in bootstrap_calls))
        validate_call = builder.RUNTIME_IMPORT_BOOTSTRAP.index(
            "main._validate_fast_config()")
        loaded_paths_call = builder.RUNTIME_IMPORT_BOOTSTRAP.index(
            "_verify_loaded_module_paths()")
        self.assertLess(validate_call, loaded_paths_call)

    def test_visible_marker_is_gray24_lsb_first_with_crc8_atm(self):
        main_tree = ast.parse(
            (builder.OPENMV_ROOT / "main.py").read_text(encoding="utf-8"))
        wanted = {
            "_preview_marker_crc8", "_draw_preview_marker_pixels_viper",
            "_preview_marker_image_buffer", "_draw_preview_frame_marker",
        }
        definitions = [
            node for node in main_tree.body
            if isinstance(node, ast.FunctionDef) and node.name in wanted]
        self.assertEqual({node.name for node in definitions}, wanted)

        class FakeImage:
            def __init__(self):
                self.pixels = bytearray([37] * (320 * 120))

            @staticmethod
            def format():
                return "GRAYSCALE"

            @staticmethod
            def width():
                return 320

            @staticmethod
            def height():
                return 120

            def bytearray(self):
                return self.pixels

        namespace = {
            "micropython": _FakeMicroPython,
            "sensor": types.SimpleNamespace(GRAYSCALE="GRAYSCALE"),
            "ptr8": lambda value: value,
            "config": types.SimpleNamespace(
                PREVIEW_FPS_MARKER_ENABLED=True),
        }
        module = ast.Module(body=definitions, type_ignores=[])
        exec(compile(module, "main_preview_marker.py", "exec"), namespace)
        frame_id = 0x123456
        image = FakeImage()
        original = bytes(image.pixels)
        self.assertTrue(namespace["_draw_preview_frame_marker"](
            image, frame_id))
        rectangles = [(4 + index * 5, 4, 4, 4) for index in range(32)]
        encoded_bits = []
        marker_offsets = set()
        for x, y, width, height in rectangles:
            cell_offsets = {
                (y + row) * 320 + x + column
                for row in range(height) for column in range(width)}
            marker_offsets.update(cell_offsets)
            cell_values = {image.pixels[offset] for offset in cell_offsets}
            self.assertEqual(len(cell_values), 1)
            value = cell_values.pop()
            self.assertIn(value, (0, 255))
            encoded_bits.append(1 if value == 255 else 0)
        self.assertEqual(len(marker_offsets), 32 * 4 * 4)
        self.assertEqual(
            {index for index, (before, after) in enumerate(
                zip(original, image.pixels)) if before != after},
            marker_offsets)

        gray = (frame_id ^ (frame_id >> 1)) & 0xFFFFFF
        expected_crc = 0
        for byte_index in range(3):
            expected_crc ^= (gray >> (byte_index * 8)) & 0xFF
            for _ in range(8):
                if expected_crc & 0x80:
                    expected_crc = ((expected_crc << 1) ^ 0x07) & 0xFF
                else:
                    expected_crc = (expected_crc << 1) & 0xFF
        decoded_gray = sum(bit << index
                           for index, bit in enumerate(encoded_bits[:24]))
        decoded_crc = sum(bit << index
                          for index, bit in enumerate(encoded_bits[24:]))
        self.assertEqual(decoded_gray, gray)
        self.assertEqual(decoded_crc, expected_crc)

        namespace["config"].PREVIEW_FPS_MARKER_ENABLED = False
        before_disabled = bytes(image.pixels)
        self.assertFalse(namespace["_draw_preview_frame_marker"](
            image, frame_id))
        self.assertEqual(bytes(image.pixels), before_disabled)

    def test_camera_root_validation_rejects_traversal_and_non_ascii(self):
        self.assertEqual(
            builder.validate_camera_root("/flash/track_trial_01"),
            "/flash/track_trial_01")
        for invalid in (
                "relative/track_trial_01", "/other/track_trial_01",
                "/flash/../F", "/flash/a/../../F", "/flash/a.b",
                "/flash/车道", "/flash\\track_trial_01", "/flash/"):
            with self.subTest(camera_root=invalid):
                with self.assertRaises(ValueError):
                    builder.validate_camera_root(invalid)
        with self.assertRaisesRegex(ValueError, "camera_root is required"):
            builder.build_probe_source(profile="B")

    def test_cli_requires_explicit_camera_root_and_rejects_traversal(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "probe.py"
            for argv in (
                    ["--profile", "B", "--output", str(output)],
                    ["--profile", "B", "--camera-root", "/flash/../F",
                     "--output", str(output)]):
                stderr = io.StringIO()
                with redirect_stderr(stderr):
                    with self.assertRaises(SystemExit) as raised:
                        builder.main(argv)
                self.assertEqual(raised.exception.code, 2)
                self.assertIn("camera-root", stderr.getvalue())
                self.assertFalse(output.exists())

    def test_generated_runtime_hash_and_loaded_module_path_checks(self):
        with tempfile.TemporaryDirectory() as temp:
            runtime = Path(temp)
            for name in builder.RUNTIME_NAMES:
                shutil.copyfile(builder.OPENMV_ROOT / name, runtime / name)
            expected_hashes = builder.verify_runtime_root(runtime)
            runtime_helpers = (builder.RUNTIME_IMPORT_HELPERS
                               .replace("__CAMERA_ROOT__", repr(str(runtime)))
                               .replace("__RUNTIME_NAMES__",
                                        repr(tuple(builder.RUNTIME_NAMES)))
                               .replace("__EXPECTED_RUNTIME_MODULE_COUNT__",
                                        str(builder.EXPECTED_RUNTIME_MODULE_COUNT))
                               .replace("__MODULE_HASHES__",
                                        repr(expected_hashes)))
            fake_sys = types.SimpleNamespace(modules={})
            namespace = {"sys": fake_sys, "uhashlib": hashlib}
            exec(runtime_helpers, namespace)
            namespace["_verify_camera_runtime_hashes"]()
            loaded_names = (
                "config.py", "main.py", "reverse_control.py",
                "vision_protocol.py", "fast_gray.py", "fast_path.py",
                "fast_single_edge.py", "track_elements.py")
            for filename in loaded_names:
                module_name = filename[:-3]
                fake_sys.modules[module_name] = types.SimpleNamespace(
                    __file__=str(runtime) + "/" + filename)
            namespace["_verify_loaded_module_paths"]()
            self.assertNotIn("road_detector", fake_sys.modules)
            self.assertNotIn("surface_detector", fake_sys.modules)
            fake_sys.modules["main"].__file__ = "/flash/F/main.py"
            with self.assertRaisesRegex(RuntimeError,
                                        "runtime module path mismatch: main"):
                namespace["_verify_loaded_module_paths"]()

            with (runtime / "main.py").open("ab") as changed:
                changed.write(b"# changed after staging\n")
            with self.assertRaisesRegex(RuntimeError,
                                        "runtime SHA256 mismatch: main.py"):
                namespace["_verify_camera_runtime_hashes"]()

    def test_runtime_directory_requires_exact_active_allowlist(self):
        with tempfile.TemporaryDirectory() as temp:
            runtime = Path(temp)
            for name in builder.RUNTIME_NAMES:
                shutil.copyfile(builder.OPENMV_ROOT / name, runtime / name)
            hashes = builder.verify_runtime_root(runtime)
            self.assertEqual(set(hashes), set(builder.RUNTIME_NAMES))
            self.assertEqual(
                hashes["main.py"],
                hashlib.sha256((runtime / "main.py").read_bytes()).hexdigest())

            (runtime / "unreviewed.py").write_text("pass\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exact 23-module allowlist"):
                builder.verify_runtime_root(runtime)
            (runtime / "unreviewed.py").unlink()
            (runtime / "track_elements.py").unlink()
            with self.assertRaises(FileNotFoundError):
                builder.verify_runtime_root(runtime)

    def test_invalid_wire_resequence_and_crc_are_independent_from_computed(self):
        namespace = self._helper_namespace()
        pack_invalid = namespace["_pack_invalid_v3"]
        classify = namespace["_computed_packet_flags"]
        crc16 = namespace["_crc16_ccitt_false"]

        computed_valid = vision_protocol.pack_frame_v3(
            0x1234, 1, 88, -27, 0, 2, 400, 2)
        computed_invalid = vision_protocol.pack_frame_v3(
            0x1235, 0, 0, 0, 0, 0, 0, 2)
        self.assertEqual(classify(computed_valid), (1, 1, 2))
        self.assertEqual(classify(computed_invalid), (1, 0, 2))

        wire_a = bytearray(16)
        wire_b = bytearray(16)
        pack_invalid(wire_a, 0xB000)
        pack_invalid(wire_b, 0xB001)
        self.assertEqual(len(wire_a), 16)
        self.assertEqual(wire_a[2:4], bytearray((3, 8)))
        self.assertEqual(wire_a[4:6], bytearray((0x00, 0xB0)))
        self.assertEqual(wire_a[6:14], bytearray(8))
        self.assertEqual(wire_a[11], 0)
        self.assertEqual(wire_a[6] & 1, 0)
        self.assertEqual(crc16(wire_a, 2, 14), wire_a[14] | (wire_a[15] << 8))
        self.assertEqual(wire_a, vision_protocol.pack_frame_v3(
            0xB000, 0, 0, 0, 0, 0, 0, 0))
        self.assertEqual(crc16(wire_a, 2, 14),
                         vision_protocol._crc16_ccitt_false_range(
                             wire_a, 2, 14))
        self.assertEqual((wire_b[4] | (wire_b[5] << 8)), 0xB001)
        self.assertNotEqual(wire_a[4:6], computed_valid[4:6])

        crc32 = namespace["_crc32_iso_hdlc"]
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)

    def test_synthetic_reverse_sends_ten_parked_states_then_active_b_session(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        indices = builder.STAT_INDEX
        fake_time = _FakeTime(0xFFF0)
        raw_rx = bytes(range(256)) * 3
        physical = _FakePhysicalUART(raw_rx)
        source = namespace["_SyntheticReverseInput"](
            physical, reverse_control, fake_time, stats, indices,
            reverse_control.PROFILE_B, sequence_seed=0xFFFE)
        parser = reverse_control.ReverseControlParser(250, 250)
        buffer = bytearray(reverse_control.MAX_FEED_BYTES)

        for profile_index in range(10):
            if profile_index:
                fake_time.now = (fake_time.now + 100) & 0xFFFF
            count = source.any()
            received = source.readinto(buffer, count)
            self.assertGreater(received, 0)
            parser.feed(buffer, received, fake_time.ticks_ms())
            self.assertFalse(parser.profile_session_active)
            self.assertEqual(stats[
                indices["synthetic_inactive_profile_frames"]],
                profile_index + 1)
            self.assertEqual(stats[
                indices["synthetic_active_profile_frames"]], 0)

        # The eleventh state begins the active lease only after the parked
        # prefix that the production calibration startup gate requires.
        fake_time.now = (fake_time.now + 100) & 0xFFFF
        count = source.any()
        received = source.readinto(buffer, count)
        self.assertGreater(received, 0)
        parser.feed(buffer, received, fake_time.ticks_ms())
        self.assertTrue(parser.profile_session_active)
        self.assertEqual(parser.session_token, 1)
        self.assertEqual(
            stats[indices["real_uart_rx_discarded_bytes"]], len(raw_rx))
        self.assertEqual(stats[indices["synthetic_reset_frames"]], 1)
        self.assertGreaterEqual(stats[indices["synthetic_keep_frames"]], 1)
        self.assertEqual(stats[indices["synthetic_profile_frames"]], 11)
        self.assertEqual(stats[indices["synthetic_inactive_profile_frames"]], 10)
        self.assertEqual(stats[indices["synthetic_active_profile_frames"]], 1)

    def test_calibration_probe_proof_requires_landed_map_parked_prefix_and_ack(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        indices = builder.STAT_INDEX
        stats[indices["sensor_identity_verified"]] = 1
        stats[indices["native_preview_source_verified"]] = 1
        stats[indices["preview_marker_enabled"]] = 1
        stats[indices["landed_calibration_ready"]] = 1
        stats[indices["synthetic_inactive_profile_frames"]] = 10
        stats[indices["synthetic_active_profile_frames"]] = 1
        stats[indices["computed_expected_ack_packets"]] = 1
        stats[indices["sensor_snapshots"]] = 2
        stats[indices["native_preview_active_frames"]] = 1
        stats[indices["native_preview_headless_frames"]] = 1
        self.assertTrue(namespace["_verify_calibration_probe_proof"](
            stats, indices))

        for name, message in (
                ("sensor_identity_verified", "sensor identity"),
                ("native_preview_source_verified", "native preview source"),
                ("preview_marker_enabled", "visible frame marker"),
                ("landed_calibration_ready", "not ready"),
                ("synthetic_inactive_profile_frames", "ten inactive"),
                ("synthetic_active_profile_frames", "active profile"),
                ("computed_expected_ack_packets", "startup check")):
            with self.subTest(name=name):
                rejected = array("I", stats)
                rejected[indices[name]] = 0
                with self.assertRaisesRegex(RuntimeError, message):
                    namespace["_verify_calibration_probe_proof"](
                        rejected, indices)

        incomplete = array("I", stats)
        incomplete[indices["native_preview_active_frames"]] = 0
        incomplete[indices["native_preview_headless_frames"]] = 0
        with self.assertRaisesRegex(RuntimeError, "state accounting"):
            namespace["_verify_calibration_probe_proof"](incomplete, indices)

    def test_real_source_inherits_native_preview_and_counts_active_headless(self):
        source = builder.build_probe_source(
            profile="B", camera_root="/flash/track_trial_01")
        tree = ast.parse(source)
        class_node = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and
            node.name == "_RealSensorSource")

        class FakeNativeSource:
            USB_PREVIEW_SOURCE_API = 1

            def __init__(self, sensor_module, usb_vcp, enabled, quality):
                self._sensor = sensor_module
                self._usb_vcp = usb_vcp
                self.enabled = enabled
                self.quality = quality
                self.frame_preview_active = False
                self.last_state = "NATIVE_HEADLESS"
                self.publish_failures = 0

            def snapshot(self):
                image = self._sensor.snapshot(preview=False)
                self.frame_preview_active = bool(
                    self.enabled and self._usb_vcp.isconnected())
                return image

            def flush(self):
                if self.frame_preview_active and self._usb_vcp.isconnected():
                    self.last_state = "NATIVE_ACTIVE"
                else:
                    self.last_state = "NATIVE_HEADLESS"
                self.clear()

            def clear(self):
                self.frame_preview_active = False

        class FakeSensor:
            def __init__(self):
                self.preview_arguments = []

            def snapshot(self, preview=True):
                self.preview_arguments.append(preview)
                return object()

        class FakeUSB:
            def __init__(self, connected):
                self.connected = connected

            def isconnected(self):
                return self.connected

        fake_main = types.SimpleNamespace(
            _NativeUSBPreviewSource=FakeNativeSource)
        stats = array("I", [0] * len(builder.STAT_NAMES))
        namespace = {
            "main": fake_main,
            "_stats": stats,
            "_STAT_INDEX": builder.STAT_INDEX,
        }
        isolated = ast.Module(body=[class_node], type_ignores=[])
        exec(compile(isolated, "probe_source_class.py", "exec"), namespace)
        source_type = namespace["_RealSensorSource"]
        self.assertEqual(source_type.USB_PREVIEW_SOURCE_API, 1)

        sensor = FakeSensor()
        active = source_type(sensor, FakeUSB(True), True, 10)
        active.snapshot()
        active.flush()
        self.assertEqual(active.last_state, "NATIVE_ACTIVE")
        self.assertEqual(sensor.preview_arguments, [False])
        self.assertEqual(stats[builder.STAT_INDEX["sensor_snapshots"]], 1)
        self.assertEqual(
            stats[builder.STAT_INDEX["native_preview_active_frames"]], 1)
        self.assertEqual(
            stats[builder.STAT_INDEX["native_preview_flush_calls"]], 1)
        self.assertEqual(
            stats[builder.STAT_INDEX["native_preview_publish_successes"]], 1)

        disconnected = source_type(sensor, FakeUSB(False), True, 10)
        disconnected.snapshot()
        disconnected.clear()
        self.assertEqual(
            stats[builder.STAT_INDEX["native_preview_headless_frames"]], 1)
        self.assertEqual(stats[builder.STAT_INDEX["sensor_snapshots"]], 2)

    def test_safe_uart_only_writes_invalid_ack0_and_tracks_computed_separately(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        physical = _FakePhysicalUART()
        wrapper = namespace["_SafeInvalidUART"](
            physical, None, 3, "production")
        old_index = builder.STAT_INDEX
        namespace["_stats"] = stats
        namespace["_STAT_INDEX"] = old_index
        valid = vision_protocol.pack_frame_v3(1, 1, 90, 12, 0, 2, 300, 3)
        invalid = vision_protocol.pack_frame_v3(2, 0, 0, 0, 0, 0, 0, 3)
        self.assertEqual(wrapper.write(valid), 16)
        self.assertEqual(wrapper.write(invalid), 16)
        self.assertEqual(len(physical.writes), 2)
        first, second = physical.writes
        for packet in physical.writes:
            self.assertEqual(len(packet), 16)
            self.assertEqual(packet[:4], bytes((0xA5, 0x5A, 3, 8)))
            self.assertEqual(packet[6:14], bytes(8))
            self.assertEqual(packet[11], 0)
            self.assertEqual(namespace["_crc16_ccitt_false"](
                packet, 2, 14), packet[14] | (packet[15] << 8))
        self.assertEqual(first[4:6], bytes((0x00, 0xB0)))
        self.assertEqual(second[4:6], bytes((0x01, 0xB0)))
        self.assertEqual(stats[builder.STAT_INDEX["computed_packets"]], 2)
        self.assertEqual(stats[builder.STAT_INDEX["computed_valid_flag_packets"]], 1)
        self.assertEqual(stats[builder.STAT_INDEX["computed_invalid_flag_packets"]], 1)
        # ACK observability is independent of the packet's valid flag; the
        # second computed packet is invalid but still carries the matching ACK.
        self.assertEqual(stats[builder.STAT_INDEX["computed_expected_ack_packets"]], 2)
        self.assertEqual(stats[builder.STAT_INDEX["invalid_wire_frames"]], 2)
        self.assertEqual(
            stats[builder.STAT_INDEX["invalid_wire_full_accepts"]], 2)
        self.assertEqual(
            stats[builder.STAT_INDEX["invalid_wire_sync_fallback_frames"]], 2)
        self.assertEqual(
            stats[builder.STAT_INDEX["invalid_wire_sync_baseline_frames"]], 0)
        self.assertEqual(stats[builder.STAT_INDEX["invalid_wire_forced_invalid_frames"]], 2)
        self.assertEqual(stats[builder.STAT_INDEX["invalid_wire_ack_zero_frames"]], 2)

    def test_production_fifo_path_counts_queue_acceptance_and_checks_txdone_once(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        physical = _FakePhysicalUART(fifo=True, txdone=True)
        namespace["_stats"] = stats
        namespace["_STAT_INDEX"] = builder.STAT_INDEX
        wrapper = namespace["_SafeInvalidUART"](
            physical, None, 3, "production")
        computed = vision_protocol.pack_frame_v3(
            0x1234, 1, 80, 7, 0, 0, 0, 3)
        self.assertEqual(wrapper.write(computed), 16)
        self.assertEqual(len(physical.writes), 1)
        self.assertEqual(
            stats[builder.STAT_INDEX["invalid_wire_fifo_queued_frames"]], 1)
        self.assertEqual(
            stats[builder.STAT_INDEX["invalid_wire_full_accepts"]], 1)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_txdone_checks"]], 0)
        self.assertIs(wrapper.finish(), True)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_txdone_checks"]], 1)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_txdone_true"]], 1)
        self.assertIs(wrapper.finish(), True)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_txdone_checks"]], 1)
        self.assertEqual(physical.txdone_checks, 1)

    def test_synchronous_mode_uses_direct_uart_baseline_without_fifo_check(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        physical = _FakePhysicalUART(fifo=True)
        namespace["_stats"] = stats
        namespace["_STAT_INDEX"] = builder.STAT_INDEX
        wrapper = namespace["_SafeInvalidUART"](
            physical, None, 3, "synchronous")
        computed = vision_protocol.pack_frame_v3(
            0x1234, 1, 80, 7, 0, 0, 0, 3)
        self.assertEqual(wrapper.write(computed), 16)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_sync_baseline_frames"]], 1)
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_queued_frames"]], 0)
        self.assertIsNone(wrapper.finish())
        self.assertEqual(stats[builder.STAT_INDEX[
            "invalid_wire_fifo_txdone_checks"]], 0)

    def test_window_table_caps_at_four_rows_and_report_crc_covers_all_stats(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        stages = ("capture", "ring")
        table = namespace["_BoundedWindowTable"](
            stages, stats, builder.STAT_INDEX)
        perf = _FakePerf(stages)
        stats[builder.STAT_INDEX["sensor_snapshots"]] = 101
        stats[builder.STAT_INDEX["invalid_wire_frames"]] = 99
        stats[builder.STAT_INDEX["native_preview_active_frames"]] = 10
        self.assertTrue(table.record(perf, 0, 10))
        stats[builder.STAT_INDEX["sensor_snapshots"]] = 201
        stats[builder.STAT_INDEX["invalid_wire_frames"]] = 199
        stats[builder.STAT_INDEX["native_preview_active_frames"]] = 30
        self.assertTrue(table.record(perf, 1, 20))
        self.assertTrue(table.record(perf, 3, 5))
        self.assertFalse(table.record(perf, 1, 20))
        self.assertFalse(table.record(perf, 4, 30))
        self.assertEqual(table.count, 3)
        self.assertEqual(table.duplicates, 1)
        self.assertEqual(table.overflow, 1)
        self.assertEqual(table.rows[table.stride + len(table.BASE_NAMES)], 100)
        self.assertEqual(table.rows[table.stride + len(table.BASE_NAMES) + 1], 100)
        active_extra = table.EXTRA_NAMES.index(
            "native_preview_active_frames")
        self.assertEqual(table.rows[
            table.stride + len(table.BASE_NAMES) + active_extra], 20)

        lines = namespace["_build_report_lines"](
            "B", 2, stats, builder.STAT_NAMES, table, 10000,
            builder.EXPECTED_RUNTIME_MODULE_COUNT,
            "none", True)
        self.assertTrue(lines[0].startswith("H750_TRACK_PROBE scope="))
        self.assertIn("modules=23", lines[0])
        self.assertTrue(any("computed_valid_flag_packets=0" in line
                            for line in lines))
        self.assertIn("production_valid_control=0", lines[0])
        self.assertIn("uart_mode=production", lines[0])
        self.assertIn("fifo_txdone=not_checked", lines[0])
        payload = ("\n".join(lines) + "\n").encode("utf-8")
        checksum = namespace["_crc32_iso_hdlc"](payload)
        self.assertEqual(checksum, namespace["_crc32_iso_hdlc"](
            ("\n".join(lines) + "\n").encode("utf-8")))
        self.assertNotEqual(checksum, namespace["_crc32_iso_hdlc"](
            ("\n".join(lines[:-1]) + "\n").encode("utf-8")))
        last_window = next(line for line in lines
                           if line.startswith("H750_TRACK_PROBE_WINDOW index=3"))
        self.assertIn("end_ms=35000", last_window)

    def test_worst_case_report_lines_and_total_output_are_bounded(self):
        namespace = self._helper_namespace()
        stats = array("I", [0xFFFFFFFF] * len(builder.STAT_NAMES))
        main_tree = ast.parse(
            (builder.OPENMV_ROOT / "main.py").read_text(encoding="utf-8"))
        stage_assignment = next(
            node for node in main_tree.body
            if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and
                target.id == "_V7_PERF_STAGES"
                for target in node.targets))
        stages = tuple(ast.literal_eval(stage_assignment.value))
        self.assertEqual(len(stages), 18)
        self.assertIn("ring_diag", stages)
        table = namespace["_BoundedWindowTable"](
            stages, stats, builder.STAT_INDEX)
        table.present = bytearray((1, 1, 1, 1))
        table.rows = array("I", [0xFFFFFFFF] * (4 * table.stride))
        lines = namespace["_build_report_lines"](
            "C", 3, stats, builder.STAT_NAMES, table, 10000,
            builder.EXPECTED_RUNTIME_MODULE_COUNT,
            "界" * 96, True)

        line_sizes = [len(line.encode("utf-8")) for line in lines]
        largest_line = max(line_sizes)
        max_lines = (1 + (len(builder.STAT_NAMES) +
                          builder.REPORT_STATS_PER_LINE - 1) //
                     builder.REPORT_STATS_PER_LINE + 1 +
                     builder.REPORT_MAX_WINDOWS * (1 + len(stages)))
        self.assertEqual(max_lines, 84)
        self.assertLessEqual(len(lines), max_lines)
        self.assertLessEqual(largest_line, builder.REPORT_LINE_MAX_UTF8_BYTES)
        self.assertEqual(sum(line.startswith(
            "H750_TRACK_PROBE_WINDOW ") for line in lines), 4)
        self.assertEqual(sum(line.startswith(
            "H750_TRACK_PROBE_STAGE ") for line in lines), 4 * len(stages))

        payload = ("\n".join(lines) + "\n").encode("utf-8")
        checksum = namespace["_crc32_iso_hdlc"](payload)
        crc_tail = ("H750_TRACK_PROBE_CRC32=%08X payload_bytes=%d "
                    "coverage=all_report_lines_before_crc" % (
                        checksum, len(payload)))
        self.assertLessEqual(len(crc_tail.encode("utf-8")),
                             builder.REPORT_CRC_TAIL_MAX_UTF8_BYTES)
        # Four added ring_diag stage rows raise the structural count to 84,
        # while preserving the former total-byte ceiling based on 80 lines.
        output_budget_lines = 80
        max_console_bytes = (output_budget_lines *
                             (builder.REPORT_LINE_MAX_UTF8_BYTES + 2) +
                             builder.REPORT_CRC_TAIL_MAX_UTF8_BYTES + 2)
        actual_console_bytes = (sum(size + 2 for size in line_sizes) +
                                len(crc_tail.encode("utf-8")) + 2)
        self.assertLessEqual(actual_console_bytes, max_console_bytes)
        self.assertLessEqual(
            len(payload), output_budget_lines *
            (builder.REPORT_LINE_MAX_UTF8_BYTES + 1))
        with self.assertRaisesRegex(RuntimeError, "850 UTF-8 bytes"):
            namespace["_validate_report_lines"](
                ["x" * (builder.REPORT_LINE_MAX_UTF8_BYTES + 1)], 0, 0)

    def test_cli_generates_only_requested_profile_and_compileable_runner(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "probe-c.py"
            status = builder.main([
                "--profile", "C", "--camera-root", "/flash/track_trial_01",
                "--output", str(output),
                "--runtime-root", str(builder.OPENMV_ROOT)])
            self.assertEqual(status, 0)
            source = output.read_text(encoding="utf-8")
            compile(source, str(output), "exec")
            self.assertIn("_PROFILE_ID = 3", source)
            self.assertIn("_PROFILE_LABEL = \"C\"", source)
            self.assertEqual(
                hashlib.sha256(output.read_bytes()).hexdigest(),
                hashlib.sha256(source.encode("utf-8")).hexdigest())


if __name__ == "__main__":
    unittest.main()
