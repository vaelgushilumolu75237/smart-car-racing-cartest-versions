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


class _FakePhysicalUART:
    def __init__(self, incoming=b""):
        self.incoming = bytearray(incoming)
        self.writes = []

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
        namespace = {"array": array, "micropython": _FakeMicroPython}
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
        self.assertEqual(len(builder.RUNTIME_NAMES), 20)
        self.assertNotIn("fast_ring_slope.py", builder.RUNTIME_NAMES)
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
        self.assertIn("sensor.snapshot()", source)
        self.assertIn("self.sensor.flush()", source)
        self.assertIn("self.physical_uart.write(self.wire_buffer)", source)
        self.assertIn("time.sleep_ms(1000)", source)
        self.assertIn("time.sleep_ms(50)", source)
        self.assertNotIn("snapshot(preview=False)", source)
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
            with self.assertRaisesRegex(ValueError, "exact 20-module allowlist"):
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
        self.assertEqual((wire_b[4] | (wire_b[5] << 8)), 0xB001)
        self.assertNotEqual(wire_a[4:6], computed_valid[4:6])

        crc32 = namespace["_crc32_iso_hdlc"]
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)

    def test_synthetic_reverse_is_independent_and_drives_fresh_b_session(self):
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

        count = source.any()
        received = source.readinto(buffer, count)
        self.assertEqual(received, 28)
        self.assertEqual(len(physical.incoming), len(raw_rx) - 256)
        self.assertEqual(
            stats[indices["real_uart_rx_discarded_bytes"]],
            256)
        parser.feed(buffer, received, fake_time.ticks_ms())
        self.assertEqual(parser.accepted_resets, 1)
        self.assertEqual(parser.accepted_profile_states, 1)
        self.assertFalse(parser.single_enabled)
        self.assertTrue(parser.profile_session_active)
        self.assertEqual(parser.profile_id, reverse_control.PROFILE_B)
        self.assertEqual(parser.session_token, 1)
        self.assertEqual(parser.mean_forward_wheel_millirev, 0)

        fake_time.now = (fake_time.now + 50) & 0xFFFF
        count = source.any()
        received = source.readinto(buffer, count)
        parser.feed(buffer, received, fake_time.ticks_ms())
        self.assertEqual(parser.accepted_keeps, 1)
        self.assertTrue(parser.single_enabled)
        self.assertEqual(parser.accepted_profile_states, 1)

        fake_time.now = (fake_time.now + 50) & 0xFFFF
        count = source.any()
        received = source.readinto(buffer, count)
        self.assertEqual(received, 28)
        parser.feed(buffer, received, fake_time.ticks_ms())
        self.assertEqual(parser.accepted_keeps, 2)
        self.assertEqual(parser.accepted_profile_states, 2)
        self.assertEqual(parser._profile_last_sequence, 0xFFFF)
        self.assertEqual(parser.mean_forward_wheel_millirev, 0)

        # Repeated parser polls discard the remaining physical bytes without
        # ever presenting them as synthetic reverse traffic.
        while physical.incoming:
            count = source.any()
            if count:
                source.readinto(buffer, count)
        self.assertEqual(physical.incoming, bytearray())
        self.assertEqual(
            stats[indices["real_uart_rx_discarded_bytes"]], len(raw_rx))

        fake_time.now = (fake_time.now + 100) & 0xFFFF
        count = source.any()
        received = source.readinto(buffer, count)
        parser.feed(buffer, received, fake_time.ticks_ms())
        self.assertEqual(parser._profile_last_sequence, 0)
        self.assertEqual(parser.profile_ack(fake_time.ticks_ms(),
                                            fake_time.ticks_diff),
                         reverse_control.PROFILE_B)
        self.assertEqual(stats[indices["synthetic_reset_frames"]], 1)
        self.assertGreaterEqual(stats[indices["synthetic_keep_frames"]], 2)
        self.assertEqual(stats[indices["synthetic_profile_frames"]], 3)

    def test_safe_uart_only_writes_invalid_ack0_and_tracks_computed_separately(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        physical = _FakePhysicalUART()
        wrapper = namespace["_SafeInvalidUART"](
            physical, None, 3)
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
        self.assertEqual(stats[builder.STAT_INDEX["invalid_wire_forced_invalid_frames"]], 2)
        self.assertEqual(stats[builder.STAT_INDEX["invalid_wire_ack_zero_frames"]], 2)

    def test_window_table_caps_at_four_rows_and_report_crc_covers_all_stats(self):
        namespace = self._helper_namespace()
        stats = array("I", [0] * len(builder.STAT_NAMES))
        stages = ("capture", "ring")
        table = namespace["_BoundedWindowTable"](
            stages, stats, builder.STAT_INDEX)
        perf = _FakePerf(stages)
        stats[builder.STAT_INDEX["sensor_snapshots"]] = 101
        stats[builder.STAT_INDEX["invalid_wire_frames"]] = 99
        self.assertTrue(table.record(perf, 0, 10))
        stats[builder.STAT_INDEX["sensor_snapshots"]] = 201
        stats[builder.STAT_INDEX["invalid_wire_frames"]] = 199
        self.assertTrue(table.record(perf, 1, 20))
        self.assertTrue(table.record(perf, 3, 5))
        self.assertFalse(table.record(perf, 1, 20))
        self.assertFalse(table.record(perf, 4, 30))
        self.assertEqual(table.count, 3)
        self.assertEqual(table.duplicates, 1)
        self.assertEqual(table.overflow, 1)
        self.assertEqual(table.rows[table.stride + len(table.BASE_NAMES)], 100)
        self.assertEqual(table.rows[table.stride + len(table.BASE_NAMES) + 2], 100)

        lines = namespace["_build_report_lines"](
            "B", 2, stats, builder.STAT_NAMES, table, 10000, 20,
            "none", True)
        self.assertTrue(lines[0].startswith("H750_TRACK_PROBE scope="))
        self.assertIn("computed_valid_flag_packets=0", lines[1])
        self.assertIn("production_valid_control=0", lines[0])
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
        self.assertEqual(len(stages), 17)
        table = namespace["_BoundedWindowTable"](
            stages, stats, builder.STAT_INDEX)
        table.present = bytearray((1, 1, 1, 1))
        table.rows = array("I", [0xFFFFFFFF] * (4 * table.stride))
        lines = namespace["_build_report_lines"](
            "C", 3, stats, builder.STAT_NAMES, table, 10000, 20,
            "界" * 96, True)

        line_sizes = [len(line.encode("utf-8")) for line in lines]
        largest_line = max(line_sizes)
        max_lines = (1 + (len(builder.STAT_NAMES) +
                          builder.REPORT_STATS_PER_LINE - 1) //
                     builder.REPORT_STATS_PER_LINE + 1 +
                     builder.REPORT_MAX_WINDOWS * (1 + len(stages)))
        self.assertEqual(max_lines, 78)
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
        max_console_bytes = (max_lines *
                             (builder.REPORT_LINE_MAX_UTF8_BYTES + 2) +
                             builder.REPORT_CRC_TAIL_MAX_UTF8_BYTES + 2)
        actual_console_bytes = (sum(size + 2 for size in line_sizes) +
                                len(crc_tail.encode("utf-8")) + 2)
        self.assertLessEqual(actual_console_bytes, max_console_bytes)
        self.assertLessEqual(
            len(payload), max_lines *
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
