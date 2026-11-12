import ast
from pathlib import Path
import unittest
import runpy
import sys
from types import SimpleNamespace
from unittest.mock import patch

from tools import build_openmv_full_pipeline_perf_probe as builder
from tools.build_track_profiles import RUNTIME_NAMES


ROOT = Path(__file__).resolve().parents[1]
MAIN_SOURCE = (ROOT / "openmv" / "main.py").read_text(encoding="utf-8")


def probe_body(source):
    return source[source.index(" _RUNTIME_NAMES="):]


class FullPipelinePerfProbeBuilderTests(unittest.TestCase):
    def test_runtime_allowlist_is_the_23_profile_modules(self):
        self.assertEqual(builder.RUNTIME_NAMES, RUNTIME_NAMES)
        self.assertEqual(len(builder.RUNTIME_NAMES), 23)
        self.assertIn("main.py", builder.RUNTIME_NAMES)
        self.assertIn("track_elements.py", builder.RUNTIME_NAMES)
        self.assertIn("ring_width_calibration.py", builder.RUNTIME_NAMES)
        self.assertIn("ring_diagnostics.py", builder.RUNTIME_NAMES)
        self.assertIn("fast_ring_slope.py", builder.RUNTIME_NAMES)

    def test_modes_generate_syntax_checked_full_resolution_sources(self):
        for mode in ("off", "jpeg"):
            with self.subTest(mode=mode):
                source = builder.build_probe_source(mode, 70)
                compile(source, "<generated-probe>", "exec")
                body = probe_body(source)
                self.assertIn("_MODE=%r" % mode, body)
                self.assertIn("_ACTIVE_SECONDS=70", body)
                self.assertIn("_RUN_SECONDS=120", body)
                self.assertIn("config.IMAGE_WIDTH!=320 or config.IMAGE_HEIGHT!=120", body)
                self.assertIn("sensor,_usb_vcp,config.USB_PREVIEW_ENABLED,config.USB_PREVIEW_QUALITY",
                              body)
                self.assertIn("USB_PREVIEW_ENABLED=(_MODE=='jpeg')", body)

    def test_generated_program_never_calls_physical_uart_and_uses_full_v3_sink(self):
        source = builder.build_probe_source("jpeg", 70)
        body = probe_body(source)
        tree = ast.parse(source)
        uart_calls = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "UART"
        ]
        self.assertEqual(uart_calls, [])
        self.assertNotIn("pyb.UART(", body)
        self.assertIn("_VirtualUARTSink", body)
        self.assertIn("packet[2] not in (3,0xd1) or packet[3]!=8", body)
        self.assertIn("self.v3_full_writes+=1", body)
        self.assertIn("return 16", body)
        self.assertIn("physical_uart=0", body)

        main_tree = ast.parse(MAIN_SOURCE)
        fast_main = next(
            node for node in main_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_main_fast")
        physical_calls = [
            node for node in ast.walk(fast_main)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "UART" and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "pyb"
        ]
        self.assertEqual(len(physical_calls), 1)
        fixture_uart_branches = [
            node for node in ast.walk(fast_main)
            if isinstance(node, ast.If) and isinstance(node.test, ast.Name)
            and node.test.id == "fixture_mode"
            and any(call is physical_calls[0] for statement in node.orelse
                    for call in ast.walk(statement))
        ]
        self.assertEqual(len(fixture_uart_branches), 1)
        self.assertFalse(any(call is physical_calls[0]
                             for statement in fixture_uart_branches[0].body
                             for call in ast.walk(statement)))

    def test_generated_program_runs_production_main_with_both_virtual_fixtures(self):
        source = builder.build_probe_source("off", 70)
        body = probe_body(source)
        tree = ast.parse(source)
        calls = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "main" and node.func.attr == "_main_fast"
        ]
        self.assertEqual(len(calls), 1)
        keywords = {keyword.arg: keyword.value for keyword in calls[0].keywords}
        self.assertIn("frame_source", keywords)
        self.assertIn("uart_sink", keywords)
        self.assertIn("fixture_sensor_id", keywords)
        self.assertIn("_source=_ProbeSource(_native_source)", body)
        self.assertIn("_native_source=main._NativeUSBPreviewSource(", body)
        self.assertIn("main._is_native_usb_preview_source(_source)", body)
        main_loop = MAIN_SOURCE[MAIN_SOURCE.index("def _main_fast("):]
        self.assertIn("img = frame_source.snapshot()", main_loop)
        main_tree = ast.parse(MAIN_SOURCE)
        fast_main = next(
            node for node in main_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_main_fast")
        while_loops = [node for node in ast.walk(fast_main)
                       if isinstance(node, ast.While)]
        self.assertTrue(any(
            isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
            and call.func.attr == "snapshot"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "frame_source"
            for loop in while_loops for statement in loop.body
            for call in ast.walk(statement)))

    def test_runtime_modules_are_all_cleared_before_any_import_and_share_identity(self):
        source = builder.build_probe_source("off", 70)
        tree = ast.parse(source)
        module_loops = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.For) and isinstance(node.target, ast.Name)
            and node.target.id == "_name"
        ]
        module_loops.sort(key=lambda node: node.lineno)
        self.assertEqual(len(module_loops), 2)
        clear_loop, import_loop = module_loops
        clear_dump = ast.dump(clear_loop, include_attributes=False)
        import_dump = ast.dump(import_loop, include_attributes=False)
        self.assertIn("sys", clear_dump)
        self.assertIn("modules", clear_dump)
        self.assertIn("Delete", clear_dump)
        self.assertNotIn("__import__", clear_dump)
        self.assertIn("__import__", import_dump)
        self.assertNotIn("Delete", import_dump)
        self.assertIn("if main.config is not config", probe_body(source))
        self.assertIn("if main.reverse_control is not reverse_control", probe_body(source))

    def test_active_window_reports_start_after_gate_ack_and_requires_70_seconds(self):
        source = builder.build_probe_source("jpeg", 70)
        body = probe_body(source)
        self.assertIn("V7_ACTIVE_PERF_WINDOW", body)
        self.assertIn("_ACTIVE_START_REL_MS", body)
        self.assertIn("_ACTIVE_ELAPSED_MS>=_ACTIVE_SECONDS*1000", body)
        self.assertIn("_ACTIVE_WINDOW_REQUIREMENT=(_ACTIVE_SECONDS//10)", body)
        self.assertIn("_ACTIVE_SECONDS=70", body)
        self.assertIn("_RUN_SECONDS=120", body)
        self.assertIn("physical_uart=0", body)

    def test_marker_is_production_marker_after_processing_and_before_flush(self):
        source = builder.build_probe_source("jpeg", 70)
        body = probe_body(source)
        self.assertIn("config.PREVIEW_FPS_MARKER_ENABLED=(_MODE=='jpeg')", body)
        self.assertIn("_original_marker=main._draw_preview_frame_marker", body)
        self.assertIn("marker_total_us=%d", body)
        self.assertTrue(
            MAIN_SOURCE.index("_draw_gray_result(", MAIN_SOURCE.index(
                "def _render_v7_preview_frame(")) <
            MAIN_SOURCE.index("_draw_preview_frame_marker(img, preview_sequence)",
                              MAIN_SOURCE.index("def _render_v7_preview_frame(")) <
            MAIN_SOURCE.index("flush_func()", MAIN_SOURCE.index(
                "def _render_v7_preview_frame(")))
        builder.validate_preview_marker_order(MAIN_SOURCE)

    def test_profile_c_parked_gate_then_fresh_active_token_one(self):
        source = builder.build_probe_source("jpeg", 70)
        body = probe_body(source)
        self.assertIn("reverse_control.PROFILE_C", body)
        self.assertIn("flags=reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED", body)
        self.assertIn("reverse_control.PROFILE_SESSION_ACTIVE", body)
        self.assertIn("session_token=token,mean_forward_wheel_millirev=0", body)
        self.assertIn("token=1", body)
        self.assertIn("self.next_keep_ms=time.ticks_add(now,50)", body)
        self.assertIn("self.next_profile_ms=time.ticks_add(now,100)", body)
        self.assertIn("self.profile_c_parked_after_ack>=3", body)
        self.assertIn("self.parked_ack_valid_seen=True", body)
        self.assertIn("self.active_ack_start_ms=time.ticks_ms()", body)
        self.assertIn("_GATE_TIMEOUT_MS=30000", body)
        self.assertIn("raise _GateTimeout", body)
        self.assertIn("reverse_control.COMMAND_RESET", body)
        self.assertIn("reverse_control.COMMAND_KEEP", body)
        self.assertNotIn("reverse_control.PROFILE_A", body)

    def test_diagnostics_persist_only_to_bounded_ram_and_calibration_is_read_only(self):
        source = builder.build_probe_source("off", 70)
        body = probe_body(source)
        self.assertIn("RingWidthCalibration.load(", body)
        self.assertNotRegex(
            body,
            r"ring_width_calibration\.RingWidthCalibration\.save\s*\(")
        self.assertIn("RingDiagnostics.save=_ram_only_diag_save", body)
        self.assertIn("RingDiagnostics.save_next=_ram_only_diag_save_next", body)
        self.assertIn("_RAM_DIAG_BLOB[0]=self._payload(build_id)", body)
        self.assertIn("config.RING_DIAGNOSTICS_PATH='/ramprobe/ring_diagnostics.bin'", body)
        self.assertIn("ring_diagnostics_records_retained_per_frame", builder.build_manifest(
            source, "off", 70)["flash_io"])
        self.assertIn("profile_c_parked_after_ack", body)
        self.assertIn("ring_route_behavior_proven=0", body)

    def test_manifest_hashes_all_exact_source_modules_and_keeps_limits_explicit(self):
        source = builder.build_probe_source("jpeg", 75)
        manifest = builder.build_manifest(source, "jpeg", 75)
        self.assertEqual(manifest["runtime_module_count"], 23)
        self.assertEqual(
            [row["name"] for row in manifest["runtime_modules"]],
            list(RUNTIME_NAMES))
        self.assertEqual(len(manifest["source_sha256"]), 64)
        self.assertTrue(all(len(row["source_sha256"]) == 64
                            for row in manifest["runtime_modules"]))
        self.assertFalse(manifest["camera"]["actual_ide_new_frame_rate_proven"])
        self.assertFalse(manifest["virtual_input"]["real_vehicle_measurement"])
        self.assertTrue(any("parked calibration" in item
                            for item in manifest["acceptance_limits"]))

    def test_builder_rejects_short_duration_or_wrong_mode(self):
        with self.assertRaises(ValueError):
            builder.build_probe_source("jpeg", 69)
        with self.assertRaises(ValueError):
            builder.build_probe_source("legacy", 70)

    def test_sink_executes_real_v3_and_d1_crc_and_gate_accounting(self):
        tree = ast.parse(builder.build_probe_source("off", 70))
        sink_class = next(n for n in ast.walk(tree)
                          if isinstance(n, ast.ClassDef) and n.name == "_VirtualUARTSink")
        micro = SimpleNamespace(native=lambda f: f, viper=lambda f: f)
        with patch.dict(sys.modules, {"micropython": micro}):
            protocol = SimpleNamespace(**runpy.run_path(str(ROOT / "openmv/vision_protocol.py")))
        namespace = {"main": SimpleNamespace(vision_protocol=protocol),
                     "reverse_control": SimpleNamespace(PROFILE_C=3),
                     "time": SimpleNamespace(ticks_ms=lambda: 1234)}
        exec(compile(ast.Module(body=[sink_class], type_ignores=[]), "<sink>", "exec"), namespace)
        sink = namespace["_VirtualUARTSink"]()
        v3 = protocol.pack_frame_v3(1, True, 90, -443, profile_ack=3)
        self.assertEqual(sink.write(v3), 16)
        self.assertTrue(sink.parked_ack_valid_seen)
        d1 = protocol.pack_diagnostic_into(bytearray(16), 2, 1, 0, 0, 0, 0, 3, 0, 0)
        self.assertEqual(sink.write(d1), 16)
        self.assertEqual((sink.v3_full_writes, sink.diagnostic_writes), (1, 1))
        self.assertIsNone(sink.active_ack_start_ms)
        sink.active_profile_sent = True
        sink.write(v3)
        self.assertEqual(sink.active_ack_start_ms, 1234)
        v3[8] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            sink.write(v3)
        self.assertEqual(sink.write_errors, 1)

    def test_lazy_loaders_precede_identity_checks_and_deadline_escapes_capture_retry(self):
        body = probe_body(builder.build_probe_source("off", 70))
        check = body.index("if main.config is not config")
        for loader in ("fast", "singleedge", "track_elements", "ring"):
            self.assertLess(body.index("main._load_%s_runtime()" % loader), check)
        self.assertIn("class _GateTimeout(BaseException)", body)
        # A capture retry catches Exception; the finite probe deadline must escape it.
        tree = ast.parse(builder.build_probe_source("off", 70))
        deadline_class = next(n for n in ast.walk(tree)
                              if isinstance(n, ast.ClassDef) and n.name == "_GateTimeout")
        namespace = {}
        exec(compile(ast.Module(body=[deadline_class], type_ignores=[]), "<deadline>", "exec"), namespace)
        self.assertFalse(issubclass(namespace["_GateTimeout"], Exception))


if __name__ == "__main__":
    unittest.main()
