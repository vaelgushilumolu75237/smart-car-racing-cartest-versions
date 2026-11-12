import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from tools import benchmark_openmv_preview as bench


ROOT = Path(__file__).resolve().parents[1]
RAM_SOURCE = ROOT / "tests" / "fixtures" / "openmv_ram_source.py"
SDK_PATH = (Path(os.environ["OPENMV_SDK_PATH"]).expanduser()
            if os.environ.get("OPENMV_SDK_PATH") else None)
SDK_AVAILABLE = bool(SDK_PATH and (SDK_PATH / "openmv" / "camera.py").is_file())


class FakeCamera:
    def __init__(self, port, timeout, max_retry, crc=True, seq=True,
                 ack=True, events=True, max_payload=4096):
        self.port = port
        self.timeout = timeout
        self.max_retry = max_retry
        self.caps = {
            "crc": crc,
            "seq": seq,
            "ack": ack,
            "events": events,
            "max_payload": max_payload,
        }
        self.transport = SimpleNamespace(stats={
            "sent": 14, "received": 22, "checksum": 0, "sequence": 0,
        })
        self.calls = []
        self.stdout_reads = 0
        self.pre_exec_stdout_reads = 0
        self.stdout_chunks = []
        self.pre_exec_stdout_chunks = []
        self.pre_exec_stdout_queue = [
            "old program V7_BUILD=stale backend=old\r\n"]
        self.pre_exec_status_polls = 0
        self.post_exec_status_polls = 0
        self.stdout_ready_after_status_polls = 1
        self.frame_reads = 0
        self.connected = False
        self.executed = False
        self.stream_enabled = False
        self.first_stdout_delay_s = 0.0
        self.stop_delay_s = 0.0
        self.streaming_delay_s = 0.0
        self.frame_data_factory = None

    def connect(self):
        self.calls.append(("connect",))
        self.connected = True
        self.caps["max_payload"] = min(self.caps["max_payload"], 3072)

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.connected = False

    def stop(self):
        self.calls.append(("stop",))
        if self.stop_delay_s:
            time.sleep(self.stop_delay_s)

    def streaming(self, enabled, raw=False, resolution=None):
        self.calls.append(("streaming", enabled, raw, resolution))
        if self.streaming_delay_s:
            time.sleep(self.streaming_delay_s)
        self.stream_enabled = enabled

    def exec(self, source):
        self.calls.append(("exec", len(source)))
        self.executed = True

    def read_stdout(self):
        self.calls.append(("read_stdout",))
        if not self.executed:
            self.pre_exec_stdout_reads += 1
            if self.pre_exec_stdout_queue:
                chunk = self.pre_exec_stdout_queue.pop(0)
                self.pre_exec_stdout_chunks.append(chunk)
                return chunk
            return None
        self.stdout_reads += 1
        if self.stdout_reads == 1:
            if self.first_stdout_delay_s:
                time.sleep(self.first_stdout_delay_s)
            chunk = ("V7_BUILD=fixture backend=fast camera=320x120 "
                     "uart_sink=fake\r\n")
            self.stdout_chunks.append(chunk)
            return chunk
        if self.stdout_reads == 2:
            chunk = (
                "V7_PERF_WINDOW run=7 index=0 start_ms=0 end_ms=10000 "
                "distinct_captures=601 valid_frames=598\r\n"
                "V7_PERF_STAGE run=7 index=0 capture:n=600,avg=900,"
                "p95_upper=1000,max=1800\r\n")
            self.stdout_chunks.append(chunk)
            return chunk
        return None

    def read_status(self):
        self.calls.append(("read_status",))
        if self.executed:
            self.post_exec_status_polls += 1
            stdout_ready = (
                self.stdout_reads < 2 and
                self.post_exec_status_polls >=
                self.stdout_ready_after_status_polls)
        else:
            self.pre_exec_status_polls += 1
            stdout_ready = bool(self.pre_exec_stdout_queue)
        return {"stdout": stdout_ready, "stream": self.stream_enabled}

    def read_frame(self):
        self.calls.append(("read_frame",))
        self.frame_reads += 1
        data = (self.frame_data_factory(self.frame_reads)
                if self.frame_data_factory else b"same static host frame")
        return {"width": 320, "height": 120,
                "format": 2, "depth": 16, "raw_size": 4800,
                "data": data}


class ExitAtSdkCallFakeCamera(FakeCamera):
    def __init__(self, exit_at, exit_exception, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.exit_at = exit_at
        self.exit_exception = exit_exception

    def connect(self):
        if self.exit_at == "connect":
            self.calls.append(("connect",))
            raise self.exit_exception
        return super().connect()

    def exec(self, source):
        if self.exit_at == "exec":
            self.calls.append(("exec", len(source)))
            raise self.exit_exception
        return super().exec(source)


def options_for(mode, output_dir, **overrides):
    values = {
        "port": "COM_TEST",
        "mode": mode,
        "ide_state": "unspecified",
        "ram_source": RAM_SOURCE,
        # The SDK is not consulted when a fake camera factory is supplied.
        # Keep all mocks self-contained and independent of checkout-local tmp.
        "sdk_path": output_dir / "sdk-not-used-by-mock",
        "duration": 0.04,
        "startup_timeout": 1.0,
        "stop_timeout": 5.0,
        "control_timeout": 2.0,
        "startup_drain_timeout": 0.5,
        "io_timeout": 0.15,
        "stdout_timeout": 5.0,
        "frame_timeout": 2.0,
        "crc": True,
        "seq": True,
        "ack": True,
        "events": True,
        "max_payload": 4096,
        "poll_ms": 1.0,
        "marker_sequences": False,
        "synthetic_marker_probe": False,
        "fast_sdk_poll": True,
        "output_dir": output_dir,
    }
    values.update(overrides)
    return type("Options", (), values)()


class BenchmarkOpenMVPreviewTests(unittest.TestCase):
    def test_operation_budgets_do_not_reconfigure_open_serial_timeout(self):
        class AssignmentCountingSerial:
            def __init__(self, timeout):
                self._timeout = timeout
                self.timeout_assignments = 0

            @property
            def timeout(self):
                return self._timeout

            @timeout.setter
            def timeout(self, value):
                self.timeout_assignments += 1
                self._timeout = value

        serial = AssignmentCountingSerial(0.15)
        camera = SimpleNamespace(
            timeout=0.15,
            transport=SimpleNamespace(timeout=0.15),
            _serial=serial,
            read_status=lambda: "status",
            read_stdout=lambda: "stdout",
        )
        samples = {"read_status": [], "read_stdout": []}

        bench._bounded_camera_call(
            camera, "read_status", (), 0.15, 0.15, samples)
        self.assertEqual(serial.timeout_assignments, 0)
        self.assertEqual(serial.timeout, 0.15)

        bench._bounded_camera_call(
            camera, "read_stdout", (), 5.0, 5.0, samples)
        self.assertEqual(camera.timeout, 5.0)
        self.assertEqual(camera.transport.timeout, 5.0)
        self.assertEqual(serial.timeout, 0.15)
        self.assertEqual(serial.timeout_assignments, 0)

        bench._bounded_camera_call(
            camera, "read_status", (), 2.0, 2.0, samples)
        self.assertEqual(camera.timeout, 2.0)
        self.assertEqual(camera.transport.timeout, 2.0)
        self.assertEqual(serial.timeout, 0.15)
        self.assertEqual(serial.timeout_assignments, 0)
        self.assertEqual(len(samples["read_status"]), 2)
        self.assertEqual(len(samples["read_stdout"]), 1)

    def test_parser_requires_explicit_run_before_any_device_setup(self):
        stderr = io.StringIO()
        arguments = [
            "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
            "--mode", "off",
        ]
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                bench.main(arguments, camera_factory=lambda *a, **k:
                           self.fail("camera factory must not be called"))
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("unless --run is supplied", stderr.getvalue())

    @unittest.skipUnless(
        SDK_AVAILABLE,
        "set OPENMV_SDK_PATH to a local SDK checkout to test SDK imports")
    def test_local_sdk_camera_import_is_available_without_connecting(self):
        camera_class = bench.load_camera_factory(SDK_PATH)
        self.assertEqual(camera_class.__name__, "Camera")
        self.assertTrue(callable(camera_class))

    def test_parser_records_protocol_settings_and_frame_deadline(self):
        args = bench.parse_args([
            "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
            "--run", "--mode", "raw", "--no-crc", "--no-ack",
            "--no-events", "--max-payload", "16384",
            "--frame-timeout", "3.5", "--stdout-timeout", "4.0",
        ])
        self.assertFalse(args.crc)
        self.assertTrue(args.seq)
        self.assertFalse(args.ack)
        self.assertFalse(args.events)
        self.assertEqual(args.max_payload, 16384)
        self.assertEqual(args.frame_timeout, 3.5)
        self.assertEqual(args.stdout_timeout, 4.0)
        self.assertEqual(args.stop_timeout, 5.0)
        self.assertEqual(args.control_timeout, 2.0)
        self.assertEqual(args.startup_drain_timeout, 0.5)
        self.assertTrue(args.fast_sdk_poll)
        self.assertEqual(bench.RAW_STREAM_RESOLUTION, (320, 120))
        marked = bench.parse_args([
            "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
            "--run", "--mode", "jpeg", "--marker-sequences",
        ])
        self.assertTrue(marked.marker_sequences)
        synthetic = bench.parse_args([
            "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
            "--run", "--mode", "jpeg", "--marker-sequences",
            "--synthetic-marker-probe",
        ])
        self.assertTrue(synthetic.synthetic_marker_probe)

    def test_synthetic_probe_requires_jpeg_and_sequence_validation(self):
        for extra, expected in ((["--mode", "off"], "requires --mode jpeg"),
                                (["--mode", "jpeg"],
                                 "requires --marker-sequences")):
            with self.subTest(extra=extra):
                stderr = io.StringIO()
                args = ["--port", "COM_TEST", "--ram-source",
                        str(RAM_SOURCE), "--run", "--synthetic-marker-probe"]
                args += extra
                with contextlib.redirect_stderr(stderr):
                    with self.assertRaises(SystemExit) as raised:
                        bench.parse_args(args)
                self.assertEqual(raised.exception.code, 2)
                self.assertIn(expected, stderr.getvalue())

    def test_parser_rejects_out_of_range_requested_payload(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                bench.parse_args([
                    "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
                    "--run", "--mode", "raw", "--max-payload", "65536",
                ])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("52..65535", stderr.getvalue())

    def test_benchmark_order_and_artifacts_for_all_three_modes(self):
        for mode in ("off", "jpeg", "raw"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as tmp:
                instance = None

                def factory(*args, **kwargs):
                    nonlocal instance
                    instance = FakeCamera(*args, **kwargs)
                    return instance

                report = bench.run_benchmark(
                    options_for(mode, Path(tmp)), camera_factory=factory)
                self.assertEqual(report["status"], "completed")
                self.assertIsNotNone(report["v7_build"])
                self.assertEqual(report["board_perf_window_count"], 1)
                self.assertEqual(report["board_counts_by_window"], [{
                    "run": 7, "index": 0, "start_ms": 0, "end_ms": 10000,
                    "distinct_captures": 601, "valid_frames": 598,
                }])
                self.assertEqual(report["operator_declared_ide_state"],
                                 "unspecified")

                calls = instance.calls
                names = [call[0] for call in calls]
                self.assertTrue(set(names).issubset({
                    "connect", "stop", "disconnect", "streaming", "exec",
                    "read_stdout", "read_status", "read_frame",
                }))
                self.assertLess(names.index("connect"), names.index("stop"))
                self.assertEqual(names.index("stop"),
                                 names.index("connect") + 1)
                initial_stream = next(i for i, call in enumerate(calls)
                                      if call[0] == "streaming")
                self.assertLess(names.index("stop"), initial_stream)
                pre_exec_read = next(i for i, call in enumerate(calls)
                                     if call[0] == "read_stdout")
                self.assertLess(initial_stream, pre_exec_read)
                exec_index = names.index("exec")
                self.assertLess(pre_exec_read, exec_index)
                self.assertEqual(calls[initial_stream],
                                 ("streaming", False, False, None))
                if mode == "off":
                    self.assertTrue(any(call[0] == "read_status"
                                        for call in calls))
                    self.assertFalse(any(call[0] == "read_frame"
                                         for call in calls))
                    self.assertEqual(report["host_observation"][
                        "host_sdk_frame_reads"], 0)
                else:
                    enabled = next(i for i, call in enumerate(calls)
                                   if call[0] == "streaming" and call[1])
                    self.assertGreater(enabled, exec_index)
                    self.assertEqual(enabled, exec_index + 1)
                    post_exec_read = next(
                        i for i, call in enumerate(calls)
                        if i > exec_index and call[0] == "read_stdout")
                    ready_status = next(
                        i for i, call in enumerate(calls)
                        if i > enabled and call[0] == "read_status")
                    self.assertLess(enabled, ready_status)
                    self.assertLess(ready_status, post_exec_read)
                    if mode == "jpeg":
                        self.assertEqual(calls[enabled],
                                         ("streaming", True, False, None))
                    else:
                        self.assertEqual(calls[enabled],
                                         ("streaming", True, True, (320, 120)))
                    host = report["host_observation"]
                    self.assertGreater(host["host_sdk_frame_reads"], 0)
                    self.assertEqual(host["host_unique_content_hash_count"], 1)
                    self.assertEqual(host["host_duplicate_content_hash_reads"],
                                     host["host_sdk_frame_reads"] - 1)
                    self.assertEqual(host["host_frame_returns"], [
                        {"width": 320, "height": 120, "format": 2,
                         "depth": 16, "raw_size_bytes": 4800,
                         "rgb888_size_bytes": len(b"same static host frame"),
                         "reads": host["host_sdk_frame_reads"]}
                    ])
                    self.assertFalse(host["camera_fps_claimed_by_host"])
                    self.assertIn("not a sensor capture count",
                                  host["frame_metric_note"])
                    self.assertIn("does not identify a fresh sensor frame",
                                  host["frame_metric_note"])
                    self.assertNotIn("host_fps", host)

                self.assertEqual(names[-1], "disconnect")
                self.assertNotEqual(names[-2], "read_stdout")
                if mode == "off":
                    self.assertEqual(names[-2], "read_status")
                else:
                    self.assertEqual(names[-3:], [
                        "streaming", "read_status", "disconnect"])
                output_dir = Path(report["report_dir"])
                raw_bytes = (output_dir / "raw_stdout.txt").read_bytes()
                self.assertEqual(raw_bytes,
                                 "".join(instance.stdout_chunks).encode("utf-8"))
                raw_stdout = raw_bytes.decode("utf-8")
                self.assertIn("V7_BUILD=fixture", raw_stdout)
                self.assertNotIn("V7_BUILD=stale", raw_stdout)
                self.assertIn("V7_PERF_WINDOW", raw_stdout)
                pre_exec_bytes = (output_dir / "pre_exec_stdout.txt").read_bytes()
                pre_exec_stdout = pre_exec_bytes.decode("utf-8")
                self.assertIn("V7_BUILD=stale", pre_exec_stdout)
                self.assertTrue(report["pre_exec_stdout"][
                    "drain_completed_before_exec"])
                self.assertEqual(report["pre_exec_stdout"]["bytes"],
                                 len(pre_exec_bytes))
                self.assertEqual(report["pre_exec_stdout"]["read_calls"], 1)
                self.assertGreaterEqual(
                    report["pre_exec_stdout"]["status_polls"], 1)
                perf = json.loads((output_dir /
                                   "board_v7_perf_windows.json").read_text(
                                       encoding="utf-8"))
                self.assertEqual(perf["windows"][0]["fields"]["distinct_captures"],
                                 601)
                self.assertEqual(perf["windows"][0]["stages"][0]["summary"],
                                 "capture:n=600,avg=900,p95_upper=1000,max=1800")
                saved = json.loads((output_dir / "report.json").read_text(
                    encoding="utf-8"))
                self.assertEqual(saved["mode"], mode)
                self.assertEqual(
                    saved["stream_enable_order"],
                    "not_enabled" if mode == "off" else
                    "immediately_after_exec_before_v7_build_wait")
                self.assertIn("no camera file write", saved["ram_execution"])
                self.assertFalse(saved["sensor_resolution_modified_by_tool"])
                self.assertEqual(saved["requested_protocol_caps"], {
                    "crc": True, "seq": True, "ack": True, "events": True,
                    "max_payload": 4096,
                })
                self.assertEqual(saved["negotiated_protocol_caps"], {
                    "crc": True, "seq": True, "ack": True, "events": True,
                    "max_payload": 3072,
                })
                self.assertEqual(saved["host_transport_stats"]["received"], 22)

    def test_run_passes_protocol_requests_and_records_negotiated_caps(self):
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                return instance

            report = bench.run_benchmark(
                options_for("raw", Path(tmp), crc=False, seq=False,
                            ack=False, events=False, max_payload=16384),
                camera_factory=factory)
            self.assertEqual(report["status"], "completed")
            self.assertEqual(report["requested_protocol_caps"], {
                "crc": False, "seq": False, "ack": False, "events": False,
                "max_payload": 16384,
            })
            self.assertEqual(report["negotiated_protocol_caps"], {
                "crc": False, "seq": False, "ack": False, "events": False,
                "max_payload": 3072,
            })
            self.assertEqual(instance.caps, report["negotiated_protocol_caps"])

    def test_marker_sequence_reporting_ignores_content_hash_for_freshness(self):
        from tools.openmv_preview_marker import paint_marker_grayscale_for_test

        def frame_for(sequence):
            gray = bytearray(320 * 120)
            paint_marker_grayscale_for_test(gray, sequence)
            return bytes(channel for value in gray for channel in (value,) * 3)

        corrupt = bytearray(frame_for(45))
        # Flip the sampled first sequence bit while leaving the CRC byte alone.
        cell_x = 32 + 8 * 8
        for y in range(106, 110):
            for x in range(cell_x + 2, cell_x + 6):
                offset = (y * 320 + x) * 3
                corrupt[offset:offset + 3] = bytes((255, 255, 255))
        corrupt = bytes(corrupt)
        valid_frames = {sequence: frame_for(sequence)
                        for sequence in (41, 42, 43)}
        frame_by_read = {
            1: valid_frames[41],
            2: valid_frames[41],
            3: valid_frames[43],
            4: valid_frames[42],
            5: corrupt,
        }
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                instance.frame_data_factory = lambda read: frame_by_read.get(
                    read, valid_frames[43])
                return instance

            report = bench.run_benchmark(
                options_for("jpeg", Path(tmp), duration=0.3,
                            marker_sequences=True), camera_factory=factory)
            observation = report["host_observation"]
            markers = observation["host_marker_sequence_observation"]
            windows = observation["host_marker_sequence_windows"]
            self.assertGreaterEqual(observation["host_sdk_frame_reads"], 4)
            self.assertEqual(markers["new_sequence_reads"], 2)
            self.assertGreaterEqual(markers["repeated_sequence_reads"], 1)
            self.assertEqual(markers["sequence_gap_events"], 1)
            self.assertEqual(markers["missing_sequence_values"], 1)
            self.assertEqual(markers["out_of_order_sequence_reads"], 1)
            self.assertGreaterEqual(markers["crc_failures"], 1)
            self.assertFalse(markers["camera_fps_claimed_by_host"])
            self.assertFalse(observation["camera_fps_claimed_by_host"])
            self.assertEqual(markers["fresh_frame_metric_basis"].split(";")[0],
                             "valid forward marker sequence transitions")
            self.assertEqual(windows["status"],
                             "incomplete_observation_window")
            self.assertFalse(windows["measurement_success"])
            self.assertEqual(windows["new_sequence_arrivals"], 2)
            self.assertEqual(windows["windows"][0]["fresh_count"], 2)
            self.assertFalse(windows["windows"][0]["complete"])
            arrival_path = (Path(report["report_dir"]) /
                            report["artifacts"]["marker_sequence_arrivals"])
            arrivals = [json.loads(line) for line in
                        arrival_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([item["sequence"] for item in arrivals], [41, 43])
            self.assertTrue(all("host_monotonic_ns" in item
                                for item in arrivals))
            self.assertLess(arrivals[0]["host_monotonic_s"],
                            arrivals[1]["host_monotonic_s"])

    def test_marker_windows_require_last_valid_arrival_to_reach_window_end(self):
        arrivals = [
            {"sequence": 20, "host_monotonic_s": 100.0,
             "run_elapsed_s": 2.0},
            {"sequence": 21, "host_monotonic_s": 109.999,
             "run_elapsed_s": 12.0},
            {"sequence": 23, "host_monotonic_s": 110.0,
             "run_elapsed_s": 13.0},
            {"sequence": 24, "host_monotonic_s": 125.0,
             "run_elapsed_s": 30.0},
        ]
        windows = bench.summarize_marker_arrival_windows(arrivals)
        self.assertEqual(windows["status"], "complete_windows_available")
        self.assertTrue(windows["measurement_success"])
        self.assertEqual(windows["t0_host_monotonic_s"], 100.0)
        self.assertEqual(windows["t0_run_elapsed_s"], 2.0)
        self.assertEqual(windows["last_marker_elapsed_from_t0_s"], 25.0)
        self.assertEqual(len(windows["windows"]), 3)
        self.assertEqual([item["fresh_count"] for item in windows["windows"]],
                         [2, 1, 1])
        self.assertEqual([item["complete"] for item in windows["windows"]],
                         [True, True, False])
        self.assertEqual(windows["windows"][0]["rate_hz"], 0.2)
        self.assertIsNone(windows["windows"][2]["rate_hz"])

    def test_no_marker_arrivals_are_not_valid_measurement_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = bench.run_benchmark(
                options_for("off", Path(tmp), duration=0.01,
                            marker_sequences=True),
                camera_factory=FakeCamera)
        windows = report["host_observation"][
            "host_marker_sequence_windows"]
        self.assertEqual(report["status"], "completed")
        self.assertEqual(windows["status"], "no_valid_marker_arrivals")
        self.assertFalse(windows["measurement_success"])
        self.assertEqual(windows["windows"], [])
        self.assertFalse(report["terminal_marker_is_success_evidence"])

    def test_board_traceback_plus_terminal_marker_is_execution_error_and_nonzero(self):
        class TracebackFakeCamera(FakeCamera):
            def read_stdout(self):
                chunk = super().read_stdout()
                if self.executed and self.stdout_reads == 2:
                    chunk = (chunk or "") + (
                        "Traceback (most recent call last):\r\n"
                        "  File \"<stdin>\", line 49, in <module>\r\n"
                        "TypeError: '_V7SummaryQueue' object is not callable\r\n"
                        "RAM_USB_PREVIEW_PROBE_FINISHED\r\n")
                    self.stdout_chunks[-1] = chunk
                    return chunk
                return chunk

        with tempfile.TemporaryDirectory() as tmp:
            stdout = io.StringIO()
            args = [
                "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
                "--run", "--mode", "jpeg", "--duration", "1",
                "--output-dir", tmp,
            ]
            with contextlib.redirect_stdout(stdout):
                exit_code = bench.main(
                    args, camera_factory=TracebackFakeCamera)
            printed = stdout.getvalue()
            report_dir = printed.split("report_dir=", 1)[1].splitlines()[0]
            saved = json.loads(
                (Path(report_dir) / "report.json").read_text(
                    encoding="utf-8"))
            raw = (Path(report_dir) / "raw_stdout.txt").read_text(
                encoding="utf-8")
        self.assertEqual(exit_code, 1)
        self.assertIn("status=execution_error", printed)
        self.assertEqual(saved["status"], "execution_error")
        self.assertEqual(saved["observation_end_reason"],
                         "board_execution_error")
        self.assertTrue(saved["script_terminal_marker_seen"])
        self.assertFalse(saved["terminal_marker_is_success_evidence"])
        self.assertEqual(saved["board_execution_error_count"], 1)
        self.assertIn("TypeError: '_V7SummaryQueue' object is not callable",
                      saved["board_execution_errors"][0]["raw_excerpt"])
        self.assertIn("Traceback (most recent call last):", raw)

    def test_sdk_system_exit_from_connect_or_exec_is_persisted_and_nonzero(self):
        for exit_at in ("connect", "exec", "keyboard_interrupt"):
            with self.subTest(exit_at=exit_at), \
                    tempfile.TemporaryDirectory() as tmp:
                output = io.StringIO()

                def factory(*args, **kwargs):
                    if exit_at == "keyboard_interrupt":
                        return ExitAtSdkCallFakeCamera(
                            "connect", KeyboardInterrupt(), *args, **kwargs)
                    return ExitAtSdkCallFakeCamera(
                        exit_at, SystemExit(0), *args, **kwargs)

                args = [
                    "--port", "COM_TEST", "--ram-source", str(RAM_SOURCE),
                    "--run", "--mode", "jpeg", "--duration", "1",
                    "--output-dir", tmp,
                ]
                with contextlib.redirect_stdout(output):
                    exit_code = bench.main(args, camera_factory=factory)

                printed = output.getvalue()
                self.assertIn("status=", printed)
                report_dir = printed.split("report_dir=", 1)[1].splitlines()[0]
                report_path = Path(report_dir) / "report.json"
                raw_path = Path(report_dir) / "raw_stdout.txt"
                self.assertTrue(report_path.is_file())
                self.assertTrue(raw_path.is_file())
                saved = json.loads(report_path.read_text(encoding="utf-8"))
                self.assertEqual(exit_code, 1)
                if exit_at == "keyboard_interrupt":
                    self.assertEqual(saved["status"], "interrupted")
                    self.assertEqual(saved["error"], "KeyboardInterrupt")
                else:
                    self.assertEqual(saved["status"], "error")
                    self.assertIn("SystemExit(code=0)", saved["error"])
                    self.assertIn("OpenMV SDK %s" % exit_at, saved["error"])

    def test_explicit_board_error_marker_is_detected(self):
        errors = bench.parse_board_execution_errors(
            "V7_BUILD=fixture\r\nV7_ERROR status=failed code=7\r\n")
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0]["kind"], "explicit_error")
        self.assertIn("V7_ERROR", errors[0]["raw_excerpt"])

    def test_reference_error_diagnostic_is_not_an_execution_error(self):
        errors = bench.parse_board_execution_errors(
            "V7DBG frame=566 reference_error=+139 cache_rows=52\r\n")
        self.assertEqual(errors, [])

    def test_synthetic_probe_status_and_origin_are_explicit_in_json(self):
        from tools import build_openmv_synthetic_marker_probe as synthetic_builder

        class SyntheticFakeCamera(FakeCamera):
            def read_stdout(self):
                if not self.executed:
                    return super().read_stdout()
                self.calls.append(("read_stdout",))
                self.stdout_reads += 1
                if self.stdout_reads == 1:
                    chunk = ("V7_BUILD=synthetic_marker_probe camera=none "
                             "frame=320x120\r\n")
                elif self.stdout_reads == 2:
                    chunk = (
                        "V7_SYNTHETIC_MARKER_COUNTS first_seq=65532 "
                        "last_seq=20 sequence_count=25 repeats_per_sequence=3 "
                        "publishes=75 elapsed_ms=1600 deadline_hit=0\r\n"
                        "V7_SYNTHETIC_MARKER_DONE status=ok\r\n")
                else:
                    return None
                self.stdout_chunks.append(chunk)
                return chunk

        with tempfile.TemporaryDirectory() as tmp:
            source_path = Path(tmp) / "synthetic-marker.py"
            source_path.write_bytes(
                synthetic_builder.build_probe_source().encode("utf-8"))
            report = bench.run_benchmark(
                options_for("jpeg", Path(tmp), duration=0.02,
                            marker_sequences=True,
                            synthetic_marker_probe=True,
                            ram_source=source_path),
                camera_factory=lambda *args, **kwargs:
                    SyntheticFakeCamera(*args, **kwargs))
            saved = json.loads(
                (Path(report["report_dir"]) / "report.json").read_text(
                    encoding="utf-8"))
        self.assertEqual(report["status"], "completed")
        self.assertTrue(report["synthetic_scene_not_camera_capture"])
        self.assertEqual(report["preview_source_kind"],
                         "synthetic_grayscale_marker_probe")
        self.assertIn("bypasses production V7/H743 path",
                      report["h743_uart_fixture"])
        self.assertEqual(report["raw_resolution_scope"],
                         "synthetic 320x120 image; sensor is not accessed")
        self.assertTrue(report["synthetic_marker_probe_completed"])
        self.assertEqual(
            report["synthetic_marker_board_counts"]["fields"]["publishes"],
            75)
        self.assertEqual(report["script_terminal_marker"],
                         "V7_SYNTHETIC_MARKER_DONE status=")
        self.assertFalse(report["requested_duration_satisfied"])
        self.assertEqual(report["observation_end_reason"],
                         "script_terminal_marker")
        self.assertFalse(report["stream_cleanup"]["disable_attempted"])
        self.assertEqual(report["stream_cleanup"]["disable_skipped_reason"],
                         "script_terminal_marker_seen")
        self.assertEqual(report["host_observation"]["frame_origin"],
                         "synthetic_grayscale_image")
        self.assertFalse(report["host_observation"]["live_sensor_capture"])
        self.assertFalse(report["host_observation"][
            "camera_fps_claimed_by_host"])
        self.assertTrue(saved["synthetic_scene_not_camera_capture"])
        self.assertEqual(saved["preview_source_kind"],
                         "synthetic_grayscale_marker_probe")

    def test_ram_completion_marker_skips_dead_stream_cleanup_without_claiming_duration(self):
        class FinishedFakeCamera(FakeCamera):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.completion_marker = ""

            def read_stdout(self):
                chunk = super().read_stdout()
                if self.executed and self.stdout_reads == 2:
                    self.stdout_chunks[-1] += self.completion_marker + "\r\n"
                    return self.stdout_chunks[-1]
                return chunk

        for marker in ("RAM_USB_PREVIEW_PROBE_FINISHED",
                       "RAM_BENCH_FINISHED"):
            with self.subTest(marker=marker), tempfile.TemporaryDirectory() as tmp:
                instance = None

                def factory(*args, **kwargs):
                    nonlocal instance
                    instance = FinishedFakeCamera(*args, **kwargs)
                    instance.completion_marker = marker
                    return instance

                report = bench.run_benchmark(
                    options_for("jpeg", Path(tmp), duration=1.0),
                    camera_factory=factory)
                self.assertEqual(report["status"], "completed")
                self.assertEqual(report["script_terminal_marker"], marker)
                self.assertEqual(report["observation_end_reason"],
                                 "script_terminal_marker")
                self.assertFalse(report["requested_duration_satisfied"])
                self.assertEqual(report["board_perf_window_count"], 1)
                self.assertEqual(report["host_observation"][
                    "host_sdk_frame_reads"], 0)
                self.assertFalse(report["stream_cleanup"]["disable_attempted"])
                self.assertEqual(instance.calls[-1][0], "disconnect")

    def test_startup_stdout_can_finish_after_short_transport_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                instance.first_stdout_delay_s = 0.22
                return instance

            report = bench.run_benchmark(
                options_for("off", Path(tmp), io_timeout=0.05,
                            stdout_timeout=0.5, startup_timeout=1.0),
                camera_factory=factory)
            self.assertEqual(report["status"], "completed")
            self.assertEqual(report["v7_build"]["fields"]["backend"], "fast")
            stdout_samples = report["host_observation"][
                "sdk_call_elapsed_ms"]["read_stdout"]
            self.assertGreaterEqual(stdout_samples["max_ms"], 200.0)
            self.assertLess(stdout_samples["max_ms"], 500.0)

    def test_post_exec_stdout_is_read_only_after_status_reports_ready(self):
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                instance.stdout_ready_after_status_polls = 4
                return instance

            report = bench.run_benchmark(
                options_for("off", Path(tmp), duration=0.005,
                            startup_timeout=1.0), camera_factory=factory)
            self.assertEqual(report["status"], "completed")
            calls = instance.calls
            exec_index = next(i for i, call in enumerate(calls)
                              if call[0] == "exec")
            first_stdout = next(i for i, call in enumerate(calls)
                                if i > exec_index and call[0] == "read_stdout")
            post_exec_statuses = [
                i for i, call in enumerate(calls)
                if exec_index < i < first_stdout and call[0] == "read_status"]
            self.assertEqual(len(post_exec_statuses), 4)
            self.assertEqual(report["stdout_read_policy"],
                             "read_status_stdout_ready_only")

    def test_startup_controls_use_longer_deadline_than_default_io_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                instance.stop_delay_s = 0.18
                instance.streaming_delay_s = 0.18
                return instance

            report = bench.run_benchmark(
                options_for("off", Path(tmp), io_timeout=0.15,
                            control_timeout=0.4, stop_timeout=0.4,
                            startup_timeout=1.5), camera_factory=factory)
            self.assertEqual(report["status"], "completed")
            samples = report["host_observation"]["sdk_call_elapsed_ms"]
            self.assertGreater(samples["stop"]["max_ms"], 150.0)
            self.assertLess(samples["stop"]["max_ms"], 400.0)
            self.assertGreater(samples["streaming"]["max_ms"], 150.0)
            self.assertLess(samples["streaming"]["max_ms"], 400.0)

    def test_startup_stream_control_still_has_a_hard_deadline(self):
        with tempfile.TemporaryDirectory() as tmp:
            instance = None

            def factory(*args, **kwargs):
                nonlocal instance
                instance = FakeCamera(*args, **kwargs)
                instance.streaming_delay_s = 0.2
                return instance

            report = bench.run_benchmark(
                options_for("off", Path(tmp), control_timeout=0.1,
                            startup_timeout=1.0), camera_factory=factory)
            self.assertEqual(report["status"], "error")
            self.assertIn("streaming exceeded 0.100 s deadline", report["error"])
            self.assertFalse(any(call[0] == "exec" for call in instance.calls))

    def test_ram_source_requires_fake_sink_and_rejects_production_uart_path(self):
        text, digest = bench.validate_ram_source(RAM_SOURCE)
        self.assertIn("LIVE_SENSOR_UART_SINK", text)
        self.assertEqual(len(digest), 64)
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "production.py"
            bad.write_text("import pyb\nuart = pyb.UART(3, 115200)\n",
                           encoding="utf-8")
            with self.assertRaisesRegex(bench.BenchmarkError,
                                       "must use the LIVE_SENSOR_UART_SINK"):
                bench.validate_ram_source(bad)

            suspicious = Path(tmp) / "fake_with_hardware.py"
            suspicious.write_text(
                "class _Sink:\n"
                " def write(self, packet): return len(packet)\n"
                "class _Source: pass\n"
                "main._main_fast(frame_source=_Source(), "
                "uart_sink=_Sink())\n"
                "LIVE_SENSOR_UART_SINK\n"
                "from pyb import UART\n"
                "UART(3, 115200)\n",
                encoding="utf-8")
            with self.assertRaisesRegex(bench.BenchmarkError,
                                       "must not construct UART or Pin"):
                bench.validate_ram_source(suspicious)

    def test_mock_ram_fixture_is_in_repo_and_has_no_logs_or_tmp_dependency(self):
        self.assertTrue(RAM_SOURCE.is_file())
        text, _digest = bench.validate_ram_source(RAM_SOURCE)
        self.assertIn("LIVE_SENSOR_UART_SINK", text)
        self.assertNotIn("production-default-final-ram-source", str(RAM_SOURCE))
        self.assertNotIn("/logs/", str(RAM_SOURCE).replace("\\", "/"))
        self.assertNotIn("/tmp/", str(RAM_SOURCE).replace("\\", "/"))

    def test_v7_window_parser_keeps_board_values_and_stage_lines(self):
        stdout = (
            "noise\r\n"
            "V7_PERF_WINDOW run=2 index=3 start_ms=30000 end_ms=40000 "
            "distinct_captures=610 valid_frames=609\r\n"
            "V7_PERF_STAGE run=2 index=3 total_cycle:n=610,avg=16000,"
            "p95_upper=18000,max=19000\r\n")
        parsed = bench.parse_v7_perf_windows(stdout)
        self.assertEqual(len(parsed["windows"]), 1)
        self.assertEqual(parsed["windows"][0]["fields"]["run"], 2)
        self.assertEqual(parsed["windows"][0]["fields"]["valid_frames"], 609)
        self.assertEqual(parsed["windows"][0]["stages"][0]["summary"],
                         "total_cycle:n=610,avg=16000,p95_upper=18000,max=19000")

    def test_deadline_closes_call_without_waiting_for_sdk_loop_forever(self):
        timed_out = threading.Event()
        late_cleanup = threading.Event()

        def slow_call():
            time.sleep(0.12)
            return "late"

        started = time.monotonic()
        with self.assertRaises(bench.DeviceCallTimeout):
            bench.call_with_deadline(
                slow_call, 0.02, on_timeout=timed_out.set,
                late_cleanup=late_cleanup.set, operation="mock read")
        self.assertLess(time.monotonic() - started, 0.08)
        self.assertTrue(timed_out.is_set())
        self.assertTrue(late_cleanup.wait(0.5))


if __name__ == "__main__":
    unittest.main()
