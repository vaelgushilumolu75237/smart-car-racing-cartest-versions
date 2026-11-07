#!/usr/bin/env python3
"""Bounded OpenMV IDE-preview transport benchmark.

This tool only runs after an explicit ``--run``. It executes a prebuilt RAM
fixture through the OpenMV SDK stdin channel, then measures one selected stream
mode: disabled, JPEG, or raw 320x120. It never writes a file to the camera and
requires the RAM fixture to pass a sink to ``_main_fast`` so it cannot drive
the H743 UART path. The SDK's serial, Pillow, and NumPy dependencies can be
provided with ``uv run --with pyserial --with pillow --with numpy``.

Host frame hashes and SDK read timings describe this PC's view of the stream;
they are not sensor capture counts or a camera FPS measurement. Use the exact
``V7_PERF_WINDOW`` records emitted by the board for board-side throughput.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import queue
import re
import sys
import threading
import time
import uuid
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

try:  # Package import in tests; sibling import when run as a script.
    from .openmv_preview_marker import PreviewMarkerTracker
except ImportError:
    from openmv_preview_marker import PreviewMarkerTracker


RAW_STREAM_RESOLUTION = (320, 120)
MAX_RAM_SOURCE_BYTES = 1_000_000
DEFAULT_IO_TIMEOUT_S = 0.15
DEFAULT_STDOUT_TIMEOUT_S = 5.0
DEFAULT_FRAME_TIMEOUT_S = 2.0
DEFAULT_DURATION_S = 35.0
DEFAULT_STARTUP_TIMEOUT_S = 15.0
DEFAULT_STOP_TIMEOUT_S = 5.0
DEFAULT_CONTROL_TIMEOUT_S = 2.0
DEFAULT_STARTUP_DRAIN_TIMEOUT_S = 0.5
DEFAULT_POLL_MS = 5.0
DEFAULT_FAST_SDK_POLL = True


class BenchmarkError(RuntimeError):
    """A preflight, SDK, or bounded-operation failure."""


class DeviceCallTimeout(BenchmarkError):
    """A single SDK operation exceeded its caller-owned deadline."""


def _finite_range(value, label, low, high):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % label)
    if not math.isfinite(numeric) or numeric < low or numeric > high:
        raise ValueError("%s must be in %.3g..%.3g" % (label, low, high))
    return numeric


def build_parser():
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=("Measure one OpenMV preview mode using a RAM-only V7 "
                     "fixture. No device is opened without --run."))
    parser.add_argument("--port", required=True,
                        help="OpenMV USB serial port, for example COM5")
    parser.add_argument("--ram-source", required=True, type=Path,
                        help="Existing H743-isolated RAM fixture source")
    parser.add_argument("--run", action="store_true",
                        help="Required explicit opt-in to connect and execute")
    parser.add_argument("--mode", required=True,
                        choices=("off", "jpeg", "raw"),
                        help="Run exactly one stream mode for this invocation")
    parser.add_argument("--ide-state", choices=("open", "closed", "unspecified"),
                        default="unspecified",
                        help="Operator-declared IDE state; never detected or changed")
    parser.add_argument("--sdk-path", type=Path,
                        default=repo_root / "tmp" / "openmv-sdk",
                        help="Local OpenMV SDK checkout (default: tmp/openmv-sdk)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help=("Board observation duration after V7_BUILD "
                              "(default 35 s covers the 30 s RAM fixture; 1..600 s)"))
    parser.add_argument("--startup-timeout", type=float,
                        default=DEFAULT_STARTUP_TIMEOUT_S,
                        help="Maximum time to connect, execute, and see V7_BUILD (1..120 s)")
    parser.add_argument("--stop-timeout", type=float,
                        default=DEFAULT_STOP_TIMEOUT_S,
                        help=("Hard deadline for stopping the currently running "
                              "script after connect (0.1..15 s; startup remains bounded)"))
    parser.add_argument("--control-timeout", type=float,
                        default=DEFAULT_CONTROL_TIMEOUT_S,
                        help=("Hard deadline for SDK streaming control calls "
                              "(0.1..10 s); status polls still use --io-timeout"))
    parser.add_argument("--startup-drain-timeout", type=float,
                        default=DEFAULT_STARTUP_DRAIN_TIMEOUT_S,
                        help=("Maximum time to drain old stdout after stop and "
                              "before exec (0..5 s; startup remains bounded)"))
    parser.add_argument("--io-timeout", type=float,
                        default=DEFAULT_IO_TIMEOUT_S,
                        help=("SDK transport timeout per receive for non-stdout "
                              "calls (0.01..2 s)"))
    parser.add_argument("--fast-sdk-poll", action=argparse.BooleanOptionalAction,
                        default=DEFAULT_FAST_SDK_POLL,
                        help=("Use the known scheduler-yield poll shim for the "
                              "local SDK transport (default on; disable for the "
                              "SDK's fixed 1 ms sleep)"))
    parser.add_argument("--stdout-timeout", type=float,
                        default=DEFAULT_STDOUT_TIMEOUT_S,
                        help=("Hard deadline and receive timeout for one complete "
                              "stdout read (0.1..30 s; startup still uses its "
                              "remaining global budget)"))
    parser.add_argument("--frame-timeout", type=float,
                        default=DEFAULT_FRAME_TIMEOUT_S,
                        help=("Hard caller deadline for a whole frame read; "
                              "serial receives still use --io-timeout (0.1..30 s)"))
    parser.add_argument("--crc", action=argparse.BooleanOptionalAction,
                        default=True, help="Request SDK packet CRC (default on)")
    parser.add_argument("--seq", action=argparse.BooleanOptionalAction,
                        default=True, help="Request sequence validation (default on)")
    parser.add_argument("--ack", action=argparse.BooleanOptionalAction,
                        default=True, help="Request packet acknowledgments (default on)")
    parser.add_argument("--events", action=argparse.BooleanOptionalAction,
                        default=True, help="Request protocol events (default on)")
    parser.add_argument("--max-payload", type=int, default=4096,
                        help="Requested protocol payload limit in bytes (52..65535)")
    parser.add_argument("--poll-ms", type=float, default=DEFAULT_POLL_MS,
                        help="Target host poll-start interval (1..500 ms)")
    parser.add_argument("--marker-sequences", action="store_true",
                        help=("Decode the RAM probe's D3/seq16/CRC8 marker "
                              "from each returned RGB888 frame"))
    parser.add_argument("--synthetic-marker-probe", action="store_true",
                        help=("Explicitly allow the bounded synthetic-image "
                              "marker fixture (no sensor capture); requires "
                              "--mode jpeg and --marker-sequences"))
    parser.add_argument("--output-dir", type=Path,
                        default=repo_root / "logs" / "openmv-preview",
                        help="Parent directory for host-side report artifacts")
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.run:
        parser.error("device access is disabled unless --run is supplied")
    try:
        args.duration = _finite_range(args.duration, "--duration", 1.0, 600.0)
        args.startup_timeout = _finite_range(
            args.startup_timeout, "--startup-timeout", 1.0, 120.0)
        args.stop_timeout = _finite_range(
            args.stop_timeout, "--stop-timeout", 0.1, 15.0)
        args.control_timeout = _finite_range(
            args.control_timeout, "--control-timeout", 0.1, 10.0)
        args.startup_drain_timeout = _finite_range(
            args.startup_drain_timeout, "--startup-drain-timeout", 0.0, 5.0)
        args.io_timeout = _finite_range(
            args.io_timeout, "--io-timeout", 0.01, 2.0)
        args.stdout_timeout = _finite_range(
            args.stdout_timeout, "--stdout-timeout", 0.1, 30.0)
        args.frame_timeout = _finite_range(
            args.frame_timeout, "--frame-timeout", 0.1, 30.0)
        args.poll_ms = _finite_range(args.poll_ms, "--poll-ms", 1.0, 500.0)
    except ValueError as exc:
        parser.error(str(exc))
    if not 52 <= args.max_payload <= 65535:
        parser.error("--max-payload must be in 52..65535")
    if args.synthetic_marker_probe:
        if args.mode != "jpeg":
            parser.error("--synthetic-marker-probe requires --mode jpeg")
        if not args.marker_sequences:
            parser.error("--synthetic-marker-probe requires --marker-sequences")
    return args


def _validate_synthetic_marker_source(source):
    """Require the bounded synthetic marker fixture and reject device I/O."""
    required = (
        "SYNTHETIC_MARKER_PROBE = True",
        "V7_BUILD=synthetic_marker_probe",
        "V7_SYNTHETIC_MARKER_COUNTS",
        "V7_SYNTHETIC_MARKER_DONE",
        "@micropython.viper",
        "def _write_preview_marker_viper(raw,sequence:int)->int:",
        "SYNTHETIC_FRAME_WIDTH = 320",
        "SYNTHETIC_FRAME_HEIGHT = 120",
        "SYNTHETIC_SEQUENCE_FIRST = 65532",
        "SYNTHETIC_SEQUENCE_COUNT = 25",
        "SYNTHETIC_REPEAT_COUNT = 3",
        "SYNTHETIC_PUBLISH_DELAY_MS = 20",
        "SYNTHETIC_DEADLINE_MS = 4500",
        "SYNTHETIC_STDOUT_DRAIN_MS = 2000",
        "image.Image(SYNTHETIC_FRAME_WIDTH,SYNTHETIC_FRAME_HEIGHT,",
        "image.GRAYSCALE",
        "_frame.to_jpeg(x_scale=1.0,y_scale=1.0,quality=30,copy=True)",
        "_jpeg.flush()",
        "time.sleep_ms(SYNTHETIC_PUBLISH_DELAY_MS)",
        "time.sleep_ms(SYNTHETIC_STDOUT_DRAIN_MS)",
    )
    missing = [fragment for fragment in required if fragment not in source]
    if missing:
        raise BenchmarkError(
            "synthetic marker source is missing required fixture markers: %s" %
            ", ".join(missing))
    if not re.search(
            r"(?m)^import\s+sys,time,gc,image,micropython\s*$",
            source):
        raise BenchmarkError(
            "synthetic marker source must use the restricted fixture imports")
    import_lines = [line.strip() for line in source.splitlines()
                    if re.match(r"^\s*(?:import|from)\s+", line)]
    if import_lines != ["import sys,time,gc,image,micropython"]:
        raise BenchmarkError(
            "synthetic marker source must use the restricted fixture imports")
    forbidden = (
        (r"\b(?:sensor|pyb|machine|UART|Pin|open|os|vfs|sdcard|uos)\b|"
         r"\b(?:__import__|exec|eval)\s*\(",
         "sensor, hardware, dynamic import/execute, or storage access"),
        (r"\b(?:main\s*\.\s*)?_main_fast\s*\(",
         "production _main_fast execution"),
        (r"\b(?:pyb|machine)\s*\.\s*(?:UART|Pin)\s*\(|"
         r"\b(?:UART|Pin)\s*\(", "UART or Pin hardware access"),
        (r"\bopen\s*\(|\b(?:os|vfs|sdcard)\s*\.",
         "camera storage access"),
        (r"\b(?:uart_sink|LIVE_SENSOR_UART_SINK)\b",
         "live sensor/UART fixture content"),
    )
    for pattern, description in forbidden:
        if re.search(pattern, source):
            raise BenchmarkError(
                "synthetic marker source must not contain %s" % description)


def validate_ram_source(source_path, synthetic_marker_probe=False):
    """Load an existing RAM fixture and verify its injected no-UART sink.

    V7's production entry point can open UART3. This benchmark accepts only a
    fixture that calls ``_main_fast`` with both an injected frame source and a
    fake UART sink, keeping the H743 outside this tool's execution path.
    """
    path = Path(source_path).expanduser().resolve()
    if not path.is_file():
        raise BenchmarkError("RAM source file does not exist: %s" % path)
    raw = path.read_bytes()
    if not raw or len(raw) > MAX_RAM_SOURCE_BYTES:
        raise BenchmarkError("RAM source must be 1..%d bytes" %
                             MAX_RAM_SOURCE_BYTES)
    try:
        source = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise BenchmarkError("RAM source must be UTF-8 text: %s" % exc)

    if synthetic_marker_probe:
        _validate_synthetic_marker_source(source)
        return source, hashlib.sha256(raw).hexdigest().upper()

    has_fixture_call = re.search(
        r"\._main_fast\([\s\S]*?\bframe_source\s*=\s*_[A-Za-z0-9_]+\(\)"
        r"[\s\S]*?\buart_sink\s*=\s*_[A-Za-z0-9_]+\(\)", source)
    has_fake_sink = re.search(
        r"class\s+_Sink\b[\s\S]*?def\s+write\s*\([^)]*\)\s*:"
        r"\s*return\s+len\(packet\)", source)
    if (not has_fixture_call or not has_fake_sink or
            "LIVE_SENSOR_UART_SINK" not in source):
        raise BenchmarkError(
            "RAM source must use the LIVE_SENSOR_UART_SINK fixture and pass "
            "a no-op _Sink to _main_fast(frame_source=..., uart_sink=...)"
        )
    if re.search(
            r"\b(?:pyb|machine)\s*\.\s*(?:UART|Pin)\s*\(|"
            r"\b(?:UART|Pin)\s*\(", source):
        raise BenchmarkError(
            "RAM source must not construct UART or Pin hardware objects")
    return source, hashlib.sha256(raw).hexdigest().upper()


def load_camera_factory(sdk_path, fast_sdk_poll=DEFAULT_FAST_SDK_POLL):
    sdk = Path(sdk_path).expanduser().resolve()
    if not (sdk / "openmv" / "camera.py").is_file():
        raise BenchmarkError("OpenMV SDK not found under --sdk-path: %s" % sdk)
    sdk_string = str(sdk)
    if sdk_string in sys.path:
        sys.path.remove(sdk_string)
    sys.path.insert(0, sdk_string)
    try:
        from openmv.camera import Camera
        import openmv.transport as transport
    except Exception as exc:
        raise BenchmarkError("could not import Camera from SDK: %s" % exc)
    if fast_sdk_poll:
        # Match the established local runner's cooperative yield. Keep this
        # namespace private to openmv.transport so the benchmark's own
        # monotonic timing and poll sleeps are unchanged.
        transport.time = SimpleNamespace(
            time=time.time, sleep=lambda _seconds: time.sleep(0))
    else:
        transport.time = time
    return Camera


def call_with_deadline(function, timeout_s, on_timeout=None,
                       late_cleanup=None, operation="SDK call"):
    """Run one SDK call with a hard caller deadline and bounded disconnect.

    The local SDK can extend its internal receive wait when fragments arrive.
    A daemon worker prevents that receive loop from blocking this tool forever;
    timeout closes the USB serial connection and the worker closes it again if
    a delayed call later returns.
    """
    timeout_s = _finite_range(timeout_s, "operation timeout", 0.001, 3600.0)
    outcome = queue.Queue(maxsize=1)
    finished = threading.Event()
    cancelled = threading.Event()
    def worker():
        try:
            outcome.put((True, function()))
        except BaseException as exc:  # Forward even SDK/SystemExit failures.
            outcome.put((False, exc))
        finally:
            finished.set()
            if cancelled.is_set() and late_cleanup is not None:
                try:
                    late_cleanup()
                except Exception:
                    pass

    thread = threading.Thread(target=worker,
                              name="openmv-sdk-bounded-call", daemon=True)
    started = time.monotonic()
    thread.start()
    if not finished.wait(timeout_s):
        cancelled.set()
        if on_timeout is not None:
            try:
                on_timeout()
            except Exception:
                pass
        raise DeviceCallTimeout(
            "%s exceeded %.3f s deadline" % (operation, timeout_s))

    ok, value = outcome.get_nowait()
    elapsed_s = time.monotonic() - started
    if not ok:
        raise value
    return value, elapsed_s


def _disconnect_quietly(camera):
    try:
        camera.disconnect()
    except Exception:
        pass


def _set_transport_timeout(camera, timeout_s):
    """Set logical SDK wait budgets without reconfiguring the open serial port.

    pyserial's ``Serial.timeout`` is fixed when ``Camera.connect()`` opens the
    port. Reassigning it can call SetCommState and toggle CDC line state on
    Windows. ``call_with_deadline`` still bounds each operation and closes the
    port on timeout; Camera/Transport timeouts only control their receive loop.
    """
    for obj, attribute in ((camera, "timeout"),
                           (getattr(camera, "transport", None), "timeout")):
        if obj is not None and hasattr(obj, attribute):
            try:
                if getattr(obj, attribute) != timeout_s:
                    setattr(obj, attribute, timeout_s)
            except Exception:
                pass


def _bounded_camera_call(camera, method_name, args, hard_timeout_s,
                         transport_timeout_s, call_samples):
    hard_timeout_s = max(0.001, float(hard_timeout_s))
    _set_transport_timeout(camera, min(hard_timeout_s, transport_timeout_s))
    method = getattr(camera, method_name)
    try:
        result, elapsed_s = call_with_deadline(
            lambda: method(*args), hard_timeout_s,
            on_timeout=lambda: _disconnect_quietly(camera),
            late_cleanup=lambda: _disconnect_quietly(camera),
            operation=method_name)
    except SystemExit as exc:
        # Some OpenMV SDK command paths catch a transport exception and call
        # sys.exit(0). Treat that as a failed bounded SDK operation so the
        # benchmark can persist its report and return nonzero. KeyboardInterrupt
        # remains distinct and continues to the run-level interrupt handler.
        raise BenchmarkError(
            "OpenMV SDK %s raised SystemExit(code=%r)" %
            (method_name, exc.code)) from exc
    call_samples[method_name].append(elapsed_s * 1000.0)
    return result


def _summary(values):
    if not values:
        return {"count": 0, "total_ms": 0.0, "mean_ms": None,
                "p95_ms": None, "max_ms": None}
    ordered = sorted(values)
    p95_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    return {
        "count": len(values),
        "total_ms": round(sum(values), 3),
        "mean_ms": round(sum(values) / len(values), 3),
        "p95_ms": round(ordered[p95_index], 3),
        "max_ms": round(max(values), 3),
    }


def _json_safe(value):
    """Copy common SDK state into JSON-safe primitives without calling it."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return repr(value)


def _parse_key_values(text):
    fields = {}
    for token in text.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if re.fullmatch(r"-?\d+", value):
            fields[key] = int(value)
        else:
            fields[key] = value
    return fields


def parse_v7_perf_windows(stdout_text):
    windows = []
    stages = defaultdict(list)
    raw_lines = []
    for line in stdout_text.splitlines():
        if line.startswith("V7_PERF_WINDOW "):
            raw_lines.append(line)
            fields = _parse_key_values(line[len("V7_PERF_WINDOW "):])
            windows.append({"fields": fields, "raw_line": line, "stages": []})
        elif line.startswith("V7_PERF_STAGE "):
            raw_lines.append(line)
            payload = line[len("V7_PERF_STAGE "):]
            match = re.match(r"run=(\d+)\s+index=(\d+)\s+(.*)", payload)
            if match:
                key = (int(match.group(1)), int(match.group(2)))
                stages[key].append({
                    "summary": match.group(3),
                    "raw_line": line,
                })
    for window in windows:
        fields = window["fields"]
        key = (fields.get("run"), fields.get("index"))
        window["stages"] = stages.get(key, [])
    return {"schema": "openmv-v7-perf-windows-v1",
            "windows": windows, "raw_perf_lines": raw_lines}


def summarize_board_counts(perf_windows):
    """Expose board-emitted counters without deriving host-side frame rates."""
    count_fields = (
        "run", "index", "start_ms", "end_ms", "distinct_captures",
        "write_attempts", "full_uart_writes", "valid_frames",
        "valid_full_writes", "attempted_by_loop", "uart_write_errors",
    )
    return [
        {key: window["fields"][key] for key in count_fields
         if key in window["fields"]}
        for window in perf_windows["windows"]
    ]


def _build_marker(stdout_text):
    for line in stdout_text.splitlines():
        if line.startswith("V7_BUILD="):
            payload = line[len("V7_BUILD="):]
            return {"raw_line": line, "fields": _parse_key_values(payload)}
    return None


def _synthetic_marker_counts(stdout_text):
    prefix = "V7_SYNTHETIC_MARKER_COUNTS "
    for line in stdout_text.splitlines():
        if line.startswith(prefix):
            return {"raw_line": line,
                    "fields": _parse_key_values(line[len(prefix):])}
    return None


def parse_board_execution_errors(stdout_text):
    """Find board tracebacks and explicit ERROR/FATAL lines in raw stdout."""
    lines = stdout_text.splitlines()
    errors = []
    traceback_end_re = re.compile(
        r"^(?:[A-Za-z_][A-Za-z0-9_.]*(?:Error|Exception|Interrupt|Exit))"
        r"(?::|\s*$)")
    # Only classify explicit status tokens at line start. In particular,
    # fields such as ``reference_error=+139`` are valid V7 diagnostics and
    # must not turn a successful run into an execution failure.
    explicit_error_re = re.compile(
        r"(?i)^\s*(?:V7_)?(?:ERROR|FATAL)(?=$|[\s:=,\]\)])")
    index = 0
    while index < len(lines):
        line = lines[index]
        if line.strip() == "Traceback (most recent call last):":
            excerpt = [line]
            cursor = index + 1
            while cursor < len(lines) and len(excerpt) < 12:
                excerpt.append(lines[cursor])
                if traceback_end_re.match(lines[cursor].strip()):
                    break
                if (lines[cursor].startswith("V7_") or
                        lines[cursor].startswith("RAM_")):
                    excerpt.pop()
                    break
                cursor += 1
            errors.append({
                "kind": "traceback",
                "raw_excerpt": "\n".join(excerpt),
                "line_number": index + 1,
            })
            index = max(index + 1, cursor + 1)
            continue
        if explicit_error_re.search(line):
            errors.append({
                "kind": "explicit_error",
                "raw_excerpt": line,
                "line_number": index + 1,
            })
        index += 1
    return errors


def summarize_marker_arrival_windows(arrivals, window_s=10.0):
    """Bin valid forward marker arrivals from first-marker host time t0.

    A bin is complete only when the last valid forward marker in the entire
    observation reaches or passes that bin's end. Partial bins retain an
    observed rate for diagnosis, but expose no full-window rate.
    """
    if not arrivals:
        return {
            "status": "no_valid_marker_arrivals",
            "measurement_success": False,
            "window_s": float(window_s),
            "t0_host_monotonic_s": None,
            "t0_run_elapsed_s": None,
            "last_marker_elapsed_from_t0_s": None,
            "new_sequence_arrivals": 0,
            "complete_window_count": 0,
            "windows": [],
        }

    window_s = float(window_s)
    t0_host = float(arrivals[0]["host_monotonic_s"])
    t0_elapsed = float(arrivals[0]["run_elapsed_s"])
    relative = [max(0.0, float(item["host_monotonic_s"]) - t0_host)
                for item in arrivals]
    last_relative = relative[-1]
    bin_count = int(math.floor(last_relative / window_s)) + 1
    windows = []
    for index in range(bin_count):
        start = index * window_s
        end = start + window_s
        fresh_count = sum(1 for value in relative
                          if start <= value < end)
        complete = last_relative >= end
        observed_span = max(0.0, min(last_relative, end) - start)
        observed_rate = (round(fresh_count / observed_span, 3)
                         if observed_span > 0 else None)
        windows.append({
            "index": index,
            "start_s_from_t0": round(start, 6),
            "end_s_from_t0": round(end, 6),
            "fresh_count": fresh_count,
            "rate_hz": round(fresh_count / window_s, 3)
                       if complete else None,
            "partial_observed_rate_hz": (None if complete else
                                          observed_rate),
            "complete": complete,
            "last_marker_s_from_t0": round(last_relative, 6),
        })
    complete_count = sum(1 for item in windows if item["complete"])
    return {
        "status": "complete_windows_available" if complete_count else
                  "incomplete_observation_window",
        "measurement_success": complete_count > 0,
        "window_s": window_s,
        "t0_host_monotonic_s": round(t0_host, 6),
        "t0_run_elapsed_s": round(t0_elapsed, 6),
        "last_marker_elapsed_from_t0_s": round(last_relative, 6),
        "new_sequence_arrivals": len(arrivals),
        "complete_window_count": complete_count,
        "windows": windows,
    }


def _host_report(mode, poll_ms, poll_starts, loop_work_ms, sleep_ms,
                 calls, frame_reads, frame_hashes, frame_metadata,
                 hash_elapsed_ms, observation_elapsed_s,
                 marker_observation=None, marker_windows=None):
    intervals = [max(0.0, (right - left) * 1000.0)
                 for left, right in zip(poll_starts, poll_starts[1:])]
    unique_hashes = sorted(frame_hashes)
    return {
        "mode": mode,
        "requested_poll_start_interval_ms": poll_ms,
        "poll_iterations": len(poll_starts),
        "observation_elapsed_s": round(observation_elapsed_s, 3),
        "observed_poll_start_interval_ms": _summary(intervals),
        "host_loop_work_ms": _summary(loop_work_ms),
        "host_loop_work_fraction": round(
            sum(loop_work_ms) / (observation_elapsed_s * 1000.0), 4)
            if observation_elapsed_s > 0 else None,
        "host_sleep_total_ms": round(sleep_ms, 3),
        "sdk_call_elapsed_ms": {
            name: _summary(samples) for name, samples in sorted(calls.items())
        },
        "host_sdk_frame_reads": frame_reads,
        "host_unique_content_hash_count": len(unique_hashes),
        "host_duplicate_content_hash_reads": max(
            0, frame_reads - len(unique_hashes)),
        "host_unique_content_sha256": unique_hashes,
        "host_marker_sequence_observation": marker_observation,
        "host_marker_sequence_windows": marker_windows,
        "host_frame_returns": [
            {"width": width, "height": height, "format": pixformat,
             "depth": depth, "raw_size_bytes": raw_size,
             "rgb888_size_bytes": rgb888_size, "reads": count}
            for (width, height, pixformat, depth, raw_size, rgb888_size), count
            in sorted(frame_metadata.items(), key=lambda item: repr(item[0]))
        ],
        "hash_processing_elapsed_ms": _summary(hash_elapsed_ms),
        "camera_fps_claimed_by_host": False,
        "frame_metric_note": (
            "SDK frame returns and content-hash differences are host-side "
            "observations only. A unique content hash means image content "
            "changed; it does not identify a fresh sensor frame. When enabled, "
            "fresh-frame reads are counted only from valid forward marker "
            "sequence transitions. Repeated or changing hashes are not a sensor "
            "capture count or camera frame rate, and marker reads are not "
            "reported as camera FPS. Use board V7_PERF_WINDOW records for "
            "board-side counts and throughput."
        ),
    }


def _session_directory(parent):
    parent = Path(parent).expanduser().resolve()
    parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    for attempt in range(20):
        suffix = "" if attempt == 0 else "-%s" % uuid.uuid4().hex[:6]
        target = parent / (stamp + suffix)
        try:
            target.mkdir()
            return target
        except FileExistsError:
            continue
    raise BenchmarkError("could not allocate a unique report directory")


def run_benchmark(options, camera_factory=None):
    """Run one preview mode and persist raw stdout and parsed board windows."""
    synthetic_probe = bool(getattr(options, "synthetic_marker_probe", False))
    source, source_sha256 = validate_ram_source(
        options.ram_source, synthetic_marker_probe=synthetic_probe)
    fast_sdk_poll_requested = bool(getattr(
        options, "fast_sdk_poll", DEFAULT_FAST_SDK_POLL))
    fast_sdk_poll_applied = False
    if camera_factory is None:
        camera_factory = load_camera_factory(
            options.sdk_path, fast_sdk_poll=fast_sdk_poll_requested)
        fast_sdk_poll_applied = fast_sdk_poll_requested
    report_dir = _session_directory(options.output_dir)
    stdout_chunks = []
    pre_exec_stdout_chunks = []
    pre_exec_stdout_read_calls = 0
    pre_exec_status_polls = 0
    pre_exec_drain_complete = False
    pre_exec_drain_elapsed_ms = 0.0
    camera = None
    connected = False
    camera_usable = False
    stream_enabled = False
    stream_cleanup_attempted = False
    stream_cleanup_skipped_reason = None
    script_terminal_marker_seen = False
    script_terminal_marker = None
    status = "error"
    error = None
    build = None
    calls = defaultdict(list)
    poll_starts = []
    loop_work_ms = []
    frame_hashes = set()
    frame_metadata = defaultdict(int)
    frame_reads = 0
    hash_elapsed_ms = []
    marker_arrivals = []
    marker_tracker = (PreviewMarkerTracker()
                      if getattr(options, "marker_sequences", False) else None)
    sleep_total_ms = 0.0
    observation_elapsed_s = 0.0
    run_started = None
    transport_stats = None
    start_clock = time.monotonic
    sleep = time.sleep
    control_timeout = getattr(options, "control_timeout",
                              DEFAULT_CONTROL_TIMEOUT_S)

    report = {
        "schema": "openmv-preview-benchmark-v1",
        "port": options.port,
        "mode": options.mode,
        "operator_declared_ide_state": options.ide_state,
        "sdk_path": str(Path(options.sdk_path).resolve()),
        "ram_source": str(Path(options.ram_source).resolve()),
        "ram_source_sha256": source_sha256,
        "ram_execution": (
            "OpenMV SDK stdin exec; synthetic image only; no camera file write"
            if synthetic_probe else
            "OpenMV SDK stdin exec; no camera file write"),
        "preview_source_kind": ("synthetic_grayscale_marker_probe"
                                if synthetic_probe else "ram_v7_fixture"),
        "synthetic_scene_not_camera_capture": synthetic_probe,
        "synthetic_marker_probe_requested": synthetic_probe,
        "stream_enable_order": (
            "immediately_after_exec_before_v7_build_wait"
            if options.mode != "off" else "not_enabled"),
        "h743_uart_fixture": (
            "not applicable; synthetic probe bypasses production V7/H743 path"
            if synthetic_probe else
            "_main_fast(frame_source=..., uart_sink=_Sink())"),
        "raw_stream_resolution": list(RAW_STREAM_RESOLUTION)
        if options.mode == "raw" else None,
        "sensor_resolution_modified_by_tool": False,
        "raw_resolution_scope": (
            "synthetic 320x120 image; sensor is not accessed"
            if synthetic_probe else
            "SDK stream output only; sensor framesize unchanged"),
        "duration_requested_s": options.duration,
        "startup_timeout_s": options.startup_timeout,
        "per_call_io_timeout_s": options.io_timeout,
        "fast_sdk_poll_requested": fast_sdk_poll_requested,
        "fast_sdk_poll_applied": fast_sdk_poll_applied,
        "stdout_read_policy": "read_status_stdout_ready_only",
        "stop_timeout_s": getattr(options, "stop_timeout",
                                   DEFAULT_STOP_TIMEOUT_S),
        "control_timeout_s": getattr(options, "control_timeout",
                                      DEFAULT_CONTROL_TIMEOUT_S),
        "startup_drain_timeout_s": getattr(
            options, "startup_drain_timeout", DEFAULT_STARTUP_DRAIN_TIMEOUT_S),
        "whole_stdout_read_timeout_s": options.stdout_timeout,
        "whole_frame_read_timeout_s": options.frame_timeout,
        "requested_observation_duration_s": options.duration,
        "requested_poll_start_interval_ms": options.poll_ms,
        "marker_sequences_requested": marker_tracker is not None,
        "requested_protocol_caps": {
            "crc": options.crc,
            "seq": options.seq,
            "ack": options.ack,
            "events": options.events,
            "max_payload": options.max_payload,
        },
        "status": status,
    }

    def stdout_read(timeout_s, destination=None):
        nonlocal script_terminal_marker_seen, script_terminal_marker
        if destination is None:
            destination = stdout_chunks
        chunk = _bounded_camera_call(
            camera, "read_stdout", (), timeout_s,
            min(timeout_s, options.stdout_timeout), calls)
        if chunk:
            if not isinstance(chunk, str):
                chunk = str(chunk)
            destination.append(chunk)
            if destination is stdout_chunks:
                for marker in ("V7_SYNTHETIC_MARKER_DONE status=",
                               "RAM_USB_PREVIEW_PROBE_FINISHED",
                               "RAM_BENCH_FINISHED"):
                    if marker in chunk:
                        script_terminal_marker_seen = True
                        script_terminal_marker = marker
                        break
        return chunk or ""

    def status_read(timeout_s):
        return _bounded_camera_call(
            camera, "read_status", (), timeout_s,
            min(timeout_s, options.io_timeout), calls)

    def remaining_call_budget(deadline, per_call_timeout=None):
        if per_call_timeout is None:
            per_call_timeout = options.io_timeout
        return max(0.001, min(per_call_timeout, deadline - start_clock()))

    try:
        camera = camera_factory(
            options.port, timeout=options.io_timeout, max_retry=1,
            crc=options.crc, seq=options.seq, ack=options.ack,
            events=options.events, max_payload=options.max_payload)
        startup_deadline = start_clock() + options.startup_timeout

        def startup_budget():
            remaining = startup_deadline - start_clock()
            if remaining <= 0:
                raise BenchmarkError("startup deadline expired")
            return remaining

        _bounded_camera_call(
            camera, "connect", (), startup_budget(),
            options.io_timeout, calls)
        connected = True
        camera_usable = True

        # Stop an old resident program immediately after connecting. Use its
        # own bounded budget; default serial read deadlines are too short for
        # control commands on some IDE/firmware states.
        stop_budget = min(getattr(options, "stop_timeout",
                                  DEFAULT_STOP_TIMEOUT_S), startup_budget())
        _bounded_camera_call(camera, "stop", (), stop_budget,
                             stop_budget, calls)
        report["negotiated_protocol_caps"] = _json_safe(
            getattr(camera, "caps", None))

        # Then disable preview, also using the dedicated control deadline.
        stream_off_budget = min(control_timeout, startup_budget())
        _bounded_camera_call(camera, "streaming", (False,),
                             stream_off_budget, stream_off_budget, calls)

        # Discard output left by the stopped script so an old V7_BUILD cannot
        # satisfy this fixture's startup check. The SDK reports stdout
        # readiness through read_status; do not issue CHANNEL_SIZE when empty.
        drain_budget = min(
            getattr(options, "startup_drain_timeout",
                    DEFAULT_STARTUP_DRAIN_TIMEOUT_S), startup_budget())
        drain_started = start_clock()
        drain_deadline = start_clock() + drain_budget
        while start_clock() < drain_deadline:
            remaining = drain_deadline - start_clock()
            pre_exec_status_polls += 1
            pre_exec_status = status_read(min(options.io_timeout, remaining))
            if not (isinstance(pre_exec_status, dict) and
                    pre_exec_status.get("stdout")):
                pre_exec_drain_complete = True
                break
            remaining = drain_deadline - start_clock()
            if remaining <= 0:
                break
            pre_exec_stdout_read_calls += 1
            stale_chunk = stdout_read(min(options.stdout_timeout, remaining),
                                      pre_exec_stdout_chunks)
            if not stale_chunk:
                pre_exec_drain_complete = True
                break
        pre_exec_drain_elapsed_ms = (start_clock() - drain_started) * 1000.0

        _bounded_camera_call(camera, "exec", (source,), startup_budget(),
                             options.io_timeout, calls)

        # The OpenMV SDK CLI starts the stream immediately after stdin exec.
        # Wait for V7_BUILD only after this control call: a RAM script can
        # begin publishing JPEGs as soon as it prints its build marker, and
        # delaying STREAM_CTRL until then can race the running script's
        # flush/publication work.
        if options.mode == "jpeg":
            _bounded_camera_call(camera, "streaming", (True,), control_timeout,
                                 control_timeout, calls)
            stream_enabled = True
        elif options.mode == "raw":
            _bounded_camera_call(
                camera, "streaming",
                (True, True, RAW_STREAM_RESOLUTION), control_timeout,
                control_timeout, calls)
            stream_enabled = True

        while build is None:
            remaining = startup_deadline - start_clock()
            if remaining <= 0:
                raise BenchmarkError("timed out waiting for V7_BUILD marker")
            status_map = status_read(min(options.io_timeout, remaining))
            if isinstance(status_map, dict) and status_map.get("stdout"):
                remaining = startup_deadline - start_clock()
                if remaining <= 0:
                    raise BenchmarkError("timed out waiting for V7_BUILD marker")
                stdout_read(min(options.stdout_timeout, remaining))
                build = _build_marker("".join(stdout_chunks))
            if build is None:
                remaining = startup_deadline - start_clock()
                if remaining > 0:
                    delay = min(options.poll_ms / 1000.0, remaining)
                    sleep(delay)
                    sleep_total_ms += delay * 1000.0

        run_started = start_clock()
        run_deadline = run_started + options.duration
        poll_interval_s = options.poll_ms / 1000.0
        while (start_clock() < run_deadline and
               not script_terminal_marker_seen):
            poll_start = start_clock()
            poll_starts.append(poll_start)
            status_map = status_read(remaining_call_budget(run_deadline))
            if (isinstance(status_map, dict) and status_map.get("stdout") and
                    start_clock() < run_deadline):
                stdout_read(remaining_call_budget(
                    run_deadline, options.stdout_timeout))

            if (options.mode != "off" and start_clock() < run_deadline and
                    not script_terminal_marker_seen):
                if (isinstance(status_map, dict) and status_map.get("stream")
                        and start_clock() < run_deadline):
                    frame_budget = min(
                        options.frame_timeout, run_deadline - start_clock())
                    if frame_budget > 0:
                        frame = _bounded_camera_call(
                            camera, "read_frame", (), frame_budget,
                            options.io_timeout, calls)
                    else:
                        frame = None
                    if isinstance(frame, dict):
                        frame_returned_at = start_clock()
                        data = frame.get("data")
                        frame_metadata[(
                            frame.get("width"), frame.get("height"),
                            frame.get("format"), frame.get("depth"),
                            frame.get("raw_size"),
                            len(data) if data is not None else None,
                        )] += 1
                        if data is not None:
                            hash_started = start_clock()
                            frame_hashes.add(hashlib.sha256(bytes(data)).hexdigest())
                            hash_elapsed_ms.append(
                                (start_clock() - hash_started) * 1000.0)
                            frame_reads += 1
                            if marker_tracker is not None:
                                previous_new_count = (
                                    marker_tracker.new_sequence_reads)
                                decoded = marker_tracker.observe_rgb888(
                                    data, frame.get("width"),
                                    frame.get("height"))
                                if (marker_tracker.new_sequence_reads >
                                        previous_new_count):
                                    marker_arrivals.append({
                                        "sequence": decoded["sequence"],
                                        "host_monotonic_s":
                                            frame_returned_at,
                                        "host_monotonic_ns": int(
                                            frame_returned_at * 1_000_000_000),
                                        "run_elapsed_s": round(max(
                                            0.0, frame_returned_at -
                                            run_started), 6),
                                    })

            work_end = start_clock()
            loop_work_ms.append(max(0.0, (work_end - poll_start) * 1000.0))
            if script_terminal_marker_seen:
                break
            remaining = run_deadline - work_end
            if remaining > 0:
                delay = min(max(0.0, poll_interval_s -
                                (work_end - poll_start)), remaining)
                if delay > 0:
                    sleep(delay)
                    sleep_total_ms += delay * 1000.0

        observation_elapsed_s = max(0.0, start_clock() - run_started)
        status = "completed"
    except KeyboardInterrupt:
        status = "interrupted"
        error = "KeyboardInterrupt"
        if run_started is not None:
            observation_elapsed_s = max(0.0, start_clock() - run_started)
    except Exception as exc:
        status = "error"
        error = "%s: %s" % (type(exc).__name__, exc)
        if isinstance(exc, DeviceCallTimeout):
            camera_usable = False
        if run_started is not None:
            observation_elapsed_s = max(0.0, start_clock() - run_started)
    finally:
        if camera is not None and connected and camera_usable:
            if stream_enabled and script_terminal_marker_seen:
                stream_cleanup_skipped_reason = "script_terminal_marker_seen"
            elif stream_enabled:
                stream_cleanup_attempted = True
                try:
                    _bounded_camera_call(
                        camera, "streaming", (False,), control_timeout,
                        control_timeout, calls)
                    stream_enabled = False
                except Exception as exc:
                    camera_usable = False
                    if error is None:
                        error = "cleanup %s: %s" % (type(exc).__name__, exc)
                        status = "error"
            else:
                stream_cleanup_skipped_reason = "stream_not_enabled"

            # The SDK marks the stdout channel ready through read_status.
            # If the synthetic script has returned to REPL, its terminal
            # marker was already consumed and no further command is needed.
            if camera_usable and not script_terminal_marker_seen:
                try:
                    final_status = status_read(options.io_timeout)
                    if isinstance(final_status, dict) and final_status.get("stdout"):
                        stdout_read(options.stdout_timeout)
                except Exception:
                    pass

            transport_stats = _json_safe(getattr(
                getattr(camera, "transport", None), "stats", None))
            _disconnect_quietly(camera)
        elif camera is not None:
            transport_stats = _json_safe(getattr(
                getattr(camera, "transport", None), "stats", None))
            _disconnect_quietly(camera)

    stdout_text = "".join(stdout_chunks)
    perf_windows = parse_v7_perf_windows(stdout_text)
    board_execution_errors = parse_board_execution_errors(stdout_text)
    if board_execution_errors:
        excerpts = [item["raw_excerpt"]
                    for item in board_execution_errors[:3]]
        board_error = "board reported execution error: %s" % " | ".join(
            excerpts)
        status = "execution_error"
        if error is None:
            error = board_error
        elif board_error not in error:
            error = "%s; %s" % (error, board_error)
    synthetic_done = any(
        line.strip() == "V7_SYNTHETIC_MARKER_DONE status=ok"
        for line in stdout_text.splitlines())
    synthetic_counts = _synthetic_marker_counts(stdout_text)
    if synthetic_probe and status == "completed" and not synthetic_done:
        status = "error"
        error = error or "synthetic marker probe did not report completion"
    marker_windows = (summarize_marker_arrival_windows(marker_arrivals)
                      if marker_tracker is not None else None)
    report["host_transport_stats"] = transport_stats
    report["board_execution_error_count"] = len(board_execution_errors)
    report["board_execution_errors"] = board_execution_errors
    report["terminal_marker_is_success_evidence"] = False
    report["script_terminal_marker_seen"] = script_terminal_marker_seen
    report["script_terminal_marker"] = script_terminal_marker
    report["requested_duration_satisfied"] = (
        observation_elapsed_s >= options.duration)
    report["observation_end_reason"] = (
        "board_execution_error" if board_execution_errors else
        "script_terminal_marker" if script_terminal_marker_seen else
        "requested_duration_elapsed" if status == "completed" else
        "stopped_before_requested_duration")
    report["stream_cleanup"] = {
        "disable_attempted": stream_cleanup_attempted,
        "disable_skipped_reason": stream_cleanup_skipped_reason,
    }
    report["status"] = status
    report["error"] = error
    report["v7_build"] = build or _build_marker(stdout_text)
    report["synthetic_marker_probe_completed"] = (
        synthetic_done and not board_execution_errors
        if synthetic_probe else None)
    report["synthetic_marker_board_counts"] = (
        synthetic_counts if synthetic_probe else None)
    report["board_perf_window_count"] = len(perf_windows["windows"])
    report["board_counts_by_window"] = summarize_board_counts(perf_windows)
    report["host_observation"] = _host_report(
        options.mode, options.poll_ms, poll_starts, loop_work_ms,
        sleep_total_ms, calls, frame_reads, frame_hashes, frame_metadata,
        hash_elapsed_ms, observation_elapsed_s,
        marker_tracker.report() if marker_tracker is not None else None,
        marker_windows)
    report["host_observation"]["frame_origin"] = (
        "synthetic_grayscale_image" if synthetic_probe else "ram_fixture_stream")
    report["host_observation"]["live_sensor_capture"] = not synthetic_probe
    pre_exec_stdout_text = "".join(pre_exec_stdout_chunks)
    report["pre_exec_stdout"] = {
        "read_calls": pre_exec_stdout_read_calls,
        "status_polls": pre_exec_status_polls,
        "bytes": len(pre_exec_stdout_text.encode("utf-8")),
        "drain_completed_before_exec": pre_exec_drain_complete,
        "drain_elapsed_ms": round(pre_exec_drain_elapsed_ms, 3),
    }
    report["artifacts"] = {
        "raw_stdout": "raw_stdout.txt",
        "pre_exec_stdout": "pre_exec_stdout.txt",
        "board_v7_perf_windows": "board_v7_perf_windows.json",
        "report": "report.json",
    }
    if marker_tracker is not None:
        report["artifacts"]["marker_sequence_arrivals"] = (
            "marker_sequence_arrivals.jsonl")
    report["report_dir"] = str(report_dir)

    # Write captured text byte-for-byte as returned by the SDK's UTF-8 decoder.
    (report_dir / "raw_stdout.txt").write_bytes(stdout_text.encode("utf-8"))
    (report_dir / "pre_exec_stdout.txt").write_bytes(
        pre_exec_stdout_text.encode("utf-8"))
    (report_dir / "board_v7_perf_windows.json").write_text(
        json.dumps(perf_windows, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    if marker_tracker is not None:
        with (report_dir / "marker_sequence_arrivals.jsonl").open(
                "w", encoding="utf-8", newline="") as arrival_file:
            for arrival in marker_arrivals:
                arrival_file.write(json.dumps(arrival, sort_keys=True) + "\n")
    (report_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    return report


def main(argv=None, camera_factory=None):
    args = parse_args(argv)
    try:
        report = run_benchmark(args, camera_factory=camera_factory)
    except Exception as exc:
        print("OpenMV preview benchmark failed: %s: %s" %
              (type(exc).__name__, exc), file=sys.stderr)
        return 1
    print("status=%s mode=%s report_dir=%s" % (
        report["status"], report["mode"], report["report_dir"]))
    print("host_sdk_frame_reads=%d unique_content_hashes=%d "
          "camera_fps_claimed_by_host=false" % (
              report["host_observation"]["host_sdk_frame_reads"],
              report["host_observation"]["host_unique_content_hash_count"]))
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
