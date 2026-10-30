"""Host checks for the bounded OpenMV ring-entry diagnostic builder."""

from __future__ import annotations

import ast
import binascii
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
import zlib
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = PROJECT_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import build_openmv_full_pipeline_perf_probe as full_probe
import build_openmv_ring_entry_probe as ring_probe


def _embedded_sources(source: str) -> dict[str, bytes]:
    tree = ast.parse(source)
    result = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (isinstance(target, ast.Subscript) and
                    isinstance(target.value, ast.Name) and
                    target.value.id == "_ram_sources"):
                name = ast.literal_eval(target.slice)
                value = ast.literal_eval(node.value)
                if isinstance(name, str) and isinstance(value, bytes):
                    result[name] = value
    return result


def _capture_helpers(source: str) -> dict:
    tree = ast.parse(source)
    nodes = []
    for node in ast.walk(tree):
        if ((isinstance(node, ast.FunctionDef) and
             node.name.startswith("_entry_")) or
                (isinstance(node, ast.ClassDef) and
                 node.name.startswith("_EntryPull"))):
            nodes.append(node)
    namespace = {
        "_EntryCaptureAbort": type("_EntryCaptureAbort", (BaseException,), {}),
        "_EntryCaptureComplete": type(
            "_EntryCaptureComplete", (BaseException,), {}),
    }
    module = ast.Module(body=nodes, type_ignores=[])
    exec(compile(module, "<extracted-ring-entry-helpers>", "exec"), namespace)
    return namespace


class _Image:
    def __init__(self, value: int):
        self.buffer = bytearray([value & 0xFF]) * 38400

    def width(self):
        return 320

    def height(self):
        return 120

    def bytearray(self):
        return self.buffer


class _Tracker:
    def __init__(self):
        from array import array
        self._current_status = bytearray((i % 3) for i in range(120))
        self._current_edge2 = array("i", range(120))
        self._baseline_row_valid = bytearray([1] * 60)
        self._baseline_left_by_y2 = array("i", range(60))
        self._baseline_right_by_y2 = array("i", range(60, 120))
        self._outer_shift_x2 = array("i", [3, -3])
        self._calibration_width_valid = bytearray([1] * 60)
        self._calibration_width_cells = array("i", [30] * 60)
        self._ring_width_valid = bytearray([1] * 60)
        self._ring_width = array("i", [31] * 60)
        self._calibration_ready = True
        self._cal_check_ready = True
        self._ring_width_calibrated = True
        self._reappear_count = 0
        self._baseline_ready = True
        self._current_valid = True
        self._current_road = True
        self._current_topology_clear = True
        self._near_left_ambiguous = 0
        self._near_right_ambiguous = 0
        self._near_left_rows = 8
        self._near_right_rows = 9
        self._near_left_run = 4
        self._near_right_run = 5
        self.floor_y = 8
        self.near_rows = 12
        self.require_calibration = True
        self.ring_arc_min_rows = 4
        self.ring_arc_min_span_rows = 4


class _Recorder:
    def __init__(self):
        self.calls = []

    def record_frame(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return "original-result"


class _ProbeBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = ring_probe.build_probe_source()

    def test_embedded_sources_and_virtual_gate_are_preserved(self):
        base = full_probe.build_probe_source(mode="off", duration_s=70)
        base_modules = _embedded_sources(base)
        generated_modules = _embedded_sources(self.source)
        self.assertEqual(generated_modules, base_modules)
        self.assertEqual(len(generated_modules), 23)
        self.assertEqual(set(generated_modules),
                         {name for name, _raw in full_probe._module_sources()})

        for contract in (
                "class _VirtualUARTSink:",
                "reverse_control.COMMAND_RESET",
                "reverse_control.COMMAND_KEEP",
                "profile_c_parked_after_ack>=3",
                "self.active_ack_start_ms=time.ticks_ms()",
                "ring_diagnostics.RingDiagnostics.save=_ram_only_diag_save",
                "_RAM_DIAG_BLOB[0]=self._payload(build_id)",
                "ENTRY_READY_TO_PUSH=1",
                "_RUN_SECONDS=60",
                "_ACTIVE_SECONDS=60",
                "physical_uart=0 flash_write=0"):
            self.assertIn(contract, self.source)
        self.assertIn("except _EntryCaptureAbort:", self.source)
        self.assertIn("protocol_module.register(name='entry_meta',backend=meta_backend)",
                      self.source)
        self.assertIn("protocol_module.register(name='entry_data',backend=data_backend)",
                      self.source)
        self.assertIn("time.ticks_add(time.ticks_ms(),180000)", self.source)
        self.assertIn("time.sleep_ms(10)", self.source)
        self.assertIn("ENTRY_PULL_READY frame_count=%d image_count=%d", self.source)
        self.assertNotIn("protocol.init(", self.source)
        self.assertNotIn(
            "ENTRY_CAPTURE_DONE frame_count=%d processed_frame_count=%d image_count=%d error=%d",
            self.source)

    def test_manifest_hashes_entire_probe_and_marks_non_acceptance(self):
        manifest = ring_probe.build_manifest(self.source)
        self.assertEqual(manifest["runtime_module_count"], 23)
        self.assertEqual(manifest["not_fps_acceptance"], True)
        self.assertEqual(manifest["flash_write"], False)
        self.assertEqual(manifest["physical_uart"], False)
        self.assertEqual(manifest["images"]["max_images"], 8)
        self.assertEqual(manifest["images"]["byte_count_each"], 38400)
        self.assertEqual(manifest["images"]["pre_sampling_sequence_modulus"], 8)
        self.assertEqual(manifest["frame_buffer"]["maximum_records"], 129)
        self.assertEqual(manifest["frame_buffer"]["pre_records"], 32)
        self.assertEqual(manifest["frame_buffer"]["following_records"], 96)
        self.assertEqual(manifest["pull_transport"]["meta_channel"], "entry_meta")
        self.assertEqual(manifest["pull_transport"]["data_channel"], "entry_data")
        self.assertTrue(manifest["pull_transport"]["read_only"])
        self.assertTrue(manifest["pull_transport"]["random_access"])
        self.assertEqual(manifest["probe_sha256"],
                         __import__("hashlib").sha256(
                             self.source.encode("utf-8")).hexdigest())
        self.assertEqual(manifest["source_sha256"], manifest["probe_sha256"])
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "ring-entry-probe.py"
            output, sidecar, written = ring_probe.build_artifacts(path)
            self.assertTrue(output.is_file())
            self.assertTrue(sidecar.is_file())
            self.assertEqual(json.loads(sidecar.read_text(encoding="utf-8")),
                             written)

    def test_hook_calls_original_and_copies_per_frame_geometry(self):
        helpers = _capture_helpers(self.source)
        output = []
        helpers["print"] = output.append
        state = {
            "tracker": None, "images": [], "pre_images": [],
            "pre_frames": [], "frame_lines": [],
            "candidate_sequence": None, "candidate_post_count": 0,
            "candidate_following_count": 0, "processed_frame_count": 0,
            "dumped_image_count": 0, "dumped_frame_count": 0,
            "error": None, "emit": output.append,
        }
        tracker_module = types.SimpleNamespace(TrackElementsTracker=_Tracker)
        diagnostics_module = types.SimpleNamespace(RingDiagnostics=_Recorder)
        source = types.SimpleNamespace(latest_image=_Image(17))
        config = types.SimpleNamespace(
            TRACK_ELEMENTS_RING_ARC_MIN_ROWS=4,
            SURFACE_GRID_WIDTH=80,
            SURFACE_ROI_FLOOR_Y=8,
            ROAD_REFERENCE_Y=40)

        saved = helpers["_entry_install_capture_hooks"](
            state, tracker_module, diagnostics_module, source, config, json)
        tracker = tracker_module.TrackElementsTracker()
        recorder = diagnostics_module.RingDiagnostics()
        result = recorder.record_frame(
            321, 16, 3, True, True,
            {"source": "road", "reason": "OK"},
            {"state": "ENTER", "reason": "WAITING_FOR_RING_ARC",
             "candidate": False, "ring_confirmed": False,
             "ring_arc_rejection": "INNER_ARC_SUPPORT"})
        self.assertEqual(result, "original-result")
        self.assertEqual(len(recorder.calls), 1)
        self.assertEqual(state["processed_frame_count"], 1)
        self.assertEqual(output, [])
        self.assertEqual(len(state["pre_frames"]), 1)
        line = state["pre_frames"][0][1]
        self.assertFalse(line.startswith("ENTRY_FRAME "))
        row = json.loads(line)
        self.assertEqual(row["sequence"], 16)
        self.assertEqual(row["capture_ms"], 321)
        self.assertEqual(row["arc_rejection"], "INNER_ARC_SUPPORT")
        self.assertNotIn("thresholds_config", row)
        self.assertNotIn("thresholds_tracker", row)
        self.assertEqual(len(row["row_status"]), 120)
        self.assertEqual(len(row["row_edge2"]), 120)
        self.assertEqual(len(row["baseline_valid"]), 60)
        self.assertEqual(len(row["baseline_left2"]), 60)
        self.assertEqual(len(row["baseline_right2"]), 60)
        self.assertEqual(row["outer_shift_x2"], [3, -3])
        config_record = json.loads(state["config_json"])
        self.assertEqual(config_record["thresholds_config"][
            "TRACK_ELEMENTS_RING_ARC_MIN_ROWS"], 4)
        self.assertEqual(config_record["thresholds_tracker"][
            "ring_arc_min_rows"], 4)
        self.assertIsInstance(state["config_json"], str)
        self.assertEqual(output, [])

        # The JSON arrays and raw image are snapshots, not aliases to buffers.
        before = state["pre_images"][0]["data"]
        tracker._current_status[0] = 99
        source.latest_image.buffer[0] = 88
        self.assertEqual(row["row_status"][0], 0)
        self.assertEqual(before[0], 17)
        helpers["_entry_restore_capture_hooks"](saved)
        self.assertIs(tracker_module.TrackElementsTracker.__init__, _Tracker.__init__)
        self.assertIs(diagnostics_module.RingDiagnostics.record_frame,
                      _Recorder.record_frame)

    def test_pull_channels_support_random_cross_segment_reads_and_crc_rebuild(self):
        helpers = _capture_helpers(self.source)
        first_image = _Image(17)
        second_image = _Image(91)
        state = {
            "config_json": json.dumps({"thresholds_config": {"ring": 4}},
                                      separators=(",", ":")),
            "candidate_sequence": 100,
            "frame_lines": [
                (98, json.dumps({"sequence": 98, "reason": "WAITING"})),
                (99, json.dumps({"sequence": 99, "reason": "ARC_WAIT"})),
                (100, json.dumps({"sequence": 100, "candidate": True})),
            ],
            "pre_frames": [],
            "images": [],
            "pre_images": [],
            "processed_frame_count": 7,
            "dumped_frame_count": 0,
            "dumped_image_count": 0,
            "error": None,
        }
        helpers["_entry_take_image"](state, first_image, 100, 2000)
        helpers["_entry_take_image"](state, second_image, 108, 2160)
        first_image.buffer[0] = 255

        class _Protocol:
            def __init__(self):
                self.channels = {}

            def register(self, *, name, backend):
                self.channels[name] = backend

        protocol = _Protocol()
        meta_backend, data_backend, metadata = helpers[
            "_entry_register_pull_channels"](state, protocol, json, zlib.crc32)
        self.assertEqual(set(protocol.channels), {"entry_meta", "entry_data"})
        self.assertIs(protocol.channels["entry_meta"], meta_backend)
        self.assertIs(protocol.channels["entry_data"], data_backend)
        self.assertFalse(hasattr(meta_backend, "write"))
        self.assertFalse(hasattr(data_backend, "write"))
        self.assertEqual(metadata["schema"], "entry-pull-v1")
        self.assertEqual(metadata["frame_count"], 3)
        self.assertEqual(metadata["processed_frame_count"], 7)
        self.assertEqual(metadata["image_count"], 2)
        self.assertEqual(metadata["error"], 0)

        meta_bytes = meta_backend.read(0, meta_backend.size())
        self.assertEqual(json.loads(meta_bytes.decode("utf-8")), metadata)
        self.assertEqual(meta_backend.read(0, 32), meta_backend.read(0, 32))
        self.assertEqual(meta_backend.read(meta_backend.size() - 5, 20),
                         meta_bytes[-5:])

        expected_segments = [
            state["config_json"].encode("utf-8"),
            *(frame.encode("utf-8") for _sequence, frame in state["frame_lines"]),
            *(image["data"] for image in state["images"]),
        ]
        expected = b"".join(expected_segments)
        self.assertEqual(data_backend.size(), len(expected))
        self.assertEqual(data_backend.read(0, 19), data_backend.read(0, 19))

        ordered_records = [metadata["config"], *metadata["frames"],
                           *metadata["images"]]
        cursor = 0
        for record, segment in zip(ordered_records, expected_segments):
            self.assertEqual(record["offset"], cursor)
            self.assertEqual(record["size"], len(segment))
            self.assertEqual(record["crc32"],
                             "%08x" % (zlib.crc32(segment) & 0xFFFFFFFF))
            cursor += len(segment)
        self.assertEqual(cursor, data_backend.size())

        boundaries = [record["offset"] for record in ordered_records]
        boundaries += [record["offset"] + record["size"]
                       for record in ordered_records]
        for boundary in boundaries:
            start = max(0, boundary - 7)
            size = min(23, len(expected) - start)
            self.assertEqual(bytes(data_backend.read(start, size)),
                             expected[start:start + size])
            self.assertEqual(bytes(data_backend.read(start, size)),
                             expected[start:start + size])

        rebuilt = bytearray()
        for offset in range(0, len(expected), 317):
            rebuilt.extend(data_backend.read(offset, 317))
        self.assertEqual(bytes(rebuilt), expected)
        config_size = metadata["config"]["size"]
        frame_bytes = [
            bytes(rebuilt[item["offset"]:item["offset"] + item["size"]])
            for item in metadata["frames"]]
        self.assertEqual([json.loads(raw.decode("utf-8"))["sequence"]
                          for raw in frame_bytes], [98, 99, 100])
        for image_meta in metadata["images"]:
            raw = bytes(rebuilt[image_meta["offset"]:
                                image_meta["offset"] + image_meta["size"]])
            self.assertEqual(len(raw), 38400)
            self.assertEqual(raw[0], 17 if image_meta["image_id"] == 0 else 91)
            self.assertEqual(
                image_meta["crc32"],
                "%08x" % (zlib.crc32(raw) & 0xFFFFFFFF))
        self.assertEqual(json.loads(bytes(rebuilt[:config_size]).decode("utf-8")),
                         {"thresholds_config": {"ring": 4}})

    def test_capture_failure_escapes_normal_exception_handler(self):
        helpers = _capture_helpers(self.source)
        output = []
        state = {
            "tracker": None, "images": [],
            "pre_images": [], "pre_frames": [], "frame_lines": [],
            "candidate_sequence": None, "candidate_post_count": 0,
            "candidate_following_count": 0, "processed_frame_count": 0,
            "dumped_frame_count": 0, "dumped_image_count": 0,
            "error": None, "emit": output.append,
        }
        helpers["print"] = output.append
        tracker_module = types.SimpleNamespace(TrackElementsTracker=_Tracker)
        diagnostics_module = types.SimpleNamespace(RingDiagnostics=_Recorder)
        source = types.SimpleNamespace(latest_image=None)
        config = types.SimpleNamespace()
        saved = helpers["_entry_install_capture_hooks"](
            state, tracker_module, diagnostics_module, source, config, json)
        tracker_module.TrackElementsTracker()
        recorder = diagnostics_module.RingDiagnostics()
        with self.assertRaises(helpers["_EntryCaptureAbort"]):
            recorder.record_frame(10, 16, 3, True, True,
                                  {"source": "road", "reason": "OK"},
                                  {"candidate": False})
        self.assertEqual(len(recorder.calls), 1)
        self.assertTrue(state["error_line"].startswith("ENTRY_CAPTURE_ERROR "))
        self.assertIsNotNone(state["error"])
        _meta, _segments, metadata = helpers["_entry_build_pull_payload"](
            state, json, zlib.crc32)
        self.assertEqual(metadata["error"], 1)
        self.assertIn("source.latest_image is missing",
                      metadata["error_message"])
        self.assertEqual(metadata["frame_count"], 1)
        helpers["_entry_restore_capture_hooks"](saved)

    def test_candidate_buffer_caps_at_32_pre_plus_candidate_and_96_following(self):
        helpers = _capture_helpers(self.source)
        progress = []
        helpers["print"] = progress.append
        state = {"tracker": None, "images": [], "pre_images": [],
                 "pre_frames": [], "frame_lines": [],
                 "candidate_sequence": None, "candidate_post_count": 0,
                 "candidate_following_count": 0, "processed_frame_count": 0,
                 "dumped_frame_count": 0, "dumped_image_count": 0,
                 "error": None, "emit": lambda _line: None}
        tracker_module = types.SimpleNamespace(TrackElementsTracker=_Tracker)
        diagnostics_module = types.SimpleNamespace(RingDiagnostics=_Recorder)
        source = types.SimpleNamespace(latest_image=None)
        config = types.SimpleNamespace(TRACK_ELEMENTS_RING_ARC_MIN_ROWS=4)
        saved = helpers["_entry_install_capture_hooks"](
            state, tracker_module, diagnostics_module, source, config, json)
        tracker_module.TrackElementsTracker()
        recorder = diagnostics_module.RingDiagnostics()
        candidate_sequence = 1032
        completed = False
        for sequence in range(1000, candidate_sequence + 97):
            source.latest_image = _Image(sequence)
            tracker_result = {
                "candidate": sequence == candidate_sequence,
                "ring_confirmed": False,
                "state": "ENTER",
                "reason": "WAITING_FOR_RING_ARC",
            }
            try:
                recorder.record_frame(
                    sequence * 20, sequence, 3, True, True,
                    {"source": "road", "reason": "OK"}, tracker_result)
            except helpers["_EntryCaptureComplete"]:
                completed = True
                break
        self.assertTrue(completed)
        self.assertTrue(state["complete"])
        self.assertIsNone(state["error"])
        self.assertEqual(state["processed_frame_count"], 129)
        self.assertEqual(state["candidate_following_count"], 96)
        self.assertEqual(len(state["frame_lines"]), 129)
        self.assertEqual(len(state["images"]), 8)
        self.assertEqual(len(recorder.calls), 129)
        pre_sequences = [
            json.loads(line)["sequence"]
            for _sequence, line in state["frame_lines"][:32]]
        self.assertEqual(pre_sequences, list(range(1000, 1032)))
        self.assertEqual(
            json.loads(state["frame_lines"][32][1])["sequence"],
            candidate_sequence)
        self.assertTrue(progress)
        self.assertTrue(all(len(line) <= 80 for line in progress))
        helpers["_entry_restore_capture_hooks"](saved)

    def test_no_candidate_pull_keeps_last_32_and_aligned_pre_images(self):
        helpers = _capture_helpers(self.source)
        progress = []
        helpers["print"] = progress.append
        state = {"tracker": None, "images": [], "pre_images": [],
                 "pre_frames": [], "frame_lines": [],
                 "candidate_sequence": None, "candidate_post_count": 0,
                 "candidate_following_count": 0, "processed_frame_count": 0,
                 "dumped_frame_count": 0, "dumped_image_count": 0,
                 "error": None, "emit": lambda _line: None}
        tracker_module = types.SimpleNamespace(TrackElementsTracker=_Tracker)
        diagnostics_module = types.SimpleNamespace(RingDiagnostics=_Recorder)
        source = types.SimpleNamespace(latest_image=None)
        config = types.SimpleNamespace(TRACK_ELEMENTS_RING_ARC_MIN_ROWS=4)
        saved = helpers["_entry_install_capture_hooks"](
            state, tracker_module, diagnostics_module, source, config, json)
        tracker_module.TrackElementsTracker()
        recorder = diagnostics_module.RingDiagnostics()
        for sequence in range(100, 140):
            source.latest_image = _Image(sequence)
            recorder.record_frame(
                sequence * 20, sequence, 3, True, True,
                {"source": "road", "reason": "OK"},
                {"candidate": False, "ring_confirmed": False,
                 "state": "OBSERVE", "reason": "OBSERVING"})

        _meta_bytes, segments, metadata = helpers[
            "_entry_build_pull_payload"](state, json, zlib.crc32)
        self.assertEqual(metadata["frame_count"], 32)
        self.assertEqual(metadata["processed_frame_count"], 40)
        self.assertEqual(metadata["image_count"], 4)
        frame_sequences = [frame["sequence"] for frame in metadata["frames"]]
        self.assertEqual(frame_sequences, list(range(108, 140)))
        self.assertTrue(all(item["sequence"] in frame_sequences
                            for item in metadata["images"]))
        self.assertEqual(len(segments), 1 + 32 + 4)
        self.assertTrue(all(isinstance(segment, str)
                            for segment in segments[:33]))
        self.assertTrue(all(isinstance(segment, bytes)
                            for segment in segments[33:]))
        helpers["_entry_restore_capture_hooks"](saved)

    def test_pull_registration_occurs_after_capture_without_stdout_done(self):
        tree = ast.parse(self.source)
        finish_index = next(
            index for index, node in enumerate(tree.body)
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id == "print"
            and any(isinstance(arg, ast.Constant) and isinstance(arg.value, str)
                    and "RAM_FULL_PIPELINE_PROBE_FINISHED" in arg.value
                    for arg in node.value.args))
        pull_node = tree.body[finish_index - 1]
        finish_node = tree.body[finish_index]
        self.assertIsInstance(pull_node, ast.If)
        self.assertIsInstance(finish_node, ast.Expr)
        self.assertTrue(any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id == "_entry_register_pull_channels"
            for node in ast.walk(pull_node)))
        registration_try = next(node for node in pull_node.body
                                if isinstance(node, ast.Try))
        self.assertFalse(any(
            isinstance(node, ast.Constant) and isinstance(node.value, str)
            and "ENTRY_CAPTURE_DONE" in node.value
            for statement in registration_try.body
            for node in ast.walk(statement)))

        output = []
        protocol_module = types.ModuleType("protocol")
        protocol_module.channels = {}
        protocol_module.register = lambda *, name, backend: \
            protocol_module.channels.__setitem__(name, backend)
        state = {"config_json": '{"thresholds_config":{}}',
                 "candidate_sequence": 1,
                 "frame_lines": [(1, '{"sequence":1}')],
                 "images": [], "pre_images": [], "processed_frame_count": 1,
                 "error": None}

        namespace = {
            "_entry_capture_state": state,
            "_entry_register_pull_channels": _capture_helpers(self.source)[
                "_entry_register_pull_channels"],
            "_entry_ujson": json,
            "_entry_ubinascii": types.SimpleNamespace(
                crc32=zlib.crc32, hexlify=binascii.hexlify),
            "time": types.SimpleNamespace(
                ticks_add=lambda now, delta: now + delta,
                ticks_ms=lambda: 0,
                ticks_diff=lambda deadline, _now: 1,
                sleep_ms=lambda _value: (_ for _ in ()).throw(KeyboardInterrupt())),
            "print": output.append,
        }
        normal_path = ast.Module(body=[pull_node, finish_node], type_ignores=[])
        with patch.dict(sys.modules, {"protocol": protocol_module}):
            exec(compile(normal_path, "<ring-probe-pull-ready>", "exec"),
                 namespace)
        finish_output_index = next(i for i, line in enumerate(output)
                                   if isinstance(line, str) and
                                   "RAM_FULL_PIPELINE_PROBE_FINISHED" in line)
        self.assertEqual(protocol_module.channels.keys(),
                         {"entry_meta", "entry_data"})
        ready_index = next(i for i,line in enumerate(output)
                           if line.startswith("ENTRY_PULL_READY "))
        self.assertLess(ready_index, finish_output_index)
        self.assertIn("frame_count=1 image_count=0", output[ready_index])
        self.assertFalse(any(line.startswith("ENTRY_CAPTURE_DONE ")
                             for line in output[:finish_output_index]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
