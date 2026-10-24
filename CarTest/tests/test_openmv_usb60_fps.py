"""Hermetic tests for the optional OpenMV USB60 FPS delivery candidate.

The public checkpoint may omit the experimental package. In that case this
module reports one explicit unittest skip and has no dependency on private
logs, temporary snapshots, or workspace absolute paths.
"""

import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CANDIDATE_MAIN = (ROOT / "deliverables" /
                  "OpenMV_USB60_Candidate_20261024" / "main.py")


class _TicksShim:
    """MicroPython-like 30-bit ticks_diff for deterministic host timing."""

    PERIOD = 1 << 30
    HALF = PERIOD >> 1

    @classmethod
    def ticks_diff(cls, newer, older):
        return ((newer - older + cls.HALF) % cls.PERIOD) - cls.HALF


def _attribute_path(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
    return ".".join(reversed(parts))


def _contains_name(node, name):
    return any(isinstance(item, ast.Name) and item.id == name
               for item in ast.walk(node))


class OpenMVUSB60FPSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CANDIDATE_MAIN.is_file():
            raise unittest.SkipTest(
                "optional deliverables/OpenMV_USB60_Candidate_20261024/main.py "
                "is not present in this checkpoint")
        cls.source = CANDIDATE_MAIN.read_bytes()
        cls.tree = ast.parse(cls.source.decode("utf-8"),
                             filename=str(CANDIDATE_MAIN))
        cls.counter_class = next(
            node for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and
            node.name == "_FreshCaptureFPS")
        cls.fast_main = next(
            node for node in cls.tree.body
            if isinstance(node, ast.FunctionDef) and
            node.name == "_main_fast")
        namespace = {"time": _TicksShim}
        class_module = ast.Module(body=[cls.counter_class], type_ignores=[])
        exec(compile(class_module, str(CANDIDATE_MAIN), "exec"), namespace)
        cls.Counter = namespace["_FreshCaptureFPS"]

    def test_first_capture_anchors_then_17ms_window_reports_interval_rate(self):
        counter = self.Counter()
        self.assertEqual(counter.fps(0), 0.0)
        counter.record_success(0)
        self.assertEqual(counter.fps(0), 0.0)
        for timestamp in range(17, 1004, 17):
            counter.record_success(timestamp)
        self.assertAlmostEqual(counter.fps(1003), 59 * 1000.0 / 1003,
                               places=8)
        self.assertEqual(counter._window_intervals, 0)

    def test_getter_is_pure_and_missing_capture_is_not_counted(self):
        counter = self.Counter()
        counter.record_success(0)
        counter.record_success(17)
        before = (counter._window_start_ms, counter._last_capture_ms,
                  counter._window_intervals, counter._fps_value)
        self.assertEqual(counter.fps(34), 0.0)  # no success at t=34
        self.assertEqual(counter.fps(40), 0.0)
        self.assertEqual((counter._window_start_ms, counter._last_capture_ms,
                          counter._window_intervals, counter._fps_value),
                         before)
        counter.record_success(51)
        self.assertEqual(counter._window_intervals, 2)
        self.assertEqual(counter._last_capture_ms, 51)

    def test_ticks_wrap_and_long_gap_reanchor(self):
        counter = self.Counter()
        start = _TicksShim.PERIOD - 34
        for index in range(61):
            counter.record_success((start + index * 17) % _TicksShim.PERIOD)
        self.assertAlmostEqual(
            counter.fps((start + 60 * 17) % _TicksShim.PERIOD),
            59 * 1000.0 / 1003, places=8)

        paused = self.Counter()
        paused.record_success(0)
        paused.record_success(17)
        before = (paused._window_start_ms, paused._last_capture_ms,
                  paused._window_intervals, paused._fps_value)
        self.assertEqual(paused.fps(2017), 0.0)
        self.assertEqual((paused._window_start_ms, paused._last_capture_ms,
                          paused._window_intervals, paused._fps_value), before)
        paused.record_success(2017)
        self.assertEqual(paused._window_start_ms, 2017)
        self.assertEqual(paused._window_intervals, 0)
        self.assertEqual(paused.fps(2017), 0.0)

    def test_update_is_after_snapshot_exception_path_and_clock_is_removed(self):
        loop = next(node for node in ast.walk(self.fast_main)
                    if isinstance(node, ast.While))
        snapshot_try_index = next(
            index for index, statement in enumerate(loop.body)
            if isinstance(statement, ast.Try) and any(
                isinstance(call, ast.Call) and
                _attribute_path(call.func) in
                ("sensor.snapshot", "frame_source.snapshot")
                for call in ast.walk(statement)))
        update = next(node for node in ast.walk(self.fast_main)
                      if isinstance(node, ast.Call) and
                      _attribute_path(node.func) ==
                      "capture_fps_counter.record_success")
        update_index = next(index for index, statement in enumerate(loop.body)
                            if update in ast.walk(statement))
        self.assertGreater(update_index, snapshot_try_index)
        snapshot_try = loop.body[snapshot_try_index]
        self.assertTrue(snapshot_try.handlers)
        self.assertTrue(any(isinstance(node, ast.Raise)
                            for handler in snapshot_try.handlers
                            for node in ast.walk(handler)))
        self.assertFalse(any(isinstance(node, ast.Call) and
                             _attribute_path(node.func) in
                             ("time.clock", "clock.tick", "clock.fps")
                             for node in ast.walk(self.fast_main)))

    def test_uart_packet_ast_does_not_consume_fps_diagnostics(self):
        calls = [node for node in ast.walk(self.fast_main)
                 if isinstance(node, ast.Call)]
        pack_calls = [node for node in calls
                      if _attribute_path(node.func) ==
                      "vision_protocol.pack_into"]
        write_calls = [node for node in calls
                       if _attribute_path(node.func) == "uart.write"]
        self.assertTrue(pack_calls)
        self.assertTrue(write_calls)
        for call in pack_calls + write_calls:
            self.assertFalse(_contains_name(call, "capture_fps_value"))
            self.assertFalse(_contains_name(call, "capture_fps_counter"))
        self.assertTrue(any(len(call.args) == 1 and
                            isinstance(call.args[0], ast.Name) and
                            call.args[0].id == "packet"
                            for call in write_calls))


if __name__ == "__main__":
    unittest.main(verbosity=2)
