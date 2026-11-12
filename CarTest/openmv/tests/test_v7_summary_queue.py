"""Bounded FIFO and post-run drain behavior for performance summaries."""
import contextlib
import io
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import main


class SummaryQueueTests(unittest.TestCase):
    def test_enqueue_preserves_existing_rows_when_capacity_is_full(self):
        with patch.object(main._V7SummaryQueue, "CAPACITY", 3):
            queue = main._V7SummaryQueue()
        self.assertTrue(queue.enqueue(("window0-a", "window0-b")))
        self.assertFalse(queue.enqueue(("window1-a", "window1-b")))
        self.assertEqual(queue.pending, 2)
        self.assertEqual(queue.overflow_windows, 1)
        self.assertEqual(queue.pop(), "window0-a")
        self.assertEqual(queue.pop(), "window0-b")
        self.assertIsNone(queue.pop())

    def test_circular_wrap_keeps_fifo_order(self):
        with patch.object(main._V7SummaryQueue, "CAPACITY", 3):
            queue = main._V7SummaryQueue()
        self.assertTrue(queue.enqueue(("a", "b")))
        self.assertEqual(queue.pop(), "a")
        self.assertTrue(queue.enqueue(("c", "d")))
        self.assertEqual([queue.pop(), queue.pop(), queue.pop()],
                         ["b", "c", "d"])
        self.assertEqual(queue.pending, 0)
        self.assertTrue(queue.enqueue(("e",)))
        self.assertEqual(queue.pop(), "e")

    def test_window_summary_is_queued_without_immediate_output(self):
        class Perf:
            @staticmethod
            def summary_lines(run_id, window_index, attempted):
                return ["head", "stage0", "stage1"]

        queue = main._V7SummaryQueue()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = main._print_v7_summary(Perf(), 4, 2, 90, queue)
        self.assertTrue(result)
        self.assertEqual(output.getvalue(), "")
        self.assertEqual(queue.pending, 3)

    def test_final_drain_prints_each_row_and_paces_only_when_requested(self):
        queue = main._V7SummaryQueue()
        queue.enqueue(("head", "stage0", "stage1"))
        sleeps = []
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            count = main._drain_v7_summary_queue(
                queue, lambda milliseconds: sleeps.append(milliseconds))
        self.assertEqual(count, 3)
        self.assertEqual(output.getvalue().splitlines(),
                         ["head", "stage0", "stage1"])
        self.assertEqual(sleeps, [5, 5, 5])
        self.assertEqual(queue.pending, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
