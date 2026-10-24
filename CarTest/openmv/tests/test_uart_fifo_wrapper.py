"""Host mocks and AST guards for the private OpenMV UART FIFO wrapper."""

import ast
import copy
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
BASELINE_MAIN = ROOT / "tests" / "fixtures" / "uart_fifo_wrapper_baseline_20261024" / "main.py"
CANDIDATE_MAIN = ROOT / "main.py"


def _find_function(tree, name):
    return next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == name)


def _find_class(tree, name):
    return next(node for node in tree.body
                if isinstance(node, ast.ClassDef) and node.name == name)


def _assigned_name(node, name):
    return any(isinstance(target, ast.Name) and target.id == name
               for target in node.targets)


def _contains_uart_constructor(node):
    return any(isinstance(child, ast.Call) and
               isinstance(child.func, ast.Attribute) and
               isinstance(child.func.value, ast.Name) and
               child.func.value.id == "pyb" and child.func.attr == "UART"
               for child in ast.walk(node))


def _find_uart_setup(function):
    return next(node for node in ast.walk(function)
                if isinstance(node, ast.If) and
                isinstance(node.test, ast.Name) and
                node.test.id == "fixture_mode" and
                _contains_uart_constructor(node))


def _assignment_to(node, target_name):
    return any(isinstance(child, ast.Assign) and
               any(isinstance(target, ast.Name) and
                   target.id == target_name for target in child.targets)
               for child in node.body)


def _normalize_main_fast(candidate, baseline):
    candidate = copy.deepcopy(candidate)
    baseline = copy.deepcopy(baseline)

    def uart_init(function):
        return next(node for node in ast.walk(function)
                    if isinstance(node, ast.If) and
                    isinstance(node.test, ast.Name) and
                    node.test.id == "fixture_mode" and
                    _contains_uart_constructor(node))

    candidate_init = uart_init(candidate)
    baseline_init = uart_init(baseline)

    class ReplaceNode(ast.NodeTransformer):
        def visit_If(self, node):
            if node is candidate_init:
                return copy.deepcopy(baseline_init)
            return self.generic_visit(node)

        def visit_Expr(self, node):
            value = node.value
            format_value = None
            if isinstance(value, ast.Call) and value.args:
                first = value.args[0]
                if isinstance(first, ast.Constant):
                    format_value = first.value
                elif (isinstance(first, ast.BinOp) and
                      isinstance(first.left, ast.Constant)):
                    format_value = first.left.value
            if (isinstance(value, ast.Call) and
                    isinstance(value.func, ast.Name) and
                    value.func.id == "print" and value.args and
                    isinstance(format_value, str) and
                    format_value.startswith("UART_TX_MODE=")):
                return None
            return self.generic_visit(node)

        def visit_Assign(self, node):
            if (_assigned_name(node, "last_tx_result") and
                    isinstance(node.value, ast.IfExp)):
                baseline_assignment = next(
                    item for item in ast.walk(baseline)
                    if isinstance(item, ast.Assign) and
                    _assigned_name(item, "last_tx_result") and
                    isinstance(item.value, ast.Constant) and
                    item.value.value == "FULL_16")
                return copy.deepcopy(baseline_assignment)
            return self.generic_visit(node)

    normalized = ReplaceNode().visit(candidate)
    return ast.fix_missing_locations(normalized)


class _FakeUART:
    WRITE_FIFO_API = 1

    def __init__(self, *, instance_api="inherit", fifo_result=16,
                 sync_result=999, any_result=3, readinto_result=7,
                 fifo_error=None, sync_error=None, include_fifo=True,
                 include_done=True):
        if instance_api != "inherit":
            self.WRITE_FIFO_API = instance_api
        self.fifo_result = fifo_result
        self.sync_result = sync_result
        self.any_result = any_result
        self.readinto_result = readinto_result
        self.fifo_error = fifo_error
        self.sync_error = sync_error
        self.fifo_calls = []
        self.sync_calls = []
        self.any_calls = []
        self.readinto_calls = []
        self.done_calls = 0
        if not include_fifo:
            self.write_fifo = None
        if not include_done:
            self.fifo_txdone = None

    def write_fifo(self, data):
        self.fifo_calls.append(data)
        if self.fifo_error is not None:
            raise self.fifo_error
        return self.fifo_result

    def fifo_txdone(self):
        self.done_calls += 1
        return True

    def write(self, data):
        self.sync_calls.append(data)
        if self.sync_error is not None:
            raise self.sync_error
        return self.sync_result

    def any(self, *args, **kwargs):
        self.any_calls.append((args, kwargs))
        return self.any_result

    def readinto(self, *args, **kwargs):
        self.readinto_calls.append((args, kwargs))
        return self.readinto_result


class UARTFIFOWrapperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        candidate_tree = ast.parse(CANDIDATE_MAIN.read_text(encoding="utf-8"))
        cls.writer_ast = _find_class(candidate_tree, "_VisionUARTWriter")
        namespace = {}
        exec(compile(ast.Module(body=[copy.deepcopy(cls.writer_ast)],
                                type_ignores=[]),
                     str(CANDIDATE_MAIN), "exec"), namespace)
        cls.writer_type = namespace["_VisionUARTWriter"]

    def test_capability_requires_integer_v1_marker_and_both_callables(self):
        candidates = (
            (_FakeUART(instance_api=0), "SYNCHRONOUS_FALLBACK"),
            (_FakeUART(instance_api=2), "SYNCHRONOUS_FALLBACK"),
            (_FakeUART(instance_api=True), "SYNCHRONOUS_FALLBACK"),
            (_FakeUART(include_fifo=False), "SYNCHRONOUS_FALLBACK"),
            (_FakeUART(include_done=False), "SYNCHRONOUS_FALLBACK"),
        )
        for uart, expected in candidates:
            with self.subTest(expected=expected, instance_api=
                              getattr(uart, "WRITE_FIFO_API", None)):
                writer = self.writer_type(uart)
                self.assertEqual(writer.mode, expected)

        class NoMarker(_FakeUART):
            WRITE_FIFO_API = 0

        self.assertEqual(self.writer_type(NoMarker()).mode,
                         "SYNCHRONOUS_FALLBACK")

    def test_class_or_instance_integer_v1_marker_enables_fifo(self):
        self.assertEqual(self.writer_type(_FakeUART()).mode, "FIFO_QUEUED")

        class InstanceOnly(_FakeUART):
            WRITE_FIFO_API = 0

            def __init__(self):
                super().__init__(instance_api=1)

        self.assertEqual(self.writer_type(InstanceOnly()).mode, "FIFO_QUEUED")

    def test_fifo_full_return_is_one_same_object_call_and_no_sync_write(self):
        uart = _FakeUART(fifo_result=16)
        writer = self.writer_type(uart)
        packet = bytearray(range(16))
        self.assertEqual(writer.write(packet), 16)
        self.assertEqual(len(uart.fifo_calls), 1)
        self.assertIs(uart.fifo_calls[0], packet)
        self.assertEqual(uart.sync_calls, [])
        # Completion is exposed but is not polled by write().
        self.assertEqual(uart.done_calls, 0)
        self.assertTrue(writer.fifo_txdone())
        self.assertEqual(uart.done_calls, 1)

    def test_busy_none_and_partial_results_are_returned_without_retry(self):
        for result in (None, 7, 0):
            uart = _FakeUART(fifo_result=result)
            writer = self.writer_type(uart)
            packet = bytearray(16)
            with self.subTest(result=result):
                self.assertIs(writer.write(packet), result)
                self.assertEqual(len(uart.fifo_calls), 1)
                self.assertEqual(uart.sync_calls, [])

    def test_fifo_exception_propagates_without_synchronous_fallback(self):
        error = RuntimeError("fifo unavailable")
        uart = _FakeUART(fifo_error=error)
        writer = self.writer_type(uart)
        with self.assertRaisesRegex(RuntimeError, "fifo unavailable"):
            writer.write(bytearray(16))
        self.assertEqual(len(uart.fifo_calls), 1)
        self.assertEqual(uart.sync_calls, [])

    def test_absent_capability_uses_exact_synchronous_write_once(self):
        uart = _FakeUART(instance_api=0, sync_result=16)
        writer = self.writer_type(uart)
        packet = bytearray(16)
        self.assertEqual(writer.write(packet), 16)
        self.assertEqual(len(uart.sync_calls), 1)
        self.assertIs(uart.sync_calls[0], packet)
        self.assertEqual(uart.fifo_calls, [])

    def test_sync_exception_propagates_without_fifo_attempt(self):
        error = OSError("sync write failed")
        uart = _FakeUART(instance_api=0, sync_error=error)
        writer = self.writer_type(uart)
        with self.assertRaisesRegex(OSError, "sync write failed"):
            writer.write(bytearray(16))
        self.assertEqual(len(uart.sync_calls), 1)
        self.assertEqual(uart.fifo_calls, [])

    def test_any_and_readinto_delegate_original_arguments_and_results(self):
        uart = _FakeUART(any_result=5, readinto_result=2)
        writer = self.writer_type(uart)
        token = object()
        buffer = bytearray(8)
        self.assertEqual(writer.any(token, mode="probe"), 5)
        self.assertEqual(uart.any_calls[0][0][0], token)
        self.assertEqual(uart.any_calls[0][1], {"mode": "probe"})
        self.assertEqual(writer.readinto(buffer, 3), 2)
        self.assertIs(uart.readinto_calls[0][0][0], buffer)
        self.assertEqual(uart.readinto_calls[0][0][1], 3)

    def test_only_uart_wrapper_and_tx_status_change_in_main_fast(self):
        baseline_tree = ast.parse(BASELINE_MAIN.read_text(encoding="utf-8"))
        candidate_tree = ast.parse(CANDIDATE_MAIN.read_text(encoding="utf-8"))
        baseline_fast = _find_function(baseline_tree, "_main_fast")
        candidate_fast = _find_function(candidate_tree, "_main_fast")
        self.assertEqual(
            ast.dump(_normalize_main_fast(candidate_fast, baseline_fast),
                     include_attributes=False),
            ast.dump(baseline_fast, include_attributes=False))

        normalized_candidate_body = []
        for node in candidate_tree.body:
            if isinstance(node, ast.ClassDef) and \
                    node.name == "_VisionUARTWriter":
                continue
            if isinstance(node, ast.FunctionDef) and \
                    node.name == "_main_fast":
                node = _normalize_main_fast(node, baseline_fast)
            normalized_candidate_body.append(
                ast.dump(node, include_attributes=False))
        self.assertEqual(
            normalized_candidate_body,
            [ast.dump(node, include_attributes=False)
             for node in baseline_tree.body])

        # The fixture/sink route must remain direct, while only the real UART
        # path receives the adapter.
        setup = _find_uart_setup(candidate_fast)
        self.assertTrue(_assignment_to(setup, "uart"))
        self.assertTrue(any(isinstance(stmt, ast.Assign) and
                            any(isinstance(target, ast.Name) and
                                target.id == "uart" for target in stmt.targets)
                            and isinstance(stmt.value, ast.Name) and
                            stmt.value.id == "uart_sink"
                            for stmt in setup.body))
        self.assertTrue(any(isinstance(stmt, ast.Assign) and
                            any(isinstance(target, ast.Name) and
                                target.id == "uart_tx_mode"
                                for target in stmt.targets) and
                            isinstance(stmt.value, ast.Constant) and
                            stmt.value.value == "SINK"
                            for stmt in setup.body))
        self.assertFalse(any(isinstance(node, ast.Call) and
                             isinstance(node.func, ast.Name) and
                             node.func.id == "_VisionUARTWriter"
                             for stmt in setup.body for node in ast.walk(stmt)))
        self.assertTrue(any(isinstance(stmt, ast.Assign) and
                            any(isinstance(target, ast.Name) and
                                target.id == "uart" for target in stmt.targets)
                            and isinstance(stmt.value, ast.Call) and
                            isinstance(stmt.value.func, ast.Name) and
                            stmt.value.func.id == "_VisionUARTWriter"
                            for stmt in setup.orelse))

        candidate_source = CANDIDATE_MAIN.read_text(encoding="utf-8")
        self.assertIn('"UART_TX_MODE=%s" % uart_tx_mode', candidate_source)
        self.assertIn('"QUEUED_16" if', candidate_source)
        self.assertIn('uart_tx_mode == "FIFO_QUEUED"', candidate_source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
