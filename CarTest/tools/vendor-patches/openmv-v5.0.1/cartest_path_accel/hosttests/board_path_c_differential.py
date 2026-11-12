"""Run a small on-device Viper/C differential after the module is linked.

Copy this helper beside the 23 runtime files and the native
``cartest_path_accel`` module. It performs no sensor capture, UART, or motor
operation. It deliberately builds typed arrays from list comprehensions.
"""

import sys


def _select_runtime_dir():
    """Optionally select a copied runtime directory before imports.

    OpenMV's REPL can retain an earlier ``fast_path`` module in
    ``sys.modules``.  Put the requested directory first and evict both
    runtime modules so this check cannot accidentally compare against the
    root/old copy.  The helper intentionally accepts a plain positional
    directory to keep invocation convenient on the board:
    ``board_path_c_differential.py [runtime_dir]``.
    """
    runtime_dir = None
    if len(sys.argv) > 2 and sys.argv[1] == "--runtime-dir":
        runtime_dir = sys.argv[2]
    elif len(sys.argv) > 1 and sys.argv[1]:
        runtime_dir = sys.argv[1]
    if runtime_dir:
        if runtime_dir not in sys.path:
            sys.path.insert(0, runtime_dir)
        for module_name in ("fast_path", "cartest_path_accel"):
            sys.modules.pop(module_name, None)


_select_runtime_dir()

from array import array
import fast_path
import cartest_path_accel


WIDTH = 80
HEIGHT = 60
FLOOR = 8
LAYERS = HEIGHT - FLOOR
MAX_PER_ROW = 8
NODES = LAYERS * MAX_PER_ROW
STATE_CAPACITY = NODES * 2


class _Fake:
    width = WIDTH
    height = HEIGHT
    floor_y = FLOOR
    row_count = LAYERS
    min_width = 8
    max_width = 70
    max_per_row = MAX_PER_ROW
    max_gap = 2
    max_width_change = 16
    max_slope_milli = 2500
    link_tolerance_milli = 2500
    reference_y_milli = 40000
    state_capacity = STATE_CAPACITY

    def __init__(self):
        self._counts = bytearray(LAYERS)
        self._candidate_data = array(
            "i", [0 for _ in range(NODES * 4)])
        self._state_data = array(
            "i", [0 for _ in range(STATE_CAPACITY * 10)])


def _frame():
    total = WIDTH * HEIGHT
    luma = bytearray([128 for _ in range(total)])
    road = bytearray(total)
    edge = bytearray(total)
    known = bytearray([1 for _ in range(total)])
    for y in range(FLOOR, HEIGHT):
        left = 10 + ((y - FLOOR) // 12)
        right = 69 + ((y - FLOOR) // 12)
        row = y * WIDTH
        for x in range(left, right + 1):
            road[row + x] = 1
        edge[row + left - 1] = 1
        edge[row + right + 1] = 1
    return luma, road, edge, known


def _extract(fake, frame):
    luma, road, edge, known = frame
    fake._current_luma = luma
    fake._current_road = road
    fake._current_edge = edge
    fake._current_known = known
    return fast_path._extract_viper(fake)


frame = _frame()
viper = _Fake()
native = _Fake()
viper_result = _extract(viper, frame)
native_result = cartest_path_accel.extract_rows(
    frame[0], frame[1], frame[2], frame[3], native._counts,
    native._candidate_data, WIDTH, HEIGHT, FLOOR,
    native.min_width, native.max_width, native.max_per_row)
if native_result != viper_result or bytes(native._counts) != bytes(viper._counts) or \
        list(native._candidate_data) != list(viper._candidate_data):
    raise RuntimeError("extract differential mismatch")
viper_connect = fast_path._connect_viper(viper)
native_connect = cartest_path_accel.connect_rows(
    native._counts, native._candidate_data, native._state_data,
    WIDTH, HEIGHT, FLOOR, native.max_gap, native.max_width_change,
    native.max_slope_milli, native.link_tolerance_milli,
    native.reference_y_milli, native.max_per_row, native.state_capacity)
if native_connect != viper_connect or \
        list(native._state_data) != list(viper._state_data):
    raise RuntimeError("connect differential mismatch")
print("PATH_C_DIFF=PASS cases=1 geometry=80x60")
