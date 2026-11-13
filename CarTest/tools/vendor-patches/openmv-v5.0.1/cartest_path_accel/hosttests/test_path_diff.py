"""Differential tests for the extracted fast_path Viper kernels."""

import ctypes
import importlib.util
import random
import sys
import types
from array import array
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
ROOT = next(
    parent for parent in TASK.parents
    if (parent / "openmv").is_dir() and (parent / "logs").is_dir())
DLL = ROOT / "logs" / "ring45-usb60-20261026" / \
    "path-c-hostbuild" / "cartest_path_accel.dll"
REFERENCE = ROOT / "logs" / "ring45-usb60-20261026" / \
    "pipeline_perf_v12_integrated" / "fast_path.py"
WIDTH = 80
HEIGHT = 60
FLOOR = 8
LAYERS = HEIGHT - FLOOR
MAX_PER_ROW = 8
NODES = LAYERS * MAX_PER_ROW
STATE_CAPACITY = NODES * 2


def load_reference():
    topology = types.ModuleType("fast_topology")
    topology.FastTopologyGuard = type("FastTopologyGuard", (), {})
    sys.modules["fast_topology"] = topology
    spec = importlib.util.spec_from_file_location("path_reference", REFERENCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ptr(buffer, ctype):
    return ctypes.cast(ctypes.addressof(ctypes.c_char.from_buffer(buffer)),
                       ctypes.POINTER(ctype))


def make_fake(ref, min_width=8, max_width=70):
    fake = types.SimpleNamespace()
    fake.width = WIDTH
    fake.height = HEIGHT
    fake.floor_y = FLOOR
    fake.row_count = LAYERS
    fake.min_width = min_width
    fake.max_width = max_width
    fake.max_per_row = MAX_PER_ROW
    fake.max_gap = 2
    fake.max_width_change = 16
    fake.max_slope_milli = 2500
    fake.link_tolerance_milli = 2500
    fake.reference_y_milli = 40000
    fake.state_capacity = STATE_CAPACITY
    fake._counts = bytearray(LAYERS)
    fake._candidate_data = array("i", [0] * (NODES * 4))
    fake._state_data = array("i", [0] * (STATE_CAPACITY * 10))
    return fake


def make_dll():
    dll = ctypes.CDLL(str(DLL))
    u8 = ctypes.POINTER(ctypes.c_uint8)
    i32 = ctypes.POINTER(ctypes.c_int32)
    z = ctypes.c_size_t
    scalar = ctypes.c_int32
    dll.cartest_path_accel_extract.argtypes = [
        u8, z, u8, z, u8, z, u8, z, u8, z, i32, z,
        scalar, scalar, scalar, scalar, scalar, scalar]
    dll.cartest_path_accel_extract.restype = ctypes.c_int32
    dll.cartest_path_accel_connect.argtypes = [
        u8, z, i32, z, i32, z, scalar, scalar, scalar, scalar,
        scalar, scalar, scalar, scalar, scalar]
    dll.cartest_path_accel_connect.restype = ctypes.c_int32
    return dll


def invoke_extract(dll, fake):
    total = WIDTH * HEIGHT
    return dll.cartest_path_accel_extract(
        ptr(fake._current_luma, ctypes.c_uint8), total,
        ptr(fake._current_road, ctypes.c_uint8), total,
        ptr(fake._current_edge, ctypes.c_uint8), total,
        ptr(fake._current_known, ctypes.c_uint8), total,
        ptr(fake._counts, ctypes.c_uint8), len(fake._counts),
        ptr(fake._candidate_data, ctypes.c_int32),
        len(fake._candidate_data) * 4,
        WIDTH, HEIGHT, FLOOR, fake.min_width, fake.max_width,
        fake.max_per_row)


def invoke_connect(dll, fake):
    return dll.cartest_path_accel_connect(
        ptr(fake._counts, ctypes.c_uint8), len(fake._counts),
        ptr(fake._candidate_data, ctypes.c_int32),
        len(fake._candidate_data) * 4,
        ptr(fake._state_data, ctypes.c_int32),
        len(fake._state_data) * 4,
        WIDTH, HEIGHT, FLOOR, fake.max_gap, fake.max_width_change,
        fake.max_slope_milli, fake.link_tolerance_milli,
        fake.reference_y_milli, fake.max_per_row, fake.state_capacity)


def paint_corridor(fake, variant, seed):
    rng = random.Random(seed)
    total = WIDTH * HEIGHT
    fake._current_luma = bytearray(rng.randrange(256) for _ in range(total))
    fake._current_road = bytearray(total)
    fake._current_edge = bytearray(total)
    fake._current_known = bytearray([1] * total)
    for y in range(FLOOR, HEIGHT):
        shift = ((y - FLOOR) * variant) % 7
        left = 10 + shift
        right = 69 + shift
        if right >= WIDTH - 1:
            left -= right - (WIDTH - 2)
            right = WIDTH - 2
        for x in range(left, right + 1):
            fake._current_road[y * WIDTH + x] = 1
        fake._current_edge[y * WIDTH + left - 1] = 1
        fake._current_edge[y * WIDTH + right + 1] = 1
        if variant == 3 and y % 7 == 0:
            fake._current_known[y * WIDTH + left:left + 2] = b"\0\0"
    return fake


def compare_case(ref, dll, variant, seed):
    py = make_fake(ref)
    c = make_fake(ref)
    paint_corridor(py, variant, seed)
    c._current_luma = bytearray(py._current_luma)
    c._current_road = bytearray(py._current_road)
    c._current_edge = bytearray(py._current_edge)
    c._current_known = bytearray(py._current_known)
    py_result = ref._extract_viper(py)
    c_result = invoke_extract(dll, c)
    assert c_result == py_result, ("extract return", variant, seed,
                                   py_result, c_result)
    assert bytes(c._counts) == bytes(py._counts), ("counts", variant, seed)
    assert list(c._candidate_data) == list(py._candidate_data), (
        "candidates", variant, seed)
    py_connect = ref._connect_viper(py)
    c_connect = invoke_connect(dll, c)
    assert c_connect == py_connect, ("connect return", variant, seed)
    assert list(c._state_data) == list(py._state_data), (
        "states", variant, seed)


def compare_buffers(ref, dll, py, c, label):
    py_result = ref._extract_viper(py)
    c_result = invoke_extract(dll, c)
    assert c_result == py_result, ("extract return", label, py_result, c_result)
    assert bytes(c._counts) == bytes(py._counts), ("counts", label)
    assert list(c._candidate_data) == list(py._candidate_data), (
        "candidates", label)
    py_connect = ref._connect_viper(py)
    c_connect = invoke_connect(dll, c)
    assert c_connect == py_connect, ("connect return", label)
    assert list(c._state_data) == list(py._state_data), ("states", label)


def compare_random(ref, dll, seed):
    rng = random.Random(seed)
    py = make_fake(ref)
    c = make_fake(ref)
    total = WIDTH * HEIGHT
    luma = bytearray(rng.randrange(256) for _ in range(total))
    road = bytearray(rng.randrange(2) for _ in range(total))
    edge = bytearray(rng.randrange(2) for _ in range(total))
    known = bytearray(rng.randrange(2) for _ in range(total))
    py._current_luma = bytearray(luma)
    py._current_road = bytearray(road)
    py._current_edge = bytearray(edge)
    py._current_known = bytearray(known)
    c._current_luma = bytearray(luma)
    c._current_road = bytearray(road)
    c._current_edge = bytearray(edge)
    c._current_known = bytearray(known)
    compare_buffers(ref, dll, py, c, "random-%d" % seed)


def compare_overflow(ref, dll):
    py = make_fake(ref)
    c = make_fake(ref)
    total = WIDTH * HEIGHT
    for obj in (py, c):
        obj._current_luma = bytearray([127] * total)
        obj._current_road = bytearray(total)
        obj._current_edge = bytearray(total)
        obj._current_known = bytearray([1] * total)
        for y in range(FLOOR, HEIGHT):
            left = 10 + ((y - FLOOR) // 12)
            right = 69 + ((y - FLOOR) // 12)
            row = y * WIDTH
            obj._current_road[row + left:row + right + 1] = b"\1" * 60
            obj._current_edge[row + left - 1] = 1
            obj._current_edge[row + right + 1] = 1
    compare_buffers(ref, dll, py, c, "candidate-overflow")
    # Counts max+1 and 255 must be rejected before any state mutation.
    for bad_count in (MAX_PER_ROW + 1, 255):
        c._counts[0] = bad_count
        before = list(c._state_data)
        result = invoke_connect(dll, c)
        assert result == -100 and list(c._state_data) == before, (
            "count overflow validation", bad_count)
    # A physically impossible candidate must also fail closed.
    c._counts[0] = 1
    c._candidate_data[1] = 0
    before = list(c._state_data)
    result = invoke_connect(dll, c)
    assert result == -100 and list(c._state_data) == before, (
        "candidate relation validation")
    # A short state buffer is rejected without touching the full state array.
    c._candidate_data[1] = c._candidate_data[0] * 2 + \
        c._candidate_data[2] - 1
    short_states = array("i", [91 for _ in range(STATE_CAPACITY * 10)])
    before = list(short_states)
    result = dll.cartest_path_accel_connect(
        ptr(c._counts, ctypes.c_uint8), len(c._counts),
        ptr(c._candidate_data, ctypes.c_int32),
        len(c._candidate_data) * 4,
        ptr(short_states, ctypes.c_int32), len(short_states) * 4 - 4,
        WIDTH, HEIGHT, FLOOR, c.max_gap, c.max_width_change,
        c.max_slope_milli, c.link_tolerance_milli,
        c.reference_y_milli, c.max_per_row, c.state_capacity)
    assert result == -100 and list(short_states) == before, (
        "short state validation")
    # Candidate/state aliasing is rejected before writes as well.
    alias = array("i", [37 for _ in range(STATE_CAPACITY * 10)])
    before = list(alias)
    result = dll.cartest_path_accel_connect(
        ptr(c._counts, ctypes.c_uint8), len(c._counts),
        ptr(alias, ctypes.c_int32), len(c._candidate_data) * 4,
        ptr(alias, ctypes.c_int32), len(alias) * 4,
        WIDTH, HEIGHT, FLOOR, c.max_gap, c.max_width_change,
        c.max_slope_milli, c.link_tolerance_milli,
        c.reference_y_milli, c.max_per_row, c.state_capacity)
    assert result == -100 and list(alias) == before, "alias validation"
    # Extreme settings outside the Python-supported domain fail closed.
    c._counts[0] = 1
    c._candidate_data[1] = c._candidate_data[0] * 2 + \
        c._candidate_data[2] - 1
    for bad_slope in (80001,):
        before = list(c._state_data)
        result = dll.cartest_path_accel_connect(
            ptr(c._counts, ctypes.c_uint8), len(c._counts),
            ptr(c._candidate_data, ctypes.c_int32),
            len(c._candidate_data) * 4,
            ptr(c._state_data, ctypes.c_int32),
            len(c._state_data) * 4, WIDTH, HEIGHT, FLOOR,
            c.max_gap, c.max_width_change, bad_slope,
            c.link_tolerance_milli, c.reference_y_milli,
            c.max_per_row, c.state_capacity)
        assert result == -100 and list(c._state_data) == before, (
            "extreme setting validation")


def main():
    ref = load_reference()
    dll = make_dll()
    for variant in range(4):
        for seed in range(20):
            compare_case(ref, dll, variant, seed)
    for seed in range(40):
        compare_random(ref, dll, seed)
    compare_overflow(ref, dll)
    print("path_kernel_differential=121 cases passed")


if __name__ == "__main__":
    main()
