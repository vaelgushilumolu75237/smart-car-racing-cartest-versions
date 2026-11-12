"""ctypes differential tests against the frozen fast_topology Viper bodies."""

import ctypes
import importlib.util
import json
import random
import sys
import types
from array import array
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
ROOT = TASK.parents[3]
REFERENCE_PATH = TASK / "reference" / "fast_topology.py"
OUTPUT_DIR = ROOT / "logs" / "cartest-accel-prototype-20261026" / "hostbuild"
DLL_PATH = OUTPUT_DIR / "cartest_accel.dll"
WIDTH = 80
HEIGHT = 60
TOTAL = WIDTH * HEIGHT
FLOOR = 20
QUEUE_CAPACITY = 4800


def load_reference():
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location(
        "fast_topology_reference", REFERENCE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module._IS_MICROPYTHON = True
    return module


def buffer_pointer(buffer, ctype):
    address = ctypes.addressof(ctypes.c_char.from_buffer(buffer))
    return ctypes.cast(address, ctypes.POINTER(ctype))


def buffer_bytes(buffer):
    return memoryview(buffer).nbytes


def make_dll_api():
    dll = ctypes.CDLL(str(DLL_PATH))
    u8p = ctypes.POINTER(ctypes.c_uint8)
    u16p = ctypes.POINTER(ctypes.c_uint16)
    i32p = ctypes.POINTER(ctypes.c_int32)
    size = ctypes.c_size_t
    i32 = ctypes.c_int32
    dll.cartest_accel_region_segments.argtypes = [
        u8p, size, u8p, size, u8p, size,
        i32, i32, i32,
        u8p, size, u8p, size, u8p, size, i32p, size,
    ]
    dll.cartest_accel_region_segments.restype = i32
    dll.cartest_accel_internal_component.argtypes = [
        u8p, size, u8p, size, i32p, size,
        u16p, size, size,
        i32, i32, i32, i32, i32, i32, i32, i32, i32,
    ]
    dll.cartest_accel_internal_component.restype = i32
    return dll


class Settings:
    SURFACE_GRID_WIDTH = WIDTH
    SURFACE_GRID_HEIGHT = HEIGHT


def make_info():
    return {
        "width": WIDTH,
        "height": HEIGHT,
        "floor": FLOOR,
        "edge_mask": bytearray(TOTAL),
        "known_region": bytearray([1]) * TOTAL,
        "row_counts": bytearray(HEIGHT),
    }


def paint(info, x, y, value=1):
    info["edge_mask"][y * WIDTH + x] = value


def paint_ring(info, left, top, right, bottom):
    for x in range(left, right + 1):
        paint(info, x, top)
        paint(info, x, bottom)
    for y in range(top, bottom + 1):
        paint(info, left, y)
        paint(info, right, y)


def finish(info, mismatch=None):
    edge = info["edge_mask"]
    rows = info["row_counts"]
    for y in range(FLOOR, HEIGHT):
        offset = y * WIDTH
        rows[y] = sum(edge[offset:offset + WIDTH])
    if mismatch is not None:
        rows[mismatch] = (rows[mismatch] + 1) & 0xFF
    return info


def case_matrix():
    cases = []
    cases.append(("all_white", finish(make_info()), QUEUE_CAPACITY, 0))

    black = make_info()
    black["edge_mask"][:] = bytes([1]) * TOTAL
    cases.append(("all_black", finish(black), QUEUE_CAPACITY, 0))

    for side, left_x, right_x in (
            ("left", 10, 60), ("right", 19, 69)):
        branch = make_info()
        for y in range(28, 32):
            for x in range(WIDTH):
                paint(branch, x, y)
        for y in range(35, HEIGHT):
            paint(branch, left_x, y)
            paint(branch, right_x, y)
        cases.append(("%s_symmetric_branch" % side,
                      finish(branch), QUEUE_CAPACITY, 0))

    for side, rect in (("left", (20, 31, 33, 44)),
                       ("right", (46, 31, 59, 44))):
        island = make_info()
        paint_ring(island, *rect)
        cases.append(("%s_internal_dark_ring" % side,
                      finish(island), QUEUE_CAPACITY, 0))

    multiple = make_info()
    paint_ring(multiple, 9, 28, 21, 41)
    paint_ring(multiple, 54, 39, 68, 52)
    for y in range(24, 49):
        paint(multiple, 39, y)
    cases.append(("multiple_dark_components", finish(multiple),
                  QUEUE_CAPACITY, 0))

    diagonal = make_info()
    for y, x in ((30, 30), (31, 31), (32, 32), (33, 33),
                 (34, 34), (35, 35), (36, 36), (37, 37), (38, 38),
                 (39, 39), (40, 40), (41, 41)):
        paint(diagonal, x, y)
    cases.append(("diagonal_8_connected_component", finish(diagonal),
                  QUEUE_CAPACITY, 0))

    solid = make_info()
    for y in range(30, 43):
        for x in range(30, 43):
            paint(solid, x, y)
    cases.append(("solid_dark_patch_without_hole", finish(solid),
                  QUEUE_CAPACITY, 0))

    unknown_row = make_info()
    paint_ring(unknown_row, 28, 27, 49, 49)
    unknown_row["known_region"][38 * WIDTH:39 * WIDTH] = bytes(WIDTH)
    cases.append(("unknown_full_row_splits_segment", finish(unknown_row),
                  QUEUE_CAPACITY, 0))

    mixed = make_info()
    paint_ring(mixed, 28, 27, 49, 49)
    mixed["known_region"][38 * WIDTH + 17] = 0
    cases.append(("mixed_unknown_cell_fails_closed", finish(mixed),
                  QUEUE_CAPACITY, 0))

    unknown_all = make_info()
    unknown_all["known_region"][FLOOR * WIDTH:] = bytes(
        (HEIGHT - FLOOR) * WIDTH)
    cases.append(("all_unknown_roi", finish(unknown_all),
                  QUEUE_CAPACITY, 0))

    mismatch = make_info()
    paint_ring(mismatch, 28, 27, 49, 49)
    cases.append(("row_count_mismatch", finish(mismatch, mismatch=31),
                  QUEUE_CAPACITY, 0))

    overflow = make_info()
    for y in range(25, HEIGHT):
        paint(overflow, 37, y)
    cases.append(("logical_queue_capacity_two", finish(overflow), 2, 0))

    checkerboard = make_info()
    for y in range(FLOOR, HEIGHT):
        for x in range(WIDTH):
            if (x + y) % 2 == 0:
                paint(checkerboard, x, y)
    cases.append(("alternating_runs_max_queue", finish(checkerboard),
                  QUEUE_CAPACITY, 0))

    rng = random.Random(0xCA47E57)
    for density in (0.005, 0.03, 0.12, 0.5, 0.9, 1.0):
        for index in range(4):
            info = make_info()
            for y in range(FLOOR, HEIGHT):
                row = y * WIDTH
                for x in range(WIDTH):
                    if rng.random() < density:
                        info["edge_mask"][row + x] = 1
            if index % 2:
                unknown_y = FLOOR + 9 + index
                info["known_region"][unknown_y * WIDTH:
                                     (unknown_y + 1) * WIDTH] = bytes(WIDTH)
            cases.append(("random_d%.3f_%d" % (density, index),
                          finish(info), QUEUE_CAPACITY, 0))
    return cases


def seed_scratch(guard, seed):
    guard._trusted_rows[:] = bytes((seed + i * 3) & 0xFF
                                   for i in range(HEIGHT))
    guard._segment_starts[:] = bytes((seed + i * 5) & 0xFF
                                     for i in range(HEIGHT))
    guard._segment_ends[:] = bytes((seed + i * 7) & 0xFF
                                   for i in range(HEIGHT))
    for index in range(TOTAL):
        epoch = (seed + index * 7) % 64
        guard._visited[index] = (epoch << 2) | (index & 3)
    for index in range(len(guard._queue)):
        guard._queue[index] = (seed + index * 19) & 0xFFFF


def call_c_region(dll, guard, floor):
    return dll.cartest_accel_region_segments(
        buffer_pointer(guard._current_mask, ctypes.c_uint8),
        len(guard._current_mask),
        buffer_pointer(guard._current_known, ctypes.c_uint8),
        len(guard._current_known),
        buffer_pointer(guard._current_rows, ctypes.c_uint8),
        len(guard._current_rows),
        guard.width, guard.height, floor,
        buffer_pointer(guard._trusted_rows, ctypes.c_uint8),
        len(guard._trusted_rows),
        buffer_pointer(guard._segment_starts, ctypes.c_uint8),
        len(guard._segment_starts),
        buffer_pointer(guard._segment_ends, ctypes.c_uint8),
        len(guard._segment_ends),
        buffer_pointer(guard._segment_count_buffer, ctypes.c_int32),
        buffer_bytes(guard._segment_count_buffer))


def call_c_component(dll, guard, floor, band_end):
    return dll.cartest_accel_internal_component(
        buffer_pointer(guard._current_mask, ctypes.c_uint8),
        len(guard._current_mask),
        buffer_pointer(guard._visited, ctypes.c_uint8),
        len(guard._visited),
        buffer_pointer(guard._visit_epoch, ctypes.c_int32),
        buffer_bytes(guard._visit_epoch),
        buffer_pointer(guard._queue, ctypes.c_uint16),
        buffer_bytes(guard._queue), guard._queue_capacity,
        guard.width, guard.height, floor, band_end,
        guard.component_min_area, guard.component_max_area,
        guard.component_min_extent, guard.component_max_extent,
        guard.hole_min_area)


def accelerated_check(dll, guard, info):
    guard._segment_count_buffer[0] = 0
    guard._trusted_row_count = 0
    if not isinstance(info, dict):
        return guard._set_result(False, False, "INVALID_MASK_INFO")
    guard._current_mask = info.get("edge_mask")
    guard._current_known = info.get("known_region")
    guard._current_rows = info.get("row_counts")
    floor = info.get("floor")
    guard._current_floor = floor
    complete = call_c_region(dll, guard, floor)
    guard._trusted_row_count = guard._segment_count_buffer[1]
    if complete == -2:
        guard._segment_count_buffer[0] = 0
        return guard._set_result(False, False, "UNKNOWN_REGION",
                                  ambiguous=True)
    if complete < 0:
        return guard._set_result(False, False, "INVALID_MASK_INFO")
    segment_count = guard._segment_count_buffer[0]
    if segment_count <= 0:
        return guard._set_result(False, False, "UNKNOWN_REGION",
                                  ambiguous=True)

    island = 0
    branch = 0
    for segment in range(segment_count):
        segment_floor = guard._segment_starts[segment]
        segment_end = guard._segment_ends[segment]
        if (segment_floor < floor or segment_end > guard.height or
                segment_floor >= segment_end):
            return guard._set_result(False, False, "KERNEL_BOUNDS")
        guard._current_floor = segment_floor
        guard._current_band_end = segment_end
        segment_island = call_c_component(
            dll, guard, segment_floor, segment_end)
        if segment_island < 0:
            return guard._set_result(False, False, "KERNEL_BOUNDS")
        if segment_island:
            island = 1
            break
    if not island:
        for segment in range(segment_count):
            segment_floor = guard._segment_starts[segment]
            segment_end = guard._segment_ends[segment]
            guard._current_floor = segment_floor
            guard._current_band_end = segment_end
            segment_branch = fast_topology._open_band_viper(
                guard, segment_floor, segment_end)
            if segment_branch < 0:
                return guard._set_result(False, False, "KERNEL_BOUNDS")
            if segment_branch:
                branch = 1
                break
    if island:
        reason = "ISLAND"
    elif branch:
        reason = "OPEN_BAND"
    elif complete == 0:
        reason = "PARTIAL_KNOWN_ROI"
    else:
        reason = "OK"
    return guard._set_result(island, branch, reason, complete == 1)


def guard_state(guard, result):
    return {
        "result": tuple(sorted(dict(result).items())),
        "trusted_rows": bytes(guard._trusted_rows),
        "segment_count_buffer": tuple(guard._segment_count_buffer),
        "segment_starts": bytes(guard._segment_starts),
        "segment_ends": bytes(guard._segment_ends),
        "trusted_row_count": guard._trusted_row_count,
        "visited": bytes(guard._visited),
        "visit_epoch": tuple(guard._visit_epoch),
        "queue": tuple(guard._queue),
        "current_floor": guard._current_floor,
        "current_band_end": guard._current_band_end,
    }


def compare_fixture(dll, name, info, queue_capacity, initial_epoch):
    ref = fast_topology.FastTopologyGuard(Settings())
    accelerated = fast_topology.FastTopologyGuard(Settings())
    seed = sum(name.encode("utf-8")) & 0xFF
    seed_scratch(ref, seed)
    seed_scratch(accelerated, seed)
    ref._queue_capacity = queue_capacity
    accelerated._queue_capacity = queue_capacity
    ref._visit_epoch[0] = initial_epoch
    accelerated._visit_epoch[0] = initial_epoch

    expected = dict(ref.check(info))
    actual = dict(accelerated_check(dll, accelerated, info))
    expected_state = guard_state(ref, expected)
    actual_state = guard_state(accelerated, actual)
    if expected_state != actual_state:
        differing = [key for key in expected_state
                     if expected_state[key] != actual_state[key]]
        raise AssertionError("%s differs in %s" % (name, differing))
    return ref, accelerated


def invalid_buffer_cases(dll):
    info = finish(make_info())
    guard = fast_topology.FastTopologyGuard(Settings())
    guard._current_mask = info["edge_mask"]
    guard._current_known = info["known_region"]
    guard._current_rows = info["row_counts"]
    guard._segment_count_buffer[0] = 101
    guard._segment_count_buffer[1] = 202
    before = (bytes(info["edge_mask"]), bytes(guard._trusted_rows),
              bytes(guard._segment_starts), bytes(guard._segment_ends),
              tuple(guard._segment_count_buffer))

    ptr = buffer_pointer
    rc = dll.cartest_accel_region_segments(
        ptr(info["edge_mask"], ctypes.c_uint8), TOTAL - 1,
        ptr(info["known_region"], ctypes.c_uint8), TOTAL,
        ptr(info["row_counts"], ctypes.c_uint8), HEIGHT,
        WIDTH, HEIGHT, FLOOR,
        ptr(guard._trusted_rows, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_starts, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_ends, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_count_buffer, ctypes.c_int32),
        buffer_bytes(guard._segment_count_buffer))
    assert rc == -100
    after = (bytes(info["edge_mask"]), bytes(guard._trusted_rows),
             bytes(guard._segment_starts), bytes(guard._segment_ends),
             tuple(guard._segment_count_buffer))
    assert before == after

    # Geometry rejection and output/input alias rejection are also atomic.
    rc = dll.cartest_accel_region_segments(
        ptr(info["edge_mask"], ctypes.c_uint8), TOTAL,
        ptr(info["known_region"], ctypes.c_uint8), TOTAL,
        ptr(info["row_counts"], ctypes.c_uint8), HEIGHT,
        WIDTH - 1, HEIGHT, FLOOR,
        ptr(guard._trusted_rows, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_starts, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_ends, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_count_buffer, ctypes.c_int32),
        buffer_bytes(guard._segment_count_buffer))
    assert rc == -100
    rc = dll.cartest_accel_region_segments(
        ptr(info["edge_mask"], ctypes.c_uint8), TOTAL,
        ptr(info["known_region"], ctypes.c_uint8), TOTAL,
        ptr(info["row_counts"], ctypes.c_uint8), HEIGHT,
        WIDTH, HEIGHT, FLOOR,
        ptr(info["edge_mask"], ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_starts, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_ends, ctypes.c_uint8), HEIGHT,
        ptr(guard._segment_count_buffer, ctypes.c_int32),
        buffer_bytes(guard._segment_count_buffer))
    assert rc == -100
    assert before == (bytes(info["edge_mask"]), bytes(guard._trusted_rows),
                      bytes(guard._segment_starts), bytes(guard._segment_ends),
                      tuple(guard._segment_count_buffer))

    # Internal kernel rejects geometry, short buffers, and invalid epoch before
    # changing any output/scratch buffer.
    guard = fast_topology.FastTopologyGuard(Settings())
    mask = bytearray(TOTAL)
    guard._visit_epoch[0] = 63
    snapshot = (bytes(guard._visited), tuple(guard._visit_epoch),
                tuple(guard._queue))
    args = [
        ptr(mask, ctypes.c_uint8), TOTAL,
        ptr(guard._visited, ctypes.c_uint8), TOTAL,
        ptr(guard._visit_epoch, ctypes.c_int32), buffer_bytes(guard._visit_epoch),
        ptr(guard._queue, ctypes.c_uint16), buffer_bytes(guard._queue),
        guard._queue_capacity,
        WIDTH, HEIGHT, FLOOR, HEIGHT,
        guard.component_min_area, guard.component_max_area,
        guard.component_min_extent, guard.component_max_extent,
        guard.hole_min_area,
    ]
    rc = dll.cartest_accel_internal_component(*args[:])
    assert rc == 0
    # Restore the known state and verify each invalid call is non-mutating.
    guard._visited[:] = snapshot[0]
    guard._visit_epoch[0] = 64
    invalid_epoch_snapshot = (bytes(guard._visited), tuple(guard._visit_epoch),
                              tuple(guard._queue))
    args[4] = ptr(guard._visit_epoch, ctypes.c_int32)
    rc = dll.cartest_accel_internal_component(*args)
    assert rc == -100
    assert invalid_epoch_snapshot == (bytes(guard._visited),
                                      tuple(guard._visit_epoch),
                                      tuple(guard._queue))
    guard._visit_epoch[0] = 63
    invalid_snapshot = (bytes(guard._visited), tuple(guard._visit_epoch),
                        tuple(guard._queue))
    args[0] = ptr(mask, ctypes.c_uint8)
    args[1] = TOTAL - 1
    rc = dll.cartest_accel_internal_component(*args)
    assert rc == -100
    assert invalid_snapshot == (bytes(guard._visited),
                                tuple(guard._visit_epoch),
                                tuple(guard._queue))


def adapter_fallback_smoke():
    openmv_dir = TASK.parents[3] / "openmv"
    adapter_path = TASK / "python" / "cartest_accel_adapter.py"
    sys.path.insert(0, str(openmv_dir))
    sys.modules["fast_topology"] = fast_topology
    sys.modules["cartest_accel"] = None
    spec = importlib.util.spec_from_file_location(
        "cartest_accel_adapter_under_test", adapter_path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    adapter.reset_backend_cache()

    info = case_matrix()[4][1]
    reference = fast_topology.FastTopologyGuard(Settings())
    fallback = fast_topology.FastTopologyGuard(Settings())
    for guard in (reference, fallback):
        guard._current_mask = info["edge_mask"]
        guard._current_known = info["known_region"]
        guard._current_rows = info["row_counts"]
    original_region = fast_topology._region_segments_viper
    original_internal = fast_topology._internal_component_viper
    expected = original_region(reference, FLOOR)
    # Model production dispatch replacing the established Viper call-site
    # symbols. The adapter's fallback must still reach the saved originals.
    fast_topology._region_segments_viper = adapter.region_segments
    fast_topology._internal_component_viper = adapter.internal_component
    actual = fast_topology._region_segments_viper(fallback, FLOOR)
    assert expected == actual
    assert bytes(reference._trusted_rows) == bytes(fallback._trusted_rows)
    assert tuple(reference._segment_count_buffer) == tuple(
        fallback._segment_count_buffer)
    assert bytes(reference._segment_starts) == bytes(fallback._segment_starts)
    assert bytes(reference._segment_ends) == bytes(fallback._segment_ends)

    internal_reference = fast_topology.FastTopologyGuard(Settings())
    internal_fallback = fast_topology.FastTopologyGuard(Settings())
    for guard in (internal_reference, internal_fallback):
        guard._current_mask = info["edge_mask"]
        guard._current_floor = FLOOR
        guard._current_band_end = HEIGHT
    expected = original_internal(internal_reference, FLOOR, HEIGHT)
    actual = fast_topology._internal_component_viper(
        internal_fallback, FLOOR, HEIGHT)
    assert expected == actual
    assert bytes(internal_reference._visited) == bytes(internal_fallback._visited)
    assert tuple(internal_reference._visit_epoch) == tuple(
        internal_fallback._visit_epoch)
    assert tuple(internal_reference._queue) == tuple(internal_fallback._queue)

    # A present API-1 native module that rejects its buffers must surface the
    # error. It may not fall through to the unchecked Viper implementation.
    rejecting = types.ModuleType("cartest_accel")
    rejecting.API_VERSION = 1
    def reject(*_args):
        raise ValueError("native validation rejection")
    rejecting.region_segments = reject
    rejecting.internal_component = reject
    sys.modules["cartest_accel"] = rejecting
    adapter.reset_backend_cache()
    try:
        try:
            adapter.region_segments(fallback, FLOOR)
        except ValueError as exc:
            assert str(exc) == "native validation rejection"
        else:
            raise AssertionError("native region validation error was hidden")
        try:
            adapter.internal_component(internal_fallback, FLOOR, HEIGHT)
        except ValueError as exc:
            assert str(exc) == "native validation rejection"
        else:
            raise AssertionError("native component validation error was hidden")
    finally:
        fast_topology._region_segments_viper = original_region
        fast_topology._internal_component_viper = original_internal
    adapter.reset_backend_cache()
    sys.modules.pop("cartest_accel", None)


def main():
    global fast_topology
    fast_topology = load_reference()
    dll = make_dll_api()
    results = []
    for name, info, queue_capacity, epoch in case_matrix():
        compare_fixture(dll, name, info, queue_capacity, epoch)
        results.append(name)

    # Explicit epoch rollover sequence: advance through a full epoch wrap while
    # comparing every result and scratch byte/entry after each call.
    wrap_info = finish(make_info())
    paint_ring(wrap_info, 30, 31, 45, 46)
    finish(wrap_info)
    ref, accelerated = compare_fixture(
        dll, "epoch_rollover", wrap_info, QUEUE_CAPACITY, 61)
    for index in range(70):
        expected = dict(ref.check(wrap_info))
        actual = dict(accelerated_check(dll, accelerated, wrap_info))
        if guard_state(ref, expected) != guard_state(accelerated, actual):
            raise AssertionError("epoch rollover differs at step %d" % index)
    results.append("epoch_rollover_70_frames")

    invalid_buffer_cases(dll)
    adapter_fallback_smoke()
    report = {
        "reference": str(REFERENCE_PATH),
        "c_core_dll": str(DLL_PATH),
        "differential_fixture_count": len(results),
        "differential_passed": results,
        "invalid_buffer_cases": [
            "short region mask", "bad geometry", "aliased region output",
            "invalid epoch", "short internal mask"],
        "adapter_no_module_fallback": "Viper region/internal results and scratch exact",
        "adapter_installed_dispatch_fallback": "frozen Viper references prevent recursion",
        "adapter_native_validation": "API-1 validation errors propagate without Viper fallback",
        "checks": "full topology result plus trusted rows, segments, epoch, "
                  "visited bytes, queue entries, and current band state",
    }
    output = OUTPUT_DIR / "results.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
