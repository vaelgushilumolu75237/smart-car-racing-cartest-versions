"""Informational CPython timing for equal-work 4x2 grayscale pool kernels.

Both paths reuse their destination buffers. This is not an OpenMV/Viper frame-
rate benchmark; use the RAM benchmark on the target camera for that result.
"""

import statistics
import sys
import time
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import config
import fast_gray
from test_fast_gray_pool_equivalence import (
    GRID_HEIGHT,
    GRID_WIDTH,
    SCALE_X,
    SCALE_Y,
    SOURCE_SIZE,
    _deterministic_noise_frame,
    _reference_min_pool,
)


def _time_calls(function, count):
    samples = []
    for _ in range(count):
        started = time.perf_counter_ns()
        function()
        samples.append((time.perf_counter_ns() - started) / 1000.0)
    return {
        "median_us": statistics.median(samples),
        "p95_us": sorted(samples)[int((len(samples) - 1) * 0.95)],
        "min_us": min(samples),
        "max_us": max(samples),
    }


def main():
    raw = _deterministic_noise_frame()
    if len(raw) != SOURCE_SIZE:
        raise RuntimeError("unexpected fixed grayscale source size")
    expected = _reference_min_pool(raw)
    builder = fast_gray.FastGrayMasks(config)
    optimized_target = builder.luma
    reference_target = bytearray(GRID_WIDTH * GRID_HEIGHT)

    def optimized_host_call():
        fast_gray._min_pool_host(
            raw, optimized_target, GRID_WIDTH * SCALE_X,
            GRID_WIDTH, GRID_HEIGHT, SCALE_X, SCALE_Y)

    def reference_call():
        for y in range(GRID_HEIGHT):
            source_y = y * SCALE_Y * GRID_WIDTH * SCALE_X
            for x in range(GRID_WIDTH):
                source_x = x * SCALE_X
                minimum = 255
                for dy in range(SCALE_Y):
                    row = source_y + dy * GRID_WIDTH * SCALE_X + source_x
                    for dx in range(SCALE_X):
                        value = raw[row + dx]
                        if value < minimum:
                            minimum = value
                reference_target[y * GRID_WIDTH + x] = minimum

    # Warm both paths before collecting informational host timings.
    for _ in range(10):
        optimized_host_call()
        reference_call()
    if bytes(optimized_target) != bytes(expected):
        raise RuntimeError("host output does not match the reference")
    if bytes(reference_target) != bytes(expected):
        raise RuntimeError("nested-loop output does not match the reference")

    optimized = _time_calls(optimized_host_call, 100)
    reference = _time_calls(reference_call, 100)
    print("FAST_GRAY_POOL_HOST_BENCH")
    print("backend=%s" % fast_gray.COMPILER_BACKEND)
    print("note=CPython timing only; Viper requires a camera RAM benchmark")
    print("unrolled_host=%s" % optimized)
    print("nested_loop_host=%s" % reference)


if __name__ == "__main__":
    main()
