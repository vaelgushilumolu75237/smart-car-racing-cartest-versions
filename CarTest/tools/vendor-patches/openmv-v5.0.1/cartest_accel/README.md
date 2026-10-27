# cartest_accel prototype

This private OpenMV v5.0.1 prototype ports only the two measured topology kernels: row segment extraction and internal dark-component/hole detection. The reference behavior is frozen in `reference/fast_topology.py`; the vehicle runtime is unchanged.

## Validation

- `hosttests/build_host.py` builds `core/cartest_accel_core.c` with the local GCC into the private `CarTest/logs/cartest-accel-prototype-20261026/hostbuild` directory.
- `hosttests/test_core.py` compares all topology result fields and reusable scratch buffers against the frozen Viper functions across 40 fixtures, including mirrored branches/rings, unknown cells, queue bounds, randomized masks, and epoch rollover. It also checks invalid-buffer non-mutation and adapter fallback behavior.
- `hosttests/check_arm_binding.py` compiles the binding and C core as ARM objects with the OpenMV IDE `arm-none-eabi-gcc` against the existing OPENMV4P headers. It writes its qstr overlay, objects, compiler record, and hashes under private `logs/cartest-accel-prototype-20261026/arm-binding`.

The ARM check is object compilation only. The C module is not integrated into the OpenMV build tree, linked into firmware, flashed, or timed on camera hardware. The host differential is correctness evidence, not board performance evidence.

## API

`cartest_accel` exposes `API_VERSION = 1`, `region_segments(...)`, and `internal_component(...)`. The binding requires the pipeline's exact buffer classes/typecodes and sizes, then the core rechecks geometry, lengths, alignment, settings, and dangerous aliasing before reading or mutating scratch memory. For MicroPython 1.28, a `bytearray` buffer reports `BYTEARRAY_TYPECODE` from `py/binary.h` (currently numeric 1), while `array('i')` and `array('H')` retain their character typecodes; the binding uses the named constant instead of assuming the character `'B'`. Validation errors propagate to the caller. The Python adapter chooses the frozen Viper kernels only when the native module is missing or API-incompatible; it does not retry a rejected native call.
