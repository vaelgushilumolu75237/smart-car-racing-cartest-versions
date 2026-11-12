"""Private optional acceleration adapter for the unchanged fast_topology API.

The production integration point is deliberately left to the lead agent. If
the optional native module is unavailable or has another API version, the
helpers call the current Viper kernels directly with the original arguments.
"""

import fast_topology


# Keep stable references so an integration that patches the established Viper
# symbols to this adapter cannot make the no-module fallback recurse.
_ORIGINAL_REGION_SEGMENTS = fast_topology._region_segments_viper
_ORIGINAL_INTERNAL_COMPONENT = fast_topology._internal_component_viper
_UNRESOLVED = object()
_ACCEL = _UNRESOLVED


def _backend():
    global _ACCEL
    if _ACCEL is _UNRESOLVED:
        try:
            module = __import__("cartest_accel")
        except ImportError:
            module = None
        if (module is not None and
                getattr(module, "API_VERSION", None) != 1):
            module = None
        if (module is not None and
                (not callable(getattr(module, "region_segments", None)) or
                 not callable(getattr(module, "internal_component", None)))):
            module = None
        _ACCEL = module
    return _ACCEL


def region_segments(guard, floor):
    """Use API 1 when available; otherwise call the original Viper kernel."""
    module = _backend()
    if module is None:
        return _ORIGINAL_REGION_SEGMENTS(guard, floor)
    # Native validation errors must fail closed; never retry with the less
    # defensive historical Viper implementation.
    return module.region_segments(
        guard._current_mask,
        guard._current_known,
        guard._current_rows,
        floor,
        guard._trusted_rows,
        guard._segment_starts,
        guard._segment_ends,
        guard._segment_count_buffer)


def internal_component(guard, floor, band_end):
    """Use API 1 when available; otherwise call the original Viper kernel."""
    module = _backend()
    if module is None:
        return _ORIGINAL_INTERNAL_COMPONENT(guard, floor, band_end)
    # Native validation errors must fail closed; only absent or incompatible
    # modules select the Viper fallback above.
    return module.internal_component(
        guard._current_mask,
        guard._visited,
        guard._visit_epoch,
        guard._queue,
        floor,
        band_end,
        guard._queue_capacity,
        guard.component_min_area,
        guard.component_max_area,
        guard.component_min_extent,
        guard.component_max_extent,
        guard.hole_min_area)


def reset_backend_cache():
    """Reset only the optional module lookup for host/API tests or restart."""
    global _ACCEL
    _ACCEL = _UNRESOLVED
