"""Small topology guard for the fast gray boundary mask.

This module retains only the independent island/hole and open-band rejection
checks from the surface detector. It does not search seeds or trace paths.
The default FastGrayMasks config uses three full-width trusted blocks, but
custom block rectangles can leave partially-known rows. Only consecutive
fully-known rows form inspected bands; completely unknown rows are crop
boundaries and are never interpreted as dark or white. A partial result
reports PARTIAL_KNOWN_ROI. Mixed-known rows fail closed as UNKNOWN_REGION,
because the path picker could otherwise use rows that topology did not check.
Zero fully-known rows also fail closed as UNKNOWN_REGION.
The guard reuses its visited map, queue, row-segment buffers, and result dict.
Input mask-info buffers belong to the current frame and may be overwritten by
the next mask build.
"""

RUNTIME_API_VERSION = 7

from array import array

try:
    import micropython
    COMPILER_BACKEND = "micropython-viper"
    _IS_MICROPYTHON = True
except ImportError:
    _IS_MICROPYTHON = False
    COMPILER_BACKEND = "cpython-shim"

    class _MicroPythonHostShim:
        def viper(self, function):
            return function

        def native(self, function):
            return function

    micropython = _MicroPythonHostShim()

    def ptr8(buffer):
        return buffer

    def ptr16(buffer):
        return buffer

    def ptr32(buffer):
        if isinstance(buffer, bytearray):
            return _HostByteWords(buffer)
        return buffer

    class _HostByteWords:
        """Read-only little-endian word view for CPython Viper shims."""

        def __init__(self, buffer):
            self._buffer = buffer

        def __getitem__(self, index):
            offset = index * 4
            buffer = self._buffer
            return (buffer[offset] | (buffer[offset + 1] << 8) |
                    (buffer[offset + 2] << 16) |
                    (buffer[offset + 3] << 24))

    uint = int


# Optional native backend. Keep the established compiler/backend flags above
# intact: this describes only the additional topology C module.
NATIVE_ACCEL_API = 1
NATIVE_ACCEL_API_VERSION = 0
NATIVE_ACCEL_BACKEND = "viper-module-unavailable"
_NATIVE_ACCEL = None
try:
    import cartest_accel as _cartest_accel_module
except ImportError:
    _cartest_accel_module = None
else:
    NATIVE_ACCEL_API_VERSION = getattr(
        _cartest_accel_module, "API_VERSION", 0)
    if NATIVE_ACCEL_API_VERSION != NATIVE_ACCEL_API:
        NATIVE_ACCEL_BACKEND = "viper-api-mismatch"
    elif (not callable(getattr(_cartest_accel_module,
                               "region_segments", None)) or
          not callable(getattr(_cartest_accel_module,
                               "internal_component", None))):
        NATIVE_ACCEL_BACKEND = "viper-methods-incomplete"
    else:
        _NATIVE_ACCEL = _cartest_accel_module
        NATIVE_ACCEL_BACKEND = "cartest_accel-api1"


@micropython.viper
def _region_segments_viper(self, floor: int) -> int:
    """Validate rows and record contiguous fully-known bands.

    Returns 1 for a fully known ROI, 0 for a row-granularity partial ROI,
    -1 for a row count mismatch, or -2 for a mixed-known row. Fully unknown
    rows break a trusted band; mixed rows fail closed because the path picker
    could otherwise use uninspected rows.
    """
    known = ptr8(self._current_known)
    mask = ptr8(self._current_mask)
    rows = ptr8(self._current_rows)
    trusted_rows = ptr8(self._trusted_rows)
    starts = ptr8(self._segment_starts)
    ends = ptr8(self._segment_ends)
    segment_count_out = ptr32(self._segment_count_buffer)
    width = int(self.width)
    height = int(self.height)
    segment_count = 0
    segment_start = -1
    all_known = 1
    partial_rows = 0
    segment_count_out[0] = 0
    segment_count_out[1] = 0
    trusted_row_count = 0
    for y in range(floor, height):
        # Every ROI row is written exactly once below. Clearing each row here
        # lets the pixel scan establish both its fresh trust bit and the count
        # without a separate pre-pass over the same rows.
        trusted_rows[y] = 0
        offset = y * width
        count = 0
        known_count = 0
        complete_row = 1
        for x in range(width):
            index = offset + x
            if mask[index] != 0:
                count += 1
            if known[index] == 0:
                complete_row = 0
                all_known = 0
            else:
                known_count += 1
        if rows[y] != count:
            return -1
        if known_count != 0 and known_count != width:
            partial_rows = 1
        if complete_row != 0:
            trusted_rows[y] = 1
            trusted_row_count += 1
            if segment_start < 0:
                segment_start = y
        else:
            if segment_start >= 0:
                starts[segment_count] = segment_start
                ends[segment_count] = y
                segment_count += 1
                segment_start = -1
    if segment_start >= 0:
        starts[segment_count] = segment_start
        ends[segment_count] = height
        segment_count += 1
    if partial_rows != 0:
        segment_count_out[0] = 0
        return -2
    segment_count_out[0] = segment_count
    segment_count_out[1] = trusted_row_count
    if segment_count == 0:
        return 0
    if all_known != 0:
        return 1
    return 0


@micropython.viper
def _paired_boundary_row_viper(self, y: int) -> int:
    """Find separated narrow dark runs on opposite sides of frame center."""
    mask = ptr8(self._current_mask)
    width = int(self.width)
    max_run = int(self.open_max_run_width)
    min_gap = int(self.open_min_pair_gap)
    center_twice = width - 1
    row = y * width
    x = 0
    max_left_end = -1
    while x < width:
        if mask[row + x] == 0:
            x += 1
            continue
        run_start = x
        while x < width and mask[row + x] != 0:
            x += 1
        run_end = x - 1
        if run_end - run_start + 1 > max_run:
            continue
        if run_start * 2 < center_twice and run_end > max_left_end:
            max_left_end = run_end
        if run_end * 2 > center_twice and max_left_end >= 0:
            if run_start - max_left_end - 1 >= min_gap:
                return 1
    return 0


@micropython.viper
def _paired_support_viper(self, start_y: int, end_y: int) -> int:
    for y in range(start_y, end_y + 1):
        if _paired_boundary_row_viper(self, y):
            return 1
    return 0


@micropython.viper
def _wide_band_viper(self, start_y: int, end_y: int,
                     backwards: int) -> int:
    """Return true for a wide run separated from the corresponding crop."""
    rows = ptr8(self._current_rows)
    floor = int(self._current_floor)
    band_end = int(self._current_band_end)
    wide_rows = int(self.open_wide_rows)
    wide_limit = int(self.width) * 3 // 4
    step = -1 if backwards else 1
    y = start_y
    streak = 0
    while (y >= end_y if backwards else y <= end_y):
        if rows[y] >= wide_limit:
            streak += 1
            if streak >= wide_rows:
                edge_y = floor if backwards else band_end - 1
                probe_y = y
                edge_connected = 1
                while probe_y != edge_y:
                    probe_y += step
                    if rows[probe_y] < wide_limit:
                        edge_connected = 0
                        break
                if edge_connected == 0:
                    return 1
                return 0
        else:
            streak = 0
        y += step
    return 0


@micropython.viper
def _open_band_viper(self, floor: int, band_end: int) -> int:
    """Detect supported, interior zero-boundary bands without list storage."""
    rows = ptr8(self._current_rows)
    min_rows = int(self.open_min_rows)
    support_distance = int(self.open_support_rows)
    y = floor
    while y < band_end:
        if rows[y] != 0:
            y += 1
            continue
        start = y
        while y < band_end and rows[y] == 0:
            y += 1
        end = y - 1
        if end - start + 1 < min_rows:
            continue
        if start <= floor or end >= band_end - 1:
            continue

        before = start - support_distance
        if before < floor:
            before = floor
        after = end + support_distance
        if after >= band_end:
            after = band_end - 1
        paired_before = _paired_support_viper(self, before, start - 1)
        paired_after = _paired_support_viper(self, end + 1, after)
        wide_before = _wide_band_viper(self, start - 1, before, 1)
        wide_after = _wide_band_viper(self, end + 1, after, 0)
        if ((wide_before and paired_after) or
                (paired_before and wide_after)):
            return 1
    return 0


@micropython.viper
def _internal_component_viper(self, floor: int, band_end: int) -> int:
    """Find a bounded 8-connected dark contour with a 4-connected hole."""
    mask = ptr8(self._current_mask)
    visited = ptr8(self._visited)
    mask_words = ptr32(self._current_mask)
    visited_words = ptr32(self._visited)
    queue = ptr16(self._queue)
    queue_capacity = int(self._queue_capacity)
    epoch_buffer = ptr32(self._visit_epoch)
    width = int(self.width)
    height = int(self.height)
    total = width * height
    min_area = int(self.component_min_area)
    max_area = int(self.component_max_area)
    min_extent = int(self.component_min_extent)
    max_extent = int(self.component_max_extent)
    min_hole_area = int(self.hole_min_area)

    # The high six bits identify this invocation; the low two bits retain the
    # queued/processed states. This avoids clearing the 80x60 map every frame.
    # Clear the full map only when the six-bit epoch wraps.
    epoch = int(epoch_buffer[0]) + 1
    if epoch >= 64:
        for index in range(total):
            visited[index] = 0
        epoch = 1
    epoch_buffer[0] = epoch
    visit_stamp = epoch << 2
    visit_word_stamp = uint(visit_stamp) * uint(0x01010101)

    for start_y in range(floor, band_end):
        start_row_offset = start_y * width
        start_x = 0
        while start_x < width:
            # Rows start on a four-byte boundary (the grid width is 80). A
            # zero mask word or a word already stamped in all four bytes has
            # no candidate seed, so advance directly to the next word. Mixed
            # words retain the exact bytewise test and scan order below.
            if (start_x & 3) == 0 and start_x + 3 < width:
                word_index = (start_row_offset + start_x) >> 2
                mask_word = uint(mask_words[word_index])
                visited_word = uint(visited_words[word_index])
                if (mask_word == uint(0) or
                        (visited_word & uint(0xFCFCFCFC)) ==
                        visit_word_stamp):
                    start_x += 4
                    continue
            start = start_row_offset + start_x
            if (mask[start] == 0 or
                    (visited[start] >> 2) == epoch):
                start_x += 1
                continue

            # Seed one horizontal dark run. The queue stores one index per
            # run, not one index per pixel; 8-connectivity is preserved by
            # joining runs whose x intervals touch within one pixel on an
            # adjacent row.
            row_offset = start_row_offset
            run_left = start_x
            while run_left > 0 and mask[row_offset + run_left - 1] != 0:
                run_left -= 1
            run_right = start_x
            while run_right + 1 < width and \
                    mask[row_offset + run_right + 1] != 0:
                run_right += 1
            seed_run_right = run_right
            for x in range(run_left, run_right + 1):
                visited[row_offset + x] = visit_stamp | 1
            queue_count = 1
            # Keep both ends of each queued run in the existing 16-bit queue.
            # The grid has at most 2,400 alternating dark runs, so two entries
            # per run fit exactly in the 4,800-entry scratch array.
            queue[0] = row_offset + run_left
            queue[1] = row_offset + run_right
            queue_head = 0
            area = 0
            touches_roi = 0
            min_x = width
            max_x = -1
            min_y = height
            max_y = -1

            while queue_head < queue_count:
                queue_slot = queue_head * 2
                index = int(queue[queue_slot])
                run_end_index = int(queue[queue_slot + 1])
                queue_head += 1
                y = index // width
                row_offset = y * width
                run_left = index - row_offset
                run_right = run_end_index - row_offset
                area += run_right - run_left + 1
                if run_left < min_x:
                    min_x = run_left
                if run_right > max_x:
                    max_x = run_right
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
                if (run_left == 0 or run_right == width - 1 or
                        y == floor or y == band_end - 1):
                    touches_roi = 1
                for direction in range(2):
                    ny = y - 1 if direction == 0 else y + 1
                    if ny < floor or ny >= band_end:
                        continue
                    neighbor_row = ny * width
                    nx = run_left - 1
                    if nx < 0:
                        nx = 0
                    nx_limit = run_right + 1
                    if nx_limit >= width:
                        nx_limit = width - 1
                    while nx <= nx_limit:
                        neighbor = neighbor_row + nx
                        if (mask[neighbor] != 0 and
                                (visited[neighbor] >> 2) != epoch):
                            next_left = nx
                            while next_left > 0 and \
                                    mask[neighbor_row + next_left - 1] != 0:
                                next_left -= 1
                            next_right = nx
                            while next_right + 1 < width and \
                                    mask[neighbor_row + next_right + 1] != 0:
                                next_right += 1
                            if queue_count * 2 + 1 >= queue_capacity:
                                return -1
                            queue_slot = queue_count * 2
                            queue[queue_slot] = neighbor_row + next_left
                            queue[queue_slot + 1] = (
                                neighbor_row + next_right)
                            queue_count += 1
                            for px in range(next_left, next_right + 1):
                                visited[neighbor_row + px] = visit_stamp | 1
                            nx = next_right + 1
                        else:
                            nx += 1

            box_width = max_x - min_x + 1
            box_height = max_y - min_y + 1
            if (touches_roi != 0 or area < min_area or area > max_area or
                    box_width < min_extent or box_height < min_extent or
                    box_width > max_extent or box_height > max_extent):
                # Every pixel through this seed run was stamped at enqueue,
                # so no other 8-connected component can begin inside it.
                start_x = seed_run_right + 1
                continue

            x0 = min_x - 1
            if x0 < 0:
                x0 = 0
            y0 = min_y - 1
            if y0 < floor:
                y0 = floor
            x1 = max_x + 1
            if x1 >= width:
                x1 = width - 1
            y1 = max_y + 1
            if y1 >= band_end:
                y1 = band_end - 1

            # Flood zero regions as horizontal runs. The queue stores the
            # start and end index per run, so 4-connectivity is preserved
            # without a queue entry and coordinate division per pixel.
            queue_count = 0
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    if (x != x0 and x != x1 and
                            y != y0 and y != y1):
                        continue
                    index = y * width + x
                    if (mask[index] != 0 or
                            (visited[index] >> 2) == epoch):
                        continue
                    run_left = x
                    while run_left > x0:
                        neighbor = y * width + run_left - 1
                        if (mask[neighbor] != 0 or
                                (visited[neighbor] >> 2) == epoch):
                            break
                        run_left -= 1
                    run_right = x
                    while run_right < x1:
                        neighbor = y * width + run_right + 1
                        if (mask[neighbor] != 0 or
                                (visited[neighbor] >> 2) == epoch):
                            break
                        run_right += 1
                    if queue_count * 2 + 1 >= queue_capacity:
                        return -1
                    queue[queue_count * 2] = y * width + run_left
                    queue[queue_count * 2 + 1] = y * width + run_right
                    queue_count += 1
                    for px in range(run_left, run_right + 1):
                        visited[y * width + px] = visit_stamp | 1

            queue_head = 0
            while queue_head < queue_count:
                index = int(queue[queue_head * 2])
                run_end_index = int(queue[queue_head * 2 + 1])
                queue_head += 1
                y = index // width
                row_offset = y * width
                run_left = index - row_offset
                run_right = run_end_index - row_offset

                for direction in range(2):
                    ny = y - 1 if direction == 0 else y + 1
                    if ny < y0 or ny > y1:
                        continue
                    neighbor_row = ny * width
                    nx = run_left
                    while nx <= run_right:
                        neighbor = neighbor_row + nx
                        if (mask[neighbor] == 0 and
                                (visited[neighbor] >> 2) != epoch):
                            next_left = nx
                            while next_left > x0:
                                neighbor = neighbor_row + next_left - 1
                                if (mask[neighbor] != 0 or
                                        (visited[neighbor] >> 2) == epoch):
                                    break
                                next_left -= 1
                            next_right = nx
                            while next_right < x1:
                                neighbor = neighbor_row + next_right + 1
                                if (mask[neighbor] != 0 or
                                        (visited[neighbor] >> 2) == epoch):
                                    break
                                next_right += 1
                            if queue_count * 2 + 1 >= queue_capacity:
                                return -1
                            queue[queue_count * 2] = (
                                neighbor_row + next_left)
                            queue[queue_count * 2 + 1] = (
                                neighbor_row + next_right)
                            queue_count += 1
                            for px in range(next_left, next_right + 1):
                                visited[neighbor_row + px] = visit_stamp | 1
                            nx = next_right + 1
                        else:
                            nx += 1

            # Count interior zero components with the same 4-connected run
            # flood. Runs cannot cross the padded border, just as the prior
            # pixel flood's neighbor checks could not cross it.
            inner_left = x0 + 1
            inner_right = x1 - 1
            for y in range(y0 + 1, y1):
                x = inner_left
                while x <= inner_right:
                    start = y * width + x
                    if (mask[start] != 0 or
                            (visited[start] >> 2) == epoch):
                        x += 1
                        continue
                    run_left = x
                    while run_left > inner_left:
                        neighbor = y * width + run_left - 1
                        if (mask[neighbor] != 0 or
                                (visited[neighbor] >> 2) == epoch):
                            break
                        run_left -= 1
                    run_right = x
                    while run_right < inner_right:
                        neighbor = y * width + run_right + 1
                        if (mask[neighbor] != 0 or
                                (visited[neighbor] >> 2) == epoch):
                            break
                        run_right += 1
                    seed_right = run_right
                    for px in range(run_left, run_right + 1):
                        visited[y * width + px] = visit_stamp | 1
                    queue_count = 1
                    queue[0] = y * width + run_left
                    queue[1] = y * width + run_right
                    queue_head = 0
                    hole_area = 0
                    while queue_head < queue_count:
                        index = int(queue[queue_head * 2])
                        run_end_index = int(queue[queue_head * 2 + 1])
                        queue_head += 1
                        py = index // width
                        row_offset = py * width
                        run_left = index - row_offset
                        run_right = run_end_index - row_offset
                        hole_area += run_right - run_left + 1
                        for direction in range(2):
                            ny = py - 1 if direction == 0 else py + 1
                            if ny <= y0 or ny >= y1:
                                continue
                            neighbor_row = ny * width
                            nx = run_left
                            while nx <= run_right:
                                neighbor = neighbor_row + nx
                                if (mask[neighbor] == 0 and
                                        (visited[neighbor] >> 2) != epoch):
                                    next_left = nx
                                    while next_left > inner_left:
                                        neighbor = (neighbor_row +
                                                    next_left - 1)
                                        if (mask[neighbor] != 0 or
                                                (visited[neighbor] >> 2) ==
                                                epoch):
                                            break
                                        next_left -= 1
                                    next_right = nx
                                    while next_right < inner_right:
                                        neighbor = (neighbor_row +
                                                    next_right + 1)
                                        if (mask[neighbor] != 0 or
                                                (visited[neighbor] >> 2) ==
                                                epoch):
                                            break
                                        next_right += 1
                                    if queue_count * 2 + 1 >= queue_capacity:
                                        return -1
                                    queue[queue_count * 2] = (
                                        neighbor_row + next_left)
                                    queue[queue_count * 2 + 1] = (
                                        neighbor_row + next_right)
                                    queue_count += 1
                                    for px in range(next_left,
                                                    next_right + 1):
                                        visited[neighbor_row + px] = (
                                            visit_stamp | 1)
                                    nx = next_right + 1
                                else:
                                    nx += 1
                    if hole_area >= min_hole_area:
                        return 1
                    x = seed_right + 1
            start_x += 1
    return 0


def _region_segments_dispatch(self, floor):
    native = _NATIVE_ACCEL
    if native is not None and self.width == 80 and self.height == 60:
        # Do not catch native validation exceptions: the calling main loop must
        # fail closed instead of retrying with a less defensive kernel.
        return native.region_segments(
            self._current_mask,
            self._current_known,
            self._current_rows,
            floor,
            self._trusted_rows,
            self._segment_starts,
            self._segment_ends,
            self._segment_count_buffer)
    if _IS_MICROPYTHON:
        return _region_segments_viper(self, floor)
    return self._region_segments_host(floor)


def _internal_component_dispatch(self, floor, band_end):
    native = _NATIVE_ACCEL
    if native is not None and self.width == 80 and self.height == 60:
        # The native binding validates all exact buffer types, lengths, and
        # settings before entering C. Any rejection propagates to main.
        return native.internal_component(
            self._current_mask,
            self._visited,
            self._visit_epoch,
            self._queue,
            floor,
            band_end,
            self._queue_capacity,
            self.component_min_area,
            self.component_max_area,
            self.component_min_extent,
            self.component_max_extent,
            self.hole_min_area)
    if _IS_MICROPYTHON:
        return _internal_component_viper(self, floor, band_end)
    return self._internal_component_runs_host(floor, band_end)


def _value(settings, name, default):
    if isinstance(settings, dict):
        return settings.get(name, default)
    return getattr(settings, name, default)


class FastTopologyGuard:
    """Check internal mask islands and supported open boundary bands."""

    def __init__(self, settings):
        self.width = _value(settings, "SURFACE_GRID_WIDTH", 80)
        self.height = _value(settings, "SURFACE_GRID_HEIGHT", 60)
        if self.width != 80 or self.height != 60:
            raise ValueError("fast_topology requires an 80x60 grid")

        self.component_min_area = _value(
            settings, "SURFACE_INTERNAL_COMPONENT_MIN_AREA", 12)
        self.component_min_extent = _value(
            settings, "SURFACE_INTERNAL_COMPONENT_MIN_EXTENT", 3)
        self.component_max_extent = _value(
            settings, "SURFACE_INTERNAL_COMPONENT_MAX_EXTENT", 18)
        self.component_max_area = _value(
            settings, "SURFACE_INTERNAL_COMPONENT_MAX_AREA", 180)
        self.hole_min_area = _value(settings, "SURFACE_INTERNAL_HOLE_MIN_AREA", 4)
        self.open_min_rows = _value(settings, "SURFACE_OPEN_BAND_MIN_ROWS", 3)
        self.open_support_rows = _value(
            settings, "SURFACE_OPEN_BAND_SUPPORT_ROWS", 5)
        self.open_max_run_width = _value(
            settings, "SURFACE_OPEN_BAND_MAX_RUN_WIDTH", 36)
        self.open_min_pair_gap = _value(
            settings, "SURFACE_OPEN_BAND_MIN_PAIR_GAP", 8)
        self.open_wide_rows = _value(
            settings, "SURFACE_OPEN_BAND_WIDE_ROWS", 4)
        self._validate_settings()

        total = self.width * self.height
        self._visited = bytearray(total)
        self._visit_epoch = array("i", [0])
        # The run queue stores two 16-bit endpoints per run. A row can have
        # ceil(width/2) separated runs, so odd-width grids need more than one
        # queue slot per pixel. This formula leaves the existing 80x60
        # allocation at 4,800 entries while covering a 5x5 checkerboard's
        # 13 runs (26 endpoint entries).
        self._queue_capacity = 2 * ((self.width + 1) // 2) * self.height
        self._queue = array("H", [0] * self._queue_capacity)
        self._trusted_rows = bytearray(self.height)
        self._segment_starts = bytearray(self.height)
        self._segment_ends = bytearray(self.height)
        # Slot 0 is the trusted-band count. Slot 1 is the number of fully
        # known rows, produced by the Viper region scan to avoid rescanning
        # the trust bitmap in Python on every frame.
        self._segment_count_buffer = array("i", [0, 0])
        self._trusted_row_count = 0
        self._current_mask = None
        self._current_known = None
        self._current_rows = None
        self._current_floor = 20
        self._current_band_end = self.height
        self._result = {
            "island_evidence": False,
            "branch_evidence": False,
            "ambiguous": False,
            "reason": "NOT_RUN",
            "complete": False,
            "trusted_segment_count": 0,
            "fully_known_rows": 0,
        }

    def _validate_settings(self):
        integer_settings = (
            ("SURFACE_INTERNAL_COMPONENT_MIN_AREA",
             self.component_min_area, 1, self.width * self.height),
            ("SURFACE_INTERNAL_COMPONENT_MIN_EXTENT",
             self.component_min_extent, 1, self.width),
            ("SURFACE_INTERNAL_COMPONENT_MAX_EXTENT",
             self.component_max_extent, 1, self.width),
            ("SURFACE_INTERNAL_COMPONENT_MAX_AREA",
             self.component_max_area, 1, self.width * self.height),
            ("SURFACE_INTERNAL_HOLE_MIN_AREA",
             self.hole_min_area, 1, self.width * self.height),
            ("SURFACE_OPEN_BAND_MIN_ROWS",
             self.open_min_rows, 1, self.height),
            ("SURFACE_OPEN_BAND_SUPPORT_ROWS",
             self.open_support_rows, 0, self.height),
            ("SURFACE_OPEN_BAND_MAX_RUN_WIDTH",
             self.open_max_run_width, 1, self.width),
            ("SURFACE_OPEN_BAND_MIN_PAIR_GAP",
             self.open_min_pair_gap, 0, self.width),
            ("SURFACE_OPEN_BAND_WIDE_ROWS",
             self.open_wide_rows, 1, self.height),
        )
        for name, value, minimum, maximum in integer_settings:
            if (isinstance(value, bool) or not isinstance(value, int) or
                    value < minimum or value > maximum):
                raise ValueError("%s must be an integer in %d..%d" %
                                 (name, minimum, maximum))
        if self.component_min_extent > self.component_max_extent:
            raise ValueError("internal component extents are inverted")
        if self.component_min_area > self.component_max_area:
            raise ValueError("internal component areas are inverted")

    def _region_segments_host(self, floor):
        known = self._current_known
        mask = self._current_mask
        rows = self._current_rows
        trusted_rows = self._trusted_rows
        starts = self._segment_starts
        ends = self._segment_ends
        for y in range(self.height):
            trusted_rows[y] = 0
            starts[y] = 0
            ends[y] = 0
        self._segment_count_buffer[0] = 0
        segment_count = 0
        segment_start = None
        all_known = True
        partial_rows = False
        for y in range(floor, self.height):
            offset = y * self.width
            count = 0
            known_count = 0
            complete_row = True
            for x in range(self.width):
                index = offset + x
                if mask[index]:
                    count += 1
                if not known[index]:
                    complete_row = False
                    all_known = False
                else:
                    known_count += 1
            if rows[y] != count:
                return -1
            if known_count and known_count != self.width:
                partial_rows = True
            if complete_row:
                trusted_rows[y] = 1
                if segment_start is None:
                    segment_start = y
            elif segment_start is not None:
                starts[segment_count] = segment_start
                ends[segment_count] = y
                segment_count += 1
                segment_start = None
        if segment_start is not None:
            starts[segment_count] = segment_start
            ends[segment_count] = self.height
            segment_count += 1
        if partial_rows:
            self._segment_count_buffer[0] = 0
            return -2
        self._segment_count_buffer[0] = segment_count
        if not segment_count:
            return 0
        return 1 if all_known else 0

    def _component_encloses_hole_host(self, bounds, floor, band_end=None):
        mask = self._current_mask
        visited = self._visited
        queue = self._queue
        width = self.width
        height = self.height
        if band_end is None:
            band_end = height
        x0, y0, x1, y1 = bounds
        x0 = max(0, x0 - 1)
        y0 = max(floor, y0 - 1)
        x1 = min(width - 1, x1 + 1)
        y1 = min(band_end - 1, y1 + 1)

        queue_count = 0
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if x != x0 and x != x1 and y != y0 and y != y1:
                    continue
                index = y * width + x
                if mask[index] or visited[index]:
                    continue
                if queue_count >= len(queue):
                    return -1
                visited[index] = 1
                queue[queue_count] = index
                queue_count += 1
        head = 0
        while head < queue_count:
            index = queue[head]
            head += 1
            x = index % width
            y = index // width
            for nx, ny in ((x - 1, y), (x + 1, y),
                           (x, y - 1), (x, y + 1)):
                if nx < x0 or nx > x1 or ny < y0 or ny > y1:
                    continue
                neighbor = ny * width + nx
                if mask[neighbor] or visited[neighbor]:
                    continue
                if queue_count >= len(queue):
                    return -1
                visited[neighbor] = 1
                queue[queue_count] = neighbor
                queue_count += 1

        for y in range(y0 + 1, y1):
            for x in range(x0 + 1, x1):
                start = y * width + x
                if mask[start] or visited[start]:
                    continue
                visited[start] = 1
                queue_count = 1
                queue[0] = start
                head = 0
                area = 0
                while head < queue_count:
                    index = queue[head]
                    head += 1
                    px = index % width
                    py = index // width
                    area += 1
                    for nx, ny in ((px - 1, py), (px + 1, py),
                                   (px, py - 1), (px, py + 1)):
                        if nx <= x0 or nx >= x1 or ny <= y0 or ny >= y1:
                            continue
                        neighbor = ny * width + nx
                        if mask[neighbor] or visited[neighbor]:
                            continue
                        if queue_count >= len(queue):
                            return -1
                        visited[neighbor] = 1
                        queue[queue_count] = neighbor
                        queue_count += 1
                if area >= self.hole_min_area:
                    return 1
        return 0

    def _internal_component_runs_host(self, floor, band_end=None):
        """Host model of the Viper scanline flood fill for equivalence tests."""
        mask = self._current_mask
        visited = self._visited
        queue = self._queue
        width = self.width
        height = self.height
        if band_end is None:
            band_end = height
        total = width * height
        for index in range(total):
            visited[index] = 0
        for start_y in range(floor, band_end):
            for start_x in range(width):
                start = start_y * width + start_x
                if not mask[start] or visited[start]:
                    continue
                row_offset = start_y * width
                run_left = start_x
                while run_left > 0 and mask[row_offset + run_left - 1]:
                    run_left -= 1
                run_right = start_x
                while run_right + 1 < width and \
                        mask[row_offset + run_right + 1]:
                    run_right += 1
                for x in range(run_left, run_right + 1):
                    visited[row_offset + x] = 1
                queue_count = 1
                queue[0] = row_offset + run_left
                head = 0
                area = 0
                touches_roi = False
                min_x = width
                max_x = -1
                min_y = height
                max_y = -1
                while head < queue_count:
                    index = queue[head]
                    head += 1
                    y, run_left = divmod(index, width)
                    row_offset = y * width
                    run_right = run_left
                    while run_right + 1 < width and \
                            mask[row_offset + run_right + 1]:
                        run_right += 1
                    area += run_right - run_left + 1
                    min_x = min(min_x, run_left)
                    max_x = max(max_x, run_right)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                    if (run_left == 0 or run_right == width - 1 or
                            y == floor or y == band_end - 1):
                        touches_roi = True
                    for x in range(run_left, run_right + 1):
                        visited[row_offset + x] = 2

                    for ny in (y - 1, y + 1):
                        if ny < floor or ny >= band_end:
                            continue
                        neighbor_row = ny * width
                        nx = max(0, run_left - 1)
                        nx_limit = min(width - 1, run_right + 1)
                        while nx <= nx_limit:
                            neighbor = neighbor_row + nx
                            if mask[neighbor] and not visited[neighbor]:
                                next_left = nx
                                while (next_left > 0 and
                                       mask[neighbor_row + next_left - 1]):
                                    next_left -= 1
                                next_right = nx
                                while (next_right + 1 < width and
                                       mask[neighbor_row + next_right + 1]):
                                    next_right += 1
                                if queue_count >= total:
                                    return -1
                                queue[queue_count] = neighbor_row + next_left
                                queue_count += 1
                                for x in range(next_left, next_right + 1):
                                    visited[neighbor_row + x] = 1
                                nx = next_right + 1
                            else:
                                nx += 1

                box_width = max_x - min_x + 1
                box_height = max_y - min_y + 1
                if (touches_roi or area < self.component_min_area or
                        area > self.component_max_area or
                        box_width < self.component_min_extent or
                        box_height < self.component_min_extent or
                        box_width > self.component_max_extent or
                        box_height > self.component_max_extent):
                    continue
                hole = self._component_encloses_hole_host(
                    (min_x, min_y, max_x, max_y), floor, band_end)
                if hole < 0:
                    return -1
                if hole:
                    return 1
        return 0

    def _internal_component_host(self, floor, band_end=None):
        """Reference pixel BFS retained for host equivalence checks."""
        mask = self._current_mask
        visited = self._visited
        queue = self._queue
        width = self.width
        height = self.height
        if band_end is None:
            band_end = height
        for index in range(width * height):
            visited[index] = 0
        for start_y in range(floor, band_end):
            for start_x in range(width):
                start = start_y * width + start_x
                if not mask[start] or visited[start]:
                    continue
                visited[start] = 1
                queue_count = 1
                queue[0] = start
                head = 0
                area = 0
                touches_roi = False
                min_x = width
                max_x = -1
                min_y = height
                max_y = -1
                while head < queue_count:
                    index = queue[head]
                    head += 1
                    x = index % width
                    y = index // width
                    area += 1
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                    if (x == 0 or x == width - 1 or
                            y == floor or y == band_end - 1):
                        touches_roi = True
                    for dy in (-1, 0, 1):
                        ny = y + dy
                        if ny < floor or ny >= band_end:
                            continue
                        for dx in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            nx = x + dx
                            if nx < 0 or nx >= width:
                                continue
                            neighbor = ny * width + nx
                            if not mask[neighbor] or visited[neighbor]:
                                continue
                            if queue_count >= len(queue):
                                return -1
                            visited[neighbor] = 1
                            queue[queue_count] = neighbor
                            queue_count += 1

                box_width = max_x - min_x + 1
                box_height = max_y - min_y + 1
                if (touches_roi or area < self.component_min_area or
                        area > self.component_max_area or
                        box_width < self.component_min_extent or
                        box_height < self.component_min_extent or
                        box_width > self.component_max_extent or
                        box_height > self.component_max_extent):
                    continue
                hole = self._component_encloses_hole_host(
                    (min_x, min_y, max_x, max_y), floor, band_end)
                if hole < 0:
                    return -1
                if hole:
                    return 1
        return 0

    def _paired_boundary_row_host(self, y):
        mask = self._current_mask
        width = self.width
        row = y * width
        center_twice = width - 1
        x = 0
        max_left_end = -1
        while x < width:
            if not mask[row + x]:
                x += 1
                continue
            run_start = x
            while x < width and mask[row + x]:
                x += 1
            run_end = x - 1
            if run_end - run_start + 1 > self.open_max_run_width:
                continue
            if run_start * 2 < center_twice and run_end > max_left_end:
                max_left_end = run_end
            if (run_end * 2 > center_twice and max_left_end >= 0 and
                    run_start - max_left_end - 1 >= self.open_min_pair_gap):
                return True
        return False

    def _paired_support_host(self, start_y, end_y):
        for y in range(start_y, end_y + 1):
            if self._paired_boundary_row_host(y):
                return True
        return False

    def _wide_band_host(self, start_y, end_y, backwards):
        rows = self._current_rows
        band_end = self._current_band_end
        step = -1 if backwards else 1
        streak = 0
        wide_limit = self.width * 3 // 4
        y = start_y
        while (y >= end_y if backwards else y <= end_y):
            if rows[y] >= wide_limit:
                streak += 1
                if streak >= self.open_wide_rows:
                    edge_y = (self._current_floor if backwards else
                              band_end - 1)
                    probe_y = y
                    edge_connected = True
                    while probe_y != edge_y:
                        probe_y += step
                        if rows[probe_y] < wide_limit:
                            edge_connected = False
                            break
                    return not edge_connected
            else:
                streak = 0
            y += step
        return False

    def _open_band_host(self, floor, band_end):
        rows = self._current_rows
        y = floor
        while y < band_end:
            if rows[y] != 0:
                y += 1
                continue
            start = y
            while y < band_end and rows[y] == 0:
                y += 1
            end = y - 1
            if end - start + 1 < self.open_min_rows:
                continue
            if start <= floor or end >= band_end - 1:
                continue
            before = max(floor, start - self.open_support_rows)
            after = min(band_end - 1, end + self.open_support_rows)
            paired_before = self._paired_support_host(before, start - 1)
            paired_after = self._paired_support_host(end + 1, after)
            wide_before = self._wide_band_host(start - 1, before, True)
            wide_after = self._wide_band_host(end + 1, after, False)
            if ((wide_before and paired_after) or
                    (paired_before and wide_after)):
                return True
        return False

    def _set_result(self, island, branch, reason, complete=False,
                    ambiguous=None):
        self._result["island_evidence"] = bool(island)
        self._result["branch_evidence"] = bool(branch)
        self._result["ambiguous"] = (bool(island or branch)
                                      if ambiguous is None else
                                      bool(ambiguous))
        self._result["reason"] = reason
        self._result["complete"] = bool(complete)
        self._result["trusted_segment_count"] = int(
            self._segment_count_buffer[0])
        self._result["fully_known_rows"] = int(self._trusted_row_count)
        return self._result

    def check(self, mask_info):
        """Return reused topology evidence, isolating unknown rows as gaps."""
        self._segment_count_buffer[0] = 0
        self._trusted_row_count = 0
        if not isinstance(mask_info, dict):
            return self._set_result(False, False, "INVALID_MASK_INFO")
        width = mask_info.get("width")
        height = mask_info.get("height")
        floor = mask_info.get("floor")
        if (isinstance(width, bool) or width != self.width or
                isinstance(height, bool) or height != self.height or
                isinstance(floor, bool) or not isinstance(floor, int) or
                floor < 0 or floor >= height):
            return self._set_result(False, False, "INVALID_MASK_INFO")
        edge_mask = mask_info.get("edge_mask")
        known_region = mask_info.get("known_region")
        row_counts = mask_info.get("row_counts")
        total = self.width * self.height
        if (not isinstance(edge_mask, bytearray) or len(edge_mask) != total or
                not isinstance(known_region, bytearray) or
                len(known_region) != total or
                not isinstance(row_counts, bytearray) or
                len(row_counts) != self.height):
            return self._set_result(False, False, "INVALID_MASK_INFO")

        self._current_mask = edge_mask
        self._current_known = known_region
        self._current_rows = row_counts
        self._current_floor = floor
        use_native_accel = (
            _NATIVE_ACCEL is not None and
            self.width == 80 and self.height == 60)
        complete = _region_segments_dispatch(self, floor)
        if _IS_MICROPYTHON or use_native_accel:
            self._trusted_row_count = self._segment_count_buffer[1]
        if complete == -2:
            self._segment_count_buffer[0] = 0
            return self._set_result(False, False, "UNKNOWN_REGION",
                                    ambiguous=True)
        if complete < 0:
            return self._set_result(False, False, "INVALID_MASK_INFO")
        segment_count = self._segment_count_buffer[0]
        if segment_count <= 0:
            return self._set_result(False, False, "UNKNOWN_REGION",
                                    ambiguous=True)

        if not _IS_MICROPYTHON and not use_native_accel:
            for y in range(floor, self.height):
                if self._trusted_rows[y]:
                    self._trusted_row_count += 1

        island = 0
        branch = 0
        for segment in range(segment_count):
            segment_floor = self._segment_starts[segment]
            segment_end = self._segment_ends[segment]
            if segment_floor < floor or segment_end > self.height or \
                    segment_floor >= segment_end:
                return self._set_result(False, False, "KERNEL_BOUNDS")
            self._current_floor = segment_floor
            self._current_band_end = segment_end
            segment_island = _internal_component_dispatch(
                self, segment_floor, segment_end)
            if segment_island < 0:
                return self._set_result(False, False, "KERNEL_BOUNDS")
            if segment_island:
                island = 1
                break
        if not island:
            for segment in range(segment_count):
                segment_floor = self._segment_starts[segment]
                segment_end = self._segment_ends[segment]
                self._current_floor = segment_floor
                self._current_band_end = segment_end
                if _IS_MICROPYTHON:
                    segment_branch = _open_band_viper(
                        self, segment_floor, segment_end)
                else:
                    segment_branch = self._open_band_host(
                        segment_floor, segment_end)
                if segment_branch < 0:
                    return self._set_result(False, False, "KERNEL_BOUNDS")
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
        return self._set_result(island, branch, reason, complete == 1)
