"""PRIVATE EXPERIMENTAL FIXTURE; not the production OpenMV module.

This owned fixture is used only for host differential tests. The corresponding
RAM-build input remains under private logs/ and requires board timing before
any production adoption.
"""

"""Reusable 320x120 grayscale to 80x60 road and edge masks.

The module owns all 80x60 byte buffers and its three 256-bin histograms.
``build`` reuses them on every call, so returned buffers and diagnostics are
valid for the most recently processed frame only. OpenMV builds use Viper and
guarded Thumb assembly for min-pooling and histogram work, plus Viper mask
writes. CPython uses explicit reference loops and reports that it did not
compile the Viper kernels.
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
    # Names used in Viper signatures are builtins on MicroPython.  The host
    # shim needs placeholders only so CPython can import these definitions.
    ptr8 = int
    ptr32 = int

try:
    import image as _image_module
except ImportError:
    _image_module = None

try:
    import uctypes as _uctypes
except ImportError:
    _uctypes = None

try:
    import ustruct as _struct
except ImportError:
    import struct as _struct


def _has_obj_repr_c_float_decode():
    """Probe both erased low bits and their restored OBJ_REPR_C payload."""
    if not _IS_MICROPYTHON:
        return False
    try:
        first_value = _struct.unpack(
            "<f", _struct.pack("<I", 0x3F800001))[0]
        first_bits = _struct.unpack(
            "<I", _struct.pack("<f", first_value))[0]
        second_value = _struct.unpack(
            "<f", _struct.pack("<I", 0x42B094DC))[0]
        second_bits = _struct.unpack(
            "<I", _struct.pack("<f", second_value))[0]
    except Exception:
        return False
    # The first case alone could also match simple truncation. The second
    # confirms OBJ_REPR_C restores bits 2..3 into the erased payload bits.
    return first_bits == 0x3F800000 and second_bits == 0x42B094DF


_OTSU_ASM_SUPPORTED = (
    _IS_MICROPYTHON and _uctypes is not None and
    _has_obj_repr_c_float_decode())

_MIN_POOL_ASM_SUPPORTED = _IS_MICROPYTHON and _uctypes is not None
_HISTOGRAM_ASM_SUPPORTED = _IS_MICROPYTHON and _uctypes is not None


if _HISTOGRAM_ASM_SUPPORTED:
    @micropython.asm_thumb
    def _histogram_block_flat4_asm(r0, r1, r2):
        # r0=contiguous luma bytes, r1=uint32 histogram, r2=groups of four.
        label(loop)
        cmp(r2, 0)
        beq(done)

        ldrb(r3, [r0, 0])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 1])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 2])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        ldrb(r3, [r0, 3])
        lsl(r3, r3, 2)
        add(r4, r1, r3)
        ldr(r3, [r4, 0])
        add(r3, r3, 1)
        str(r3, [r4, 0])

        add(r0, r0, 4)
        sub(r2, r2, 1)
        b(loop)

        label(done)
        mov(r0, 0)
else:
    _histogram_block_flat4_asm = None


def _histogram_asm_geometry_supported(rectangles, width, block_count):
    """Return true only for full-width, byte-contiguous flat row spans."""
    for block in range(block_count):
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        pixel_count = (x1 - x0) * (y1 - y0)
        if x0 != 0 or x1 != width or (pixel_count & 3):
            return False
    return True


if _MIN_POOL_ASM_SUPPORTED:
    @micropython.asm_thumb
    def _min_pool_asm(r0, r1):
        # Fixed 320x120 source crop -> 80x60 output, 4x2 exact minimum.
        # r0=source byte pointer; r1=output byte pointer. The framebuffer
        # pointer is word-aligned before entering this kernel. Grayscale
        # bytes are 0..255, so signed BGE has the same order as unsigned.
        mov(r2, r0)
        mov(r0, 80)
        lsl(r0, r0, 2)
        add(r3, r2, r0)
        mov(r4, 60)

        label(row_loop)
        mov(r5, 80)

        label(column_loop)
        ldrb(r6, [r2, 0])
        ldrb(r7, [r2, 1])
        cmp(r7, r6)
        bge(top1_done)
        mov(r6, r7)
        label(top1_done)
        ldrb(r7, [r2, 2])
        cmp(r7, r6)
        bge(top2_done)
        mov(r6, r7)
        label(top2_done)
        ldrb(r7, [r2, 3])
        cmp(r7, r6)
        bge(top3_done)
        mov(r6, r7)
        label(top3_done)
        ldrb(r7, [r3, 0])
        cmp(r7, r6)
        bge(bottom0_done)
        mov(r6, r7)
        label(bottom0_done)
        ldrb(r7, [r3, 1])
        cmp(r7, r6)
        bge(bottom1_done)
        mov(r6, r7)
        label(bottom1_done)
        ldrb(r7, [r3, 2])
        cmp(r7, r6)
        bge(bottom2_done)
        mov(r6, r7)
        label(bottom2_done)
        ldrb(r7, [r3, 3])
        cmp(r7, r6)
        bge(bottom3_done)
        mov(r6, r7)
        label(bottom3_done)
        strb(r6, [r1, 0])

        add(r2, r2, 4)
        add(r3, r3, 4)
        add(r1, r1, 1)
        sub(r5, r5, 1)
        cmp(r5, 0)
        bne(column_loop)

        # Skip the next source row after consuming a 2-row output cell.
        mov(r0, 80)
        lsl(r0, r0, 2)
        add(r2, r2, r0)
        add(r3, r3, r0)
        sub(r4, r4, 1)
        cmp(r4, 0)
        bne(row_loop)
        mov(r0, 0)
else:
    _min_pool_asm = None


@micropython.viper
def _min_pool_words_viper(self, source_object, source_width: int) -> int:
    """Min-pool two aligned little-endian words per 4x2 output cell."""
    source = ptr32(source_object)
    target = ptr8(self.luma)
    output_width = int(self.width)
    output_height = int(self.height)
    scale_y = int(self.scale_y)
    source_word_width = int(source_width) >> 2
    for y in range(output_height):
        source_top = y * scale_y * source_word_width
        source_bottom = source_top + source_word_width
        output_y = y * output_width
        for x in range(output_width):
            top_word = source[source_top + x]
            bottom_word = source[source_bottom + x]
            minimum = top_word & 0xFF
            value = (top_word >> 8) & 0xFF
            if value < minimum:
                minimum = value
            value = (top_word >> 16) & 0xFF
            if value < minimum:
                minimum = value
            value = (top_word >> 24) & 0xFF
            if value < minimum:
                minimum = value
            value = bottom_word & 0xFF
            if value < minimum:
                minimum = value
            value = (bottom_word >> 8) & 0xFF
            if value < minimum:
                minimum = value
            value = (bottom_word >> 16) & 0xFF
            if value < minimum:
                minimum = value
            value = (bottom_word >> 24) & 0xFF
            if value < minimum:
                minimum = value
            target[output_y + x] = minimum
    return 0


@micropython.viper
def _min_pool_viper(self, source_object, source_width: int) -> int:
    """Min-pool every pixel in each 4x2 source cell into the grid."""
    source = ptr8(source_object)
    target = ptr8(self.luma)
    output_width = int(self.width)
    output_height = int(self.height)
    scale_x = int(self.scale_x)
    scale_y = int(self.scale_y)
    for y in range(output_height):
        source_top = y * scale_y * source_width
        source_bottom = source_top + source_width
        output_y = y * output_width
        source_x = 0
        for x in range(output_width):
            top = source_top + source_x
            bottom = source_bottom + source_x
            minimum = source[top]
            value = source[top + 1]
            if value < minimum:
                minimum = value
            value = source[top + 2]
            if value < minimum:
                minimum = value
            value = source[top + 3]
            if value < minimum:
                minimum = value
            value = source[bottom]
            if value < minimum:
                minimum = value
            value = source[bottom + 1]
            if value < minimum:
                minimum = value
            value = source[bottom + 2]
            if value < minimum:
                minimum = value
            value = source[bottom + 3]
            if value < minimum:
                minimum = value
            target[output_y + x] = minimum
            source_x += scale_x
    return 0


@micropython.viper
def _histogram_viper(self) -> int:
    """Refresh histograms and compact per-block summaries in place."""
    luma = ptr8(self.luma)
    histograms = ptr32(self._histograms)
    rectangles = ptr32(self._rectangles)
    nonzero_values = ptr8(self._nonzero_values)
    nonzero_counts = ptr32(self._nonzero_counts)
    block_stats = ptr32(self._block_stats)
    width = int(self.width)
    block_count = int(self._block_count)
    for index in range(block_count * 256):
        histograms[index] = 0
    for block in range(block_count):
        hist_base = block * 256
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        for y in range(y0, y1):
            row = y * width
            for x in range(x0, x1):
                value = luma[row + x]
                histograms[hist_base + value] += 1

        # Compact the ascending non-empty bins while accumulating the block
        # count, weighted sum, and value range.  This moves the former
        # 3x256-element Python scan into the already-compiled Viper kernel.
        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = int(histograms[hist_base + value])
            if amount != 0:
                nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        block_stats[stats_base] = count
        block_stats[stats_base + 1] = value_sum
        block_stats[stats_base + 2] = minimum
        block_stats[stats_base + 3] = maximum
    return 0


@micropython.viper
def _histogram_clear_viper(self) -> int:
    """Clear bounded per-block histogram bins before the Thumb count pass."""
    histograms = ptr32(self._histograms)
    total = int(self._block_count) * 256
    for index in range(total):
        histograms[index] = 0
    return 0


@micropython.viper
def _histogram_compact_viper(self) -> int:
    """Compact ascending nonzero bins and collect block statistics."""
    histograms = ptr32(self._histograms)
    nonzero_values = ptr8(self._nonzero_values)
    nonzero_counts = ptr32(self._nonzero_counts)
    block_stats = ptr32(self._block_stats)
    block_count = int(self._block_count)
    for block in range(block_count):
        hist_base = block * 256
        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = int(histograms[hist_base + value])
            if amount != 0:
                nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        block_stats[stats_base] = count
        block_stats[stats_base + 1] = value_sum
        block_stats[stats_base + 2] = minimum
        block_stats[stats_base + 3] = maximum
    return 0


@micropython.native
def _histogram_refresh(self):
    """Use flat-block Thumb counting when its instance geometry is proven."""
    if not self._histogram_asm_ready:
        return _histogram_viper(self)

    _histogram_clear_viper(self)
    width = self.width
    rectangles = self._rectangles
    luma_ptr = self._histogram_luma_ptr
    histograms_ptr = self._histogram_histograms_ptr
    for block in range(self._block_count):
        rect_base = block * 4
        y0 = rectangles[rect_base + 1]
        y1 = rectangles[rect_base + 3]
        source_ptr = luma_ptr + y0 * width
        hist_ptr = histograms_ptr + block * 256 * 4
        pixel_count = (y1 - y0) * width
        _histogram_block_flat4_asm(
            source_ptr, hist_ptr, pixel_count >> 2)
    _histogram_compact_viper(self)
    return 0


@micropython.viper
def _mask_viper(self) -> int:
    """Clear reusable outputs, then write only trusted configured blocks."""
    luma = ptr8(self.luma)
    road_mask = ptr8(self.road_mask)
    edge_mask = ptr8(self.edge_mask)
    known_region = ptr8(self.known_region)
    rectangles = ptr32(self._rectangles)
    thresholds = ptr32(self._threshold_values)
    trusted = ptr32(self._trusted_values)
    row_counts = ptr8(self.row_counts)
    road_row_counts = ptr8(self.road_row_counts)
    pixel_totals = ptr32(self._pixel_totals)
    width = int(self.width)
    height = int(self.height)
    block_count = int(self._block_count)
    total = width * height
    for index in range(total):
        road_mask[index] = 0
        edge_mask[index] = 0
        known_region[index] = 0
    pixel_totals[0] = 0
    pixel_totals[1] = 0
    for y in range(height):
        row_counts[y] = 0
        road_row_counts[y] = 0

    for block in range(block_count):
        if trusted[block] == 0:
            continue
        threshold = thresholds[block]
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        for y in range(y0, y1):
            row = y * width
            for x in range(x0, x1):
                index = row + x
                known_region[index] = 1
                if luma[index] > threshold:
                    road_mask[index] = 255
                    road_row_counts[y] += 1
                    pixel_totals[0] += 1
                else:
                    edge_mask[index] = 1
                    row_counts[y] += 1
                    pixel_totals[1] += 1
    return 0


@micropython.viper
def _mask_viper_full_width_tiled(self) -> int:
    """Classify four pixels at a time into complete output mask words.

    This fast path is valid only when configured blocks fully tile the ROI
    across the whole image width and every block is trusted for this frame.
    It overwrites every output word, including the unknown rows above the ROI,
    so it does not depend on a preceding plane clear.
    """
    if int(self._mask_full_width_tiled_ready) == 0:
        return -1

    luma = ptr8(self.luma)
    road_words = ptr32(self.road_mask)
    edge_words = ptr32(self.edge_mask)
    known_words = ptr32(self.known_region)
    rectangles = ptr32(self._rectangles)
    thresholds = ptr32(self._threshold_values)
    trusted = ptr32(self._trusted_values)
    row_counts = ptr8(self.row_counts)
    road_row_counts = ptr8(self.road_row_counts)
    totals = ptr32(self._pixel_totals)
    width = int(self.width)
    height = int(self.height)
    floor = int(self.floor)
    block_count = int(self._block_count)

    # The check must complete before any output is changed. Partial trust
    # falls through to the general kernel, which clears unknown regions.
    for block in range(block_count):
        if trusted[block] == 0:
            return -1

    words_per_row = width >> 2
    for y in range(floor):
        row_word = y * words_per_row
        for group in range(words_per_row):
            index = row_word + group
            road_words[index] = 0
            edge_words[index] = 0
            known_words[index] = 0
        row_counts[y] = 0
        road_row_counts[y] = 0

    road_total = 0
    edge_total = 0
    for block in range(block_count):
        rect_base = block * 4
        y0 = rectangles[rect_base + 1]
        y1 = rectangles[rect_base + 3]
        threshold = thresholds[block]
        for y in range(y0, y1):
            row = y * width
            row_word = y * words_per_row
            row_road_count = 0
            row_edge_count = 0
            for group in range(words_per_row):
                index = row + (group << 2)
                lane0 = luma[index]
                lane1 = luma[index + 1]
                lane2 = luma[index + 2]
                lane3 = luma[index + 3]
                road_word = 0
                edge_word = 0
                if lane0 > threshold:
                    road_word = road_word | 255
                    row_road_count += 1
                else:
                    edge_word = edge_word | 1
                    row_edge_count += 1
                if lane1 > threshold:
                    road_word = road_word | 65280
                    row_road_count += 1
                else:
                    edge_word = edge_word | 256
                    row_edge_count += 1
                if lane2 > threshold:
                    road_word = road_word | 16711680
                    row_road_count += 1
                else:
                    edge_word = edge_word | 65536
                    row_edge_count += 1
                if lane3 > threshold:
                    road_word = road_word | -16777216
                    row_road_count += 1
                else:
                    edge_word = edge_word | 16777216
                    row_edge_count += 1
                output_word = row_word + group
                road_words[output_word] = road_word
                edge_words[output_word] = edge_word
                known_words[output_word] = 16843009
            row_counts[y] = row_edge_count
            road_row_counts[y] = row_road_count
            road_total += row_road_count
            edge_total += row_edge_count

    totals[0] = road_total
    totals[1] = edge_total
    return 0


@micropython.viper
def _mask_viper_local_counts(self) -> int:
    """Clear aligned planes by word and update row/totals once per row."""
    luma = ptr8(self.luma)
    road = ptr8(self.road_mask)
    edge = ptr8(self.edge_mask)
    known = ptr8(self.known_region)
    road_words = ptr32(self.road_mask)
    edge_words = ptr32(self.edge_mask)
    known_words = ptr32(self.known_region)
    rectangles = ptr32(self._rectangles)
    thresholds = ptr32(self._threshold_values)
    trusted = ptr32(self._trusted_values)
    row_counts = ptr8(self.row_counts)
    road_row_counts = ptr8(self.road_row_counts)
    totals = ptr32(self._pixel_totals)
    width = int(self.width)
    height = int(self.height)
    block_count = int(self._block_count)
    total = width * height

    for word_index in range(total >> 2):
        road_words[word_index] = 0
        edge_words[word_index] = 0
        known_words[word_index] = 0
    for y in range(height):
        row_counts[y] = 0
        road_row_counts[y] = 0
    totals[0] = 0
    totals[1] = 0

    road_total = 0
    edge_total = 0
    for block in range(block_count):
        if trusted[block] == 0:
            continue
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        threshold = thresholds[block]
        for y in range(y0, y1):
            row = y * width
            row_road_count = 0
            row_edge_count = 0
            for x in range(x0, x1):
                index = row + x
                known[index] = 1
                if luma[index] > threshold:
                    road[index] = 255
                    row_road_count += 1
                else:
                    edge[index] = 1
                    row_edge_count += 1
            row_counts[y] += row_edge_count
            road_row_counts[y] += row_road_count
            road_total += row_road_count
            edge_total += row_edge_count
    totals[0] = road_total
    totals[1] = edge_total
    return 0


@micropython.native
def _otsu_native(histograms, nonzero_values, nonzero_base: int,
                 nonzero_count: int, count: int, value_sum: int):
    """Sweep only populated bins, preserving the old earliest-cutoff tie."""
    low_count = 0
    low_sum = 0
    best_score = -1.0
    best_threshold = 0
    best_low_count = 0
    best_high_count = 0
    best_low_mean = 0.0
    best_high_mean = 0.0
    scale = float(count * count)
    for item in range(nonzero_count):
        value = nonzero_values[nonzero_base + item]
        amount = histograms[nonzero_base + value]
        low_count += amount
        low_sum += value * amount
        high_count = count - low_count
        if low_count == 0 or high_count == 0:
            continue
        high_sum = value_sum - low_sum
        low_mean = low_sum / float(low_count)
        high_mean = high_sum / float(high_count)
        difference = high_mean - low_mean
        score = ((low_count * high_count / scale) * difference) * difference
        if score > best_score:
            best_score = score
            best_threshold = value
            best_low_count = low_count
            best_high_count = high_count
            best_low_mean = low_mean
            best_high_mean = high_mean
    contrast = best_high_mean - best_low_mean
    return (best_threshold, best_low_count, best_high_count,
            best_low_mean, best_high_mean, contrast)


if _OTSU_ASM_SUPPORTED:
    @micropython.asm_thumb
    def _otsu_threshold_asm(r0, r1, r2, r3):
        # r0=histogram base, r1=nonzero-values base,
        # r2=[count,sum,nz_count,hist_base,nonzero_base], r3=workspace.
        # The wrapper saves/restores r1-r7 and lr around this body.
        ldr(r4, [r2, 0])
        ldr(r5, [r2, 4])
        ldr(r6, [r2, 8])
        str(r0, [r3, 24])
        str(r1, [r3, 28])
        ldr(r7, [r2, 12])
        lsl(r7, r7, 2)
        str(r7, [r3, 60])
        ldr(r7, [r2, 16])
        str(r7, [r3, 64])
        mov(r0, 0)
        str(r0, [r3, 4])
        str(r0, [r3, 8])
        str(r0, [r3, 12])
        mov(r7, 0)

        label(loop)
        cmp(r7, r6)
        blt(process)
        b(done)

        label(process)
        # Accumulate the next populated value into the low class.
        ldr(r0, [r3, 28])
        ldr(r2, [r3, 64])
        add(r0, r0, r2)
        add(r0, r0, r7)
        ldrb(r1, [r0, 0])
        str(r1, [r3, 16])
        lsl(r2, r1, 2)
        ldr(r0, [r3, 24])
        ldr(r1, [r3, 60])
        add(r0, r0, r1)
        add(r0, r0, r2)
        ldr(r1, [r0, 0])
        str(r1, [r3, 20])
        ldr(r0, [r3, 4])
        add(r0, r0, r1)
        str(r0, [r3, 4])
        ldr(r2, [r3, 8])
        ldr(r0, [r3, 16])
        ldr(r1, [r3, 20])
        mul(r0, r1)
        add(r2, r2, r0)
        str(r2, [r3, 8])

        # Empty high class is skipped exactly as in the native reference.
        ldr(r0, [r3, 4])
        sub(r1, r4, r0)
        cmp(r1, 0)
        bne(nonempty_high)
        b(skip)

        label(nonempty_high)
        str(r1, [r3, 36])
        ldr(r0, [r3, 8])
        sub(r0, r5, r0)
        str(r0, [r3, 32])

        # Match OBJ_REPR_C after every boxed float conversion or operation:
        # bits = (bits & ~3) | ((bits >> 2) & 3).
        ldr(r0, [r3, 8])
        vmov(s0, r0)
        vcvt_f32_s32(s0, s0)
        ldr(r0, [r3, 4])
        vmov(s1, r0)
        vcvt_f32_s32(s1, s1)
        mov(r0, 3)
        vmov(r1, s1)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s1, r1)
        vdiv(s0, s0, s1)
        mov(r0, 3)
        vmov(r1, s0)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s0, r1)

        # high_mean = high_sum / float(high_count)
        ldr(r0, [r3, 32])
        vmov(s1, r0)
        vcvt_f32_s32(s1, s1)
        ldr(r0, [r3, 36])
        vmov(s2, r0)
        vcvt_f32_s32(s2, s2)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)
        vdiv(s1, s1, s2)
        mov(r0, 3)
        vmov(r1, s1)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s1, r1)
        vsub(s2, s1, s0)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)

        # Keep the original left-to-right score expression and tie rule.
        ldr(r0, [r3, 4])
        ldr(r1, [r3, 36])
        mul(r0, r1)
        vmov(s3, r0)
        vcvt_f32_s32(s3, s3)
        mov(r0, r4)
        mul(r0, r4)
        vmov(s4, r0)
        vcvt_f32_s32(s4, s4)
        mov(r0, 3)
        vmov(r1, s4)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s4, r1)
        vdiv(s3, s3, s4)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)
        vmul(s3, s3, s2)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)
        vmul(s3, s3, s2)
        mov(r0, 3)
        vmov(r1, s3)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s3, r1)

        vldr(s4, [r3, 0])
        vcmp(s3, s4)
        vmrs(APSR_nzcv, FPSCR)
        ble(skip)
        vstr(s3, [r3, 0])
        ldr(r0, [r3, 16])
        str(r0, [r3, 12])
        ldr(r0, [r3, 4])
        str(r0, [r3, 40])
        ldr(r0, [r3, 36])
        str(r0, [r3, 44])
        vstr(s0, [r3, 48])
        vstr(s1, [r3, 52])
        vstr(s2, [r3, 56])

        label(skip)
        add(r7, r7, 1)
        b(loop)

        label(done)
        # Contrast is computed after the winning means are retained.
        vldr(s0, [r3, 48])
        vldr(s1, [r3, 52])
        vsub(s2, s1, s0)
        mov(r0, 3)
        vmov(r1, s2)
        mov(r2, r1)
        lsr(r2, r2, 2)
        and_(r2, r0)
        bic(r1, r0)
        orr(r1, r2)
        vmov(s2, r1)
        vstr(s2, [r3, 56])
        ldr(r0, [r3, 12])
else:
    _otsu_threshold_asm = None


@micropython.native
def _otsu_asm_result(self, hist_base: int, nonzero_base: int,
                     nonzero_count: int, count: int, value_sum: int):
    meta = self._otsu_asm_meta
    meta[0] = count
    meta[1] = value_sum
    meta[2] = nonzero_count
    meta[3] = hist_base
    meta[4] = nonzero_base
    workspace = self._otsu_asm_workspace
    workspace[0] = 0xBF800000
    workspace[3] = 0
    workspace[10] = 0
    workspace[11] = 0
    workspace[12] = 0
    workspace[13] = 0
    workspace[14] = 0
    histograms_ptr = self._otsu_histograms_ptr
    nonzero_values_ptr = self._otsu_nonzero_values_ptr
    meta_ptr = self._otsu_asm_meta_ptr
    workspace_ptr = self._otsu_asm_workspace_ptr
    _otsu_threshold_asm(histograms_ptr, nonzero_values_ptr,
                        meta_ptr, workspace_ptr)
    low_mean = _struct.unpack_from("<f", workspace, 48)[0]
    high_mean = _struct.unpack_from("<f", workspace, 52)[0]
    contrast = _struct.unpack_from("<f", workspace, 56)[0]
    return (workspace[3], workspace[10], workspace[11],
            low_mean, high_mean, contrast)


def _min_pool_host(source, target, source_width, output_width,
                   output_height, scale_x, scale_y):
    for y in range(output_height):
        source_top = y * scale_y * source_width
        source_bottom = source_top + source_width
        output_y = y * output_width
        for x in range(output_width):
            source_x = x * scale_x
            top = source_top + source_x
            bottom = source_bottom + source_x
            minimum = source[top]
            value = source[top + 1]
            if value < minimum:
                minimum = value
            value = source[top + 2]
            if value < minimum:
                minimum = value
            value = source[top + 3]
            if value < minimum:
                minimum = value
            value = source[bottom]
            if value < minimum:
                minimum = value
            value = source[bottom + 1]
            if value < minimum:
                minimum = value
            value = source[bottom + 2]
            if value < minimum:
                minimum = value
            value = source[bottom + 3]
            if value < minimum:
                minimum = value
            target[output_y + x] = minimum


def _histogram_host(luma, histograms, rectangles, width, block_count,
                    nonzero_values, nonzero_counts, block_stats):
    for index in range(block_count * 256):
        histograms[index] = 0
    for block in range(block_count):
        hist_base = block * 256
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        for y in range(y0, y1):
            row = y * width
            for x in range(x0, x1):
                value = luma[row + x]
                histograms[hist_base + value] += 1

        nonzero_count = 0
        count = 0
        value_sum = 0
        minimum = 256
        maximum = 0
        for value in range(256):
            amount = histograms[hist_base + value]
            if amount:
                nonzero_values[hist_base + nonzero_count] = value
                nonzero_count += 1
                count += amount
                value_sum += value * amount
                if minimum == 256:
                    minimum = value
                maximum = value
        nonzero_counts[block] = nonzero_count
        stats_base = block * 4
        block_stats[stats_base] = count
        block_stats[stats_base + 1] = value_sum
        block_stats[stats_base + 2] = minimum
        block_stats[stats_base + 3] = maximum


def _mask_host(luma, road_mask, edge_mask, known_region, rectangles,
               thresholds, trusted, row_counts, road_row_counts,
               pixel_totals, width, height, block_count):
    total = width * height
    for index in range(total):
        road_mask[index] = 0
        edge_mask[index] = 0
        known_region[index] = 0
    pixel_totals[0] = 0
    pixel_totals[1] = 0
    for y in range(height):
        row_counts[y] = 0
        road_row_counts[y] = 0

    for block in range(block_count):
        if not trusted[block]:
            continue
        threshold = thresholds[block]
        rect_base = block * 4
        x0 = rectangles[rect_base]
        y0 = rectangles[rect_base + 1]
        x1 = rectangles[rect_base + 2]
        y1 = rectangles[rect_base + 3]
        for y in range(y0, y1):
            row = y * width
            for x in range(x0, x1):
                index = row + x
                known_region[index] = 1
                if luma[index] > threshold:
                    road_mask[index] = 255
                    road_row_counts[y] += 1
                    pixel_totals[0] += 1
                else:
                    edge_mask[index] = 1
                    row_counts[y] += 1
                    pixel_totals[1] += 1


def _pixel_luma(pixel):
    if isinstance(pixel, (tuple, list)):
        if not pixel:
            raise ValueError("get_pixel returned an empty pixel")
        if len(pixel) < 3:
            return int(pixel[0])
        red, green, blue = int(pixel[0]), int(pixel[1]), int(pixel[2])
        return (77 * red + 150 * green + 29 * blue) >> 8
    value = int(pixel)
    if value < 0 or value > 255:
        raise ValueError("get_pixel values must be in 0..255")
    return value


class FastGrayMasks:
    """Build reusable Otsu masks from one fixed grayscale camera frame.

    ``settings`` is the existing ``openmv/config.py`` module or an object with
    its ``SURFACE_GRID_*`` and ``GRAY_THRESHOLD_*`` values.  Configured blocks
    must be non-overlapping and lie inside the 80x60 grid below ``floor``.
    Neighbor smoothing is unsupported; it is rejected if enabled so callers
    cannot silently get masks with different trust semantics.
    """

    def __init__(self, settings, debug=False):
        self.width = getattr(settings, "SURFACE_GRID_WIDTH", 80)
        self.height = getattr(settings, "SURFACE_GRID_HEIGHT", 60)
        self.floor = getattr(settings, "SURFACE_ROI_FLOOR_Y", 8)
        self.scale_x = getattr(settings, "SURFACE_PIXEL_SCALE_X", 4)
        self.scale_y = getattr(settings, "SURFACE_PIXEL_SCALE_Y", 2)
        if (self.width != 80 or self.height != 60 or
                self.scale_x != 4 or self.scale_y != 2):
            raise ValueError("fast_gray requires 80x60 grid with scale 4x2")
        if (isinstance(self.floor, bool) or not isinstance(self.floor, int) or
                self.floor < 0 or self.floor >= self.height):
            raise ValueError("SURFACE_ROI_FLOOR_Y must be an integer in 0..59")

        self.min_contrast = getattr(
            settings, "GRAY_THRESHOLD_MIN_CONTRAST", 18.0)
        if (isinstance(self.min_contrast, bool) or
                not isinstance(self.min_contrast, (int, float)) or
                self.min_contrast < 0 or
                self.min_contrast != self.min_contrast or
                self.min_contrast == float("inf")):
            raise ValueError("GRAY_THRESHOLD_MIN_CONTRAST must be finite and non-negative")
        self.min_contrast = float(self.min_contrast)
        if getattr(settings, "GRAY_THRESHOLD_SMOOTH_NEIGHBORS", False):
            raise ValueError("fast_gray requires GRAY_THRESHOLD_SMOOTH_NEIGHBORS=False")

        configured_blocks = getattr(settings, "GRAY_THRESHOLD_BLOCKS", None)
        if configured_blocks is None:
            split_a = self.floor + (self.height - self.floor) // 3
            split_b = self.floor + 2 * (self.height - self.floor) // 3
            configured_blocks = ((0, self.floor, self.width, split_a),
                                 (0, split_a, self.width, split_b),
                                 (0, split_b, self.width, self.height))
        if not isinstance(configured_blocks, (tuple, list)) or \
                len(configured_blocks) < 1 or len(configured_blocks) > 4:
            raise ValueError("GRAY_THRESHOLD_BLOCKS must contain 1..4 blocks")
        self.blocks = []
        for rect in configured_blocks:
            if not isinstance(rect, (tuple, list)) or len(rect) != 4:
                raise ValueError("each threshold block must be (x0, y0, x1, y1)")
            x0, y0, x1, y1 = rect
            if any(isinstance(value, bool) or not isinstance(value, int)
                   for value in rect):
                raise ValueError("threshold block coordinates must be integers")
            if (x0 < 0 or y0 < self.floor or x1 > self.width or
                    y1 > self.height or x0 > x1 or y0 > y1):
                raise ValueError("threshold block is outside the grayscale ROI")
            self.blocks.append((x0, y0, x1, y1))
        block_count = len(self.blocks)
        for first in range(block_count):
            ax0, ay0, ax1, ay1 = self.blocks[first]
            for second in range(first + 1, block_count):
                bx0, by0, bx1, by1 = self.blocks[second]
                if (min(ax1, bx1) > max(ax0, bx0) and
                        min(ay1, by1) > max(ay0, by0)):
                    raise ValueError("GRAY_THRESHOLD_BLOCKS must not overlap")

        # The isolated word-mask candidate can skip the full-plane clear only
        # when full-width blocks tile every row of the configured ROI. This is
        # a structural optimization guard; the frame-local trust check remains
        # in the kernel before it writes any output.
        self._mask_full_width_tiled_ready = False
        expected_y = self.floor
        full_width_tiled = True
        for rect in self.blocks:
            x0, y0, x1, y1 = rect
            if (x0 != 0 or x1 != self.width or y0 != expected_y or
                    y1 <= y0):
                full_width_tiled = False
                break
            expected_y = y1
        self._mask_full_width_tiled_ready = (
            full_width_tiled and expected_y == self.height)

        self.debug = bool(debug)
        self._block_count = len(self.blocks)
        self._rectangles = array("I")
        for rect in self.blocks:
            for value in rect:
                self._rectangles.append(value)
        # All frame-sized buffers and all bounded uint32 histograms are owned by
        # this instance and allocated once.  Threshold scratch is also stable.
        total = self.width * self.height
        self.luma = bytearray(total)
        self.road_mask = bytearray(total)
        self.edge_mask = bytearray(total)
        self.known_region = bytearray(total)
        self._mask_local_counts_ready = False
        if (_IS_MICROPYTHON and _uctypes is not None and
                (total & 3) == 0):
            try:
                mask_pointers = (
                    _uctypes.addressof(self.road_mask) |
                    _uctypes.addressof(self.edge_mask) |
                    _uctypes.addressof(self.known_region))
                self._mask_local_counts_ready = (mask_pointers & 3) == 0
            except Exception:
                self._mask_local_counts_ready = False
        self.row_counts = bytearray(self.height)
        self.road_row_counts = bytearray(self.height)
        self._histograms = array("I", [0] * (self._block_count * 256))
        self._nonzero_values = bytearray(self._block_count * 256)
        self._nonzero_counts = array("I", [0] * self._block_count)
        # Per-block [pixel count, weighted sum, min value, max value], filled
        # by the histogram kernel so thresholding does not rescan 768 bins in
        # Python every frame. Every allowed rectangle is <= 80*60=4800
        # pixels, and its weighted sum is <= 4800*255=1,224,000; both fit
        # array('I') and signed native locals in Viper.
        self._block_stats = array("I", [0] * (self._block_count * 4))
        self._histogram_asm_ready = False
        self._histogram_luma_ptr = None
        self._histogram_histograms_ptr = None
        if (_HISTOGRAM_ASM_SUPPORTED and
                _histogram_asm_geometry_supported(
                    self._rectangles, self.width, self._block_count)):
            try:
                luma_ptr = _uctypes.addressof(self.luma)
                histograms_ptr = _uctypes.addressof(self._histograms)
                if histograms_ptr & 3:
                    raise ValueError("histogram bins are not word-aligned")
                self._histogram_luma_ptr = luma_ptr
                self._histogram_histograms_ptr = histograms_ptr
                self._histogram_asm_ready = True
            except Exception:
                self._histogram_luma_ptr = None
                self._histogram_histograms_ptr = None
                self._histogram_asm_ready = False
        self._otsu_asm_ready = False
        self._otsu_asm_meta = None
        self._otsu_asm_workspace = None
        if _OTSU_ASM_SUPPORTED:
            self._otsu_asm_meta = array("I", [0] * 5)
            self._otsu_asm_workspace = array("I", [0] * 17)
            try:
                self._otsu_histograms_ptr = _uctypes.addressof(
                    self._histograms)
                self._otsu_nonzero_values_ptr = _uctypes.addressof(
                    self._nonzero_values)
                self._otsu_asm_meta_ptr = _uctypes.addressof(
                    self._otsu_asm_meta)
                self._otsu_asm_workspace_ptr = _uctypes.addressof(
                    self._otsu_asm_workspace)
                self._otsu_asm_ready = not (
                    (self._otsu_histograms_ptr |
                     self._otsu_asm_meta_ptr |
                     self._otsu_asm_workspace_ptr) & 3)
            except Exception:
                self._otsu_asm_ready = False
        self._min_pool_asm_ready = False
        self._min_pool_target_ptr = None
        if _MIN_POOL_ASM_SUPPORTED:
            try:
                self._min_pool_target_ptr = _uctypes.addressof(self.luma)
                self._min_pool_asm_ready = True
            except Exception:
                self._min_pool_asm_ready = False
        self._threshold_values = array("I", [0] * self._block_count)
        self._trusted_values = array("I", [0] * self._block_count)
        self._pixel_totals = array("I", [0, 0])
        self.thresholds = [None] * self._block_count
        self.raw_thresholds = [None] * self._block_count
        self.contrasts = [0.0] * self._block_count
        self.classes = [(0, 0)] * self._block_count
        self.class_means = [(0.0, 0.0)] * self._block_count
        self.trusted = [False] * self._block_count
        self.reasons = ["EMPTY"] * self._block_count
        self._debug_blocks = None
        if self.debug:
            self._debug_blocks = []
            for rect in self.blocks:
                self._debug_blocks.append({
                    "rect": rect,
                    "pixel_count": 0,
                    "raw_threshold": None,
                    "threshold": None,
                    "contrast": 0.0,
                    "classes": (0, 0),
                    "class_means": (0.0, 0.0),
                    "trusted": False,
                    "reason": "EMPTY",
                })

        self._diagnostics = {
            "sampling": "min4x2",
            "sampling_source": "buffer",
            "valid": False,
            "complete": False,
            "reason": "NOT_RUN",
            "roi": (0, self.floor, self.width, self.height),
            "block_count": self._block_count,
            "trusted_block_count": 0,
            "thresholds": self.thresholds,
            "contrast": self.contrasts,
            "classes": self.classes,
            "trusted": self.trusted,
            "road_pixels": 0,
            "edge_pixels": 0,
            "mask_pixels": 0,
            "contrast_min": None,
            "contrast_max": None,
            "threshold_min": None,
            "threshold_max": None,
            "row_counts": self.row_counts,
            "road_row_counts": self.road_row_counts,
        }
        if self._debug_blocks is not None:
            self._diagnostics["blocks"] = self._debug_blocks

        # One stable result dict reduces per-frame allocation.  Its buffers,
        # arrays, rows, and diagnostics are all overwritten by the next build.
        self._mask_info = {
            "luma": self.luma,
            "road_mask": self.road_mask,
            "edge_mask": self.edge_mask,
            "known_region": self.known_region,
            "width": self.width,
            "height": self.height,
            "floor": self.floor,
            "sampling": "min4x2",
            "thresholds": self.thresholds,
            "raw_thresholds": self.raw_thresholds,
            "contrasts": self.contrasts,
            "contrast": self.contrasts,
            "classes": self.classes,
            "trusted": self.trusted,
            "reasons": self.reasons,
            "reason": "NOT_RUN",
            "valid": False,
            "complete": False,
            "trusted_block_count": 0,
            "block_count": self._block_count,
            "roi": (0, self.floor, self.width, self.height),
            "road_pixels": 0,
            "edge_pixels": 0,
            "mask_pixels": 0,
            "contrast_min": None,
            "contrast_max": None,
            "threshold_min": None,
            "threshold_max": None,
            "row_counts": self.row_counts,
            "road_row_counts": self.road_row_counts,
            "diagnostics": self._diagnostics,
        }
        if self._debug_blocks is not None:
            self._mask_info["blocks"] = self._debug_blocks

    def _validate_image_format(self, image):
        format_method = getattr(image, "format", None)
        if not callable(format_method):
            # Host fakes in the explicit get_pixel fallback do not expose the
            # OpenMV Image metadata API.  Real OpenMV images do.
            return
        actual = format_method()
        expected = None
        if _image_module is not None:
            expected = getattr(_image_module, "GRAYSCALE", None)
        if expected is None:
            expected = getattr(self, "grayscale_format", None)
        if expected is not None:
            if actual != expected:
                raise ValueError("input image format is not GRAYSCALE")
            return
        if isinstance(actual, str):
            if actual.lower() not in ("grayscale", "gray", "sensor.grayscale"):
                raise ValueError("input image format is not GRAYSCALE")
            return
        # An image that advertises an unknown numeric format cannot be
        # certified by byte length alone (for example, Bayer is also 1 bpp).
        raise ValueError("cannot validate the advertised image pixel format")

    @micropython.native
    def _read_frame(self, image):
        self._validate_image_format(image)
        image_width = image.width()
        image_height = image.height()
        expected_width = self.width * self.scale_x
        expected_height = self.height * self.scale_y
        if image_width != expected_width or image_height != expected_height:
            raise ValueError("image size differs from fixed grayscale input")

        raw = None
        bytearray_reader = getattr(image, "bytearray", None)
        if callable(bytearray_reader):
            try:
                candidate = bytearray_reader()
                # This is the one buffer legality check.  The Viper kernel
                # makes no per-pixel type or bounds checks after this point.
                byte_length = len(candidate)
                is_byte_storage = isinstance(candidate, bytearray)
                if (not _IS_MICROPYTHON and isinstance(candidate, bytes)):
                    is_byte_storage = True
                if is_byte_storage and byte_length == image_width * image_height:
                    raw = candidate
            except Exception:
                raw = None

        if raw is not None:
            if _IS_MICROPYTHON:
                aligned = False
                source_ptr = None
                if _uctypes is not None:
                    try:
                        source_ptr = _uctypes.addressof(raw)
                        aligned = (source_ptr & 3) == 0
                    except Exception:
                        aligned = False
                if aligned and self._min_pool_asm_ready:
                    _min_pool_asm(source_ptr, self._min_pool_target_ptr)
                elif aligned:
                    _min_pool_words_viper(self, raw, image_width)
                else:
                    _min_pool_viper(self, raw, image_width)
            else:
                _min_pool_host(raw, self.luma, image_width,
                               self.width, self.height,
                               self.scale_x, self.scale_y)
            return "buffer"

        if _IS_MICROPYTHON:
            raise ValueError("OpenMV grayscale bytearray is missing or invalid")
        pixel_reader = getattr(image, "get_pixel", None)
        if not callable(pixel_reader):
            raise ValueError("host fallback requires get_pixel((x, y))")
        for y in range(self.height):
            source_y = y * self.scale_y
            output_y = y * self.width
            for x in range(self.width):
                source_x = x * self.scale_x
                minimum = 255
                for dy in range(self.scale_y):
                    for dx in range(self.scale_x):
                        pixel = _pixel_luma(pixel_reader((source_x + dx,
                                                          source_y + dy)))
                        if pixel < minimum:
                            minimum = pixel
                self.luma[output_y + x] = minimum
        return "host-get-pixel-fallback"

    @micropython.native
    def _otsu_for_block(self, hist_base: int, nonzero_base: int,
                        nonzero_count: int, count: int, value_sum: int):
        if self._otsu_asm_ready:
            return _otsu_asm_result(
                self, hist_base, nonzero_base, nonzero_count, count,
                value_sum)
        return _otsu_native(
            self._histograms, self._nonzero_values, nonzero_base,
            nonzero_count, count, value_sum)

    @micropython.native
    def _calculate_thresholds(self):
        if _IS_MICROPYTHON:
            _histogram_refresh(self)
        else:
            _histogram_host(
                self.luma, self._histograms, self._rectangles,
                self.width, self._block_count, self._nonzero_values,
                self._nonzero_counts, self._block_stats)

        trusted_count = 0
        threshold_min = None
        threshold_max = None
        contrast_min = None
        contrast_max = None
        for block in range(self._block_count):
            hist_base = block * 256
            nonzero_base = hist_base
            nonzero_count = self._nonzero_counts[block]
            stats_base = block * 4
            count = self._block_stats[stats_base]
            value_sum = self._block_stats[stats_base + 1]
            minimum = self._block_stats[stats_base + 2]
            maximum = self._block_stats[stats_base + 3]

            raw_threshold = None
            threshold = None
            contrast = 0.0
            classes = (0, 0)
            means = (0.0, 0.0)
            trusted = False
            if not count:
                reason = "EMPTY"
                classes = (0, 0)
            elif minimum == maximum:
                reason = "CONSTANT"
                classes = (count, 0)
                means = (float(minimum), 0.0)
            else:
                # Empty bins do not change either class.  The legacy sweep
                # gives the same score on that plateau and strict `>` keeps
                # its earliest non-empty cutoff, so populated bins alone are
                # sufficient.  Products and means stay full-width Python ints
                # and floats rather than overflowing Viper's 32-bit math.
                (best_threshold, low_class_count, high_class_count,
                 low_mean, high_mean, contrast) = self._otsu_for_block(
                    hist_base, nonzero_base, nonzero_count, count, value_sum)
                best_counts = (low_class_count, high_class_count)
                best_means = (low_mean, high_mean)
                raw_threshold = best_threshold
                if best_threshold is None or not best_counts[0] or \
                        not best_counts[1]:
                    reason = "NO_CLASS_SPLIT"
                    classes = best_counts
                    means = best_means
                else:
                    classes = best_counts
                    means = best_means
                    if contrast <= 0.0:
                        reason = "NO_CONTRAST"
                    elif contrast < self.min_contrast:
                        reason = "LOW_CONTRAST"
                    else:
                        trusted = True
                        threshold = best_threshold
                        reason = "OK"

            self.raw_thresholds[block] = raw_threshold
            self.thresholds[block] = threshold
            self.contrasts[block] = contrast
            self.classes[block] = classes
            self.class_means[block] = means
            self.trusted[block] = trusted
            self.reasons[block] = reason
            self._threshold_values[block] = threshold if trusted else 0
            self._trusted_values[block] = 1 if trusted else 0
            if trusted:
                trusted_count += 1
                contrast_min = (contrast if contrast_min is None else
                                min(contrast_min, contrast))
                contrast_max = (contrast if contrast_max is None else
                                max(contrast_max, contrast))
                threshold_min = (threshold if threshold_min is None else
                                 min(threshold_min, threshold))
                threshold_max = (threshold if threshold_max is None else
                                 max(threshold_max, threshold))

            if self._debug_blocks is not None:
                record = self._debug_blocks[block]
                rect = self.blocks[block]
                record["rect"] = rect
                record["pixel_count"] = count
                record["raw_threshold"] = raw_threshold
                record["threshold"] = threshold
                record["contrast"] = contrast
                record["classes"] = classes
                record["class_means"] = means
                record["trusted"] = trusted
                record["reason"] = reason

        self._trusted_count = trusted_count
        self._valid = trusted_count > 0
        self._complete = trusted_count == self._block_count
        self._reason = ("OK" if self._complete else
                        "PARTIAL" if self._valid else "NO_TRUSTED_BLOCK")
        self._contrast_min = contrast_min
        self._contrast_max = contrast_max
        self._threshold_min = threshold_min
        self._threshold_max = threshold_max

    def _write_masks(self):
        if _IS_MICROPYTHON:
            if self._mask_local_counts_ready:
                if self._mask_full_width_tiled_ready:
                    if _mask_viper_full_width_tiled(self) == 0:
                        return
                _mask_viper_local_counts(self)
            else:
                _mask_viper(self)
        else:
            _mask_host(
                self.luma, self.road_mask, self.edge_mask,
                self.known_region, self._rectangles,
                self._threshold_values, self._trusted_values,
                self.row_counts, self.road_row_counts, self._pixel_totals,
                self.width, self.height, self._block_count)

    @micropython.native
    def build(self, image):
        """Return a single mask-info dictionary, reusing all large buffers."""
        sampling_source = self._read_frame(image)
        self._calculate_thresholds()
        self._write_masks()

        road_pixels = self._pixel_totals[0]
        edge_pixels = self._pixel_totals[1]

        diagnostics = self._diagnostics
        diagnostics["sampling_source"] = sampling_source
        diagnostics["valid"] = self._valid
        diagnostics["complete"] = self._complete
        diagnostics["reason"] = self._reason
        diagnostics["trusted_block_count"] = self._trusted_count
        diagnostics["road_pixels"] = road_pixels
        diagnostics["edge_pixels"] = edge_pixels
        diagnostics["mask_pixels"] = edge_pixels
        diagnostics["contrast_min"] = self._contrast_min
        diagnostics["contrast_max"] = self._contrast_max
        diagnostics["threshold_min"] = self._threshold_min
        diagnostics["threshold_max"] = self._threshold_max

        info = self._mask_info
        info["sampling_source"] = sampling_source
        info["valid"] = self._valid
        info["complete"] = self._complete
        info["reason"] = self._reason
        info["trusted_block_count"] = self._trusted_count
        info["road_pixels"] = road_pixels
        info["edge_pixels"] = edge_pixels
        info["mask_pixels"] = edge_pixels
        info["contrast_min"] = self._contrast_min
        info["contrast_max"] = self._contrast_max
        info["threshold_min"] = self._threshold_min
        info["threshold_max"] = self._threshold_max
        return info
