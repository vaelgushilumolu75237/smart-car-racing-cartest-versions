"""Exact fixed-grid kernels using CanMV's native ulab array operations.

No frame reuse, threshold changes, or topology checks are removed.
"""
try:
    import ulab.numpy as np
except ImportError:
    import numpy as np  # Host equivalence tests only.
_FLOAT = getattr(np, "float", float)
try:
    import image as _image
except ImportError:
    _image = None


def min_pool(source, target, source_width, width, height):
    pixels = np.frombuffer(source, dtype=np.uint8).reshape((height * 2, source_width))
    pooled = pixels[0::2, 0::4].copy()
    for dy, dx in ((0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3)):
        pooled = np.minimum(pooled, pixels[dy::2, dx::4])
    target[:] = pooled.tobytes()


def histogram(luma, histograms, rectangles, width, blocks, values, counts, stats):
    pixels = np.frombuffer(luma, dtype=np.uint8).reshape((len(luma) // width, width))
    native = (_image.Image(width, len(luma)//width, _image.GRAYSCALE,
                           data=luma)
              if _image is not None else None)
    hist16 = np.frombuffer(histograms, dtype=np.uint16)
    indices = np.arange(256, dtype=np.uint16)
    for block in range(blocks):
        base = block * 256
        at = block * 4
        x0, y0, x1, y1 = rectangles[at:at + 4]
        if native is not None:
            n = (y1-y0)*(x1-x0)
            bins = native.get_histogram(roi=(x0,y0,x1-x0,y1-y0), bins=256).bins()
            # Counts fit 16 bits. The upper half of each native uint32 stays 0.
            low = hist16[base*2:(base+256)*2:2]
            low[:] = np.floor(np.array(bins) * n + 0.5)
            hist16[base*2+1:(base+256)*2:2] = 0
            occupied = np.nonzero(low)[0]
            nonzero = len(occupied)
            measured = int(np.sum(low))
            total = int(np.sum(np.array(low,dtype=_FLOAT)*indices)+0.5)
            minimum = int(occupied[0])
            maximum = int(occupied[-1])
            values[base:base+nonzero] = np.array(occupied,dtype=np.uint8).tobytes()
            if measured != n:
                raise ValueError("Native histogram count mismatch %d/%d max=%s" % (measured,n,str(np.max(low))))
            counts[block] = nonzero
            stats[at] = n
            stats[at+1] = total
            stats[at+2] = minimum
            stats[at+3] = maximum
            continue
        for i in range(256):
            histograms[base + i] = 0
        ordered = np.sort(pixels[y0:y1, x0:x1].copy().reshape(((y1-y0)*(x1-x0),)))
        n = len(ordered)
        last = 0
        nonzero = 0
        total = 0
        boundaries = np.nonzero(np.diff(ordered))[0]
        for j in range(len(boundaries) + 1):
            end = int(boundaries[j]) + 1 if j < len(boundaries) else n
            value = int(ordered[last])
            amount = end - last
            histograms[base + value] = amount
            values[base + nonzero] = value
            nonzero += 1
            total += value * amount
            last = end
        counts[block] = nonzero
        stats[at] = n
        stats[at + 1] = total
        stats[at + 2] = int(ordered[0])
        stats[at + 3] = int(ordered[-1])


def masks(luma, road, edge, known, rectangles, thresholds, trusted,
          edge_rows, road_rows, totals, width, height, blocks):
    if all(rectangles[b*4] == 0 and rectangles[b*4+2] == width and
           (b == 0 or rectangles[b*4+1] >= rectangles[(b-1)*4+3]) for b in range(blocks)):
        limits = bytearray(height)
        trust = bytearray(height)
        for b in range(blocks):
            y0,y1 = rectangles[b*4+1],rectangles[b*4+3]
            limits[y0:y1] = bytes([thresholds[b]])*(y1-y0)
            trust[y0:y1] = bytes([int(bool(trusted[b]))])*(y1-y0)
        pixels = np.frombuffer(luma,dtype=np.uint8).reshape((height,width))
        row_trust = np.frombuffer(trust,dtype=np.uint8).reshape((height,1))
        bright = (pixels > np.frombuffer(limits,dtype=np.uint8).reshape((height,1))) * row_trust
        dark = (pixels <= np.frombuffer(limits,dtype=np.uint8).reshape((height,1))) * row_trust
        np.frombuffer(road,dtype=np.uint8)[:] = (bright*255).reshape((width*height,))
        np.frombuffer(edge,dtype=np.uint8)[:] = dark.reshape((width*height,))
        known[:] = bytes(width*height)
        for b in range(blocks):
            if trusted[b]:
                start,stop = rectangles[b*4+1]*width,rectangles[b*4+3]*width
                known[start:stop] = b'\x01'*(stop-start)
        white_counts = np.sum(np.array(bright,dtype=np.uint16),axis=1)
        dark_counts = np.sum(np.array(dark,dtype=np.uint16),axis=1)
        road_rows[:] = np.array(white_counts,dtype=np.uint8).tobytes()
        edge_rows[:] = np.array(dark_counts,dtype=np.uint8).tobytes()
        totals[0],totals[1] = int(np.sum(white_counts)),int(np.sum(dark_counts))
        return
    pixels = np.frombuffer(luma, dtype=np.uint8).reshape((height, width))
    road_view = np.frombuffer(road, dtype=np.uint8).reshape((height, width))
    edge_view = np.frombuffer(edge, dtype=np.uint8).reshape((height, width))
    known_view = np.frombuffer(known, dtype=np.uint8).reshape((height, width))
    road_view[:] = 0
    edge_view[:] = 0
    known_view[:] = 0
    edge_rows[:] = bytes(height)
    road_rows[:] = bytes(height)
    totals[0] = totals[1] = 0
    for block in range(blocks):
        if not trusted[block]:
            continue
        at = block * 4
        x0, y0, x1, y1 = rectangles[at:at + 4]
        bright = pixels[y0:y1, x0:x1] > thresholds[block]
        road_view[y0:y1, x0:x1] = bright * 255
        edge_view[y0:y1, x0:x1] = 1 - bright
        known_view[y0:y1, x0:x1] = 1
        bright_bytes = bright.tobytes()
        row_width = x1-x0
        for j in range(y1 - y0):
            count = bright_bytes.count(b'\x01', j*row_width, (j+1)*row_width)
            dark = x1 - x0 - count
            road_rows[y0 + j] += count
            edge_rows[y0 + j] += dark
            totals[0] += count
            totals[1] += dark


def region_segments(guard, floor):
    width, height = guard.width, guard.height
    mask = guard._current_mask
    known = guard._current_known
    guard._trusted_rows[:] = bytes(height)
    for y in range(height):
        guard._segment_starts[y] = guard._segment_ends[y] = 0
    guard._segment_count_buffer[0] = 0
    start = -1
    segments = 0
    all_known = True
    partial = False
    for y in range(floor, height):
        offset = y*width
        if width-mask.count(b'\x00',offset,offset+width) != guard._current_rows[y]:
            return -1
        count = width-known.count(b'\x00',offset,offset+width)
        if count != width:
            all_known = False
            if count:
                partial = True
        if count == width:
            guard._trusted_rows[y] = 1
            if start < 0:
                start = y
        elif start >= 0:
            guard._segment_starts[segments] = start
            guard._segment_ends[segments] = y
            segments += 1
            start = -1
    if start >= 0:
        guard._segment_starts[segments] = start
        guard._segment_ends[segments] = height
        segments += 1
    if partial:
        return -2
    guard._segment_count_buffer[0] = segments
    return 1 if all_known else 0


_BIT_COUNTS = bytes(bin(value).count("1") for value in range(256))
_BIT_LOW = bytes(next((bit for bit in range(8) if value & (1<<bit)),0) for value in range(256))
_BIT_HIGH = bytes(next((bit for bit in range(7,-1,-1) if value & (1<<bit)),0) for value in range(256))


def internal_component(guard, floor, band_end):
    """Exact 8-connectivity, growing packed row bitsets in native ints."""
    width = guard.width
    if _image is not None and hasattr(_image,"BINARY"):
        native = _image.Image(width,guard.height,_image.GRAYSCALE,data=guard._current_mask[:])
        native.binary([(1,255)],to_bitmap=True)
        data = native.bytearray()
        stride = ((width+31)//32)*4
        if len(data) != stride*guard.height:
            raise ValueError("Unexpected native bitmap stride")
    else:
        pixels = (np.frombuffer(guard._current_mask,dtype=np.uint8) != 0).reshape((guard.height,width))
        packed = np.zeros((guard.height,width//8),dtype=np.uint8)
        for bit in range(8):
            packed = np.bitwise_or(packed,np.left_shift(pixels[:,bit::8],bit))
        data = np.array(packed,dtype=np.uint8).tobytes()
        stride = width//8
    rows = [int.from_bytes(data[y*stride:(y+1)*stride], "little") & ((1<<width)-1)
            for y in range(floor,band_end)]
    height = len(rows)
    full_bits = (1<<width)-1
    boundary = []
    for y in range(height):
        bits = rows[y]
        if y == 0 or y == height-1:
            outer = bits
        else:
            right = full_bits
            for xbyte in range(width//8-1,-1,-1):
                inverse = data[(y+floor)*stride+xbyte] ^ 255
                if inverse:
                    right = full_bits ^ ((1<<(xbyte*8+_BIT_HIGH[inverse]+1))-1)
                    break
            outer = ((bits^(bits+1))>>1) | right
        boundary.append(outer)
        rows[y] &= ~outer
    pending = []
    for y in range(height):
        neighbor = (boundary[y-1] if y else 0) | (boundary[y+1] if y+1 < height else 0)
        seeds = rows[y] & (neighbor | (neighbor<<1) | (neighbor>>1))
        if seeds:
            pending.append((y,seeds))
    while pending:
        y,bits = pending.pop()
        grown = bits & rows[y]
        if not grown:
            continue
        while True:
            expanded = (grown | (grown<<1) | (grown>>1)) & rows[y]
            if expanded == grown:
                break
            grown = expanded
        rows[y] ^= grown
        adjacent = grown | (grown<<1) | (grown>>1)
        if y and adjacent & rows[y-1]:
            pending.append((y-1,adjacent))
        if y+1 < height and adjacent & rows[y+1]:
            pending.append((y+1,adjacent))
    last_column = 1 << (width-1)
    guard._visited[:] = bytes(len(guard._visited))
    for seed_y in range(height):
        while rows[seed_y]:
            seed = rows[seed_y] & -rows[seed_y]
            pending = [(seed_y,seed)]
            area = 0
            x0,y0,x1,y1 = width,height,-1,-1
            touches = False
            while pending:
                y,bits = pending.pop()
                grown = bits & rows[y]
                if not grown:
                    continue
                while True:
                    expanded = (grown | (grown<<1) | (grown>>1)) & rows[y]
                    if expanded == grown:
                        break
                    grown = expanded
                rows[y] ^= grown
                adjacent = grown | (grown<<1) | (grown>>1)
                if y and adjacent & rows[y-1]:
                    pending.append((y-1,adjacent))
                if y+1 < height and adjacent & rows[y+1]:
                    pending.append((y+1,adjacent))
                if y == 0 or y == height-1 or grown & (1|last_column):
                    touches = True
                if not touches:
                    row_bytes = grown.to_bytes(stride,"little")
                    for xbyte,value in enumerate(row_bytes):
                        if value:
                            area += _BIT_COUNTS[value]
                            x0 = min(x0,xbyte*8+_BIT_LOW[value])
                            x1 = max(x1,xbyte*8+_BIT_HIGH[value])
                    y0 = min(y0,y)
                    y1 = max(y1,y)
            if touches:
                continue
            bw,bh = x1-x0+1,y1-y0+1
            if (area < guard.component_min_area or area > guard.component_max_area or
                    bw < guard.component_min_extent or bh < guard.component_min_extent or
                    bw > guard.component_max_extent or bh > guard.component_max_extent):
                continue
            hole = guard._component_encloses_hole_host((x0,y0+floor,x1,y1+floor),floor,band_end)
            if hole:
                return hole
    return 0


def extract_rows(detector):
    width, height = detector.width, detector.height
    road = np.frombuffer(detector._current_road, dtype=np.uint8)
    known = np.frombuffer(detector._current_known, dtype=np.uint8)
    runs = np.bitwise_and(road != 0, known != 0).tobytes()
    luma, edge = detector._current_luma, detector._current_edge
    known, road = detector._current_known, detector._current_road
    candidates, counts = detector._candidate_data, detector._counts
    for layer in range(detector.row_count):
        y = height-1-layer
        row, limit = y*width, (y+1)*width
        at = row
        count = 0
        while at < limit:
            start = runs.find(b'\x01',at,limit)
            if start < 0:
                break
            stop = runs.find(b'\x00',start,limit)
            if stop < 0:
                stop = limit
            at = stop
            run_width = stop-start
            if run_width < detector.min_width or run_width > detector.max_width:
                continue
            left, right = start-1, stop
            if left < row or right >= limit:
                continue
            if not known[left] or not known[right] or road[left] or road[right] or not edge[left] or not edge[right]:
                continue
            if count >= detector.max_per_row:
                return -1
            offset = (layer*detector.max_per_row+count)*4
            candidates[offset] = start-row
            candidates[offset+1] = start+stop-1-row*2
            candidates[offset+2] = run_width
            candidates[offset+3] = sum(luma[start:stop])*1000//run_width-(luma[left]+luma[right])*500
            count += 1
        counts[layer] = count
    return 0
