"""Exact fixed-grid kernels using CanMV's native ulab array operations.

Native kernels keep fresh frames and full topology verification. Native Otsu
may resolve threshold ties differently from the reference histogram sweep.
"""
try:
    import ulab.numpy as np
except ImportError:
    import numpy as np  # Host equivalence tests only.
try:
    import cv_lite as _cv
except ImportError:
    _cv = None
from struct import pack_into as _pack_into

_FLOAT = getattr(np, "float", float)
try:
    import image as _image
except ImportError:
    _image = None


def min_pool(source, target, source_width, width, height):
    pixels = np.frombuffer(source, dtype=np.uint8).reshape((height * 2, source_width))
    pairs = np.minimum(pixels[:, 0::2], pixels[:, 1::2])
    fours = np.minimum(pairs[:, 0::2], pairs[:, 1::2])
    target[:] = np.minimum(fours[0::2, :], fours[1::2, :]).tobytes()


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
                # Reject inconsistent native/array output before it can affect
                # a threshold. Recount this same frame using integer byte counts.
                _histogram_recount(luma, histograms, base, x0, y0, x1, y1,
                                   width, values, counts, stats, block)
                continue
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


def _histogram_recount(luma, histograms, base, x0, y0, x1, y1,
                       width, values, counts, stats, block):
    region = b"".join(bytes(luma[y*width+x0:y*width+x1])
                      for y in range(y0, y1))
    nonzero = 0
    total = 0
    minimum, maximum = 256, 0
    for value in range(256):
        amount = region.count(bytes([value]))
        histograms[base+value] = amount
        if amount:
            values[base+nonzero] = value
            nonzero += 1
            total += value*amount
            if value < minimum:
                minimum = value
            maximum = value
    counts[block] = nonzero
    at = block*4
    stats[at] = len(region)
    stats[at+1] = total
    stats[at+2] = minimum
    stats[at+3] = maximum


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
    guard._segment_starts[:] = bytes(height)
    guard._segment_ends[:] = bytes(height)
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


def _white_exterior_runs(binary, width, floor, band_end):
    """Prove every four-connected white run reaches the trusted band's edge.

    Unlike external contours this enumerates nested white components too.
    A bounded inconclusive scan falls back to the complete island verifier.
    """
    parents = []
    exterior = []
    previous = []
    run_count = 0
    for y in range(floor, band_end):
        row = y * width
        limit = row + width
        current = []
        at = row
        cursor = 0
        while at < limit:
            start = binary.find(b'\x00', at, limit)
            if start < 0:
                break
            stop = binary.find(b'\x01', start, limit)
            if stop < 0:
                stop = limit
            at = stop
            left, right = start - row, stop - row
            run_count += 1
            if run_count > 128:
                return False
            while cursor < len(previous) and previous[cursor][1] <= left:
                cursor += 1
            index = cursor
            label = -1
            while index < len(previous) and previous[index][0] < right:
                other = previous[index][2]
                while parents[other] != other:
                    other = parents[other]
                if label < 0:
                    label = other
                elif other > label:
                    parents[label] = other
                    exterior[other] = exterior[other] or exterior[label]
                    label = other
                elif other < label:
                    parents[other] = label
                    exterior[label] = exterior[label] or exterior[other]
                index += 1
            if label < 0:
                label = len(parents)
                parents.append(label)
                exterior.append(False)
            if y == floor or y == band_end - 1 or left == 0 or right == width:
                exterior[label] = True
            current.append((left, right, label))
        previous = current
    # Every union points to the larger label. Resolve in
    # reverse order rather than repeatedly walking long straight-road chains.
    for label in range(len(parents) - 1, -1, -1):
        exterior[label] = exterior[parents[label]]
        if not exterior[label]:
            return False
    return True


def internal_component(guard, floor, band_end):
    # If every dark run touches a horizontal image edge, every dark pixel
    # already belongs to an exterior component. This proves absence of an
    # internal island using this frame alone; internal runs use full checking.
    width = guard.width
    binary = (np.frombuffer(guard._current_mask, dtype=np.uint8) != 0).tobytes()
    for y in range(floor, band_end):
        if b'\x01' in binary[y*width:(y+1)*width].strip(b'\x01'):
            break
    else:
        return 0
    if _white_exterior_runs(binary, width, floor, band_end):
        return 0
    if _cv is not None:
        try:
            if _native_no_candidate(guard, floor, band_end):
                return 0
        except Exception:
            pass  # Native failure always falls back to complete bitset verification.
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
    import k230_settings as settings
    step = getattr(detector, 'path_row_step', getattr(settings, 'PATH_ROW_STEP', 1)) if _cv is not None else 1
    phase = getattr(detector, 'path_row_phase', getattr(settings, 'PATH_ROW_PHASE', 0)) if _cv is not None else 0
    if step not in (1, 2, 3) or not 0 <= phase < step:
        raise ValueError('invalid path row sampling')
    counts[:] = bytes(len(counts))
    layers = range(phase, detector.row_count, step)
    reference_layer = height - 1 - (detector.reference_y_milli + 500) // 1000
    if (step > 1 and 0 <= reference_layer < detector.row_count and
            reference_layer % step != phase):
        # Keep the same actually measured control row while refreshing the
        # remaining cache rows. A rotating nearest row resets position PD.
        layers = list(layers) + [reference_layer]
    if step > 1:
        # The ring observer requires a real contiguous near-field baseline.
        # Keep its four measured adjacent rows in every phase; sparse rows
        # alone never satisfy TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS=4.
        contiguous = getattr(detector.config, 'TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS', 4)
        layers = sorted(set(layers) | set(range(min(contiguous, detector.row_count))))
    for layer in layers:
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


def connect_rows(self):
    """Exact two-best path search with a nonnegative-penalty upper bound."""
    counts = self._counts
    candidates = self._candidate_data
    states = self._state_data
    width = (self.width)
    height = (self.height)
    layers = (self.row_count)
    max_per_row = (self.max_per_row)
    max_gap = (self.max_gap)
    max_width_change = (self.max_width_change)
    max_slope_milli = (self.max_slope_milli)
    link_tolerance_milli = (self.link_tolerance_milli)
    reference_y_milli = (self.reference_y_milli)
    state_capacity = (self.state_capacity)
    sentinel = (-2147483647)

    for layer in range(layers):
        current_y = height - 1 - layer
        layer_reference_delta = abs(current_y * 1000 - reference_y_milli)
        for slot in range(counts[layer]):
            node = layer * max_per_row + slot
            cbase = node * 4
            center_twice = (candidates[cbase + 1])
            run_width = (candidates[cbase + 2])
            contrast_milli = (candidates[cbase + 3])
            if contrast_milli < 0:
                contrast_milli = 0
            elif contrast_milli > 80000:
                contrast_milli = 80000
            local_reward = 10000 + (contrast_milli * 25 + 500) // 1000

            first = node * 2
            first_base = first * 10
            second_base = (first + 1) * 10
            states[first_base] = sentinel
            states[first_base + 1] = -1
            states[first_base + 8] = -1
            states[first_base + 9] = 0
            states[second_base] = sentinel
            states[second_base + 1] = -1
            states[second_base + 8] = -1
            states[second_base + 9] = 0

            distance_limit = max_gap + 1
            if distance_limit > layer:
                distance_limit = layer
            for distance in range(1, distance_limit + 1):
                previous_layer = layer - distance
                previous_count = counts[previous_layer]
                if previous_count == 0:
                    continue
                old_y = height - 1 - previous_layer
                for previous_slot in range(previous_count):
                    previous_node = previous_layer * max_per_row + previous_slot
                    previous_center_twice = (
                        candidates[previous_node * 4 + 1])
                    previous_width = (candidates[previous_node * 4 + 2])
                    width_delta = run_width - previous_width
                    if width_delta < 0:
                        width_delta = -width_delta
                    if width_delta > max_width_change * distance:
                        continue
                    center_step_twice = center_twice - previous_center_twice
                    if center_step_twice < 0:
                        center_step_twice = -center_step_twice
                    actual_step_milli = center_step_twice * 500
                    if actual_step_milli > (
                            max_slope_milli * distance +
                            link_tolerance_milli):
                        continue

                    edge_reward = (local_reward + distance * 550 -
                        (width_delta * 1925 + 5) // 10 - (distance - 1) * 2000)
                    step_slope_milli = (actual_step_milli + distance // 2) // distance

                    for previous_slot_state in range(2):
                        previous_state = previous_node * 2 + previous_slot_state
                        previous_base = previous_state * 10
                        previous_score = (states[previous_base])
                        if previous_score <= sentinel // 2:
                            continue
                        # A candidate can never exceed this score: the
                        # omitted center penalty is nonnegative. Once two
                        # better paths exist, skip prediction and field reads.
                        upper_score = previous_score + edge_reward
                        if (upper_score <= states[first_base] and
                                upper_score <= states[second_base]):
                            continue
                        parent_state = (states[previous_base + 1])
                        if parent_state < -1 or parent_state >= state_capacity:
                            continue

                        predicted_twice = previous_center_twice
                        if parent_state >= 0:
                            parent_node = parent_state // 2
                            prior_layer = parent_node // max_per_row
                            base_dy = previous_layer - prior_layer
                            if base_dy <= 0 or prior_layer >= layers:
                                continue
                            prior_center_twice = (
                                candidates[parent_node * 4 + 1])
                            prediction_delta = (
                                (previous_center_twice - prior_center_twice) *
                                distance)
                            if prediction_delta >= 0:
                                predicted_twice += prediction_delta // base_dy
                            else:
                                predicted_twice -= (-prediction_delta) // base_dy

                        center_error_twice = center_twice - predicted_twice
                        if center_error_twice < 0:
                            center_error_twice = -center_error_twice
                        center_error_milli = center_error_twice * 500
                        if center_error_milli > (
                                max_slope_milli * distance +
                                link_tolerance_milli):
                            continue

                        center_penalty = (
                            center_error_milli * 550 + 500) // 1000
                        score = upper_score - center_penalty

                        reference_delta = layer_reference_delta
                        old_reference_delta = (states[previous_base + 5])
                        if reference_delta < old_reference_delta:
                            reference_node = node
                        else:
                            reference_node = (states[previous_base + 4])
                            reference_delta = old_reference_delta
                        slope_sum = (states[previous_base + 6]) + step_slope_milli
                        width_sum = (states[previous_base + 7]) + width_delta
                        path_count = (states[previous_base + 2]) + 1
                        bottom_y = (states[previous_base + 3])
                        first_score = (states[first_base])
                        if score > first_score:
                            states[second_base:second_base+10] = states[first_base:first_base+10]
                            _pack_into('<10i', states, first_base * 4,
                                score, previous_state, path_count, bottom_y,
                                reference_node, reference_delta, slope_sum,
                                width_sum, -1, 0)
                        elif (previous_state != (states[first_base + 1]) and
                              score > (states[second_base])):
                            _pack_into('<10i', states, second_base * 4,
                                score, previous_state, path_count, bottom_y,
                                reference_node, reference_delta, slope_sum,
                                width_sum, -1, 0)

            reference_delta = layer_reference_delta
            if local_reward > (states[first_base]):
                states[second_base:second_base+10] = states[first_base:first_base+10]
                _pack_into('<10i', states, first_base * 4,
                    local_reward, -1, 1, current_y, node,
                    reference_delta, 0, 0, -1, 0)
            elif ((states[first_base + 1]) != -1 and
                  local_reward > (states[second_base])):
                _pack_into('<10i', states, second_base * 4,
                    local_reward, -1, 1, current_y, node,
                    reference_delta, 0, 0, -1, 0)
    return 0




def single_edge_positions(tracker):
    """Enumerate every dark/white transition, preserving original gate order."""
    width, height = tracker.width, tracker.height
    edge = np.frombuffer(tracker._current_edge, dtype=np.uint8)
    road = np.frombuffer(tracker._current_road, dtype=np.uint8)
    left = np.bitwise_and(edge[:-1] != 0, road[1:] != 0).tobytes() + b'\x00'
    right = b'\x00' + np.bitwise_and(edge[1:] != 0, road[:-1] != 0).tobytes()
    result = ([], [])
    for side, bits in enumerate((left, right)):
        rows = result[side]
        for y in range(height):
            start, end = y*width+1, (y+1)*width-1
            positions = []
            while start < end:
                found = bits.find(b'\x01', start, end)
                if found < 0:
                    break
                positions.append(found-y*width)
                start = found+1
            rows.append(positions)
    return result


def batch_otsu(builder):
 blocks=builder._block_count
 raw=np.array(np.frombuffer(builder._histograms,dtype=np.uint16)[::2],dtype=_FLOAT).reshape((blocks,256))
 low=raw.copy();sums=raw*np.arange(256).reshape((1,256));step=1
 while step<256:
  low[:,step:]=low[:,step:]+low[:,:-step]
  sums[:,step:]=sums[:,step:]+sums[:,:-step]
  step*=2
 counts=np.array([builder._block_stats[b*4] for b in range(blocks)]).reshape((blocks,1))
 totals=np.array([builder._block_stats[b*4+1] for b in range(blocks)]).reshape((blocks,1))
 high=counts-low
 low_mean=sums/np.maximum(low,1)
 high_mean=(totals-sums)/np.maximum(high,1)
 diff=high_mean-low_mean
 scores=((low*high/np.maximum(counts*counts,1))*diff)*diff
 valid=(low>0)*(high>0)
 scores=scores*valid-(1-valid)
 results=[]
 for b in range(blocks):
  if builder._nonzero_counts[b]<2:
   results.append((0,0,0,0.0,0.0,0.0));continue
  value=int(np.argmax(scores[b,:]))
  results.append((value,int(low[b,value]),int(high[b,value]),float(low_mean[b,value]),float(high_mean[b,value]),float(diff[b,value])))
 return results


def native_thresholds(builder):
 luma=builder.luma;rectangles=builder._rectangles;width=builder.width
 blocks=builder._block_count;stats=builder._block_stats
 pixels=np.frombuffer(luma,dtype=np.uint8).reshape((len(luma)//width,width))
 rgb=getattr(builder,'_native_rgb',None)
 if rgb is None:
  rgb=np.empty((len(luma),3),dtype=np.uint8)
  builder._native_rgb=rgb
 rgb[:,:]=pixels.reshape((len(luma),1))
 results=[];binaries=[]
 for b in range(blocks):
  x0,y0,x1,y1=rectangles[b*4:b*4+4]
  w,h=x1-x0,y1-y0;n=w*h
  gray=pixels[y0:y1,x0:x1]
  block_rgb=(rgb[y0*width:y1*width,:] if x0==0 and x1==width else
             np.empty((n,3),dtype=np.uint8))
  if x0!=0 or x1!=width:
   block_rgb[:,:]=gray.copy().reshape((n,1))
  binary=_cv.rgb888_erode([h,w],block_rgb,1,1,0).reshape((h,w))
  data=binary.tobytes();low_count=data.count(b'\x00');high_count=n-low_count
  total=int(np.sum(gray))
  stats[b*4]=n;stats[b*4+1]=total;stats[b*4+2]=int(np.min(gray));stats[b*4+3]=int(np.max(gray))
  if not low_count or not high_count:
   results.append((0,0,0,0.0,0.0,0.0))
  else:
   dark=binary==0
   # Multiplication is at most 255; ulab's reduction accumulates in float,
   # exactly for this <=4800-cell plane. No intermediate uint16 copy is needed.
   low_total=int(np.sum(gray*dark)+0.5)
   threshold=int(np.max(gray*dark))
   low_mean=low_total/low_count;high_mean=(total-low_total)/high_count
   results.append((threshold,low_count,high_count,low_mean,high_mean,high_mean-low_mean))
  binaries.append(data)
 builder._native_binaries=binaries
 return results


def native_masks(builder):
 rectangles=builder._rectangles;width=builder.width;height=builder.height
 blocks=builder._block_count
 if not all(rectangles[b*4]==0 and rectangles[b*4+2]==width and
            (b==0 or rectangles[b*4+1]>=rectangles[(b-1)*4+3]) for b in range(blocks)):
  return False
 road=builder.road_mask;edge=builder.edge_mask;known=builder.known_region
 road_rows=builder.road_row_counts;edge_rows=builder.row_counts;totals=builder._pixel_totals
 road[:]=bytes(width*height);known[:]=bytes(width*height)
 for b in range(blocks):
  if not builder._trusted_values[b]:continue
  y0,y1=rectangles[b*4+1],rectangles[b*4+3]
  data=builder._native_binaries[b]
  start,stop=y0*width,y1*width
  road[start:stop]=data
  known[start:stop]=b'\x01'*(stop-start)
 pixels=np.frombuffer(road,dtype=np.uint8)
 trusted=np.frombuffer(known,dtype=np.uint8)
 # Trusted cells are 1 and the binary road is 0/255. Unknown cells were
 # cleared above, so subtraction preserves unknown=0 with no extra mask.
 bright=pixels!=0
 edge[:]=(trusted-bright).tobytes()
 bright_counts=np.sum(bright.reshape((height,width)),axis=1)
 # All trusted rectangles span full rows, so each row contains either width
 # known cells or none. Counts stay <=80 even with ulab's uint8 reduction.
 dark_counts=trusted.reshape((height,width))[:,0]*width-bright_counts
 road_rows[:]=np.array(bright_counts,dtype=np.uint8).tobytes()
 edge_rows[:]=np.array(dark_counts,dtype=np.uint8).tobytes()
 totals[0]=int(np.sum(bright_counts));totals[1]=int(np.sum(dark_counts))
 return True


def pack_binary(source):
    """Pack all current nonzero cells with the wire protocol's LSB bit order."""
    size = 80 * 60
    data = source[:size]
    if len(data) < size:
        data = bytes(data) + bytes(size - len(data))
    pixels = (np.frombuffer(data, dtype=np.uint8) != 0).reshape((size // 8, 8))
    result = np.zeros(size // 8, dtype=np.uint8)
    for bit in range(8):
        result = np.bitwise_or(result, np.left_shift(pixels[:, bit], bit))
    return bytearray(np.array(result, dtype=np.uint8).tobytes())


def _native_no_candidate(guard,floor,band_end):
 width=guard.width
 pixels=np.frombuffer(guard._current_mask,dtype=np.uint8).reshape((guard.height,width))
 pending=[(0,floor,width,band_end)]
 calls=0
 while pending:
  x0,y0,x1,y1=pending.pop()
  w,h=x1-x0,y1-y0
  region=pixels[y0:y1,x0:x1]
  white=np.array(region==0,dtype=np.uint8)
  # Separate tiles share one native contour call. The zero spacer prevents
  # even diagonal foreground connectivity between the dark and white tiles.
  # White's inserted horizontal connectors encode four-connectivity exactly.
  graph=np.zeros((h,w*3),dtype=np.uint8)
  graph[:,:w]=region
  graph[:,w+1::2]=white
  graph[:,w+2::2]=np.bitwise_and(white[:,:-1],white[:,1:])
  contours=_cv.grayscale_find_blobs([h,w*3],graph,1,255,0,1)
  calls+=2
  for i in range(0,len(contours),4):
   x,y,bw,bh=contours[i:i+4]
   if x<w:
    if (x>0 and y>0 and x+bw<w and y+bh<h and
        guard.component_min_extent<=bw<=guard.component_max_extent and
        guard.component_min_extent<=bh<=guard.component_max_extent and
        bw*bh>=guard.component_min_area):return False
    continue
   x-=w+1
   x//=2;bw=(bw+1)//2
   if (x>0 and y>0 and x+bw<w and y+bh<h and
       bw>=guard.component_min_extent+2 and bh>=guard.component_min_extent+2):
    pending.append((x0+x,y0+y,x0+x+bw,y0+y+bh))
  if calls>20 or len(pending)>10:return False
 return True
