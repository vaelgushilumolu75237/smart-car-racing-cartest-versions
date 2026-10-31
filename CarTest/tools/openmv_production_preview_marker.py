"""Decode main.py's production 24-bit Gray frame ID and CRC-8.

This is distinct from the older large magic/seq16 RAM-probe marker. Counts
represent observed new IDs only; missing IDs are never counted as displayed.
"""
from tools.openmv_preview_marker import crc8_atm


def decode_rgb888(data, width, height):
    pixels = memoryview(data).cast('B')
    if width != 320 or height != 120 or len(pixels) != width * height * 3:
        return {'status': 'shape_error', 'sequence': None}
    value = 0
    for cell in range(32):
        total = 0
        for y in (5, 6):
            for x in (5 + cell * 5, 6 + cell * 5):
                off = (y * width + x) * 3
                total += (77 * pixels[off] + 150 * pixels[off+1] + 29 * pixels[off+2]) >> 8
        if total >= 4 * 128:
            value |= 1 << cell
    gray = value & 0xffffff
    actual = value >> 24
    expected = crc8_atm(bytes((gray & 255, (gray >> 8) & 255, (gray >> 16) & 255)))
    if actual != expected:
        return {'status': 'crc_fail', 'sequence': None,
                'crc_actual': actual, 'crc_expected': expected}
    sequence = gray
    shift = gray >> 1
    while shift:
        sequence ^= shift
        shift >>= 1
    return {'status': 'ok', 'sequence': sequence}


class ProductionMarkerTracker:
    def __init__(self):
        self.last = None
        self.new_reads = 0
        self.repeats = 0
        self.backwards = 0
        self.crc_errors = 0
        self.shape_errors = 0

    def observe(self, data, width=320, height=120):
        result = decode_rgb888(data, width, height)
        result['distinct'] = False
        if result['status'] != 'ok':
            if result['status'] == 'crc_fail':self.crc_errors += 1
            else:self.shape_errors += 1
            return result
        sequence = result['sequence']
        delta = None if self.last is None else (sequence - self.last) & 0xffffff
        if delta == 0:
            self.repeats += 1
        elif delta is not None and delta >= 0x800000:
            self.backwards += 1
        else:
            self.last = sequence
            self.new_reads += 1
            result['distinct'] = True
        return result
