import ast
from pathlib import Path
import unittest
from tools.openmv_production_preview_marker import decode_rgb888, ProductionMarkerTracker


ROOT = Path(__file__).resolve().parents[1]


def production_frame(sequence):
    tree = ast.parse((ROOT/'openmv/main.py').read_text(encoding='utf8'))
    functions = []
    for name in ('_preview_marker_crc8', '_draw_preview_marker_pixels_viper'):
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
        fn.decorator_list = []
        functions.append(fn)
    namespace = {'ptr8': lambda x: x}
    exec(compile(ast.Module(body=functions, type_ignores=[]), '<actual-production-marker>', 'exec'), namespace)
    gray = (sequence ^ (sequence >> 1)) & 0xffffff
    frame = bytearray([127]) * (320*120)
    namespace['_draw_preview_marker_pixels_viper'](frame, 320, gray, namespace['_preview_marker_crc8'](gray))
    return bytes(channel for pixel in frame for channel in (pixel, pixel, pixel))


class ProductionMarkerTests(unittest.TestCase):
    def test_decodes_actual_main_writer_layout_bit_order_and_crc(self):
        for sequence in (0, 1, 2, 255, 65535, 65536, 0x123456, 0xffffff):
            with self.subTest(sequence=sequence):
                self.assertEqual(decode_rgb888(production_frame(sequence),320,120),
                                 {'status':'ok','sequence':sequence})

    def test_crc_corruption_is_rejected(self):
        data = bytearray(production_frame(100))
        for y in (5,6):
            for x in (5+24*5,6+24*5):
                off=(y*320+x)*3
                data[off:off+3]=bytes(255-v for v in data[off:off+3])
        self.assertEqual(decode_rgb888(data,320,120)['status'],'crc_fail')

    def test_only_observed_forward_ids_count_including_wrap(self):
        tracker=ProductionMarkerTracker()
        for sequence in (0xfffffe,0xfffffe,0xffffff,0,2,1):
            tracker.observe(production_frame(sequence))
        self.assertEqual((tracker.new_reads,tracker.repeats,tracker.backwards),(4,1,1))

    def test_wrong_frame_shape_is_rejected(self):
        self.assertEqual(decode_rgb888(b'\0'*100,320,120)['status'],'shape_error')


if __name__=='__main__':unittest.main()
