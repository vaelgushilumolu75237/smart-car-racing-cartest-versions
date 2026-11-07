import json
from pathlib import Path
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.capture_openmv_ring_entry import EntryCollector, receive_pull_capture


class RingEntryCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.collector = EntryCollector(Path(self.temp.name))

    def tearDown(self):
        self.collector.frame_file.close()
        self.temp.cleanup()

    def frame(self, sequence=1):
        self.collector.line("ENTRY_FRAME " + json.dumps({
            "sequence": sequence, "row_status": [1]*120,
            "row_edge2": [20]*60+[100]*60, "baseline_valid": [1]*60,
            "baseline_left2": [20]*60, "baseline_right2": [100]*60,
            "outer_shift_x2": [0, 0]}))

    def meta(self, raw):
        self.collector.line("ENTRY_IMAGE_META " + json.dumps({
            "image_id": 0, "sequence": 1, "width": 320, "height": 120,
            "byte_count": len(raw), "crc32": "%08x" % (zlib.crc32(raw)&0xffffffff)}))

    def test_chunked_stdout_reconstructs_exact_crc_verified_image(self):
        self.frame()
        raw = bytes(range(256))*150
        self.meta(raw)
        for offset in range(0, len(raw), 384):
            line = "ENTRY_IMAGE_CHUNK 0 %d %s\n" % (offset, raw[offset:offset+384].hex())
            for piece in (line[:13], line[13:78], line[78:]):
                self.collector.feed(piece)
        self.collector.line("ENTRY_IMAGE_END 0")
        self.assertEqual((Path(self.temp.name)/"image-0-seq-00001.pgm").read_bytes(),
                         b"P5\n320 120\n255\n"+raw)

    def test_missing_chunk_and_crc_corruption_are_rejected(self):
        self.frame()
        raw = bytes(38400)
        self.meta(raw)
        with self.assertRaisesRegex(ValueError, "missing"):
            self.collector.line("ENTRY_IMAGE_CHUNK 0 384 " + bytes(384).hex())
        self.collector.images[0]["data"] = bytearray([1])*38400
        with self.assertRaisesRegex(ValueError, "CRC"):
            self.collector.line("ENTRY_IMAGE_END 0")
        self.assertEqual(self.collector.close()["incomplete_image_ids"], [0])

    def test_sequence_wrap_and_backward_rejection(self):
        self.frame(65535)
        self.frame(0)
        with self.assertRaisesRegex(ValueError, "backward"):
            self.frame(65535)

    def test_truncated_record_is_incomplete_but_finished_repl_is_not(self):
        self.collector.feed("ENTRY_FRAME {\"sequence\":")
        self.assertTrue(self.collector.close()["errors"])
        self.collector.errors.clear()
        self.collector.pending = ">>> "
        self.collector.done = True
        self.assertFalse(self.collector.close()["errors"])

    def test_stdout_loss_is_not_reported_as_complete(self):
        self.frame()
        self.collector.line("ENTRY_CAPTURE_DONE frame_count=2 image_count=0 error=0")
        self.assertTrue(any("counts differ" in x for x in self.collector.close()["errors"]))

    def test_pull_channel_reconstructs_crc_verified_objects(self):
        self.frame()
        row = self.collector.frames.pop()
        self.collector.last_sequence = None
        config = b'{"thresholds_tracker":{}}'
        frame = json.dumps(row).encode()
        image = bytes(range(256))*150
        payload = config+frame+image

        def describe(offset, raw):
            return {"offset": offset, "size": len(raw),
                    "crc32": "%08x" % (zlib.crc32(raw)&0xffffffff)}
        meta = {"schema": "entry-pull-v1", "config": describe(0, config),
                "frames": [dict(describe(len(config), frame), sequence=1)],
                "images": [dict(describe(len(config)+len(frame), image), image_id=0,
                                sequence=1, capture_ms=1, width=320, height=120,
                                byte_count=38400)], "frame_count": 1,
                "processed_frame_count": 99, "image_count": 1, "error": 0}
        meta_raw = json.dumps(meta).encode()

        def call(method, channel, *args):
            raw = meta_raw if channel == 0 else payload
            if method == "_channel_size":
                return len(raw)
            offset, size = args
            return raw[offset:offset+size]
        receive_pull_capture(call, self.collector, 0, 1)
        self.assertEqual(self.collector.completed_images, [0])
        self.assertEqual(self.collector.close()["errors"], [])


if __name__ == "__main__":
    unittest.main()
