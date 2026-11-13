from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "overlay/ports/stm32/stm_jpeg.c").read_text(encoding="utf-8")
HEADER = (ROOT / "overlay/lib/imlib/imlib.h").read_text(encoding="utf-8")


class SourceContractTests(unittest.TestCase):
    def test_timeout_api_is_defaulted_bounded_and_declared(self):
        self.assertIn("static uint32_t jpeg_encode_timeout_ms = JPEG_CODEC_TIMEOUT;", SOURCE)
        self.assertIn("jpeg_preview_guard_timeout_valid(timeout_ms, JPEG_CODEC_TIMEOUT)", SOURCE)
        self.assertIn("uint32_t jpeg_get_encode_timeout_ms(void);", HEADER)
        self.assertIn("bool jpeg_set_encode_timeout_ms(uint32_t timeout_ms);", HEADER)

    def test_only_encode_waits_use_configurable_timeout(self):
        start = SOURCE.index("int jpeg_compress(image_t *src")
        end = SOURCE.index("static void jpeg_decompress_data_ready_abort", start)
        encode = SOURCE[start:end]
        self.assertGreaterEqual(encode.count("timeout_ms - elapsed"), 2)
        self.assertNotIn("JPEG_CODEC_TIMEOUT - elapsed", encode)
        self.assertIn("mp_event_wait_ms(JPEG_CODEC_TIMEOUT - elapsed)", SOURCE[end:])

    def test_exception_path_cleans_then_reraises(self):
        start = SOURCE.index("int jpeg_compress(image_t *src")
        end = SOURCE.index("static void jpeg_decompress_data_ready_abort", start)
        encode = SOURCE[start:end]
        catch = encode.index("if (nlr_push(&nlr) != 0)")
        cleanup = encode.index("jpeg_compress_cleanup(dst", catch)
        rereraise = encode.index("nlr_jump(exception)", cleanup)
        self.assertLess(catch, cleanup)
        self.assertLess(cleanup, rereraise)
        self.assertIn("static bool jpeg_compress_abort", SOURCE)
        self.assertIn("static void jpeg_compress_unbind_callbacks", SOURCE)


if __name__ == "__main__":
    unittest.main()
