from pathlib import Path
import tempfile
import unittest

from tools import benchmark_openmv_preview as bench
from tools import build_openmv_ram_preview_probe as camera_builder
from tools import build_openmv_synthetic_marker_probe as synthetic_builder


class BuildOpenMVSyntheticMarkerProbeTests(unittest.TestCase):
    def test_synthetic_probe_reuses_marker_and_passes_only_explicit_guard(self):
        source = synthetic_builder.build_probe_source()
        compile(source, "synthetic-marker-ram-source.py", "exec")
        marker = camera_builder.VIPER_MARKER_FUNCTION
        self.assertIn(marker, source.replace("\r\n", "\n"))
        self.assertIn(marker,
                      camera_builder.build_probe_source().replace("\r\n", "\n"))
        self.assertIn("SYNTHETIC_SEQUENCE_FIRST = 65532", source)
        self.assertIn("SYNTHETIC_SEQUENCE_COUNT = 25", source)
        self.assertIn("SYNTHETIC_REPEAT_COUNT = 3", source)
        self.assertIn("SYNTHETIC_PUBLISH_DELAY_MS = 20", source)
        self.assertIn("SYNTHETIC_DEADLINE_MS = 4500", source)
        self.assertIn("SYNTHETIC_STDOUT_DRAIN_MS = 2000", source)
        done_index = source.index(
            'print("V7_SYNTHETIC_MARKER_DONE status=%s" % _SYNTHETIC_STATUS)')
        drain_index = source.index(
            "time.sleep_ms(SYNTHETIC_STDOUT_DRAIN_MS)")
        self.assertGreater(drain_index, done_index)
        self.assertEqual(source.count("time.sleep_ms(SYNTHETIC_STDOUT_DRAIN_MS)"),
                         1)
        self.assertIn("_frame.clear()", source)
        self.assertNotIn("_frame.clear(96)", source)
        self.assertNotIn("sensor.snapshot", source)
        self.assertNotIn("import sensor", source)
        self.assertNotIn("UART(", source)
        self.assertNotIn("Pin(", source)
        self.assertNotIn("open(", source)

        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "synthetic.py"
            fixture.write_bytes(source.encode("utf-8"))
            with self.assertRaisesRegex(bench.BenchmarkError,
                                        "LIVE_SENSOR_UART_SINK"):
                bench.validate_ram_source(fixture)
            validated, digest = bench.validate_ram_source(
                fixture, synthetic_marker_probe=True)
        self.assertEqual(validated, source)
        self.assertEqual(len(digest), 64)

        unsafe_source = source.replace(
            " _frame.clear()", " sensor.snapshot()\n _frame.clear()")
        with tempfile.TemporaryDirectory() as directory:
            unsafe_fixture = Path(directory) / "unsafe-synthetic.py"
            unsafe_fixture.write_bytes(unsafe_source.encode("utf-8"))
            with self.assertRaisesRegex(bench.BenchmarkError,
                                        "sensor, hardware"):
                bench.validate_ram_source(
                    unsafe_fixture, synthetic_marker_probe=True)


if __name__ == "__main__":
    unittest.main()
