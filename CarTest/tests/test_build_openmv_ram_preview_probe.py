import ast
from pathlib import Path
import tempfile
import unittest
import zlib

from tools import benchmark_openmv_preview as bench
from tools import build_openmv_ram_preview_probe as builder


class BuildOpenMVRamPreviewProbeTests(unittest.TestCase):
    def test_probe_compiles_embeds_all_production_modules_and_passes_guard(self):
        source = builder.build_probe_source()
        compile(source, "fullres-q30-ram-source.py", "exec")
        self.assertLess(len(source.encode("utf-8")),
                        bench.MAX_RAM_SOURCE_BYTES)
        self.assertIn("PREVIEW_CONTROL_API", source)
        self.assertIn("quality=PREVIEW_QUALITY,copy=True", source)
        self.assertIn("x_scale=1.0,y_scale=1.0", source)
        self.assertIn("sensor.snapshot(preview=False)", source)
        self.assertIn("marker=D3/seq16/crc8", source)

        tree = ast.parse(source)
        try_suite = next(node for node in tree.body if isinstance(node, ast.Try))
        compressed_modules = {}
        for statement in try_suite.body:
            if not isinstance(statement, ast.Assign):
                continue
            target = statement.targets[0]
            if (not isinstance(target, ast.Subscript) or
                    not isinstance(target.value, ast.Name) or
                    target.value.id != "_ram_sources"):
                continue
            name = ast.literal_eval(target.slice)
            compressed = ast.literal_eval(statement.value)
            compressed_modules[name] = zlib.decompress(compressed)

        self.assertEqual(set(compressed_modules), {
            name + ".py" for name in builder.RUNTIME_MODULES
        })
        for name in builder.RUNTIME_MODULES:
            with self.subTest(module=name):
                self.assertEqual(
                    compressed_modules[name + ".py"],
                    (builder.OPENMV_ROOT / (name + ".py")).read_bytes())

        with tempfile.TemporaryDirectory() as directory:
            ram_source = Path(directory) / "probe.py"
            ram_source.write_bytes(source.encode("utf-8"))
            validated, digest = bench.validate_ram_source(ram_source)
            self.assertEqual(validated, source)
            self.assertEqual(len(digest), 64)


if __name__ == "__main__":
    unittest.main()
