"""Static contract checks for the MicroPython 1.28 buffer binding."""

from pathlib import Path
import re
import unittest


TASK = Path(__file__).resolve().parents[1]
SOURCE = TASK / "binding" / "py_cartest_accel.c"
MICROPY_BINARY = (TASK.parents[3] / "tmp" /
                  "openmv-v5.0.1-h750-audit" / "lib" /
                  "micropython" / "py" / "binary.h")


class BindingContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.binary_header = MICROPY_BINARY.read_text(encoding="utf-8")

    def test_bytearray_uses_micro_python_named_typecode(self):
        self.assertIn('#include "py/binary.h"', self.source)
        self.assertNotRegex(self.source,
                            r"mp_type_bytearray\s*,\s*'B'")
        self.assertEqual(
            self.source.count("mp_type_bytearray, BYTEARRAY_TYPECODE"), 8)
        self.assertRegex(self.binary_header,
                         r"#define\s+BYTEARRAY_TYPECODE\s+1\b")

    def test_array_typecodes_remain_explicit(self):
        self.assertIn("&mp_type_array, 'i'", self.source)
        self.assertIn("queue.typecode != 'H'", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
