"""Host tests for the independent 8-byte H743-to-OpenMV control stream."""

import csv
from pathlib import Path
import sys
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VECTOR_FILE = Path(__file__).with_name("data") / "reverse-control-v1.csv"
sys.path.insert(0, str(PACKAGE_DIR))

import reverse_control  # noqa: E402


class ReverseControlTests(unittest.TestCase):
    def setUp(self):
        self.parser = reverse_control.ReverseControlParser()

    def _feed(self, frame, now_ms):
        buffer = bytearray(frame)
        return self.parser.feed(buffer, len(buffer), now_ms)

    def test_vectors_match_exact_reset_keep_and_wrap_frames(self):
        commands = {
            "RESET": reverse_control.COMMAND_RESET,
            "KEEP": reverse_control.COMMAND_KEEP,
        }
        with VECTOR_FILE.open("r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 4)
        self.assertEqual(
            list(rows[0]), ["case", "sequence", "command", "frame_hex"])
        for row in rows:
            with self.subTest(case=row["case"]):
                frame = reverse_control.pack_frame(
                    commands[row["command"]], int(row["sequence"]))
                self.assertEqual(len(frame), reverse_control.FRAME_SIZE)
                self.assertEqual(bytes(frame).hex(), row["frame_hex"])

    def test_crc_check_value_and_little_endian_sequence(self):
        self.assertEqual(
            reverse_control.crc16_ccitt_false(b"123456789", 9), 0x29B1)
        frame = reverse_control.pack_frame(
            reverse_control.COMMAND_KEEP, 0x1234)
        self.assertEqual(bytes(frame[:6]), b"\xA6\x5A\x01\x01\x34\x12")
        crc = reverse_control.crc16_ccitt_false(frame, 6)
        self.assertEqual(frame[6] | frame[7] << 8, crc)

    def test_reset_arms_but_only_forward_keep_enables_single_edge(self):
        reset = reverse_control.pack_frame(0, 0xFFFE)
        keep_before_wrap = reverse_control.pack_frame(1, 0xFFFF)
        keep_after_wrap = reverse_control.pack_frame(1, 0)
        self._feed(reset, 0)
        self.assertFalse(self.parser.single_enabled)
        self._feed(keep_before_wrap, 50)
        self.assertTrue(self.parser.single_enabled)
        self._feed(keep_after_wrap, 100)
        self.assertTrue(self.parser.single_enabled)
        self.assertEqual(self.parser.accepted_keeps, 2)

    def test_fragmented_and_concatenated_frames_resynchronize(self):
        reset = reverse_control.pack_frame(0, 10)
        keep = reverse_control.pack_frame(1, 11)
        self.parser.feed(bytearray(b"garbage\xA6\xA6"), 9, 0)
        self.assertEqual(self.parser.buffered_bytes, 1)
        self.parser.feed(bytearray(reset[1:]), 7, 1)
        self.assertFalse(self.parser.single_enabled)
        combined = bytearray(keep + keep)
        self.parser.feed(combined, len(combined), 2)
        self.assertTrue(self.parser.single_enabled)
        self.assertEqual(self.parser.accepted_keeps, 1)
        self.assertEqual(self.parser.duplicate_keeps, 1)

    def test_duplicate_keep_does_not_renew_freshness(self):
        self._feed(reverse_control.pack_frame(0, 20), 0)
        keep = reverse_control.pack_frame(1, 21)
        self._feed(keep, 10)
        generation = self.parser.generation
        self._feed(keep, 200)
        self.assertEqual(self.parser.generation, generation)
        self.assertEqual(self.parser.last_event, "DUPLICATE_KEEP")
        self.assertFalse(self.parser.poll(259, lambda now, then: now - then))
        self.assertTrue(self.parser.poll(260, lambda now, then: now - then))
        self.assertFalse(self.parser.single_enabled)
        self.assertEqual(self.parser.last_event, "CONTROL_TIMEOUT")

    def test_bad_version_command_and_crc_clear_session(self):
        valid_reset = reverse_control.pack_frame(0, 1)
        valid_keep = reverse_control.pack_frame(1, 2)
        for reason, index, value in (
                ("BAD_VERSION", 2, 9),
                ("BAD_COMMAND", 3, 7),
                ("BAD_CRC", 6, None)):
            with self.subTest(reason=reason):
                self.parser = reverse_control.ReverseControlParser()
                self._feed(valid_reset, 0)
                self._feed(valid_keep, 1)
                self.assertTrue(self.parser.single_enabled)
                bad = bytearray(valid_keep)
                if value is None:
                    bad[index] ^= 1
                else:
                    bad[index] = value
                self._feed(bad, 2)
                self.assertFalse(self.parser.single_enabled)
                self.assertEqual(self.parser.last_event, reason)
                self._feed(reverse_control.pack_frame(1, 3), 3)
                self.assertFalse(self.parser.single_enabled)

    def test_rollback_requires_a_new_reset(self):
        self._feed(reverse_control.pack_frame(0, 100), 0)
        self._feed(reverse_control.pack_frame(1, 101), 10)
        self.assertTrue(self.parser.single_enabled)
        self._feed(reverse_control.pack_frame(1, 99), 20)
        self.assertFalse(self.parser.single_enabled)
        self.assertEqual(self.parser.last_event, "SEQUENCE_ROLLBACK")
        self._feed(reverse_control.pack_frame(1, 102), 30)
        self.assertFalse(self.parser.single_enabled)
        self._feed(reverse_control.pack_frame(0, 99), 40)
        self.assertFalse(self.parser.single_enabled)
        self._feed(reverse_control.pack_frame(1, 100), 50)
        self.assertTrue(self.parser.single_enabled)

    def test_ticks_wrap_and_feed_is_capped_at_32_bytes(self):
        period = 1 << 30
        diff = lambda now, then: ((now - then + period // 2) % period) - period // 2
        self._feed(reverse_control.pack_frame(0, 1), period - 40)
        self._feed(reverse_control.pack_frame(1, 2), period - 10)
        self.assertTrue(self.parser.poll(400, diff))
        self.assertFalse(self.parser.single_enabled)

        parser = reverse_control.ReverseControlParser()
        data = bytearray(b"\x00" * 33)
        self.assertEqual(parser.feed(data, len(data), 0), 32)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_pack_frame_rejects_out_of_range_inputs(self):
        for command, sequence in ((-1, 0), (2, 0), (True, 0), (0, -1),
                                  (0, 0x10000), (0, True)):
            with self.subTest(command=command, sequence=sequence):
                with self.assertRaises(ValueError):
                    reverse_control.pack_frame(command, sequence)


if __name__ == "__main__":
    result = unittest.TextTestRunner(stream=sys.stdout, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(ReverseControlTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
