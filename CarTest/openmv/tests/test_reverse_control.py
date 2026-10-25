"""Host tests for mixed V1 control and V2 profile-state frames."""

import csv
from pathlib import Path
import sys
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
VECTOR_FILE = Path(__file__).with_name("data") / "reverse-control-v1.csv"
PROFILE_VECTOR_FILE = (
    PACKAGE_DIR.parent / "tests" / "data" /
    "reverse-profile-state-v2.csv")
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

    def test_ticks_wrap_and_feed_is_capped_at_64_bytes(self):
        period = 1 << 30
        diff = lambda now, then: ((now - then + period // 2) % period) - period // 2
        self._feed(reverse_control.pack_frame(0, 1), period - 40)
        self._feed(reverse_control.pack_frame(1, 2), period - 10)
        self.assertTrue(self.parser.poll(400, diff))
        self.assertFalse(self.parser.single_enabled)

        parser = reverse_control.ReverseControlParser()
        data = bytearray(b"\x00" * (reverse_control.MAX_FEED_BYTES + 1))
        self.assertEqual(
            parser.feed(data, len(data), 0), reverse_control.MAX_FEED_BYTES)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_profile_state_layout_crc_and_unsigned_fields(self):
        frame = reverse_control.pack_profile_state(
            0x1234, reverse_control.PROFILE_C,
            reverse_control.PROFILE_SESSION_ACTIVE,
            reverse_control.PROFILE_REVISION, 0x89ABCDEF, 0x10203040)
        self.assertEqual(len(frame), reverse_control.PROFILE_FRAME_SIZE)
        self.assertEqual(
            bytes(frame[:10]), b"\xA6\x5A\x02\x02\x34\x12\x03\x01\x01\x00")
        self.assertEqual(bytes(frame[10:14]), b"\xEF\xCD\xAB\x89")
        self.assertEqual(bytes(frame[14:18]), b"\x40\x30\x20\x10")
        self.assertEqual(frame[18] | frame[19] << 8,
                         reverse_control.crc16_ccitt_false(frame, 18))

    def test_shared_profile_state_vectors_match_exact_bytes(self):
        with PROFILE_VECTOR_FILE.open(
                "r", newline="", encoding="ascii") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 4)
        self.assertEqual(
            list(rows[0]), ["case", "sequence", "profile_id", "active",
                            "revision", "session_token",
                            "mean_forward_wheel_millirev", "hex"])
        for row in rows:
            with self.subTest(case=row["case"]):
                flags = (reverse_control.PROFILE_SESSION_ACTIVE
                         if int(row["active"]) else 0)
                frame = reverse_control.pack_profile_state(
                    int(row["sequence"]), int(row["profile_id"]), flags,
                    int(row["revision"]), int(row["session_token"]),
                    int(row["mean_forward_wheel_millirev"]))
                self.assertEqual(bytes(frame).hex(), row["hex"].lower())

    def test_profile_sequence_and_freshness_are_independent_of_keep(self):
        self.parser = reverse_control.ReverseControlParser(timeout_ms=1000)
        self._feed(reverse_control.pack_frame(0, 100), 0)
        self._feed(reverse_control.pack_frame(1, 101), 5)
        self.assertTrue(self.parser.single_enabled)
        control_generation = self.parser.generation
        profile = reverse_control.pack_profile_state(
            0xFFFE, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE, session_token=9,
            mean_forward_wheel_millirev=54321)
        self._feed(profile, 10)
        self.assertTrue(self.parser.single_enabled)
        self.assertEqual(self.parser.generation, control_generation)
        self.assertEqual(self.parser.profile_ack(10, lambda n, o: n - o), 2)
        self.assertTrue(self.parser.ring_profile_enabled(
            10, lambda n, o: n - o))
        self.assertEqual(self.parser.session_token, 9)
        self.assertEqual(self.parser.mean_forward_wheel_millirev, 54321)

        next_profile = reverse_control.pack_profile_state(
            0xFFFF, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE, session_token=9)
        self._feed(next_profile, 100)
        self.assertTrue(self.parser.single_enabled)
        self.assertEqual(self.parser.profile_generation, 0)
        self.assertEqual(self.parser.profile_ack(349, lambda n, o: n - o), 2)
        self.assertEqual(self.parser.profile_ack(350, lambda n, o: n - o), 0)
        self.assertTrue(self.parser.poll(350, lambda n, o: n - o))
        self.assertEqual(self.parser.last_profile_event, "PROFILE_TIMEOUT")
        self.assertFalse(self.parser.ring_profile_enabled(
            350, lambda n, o: n - o))
        self.assertEqual(self.parser.generation, control_generation)
        self.assertEqual(self.parser.profile_generation, 1)

    def test_duplicate_profile_heartbeat_does_not_refresh_or_clear(self):
        parser = reverse_control.ReverseControlParser()
        active = reverse_control.PROFILE_SESSION_ACTIVE
        frame = reverse_control.pack_profile_state(
            10, reverse_control.PROFILE_C, active, session_token=2)
        parser.feed(frame, len(frame), 0)
        profile_generation = parser.profile_generation
        parser.feed(frame, len(frame), 200)
        self.assertEqual(parser.duplicate_profile_states, 1)
        self.assertEqual(parser.profile_generation, profile_generation)
        self.assertEqual(parser.profile_ack(249, lambda n, o: n - o), 3)
        self.assertEqual(parser.profile_ack(250, lambda n, o: n - o), 0)

    def test_profile_switch_and_session_token_change_bump_once(self):
        parser = reverse_control.ReverseControlParser()
        active = reverse_control.PROFILE_SESSION_ACTIVE
        for sequence in range(4):
            profile = reverse_control.PROFILE_B
            token = 7
            if sequence >= 2:
                profile = reverse_control.PROFILE_C
            if sequence >= 3:
                token = 8
            frame = reverse_control.pack_profile_state(
                sequence, profile, active, session_token=token)
            parser.feed(frame, len(frame), sequence * 50)
            expected = 0 if sequence < 2 else sequence - 1
            self.assertEqual(parser.profile_generation, expected)
        self.assertEqual(parser.last_profile_event, "SESSION_CHANGE")

    def test_profile_validation_rejects_bad_crc_rollback_and_flags_without_resetting_v1(self):
        parser = reverse_control.ReverseControlParser()
        parser.feed(reverse_control.pack_frame(0, 1), 8, 0)
        parser.feed(reverse_control.pack_frame(1, 2), 8, 1)
        self.assertTrue(parser.single_enabled)
        valid = reverse_control.pack_profile_state(
            10, reverse_control.PROFILE_A)
        parser.feed(valid, len(valid), 2)
        profile_generation = parser.profile_generation

        bad_crc = bytearray(reverse_control.pack_profile_state(
            11, reverse_control.PROFILE_C))
        bad_crc[18] ^= 1
        parser.feed(bad_crc, len(bad_crc), 3)
        self.assertEqual(parser.last_profile_event, "BAD_PROFILE_CRC")
        self.assertTrue(parser.single_enabled)
        self.assertEqual(parser.profile_generation, profile_generation)

        rollback = reverse_control.pack_profile_state(
            9, reverse_control.PROFILE_C)
        parser.feed(rollback, len(rollback), 4)
        self.assertEqual(parser.last_profile_event,
                         "PROFILE_SEQUENCE_ROLLBACK")
        self.assertEqual(parser.profile_ack(4, lambda n, o: n - o), 1)
        self.assertTrue(parser.single_enabled)

        bad_flags = bytearray(reverse_control.pack_profile_state(
            11, reverse_control.PROFILE_C, 0x80))
        parser.feed(bad_flags, len(bad_flags), 5)
        self.assertEqual(parser.last_profile_event, "BAD_PROFILE_FLAGS")
        self.assertTrue(parser.single_enabled)

    def test_unknown_profile_revision_and_inactive_session_fail_closed(self):
        parser = reverse_control.ReverseControlParser()
        unknown = reverse_control.pack_profile_state(1, 99, 1, 1, 7)
        parser.feed(unknown, len(unknown), 0)
        self.assertEqual(parser.profile_ack(0, lambda n, o: n - o), 0)
        self.assertFalse(parser.ring_profile_enabled(
            0, lambda n, o: n - o))

        unsupported = reverse_control.pack_profile_state(2, 2, 1, 2, 7)
        parser.feed(unsupported, len(unsupported), 100)
        self.assertEqual(parser.profile_ack(100, lambda n, o: n - o), 0)
        self.assertFalse(parser.ring_profile_enabled(
            100, lambda n, o: n - o))

        inactive = reverse_control.pack_profile_state(3, 2, 0, 1, 7)
        parser.feed(inactive, len(inactive), 200)
        self.assertEqual(parser.profile_ack(200, lambda n, o: n - o), 2)
        self.assertFalse(parser.ring_profile_enabled(
            200, lambda n, o: n - o))

    def test_mixed_fragmented_and_concatenated_frames_resynchronize(self):
        parser = reverse_control.ReverseControlParser()
        reset = reverse_control.pack_frame(0, 10)
        status = reverse_control.pack_profile_state(
            100, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE, session_token=5)
        keep = reverse_control.pack_frame(1, 11)
        combined = bytearray(reset + status + keep)
        parser.feed(combined, 13, 0)
        self.assertFalse(parser.single_enabled)
        parser.feed(combined[13:], len(combined) - 13, 1)
        self.assertTrue(parser.single_enabled)
        self.assertEqual(parser.accepted_resets, 1)
        self.assertEqual(parser.accepted_profile_states, 1)
        self.assertEqual(parser.accepted_keeps, 1)
        self.assertEqual(parser.profile_ack(1, lambda n, o: n - o), 2)

    def test_profile_partial_timeout_preserves_fresh_v1_keep(self):
        parser = reverse_control.ReverseControlParser(
            timeout_ms=1000, profile_timeout_ms=250)
        parser.feed(reverse_control.pack_frame(0, 1), 8, 0)
        parser.feed(reverse_control.pack_frame(1, 2), 8, 10)
        profile = reverse_control.pack_profile_state(
            1, reverse_control.PROFILE_B,
            reverse_control.PROFILE_SESSION_ACTIVE, session_token=1)
        parser.feed(profile, 10, 20)
        self.assertEqual(parser.buffered_bytes, 10)
        self.assertTrue(parser.single_enabled)
        self.assertFalse(parser.poll(271, lambda n, o: n - o))
        self.assertEqual(parser.last_profile_event,
                         "PROFILE_PARTIAL_TIMEOUT")
        self.assertTrue(parser.single_enabled)

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
