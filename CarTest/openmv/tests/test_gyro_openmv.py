from pathlib import Path
import sys
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import gyro_link
import reverse_control


def _ticks_diff(new_tick, old_tick):
    return int(new_tick) - int(old_tick)


class GyroLinkTests(unittest.TestCase):
    def setUp(self):
        self.parser = reverse_control.ReverseControlParser(
            timeout_ms=250, profile_timeout_ms=250)
        self.link = gyro_link.GyroLink()
        self.profile_sequence = 0
        self._feed(reverse_control.pack_frame(0, 100), 0)
        self._feed(reverse_control.pack_frame(1, 101), 1)
        self._profile(2, reverse_control.PROFILE_SESSION_ACTIVE |
                      reverse_control.PROFILE_GYRO_SUPPORTED, 1, 2)

    def _feed(self, frame, now_ms):
        self.parser.feed(frame, len(frame), now_ms)

    def _profile(self, profile, flags, token, now_ms):
        frame = reverse_control.pack_profile_state(
            self.profile_sequence, profile, flags, session_token=token)
        self.profile_sequence += 1
        self._feed(frame, now_ms)

    def _update(self, info=None, valid=True, now_ms=3, ordinary=False):
        ring_info = {} if info is None else info
        return self.link.update(self.parser, ring_info, valid, now_ms,
                                _ticks_diff, ordinary_valid=ordinary)

    @staticmethod
    def _inside(side=gyro_link.TRACK_SIDE_LEFT, transition=False):
        return {"state": "INSIDE", "state_id": 3,
                "ring_confirmed": True, "side": side,
                "entry_transition": transition}

    def test_profile_b_c_cap_and_fresh_keep_are_required(self):
        self.assertEqual(self._update(self._inside())["phase"], 1)

        parser = reverse_control.ReverseControlParser(
            timeout_ms=250, profile_timeout_ms=250)
        parser.feed(reverse_control.pack_frame(0, 1), 8, 0)
        parser.feed(reverse_control.pack_frame(1, 2), 8, 1)
        parser.feed(reverse_control.pack_profile_state(
            0, reverse_control.PROFILE_A,
            reverse_control.PROFILE_SESSION_ACTIVE |
            reverse_control.PROFILE_GYRO_SUPPORTED,
            session_token=1), 20, 2)
        link = gyro_link.GyroLink()
        result = link.update(parser, self._inside(), True, 3,
                             _ticks_diff)
        self.assertEqual(result["phase"], 0)

        self._profile(2, reverse_control.PROFILE_SESSION_ACTIVE, 1, 4)
        self.assertEqual(self._update(self._inside(), now_ms=5)["phase"], 0)

        self._profile(2, reverse_control.PROFILE_SESSION_ACTIVE |
                      reverse_control.PROFILE_GYRO_SUPPORTED, 1, 6)
        self.assertEqual(self._update(self._inside(), now_ms=260)["phase"], 0)

    def test_enter_candidate_does_not_arm(self):
        candidate = {"state": "ENTER", "state_id": 2,
                     "ring_confirmed": False,
                     "side": gyro_link.TRACK_SIDE_LEFT}
        result = self._update(candidate)
        self.assertEqual(result, {"phase": 0,
                                  "side": gyro_link.SIDE_NONE,
                                  "ordinary": False})

    def test_invalid_frame_breaks_consecutive_inside_confirmation(self):
        inside = self._inside()
        self._update(inside, now_ms=3)
        self._update(inside, now_ms=4)
        self._update(inside, valid=False, now_ms=5)
        for now in (6, 7):
            self.assertEqual(self._update(inside, now_ms=now)["phase"], 1)
        self.assertEqual(self._update(inside, now_ms=8)["phase"], 2)

    def test_confirmed_inside_starts_entry_and_three_post_transition_frames_orbit(self):
        result = self._update(
            self._inside(gyro_link.TRACK_SIDE_RIGHT, True))
        self.assertEqual(result["phase"], 1)
        self.assertEqual(result["side"], gyro_link.SIDE_RIGHT)
        for index in range(2):
            result = self._update(
                self._inside(gyro_link.TRACK_SIDE_RIGHT, False),
                                  now_ms=4 + index)
            self.assertEqual(result["phase"], 1)
        result = self._update(
            self._inside(gyro_link.TRACK_SIDE_RIGHT, False),
                              now_ms=6)
        self.assertEqual(result["phase"], 2)
        self.assertEqual(result["side"], gyro_link.SIDE_RIGHT)

    def test_invalid_frame_keeps_phase_and_side_but_not_ordinary_validity(self):
        self._update(self._inside(transition=True))
        result = self._update(self._inside(), valid=False, ordinary=True,
                              now_ms=4)
        self.assertEqual(result["phase"], 1)
        self.assertEqual(result["side"], gyro_link.SIDE_LEFT)
        self.assertFalse(result["ordinary"])

    def test_release_suppresses_reentry_until_three_safe_ordinary_frames(self):
        self._update(self._inside(transition=True))
        release_flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                         reverse_control.PROFILE_GYRO_SUPPORTED |
                         reverse_control.PROFILE_GYRO_RELEASE)
        self._profile(2, release_flags, 1, 4)
        result = self._update(self._inside(), now_ms=5)
        self.assertEqual(result, {"phase": 0,
                                  "side": gyro_link.SIDE_NONE,
                                  "ordinary": False})
        for now_ms in (6, 7):
            result = self._update({}, now_ms=now_ms, ordinary=True)
            self.assertEqual(result["phase"], 0)
            self.assertTrue(result["ordinary"])
        result = self._update({}, now_ms=8, ordinary=True)
        self.assertTrue(result["ordinary"])
        result = self._update(self._inside(), now_ms=9)
        self.assertEqual(result["phase"], 1)

        self._profile(2, reverse_control.PROFILE_SESSION_ACTIVE |
                      reverse_control.PROFILE_GYRO_SUPPORTED, 1, 10)
        result = self._update(self._inside(), now_ms=11)
        self.assertEqual(result["phase"], 1)

    def test_session_exit_clears_release_suppression(self):
        self._update(self._inside(transition=True))
        release_flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                         reverse_control.PROFILE_GYRO_SUPPORTED |
                         reverse_control.PROFILE_GYRO_RELEASE)
        self._profile(2, release_flags, 1, 4)
        self._update(self._inside(), now_ms=5)
        self._profile(2, 0, 0, 6)
        result = self._update(self._inside(), now_ms=7)
        self.assertEqual(result["phase"], 0)
        self._profile(2, reverse_control.PROFILE_SESSION_ACTIVE |
                      reverse_control.PROFILE_GYRO_SUPPORTED, 2, 8)
        result = self._update(self._inside(), now_ms=9)
        self.assertEqual(result["phase"], 1)


if __name__ == "__main__":
    unittest.main()
