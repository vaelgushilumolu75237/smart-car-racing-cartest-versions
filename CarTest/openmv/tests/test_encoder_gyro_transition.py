from pathlib import Path
import sys
import unittest


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import gyro_link


class _Parser:
    profile_id = 2
    profile_flags = (gyro_link.PROFILE_SESSION_ACTIVE |
                     gyro_link.PROFILE_GYRO_SUPPORTED)
    session_token = 1
    profile_session_active = True

    @staticmethod
    def profile_is_fresh(now_ms, ticks_diff):
        return True

    @staticmethod
    def control_is_fresh(now_ms, ticks_diff):
        return True


def _ticks_diff(new_tick, old_tick):
    return int(new_tick) - int(old_tick)


def _inside(transition=False):
    return {"state": "INSIDE", "state_id": 3,
            "ring_confirmed": True, "side": gyro_link.TRACK_SIDE_LEFT,
            "entry_ready": True, "entry_evidence": True,
            "entry_transition": transition}


class EncoderGyroTransitionTests(unittest.TestCase):
    def test_distance_admission_waits_for_fresh_transition(self):
        parser = _Parser()
        link = gyro_link.GyroLink()
        self.assertEqual(
            link.update(parser, _inside(False), True, 0,
                        _ticks_diff)["phase"], gyro_link.PHASE_ENTRY)
        for now_ms in (1, 2, 3):
            self.assertEqual(
                link.update(parser, _inside(False), True, now_ms,
                            _ticks_diff)["phase"], gyro_link.PHASE_ENTRY)
        self.assertEqual(
            link.update(parser, _inside(True), True, 4,
                        _ticks_diff)["phase"], gyro_link.PHASE_ENTRY)
        for now_ms in (5, 6):
            self.assertEqual(
                link.update(parser, _inside(False), True, now_ms,
                            _ticks_diff)["phase"], gyro_link.PHASE_ENTRY)
        self.assertEqual(
            link.update(parser, _inside(False), True, 7,
                        _ticks_diff)["phase"], gyro_link.PHASE_ORBIT)


if __name__ == "__main__":
    unittest.main()
