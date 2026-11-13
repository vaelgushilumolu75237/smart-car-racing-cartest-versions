"""A wrong feature flag or C speed must never label a paired deliverable."""
import tempfile
from pathlib import Path
import unittest

from tools.build_openmv_gyro_wireless import PROFILES, expected_profile, verify_cache, public_delivery_files


class GyroWirelessDeliveryTests(unittest.TestCase):
    def test_three_profiles_change_only_ring_parameters(self):
        rows = [expected_profile(row) for row in PROFILES]
        self.assertEqual([r["ring_outer_target_millirps"] for r in rows], [2800,4200,7600])
        self.assertEqual([r["ring_rate_mdps"] for r in rows], [100000,145000,282000])
        for row in rows:
            self.assertEqual(row["straight_target_millirps"], 7600 if row["preset"] == "DebugRingFastLimit2" else 5600)
            self.assertEqual(row["curve_outer_target_millirps"], 4400)
            self.assertEqual(row["ring_exit_outer_target_millirps"], 5280)
            self.assertEqual(row["ring_entry_straight_ms"], {"DebugRingFast":1100,"DebugRingFastLimit":800,"DebugRingFastLimit2":700}[row["preset"]])
            self.assertEqual(row["ring_exit_progress_mdeg"], 280000)
            self.assertEqual(row["ring_turn_servo_percent"], 187.5)

    def test_cache_requires_both_feature_flags_and_exact_speed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            row = PROFILES[0]
            cache = root / "build" / row[1] / "CMakeCache.txt"
            cache.parent.mkdir(parents=True)
            values = dict(CARTEST_TRACK_PROFILE="3", CARTEST_SPEED_PERCENT="100", CARTEST_RING_TUNE="0",
                          CARTEST_ENABLE_GYRO350="ON", CARTEST_ENABLE_WIRELESS="ON",
                          CARTEST_ENABLE_RED_OBSTACLE="ON",
                          CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI="ON", CARTEST_ENABLE_ACKERMANN_COORDINATION="ON",
                          CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS="5600", CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS="4400")
            def write(data):
                cache.write_text("\n".join(k + ":STRING=" + v for k, v in data.items()), encoding="utf-8")
            write(values)
            verify_cache(root, row)
            for key, wrong in (("CARTEST_SPEED_PERCENT", "140"), ("CARTEST_ENABLE_GYRO350", "OFF"),
                               ("CARTEST_ENABLE_WIRELESS", "OFF"), ("CARTEST_ENABLE_RED_OBSTACLE", "OFF"), ("CARTEST_TRACK_PROFILE", "2")):
                with self.subTest(key=key):
                    write(dict(values, **{key: wrong}))
                    with self.assertRaises(ValueError):
                        verify_cache(root, row)

    def test_publication_rejects_path_escape(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                public_delivery_files(Path(temp), "../unreviewed")


if __name__ == "__main__":
    unittest.main()
