"""A wrong feature flag or C speed must never label a paired deliverable."""
import tempfile
from pathlib import Path
import unittest

from tools.build_openmv_gyro_wireless import PROFILES, expected_profile, verify_cache, public_delivery_files


class GyroWirelessDeliveryTests(unittest.TestCase):
    def test_six_profiles_keep_independent_gyro_speeds(self):
        rows = [expected_profile(row) for row in PROFILES[:6]]
        self.assertEqual(len({(r["profile_id"], r["speed_percent"]) for r in rows}), 6)
        self.assertEqual([r["ring_outer_target_millirps"] for r in rows], [0, 1500, 2800, 2376, 2574, 2772])
        self.assertEqual([r["ring_rate_mdps"] for r in rows], [0, 64935, 100000, 102857, 111429, 120000])
        self.assertEqual([r["ring_exit_outer_target_millirps"] for r in rows], [0, 4000, 5280, 6336, 6864, 7392])
        self.assertEqual([r["ring_entry_straight_ms"] for r in rows], [0, 1100, 1100, 1100, 1100, 1100])
        self.assertEqual([r["ring_turn_servo_percent"] for r in rows], [100, 187.5, 187.5, 187.5, 187.5, 187.5])
        self.assertFalse(rows[0]["gyro_authority"])
        self.assertTrue(all(row["gyro_authority"] for row in rows[1:]))
        variant = expected_profile(PROFILES[6])
        base = rows[2]
        self.assertEqual(variant["preview_amplitude_percent"], 95)
        self.assertEqual(variant["curve_outer_target_millirps"], 4840)
        for name in ("straight_target_millirps", "ring_outer_target_millirps",
                     "ring_rate_mdps", "ring_exit_outer_target_millirps",
                     "ring_entry_straight_ms", "ring_exit_progress_mdeg",
                     "ring_turn_servo_percent"):
            self.assertEqual(variant[name], base[name])

    def test_cache_requires_both_feature_flags_and_exact_speed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            row = PROFILES[5]
            cache = root / "build" / row[1] / "CMakeCache.txt"
            cache.parent.mkdir(parents=True)
            values = dict(CARTEST_TRACK_PROFILE="3", CARTEST_SPEED_PERCENT="140",
                          CARTEST_ENABLE_GYRO350="ON", CARTEST_ENABLE_WIRELESS="ON",
                          CARTEST_ENABLE_RED_OBSTACLE="ON",
                          CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI="ON", CARTEST_ENABLE_ACKERMANN_COORDINATION="ON",
                          CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS="5600", CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS="4400",
                          CARTEST_PREVIEW95_BEND110="OFF")
            def write(data):
                cache.write_text("\n".join(k + ":STRING=" + v for k, v in data.items()), encoding="utf-8")
            write(values)
            verify_cache(root, row)
            for key, wrong in (("CARTEST_SPEED_PERCENT", "100"), ("CARTEST_ENABLE_GYRO350", "OFF"),
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
