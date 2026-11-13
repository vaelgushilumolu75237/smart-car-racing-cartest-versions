import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from test_ring_width_calibration import straight_rows
from ring_width_calibration import AutoBootCalibration, RingWidthCalibration


class AutoBootTests(unittest.TestCase):
    def test_all_retained_profiles_collect_without_enabling_ring_for_a(self):
        import config
        self.assertEqual(config.AUTO_BOOT_WIDTH_CALIBRATION_PROFILE_IDS, (1, 2, 3))
        self.assertNotIn(config.VEHICLE_PROFILE_A, config.RING_ENABLED_PROFILE_IDS)
        for profile in config.AUTO_BOOT_WIDTH_CALIBRATION_PROFILE_IDS:
            self.controller = AutoBootCalibration(RingWidthCalibration(), self.path)
            loaded = self.feed(identity=(profile, 0, 0))
            self.assertTrue(loaded.ready)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = str(Path(self.tmp.name) / "width.json")
        self.rows = straight_rows()
        self.controller = AutoBootCalibration(RingWidthCalibration(), self.path)

    def feed(self, start=0, count=75, eligible=True, identity=(3, 0, 0)):
        result = None
        for i in range(start, start + count):
            result = self.controller.step(i * 20, lambda a, b: a-b,
                eligible, identity, self.rows, len(self.rows)//4, 4) or result
        return result

    def test_missing_map_collects_saves_and_reloads_crc(self):
        loaded = self.feed()
        self.assertTrue(loaded.ready)
        self.assertGreaterEqual(loaded.accepted_frames, 64)
        self.assertTrue(RingWidthCalibration.load(self.path).ready)
        self.assertIsNone(self.feed(start=100))

    def test_no_parked_gate_never_writes(self):
        self.assertIsNone(self.feed(eligible=False))
        self.assertFalse(Path(self.path).exists())
        self.assertIsNone(self.controller.started_ms)

    def test_interrupted_or_changed_identity_discards_partial_map(self):
        self.feed(count=40)
        self.feed(start=40, count=1, eligible=False)
        self.assertIsNone(self.feed(start=41, count=40))
        self.assertIsNone(self.feed(start=81, count=40, identity=(3, 1, 2)))
        self.assertFalse(Path(self.path).exists())

    def test_timeout_and_write_failure_keep_gate_closed(self):
        self.feed(count=2)
        self.assertIsNone(self.controller.step(60000, lambda a,b:a-b,
            True, (3,0,0), self.rows, len(self.rows)//4, 4))
        self.assertEqual(self.controller.reason, "TIMEOUT")
        self.controller = AutoBootCalibration(RingWidthCalibration(), self.path)
        with patch.object(RingWidthCalibration, "save", side_effect=OSError("read only")):
            self.assertIsNone(self.feed())
        self.assertEqual(self.controller.reason, "SAVE_FAILED")
        self.assertFalse(Path(self.path).exists())

    def test_independent_collection_survives_long_wait_without_vehicle_identity(self):
        self.controller = AutoBootCalibration(RingWidthCalibration(), self.path,
                                              timeout_ms=None)
        for now in (0, 60000, 120000):
            self.controller.step(now, lambda a,b:a-b, True,
                                 (320,120), None, 0, 0)
        self.assertFalse(self.controller.done)
        loaded = self.feed(start=7000, identity=(320,120))
        self.assertTrue(loaded.ready)

    def test_main_calibration_has_no_vehicle_gate(self):
        source = (Path(__file__).resolve().parents[1] / "main.py").read_text(encoding="utf-8")
        block = source.split("        if not ring_calibration_passed:\n",1)[1].split("        single_rows_fresh",1)[0]
        for forbidden in ("reverse_fresh", "parked_profile", "profile_token",
                          "normal_cache_commit_allowed", "ring_stationary_status_frames"):
            self.assertNotIn(forbidden, block)
        self.assertIn("time.ticks_diff, True", block)

    def test_existing_map_not_overwritten(self):
        self.feed()
        before = Path(self.path).read_bytes()
        self.controller = AutoBootCalibration(RingWidthCalibration.load(self.path), self.path)
        self.assertIsNone(self.feed())
        self.assertEqual(Path(self.path).read_bytes(), before)

    def test_loaded_map_still_requires_original_tracker_parked_check(self):
        import config
        from track_elements import TrackElementsTracker
        loaded = self.feed()
        tracker = TrackElementsTracker(config, None, require_calibration=True)
        tracker.set_ring_calibration(loaded, True)
        self.assertTrue(tracker.ring_calibration_ready)
        self.assertFalse(tracker.ring_calibration_check_ready)
        for _ in range(4):
            tracker.observe_ring_calibration(self.rows, len(self.rows)//4, 4)
        self.assertTrue(tracker.ring_calibration_check_ready)

    def test_readback_failure_cannot_authorize_route(self):
        invalid = RingWidthCalibration()
        with patch.object(RingWidthCalibration, "load", return_value=invalid):
            self.assertIsNone(self.feed())
        self.assertEqual(self.controller.reason, "SAVE_FAILED")

    def test_curved_samples_cannot_create_map(self):
        self.rows = straight_rows(curve=True)
        self.assertIsNone(self.feed())
        self.assertFalse(Path(self.path).exists())


if __name__ == "__main__":
    unittest.main()
