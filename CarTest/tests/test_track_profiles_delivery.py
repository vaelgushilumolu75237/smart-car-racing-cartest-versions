"""Delivery checks prevent a mislabeled ELF or altered upload directory."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.build_track_profiles import verify_build_cache, verify_manifest


class DeliveryTests(unittest.TestCase):
    def test_changed_and_extra_payloads_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            payload = root / "OpenMV" / "main.py"
            payload.parent.mkdir()
            payload.write_bytes(b"original")
            manifest = {"OpenMV/main.py": hashlib.sha256(b"original").hexdigest()}
            (root / "SHA256SUMS.json").write_text(json.dumps(manifest))
            self.assertEqual(verify_manifest(root), manifest)
            payload.write_bytes(b"changed")
            with self.assertRaises(ValueError):
                verify_manifest(root)
            payload.write_bytes(b"original")
            (root / "unreviewed.py").write_bytes(b"extra")
            with self.assertRaises(ValueError):
                verify_manifest(root)

    def test_cache_rejects_an_elf_built_for_another_profile(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            cache = root / "build" / "DebugFastNoRing" / "CMakeCache.txt"
            cache.parent.mkdir(parents=True)
            cache.write_text(
                "CARTEST_TRACK_PROFILE:STRING=1\n"
                "CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS:STRING=5600\n"
                "CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS:STRING=4400\n"
                "CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI:BOOL=ON\n"
                "CARTEST_ENABLE_ACKERMANN_COORDINATION:BOOL=ON\n")
            verify_build_cache(root, "DebugFastNoRing", 1, 5600, 4400)
            with self.assertRaises(ValueError):
                verify_build_cache(root, "DebugFastNoRing", 2, 4800, 4000)


if __name__ == "__main__":
    unittest.main()
