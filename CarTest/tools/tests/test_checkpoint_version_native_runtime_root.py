from __future__ import annotations

import contextlib
import io
import sys
import unittest
from unittest.mock import patch

from tools import checkpoint_version


class CheckpointNativeRuntimeRootTests(unittest.TestCase):
    def test_frozen_runtime_root_requires_native_bundle_option(self):
        stderr = io.StringIO()
        argv = [
            "checkpoint_version.py",
            "--version", "fixture-v1",
            "--branch", "fixture/branch",
            "--status", "experimental-offline",
            "--openmv-native-v501-runtime-root",
            "deliverables/TrackProfilesFixture_v16/OpenMV",
        ]
        with patch.object(sys, "argv", argv), \
                contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                checkpoint_version.main()
        self.assertEqual(raised.exception.code, 2)
        self.assertIn(
            "--openmv-native-v501-runtime-root requires --openmv-native-v501",
            stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
