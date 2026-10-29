"""Exercise the real shell canonicalizer at an MSYS drive root (no writes)."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / 'tools/vendor-patches/openmv-v5.0.1/build-openmv4p-ring45-native-clean.sh'
BASH = Path(os.environ.get('CARTEST_BUILD_BASH', 'D:/OMVEnv/tools/git/usr/bin/bash.exe'))


@unittest.skipUnless(BASH.is_file(), 'portable build bash unavailable')
class NativeBuildPathTests(unittest.TestCase):
    def canonicalize(self, path):
        source = RUNNER.read_text(encoding='utf8')
        start = source.index('canonical_future_path() {')
        end = source.index('\nsource_root=', start)
        script = 'set -e; export PATH=/usr/bin:/bin\n' + source[start:end]
        script += '\ncanonical_future_path "$1"\n'
        return subprocess.run([str(BASH), '--noprofile', '--norc', '-c', script, 'path-test', path],
                              capture_output=True, text=True, check=False)

    def test_drive_root_remains_drive_not_unc(self):
        result = self.canonicalize('/d')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '/d')

    def test_nested_future_output_keeps_parent(self):
        result = self.canonicalize('/d/OMVEnv/runner/work/nonexistent-regression-output')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '/d/OMVEnv/runner/work/nonexistent-regression-output')
        self.assertFalse(Path('D:/OMVEnv/runner/work/nonexistent-regression-output').exists())

    def test_dot_target_is_rejected(self):
        result = self.canonicalize('/d/.')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Expected a new directory path', result.stderr)


if __name__ == '__main__':
    unittest.main()
