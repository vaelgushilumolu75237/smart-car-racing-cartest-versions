"""Run the CarTest host-only test harness from the repository's PowerShell entry."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    runner = project_root / "tests" / "host" / "run_tests.ps1"
    if not runner.is_file():
        print(f"Host test runner is missing: {runner}", file=sys.stderr)
        return 2

    powershell = shutil.which("pwsh") or shutil.which("powershell")
    if powershell is None:
        print("PowerShell (pwsh or powershell) was not found on PATH.", file=sys.stderr)
        return 2

    result = subprocess.run(
        [powershell, "-NoLogo", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(runner)],
        cwd=project_root,
        check=False,
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
