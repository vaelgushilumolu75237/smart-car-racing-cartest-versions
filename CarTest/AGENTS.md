# CarTest version and restoration requirements

These project-specific requirements follow the user's 2026-10-15 instructions.

- Before changes, preserve the current source, configuration, paired ELFs and hashes.
- Every completed version must have a GitHub commit and immutable version tag in
  `vaelgushilumolu75237/smart-car-racing-cartest-versions`. The user authorized a
  public repository. Report the verified remote commit and version tag; if upload
  fails, state the failure instead of claiming a backup exists.
- Use `tools/checkpoint_version.py` to publish a complete H743/OpenMV checkpoint.
  Keep actual vehicle logs, user pictures, credentials, caches and temporary files
  outside public source checkpoints. This tool stages a separate worktree and must
  never overwrite the live source tree or force-push remote history.
- Retain the pre-ring `CACHED_WIDTH_TURNRESPONSE_20261014` camera package and its
  paired H743 ELF snapshots as a restoration baseline. A snapshot date/version is
  not proof that the user flashed that exact ELF or that it passes all road tests.
- The user reports regression after ring changes: straight travel shifts right,
  early turns and missed bends. Treat current ring route selection as experimental.
  Test ordinary turns separately from ring entry; compilation and synthetic-image
  tests do not establish vehicle behavior.
- Maintain only the production `openmv` package going forward. Do not update or
  regenerate `openmv_clipped_midpoint` unless the user explicitly reopens it.
- The lead model owns architecture and personal review. Luna execution agents own
  bounded implementation/testing/build work; do not add planning/review agents.
- Do not upload camera programs, flash/reset the H743, or start motors automatically.

Publishing example, after code review, tests and actual ELF builds are complete:

```powershell
python .\tools\checkpoint_version.py --version <unique-version> --branch experimental/pi-trial --status experimental-software-tested --publish
```
