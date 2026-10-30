# v26-r8 H750 ring topology candidate

Build: CARTEST_H750_RING_ENTRY_V26R8_20261030.
Fixes the pure ISLAND / ambiguous contract after confirmed ring entry with
frozen calibrated width. Entry, independent branch/multiple-exit rejection,
current route/corridor checks and profile pairing remain. Requires island
topology API 1; update all 23 Python files together only when authorized.

414 camera regression tests: zero failures/errors, three existing skips;
one unavailable private RAW replay explicitly excluded. All 23 files passed
MicroPython 1.28 armv7emsp cross-compilation. These are software checks.

H743 source/configuration, A/B/C ELFs, v24 PE7, ordinary steering/gains,
200ms slowdown and two-second loss hold are unchanged. No H743 flash required.
Keep the installed H750 native APP SHA256
c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154.
Preserve the same-camera calibration, which is excluded from public packages.

Native RAM retest, powered left/right road tests and independent 60+ FPS
including actual IDE preview remain pending. This is a candidate, not an
accepted road or FPS release. See docs/RING-ENTRY-V26-20261030.md.

Outer continuation API 2 preserves a virtual offscreen inferred boundary and
its original in-view geometric center. Explicit flag 128 and a same-frame
outer certificate are required. The entire visible corridor must be known
white. The observed edge, center, near/reference support, calibrated width,
continuity and two-second history remain checked. No center is clamped.
Independent branch scanning is retained even with an island. Native RAM,
powered road and board/actual-IDE 60+ FPS retests remain pending.
