# v26-r6 H750 ring-entry candidate

Build: CARTEST_H750_RING_ENTRY_V26R6_20261030. This camera-only candidate corrects
mirrored ring-side expansion geometry and adds a bounded entry transition
from a current far inner-arc anchor to a current near outer-edge anchor.
Interpolated bridge rows are explicitly inferred; every transition corridor
cell must be known road. Calibration, parked checks, independent ordinary
route counter-evidence, topology, time limits and profile pairing remain.

Vehicle H743 A/B/C source, ELFs, v24 PE7, ordinary steering mapping, gains,
200 ms loss ramp and 2 s vision hold are unchanged. No H743 reflash is required.
Keep the already restored H750 native APP (SHA256
c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154).

403 camera regression tests: zero failures/errors, three existing skips; one
additional ground replay is excluded because its private RAW fixture is absent.
23/23 sources passed MicroPython armv7emsp compilation. Actual main-loop V3
output and host H743 parser/servo mapping were checked separately. These tests
do not establish powered ring traversal or v26 board/actual-IDE 60+ FPS.

The candidate remains hardware/road/FPS pending. When explicitly authorized,
install all 23 Python files and PACKAGE_MANIFEST.json together; compilation
MPYs are validation artifacts only. Preserve the same-camera calibration.
Private images, records, calibration and original camera backups are excluded.
See docs/RING-ENTRY-V26-20261030.md in the complete source checkpoint.

The r1 physical-camera RAM hand-push test confirmed ring entry and a valid
right-ring transition. A provisional exit now retains a newly checked inner
route while waiting for exit confirmation. Missing or blocked current edge
evidence still sends invalid frames; no output is reused without checking.
Powered ground traversal and dynamic/actual-IDE FPS are still pending.

Confirmed ring states now request a fresh raw boundary observation even when the ordinary route is valid. Accepted ordinary rows and their near support are retained separately for the original false-ring recovery and aligned exit checks. Stale or malformed raw data cannot manufacture a ring target. This candidate has not yet passed a board retest or powered traversal.

Ring observation now retains paired edges instead of applying the ordinary single-missing-side exclusion. Ordinary detect() behavior, contrast/identity/ambiguity/corridor checks and the control UART remain unchanged. Ring observation API 1 is required. This candidate is RAM/road/FPS pending and is not the permanently installed camera Python.

Inside-ring outer-edge continuation is available only after a normal inner-edge route was actually admitted by main. It uses current observed outer rows, frozen calibration, full known-white retained rows, near/reference support and a 12-x2 center handoff bound with at most 2000ms history. This is a camera candidate, not powered-road or board/IDE FPS acceptance.

The outer-handoff history limit is aligned to the existing 2-second H743 vision hold; no vehicle timeout or protection is modified. Software tests include qualified recovery at 1800ms and rejection beyond 2000ms. Native RAM retest and powered-road/FPS acceptance remain pending.
