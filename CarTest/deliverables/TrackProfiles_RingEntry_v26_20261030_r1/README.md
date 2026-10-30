# v26 H750 ring-entry candidate

Build: CARTEST_H750_RING_ENTRY_V26_20261030. This camera-only candidate corrects
mirrored ring-side expansion geometry and adds a bounded entry transition
from a current far inner-arc anchor to a current near outer-edge anchor.
Interpolated bridge rows are explicitly inferred; every transition corridor
cell must be known road. Calibration, parked checks, independent ordinary
route counter-evidence, topology, time limits and profile pairing remain.

Vehicle H743 A/B/C source, ELFs, v24 PE7, ordinary steering mapping, gains,
200 ms loss ramp and 2 s vision hold are unchanged. No H743 reflash is required.
Keep the already restored H750 native APP (SHA256
c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154).

381 camera regression tests: zero failures/errors, three existing skips; one
additional ground replay is excluded because its private RAW fixture is absent.
23/23 sources passed MicroPython armv7emsp compilation. Actual main-loop V3
output and host H743 parser/servo mapping were checked separately. These tests
do not establish powered ring traversal or v26 board/actual-IDE 60+ FPS.

The candidate remains hardware/road/FPS pending. When explicitly authorized,
install all 23 Python files and PACKAGE_MANIFEST.json together; compilation
MPYs are validation artifacts only. Preserve the same-camera calibration.
Private images, records, calibration and original camera backups are excluded.
See docs/RING-ENTRY-V26-20261030.md in the complete source checkpoint.
