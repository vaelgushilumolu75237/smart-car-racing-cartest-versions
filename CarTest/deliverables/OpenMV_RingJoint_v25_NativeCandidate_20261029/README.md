# v25 native APP software-build candidate

Fresh build using the restored offline environment, fixed mapped-root handling,
and v25's exact 23-module runtime manifest. No old objects were reused. This is
an APP-only OPENMV4P layout build for the user H750 camera compatibility path;
the build target retains the historical STM32H743xx label. Vehicle H743 firmware
and PE7 are unchanged. Do not confuse the two controllers.

No camera was uploaded, flashed, reset or started by this build. The replacement
camera layout, native ABI at runtime, FPS, actual IDE preview and road behavior
are pending. This package does not authorize deployment. Keep v25 Python; the
v16 Python reference was not substituted. Runtime sources are paired separately
at deliverables/TrackProfiles_RingJoint_v25_20261029/OpenMV.

Application range: 0x08040000..0x08200000 (end exclusive). Bootloader and ROMFS
are not included. The rollback/ pair is a pinned HISTORICAL current-j reference,
not a backup of the replacement camera. Back up that camera's current partitions
and private calibration before any separately authorized device update.

build-verification.json retains facts from the successful runner, with three
field-name/target-label aliases documented in normalization_notes and the raw
runner JSON hash retained. No historical flash/readback/bench evidence is claimed.
Local complete build and console logs remain in D:/OMVEnv/runner/artifacts;
they are not vehicle tests. See docs/OPENMV-NATIVE-V25-BUILD-REPAIR-20261029.md.
