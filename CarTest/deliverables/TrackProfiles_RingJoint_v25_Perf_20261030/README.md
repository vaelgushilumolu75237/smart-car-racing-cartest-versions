# v25 corridor Viper performance candidate

The 23 OpenMV sources preserve v25 ring admission/recovery, route geometry,
UART protocol and profile checks. The calibrated ordinary-corridor boolean
scan uses a bounded typed Viper kernel; list/non-i32 adapters retain the old
object-safe path. Native buffers are validated once by owner/length, including
MicroPython ports without array.typecode. No threshold or steering sign change.

Vehicle H743 profiles/ELFs and v24 PE7 are unchanged. The paired native camera
APP is OpenMV_RingJoint_v25_NativeCandidate_20261029, firmware.bin SHA256
c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154.
The current sample's APP update/readback was verified separately; bootloader,
internal filesystem and complete 32MiB QSPI contents were unchanged. Private
partition backups, calibration, source/video and run logs are excluded here.

These sources remain a candidate. The stationary calibrated-straight RAM test
measured 60.8?62.2 displayed new frames/s in ten consecutive 10-second actual
IDE windows (6,160 new images, zero marker CRC failures). Preview-off board
windows measured 77.0?77.6 FPS. The isolated performance probe uses virtual
profile status, not physical vehicle UART. The matching board log confirms 62.9?63.6 processing FPS with IDE open
(seventeen complete active windows). Separate parked real-UART tests received
V3/ACK3/ready=1; full RESET/KEEP startup, physical active-session pairing,
dynamic-scene FPS and full road/left/right ring acceptance remain pending. See docs/OPENMV-V25-PERFORMANCE-20261030.md in the
source checkpoint. Host/SDK/actual IDE are distinct.
Keep the same-camera calibration; do not replace it with another camera's map.
Upload all 23 Python sources as one matching set only when authorized; MPYs
under validation are compilation artifacts, not upload files. H743 need not
be flashed again for this camera-only performance change.
