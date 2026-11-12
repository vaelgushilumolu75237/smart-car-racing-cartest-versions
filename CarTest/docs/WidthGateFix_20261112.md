# Delay150 ring-only width gate and parked calibration bootstrap

Evidence: read-only ST-Link matched the installed C-original ELF. Camera D1 status reported FILE_NOT_FOUND (2), RING_CALIBRATION_REQUIRED (2), missing width ready/check flags, fresh profile C but outgoing profile ACK zero. UART error, CRC error and overflow counters were zero. Raw live dumps remain private.

Fix 1 (H743): allow cache-lifecycle KEEP during a verified parked, output-disabled, no-fault state with a fresh independent camera handshake. This breaks the cycle where missing width masks camera profile ACK, preventing KEEP, while the width learner itself needs KEEP. Startup release, local/manual stop, fault and freshness safeguards remain. KEEP grants neither motor permission nor a Follow session.

Fix 2 (camera): missing ring width must not invalidate a genuine validated ordinary bilateral/single-edge route or mask its profile ACK. Ring recognition/target authority retains width checks. An absent ordinary route remains invalid; no synthetic valid frame is created.

Fix 3 (camera): during the 150ms delay, confirmed-ring observation alone no longer suppresses the ordinary-route V4 flag. Once entrance authority is accepted, ordinary permission is withdrawn and the existing gyro ENTRY/ORBIT evidence rules apply.

Install H743/C_RingFast/CarTest.elf for the selected C-original profile. Camera uses the prior Delay150 files plus this package's updated OpenMV/main.py. If camera USB cannot connect, manually replace main.py on the active camera storage via a card reader; no wireless code uploader exists. Preserve real calibration files. No device upload, flash, reset or motor action was performed.

H743-only staging is also useful: after manual flash, keep the car stationary on a clear two-edge straight, issue CALWIDTH once through the existing wireless assistant to restart the old camera's width learner. This still uses the old camera global width gate until main.py is replaced. Width success requires valid measured image support and file write/readback; no guaranteed time or physical success is claimed.

Both software fixes should be installed for final behavior: original confirmed ring, ordinary following 150ms, gyro entry; ENTRY/ORBIT 75% wheel speed, relative servo 130%; actual entrance evidence before net 350deg. STARTCHECK remains available. Hardware acceptance and frame rate remain pending.
