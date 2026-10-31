# r20-derived r22 candidate: ring speed and immediate right exit

Baseline: v20261031-ring-exit-v26-r20, commit 3cf91729c2670d3e721e9344a68bea3f9b464110.
The r20 source/ELFs remain preserved in the original full-source package and a pre-edit SHA256 manifest.
This candidate does not include r21's EXIT_RING_HOLD fallback or v24/v25 speed scaling.

C steady INSIDE requests slowdown 267/1000. The unchanged H743 Ackermann function
computes 5600-round(1200*267/1000)=5280 milli-RPS, 20% over 4400.
Straight remains 5600; entry transition and exit retain the full slowdown request.
C vision-loss last-error hold becomes 3000ms and hold-output deceleration 300ms.
A/B and legacy hold profiles remain 2000/200ms. The normal Ackermann slowdown
request ramp is independent and remains unchanged.
C CMake and Src/main.c assertions both require the new 3000/300ms configuration.
All pairing, hard faults, motor gates, PI gains, steering maps and original PE7 remain.

Only when already in right-ring EXIT_CONFIRM/EXIT_REPAIR/REACQUIRE can the first
fresh spatially qualified left edge replace the frozen handoff anchor. Known
white corridor, calibrated width, current raw observed rows, near/reference
support, straight-exit shape, unambiguous topology, valid ring identity/session
and a last actually transmitted route no older than 3000ms remain required.
Ordinary accepted bilateral routes keep their existing path. INSIDE admission
and left-ring single exit retain three fresh frames. Targets are rebuilt from
the current left edge and per-row calibrated width; steering follows the resulting
weighted center error, never a forced zero or a fixed-sign turn.
Missing/ambiguous/unknown/branch/wrong-width/curved lines cannot get a certificate.
Invalid output cannot renew the transmitted-route timestamp or extend the hold.

Verification: 509 camera tests (4 skipped), 23 runtime cross-compiles; 15 ELF
builds; 72 H743 cases / 7713 assertions; exact 2999/3000ms hold boundary and
299/300ms deceleration; actual C request=267 -> target=5280; 11 actual camera
UART packets parsed through the extracted production H743 steering mapping.
The first-frame certificate is also covered by the actual main UART converter.
Saved actual exit images 1737/1738, formerly HANDOFF_UNSUPPORTED, admit 52/51
current target rows in isolated geometry reconstruction. This assumes the recent
sent-route history from adjacent captured frames and is not full temporal replay.
No camera upload, flash, reset, motor start, road acceptance or actual IDE FPS test.

Install both the new C_RingFast/CarTest.elf and the matching complete OpenMV folder.
The private local installation package contains the calibration JSON in every
OpenMV package. Calibration, actual raw images and vehicle logs stay outside the
public checkpoint. Complete source includes Src/main.c and openmv/main.py.
