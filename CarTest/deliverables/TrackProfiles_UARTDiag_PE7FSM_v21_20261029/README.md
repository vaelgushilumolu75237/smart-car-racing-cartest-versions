# v21 paired camera diagnostics and PE7 FSM candidate

Status: experimental-software-tested-hardware-pending. This package is not road or60FPS acceptance.

H743: select exactly one A/B/C ELF from H743/. A=5.6/4.4rps ring off; B=4.8/4.0rps ring on; C=5.6/4.4rps ring on. These are target wheel speeds. Existing vision/profile/fault protection remains mandatory.

Camera: OpenMV/ contains all23 production Python runtime files plus PACKAGE_MANIFEST.json. Replace the complete runtime set together; validation .mpy files are compile evidence only. Retain the currently compatible vendor firmware. Do not format the camera or delete its private ring_width_calibration.json. B/C still require matching45cm calibration and a passed parked-width check. The public package deliberately excludes private calibration and vehicle captures.

This version changes the reverse profile capability flags, so deploy H743 and camera together. Mixed versions can reject pairing and send ACK0; do not bypass this check. No automatic deployment is provided.

PE7: power-on stopped; high stable20ms arms, low stable20ms runs; sustained low runs; first observed high stops immediately. Rearm high20ms then low20ms. Follow uses1ms SysTick; rising EXTI only provides earlier stop. Hard faults and all motor-output gates remain.

Existing UART wires carry sparse16byte compact diagnostic records at no more than1Hz. H743 caches16 records, readable with tools/read_run_status.ps1 using the exact selected ELF. This exposes calibration/parked gate, raw vs sent ACK, reverse state and error counters; it is not full IDE text forwarding. Diagnostic overhead is nonzero; hardware FPS is unverified.

All15 H743 builds passed and23 camera files cross-compiled. Host tests do not establish physical switch/PWM timing, camera FPS or road behavior. See docs/PE7-THREE-STATE-1MS-20261029.md and docs/CAMERA-UART-DIAGNOSTICS-V21.md in the source checkpoint for evidence and limits. No v21 camera upload, H743 flash/reset or motor start was performed.
