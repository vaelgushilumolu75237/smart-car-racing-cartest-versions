# v24 PE7 reference restoration candidate

Status: experimental-software-tested-hardware-pending. No automatic H743 flash/reset, camera upload or motor start. Hardware/road/60FPS acceptance remains pending.

## Restored switch behavior

The switch module is copied byte-for-byte from the user-specified 10/25 perfect reference directory. Follow uses high stable20ms to arm, then a low event toggles RUN/STOP. Sustained low cannot repeat an event; release alone does not stop. Rearm high20ms, then another low toggles STOP. Both-edge EXTI and the three original main-loop fallback samples are restored. The newer50ms low dwell, maintained-level FSM and1000ms ignore-high window are removed. Hard faults, profile pairing, vision freshness and hardware/PWM safety gates remain.

## Package

H743: choose exactly one A/B/C ELF. A=5.6/4.4rps, ring off; B=4.8/4.0rps, ring on; C=5.6/4.4rps, ring on. Wheel targets, gains, camera UART diagnostics, crossroad/ring routing and two-second vision-hold policy are unchanged by this switch-only restoration. This newly built C ELF is not the historical reference frozen C ELF.

All23 camera Python files are byte-identical to v21/v22/v23. If that complete camera runtime is already installed, only the chosenv24 H743 ELF needs updating. Older camera runtimes still need the paired diagnostic-capable package. OpenMV/ contains23 Python sources and manifest; validation .mpy files are compiler evidence only. Preserve the compatible current vendor firmware and valid private calibration. B/C still require45cm calibration/parked validation and correct ACK; do not bypass them. Public source does not include private calibration.

Validation:72 host cases/7713 assertions,26 real production-call-chain checks under fake registers,9 motor-output harness configurations and36 reader/wire/delivery tests passed. All15 H743 presets freshly compiled;23 unchanged camera files cross-compiled. No physical vehicle timing, road or FPS claim follows from these tests.

See docs/PE7-REFERENCE-RESTORE-20261029.md in the complete source checkpoint for exact reference hashes, changed callers and validation boundaries.
