# v23 PE7 startup high-stop immunity candidate

Status: experimental-software-tested-hardware-pending. No automatic flash/reset, camera upload or motor start was performed. Hardware, road and60FPS acceptance remain pending.

## PE7 behavior

Power-on stopped. High stable20ms arms; low stable50ms creates a RUN request. From this qualified request through elapsed999ms, PE7 high is ignored by both1ms SysTick and rising EXTI. At1000ms, high stops immediately even without another edge. Low at the deadline ends the guard and the next high stops immediately. Bounce does not extend the guard. After stop, high20ms followed by low50ms is required again.

PE7 release cannot stop the vehicle during this initial window. Only PE7 is masked: hard faults, emergency stop, loop gap, communication timeout, profile ACK and vision safety remain active. This timer begins at the qualified request, not at first PWM or measured vehicle motion. RESET/KEEP/fresh-frame and servo gates still apply and can block startup.

## Paired package

H743: select exactly one A/B/C ELF in H743/. A=5.6/4.4rps ring off; B=4.8/4.0rps ring on; C=5.6/4.4rps ring on. These are target wheel speeds. Do not bypass pairing or fault gates.

Camera: all23 Python runtime files are byte-identical to v21/v22. If the completev21/v22 camera is already installed, only the selectedv23 H743 ELF needs updating. Older camera runtimes still require the complete paired package. OpenMV/ contains23 Python files and its manifest; validation .mpy files are compile evidence only. Keep the compatible current vendor firmware and valid private ring_width_calibration.json. B/C require the calibration and parked-width checks. Private calibration is intentionally absent from this public package.

All15 H743 builds passed. Host C/C++ regression:73 tests,7959 assertions,0 failures; actual main.c SysTick/EXTI/stop under fake registers:40 checks,0 failures; motor-output harness9 variants passed;36 reader/wire/delivery tests passed. All23 camera files cross-compiled. These checks do not establish physical switch timing, vehicle behavior or camera/USB FPS.

See docs/PE7-START-GUARD-20261029.md in the full source checkpoint for design, review and validation boundaries. Existing compact camera UART diagnostic behavior is unchanged; it does not forward full IDE text logs.
