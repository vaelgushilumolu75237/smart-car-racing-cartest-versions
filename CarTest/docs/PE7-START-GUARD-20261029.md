# v23 PE7 one-second high-stop immunity after qualified start

## User-approved behavior

The user explicitly selected ignoring only PE7 high for one second after startup, including release during that window, while retaining communication timeout, profile pairing, hard faults and emergency stop. The implementation starts its timer when the low50ms qualification creates the manual RUN request. This is before motor motion is authorized: RESET/KEEP, fresh observations, ACK and servo/output gates can still delay or reject actual PWM. It does not guarantee one second of motor movement.

Power-up still stops and requires high20ms, then low50ms. The FSM remains WAIT_HIGH/WAIT_LOW/RUNNING sampled by1ms SysTick. From qualified start through elapsed999ms, PE7 high from either SysTick or rising EXTI is ignored without clearing the request or adding a stop event. At elapsed1000ms, a currently high level stops immediately even if no new rising edge occurs. Low at the deadline expires the one-shot guard; any subsequent high stops immediately. No high or low bounce extends the deadline. After stop, high20ms and low50ms must rearm. Hard fault handling runs before high handling and cancels the guard/request. Timer arithmetic handles uint32 wrap; the expired guard never reopens until another qualified start.

The driver must expect that PE7 release alone cannot stop the vehicle during this initial window. Other protections and the independent hardware motor power cutoff remain available. Fault or pairing rejection can stop/block the vehicle during the window. This change does not claim the physical high-level disturbance has been identified or repaired.

## Production code and review

Only `Inc/manual_run_button.h` and `App/manual_run_button.c` change. Added a1000ms constant, qualified-start timestamp and one-shot guard flag. Both SysTick and EXTI already call the shared module, so no duplicate main.c mask or raw-GPIO substitution is used. Reviewed Follow motor_output_guard, pending-start handoff, main/SysTick/EXTI paths and commissioning gates: Follow uses the latched request, while Bench raw-high stop remains unchanged. Controller protocol/vision/ACK/fault/loop-gap and PWM emergency-stop checks do not consult this guard. Camera runtime, UART protocol, wheel targets and gains are unchanged fromv22/v21.

## Validation and boundaries

- 73 C/C++ host tests,7959 assertions,0 failures, including low49/50ms, ignored high999ms, high stop at1000ms, rearm, counter integrity, wrap, expiration on low, fault override with high and actual controller raw-high handling. Protocol fault inside the window still clears drive/output.
- Actual production SysTick/EXTI/stop under fake registers:40 checks,0 failures, including ignored EXTI, unchanged applied PWM in the window, held-high deadline stop without another edge and hard-fault output shutdown.
- Actual main.c output guard/apply harness:9 configurations,0 failures, including hardware locked, PI, incremental PI and Ackermann.
- Static start/stop wiring audit and36 reader/status/wire/delivery tests pass.
- All15 H743 presets freshly compiled, and23 unchanged camera files cross-compiled. Evidence saved locally under logs/diagnostics/pe7-startguard-v23-tests and pe7-startguard-v23-builds-20261029. Pre-edit source/config/paired ELF preservation is under pre-pe7-startguard-v23-20261029.

Host registers are mocks; this is not a measured physical startup/stop timing result. No board flash/reset, camera upload or motor start was performed forv23. Hardware/road validation and camera/USB60FPS remain pending. A completev21 orv22 camera already installed needs no re-upload; older versions still need the paired diagnostic-capable camera package. ACK and private calibration checks must never be bypassed. Private vehicle data/calibration remain outside the public checkpoint.
