# PE7 three-state 1ms control and v21 validation

## Cause and change

PE7 is an active-low maintained switch, not a momentary toggle. Historical edge/toggle behavior and main-loop sampling did not implement the requested maintained-switch contract. The v20 implementation used low/high levels but lacked the explicitly requested three-state, SysTick-driven implementation. This revision replaces it with WAIT_HIGH, WAIT_LOW, RUNNING.

The latest successful historical board capture had two start events and two stop events, while C required ACK3 and received ACK0 with invalid vision; fault6 followed approximately251ms after a start. That capture establishes a profile/vision safety rejection for that incident, not a missed PE7 event. A private calibration file had previously been missing; after the later camera re-upload, ACK0 alone cannot distinguish missing calibration from failed parked validation or reverse-state problems. The latest read-only ST-Link attempt failed core access; this revision has not been read back on the board.

## Production implementation

- `Inc/manual_run_button.h`, `App/manual_run_button.c`: explicit three-state FSM, unsigned wrap-safe20ms timers. Power-on defaults stopped; high20ms arms, low20ms requests RUN; continuous low retains request. First observed high clears request immediately. Stop must be consumed and high20ms followed by low20ms before another start. Hard faults cancel and require rearming; held-low never restarts after a fault.
- `Src/main.c`: Follow-mode sampling from SysTick every1ms after GPIO/controller initialization. Rising EXTI only accelerates stopping; no falling-edge start dependency. Start handoff rechecks FSM/request/pending stop under IRQ protection. Existing profile, vision, fault, servo and hardware gates remain. Stop clears TIM3 CCER, CCR1..4 and CEN and drives motor pins low. Ordinary speed/steering gains are unchanged.
- Debug: GPIO/raw level, FSM, request/start/stop counts, sample count/time/max gap, CCR2/CCR3/CCER and timer/channel enable. RUNNING denotes a latched request, not proof of actual motion. Main-context debug mirrors can lag an ISR; output protection uses the live state.
- `tools/run_status_reader.py`, `tools/read_run_status.ps1`: selected-ELF identity checks retained; optional new fields remain compatible with older ELFs.

## Camera diagnostics

See [UART diagnostic contract](CAMERA-UART-DIAGNOSTICS-V21.md). New camera and H743 must be deployed together. Existing PA9/P5, PA10/P4 and common ground suffice. ST-Link reads a bounded H743 RAM cache of compact camera status/counters, not the camera's full Python/IDE text log. Freshness, CRC and generation checks prevent old/torn data being presented as current. Diagnostics never grant motor permission, count as vision, refresh its timeout, or clear a fault.

## Software evidence

- All15 H743 presets freshly compiled successfully, including A/B/C. Source hashes remained unchanged during the builds.
- Pure C/C++ regression:73 tests,7791 assertions,0 failures, built with Zig0.14.1 because MSVC is unavailable locally.
- Actual main.c SysTick/EXTI/stop functions plus real button/output module under fake registers:37 checks,0 failures. Includes exact20ms boundaries, bounce, high immediate stop, hard fault, initialization guard and caller IRQ preservation.
- Actual main.c output guard/apply functions under HAL/register mocks:9 variants,0 failures, including PI/incremental PI/Ackermann and hardware-locked build. Legacy standalone MSVC servo16-case invocation was not run by this portable wrapper.
- Camera suite:348 tests run,0 failures,3 existing skips. One additional private ground-RAW replay test was excluded because its original capture is unavailable on this PC.
- All23 camera runtime files cross-compiled with MicroPython1.28 armv7emsp. FIFO tests exercise late-cycle idle scheduling and unchanged primary-frame timestamps; synchronous fallback must have22000us remaining budget.
- Reader/status/wire/delivery regressions:36 tests passed. PowerShell fake-programmer tests passed with old v20 and new v21 ELFs. Actual C-produced diagnostic snapshot decoded correctly with the Python reader. Focused MCU diagnostic tests cover bad CRC/fields, duplicate/rollback, ring wrap, stale controls and split frames.

These are host/build checks. No camera upload, H743 flash/reset or motor start was performed for v21. Physical debounce timing, switch wiring, motor enable, UART operation, road behavior and camera/USB60FPS remain unverified. Ring routing remains experimental. Latest local evidence is under `logs/diagnostics` and is excluded from the public checkpoint.
