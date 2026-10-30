# PE7 level RUN/STOP with asymmetric debounce (2026-10-29)

The user explicitly requests maintained-switch level control: low requests RUN and retains it while low; high cancels RUN and stops. This supersedes the previous v19 request to restore toggle semantics. Only the PE7 helper/header and its existing main sampler integration are changed in production. Camera algorithms, speed targets, profile pairing, and vehicle control parameters are unchanged.

Startup and new RUN qualification require a continuous released-high interval of 20 ms followed by a continuous low interval of 20 ms. Any intervening opposite level restarts the respective qualification interval. The first high sample while a run or deferred start is active immediately clears the request and pending start, queues the existing stop, and invokes the existing motor emergency-stop path. STOP does not wait for debounce. A queued stop must be consumed before rearming; holding low after stop or a fault cannot restart automatically.

Both EXTI and the main-loop fallback use the same interrupt-serialized sampler. A held low can create only one start event, and held high can create only one stop generation for that request. The controller's existing reverse RESET/KEEP and fresh observation recovery still apply after each accepted start. C pairing remains V3/ACK3/ready1, and B/C camera width calibration gates remain mandatory.

No H743 flash/reset, camera program upload, or motor start is performed for this software change. The board was last identified by a matching ROM descriptor as the user's October25 C reference; this identification is not evidence that the new level-control firmware has been flashed. Physical switch timing, motor stopping, road behavior, and board/USB60FPS acceptance remain pending. Ring routing remains experimental.

For manual validation after an explicitly authorized flash, first disconnect motor drive power. Confirm startup while held low does not request motion; release high, then switch low and verify exactly one qualified request. Switch high once and verify the request clears and stop generation advances once. Check sub-20-ms bounce and restart qualification before any secured raised-wheel motor test.

Private width calibration and actual hardware diagnostics remain outside public checkpoints. Previous immutable tags are preserved.

## Completed software verification

Lead personally reviewed the helper/header and the one-line main sampler change. Production edits are restricted to `App/manual_run_button.c`, `Inc/manual_run_button.h`, and `Src/main.c`'s event assignment. All other production code and control parameters remain unchanged from v19.

- Host C11/C++17 production-module regression (Zig 0.14.1; same module/definition matrix as the MSVC runner): 73 cases, 7787 assertions, 0 failures, 13 protocol vectors. MSVC is not installed on this machine.
- Tests include sub-20-ms and interrupted low pulses, exactly-20-ms low acceptance, held low, immediate high stop during running/deferred start, no duplicate stop, stop-pending priority, high/low debounce tick rollover, hard-fault start rejection, high stop despite hard fault, and held-low no restart after fault recovery.
- Production main STOP hardware-routing and start-handoff guard checks passed.
- Delivery/profile wire checks: 4 tests passed.
- All 15 H743 presets freshly built with production hashes unchanged throughout the build.
- All 23 unchanged OpenMV production modules cross-compiled with MicroPython1.28 armv7emsp.
- No physical switch, motor, road, board-FPS, or USB60FPS acceptance claimed.
