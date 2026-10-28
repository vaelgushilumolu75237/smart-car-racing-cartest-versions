# PE7 maintained switch correction (2026-10-29)

The physical PE7 control is a maintained/latching switch, not a momentary pushbutton. The previous Follow implementation toggled the run request only on low edges; moving the switch back to released-high did not stop Follow. This explains why another low transition was needed.

The candidate uses low as RUN and high as STOP. A high sample cancels a running or pending start immediately, queues the existing stop path, and stops motor hardware through both EXTI and main-loop sampling. Stop does not wait for the 20 ms release qualification. A new low start requires a continuous 20 ms high interval; startup while low, held-low fault recovery, and bounce must not start automatically. A stop queued before the start handoff wins.

All Follow profiles share this switch behavior. Bench and DebugSteering keep their existing behavior. A/B/C profile pairing, reverse RESET/KEEP recovery, vision validity, communication timeout, servo pulse completion, and motor ramp protections remain required. Ring route selection remains experimental.

This is a software candidate. Build and regression results are recorded in the paired delivery. It has not been flashed or checked on the physical switch. The firmware currently on the vehicle still uses the old toggle semantics until a deliberate H743 flash. Never infer road behavior or camera FPS from host tests or UART accepted-frame rates.

## Manual validation after an explicitly authorized flash

Disconnect motor drive power for the first switch checks. Start with PE7 at STOP; confirm no start request. Move to RUN once and verify exactly one request. Move back to STOP once and verify request clears and stop generation advances once. Repeat with a quick release during the startup handshake, held positions, and modest contact bounce; no rebound may restart the session. Then perform secured raised-wheel start/stop checks before road tests. Do not perform a driving test with uncertain stop behavior.

Production OpenMV sources are unchanged. Private camera width calibration and actual vehicle diagnostics are excluded from public checkpoints.

## Software verification completed

- Personal lead review: production change is confined to the manual-switch helper/header and the event assignment in `manual_button_sample`. Its existing STOP branch invokes `emergency_stop_hardware`; both EXTI and the polling fallback call this helper. Deferred start still rechecks the live run/stop state while interrupts are masked.
- All 15 CMake build presets freshly configured and built with STM32Cube ARM GCC 14.3.1; no target connection used by builds.
- Host core assertions: 73 cases, 7749 assertions, 0 failures, 13 protocol vectors. This machine lacks MSVC, so the same production modules, compile definitions, and `test_main.cpp` assertions from `tests/host/run_tests.ps1` were compiled with Zig 0.14.1's C11/C++17 host toolchain. The unchanged MSVC entry point was not runnable on this machine.
- `python -B tests/host/test_manual_start_resync_main.py --source Src/main.c`: passed high-stop hardware routing and live-state handoff checks.
- `python -B -m unittest tests.test_track_profiles_delivery tests.test_track_wire_vectors`: 4 tests passed.
- 23/23 unchanged production camera modules cross-compiled using MicroPython 1.28 armv7emsp. No offline timing was converted to board FPS.
- No physical switch edge-to-motor-stop latency, raised-wheel stopping, or road acceptance for this candidate. No board/USB 60 FPS acceptance.

## Candidate profile ELF hashes

- `A_FastNoRing`: `d747e8ce768957a18a9a5faa536632147f28a0b769194f9f9f82d3eb3d759f80`
- `B_RingBase`: `99684700f21655fd5d35cfc35145c757056be3d5ae4d814d0382a0cf1176c991`
- `C_RingFast`: `2ed04114c0a3384f37fd07592dbeced980ff418db257f4cf44753e4f4205293f`
