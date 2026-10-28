# PE7 maintained-switch candidate v18 (2026-10-29)

This candidate corrects H743 Follow handling for the physical maintained/latching PE7 switch: low=RUN, high=STOP. Previously a high release only rearmed a second low-edge toggle and did not stop. High now cancels both a running request and a pending start at the first sample, queues the existing normal-stop state transition, and immediately stops motor hardware. EXTI and main-loop fallback share the path. A new low start still requires 20 ms continuous high; held-low startup and fault recovery cannot start automatically. Bench and DebugSteering behavior is unchanged.

Status: experimental software candidate, hardware validation pending. No H743 flash/reset, camera program upload, or motor start was performed for this source change. The vehicle still runs the old switch logic until the user deliberately flashes a matching new ELF. Disconnect motor drive power for initial switch checks, then validate secured raised-wheel stop behavior before road testing.

All 15 declared H743 configurations were rebuilt with STM32Cube ARM GCC. A/B/C target wheel speeds and profile pairing remain unchanged: A=5.6/4.4 rps with ring off; B=4.8/4.0 with ring enabled; C=5.6/4.4 with ring enabled. Values are wheel targets, not measured body speeds.

The production OpenMV Python sources are byte-for-byte unchanged from v17 and all 23 were cross-compiled using MicroPython 1.28 armv7emsp. An existing complete v17 camera installation does not need re-upload for this switch change. Keep its private camera-specific width calibration. This public delivery contains no private calibration, real imagery, or actual vehicle logs. Validation .mpy files must not be installed on the camera.

C pairing still requires V3/ACK3/ready1; B/C additionally require valid camera width calibration and the parked startup width check. Do not bypass these checks. Communication, invalid-observation, reverse RESET/KEEP, servo-first-frame, and motor ramp gates remain intact.

Host assertions and builds do not establish physical stopping latency, camera full-pipeline 60 FPS, USB 320x120 new-frame throughput, or vehicle behavior. Ring route selection remains experimental. See docs/PE7-MAINTAINED-SWITCH-20261029.md in the complete checkpoint for review and manual checks.
