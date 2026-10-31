# r20-derived r22 experimental candidate

C now uses a 3000 ms last-valid-error hold and 300 ms vision-loss deceleration.
C steady INSIDE requests 267 permille slowdown = 5.28 rps (+20% over 4.4); straight stays 5.6 rps.
Right-ring exit states admit the first spatially qualified current left boundary for calibrated centerline guidance.
Use the matching new C ELF and all 23 camera Python files together. No road/FPS acceptance.

# H750 camera and three H743 profiles

Upload all 23 Python files in OpenMV together; do not upload validation .mpy files.
Flash exactly one H743 ELF. It selects A/B/C through versioned communication.
An unknown, stale, or mismatched profile must not start a new Follow session.
B/C require the CRC-verified 45 cm straight calibration and parked width check; A is unchanged.
Run run_ring_width_calibration('/flash/ring45_ground_raw.bin') only at the REPL after stopping the loop; it sends invalid V3/ACK0 frames.
Persist frozen ring history with ring_diag_persist() before removing camera power.
A=5.6/4.4 rps, ring off; B=4.8/4.0 rps, ring on; C=5.6/4.4 rps, ring on.
Values are wheel-speed targets, not measured body speeds.

Keep the current H750 vendor firmware. No old H743 camera firmware is included.
Software tests and cross-compilation do not establish ring traversal or FPS.
Perform manual raised-wheel checks, straight, individual bends, crossroads, left/right rings, then complete laps with matching logs/video.
