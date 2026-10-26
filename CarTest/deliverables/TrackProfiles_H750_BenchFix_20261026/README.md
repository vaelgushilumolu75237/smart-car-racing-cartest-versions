# H750 camera and three H743 profiles

Upload all 20 Python files in OpenMV together; do not upload validation .mpy files.
Flash exactly one H743 ELF. It selects A/B/C through versioned communication.
An unknown, stale, or mismatched profile must not start a new Follow session.
A=5.6/4.4 rps, ring off; B=4.8/4.0 rps, ring on; C=5.6/4.4 rps, ring on.
Values are wheel-speed targets, not measured body speeds.

Keep the current H750 vendor firmware. No old H743 camera firmware is included.
Software tests and cross-compilation do not establish ring traversal or FPS.
Perform manual raised-wheel checks, straight, individual bends, crossroads, left/right rings, then complete laps with matching logs/video.
