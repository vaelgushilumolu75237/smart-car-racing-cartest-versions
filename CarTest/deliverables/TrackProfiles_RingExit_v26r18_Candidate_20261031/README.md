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

## v26-r18 experimental exit candidate (2026-10-31)

The calibrated exit near-edge check now compares the measured inner edge with
the recent successfully transmitted ring lane at each row, rather than entry
image coordinates. Missing-side evidence must be absent (not ambiguous), with
consecutive near support. A topology, row, or fresh-frame interruption resets
the exit evidence streak. Confirmed calibrated ring ownership and frozen width
are retained when geometry is invalid or the ring times out; unconfirmed exit
cannot fall back to an ordinary single edge merely because time elapsed.

The existing H743 line-loss hold is retained: keep the last trusted signed
error/servo target, apply the existing 200 ms deceleration, and expire 2000 ms
after the last valid route. Repeated invalid V3 frames cannot renew that
deadline. The camera does not label an old error as a new valid route.
A newly measured valid route still needs the existing recovery checks.
PE7, ordinary steering mapping, profile pairing, hard fault protections and
the r16 C speed request (1000 permille in every ring phase) are retained.
H743 production sources and the paired C ELF are unchanged from r16.

Validation: 486 camera tests, zero failures/errors, 4 explicit skips (three
retired independent-routing tests and one unavailable private raw calibration
fixture); 23 armv7emsp cross-compilations; all 15 H743 builds; H743 host harness
72 cases/7713 assertions; B/C 2000 ms hold and 200 ms ramp assertions;
7 V3 parser/steering mapping vectors including mirrored current-anchor exits.
Historical review covered 443 saved records and 21 retained raw images across
three earlier hand-push captures. This is a sparse geometry check: those
captures lack the full frozen per-row transmitted-route history needed for
exact sequential final-UART verification. Several poses remain unsupported;
the review does not demonstrate continuous valid exit guidance.

This candidate has not been installed or road-tested. Left/right ring ground
traversal, ordinary road regressions and over-60-FPS with actual IDE preview
remain pending. Preserving an already wrong last error does not correct its
direction. Software tests do not establish physical exit performance.
