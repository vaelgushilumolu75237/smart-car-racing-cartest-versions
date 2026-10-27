# H750 production camera package - three paired profiles

Upload the 23 Python files declared in PACKAGE_MANIFEST.json together. The
ready-to-upload directory is deliverables/TrackProfiles_H750_20261026/OpenMV.
`fast_ring_slope.py` remains an inactive compatibility module; `main.py` does
not import it. Do not update openmv_clipped_midpoint.

Use the matching H743 ELF from the same delivery. A/B/C are selected by H743
through a versioned profile-state message; the camera acknowledges the profile.
No manual camera config changes are needed when switching these three ELFs.

| Profile | Straight target | Curve/ring outer-wheel target | Ring |
| --- | --- | --- | --- |
| A / DebugFastNoRing | 5.6 rps | 4.4 rps | off |
| B / DebugRingBase | 4.8 rps | 4.0 rps | on |
| C / DebugRingFast | 5.6 rps | 4.4 rps | on |

Keep the current vendor H750 firmware (measured 5.0.1) and MT9V034 sensor.
Do not install the previous H743 camera FIFO/native-preview firmware.
Images remain 320x120. Forward traffic is V3/16 bytes; reverse RESET/KEEP is
V1/8 bytes, and profile/session/wheel-revolution status is V2/20 bytes.
The application fails closed on an unknown, mismatched, or stale profile.
Profiles B/C also require the CRC-verified 45 cm straight width map and a
three-frame parked bilateral width check before they acknowledge the profile
or send a valid steering observation. Profile A keeps its prior route.

After stopping the main camera loop, run `run_ring_width_calibration()` at the
REPL while the vehicle is parked on the measured straight. The recorded ground
frames can be replayed with
`run_ring_width_calibration('/flash/ring45_ground_raw.bin')`; the adjacent
JSON manifest and streaming raw-frame CRC are checked before the calibration
file is written. The collector sends only invalid V3 frames with ACK 0 and
publishes `ring_width_calibration.json` only after 64 accepted frames pass.

Ring history stays in bounded RAM during operation. After a real RESET or a
fresh inactive-profile transition, stop the camera script and call
`ring_diag_dump()` for the short summary, then `ring_diag_persist()` before
changing camera power. If the script was stopped manually without a received
transition, call `ring_diag_freeze()` first. The CRC-protected history is
written atomically to `ring_diagnostics.bin`.

Ring entry follows a measured-boundary sequence and freezes measured row
widths. Exit repair uses the opposite current boundary. Inferred points never
seed measured width. Confirmed ring targets that lack support produce invalid
observations; they do not silently select a straight branch. Profile A bypasses
ring state processing. Crossroads use a separate validated straight corridor.

Software and static-camera checks do not establish moving ring traversal.
See docs/H750-TrackProfiles-Implementation-20261025.md and the Chinese switching
instructions under docs for measured performance and remaining road tests.
The stable rollback ZIP and prior release remain available.
