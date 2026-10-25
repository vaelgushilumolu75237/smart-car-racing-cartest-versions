# H750 production camera package - three paired profiles

Upload the 20 Python files declared in PACKAGE_MANIFEST.json together. The
ready-to-upload directory is deliverables/TrackProfiles_H750_20261026/OpenMV.
The retained fast_ring_slope.py is only an inactive historical host-test module;
it is excluded from the upload package. Do not update openmv_clipped_midpoint.

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

Ring entry follows a measured-boundary sequence and freezes measured row
widths. Exit repair uses the opposite current boundary. Inferred points never
seed measured width. Confirmed ring targets that lack support produce invalid
observations; they do not silently select a straight branch. Profile A bypasses
ring state processing. Crossroads use a separate validated straight corridor.

Software and static-camera checks do not establish moving ring traversal.
See docs/H750-TrackProfiles-Implementation-20261025.md and the Chinese switching
instructions under docs for measured performance and remaining road tests.
The stable rollback ZIP and prior release remain available.
