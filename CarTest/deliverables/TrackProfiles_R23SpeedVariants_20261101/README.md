# r23 C whole-speed variants: +20%, +30%, +40% (2026-11-01)

Open CarTest/CarTest.code-workspace. Debug entries show percentage, straight/ring
wheel targets, and the retained 3s hold / 300ms hold deceleration. Each entry has
its own checked preLaunchTask, build preset and ELF path. The baseline and
historical entries remain labeled separately.

| Variant | Straight rps | Steady ring outer rps | Full slowdown outer rps |
| --- | ---: | ---: | ---: |
| Base | 5.600 | 5.280 | 4.400 |
| +20% | 6.720 | 6.336 | 5.280 |
| +30% | 7.280 | 6.864 | 5.720 |
| +40% | 7.840 | 7.392 | 6.160 |

These are wheel-speed targets, not measured vehicle speed. Inner-wheel targets
retain the steering-derived Ackermann ratio. Request interpolation occurs at
the baseline speed before percentage scaling, so the existing ring slowdown
request 267 produces the exact values above. PWM limits, PI gains, steering
mapping/slew, PE7, pairing/ACK checks, hard faults and hold times are retained.
Percentage is included in the MCU build fingerprint. Only C allows speed >100%.

All variants share the EXACT r23 camera runtime (23 Python source hashes match).
Camera TRACK_BUILD remains TRACK_RING_EXIT_V26_R23_20261101. Each local Install
folder includes its own complete OpenMV subdirectory, main.py and the unchanged
calibration JSON. H743 is different for each percentage: flash ONE corresponding
ELF or HEX, using its matching debug entry. No board has been flashed/reset and
no camera uploaded by this packaging task.

Validation: all 18 actual ELF builds passed; 4x15 actual Ackermann control cases
verified target values and per-wheel scaling/geometry; 4 negative header builds
and 3 negative CMake configurations rejected; 72 MCU host cases / 7713 assertions
passed; C 3000/300 boundary fixture passed. Debug launch/task/path correspondence
verified. Camera source is byte-identical to tested r23 (519 tests, 4 skipped;
23 cross-compiles); no claim of a new board camera measurement is made.

Road exit and high-speed tracking remain unverified. Actual board and IDE-preview
>=60FPS remain unverified. Host tests/builds do not establish road behavior.
The original r23 source/configuration/paired ELFs remain preserved; hashes are
in R23_BASELINE_SHA256.json. Private calibration/logs/images are excluded from
public checkpoints. Use the included base C firmware to restore base speed.
