# v25 wheel-speed variants

This is a derivative of the original v25 checkpoint, not v26 or v25_Perf.
Wheel-speed targets can be 100%, 120%, 130%, or 140% of the original profile.
A/C: 5.6/4.4 -> 6.72/5.28, 7.28/5.72, 7.84/6.16 rps.
B: 4.8/4.0 -> 5.76/4.80, 6.24/5.20, 6.72/5.60 rps.
These are requested straight / maximum-slowdown outer-wheel targets. Actual
speed increase depends on motor capability and PI output saturation.

Original nominal pairing speed macros and A/B/C ACK admission remain checked.
A separate CARTEST_SPEED_PERCENT scales the target enum consumed by Ackermann,
incremental PI, ordinary PI, and the snapshot's legal target range.
Each variant's profile and speed percentage are part of the ELF build ID.
Steering mapping, PI gains, PE7, PWM limits, 200ms deceleration, 2-second
vision hold, communications protection and hard faults are unchanged.
Production OpenMV runtime bytes are identical to the original v25.

Build, for example: python tools/build_speed_variants.py --profile C --percent 120
The C preset is DebugRingFast_Plus20, not the nominal DebugRingFast preset.
Build artifacts and hashes for every profile are in SPEED_VARIANT_ARTIFACTS.json.

All 24 declared build presets were built. Host validation covers the 12 A/B/C
and percentage combinations for Ackermann targets/ramps/left-right geometry,
PI override acceptance, hold priority, and snapshot publish/CRC/range rejection.
The common H743 suite passed 72 cases / 7713 assertions and the 2000ms/200ms
vision-hold harness passed. Camera tests: 366 run,
4 skipped, zero failures. One skip needs the missing private ground-raw fixture;
ground replay remains unverified. All 23 camera runtime modules cross-compiled.
Software validation does not establish road behavior or camera/IDE FPS.
No vehicle flash, camera file update, reset, or motor start was performed.

The private local distribution includes the calibration JSON in each OpenMV
installation directory; private calibration and actual logs are excluded from
the public checkpoint. Keep the package paired with its own v25 vision.
Do not combine these older vision files with the installed r21 modules.
