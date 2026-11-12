# Restore previous PE7 startup logic (2026-10-29)

At the user's explicit request, the v18 maintained-switch change is reverted. Production `App/manual_run_button.c`, `Inc/manual_run_button.h`, and `Src/main.c`, plus their two affected host test files, are restored byte-for-byte from `v20261027-ring45-offline-v17` (bf4fffc1acb82b4fc192638d5795a3c4b3354011).

The restored Follow behavior is: after 20 ms stable released-high arming, a low press latches a RUN request; releasing high does not stop it; after high rearming, another low press cancels the request and stops. This is the prior toggle behavior, including its behavior with the user's maintained switch. No changes are made to vision, camera calibration, profile ACK checks, reverse RESET/KEEP, or output protection.

The v18 source change and artifacts remain available under their immutable tag. This restore does not alter old tags. The live camera width calibration is private and is not included in the public checkpoint. This operation does not flash/reset H743, upload camera programs, or start motors. Firmware on the vehicle is not inferred from source files.

Source restoration and software checks do not constitute physical button, vehicle, camera FPS, or USB 60 FPS acceptance.

## Verification

- Compared all three restored production files and both affected test files byte-for-byte with the frozen v17 Git tag.
- Host regression compiled with Zig 0.14.1 C11/C++17, using the production module and definition matrix from the existing MSVC runner: 73 cases, 7720 assertions, 0 failures, 13 protocol vectors.
- Production manual-start resync handoff check passed.
- Delivery/profile wire checks: 4 tests passed.
- All 15 H743 presets freshly built; all 23 unchanged production camera modules cross-compiled with MicroPython 1.28 armv7emsp.
- Hardware actions for this restore: none. Physical switch and road tests remain unverified. Camera private calibration is retained separately.
