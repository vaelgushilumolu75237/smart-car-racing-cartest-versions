# Restore previous PE7 toggle logic v19 (2026-10-29)

The user requested restoration of the previous startup logic. The three H743 production files and two affected host test files are restored byte-for-byte from v20261027-ring45-offline-v17. The v18 maintained-switch change is not active in this candidate.

Follow semantics are the original toggle behavior: after stable released-high for 20 ms, one low press requests RUN; release rearms the next press but does not stop; a later armed low press stops. This deliberately restores the previous behavior even with a maintained physical switch.

73 host cases and 7720 assertions passed. All 15 H743 presets were freshly built. This delivery freezes new A/B/C ELFs; target wheel rates remain A=5.6/4.4 rps (ring off), B=4.8/4.0 (ring on), C=5.6/4.4 (ring on). These are wheel targets, not measured body speeds. Full manifests include ELF hashes and build IDs.

All 23 production OpenMV sources remain identical to v17 and passed MicroPython1.28 armv7emsp cross-compilation. No re-upload is needed for an already matching camera installation. Preserve its private camera-specific width calibration; this public package contains no private calibration, vehicle imagery, or actual vehicle logs. Validation .mpy files are not deployment files.

A/B/C profile pairing and all safety checks are retained. C still requires V3/ACK3/ready1. B/C require valid width calibration and parked startup checks. Ring routes remain experimental.

No H743 flash/reset, camera program upload, motor start, physical switch verification, road acceptance, or board/USB60FPS acceptance was performed for this restore. Firmware on the vehicle is unchanged by source restoration. v18 remains in its immutable Git tag. See docs/PE7-RESTORE-PREVIOUS-20261029.md in the complete checkpoint.
