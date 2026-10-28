# v25 Ring joint repair - software-tested candidate

Status: experimental-software-tested-fps-road-pending. Hardware validation, actual steering behavior and both no-preview/IDE-preview more-than60FPS acceptance remain pending. No camera upload, H743 flash/reset or motor start was performed.

## Installation payload and pairing

OpenMV/ contains all23 production Python files and PACKAGE_MANIFEST.json. Update the runtime as a complete set; do not upload validation .mpy files. Keep the currently compatible vendor firmware. Never format the camera or remove its valid private ring_width_calibration.json. Public checkpoints exclude private calibration; a separate local-private upload copy is supplied only for the same replacement camera/lens/mount. B/C still require calibration, parked check, fresh profile ACK and all ordinary safety gates.

H743/ has A/B/C ELF snapshots freshly built from unchangedv24 source. All15 ELFs are byte-identical to v24; PE7 is the exact requested original toggle switch. If already on the matching v24 C ELF, it does not need reflashing for this camera-only candidate. C SHA256=9e0ab9033746f5fd885a2a8136e4c78e323b7f44129969f2a51a823d33048076. A=5.6/4.4rps ring off; B=4.8/4.0rps ring on; C=5.6/4.4rps ring on; these are wheel targets.

## Changes

Camera tracker subtracts common edge motion, vetoes entry with a validated ordinary bilateral corridor and requires3 fresh growing-arc frames. A confirmed but unusable ring may release control only after3 new ordinary-route confirmations with no adequate ring geometry; invalid routes remain invalid. Calibrated target provenance32 now passes the converter's strict calibration/parked gate instead of being wrongly rejected by its old cache-only16 check. Global servo polarity, speed gains and2second loss continuation are unchanged.

Use the full source checkpoint's docs/RING-JOINT-V25-20261029.md for detailed guards and validation. Startup must print TRACK_BUILD=CARTEST_H750_RING_JOINT_V25_20261029. Mixing older track_elements/ring_diagnostics modules is rejected. RNGD format3 records target side/center, weighted and sent error, and route admission; the updated read-only exporter remains compatible with old formats1/2.

365 camera host tests passed with3 existing skips and one separately excluded missing-private-RAW test;23 files cross-compiled;72 H743 cases/7713 assertions and26 switch-chain checks passed;15 fresh builds passed. These results do not prove real ring entry, vehicle steering or FPS. User video is compressed/overlaid and cannot stand in for an exact raw replay.

FPS remains a hard outstanding requirement: complete320x120 fresh processing/full sending>60FPS, including actual IDE fresh preview>60FPS. No host timing, nominal video rate, repeated image or reduced preview rate qualifies. tools/check_openmv_fps_acceptance.py requires at least60s of aligned board/preview evidence. No performance PASS is claimed by this candidate package.
