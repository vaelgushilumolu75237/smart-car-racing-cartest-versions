# v26-r10 diagnostic candidate

This revision instruments the first loss of an actually transmitted outer-edge ring route. It does not change route admission, steering, speed, PE7, the 200 ms slowdown, the H743 2 second loss policy, firmware, or calibration.

The tracker exposes independent outer eligibility/build/filter/handoff reasons and support metrics. Legacy RNGD format 3 and its rejection counters remain unchanged. A separate bounded capture retains the last valid transmitted outer-route image, the first invalid frame, and the next active frame. Each image is raw GRAYSCALE 320x120 before preview overlays, tied to the frame sequence, session, target and actual UART error. Candidate tracker success alone cannot arm the capture.

Three raw buffers consume 115200 bytes. Active recording performs no filesystem writes or console output. At a recognized inactive Follow stop, the existing frozen BIN is saved, followed by three uniquely named PGM files and a JSON manifest with raw CRC32 values. The JSON manifest commits the set only after image writes succeed. Partial writes retain RAM evidence and do not overwrite completed images. A RESET that still reports active Follow cannot persist raw images; diagnostics block and retain the evidence in RAM. Preserve camera power until extraction. Do not delete older recordings automatically.

Host regression: 434 tests, zero failures/errors, 3 existing skips. One existing replay requiring missing private RAW data remains excluded explicitly. Added 13 tests cover actual UART selection, first failure retention, identity isolation, ring exit, no active file I/O, stopped persistence, active RESET rejection, partial write retry, CRC, and genuine outer rejection preservation. All 23 runtime files cross compile for MicroPython 1.28 armv7emsp. Three paired H743 ELFs match r9 byte for byte; no new H743 source or binary change is required.

An isolated H750 RAM check allocated the three buffers alongside the existing ring recorder and runtime modules: 2836864 bytes free. Copying one raw frame and its metadata averaged 625 us over 200 samples. This is not full pipeline FPS, USB preview acceptance, or a road test.

The candidate remains subject to explicit camera Python installation authorization. Do not flash camera APP, change the H743, or start motors as part of this update. Keep the existing calibration. Ground ring performance and 60+ FPS with actual IDE preview remain pending. This version gathers the missing evidence; it does not claim the ring failure is fixed.
