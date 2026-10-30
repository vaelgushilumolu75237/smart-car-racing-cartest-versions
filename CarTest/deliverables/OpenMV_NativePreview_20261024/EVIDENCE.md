# Candidate build and test evidence

This candidate is compiled, not flashed or accepted on hardware. The original production camera runtime and paired H743 images remain unchanged.

- Upstream: OpenMV v5.0.0, commit `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`.
- SDK: 1.6.0; target OPENMV4P.
- Preview patch SHA-256: `2bbb92b2b25705002ae16b9e9f18f57fc171ca8f0549680cc15260cce0b468d5`.
- [CI run 40003611704](https://github.com/vaelgushilumolu75237/smart-car-racing-cartest-versions/actions/runs/40003611704) completed successfully, including fixed-source checks, patch application, actual compilation and artifact upload.
- CI checkpoint commit: `78c8d4f6d7e7c9a83935aea60c749c4685bf2cbf`.
- `firmware.bin`: 1,736,532 bytes; SHA-256 `9689d686881d2d358694f580256d0698f01226cd018c6f6951d1d7abf5c663b7`.
- `firmware.elf`: 11,322,036 bytes; SHA-256 `dafaec93152163769e81e0f86fcd9b280b289f4e6a49a87d1c29ac1fd559caaa`.

Read-only ELF review confirmed ARM32 LE ET_EXEC, v7E-M and FPv5-SP-D16 attributes, OPENMV4P/STM32H743 identifiers, vectors at 0x08040000, Thumb Reset_Handler/entry 0x08040299, and the official 64 KiB ITCM stack with top 0x00010000. Reconstructing the Flash load image from ELF PT_LOAD contents exactly reproduces the bin. This proves target/layout consistency, not that the board boots or runs the image.

## Existing firmware measurements

The following are separate RAM experiments on the existing, unmodified OpenMV v5.0.0 firmware, using real sensor images on a static straight track and a RAM UART sink with simulated KEEP. They are not measurements of this candidate firmware.

| Condition | Fresh processed captures per complete 10-second window |
| --- | --- |
| USB preview disabled, production overlay | 608 / 608 / 608 |
| SDK JPEG preview, production overlay | 354 / 351 / 353 |
| Actual OpenMV IDE 5.0.0, visibly active 320×120 preview | 380 / 381 / 365 |
| SDK JPEG preview, compact text RAM experiment | 387 / 383 / 383 |
| SDK RAW320×120 preview | 132 / 135 / 134 |

Other lower-resolution preview experiments did not meet the strict target and are not accepted as a substitute. An initial IDE auto-run produced 631/632/631 counts without observed live preview; that run is excluded from preview-on acceptance. The repeat run had visible 320×120 live imagery and the 380/381/365 board counts above. The IDE's separate camera/GUI FPS labels are not substitutes for processed-frame and advancing-marker counts.

Actual scene screenshots, raw board stdout and user environment details stay local under `logs/usb-preview-20261024/` and `tmp/`; only this aggregate evidence is public. Physical UART reception, motor output and road behavior were not tested in these probes.

## Software coverage

The native-patch tests check pinned source hashes, the exact MicroPython gitlink, parameter semantics, the NO_UPDATE flag, module capability exports and patch application. The host preview harness has bounded operation deadlines. The independent binary marker decoder checks magic, sequence and CRC, including duplicate/gap/wrap/out-of-order behavior and synthetic quality-30 JPEG decoding. Synthetic JPEG tests do not prove live USB delivery.

The new full-resolution Q30 path still requires manual firmware installation and physical measurement. No simultaneous 60+ processing and 60+ unique 320×120 preview result is claimed.

## Encoder microbenchmark on the existing firmware

A separate RAM-only experiment processed the same real sensor image, retained
its production overlays, then made 20 independent full-size JPEG copies per
quality. It did not publish the JPEGs or measure whole-pipeline throughput.

| Quality | Bytes per JPEG | Mean encode microseconds | P95 microseconds | Maximum microseconds |
| --- | ---: | ---: | ---: | ---: |
| 10 | 1695 | 673 | 693 | 1129 |
| 20 | 2155 | 758 | 1227 | 1807 |
| 30 | 2589 | 834 | 1255 | 1887 |
| 40 | 2894 | 851 | 1254 | 1834 |
| 50 | 3213 | 776 | 790 | 1816 |
| 60 | 3591 | 793 | 799 | 1918 |

The 320x120 grayscale input remained unchanged after all copies. Quality 30
produced a 2589-byte sample, below the observed 4082-byte protocol payload limit.
This supports testing quality control; it does not prove a USB or IDE frame rate.
Host status polling can interrupt these measurements.

## Native performance-counter experiment

A RAM-only candidate compiled three performance-counter methods with the native
decorator. Board differential tests retained the same values and container
identities, including large integer sums and reset. A 5700-call record loop took
128774 microseconds in Python and 85917 natively, about 7.5 microseconds saved per
call. This is approximately 0.14 ms per 19-call frame, not a solution to the USB
preview bottleneck. Production runtime files remain unchanged pending integrated
measurements and review.

The integrated 30-second counter candidate measured 610/610/610 fresh captures
per 10 seconds with preview off, and 355/351/350 with full-size SDK JPEG preview.
All complete windows had valid routes and no UART-sink write errors. This confirms
only a small no-preview gain; it did not resolve the preview bottleneck. The
candidate was not adopted into production. The final partial 1-frame window is
excluded from throughput comparisons.

## Physical JPEG marker validation (synthetic scene)

The old firmware executed a RAM-only 320x120 grayscale marker probe, independent
of the camera and H743. It published 75 quality-30 JPEG copies over 1510 ms.
The PC received 74 frames: all 74 passed magic and CRC; 25 distinct advancing
sequence values, 49 repeats and one 16-bit wrap were detected, with zero sequence
gaps or out-of-order values. This validates the board marker encoder and host
decoder across JPEG and USB for this synthetic probe. It is not a real sensor
capture or 60 FPS acceptance test.

The first probe attempt used an unsupported positional argument to Image.clear;
that diagnostic-only API error was corrected to clear(). Failed attempts remain
in local logs and are excluded from performance results. No production file was
changed by this correction.

The final bounded benchmark harness also passed on the board after adding a
2-second post-DONE stdout-drain interval outside the measured publication loop.
It received all 75 published JPEGs: 75 valid CRC markers, 25 advancing sequence
values, 50 repeats and one wrap; no gaps, order errors, CRC, magic or shape errors.
The harness explicitly recorded script completion before the requested 4-second
observation duration instead of claiming a complete performance window. The
previous soft-reboot/end-of-script transport timeout is retained as a failed
attempt, not silently reported as success.

Final software verification passed 40 host tests with the SDK import test enabled.
The standalone live-preview probe and both generated RAM probes passed the pinned
armv7emsp mpy-cross compiler. Production source/package and H743 artifact hashes
were checked against their frozen manifests; the 12 existing H743 configuration
ELFs were not rebuilt because their source was unchanged in this preview-only
work. Native firmware compilation is separately recorded by the CI above.
