# Evidence and acceptance boundary

## Build

- GitHub Actions run 40095756045 completed successfully for source commit `a44ba5186711e37cde4a65c19b6b3dd9780e75af`.
- OpenMV `v5.0.0` commit: `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`.
- MicroPython commit: `d18a77e5d55664045e71a0c839695df07a96a6e1`.
- Preview control API marker: `1`; UART FIFO API marker: `1` (`UART.WRITE_FIFO_API`).
- Binary and ELF hashes and byte counts are pinned in `artifact-manifest.json`.

## Reported flash and earlier static UART observation

The lead reports that, on 2026-10-25, the board was fully flashed and the complete flash readback hash matched this `firmware.bin` SHA-256: `0bbf81a534f99365a62187b3116559e313527e0aa1b7d585e618022b11feffc3`. This delivery task did not repeat or independently verify that board operation.

The earlier static straight test used the preview application wrapper for 65 seconds. Six complete board-side windows ranged from 60.6 to 60.7 FPS; six independent host marker windows ranged from 60.2 to 60.8 FPS. The UART FIFO accepted 3,944 frames (63,104 bytes) with no partial writes, busy results, or errors; transmit-complete was true.

A concurrent read-only H743 RX/parser observation lasted one second and saw 60 invalid frames. The camera sequence stayed fixed at zero, so sequence 60 was rejected. There were no accepted valid route frames, control outputs, tokens, or events. This is static invalid-only transport evidence; it does not validate a usable route-control stream or vehicle response.

## Newer IDE 65-second payload-read measurement

The lead reports a 65-second OpenMV IDE run using the same 19-module batch and UART FIFO wrapper, with intentionally invalid wire frames and a fixed sequence value of zero. The board-side performance counters recorded valid frames in all six complete 10-second windows; their FPS samples were 60.7, 60.8, 60.7, 60.8, 60.6, and 60.8 (range 60.6-60.8). The run accepted 3,949 FIFO frames (63,184 bytes), with zero busy results, short writes, or UART errors; TC was true before and after.

The IDE terminal measurement counted complete `FrameDataSize` payload-read records. Across six conservative windows, the rates were 60.5142647373, 60.2316303329, 60.2352603627, 60.1152914784, 60.3861933008, and 60.1489975176 Hz (range 60.1152914784-60.5142647373 Hz). All 3,986 advertised/read size records matched, and there were no error lines. These are matched size records, not proof of 3,986 unique frames.

This IDE result establishes complete payload reads only. It does not establish independent per-frame JPEG decoding, IDE marker freshness, or physical display refresh. The wire frames remained intentionally invalid with sequence zero, so this did not validate accepted route control.

A separate SDK marker measurement recorded six complete windows in the 60.2-60.8 FPS range and supports independent decoding of new frames in that separate test. It is not merged with the IDE payload-read measurements and does not establish that each IDE payload was independently decoded or physically refreshed.

## New CDC-closed native headless run

The lead reports a separate 65-second real-sensor run using the same camera `main.py` revision as the IDE payload-read run and the real UART FIFO. The wire stream was intentionally invalid with sequence fixed at zero. CDC was closed for 68 seconds and `NATIVE_HEADLESS`, close-confirmed, and headless-observed indicators were set. The session report does not measure physical cable state; the lead observed that the USB cable remained inserted. This is not a physical USB-disconnect test.

Six complete 10-second board windows reported capture, valid, and accepted counts of 794, 794, 792, 794, 792, and 793, with corresponding rates of 79.4, 79.4, 79.2, 79.4, 79.2, and 79.3 FPS. Each window's total-cycle p95 upper bound was 13,000 microseconds; the largest observed total-cycle maximum was 30,759 microseconds.

Across the 65-second run, the FIFO accepted 5,156 frames (82,496 bytes) from 5,156 attempts. Busy results, partial writes, and errors were zero; TC was true before and after. JPEG marker writes, preview errors, and buffer overflow were zero. The complete 2,271-byte report was returned successfully by line. These are report and transport measurements; no IDE JPEG decode or physical display refresh claim is made from this run.

A separate one-second read-only SWD RX observation saw 79 invalid frames on the fixed sequence-zero wire. 79 fixed-sequence-zero frames were rejected by the sequence check; parser errors, outputs, tokens, and events remained zero. This confirms rejection of the invalid test stream and does not validate effective route control.

## Native and remaining checks

For the earlier candidate run, 10 Python tests passed and 2 errored because the temporary source checkout lacked Git metadata. The separate C harness passed 5 groups. The complete native Python suite therefore was not cleanly passing. The firmware CI build is build evidence, not proof of route behavior or physical display refresh.

Physical USB-disconnect testing, valid route-control validation, end-to-end 60 FPS acceptance, and road/vehicle acceptance remain pending. No claim of complete 60 FPS acceptance is made.
