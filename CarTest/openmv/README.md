# OpenMV USB60 FIFO wrapper package

Package: `OPENMV_USB60_FIFO_20261025`  
Unchanged algorithm build ID: `OPENMV_ACKERMANN320_COORDINATED_20261022`  
Unchanged runtime build ID: `OPENMV_STATIC_VISION_PREVIEW_CANDIDATE_20261024`  
Algorithm profile and config selector: `coordinated`

## Runtime files

`PACKAGE_MANIFEST.json` allowlists exactly 19 runtime `.py` modules. Upload those 19 files as a source package. `fast_ring_slope.py` remains in the repository for legacy host-test imports and is not part of the runtime allowlist. Firmware installation is a separate operation; this package does not upload files to the camera root or startup directory.

The promotion changes only the UART writer adapter and transport status in `main.py`. UART framing, packet bytes, scheduler reserve, route detection, steering parameters, and configuration remain unchanged. The promoted source SHA-256 is `044416e9161ba824bc9037ed7f40260003bfe942905d02106721a41b6cc26b06`.

## UART FIFO behavior and meaning of counters

When the UART advertises integer `WRITE_FIFO_API == 1` and both `write_fifo` and `fifo_txdone` are callable, the wrapper calls `write_fifo(packet)` exactly once. Its return value is passed through unchanged. Busy, partial, or exceptional FIFO operations are not retried and do not fall back to synchronous `write`. Fixture mode continues to use its original sink directly and reports `UART_TX_MODE=SINK`.

`QUEUED_16`, `full_write` counters, and successful-write counts mean the FIFO API accepted all 16 bytes. Those counters alone do not prove that the bits have left the UART pin or that H743 received or accepted the frame. `fifo_txdone()` is a separate hardware completion query and is not polled in the per-frame path. `send_end_us`, `capture_to_send_us`, the `send_ms` diagnostic timestamp, and related timings are captured when the write API returns; they are not wire-completion timestamps. A firmware image without FIFO API 1 uses the existing one-call synchronous `uart.write` path and may run below 60 FPS.

## Firmware and H743 pairing

The paired camera firmware is `deliverables/OpenMV_NativePreview_UARTFIFO_20261024/firmware.bin`, SHA-256 `0bbf81a534f99365a62187b3116559e313527e0aa1b7d585e618022b11feffc3`. Its declared markers are preview-control API 1 and UART FIFO API 1. The full camera firmware readback hash was reported as matching this image. Firmware is not included in the 19-file source upload directory.

The paired H743 reference is `deliverables/H743_Ackermann60_20261022/C_Coordinated/CarTest.elf`, SHA-256 `6370200c9b843cb1ecb5983b1eb886910a77eb145dff9f51cd119074bbd36c70`; the preserved baseline ELF is `C_Baseline/CarTest.elf`, SHA-256 `3729bd00e6015f06a106d3c0a54352e5f3dbd0f17a6d129a252bee59912b7655`. The reported read-only full-image comparison matched the running H743 image to C_Coordinated. This promotion did not rebuild, flash, or reset the H743.

## Host and compile validation

- `python -B -m unittest discover -s openmv/tests -p 'test_*.py' -v`: 251 tests run, 248 passed, 3 skipped, 0 failed.
- `python -B -m unittest -v openmv.tests.test_uart_fifo_wrapper`: 9 FIFO wrapper tests passed.
- All 19 runtime files compiled with the local OpenMV IDE `mpy-cross`, target `armv7emsp`, MicroPython v1.25.0 (2025-04-19), exit code 0 for every module. See `CROSS_COMPILE_SUMMARY.json` for source and output hashes.

## 65-second invalid-only FIFO run and IDE payload-read measurement

For the latest v6 run, the operator reported six complete board windows at 60.7, 60.8, 60.7, 60.8, 60.6, and 60.8 FPS. The camera computed valid routes in all six windows: 607, 608, 607, 608, 606, and 608 frames. The test sink deliberately discarded each algorithm packet and transmitted only the fixed invalid frame, so `wire_valid=0`; UART errors were zero. The FIFO accepted 3,949 frames; busy, partial, and error counts were all zero, and `fifo_txdone()` was true before and after.

The actual IDE terminal payload-read measurement completed six conservative 10-second windows at 60.5142647373, 60.2316303329, 60.2352603627, 60.1152914784, 60.3861933008, and 60.1489975176 reads/second. It counted 3,986 advertised and read payloads; all sizes matched, with zero read errors. This metric counts complete `Frame Data Size` payload-read records using each `read_begin`/`read_end` interval. It is not independent per-frame JPEG decode, marker-freshness verification, or a measurement of physical screen refresh.

A separate SDK marker decode measured six windows from 60.2 to 60.8 FPS, with three missed frames, no duplicates, no reordering, and zero CRC errors. This is independent host evidence, not IDE per-frame decode evidence. The camera computed valid route candidates, but the wrapper sink replaced their UART payload with the fixed invalid frame; no valid route/control frame was transmitted to H743. This does not establish end-to-end control, road/vehicle, or full-scenario 60 FPS acceptance.

## 65-second CDC-closed FIFO headless run

A separate 65-second headless run completed six full 10-second windows. The camera computed valid routes, captured frames, and reported accepted frames of 794, 794, 792, 794, 792, and 793 per window, at 79.4, 79.4, 79.2, 79.4, 79.2, and 79.3 FPS. The runner reported `NATIVE_HEADLESS`, no JPEG marker writes, zero preview errors, and zero buffer overflow. Its upper p95 total-cycle value was 13,000 us in every window; the maximum observed total cycle was 30,759 us.

The operator reported the USB CDC serial session closed for 68.00 seconds during this run; the physical USB cable remained connected. The captured `report.json` records the 65-second capture scope and says cable state was not instrumented. This is a CDC-session-closed test, not a physical USB-disconnect test.

The FIFO summary recorded API v1 `FIFO_QUEUED`, 5,156 attempts and accepts, zero busy/partial/error results, and `fifo_txdone()` true in the before/after samples. The sink submitted a fixed 16-byte V2 invalid vision frame to the FIFO for UART transmission toward H743 (`seq=0`, `valid=0`, and all route/control fields zero). `computed_route_forwarded=0` means the computed valid route packets were replaced by this invalid V2 transmit frame; `wire_valid=0` and `wire_seq=0` describe that outbound frame. `synthetic_RESET_KEEP` is separate reverse-direction test input supplied by the sink's `any()`/`readinto()` methods to OpenMV; it was not sent over the physical UART. FIFO acceptance is not line completion.

A separate read-only SWD snapshot during the headless run recorded 79 invalid-frame observations and 79 sequence rejects in a 1,000 ms window, with a valid snapshot CRC and zero parser errors. The earlier invalid-only wrapper run recorded 60 invalid frames and 60 sequence rejects. These measurements confirm H743 parsed and rejected the invalid test frames; they do not demonstrate H743 acceptance of a valid route or control output.

The successful report and its captured 2,271-byte stdout are workspace-local evidence at `logs/usb-preview-20261024/uart-fifo-headless-20261025/session-1792870049252764000/report.json` and `logs/usb-preview-20261024/uart-fifo-headless-20261025/session-1792870049252764000/reconnected-session-stdout.txt`. The SWD snapshot is at `logs/usb-preview-20261024/resume-20261025-during-headless-fifo/20261025-031227-330-8828/run-status.json`. An earlier failed attempt remains separately recorded at `logs/usb-preview-20261024/uart-fifo-headless-20261025/session-1792869125535625200/failure.json` and is not counted as a successful run. The older 79.5-79.7 FPS headless result was a valid-route source-pipeline run with a no-op UART sink and no JPEG marker writes; it is historical evidence and not FIFO throughput evidence.

## Validation boundary

This package records bounded static invalid-only FIFO, CDC-closed headless, and IDE payload-read measurements. The headless run measured valid route computation but transmitted only a fixed invalid V2 frame; H743 parsed and rejected invalid frames in the separate SWD snapshots, while acceptance of valid routes and control behavior remain unverified. These runs do not establish road/vehicle acceptance, per-frame IDE JPEG decoding, physical display refresh, full-pipeline 60 FPS, or all-scene acceptance. `QUEUED_16` continues to mean FIFO API acceptance only, not wire completion.
