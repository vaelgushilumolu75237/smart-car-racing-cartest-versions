# CarTest OpenMV vision packages

Two separately generated V7 packages share the same 17 runtime source files. The formal `openmv/` package selects `cached_width` by default with build ID `CARTEST_OPENMV_CACHED_WIDTH_DIFFTRIAL_20261013`. The independently generated `openmv_clipped_midpoint/` package selects `clipped_midpoint` with build ID `CARTEST_OPENMV_CLIPPED_MIDPOINT_DIFFTRIAL_20261013`. The validator accepts only these exact route/build pairs. Do not mix files between packages.

Each package manifest records all runtime source hashes, API versions, route semantics, host evidence, and compile evidence. The full pre-change project snapshot for this differential trial is under `../test-data/differential-20261013/before/`, indexed by `BASELINE_MANIFEST.json` (433 files and five existing ELF builds). The prior two-route synthetic evidence remains under `../test-data/vision-two-routes-20261013/`.

## Upload one package

Choose one directory and copy its 17 top-level `.py` files together to the OpenMV filesystem root. `openmv/` is the formal default. Upload `openmv_clipped_midpoint/` only when intentionally selecting the clipped route. Do not upload a mixture, either directory's `tests/`, host tools, manifests, or generated `.mpy` compile artifacts.

## Shared protocol and control lease

The P4/TX → H743 RX observation frame remains the same 12-byte UART v1 packet, including its CRC, sequence, validity and diagnostic-only quality field. The independent H743 TX → OpenMV P5/RX control channel remains the 8-byte `A6 5A 01 CMD seqLE16 CRC16LE` protocol with CRC-16/CCITT-FALSE. Both single-edge routes require a fresh RESET/KEEP session; a missing or stale KEEP lease still invalidates single-edge output after 250 ms. RESET/KEEP never grants motor permission.

## Route behavior

`cached_width` keeps the formal route's validated bilateral-width model. It falls back to a current-frame measured dark-to-white edge and derives the center from the previously accepted bilateral width. Only a valid, fully written bilateral observation updates the width cache.

`clipped_midpoint` does not read or update width-cache contents. It keeps the real bilateral path unchanged. For one-sided recovery, it requires a current-frame dark-to-white boundary, road-white pixels on the inside, and a fully known white-road corridor from that boundary through the opposite image endpoint. For a left edge it computes `(measured left edge + right frame endpoint) / 2`; for a right edge it mirrors the calculation to the left endpoint. The stored midpoint uses half-cell precision, with at most 0.25 cell rounding error. The image endpoint is reported as a clipped endpoint, never as a measured boundary or `OBSERVED_BOTH` evidence.

Both routes reuse the single-edge support, span, continuity, and ambiguity checks. Clipped midpoint rejects internal islands, branches, ambiguous topology, multiple plausible boundaries, unknown or non-white corridor pixels, and a side switch until a valid bilateral observation reacquires the track. The clipped route does not infer boundary identity merely because a dark component touches the ROI or frame border.

The runtime log identifies the route, fallback/rejection reason, measured left/right boundaries, and current controller reference. For a valid clipped single edge it also identifies the opposite clipped endpoint and midpoint. The grayscale overlay marks measured edges and clipped endpoints on at most 13 evenly sampled rows, including the near and far endpoints. It links the selected centerline with at most 12 segments. Bilateral frames draw both current boundaries and the selected path centerline. Invalid frames clear all current geometry; a `NOW INVALID` label identifies current validity even while the `D100` detail strings wait for their next refresh.

After each new capture, V7 draws that frame's current valid geometry after detection and the observation UART send, then calls `sensor.flush()` once so the IDE preview receives the same overlaid image. Geometry and the current validity/route label are drawn every frame. Compact `D100` detail strings refresh at most every 100 ms; `OVERLAY_PERIOD_MS` controls that text cache only. `V7_PERF_STAGE overlay` includes cache refresh formatting and per-frame drawing; `preview_flush` records flush-copy time separately. `capture_to_send`/`age_send_ms` stop at the UART send and do not include preview copying.

## Host verification and limits

Run from the `CarTest` directory:

```powershell
python -B .\openmv\tests\test_protocol_host.py
python -B .\openmv\tests\test_reverse_control.py
python -B .\openmv\tests\test_fast_single_edge.py
python -B .\openmv\tests\test_singleedge_main.py
python -B .\openmv\tests\test_two_routes.py
python -B .\openmv\tests\test_v7_overlay_each_frame.py
python -B .\openmv\tests\test_v7_overlay_accounting.py
python -B .\openmv\tests\run_v7_host_regression.py
```

The repeatable evidence runner writes this revision under `../test-data/differential-20261013/vision/final/` and summarizes it in `../test-data/differential-20261013/vision/REPORT.md` and `REPORT.json`; prior logs remain alongside it. Both source sets are compiled with the bundled OpenMV IDE mpy-cross 1.25 using `-march=armv7em`; the requested 1.28 compiler target is recorded separately if that exact compiler is unavailable. The previously reported 383 frames/10 s is a historical camera result and is not a measurement of this overlay revision. The 450 frames/10 s target remains unmeasured. This revision has host/compiler evidence only: no camera frames or board runtime were tested, and no upload, H743 build/flash, or vehicle run was performed.
