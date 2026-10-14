# CarTest OpenMV vision packages

`openmv/` and `openmv_clipped_midpoint/` are separately uploadable packages with 17 top-level runtime Python files each. Their exact accepted build pairs are `cached_width` / `CACHED_WIDTH_TURNRESPONSE_20261014` and `clipped_midpoint` / `CLIPPED_MIDPOINT_TURNRESPONSE_20261014`. Upload all 17 files from one directory together; never mix the packages.

The original full package snapshot for this revision is preserved under `../test-data/turn-response-20261014/before/vision/`, indexed by `BASELINE_MANIFEST.json`. Earlier evidence folders remain unchanged.

## Turn response

Both routes use accepted same-frame rows to calculate one virtual controller center. The lower 70% of the full camera image carries 80% of the aggregate weight; the upper 30% carries 20%. The split is full-resolution y=36, corresponding to grid row 18. Each group is normalized internally, so denser row sampling cannot change the configured 80/20 share.

The grayscale ROI now starts at grid row 8 (full-resolution y=16) and uses four threshold blocks. The new upper block `(0,8,80,20)` sits above the three original lower blocks. Bilateral aggregation includes only rows with current per-row support for both measured boundaries. Single-edge aggregation includes only accepted tracker rows. Unsupported, ambiguous, and inferred bilateral rows are excluded. If one group has no accepted rows, the available group is renormalized to 100% and the missing group is shown in diagnostics.

The weighted center is a virtual aggregate, not a measured boundary point. Boundary gates and the existing geometric safety reference remain in force. The aggregate supplies the raw steering error in the 12-byte UART observation, before the observation filter, for bilateral and either single-edge route. The initial EMA alpha is 0.70. Weighting affects steering response; it does not guarantee that path selection will never move to an adjacent lane.

## Routes and protocol

`cached_width` uses the validated bilateral-width cache for one-sided recovery. Only a fully accepted bilateral frame updates that cache. `clipped_midpoint` does not access the cache; it requires a current dark-to-white edge and a known white-road corridor to the opposite frame endpoint. The clipped endpoint is never reported as a measured boundary. Both routes require a fresh reverse RESET/KEEP lease; that lease does not grant motor permission.

The P4/TX to H743 RX observation remains the fixed 12-byte UART v1 packet at 115200 8N1. The separate H743 TX to OpenMV P5/RX control channel remains the 8-byte RESET/KEEP protocol with a 250 ms lease. Invalid, stale, or missing input keeps the observation invalid.

The grayscale overlay distinguishes the geometric safety reference from the virtual weighted controller column, displays current near/far row counts and effective weights, and clears current geometry on invalid frames. `row_weight` is a separate timing stage. `capture_to_send` ends at UART transmission; preview drawing and flush are measured separately.

## Verification and limits

Run from the `CarTest` directory:

```powershell
python -B .\openmv\tests\test_turn_response_weights.py
python -B .\openmv\tests\test_singleedge_main.py
python -B .\openmv\tests\test_fast_single_edge.py
python -B .\openmv\tests\test_two_routes.py
python -B .\openmv\tests\run_v7_host_regression.py
python -B .\tools\verify_openmv_two_routes.py
```

The 2026-10-14 evidence and saved logs are under `../test-data/turn-response-20261014/vision/`. The frozen V7 suite is a legacy behavioral compatibility regression against the current runtime; its fixture source hashes remain unchanged and its temporary test-copy adaptations are listed in the report. `test_turn_response_weights.py` and `test_singleedge_main.py` exercise the current weighted configuration directly.

Both packages are compiled file-by-file with the bundled OpenMV IDE `mpy-cross` 1.25 for `armv7em`. This is compiler evidence, not board execution; the board's MicroPython 1.28 runtime is not compiled or tested here. No camera upload, board runtime, H743 build or flash, motor start, or vehicle track test is part of this evidence.
