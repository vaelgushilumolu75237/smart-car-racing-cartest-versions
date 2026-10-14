# CarTest OpenMV vision packages

`openmv/` and `openmv_clipped_midpoint/` are separately uploadable packages with 18 top-level runtime Python files each. Their exact accepted build pairs are `cached_width` / `CACHED_WIDTH_RING_20261014` and `clipped_midpoint` / `CLIPPED_MIDPOINT_RING_20261014`. Upload all 18 files from one directory together; never mix the packages. These IDs distinguish the slope-disabled runtime from the earlier ring-and-slope bundle.

The original full package snapshot for this revision is preserved under `../test-data/turn-response-20261014/before/vision/`, indexed by `BASELINE_MANIFEST.json`. Earlier evidence folders remain unchanged.

## Turn response

Both routes use accepted same-frame rows to calculate one virtual controller center. The lower 70% of the full camera image carries 80% of the aggregate weight; the upper 30% carries 20%. The split is full-resolution y=36, corresponding to grid row 18. Each group is normalized internally, so denser row sampling cannot change the configured 80/20 share.

The grayscale ROI now starts at grid row 8 (full-resolution y=16) and uses four threshold blocks. The new upper block `(0,8,80,20)` sits above the three original lower blocks. Bilateral aggregation includes only rows with current per-row support for both measured boundaries. Single-edge aggregation includes only accepted tracker rows. Unsupported, ambiguous, and inferred bilateral rows are excluded. If one group has no accepted rows, the available group is renormalized to 100% and the missing group is shown in diagnostics.

The weighted center is a virtual aggregate, not a measured boundary point. Boundary gates and the existing geometric safety reference remain in force. The aggregate supplies the raw steering error in the 12-byte UART observation, before the observation filter, for bilateral and either single-edge route. The initial EMA alpha is 0.70. Weighting affects steering response; it does not guarantee that path selection will never move to an adjacent lane.

## Routes and protocol

`cached_width` uses the validated bilateral-width cache for one-sided recovery. Only a fully accepted bilateral frame updates that cache. `clipped_midpoint` does not access the cache; it requires a current dark-to-white edge and a known white-road corridor to the opposite frame endpoint. The clipped endpoint is never reported as a measured boundary. Both routes require a fresh reverse RESET/KEEP lease; that lease does not grant motor permission.

The P4/TX to H743 RX observation remains the fixed 12-byte UART v1 packet at 115200 8N1. The separate H743 TX to OpenMV P5/RX control channel remains the 8-byte RESET/KEEP protocol with a 250 ms lease. A stale or missing reverse lease denies lease-dependent single-edge recovery and ring takeover; it does not invalidate an otherwise current, independently accepted bilateral road observation. If the current road route itself is invalid, the observation remains invalid.

The protocol encoder retains the optional flags bit 1 (`SLOPE_HINT`) for compatibility, and four-argument `pack_frame` calls preserve their exact old bytes. Slope classification is disabled in this runtime: `_main_fast` always passes zero, so every emitted packet has the slope bit clear. Invalid frames keep the entire flags byte zero, and receivers reject bits above bit 1. The slope-fitting methods and their isolated host tests remain in `fast_ring_slope.py` for possible future work; the main loop does not call them.

## Ring behavior

Ring detection uses only current known-road/edge masks and per-row measured boundary support. A candidate needs a supported convex inner arc, continuous opposite boundary, a connected white-road port, and a bounded path that keeps clearance from dark or unknown pixels. The port may reach a 5% viewport inset (4 grid cells on this 80-cell image) to tolerate one-cell crop-side variation; this margin is not a steering weight and does not bypass the current-white, two-cell-clearance, arc, or uniqueness checks. The route seed is the nearest current supported bilateral row no more than four grid rows from the last supported row that also has known-white and full two-cell clearance; if none qualifies, the candidate is rejected. The selected seed coordinate is used for both the ring reference and port heading. Three consecutive fresh KEEP frames confirm entry. A partial current inner edge may retain INSIDE identity, but without a full current arc/port candidate the main loop falls back to its ordinary accepted road/single-edge route; it does not retain ring heading from history. Missing or unknown current boundaries cannot synthesize a valid route. Exit requires side-specific outward near-crop inner-edge movement plus a current reliable bilateral road and forward continuation for three consecutive frames; an inward jump or unknown edge does not confirm exit. Ambiguous branches and multiple ports remain invalid. The image crop marker is kept separate from the reachable white port cell.

The grayscale overlay distinguishes the geometric safety reference from the virtual weighted controller column, displays current near/far row counts and effective weights, and clears current geometry on invalid frames. `ring` and `row_weight` are separate timing stages. `capture_to_send` ends at UART transmission; preview drawing and flush are measured separately.

## Verification and limits

Run from the `CarTest` directory:

```powershell
python -B .\openmv\tests\test_turn_response_weights.py
python -B .\openmv\tests\test_ring_slope_host.py
python -B .\openmv\tests\test_fast_main_ring_slope.py
python -B .\openmv\tests\test_protocol_host.py
python -B .\openmv\tests\test_singleedge_main.py
python -B .\openmv\tests\test_fast_single_edge.py
python -B .\openmv\tests\test_two_routes.py
python -B .\openmv\tests\run_v7_host_regression.py
python -B .\tools\verify_openmv_two_routes.py --output-dir .\test-data\speed-ring-20261014\vision
```

The slope-disabled package evidence and saved logs are under `../test-data/speed-ring-20261014/vision/`; the pre-inset source/evidence snapshot is `../test-data/speed-ring-20261014/vision/pre-inset-port4/`, and the original pre-edit package snapshot is under `../test-data/speed-ring-20261014/before/vision/`. Start at the [vision evidence index](../test-data/speed-ring-20261014/vision/README.md), which links the current report and Chinese screenshot report. The frozen V7 suite is a legacy behavioral compatibility regression against the current runtime; its fixture source hashes remain unchanged and its temporary test-copy adaptations are listed in the report. Ring masks and repeated-frame state sequences are synthetic tests, not camera recordings. The eight available photos are static IDE screenshots with text over the top of the image; they were captured while the H743 was unpowered and cannot establish temporal ring entry, a live reverse-control fault, or an incline transition. The separate [eight-stage ring/speed design document](../test-data/speed-ring-20261014/环岛与速度环方案.md) is future architecture only; that plan is not implemented or accepted by these results.

Both packages are compiled file-by-file with the bundled OpenMV IDE `mpy-cross` 1.25 for `armv7em`. This is compiler evidence, not board execution; the board's MicroPython 1.28 runtime is not compiled or tested here. No camera upload, board runtime, H743 build or flash, motor start, or vehicle track test is part of this evidence.
