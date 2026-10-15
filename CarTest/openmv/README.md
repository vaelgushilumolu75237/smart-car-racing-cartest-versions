# CarTest OpenMV vision packages

`openmv/` is the only current production package: `cached_width` / `CACHED_WIDTH_BEND_PREVIEW_20261015`, with 18 top-level runtime Python files. Upload all 18 `.py` files from this directory together, then restart the OpenMV interpreter so one build and API set is active. Do not mix package versions. `openmv_clipped_midpoint/` and its references are historical, previously unit-supported context; that variant was not changed or regenerated for this repair and is not a current deployment target.

The historical full package baseline for the preceding ring-guard repair is preserved in `../test-data/ring-ordinary-guard-20261015/before/openmv/`, indexed by `../test-data/ring-ordinary-guard-20261015/before/SNAPSHOT_MANIFEST.json`. The separate ordinary-route parity oracle under `tests/frozen_pre_ring/` was recovered from the v20261014-pre-ring archive and is documented by `tests/frozen_pre_ring/PROVENANCE.json`; it is test-only, not upload runtime. Earlier evidence folders remain unchanged.

Current bend-preview repair details and upload instructions: `../docs/BEND_RECOVERY_20261015_CN.md`. The production bilateral reference is restricted to grid row 40 +/-4; far-only fragments cannot serve as the current road route. The current before-edit snapshot and validation evidence are local under `../test-data/first-bend-20261015/`. Earlier ring evidence below is historical and does not validate this new controller.

## Turn response

Both routes use accepted same-frame rows to calculate one virtual controller center. The lower 70% of the full camera image carries 80% of the aggregate weight; the upper 30% carries 20%. The split is full-resolution y=36, corresponding to grid row 18. Each group is normalized internally, so denser row sampling cannot change the configured 80/20 share.

The grayscale ROI now starts at grid row 8 (full-resolution y=16) and uses four threshold blocks. The new upper block `(0,8,80,20)` sits above the three original lower blocks. Bilateral aggregation includes only rows with current per-row support for both measured boundaries. Single-edge aggregation includes only accepted tracker rows. Unsupported, ambiguous, and inferred bilateral rows are excluded. If one group has no accepted rows, the available group is renormalized to 100% and the missing group is shown in diagnostics.

The weighted center is a virtual aggregate, not a measured boundary point. Boundary gates and the existing geometric safety reference remain in force. For ordinary bilateral/single-edge routes, the UART control value combines this position aggregate with a bounded current-route curve preview, then passes through the existing observation filter. The preview needs three disjoint five-row windows on one supported lower-image chain near grid row 40. A 1.5-cell departure from the near-to-middle tangent must confirm a bend before the heading contribution (gain 2, cap +/-400) is enabled. A diagonal straight therefore receives no curve boost. These are initial software-tested settings, not a measured optimum. The initial EMA alpha is 0.70. Weighting affects steering response; it does not guarantee that path selection will never move to an adjacent lane.

## Routes and protocol

`cached_width` uses the validated bilateral-width cache for one-sided recovery. Only a fully accepted bilateral frame updates that cache. `clipped_midpoint` does not access the cache; it requires a current dark-to-white edge and a known white-road corridor to the opposite frame endpoint. The clipped endpoint is never reported as a measured boundary. Both routes require a fresh reverse RESET/KEEP lease; that lease does not grant motor permission.

The P4/TX to H743 RX observation remains the fixed 12-byte UART v1 packet at 115200 8N1. The separate H743 TX to OpenMV P5/RX control channel remains the 8-byte RESET/KEEP protocol with a 250 ms lease. A stale or missing reverse lease denies lease-dependent single-edge recovery and ring takeover; it does not invalidate an otherwise current, independently accepted bilateral road observation. If the current road route itself is invalid, the observation remains invalid.

The protocol encoder retains the optional flags bit 1 (`SLOPE_HINT`) for compatibility, and four-argument `pack_frame` calls preserve their exact old bytes. Slope classification is disabled in this runtime: `_main_fast` always passes zero, so every emitted packet has the slope bit clear. Invalid frames keep the entire flags byte zero, and receivers reject bits above bit 1. The slope-fitting methods and their isolated host tests remain in `fast_ring_slope.py` for possible future work; the main loop does not call them.

## Ring behavior

Ring detection uses only current known-road/edge masks and per-row measured boundary support. A candidate needs a supported convex inner arc, a measured opposite edge with at least 10 rows spanning 12 grid cells, a connected white-road port, and a bounded path that keeps clearance from dark or unknown pixels. The port may reach a 5% viewport inset (4 grid cells on this 80-cell image); this does not bypass current-white, two-cell-clearance, arc, or uniqueness checks. The route seed is the nearest current supported bilateral row no more than four grid rows from the last supported row that also has known-white and full two-cell clearance. If none qualifies, the candidate is rejected.

The outer-edge shape test has two bounded modes. A straight-outer candidate has a maximum line-fit residual of 1.5 grid cells. A coherent residual shape with correlation at least 0.85 is rejected when the outer residual is at least 0.75 cells, including in that straight-fit band. A curved outer edge is considered only when the measured residual is at most 3.5 cells, the inner residual is at least 6 cells and at least twice the outer residual, their centered residual correlation is nonpositive, the width range is at least 6 cells, and their residual difference reaches 6 cells. These are provisional software-trial criteria, not physical calibration or a unique real-world ring classifier. Perspective and cropped ordinary curves may still mimic the accepted geometry.

Three consecutive frames with fresh KEEP, the same side/port, current candidate geometry, and the already validated normal route confirm entry. Ring identity and steering authorization are separate: every ring override needs a qualified current candidate and current connected path, fresh KEEP, and a valid normal route. If ring geometry disappears, the loop does not force ring steering; identity expires after at most 1000 ms without qualified current geometry. A partial current inner edge cannot extend that lease or authorize steering. The accepted ring path uses the same 80/20 row weighting and existing filter as the ordinary route; port heading is diagnostic only and contributes zero steering. Exit still requires current side-specific outward near-crop inner-edge movement plus a reliable bilateral road and forward continuation for three frames. Ambiguous branches and multiple ports remain invalid. The image crop marker is kept separate from the reachable white port cell.

This repair does not implement the complete eight-phase lost/reappear/second-loss ring sequence. The geometry guard remains experimental until continuous native-camera footage covers ordinary curves and ring entry/exit.

The compact grayscale overlay uses at most four cached detail rows for the RG15 build/source/state, current ring gate and fit mode, inner/outer fit residuals, path rows/span, authorization/current-geometry status and residual correlation, plus raw/filtered steering and effective weights. It draws only current geometry and clears current geometry on invalid frames. `ring` and `row_weight` are separate timing stages. `capture_to_send` ends at UART transmission; preview drawing and flush are measured separately.

## Verification and limits

Run from the `CarTest` directory:

```powershell
python -B .\openmv\tests\test_turn_response_weights.py
python -B .\openmv\tests\test_ring_slope_host.py
python -B .\openmv\tests\test_ordinary_curve_ring_guard.py
python -B .\openmv\tests\test_fast_main_ring_slope.py
python -B .\openmv\tests\test_protocol_host.py
python -B .\openmv\tests\test_singleedge_main.py
python -B .\openmv\tests\test_fast_single_edge.py
python -B .\openmv\tests\test_two_routes.py
python -B .\openmv\tests\run_v7_host_regression.py
python -B .\test-data\ring-ordinary-guard-20261015\validation\run_host_suite.py
```

The current ring/ordinary regression evidence and original failing negative control are in workspace-local `../test-data/ring-ordinary-guard-20261015/`; those evidence paths are not promised in public source checkpoints. The [revised static screenshot report](../test-data/ring-ordinary-guard-20261015/vision/STATIC_SCREENSHOT_REPLAY_INDEPENDENT_CURVATURE_REPORT.md) includes before/after rows for all eight screenshots and both preprocessing variants. Images 3 and 5 produce current geometry previews without route authorization; static screenshots do not establish temporal entry or exit. The pre-ring ordinary-route oracle is frozen under `tests/frozen_pre_ring/` with source provenance and hashes. The main-loop integration harness stubs grayscale mask building, path detection, and boundary validation, while running the actual ring detector, route handoff, packet encoder, and UART writes. It is host evidence, not a full image-to-UART runtime test or board proof. Ring masks and repeated-frame state sequences are synthetic tests, not camera recordings. The eight available photos were static IDE screenshots captured while the H743 was unpowered; they cannot establish a live reverse-control fault or an incline transition. The separate eight-stage ring/speed design document remains future architecture only.

The aggregate host runner retains each log and exit code under `../test-data/ring-ordinary-guard-20261015/validation/host/`; the current run passed 159 tests across nine runners. The 18 current production `.py` files are compiled individually with the bundled OpenMV IDE `mpy-cross` for `armv7em`; logs, exit codes, and `.mpy` output hashes are under `../test-data/ring-ordinary-guard-20261015/validation/compiled_mpy/`. The bundled compiler reports MicroPython v1.25.0 (2025-04-19), emitting mpy v6.3; the board runtime is MicroPython 1.28. This is compiler-front-end evidence, not board-version equivalence or board execution. No `.mpy` files are part of the upload package. No camera upload, board runtime, H743 build or flash, motor start, or vehicle track test is part of this evidence.
