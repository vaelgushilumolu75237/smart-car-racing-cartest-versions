# CarTest OpenMV production package

Current package: `SECOND_TURN_CONTINUITY_20261018`, route `cached_width`. Upload all 20 top-level `.py` files together and restart the interpreter. Do not upload tests or host-compiled `.mpy` files. `PACKAGE_MANIFEST.json` lists the exact runtime hashes. The historical clipped-midpoint variant is not maintained.

The previous MicroPython array-allocation compatibility fix is retained. This release retains RAM diagnostics and paired H743 3000ms last-error hold, repairs single-edge segment selection, and increases the normal FollowTurn output rates. See `../docs/CONTOUR_ARRAYFIX_20261016_CN.md` for the historical allocation issue, and the current steering-hold guide below for this release.

## Steering-chain diagnostics and paired H743 hold

This release changes single-edge recovery and the normal FollowTurn slew/ramp rates. Optional RAM-only diagnostics are enabled by default for this diagnostic release: 128 records of 32 bytes, rolling oldest-first and frozen after the first successfully sent valid-to-invalid transition. Freezing diagnostics never freezes detection or UART transmission. No per-frame print is added; the independent `diag_trace` performance stage accounts for record overhead. Disable `STEERING_DIAG_ENABLED` to avoid allocating the buffer.

After manually stopping the vehicle with PE7, stop the camera script without resetting the interpreter and call `steering_diag_dump()` in the REPL. Save the BEGIN/END text for offline comparison. Power loss, reset, or re-running clears camera RAM. Camera and MCU clocks are independent; do not treat their absolute difference as UART latency.

The paired H743 firmware retains the last trusted received error for at most 3000ms from its receive time. Servo motion may finish toward that existing target through the existing endpoint protection; each motor holds its actual entry duty. Communications faults and manual stop still override the hold. MCU log semantic version is 2 with the same 64-byte layout; the new decoder remains compatible with version 1. See `../docs/AGGRESSIVE_TURN_20261016_CN.md` for paired upload/export and acceptance steps.

## Single-edge segment recovery

Single-edge recovery follows the nearest supported segment from bottom to top; a local bad row no longer vetoes all valid segments. Single-edge-only limits are five rows, six grid rows of span, up to three missing rows and a 16-cell identity-anchor radius. A short segment may use its closest actually supported reference row. Opposite-side recovery needs two consecutive candidates and overlapping accepted centers within 12 cells; no overlap cannot authorize a side switch. True transitions, measured-width provenance, clear road corridors and unresolved branch rejection remain required. Detailed rejection diagnostics use an independent extension and do not change the 32-byte camera trace or 12-byte UART protocol.

## Normal steering

This comparison version disables the ordinary-road preview contribution while
leaving its computation visible. `preview_candidate` is the geometry-qualified
candidate and `heading_contribution`/`preview` is the amount actually sent;
with `ORDINARY_PREVIEW_CONTROL_ENABLED = False`, the latter is 0 and the
ordinary road packet uses only the current weighted position error. This does
not change the existing 80/20 lower/upper image weighting or turn the
controller into a fixed single-row reference. Ring routes retain their
existing position-only behavior.

The original current-route preview gates remain: three disjoint five-row windows, at least 15 rows over 12 grid rows, a supported reference near y=40, and at least 1.5 cells of curvature residual in the heading direction. Heading gain is 2 and the diagnostic candidate is bounded to +/-400. The transmitted position control remains bounded to +/-1000. `preview_candidate` reports the qualified candidate, while `preview`/`heading_contribution` reports the applied amount. The supported row-40 `reference_error` is diagnostic only. No past heading substitutes for missing current geometry.

Accepted centers retain lower-70%-image weight 0.8 and upper-30%-image weight 0.2, with internal normalization per group. The full-image split is y=36, grid y=18. Missing groups are reported and the available group is normalized to 1.0. Initial observation EMA remains 0.70. These are software-trial settings, not measured optimal gains.

## Supplied contour code and current recovery

`contour_trace.py` ports the supplied four-direction left/right wall-following decisions with configurable dimensions, fixed storage and bounded termination. The missing original `get_image_sum()` is replaced by an explicit convention: known white=255, known dark=0, unknown/outside=125. This is an adaptation, not a verified reconstruction of an unavailable complete champion project.

Moving-average filtering, equal-arc resampling, tangent/normal offset and local-angle corner utilities are also provided. Their original C bodies were not supplied; these functions are implementations from the description. They do not establish camera inverse perspective, physical track width or a complete element state machine. Uncalibrated fixed-pixel normal offsets do not control this vehicle.

`contour_route.py` tries the follower after explicit ordinary path misses. Healthy prior paths and rejected prior failures are preserved. Unresolved branches/islands/ambiguities cannot be overridden. The near seed must contain the image center and have two known-dark borders. Accepted rows require unique actual horizontal dark/white transitions, a known-white interior, enough span and continuity, and the existing nearby reference. Clipped or unknown cells are never measured edges. A contour candidate still passes the original boundary/reference validators before selection.

Runtime recovery uses the 80x60 grid, at most 256 points per side and 1024 follower iterations. Its Python backend and execution time are reported separately; existing grayscale/path Viper labels do not imply that this follower is Viper. Board time and heap use remain to be measured.

## Lifecycle and unchanged interfaces

Reliable single-edge tracking still uses validated bilateral width/identity history and fresh H743 RESET/KEEP. Inferred rows never update measured widths. Validated current contour pairs may update the existing bilateral cache through the same gates.

The paired H743 repair restarts RESET/KEEP for each consumed physical PE7 start event and clears stale local KEEP/fresh gates. One start intent remains pending through reset and fresh observations. KEEP without RESET stays rejected by the camera. Invalid route frames do not repeatedly RESET. PE7 remains press once to start, press again to stop.

Ring routing remains experimental with its existing current-geometry and authorization gates. No new ring override, slope classifier, pitch-sensor behavior or finish-line stop is introduced. The missing `find_circle`, `GetExpectSpeed` and `find_zebra` bodies are not presented as reproduced originals. Normal-turn and ring tests remain separate.

Observation protocol remains 12-byte V1, CRC-16/CCITT-FALSE, 115200 8N1; confidence stays diagnostic. Reverse control remains 8-byte RESET/KEEP with a 250ms lease. FollowTurn normal servo slew is 120us/20ms and each motor changes at most 40 permille/10ms. Startup ramps, endpoints, 70/30 maximum differential targets, first complete PWM checks, 3000ms vision-loss cutoff and three-frame recovery are retained. PWM telemetry is not a mechanical angle measurement.

## Validation and delivery

Final hashes, host results, actual ARMv7em compiler results and board-validation limits are in `PACKAGE_MANIFEST.json`. The bundled compiler is older than the reported board runtime; compilation is not runtime equivalence. The real-board Viper equivalence case is skipped without a board-produced manifest.

Run host tests from `CarTest`, including both new `openmv/tests/test_contour_*.py` suites and existing regressions. H743 is built separately with `tools/build.ps1`; camera Python upload and MCU ELF flashing are separate operations.

Source/configuration/ELF snapshots and detailed validation stay under workspace `test-data/`. Actual vehicle logs/media are excluded from public checkpoints. Annotated compressed videos and overlay-masking sensitivity runs are not native sensor captures or closed-loop vehicle acceptance.

See `../docs/CONTOUR_PORT_20261016_CN.md` and `../docs/CONTOUR_REPAIR_20261016_CN.md`. The withdrawn sign-priority controller is historical in `../docs/STEERING_PRIORITY_20261015_CN.md`.
