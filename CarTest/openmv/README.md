# CarTest OpenMV production package

Current package: `CONTOUR_REPAIR_20261016`, route `cached_width`. Upload all 20 top-level `.py` files together and restart the interpreter. Do not upload tests or host-compiled `.mpy` files. `PACKAGE_MANIFEST.json` lists the exact runtime hashes. The historical clipped-midpoint variant is not maintained.

## Normal steering

This version withdraws `CORR_FIRST` / `CONFLICT_FADE`. Lateral position and the curve ahead can have opposite signs during a normal turn; sign disagreement alone cannot reject a preview. Position -25 and a geometry-qualified preview +330 therefore compose to +305 before filtering. This is a software example, not a prediction of the vehicle trajectory.

The original current-route preview gates remain: three disjoint five-row windows, at least 15 rows over 12 grid rows, a supported reference near y=40, and at least 1.5 cells of curvature residual in the heading direction. Heading gain is 2, preview is bounded to +/-400 and combined control to +/-1000. `preview_candidate` reports the qualified candidate, `preview` the applied amount. The supported row-40 `reference_error` is diagnostic only. No past heading substitutes for missing current geometry.

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

Observation protocol remains 12-byte V1, CRC-16/CCITT-FALSE, 115200 8N1; confidence stays diagnostic. Reverse control remains 8-byte RESET/KEEP with a 250ms lease. Current motor/servo settings, first complete PWM checks, 1000ms vision-loss cutoff and three-frame recovery are retained. PWM telemetry is not a mechanical angle measurement.

## Validation and delivery

Final hashes, host results, actual ARMv7em compiler results and board-validation limits are in `PACKAGE_MANIFEST.json`. The bundled compiler is older than the reported board runtime; compilation is not runtime equivalence. The real-board Viper equivalence case is skipped without a board-produced manifest.

Run host tests from `CarTest`, including both new `openmv/tests/test_contour_*.py` suites and existing regressions. H743 is built separately with `tools/build.ps1`; camera Python upload and MCU ELF flashing are separate operations.

Source/configuration/ELF snapshots and detailed validation stay under workspace `test-data/`. Actual vehicle logs/media are excluded from public checkpoints. Annotated compressed videos and overlay-masking sensitivity runs are not native sensor captures or closed-loop vehicle acceptance.

See `../docs/CONTOUR_PORT_20261016_CN.md` and `../docs/CONTOUR_REPAIR_20261016_CN.md`. The withdrawn sign-priority controller is historical in `../docs/STEERING_PRIORITY_20261015_CN.md`.
