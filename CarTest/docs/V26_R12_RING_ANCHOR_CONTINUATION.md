# v26-r12 measured ring anchor continuation candidate

The r11 right-ring road run stopped after an unrecovered invalid route. H743's last valid observation was sequence 8565; sequence 8566 entered its existing vision hold and PWM reached zero at the existing two-second limit. Fault remained zero and C-profile acknowledgement was valid. The matching camera session-1 capture records 26 outer target rows at 8565, then only three observed near rows at 8566/8567, rejected by MIN_ROWS. All three frames have complete, unambiguous topology. Their saved images still show the curved boundary. Thus the r11 leading-dark resolver is no longer the rejection in this capture.

Code review found that ordinary accepted single-edge packets advance their observed identity anchors, while successfully transmitted ring packets do not. Ring scans extend beyond the straight anchor radius only after four closer rows are already accepted. Losing the fourth anchored seed can consequently reject the entire farther continuous curve. This is a temporal reference maintenance defect; reducing the required final target rows does not address it.

The actual three raw images and exact private calibration reproduce 26 -> 3 -> 3 observed rows with a reconstructed straight reference at center2=78. The precise on-road per-row anchors were not saved, so this is an evidence-consistent reconstruction, not recovery of the complete actual scanner history. With bounded accepted-observation commits, the same reconstruction retains 26 -> 26 -> 26 observed rows and valid host targets centered at 136/138/138. Synthetic mirrored sequences reproduce the same fourth-seed cliff and remain supported through successive curve motion after commits.

## Implementation

`main.py` calls `FastSingleEdgeTracker.commit_ring_observation` only after a full valid UART write, fresh reverse control, and final ring route selection. The method verifies the complete selected route before updating anything: a current completed scan, cached-width mode, signed 32-bit producer buffer, ordered bounded rows, valid observed side, existing width cache, unambiguous current status and exact equality to the current measured edge. At least the existing minimum support must match. Inferred entry bridge rows never update anchors.

The Viper kernel advances each actually selected observed anchor by at most two half-cell units per frame, matching the existing ordinary single-edge smoother. It leaves the opposite unobserved side, bilateral baseline positions, width cache, calibrated frozen widths, ordinary continuity history and scanner gates unchanged. No current image boundary, white corridor, topology, calibration, target support or profile acknowledgement check is bypassed. Ring observation API 3 and the r12 build identity reject mixed Python packages.

The H743 source, three paired ELFs, PE7, steering mapping/gains, target wheel speeds, 200 ms slowdown and two-second vision hold are unchanged. Camera firmware/native APP is unchanged.

## Validation and limits

- 445 host tests: zero failures/errors, three existing skips. One separate existing replay needing an unavailable private RAW is explicitly excluded.
- Tests cover both mirrored temporal seed cliffs, bounded updates, frozen baseline/widths, opposite-side isolation, inferred bridges, malformed/stale/short/forged routes, unknown-row rejection and ordinary scanner bounds. The actual main UART path commits only full valid ring writes, never short writes, UART errors or invalid ring targets. Existing mirrored final UART/steering regressions pass.
- All 23 runtime files cross-compile for MicroPython 1.28 armv7emsp. Final candidate delivery has 101 verified hashes and three H743 ELFs byte-identical to r11. Earlier r12 draft delivery predates the Viper optimization and is superseded by `TrackProfiles_RingAnchor_v26r12_FinalCandidate_20261031`.
- On the actual H750 native scanner in RAM, the same private originals and reconstructed anchors yield 26/3/3 without commit and 26/26/26 with commit. The final commit method took 147-159 microseconds for 26 rows in this probe. This is isolated added-step timing, not full pipeline FPS, live input validation, IDE preview FPS or road acceptance.
- RAM probes did not write camera files or connect physical control UART. No automatic H743 reset/flash, Python installation or motor start was performed. The camera remains permanently on r11 pending separate authorization.

The main model reviewed the implementation and final UART admission. No delegate was used. Source/configuration/paired ELF preservation and detailed actual evidence remain in private local diagnostic storage. Private images, logs and calibration are excluded from publication. Right-ring ground completion, left-ring ground behavior, ordinary-road checks and 60+ FPS including actual IDE preview remain pending. This is a candidate, not a passed road release.
