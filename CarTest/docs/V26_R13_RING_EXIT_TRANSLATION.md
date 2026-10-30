# v26-r13 exit translation candidate

The r12 right-ring vehicle run completed the ring and departed at the exit.
The original H743 active RAM salvage and subsequent frozen log were preserved.
The matching camera episode ends 35 frames before the final valid observation,
so it does not identify the exact final exit failure.

A later RAM-only ground hand-push captured 3,817 processed frames over 120 seconds,
152 retained records and seven CRC-verified images. It reached INSIDE, then
EXIT_CONFIRM, but never EXIT_REPAIR. It recovered through STATE_TIMEOUT about
16 seconds after entering EXIT_CONFIRM. Accepted bilateral exit-straight records
still failed the absolute entry-baseline fit: both observed edges moved together
by approximately 12-20 half-cell units (6-10 full-resolution pixels). This proves
a reproducible exit-confirmation defect, not recovery of the missing road frames.
The dropout transition also occurs before the full hand-pushed circuit is complete;
its timing and actual driven exit selection remain subjects of road validation.

## Change

`track_elements.py` retains the existing absolute fit and adds a registered
bilateral corridor check only in EXIT_CONFIRM, EXIT_REPAIR and REACQUIRE, after
the existing opposite-side dropout count. The entry baseline remains frozen.
The current accepted ordinary route must pass existing topology, near support,
calibration and width-corridor checks. Both near sides must agree on displacement
within two half-cell units on average. A shared full-profile displacement must
agree with the near estimate within four half-cell units. After registration,
at least 80 percent of paired rows must fit both entry profiles within four
half-cell units, including eight consecutive rows over a span of twelve rows.

The existing consecutive confirmation, repair and reacquisition states remain.
Mere bilateral visibility in INSIDE cannot invoke this check. It cannot authorize
an unaccepted ordinary route, change error signs, infer a missing edge, or bypass
profile pairing. `main.py` requires EXIT_TRANSLATION_API_VERSION=1 and the r13
package identity to reject mixed installations.

H743 sources and paired A/B/C ELFs, v24 PE7, steering mappings/gains, speeds,
200 ms slowdown, two-second vision hold and camera firmware are unchanged.
The earlier `TrackProfiles_RingExit_v26r13_Candidate_20261031` directory is a
superseded intermediate package. Use only
`TrackProfiles_RingExit_v26r13_FinalCandidate_20261031`.

## Validation

- 449 camera host tests: zero failures/errors, three existing skips. The existing
  replay requiring unavailable private RAW remains explicitly excluded.
- New tests cover both ring sides and both displacement directions, continuous
  confirmation and interruption reset, frozen baselines/widths, single-side drift,
  unaccepted/ambiguous/curved routes, and bilateral visibility without dropout.
- Existing full camera UART and steering-direction regressions remain passing.
- H743 production host tests built with the existing Zig fallback: 72 cases,
  7,713 assertions, zero failures. The MSVC runner could not discover its compiler;
  the failure log is retained. H743 target ELFs were not rebuilt for this camera-only
  change; all three hashes match r12.
- All 23 Python runtime modules cross-compile using MicroPython 1.28 armv7emsp.
  The final delivery contains 101 verified hash entries.
- Five saved current boundary records were tested on H750 in RAM using the
  actual saved camera calibration and the production width-corridor check. Old
  absolute fitting rejected all five; registered fitting accepted all five.
  Each additional check took 2,270-2,303 microseconds. These are sampled records,
  not five consecutive frames, and do not prove the complete exit transition.

No permanent r13 camera write, H743 reset/flash or motor start occurred. Private
logs, images and calibration remain outside public checkpoints. The lead model
personally reviewed the implementation; no delegate was used. Full r13 RAM
exit sequencing, ground exit behavior, left-ring behavior and actual IDE-preview
60+ FPS remain pending. This is a software-tested candidate.
