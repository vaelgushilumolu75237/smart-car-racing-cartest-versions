# v26-r15 exit reobservation candidate

This candidate includes the r14 inner-first exit guidance and bounded recorder
changes described in `V26_R14_EXIT_INNER_GUIDANCE.md`. Neither r14 nor r15 was
permanently installed during the development/RAM tests described here.

## r14 RAM failure and additional correction

The r14 hand-push did not complete exit handover. Saved originals show an open
white exit gap. Its last admitted route at sequence 743 had reference center2=53.
From 744 the route was invalid. Accepted paired ordinary geometry later returned,
but both inner restoration and outer continuation were rejected against that old
reference; repair remained zero and the tail remained invalid. The r14 GitHub tag
is an immutable software-tested snapshot, not a passed candidate for installation.

r15 adds independent consecutive exit-corridor observations. These require a
current accepted bilateral ordinary route, existing near/topology/calibration and
frozen-width checks, and the registered exit profile. The reference row must have
both actually measured edges. Four consecutive new frames (the configured repair
count), less than 250 ms apart, must agree within the existing handoff displacement
limit. One-sided, guessed, ambiguous, width-inconsistent, jumping or interrupted
evidence cannot establish this reference.

Only this fresh repeated bilateral evidence can authorize an inner route that
could not agree with a stale previous outer reference. The constructed current
inner target must also agree with the measured paired center and still pass all
existing route/corridor checks. The evidence alone does not release the ring lock:
normal consecutive repair and reacquisition still apply. A timed-out exit can retry
only when this measured evidence is present; time alone never authorizes handover.

## Verification

- 463 camera host tests: zero failures/errors, three existing skips; the existing
  private-RAW replay remains explicitly excluded. Tests include four-frame restart,
  counter interruption, time gap, width mismatch, branch, one-sided input and a
  center jump. r14 inner/outer guidance and real main-loop UART tests still pass.
- All 23 production Python modules and the private full-pipeline RAM harness
  cross-compile for MicroPython 1.28 armv7emsp. Final delivery has 101 verified hashes.
- H743 source/ELFs and steering mapping are unchanged from r14. The previously
  executed 72-case/7,713-assertion H743 host suite and the actual mapping of final
  UART errors -89/+114 to 1504/1607 us apply to the unchanged control chain. No new
  H743 target builds or hardware actions are claimed.
- The actual H750 r15 RAM hand-push processed 3,641 frames and retained 157 records
  plus eight CRC-verified images. Repair counts at sequences 924-927 were 1/2/3/4,
  all valid right-inner-edge targets. 927 entered REACQUIRE, 928 retained valid
  inner guidance, and 929 switched to valid ordinary road. The recorded tail
  through 3648 remained valid ordinary road. No physical UART or flash write was
  used. This proves this hand-push sequence, not driven exit success.
- This RAM run used a recent continuous route at its repair transition; it does
  not independently prove recovery from the precise stale-reference r14 failure.
  A separate H750 RAM test held the saved r14 sequence-832 boundary geometry and
  actual calibration with stale reference center2=53. Repeated synthetic 33-ms
  steps rejected continuity on steps 1/2/3 and admitted it on step 4. This tests
  the actual implementation on that geometry, not the missing live frame sequence.
  The check plus continuity evaluation took 1,796-1,963 us, not full-pipeline FPS.

The r15 hand-push still contained invalid intervals of 1,074 and 1,198 ms. They
must be evaluated on the vehicle with the existing two-second hold. They must not
be described as uninterrupted route output or assumed harmless on the road.

Use `TrackProfiles_RingExit_v26r15_FinalCandidate_20261031`; the same-version
`Candidate` directory is an intermediate packaging step. The lead model personally
implemented/reviewed all r15 changes. GPT-6 Luna Max's bounded r14 recorder work
was retained and personally reviewed. Private logs, originals and calibration
remain outside public source checkpoints.

Driven right/left exit, ordinary-road regression and actual IDE-preview 60+ FPS
are pending. Current RAM results do not prove which visual failure caused the
earlier r13 vehicle departure. Permanent Python installation requires the user's
separate confirmation; camera firmware, H743, PE7, motor operation, profile pairing,
speed/gain settings, 200 ms slowdown and two-second hold remain unchanged.
