# v26-r14 measured exit guidance candidate

The r13 vehicle run again departed at the right-ring exit. H743 records show
valid error declining from +552 to +48 before observation 18790 became invalid;
the requested servo pulse approached center before the existing two-second hold
expired. Fault was zero and profile C acknowledgement was valid. The available
camera files do not contain the matching observation interval. Therefore the
exact physical departure cause is not established by the saved camera evidence.

Source inspection found two independent exit defects: EXIT_REPAIR requested the
opposite observed side instead of retaining the preferred inner edge, and it
advanced on one matching frame without using the configured repair count. The
r13 RAM state-transition success did not validate these choices on the road.

## Guidance and handover

INSIDE entry/curve behavior remains experimental and unchanged. EXIT_CONFIRM,
EXIT_REPAIR and guarded REACQUIRE now share inner-edge-first construction. For a
right ring, the measured right inner edge projects the opposite boundary left
by the calibrated per-row width; the center is halfway between the boundaries.
Left-ring geometry mirrors this construction. No steering sign is forced.

If the current inner edge is unavailable, the existing calibrated outer-edge
route can continue in the repair/reacquisition states too, with its existing
visible-white-corridor, known-region, topology, support, age and handoff checks.
Restoring the inner side during exit cannot bypass continuity because its prior
outer reference expired. Unsupported guidance remains invalid.

Repair requires the configured consecutive number of current valid targets,
accepted paired exit geometry and continuity with recent admitted ring guidance.
An interruption resets the count. Guarded REACQUIRE keeps the ring identity,
frozen widths and baseline until its consecutive evidence passes; scanner input,
dark-block handling and final main-loop invalid-target guards remain active.
Only then does ordinary routing regain control. False-entry recovery cannot
bypass these exit stages. Exit timeout or an invalid frame does not release the
lock into an unverified ordinary route. Session/profile reset protection remains.

## Recording

The bounded recorder opens continuation episodes after 256 frames, overlapping
the previous 64 frames. At a real stop, it also stores the newest 64-frame stop
snapshot even when a candidate was recorded earlier. The existing eight-slot
overwrite policy, format 3, record size and allocated buffers remain unchanged.
Long sessions can still overwrite earlier episodes; this is bounded history,
not unlimited recording. No new per-frame flash I/O was introduced.

## Validation and boundaries

- 461 camera host tests passed, with three existing skips and the existing
  private-RAW replay explicitly excluded. Tests cover mirrored inner projection,
  outer fallback, stale/jumping/unknown/branch rejection, repair interruption,
  guarded reacquisition, timeout behavior and final UART validity/direction.
- Real main-loop UART outputs from two mirrored generated exit targets were
  -89 and +114. The actual H743 steering function compiled on the host maps them
  to 1504 and 1607 microseconds around its 1530-microsecond center.
- H743 production host suite: 72 cases, 7,713 assertions, zero failures.
- All 23 production Python modules and the private RAM harness cross-compile for
  MicroPython 1.28 armv7emsp. Final package has 101 verified hashes. The earlier
  intermediate `TrackProfiles_RingExit_v26r14_Candidate_20261031` is superseded;
  use `TrackProfiles_RingExit_v26r14_FinalCandidate_20261031`.
- H743 source and three paired ELFs, v24 PE7, steering mappings and gains,
  speed targets, 200 ms slowdown, two-second hold and camera firmware are unchanged.
  No target ELF rebuild is required or claimed for this camera-only change.

The lead model owns and reviewed route architecture, main integration, tests and
the recorder diff. GPT-6 Luna Max implemented only the bounded recorder changes
and its new tests. Source/configuration/ELF backups and private logs/images remain
outside public checkpoints.

Full r14 RAM exit capture, ground right/left exit behavior, ordinary-road checks
and actual IDE-preview 60+ FPS are pending. No permanent r14 camera write, H743
reset/flash or motor start is authorized by this candidate publication.
