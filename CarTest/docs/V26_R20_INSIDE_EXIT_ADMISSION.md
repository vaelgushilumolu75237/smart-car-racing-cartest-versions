# v26-r20 independent exit admission (candidate)

The r19 RAM hand push produced 120 verified sampled records and four images.
It entered INSIDE, never reached EXIT_CONFIRM, and exceeded the unchanged
16000 ms active-state limit. Every recorded single-exit reason was INELIGIBLE.
One recorded dropout sample had four missing outer rows but no inner anchor.
These observations expose a coverage/admission gap; they do not establish that
the historical road failures had the same timing cause or provide a full replay.

## Change

The current outer-edge corridor can now be independently assessed in INSIDE.
It requires an established ring identity, completed entry transition, calibration
and parked checks, an actually transmitted inner lane, recent transmitted route,
clear topology, current observed edge and known white corridor. The existing
shape/width/near/reference checks and three consecutive fresh frames remain.
No inner near-anchor or previously observed opposite dropout is required for
this independent path. The original dropout path is retained.

On its third qualifying frame, the tracker sets the exit side from ring identity
and enters EXIT_CONFIRM with EXIT_SINGLE_CORRIDOR_OBSERVED. Current measured
outer geometry guides through EXIT_REPAIR and REACQUIRE. Ordinary bilateral
reacquisition is still required to release the ring identity and frozen width.

The entry lease cannot authorize an exit. After 16000 ms in INSIDE this new path
reports INSIDE_TIMEOUT and cannot restart the clock or restore valid output.
No timeout constants, H743 code, PE7, gains, speed requests, UART/BIN layouts,
pairing gates, 200 ms ramp or 2 s invalid-route hold were changed.
EXIT_INSIDE_API_VERSION=1 and the r20 build identity reject mixed runtime files.

## Software evidence

503 camera regression cases: zero failures/errors, four explicit skips.
Seven new cases start with measured entry/dropout/arc frames rather than seeding
EXIT_CONFIRM. They cover mirrored three-frame admission, no inner anchor,
repair/reacquire and bilateral release, entry lease, no actual write, stale
history, interrupted evidence, branch/unknown corridor, curve and 16 s timeout.
The actual camera route conversion and UART path produce mirrored -190/+190.
The real H743 parser and extracted steering mapping produce 1473/1659 us;
all 11 bridge vectors pass. This is host evidence, not physical servo acceptance.
H743 regression: 72 cases / 7713 assertions, zero failures; exact 2 s hold and
200 ms ramp harness passes. All 15 ELF builds pass and H743 source is unchanged.
The 23 runtime modules are cross-compiled in the delivery build.

An isolated check of the four latest saved images and their row buffers rejects
the pre-timeout curved boundary at sequence 1014 as NOT_STRAIGHT_EXIT (residual
16 half-cells versus the allowed 4). The following two snapshots remain blocked
by INSIDE_TIMEOUT. This deliberately assumes eligible prior ring history to
stress shape rejection; it is not a restoration of the full tracker history.

## Unfinished acceptance

No r20 camera upload, RAM run, H743 flash/reset or motor start was performed.
Full raw-image sequential replay of the exit is unavailable from the sparse
historical evidence. The independent path must still be checked against real
ring curves and the actual exit, then both road directions. Hand-push timing
must be reported separately from driving timing; do not extend production
timeouts just to obtain a passing hand push. Real H750 performance, memory
headroom, ordinary roads, physical steering and actual IDE preview above 60 FPS
remain unverified. Deliver as a candidate only.

Private installation packages include the camera-specific calibration JSON.
Public checkpoints exclude private calibration, captured pictures and logs.
