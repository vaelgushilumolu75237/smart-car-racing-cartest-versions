# Ring and cross observation module

Date: 2026-10-25  
Module: `openmv/track_elements.py`  
Runtime API: `RUNTIME_API_VERSION = 1`

## Scope and evidence boundary

The module adds a bounded temporal ring observer and an isolated straight-cross
route observer. It does not use or revive `fast_ring_slope.py`. Candidate frames
do not change normal selection, steering, or the measured bilateral cache.
Ring targets and confirmed cross routes are returned in separate fixed buffers;
the caller must still pass them through the existing boundary and route
validity chain.

The accepted video evidence supports the ring entry and arc sequence described
below. It does not show a calibrated exit sequence or give usable numerical
thresholds. The module defaults are conservative software guardrails, not
measurements from the video or track. Exit handling therefore remains
experimental, must be checked on the moving vehicle, and may abort to
normal-road reacquisition. No elapsed-time or odometer value guesses that the
car has left a ring.

Host tests use synthetic 80 by 60 masks and row observations. They establish
the state and buffer rules, not camera performance, the actual competition
track geometry, or hardware behavior.

## Per-frame API

Create one reusable `TrackElementsTracker` and call `update()` at most once per
captured frame after the normal path and current mask evidence are available.
The argument order is:

```python
tracker.update(
    now_ms, profile_id, session_active, session_token,
    frame_fresh, road_accepted, road_rows, road_count,
    road_stride, road_candidate_tracks, topology,
    single_rows_fresh, row_status, row_edge2, row_contrast,
    cache_valid, cache_width, cache_left2, cache_right2,
    mask_info, profile_seq=0,
    mean_forward_wheel_millirev=None)
```

`profile_id` is 1=A, 2=B, 3=C; 0 means stale or unavailable. A profile,
session-active, or session-token change resets temporal state. Profile A
disables the ring candidate and state machine; it still maintains the normal
road baseline required by the independent cross observer. The V2
`profile_seq` changes independently and is diagnostic only; it does not reset
the tracker.

`road_rows` is the already selected and boundary-validated normal road result:
an integer array with stride 4 and rows `[y, left_x, right_x, flags]`. `x` is
in grid cells, and measured bilateral rows use flags 7. Pass `road_accepted`
only when that normal source-road result is accepted. The module reads those
rows without changing them.

`row_status`, `row_edge2`, and `row_contrast` are the existing
`FastSingleEdgeTracker` arrays. They are read only when `single_rows_fresh` is
true for this same frame. These are side-major arrays of 120 entries; edge
positions use doubled x coordinates. A selected or inferred single-edge row
is not accepted as a fresh measurement.

The cache arrays are the previously committed bilateral cache. Width uses grid
cells; left and right positions use doubled x coordinates. The tracker reads
the cache, and at ring entry copies an independent frozen width profile. It
never writes inferred geometry to the cache.

`mask_info` supplies fixed 80 by 60 `road_mask`, `edge_mask`, and
`known_region` bytearrays. The ring corridor check reads only target rows. A
Viper sampler scans the mask only on the dedicated `OPEN_BAND` cross path;
this path is considered only when normal road selection failed early, topology
reports `OPEN_BAND`, and no island evidence is present.

The result is one reusable dictionary at `tracker.result`; all row, event, and
counter arrays are preallocated. Main-loop code should keep the returned
dictionary reference and consume counts rather than copying buffers.

## Ring evidence and control output

Left and right use the same mirrored tests. Entry requires:

1. Several accepted, complete, unambiguous bilateral road frames establish a
   stable baseline and measured width profile.
2. One side disappears in fresh raw side-scan rows over multiple frames.
3. The other side remains measured at the same reference row as the baseline
   and within the configured x tolerance.
4. The missing side reappears, moves outward from its baseline by the configured
   minimum, reaches an observed peak, then reverses over multiple frames.

A dropout alone, wheel motion alone, or a moving opposite boundary does not
confirm a ring. Any failure to preserve the required evidence leaves the
candidate unconfirmed or aborts it to `REACQUIRE`.

Once `INSIDE` is confirmed, each target row is built from the locked ring-side
real boundary and the width profile frozen at entry. The other edge is marked
inferred even when a different current edge is visible. The live bilateral
cache cannot move the frozen width. A target row is emitted only with fresh
locked-side support, adequate near-row support, and a known-road corridor from
that boundary to the computed centerline. If a dark island breaks that corridor,
the target is invalid. Island topology is allowed only for a complete,
non-branching confirmed ring with that current measured route support.

`side` remains the ring identity side. `target_side` reports which current real
boundary supports `target_rows`. During `INSIDE`, both sides are the locked ring
side. During `EXIT_CONFIRM` and `EXIT_REPAIR`, they are the opposite side; rows
are rebuilt from that opposite real edge and the same frozen width, with flags
marking the opposite edge observed and the ring side inferred. The route still
needs near support, continuity, and a clear known-road corridor to its
centerline. If the opposite boundary or corridor is missing, `target_valid` and
`control_eligible` stay false. A caller must keep the ring route invalid in that
case instead of silently selecting a normal straight route.

`control_eligible` is true only for profiles B/C in `INSIDE`, `EXIT_CONFIRM`,
or `EXIT_REPAIR` when `target_valid` is true. Profile A never receives ring
takeover. Candidate and reacquire states do not publish a ring control target.

## Exit behavior is experimental

The module does not exit a ring because both boundaries are visible for several
frames. It first requires a fresh, sustained dropout of the locked ring side
while the opposite side remains at baseline. Then it requires multiple
measured opposite-edge frames to publish an experimental exit route using that
real edge and the frozen width. It then requires multiple accepted bilateral
frames near the baseline row, center, and frozen width, followed by another
repair confirmation period. This progresses through `EXIT_CONFIRM` and
`EXIT_REPAIR`; `exit_experimental` labels those states. Only sustained normal
bilateral reacquisition can finish the exit. If opposite-side support, corridor
support, or the later full-lane evidence is absent, the route stays invalid or
the tracker reaches its bounded active-state timeout and aborts to `REACQUIRE`.

The entry states `ENTRY_CONFIRM` and `ENTER` each have a wrap-safe **2,000 ms**
limit (`TRACK_ELEMENTS_MAX_ENTRY_MS`). Confirmed ring states `INSIDE`,
`EXIT_CONFIRM`, and `EXIT_REPAIR` each have a wrap-safe **16,000 ms** limit
(`TRACK_ELEMENTS_MAX_ACTIVE_MS`). These are provisional safety timeouts, not
exit triggers; exceeding them aborts to `REACQUIRE`. The module uses
`time.ticks_diff()` on OpenMV. The host shim uses the same signed 30-bit tick
wrap convention. State frame counts remain diagnostic; the older
`TRACK_ELEMENTS_MAX_ENTRY_FRAMES` and `TRACK_ELEMENTS_MAX_ACTIVE_FRAMES`
settings do not cause aborts.

The cumulative mean forward wheel value is in millirevolutions. The default
`TRACK_ELEMENTS_MAX_ACTIVE_MILLIREV = 60000` is enabled and means a broad
60-revolution abnormal-progress ceiling, measured from the first ring dropout
candidate in the current session token. It is not a distance calibration or a
lap counter. Crossing it only aborts a candidate/active ring to `REACQUIRE`; it
cannot trigger entry, arc confirmation, exit, or a cross route. Set it to zero
only to explicitly disable the guard. `profile_seq` is not used as a
configuration identity.

## Cross route

A cross candidate is considered only when the ordinary path was rejected,
topology says `OPEN_BAND`, topology is complete and non-ambiguous, and no island
or multiple-exit evidence is present. The bounded sampler requires:

- one edge-supported known-road run per sampled row; a second run rejects the
  route;
- one contiguous, symmetric opening relative to the last accepted bilateral
  cache, with expansion on both sides;
- at least five real bilateral rows in each near and far support band, with a
  near/far span of at least six grid rows;
- near and far centers within three grid cells, and widths within the configured
  tolerance;
- every pixel between the interpolated near/far boundaries to remain known road.

No historical rows can create missing far support. The candidate must be seen
for multiple frames before `cross_route_usable` and
`cross_suppress_slowdown` become true. A confirmed route is separate from the
normal result and cache. The main loop must run it through its boundary,
weighting, and validity checks, and must not call `observe_bilateral()` with it.

`cross_route_rows` has stride 4 and rows `[y, left_x, right_x, flags]` in grid
cells. Import these flags rather than hardcoding them:

- `CROSS_ROUTE_MEASURED = 7`: current mask has real bilateral boundary support;
- `CROSS_ROUTE_INTERPOLATED = 2`: row lies on the verified corridor and is
  interpolated; it is not bilateral evidence and must not seed the cache.

The reference fields are `cross_reference_center2`, `cross_reference_y`, and
`cross_lateral_error`. Row counts, span, opening count, opening segment count,
candidate-track count, and a rejection reason remain available for diagnosis.

## States, event log, and settings

States are `OBSERVE`, `ENTRY_CONFIRM`, `ENTER`, `INSIDE`, `EXIT_CONFIRM`,
`EXIT_REPAIR`, and `REACQUIRE`. Per-state frame bounds prevent indefinite
tracking. Timeouts and invalid active frames fail to reacquisition rather than
guessing a route.

`event_log` is a fixed 16-record circular array with `EVENT_STRIDE = 8`. Each
record contains eight signed 32-bit values, in order: frame index, event id,
state id, side id, rejection/event reason code, edge x2, timestamp in ms, and
target center x2. The result dictionary carries profile id, profile sequence,
and session token separately for correlation. `reject_counts` is a fixed
per-reason counter array; neither diagnostic path prints per frame.

Conservative defaults are held in the tracker and can be overridden by the
settings mapping passed to its constructor. Distances below are in grid cells
unless marked x2 or millirev.

| Setting | Default | Use |
|---|---:|---|
| `TRACK_ELEMENTS_MIN_STABLE_FRAMES` | 3 | Bilateral baseline frames |
| `TRACK_ELEMENTS_NEAR_ROWS` | 12 | Near support band height |
| `TRACK_ELEMENTS_MIN_NEAR_SUPPORT` | 5 | Required measured near rows |
| `TRACK_ELEMENTS_MIN_NEAR_CONTIGUOUS` | 4 | Required continuous near rows |
| `TRACK_ELEMENTS_MIN_DROP_FRAMES` | 2 | Required one-side dropout frames |
| `TRACK_ELEMENTS_MIN_SIDE_SUPPORT` | 5 | Required measured rows on surviving side |
| `TRACK_ELEMENTS_MIN_SIDE_CONTIGUOUS` | 4 | Required continuous surviving-side rows |
| `TRACK_ELEMENTS_OPPOSITE_POSITION_TOLERANCE_X2` | 6 | Opposite edge baseline tolerance |
| `TRACK_ELEMENTS_REAPPEAR_SHIFT_X2` | 8 | Minimum outward reappearance |
| `TRACK_ELEMENTS_PEAK_REVERSAL_X2` | 6 | Minimum observed reversal |
| `TRACK_ELEMENTS_PEAK_CONFIRM_FRAMES` | 2 | Reversal debounce |
| `TRACK_ELEMENTS_MAX_ENTRY_MS` | 2000 | Entry-state abort timeout |
| `TRACK_ELEMENTS_MAX_ACTIVE_MS` | 16000 | Each confirmed ring-state abort timeout |
| `TRACK_ELEMENTS_MAX_ENTRY_FRAMES` | 30 | Diagnostic only; no abort |
| `TRACK_ELEMENTS_MAX_ACTIVE_FRAMES` | 180 | Diagnostic only; no abort |
| `TRACK_ELEMENTS_EXIT_DROP_FRAMES` | 3 | Experimental exit dropout debounce |
| `TRACK_ELEMENTS_EXIT_CONFIRM_FRAMES` | 4 | Baseline reappearance frames |
| `TRACK_ELEMENTS_EXIT_REPAIR_FRAMES` | 4 | Additional exit repair frames |
| `TRACK_ELEMENTS_EXIT_WIDTH_TOLERANCE_CELLS` | 6 | Frozen-width exit tolerance |
| `TRACK_ELEMENTS_EXIT_CENTER_TOLERANCE_X2` | 12 | Baseline center exit tolerance |
| `TRACK_ELEMENTS_MAX_ACTIVE_MILLIREV` | 60000 | Enabled 60-revolution abnormal-progress abort |
| `TRACK_ELEMENTS_CROSS_CONFIRM_FRAMES` | 2 | Cross candidate debounce |
| `TRACK_ELEMENTS_CROSS_MIN_OPEN_ROWS` | 3 | Minimum symmetric opening rows |
| `TRACK_ELEMENTS_CROSS_OPEN_MIN_CELLS` | 3 | Minimum symmetric expansion per side |
| `TRACK_ELEMENTS_CROSS_SYMMETRY_TOL_CELLS` | 2 | Side expansion difference tolerance |
| `TRACK_ELEMENTS_CROSS_CENTER_TOLERANCE_X2` | 6 | Opening center baseline tolerance |
| `TRACK_ELEMENTS_CROSS_WIDTH_TOLERANCE_CELLS` | 6 | Near/far path width tolerance |

## Host verification

From the `CarTest` directory:

```powershell
python -m unittest discover -s openmv/tests -p "test_track_elements.py" -v
```

The suite covers mirrored ring evidence, candidate cancellation and timeout,
opposite-edge curve rejection, frozen width behavior, ring-inside non-exit,
experimental exit reobservation, island corridor rejection, session/profile
reset behavior, profile A ring-off with cross still enabled, odometer-only
non-triggering and abort-only use, tick wrap and exact timeout boundaries,
continued tracking at 2.25 seconds, a symmetric cross route, and
asymmetric/ambiguous/adjacent-run cross rejection.

Host execution does not establish OpenMV Viper timing, camera-mask equivalence,
UART-to-main integration, steering stability, or a physical ring exit.
