# v25 H750 performance candidate ? 2026-10-30

Build identity: `CARTEST_H750_RING_JOINT_V25_PERF_20261030`.
Delivery: `deliverables/TrackProfiles_RingJoint_v25_Perf_20261030`.
This is a camera-only performance change on top of the v25 ring admission,
false-entry recovery and mirrored steering corrections. H743 source, profile
ELFs, v24 PE7, steering mapping and loss protections are unchanged.

## Change and safeguards

The calibrated ordinary-corridor scan dominated the profiled ring-tracker
cost (approximately 1.18 ms per call on this fixture). A bounded Viper kernel
now scans the existing signed-i32 buffers. Geometry, calibration, topology,
near support and admission gates are unchanged. The exact 20% width criterion,
minimum eight rows and twelve-row span are retained. Typed buffers are checked
by owner/length, representation and available type/size metadata. Non-i32
adapters use the previous object-safe implementation. MicroPython builds
without array.typecode are supported. Signed extrema do not overflow the
kernel bounds. The private native-decorator experiment was not promoted.

369 camera regression tests ran: zero failures/errors, three existing skips.
One additional ground replay requiring unavailable private RAW input was
explicitly excluded, not passed. Four kernel tests include 4,000 deterministic
fast/fallback comparisons and buffer, frozen-width and boundary cases.
All 23 production Python modules passed armv7emsp MicroPython cross compilation.
The prior full fifteen H743 builds remain unchanged for this camera-only edit.

## Native APP and recovery

The repaired offline build environment produced the matching native APP twice
from empty build directories. Paired BIN SHA256:
`c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154`.
The user authorized and received only the camera APP update. Readback matched;
bootloader, internal filesystem and complete 32 MiB QSPI were unchanged during
that operation. Original partitions, filesystem, calibration and failure logs
are preserved privately. No vehicle firmware was flashed or reset.
The APP build reference remains the frozen original v25 Python bundle;
the PERF Python package is a separate runtime candidate using the same ABI.

## Measurements and limits

Real H750/MT9V034, 320 x 120, actual sensor frames and complete production
pipeline. RAM probes retain the calibration gate and CRC-checked protocol;
the performance probes provide virtual parked/active C status and zero wheel
counts, with no physical control UART. They do not prove real vehicle pairing.

| Configuration | Observed result | Scope |
| --- | --- | --- |
| Replacement stock APP, original v25, preview off | 54.9?55.4 FPS | Board algorithm windows |
| Replacement stock APP, SDK JPEG | 29.0?29.4 FPS | SDK, not actual IDE |
| Restored native APP, original v25, preview off | 70.1?70.6 FPS | Board algorithm windows |
| Restored native APP, original v25, SDK JPEG | 53.0?53.4 FPS | SDK, not actual IDE |
| PERF candidate, preview off | 77.0?77.6 FPS | Eleven complete active windows |
| PERF candidate, actual IDE displayed new images | 60.8?62.2 FPS | Ten consecutive 10-second windows, 100 seconds total |

IDE measurement uses read-only DXGI desktop duplication with actual QPC/
LastPresentTime timestamps. Each observed Gray24/CRC8 sequence is counted once;
sequence gaps are never converted into displayed frames. There were 6,160 new
images and no CRC errors among 13,661 desktop events. Per-window new-image
counts: 611, 608, 620, 619, 610, 617, 622, 616, 616, 621.
The fixture is a stationary calibrated straight. The corresponding board
window summaries still need collection, and physical UART and dynamic
straight/ordinary-turn/cross/left-ring/right-ring performance remain separate
acceptance steps. These results do not establish >=60 FPS on every road scene.

Earlier gdigrab screen capture requested 120 samples/s but delivered about
85 samples/s. Its analysis was corrected to Matroska presentation timestamps;
nominal capture rate was never accepted as the timing basis. Previous small
marker scale/decoder attempts are retained privately and are not acceptance
records. The passing measurement uses the production Gray marker at the
actual 1.6 display scale and DXGI event timestamps.

## Current vehicle diagnostic

Read-only ST-Link HOTPLUG at 100 kHz now connects to the H743. Its flash
descriptor matches C build ID `f9c8d0412652ba88045f8aedda099f08` and paired ELF
SHA256 `9e0ab9033746f5fd885a2a8136e4c78e323b7f44129969f2a51a823d33048076`.
The captured state is fault=none, wait_switch_release, PE7 low, no start event,
PWM disabled and CCR2/CCR3 zero. Profile state C is being transmitted. During
the isolated IDE performance probe the H743 receives no camera frames, as
expected because that probe does not connect physical UART. Real UART pairing
must be checked independently; no gate is bypassed.

## Remaining acceptance

The sources remain a candidate. Permanent Python installation has not been
performed in this run. Ground tests must independently verify ordinary left/
right steering, straight, ordinary curves, cross and both ring directions.
Handheld spinning-wheel results do not replace ground ring entry. Preview
FPS and road behavior have separate acceptance records. Private video, logs
and camera calibration are excluded from public checkpoints.
