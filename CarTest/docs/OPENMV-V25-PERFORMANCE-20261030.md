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
Checkpoint-tool tests: 21 passed; one legacy evidence-file test could not run
because OpenMV_NativePreview_UARTFIFO_20261024/EVIDENCE.md is absent in this
workspace. That missing historical fixture was not fabricated or marked passed.

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
| PERF candidate, board processing with actual IDE open | 62.9?63.6 FPS | Seventeen complete active windows, zero virtual write errors |
| PERF candidate, actual IDE displayed new images | 60.8?62.2 FPS | Ten consecutive 10-second windows, 100 seconds total |

IDE measurement uses read-only DXGI desktop duplication with actual QPC/
LastPresentTime timestamps. Each observed Gray24/CRC8 sequence is counted once;
sequence gaps are never converted into displayed frames. There were 6,160 new
images and no CRC errors among 13,661 desktop events. Per-window new-image
counts: 611, 608, 620, 619, 610, 617, 622, 616, 616, 621.
The fixture is a stationary calibrated straight. The user-provided terminal summaries confirm seventeen complete active board
windows at 62.9?63.6 FPS, every captured frame valid and fully written to the
virtual sink. All 11,408 preview frames were published, without native publish
or marker errors. Nine complete board sequence ranges lie inside the screen
recording; they contain 609?623 individually observed new images each.
Physical active-session UART and dynamic straight/ordinary-turn/cross/left-ring/
right-ring performance remain separate acceptance steps. These results do not establish >=60 FPS on every road scene.

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
The first capture was fault=none, wait_switch_release, no start event and PWM
zero. During the isolated IDE performance probe there were no physical camera
frames by design. Subsequent RAM tests used real UART3 and the unmodified 23
production modules. H743 received V3 / ACK3 / ready=1, and camera diagnostics
showed CALIBRATION_READY, parked check passed, observation OK and zero camera
TX/detector errors. Two live reads about 16 seconds apart showed unchanged
parser-error and framing-error counters while accepted frames progressed.
After the operator released PE7, the existing startup gate recovered to
fault=none / wait_start_edge, run_request=0, PWM disabled and CCR2/CCR3 zero.

These physical tests were parked session token 0, not active ring processing
or active FPS acceptance. Restarting the RAM camera after the H743 had already
sent its RESET burst produced KEEP_WITHOUT_RESET; the camera correctly rejected
KEEP alone. Full RESET/KEEP startup synchronization remains pending and is not
bypassed. Parser/framing counters also increased between stop/start transitions;
the exact transition-level cause was not isolated. During each sampled live
interval parser errors were stable (720 in the first run, 1926 in the second).
Do not interpret these cumulative values as continuing line corruption or as a
clean end-to-end startup acceptance.

## Remaining acceptance

The sources remain a candidate. After explicit user approval, all 23 Python
modules plus PACKAGE_MANIFEST.json were written to this camera and verified
by filesystem SHA256 readback. A fresh private backup preserves 105 preceding
files. The original calibration and every other original file were unchanged.
No program was automatically started after the copy. Power-cycle persistence
and normal startup are still pending; the installation is not road acceptance. Ground tests must independently verify ordinary left/
right steering, straight, ordinary curves, cross and both ring directions.
Handheld spinning-wheel results do not replace ground ring entry. Preview
FPS and road behavior have separate acceptance records. Private video, logs
and camera calibration are excluded from public checkpoints.
