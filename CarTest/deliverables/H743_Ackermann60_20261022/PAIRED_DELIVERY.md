# Ackermann 60 / OpenMV 320 paired software packages

Lead review update (2026-10-24): both delivered camera profiles reached
608 fresh captures in each of three 10-second static straight-track windows
with USB image preview disabled, a RAM UART sink and simulated KEEP. USB
preview remained approximately 35 FPS. No H743 flashing or vehicle validation
was performed. See the [final delivery and test limits](../../docs/ackermann320-final-delivery-20261024.md)
and [aggregate evidence](../../docs/ackermann320-static-evidence-20261024.json).

This package contains the H743 side of two matched software configurations.
Install only the matching OpenMV and H743 pair from the table.

| Pair | OpenMV package and preview tuning | H743 image | H743 preset | Coordination |
| --- | --- | --- | --- | --- |
| Coordinated | `../OpenMV_Ackermann320_20261022/coordinated/`; PD preview `.45/120`, speed stage on | `C_Coordinated/CarTest.elf` | `DebugFollowIncrementalPI` | Enabled |
| Geometry/performance comparison | `../OpenMV_Ackermann320_20261022/geometry_perf_only/`; PD preview `.35/100`, speed stage off | `C_Baseline/CarTest.elf` | `DebugFollowIncrementalPIBaseline` | Disabled |

Both camera packages use the 320 x 120 center crop and center-weighted
steering geometry. Both camera packages retain PD preview. The
`geometry_perf_only` name describes the camera-side geometry/performance
package: it uses the `.35/100`
settings, disables the speed stage, and transmits V2 `curve_phase=0` and
`slowdown_request_permille=0`.
It is paired with the H743 baseline, which keeps the 65:35 legacy steering
mix and 120 us / 20 ms servo slew step. This is not an unchanged-camera control
baseline or a pure performance-only comparison.

The coordinated camera package uses PD preview with `.45/120` settings and the
curve speed request enabled. Its V2 frame carries phase
`0..3` and a slowdown request in `0..1000` permille for the coordinated H743
controller. The UART frame is always the fixed 16-byte V2 format documented in
[`docs/vision-protocol-v2.md`](../../docs/vision-protocol-v2.md). Invalid
vision observations retain their advancing sequence while all observation and
curve fields are zero.

The final H743 artifacts are bound by the manifests in this directory:

- Coordinated: 2,041,352 bytes,
  SHA-256 `6370200c9b843cb1ecb5983b1eb886910a77eb145dff9f51cd119074bbd36c70`.
- Baseline: 2,039,224 bytes,
  SHA-256 `3729bd00e6015f06a106d3c0a54352e5f3dbd0f17a6d129a252bee59912b7655`.

The H743 build summary records all 12 preset results. The paired images were
host-built and their snapshot symbol/section layout was checked. Host tests and
the synthetic HotPlug export checks are software evidence only. No board flash,
camera upload, vehicle run, road result, or frame-rate result is asserted here;
those checks remain for the lead review and later physical validation.

The independent 64-byte `ACK1/v1` block is one coordination snapshot, not a
run timeline. Existing 64-byte RLOG records remain the source for a captured
run CSV. See [`docs/ackermann-snapshot-ack1.md`](../../docs/ackermann-snapshot-ack1.md)
for the snapshot layout and export conditions.
