# OpenMV USB60 candidate package (isolated)

Candidate ID: `OPENMV_USB60_CANDIDATE_20261024`. This package is a local, experimental
candidate built from the coordinated OpenMV 320x120 runtime. It contains the
19-file runtime set for review; this document is not an instruction to upload
it. It has not been uploaded to a camera, and full-pipeline 60 FPS is not
verified.

The source keeps the coordinated PD, route, row-weighting, 16-byte UART V2,
H743 protocol and controller settings. The candidate algorithm optimizations
are the previously tested packed geometry/road overlay main, queue-end
topology, mask-word `fast_gray`, and single-edge skip helper. The main also
counts successful fresh captures with a wrap-safe FPS window. This adds no
diagnostic fields to the H743 packet.

The paired H743 reference ELFs are
`deliverables/H743_Ackermann60_20261022/C_Coordinated/CarTest.elf` and
`deliverables/H743_Ackermann60_20261022/C_Baseline/CarTest.elf`; the candidate
does not change or flash either image. Native preview requires the API-1 camera
system firmware at `deliverables/OpenMV_NativePreview_20261024/firmware.bin`.
The runtime package builder did not upload camera code or flash system firmware.

USB preview is independent of H743 observations. With both firmware capability
markers present (`sensor.PREVIEW_CONTROL_API == 1` and
`csi.PREVIEW_CONTROL_API == 1`), runtime captures with
`sensor.snapshot(preview=False)`. Once processing and UART output finish, the
current 320x120 image may be copied to Q10 JPEG (`copy=True`) and flushed once
only when USB CDC reports connected. A disconnected host skips preview-side
rendering and JPEG work while capture, detection, diagnostics, and UART
continue. Reconnect always publishes the image captured in that loop; no old
image is queued. Compact mode retains the existing per-frame geometry and
status bitmap drawing, adds only one FPS detail string refreshed at the
configured 200 ms interval, and skips allocation of the detail bitmap.

If either capability marker is absent, the main falls back to the existing
`sensor.snapshot()` and `sensor.flush()` path and labels it `LEGACY_FALLBACK`.
That fallback cannot suppress firmware auto-preview. Runtime status labels
distinguish `NATIVE_ACTIVE`, `NATIVE_HEADLESS`, `NATIVE_DISABLED`,
`NATIVE_ERROR`, and `LEGACY_FALLBACK`; the build label says the rate is
unverified.

The intended runtime set is exactly the 19 top-level `.py` files.
`CROSS_COMPILE_SUMMARY.json` records per-file armv7em-sp mpy-cross results; the
compiled `.mpy` files are kept outside this package. `PACKAGE_MANIFEST.json`
binds all 19 source hashes, the paired H743 reference hashes and document
hashes.

Validation so far is 6
USB preview state tests, 5
fresh-capture FPS tests, and 19-module cross-compilation only. It does not prove
camera execution, USB preview delivery, H743 reception, vehicle behavior,
60 FPS, or physical UART throughput. UART framing and baud remain unchanged;
the camera-to-H743 wire-time acceptance still needs a real receive measurement.
See the [UART timing audit](../../docs/OpenMV-USB60-UART-Audit-20261024.md) for
the computed 1.389 ms 16-byte 8N1 serialization floor and the pending receive
measurement. That floor is a calculation, not a board timing measurement.
No camera upload, H743 flash/reset, or motor start was performed.
