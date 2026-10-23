# OpenMV Ackermann 320x120 package: coordinated

Build ID: `OPENMV_ACKERMANN320_COORDINATED_20261022`  
Runtime selector: `coordinated`  
Preview bound: gain `0.45`, contribution limit `+/-120`  
Speed stage: `enabled`

Upload the 19 top-level `.py` files from this folder together and restart the
OpenMV interpreter. Do not mix files between profiles, or upload the host-only
`.mpy` validation outputs. The package manifest lists every production module
and its source and cross-compile hashes.

Both packages use the same perception and route code. The sensor is configured
for QVGA and captures the explicit center crop `(x=0, y=60, width=320,
height=120)`. This changes the vertical field of view: the 320x240 sensor image
is cropped to its middle 120 rows; it is not resized or stretched. The 80x60
grid uses 4x2 min-pooling and examines every source pixel in the crop. Overlay
coordinates scale independently by x4 and y2.

Row weights are the fixed-point Q1000 table for
`w(y)=0.15 + 0.85*(1-((y-59.5)/59.5)^2)` over original crop rows y=0..119.
Weights peak in the center and taper toward both edges; normalization includes
only supported rows from the selected same-lane chain. Full weighting requires
at least three accepted core rows spanning two grid rows in original y=40..79.
If that core support is absent, the selected chain's nearest measured core row
is used and diagnostics mark `DEGRADED_CORE_SUPPORT`; a disconnected point
cannot take control. Existing route validity and topology checks remain in
force.

Both packages retain the existing PD controller and confirmed preview state
machine. `geometry_perf_only` keeps the prior preview limits (`0.35`, `100`)
and disables the independent speed request, so its V2 phase/request fields are
always zero. `coordinated` uses preview limits (`0.45`, `120`) and enables the
independent speed stage: after three fresh same-direction candidate frames
over at least 60 ms it requests slowdown `1000`; release evidence requires a
valid absolute filtered position below 50 and valid candidate magnitude below
80 for 300 ms. Invalid frames publish phase/request zero but do not clear the
internal speed latch. Only STOP or RESET clears that latch.

Both packages send the same 16-byte UART V2 wire format. Byte 10 is curve
phase (0 straight, 1 candidate, 2 turn, 3 exit), byte 11 is reserved zero,
bytes 12..13 are the little-endian slowdown request in permille, and bytes
14..15 contain CRC-16/CCITT-FALSE over bytes 2..13. Invalid observations keep
their incrementing sequence but clear all observation and control fields.
The existing 32-byte camera diagnostic and control-extension record layouts
remain intact; `curve_trace_v2_hex` stores the actual 16-byte V2 frame.

Validation: all 19 production `.py` modules were cross-compiled for
`armv7emsp` with the compiler recorded in `PACKAGE_MANIFEST.json`. The IDE
archive's mpy-cross is older than the reported board MicroPython runtime, so
this is source compatibility evidence, not proof that the camera accepts the
`.mpy` output or runs the source. Host regression tests also do not prove
hardware timing. Run the current host suite with
`python -m unittest discover -s openmv/tests -p 'test_*.py'`. It ran 242 tests with
3 expected skips. It covers V2 framing/CRC and trace, the 4x2 image
geometry and mask equivalence, path/topology/boundary validation, weighted
reference and degraded core support, steering and speed-stage transitions,
overlay/summary behavior, and package manifests/hashes. The historical
`openmv/tests/run_v7_host_regression.py` uses frozen 160x120/2x2 fixtures and
does not validate this package's 320x120/4x2 geometry. The 60-new-frame/s
full-pipeline target is **not accepted** by this package: final board and
OpenMV IDE preview measurements remain separate required evidence. No camera
upload, H743 flash/reset, or motor start was performed by this builder.
