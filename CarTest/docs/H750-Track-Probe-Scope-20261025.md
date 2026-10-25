# H750 RAM Track Probe Scope — 2026-10-25

## Purpose

`tools/build_h750_track_probe.py` creates a standalone MicroPython runner for
the staged 20-module camera runtime. It executes the production
`main._main_fast()` pipeline for 35 seconds using real sensor captures at
320x120, while supplying synthetic reverse-control traffic and forcing every
physical UART3 transmission to a CRC-correct V3 frame with `valid=0` and
`profile_ack=0`.

This is a bounded camera performance probe. It is not a production-valid
H743/OpenMV control-chain or vehicle acceptance run.

The runner hashes all 20 staged source files before its measured interval, but
imports only `config`, `main`, `reverse_control`, and `vision_protocol`
explicitly. It calls `main._validate_fast_config()` so production's own lazy
loader selects the active fast modules, then checks `__file__` for the
allowlisted modules actually loaded. It does not eagerly import the unused
legacy runtime only to satisfy the package manifest.

## Scope labels and what they mean

- `real_sensor`: the runner first calls `main._configure_gray_sensor(sensor,
  fast=True)`, then `main._main_fast()` captures through `sensor.snapshot()`
  and calls the factory firmware's `sensor.flush()` after each processed
  frame. It keeps 320x120 and does not call native FIFO or preview APIs that
  are absent from factory H750 firmware 5.0.1.
- `synthetic_reverse`: an independent input wrapper injects V1 RESET/KEEP
  frames and V2 profile-state frames for profile B or C. It uses an active
  session flag, fixed nonzero token 1, zero odometry, 50 ms KEEP cadence, and
  100 ms profile-state cadence with an independent wrapping sequence. This
  exercises camera-side parser and ring-observer behavior with a synthetic
  session.
- `invalid_real_uart`: the output wrapper calls the physical synchronous
  `UART3.write()` but substitutes a separately sequenced V3 frame whose valid
  flag and profile ACK are both zero, with a recomputed CRC. The production
  packet computed by the camera is counted separately from the packet put on
  the wire. The probe therefore cannot provide the H743 a valid vision frame
  through this wrapper. The wrapper also inspects and CRC-checks every
  computed frame, then packs and CRC-checks a second invalid frame. These
  operations run in the frame loop and add probe-specific timing overhead;
  native decorators reduce their cost but do not make the FPS a production
  throughput measurement or justify extrapolation to 60 FPS.

Any bytes already arriving from the physical H743 UART RX path are drained and
counted as discarded. They are never appended to the synthetic parser input.
The script does not flash or upload firmware, reset the H743, or deliberately
start the vehicle.

## RAM and output bounds

The probe preallocates its counters and a four-row performance table. The
production 10-second window callback copies only per-window counters and
histogram-derived stage summaries into this table; no per-frame history or
image/video is retained. At most four windows are reported for the 35-second
run, with the final nominal 10-second window clamped to the actual 35,000 ms
run boundary. Each window has one summary row and one short row per stage,
including its window index. Total counters are split into groups of eight.
Every UTF-8 report line is checked against an 850-byte limit before output.
With the current 29 counters, 17 stages and four windows, the report has at
most 78 CRC-covered lines. Allowing CRLF terminators and the bounded CRC tail,
the full console output is at most 66,554 bytes.

Production main-module print calls and the large summary-string queue are
suppressed during frame processing. The runner polls USB CDC at most once per
second, waits at most 120 seconds for reconnection after the measured run,
then waits one second after CDC reports connected. It prints the bounded batch
with a 50 ms pause between lines. A CRC-32/ISO-HDLC checksum covers every
report line and newline before the final checksum line.
If CDC does not reconnect within the bound, results remain in
`H750_TRACK_PROBE_RESULT`; `h750_track_probe_dump()` can print them after a
later reconnect.

## Build and host checks

Generate the runner from the exact staged runtime directory (19 active base
modules plus `track_elements.py`; inactive `fast_ring_slope.py` is excluded).
Pass the exact POSIX camera import folder explicitly; the generator never
copies or uploads modules:

```powershell
python tools/build_h750_track_probe.py `
  --runtime-root <staged-20-module-directory> `
  --camera-root /flash/track_trial_01 `
  --profile B `
  --output logs/h750-track-probe-20261025/stagedbench/h750-track-probe-B.py
```

Use `--profile C` for the C session fixture. The generator prints the runner
SHA-256 and SHA-256 for each of its 20 source modules. Preserve those values
with the private staged-bench result so the executed source can be identified.
Camera roots must be simple ASCII paths below `/flash`; traversal, backslashes,
and punctuation outside letters, digits, `_`, or `-` are rejected. At runner
startup, before importing the runtime, it hashes every target `.py` file using
`uhashlib.sha256()` and fixed 1,024-byte reads. After import it requires every
runtime module's `__file__` to equal that exact target path. A stale F-root
module, missing file, or changed camera copy stops the probe before timing.

Run host-side generator, protocol fixture, wire-safety and bounded-report
checks with:

```powershell
python -m unittest tests.test_build_h750_track_probe -v
```

These checks verify generated-source syntax and ordering, exact runtime
allowlisting, synthetic V1/V2 scheduling and freshness, discarded real RX,
forced invalid wire bytes and CRC, separation of computed and transmitted
packet counters, four-row capacity, report CRC coverage, and CLI output for
C. Host checks do not execute MicroPython or measure the camera.

## Acceptance boundary

The probe result can describe camera throughput and timing under real sensor
capture with synthetic reverse-session context and invalid-only real UART TX.
It cannot establish a fresh matching production profile ACK, valid H743
reception, full Follow session, motor/servo behavior, road performance, or
successful track completion. Those require a separate reviewed test with
valid production control traffic and the appropriate hardware safety setup.
