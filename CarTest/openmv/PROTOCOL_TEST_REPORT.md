# v26-r24 independent ring-exit candidate — 2026-11-01

r23 rejected a qualified current exit edge when the ordinary detector also
accepted a road, until exit ownership had already been latched. r24 removes
that conflict for right-ring exit search after established departure, provided
the current raw boundaries and ordinary observation were retained separately.
It uses the same calibrated exit-center geometry and final UART command path.

This change does not establish that every previous road failure had this cause.
It does not change left-ring admission, steering signs, speed gains, PE7,
H743 source, pairing, faults, 3000 ms C hold or 300 ms C ramp-down.
Current edge quality, topology, near/reference support and deadlines still apply.
The debug line now includes ordinary_exit=1 when coexistence admission is used;
this field alone does not mean the final route is valid or was transmitted.

Validation completed locally:
- Camera suite: 529 tests, 525 passed, 4 skipped (not passes).
- All 23 runtime Python files cross-compiled for armv7emsp / MicroPython 1.28.
- All 18 declared H743 configurations built from this independent workspace.
- 15 final UART cases passed real H743 parsing and production steering mapping.
- H743 host suite: 72 cases / 7713 assertions; default and C 3000/300 holds passed.
- Four speed variants: 60 production wheel-target cases passed.
- 26 unique historical saved images checked with the same raw geometry:
  ready cases rose from 1 to 7; the 6 newly eligible frames supplied 50–52 rows.
  Prior departure, recent valid UART, short state age and coexistence availability
  were assumed to isolate this decision. This is not sequential recording replay.

Personal review covered all runtime differences, ordinary and exit ownership,
full UART commit, stale edge rejection, release after alignment, and mixed-module
identity rejection. The new release test also verifies that displaced single-edge
guidance stays active until aligned bilateral recovery is actually observed.

No hardware was connected or operated for this version. Road success, H750 FPS,
and IDE preview >=60 FPS are unverified. This is a software-tested candidate.

Wire formats remain V3/16-byte forward, V2/20-byte reverse state, V1/8-byte RESET/KEEP.
