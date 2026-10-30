# ACK1 coordination snapshot and host export

`g_ackermann_snapshot` is an independent 64-byte ACK1/v1 snapshot in the
dedicated `.ackermann_snapshot` RAM section. It is not part of the 64-byte RLOG
header or the RLOG record stream. The section is intended for bounded SWD
HotPlug reads and contains the latest coordination state, not a full drive
timeline.

| Offset | Field | Encoding |
| ---: | --- | --- |
| 0 | magic | ASCII `ACK1` |
| 4 | version | `1` |
| 5 | size | `64` |
| 6 | flags | u16 LE; bits 0..5 are coord enabled, request active, vision hold, fault locked, session active, frozen |
| 8 | generation | u32 LE; stable values are even; odd means a write is in progress |
| 12 | timestamp | u32 LE milliseconds |
| 16 | vision sequence | u16 LE |
| 18 | curve phase | u8, `0..3` |
| 19 | reserved | zero |
| 20 | slowdown request | u16 LE, `0..1000` permille |
| 22 | reserved | u16 LE, zero |
| 24 | model steering angle | i16 LE, centidegrees |
| 26 | ratio | u16 LE, Q15 |
| 28 | applied PWM delta | i16 LE, microseconds |
| 30 | servo pulse | u16 LE, microseconds |
| 32 | fastest target | i32 LE, milli-revolutions/second |
| 36 | left target | i32 LE, milli-revolutions/second |
| 40 | right target | i32 LE, milli-revolutions/second |
| 44 | left measured | i32 LE, milli-revolutions/second |
| 48 | right measured | i32 LE, milli-revolutions/second |
| 52 | left duty | u16 LE, `0..1000` permille |
| 54 | right duty | u16 LE, `0..1000` permille |
| 56 | run state | u8, `0..12` |
| 57 | fault reason | u8, `0..15` |
| 58 | reserved | u16 LE, zero |
| 60 | CRC-32 | u32 LE, CRC-32/ISO-HDLC over bytes 0..59 |

The CRC uses reflected polynomial `0xEDB88320`, initial and final xor
`0xFFFFFFFF`. Stable generations are read both immediately before and after
the 64-byte snapshot. The reader requires the two generation guards to equal
the snapshot generation, the generation to be even, and the CRC and reserved
fields to validate. Generation zero is valid after uint32 wrap; only a
64-byte all-zero section is treated as uninitialized.

An ACK1 snapshot can be exported only if it is valid, marked FROZEN, and has
`session_active=0`. The export wrapper additionally requires the preflight and
post-export 64-byte snapshots to be byte-identical. The resulting
`ackermann-snapshot.csv` contains one snapshot row and is labeled in metadata
as `ACK1/v1/64B single coordination snapshot; not a run timeline`. It never
substitutes for `run.csv`; `run.csv` is produced only from a validated frozen
RLOG with at least two records. IDLE, ACTIVE, incomplete, or invalid exports
do not produce a successful `run.csv` or an empty ACK1 CSV.

Older ELFs without the ACK1 symbol continue through the optional snapshot
path as unavailable, while RLOG V1/V2 decoding remains independent of ACK1.
