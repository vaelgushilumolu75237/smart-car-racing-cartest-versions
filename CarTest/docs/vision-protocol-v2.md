# Vision UART Protocol V2

This document describes the strict STM32H743/OpenMV wire contract. V2 is a
fixed 16-byte frame. The UART stream has no padding between frames.

| Offset | Size | Field | Encoding and validation |
| --- | ---: | --- | --- |
| 0 | 1 | sync 0 | `0xA5` |
| 1 | 1 | sync 1 | `0x5A` |
| 2 | 1 | version | `2`; other versions are reported as `VERSION_MISMATCH` |
| 3 | 1 | payload length | `8` |
| 4 | 2 | sequence | unsigned 16-bit little-endian; wraps naturally through `65535 -> 0` |
| 6 | 1 | flags | bit 0 valid, bit 1 slope hint; all other bits zero |
| 7 | 1 | confidence | unsigned `0..100` |
| 8 | 2 | lateral error | signed 16-bit little-endian, `-1000..1000` |
| 10 | 1 | curve phase | `0` straight, `1` candidate, `2` turn, `3` exit |
| 11 | 1 | reserved | must be zero |
| 12 | 2 | slowdown request | unsigned 16-bit little-endian, `0..1000` permille |
| 14 | 2 | CRC | CRC-16/CCITT-FALSE, little-endian |

`payload length = 8` counts bytes 6 through 13 only. The 2-byte sequence at
bytes 4–5 is a header field and is excluded from the payload length. This keeps
the same convention as V1, whose payload length `4` counts bytes 6–9.

CRC-16/CCITT-FALSE uses polynomial `0x1021`, initial value `0xFFFF`, no
reflection and xorout `0`. It covers bytes 2 through 13 inclusive. The sync
bytes are not covered.

For a valid observation, the parser accepts only the flag mask, confidence,
lateral-error, phase, and slowdown ranges in the table. For `valid=0`, sequence
remains ordinary transport metadata and may advance or wrap. Flags,
confidence, lateral error, phase, and slowdown request must all be zero. This
lets the receiver distinguish a live camera stream with no route from a stale
or silent stream. Invalid observations do not carry slope hints.

The C/OpenMV shared vectors live in
[`tests/data/vision-protocol-v2.csv`](../tests/data/vision-protocol-v2.csv) and
[`openmv/tests/vision-protocol-v2.csv`](../openmv/tests/vision-protocol-v2.csv).
They include invalid sequence wrap, valid sequence wrap, signed lateral-error
cases, all four phases, and the slowdown upper bound.

The OpenMV encoder exposes:

```python
pack_into(out, sequence, valid, confidence, lateral_error,
          slope_hint=0, curve_phase=0, slowdown_request_permille=0)
```

`out` is a caller-owned writable `bytearray` with at least 16 bytes. The
function modifies only the first 16 bytes and returns the same buffer. The
allocating `pack_frame` and `pack` helpers accept the same optional phase and
slowdown fields.

On the H743, `vp_unpack_ex(in, frame)` returns one `vp_status` value. The
streaming parser reports its most recent result through
`vp_parser_last_status(parser)` and preserves the received version through
`vp_parser_last_version(parser)`. A V1 or unknown version is rejected as
`VP_STATUS_VERSION_MISMATCH` immediately after the version byte is received;
there is no implicit V1 decoding in the V2 parser.

The 32-byte OpenMV camera diagnostic frame is unchanged. Curve tracing stores
the exact V2 UART bytes as `curve_trace_v2_hex`; its CRC is checked and its
sequence/valid/error fields are matched to the corresponding captured frame.
The existing 64-byte RLOG record layout is unchanged. RLOG V1 and V2 records
remain separately decoded by the host exporter.

Protocol host tests verify C/Python golden-vector agreement, partial and
concatenated UART input, CRC/reserved/range rejection, sequence wrap, and
explicit V1-version mismatch. These checks establish software-format behavior;
they do not establish sensor frame rate or physical vehicle behavior.
