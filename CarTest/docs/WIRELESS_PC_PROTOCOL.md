# Wireless PC protocol and SeekFree compatibility

The H743/K230 link uses the bounded `D3 91` envelope implemented in
`Inc/wireless_protocol.h` and `App/wireless_protocol.c`:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | magic `D3 91` |
| 2 | 1 | version `1` |
| 3 | 1 | frame type |
| 4 | 2 | payload length, little endian, at most 160 |
| 6 | 2 | reserved, must be zero |
| 8 | 4 | sequence, little endian |
| 12 | 4 | H743 session token, little endian |
| 16 | 4 | sender uptime in milliseconds, little endian |
| 20 | n | payload |
| 20+n | 4 | CRC-32/IEEE of bytes 0 through 19+n, little endian |

The fixed envelope is 20 bytes and the maximum complete frame is 184 bytes.
The parser has a fixed buffer and accepts noise, truncation, wrap-safe sequence
numbers, and CRC failures without allocating memory. Commands are accepted by
the H743 command layer only when their session and timestamp are valid; this
codec does not grant command permissions by itself.

The current state payload is 32 little-endian uint32 values (128 bytes). The
field map is maintained in `tools/wireless/complete_tool.py` and includes
encoder speeds, applied motor/servo output, vision evidence, IMU/gyro state,
camera age/counts, settings generation, radio drops, and uptime. Signed fields
are decoded as two's-complement int32 by the PC tool.

## SeekFree Assistant compatibility mode

The compatibility helpers reproduce the public SeekFree source at commit
`9adccd9caa536ce6bfb5deea4107e186c65c77b9`:

`https://gitee.com/seekfree/TC264_Library/tree/9adccd9caa536ce6bfb5deea4107e186c65c77b9/Example/Beta_Demo/Seekfree_TC264_auto_exposure_adjust/libraries/zf_components`

Source constants are `0xAA` for MCU-to-PC packets and `0x55` for PC-to-MCU
parameter packets. The checksum is an 8-bit sum. For MCU packets it is
`0xAA + sum(bytes after the checksum)`. For the 8-byte parameter packet the
official receiver clears the checksum byte and sums all 8 bytes with an initial
accumulator of `0xFF`.

The official non-STC camera IDs are binary `0x01`, gray `0x02`, and RGB565
`0x03`. The camera configuration is 12 bytes: `AA`, checksum, command `02`,
type, width little endian, height little endian, data-type `01`, and three
reserved bytes, followed by the packed image. Binary images consume
`ceil(width*height/8)` bytes. The 8-byte oscilloscope configuration is
`AA`, checksum, command `10`, channel count (1..16), data type, three reserved
bytes; its data is channel-count little-endian IEEE754 float values. The
8-byte incoming debug parameter packet is `55`, checksum, command `20`,
zero-based channel index, and one IEEE754 float. The H743 command layer maps
these values into its separately validated parameter groups.

The complete tool and the native compatibility mode are serial-exclusive:
close one before opening the other. The complete tool requires an explicit
Connect click, then an explicit HELLO/START action. It never opens a COM port,
flashes firmware, or starts a vehicle from import or process startup.

The binary image stream uses the D3/91 IMAGE frame and is bounded to one
capture session. Its payload is 18 bytes of metadata followed by a chunk:
`frame_id u32, offset u32, total_bytes u32, width u16, height u16,
format u8, flags u8, data`. A receiver rejects mixed dimensions, stale frame
IDs/sessions, totals above 76800 bytes, and chunks that exceed the declared
range. The official 12-byte SeekFree camera
header is emitted only in compatibility mode, before its contiguous image
bytes; it is never interleaved with D3/91 telemetry frames.

## Module configuration

The supplied module manual describes 5 V module power and 0-3.3 V RX/TX
signals. RTS may be left unconnected only with the manual's 10 kOhm pull-up to
3.3 V; CMD is active-low configuration mode and stays high during normal use.
The firmware does not send AT commands automatically. Configure the module's
retry policy deliberately; infinite retransmission can create internal queues
that are unsuitable for real-time telemetry. The module's physical H743
USART2 pin header still requires board-level verification.


## SeekFree Assistant width calibration (v5)

While parked on a measured 45 cm straight, keep the K230 in its final race
mount and the H743 running in profile C. In the existing SeekFree Assistant
serial tab, disable HEX send and automatic send. Send the ASCII line
`CALWIDTH\n` (or `CALWIDTH\r\n`). The characters `\n` here denote an actual
line ending, not a literal backslash followed by n. The GUI's line-ending
option may append the ending. This command does not start driving.

The H743 requires released startup/local buttons, inactive motion and
actuator tests, no fault, no other calibration/capture/Flash operation, and
a fresh negotiated K230 link. The existing K230 routine still validates
stationary profile state and stable bilateral straight rows. Only the
camera's successful final save/reload ACK means success; visible edge
lines or a ready progress event alone do not mean the width map was saved.
The K230 stores the private width map and updates its current tracker.

A successfully forwarded request is allowed once per H743 boot. Further
copies of the same text do not restart the job or erase its result.
Rejected prerequisites may be corrected and retried. To retry a forwarded
job after failure, restart the H743 manually and send a new request after
stopping the car; the software never resets hardware itself.

Sending CALWIDTH selects scope page 3 automatically. Scope pages 0 to 2
retain their existing mappings. Debug parameter channel 7 selects a page
0 to 3; parameter writes on channels 0 to 6 are ignored on page 3.
The 16-channel status mapping is documented in the matching release notes.
Use SeekFree Assistant binary scope mode (Printf off).

Page 3 channels (zero based):

| Channel | Value |
| --- | --- |
| 0 | Calibration state: 0 idle, 1 pending, 2 saved/reloaded successfully, 3 rejected, 4 failed/disconnected, 5 timeout |
| 1 | ACK status: 0 OK, 1 pending, 7 not parked, 8 busy, 13 fault, 15 camera not ready |
| 2 | Accepted straight-row frames (64 required) |
| 3 | Attempted eligible frames |
| 4 | Camera handshake ready |
| 5 | Camera link fresh (within 1000 ms) |
| 6 | Camera message age in ms; -1 if disconnected or older than 1000 ms |
| 7 | A request has been forwarded this H743 boot |
| 8 | Received camera message count |
| 9 | Dropped camera messages |
| 10 | Dropped radio messages |
| 11 | Radio RX overflow count |
| 12 | Settings generation |
| 13 | Parsed complete-protocol radio packets |
| 14 | Current scope page (3) |
| 15 | Camera calibration reason hash (diagnostic code, not a success indication) |

Show only channels 0 to 5 while calibrating to avoid large counters
stretching the scope's automatic Y axis. When finished, send debug
parameter channel 7 value 0 to restore the ordinary 16-channel page.


## Read-only transport diagnostic scope page 4 (v6)

Use the existing SeekFree debug parameter channel 7 value 4, or send this
native parameter packet in the serial tab with HEX send enabled and line
endings/automatic send disabled: `55 3B 20 07 00 00 80 40`.
This selects telemetry only and never opens outputs or reinitializes sensors.
Scope pages 3 and 4 ignore parameter writes on channels 0 to 6.

| Channel | Value |
| --- | --- |
| 0 | WHO_AM_I has actually been read successfully (bus callback success) |
| 1 | Last actual WHO_AM_I byte, decimal (not an inferred fault code) |
| 2 | Expected ICM42688-P identity 71 decimal = 0x47 |
| 3 | Number of completed WHO_AM_I reads |
| 4 | IMU fault bits |
| 5 | IMU ready and calibrated |
| 6 | Raw received K230 UART bytes |
| 7 | K230 UART error event count |
| 8 | K230 control-frame CRC error count |
| 9 | K230 RX queue overflow count |
| 10 | Bytes actually written to the K230 USART1 transmit register |
| 11 | Received camera diagnostic-protocol frames |
| 12 | Wireless USART2 error count |
| 13 | IMU sample count |
| 14 | Page number (4) |
| 15 | Last admitted vision sequence |

WHO_AM_I valid=0 means there is no successful identity read; a displayed zero
must not be called a measured chip identity in that case. Valid=1 with identity
0/255/another value preserves the actual mismatch across the latched fault.
No timeout, calibration gate, sample validity or motor safety logic is bypassed.
The page is meant to locate the fault; no hardware repair is claimed by passing
host tests or compiling this diagnostic firmware.


## SeekFree-only text start/stop (v10)

See `SEEKFREE_START_V10_CN.md` for the operating instructions and scope page 5.
In assistant mode only, send ASCII `ARM` with a real line ending, read the
seven-digit one-use ticket from scope channel 2, then send `START <ticket>`
within 30 seconds. `STOP` needs no ticket and uses the normal MCU stop effect.
Both LF and CRLF are accepted; HEX/automatic send must be off. START uses the
existing session/command/effect path, and only the original MCU start handler
can authorize the follow request. A successful effect acknowledgement means
request accepted, not proof of driving output. Expired, consumed, fault/PE7
revoked, replaced-session or UART-error tickets cannot restart the vehicle.
These tickets prevent accidental duplicate execution; they are not a radio
authentication or encryption mechanism. No other PC application is required.
