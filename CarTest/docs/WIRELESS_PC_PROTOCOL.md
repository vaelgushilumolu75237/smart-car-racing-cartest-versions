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
