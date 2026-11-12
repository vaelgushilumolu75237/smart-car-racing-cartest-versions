# CarTest wireless protocol

The H743 wireless UART uses USART2: PA2 is TX, PA3 is RX, alternate function
AF7, 115200 8N1, with CR3 left at zero. PA4 (RTS) and PA5 (CMD) are input
status lines with pull-ups; the firmware does not drive either line.

## D3/91 framing

Every binary frame starts with `D3 91`, followed by version `1`, a 20-byte
header, a payload of at most 160 bytes, and a little-endian CRC32 over the
header and payload. Header fields are:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | magic `D3 91` |
| 2 | 1 | version `1` |
| 3 | 1 | frame type |
| 4 | 2 | payload length |
| 6 | 2 | reserved, zero |
| 8 | 4 | sequence |
| 12 | 4 | session |
| 16 | 4 | timestamp in milliseconds |

Command payloads begin with `opcode:u8`, `target:u8` (`0` MCU, `1` camera),
and `reserved:u16=0`. ACK payloads are 16 bytes:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | status |
| 1 | 1 | opcode |
| 2 | 1 | target |
| 3 | 1 | flags; bit 0 means duplicate |
| 4 | 4 | request sequence |
| 8 | 4 | detail |
| 12 | 4 | settings generation |

The H743-to-camera HELLO payload is 8 bytes: `epoch:u32`, `parked:u8`,
`profile_key:u8`, and two reserved bytes. The six profile keys are A no-ring,
B ring base, C 100%, C +20%, C +30%, and C +40%. The camera replies with
`epoch:u32` and `capabilities:u32`.

## SeekFree assistant

Compatibility mode emits oscilloscope frames with head `0xAA`, command `0x10`,
and up to 16 float channels. Incoming tuning frames use head `0x55` and
command `0x20`, with up to 8 float channels. The text commands are:

```text
ARM
START <7-digit-ticket>
STOP
READ
READCAM
SAVE
SAVECAM
DEFAULT
IMUCAL
CALWIDTH
ECALRESET
ECALMARK00 ... ECALMARK12
ECALCOMMIT0 / ECALCOMMIT1
```

`SAVE` and `DEFAULT` require the vehicle to be parked, PE7 released, startup
release confirmed, and all faults, tests, and calibrations inactive. `SAVE`
updates the active profile's persistent record only after the asynchronous
flash write and verify complete. `DEFAULT` replaces RAM settings with the
compiled defaults for the active profile; it does not write flash.
`READCAM` queries the camera separately. `SAVECAM` saves the camera's accepted
PD values only after the actual synchronization ACK; it refuses unsynchronized
values. The MCU and camera report separate operation results. `IMUCAL` requests
the existing parked/stationary IMU calibration. `DEFAULT` restores MCU defaults
and sends the desired default vision gains through the normal camera ACK path.
Text commands require a newline; `ARM` opens a session for the query/save commands
without starting the vehicle. Any intervening query uses its own sequence number,
so the single-use `START` ticket remains usable until its 30-second expiry.

The assistant page 4 carries current entrance diagnostics: FPS, valid flag,
ring state, entry ready, gate state, calibration ready, distance progress,
transition flag, distance threshold, side, reserved value, side again,
ring confirmed, curve phase, profile, and processing time. These values come
from camera STATE frames; receive counters are not used as substitutes.

## Safety and transport rules

The current encoder entrance version uses the parked `ECALRESET`,
`ECALMARK00`/`01`/`02` (left) or `ECALMARK10`/`11`/`12` (right), and
`ECALCOMMIT0`/`1` commands after ARM. A single hand-push records local images,
geometry and mean forward wheel millirevolutions for both independent calibration
files. It does not install the visual entrance gate as a runtime requirement.
No distance is supplied through parameter 18. FIRST and the runtime distance
origin must use the same stable confirmed-roundabout observation.

COMMIT ACK detail is a bitmask: bit 0 visual file verified, bit 1 encoder file
verified. Page 5 channel 15 reports this detail; value 3 confirms both files.
Partial saves must report a non-success ACK. Page 4 channels 7 and 9 contain
current/target mean wheel millirevolutions; channel 10 is gate side (0 unknown,
1 left, 2 right). The complete channel map is documented in
`docs/RingEntry_TwoVersions_20261112.md`.

Only bounded live controls can be changed while moving. Ring progress, timing,
servo limits, encoder configuration, and calibration remain parking-only. A
wireless PC disconnect does not stop local follow, and reconnecting does not
replay old commands. Command sessions, timestamps, sequence numbers, and CRCs
are checked before execution. The OpenMV port has no image transfer on this
link; diagnostic STATE, DIAG, CONFIG, EVENT, and camera ACK frames remain
available.

The six default yaw targets are 120000, 64935, 85714, 102857, 111429, and
120000 mdps for A, B, C100, C120, C130, and C140 respectively. They are
software candidates and require vehicle tuning; software tests do not prove a
track result.

Persistent H743 storage uses two complete six-profile tables with a table CRC
and a 32-byte commit marker written last. A save erases the alternate table;
the newest committed table remains intact until all new records and the marker
are verified. Individual records without a table marker are never loaded.
Host tests execute the production flash state machine against mapped RAM at
every interruption point and inject failure at each of its 25 flash writes.
