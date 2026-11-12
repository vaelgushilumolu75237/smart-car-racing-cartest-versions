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

For `target=1`, opcode `0x0b` is the visual-entry calibration extension (the
legacy image-capture operation remains unsupported). Its payload is seven
bytes: `0x0b, 1, 0, 0, action, side, marker`. `action=0` resets the parked
collector, `action=1` records the current transition geometry, and `action=2`
commits that side's three-marker record. Marker numbers are `0=FIRST_STABLE`,
`1=LOWER_OPENING`, and `2=UPPER_ENTRANCE`; side is `0=left` or `1=right`.
Every action is session/parked/fault gated and the camera's final ACK is
forwarded without fabricating success.

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
VCALRESET
VCALMARK00 ... VCALMARK12
VCALCOMMIT0 / VCALCOMMIT1
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

The `VCAL*` commands require an `ARM` session and the vehicle to remain parked.
Use `VCALRESET`, then hand-push one side while issuing `VCALMARK00` (first
stable view), `VCALMARK01` (lower opening), and `VCALMARK02` (upper physical
entrance); use `1` as the first digit for the right side. Commit with
`VCALCOMMIT0` or `VCALCOMMIT1` only after all three markers have been accepted.
The software rejects missing/indistinguishable geometry and CRC-protects the
resulting `VISUAL_ENTRY_V1` record. No calibration file is included in the
software package, so the visual variant remains `pending_manual_hand_push`.

One parked three-marker pass records local grayscale images, measured geometry,
and mean forward wheel millirevolutions. It exports independent
`ring_visual_entry_calibration.json` and `ring_encoder_entry_calibration.json`
files. Marker images stay under the relative `ring_entry_marks/` directory and
are excluded from public checkpoints. The FIRST marker and runtime distance
origin use the same stable confirmed-roundabout observation, not an ordinary
straight-road baseline. No image is sent over the wireless link.

On page 5, channel 15 exposes the actual camera ACK detail. For COMMIT, bit 0
means the visual file was saved and verified; bit 1 means the independent encoder
file was saved and verified. Value 3 confirms both. Partial results retain a
non-success status and cannot be interpreted as both files having committed.
Each version runs only its own entrance gate after calibration.

The assistant page 4 is the live visual-entry page: FPS, valid flag, ring
state, entry-ready flag, gate state, calibration-ready flag, consecutive
matches, transition flag, far/near transition rows, reason code, gate side,
ring-confirmed flag, curve phase, selected profile, and processing time. These
values are copied from camera STATE/CONFIG/ACK frames; receive counters are
not used as substitutes. A successful START selects page 1 automatically for
wheel, servo, IMU, and ring telemetry; STOP returns to page 5 for the command
ticket and safety gates.

Camera STATE payload bytes 26--35 additionally carry visual-entry diagnostics:
ready, gate state, calibration-ready, consecutive matches, transition flag,
ring-confirmed, far/near transition rows (`255` means unavailable), reason
code, and gate side (`0` unknown, `1` left, `2` right). Complete-tool mode
forwards these STATE bytes unchanged; older readers may ignore them.

## Safety and transport rules

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
