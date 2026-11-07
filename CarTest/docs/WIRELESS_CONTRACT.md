# Wireless implementation contract (2026-11-07)

Root owns architecture, target integration and review. Workers own only assigned files, do not revert other edits.
Production opt-in: CARTEST_WIRELESS_ENABLED=1 only new DebugWirelessGyro350Speed140; old profiles default 0.
Root handles CMake/presets/USART2 target adapter/Src/main.c/commissioning.c/Flash linking and dispatch bridge.

## Shared wire
C codec API owned by protocol worker, agree signatures promptly with root and camera worker.
Magic D3 91, version=1, type u8, payload_len u16LE, reserved u16LE=0, seq u32LE, session u32LE, timestamp_ms u32LE: 20-byte header; payload <=160; CRC32 IEEE of header+payload, trailing u32LE. Total <=184. No heap; bounded stream parser and bounded TX. Types HELLO=1, HELLO_REPLY=2, COMMAND=3, ACK=4, STATE=16, DIAG=17, IMAGE=18, CONFIG=19, EVENT=20.
Camera uses same outer framing, session=H743 camera-link epoch provided in HELLO; camera diagnostics and image packets must not alter legacy V4 sequence/freshness/profile-ACK. H743 accepts image only with current capture/session identity, aggregate maximum 76800 bytes, in-order chunks with fixed total/dimensions and TTL; no raw unbounded buffers.
HEADER timestamp for COMMAND is last known H743 uptime plus locally elapsed monotonic time, accepted age 0..1000ms only (no future). Session token assigned by H743 after HELLO; new HELLO while actuator test running must not extend it, and session changes require idle or old commands invalidated. Strict increasing command seq, duplicates return cached result without executing, backwards reject. STOP can revoke outputs independently of START readiness but must have valid framing.

## Commands/parameters
Control worker owns authoritative IDs in Inc/wireless_control.h and Inc/wireless_settings.h. Publish APIs/IDs to root/protocol and camera worker early. Generic COMMAND payload starts opcode u8, target u8 (0 MCU /1 camera), reserved u16=0, remaining opcode data. Read-only INFO/GET, START, STOP, SET_PARAMS(group atomic)/SAVE/DEFAULT, TEST_MOTOR/TEST_SERVO, IMU_CAL, CAMERA_CAL, CAPTURE. No remote fault bypass, firmware/reset/flash/download commands.
Parameter schema fixed integer units: wheel_scale_permille 250..1000 default1000; gyro_rate_mdps 30000..180000 default120000; gain scales permille500..1500 default1000 for vision P/D, speed P/I, gyro P/I. Parked: ring progress default350000, exit default300, search default3000 with bounded ranges; servo endpoints within1100..2500 center fixed1530; encoder CPR/sign bounded nonzero, signs +-1. Never exposes safety-disable/timeouts permissions.
SET/SAVE/DEFAULT cannot lie about cross-device success: MCU ACK and camera ACK distinct. Camera settings versioned CRC checked private sidecar separate calibration.
Actuator tests require stopped, local button released/startup confirmed, no hard fault, no concurrent calibration/capture, hardware acceptance; bypass of absent road only in explicit isolated test mode, never enables follow. Motors forward left/right/both <=400permille duration<=5000ms ramp bounded; servo1100..2500 duration<=1000ms then1530. Duplicates never restart timers; local stop/fault overrides. No heartbeat requirement; run continues on radio loss; timed tests still expire. Fresh start reruns original commissioning start gate.

## Telemetry/tools
STATE20Hz,DIAG2Hz,80x60 packed binary2fps,320x240 gray only requested parked;115200, bounded chunks; V4 control traffic first. Root takes MCU snapshots, camera supplies immutable frame snapshot/chunks nonblocking. Seekfree real assistant framing from official source, no fabricated compatibility: <=16 scope channels, 8 incoming parameters paging, image types. Desktop complete tool serial-exclusive switch with assistant, mandatory explicit Connect and Start, never opens a real COM in tests.
Workers add meaningful host tests on production implementation; no fake pass switches. No flashing/reset/serial opening/motors. Preserve source and no public real logs/images/calibration.
