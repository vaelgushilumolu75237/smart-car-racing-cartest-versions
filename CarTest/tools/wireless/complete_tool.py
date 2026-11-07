"""Explicit-connect diagnostic tool for the CarTest wireless link.

This is a deliberately small companion to SeekFree Assistant.  It supports
the complete D3/91 command and telemetry stream, CSV/raw logging, and a
simple state view.  It never opens a COM port or sends a vehicle command at
startup; both connection and START require an explicit button click.

The program uses only Tkinter from the standard library.  Install pyserial
only when connecting to hardware: ``python -m pip install pyserial``.
"""

from __future__ import annotations

import csv
import base64
from dataclasses import dataclass
from collections import deque
import queue
import struct
import zlib
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path

try:  # Package execution: python -m tools.wireless.complete_tool
    from .protocol import Frame, FrameParser, FrameType, ProtocolError, encode_frame, pack_command
except ImportError:  # Direct execution from tools/wireless
    from protocol import Frame, FrameParser, FrameType, ProtocolError, encode_frame, pack_command


# These are the stable generic command IDs expected by the H743 command
# worker.  The complete command payload remains generic so a later firmware
# revision can add a target opcode without changing framing.
OP_INFO = 0x01
OP_START = 0x02
OP_STOP = 0x03
OP_SET_PARAMS = 0x04
OP_SAVE = 0x05
OP_DEFAULT = 0x06
OP_TEST_MOTOR = 0x07
OP_TEST_SERVO = 0x08
OP_IMU_CAL = 0x09
OP_CAMERA_CAL = 0x0A
OP_CAPTURE = 0x0B
OP_MODE = 0x0C
OP_PAGE = 0x0D
OP_GET_PARAMS = 0x0E
# Compatibility alias for callers that used the earlier short name.  The
# firmware opcode is GET_PARAMS=14; do not alias it to START=2.
OP_GET = OP_GET_PARAMS


def _signed(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def decode_state_payload(payload: bytes) -> dict[str, int]:
    """Decode the contract's 32 little-endian uint32 state fields."""
    if len(payload) != 128:
        raise ProtocolError("STATE payload must contain exactly 32 uint32 fields")
    values = list(struct.unpack("<32I", payload))
    names = (
        "now_ms", "mode", "fault", "fault_latched", "follow_active",
        "road_allowed", "vision_valid", "vision_sequence", "vision_error",
        "confidence", "measured_left_millirps", "measured_right_millirps",
        "target_left_millirps", "target_right_millirps", "servo_us",
        "duty_left_permille", "duty_right_permille", "imu_ready", "imu_fault",
        "imu_z_mdps", "yaw_mdeg", "gyro_phase", "gyro_progress_mdeg",
        "gyro_fault", "camera_diag_age_ms", "camera_count", "setting_generation",
        "radio_overflows", "radio_tx_drops", "camera_drops", "current_page",
        "uptime_ms",
    )
    state = dict(zip(names, values))
    for key in ("vision_error", "imu_z_mdps", "yaw_mdeg", "gyro_progress_mdeg"):
        state[key] = _signed(state[key])
    return state


def decode_camera_state_payload(payload: bytes) -> dict[str, int]:
    """Decode the K230 camera STATE payload (36 bytes)."""
    if len(payload) != 36:
        raise ProtocolError("camera STATE payload must contain exactly 36 bytes")
    return {
        "camera_frame_id": int.from_bytes(payload[0:4], "little"),
        "camera_valid": payload[4],
        "camera_quality": payload[5],
        "camera_source": payload[6],
        "camera_ring_state": payload[7],
        "camera_side": payload[8],
        "camera_lateral_error": int.from_bytes(payload[9:11], "little", signed=True),
        "camera_control": int.from_bytes(payload[11:13], "little", signed=True),
        "camera_processing_ms": int.from_bytes(payload[13:15], "little"),
        "camera_fps_x10": int.from_bytes(payload[15:17], "little"),
        "camera_curve_phase": payload[17],
        "camera_slowdown": int.from_bytes(payload[18:20], "little"),
        "camera_profile": payload[20],
        "camera_profile_active": payload[21],
        "camera_session_token": int.from_bytes(payload[22:26], "little"),
        "camera_detector_reason": int.from_bytes(payload[26:28], "little"),
    }


def decode_camera_diag_payload(payload: bytes) -> dict[str, int]:
    """Decode the K230 camera DIAG payload (eight little-endian uint32s)."""
    if len(payload) != 32:
        raise ProtocolError("camera DIAG payload must contain exactly 32 bytes")
    values = struct.unpack("<8I", payload)
    return dict(zip((
        "camera_frame_id", "camera_processing_us", "camera_capture_us",
        "camera_tx_errors", "camera_parser_errors", "camera_command_errors",
        "camera_memory_free", "camera_flags",
    ), values))


def decode_h743_config_payload(payload: bytes) -> dict[str, int | str]:
    """Decode the 128-byte WGS1 settings slot emitted by H743."""
    if len(payload) != 128:
        raise ProtocolError("H743 CONFIG payload must contain exactly 128 bytes")
    magic, schema, generation, payload_bytes = struct.unpack_from("<IIII", payload)
    if magic != 0x31534757 or schema != 1 or payload_bytes != 50:
        raise ProtocolError("CONFIG is not a valid H743 WGS1 slot")
    payload_crc, record_crc = struct.unpack_from("<II", payload, 70)
    if payload_crc != (zlib.crc32(payload[20:70]) & 0xFFFFFFFF):
        raise ProtocolError("H743 CONFIG payload CRC mismatch")
    if record_crc != (zlib.crc32(payload[:74]) & 0xFFFFFFFF):
        raise ProtocolError("H743 CONFIG record CRC mismatch")
    values = {
        "config_target": "h743", "config_schema": schema,
        "settings_generation": generation,
        "wheel_scale_permille": struct.unpack_from("<H", payload, 20)[0],
        "gyro_rate_mdps": struct.unpack_from("<I", payload, 22)[0],
        "vision_gain_permille": struct.unpack_from("<H", payload, 26)[0],
        "speed_p_gain_permille": struct.unpack_from("<H", payload, 28)[0],
        "speed_i_gain_permille": struct.unpack_from("<H", payload, 30)[0],
        "gyro_p_gain_permille": struct.unpack_from("<H", payload, 32)[0],
        "gyro_i_gain_permille": struct.unpack_from("<H", payload, 34)[0],
        "ring_progress_mdeg": struct.unpack_from("<I", payload, 36)[0],
        "ring_exit_ms": struct.unpack_from("<H", payload, 40)[0],
        "ring_search_ms": struct.unpack_from("<H", payload, 42)[0],
        "servo_left_us": struct.unpack_from("<H", payload, 44)[0],
        "servo_right_us": struct.unpack_from("<H", payload, 46)[0],
        "encoder_left_cpr": struct.unpack_from("<I", payload, 48)[0],
        "encoder_right_cpr": struct.unpack_from("<I", payload, 52)[0],
        "encoder_left_sign": struct.unpack_from("<b", payload, 56)[0],
        "encoder_right_sign": struct.unpack_from("<b", payload, 57)[0],
        "vision_d_gain_permille": struct.unpack_from("<H", payload, 58)[0],
    }
    return values


def decode_camera_config_payload(payload: bytes) -> dict[str, int | str]:
    """Decode the compact K230 camera settings sidecar when present.

    The camera sidecar uses version/generation/P/D/CRC fields. Unknown future
    records remain visible as bounded hex rather than being guessed.
    """
    if len(payload) == 16:
        version, generation, vision_p, vision_d, record_crc = struct.unpack(
            "<IIHHI", payload)
        if record_crc != (zlib.crc32(payload[:12]) & 0xFFFFFFFF):
            raise ProtocolError("camera CONFIG CRC mismatch")
        return {"config_target": "camera", "config_version": version,
                "settings_generation": generation,
                "vision_gain_permille": vision_p,
                "vision_d_gain_permille": vision_d}
    if len(payload) == 18 and payload[:4] == b"CAM1":
        version, generation, vision_p, vision_d, record_crc = struct.unpack(
            "<HIHHI", payload[4:])
        if record_crc != (zlib.crc32(payload[:14]) & 0xFFFFFFFF):
            raise ProtocolError("camera CONFIG CRC mismatch")
        return {"config_target": "camera", "config_version": version,
                "settings_generation": generation,
                "vision_gain_permille": vision_p,
                "vision_d_gain_permille": vision_d}
    return {"config_target": "camera", "config_bytes": len(payload),
            "config_hex": payload.hex()}


def decode_config_payload(payload: bytes) -> dict[str, int | str]:
    if len(payload) == 128 and payload[:4] == b"WGS1":
        return decode_h743_config_payload(payload)
    return decode_camera_config_payload(payload)


def decode_event_payload(payload: bytes) -> dict[str, int | str]:
    return {"event_bytes": len(payload), "event_hex": payload.hex()}


def decode_image_payload(payload: bytes) -> tuple[dict[str, int], bytes]:
    """Decode the H743 image chunk layout.

    Payload is ``frame_id u32, offset u32, total u32, width u16, height u16,
    format u8, flags u8, data``.  The H743 uses one frame ID and monotonically
    increasing offsets; the GUI assembles chunks after validating all metadata.
    """
    if len(payload) < 18:
        raise ProtocolError("IMAGE chunk header is truncated")
    frame_id, offset, total, width, height, image_format, flags = \
        struct.unpack_from("<IIIHHBB", payload)
    if width == 0 or height == 0 or total == 0 or total > 76800 or image_format not in (1, 2):
        raise ProtocolError("IMAGE chunk metadata is invalid")
    expected = ((width * height + 7) // 8 if image_format == 1
                else width * height)
    if total != expected:
        raise ProtocolError("IMAGE total does not match dimensions and format")
    data = payload[18:]
    if offset + len(data) > total:
        raise ProtocolError("IMAGE chunk range is invalid")
    return ({"frame_id": frame_id, "width": width, "height": height,
             "format": image_format, "flags": flags, "total": total,
             "offset": offset}, data)


@dataclass
class TelemetryEvent:
    frame: Frame
    decoded: dict[str, object] | None = None


class ImageAssembler:
    """Accept one monotonic, fixed-metadata image stream at a time."""

    def __init__(self) -> None:
        self.frame_id: int | None = None
        self.meta: dict[str, int] | None = None
        self.buffer = bytearray()
        self.received = 0
        self.started_ms: int | None = None

    def reset(self) -> None:
        self.frame_id = None
        self.meta = None
        self.buffer.clear()
        self.received = 0
        self.started_ms = None

    def add(self, meta: dict[str, int], data: bytes,
            now_ms: int | None = None) -> tuple[dict[str, int], bytes] | None:
        fixed = {key: value for key, value in meta.items() if key != "offset"}
        now_ms = int(time.monotonic() * 1000) if now_ms is None else int(now_ms)
        if self.frame_id != meta["frame_id"]:
            self.reset()
            self.frame_id = meta["frame_id"]
            self.meta = fixed
            self.buffer = bytearray(meta["total"])
            self.started_ms = now_ms
        if self.started_ms is None or now_ms - self.started_ms > 60000:
            self.reset()
            raise ProtocolError("IMAGE assembly exceeded 60000 ms")
        if self.meta != fixed or meta["offset"] != self.received:
            self.reset()
            raise ProtocolError("IMAGE chunks changed metadata or were out of order")
        self.buffer[meta["offset"]:meta["offset"] + len(data)] = data
        self.received += len(data)
        if self.received == meta["total"]:
            completed = (dict(meta), bytes(self.buffer))
            self.reset()
            return completed
        return None


class WirelessClient:
    """Serial client with explicit lifecycle and bounded parser state."""

    def __init__(self, event_queue: queue.Queue[TelemetryEvent | tuple[str, str]]):
        self.events = event_queue
        self.serial = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.parser = FrameParser()
        self.sequence = 0
        self.session = 0
        self.hello_nonce = 0
        self.h743_clock_offset_ms: int | None = None
        self.tx_log = None
        self.csv_file = None
        self.csv_writer = None
        self.lock = threading.Lock()
        self.mode_complete = False
        self.requested_mode: int | None = None
        self.images = ImageAssembler()

    @property
    def connected(self) -> bool:
        return self.serial is not None

    def _emit(self, item: TelemetryEvent | tuple[str, str]) -> None:
        """Publish without blocking the serial reader on a full GUI queue."""
        try:
            self.events.put_nowait(item)
            return
        except queue.Full:
            pass
        if isinstance(item, TelemetryEvent):
            return
        try:
            self.events.get_nowait()
            self.events.put_nowait(item)
        except (queue.Empty, queue.Full):
            return

    def connect(self, port: str, baud: int = 115200) -> None:
        if self.connected:
            raise RuntimeError("already connected; disconnect the current COM first")
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required only for an explicit hardware connection") from exc
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=0.1,
                                    write_timeout=0.2)
        self.parser.reset()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader, name="cartest-wireless-rx",
                                       daemon=True)
        self.thread.start()
        self._emit(("status", f"connected {port} @ {baud}"))

    def disconnect(self) -> None:
        self.stop_event.set()
        serial_obj = self.serial
        self.serial = None
        if serial_obj is not None:
            serial_obj.close()
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        self.thread = None
        self.session = 0
        self.mode_complete = False
        self.requested_mode = None
        self.h743_clock_offset_ms = None
        self.images.reset()
        self._emit(("status", "disconnected"))

    def set_log_directory(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.close_log()
        self.tx_log = (directory / "wireless_raw.bin").open("ab")
        self.csv_file = (directory / "wireless_state.csv").open("a", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(("host_time", "frame_type", "sequence", "session",
                                      "timestamp_ms", "field", "value"))
        self._emit(("status", f"logging to {directory}"))

    def close_log(self) -> None:
        for handle in (self.tx_log, self.csv_file):
            if handle is not None:
                handle.close()
        self.tx_log = None
        self.csv_file = None
        self.csv_writer = None

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            serial_obj = self.serial
            if serial_obj is None:
                return
            try:
                chunk = serial_obj.read(256)
            except Exception as exc:  # serial drivers raise platform-specific errors
                self._emit(("status", f"serial read error: {exc}"))
                return
            if not chunk:
                continue
            if self.tx_log is not None:
                self.tx_log.write(chunk)
                self.tx_log.flush()
            for frame in self.parser.feed(chunk):
                self._handle_frame(frame)

    def _handle_frame(self, frame: Frame) -> None:
        decoded = None
        if frame.type != FrameType.HELLO_REPLY and (
                self.session == 0 or frame.session != self.session):
            self._emit(("status", "ignored frame from stale wireless session"))
            return
        try:
            if frame.type == FrameType.STATE:
                if len(frame.payload) == 128:
                    decoded = decode_state_payload(frame.payload)
                elif len(frame.payload) == 36:
                    decoded = decode_camera_state_payload(frame.payload)
                else:
                    raise ProtocolError("STATE payload has an unknown size")
                if len(frame.payload) == 128:
                    self.h743_clock_offset_ms = (
                        int(time.monotonic() * 1000) - frame.timestamp_ms) & 0xFFFFFFFF
                if self.csv_writer is not None:
                    for key, value in decoded.items():
                        self.csv_writer.writerow((time.time(), frame.type, frame.sequence,
                                                  frame.session, frame.timestamp_ms, key, value))
                    self.csv_file.flush()
            elif frame.type == FrameType.DIAG:
                if len(frame.payload) == 128:
                    decoded = decode_state_payload(frame.payload)
                elif len(frame.payload) == 32:
                    decoded = decode_camera_diag_payload(frame.payload)
                else:
                    raise ProtocolError("DIAG payload has an unknown size")
                if len(frame.payload) == 128:
                    self.h743_clock_offset_ms = (
                        int(time.monotonic() * 1000) - frame.timestamp_ms) & 0xFFFFFFFF
                if self.csv_writer is not None:
                    for key, value in decoded.items():
                        self.csv_writer.writerow((time.time(), frame.type, frame.sequence,
                                                  frame.session, frame.timestamp_ms, key, value))
                    self.csv_file.flush()
            elif frame.type == FrameType.HELLO_REPLY and len(frame.payload) >= 8:
                # The assigned session is in the outer frame header; the
                # payload is nonce, settings generation, capabilities.
                if len(frame.payload) >= 12:
                    nonce, generation, capabilities = struct.unpack_from(
                        "<III", frame.payload)
                else:
                    nonce, capabilities = struct.unpack_from("<II", frame.payload)
                    generation = 0
                valid = int(nonce == self.hello_nonce)
                if valid:
                    self.session = frame.session
                    self.h743_clock_offset_ms = (
                        int(time.monotonic() * 1000) - frame.timestamp_ms) & 0xFFFFFFFF
                else:
                    self._emit(("status", "ignored HELLO_REPLY with nonce mismatch"))
                decoded = {"hello_nonce": nonce, "settings_generation": generation,
                           "capabilities": capabilities, "hello_valid": valid}
            elif frame.type == FrameType.ACK and len(frame.payload) >= 16:
                status, opcode, target, flags, request_seq, detail, generation = \
                    struct.unpack_from("<BBBBIII", frame.payload)
                decoded = {"ack_status": status, "ack_opcode": opcode,
                           "ack_target": target, "ack_flags": flags,
                           "ack_request_seq": request_seq, "ack_detail": detail,
                           "settings_generation": generation}
                if opcode == OP_MODE and status == 0 and target == 0:
                    self.mode_complete = self.requested_mode == 1
            elif frame.type == FrameType.IMAGE:
                meta, data = decode_image_payload(frame.payload)
                completed = self.images.add(meta, data,
                                            int(time.monotonic() * 1000))
                decoded = {key: value for key, value in meta.items()}
                if completed is not None:
                    decoded["complete_image"] = completed[1]
            elif frame.type == FrameType.CONFIG:
                decoded = decode_config_payload(frame.payload)
            elif frame.type == FrameType.EVENT:
                decoded = decode_event_payload(frame.payload)
        except ProtocolError as exc:
            self._emit(("status", f"payload error: {exc}"))
        self._emit(TelemetryEvent(frame, decoded))

    def _send(self, frame_type: int, payload: bytes = b"") -> None:
        with self.lock:
            serial_obj = self.serial
            if serial_obj is None:
                raise RuntimeError("connect explicitly before sending")
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            host_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            if self.h743_clock_offset_ms is not None:
                timestamp_ms = (host_ms - self.h743_clock_offset_ms) & 0xFFFFFFFF
            else:
                timestamp_ms = host_ms
            frame = Frame(frame_type, self.sequence, self.session,
                          timestamp_ms, payload)
            raw = encode_frame(frame)
            serial_obj.write(raw)
            if self.tx_log is not None:
                self.tx_log.write(raw)
                self.tx_log.flush()

    def hello(self) -> None:
        if not self.connected:
            raise RuntimeError("connect explicitly before HELLO")
        # A HELLO starts a new link handshake.  Do not let a previous session
        # authorize commands while the new session token is unknown.
        self.session = 0
        self.mode_complete = False
        self.requested_mode = None
        self.images.reset()
        self.h743_clock_offset_ms = None
        self.hello_nonce = (int(time.monotonic() * 1000) ^ 0xCA7E357) & 0xFFFFFFFF
        self._send(FrameType.HELLO, struct.pack("<I", self.hello_nonce))

    def command(self, opcode: int, target: int = 0, data: bytes = b"") -> None:
        if self.session == 0:
            raise RuntimeError("click HELLO and wait for HELLO_REPLY first")
        if opcode not in (OP_MODE, OP_STOP) and not self.mode_complete:
            raise RuntimeError("click HELLO, wait for reply, then select Complete mode")
        if opcode == OP_MODE and data != bytes((1,)):
            self.mode_complete = False
        if opcode == OP_MODE and data:
            self.requested_mode = data[0]
        self._send(FrameType.COMMAND, pack_command(opcode, target, data))

    def close(self) -> None:
        self.disconnect()
        self.close_log()


class WirelessApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CarTest Wireless Diagnostic")
        self.geometry("920x640")
        self.queue: queue.Queue[TelemetryEvent | tuple[str, str]] = queue.Queue(maxsize=256)
        self.client = WirelessClient(self.queue)
        self.vars: dict[str, tk.StringVar] = {}
        self.mcu_state: dict[str, object] = {}
        self.camera_state: dict[str, object] = {}
        self.config_state: dict[str, object] = {}
        self.h743_config: dict[str, object] = {}
        self.camera_config: dict[str, object] = {}
        self.last_ack: dict[str, object] = {}
        self.plot_history = deque(maxlen=180)
        self.curve_kind = tk.StringVar(value="wheel")
        self._build()
        self.after(100, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="COM").pack(side="left")
        self.port = tk.StringVar(value="COM1")
        ttk.Entry(top, textvariable=self.port, width=10).pack(side="left", padx=4)
        ttk.Label(top, text="baud").pack(side="left")
        self.baud = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud, width=8).pack(side="left", padx=4)
        ttk.Label(top, text="curve").pack(side="left", padx=(10, 2))
        ttk.Combobox(top, textvariable=self.curve_kind,
                     values=("wheel", "gyro", "vision"), state="readonly",
                     width=8).pack(side="left")
        ttk.Button(top, text="Connect", command=self._connect).pack(side="left", padx=3)
        ttk.Button(top, text="Disconnect", command=self.client.disconnect).pack(side="left", padx=3)
        ttk.Button(top, text="HELLO", command=self._hello).pack(side="left", padx=3)
        ttk.Button(top, text="Complete mode", command=lambda: self._command(
            OP_MODE, 0, bytes((1,)))).pack(side="left", padx=3)
        ttk.Button(top, text="Assistant mode", command=lambda: self._command(
            OP_MODE, 0, bytes((0,)))).pack(side="left", padx=3)
        ttk.Button(top, text="START", command=lambda: self._command(OP_START)).pack(side="left", padx=3)
        ttk.Button(top, text="STOP", command=lambda: self._command(OP_STOP)).pack(side="left", padx=3)
        ttk.Button(top, text="Save log", command=self._log).pack(side="left", padx=3)

        command_bar = ttk.Frame(self, padding=(8, 0))
        command_bar.pack(fill="x")
        ttk.Button(command_bar, text="Info", command=lambda: self._command(OP_INFO)).pack(side="left")
        ttk.Button(command_bar, text="Get config", command=lambda: self._command(OP_GET_PARAMS)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Save RAM config", command=lambda: self._command(OP_SAVE)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Defaults", command=lambda: self._command(OP_DEFAULT)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="IMU calibrate", command=lambda: self._command(OP_IMU_CAL)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Camera calibrate", command=lambda: self._command(OP_CAMERA_CAL, 1)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Capture parked image", command=lambda: self._command(OP_CAPTURE, 1)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Camera get", command=lambda: self._command(OP_GET_PARAMS, 1)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Camera save", command=lambda: self._command(OP_SAVE, 1)).pack(side="left", padx=3)
        ttk.Button(command_bar, text="Camera defaults", command=lambda: self._command(OP_DEFAULT, 1)).pack(side="left", padx=3)

        controls = ttk.Frame(self, padding=(8, 4))
        controls.pack(fill="x")
        ttk.Label(controls, text="SET group/id/value").grid(row=0, column=0, sticky="w")
        self.param_group = tk.StringVar(value="1")
        self.param_id = tk.StringVar(value="1")
        self.param_value = tk.StringVar(value="1000")
        for variable, width, column in ((self.param_group, 5, 1),
                                        (self.param_id, 5, 2),
                                        (self.param_value, 10, 3)):
            ttk.Entry(controls, textvariable=variable, width=width).grid(
                row=0, column=column, padx=2)
        ttk.Button(controls, text="Apply RAM parameter", command=self._set_parameter).grid(
            row=0, column=4, padx=4)
        ttk.Label(controls, text="motor duty/duration").grid(row=1, column=0, sticky="w")
        self.motor_duty = tk.StringVar(value="400")
        self.motor_duration = tk.StringVar(value="1000")
        ttk.Entry(controls, textvariable=self.motor_duty, width=7).grid(row=1, column=1)
        ttk.Entry(controls, textvariable=self.motor_duration, width=7).grid(row=1, column=2)
        for label, motor in (("L test", 1), ("R test", 2), ("Both test", 3)):
            ttk.Button(controls, text=label,
                       command=lambda motor=motor: self._motor_test(motor)).grid(
                           row=1, column=3 + motor, padx=2)
        ttk.Label(controls, text="servo us/ms").grid(row=2, column=0, sticky="w")
        self.servo_us = tk.StringVar(value="1530")
        self.servo_duration = tk.StringVar(value="1000")
        ttk.Entry(controls, textvariable=self.servo_us, width=7).grid(row=2, column=1)
        ttk.Entry(controls, textvariable=self.servo_duration, width=7).grid(row=2, column=2)
        ttk.Button(controls, text="Servo test", command=self._servo_test).grid(
            row=2, column=3, padx=4)
        self.page = tk.StringVar(value="0")
        ttk.Label(controls, text="page").grid(row=2, column=4, sticky="e")
        ttk.Entry(controls, textvariable=self.page, width=5).grid(row=2, column=5)
        ttk.Button(controls, text="Scope page", command=self._page).grid(row=2, column=6, padx=4)

        self.curve_canvas = tk.Canvas(self, height=180, background="#101820",
                                     highlightthickness=1, highlightbackground="#486070")
        self.curve_canvas.pack(fill="x", padx=8, pady=(4, 0))
        self.state_text = tk.Text(self, height=10, width=110, state="disabled")
        self.state_text.pack(fill="both", expand=True, padx=8, pady=(4, 0))
        self.ack_text = tk.Text(self, height=3, width=110, state="disabled")
        self.ack_text.pack(fill="x", padx=8, pady=(4, 0))
        self.image_label = ttk.Label(self, text="No image received")
        self.image_label.pack(fill="x", padx=8)
        bottom = ttk.Frame(self, padding=8)
        bottom.pack(fill="x")
        self.status = tk.StringVar(value="Disconnected; click Connect explicitly")
        ttk.Label(bottom, textvariable=self.status).pack(side="left")
        ttk.Label(bottom, text="  Mode: assistant and complete tool use the COM exclusively").pack(side="right")

    def _connect(self) -> None:
        try:
            self.client.connect(self.port.get().strip(), int(self.baud.get()))
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def _hello(self) -> None:
        try:
            self.client.hello()
        except Exception as exc:
            messagebox.showerror("HELLO failed", str(exc))

    def _command(self, opcode: int, target: int = 0, data: bytes = b"") -> None:
        try:
            self.client.command(opcode, target, data)
        except Exception as exc:
            messagebox.showerror("Command failed", str(exc))

    def _set_parameter(self) -> None:
        try:
            group = int(self.param_group.get())
            parameter = int(self.param_id.get())
            value = int(self.param_value.get())
            self._command(OP_SET_PARAMS, 0,
                          struct.pack("<BBBi", group, 1, parameter, value))
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Parameter failed", str(exc))

    def _motor_test(self, motor: int) -> None:
        try:
            duty = int(self.motor_duty.get())
            duration = int(self.motor_duration.get())
            self._command(OP_TEST_MOTOR, 0, struct.pack("<BHH", motor, duty, duration))
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Motor test failed", str(exc))

    def _servo_test(self) -> None:
        try:
            pulse = int(self.servo_us.get())
            duration = int(self.servo_duration.get())
            self._command(OP_TEST_SERVO, 0, struct.pack("<HH", pulse, duration))
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Servo test failed", str(exc))

    def _page(self) -> None:
        try:
            page = int(self.page.get())
            if not 0 <= page <= 255:
                raise ValueError("page must be 0..255")
            self._command(OP_PAGE, 0, bytes((page,)))
        except (TypeError, ValueError) as exc:
            messagebox.showerror("Page failed", str(exc))

    def _log(self) -> None:
        directory = filedialog.askdirectory(title="Choose local log directory")
        if directory:
            try:
                self.client.set_log_directory(Path(directory))
            except Exception as exc:
                messagebox.showerror("Log failed", str(exc))

    def _render_states(self) -> None:
        lines = ["H743 latest STATE"]
        if self.mcu_state:
            lines.extend(f"{key}: {value}" for key, value in self.mcu_state.items())
        else:
            lines.append("waiting for H743 STATE")
        lines.append("")
        lines.append("K230 latest STATE/DIAG")
        if self.camera_state:
            lines.extend(f"{key}: {value}" for key, value in self.camera_state.items())
        else:
            lines.append("waiting for K230 telemetry")
        if self.h743_config:
            lines.append("")
            lines.append("H743 CONFIG")
            lines.extend(f"{key}: {value}" for key, value in self.h743_config.items())
        if self.camera_config:
            lines.append("")
            lines.append("K230 CONFIG")
            lines.extend(f"{key}: {value}" for key, value in self.camera_config.items())
        self.state_text.configure(state="normal")
        self.state_text.delete("1.0", "end")
        self.state_text.insert("end", "\n".join(lines))
        self.state_text.configure(state="disabled")

    def _render_ack(self) -> None:
        lines = ["Latest ACK"]
        if self.last_ack:
            lines.extend(f"{key}: {value}" for key, value in self.last_ack.items())
        else:
            lines.append("waiting for ACK")
        self.ack_text.configure(state="normal")
        self.ack_text.delete("1.0", "end")
        self.ack_text.insert("end", "\n".join(lines))
        self.ack_text.configure(state="disabled")

    def _draw_curve(self) -> None:

        canvas = self.curve_canvas

        canvas.delete("all")

        width = max(320, int(canvas.winfo_width()))

        height = max(120, int(canvas.winfo_height()))

        left, right, top, bottom = 8, width - 8, 8, height - 18

        canvas.create_line(left, bottom, right, bottom, fill="#486070")

        canvas.create_line(left, top, left, bottom, fill="#486070")

        kind = self.curve_kind.get()

        if kind == "gyro":

            series = (("gyro actual", "gyro_actual", "#54b7ff"),

                      ("config +rate", "gyro_config_plus", "#ffad5a"),

                      ("config -rate", "gyro_config_minus", "#ff6b6b"))

            unit = "mdps"

        elif kind == "vision":

            series = (("vision error", "vision_error", "#ff6b6b"),

                      ("vision target", "vision_target", "#73d68a"))

            unit = "error"

        else:

            series = (("L actual", "wheel_l_actual", "#54b7ff"),

                      ("L target", "wheel_l_target", "#73d68a"),

                      ("R actual", "wheel_r_actual", "#ffad5a"),

                      ("R target", "wheel_r_target", "#ff6b6b"))

            unit = "millirps"

        if len(self.plot_history) < 2:

            canvas.create_text(left + 4, top + 4, anchor="nw",

                               fill="#b8c8d0", text=f"Waiting for {kind} telemetry")

            return

        values = [float(sample[key]) for sample in self.plot_history for _, key, _ in series]

        maximum = max(1.0, max(abs(value) for value in values))

        center = (top + bottom) / 2.0

        scale = (bottom - top) / 2.0 / maximum

        canvas.create_line(left, center, right, center, fill="#344b58")

        for index, (name, key, color) in enumerate(series):

            points = []

            for position, sample in enumerate(self.plot_history):

                x = left + (right - left) * position / max(1, len(self.plot_history) - 1)

                y = center - float(sample[key]) * scale

                points.extend((x, max(top, min(bottom, y))))

            canvas.create_line(*points, fill=color, width=1.5, smooth=False)

            canvas.create_text(right - 4 - index * 78, top + 3, anchor="ne",

                               fill=color, text=name)

        canvas.create_text(left + 4, top + 3, anchor="nw",

                           fill="#b8c8d0", text=f"{kind} +/- {maximum:g} {unit}")


    def _poll(self) -> None:
        try:
            while True:
                item = self.queue.get_nowait()
                if isinstance(item, tuple):
                    self.status.set(item[1])
                    continue
                decoded = dict(item.decoded) if item.decoded is not None else {}
                complete_image = decoded.pop("complete_image", None)
                if isinstance(complete_image, bytes):
                    self._show_image(decoded, complete_image)
                if item.frame.type == FrameType.STATE:

                    if len(item.frame.payload) == 128 and decoded:

                        self.mcu_state.update(decoded)

                        actual_gyro = int(decoded.get("imu_z_mdps", 0))
                        configured_rate = abs(int(self.h743_config.get(
                            "gyro_rate_mdps", 0)))

                        self.plot_history.append({

                            "timestamp": item.frame.timestamp_ms,

                            "wheel_l_actual": int(decoded.get("measured_left_millirps", 0)),

                            "wheel_l_target": int(decoded.get("target_left_millirps", 0)),

                            "wheel_r_actual": int(decoded.get("measured_right_millirps", 0)),

                            "wheel_r_target": int(decoded.get("target_right_millirps", 0)),

                            "gyro_actual": actual_gyro,

                            "gyro_config_plus": configured_rate,

                            "gyro_config_minus": -configured_rate,

                            "vision_error": int(decoded.get("vision_error", 0)),

                            "vision_target": 0,

                        })

                    elif len(item.frame.payload) == 36 and decoded:

                        self.camera_state.update(decoded)

                elif item.frame.type == FrameType.DIAG and len(item.frame.payload) == 32 and decoded:

                    self.camera_state.update(decoded)
                elif item.frame.type == FrameType.ACK:
                    self.last_ack = decoded
                elif item.frame.type == FrameType.CONFIG:
                    self.config_state = decoded
                    if decoded.get("config_target") == "h743":
                        self.h743_config = decoded
                    else:
                        self.camera_config = decoded
            
        except queue.Empty:
            pass
        self._render_states()
        self._render_ack()
        self._draw_curve()
        self.after(100, self._poll)

    def _show_image(self, meta: dict[str, object], image: bytes) -> None:
        try:
            width, height = int(meta["width"]), int(meta["height"])
            if int(meta["format"]) == 1:
                stride = (width + 7) // 8
                expanded = bytearray()
                for row in range(height):
                    packed = image[row * stride:(row + 1) * stride]
                    for x in range(width):
                        expanded.append(255 if packed[x // 8] & (0x80 >> (x & 7)) else 0)
                image = bytes(expanded)
            if len(image) != width * height:
                raise ValueError("image size does not match metadata")
            pgm = f"P5\n{width} {height}\n255\n".encode("ascii") + image
            self.photo = tk.PhotoImage(data=base64.b64encode(pgm).decode("ascii"))
            self.image_label.configure(image=self.photo, text="")
        except Exception as exc:
            self.status.set(f"image display error: {exc}")

    def _close(self) -> None:
        self.client.close()
        self.destroy()


def main() -> None:
    WirelessApp().mainloop()


if __name__ == "__main__":
    main()
