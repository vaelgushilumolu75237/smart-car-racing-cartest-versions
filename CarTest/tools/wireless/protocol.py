"""Portable implementation of the bounded D3/91 wireless protocol.

The H743 implementation in ``App/wireless_protocol.c`` is the authority for
the embedded wire format.  This file deliberately uses only the standard
library so it can be used by the complete PC tool without a package install.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
import zlib


MAGIC = b"\xD3\x91"
VERSION = 1
HEADER_SIZE = 20
CRC_SIZE = 4
MAX_PAYLOAD = 160
MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE
IMAGE_MAX_BYTES = 76800


class FrameType(IntEnum):
    HELLO = 1
    HELLO_REPLY = 2
    COMMAND = 3
    ACK = 4
    STATE = 16
    DIAG = 17
    IMAGE = 18
    CONFIG = 19
    EVENT = 20


class ProtocolError(ValueError):
    """Malformed, oversized, or inconsistent protocol data."""


@dataclass(frozen=True)
class Frame:
    type: int
    sequence: int
    session: int
    timestamp_ms: int
    payload: bytes = b""

    def __post_init__(self) -> None:
        if not 0 <= self.type <= 255:
            raise ProtocolError("frame type outside uint8")
        for name, value in (("sequence", self.sequence),
                            ("session", self.session),
                            ("timestamp_ms", self.timestamp_ms)):
            if not 0 <= value <= 0xFFFFFFFF:
                raise ProtocolError(f"{name} outside uint32")
        if len(self.payload) > MAX_PAYLOAD:
            raise ProtocolError("payload exceeds 160 bytes")


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_frame(frame: Frame) -> bytes:
    header = struct.pack(
        "<2sBBHHIII", MAGIC, VERSION, frame.type, len(frame.payload), 0,
        frame.sequence, frame.session, frame.timestamp_ms
    )
    return header + frame.payload + struct.pack("<I", crc32(header + frame.payload))


def decode_frame(data: bytes) -> Frame:
    if len(data) < HEADER_SIZE + CRC_SIZE:
        raise ProtocolError("frame is shorter than the fixed envelope")
    if data[:2] != MAGIC or data[2] != VERSION:
        raise ProtocolError("bad magic or version")
    _, _, kind, payload_len, reserved, sequence, session, timestamp = \
        struct.unpack_from("<2sBBHHIII", data)
    if reserved:
        raise ProtocolError("reserved header bits are nonzero")
    if payload_len > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 160 bytes")
    expected = HEADER_SIZE + payload_len + CRC_SIZE
    if len(data) != expected:
        raise ProtocolError("frame length does not match payload length")
    payload = data[HEADER_SIZE:HEADER_SIZE + payload_len]
    got, = struct.unpack_from("<I", data, HEADER_SIZE + payload_len)
    if got != crc32(data[:HEADER_SIZE + payload_len]):
        raise ProtocolError("CRC32 mismatch")
    return Frame(kind, sequence, session, timestamp, payload)


class FrameParser:
    """Allocation-bounded stream parser with no resynchronization scans."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._expected: int | None = None
        self.frames_ok = 0
        self.sync_errors = 0
        self.length_errors = 0
        self.crc_errors = 0
        self.reserved_errors = 0

    def reset(self) -> None:
        self._buffer.clear()
        self._expected = None

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        result: list[Frame] = []
        for byte in bytes(data):
            frame = self._feed_byte(byte)
            if frame is not None:
                result.append(frame)
        return result

    def _restart(self, byte: int) -> None:
        self._buffer.clear()
        self._expected = None
        if byte == MAGIC[0]:
            self._buffer.append(byte)

    def _feed_byte(self, byte: int) -> Frame | None:
        if not self._buffer:
            if byte == MAGIC[0]:
                self._buffer.append(byte)
            return None
        if len(self._buffer) == 1 and byte != MAGIC[1]:
            self.sync_errors += 1
            self._restart(byte)
            return None
        if len(self._buffer) >= MAX_FRAME:
            self.length_errors += 1
            self._restart(byte)
            return None
        self._buffer.append(byte)
        if len(self._buffer) == HEADER_SIZE:
            if self._buffer[2] != VERSION:
                self.sync_errors += 1
                self._restart(byte)
                return None
            reserved = int.from_bytes(self._buffer[6:8], "little")
            if reserved:
                self.reserved_errors += 1
                self._restart(byte)
                return None
            payload_len = int.from_bytes(self._buffer[4:6], "little")
            if payload_len > MAX_PAYLOAD:
                self.length_errors += 1
                self._restart(byte)
                return None
            self._expected = HEADER_SIZE + payload_len + CRC_SIZE
        if self._expected is not None and len(self._buffer) == self._expected:
            raw = bytes(self._buffer)
            self.reset()
            try:
                frame = decode_frame(raw)
            except ProtocolError:
                self.crc_errors += 1
                return None
            self.frames_ok += 1
            return frame
        return None


def sequence_forward(sequence: int, previous: int) -> bool:
    delta = (sequence - previous) & 0xFFFFFFFF
    return delta != 0 and delta < 0x80000000


def pack_command(opcode: int, target: int = 0, data: bytes = b"") -> bytes:
    """Build the contract's generic command payload, without a frame header."""
    if not 0 <= opcode <= 255 or not 0 <= target <= 255:
        raise ProtocolError("command opcode/target outside uint8")
    if len(data) > MAX_PAYLOAD - 4:
        raise ProtocolError("command data exceeds payload budget")
    return struct.pack("<BBH", opcode, target, 0) + data


class SeekFree:
    """Exact compatibility bytes from SeekFree's public assistant source.

    The source is pinned in docs/WIRELESS_PC_PROTOCOL.md.  These helpers use
    little-endian non-STC fields, matching the H743/K230 UART path.
    """

    SEND_HEAD = 0xAA
    RECEIVE_HEAD = 0x55
    CAMERA_CMD = 0x02
    OSCILLOSCOPE_CMD = 0x10
    DEBUG_PARAM_CMD = 0x20
    CAMERA_BINARY = 0x01
    CAMERA_GRAY = 0x02
    CAMERA_RGB565 = 0x03

    @staticmethod
    def _sum(data: bytes) -> int:
        return sum(data) & 0xFF

    @classmethod
    def oscilloscope_config(cls, channels: int, data_type: int = 0) -> bytes:
        channels = 1 if channels == 0 else min(channels, 16)
        body = bytes((cls.OSCILLOSCOPE_CMD, channels, data_type, 0, 0, 0))
        return bytes((cls.SEND_HEAD, (cls.SEND_HEAD + cls._sum(body)) & 0xFF)) + body

    @classmethod
    def oscilloscope(cls, values: list[float] | tuple[float, ...]) -> bytes:
        if not 1 <= len(values) <= 16:
            raise ProtocolError("SeekFree oscilloscope channels must be 1..16")
        config = cls.oscilloscope_config(len(values))
        data = b"".join(struct.pack("<f", float(value)) for value in values)
        raw = bytearray(config + data)
        raw[1] = (cls.SEND_HEAD + cls._sum(raw[2:])) & 0xFF
        return bytes(raw)

    @classmethod
    def debug_param(cls, channel: int, value: float) -> bytes:
        if not 0 <= channel < 8:
            raise ProtocolError("SeekFree parameter channel must be 0..7")
        raw = bytearray((cls.RECEIVE_HEAD, 0, cls.DEBUG_PARAM_CMD, channel))
        raw += struct.pack("<f", float(value))
        raw[1] = (0xFF + cls._sum(raw[:1] + raw[2:])) & 0xFF
        return bytes(raw)

    @classmethod
    def camera(cls, camera_type: int, width: int, height: int,
               image: bytes | bytearray | memoryview) -> bytes:
        if camera_type not in (cls.CAMERA_BINARY, cls.CAMERA_GRAY,
                               cls.CAMERA_RGB565):
            raise ProtocolError("unsupported SeekFree camera type")
        expected = width * height
        if camera_type == cls.CAMERA_BINARY:
            expected = (expected + 7) // 8
        elif camera_type == cls.CAMERA_RGB565:
            expected *= 2
        image = bytes(image)
        if len(image) != expected or not 0 <= width <= 0xFFFF or not 0 <= height <= 0xFFFF:
            raise ProtocolError("SeekFree image dimensions/data mismatch")
        body = bytes((cls.CAMERA_CMD, camera_type, width & 0xFF, width >> 8,
                      height & 0xFF, height >> 8, 1, 0, 0, 0))
        return bytes((cls.SEND_HEAD, (cls.SEND_HEAD + cls._sum(body)) & 0xFF)) + body + image


__all__ = ["Frame", "FrameParser", "FrameType", "ProtocolError", "SeekFree",
           "MAX_PAYLOAD", "MAX_FRAME", "IMAGE_MAX_BYTES", "crc32", "decode_frame",
           "encode_frame", "pack_command", "sequence_forward"]
