"""H743-to-OpenMV lifecycle control frames for the V1 vision stream.

The control stream is independent from :mod:`vision_protocol`: it never
changes the fixed 12-byte observation frame. The parser keeps a single
8-byte candidate buffer and accepts at most 32 input bytes per feed call.
"""

RUNTIME_API_VERSION = 1

SYNC_0 = 0xA6
SYNC_1 = 0x5A
VERSION = 1
COMMAND_RESET = 0
COMMAND_KEEP = 1
FRAME_SIZE = 8
MAX_FEED_BYTES = 32
DEFAULT_TIMEOUT_MS = 250


def crc16_ccitt_false(data, length):
    """Return CRC-16/CCITT-FALSE over the first ``length`` bytes."""
    crc = 0xFFFF
    for index in range(length):
        crc ^= int(data[index]) << 8
        for _bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_frame(command, sequence):
    """Build one complete 8-byte control frame for shared host vectors."""
    if (isinstance(command, bool) or not isinstance(command, int) or
            command not in (COMMAND_RESET, COMMAND_KEEP)):
        raise ValueError("command must be RESET(0) or KEEP(1)")
    if (isinstance(sequence, bool) or not isinstance(sequence, int) or
            sequence < 0 or sequence > 0xFFFF):
        raise ValueError("sequence must be an unsigned 16-bit integer")
    frame = bytearray(FRAME_SIZE)
    frame[0] = SYNC_0
    frame[1] = SYNC_1
    frame[2] = VERSION
    frame[3] = command
    frame[4] = sequence & 0xFF
    frame[5] = sequence >> 8
    crc = crc16_ccitt_false(frame, 6)
    frame[6] = crc & 0xFF
    frame[7] = crc >> 8
    return frame


class ReverseControlParser:
    """Bounded parser and freshness state for RESET/KEEP controls.

    A valid RESET arms a control session but does not enable single-edge
    steering. A later forward-sequence KEEP enables the cached-width route.
    Duplicate KEEP frames do not refresh the heartbeat. Any malformed frame,
    sequence rollback, or 250 ms control gap clears the session and increments
    ``generation`` so the caller can clear all dependent inference state.
    """

    def __init__(self, timeout_ms=DEFAULT_TIMEOUT_MS):
        if (isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or
                timeout_ms < 1 or timeout_ms > 10000):
            raise ValueError("timeout_ms must be in 1..10000")
        self.timeout_ms = timeout_ms
        self._frame = bytearray(FRAME_SIZE)
        self._frame_count = 0
        self._partial_ms = None
        self._sequence_valid = False
        self._last_sequence = 0
        self._last_ms = None
        self._armed = False
        self.single_enabled = False
        self.generation = 0
        self.last_event = "STARTUP"
        self.accepted_resets = 0
        self.accepted_keeps = 0
        self.duplicate_keeps = 0
        self.rejected_frames = 0

    @property
    def buffered_bytes(self):
        return self._frame_count

    def _bump_generation(self):
        self.generation = (self.generation + 1) & 0x7FFFFFFF

    def _invalidate(self, reason):
        self._frame_count = 0
        self._partial_ms = None
        self._sequence_valid = False
        self._last_ms = None
        self._armed = False
        self.single_enabled = False
        self.last_event = reason
        self._bump_generation()

    def invalidate(self, reason="RX_ERROR"):
        """Invalidate the reverse channel after a UART read failure."""
        self._invalidate(reason)
        self.rejected_frames += 1

    def poll(self, now_ms, ticks_diff):
        """Expire a control session after a missing heartbeat."""
        if self._last_ms is None:
            if self._partial_ms is not None:
                partial_age = ticks_diff(now_ms, self._partial_ms)
                if partial_age < 0 or partial_age >= self.timeout_ms:
                    self._invalidate("PARTIAL_TIMEOUT")
                    return True
            return False
        age_ms = ticks_diff(now_ms, self._last_ms)
        if age_ms < 0 or age_ms >= self.timeout_ms:
            self._invalidate("CONTROL_TIMEOUT")
            return True
        return False

    def _accept_complete_frame(self, now_ms):
        frame = self._frame
        if frame[2] != VERSION:
            self.rejected_frames += 1
            self._invalidate("BAD_VERSION")
            return
        command = frame[3]
        if command not in (COMMAND_RESET, COMMAND_KEEP):
            self.rejected_frames += 1
            self._invalidate("BAD_COMMAND")
            return
        expected_crc = crc16_ccitt_false(frame, 6)
        actual_crc = int(frame[6]) | (int(frame[7]) << 8)
        if actual_crc != expected_crc:
            self.rejected_frames += 1
            self._invalidate("BAD_CRC")
            return

        sequence = int(frame[4]) | (int(frame[5]) << 8)
        if command == COMMAND_RESET:
            # RESET remains safe and idempotent even when repeated with the
            # same sequence. It deliberately clears downstream caches every
            # time while establishing this frame as the new sequence base.
            self._sequence_valid = True
            self._last_sequence = sequence
            self._last_ms = now_ms
            self._armed = True
            self.single_enabled = False
            self.accepted_resets += 1
            self.last_event = "RESET"
            self._bump_generation()
            return

        if not self._armed or not self._sequence_valid:
            self.rejected_frames += 1
            self._invalidate("KEEP_WITHOUT_RESET")
            return
        delta = (sequence - self._last_sequence) & 0xFFFF
        if delta == 0:
            self.duplicate_keeps += 1
            self.last_event = "DUPLICATE_KEEP"
            return
        if delta >= 0x8000:
            self.rejected_frames += 1
            self._invalidate("SEQUENCE_ROLLBACK")
            return

        self._last_sequence = sequence
        self._last_ms = now_ms
        self.single_enabled = True
        self.accepted_keeps += 1
        self.last_event = "KEEP"

    def _feed_byte(self, value, now_ms):
        count = self._frame_count
        if count == 0:
            if value == SYNC_0:
                self._frame[0] = value
                self._frame_count = 1
                self._partial_ms = now_ms
            return
        if count == 1:
            if value == SYNC_1:
                self._frame[1] = value
                self._frame_count = 2
            elif value == SYNC_0:
                self._frame[0] = value
                self._frame_count = 1
                self._partial_ms = now_ms
            else:
                self._frame_count = 0
            return

        self._frame[count] = value
        count += 1
        if count == FRAME_SIZE:
            self._frame_count = 0
            self._partial_ms = None
            self._accept_complete_frame(now_ms)
        else:
            self._frame_count = count

    def feed(self, buffer, length, now_ms):
        """Consume no more than 32 bytes from a caller-owned buffer."""
        if (isinstance(length, bool) or not isinstance(length, int) or
                length < 0 or length > len(buffer)):
            raise ValueError("length must be within the supplied buffer")
        if length > MAX_FEED_BYTES:
            length = MAX_FEED_BYTES
        for index in range(length):
            self._feed_byte(int(buffer[index]), now_ms)
        return length


__all__ = (
    "COMMAND_KEEP", "COMMAND_RESET", "DEFAULT_TIMEOUT_MS", "FRAME_SIZE",
    "MAX_FEED_BYTES", "RUNTIME_API_VERSION", "ReverseControlParser",
    "crc16_ccitt_false", "pack_frame",
)
