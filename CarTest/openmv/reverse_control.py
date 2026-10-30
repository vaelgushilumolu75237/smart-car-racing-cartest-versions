"""Bounded H743-to-OpenMV RESET/KEEP and profile-state parser.

V1 RESET/KEEP frames remain eight bytes. V2 profile-state frames are fixed
twenty-byte records on the same byte stream and use an independent sequence
and 250 ms lease. Parser storage is allocated once; each feed is capped.
"""

RUNTIME_API_VERSION = 2

SYNC_0 = 0xA6
SYNC_1 = 0x5A
VERSION = 1
PROFILE_VERSION = 2
PROFILE_STATE_TYPE = 2
COMMAND_RESET = 0
COMMAND_KEEP = 1
FRAME_SIZE = 8
PROFILE_FRAME_SIZE = 20
MAX_FRAME_SIZE = PROFILE_FRAME_SIZE
MAX_FEED_BYTES = 64
DEFAULT_TIMEOUT_MS = 250
PROFILE_TIMEOUT_MS = 250

PROFILE_UNKNOWN = 0
PROFILE_A = 1
PROFILE_B = 2
PROFILE_C = 3
PROFILE_SESSION_ACTIVE = 0x01
PROFILE_DIAGNOSTICS_SUPPORTED = 0x02
PROFILE_FLAGS_MASK = (PROFILE_SESSION_ACTIVE |
                      PROFILE_DIAGNOSTICS_SUPPORTED)
PROFILE_REVISION = 1


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


def _require_integer(value, name, minimum, maximum):
    if (isinstance(value, bool) or not isinstance(value, int) or
            value < minimum or value > maximum):
        raise ValueError(name + " is outside its supported range")


def pack_frame(command, sequence):
    """Build one complete 8-byte V1 RESET/KEEP control frame."""
    _require_integer(command, "command", COMMAND_RESET, COMMAND_KEEP)
    _require_integer(sequence, "sequence", 0, 0xFFFF)
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


def pack_profile_state(sequence, profile, flags=0,
                       revision=PROFILE_REVISION, session_token=0,
                       mean_forward_wheel_millirev=0):
    """Build the 20-byte V2 status frame with CRC over bytes 0..17.

    Layout: sync, version=2, type=2, independent sequence, profile byte,
    flags byte, revision u16, Follow session token u32, and mean forward wheel
    millirevolutions u32. Profile 0 and unknown profile values are encodable
    so compatibility/fail-closed behavior can be tested.
    """
    _require_integer(sequence, "sequence", 0, 0xFFFF)
    _require_integer(profile, "profile", 0, 0xFF)
    _require_integer(flags, "flags", 0, 0xFF)
    _require_integer(revision, "revision", 0, 0xFFFF)
    _require_integer(session_token, "session_token", 0, 0xFFFFFFFF)
    _require_integer(mean_forward_wheel_millirev,
                     "mean_forward_wheel_millirev", 0, 0xFFFFFFFF)
    frame = bytearray(PROFILE_FRAME_SIZE)
    frame[0] = SYNC_0
    frame[1] = SYNC_1
    frame[2] = PROFILE_VERSION
    frame[3] = PROFILE_STATE_TYPE
    frame[4] = sequence & 0xFF
    frame[5] = (sequence >> 8) & 0xFF
    frame[6] = profile
    frame[7] = flags
    frame[8] = revision & 0xFF
    frame[9] = (revision >> 8) & 0xFF
    frame[10] = session_token & 0xFF
    frame[11] = (session_token >> 8) & 0xFF
    frame[12] = (session_token >> 16) & 0xFF
    frame[13] = (session_token >> 24) & 0xFF
    frame[14] = mean_forward_wheel_millirev & 0xFF
    frame[15] = (mean_forward_wheel_millirev >> 8) & 0xFF
    frame[16] = (mean_forward_wheel_millirev >> 16) & 0xFF
    frame[17] = (mean_forward_wheel_millirev >> 24) & 0xFF
    crc = crc16_ccitt_false(frame, 18)
    frame[18] = crc & 0xFF
    frame[19] = crc >> 8
    return frame


class ReverseControlParser:
    """Bounded parser and freshness state for both reverse frame versions.

    V1 RESET/KEEP drives the existing single-boundary lease. V2 status frames
    do not renew that lease. A fresh, supported V2 profile is exposed for
    acknowledgement and ring gating. Identity transitions and profile expiry
    have a separate generation so duplicate heartbeats cannot clear the
    ordinary width cache.
    """

    def __init__(self, timeout_ms=DEFAULT_TIMEOUT_MS,
                 profile_timeout_ms=PROFILE_TIMEOUT_MS):
        if (isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or
                timeout_ms < 1 or timeout_ms > 10000):
            raise ValueError("timeout_ms must be in 1..10000")
        if (isinstance(profile_timeout_ms, bool) or
                not isinstance(profile_timeout_ms, int) or
                profile_timeout_ms < 1 or profile_timeout_ms > 10000):
            raise ValueError("profile_timeout_ms must be in 1..10000")
        self.timeout_ms = timeout_ms
        self.profile_timeout_ms = profile_timeout_ms
        self._frame = bytearray(MAX_FRAME_SIZE)
        self._frame_count = 0
        self._frame_expected = 0
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

        self._profile_sequence_valid = False
        self._profile_last_sequence = 0
        self._profile_last_ms = None
        self._profile_state_valid = False
        self.profile_id = PROFILE_UNKNOWN
        self.profile_raw_id = PROFILE_UNKNOWN
        self.profile_flags = 0
        self.profile_revision = 0
        self.session_token = 0
        self.mean_forward_wheel_millirev = 0
        self.profile_generation = 0
        self.last_profile_event = "STARTUP"
        self.accepted_profile_states = 0
        self.duplicate_profile_states = 0
        self.rejected_profile_states = 0
        self._profile_identity_valid = False
        self._identity_profile_id = PROFILE_UNKNOWN
        self._identity_session_active = False
        self._identity_session_token = 0

    @property
    def buffered_bytes(self):
        return self._frame_count

    @property
    def profile_session_active(self):
        return bool(self._profile_state_valid and
                    self.profile_revision == PROFILE_REVISION and
                    self.profile_id in (PROFILE_A, PROFILE_B, PROFILE_C) and
                    (self.profile_flags & PROFILE_SESSION_ACTIVE))

    def control_is_fresh(self, now_ms, ticks_diff):
        """Return true only while a valid RESET/KEEP control lease is fresh."""
        if not self.single_enabled or self._last_ms is None:
            return False
        age_ms = ticks_diff(now_ms, self._last_ms)
        return 0 <= age_ms < self.timeout_ms

    def _bump_generation(self):
        self.generation = (self.generation + 1) & 0x7FFFFFFF

    def _bump_profile_generation(self):
        self.profile_generation = (self.profile_generation + 1) & 0x7FFFFFFF

    def _clear_candidate(self):
        self._frame_count = 0
        self._frame_expected = 0
        self._partial_ms = None

    def _invalidate(self, reason):
        self._clear_candidate()
        self._sequence_valid = False
        self._last_ms = None
        self._armed = False
        self.single_enabled = False
        self.last_event = reason
        self._bump_generation()

    def invalidate(self, reason="RX_ERROR"):
        """Invalidate V1 controls after a UART read/write failure."""
        self._invalidate(reason)
        self.rejected_frames += 1

    def _expire_profile(self):
        self._profile_state_valid = False
        self._profile_sequence_valid = False
        self._profile_last_ms = None
        self.profile_id = PROFILE_UNKNOWN
        self.profile_raw_id = PROFILE_UNKNOWN
        self.profile_flags = 0
        self.profile_revision = 0
        self.session_token = 0
        self.mean_forward_wheel_millirev = 0
        self._profile_identity_valid = False
        self.last_profile_event = "PROFILE_TIMEOUT"
        self._bump_profile_generation()

    def _expire_profile_candidate(self):
        self._clear_candidate()
        self.last_profile_event = "PROFILE_PARTIAL_TIMEOUT"

    def poll(self, now_ms, ticks_diff):
        """Expire either independent lease without conflating their state."""
        changed = False
        if self._profile_state_valid:
            age_ms = ticks_diff(now_ms, self._profile_last_ms)
            if age_ms < 0 or age_ms >= self.profile_timeout_ms:
                self._expire_profile()
                changed = True

        if self._partial_ms is not None:
            partial_age = ticks_diff(now_ms, self._partial_ms)
            is_profile_partial = (self._frame_count > 2 and
                                  self._frame[2] == PROFILE_VERSION)
            partial_timeout = (self.profile_timeout_ms if is_profile_partial
                               else self.timeout_ms)
            if partial_age < 0 or partial_age >= partial_timeout:
                if is_profile_partial:
                    self._expire_profile_candidate()
                else:
                    self._invalidate("PARTIAL_TIMEOUT")
                    return True

        if self._last_ms is None:
            return changed
        age_ms = ticks_diff(now_ms, self._last_ms)
        if age_ms < 0 or age_ms >= self.timeout_ms:
            self._invalidate("CONTROL_TIMEOUT")
            return True
        return changed

    def profile_is_fresh(self, now_ms, ticks_diff):
        """Return true only while the latest accepted V2 state is leased."""
        if not self._profile_state_valid or self._profile_last_ms is None:
            return False
        age_ms = ticks_diff(now_ms, self._profile_last_ms)
        return 0 <= age_ms < self.profile_timeout_ms

    def profile_diagnostics_enabled(self, now_ms, ticks_diff):
        """Require a fresh recognized revision-1 profile and capability bit."""
        return bool(
            self.profile_is_fresh(now_ms, ticks_diff) and
            self.profile_revision == PROFILE_REVISION and
            self.profile_id in (PROFILE_A, PROFILE_B, PROFILE_C) and
            (self.profile_flags & PROFILE_DIAGNOSTICS_SUPPORTED))

    def profile_ack(self, now_ms, ticks_diff):
        """Return A/B/C only for a fresh revision-1 recognized profile."""
        if (not self.profile_is_fresh(now_ms, ticks_diff) or
                self.profile_revision != PROFILE_REVISION or
                self.profile_id not in (PROFILE_A, PROFILE_B, PROFILE_C)):
            return PROFILE_UNKNOWN
        return self.profile_id

    def ring_profile_enabled(self, now_ms, ticks_diff):
        """Require a fresh enabled Follow session on profile B or C."""
        return bool(self.profile_is_fresh(now_ms, ticks_diff) and
                    self.profile_session_active and
                    self.profile_id in (PROFILE_B, PROFILE_C) and
                    self.session_token != 0)

    def _accept_control_frame(self, now_ms):
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
            # RESET is safe and idempotent. It clears existing dependent
            # inference state in the caller even when the sequence repeats.
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

    def _accept_profile_frame(self, now_ms):
        frame = self._frame
        if (frame[2] != PROFILE_VERSION or
                frame[3] != PROFILE_STATE_TYPE):
            self.rejected_profile_states += 1
            self.last_profile_event = "BAD_PROFILE_TYPE"
            return
        expected_crc = crc16_ccitt_false(frame, 18)
        actual_crc = int(frame[18]) | (int(frame[19]) << 8)
        if actual_crc != expected_crc:
            self.rejected_profile_states += 1
            self.last_profile_event = "BAD_PROFILE_CRC"
            return

        flags = int(frame[7])
        if flags & ~PROFILE_FLAGS_MASK:
            self.rejected_profile_states += 1
            self.last_profile_event = "BAD_PROFILE_FLAGS"
            return
        sequence = int(frame[4]) | (int(frame[5]) << 8)
        if self._profile_sequence_valid:
            delta = (sequence - self._profile_last_sequence) & 0xFFFF
            if delta == 0:
                self.duplicate_profile_states += 1
                self.last_profile_event = "DUPLICATE_PROFILE_STATE"
                return
            if delta >= 0x8000:
                self.rejected_profile_states += 1
                self.last_profile_event = "PROFILE_SEQUENCE_ROLLBACK"
                return

        profile = int(frame[6])
        revision = int(frame[8]) | (int(frame[9]) << 8)
        token = (int(frame[10]) | (int(frame[11]) << 8) |
                 (int(frame[12]) << 16) | (int(frame[13]) << 24))
        mean_forward = (int(frame[14]) | (int(frame[15]) << 8) |
                        (int(frame[16]) << 16) | (int(frame[17]) << 24))
        recognized = (revision == PROFILE_REVISION and
                      profile in (PROFILE_A, PROFILE_B, PROFILE_C))
        profile_id = profile if recognized else PROFILE_UNKNOWN
        session_active = bool(recognized and
                              (flags & PROFILE_SESSION_ACTIVE) and token != 0)

        identity_changed = bool(
            self._profile_identity_valid and
            (profile_id != self._identity_profile_id or
             session_active != self._identity_session_active or
             token != self._identity_session_token))
        if identity_changed:
            self._bump_profile_generation()
            if profile_id != self._identity_profile_id:
                self.last_profile_event = "PROFILE_SWITCH"
            else:
                self.last_profile_event = "SESSION_CHANGE"
        else:
            self.last_profile_event = "PROFILE_STATE"

        self._profile_sequence_valid = True
        self._profile_last_sequence = sequence
        self._profile_last_ms = now_ms
        self._profile_state_valid = True
        self.profile_raw_id = profile
        self.profile_id = profile_id
        self.profile_flags = flags if recognized else 0
        self.profile_revision = revision
        self.session_token = token if recognized else 0
        self.mean_forward_wheel_millirev = mean_forward
        self._profile_identity_valid = True
        self._identity_profile_id = profile_id
        self._identity_session_active = session_active
        self._identity_session_token = token if recognized else 0
        self.accepted_profile_states += 1

    def _accept_complete_frame(self, now_ms):
        version = self._frame[2]
        if version == VERSION:
            self._accept_control_frame(now_ms)
        elif version == PROFILE_VERSION:
            self._accept_profile_frame(now_ms)
        else:
            self.rejected_frames += 1
            self._invalidate("BAD_VERSION")

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
                self._clear_candidate()
            return

        self._frame[count] = value
        count += 1
        if count == 3:
            version = self._frame[2]
            if version == VERSION:
                self._frame_expected = FRAME_SIZE
            elif version == PROFILE_VERSION:
                self._frame_expected = PROFILE_FRAME_SIZE
            else:
                self.rejected_frames += 1
                self._invalidate("BAD_VERSION")
                return
        if count == self._frame_expected:
            self._clear_candidate()
            self._accept_complete_frame(now_ms)
        else:
            self._frame_count = count

    def feed(self, buffer, length, now_ms):
        """Consume a bounded prefix from a caller-owned UART buffer."""
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
    "MAX_FEED_BYTES", "MAX_FRAME_SIZE", "PROFILE_A", "PROFILE_B",
    "PROFILE_C", "PROFILE_DIAGNOSTICS_SUPPORTED", "PROFILE_FLAGS_MASK",
    "PROFILE_FRAME_SIZE",
    "PROFILE_REVISION", "PROFILE_SESSION_ACTIVE", "PROFILE_TIMEOUT_MS",
    "PROFILE_UNKNOWN", "PROFILE_VERSION", "RUNTIME_API_VERSION",
    "ReverseControlParser", "crc16_ccitt_false", "pack_frame",
    "pack_profile_state",
)
