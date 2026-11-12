"""Safety gate from OpenMV ring observations to the H743 gyro controller.

The link deliberately has a smaller contract than the image tracker. It only
accepts a fresh, confirmed INSIDE observation from a gyro-enabled B/C session.
The returned dictionary is allocated once and reused on every call so the
frame loop does not create a per-frame result object.
"""

PHASE_OFF = 0
PHASE_ENTRY = 1
PHASE_ORBIT = 2

# Wire-side values used by the V4 camera frame: OFF=0, LEFT=-1, RIGHT=+1.
SIDE_NONE = 0
SIDE_LEFT = -1
SIDE_RIGHT = 1

# track_elements.py uses NONE=-1, LEFT=0, RIGHT=1 internally.
TRACK_SIDE_NONE = -1
TRACK_SIDE_LEFT = 0
TRACK_SIDE_RIGHT = 1

# These values mirror track_elements.py. ENTER is a candidate state and must
# never arm the gyro link; INSIDE is the first confirmed ring state.
STATE_ENTER = 2
STATE_INSIDE = 3

PROFILE_B = 2
PROFILE_C = 3
PROFILE_SESSION_ACTIVE = 0x01
PROFILE_GYRO_SUPPORTED = 0x04
PROFILE_GYRO_RELEASE = 0x08

ORDINARY_RECOVERY_FRAMES = 3


class GyroLink:
    """Translate accepted ring evidence into phase 1/2 gyro authority."""

    def __init__(self, ordinary_recovery_frames=ORDINARY_RECOVERY_FRAMES):
        if (isinstance(ordinary_recovery_frames, bool) or
                not isinstance(ordinary_recovery_frames, int) or
                ordinary_recovery_frames < 1):
            raise ValueError("ordinary_recovery_frames must be a positive int")
        self.ordinary_recovery_frames = ordinary_recovery_frames
        self._result = {"phase": PHASE_OFF, "side": SIDE_NONE,
                        "ordinary": False}
        self._phase = PHASE_OFF
        self._side = TRACK_SIDE_NONE
        self._post_entry_frames = 0
        self._entry_transition_seen = False
        self._entry_transition_ended = False
        self._identity_valid = False
        self._profile_id = 0
        self._session_active = False
        self._session_token = 0
        self._profile_flags = 0
        self._lease_known = False
        self._lease_fresh = False
        self._suppressed = False
        self._release_latched = False
        self._ordinary_recovery_count = 0

    def _clear_event(self):
        self._phase = PHASE_OFF
        self._side = TRACK_SIDE_NONE
        self._post_entry_frames = 0
        self._entry_transition_seen = False
        self._entry_transition_ended = False

    def _clear_session(self):
        self._clear_event()
        self._suppressed = False
        self._release_latched = False
        self._ordinary_recovery_count = 0

    def reset(self):
        """Clear all remembered authority and session suppression."""
        self._clear_session()
        self._identity_valid = False
        self._lease_known = False
        self._lease_fresh = False

    def _sync_result(self, ordinary):
        result = self._result
        result["phase"] = self._phase
        if self._phase == PHASE_OFF or self._side == TRACK_SIDE_NONE:
            result["side"] = SIDE_NONE
        elif self._side == TRACK_SIDE_LEFT:
            result["side"] = SIDE_LEFT
        else:
            result["side"] = SIDE_RIGHT
        result["ordinary"] = bool(ordinary)
        return result

    @staticmethod
    def _profile_is_fresh(parser, now_ms, ticks_diff):
        check = getattr(parser, "profile_is_fresh", None)
        if callable(check):
            return bool(check(now_ms, ticks_diff))
        last_ms = getattr(parser, "_profile_last_ms", None)
        if last_ms is None:
            return False
        timeout_ms = int(getattr(parser, "profile_timeout_ms", 250))
        age_ms = ticks_diff(now_ms, last_ms)
        return 0 <= age_ms < timeout_ms

    @staticmethod
    def _control_is_fresh(parser, now_ms, ticks_diff):
        check = getattr(parser, "control_is_fresh", None)
        if callable(check):
            return bool(check(now_ms, ticks_diff))
        return bool(getattr(parser, "single_enabled", False))

    @staticmethod
    def _ring_observation(ring_info):
        if not isinstance(ring_info, dict):
            return False, False, TRACK_SIDE_NONE, False
        state_id = ring_info.get("state_id", -1)
        state = ring_info.get("state")
        try:
            state_id = int(state_id)
        except (TypeError, ValueError):
            state_id = -1
        inside = state_id == STATE_INSIDE or state == "INSIDE"
        # The tracker can confirm a ring before the vehicle reaches the
        # physical entrance.  The visual-entry gate is the authority that
        # decides when that observation may become gyro evidence.
        confirmed = bool(ring_info.get("ring_confirmed")) and \
            bool(ring_info.get("entry_ready"))
        try:
            side = int(ring_info.get("side", TRACK_SIDE_NONE))
        except (TypeError, ValueError):
            side = TRACK_SIDE_NONE
        side_valid = side in (TRACK_SIDE_LEFT, TRACK_SIDE_RIGHT)
        entry_transition = bool(ring_info.get("entry_transition"))
        # The lease is only a physical-entry certificate when the current
        # frame also carries a fresh transition geometry.  A stale INSIDE
        # frame must not be able to arm ENTRY merely by echoing a prior gate
        # result.
        entry_evidence = bool(confirmed and entry_transition and
                              ring_info.get("entry_evidence"))
        return bool(confirmed and inside and side_valid), entry_transition, side, entry_evidence

    @staticmethod
    def _has_confirmed_ring(ring_info):
        confirmed, _, _, evidence = GyroLink._ring_observation(ring_info)
        return bool(confirmed and evidence)

    @staticmethod
    def _safe_ordinary(valid, ordinary_valid, ring_info):
        return bool(valid and ordinary_valid and
                    not GyroLink._has_confirmed_ring(ring_info))

    def _ordinary_recovery(self, valid, ordinary_valid, ring_info,
                           release_requested):
        if not self._suppressed:
            return
        if self._safe_ordinary(valid, ordinary_valid, ring_info):
            self._ordinary_recovery_count += 1
            if self._ordinary_recovery_count >= self.ordinary_recovery_frames:
                self._suppressed = False
                self._ordinary_recovery_count = 0
                # A level-held release bit is one event. Do not immediately
                # consume it again after ordinary recovery.
                if not release_requested:
                    self._release_latched = False
        else:
            self._ordinary_recovery_count = 0

    def update(self, parser, ring_info, valid, now_ms, ticks_diff,
               ordinary_valid=False):
        """Return the reusable phase/side/ordinary result.

        valid is the freshness/validity of this camera frame. A false frame
        never claims ordinary vision, but an already armed phase and side
        remain available to the H743 until its transport lease expires.
        ordinary_valid is only exposed when the current frame is valid and no
        confirmed ring observation owns the frame.
        """
        profile_id = int(getattr(parser, "profile_id", 0))
        profile_flags = int(getattr(parser, "profile_flags", 0))
        session_token = int(getattr(parser, "session_token", 0))
        session_active = bool(getattr(
            parser, "profile_session_active",
            bool((profile_flags & PROFILE_SESSION_ACTIVE) and
                 session_token != 0)))
        profile_fresh = self._profile_is_fresh(parser, now_ms, ticks_diff)
        lease_fresh = self._control_is_fresh(parser, now_ms, ticks_diff)

        identity_changed = bool(
            self._identity_valid and
            (profile_id != self._profile_id or
             session_active != self._session_active or
             session_token != self._session_token or
             profile_flags != self._profile_flags))
        if identity_changed:
            self._clear_session()
        self._identity_valid = True
        self._profile_id = profile_id
        self._session_active = session_active
        self._session_token = session_token
        self._profile_flags = profile_flags

        if self._lease_known and lease_fresh != self._lease_fresh:
            self._clear_event()
            self._ordinary_recovery_count = 0
        self._lease_known = True
        self._lease_fresh = lease_fresh

        gyro_enabled = bool(
            profile_fresh and lease_fresh and session_active and
            profile_id in (PROFILE_B, PROFILE_C) and session_token != 0 and
            (profile_flags & PROFILE_GYRO_SUPPORTED))
        release_requested = bool(
            gyro_enabled and (profile_flags & PROFILE_GYRO_RELEASE))

        if release_requested and not self._release_latched:
            self._release_latched = True
            self._suppressed = True
            self._clear_event()

        # A session exit is the only identity transition that explicitly
        # releases the post-350 suppression latch.
        if not session_active or profile_id not in (PROFILE_B, PROFILE_C) or \
                session_token == 0:
            self._clear_session()
            return self._sync_result(bool(valid and ordinary_valid))

        self._ordinary_recovery(valid, ordinary_valid, ring_info,
                                release_requested)
        if self._suppressed:
            return self._sync_result(
                self._safe_ordinary(valid, ordinary_valid, ring_info))

        if not gyro_enabled:
            self._clear_event()
            return self._sync_result(bool(valid and ordinary_valid))

        if (not isinstance(ring_info, dict) or
                not bool(ring_info.get("entry_ready"))):
            self._clear_event()
            return self._sync_result(bool(valid and ordinary_valid))

        if not valid:
            if self._phase == PHASE_ENTRY:
                self._post_entry_frames = 0
            return self._sync_result(False)

        confirmed, entry_transition, side, entry_evidence = \
            self._ring_observation(ring_info)
        if confirmed:
            if self._phase == PHASE_OFF:
                # The first calibrated frame only arms the visual gate.  The
                # next fresh transition certificate is required to start the
                # physical entry phase; an old INSIDE state cannot advance
                # the yaw origin into ORBIT.
                if not entry_evidence:
                    return self._sync_result(bool(valid and ordinary_valid))
                self._phase = PHASE_ENTRY
                self._side = side
                self._post_entry_frames = 0
                self._entry_transition_seen = True
                self._entry_transition_ended = False
            elif self._phase == PHASE_ENTRY:
                if side != self._side:
                    # A new confirmed direction cannot advance the old entry
                    # lease. Start a fresh phase-1 observation instead.
                    self._phase = PHASE_ENTRY
                    self._side = side
                    self._post_entry_frames = 0
                    self._entry_transition_seen = True
                    self._entry_transition_ended = False
                elif entry_transition:
                    self._entry_transition_seen = True
                    self._entry_transition_ended = False
                    self._post_entry_frames = 0
                elif self._entry_transition_seen:
                    self._entry_transition_ended = True
                    self._post_entry_frames += 1
                    if (self._entry_transition_ended and
                            self._post_entry_frames >= 3):
                        self._phase = PHASE_ORBIT
            # Once orbit is established, a transient opposite-side frame does
            # not revoke the gyro side; the H743 keeps its independent orbit.
            return self._sync_result(False)

        if self._phase == PHASE_ENTRY:
            self._post_entry_frames = 0
        return self._sync_result(bool(ordinary_valid and self._phase == PHASE_OFF))


__all__ = (
    "GyroLink", "ORDINARY_RECOVERY_FRAMES", "PHASE_ENTRY", "PHASE_OFF",
    "PHASE_ORBIT", "SIDE_LEFT", "SIDE_NONE", "SIDE_RIGHT",
    "TRACK_SIDE_LEFT", "TRACK_SIDE_NONE", "TRACK_SIDE_RIGHT",
)
