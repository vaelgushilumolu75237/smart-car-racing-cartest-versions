"""V4 camera evidence accompanying (never replacing) route validity.

The MCU requests release after 350 degrees. Camera then releases old ring
ownership so ordinary roads can be observed without the old exit state chain.
"""
import vision_protocol

CAP_GYRO = 0x04
RELEASE_RING = 0x08


class GyroLink:
    def __init__(self, required=True):
        self.required = required
        self.count = 0
        self.side = 0
        self.last_sequence = None
        self.last_ms = None
        self.identity = None
        self.entry_side = 0

    @staticmethod
    def capable(parser, now_ms, ticks_diff):
        return bool(parser.profile_is_fresh(now_ms, ticks_diff) and
                    parser.profile_id == 3 and parser.profile_flags & CAP_GYRO)

    def release(self, parser, now_ms, ticks_diff):
        return bool(self.capable(parser, now_ms, ticks_diff) and
                    parser.profile_flags & RELEASE_RING)

    def transform(self, packet, parser, evidence, ordinary, now_ms,
                  ticks_diff, budget_ok=True, calibrated=True):
        capable = self.capable(parser, now_ms, ticks_diff)
        if not capable:
            self.count = 0
            self.entry_side = 0
            if self.required:
                # Unknown/old MCU cannot accidentally run the previous vision
                # ring policy. Keep an invalid V3 heartbeat with ACK0.
                seq = packet[4] | (packet[5] << 8)
                vision_protocol.pack_into_v3(packet, seq, 0, 0, 0, profile_ack=0)
            return packet
        identity = (parser.generation, parser.profile_generation, parser.session_token)
        if identity != self.identity:
            self.identity = identity
            self.count = 0
            self.side = 0
            self.last_sequence = None
            self.last_ms = None
            self.entry_side = 0
        phase = 0
        side = 0
        seq = packet[4] | (packet[5] << 8)
        ring_side = evidence.get("side_name", "NONE")
        candidate_side = -1 if ring_side == "LEFT" else 1 if ring_side == "RIGHT" else 0
        session_ok = parser.profile_session_active and parser.session_token != 0
        fresh_sequence = (self.last_sequence is None or
                          (0 < ((seq - self.last_sequence) & 0xffff) < 0x8000 and
                           self.last_ms is not None and
                           0 < ticks_diff(now_ms, self.last_ms) < 250))
        if self.release(parser, now_ms, ticks_diff) or not session_ok:
            self.entry_side = 0
        if (budget_ok and calibrated and session_ok and candidate_side and
                packet[6] & vision_protocol.FLAG_VALID and
                not self.release(parser, now_ms, ticks_diff)):
            state = evidence.get("state_id")
            confirmed = (state == 3 and evidence.get("ring_confirmed") and
                         evidence.get("target_valid") and
                         evidence.get("entry_confirmation_path") in
                         ("GROWING_ARC", "PEAK_REVERSAL"))
            # ENTER means sustained missing boundary, not a confirmed ring.
            # Only the tracker's subsequent measured arc confirmation may
            # authorize entry. Never upgrade an invalid route to an actuator
            # command, or send INSIDE before ENTRY in this session.
            if confirmed and fresh_sequence and self.entry_side == 0:
                phase, side = 1, candidate_side
                self.entry_side = candidate_side
                self.count = 0
            elif confirmed and self.entry_side == candidate_side:
                phase, side = 1, candidate_side
                adjacent = (self.last_sequence is not None and
                            ((seq - self.last_sequence) & 0xffff) == 1 and
                            self.last_ms is not None and
                            0 < ticks_diff(now_ms, self.last_ms) < 250 and
                            self.side == candidate_side)
                if evidence.get("entry_transition"):
                    self.count = 0
                else:
                    self.count = min(3, self.count + 1) if adjacent else 1
                if self.count >= 3:
                    phase, side = 2, candidate_side
            else:
                self.count = 0
        else:
            self.count = 0
        self.side = candidate_side
        self.last_sequence, self.last_ms = seq, now_ms
        packet[2] = 4
        side_bits = 1 if side < 0 else 2 if side > 0 else 0
        packet[10] = ((packet[10] & 3) | (phase << 2) | (side_bits << 4) |
                      (64 if ordinary and packet[6] & 1 and budget_ok else 0))
        crc = vision_protocol._crc16_ccitt_false_range(packet, 2, 14)
        packet[14], packet[15] = crc & 255, crc >> 8
        return packet
