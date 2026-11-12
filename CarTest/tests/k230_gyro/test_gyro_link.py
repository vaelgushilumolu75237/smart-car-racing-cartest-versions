import json
import sys
import unittest
import contextlib
import io
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OPENMV = ROOT / "openmv"
if str(OPENMV) not in sys.path:
    sys.path.insert(0, str(OPENMV))
OPENMV_TESTS = OPENMV / "tests"
if str(OPENMV_TESTS) not in sys.path:
    sys.path.insert(0, str(OPENMV_TESTS))

import config
import gyro_link
import reverse_control
import track_elements
import vision_protocol


def ticks_diff(now, then):
    return now - then


def profile_parser(flags, token=0x1234, now_ms=0):
    parser = reverse_control.ReverseControlParser()
    frame = reverse_control.pack_profile_state(
        1, reverse_control.PROFILE_C, flags,
        session_token=token,
    )
    parser.feed(frame, len(frame), now_ms)
    return parser


def vision_frame(sequence, valid=1, curve_phase=2, profile_ack=3):
    return bytearray(vision_protocol.pack_frame_v3(
        sequence, valid, 80 if valid else 0, 120 if valid else 0,
        curve_phase=curve_phase if valid else 0, profile_ack=profile_ack,
    ))


def metadata(packet):
    packed = packet[10]
    side_bits = (packed >> 4) & 3
    return {
        "version": packet[2],
        "curve_phase": packed & 3,
        "ring_phase": (packed >> 2) & 3,
        "side_bits": side_bits,
        "ordinary": (packed >> 6) & 1,
        "crc_ok": (
            int(packet[14]) | (int(packet[15]) << 8)
        ) == vision_protocol._crc16_ccitt_false_range(packet, 2, 14),
    }


class GyroLinkProtocolTests(unittest.TestCase):
    def test_real_dropout_state_cannot_authorize_gyro_on_ordinary_bend(self):
        from unittest.mock import patch
        tracker = track_elements.TrackElementsTracker(require_calibration=False)
        tracker._side = track_elements.SIDE_LEFT
        tracker._state = track_elements.STATE_ENTRY_CONFIRM
        tracker._drop_count = 1
        tracker._current_valid = True
        with patch.object(tracker, '_topology_clear', return_value=True), \
                patch.object(tracker, '_ordinary_corridor_frame', return_value=False), \
                patch.object(tracker, '_entry_dropout_evidence', return_value=(True, 15, 5)):
            tracker._confirm_entry_drop(20, {})
        self.assertEqual(tracker._state, track_elements.STATE_ENTER)
        tracker._publish()
        self.assertFalse(tracker.result['ring_confirmed'])
        parser = profile_parser(5)
        link = gyro_link.GyroLink()
        # Repeat the actual dropout-only state through multiple profile leases.
        # Both left and right valid single-edge routes remain ordinary.
        for side in ('LEFT', 'RIGHT'):
            for i in range(1, 10):
                now = i * 15
                evidence = dict(tracker.result, side_name=side)
                packet = vision_frame(i if side == 'LEFT' else i + 10)
                link.transform(packet, parser, evidence, True, now, ticks_diff)
                self.assertEqual(metadata(packet)['ring_phase'], 0)
                self.assertEqual(metadata(packet)['ordinary'], 1)
                self.assertTrue(metadata(packet)['crc_ok'])

    def test_unconfirmed_arc_missing_target_or_invalid_frame_never_starts_entry(self):
        parser = profile_parser(5)
        for changes in ({'entry_confirmation_path': 'NONE'},
                        {'ring_confirmed': False}, {'target_valid': False},
                        {'state_id': 2}, {'invalid': True}):
            link = gyro_link.GyroLink()
            evidence = {'state_id': 3, 'side_name': 'LEFT',
                        'ring_confirmed': True, 'target_valid': True,
                        'entry_confirmation_path': 'PEAK_REVERSAL'}
            evidence.update(changes)
            for i in range(4):
                packet = vision_frame(i, valid=not changes.get('invalid', False))
                link.transform(packet, parser, evidence, True, i * 20, ticks_diff)
                self.assertEqual(metadata(packet)['ring_phase'], 0, changes)

    def test_confirmed_arc_cannot_start_from_repeated_or_backward_packet(self):
        parser = profile_parser(5)
        for sequence in (10, 9):
            link = gyro_link.GyroLink()
            link.transform(vision_frame(10), parser,
                           {'state_id': 2, 'side_name': 'LEFT'}, True, 0, ticks_diff)
            packet = vision_frame(sequence)
            evidence = {'state_id': 3, 'side_name': 'LEFT',
                        'ring_confirmed': True, 'target_valid': True,
                        'entry_confirmation_path': 'GROWING_ARC'}
            link.transform(packet, parser, evidence, True, 20, ticks_diff)
            self.assertEqual(metadata(packet)['ring_phase'], 0)

    def test_main_keeps_verified_single_edge_routes_in_ordinary_authority(self):
        """Exercise the real dual/single main loop and inspect its V4 input."""
        from unittest.mock import patch
        import main
        import test_singleedge_main as runtime

        calls = {}
        base = gyro_link.GyroLink

        class RecordingLink(base):
            def transform(self, packet, parser, evidence, ordinary, now_ms,
                          ticks_diff, **kwargs):
                seq = packet[4] | (packet[5] << 8)
                calls[seq] = (bool(packet[6] & 1), ordinary)
                return super().transform(packet, parser, evidence, ordinary,
                                         now_ms, ticks_diff, **kwargs)

        # This existing fixture drives real validated bilateral/left-edge
        # masks, parser and main loop. Legacy capability mode preserves its
        # original wire assertions; the recorded ordinary argument is the
        # production decision passed to the V4 transformer.
        with patch.object(config, 'GYRO350_REQUIRED', False), \
                patch.object(main, 'GyroLink', RecordingLink):
            case = runtime.SingleEdgeMainTests(
                'test_main_pipeline_dual_single_dual_preserves_filter_and_clears_on_error')
            case.test_main_pipeline_dual_single_dual_preserves_filter_and_clears_on_error()
        self.assertEqual(calls[2], (True, True))   # measured bilateral road
        self.assertEqual(calls[3], (True, True))   # verified left-edge route
        self.assertEqual(calls[1], (False, False)) # no width cache yet

    def test_default_build_requires_the_gyro_capability(self):
        self.assertTrue(config.GYRO350_REQUIRED)

    def test_missing_capability_fails_closed_to_invalid_v3(self):
        parser = profile_parser(reverse_control.PROFILE_SESSION_ACTIVE)
        link = gyro_link.GyroLink(required=True)
        packet = vision_frame(7)
        link.transform(packet, parser, {"state_id": 3, "side_name": "LEFT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True},
                       True, 0, ticks_diff)

        self.assertEqual(packet[2], vision_protocol.VERSION_V3)
        self.assertEqual(packet[6], 0)
        self.assertEqual(packet[7], 0)
        self.assertEqual(packet[11], 0)
        self.assertTrue(metadata(packet)["crc_ok"])

    def test_stale_profile_fails_closed_even_when_last_state_was_capable(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        parser.poll(250, ticks_diff)
        link = gyro_link.GyroLink(required=True)
        packet = vision_frame(8)
        link.transform(packet, parser, {"state_id": 3, "side_name": "RIGHT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True},
                       True, 250, ticks_diff)
        self.assertEqual(packet[2], vision_protocol.VERSION_V3)
        self.assertEqual(packet[6], 0)
        self.assertTrue(metadata(packet)["crc_ok"])

    def test_entry_then_three_adjacent_inside_frames_emit_v4_inside(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)

        entry = vision_frame(10)
        link.transform(entry, parser, {"state_id": 3, "side_name": "LEFT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True},
                       True, 0, ticks_diff)
        self.assertEqual(metadata(entry), {
            "version": 4, "curve_phase": 2, "ring_phase": 1,
            "side_bits": 1, "ordinary": 1, "crc_ok": True,
        })

        for sequence, now_ms in ((11, 40), (12, 80)):
            packet = vision_frame(sequence)
            link.transform(packet, parser, {
                "state_id": 3,
                "side_name": "LEFT",
                "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
                "target_valid": True,
                "entry_transition": False,
            }, True, now_ms, ticks_diff)
            self.assertEqual(metadata(packet)["ring_phase"], 1)

        inside = vision_frame(13)
        link.transform(inside, parser, {
            "state_id": 3,
            "side_name": "LEFT",
            "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
            "target_valid": True,
            "entry_transition": False,
        }, True, 120, ticks_diff)
        self.assertEqual(metadata(inside), {
            "version": 4, "curve_phase": 2, "ring_phase": 2,
            "side_bits": 1, "ordinary": 1, "crc_ok": True,
        })

    def test_right_side_and_ordinary_flag_are_mirrored_and_validity_gated(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)
        packet = vision_frame(20, valid=1, curve_phase=1)
        link.transform(packet, parser, {"state_id": 3, "side_name": "RIGHT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True},
                       True, 0, ticks_diff)
        self.assertEqual(metadata(packet)["side_bits"], 2)
        self.assertEqual(metadata(packet)["ordinary"], 1)

        invalid = vision_frame(21, valid=0)
        link.transform(invalid, parser, {"state_id": 3,
                       "side_name": "RIGHT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
                       "target_valid": True, "entry_transition": False},
                       True, 40, ticks_diff)
        self.assertEqual(metadata(invalid)["ordinary"], 0)
        self.assertEqual(metadata(invalid)["ring_phase"], 0)

    def test_budget_calibration_and_tracker_state_gate_inside_evidence(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)
        evidence = {"state_id": 3, "side_name": "LEFT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True}
        packet = vision_frame(30)
        link.transform(packet, parser, evidence, True, 0, ticks_diff,
                       budget_ok=False, calibrated=True)
        self.assertEqual(metadata(packet)["ring_phase"], 0)

        packet = vision_frame(31)
        link.transform(packet, parser, evidence, True, 40, ticks_diff,
                       budget_ok=True, calibrated=False)
        self.assertEqual(metadata(packet)["ring_phase"], 0)

        packet = vision_frame(32)
        link.transform(packet, parser, {
            "state_id": 3, "side_name": "LEFT", "ring_confirmed": False,
            "target_valid": True, "entry_transition": False,
        }, True, 80, ticks_diff)
        self.assertEqual(metadata(packet)["ring_phase"], 0)

    def test_duplicate_and_rollback_sequences_never_confirm_inside(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)
        evidence = {
            "state_id": 3, "side_name": "LEFT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
            "target_valid": True, "entry_transition": False,
        }
        for sequence, now_ms in ((40, 0), (41, 40)):
            packet = vision_frame(sequence)
            link.transform(packet, parser, evidence, True, now_ms, ticks_diff)
            self.assertEqual(metadata(packet)["ring_phase"], 1)

        duplicate = vision_frame(41)
        link.transform(duplicate, parser, evidence, True, 80, ticks_diff)
        self.assertNotEqual(metadata(duplicate)["ring_phase"], 2)

        rollback = vision_frame(39)
        link.transform(rollback, parser, evidence, True, 120, ticks_diff)
        self.assertNotEqual(metadata(rollback)["ring_phase"], 2)

    def test_forward_sequence_wrap_is_accepted_once(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)
        link.transform(vision_frame(0xFFFD), parser,
                        {"state_id": 3, "side_name": "RIGHT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True, "entry_transition": True},
                        True, 0, ticks_diff)
        evidence = {
            "state_id": 3, "side_name": "RIGHT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
            "target_valid": True, "entry_transition": False,
        }
        for sequence, now_ms in ((0xFFFE, 40), (0xFFFF, 80), (0, 120)):
            packet = vision_frame(sequence)
            link.transform(packet, parser, evidence, True, now_ms, ticks_diff)
        self.assertEqual(metadata(packet)["ring_phase"], 2)

    def test_profile_identity_change_restarts_inside_streak(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        parser = profile_parser(flags, token=1)
        link = gyro_link.GyroLink(required=True)
        evidence = {
            "state_id": 3, "side_name": "LEFT", "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True,
            "target_valid": True, "entry_transition": False,
        }
        for sequence, now_ms in ((49, 0), (50, 20), (51, 40), (52, 80)):
            packet = vision_frame(sequence)
            link.transform(packet, parser, evidence, True, now_ms, ticks_diff)
        self.assertEqual(link.count, 3)

        changed = reverse_control.pack_profile_state(
            2, reverse_control.PROFILE_C, flags, session_token=2)
        parser.feed(changed, len(changed), 120)
        packet = vision_frame(53)
        link.transform(packet, parser, evidence, True, 120, ticks_diff)
        self.assertEqual(metadata(packet)["ring_phase"], 1)
        self.assertEqual(link.count, 0)

    def test_release_flag_is_fresh_and_clears_old_ring_owner(self):
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED |
                 reverse_control.PROFILE_GYRO350_RELEASE)
        parser = profile_parser(flags)
        link = gyro_link.GyroLink(required=True)
        self.assertTrue(link.release(parser, 0, ticks_diff))

        tracker = track_elements.TrackElementsTracker(require_calibration=False)
        tracker._state = track_elements.STATE_INSIDE
        tracker._ring_cache_frozen = True
        tracker.gyro_exit_override = True
        result = tracker.update(
            100, 3, True, 1, True, False, None, 0, 4, [], {},
            False, None, None, None, False, None, None, None, None,
        )
        self.assertEqual(result["state_id"], track_elements.STATE_OBSERVE)
        self.assertEqual(result["reason"], "GYRO_EXIT_RELEASE")
        self.assertFalse(result["ring_confirmed"])

        parser.poll(250, ticks_diff)
        self.assertFalse(link.release(parser, 250, ticks_diff))

    def test_checked_in_v4_fixture_matches_crc_and_metadata_layout(self):
        fixture_path = Path(__file__).parent / "fixtures" / "v4_cases.json"
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        for case in fixture["cases"]:
            packet = bytearray(case["bytes"])
            self.assertEqual(bytes(packet).hex(), case["hex"])
            decoded = metadata(packet)
            self.assertEqual(decoded["version"], 4)
            self.assertEqual(decoded["ring_phase"], case["ring_phase"])
            self.assertEqual(decoded["side_bits"], case["side_bits"])
            self.assertTrue(decoded["crc_ok"])

    def test_checked_in_v4_fixture_regenerates_through_gyro_link(self):
        fixture_path = Path(__file__).parent / "fixtures" / "v4_cases.json"
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                 reverse_control.PROFILE_GYRO350_SUPPORTED)
        for case in fixture["cases"]:
            parser = profile_parser(flags)
            link = gyro_link.GyroLink(required=True)
            side = "RIGHT" if case["name"].startswith("right") else "LEFT"
            valid = case["name"] != "invalid_route_entry"
            sequence = case["input_sequence"]
            packet = vision_frame(sequence, valid=valid)
            link.transform(packet, parser,
                           {"state_id": 3, "side_name": side, "ring_confirmed": True, "target_valid": True, "entry_confirmation_path": "GROWING_ARC", "entry_transition": True},
                           True, 0, ticks_diff)
            if "inside" in case["name"]:
                evidence = {
                    "state_id": 3, "side_name": side,
                    "entry_confirmation_path": "GROWING_ARC", "ring_confirmed": True, "target_valid": True,
                    "entry_transition": False,
                }
                for offset, now_ms in ((1, 40), (2, 80), (3, 120)):
                    packet = vision_frame(sequence + offset, valid=valid)
                    link.transform(packet, parser, evidence, True,
                                   now_ms, ticks_diff)
            if case["name"] == "invalid_route_entry":
                self.assertEqual(metadata(packet)["ring_phase"], 0)
                self.assertEqual(metadata(packet)["ordinary"], 0)
            else:
                self.assertEqual(bytes(packet).hex(), case["hex"], case["name"])

    def test_main_fast_final_uart_write_emits_v4_with_capable_profile(self):
        from unittest.mock import patch
        import main
        import test_fast_main_ring_slope as runtime

        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()
        tracker_module = main.track_elements
        instances = []

        class Feedback(runtime._ConfirmedRingNoTarget):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                instances.append(self)

            def update(self, *args, **kwargs):
                return self.result

        clock = runtime._Time()
        profile_flags = (reverse_control.PROFILE_SESSION_ACTIVE |
                         reverse_control.PROFILE_GYRO350_SUPPORTED)
        controls = (
            bytes(main.reverse_control.pack_frame(0, 100)) +
            bytes(main.reverse_control.pack_frame(1, 101)) +
            bytes(main.reverse_control.pack_profile_state(
                11, 3, profile_flags, session_token=1))
        )
        uart = runtime._UART(clock, stop_after=3, rx_data=controls)
        with contextlib.ExitStack() as stack:
            for target, name, value in (
                (main, "_configure_gray_sensor", lambda *_a, **_k: 1),
                (main, "sensor", runtime._Sensor()),
                (main, "time", clock),
                (main.fast_gray, "FastGrayMasks", runtime._GrayBuilder),
                (main.fast_path, "FastPathDetector", runtime._PathDetector),
                (main.fast_boundary, "FastBoundaryEvidence",
                 runtime._BoundaryValidator),
                (main.fast_single_edge, "FastSingleEdgeTracker",
                 runtime._SingleEdgeTracker),
                (main.track_elements, "TrackElementsTracker", Feedback),
                (main.pyb, "UART", lambda *_a, **_k: uart),
                (main, "_validate_selected_fast_boundary",
                 lambda selection, _r, _v: (
                     selection, {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                     {"valid": True, "reason": "TEST_VALID"})),
                (config, "DEBUG_ENABLED", False),
                (config, "OVERLAY_ENABLED", False),
                (config, "GYRO350_REQUIRED", True),
            ):
                stack.enter_context(patch.object(target, name, value,
                                                 create=True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            main._main_fast(duration_s=1)

        self.assertEqual(len(instances), 1)
        self.assertEqual(len(uart.frames), 3)
        self.assertTrue(all(frame[2] == 4 for frame in uart.frames))
        self.assertTrue(all(
            (int(frame[14]) | (int(frame[15]) << 8)) ==
            vision_protocol._crc16_ccitt_false_range(frame, 2, 14)
            for frame in uart.frames
        ))

    def test_main_fast_required_mode_fails_closed_without_capability(self):
        from unittest.mock import patch
        import main
        import test_fast_main_ring_slope as runtime

        main._load_fast_runtime()
        main._load_singleedge_runtime()
        main._load_track_elements_runtime()

        class Feedback(runtime._ConfirmedRingNoTarget):
            def update(self, *args, **kwargs):
                return self.result

        clock = runtime._Time()
        controls = (
            bytes(main.reverse_control.pack_frame(0, 200)) +
            bytes(main.reverse_control.pack_frame(1, 201)) +
            bytes(main.reverse_control.pack_profile_state(
                12, 3, reverse_control.PROFILE_SESSION_ACTIVE,
                session_token=1))
        )
        uart = runtime._UART(clock, stop_after=3, rx_data=controls)
        with contextlib.ExitStack() as stack:
            for target, name, value in (
                (main, "_configure_gray_sensor", lambda *_a, **_k: 1),
                (main, "sensor", runtime._Sensor()),
                (main, "time", clock),
                (main.fast_gray, "FastGrayMasks", runtime._GrayBuilder),
                (main.fast_path, "FastPathDetector", runtime._PathDetector),
                (main.fast_boundary, "FastBoundaryEvidence",
                 runtime._BoundaryValidator),
                (main.fast_single_edge, "FastSingleEdgeTracker",
                 runtime._SingleEdgeTracker),
                (main.track_elements, "TrackElementsTracker", Feedback),
                (main.pyb, "UART", lambda *_a, **_k: uart),
                (main, "_validate_selected_fast_boundary",
                 lambda selection, _r, _v: (
                     selection, {"x": 50.0, "y": 40.0, "kind": "ROAD"},
                     {"valid": True, "reason": "TEST_VALID"})),
                (config, "DEBUG_ENABLED", False),
                (config, "OVERLAY_ENABLED", False),
                (config, "GYRO350_REQUIRED", True),
            ):
                stack.enter_context(patch.object(target, name, value,
                                                 create=True))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            main._main_fast(duration_s=1)

        self.assertEqual(len(uart.frames), 3)
        for frame in uart.frames:
            self.assertEqual(frame[2], vision_protocol.VERSION_V3)
            self.assertEqual(frame[6], 0)
            self.assertEqual(frame[11], 0)
            self.assertEqual(
                int(frame[14]) | (int(frame[15]) << 8),
                vision_protocol._crc16_ccitt_false_range(frame, 2, 14),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
