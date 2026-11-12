"""C inside-only speed request through actual V3 main-loop packing."""
import unittest
import test_fast_main_ring_slope as wire
import main
import config


class RingCSpeedRequestTests(unittest.TestCase):
    def packet(self, profile, state, valid=True):
        main._load_track_elements_runtime()
        class Ring(wire._ActiveRingElements):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.result.update(profile_id=profile, state_id=state,
                    state=str(state), ring_confirmed=True,
                    target_valid=valid, control_eligible=valid)
        return wire.FastMainRingSlopeWireTests()._run_profile_element_case(
            Ring, profile)[0]

    def test_c_inside_twenty_percent_faster_request_in_actual_uart_packet(self):
        main._load_track_elements_runtime()
        p = self.packet(3, main.track_elements.STATE_INSIDE)
        self.assertTrue(p[6] & 1)
        self.assertEqual(p[11], 3)
        self.assertEqual(p[10], main.vision_protocol.CURVE_PHASE_TURN)
        self.assertEqual(int.from_bytes(p[12:14], 'little'), 267)

    def test_c_entry_exit_and_reacquire_keep_previous_request(self):
        main._load_track_elements_runtime()
        for state in (main.track_elements.STATE_EXIT_CONFIRM,
                      main.track_elements.STATE_EXIT_REPAIR,
                      main.track_elements.STATE_REACQUIRE):
            p = self.packet(3, state)
            self.assertTrue(p[6] & 1)
            self.assertEqual(int.from_bytes(p[12:14], 'little'), 1000)
        self.assertEqual(main._ring_slowdown_request(
            3, main.track_elements.STATE_ENTER), 1000)

    def test_b_inside_keeps_previous_full_request(self):
        main._load_track_elements_runtime()
        p = self.packet(2, main.track_elements.STATE_INSIDE)
        self.assertTrue(p[6] & 1)
        self.assertEqual(p[11], 2)
        self.assertEqual(int.from_bytes(p[12:14], 'little'), 1000)

    def test_invalid_ring_cannot_send_speed_request(self):
        main._load_track_elements_runtime()
        p = self.packet(3, main.track_elements.STATE_INSIDE, valid=False)
        self.assertFalse(p[6] & 1)
        self.assertEqual(int.from_bytes(p[12:14], 'little'), 0)

    def test_unknown_state_does_not_get_speed_increase(self):
        main._load_track_elements_runtime()
        self.assertEqual(main._ring_slowdown_request(
            3, main.track_elements.STATE_INSIDE, True), 1000)
        for state in (None, -1, 100):
            self.assertEqual(main._ring_slowdown_request(3, state), 1000)


if __name__ == '__main__':
    unittest.main()
