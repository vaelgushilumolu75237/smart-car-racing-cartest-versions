"""Exercise parked command dispatch inside the actual finite camera loop."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_fast_main_ring_slope as runtime
from test_entry_release_contract import collect
import config
import main
import visual_entry_gate as visual
import wireless_sideband as wire


class ParkedMainCommandTests(unittest.TestCase):
    def test_real_parked_main_commit_reports_both_files_and_partial_failure(self):
        for fail_encoder in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                collector, _ = collect(directory)
                visual_path = str(Path(directory) / 'visual.json')
                encoder_path = str(Path(directory) / 'encoder.json')
                completions = []

                class Sideband(wire.WirelessSideband):
                    def __init__(self, *args, **kwargs):
                        super().__init__(*args, **kwargs)
                        self.encoder_entry_request = dict(action=2, side=1, marker=0)
                    def encoder_entry_complete(self, now, status, detail=0, generation=None):
                        completions.append((status, detail))
                        self.encoder_entry_request = None

                main._load_fast_runtime()
                main._load_singleedge_runtime()
                main._load_track_elements_runtime()
                clock = runtime._Time()
                controls = (bytes(main.reverse_control.pack_frame(0, 100)) +
                    bytes(main.reverse_control.pack_frame(1, 101)) +
                    bytes(main.reverse_control.pack_profile_state(11, 3, 0,
                        session_token=7, mean_forward_wheel_millirev=600)))
                uart = runtime._UART(clock, rx_data=controls)
                class PassiveElements(runtime._ActiveRingElements):
                    def observe_parked(self, *args, **kwargs):
                        return {}
                with contextlib.ExitStack() as stack:
                    patches = (
                        (main, '_configure_gray_sensor', lambda *_a, **_k: 1),
                        (main, 'time', clock),
                        (main, '_validate_selected_fast_boundary',
                         lambda selection, _road, _validator: (selection,
                             {'x': 50.0, 'y': 40.0, 'kind': 'ROAD'},
                             {'valid': True, 'reason': 'TEST_VALID'})),
                        (main.fast_gray, 'FastGrayMasks', runtime._GrayBuilder),
                        (main.fast_path, 'FastPathDetector', runtime._PathDetector),
                        (main.fast_boundary, 'FastBoundaryEvidence', runtime._BoundaryValidator),
                        (main.fast_single_edge, 'FastSingleEdgeTracker', runtime._SingleEdgeTracker),
                        (main.track_elements, 'TrackElementsTracker', PassiveElements),
                        (wire, 'WirelessSideband', Sideband),
                        (visual, 'VisualEntryCalibrationCollector', lambda: collector),
                        (config, 'RING_VISUAL_ENTRY_CALIBRATION_PATH', visual_path),
                        (config, 'ENCODER_ENTRY_CALIBRATION_PATH', encoder_path),
                        (config, 'DEBUG_ENABLED', False),
                        (config, 'OVERLAY_ENABLED', False),
                    )
                    for target, name, value in patches:
                        stack.enter_context(patch.object(target, name, value, create=True))
                    if fail_encoder:
                        stack.enter_context(patch.object(visual.VisualEntryCalibration,
                            'save_encoder', side_effect=OSError('SD companion failed')))
                    stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                    main._main_fast(duration_s=1, frame_source=runtime._Sensor(),
                                    uart_sink=uart)
                expected = (wire.ACK_NOT_READY, 1) if fail_encoder else (wire.ACK_OK, 3)
                self.assertEqual(completions, [expected])
                self.assertTrue(Path(visual_path).is_file())
                self.assertEqual(Path(encoder_path).is_file(), not fail_encoder)
                self.assertEqual((uart.frames[0][10] >> 2) & 3, 0)


if __name__ == '__main__':
    unittest.main()
