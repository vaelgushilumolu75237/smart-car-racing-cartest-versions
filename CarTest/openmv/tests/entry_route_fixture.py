"""Explicit prior-admission fixture for legacy downstream route tests.

These tests exercise ring target construction/UART ownership after the entry
certificate has already been accepted. Production entrance admission is tested
separately with real calibration in tests/test_visual_entry_gate.py. This
fixture is installed only by an individual test's patch context.
"""

def previously_admitted_tracker(tracker_type):
    """Declare prior admission before the first frame's observation scan."""
    def create(*args, **kwargs):
        tracker = tracker_type(*args, **kwargs)
        if tracker.result.get('ring_confirmed'):
            tracker.result['entry_ready'] = True
            tracker.result.setdefault('state_id', 3)
        return tracker
    return create


class PreviouslyAdmittedEntrance:
    def __init__(self, calibration=None, **kwargs):
        self.calibration = calibration

    def reset(self, reason='TEST_RESET'):
        pass

    def update(self, info, **kwargs):
        admitted = bool(info.get('ring_confirmed'))
        return {'ready': admitted, 'active': admitted,
                'entry_evidence': admitted,
                'state': 2 if admitted else 0, 'side': info.get('side', -1),
                'match_frames': 3 if admitted else 0,
                'calibration_ready': True, 'threshold_millirev': 200, 'progress_millirev': 200,
                'reason': 'TEST_PRIOR_PHYSICAL_ENTRY' if admitted else 'TEST_OBSERVE'}
