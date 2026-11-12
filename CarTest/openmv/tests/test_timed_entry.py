import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visual_entry_gate import TimedEntryGate
from gyro_link import GyroLink

class TimedEntryTests(unittest.TestCase):
    def ring(self, side=1, transition=False):
        return dict(state_id=3, state="INSIDE", ring_confirmed=True,
                    candidate=True, side=side, entry_transition=transition)
    def step(self, gate, now, info=None, **kw):
        return dict(gate.update(self.ring() if info is None else info,
                               profile_id=3, session_token=77, now_ms=now,
                               ticks_diff=lambda a,b:a-b, **kw))
    def test_150ms_boundary_both_directions_no_calibration(self):
        for side in (0,1):
            gate=TimedEntryGate()
            for now in (100,149,200,249):
                self.assertFalse(self.step(gate,now,self.ring(side))["ready"])
            self.assertTrue(self.step(gate,250,self.ring(side))["ready"])
            self.assertTrue(self.step(gate,251,self.ring(side))["ready"])
    def test_candidate_is_not_confirmation(self):
        gate=TimedEntryGate()
        info=self.ring();info.update(state_id=1,ring_confirmed=False)
        self.assertFalse(self.step(gate,0,info)["ready"])
        self.assertFalse(self.step(gate,200,info)["ready"])
        self.assertFalse(self.step(gate,201)["ready"])
        self.assertTrue(self.step(gate,351)["ready"])
    def test_invalid_frame_cancels_pending(self):
        gate=TimedEntryGate();self.step(gate,100)
        self.step(gate,200,valid=False)
        self.assertFalse(self.step(gate,500)["ready"])
    def test_stopped_session_cancels_pending(self):
        gate=TimedEntryGate();self.step(gate,100)
        self.step(gate,200,session_active=False)
        self.assertFalse(self.step(gate,500)["ready"])
    def test_direction_change_does_not_trigger_old_timer(self):
        gate=TimedEntryGate();self.step(gate,100)
        self.assertFalse(self.step(gate,300,self.ring(0))["ready"])
    def test_candidate_clear_allows_new_event(self):
        gate=TimedEntryGate();self.step(gate,100)
        self.step(gate,200,dict(state_id=0,candidate=False,ring_confirmed=False))
        self.assertFalse(self.step(gate,300)["ready"])
        self.assertTrue(self.step(gate,450)["ready"])
    def test_real_gyro_entry_at150_orbit_requires_new_visual_transition(self):
        class Parser:
            profile_id=3; profile_flags=5; session_token=77
            profile_session_active=True
            def profile_is_fresh(self,*args):return True
            def control_is_fresh(self,*args):return True
        gate=TimedEntryGate();gyro=GyroLink();parser=Parser()
        def cycle(now,transition=False):
            info=self.ring(transition=transition)
            status=self.step(gate,now,info)
            info.update(entry_ready=status["ready"],entry_evidence=status["entry_evidence"],
                        timed_entry_admission=status["timed_admission"])
            return dict(gyro.update(parser,info,True,now,lambda a,b:a-b,ordinary_valid=True))
        self.assertEqual(cycle(100)["phase"],0)
        waiting=cycle(249)
        self.assertEqual(waiting["phase"],0)
        self.assertTrue(waiting["ordinary"])
        self.assertEqual(cycle(250)["phase"],1)
        for now in (270,290,310,330):self.assertEqual(cycle(now)["phase"],1)
        self.assertEqual(cycle(350,True)["phase"],1)
        self.assertEqual(cycle(370)["phase"],1)
        self.assertEqual(cycle(390)["phase"],1)
        self.assertEqual(cycle(410)["phase"],2)

if __name__ == "__main__":unittest.main()
