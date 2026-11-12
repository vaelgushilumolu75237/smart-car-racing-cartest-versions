from test_timed_entry import TimedEntryTests
from visual_entry_gate import OriginalTriggerGate
from gyro_link import GyroLink
import unittest

class OriginalTriggerTests(unittest.TestCase):
    def test_immediate_original_trigger_and_latched_invalid(self):
        for side in (0,1):
            gate=OriginalTriggerGate()
            info=TimedEntryTests().ring(side)
            def step(info,**kw):
                return dict(gate.update(info,profile_id=3,session_token=77,now_ms=100,**kw))
            candidate=dict(info,ring_confirmed=False,state_id=1,state="CANDIDATE")
            self.assertFalse(step(candidate)["ready"])
            self.assertTrue(step(info)["ready"])
            self.assertTrue(step({},valid=False)["ready"])
            self.assertFalse(step(info,session_active=False)["ready"])
    def test_no_camera_delay_and_no_early_orbit(self):
        class Parser:
            profile_id=3;profile_flags=5;session_token=77;profile_session_active=True
            def profile_is_fresh(self,*a):return True
            def control_is_fresh(self,*a):return True
        for side in (0,1):
            gyro=GyroLink();gate=OriginalTriggerGate()
            def cycle(now,transition):
                info=TimedEntryTests().ring(side,transition)
                status=gate.update(info,profile_id=3,session_token=77,now_ms=now)
                info.update(entry_ready=status["ready"],entry_evidence=status["entry_evidence"],timed_entry_admission=True,original_trigger_300=True)
                return gyro.update(Parser(),info,True,now,lambda a,b:a-b)["phase"]
            self.assertEqual(cycle(100,False),1)
            for now in (150,200,300,399):self.assertEqual(cycle(now,True),1)
            self.assertEqual(cycle(400,False),1)
            self.assertEqual(cycle(410,True),1)
            for now in (420,430):self.assertEqual(cycle(now,False),1)
            self.assertEqual(cycle(440,False),2)
