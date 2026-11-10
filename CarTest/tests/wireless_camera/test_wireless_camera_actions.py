"""Exercise wireless settings and real incremental camera calibration, without hardware."""
import sys
import struct
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openmv"))
sys.path.insert(0, str(Path(__file__).parent))
import main
import config
import ring_width_calibration as calibration
import track_elements
import wireless_link as wl
from test_wireless_camera import FakeClock, Sink, hello, command

class Actions(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.parked = True
        self.link = wl.WirelessCameraLink(enabled=True, clock=self.clock,
                                         parked_cb=lambda: self.parked)
        self.feed(hello())
        self.clock_patch = patch.object(main.time, "ticks_ms", self.clock.ticks_ms, create=True)
        self.diff_patch = patch.object(main.time, "ticks_diff", lambda a,b:a-b, create=True)
        self.clock_patch.start(); self.diff_patch.start()
        main.ring_width_calibration = calibration
    def tearDown(self):
        self.diff_patch.stop(); self.clock_patch.stop()
    def feed(self, data):
        self.link.feed_bytes(data, len(data), self.clock.value)
    def frames(self):
        sink=Sink()
        for _ in range(10): self.link.service_tx(sink, self.clock.value)
        return [wl.decode_frame(raw) for raw in sink.frames]
    def test_pd_changes_actual_output_independently(self):
        def result(p,d):
            pd=main._FastPositionPD(config)
            pd.update(100,20,True,900,"road","both","row",50,"pd",lambda a,b:a-b,p,d)
            return dict(pd.update(120,20,True,1000,"road","both","row",50,"pd",lambda a,b:a-b,p,d))
        base=result(1000,1000); lower=result(500,1000); low_d=result(1000,500)
        self.assertEqual(base["position_p"],1000)
        self.assertEqual(lower["position_p"],500)
        self.assertEqual(base["derivative_d"],lower["derivative_d"])
        self.assertLess(low_d["derivative_d"],base["derivative_d"])
        self.assertNotEqual(base["pd_output"],lower["pd_output"])
    def test_get_config_and_ram_defaults(self):
        store=types.SimpleNamespace(defaults=lambda: self.fail("DEFAULT wrote persistent storage"))
        self.link.settings_store=store
        self.link.vision_p_permille=700; self.link.vision_d_permille=1200
        self.feed(command(wl.OP_GET_PARAMS,2,self.link.session,1000))
        frames=self.frames(); payload=next(f["payload"] for f in frames if f["type"]==wl.TYPE_CONFIG)
        self.assertEqual(len(payload),16)
        self.assertEqual(struct.unpack_from("<HH",payload,8),(700,1200))
        self.feed(command(wl.OP_DEFAULT,3,self.link.session,1000))
        self.assertEqual((self.link.vision_p_permille,self.link.vision_d_permille),(1000,1000))
    def controller(self,path):
        tracker=types.SimpleNamespace(require_calibration=True)
        tracker.set_ring_calibration=lambda value,required:setattr(tracker,"calibration",value)
        controller=main._WirelessCameraCalibrationController(str(path),tracker,self.link)
        self.link.camera_cal=controller.request
        return controller,tracker
    def test_real_collection_saves_and_finishes_same_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"calibration.json"
            controller,tracker=self.controller(path)
            self.feed(command(wl.OP_CAMERA_CAL,2,self.link.session,1000))
            self.assertTrue(controller.active)
            self.assertEqual(self.link.last_ack[0],wl.ACK_PENDING)
            rows=[]
            for y in range(8,60): rows.extend((y,24,53,7))
            for _ in range(calibration.MAX_ATTEMPTS):
                if controller.step(1000,rows,len(rows)//4,4,True): break
                self.frames()
            self.assertFalse(controller.active)
            self.assertTrue(controller.last_success)
            self.assertTrue(path.exists())
            self.assertTrue(tracker.calibration.ready)
            self.assertIsNone(self.link.pending_camera_cal_seq)
            self.assertEqual(self.link.last_ack[0],wl.ACK_OK)
            self.assertEqual(struct.unpack_from("<I",self.link.last_ack,4)[0],2)
    def test_calibration_loses_parked_or_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            controller,_=self.controller(Path(tmp)/"calibration.json")
            self.feed(command(wl.OP_CAMERA_CAL,2,self.link.session,1000))
            self.parked=False
            controller.step(1000,None,0,4,False)
            self.assertFalse(controller.active)
            self.assertEqual(self.link.last_ack[0],wl.ACK_NOT_PARKED)
            self.parked=True
            self.feed(command(wl.OP_CAMERA_CAL,3,self.link.session,1000))
            self.feed(hello(epoch=0x5678,sequence=4))
            controller.step(1000,None,0,4,False)
            self.assertFalse(controller.active)
            self.assertIsNone(self.link.pending_camera_cal_seq)

    def test_collected_map_requires_real_parked_check_before_release(self):
        with tempfile.TemporaryDirectory() as tmp:
            tracker = track_elements.TrackElementsTracker(
                config, require_calibration=True)
            controller = main._WirelessCameraCalibrationController(
                str(Path(tmp)/"calibration.json"), tracker, self.link)
            controller.request()
            rows = [value for y in range(8,60) for value in (y,24,53,7)]
            for _ in range(calibration.MAX_ATTEMPTS):
                if controller.step(1000,rows,len(rows)//4,4,True): break
            self.assertTrue(controller.last_success)
            self.assertTrue(tracker.ring_calibration_ready)
            self.assertFalse(tracker.ring_calibration_check_ready)
            was = False
            for expected in (False, False, True):
                ready, was = main._advance_parked_width_check(
                    tracker,controller,True,rows,len(rows)//4,4,was)
                self.assertEqual(ready,expected)
            self.assertTrue(tracker.ring_calibration_check_ready)

            # A new collection must not reuse the old successful startup check.
            controller.request()
            ready, was = main._advance_parked_width_check(
                tracker,controller,True,rows,len(rows)//4,4,was)
            self.assertFalse(ready)
            for _ in range(calibration.MAX_ATTEMPTS):
                if controller.step(1000,rows,len(rows)//4,4,True): break
            self.assertTrue(controller.last_success)
            self.assertFalse(tracker.ring_calibration_check_ready)
            for _ in range(2):
                ready, was = main._advance_parked_width_check(
                    tracker,controller,True,rows,len(rows)//4,4,was)
                self.assertFalse(ready)
            ready, was = main._advance_parked_width_check(
                tracker,controller,False,None,0,4,was)
            self.assertFalse(ready)
            # Losing stationary/route eligibility restarts the three-frame check.
            for expected in (False, False, True):
                ready, was = main._advance_parked_width_check(
                    tracker,controller,True,rows,len(rows)//4,4,was)
                self.assertEqual(ready,expected)

if __name__=="__main__": unittest.main()
