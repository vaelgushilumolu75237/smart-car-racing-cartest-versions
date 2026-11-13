import importlib.util
from pathlib import Path
import unittest
import sys
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("red_detector", ROOT / "k230_red_obstacle/red_detector.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Blob:
    def __init__(self, pixels): self.n = pixels
    def pixels(self): return self.n
class Frame:
    def __init__(self, counts, width=320): self.counts, self.w = counts, width
    def width(self): return self.w
    def height(self): return 240
    def find_blobs(self, thresholds, **kw):
        assert kw == dict(x_stride=1, y_stride=1, pixels_threshold=1, area_threshold=1, merge=False)
        assert thresholds == [(0,100,25,127,-20,127)]
        return [Blob(x) for x in self.counts]
class UART:
    def __init__(self): self.writes = []; self.result = 1
    def write(self, b): self.writes.append(b); return self.result
class Tests(unittest.TestCase):
    def test_all_components_and_holes(self):
        self.assertEqual(m.count_red_pixels(Frame([11520,1]), (0,100,25,127,-20,127)),11521)
        self.assertEqual(m.count_red_pixels(Frame([1]*11521), (0,100,25,127,-20,127)),11521)
        self.assertEqual(m.count_red_pixels(Frame([]), (0,100,25,127,-20,127)),0)
    def test_geometry(self):
        with self.assertRaises(ValueError): m.count_red_pixels(Frame([],640), ())
    def test_strict_threshold_once_rearm(self):
        event, uart = m.RedEvent(), UART()
        for count, result in [(0,False),(11520,False),(11521,True),(76800,False),(11521,False),(11520,False),(11521,True)]:
            self.assertEqual(event.update(count,uart),result)
        self.assertEqual(uart.writes,[b"R",b"R"])
    def test_failed_write_not_latched(self):
        event, uart = m.RedEvent(), UART(); uart.result=0
        self.assertFalse(event.update(11521,uart)); uart.result=1
        self.assertTrue(event.update(11521,uart))

class StartupTests(unittest.TestCase):
    def test_sensor_uart_and_cleanup(self):
        calls=[]
        class StopTest(Exception): pass
        class FPIOA:
            UART1_TX=123
            def set_function(self,*a,**kw): calls.append(("pin",a,kw))
        class FakeUART(UART):
            EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=0
            def __init__(self,*a,**kw):
                super().__init__();calls.append(("uart",a,kw))
            def deinit(self): calls.append("uart-close")
        class Sensor:
            RGB565=17
            def reset(self): calls.append("reset")
            def set_framesize(self,**kw): calls.append(("size",kw))
            def set_pixformat(self,p): calls.append(("format",p))
            def run(self): calls.append("run")
            def snapshot(self): raise StopTest()
            def stop(self): calls.append("stop")
        media=types.SimpleNamespace(init=lambda:calls.append("media-init"),deinit=lambda:calls.append("media-close"))
        modules={"machine":types.SimpleNamespace(UART=FakeUART,FPIOA=FPIOA),
                 "media.sensor":types.SimpleNamespace(Sensor=Sensor),
                 "media.media":types.SimpleNamespace(MediaManager=media),"red_detector":m}
        settings_spec=importlib.util.spec_from_file_location("settings",ROOT/"k230_red_obstacle/settings.py")
        settings=importlib.util.module_from_spec(settings_spec);settings_spec.loader.exec_module(settings)
        modules["settings"]=settings
        with patch.dict(sys.modules,modules):
            entry_spec=importlib.util.spec_from_file_location("k230_entry",ROOT/"k230_red_obstacle/main.py")
            entry=importlib.util.module_from_spec(entry_spec);entry_spec.loader.exec_module(entry)
            with self.assertRaises(StopTest):entry.run()
        self.assertIn(("size",dict(width=320,height=240)),calls)
        self.assertIn(("pin",(9,123),dict(ie=0,oe=1)),calls)
        self.assertIn(("uart",(1,),dict(baudrate=115200,bits=8,parity=None,stop=0)),calls)
        self.assertEqual(calls[-3:],["stop","media-close","uart-close"])

if __name__ == "__main__": unittest.main()
