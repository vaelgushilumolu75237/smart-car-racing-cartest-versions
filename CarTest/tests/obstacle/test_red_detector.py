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
    def rect(self): return (10,20,30,40)
class Frame:
    def __init__(self, counts, width=320): self.counts, self.w = counts, width
    def width(self): return self.w
    def height(self): return 120
    def find_blobs(self, thresholds, **kw):
        assert kw == dict(x_stride=1, y_stride=1, pixels_threshold=1, area_threshold=1, merge=False)
        assert thresholds == [(0,100,25,127,-20,127)]
        return [Blob(x) for x in self.counts]
class UART:
    def __init__(self): self.writes = []; self.result = 1
    def write(self, b): self.writes.append(b); return self.result
class Tests(unittest.TestCase):
    def test_all_components_and_holes(self):
        self.assertEqual(m.count_red_pixels(Frame([4608,1]), (0,100,25,127,-20,127)),4609)
        self.assertEqual(m.count_red_pixels(Frame([1]*4609), (0,100,25,127,-20,127)),4609)
        self.assertEqual(m.count_red_pixels(Frame([]), (0,100,25,127,-20,127)),0)
    def test_geometry(self):
        old = Frame([]); old.height = lambda: 240
        with self.assertRaises(ValueError): m.count_red_pixels(old, ())
        with self.assertRaises(ValueError): m.count_red_pixels(Frame([],640), ())
    def test_strict_threshold_once_rearm(self):
        event, uart = m.RedEvent(), UART()
        for count, result in [(0,False),(4608,False),(4609,True),(38400,False),(4609,False),(4608,False),(4609,True)]:
            self.assertEqual(event.update(count,uart),result)
        self.assertEqual(uart.writes,[b"R",b"R"])
    def test_failed_write_not_latched(self):
        event, uart = m.RedEvent(), UART(); uart.result=0
        self.assertFalse(event.update(4609,uart)); uart.result=1
        self.assertTrue(event.update(4609,uart))

class BoxTests(unittest.TestCase):
    def test_threshold_color_and_coordinates(self):
        class Preview:
            def draw_rectangle(self,*a,**kw):self.args=a;self.kw=kw
        p=Preview()
        m.draw_red_boxes(p,[Blob(4608)],4608,2,0,0)
        self.assertEqual(p.args,(20,40,60,80));self.assertEqual(p.kw['color'],(0,0,255))
        m.draw_red_boxes(p,[Blob(4609)],4609,2,0,0)
        self.assertEqual(p.kw['color'],(0,255,0))

class StartupTests(unittest.TestCase):
    def exercise(self, legacy_tx=False, fail_display=False):
        calls=[]
        class StopTest(Exception): pass
        class FPIOA:
            def set_function(self,*a,**kw): calls.append(("pin",a,kw))
        setattr(FPIOA,"UART1_TX" if legacy_tx else "UART1_TXD",123)
        setattr(FPIOA,"UART1_RX" if legacy_tx else "UART1_RXD",124)
        class FakeUART(UART):
            EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=0
            def __init__(self,*a,**kw):
                super().__init__()
                assert ("pin",(9,123),dict(ie=0,oe=1)) in calls
                assert ("pin",(10,124),dict(ie=1,oe=0)) in calls
                calls.append(("uart",a,kw))
            def write(self,b): calls.append(("write",b));return 1
            def deinit(self): calls.append("uart-close")
        class Sensor:
            RGB565=17
            def __init__(self):self.snapshots=0
            def reset(self): calls.append("reset")
            def set_framesize(self,**kw): calls.append(("size",kw))
            def set_pixformat(self,p): calls.append(("format",p))
            def run(self): calls.append("run")
            def snapshot(self):
                self.snapshots+=1
                if self.snapshots>1:raise StopTest()
                return Frame([4609])
            def stop(self): calls.append("stop")
        class Preview:
            def __init__(self,*args): calls.append(("preview",args))
            def clear(self): calls.append("clear")
            def draw_rectangle(self,*a,**kw): calls.append(("box",a,kw))
            def draw_image(self,frame,x,y,**kw):
                assert frame.width()==320 and frame.height()==120
                calls.append(("draw",x,y,kw))
        def display_init(mode,**kw):
            calls.append(("display-init",mode,kw))
            if fail_display:raise StopTest()
        def show_image(frame,**kw):calls.append(("show",kw))
        display=types.SimpleNamespace(ST7701=99,init=display_init,show_image=show_image,
                                      deinit=lambda:calls.append("display-close"))
        media=types.SimpleNamespace(init=lambda:calls.append("media-init"),deinit=lambda:calls.append("media-close"))
        modules={"machine":types.SimpleNamespace(UART=FakeUART,FPIOA=FPIOA),
                 "media.sensor":types.SimpleNamespace(Sensor=Sensor),
                 "media.media":types.SimpleNamespace(MediaManager=media),
                 "media.display":types.SimpleNamespace(Display=display),
                 "image":types.SimpleNamespace(Image=Preview,RGB565=17),"red_detector":m}
        settings_spec=importlib.util.spec_from_file_location("settings",ROOT/"k230_red_obstacle/settings.py")
        settings=importlib.util.module_from_spec(settings_spec);settings_spec.loader.exec_module(settings)
        modules["settings"]=settings
        with patch.dict(sys.modules,modules):
            entry_spec=importlib.util.spec_from_file_location("k230_entry",ROOT/"k230_red_obstacle/main.py")
            entry=importlib.util.module_from_spec(entry_spec);entry_spec.loader.exec_module(entry)
            with self.assertRaises(StopTest):entry.run()
        return calls

    def test_lcd_and_uart_factory_firmware(self):
        calls=self.exercise()
        self.assertIn(("size",dict(width=320,height=120)),calls)
        self.assertIn(("pin",(9,123),dict(ie=0,oe=1)),calls)
        self.assertIn(("uart",(1,),dict(baudrate=115200,bits=8,parity=None,stop=0)),calls)
        init=("display-init",99,dict(width=640,height=480,to_ide=True,fps=30))
        self.assertIn(init,calls)
        self.assertLess(calls.index(init),calls.index("media-init"))
        self.assertIn(("preview",(640,480,17)),calls)
        draw=("draw",0,120,dict(x_scale=2.0,y_scale=2.0))
        self.assertIn(draw,calls)
        self.assertLess(calls.index(("write",b"R")),calls.index(draw))
        self.assertIn(("show",dict(x=0,y=0)),calls)
        self.assertEqual(calls[-4:],["stop","display-close","media-close","uart-close"])

    def test_legacy_tx_name_supported(self):
        self.assertIn(("pin",(9,123),dict(ie=0,oe=1)),self.exercise(legacy_tx=True))

    def test_partial_display_initialization_cleanup(self):
        calls=self.exercise(fail_display=True)
        self.assertNotIn("media-init",calls)
        self.assertEqual(calls[-3:],["stop","display-close","uart-close"])

if __name__ == "__main__": unittest.main()
