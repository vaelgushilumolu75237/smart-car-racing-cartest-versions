import ast,types,unittest
from pathlib import Path
from unittest.mock import Mock

class AutoWidthBootTests(unittest.TestCase):
 def setUp(self):
  source=(Path(__file__).resolve().parents[2]/'openmv/main.py').read_text(encoding='utf-8')
  tree=ast.parse(source)
  block=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and 'auto_width_requested = True' in ast.unparse(n) and 'ring_stationary_status_frames >= 3' in ast.unparse(n.test))
  self.code=compile(ast.Module(body=[block],type_ignores=[]),'production-auto-width','exec')
 def env(self):
  controller=types.SimpleNamespace(request=Mock())
  service=types.SimpleNamespace(camera_calibration_controller=controller,_hello_is_fresh=lambda now:True,_parked=lambda:True)
  return dict(config=types.SimpleNamespace(AUTO_BOOT_WIDTH_CALIBRATION=True),auto_width_requested=False,ring_stationary_status_frames=3,parked_profile=True,wireless_service=service,profile_now_ms=1000,print=lambda *args:None)
 def test_requests_once_after_fresh_stationary_confirmation(self):
  e=self.env();exec(self.code,e);exec(self.code,e)
  e['wireless_service'].camera_calibration_controller.request.assert_called_once()
  self.assertTrue(e['auto_width_requested'])
 def test_no_request_before_three_stationary_reports(self):
  e=self.env();e['ring_stationary_status_frames']=2;exec(self.code,e)
  e['wireless_service'].camera_calibration_controller.request.assert_not_called()
 def test_stale_hello_or_moving_profile_cannot_calibrate(self):
  for why in ('hello','moving','parked'):
   e=self.env()
   if why=='hello':e['wireless_service']._hello_is_fresh=lambda now:False
   elif why=='moving':e['parked_profile']=False
   else:e['wireless_service']._parked=lambda:False
   exec(self.code,e);e['wireless_service'].camera_calibration_controller.request.assert_not_called()
 def test_disabled_config_keeps_legacy_behavior(self):
  e=self.env();e['config'].AUTO_BOOT_WIDTH_CALIBRATION=False;exec(self.code,e)
  e['wireless_service'].camera_calibration_controller.request.assert_not_called()
