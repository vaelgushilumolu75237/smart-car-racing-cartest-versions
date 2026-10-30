#!/usr/bin/env python3
"""Build a private RAM-only 320x120 OpenMV full-pipeline perf candidate.

This only packages Python source and a hash manifest. It does not connect to,
execute on, reset, or upload to a camera. The generated board program uses the
production 23-module runtime, the real sensor, production native JPEG preview,
and a virtual RESET/KEEP + profile-C input sink. The sink is not an H743 or a
physical UART, and its zero wheel count is not vehicle evidence.

Important known limit: production requires a parked profile-C calibration
check before it acknowledges active profile C. The generated virtual sink
first sends parked profile C with token 0, waits for a real production V3
ACK=3/valid frame, holds three more 100 ms parked statuses, and only then sends
active profile C/token 1. If this does not happen within 30 seconds it stops
with FAIL_GATE. The camera must be stationary on its calibrated 45 cm straight
fixture. Even a passed gate is not a road-route or vehicle-acceptance result.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
OPENMV_ROOT = PROJECT_ROOT / "openmv"
DEFAULT_OUTPUT_DIR = (
    PROJECT_ROOT / "logs" / "diagnostics" / "ring-joint-v25-fps")
PROFILE_GATE_TIMEOUT_S = 30
ACTIVE_WINDOW_ALIGNMENT_MARGIN_S = 20

try:
    from .build_track_profiles import RUNTIME_NAMES
except ImportError:
    from build_track_profiles import RUNTIME_NAMES

if len(RUNTIME_NAMES) != 23:
    raise RuntimeError("build_track_profiles must declare exactly 23 runtime modules")


RAM_VFS_PREFIX = '''import sys,time
try:
 import gc,deflate,io,vfs,micropython
 _ram_sources={}
 class _RAMOnlyFS:
  def mount(self,*args):pass
  def umount(self):pass
  def stat(self,path):
   if path in ('','/'):return (0x4000,0,0,0,0,0,0,0,0,0)
   if path.lstrip('/') in _ram_sources:return (0x8000,0,0,0,0,0,1,0,0,0)
   raise OSError(2)
  def open(self,path,mode):
   name=path.lstrip('/')
   if mode not in ('r','rb'):raise OSError("RAM module VFS is read-only")
   if name not in _ram_sources:raise OSError(2)
   raw=deflate.DeflateIO(io.BytesIO(_ram_sources[name]),deflate.ZLIB).read()
   return io.BytesIO(raw)
 vfs.mount(_RAMOnlyFS(),'/ramprobe')
 sys.path.insert(0,'/ramprobe')
'''


PROBE_SUFFIX = r'''
 _RUNTIME_NAMES=RUNTIME_NAMES_PLACEHOLDER
 _MODE=MODE_PLACEHOLDER
 _ACTIVE_SECONDS=ACTIVE_SECONDS_PLACEHOLDER
 _GATE_TIMEOUT_MS=30000
 _WINDOW_MARGIN_MS=20000
 _RUN_SECONDS=RUN_SECONDS_PLACEHOLDER
 _RUN_ID=20261029
 _PREVIEW_MARKER_STATS=[0,0,0]
 _RAM_DIAG_BLOB=[None]
 _RAM_DIAG_SAVE_COUNT=[0]
 _PERF_EPOCH_MS=[None]
 _ACTIVE_WINDOWS=[]
 _RUN_START_MS=None
 # Production deliberately catches capture Exceptions and retries. A probe
 # deadline must escape that recovery loop and reach the outer report handler.
 class _GateTimeout(BaseException):pass
 # Clear all cached production modules before importing any one of them.
 # Some modules import config/reverse_control transitively; interleaving
 # deletion and import can otherwise split their module identities.
 for _name in _RUNTIME_NAMES:
  _module_name=_name[:-3]
  if _module_name in sys.modules:del sys.modules[_module_name]
 for _name in _RUNTIME_NAMES:
  __import__(_name[:-3])
  print('RAM_LOADED',_name)
  gc.collect()
 import main,sensor,csi,config,reverse_control,ring_width_calibration
 import ring_diagnostics,track_elements
 main._load_fast_runtime()
 main._load_singleedge_runtime()
 main._load_track_elements_runtime()
 main._load_ring_runtime()
 if main.config is not config:
  raise RuntimeError('main.config identity differs from the configured RAM module')
 if main.reverse_control is not reverse_control:
  raise RuntimeError('main.reverse_control identity differs from the virtual sink module')
 if main.track_elements is not track_elements or main.ring_diagnostics is not ring_diagnostics:
  raise RuntimeError('main route/RNGD modules differ from the RAM allowlist instances')
 if len(_RUNTIME_NAMES)!=23:
  raise RuntimeError('exactly 23 production modules are required')
 if getattr(track_elements,'JOINT_ROUTE_API_VERSION',None)!=1:
  raise RuntimeError('track_elements JOINT_ROUTE_API_VERSION=1 required')
 if getattr(ring_diagnostics,'FORMAT_VERSION',None)!=3:
  raise RuntimeError('ring_diagnostics FORMAT_VERSION=3 required')
 if getattr(sensor,'PREVIEW_CONTROL_API',None)!=1:
  raise RuntimeError('sensor PREVIEW_CONTROL_API=1 required')
 if getattr(csi,'PREVIEW_CONTROL_API',None)!=1 or not main._native_preview_control_api_available():
  raise RuntimeError('native preview API mismatch; legacy fallback is forbidden')
 if not callable(getattr(main,'_NativeUSBPreviewSource',None)) or not callable(getattr(main,'_is_native_usb_preview_source',None)):
  raise RuntimeError('production native USB preview source API unavailable')
 config.USB_PREVIEW_ENABLED=(_MODE=='jpeg')
 config.PREVIEW_FPS_MARKER_ENABLED=(_MODE=='jpeg')
 config.RING_DIAGNOSTICS_PATH='/ramprobe/ring_diagnostics.bin'
 # Keep production JPEG quality, dimensions, overlay policy, thresholds and
 # detector configuration unchanged. The only runtime switches are preview
 # on/off, the after-processing frame marker, finite duration, and RAM diag IO.
 if config.IMAGE_WIDTH!=320 or config.IMAGE_HEIGHT!=120 or config.FRAME_SIZE_NAME!='QVGA':
  raise RuntimeError('production 320x120 QVGA crop is required')
 if config.RING_WIDTH_CALIBRATION_REQUIRED is not True:
  raise RuntimeError('production ring calibration gate must remain enabled')
 main._validate_fast_config()
 _sensor_id=main._configure_gray_sensor(sensor,fast=True)
 if _sensor_id!=ring_width_calibration.SENSOR_ID:
  raise RuntimeError('real sensor ID does not match private calibration')
 # Load the existing configured calibration path read-only and reject a missing
 # or invalid map. Production _main_fast loads the same path again. Never call
 # RingWidthCalibration.save() or the manual calibration routine.
 _calibration=ring_width_calibration.RingWidthCalibration.load(
     config.RING_WIDTH_CALIBRATION_PATH)
 if not _calibration.ready:
  raise RuntimeError('existing ring-width calibration is not ready: '+str(_calibration.reason))
 _calibration_probe=track_elements.TrackElementsTracker(
     config,_calibration,require_calibration=True)
 if not _calibration_probe.ring_calibration_ready:
  raise RuntimeError('loaded calibration fails production tracker validation')
 print('CALIBRATION_READ_ONLY path=%s map_ready=1 startup_check_ready=%d' % (
     config.RING_WIDTH_CALIBRATION_PATH,
     1 if _calibration_probe.ring_calibration_check_ready else 0))
 print('PROFILE_GATE_PREFLIGHT profile=C parked_token=0 then active_token=1 '
       'zero_wheel_millirev=1 timeout_s=30 '
       'requires_stationary_calibrated_45cm_straight=1')
 if config.USB_PREVIEW_ENABLED:
  _usb_vcp=main._open_usb_vcp_safely()
  if not main._usb_vcp_is_connected(_usb_vcp):
   raise RuntimeError('native USB VCP is not connected; no preview fallback')
 else:
  _usb_vcp=main._open_usb_vcp_safely()
 _native_source=main._NativeUSBPreviewSource(
     sensor,_usb_vcp,config.USB_PREVIEW_ENABLED,config.USB_PREVIEW_QUALITY)
 class _ProbeSource:
  USB_PREVIEW_SOURCE_API=1
  def __init__(self,native):
   self.native=native
   self._usb_vcp=native._usb_vcp
   self.latest_image=None
   self.frame_preview_active=False
   self.last_state='NATIVE_HEADLESS'
   self.publish_failures=0
   self.capture_count=0
   self.preview_publish_count=0
   self.active_capture_count=0
   self.gate=None
  def configure(self,enabled,quality):
   self.native.configure(enabled,quality)
  def snapshot(self):
   if (self.gate is not None and self.gate.started_ms is not None and
       self.gate.active_ack_start_ms is None):
    if time.ticks_diff(time.ticks_ms(),self.gate.started_ms)>=_GATE_TIMEOUT_MS:
     self.gate.gate_failed=True
     raise _GateTimeout('FAIL_GATE: parked calibration V3 ACK3 valid1 not observed within 30s')
   image=self.native.snapshot()
   self.latest_image=image
   self.frame_preview_active=self.native.frame_preview_active
   self.last_state=self.native.last_state
   self.publish_failures=self.native.publish_failures
   self.capture_count+=1
   if self.frame_preview_active:self.active_capture_count+=1
   return image
  def flush(self):
   self.native.flush()
   self.last_state=self.native.last_state
   self.publish_failures=self.native.publish_failures
   if self.last_state=='NATIVE_ACTIVE':self.preview_publish_count+=1
   self.latest_image=None
   self.frame_preview_active=False
  def clear(self):
   self.native.clear()
   self.latest_image=None
   self.frame_preview_active=False
   self.last_state=self.native.last_state
   self.publish_failures=self.native.publish_failures
 _source=_ProbeSource(_native_source)
 if not main._is_native_usb_preview_source(_source):
  raise RuntimeError('source proxy does not satisfy exact production native API')
 class _VirtualUARTSink:
  def __init__(self):
   self.buffer=bytearray(64)
   self.pending=0
   self.offset=0
   self.reset_pending=True
   self.control_sequence=0
   self.profile_sequence=0
   self.started_ms=None
   self.next_keep_ms=None
   self.next_profile_ms=None
   self.reset_frames=0
   self.keep_frames=0
   self.profile_c_parked_frames=0
   self.profile_c_parked_after_ack=0
   self.profile_c_active_frames=0
   self.parked_ack_valid_seen=False
   self.active_profile_sent=False
   self.active_ack_start_ms=None
   self.gate_failed=False
   self.v3_full_writes=0
   self.diagnostic_writes=0
   self.write_errors=0
  def _append(self,frame):
   if self.pending+len(frame)>len(self.buffer):
    raise OSError('virtual reverse-control queue overflow')
   for index in range(len(frame)):
    self.buffer[self.pending+index]=frame[index]
   self.pending+=len(frame)
  def _schedule(self):
   now=time.ticks_ms()
   if self.started_ms is None:
    self.started_ms=now
    self.next_keep_ms=time.ticks_add(now,50)
    self.next_profile_ms=now
   if self.reset_pending:
    self._append(reverse_control.pack_frame(
        reverse_control.COMMAND_RESET,self.control_sequence))
    self.reset_pending=False
    self.control_sequence=(self.control_sequence+1)&0xffff
    self.next_keep_ms=time.ticks_add(now,50)
    self.reset_frames+=1
   elif time.ticks_diff(now,self.next_keep_ms)>=0:
    self._append(reverse_control.pack_frame(
        reverse_control.COMMAND_KEEP,self.control_sequence))
    self.control_sequence=(self.control_sequence+1)&0xffff
    self.next_keep_ms=time.ticks_add(now,50)
    self.keep_frames+=1
   if time.ticks_diff(now,self.next_profile_ms)>=0:
    make_active=(self.parked_ack_valid_seen and
                 self.profile_c_parked_after_ack>=3)
    flags=reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED
    token=0
    if make_active:
     flags=(reverse_control.PROFILE_SESSION_ACTIVE|
            reverse_control.PROFILE_DIAGNOSTICS_SUPPORTED)
     token=1
     self.active_profile_sent=True
    self._append(reverse_control.pack_profile_state(
        self.profile_sequence,reverse_control.PROFILE_C,
        flags=flags,revision=reverse_control.PROFILE_REVISION,
        session_token=token,mean_forward_wheel_millirev=0))
    self.profile_sequence=(self.profile_sequence+1)&0xffff
    self.next_profile_ms=time.ticks_add(now,100)
    if make_active:
     self.profile_c_active_frames+=1
    else:
     self.profile_c_parked_frames+=1
     if self.parked_ack_valid_seen:
      self.profile_c_parked_after_ack+=1
  def any(self):
   if self.pending==0:
    self.offset=0
    self._schedule()
   return self.pending
  def readinto(self,destination,requested=None):
   if requested is None:requested=len(destination)
   count=min(len(destination),requested,self.pending)
   for index in range(count):
    destination[index]=self.buffer[self.offset+index]
   self.offset+=count
   self.pending-=count
   if self.pending==0:self.offset=0
   return count
  def write(self,packet):
   if (len(packet)!=16 or packet[0]!=0xa5 or packet[1]!=0x5a or
       packet[2] not in (3,0xd1) or packet[3]!=8):
    self.write_errors+=1
    raise ValueError('virtual sink requires a complete V3 or D1 frame')
   crc=main.vision_protocol.crc16_ccitt_false(memoryview(packet)[2:14])
   if crc!=(packet[14]|(packet[15]<<8)):
    self.write_errors+=1
    raise ValueError('virtual sink CRC mismatch')
   if packet[2]==0xd1:
    self.diagnostic_writes+=1
    return 16
   self.v3_full_writes+=1
   if packet[11]==reverse_control.PROFILE_C and (packet[6]&1):
    if self.active_profile_sent:
     if self.active_ack_start_ms is None:
      self.active_ack_start_ms=time.ticks_ms()
    else:
     self.parked_ack_valid_seen=True
   return 16
 _sink=_VirtualUARTSink()
 _source.gate=_sink
 # Preserve every production per-frame RNGD record, but route any automatic
 # post-stop persistence into one bounded RAM slot. The calibration file path
 # remains the configured production read path and is never opened for write.
 def _ram_only_diag_save(self,path,build_id):
  if not self.frozen:raise RuntimeError('RNGD must be frozen before RAM save')
  if self.persisted:return True
  if self.episode_count<=0:raise RuntimeError('RNGD has no stopped frames')
  _RAM_DIAG_BLOB[0]=self._payload(build_id)
  _RAM_DIAG_SAVE_COUNT[0]+=1
  self._persisted=True
  self._last_saved_path='ram://ring_diagnostics.bin'
  return True
 def _ram_only_diag_save_next(self,base_path,build_id):
  self.save('ram://ring_diagnostics.bin',build_id)
  return 'ram://ring_diagnostics.bin'
 ring_diagnostics.RingDiagnostics.save=_ram_only_diag_save
 ring_diagnostics.RingDiagnostics.save_next=_ram_only_diag_save_next
 _original_marker=main._draw_preview_frame_marker
 def _timed_marker(image,sequence):
  started=time.ticks_us()
  _PREVIEW_MARKER_STATS[0]+=1
  try:
   ok=_original_marker(image,sequence)
   if ok:_PREVIEW_MARKER_STATS[1]+=1
   else:_PREVIEW_MARKER_STATS[2]+=1
   return ok
  finally:
   _MARKER_ELAPSED=time.ticks_diff(time.ticks_us(),started)
   _PREVIEW_MARKER_TOTAL_US[0]+=_MARKER_ELAPSED
 _PREVIEW_MARKER_TOTAL_US=[0]
 _original_perf_factory=main._V7PerfWindow
 _original_summary=main._print_v7_summary
 def _timestamped_perf_factory():
  _PERF_EPOCH_MS[0]=time.ticks_ms()
  return _original_perf_factory()
 def _capture_perf_summary(perf,run_id,window_index,attempted,queue=None):
  _ACTIVE_WINDOWS.append({
      'run':run_id,'index':window_index,
      'start_ms':window_index*config.PERF_WINDOW_MS,
      'end_ms':(window_index+1)*config.PERF_WINDOW_MS,
      'distinct_captures':perf.frames,
      'write_attempts':perf.write_attempts,
      'full_uart_writes':perf.full_writes,
      'valid_frames':perf.valid_frames,
      'uart_write_errors':perf.write_errors})
  return _original_summary(perf,run_id,window_index,attempted,queue)
 main._V7PerfWindow=_timestamped_perf_factory
 main._print_v7_summary=_capture_perf_summary
 main._draw_preview_frame_marker=_timed_marker
 _RUN_START_MS=time.ticks_ms()
 print('RAM_FULL_PIPELINE_PROBE mode=%s active_measure_s=%d '
       'max_total_run_s=%d capture=real_sensor '
       'crop=320x120 preview=production_native_jpeg quality=Q%d '
       'rate=unverified uart=virtual_sink v3_write=16 '
       'wheel_millirev=0 physical_uart=0 flash_write=0 '
       'actual_ide_acceptance=not_proven' % (
           _MODE,_ACTIVE_SECONDS,_RUN_SECONDS,config.USB_PREVIEW_QUALITY))
 try:
  main._main_fast(duration_s=_RUN_SECONDS,run_id=_RUN_ID,
                  frame_source=_source,uart_sink=_sink,
                  fixture_sensor_id=_sensor_id)
 finally:
  main._draw_preview_frame_marker=_original_marker
  main._print_v7_summary=_original_summary
  main._V7PerfWindow=_original_perf_factory
 _ACTIVE_END_MS=time.ticks_ms()
 _ACTIVE_START_REL_MS=(None if _sink.active_ack_start_ms is None else
     time.ticks_diff(_sink.active_ack_start_ms,_PERF_EPOCH_MS[0]))
 _ACTIVE_END_REL_MS=(None if _PERF_EPOCH_MS[0] is None else
     time.ticks_diff(_ACTIVE_END_MS,_PERF_EPOCH_MS[0]))
 _ACTIVE_ELAPSED_MS=(None if _sink.active_ack_start_ms is None else
     time.ticks_diff(_ACTIVE_END_MS,_sink.active_ack_start_ms))
 _FULL_ACTIVE_WINDOWS=[]
 if _ACTIVE_START_REL_MS is not None and _ACTIVE_END_REL_MS is not None:
  for _window in _ACTIVE_WINDOWS:
   if (_window['start_ms']>=_ACTIVE_START_REL_MS+100 and
       _window['end_ms']<=_ACTIVE_END_REL_MS-100):
    _FULL_ACTIVE_WINDOWS.append(_window)
 for _window in _FULL_ACTIVE_WINDOWS:
  print('V7_ACTIVE_PERF_WINDOW run=%d index=%d start_ms=%d end_ms=%d '
        'distinct_captures=%d write_attempts=%d full_uart_writes=%d '
        'valid_frames=%d uart_write_errors=%d source=production_window '
        'gate=active_ack3_valid1' % (
            _window['run'],_window['index'],_window['start_ms'],
            _window['end_ms'],_window['distinct_captures'],
            _window['write_attempts'],_window['full_uart_writes'],
            _window['valid_frames'],_window['uart_write_errors']))
 _ACTIVE_WINDOW_REQUIREMENT=(_ACTIVE_SECONDS//10)
 _ACTIVE_MEASUREMENT_OK=bool(
     _ACTIVE_ELAPSED_MS is not None and
     _ACTIVE_ELAPSED_MS>=_ACTIVE_SECONDS*1000 and
     len(_FULL_ACTIVE_WINDOWS)>=_ACTIVE_WINDOW_REQUIREMENT)
 print('RAM_FULL_PIPELINE_RESULT mode=%s captures=%d '
       'preview_usb_published=%d preview_active_captures=%d '
       'native_publish_failures=%d marker_calls=%d marker_writes=%d '
       'marker_failures=%d marker_total_us=%d '
       'v3_full_writes=%d virtual_write_errors=%d reset_frames=%d '
       'keep_frames=%d profile_c_parked_frames=%d '
       'profile_c_parked_after_ack=%d profile_c_active_frames=%d '
       'profile_c_active_ack_seen=%d active_start_ms=%s active_elapsed_ms=%s '
       'active_full_windows=%d '
       'rngd_ram_saves=%d rngd_last_blob_bytes=%d '
       'ring_route_behavior_proven=0 actual_ide_acceptance=not_proven' % (
           _MODE,_source.capture_count,_source.preview_publish_count,
           _source.active_capture_count,_source.publish_failures,
           _PREVIEW_MARKER_STATS[0],_PREVIEW_MARKER_STATS[1],
           _PREVIEW_MARKER_STATS[2],_PREVIEW_MARKER_TOTAL_US[0],
           _sink.v3_full_writes,_sink.write_errors,_sink.reset_frames,
           _sink.keep_frames,_sink.profile_c_parked_frames,
           _sink.profile_c_parked_after_ack,_sink.profile_c_active_frames,
           1 if _sink.active_ack_start_ms is not None else 0,
           str(_ACTIVE_START_REL_MS),str(_ACTIVE_ELAPSED_MS),
           len(_FULL_ACTIVE_WINDOWS),
           _RAM_DIAG_SAVE_COUNT[0],
           0 if _RAM_DIAG_BLOB[0] is None else len(_RAM_DIAG_BLOB[0])))
 if _sink.active_ack_start_ms is None:
  print('RAM_FULL_PIPELINE_STATUS=FAIL_GATE reason=active_profile_ack3_valid1_missing')
 elif not _ACTIVE_MEASUREMENT_OK:
  print('RAM_FULL_PIPELINE_STATUS=INCOMPLETE reason=insufficient_active_measurement')
 elif _MODE=='jpeg' and _source.preview_publish_count!=_source.capture_count:
  print('RAM_FULL_PIPELINE_STATUS=INCOMPLETE reason=not_every_capture_published')
 else:
  print('RAM_FULL_PIPELINE_STATUS=CANDIDATE_ONLY '
        'reason=virtual_uart_and_actual_ide_not_proven')
except _GateTimeout as _exc:
 print('RAM_FULL_PIPELINE_STATUS=FAIL_GATE reason=parked_calibration_ack_timeout')
 try:sys.print_exception(_exc)
 except Exception:print('PROBE_ERROR',type(_exc).__name__,str(_exc))
except Exception as _exc:
 try:sys.print_exception(_exc)
 except Exception:print('PROBE_ERROR',type(_exc).__name__,str(_exc))
 print('RAM_FULL_PIPELINE_STATUS=INCOMPLETE reason=preflight_or_runtime_error')
print('RAM_FULL_PIPELINE_PROBE_FINISHED no_device_action_by_builder=1')
# Bounded stdout-drain interval outside measured camera loop.
time.sleep_ms(1000)
'''


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _module_sources(module_root: Path = OPENMV_ROOT) -> list[tuple[str, bytes]]:
    module_root = Path(module_root)
    if len(RUNTIME_NAMES) != 23:
        raise ValueError("exactly 23 production modules are required")
    if len(set(RUNTIME_NAMES)) != len(RUNTIME_NAMES):
        raise ValueError("build_track_profiles runtime module names are duplicated")
    sources = []
    for name in RUNTIME_NAMES:
        if Path(name).name != name or not name.endswith(".py"):
            raise ValueError("runtime allowlist contains an unsafe name: " + name)
        source_path = module_root / name
        if not source_path.is_file():
            raise FileNotFoundError("missing production module: " + str(source_path))
        sources.append((name, source_path.read_bytes()))
    by_name = {name: raw.decode("utf-8") for name, raw in sources}
    if not re.search(r"(?m)^JOINT_ROUTE_API_VERSION\s*=\s*1\s*$",
                     by_name.get("track_elements.py", "")):
        raise ValueError("track_elements.py must expose JOINT_ROUTE_API_VERSION=1")
    if not re.search(r"(?m)^FORMAT_VERSION\s*=\s*3\s*$",
                     by_name.get("ring_diagnostics.py", "")):
        raise ValueError("ring_diagnostics.py must expose FORMAT_VERSION=3")
    main_source = by_name.get("main.py", "")
    for contract in (
            "class _NativeUSBPreviewSource:",
            "def _is_native_usb_preview_source(",
            "def _main_fast(",
            "def _draw_preview_frame_marker(",
            "PREVIEW_FPS_MARKER_ENABLED"):
        if contract not in main_source:
            raise ValueError("production main.py is missing native preview contract: " + contract)
    return sources


def validate_preview_marker_order(main_source: str) -> None:
    """Require processing overlay -> new-frame marker -> native flush order."""
    start = main_source.find("def _render_v7_preview_frame(")
    end = main_source.find("def _format_gray_vdbg(", start)
    if start < 0 or end < 0:
        raise ValueError("production v25 renderer boundaries are missing")
    renderer = main_source[start:end]
    draw_pos = renderer.find("_draw_gray_result(")
    marker_pos = renderer.find("_draw_preview_frame_marker(img, preview_sequence)")
    flush_pos = renderer.find("flush_func()")
    if not (0 <= draw_pos < marker_pos < flush_pos):
        raise ValueError(
            "production marker must run after frame processing and before preview flush")
    if "This runs only after detection and the observation UART write." not in renderer:
        raise ValueError("production renderer timing contract changed")


def build_probe_source(
    mode: str,
    duration_s: int = 70,
    module_root: Path = OPENMV_ROOT,
) -> str:
    if mode not in ("off", "jpeg"):
        raise ValueError("mode must be off or jpeg")
    if isinstance(duration_s, bool) or not isinstance(duration_s, int) or duration_s < 70:
        raise ValueError("duration_s must be an integer of at least 70")
    sources = _module_sources(module_root)
    by_name = {name: raw.decode("utf-8") for name, raw in sources}
    validate_preview_marker_order(by_name["main.py"])

    parts = [RAM_VFS_PREFIX.rstrip("\r\n")]
    for name, raw in sources:
        parts.append(" _ram_sources[%r]=%r" % (name, zlib_compress(raw)))
    total_run_s = PROFILE_GATE_TIMEOUT_S + duration_s + ACTIVE_WINDOW_ALIGNMENT_MARGIN_S
    suffix = PROBE_SUFFIX.replace(
        "RUNTIME_NAMES_PLACEHOLDER", repr(tuple(RUNTIME_NAMES))).replace(
            "MODE_PLACEHOLDER", repr(mode)).replace(
                "ACTIVE_SECONDS_PLACEHOLDER", str(duration_s)).replace(
                    "RUN_SECONDS_PLACEHOLDER", str(total_run_s))
    parts.append(suffix.strip("\r\n"))
    source = "\n".join(parts) + "\n"
    source = source.replace("\r\n", "\n").replace("\r", "\n")
    compile(source, "<openmv-full-pipeline-ram-probe>", "exec")
    return source.replace("\n", "\r\n")


def zlib_compress(raw: bytes) -> bytes:
    # Local import keeps the host builder's imports small and matches the
    # firmware-side deflate.ZLIB reader used by the RAM VFS prefix.
    import zlib
    return zlib.compress(raw)


def build_manifest(
    source: str,
    mode: str,
    duration_s: int,
    module_root: Path = OPENMV_ROOT,
) -> dict:
    sources = _module_sources(module_root)
    return {
        "schema": "openmv-full-pipeline-ram-probe-manifest-v1",
        "probe_mode": mode,
        "active_measurement_duration_s": duration_s,
        "profile_gate_timeout_s": PROFILE_GATE_TIMEOUT_S,
        "active_window_alignment_margin_s": ACTIVE_WINDOW_ALIGNMENT_MARGIN_S,
        "maximum_total_run_s": PROFILE_GATE_TIMEOUT_S + duration_s + ACTIVE_WINDOW_ALIGNMENT_MARGIN_S,
        "source_bytes": len(source.encode("utf-8")),
        "source_sha256": _sha256(source.encode("utf-8")),
        "runtime_module_count": len(RUNTIME_NAMES),
        "runtime_names_source": "tools.build_track_profiles.RUNTIME_NAMES",
        "runtime_modules": [
            {
                "name": name,
                "source_bytes": len(raw),
                "source_sha256": _sha256(raw),
            }
            for name, raw in sources
        ],
        "camera": {
            "sensor": "real OpenMV sensor",
            "format": "GRAYSCALE",
            "crop": [0, 60, 320, 120],
            "algorithm_grid": [80, 60],
            "jpeg_preview": mode == "jpeg",
            "jpeg_quality": "production config.USB_PREVIEW_QUALITY (unchanged)",
            "native_api_required": True,
            "preview_every_capture_requested": mode == "jpeg",
            "actual_ide_new_frame_rate_proven": False,
        },
        "virtual_input": {
            "physical_uart3_access": False,
            "reset_keep_v1_period_ms": 50,
            "profile_state_v2_period_ms": 100,
            "profile_id": "C",
            "parked_calibration_preamble": True,
            "profile_active_flag_after_gate": True,
            "diagnostics_supported_flag": True,
            "session_token": 1,
            "mean_forward_wheel_millirev": 0,
            "v3_write_acceptance_bytes": 16,
            "full_uart_wire_time_included": False,
            "physical_uart_throughput_proven": False,
            "real_vehicle_measurement": False,
        },
        "flash_io": {
            "calibration_path": "production config.RING_WIDTH_CALIBRATION_PATH, read-only",
            "calibration_save_called": False,
            "ring_diagnostics_records_retained_per_frame": True,
            "ring_diagnostics_persist_target": "bounded RAM blob; no board filesystem write",
        },
        "acceptance_limits": [
            "The camera must remain stationary on the mounted camera's calibrated 45 cm straight so the production parked calibration check can pass.",
            "Active profile C starts only after production V3 ACK=3 and valid=1, plus three further 100 ms parked profile statuses; failure within 30 s is FAIL_GATE.",
            "The virtual 16-byte sink has no physical UART wire time; its write rate cannot establish full vehicle UART throughput.",
            "The probe does not prove actual ring-route behavior or vehicle acceptance.",
            "Native USB VCP publication is not proof that the IDE displays every new image or exceeds 60 FPS.",
            "Wheel position remains zero; this is not a real vehicle performance benchmark.",
        ],
    }


def build_artifacts(
    *,
    mode: str,
    duration_s: int = 70,
    module_root: Path = OPENMV_ROOT,
    output: Path | None = None,
    manifest_path: Path | None = None,
) -> tuple[Path, Path, dict]:
    source = build_probe_source(mode, duration_s, module_root)
    if output is None:
        output = DEFAULT_OUTPUT_DIR / (
            "full-pipeline-%s-%ds-ram-probe.py" % (mode, duration_s))
    output = Path(output)
    if manifest_path is None:
        manifest_path = output.with_suffix(".manifest.json")
    manifest_path = Path(manifest_path)
    if output.resolve() == manifest_path.resolve():
        raise ValueError("probe source and manifest paths must differ")
    if output.exists() or manifest_path.exists():
        raise FileExistsError("refusing to overwrite a probe artifact or manifest")
    manifest = build_manifest(source, mode, duration_s, module_root)
    manifest["probe_source_path"] = str(output.resolve())
    manifest["manifest_path"] = str(manifest_path.resolve())
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(source.encode("utf-8"))
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return output.resolve(), manifest_path.resolve(), manifest


def _duration_argument(value: str) -> int:
    try:
        seconds = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer number of seconds") from exc
    if seconds < 70:
        raise argparse.ArgumentTypeError("must be at least 70 seconds")
    return seconds


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("off", "jpeg"),
                        help="production USB preview off or native full-resolution JPEG")
    parser.add_argument("--duration-s", type=_duration_argument, default=70,
                        help="board measurement duration, minimum 70 seconds")
    parser.add_argument("--module-root", type=Path, default=OPENMV_ROOT,
                        help="production OpenMV source directory")
    parser.add_argument("--output", type=Path,
                        help="private generated RAM source; default is under logs/diagnostics")
    parser.add_argument("--manifest", type=Path,
                        help="private source-hash manifest; default beside --output")
    args = parser.parse_args(argv)
    output, manifest_path, manifest = build_artifacts(
        mode=args.mode,
        duration_s=args.duration_s,
        module_root=args.module_root,
        output=args.output,
        manifest_path=args.manifest,
    )
    print("source=%s" % output)
    print("manifest=%s" % manifest_path)
    print("source_sha256=%s" % manifest["source_sha256"])
    print("runtime_modules=%d profile_c_route_guaranteed=0 "
          "actual_ide_acceptance_proven=0 physical_uart=0 flash_write=0" %
          manifest["runtime_module_count"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
