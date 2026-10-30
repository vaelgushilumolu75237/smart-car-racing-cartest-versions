#!/usr/bin/env python3
"""Build a RAM-only OpenMV ring-entry diagnostic candidate.

The source embeds the unchanged 23-module full-pipeline probe, its virtual
RESET/KEEP and parked-ACK3 gate, and bounded RAM-only RNGD persistence. It adds
per-frame ring geometry records and a small bounded set of raw grayscale
images. This builder never connects to or uploads to a camera.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
try:
    from . import build_openmv_full_pipeline_perf_probe as full_probe
except ImportError:
    import build_openmv_full_pipeline_perf_probe as full_probe


CAPTURE_INJECTION = r''' _EntryCaptureAbort=type('_EntryCaptureAbort',(BaseException,),{})
 _EntryCaptureComplete=type('_EntryCaptureComplete',(BaseException,),{})
 def _entry_emit(state,line):
  emitter=state.get('emit')
  if callable(emitter):emitter(line)
  else:print(line)
 def _entry_copy_array(value):
  if value is None:return None
  return [int(item) for item in value]
 def _entry_copy_image(image):
  if image is None:raise ValueError('source.latest_image is missing')
  width=image.width()
  height=image.height()
  if width!=320 or height!=120:raise ValueError('source image dimensions differ from 320x120')
  reader=getattr(image,'bytearray',None)
  if not callable(reader):raise ValueError('source image has no bytearray() reader')
  raw=reader()
  if len(raw)!=38400:raise ValueError('source image bytearray length differs from 38400')
  # Force an owned immutable copy; OpenMV bytearray() exposes its live frame.
  return bytes(bytearray(raw))
 def _entry_take_image(state,image,sequence,capture_ms):
  if len(state['images'])>=8:return False
  data=_entry_copy_image(image)
  state['images'].append({'sequence':int(sequence),'capture_ms':int(capture_ms),
      'width':320,'height':120,'byte_count':38400,'data':data})
  return True
 def _entry_threshold_config(config):
  result={}
  for name in dir(config):
   if (name.startswith(('TRACK_ELEMENTS_','SURFACE_','GRAY_',
                        'SINGLE_EDGE_','FAST_','RING_')) or
       name=='ROAD_REFERENCE_Y'):
    value=getattr(config,name)
    if isinstance(value,(bool,int,float,str)):
     result[name]=value
    elif isinstance(value,(tuple,list)):
     clean=[]
     for item in value:
      if isinstance(item,(bool,int,float,str)):
       clean.append(item)
      else:
       clean=[]
       break
     if clean:result[name]=clean
  return result
 def _entry_tracker_thresholds(tracker):
  result={}
  for name in dir(tracker):
   if name.startswith(('ring_','min_','max_','near_rows','floor_y',
                       'reference_y','opposite_','reappear_',
                       'peak_','exit_','require_calibration')):
    value=getattr(tracker,name)
    if isinstance(value,(bool,int,float)):
     result[name]=value
  return result
 def _entry_frame_payload(state,now_ms,sequence,profile_id,observation_valid,
                          selection,tracker_result):
  tracker=state.get('tracker')
  if tracker is None:raise RuntimeError('actual TrackElementsTracker instance was not captured')
  if not isinstance(selection,dict):selection={}
  if not isinstance(tracker_result,dict):tracker_result={}
  get=lambda name,default=None:getattr(tracker,name,default)
  return {
   'sequence':int(sequence),
   'capture_ms':int(now_ms),
   'profile':int(profile_id),
   'valid':bool(observation_valid),
   'source':selection.get('source','NONE'),
   'source_reason':selection.get('reason',''),
   'state':tracker_result.get('state',''),
   'reason':tracker_result.get('reason',''),
   'candidate':bool(tracker_result.get('candidate')),
   'ring_confirmed':bool(tracker_result.get('ring_confirmed')),
   'side':tracker_result.get('side',-1),
   'arc_rejection':tracker_result.get('ring_arc_rejection','NONE'),
   'arc_rows':tracker_result.get('ring_arc_support_rows',0),
   'arc_span_rows':tracker_result.get('ring_arc_span_rows',0),
   'near_anchor_rows':tracker_result.get('ring_near_anchor_rows',0),
   'near_anchor_run':tracker_result.get('ring_near_anchor_run',0),
   'growth_frames':tracker_result.get('ring_arc_growth_frames',0),
   'growth_x2':tracker_result.get('ring_arc_growth_x2',0),
   'target_center2':tracker_result.get('target_center2',-1),
   'target_side':tracker_result.get('target_side',-1),
   'route_admission_reason':tracker_result.get('route_admission_reason','NOT_RUN'),
   'diagnostic_sent_error':tracker_result.get('diagnostic_sent_error',0),
   'row_status':_entry_copy_array(get('_current_status')),
   'row_edge2':_entry_copy_array(get('_current_edge2')),
   'baseline_valid':_entry_copy_array(get('_baseline_row_valid')),
   'baseline_left2':_entry_copy_array(get('_baseline_left_by_y2')),
   'baseline_right2':_entry_copy_array(get('_baseline_right_by_y2')),
   'outer_shift_x2':_entry_copy_array(get('_outer_shift_x2')),
   'calibration_width_valid':_entry_copy_array(get('_calibration_width_valid')),
   'calibration_width_cells':_entry_copy_array(get('_calibration_width_cells')),
   'ring_width_valid':_entry_copy_array(get('_ring_width_valid')),
   'ring_width':_entry_copy_array(get('_ring_width')),
   'floor_y':get('floor_y',-1),
   'near_rows':get('near_rows',-1),
   'require_calibration':bool(get('require_calibration',False)),
   'calibration_ready':bool(get('_calibration_ready',False)),
   'calibration_check_ready':bool(get('_cal_check_ready',False)),
   'ring_width_calibrated':bool(get('_ring_width_calibrated',False)),
   'reappear_count':get('_reappear_count',0),
   'baseline_ready':bool(get('_baseline_ready',False)),
   'current_valid':bool(get('_current_valid',False)),
   'current_road':bool(get('_current_road',False)),
   'current_topology_clear':bool(get('_current_topology_clear',False)),
   'near_left_ambiguous':get('_near_left_ambiguous',0),
   'near_right_ambiguous':get('_near_right_ambiguous',0),
   'near_left_rows':get('_near_left_rows',0),
   'near_right_rows':get('_near_right_rows',0),
   'near_left_run':get('_near_left_run',0),
   'near_right_run':get('_near_right_run',0),
  }
 def _entry_capture_error(state,stage,exc):
  state['error']=str(stage)+': '+type(exc).__name__+': '+str(exc)
  try:
   detail=state['json'].dumps({'stage':stage,'error_type':type(exc).__name__,
                               'error':str(exc)})
   state['error_line']='ENTRY_CAPTURE_ERROR '+detail
  except BaseException:
   state['error_line']='ENTRY_CAPTURE_ERROR stage='+str(stage)
  raise _EntryCaptureAbort()
 def _entry_cache_active_frame(state,source,record,json_module):
  sequence=int(record['sequence'])&0xffff
  capture_ms=record['capture_ms']
  line=json_module.dumps(record)
  state['processed_frame_count']+=1
  image=source.latest_image
  if state['candidate_sequence'] is None:
   if record['candidate']:
    state['frame_lines']=[item for item in state['pre_frames']]
    state['frame_lines'].append((sequence,line))
    if len(state['frame_lines'])>129:
     raise ValueError('candidate frame cache exceeded 129 records')
    state['candidate_sequence']=sequence
    state['images']=[item for item in state['pre_images']]
    state['pre_images']=[]
    _entry_take_image(state,image,sequence,capture_ms)
    state['candidate_post_count']=1
    state['candidate_following_count']=0
   else:
    state['pre_frames'].append((sequence,line))
    if len(state['pre_frames'])>32:state['pre_frames'].pop(0)
    if sequence%8==0:
     temporary={'images':[]}
     _entry_take_image(temporary,image,sequence,capture_ms)
     state['pre_images'].append(temporary['images'][0])
     if len(state['pre_images'])>4:state['pre_images'].pop(0)
    retained_sequences=set(item[0] for item in state['pre_frames'])
    state['pre_images']=[item for item in state['pre_images']
                         if item['sequence'] in retained_sequences]
  else:
   state['frame_lines'].append((sequence,line))
   if len(state['frame_lines'])>129:
    raise ValueError('candidate frame cache exceeded 129 records')
   state['candidate_following_count']+=1
   delta=(sequence-state['candidate_sequence'])&0xffff
   if delta>0 and delta%8==0 and state['candidate_post_count']<4:
    if _entry_take_image(state,image,sequence,capture_ms):
     state['candidate_post_count']+=1
   if state['candidate_following_count']>=96:
    state['complete']=True
    raise _EntryCaptureComplete()
  if state['processed_frame_count']%32==0:
   print('ENTRY_BUFFERED seq=%d n=%d' %
         (sequence,state['processed_frame_count']))
 def _entry_install_capture_hooks(state,track_elements,ring_diagnostics,
                                 source,config_module,json_module):
  tracker_class=track_elements.TrackElementsTracker
  diagnostics_class=ring_diagnostics.RingDiagnostics
  original_init=tracker_class.__init__
  original_record=diagnostics_class.record_frame
  state['json']=json_module
  state['thresholds_config']=_entry_threshold_config(config_module)
  state.setdefault('pre_frames',[])
  state.setdefault('pre_images',[])
  state.setdefault('frame_lines',[])
  state.setdefault('images',[])
  state.setdefault('processed_frame_count',0)
  state.setdefault('candidate_sequence',None)
  state.setdefault('candidate_post_count',0)
  state.setdefault('candidate_following_count',0)
  state.setdefault('dumped_image_count',0)
  state.setdefault('dumped_frame_count',0)
  state.setdefault('error',None)
  state.setdefault('error_line',None)
  state.setdefault('complete',False)
  state.setdefault('config_json',None)
  state.setdefault('config_emitted',False)
  def wrapped_init(self,*args,**kwargs):
   try:
    original_init(self,*args,**kwargs)
    state['tracker']=self
    if not state.get('config_emitted'):
     state['thresholds_tracker']=_entry_tracker_thresholds(self)
     state['config_json']=json_module.dumps({
         'thresholds_config':state['thresholds_config'],
         'thresholds_tracker':state['thresholds_tracker']})
     state['config_emitted']=True
   except BaseException as exc:
    _entry_capture_error(state,'tracker_init',exc)
  def wrapped_record(self,now_ms,sequence,profile_id,active_follow,
                     observation_valid,selection,tracker_result,
                     ticks_diff=None):
   try:
    result=original_record(self,now_ms,sequence,profile_id,active_follow,
        observation_valid,selection,tracker_result,ticks_diff)
   except BaseException as exc:
    _entry_capture_error(state,'record_frame',exc)
   if active_follow:
    try:
     record=_entry_frame_payload(state,now_ms,sequence,profile_id,
                                 observation_valid,selection,tracker_result)
     _entry_cache_active_frame(state,source,record,json_module)
    except BaseException as exc:
     if isinstance(exc,_EntryCaptureComplete):raise
     if isinstance(exc,_EntryCaptureAbort):raise
     _entry_capture_error(state,'frame_capture',exc)
   return result
  tracker_class.__init__=wrapped_init
  diagnostics_class.record_frame=wrapped_record
  state['original_tracker_init']=original_init
  state['original_record_frame']=original_record
  return (tracker_class,diagnostics_class,original_init,original_record)
 def _entry_restore_capture_hooks(saved):
  tracker_class,diagnostics_class,original_init,original_record=saved
  tracker_class.__init__=original_init
  diagnostics_class.record_frame=original_record
 def _entry_install_ready_hook(state,sink):
  original_write=sink.write
  def wrapped_write(packet):
   before=sink.active_ack_start_ms
   result=original_write(packet)
   if before is None and sink.active_ack_start_ms is not None:
    _entry_emit(state,'ENTRY_READY_TO_PUSH=1')
   return result
  sink.write=wrapped_write
  return (sink,original_write)
 def _entry_restore_ready_hook(saved):
  sink,original_write=saved
  sink.write=original_write
 def _entry_crc32_hex(data,crc32_fn):
  return '%08x' % (int(crc32_fn(data))&0xffffffff)
 class _EntryPullMetaBackend:
  def __init__(self,data):
   self._data=data
  def size(self):
   return len(self._data)
  def read(self,offset,size):
   offset=int(offset)
   size=int(size)
   if offset<0 or size<0:raise ValueError('negative protocol read')
   return self._data[offset:min(len(self._data),offset+size)]
 class _EntryPullDataBackend:
  def __init__(self,segments):
   self._segments=[]
   offset=0
   for segment in segments:
    if isinstance(segment,str):
     encoded=segment.encode('utf-8')
     segment_size=len(encoded)
     del encoded
    else:
     segment_size=len(segment)
    self._segments.append((offset,segment_size,segment))
    offset+=segment_size
   self._size=offset
  def size(self):
   return self._size
  def read(self,offset,size):
   offset=int(offset)
   size=int(size)
   if offset<0 or size<0:raise ValueError('negative protocol read')
   end=min(self._size,offset+size)
   if offset>=end:return bytearray()
   output=bytearray()
   for start,segment_size,segment in self._segments:
    segment_end=start+segment_size
    if segment_end<=offset:continue
    if start>=end:break
    copy_start=max(offset,start)-start
    copy_end=min(end,segment_end)-start
    encoded=(segment.encode('utf-8') if isinstance(segment,str)
             else segment)
    output.extend(encoded[copy_start:copy_end])
   return output
 def _entry_build_pull_payload(state,json_module,crc32_fn):
  config_json=state.get('config_json')
  if config_json is None:
   if state.get('error') is None:
    state['error']='tracker threshold config record is missing'
   config_json='{}'
  if state.get('candidate_sequence') is None:
   frames=state.get('pre_frames',[])
  else:
   frames=state.get('frame_lines',[])
  images=state.get('images',[])
  if state.get('candidate_sequence') is None:
   images=state.get('pre_images',[])
  if len(frames)>129:raise ValueError('frame pull segment count exceeds 129')
  if len(images)>8:raise ValueError('image pull segment count exceeds eight')
  segments=[config_json]
  offset=0
  config_bytes=config_json.encode('utf-8')
  config_record={'offset':offset,'size':len(config_bytes),
                 'crc32':_entry_crc32_hex(config_bytes,crc32_fn)}
  offset+=len(config_bytes)
  del config_bytes
  frame_meta=[]
  for sequence,frame_json in frames:
   encoded=frame_json.encode('utf-8')
   segment_size=len(encoded)
   frame_meta.append({'sequence':int(sequence),'offset':offset,
       'size':segment_size,'crc32':_entry_crc32_hex(encoded,crc32_fn)})
   segments.append(frame_json)
   offset+=segment_size
   del encoded
  image_meta=[]
  for image_id,item in enumerate(images):
   data=item['data']
   if len(data)!=38400:raise ValueError('captured image length differs from 38400')
   image_meta.append({'image_id':image_id,'sequence':int(item['sequence']),
       'capture_ms':int(item['capture_ms']),'width':320,'height':120,
       'byte_count':38400,'offset':offset,'size':38400,
       'crc32':_entry_crc32_hex(data,crc32_fn)})
   segments.append(data)
   offset+=38400
  metadata={'schema':'entry-pull-v1','config':config_record,
      'frames':frame_meta,'images':image_meta,'frame_count':len(frame_meta),
      'processed_frame_count':state.get('processed_frame_count',0),
      'image_count':len(image_meta),'error':1 if state.get('error') else 0,
      'error_message':state.get('error') or ''}
  meta_bytes=json_module.dumps(metadata).encode('utf-8')
  state['dumped_frame_count']=len(frame_meta)
  state['dumped_image_count']=len(image_meta)
  return meta_bytes,segments,metadata
 def _entry_register_pull_channels(state,protocol_module,json_module,crc32_fn):
  meta_bytes,segments,metadata=_entry_build_pull_payload(
      state,json_module,crc32_fn)
  meta_backend=_EntryPullMetaBackend(meta_bytes)
  data_backend=_EntryPullDataBackend(segments)
  protocol_module.register(name='entry_meta',backend=meta_backend)
  protocol_module.register(name='entry_data',backend=data_backend)
  return meta_backend,data_backend,metadata
'''


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _inject_probe(base_source: str) -> str:
    source = base_source.replace("\r\n", "\n").replace("\r", "\n")
    if source.count(" _RUN_SECONDS=120\n") != 1:
        raise ValueError("full-pipeline probe run-time template changed")
    source = source.replace(" _RUN_SECONDS=120\n", " _RUN_SECONDS=60\n", 1)
    if source.count(" _ACTIVE_SECONDS=70\n") != 1:
        raise ValueError("full-pipeline active-window template changed")
    source = source.replace(" _ACTIVE_SECONDS=70\n", " _ACTIVE_SECONDS=60\n", 1)

    call_anchor = " try:\n  main._main_fast(duration_s=_RUN_SECONDS"
    if source.count(call_anchor) != 1:
        raise ValueError("full-pipeline main-call insertion point changed")
    setup = CAPTURE_INJECTION + r'''
 _entry_capture_state={'tracker':None,'images':[],'pre_images':[],
     'pre_frames':[],'frame_lines':[],'candidate_sequence':None,
     'candidate_post_count':0,'candidate_following_count':0,
     'processed_frame_count':0,'dumped_frame_count':0,
     'dumped_image_count':0,'error':None,'error_line':None,
     'config_json':None,'config_emitted':False,'complete':False,
     'emit':print}
 import ujson as _entry_ujson
 import ubinascii as _entry_ubinascii
 _entry_capture_state['json']=_entry_ujson
 _entry_capture_restore=_entry_install_capture_hooks(
     _entry_capture_state,track_elements,ring_diagnostics,_source,
     config,_entry_ujson)
 _entry_ready_restore=_entry_install_ready_hook(_entry_capture_state,_sink)
'''
    source = source.replace(call_anchor, setup + call_anchor, 1)

    catch_anchor = "  main._main_fast(duration_s=_RUN_SECONDS,run_id=_RUN_ID,\n                  frame_source=_source,uart_sink=_sink,\n                  fixture_sensor_id=_sensor_id)\n finally:\n"
    catch_replacement = "  main._main_fast(duration_s=_RUN_SECONDS,run_id=_RUN_ID,\n                  frame_source=_source,uart_sink=_sink,\n                  fixture_sensor_id=_sensor_id)\n except _EntryCaptureComplete:\n  _entry_capture_state['complete']=True\n except _EntryCaptureAbort:\n  _entry_capture_state['error']=_entry_capture_state.get('error') or 'capture hook aborted'\n finally:\n"
    if source.count(catch_anchor) != 1:
        raise ValueError("full-pipeline run-finally insertion point changed")
    source = source.replace(catch_anchor, catch_replacement, 1)

    restore_anchor = "  main._V7PerfWindow=_original_perf_factory\n"
    restore_replacement = restore_anchor + "  _entry_restore_capture_hooks(_entry_capture_restore)\n  _entry_restore_ready_hook(_entry_ready_restore)\n"
    if source.count(restore_anchor) != 1:
        raise ValueError("full-pipeline hook-restoration insertion point changed")
    source = source.replace(restore_anchor, restore_replacement, 1)

    status_anchor = " if _sink.active_ack_start_ms is None:\n"
    status_replacement = " if _entry_capture_state.get('error') is not None:\n  print('RAM_FULL_PIPELINE_STATUS=INCOMPLETE reason=entry_capture_error')\n elif _sink.active_ack_start_ms is None:\n"
    if source.count(status_anchor) != 1:
        raise ValueError("full-pipeline status insertion point changed")
    source = source.replace(status_anchor, status_replacement, 1)

    finish_anchor = "print('RAM_FULL_PIPELINE_PROBE_FINISHED no_device_action_by_builder=1')\n"
    dump_block = r'''if '_entry_capture_state' in globals():
 try:
  import protocol as _entry_protocol
  _entry_meta_backend,_entry_data_backend,_entry_pull_metadata=(
      _entry_register_pull_channels(_entry_capture_state,_entry_protocol,
          _entry_ujson,_entry_ubinascii.crc32))
  print('ENTRY_PULL_READY frame_count=%d image_count=%d' % (
      _entry_pull_metadata['frame_count'],
      _entry_pull_metadata['image_count']))
  _entry_pull_deadline=time.ticks_add(time.ticks_ms(),180000)
  while time.ticks_diff(_entry_pull_deadline,time.ticks_ms())>0:
   try:time.sleep_ms(10)
   except KeyboardInterrupt:break
 except BaseException as _entry_pull_exc:
  print('ENTRY_CAPTURE_ERROR stage=pull_register type=%s' %
        type(_entry_pull_exc).__name__)
  print('ENTRY_CAPTURE_DONE frame_count=%d processed_frame_count=%d image_count=%d error=1' % (
      _entry_capture_state.get('dumped_frame_count',0),
      _entry_capture_state.get('processed_frame_count',0),
      _entry_capture_state.get('dumped_image_count',0)))
  print('RAM_FULL_PIPELINE_STATUS=INCOMPLETE reason=pull_registration_error')
else:
 print('ENTRY_CAPTURE_ERROR stage=probe_setup_missing')
 print('ENTRY_CAPTURE_DONE frame_count=0 processed_frame_count=0 image_count=0 error=1')
'''
    if source.count(finish_anchor) != 1:
        raise ValueError("full-pipeline completion insertion point changed")
    source = source.replace(finish_anchor, dump_block + finish_anchor, 1)
    source = source.replace("\n", "\r\n")
    compile(source, "<openmv-ring-entry-ram-probe>", "exec")
    return source


def build_probe_source(module_root: Path = full_probe.OPENMV_ROOT) -> str:
    """Return the instrumented probe with the production module payload intact."""
    base = full_probe.build_probe_source(
        mode="off", duration_s=70, module_root=module_root)
    return _inject_probe(base)


def build_manifest(source: str, module_root: Path = full_probe.OPENMV_ROOT) -> dict:
    base = full_probe.build_probe_source(
        mode="off", duration_s=70, module_root=module_root)
    base_manifest = full_probe.build_manifest(
        base, mode="off", duration_s=70, module_root=module_root)
    return {
        "schema": "openmv-ring-entry-ram-probe-manifest-v1",
        "probe_mode": "off",
        "run_limit_s": 60,
        "not_fps_acceptance": True,
        "source_bytes": len(source.encode("utf-8")),
        "source_sha256": _sha256(source.encode("utf-8")),
        "probe_sha256": _sha256(source.encode("utf-8")),
        "embedded_full_pipeline_probe_sha256": base_manifest["source_sha256"],
        "runtime_module_count": base_manifest["runtime_module_count"],
        "runtime_names_source": base_manifest["runtime_names_source"],
        "runtime_modules": base_manifest["runtime_modules"],
        "virtual_input": base_manifest["virtual_input"],
        "flash_write": False,
        "physical_uart": False,
        "ring_diagnostics_persist_target": "bounded RAM blob; no board filesystem write",
        "capture_record_fields": [
            "sequence", "capture_ms", "profile", "valid", "source",
            "source_reason", "state", "reason", "candidate", "ring_confirmed",
            "side", "arc_rejection", "arc_rows", "arc_span_rows",
            "near_anchor_rows", "near_anchor_run", "growth_frames", "growth_x2",
            "target_center2", "target_side", "route_admission_reason",
            "diagnostic_sent_error", "row_status", "row_edge2", "baseline_valid",
            "baseline_left2", "baseline_right2", "outer_shift_x2",
            "calibration_width_valid", "calibration_width_cells",
            "ring_width_valid", "ring_width", "floor_y", "near_rows", "require_calibration",
            "calibration_ready", "calibration_check_ready", "reappear_count",
            "baseline_ready", "current_valid", "current_road",
            "current_topology_clear", "near_left_ambiguous",
            "near_right_ambiguous", "near_left_rows", "near_right_rows",
            "near_left_run", "near_right_run",
        ],
        "threshold_config_segment": "first entry_data segment; referenced by entry_meta.config",
        "frame_buffer": {
            "stored_format": "UTF-8 frame JSON strings without ENTRY_FRAME prefix",
            "pre_records": 32,
            "candidate_record_included": True,
            "following_records": 96,
            "maximum_records": 129,
            "no_candidate_dump_records": 32,
        },
        "images": {
            "format": "raw grayscale bytes",
            "width": 320,
            "height": 120,
            "byte_count_each": 38400,
            "max_images": 8,
            "pre_frames": 4,
            "pre_sampling_sequence_modulus": 8,
            "post_frames_including_candidate": 4,
            "post_sampling_sequence_stride": 8,
            "estimated_max_payload_bytes": 307200,
            "crc32": "8 lowercase hexadecimal digits; CRC-32 masked to uint32",
        },
        "pull_transport": {
            "kind": "OpenMV dynamic protocol channels",
            "meta_channel": "entry_meta",
            "data_channel": "entry_data",
            "read_only": True,
            "random_access": True,
            "max_wait_s": 180,
            "no_protocol_init": True,
            "data_segments": [
                "UTF-8 config JSON",
                "N UTF-8 frame JSON segments without ENTRY_FRAME prefix",
                "M raw grayscale image segments",
            ],
        },
        "acceptance_limits": [
            "This is a bounded RAM diagnostic capture, not an FPS acceptance test.",
            "The source retains the full-pipeline builder's virtual sink; no physical UART is used.",
            "The ring-route behavior remains unverified by source generation or synthetic host tests.",
        ],
    }


def build_artifacts(output: Path, module_root: Path = full_probe.OPENMV_ROOT,
                    manifest_path: Path | None = None) -> tuple[Path, Path, dict]:
    source = build_probe_source(module_root)
    output = Path(output)
    if manifest_path is None:
        manifest_path = output.with_suffix(".manifest.json")
    manifest_path = Path(manifest_path)
    if output.resolve() == manifest_path.resolve():
        raise ValueError("probe source and manifest paths must differ")
    if output.exists() or manifest_path.exists():
        raise FileExistsError("refusing to overwrite a probe artifact or manifest")
    manifest = build_manifest(source, module_root)
    manifest["probe_source_path"] = str(output.resolve())
    manifest["manifest_path"] = str(manifest_path.resolve())
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(source.encode("utf-8"))
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return output.resolve(), manifest_path.resolve(), manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="new private generated probe source path")
    parser.add_argument("--manifest", type=Path,
                        help="manifest path (default: beside --output)")
    parser.add_argument("--module-root", type=Path, default=full_probe.OPENMV_ROOT,
                        help="production OpenMV source directory")
    args = parser.parse_args(argv)
    output, manifest_path, manifest = build_artifacts(
        output=args.output, module_root=args.module_root,
        manifest_path=args.manifest)
    print("source=%s" % output)
    print("manifest=%s" % manifest_path)
    print("probe_sha256=%s" % manifest["probe_sha256"])
    print("runtime_modules=%d physical_uart=%s flash_write=%s not_fps_acceptance=%s" % (
        manifest["runtime_module_count"], manifest["physical_uart"],
        manifest["flash_write"], manifest["not_fps_acceptance"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
