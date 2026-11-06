#!/usr/bin/env python3
"""Build a RAM-VFS probe that isolates JPEG encoding from USB preview.

On legacy firmware sensor.snapshot() may auto-publish while the IDE stream is
enabled. Disable the IDE image stream on the host before running if no live
image should reach the computer; this probe never saves or explicitly flushes
an image. Its JPEG sweep starts after capture stops and uses no preview flush.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import zlib

if __package__:
    from .build_openmv_ackermann320_packages import PACKAGES, package_source_text
    from .build_openmv_ram_preview_probe import (
        OPENMV_ROOT, RAM_VFS_PREFIX, RUNTIME_MODULES)
else:
    from build_openmv_ackermann320_packages import PACKAGES, package_source_text
    from build_openmv_ram_preview_probe import (
        OPENMV_ROOT, RAM_VFS_PREFIX, RUNTIME_MODULES)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (PROJECT_ROOT / "logs" / "usb-preview-20261024" /
                  "jpeg-encoder-q10-q60-paced-ram-source.py")
RUN_SECONDS = 3
JPEG_QUALITIES = (10, 20, 30, 40, 50, 60)
SAMPLES_PER_QUALITY = 20
HOST_PRE_ENCODER_PAUSE_S = 2
SAMPLE_LOG_PAUSE_MS = 10

RAM_PROBE_SUFFIX = '''
 for _name in RUNTIME_MODULES_PLACEHOLDER:
  if _name in sys.modules:del sys.modules[_name]
  __import__(_name)
  print('RAM_LOADED',_name)
  gc.collect()
 import main,sensor,reverse_control,config
 RUN_SECONDS=RUN_SECONDS_PLACEHOLDER
 RUN_ID=20261024
 JPEG_QUALITIES=JPEG_QUALITIES_PLACEHOLDER
 SAMPLES_PER_QUALITY=SAMPLES_PER_QUALITY_PLACEHOLDER
 HOST_PRE_ENCODER_PAUSE_S=HOST_PRE_ENCODER_PAUSE_S_PLACEHOLDER
 SAMPLE_LOG_PAUSE_MS=SAMPLE_LOG_PAUSE_MS_PLACEHOLDER
 class _Source:
  name='LIVE_SENSOR_LAST_FULL_OVERLAY_NO_STREAM_FLUSH'
  def __init__(self):
   self.frame_count=0
   self.overlay_frames=0
   self.latest=None
   self.last=None
  def snapshot(self):
   self.latest=sensor.snapshot()
   self.frame_count+=1
   return self.latest
  def flush(self):
   # _main_fast calls this after production overlay drawing. Keep the image
   # reference only; do not call sensor.flush() or Image.flush().
   if self.latest is not None:
    self.last=self.latest
    self.overlay_frames+=1
 class _Sink:
  def __init__(self):
   self.pending=reverse_control.pack_frame(
       reverse_control.COMMAND_RESET,0)
   self.seq=1
   self.due=time.ticks_ms()
  def write(self,packet):return len(packet)
  def any(self):
   if not self.pending and time.ticks_diff(time.ticks_ms(),self.due)>=0:
    self.pending=reverse_control.pack_frame(
        reverse_control.COMMAND_KEEP,self.seq)
    self.seq=(self.seq+1)&65535
    self.due=time.ticks_add(time.ticks_ms(),50)
   return len(self.pending)
  def readinto(self,buffer,requested=None):
   n=min(len(buffer),len(self.pending))
   if requested is not None:n=min(n,requested)
   for index in range(n):buffer[index]=self.pending[index]
   self.pending=self.pending[n:]
   return n
 def _image_signature(image):
  return (image.width(),image.height(),image.format())
 try:
  if not config.OVERLAY_ENABLED:
   raise RuntimeError('full-overlay encoder probe requires OVERLAY_ENABLED')
  print('RAM_JPEG_ENCODER_PROBE start run_s=%d qualities=%s samples_per_quality=%d ' % (
      RUN_SECONDS,str(JPEG_QUALITIES),SAMPLES_PER_QUALITY) +
      'source=320x120_grayscale image_file_saved=false '
      'legacy_snapshot_auto_preview=host-controlled')
  gc.collect()
  main._configure_gray_sensor(sensor,fast=True)
  _source=_Source()
  main._main_fast(duration_s=RUN_SECONDS,run_id=RUN_ID,
                  frame_source=_source,uart_sink=_Sink())
  if _source.last is None or not _source.overlay_frames:
   raise RuntimeError('no post-overlay source image was retained')
  _source_sig=_image_signature(_source.last)
  if (_source_sig[0]!=320 or _source_sig[1]!=120 or
      _source_sig[2]!=sensor.GRAYSCALE):
   raise RuntimeError('retained image is not 320x120 grayscale: %s' %
                      (str(_source_sig),))
  print('JPEG_SOURCE {"frames":%d,"overlay_frames":%d,"width":%d,'
        '"height":%d,"pixformat":%d,"mode":"coordinated"}' % (
            _source.frame_count,_source.overlay_frames,
            _source_sig[0],_source_sig[1],_source_sig[2]))
  print('JPEG_ENCODER_PHASE no further sensor.snapshot or flush calls; '
        'host may disable IDE preview now; pause_s=%d' %
        HOST_PRE_ENCODER_PAUSE_S)
  time.sleep(HOST_PRE_ENCODER_PAUSE_S)
  for quality in JPEG_QUALITIES:
   gc.collect()
   _times=[]
   _sizes=[]
   for sample_index in range(1,SAMPLES_PER_QUALITY+1):
    _started=time.ticks_us()
    _jpeg=_source.last.to_jpeg(copy=True,quality=quality)
    _elapsed=time.ticks_diff(time.ticks_us(),_started)
    _bytes=_jpeg.size()
    _jpeg_w=_jpeg.width()
    _jpeg_h=_jpeg.height()
    _after_sig=_image_signature(_source.last)
    if _after_sig!=_source_sig:
     raise RuntimeError('source image changed during q=%d sample=%d: %s' %
                        (quality,sample_index,str(_after_sig)))
    if _jpeg_w!=_source_sig[0] or _jpeg_h!=_source_sig[1]:
     raise RuntimeError('JPEG size changed at q=%d sample=%d: %dx%d' %
                        (quality,sample_index,_jpeg_w,_jpeg_h))
    _times.append(_elapsed)
    _sizes.append(_bytes)
    print('JPEG_SAMPLE {"q":%d,"sample":%d,"encode_us":%d,'
          '"jpeg_bytes":%d,"jpeg_width":%d,"jpeg_height":%d,'
          '"source_unchanged":1}' % (
              quality,sample_index,_elapsed,_bytes,_jpeg_w,_jpeg_h))
    time.sleep_ms(SAMPLE_LOG_PAUSE_MS)
    _jpeg=None
   _ordered=sorted(_times)
   _p95_index=(len(_ordered)*95+99)//100-1
   _average=(sum(_times)+len(_times)//2)//len(_times)
   print('JPEG_SUMMARY {"q":%d,"samples":%d,"avg_us":%d,'
         '"p95_us":%d,"max_us":%d,"jpeg_bytes_min":%d,'
         '"jpeg_bytes_max":%d,"source_unchanged":1}' % (
             quality,len(_times),_average,_ordered[_p95_index],
             _ordered[-1],min(_sizes),max(_sizes)))
   gc.collect()
  print('JPEG_SOURCE_FINAL {"unchanged":%d,"width":%d,"height":%d,'
        '"pixformat":%d}' % (
            1 if _image_signature(_source.last)==_source_sig else 0,
            _source_sig[0],_source_sig[1],_source_sig[2]))
 except Exception as exc:
  sys.print_exception(exc)
 print('RAM_JPEG_ENCODER_PROBE_DONE')
 time.sleep(8)
except Exception as exc:
 sys.print_exception(exc)
 print('RAM_JPEG_ENCODER_PROBE_DONE')
 time.sleep(8)
'''


def build_probe_source(module_root: Path = OPENMV_ROOT) -> str:
    module_root = Path(module_root)
    coordinated = next(package for package in PACKAGES
                       if package["name"] == "coordinated")
    parts = [RAM_VFS_PREFIX.rstrip("\r\n")]
    for name in RUNTIME_MODULES:
        source_path = module_root / (name + ".py")
        source_text = package_source_text(source_path, coordinated)
        compressed = zlib.compress(source_text.encode("utf-8"))
        parts.append(" _ram_sources[%r]=%r" %
                     (name + ".py", compressed))
    suffix = RAM_PROBE_SUFFIX.replace(
        "RUNTIME_MODULES_PLACEHOLDER", repr(list(RUNTIME_MODULES)))
    replacements = {
        "RUN_SECONDS_PLACEHOLDER": RUN_SECONDS,
        "JPEG_QUALITIES_PLACEHOLDER": JPEG_QUALITIES,
        "SAMPLES_PER_QUALITY_PLACEHOLDER": SAMPLES_PER_QUALITY,
        "HOST_PRE_ENCODER_PAUSE_S_PLACEHOLDER": HOST_PRE_ENCODER_PAUSE_S,
        "SAMPLE_LOG_PAUSE_MS_PLACEHOLDER": SAMPLE_LOG_PAUSE_MS,
    }
    for placeholder, value in replacements.items():
        suffix = suffix.replace(placeholder, repr(value))
    parts.append(suffix.strip("\r\n"))
    source = "\n".join(parts) + "\n"
    source = source.replace("\r\n", "\n").replace("\r", "\n")
    source = source.replace("\n", "\r\n")
    compile(source, "ram_jpeg_encoder_probe.py", "exec")
    return source


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="RAM-only source output path")
    parser.add_argument("--module-root", type=Path, default=OPENMV_ROOT,
                        help="production OpenMV module directory")
    args = parser.parse_args(argv)
    source = build_probe_source(args.module_root)
    payload = source.encode("utf-8")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(payload)
    print("output=%s" % output)
    print("bytes=%d" % len(payload))
    print("sha256=%s" % hashlib.sha256(payload).hexdigest())
    print("modules=%s" % ",".join(RUNTIME_MODULES))
    print("config_profile=coordinated")
    print("source_image_file_saved=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
