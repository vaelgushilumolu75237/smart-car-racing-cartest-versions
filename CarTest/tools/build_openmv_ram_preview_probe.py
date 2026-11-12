#!/usr/bin/env python3
"""Build a self-contained RAM-VFS OpenMV full-resolution JPEG probe."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OPENMV_ROOT = PROJECT_ROOT / "openmv"
DEFAULT_OUTPUT = (PROJECT_ROOT / "logs" / "usb-preview-20261024" /
                  "fullres-q30-ram-source.py")

# These are the eleven production modules loaded by the established RAM VFS
# probe packaging pattern. Firmware-provided sensor/image/csi/micropython are
# deliberately not bundled.
RUNTIME_MODULES = (
    "config",
    "vision_protocol",
    "fast_topology",
    "fast_gray",
    "fast_path",
    "fast_boundary",
    "fast_single_edge",
    "reverse_control",
    "contour_trace",
    "contour_route",
    "main",
)

RAM_VFS_PREFIX = '''import sys,time
try:
 import gc,deflate,io,vfs,micropython
 _ram_sources={}
 class _FS:
  def mount(self,*args):pass
  def umount(self):pass
  def stat(self,path):
   if path in ('','/'):return (0x4000,0,0,0,0,0,0,0,0,0)
   if path.lstrip('/') in _ram_sources:return (0x8000,0,0,0,0,0,1,0,0,0)
   raise OSError(2)
  def open(self,path,mode):
   name=path.lstrip('/')
   if name not in _ram_sources:raise OSError(2)
   raw=deflate.DeflateIO(io.BytesIO(_ram_sources[name]),deflate.ZLIB).read()
   return io.BytesIO(raw)
 vfs.mount(_FS(),'/rambench')
 sys.path.insert(0,'/rambench')
'''

VIPER_MARKER_FUNCTION = ''' @micropython.viper
 def _write_preview_marker_viper(raw,sequence:int)->int:
  pixels=ptr8(raw)
  seq=sequence&0xffff
  crc=0
  for byte_index in range(3):
   if byte_index==0:
    value=0xD3
   elif byte_index==1:
    value=(seq>>8)&0xff
   else:
    value=seq&0xff
   crc=crc^value
   for crc_bit in range(8):
    if crc&0x80:
     crc=((crc<<1)^0x07)&0xff
    else:
     crc=(crc<<1)&0xff
  for bit_index in range(32):
   if bit_index<8:
    bit=(0xD3>>(7-bit_index))&1
   elif bit_index<24:
    bit=(seq>>(23-bit_index))&1
   else:
    bit=(crc>>(31-bit_index))&1
   value=0
   if bit:value=255
   x=32+bit_index*8
   for dy in range(8):
    row=(104+dy)*320+x
    for dx in range(8):
     pixels[row+dx]=value
  return crc'''

RAM_PROBE_SUFFIX = '''
 for _name in RUNTIME_MODULES_PLACEHOLDER:
  if _name in sys.modules:del sys.modules[_name]
  __import__(_name)
  print('RAM_LOADED',_name)
  gc.collect()
 import main,sensor,reverse_control
 try:
  import csi
 except ImportError:
  csi=None
 if getattr(sensor,"PREVIEW_CONTROL_API",None)!=1:
  raise RuntimeError("sensor.PREVIEW_CONTROL_API=1 is required")
 if getattr(csi,"PREVIEW_CONTROL_API",None)!=1:
  raise RuntimeError("csi.PREVIEW_CONTROL_API=1 is required")
 RUN_SECONDS=30
 RUN_ID=20261024
 PREVIEW_QUALITY=30
 PREVIEW_X=32
 PREVIEW_Y=104
 PREVIEW_CELL=8
 PREVIEW_WIDTH=320
 PREVIEW_HEIGHT=120
 PREVIEW_MAGIC=0xD3
 _marker_stats=[0,0]
VIPER_MARKER_FUNCTION_PLACEHOLDER
 class _Source:
  name='LIVE_SENSOR_UART_SINK_FULLRES_Q30_MARKER'
  frame_count=0
  preview_count=0
  marker_us=0
  marker_writes=0
  def __init__(self):
   self.latest_image=None
  def snapshot(self):
   self.latest_image=sensor.snapshot(preview=False)
   _Source.frame_count+=1
   return self.latest_image
  def flush(self):
   image=self.latest_image
   if image is None:return
   started=time.ticks_us()
   _write_preview_marker_viper(image.bytearray(),_Source.frame_count)
   _Source.marker_us+=time.ticks_diff(time.ticks_us(),started)
   _Source.marker_writes+=1
   preview=image.to_jpeg(x_scale=1.0,y_scale=1.0,
                         quality=PREVIEW_QUALITY,copy=True)
   preview.flush()
   _Source.preview_count+=1
   self.latest_image=None
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
 print("RAM_USB_PREVIEW_PROBE start: capture=320x120 grayscale "
       "preview=320x120 JPEG q=30 every=1 marker=D3/seq16/crc8 "
       "x=32..287 y=104..111 UART=RESET/KEEP sink")
 gc.collect()
 main._configure_gray_sensor(sensor,fast=True)
 main._main_fast(duration_s=RUN_SECONDS,run_id=RUN_ID,
                 frame_source=_Source(),uart_sink=_Sink())
 print("BOARD_PREVIEW_MARKER_COUNTS input_frames=%d preview_publishes=%d "
       "marker_writes=%d marker_total_us=%d" % (
           _Source.frame_count,_Source.preview_count,
           _Source.marker_writes,_Source.marker_us))
except Exception as exc:
 sys.print_exception(exc)
print("RAM_USB_PREVIEW_PROBE_FINISHED")
# Bounded stdout-drain interval; outside measured processing windows.
time.sleep_ms(2000)
'''


def build_probe_source(module_root: Path = OPENMV_ROOT) -> str:
    module_root = Path(module_root)
    # Preserve the VFS script's one-space try-suite indentation while joining
    # each generated assignment as a separate source line.  The compressed
    # payloads are literals, so this builder never reads or writes camera files.
    parts = [RAM_VFS_PREFIX.rstrip("\r\n")]
    for name in RUNTIME_MODULES:
        source_path = module_root / (name + ".py")
        source = source_path.read_bytes()
        compressed = zlib.compress(source)
        parts.append(" _ram_sources[%r]=%r" % (name + ".py", compressed))
    suffix = RAM_PROBE_SUFFIX.replace(
        "RUNTIME_MODULES_PLACEHOLDER",
        repr(list(RUNTIME_MODULES))).replace(
            "VIPER_MARKER_FUNCTION_PLACEHOLDER", VIPER_MARKER_FUNCTION)
    parts.append(suffix.strip("\r\n"))
    source = "\n".join(parts) + "\n"
    # RAM VFS scripts use a single-space try suite, matching the established
    # generated source layout and MicroPython's accepted syntax.
    return source.replace("\r\n", "\n").replace("\r", "\n").replace(
        "\n", "\r\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="RAM-only source output path")
    parser.add_argument("--module-root", type=Path, default=OPENMV_ROOT,
                        help="Production OpenMV module directory")
    args = parser.parse_args(argv)
    source = build_probe_source(args.module_root)
    compile(source, str(args.output), "exec")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(source.encode("utf-8"))
    print("output=%s" % output)
    print("bytes=%d" % len(source.encode("utf-8")))
    print("sha256=%s" % hashlib.sha256(source.encode("utf-8")).hexdigest())
    print("modules=%s" % ",".join(RUNTIME_MODULES))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
