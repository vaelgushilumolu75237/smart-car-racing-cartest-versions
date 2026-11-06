#!/usr/bin/env python3
"""Build a bounded synthetic-image Viper marker and JPEG probe for OpenMV."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

try:
    from .build_openmv_ram_preview_probe import VIPER_MARKER_FUNCTION
except ImportError:  # Executed directly from the tools directory.
    from build_openmv_ram_preview_probe import VIPER_MARKER_FUNCTION


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (PROJECT_ROOT / "logs" / "usb-preview-20261024" /
                  "synthetic-marker-ram-source.py")

SYNTHETIC_PROBE_TEMPLATE = '''import sys,time,gc,image,micropython
SYNTHETIC_MARKER_PROBE = True
SYNTHETIC_FRAME_WIDTH = 320
SYNTHETIC_FRAME_HEIGHT = 120
SYNTHETIC_SEQUENCE_FIRST = 65532
SYNTHETIC_SEQUENCE_COUNT = 25
SYNTHETIC_REPEAT_COUNT = 3
SYNTHETIC_PUBLISH_DELAY_MS = 20
SYNTHETIC_DEADLINE_MS = 4500
SYNTHETIC_STDOUT_DRAIN_MS = 2000
_SYNTHETIC_STATUS = "error"
try:
VIPER_MARKER_FUNCTION_PLACEHOLDER
 print("V7_BUILD=synthetic_marker_probe camera=none frame=320x120 "
       "jpeg_quality=30 marker=D3/seq16/crc8")
 _frame=image.Image(SYNTHETIC_FRAME_WIDTH,SYNTHETIC_FRAME_HEIGHT,
                    image.GRAYSCALE)
 _frame.clear()
 _started=time.ticks_ms()
 _publishes=0
 _deadline_hit=0
 for _seq_index in range(SYNTHETIC_SEQUENCE_COUNT):
  if time.ticks_diff(time.ticks_ms(),_started)>=SYNTHETIC_DEADLINE_MS:
   _deadline_hit=1
   break
  _sequence=(SYNTHETIC_SEQUENCE_FIRST+_seq_index)&0xffff
  for _repeat in range(SYNTHETIC_REPEAT_COUNT):
   if time.ticks_diff(time.ticks_ms(),_started)>=SYNTHETIC_DEADLINE_MS:
    _deadline_hit=1
    break
   _write_preview_marker_viper(_frame.bytearray(),_sequence)
   _jpeg=_frame.to_jpeg(x_scale=1.0,y_scale=1.0,quality=30,copy=True)
   _jpeg.flush()
   _publishes+=1
   del _jpeg
   time.sleep_ms(SYNTHETIC_PUBLISH_DELAY_MS)
  if _deadline_hit:break
 _elapsed=time.ticks_diff(time.ticks_ms(),_started)
 print("V7_SYNTHETIC_MARKER_COUNTS first_seq=65532 last_seq=20 "
       "sequence_count=25 repeats_per_sequence=3 publishes=%d "
       "elapsed_ms=%d deadline_hit=%d" % (
           _publishes,_elapsed,_deadline_hit))
 gc.collect()
 _SYNTHETIC_STATUS="ok"
except Exception as exc:
 sys.print_exception(exc)
 print("V7_SYNTHETIC_MARKER_ERROR",type(exc).__name__)
print("V7_SYNTHETIC_MARKER_DONE status=%s" % _SYNTHETIC_STATUS)
time.sleep_ms(SYNTHETIC_STDOUT_DRAIN_MS)
'''


def build_probe_source() -> str:
    source = SYNTHETIC_PROBE_TEMPLATE.replace(
        "VIPER_MARKER_FUNCTION_PLACEHOLDER", VIPER_MARKER_FUNCTION)
    source = source.replace("\r\n", "\n").replace("\r", "\n")
    return source.replace("\n", "\r\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="Local RAM-exec source output path")
    args = parser.parse_args(argv)
    source = build_probe_source()
    compile(source, str(args.output), "exec")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(source.encode("utf-8"))
    print("output=%s" % output)
    print("bytes=%d" % len(source.encode("utf-8")))
    print("sha256=%s" % hashlib.sha256(source.encode("utf-8")).hexdigest())
    print("kind=synthetic_grayscale_marker_no_sensor_capture")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
