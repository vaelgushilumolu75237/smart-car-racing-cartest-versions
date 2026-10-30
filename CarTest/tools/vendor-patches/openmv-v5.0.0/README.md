# OpenMV v5.0.0 optional snapshot preview control

This patch adds `preview=True` to both the legacy `sensor.snapshot()` wrapper
and the v5 `csi.CSI.snapshot()` method. The default remains enabled. Passing
`preview=False` sets the existing `OMV_CSI_FLAG_NO_UPDATE` bit, preventing
that capture from replacing the IDE stream image. Both Python modules export
`PREVIEW_CONTROL_API = 1`; the RAM probe requires both markers before camera
setup, so it cannot silently run against an older firmware that ignores the
legacy snapshot keyword. The caller can then publish a selected full-resolution JPEG
with `image.to_jpeg(...).flush()`.

## Frozen upstream base

- Repository: `https://github.com/openmv/openmv`
- Tag: `v5.0.0`
- Commit: `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`
- MicroPython submodule commit: `d18a77e5d55664045e71a0c839695df07a96a6e1`
- Patch SHA-256: `2BBB92B2B25705002AE16B9E9F18F57FC171CA8F0549680CC15260CCE0B468D5`
- Base `modules/py_csi.c` SHA-256: `6DE532C54D5F0BD4660F82587BEF7F48D2199A17E72B677578BF289605A044D3`
- Base `modules/py_csi_ng.c` SHA-256: `6BA5B285B5BAC277D456C9B576B5516314F6AB8DC3B46325E7FFECB1DDE430A6`

Apply from a clean checkout at that exact commit (use the patch's absolute
path, or copy it into the checkout first):

```sh
git apply --check /path/to/0001-csi-snapshot-preview-kwarg.patch
git apply /path/to/0001-csi-snapshot-preview-kwarg.patch
```

`tools/test_openmv_preview_patch.py` checks the patch structure and can run
`git apply --check` against an upstream source tree with `--source-root`.
These host checks do not replace a C build or board test.

The legacy lookup uses `MP_OBJ_NEW_QSTR(MP_QSTR_preview)` because the pinned
MicroPython `mp_map_lookup` takes an `mp_obj_t` key. `MP_ROM_QSTR` is the
ROM-object initializer used in constant map tables. See the pinned
[MicroPython `obj.h`](https://github.com/micropython/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/py/obj.h).

## RAM call pattern

`openmv/tests/ram_usb_preview_probe.py` uses the production detector path, a
RESET/KEEP UART sink, and the unchanged 320x120 grayscale detector input. It
defaults to a 320x120 JPEG preview on every frame (`PREVIEW_EVERY_N=1`,
`SCALE=1.0`, `PREVIEW_QUALITY=30`). Edit those constants in the probe to sweep
preview cadence, scale, and JPEG quality (1–100); only the defaults exercise
full-size, every-frame preview. This changes the `Image.to_jpeg()` quality, not
the sensor's JPEG quality setting. After detector processing and the UART sink
write, the probe paints a 32-bit marker (`D3` magic, 16-bit frame sequence,
CRC-8/ATM) as 32 solid 8x8 cells at x=32..287, y=104..111. Its Viper writer
matches `tools/build_openmv_ram_preview_probe.py` and the host decoder in
`tools/openmv_preview_marker.py`. `Image.bytearray()` returns a by-reference
view of the image data for this write. The pinned `draw_string` wrapper also
accepts a 2-element `(x, y)` tuple; the text marker was replaced for host
decoder compatibility, not because that coordinate form is invalid. The
`Image.clear` mask parameter is keyword-only, and this probe makes no positional
`clear(mask)` call. The pinned `to_jpeg` implementation defaults to
`copy=False`, which may convert the source image in place. `copy=True` allocates
a separate image so the capture buffer remains available to the detector and
camera path. Marker writing and full-frame JPEG encode/flush add time and are
included in the probe's preview-stage cost.
In the pinned source, `Image.flush()` passes the compressed image to
`framebuffer_update_preview()`, which copies its bytes into the IDE stream
buffer; the dynamic JPEG does not need to outlive that call. The `to_jpeg`
wrapper, copy allocation, and flush behavior are in the pinned
[OpenMV v5.0.0 `py_image.c`](https://github.com/openmv/openmv/blob/v5.0.0/modules/py_image.c).
The exact [by-reference `Image.bytearray()`](https://github.com/openmv/openmv/blob/v5.0.0/modules/py_image.c#L580-L584),
[`Image.clear()` keyword-only mask](https://github.com/openmv/openmv/blob/v5.0.0/modules/py_image.c#L1142-L1164), and
[`Image.draw_string()` tuple position](https://github.com/openmv/openmv/blob/v5.0.0/modules/py_image.c#L1260-L1300)
signatures are in the same pinned source.

The probe reports board-side input, preview-publish, and marker-write counts
separately. PC validation decodes valid forward sequence transitions from the
binary marker. A repeated image hash is not evidence of a repeated capture, and
a board frame counter is not evidence that the IDE displayed every frame. The
first run on the patched firmware remains pending; no full-size, every-frame 60
FPS result is claimed.

## Build requirements

OpenMV v5.0.0 documents Docker or native Linux/macOS builds; it has no native
Windows build environment. For `OPENMV4P`, use a complete recursive v5.0.0
checkout and SDK/toolchain, then run from the repository root:

```sh
make sdk
make -j$(nproc) -C lib/micropython/mpy-cross
make -j$(nproc) TARGET=OPENMV4P
```

The Windows host has no supported complete native build environment. The patch
was instead compiled by the pinned Linux workflow in
`openmv4p-build-workflow.yml` using OpenMV SDK 1.6.0.
[CI run 40003611704](https://github.com/vaelgushilumolu75237/smart-car-racing-cartest-versions/actions/runs/40003611704)
succeeded. The reviewed `firmware.bin` and `firmware.elf`, hashes and build
manifest are under `deliverables/OpenMV_NativePreview_20261024/`.
The candidate has not been flashed or tested on the camera. Compilation is
not evidence of boot, preview throughput, or simultaneous 60+ FPS.
