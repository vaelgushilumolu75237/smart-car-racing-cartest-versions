# OpenMV v5.0.0 optional snapshot preview control

This patch adds `preview=True` to both the legacy `sensor.snapshot()` wrapper
and the v5 `csi.CSI.snapshot()` method. The default remains enabled. Passing
`preview=False` sets the existing `OMV_CSI_FLAG_NO_UPDATE` bit, preventing
that capture from replacing the IDE stream image. Both Python modules export
`PREVIEW_CONTROL_API = 1`; the RAM probe requires both markers before camera
setup, so it cannot silently run against an older firmware that ignores the
legacy snapshot keyword. The caller can then publish a selected scaled JPEG
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

`openmv/tests/ram_usb_preview_probe.py` demonstrates the intended runtime
pattern using the production detector path, a RESET/KEEP UART sink, a full
320x120 grayscale capture for processing, and a scaled JPEG preview every
`PREVIEW_EVERY_N` frames (`SCALE=0.25` by default). This reduced-size,
decimated preview is a candidate A/B probe, not a full-size every-frame 60 FPS
acceptance test.

## Build requirements

OpenMV v5.0.0 documents Docker or native Linux/macOS builds; it has no native
Windows build environment. For `OPENMV4P`, use a complete recursive v5.0.0
checkout and SDK/toolchain, then run from the repository root:

```sh
make sdk
make -j$(nproc) -C lib/micropython/mpy-cross
make -j$(nproc) TARGET=OPENMV4P
```

The current Windows host has neither Docker nor `make`, and `wsl.exe` is
present without an installed Linux distribution. No firmware build or flash
has been performed for this patch.
