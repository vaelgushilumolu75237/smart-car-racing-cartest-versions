# OpenMV JPEG preview guard patch (private review copy)

Pinned input: `tmp/openmv-v5.0.1-h750-audit/ports/stm32/stm_jpeg.c`.
The source SHA-256 at preparation time was
`83D5FCA6AEC29CB339104AE3FBF3BBB2F4A960B4C60AB1E0922491DE1988C548`.
`overlay/` is the review copy; `jpeg_preview_guard.patch` is generated from it
and the pinned tree by `python make_patch.py`. No pinned or production source is
modified here.

## Changes

- Add `jpeg_get_encode_timeout_ms()` and `jpeg_set_encode_timeout_ms()` in
  `imlib.h`/`stm_jpeg.c`. The default is 1000 ms; setters accept 1 through 1000
  and reject other values without changing the current setting. The encoder
  (including the optional VC8000 path) uses this value. Decode waits continue to
  use `JPEG_CODEC_TIMEOUT`.
- Catch MicroPython NLR exceptions around each encoder path, release resources,
  and rethrow the original `nlr.ret_val`. The STM32 JPEG path aborts, unbinds
  callbacks, and releases temporary/owned buffers only after HAL confirms stop.
- If HAL cannot confirm stop, unbind the callbacks, quarantine owned buffers,
  and block later STM32 JPEG encodes instead of freeing DMA-visible memory. This
  fail-closed branch is not a proven hardware recovery path.
- On the normal `HAL_OK` abort path, the pinned HAL returns the JPEG handle to
  `READY`; later frames reuse that initialized handle. The patch does not call
  JPEG init/deinit per frame. If HAL abort reports an error, this patch does not
  claim automatic recovery; the sticky encode block remains until MCU reset.
- No encode algorithm or decode code is changed. No MicroPython binding is
  added; the C API is exposed for lead review.

## HAL bound found in pinned sources

`stm32h7xx_hal_jpeg.c` sets `JPEG_TIMEOUT_VALUE` to 1000 ms. In DMA context,
`HAL_JPEG_Abort()` synchronously attempts to abort output then input MDMA, then
waits for the JPEG COF flag. `stm32h7xx_hal_mdma.c` sets each
`HAL_TIMEOUT_MDMA_ABORT` to 5 ms and its loop exits when elapsed time is greater
than that value. With a running HAL tick, the nominal worst-case is about
`2 * 6 ms + 1001 ms`, roughly 1.013 seconds. A shorter encode timeout therefore
does not bound cleanup latency. A stopped HAL tick or a failed peripheral abort
falls outside this bound.

## Verification

- `tests/test_source_contract.py` checks source wiring, API bounds/default, and
  that decode waits retain the fixed timeout.
- `tests/jpeg_preview_guard_cleanup_test.c` mocks HAL abort/unregister and the
  allocator release action. It checks `abort -> unbind -> free` on success and
  verifies that buffers are not freed when abort reports failure. It does not
  model DMA registers, HAL timing, MicroPython NLR internals, or hardware.
- The C mock was built and run with Visual Studio 2026 C compiler.
- The patched H750 translation unit was emitted as an ARMv7E-M relocatable
  object with Keil MDK-ARM Lite 5.24 / ARM Compiler 6.7 (`cortex-m7`, Thumb,
  FPv5-SP-D16, hard-float). Private compile-only `sys/types.h` and `stdatomic.h`
  shims were needed because that compiler installation lacks the newlib headers
  used transitively by OpenMV. This is an object compile, not the pinned GCC
  firmware build, link, flash, or hardware validation.
- The optional VC8000 conditional path was also emitted as an ARM relocatable
  object using `venc_compile_shim.h` for peripheral symbols absent on this
  H750 target. This checks the conditional C path, not a real VENC platform.
- Generated validation outputs are archived outside the public tools tree at
  `logs/ring45-usb60-20261026/jpeg-guard-validation/build/` (the mock EXE,
  ARM objects, and compile-only include shims). The Python test bytecode cache
  is at `logs/ring45-usb60-20261026/jpeg-guard-validation/python-tests-cache/`.

Run the source test from the CarTest root:

```powershell
python -m unittest discover -s tools/vendor-patches/openmv-v5.0.1/jpeg_preview_guard/tests -p 'test_*.py' -v
```
