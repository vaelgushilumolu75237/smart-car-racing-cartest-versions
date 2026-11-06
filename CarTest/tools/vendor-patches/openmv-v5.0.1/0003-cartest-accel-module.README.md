# OpenMV v5.0.1 `cartest_accel` module patch

`0003-cartest-accel-module.patch` adds the reviewed pure-C topology core and MicroPython binding to the pinned OpenMV v5.0.1 tree. It follows 0001 preview control and 0002 STM32H7 UART FIFO write; 0004 then adds the path core.

The binding keeps `API_VERSION == 1`, the topology methods, and the optional `HEAP_STATS_API == 1` / `heap_stats()` interface. On builds where `OMV_JPEG_CODEC_ENABLE == 1`, it also exports `PREVIEW_CODEC_GUARD_API == 1` and `jpeg_encode_timeout_ms([value])`. The no-argument form returns the current timeout; the setter accepts only a non-boolean integer from 1 through 1000 ms, updates the OpenMV JPEG encoder timeout, and returns the previous value. Other codec builds omit the optional guard API.

`0003-cartest-accel-module.manifest.json` pins the patch SHA, base commits, and the three module source hashes. The 0004 source hash manifest pins the path module files. The 0005 manifest pins the JPEG guard overlay hashes and preserves the reviewed cleanup limits: the caller may request a 5 ms encode budget, but HAL abort cleanup has no 5 ms hard bound and may take about 1.013 seconds in the nominal running-tick case.

## Reproducible clean app build

`build-openmv4p-ring45-native-clean.sh` checks an already-patched tree at the pinned commits, reverse-checks all five patches, validates C module and JPEG overlay hashes, fingerprints the 23 runtime Python files, and builds the OPENMV4P application into a new empty build directory. It requires no old `.o`/`.P` files. The script does not apply patches, download files, flash a camera, or access hardware. It builds with `OMV_ENABLE_BL=0` and the `firmware` target, then writes `firmware.bin`, `firmware.elf`, generated QSTRs, build logs, exit codes, and `build-verification.json` to the requested artifact directory.

For the Git Bash build environment used by the successful `l` build, set:

```sh
export PATH=/d/OpenMV_IDE/share/qtcreator/arm/bin:/r/openmv-build-tools/bin:/usr/bin:$PATH
export MAKE=make
export PYTHON=/r/openmv-build-tools/bin/python3
export MICROPY_MPYCROSS=/r/mpy-cross-1.28.0/mpy-cross.exe
export SDK_DIR=/r/openmv-sdk-version-check-only
```

The script defaults `MAKE` to `make`, `JOBS` to 4, and the Python, mpy-cross, and SDK marker paths above. The board makefile supplies `PORT=stm32`; the runner does not override it. Check syntax and preconditions without building:

```sh
script=tools/vendor-patches/openmv-v5.0.1/build-openmv4p-ring45-native-clean.sh
bash -n "$script"
"$script" --verify-only \
  /q \
  /q/ring45-native-verify \
  /d/Codex/smart-car-racing-codex-handoff/smart-car-racing/firmware/vehicle/CarTest/tmp/ring45-native-verify-artifacts \
  /d/Codex/smart-car-racing-codex-handoff/smart-car-racing/firmware/vehicle/CarTest \
  /d/Codex/smart-car-racing-codex-handoff/smart-car-racing/firmware/vehicle/CarTest/logs/ring45-usb60-20261026/pipeline_perf_v16_codec_guard
```

Use a new build directory and a new artifact directory for a full build. The runner refuses to overwrite an existing artifact directory and refuses any build directory that already contains files.

The successful software build recorded on 2026-10-27 used a task-private runner, not the new script. Its exact make invocation was:

```sh
make --no-print-directory -j4 -C /q \
  TARGET=OPENMV4P BUILD=/q/l OMV_ENABLE_BL=0 \
  SDK_DIR=/r/openmv-sdk-version-check-only \
  MICROPY_MPYCROSS=/r/mpy-cross-1.28.0/mpy-cross.exe \
  PYTHON=/r/openmv-build-tools/bin/python3 firmware
```

That build began with an empty `/q/l`, produced 787 objects and 531 dependency files, and passed the 15-name QSTR and app-vector checks. Its evidence is `logs/ring45-usb60-20261026/pipeline_perf_v16_codec_guard/build_artifacts_l_final4/build-verification.json`; `firmware.bin` is 1,741,964 bytes with SHA-256 `BAA2B1DE63A2DB71416DA23321C1A628CFD634E3AEC20F16C508E793A156326C`. The new reusable script has passed syntax/precondition checks only; it has not been used for another full build.

## Separate hardware evidence and limits

The build runner reports `hardware_touched=false` and is not hardware validation. Root's separate 2026-10-27 board evidence for the `l` image records a matching full-region readback, passing API smoke test, five interrupt-recovery rounds followed by 500 new frames, and headless windows of 84.0 / 82.7 / 82.7. The first valid IDE measurement was 33.5 seconds at 61.241 fps, with three windows at 61.3 fps each. The 90-second IDE run was still in progress when this note was written.

This source tree identifies the target as vendor `OPENMV4P` / `STM32H743xx`; the tested sample is the user's H750-marked board. These results do not establish generic H750 compatibility. A successful runner build proves compilation for the vendor OPENMV4P tree and the selected patched source state only.
