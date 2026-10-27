# OpenMV v5.0.1 snapshot preview control candidate

This patch adds `preview=False` to `csi.CSI.snapshot()` and forwards that
keyword through the v5.0.1 compatibility `sensor.snapshot()` wrapper. The
default remains enabled. `preview=False` passes the existing
`OMV_CSI_FLAG_NO_UPDATE` flag to capture, so the capture is not published as
the automatic IDE frame. `Image.flush()` remains the publication operation for
the processed frame's JPEG. The sensor capture remains grayscale and the
Python-side call remains synchronous; this patch adds no cache, frame replay,
queue, or second capture.

## Pinned upstream source

- Repository: `https://github.com/openmv/openmv`
- Tag: `v5.0.1`
- Commit: `631681e5ac5ab332c920b6bf9cd3dba5cf299eac`
- MicroPython submodule: `dd984559aa42841908b78397b7dac32f57da803f`
- SDK version: `1.6.0`
- Patch SHA-256: `E3A93287D75298664E68068547F9A1F68D15DB0422F0BC08DCC1E7D4D6ADF193`
- Base hashes for the changed files and target configuration are in
  `manifest.json`.

## Board target and observed camera

This source tag contains `OPENMV4P`, but no `H750` target. Its
`boards/OPENMV4P/board_config.mk` specifies `STM32H743xx` / `STM32H743VI` and
2 MiB of internal flash. The board config places 32 MiB SDRAM at
`0xC0000000`; `boot_config.h` declares a 32 MiB QSPI device with a raw
`0x00000000..0x02000000` partition and ROMFS at
`0x01800000..0x02000000`. These are still the upstream H743 build settings;
the patch does not add a generic H750 build target.

Read-only inspection of the current H750-reported camera found that it already
runs the byte-identical IDE `OPENMV4P` firmware at `0x08040000`. It reports the
`OPENMV4P` runtime and enumerates the matching `37C5:924A` OpenMV DFU bootloader.
The verified alt map is `BOOTLOADER`, `FILESYSTEM`, `FIRMWARE`, `SPI_FLASH`,
`ROMFS0`; the alt-0 DFU upload matched the internal boot readback and returned
the expected four-zero EOF packet. The H750 probe reports 128 KiB in
`FLASHSIZE`; the active vendor app, VTOR, 2 MiB internal read window, and
32 MiB QSPI controller map establish the currently used vendor layout on this
sample. This supports classifying the build as an **OPENMV4P layout
compatibility candidate for this observed H750 sample**, with deployment
limited to app alt 2 after the original app and external-flash backups are
verified. The candidate has now been transferred to alt 2; the camera reported
both preview API markers and the expected clock tuple after reboot. This is
initial boot/API evidence only: patched frame processing, full app readback,
and visible preview performance have not yet been confirmed. It is not a
generic H750 target.

The patch changes only the Python binding and compatibility wrapper. It does
not alter the board config, bootloader, linker script, QSPI driver, USB stack,
framebuffer sizing, or camera sensor driver. A successful build proves
compilation for upstream `OPENMV4P`. The exact stock OPENMV4P app already runs
on the observed sample, but that does not prove the patched app boots there or
that IDE frames are visible at 60 FPS.

## Patch and build checks

From a clean recursive checkout at the pinned commit, apply with:

```sh
git apply --check tools/vendor-patches/openmv-v5.0.1/0001-snapshot-preview-control.patch
git apply tools/vendor-patches/openmv-v5.0.1/0001-snapshot-preview-control.patch
```

On the repository's supported Linux build environment, run:

```sh
bash tools/vendor-patches/openmv-v5.0.1/build-openmv4p.sh /path/to/openmv-v5.0.1
```

The build script verifies the source commit and patch hash, checks the
OPENMV4P board files, applies the patch if needed, installs the pinned OpenMV
SDK, builds the target, and copies `firmware.bin`, `firmware.elf`, and a hash
manifest under the project `tmp/` directory. It never flashes a camera.

Do not treat this sample's matching vendor layout as validation for another
H750 camera or firmware image. The app-only DFU and readback procedure is in
`DFU_READBACK.md`; it leaves the protected bootloader and ROMFS unchanged.
