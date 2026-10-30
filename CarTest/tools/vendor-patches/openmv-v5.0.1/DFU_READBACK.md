# OPENMV4P USB DFU readback and app-only update

The observed camera currently runs the byte-identical IDE `OPENMV4P`
application and enters the matching OpenMV DFU bootloader (`37C5:924A`). The
firmware patch is compiled into the application: `boards/OPENMV4P/manifest.py`
freezes `sensor.py`, and the linked map contains `sensor_snapshot` and
`sensor_flush` frozen functions. The OpenMV bootloader DFU partition list
advertises uploads and downloads. This supports an app-only candidate update;
it does not require replacing the bootloader or ROMFS.

## Read-only partition backups

The OpenMV IDE 5.1.1 bundled Windows tool is:

```powershell
$dfu = 'D:\OpenMV_IDE\share\qtcreator\dfu-util\windows\dfu-util.exe'
```

List attached DFU devices and alt settings first:

```powershell
& $dfu --list
```

For the observed H7 Plus layout, the alt settings map as follows:

| Alt | Partition | Range | Size | Access |
| --- | --- | --- | ---: | --- |
| 0 | `BOOTLOADER` | `0x08000000..0x0801FFFF` | 131,072 B | read-only |
| 1 | `FILESYSTEM` | `0x08020000..0x0803FFFF` | 131,072 B | read/write |
| 2 | `FIRMWARE` | `0x08040000..0x081FFFFF` | 1,835,008 B | read/write |
| 3 | `SPI_FLASH` | `0x00000000..0x01FFFFFF` | 33,554,432 B | read/write |
| 4 | `ROMFS0` | `0x01800000..0x01FFFFFF` | 8,388,608 B | read/write |

These are custom OpenMV alt settings; do not add an absolute address with
`-s`. The DFU source marks alt 0 read-only and implements upload/download on
the others.

```powershell
& $dfu -w -d ',37C5:924A' -a 2 -Z 1835008 -U .\firmware-alt2-before.raw
& $dfu -w -d ',37C5:924A' -a 4 -Z 8388608 -U .\romfs-alt4-before.raw
```

If a complete external flash backup is required, alt 3 covers 33,554,432
bytes. Stream it to a host file; do not save it to the camera's own
filesystem.

```powershell
& $dfu -w -d ',37C5:924A' -a 3 -Z 33554432 -U .\spi-alt3-before.raw
```

This bootloader returns a four-byte zero short packet after the partition's
last byte. The observed 128 KiB boot partition upload was exactly 131,072
bytes of flash followed by `00 00 00 00` (131,076 bytes total). The verifier
checks that framing before trimming the sentinel for hashes/comparisons:

```powershell
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\firmware-alt2-before.raw --partition-size 0x1c0000
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\firmware-alt2-before.raw --partition-size 0x1c0000 `
  --write-payload .\firmware-alt2-stock.bin
```

## Firmware update and readback

After saving the original app and the filesystem data needed for rollback,
write only the patched `firmware.bin` to alt 2. The final `--reset` leaves DFU
and starts the application. The bootloader (alt 0) and external ROMFS (alt 4)
are not part of this command.

```powershell
& $dfu -w -d ',37C5:924A' -a 2 --reset -D .\firmware.bin
```

Re-enter DFU and read the full firmware partition back. Compare the partition
prefix against the exact candidate image:

```powershell
& $dfu -w -d ',37C5:924A' -a 2 -Z 1835008 -U .\firmware-alt2-after.raw
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\firmware-alt2-after.raw --partition-size 0x1c0000 `
  --expected-prefix .\firmware.bin
```

Re-read alt 0 and alt 4 after the update to confirm the bootloader and ROMFS
are unchanged. The already saved full alt-3 image can also be compared if an
external-flash preservation check is desired.

```powershell
& $dfu -w -d ',37C5:924A' -a 0 -Z 131072 -U .\bootloader-alt0-after.raw
& $dfu -w -d ',37C5:924A' -a 4 -Z 8388608 -U .\romfs-alt4-after.raw
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\bootloader-alt0-after.raw --partition-size 0x20000 `
  --compare-upload .\bootloader-alt0-before.raw
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\romfs-alt4-after.raw --partition-size 0x800000 `
  --compare-upload .\romfs-alt4-before.raw
```

After a read-only upload, return from DFU to the application without writing
or resetting the application partition by sending DFU_DETACH:

```powershell
$serial = '<serial shown by dfu-util --list>'
& $dfu -w -d ',37C5:924A' -S $serial -e
```

Pin `-S` to the intended camera if more than one DFU device is attached; without
it, `dfu-util` can reject the detach as ambiguous.

This path is source-verified for this OpenMV bootloader: `tud_dfu_detach_cb()`
sets `DFU_STATE_RESET`; the boot main loop then disconnects/deinitializes USB
and jumps to the application at `OMV_BOOT_JUMP`. `-R` is a USB bus reset and
is not the documented no-write exit path here.

After restoring the original app, read back alt 2 again and compare the full
partition payload to the saved pre-update upload:

```powershell
& $dfu -w -d ',37C5:924A' -a 2 --reset -D .\firmware-alt2-stock.bin
& $dfu -w -d ',37C5:924A' -a 2 -Z 1835008 -U .\firmware-alt2-restored.raw
python tools/vendor-patches/openmv-v5.0.1/verify_dfu_upload.py `
  --upload .\firmware-alt2-restored.raw --partition-size 0x1c0000 `
  --compare-upload .\firmware-alt2-before.raw
```

`firmware-alt2-stock.bin` is the exact original alt-2 partition payload
without the four-byte EOF sentinel. The verifier creates it only after
checking the raw upload size and sentinel.

The current build artifact remains an upstream OPENMV4P/H743 target image,
and has now been transferred to alt 2 on the observed sample. The candidate
reports its preview API marker and expected clocks after reboot. Full app
readback comparison, camera-loop processing, and visible IDE frame-rate
measurement are still pending; the stock layout match does not establish
those results.
