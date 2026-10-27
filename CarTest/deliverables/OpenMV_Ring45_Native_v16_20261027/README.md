# OpenMV Ring45 native v16 app bundle

Status: bench-only; road acceptance is pending; motor_start=false.

## Device and image boundary

- Device sample: OpenMV4P target, USB DFU ID 37c5:924a from the recorded app-flash log. No serial number is asserted.
- The image is an application-only payload linked at 0x08040000. This bundle contains no bootloader and no ROMFS.
- Track RAM diagnostics and ring-history auto-save consume camera flash; keep computer archive/log directories on the PC and do not leave them on camera flash.
- firmware.bin is 1741964 bytes with SHA-256 BAA2B1DE63A2DB71416DA23321C1A628CFD634E3AEC20F16C508E793A156326C.
- Flash evidence log SHA-256: 6A5CC71C2A1953504EBEA5DD40839E601EF986FDBD39371F8036E37692154271. App-readback verification JSON SHA-256: 4A37D9BFF5765F9D9C22ED4D4A7B4C36099969E34AC84F8D386205CDAD1D4AB0. Neither file is included.
- Private bench evidence SHA-256: 937A98A57A2CE1DA8438A8EF300F1C7F1814B3E2FF5BEFDF4AB14A92A450F728; the bundle records only this hash and the explicitly curated public result: Static MT9V034 320x120 bench: headless 10s windows 84.0/82.7/82.7 FPS; real IDE native-zoom JPEG Q10 six 10s windows 61.3/61.0/61.1/61.3/61.4/61.5 FPS, screen marker CRC errors 0; 90s board report CRC verified; five SDK-stop recovery rounds encoded 500 fresh frames without errors; parked real UART97 FPS, outputs zero. Moving-track and ring entry/exit acceptance pending..
- Raw logs, camera images, vehicle RAM logs, and calibration data are not included.

## Timeout and recovery limits

- The pinned current-j rollback image is included by hash. It has a known repeated-IDE-Run JPEG preview timeout boundary; rollback does not claim to remove that boundary.
- The caller's 5 ms encode timeout is not a hard bound on HAL abort cleanup. The pinned HAL's nominal cleanup may reach about 1013 ms with a running tick; stopped ticks and failed peripheral aborts are outside that nominal estimate.
- These limits describe expected software cleanup behavior, not validated hardware recovery or continuous-run performance.

## Acceptance boundary

- This is a bench candidate only. Do not infer road acceptance or motor-start authorization from the package.
- A Windows Device Manager Code 38 or other host USB driver warning is not evidence of a firmware defect or proof that a firmware defect was fixed.
- Flash/readback references are evidence records only; the bundle does not contain the raw logs or private evidence payload.
