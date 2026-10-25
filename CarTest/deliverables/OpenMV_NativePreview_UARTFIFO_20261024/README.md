# OpenMV UART FIFO firmware delivery

This directory contains the prebuilt OpenMV v5.0.0 `OPENMV4P` firmware outputs, their exact build summary and checksum, and review metadata for this binary-only delivery. It does not contain an OpenMV application source package.

The GitHub Actions build completed successfully in run 40095756045 at repository commit `a44ba5186711e37cde4a65c19b6b3dd9780e75af`. It pins OpenMV commit `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`, MicroPython commit `d18a77e5d55664045e71a0c839695df07a96a6e1`, and UART FIFO patch SHA-256 `8d1649cc9fb74e00b6d1234f6c8a3a62d70b90ebd605053f069b202ec36890fb`.

`firmware.bin` is 1,736,932 bytes (SHA-256 `0bbf81a534f99365a62187b3116559e313527e0aa1b7d585e618022b11feffc3`). `firmware.elf` is 11,330,088 bytes (SHA-256 `aae890556a1f35fe784ec451c064e4e24d1710ef91cff9f3f051117f1c18cde6`). The binary vector reset and ELF entry both resolve to `0x08040299`.

A reported 65-second IDE run measured complete payload reads across six windows. Board-side rates were 60.6-60.8 FPS; conservative IDE payload-read rates were 60.1153-60.5143 Hz. All 3,986 advertised/read size records matched; these are matched size records, not a verified count of unique frames. The FIFO accepted 3,949 frames. These records do not establish independent JPEG decoding per frame or physical display refresh. See `EVIDENCE.md` for scope and remaining acceptance items.

A separate 65-second native headless run used the same camera `main.py` revision with intentionally invalid fixed-sequence UART frames. CDC was closed for 68 seconds; the lead reports the physical USB cable remained connected. Six board windows ranged from 79.2 to 79.4 FPS; the FIFO accepted 5,156 frames with no busy, partial, or error results. This is static invalid-only transport evidence, not a physical USB-disconnect, route-control, road, or end-to-end acceptance test.

The checkpoint exporter includes these four build files and this README, `EVIDENCE.md`, and `artifact-manifest.json` only when invoked with `--include-openmv-uart-fifo-firmware`.
