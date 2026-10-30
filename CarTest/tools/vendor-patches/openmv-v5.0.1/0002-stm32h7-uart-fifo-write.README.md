# OpenMV v5.0.1 STM32H7 UART TX FIFO patch

This is series patch 2 for the pinned OpenMV v5.0.1 source commit `631681e5ac5ab332c920b6bf9cd3dba5cf299eac` and MicroPython commit `dd984559aa42841908b78397b7dac32f57da803f`. It follows `0001-snapshot-preview-control.patch`; the manifest pins both patch hashes and the unmodified UART source-file hashes.

The v5.0.0 FIFO implementation applies to these v5.0.1 sources without code-context changes. The four UART base files have matching SHA-256 values across the pinned commits, so the patch body remains byte-identical to the previously reviewed v5.0.0 patch. This is a v5.0.1-specific manifest/series entry, not a claim that firmware built for another MCU is compatible.

On STM32H7 only, the patch exposes `UART.WRITE_FIFO_API == 1`, `write_fifo(data)` for a bounded 1..16 byte nonblocking FIFO burst, and `fifo_txdone()`. Existing `write()` and `txdone()` behavior is unchanged. It checks UART mode and pending transfer state before writing and restores the prior interrupt state on every path.

## Validation

The UART C harness and the requested host test suite are recorded with the build evidence. The OPENMV4P build is source/build-only; this script does not access a camera, USB DFU, or flash. Parent agent owns all device testing and firmware deployment decisions.
