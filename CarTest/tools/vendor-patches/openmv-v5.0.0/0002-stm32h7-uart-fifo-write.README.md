# OpenMV v5.0.0 STM32H7 UART TX FIFO patch

`0002-stm32h7-uart-fifo-write.patch` is the second patch in the pinned OpenMV v5.0.0 series. Its separate manifest pins the patch SHA-256, upstream OpenMV commit, MicroPython submodule commit, predecessor patch, and audited base-file hashes. The existing `0001` manifest and patch remain independent and unchanged.

The patch exposes `UART.WRITE_FIFO_API == 1` on STM32H7 and adds bounded nonblocking FIFO writes of up to 16 bytes plus a FIFO transmit-complete query. Its implementation checks UART/FIFO mode and interrupt/DMA constraints, restores the prior IRQ state on every return path, and leaves ordinary blocking `write()` and `txdone()` behavior unchanged.

## Validation status

The pinned CI firmware build succeeded in run 40095756045 at source commit `a44ba5186711e37cde4a65c19b6b3dd9780e75af`. In the earlier candidate test run, 10 Python tests passed and 2 errored because the temporary source checkout lacked Git metadata; the separate C harness passed 5 groups. This is not a claim that the entire native test suite passed. The build result and static invalid-only board evidence are recorded in the firmware delivery's `EVIDENCE.md`; valid route control, the newer IDE test, and road acceptance remain pending.