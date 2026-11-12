# OpenMV FIFO reply fix

API 1 write_fifo accepts only 1..16 bytes. The shared writer previously passed 32-byte HELLO replies, 40-byte ACKs and larger STATE/CONFIG packets to it. These raised ValueError and were dropped while 16-byte vision still worked.

Long frames now use one ordinary UART.write after the existing idle check. Short vision retains FIFO. No fragmentation or interleaving is introduced. Long writes are synchronous; hardware frame rate and latency remain unmeasured.

Full camera regression and strict FIFO sideband tests passed. H743 is unchanged; paired ELFs retain prior build evidence. Install this package OpenMV/main.py into the active camera directory, preserving calibration. No H743 reflash is needed if the matching entry version is installed. No hardware upload/reset was performed.
