# OpenMV USB60 candidate: UART timing audit (2026-10-24)

## Scope and source version

This is a read-only audit of the pinned OpenMV 5.0.0 STM32 MicroPython UART implementation and the CarTest V2 sender. It does not change UART settings, packet format, production firmware, or hardware state.

The local OpenMV firmware checkout is tag `v5.0.0`, commit `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`; its MicroPython submodule is commit `d18a77e5d55664045e71a0c839695df07a96a6e1`. The source links below pin that submodule commit.

## What the current path does

The CarTest OpenMV configuration uses UART3 at 115200 baud, 8 data bits, no parity, and one stop bit (`openmv/config.py:7-11`). The V2 frame is 16 bytes (`openmv/vision_protocol.py:3`). In `_main_fast`, the sender calls `uart.write(packet)`, labels the result `FULL_16` when the returned count equals the packet length, and records a lower count as `SHORT_WRITE`; `pack_uart_us` includes the call (`openmv/main.py:5361-5376`).

At 8N1, each byte takes 10 wire bits. A 16-byte packet therefore has a **computed** serialization floor of `16 * 10 / 115200 = 1.389 ms`. This is a line-time calculation, not a board measurement. At 60 packets per second, the stream carries `16 * 10 * 60 = 9,600 bit/s`, or `9,600 / 115,200 = 8.33%` of the configured link rate. This does not indicate a baud-capacity limit. The relevant loop cost is the synchronous TX busy-wait: at a 60 Hz target, the 1.389 ms wire interval is about 8.33% of the 16.67 ms frame period. This is a calculated lower-bound budget, not a measured pipeline cost.

Pinned-source behavior:

- The UART constructor accepts baud, bit, parity, stop, flow, timeout, `rxbuf`, and `read_buf_len` arguments; it has no `txbuf` argument. See [`machine_uart.c`, initialization arguments](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/machine_uart.c#L151-L187).
- `uart.write()` calls the STM32 TX routine synchronously. The routine polls TX-empty for each byte, writes the data register, then waits for the UART transmission-complete flag before returning. The flag wait is a tight register-polling loop; the source explicitly says it does not sleep because UART TX does not generate an interrupt. See [`machine_uart.c`, write implementation](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/machine_uart.c#L516-L539), [`uart.c`, TX loop and completion wait](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/uart.c#L1141-L1193), and [`uart.c`, TX wait loop](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/uart.c#L1178-L1196).
- The Python UART IRQ mask allows RX-idle and RX-data events only. It excludes TX-empty and TX-complete, and the UART object stores an RX ring buffer but no software TX ring. See [`uart.h`, allowed flags and UART object](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/uart.h#L52-L78) and [`machine_uart.c`, IRQ trigger validation](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/machine_uart.c#L450-L475).
- `txdone()` always returns true because `write()` waits for the last byte; `flush()` is correspondingly a no-op. These are not asynchronous completion hooks. See [`machine_uart.c`, `txdone()` and stream flush](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/machine_uart.c#L412-L415) and [flush handling](https://github.com/openmv/micropython/blob/d18a77e5d55664045e71a0c839695df07a96a6e1/ports/stm32/machine_uart.c#L541-L560).

The driver returns a partial byte count if it times out while waiting to load later bytes. If it times out while waiting for the final transmission-complete flag after loading all bytes, it returns the full count; the Python wrapper also maps that timeout to a full-length return. Therefore `FULL_16` means the API returned 16, not that the H743 received a valid frame. In that final-timeout case it also does not prove that the final UART frame had completed before `write()` returned.

## Consequence for preview benchmarks

The RAM UART sink used by the USB preview experiments returns `len(packet)` immediately. It does not model the pinned driver's TX polling and completion wait, the 1.389 ms calculated wire floor, or H743 reception. Consequently the reported 58.5 FPS sink run is not evidence of real-UART throughput. No real-UART full-pipeline rate is asserted here.

## Next measurement when the camera can be recovered

Keep the existing UART3/115200/8N1 configuration and V2 packet unchanged for the first comparison. With the paired V2 H743 receiver reviewed and validated before any flash, measure a non-driving run with:

1. OpenMV fresh-capture count and independent full-cycle timing over one-second windows.
2. The camera's actual `pack_uart_us` distribution, which wraps `uart.write(packet)`.
3. H743 accepted, CRC-valid V2 sequence count in matching one-second windows, including sequence gaps or rejects.
4. USB preview open and closed as separate conditions; when open, count unique host frame markers independently of the camera capture counter.

No motors are required for this measurement. It remains pending physical camera recovery and paired-receiver review. A sink-only result, `UART.write()` return count, or preview FPS by itself is not a receiver-throughput result.

If measurements later show UART serialization is the limiting stage, an asynchronous native API would be a separate firmware design requiring a persistent TX buffer and explicit accepted/completed/error semantics. It is not implemented or promised by this audit; do not infer support from the current `txdone()`, `flush()`, or Python UART IRQ API.
