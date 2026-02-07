# RP-Lib Code Generation

This directory contains template-based code generation for RP2040/RP2350 peripherals.

## UART Generation

**Template:** `rp-uart-template.h`
**Generator:** `rp-uart-generate.py`

### Generated Files
- `rp-uart.h` (UART)
- `rp-uartb.h` (UARTB)
- `rp-uartc.h` (UARTC)
- `rp-uartd.h` (UARTD)
- `rp-uarte.h` (UARTE)
- `rp-uartf.h` (UARTF)

### Usage
```bash
cd mLRS/Common/rp-lib
python3 rp-uart-generate.py
```

### Template Substitution
- `UART$` → `UART`, `UARTB`, ..., `UARTF` (uppercase defines)
- `uart$` → `uart`, `uartb`, ..., `uartf` (lowercase functions)

### Supported Serial Types
- **HW UART** — `UART$_USE_SERIAL1` (uart0), `UART$_USE_SERIAL2` (uart1)
- **USB Serial** — `UART$_USE_SERIAL` (native USB CDC)

### Removed: PIO Serial (2026-02-07)

PIO-based UART support (`UART$_USE_SERIALPIO1`, `UART$_USE_SERIALPIO2`) was removed
from the template. It used PIO0 with IRQ-driven software ring buffers and inline
TX/RX PIO programs. **8N1 only** — parity and stop bit parameters were ignored.

#### Defines

Two PIO serial variants were supported, sharing PIO0 but using separate IRQ lines:

- `UART$_USE_SERIALPIO1` → `PIO0_IRQ_0`, `pio_set_irq0_source_enabled`
- `UART$_USE_SERIALPIO2` → `PIO0_IRQ_1`, `pio_set_irq1_source_enabled`

Each defined `UART$_PIO_INST`, `UART$_PIO_IRQ`, and `UART$_PIO_SET_IRQ_SOURCE_ENABLED`.
Signal inversion was supported via `UART$_INVERT_TX` / `UART$_INVERT_RX` using
`gpio_set_outover` / `gpio_set_inover` with `GPIO_OVERRIDE_INVERT`.

Required SDK headers: `hardware/pio.h`, `hardware/irq.h`, `hardware/gpio.h`,
`hardware/clocks.h`.

#### PIO Programs

Both programs run at **8 PIO cycles per bit**. Clock divider:
`div = clock_get_hz(clk_sys) / (8.0f * baud)`

**TX program** (5 instructions, sideset-based, from pico-examples `uart_tx`):
```
0x9fa0  // 0: pull block side 1 [7]  - stall with line idle high
0xf727  // 1: set x, 7 side 0 [7]   - start bit (low)
0x6001  // 2: out pins, 1            - output data bit
0x0642  // 3: jmp x-- 2 [6]          - loop for 8 bits
0xf767  // 4: nop side 1 [7]         - stop bit (high), wraps to 0
```
SM config: `out_shift` right, no autopull, 32-bit. `sideset` 2 bits (1 bit + optional).
`fifo_join` TX. Out pin and sideset pin both set to `UART$_TX_PIN`. Pin direction output.

**RX program** (4 instructions, from pico-examples `uart_rx`):
```
0x2020  // 0: wait 0 pin 0           - wait for start bit (falling edge)
0xea27  // 1: set x, 7 [10]          - preload bit counter, delay to mid-bit
0x4001  // 2: in pins, 1             - sample data bit
0x0642  // 3: jmp x-- 2 [6]          - loop for 8 bits, wraps to 0
```
SM config: `in_shift` right, autopush at 8 bits. `fifo_join` RX. In pin set to
`UART$_RX_PIN`. Pin direction input with pull-up (`gpio_set_pulls(pin, true, false)`).
Data arrives in **upper 8 bits** of the 32-bit FIFO word (due to autopush at 8 bits
with right shift) — extract with `(raw >> 24) & 0xFF`.

#### IRQ Handler

Single shared handler per PIO instance, placed in RAM via `__not_in_flash_func`.
Handles both RX and TX in one ISR:

1. **RX**: drain PIO RX FIFO → software ring buffer. Extract byte from upper 8 bits
   of each 32-bit FIFO word. Drop bytes if SW buffer full.
2. **TX**: drain software TX ring buffer → PIO TX FIFO. Write raw byte cast to
   `uint32_t` (PIO program handles bit shifting).
3. **TX IRQ disable**: when SW TX buffer is empty, disable the TX FIFO not-full IRQ
   source to prevent endless interrupts. Uses `UART$_PIO_SET_IRQ_SOURCE_ENABLED`
   with source `(pis_sm0_tx_fifo_not_full + sm_tx)`.

RX IRQ source (`pis_sm0_rx_fifo_not_empty + sm_rx`) was enabled once during init
and left on permanently.

#### Init Sequence (`_uart$_initit`)

1. Zero software buffer write/read positions
2. Calculate clock divider from `clk_sys` and baud rate
3. **TX setup** (if `UART$_TX_PIN >= 0`):
   - `pio_claim_unused_sm` to dynamically claim a state machine
   - `pio_add_program` to load the TX program
   - Configure SM: wrap, out_shift (right, no autopull, 32), out_pins, sideset_pins,
     sideset (2, optional, no pindirs), fifo_join TX, clkdiv
   - `pio_gpio_init`, set pin direction output, apply inversion if defined
   - `pio_sm_init` + `pio_sm_set_enabled`
4. **RX setup** (if `UART$_RX_PIN >= 0`):
   - `pio_claim_unused_sm` to dynamically claim a state machine
   - `pio_add_program` to load the RX program
   - Configure SM: wrap, in_pins, in_shift (right, autopush, 8), fifo_join RX, clkdiv
   - `pio_gpio_init`, set pin direction input, pull-up, apply inversion if defined
   - `pio_sm_init` + `pio_sm_set_enabled`
   - Enable RX FIFO not-empty IRQ source
5. `irq_set_exclusive_handler` + `irq_set_enabled` for the PIO IRQ

#### TX Routines

- `uart$_putc`: write byte to SW ring buffer, enable TX FIFO not-full IRQ source
- `uart$_putbuf`: loop writing to SW buffer, spin-wait if full (enabling TX IRQ
  to drain), then enable TX IRQ after filling
- `uart$_tx_flush`: spin until SW buffer empty, then spin until PIO TX FIFO empty
  (`pio_sm_is_tx_fifo_empty`)

No direct-to-FIFO fast path (unlike HW UART) — all bytes went through the SW buffer.

#### Baud Rate Change

`uart$_setbaudrate`: recalculate divider, call `pio_sm_set_clkdiv` on both TX and
RX state machines (guarded by pin validity checks).

---

## SPI Generation

**Template:** `rp-spi-template.h`
**Generator:** `rp-spi-generate.py`

### Generated Files
- `rp-spi.h` (SPI)
- `rp-spib.h` (SPIB)

### Usage
```bash
cd mLRS/Common/rp-lib
python3 rp-spi-generate.py
```

### Template Substitution
- `SPI$` → `SPI`, `SPIB` (uppercase defines)
- `spi$` → `spi`, `spib` (lowercase functions)

---

## Important Notes

1. **Edit the template, not the generated files** — Changes to generated files will be overwritten
2. **Run the generator after template changes** — Always regenerate all files after modifying a template
3. **Verify builds for all targets** — Template changes affect multiple peripherals
