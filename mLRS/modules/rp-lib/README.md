# RP-Lib Code Generation

This directory contains template-based code generation for RP2350 peripherals.

## UART Generation

**Template:** `rp-uart-template.h`
**Generator:** `rp-uart-generate.py`

### Generated Files
- `rp-uart.h` (UART)
- `rp-uartb.h` (UARTB)
- `rp-uartc.h` (UARTC)
- `rp-uartd.h` (UARTD)
- `rp-uartf.h` (UARTF)

### Usage
```bash
cd mLRS/modules/rp-lib
python3 rp-uart-generate.py
```

### Template Substitution
- `UART$` → `UART`, `UARTB`, ..., `UARTF` (uppercase defines)
- `uart$` → `uart`, `uartb`, ..., `uartf` (lowercase functions)

### Supported Serial Types
- **HW UART** — `UART$_USE_SERIAL1` (uart0), `UART$_USE_SERIAL2` (uart1)
- **USB Serial** — `UART$_USE_SERIAL` (native USB CDC)

---

## SPI Generation

**Template:** `rp-spi-template.h`
**Generator:** `rp-spi-generate.py`

### Generated Files
- `rp-spi.h` (SPI)
- `rp-spib.h` (SPIB)

### Usage
```bash
cd mLRS/modules/rp-lib
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
