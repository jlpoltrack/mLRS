# RP2040/RP2350 Targets

*2026-02-06*

## Overview

mLRS supports four RP-based targets covering both receiver (RX) and transmitter (TX) roles on the RP2040 (Pi Pico) and RP2350 (Pi Pico 2 W) platforms.

All targets use 2.4 GHz SX128x radios with the E28-2G4M27SX PA module.

## Target Summary

- **rx-generic-2400-rp2040** — RX on Pi Pico (RP2040), board `rpipico`
- **rx-generic-2400-rp2350** — RX on Pi Pico 2 W (RP2350), board `rpipico2w`
- **tx-generic-2400-rp2040** — TX on Pi Pico (RP2040), board `rpipico`, with I2C display + 5-way switch
- **tx-generic-2400-rp2350** — TX on Pi Pico 2 W (RP2350), board `rpipico2w`, with I2C display + 5-way switch

The RX targets share a single HAL file (`rx-hal-generic-2400-rp.h`), and the TX targets share a single HAL file (`tx-hal-generic-2400-rp.h`). Pin assignments are identical across RP2040 and RP2350 for both roles.

## Building

Build a specific target with PlatformIO:

```bash
# build one target
pio run -e rx-generic-2400-rp2040

# build all four RP targets
pio run -e rx-generic-2400-rp2040 -e rx-generic-2400-rp2350 -e tx-generic-2400-rp2040 -e tx-generic-2400-rp2350
```

Output firmware files are generated in `.pio/build/<env-name>/`:
- `firmware.uf2` — for drag-and-drop flashing via USB mass storage
- `firmware.bin` — raw binary

## Flashing

### UF2 Drag-and-Drop (recommended)

1. Hold the **BOOTSEL** button on the Pico and plug in the USB cable (or reset while holding BOOTSEL).
2. The board appears as a USB mass storage device (e.g., `RPI-RP2` or `RP2350`).
3. Copy the `.uf2` file to the drive:

```bash
cp .pio/build/tx-generic-2400-rp2350/firmware.uf2 /Volumes/RP2350/
```

The board reboots automatically after the copy completes.

### PlatformIO Upload (BOOTSEL)

```bash
pio run -e tx-generic-2400-rp2350 --target upload
```

This requires the board to be in BOOTSEL mode. PlatformIO will detect the mounted drive and copy the UF2 automatically.

### PlatformIO Upload (serial port)

If the board is already running firmware with USB serial enabled, you can upload by specifying the serial port directly — no BOOTSEL button press needed:

```bash
pio run -e tx-generic-2400-rp2350 --target upload --upload-port /dev/cu.usbmodemXXXX
```

PlatformIO will automatically reset the board into bootloader mode, upload the firmware, and reboot.

## HAL Architecture

```
rp-hal.h                    ← dispatch: selects HAL based on build define
├── rx-hal-generic-2400-rp.h ← shared RX HAL (RP2040 + RP2350)
└── tx-hal-generic-2400-rp.h ← shared TX HAL (RP2040 + RP2350)

rp-device_conf.h             ← device names and role/frequency defines
```

The build define (e.g., `-D TX_GENERIC_2400_RP2350`) controls which HAL is included and which device name is compiled in.
