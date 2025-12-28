---
description: List available hardware targets for STM32 and ESP32
---
This workflow helps you discover the correct target names for building and flashing.

### STM32 Targets
The STM32 build system uses a Python script to manage targets defined in its `TLIST`.

// turbo
1. List all STM32 targets and their variants:
   `python tools/run_make_firmwares.py --list-targets --nopause`

### ESP32 Targets
The ESP32 system uses PlatformIO environments defined in `platformio.ini`.

// turbo
2. List all ESP32 environments:
   `python tools/run_make_esp_firmwares.py --list-targets --nopause`
   (Note: Use target names like `rx-generic-2400-d-pa` or `tx-generic-2400-v2`)
