---
description: Build ESP32 firmware for a specific target
---
Compiles the firmware for the specified ESP32 or ESP8285 PlatformIO environment.

// turbo
1. Build the target (clean build - default):
   `python tools/run_make_esp_firmwares.py --target {{target}} --nopause`

// turbo
2. Build the target (incremental build - faster for development):
   `python tools/run_make_esp_firmwares.py --target {{target}} --no-clean --nopause`

// turbo
3. Find the built firmware:
   `ls -l tools/esp-build/firmware/ | grep {{target}}`

> [!TIP]
> Use `/list-targets` or check `platformio.ini` for environment names.
> Use `--no-clean` for faster rebuilds during development (only recompiles changed files).
