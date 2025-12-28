---
description: Build and flash ESP32 firmware via USB serial
---
Builds the firmware and then flashes it to the hardware via USB serial.

// turbo
1. Build and flash the target:
   `python tools/run_make_esp_firmwares.py --target {{target}} --flash --nopause`
