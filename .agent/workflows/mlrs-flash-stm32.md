---
description: Build and flash STM32 firmware
---
Simplified workflow using the auto-detection flash mechanism.

// turbo
1. Build and Flash (Auto-detect DFU/SWD):
   `python tools/run_make_firmwares.py --target {{target}} --flash --nopause`

// turbo
2. Fast Development Iteration (Incremental Build + Flash):
   `python tools/run_make_firmwares.py --target {{target}} --flash --no-clean --nopause`

> [!TIP]
> **Performance**: Use `--no-clean` for ~32x faster rebuilds during active development (only recompiles changed files).
> **Auto-detection**: The `--flash` flag automatically detects if the device is connected via SWD (ST-Link) or DFU (USB). SWD is prioritized and tried first.
