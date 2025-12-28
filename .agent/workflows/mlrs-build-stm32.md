---
description: Build STM32 firmware for a specific target
---
Compiles the firmware for the specified STM32 hardware target.

// turbo
1. Build the target (clean build - default):
   `python tools/run_make_firmwares.py --target {{target}} --nopause`

// turbo
2. Build the target (incremental build - faster for development):
   `python tools/run_make_firmwares.py --target {{target}} --no-clean --nopause`

// turbo
3. Find the built firmware:
   `ls -l tools/build/firmware/ | grep {{target}}`

> [!TIP]
> Use `/mlrs-list-stm32` if you are unsure of the target name.
> 
> **Clean build** (default): Use for first build or after pulling changes.
> **Incremental build** (`--no-clean`): ~32x faster for development (only recompiles changed files).
