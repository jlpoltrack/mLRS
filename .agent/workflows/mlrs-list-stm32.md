---
description: List available STM32 hardware targets
---
Lists all STM32 targets from the build system's TLIST including all variants (default, siktelem, can, etc.).

// turbo
1. List all STM32 targets:
   `python tools/run_make_firmwares.py --list-targets --nopause`

> [!TIP]
> This is much faster than building - it just reads the target list without compiling anything.
