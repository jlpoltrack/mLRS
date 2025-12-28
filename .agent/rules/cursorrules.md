---
trigger: always_on
glob:
description: mLRS build and flash rules
---

# mLRS Agent Rules - Build & Flash

You are an expert at building and flashing mLRS firmware. Use these rules to assist the user proactively.

## Build System Knowledge

### STM32 Targets
- **Script**: `tools/run_make_firmwares.py`
- **Target Definition**: Found in the `TLIST` array within the script.
- **Common Patterns**: 
  - `rx-matek-...`: Receiver targets
  - `tx-matek-...`: Transmitter targets
  - `-default`, `-can`, `-siktelem`: Common variant suffixes.
- **Commands**:
  - Build: `python tools/run_make_firmwares.py --target <TARGET_NAME>`
  - Flash: Add `--flash-dfu` or `--flash-swd`.
  - Always use `--nopause` for non-interactive execution.

### ESP32/ESP8285 Targets
- **Script**: `tools/run_make_esp_firmwares.py`
- **Target Definition**: PlatformIO environments in `platformio.ini`.
- **Commands**:
  - Build: `python tools/run_make_esp_firmwares.py --target <TARGET_NAME>`
  - Flash: Add `--flash`.

## Build Artifacts
- **STM32 binaries**: Located in `tools/build/<TARGET_NAME>-<VARIANT>/` (e.g., `tx-...-default/`). Look for `.hex` and `.elf`.
- **ESP32 binaries**: Copied to `tools/esp-build/firmware/` after build.
- **Naming Convention**: Binaries typically include version, branch, and git hash (e.g., `target-v1.2.3-branch-@hash.bin`). Always report the full filename to the user.
- **Size Verification**: When reporting build success, always include a summary of the `text`, `data`, and `bss` sizes to help the user track memory usage.

## Proactive Assistance Rules

1. **Smart Target Discovery**: 
   - When a user mentioned hardware (e.g., "Nomad", "Matek G4", or "mr900"), search both `tools/run_make_firmwares.py` (TLIST) and `platformio.ini` (env names or defines) for matching strings.
   - **Multi-Part Resolution**: If a user says "mr900 tx", look for strings containing both `mr900` and `tx`. Map "mr900" to `tx-matek-mr900-30-g431kb` if it's the most likely match.
   - **Variant Mapping**: Map user keywords like "default", "can", "siktelem", or "oled" to the target's `-appendix` or `-variant` suffix.
   - **Variant Defaulting**: If a user specifies a hardware name without a variant, prioritize the variant with the `-default` appendix if available.
   - Infer prefixes: If only "Nomad" is given, resolve to `tx-radiomaster-nomad`.
   - If the user has recently edited files in a specific HAL directory (e.g., `mLRS/G4`), prioritize targets using that MCU.
   - If multiple targets match, list them and ask for clarification.

2. **Suggesting Builds**:
   - After significant code changes or fixing a bug, proactively ask: *"Would you like me to build the firmware for [TARGET] to verify these changes?"*

3. **Error Troubleshooting**:
   - If a build fails due to a missing toolchain, check if `arm-none-eabi-gcc` is in the path or if `ST_DIR` needs to be set.
   - If flashing fails, suggest checking the USB connection or bootloader mode (DFU).

4. **Workflow Usage**:
   - Prefer using the established workflows in `.agent/workflows/` (e.g., `/mlrs-build-stm32`) when assisting the user, to ensure consistency.
