# Receiver OTA Architecture

How mLRS updates a receiver's firmware over the air, as built on the `OTA-LoRa` branch, followed by the open issues from review and the proposed direction.

## Overview

```
 Host                       Tx module                     Receiver
 ────                       ─────────                     ────────
 run_rx_ota.py ──USB/com──> ota_relay_tx.h ──LoRa──────> STM32: ota_loader.cpp (own flash region)
 (or radio VCP passthrough  (relay, dumb pipe)            ESP:   ota_rx.h (inside the app)
  to JR pin5, ESP Tx only)
 Lua + SD card ──CRSF─────> ota_relay_tx.h (acts as host)
```

The OTA link does not use the normal mLRS link. Both sides leave their link loops and switch to a dedicated stop-and-wait LoRa link on the band's bind frequency:

| Chip    | Settings                                        |
|---------|-------------------------------------------------|
| SX128x  | SF5, BW800, CR4/5 LI, explicit header, CRC on   |
| SX126x  | SF5, BW500, CR4/5, explicit header, CRC on      |
| SX127x  | SF6, BW500, CR4/5, implicit header (SF6 requires it) |
| LR11xx, LR20xx | settings of the band and mode it was connected on, see `ota_link.h` |

Both sides transmit at minimum power, since the Tx and Rx sit next to each other. Every packet carries a `session_id` (= `Config.FrameSyncWord`) so that only the paired Tx and Rx talk to each other.

## Entering OTA

1. The user runs `rxota` in the Tx CLI. Alternatives are `MBRIDGE_CMD_RX_OTA` (internal module through the radio's USB passthrough) or a Lua script sending the file header (SD card).
2. If the link is connected, the Tx repeats `FRAME_CMD_RX_OTA_ENTER` in its command frames for 1 s, then starts the relay. A receiver that is already in its loader isn't connected, so the Tx starts the relay regardless.
3. The receiver calls `ota_enter_loader(bind_freq, FrameSyncWord)`:
   - **STM32:** writes a `tOtaParams` record (magic, freq reg, SF/BW/CR, power, session id, check word) to the params page and resets.
   - **ESP:** runs the OTA loop in-app, with no reboot.

## STM32 receiver: loader and app in one build

Currently this exists only for the Matek mR24-30 and mR900-30 (+C) receivers on G431KB.

### Flash layout (G431KB, 128K, 2K pages)

| Address      | Page  | Content                                             |
|--------------|-------|-----------------------------------------------------|
| `0x08000000` | 0–1   | OTA loader, 4K, owns the reset vector               |
| `0x08001000` | 2–59  | App, 116K max; app info at app base + `0x200`       |
| `0x0801E000` | 60–61 | EE (setup)                                          |
| `0x0801F000` | 62    | OTA params **and** power-up counter (see issues)    |
| `0x0801F800` | 63    | unused                                              |

### How the single build works

There is no separate bootloader project. `ota_loader.cpp` is compiled with the app, and the linker script splits the output:

```ld
LOADER (rx) : ORIGIN = 0x8000000, LENGTH = 4K
FLASH  (rx) : ORIGIN = 0x8001000, LENGTH = 116K

.ota_loader : { KEEP(*(.ota_loader_vectors)) *ota_loader.o(.text .text* .rodata .rodata*) } >LOADER
.isr_vector : { KEEP(*(.isr_vector)) . = 0x200; KEEP(*(.ota_app_info)) } >FLASH
```

- The loader has its own 16-entry vector table (SP = RAM end, reset = `ota_loader_reset`).
- The app's vector table moves to `0x08001000`. The app doesn't shift VTOR itself: the loader sets `SCB->VTOR = OTA_APP_BASE` before jumping. This relies on CMSIS `SystemInit` leaving VTOR alone, which holds when `USER_VECT_TAB_ADDRESS` is undefined.
- A wired flash writes loader + app. An OTA transfer sends only the app: `run_rx_ota.py` strips the first 4K. **The loader itself is never updated over the air.**
- `tOtaAppInfo` (magic, target id, length, version) sits at app base + `0x200`. The build leaves `length` at 0. The tool fills in `length` and appends a CRC32 trailer.

### Loader constraints

The loader erases and programs the app region while it runs, so it must never execute anything that lives there:
- no calls into app code, no libc, no `.data` or `.bss`;
- `#pragma GCC optimize ("no-tree-loop-distribute-patterns")` stops gcc from emitting `memcpy`/`memset` calls.

As a result, the GPIO, SPI, flash, SysTick and SX126x/SX128x drivers are reimplemented at register level in `ota_loader.cpp`. The SX chips' constants come from `sx12xx-lib` headers. All timing is polled, with no interrupts.

### Boot decision

```
reset → ota_loader_reset()
  app valid && no params        → jump to app
  no params && app invalid      → fail (blink red, wired flash needed)
  params valid                  → OTA loop
        30 s idle, no transfer, app valid → leave
        END ok                  → erase params page, reset
```

"App valid" means: magic and target id match, and either `length == 0` (a wired build, which can't be CRC-checked) or the CRC32 over `[0, length-4)` matches the trailer. The loader computes the CRC with the STM32 CRC unit, set up for the standard (zlib) CRC-32.

## ESP receivers

ESP receivers have no loader, because the app receives the image itself (`ota_rx.h`):
- **ESP32:** `esp_ota_*` into the second app partition. A deflate (zlib) stream is accepted on C3/S3, which have inflate in ROM.
- **ESP8266:** a gzip stream is stored in the free flash behind the app, and the Updater unpacks it on boot.

A task watchdog runs during the transfer. On idle, stall or after END, the app restarts.

## Radio protocol (loader version 1)

The protocol is push, stop-and-wait. The Tx side sends a packet, and the Rx answers each one.

```
tx -> rx: cmd, session_id[2], payload
rx -> tx: cmd|0x80, session_id[2], status, payload
```

| Cmd   | Tx payload                                        | Rx response                                               |
|-------|---------------------------------------------------|-----------------------------------------------------------|
| HELLO | –                                                 | status, loader_version, block_size, target_id[4], app_size_max[4], flags |
| BEGIN | length[4], target_id[4], flags, image_length[4]   | status, next_block[2]                                     |
| DATA  | block[2], data[n]                                 | status, next_block[2]                                     |
| END   | crc32[4]                                          | status, next_block[2]; reboots if ok                      |

- The STM32 block size is 128 bytes, and each DATA that starts a 2K page erases that page first. ESP uses 240-byte blocks.
- The Rx accepts only `next_block`. Repeated or out-of-order blocks are ignored, and the response says which block the Rx wants.
- Flags: `DEFLATE` (ESP32) and `GZIP` (ESP8266). The STM32 loader advertises none.

## Tx relay

`ota_relay_tx.h` knows only the session id and the link, not the OTA protocol. The host frames packets as:

```
host -> tx: 0xA5, len, packet[len], crc16     (len = 0 ends the relay)
tx -> host: 0xA5, len, response[len], crc16   (len = 0: receiver did not respond)
```

For each host packet, the relay sends it over LoRa, waits up to 400 ms for one response (long enough to cover a page erase), and passes it back. It leaves after 10 s of host silence and then restarts the Tx.

**The protocol state lives in the host tool** (`run_rx_ota.py`). The tool keeps a send-ahead window of DATA packets queued in the Tx's com buffer. On a lost or unexpected response it drains what is in flight and goes back to the Rx's `next_block`. It also handles retries and the overall timeout.

Host transports:
- **Com port:** the default.
- **JR pin5** (ESP Tx with full-duplex pin5): the radio passes USB through, the mBridge command starts the relay, and the uart drops to 230400.
- **SD-card stream** (`USE_RX_OTA_STREAM`, any Tx with JR pin5): the Tx acts as the host. A Lua script sends a `tOtaFileHeader`, then the file data in CRSF frames (`0xEE, len, 0x81, 0x67, offset[3], n, data[n], crc8`). The frames are intercepted in the CRSF parser in ISR context. The Tx paces the radio with `MBRIDGE_CMD_RX_OTA_STATUS` (rx_offset, room, nack_seq). If the Rx wants deflate or gzip, the Tx wraps the raw deflate data from the file in zlib or gzip framing.

## Target identification

`OTA_TARGET_ID` is the FNV-1a hash of `DEVICE_NAME`. It is checked in BEGIN and when validating the app. Renaming a device makes it a new target, and its old firmware then refuses the renamed image.

---

## Open issues from review

1. **Page 62 is shared by the OTA params and the power-up counter** (`POWERUPCNT_EE_PAGE = EE_START_PAGE + 2`). Each side erases the page independently.
2. **There is no fallback for a bad image.** A CRC-valid image that hangs or never links can't get back into the loader, because only the app can write the params page. Recovery needs a wired flash.
3. **The `.ld` coupling.** The `.ota_app_info` section is placed at a fixed `0x200` offset. That fits G431's vector table, but larger MCUs (G474, H5) have bigger tables.
4. **The loader doesn't reuse existing code.** It is written at register level instead of using LL or the existing drivers.
5. **The Tx integration is invasive.** The `#ifdef` blocks and loose globals in `mlrs-tx.cpp` and the ISR hook in the CRSF parser all come mostly from the SD stream.
6. **Unrelated changes on the branch:** the LR1121 firmware update and `USE_COM_ON_SERIAL`.
7. **The target id is a hash** rather than readable text.
8. **The link settings are per chip** (explicit/implicit header) instead of one common configuration such as bind's.

## Proposed direction

### Flash layout and boot info
- Move the boot info and params into the **last page of the loader region**, as in OlliW's UAVCAN bootloader. This removes the page-62 collision and the app-side `.ota_app_info` section. The app then only needs its vector table at `OTA_APP_BASE`.
- Add a three-state boot flag: `IS_CALL_FROM_APP`, `APP_NEEDS_TO_ACK`, and `IS_EMPTY = 0xFFFFFFFF`.
  - The loader sets `APP_NEEDS_TO_ACK` before the first jump into a new image and starts the IWDG.
  - The app clears the flag once it has proven itself, for example at the first link connection.
  - If the flag is still set at reset, the loader stays in OTA mode.

### Loader code
- Rebuild the loader on CMSIS/LL `static inline` helpers. Their code lands inside `ota_loader.o`, so the self-containment rule is kept.
- Avoid non-static `inline` (COMDAT) functions, which the linker may resolve to a copy in the app region.
- Add a post-link check that no loader symbol references an address ≥ `OTA_APP_BASE`.

### Protocol: receiver pull
The Rx becomes the active side: it requests block N and the server answers with that block. The loader's state machine then becomes:

```
IDLE → SENDNEXTREQUEST → WAITFORRESPONSE → RESPONSE | LASTRESPONSE → DONE
```

A timeout goes back to `SENDNEXTREQUEST`, so there are no error states.
- **Gains:** the Rx controls timing, so the sender no longer has to guess flash erase and program times. The server side becomes trivial: no window, resync or retry logic. The Rx also decides itself when the transfer is complete.
- **Costs:** the loader gains a retry timer and a give-up policy. Throughput suffers if the Tx stays a dumb pipe, because every block then costs a radio + USB + host round trip. Keeping today's speed would need a small prefetch ring on the Tx.

### Link settings
- Use the bind link configuration as the one common setup for all chips, and remove the per-chip table. Throughput needs to be measured first.

### Tx integration
- Add a dummy `tTxOtaRelay` class for builds without `USE_RX_OTA`, and move the OTA globals into it.
- Hook the CRSF parser through a registered handler instead of inline code in `parse_nextchar`.

### SD card with pull (deferred)
- **Option 1:** the Lua script serves block requests (`crossfireTelemetryPop`, then `io.seek`/`io.read`, then `crossfireTelemetryPush`). This is fully pull, at roughly 20–40 ms of extra time per block (estimated).
- **Option 2:** Lua streams the file sequentially into a Tx prefetch ring, and the Rx pulls from that ring. On a miss, the Tx asks Lua to rewind. The USB path's prefetch ring could be shared.
- Start with option 1, and move to option 2 only if the measured transfer time is unacceptable.

### Proposed PR split
1. LR1121 firmware update: its own PR, with its own justification.
2. `USE_COM_ON_SERIAL`: its own PR.
3. ESP receivers, USB only. Brings in the Tx relay and switches the radio protocol to receiver pull, so it doesn't change again for STM32.
4. STM32 + USB-only OTA, with the flash layout, boot flag and LL loader changes above, on the pull protocol from 3.
5. SD-card stream.
