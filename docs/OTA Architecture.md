# Receiver OTA Architecture

This document describes how mLRS updates ESP32, ESP8285 and STM32 receivers over the air. The receiver pulls the image, the Tx module relays, and a host answers the receiver's requests. The host is either a tool on the PC or a Lua script on the radio. Both speak the same packets; only the transport to the Tx differs.

```
 Host                         Tx module                      Receiver (ESP32 / ESP8285 / STM32)
 tools/run_rx_ota.py ─USB──┐
 ...  ────────radio USB, ──┤
              JR pin5      ├─ ota_relay_tx.h ───LoRa──────── ota_rx.h
 mLRS-RxUpdate.lua ─CRSF───┘  bridge, no protocol state      asks, owns all timing
 answers requests
```

## Files

| File | Role |
|---|---|
| `Common/ota/ota_protocol.h` | App info, packet structs, commands |
| `Common/ota/ota_link.h` | LoRa link settings and DIO polling, shared by Tx and Rx |
| `CommonRx/ota_rx.h` | `tRxOta`: the request loop and storage for ESP32 / ESP8266 and the STM32 loader, the STM32 loader's state |
| `CommonRx/ota_loader.cpp` | The STM32 loader, in place of `mlrs-rx.cpp` |
| `CommonTx/ota_relay_tx.h` | `tTxOtaRelay`: the radio ↔ com bridge |
| `tools/run_rx_ota.py` | Host tool on the PC |
| `lua/mLRS-RxUpdate.lua` | Host on the radio, image from the SD card |
| `tools/run_rx_ota_file.py` | Makes the image file for the SD card |

When OTA isn't built, both classes compile as empty dummies, so `mlrs-tx.cpp` and `mlrs-rx.cpp` need no `#ifdef`s. `USE_RX_OTA` is defined for:
- all transmitters;
- ESP32 and ESP8266 receivers with an LR11xx, SX128x or SX127x radio;
- STM32 receivers with `DEVICE_HAS_OTA_LOADER`, for now the Matek mR900-30(C) and mR24-30(C).

## Starting an update

1. The host opens the Tx's com port, checks that the CLI answers `v`, and sends `rxota`.
2. If the link is up, the Tx repeats `FRAME_CMD_RX_OTA_ENTER` in its command frames. One second later it starts the relay.
3. When the receiver gets the command, it leaves its main loop and calls `tRxOta::Run()`. On ESP this doesn't reboot. On STM32 the app hands over to the loader, see below.
4. The host switches to relay framing and waits for the receiver's first request.

Internal Tx modules start differently, see [PC via the radio](#pc-via-the-radio-internal-tx-modules).

## Radio link

The update uses LoRa with explicit header and the radio CRC on, on a fixed frequency per band, `ota_link_freq_reg()` in `ota_link.h`. These are the bind channels of `fhss.h`: 2447, 913.8, 863.275, 865.375, 433.36 and 436.4 MHz. Both sides derive the settings from the band and mode they were connected in. The settings are `tSxLoraConfiguration` entries in `ota_link.h`, applied with the drivers' own `SetLoraConfiguration()`.

| Band / mode | Settings |
|---|---|
| 2.4 GHz | SF5, BW800, CR LI 4/5 |
| Sub-GHz | SF5, BW500, CR 4/5 |
| Sub-GHz, 19 Hz 7x | SF7, BW500, CR 4/5. An SX127x can only be on the other side in this mode, and SF7 is its lowest SF with explicit header. |

LR20xx (LR2021) uses the same settings, with BW812 at 2.4 GHz. LR2021 Tx modules can relay, but no LR2021 receiver has OTA yet: none is ESP, and the STM32 ones have no loader.

The `session_id` (= `Config.FrameSyncWord`) is in every packet, so that only the paired Tx and Rx talk to each other.

Both sides **poll the DIO pin** and read the IRQ status over SPI only after DIO is set. The IRQ status itself is never polled. The STM32WL radio has no DIO pin, so there the pending bit of its disabled radio interrupt takes the place of the pin. The setup enables the CRC-error IRQ on all chips, and the header-error IRQ on all but the SX127x, which has none, so a bad packet is never taken as good.

## Protocol: receiver pull

The protocol is strict request-response. The receiver asks and the host answers; the Tx only relays, and neither the Tx nor the host keeps any state.

```
rx -> host: cmd, session_id[2], payload
host -> rx: cmd | 0x80, session_id[2], payload
```

The payloads are packed structs in `ota_protocol.h`, little endian. The protocol version is 1.

| Cmd | Rx asks | Host answers |
|---|---|---|
| INFO | `tOtaInfoRequest`: version, space[4], flags, device_name_20[20] | `tOtaInfoResponse`: status, length[4], flags, image_length[4], crc[4] |
| DATA | `tOtaDataRequest`: offset[4], max_len | `tOtaDataResponse`: offset[4], data[n], with 1 ≤ n ≤ max_len |
| RESULT | status | – |

- **Which receiver:** the receiver is identified by its device name in plain text, as in the bind and setup frames. The image has it in `tOtaAppInfo` (magic, version, `DEVICE_NAME`), which the host tools find by its magic. The receiver sends the same string in INFO, read from `tOtaAppInfo`. Before starting, `run_rx_ota.py` also checks the name of the connected receiver, which the Tx knows from the setup data (`v` in the CLI).
- **INFO:** if the status isn't OK (wrong receiver or protocol version), the receiver leaves straight away.
- **DATA:** `max_len` is 248, the largest payload that fits a LoRa frame. The receiver advances its offset by whatever it gets, and ignores an answer for any other offset.
- **RESULT:** the receiver repeats it until it gets the answer, at most 5 times.

### Receiver state machine

```
INFO -> DATA (repeat until all received) -> RESULT -> DONE -> restart
```

| Rule | Value |
|---|---|
| No answer to a request | Repeat the same request after 300 ms |
| Retries per request | 5, then leave |
| No answer at all | 5 s, then leave |
| Waiting for the host to appear | 10 s (the relay and tool need a moment to start) |

On ESP, leaving always means a restart. If the update didn't complete, the receiver restarts on its previous firmware. On STM32 the loader decides what comes next, see below.

### Storage

| MCU | How the image is stored | Compression |
|---|---|---|
| ESP32 / C3 / S3 | `esp_ota_*` writes into the inactive app partition; the bootloader switches over | deflate (zlib), inflated with the ROM miniz |
| ESP8266 / ESP8285 | Updater writes into the free flash behind the app; eboot unpacks it over the app at the next reset | gzip, stored as received |
| STM32 | the loader writes the app to its place | none |

- **CRC check:** the receiver checks a CRC over everything transferred before it finalizes. For ESP8266 this is the only check, since eboot doesn't verify the stream. It is CRC-32/MPEG-2, as the platforms have it already: the ESP32 ROM (`esp_rom_crc32_be()`), the ESP8266 core (`crc32()`) and the STM32 CRC unit in its default setup.
- **Compression:** ESP receivers are always sent a compressed image. STM32 receivers advertise no compression and are sent the raw image.
- **Params are untouched:** they live in NVS, which the update never writes.

## STM32 loader

STM32 receivers can't hold a second app, so a loader in front of the app receives the update. The loader is built from the same sources as the app: the CubeMX `main.cpp`, HAL, the device hal, the sx driver, `ota_link.h` and the request loop of `ota_rx.h`. Only `CommonRx/ota_loader.cpp` takes the place of `mlrs-rx.cpp`. `tools/run_make_firmwares.py` builds it when the app's map has `mlrs_ota_app`, and puts loader and app into the one `.hex`. The host tool takes that `.hex` and sends the app part of it.

| G431, 128 kB | |
|---|---|
| `0x08000000` | loader, 14 kB (12.3 kB used) |
| `0x08003800` | loader state, one 2 kB page |
| `0x08004000` | app, up to the EE pages, 104 kB |
| `0x0801E000` | EE pages, powerup counter page, untouched |

The `.ld` has one changed line, in the style of the ELRS bootloader targets: `DEFINED(mlrs_ota_app)` moves the app behind the loader, `DEFINED(mlrs_ota_loader)` limits the loader to 14 kB. The loader sets `VTOR` and starts the app with the clocks and peripherals deinitialized by the HAL, so the app starts as after a reset.

### Loader state

The state page is a list of records (16 bytes, the type word is written last), which are only appended; the last one counts. The app writes two of them, the loader the others.

| Record | Written by | Meaning |
|---|---|---|
| LINK | app | please update; frequency, session id, band and mode for the link |
| ERASE | loader | the app is being written; the page is started over here, with the LINK copied |
| NEW | loader | a complete new app; its length and crc |
| BOOT | loader | the new app was started |
| OK | app | the app runs fine, written when it connects for the first time |

At boot the loader starts the app, unless:
- the last record is LINK: the app asked for an update;
- the app is invalid: its first double word holds the stack pointer, and is written last, after the crc check over the flash. A half-written app is never started;
- the new app was started 3 times and never wrote OK.

In these cases it waits for an update, with the link from the LINK record, until one comes. If the app asked but no update came, it writes OK and starts the app again. Without a LINK record it doesn't know the link; it blinks red, and only flashing by wire helps.

Flashing by wire always wins: the loader compares the app with the crc in NEW, and starts it if it is a different one.

### Measured

Matek mR900-30C (G431, SX1262) from the Nomad (115200 UART), 915 FCC 31 Hz, 77.7 kB raw: 21.6 s, 314 requests, no repeats.

- Update cut off half way, then reset: the loader waits, a second run without a connected receiver completes.
- An app which hangs before connecting: started 3 times, after the 4th reset the loader waits; an update or a flash by wire brings it back.

### Open

- A power loss right while a record is written can leave an ECC error in the state page, and reading it gives an NMI.
- Other STM32 families: only G4 is set up. The state records have a word variant, but the app is written in double words (G4, L4, WL), and F1 needs its own crc, since its CRC unit has no initial value.

## Tx relay

```
host -> tx / tx -> host: 0xA5, len, packet[len], crc16     (len = 0 from the host ends the relay)
```

- **Forwarding:** whatever the receiver sends goes to the host, and whatever the host sends goes over the radio.
- **IRQ:** the DIO interrupt is off while the relay runs. The controller restart afterwards turns it back on.
- **Ending:** the relay ends when the host sends an empty frame, or after 10 s of silence from the host.
- **Power:** the Tx sends at its lowest power level, as Tx and receiver usually sit next to each other.
- **RAM:** the relay's buffers are on the stack of `Run()`, so OTA costs no RAM outside of an update (R9M: 16 instead of 544 bytes).
- **Transport:** the relay talks to a `tSerialBase`. For the PC this is the com port, or `jrpin5serial` for an internal Tx module. For the radio it is `crsf`: the frames go as a byte stream in crsfbridge envelopes, both ways, see below. The relay enables the bridge for the update and keeps the CRSF telemetry slots going itself, since the main loop is blocked.

## PC via the radio (internal Tx modules)

Internal Tx modules have no com port. Through EdgeTX's `serialpassthrough rfmod 0` the PC reaches only the JR pin5 UART, which belongs to the Tx's CRSF driver. `run_rx_ota.py <radio's USB serial> <image>`, the tool finds the radio by itself: the Tx CLI answers `v;` within 1 s, the radio doesn't. Then it:

1. Stops the pulses, powers the module on, and starts the passthrough. Stopping the pulses powers the module off, so it boots anew.
2. Plays the radio: sends channel frames (ch3 at -100%, the rest mid) until the link statistics show the receiver connected. The module only replies to frames, and only sends telemetry after channel frames. It autobauds; the radio follows the PC's baud, so the tool scans the module's rates.
3. Sends `MBRIDGE_CMD_RX_OTA_JRPIN5` (20) in a CRSF frame, three times. The Tx starts the relay with `jrpin5serial`. Its `SetBaudRate(115200)` restarts the UART without the CRSF `onReceive()` callback, as the ESP passthrough does. From here it's the same framing as on a com port.
4. The controller restart after the relay gives the UART back to CRSF. The radio must be power cycled, so that it and the module redo the CRSF autobaud and the radio sends the model ID again.

- **Only ESP32 Tx modules with a full-duplex pin5** (`USE_RX_OTA_VIA_JRPIN5`).
- **Buffer:** with `UART_RXBUFSIZE 0` (TX15) the plain UART has only the 128-byte FIFO, half of a 259-byte answer. The relay loop polls it often enough.
- **Measured:** TX15 → Matek mR900-30C, 24.5 s, 314 requests, 0 repeated. The same receiver through a Nomad's CLI took 21.6 s.

## Radio as host (SD card)

`mLRS-RxUpdate.lua` is a second host. It works with internal and external Tx modules in CRSF mode, with no PC. The receiver side is the same as with the PC.

1. The script sends `MBRIDGE_CMD_RX_OTA`. The Tx starts the relay just as for `rxota`, but with `crsf` as the host serial. The script repeats the command every 3 s until the receiver shows up.
2. The script answers INFO, DATA and RESULT, as `run_rx_ota.py` does, and ends the relay with an empty frame.

```
radio -> tx: 0xEE, len, 0x81, 0x66, seq, n, data[n], crc8     (crsfbridge envelope)
tx -> radio: 0xEA, len, 0x82, 0x66, seq, n, data[n], crc8
data:        the byte stream of 0xA5, len, packet[len], crc16, as with the PC
```

The relay is the same as with the PC. A 259-byte answer takes 5 envelopes, and the Tx puts it together in the bridge's receive FIFO and the relay's frame buffer. The crc16 protects it: if an envelope gets lost, the frame is dropped, and the receiver asks again after 300 ms.

### Why the script answers each request

A first version also answered each request, but with answers of one frame, 43 bytes. It reached less than 1 kB/s, and on the TX15 EdgeTX went into Emergency Mode, see below. A second version streamed the data ahead into a 512-byte window in the Tx, which answered the requests itself; it reached ~5 kB/s, but needed its own framing, flow control and go-back logic in the script and the Tx.

The current version keeps the request-response of the PC, with full answers:

- **An answer spans envelopes.** EdgeTX sends a Lua frame of at most 64 bytes (`TELEMETRY_OUTPUT_BUFFER_SIZE`), one per mixer cycle. The relay's byte stream doesn't care where the envelopes split it, so the receiver gets 248-byte answers as from the PC.
- **The script waits for requests.** EdgeTX runs it every 50 ms. The script stays 30–40 ms of it (`BURST_TICKS`) and answers what comes in meanwhile, so most requests don't wait for its next turn. Returning as soon as nothing was to be sent gave 2.25 kB/s on the TX15.
- **The script reads ahead 1 kB from the file.** The receiver asks in order, so the file is read in chunks instead of a seek and read per request.

### Finding: crossfireTelemetryPush() overwrites the heap

EdgeTX up to 2.12.4 leaves each payload byte of `crossfireTelemetryPush()` on the Lua stack (`lua_rawgeti()` without `lua_pop()`). A C function gets 20 free slots (`LUA_MINSTACK`), so a frame with more than about 20 payload bytes writes past the Lua stack once the garbage collector has shrunk it, into the next heap blocks. The radio crashes later, in `free()` or in the garbage collector, and the watchdog puts it into Emergency Mode, after 2 to 9 requests or right away. This was the Emergency Mode of the first version and of the first versions of the current one. EdgeTX #7773 fixes it on main; it is not in the 2.12 branch.

- **Found:** with SWD on the TX15 (pyocd attached to the running radio, PC sampling, a walk of newlib's free list). The overwritten block was the Lua stack, freshly shrunk to 68 slots, followed by the envelope's bytes.
- **Why the stream-ahead version got away with it:** it built each frame with `string.byte(s, 1, n)`, which grows the stack for 47 values just before each push.
- **Workaround:** the script pushes through `telemetryPush()`, whose unused locals make Lua reserve 67 slots for its frame, so the bytes left on the stack stay in that frame. `mLRS.lua` pushes up to 27 bytes and is exposed as well.

### The file

`run_rx_ota_file.py <firmware.bin>` makes it, and it goes into `/FIRMWARE`. It has a 56-byte header (magic, version, lengths, the zlib/gzip trailer values, the CRC of the zlib and of the gzip stream, device name, flags), then the image's raw deflate stream. For STM32 it takes the `.hex`, and the file has the app as it is, with the RAW flag; both CRCs are then the CRC of the app, and the script sends it unchanged. Lua can't compress, so the script puts the zlib head/tail (ESP32) or the gzip head/tail (ESP8266) around the stored stream, depending on what the receiver advertises. One file serves both. The firmware knows nothing about the file.

EdgeTX's Lua has 32-bit integers and single-precision floats, so a crc32 doesn't survive a division. The script splits values into bytes with `bit32`.

### Finding: CRSF frames split on ESP32 Tx modules

ESP32 Tx modules lost about 3–4% of the script's 61-byte frames. The script then went back in the file: ~190 times per image, at 3.6 kB/s.

- **Mechanism:** the ESP32 Tx gets the radio's bytes in an `onReceive()` callback, which the UART's receive timeout triggers after one symbol of silence (`setRxTimeout(1)`, ~11 bit times). `pin5_rx_callback()` took each callback for a whole frame and reset the parser at its end. Now and then the timeout comes in the middle of a frame. The rest comes with the next callback, but the frame was dropped.
- **Measured:** the cut is at a random place in the frame (byte 10 to 54), the bytes are intact, and the time between the two parts is 0–13 µs more than the bytes need, so at most a symbol or two. This is right at the one-symbol threshold.
- **Not the cause:** the Tx's own CRSF replies, its LoRa transmissions, the script's SD reads, the UART driver's buffers (no errors), or the rate (3 frames per turn gave the same 4%). The radio had EdgeTX 2.12.4, which already has the TX DMA FIFO fix for byte starvation (EdgeTX #7490).
- **Fix:** the parser keeps a frame over two callbacks once its len byte is in. The crc8 still decides, and the parser's 500 µs character timeout still ends a frame that really broke off. A lone begin byte is dropped as before, since on half-duplex it could be the ghost byte of the line switch. This is what STM32 Tx modules do anyway, as they parse each byte in the UART interrupt.
- **Normal operation:** channel frames (26 bytes) get split the same way, just less often, and are now kept too.

### Measured

With the second version (stream ahead), before the new version:

| Tx | Rx | Rate | Went back |
|---|---|---|---|
| TX15 internal (ESP32, full duplex, 1.87 MBaud, 500 Hz) | XR4 | ~5 kB/s | 0 |
| BetaFPV Micro 1W (ESP32, half duplex) | XR4 | 5.5 kB/s | 0 |

Before the fix: TX15 3.6 kB/s with 191 go-backs. EdgeTX didn't go into Emergency Mode with this design.

## Host tool

`run_rx_ota.py <port> <firmware.bin>` does the following:
- reads the device name from the image;
- checks the CLI and the link before starting;
- compresses the image according to what the receiver advertises;
- answers requests until the RESULT arrives.

## Measured times

All runs used the compressed image (~216–224 kB for ESP32, 247 kB for ESP8285) at the minimum power level. Every receiver came back on its new firmware.

| Rx | Tx | Mode → OTA link | Time | Requests |
|---|---|---|---|---|
| XR4 (ESP32, LR1121) | Nomad (LR1121) | 2.4 GHz 50 Hz → SF5 | 47.6 s | 902 |
| XR4 | Nomad | 915 31 Hz → SF5 | 61.9 s | 902 |
| XR4 | Nomad | 915 19 Hz 7x → SF7 | 127.7 s | 901 |
| XR1 (ESP32-C3, LR1121) | Nomad | 2.4 GHz 50 Hz → SF5 | 48.7 s | 873 |
| XR1 | Nomad | 915 31 Hz → SF5 | 60.7 s | 872 |
| XR4 | BetaFPV Micro 1W (SX1281) | 2.4 GHz 50 Hz → SF5 | 46.9 s | 901 |
| XR1 | BetaFPV Micro 1W | 2.4 GHz 50 Hz → SF5 | 48.1 s | 872 |
| Generic 900 (ESP8285, SX1276) | Nomad | 915 19 Hz 7x → SF7 | 141.4 s | 997 |
| Generic 900 | Bandit 900 (SX1276) | 915 19 Hz 7x → SF7 | 141.3 s | 1004 |
| XR4 | Matek mR900-30C (G431, SX1262), native USB | 915 31 Hz → SF5 | 39.9 s | 901 |
| Generic 900 | R9M (F103, SX1276), from the radio's SD card, CRSF 400k | 19 Hz 7x → SF7 | 157 s | – |

- **Repeated requests:** 0–7 per run.
- **Cross-chip pairs:** SX1281 ↔ LR1121 at 2.4 GHz and LR1121 ↔ SX1276 at SF7 work with explicit header.
- **LR1121 at SF7:** with the SX127x compatibility register left on from 19 Hz 7x mode, it causes no problem.
- **Not run on hardware** (judged low risk): SuperD (ESP32, SX1281) and Generic 2400 D PA (ESP8285, SX1280). LR2021 Tx modules as the relay are build-checked only.

## Findings and future work

- **Privacy must be off**, by design. The OTA link isn't encrypted, so the receiver ignores the enter command when Privacy is on.
- **CLI on some Tx modules.** On the BetaFPV and Bandit, the CLI is only on the USB-UART when the 5-way is held down at boot, or when the serial port setting leaves that UART free (`USE_COM_ON_SERIAL`).
