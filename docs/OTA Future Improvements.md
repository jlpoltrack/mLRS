# OTA Future Improvements

Ideas for after the current OTA (see `OTA Architecture.md`), with rough sizes. None of this is built.

## 1. STM32 loader update from the app

The app carries a copy of the loader and replaces the loader in flash when they differ.

- **How:** `ota_loader.cpp` already `.incbin`s `ota_loader.bin` into `.ota_loader` at the flash start. A second `.incbin` into the app's `.rodata` puts a copy into the part the host sends. In `tRxOta::Run()`, when the next update is asked for, the app compares the copy with the flash at `0x08000000` and rewrites the loader if they differ, verifies it, and only then writes LINK and resets. So the loader that receives update N is the one which came with app N-1. No version number is needed, and the host tools stay the same. About 40 lines in the app part of `ota_rx.h`.
- **Why there:** with `Confirm()` gone, this is the best point at which the app is known to run: it connected and took an ENTER from its Tx. The receiver is on the bench and powered on purpose, the radio is idle and the isrs are off already. It never runs in flight, nor at power-up, where connector bounce makes power losses most likely. A wire flash writes the `.hex` with both parts, so loader and copy match after it.
- **Cost:** the app grows by the loader, 12.3 kB (G431) and 12.7 kB (WLE5). That's about 16% more transfer, 3–4 s on a G431, and less room for the app (104 kB on the G431).
- **Risk, accepted:** a power loss while the loader is rewritten (only once per new loader) bricks the receiver until it's flashed by SWD. The window is about 0.3 s: 7 page erases of ~22 ms plus ~1750 double-word writes. No stage-0 in front of the loader; we accept the window.
- **Contract:** the state-page records and `OTA_APP_BASE` must stay the same for every loader and app, since any loader may meet any app.

**Space:** every byte the loader gains now counts twice: in the loader's 14 kB region, and again in every app image, which is sent over the air. The 128 kB parts aren't flush with flash, so every loader feature must be weighed against this.

## 2. OTA with Privacy on

Today OTA needs Privacy off (`ota_relay_tx.h:82`, `mlrs-rx.cpp:287`). With Privacy on, `RX_OTA_ENTER` already goes as an encrypted command frame. But the update itself is clear text on a fixed, known frequency, and the session id is in every packet. Anyone in range can answer INFO and DATA before the host and give the receiver their own image. The CRC32 comes in the INFO answer, so it doesn't help.

**Proposal:** the encrypted ENTER also carries the length and a 16-byte hash of what will be transferred. The receiver commits the image only if the hash matches.

| Part | ~Lines |
|---|---|
| `frames.h`: length[4] + hash[16] in the ENTER payload (64 bytes; the startup command uses 29) | 8 |
| Tx: `rxota <hash>` in the CLI, hash in the mBridge command (Lua, jrpin5), handed to the link task | 25 |
| Rx ESP: unpack, incremental hash next to the CRC in `handle_response()`, compare before `ota_storage_end()` | 25 |
| STM32 app to loader: the hash goes into the state page, as one more record type (a LINK record has only 12 data bytes) | 20 |
| STM32 loader: SHA-256 (borrowed public-domain implementation) | ~150 |
| `run_rx_ota.py`, `run_rx_ota_file.py` | 12 |
| Lua: passes the hash from the file header on in the mBridge command | 10 |
| Web Flasher `rxOta.ts` | 15 |

- **The hash covers the stream as transferred.** It's the only thing ESP8266 can check before eboot runs. The Lua path wraps the same deflate stream with zlib or gzip framing, so the `.ota` file needs both hashes, as it has both CRCs now. Its header grows from 56 to 88 bytes.
- **Algorithm:** SHA-256, because it's 32-bit friendly. It's about 1 kB in the loader, against about 2 kB for Monocypher's BLAKE2b. Every other side has SHA-256 already: ESP32 (mbedtls), ESP8266 (BearSSL), Python (`hashlib`), browser (WebCrypto). Neither the G431 nor the WLE5 has a HASH unit.
- **Loader space:** 12.3 + ~1 kB (G431) and 12.7 + ~1 kB (WLE5) of 14 kB, and with item 1 the same ~1 kB again in each app. It just fits; nothing else would after it.

### What it gains

- **Image integrity and origin, Privacy levels 2 and 3.** The receiver installs only the image whose hash came in an authenticated frame from its own Tx. A third party who answers the requests gets a hash mismatch, and the image is never committed:
  - ESP32: the boot partition isn't switched;
  - ESP8266: `Update.end()` isn't called, so eboot doesn't run;
  - STM32: the first double word isn't written, so the app is invalid and the loader waits for another try.
- **Starting an update stays as it is.** Only the paired Tx can send ENTER.

### What it doesn't gain

- **Level 1:** it has no MAC, and the stream cipher is malleable. The hash of a public image is known, so an attacker can XOR in the hash of their own image. No protection at level 1.
- **Availability:** jamming or corrupt answers still fail the update. On STM32 an interrupted update leaves the receiver in its loader until the next try, which is the same as today.
- **Replay:** there is no expected-nonce check yet. A replayed ENTER would start an update in flight, so the link drops. With an older hash it would allow a downgrade to an older genuine image. The expected-nonce rule would cover both. Independent of this, an "only when not armed / RC idle" guard on ENTER is worth having.
- **Trust anchor:** the anchor is the Tx and its host, not a signature from the mLRS project. A compromised PC or radio can still install anything; that's no worse than flashing by wire.
- **Confidentiality:** not needed, the firmware is public.
- **Tx modules that reset on port open** (BetaFPV): with Privacy on the receiver doesn't re-authenticate after the Tx restarts, so the ENTER never arrives. These need the receiver power cycled.
