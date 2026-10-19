# RP2354 OTA

This document describes how RP2354 devices (RP2350 with 2 MB in-package flash) take firmware updates over the air (receiver-pull, see [OTA Architecture](OTA%20Architecture.md)), over DroneCAN, and by USB. All three write into the inactive half of an A/B partition pair, so the running image is untouched until the new one is complete and checked. References are to the RP2350 datasheet (`docs/datasheets/RP-008373-DS-3-rp2350-datasheet.pdf`).

The rule throughout: use what is easy. The RP2350 bootrom offers more (try before you buy, signed images, versions, a DMA CRC); none of it is needed here.

Status: design. Nothing of this is implemented yet.

## Why A/B

On RP2040 the app runs from RAM and overwrites itself in place: a lost link or a power loss in the middle leaves no app, and only BOOTSEL helps. The RP2350 bootrom does A/B by itself:
- **Address translation** (§5.1.19): the bootrom maps the booted partition to `0x10000000`. The same `.bin` runs from A or B, nothing is relinked.
- **Flash update boot** (§5.1.16): a reboot with `REBOOT_TYPE_FLASH_UPDATE` and the start of a partition boots that partition, whatever the versions say.
- **UF2 targeting** (§5.1.18): a UF2 dropped on the BOOTSEL drive goes into the partition that doesn't boot, followed by a flash update boot.

## Flash layout

| Offset | Size | |
|---|---|---|
| `0x000000` | 4 kB | partition table, slot 0 |
| `0x001000` | 4 kB | slot 1, unused, kept free |
| `0x002000` | 1008 kB | partition A |
| `0x0FE000` | 1008 kB | partition B |
| `0x1FA000` | 4 kB | free |
| `0x1FB000` | 16 kB | EEPROM, 4 sectors, unpartitioned, as now |
| `0x1FF000` | 4 kB | powerup counter, unpartitioned, as now |

The images are well below this: rx 147 kB, tx with WiFi and BT 590 kB. The offsets of A and B are constants in `rp-ota.h`, kept in sync with the table; nothing reads the table at runtime.

`mLRS/Common/hal/rp/rp2354_pt.json`, in the format of the SDK's `pico_cyw43_driver/wifi_pt.json`:

```json
{
  "version": [1, 0],
  "unpartitioned": {
    "families": ["absolute"],
    "permissions": { "secure": "rw", "nonsecure": "rw", "bootloader": "rw" }
  },
  "partitions": [
    {
      "name": "A", "id": 0, "start": "8K", "size": "1008K",
      "families": ["rp2350-arm-s"],
      "permissions": { "secure": "rw", "nonsecure": "rw", "bootloader": "rw" }
    },
    {
      "start": "1016K", "size": "1008K",
      "families": ["rp2350-arm-s"],
      "permissions": { "secure": "rw", "nonsecure": "rw", "bootloader": "rw" },
      "link": ["a", 0]
    }
  ]
}
```

The EEPROM and powerup counter stay in unpartitioned space at their current physical offsets, so `flash_range_erase()` / `flash_range_program()` don't change.

## Boot-time erase

The images carry no version, so both partitions look equally new to the bootrom, and the datasheet doesn't say which it then picks. So the running image makes itself the only one:

At the start of `setup()` on core 0, before core 1 is launched, `rom_get_boot_info()` gives the booted partition. If the first word of the other partition isn't erased, its first 4 kB sector is erased. That sector holds the image's block loop (the IMAGE_DEF is at `0x124` in the current tx build), so the bootrom no longer sees an image there.

- It happens once after each update, after that the first word reads `0xFFFFFFFF`.
- Core 1 isn't running yet, so only interrupts are turned off around the erase, as in `rp-eeprom.h`. All code runs from RAM.
- Without a partition table, `rom_get_boot_info()` reports no partition, and nothing is done.
- It covers all update paths alike: LoRa OTA, DroneCAN, and a UF2 dropped on the BOOTSEL drive.

Given up: a complete image that passes the CRC but doesn't run needs BOOTSEL/USB to recover, as on STM32 today.

## Writing an update: `rp-ota.h`

One backend for the LoRa OTA (`ota_rx.h`) and DroneCAN (`dronecan_firmware_update.h`), replacing today's in-place writer:

| Call | Does |
|---|---|
| `begin(length)` | takes the partition that didn't boot, checks that `length` fits, erases it sector by sector |
| `write(data, len)` | buffers into 4 kB, programs each full sector, `idleOtherCore()` around it, as now |
| `finish(crc)` | flushes, checks the CRC over the written length, then `rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_FLASH_UPDATE, 10, XIP_BASE + partition_start, 0)` |
| `abort()` | nothing to undo, the booted image is untouched |

- **CRC:** CRC-32/MPEG-2 in software, as the OTA protocol uses; over 150 kB it takes about 40 ms.
- **Reads go untranslated:** the written partition is read through `XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE` (`0x1c000000`) plus its physical offset. Through `XIP_BASE` it would be translated, or fault.
- **Raw image:** the receiver advertises the raw format, like STM32, and the partition size as its space. No compression.
- **DroneCAN:** the NOTE in `rp-ota.h` about lost CAN frames during the sector stall stays true, but it now costs a retry, not the app.

## Changes to existing code

- **`rp-eeprom.h:86`, `rp-powerup.h:70`:** both read via `XIP_BASE + offset`. Booted from a partition, an access beyond its size faults (§5.1.19), so on RP2350 they read via `XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE`.
- **`rp-glue.h`:** the boot-time erase in `setup()`.
- **`ota_rx.h`, `ota_relay_tx.h`:** the RP branches. `Run()` ends through `finish()`, the relay takes the `sx_dio_init_exti_isroff()` path. `SX_DIO` is already the pin name on RP.
- **`tools/run_rx_ota.py`, `tools/run_rx_ota_file.py`:** take the RP `.bin` as a raw image. The device name is found by the `tOtaAppInfo` magic, as now.
- **`common.h`:** `USE_RX_OTA` for RP receivers.

## Flashing

- **New board, and boards flashed before:** once, the partition table, then the firmware: `picotool partition create rp2354_pt.json pt.uf2`, load `pt.uf2`, then drop `firmware.uf2` onto the BOOTSEL drive. The current images sit at offset 0, over the table's slot, so boards flashed before need this too.
- **Later updates:** by UF2, DroneCAN or LoRa OTA. Each goes into the inactive partition.
- **SWD:** writes absolute addresses, so write the `.bin` to `0x10002000` (A), not the `.elf` to `0x10000000`, which is the table, and erase the first sector of B (`0x100FE000`), so that A is the only image.

## To check on the bench

1. **ATRANS after flash writes:** booted from B, do `flash_range_erase()` / `flash_range_program()` (`flash_exit_xip()` and back) keep the address translation? If not, the app faults after the first EEPROM write.
2. **Boot-time erase:** after an update, the old partition's first sector is erased, and power cycles keep booting the new image.
3. **UF2 targeting:** a UF2 dropped while A boots goes into B, and back.
4. **LoRa OTA and DroneCAN:** both end in the new image, with params kept.
