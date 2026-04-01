//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP OTA
// flash write abstraction for firmware update over CAN
// (DroneCAN) on RP2040/RP2350
//*******************************************************
// Since the copy-to-ram linker strategy runs all code
// from SRAM, flash can be safely erased and reprogrammed
// at runtime. No dual-partition scheme is needed.
//*******************************************************
#ifndef RPLIB_OTA_H
#define RPLIB_OTA_H

#include <hardware/flash.h>
#include <hardware/watchdog.h>


// FLASH_SECTOR_SIZE = 4096, FLASH_PAGE_SIZE = 256 (from pico sdk)
// firmware area is flash minus 20KB reserved at end:
// 4 x 4KB EEPROM wear-leveled sectors + 1 x 4KB powerup counter
#define RP_OTA_MAX_SIZE           (PICO_FLASH_SIZE_BYTES - 5 * FLASH_SECTOR_SIZE)


typedef enum {
    RP_OTA_STATE_IDLE = 0,
    RP_OTA_STATE_IN_PROGRESS,
    RP_OTA_STATE_DONE,
    RP_OTA_STATE_ERROR,
} rp_ota_state_e;


class tRpOta
{
  public:

    // prepare for writing firmware starting at flash offset 0
    // returns true on success
    bool ota_hal_begin(void)
    {
        if (_state == RP_OTA_STATE_IN_PROGRESS) return false;

        _write_offset = 0;
        _bytes_written = 0;
        _sector_buf_len = 0;
        _state = RP_OTA_STATE_IN_PROGRESS;
        return true;
    }


    // write a chunk of firmware data to flash
    // buffers a full 4KB sector before erasing+programming, so Core 0
    // is only stalled once per sector instead of once per 256-byte page.
    // returns true on success
    bool ota_hal_write_chunk(const uint8_t* data, uint16_t len)
    {
        if (_state != RP_OTA_STATE_IN_PROGRESS) return false;
        if (len == 0) return true;

        uint16_t remaining = len;
        const uint8_t* src = data;

        while (remaining > 0) {
            uint16_t space = FLASH_SECTOR_SIZE - _sector_buf_len;
            uint16_t to_copy = (remaining < space) ? remaining : space;
            memcpy(&_sector_buf[_sector_buf_len], src, to_copy);
            _sector_buf_len += to_copy;
            src += to_copy;
            remaining -= to_copy;

            if (_sector_buf_len == FLASH_SECTOR_SIZE) {
                if (!_erase_and_program_sector()) {
                    _state = RP_OTA_STATE_ERROR;
                    return false;
                }
                _sector_buf_len = 0;
            }
        }

        _bytes_written += len;
        return true;
    }


    // finalize the update: flush remaining data, reboot
    // does not return on success
    bool ota_hal_finish(void)
    {
        if (_state != RP_OTA_STATE_IN_PROGRESS) return false;

        // flush any remaining partial sector (pad with 0xFF)
        if (_sector_buf_len > 0) {
            memset(&_sector_buf[_sector_buf_len], 0xFF, FLASH_SECTOR_SIZE - _sector_buf_len);
            _sector_buf_len = FLASH_SECTOR_SIZE;
            if (!_erase_and_program_sector()) {
                _state = RP_OTA_STATE_ERROR;
                return false;
            }
        }

        _state = RP_OTA_STATE_DONE;

        // delay briefly so the CAN TX queue can drain any final status messages
        delay(100);

        watchdog_reboot(0, 0, 0); // does not return
        return true; // unreachable
    }


    // abort an in-progress OTA update and clean up
    void ota_hal_abort(void)
    {
        _write_offset = 0;
        _bytes_written = 0;
        _sector_buf_len = 0;
        _state = RP_OTA_STATE_IDLE;
    }


    uint32_t ota_hal_bytes_written(void) { return _bytes_written; }
    rp_ota_state_e ota_hal_state(void) { return _state; }

  private:
    rp_ota_state_e _state = RP_OTA_STATE_IDLE;
    uint32_t _write_offset = 0;    // next flash offset to program
    uint32_t _bytes_written = 0;
    uint8_t _sector_buf[FLASH_SECTOR_SIZE]; // 4KB sector buffer
    uint16_t _sector_buf_len = 0;


    // erase one sector and program it in a single Core 0 stall.
    // advances _write_offset by FLASH_SECTOR_SIZE.
    bool _erase_and_program_sector(void)
    {
        if (_write_offset + FLASH_SECTOR_SIZE > RP_OTA_MAX_SIZE) {
            return false; // would overwrite powerup counter or EEPROM area
        }

        rp2040.idleOtherCore();
        noInterrupts();
        flash_range_erase(_write_offset, FLASH_SECTOR_SIZE);
        flash_range_program(_write_offset, _sector_buf, FLASH_SECTOR_SIZE);
        interrupts();
        rp2040.resumeOtherCore();

        _write_offset += FLASH_SECTOR_SIZE;
        return true;
    }
};


#endif // RPLIB_OTA_H
