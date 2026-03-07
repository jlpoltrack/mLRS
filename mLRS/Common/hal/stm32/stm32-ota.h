//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// STM32 OTA
// dual-bank flash update for firmware update over CAN
// (DroneCAN). analogous to esp-ota.h for ESP32.
// 7.Mar.2026
//*******************************************************
#ifndef STM32_OTA_H
#define STM32_OTA_H
#pragma once

#include <string.h>
#include "stm32g4xx_hal.h"


// flash layout for G474CE in dual-bank mode:
//   bank 1: 0x08000000 - 0x0803FFFF (256KB, 128 pages x 2KB)
//   bank 2: 0x08040000 - 0x0807FFFF (256KB, 128 pages x 2KB)
// firmware occupies pages 0-119 (240KB) in each bank.
// EEPROM occupies pages 120-121 (4KB) in each bank.
// the BFB2 option byte swaps the banks so firmware always
// runs from 0x08000000.
// IMPORTANT: HAL_FLASH_Program uses the AHB bus address
// (affected by FB_MODE), but FLASH_PageErase uses the
// physical bank id (BKER bit, NOT affected by FB_MODE).
// we must select the erase bank based on the current BFB2
// state so that the erase and write target the same physical
// bank.

#define STM32_OTA_PAGE_SIZE           0x0800  // 2KB in dual-bank mode
#define STM32_OTA_FW_NUM_PAGES       120     // pages 0-119 for firmware
#define STM32_OTA_EE_NUM_PAGES       2       // pages 120-121 for EEPROM
#define STM32_OTA_EE_START_PAGE      120     // must match EE_START_PAGE
#define STM32_OTA_BANK2_OFFSET       0x40000 // 256KB offset to "other" bank


typedef enum {
    STM32_OTA_STATE_IDLE = 0,
    STM32_OTA_STATE_IN_PROGRESS,
    STM32_OTA_STATE_DONE,
    STM32_OTA_STATE_ERROR,
} stm32_ota_state_e;


class tStm32Ota
{
  public:

    // prepare the "other" bank for writing.
    // erases all firmware pages on the target bank.
    // returns true on success.
    bool ota_hal_begin(void)
    {
        if (_ota_state == STM32_OTA_STATE_IN_PROGRESS) return false;

        // dual-bank mode is required for bank-swap OTA
        if ((FLASH->OPTR & FLASH_OPTR_DBANK) == 0) return false;

        HAL_FLASH_Unlock();

        // erase firmware pages (0 to FW_NUM_PAGES-1) on the other bank.
        // FLASH_PageErase uses the physical bank id (BKER bit), which is
        // NOT affected by FB_MODE. we must pick the correct physical bank.
        FLASH_EraseInitTypeDef erase;
        uint32_t page_error = 0;

        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = _other_bank_id();
        erase.Page = 0;
        erase.NbPages = STM32_OTA_FW_NUM_PAGES;

        HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            _ota_state = STM32_OTA_STATE_ERROR;
            return false;
        }

        _ota_write_address = _other_bank_base();
        _ota_bytes_written = 0;
        _buf_len = 0;
        _ota_state = STM32_OTA_STATE_IN_PROGRESS;
        return true;
    }


    // write a chunk of firmware data to the target bank.
    // data is programmed as double-words (8 bytes). any trailing
    // bytes in a non-8-byte-aligned chunk are buffered until the
    // next call or until finish().
    // returns true on success.
    bool ota_hal_write_chunk(const uint8_t* data, uint16_t len)
    {
        if (_ota_state != STM32_OTA_STATE_IN_PROGRESS) return false;
        if (len == 0) return true;

        uint16_t i = 0;

        // if there are leftover bytes from a previous call, fill the buffer first
        while (_buf_len > 0 && _buf_len < 8 && i < len) {
            _buf[_buf_len++] = data[i++];
        }
        if (_buf_len == 8) {
            uint64_t dword;
            memcpy(&dword, _buf, 8);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _ota_write_address, dword) != HAL_OK) {
                _ota_state = STM32_OTA_STATE_ERROR;
                return false;
            }
            _ota_write_address += 8;
            _ota_bytes_written += 8;
            _buf_len = 0;
        }

        // program full double-words from the input
        while ((len - i) >= 8) {
            uint64_t dword;
            memcpy(&dword, &data[i], 8);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _ota_write_address, dword) != HAL_OK) {
                _ota_state = STM32_OTA_STATE_ERROR;
                return false;
            }
            _ota_write_address += 8;
            _ota_bytes_written += 8;
            i += 8;
        }

        // buffer any remaining bytes
        while (i < len) {
            _buf[_buf_len++] = data[i++];
        }

        return true;
    }


    // finalize the OTA update:
    //   1. flush any buffered bytes (pad with 0xFF)
    //   2. copy EEPROM from current bank to target bank
    //   3. toggle BFB2 option byte to swap boot bank
    //   4. reset — does not return on success
    bool ota_hal_finish(void)
    {
        if (_ota_state != STM32_OTA_STATE_IN_PROGRESS) return false;

        // flush remaining buffered bytes, pad to 8-byte alignment with 0xFF
        if (_buf_len > 0) {
            while (_buf_len < 8) _buf[_buf_len++] = 0xFF;
            uint64_t dword;
            memcpy(&dword, _buf, 8);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, _ota_write_address, dword) != HAL_OK) {
                _ota_state = STM32_OTA_STATE_ERROR;
                HAL_FLASH_Lock();
                return false;
            }
            _ota_bytes_written += 8;
            _buf_len = 0;
        }

        // copy EEPROM pages from current bank to target bank
        if (!_copy_eeprom()) {
            _ota_state = STM32_OTA_STATE_ERROR;
            HAL_FLASH_Lock();
            return false;
        }

        HAL_FLASH_Lock();
        _ota_state = STM32_OTA_STATE_DONE;

        // toggle BFB2 option byte to swap boot bank
        _swap_boot_bank();

        // does not return — _swap_boot_bank calls HAL_FLASH_OB_Launch
        // which triggers a system reset. if we somehow get here, fail.
        _ota_state = STM32_OTA_STATE_ERROR;
        return false;
    }


    // abort an in-progress OTA update and clean up
    void ota_hal_abort(void)
    {
        if (_ota_state == STM32_OTA_STATE_IN_PROGRESS) {
            HAL_FLASH_Lock();
        }
        _ota_write_address = 0;
        _ota_bytes_written = 0;
        _buf_len = 0;
        _ota_state = STM32_OTA_STATE_IDLE;
    }



  private:

    stm32_ota_state_e _ota_state;
    uint32_t _ota_write_address;
    uint32_t _ota_bytes_written;
    uint8_t _buf[8];      // alignment buffer for double-word writes
    uint8_t _buf_len;


    // base address of the "other" bank (the one not currently executing).
    // the AHB bus always maps the other bank at FLASH_BASE + BANK2_OFFSET
    // regardless of BFB2 state, so this is a fixed address.
    uint32_t _other_bank_base(void)
    {
        return FLASH_BASE + STM32_OTA_BANK2_OFFSET;
    }


    // physical bank id of the "other" bank for erase operations.
    // FLASH_PageErase uses BKER to select the physical bank, and BKER
    // is NOT affected by FB_MODE. when BFB2 is enabled (banks swapped),
    // we are running from physical bank 2, so the other is bank 1.
    uint32_t _other_bank_id(void)
    {
        FLASH_OBProgramInitTypeDef ob_cfg;
        memset(&ob_cfg, 0, sizeof(ob_cfg));
        HAL_FLASHEx_OBGetConfig(&ob_cfg);
        return (ob_cfg.USERConfig & OB_BFB2_ENABLE) ? FLASH_BANK_1 : FLASH_BANK_2;
    }


    // copy EEPROM pages from current bank to target bank.
    // reads EE_NUM_PAGES pages from current EEPROM location and
    // writes them to the same page offset on the other bank.
    // flash must be unlocked before calling.
    bool _copy_eeprom(void)
    {
        uint32_t src_base = FLASH_BASE + (STM32_OTA_EE_START_PAGE * STM32_OTA_PAGE_SIZE);
        uint32_t dst_base = src_base + STM32_OTA_BANK2_OFFSET;

        // erase EEPROM pages on the target bank
        FLASH_EraseInitTypeDef erase;
        uint32_t page_error = 0;

        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = _other_bank_id();
        erase.Page = STM32_OTA_EE_START_PAGE;
        erase.NbPages = STM32_OTA_EE_NUM_PAGES;

        if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
            return false;
        }

        // copy page data as double-words
        uint32_t total_bytes = STM32_OTA_EE_NUM_PAGES * STM32_OTA_PAGE_SIZE;
        for (uint32_t offset = 0; offset < total_bytes; offset += 8) {
            uint64_t dword = *(__IO uint64_t*)(src_base + offset);
            // skip erased double-words (all 0xFF) — already erased
            if (dword == 0xFFFFFFFFFFFFFFFF) continue;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst_base + offset, dword) != HAL_OK) {
                return false;
            }
        }

        return true;
    }


    // toggle the BFB2 option byte to swap the boot bank.
    // this triggers a system reset via HAL_FLASH_OB_Launch()
    // and does not return.
    void _swap_boot_bank(void)
    {
        HAL_FLASH_Unlock();

        // clear stale option byte verification error flag (per ST FLASH_DualBoot example).
        // a leftover OPTVERR can cause HAL_FLASHEx_OBProgram to silently fail.
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

        HAL_FLASH_OB_Unlock();

        FLASH_OBProgramInitTypeDef ob_cfg;
        memset(&ob_cfg, 0, sizeof(ob_cfg));
        HAL_FLASHEx_OBGetConfig(&ob_cfg);

        // toggle BFB2 bit
        ob_cfg.OptionType = OPTIONBYTE_USER;
        ob_cfg.USERType = OB_USER_BFB2;
        if (ob_cfg.USERConfig & OB_BFB2_ENABLE) {
            ob_cfg.USERConfig = OB_BFB2_DISABLE;
        } else {
            ob_cfg.USERConfig = OB_BFB2_ENABLE;
        }

        if (HAL_FLASHEx_OBProgram(&ob_cfg) != HAL_OK) {
            HAL_FLASH_OB_Lock();
            HAL_FLASH_Lock();
            _ota_state = STM32_OTA_STATE_ERROR;
            return;
        }

        // launch triggers reset — does not return
        HAL_FLASH_OB_Launch();
    }
};


#endif // STM32_OTA_H
