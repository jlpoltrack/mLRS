//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 LittleFS EEPROM Emulation
//*******************************************************
#ifndef RP_EEPROM_H
#define RP_EEPROM_H

#include <LittleFS.h>

typedef enum {
    EE_STATUS_FLASH_FAIL = 0,
    EE_STATUS_PAGE_UNDEF,
    EE_STATUS_PAGE_EMPTY,
    EE_STATUS_PAGE_FULL,
    EE_STATUS_OK
} EE_STATUS_ENUM;

inline EE_STATUS_ENUM ee_init(void) {
#if defined(ARDUINO_ARCH_RP2040) && defined(DBG_BOOT)
    DBG_BOOT("    ee_init: LittleFS.begin()...");
#endif
    if (!LittleFS.begin()) {
#if defined(ARDUINO_ARCH_RP2040) && defined(DBG_BOOT)
        DBG_BOOT("    ee_init: begin failed, formatting...");
#endif
        if (!LittleFS.format() || !LittleFS.begin()) return EE_STATUS_FLASH_FAIL;
    }
#if defined(ARDUINO_ARCH_RP2040) && defined(DBG_BOOT)
    DBG_BOOT("    ee_init: success");
#endif
    return EE_STATUS_OK;
}

inline EE_STATUS_ENUM ee_format(void) {
    if (LittleFS.format()) return EE_STATUS_OK;
    return EE_STATUS_FLASH_FAIL;
}

inline EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen) {
    File f = LittleFS.open("/config.bin", "r");
    if (!f) return EE_STATUS_PAGE_EMPTY;
    size_t read = f.read((uint8_t*)data, datalen);
    f.close();
    return (read == datalen) ? EE_STATUS_OK : EE_STATUS_FLASH_FAIL;
}

inline EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen) {
    File f = LittleFS.open("/config.bin", "w");
    if (!f) return EE_STATUS_FLASH_FAIL;
    size_t written = f.write((const uint8_t*)data, datalen);
    f.close();
    return (written == datalen) ? EE_STATUS_OK : EE_STATUS_FLASH_FAIL;
}

// Dummy functions for powerup.h compatibility
inline void ee_hal_unlock(void) {}
inline void ee_hal_lock(void) {}
inline void ee_hal_erasepage(uint32_t address, uint32_t page) { (void)address; (void)page; }
inline bool ee_hal_programword(uint32_t address, uint32_t data) { (void)address; (void)data; return true; }
inline bool ee_hal_programhalfword(uint32_t address, uint16_t data) { (void)address; (void)data; return true; }
inline bool ee_hal_programdoubleword(uint32_t address, uint64_t data) { (void)address; (void)data; return true; }

#endif // RP_EEPROM_H
