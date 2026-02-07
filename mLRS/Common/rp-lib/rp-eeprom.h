//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP EEPROM emulation
//*******************************************************
#ifndef RP_EEPROM_H
#define RP_EEPROM_H

#include <EEPROM.h>

typedef enum {
    EE_PAGE0 = 0,
    EE_PAGE1,
} EE_PAGE_ENUM;

typedef enum {
    EE_STATUS_FLASH_FAIL = 0,
    EE_STATUS_PAGE_UNDEF,
    EE_STATUS_PAGE_EMPTY,
    EE_STATUS_PAGE_FULL,
    EE_STATUS_OK
} EE_STATUS_ENUM;

// redundant pages for robustness, matching esp-eeprom.h logic
#ifndef EE_PAGE_SIZE
#define EE_PAGE_SIZE              0x0400 
#endif

#define EE_HEADER_SIZE            16

// EEPROM start address
#define EE_START_ADDRESS          ((uint32_t)(0x0000))

// pages 0 and 1 base addresses
#define EE_PAGE0_BASE_ADDRESS     ((uint32_t)(EE_START_ADDRESS + 0x0000))
#define EE_PAGE1_BASE_ADDRESS     ((uint32_t)(EE_START_ADDRESS + EE_PAGE_SIZE))

// page status definitions
#define EE_ERASE                  ((uint32_t)0xFFFFFFFF)
#define EE_VALID_PAGE             ((uint32_t)0x11111111)

#define EE_USE_WORD


//-------------------------------------------------------
// HAL functions for compatibility with powerup.h
//-------------------------------------------------------

inline void ee_hal_unlock(void) {}
inline void ee_hal_lock(void) {}

inline bool ee_hal_programword(uint32_t Address, uint32_t Data) {
    EEPROM.put(Address, Data);
    return true;
}

inline bool ee_hal_erasepage(uint32_t Page_Address, uint32_t Page_No) {
    (void)Page_No;
    ee_hal_programword(Page_Address, EE_ERASE);
    return true;
}

// stubs for halfword and doubleword for compilation compatibility
inline bool ee_hal_programhalfword(uint32_t address, uint16_t data) { (void)address; (void)data; return true; }
inline bool ee_hal_programdoubleword(uint32_t address, uint64_t data) { (void)address; (void)data; return true; }


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

// this function is only for internal use
// if data != NULL: write data to specified page
// if data == NULL: copy data from other page to specified page
inline EE_STATUS_ENUM _ee_write_to(uint16_t ToPage, void* data, uint16_t datalen) {
    EE_STATUS_ENUM status = EE_STATUS_OK;
    uint32_t ToPageBaseAddress, ToPageEndAddress, FromPageBaseAddress, adr;
    uint32_t PageNo;
    uint32_t val;
    uint16_t word_datalen;

    if (ToPage == EE_PAGE1) {
        PageNo = 1;
        ToPageBaseAddress = EE_PAGE1_BASE_ADDRESS;
        FromPageBaseAddress = EE_PAGE0_BASE_ADDRESS;
    } else {
        PageNo = 0;
        ToPageBaseAddress = EE_PAGE0_BASE_ADDRESS;
        FromPageBaseAddress = EE_PAGE1_BASE_ADDRESS;
    }
    ToPageEndAddress = ToPageBaseAddress + EE_PAGE_SIZE - 1;

    // erase ToPage
    if (!ee_hal_erasepage(ToPageBaseAddress, PageNo)) { status = EE_STATUS_FLASH_FAIL; goto QUICK_EXIT; }

    // write data to ToPage
    if (data == NULL) datalen = EE_PAGE_SIZE - EE_HEADER_SIZE; 

    word_datalen = (datalen + 3) / 4; 

    for (uint16_t n = 0; n < word_datalen; n++) {
        if (data == NULL) {
            adr = FromPageBaseAddress + EE_HEADER_SIZE + 4 * n;
            EEPROM.get(adr, val);
        } else {
            val = ((uint32_t*)data)[n];
        }
        if (val != (uint32_t)0xFFFFFFFF) {
            adr = ToPageBaseAddress + EE_HEADER_SIZE + 4 * n;
            if (adr >= ToPageEndAddress) { status = EE_STATUS_PAGE_FULL; goto QUICK_EXIT; }
            ee_hal_programword(adr, val);
        }
    }
    
    // set ToPage status to EE_VALID_PAGE
    ee_hal_programword(ToPageBaseAddress, EE_VALID_PAGE);
    
    // commit to flash
    EEPROM.commit();
    
    status = EE_STATUS_OK;
QUICK_EXIT:
    return status;
}


//-------------------------------------------------------
// API
//-------------------------------------------------------

inline EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen) {
    uint32_t adr;
    for (uint16_t n = 0; n < datalen; n++) {
        adr = EE_PAGE0_BASE_ADDRESS + EE_HEADER_SIZE + n;
        if (adr >= EE_PAGE0_BASE_ADDRESS + EE_PAGE_SIZE) return EE_STATUS_PAGE_FULL;
        ((uint8_t*)data)[n] = EEPROM.read(adr);
    }
    return EE_STATUS_OK;
}

inline EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen) {
    EE_STATUS_ENUM status;
    status = _ee_write_to(EE_PAGE0, data, datalen);
    if (status != EE_STATUS_OK) return status;
    status = _ee_write_to(EE_PAGE1, data, datalen);
    return status;
}

inline EE_STATUS_ENUM ee_format(void) {
    if (!ee_hal_erasepage(EE_PAGE0_BASE_ADDRESS, 0)) return EE_STATUS_FLASH_FAIL;
    if (!ee_hal_erasepage(EE_PAGE1_BASE_ADDRESS, 1)) return EE_STATUS_FLASH_FAIL;
    return EE_STATUS_OK;
}

inline EE_STATUS_ENUM ee_init(void) {
    EEPROM.begin(EE_PAGE_SIZE * 2);
    EE_STATUS_ENUM status;
    uint32_t Page0Status, Page1Status;

    EEPROM.get(EE_PAGE0_BASE_ADDRESS, Page0Status);
    EEPROM.get(EE_PAGE1_BASE_ADDRESS, Page1Status);

    if ((Page0Status == EE_VALID_PAGE) && (Page1Status == EE_VALID_PAGE)) {
        status = EE_STATUS_OK;
    } else if ((Page0Status == EE_VALID_PAGE) && (Page1Status != EE_VALID_PAGE)) {
        status = _ee_write_to(EE_PAGE1, NULL, 0);
    } else if ((Page0Status != EE_VALID_PAGE) && (Page1Status == EE_VALID_PAGE)) {
        status = _ee_write_to(EE_PAGE0, NULL, 0);
    } else {
        status = ee_format();
        if (status != EE_STATUS_OK) return status;
        status = (Page0Status == EE_ERASE && Page1Status == EE_ERASE) ? EE_STATUS_PAGE_EMPTY : EE_STATUS_PAGE_UNDEF;
    }
    return status;
}

#endif // RP_EEPROM_H