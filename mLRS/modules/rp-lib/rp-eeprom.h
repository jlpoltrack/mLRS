//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP EEPROM emulation
//*******************************************************
// Wear-leveled settings storage using raw flash.
// 4 sectors rotated round-robin. Each write uses the next sector.
// One sector erase per write; wear spread 4x across sectors.
// At 100K erases per sector x 4 sectors = 400K total writes.
//
// Crash-safe: new sector is fully written before becoming active
// (highest sequence number wins). If power is lost mid-write,
// the previous sector remains valid.
//
// Flash layout (end of flash, 5 sectors = 20KB):
//   PICO_FLASH_SIZE - 20KB : EEPROM sector 0
//   ...
//   PICO_FLASH_SIZE -  8KB : EEPROM sector 3
//   PICO_FLASH_SIZE -  4KB : powerup counter
//*******************************************************
#ifndef RP_EEPROM_H
#define RP_EEPROM_H
#pragma once

#include <string.h>
#include <hardware/flash.h>


//-------------------------------------------------------
// Configuration
//-------------------------------------------------------

#define EE_NUM_SECTORS            4
#define EE_BASE_OFFSET            (PICO_FLASH_SIZE_BYTES - (EE_NUM_SECTORS + 1) * FLASH_SECTOR_SIZE)

#define EE_HEADER_SIZE            16
#define EE_HEADER_MAGIC           ((uint32_t)0x4D4C5253)

// keep for compilation guards in other headers
#define EE_USE_WORD


//-------------------------------------------------------
// Types
//-------------------------------------------------------

typedef enum {
    EE_STATUS_FLASH_FAIL = 0,
    EE_STATUS_PAGE_UNDEF,
    EE_STATUS_PAGE_EMPTY,
    EE_STATUS_PAGE_FULL,
    EE_STATUS_OK
} EE_STATUS_ENUM;

typedef struct {
    uint32_t magic;      // EE_HEADER_MAGIC when valid
    uint32_t sequence;   // incrementing, highest = active sector
    uint32_t datalen;    // bytes of settings data
    uint32_t checksum;   // sum of data bytes
} ee_header_t;


//-------------------------------------------------------
// Internal state
//-------------------------------------------------------

static uint8_t ee_active_sector;
static uint32_t ee_active_sequence;
static uint8_t ee_sector_buf[FLASH_SECTOR_SIZE]; // write buffer, avoids 4KB on stack


//-------------------------------------------------------
// Helpers
//-------------------------------------------------------

static inline uint32_t ee_sector_flash_offset(uint8_t sector)
{
    return EE_BASE_OFFSET + sector * FLASH_SECTOR_SIZE;
}


static inline const uint8_t* ee_sector_xip_ptr(uint8_t sector)
{
    return (const uint8_t*)(XIP_BASE + ee_sector_flash_offset(sector));
}


static uint32_t ee_checksum(const void* data, uint16_t len)
{
    const uint8_t* p = (const uint8_t*)data;
    uint32_t csum = 0;
    for (uint16_t i = 0; i < len; i++) csum += p[i];
    return csum;
}


static void ee_flash_erase_and_program(uint8_t sector)
{
    uint32_t offset = ee_sector_flash_offset(sector);

    rp2040.idleOtherCore();
    noInterrupts();
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
    flash_range_program(offset, ee_sector_buf, FLASH_SECTOR_SIZE);
    interrupts();
    rp2040.resumeOtherCore();
}


static void ee_flash_erase(uint8_t sector)
{
    uint32_t offset = ee_sector_flash_offset(sector);

    rp2040.idleOtherCore();
    noInterrupts();
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
    interrupts();
    rp2040.resumeOtherCore();
}


//-------------------------------------------------------
// API
//-------------------------------------------------------

inline EE_STATUS_ENUM ee_init(void)
{
    ee_active_sector = 0;
    ee_active_sequence = 0;
    bool found = false;

    // scan all sectors, find the one with valid header and highest sequence
    for (uint8_t i = 0; i < EE_NUM_SECTORS; i++) {
        const ee_header_t* hdr = (const ee_header_t*)ee_sector_xip_ptr(i);
        if (hdr->magic != EE_HEADER_MAGIC) continue;
        if (hdr->datalen > FLASH_SECTOR_SIZE - EE_HEADER_SIZE) continue;

        const uint8_t* data = ee_sector_xip_ptr(i) + EE_HEADER_SIZE;
        if (ee_checksum(data, hdr->datalen) != hdr->checksum) continue;

        if (!found || hdr->sequence > ee_active_sequence) {
            ee_active_sector = i;
            ee_active_sequence = hdr->sequence;
            found = true;
        }
    }

    if (found) return EE_STATUS_OK;

    // no valid sector found, check if flash is erased or corrupt
    const uint32_t* p = (const uint32_t*)ee_sector_xip_ptr(0);
    return (p[0] == 0xFFFFFFFF) ? EE_STATUS_PAGE_EMPTY : EE_STATUS_PAGE_UNDEF;
}


inline EE_STATUS_ENUM ee_readdata(void* data, uint16_t datalen)
{
    const ee_header_t* hdr = (const ee_header_t*)ee_sector_xip_ptr(ee_active_sector);
    if (hdr->magic != EE_HEADER_MAGIC) return EE_STATUS_PAGE_EMPTY;

    const uint8_t* src = ee_sector_xip_ptr(ee_active_sector) + EE_HEADER_SIZE;
    uint16_t copylen = (datalen < hdr->datalen) ? datalen : hdr->datalen;
    memcpy(data, src, copylen);

    return EE_STATUS_OK;
}


inline EE_STATUS_ENUM ee_writedata(void* data, uint16_t datalen)
{
    if (datalen > FLASH_SECTOR_SIZE - EE_HEADER_SIZE) return EE_STATUS_PAGE_FULL;

    uint8_t next = (ee_active_sector + 1) % EE_NUM_SECTORS;
    uint32_t next_seq = ee_active_sequence + 1;

    // build sector image: header + data + 0xFF padding
    memset(ee_sector_buf, 0xFF, FLASH_SECTOR_SIZE);

    ee_header_t hdr;
    hdr.magic = EE_HEADER_MAGIC;
    hdr.sequence = next_seq;
    hdr.datalen = datalen;
    hdr.checksum = ee_checksum(data, datalen);
    memcpy(ee_sector_buf, &hdr, sizeof(hdr));
    memcpy(&ee_sector_buf[EE_HEADER_SIZE], data, datalen);

    ee_flash_erase_and_program(next);

    ee_active_sector = next;
    ee_active_sequence = next_seq;

    return EE_STATUS_OK;
}


inline EE_STATUS_ENUM ee_format(void)
{
    for (uint8_t i = 0; i < EE_NUM_SECTORS; i++) {
        ee_flash_erase(i);
    }
    ee_active_sector = 0;
    ee_active_sequence = 0;
    return EE_STATUS_OK;
}


#endif // RP_EEPROM_H
