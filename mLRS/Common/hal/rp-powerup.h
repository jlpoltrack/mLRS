//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP Powerup Counter
//*******************************************************
// Detects rapid power-cycles to enter bind mode (3 cycles within 2s).
//
// Uses raw flash via pico SDK to exploit NOR flash bit-flipping:
//   FF → AA → 00 transitions require no erase (only flips 1→0 bits).
// A dedicated 4KB sector holds 512 slots (8 bytes each).
// Sector erase needed only every ~512 power cycles.
// At 100K erase cycles per sector: ~51M power cycles before wear-out.
//
// Watchdog scratch registers distinguish power-cycle from soft reset:
//   - Power cycle: scratch cleared → counts as powerup
//   - Watchdog reboot: scratch preserved → skip counting
//
// Flash layout (end of flash, 5 sectors = 20KB):
//   PICO_FLASH_SIZE - 20KB : EEPROM sectors 0..3 (4 x 4KB, wear-leveled)
//   PICO_FLASH_SIZE -  4KB : powerup counter (4KB)
//*******************************************************
#ifndef RP_POWERUP_H
#define RP_POWERUP_H
#pragma once

#include <inttypes.h>
#include <string.h>
#include <hardware/flash.h>


extern volatile uint32_t millis32(void);


// powerup counter sector: last 4KB sector of flash (after EEPROM wear-leveled area)
#define POWERUPCNT_FLASH_OFFSET     (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

#define POWERUPCNT_FF               ((uint32_t)0xFFFFFFFF)
#define POWERUPCNT_AA               ((uint32_t)0xAAAAAAAA)

#define POWERUPCNT_TMO_MS           2000

// watchdog scratch register for reset-vs-powercycle detection
#define POWERUPCNT_SCRATCH_INDEX    7
#define POWERUPCNT_SCRATCH_SIG      ((uint32_t)0x4D4C5253)


typedef enum {
    POWERUPCNT_TASK_NONE = 0,
    POWERUPCNT_TASK_BIND,
} POWERUPCNT_TASK_ENUM;


class tPowerupCounter
{
  public:
    void Init(void);
    void Do(void);
    uint8_t Task(void);

  private:
    bool powerup_do;
    uint8_t task;
    uint16_t cur_slot;

    // read a slot via XIP memory map
    void flash_read_slot(uint16_t slot, uint32_t* v0, uint32_t* v1)
    {
        const uint32_t* p = (const uint32_t*)(XIP_BASE + POWERUPCNT_FLASH_OFFSET + slot * 8);
        *v0 = p[0];
        *v1 = p[1];
    }

    void flash_program_slot(uint16_t slot, uint32_t val0, uint32_t val1);
    void flash_erase_sector(void);
};


void tPowerupCounter::Init(void)
{
    powerup_do = true;
    task = POWERUPCNT_TASK_NONE;

    cur_slot = 0;

    // watchdog scratch registers survive watchdog reboot but are cleared on power cycle
    // if signature is present this is a soft reset, skip counting
    if (watchdog_hw->scratch[POWERUPCNT_SCRATCH_INDEX] == POWERUPCNT_SCRATCH_SIG) {
        powerup_do = false;
        return;
    }
    watchdog_hw->scratch[POWERUPCNT_SCRATCH_INDEX] = POWERUPCNT_SCRATCH_SIG;

    // search for current usable slot
    // usable: v0 in {FF,AA,00} and v1 in {FF,AA}
    // consumed (00,00): v1==00, skipped
    uint16_t num_slots = FLASH_SECTOR_SIZE / 8; // 512 for 4KB sector
    cur_slot = 0;
    bool found = false;

    for (uint16_t i = 0; i < num_slots; i++) {
        uint32_t v0, v1;
        flash_read_slot(i, &v0, &v1);
        if ((v0 == POWERUPCNT_FF || v0 == POWERUPCNT_AA || v0 == 0) &&
            (v1 == POWERUPCNT_FF || v1 == POWERUPCNT_AA)) {
            cur_slot = i;
            found = true;
            break;
        }
    }

    // no usable slot, all consumed, erase and start over
    if (!found) {
        flash_erase_sector();
        cur_slot = 0;
    }

    // count using state machine: FF FF -> AA FF -> AA AA -> 00 AA -> 00 00 (bind)
    uint32_t v0, v1;
    flash_read_slot(cur_slot, &v0, &v1);

    if (v0 == POWERUPCNT_FF && v1 == POWERUPCNT_FF) {
        flash_program_slot(cur_slot, POWERUPCNT_AA, POWERUPCNT_FF);
    } else
    if (v0 == POWERUPCNT_AA && v1 == POWERUPCNT_FF) {
        flash_program_slot(cur_slot, POWERUPCNT_AA, POWERUPCNT_AA);
    } else
    if (v0 == POWERUPCNT_AA && v1 == POWERUPCNT_AA) {
        flash_program_slot(cur_slot, 0, POWERUPCNT_AA);
    } else
    if (v0 == 0 && v1 == POWERUPCNT_AA) {
        task = POWERUPCNT_TASK_BIND;
        powerup_do = false;
        flash_program_slot(cur_slot, 0, 0);
    } else {
        // unexpected state, clear and move on
        powerup_do = false;
        flash_program_slot(cur_slot, 0, 0);
    }
}


void tPowerupCounter::Do(void)
{
    if (!powerup_do) return;

    uint32_t tnow_ms = millis32();
    if (tnow_ms < POWERUPCNT_TMO_MS) return;

    powerup_do = false;

    // no rapid power-cycle within timeout, reset counter
    flash_program_slot(cur_slot, 0, 0);
}


uint8_t tPowerupCounter::Task(void)
{
    switch (task) {
    case POWERUPCNT_TASK_BIND:
        task = POWERUPCNT_TASK_NONE;
        return POWERUPCNT_TASK_BIND;
    }

    return POWERUPCNT_TASK_NONE;
}


//-------------------------------------------------------
// Flash helpers
//-------------------------------------------------------

// program two uint32_t values into a slot
// pads a 256-byte page buffer with 0xFF so only the target bytes change
// (NOR flash only flips 1->0 on program, 0xFF = no change)
void tPowerupCounter::flash_program_slot(uint16_t slot, uint32_t val0, uint32_t val1)
{
    uint32_t byte_offset = slot * 8;
    uint32_t page_offset = byte_offset & ~(FLASH_PAGE_SIZE - 1);  // 256-byte aligned
    uint32_t offset_in_page = byte_offset & (FLASH_PAGE_SIZE - 1);

    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, FLASH_PAGE_SIZE);
    memcpy(&buf[offset_in_page], &val0, 4);
    memcpy(&buf[offset_in_page + 4], &val1, 4);

    rp2040.idleOtherCore();
    noInterrupts();
    flash_range_program(POWERUPCNT_FLASH_OFFSET + page_offset, buf, FLASH_PAGE_SIZE);
    interrupts();
    rp2040.resumeOtherCore();
}


void tPowerupCounter::flash_erase_sector(void)
{
    rp2040.idleOtherCore();
    noInterrupts();
    flash_range_erase(POWERUPCNT_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    interrupts();
    rp2040.resumeOtherCore();
}


#endif // RP_POWERUP_H
