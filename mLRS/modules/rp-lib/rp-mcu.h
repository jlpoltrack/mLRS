//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP MCU utilities
//*******************************************************
#ifndef RP_MCU_H
#define RP_MCU_H

#include <pico/unique_id.h>

// same length as STM32_MCU_UID_LEN, callers needing more append it themselves
#define RP_MCU_UID_LEN  12

inline void mcu_uid(uint8_t uid[RP_MCU_UID_LEN])
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    // board id is only 8 bytes, so repeat it to fill all 12
    for (uint8_t i = 0; i < RP_MCU_UID_LEN; i++) uid[i] = id.id[i % PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
    uid[RP_MCU_UID_LEN - 1] ^= 0x5A; // ensure uid is never all FF, which means 'unknown'
}

inline uint32_t mcu_cpu_id(void)
{
    // rp2040/rp2350 cpuid register
    return *(volatile uint32_t*)0xE000ED00;
}

inline void BootLoaderInit(void) {}

#endif // RP_MCU_H
