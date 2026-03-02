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

#define RP_MCU_UID_LEN  8

inline void mcu_serial_number(uint8_t* sn)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(sn, id.id, RP_MCU_UID_LEN);
}

// for dronecan unique id (fills 16 bytes: 8 bytes uid + 8 bytes padded)
inline void mcu_uid(uint8_t uid[16])
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(uid, id.id, RP_MCU_UID_LEN);
    memset(uid + RP_MCU_UID_LEN, 0, 16 - RP_MCU_UID_LEN);
}

inline uint32_t mcu_cpu_id(void)
{
    // rp2040/rp2350 cpuid register
    return *(volatile uint32_t*)0xE000ED00;
}

inline void BootLoaderInit(void) {}

#endif // RP_MCU_H
