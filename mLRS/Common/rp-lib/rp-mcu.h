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

inline void mcu_serial_number(uint8_t* sn) {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(sn, id.id, 8);
}

inline void BootLoaderInit(void) { }

#endif // RP_MCU_H
