//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// RP2040 HAL Splicer
//*******************************************************
#ifndef RP_HAL_H
#define RP_HAL_H

#if defined RX_GENERIC_900_RP2040
    #include "rx-hal-generic-900-rp2040.h"
#else
    #error No RP2040 device HAL defined!
#endif

#endif // RP_HAL_H
