//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
#ifndef RP_HAL_H
#define RP_HAL_H

#if defined RX_RP_MLRS_DUAL
    #include "rx-hal-rp-mlrs-dual.h"
#endif

#if defined TX_DIY_LR2021_RP2350
    #include "tx-hal-diy-lr2021-rp.h"
#endif

#endif // RP_HAL_H
